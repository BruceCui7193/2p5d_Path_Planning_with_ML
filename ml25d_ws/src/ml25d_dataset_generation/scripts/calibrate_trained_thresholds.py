#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.risk_model import RiskModelConfig, build_model, require_torch
from ml25d_dataset_generation.risk_planner import evaluate_proxy_astar
from ml25d_dataset_generation.training_data import load_hdf5_dataset, make_stratified_split, normalize_x_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate planning thresholds for a trained risk model")
    parser.add_argument(
        "--pattern",
        default="data/generated_dataset_v1_20k_n6_comp/shard_*/samples_batch_*.h5",
        help="Glob pattern for HDF5 batches",
    )
    parser.add_argument("--model", type=Path, required=True, help="Path to trained model .pt")
    parser.add_argument("--report", type=Path, required=True, help="Path to training_report.json")
    parser.add_argument("--output-report", type=Path, required=True)
    parser.add_argument("--output-model", type=Path, default=Path(""))
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--trials", type=int, default=240)
    parser.add_argument("--val-tasks", type=int, default=80)
    parser.add_argument("--test-tasks", type=int, default=120)
    parser.add_argument("--eval-seeds", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=512)
    return parser.parse_args()


def _predict(torch, model, x_norm: np.ndarray, param: np.ndarray, idx: np.ndarray, batch_size: int) -> np.ndarray:
    preds: list[np.ndarray] = []
    model.eval()
    with torch.no_grad():
        for start in range(0, idx.shape[0], batch_size):
            batch = idx[start : start + batch_size]
            x = torch.from_numpy(np.transpose(x_norm[batch], (0, 3, 1, 2)).astype(np.float32))
            p = torch.from_numpy(param[batch].astype(np.float32))
            preds.append(torch.sigmoid(model(x, p)).cpu().numpy())
    return np.concatenate(preds, axis=0)


def _stable_planner(
    *,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    fusion_weights: list[float],
    thresholds: dict[str, float],
    base_seed: int,
    eval_seeds: int,
    num_tasks: int,
) -> dict[str, float]:
    plan_sr = 0.0
    oracle_sr = 0.0
    len_ratio = 0.0
    solved = 0
    total = 0
    for i in range(max(eval_seeds, 1)):
        res = evaluate_proxy_astar(
            y_true=y_true,
            y_pred=y_pred,
            fusion_weights=fusion_weights,
            thresholds=thresholds,
            seed=int(base_seed + 7919 * i),
            num_tasks=max(int(num_tasks), 4),
        )
        plan_sr += float(res.plan_success_rate)
        oracle_sr += float(res.oracle_safe_rate)
        len_ratio += float(res.mean_length_ratio)
        solved += int(res.solved_tasks)
        total += int(res.total_tasks)
    denom = float(max(eval_seeds, 1))
    return {
        "plan_success_rate": plan_sr / denom,
        "oracle_safe_rate": oracle_sr / denom,
        "mean_length_ratio": len_ratio / denom,
        "solved_tasks": solved,
        "total_tasks": total,
    }


def _score(planner: dict[str, float]) -> float:
    return float(
        planner["plan_success_rate"]
        + 0.8 * planner["oracle_safe_rate"]
        - 0.2 * max(planner["mean_length_ratio"] - 1.0, 0.0)
    )


def _clip_thr(edge: float, path_max: float, path_avg: float) -> dict[str, float]:
    return {
        "edge": float(np.clip(edge, 0.4, 0.9)),
        "path_max": float(np.clip(path_max, 0.4, 0.9)),
        "path_avg": float(np.clip(path_avg, 0.1, 0.5)),
    }


def main() -> int:
    args = parse_args()
    torch, _ = require_torch()

    data = load_hdf5_dataset(args.pattern)
    splits = make_stratified_split(data.band, seed=int(args.seed))
    report = json.loads(args.report.read_text(encoding="utf-8"))
    ckpt = torch.load(args.model, map_location="cpu")

    model_cfg = RiskModelConfig(**ckpt["model_config"])
    model = build_model(model_cfg)
    model.load_state_dict(ckpt["state_dict"])

    x_norm = normalize_x_map(data.x_map, ckpt["channel_stats"])
    val_idx = splits.val
    test_idx = splits.test
    y_val_true = data.y[val_idx]
    y_test_true = data.y[test_idx]
    y_val_pred = _predict(torch, model, x_norm, data.param, val_idx, batch_size=max(int(args.batch_size), 32))
    y_test_pred = _predict(torch, model, x_norm, data.param, test_idx, batch_size=max(int(args.batch_size), 32))

    fusion = list(ckpt["fusion_weights"])
    base_thr = {
        "edge": float(ckpt["thresholds"]["edge"]),
        "path_max": float(ckpt["thresholds"]["path_max"]),
        "path_avg": float(ckpt["thresholds"]["path_avg"]),
    }

    base_val = _stable_planner(
        y_true=y_val_true,
        y_pred=y_val_pred,
        fusion_weights=fusion,
        thresholds=base_thr,
        base_seed=args.seed + 11,
        eval_seeds=max(int(args.eval_seeds), 1),
        num_tasks=max(int(args.val_tasks), 4),
    )
    base_test = _stable_planner(
        y_true=y_test_true,
        y_pred=y_test_pred,
        fusion_weights=fusion,
        thresholds=base_thr,
        base_seed=args.seed + 29,
        eval_seeds=max(int(args.eval_seeds), 1),
        num_tasks=max(int(args.test_tasks), 4),
    )

    rng = np.random.default_rng(int(args.seed) + 1337)
    candidates: list[dict[str, float]] = [
        base_thr,
        {"edge": 0.75, "path_max": 0.85, "path_avg": 0.45},
    ]
    for _ in range(max(int(args.trials) - len(candidates), 0)):
        if rng.random() < 0.25:
            candidates.append(
                {
                    "edge": float(rng.uniform(0.4, 0.9)),
                    "path_max": float(rng.uniform(0.4, 0.9)),
                    "path_avg": float(rng.uniform(0.1, 0.5)),
                }
            )
            continue
        candidates.append(
            _clip_thr(
                edge=base_thr["edge"] + float(rng.normal(0.0, 0.12)),
                path_max=base_thr["path_max"] + float(rng.normal(0.0, 0.10)),
                path_avg=base_thr["path_avg"] + float(rng.normal(0.0, 0.08)),
            )
        )

    best_thr = base_thr
    best_val = base_val
    best_score = _score(base_val)
    for i, thr in enumerate(candidates):
        val = _stable_planner(
            y_true=y_val_true,
            y_pred=y_val_pred,
            fusion_weights=fusion,
            thresholds=thr,
            base_seed=args.seed + 3000 + i * 17,
            eval_seeds=max(int(args.eval_seeds), 1),
            num_tasks=max(int(args.val_tasks), 4),
        )
        s = _score(val)
        if s > best_score:
            best_score = s
            best_thr = thr
            best_val = val

    best_test = _stable_planner(
        y_true=y_test_true,
        y_pred=y_test_pred,
        fusion_weights=fusion,
        thresholds=best_thr,
        base_seed=args.seed + 5000,
        eval_seeds=max(int(args.eval_seeds), 1),
        num_tasks=max(int(args.test_tasks), 4),
    )

    out: dict[str, Any] = {
        "model_path": str(args.model),
        "report_path": str(args.report),
        "dataset_pattern": str(args.pattern),
        "seed": int(args.seed),
        "candidates": int(len(candidates)),
        "base_thresholds": base_thr,
        "best_thresholds": best_thr,
        "base_val": base_val,
        "best_val": best_val,
        "base_test": base_test,
        "best_test": best_test,
        "base_score_val": _score(base_val),
        "best_score_val": best_score,
        "base_score_test": _score(base_test),
        "best_score_test": _score(best_test),
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_text(json.dumps(out, indent=2, ensure_ascii=True), encoding="utf-8")

    if str(args.output_model):
        ckpt["thresholds"] = best_thr
        args.output_model.parent.mkdir(parents=True, exist_ok=True)
        torch.save(ckpt, args.output_model)

    print(json.dumps(out, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
