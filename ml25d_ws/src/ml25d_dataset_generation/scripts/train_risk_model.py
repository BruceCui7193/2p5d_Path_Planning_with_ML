#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.pso_training import TrainConfig, run_baseline_training, run_pso_training
from ml25d_dataset_generation.training_data import RiskDatasetArrays, compute_channel_stats, load_hdf5_dataset, make_stratified_split


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train CNN+MLP risk model with PSO hyperparameter search")
    parser.add_argument(
        "--pattern",
        default="data/generated_dataset_v1_20k_n6_comp/shard_*/samples_batch_*.h5",
        help="Glob pattern of HDF5 dataset batches",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("data/training_runs/cnn_pso_dataset_v1"))
    parser.add_argument("--mode", choices=["pso", "baseline", "both"], default="both")
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--device", default="auto", help="auto, cuda, cuda:0, or cpu")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--loader-workers", type=int, default=4)
    parser.add_argument("--pso-particles", type=int, default=8)
    parser.add_argument("--pso-iters", type=int, default=6)
    parser.add_argument("--pso-epochs", type=int, default=6)
    parser.add_argument("--final-epochs", type=int, default=90, help="Maximum epochs for final training")
    parser.add_argument("--final-min-epochs", type=int, default=25)
    parser.add_argument("--final-patience", type=int, default=12)
    parser.add_argument("--final-min-delta", type=float, default=5e-4)
    parser.add_argument("--final-lr-patience", type=int, default=4)
    parser.add_argument("--final-lr-factor", type=float, default=0.5)
    parser.add_argument("--max-pso-train-samples", type=int, default=4096)
    parser.add_argument("--proxy-tasks-pso", type=int, default=32)
    parser.add_argument("--proxy-tasks-final", type=int, default=80)
    parser.add_argument("--proxy-eval-seeds-pso", type=int, default=2)
    parser.add_argument("--proxy-eval-seeds-final", type=int, default=4)
    parser.add_argument("--disable-threshold-calibration", action="store_true")
    parser.add_argument("--threshold-calibration-trials", type=int, default=48)
    parser.add_argument("--threshold-calibration-tasks", type=int, default=40)
    parser.add_argument("--threshold-calibration-seeds", type=int, default=3)
    parser.add_argument("--filter-actions", default="a0,a1,a2", help="Comma-separated action ids to keep for training")
    parser.add_argument("--max-samples", type=int, default=0, help="Optional cap after filtering (0=all)")
    parser.add_argument("--flat-fail-warn-threshold", type=float, default=0.05)
    parser.add_argument("--quiet", action="store_true", help="Disable progress logging")
    return parser.parse_args()


def _subset_dataset(data: RiskDatasetArrays, indices: np.ndarray) -> RiskDatasetArrays:
    idx = np.asarray(indices, dtype=np.int64)
    return RiskDatasetArrays(
        x_map=data.x_map[idx],
        theta_v=data.theta_v[idx],
        action=data.action[idx],
        mu=data.mu[idx],
        y=data.y[idx],
        band=data.band[idx],
        metadata=[data.metadata[int(i)] for i in idx.tolist()],
    )


def _filter_by_actions(data: RiskDatasetArrays, action_ids: set[str]) -> RiskDatasetArrays:
    if not action_ids:
        return data
    keep = []
    for i, meta in enumerate(data.metadata):
        action_id = str(meta.get("action_id", ""))
        if not action_id or action_id in action_ids:
            keep.append(i)
    if not keep:
        raise RuntimeError(f"no samples left after action filter: {sorted(action_ids)}")
    return _subset_dataset(data, np.asarray(keep, dtype=np.int64))


def _cap_samples_stratified(data: RiskDatasetArrays, max_samples: int, seed: int) -> RiskDatasetArrays:
    if max_samples <= 0 or data.y.shape[0] <= max_samples:
        return data
    rng = np.random.default_rng(seed)
    selected_parts: list[np.ndarray] = []
    for band in np.unique(data.band):
        idx = np.flatnonzero(data.band == band)
        target = int(round(max_samples * idx.shape[0] / data.y.shape[0]))
        target = max(1, min(target, idx.shape[0]))
        chosen = rng.choice(idx, size=target, replace=False)
        selected_parts.append(chosen)
    selected = np.sort(np.concatenate(selected_parts))
    if selected.shape[0] > max_samples:
        selected = np.sort(rng.choice(selected, size=max_samples, replace=False))
    return _subset_dataset(data, selected)


def _summarize_dataset(data: RiskDatasetArrays, flat_fail_warn: float) -> dict[str, Any]:
    terrain_counts: dict[str, int] = {}
    vehicle_counts: dict[str, int] = {}
    action_counts: dict[str, int] = {}
    for meta in data.metadata:
        terrain = str(meta.get("terrain_class", "unknown"))
        vehicle = str(meta.get("vehicle_id", "unknown"))
        action = str(meta.get("action_id", "unknown"))
        terrain_counts[terrain] = terrain_counts.get(terrain, 0) + 1
        vehicle_counts[vehicle] = vehicle_counts.get(vehicle, 0) + 1
        action_counts[action] = action_counts.get(action, 0) + 1

    flat_idx = np.array(
        [i for i, meta in enumerate(data.metadata) if str(meta.get("terrain_class", "")) == "flat"],
        dtype=np.int64,
    )
    flat_fail_rate = float(np.mean(data.y[flat_idx, 0] >= 0.5)) if flat_idx.shape[0] > 0 else float("nan")
    summary = {
        "num_samples": int(data.y.shape[0]),
        "band_counts": {k: int(v) for k, v in zip(*np.unique(data.band, return_counts=True))},
        "terrain_counts": dict(sorted(terrain_counts.items())),
        "vehicle_counts": dict(sorted(vehicle_counts.items())),
        "action_counts": dict(sorted(action_counts.items())),
        "flat_count": int(flat_idx.shape[0]),
        "flat_fail_rate": flat_fail_rate,
        "flat_fail_warn_threshold": float(flat_fail_warn),
    }
    return summary


def _extract_key_metrics(report: dict[str, Any]) -> dict[str, float]:
    m = report["final"]["test_metrics"]
    planner = m.get("planner", {})
    return {
        "auc_fail": float(m.get("auc_fail", float("nan"))),
        "recall_fail": float(m.get("recall_fail", float("nan"))),
        "f1_fail": float(m.get("f1_fail", float("nan"))),
        "mae_risk": float(m.get("mae_risk", float("nan"))),
        "infer_ms_per_sample": float(m.get("infer_ms_per_sample", float("nan"))),
        "plan_success_rate": float(planner.get("plan_success_rate", float("nan"))),
        "oracle_safe_rate": float(planner.get("oracle_safe_rate", float("nan"))),
        "mean_length_ratio": float(planner.get("mean_length_ratio", float("nan"))),
    }


def main() -> int:
    args = parse_args()
    data = load_hdf5_dataset(args.pattern)
    action_filter = {part.strip() for part in str(args.filter_actions).split(",") if part.strip()}
    data = _filter_by_actions(data, action_filter)
    data = _cap_samples_stratified(data, int(args.max_samples), seed=args.seed)
    dataset_summary = _summarize_dataset(data, flat_fail_warn=float(args.flat_fail_warn_threshold))
    if (not args.quiet) and dataset_summary["flat_count"] > 0:
        warn = ""
        if dataset_summary["flat_fail_rate"] > float(args.flat_fail_warn_threshold):
            warn = " [WARN flat fail high]"
        print(
            "[train] dataset "
            f"samples={dataset_summary['num_samples']} flat_fail_rate={dataset_summary['flat_fail_rate']:.4f}"
            f"{warn}"
        )
        print(
            json.dumps(
                {
                    "band_counts": dataset_summary["band_counts"],
                    "vehicle_counts": dataset_summary["vehicle_counts"],
                    "terrain_counts_top10": dict(list(dataset_summary["terrain_counts"].items())[:10]),
                    "action_counts": dataset_summary["action_counts"],
                },
                ensure_ascii=True,
            )
        )

    splits = make_stratified_split(data.band, seed=args.seed)
    channel_stats = compute_channel_stats(data.x_map, splits.train)

    cfg = TrainConfig(
        batch_size=args.batch_size,
        pso_epochs=args.pso_epochs,
        final_epochs=args.final_epochs,
        pso_particles=args.pso_particles,
        pso_iters=args.pso_iters,
        seed=args.seed,
        device=args.device,
        max_pso_train_samples=args.max_pso_train_samples,
        loader_num_workers=max(int(args.loader_workers), 0),
        proxy_tasks_pso=max(int(args.proxy_tasks_pso), 4),
        proxy_tasks_final=max(int(args.proxy_tasks_final), 4),
        proxy_eval_seeds_pso=max(int(args.proxy_eval_seeds_pso), 1),
        proxy_eval_seeds_final=max(int(args.proxy_eval_seeds_final), 1),
        final_min_epochs=max(int(args.final_min_epochs), 1),
        final_patience=max(int(args.final_patience), 1),
        final_min_delta=max(float(args.final_min_delta), 0.0),
        final_lr_patience=max(int(args.final_lr_patience), 1),
        final_lr_factor=float(np.clip(args.final_lr_factor, 0.1, 0.95)),
        calibrate_thresholds=not bool(args.disable_threshold_calibration),
        threshold_calibration_trials=max(int(args.threshold_calibration_trials), 1),
        threshold_calibration_tasks=max(int(args.threshold_calibration_tasks), 4),
        threshold_calibration_eval_seeds=max(int(args.threshold_calibration_seeds), 1),
        verbose=not args.quiet,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)

    outputs: dict[str, Any] = {"dataset_summary": dataset_summary, "mode": args.mode}
    if args.mode in {"baseline", "both"}:
        baseline_dir = args.output_dir / ("baseline" if args.mode == "both" else ".")
        baseline_dir = baseline_dir if args.mode == "both" else args.output_dir
        baseline_report = run_baseline_training(
            data=data,
            splits=splits,
            output_dir=baseline_dir,
            cfg=cfg,
            channel_stats=channel_stats,
        )
        outputs["baseline"] = {
            "report_path": str(baseline_dir / "training_report.json"),
            "key_metrics": _extract_key_metrics(baseline_report),
        }

    if args.mode in {"pso", "both"}:
        pso_dir = args.output_dir / ("pso" if args.mode == "both" else ".")
        pso_dir = pso_dir if args.mode == "both" else args.output_dir
        pso_report = run_pso_training(
            data=data,
            splits=splits,
            output_dir=pso_dir,
            cfg=cfg,
            channel_stats=channel_stats,
        )
        outputs["pso"] = {
            "report_path": str(pso_dir / "training_report.json"),
            "key_metrics": _extract_key_metrics(pso_report),
        }

    if args.mode == "both":
        compare = {
            "baseline": outputs["baseline"]["key_metrics"],
            "pso": outputs["pso"]["key_metrics"],
        }
        outputs["compare"] = compare
        (args.output_dir / "compare_report.json").write_text(
            json.dumps(outputs, indent=2, ensure_ascii=True),
            encoding="utf-8",
        )

    print(json.dumps(outputs, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
