#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from sklearn.metrics import f1_score, precision_score, recall_score, roc_auc_score

from ml25d_dataset_generation.pso_training import _make_loader, _resolve_device
from ml25d_dataset_generation.risk_model import RiskModelConfig, build_model, require_torch
from ml25d_dataset_generation.risk_planner import evaluate_proxy_astar
from ml25d_dataset_generation.training_data import load_hdf5_dataset, make_stratified_split, normalize_x_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate fail decision threshold for a trained risk model")
    parser.add_argument("--checkpoint", type=Path, default=Path("data/training_runs/cnn_pso_v1/best_model.pt"))
    parser.add_argument("--pattern", default="data/generated_hq_v1/samples_batch_*.h5")
    parser.add_argument("--output", type=Path, default=Path("data/training_runs/cnn_pso_v1/calibration_report.json"))
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--target-recall", type=float, default=0.85)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    torch, _ = require_torch()
    device = _resolve_device(torch, args.device)

    checkpoint = torch.load(args.checkpoint, map_location=device)
    model_cfg_raw = dict(checkpoint["model_config"])
    model_cfg_raw["conv_channels"] = tuple(model_cfg_raw["conv_channels"])
    model = build_model(RiskModelConfig(**model_cfg_raw)).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    data = load_hdf5_dataset(args.pattern)
    splits = make_stratified_split(data.band, seed=args.seed)
    x_norm = normalize_x_map(data.x_map, checkpoint["channel_stats"])
    val_loader = _make_loader(torch, x_norm, data.param, data.y, splits.val, args.batch_size, shuffle=False, seed=args.seed)
    test_loader = _make_loader(torch, x_norm, data.param, data.y, splits.test, args.batch_size, shuffle=False, seed=args.seed)

    y_val_pred = _predict(torch, model, val_loader, device)
    y_test_pred = _predict(torch, model, test_loader, device)
    y_val = data.y[splits.val]
    y_test = data.y[splits.test]
    y_val_fail = y_val[:, 0].astype(int)
    y_test_fail = y_test[:, 0].astype(int)
    p_val_fail = y_val_pred[:, 0]
    p_test_fail = y_test_pred[:, 0]

    default_metrics = _threshold_metrics(y_test_fail, p_test_fail, 0.5)
    threshold_table = [_threshold_metrics(y_val_fail, p_val_fail, float(t)) for t in np.linspace(0.05, 0.95, 181)]
    feasible = [row for row in threshold_table if row["recall"] >= args.target_recall]
    if feasible:
        selected = max(feasible, key=lambda row: (row["threshold"], row["precision"]))
    else:
        selected = max(threshold_table, key=lambda row: (row["recall"], row["f2"], row["precision"]))
    max_f1 = max(threshold_table, key=lambda row: (row["f1"], row["precision"], row["recall"]))
    max_f2 = max(threshold_table, key=lambda row: (row["f2"], row["precision"], row["recall"]))
    selected_test = _threshold_metrics(y_test_fail, p_test_fail, selected["threshold"])
    max_f1_test = _threshold_metrics(y_test_fail, p_test_fail, max_f1["threshold"])
    max_f2_test = _threshold_metrics(y_test_fail, p_test_fail, max_f2["threshold"])

    planner = evaluate_proxy_astar(
        y_true=y_test,
        y_pred=y_test_pred,
        fusion_weights=checkpoint["fusion_weights"],
        thresholds=checkpoint["thresholds"],
        seed=args.seed + 77,
        num_tasks=40,
    )
    report = {
        "checkpoint": str(args.checkpoint),
        "device": str(device),
        "target_recall": args.target_recall,
        "calibration_split": "val",
        "evaluation_split": "test",
        "auc_fail_val": float(roc_auc_score(y_val_fail, p_val_fail)),
        "auc_fail_test": float(roc_auc_score(y_test_fail, p_test_fail)),
        "default_threshold_test": default_metrics,
        "selected_threshold_val": selected,
        "selected_threshold_test": selected_test,
        "selected_policy": "highest_threshold_meeting_target_recall" if feasible else "highest_recall_available",
        "max_f1_threshold_val": max_f1,
        "max_f1_threshold_test": max_f1_test,
        "max_f2_threshold_val": max_f2,
        "max_f2_threshold_test": max_f2_test,
        "planner": planner.__dict__,
        "threshold_table_val": threshold_table,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")
    print(
        json.dumps(
            {
                k: report[k]
                for k in [
                    "auc_fail_val",
                    "auc_fail_test",
                    "default_threshold_test",
                    "selected_threshold_val",
                    "selected_threshold_test",
                    "max_f1_threshold_val",
                    "max_f1_threshold_test",
                    "max_f2_threshold_val",
                    "max_f2_threshold_test",
                    "planner",
                ]
            },
            indent=2,
        )
    )
    return 0


def _predict(torch, model, loader, device) -> np.ndarray:
    preds = []
    with torch.no_grad():
        for x_map, param, _ in loader:
            logits = model(x_map.to(device), param.to(device))
            preds.append(torch.sigmoid(logits).cpu().numpy())
    return np.concatenate(preds, axis=0)


def _threshold_metrics(y_true: np.ndarray, p_fail: np.ndarray, threshold: float) -> dict[str, float]:
    pred = (p_fail >= threshold).astype(int)
    precision = precision_score(y_true, pred, zero_division=0)
    recall = recall_score(y_true, pred, zero_division=0)
    f1 = f1_score(y_true, pred, zero_division=0)
    beta2 = 2.0
    denom = beta2 * beta2 * precision + recall
    f2 = 0.0 if denom <= 1e-12 else (1.0 + beta2 * beta2) * precision * recall / denom
    return {
        "threshold": float(threshold),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "f2": float(f2),
        "pred_positive_rate": float(pred.mean()),
        "false_negative_count": int(((y_true == 1) & (pred == 0)).sum()),
        "false_positive_count": int(((y_true == 0) & (pred == 1)).sum()),
    }


if __name__ == "__main__":
    raise SystemExit(main())
