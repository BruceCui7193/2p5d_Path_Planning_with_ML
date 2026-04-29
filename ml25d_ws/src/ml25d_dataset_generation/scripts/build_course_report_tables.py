#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build course report tables from training + planning outputs")
    parser.add_argument("--planning-csv", type=Path, required=True)
    parser.add_argument("--training-report-pso", type=Path, required=True)
    parser.add_argument("--training-report-baseline", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def _load_csv(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            out = dict(row)
            for key in ["found", "path_length_m", "risk_max", "risk_avg", "expanded_nodes", "planning_time_ms"]:
                if key in out:
                    out[key] = float(out[key])
            rows.append(out)
    return rows


def _load_training_metrics(path: Path) -> dict[str, float]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    tm = obj.get("final", {}).get("test_metrics", {})
    planner = tm.get("planner", {})
    return {
        "auc_fail": float(tm.get("auc_fail", float("nan"))),
        "recall_fail": float(tm.get("recall_fail", float("nan"))),
        "f1_fail": float(tm.get("f1_fail", float("nan"))),
        "mae_risk": float(tm.get("mae_risk", float("nan"))),
        "infer_ms_per_sample": float(tm.get("infer_ms_per_sample", float("nan"))),
        "proxy_plan_success_rate": float(planner.get("plan_success_rate", float("nan"))),
        "proxy_oracle_safe_rate": float(planner.get("oracle_safe_rate", float("nan"))),
        "proxy_mean_length_ratio": float(planner.get("mean_length_ratio", float("nan"))),
    }


def _group_planning(rows: list[dict[str, Any]], keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    for row in rows:
        k = tuple(str(row[key]) for key in keys)
        groups.setdefault(k, []).append(row)
    out: list[dict[str, Any]] = []
    for key_tuple, items in sorted(groups.items()):
        found_mask = np.asarray([float(r["found"]) >= 0.5 for r in items], dtype=bool)
        path_len = np.asarray([float(r["path_length_m"]) for r in items], dtype=np.float64)
        risk_max = np.asarray([float(r["risk_max"]) for r in items], dtype=np.float64)
        risk_avg = np.asarray([float(r["risk_avg"]) for r in items], dtype=np.float64)
        expanded = np.asarray([float(r["expanded_nodes"]) for r in items], dtype=np.float64)
        time_ms = np.asarray([float(r["planning_time_ms"]) for r in items], dtype=np.float64)

        found_len = path_len[found_mask & np.isfinite(path_len)]
        found_rmax = risk_max[found_mask & np.isfinite(risk_max)]
        found_ravg = risk_avg[found_mask & np.isfinite(risk_avg)]
        found_expanded = expanded[found_mask & np.isfinite(expanded)]
        found_time = time_ms[found_mask & np.isfinite(time_ms)]

        row_out: dict[str, Any] = {k: v for k, v in zip(keys, key_tuple)}
        row_out.update(
            {
                "count": int(len(items)),
                "success_rate": float(np.mean(found_mask)),
                "path_length_mean": float(np.mean(found_len)) if found_len.size > 0 else float("nan"),
                "risk_max_mean": float(np.mean(found_rmax)) if found_rmax.size > 0 else float("nan"),
                "risk_avg_mean": float(np.mean(found_ravg)) if found_ravg.size > 0 else float("nan"),
                "expanded_nodes_mean": float(np.mean(found_expanded)) if found_expanded.size > 0 else float("nan"),
                "planning_time_ms_mean": float(np.mean(found_time)) if found_time.size > 0 else float("nan"),
            }
        )
        out.append(row_out)
    return out


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row.keys():
            if key in seen:
                continue
            seen.add(key)
            fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _fmt(v: float) -> str:
    if not np.isfinite(v):
        return "NaN"
    return f"{v:.4f}"


def main() -> int:
    args = parse_args()
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    planning_rows = _load_csv(args.planning_csv.resolve())
    pso_metrics = _load_training_metrics(args.training_report_pso.resolve())
    baseline_metrics = _load_training_metrics(args.training_report_baseline.resolve()) if args.training_report_baseline else None

    model_table = [{"model_tag": "pso", **pso_metrics}]
    if baseline_metrics is not None:
        model_table.append({"model_tag": "baseline", **baseline_metrics})
    _write_csv(out_dir / "model_metrics_table.csv", model_table)

    planning_method = _group_planning(planning_rows, ["method", "model_tag"])
    _write_csv(out_dir / "planning_metrics_method_table.csv", planning_method)

    planning_vehicle_terrain = _group_planning(planning_rows, ["vehicle_id", "terrain_class", "method", "model_tag"])
    _write_csv(out_dir / "planning_metrics_vehicle_terrain_table.csv", planning_vehicle_terrain)

    compare_rows: list[dict[str, Any]] = []
    if baseline_metrics is not None:
        row = {"compare": "pso_minus_baseline"}
        for key in [
            "auc_fail",
            "recall_fail",
            "f1_fail",
            "mae_risk",
            "infer_ms_per_sample",
            "proxy_plan_success_rate",
            "proxy_oracle_safe_rate",
            "proxy_mean_length_ratio",
        ]:
            row[f"delta_{key}"] = float(pso_metrics.get(key, float("nan")) - baseline_metrics.get(key, float("nan")))
        compare_rows.append(row)
    # Planning-level PSO compare: proposed method only.
    proposed_main = [
        r for r in planning_method if str(r.get("method")) == "proposed_ml_risk_constrained_astar" and str(r.get("model_tag")) == "main"
    ]
    proposed_compare = [
        r for r in planning_method if str(r.get("method")) == "proposed_ml_risk_constrained_astar" and str(r.get("model_tag")) == "compare"
    ]
    if proposed_main and proposed_compare:
        pm = proposed_main[0]
        pc = proposed_compare[0]
        compare_rows.append(
            {
                "compare": "planning_proposed_main_minus_compare",
                "delta_success_rate": float(pm["success_rate"] - pc["success_rate"]),
                "delta_path_length_mean": float(pm["path_length_mean"] - pc["path_length_mean"]),
                "delta_risk_max_mean": float(pm["risk_max_mean"] - pc["risk_max_mean"]),
                "delta_risk_avg_mean": float(pm["risk_avg_mean"] - pc["risk_avg_mean"]),
                "delta_expanded_nodes_mean": float(pm["expanded_nodes_mean"] - pc["expanded_nodes_mean"]),
                "delta_planning_time_ms_mean": float(pm["planning_time_ms_mean"] - pc["planning_time_ms_mean"]),
            }
        )
    _write_csv(out_dir / "pso_compare_table.csv", compare_rows)

    md = []
    md.append("## 课程报告关键表格汇总")
    md.append("")
    md.append("### 模型指标")
    md.append("| model_tag | auc_fail | recall_fail | f1_fail | mae_risk | infer_ms | proxy_plan_sr |")
    md.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in model_table:
        md.append(
            "| "
            + f"{row['model_tag']} | {_fmt(row['auc_fail'])} | {_fmt(row['recall_fail'])} | {_fmt(row['f1_fail'])} | "
            + f"{_fmt(row['mae_risk'])} | {_fmt(row['infer_ms_per_sample'])} | {_fmt(row['proxy_plan_success_rate'])} |"
        )
    md.append("")
    md.append("### 规划指标（方法汇总）")
    md.append("| method | model_tag | success_rate | path_length_mean | risk_max_mean | risk_avg_mean | expanded_mean | time_ms_mean |")
    md.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in planning_method:
        md.append(
            "| "
            + f"{row['method']} | {row['model_tag']} | {_fmt(row['success_rate'])} | {_fmt(row['path_length_mean'])} | "
            + f"{_fmt(row['risk_max_mean'])} | {_fmt(row['risk_avg_mean'])} | {_fmt(row['expanded_nodes_mean'])} | {_fmt(row['planning_time_ms_mean'])} |"
        )
    if compare_rows:
        md.append("")
        md.append("### PSO 对比")
        md.append("`pso_compare_table.csv` 已生成。")
    (out_dir / "course_report_tables.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "output_dir": str(out_dir),
                "model_metrics_table": str(out_dir / "model_metrics_table.csv"),
                "planning_metrics_method_table": str(out_dir / "planning_metrics_method_table.csv"),
                "planning_metrics_vehicle_terrain_table": str(out_dir / "planning_metrics_vehicle_terrain_table.csv"),
                "pso_compare_table": str(out_dir / "pso_compare_table.csv"),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
