#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import h5py
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build pilot dataset statistics report")
    parser.add_argument("--dataset-dir", type=Path, required=True, help="Pilot dataset directory")
    parser.add_argument(
        "--output-prefix",
        type=str,
        default="pilot_stats_report",
        help="Output filename prefix under dataset dir",
    )
    parser.add_argument("--bottom-threshold", type=float, default=0.05, help="bottom fail threshold")
    parser.add_argument("--stuck-threshold", type=float, default=1.0, help="stuck fail threshold")
    return parser.parse_args()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _pct(arr: np.ndarray, p: float) -> float:
    if arr.size == 0:
        return float("nan")
    return float(np.percentile(arr, p))


def _fmt(x: Any) -> str:
    if isinstance(x, float):
        if np.isnan(x):
            return "NaN"
        return f"{x:.4f}"
    return str(x)


def _table(rows: list[dict[str, Any]], headers: list[str]) -> str:
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(lines)


def _heatmap(records: list[dict[str, Any]]) -> tuple[list[str], list[str], dict[tuple[str, str], float]]:
    vehicles = sorted({str(r["vehicle_id"]) for r in records})
    terrains = sorted({str(r["terrain_class"]) for r in records})
    stats: dict[tuple[str, str], float] = {}
    for v in vehicles:
        for t in terrains:
            vals = [float(r["y_fail"]) for r in records if r["vehicle_id"] == v and r["terrain_class"] == t]
            stats[(v, t)] = float(np.mean(vals)) if vals else float("nan")
    return vehicles, terrains, stats


def _progress_dist(records: list[dict[str, Any]]) -> dict[str, Any]:
    fail = np.array([float(r["y_fail"]) >= 0.5 for r in records], dtype=bool)

    translation_progress = np.array(
        [float(r.get("translation_progress", r.get("progress_ratio", float("nan")))) for r in records],
        dtype=np.float64,
    )
    angular_progress = np.array(
        [float(r.get("angular_progress", float("nan"))) for r in records],
        dtype=np.float64,
    )
    translation_drift = np.array(
        [float(r.get("translation_drift", float("nan"))) for r in records],
        dtype=np.float64,
    )

    bins = np.array([0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, np.inf], dtype=np.float64)

    def _group_stats(arr: np.ndarray, mask: np.ndarray) -> dict[str, float]:
        use = arr[mask]
        use = use[np.isfinite(use)]
        if use.size == 0:
            return {"count": 0, "mean": float("nan"), "p50": float("nan"), "p95": float("nan"), "min": float("nan"), "max": float("nan")}
        return {
            "count": int(use.size),
            "mean": float(np.mean(use)),
            "p50": float(np.percentile(use, 50)),
            "p95": float(np.percentile(use, 95)),
            "min": float(np.min(use)),
            "max": float(np.max(use)),
        }

    def _hist(arr: np.ndarray, mask: np.ndarray) -> list[dict[str, Any]]:
        use = arr[mask]
        use = use[np.isfinite(use)]
        if use.size == 0:
            return []
        counts = np.histogram(use, bins=bins)[0]
        rows: list[dict[str, Any]] = []
        for i, c in enumerate(counts):
            lo = bins[i]
            hi = bins[i + 1]
            hi_text = "inf" if np.isinf(hi) else f"{hi:.2f}"
            rows.append({"bin": f"[{lo:.2f}, {hi_text})", "count": int(c), "ratio": float(c / use.size)})
        return rows

    return {
        "translation_progress": {
            "pass": _group_stats(translation_progress, ~fail),
            "fail": _group_stats(translation_progress, fail),
            "pass_hist": _hist(translation_progress, ~fail),
            "fail_hist": _hist(translation_progress, fail),
        },
        "angular_progress": {
            "pass": _group_stats(angular_progress, ~fail),
            "fail": _group_stats(angular_progress, fail),
            "pass_hist": _hist(angular_progress, ~fail),
            "fail_hist": _hist(angular_progress, fail),
        },
        "translation_drift": {
            "pass": _group_stats(translation_drift, ~fail),
            "fail": _group_stats(translation_drift, fail),
        },
    }


def _h5_count(pattern: str) -> int:
    total = 0
    for path in sorted(glob.glob(pattern)):
        with h5py.File(path, "r") as h5f:
            total += int(h5f["y"].shape[0])
    return total


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset_dir.resolve()
    if not dataset_dir.exists():
        raise FileNotFoundError(f"dataset dir not found: {dataset_dir}")

    manifest_path = dataset_dir / "dataset_manifest.json"
    accepted_path = dataset_dir / "accepted_samples.jsonl"

    if not manifest_path.exists():
        raise FileNotFoundError(f"manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = _load_jsonl(accepted_path)
    if not records:
        raise RuntimeError(f"accepted records missing/empty: {accepted_path}")

    h5_total = _h5_count(str(dataset_dir / "samples_batch_*.h5"))
    valid_samples = len(records)
    total_target = int(manifest["dataset"].get("num_samples_target", valid_samples))
    invalid_attempts = int(manifest["dataset"].get("invalid_attempts", 0))
    invalid_rate = float(manifest["dataset"].get("invalid_sample_rate", invalid_attempts / max(valid_samples + invalid_attempts, 1)))

    y_fail = np.array([float(r["y_fail"]) for r in records], dtype=np.float64)
    q_roll = np.array([float(r["q_roll"]) for r in records], dtype=np.float64)
    q_pitch = np.array([float(r["q_pitch"]) for r in records], dtype=np.float64)
    q_slip = np.array([float(r["q_slip"]) for r in records], dtype=np.float64)
    q_lift = np.array([float(r["q_lift"]) for r in records], dtype=np.float64)
    p_bottom = np.array([float(r["p_bottom"]) for r in records], dtype=np.float64)
    p_stuck = np.array([float(r["p_stuck"]) for r in records], dtype=np.float64)

    total_fail_rate = float(np.mean(y_fail >= 0.5))

    by_vehicle = []
    for vehicle in sorted({str(r["vehicle_id"]) for r in records}):
        arr = np.array([float(r["y_fail"]) for r in records if r["vehicle_id"] == vehicle], dtype=np.float64)
        by_vehicle.append({"vehicle_type": vehicle, "count": int(arr.size), "fail_rate": float(np.mean(arr >= 0.5)) if arr.size else float("nan")})

    by_terrain = []
    for terrain in sorted({str(r["terrain_class"]) for r in records}):
        arr = np.array([float(r["y_fail"]) for r in records if r["terrain_class"] == terrain], dtype=np.float64)
        by_terrain.append({"terrain_type": terrain, "count": int(arr.size), "fail_rate": float(np.mean(arr >= 0.5)) if arr.size else float("nan")})

    fail_reason_counter: Counter[str] = Counter()
    fail_sample_count = 0
    for r in records:
        if float(r["y_fail"]) < 0.5:
            continue
        fail_sample_count += 1
        for reason in r.get("fail_reasons", []):
            fail_reason_counter[str(reason)] += 1

    q_stats = {
        "q_roll": {"mean": float(np.mean(q_roll)), "p50": _pct(q_roll, 50), "p95": _pct(q_roll, 95)},
        "q_pitch": {"mean": float(np.mean(q_pitch)), "p50": _pct(q_pitch, 50), "p95": _pct(q_pitch, 95)},
        "q_slip": {"mean": float(np.mean(q_slip)), "p50": _pct(q_slip, 50), "p95": _pct(q_slip, 95)},
        "q_lift": {"mean": float(np.mean(q_lift)), "p50": _pct(q_lift, 50), "p95": _pct(q_lift, 95)},
    }

    bottom_flag = p_bottom > float(args.bottom_threshold)
    stuck_flag = p_stuck >= float(args.stuck_threshold)
    bottom_stuck = {
        "bottom_fail_rate": float(np.mean(bottom_flag)),
        "stuck_fail_rate": float(np.mean(stuck_flag)),
        "p_bottom": {"mean": float(np.mean(p_bottom)), "p50": _pct(p_bottom, 50), "p95": _pct(p_bottom, 95)},
        "p_stuck": {"mean": float(np.mean(p_stuck)), "p50": _pct(p_stuck, 50), "p95": _pct(p_stuck, 95)},
        "joint_distribution": {
            "bottom0_stuck0": int(np.sum((~bottom_flag) & (~stuck_flag))),
            "bottom1_stuck0": int(np.sum(bottom_flag & (~stuck_flag))),
            "bottom0_stuck1": int(np.sum((~bottom_flag) & stuck_flag)),
            "bottom1_stuck1": int(np.sum(bottom_flag & stuck_flag)),
        },
    }

    vehicles, terrains, heat = _heatmap(records)
    heat_rows: list[dict[str, Any]] = []
    for v in vehicles:
        row = {"vehicle_type": v}
        for t in terrains:
            row[t] = heat[(v, t)]
        heat_rows.append(row)

    progress = _progress_dist(records)

    report = {
        "dataset_dir": str(dataset_dir),
        "counts": {
            "total_samples_target": total_target,
            "valid_samples": valid_samples,
            "hdf5_samples": h5_total,
            "invalid_attempts": invalid_attempts,
            "invalid_sample_rate": invalid_rate,
        },
        "total_fail_rate": total_fail_rate,
        "by_vehicle_fail_rate": by_vehicle,
        "by_terrain_fail_rate": by_terrain,
        "fail_reason_distribution": {
            "fail_sample_count": fail_sample_count,
            "counts": dict(sorted(fail_reason_counter.items())),
            "per_fail_sample": {k: float(v / max(fail_sample_count, 1)) for k, v in sorted(fail_reason_counter.items())},
        },
        "q_metrics": q_stats,
        "bottom_stuck_distribution": bottom_stuck,
        "risk_heatmap_fail_rate": {
            "vehicles": vehicles,
            "terrains": terrains,
            "matrix": {f"{v}::{t}": heat[(v, t)] for v in vehicles for t in terrains},
        },
        "progress_distribution": progress,
        "manifest_invalid_reason_distribution": manifest.get("counts", {}).get("invalid_reason", {}),
    }

    json_path = dataset_dir / f"{args.output_prefix}.json"
    md_path = dataset_dir / f"{args.output_prefix}.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    sections: list[str] = []
    sections.append("## Pilot Dataset Statistics")
    sections.append("")
    sections.append(
        _table(
            [
                {
                    "total_samples": total_target,
                    "valid_samples": valid_samples,
                    "hdf5_samples": h5_total,
                    "invalid_attempts": invalid_attempts,
                    "invalid_sample_rate": invalid_rate,
                    "total_fail_rate": total_fail_rate,
                }
            ],
            ["total_samples", "valid_samples", "hdf5_samples", "invalid_attempts", "invalid_sample_rate", "total_fail_rate"],
        )
    )
    sections.append("")
    sections.append("### By Vehicle Fail Rate")
    sections.append(_table(by_vehicle, ["vehicle_type", "count", "fail_rate"]))
    sections.append("")
    sections.append("### By Terrain Fail Rate")
    sections.append(_table(by_terrain, ["terrain_type", "count", "fail_rate"]))
    sections.append("")
    sections.append("### Fail Reason Distribution")
    reason_rows = []
    for k, v in sorted(fail_reason_counter.items()):
        reason_rows.append({"fail_reason": k, "count": v, "per_fail_sample": v / max(fail_sample_count, 1)})
    sections.append(_table(reason_rows, ["fail_reason", "count", "per_fail_sample"]))
    sections.append("")
    sections.append("### q Metrics")
    q_rows = []
    for k, v in q_stats.items():
        q_rows.append({"metric": k, "mean": v["mean"], "p50": v["p50"], "p95": v["p95"]})
    sections.append(_table(q_rows, ["metric", "mean", "p50", "p95"]))
    sections.append("")
    sections.append("### Bottom / Stuck Distribution")
    sections.append(
        _table(
            [
                {
                    "bottom_fail_rate": bottom_stuck["bottom_fail_rate"],
                    "stuck_fail_rate": bottom_stuck["stuck_fail_rate"],
                    "p_bottom_mean": bottom_stuck["p_bottom"]["mean"],
                    "p_bottom_p50": bottom_stuck["p_bottom"]["p50"],
                    "p_bottom_p95": bottom_stuck["p_bottom"]["p95"],
                    "p_stuck_mean": bottom_stuck["p_stuck"]["mean"],
                    "p_stuck_p50": bottom_stuck["p_stuck"]["p50"],
                    "p_stuck_p95": bottom_stuck["p_stuck"]["p95"],
                }
            ],
            [
                "bottom_fail_rate",
                "stuck_fail_rate",
                "p_bottom_mean",
                "p_bottom_p50",
                "p_bottom_p95",
                "p_stuck_mean",
                "p_stuck_p50",
                "p_stuck_p95",
            ],
        )
    )
    sections.append("")
    sections.append("### Risk Heatmap (Fail Rate)")
    sections.append(_table(heat_rows, ["vehicle_type"] + terrains))
    sections.append("")
    sections.append("### Translation Progress Distribution (Pass/Fail)")
    t_prog_rows = []
    for grp in ["pass", "fail"]:
        s = progress["translation_progress"][grp]
        t_prog_rows.append(
            {
                "group": grp,
                "count": s["count"],
                "mean": s["mean"],
                "p50": s["p50"],
                "p95": s["p95"],
                "min": s["min"],
                "max": s["max"],
            }
        )
    sections.append(_table(t_prog_rows, ["group", "count", "mean", "p50", "p95", "min", "max"]))
    sections.append("")
    sections.append("### Translation Progress Histogram (Pass)")
    sections.append(_table(progress["translation_progress"]["pass_hist"], ["bin", "count", "ratio"]))
    sections.append("")
    sections.append("### Translation Progress Histogram (Fail)")
    sections.append(_table(progress["translation_progress"]["fail_hist"], ["bin", "count", "ratio"]))
    sections.append("")
    sections.append("### Angular Progress Distribution (Pass/Fail)")
    a_prog_rows = []
    for grp in ["pass", "fail"]:
        s = progress["angular_progress"][grp]
        a_prog_rows.append(
            {
                "group": grp,
                "count": s["count"],
                "mean": s["mean"],
                "p50": s["p50"],
                "p95": s["p95"],
                "min": s["min"],
                "max": s["max"],
            }
        )
    sections.append(_table(a_prog_rows, ["group", "count", "mean", "p50", "p95", "min", "max"]))
    sections.append("")
    sections.append("### Angular Progress Histogram (Pass)")
    sections.append(_table(progress["angular_progress"]["pass_hist"], ["bin", "count", "ratio"]))
    sections.append("")
    sections.append("### Angular Progress Histogram (Fail)")
    sections.append(_table(progress["angular_progress"]["fail_hist"], ["bin", "count", "ratio"]))
    sections.append("")
    sections.append("### Translation Drift Distribution (Pass/Fail)")
    drift_rows = []
    for grp in ["pass", "fail"]:
        s = progress["translation_drift"][grp]
        drift_rows.append(
            {
                "group": grp,
                "count": s["count"],
                "mean": s["mean"],
                "p50": s["p50"],
                "p95": s["p95"],
                "min": s["min"],
                "max": s["max"],
            }
        )
    sections.append(_table(drift_rows, ["group", "count", "mean", "p50", "p95", "min", "max"]))
    sections.append("")
    sections.append("### Invalid Reasons (from manifest)")
    invalid_reason_rows = [
        {"invalid_reason": k, "count": v}
        for k, v in sorted(manifest.get("counts", {}).get("invalid_reason", {}).items())
    ]
    sections.append(_table(invalid_reason_rows, ["invalid_reason", "count"]))
    sections.append("")
    sections.append(f"json_path: `{json_path}`")
    sections.append(f"md_path: `{md_path}`")

    md_path.write_text("\n".join(sections) + "\n", encoding="utf-8")
    print(json.dumps({"json": str(json_path), "md": str(md_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
