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
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run flat-fail and progress-outlier audits on pilot dataset")
    parser.add_argument("--dataset-dir", type=Path, required=True, help="Dataset directory")
    parser.add_argument(
        "--action-config",
        type=Path,
        default=Path("src/ml25d_dataset_generation/config/action_primitives.yaml"),
        help="Action primitives yaml",
    )
    parser.add_argument("--flat-terrain-name", type=str, default="flat")
    parser.add_argument("--progress-threshold", type=float, default=3.0)
    parser.add_argument("--output-prefix", type=str, default="preformal_audits")
    return parser.parse_args()


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


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            out.append(json.loads(line))
    return out


def _load_metadata(dataset_dir: Path) -> dict[int, dict[str, Any]]:
    meta: dict[int, dict[str, Any]] = {}
    for h5_path in sorted(glob.glob(str(dataset_dir / "samples_batch_*.h5"))):
        with h5py.File(h5_path, "r") as h5f:
            meta_json = h5f["metadata_json"][:]
            for item in meta_json:
                text = item.decode("utf-8") if isinstance(item, bytes) else str(item)
                m = json.loads(text)
                sid = int(m["sample_id"])
                meta[sid] = m
    return meta


def _load_actions(path: Path) -> dict[str, dict[str, Any]]:
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    actions = cfg["actions"]["primitives"]
    out = {}
    for a in actions:
        out[str(a["id"])] = a
    return out


def _mu_bin(mu: float) -> str:
    edges = [0.0, 0.4, 0.6, 0.8, 1.0, 1.2, 10.0]
    labels = ["[0.0,0.4)", "[0.4,0.6)", "[0.6,0.8)", "[0.8,1.0)", "[1.0,1.2)", "[1.2,+inf)"]
    for i in range(len(edges) - 1):
        if edges[i] <= mu < edges[i + 1]:
            return labels[i]
    return labels[-1]


def _flat_fail_audit(
    records: list[dict[str, Any]],
    metadata_by_id: dict[int, dict[str, Any]],
    flat_terrain_name: str,
) -> dict[str, Any]:
    rows = []
    for r in records:
        sid = int(r["sample_id"])
        meta = metadata_by_id.get(sid, {})
        rr = dict(r)
        rr["friction_mu"] = float(meta.get("friction_mu", np.nan))
        rr["mu_bin"] = _mu_bin(rr["friction_mu"]) if not np.isnan(rr["friction_mu"]) else "unknown"
        rows.append(rr)

    flat_rows = [r for r in rows if str(r["terrain_class"]) == flat_terrain_name]
    flat_fail = [r for r in flat_rows if float(r["y_fail"]) >= 0.5]
    flat_fail_rate = float(len(flat_fail) / max(len(flat_rows), 1))

    fail_reasons = Counter()
    for r in flat_fail:
        fail_reasons.update([str(x) for x in r.get("fail_reasons", [])])

    by_vehicle = []
    for vehicle in sorted({str(r["vehicle_id"]) for r in flat_rows}):
        sub = [r for r in flat_rows if r["vehicle_id"] == vehicle]
        n = len(sub)
        f = sum(float(x["y_fail"]) >= 0.5 for x in sub)
        by_vehicle.append({"vehicle_type": vehicle, "count": n, "fail_count": f, "fail_rate": f / max(n, 1)})

    by_action = []
    for action in sorted({str(r["action_id"]) for r in flat_rows}):
        sub = [r for r in flat_rows if r["action_id"] == action]
        n = len(sub)
        f = sum(float(x["y_fail"]) >= 0.5 for x in sub)
        by_action.append({"action_id": action, "count": n, "fail_count": f, "fail_rate": f / max(n, 1)})

    by_friction_class = []
    for fr in sorted({str(r["friction_class"]) for r in flat_rows}):
        sub = [r for r in flat_rows if r["friction_class"] == fr]
        n = len(sub)
        f = sum(float(x["y_fail"]) >= 0.5 for x in sub)
        by_friction_class.append({"friction_class": fr, "count": n, "fail_count": f, "fail_rate": f / max(n, 1)})

    by_mu_bin = []
    for mb in ["[0.0,0.4)", "[0.4,0.6)", "[0.6,0.8)", "[0.8,1.0)", "[1.0,1.2)", "[1.2,+inf)", "unknown"]:
        sub = [r for r in flat_rows if r["mu_bin"] == mb]
        if not sub:
            continue
        n = len(sub)
        f = sum(float(x["y_fail"]) >= 0.5 for x in sub)
        by_mu_bin.append({"mu_bin": mb, "count": n, "fail_count": f, "fail_rate": f / max(n, 1)})

    fail_detail = []
    for r in sorted(flat_fail, key=lambda x: int(x["sample_id"])):
        fail_detail.append(
            {
                "sample_id": int(r["sample_id"]),
                "vehicle_type": str(r["vehicle_id"]),
                "action_id": str(r["action_id"]),
                "friction_class": str(r["friction_class"]),
                "friction_mu": float(r["friction_mu"]),
                "motion_model": str(r["motion_model"]),
                "q_roll": float(r["q_roll"]),
                "q_pitch": float(r["q_pitch"]),
                "q_slip": float(r["q_slip"]),
                "q_lift": float(r["q_lift"]),
                "p_bottom": float(r["p_bottom"]),
                "p_stuck": float(r["p_stuck"]),
                "progress_ratio": float(r["progress_ratio"]),
                "translation_progress": float(r.get("translation_progress", np.nan)),
                "angular_progress": float(r.get("angular_progress", np.nan)),
                "translation_drift": float(r.get("translation_drift", np.nan)),
                "fail_reasons": list(r.get("fail_reasons", [])),
            }
        )

    major_reasons = set(fail_reasons.keys())
    if major_reasons.issubset({"slip", "stuck"}):
        assessment = "flat_fail 主要由 slip/stuck 驱动，可接受（低摩擦或动作导致）。"
        assessment_flag = "acceptable_if_low_mu"
    else:
        assessment = (
            "flat_fail 含 roll/pitch/bottom/lift 主因，说明平地仍有结构性非 slip 失败，需要继续排查。"
        )
        assessment_flag = "needs_investigation"

    return {
        "flat_sample_count": len(flat_rows),
        "flat_fail_count": len(flat_fail),
        "flat_fail_rate": flat_fail_rate,
        "fail_reason_distribution": dict(sorted(fail_reasons.items())),
        "by_vehicle": by_vehicle,
        "by_action": by_action,
        "by_friction_class": by_friction_class,
        "by_friction_mu_bin": by_mu_bin,
        "flat_fail_details": fail_detail,
        "assessment": assessment,
        "assessment_flag": assessment_flag,
    }


def _progress_outlier_audit(
    records: list[dict[str, Any]],
    metadata_by_id: dict[int, dict[str, Any]],
    actions: dict[str, dict[str, Any]],
    progress_threshold: float,
    action_duration_sec: float = 2.0,
) -> dict[str, Any]:
    outliers: list[dict[str, Any]] = []
    for r in records:
        sid = int(r["sample_id"])
        meta = metadata_by_id.get(sid, {})
        action_id = str(r["action_id"])
        action = actions[action_id]
        delta_s = float(action["delta_s_m"])
        delta_psi = float(action["delta_psi_deg"])
        if delta_s > 1e-6:
            progress_metric = "translation_progress"
            progress = float(r.get("translation_progress", r.get("progress_ratio", np.nan)))
        else:
            progress_metric = "angular_progress"
            progress = float(r.get("angular_progress", r.get("progress_ratio", np.nan)))
        if (not np.isfinite(progress)) or progress <= progress_threshold:
            continue
        cmd_v = float(action["ackermann_cmd"]["v_cmd_mps"]) if str(r["motion_model"]) == "ackermann" else float(
            action["skid_cmd"]["v_cmd_mps"]
        )
        drift = float(r.get("translation_drift", np.nan))
        est_disp = float(progress * delta_s) if delta_s > 1e-6 else drift
        est_speed = float(est_disp / action_duration_sec) if delta_s > 1e-6 else float("nan")
        est_heading_deg = float(progress * abs(delta_psi)) if abs(delta_psi) > 1e-6 else 0.0

        reasons = [str(x) for x in r.get("fail_reasons", [])]
        terrain = str(r["terrain_class"])
        heuristic = "likely_valid_outlier"
        if delta_s <= 1e-6 and progress > progress_threshold:
            heuristic = "turn_progress_definition_bug"
        elif terrain in {"uniform_slope", "slope_bumps"} and float(r["q_slip"]) > 0.5:
            heuristic = "downhill_or_low_mu_sliding"
        elif np.isnan(est_disp) is False and est_disp > 2.0:
            heuristic = "possible_abnormal_long_travel"
        elif "bottom" in reasons and progress > progress_threshold:
            heuristic = "possible_collision_rebound_or_bounce"

        outliers.append(
            {
                "sample_id": sid,
                "terrain_type": terrain,
                "vehicle_type": str(r["vehicle_id"]),
                "action_id": action_id,
                "motion_model": str(r["motion_model"]),
                "friction_class": str(r["friction_class"]),
                "friction_mu": float(meta.get("friction_mu", np.nan)),
                "progress_metric": progress_metric,
                "progress_value": progress,
                "estimated_displacement_m": est_disp,
                "estimated_mean_speed_mps": est_speed,
                "commanded_speed_mps": cmd_v,
                "estimated_heading_change_deg": est_heading_deg,
                "translation_drift": drift,
                "y_fail": float(r["y_fail"]),
                "fail_reasons": reasons,
                "q_roll": float(r["q_roll"]),
                "q_pitch": float(r["q_pitch"]),
                "q_slip": float(r["q_slip"]),
                "q_lift": float(r["q_lift"]),
                "p_bottom": float(r["p_bottom"]),
                "p_stuck": float(r["p_stuck"]),
                "heuristic_judgement": heuristic,
            }
        )

    outliers = sorted(outliers, key=lambda x: float(x["progress_value"]), reverse=True)
    total = len(records)
    ratio = float(len(outliers) / max(total, 1))
    by_heuristic = Counter([str(r["heuristic_judgement"]) for r in outliers])
    by_terrain = Counter([str(r["terrain_type"]) for r in outliers])
    by_vehicle = Counter([str(r["vehicle_type"]) for r in outliers])
    by_action = Counter([str(r["action_id"]) for r in outliers])
    by_fail_reason = Counter()
    for r in outliers:
        by_fail_reason.update(r["fail_reasons"])

    if by_heuristic["possible_abnormal_long_travel"] > 0 or by_heuristic["possible_collision_rebound_or_bounce"] > 0:
        conclusion = "存在疑似异常滑走/弹飞样本，建议进入 invalid 或收紧 progress 定义。"
        conclusion_flag = "needs_investigation"
    elif by_heuristic["turn_progress_definition_bug"] > 0:
        conclusion = "仍存在转向动作 progress 定义异常，需要修复动作语义。"
        conclusion_flag = "needs_investigation"
    else:
        conclusion = "progress 超阈值主要来自平移动作，转向动作已不再因定义错误触发离群。"
        conclusion_flag = "acceptable"

    return {
        "progress_threshold": progress_threshold,
        "sample_count": total,
        "outlier_count": len(outliers),
        "outlier_ratio": ratio,
        "by_heuristic": dict(sorted(by_heuristic.items())),
        "by_terrain": dict(sorted(by_terrain.items())),
        "by_vehicle": dict(sorted(by_vehicle.items())),
        "by_action": dict(sorted(by_action.items())),
        "fail_reason_distribution": dict(sorted(by_fail_reason.items())),
        "outlier_details": outliers,
        "conclusion": conclusion,
        "conclusion_flag": conclusion_flag,
    }


def _write_markdown(
    *,
    path: Path,
    flat_audit: dict[str, Any],
    outlier_audit: dict[str, Any],
) -> None:
    sections: list[str] = []
    sections.append("## Flat Fail Audit")
    sections.append(
        _table(
            [
                {
                    "flat_sample_count": flat_audit["flat_sample_count"],
                    "flat_fail_count": flat_audit["flat_fail_count"],
                    "flat_fail_rate": flat_audit["flat_fail_rate"],
                    "assessment_flag": flat_audit["assessment_flag"],
                }
            ],
            ["flat_sample_count", "flat_fail_count", "flat_fail_rate", "assessment_flag"],
        )
    )
    sections.append("")
    sections.append(f"assessment: {flat_audit['assessment']}")
    sections.append("")
    sections.append("### Flat Fail Reason Distribution")
    reason_rows = [{"fail_reason": k, "count": v} for k, v in flat_audit["fail_reason_distribution"].items()]
    sections.append(_table(reason_rows, ["fail_reason", "count"]))
    sections.append("")
    sections.append("### Flat By Vehicle")
    sections.append(_table(flat_audit["by_vehicle"], ["vehicle_type", "count", "fail_count", "fail_rate"]))
    sections.append("")
    sections.append("### Flat By Action")
    sections.append(_table(flat_audit["by_action"], ["action_id", "count", "fail_count", "fail_rate"]))
    sections.append("")
    sections.append("### Flat By Friction Class")
    sections.append(_table(flat_audit["by_friction_class"], ["friction_class", "count", "fail_count", "fail_rate"]))
    sections.append("")
    sections.append("### Flat By Friction Mu Bin")
    sections.append(_table(flat_audit["by_friction_mu_bin"], ["mu_bin", "count", "fail_count", "fail_rate"]))
    sections.append("")
    sections.append("### Flat Fail Details")
    sections.append(
        _table(
            flat_audit["flat_fail_details"],
            [
                "sample_id",
                "vehicle_type",
                "action_id",
                "friction_class",
                "friction_mu",
                "q_roll",
                "q_pitch",
                "q_slip",
                "q_lift",
                "p_bottom",
                "p_stuck",
                "progress_ratio",
                "translation_progress",
                "angular_progress",
                "translation_drift",
                "fail_reasons",
            ],
        )
    )
    sections.append("")
    sections.append("## Progress Outlier Audit")
    sections.append(
        _table(
            [
                {
                    "progress_threshold": outlier_audit["progress_threshold"],
                    "sample_count": outlier_audit["sample_count"],
                    "outlier_count": outlier_audit["outlier_count"],
                    "outlier_ratio": outlier_audit["outlier_ratio"],
                    "conclusion_flag": outlier_audit["conclusion_flag"],
                }
            ],
            ["progress_threshold", "sample_count", "outlier_count", "outlier_ratio", "conclusion_flag"],
        )
    )
    sections.append("")
    sections.append(f"conclusion: {outlier_audit['conclusion']}")
    sections.append("")
    sections.append("### Outlier Heuristic Distribution")
    sections.append(_table([{"heuristic": k, "count": v} for k, v in outlier_audit["by_heuristic"].items()], ["heuristic", "count"]))
    sections.append("")
    sections.append("### Outlier By Terrain")
    sections.append(_table([{"terrain_type": k, "count": v} for k, v in outlier_audit["by_terrain"].items()], ["terrain_type", "count"]))
    sections.append("")
    sections.append("### Outlier By Vehicle")
    sections.append(_table([{"vehicle_type": k, "count": v} for k, v in outlier_audit["by_vehicle"].items()], ["vehicle_type", "count"]))
    sections.append("")
    sections.append("### Outlier By Action")
    sections.append(_table([{"action_id": k, "count": v} for k, v in outlier_audit["by_action"].items()], ["action_id", "count"]))
    sections.append("")
    sections.append("### Outlier Fail Reason Distribution")
    sections.append(_table([{"fail_reason": k, "count": v} for k, v in outlier_audit["fail_reason_distribution"].items()], ["fail_reason", "count"]))
    sections.append("")
    sections.append("### Outlier Details")
    sections.append(
        _table(
            outlier_audit["outlier_details"],
            [
                "sample_id",
                "terrain_type",
                "vehicle_type",
                "action_id",
                "friction_class",
                "friction_mu",
                "progress_metric",
                "progress_value",
                "estimated_displacement_m",
                "estimated_mean_speed_mps",
                "commanded_speed_mps",
                "estimated_heading_change_deg",
                "translation_drift",
                "fail_reasons",
                "heuristic_judgement",
            ],
        )
    )
    path.write_text("\n".join(sections) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset_dir.resolve()
    accepted_path = dataset_dir / "accepted_samples.jsonl"
    if not accepted_path.exists():
        raise FileNotFoundError(f"missing {accepted_path}")

    records = _load_jsonl(accepted_path)
    metadata_by_id = _load_metadata(dataset_dir)
    actions = _load_actions(args.action_config.resolve())

    flat_audit = _flat_fail_audit(records, metadata_by_id, flat_terrain_name=args.flat_terrain_name)
    outlier_audit = _progress_outlier_audit(
        records=records,
        metadata_by_id=metadata_by_id,
        actions=actions,
        progress_threshold=float(args.progress_threshold),
    )

    out_json = dataset_dir / f"{args.output_prefix}.json"
    out_md = dataset_dir / f"{args.output_prefix}.md"
    payload = {
        "flat_fail_audit": flat_audit,
        "progress_outlier_audit": outlier_audit,
    }
    out_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_markdown(path=out_md, flat_audit=flat_audit, outlier_audit=outlier_audit)
    print(json.dumps({"json": str(out_json), "md": str(out_md)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
