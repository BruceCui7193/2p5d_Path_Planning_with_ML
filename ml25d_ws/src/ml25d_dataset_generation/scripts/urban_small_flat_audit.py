#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import glob
import json
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, VEHICLE_PARAM_ORDER, VehicleParams
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


@dataclass(frozen=True)
class FlatCase:
    sample_id: int
    case_type: str  # fail | pass
    action_id: str
    friction_class: str
    motion_model: str
    original_fail: float
    original_fail_reasons: tuple[str, ...]
    mu: float
    heightmap: np.ndarray
    vehicle: VehicleParams


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="urban_small flat failure localization audit")
    p.add_argument("--dataset-dir", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, default=Path("data/diagnostics/urban_small_flat_audit"))
    p.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    p.add_argument("--replay-fail-count", type=int, default=10)
    p.add_argument("--replay-pass-count", type=int, default=5)
    p.add_argument("--replay-repeats", type=int, default=5)
    p.add_argument("--seed", type=int, default=20260428)
    return p.parse_args()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _mu_bin(mu: float) -> str:
    bins = [0.20, 0.35, 0.50, 0.65, 0.80, 0.95]
    mu = float(np.clip(mu, bins[0], bins[-1]))
    for lo, hi in zip(bins[:-1], bins[1:]):
        if lo <= mu < hi:
            return f"[{lo:.2f},{hi:.2f})"
    return f"[{bins[-2]:.2f},{bins[-1]:.2f}]"


def _decode_vehicle(theta_v: np.ndarray, vehicle_id: str, bounds: dict[str, list[float]]) -> VehicleParams:
    values: dict[str, float] = {}
    for idx, key in enumerate(VEHICLE_PARAM_ORDER):
        lo, hi = bounds[key]
        values[key] = float(lo + float(theta_v[idx]) * float(hi - lo))
    return VehicleParams(vehicle_id=vehicle_id, **values)


def _load_sample_cache(dataset_dir: Path, bounds: dict[str, list[float]]) -> dict[int, dict[str, Any]]:
    cache: dict[int, dict[str, Any]] = {}
    for h5_path in sorted(glob.glob(str(dataset_dir / "samples_batch_*.h5"))):
        with h5py.File(h5_path, "r") as h5f:
            x_map = h5f["X_map"]
            theta_v = h5f["theta_v"]
            mu = h5f["mu"]
            metadata = h5f["metadata_json"]
            n = int(x_map.shape[0])
            for i in range(n):
                md = json.loads(metadata[i].decode("utf-8") if isinstance(metadata[i], bytes) else metadata[i])
                sid = int(md["sample_id"])
                vehicle = _decode_vehicle(np.asarray(theta_v[i], dtype=np.float64), str(md["vehicle_id"]), bounds)
                cache[sid] = {
                    "heightmap": np.asarray(x_map[i, :, :, 0], dtype=np.float32).copy(),
                    "grad_u": np.asarray(x_map[i, :, :, 1], dtype=np.float32).copy(),
                    "grad_v": np.asarray(x_map[i, :, :, 2], dtype=np.float32).copy(),
                    "mu": float(mu[i, 0]),
                    "vehicle": vehicle,
                }
    return cache


def _action_balanced_pick(rows: list[dict[str, Any]], n: int) -> list[dict[str, Any]]:
    pools: dict[str, list[dict[str, Any]]] = {
        aid: sorted([r for r in rows if str(r["action_id"]) == aid], key=lambda x: int(x["sample_id"]))
        for aid in ["a0", "a1", "a2"]
    }
    selected: list[dict[str, Any]] = []
    while len(selected) < n and any(pools[k] for k in pools):
        for aid in ["a0", "a1", "a2"]:
            if len(selected) >= n:
                break
            if pools[aid]:
                selected.append(pools[aid].pop(0))
    if len(selected) < n:
        leftovers = sorted(
            [r for aid in ["a0", "a1", "a2"] for r in pools[aid]],
            key=lambda x: int(x["sample_id"]),
        )
        selected.extend(leftovers[: n - len(selected)])
    return selected[:n]


def _table(rows: list[dict[str, Any]], headers: list[str]) -> str:
    def _fmt(v: Any) -> str:
        if isinstance(v, float):
            if np.isnan(v):
                return "NaN"
            return f"{v:.4f}"
        return str(v)

    out = []
    out.append("| " + " | ".join(headers) + " |")
    out.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        out.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(out)


def _sample_fail_reasons(labels, thresholds) -> list[str]:
    reasons: list[str] = []
    if labels.q_roll > thresholds.roll_fail_threshold:
        reasons.append("roll")
    if labels.q_pitch > thresholds.pitch_fail_threshold:
        reasons.append("pitch")
    if labels.q_slip > thresholds.slip_fail_threshold:
        reasons.append("slip")
    if labels.q_lift > thresholds.lift_fail_threshold:
        reasons.append("lift")
    if labels.p_bottom > thresholds.bottom_fail_threshold:
        reasons.append("bottom")
    if labels.p_stuck >= 1.0:
        reasons.append("stuck")
    return reasons


def _run_with_retry(*, runner, context: SimulationContext, seed: int, retries: int = 2):
    for k in range(retries + 1):
        rng = np.random.default_rng(int(seed) + 7919 * k)
        try:
            return runner.run(context, rng), None
        except Exception as exc:
            if k == retries:
                return None, str(exc)
    return None, "unreachable"


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    bounds = manager.vehicle_cfg["normalization_bounds"]
    action_map = {a.action_id: a for a in manager.action_library}

    accepted_rows = _load_jsonl(dataset_dir / "accepted_samples.jsonl")
    accepted_by_id = {int(r["sample_id"]): r for r in accepted_rows}
    cache = _load_sample_cache(dataset_dir, bounds=bounds)

    flat_rows: list[dict[str, Any]] = []
    for sid in sorted(accepted_by_id):
        row = accepted_by_id[sid]
        if str(row.get("terrain_class")) != "flat":
            continue
        if sid not in cache:
            continue
        c = cache[sid]
        h = c["heightmap"].astype(np.float64)
        grad_u = c["grad_u"].astype(np.float64)
        grad_v = c["grad_v"].astype(np.float64)
        slope_mag = np.sqrt(grad_u * grad_u + grad_v * grad_v)
        flat_rows.append(
            {
                **row,
                "mu": float(c["mu"]),
                "mu_bin": _mu_bin(float(c["mu"])),
                "terrain_H_std": float(np.std(h)),
                "terrain_H_range": float(np.max(h) - np.min(h)),
                "max_slope": float(np.max(slope_mag)),
                "vehicle_params": c["vehicle"],
                "heightmap": c["heightmap"],
            }
        )

    # ---- Check 1: offline flat slice report ----
    flat_fail = [r for r in flat_rows if float(r["y_fail"]) >= 0.5]
    report: dict[str, Any] = {
        "flat_total_count": len(flat_rows),
        "flat_fail_count": len(flat_fail),
        "flat_fail_rate": float(len(flat_fail) / max(len(flat_rows), 1)),
    }

    # vehicle x action x mu_bin fail rate
    g_vam: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    for r in flat_rows:
        g_vam[(str(r["vehicle_id"]), str(r["action_id"]), str(r["mu_bin"]))].append(float(r["y_fail"]) >= 0.5)
    vam_rows = []
    for (v, a, m), vals in sorted(g_vam.items()):
        vam_rows.append(
            {
                "vehicle_id": v,
                "action_id": a,
                "mu_bin": m,
                "count": len(vals),
                "fail_rate": float(np.mean(vals)),
            }
        )
    report["vehicle_action_mu_bin_fail_rate"] = vam_rows

    # vehicle x fail_reason distribution
    reason_by_vehicle: dict[str, Counter[str]] = defaultdict(Counter)
    for r in flat_fail:
        for rs in r.get("fail_reasons", []):
            reason_by_vehicle[str(r["vehicle_id"])][str(rs)] += 1
    reason_rows = []
    for v in sorted(reason_by_vehicle):
        total = sum(reason_by_vehicle[v].values())
        for reason, cnt in sorted(reason_by_vehicle[v].items()):
            reason_rows.append(
                {
                    "vehicle_id": v,
                    "fail_reason": reason,
                    "count": int(cnt),
                    "ratio_in_vehicle_fail_reasons": float(cnt / max(total, 1)),
                }
            )
    report["vehicle_fail_reason_distribution"] = reason_rows

    # per-sample terrain stats
    terrain_rows = []
    for r in flat_rows:
        terrain_rows.append(
            {
                "sample_id": int(r["sample_id"]),
                "vehicle_id": str(r["vehicle_id"]),
                "action_id": str(r["action_id"]),
                "mu": float(r["mu"]),
                "mu_bin": str(r["mu_bin"]),
                "y_fail": float(r["y_fail"]),
                "terrain_H_std": float(r["terrain_H_std"]),
                "terrain_H_range": float(r["terrain_H_range"]),
                "max_slope": float(r["max_slope"]),
            }
        )
    report["flat_sample_terrain_stats"] = terrain_rows
    report["flat_terrain_stats_summary"] = {
        "terrain_H_std_mean": float(np.mean([x["terrain_H_std"] for x in terrain_rows])) if terrain_rows else float("nan"),
        "terrain_H_std_p95": float(np.percentile([x["terrain_H_std"] for x in terrain_rows], 95)) if terrain_rows else float("nan"),
        "terrain_H_range_mean": float(np.mean([x["terrain_H_range"] for x in terrain_rows])) if terrain_rows else float("nan"),
        "terrain_H_range_p95": float(np.percentile([x["terrain_H_range"] for x in terrain_rows], 95)) if terrain_rows else float("nan"),
        "max_slope_mean": float(np.mean([x["max_slope"] for x in terrain_rows])) if terrain_rows else float("nan"),
        "max_slope_p95": float(np.percentile([x["max_slope"] for x in terrain_rows], 95)) if terrain_rows else float("nan"),
    }

    # urban_small parameter distribution fail/pass
    urban = [r for r in flat_rows if str(r["vehicle_id"]) == "urban_small"]
    urban_fail = [r for r in urban if float(r["y_fail"]) >= 0.5]
    urban_pass = [r for r in urban if float(r["y_fail"]) < 0.5]
    param_map = {
        "wheel_radius": "r_w",
        "track_width": "W",
        "wheelbase": "l",
        "clearance": "c_g",
        "z_com": "z_c",
        "roll_limit": "phi_max_deg",
        "pitch_limit": "theta_max_deg",
        "drive_force": "F_max",
    }

    param_rows = []
    for out_name, key in param_map.items():
        fail_vals = np.array([float(getattr(r["vehicle_params"], key)) for r in urban_fail], dtype=np.float64)
        pass_vals = np.array([float(getattr(r["vehicle_params"], key)) for r in urban_pass], dtype=np.float64)
        param_rows.append(
            {
                "param": out_name,
                "fail_mean": float(np.mean(fail_vals)) if fail_vals.size else float("nan"),
                "fail_p50": float(np.percentile(fail_vals, 50)) if fail_vals.size else float("nan"),
                "pass_mean": float(np.mean(pass_vals)) if pass_vals.size else float("nan"),
                "pass_p50": float(np.percentile(pass_vals, 50)) if pass_vals.size else float("nan"),
                "delta_fail_minus_pass": (
                    float(np.mean(fail_vals) - np.mean(pass_vals)) if (fail_vals.size and pass_vals.size) else float("nan")
                ),
            }
        )
    report["urban_small_param_fail_pass_compare"] = param_rows

    # urban_small fail mainly from a0 or a1/a2
    urban_fail_action = Counter(str(r["action_id"]) for r in urban_fail)
    urban_fail_total = int(sum(urban_fail_action.values()))
    a0_cnt = int(urban_fail_action.get("a0", 0))
    a12_cnt = int(urban_fail_action.get("a1", 0) + urban_fail_action.get("a2", 0))
    report["urban_small_fail_action_breakdown"] = {
        "total_fail": urban_fail_total,
        "a0_fail_count": a0_cnt,
        "a1_fail_count": int(urban_fail_action.get("a1", 0)),
        "a2_fail_count": int(urban_fail_action.get("a2", 0)),
        "a1_a2_fail_count": a12_cnt,
        "a0_ratio": float(a0_cnt / max(urban_fail_total, 1)),
        "a1_a2_ratio": float(a12_cnt / max(urban_fail_total, 1)),
        "dominant": "a1_a2" if a12_cnt > a0_cnt else ("a0" if a0_cnt > a12_cnt else "tie"),
    }

    # ---- Check 2: replay selected urban_small+flat samples ----
    urban_fail_pool = sorted(urban_fail, key=lambda x: int(x["sample_id"]))
    urban_pass_pool = sorted(urban_pass, key=lambda x: int(x["sample_id"]))
    fail_cases = _action_balanced_pick(urban_fail_pool, int(args.replay_fail_count))
    pass_cases = _action_balanced_pick(urban_pass_pool, int(args.replay_pass_count))

    cases: list[FlatCase] = []
    for r in fail_cases:
        sid = int(r["sample_id"])
        cases.append(
            FlatCase(
                sample_id=sid,
                case_type="fail",
                action_id=str(r["action_id"]),
                friction_class=str(r["friction_class"]),
                motion_model=str(r["motion_model"]),
                original_fail=float(r["y_fail"]),
                original_fail_reasons=tuple(str(x) for x in r.get("fail_reasons", [])),
                mu=float(r["mu"]),
                heightmap=np.asarray(r["heightmap"], dtype=np.float32),
                vehicle=r["vehicle_params"],
            )
        )
    for r in pass_cases:
        sid = int(r["sample_id"])
        cases.append(
            FlatCase(
                sample_id=sid,
                case_type="pass",
                action_id=str(r["action_id"]),
                friction_class=str(r["friction_class"]),
                motion_model=str(r["motion_model"]),
                original_fail=float(r["y_fail"]),
                original_fail_reasons=tuple(str(x) for x in r.get("fail_reasons", [])),
                mu=float(r["mu"]),
                heightmap=np.asarray(r["heightmap"], dtype=np.float32),
                vehicle=r["vehicle_params"],
            )
        )

    sim_cfg = copy.deepcopy(manager.sim_cfg)
    if args.backend == "ros_gz":
        ros = sim_cfg.setdefault("ros_gz", {})
        ros["model_name"] = "ml25d_us_flat_replay"
        ros["log_dir"] = str((output_dir / "runner_logs_replay").resolve())
    runner = make_runner(args.backend, sim_cfg)
    replay_rows: list[dict[str, Any]] = []
    try:
        for ci, case in enumerate(cases):
            action: ActionPrimitive = action_map[case.action_id]
            for rep in range(int(args.replay_repeats)):
                context = SimulationContext(
                    heightmap=case.heightmap,
                    heading_rad=0.0,
                    vehicle=case.vehicle,
                    action=action,
                    friction_mu=float(case.mu),
                    motion_model=case.motion_model,
                    sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
                    duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                    settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
                    scene_id=f"usflat_s{case.sample_id}_c{ci}_r{rep}",
                )
                traj, err = _run_with_retry(
                    runner=runner,
                    context=context,
                    seed=int(args.seed) + int(case.sample_id) * 17 + rep,
                    retries=2,
                )
                if traj is None:
                    replay_rows.append(
                        {
                            "sample_id": case.sample_id,
                            "case_type": case.case_type,
                            "action_id": case.action_id,
                            "rep": rep,
                            "status": "runtime_failure",
                            "error": err,
                        }
                    )
                    continue
                labels, _ = manager.label_extractor.compute_labels(traj, case.vehicle, action)
                reasons = _sample_fail_reasons(labels, manager.label_extractor.thresholds)
                replay_rows.append(
                    {
                        "sample_id": case.sample_id,
                        "case_type": case.case_type,
                        "action_id": case.action_id,
                        "rep": rep,
                        "status": "ok",
                        "replay_fail": float(labels.y_fail),
                        "replay_fail_reasons": reasons,
                        "q_roll": float(labels.q_roll),
                        "q_pitch": float(labels.q_pitch),
                        "q_slip": float(labels.q_slip),
                        "q_lift": float(labels.q_lift),
                        "p_bottom": float(labels.p_bottom),
                        "p_stuck": float(labels.p_stuck),
                    }
                )
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass

    # case-level reproduction summary
    by_sid = defaultdict(list)
    for r in replay_rows:
        by_sid[int(r["sample_id"])].append(r)

    case_summary_rows = []
    for case in cases:
        rr = by_sid.get(case.sample_id, [])
        ok = [x for x in rr if x["status"] == "ok"]
        fail_rate = float(np.mean([float(x["replay_fail"]) >= 0.5 for x in ok])) if ok else float("nan")
        reason_counter = Counter()
        for x in ok:
            if float(x["replay_fail"]) >= 0.5:
                reason_counter.update(str(s) for s in x.get("replay_fail_reasons", []))
        case_summary_rows.append(
            {
                "sample_id": case.sample_id,
                "case_type": case.case_type,
                "action_id": case.action_id,
                "original_fail": case.original_fail,
                "ok_repeats": len(ok),
                "runtime_fail_repeats": len(rr) - len(ok),
                "replay_fail_rate": fail_rate,
                "replay_pass_rate": (1.0 - fail_rate) if np.isfinite(fail_rate) else float("nan"),
                "original_fail_reasons": list(case.original_fail_reasons),
                "replay_fail_reason_distribution": dict(sorted(reason_counter.items())),
            }
        )

    fail_case_rates = [r["replay_fail_rate"] for r in case_summary_rows if r["case_type"] == "fail" and np.isfinite(r["replay_fail_rate"])]
    pass_case_rates = [r["replay_fail_rate"] for r in case_summary_rows if r["case_type"] == "pass" and np.isfinite(r["replay_fail_rate"])]
    replay_summary = {
        "selected_fail_cases": int(sum(1 for c in cases if c.case_type == "fail")),
        "selected_pass_cases": int(sum(1 for c in cases if c.case_type == "pass")),
        "repeats_per_case": int(args.replay_repeats),
        "fail_case_mean_replay_fail_rate": float(np.mean(fail_case_rates)) if fail_case_rates else float("nan"),
        "pass_case_mean_replay_fail_rate": float(np.mean(pass_case_rates)) if pass_case_rates else float("nan"),
        "rows": case_summary_rows,
    }

    # save files
    (output_dir / "flat_slice_samples.jsonl").write_text(
        "\n".join(json.dumps(r, ensure_ascii=False) for r in terrain_rows) + ("\n" if terrain_rows else ""),
        encoding="utf-8",
    )
    (output_dir / "replay_runs.jsonl").write_text(
        "\n".join(json.dumps(r, ensure_ascii=False) for r in replay_rows) + ("\n" if replay_rows else ""),
        encoding="utf-8",
    )

    flat_slice_path = output_dir / "flat_slice_report.json"
    flat_slice_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    replay_path = output_dir / "urban_small_flat_replay_report.json"
    replay_path.write_text(json.dumps(replay_summary, ensure_ascii=False, indent=2), encoding="utf-8")

    md_lines: list[str] = []
    md_lines.append("## Flat Slice Report")
    md_lines.append(
        _table(
            [
                {
                    "flat_total_count": report["flat_total_count"],
                    "flat_fail_count": report["flat_fail_count"],
                    "flat_fail_rate": report["flat_fail_rate"],
                }
            ],
            ["flat_total_count", "flat_fail_count", "flat_fail_rate"],
        )
    )
    md_lines.append("")
    md_lines.append("### vehicle × action × mu_bin fail_rate")
    md_lines.append(_table(vam_rows, ["vehicle_id", "action_id", "mu_bin", "count", "fail_rate"]))
    md_lines.append("")
    md_lines.append("### vehicle × fail_reason")
    md_lines.append(_table(reason_rows, ["vehicle_id", "fail_reason", "count", "ratio_in_vehicle_fail_reasons"]))
    md_lines.append("")
    md_lines.append("### urban_small parameter fail/pass compare")
    md_lines.append(_table(param_rows, ["param", "fail_mean", "fail_p50", "pass_mean", "pass_p50", "delta_fail_minus_pass"]))
    md_lines.append("")
    md_lines.append("### urban_small fail action breakdown")
    md_lines.append(_table([report["urban_small_fail_action_breakdown"]], list(report["urban_small_fail_action_breakdown"].keys())))
    md_lines.append("")
    md_lines.append("## Replay Reproducibility")
    md_lines.append(
        _table(
            [
                {
                    "selected_fail_cases": replay_summary["selected_fail_cases"],
                    "selected_pass_cases": replay_summary["selected_pass_cases"],
                    "repeats_per_case": replay_summary["repeats_per_case"],
                    "fail_case_mean_replay_fail_rate": replay_summary["fail_case_mean_replay_fail_rate"],
                    "pass_case_mean_replay_fail_rate": replay_summary["pass_case_mean_replay_fail_rate"],
                }
            ],
            [
                "selected_fail_cases",
                "selected_pass_cases",
                "repeats_per_case",
                "fail_case_mean_replay_fail_rate",
                "pass_case_mean_replay_fail_rate",
            ],
        )
    )
    md_lines.append("")
    md_lines.append("### per-case replay summary")
    md_lines.append(
        _table(
            case_summary_rows,
            [
                "sample_id",
                "case_type",
                "action_id",
                "original_fail",
                "ok_repeats",
                "runtime_fail_repeats",
                "replay_fail_rate",
                "replay_pass_rate",
                "original_fail_reasons",
                "replay_fail_reason_distribution",
            ],
        )
    )
    (output_dir / "urban_small_flat_audit.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "flat_slice_report": str(flat_slice_path),
                "replay_report": str(replay_path),
                "markdown": str(output_dir / "urban_small_flat_audit.md"),
                "flat_slice_samples": str(output_dir / "flat_slice_samples.jsonl"),
                "replay_runs": str(output_dir / "replay_runs.jsonl"),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

