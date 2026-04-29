#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, SimulationTrajectory, VehicleParams
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


@dataclass(frozen=True)
class ReplayCase:
    case_id: str
    case_type: str
    vehicle_id: str
    action_id: str
    friction_class: str
    friction_mu: float
    seed: int
    original_fail_reasons: list[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal root-cause diagnostics for flat fail issue")
    parser.add_argument(
        "--samples-jsonl",
        type=Path,
        default=Path("data/diagnostics/flat_action_audit_v1/flat_action_samples.jsonl"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/diagnostics/flat_root_cause_v1"),
    )
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    parser.add_argument("--exp2-max-retry", type=int, default=2)
    parser.add_argument("--exp3-max-retry", type=int, default=2)
    parser.add_argument("--golden-repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260427)
    return parser.parse_args()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        if np.isnan(value):
            return "NaN"
        return f"{value:.4f}"
    return str(value)


def _table(rows: list[dict[str, Any]], headers: list[str]) -> str:
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(lines)


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


def _exp1_offline_tables(rows: list[dict[str, Any]]) -> dict[str, Any]:
    fail_rows = [r for r in rows if float(r["y_fail"]) >= 0.5]
    reason_counter: Counter[str] = Counter()
    for r in fail_rows:
        reason_counter.update(str(x) for x in r.get("fail_reasons", []))

    group_rows = []
    for vehicle in sorted({str(r["vehicle_type"]) for r in rows}):
        for action in sorted({str(r["action_id"]) for r in rows}):
            for friction in sorted({str(r["friction_class"]) for r in rows}):
                sub = [
                    r
                    for r in rows
                    if str(r["vehicle_type"]) == vehicle
                    and str(r["action_id"]) == action
                    and str(r["friction_class"]) == friction
                ]
                if not sub:
                    continue
                fail_rate = float(np.mean(np.array([float(x["y_fail"]) >= 0.5 for x in sub], dtype=bool)))
                group_rows.append(
                    {
                        "vehicle_type": vehicle,
                        "action_id": action,
                        "friction_class": friction,
                        "count": len(sub),
                        "fail_rate": fail_rate,
                    }
                )

    reason_names = ["roll", "pitch", "slip", "lift", "bottom", "stuck"]
    co = {a: {b: 0 for b in reason_names} for a in reason_names}
    for r in fail_rows:
        present = set(str(x) for x in r.get("fail_reasons", []))
        for a in reason_names:
            if a not in present:
                continue
            for b in reason_names:
                if b in present:
                    co[a][b] += 1

    def _stats(arr: np.ndarray) -> dict[str, float]:
        return {
            "mean": float(np.mean(arr)),
            "p50": float(np.percentile(arr, 50)),
            "p95": float(np.percentile(arr, 95)),
            "max": float(np.max(arr)),
            "frac_eq_1": float(np.mean(arr >= 0.999)),
        }

    q_rows = []
    for metric in ["q_roll", "q_pitch", "q_lift", "p_stuck"]:
        arr_fail = np.array([float(r[metric]) for r in fail_rows], dtype=np.float64)
        arr_pass = np.array([float(r[metric]) for r in rows if float(r["y_fail"]) < 0.5], dtype=np.float64)
        sf = _stats(arr_fail) if arr_fail.size > 0 else {k: float("nan") for k in ["mean", "p50", "p95", "max", "frac_eq_1"]}
        sp = _stats(arr_pass) if arr_pass.size > 0 else {k: float("nan") for k in ["mean", "p50", "p95", "max", "frac_eq_1"]}
        q_rows.append(
            {
                "metric": metric,
                "fail_mean": sf["mean"],
                "fail_p50": sf["p50"],
                "fail_p95": sf["p95"],
                "fail_max": sf["max"],
                "fail_frac_eq_1": sf["frac_eq_1"],
                "pass_mean": sp["mean"],
                "pass_p50": sp["p50"],
                "pass_p95": sp["p95"],
                "pass_max": sp["max"],
                "pass_frac_eq_1": sp["frac_eq_1"],
            }
        )

    return {
        "sample_count": len(rows),
        "fail_count": len(fail_rows),
        "fail_rate": float(len(fail_rows) / max(len(rows), 1)),
        "fail_reason_distribution": dict(sorted(reason_counter.items())),
        "group_fail_table": group_rows,
        "reason_cooccurrence": co,
        "q_distribution": q_rows,
    }


def _select_replay_cases(rows: list[dict[str, Any]]) -> list[ReplayCase]:
    selected: list[ReplayCase] = []
    used: set[tuple[int, str]] = set()

    def _pick(cond, n: int, prefix: str, case_type: str) -> None:
        picked = 0
        for r in rows:
            key = (int(r["combo_id"]), str(r["seed"]))
            if key in used:
                continue
            if not cond(r):
                continue
            idx = len([x for x in selected if x.case_type == case_type]) + 1
            selected.append(
                ReplayCase(
                    case_id=f"{prefix}_{idx:02d}",
                    case_type=case_type,
                    vehicle_id=str(r["vehicle_id"]),
                    action_id=str(r["action_id"]),
                    friction_class=str(r["friction_class"]),
                    friction_mu=float(r["friction_mu"]),
                    seed=int(r["seed"]),
                    original_fail_reasons=[str(x) for x in r.get("fail_reasons", [])],
                )
            )
            used.add(key)
            picked += 1
            if picked >= n:
                return

    fail_rows = [r for r in rows if float(r["y_fail"]) >= 0.5]
    pass_rows = [r for r in rows if float(r["y_fail"]) < 0.5]

    for action_id in ["a0", "a1", "a2"]:
        _pick(lambda r, aid=action_id: float(r["y_fail"]) >= 0.5 and str(r["action_id"]) == aid, 3, f"{action_id}_fail", "fail")
    _pick(lambda r: float(r["y_fail"]) >= 0.5 and str(r["action_id"]) in {"a3", "a4"}, 3, "a34_fail", "fail")

    for action_id in ["a0", "a1", "a2", "a3", "a4"]:
        _pick(lambda r, aid=action_id: float(r["y_fail"]) < 0.5 and str(r["action_id"]) == aid, 1, f"{action_id}_pass", "pass")
    _pick(lambda r: float(r["y_fail"]) < 0.5, 1, "pass_extra", "pass")

    # Ensure deterministic ordering and exact expected counts.
    return selected


def _slice_trajectory(traj: SimulationTrajectory, start_sec: float) -> SimulationTrajectory:
    if start_sec <= 0.0 or traj.timestamps.size <= 2:
        return traj
    idx = int(np.searchsorted(traj.timestamps, start_sec, side="left"))
    if idx >= traj.timestamps.size - 1:
        idx = max(traj.timestamps.size - 2, 0)
    t = traj.timestamps[idx:].copy()
    t = (t - t[0]).astype(np.float32)
    pos = traj.positions_xy[idx:].copy()
    yaw = traj.yaw_rad[idx:].copy()
    roll = traj.roll_rad[idx:].copy()
    pitch = traj.pitch_rad[idx:].copy()
    cmd_lin = traj.commanded_linear_speed[idx:].copy()
    act_lin = traj.actual_linear_speed[idx:].copy()
    cmd_ang = traj.commanded_angular_speed[idx:].copy()
    act_ang = traj.actual_angular_speed[idx:].copy()
    wheel_forces = traj.wheel_contact_forces[idx:].copy()
    chassis_contacts = traj.chassis_contacts[idx:].copy()
    completed_displacement = float(np.linalg.norm(pos[-1] - pos[0])) if pos.shape[0] >= 2 else 0.0
    unwrapped_yaw = np.unwrap(yaw) if yaw.size > 0 else yaw
    completed_heading = float(abs(unwrapped_yaw[-1] - unwrapped_yaw[0])) if yaw.size >= 2 else 0.0

    def _slice_opt(arr):
        if arr is None:
            return None
        return arr[idx:].copy()

    return SimulationTrajectory(
        timestamps=t,
        positions_xy=pos,
        yaw_rad=yaw,
        roll_rad=roll,
        pitch_rad=pitch,
        commanded_linear_speed=cmd_lin,
        actual_linear_speed=act_lin,
        commanded_angular_speed=cmd_ang,
        actual_angular_speed=act_ang,
        wheel_contact_forces=wheel_forces,
        chassis_contacts=chassis_contacts,
        completed_displacement_m=completed_displacement,
        completed_heading_change_rad=completed_heading,
        wheel_contact_observed=_slice_opt(traj.wheel_contact_observed),
        wheel_contact_latched=_slice_opt(traj.wheel_contact_latched),
        wheel_clearance_m=_slice_opt(traj.wheel_clearance_m),
        wheel_lift_valid_mask=_slice_opt(traj.wheel_lift_valid_mask),
        wheel_lift_state=_slice_opt(traj.wheel_lift_state),
        chassis_min_clearance_m=_slice_opt(traj.chassis_min_clearance_m),
    )


def _first_crossing_time(t: np.ndarray, values: np.ndarray, threshold: float) -> float:
    if values.size == 0:
        return float("nan")
    idx = np.where(values > threshold)[0]
    if idx.size == 0:
        return float("nan")
    return float(t[int(idx[0])])


def _nanmean(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    if np.all(~np.isfinite(values)):
        return float("nan")
    return float(np.nanmean(values))


def _timeseries_diagnostics(
    *,
    traj: SimulationTrajectory,
    vehicle: VehicleParams,
    action: ActionPrimitive,
    thresholds,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    t = traj.timestamps.astype(np.float64)
    n = t.size
    if n < 2:
        raise RuntimeError("trajectory too short")
    dt = float(np.mean(np.diff(t)))
    phi_max = max(np.deg2rad(float(vehicle.phi_max_deg)), 1e-6)
    theta_max = max(np.deg2rad(float(vehicle.theta_max_deg)), 1e-6)

    roll_ratio = np.abs(traj.roll_rad.astype(np.float64)) / phi_max
    pitch_ratio = np.abs(traj.pitch_rad.astype(np.float64)) / theta_max
    roll_deg = np.rad2deg(traj.roll_rad.astype(np.float64))
    pitch_deg = np.rad2deg(traj.pitch_rad.astype(np.float64))
    yaw_unwrap = np.unwrap(traj.yaw_rad.astype(np.float64))
    yaw_deg = np.rad2deg(yaw_unwrap)

    vx = np.gradient(traj.positions_xy[:, 0].astype(np.float64), t)
    vy = np.gradient(traj.positions_xy[:, 1].astype(np.float64), t)
    omega = np.gradient(yaw_unwrap, t)

    cmd_lin = np.abs(traj.commanded_linear_speed.astype(np.float64))
    act_lin = np.abs(traj.actual_linear_speed.astype(np.float64))
    slip_frame = np.zeros(n, dtype=np.float64)
    valid_slip = cmd_lin > 0.03
    slip_frame[valid_slip] = np.abs(cmd_lin[valid_slip] - act_lin[valid_slip]) / (cmd_lin[valid_slip] + 1e-6)

    warmup_idx = int(0.2 * n)
    running_q_slip = np.zeros(n, dtype=np.float64)
    for i in range(n):
        if i < warmup_idx:
            running_q_slip[i] = 0.0
            continue
        s = slip_frame[warmup_idx : i + 1]
        v = valid_slip[warmup_idx : i + 1]
        if np.any(v):
            robust = float(np.percentile(np.clip(s[v], 0.0, 1.5), 70))
        else:
            robust = 0.0
        running_q_slip[i] = float(np.clip(robust / max(thresholds.slip_normalizer, 1e-6), 0.0, 1.0))

    lift_ratio_frame = np.full(n, np.nan, dtype=np.float64)
    running_q_lift = np.full(n, np.nan, dtype=np.float64)
    if traj.wheel_lift_valid_mask is not None and traj.wheel_lift_state is not None:
        valid = np.asarray(traj.wheel_lift_valid_mask, dtype=bool)
        lifted = np.asarray(traj.wheel_lift_state, dtype=bool)
        valid_cnt = np.sum(valid, axis=1).astype(np.float64)
        lift_cnt = np.sum(valid & lifted, axis=1).astype(np.float64)
        usable = valid_cnt > 0.0
        lift_ratio_frame[usable] = lift_cnt[usable] / valid_cnt[usable]
        cum_valid = np.cumsum(valid_cnt)
        cum_lift = np.cumsum(lift_cnt)
        ok = cum_valid > 0.0
        running_q_lift[ok] = cum_lift[ok] / cum_valid[ok]

    bottom_clear = (
        np.asarray(traj.chassis_min_clearance_m, dtype=np.float64)
        if traj.chassis_min_clearance_m is not None
        else np.full(n, np.nan, dtype=np.float64)
    )
    bottom_clear_event = np.isfinite(bottom_clear) & (bottom_clear < thresholds.bottom_clearance_threshold_m)
    bottom_contact_event = np.asarray(traj.chassis_contacts, dtype=np.float64) > 0.5
    running_clear_ratio = np.cumsum(bottom_clear_event.astype(np.float64)) / np.arange(1, n + 1)
    running_contact_raw = np.cumsum(bottom_contact_event.astype(np.float64)) / np.arange(1, n + 1)
    running_contact_ratio = np.where(
        running_contact_raw >= thresholds.bottom_contact_min_duration_ratio,
        running_contact_raw,
        0.0,
    )
    running_bottom = np.maximum(running_clear_ratio, running_contact_ratio)

    roll_trigger = _first_crossing_time(t, roll_ratio, thresholds.roll_fail_threshold)
    pitch_trigger = _first_crossing_time(t, pitch_ratio, thresholds.pitch_fail_threshold)
    slip_trigger = _first_crossing_time(t, running_q_slip, thresholds.slip_fail_threshold)
    lift_trigger = _first_crossing_time(t, np.nan_to_num(running_q_lift, nan=-1.0), thresholds.lift_fail_threshold)
    bottom_trigger = _first_crossing_time(t, running_bottom, thresholds.bottom_fail_threshold)

    trigger_candidates = {
        "roll": roll_trigger,
        "pitch": pitch_trigger,
        "slip": slip_trigger,
        "lift": lift_trigger,
        "bottom": bottom_trigger,
    }
    valid_triggers = {k: v for k, v in trigger_candidates.items() if np.isfinite(v)}
    if valid_triggers:
        trigger_reason = min(valid_triggers, key=valid_triggers.get)
        first_trigger = float(valid_triggers[trigger_reason])
    else:
        trigger_reason = "none"
        first_trigger = float("nan")

    split = 0.30
    early = t <= split
    post = t > split
    roll_early_max = float(np.max(np.abs(roll_deg[early]))) if np.any(early) else float("nan")
    roll_post_max = float(np.max(np.abs(roll_deg[post]))) if np.any(post) else float("nan")
    pitch_early_max = float(np.max(np.abs(pitch_deg[early]))) if np.any(early) else float("nan")
    pitch_post_max = float(np.max(np.abs(pitch_deg[post]))) if np.any(post) else float("nan")

    target_yaw = abs(np.deg2rad(float(action.delta_psi_deg)))
    translation_progress = float(traj.completed_displacement_m / max(action.delta_s_m, 1e-6)) if action.delta_s_m > 1e-6 else float("nan")
    angular_progress = float(traj.completed_heading_change_rad / max(target_yaw, 1e-6)) if target_yaw > 1e-6 else float("nan")

    summary = {
        "roll_max_deg": float(np.max(np.abs(roll_deg))),
        "roll_p95_deg": float(np.percentile(np.abs(roll_deg), 95)),
        "pitch_max_deg": float(np.max(np.abs(pitch_deg))),
        "pitch_p95_deg": float(np.percentile(np.abs(pitch_deg), 95)),
        "roll_over_thr_duration_s": float(np.sum(roll_ratio > thresholds.roll_fail_threshold) * dt),
        "pitch_over_thr_duration_s": float(np.sum(pitch_ratio > thresholds.pitch_fail_threshold) * dt),
        "running_q_slip_final": float(running_q_slip[-1]),
        "running_q_lift_final": _nanmean(running_q_lift),
        "running_bottom_ratio_final": float(running_bottom[-1]),
        "roll_trigger_s": roll_trigger,
        "pitch_trigger_s": pitch_trigger,
        "slip_trigger_s": slip_trigger,
        "lift_trigger_s": lift_trigger,
        "bottom_trigger_s": bottom_trigger,
        "first_fail_trigger_s": first_trigger,
        "first_fail_trigger_reason": trigger_reason,
        "first_trigger_in_0p3s": bool(np.isfinite(first_trigger) and first_trigger <= split),
        "roll_early_max_deg": roll_early_max,
        "roll_post_max_deg": roll_post_max,
        "pitch_early_max_deg": pitch_early_max,
        "pitch_post_max_deg": pitch_post_max,
        "translation_progress": translation_progress,
        "angular_progress": angular_progress,
        "translation_drift": float(traj.completed_displacement_m),
    }

    rows = []
    for i in range(n):
        rows.append(
            {
                "t": float(t[i]),
                "roll_deg": float(roll_deg[i]),
                "pitch_deg": float(pitch_deg[i]),
                "yaw_deg": float(yaw_deg[i]),
                "base_z": float("nan"),
                "vx_world": float(vx[i]),
                "vy_world": float(vy[i]),
                "omega": float(omega[i]),
                "cmd_v": float(traj.commanded_linear_speed[i]),
                "cmd_omega": float(traj.commanded_angular_speed[i]),
                "actual_v": float(traj.actual_linear_speed[i]),
                "actual_omega": float(traj.actual_angular_speed[i]),
                "slip_frame": float(slip_frame[i]),
                "running_q_slip": float(running_q_slip[i]),
                "q_lift_frame": float(lift_ratio_frame[i]) if np.isfinite(lift_ratio_frame[i]) else float("nan"),
                "running_q_lift": float(running_q_lift[i]) if np.isfinite(running_q_lift[i]) else float("nan"),
                "bottom_min_clearance": float(bottom_clear[i]) if np.isfinite(bottom_clear[i]) else float("nan"),
                "running_bottom_ratio": float(running_bottom[i]),
            }
        )
    return summary, rows


def _run_context_with_retry(
    *,
    runner,
    context: SimulationContext,
    seed: int,
    max_retry: int,
) -> tuple[SimulationTrajectory | None, str | None]:
    for k in range(max_retry + 1):
        local_seed = int(seed + 1009 * k)
        rng = np.random.default_rng(local_seed)
        try:
            return runner.run(context, rng), None
        except Exception as exc:
            if k == max_retry:
                return None, str(exc)
    return None, "unreachable"


def _exp2_replay_and_timeseries(
    *,
    rows: list[dict[str, Any]],
    manager: DatasetManager,
    backend: str,
    output_dir: Path,
    max_retry: int,
) -> dict[str, Any]:
    cases = _select_replay_cases(rows)
    vehicle_map = {v.vehicle_id: v for v in manager.vehicle_library}
    action_map = {a.action_id: a for a in manager.action_library}
    flat_map = np.zeros((int(manager.map_cfg["patch_size"]), int(manager.map_cfg["patch_size"])), dtype=np.float32)

    sim_cfg = copy.deepcopy(manager.sim_cfg)
    if backend == "ros_gz":
        ros = sim_cfg.setdefault("ros_gz", {})
        ros["model_name"] = "ml25d_exp2_vehicle"
        ros["log_dir"] = str((output_dir / "runner_logs_exp2").resolve())
    runner = make_runner(backend, sim_cfg)

    result_rows: list[dict[str, Any]] = []
    ts_dir = output_dir / "exp2_timeseries"
    ts_dir.mkdir(parents=True, exist_ok=True)
    try:
        for idx, case in enumerate(cases):
            print(
                "[exp2] replay "
                f"{idx + 1}/{len(cases)} case={case.case_id} "
                f"vehicle={case.vehicle_id} action={case.action_id} mu={case.friction_mu:.3f}",
                flush=True,
            )
            vehicle = vehicle_map[case.vehicle_id]
            action = action_map[case.action_id]
            context = SimulationContext(
                heightmap=flat_map,
                heading_rad=0.0,
                vehicle=vehicle,
                action=action,
                friction_mu=float(case.friction_mu),
                motion_model="skid",
                sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
                duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
                scene_id=f"exp2_{case.case_id}_{idx}",
            )
            traj, err = _run_context_with_retry(runner=runner, context=context, seed=case.seed, max_retry=max_retry)
            if traj is None:
                print(f"[exp2] runtime_failure case={case.case_id}: {err}", flush=True)
                result_rows.append(
                    {
                        "case_id": case.case_id,
                        "case_type": case.case_type,
                        "vehicle_id": case.vehicle_id,
                        "action_id": case.action_id,
                        "friction_class": case.friction_class,
                        "friction_mu": case.friction_mu,
                        "status": "runtime_failure",
                        "error": err,
                    }
                )
                continue

            labels, _ = manager.label_extractor.compute_labels(traj, vehicle, action)
            replay_reasons = _sample_fail_reasons(labels, manager.label_extractor.thresholds)
            diag, ts_rows = _timeseries_diagnostics(
                traj=traj,
                vehicle=vehicle,
                action=action,
                thresholds=manager.label_extractor.thresholds,
            )
            ts_path = ts_dir / f"{case.case_id}.jsonl"
            with ts_path.open("w", encoding="utf-8") as fp:
                for row in ts_rows:
                    fp.write(json.dumps(row, ensure_ascii=False) + "\n")

            result_rows.append(
                {
                    "case_id": case.case_id,
                    "case_type": case.case_type,
                    "vehicle_id": case.vehicle_id,
                    "action_id": case.action_id,
                    "friction_class": case.friction_class,
                    "friction_mu": case.friction_mu,
                    "status": "ok",
                    "original_fail_reasons": case.original_fail_reasons,
                    "replay_fail": float(labels.y_fail),
                    "replay_fail_reasons": replay_reasons,
                    "q_roll": float(labels.q_roll),
                    "q_pitch": float(labels.q_pitch),
                    "q_slip": float(labels.q_slip),
                    "q_lift": float(labels.q_lift),
                    "p_bottom": float(labels.p_bottom),
                    "p_stuck": float(labels.p_stuck),
                    **diag,
                    "timeseries_path": str(ts_path),
                }
            )
            print(
                "[exp2] done "
                f"case={case.case_id} replay_fail={float(labels.y_fail):.1f} "
                f"reasons={replay_reasons}",
                flush=True,
            )
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass

    ok_rows = [r for r in result_rows if r["status"] == "ok"]
    trigger_early = sum(int(r["first_trigger_in_0p3s"]) for r in ok_rows)
    trigger_reason_counter: Counter[str] = Counter(str(r["first_fail_trigger_reason"]) for r in ok_rows)
    return {
        "selected_case_count": len(cases),
        "ok_count": len(ok_rows),
        "runtime_failure_count": len(result_rows) - len(ok_rows),
        "first_trigger_in_0p3s_count": int(trigger_early),
        "first_trigger_reason_distribution": dict(sorted(trigger_reason_counter.items())),
        "rows": result_rows,
    }


def _exp3_golden_flat_ab(
    *,
    manager: DatasetManager,
    backend: str,
    output_dir: Path,
    max_retry: int,
    repeats: int,
    seed: int,
) -> dict[str, Any]:
    vehicle_alias = {
        "city_small": "urban_small",
        "offroad_medium": "standard_offroad",
        "mountain_large": "mountain_large",
    }
    vehicle_map = {v.vehicle_id: v for v in manager.vehicle_library}
    action_map = {a.action_id: a for a in manager.action_library}
    flat_map = np.zeros((int(manager.map_cfg["patch_size"]), int(manager.map_cfg["patch_size"])), dtype=np.float32)
    action_ids = ["a0", "a1", "a2"]
    mu = 0.8
    rng = np.random.default_rng(seed)

    versions = [
        {"name": "A_current_cmd", "cmd_ramp_sec": 0.0, "label_start_sec": 0.0},
        {"name": "B_ramp0p3_label_after_ramp", "cmd_ramp_sec": 0.3, "label_start_sec": 0.3},
    ]
    all_rows: list[dict[str, Any]] = []

    for ver in versions:
        print(
            "[exp3] start "
            f"version={ver['name']} ramp={ver['cmd_ramp_sec']:.2f}s label_start={ver['label_start_sec']:.2f}s",
            flush=True,
        )
        sim_cfg = copy.deepcopy(manager.sim_cfg)
        if backend == "ros_gz":
            ros = sim_cfg.setdefault("ros_gz", {})
            ros["model_name"] = "ml25d_exp3_vehicle"
            ros["log_dir"] = str((output_dir / f"runner_logs_exp3_{ver['name']}").resolve())
        runner = make_runner(backend, sim_cfg)
        try:
            for v_alias, v_id in vehicle_alias.items():
                vehicle = vehicle_map[v_id]
                for action_id in action_ids:
                    action = action_map[action_id]
                    for k in range(repeats):
                        print(
                            "[exp3] run "
                            f"version={ver['name']} vehicle={v_alias} action={action_id} rep={k + 1}/{repeats}",
                            flush=True,
                        )
                        sample_seed = int(rng.integers(0, 2**31 - 1))
                        context = SimulationContext(
                            heightmap=flat_map,
                            heading_rad=0.0,
                            vehicle=vehicle,
                            action=action,
                            friction_mu=mu,
                            motion_model="skid",
                            sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
                            duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                            settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
                            cmd_ramp_sec=float(ver["cmd_ramp_sec"]),
                            scene_id=f"exp3_{ver['name']}_{v_alias}_{action_id}_{k}",
                        )
                        traj, err = _run_context_with_retry(
                            runner=runner,
                            context=context,
                            seed=sample_seed,
                            max_retry=max_retry,
                        )
                        if traj is None:
                            print(
                                "[exp3] runtime_failure "
                                f"version={ver['name']} vehicle={v_alias} action={action_id} rep={k + 1}: {err}",
                                flush=True,
                            )
                            all_rows.append(
                                {
                                    "version": ver["name"],
                                    "vehicle_type": v_alias,
                                    "action_id": action_id,
                                    "repeat_idx": k,
                                    "status": "runtime_failure",
                                    "error": err,
                                }
                            )
                            continue

                        eval_traj = _slice_trajectory(traj, float(ver["label_start_sec"]))
                        labels, _ = manager.label_extractor.compute_labels(eval_traj, vehicle, action)
                        reasons = _sample_fail_reasons(labels, manager.label_extractor.thresholds)
                        all_rows.append(
                            {
                                "version": ver["name"],
                                "vehicle_type": v_alias,
                                "action_id": action_id,
                                "repeat_idx": k,
                                "status": "ok",
                                "y_fail": float(labels.y_fail),
                                "q_roll": float(labels.q_roll),
                                "q_pitch": float(labels.q_pitch),
                                "q_slip": float(labels.q_slip),
                                "q_lift": float(labels.q_lift),
                                "p_bottom": float(labels.p_bottom),
                                "p_stuck": float(labels.p_stuck),
                                "fail_reasons": reasons,
                                "translation_progress": float(eval_traj.completed_displacement_m / max(action.delta_s_m, 1e-6)),
                            }
                        )
                        print(
                            "[exp3] done "
                            f"version={ver['name']} vehicle={v_alias} action={action_id} rep={k + 1} "
                            f"fail={float(labels.y_fail):.1f} reasons={reasons}",
                            flush=True,
                        )
        finally:
            try:
                runner.shutdown()
            except Exception:
                pass

    summary_rows = []
    by_action_rows = []
    by_vehicle_rows = []
    for ver in [v["name"] for v in versions]:
        sub = [r for r in all_rows if r["version"] == ver and r["status"] == "ok"]
        runtime_fail = [r for r in all_rows if r["version"] == ver and r["status"] != "ok"]
        fail_rate = float(np.mean(np.array([float(r["y_fail"]) >= 0.5 for r in sub], dtype=bool))) if sub else float("nan")
        reason_counter: Counter[str] = Counter()
        for r in sub:
            if float(r["y_fail"]) >= 0.5:
                reason_counter.update(str(x) for x in r.get("fail_reasons", []))
        summary_rows.append(
            {
                "version": ver,
                "valid_count": len(sub),
                "runtime_failure_count": len(runtime_fail),
                "fail_rate": fail_rate,
                "q_roll_mean": float(np.mean([float(r["q_roll"]) for r in sub])) if sub else float("nan"),
                "q_pitch_mean": float(np.mean([float(r["q_pitch"]) for r in sub])) if sub else float("nan"),
                "q_slip_mean": float(np.mean([float(r["q_slip"]) for r in sub])) if sub else float("nan"),
                "p_bottom_mean": float(np.mean([float(r["p_bottom"]) for r in sub])) if sub else float("nan"),
                "p_stuck_mean": float(np.mean([float(r["p_stuck"]) for r in sub])) if sub else float("nan"),
                "fail_reason_distribution": dict(sorted(reason_counter.items())),
            }
        )
        for action_id in action_ids:
            ss = [r for r in sub if r["action_id"] == action_id]
            by_action_rows.append(
                {
                    "version": ver,
                    "action_id": action_id,
                    "count": len(ss),
                    "fail_rate": float(np.mean(np.array([float(r["y_fail"]) >= 0.5 for r in ss], dtype=bool))) if ss else float("nan"),
                    "q_roll_mean": float(np.mean([float(r["q_roll"]) for r in ss])) if ss else float("nan"),
                    "q_pitch_mean": float(np.mean([float(r["q_pitch"]) for r in ss])) if ss else float("nan"),
                    "q_slip_mean": float(np.mean([float(r["q_slip"]) for r in ss])) if ss else float("nan"),
                    "p_bottom_mean": float(np.mean([float(r["p_bottom"]) for r in ss])) if ss else float("nan"),
                    "p_stuck_mean": float(np.mean([float(r["p_stuck"]) for r in ss])) if ss else float("nan"),
                }
            )
        for v_alias in vehicle_alias:
            ss = [r for r in sub if r["vehicle_type"] == v_alias]
            by_vehicle_rows.append(
                {
                    "version": ver,
                    "vehicle_type": v_alias,
                    "count": len(ss),
                    "fail_rate": float(np.mean(np.array([float(r["y_fail"]) >= 0.5 for r in ss], dtype=bool))) if ss else float("nan"),
                    "q_roll_mean": float(np.mean([float(r["q_roll"]) for r in ss])) if ss else float("nan"),
                    "q_pitch_mean": float(np.mean([float(r["q_pitch"]) for r in ss])) if ss else float("nan"),
                    "q_slip_mean": float(np.mean([float(r["q_slip"]) for r in ss])) if ss else float("nan"),
                    "p_bottom_mean": float(np.mean([float(r["p_bottom"]) for r in ss])) if ss else float("nan"),
                    "p_stuck_mean": float(np.mean([float(r["p_stuck"]) for r in ss])) if ss else float("nan"),
                }
            )

    return {
        "summary": summary_rows,
        "by_action": by_action_rows,
        "by_vehicle": by_vehicle_rows,
        "rows": all_rows,
    }


def main() -> int:
    args = parse_args()
    samples_path = args.samples_jsonl.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = _load_jsonl(samples_path)
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root, config_dir=None)

    exp1 = _exp1_offline_tables(rows)
    exp2 = _exp2_replay_and_timeseries(
        rows=rows,
        manager=manager,
        backend=args.backend,
        output_dir=output_dir,
        max_retry=int(args.exp2_max_retry),
    )
    exp3 = _exp3_golden_flat_ab(
        manager=manager,
        backend=args.backend,
        output_dir=output_dir,
        max_retry=int(args.exp3_max_retry),
        repeats=int(args.golden_repeats),
        seed=int(args.seed),
    )

    payload = {
        "exp1_offline": exp1,
        "exp2_replay": exp2,
        "exp3_golden_ab": exp3,
    }
    json_path = output_dir / "flat_root_cause_diagnostic.json"
    md_path = output_dir / "flat_root_cause_diagnostic.md"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    sections: list[str] = []
    sections.append("## Experiment 1: Existing Flat Sample Offline Audit")
    sections.append(
        _table(
            [
                {
                    "sample_count": exp1["sample_count"],
                    "fail_count": exp1["fail_count"],
                    "fail_rate": exp1["fail_rate"],
                }
            ],
            ["sample_count", "fail_count", "fail_rate"],
        )
    )
    sections.append("")
    reason_rows = [{"fail_reason": k, "count": v} for k, v in exp1["fail_reason_distribution"].items()]
    sections.append("### Fail Reason Distribution")
    sections.append(_table(reason_rows, ["fail_reason", "count"]))
    sections.append("")
    sections.append("### q Distribution (Fail vs Pass)")
    sections.append(
        _table(
            exp1["q_distribution"],
            [
                "metric",
                "fail_mean",
                "fail_p50",
                "fail_p95",
                "fail_max",
                "fail_frac_eq_1",
                "pass_mean",
                "pass_p50",
                "pass_p95",
                "pass_max",
                "pass_frac_eq_1",
            ],
        )
    )

    co = exp1["reason_cooccurrence"]
    co_rows = []
    for a in ["roll", "pitch", "slip", "lift", "bottom", "stuck"]:
        row = {"reason": a}
        for b in ["roll", "pitch", "slip", "lift", "bottom", "stuck"]:
            row[b] = co[a][b]
        co_rows.append(row)
    sections.append("")
    sections.append("### Fail Reason Co-occurrence")
    sections.append(_table(co_rows, ["reason", "roll", "pitch", "slip", "lift", "bottom", "stuck"]))

    sections.append("")
    sections.append("## Experiment 2: Replay 12 Fail + Pass Controls with Time Series")
    sections.append(
        _table(
            [
                {
                    "selected_case_count": exp2["selected_case_count"],
                    "ok_count": exp2["ok_count"],
                    "runtime_failure_count": exp2["runtime_failure_count"],
                    "first_trigger_in_0p3s_count": exp2["first_trigger_in_0p3s_count"],
                }
            ],
            ["selected_case_count", "ok_count", "runtime_failure_count", "first_trigger_in_0p3s_count"],
        )
    )
    sections.append("")
    trig_rows = [{"trigger_reason": k, "count": v} for k, v in exp2["first_trigger_reason_distribution"].items()]
    sections.append("### First Trigger Reason Distribution")
    sections.append(_table(trig_rows, ["trigger_reason", "count"]))
    sections.append("")
    ok_rows = [r for r in exp2["rows"] if r["status"] == "ok"]
    sections.append("### Replay Case Summary")
    sections.append(
        _table(
            ok_rows,
            [
                "case_id",
                "case_type",
                "vehicle_id",
                "action_id",
                "friction_class",
                "friction_mu",
                "original_fail_reasons",
                "replay_fail_reasons",
                "first_fail_trigger_reason",
                "first_fail_trigger_s",
                "first_trigger_in_0p3s",
                "roll_max_deg",
                "roll_p95_deg",
                "pitch_max_deg",
                "pitch_p95_deg",
                "roll_over_thr_duration_s",
                "pitch_over_thr_duration_s",
                "running_bottom_ratio_final",
                "running_q_lift_final",
                "p_stuck",
                "translation_progress",
                "angular_progress",
                "translation_drift",
            ],
        )
    )

    sections.append("")
    sections.append("## Experiment 3: Golden Flat 45 Samples A/B")
    sections.append("### Version Summary")
    sections.append(
        _table(
            exp3["summary"],
            [
                "version",
                "valid_count",
                "runtime_failure_count",
                "fail_rate",
                "q_roll_mean",
                "q_pitch_mean",
                "q_slip_mean",
                "p_bottom_mean",
                "p_stuck_mean",
                "fail_reason_distribution",
            ],
        )
    )
    sections.append("")
    sections.append("### By Action")
    sections.append(
        _table(
            exp3["by_action"],
            [
                "version",
                "action_id",
                "count",
                "fail_rate",
                "q_roll_mean",
                "q_pitch_mean",
                "q_slip_mean",
                "p_bottom_mean",
                "p_stuck_mean",
            ],
        )
    )
    sections.append("")
    sections.append("### By Vehicle")
    sections.append(
        _table(
            exp3["by_vehicle"],
            [
                "version",
                "vehicle_type",
                "count",
                "fail_rate",
                "q_roll_mean",
                "q_pitch_mean",
                "q_slip_mean",
                "p_bottom_mean",
                "p_stuck_mean",
            ],
        )
    )
    sections.append("")
    sections.append(f"json_path: `{json_path}`")
    sections.append(f"md_path: `{md_path}`")
    md_path.write_text("\n".join(sections) + "\n", encoding="utf-8")

    print(json.dumps({"json": str(json_path), "md": str(md_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
