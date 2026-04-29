#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
import multiprocessing as mp
import os
import queue
import re
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, SampleMetadata
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Flat-only unbiased sanity check (no band balancing)")
    p.add_argument("--config-dir", type=Path, default=None)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--num-samples", type=int, default=300)
    p.add_argument("--seed", type=int, default=20260427)
    p.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    p.add_argument("--num-workers", type=int, default=1, choices=[1, 2, 4, 6, 8])
    p.add_argument("--base-domain", type=int, default=170)
    p.add_argument("--mu", type=float, default=0.8)
    p.add_argument("--max-attempt-multiplier", type=int, default=20)
    p.add_argument("--flush-jsonl-batch", type=int, default=64)
    p.add_argument("--worker-startup-stagger-sec", type=float, default=0.8)
    p.add_argument("--ros-startup-timeout-sec", type=float, default=35.0)
    p.add_argument("--ros-service-timeout-sec", type=float, default=12.0)
    return p.parse_args()


def _resolve_world_sdf_file(package_root: Path, world_path_text: str) -> Path:
    p = Path(world_path_text).expanduser()
    if p.is_absolute() and p.exists():
        return p
    candidates = [Path.cwd() / p, package_root / p, package_root.parent / p]
    for c in candidates:
        if c.exists():
            return c
    return p


def _create_worker_world_file(template_world: Path, worker_dir: Path, world_name: str) -> Path:
    text = template_world.read_text(encoding="utf-8")
    patched = re.sub(r'(<world\s+name=")([^"]+)(")', rf"\1{world_name}\3", text, count=1)
    out = worker_dir / f"{world_name}.sdf"
    out.write_text(patched, encoding="utf-8")
    return out


def _classify_error(message: str) -> str:
    msg = message.lower()
    if "start stability gate failed" in msg:
        return "start_gate_failed"
    if "set_pose service call timed out" in msg:
        return "set_pose_timeout"
    if "world control service call timed out" in msg:
        return "world_control_timeout"
    if "timed out waiting for /world/<name>/set_pose bridge service" in msg:
        return "bridge_startup_timeout"
    if "insufficient odometry samples" in msg:
        return "insufficient_odometry"
    if "odometry discontinuity detected" in msg:
        return "odometry_discontinuity"
    if "insufficient wheel contact observability" in msg:
        return "contact_observability_low"
    if "no wheel contact sensor samples" in msg:
        return "no_wheel_contact_samples"
    if "exited with code" in msg:
        return "process_exited"
    if "already exists" in msg:
        return "entity_name_conflict"
    if "timed out" in msg:
        return "timeout"
    return "runtime_failure"


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


def _put_with_stop(q: mp.Queue, payload: dict[str, Any], stop_event: mp.Event) -> bool:
    while not stop_event.is_set():
        try:
            q.put(payload, timeout=0.5)
            return True
        except queue.Full:
            continue
    return False


def _worker_main(worker_id: int, args_dict: dict[str, Any], stop_event: mp.Event, result_queue: mp.Queue) -> None:
    output_dir = Path(args_dict["output_dir"])
    backend = str(args_dict["backend"])
    config_dir = Path(args_dict["config_dir"]) if args_dict["config_dir"] else None
    mu = float(args_dict["mu"])
    package_root = Path(args_dict["package_root"])
    worker_seed = int(args_dict["seed"]) + 100003 * (worker_id + 1)
    stage_name = str(args_dict["stage_name"])

    worker_dir = output_dir / f"worker_{worker_id}"
    worker_dir.mkdir(parents=True, exist_ok=True)

    os.environ["ROS_DOMAIN_ID"] = str(int(args_dict["base_domain"]) + worker_id)
    os.environ["GZ_PARTITION"] = f"ml25d_flat_sanity_{stage_name}_w{worker_id}"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")
    stagger_sec = float(args_dict.get("worker_startup_stagger_sec", 0.0))
    if stagger_sec > 1e-6:
        time.sleep(max(0.0, worker_id * stagger_sec))

    manager = DatasetManager(package_root=package_root, config_dir=config_dir)
    thresholds = manager.label_extractor.thresholds
    sim_cfg = copy.deepcopy(manager.sim_cfg)

    if backend == "ros_gz":
        ros_cfg = sim_cfg.setdefault("ros_gz", {})
        world_name = f"ml25d_flat_sanity_{stage_name}_w{worker_id}"
        model_name = f"ml25d_flat_vehicle_{stage_name}_w{worker_id}"
        ros_cfg["startup_timeout_sec"] = float(max(args_dict.get("ros_startup_timeout_sec", 35.0), 5.0))
        ros_cfg["service_timeout_sec"] = float(max(args_dict.get("ros_service_timeout_sec", 12.0), 2.0))
        template_world = _resolve_world_sdf_file(package_root, str(ros_cfg.get("world_sdf_file", "worlds/ml25d_empty.sdf")))
        worker_world = _create_worker_world_file(template_world, worker_dir, world_name)
        ros_cfg["world_sdf_file"] = str(worker_world)
        ros_cfg["world_name"] = world_name
        ros_cfg["model_name"] = model_name
        ros_cfg["log_dir"] = str(worker_dir / "runner_logs")

    runner = make_runner(backend, sim_cfg)
    rng = np.random.default_rng(worker_seed)
    patch = int(manager.map_cfg["patch_size"])
    flat_map = np.zeros((patch, patch), dtype=np.float32)

    vehicles = [v.vehicle_id for v in manager.vehicle_library]
    vehicle_map = {v.vehicle_id: v for v in manager.vehicle_library}
    actions = [a for a in manager.action_library if a.action_id in {"a0", "a1", "a2"}]
    actions = sorted(actions, key=lambda a: a.action_id)
    combos = [(vid, act.action_id) for vid in vehicles for act in actions]
    action_map = {a.action_id: a for a in actions}

    attempt_idx = 0
    try:
        while not stop_event.is_set():
            attempt_idx += 1
            combo_idx = (attempt_idx + 13 * worker_id) % max(len(combos), 1)
            vehicle_id, action_id = combos[combo_idx]
            vehicle = vehicle_map[vehicle_id]
            action: ActionPrimitive = action_map[action_id]

            sample_seed = int(rng.integers(0, 2**31 - 1))
            local_rng = np.random.default_rng(sample_seed)
            heading_rad = float(local_rng.uniform(0.0, 2.0 * np.pi))
            motion_model = "skid" if float(local_rng.random()) < 0.5 else "ackermann"
            scene_id = f"{stage_name}_w{worker_id}_a{attempt_idx}"

            context = SimulationContext(
                heightmap=flat_map,
                heading_rad=heading_rad,
                vehicle=vehicle,
                action=action,
                friction_mu=mu,
                motion_model=motion_model,
                sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
                duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
                cmd_ramp_sec=float(manager.sim_cfg.get("cmd_ramp_sec", 0.3)),
                scene_id=scene_id,
            )

            try:
                traj = runner.run(context, local_rng)
            except Exception as exc:
                msg = str(exc)
                payload = {
                    "kind": "invalid",
                    "worker_id": worker_id,
                    "attempt_idx": attempt_idx,
                    "seed": sample_seed,
                    "vehicle_id": vehicle_id,
                    "action_id": action_id,
                    "motion_model": motion_model,
                    "heading_rad": heading_rad,
                    "friction_mu": mu,
                    "reason": _classify_error(msg),
                    "error": msg[:500],
                }
                if not _put_with_stop(result_queue, payload, stop_event):
                    break
                continue

            labels, band = manager.label_extractor.compute_labels(traj, vehicle, action)
            reasons = _sample_fail_reasons(labels, thresholds)
            debug = getattr(runner, "get_last_run_debug", lambda: {})()
            odom_pose_forward_mae = float(debug.get("odom_pose_forward_mae", float("nan")))

            payload = {
                "kind": "valid",
                "worker_id": worker_id,
                "attempt_idx": attempt_idx,
                "seed": sample_seed,
                "vehicle_id": vehicle_id,
                "action_id": action_id,
                "motion_model": motion_model,
                "heading_rad": heading_rad,
                "abs_cos_heading": float(abs(math.cos(heading_rad))),
                "friction_mu": mu,
                "band": band,
                "labels": {
                    "y_fail": float(labels.y_fail),
                    "q_roll": float(labels.q_roll),
                    "q_pitch": float(labels.q_pitch),
                    "q_slip": float(labels.q_slip),
                    "q_lift": float(labels.q_lift),
                    "p_bottom": float(labels.p_bottom),
                    "p_stuck": float(labels.p_stuck),
                },
                "fail_reasons": reasons,
                "debug": {
                    "odom_pose_forward_mae": odom_pose_forward_mae,
                    "odom_linear_abs_max": float(debug.get("odom_linear_abs_max", float("nan"))),
                    "pose_forward_abs_max": float(debug.get("pose_forward_abs_max", float("nan"))),
                    "roll_abs_max_deg": float(debug.get("roll_abs_max_deg", float("nan"))),
                    "pitch_abs_max_deg": float(debug.get("pitch_abs_max_deg", float("nan"))),
                    "roll_abs_max_first_0p5s_deg": float(debug.get("roll_abs_max_first_0p5s_deg", float("nan"))),
                    "pitch_abs_max_first_0p5s_deg": float(debug.get("pitch_abs_max_first_0p5s_deg", float("nan"))),
                    "roll_abs_max_after_0p5s_deg": float(debug.get("roll_abs_max_after_0p5s_deg", float("nan"))),
                    "pitch_abs_max_after_0p5s_deg": float(debug.get("pitch_abs_max_after_0p5s_deg", float("nan"))),
                    "completed_displacement_m": float(debug.get("completed_displacement_m", float("nan"))),
                    "completed_heading_change_deg": float(debug.get("completed_heading_change_deg", float("nan"))),
                    "sample_start_msg_time": float(debug.get("sample_start_msg_time", float("nan"))),
                    "sample_start_recv_time": float(debug.get("sample_start_recv_time", float("nan"))),
                    "odom_msg_time_min": float(debug.get("odom_msg_time_min", float("nan"))),
                    "odom_msg_time_max": float(debug.get("odom_msg_time_max", float("nan"))),
                    "contact_msg_time_min": float(debug.get("contact_msg_time_min", float("nan"))),
                    "contact_msg_time_max": float(debug.get("contact_msg_time_max", float("nan"))),
                },
            }
            if not _put_with_stop(result_queue, payload, stop_event):
                break
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass


def _summary_stats(values: list[float]) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"mean": float("nan"), "p95": float("nan"), "max": float("nan")}
    return {
        "mean": float(np.mean(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def _pearson_corr(x: list[float], y: list[float]) -> float:
    xx = np.asarray(x, dtype=np.float64)
    yy = np.asarray(y, dtype=np.float64)
    mask = np.isfinite(xx) & np.isfinite(yy)
    xx = xx[mask]
    yy = yy[mask]
    if xx.size < 3:
        return float("nan")
    if float(np.std(xx)) < 1e-9 or float(np.std(yy)) < 1e-9:
        return float("nan")
    return float(np.corrcoef(xx, yy)[0, 1])


def _flush_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("a", encoding="utf-8") as fp:
        for row in rows:
            fp.write(json.dumps(row, ensure_ascii=False) + "\n")


def main() -> int:
    args = parse_args()
    if args.num_samples <= 0:
        raise ValueError("num_samples must be positive")
    max_domain = int(args.base_domain) + int(args.num_workers) - 1
    if max_domain > 232:
        raise ValueError(
            f"ROS_DOMAIN_ID out of supported range for FastDDS: "
            f"base_domain={args.base_domain}, num_workers={args.num_workers}, max_domain={max_domain}"
        )

    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root, config_dir=args.config_dir)
    thresholds = manager.label_extractor.thresholds

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    valid_jsonl = output_dir / "flat_only_valid_samples.jsonl"
    invalid_jsonl = output_dir / "flat_only_invalid_samples.jsonl"
    valid_jsonl.write_text("", encoding="utf-8")
    invalid_jsonl.write_text("", encoding="utf-8")

    ctx = mp.get_context("spawn")
    stop_event: mp.Event = ctx.Event()
    result_queue: mp.Queue = ctx.Queue(maxsize=max(8, args.num_workers * 4))

    workers = []
    worker_args = {
        "output_dir": str(output_dir),
        "backend": str(args.backend),
        "config_dir": str(args.config_dir.resolve()) if args.config_dir else None,
        "seed": int(args.seed),
        "base_domain": int(args.base_domain),
        "package_root": str(package_root),
        "mu": float(args.mu),
        "stage_name": f"n{args.num_workers}",
        "worker_startup_stagger_sec": float(args.worker_startup_stagger_sec),
        "ros_startup_timeout_sec": float(args.ros_startup_timeout_sec),
        "ros_service_timeout_sec": float(args.ros_service_timeout_sec),
    }
    for wid in range(int(args.num_workers)):
        p = ctx.Process(target=_worker_main, args=(wid, worker_args, stop_event, result_queue))
        p.start()
        workers.append(p)

    max_attempts = max(int(args.max_attempt_multiplier) * int(args.num_samples), int(args.num_samples))
    accepted = 0
    attempts = 0
    invalid = 0
    valid_rows: list[dict[str, Any]] = []
    invalid_rows: list[dict[str, Any]] = []
    valid_buffer: list[dict[str, Any]] = []
    invalid_buffer: list[dict[str, Any]] = []
    invalid_reason_counter: Counter[str] = Counter()
    worker_attempts: Counter[str] = Counter()
    worker_accepted: Counter[str] = Counter()

    t0 = time.time()
    try:
        while accepted < int(args.num_samples) and attempts < max_attempts:
            try:
                msg = result_queue.get(timeout=1.0)
            except queue.Empty:
                if not any(p.is_alive() for p in workers):
                    raise RuntimeError("all workers exited unexpectedly before collecting enough flat-only samples")
                continue

            attempts += 1
            worker_id = int(msg.get("worker_id", -1))
            worker_attempts[str(worker_id)] += 1

            if msg.get("kind") != "valid":
                invalid += 1
                invalid_reason_counter[str(msg.get("reason", "runtime_failure"))] += 1
                invalid_rows.append(msg)
                invalid_buffer.append(msg)
                if len(invalid_buffer) >= int(args.flush_jsonl_batch):
                    _flush_jsonl(invalid_jsonl, invalid_buffer)
                    invalid_buffer.clear()
                if invalid <= 10 or invalid % 20 == 0:
                    print(
                        "[flat-only] invalid "
                        f"attempts={attempts} invalid={invalid} accepted={accepted}/{args.num_samples} "
                        f"reason={msg.get('reason', 'runtime_failure')}",
                        flush=True,
                    )
                continue

            sid = accepted
            accepted += 1
            worker_accepted[str(worker_id)] += 1
            row = dict(msg)
            row["sample_id"] = sid
            valid_rows.append(row)
            valid_buffer.append(row)
            if len(valid_buffer) >= int(args.flush_jsonl_batch):
                _flush_jsonl(valid_jsonl, valid_buffer)
                valid_buffer.clear()

            if accepted <= 20 or accepted % 20 == 0 or accepted == int(args.num_samples):
                print(
                    "[flat-only] accepted "
                    f"{accepted}/{args.num_samples} attempts={attempts} invalid={invalid}",
                    flush=True,
                )
    finally:
        stop_event.set()
        for p in workers:
            p.join(timeout=20.0)
        for p in workers:
            if p.is_alive():
                p.terminate()
                p.join(timeout=5.0)
        _flush_jsonl(valid_jsonl, valid_buffer)
        _flush_jsonl(invalid_jsonl, invalid_buffer)

    labels_keys = ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]
    metrics_summary = {
        key: _summary_stats([float(r["labels"][key]) for r in valid_rows]) for key in labels_keys
    }
    y_fail = np.asarray([float(r["labels"]["y_fail"]) for r in valid_rows], dtype=np.float64)
    fail_rate = float(np.mean(y_fail >= 0.5)) if y_fail.size > 0 else float("nan")

    fail_reason_counter: Counter[str] = Counter()
    for r in valid_rows:
        if float(r["labels"]["y_fail"]) < 0.5:
            continue
        for reason in r.get("fail_reasons", []):
            fail_reason_counter[str(reason)] += 1

    slip_fail_rate = float(
        np.mean([float(r["labels"]["q_slip"]) > float(thresholds.slip_fail_threshold) for r in valid_rows])
    ) if valid_rows else float("nan")
    bottom_fail_rate = float(
        np.mean([float(r["labels"]["p_bottom"]) > float(thresholds.bottom_fail_threshold) for r in valid_rows])
    ) if valid_rows else float("nan")
    stuck_fail_rate = float(
        np.mean([float(r["labels"]["p_stuck"]) >= 1.0 for r in valid_rows])
    ) if valid_rows else float("nan")

    abs_cos = [float(r.get("abs_cos_heading", float("nan"))) for r in valid_rows]
    q_slip = [float(r["labels"]["q_slip"]) for r in valid_rows]
    slip_heading_corr = _pearson_corr(abs_cos, q_slip)

    odom_pose_mae = [
        float(r.get("debug", {}).get("odom_pose_forward_mae", float("nan")))
        for r in valid_rows
    ]
    odom_pose_mae_stats = _summary_stats(odom_pose_mae)

    vehicle_counts: Counter[str] = Counter(str(r["vehicle_id"]) for r in valid_rows)
    action_counts: Counter[str] = Counter(str(r["action_id"]) for r in valid_rows)
    motion_counts: Counter[str] = Counter(str(r["motion_model"]) for r in valid_rows)

    elapsed_sec = float(max(time.time() - t0, 1e-6))
    summary = {
        "config": {
            "num_samples_target": int(args.num_samples),
            "backend": str(args.backend),
            "num_workers": int(args.num_workers),
            "mu": float(args.mu),
            "seed": int(args.seed),
            "created_at_utc": datetime.now(tz=timezone.utc).isoformat(),
            "flat_definition": {
                "terrain_class": "flat_absolute",
                "slope_deg": [0.0, 0.0],
                "cross_deg": [0.0, 0.0],
                "wave_amp_m": [0.0, 0.0],
                "bump_count": [0, 0],
                "pit_count": [0, 0],
                "step_height_m": [0.0, 0.0],
                "noise_std_m": [0.0, 0.0],
            },
        },
        "counts": {
            "attempts_total": int(attempts),
            "accepted_valid_samples": int(len(valid_rows)),
            "invalid_attempts": int(invalid),
            "invalid_sample_rate": float(invalid / max(attempts, 1)),
            "worker_attempts": dict(worker_attempts),
            "worker_accepted": dict(worker_accepted),
            "vehicle_counts": dict(vehicle_counts),
            "action_counts": dict(action_counts),
            "motion_model_counts": dict(motion_counts),
            "invalid_reason_distribution": dict(sorted(invalid_reason_counter.items())),
        },
        "performance": {
            "elapsed_sec": elapsed_sec,
            "accepted_samples_per_min": float(60.0 * len(valid_rows) / elapsed_sec),
        },
        "flat_metrics": {
            "flat_fail_rate": fail_rate,
            "fail_reason_distribution": dict(sorted(fail_reason_counter.items())),
            "slip_fail_rate": slip_fail_rate,
            "bottom_fail_rate": bottom_fail_rate,
            "stuck_fail_rate": stuck_fail_rate,
            "q_roll": metrics_summary["q_roll"],
            "q_pitch": metrics_summary["q_pitch"],
            "q_slip": metrics_summary["q_slip"],
            "q_lift": metrics_summary["q_lift"],
            "p_bottom": metrics_summary["p_bottom"],
            "p_stuck": metrics_summary["p_stuck"],
            "q_slip_vs_abs_cos_heading_corr": slip_heading_corr,
            "odom_pose_forward_mae": odom_pose_mae_stats,
        },
    }

    report_json = output_dir / "flat_only_sanity_report.json"
    report_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    md = []
    md.append("## Flat-only Sanity Report")
    md.append("")
    md.append("| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |")
    md.append("| --- | --- | --- | --- | --- | --- |")
    md.append(
        f"| {int(args.num_workers)} | {int(args.num_samples)} | {len(valid_rows)} | {invalid} | "
        f"{summary['counts']['invalid_sample_rate']:.4f} | {fail_rate:.4f} |"
    )
    md.append("")
    md.append("| metric | mean | p95 | max |")
    md.append("| --- | --- | --- | --- |")
    for key in ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]:
        cur = summary["flat_metrics"][key]
        md.append(f"| {key} | {cur['mean']:.6f} | {cur['p95']:.6f} | {cur['max']:.6f} |")
    md.append("")
    md.append(f"- q_slip_vs_abs_cos_heading_corr: {summary['flat_metrics']['q_slip_vs_abs_cos_heading_corr']:.6f}")
    opm = summary["flat_metrics"]["odom_pose_forward_mae"]
    md.append(f"- odom_pose_forward_mae mean/p95/max: {opm['mean']:.6f} / {opm['p95']:.6f} / {opm['max']:.6f}")
    md.append(f"- slip_fail_rate: {summary['flat_metrics']['slip_fail_rate']:.6f}")
    md.append(f"- bottom_fail_rate: {summary['flat_metrics']['bottom_fail_rate']:.6f}")
    md.append(f"- stuck_fail_rate: {summary['flat_metrics']['stuck_fail_rate']:.6f}")
    md.append("")
    md.append("### Fail Reasons")
    md.append("| reason | count |")
    md.append("| --- | --- |")
    if fail_reason_counter:
        for reason, cnt in sorted(fail_reason_counter.items(), key=lambda kv: (-kv[1], kv[0])):
            md.append(f"| {reason} | {cnt} |")
    else:
        md.append("| (none) | 0 |")
    md.append("")
    md.append("### Coverage")
    md.append(f"- vehicle_counts: {json.dumps(dict(vehicle_counts), ensure_ascii=False)}")
    md.append(f"- action_counts: {json.dumps(dict(action_counts), ensure_ascii=False)}")
    md.append(f"- motion_model_counts: {json.dumps(dict(motion_counts), ensure_ascii=False)}")
    md_path = output_dir / "flat_only_sanity_report.md"
    md_path.write_text("\n".join(md) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "report_json": str(report_json),
                "report_md": str(md_path),
                "valid_jsonl": str(valid_jsonl),
                "invalid_jsonl": str(invalid_jsonl),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
