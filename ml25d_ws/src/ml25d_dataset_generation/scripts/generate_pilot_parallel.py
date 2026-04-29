#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import multiprocessing as mp
import os
import queue
import re
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import time
from typing import Any

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, SampleMetadata
from ml25d_dataset_generation.config_loader import weighted_table
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner

ROS_DOMAIN_ID_MAX = 232
ROS_DOMAIN_ID_SPAN = ROS_DOMAIN_ID_MAX + 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parallel pilot dataset generation on ros_gz backend")
    parser.add_argument("--config-dir", type=Path, default=None, help="Optional config directory")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory")
    parser.add_argument("--num-samples", type=int, default=1000, help="Number of accepted samples")
    parser.add_argument("--seed", type=int, default=20260426, help="Global seed")
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    parser.add_argument(
        "--num-workers",
        type=int,
        default=4,
        help=(
            "Parallel workers. Supports values >8. "
            f"Unique ROS domains are assigned modulo {ROS_DOMAIN_ID_SPAN}."
        ),
    )
    parser.add_argument(
        "--disable-balance",
        action="store_true",
        help="Disable safe/fail/critical band balancing and accept the first N valid samples.",
    )
    parser.add_argument(
        "--balance-mode",
        type=str,
        default="vehicle_band",
        choices=["band", "vehicle_band"],
        help=(
            "Balancing strategy when balance is enabled: "
            "'band' enforces global safe/fail/critical targets; "
            "'vehicle_band' enforces per-vehicle safe/fail/critical targets."
        ),
    )
    parser.add_argument(
        "--stall-attempt-window",
        type=int,
        default=3000,
        help=(
            "Accepted-sample stagnation window (in attempts) before relaxing balance constraints. "
            "Only used when balance-mode=vehicle_band."
        ),
    )
    parser.add_argument("--base-domain", type=int, default=180, help="Base ROS_DOMAIN_ID")
    parser.add_argument(
        "--result-queue-mult",
        type=int,
        default=12,
        help="Result queue size multiplier (queue_size = max(32, num_workers * mult)).",
    )
    parser.add_argument("--flush-batch-size", type=int, default=200, help="Samples per HDF5 batch")
    parser.add_argument("--max-attempt-multiplier", type=int, default=80, help="Max attempts = num_samples * multiplier")
    parser.add_argument(
        "--worker-startup-stagger-sec",
        type=float,
        default=0.8,
        help="Stagger worker startup to reduce ros_gz service storms under high parallelism.",
    )
    parser.add_argument(
        "--ros-startup-timeout-sec",
        type=float,
        default=35.0,
        help="Override ros_gz startup timeout per worker.",
    )
    parser.add_argument(
        "--ros-service-timeout-sec",
        type=float,
        default=12.0,
        help="Override ros_gz service timeout per worker.",
    )
    parser.add_argument(
        "--run-tag",
        type=str,
        default="",
        help="Optional run tag used to isolate world/model/partition names across parallel runs.",
    )
    parser.add_argument(
        "--terrain-compensation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable adaptive terrain sampling to compensate accepted-distribution drift.",
    )
    parser.add_argument(
        "--terrain-comp-strength",
        type=float,
        default=1.8,
        help="Adaptive terrain compensation strength (>0).",
    )
    parser.add_argument(
        "--terrain-comp-warmup",
        type=int,
        default=40,
        help="Accepted-sample warmup per worker before enabling terrain compensation.",
    )
    parser.add_argument(
        "--terrain-comp-min-mult",
        type=float,
        default=0.30,
        help="Lower bound of terrain compensation multiplier.",
    )
    parser.add_argument(
        "--terrain-comp-max-mult",
        type=float,
        default=5.00,
        help="Upper bound of terrain compensation multiplier.",
    )
    return parser.parse_args()


def _worker_domain_id(base_domain: int, worker_id: int) -> int:
    return int((int(base_domain) + int(worker_id)) % ROS_DOMAIN_ID_SPAN)


def _adaptive_terrain_probs(
    *,
    base_probs: np.ndarray,
    accepted_counts: np.ndarray,
    accepted_total: int,
    enabled: bool,
    strength: float,
    warmup: int,
    min_mult: float,
    max_mult: float,
) -> np.ndarray:
    if (not enabled) or accepted_total < max(int(warmup), 0):
        return base_probs
    if strength <= 1e-9:
        return base_probs

    expected = np.maximum(base_probs * float(accepted_total), 1e-6)
    deficit = expected - accepted_counts.astype(np.float64)
    z = deficit / np.sqrt(expected + 1.0)
    multipliers = np.exp(float(strength) * z)
    multipliers = np.clip(multipliers, float(min_mult), float(max_mult))
    adjusted = base_probs * multipliers
    total = float(np.sum(adjusted))
    if not np.isfinite(total) or total <= 1e-12:
        return base_probs
    return adjusted / total


def _resolve_world_sdf_file(package_root: Path, world_path_text: str) -> Path:
    p = Path(world_path_text).expanduser()
    if p.is_absolute() and p.exists():
        return p
    candidates = [
        Path.cwd() / p,
        package_root / p,
        package_root.parent / p,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return p


def _create_worker_world_file(template_world: Path, worker_dir: Path, world_name: str) -> Path:
    text = template_world.read_text(encoding="utf-8")
    patched = re.sub(r'(<world\s+name=")([^"]+)(")', rf"\1{world_name}\3", text, count=1)
    out_path = worker_dir / f"{world_name}.sdf"
    out_path.write_text(patched, encoding="utf-8")
    return out_path


def _is_invalid_contact_error(message: str) -> bool:
    msg = message.lower()
    return ("no wheel contact sensor samples" in msg) or ("insufficient wheel contact observability" in msg)


def _classify_error(message: str) -> str:
    msg = message.lower()
    if _is_invalid_contact_error(message):
        return "no_wheel_contact_samples"
    if "start stability gate failed" in msg:
        return "start_gate_failed"
    if "timed out waiting for fresh odometry after pose reset" in msg:
        return "reset_odom_timeout"
    if "set_pose service call timed out" in msg:
        return "set_pose_timeout"
    if "world control service call timed out" in msg:
        return "world_control_timeout"
    if "timed out waiting for /world/<name>/set_pose bridge service" in msg:
        return "bridge_startup_timeout"
    if "ros gazebo python interfaces are unavailable" in msg:
        return "ros_gz_python_unavailable"
    if "insufficient odometry samples" in msg:
        return "insufficient_odometry"
    if "odometry discontinuity detected" in msg:
        return "odometry_discontinuity"
    if "exited with code" in msg:
        return "process_exited"
    if "already exists" in msg:
        return "entity_name_conflict"
    if "segmentation fault" in msg:
        return "segfault"
    if "timed out" in msg:
        return "timeout"
    return "runtime_failure"


def _compute_motion_progress(trajectory, action: ActionPrimitive) -> tuple[float, float, float, float]:
    # translation_progress: only for translation actions (a0/a1/a2)
    # angular_progress: only for turn actions (a3/a4), clipped to [0, 1] to avoid
    # progress outliers caused purely by over-rotation definition.
    translation_progress = float("nan")
    angular_progress = float("nan")
    translation_drift = float(trajectory.completed_displacement_m)

    if action.delta_s_m > 1e-4:
        translation_progress = float(trajectory.completed_displacement_m / max(action.delta_s_m, 1e-6))
        progress_ratio = translation_progress
    else:
        target_yaw = abs(np.deg2rad(action.delta_psi_deg))
        if target_yaw > 1e-6:
            raw_angular_progress = float(trajectory.completed_heading_change_rad / target_yaw)
            angular_progress = float(np.clip(raw_angular_progress, 0.0, 1.0))
        else:
            angular_progress = 0.0
        progress_ratio = angular_progress

    return progress_ratio, translation_progress, angular_progress, translation_drift


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


def _spawn_prefilter_ok(runner, terrain_map: np.ndarray, vehicle, heading_rad: float) -> bool:
    # Keep generation logic unchanged; only skip obviously unstable starts before expensive Gazebo rollout.
    required = [
        "_estimate_initial_vehicle_pose",
        "_compute_wheel_clearance_center_sample",
        "_compute_chassis_min_clearance",
    ]
    if not all(hasattr(runner, name) for name in required):
        return True
    try:
        z_pos, roll_rad, pitch_rad = runner._estimate_initial_vehicle_pose(  # type: ignore[attr-defined]
            heightmap=terrain_map,
            vehicle=vehicle,
            heading_rad=float(heading_rad),
        )
        if not np.isfinite([z_pos, roll_rad, pitch_rad]).all():
            return False
        if abs(float(np.rad2deg(roll_rad))) > 35.0 or abs(float(np.rad2deg(pitch_rad))) > 35.0:
            return False

        one_pos = np.array([[0.0, 0.0]], dtype=np.float32)
        one_z = np.array([float(z_pos)], dtype=np.float32)
        one_roll = np.array([float(roll_rad)], dtype=np.float32)
        one_pitch = np.array([float(pitch_rad)], dtype=np.float32)
        one_yaw = np.array([float(heading_rad)], dtype=np.float32)
        wheel_clear = runner._compute_wheel_clearance_center_sample(  # type: ignore[attr-defined]
            heightmap=terrain_map,
            vehicle=vehicle,
            positions_xy=one_pos,
            positions_z=one_z,
            roll=one_roll,
            pitch=one_pitch,
            yaw=one_yaw,
        )[0]
        if not np.isfinite(wheel_clear).all():
            return False
        if float(np.min(wheel_clear)) < -0.10:
            return False
        if float(np.max(wheel_clear)) > 0.20:
            return False

        chassis_clear = float(
            runner._compute_chassis_min_clearance(  # type: ignore[attr-defined]
                heightmap=terrain_map,
                vehicle=vehicle,
                positions_xy=one_pos,
                positions_z=one_z,
                roll=one_roll,
                pitch=one_pitch,
                yaw=one_yaw,
            )[0]
        )
        if not np.isfinite(chassis_clear) or chassis_clear < -0.03:
            return False
        return True
    except Exception:
        return False


def _worker_main(
    worker_id: int,
    args_dict: dict[str, Any],
    stop_event: mp.Event,
    result_queue: mp.Queue,
) -> None:
    output_dir = Path(args_dict["output_dir"])
    backend = str(args_dict["backend"])
    config_dir = Path(args_dict["config_dir"]) if args_dict["config_dir"] else None
    worker_seed = int(args_dict["seed"]) + 100003 * (worker_id + 1)
    worker_dir = output_dir / f"worker_{worker_id}"
    worker_dir.mkdir(parents=True, exist_ok=True)

    run_tag = str(args_dict.get("run_tag", ""))
    os.environ["ROS_DOMAIN_ID"] = str(_worker_domain_id(int(args_dict["base_domain"]), int(worker_id)))
    os.environ["GZ_PARTITION"] = f"ml25d_pilot_{run_tag}_worker_{worker_id}"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")
    stagger_sec = float(args_dict.get("worker_startup_stagger_sec", 0.0))
    if stagger_sec > 1e-6:
        time.sleep(max(0.0, worker_id * stagger_sec))

    package_root = Path(args_dict["package_root"])
    manager = DatasetManager(package_root=package_root, config_dir=config_dir)

    sim_cfg = copy.deepcopy(manager.sim_cfg)
    if backend == "ros_gz":
        ros_cfg = sim_cfg.setdefault("ros_gz", {})
        world_name = f"ml25d_pilot_{run_tag}_w{worker_id}"
        model_name = f"ml25d_vehicle_{run_tag}_w{worker_id}"
        ros_cfg["startup_timeout_sec"] = float(max(args_dict.get("ros_startup_timeout_sec", 35.0), 5.0))
        ros_cfg["service_timeout_sec"] = float(max(args_dict.get("ros_service_timeout_sec", 12.0), 2.0))
        world_template = _resolve_world_sdf_file(
            package_root,
            str(ros_cfg.get("world_sdf_file", "worlds/ml25d_empty.sdf")),
        )
        worker_world = _create_worker_world_file(world_template, worker_dir, world_name)
        ros_cfg["world_sdf_file"] = str(worker_world)
        ros_cfg["world_name"] = world_name
        ros_cfg["model_name"] = model_name
        ros_cfg["log_dir"] = str(worker_dir / "runner_logs")

    runner = make_runner(backend, sim_cfg)
    rng = np.random.default_rng(worker_seed)

    terrain_rows, terrain_probs = weighted_table(manager.terrain_cfg["terrain"]["classes"])
    terrain_base_probs = np.asarray(terrain_probs, dtype=np.float64)
    terrain_base_probs = terrain_base_probs / max(float(np.sum(terrain_base_probs)), 1e-12)
    terrain_accepted_counts = np.zeros(len(terrain_rows), dtype=np.int64)
    terrain_accepted_total = 0
    terrain_comp_enabled = bool(args_dict.get("terrain_compensation", True))
    terrain_comp_strength = float(args_dict.get("terrain_comp_strength", 1.8))
    terrain_comp_warmup = int(args_dict.get("terrain_comp_warmup", 40))
    terrain_comp_min_mult = float(args_dict.get("terrain_comp_min_mult", 0.30))
    terrain_comp_max_mult = float(args_dict.get("terrain_comp_max_mult", 5.00))
    thresholds = manager.label_extractor.thresholds
    attempt_id = 0
    prefilter_rejects = 0
    try:
        while not stop_event.is_set():
            attempt_id += 1
            sample_seed = int(rng.integers(0, 2**31 - 1))
            local_rng = np.random.default_rng(sample_seed)
            heading_rad = float(local_rng.uniform(0.0, 2.0 * np.pi))

            terrain_sample_probs = _adaptive_terrain_probs(
                base_probs=terrain_base_probs,
                accepted_counts=terrain_accepted_counts,
                accepted_total=terrain_accepted_total,
                enabled=terrain_comp_enabled,
                strength=terrain_comp_strength,
                warmup=terrain_comp_warmup,
                min_mult=terrain_comp_min_mult,
                max_mult=terrain_comp_max_mult,
            )
            terrain_idx = int(local_rng.choice(len(terrain_rows), p=terrain_sample_probs))
            terrain_class = terrain_rows[terrain_idx]
            terrain_name = str(terrain_class["name"])
            terrain = manager.terrain_generator.generate(local_rng, terrain_class, travel_heading_rad=heading_rad)

            vehicle = manager._sample_vehicle(local_rng)
            motion_model = "skid"
            action = manager._sample_action(local_rng, motion_model)

            friction_rows, friction_probs = weighted_table(manager.friction_cfg["friction"]["classes"])
            friction_idx = int(local_rng.choice(len(friction_rows), p=friction_probs))
            friction_class = friction_rows[friction_idx]
            friction_name = str(friction_class["name"])
            mu_lo, mu_hi = friction_class["mu_range"]
            friction_mu = float(local_rng.uniform(mu_lo, mu_hi))

            if not _spawn_prefilter_ok(runner, terrain.heightmap, vehicle, heading_rad):
                # Skip before simulation when start pose is clearly unstable.
                prefilter_rejects += 1
                continue
            scene_id = f"{run_tag}_w{worker_id}_a{attempt_id}"
            context = SimulationContext(
                heightmap=terrain.heightmap,
                heading_rad=heading_rad,
                vehicle=vehicle,
                action=action,
                friction_mu=friction_mu,
                motion_model=motion_model,
                sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
                duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
                cmd_ramp_sec=float(manager.sim_cfg.get("cmd_ramp_sec", 0.3)),
                scene_id=scene_id,
            )

            try:
                trajectory = runner.run(context, local_rng)
            except Exception as exc:
                msg = str(exc)
                reason = _classify_error(msg)
                if reason == "start_gate_failed":
                    # Initialization instability is handled as local retry, not dataset invalid sample.
                    continue
                payload = {
                    "kind": "invalid",
                    "worker_id": worker_id,
                    "seed": sample_seed,
                    "reason": reason,
                    "error": msg[:500],
                }
                if not _put_with_stop(result_queue, payload, stop_event):
                    break
                continue

            labels, band = manager.label_extractor.compute_labels(trajectory, vehicle, action)
            metadata = SampleMetadata(
                sample_id=-1,
                seed=sample_seed,
                terrain_class=terrain_name,
                friction_class=friction_name,
                vehicle_id=vehicle.vehicle_id,
                action_id=action.action_id,
                action_name=action.name,
                motion_model=motion_model,
                heading_rad=heading_rad,
            )
            sample = manager.packager.create_sample(
                heightmap=terrain.heightmap,
                heading_rad=heading_rad,
                vehicle=vehicle,
                action=action,
                friction_mu=friction_mu,
                labels=labels,
                band=band,
                metadata=metadata,
            )
            sample["metadata"]["terrain_parameters"] = terrain.parameters
            sample["metadata"]["friction_mu"] = friction_mu

            progress_ratio, translation_progress, angular_progress, translation_drift = _compute_motion_progress(
                trajectory,
                action,
            )
            fail_reasons = _sample_fail_reasons(labels, thresholds)

            payload = {
                "kind": "valid",
                "worker_id": worker_id,
                "seed": sample_seed,
                "band": band,
                "counts_info": {
                    "terrain_class": terrain_name,
                    "vehicle_id": vehicle.vehicle_id,
                    "friction_class": friction_name,
                    "motion_model": motion_model,
                    "action_id": action.action_id,
                },
                "sample": sample,
                "labels": {
                    "y_fail": float(labels.y_fail),
                    "q_roll": float(labels.q_roll),
                    "q_pitch": float(labels.q_pitch),
                    "q_slip": float(labels.q_slip),
                    "q_lift": float(labels.q_lift),
                    "p_bottom": float(labels.p_bottom),
                    "p_stuck": float(labels.p_stuck),
                },
                "progress_ratio": float(progress_ratio),
                "translation_progress": float(translation_progress),
                "angular_progress": float(angular_progress),
                "translation_drift": float(translation_drift),
                "fail_reasons": fail_reasons,
            }
            terrain_accepted_counts[terrain_idx] += 1
            terrain_accepted_total += 1
            if not _put_with_stop(result_queue, payload, stop_event):
                break
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("a", encoding="utf-8") as fp:
        for row in rows:
            fp.write(json.dumps(row, ensure_ascii=False) + "\n")


def _ordered_vehicle_ids(manager: DatasetManager) -> list[str]:
    ids: list[str] = []
    for vehicle in manager.vehicle_library:
        vid = str(vehicle.vehicle_id)
        if vid not in ids:
            ids.append(vid)
    return ids


def _split_total_evenly(total: int, keys: list[str]) -> dict[str, int]:
    if not keys:
        return {}
    base = total // len(keys)
    remainder = total % len(keys)
    out: dict[str, int] = {}
    for i, key in enumerate(keys):
        out[key] = base + (1 if i < remainder else 0)
    return out


def _compute_vehicle_band_targets(manager: DatasetManager, num_samples: int, vehicle_ids: list[str]) -> dict[str, dict[str, int]]:
    ratio_cfg = manager.quality_cfg["target_ratio"]
    safe_ratio = float(ratio_cfg["safe"])
    fail_ratio = float(ratio_cfg["fail"])
    vehicle_targets = _split_total_evenly(num_samples, vehicle_ids)
    per_vehicle_band: dict[str, dict[str, int]] = {}
    for vehicle_id in vehicle_ids:
        n = int(vehicle_targets[vehicle_id])
        safe = int(round(n * safe_ratio))
        fail = int(round(n * fail_ratio))
        critical = max(n - safe - fail, 0)
        per_vehicle_band[vehicle_id] = {"safe": safe, "fail": fail, "critical": critical}
    return per_vehicle_band


def _build_manifest(
    *,
    manager: DatasetManager,
    num_samples: int,
    seed: int,
    backend: str,
    num_workers: int,
    run_tag: str,
    balance_mode: str,
    balance_relax_stage_final: str,
    balance_relax_events: list[dict[str, int | str]],
    batch_files: list[str],
    band_counts: dict[str, int],
    targets: dict[str, int],
    vehicle_targets: dict[str, int],
    vehicle_band_targets: dict[str, dict[str, int]],
    attempts_total: int,
    invalid_attempts: int,
    rejected_over_target: int,
    rejected_over_target_detail: Counter[str],
    invalid_reason_counter: Counter[str],
    worker_attempts: dict[str, int],
    worker_accepted: dict[str, int],
    terrain_counts: dict[str, int],
    vehicle_counts: dict[str, int],
    vehicle_band_counts: dict[str, dict[str, int]],
    friction_counts: dict[str, int],
    motion_counts: dict[str, int],
    action_counts: dict[str, int],
    terrain_compensation: dict[str, Any],
) -> dict[str, Any]:
    return {
        "dataset": {
            "name": manager.ds_meta["name"],
            "version": manager.ds_meta["version"],
            "created_at_utc": datetime.now(tz=timezone.utc).isoformat(),
            "num_samples_target": int(num_samples),
            "num_samples_accepted": int(sum(band_counts.values())),
            "attempts": int(attempts_total),
            "invalid_attempts": int(invalid_attempts),
            "invalid_sample_rate": float(invalid_attempts / max(sum(band_counts.values()) + invalid_attempts, 1)),
            "rejected_over_target": int(rejected_over_target),
            "rejected_over_target_detail": dict(sorted(rejected_over_target_detail.items())),
            "backend": backend,
            "seed": int(seed),
            "num_workers": int(num_workers),
            "run_tag": str(run_tag),
            "balance_mode": str(balance_mode),
            "balance_relax_stage_final": str(balance_relax_stage_final),
            "balance_relax_events": list(balance_relax_events),
        },
        "targets": dict(targets),
        "targets_detail": {
            "band": targets,
            "vehicle": vehicle_targets,
            "vehicle_band": vehicle_band_targets,
        },
        "counts": {
            "band": dict(band_counts),
            "terrain": dict(terrain_counts),
            "vehicle": dict(vehicle_counts),
            "vehicle_band": vehicle_band_counts,
            "friction": dict(friction_counts),
            "motion_model": dict(motion_counts),
            "action": dict(action_counts),
            "invalid_reason": dict(sorted(invalid_reason_counter.items())),
        },
        "worker": {
            "attempts": worker_attempts,
            "accepted": worker_accepted,
        },
        "batches": batch_files,
        "config_snapshot": {
            "map": manager.map_cfg,
            "simulation": manager.sim_cfg,
            "quality": manager.quality_cfg,
            "serialization": manager.ser_cfg,
        },
        "sampling": {
            "terrain_compensation": terrain_compensation,
        },
    }


def main() -> int:
    args = parse_args()
    if args.num_samples <= 0:
        raise ValueError("num_samples must be positive")
    if int(args.num_workers) <= 0:
        raise ValueError("num_workers must be positive")
    if int(args.num_workers) > ROS_DOMAIN_ID_SPAN:
        raise ValueError(
            f"num_workers={args.num_workers} exceeds unique ROS domain span {ROS_DOMAIN_ID_SPAN}; "
            "reduce workers or run multiple jobs with disjoint worker pools."
        )
    if int(args.result_queue_mult) <= 0:
        raise ValueError("result_queue_mult must be positive")

    package_root = Path(__file__).resolve().parents[1]
    manager_main = DatasetManager(package_root=package_root, config_dir=args.config_dir)
    run_tag = str(args.run_tag).strip()
    if not run_tag:
        run_tag = datetime.now(tz=timezone.utc).strftime("%m%d%H%M%S")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    accepted_jsonl = output_dir / "accepted_samples.jsonl"
    invalid_jsonl = output_dir / "invalid_samples.jsonl"
    accepted_jsonl.write_text("", encoding="utf-8")
    invalid_jsonl.write_text("", encoding="utf-8")

    vehicle_ids = _ordered_vehicle_ids(manager_main)
    if not vehicle_ids:
        raise RuntimeError("vehicle library is empty, cannot compute balancing targets")

    balance_mode = str(args.balance_mode)
    targets = manager_main._compute_band_targets(args.num_samples)
    vehicle_targets: dict[str, int] = {}
    vehicle_band_targets: dict[str, dict[str, int]] = {}
    if balance_mode == "vehicle_band":
        vehicle_targets = _split_total_evenly(args.num_samples, vehicle_ids)
        vehicle_band_targets = _compute_vehicle_band_targets(manager_main, args.num_samples, vehicle_ids)
        # Keep global targets consistent with per-vehicle targets in manifest / progress logs.
        targets = {"safe": 0, "fail": 0, "critical": 0}
        for vehicle_id in vehicle_ids:
            for band_name in ("safe", "fail", "critical"):
                targets[band_name] += int(vehicle_band_targets[vehicle_id][band_name])
    else:
        vehicle_targets = _split_total_evenly(args.num_samples, vehicle_ids)
        vehicle_band_targets = {vehicle_id: {"safe": 0, "fail": 0, "critical": 0} for vehicle_id in vehicle_ids}

    band_counts = {"safe": 0, "fail": 0, "critical": 0}
    vehicle_band_counts: dict[str, dict[str, int]] = {
        vehicle_id: {"safe": 0, "fail": 0, "critical": 0} for vehicle_id in vehicle_ids
    }
    enforce_balance = (not bool(args.disable_balance)) and args.num_samples >= 50
    max_attempts = max(args.num_samples * args.max_attempt_multiplier, 1000)

    terrain_counts: dict[str, int] = {}
    vehicle_counts: dict[str, int] = {}
    friction_counts: dict[str, int] = {}
    motion_counts: dict[str, int] = {}
    action_counts: dict[str, int] = {}

    attempts_total = 0
    invalid_attempts = 0
    rejected_over_target = 0
    rejected_over_target_detail: Counter[str] = Counter()
    invalid_reason_counter: Counter[str] = Counter()
    worker_attempts: Counter[str] = Counter()
    worker_accepted: Counter[str] = Counter()

    batch_size = int(args.flush_batch_size)
    batch_samples: list[dict[str, Any]] = []
    batch_files: list[str] = []
    accepted_records_buffer: list[dict[str, Any]] = []
    accepted_records_flush = 64
    invalid_records_buffer: list[dict[str, Any]] = []
    invalid_records_flush = 64
    accepted = 0
    stagnant_attempts = 0
    relax_stage = 0  # 0: strict vehicle_band, 1: vehicle + global band, 2: vehicle only
    relax_stage_name = {0: "strict_vehicle_band", 1: "vehicle_plus_global_band", 2: "vehicle_only"}
    relax_events: list[dict[str, int | str]] = []
    stall_window = max(int(args.stall_attempt_window), 1)

    ctx = mp.get_context("spawn")
    stop_event: mp.Event = ctx.Event()
    result_queue: mp.Queue = ctx.Queue(maxsize=max(32, int(args.num_workers) * int(args.result_queue_mult)))

    domain_map = {worker_id: _worker_domain_id(int(args.base_domain), worker_id) for worker_id in range(int(args.num_workers))}
    print(
        "[pilot] launch "
        f"workers={args.num_workers} base_domain={args.base_domain} "
        f"domain_ids={list(domain_map.values())}",
        flush=True,
    )

    worker_args = {
        "output_dir": str(output_dir),
        "backend": args.backend,
        "config_dir": str(args.config_dir.resolve()) if args.config_dir is not None else None,
        "seed": int(args.seed),
        "base_domain": int(args.base_domain),
        "package_root": str(package_root),
        "worker_startup_stagger_sec": float(args.worker_startup_stagger_sec),
        "ros_startup_timeout_sec": float(args.ros_startup_timeout_sec),
        "ros_service_timeout_sec": float(args.ros_service_timeout_sec),
        "run_tag": run_tag,
        "terrain_compensation": bool(args.terrain_compensation),
        "terrain_comp_strength": float(args.terrain_comp_strength),
        "terrain_comp_warmup": int(args.terrain_comp_warmup),
        "terrain_comp_min_mult": float(args.terrain_comp_min_mult),
        "terrain_comp_max_mult": float(args.terrain_comp_max_mult),
    }

    workers = []
    for worker_id in range(args.num_workers):
        p = ctx.Process(target=_worker_main, args=(worker_id, worker_args, stop_event, result_queue))
        p.start()
        workers.append(p)

    try:
        while accepted < args.num_samples and attempts_total < max_attempts:
            try:
                msg = result_queue.get(timeout=1.0)
            except queue.Empty:
                if not any(p.is_alive() for p in workers):
                    raise RuntimeError("all workers exited unexpectedly before collecting enough samples")
                continue

            attempts_total += 1
            worker_key = str(msg.get("worker_id", -1))
            worker_attempts[worker_key] += 1

            if msg.get("kind") != "valid":
                invalid_attempts += 1
                stagnant_attempts += 1
                reason = str(msg.get("reason", "runtime_failure"))
                invalid_reason_counter[reason] += 1
                if reason == "ros_gz_python_unavailable":
                    raise RuntimeError(
                        "ros_gz backend unavailable: missing ROS Gazebo Python interfaces in current environment."
                    )
                invalid_records_buffer.append(
                    {
                        "attempt_idx": attempts_total,
                        "worker_id": int(msg.get("worker_id", -1)),
                        "seed": int(msg.get("seed", -1)),
                        "reason": reason,
                        "error": str(msg.get("error", "")),
                    }
                )
                if len(invalid_records_buffer) >= invalid_records_flush:
                    _write_jsonl(invalid_jsonl, invalid_records_buffer)
                    invalid_records_buffer.clear()
                if invalid_attempts <= 10 or invalid_attempts % 20 == 0:
                    print(
                        "[pilot] invalid "
                        f"attempts={invalid_attempts} accepted={accepted}/{args.num_samples} "
                        f"reason={reason}",
                        flush=True,
                    )
                continue

            band = str(msg["band"])
            counts_info = msg["counts_info"]
            vehicle_id = str(counts_info["vehicle_id"])
            if vehicle_id not in vehicle_band_counts:
                vehicle_band_counts[vehicle_id] = {"safe": 0, "fail": 0, "critical": 0}
            if vehicle_id not in vehicle_targets:
                # Unknown vehicle ids are allowed but cannot be balanced to a predefined target.
                vehicle_targets[vehicle_id] = args.num_samples
            if vehicle_id not in vehicle_band_targets:
                vehicle_band_targets[vehicle_id] = {"safe": args.num_samples, "fail": args.num_samples, "critical": args.num_samples}

            if enforce_balance:
                reject_reason = None
                if balance_mode == "vehicle_band":
                    if vehicle_counts.get(vehicle_id, 0) >= int(vehicle_targets[vehicle_id]):
                        reject_reason = "vehicle_over_target"
                    elif relax_stage == 0:
                        if int(vehicle_band_counts[vehicle_id][band]) >= int(vehicle_band_targets[vehicle_id][band]):
                            reject_reason = "vehicle_band_over_target"
                    elif relax_stage == 1:
                        if int(band_counts[band]) >= int(targets[band]):
                            reject_reason = "band_over_target_relaxed"
                    else:
                        # relax_stage == 2: only vehicle totals are enforced
                        pass
                else:
                    if band_counts[band] >= targets[band]:
                        reject_reason = "band_over_target"
                if reject_reason is not None:
                    rejected_over_target += 1
                    rejected_over_target_detail[reject_reason] += 1
                    stagnant_attempts += 1
                    if (
                        enforce_balance
                        and balance_mode == "vehicle_band"
                        and relax_stage < 2
                        and stagnant_attempts >= stall_window
                    ):
                        relax_stage += 1
                        relax_events.append(
                            {
                                "attempt": int(attempts_total),
                                "accepted": int(accepted),
                                "new_stage": str(relax_stage_name[relax_stage]),
                            }
                        )
                        print(
                            f"[pilot] balance relaxed -> {relax_stage_name[relax_stage]} "
                            f"(attempt={attempts_total}, accepted={accepted}, stagnant_attempts={stagnant_attempts})",
                            flush=True,
                        )
                        stagnant_attempts = 0
                    continue

            sample = msg["sample"]
            sample["metadata"]["sample_id"] = accepted
            batch_samples.append(sample)

            accepted += 1
            stagnant_attempts = 0
            band_counts[band] += 1
            worker_accepted[worker_key] += 1
            vehicle_band_counts[vehicle_id][band] = vehicle_band_counts[vehicle_id].get(band, 0) + 1
            terrain_counts[counts_info["terrain_class"]] = terrain_counts.get(counts_info["terrain_class"], 0) + 1
            vehicle_counts[counts_info["vehicle_id"]] = vehicle_counts.get(counts_info["vehicle_id"], 0) + 1
            friction_counts[counts_info["friction_class"]] = friction_counts.get(counts_info["friction_class"], 0) + 1
            motion_counts[counts_info["motion_model"]] = motion_counts.get(counts_info["motion_model"], 0) + 1
            action_counts[counts_info["action_id"]] = action_counts.get(counts_info["action_id"], 0) + 1

            labels = msg["labels"]
            record = {
                "sample_id": accepted - 1,
                "worker_id": int(msg["worker_id"]),
                "seed": int(msg["seed"]),
                "band": band,
                "vehicle_id": counts_info["vehicle_id"],
                "terrain_class": counts_info["terrain_class"],
                "friction_class": counts_info["friction_class"],
                "motion_model": counts_info["motion_model"],
                "action_id": counts_info["action_id"],
                "y_fail": float(labels["y_fail"]),
                "q_roll": float(labels["q_roll"]),
                "q_pitch": float(labels["q_pitch"]),
                "q_slip": float(labels["q_slip"]),
                "q_lift": float(labels["q_lift"]),
                "p_bottom": float(labels["p_bottom"]),
                "p_stuck": float(labels["p_stuck"]),
                "progress_ratio": float(msg["progress_ratio"]),
                "translation_progress": float(msg["translation_progress"]),
                "angular_progress": float(msg["angular_progress"]),
                "translation_drift": float(msg["translation_drift"]),
                "fail_reasons": list(msg["fail_reasons"]),
            }
            accepted_records_buffer.append(record)
            if len(accepted_records_buffer) >= accepted_records_flush:
                _write_jsonl(accepted_jsonl, accepted_records_buffer)
                accepted_records_buffer.clear()

            if len(batch_samples) >= batch_size:
                batch_file = output_dir / f"samples_batch_{len(batch_files) + 1:04d}.h5"
                manager_main.packager.write_hdf5_batch(
                    samples=batch_samples,
                    output_path=batch_file,
                    compression=str(manager_main.ser_cfg["compression"]),
                    compression_level=int(manager_main.ser_cfg["compression_level"]),
                )
                batch_files.append(batch_file.name)
                batch_samples = []

            if accepted <= 20 or accepted % 25 == 0 or accepted == args.num_samples:
                print(
                    "[pilot] accepted "
                    f"{accepted}/{args.num_samples} attempts={attempts_total} invalid={invalid_attempts} "
                    f"band_counts={band_counts}",
                    flush=True,
                )

        if accepted < args.num_samples:
            raise RuntimeError(
                f"insufficient accepted samples: accepted={accepted}, target={args.num_samples}, "
                f"attempts={attempts_total}, max_attempts={max_attempts}"
            )
    finally:
        stop_event.set()
        for p in workers:
            p.join(timeout=15.0)
        for p in workers:
            if p.is_alive():
                p.terminate()
                p.join(timeout=5.0)

    if batch_samples:
        batch_file = output_dir / f"samples_batch_{len(batch_files) + 1:04d}.h5"
        manager_main.packager.write_hdf5_batch(
            samples=batch_samples,
            output_path=batch_file,
            compression=str(manager_main.ser_cfg["compression"]),
            compression_level=int(manager_main.ser_cfg["compression_level"]),
        )
        batch_files.append(batch_file.name)

    if accepted_records_buffer:
        _write_jsonl(accepted_jsonl, accepted_records_buffer)
    if invalid_records_buffer:
        _write_jsonl(invalid_jsonl, invalid_records_buffer)

    manifest = _build_manifest(
        manager=manager_main,
        num_samples=args.num_samples,
        seed=args.seed,
        backend=args.backend,
        num_workers=args.num_workers,
        run_tag=run_tag,
        balance_mode=balance_mode,
        balance_relax_stage_final=relax_stage_name[relax_stage],
        balance_relax_events=relax_events,
        batch_files=batch_files,
        band_counts=band_counts,
        targets=targets,
        vehicle_targets=vehicle_targets,
        vehicle_band_targets=vehicle_band_targets,
        attempts_total=attempts_total,
        invalid_attempts=invalid_attempts,
        rejected_over_target=rejected_over_target,
        rejected_over_target_detail=rejected_over_target_detail,
        invalid_reason_counter=invalid_reason_counter,
        worker_attempts=dict(worker_attempts),
        worker_accepted=dict(worker_accepted),
        terrain_counts=terrain_counts,
        vehicle_counts=vehicle_counts,
        vehicle_band_counts=vehicle_band_counts,
        friction_counts=friction_counts,
        motion_counts=motion_counts,
        action_counts=action_counts,
        terrain_compensation={
            "enabled": bool(args.terrain_compensation),
            "strength": float(args.terrain_comp_strength),
            "warmup": int(args.terrain_comp_warmup),
            "min_multiplier": float(args.terrain_comp_min_mult),
            "max_multiplier": float(args.terrain_comp_max_mult),
        },
    )

    manifest_path = output_dir / str(manager_main.ser_cfg["manifest_name"])
    manifest.setdefault("runtime", {})
    manifest["runtime"]["ros_domain_ids_by_worker"] = {str(k): int(v) for k, v in domain_map.items()}
    manifest["runtime"]["result_queue_maxsize"] = int(max(32, int(args.num_workers) * int(args.result_queue_mult)))
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(
        json.dumps(
            {
                "manifest_path": str(manifest_path),
                "accepted_jsonl": str(accepted_jsonl),
                "invalid_jsonl": str(invalid_jsonl),
                "batches": batch_files,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
