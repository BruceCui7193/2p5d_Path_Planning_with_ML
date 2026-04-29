#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import multiprocessing as mp
import os
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from ml25d_dataset_generation.common_types import VehicleParams
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    build: Callable[[int, float], tuple[np.ndarray, list[dict[str, float]]]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run terrain ladder diagnostics on ros_gz backend")
    parser.add_argument("--repeats", type=int, default=10, help="Valid samples per (vehicle, terrain) pair")
    parser.add_argument("--max-retry", type=int, default=2, help="Retry count for invalid contact samples")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/diagnostics/terrain_ladder"),
        help="Output directory for markdown/json artifacts",
    )
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--num-workers", type=int, default=1, choices=[1, 2, 4], help="Number of parallel workers")
    parser.add_argument("--base-domain", type=int, default=90, help="Base ROS_DOMAIN_ID for worker isolation")
    parser.add_argument(
        "--vehicles",
        type=str,
        default="city_small,offroad_medium,mountain_large",
        help="Comma-separated vehicle types to include",
    )
    parser.add_argument(
        "--terrains",
        type=str,
        default="",
        help="Comma-separated terrain names to include; empty means all",
    )
    parser.add_argument(
        "--max-combos",
        type=int,
        default=0,
        help="Optional cap for total (vehicle, terrain) combos after filtering; 0 means no cap",
    )
    return parser.parse_args()


def _terrain_flat(patch: int, _: float) -> np.ndarray:
    return np.zeros((patch, patch), dtype=np.float32)


def _terrain_slope_x(patch: int, resolution_m: float, slope_deg: float) -> np.ndarray:
    slope = float(np.tan(np.deg2rad(slope_deg)))
    center = (patch - 1) / 2.0
    x = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    return np.repeat((slope * x)[:, None], patch, axis=1).astype(np.float32)


def _terrain_slope_y(patch: int, resolution_m: float, slope_deg: float) -> np.ndarray:
    slope = float(np.tan(np.deg2rad(slope_deg)))
    center = (patch - 1) / 2.0
    y = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    return np.repeat((slope * y)[None, :], patch, axis=0).astype(np.float32)


def _terrain_step_hard(patch: int, resolution_m: float, step_h_m: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    step_x = 0.22
    h[x_vals >= step_x, :] = float(step_h_m)
    obstacles = [
        {
            "x_m": float(step_x),
            "y_m": 0.0,
            "length_m": 0.04,
            "width_m": float((patch - 2) * resolution_m),
            "height_m": float(step_h_m),
        }
    ]
    return h, obstacles


def _terrain_trench_center(patch: int, resolution_m: float, width_m: float, depth_m: float) -> np.ndarray:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    mask = np.abs(x_vals - 0.12) <= (0.5 * width_m)
    h[mask, :] = -float(depth_m)
    return h


def _terrain_single_wheel_trench(patch: int, resolution_m: float, width_m: float, depth_m: float) -> np.ndarray:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    y_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(x_vals, y_vals, indexing="ij")
    mask = (np.abs(xx - 0.12) <= (0.5 * width_m)) & (yy > 0.08)
    h[mask] = -float(depth_m)
    return h


def _terrain_diagonal_pit(patch: int, resolution_m: float, major_m: float, minor_m: float, depth_m: float) -> np.ndarray:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    y_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(x_vals, y_vals, indexing="ij")
    angle = np.deg2rad(35.0)
    u = xx * np.cos(angle) + yy * np.sin(angle) - 0.05
    v = -xx * np.sin(angle) + yy * np.cos(angle)
    pit = (u / max(major_m, 1e-6)) ** 2 + (v / max(minor_m, 1e-6)) ** 2
    mask = pit <= 1.0
    h[mask] = -float(depth_m) * (1.0 - pit[mask]).astype(np.float32)
    return h


def _terrain_bump_field(patch: int, resolution_m: float) -> np.ndarray:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    bumps = (
        0.030 * np.exp(-((xx - 0.4) ** 2 + (yy + 0.3) ** 2) / 0.10)
        + 0.025 * np.exp(-((xx + 0.5) ** 2 + (yy - 0.2) ** 2) / 0.08)
        + 0.020 * np.exp(-((xx + 0.1) ** 2 + (yy + 0.5) ** 2) / 0.06)
    )
    micro = 0.006 * (np.sin(8.0 * xx) * np.cos(6.0 * yy))
    return (bumps + micro).astype(np.float32)


def _terrain_low_bump(patch: int, resolution_m: float, bump_h_m: float = 0.03) -> np.ndarray:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    bump = bump_h_m * np.exp(-((xx - 0.14) ** 2 + yy**2) / max(2.0 * 0.06 * 0.06, 1e-6))
    return bump.astype(np.float32)


def _terrain_pit_gaussian(patch: int, resolution_m: float, sigma_m: float, depth_m: float) -> np.ndarray:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    pit = -float(depth_m) * np.exp(-((xx - 0.12) ** 2 + yy**2) / max(2.0 * sigma_m * sigma_m, 1e-6))
    rough = 0.003 * (np.sin(7.0 * xx + 0.5) * np.sin(6.0 * yy - 0.4))
    return (pit + rough).astype(np.float32)


def _scenario_builders() -> dict[str, ScenarioSpec]:
    return {
        "flat_forward": ScenarioSpec("flat_forward", lambda p, r: (_terrain_flat(p, r), [])),
        "slope_forward_5deg": ScenarioSpec("slope_forward_5deg", lambda p, r: (_terrain_slope_x(p, r, 5.0), [])),
        "slope_forward_10deg": ScenarioSpec("slope_forward_10deg", lambda p, r: (_terrain_slope_x(p, r, 10.0), [])),
        "slope_forward_20deg": ScenarioSpec("slope_forward_20deg", lambda p, r: (_terrain_slope_x(p, r, 20.0), [])),
        "cross_slope_10deg": ScenarioSpec("cross_slope_10deg", lambda p, r: (_terrain_slope_y(p, r, 10.0), [])),
        "cross_slope_20deg": ScenarioSpec("cross_slope_20deg", lambda p, r: (_terrain_slope_y(p, r, 20.0), [])),
        "step_5cm": ScenarioSpec("step_5cm", lambda p, r: _terrain_step_hard(p, r, 0.05)),
        "step_10cm": ScenarioSpec("step_10cm", lambda p, r: _terrain_step_hard(p, r, 0.10)),
        "step_15cm": ScenarioSpec("step_15cm", lambda p, r: _terrain_step_hard(p, r, 0.15)),
        "bump_field": ScenarioSpec("bump_field", lambda p, r: (_terrain_bump_field(p, r), [])),
        "low_bump": ScenarioSpec("low_bump", lambda p, r: (_terrain_low_bump(p, r, bump_h_m=0.03), [])),
        "pit_small": ScenarioSpec("pit_small", lambda p, r: (_terrain_pit_gaussian(p, r, sigma_m=0.09, depth_m=0.08), [])),
        "pit_medium": ScenarioSpec("pit_medium", lambda p, r: (_terrain_pit_gaussian(p, r, sigma_m=0.14, depth_m=0.15), [])),
        "pit_large": ScenarioSpec("pit_large", lambda p, r: (_terrain_pit_gaussian(p, r, sigma_m=0.20, depth_m=0.22), [])),
        "trench_forward": ScenarioSpec("trench_forward", lambda p, r: (_terrain_trench_center(p, r, width_m=0.14, depth_m=0.10), [])),
        "full_width_trench": ScenarioSpec("full_width_trench", lambda p, r: (_terrain_trench_center(p, r, width_m=0.24, depth_m=0.16), [])),
        "single_wheel_trench": ScenarioSpec("single_wheel_trench", lambda p, r: (_terrain_single_wheel_trench(p, r, width_m=0.18, depth_m=0.14), [])),
        "diagonal_pit": ScenarioSpec("diagonal_pit", lambda p, r: (_terrain_diagonal_pit(p, r, major_m=0.26, minor_m=0.12, depth_m=0.18), [])),
    }


def _parse_csv_arg(text: str) -> list[str]:
    return [item.strip() for item in str(text).split(",") if item.strip()]


def _is_invalid_contact_error(message: str) -> bool:
    msg = message.lower()
    return ("no wheel contact sensor samples" in msg) or ("insufficient wheel contact observability" in msg)


def _fmt(value: object) -> str:
    if isinstance(value, float):
        if np.isnan(value):
            return "NaN"
        return f"{value:.4f}"
    return str(value)


def _to_md_table(rows: list[dict[str, object]], headers: list[str]) -> str:
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(lines)


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


def _append_jsonl(path: Path, records: list[dict[str, object]]) -> None:
    if not records:
        return
    with path.open("a", encoding="utf-8") as fp:
        for rec in records:
            fp.write(json.dumps(rec, ensure_ascii=False) + "\n")


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


def _run_combo(
    *,
    task: dict,
    manager: DatasetManager,
    runner,
    scenario_builders: dict[str, ScenarioSpec],
    sample_jsonl_path: Path,
) -> dict[str, object]:
    vehicle_map: dict[str, VehicleParams] = {
        "city_small": next(v for v in manager.vehicle_library if v.vehicle_id == "urban_small"),
        "offroad_medium": next(v for v in manager.vehicle_library if v.vehicle_id == "standard_offroad"),
        "mountain_large": next(v for v in manager.vehicle_library if v.vehicle_id == "mountain_large"),
    }
    extractor = manager.label_extractor
    thresholds = extractor.thresholds

    patch = int(manager.map_cfg["patch_size"])
    resolution_m = float(manager.map_cfg["resolution_m_per_cell"])
    friction_mu = 0.8
    sample_rate_hz = int(manager.sim_cfg["sample_rate_hz"])
    settle_time_sec = float(manager.sim_cfg["settle_time_sec"])
    action_forward = next(a for a in manager.action_library if a.action_id == "a0")

    vehicle_type = task["vehicle_type"]
    terrain_type = task["terrain_type"]
    repeats = int(task["repeats"])
    max_retry = int(task["max_retry"])
    combo_seed = int(task["combo_seed"])
    combo_index = int(task["combo_index"])
    worker_id = int(task["worker_id"])

    vehicle = vehicle_map[vehicle_type]
    scenario = scenario_builders[terrain_type]
    combo_rng = np.random.default_rng(combo_seed)

    valid_collected = 0
    max_attempt_budget = repeats * max(max_retry + 3, 8)
    logical_attempts = 0
    dropped_invalid = 0
    retry_count = 0
    fail_samples = 0
    fail_reason_counter: Counter[str] = Counter()
    invalid_reason_counter: Counter[str] = Counter()

    q_roll_values: list[float] = []
    q_pitch_values: list[float] = []
    q_slip_values: list[float] = []
    q_lift_values: list[float] = []
    bottom_flags: list[float] = []
    stuck_flags: list[float] = []
    progress_values: list[float] = []
    sample_records: list[dict[str, object]] = []

    while valid_collected < repeats and logical_attempts < max_attempt_budget:
        logical_attempts += 1
        accepted = False
        for retry_idx in range(max_retry + 1):
            if retry_idx > 0:
                retry_count += 1
            seed = int(combo_rng.integers(0, 2**31 - 1))
            rng = np.random.default_rng(seed)
            terrain_map, obstacles = scenario.build(patch, resolution_m)
            scene_id = f"w{worker_id}_c{combo_index}_la{logical_attempts}_r{retry_idx}_v{valid_collected}"

            context = SimulationContext(
                heightmap=terrain_map,
                heading_rad=0.0,
                vehicle=vehicle,
                action=action_forward,
                friction_mu=friction_mu,
                motion_model="skid",
                sample_rate_hz=sample_rate_hz,
                duration_sec=float(manager.sim_cfg["action_duration_sec"]),
                settle_time_sec=settle_time_sec,
                extra_obstacles=obstacles,
                scene_id=scene_id,
            )

            try:
                traj = runner.run(context, rng)
            except Exception as exc:
                msg = str(exc)
                if _is_invalid_contact_error(msg):
                    if retry_idx == max_retry:
                        dropped_invalid += 1
                        invalid_reason_counter["no_wheel_contact_samples"] += 1
                        sample_records.append(
                            {
                                "worker_id": worker_id,
                                "combo_index": combo_index,
                                "vehicle_type": vehicle_type,
                                "terrain_type": terrain_type,
                                "seed": seed,
                                "scene_id": scene_id,
                                "status": "invalid",
                                "invalid_reason": "no_wheel_contact_samples",
                            }
                        )
                    continue
                dropped_invalid += 1
                invalid_reason_counter["runtime_failure"] += 1
                sample_records.append(
                    {
                        "worker_id": worker_id,
                        "combo_index": combo_index,
                        "vehicle_type": vehicle_type,
                        "terrain_type": terrain_type,
                        "seed": seed,
                        "scene_id": scene_id,
                        "status": "invalid",
                        "invalid_reason": "runtime_failure",
                        "error": msg,
                    }
                )
                break

            labels, _ = extractor.compute_labels(traj, vehicle, action_forward)
            valid_collected += 1
            accepted = True

            q_roll_values.append(float(labels.q_roll))
            q_pitch_values.append(float(labels.q_pitch))
            q_slip_values.append(float(labels.q_slip))
            q_lift_values.append(float(labels.q_lift))

            roll_fail = float(labels.q_roll > thresholds.roll_fail_threshold)
            pitch_fail = float(labels.q_pitch > thresholds.pitch_fail_threshold)
            slip_fail = float(labels.q_slip > thresholds.slip_fail_threshold)
            lift_fail = float(labels.q_lift > thresholds.lift_fail_threshold)
            bottom_fail = float(labels.p_bottom > thresholds.bottom_fail_threshold)
            stuck_fail = float(labels.p_stuck >= 1.0)

            bottom_flags.append(bottom_fail)
            stuck_flags.append(stuck_fail)
            progress_ratio = float(traj.completed_displacement_m / max(action_forward.delta_s_m, 1e-6))
            progress_values.append(progress_ratio)

            final_fail = bool(
                roll_fail > 0.5
                or pitch_fail > 0.5
                or slip_fail > 0.5
                or lift_fail > 0.5
                or bottom_fail > 0.5
                or stuck_fail > 0.5
            )
            if final_fail:
                fail_samples += 1
            fail_reasons = _sample_fail_reasons(labels, thresholds)
            for reason in fail_reasons:
                fail_reason_counter[reason] += 1

            sample_records.append(
                {
                    "worker_id": worker_id,
                    "combo_index": combo_index,
                    "vehicle_type": vehicle_type,
                    "terrain_type": terrain_type,
                    "seed": seed,
                    "scene_id": scene_id,
                    "status": "valid",
                    "y_fail": float(labels.y_fail),
                    "q_roll": float(labels.q_roll),
                    "q_pitch": float(labels.q_pitch),
                    "q_slip": float(labels.q_slip),
                    "q_lift": float(labels.q_lift),
                    "p_bottom": float(labels.p_bottom),
                    "p_stuck": float(labels.p_stuck),
                    "progress_ratio": progress_ratio,
                    "fail_reasons": fail_reasons,
                }
            )
            break

        if accepted and (valid_collected <= 2 or valid_collected % 5 == 0 or valid_collected == repeats):
            print(
                "[ladder] progress "
                f"worker={worker_id} vehicle={vehicle_type} terrain={terrain_type} "
                f"valid={valid_collected}/{repeats} logical_attempts={logical_attempts} "
                f"invalid={dropped_invalid} retries={retry_count}",
                flush=True,
            )

    _append_jsonl(sample_jsonl_path, sample_records)

    valid = len(q_roll_values)
    invalid_sample_rate = float(dropped_invalid / max(valid + dropped_invalid, 1))
    row: dict[str, object] = {
        "combo_index": combo_index,
        "worker_id": worker_id,
        "vehicle_type": vehicle_type,
        "terrain_type": terrain_type,
        "final_fail_rate": float(fail_samples / max(valid, 1)),
        "fail_reason_distribution": dict(sorted(fail_reason_counter.items())),
        "q_roll_mean": float(np.mean(q_roll_values)) if valid else float("nan"),
        "q_roll_p95": float(np.percentile(q_roll_values, 95)) if valid else float("nan"),
        "q_pitch_mean": float(np.mean(q_pitch_values)) if valid else float("nan"),
        "q_pitch_p95": float(np.percentile(q_pitch_values, 95)) if valid else float("nan"),
        "q_slip_mean": float(np.mean(q_slip_values)) if valid else float("nan"),
        "q_slip_p95": float(np.percentile(q_slip_values, 95)) if valid else float("nan"),
        "q_lift_mean": float(np.mean(q_lift_values)) if valid else float("nan"),
        "q_lift_p95": float(np.percentile(q_lift_values, 95)) if valid else float("nan"),
        "bottom_fail_rate": float(np.mean(bottom_flags)) if valid else float("nan"),
        "stuck_fail_rate": float(np.mean(stuck_flags)) if valid else float("nan"),
        "progress_mean": float(np.mean(progress_values)) if valid else float("nan"),
        "invalid_sample_rate": invalid_sample_rate,
        "retry_count": int(retry_count),
        "valid_count": int(valid),
        "invalid_reason_distribution": dict(sorted(invalid_reason_counter.items())),
    }
    print(
        "[ladder] combo done "
        f"worker={worker_id} vehicle={vehicle_type} terrain={terrain_type} "
        f"valid={valid} fail_rate={row['final_fail_rate']:.3f} invalid_rate={invalid_sample_rate:.3f}",
        flush=True,
    )
    return row


def _worker_main(worker_id: int, task_queue: mp.Queue, result_queue: mp.Queue, cfg: dict) -> None:
    worker_dir = Path(cfg["output_dir"]) / f"worker_{worker_id}"
    worker_dir.mkdir(parents=True, exist_ok=True)
    sample_jsonl_path = worker_dir / "samples.jsonl"
    sample_jsonl_path.write_text("", encoding="utf-8")

    os.environ["ROS_DOMAIN_ID"] = str(int(cfg["base_domain"]) + worker_id)
    os.environ["GZ_PARTITION"] = f"ml25d_worker_{worker_id}"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")

    package_root = Path(cfg["package_root"])
    manager = DatasetManager(package_root=package_root)
    scenario_builders = _scenario_builders()

    sim_cfg = copy.deepcopy(manager.sim_cfg)
    ros_cfg = sim_cfg.setdefault("ros_gz", {})
    world_name = f"ml25d_w{worker_id}"
    model_name = f"ml25d_vehicle_w{worker_id}"
    world_template = _resolve_world_sdf_file(package_root, str(ros_cfg.get("world_sdf_file", "worlds/ml25d_empty.sdf")))
    worker_world = _create_worker_world_file(world_template, worker_dir, world_name)
    ros_cfg["world_sdf_file"] = str(worker_world)
    ros_cfg["world_name"] = world_name
    ros_cfg["model_name"] = model_name
    ros_cfg["log_dir"] = str(worker_dir / "runner_logs")

    runner = make_runner("ros_gz", sim_cfg)
    try:
        while True:
            task = task_queue.get()
            if task is None:
                break
            task_local = dict(task)
            task_local["worker_id"] = worker_id
            try:
                row = _run_combo(
                    task=task_local,
                    manager=manager,
                    runner=runner,
                    scenario_builders=scenario_builders,
                    sample_jsonl_path=sample_jsonl_path,
                )
                result_queue.put({"ok": True, "row": row})
            except Exception as exc:
                result_queue.put(
                    {
                        "ok": False,
                        "combo_index": int(task_local["combo_index"]),
                        "worker_id": worker_id,
                        "error": str(exc),
                    }
                )
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass


def _build_tasks(
    seed: int,
    repeats: int,
    max_retry: int,
    vehicles: list[str],
    terrains: list[str],
    max_combos: int,
) -> list[dict[str, object]]:
    master_rng = np.random.default_rng(seed)
    tasks: list[dict[str, object]] = []
    combo_index = 0
    for vehicle_type in vehicles:
        for terrain_type in terrains:
            combo_index += 1
            tasks.append(
                {
                    "combo_index": combo_index,
                    "vehicle_type": vehicle_type,
                    "terrain_type": terrain_type,
                    "combo_seed": int(master_rng.integers(0, 2**31 - 1)),
                    "repeats": repeats,
                    "max_retry": max_retry,
                }
            )
            if max_combos > 0 and len(tasks) >= max_combos:
                return tasks
    return tasks


def _run_serial(args: argparse.Namespace, package_root: Path, tasks: list[dict[str, object]]) -> list[dict[str, object]]:
    worker_dir = args.output_dir / "worker_0"
    worker_dir.mkdir(parents=True, exist_ok=True)
    sample_jsonl_path = worker_dir / "samples.jsonl"
    sample_jsonl_path.write_text("", encoding="utf-8")

    os.environ["ROS_DOMAIN_ID"] = str(int(args.base_domain))
    os.environ["GZ_PARTITION"] = "ml25d_worker_0"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")

    manager = DatasetManager(package_root=package_root)
    scenario_builders = _scenario_builders()
    sim_cfg = copy.deepcopy(manager.sim_cfg)
    ros_cfg = sim_cfg.setdefault("ros_gz", {})
    world_name = "ml25d_w0"
    model_name = "ml25d_vehicle_w0"
    world_template = _resolve_world_sdf_file(package_root, str(ros_cfg.get("world_sdf_file", "worlds/ml25d_empty.sdf")))
    worker_world = _create_worker_world_file(world_template, worker_dir, world_name)
    ros_cfg["world_sdf_file"] = str(worker_world)
    ros_cfg["world_name"] = world_name
    ros_cfg["model_name"] = model_name
    ros_cfg["log_dir"] = str(worker_dir / "runner_logs")
    runner = make_runner("ros_gz", sim_cfg)
    rows: list[dict[str, object]] = []
    try:
        total = len(tasks)
        for idx, task in enumerate(tasks, start=1):
            print(
                f"[ladder] combo {idx}/{total} start vehicle={task['vehicle_type']} terrain={task['terrain_type']}",
                flush=True,
            )
            task_local = dict(task)
            task_local["worker_id"] = 0
            row = _run_combo(
                task=task_local,
                manager=manager,
                runner=runner,
                scenario_builders=scenario_builders,
                sample_jsonl_path=sample_jsonl_path,
            )
            rows.append(row)
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass
    return rows


def _run_parallel(args: argparse.Namespace, package_root: Path, tasks: list[dict[str, object]]) -> list[dict[str, object]]:
    ctx = mp.get_context("spawn")
    task_queue: mp.Queue = ctx.Queue()
    result_queue: mp.Queue = ctx.Queue()
    worker_cfg = {
        "package_root": str(package_root),
        "output_dir": str(args.output_dir),
        "base_domain": int(args.base_domain),
    }

    workers = []
    for worker_id in range(args.num_workers):
        proc = ctx.Process(target=_worker_main, args=(worker_id, task_queue, result_queue, worker_cfg))
        proc.start()
        workers.append(proc)

    for task in tasks:
        task_queue.put(task)
    for _ in range(args.num_workers):
        task_queue.put(None)

    rows: list[dict[str, object]] = []
    errors: list[dict[str, object]] = []
    total = len(tasks)
    for i in range(total):
        result = result_queue.get()
        if result.get("ok"):
            row = result["row"]
            rows.append(row)
            print(
                f"[ladder] merged {i+1}/{total} worker={row['worker_id']} "
                f"vehicle={row['vehicle_type']} terrain={row['terrain_type']}",
                flush=True,
            )
        else:
            errors.append(result)
            print(
                f"[ladder] worker error combo={result.get('combo_index')} "
                f"worker={result.get('worker_id')} error={result.get('error')}",
                flush=True,
            )

    for proc in workers:
        proc.join(timeout=30.0)
        if proc.is_alive():
            proc.terminate()
            proc.join(timeout=5.0)

    if errors:
        raise RuntimeError(f"parallel worker errors detected: {errors}")
    return rows


def _merge_worker_jsonl(output_dir: Path, num_workers: int) -> Path:
    merged_path = output_dir / "samples_merged.jsonl"
    with merged_path.open("w", encoding="utf-8") as out_fp:
        for worker_id in range(num_workers):
            p = output_dir / f"worker_{worker_id}" / "samples.jsonl"
            if not p.exists():
                continue
            out_fp.write(p.read_text(encoding="utf-8"))
    return merged_path


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    scenario_builders = _scenario_builders()
    valid_vehicles = ["city_small", "offroad_medium", "mountain_large"]
    selected_vehicles = _parse_csv_arg(args.vehicles)
    if not selected_vehicles:
        selected_vehicles = valid_vehicles
    invalid_vehicles = sorted(set(selected_vehicles) - set(valid_vehicles))
    if invalid_vehicles:
        raise ValueError(f"unknown vehicles: {invalid_vehicles}; valid={valid_vehicles}")

    selected_terrains = _parse_csv_arg(args.terrains)
    if not selected_terrains:
        selected_terrains = list(scenario_builders.keys())
    invalid_terrains = sorted(set(selected_terrains) - set(scenario_builders.keys()))
    if invalid_terrains:
        raise ValueError(f"unknown terrains: {invalid_terrains}")

    tasks = _build_tasks(
        seed=args.seed,
        repeats=args.repeats,
        max_retry=args.max_retry,
        vehicles=selected_vehicles,
        terrains=selected_terrains,
        max_combos=max(0, int(args.max_combos)),
    )
    if args.num_workers == 1:
        rows = _run_serial(args, package_root, tasks)
    else:
        rows = _run_parallel(args, package_root, tasks)

    rows_sorted = sorted(rows, key=lambda x: int(x["combo_index"]))
    for row in rows_sorted:
        row.pop("combo_index", None)
    merged_samples = _merge_worker_jsonl(args.output_dir, args.num_workers)

    json_path = args.output_dir / "terrain_ladder_summary.json"
    json_path.write_text(
        json.dumps(
            {
                "repeats": args.repeats,
                "max_retry": args.max_retry,
                "scenario_count": len(selected_terrains),
                "vehicles": selected_vehicles,
                "terrains": selected_terrains,
                "num_workers": args.num_workers,
                "rows": rows_sorted,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    md_headers = [
        "worker_id",
        "vehicle_type",
        "terrain_type",
        "final_fail_rate",
        "fail_reason_distribution",
        "q_roll_mean",
        "q_roll_p95",
        "q_pitch_mean",
        "q_pitch_p95",
        "q_slip_mean",
        "q_slip_p95",
        "q_lift_mean",
        "q_lift_p95",
        "bottom_fail_rate",
        "stuck_fail_rate",
        "progress_mean",
        "invalid_sample_rate",
        "retry_count",
    ]
    md_path = args.output_dir / "terrain_ladder_summary.md"
    md_path.write_text(_to_md_table(rows_sorted, md_headers) + "\n", encoding="utf-8")

    print(json.dumps({"json": str(json_path), "md": str(md_path), "samples": str(merged_samples)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
