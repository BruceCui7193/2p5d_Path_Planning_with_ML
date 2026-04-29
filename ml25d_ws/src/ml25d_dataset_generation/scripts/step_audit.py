#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit step scenarios for geometry/contact realism")
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/diagnostics/step_audit"),
    )
    return parser.parse_args()


def _fmt(v: object) -> str:
    if isinstance(v, float):
        if np.isnan(v):
            return "NaN"
        return f"{v:.4f}"
    return str(v)


def _md_table(rows: list[dict[str, object]], headers: list[str]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(lines)


def _sample_height_nearest(heightmap: np.ndarray, resolution_m: float, x: float, y: float) -> float:
    n, m = heightmap.shape
    ci = (n - 1) / 2.0
    cj = (m - 1) / 2.0
    i = int(round(x / max(resolution_m, 1e-6) + ci))
    j = int(round(y / max(resolution_m, 1e-6) + cj))
    i = int(np.clip(i, 0, n - 1))
    j = int(np.clip(j, 0, m - 1))
    return float(heightmap[i, j])


def _make_step_hard(patch: int, resolution_m: float, step_h_m: float, step_x_m: float = 0.22) -> tuple[np.ndarray, list[dict[str, float]]]:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    h[x_vals >= step_x_m, :] = float(step_h_m)
    obstacles = [
        {
            "x_m": float(step_x_m),
            "y_m": 0.0,
            "length_m": 0.04,
            "width_m": float((patch - 2) * resolution_m),
            "height_m": float(step_h_m),
        }
    ]
    return h, obstacles


def _compute_slip_p95(cmd: np.ndarray, actual: np.ndarray) -> float:
    cmd_abs = np.abs(cmd.astype(np.float64))
    act_abs = np.abs(actual.astype(np.float64))
    warm = int(0.2 * cmd_abs.size)
    cmd_abs = cmd_abs[warm:]
    act_abs = act_abs[warm:]
    valid = cmd_abs > 3e-2
    if not np.any(valid):
        return 0.0
    slip = np.abs(cmd_abs[valid] - act_abs[valid]) / (cmd_abs[valid] + 1e-6)
    return float(np.percentile(slip, 95))


def _step_transition_width(height_profile: np.ndarray, x_samples: np.ndarray) -> float:
    h_min = float(np.min(height_profile))
    h_max = float(np.max(height_profile))
    if h_max - h_min < 1e-4:
        return 0.0
    h10 = h_min + 0.1 * (h_max - h_min)
    h90 = h_min + 0.9 * (h_max - h_min)
    idx10 = np.flatnonzero(height_profile >= h10)
    idx90 = np.flatnonzero(height_profile >= h90)
    if idx10.size == 0 or idx90.size == 0:
        return float("nan")
    return float(max(0.0, x_samples[idx90[0]] - x_samples[idx10[0]]))


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    runner = make_runner("ros_gz", manager.sim_cfg)
    extractor = manager.label_extractor
    action_forward = next(a for a in manager.action_library if a.action_id == "a0")
    vehicle = next(v for v in manager.vehicle_library if v.vehicle_id == "urban_small")

    patch = int(manager.map_cfg["patch_size"])
    resolution_m = float(manager.map_cfg["resolution_m_per_cell"])
    friction_mu = 0.8

    step_cases = [
        ("step_5cm", 0.05),
        ("step_10cm", 0.10),
        ("step_15cm", 0.15),
    ]

    summary_rows: list[dict[str, object]] = []
    detail_payload: dict[str, object] = {"cases": {}}

    master_rng = np.random.default_rng(args.seed)
    for idx, (name, step_h) in enumerate(step_cases):
        step_x = 0.22
        heightmap, obstacles = _make_step_hard(patch, resolution_m, step_h_m=step_h, step_x_m=step_x)
        seed = int(master_rng.integers(0, 2**31 - 1))
        rng = np.random.default_rng(seed)
        context = SimulationContext(
            heightmap=heightmap,
            heading_rad=0.0,
            vehicle=vehicle,
            action=action_forward,
            friction_mu=friction_mu,
            motion_model="skid",
            sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
            duration_sec=float(manager.sim_cfg["action_duration_sec"]),
            settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
            extra_obstacles=obstacles,
        )

        fail_reason = ""
        row: dict[str, object] = {"terrain_type": name, "seed": seed}
        detail: dict[str, object] = {"seed": seed, "step_height_m": step_h, "step_x_m": step_x}
        try:
            traj = runner.run(context, rng)
            labels, _ = extractor.compute_labels(traj, vehicle, action_forward)

            centerline_x = np.linspace(-0.4, 0.6, 51, dtype=np.float64)
            centerline_h = np.array([_sample_height_nearest(heightmap, resolution_m, float(x), 0.0) for x in centerline_x], dtype=np.float64)
            transition_w = _step_transition_width(centerline_h, centerline_x)

            path_x = traj.positions_xy[:, 0].astype(np.float64)
            path_y = traj.positions_xy[:, 1].astype(np.float64)
            path_h = np.array(
                [_sample_height_nearest(heightmap, resolution_m, float(x), float(y)) for x, y in zip(path_x, path_y, strict=False)],
                dtype=np.float64,
            )

            # Estimate front-wheel crossing time for the step edge.
            wheel_offsets = runner._wheel_offsets(vehicle)  # pylint: disable=protected-access
            front_cross_time = float("nan")
            for t in range(traj.timestamps.shape[0]):
                rot = runner._rpy_to_rot(float(traj.roll_rad[t]), float(traj.pitch_rad[t]), float(traj.yaw_rad[t]))  # pylint: disable=protected-access
                model_pos = np.array([traj.positions_xy[t, 0], traj.positions_xy[t, 1], 0.0], dtype=np.float64)
                front_left = model_pos + rot @ np.array([wheel_offsets[0, 0], wheel_offsets[0, 1], 0.0], dtype=np.float64)
                front_right = model_pos + rot @ np.array([wheel_offsets[1, 0], wheel_offsets[1, 1], 0.0], dtype=np.float64)
                if max(front_left[0], front_right[0]) >= step_x:
                    front_cross_time = float(traj.timestamps[t])
                    break

            bottom_rate = float(np.mean(traj.chassis_contacts > 0))
            progress = float(traj.completed_displacement_m / max(action_forward.delta_s_m, 1e-6))
            pitch_max = float(np.max(np.abs(traj.pitch_rad)))
            slip_p95 = _compute_slip_p95(traj.commanded_linear_speed, traj.actual_linear_speed)
            chassis_min_clearance = float(np.min(traj.chassis_min_clearance_m)) if traj.chassis_min_clearance_m is not None else float("nan")
            wheel_min_clearance = (
                float(np.min(traj.wheel_clearance_m)) if traj.wheel_clearance_m is not None else float("nan")
            )

            row.update(
                {
                    "terrain_height_delta_m": float(np.max(heightmap) - np.min(heightmap)),
                    "centerline_height_delta_m": float(np.max(centerline_h) - np.min(centerline_h)),
                    "step_transition_width_m": float(transition_w),
                    "step_on_vehicle_path": bool(np.max(path_x) >= step_x and np.min(np.abs(path_y)) < 0.25),
                    "front_wheel_first_reach_step_time_s": front_cross_time,
                    "progress": progress,
                    "pitch_max_rad": pitch_max,
                    "slip_p95": slip_p95,
                    "bottom_contact_rate": bottom_rate,
                    "q_lift": float(labels.q_lift),
                    "q_pitch": float(labels.q_pitch),
                    "q_slip": float(labels.q_slip),
                    "chassis_min_clearance_m": chassis_min_clearance,
                    "wheel_min_clearance_m": wheel_min_clearance,
                    "fail_reason": "-",
                }
            )
            detail.update(
                {
                    "row_metrics": row,
                    "centerline_profile": [[float(x), float(h)] for x, h in zip(centerline_x, centerline_h, strict=False)],
                    "vehicle_path_profile": [
                        [float(x), float(y), float(h)] for x, y, h in zip(path_x, path_y, path_h, strict=False)
                    ],
                    "spawn_pose": {
                        "z": float(runner._last_spawn_z),  # pylint: disable=protected-access
                        "roll": float(runner._last_spawn_roll),  # pylint: disable=protected-access
                        "pitch": float(runner._last_spawn_pitch),  # pylint: disable=protected-access
                    },
                }
            )
        except Exception as exc:
            fail_reason = str(exc)
            row.update(
                {
                    "terrain_height_delta_m": float(np.max(heightmap) - np.min(heightmap)),
                    "centerline_height_delta_m": float("nan"),
                    "step_transition_width_m": float("nan"),
                    "step_on_vehicle_path": False,
                    "front_wheel_first_reach_step_time_s": float("nan"),
                    "progress": float("nan"),
                    "pitch_max_rad": float("nan"),
                    "slip_p95": float("nan"),
                    "bottom_contact_rate": float("nan"),
                    "q_lift": float("nan"),
                    "q_pitch": float("nan"),
                    "q_slip": float("nan"),
                    "chassis_min_clearance_m": float("nan"),
                    "wheel_min_clearance_m": float("nan"),
                    "fail_reason": fail_reason,
                }
            )
            detail["error"] = fail_reason

        summary_rows.append(row)
        detail_payload["cases"][name] = detail

    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "step_audit.json"
    md_path = args.output_dir / "step_audit.md"
    json_path.write_text(json.dumps(detail_payload, ensure_ascii=False, indent=2), encoding="utf-8")
    md_headers = [
        "terrain_type",
        "seed",
        "terrain_height_delta_m",
        "centerline_height_delta_m",
        "step_transition_width_m",
        "step_on_vehicle_path",
        "front_wheel_first_reach_step_time_s",
        "progress",
        "pitch_max_rad",
        "slip_p95",
        "bottom_contact_rate",
        "chassis_min_clearance_m",
        "wheel_min_clearance_m",
        "q_lift",
        "q_pitch",
        "q_slip",
        "fail_reason",
    ]
    md_path.write_text(_md_table(summary_rows, md_headers) + "\n", encoding="utf-8")

    try:
        runner.shutdown()
    except Exception:
        pass

    print(json.dumps({"json": str(json_path), "md": str(md_path)}, ensure_ascii=False))
    print(_md_table(summary_rows, md_headers))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

