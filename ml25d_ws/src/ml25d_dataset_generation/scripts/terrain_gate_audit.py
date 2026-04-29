#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from ml25d_dataset_generation.common_types import VehicleParams
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import RosGzSimulationRunner, SimulationContext, make_runner


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    build: Callable[[int, float], tuple[np.ndarray, list[dict[str, float]]]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Terrain-aware start-gate audit (reset -> spawn -> settle -> gate only)")
    parser.add_argument("--output-dir", type=Path, default=Path("data/diagnostics/terrain_gate_audit"))
    parser.add_argument("--repeats", type=int, default=5, help="Runs per (vehicle, terrain)")
    parser.add_argument("--seed", type=int, default=20260426)
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz"])
    parser.add_argument(
        "--vehicles",
        type=str,
        default="city_small,offroad_medium,mountain_large",
        help="Comma-separated logical vehicle types",
    )
    parser.add_argument(
        "--terrains",
        type=str,
        default="flat,slope_forward_20deg,cross_slope_20deg,step_15cm_approach,pit_medium_approach,trench_forward,bump_field",
        help="Comma-separated terrain names",
    )
    return parser.parse_args()


def _parse_csv(text: str) -> list[str]:
    return [x.strip() for x in str(text).split(",") if x.strip()]


def _terrain_flat(patch: int, _: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    return np.zeros((patch, patch), dtype=np.float32), []


def _terrain_slope_x(patch: int, resolution_m: float, slope_deg: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    slope = float(np.tan(np.deg2rad(slope_deg)))
    center = (patch - 1) / 2.0
    x = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    h = np.repeat((slope * x)[:, None], patch, axis=1).astype(np.float32)
    return h, []


def _terrain_slope_y(patch: int, resolution_m: float, slope_deg: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    slope = float(np.tan(np.deg2rad(slope_deg)))
    center = (patch - 1) / 2.0
    y = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    h = np.repeat((slope * y)[None, :], patch, axis=0).astype(np.float32)
    return h, []


def _terrain_step_hard(
    patch: int,
    resolution_m: float,
    step_h_m: float,
    *,
    step_x_m: float = 0.22,
) -> tuple[np.ndarray, list[dict[str, float]]]:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    step_x = float(step_x_m)
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


def _terrain_trench_center(patch: int, resolution_m: float, width_m: float, depth_m: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    mask = np.abs(x_vals - 0.12) <= (0.5 * width_m)
    h[mask, :] = -float(depth_m)
    return h, []


def _terrain_pit_gaussian(
    patch: int,
    resolution_m: float,
    sigma_m: float,
    depth_m: float,
    *,
    center_x_m: float = 0.12,
) -> tuple[np.ndarray, list[dict[str, float]]]:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    pit = -float(depth_m) * np.exp(-((xx - float(center_x_m)) ** 2 + yy**2) / max(2.0 * sigma_m * sigma_m, 1e-6))
    rough = 0.003 * (np.sin(7.0 * xx + 0.5) * np.sin(6.0 * yy - 0.4))
    return (pit + rough).astype(np.float32), []


def _terrain_bump_field(patch: int, resolution_m: float) -> tuple[np.ndarray, list[dict[str, float]]]:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    bumps = (
        0.030 * np.exp(-((xx - 0.4) ** 2 + (yy + 0.3) ** 2) / 0.10)
        + 0.025 * np.exp(-((xx + 0.5) ** 2 + (yy - 0.2) ** 2) / 0.08)
        + 0.020 * np.exp(-((xx + 0.1) ** 2 + (yy + 0.5) ** 2) / 0.06)
    )
    micro = 0.006 * (np.sin(8.0 * xx) * np.cos(6.0 * yy))
    return (bumps + micro).astype(np.float32), []


def _scenario_builders() -> dict[str, ScenarioSpec]:
    return {
        "flat": ScenarioSpec("flat", lambda p, r: _terrain_flat(p, r)),
        "slope_forward_20deg": ScenarioSpec("slope_forward_20deg", lambda p, r: _terrain_slope_x(p, r, 20.0)),
        "cross_slope_20deg": ScenarioSpec("cross_slope_20deg", lambda p, r: _terrain_slope_y(p, r, 20.0)),
        "step_15cm_approach": ScenarioSpec("step_15cm_approach", lambda p, r: _terrain_step_hard(p, r, 0.15, step_x_m=0.55)),
        "pit_medium_approach": ScenarioSpec(
            "pit_medium_approach",
            lambda p, r: _terrain_pit_gaussian(p, r, sigma_m=0.14, depth_m=0.15, center_x_m=0.55),
        ),
        "trench_forward": ScenarioSpec("trench_forward", lambda p, r: _terrain_trench_center(p, r, width_m=0.14, depth_m=0.10)),
        "bump_field": ScenarioSpec("bump_field", lambda p, r: _terrain_bump_field(p, r)),
    }


def _as_float(v: object, default: float = float("nan")) -> float:
    try:
        x = float(v)
    except Exception:
        return default
    return x


def _mean(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def _p95(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), 95))


def _to_md_table(rows: list[dict[str, object]], headers: list[str]) -> str:
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        cells: list[str] = []
        for h in headers:
            val = row.get(h, "")
            if isinstance(val, float):
                if np.isnan(val):
                    cells.append("NaN")
                else:
                    cells.append(f"{val:.4f}")
            else:
                cells.append(str(val))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _vehicle_map(manager: DatasetManager) -> dict[str, VehicleParams]:
    return {
        "city_small": next(v for v in manager.vehicle_library if v.vehicle_id == "urban_small"),
        "offroad_medium": next(v for v in manager.vehicle_library if v.vehicle_id == "standard_offroad"),
        "mountain_large": next(v for v in manager.vehicle_library if v.vehicle_id == "mountain_large"),
    }


def main() -> int:
    args = parse_args()
    if args.repeats <= 0:
        raise ValueError("repeats must be positive")

    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    runnersim = make_runner(args.backend, manager.sim_cfg)
    if not isinstance(runnersim, RosGzSimulationRunner):
        raise RuntimeError("terrain_gate_audit requires ros_gz runner")
    runner = runnersim

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_jsonl = output_dir / "terrain_gate_audit_samples.jsonl"
    samples_jsonl.write_text("", encoding="utf-8")

    scenarios = _scenario_builders()
    selected_terrains = _parse_csv(args.terrains)
    invalid_terrains = sorted(set(selected_terrains) - set(scenarios.keys()))
    if invalid_terrains:
        raise ValueError(f"unknown terrains: {invalid_terrains}")

    vehicle_map = _vehicle_map(manager)
    selected_vehicles = _parse_csv(args.vehicles)
    invalid_vehicles = sorted(set(selected_vehicles) - set(vehicle_map.keys()))
    if invalid_vehicles:
        raise ValueError(f"unknown vehicles: {invalid_vehicles}")

    action_forward = next(a for a in manager.action_library if a.action_id == "a0")
    patch = int(manager.map_cfg["patch_size"])
    resolution_m = float(manager.map_cfg["resolution_m_per_cell"])
    sample_rate_hz = int(manager.sim_cfg["sample_rate_hz"])
    settle_time_sec = float(manager.sim_cfg["settle_time_sec"])
    duration_sec = float(manager.sim_cfg["action_duration_sec"])

    rng = np.random.default_rng(int(args.seed))
    rows: list[dict[str, object]] = []
    all_samples: list[dict[str, object]] = []
    total = len(selected_vehicles) * len(selected_terrains)
    combo_idx = 0

    try:
        for vehicle_type in selected_vehicles:
            vehicle = vehicle_map[vehicle_type]
            for terrain_name in selected_terrains:
                combo_idx += 1
                print(
                    f"[gate-audit] combo {combo_idx}/{total} start vehicle={vehicle_type} terrain={terrain_name}",
                    flush=True,
                )
                scenario = scenarios[terrain_name]

                gate_pass_flags: list[float] = []
                init_invalid_flags: list[float] = []
                settle_values: list[float] = []
                roll_expected_values: list[float] = []
                roll_actual_values: list[float] = []
                roll_error_world_values: list[float] = []
                roll_error_gate_values: list[float] = []
                pitch_expected_values: list[float] = []
                pitch_actual_values: list[float] = []
                pitch_error_world_values: list[float] = []
                pitch_error_gate_values: list[float] = []
                linear_values: list[float] = []
                angular_values: list[float] = []
                bottom_flags: list[float] = []
                lift_flags: list[float] = []
                msg_valid_flags: list[float] = []
                error_counter: dict[str, int] = {}

                for rep in range(args.repeats):
                    seed = int(rng.integers(0, 2**31 - 1))
                    terrain_map, obstacles = scenario.build(patch, resolution_m)
                    context = SimulationContext(
                        heightmap=terrain_map,
                        heading_rad=0.0,
                        vehicle=vehicle,
                        action=action_forward,
                        friction_mu=0.8,
                        motion_model="skid",
                        sample_rate_hz=sample_rate_hz,
                        duration_sec=duration_sec,
                        settle_time_sec=settle_time_sec,
                        extra_obstacles=obstacles,
                        scene_id=f"gate_{vehicle_type}_{terrain_name}_{rep}",
                    )

                    try:
                        rec = runner.audit_start_gate(context)
                    except Exception as exc:
                        rec = {
                            "gate_pass": False,
                            "initialization_invalid": True,
                            "settle_time_sec": float("nan"),
                            "bottom_before_action": False,
                            "lift_before_action": False,
                            "message_time_valid": False,
                            "error": str(exc),
                        }

                    gate_pass = bool(rec.get("gate_pass", False))
                    init_invalid = bool(rec.get("initialization_invalid", True))
                    error_text = str(rec.get("error", ""))
                    error_key = "none" if gate_pass else ("start_gate_failed" if "start stability gate failed" in error_text else "runtime_failure")
                    error_counter[error_key] = error_counter.get(error_key, 0) + 1

                    sample_row: dict[str, object] = {
                        "vehicle_type": vehicle_type,
                        "terrain_type": terrain_name,
                        "repeat_idx": rep,
                        "seed": seed,
                        "gate_pass": gate_pass,
                        "initialization_invalid": init_invalid,
                        "settle_time_sec": _as_float(rec.get("settle_time_sec")),
                        "quat_x": _as_float(rec.get("quat_x")),
                        "quat_y": _as_float(rec.get("quat_y")),
                        "quat_z": _as_float(rec.get("quat_z")),
                        "quat_w": _as_float(rec.get("quat_w")),
                        "roll_expected_deg": _as_float(rec.get("roll_expected_deg")),
                        "roll_actual_deg": _as_float(rec.get("roll_actual_deg")),
                        "roll_error_world_deg": _as_float(rec.get("roll_error_world_deg")),
                        "roll_error_gate_deg": _as_float(rec.get("roll_error_gate_deg")),
                        "roll_gate_deg": _as_float(rec.get("roll_gate_deg")),
                        "pitch_expected_deg": _as_float(rec.get("pitch_expected_deg")),
                        "pitch_actual_deg": _as_float(rec.get("pitch_actual_deg")),
                        "pitch_error_world_deg": _as_float(rec.get("pitch_error_world_deg")),
                        "pitch_error_gate_deg": _as_float(rec.get("pitch_error_gate_deg")),
                        "pitch_gate_deg": _as_float(rec.get("pitch_gate_deg")),
                        "linear_speed": _as_float(rec.get("linear_speed")),
                        "angular_speed": _as_float(rec.get("angular_speed")),
                        "bottom_before_action": bool(rec.get("bottom_before_action", False)),
                        "lift_before_action": bool(rec.get("lift_before_action", False)),
                        "message_time_valid": bool(rec.get("message_time_valid", False)),
                        "odom_planar_fallback": bool(_as_float(rec.get("odom_planar_fallback", 0.0), 0.0) > 0.5),
                        "error": error_text,
                    }
                    all_samples.append(sample_row)

                    gate_pass_flags.append(1.0 if gate_pass else 0.0)
                    init_invalid_flags.append(1.0 if init_invalid else 0.0)
                    settle_values.append(_as_float(sample_row["settle_time_sec"]))
                    roll_expected_values.append(_as_float(sample_row["roll_expected_deg"]))
                    roll_actual_values.append(_as_float(sample_row["roll_actual_deg"]))
                    roll_error_world_values.append(_as_float(sample_row["roll_error_world_deg"]))
                    roll_error_gate_values.append(_as_float(sample_row["roll_error_gate_deg"]))
                    pitch_expected_values.append(_as_float(sample_row["pitch_expected_deg"]))
                    pitch_actual_values.append(_as_float(sample_row["pitch_actual_deg"]))
                    pitch_error_world_values.append(_as_float(sample_row["pitch_error_world_deg"]))
                    pitch_error_gate_values.append(_as_float(sample_row["pitch_error_gate_deg"]))
                    linear_values.append(_as_float(sample_row["linear_speed"]))
                    angular_values.append(_as_float(sample_row["angular_speed"]))
                    bottom_flags.append(1.0 if bool(sample_row["bottom_before_action"]) else 0.0)
                    lift_flags.append(1.0 if bool(sample_row["lift_before_action"]) else 0.0)
                    msg_valid_flags.append(1.0 if bool(sample_row["message_time_valid"]) else 0.0)

                row = {
                    "vehicle_type": vehicle_type,
                    "terrain_type": terrain_name,
                    "gate_pass_rate": _mean(gate_pass_flags),
                    "initialization_invalid_rate": _mean(init_invalid_flags),
                    "settle_time_mean": _mean(settle_values),
                    "settle_time_p95": _p95(settle_values),
                    "roll_expected_mean": _mean(roll_expected_values),
                    "roll_actual_mean": _mean(roll_actual_values),
                    "roll_error_world_mean": _mean(roll_error_world_values),
                    "roll_error_world_p95": _p95(roll_error_world_values),
                    "roll_error_gate_mean": _mean(roll_error_gate_values),
                    "roll_error_gate_p95": _p95(roll_error_gate_values),
                    "pitch_expected_mean": _mean(pitch_expected_values),
                    "pitch_actual_mean": _mean(pitch_actual_values),
                    "pitch_error_world_mean": _mean(pitch_error_world_values),
                    "pitch_error_world_p95": _p95(pitch_error_world_values),
                    "pitch_error_gate_mean": _mean(pitch_error_gate_values),
                    "pitch_error_gate_p95": _p95(pitch_error_gate_values),
                    "linear_speed_mean": _mean(linear_values),
                    "linear_speed_p95": _p95(linear_values),
                    "angular_speed_mean": _mean(angular_values),
                    "angular_speed_p95": _p95(angular_values),
                    "bottom_before_action_rate": _mean(bottom_flags),
                    "lift_before_action_rate": _mean(lift_flags),
                    "message_time_valid_rate": _mean(msg_valid_flags),
                    "error_distribution": error_counter,
                }
                rows.append(row)
                print(
                    "[gate-audit] combo done "
                    f"vehicle={vehicle_type} terrain={terrain_name} gate_pass_rate={row['gate_pass_rate']:.3f} "
                    f"init_invalid_rate={row['initialization_invalid_rate']:.3f}",
                    flush=True,
                )
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass

    with samples_jsonl.open("w", encoding="utf-8") as fp:
        for rec in all_samples:
            fp.write(json.dumps(rec, ensure_ascii=False) + "\n")

    summary = {
        "meta": {
            "seed": int(args.seed),
            "repeats": int(args.repeats),
            "vehicles": selected_vehicles,
            "terrains": selected_terrains,
            "sample_count": len(all_samples),
            "combo_count": len(rows),
        },
        "rows": rows,
    }
    summary_path = output_dir / "terrain_gate_audit_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    headers = [
        "vehicle_type",
        "terrain_type",
        "gate_pass_rate",
        "initialization_invalid_rate",
        "settle_time_mean",
        "settle_time_p95",
        "roll_expected_mean",
        "roll_actual_mean",
        "roll_error_world_mean",
        "roll_error_world_p95",
        "roll_error_gate_mean",
        "roll_error_gate_p95",
        "pitch_expected_mean",
        "pitch_actual_mean",
        "pitch_error_world_mean",
        "pitch_error_world_p95",
        "pitch_error_gate_mean",
        "pitch_error_gate_p95",
        "linear_speed_mean",
        "angular_speed_mean",
        "bottom_before_action_rate",
        "lift_before_action_rate",
        "message_time_valid_rate",
    ]
    md = []
    md.append("# Terrain Gate Audit")
    md.append("")
    md.append(f"sample_count={len(all_samples)}, combos={len(rows)}, repeats={args.repeats}")
    md.append("")
    md.append(_to_md_table(rows, headers))
    md_path = output_dir / "terrain_gate_audit_summary.md"
    md_path.write_text("\n".join(md), encoding="utf-8")

    print(f"[gate-audit] summary json: {summary_path}")
    print(f"[gate-audit] summary md:   {md_path}")
    print(f"[gate-audit] samples:      {samples_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
