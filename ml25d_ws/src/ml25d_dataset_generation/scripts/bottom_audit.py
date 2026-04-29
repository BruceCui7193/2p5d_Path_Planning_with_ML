#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
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
    parser = argparse.ArgumentParser(description="Run bottom-specific audit scenarios")
    parser.add_argument("--repeats", type=int, default=10, help="Valid samples per scenario row")
    parser.add_argument("--max-retry", type=int, default=2, help="Retry count for invalid samples")
    parser.add_argument("--seed", type=int, default=20260426)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/diagnostics/bottom_audit"),
        help="Output directory for markdown/json artifacts",
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


def _terrain_low_bump(patch: int, resolution_m: float, bump_h_m: float = 0.03) -> np.ndarray:
    center = (patch - 1) / 2.0
    xy = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    xx, yy = np.meshgrid(xy, xy, indexing="ij")
    bump = bump_h_m * np.exp(-((xx - 0.14) ** 2 + yy**2) / max(2.0 * 0.06 * 0.06, 1e-6))
    return bump.astype(np.float32)


def _terrain_central_ridge(patch: int, resolution_m: float, ridge_h_m: float = 0.12) -> np.ndarray:
    h = np.zeros((patch, patch), dtype=np.float32)
    center = (patch - 1) / 2.0
    x_vals = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    mask = np.abs(x_vals - 0.12) <= 0.04
    h[mask, :] = ridge_h_m
    return h


def _scenario_builders() -> dict[str, ScenarioSpec]:
    return {
        "flat_forward": ScenarioSpec("flat_forward", lambda p, r: (_terrain_flat(p, r), [])),
        "slope5": ScenarioSpec("slope5", lambda p, r: (_terrain_slope_x(p, r, 5.0), [])),
        "slope10": ScenarioSpec("slope10", lambda p, r: (_terrain_slope_x(p, r, 10.0), [])),
        "cross10": ScenarioSpec("cross10", lambda p, r: (_terrain_slope_y(p, r, 10.0), [])),
        "cross20": ScenarioSpec("cross20", lambda p, r: (_terrain_slope_y(p, r, 20.0), [])),
        "low_bump": ScenarioSpec("low_bump", lambda p, r: (_terrain_low_bump(p, r, bump_h_m=0.03), [])),
        "central_ridge": ScenarioSpec("central_ridge", lambda p, r: (_terrain_central_ridge(p, r, ridge_h_m=0.12), [])),
        "step15": ScenarioSpec("step15", lambda p, r: _terrain_step_hard(p, r, 0.15)),
    }


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


def _eval_fail_reasons(
    labels,
    thresholds,
) -> list[str]:
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


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    runner = make_runner("ros_gz", manager.sim_cfg)
    extractor = manager.label_extractor
    thresholds = extractor.thresholds

    patch = int(manager.map_cfg["patch_size"])
    resolution_m = float(manager.map_cfg["resolution_m_per_cell"])
    friction_mu = 0.8
    sample_rate_hz = int(manager.sim_cfg["sample_rate_hz"])
    settle_time_sec = float(manager.sim_cfg["settle_time_sec"])
    action_forward = next(a for a in manager.action_library if a.action_id == "a0")

    vehicle_map: dict[str, VehicleParams] = {
        "city_small": next(v for v in manager.vehicle_library if v.vehicle_id == "urban_small"),
        "offroad_medium": next(v for v in manager.vehicle_library if v.vehicle_id == "standard_offroad"),
        "mountain_large": next(v for v in manager.vehicle_library if v.vehicle_id == "mountain_large"),
    }
    scenario_builders = _scenario_builders()

    # Requested audit rows.
    rows_plan: list[tuple[str, str]] = []
    for terrain_name in ["flat_forward", "slope5", "slope10", "cross10", "cross20", "low_bump", "central_ridge"]:
        for vehicle_type in ["city_small", "offroad_medium", "mountain_large"]:
            rows_plan.append((vehicle_type, terrain_name))
    rows_plan.extend(
        [
            ("city_small", "step15"),
            ("mountain_large", "step15"),
        ]
    )

    all_rows: list[dict[str, object]] = []
    detail_records: list[dict[str, object]] = []
    master_rng = np.random.default_rng(args.seed)

    total = len(rows_plan)
    for idx, (vehicle_type, terrain_name) in enumerate(rows_plan, start=1):
        print(f"[bottom_audit] combo {idx}/{total} start vehicle={vehicle_type} terrain={terrain_name}", flush=True)
        vehicle = vehicle_map[vehicle_type]
        scenario = scenario_builders[terrain_name]

        valid = 0
        logical_attempts = 0
        max_attempt_budget = args.repeats * max(args.max_retry + 3, 8)
        dropped_invalid = 0
        retry_count = 0
        fail_counter: Counter[str] = Counter()

        bottom_fail_flags: list[float] = []
        bottom_duration_values: list[float] = []
        min_clearance_means: list[float] = []
        min_clearance_p5s: list[float] = []
        min_clearance_mins: list[float] = []
        progress_values: list[float] = []
        pitch_max_values: list[float] = []
        roll_max_values: list[float] = []

        while valid < args.repeats and logical_attempts < max_attempt_budget:
            logical_attempts += 1
            accepted = False
            for retry_idx in range(args.max_retry + 1):
                if retry_idx > 0:
                    retry_count += 1

                seed = int(master_rng.integers(0, 2**31 - 1))
                rng = np.random.default_rng(seed)
                terrain_map, obstacles = scenario.build(patch, resolution_m)
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
                )

                try:
                    traj = runner.run(context, rng)
                except Exception as exc:
                    msg = str(exc)
                    if _is_invalid_contact_error(msg):
                        if retry_idx == args.max_retry:
                            dropped_invalid += 1
                        continue
                    dropped_invalid += 1
                    break

                labels, _ = extractor.compute_labels(traj, vehicle, action_forward)
                btm = extractor.bottom_metrics(traj)
                valid += 1
                accepted = True

                sample_reasons = _eval_fail_reasons(labels, thresholds)
                for reason in sample_reasons:
                    fail_counter[reason] += 1

                bottom_fail_flags.append(float(labels.p_bottom > thresholds.bottom_fail_threshold))
                bottom_duration_values.append(float(btm["bottom_duration_ratio"]))
                min_clearance_means.append(float(btm["min_clearance_mean"]))
                min_clearance_p5s.append(float(btm["min_clearance_p5"]))
                min_clearance_mins.append(float(btm["min_clearance_min"]))
                progress_values.append(float(traj.completed_displacement_m / max(action_forward.delta_s_m, 1e-6)))
                pitch_max_values.append(float(np.max(np.abs(traj.pitch_rad))))
                roll_max_values.append(float(np.max(np.abs(traj.roll_rad))))

                detail_records.append(
                    {
                        "vehicle_type": vehicle_type,
                        "terrain_type": terrain_name,
                        "seed": seed,
                        "bottom_duration_ratio": float(btm["bottom_duration_ratio"]),
                        "bottom_clearance_duration_ratio": float(btm["bottom_clearance_duration_ratio"]),
                        "bottom_contact_duration_ratio": float(btm["bottom_contact_duration_ratio"]),
                        "bottom_contact_duration_ratio_raw": float(btm["bottom_contact_duration_ratio_raw"]),
                        "min_clearance_mean": float(btm["min_clearance_mean"]),
                        "min_clearance_p5": float(btm["min_clearance_p5"]),
                        "min_clearance_min": float(btm["min_clearance_min"]),
                        "q_roll": float(labels.q_roll),
                        "q_pitch": float(labels.q_pitch),
                        "q_slip": float(labels.q_slip),
                        "q_lift": float(labels.q_lift),
                        "p_bottom": float(labels.p_bottom),
                        "p_stuck": float(labels.p_stuck),
                        "fail_reasons": sample_reasons,
                        "progress": float(progress_values[-1]),
                        "pitch_max": float(pitch_max_values[-1]),
                        "roll_max": float(roll_max_values[-1]),
                    }
                )
                break

            if accepted and (valid <= 2 or valid % 5 == 0 or valid == args.repeats):
                print(
                    "[bottom_audit] progress "
                    f"vehicle={vehicle_type} terrain={terrain_name} "
                    f"valid={valid}/{args.repeats} invalid={dropped_invalid} retries={retry_count}",
                    flush=True,
                )

        row: dict[str, object] = {
            "vehicle_type": vehicle_type,
            "terrain_type": terrain_name,
            "bottom_fail_rate": float(np.mean(bottom_fail_flags)) if bottom_fail_flags else float("nan"),
            "bottom_duration_ratio_mean": float(np.mean(bottom_duration_values)) if bottom_duration_values else float("nan"),
            "bottom_duration_ratio_p95": float(np.percentile(bottom_duration_values, 95)) if bottom_duration_values else float("nan"),
            "min_clearance_mean": float(np.mean(min_clearance_means)) if min_clearance_means else float("nan"),
            "min_clearance_p5": float(np.percentile(min_clearance_p5s, 5)) if min_clearance_p5s else float("nan"),
            "min_clearance_min": float(np.min(min_clearance_mins)) if min_clearance_mins else float("nan"),
            "fail_reason": dict(sorted(fail_counter.items())),
            "progress_mean": float(np.mean(progress_values)) if progress_values else float("nan"),
            "pitch_max": float(np.mean(pitch_max_values)) if pitch_max_values else float("nan"),
            "roll_max": float(np.mean(roll_max_values)) if roll_max_values else float("nan"),
            "invalid_sample_rate": float(dropped_invalid / max(dropped_invalid + len(bottom_fail_flags), 1)),
            "retry_count": int(retry_count),
            "valid_count": int(len(bottom_fail_flags)),
        }
        all_rows.append(row)
        print(
            f"[bottom_audit] combo done vehicle={vehicle_type} terrain={terrain_name} "
            f"bottom_fail_rate={row['bottom_fail_rate']:.3f} invalid_rate={row['invalid_sample_rate']:.3f}",
            flush=True,
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "bottom_audit_summary.json"
    md_path = args.output_dir / "bottom_audit_summary.md"
    detail_path = args.output_dir / "bottom_audit_samples.jsonl"

    json_path.write_text(
        json.dumps(
            {
                "repeats": args.repeats,
                "max_retry": args.max_retry,
                "rows": all_rows,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    detail_path.write_text(
        "\n".join(json.dumps(rec, ensure_ascii=False) for rec in detail_records) + ("\n" if detail_records else ""),
        encoding="utf-8",
    )

    md_headers = [
        "vehicle_type",
        "terrain_type",
        "bottom_fail_rate",
        "bottom_duration_ratio_mean",
        "bottom_duration_ratio_p95",
        "min_clearance_mean",
        "min_clearance_p5",
        "min_clearance_min",
        "fail_reason",
        "progress_mean",
        "pitch_max",
        "roll_max",
        "invalid_sample_rate",
        "retry_count",
    ]
    md_path.write_text(_to_md_table(all_rows, md_headers) + "\n", encoding="utf-8")

    try:
        runner.shutdown()
    except Exception:
        pass

    print(json.dumps({"json": str(json_path), "md": str(md_path), "samples": str(detail_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

