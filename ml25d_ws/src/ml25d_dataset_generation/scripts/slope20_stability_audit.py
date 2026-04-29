#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit mountain_large + slope_forward_20deg stability")
    parser.add_argument("--repeats", type=int, default=30)
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/diagnostics/slope20_stability"),
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


def _tail(text: str, max_len: int = 600) -> str:
    return text if len(text) <= max_len else text[-max_len:]


def _slice_log(path: Path, start: int) -> str:
    if not path.exists():
        return ""
    text = path.read_text(encoding="utf-8", errors="replace")
    if start >= len(text):
        return ""
    return text[start:]


def _terrain_slope_x(patch: int, resolution_m: float, slope_deg: float) -> np.ndarray:
    slope = float(np.tan(np.deg2rad(slope_deg)))
    center = (patch - 1) / 2.0
    x = (np.arange(patch, dtype=np.float32) - center) * resolution_m
    return np.repeat((slope * x)[:, None], patch, axis=1).astype(np.float32)


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    runner = make_runner("ros_gz", manager.sim_cfg)
    vehicle = next(v for v in manager.vehicle_library if v.vehicle_id == "mountain_large")
    action_forward = next(a for a in manager.action_library if a.action_id == "a0")

    patch = int(manager.map_cfg["patch_size"])
    resolution_m = float(manager.map_cfg["resolution_m_per_cell"])
    heightmap = _terrain_slope_x(patch, resolution_m, 20.0)
    sample_rate_hz = int(manager.sim_cfg["sample_rate_hz"])
    duration_sec = float(manager.sim_cfg["action_duration_sec"])
    settle_time_sec = float(manager.sim_cfg["settle_time_sec"])

    gz_log = Path("/tmp/ml25d_ros_gz_logs/gzserver.log")
    rows: list[dict[str, object]] = []
    master_rng = np.random.default_rng(args.seed)
    for i in range(args.repeats):
        seed = int(master_rng.integers(0, 2**31 - 1))
        rng = np.random.default_rng(seed)
        start_len = len(gz_log.read_text(encoding="utf-8", errors="replace")) if gz_log.exists() else 0
        context = SimulationContext(
            heightmap=heightmap,
            heading_rad=0.0,
            vehicle=vehicle,
            action=action_forward,
            friction_mu=0.8,
            motion_model="skid",
            sample_rate_hz=sample_rate_hz,
            duration_sec=duration_sec,
            settle_time_sec=settle_time_sec,
        )

        row: dict[str, object] = {"trial": i + 1, "seed": seed}
        status = "success"
        fail_reason = "-"
        runtime_log = "-"
        has_nan = False
        timeout = False
        try:
            traj = runner.run(context, rng)
            has_nan = bool(
                np.isnan(traj.positions_xy).any()
                or np.isnan(traj.roll_rad).any()
                or np.isnan(traj.pitch_rad).any()
                or np.isnan(traj.actual_linear_speed).any()
            )
            row["progress"] = float(traj.completed_displacement_m / max(action_forward.delta_s_m, 1e-6))
        except Exception as exc:
            status = "runtime_failure"
            fail_reason = str(exc)
            timeout = ("timed out" in fail_reason.lower()) or ("timeout" in fail_reason.lower())
            runtime_log = _tail(fail_reason.replace("\n", " "))
            row["progress"] = float("nan")

        appended_log = _slice_log(gz_log, start_len)
        terrain_exists = "already exists" in appended_log.lower()

        spawn_z = float(getattr(runner, "_last_spawn_z", float("nan")))
        spawn_roll = float(getattr(runner, "_last_spawn_roll", float("nan")))
        spawn_pitch = float(getattr(runner, "_last_spawn_pitch", float("nan")))

        # Recompute nominal initial wheel clearances for diagnostics.
        pos_xy = np.array([[0.0, 0.0]], dtype=np.float32)
        pos_z = np.array([spawn_z], dtype=np.float32)
        roll = np.array([spawn_roll], dtype=np.float32)
        pitch = np.array([spawn_pitch], dtype=np.float32)
        yaw = np.array([0.0], dtype=np.float32)
        try:
            init_clear = runner._compute_wheel_clearance(  # pylint: disable=protected-access
                heightmap=heightmap,
                vehicle=vehicle,
                positions_xy=pos_xy,
                positions_z=pos_z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )[0]
            init_clearances = [float(v) for v in init_clear]
        except Exception:
            init_clearances = [float("nan")] * 4

        row.update(
            {
                "status": status,
                "spawn_z": spawn_z,
                "spawn_roll": spawn_roll,
                "spawn_pitch": spawn_pitch,
                "init_clear_fl": init_clearances[0],
                "init_clear_fr": init_clearances[1],
                "init_clear_rl": init_clearances[2],
                "init_clear_rr": init_clearances[3],
                "has_nan": has_nan,
                "timeout": timeout,
                "terrain_already_exists": terrain_exists,
                "fail_reason": fail_reason if status != "success" else "-",
                "runtime_failure_log": runtime_log if status != "success" else "-",
            }
        )
        rows.append(row)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "slope20_stability_audit.json"
    md_path = args.output_dir / "slope20_stability_audit.md"
    summary = {
        "repeats": args.repeats,
        "success_count": int(sum(1 for r in rows if r["status"] == "success")),
        "runtime_failure_count": int(sum(1 for r in rows if r["status"] != "success")),
        "rows": rows,
    }
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    headers = [
        "trial",
        "seed",
        "status",
        "spawn_z",
        "spawn_roll",
        "spawn_pitch",
        "init_clear_fl",
        "init_clear_fr",
        "init_clear_rl",
        "init_clear_rr",
        "has_nan",
        "timeout",
        "terrain_already_exists",
        "progress",
        "fail_reason",
    ]
    md_path.write_text(_md_table(rows, headers) + "\n", encoding="utf-8")

    try:
        runner.shutdown()
    except Exception:
        pass

    print(json.dumps({"json": str(json_path), "md": str(md_path)}, ensure_ascii=False))
    print(_md_table(rows, headers))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

