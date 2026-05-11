#!/usr/bin/env python3
"""Run a hand-picked sample with exact terrain/vehicle/action/friction for screenshots.

Usage:
  cd ~/文档/Machine_Learning_25D/ml25d_ws
  source /opt/ros/jazzy/setup.bash
  export ROS_LOG_DIR=/tmp/ml25d_ros_logs
  export GZ_LOG_PATH=/tmp/ml25d_gz_logs
  PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
    src/ml25d_dataset_generation/scripts/demo_pick_sample.py \
    --terrain slope_bumps --vehicle standard_offroad --action a0 --friction grass_soft

To list available options:
  PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
    src/ml25d_dataset_generation/scripts/demo_pick_sample.py --list
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.config_loader import (
    build_action_library,
    build_vehicle_library,
    load_all_configs,
    weighted_table,
)
from ml25d_dataset_generation.gazebo_runner import (
    RosGzRuntimeConfig,
    RosGzSimulationRunner,
    SimulationContext,
    StartGateError,
    make_runner,
)
from ml25d_dataset_generation.label_extractor import LabelExtractor
from ml25d_dataset_generation.sample_packager import SamplePackager
from ml25d_dataset_generation.terrain_generator import TerrainGenerator


TERRAIN_CN: dict[str, str] = {
    "flat": "平坦", "uniform_slope": "纵坡", "lateral_slope": "横坡",
    "steps": "台阶", "pits": "坑洞", "bumps": "凸起", "waves": "波浪",
    "slope_bumps": "斜坡+凸起", "lateral_pits": "侧坡+坑洞", "mixed_random": "混合",
}
VEHICLE_CN: dict[str, str] = {
    "urban_small": "小型城市车", "standard_offroad": "标准越野车", "mountain_large": "大型山地车",
}
FRICTION_CN: dict[str, str] = {
    "dry_hard": "干燥硬地", "grass_soft": "草地软土", "wet_muddy": "湿滑泥地", "mixed": "混合",
}
BAND_CN: dict[str, str] = {
    "safe": "SAFE ✓", "fail": "FAIL ✗", "critical": "CRITICAL ⚠",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run a hand-picked sample for screenshots")
    p.add_argument("--terrain", type=str, default="slope_bumps", help="Terrain class name")
    p.add_argument("--vehicle", type=str, default="standard_offroad", help="Vehicle id")
    p.add_argument("--action", type=str, default="a0", help="Action id (a0/a1/a2)")
    p.add_argument("--friction", type=str, default="grass_soft", help="Friction class name")
    p.add_argument("--seed", type=int, default=None, help="Random seed (random if unset)")
    p.add_argument("--settle-sec", type=float, default=4.0, help="Settle time for screenshot")
    p.add_argument("--duration-sec", type=float, default=2.0, help="Action duration")
    p.add_argument("--heading-deg", type=float, default=None, help="Vehicle heading in degrees")
    p.add_argument("--list", action="store_true", help="List available options and exit")
    p.add_argument("--output-dir", type=Path, default=Path("data/demo_screenshots"))
    p.add_argument("--config-dir", type=Path, default=None)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    config_dir = args.config_dir or (package_root / "config")

    cfgs = load_all_configs(config_dir)

    terrain_classes = {c["name"]: c for c in cfgs["terrain"]["terrain"]["classes"]}
    vehicle_lib = {v.vehicle_id: v for v in build_vehicle_library(cfgs["vehicles"])}
    action_lib = {a.action_id: a for a in build_action_library(cfgs["actions"]) if a.delta_s_m > 1e-4}
    friction_classes = {c["name"]: c for c in cfgs["friction"]["friction"]["classes"]}

    if args.list:
        print("Available terrains:", ", ".join(terrain_classes))
        print("Available vehicles:", ", ".join(vehicle_lib))
        print("Available actions:", ", ".join(action_lib))
        print("Available frictions:", ", ".join(friction_classes))
        return 0

    # Validate
    if args.terrain not in terrain_classes:
        print(f"Unknown terrain: {args.terrain}. Available: {list(terrain_classes)}")
        return 1
    if args.vehicle not in vehicle_lib:
        print(f"Unknown vehicle: {args.vehicle}. Available: {list(vehicle_lib)}")
        return 1
    if args.action not in action_lib:
        print(f"Unknown action: {args.action}. Available: {list(action_lib)}")
        return 1
    if args.friction not in friction_classes:
        print(f"Unknown friction: {args.friction}. Available: {list(friction_classes)}")
        return 1

    terrain_class = terrain_classes[args.terrain]
    vehicle = vehicle_lib[args.vehicle]
    action = action_lib[args.action]
    friction_class = friction_classes[args.friction]

    seed = args.seed if args.seed is not None else int(np.random.default_rng().integers(0, 2**31 - 1))
    rng = np.random.default_rng(seed)
    heading_deg = args.heading_deg if args.heading_deg is not None else float(rng.uniform(0, 360))
    heading_rad = float(np.deg2rad(heading_deg))

    # Generate terrain
    patch_size = int(cfgs["dataset"]["map"]["patch_size"])
    resolution_m = float(cfgs["dataset"]["map"]["resolution_m_per_cell"])
    tg = TerrainGenerator(patch_size=patch_size, resolution_m=resolution_m)
    terrain = tg.generate(rng, terrain_class, travel_heading_rad=heading_rad)

    # Friction mu
    mu_lo, mu_hi = friction_class["mu_range"]
    friction_mu = float(rng.uniform(mu_lo, mu_hi))

    print("=" * 70)
    print("  自定义样本 — 截图模式")
    print("=" * 70)
    print(f"  地形: {TERRAIN_CN.get(args.terrain, args.terrain)} ({args.terrain})")
    print(f"  车辆: {VEHICLE_CN.get(args.vehicle, args.vehicle)} ({args.vehicle})")
    print(f"  动作: {args.action} (直行)" if args.action == "a0" else f"  动作: {args.action}")
    print(f"  摩擦: {FRICTION_CN.get(args.friction, args.friction)} μ={friction_mu:.3f}")
    print(f"  朝向: {heading_deg:.0f}°")
    print(f"  种子: {seed}")
    print()

    # Setup runner with GUI
    sim_cfg = cfgs["dataset"]["simulation"]
    sim_cfg.setdefault("ros_gz", {})
    sim_cfg["ros_gz"]["headless"] = False
    sim_cfg["ros_gz"]["world_sdf_file"] = "worlds/ml25d_demo.sdf"
    sim_cfg["settle_time_sec"] = float(args.settle_sec)
    sim_cfg["action_duration_sec"] = float(args.duration_sec)

    runner = make_runner("ros_gz", sim_cfg)

    context = SimulationContext(
        heightmap=terrain.heightmap,
        heading_rad=heading_rad,
        vehicle=vehicle,
        action=action,
        friction_mu=friction_mu,
        motion_model="skid",
        sample_rate_hz=int(sim_cfg["sample_rate_hz"]),
        duration_sec=float(args.duration_sec),
        settle_time_sec=float(args.settle_sec),
        cmd_ramp_sec=float(sim_cfg.get("cmd_ramp_sec", 0.3)),
    )

    # Run with retry
    max_retries = 8
    attempt = 0
    trajectory = None
    while attempt <= max_retries:
        try:
            trajectory = runner.run(context, rng)
            break
        except StartGateError as exc:
            d = exc.diag
            reasons = []
            if float(d.get("bottom_flag", 0)) > 0.5:
                reasons.append("底盘触地")
            if float(d.get("lift_before_action", 0)) > 0.5:
                reasons.append("车轮悬空")
            print(f"  ⚠ 稳定门失败 ({'; '.join(reasons) if reasons else '不稳定'}) → 重试 {attempt+1}/{max_retries}")
            attempt += 1
            if attempt <= max_retries:
                heading_deg = float(rng.uniform(0, 360))
                heading_rad = float(np.deg2rad(heading_deg))
                terrain = tg.generate(rng, terrain_class, travel_heading_rad=heading_rad)
                context = SimulationContext(
                    heightmap=terrain.heightmap,
                    heading_rad=heading_rad,
                    vehicle=vehicle,
                    action=action,
                    friction_mu=friction_mu,
                    motion_model="skid",
                    sample_rate_hz=int(sim_cfg["sample_rate_hz"]),
                    duration_sec=float(args.duration_sec),
                    settle_time_sec=float(args.settle_sec),
                    cmd_ramp_sec=float(sim_cfg.get("cmd_ramp_sec", 0.3)),
                )
                time.sleep(1.5)
        except Exception as exc:
            print(f"  ✗ 异常: {exc}")
            attempt += 1
            time.sleep(1.5)

    if trajectory is None:
        print("  ✗ 重试耗尽，样本生成失败")
        return 1

    label_extractor = LabelExtractor(cfgs["labels"])
    labels, band = label_extractor.compute_labels(trajectory, vehicle, action)

    print(f"  结果: {BAND_CN.get(band, band)}")
    print(f"  roll={labels.q_roll:.3f} pitch={labels.q_pitch:.3f} slip={labels.q_slip:.3f} "
          f"lift={labels.q_lift:.3f} bottom={labels.p_bottom:.3f} stuck={labels.p_stuck:.3f}")
    print()

    packager = SamplePackager(cfgs["dataset"]["map"], cfgs["vehicles"])
    from ml25d_dataset_generation.common_types import SampleMetadata

    metadata = SampleMetadata(
        sample_id=0, seed=seed,
        terrain_class=args.terrain, friction_class=args.friction,
        vehicle_id=args.vehicle, action_id=args.action,
        action_name=action.name, motion_model="skid", heading_rad=heading_rad,
    )
    sample = packager.create_sample(
        heightmap=terrain.heightmap,
        heading_rad=heading_rad,
        vehicle=vehicle,
        action=action,
        friction_mu=friction_mu,
        labels=labels,
        band=band,
        metadata=metadata,
    )

    # Save
    output_dir = Path.cwd() / args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    npz_path = output_dir / f"demo_{args.terrain}_{args.vehicle}_{args.action}_{args.friction}_seed{seed}.npz"
    np.savez_compressed(npz_path, X_map=sample["X_map"], theta_v=sample["theta_v"],
                        a=sample["a"], mu=sample["mu"], y=sample["y"])
    print(f"  数据已保存: {npz_path}")

    with (output_dir / f"demo_sample_info_{seed}.json").open("w") as f:
        json.dump({
            "seed": seed, "terrain": args.terrain, "vehicle": args.vehicle,
            "action": args.action, "friction": args.friction, "band": band,
            "labels": {"q_roll": float(labels.q_roll), "q_pitch": float(labels.q_pitch),
                       "q_slip": float(labels.q_slip), "q_lift": float(labels.q_lift),
                       "p_bottom": float(labels.p_bottom), "p_stuck": float(labels.p_stuck)},
        }, f, indent=2, ensure_ascii=False)
    print(f"  信息已保存: {output_dir}/demo_sample_info_{seed}.json")
    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
