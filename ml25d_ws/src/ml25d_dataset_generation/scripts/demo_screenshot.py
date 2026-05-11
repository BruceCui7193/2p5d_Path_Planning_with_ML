#!/usr/bin/env python3
"""Screenshot-ready demo with fixed cameras and generous pauses.

Two fixed cameras are embedded in the world SDF:
  - overview_camera: high-angle overview (2.0, -3.0, 2.5)
  - closeup_camera:  low-angle close-up (1.2, -2.0, 0.8)

In the Gazebo GUI, switch cameras via right-panel Camera dropdown,
or manually orbit (middle-click drag = rotate, scroll = zoom).

Each sample pauses THREE times for screenshots:
  1. BEFORE action: vehicle spawned and settled on terrain
  2. AFTER action:  vehicle completed the 2s maneuver
  3. Between samples: press Enter to advance

The settle time is lengthened to give you time to adjust camera.

Usage:
  cd ~/文档/Machine_Learning_25D/ml25d_ws
  source /opt/ros/jazzy/setup.bash
  export ROS_LOG_DIR=/tmp/ml25d_ros_logs
  export GZ_LOG_PATH=/tmp/ml25d_gz_logs
  PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
    src/ml25d_dataset_generation/scripts/demo_screenshot.py
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import StartGateError, make_runner


TERRAIN_CN: dict[str, str] = {
    "flat": "平坦", "uniform_slope": "纵坡", "lateral_slope": "横坡",
    "steps": "台阶", "pits": "坑洞", "bumps": "凸起", "waves": "波浪",
    "slope_bumps": "斜坡+凸起", "lateral_pits": "侧坡+坑洞", "mixed_random": "混合",
}
VEHICLE_CN: dict[str, str] = {
    "urban_small": "小型城市车", "standard_offroad": "标准越野车", "mountain_large": "大型山地车",
}
BAND_CN: dict[str, str] = {
    "safe": "安全 ✓", "fail": "失败 ✗", "critical": "临界 ⚠",
}


def _wait(prompt: str) -> None:
    print(f"\n  >>> {prompt}")
    print(f"  >>> 按 Enter 继续 ...", end=" ", flush=True)
    input()


def _explain(exc: StartGateError) -> str:
    d = exc.diag
    r: list[str] = []
    if float(d.get("bottom_flag", 0)) > 0.5:
        r.append("底盘触地")
    if float(d.get("lift_before_action", 0)) > 0.5:
        r.append("车轮悬空")
    if float(d.get("roll_error_gate_deg", 0)) >= 4.0:
        r.append(f"侧倾偏差{d['roll_error_gate_deg']:.0f}°")
    if float(d.get("pitch_error_gate_deg", 0)) >= 4.0:
        r.append(f"俯仰偏差{d['pitch_error_gate_deg']:.0f}°")
    return "；".join(r) if r else "姿态不稳定"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Step-by-step Gazebo screenshot demo")
    p.add_argument("--num-samples", type=int, default=2)
    p.add_argument("--seed", type=int, default=20260509)
    p.add_argument("--output-dir", type=Path, default=Path("data/demo_screenshots"))
    p.add_argument("--config-dir", type=Path, default=None)
    p.add_argument("--settle-sec", type=float, default=3.0,
                   help="Extra settle time for camera adjustment (default 3s)")
    p.add_argument("--max-retries", type=int, default=12)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]

    manager = DatasetManager(package_root=package_root, config_dir=args.config_dir)

    # Demo world (has fixed cameras) + GUI mode
    manager.sim_cfg.setdefault("ros_gz", {})
    manager.sim_cfg["ros_gz"]["headless"] = False
    manager.sim_cfg["ros_gz"]["world_sdf_file"] = "worlds/ml25d_demo.sdf"
    # Lengthen settle time so you have time to adjust camera before action
    manager.sim_cfg["settle_time_sec"] = float(args.settle_sec)

    print("=" * 70)
    print("  2.5D 地形风险感知 — 截图演示")
    print("=" * 70)
    print()
    print("  内置固定相机 (Gazebo 右侧面板 Camera 菜单切换):")
    print("    overview_camera — 俯瞰全局")
    print("    closeup_camera  — 近景低角度")
    print()
    print("  手动操作: 中键拖拽=旋转, 滚轮=缩放, Shift+中键=平移")
    print()
    print(f"  共 {args.num_samples} 个样本，每样本分两个阶段截图:")
    print(f"    阶段1: 车辆静止在初始位置 (settle {args.settle_sec:.0f}s)")
    print(f"    阶段2: 动作执行完毕后的最终姿态")
    print()

    _wait("准备好了，启动 Gazebo ...")

    runner = make_runner("ros_gz", manager.sim_cfg)
    rng = np.random.default_rng(args.seed)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    all_info: list[dict] = []

    for i in range(args.num_samples):
        print()
        print("=" * 70)
        print(f"  样本 {i + 1} / {args.num_samples}")
        print("=" * 70)

        # Retry loop for gate failures
        attempt = 0
        sample = None
        band = ""
        counts_info: dict = {}
        while attempt <= args.max_retries:
            seed = int(rng.integers(0, 2**31 - 1))
            try:
                sample, band, counts_info = manager.generate_one_sample(
                    sample_id=i, seed=seed, runner=runner,
                )
                break
            except StartGateError as exc:
                print(f"  ⚠ 稳定门失败 ({_explain(exc)}) → 重试 {attempt+1}/{args.max_retries}")
                attempt += 1
                time.sleep(1.5)
            except Exception as exc:
                print(f"  ✗ 异常: {exc}")
                attempt += 1
                time.sleep(1.5)

        if sample is None:
            print(f"  ✗ 样本 {i+1} 重试耗尽，跳过")
            continue

        tn = counts_info.get("terrain_class", "?")
        vn = counts_info.get("vehicle_id", "?")
        an = counts_info.get("action_id", "?")
        fn = counts_info.get("friction_class", "?")
        y = sample["y"]

        print(f"  地形: {TERRAIN_CN.get(tn, tn)} ({tn})")
        print(f"  车辆: {VEHICLE_CN.get(vn, vn)} ({vn})")
        print(f"  动作: {an}  摩擦: {fn}  结果: {BAND_CN.get(band, band)}")
        print(f"  roll={y[1]:.3f} pitch={y[2]:.3f} slip={y[3]:.3f} "
              f"lift={y[4]:.3f} bottom={y[5]:.3f} stuck={y[6]:.3f}")
        print()
        print(f"  [截图提示] 车辆已停在最终位置，调整相机后截图。")

        all_info.append({
            "index": i, "seed": seed,
            "terrain": tn, "vehicle": vn, "action": an, "friction": fn, "band": band,
            "labels": {f"y_{k}": float(v) for k, v in enumerate(y)},
        })

        if i < args.num_samples - 1:
            _wait(f"截图完毕，按 Enter 进入下一个样本")
        else:
            _wait(f"截图完毕，按 Enter 结束演示")

    print()
    print("=" * 70)
    print("  演示完成")
    print("=" * 70)

    summary_path = output_dir / "demo_summary.json"
    with summary_path.open("w", encoding="utf-8") as fp:
        json.dump({"seed": args.seed, "samples": all_info}, fp, indent=2, ensure_ascii=False)
    print(f"  摘要: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
