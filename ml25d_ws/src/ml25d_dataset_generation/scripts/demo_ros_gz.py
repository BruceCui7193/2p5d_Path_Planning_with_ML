#!/usr/bin/env python3
"""Gazebo GUI demo script for defense presentation.

Run this to launch Gazebo with the GUI visible, so you can record
the simulation with OBS or ffmpeg. Samples are generated one-by-one
with pauses in between for narration.

Usage:
  cd ~/文档/Machine_Learning_25D/ml25d_ws
  source /opt/ros/jazzy/setup.bash
  export ROS_LOG_DIR=/tmp/ml25d_ros_logs
  export GZ_LOG_PATH=/tmp/ml25d_gz_logs
  PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
    src/ml25d_dataset_generation/scripts/demo_ros_gz.py \
    --num-samples 6 \
    --pause-sec 4.0
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import StartGateError, make_runner


TERRAIN_DESCRIPTIONS: dict[str, str] = {
    "flat": "平坦地面 — 最简单的场景，车辆应轻松通过",
    "uniform_slope": "纵向斜坡 — 车辆沿行进方向上坡，考验驱动力和俯仰稳定性",
    "lateral_slope": "横向斜坡 — 车辆侧倾，考验侧翻风险",
    "steps": "台阶地形 — 地面有垂直台阶，考验底盘通过性",
    "pits": "坑洞地形 — 地面有凹坑，车轮可能陷入",
    "bumps": "凸起障碍 — 地面有凸起石块，考验悬挂和底盘高度",
    "waves": "波浪地形 — 正弦波起伏，考验连续颠簸稳定性",
    "slope_bumps": "斜坡+凸起 — 上坡同时有障碍物，综合考验",
    "lateral_pits": "侧坡+坑洞 — 横向倾斜且有坑洞，高难度场景",
    "mixed_random": "混合随机 — 随机组合多种地形特征，最接近真实野外环境",
}

VEHICLE_DESCRIPTIONS: dict[str, str] = {
    "urban_small": "小型城市车 (45×30cm, 8kg) — 小巧灵活但通过性有限",
    "standard_offroad": "标准越野车 (65×45cm, 14kg) — 均衡的野外通过能力",
    "mountain_large": "大型山地车 (85×55cm, 20kg) — 最强通过性，但车身大转弯难",
}

ACTION_DESCRIPTIONS: dict[str, str] = {
    "a0": "直行 (a0) — 车辆沿当前方向直线前进",
    "a1": "左小转 (a1) — 车辆向左前方转弯",
    "a2": "右小转 (a2) — 车辆向右前方转弯",
}

FRICTION_DESCRIPTIONS: dict[str, str] = {
    "dry_hard": "干燥硬地面 — 高摩擦力 (μ=0.70-0.90)",
    "grass_soft": "草地软土 — 中等摩擦力 (μ=0.40-0.60)",
    "wet_muddy": "湿滑泥地 — 低摩擦力 (μ=0.20-0.40)",
    "mixed": "混合摩擦 — 变化范围大 (μ=0.20-0.90)",
}

BAND_DESCRIPTIONS: dict[str, str] = {
    "safe": "SAFE (安全) ✓ — 车辆稳定完成动作，风险指标均在安全范围内",
    "fail": "FAIL (失败) ✗ — 车辆发生侧翻、卡住或严重打滑",
    "critical": "CRITICAL (临界) ⚠ — 车辆勉强通过，接近失败边缘",
}


MAX_RETRIES = 8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Gazebo GUI demo for defense presentation")
    parser.add_argument("--num-samples", type=int, default=6, help="Number of samples to generate")
    parser.add_argument("--pause-sec", type=float, default=5.0, help="Pause between samples (seconds)")
    parser.add_argument("--output-dir", type=Path, default=Path("data/demo_gazebo_gui"))
    parser.add_argument("--seed", type=int, default=20260509)
    parser.add_argument("--config-dir", type=Path, default=None)
    return parser.parse_args()


def _print_header(text: str) -> None:
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}\n")


def _explain_gate_failure(exc: StartGateError) -> str:
    """Produce a short Chinese explanation of why the stability gate failed."""
    d = exc.diag
    reasons: list[str] = []

    bottom = float(d.get("bottom_flag", 0.0))
    chassis_clear = float(d.get("chassis_min_clearance", 0.0))
    lift = float(d.get("lift_before_action", 0.0))
    roll_err = float(d.get("roll_error_gate_deg", 0.0))
    pitch_err = float(d.get("pitch_error_gate_deg", 0.0))
    roll_abs = float(d.get("roll_abs_deg", 0.0))
    pitch_abs = float(d.get("pitch_abs_deg", 0.0))
    contact_fresh = float(d.get("contact_fresh", 0.0))
    contact_geom = float(d.get("contact_geom_ready", 0.0))
    odom_planar = float(d.get("odom_planar_fallback", 0.0))
    lin_spd = float(d.get("linear_speed", 0.0))
    ang_spd = float(d.get("angular_speed", 0.0))
    wheel_min = float(d.get("wheel_clearance_min", 0.0))
    wheel_max = float(d.get("wheel_clearance_max", 0.0))

    if bottom > 0.5 and chassis_clear < 0.0:
        reasons.append(f"底盘擦地 (穿透 {abs(chassis_clear)*100:.1f}cm)")
    elif bottom > 0.5:
        reasons.append("底盘触底")
    if lift > 0.5:
        reasons.append("车轮悬空 (>5cm)")
    if roll_err >= 4.0:
        reasons.append(f"车身侧倾角度偏差过大 ({roll_err:.1f}°)")
    if pitch_err >= 4.0:
        reasons.append(f"车身俯仰角度偏差过大 ({pitch_err:.1f}°)")
    if roll_abs >= 35.0:
        reasons.append(f"车身侧倾角过大 ({roll_abs:.1f}°)")
    if pitch_abs >= 35.0:
        reasons.append(f"车身俯仰角过大 ({pitch_abs:.1f}°)")
    if not contact_fresh:
        reasons.append("轮地接触传感器数据过期")
    if odom_planar > 0.5:
        reasons.append("里程计姿态未更新（车身直接搁在地面上）")
    if lin_spd >= 0.02:
        reasons.append("车辆尚未静止")
    if ang_spd >= 0.05:
        reasons.append("车辆仍在旋转")
    if wheel_min <= -0.03:
        reasons.append(f"车轮穿透地面 ({abs(wheel_min)*100:.1f}cm)")
    if wheel_max >= 0.30:
        reasons.append("车轮离地过高")
    if contact_geom < 0.5 and contact_fresh:
        reasons.append("接触几何未收敛（车在颠簸地形上未稳定）")

    if not reasons:
        return "稳定门检查未通过（具体原因未知）"
    return "；".join(reasons)


def _print_sample_info(
    index: int,
    total: int,
    terrain_name: str,
    vehicle_name: str,
    action_name: str,
    friction_name: str,
    band: str,
    labels: dict,
) -> None:
    terrain_desc = TERRAIN_DESCRIPTIONS.get(terrain_name, terrain_name)
    vehicle_desc = VEHICLE_DESCRIPTIONS.get(vehicle_name, vehicle_name)
    action_desc = ACTION_DESCRIPTIONS.get(action_name, action_name)
    friction_desc = FRICTION_DESCRIPTIONS.get(friction_name, friction_name)
    band_desc = BAND_DESCRIPTIONS.get(band, band)

    print(f"  >>> 样本 {index}/{total} <<<")
    print(f"  地形:   {terrain_desc}")
    print(f"  车辆:   {vehicle_desc}")
    print(f"  动作:   {action_desc}")
    print(f"  摩擦:   {friction_desc}")
    print(f"  结果:   {band_desc}")
    print(f"  标签:   roll={labels.get('q_roll',0):.3f} pitch={labels.get('q_pitch',0):.3f} "
          f"slip={labels.get('q_slip',0):.3f} lift={labels.get('q_lift',0):.3f} "
          f"bottom={labels.get('p_bottom',0):.3f} stuck={labels.get('p_stuck',0):.3f}")
    print()


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]

    manager = DatasetManager(package_root=package_root, config_dir=args.config_dir)

    # Force GUI mode
    manager.sim_cfg.setdefault("ros_gz", {})
    manager.sim_cfg["ros_gz"]["headless"] = False

    _print_header("2.5D 野外地形风险感知 — 数据生成演示")

    print("启动 Gazebo 仿真引擎 (GUI 模式)...")
    print("请确保 Gazebo 窗口可见，准备开始录屏。")
    print(f"将生成 {args.num_samples} 个样本，每个样本间隔 {args.pause_sec:.0f} 秒。\n")

    runner = make_runner("ros_gz", manager.sim_cfg)
    rng = np.random.default_rng(args.seed)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    samples_data: list[dict] = []
    total_retries = 0

    for i in range(args.num_samples):
        _print_header(f"样本 {i + 1} / {args.num_samples}")

        attempt = 0
        sample = None
        band = ""
        counts_info: dict = {}

        while attempt <= MAX_RETRIES:
            sample_seed = int(rng.integers(0, 2**31 - 1))

            if attempt == 0:
                print(f"  尝试 {attempt + 1} ...")
            else:
                print(f"  重试 {attempt} (共 {total_retries} 次重试) ...")

            try:
                sample, band, counts_info = manager.generate_one_sample(
                    sample_id=i,
                    seed=sample_seed,
                    runner=runner,
                )
                break
            except StartGateError as exc:
                reason = _explain_gate_failure(exc)
                print(f"  ⚠ 稳定门失败: {reason}")
                print(f"    → 地形太险/车辆无法平稳停放，换一个随机种子重试")
                attempt += 1
                total_retries += 1
                if attempt > MAX_RETRIES:
                    print(f"  ✗ 重试 {MAX_RETRIES} 次后仍失败，跳过此样本")
                else:
                    # Brief pause to let the viewer understand what happened
                    time.sleep(1.5)
            except Exception as exc:
                print(f"  ✗ 样本生成异常: {exc}")
                attempt += 1
                total_retries += 1
                if attempt > MAX_RETRIES:
                    print(f"  ✗ 重试 {MAX_RETRIES} 次后仍失败，跳过此样本")
                else:
                    time.sleep(1.5)

        if sample is None:
            continue

        labels = {
            "q_roll": float(sample["y"][1]),
            "q_pitch": float(sample["y"][2]),
            "q_slip": float(sample["y"][3]),
            "q_lift": float(sample["y"][4]),
            "p_bottom": float(sample["y"][5]),
            "p_stuck": float(sample["y"][6]),
        }

        _print_sample_info(
            index=i + 1,
            total=args.num_samples,
            terrain_name=counts_info.get("terrain_class", "?"),
            vehicle_name=counts_info.get("vehicle_id", "?"),
            action_name=counts_info.get("action_id", "?"),
            friction_name=counts_info.get("friction_class", "?"),
            band=band,
            labels=labels,
        )

        samples_data.append(
            {
                "sample_id": i,
                "seed": sample_seed,
                "band": band,
                "counts_info": counts_info,
                "metadata": sample.get("metadata", {}),
            }
        )

        if i < args.num_samples - 1:
            print(f"  --- {args.pause_sec:.0f} 秒后生成下一个样本 ---")
            time.sleep(args.pause_sec)

    _print_header("演示完成")

    print(f"共生成 {len(samples_data)} 个有效样本，总重试 {total_retries} 次。")
    if total_retries > 0:
        print("重试是正常现象——系统在自动剔除车辆无法稳定停放的极端地形。")
    summary_path = output_dir / "demo_summary.json"
    with summary_path.open("w", encoding="utf-8") as fp:
        json.dump(samples_data, fp, indent=2, ensure_ascii=True)
    print(f"摘要已保存到: {summary_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
