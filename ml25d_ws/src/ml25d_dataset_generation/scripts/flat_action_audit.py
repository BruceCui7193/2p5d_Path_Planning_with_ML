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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, VehicleParams
from ml25d_dataset_generation.config_loader import weighted_table
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


@dataclass(frozen=True)
class ComboTask:
    combo_id: int
    vehicle_alias: str
    vehicle_id: str
    action_id: str
    friction_class: str
    mu_lo: float
    mu_hi: float
    repeats: int
    max_retry: int
    seed: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flat action audit for action semantics validation")
    parser.add_argument("--output-dir", type=Path, default=Path("data/diagnostics/flat_action_audit"))
    parser.add_argument("--seed", type=int, default=20260426)
    parser.add_argument("--repeats", type=int, default=20, help="Valid samples per (vehicle, action, friction)")
    parser.add_argument("--max-retry", type=int, default=2, help="Retry count for invalid samples")
    parser.add_argument("--num-workers", type=int, default=4, choices=[1, 2, 4], help="Parallel workers")
    parser.add_argument("--base-domain", type=int, default=140, help="ROS_DOMAIN_ID base")
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    parser.add_argument(
        "--vehicles",
        type=str,
        default="city_small,offroad_medium,mountain_large",
        help="Vehicle aliases to include",
    )
    parser.add_argument(
        "--actions",
        type=str,
        default="a0,a1,a2,a3,a4",
        help="Action IDs to include",
    )
    parser.add_argument(
        "--frictions",
        type=str,
        default="dry_hard,grass_soft,wet_muddy,mixed",
        help="Friction class names to include",
    )
    parser.add_argument(
        "--turn-remove-fail-threshold",
        type=float,
        default=0.15,
        help="Recommend removing a3/a4 if non-slip flat fail rate exceeds this threshold",
    )
    parser.add_argument(
        "--fixed-mu",
        type=float,
        default=float("nan"),
        help="If finite, use fixed friction mu for all samples (ignore sampled ranges).",
    )
    return parser.parse_args()


def _parse_csv(text: str) -> list[str]:
    return [x.strip() for x in str(text).split(",") if x.strip()]


def _pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    arr = np.asarray(values, dtype=np.float64)
    return float(np.percentile(arr, p))


def _mean(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def _finite_stats(values: list[float]) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"mean": float("nan"), "p50": float("nan"), "p95": float("nan")}
    return {
        "mean": float(np.mean(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
    }


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        if np.isnan(value):
            return "NaN"
        return f"{value:.4f}"
    return str(value)


def _to_md_table(rows: list[dict[str, Any]], headers: list[str]) -> str:
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
    candidates = [Path.cwd() / p, package_root / p, package_root.parent / p]
    for c in candidates:
        if c.exists():
            return c
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
    if "already exists" in msg:
        return "entity_name_conflict"
    if "segmentation fault" in msg:
        return "segfault"
    if "timed out" in msg:
        return "timeout"
    return "runtime_failure"


def _compute_motion_metrics(trajectory, action: ActionPrimitive) -> tuple[float, float, float]:
    translation_progress = float("nan")
    angular_progress = float("nan")
    translation_drift = float(trajectory.completed_displacement_m)

    if action.delta_s_m > 1e-4:
        translation_progress = float(trajectory.completed_displacement_m / max(action.delta_s_m, 1e-6))
    else:
        target_yaw = abs(np.deg2rad(action.delta_psi_deg))
        if target_yaw > 1e-6:
            raw = float(trajectory.completed_heading_change_rad / target_yaw)
            angular_progress = float(np.clip(raw, 0.0, 1.0))
        else:
            angular_progress = 0.0

    return translation_progress, angular_progress, translation_drift


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


def _worker_main(
    worker_id: int,
    *,
    args_dict: dict[str, Any],
    task_queue: mp.Queue,
    result_queue: mp.Queue,
) -> None:
    output_dir = Path(args_dict["output_dir"])
    backend = str(args_dict["backend"])
    fixed_mu_raw = float(args_dict.get("fixed_mu", float("nan")))
    fixed_mu = fixed_mu_raw if np.isfinite(fixed_mu_raw) else None
    package_root = Path(args_dict["package_root"])
    worker_dir = output_dir / f"worker_{worker_id}"
    worker_dir.mkdir(parents=True, exist_ok=True)

    os.environ["ROS_DOMAIN_ID"] = str(int(args_dict["base_domain"]) + worker_id)
    os.environ["GZ_PARTITION"] = f"ml25d_flat_audit_worker_{worker_id}"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")

    manager = DatasetManager(package_root=package_root, config_dir=None)
    sim_cfg = copy.deepcopy(manager.sim_cfg)
    if backend == "ros_gz":
        ros_cfg = sim_cfg.setdefault("ros_gz", {})
        world_name = f"ml25d_flat_audit_w{worker_id}"
        model_name = f"ml25d_vehicle_w{worker_id}"
        template = _resolve_world_sdf_file(
            package_root,
            str(ros_cfg.get("world_sdf_file", "worlds/ml25d_empty.sdf")),
        )
        worker_world = _create_worker_world_file(template, worker_dir, world_name)
        ros_cfg["world_sdf_file"] = str(worker_world)
        ros_cfg["world_name"] = world_name
        ros_cfg["model_name"] = model_name
        ros_cfg["log_dir"] = str(worker_dir / "runner_logs")

    runner = make_runner(backend, sim_cfg)
    thresholds = manager.label_extractor.thresholds

    vehicle_by_id: dict[str, VehicleParams] = {v.vehicle_id: v for v in manager.vehicle_library}
    action_by_id: dict[str, ActionPrimitive] = {a.action_id: a for a in manager.action_library}
    patch = int(manager.map_cfg["patch_size"])
    flat_map = np.zeros((patch, patch), dtype=np.float32)
    sample_rate_hz = int(manager.sim_cfg["sample_rate_hz"])
    duration_sec = float(manager.sim_cfg["action_duration_sec"])
    settle_time_sec = float(manager.sim_cfg["settle_time_sec"])

    try:
        while True:
            try:
                raw_task = task_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if raw_task is None:
                break

            task = ComboTask(**raw_task)
            rng = np.random.default_rng(int(task.seed))
            vehicle = vehicle_by_id[task.vehicle_id]
            action = action_by_id[task.action_id]

            valid = 0
            logical_attempts = 0
            max_attempt_budget = int(task.repeats * max(task.max_retry + 2, 4))
            retry_count = 0
            invalid_attempts = 0
            invalid_reason_counter: Counter[str] = Counter()
            fail_reason_counter: Counter[str] = Counter()

            q_roll_vals: list[float] = []
            q_pitch_vals: list[float] = []
            q_lift_vals: list[float] = []
            p_bottom_vals: list[float] = []
            stuck_vals: list[float] = []
            y_fail_vals: list[float] = []
            t_progress_vals: list[float] = []
            a_progress_vals: list[float] = []
            drift_vals: list[float] = []
            a34_progress_gt3 = 0
            a34_stuck_with_good_angular = 0
            sample_rows: list[dict[str, Any]] = []

            while valid < task.repeats and logical_attempts < max_attempt_budget:
                logical_attempts += 1
                accepted = False
                for retry_idx in range(task.max_retry + 1):
                    if retry_idx > 0:
                        retry_count += 1
                    seed = int(rng.integers(0, 2**31 - 1))
                    local_rng = np.random.default_rng(seed)
                    if fixed_mu is None:
                        friction_mu = float(local_rng.uniform(task.mu_lo, task.mu_hi))
                    else:
                        friction_mu = float(np.clip(fixed_mu, 0.05, 3.0))
                    scene_id = f"w{worker_id}_c{task.combo_id}_la{logical_attempts}_r{retry_idx}_v{valid}"
                    context = SimulationContext(
                        heightmap=flat_map,
                        heading_rad=0.0,
                        vehicle=vehicle,
                        action=action,
                        friction_mu=friction_mu,
                        motion_model="skid",
                        sample_rate_hz=sample_rate_hz,
                        duration_sec=duration_sec,
                        settle_time_sec=settle_time_sec,
                        scene_id=scene_id,
                    )
                    try:
                        traj = runner.run(context, local_rng)
                    except Exception as exc:
                        reason = _classify_error(str(exc))
                        invalid_attempts += 1
                        invalid_reason_counter[reason] += 1
                        if _is_invalid_contact_error(str(exc)) and retry_idx < task.max_retry:
                            continue
                        break

                    labels, _ = manager.label_extractor.compute_labels(traj, vehicle, action)
                    t_progress, a_progress, drift = _compute_motion_metrics(traj, action)
                    fail_reasons = _sample_fail_reasons(labels, thresholds)

                    valid += 1
                    accepted = True
                    y_fail_vals.append(float(labels.y_fail))
                    q_roll_vals.append(float(labels.q_roll))
                    q_pitch_vals.append(float(labels.q_pitch))
                    q_lift_vals.append(float(labels.q_lift))
                    p_bottom_vals.append(float(labels.p_bottom))
                    stuck_vals.append(float(labels.p_stuck))
                    t_progress_vals.append(float(t_progress))
                    a_progress_vals.append(float(a_progress))
                    drift_vals.append(float(drift))

                    for reason in fail_reasons:
                        fail_reason_counter[reason] += 1

                    if action.action_id in {"a3", "a4"}:
                        if np.isfinite(a_progress) and a_progress > 3.0:
                            a34_progress_gt3 += 1
                        if float(labels.p_stuck) >= 1.0 and np.isfinite(a_progress) and a_progress >= 0.5:
                            a34_stuck_with_good_angular += 1

                    sample_rows.append(
                        {
                            "combo_id": task.combo_id,
                            "worker_id": worker_id,
                            "vehicle_type": task.vehicle_alias,
                            "vehicle_id": task.vehicle_id,
                            "action_id": action.action_id,
                            "friction_class": task.friction_class,
                            "friction_mu": friction_mu,
                            "seed": seed,
                            "y_fail": float(labels.y_fail),
                            "fail_reasons": fail_reasons,
                            "q_roll": float(labels.q_roll),
                            "q_pitch": float(labels.q_pitch),
                            "q_lift": float(labels.q_lift),
                            "p_bottom": float(labels.p_bottom),
                            "p_stuck": float(labels.p_stuck),
                            "translation_progress": float(t_progress),
                            "angular_progress": float(a_progress),
                            "translation_drift": float(drift),
                        }
                    )
                    break

                if not accepted:
                    continue

            summary = {
                "combo_id": task.combo_id,
                "worker_id": worker_id,
                "vehicle_type": task.vehicle_alias,
                "vehicle_id": task.vehicle_id,
                "action_id": task.action_id,
                "friction_class": task.friction_class,
                "mu_range": [task.mu_lo, task.mu_hi],
                "target_count": task.repeats,
                "valid_count": valid,
                "logical_attempts": logical_attempts,
                "invalid_attempts": invalid_attempts,
                "retry_count": retry_count,
                "fail_rate": float(np.mean(np.asarray(y_fail_vals) >= 0.5)) if y_fail_vals else float("nan"),
                "fail_reason_distribution": dict(sorted(fail_reason_counter.items())),
                "q_roll_mean": _mean(q_roll_vals),
                "q_roll_p95": _pct(q_roll_vals, 95),
                "q_pitch_mean": _mean(q_pitch_vals),
                "q_pitch_p95": _pct(q_pitch_vals, 95),
                "q_lift_mean": _mean(q_lift_vals),
                "q_lift_p95": _pct(q_lift_vals, 95),
                "p_bottom_mean": _mean(p_bottom_vals),
                "p_bottom_p95": _pct(p_bottom_vals, 95),
                "stuck_fail_rate": float(np.mean(np.asarray(stuck_vals) >= 1.0)) if stuck_vals else float("nan"),
                "translation_progress": _finite_stats(t_progress_vals),
                "angular_progress": _finite_stats(a_progress_vals),
                "translation_drift": _finite_stats(drift_vals),
                "a3a4_progress_gt3_count": int(a34_progress_gt3),
                "a3a4_stuck_with_good_angular_count": int(a34_stuck_with_good_angular),
                "invalid_reason_distribution": dict(sorted(invalid_reason_counter.items())),
            }
            result_queue.put({"kind": "combo_result", "summary": summary, "samples": sample_rows})
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass


def _build_tasks(
    *,
    manager: DatasetManager,
    args: argparse.Namespace,
) -> list[ComboTask]:
    alias_to_vehicle = {
        "city_small": "urban_small",
        "offroad_medium": "standard_offroad",
        "mountain_large": "mountain_large",
    }
    valid_vehicle_ids = {v.vehicle_id for v in manager.vehicle_library}
    vehicles = _parse_csv(args.vehicles)
    actions = _parse_csv(args.actions)
    frictions = _parse_csv(args.frictions)

    for alias in vehicles:
        if alias not in alias_to_vehicle:
            raise ValueError(f"unknown vehicle alias: {alias}")
        if alias_to_vehicle[alias] not in valid_vehicle_ids:
            raise ValueError(f"vehicle missing in config: {alias_to_vehicle[alias]}")

    action_ids = {a.action_id for a in manager.action_library}
    for aid in actions:
        if aid not in action_ids:
            raise ValueError(f"unknown action id: {aid}")

    friction_rows, _ = weighted_table(manager.friction_cfg["friction"]["classes"])
    friction_map = {str(r["name"]): r for r in friction_rows}
    for name in frictions:
        if name not in friction_map:
            raise ValueError(f"unknown friction class: {name}")

    rng = np.random.default_rng(int(args.seed))
    tasks: list[ComboTask] = []
    combo_id = 0
    for vehicle_alias in vehicles:
        vehicle_id = alias_to_vehicle[vehicle_alias]
        for action_id in actions:
            for friction_class in frictions:
                row = friction_map[friction_class]
                mu_lo, mu_hi = row["mu_range"]
                combo_seed = int(rng.integers(0, 2**31 - 1))
                tasks.append(
                    ComboTask(
                        combo_id=combo_id,
                        vehicle_alias=vehicle_alias,
                        vehicle_id=vehicle_id,
                        action_id=action_id,
                        friction_class=friction_class,
                        mu_lo=float(mu_lo),
                        mu_hi=float(mu_hi),
                        repeats=int(args.repeats),
                        max_retry=int(args.max_retry),
                        seed=combo_seed,
                    )
                )
                combo_id += 1
    return tasks


def _main_markdown(
    *,
    path: Path,
    summary: dict[str, Any],
    combo_rows: list[dict[str, Any]],
) -> None:
    lines: list[str] = []
    lines.append("## Flat Action Audit Summary")
    lines.append(
        _to_md_table(
            [
                {
                    "groups_total": summary["groups_total"],
                    "groups_completed": summary["groups_completed"],
                    "valid_samples": summary["valid_samples"],
                    "invalid_attempts": summary["invalid_attempts"],
                    "flat_fail_rate": summary["flat_fail_rate"],
                    "accept_flat_fail_lt_5pct": summary["accept_flat_fail_lt_5pct"],
                    "accept_a3a4_no_progress_gt3": summary["accept_a3a4_no_progress_gt3"],
                    "accept_a3a4_stuck_semantics": summary["accept_a3a4_stuck_semantics"],
                    "recommend_remove_a3a4": summary["recommend_remove_a3a4"],
                }
            ],
            [
                "groups_total",
                "groups_completed",
                "valid_samples",
                "invalid_attempts",
                "flat_fail_rate",
                "accept_flat_fail_lt_5pct",
                "accept_a3a4_no_progress_gt3",
                "accept_a3a4_stuck_semantics",
                "recommend_remove_a3a4",
            ],
        )
    )
    lines.append("")
    lines.append("### Notes")
    lines.append(f"- {summary['note_flat_fail']}")
    lines.append(f"- {summary['note_a3a4']}")
    lines.append("")
    lines.append("## Per Group Metrics")
    lines.append(
        _to_md_table(
            combo_rows,
            [
                "vehicle_type",
                "action_id",
                "friction_class",
                "target_count",
                "valid_count",
                "fail_rate",
                "stuck_fail_rate",
                "q_roll_mean",
                "q_pitch_mean",
                "q_lift_mean",
                "p_bottom_mean",
                "translation_progress_mean",
                "angular_progress_mean",
                "translation_drift_mean",
                "fail_reason_distribution",
            ],
        )
    )
    lines.append("")
    lines.append(f"json_path: `{path.with_suffix('.json')}`")
    lines.append(f"samples_jsonl: `{path.parent / 'flat_action_samples.jsonl'}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    max_domain = int(args.base_domain) + int(args.num_workers) - 1
    if max_domain > 232:
        raise ValueError(
            f"ROS_DOMAIN_ID out of supported range: base_domain={args.base_domain}, "
            f"num_workers={args.num_workers}, max_domain={max_domain} (>232)"
        )
    if args.repeats < 10:
        raise ValueError("repeats must be >= 10 for this audit")

    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root, config_dir=None)
    tasks = _build_tasks(manager=manager, args=args)

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    sample_jsonl = output_dir / "flat_action_samples.jsonl"
    sample_jsonl.write_text("", encoding="utf-8")

    print(
        "[flat-audit] start "
        f"groups={len(tasks)} repeats={args.repeats} workers={args.num_workers} backend={args.backend}",
        flush=True,
    )

    ctx = mp.get_context("spawn")
    task_queue: mp.Queue = ctx.Queue()
    result_queue: mp.Queue = ctx.Queue(maxsize=max(8, args.num_workers * 4))
    for task in tasks:
        task_queue.put(task.__dict__)
    for _ in range(args.num_workers):
        task_queue.put(None)

    worker_args = {
        "output_dir": str(output_dir),
        "backend": args.backend,
        "base_domain": int(args.base_domain),
        "package_root": str(package_root),
        "fixed_mu": float(args.fixed_mu),
    }

    workers = []
    for wid in range(args.num_workers):
        proc = ctx.Process(
            target=_worker_main,
            kwargs={
                "worker_id": wid,
                "args_dict": worker_args,
                "task_queue": task_queue,
                "result_queue": result_queue,
            },
        )
        proc.start()
        workers.append(proc)

    combo_summaries: list[dict[str, Any]] = []
    sample_rows_all: list[dict[str, Any]] = []
    idle_ticks = 0
    try:
        while len(combo_summaries) < len(tasks):
            try:
                msg = result_queue.get(timeout=15.0)
            except queue.Empty:
                idle_ticks += 1
                if idle_ticks % 4 == 0:
                    alive = sum(int(p.is_alive()) for p in workers)
                    print(
                        "[flat-audit] waiting "
                        f"done={len(combo_summaries)}/{len(tasks)} alive_workers={alive}",
                        flush=True,
                    )
                if not any(p.is_alive() for p in workers):
                    raise RuntimeError(
                        "all workers exited before returning all combo results: "
                        f"done={len(combo_summaries)}/{len(tasks)}"
                    )
                continue

            idle_ticks = 0
            if msg.get("kind") != "combo_result":
                continue
            summary = dict(msg["summary"])
            samples = list(msg["samples"])
            combo_summaries.append(summary)
            sample_rows_all.extend(samples)
            print(
                "[flat-audit] combo "
                f"{len(combo_summaries)}/{len(tasks)} "
                f"vehicle={summary['vehicle_type']} action={summary['action_id']} "
                f"friction={summary['friction_class']} valid={summary['valid_count']}/{summary['target_count']} "
                f"fail_rate={summary['fail_rate']:.3f}",
                flush=True,
            )
    finally:
        for proc in workers:
            proc.join(timeout=15.0)
        for proc in workers:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=5.0)

    with sample_jsonl.open("w", encoding="utf-8") as fp:
        for row in sample_rows_all:
            fp.write(json.dumps(row, ensure_ascii=False) + "\n")

    total_valid = sum(int(r["valid_count"]) for r in combo_summaries)
    total_invalid = sum(int(r["invalid_attempts"]) for r in combo_summaries)
    fail_rates_weighted = []
    for r in combo_summaries:
        if int(r["valid_count"]) > 0 and np.isfinite(float(r["fail_rate"])):
            fail_rates_weighted.extend([float(r["fail_rate"])] * int(r["valid_count"]))
    flat_fail_rate = float(np.mean(fail_rates_weighted)) if fail_rates_weighted else float("nan")

    a34 = [r for r in combo_summaries if str(r["action_id"]) in {"a3", "a4"}]
    a34_progress_gt3 = sum(int(r["a3a4_progress_gt3_count"]) for r in a34)
    a34_stuck_bad = sum(int(r["a3a4_stuck_with_good_angular_count"]) for r in a34)
    a34_non_slip_fail_rates = []
    for r in a34:
        d = r["fail_reason_distribution"]
        nonslip = int(d.get("roll", 0)) + int(d.get("pitch", 0)) + int(d.get("lift", 0))
        valid = int(r["valid_count"])
        if valid > 0:
            a34_non_slip_fail_rates.append(float(nonslip / valid))
    a34_non_slip_fail_rate = float(np.mean(a34_non_slip_fail_rates)) if a34_non_slip_fail_rates else float("nan")

    accept_flat_fail_lt_5 = bool(np.isfinite(flat_fail_rate) and flat_fail_rate < 0.05)
    accept_a3a4_no_progress_gt3 = bool(a34_progress_gt3 == 0)
    accept_a3a4_stuck_semantics = bool(a34_stuck_bad == 0)
    recommend_remove_a3a4 = bool(
        np.isfinite(a34_non_slip_fail_rate) and a34_non_slip_fail_rate > float(args.turn_remove_fail_threshold)
    )

    note_flat = (
        "flat 总 fail_rate 已低于 5%。"
        if accept_flat_fail_lt_5
        else "flat 总 fail_rate 仍高于 5%，不满足验收。"
    )
    if recommend_remove_a3a4:
        note_turn = (
            "a3/a4 在 flat 上仍有较高 roll/pitch/lift 非 slip 失败，建议从正式数据集与主动作空间移除。"
        )
    else:
        note_turn = "a3/a4 未出现显著非 slip 失败放大，可保留。"

    final_summary = {
        "groups_total": len(tasks),
        "groups_completed": int(sum(int(r["valid_count"]) >= int(r["target_count"]) for r in combo_summaries)),
        "valid_samples": int(total_valid),
        "invalid_attempts": int(total_invalid),
        "flat_fail_rate": flat_fail_rate,
        "accept_flat_fail_lt_5pct": accept_flat_fail_lt_5,
        "accept_a3a4_no_progress_gt3": accept_a3a4_no_progress_gt3,
        "accept_a3a4_stuck_semantics": accept_a3a4_stuck_semantics,
        "a3a4_progress_gt3_count": int(a34_progress_gt3),
        "a3a4_stuck_with_good_angular_count": int(a34_stuck_bad),
        "a3a4_non_slip_fail_rate": a34_non_slip_fail_rate,
        "recommend_remove_a3a4": recommend_remove_a3a4,
        "note_flat_fail": note_flat,
        "note_a3a4": note_turn,
        "config": {
            "seed": int(args.seed),
            "repeats": int(args.repeats),
            "max_retry": int(args.max_retry),
            "fixed_mu": (float(args.fixed_mu) if np.isfinite(float(args.fixed_mu)) else None),
            "num_workers": int(args.num_workers),
            "backend": str(args.backend),
            "vehicles": _parse_csv(args.vehicles),
            "actions": _parse_csv(args.actions),
            "frictions": _parse_csv(args.frictions),
        },
    }

    combo_rows = []
    for row in sorted(
        combo_summaries,
        key=lambda r: (str(r["vehicle_type"]), str(r["action_id"]), str(r["friction_class"])),
    ):
        combo_rows.append(
            {
                "vehicle_type": row["vehicle_type"],
                "action_id": row["action_id"],
                "friction_class": row["friction_class"],
                "target_count": row["target_count"],
                "valid_count": row["valid_count"],
                "fail_rate": row["fail_rate"],
                "stuck_fail_rate": row["stuck_fail_rate"],
                "q_roll_mean": row["q_roll_mean"],
                "q_pitch_mean": row["q_pitch_mean"],
                "q_lift_mean": row["q_lift_mean"],
                "p_bottom_mean": row["p_bottom_mean"],
                "translation_progress_mean": row["translation_progress"]["mean"],
                "angular_progress_mean": row["angular_progress"]["mean"],
                "translation_drift_mean": row["translation_drift"]["mean"],
                "fail_reason_distribution": json.dumps(row["fail_reason_distribution"], ensure_ascii=False),
            }
        )

    payload = {
        "summary": final_summary,
        "combo_metrics": combo_summaries,
    }
    json_path = output_dir / "flat_action_audit_report.json"
    md_path = output_dir / "flat_action_audit_report.md"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    _main_markdown(path=md_path, summary=final_summary, combo_rows=combo_rows)
    print(json.dumps({"json": str(json_path), "md": str(md_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
