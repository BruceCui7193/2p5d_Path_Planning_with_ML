#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import multiprocessing as mp
import os
import queue
import re
import signal
import subprocess
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, VehicleParams
from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import SimulationContext, make_runner


@dataclass(frozen=True)
class SampleTask:
    sample_id: int
    seed: int
    action_id: str


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except OSError:
        return False


def _cleanup_stale_runtime_processes(stage_name: str, worker_id: int) -> None:
    # If a prior run was interrupted, stale bridge/gz processes on the same
    # world/model namespace can leak messages into this run.
    world_tag = f"ml25d_repro_{stage_name}_w{worker_id}"
    model_tag = f"ml25d_repro_vehicle_{stage_name}_w{worker_id}"
    tokens = [world_tag, model_tag]
    try:
        proc_list = subprocess.check_output(["ps", "-eo", "pid=,cmd="], text=True)
    except Exception:
        return

    current_pid = os.getpid()
    target_pids: list[int] = []
    for raw_line in proc_list.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        pid_text, _, cmd = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if not any(tok in cmd for tok in tokens):
            continue
        if (
            "ros_gz_bridge/parameter_bridge" not in cmd
            and "ros2 run ros_gz_bridge parameter_bridge" not in cmd
            and "gz sim -s -r" not in cmd
        ):
            continue
        target_pids.append(pid)

    if not target_pids:
        return

    for pid in target_pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [pid for pid in target_pids if _pid_alive(pid)]
        if not alive:
            return
        time.sleep(0.1)

    for pid in target_pids:
        if _pid_alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="mini batch reproducibility check for urban_small flat_absolute")
    p.add_argument("--output-dir", type=Path, default=Path("data/diagnostics/mini_batch_repro_check"))
    p.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    p.add_argument("--num-samples", type=int, default=60)
    p.add_argument("--mu", type=float, default=0.8)
    p.add_argument("--seed", type=int, default=20260429)
    p.add_argument("--num-workers-stage1", type=int, default=1)
    p.add_argument("--num-workers-stage2", type=int, default=4)
    p.add_argument("--base-domain", type=int, default=180)
    p.add_argument("--run-stage2-if-stage1-pass", action="store_true", default=True)
    p.add_argument("--stage2-policy", type=str, choices=["auto", "always", "never"], default="auto")
    p.add_argument("--mismatch-tol", type=float, default=1e-6)
    return p.parse_args()


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


def _vehicle_hash(vehicle: VehicleParams) -> str:
    payload = {
        "vehicle_id": vehicle.vehicle_id,
        "L": vehicle.L,
        "W": vehicle.W,
        "l": vehicle.l,
        "b": vehicle.b,
        "r_w": vehicle.r_w,
        "c_g": vehicle.c_g,
        "m": vehicle.m,
        "z_c": vehicle.z_c,
        "phi_max_deg": vehicle.phi_max_deg,
        "theta_max_deg": vehicle.theta_max_deg,
        "alpha_max_deg": vehicle.alpha_max_deg,
        "F_max": vehicle.F_max,
    }
    s = json.dumps(payload, sort_keys=True, ensure_ascii=True)
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _terrain_hash(heightmap: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(heightmap, dtype=np.float32).tobytes()).hexdigest()


def _labels_to_dict(labels) -> dict[str, float]:
    return {
        "y_fail": float(labels.y_fail),
        "q_roll": float(labels.q_roll),
        "q_pitch": float(labels.q_pitch),
        "q_slip": float(labels.q_slip),
        "q_lift": float(labels.q_lift),
        "p_bottom": float(labels.p_bottom),
        "p_stuck": float(labels.p_stuck),
    }


def _diag_subset(diag: dict[str, float]) -> dict[str, float]:
    keys = [
        "buffer_reset_before",
        "buffer_reset_after",
        "buffer_reset_delta",
        "sample_start_msg_time",
        "sample_start_recv_time",
        "odom_msg_time_min",
        "odom_msg_time_max",
        "contact_msg_time_min",
        "contact_msg_time_max",
    ]
    out: dict[str, float] = {}
    for k in keys:
        if k in diag:
            out[k] = float(diag[k])
    return out


def _run_pair_once(
    *,
    runner,
    manager: DatasetManager,
    task: SampleTask,
    vehicle: VehicleParams,
    action: ActionPrimitive,
    heightmap: np.ndarray,
    terrain_hash: str,
    vehicle_hash: str,
    mu: float,
    stage_name: str,
    worker_id: int,
    mismatch_tol: float,
) -> dict[str, Any]:
    context = SimulationContext(
        heightmap=heightmap,
        heading_rad=0.0,
        vehicle=vehicle,
        action=action,
        friction_mu=float(mu),
        motion_model="skid",
        sample_rate_hz=int(manager.sim_cfg["sample_rate_hz"]),
        duration_sec=float(manager.sim_cfg["action_duration_sec"]),
        settle_time_sec=float(manager.sim_cfg["settle_time_sec"]),
        cmd_ramp_sec=float(manager.sim_cfg.get("cmd_ramp_sec", 0.3)),
        scene_id=f"{stage_name}_w{worker_id}_s{task.sample_id}",
    )

    row: dict[str, Any] = {
        "stage": stage_name,
        "worker_id": int(worker_id),
        "sample_id": int(task.sample_id),
        "seed": int(task.seed),
        "terrain_hash": terrain_hash,
        "vehicle_hash": vehicle_hash,
        "action_id": str(task.action_id),
        "mu": float(mu),
    }

    first_traj = None
    first_err = None
    try:
        first_traj = runner.run(context, np.random.default_rng(int(task.seed)))
    except Exception as exc:  # pragma: no cover - runtime environment dependent
        first_err = str(exc)
    first_diag = _diag_subset(getattr(runner, "get_last_run_debug", lambda: {})())

    replay_traj = None
    replay_err = None
    try:
        replay_traj = runner.run(context, np.random.default_rng(int(task.seed)))
    except Exception as exc:  # pragma: no cover - runtime environment dependent
        replay_err = str(exc)
    replay_diag = _diag_subset(getattr(runner, "get_last_run_debug", lambda: {})())

    if first_traj is None or replay_traj is None:
        row.update(
            {
                "status": "runtime_failure",
                "first_error": first_err,
                "replay_error": replay_err,
                "first_diag": first_diag,
                "replay_diag": replay_diag,
                "mismatch": True,
            }
        )
        return row

    labels_first, _ = manager.label_extractor.compute_labels(first_traj, vehicle, action)
    labels_replay, _ = manager.label_extractor.compute_labels(replay_traj, vehicle, action)
    first = _labels_to_dict(labels_first)
    replay = _labels_to_dict(labels_replay)

    metric_keys = ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]
    metric_diff = {k: float(abs(first[k] - replay[k])) for k in metric_keys}
    y_diff = abs(first["y_fail"] - replay["y_fail"])
    class_mismatch = bool(y_diff > 0.5)
    metric_mismatch = bool(any(v > mismatch_tol for v in metric_diff.values()))

    row.update(
        {
            "status": "ok",
            "first": first,
            "replay": replay,
            "metric_diff": metric_diff,
            "y_diff": float(y_diff),
            "class_mismatch": bool(class_mismatch),
            "metric_mismatch": bool(metric_mismatch),
            # Backward-compatible alias used by older reports: now tracks class mismatch only.
            "mismatch": bool(class_mismatch),
            "first_diag": first_diag,
            "replay_diag": replay_diag,
        }
    )
    return row


def _worker_main(
    worker_id: int,
    args_dict: dict[str, Any],
    tasks_q: mp.Queue,
    results_q: mp.Queue,
) -> None:
    output_dir = Path(args_dict["output_dir"])
    stage_name = str(args_dict["stage_name"])
    backend = str(args_dict["backend"])
    package_root = Path(args_dict["package_root"])
    mu = float(args_dict["mu"])
    mismatch_tol = float(args_dict["mismatch_tol"])

    worker_dir = output_dir / stage_name / f"worker_{worker_id}"
    worker_dir.mkdir(parents=True, exist_ok=True)

    _cleanup_stale_runtime_processes(stage_name=stage_name, worker_id=worker_id)

    os.environ["ROS_DOMAIN_ID"] = str(int(args_dict["base_domain"]) + worker_id)
    os.environ["GZ_PARTITION"] = f"ml25d_repro_{stage_name}_w{worker_id}"
    os.environ["ROS_LOG_DIR"] = str(worker_dir / "ros_logs")
    os.environ["GZ_LOG_PATH"] = str(worker_dir / "gz_logs")

    manager = DatasetManager(package_root=package_root, config_dir=None)
    vehicle_map = {v.vehicle_id: v for v in manager.vehicle_library}
    action_map = {a.action_id: a for a in manager.action_library}
    vehicle = vehicle_map["urban_small"]

    patch = int(manager.map_cfg["patch_size"])
    heightmap = np.zeros((patch, patch), dtype=np.float32)
    terr_hash = _terrain_hash(heightmap)
    veh_hash = _vehicle_hash(vehicle)

    sim_cfg = copy.deepcopy(manager.sim_cfg)
    if backend == "ros_gz":
        ros = sim_cfg.setdefault("ros_gz", {})
        world_name = f"ml25d_repro_{stage_name}_w{worker_id}"
        model_name = f"ml25d_repro_vehicle_{stage_name}_w{worker_id}"
        world_template = _resolve_world_sdf_file(
            package_root,
            str(ros.get("world_sdf_file", "worlds/ml25d_empty.sdf")),
        )
        worker_world = _create_worker_world_file(world_template, worker_dir, world_name)
        ros["world_name"] = world_name
        ros["model_name"] = model_name
        ros["world_sdf_file"] = str(worker_world)
        ros["log_dir"] = str(worker_dir / "runner_logs")

    runner = make_runner(backend, sim_cfg)
    try:
        while True:
            try:
                task_obj = tasks_q.get(timeout=1.0)
            except queue.Empty:
                continue
            if task_obj is None:
                break
            task: SampleTask = task_obj
            action = action_map[task.action_id]
            row = _run_pair_once(
                runner=runner,
                manager=manager,
                task=task,
                vehicle=vehicle,
                action=action,
                heightmap=heightmap,
                terrain_hash=terr_hash,
                vehicle_hash=veh_hash,
                mu=mu,
                stage_name=stage_name,
                worker_id=worker_id,
                mismatch_tol=mismatch_tol,
            )
            results_q.put(row)
    finally:
        try:
            runner.shutdown()
        except Exception:
            pass


def _make_tasks(num_samples: int, seed: int) -> list[SampleTask]:
    rng = np.random.default_rng(int(seed))
    action_cycle = ["a0", "a1", "a2"]
    tasks: list[SampleTask] = []
    for i in range(int(num_samples)):
        s = int(rng.integers(0, 2**31 - 1))
        action_id = action_cycle[i % len(action_cycle)]
        tasks.append(SampleTask(sample_id=i, seed=s, action_id=action_id))
    return tasks


def _aggregate_stage(rows: list[dict[str, Any]], *, stage_name: str, terrain_h_std: float, terrain_h_range: float) -> dict[str, Any]:
    ok_rows = [r for r in rows if r.get("status") == "ok"]
    class_mismatch_rows = [r for r in rows if bool(r.get("class_mismatch", r.get("mismatch", False)))]
    metric_mismatch_rows = [r for r in rows if bool(r.get("metric_mismatch", False))]

    first_fail = []
    replay_fail = []
    for r in ok_rows:
        first_fail.append(float(r["first"]["y_fail"]) >= 0.5)
        replay_fail.append(float(r["replay"]["y_fail"]) >= 0.5)

    first_run_fail_rate = float(np.mean(first_fail)) if first_fail else float("nan")
    replay_fail_rate = float(np.mean(replay_fail)) if replay_fail else float("nan")
    class_mismatch_rate = float(len(class_mismatch_rows) / max(len(rows), 1))
    metric_mismatch_rate = float(len(metric_mismatch_rows) / max(len(rows), 1))

    mismatch_table = []
    for r in class_mismatch_rows:
        row = {
            "sample_id": int(r["sample_id"]),
            "seed": int(r["seed"]),
            "terrain_hash": str(r["terrain_hash"]),
            "vehicle_hash": str(r["vehicle_hash"]),
            "action_id": str(r["action_id"]),
            "mu": float(r["mu"]),
            "status": str(r["status"]),
            "first_q_roll": float(r.get("first", {}).get("q_roll", float("nan"))),
            "first_q_pitch": float(r.get("first", {}).get("q_pitch", float("nan"))),
            "first_q_lift": float(r.get("first", {}).get("q_lift", float("nan"))),
            "first_p_bottom": float(r.get("first", {}).get("p_bottom", float("nan"))),
            "first_p_stuck": float(r.get("first", {}).get("p_stuck", float("nan"))),
            "replay_q_roll": float(r.get("replay", {}).get("q_roll", float("nan"))),
            "replay_q_pitch": float(r.get("replay", {}).get("q_pitch", float("nan"))),
            "replay_q_lift": float(r.get("replay", {}).get("q_lift", float("nan"))),
            "replay_p_bottom": float(r.get("replay", {}).get("p_bottom", float("nan"))),
            "replay_p_stuck": float(r.get("replay", {}).get("p_stuck", float("nan"))),
            "first_diag": r.get("first_diag", {}),
            "replay_diag": r.get("replay_diag", {}),
            "first_error": r.get("first_error"),
            "replay_error": r.get("replay_error"),
        }
        mismatch_table.append(row)

    metric_keys = ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]
    metric_diff_summary: dict[str, dict[str, float]] = {}
    for key in metric_keys:
        diffs = np.asarray([abs(float(r["first"][key]) - float(r["replay"][key])) for r in ok_rows], dtype=np.float64)
        if diffs.size == 0:
            metric_diff_summary[key] = {"mean": float("nan"), "p50": float("nan"), "p95": float("nan"), "max": float("nan")}
            continue
        metric_diff_summary[key] = {
            "mean": float(np.mean(diffs)),
            "p50": float(np.quantile(diffs, 0.5, method="nearest")),
            "p95": float(np.quantile(diffs, 0.95, method="nearest")),
            "max": float(np.max(diffs)),
        }

    reset_deltas = []
    sample_start_times = []
    msg_mins = []
    msg_maxs = []
    for r in ok_rows:
        for which in ["first_diag", "replay_diag"]:
            d = r.get(which, {})
            if "buffer_reset_delta" in d:
                reset_deltas.append(float(d["buffer_reset_delta"]))
            if "sample_start_msg_time" in d and np.isfinite(float(d["sample_start_msg_time"])):
                sample_start_times.append(float(d["sample_start_msg_time"]))
            for kmin, kmax in [("odom_msg_time_min", "odom_msg_time_max"), ("contact_msg_time_min", "contact_msg_time_max")]:
                if kmin in d and np.isfinite(float(d[kmin])):
                    msg_mins.append(float(d[kmin]))
                if kmax in d and np.isfinite(float(d[kmax])):
                    msg_maxs.append(float(d[kmax]))

    debug_summary = {
        "label_buffer_reset_delta_mean": float(np.mean(reset_deltas)) if reset_deltas else float("nan"),
        "label_buffer_reset_delta_unique": sorted({float(x) for x in reset_deltas}),
        "sample_start_time_min": float(np.min(sample_start_times)) if sample_start_times else float("nan"),
        "sample_start_time_max": float(np.max(sample_start_times)) if sample_start_times else float("nan"),
        "message_time_min": float(np.min(msg_mins)) if msg_mins else float("nan"),
        "message_time_max": float(np.max(msg_maxs)) if msg_maxs else float("nan"),
    }

    return {
        "stage": stage_name,
        "num_rows": len(rows),
        "ok_rows": len(ok_rows),
        "runtime_failure_rows": len(rows) - len(ok_rows),
        "first_run_fail_rate": first_run_fail_rate,
        "replay_fail_rate": replay_fail_rate,
        "class_mismatch_rate": class_mismatch_rate,
        # Backward-compatible alias expected by previous reports / scripts.
        "label_mismatch_rate": class_mismatch_rate,
        "metric_mismatch_rate": metric_mismatch_rate,
        "terrain_H_std": float(terrain_h_std),
        "terrain_H_range": float(terrain_h_range),
        "mismatch_rows": mismatch_table,
        "metric_diff_summary": metric_diff_summary,
        "debug_summary": debug_summary,
    }


def _run_stage(
    *,
    stage_name: str,
    num_workers: int,
    tasks: list[SampleTask],
    args: argparse.Namespace,
    output_dir: Path,
) -> dict[str, Any]:
    if int(num_workers) <= 0:
        raise ValueError("num_workers must be positive")

    # flat_absolute definition
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root, config_dir=None)
    patch = int(manager.map_cfg["patch_size"])
    flat = np.zeros((patch, patch), dtype=np.float32)
    terrain_h_std = float(np.std(flat))
    terrain_h_range = float(np.max(flat) - np.min(flat))

    stage_dir = output_dir / stage_name
    stage_dir.mkdir(parents=True, exist_ok=True)

    if int(num_workers) == 1:
        # Reuse worker entry for single process parity.
        ctx = mp.get_context("spawn")
        tasks_q: mp.Queue = ctx.Queue()
        results_q: mp.Queue = ctx.Queue()
        for t in tasks:
            tasks_q.put(t)
        tasks_q.put(None)
        args_dict = {
            "output_dir": str(output_dir),
            "stage_name": stage_name,
            "backend": args.backend,
            "package_root": str(package_root),
            "mu": float(args.mu),
            "mismatch_tol": float(args.mismatch_tol),
            "base_domain": int(args.base_domain),
        }
        _worker_main(0, args_dict, tasks_q, results_q)
        rows = []
        while not results_q.empty():
            rows.append(results_q.get())
    else:
        ctx = mp.get_context("spawn")
        tasks_q: mp.Queue = ctx.Queue()
        results_q: mp.Queue = ctx.Queue()
        for t in tasks:
            tasks_q.put(t)
        for _ in range(int(num_workers)):
            tasks_q.put(None)

        args_dict = {
            "output_dir": str(output_dir),
            "stage_name": stage_name,
            "backend": args.backend,
            "package_root": str(package_root),
            "mu": float(args.mu),
            "mismatch_tol": float(args.mismatch_tol),
            "base_domain": int(args.base_domain),
        }
        workers = []
        for wid in range(int(num_workers)):
            p = ctx.Process(target=_worker_main, args=(wid, args_dict, tasks_q, results_q))
            p.start()
            workers.append(p)

        rows = []
        while len(rows) < len(tasks):
            try:
                rows.append(results_q.get(timeout=2.0))
            except queue.Empty:
                if not any(p.is_alive() for p in workers):
                    break
        for p in workers:
            p.join(timeout=15.0)
        for p in workers:
            if p.is_alive():
                p.terminate()
                p.join(timeout=5.0)

    rows = sorted(rows, key=lambda x: int(x["sample_id"]))
    (stage_dir / "rows.jsonl").write_text(
        "\n".join(json.dumps(r, ensure_ascii=False) for r in rows) + ("\n" if rows else ""),
        encoding="utf-8",
    )

    summary = _aggregate_stage(rows, stage_name=stage_name, terrain_h_std=terrain_h_std, terrain_h_range=terrain_h_range)
    (stage_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _stage_pass(summary: dict[str, Any]) -> bool:
    # Acceptance from user: close to zero + absolute flat.
    return (
        float(summary["first_run_fail_rate"]) <= 0.05
        and float(summary["replay_fail_rate"]) <= 0.05
        and float(summary.get("class_mismatch_rate", summary.get("label_mismatch_rate", float("inf")))) <= 0.01
        and float(summary["terrain_H_std"]) <= 1e-9
        and float(summary["terrain_H_range"]) <= 1e-9
    )


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    tasks = _make_tasks(num_samples=int(args.num_samples), seed=int(args.seed))
    stage1 = _run_stage(
        stage_name="stage1_n1",
        num_workers=int(args.num_workers_stage1),
        tasks=tasks,
        args=args,
        output_dir=output_dir,
    )

    stage2 = None
    if args.stage2_policy == "always":
        run_stage2 = True
    elif args.stage2_policy == "never":
        run_stage2 = False
    else:
        run_stage2 = bool(args.run_stage2_if_stage1_pass) and _stage_pass(stage1)
    if run_stage2:
        stage2 = _run_stage(
            stage_name="stage2_n4",
            num_workers=int(args.num_workers_stage2),
            tasks=tasks,
            args=args,
            output_dir=output_dir,
        )

    payload = {
        "config": {
            "num_samples": int(args.num_samples),
            "mu": float(args.mu),
            "seed": int(args.seed),
            "backend": args.backend,
            "num_workers_stage1": int(args.num_workers_stage1),
            "num_workers_stage2": int(args.num_workers_stage2),
            "mismatch_tol": float(args.mismatch_tol),
        },
        "stage1": stage1,
        "stage2": stage2,
        "stage1_pass": _stage_pass(stage1),
        "stage2_ran": bool(stage2 is not None),
    }
    report_path = output_dir / "mini_batch_repro_check_report.json"
    report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    # compact markdown
    md_lines = []
    md_lines.append("## Mini Batch Repro Check")
    for name, s in [("stage1_n1", stage1), ("stage2_n4", stage2)]:
        if s is None:
            continue
        md_lines.append("")
        md_lines.append(f"### {name}")
        md_lines.append("| first_run_fail_rate | replay_fail_rate | class_mismatch_rate | metric_mismatch_rate | terrain_H_std | terrain_H_range |")
        md_lines.append("| --- | --- | --- | --- | --- | --- |")
        md_lines.append(
            f"| {float(s['first_run_fail_rate']):.6f} | {float(s['replay_fail_rate']):.6f} | "
            f"{float(s.get('class_mismatch_rate', s.get('label_mismatch_rate', float('nan')))):.6f} | "
            f"{float(s.get('metric_mismatch_rate', float('nan'))):.6f} | "
            f"{float(s['terrain_H_std']):.6e} | {float(s['terrain_H_range']):.6e} |"
        )
        md_lines.append("")
        md_lines.append("| ok_rows | runtime_failure_rows | sample_start_time_min | sample_start_time_max | message_time_min | message_time_max |")
        md_lines.append("| --- | --- | --- | --- | --- | --- |")
        d = s["debug_summary"]
        md_lines.append(
            f"| {int(s['ok_rows'])} | {int(s['runtime_failure_rows'])} | {float(d['sample_start_time_min']):.6f} | {float(d['sample_start_time_max']):.6f} | {float(d['message_time_min']):.6f} | {float(d['message_time_max']):.6f} |"
        )
        md_lines.append("")
        msum = s.get("metric_diff_summary", {})
        if msum:
            md_lines.append("| metric | diff_mean | diff_p50 | diff_p95 | diff_max |")
            md_lines.append("| --- | --- | --- | --- | --- |")
            for mk in ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]:
                cur = msum.get(mk, {})
                md_lines.append(
                    f"| {mk} | {float(cur.get('mean', float('nan'))):.6f} | {float(cur.get('p50', float('nan'))):.6f} | "
                    f"{float(cur.get('p95', float('nan'))):.6f} | {float(cur.get('max', float('nan'))):.6f} |"
                )
            md_lines.append("")
        mm = s.get("mismatch_rows", [])
        if mm:
            md_lines.append("Mismatch rows:")
            md_lines.append("| sample_id | seed | action_id | mu | first_q_roll | first_q_pitch | first_q_lift | first_p_bottom | first_p_stuck | replay_q_roll | replay_q_pitch | replay_q_lift | replay_p_bottom | replay_p_stuck |")
            md_lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
            for r in mm:
                md_lines.append(
                    f"| {int(r['sample_id'])} | {int(r['seed'])} | {r['action_id']} | {float(r['mu']):.3f} | "
                    f"{float(r['first_q_roll']):.6f} | {float(r['first_q_pitch']):.6f} | {float(r['first_q_lift']):.6f} | "
                    f"{float(r['first_p_bottom']):.6f} | {float(r['first_p_stuck']):.6f} | "
                    f"{float(r['replay_q_roll']):.6f} | {float(r['replay_q_pitch']):.6f} | {float(r['replay_q_lift']):.6f} | "
                    f"{float(r['replay_p_bottom']):.6f} | {float(r['replay_p_stuck']):.6f} |"
                )
        else:
            md_lines.append("No mismatches.")

    md_path = output_dir / "mini_batch_repro_check_report.md"
    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "report_json": str(report_path),
                "report_md": str(md_path),
                "stage1_rows": str(output_dir / "stage1_n1" / "rows.jsonl"),
                "stage1_summary": str(output_dir / "stage1_n1" / "summary.json"),
                "stage2_rows": str(output_dir / "stage2_n4" / "rows.jsonl"),
                "stage2_summary": str(output_dir / "stage2_n4" / "summary.json"),
                "stage1_pass": _stage_pass(stage1),
                "stage2_ran": bool(stage2 is not None),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
