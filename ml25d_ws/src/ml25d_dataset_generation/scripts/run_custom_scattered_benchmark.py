#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.config_loader import build_action_library, build_vehicle_library, load_all_configs
from ml25d_dataset_generation.planning_types import PlannerThresholds, PlanningScene
from ml25d_dataset_generation.risk_astar import (
    METHOD_BASELINE_1,
    METHOD_BASELINE_2,
    METHOD_BASELINE_3,
    METHOD_PROPOSED,
    PlannerConfig,
    plan_path,
)
from ml25d_dataset_generation.risk_model_infer import RiskModelInfer

CTX: dict[str, Any] | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run planning benchmark on pre-generated custom scattered scenes")
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/planning_runs/custom_random_scattered_obstacles_v2"),
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/training_runs/cnn_pso_dataset_v1_20k/pso/best_model_calibrated.pt"),
    )
    parser.add_argument(
        "--config-dir",
        type=Path,
        default=Path("/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/config"),
    )
    parser.add_argument("--scene-glob", type=str, default="custom_scattered_scene_*.npz")
    parser.add_argument("--num-workers", type=int, default=20)
    parser.add_argument("--worker-threads", type=int, default=1)
    parser.add_argument("--goal-radius-cells", type=int, default=2)
    parser.add_argument("--max-expansions", type=int, default=140000)
    parser.add_argument("--max-labels-per-state", type=int, default=4)
    parser.add_argument("--manual-risk-lambda", type=float, default=1.0)
    parser.add_argument("--ml-risk-lambda", type=float, default=1.0)
    parser.add_argument("--proposed-risk-lambda", type=float, default=0.2)
    parser.add_argument("--proposed-manual-guard-weight", type=float, default=0.0)
    parser.add_argument("--edge-safe", type=float, default=0.55)
    parser.add_argument("--path-max-safe", type=float, default=0.82)
    parser.add_argument("--path-avg-safe", type=float, default=0.36)
    return parser.parse_args()


def load_scene_npz(path: Path) -> PlanningScene:
    s = np.load(path)
    return PlanningScene(
        scene_id=str(s["scene_id"].item()),
        terrain_class=str(s["terrain_class"].item()),
        heightmap=np.asarray(s["heightmap"], dtype=np.float32),
        resolution_m=float(s["resolution_m"]),
        friction_mu=float(s["friction_mu"]),
        start_state=tuple(int(v) for v in np.asarray(s["start_state"]).tolist()),
        goal_state=tuple(int(v) for v in np.asarray(s["goal_state"]).tolist()),
        heading_bins=int(s["heading_bins"]),
    )


def init_worker(
    config_dir: str,
    ckpt: str,
    goal_radius_cells: int,
    max_expansions: int,
    max_labels_per_state: int,
    manual_risk_lambda: float,
    ml_risk_lambda: float,
    proposed_risk_lambda: float,
    proposed_manual_guard_weight: float,
    edge_safe: float,
    path_max_safe: float,
    path_avg_safe: float,
    worker_threads: int = 1,
) -> None:
    global CTX
    import os

    threads = max(int(worker_threads), 1)
    os.environ["OMP_NUM_THREADS"] = str(threads)
    os.environ["OPENBLAS_NUM_THREADS"] = str(threads)
    os.environ["MKL_NUM_THREADS"] = str(threads)
    os.environ["NUMEXPR_NUM_THREADS"] = str(threads)

    cfg = load_all_configs(Path(config_dir))
    actions = sorted([a for a in build_action_library(cfg["actions"]) if a.action_id in {"a0", "a1", "a2"}], key=lambda a: a.action_id)
    vehicles = {v.vehicle_id: v for v in build_vehicle_library(cfg["vehicles"])}

    infer = RiskModelInfer(
        checkpoint_path=ckpt,
        config_dir=config_dir,
        device="cpu",
    )

    planner_cfg = PlannerConfig(
        goal_radius_cells=int(goal_radius_cells),
        max_expansions=int(max_expansions),
        max_labels_per_state=int(max_labels_per_state),
        manual_risk_lambda=float(manual_risk_lambda),
        ml_risk_lambda=float(ml_risk_lambda),
        proposed_risk_lambda=float(proposed_risk_lambda),
        proposed_manual_guard_weight=float(proposed_manual_guard_weight),
        default_thresholds=infer.thresholds,
    )
    proposed_thr = PlannerThresholds(edge_safe=float(edge_safe), path_max_safe=float(path_max_safe), path_avg_safe=float(path_avg_safe))

    CTX = {
        "actions": actions,
        "vehicles": vehicles,
        "infer": infer,
        "planner_cfg": planner_cfg,
        "proposed_thr": proposed_thr,
    }


def run_task(task: dict[str, Any]) -> dict[str, Any]:
    global CTX
    assert CTX is not None

    scene = load_scene_npz(Path(task["scene_npz"]))
    vehicle = CTX["vehicles"][task["vehicle_id"]]
    method = str(task["method"])

    if method in {METHOD_BASELINE_1, METHOD_BASELINE_2}:
        model_tag = "none"
        infer_model = None
        thr = None
    elif method == METHOD_BASELINE_3:
        model_tag = "main"
        infer_model = CTX["infer"]
        thr = None
    elif method == METHOD_PROPOSED:
        model_tag = "main"
        infer_model = CTX["infer"]
        thr = CTX["proposed_thr"]
    else:
        raise ValueError(method)

    result = plan_path(
        scene=scene,
        vehicle=vehicle,
        actions=CTX["actions"],
        method=method,
        model_infer=infer_model,
        config=CTX["planner_cfg"],
        thresholds=thr,
    )

    run_id = f"{scene.scene_id}__{vehicle.vehicle_id}__{method}__{model_tag}"
    row = {
        "run_id": run_id,
        "scene_id": scene.scene_id,
        "terrain_class": scene.terrain_class,
        "vehicle_id": vehicle.vehicle_id,
        "method": method,
        "model_tag": model_tag,
        "friction_mu": float(scene.friction_mu),
        "start_i": int(scene.start_state[0]),
        "start_j": int(scene.start_state[1]),
        "start_k": int(scene.start_state[2]),
        "goal_i": int(scene.goal_state[0]),
        "goal_j": int(scene.goal_state[1]),
        "goal_k": int(scene.goal_state[2]),
        "goal_radius_cells": int(CTX["planner_cfg"].goal_radius_cells),
    }
    row.update(result.as_metrics_dict())

    path_row = {
        "run_id": run_id,
        "scene_id": scene.scene_id,
        "terrain_class": scene.terrain_class,
        "vehicle_id": vehicle.vehicle_id,
        "method": method,
        "model_tag": model_tag,
        "found": bool(result.found),
        "states": [[int(a), int(b), int(c)] for (a, b, c) in result.states],
        "actions": [str(a) for a in result.actions],
        "edge_risks": [float(v) for v in result.edge_risks],
        "edge_risk_vectors": [[float(x) for x in r] for r in result.edge_risk_vectors],
    }
    return {
        "scene_id": scene.scene_id,
        "vehicle_id": vehicle.vehicle_id,
        "method": method,
        "metrics": row,
        "path": path_row,
    }


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    scenes_dir = run_dir / "scenes"
    config_dir = args.config_dir.resolve()
    ckpt = args.checkpoint.resolve()

    scenes = sorted(scenes_dir.glob(str(args.scene_glob)))
    if not scenes:
        raise RuntimeError(f"no scenes found by glob={args.scene_glob} in {scenes_dir}")
    vehicles = ["urban_small", "standard_offroad", "mountain_large"]
    methods = [METHOD_BASELINE_1, METHOD_BASELINE_2, METHOD_BASELINE_3, METHOD_PROPOSED]

    tasks = []
    for sp in scenes:
        for v in vehicles:
            for m in methods:
                tasks.append({"scene_npz": str(sp), "vehicle_id": v, "method": m})

    num_workers = max(int(args.num_workers), 1)
    print(f"[custom-plan] tasks={len(tasks)} workers={num_workers}", flush=True)
    metrics_rows: list[dict[str, Any]] = []
    path_rows: list[dict[str, Any]] = []

    with ProcessPoolExecutor(
        max_workers=num_workers,
        initializer=init_worker,
        mp_context=mp.get_context("spawn"),
        initargs=(
            str(config_dir),
            str(ckpt),
            int(args.goal_radius_cells),
            int(args.max_expansions),
            int(args.max_labels_per_state),
            float(args.manual_risk_lambda),
            float(args.ml_risk_lambda),
            float(args.proposed_risk_lambda),
            float(args.proposed_manual_guard_weight),
            float(args.edge_safe),
            float(args.path_max_safe),
            float(args.path_avg_safe),
            int(args.worker_threads),
        ),
    ) as ex:
        futs = [ex.submit(run_task, t) for t in tasks]
        done = 0
        for fut in as_completed(futs):
            r = fut.result()
            done += 1
            print(f"[custom-plan] {done}/{len(tasks)} scene={r['scene_id']} vehicle={r['vehicle_id']} method={r['method']}", flush=True)
            metrics_rows.append(r["metrics"])
            path_rows.append(r["path"])

    metrics_csv = run_dir / "planning_metrics.csv"
    with metrics_csv.open("w", encoding="utf-8", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=list(metrics_rows[0].keys()))
        w.writeheader()
        w.writerows(metrics_rows)

    paths_jsonl = run_dir / "planning_paths.jsonl"
    with paths_jsonl.open("w", encoding="utf-8") as fp:
        for row in path_rows:
            fp.write(json.dumps(row, ensure_ascii=False) + "\n")

    summary = {
        "run_dir": str(run_dir),
        "num_rows": len(metrics_rows),
        "num_paths": len(path_rows),
        "num_workers": int(num_workers),
        "methods": methods,
        "vehicles": vehicles,
        "goal_radius_cells": int(args.goal_radius_cells),
        "max_expansions": int(args.max_expansions),
        "max_labels_per_state": int(args.max_labels_per_state),
        "manual_risk_lambda": float(args.manual_risk_lambda),
        "ml_risk_lambda": float(args.ml_risk_lambda),
        "proposed_risk_lambda": float(args.proposed_risk_lambda),
        "proposed_manual_guard_weight": float(args.proposed_manual_guard_weight),
        "edge_safe": float(args.edge_safe),
        "path_max_safe": float(args.path_max_safe),
        "path_avg_safe": float(args.path_avg_safe),
        "scene_glob": str(args.scene_glob),
    }
    (run_dir / "planning_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
