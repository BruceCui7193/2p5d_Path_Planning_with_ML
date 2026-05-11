#!/usr/bin/env python3
"""Add guard_only planning results to existing custom scattered benchmark run.

Appends guard_only_constrained_astar results to the existing planning_metrics.csv
and planning_paths.jsonl in the target run directory.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.config_loader import build_action_library, build_vehicle_library, load_all_configs
from ml25d_dataset_generation.planning_types import PlannerThresholds, PlanningScene
from ml25d_dataset_generation.risk_astar import (
    METHOD_GUARD_ONLY,
    PlannerConfig,
    plan_path,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Add guard_only results to an existing planning run")
    p.add_argument(
        "--run-dir",
        type=Path,
        default=Path("/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/planning_runs/custom_random_scattered_obstacles_v2_guard_tight"),
    )
    p.add_argument(
        "--config-dir",
        type=Path,
        default=Path("/home/crh/文档/Machine_Learning_25D/ml25d_ws/src/ml25d_dataset_generation/config"),
    )
    p.add_argument("--max-expansions", type=int, default=140000)
    p.add_argument("--max-labels-per-state", type=int, default=4)
    p.add_argument("--goal-radius-cells", type=int, default=2)
    p.add_argument("--proposed-risk-lambda", type=float, default=0.2)
    p.add_argument("--edge-safe", type=float, default=0.55)
    p.add_argument("--path-max-safe", type=float, default=0.82)
    p.add_argument("--path-avg-safe", type=float, default=0.36)
    return p.parse_args()


def load_scene(path: Path) -> PlanningScene:
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


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    scenes_dir = run_dir / "scenes"
    config_dir = args.config_dir.resolve()

    cfg = load_all_configs(config_dir)
    actions = [a for a in build_action_library(cfg["actions"]) if a.action_id in {"a0", "a1", "a2"}]
    actions.sort(key=lambda a: a.action_id)
    vehicles_all = {v.vehicle_id: v for v in build_vehicle_library(cfg["vehicles"])}
    vehicles = {k: vehicles_all[k] for k in ["urban_small", "standard_offroad", "mountain_large"]}

    thresholds = PlannerThresholds(
        edge_safe=args.edge_safe,
        path_max_safe=args.path_max_safe,
        path_avg_safe=args.path_avg_safe,
    )
    planner_cfg = PlannerConfig(
        goal_radius_cells=args.goal_radius_cells,
        max_expansions=args.max_expansions,
        max_labels_per_state=args.max_labels_per_state,
        proposed_risk_lambda=float(args.proposed_risk_lambda),
        proposed_manual_guard_weight=0.0,
    )

    scenes = sorted(scenes_dir.glob("custom_scattered_scene_*.npz"))
    if not scenes:
        scenes = sorted(scenes_dir.glob("*.npz"))
    print(f"Found {len(scenes)} scenes in {scenes_dir}")

    # Read existing metrics to check what's already done
    metrics_csv = run_dir / "planning_metrics.csv"
    existing_ids = set()
    if metrics_csv.exists():
        with metrics_csv.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                existing_ids.add((row["scene_id"], row["vehicle_id"], row["method"]))
    print(f"Existing rows: {len(existing_ids)}")

    new_metrics = []
    new_paths = []

    for sp in scenes:
        scene = load_scene(sp)
        for vid, vehicle in vehicles.items():
            key = (str(scene.scene_id), vid, METHOD_GUARD_ONLY)
            if key in existing_ids:
                print(f"  SKIP {scene.scene_id} {vid} guard_only (already exists)")
                continue

            print(f"  RUN  {scene.scene_id} {vid} guard_only ...", end=" ", flush=True)
            result = plan_path(
                scene=scene,
                vehicle=vehicle,
                actions=actions,
                method=METHOD_GUARD_ONLY,
                model_infer=None,
                config=planner_cfg,
                thresholds=thresholds,
            )

            run_id = f"{scene.scene_id}__{vid}__{METHOD_GUARD_ONLY}__none"
            row = {
                "run_id": run_id,
                "scene_id": scene.scene_id,
                "terrain_class": scene.terrain_class,
                "vehicle_id": vid,
                "method": METHOD_GUARD_ONLY,
                "model_tag": "none",
                "friction_mu": float(scene.friction_mu),
                "start_i": int(scene.start_state[0]),
                "start_j": int(scene.start_state[1]),
                "start_k": int(scene.start_state[2]),
                "goal_i": int(scene.goal_state[0]),
                "goal_j": int(scene.goal_state[1]),
                "goal_k": int(scene.goal_state[2]),
                "goal_radius_cells": int(planner_cfg.goal_radius_cells),
            }
            row.update(result.as_metrics_dict())
            new_metrics.append(row)

            path_row = {
                "run_id": run_id,
                "scene_id": scene.scene_id,
                "terrain_class": scene.terrain_class,
                "vehicle_id": vid,
                "method": METHOD_GUARD_ONLY,
                "model_tag": "none",
                "found": bool(result.found),
                "states": [[int(a), int(b), int(c)] for (a, b, c) in result.states],
                "actions": [str(a) for a in result.actions],
                "edge_risks": [float(v) for v in result.edge_risks],
                "edge_risk_vectors": [[float(x) for x in r] for r in result.edge_risk_vectors],
            }
            new_paths.append(path_row)
            print(f"found={result.found} len={result.path_length_m:.1f}m risk_max={result.risk_max:.3f}")

    if not new_metrics:
        print("No new results to add (all already present).")
        return 0

    # Append to CSV
    fieldnames = list(new_metrics[0].keys())
    with metrics_csv.open("a", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if metrics_csv.stat().st_size == 0:
            w.writeheader()
        w.writerows(new_metrics)

    # Append to JSONL
    paths_jsonl = run_dir / "planning_paths.jsonl"
    with paths_jsonl.open("a", encoding="utf-8") as f:
        for row in new_paths:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    # Update summary
    summary = json.loads((run_dir / "planning_summary.json").read_text(encoding="utf-8"))
    old_methods = summary.get("methods", [])
    if METHOD_GUARD_ONLY not in old_methods:
        old_methods.append(METHOD_GUARD_ONLY)
        summary["methods"] = old_methods
    summary["num_rows"] = int(summary.get("num_rows", 0)) + len(new_metrics)
    summary["num_paths"] = int(summary.get("num_paths", 0)) + len(new_paths)
    (run_dir / "planning_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\nAdded {len(new_metrics)} rows to {metrics_csv}")
    print(f"Added {len(new_paths)} paths to {paths_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
