#!/usr/bin/env python3
"""Ablation: compare pure manual-guard vs ML+guard planning.

Runs two planning methods on the same scenes:
  - guard_only_constrained_astar: manual geometric risk + proposed constraints
  - proposed_ml_risk_constrained_astar: ML risk + manual guard + proposed constraints

Usage:
  cd ~/文档/Machine_Learning_25D/ml25d_ws
  .venv/bin/python3 src/ml25d_dataset_generation/scripts/ablate_guard_vs_model.py
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.config_loader import build_action_library, build_vehicle_library, load_all_configs
from ml25d_dataset_generation.planning_types import PlannerThresholds
from ml25d_dataset_generation.risk_astar import (
    METHOD_GUARD_ONLY,
    METHOD_PROPOSED,
    PlannerConfig,
    plan_path,
)
from ml25d_dataset_generation.risk_model_infer import RiskModelInfer


def _load_scene_npz(path: str | Path):
    from ml25d_dataset_generation.planning_types import PlanningScene

    p = Path(path)
    s = np.load(p)
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ablation: guard-only vs ML+guard planning")
    parser.add_argument(
        "--scenes-dir",
        type=Path,
        default=Path("data/planning_runs/calibrated_compare_v6_n14_fastfix/scenes"),
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("data/training_runs/cnn_pso_dataset_v1_20k/pso/best_model_calibrated.pt"),
    )
    parser.add_argument("--config-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=Path("data/ablation_guard_vs_model"))
    parser.add_argument("--device", type=str, default="auto")
    parser.add_argument("--guard-weight", type=float, default=0.5, help="proposed_manual_guard_weight for ML+guard")
    parser.add_argument("--edge-safe", type=float, default=0.70)
    parser.add_argument("--path-max-safe", type=float, default=0.90)
    parser.add_argument("--path-avg-safe", type=float, default=0.49)
    parser.add_argument(
        "--vehicles",
        type=str,
        default="urban_small,standard_offroad,mountain_large",
    )
    parser.add_argument("--max-expansions", type=int, default=60000)
    parser.add_argument("--max-labels-per-state", type=int, default=4)
    return parser.parse_args()


def _summarize(rows: list[dict], label: str) -> dict[str, Any]:
    found = sum(1 for r in rows if r["found"])
    total = len(rows)
    lengths = [r["path_length_m"] for r in rows if r["found"]]
    rmax = [r["risk_max"] for r in rows if r["found"]]
    ravg = [r["risk_avg"] for r in rows if r["found"]]
    times = [r["planning_time_ms"] for r in rows if r["found"]]

    def _stats(vals):
        if not vals:
            return {}
        return {
            "mean": float(np.mean(vals)),
            "median": float(np.median(vals)),
            "std": float(np.std(vals)),
            "min": float(np.min(vals)),
            "max": float(np.max(vals)),
        }

    return {
        "method": label,
        "found": f"{found}/{total} ({100*found/total:.1f}%)",
        "success_rate": float(found / max(total, 1)),
        "path_length_m": _stats(lengths),
        "risk_max": _stats(rmax),
        "risk_avg": _stats(ravg),
        "planning_time_ms": _stats(times),
        "expanded_mean": float(np.mean([r["expanded_nodes"] for r in rows if r["found"]])) if found else 0,
    }


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    config_dir = args.config_dir or (package_root / "config")

    scenes_dir = args.scenes_dir
    scene_files = sorted(scenes_dir.glob("*.npz"))
    if not scene_files:
        scene_files = sorted((Path.cwd() / scenes_dir).glob("*.npz"))
    if not scene_files:
        print(f"No scene npz files found in: {scenes_dir}")
        print(f"cwd: {Path.cwd()}")
        return 1

    print(f"Loading {len(scene_files)} scenes from {scenes_dir.resolve()}")
    scenes = [_load_scene_npz(f) for f in scene_files]

    # Also gather available terrain classes and all scene ids
    terrain_classes = sorted(set(s.terrain_class for s in scenes))
    print(f"Terrain classes: {terrain_classes}")

    cfg = load_all_configs(config_dir)
    vehicles_all = {v.vehicle_id: v for v in build_vehicle_library(cfg["vehicles"])}
    selected_vehicles = args.vehicles.split(",")
    vehicles = {}
    for vid in selected_vehicles:
        if vid in vehicles_all:
            vehicles[vid] = vehicles_all[vid]
    print(f"Vehicles: {list(vehicles.keys())}")

    actions = [a for a in build_action_library(cfg["actions"]) if a.delta_s_m > 1e-4]
    actions_by_id = {a.action_id: a for a in actions}
    if {"a0", "a1", "a2"}.issubset(actions_by_id.keys()):
        actions = [actions_by_id["a0"], actions_by_id["a1"], actions_by_id["a2"]]

    # Load model
    checkpoint = args.checkpoint
    if not checkpoint.exists():
        checkpoint = Path.cwd() / checkpoint
    print(f"Loading model from {checkpoint}")
    model_infer = RiskModelInfer(checkpoint, config_dir=config_dir, device=args.device)

    thresholds = PlannerThresholds(
        edge_safe=args.edge_safe,
        path_max_safe=args.path_max_safe,
        path_avg_safe=args.path_avg_safe,
    )
    print(f"Thresholds: edge={thresholds.edge_safe}, pmax={thresholds.path_max_safe}, pavg={thresholds.path_avg_safe}")

    planner_cfg = PlannerConfig(
        max_expansions=args.max_expansions,
        max_labels_per_state=args.max_labels_per_state,
        proposed_manual_guard_weight=float(args.guard_weight),
        proposed_risk_lambda=0.0,
    )

    output_dir = args.output_dir
    output_dir = Path.cwd() / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    all_rows: list[dict] = []

    total_tasks = len(scenes) * len(vehicles)
    completed = 0
    t_start = time.monotonic()

    for scene in scenes:
        for vid, vehicle in vehicles.items():
            completed += 1

            # --- Guard Only ---
            t0 = time.perf_counter()
            try:
                result_g = plan_path(
                    scene=scene,
                    vehicle=vehicle,
                    actions=actions,
                    method=METHOD_GUARD_ONLY,
                    model_infer=None,
                    config=planner_cfg,
                    thresholds=thresholds,
                )
                t_g = (time.perf_counter() - t0) * 1000
                row_g = {
                    "scene_id": scene.scene_id,
                    "terrain_class": scene.terrain_class,
                    "vehicle_id": vid,
                    "method": "guard_only",
                    "found": result_g.found,
                    "fail_reason": result_g.fail_reason or "",
                    "path_length_m": float(result_g.path_length_m),
                    "risk_max": float(result_g.risk_max),
                    "risk_avg": float(result_g.risk_avg),
                    "expanded_nodes": int(result_g.expanded_nodes),
                    "planning_time_ms": float(t_g),
                    "num_states": len(list(result_g.states)),
                    "num_actions": len(list(result_g.actions)),
                }
            except Exception as exc:
                row_g = {
                    "scene_id": scene.scene_id,
                    "terrain_class": scene.terrain_class,
                    "vehicle_id": vid,
                    "method": "guard_only",
                    "found": False,
                    "fail_reason": f"exception: {exc}",
                    "path_length_m": 0.0,
                    "risk_max": 0.0,
                    "risk_avg": 0.0,
                    "expanded_nodes": 0,
                    "planning_time_ms": 0.0,
                    "num_states": 0,
                    "num_actions": 0,
                }
            all_rows.append(row_g)

            # --- ML + Guard ---
            t0 = time.perf_counter()
            try:
                result_m = plan_path(
                    scene=scene,
                    vehicle=vehicle,
                    actions=actions,
                    method=METHOD_PROPOSED,
                    model_infer=model_infer,
                    config=planner_cfg,
                    thresholds=thresholds,
                )
                t_m = (time.perf_counter() - t0) * 1000
                row_m = {
                    "scene_id": scene.scene_id,
                    "terrain_class": scene.terrain_class,
                    "vehicle_id": vid,
                    "method": "ml_guard",
                    "found": result_m.found,
                    "fail_reason": result_m.fail_reason or "",
                    "path_length_m": float(result_m.path_length_m),
                    "risk_max": float(result_m.risk_max),
                    "risk_avg": float(result_m.risk_avg),
                    "expanded_nodes": int(result_m.expanded_nodes),
                    "planning_time_ms": float(t_m),
                    "num_states": len(list(result_m.states)),
                    "num_actions": len(list(result_m.actions)),
                }
            except Exception as exc:
                row_m = {
                    "scene_id": scene.scene_id,
                    "terrain_class": scene.terrain_class,
                    "vehicle_id": vid,
                    "method": "ml_guard",
                    "found": False,
                    "fail_reason": f"exception: {exc}",
                    "path_length_m": 0.0,
                    "risk_max": 0.0,
                    "risk_avg": 0.0,
                    "expanded_nodes": 0,
                    "planning_time_ms": 0.0,
                    "num_states": 0,
                    "num_actions": 0,
                }
            all_rows.append(row_m)

            elapsed = time.monotonic() - t_start
            rate = completed / max(elapsed, 1e-6)
            eta = (total_tasks - completed) / max(rate, 1e-6)
            g_found = "Y" if row_g["found"] else "N"
            m_found = "Y" if row_m["found"] else "N"
            print(
                f"  [{completed}/{total_tasks}] {scene.scene_id} {vid} "
                f"| guard={g_found} ml+guard={m_found} "
                f"| {elapsed:.0f}s ETA {eta:.0f}s"
            )

    # --- Summaries ---
    guard_rows = [r for r in all_rows if r["method"] == "guard_only"]
    ml_rows = [r for r in all_rows if r["method"] == "ml_guard"]

    guard_summary = _summarize(guard_rows, "guard_only (pure manual risk + constraints)")
    ml_summary = _summarize(ml_rows, f"ml_guard (ML risk + guard w={args.guard_weight})")

    print()
    print("=" * 80)
    print("  ABLATION RESULTS")
    print("=" * 80)
    for s in [guard_summary, ml_summary]:
        print(f"\n{s['method']}:")
        print(f"  Success rate: {s['found']}")
        for metric in ["path_length_m", "risk_max", "risk_avg", "planning_time_ms"]:
            stats = s.get(metric, {})
            if stats:
                print(
                    f"  {metric}: mean={stats['mean']:.3f} "
                    f"median={stats['median']:.3f} "
                    f"std={stats['std']:.3f}"
                )
        print(f"  expanded_nodes mean: {s.get('expanded_mean', 0):.0f}")

    # --- Per-scene diff ---
    print()
    print("=" * 80)
    print("  PER-SCENE DIFFERENCE (ml_guard - guard_only)")
    print("=" * 80)
    diff_found = 0
    for scene in scenes:
        for vid in vehicles:
            g = next((r for r in guard_rows if r["scene_id"] == scene.scene_id and r["vehicle_id"] == vid), None)
            m = next((r for r in ml_rows if r["scene_id"] == scene.scene_id and r["vehicle_id"] == vid), None)
            if g and m and g["found"] != m["found"]:
                diff_found += 1
                winner = "ML" if m["found"] else "GUARD"
                print(
                    f"  {scene.scene_id} {vid}: guard={'Y' if g['found'] else 'N'} "
                    f"ml={'Y' if m['found'] else 'N'} → {winner} wins"
                )
    if diff_found == 0:
        print("  All scenes have same found/not-found status for both methods.")

    # Per-terrain breakdown
    print()
    print("=" * 80)
    print("  SUCCESS RATE BY TERRAIN CLASS")
    print("=" * 80)
    for tc in terrain_classes:
        g_tc = [r for r in guard_rows if r["terrain_class"] == tc]
        m_tc = [r for r in ml_rows if r["terrain_class"] == tc]
        g_rate = sum(1 for r in g_tc if r["found"]) / max(len(g_tc), 1)
        m_rate = sum(1 for r in m_tc if r["found"]) / max(len(m_tc), 1)
        diff = m_rate - g_rate
        marker = " *** ML BETTER" if diff > 0.05 else (" *** GUARD BETTER" if diff < -0.05 else "")
        print(f"  {tc:20s}: guard={g_rate:.2f} ml+guard={m_rate:.2f} diff={diff:+.2f}{marker}")

    # Save
    results = {
        "config": {
            "guard_weight": args.guard_weight,
            "thresholds": {"edge_safe": thresholds.edge_safe, "path_max_safe": thresholds.path_max_safe, "path_avg_safe": thresholds.path_avg_safe},
            "checkpoint": str(checkpoint),
            "num_scenes": len(scenes),
            "vehicles": list(vehicles.keys()),
        },
        "guard_only_summary": guard_summary,
        "ml_guard_summary": ml_summary,
        "rows": all_rows,
    }
    out_path = output_dir / "ablation_results.json"
    with out_path.open("w", encoding="utf-8") as fp:
        json.dump(results, fp, indent=2, ensure_ascii=False)
    print(f"\nFull results saved to {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
