#!/usr/bin/env python3
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import json
import multiprocessing as mp
import os
from pathlib import Path
from typing import Any

import numpy as np

from ml25d_dataset_generation.config_loader import build_action_library, build_vehicle_library, load_all_configs
from ml25d_dataset_generation.map_builder_from_h5 import H5PlanningMapBuilder
from ml25d_dataset_generation.planning_types import PlannerThresholds
from ml25d_dataset_generation.risk_astar import (
    ALL_METHODS,
    METHOD_BASELINE_1,
    METHOD_BASELINE_2,
    METHOD_BASELINE_3,
    METHOD_PROPOSED,
    PlannerConfig,
    plan_path,
)
from ml25d_dataset_generation.risk_model_infer import RiskModelInfer
from ml25d_dataset_generation.training_data import load_hdf5_dataset

_WORKER_CTX: dict[str, Any] | None = None


def _discover_config_dir(package_root: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        p = explicit.resolve()
        if not (p / "dataset_config.yaml").exists():
            raise FileNotFoundError(f"dataset_config.yaml not found in: {p}")
        return p
    candidates = [
        package_root / "config",
        Path.cwd() / "src" / "ml25d_dataset_generation" / "config",
        package_root.parent.parent / "install" / "ml25d_dataset_generation" / "share" / "ml25d_dataset_generation" / "config",
    ]
    for c in candidates:
        if (c / "dataset_config.yaml").exists():
            return c.resolve()
    raise FileNotFoundError("cannot find config dir")


def _parse_list(raw: str) -> list[str]:
    out = [part.strip() for part in str(raw).split(",") if part.strip()]
    return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run planning benchmark with 4 A* variants")
    parser.add_argument(
        "--pattern",
        type=str,
        default="data/generated_dataset_v1_20k_n6_comp/shard_*/samples_batch_*.h5",
        help="H5 glob pattern",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config-dir", type=Path, default=None)
    parser.add_argument(
        "--checkpoint-main",
        type=Path,
        required=True,
        help="Main model checkpoint (typically PSO model) for ML methods",
    )
    parser.add_argument(
        "--checkpoint-compare",
        type=Path,
        default=None,
        help="Optional compare checkpoint (typically no-PSO baseline model)",
    )
    parser.add_argument("--device", type=str, default="auto")
    parser.add_argument("--num-workers", type=int, default=1, help="Parallel workers across scene×vehicle tasks")
    parser.add_argument("--worker-threads", type=int, default=1, help="Torch/BLAS threads per worker")
    parser.add_argument("--seed", type=int, default=20260428)
    parser.add_argument("--global-size", type=int, default=121)
    parser.add_argument("--overlap-cells", type=int, default=15)
    parser.add_argument("--scenes-per-terrain", type=int, default=3)
    parser.add_argument(
        "--terrain-classes",
        type=str,
        default="flat,uniform_slope,lateral_slope,steps,pits,bumps,slope_bumps,lateral_pits,mixed_random",
    )
    parser.add_argument(
        "--vehicles",
        type=str,
        default="urban_small,standard_offroad,mountain_large",
    )
    parser.add_argument(
        "--methods",
        type=str,
        default="baseline1_2p5d_astar,baseline2_manual_risk_weighted_astar,baseline3_ml_risk_weighted_astar,proposed_ml_risk_constrained_astar",
    )
    parser.add_argument("--goal-radius-cells", type=int, default=2)
    parser.add_argument("--max-expansions", type=int, default=60000)
    parser.add_argument("--max-labels-per-state", type=int, default=4)
    parser.add_argument("--manual-risk-lambda", type=float, default=1.0)
    parser.add_argument("--ml-risk-lambda", type=float, default=1.0)
    parser.add_argument(
        "--proposed-risk-lambda",
        type=float,
        default=0.0,
        help="Extra risk shaping weight for proposed method (constraints still active)",
    )
    parser.add_argument(
        "--proposed-manual-guard-weight",
        type=float,
        default=0.0,
        help="Blend weight for manual geometric guard in proposed method (0..1).",
    )
    parser.add_argument("--proposed-edge-safe", type=float, default=0.0, help="<=0 means use checkpoint threshold")
    parser.add_argument("--proposed-path-max-safe", type=float, default=0.0, help="<=0 means use checkpoint threshold")
    parser.add_argument("--proposed-path-avg-safe", type=float, default=0.0, help="<=0 means use checkpoint threshold")
    return parser.parse_args()


def _model_tags_for_method(
    method: str,
    main_infer: RiskModelInfer,
    compare_infer: RiskModelInfer | None,
) -> list[tuple[str, RiskModelInfer | None]]:
    if method in {METHOD_BASELINE_1, METHOD_BASELINE_2}:
        return [("none", None)]
    tags: list[tuple[str, RiskModelInfer | None]] = [("main", main_infer)]
    if compare_infer is not None:
        tags.append(("compare", compare_infer))
    return tags


def _threshold_override(args: argparse.Namespace) -> PlannerThresholds | None:
    if args.proposed_edge_safe > 0 and args.proposed_path_max_safe > 0 and args.proposed_path_avg_safe > 0:
        return PlannerThresholds(
            edge_safe=float(args.proposed_edge_safe),
            path_max_safe=float(args.proposed_path_max_safe),
            path_avg_safe=float(args.proposed_path_avg_safe),
        )
    return None


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


def _init_worker(
    config_dir: str,
    checkpoint_main: str,
    checkpoint_compare: str | None,
    device: str,
    methods: list[str],
    vehicles: list[str],
    planner_cfg_json: dict[str, Any],
    thresholds_override_json: dict[str, float] | None,
    worker_threads: int,
) -> None:
    global _WORKER_CTX

    threads = max(int(worker_threads), 1)
    os.environ["OMP_NUM_THREADS"] = str(threads)
    os.environ["OPENBLAS_NUM_THREADS"] = str(threads)
    os.environ["MKL_NUM_THREADS"] = str(threads)
    os.environ["NUMEXPR_NUM_THREADS"] = str(threads)
    os.environ["VECLIB_MAXIMUM_THREADS"] = str(threads)
    os.environ["BLIS_NUM_THREADS"] = str(threads)

    cfg = load_all_configs(Path(config_dir))
    actions = [a for a in build_action_library(cfg["actions"]) if a.delta_s_m > 1e-4]
    actions_by_id = {a.action_id: a for a in actions}
    if {"a0", "a1", "a2"}.issubset(actions_by_id.keys()):
        actions = [actions_by_id["a0"], actions_by_id["a1"], actions_by_id["a2"]]

    vehicles_all = {v.vehicle_id: v for v in build_vehicle_library(cfg["vehicles"])}
    selected_vehicles = {}
    for vid in vehicles:
        if vid not in vehicles_all:
            raise KeyError(f"unknown vehicle id in worker init: {vid}")
        selected_vehicles[vid] = vehicles_all[vid]

    main_infer = RiskModelInfer(checkpoint_main, config_dir=config_dir, device=device)
    compare_infer = None
    if checkpoint_compare:
        compare_infer = RiskModelInfer(checkpoint_compare, config_dir=config_dir, device=device)
    try:
        torch = main_infer.torch
        torch.set_num_threads(threads)
        torch.set_num_interop_threads(1)
    except Exception:
        pass

    planner_cfg = PlannerConfig(
        goal_radius_cells=int(planner_cfg_json["goal_radius_cells"]),
        max_expansions=int(planner_cfg_json["max_expansions"]),
        max_labels_per_state=int(planner_cfg_json["max_labels_per_state"]),
        manual_risk_lambda=float(planner_cfg_json["manual_risk_lambda"]),
        ml_risk_lambda=float(planner_cfg_json["ml_risk_lambda"]),
        proposed_risk_lambda=float(planner_cfg_json.get("proposed_risk_lambda", 0.0)),
        proposed_manual_guard_weight=float(planner_cfg_json.get("proposed_manual_guard_weight", 0.0)),
        default_thresholds=PlannerThresholds(
            edge_safe=float(planner_cfg_json["default_thresholds"]["edge_safe"]),
            path_max_safe=float(planner_cfg_json["default_thresholds"]["path_max_safe"]),
            path_avg_safe=float(planner_cfg_json["default_thresholds"]["path_avg_safe"]),
        ),
    )
    thresholds_override = None
    if thresholds_override_json is not None:
        thresholds_override = PlannerThresholds(
            edge_safe=float(thresholds_override_json["edge_safe"]),
            path_max_safe=float(thresholds_override_json["path_max_safe"]),
            path_avg_safe=float(thresholds_override_json["path_avg_safe"]),
        )

    _WORKER_CTX = {
        "methods": list(methods),
        "actions": actions,
        "vehicles": selected_vehicles,
        "main_infer": main_infer,
        "compare_infer": compare_infer,
        "planner_cfg": planner_cfg,
        "thresholds_override": thresholds_override,
    }


def _run_scene_vehicle_task(task: dict[str, Any]) -> dict[str, Any]:
    global _WORKER_CTX
    if _WORKER_CTX is None:
        raise RuntimeError("worker context is not initialized")
    scene = _load_scene_npz(task["scene_npz"])
    vehicle_id = str(task["vehicle_id"])
    vehicle = _WORKER_CTX["vehicles"][vehicle_id]
    method_override = str(task.get("method", "")).strip()
    if method_override:
        methods = [method_override]
    else:
        methods = _WORKER_CTX["methods"]
    actions = _WORKER_CTX["actions"]
    main_infer = _WORKER_CTX["main_infer"]
    compare_infer = _WORKER_CTX["compare_infer"]
    planner_cfg = _WORKER_CTX["planner_cfg"]
    thresholds_override = _WORKER_CTX["thresholds_override"]

    metrics_rows: list[dict[str, Any]] = []
    path_rows: list[dict[str, Any]] = []
    for method in methods:
        for model_tag, infer in _model_tags_for_method(method, main_infer, compare_infer):
            use_thresholds = thresholds_override if method == METHOD_PROPOSED else None
            result = plan_path(
                scene=scene,
                vehicle=vehicle,
                actions=actions,
                method=method,
                model_infer=infer,
                config=planner_cfg,
                thresholds=use_thresholds,
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
                "goal_radius_cells": int(planner_cfg.goal_radius_cells),
            }
            row.update(result.as_metrics_dict())
            metrics_rows.append(row)
            path_rows.append(
                {
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
            )
    return {
        "metrics_rows": metrics_rows,
        "path_rows": path_rows,
        "scene_id": scene.scene_id,
        "vehicle_id": vehicle_id,
        "method": method_override if method_override else "all",
    }


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    config_dir = _discover_config_dir(package_root, args.config_dir)
    cfg = load_all_configs(config_dir)
    map_cfg = cfg["dataset"]["map"]
    action_cfg = cfg["actions"]

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    scenes_dir = output_dir / "scenes"
    scenes_dir.mkdir(parents=True, exist_ok=True)

    data = load_hdf5_dataset(args.pattern)
    builder = H5PlanningMapBuilder(
        data=data,
        patch_size=int(map_cfg["patch_size"]),
        resolution_m=float(map_cfg["resolution_m_per_cell"]),
        heading_bins=int(action_cfg["actions"]["heading_bins"]),
        seed=int(args.seed),
    )
    terrain_classes_req = _parse_list(args.terrain_classes)
    terrain_classes = [t for t in terrain_classes_req if t in set(builder.available_terrain_classes())]
    if not terrain_classes:
        raise RuntimeError(f"none of requested terrain classes exist in dataset: {terrain_classes_req}")

    actions = [a for a in build_action_library(cfg["actions"]) if a.delta_s_m > 1e-4]
    if not actions:
        raise RuntimeError("no forward actions found")
    actions_by_id = {a.action_id: a for a in actions}
    # Keep exactly the main action pool by default.
    if {"a0", "a1", "a2"}.issubset(actions_by_id.keys()):
        actions = [actions_by_id["a0"], actions_by_id["a1"], actions_by_id["a2"]]

    vehicles_all = {v.vehicle_id: v for v in build_vehicle_library(cfg["vehicles"])}
    vehicles = []
    for vid in _parse_list(args.vehicles):
        if vid not in vehicles_all:
            raise KeyError(f"unknown vehicle id: {vid}")
        vehicles.append(vehicles_all[vid])

    methods = _parse_list(args.methods)
    for m in methods:
        if m not in ALL_METHODS:
            raise ValueError(f"unsupported method: {m}")

    main_infer = RiskModelInfer(args.checkpoint_main, config_dir=config_dir, device=args.device)
    planner_cfg = PlannerConfig(
        goal_radius_cells=int(args.goal_radius_cells),
        max_expansions=int(args.max_expansions),
        max_labels_per_state=int(args.max_labels_per_state),
        manual_risk_lambda=float(args.manual_risk_lambda),
        ml_risk_lambda=float(args.ml_risk_lambda),
        proposed_risk_lambda=float(args.proposed_risk_lambda),
        proposed_manual_guard_weight=float(args.proposed_manual_guard_weight),
        default_thresholds=main_infer.thresholds,
    )
    thresholds_override = _threshold_override(args)

    planner_cfg_json = {
        "goal_radius_cells": int(planner_cfg.goal_radius_cells),
        "max_expansions": int(planner_cfg.max_expansions),
        "max_labels_per_state": int(planner_cfg.max_labels_per_state),
        "manual_risk_lambda": float(planner_cfg.manual_risk_lambda),
        "ml_risk_lambda": float(planner_cfg.ml_risk_lambda),
        "proposed_risk_lambda": float(planner_cfg.proposed_risk_lambda),
        "proposed_manual_guard_weight": float(planner_cfg.proposed_manual_guard_weight),
        "default_thresholds": {
            "edge_safe": float(planner_cfg.default_thresholds.edge_safe),
            "path_max_safe": float(planner_cfg.default_thresholds.path_max_safe),
            "path_avg_safe": float(planner_cfg.default_thresholds.path_avg_safe),
        },
    }
    thresholds_override_json = None
    if thresholds_override is not None:
        thresholds_override_json = {
            "edge_safe": float(thresholds_override.edge_safe),
            "path_max_safe": float(thresholds_override.path_max_safe),
            "path_avg_safe": float(thresholds_override.path_avg_safe),
        }

    metrics_rows: list[dict[str, Any]] = []
    path_rows: list[dict[str, Any]] = []
    scene_npz_paths: list[str] = []
    for terrain in terrain_classes:
        for scene_idx in range(1, int(args.scenes_per_terrain) + 1):
            scene = builder.build_scene(
                terrain_class=terrain,
                scene_index=scene_idx,
                global_size=int(args.global_size),
                overlap_cells=int(args.overlap_cells),
            )
            scene_npz = scenes_dir / f"{scene.scene_id}.npz"
            H5PlanningMapBuilder.save_scene_npz(scene, str(scene_npz))
            scene_npz_paths.append(str(scene_npz))

    tasks: list[dict[str, Any]] = []
    for scene_npz in scene_npz_paths:
        for vehicle in vehicles:
            for method in methods:
                tasks.append({"scene_npz": scene_npz, "vehicle_id": vehicle.vehicle_id, "method": method})

    num_workers = max(int(args.num_workers), 1)
    if num_workers > 1 and str(args.device).startswith("cuda"):
        print("[plan] warning: num_workers>1 with cuda may oversubscribe GPU; prefer --device cpu for high parallel speed", flush=True)

    print(
        f"[plan] tasks={len(tasks)} workers={num_workers} methods={len(methods)}",
        flush=True,
    )

    if num_workers == 1:
        _init_worker(
            str(config_dir),
            str(args.checkpoint_main),
            None if args.checkpoint_compare is None else str(args.checkpoint_compare),
            str(args.device),
            list(methods),
            [v.vehicle_id for v in vehicles],
            planner_cfg_json,
            thresholds_override_json,
            int(args.worker_threads),
        )
        for idx, task in enumerate(tasks, start=1):
            result = _run_scene_vehicle_task(task)
            print(
                f"[plan] {idx}/{len(tasks)} scene={result['scene_id']} vehicle={result['vehicle_id']} method={result['method']}",
                flush=True,
            )
            metrics_rows.extend(result["metrics_rows"])
            path_rows.extend(result["path_rows"])
    else:
        with ProcessPoolExecutor(
            max_workers=num_workers,
            initializer=_init_worker,
            mp_context=mp.get_context("spawn"),
            initargs=(
                str(config_dir),
                str(args.checkpoint_main),
                None if args.checkpoint_compare is None else str(args.checkpoint_compare),
                str(args.device),
                list(methods),
                [v.vehicle_id for v in vehicles],
                planner_cfg_json,
                thresholds_override_json,
                int(args.worker_threads),
            ),
        ) as executor:
            futures = [executor.submit(_run_scene_vehicle_task, task) for task in tasks]
            done = 0
            for fut in as_completed(futures):
                result = fut.result()
                done += 1
                print(
                    f"[plan] {done}/{len(tasks)} scene={result['scene_id']} vehicle={result['vehicle_id']} method={result['method']}",
                    flush=True,
                )
                metrics_rows.extend(result["metrics_rows"])
                path_rows.extend(result["path_rows"])

    metrics_csv = output_dir / "planning_metrics.csv"
    if not metrics_rows:
        raise RuntimeError("no planning results generated")
    fieldnames = list(metrics_rows[0].keys())
    with metrics_csv.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics_rows)

    paths_jsonl = output_dir / "planning_paths.jsonl"
    with paths_jsonl.open("w", encoding="utf-8") as fp:
        for row in path_rows:
            fp.write(json.dumps(row, ensure_ascii=False) + "\n")

    summary = {
        "pattern": str(args.pattern),
        "output_dir": str(output_dir),
        "config_dir": str(config_dir),
        "checkpoint_main": str(args.checkpoint_main),
        "checkpoint_compare": str(args.checkpoint_compare) if args.checkpoint_compare else None,
        "terrain_classes": terrain_classes,
        "vehicles": [v.vehicle_id for v in vehicles],
        "methods": methods,
        "num_rows": len(metrics_rows),
        "num_paths": len(path_rows),
        "num_workers": int(num_workers),
        "scenes_per_terrain": int(args.scenes_per_terrain),
        "global_size": int(args.global_size),
        "overlap_cells": int(args.overlap_cells),
        "thresholds_main": main_infer.debug_thresholds(),
        "threshold_override": {
            "edge_safe": None if thresholds_override is None else thresholds_override.edge_safe,
            "path_max_safe": None if thresholds_override is None else thresholds_override.path_max_safe,
            "path_avg_safe": None if thresholds_override is None else thresholds_override.path_avg_safe,
        },
    }
    summary_path = output_dir / "planning_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"metrics_csv": str(metrics_csv), "paths_jsonl": str(paths_jsonl), "summary": str(summary_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
