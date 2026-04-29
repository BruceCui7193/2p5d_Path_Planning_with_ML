#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render proposed-only figures (2D + 3D)")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir-2d", type=Path, required=True)
    parser.add_argument("--output-dir-3d", type=Path, required=True)
    parser.add_argument("--model-tag", type=str, default="main")
    parser.add_argument("--z-scale", type=float, default=2.6)
    parser.add_argument("--path-lift-m", type=float, default=0.04)
    return parser.parse_args()


def _load_metrics(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            out: dict[str, Any] = dict(row)
            for key in [
                "found",
                "path_length_m",
                "risk_max",
                "risk_avg",
                "expanded_nodes",
                "planning_time_ms",
                "num_states",
                "num_actions",
                "friction_mu",
                "start_i",
                "start_j",
                "start_k",
                "goal_i",
                "goal_j",
                "goal_k",
                "goal_radius_cells",
            ]:
                if key not in out:
                    continue
                if key in {
                    "found",
                    "expanded_nodes",
                    "num_states",
                    "num_actions",
                    "start_i",
                    "start_j",
                    "start_k",
                    "goal_i",
                    "goal_j",
                    "goal_k",
                    "goal_radius_cells",
                }:
                    out[key] = int(float(out[key]))
                else:
                    out[key] = float(out[key])
            rows.append(out)
    return rows


def _load_paths(path: Path) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            out[str(row["run_id"])] = row
    return out


def _state_to_xy(states: list[list[int]], resolution_m: float) -> tuple[np.ndarray, np.ndarray]:
    if not states:
        return np.zeros(0, dtype=np.float32), np.zeros(0, dtype=np.float32)
    arr = np.asarray(states, dtype=np.float32)
    y = arr[:, 0] * float(resolution_m)
    x = arr[:, 1] * float(resolution_m)
    return x, y


def _state_to_xyz(
    states: list[list[int]],
    heightmap: np.ndarray,
    resolution_m: float,
    z_scale: float,
    path_lift_m: float,
) -> np.ndarray:
    if not states:
        return np.zeros((0, 3), dtype=np.float32)
    arr = np.asarray(states, dtype=np.int64)
    ii = np.clip(arr[:, 0], 0, heightmap.shape[0] - 1)
    jj = np.clip(arr[:, 1], 0, heightmap.shape[1] - 1)
    x = jj.astype(np.float32) * float(resolution_m)
    y = ii.astype(np.float32) * float(resolution_m)
    z = (heightmap[ii, jj].astype(np.float32) + float(path_lift_m)) * float(z_scale)
    return np.column_stack([x, y, z]).astype(np.float32)


def _terrain_grid_3d(pv, h: np.ndarray, res: float, z_scale: float):
    ny, nx = h.shape
    xs = np.arange(nx, dtype=np.float32) * float(res)
    ys = np.arange(ny, dtype=np.float32) * float(res)
    xx, yy = np.meshgrid(xs, ys)
    zz = h.astype(np.float32) * float(z_scale)
    grid = pv.StructuredGrid(xx, yy, zz)
    grid["height"] = zz.ravel(order="F")
    return grid


def _setup_camera_3d(plotter, h: np.ndarray, res: float, z_scale: float) -> None:
    ny, nx = h.shape
    x_max = nx * res
    y_max = ny * res
    x_mid = 0.5 * x_max
    y_mid = 0.5 * y_max
    z_min = float(np.min(h) * z_scale)
    z_max = float(np.max(h) * z_scale)
    span_xy = max(x_max, y_max)
    cam_pos = (x_mid - 0.9 * span_xy, y_mid - 0.8 * span_xy, z_max + 0.9 * span_xy)
    focal = (x_mid, y_mid, 0.5 * (z_min + z_max))
    viewup = (0.0, 0.0, 1.0)
    plotter.camera_position = [cam_pos, focal, viewup]


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    out2d = args.output_dir_2d.resolve()
    out3d = args.output_dir_3d.resolve()
    out2d.mkdir(parents=True, exist_ok=True)
    out3d.mkdir(parents=True, exist_ok=True)

    metrics_csv = run_dir / "planning_metrics.csv"
    paths_jsonl = run_dir / "planning_paths.jsonl"
    scenes_dir = run_dir / "scenes"
    if not metrics_csv.exists() or not paths_jsonl.exists() or not scenes_dir.exists():
        raise FileNotFoundError("run-dir must contain planning_metrics.csv, planning_paths.jsonl, scenes/")

    rows = _load_metrics(metrics_csv)
    paths = _load_paths(paths_jsonl)
    rows = [
        r
        for r in rows
        if str(r["method"]) == "proposed_ml_risk_constrained_astar" and str(r["model_tag"]) == str(args.model_tag)
    ]

    import matplotlib.pyplot as plt
    import pyvista as pv

    try:
        if hasattr(pv, "start_xvfb"):
            pv.start_xvfb()
    except Exception:
        pass

    scene_cache: dict[str, Any] = {}
    count_2d = 0
    count_3d = 0
    for row in rows:
        run_id = str(row["run_id"])
        p = paths.get(run_id)
        if p is None:
            continue
        scene_id = str(row["scene_id"])
        if scene_id not in scene_cache:
            scene_cache[scene_id] = np.load(scenes_dir / f"{scene_id}.npz")
        scene = scene_cache[scene_id]
        h = scene["heightmap"].astype(np.float32)
        res = float(scene["resolution_m"])
        start = scene["start_state"].astype(int)
        goal = scene["goal_state"].astype(int)
        vehicle_id = str(row["vehicle_id"])
        basename = f"{scene_id}__{vehicle_id}__proposed_only"

        # 2D
        fig, ax = plt.subplots(figsize=(7.5, 6.0))
        extent = [0.0, h.shape[1] * res, 0.0, h.shape[0] * res]
        ax.imshow(h, origin="lower", cmap="terrain", extent=extent)
        found = bool(p.get("found", False))
        if found:
            x, y = _state_to_xy(p["states"], res)
            ax.plot(x, y, color="#d62728", lw=2.4, label="proposed")
        else:
            ax.text(
                0.02 * h.shape[1] * res,
                0.95 * h.shape[0] * res,
                "NO PATH (constraints too strict)",
                color="red",
                fontsize=10,
                bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": "red", "alpha": 0.9},
            )
        ax.scatter([(start[1] + 0.5) * res], [(start[0] + 0.5) * res], c="lime", s=80, marker="o", label="start")
        ax.scatter([(goal[1] + 0.5) * res], [(goal[0] + 0.5) * res], c="red", s=100, marker="*", label="goal")
        ax.set_title(f"{scene_id} | {vehicle_id} | proposed only | found={int(found)}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(out2d / f"{basename}.png", dpi=180)
        fig.savefig(out2d / f"{basename}.svg")
        plt.close(fig)
        count_2d += 1

        # 3D
        plotter = pv.Plotter(off_screen=True, window_size=(1920, 1200))
        grid = _terrain_grid_3d(pv, h, res, float(args.z_scale))
        plotter.add_mesh(
            grid,
            scalars="height",
            cmap="terrain",
            smooth_shading=True,
            ambient=0.35,
            diffuse=0.7,
            specular=0.1,
            scalar_bar_args={"title": "height (scaled)", "vertical": True},
        )
        pts = _state_to_xyz(p["states"], h, res, float(args.z_scale), float(args.path_lift_m))
        if found and pts.shape[0] >= 2:
            poly = pv.lines_from_points(pts, close=False)
            plotter.add_mesh(poly, color="#d62728", line_width=7)
        start_xyz = _state_to_xyz([[int(start[0]), int(start[1]), int(start[2])]], h, res, float(args.z_scale), float(args.path_lift_m))[0]
        goal_xyz = _state_to_xyz([[int(goal[0]), int(goal[1]), int(goal[2])]], h, res, float(args.z_scale), float(args.path_lift_m))[0]
        marker_radius = 0.12 * res
        plotter.add_mesh(pv.Sphere(radius=marker_radius, center=start_xyz), color="lime")
        plotter.add_mesh(pv.Sphere(radius=marker_radius * 1.2, center=goal_xyz), color="red")
        legend = [("start", "lime"), ("goal", "red")]
        if found:
            legend = [("proposed", "#d62728")] + legend
        else:
            plotter.add_text("NO PATH (constraints too strict)", position="upper_left", font_size=12, color="red")
        plotter.add_legend(legend, bcolor=(0.06, 0.06, 0.06), border=True, size=(0.30, 0.2))
        plotter.add_text(f"{scene_id} | {vehicle_id} | proposed only | found={int(found)}", font_size=12)
        _setup_camera_3d(plotter, h, res, float(args.z_scale))
        plotter.screenshot(str(out3d / f"{basename}_3d.png"), transparent_background=False)
        plotter.close()
        count_3d += 1

    summary = {
        "run_dir": str(run_dir),
        "output_dir_2d": str(out2d),
        "output_dir_3d": str(out3d),
        "model_tag": str(args.model_tag),
        "count_2d": int(count_2d),
        "count_3d": int(count_3d),
    }
    (out2d / "proposed_only_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (out3d / "proposed_only_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
