#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render planning benchmark figures (2D or PyVista 3D)")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--model-tag", type=str, default="main", help="model tag for ML methods")
    parser.add_argument("--max-scenes", type=int, default=0, help="0 means all")
    parser.add_argument(
        "--renderer",
        type=str,
        default="matplotlib",
        choices=["matplotlib", "pyvista"],
        help="Rendering backend: matplotlib(2D) or pyvista(3D).",
    )
    parser.add_argument("--z-scale", type=float, default=4.0, help="Vertical exaggeration for 3D rendering")
    parser.add_argument("--path-lift-m", type=float, default=0.03, help="Lift path over terrain in meters (before z-scale)")
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
                if key in {"found", "expanded_nodes", "num_states", "num_actions", "start_i", "start_j", "start_k", "goal_i", "goal_j", "goal_k", "goal_radius_cells"}:
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


def _plot_vehicle_compare(
    plt,
    scene_npz: Path,
    rows: list[dict[str, Any]],
    paths: dict[str, dict[str, Any]],
    out_path: Path,
) -> None:
    scene = np.load(scene_npz)
    h = scene["heightmap"].astype(np.float32)
    res = float(scene["resolution_m"])
    start = scene["start_state"].astype(int)
    goal = scene["goal_state"].astype(int)

    fig, ax = plt.subplots(figsize=(7.5, 6.0))
    extent = [0.0, h.shape[1] * res, 0.0, h.shape[0] * res]
    ax.imshow(h, origin="lower", cmap="terrain", extent=extent)
    colors = {"urban_small": "#1f77b4", "standard_offroad": "#2ca02c", "mountain_large": "#d62728"}
    for row in rows:
        run_id = str(row["run_id"])
        p = paths.get(run_id)
        if p is None or not p.get("found", False):
            continue
        x, y = _state_to_xy(p["states"], res)
        label = f"{row['vehicle_id']} (L={row['path_length_m']:.1f}, Ravg={row['risk_avg']:.2f})"
        ax.plot(x, y, lw=2.2, color=colors.get(str(row["vehicle_id"]), None), label=label)
    ax.scatter([(start[1] + 0.5) * res], [(start[0] + 0.5) * res], c="lime", s=80, marker="o", label="start")
    ax.scatter([(goal[1] + 0.5) * res], [(goal[0] + 0.5) * res], c="red", s=100, marker="*", label="goal")
    ax.set_title(f"{scene['scene_id'].item()} | Proposed | vehicle compare")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path.with_suffix(".png"), dpi=180)
    fig.savefig(out_path.with_suffix(".svg"))
    plt.close(fig)


def _plot_method_compare(
    plt,
    scene_npz: Path,
    rows: list[dict[str, Any]],
    paths: dict[str, dict[str, Any]],
    out_path: Path,
) -> None:
    scene = np.load(scene_npz)
    h = scene["heightmap"].astype(np.float32)
    res = float(scene["resolution_m"])
    start = scene["start_state"].astype(int)
    goal = scene["goal_state"].astype(int)

    fig, ax = plt.subplots(figsize=(7.5, 6.0))
    extent = [0.0, h.shape[1] * res, 0.0, h.shape[0] * res]
    ax.imshow(h, origin="lower", cmap="terrain", extent=extent)
    colors = {
        "baseline1_2p5d_astar": "#1f77b4",
        "baseline2_manual_risk_weighted_astar": "#ff7f0e",
        "baseline3_ml_risk_weighted_astar": "#9467bd",
        "proposed_ml_risk_constrained_astar": "#d62728",
        "guard_only_constrained_astar": "#2ca02c",
    }
    for row in rows:
        run_id = str(row["run_id"])
        p = paths.get(run_id)
        if p is None or not p.get("found", False):
            continue
        x, y = _state_to_xy(p["states"], res)
        label = f"{row['method']} (L={row['path_length_m']:.1f}, Ravg={row['risk_avg']:.2f})"
        ax.plot(x, y, lw=2.2, color=colors.get(str(row["method"]), None), label=label)
    ax.scatter([(start[1] + 0.5) * res], [(start[0] + 0.5) * res], c="lime", s=80, marker="o", label="start")
    ax.scatter([(goal[1] + 0.5) * res], [(goal[0] + 0.5) * res], c="red", s=100, marker="*", label="goal")
    vehicle = rows[0]["vehicle_id"] if rows else "unknown_vehicle"
    ax.set_title(f"{scene['scene_id'].item()} | {vehicle} | method compare")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best", fontsize=7)
    fig.tight_layout()
    fig.savefig(out_path.with_suffix(".png"), dpi=180)
    fig.savefig(out_path.with_suffix(".svg"))
    plt.close(fig)


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


def _plot_vehicle_compare_3d(
    pv,
    scene_npz: Path,
    rows: list[dict[str, Any]],
    paths: dict[str, dict[str, Any]],
    out_png_path: Path,
    z_scale: float,
    path_lift_m: float,
) -> None:
    scene = np.load(scene_npz)
    h = scene["heightmap"].astype(np.float32)
    res = float(scene["resolution_m"])
    start = scene["start_state"].astype(int)
    goal = scene["goal_state"].astype(int)

    plotter = pv.Plotter(off_screen=True, window_size=(1920, 1200))
    grid = _terrain_grid_3d(pv, h, res, z_scale)
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

    colors = {"urban_small": "#1f77b4", "standard_offroad": "#2ca02c", "mountain_large": "#d62728"}
    legend_entries: list[tuple[str, str]] = []
    for row in rows:
        run_id = str(row["run_id"])
        p = paths.get(run_id)
        if p is None or not p.get("found", False):
            continue
        pts = _state_to_xyz(p["states"], h, res, z_scale, path_lift_m)
        if pts.shape[0] < 2:
            continue
        color = colors.get(str(row["vehicle_id"]), "white")
        poly = pv.lines_from_points(pts, close=False)
        label = f"{row['vehicle_id']} (L={row['path_length_m']:.1f}, Ravg={row['risk_avg']:.2f})"
        plotter.add_mesh(poly, color=color, line_width=6)
        legend_entries.append((label, color))

    start_xyz = _state_to_xyz([[int(start[0]), int(start[1]), int(start[2])]], h, res, z_scale, path_lift_m)[0]
    goal_xyz = _state_to_xyz([[int(goal[0]), int(goal[1]), int(goal[2])]], h, res, z_scale, path_lift_m)[0]
    marker_radius = 0.12 * res
    plotter.add_mesh(pv.Sphere(radius=marker_radius, center=start_xyz), color="lime")
    plotter.add_mesh(pv.Sphere(radius=marker_radius * 1.2, center=goal_xyz), color="red")
    legend_entries.extend([("start", "lime"), ("goal", "red")])
    if legend_entries:
        plotter.add_legend(legend_entries, bcolor=(0.06, 0.06, 0.06), border=True, size=(0.34, 0.25))

    plotter.add_text(f"{scene['scene_id'].item()} | Proposed | vehicle compare", font_size=12)
    _setup_camera_3d(plotter, h, res, z_scale)
    plotter.screenshot(str(out_png_path), transparent_background=False)
    plotter.close()


def _plot_method_compare_3d(
    pv,
    scene_npz: Path,
    rows: list[dict[str, Any]],
    paths: dict[str, dict[str, Any]],
    out_png_path: Path,
    z_scale: float,
    path_lift_m: float,
) -> None:
    scene = np.load(scene_npz)
    h = scene["heightmap"].astype(np.float32)
    res = float(scene["resolution_m"])
    start = scene["start_state"].astype(int)
    goal = scene["goal_state"].astype(int)

    plotter = pv.Plotter(off_screen=True, window_size=(1920, 1200))
    grid = _terrain_grid_3d(pv, h, res, z_scale)
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

    colors = {
        "baseline1_2p5d_astar": "#1f77b4",
        "baseline2_manual_risk_weighted_astar": "#ff7f0e",
        "baseline3_ml_risk_weighted_astar": "#9467bd",
        "proposed_ml_risk_constrained_astar": "#d62728",
        "guard_only_constrained_astar": "#2ca02c",
    }
    legend_entries: list[tuple[str, str]] = []
    for row in rows:
        run_id = str(row["run_id"])
        p = paths.get(run_id)
        if p is None or not p.get("found", False):
            continue
        pts = _state_to_xyz(p["states"], h, res, z_scale, path_lift_m)
        if pts.shape[0] < 2:
            continue
        color = colors.get(str(row["method"]), "white")
        poly = pv.lines_from_points(pts, close=False)
        label = f"{row['method']} (L={row['path_length_m']:.1f}, Ravg={row['risk_avg']:.2f})"
        plotter.add_mesh(poly, color=color, line_width=6)
        legend_entries.append((label, color))

    start_xyz = _state_to_xyz([[int(start[0]), int(start[1]), int(start[2])]], h, res, z_scale, path_lift_m)[0]
    goal_xyz = _state_to_xyz([[int(goal[0]), int(goal[1]), int(goal[2])]], h, res, z_scale, path_lift_m)[0]
    marker_radius = 0.12 * res
    plotter.add_mesh(pv.Sphere(radius=marker_radius, center=start_xyz), color="lime")
    plotter.add_mesh(pv.Sphere(radius=marker_radius * 1.2, center=goal_xyz), color="red")
    legend_entries.extend([("start", "lime"), ("goal", "red")])
    if legend_entries:
        plotter.add_legend(legend_entries, bcolor=(0.06, 0.06, 0.06), border=True, size=(0.42, 0.28))

    vehicle = rows[0]["vehicle_id"] if rows else "unknown_vehicle"
    plotter.add_text(f"{scene['scene_id'].item()} | {vehicle} | method compare", font_size=12)
    _setup_camera_3d(plotter, h, res, z_scale)
    plotter.screenshot(str(out_png_path), transparent_background=False)
    plotter.close()


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    out_dir = (args.output_dir or (run_dir / "figures")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    metrics_csv = run_dir / "planning_metrics.csv"
    paths_jsonl = run_dir / "planning_paths.jsonl"
    scenes_dir = run_dir / "scenes"
    if not metrics_csv.exists() or not paths_jsonl.exists() or not scenes_dir.exists():
        raise FileNotFoundError("run-dir must contain planning_metrics.csv, planning_paths.jsonl, scenes/")

    rows = _load_metrics(metrics_csv)
    paths = _load_paths(paths_jsonl)
    scene_ids = sorted({str(r["scene_id"]) for r in rows})
    if int(args.max_scenes) > 0:
        scene_ids = scene_ids[: int(args.max_scenes)]

    plt = None
    pv = None
    if args.renderer == "matplotlib":
        try:
            import matplotlib.pyplot as plt  # type: ignore[no-redef]
        except Exception as exc:
            raise RuntimeError("matplotlib is required for renderer=matplotlib") from exc
    else:
        try:
            import pyvista as pv  # type: ignore[no-redef]
        except Exception as exc:
            raise RuntimeError("pyvista is required for renderer=pyvista") from exc
        try:
            # Helps in headless environments without an active X display.
            if hasattr(pv, "start_xvfb"):
                pv.start_xvfb()
        except Exception:
            pass

    # Figure 1: same scene + proposed + three vehicles.
    vehicle_figs = 0
    for scene_id in scene_ids:
        scene_rows = [
            r
            for r in rows
            if str(r["scene_id"]) == scene_id
            and str(r["method"]) == "proposed_ml_risk_constrained_astar"
            and str(r["model_tag"]) == str(args.model_tag)
        ]
        if not scene_rows:
            continue
        scene_npz = scenes_dir / f"{scene_id}.npz"
        if not scene_npz.exists():
            continue
        if args.renderer == "matplotlib":
            out_path = out_dir / f"{scene_id}__vehicles_proposed"
            _plot_vehicle_compare(plt, scene_npz, scene_rows, paths, out_path)
        else:
            out_path = out_dir / f"{scene_id}__vehicles_proposed_3d.png"
            _plot_vehicle_compare_3d(
                pv,
                scene_npz,
                scene_rows,
                paths,
                out_path,
                z_scale=float(args.z_scale),
                path_lift_m=float(args.path_lift_m),
            )
        vehicle_figs += 1

    # Figure 2: same scene + same vehicle + four methods.
    method_figs = 0
    keys = sorted({(str(r["scene_id"]), str(r["vehicle_id"])) for r in rows if str(r["scene_id"]) in set(scene_ids)})
    for scene_id, vehicle_id in keys:
        group = []
        for method in [
            "baseline1_2p5d_astar",
            "baseline2_manual_risk_weighted_astar",
            "baseline3_ml_risk_weighted_astar",
            "proposed_ml_risk_constrained_astar",
            "guard_only_constrained_astar",
        ]:
            if method in {"baseline1_2p5d_astar", "baseline2_manual_risk_weighted_astar", "guard_only_constrained_astar"}:
                cand = [
                    r
                    for r in rows
                    if str(r["scene_id"]) == scene_id
                    and str(r["vehicle_id"]) == vehicle_id
                    and str(r["method"]) == method
                    and str(r["model_tag"]) == "none"
                ]
            else:
                cand = [
                    r
                    for r in rows
                    if str(r["scene_id"]) == scene_id
                    and str(r["vehicle_id"]) == vehicle_id
                    and str(r["method"]) == method
                    and str(r["model_tag"]) == str(args.model_tag)
                ]
            if cand:
                group.append(cand[0])
        if len(group) < 2:
            continue
        scene_npz = scenes_dir / f"{scene_id}.npz"
        if not scene_npz.exists():
            continue
        if args.renderer == "matplotlib":
            out_path = out_dir / f"{scene_id}__{vehicle_id}__methods"
            _plot_method_compare(plt, scene_npz, group, paths, out_path)
        else:
            out_path = out_dir / f"{scene_id}__{vehicle_id}__methods_3d.png"
            _plot_method_compare_3d(
                pv,
                scene_npz,
                group,
                paths,
                out_path,
                z_scale=float(args.z_scale),
                path_lift_m=float(args.path_lift_m),
            )
        method_figs += 1

    summary = {
        "run_dir": str(run_dir),
        "output_dir": str(out_dir),
        "model_tag": str(args.model_tag),
        "renderer": str(args.renderer),
        "z_scale": float(args.z_scale),
        "path_lift_m": float(args.path_lift_m),
        "vehicle_compare_figures": int(vehicle_figs),
        "method_compare_figures": int(method_figs),
    }
    (out_dir / "figures_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
