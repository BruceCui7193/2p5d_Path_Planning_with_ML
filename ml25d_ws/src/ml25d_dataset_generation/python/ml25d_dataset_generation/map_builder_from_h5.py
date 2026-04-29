from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .planning_types import PlanningScene
from .training_data import RiskDatasetArrays


@dataclass(frozen=True)
class PatchRecord:
    rel_height_patch: np.ndarray
    heading_rad: float
    terrain_class: str
    friction_mu: float


class H5PlanningMapBuilder:
    def __init__(
        self,
        data: RiskDatasetArrays,
        patch_size: int,
        resolution_m: float,
        heading_bins: int,
        seed: int = 0,
    ) -> None:
        if data.x_map.ndim != 4 or data.x_map.shape[-1] < 1:
            raise ValueError(f"x_map must have shape (N,H,W,C>=1), got {data.x_map.shape}")
        self.patch_size = int(patch_size)
        self.resolution_m = float(resolution_m)
        self.heading_bins = int(heading_bins)
        self.rng = np.random.default_rng(int(seed))

        self._records: list[PatchRecord] = []
        for idx, meta in enumerate(data.metadata):
            terrain_class = str(meta.get("terrain_class", "unknown"))
            heading = float(meta.get("heading_rad", 0.0))
            friction_mu = float(meta.get("friction_mu", float(data.mu[idx, 0])))
            rel_patch = np.asarray(data.x_map[idx, :, :, 0], dtype=np.float32)
            global_patch = self._rotate_patch(rel_patch, -heading)
            self._records.append(
                PatchRecord(
                    rel_height_patch=global_patch.astype(np.float32),
                    heading_rad=heading,
                    terrain_class=terrain_class,
                    friction_mu=friction_mu,
                )
            )
        self._by_terrain: dict[str, list[int]] = {}
        for i, rec in enumerate(self._records):
            self._by_terrain.setdefault(rec.terrain_class, []).append(i)

    @staticmethod
    def _bilinear(src: np.ndarray, x: float, y: float) -> float:
        x = float(np.clip(x, 0.0, src.shape[0] - 1.001))
        y = float(np.clip(y, 0.0, src.shape[1] - 1.001))
        x0 = int(math.floor(x))
        y0 = int(math.floor(y))
        x1 = min(x0 + 1, src.shape[0] - 1)
        y1 = min(y0 + 1, src.shape[1] - 1)
        wx = x - x0
        wy = y - y0
        v00 = float(src[x0, y0])
        v10 = float(src[x1, y0])
        v01 = float(src[x0, y1])
        v11 = float(src[x1, y1])
        return float((1.0 - wx) * (1.0 - wy) * v00 + wx * (1.0 - wy) * v10 + (1.0 - wx) * wy * v01 + wx * wy * v11)

    def _rotate_patch(self, src: np.ndarray, heading_rad: float) -> np.ndarray:
        n = src.shape[0]
        out = np.zeros_like(src, dtype=np.float32)
        center = (n - 1) / 2.0
        c = math.cos(heading_rad)
        s = math.sin(heading_rad)
        for i in range(n):
            for j in range(n):
                u = i - center
                v = j - center
                x = center + (c * u - s * v)
                y = center + (s * u + c * v)
                out[i, j] = self._bilinear(src, x, y)
        return out

    def available_terrain_classes(self) -> list[str]:
        return sorted(self._by_terrain.keys())

    def build_scene(
        self,
        terrain_class: str,
        scene_index: int,
        global_size: int = 121,
        overlap_cells: int = 15,
        start_margin_cells: int = 10,
        goal_margin_cells: int = 10,
    ) -> PlanningScene:
        pool = self._by_terrain.get(str(terrain_class), [])
        if not pool:
            pool = list(range(len(self._records)))
        if not pool:
            raise RuntimeError("no patches available for planning scene construction")

        step = self.patch_size - int(overlap_cells)
        if step <= 0:
            raise ValueError(f"invalid overlap_cells={overlap_cells}, step must be positive")

        n_tiles = int(math.ceil((int(global_size) - self.patch_size) / max(step, 1))) + 1
        stitched_size = self.patch_size + (n_tiles - 1) * step
        weight_1d = np.hanning(self.patch_size).astype(np.float32)
        if np.max(weight_1d) <= 1e-6:
            weight_1d = np.ones(self.patch_size, dtype=np.float32)
        # Keep non-zero edge weights to avoid seams/holes at tile boundaries.
        weight_1d = 0.15 + 0.85 * (weight_1d / np.max(weight_1d))
        weight = np.outer(weight_1d, weight_1d).astype(np.float32)

        sum_map = np.zeros((stitched_size, stitched_size), dtype=np.float64)
        sum_weight = np.zeros((stitched_size, stitched_size), dtype=np.float64)
        used_friction: list[float] = []

        for ti in range(n_tiles):
            for tj in range(n_tiles):
                rec = self._records[int(self.rng.choice(pool))]
                patch = np.asarray(rec.rel_height_patch, dtype=np.float64)
                x0 = ti * step
                y0 = tj * step
                x1 = x0 + self.patch_size
                y1 = y0 + self.patch_size

                existing_w = sum_weight[x0:x1, y0:y1]
                overlap = existing_w > 1e-9
                if np.any(overlap):
                    existing_h = sum_map[x0:x1, y0:y1] / np.maximum(existing_w, 1e-9)
                    bias = float(np.mean(existing_h[overlap] - patch[overlap]))
                else:
                    bias = 0.0
                patch_adj = patch + bias

                sum_map[x0:x1, y0:y1] += patch_adj * weight
                sum_weight[x0:x1, y0:y1] += weight
                used_friction.append(float(rec.friction_mu))

        stitched = (sum_map / np.maximum(sum_weight, 1e-9)).astype(np.float32)
        if stitched_size != global_size:
            start = (stitched_size - int(global_size)) // 2
            end = start + int(global_size)
            stitched = stitched[start:end, start:end]
        stitched = stitched - float(stitched[int(global_size) // 2, int(global_size) // 2])

        start_i = int(max(start_margin_cells, 0))
        goal_i = int(min(global_size - 1 - max(goal_margin_cells, 0), global_size - 1))
        mid_j = int(global_size // 2)
        k0 = 0
        scene_id = f"{terrain_class}_scene_{scene_index:03d}"
        friction_mu = float(np.median(np.asarray(used_friction, dtype=np.float32))) if used_friction else 0.8
        return PlanningScene(
            scene_id=scene_id,
            terrain_class=str(terrain_class),
            heightmap=stitched.astype(np.float32),
            resolution_m=self.resolution_m,
            friction_mu=friction_mu,
            start_state=(start_i, mid_j, k0),
            goal_state=(goal_i, mid_j, k0),
            heading_bins=self.heading_bins,
        )

    @staticmethod
    def save_scene_npz(scene: PlanningScene, output_path: str) -> None:
        np.savez_compressed(
            output_path,
            scene_id=np.asarray(scene.scene_id),
            terrain_class=np.asarray(scene.terrain_class),
            heightmap=scene.heightmap.astype(np.float32),
            resolution_m=np.asarray(scene.resolution_m, dtype=np.float32),
            friction_mu=np.asarray(scene.friction_mu, dtype=np.float32),
            start_state=np.asarray(scene.start_state, dtype=np.int32),
            goal_state=np.asarray(scene.goal_state, dtype=np.int32),
            heading_bins=np.asarray(scene.heading_bins, dtype=np.int32),
        )
