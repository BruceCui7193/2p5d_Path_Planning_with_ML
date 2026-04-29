from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class TerrainSample:
    heightmap: np.ndarray
    parameters: Dict[str, float]


class TerrainGenerator:
    def __init__(self, patch_size: int = 31, resolution_m: float = 0.1) -> None:
        self.patch_size = patch_size
        self.resolution_m = resolution_m
        half_extent = (patch_size - 1) * resolution_m / 2.0
        self.half_extent_m = half_extent
        axis = np.linspace(-half_extent, half_extent, patch_size)
        self._xx, self._yy = np.meshgrid(axis, axis, indexing="ij")

    def generate(
        self,
        rng: np.random.Generator,
        terrain_class_cfg: Dict[str, Any],
        travel_heading_rad: float | None = None,
    ) -> TerrainSample:
        params = self._sample_parameters(rng, terrain_class_cfg)
        height = np.zeros((self.patch_size, self.patch_size), dtype=np.float64)
        spawn_keepout = float(params["spawn_keepout_radius_m"])

        slope_dir = float(rng.uniform(0.0, 2.0 * np.pi))
        cross_dir = slope_dir + np.pi / 2.0

        height += self._plane(np.deg2rad(params["slope_deg"]), slope_dir)
        height += self._plane(np.deg2rad(params["cross_deg"]), cross_dir)
        height += self._wave(params["wave_amp_m"], params["wave_len_m"], rng)
        height += self._bumps(
            rng,
            int(params["bump_count"]),
            float(params["bump_radius_min_m"]),
            float(params["bump_radius_max_m"]),
            float(params["bump_height_max_m"]),
            spawn_keepout,
            travel_heading_rad=travel_heading_rad,
        )
        height -= self._bumps(
            rng,
            int(params["pit_count"]),
            float(params["pit_radius_min_m"]),
            float(params["pit_radius_max_m"]),
            float(params["pit_depth_max_m"]),
            spawn_keepout,
            travel_heading_rad=travel_heading_rad,
        )
        height += self._step(
            rng,
            params["step_height_m"],
            params["step_width_m"],
            spawn_keepout,
            travel_heading_rad=travel_heading_rad,
        )
        height += self._noise(rng, params["noise_std_m"])
        height = self._box_smooth(height, kernel_size=3)
        # Keep spawn neighborhood physically realizable: suppress high-frequency
        # roughness near origin while preserving global terrain difficulty.
        height = self._stabilize_spawn_zone(
            height,
            core_radius_m=float(params["spawn_stabilize_core_radius_m"]),
            blend_radius_m=float(params["spawn_stabilize_blend_radius_m"]),
            kernel_size=5,
        )

        return TerrainSample(heightmap=height.astype(np.float32), parameters=params)

    def _sample_parameters(self, rng: np.random.Generator, terrain_class_cfg: Dict[str, Any]) -> Dict[str, float]:
        params_cfg = terrain_class_cfg["params"]

        slope_deg = self._sample_range(rng, params_cfg["slope_deg"])
        cross_deg = self._sample_range(rng, params_cfg["cross_deg"])
        wave_amp_m = self._sample_range(rng, params_cfg["wave_amp_m"])
        wave_len_m = self._sample_range(rng, params_cfg["wave_len_m"])
        bump_count = int(round(self._sample_range(rng, params_cfg["bump_count"])))
        pit_count = int(round(self._sample_range(rng, params_cfg["pit_count"])))
        step_height_m = self._sample_range(rng, params_cfg["step_height_m"])
        noise_std_m = self._sample_range(rng, params_cfg["noise_std_m"])

        return {
            "slope_deg": slope_deg,
            "cross_deg": cross_deg,
            "wave_amp_m": wave_amp_m,
            "wave_len_m": wave_len_m,
            "bump_count": float(bump_count),
            "pit_count": float(pit_count),
            "step_height_m": step_height_m,
            "noise_std_m": noise_std_m,
            "bump_radius_min_m": 0.05,
            "bump_radius_max_m": 0.30,
            "pit_radius_min_m": 0.05,
            "pit_radius_max_m": 0.30,
            "bump_height_max_m": 0.15,
            "pit_depth_max_m": 0.20,
            "step_width_m": float(rng.uniform(0.10, 0.50)),
            "spawn_keepout_radius_m": float(terrain_class_cfg.get("spawn_keepout_radius_m", 0.22)),
            # Defaults are sized to cover the full initial wheel footprint of
            # all configured vehicles; smaller radii can leave one axle outside
            # the stabilized zone and trigger invalid spawn geometry.
            "spawn_stabilize_core_radius_m": float(terrain_class_cfg.get("spawn_stabilize_core_radius_m", 0.30)),
            "spawn_stabilize_blend_radius_m": float(terrain_class_cfg.get("spawn_stabilize_blend_radius_m", 0.55)),
        }

    @staticmethod
    def _sample_range(rng: np.random.Generator, interval: Tuple[float, float]) -> float:
        lo, hi = float(interval[0]), float(interval[1])
        if hi < lo:
            raise ValueError(f"invalid interval: {interval}")
        return float(rng.uniform(lo, hi))

    def _plane(self, angle_rad: float, direction_rad: float) -> np.ndarray:
        grad = np.tan(angle_rad)
        proj = self._xx * np.cos(direction_rad) + self._yy * np.sin(direction_rad)
        return grad * proj

    def _wave(self, amplitude_m: float, wavelength_m: float, rng: np.random.Generator) -> np.ndarray:
        if amplitude_m <= 1e-9:
            return np.zeros_like(self._xx)
        phase_u = float(rng.uniform(0.0, 2.0 * np.pi))
        phase_v = float(rng.uniform(0.0, 2.0 * np.pi))
        k = 2.0 * np.pi / max(wavelength_m, 1e-3)
        return amplitude_m * np.sin(k * self._xx + phase_u) * np.cos(k * self._yy + phase_v)

    def _bumps(
        self,
        rng: np.random.Generator,
        count: int,
        radius_min: float,
        radius_max: float,
        height_max: float,
        spawn_keepout_radius_m: float,
        travel_heading_rad: float | None = None,
    ) -> np.ndarray:
        if count <= 0 or height_max <= 1e-9:
            return np.zeros_like(self._xx)

        out = np.zeros_like(self._xx)
        margin = float(self.half_extent_m)
        keepout = max(float(spawn_keepout_radius_m), 0.0)
        for _ in range(count):
            radius = float(rng.uniform(radius_min, radius_max))
            cx, cy = self._sample_feature_center(
                rng=rng,
                margin=margin,
                keepout_radius=max(keepout + radius, radius),
                forward_heading_rad=travel_heading_rad,
            )
            amplitude = float(rng.uniform(0.0, height_max))
            dist2 = (self._xx - cx) ** 2 + (self._yy - cy) ** 2
            out += amplitude * np.exp(-dist2 / (2.0 * max(radius * radius, 1e-6)))
        return out

    def _step(
        self,
        rng: np.random.Generator,
        step_height_m: float,
        step_width_m: float,
        spawn_keepout_radius_m: float,
        travel_heading_rad: float | None = None,
    ) -> np.ndarray:
        if step_height_m <= 1e-9:
            return np.zeros_like(self._xx)
        max_pos = max(float(self.half_extent_m) - 0.5 * float(step_width_m), 0.0)
        min_abs_pos = 0.5 * float(step_width_m) + max(float(spawn_keepout_radius_m), 0.0)
        if max_pos <= 1e-6:
            return np.zeros_like(self._xx)
        if travel_heading_rad is not None and float(rng.random()) < 0.70:
            # Bias step to intersect the intended forward path within one action horizon.
            direction = float(travel_heading_rad + rng.uniform(-0.2, 0.2))
            pos_lo = max(min_abs_pos, max(float(spawn_keepout_radius_m), 0.0) + 0.05)
            pos_hi = min(max_pos, 0.40)
            if pos_hi <= pos_lo:
                position = float(np.clip(pos_lo, -max_pos, max_pos))
            else:
                position = float(rng.uniform(pos_lo, pos_hi))
        else:
            direction = float(rng.uniform(0.0, 2.0 * np.pi))
            if min_abs_pos >= max_pos:
                position = float(np.sign(rng.uniform(-1.0, 1.0)) * max_pos)
            else:
                for _ in range(24):
                    cand = float(rng.uniform(-max_pos, max_pos))
                    if abs(cand) >= min_abs_pos:
                        position = cand
                        break
                else:
                    position = float(np.sign(rng.uniform(-1.0, 1.0)) * min_abs_pos)
        projection = self._xx * np.cos(direction) + self._yy * np.sin(direction)
        return np.where(
            (projection >= position - step_width_m / 2.0) & (projection <= position + step_width_m / 2.0),
            step_height_m,
            0.0,
        )

    @staticmethod
    def _sample_feature_center(
        *,
        rng: np.random.Generator,
        margin: float,
        keepout_radius: float,
        forward_heading_rad: float | None = None,
    ) -> tuple[float, float]:
        # In most samples place pits/bumps directly on the likely traversed corridor.
        if forward_heading_rad is not None and float(rng.random()) < 0.70:
            d_min = max(float(keepout_radius) + 0.05, 0.10)
            d_max = min(float(margin), 0.45)
            if d_max > d_min:
                d = float(rng.uniform(d_min, d_max))
                lateral = float(rng.uniform(-0.20, 0.20))
                c, s = float(np.cos(forward_heading_rad)), float(np.sin(forward_heading_rad))
                cx = d * c - lateral * s
                cy = d * s + lateral * c
                if abs(cx) <= margin and abs(cy) <= margin and float(np.hypot(cx, cy)) >= keepout_radius:
                    return cx, cy
        for _ in range(32):
            cx = float(rng.uniform(-margin, margin))
            cy = float(rng.uniform(-margin, margin))
            if float(np.hypot(cx, cy)) >= keepout_radius:
                return cx, cy
        # Fallback: keep generation moving even for degenerate geometry.
        return float(rng.uniform(-margin, margin)), float(rng.uniform(-margin, margin))

    def _noise(self, rng: np.random.Generator, std_m: float) -> np.ndarray:
        if std_m <= 1e-9:
            return np.zeros_like(self._xx)
        return rng.normal(0.0, std_m, size=self._xx.shape)

    @staticmethod
    def _box_smooth(values: np.ndarray, kernel_size: int = 3) -> np.ndarray:
        if kernel_size <= 1:
            return values
        pad = kernel_size // 2
        padded = np.pad(values, ((pad, pad), (pad, pad)), mode="edge")
        out = np.zeros_like(values)
        denom = float(kernel_size * kernel_size)
        for i in range(values.shape[0]):
            for j in range(values.shape[1]):
                window = padded[i : i + kernel_size, j : j + kernel_size]
                out[i, j] = float(np.sum(window) / denom)
        return out

    def _stabilize_spawn_zone(
        self,
        height: np.ndarray,
        *,
        core_radius_m: float,
        blend_radius_m: float,
        kernel_size: int = 5,
    ) -> np.ndarray:
        core = max(float(core_radius_m), 0.0)
        blend = max(float(blend_radius_m), core + 1e-6)
        smooth = self._box_smooth(height, kernel_size=max(int(kernel_size), 3))
        rr = np.hypot(self._xx, self._yy)
        # Full smoothing in core zone, then cosine falloff to original terrain.
        w = np.zeros_like(height, dtype=np.float64)
        w[rr <= core] = 1.0
        ring = (rr > core) & (rr < blend)
        if np.any(ring):
            alpha = (rr[ring] - core) / max(blend - core, 1e-6)
            w[ring] = 0.5 * (1.0 + np.cos(np.pi * alpha))
        return (w * smooth + (1.0 - w) * height).astype(np.float64)
