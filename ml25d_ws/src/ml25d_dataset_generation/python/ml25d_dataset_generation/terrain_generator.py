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
        axis = np.linspace(-half_extent, half_extent, patch_size)
        self._xx, self._yy = np.meshgrid(axis, axis, indexing="ij")

    def generate(self, rng: np.random.Generator, terrain_class_cfg: Dict[str, Any]) -> TerrainSample:
        params = self._sample_parameters(rng, terrain_class_cfg)
        height = np.zeros((self.patch_size, self.patch_size), dtype=np.float64)

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
        )
        height -= self._bumps(
            rng,
            int(params["pit_count"]),
            float(params["pit_radius_min_m"]),
            float(params["pit_radius_max_m"]),
            float(params["pit_depth_max_m"]),
        )
        height += self._step(rng, params["step_height_m"], params["step_width_m"])
        height += self._noise(rng, params["noise_std_m"])
        height = self._box_smooth(height, kernel_size=3)

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
    ) -> np.ndarray:
        if count <= 0 or height_max <= 1e-9:
            return np.zeros_like(self._xx)

        out = np.zeros_like(self._xx)
        margin = (self.patch_size - 1) * self.resolution_m / 2.0
        for _ in range(count):
            cx = float(rng.uniform(-margin, margin))
            cy = float(rng.uniform(-margin, margin))
            radius = float(rng.uniform(radius_min, radius_max))
            amplitude = float(rng.uniform(0.0, height_max))
            dist2 = (self._xx - cx) ** 2 + (self._yy - cy) ** 2
            out += amplitude * np.exp(-dist2 / (2.0 * max(radius * radius, 1e-6)))
        return out

    def _step(self, rng: np.random.Generator, step_height_m: float, step_width_m: float) -> np.ndarray:
        if step_height_m <= 1e-9:
            return np.zeros_like(self._xx)
        direction = float(rng.uniform(0.0, 2.0 * np.pi))
        position = float(rng.uniform(-0.6, 0.6))
        projection = self._xx * np.cos(direction) + self._yy * np.sin(direction)
        return np.where(
            (projection >= position - step_width_m / 2.0) & (projection <= position + step_width_m / 2.0),
            step_height_m,
            0.0,
        )

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
