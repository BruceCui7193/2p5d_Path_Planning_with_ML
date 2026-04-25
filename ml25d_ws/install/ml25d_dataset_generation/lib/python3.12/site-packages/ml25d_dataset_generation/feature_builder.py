from __future__ import annotations

from typing import Tuple

import numpy as np

from .common_types import ActionPrimitive, VehicleParams


class FeatureBuilder:
    def __init__(self, patch_size: int = 31, resolution_m: float = 0.1) -> None:
        self.patch_size = patch_size
        self.resolution_m = resolution_m
        half_extent = (patch_size - 1) * resolution_m / 2.0
        axis = np.linspace(-half_extent, half_extent, patch_size)
        self._uu, self._vv = np.meshgrid(axis, axis, indexing="ij")

    def build_feature_patch(
        self,
        heightmap: np.ndarray,
        heading_rad: float,
        vehicle: VehicleParams,
        action: ActionPrimitive,
    ) -> np.ndarray:
        rotated = self._rotate_to_vehicle_frame(heightmap, heading_rad)
        center = self.patch_size // 2
        relative = rotated - rotated[center, center]

        grad_u, grad_v = np.gradient(relative, self.resolution_m, edge_order=1)
        roughness = self._local_std(relative, kernel_size=3)

        body = self._rasterize_rect(cx=0.0, cy=0.0, yaw=0.0, length=vehicle.L, width=vehicle.W)
        swept = self._build_swept_mask(vehicle, action)

        stacked = np.stack(
            [
                relative.astype(np.float32),
                grad_u.astype(np.float32),
                grad_v.astype(np.float32),
                roughness.astype(np.float32),
                body.astype(np.float32),
                swept.astype(np.float32),
            ],
            axis=-1,
        )
        return stacked

    def _rotate_to_vehicle_frame(self, src: np.ndarray, heading_rad: float) -> np.ndarray:
        center = (self.patch_size - 1) / 2.0
        out = np.zeros_like(src, dtype=np.float64)
        c = np.cos(heading_rad)
        s = np.sin(heading_rad)

        for i in range(self.patch_size):
            for j in range(self.patch_size):
                u = i - center
                v = j - center
                x = center + (c * u - s * v)
                y = center + (s * u + c * v)
                out[i, j] = self._bilinear(src, x, y)
        return out.astype(np.float32)

    @staticmethod
    def _bilinear(src: np.ndarray, x: float, y: float) -> float:
        x = np.clip(x, 0.0, src.shape[0] - 1.001)
        y = np.clip(y, 0.0, src.shape[1] - 1.001)
        x0 = int(np.floor(x))
        y0 = int(np.floor(y))
        x1 = min(x0 + 1, src.shape[0] - 1)
        y1 = min(y0 + 1, src.shape[1] - 1)

        wx = x - x0
        wy = y - y0

        v00 = src[x0, y0]
        v10 = src[x1, y0]
        v01 = src[x0, y1]
        v11 = src[x1, y1]

        return float((1.0 - wx) * (1.0 - wy) * v00 + wx * (1.0 - wy) * v10 + (1.0 - wx) * wy * v01 + wx * wy * v11)

    @staticmethod
    def _local_std(values: np.ndarray, kernel_size: int = 3) -> np.ndarray:
        pad = kernel_size // 2
        padded = np.pad(values, ((pad, pad), (pad, pad)), mode="edge")
        out = np.zeros_like(values, dtype=np.float32)
        for i in range(values.shape[0]):
            for j in range(values.shape[1]):
                win = padded[i : i + kernel_size, j : j + kernel_size]
                out[i, j] = float(np.std(win))
        return out

    def _rasterize_rect(self, cx: float, cy: float, yaw: float, length: float, width: float) -> np.ndarray:
        c = np.cos(yaw)
        s = np.sin(yaw)

        du = self._uu - cx
        dv = self._vv - cy

        local_u = c * du + s * dv
        local_v = -s * du + c * dv

        inside = (np.abs(local_u) <= length / 2.0) & (np.abs(local_v) <= width / 2.0)
        return inside.astype(np.uint8)

    def _build_swept_mask(self, vehicle: VehicleParams, action: ActionPrimitive) -> np.ndarray:
        steps = 15
        delta_psi = np.deg2rad(action.delta_psi_deg)
        out = np.zeros((self.patch_size, self.patch_size), dtype=np.uint8)

        for t in np.linspace(0.0, 1.0, steps):
            pose_x, pose_y, pose_yaw = self._pose_on_action(action.delta_s_m, delta_psi, float(t))
            rect = self._rasterize_rect(pose_x, pose_y, pose_yaw, vehicle.L, vehicle.W)
            out = np.maximum(out, rect)

        return out

    @staticmethod
    def _pose_on_action(delta_s: float, delta_psi: float, t: float) -> Tuple[float, float, float]:
        if abs(delta_psi) < 1e-8:
            return delta_s * t, 0.0, 0.0

        if abs(delta_s) < 1e-8:
            yaw = delta_psi * t
            return 0.0, 0.0, yaw

        yaw = delta_psi * t
        radius = delta_s / delta_psi
        x = radius * np.sin(yaw)
        y = radius * (1.0 - np.cos(yaw))
        return float(x), float(y), float(yaw)
