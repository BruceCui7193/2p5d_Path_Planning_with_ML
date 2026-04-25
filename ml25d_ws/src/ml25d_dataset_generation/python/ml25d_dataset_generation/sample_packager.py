from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import h5py
import numpy as np

from .common_types import ActionPrimitive, LabelOutput, SampleMetadata, VEHICLE_PARAM_ORDER, VehicleParams
from .feature_builder import FeatureBuilder


class SamplePackager:
    def __init__(self, map_cfg: Dict, vehicle_cfg: Dict) -> None:
        self.patch_size = int(map_cfg["patch_size"])
        self.resolution_m = float(map_cfg["resolution_m_per_cell"])
        self.feature_builder = FeatureBuilder(self.patch_size, self.resolution_m)
        self.bounds = vehicle_cfg["normalization_bounds"]

    def normalize_vehicle(self, vehicle: VehicleParams) -> np.ndarray:
        vec = vehicle.as_vector()
        out = np.zeros_like(vec, dtype=np.float32)
        for idx, key in enumerate(VEHICLE_PARAM_ORDER):
            lo, hi = self.bounds[key]
            denom = float(max(hi - lo, 1e-6))
            out[idx] = float((vec[idx] - lo) / denom)
        return np.clip(out, 0.0, 1.0)

    @staticmethod
    def encode_action(action: ActionPrimitive) -> np.ndarray:
        encoded = action.encoded_vector().astype(np.float32)
        encoded[0] = encoded[0] / 0.3
        encoded[1] = encoded[1] / (np.pi / 8.0)
        return encoded

    def create_sample(
        self,
        heightmap: np.ndarray,
        heading_rad: float,
        vehicle: VehicleParams,
        action: ActionPrimitive,
        friction_mu: float,
        labels: LabelOutput,
        band: str,
        metadata: SampleMetadata,
    ) -> Dict:
        x_map = self.feature_builder.build_feature_patch(heightmap, heading_rad, vehicle, action)

        return {
            "X_map": x_map.astype(np.float32),
            "theta_v": self.normalize_vehicle(vehicle),
            "a": self.encode_action(action),
            "mu": np.array([friction_mu], dtype=np.float32),
            "y": labels.as_vector(),
            "band": band,
            "metadata": metadata.as_dict(),
        }

    @staticmethod
    def write_hdf5_batch(
        samples: List[Dict],
        output_path: Path,
        compression: str = "gzip",
        compression_level: int = 4,
    ) -> None:
        if not samples:
            raise ValueError("cannot write empty sample batch")

        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        x_map = np.stack([row["X_map"] for row in samples], axis=0)
        theta_v = np.stack([row["theta_v"] for row in samples], axis=0)
        actions = np.stack([row["a"] for row in samples], axis=0)
        mu = np.stack([row["mu"] for row in samples], axis=0)
        labels = np.stack([row["y"] for row in samples], axis=0)

        bands = [row["band"] for row in samples]
        metadata = [json.dumps(row["metadata"], ensure_ascii=True, sort_keys=True) for row in samples]

        str_dtype = h5py.string_dtype(encoding="utf-8")

        with h5py.File(output_path, "w") as h5f:
            h5f.create_dataset(
                "X_map",
                data=x_map,
                compression=compression,
                compression_opts=compression_level,
            )
            h5f.create_dataset("theta_v", data=theta_v)
            h5f.create_dataset("a", data=actions)
            h5f.create_dataset("mu", data=mu)
            h5f.create_dataset("y", data=labels)
            h5f.create_dataset("band", data=np.array(bands, dtype=object), dtype=str_dtype)
            h5f.create_dataset("metadata_json", data=np.array(metadata, dtype=object), dtype=str_dtype)

            h5f.attrs["schema"] = "(X_map,theta_v,a,mu,y,band,metadata_json)"
            h5f.attrs["version"] = "0.3.0"
