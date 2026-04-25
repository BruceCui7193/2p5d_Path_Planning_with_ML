from __future__ import annotations

from dataclasses import dataclass
import glob
import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np


LABEL_NAMES = ["y_fail", "q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]


@dataclass(frozen=True)
class RiskDatasetArrays:
    x_map: np.ndarray
    theta_v: np.ndarray
    action: np.ndarray
    mu: np.ndarray
    y: np.ndarray
    band: np.ndarray
    metadata: list[dict[str, Any]]

    @property
    def param(self) -> np.ndarray:
        return np.concatenate([self.theta_v, self.action, self.mu], axis=1).astype(np.float32)


@dataclass(frozen=True)
class SplitIndices:
    train: np.ndarray
    val: np.ndarray
    test: np.ndarray


def load_hdf5_dataset(pattern: str | Path) -> RiskDatasetArrays:
    files = sorted(glob.glob(str(pattern)))
    if not files:
        raise FileNotFoundError(f"no HDF5 files matched: {pattern}")

    x_map: list[np.ndarray] = []
    theta_v: list[np.ndarray] = []
    action: list[np.ndarray] = []
    mu: list[np.ndarray] = []
    y: list[np.ndarray] = []
    bands: list[str] = []
    metadata: list[dict[str, Any]] = []

    for file_path in files:
        with h5py.File(file_path, "r") as h5f:
            x_map.append(h5f["X_map"][:].astype(np.float32))
            theta_v.append(h5f["theta_v"][:].astype(np.float32))
            action.append(h5f["a"][:].astype(np.float32))
            mu.append(h5f["mu"][:].astype(np.float32))
            y.append(h5f["y"][:].astype(np.float32))
            bands.extend(_decode_text(row) for row in h5f["band"][:])
            metadata.extend(json.loads(_decode_text(row)) for row in h5f["metadata_json"][:])

    return RiskDatasetArrays(
        x_map=np.concatenate(x_map, axis=0),
        theta_v=np.concatenate(theta_v, axis=0),
        action=np.concatenate(action, axis=0),
        mu=np.concatenate(mu, axis=0),
        y=np.concatenate(y, axis=0),
        band=np.asarray(bands),
        metadata=metadata,
    )


def make_split(num_samples: int, seed: int, train_ratio: float = 0.70, val_ratio: float = 0.15) -> SplitIndices:
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")
    if not 0.0 < train_ratio < 1.0:
        raise ValueError("train_ratio must be in (0, 1)")
    if not 0.0 < val_ratio < 1.0:
        raise ValueError("val_ratio must be in (0, 1)")
    if train_ratio + val_ratio >= 1.0:
        raise ValueError("train_ratio + val_ratio must be < 1")

    rng = np.random.default_rng(seed)
    indices = rng.permutation(num_samples)
    n_train = int(round(num_samples * train_ratio))
    n_val = int(round(num_samples * val_ratio))
    train = np.sort(indices[:n_train])
    val = np.sort(indices[n_train : n_train + n_val])
    test = np.sort(indices[n_train + n_val :])
    return SplitIndices(train=train, val=val, test=test)


def make_stratified_split(
    labels: np.ndarray,
    seed: int,
    train_ratio: float = 0.70,
    val_ratio: float = 0.15,
) -> SplitIndices:
    labels = np.asarray(labels)
    rng = np.random.default_rng(seed)
    train_parts: list[np.ndarray] = []
    val_parts: list[np.ndarray] = []
    test_parts: list[np.ndarray] = []

    for value in np.unique(labels):
        idx = np.flatnonzero(labels == value)
        shuffled = rng.permutation(idx)
        n_train = int(round(idx.shape[0] * train_ratio))
        n_val = int(round(idx.shape[0] * val_ratio))
        train_parts.append(shuffled[:n_train])
        val_parts.append(shuffled[n_train : n_train + n_val])
        test_parts.append(shuffled[n_train + n_val :])

    return SplitIndices(
        train=np.sort(np.concatenate(train_parts)),
        val=np.sort(np.concatenate(val_parts)),
        test=np.sort(np.concatenate(test_parts)),
    )


def compute_channel_stats(x_map: np.ndarray, indices: np.ndarray) -> dict[str, list[float]]:
    if x_map.ndim != 4 or x_map.shape[-1] != 6:
        raise ValueError(f"x_map must have shape (N,H,W,6), got {x_map.shape}")
    subset = x_map[indices].reshape(-1, x_map.shape[-1])
    mean = subset.mean(axis=0)
    std = subset.std(axis=0)
    std = np.maximum(std, 1e-6)
    return {"mean": mean.astype(float).tolist(), "std": std.astype(float).tolist()}


def normalize_x_map(x_map: np.ndarray, channel_stats: dict[str, list[float]]) -> np.ndarray:
    mean = np.asarray(channel_stats["mean"], dtype=np.float32).reshape(1, 1, 1, -1)
    std = np.asarray(channel_stats["std"], dtype=np.float32).reshape(1, 1, 1, -1)
    return ((x_map - mean) / std).astype(np.float32)


def _decode_text(value: Any) -> str:
    return value.decode("utf-8") if isinstance(value, (bytes, bytearray)) else str(value)
