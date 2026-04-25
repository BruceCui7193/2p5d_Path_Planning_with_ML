#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
from typing import Dict, List

import h5py
import numpy as np


REQUIRED_KEYS = ["X_map", "theta_v", "a", "mu", "y", "band", "metadata_json"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate generated HDF5 sample files")
    parser.add_argument("--pattern", type=str, required=True, help="Glob pattern for hdf5 files")
    parser.add_argument("--report", type=Path, default=None, help="Optional report output path")
    return parser.parse_args()


def validate_file(file_path: Path) -> Dict:
    problems: List[str] = []
    stats: Dict[str, float] = {}

    with h5py.File(file_path, "r") as h5f:
        for key in REQUIRED_KEYS:
            if key not in h5f:
                problems.append(f"missing dataset: {key}")

        if problems:
            return {"file": str(file_path), "ok": False, "problems": problems, "stats": stats}

        x_map = h5f["X_map"][:]
        theta_v = h5f["theta_v"][:]
        actions = h5f["a"][:]
        mu = h5f["mu"][:]
        y = h5f["y"][:]

        if x_map.ndim != 4 or x_map.shape[-1] != 6:
            problems.append(f"X_map shape invalid: {x_map.shape}")
        if theta_v.ndim != 2 or theta_v.shape[-1] != 12:
            problems.append(f"theta_v shape invalid: {theta_v.shape}")
        if actions.ndim != 2 or actions.shape[-1] != 4:
            problems.append(f"a shape invalid: {actions.shape}")
        if mu.ndim != 2 or mu.shape[-1] != 1:
            problems.append(f"mu shape invalid: {mu.shape}")
        if y.ndim != 2 or y.shape[-1] != 7:
            problems.append(f"y shape invalid: {y.shape}")

        for arr_name, arr in [("X_map", x_map), ("theta_v", theta_v), ("a", actions), ("mu", mu), ("y", y)]:
            if np.any(np.isnan(arr)):
                problems.append(f"{arr_name} contains NaN")
            if np.any(np.isinf(arr)):
                problems.append(f"{arr_name} contains Inf")

        if np.any(y < 0.0) or np.any(y > 1.0):
            problems.append("y values must stay in [0,1]")

        stats["samples"] = float(y.shape[0])
        stats["y_fail_rate"] = float(np.mean(y[:, 0]))
        stats["q_roll_max"] = float(np.max(y[:, 1]))
        stats["q_pitch_max"] = float(np.max(y[:, 2]))

    return {"file": str(file_path), "ok": len(problems) == 0, "problems": problems, "stats": stats}


def main() -> int:
    args = parse_args()
    files = sorted(glob.glob(args.pattern))
    if not files:
        raise SystemExit(f"no files matched pattern: {args.pattern}")

    results = [validate_file(Path(path)) for path in files]
    summary = {
        "num_files": len(results),
        "num_failed": int(sum(0 if row["ok"] else 1 for row in results)),
        "files": results,
    }

    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8")

    print(json.dumps(summary, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
