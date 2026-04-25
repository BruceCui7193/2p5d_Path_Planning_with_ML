#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
from typing import Dict, List

import h5py
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build dataset statistics report")
    parser.add_argument("--pattern", type=str, required=True, help="Glob pattern for hdf5 files")
    parser.add_argument("--output", type=Path, default=None, help="Optional output json path")
    return parser.parse_args()


def collect(files: List[str]) -> Dict:
    total = 0
    y_all = []
    band_all = []

    for file_path in files:
        with h5py.File(file_path, "r") as h5f:
            y = h5f["y"][:]
            bands = [b.decode("utf-8") if isinstance(b, bytes) else str(b) for b in h5f["band"][:]]
            total += y.shape[0]
            y_all.append(y)
            band_all.extend(bands)

    y_cat = np.concatenate(y_all, axis=0) if y_all else np.zeros((0, 7), dtype=np.float32)

    band_counts = {
        "safe": int(sum(1 for b in band_all if b == "safe")),
        "fail": int(sum(1 for b in band_all if b == "fail")),
        "critical": int(sum(1 for b in band_all if b == "critical")),
    }

    return {
        "total_samples": total,
        "band_counts": band_counts,
        "band_ratio": {k: (v / total if total else 0.0) for k, v in band_counts.items()},
        "label_mean": y_cat.mean(axis=0).tolist() if total else [0.0] * 7,
        "label_max": y_cat.max(axis=0).tolist() if total else [0.0] * 7,
        "label_min": y_cat.min(axis=0).tolist() if total else [0.0] * 7,
    }


def main() -> int:
    args = parse_args()
    files = sorted(glob.glob(args.pattern))
    if not files:
        raise SystemExit(f"no files matched pattern: {args.pattern}")

    report = collect(files)
    print(json.dumps(report, indent=2, ensure_ascii=True))

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
