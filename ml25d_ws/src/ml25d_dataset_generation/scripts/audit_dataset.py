#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from typing import Any

import numpy as np

from ml25d_dataset_generation.training_data import LABEL_NAMES, load_hdf5_dataset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit generated HDF5 dataset statistics")
    parser.add_argument("pattern", type=str, help="Glob pattern for HDF5 files")
    return parser.parse_args()


def _value_counts(values: list[Any]) -> dict[str, int]:
    counter = Counter(str(v) for v in values)
    return dict(sorted(counter.items(), key=lambda kv: (-kv[1], kv[0])))


def _crosstab(rows: list[Any], cols: list[Any]) -> tuple[list[str], list[str], dict[str, dict[str, int]]]:
    row_keys = sorted({str(v) for v in rows})
    col_keys = sorted({str(v) for v in cols})
    table: dict[str, dict[str, int]] = {r: {c: 0 for c in col_keys} for r in row_keys}
    for r, c in zip(rows, cols):
        table[str(r)][str(c)] += 1
    return row_keys, col_keys, table


def _print_crosstab(title: str, rows: list[Any], cols: list[Any], normalize_row: bool = False) -> None:
    row_keys, col_keys, table = _crosstab(rows, cols)
    print(f"\n{title}")
    header = "row".ljust(24) + "".join(c.rjust(14) for c in col_keys)
    print(header)
    for r in row_keys:
        total = max(sum(table[r].values()), 1)
        if normalize_row:
            vals = [f"{table[r][c] / total:.3f}" for c in col_keys]
        else:
            vals = [str(table[r][c]) for c in col_keys]
        print(r.ljust(24) + "".join(v.rjust(14) for v in vals))


def _describe(arr: np.ndarray) -> dict[str, float]:
    if arr.size == 0:
        return {"mean": float("nan"), "p05": float("nan"), "p25": float("nan"), "p50": float("nan"), "p75": float("nan"), "p95": float("nan")}
    return {
        "mean": float(np.mean(arr)),
        "p05": float(np.percentile(arr, 5)),
        "p25": float(np.percentile(arr, 25)),
        "p50": float(np.percentile(arr, 50)),
        "p75": float(np.percentile(arr, 75)),
        "p95": float(np.percentile(arr, 95)),
    }


def _print_label_summary(y: np.ndarray) -> None:
    print("\nLabel summary:")
    for i, name in enumerate(LABEL_NAMES):
        d = _describe(y[:, i])
        print(
            f"{name:10s} mean={d['mean']:.4f} p05={d['p05']:.4f} p25={d['p25']:.4f} "
            f"p50={d['p50']:.4f} p75={d['p75']:.4f} p95={d['p95']:.4f}"
        )


def main() -> int:
    args = parse_args()
    data = load_hdf5_dataset(args.pattern)
    meta = data.metadata
    n = len(meta)
    if n == 0:
        raise RuntimeError("no samples loaded")

    band = [str(v) for v in data.band.tolist()]
    vehicle = [str(m.get("vehicle_id", "")) for m in meta]
    terrain = [str(m.get("terrain_class", "")) for m in meta]
    action = [str(m.get("action_id", "")) for m in meta]
    friction = [str(m.get("friction_class", "")) for m in meta]

    print(f"N = {n}")
    print("\nBand counts:")
    for k, v in _value_counts(band).items():
        print(f"{k:12s} {v}")

    _print_crosstab("Vehicle × band", vehicle, band, normalize_row=False)
    _print_crosstab("Vehicle × band normalized", vehicle, band, normalize_row=True)
    _print_crosstab("Terrain × band", terrain, band, normalize_row=False)
    _print_crosstab("Action × band", action, band, normalize_row=False)
    _print_crosstab("Friction × band", friction, band, normalize_row=False)

    _print_label_summary(data.y)

    print("\nFlat sanity:")
    flat_idx = [i for i, t in enumerate(terrain) if t == "flat"]
    print(f"flat N = {len(flat_idx)}")
    if flat_idx:
        flat_y = data.y[np.asarray(flat_idx, dtype=np.int64)]
        flat_fail_rate = float(np.mean(flat_y[:, 0]))
        print(f"flat fail rate = {flat_fail_rate:.6f}")
        flat_vehicle = [vehicle[i] for i in flat_idx]
        flat_band = [band[i] for i in flat_idx]
        _print_crosstab("Flat vehicle × band", flat_vehicle, flat_band, normalize_row=False)
        for name in ["q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"]:
            idx = LABEL_NAMES.index(name)
            d = _describe(flat_y[:, idx])
            print(f"flat {name:8s} mean={d['mean']:.4f} p50={d['p50']:.4f} p95={d['p95']:.4f}")

    print("\nNaN check:")
    print("x_map nan:", int(np.isnan(data.x_map).sum()))
    print("theta_v nan:", int(np.isnan(data.theta_v).sum()))
    print("action nan:", int(np.isnan(data.action).sum()))
    print("mu nan:", int(np.isnan(data.mu).sum()))
    print("y nan:", int(np.isnan(data.y).sum()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

