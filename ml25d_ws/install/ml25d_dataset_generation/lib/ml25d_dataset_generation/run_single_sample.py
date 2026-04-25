#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ml25d_dataset_generation.dataset_manager import DatasetManager
from ml25d_dataset_generation.gazebo_runner import make_runner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate one sample for debugging")
    parser.add_argument("--config-dir", type=Path, default=None, help="Path to config directory")
    parser.add_argument("--seed", type=int, default=42, help="Sample seed")
    parser.add_argument("--sample-id", type=int, default=0, help="Sample id")
    parser.add_argument(
        "--backend",
        type=str,
        default="surrogate",
        choices=["surrogate", "mock", "ros_gz"],
        help="Simulation backend",
    )
    parser.add_argument("--save-npz", type=Path, default=None, help="Optional npz output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]

    manager = DatasetManager(package_root=package_root, config_dir=args.config_dir)
    sample, band, meta = manager.generate_one_sample(
        sample_id=args.sample_id,
        seed=args.seed,
        runner=make_runner(args.backend, manager.sim_cfg),
    )

    print(json.dumps({"band": band, "meta": meta, "metadata": sample["metadata"]}, indent=2, ensure_ascii=True))

    if args.save_npz is not None:
        args.save_npz.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            args.save_npz,
            X_map=sample["X_map"],
            theta_v=sample["theta_v"],
            a=sample["a"],
            mu=sample["mu"],
            y=sample["y"],
        )
        print(f"saved sample npz to {args.save_npz}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
