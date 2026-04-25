#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ml25d_dataset_generation.dataset_manager import DatasetManager


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a small real ros_gz dataset-generation smoke test")
    parser.add_argument("--num-samples", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--output-dir", type=Path, default=Path("data/generated_ros_gz_smoke"))
    parser.add_argument("--config-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root, config_dir=args.config_dir)
    result = manager.generate_dataset(
        num_samples=args.num_samples,
        backend="ros_gz",
        output_dir=args.output_dir,
        seed=args.seed,
    )
    print(json.dumps(result, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
