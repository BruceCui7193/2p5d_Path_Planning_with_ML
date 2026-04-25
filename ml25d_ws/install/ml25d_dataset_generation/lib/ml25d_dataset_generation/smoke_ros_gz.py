#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from ml25d_dataset_generation.dataset_manager import DatasetManager


def main() -> int:
    package_root = Path(__file__).resolve().parents[1]
    manager = DatasetManager(package_root=package_root)
    result = manager.generate_dataset(num_samples=1, backend="ros_gz", output_dir=Path("data/generated_ros_gz_smoke"), seed=20260424)
    print(result["manifest_path"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
