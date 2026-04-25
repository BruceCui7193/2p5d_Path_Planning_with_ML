#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ml25d_dataset_generation.pso_training import TrainConfig, run_pso_training
from ml25d_dataset_generation.training_data import compute_channel_stats, load_hdf5_dataset, make_stratified_split


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train CNN+MLP risk model with PSO hyperparameter search")
    parser.add_argument(
        "--pattern",
        default="data/generated_hq_v1/samples_batch_*.h5",
        help="Glob pattern of HDF5 dataset batches",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("data/training_runs/cnn_pso_v1"))
    parser.add_argument("--seed", type=int, default=20260425)
    parser.add_argument("--device", default="auto", help="auto, cuda, cuda:0, or cpu")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--pso-particles", type=int, default=6)
    parser.add_argument("--pso-iters", type=int, default=4)
    parser.add_argument("--pso-epochs", type=int, default=5)
    parser.add_argument("--final-epochs", type=int, default=30)
    parser.add_argument("--max-pso-train-samples", type=int, default=1600)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data = load_hdf5_dataset(args.pattern)
    splits = make_stratified_split(data.band, seed=args.seed)
    channel_stats = compute_channel_stats(data.x_map, splits.train)

    cfg = TrainConfig(
        batch_size=args.batch_size,
        pso_epochs=args.pso_epochs,
        final_epochs=args.final_epochs,
        pso_particles=args.pso_particles,
        pso_iters=args.pso_iters,
        seed=args.seed,
        device=args.device,
        max_pso_train_samples=args.max_pso_train_samples,
    )
    report = run_pso_training(
        data=data,
        splits=splits,
        output_dir=args.output_dir,
        cfg=cfg,
        channel_stats=channel_stats,
    )
    print(json.dumps({"report": str(args.output_dir / "training_report.json"), "final": report["final"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
