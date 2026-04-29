#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate dataset in shards using parallel pilot generator")
    parser.add_argument("--output-root", type=Path, required=True, help="Root output directory for shard_*")
    parser.add_argument("--num-shards", type=int, default=20)
    parser.add_argument("--samples-per-shard", type=int, default=1000)
    parser.add_argument("--workers", type=int, default=4, help="Workers per shard (supports >8).")
    parser.add_argument("--base-domain-start", type=int, default=140)
    parser.add_argument("--seed-start", type=int, default=20260428)
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    parser.add_argument(
        "--balance-mode",
        type=str,
        default="vehicle_band",
        choices=["band", "vehicle_band"],
        help="Balancing strategy forwarded to generate_pilot_parallel.py.",
    )
    parser.add_argument(
        "--disable-balance",
        action="store_true",
        help="Disable balance constraints in shard generation.",
    )
    parser.add_argument(
        "--generator-script",
        type=Path,
        default=Path("src/ml25d_dataset_generation/scripts/generate_pilot_parallel.py"),
    )
    parser.add_argument(
        "--run-tag-prefix",
        type=str,
        default="datasetv1",
        help="Prefix for per-shard run tags, used for world/model isolation.",
    )
    parser.add_argument(
        "--terrain-compensation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable adaptive terrain sampling compensation in shard generation.",
    )
    parser.add_argument("--terrain-comp-strength", type=float, default=1.8)
    parser.add_argument("--terrain-comp-warmup", type=int, default=40)
    parser.add_argument("--terrain-comp-min-mult", type=float, default=0.30)
    parser.add_argument("--terrain-comp-max-mult", type=float, default=5.00)
    parser.add_argument("--max-attempt-multiplier", type=int, default=80)
    parser.add_argument(
        "--worker-startup-stagger-sec",
        type=float,
        default=0.8,
        help="Forwarded to generate_pilot_parallel.py (--worker-startup-stagger-sec).",
    )
    parser.add_argument(
        "--ros-startup-timeout-sec",
        type=float,
        default=35.0,
        help="Forwarded to generate_pilot_parallel.py (--ros-startup-timeout-sec).",
    )
    parser.add_argument(
        "--ros-service-timeout-sec",
        type=float,
        default=12.0,
        help="Forwarded to generate_pilot_parallel.py (--ros-service-timeout-sec).",
    )
    parser.add_argument(
        "--result-queue-mult",
        type=int,
        default=12,
        help="Forwarded to generate_pilot_parallel.py (--result-queue-mult).",
    )
    parser.add_argument("--resume", action="store_true", help="Skip completed shards")
    return parser.parse_args()


def _load_manifest(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _is_completed_shard(path: Path, target_samples: int) -> bool:
    manifest_path = path / "dataset_manifest.json"
    if not manifest_path.exists():
        return False
    try:
        manifest = _load_manifest(manifest_path)
    except Exception:
        return False
    accepted = int(manifest.get("dataset", {}).get("num_samples_accepted", 0))
    return accepted >= target_samples


def main() -> int:
    args = parse_args()
    if int(args.workers) <= 0:
        raise ValueError("workers must be positive")
    if int(args.result_queue_mult) <= 0:
        raise ValueError("result_queue_mult must be positive")
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    gen_script = args.generator_script.resolve()
    if not gen_script.exists():
        raise FileNotFoundError(f"generator script not found: {gen_script}")

    shard_rows: list[dict[str, Any]] = []
    total_valid = 0
    started_at = datetime.now(tz=timezone.utc).isoformat()

    for shard_idx in range(1, args.num_shards + 1):
        shard_name = f"shard_{shard_idx:04d}"
        shard_dir = output_root / shard_name
        shard_dir.mkdir(parents=True, exist_ok=True)

        if args.resume and _is_completed_shard(shard_dir, args.samples_per_shard):
            manifest = _load_manifest(shard_dir / "dataset_manifest.json")
            accepted = int(manifest["dataset"]["num_samples_accepted"])
            total_valid += accepted
            row = {
                "shard": shard_name,
                "status": "skipped_completed",
                "accepted": accepted,
                "manifest_path": str(shard_dir / "dataset_manifest.json"),
            }
            shard_rows.append(row)
            print(f"[shards] skip {shard_name} accepted={accepted}", flush=True)
            continue

        base_domain = int(args.base_domain_start)
        seed = int(args.seed_start) + shard_idx * 101
        cmd = [
            sys.executable,
            str(gen_script),
            "--output-dir",
            str(shard_dir),
            "--num-samples",
            str(args.samples_per_shard),
            "--seed",
            str(seed),
            "--backend",
            str(args.backend),
            "--num-workers",
            str(args.workers),
            "--base-domain",
            str(base_domain),
            "--balance-mode",
            str(args.balance_mode),
            "--worker-startup-stagger-sec",
            str(float(args.worker_startup_stagger_sec)),
            "--ros-startup-timeout-sec",
            str(float(args.ros_startup_timeout_sec)),
            "--ros-service-timeout-sec",
            str(float(args.ros_service_timeout_sec)),
            "--run-tag",
            f"{args.run_tag_prefix}_s{shard_idx:04d}",
            "--max-attempt-multiplier",
            str(int(args.max_attempt_multiplier)),
            "--result-queue-mult",
            str(int(args.result_queue_mult)),
            "--terrain-comp-strength",
            str(float(args.terrain_comp_strength)),
            "--terrain-comp-warmup",
            str(int(args.terrain_comp_warmup)),
            "--terrain-comp-min-mult",
            str(float(args.terrain_comp_min_mult)),
            "--terrain-comp-max-mult",
            str(float(args.terrain_comp_max_mult)),
        ]
        cmd.append("--terrain-compensation" if bool(args.terrain_compensation) else "--no-terrain-compensation")
        if args.disable_balance:
            cmd.append("--disable-balance")
        print(f"[shards] start {shard_name} seed={seed}", flush=True)
        proc = subprocess.run(cmd)
        if proc.returncode != 0:
            row = {"shard": shard_name, "status": "failed", "return_code": int(proc.returncode)}
            shard_rows.append(row)
            summary = {
                "started_at_utc": started_at,
                "finished_at_utc": datetime.now(tz=timezone.utc).isoformat(),
                "output_root": str(output_root),
                "num_shards": int(args.num_shards),
                "samples_per_shard": int(args.samples_per_shard),
                "workers": int(args.workers),
                "run_tag_prefix": str(args.run_tag_prefix),
                "terrain_compensation": {
                    "enabled": bool(args.terrain_compensation),
                    "strength": float(args.terrain_comp_strength),
                    "warmup": int(args.terrain_comp_warmup),
                    "min_multiplier": float(args.terrain_comp_min_mult),
                    "max_multiplier": float(args.terrain_comp_max_mult),
                },
                "total_valid_samples": total_valid,
                "rows": shard_rows,
            }
            summary_path = output_root / "shards_summary.json"
            summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
            raise RuntimeError(f"shard failed: {shard_name} rc={proc.returncode}")

        manifest_path = shard_dir / "dataset_manifest.json"
        manifest = _load_manifest(manifest_path)
        accepted = int(manifest["dataset"]["num_samples_accepted"])
        total_valid += accepted
        row = {
            "shard": shard_name,
            "status": "completed",
            "accepted": accepted,
            "manifest_path": str(manifest_path),
        }
        shard_rows.append(row)
        print(f"[shards] done {shard_name} accepted={accepted} total_valid={total_valid}", flush=True)

    summary = {
        "started_at_utc": started_at,
        "finished_at_utc": datetime.now(tz=timezone.utc).isoformat(),
        "output_root": str(output_root),
        "num_shards": int(args.num_shards),
        "samples_per_shard": int(args.samples_per_shard),
        "workers": int(args.workers),
        "run_tag_prefix": str(args.run_tag_prefix),
        "terrain_compensation": {
            "enabled": bool(args.terrain_compensation),
            "strength": float(args.terrain_comp_strength),
            "warmup": int(args.terrain_comp_warmup),
            "min_multiplier": float(args.terrain_comp_min_mult),
            "max_multiplier": float(args.terrain_comp_max_mult),
        },
        "total_valid_samples": total_valid,
        "rows": shard_rows,
    }
    summary_path = output_root / "shards_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"summary_path": str(summary_path), "total_valid_samples": total_valid}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
