from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
import time
from typing import Any, Dict, List, Tuple

import numpy as np

from .common_types import ActionPrimitive, SampleMetadata, VEHICLE_PARAM_ORDER, VehicleParams
from .config_loader import build_action_library, build_vehicle_library, load_all_configs, weighted_table
from .gazebo_runner import SimulationContext, make_runner
from .label_extractor import LabelExtractor
from .sample_packager import SamplePackager
from .terrain_generator import TerrainGenerator


class DatasetManager:
    def __init__(self, package_root: Path, config_dir: Path | None = None) -> None:
        self.package_root = package_root.resolve()
        self.workspace_root = Path.cwd()
        self.config_dir = self._discover_config_dir(config_dir)
        self.configs = load_all_configs(self.config_dir)

        self.dataset_cfg = self.configs["dataset"]
        self.vehicle_cfg = self.configs["vehicles"]
        self.terrain_cfg = self.configs["terrain"]
        self.friction_cfg = self.configs["friction"]
        self.action_cfg = self.configs["actions"]
        self.label_cfg = self.configs["labels"]

        self.map_cfg = self.dataset_cfg["map"]
        self.sim_cfg = self.dataset_cfg["simulation"]
        self.ser_cfg = self.dataset_cfg["serialization"]
        self.quality_cfg = self.dataset_cfg["quality"]
        self.ds_meta = self.dataset_cfg["dataset"]

        self.vehicle_library = build_vehicle_library(self.vehicle_cfg)
        self.action_library = build_action_library(self.action_cfg)
        self.terrain_generator = TerrainGenerator(
            patch_size=int(self.map_cfg["patch_size"]),
            resolution_m=float(self.map_cfg["resolution_m_per_cell"]),
        )
        self.label_extractor = LabelExtractor(self.label_cfg)
        self.packager = SamplePackager(self.map_cfg, self.vehicle_cfg)

    def generate_dataset(
        self,
        num_samples: int | None = None,
        output_dir: Path | None = None,
        seed: int | None = None,
        backend: str | None = None,
    ) -> Dict[str, Any]:
        num_samples = int(num_samples if num_samples is not None else self.ds_meta["num_samples"])
        if num_samples <= 0:
            raise ValueError("num_samples must be positive")

        seed = int(seed if seed is not None else self.ds_meta["random_seed"])
        backend = str(backend if backend is not None else self.ds_meta["backend"])
        output_dir = self._resolve_output_dir(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        verbose_ros_gz = backend == "ros_gz"

        runner = make_runner(backend, self.sim_cfg)
        rng = np.random.default_rng(seed)

        targets = self._compute_band_targets(num_samples)
        band_counts = {"safe": 0, "fail": 0, "critical": 0}
        enforce_balance = num_samples >= 50

        terrain_counts: Dict[str, int] = {}
        vehicle_counts: Dict[str, int] = {}
        friction_counts: Dict[str, int] = {}
        motion_counts: Dict[str, int] = {}
        action_counts: Dict[str, int] = {}

        samples_in_batch: List[Dict[str, Any]] = []
        batch_files: List[str] = []
        batch_size = int(self.ds_meta["batch_size"])
        max_attempts = max(num_samples * 80, 1000)

        accepted = 0
        attempts = 0
        failed_attempts = 0
        consecutive_failures = 0
        while accepted < num_samples and attempts < max_attempts:
            attempts += 1
            sample_seed = int(rng.integers(0, 2**31 - 1))
            attempt_started = time.monotonic()
            try:
                sample, band, counts_info = self.generate_one_sample(sample_id=accepted, seed=sample_seed, runner=runner)
            except Exception as exc:
                failed_attempts += 1
                consecutive_failures += 1
                if verbose_ros_gz and (failed_attempts <= 10 or failed_attempts % 10 == 0):
                    print(
                        "[ml25d][ros_gz] failed "
                        f"attempt={attempts} accepted={accepted} failed={failed_attempts} "
                        f"error={str(exc)[:240]}",
                        flush=True,
                    )
                if consecutive_failures >= max(20, num_samples * 3):
                    raise RuntimeError(
                        f"too many consecutive simulation failures while generating dataset: {consecutive_failures}"
                    ) from exc
                continue
            consecutive_failures = 0

            if enforce_balance and band_counts[band] >= targets[band]:
                continue

            samples_in_batch.append(sample)
            accepted += 1
            band_counts[band] += 1
            if verbose_ros_gz and (accepted <= 20 or accepted % 10 == 0 or accepted == num_samples):
                print(
                    "[ml25d][ros_gz] accepted "
                    f"{accepted}/{num_samples} band={band} attempts={attempts} failed={failed_attempts} "
                    f"elapsed={time.monotonic() - attempt_started:.1f}s "
                    f"band_counts={band_counts}",
                    flush=True,
                )

            terrain_counts[counts_info["terrain_class"]] = terrain_counts.get(counts_info["terrain_class"], 0) + 1
            vehicle_counts[counts_info["vehicle_id"]] = vehicle_counts.get(counts_info["vehicle_id"], 0) + 1
            friction_counts[counts_info["friction_class"]] = friction_counts.get(counts_info["friction_class"], 0) + 1
            motion_counts[counts_info["motion_model"]] = motion_counts.get(counts_info["motion_model"], 0) + 1
            action_counts[counts_info["action_id"]] = action_counts.get(counts_info["action_id"], 0) + 1

            if len(samples_in_batch) >= batch_size:
                batch_file = output_dir / f"samples_batch_{len(batch_files) + 1:04d}.h5"
                self.packager.write_hdf5_batch(
                    samples=samples_in_batch,
                    output_path=batch_file,
                    compression=str(self.ser_cfg["compression"]),
                    compression_level=int(self.ser_cfg["compression_level"]),
                )
                batch_files.append(str(batch_file.name))
                samples_in_batch = []

        if accepted < num_samples:
            if enforce_balance:
                print(
                    f"[ml25d] balance fill stage: accepted={accepted}, attempts={attempts}, target={num_samples}"
                )
            while accepted < num_samples:
                attempts += 1
                sample_seed = int(rng.integers(0, 2**31 - 1))
                attempt_started = time.monotonic()
                try:
                    sample, band, counts_info = self.generate_one_sample(sample_id=accepted, seed=sample_seed, runner=runner)
                except Exception as exc:
                    failed_attempts += 1
                    consecutive_failures += 1
                    if verbose_ros_gz and (failed_attempts <= 10 or failed_attempts % 10 == 0):
                        print(
                            "[ml25d][ros_gz] failed "
                            f"attempt={attempts} accepted={accepted} failed={failed_attempts} "
                            f"error={str(exc)[:240]}",
                            flush=True,
                        )
                    if consecutive_failures >= max(20, num_samples * 3):
                        raise RuntimeError(
                            f"too many consecutive simulation failures during balance fill stage: {consecutive_failures}"
                        ) from exc
                    continue
                consecutive_failures = 0
                samples_in_batch.append(sample)
                accepted += 1
                band_counts[band] += 1
                if verbose_ros_gz and (accepted <= 20 or accepted % 10 == 0 or accepted == num_samples):
                    print(
                        "[ml25d][ros_gz] accepted "
                        f"{accepted}/{num_samples} band={band} attempts={attempts} failed={failed_attempts} "
                        f"elapsed={time.monotonic() - attempt_started:.1f}s "
                        f"band_counts={band_counts}",
                        flush=True,
                    )
                terrain_counts[counts_info["terrain_class"]] = terrain_counts.get(counts_info["terrain_class"], 0) + 1
                vehicle_counts[counts_info["vehicle_id"]] = vehicle_counts.get(counts_info["vehicle_id"], 0) + 1
                friction_counts[counts_info["friction_class"]] = friction_counts.get(counts_info["friction_class"], 0) + 1
                motion_counts[counts_info["motion_model"]] = motion_counts.get(counts_info["motion_model"], 0) + 1
                action_counts[counts_info["action_id"]] = action_counts.get(counts_info["action_id"], 0) + 1

                if len(samples_in_batch) >= batch_size:
                    batch_file = output_dir / f"samples_batch_{len(batch_files) + 1:04d}.h5"
                    self.packager.write_hdf5_batch(
                        samples=samples_in_batch,
                        output_path=batch_file,
                        compression=str(self.ser_cfg["compression"]),
                        compression_level=int(self.ser_cfg["compression_level"]),
                    )
                    batch_files.append(str(batch_file.name))
                    samples_in_batch = []

        if samples_in_batch:
            batch_file = output_dir / f"samples_batch_{len(batch_files) + 1:04d}.h5"
            self.packager.write_hdf5_batch(
                samples=samples_in_batch,
                output_path=batch_file,
                compression=str(self.ser_cfg["compression"]),
                compression_level=int(self.ser_cfg["compression_level"]),
            )
            batch_files.append(str(batch_file.name))

        manifest = {
            "dataset": {
                "name": self.ds_meta["name"],
                "version": self.ds_meta["version"],
                "created_at_utc": datetime.now(tz=timezone.utc).isoformat(),
                "num_samples": num_samples,
                "attempts": attempts,
                "failed_attempts": failed_attempts,
                "backend": backend,
                "seed": seed,
            },
            "targets": targets,
            "counts": {
                "band": band_counts,
                "terrain": terrain_counts,
                "vehicle": vehicle_counts,
                "friction": friction_counts,
                "motion_model": motion_counts,
                "action": action_counts,
            },
            "batches": batch_files,
            "config_snapshot": {
                "map": self.map_cfg,
                "simulation": self.sim_cfg,
                "quality": self.quality_cfg,
                "serialization": self.ser_cfg,
            },
        }

        manifest_path = output_dir / str(self.ser_cfg["manifest_name"])
        with manifest_path.open("w", encoding="utf-8") as fp:
            json.dump(manifest, fp, indent=2, ensure_ascii=True)

        return {"manifest_path": str(manifest_path), "manifest": manifest}

    def generate_one_sample(self, sample_id: int, seed: int, runner=None) -> Tuple[Dict[str, Any], str, Dict[str, str]]:
        local_rng = np.random.default_rng(seed)
        runner = runner or make_runner(str(self.ds_meta["backend"]), self.sim_cfg)

        terrain_rows, terrain_probs = weighted_table(self.terrain_cfg["terrain"]["classes"])
        terrain_idx = int(local_rng.choice(len(terrain_rows), p=terrain_probs))
        terrain_class = terrain_rows[terrain_idx]
        terrain_name = str(terrain_class["name"])
        terrain = self.terrain_generator.generate(local_rng, terrain_class)

        vehicle = self._sample_vehicle(local_rng)
        motion_model = "skid" if local_rng.random() < 0.5 else "ackermann"
        action = self._sample_action(local_rng, motion_model)

        friction_rows, friction_probs = weighted_table(self.friction_cfg["friction"]["classes"])
        friction_idx = int(local_rng.choice(len(friction_rows), p=friction_probs))
        friction_class = friction_rows[friction_idx]
        friction_name = str(friction_class["name"])
        mu_lo, mu_hi = friction_class["mu_range"]
        friction_mu = float(local_rng.uniform(mu_lo, mu_hi))

        heading_rad = float(local_rng.uniform(0.0, 2.0 * np.pi))
        context = SimulationContext(
            heightmap=terrain.heightmap,
            heading_rad=heading_rad,
            vehicle=vehicle,
            action=action,
            friction_mu=friction_mu,
            motion_model=motion_model,
            sample_rate_hz=int(self.sim_cfg["sample_rate_hz"]),
            duration_sec=float(self.sim_cfg["action_duration_sec"]),
            settle_time_sec=float(self.sim_cfg["settle_time_sec"]),
        )
        trajectory = runner.run(context, local_rng)

        labels, band = self.label_extractor.compute_labels(trajectory, vehicle, action)

        metadata = SampleMetadata(
            sample_id=sample_id,
            seed=seed,
            terrain_class=terrain_name,
            friction_class=friction_name,
            vehicle_id=vehicle.vehicle_id,
            action_id=action.action_id,
            action_name=action.name,
            motion_model=motion_model,
            heading_rad=heading_rad,
        )

        sample = self.packager.create_sample(
            heightmap=terrain.heightmap,
            heading_rad=heading_rad,
            vehicle=vehicle,
            action=action,
            friction_mu=friction_mu,
            labels=labels,
            band=band,
            metadata=metadata,
        )
        sample["metadata"]["terrain_parameters"] = terrain.parameters
        sample["metadata"]["friction_mu"] = friction_mu

        counts_info = {
            "terrain_class": terrain_name,
            "vehicle_id": vehicle.vehicle_id,
            "friction_class": friction_name,
            "motion_model": motion_model,
            "action_id": action.action_id,
        }
        return sample, band, counts_info

    def _sample_vehicle(self, rng: np.random.Generator) -> VehicleParams:
        idx = int(rng.integers(0, len(self.vehicle_library)))
        base = self.vehicle_library[idx]

        perturb_cfg = self.vehicle_cfg["vehicles"]["perturbation"]
        if not bool(perturb_cfg.get("enabled", False)):
            return base

        sigma = float(perturb_cfg.get("gaussian_sigma_ratio", 0.10))
        bounds = self.vehicle_cfg["normalization_bounds"]

        values: Dict[str, float] = {}
        for key in VEHICLE_PARAM_ORDER:
            raw = float(getattr(base, key))
            perturbed = raw * (1.0 + float(rng.normal(0.0, sigma)))
            lo, hi = bounds[key]
            values[key] = float(np.clip(perturbed, lo, hi))

        return VehicleParams(vehicle_id=base.vehicle_id, **values)

    def _sample_action(self, rng: np.random.Generator, motion_model: str) -> ActionPrimitive:
        feasible = self.action_library
        if motion_model == "ackermann":
            feasible = [action for action in self.action_library if action.delta_s_m > 1e-4]
        if not feasible:
            raise ValueError(f"no feasible actions configured for motion model: {motion_model}")

        idx = int(rng.integers(0, len(feasible)))
        return feasible[idx]

    def _resolve_output_dir(self, output_dir: Path | None) -> Path:
        if output_dir is not None:
            return output_dir.resolve()

        configured = Path(str(self.ds_meta["output_dir"]))
        if configured.is_absolute():
            return configured
        return (self.workspace_root / configured).resolve()

    def _discover_config_dir(self, config_dir: Path | None) -> Path:
        if config_dir is not None:
            resolved = config_dir.resolve()
            self._assert_config_dir(resolved)
            return resolved

        candidates = [
            (self.package_root / "config").resolve(),
            (self.package_root.parent.parent / "share" / "ml25d_dataset_generation" / "config").resolve(),
            (Path.cwd() / "src" / "ml25d_dataset_generation" / "config").resolve(),
        ]

        for candidate in candidates:
            if (candidate / "dataset_config.yaml").exists():
                return candidate

        candidate_text = "\n".join(str(c) for c in candidates)
        raise FileNotFoundError(f"could not locate config directory. tried:\n{candidate_text}")

    @staticmethod
    def _assert_config_dir(path: Path) -> None:
        required = [
            "dataset_config.yaml",
            "vehicle_params.yaml",
            "terrain_distribution.yaml",
            "friction_table.yaml",
            "action_primitives.yaml",
            "label_thresholds.yaml",
        ]
        missing = [name for name in required if not (path / name).exists()]
        if missing:
            missing_text = ", ".join(missing)
            raise FileNotFoundError(f"config directory is missing files: {missing_text}")

    def _compute_band_targets(self, num_samples: int) -> Dict[str, int]:
        if num_samples < 50:
            # Small smoke runs should not be constrained by strict class quota,
            # otherwise retries can dominate runtime and appear as hangs.
            return {"safe": num_samples, "fail": num_samples, "critical": num_samples}

        ratio_cfg = self.quality_cfg["target_ratio"]
        safe = int(round(num_samples * float(ratio_cfg["safe"])))
        fail = int(round(num_samples * float(ratio_cfg["fail"])))
        critical = max(num_samples - safe - fail, 0)
        return {"safe": safe, "fail": fail, "critical": critical}
