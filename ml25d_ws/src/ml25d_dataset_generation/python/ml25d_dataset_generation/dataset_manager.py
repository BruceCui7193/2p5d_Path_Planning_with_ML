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
            band_hint = self._pick_band_hint(rng, band_counts, targets)
            vehicle_hint = self._pick_vehicle_hint(rng, vehicle_counts, accepted)
            try:
                sample, band, counts_info = self.generate_one_sample(
                    sample_id=accepted,
                    seed=sample_seed,
                    runner=runner,
                    band_hint=band_hint,
                    forced_vehicle_id=vehicle_hint,
                )
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

            samples_in_batch.append(sample)
            accepted += 1
            band_counts[band] += 1
            if verbose_ros_gz and (accepted <= 20 or accepted % 50 == 0 or accepted == num_samples):
                print(
                    "[ml25d][ros_gz] accepted "
                    f"{accepted}/{num_samples} band={band} hint={band_hint} attempts={attempts} failed={failed_attempts} "
                    f"elapsed={time.monotonic() - attempt_started:.1f}s "
                    f"band_counts={band_counts} vehicle_counts={vehicle_counts}",
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

    def generate_one_sample(
        self,
        sample_id: int,
        seed: int,
        runner=None,
        band_hint: str | None = None,
        forced_vehicle_id: str | None = None,
    ) -> Tuple[Dict[str, Any], str, Dict[str, str]]:
        local_rng = np.random.default_rng(seed)
        runner = runner or make_runner(str(self.ds_meta["backend"]), self.sim_cfg)

        heading_rad = float(local_rng.uniform(0.0, 2.0 * np.pi))
        terrain_class = self._sample_terrain_class_for_band(local_rng, band_hint)
        terrain_name = str(terrain_class["name"])
        terrain = self.terrain_generator.generate(local_rng, terrain_class, travel_heading_rad=heading_rad)

        vehicle = self._sample_vehicle(local_rng, forced_vehicle_id=forced_vehicle_id)
        motion_model = "skid"
        action = self._sample_action(local_rng, motion_model)

        friction_name, friction_mu = self._sample_friction_for_band(local_rng, band_hint)

        max_retries = max(int(self.sim_cfg.get("retries_per_sample", 0)), 0)
        trajectory = None
        retry_count = 0
        last_error: Exception | None = None
        while retry_count <= max_retries:
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
                cmd_ramp_sec=float(self.sim_cfg.get("cmd_ramp_sec", 0.3)),
            )
            try:
                trajectory = runner.run(context, local_rng)
                break
            except Exception as exc:
                last_error = exc
                retry_count += 1
                if retry_count > max_retries:
                    raise
                # Retry the same semantic sample with a fresh heading to avoid
                # unstable spawn orientations on rough terrain.
                heading_rad = float(local_rng.uniform(0.0, 2.0 * np.pi))
        if trajectory is None:
            assert last_error is not None
            raise last_error

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
        sample["metadata"]["simulation_retry_count"] = int(retry_count)

        counts_info = {
            "terrain_class": terrain_name,
            "vehicle_id": vehicle.vehicle_id,
            "friction_class": friction_name,
            "motion_model": motion_model,
            "action_id": action.action_id,
        }
        return sample, band, counts_info

    def _sample_vehicle_with_hint(
        self,
        rng: np.random.Generator,
        forced_vehicle_id: str | None = None,
    ) -> VehicleParams:
        if forced_vehicle_id is not None:
            matches = [v for v in self.vehicle_library if v.vehicle_id == forced_vehicle_id]
            if not matches:
                raise ValueError(f"unknown forced_vehicle_id: {forced_vehicle_id}")
            base = matches[0]
        else:
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

    def _sample_vehicle(self, rng: np.random.Generator, forced_vehicle_id: str | None = None) -> VehicleParams:  # type: ignore[override]
        return self._sample_vehicle_with_hint(rng, forced_vehicle_id=forced_vehicle_id)

    def _pick_band_hint(
        self,
        rng: np.random.Generator,
        band_counts: Dict[str, int],
        targets: Dict[str, int],
    ) -> str:
        deficits = {
            band: max(int(targets.get(band, 0)) - int(band_counts.get(band, 0)), 0)
            for band in ["safe", "fail", "critical"]
        }
        if sum(deficits.values()) <= 0:
            return str(rng.choice(["safe", "fail", "critical"], p=[0.3, 0.3, 0.4]))

        bands = list(deficits.keys())
        weights = np.asarray([deficits[b] for b in bands], dtype=np.float64)
        weights = weights / max(float(weights.sum()), 1e-12)
        return bands[int(rng.choice(len(bands), p=weights))]

    def _pick_vehicle_hint(
        self,
        rng: np.random.Generator,
        vehicle_counts: Dict[str, int],
        accepted: int,
    ) -> str | None:
        target_each = max((accepted + 1) / max(len(self.vehicle_library), 1), 1.0)
        deficits = []
        ids = []
        for vehicle in self.vehicle_library:
            ids.append(vehicle.vehicle_id)
            deficits.append(max(target_each - float(vehicle_counts.get(vehicle.vehicle_id, 0)), 0.0))

        weights = np.asarray(deficits, dtype=np.float64)
        if float(weights.sum()) <= 1e-9:
            return None

        weights = weights / float(weights.sum())
        return ids[int(rng.choice(len(ids), p=weights))]

    def _sample_terrain_class_for_band(
        self,
        rng: np.random.Generator,
        band_hint: str | None,
    ) -> Dict[str, Any]:
        terrain_rows, terrain_probs = weighted_table(self.terrain_cfg["terrain"]["classes"])

        if band_hint == "safe":
            preferred = {"flat": 0.55, "uniform_slope": 0.25, "lateral_slope": 0.10, "waves": 0.10}
        elif band_hint == "fail":
            preferred = {"steps": 0.25, "pits": 0.20, "bumps": 0.15, "slope_bumps": 0.20, "mixed_random": 0.20}
        elif band_hint == "critical":
            preferred = {
                "uniform_slope": 0.15,
                "lateral_slope": 0.15,
                "steps": 0.15,
                "pits": 0.15,
                "bumps": 0.15,
                "waves": 0.10,
                "slope_bumps": 0.10,
                "lateral_pits": 0.05,
            }
        else:
            idx = int(rng.choice(len(terrain_rows), p=terrain_probs))
            return terrain_rows[idx]

        names = [str(row["name"]) for row in terrain_rows]
        weights = np.zeros(len(terrain_rows), dtype=np.float64)
        for i, name in enumerate(names):
            weights[i] = float(preferred.get(name, 0.0))

        if float(weights.sum()) <= 1e-12:
            idx = int(rng.choice(len(terrain_rows), p=terrain_probs))
            return terrain_rows[idx]

        weights = weights / float(weights.sum())
        return terrain_rows[int(rng.choice(len(terrain_rows), p=weights))]

    def _sample_friction_for_band(
        self,
        rng: np.random.Generator,
        band_hint: str | None,
    ) -> tuple[str, float]:
        friction_rows, friction_probs = weighted_table(self.friction_cfg["friction"]["classes"])

        if band_hint == "safe":
            preferred = {"dry_hard": 0.75, "grass_soft": 0.20, "mixed": 0.05}
        elif band_hint == "fail":
            preferred = {"wet_muddy": 0.55, "grass_soft": 0.25, "mixed": 0.20}
        elif band_hint == "critical":
            preferred = {"grass_soft": 0.45, "wet_muddy": 0.30, "mixed": 0.20, "dry_hard": 0.05}
        else:
            preferred = None

        if preferred is None:
            idx = int(rng.choice(len(friction_rows), p=friction_probs))
        else:
            weights = np.asarray([float(preferred.get(str(row["name"]), 0.0)) for row in friction_rows], dtype=np.float64)
            if float(weights.sum()) <= 1e-12:
                weights = np.asarray(friction_probs, dtype=np.float64)
            weights = weights / float(weights.sum())
            idx = int(rng.choice(len(friction_rows), p=weights))

        friction_class = friction_rows[idx]
        friction_name = str(friction_class["name"])
        mu_lo, mu_hi = friction_class["mu_range"]
        mu_lo = float(mu_lo)
        mu_hi = float(mu_hi)

        if band_hint == "safe":
            lo = max(mu_lo, mu_hi - 0.25 * (mu_hi - mu_lo))
            friction_mu = float(rng.uniform(lo, mu_hi))
        elif band_hint == "fail":
            hi = min(mu_hi, mu_lo + 0.35 * (mu_hi - mu_lo))
            friction_mu = float(rng.uniform(mu_lo, hi))
        else:
            friction_mu = float(rng.uniform(mu_lo, mu_hi))

        return friction_name, friction_mu

    def _sample_action(self, rng: np.random.Generator, motion_model: str) -> ActionPrimitive:
        # Main dataset path excludes in-place rotate actions (a3/a4); keep forward primitives only.
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
