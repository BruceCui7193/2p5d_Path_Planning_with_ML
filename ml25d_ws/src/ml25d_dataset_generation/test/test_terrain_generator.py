from __future__ import annotations

import numpy as np

from ml25d_dataset_generation.terrain_generator import TerrainGenerator


def _terrain_cfg() -> dict:
    return {
        "name": "mixed_random",
        "params": {
            "slope_deg": [0.0, 25.0],
            "cross_deg": [0.0, 15.0],
            "wave_amp_m": [0.0, 0.10],
            "wave_len_m": [0.4, 2.0],
            "bump_count": [1, 8],
            "pit_count": [1, 6],
            "step_height_m": [0.0, 0.10],
            "noise_std_m": [0.0, 0.03],
        },
    }


def test_generate_shape_and_dtype() -> None:
    gen = TerrainGenerator(patch_size=31, resolution_m=0.1)
    sample = gen.generate(np.random.default_rng(7), _terrain_cfg())

    assert sample.heightmap.shape == (31, 31)
    assert sample.heightmap.dtype == np.float32


def test_deterministic_seed() -> None:
    gen = TerrainGenerator(patch_size=31, resolution_m=0.1)
    cfg = _terrain_cfg()

    a = gen.generate(np.random.default_rng(1234), cfg)
    b = gen.generate(np.random.default_rng(1234), cfg)

    assert np.allclose(a.heightmap, b.heightmap)
