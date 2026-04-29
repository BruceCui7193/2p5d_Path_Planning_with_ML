from __future__ import annotations

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, VehicleParams
from ml25d_dataset_generation.map_builder_from_h5 import H5PlanningMapBuilder
from ml25d_dataset_generation.planning_types import PlanningScene
from ml25d_dataset_generation.risk_astar import METHOD_BASELINE_1, PlannerConfig, plan_path
from ml25d_dataset_generation.training_data import RiskDatasetArrays


def _vehicle() -> VehicleParams:
    return VehicleParams(
        vehicle_id="urban_small",
        L=0.45,
        W=0.28,
        l=0.25,
        b=0.22,
        r_w=0.07,
        c_g=0.04,
        m=8.0,
        z_c=0.09,
        phi_max_deg=24.0,
        theta_max_deg=22.0,
        alpha_max_deg=28.0,
        F_max=70.0,
    )


def test_map_builder_builds_expected_scene_shape() -> None:
    n = 6
    x_map = np.zeros((n, 31, 31, 6), dtype=np.float32)
    for i in range(n):
        x_map[i, :, :, 0] = np.linspace(-0.02, 0.02, 31, dtype=np.float32)[:, None]
    theta_v = np.zeros((n, 12), dtype=np.float32)
    action = np.zeros((n, 4), dtype=np.float32)
    mu = np.full((n, 1), 0.8, dtype=np.float32)
    y = np.zeros((n, 7), dtype=np.float32)
    band = np.array(["safe"] * n)
    metadata = [
        {
            "terrain_class": "flat",
            "heading_rad": float(i) * 0.1,
            "friction_mu": 0.8,
        }
        for i in range(n)
    ]
    data = RiskDatasetArrays(x_map=x_map, theta_v=theta_v, action=action, mu=mu, y=y, band=band, metadata=metadata)
    builder = H5PlanningMapBuilder(data=data, patch_size=31, resolution_m=0.1, heading_bins=16, seed=0)
    scene = builder.build_scene("flat", scene_index=1, global_size=61, overlap_cells=15)
    assert scene.heightmap.shape == (61, 61)
    assert scene.start_state[0] < scene.goal_state[0]
    assert scene.heading_bins == 16


def test_baseline1_astar_finds_path_on_flat() -> None:
    h = np.zeros((31, 31), dtype=np.float32)
    scene = PlanningScene(
        scene_id="flat_test",
        terrain_class="flat",
        heightmap=h,
        resolution_m=0.1,
        friction_mu=0.8,
        start_state=(5, 15, 0),
        goal_state=(25, 15, 0),
        heading_bins=16,
    )
    actions = [
        ActionPrimitive(
            action_id="a0",
            name="forward",
            delta_s_m=0.5,
            delta_psi_deg=0.0,
            skid_cmd={"v_cmd_mps": 0.2, "omega_cmd_rps": 0.0},
            ackermann_cmd={"v_cmd_mps": 0.2, "steering_angle_deg": 0.0},
        )
    ]
    result = plan_path(
        scene=scene,
        vehicle=_vehicle(),
        actions=actions,
        method=METHOD_BASELINE_1,
        config=PlannerConfig(goal_radius_cells=1, max_expansions=2000),
    )
    assert result.found
    assert result.path_length_m > 0.0
    assert len(result.actions) > 0
