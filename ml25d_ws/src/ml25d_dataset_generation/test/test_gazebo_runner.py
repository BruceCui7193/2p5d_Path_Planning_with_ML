from __future__ import annotations

import numpy as np

from ml25d_dataset_generation.common_types import VehicleParams
from ml25d_dataset_generation.gazebo_runner import RosGzSimulationRunner, _ContactBuffer


def _vehicle() -> VehicleParams:
    return VehicleParams(
        vehicle_id="test",
        L=0.65,
        W=0.45,
        l=0.48,
        b=0.36,
        r_w=0.10,
        c_g=0.08,
        m=14.0,
        z_c=0.20,
        phi_max_deg=22.0,
        theta_max_deg=25.0,
        alpha_max_deg=45.0,
        F_max=120.0,
    )


def test_ros_gz_terrain_uses_mesh_not_column_grid() -> None:
    runner = RosGzSimulationRunner()
    heightmap = np.zeros((31, 31), dtype=np.float32)
    heightmap[14:17, 14:17] = 0.05

    sdf = runner._terrain_sdf("terrain_test", heightmap, friction_mu=0.8)
    assert "<mesh>" in sdf
    assert "file:///tmp/ml25d_ros_gz_meshes/" in sdf
    assert "cell_0_0" not in sdf


def test_ros_gz_vehicle_start_height_uses_local_footprint() -> None:
    runner = RosGzSimulationRunner()
    heightmap = np.zeros((31, 31), dtype=np.float32)
    heightmap[15, 15] = 0.07

    start_z = runner._vehicle_start_z(heightmap, _vehicle())
    assert 0.09 <= start_z <= 0.11


def test_ros_gz_vehicle_sdf_has_contact_sensors() -> None:
    runner = RosGzSimulationRunner()
    sdf = runner._vehicle_sdf("ml25d_vehicle", _vehicle())

    assert "chassis_contacts" in sdf
    assert "front_left_contacts" in sdf
    assert "front_right_contacts" in sdf
    assert "rear_left_contacts" in sdf
    assert "rear_right_contacts" in sdf
    assert "<cylinder>" in sdf
    assert "front_left_wheel_joint" in sdf
    assert "rear_right_wheel_joint" in sdf
    assert "gz-sim-diff-drive-system" in sdf


def test_contact_buffer_extracts_wrench_magnitude() -> None:
    class Force:
        x = 3.0
        y = 4.0
        z = 12.0

    class WrenchValue:
        force = Force()

    class Wrench:
        body_1_wrench = WrenchValue()
        body_2_wrench = WrenchValue()

    class Contact:
        wrenches = [Wrench()]

    assert np.isclose(_ContactBuffer._contact_force_magnitude(Contact()), 13.0)
