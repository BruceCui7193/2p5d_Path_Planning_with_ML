from __future__ import annotations

import numpy as np

from ml25d_dataset_generation.common_types import ActionPrimitive, SimulationTrajectory, VehicleParams
from ml25d_dataset_generation.label_extractor import LabelExtractor


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


def _action_forward() -> ActionPrimitive:
    return ActionPrimitive(
        action_id="a0",
        name="forward",
        delta_s_m=0.30,
        delta_psi_deg=0.0,
        skid_cmd={"v_cmd_mps": 0.15, "omega_cmd_rps": 0.0},
        ackermann_cmd={"v_cmd_mps": 0.15, "steering_angle_deg": 0.0},
    )


def _action_rotate() -> ActionPrimitive:
    return ActionPrimitive(
        action_id="a3",
        name="rotate_left_in_place",
        delta_s_m=0.0,
        delta_psi_deg=22.5,
        skid_cmd={"v_cmd_mps": 0.0, "omega_cmd_rps": 0.25},
        ackermann_cmd={"v_cmd_mps": 0.04, "steering_angle_deg": 28.0},
    )


def _threshold_cfg() -> dict:
    return {
        "labels": {
            "roll_fail_threshold": 0.9,
            "pitch_fail_threshold": 0.9,
            "slip_fail_threshold": 0.8,
            "lift_fail_threshold": 0.25,
            "stuck_displacement_ratio": 0.3,
            "slip_normalizer": 0.5,
            "contact_force_ratio": 0.05,
        }
    }


def _safe_trajectory() -> SimulationTrajectory:
    n = 100
    return SimulationTrajectory(
        timestamps=np.linspace(0.0, 2.0, n, dtype=np.float32),
        positions_xy=np.column_stack([np.linspace(0.0, 0.28, n), np.zeros(n)]).astype(np.float32),
        yaw_rad=np.zeros(n, dtype=np.float32),
        roll_rad=np.full(n, np.deg2rad(2.0), dtype=np.float32),
        pitch_rad=np.full(n, np.deg2rad(3.0), dtype=np.float32),
        commanded_linear_speed=np.full(n, 0.15, dtype=np.float32),
        actual_linear_speed=np.full(n, 0.14, dtype=np.float32),
        commanded_angular_speed=np.zeros(n, dtype=np.float32),
        actual_angular_speed=np.zeros(n, dtype=np.float32),
        wheel_contact_forces=np.full((n, 4), 30.0, dtype=np.float32),
        chassis_contacts=np.zeros(n, dtype=np.uint8),
        completed_displacement_m=0.28,
        completed_heading_change_rad=0.0,
    )


def test_safe_label() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    labels, band = extractor.compute_labels(_safe_trajectory(), _vehicle(), _action_forward())

    assert labels.y_fail == 0.0
    assert band in {"safe", "critical"}
    assert labels.q_roll < 0.5
    assert labels.q_pitch < 0.5


def test_bottom_contact_failure() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.chassis_contacts[:] = 1

    labels, band = extractor.compute_labels(traj, _vehicle(), _action_forward())
    assert labels.p_bottom == 1.0
    assert labels.y_fail == 1.0
    assert band in {"fail", "critical"}


def test_bottom_clearance_failure_without_contact() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.chassis_contacts[:] = 0
    n = traj.timestamps.shape[0]
    clearance = np.full(n, 0.02, dtype=np.float32)
    clearance[n // 2 :] = -0.03
    traj.chassis_min_clearance_m = clearance

    labels, band = extractor.compute_labels(traj, _vehicle(), _action_forward())
    assert labels.p_bottom >= 0.5
    assert labels.y_fail == 1.0
    assert band in {"fail", "critical"}


def test_rotation_action_ignores_linear_slip() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.commanded_linear_speed[:] = 0.04
    traj.actual_linear_speed[:] = 0.0
    traj.completed_heading_change_rad = np.deg2rad(22.5)

    labels, _ = extractor.compute_labels(traj, _vehicle(), _action_rotate())
    assert labels.q_slip == 0.0


def test_rotation_stuck_uses_angular_progress() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.completed_displacement_m = 0.40
    traj.completed_heading_change_rad = np.deg2rad(5.0)

    labels, _ = extractor.compute_labels(traj, _vehicle(), _action_rotate())
    assert labels.p_stuck == 1.0


def test_rotation_not_stuck_when_heading_reached() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.completed_displacement_m = 0.0
    traj.completed_heading_change_rad = np.deg2rad(22.5)

    labels, _ = extractor.compute_labels(traj, _vehicle(), _action_rotate())
    assert labels.p_stuck == 0.0


def test_lift_ignores_sparse_contact_observations() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.wheel_contact_forces[:] = 0.0
    traj.wheel_contact_forces[0, :] = 40.0

    labels, _ = extractor.compute_labels(traj, _vehicle(), _action_forward())
    assert labels.q_lift == 0.0
    assert labels.y_fail == 0.0


def test_lift_detects_dense_low_wheel_force() -> None:
    extractor = LabelExtractor(_threshold_cfg())
    traj = _safe_trajectory()
    traj.wheel_contact_forces[:] = 0.5

    labels, _ = extractor.compute_labels(traj, _vehicle(), _action_forward())
    assert labels.q_lift > 0.9
    assert labels.y_fail == 1.0
