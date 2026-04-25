from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from .common_types import ActionPrimitive, LabelOutput, SimulationTrajectory, VehicleParams


@dataclass(frozen=True)
class LabelThresholds:
    roll_fail_threshold: float = 0.9
    pitch_fail_threshold: float = 0.9
    slip_fail_threshold: float = 0.8
    lift_fail_threshold: float = 0.25
    bottom_fail_threshold: float = 0.2
    stuck_displacement_ratio: float = 0.3
    slip_normalizer: float = 0.5
    contact_force_ratio: float = 0.05


class LabelExtractor:
    def __init__(self, threshold_cfg: Dict[str, float]) -> None:
        labels_cfg = threshold_cfg["labels"]
        self.thresholds = LabelThresholds(
            roll_fail_threshold=float(labels_cfg["roll_fail_threshold"]),
            pitch_fail_threshold=float(labels_cfg["pitch_fail_threshold"]),
            slip_fail_threshold=float(labels_cfg["slip_fail_threshold"]),
            lift_fail_threshold=float(labels_cfg["lift_fail_threshold"]),
            bottom_fail_threshold=float(labels_cfg.get("bottom_fail_threshold", 0.2)),
            stuck_displacement_ratio=float(labels_cfg["stuck_displacement_ratio"]),
            slip_normalizer=float(labels_cfg["slip_normalizer"]),
            contact_force_ratio=float(labels_cfg["contact_force_ratio"]),
        )

    def compute_labels(
        self,
        trajectory: SimulationTrajectory,
        vehicle: VehicleParams,
        action: ActionPrimitive,
    ) -> Tuple[LabelOutput, str]:
        q_roll = self._q_roll(trajectory, vehicle)
        q_pitch = self._q_pitch(trajectory, vehicle)
        q_slip = self._q_slip(trajectory, action)
        q_lift = self._q_lift(trajectory, vehicle)
        p_bottom = self._p_bottom(trajectory)
        p_stuck = self._p_stuck(trajectory, action)

        y_fail = float(
            (q_roll > self.thresholds.roll_fail_threshold)
            or (q_pitch > self.thresholds.pitch_fail_threshold)
            or (q_slip > self.thresholds.slip_fail_threshold)
            or (q_lift > self.thresholds.lift_fail_threshold)
            or (p_bottom > self.thresholds.bottom_fail_threshold)
            or (p_stuck >= 1.0)
        )

        label = LabelOutput(
            y_fail=y_fail,
            q_roll=q_roll,
            q_pitch=q_pitch,
            q_slip=q_slip,
            q_lift=q_lift,
            p_bottom=p_bottom,
            p_stuck=p_stuck,
        )
        band = self.classify_sample_band(label)
        return label, band

    @staticmethod
    def _safe_max_abs(arr: np.ndarray) -> float:
        if arr.size == 0:
            return 0.0
        return float(np.max(np.abs(arr)))

    def _q_roll(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        phi_max = np.deg2rad(max(vehicle.phi_max_deg, 1e-6))
        return float(np.clip(self._safe_max_abs(trajectory.roll_rad) / phi_max, 0.0, 1.0))

    def _q_pitch(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        theta_max = np.deg2rad(max(vehicle.theta_max_deg, 1e-6))
        return float(np.clip(self._safe_max_abs(trajectory.pitch_rad) / theta_max, 0.0, 1.0))

    def _q_slip(self, trajectory: SimulationTrajectory, action: ActionPrimitive) -> float:
        if action.delta_s_m <= 1e-4:
            return 0.0

        cmd = np.abs(trajectory.commanded_linear_speed)
        actual = np.abs(trajectory.actual_linear_speed)
        if cmd.size == 0:
            return 0.0

        # Ignore startup transient to avoid inflating slip due to acceleration lag.
        warmup_idx = int(0.2 * cmd.size)
        cmd = cmd[warmup_idx:]
        actual = actual[warmup_idx:]

        valid = cmd > 3e-2
        slip = np.zeros_like(cmd)
        slip[valid] = np.abs(cmd[valid] - actual[valid]) / (cmd[valid] + 1e-6)
        if np.any(valid):
            robust_slip = float(np.percentile(slip[valid], 90))
        else:
            robust_slip = 0.0
        return float(np.clip(robust_slip / max(self.thresholds.slip_normalizer, 1e-6), 0.0, 1.0))

    def _q_lift(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        if trajectory.wheel_contact_forces.size == 0:
            return 0.0
        f_min = self.thresholds.contact_force_ratio * (vehicle.m * 9.81 / 4.0)
        lifted = trajectory.wheel_contact_forces < f_min
        return float(np.clip(np.mean(lifted), 0.0, 1.0))

    @staticmethod
    def _p_bottom(trajectory: SimulationTrajectory) -> float:
        if trajectory.chassis_contacts.size == 0:
            return 0.0
        return float(np.clip(np.mean(trajectory.chassis_contacts > 0), 0.0, 1.0))

    def _p_stuck(self, trajectory: SimulationTrajectory, action: ActionPrimitive) -> float:
        if action.delta_s_m > 1e-4:
            completed_ratio = trajectory.completed_displacement_m / max(action.delta_s_m, 1e-6)
            return float(1.0 if completed_ratio < self.thresholds.stuck_displacement_ratio else 0.0)

        target_yaw = abs(np.deg2rad(action.delta_psi_deg))
        if target_yaw <= 1e-6:
            return 0.0
        return float(1.0 if trajectory.completed_heading_change_rad < 0.5 * target_yaw else 0.0)

    @staticmethod
    def classify_sample_band(label: LabelOutput) -> str:
        max_risk = max(label.q_roll, label.q_pitch, label.q_slip, label.q_lift, label.p_bottom, label.p_stuck)

        if label.y_fail < 0.5 and max_risk < 0.6:
            return "safe"
        if label.y_fail >= 0.5 and (label.p_bottom >= 0.8 or label.p_stuck >= 1.0 or max_risk >= 0.95):
            return "fail"
        return "critical"
