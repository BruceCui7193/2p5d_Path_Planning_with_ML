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
    bottom_fail_threshold: float = 0.05
    bottom_fail_min_duration_sec: float = 0.05
    bottom_clearance_threshold_m: float = 0.008
    bottom_contact_min_duration_ratio: float = 0.05
    roll_pitch_quantile: float = 95.0
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
            bottom_fail_threshold=float(labels_cfg.get("bottom_fail_threshold", 0.05)),
            bottom_fail_min_duration_sec=float(labels_cfg.get("bottom_fail_min_duration_sec", 0.05)),
            bottom_clearance_threshold_m=float(labels_cfg.get("bottom_clearance_threshold_m", 0.008)),
            bottom_contact_min_duration_ratio=float(labels_cfg.get("bottom_contact_min_duration_ratio", 0.05)),
            roll_pitch_quantile=float(labels_cfg.get("roll_pitch_quantile", 95.0)),
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
        bottom_metrics = self.bottom_metrics(trajectory)
        p_bottom = float(bottom_metrics["bottom_duration_ratio"])
        p_stuck = self._p_stuck(trajectory, action)
        bottom_fail = bool(
            float(bottom_metrics["bottom_duration_sec"]) >= float(self.thresholds.bottom_fail_min_duration_sec)
            and p_bottom >= float(self.thresholds.bottom_fail_threshold)
        )

        y_fail = float(
            (q_roll > self.thresholds.roll_fail_threshold)
            or (q_pitch > self.thresholds.pitch_fail_threshold)
            or (q_slip > self.thresholds.slip_fail_threshold)
            or (q_lift > self.thresholds.lift_fail_threshold)
            or bottom_fail
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

    @staticmethod
    def _safe_abs_percentile(arr: np.ndarray, q: float) -> float:
        if arr.size == 0:
            return 0.0
        return float(np.percentile(np.abs(arr), float(np.clip(q, 50.0, 100.0))))

    def _q_roll(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        phi_max = np.deg2rad(max(vehicle.phi_max_deg, 1e-6))
        roll_p = self._safe_abs_percentile(trajectory.roll_rad, self.thresholds.roll_pitch_quantile)
        return float(np.clip(roll_p / phi_max, 0.0, 1.0))

    def _q_pitch(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        theta_max = np.deg2rad(max(vehicle.theta_max_deg, 1e-6))
        pitch_p = self._safe_abs_percentile(trajectory.pitch_rad, self.thresholds.roll_pitch_quantile)
        return float(np.clip(pitch_p / theta_max, 0.0, 1.0))

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
            robust_slip = float(np.percentile(np.clip(slip[valid], 0.0, 1.5), 70))
        else:
            robust_slip = 0.0
        return float(np.clip(robust_slip / max(self.thresholds.slip_normalizer, 1e-6), 0.0, 1.0))

    def _q_lift(self, trajectory: SimulationTrajectory, vehicle: VehicleParams) -> float:
        lift_valid = trajectory.wheel_lift_valid_mask
        lift_state = trajectory.wheel_lift_state
        if lift_valid is not None and lift_state is not None:
            valid = np.asarray(lift_valid, dtype=bool)
            lifted = np.asarray(lift_state, dtype=bool)
            if valid.size == 0:
                return 0.0
            valid_per_frame = np.sum(valid, axis=1)
            usable = valid_per_frame > 0
            if not np.any(usable):
                return 0.0
            lifted_per_frame = np.sum(lifted & valid, axis=1)
            lift_ratio = np.zeros(valid.shape[0], dtype=np.float32)
            lift_ratio[usable] = lifted_per_frame[usable] / np.maximum(valid_per_frame[usable], 1)
            return float(np.clip(np.mean(lift_ratio[usable]), 0.0, 1.0))

        if trajectory.wheel_contact_forces.size == 0:
            return 0.0
        f_min = self.thresholds.contact_force_ratio * (vehicle.m * 9.81 / 4.0)
        forces = np.asarray(trajectory.wheel_contact_forces, dtype=np.float32)
        observed = trajectory.wheel_contact_observed
        if observed is None:
            observed_mask = forces > 1e-3
        else:
            observed_mask = np.asarray(observed, dtype=bool)
        observed_ratio = float(np.mean(observed_mask))
        if observed_ratio < 0.08:
            return 0.0
        lifted = (forces < f_min) & observed_mask
        return float(np.clip(np.sum(lifted) / max(np.sum(observed_mask), 1), 0.0, 1.0))

    def bottom_metrics(self, trajectory: SimulationTrajectory) -> Dict[str, float]:
        contact_ratio_raw = 0.0
        if trajectory.chassis_contacts.size > 0:
            contact_ratio_raw = float(np.clip(np.mean(trajectory.chassis_contacts > 0), 0.0, 1.0))
        contact_ratio = (
            contact_ratio_raw
            if contact_ratio_raw >= self.thresholds.bottom_contact_min_duration_ratio
            else 0.0
        )

        clearance_ratio = 0.0
        min_clearance_mean = float("nan")
        min_clearance_p5 = float("nan")
        min_clearance_min = float("nan")
        if trajectory.chassis_min_clearance_m is not None and trajectory.chassis_min_clearance_m.size > 0:
            clearance = np.asarray(trajectory.chassis_min_clearance_m, dtype=np.float32)
            clearance_ratio = float(
                np.clip(
                    np.mean(clearance < self.thresholds.bottom_clearance_threshold_m),
                    0.0,
                    1.0,
                )
            )
            min_clearance_mean = float(np.mean(clearance))
            min_clearance_p5 = float(np.percentile(clearance, 5))
            min_clearance_min = float(np.min(clearance))

        bottom_duration_ratio = float(np.clip(max(clearance_ratio, contact_ratio), 0.0, 1.0))
        duration_sec = self._trajectory_duration_sec(trajectory)
        bottom_duration_sec = float(bottom_duration_ratio * duration_sec)
        return {
            "bottom_duration_ratio": bottom_duration_ratio,
            "bottom_duration_sec": bottom_duration_sec,
            "bottom_clearance_duration_ratio": clearance_ratio,
            "bottom_contact_duration_ratio_raw": contact_ratio_raw,
            "bottom_contact_duration_ratio": contact_ratio,
            "min_clearance_mean": min_clearance_mean,
            "min_clearance_p5": min_clearance_p5,
            "min_clearance_min": min_clearance_min,
        }

    @staticmethod
    def _trajectory_duration_sec(trajectory: SimulationTrajectory) -> float:
        ts = np.asarray(trajectory.timestamps, dtype=np.float64)
        if ts.size <= 1:
            return 0.0
        dt = np.diff(ts)
        dt = dt[np.isfinite(dt) & (dt > 0.0)]
        if dt.size == 0:
            return 0.0
        return float(np.median(dt) * ts.size)

    def _p_bottom(self, trajectory: SimulationTrajectory) -> float:
        metrics = self.bottom_metrics(trajectory)
        return float(metrics["bottom_duration_ratio"])

    @staticmethod
    def translation_progress(trajectory: SimulationTrajectory, action: ActionPrimitive) -> float:
        if action.delta_s_m <= 1e-4:
            return float("nan")
        return float(trajectory.completed_displacement_m / max(action.delta_s_m, 1e-6))

    @staticmethod
    def angular_progress(
        trajectory: SimulationTrajectory,
        action: ActionPrimitive,
        *,
        clip_to_one: bool = False,
    ) -> float:
        target_yaw = abs(np.deg2rad(action.delta_psi_deg))
        if target_yaw <= 1e-6:
            return float("nan")
        value = float(trajectory.completed_heading_change_rad / target_yaw)
        if clip_to_one:
            return float(np.clip(value, 0.0, 1.0))
        return value

    def _p_stuck(self, trajectory: SimulationTrajectory, action: ActionPrimitive) -> float:
        if action.delta_s_m > 1e-4:
            t_progress = self.translation_progress(trajectory, action)
            if np.isnan(t_progress):
                return 0.0
            return float(1.0 if t_progress < self.thresholds.stuck_displacement_ratio else 0.0)

        a_progress = self.angular_progress(trajectory, action, clip_to_one=False)
        if np.isnan(a_progress):
            return 0.0
        return float(1.0 if a_progress < 0.5 else 0.0)

    @staticmethod
    def classify_sample_band(label: LabelOutput) -> str:
        max_risk = max(label.q_roll, label.q_pitch, label.q_slip, label.q_lift, label.p_bottom, label.p_stuck)

        if label.y_fail < 0.5 and max_risk < 0.6:
            return "safe"
        if label.y_fail >= 0.5 and (label.p_bottom >= 0.8 or label.p_stuck >= 1.0 or max_risk >= 0.95):
            return "fail"
        return "critical"
