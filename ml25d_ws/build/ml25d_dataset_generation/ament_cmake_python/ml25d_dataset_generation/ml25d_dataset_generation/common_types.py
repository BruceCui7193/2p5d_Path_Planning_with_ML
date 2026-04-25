from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np


VEHICLE_PARAM_ORDER = [
    "L",
    "W",
    "l",
    "b",
    "r_w",
    "c_g",
    "m",
    "z_c",
    "phi_max_deg",
    "theta_max_deg",
    "alpha_max_deg",
    "F_max",
]


@dataclass(frozen=True)
class VehicleParams:
    vehicle_id: str
    L: float
    W: float
    l: float
    b: float
    r_w: float
    c_g: float
    m: float
    z_c: float
    phi_max_deg: float
    theta_max_deg: float
    alpha_max_deg: float
    F_max: float

    def as_vector(self) -> np.ndarray:
        return np.array(
            [
                self.L,
                self.W,
                self.l,
                self.b,
                self.r_w,
                self.c_g,
                self.m,
                self.z_c,
                self.phi_max_deg,
                self.theta_max_deg,
                self.alpha_max_deg,
                self.F_max,
            ],
            dtype=np.float32,
        )


@dataclass(frozen=True)
class ActionPrimitive:
    action_id: str
    name: str
    delta_s_m: float
    delta_psi_deg: float
    skid_cmd: Dict[str, float]
    ackermann_cmd: Dict[str, float]

    def encoded_vector(self) -> np.ndarray:
        delta_psi_rad = np.deg2rad(self.delta_psi_deg)
        return np.array(
            [
                self.delta_s_m,
                delta_psi_rad,
                np.sin(delta_psi_rad),
                np.cos(delta_psi_rad),
            ],
            dtype=np.float32,
        )


@dataclass
class SimulationTrajectory:
    timestamps: np.ndarray
    positions_xy: np.ndarray
    yaw_rad: np.ndarray
    roll_rad: np.ndarray
    pitch_rad: np.ndarray
    commanded_linear_speed: np.ndarray
    actual_linear_speed: np.ndarray
    commanded_angular_speed: np.ndarray
    actual_angular_speed: np.ndarray
    wheel_contact_forces: np.ndarray
    chassis_contacts: np.ndarray
    completed_displacement_m: float
    completed_heading_change_rad: float


@dataclass(frozen=True)
class LabelOutput:
    y_fail: float
    q_roll: float
    q_pitch: float
    q_slip: float
    q_lift: float
    p_bottom: float
    p_stuck: float

    def as_vector(self) -> np.ndarray:
        return np.array(
            [
                self.y_fail,
                self.q_roll,
                self.q_pitch,
                self.q_slip,
                self.q_lift,
                self.p_bottom,
                self.p_stuck,
            ],
            dtype=np.float32,
        )


@dataclass(frozen=True)
class SampleMetadata:
    sample_id: int
    seed: int
    terrain_class: str
    friction_class: str
    vehicle_id: str
    action_id: str
    action_name: str
    motion_model: str
    heading_rad: float

    def as_dict(self) -> Dict[str, Any]:
        return {
            "sample_id": self.sample_id,
            "seed": self.seed,
            "terrain_class": self.terrain_class,
            "friction_class": self.friction_class,
            "vehicle_id": self.vehicle_id,
            "action_id": self.action_id,
            "action_name": self.action_name,
            "motion_model": self.motion_model,
            "heading_rad": self.heading_rad,
        }
