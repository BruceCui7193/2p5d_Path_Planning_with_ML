from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import yaml

from .common_types import ActionPrimitive, VehicleParams


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as fp:
        return yaml.safe_load(fp)


def load_all_configs(config_dir: Path) -> Dict[str, Any]:
    config_dir = config_dir.resolve()
    return {
        "dataset": _load_yaml(config_dir / "dataset_config.yaml"),
        "vehicles": _load_yaml(config_dir / "vehicle_params.yaml"),
        "terrain": _load_yaml(config_dir / "terrain_distribution.yaml"),
        "friction": _load_yaml(config_dir / "friction_table.yaml"),
        "actions": _load_yaml(config_dir / "action_primitives.yaml"),
        "labels": _load_yaml(config_dir / "label_thresholds.yaml"),
    }


def build_vehicle_library(vehicle_cfg: Dict[str, Any]) -> List[VehicleParams]:
    result: List[VehicleParams] = []
    for row in vehicle_cfg["vehicles"]["base_types"]:
        result.append(
            VehicleParams(
                vehicle_id=row["id"],
                L=float(row["L"]),
                W=float(row["W"]),
                l=float(row["l"]),
                b=float(row["b"]),
                r_w=float(row["r_w"]),
                c_g=float(row["c_g"]),
                m=float(row["m"]),
                z_c=float(row["z_c"]),
                phi_max_deg=float(row["phi_max_deg"]),
                theta_max_deg=float(row["theta_max_deg"]),
                alpha_max_deg=float(row["alpha_max_deg"]),
                F_max=float(row["F_max"]),
            )
        )
    return result


def build_action_library(action_cfg: Dict[str, Any]) -> List[ActionPrimitive]:
    result: List[ActionPrimitive] = []
    for row in action_cfg["actions"]["primitives"]:
        result.append(
            ActionPrimitive(
                action_id=row["id"],
                name=row["name"],
                delta_s_m=float(row["delta_s_m"]),
                delta_psi_deg=float(row["delta_psi_deg"]),
                skid_cmd={
                    "v_cmd_mps": float(row["skid_cmd"]["v_cmd_mps"]),
                    "omega_cmd_rps": float(row["skid_cmd"]["omega_cmd_rps"]),
                },
                ackermann_cmd={
                    "v_cmd_mps": float(row["ackermann_cmd"]["v_cmd_mps"]),
                    "steering_angle_deg": float(row["ackermann_cmd"]["steering_angle_deg"]),
                },
            )
        )
    return result


def weighted_table(items: Iterable[Dict[str, Any]], key: str = "weight") -> Tuple[List[Dict[str, Any]], List[float]]:
    rows = list(items)
    weights = [float(row[key]) for row in rows]
    total = sum(weights)
    if total <= 0.0:
        raise ValueError("weights must sum to a positive number")
    return rows, [w / total for w in weights]
