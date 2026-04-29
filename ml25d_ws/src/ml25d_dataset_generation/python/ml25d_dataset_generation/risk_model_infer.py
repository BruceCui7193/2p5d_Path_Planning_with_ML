from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .common_types import ActionPrimitive, VehicleParams
from .config_loader import build_action_library, build_vehicle_library, load_all_configs
from .planning_types import PlannerThresholds
from .risk_model import RiskModelConfig, build_model, require_torch
from .risk_planner import fuse_risk
from .sample_packager import SamplePackager


def _resolve_device(torch, device: str) -> str:
    if device == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    if str(device).startswith("cuda") and (not torch.cuda.is_available()):
        return "cpu"
    return str(device)


@dataclass(frozen=True)
class EdgeRiskPrediction:
    y_hat: np.ndarray
    edge_risk: float


class RiskModelInfer:
    def __init__(
        self,
        checkpoint_path: str | Path,
        config_dir: str | Path,
        device: str = "auto",
    ) -> None:
        torch, _ = require_torch()

        self.checkpoint_path = Path(checkpoint_path).resolve()
        if not self.checkpoint_path.exists():
            raise FileNotFoundError(f"checkpoint not found: {self.checkpoint_path}")

        cfg = load_all_configs(Path(config_dir).resolve())
        self.dataset_cfg = cfg["dataset"]
        self.vehicle_cfg = cfg["vehicles"]
        self.action_cfg = cfg["actions"]
        self.packager = SamplePackager(self.dataset_cfg["map"], self.vehicle_cfg)
        self.vehicle_by_id = {v.vehicle_id: v for v in build_vehicle_library(self.vehicle_cfg)}
        self.action_by_id = {a.action_id: a for a in build_action_library(self.action_cfg)}

        self.torch = torch
        self.device = _resolve_device(torch, device)

        ckpt = torch.load(str(self.checkpoint_path), map_location="cpu")
        model_cfg_raw = ckpt.get("model_config", {})
        model_cfg = RiskModelConfig(
            conv_channels=tuple(model_cfg_raw.get("conv_channels", (16, 32, 64))),
            param_hidden=int(model_cfg_raw.get("param_hidden", 64)),
            fusion_hidden=int(model_cfg_raw.get("fusion_hidden", 128)),
            dropout=float(model_cfg_raw.get("dropout", 0.1)),
        )
        self.model = build_model(model_cfg).to(self.device)
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()

        self.channel_stats = ckpt["channel_stats"]
        mean = np.asarray(self.channel_stats["mean"], dtype=np.float32).reshape(1, 1, -1)
        std = np.asarray(self.channel_stats["std"], dtype=np.float32).reshape(1, 1, -1)
        self._mean = mean
        self._std = np.maximum(std, 1e-6)

        self.fusion_weights = [float(v) for v in ckpt.get("fusion_weights", [1.0] * 7)]
        thresholds = ckpt.get("thresholds", {})
        self.thresholds = PlannerThresholds(
            edge_safe=float(thresholds.get("edge", 0.75)),
            path_max_safe=float(thresholds.get("path_max", 0.85)),
            path_avg_safe=float(thresholds.get("path_avg", 0.45)),
        )

    def get_vehicle(self, vehicle_id: str) -> VehicleParams:
        if vehicle_id not in self.vehicle_by_id:
            raise KeyError(f"unknown vehicle_id: {vehicle_id}")
        return self.vehicle_by_id[vehicle_id]

    def get_action(self, action_id: str) -> ActionPrimitive:
        if action_id not in self.action_by_id:
            raise KeyError(f"unknown action_id: {action_id}")
        return self.action_by_id[action_id]

    def predict_edge(
        self,
        local_heightmap: np.ndarray,
        heading_rad: float,
        vehicle: VehicleParams,
        action: ActionPrimitive,
        friction_mu: float,
    ) -> EdgeRiskPrediction:
        if local_heightmap.ndim != 2:
            raise ValueError(f"local_heightmap must be 2D, got shape {local_heightmap.shape}")

        x_map = self.packager.feature_builder.build_feature_patch(
            local_heightmap.astype(np.float32),
            heading_rad=float(heading_rad),
            vehicle=vehicle,
            action=action,
        )
        x_map = ((x_map - self._mean) / self._std).astype(np.float32)
        theta_v = self.packager.normalize_vehicle(vehicle).astype(np.float32)
        act = self.packager.encode_action(action).astype(np.float32)
        mu = np.array([float(friction_mu)], dtype=np.float32)
        param = np.concatenate([theta_v, act, mu], axis=0).astype(np.float32)

        torch = self.torch
        with torch.inference_mode():
            x_tensor = torch.from_numpy(np.transpose(x_map, (2, 0, 1))[None, ...]).to(self.device)
            p_tensor = torch.from_numpy(param[None, ...]).to(self.device)
            logits = self.model(x_tensor, p_tensor)
            y_hat = torch.sigmoid(logits).detach().cpu().numpy().astype(np.float32)[0]
        edge_risk = float(fuse_risk(y_hat[None, :], self.fusion_weights)[0])
        return EdgeRiskPrediction(y_hat=y_hat, edge_risk=edge_risk)

    def debug_thresholds(self) -> dict[str, Any]:
        return {
            "edge_safe": float(self.thresholds.edge_safe),
            "path_max_safe": float(self.thresholds.path_max_safe),
            "path_avg_safe": float(self.thresholds.path_avg_safe),
        }
