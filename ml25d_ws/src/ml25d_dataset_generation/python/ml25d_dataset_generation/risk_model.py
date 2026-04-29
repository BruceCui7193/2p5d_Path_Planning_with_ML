from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class RiskModelConfig:
    conv_channels: tuple[int, int, int] = (16, 32, 64)
    param_hidden: int = 64
    fusion_hidden: int = 128
    dropout: float = 0.10


def require_torch():
    try:
        import torch
        import torch.nn as nn
    except Exception as exc:
        raise RuntimeError(
            "PyTorch is required for CNN+MLP risk training. Install it in the project venv, for example:\n"
            "  .venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128\n"
            "or use the CPU wheel if CUDA is not needed."
        ) from exc
    return torch, nn


def build_model(config: RiskModelConfig, param_dim: int = 17):
    torch, nn = require_torch()

    class RiskCNNMLP(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            c1, c2, c3 = config.conv_channels
            self.map_encoder = nn.Sequential(
                nn.Conv2d(6, c1, kernel_size=3, padding=1),
                nn.BatchNorm2d(c1),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(c1, c2, kernel_size=3, padding=1),
                nn.BatchNorm2d(c2),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(c2, c3, kernel_size=3, padding=1),
                nn.BatchNorm2d(c3),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
            )
            self.param_encoder = nn.Sequential(
                nn.Linear(param_dim, config.param_hidden),
                nn.ReLU(inplace=True),
                nn.Dropout(config.dropout),
                nn.Linear(config.param_hidden, config.param_hidden),
                nn.ReLU(inplace=True),
            )
            self.head = nn.Sequential(
                nn.Linear(c3 + config.param_hidden, config.fusion_hidden),
                nn.ReLU(inplace=True),
                nn.Dropout(config.dropout),
                nn.Linear(config.fusion_hidden, 7),
            )

        def forward(self, x_map, param):
            z_map = self.map_encoder(x_map)
            z_param = self.param_encoder(param)
            return self.head(torch.cat([z_map, z_param], dim=1))

    return RiskCNNMLP()


def decode_particle(
    values: Sequence[float],
) -> tuple[RiskModelConfig, dict[str, float], dict[str, float], list[float], dict[str, float]]:
    if len(values) < 17:
        raise ValueError("particle must have at least 17 values")

    conv_options = [(8, 16, 32), (16, 32, 64), (24, 48, 96), (32, 64, 128)]
    hidden_options = [32, 64, 96, 128]

    conv = conv_options[int(round(values[0])) % len(conv_options)]
    param_hidden = hidden_options[int(round(values[1])) % len(hidden_options)]
    fusion_hidden = hidden_options[int(round(values[2])) % len(hidden_options)]
    dropout = float(_clip(values[3], 0.0, 0.35))
    lr = float(10 ** _clip(values[4], -4.0, -2.0))
    weight_decay = float(10 ** _clip(values[5], -6.0, -3.0))

    loss_weights = {
        "fail": float(_clip(values[6], 0.5, 5.0)),
        "bottom": float(_clip(values[7], 0.5, 5.0)),
        "stuck": float(_clip(values[8], 0.5, 5.0)),
        "risk": float(_clip(values[9], 0.5, 5.0)),
    }
    raw_fusion = [max(float(v), 1e-6) for v in values[10:17]]
    total = sum(raw_fusion)
    fusion_weights = [v / total for v in raw_fusion]
    thresholds = {
        "edge": float(_clip(values[17] if len(values) > 17 else 0.75, 0.4, 0.9)),
        "path_max": float(_clip(values[18] if len(values) > 18 else 0.8, 0.4, 0.9)),
        "path_avg": float(_clip(values[19] if len(values) > 19 else 0.45, 0.1, 0.5)),
    }
    model_cfg = RiskModelConfig(conv_channels=conv, param_hidden=param_hidden, fusion_hidden=fusion_hidden, dropout=dropout)
    train_cfg = {"lr": lr, "weight_decay": weight_decay}
    return model_cfg, train_cfg, loss_weights, fusion_weights, thresholds


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))
