from __future__ import annotations

import numpy as np
import pytest

from ml25d_dataset_generation.pso_training import _loss
from ml25d_dataset_generation.risk_model import decode_particle
from ml25d_dataset_generation.risk_planner import evaluate_proxy_astar, fuse_risk
from ml25d_dataset_generation.training_data import make_split, make_stratified_split


def test_make_split_covers_all_indices() -> None:
    split = make_split(100, seed=7)
    merged = np.concatenate([split.train, split.val, split.test])
    assert sorted(merged.tolist()) == list(range(100))
    assert len(set(merged.tolist())) == 100


def test_stratified_split_preserves_classes() -> None:
    labels = np.array(["safe"] * 30 + ["fail"] * 30 + ["critical"] * 40)
    split = make_stratified_split(labels, seed=4)
    assert set(labels[split.train]) == {"safe", "fail", "critical"}
    assert set(labels[split.val]) == {"safe", "fail", "critical"}
    assert set(labels[split.test]) == {"safe", "fail", "critical"}


def test_particle_decoding_outputs_valid_weights() -> None:
    particle = np.linspace(0.1, 0.9, 20, dtype=np.float32)
    model_cfg, train_cfg, loss_weights, fusion_weights, thresholds = decode_particle(particle)
    assert len(model_cfg.conv_channels) == 3
    assert 1e-4 <= train_cfg["lr"] <= 1e-2
    assert all(v > 0.0 for v in loss_weights.values())
    assert abs(sum(fusion_weights) - 1.0) < 1e-6
    assert 0.5 <= thresholds["edge"] <= 0.95


def test_loss_uses_continuous_bottom_probability() -> None:
    torch = pytest.importorskip("torch")
    logits = torch.zeros((2, 7), dtype=torch.float32)
    logits[:, 5] = torch.tensor([-2.0, 2.0])
    y = torch.zeros((2, 7), dtype=torch.float32)
    y[:, 5] = torch.tensor([0.1, 0.9])
    weights = {"fail": 0.0, "bottom": 1.0, "stuck": 0.0, "risk": 0.0}

    expected = torch.nn.functional.binary_cross_entropy_with_logits(logits[:, 5], y[:, 5])
    assert torch.allclose(_loss(torch, logits, y, weights), expected)


def test_proxy_astar_evaluation_is_bounded() -> None:
    rng = np.random.default_rng(3)
    y_true = rng.uniform(0.0, 0.5, size=(300, 7)).astype(np.float32)
    y_pred = y_true.copy()
    y_true[:, 0] = (np.max(y_true[:, 1:], axis=1) > 0.35).astype(np.float32)
    y_pred[:, 0] = y_true[:, 0]
    weights = [1.0 / 7.0] * 7
    risks = fuse_risk(y_true, weights)
    assert risks.shape == (300,)
    result = evaluate_proxy_astar(
        y_true=y_true,
        y_pred=y_pred,
        fusion_weights=weights,
        thresholds={"edge": 0.8, "path_max": 0.8, "path_avg": 0.5},
        seed=9,
        num_tasks=4,
        grid_size=6,
    )
    assert 0.0 <= result.plan_success_rate <= 1.0
    assert result.total_tasks == 4
