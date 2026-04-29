from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np
from sklearn.metrics import f1_score, precision_score, recall_score, roc_auc_score

from .risk_model import RiskModelConfig, build_model, decode_particle, require_torch
from .risk_planner import evaluate_proxy_astar
from .training_data import RiskDatasetArrays, SplitIndices, compute_channel_stats, normalize_x_map


@dataclass(frozen=True)
class TrainConfig:
    batch_size: int = 128
    pso_epochs: int = 6
    final_epochs: int = 90
    pso_particles: int = 8
    pso_iters: int = 6
    seed: int = 20260425
    device: str = "auto"
    max_pso_train_samples: int = 4096
    loader_num_workers: int = 0
    proxy_tasks_pso: int = 32
    proxy_tasks_final: int = 80
    proxy_eval_seeds_pso: int = 2
    proxy_eval_seeds_final: int = 4
    final_min_epochs: int = 25
    final_patience: int = 12
    final_min_delta: float = 5e-4
    final_lr_patience: int = 4
    final_lr_factor: float = 0.5
    calibrate_thresholds: bool = True
    threshold_calibration_trials: int = 48
    threshold_calibration_tasks: int = 40
    threshold_calibration_eval_seeds: int = 3
    verbose: bool = True


@dataclass(frozen=True)
class CandidateResult:
    fitness: float
    metrics: dict[str, Any]
    particle: list[float]


PARTICLE_LO = np.array([0, 0, 0, 0.0, -4.0, -6.0, 0.5, 0.5, 0.5, 0.5, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.40, 0.40, 0.10], dtype=np.float32)
PARTICLE_HI = np.array([3, 3, 3, 0.35, -2.0, -3.0, 5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.90, 0.90, 0.50], dtype=np.float32)
BASELINE_PARTICLE = np.array(
    [
        1,      # conv_channels=(16,32,64)
        1,      # param_hidden=64
        2,      # fusion_hidden=96
        0.12,   # dropout
        -3.30,  # lr ~= 5e-4
        -4.50,  # weight_decay ~= 3.16e-5
        1.5,    # loss weight fail
        1.0,    # loss weight bottom
        1.0,    # loss weight stuck
        1.2,    # loss weight risk
        1.0,    # fusion: y_fail
        1.0,    # fusion: q_roll
        1.0,    # fusion: q_pitch
        1.0,    # fusion: q_slip
        1.0,    # fusion: q_lift
        1.0,    # fusion: p_bottom
        1.0,    # fusion: p_stuck
        0.75,   # threshold edge
        0.85,   # threshold path_max
        0.45,   # threshold path_avg
    ],
    dtype=np.float32,
)


def run_pso_training(
    data: RiskDatasetArrays,
    splits: SplitIndices,
    output_dir: Path,
    cfg: TrainConfig,
    channel_stats: dict[str, list[float]] | None = None,
) -> dict[str, Any]:
    torch, _ = require_torch()
    device = _resolve_device(torch, cfg.device)
    output_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(cfg.seed)

    channel_stats = channel_stats or compute_channel_stats(data.x_map, splits.train)
    x_norm = normalize_x_map(data.x_map, channel_stats)
    pso_train_idx = _subsample_indices(splits.train, cfg.max_pso_train_samples, rng)
    _log(
        cfg,
        "[train] "
        f"samples={data.y.shape[0]} train={splits.train.shape[0]} val={splits.val.shape[0]} "
        f"test={splits.test.shape[0]} pso_train={pso_train_idx.shape[0]} device={device}",
    )
    if str(device).startswith("cuda"):
        _log(cfg, f"[train] cuda_device={torch.cuda.get_device_name(device)}")

    particles = rng.uniform(PARTICLE_LO, PARTICLE_HI, size=(cfg.pso_particles, PARTICLE_LO.shape[0])).astype(np.float32)
    velocity = np.zeros_like(particles)
    personal_best = particles.copy()
    personal_scores = np.full(cfg.pso_particles, -np.inf, dtype=np.float32)
    global_best = particles[0].copy()
    global_score = -np.inf
    history: list[dict[str, Any]] = []

    for iter_idx in range(cfg.pso_iters):
        inertia = 0.65 - 0.20 * (iter_idx / max(cfg.pso_iters - 1, 1))
        _log(cfg, f"[pso] iter {iter_idx + 1}/{cfg.pso_iters} inertia={inertia:.3f}")
        for particle_idx in range(cfg.pso_particles):
            _log(cfg, f"[pso] iter {iter_idx + 1}/{cfg.pso_iters} particle {particle_idx + 1}/{cfg.pso_particles} start")
            result = evaluate_particle(
                particle=particles[particle_idx],
                data=data,
                x_norm=x_norm,
                train_idx=pso_train_idx,
                val_idx=splits.val,
                cfg=cfg,
                epochs=cfg.pso_epochs,
                device=device,
                seed=cfg.seed + 1000 * iter_idx + particle_idx,
                planner_seed=cfg.seed + 100000 + iter_idx,
                iter_idx=iter_idx,
                particle_idx=particle_idx,
            )
            score = result.fitness
            _log(
                cfg,
                "[pso] "
                f"iter={iter_idx + 1} particle={particle_idx + 1} fitness={score:.4f} "
                f"auc={result.metrics['auc_fail']:.4f} recall={result.metrics['recall_fail']:.4f} "
                f"mae={result.metrics['mae_risk']:.4f} "
                f"plan_sr={result.metrics['planner']['plan_success_rate']:.3f} "
                f"best={max(global_score, score):.4f}",
            )
            if score > personal_scores[particle_idx]:
                personal_scores[particle_idx] = score
                personal_best[particle_idx] = particles[particle_idx].copy()
            if score > global_score:
                global_score = float(score)
                global_best = particles[particle_idx].copy()

            history.append(
                {
                    "iter": iter_idx,
                    "particle": particle_idx,
                    "fitness": float(score),
                    "metrics": result.metrics,
                    "position": particles[particle_idx].astype(float).tolist(),
                }
            )

        r1 = rng.random(size=particles.shape, dtype=np.float32)
        r2 = rng.random(size=particles.shape, dtype=np.float32)
        velocity = inertia * velocity + 1.4 * r1 * (personal_best - particles) + 1.4 * r2 * (global_best - particles)
        particles = np.clip(particles + velocity, PARTICLE_LO, PARTICLE_HI)

    _log(cfg, f"[final] training final model with best_pso_fitness={global_score:.4f}")
    final = train_final_model(
        particle=global_best,
        data=data,
        x_norm=x_norm,
        splits=splits,
        cfg=cfg,
        output_dir=output_dir,
        channel_stats=channel_stats,
        device=device,
    )
    report = {
        "seed": cfg.seed,
        "device": str(device),
        "channel_stats": channel_stats,
        "best_particle": global_best.astype(float).tolist(),
        "best_pso_fitness": global_score,
        "pso_history": history,
        "final": final,
    }
    (output_dir / "training_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")
    _log(cfg, f"[done] report={output_dir / 'training_report.json'} model={output_dir / 'best_model.pt'}")
    return report


def run_baseline_training(
    data: RiskDatasetArrays,
    splits: SplitIndices,
    output_dir: Path,
    cfg: TrainConfig,
    channel_stats: dict[str, list[float]] | None = None,
    particle: np.ndarray | None = None,
) -> dict[str, Any]:
    torch, _ = require_torch()
    device = _resolve_device(torch, cfg.device)
    output_dir.mkdir(parents=True, exist_ok=True)

    channel_stats = channel_stats or compute_channel_stats(data.x_map, splits.train)
    x_norm = normalize_x_map(data.x_map, channel_stats)
    baseline_particle = np.asarray(particle if particle is not None else BASELINE_PARTICLE, dtype=np.float32).copy()
    baseline_particle = np.clip(baseline_particle, PARTICLE_LO, PARTICLE_HI)
    _log(cfg, f"[baseline] training with fixed particle on device={device}")

    final = train_final_model(
        particle=baseline_particle,
        data=data,
        x_norm=x_norm,
        splits=splits,
        cfg=cfg,
        output_dir=output_dir,
        channel_stats=channel_stats,
        device=device,
    )
    report = {
        "seed": cfg.seed,
        "device": str(device),
        "channel_stats": channel_stats,
        "baseline_particle": baseline_particle.astype(float).tolist(),
        "final": final,
    }
    (output_dir / "training_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")
    _log(cfg, f"[done] report={output_dir / 'training_report.json'} model={output_dir / 'best_model.pt'}")
    return report


def evaluate_particle(
    particle: np.ndarray,
    data: RiskDatasetArrays,
    x_norm: np.ndarray,
    train_idx: np.ndarray,
    val_idx: np.ndarray,
    cfg: TrainConfig,
    epochs: int,
    device,
    seed: int,
    planner_seed: int,
    iter_idx: int | None = None,
    particle_idx: int | None = None,
) -> CandidateResult:
    torch, _ = require_torch()
    model_cfg, train_cfg, loss_weights, fusion_weights, thresholds = decode_particle(particle)
    _set_torch_seed(torch, seed)
    model = build_model(model_cfg).to(device)
    prefix = _candidate_prefix(iter_idx, particle_idx)
    _log(
        cfg,
        f"{prefix} config conv={model_cfg.conv_channels} param_hidden={model_cfg.param_hidden} "
        f"fusion_hidden={model_cfg.fusion_hidden} dropout={model_cfg.dropout:.3f} "
        f"lr={train_cfg['lr']:.2e} wd={train_cfg['weight_decay']:.2e}",
    )
    train_loader = _make_loader(
        torch,
        x_norm,
        data.param,
        data.y,
        train_idx,
        cfg.batch_size,
        shuffle=True,
        seed=seed,
        num_workers=cfg.loader_num_workers,
        pin_memory=str(device).startswith("cuda"),
    )
    val_loader = _make_loader(
        torch,
        x_norm,
        data.param,
        data.y,
        val_idx,
        cfg.batch_size,
        shuffle=False,
        seed=seed,
        num_workers=cfg.loader_num_workers,
        pin_memory=str(device).startswith("cuda"),
    )
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(train_cfg["lr"]),
        weight_decay=float(train_cfg["weight_decay"]),
    )

    try:
        for epoch in range(epochs):
            epoch_start = perf_counter()
            loss = _train_one_epoch(torch, model, train_loader, optimizer, loss_weights, device)
            _sync_if_cuda(torch, device)
            _log(
                cfg,
                f"{prefix} epoch {epoch + 1}/{epochs} loss={loss:.5f} "
                f"elapsed={perf_counter() - epoch_start:.1f}s",
            )

        _log(cfg, f"{prefix} evaluating validation metrics")
        metrics, y_pred = _evaluate(torch, model, val_loader, data.y[val_idx], device)
        _log(cfg, f"{prefix} evaluating proxy A*")
        planner = _evaluate_proxy_stable(
            y_true=data.y[val_idx],
            y_pred=y_pred,
            fusion_weights=fusion_weights,
            thresholds=thresholds,
            base_seed=planner_seed,
            eval_seeds=max(int(cfg.proxy_eval_seeds_pso), 1),
            num_tasks=max(int(cfg.proxy_tasks_pso), 4),
        )
        fitness = _fitness(metrics, planner)
        metrics["planner"] = planner
        metrics["model_config"] = model_cfg.__dict__
        metrics["train_config"] = train_cfg
        metrics["loss_weights"] = loss_weights
        metrics["fusion_weights"] = fusion_weights
        metrics["thresholds"] = thresholds
        return CandidateResult(fitness=fitness, metrics=metrics, particle=particle.astype(float).tolist())
    finally:
        del model, optimizer, train_loader, val_loader
        _clear_cuda_cache(torch, device)


def train_final_model(
    particle: np.ndarray,
    data: RiskDatasetArrays,
    x_norm: np.ndarray,
    splits: SplitIndices,
    cfg: TrainConfig,
    output_dir: Path,
    channel_stats: dict[str, list[float]],
    device,
) -> dict[str, Any]:
    torch, _ = require_torch()
    model_cfg, train_cfg, loss_weights, fusion_weights, thresholds = decode_particle(particle)
    _set_torch_seed(torch, cfg.seed + 99991)
    model = build_model(model_cfg).to(device)
    train_loader = _make_loader(
        torch,
        x_norm,
        data.param,
        data.y,
        splits.train,
        cfg.batch_size,
        shuffle=True,
        seed=cfg.seed,
        num_workers=cfg.loader_num_workers,
        pin_memory=str(device).startswith("cuda"),
    )
    val_loader = _make_loader(
        torch,
        x_norm,
        data.param,
        data.y,
        splits.val,
        cfg.batch_size,
        shuffle=False,
        seed=cfg.seed,
        num_workers=cfg.loader_num_workers,
        pin_memory=str(device).startswith("cuda"),
    )
    test_loader = _make_loader(
        torch,
        x_norm,
        data.param,
        data.y,
        splits.test,
        cfg.batch_size,
        shuffle=False,
        seed=cfg.seed,
        num_workers=cfg.loader_num_workers,
        pin_memory=str(device).startswith("cuda"),
    )
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(train_cfg["lr"]),
        weight_decay=float(train_cfg["weight_decay"]),
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode="max",
        factor=float(cfg.final_lr_factor),
        patience=max(int(cfg.final_lr_patience), 1),
        threshold=max(float(cfg.final_min_delta), 0.0),
        threshold_mode="abs",
        min_lr=1e-6,
    )

    losses: list[float] = []
    val_history: list[dict[str, float]] = []
    best_state: dict[str, Any] | None = None
    best_epoch = -1
    best_score = -np.inf
    no_improve = 0
    stop_reason = "max_epochs"

    for epoch in range(cfg.final_epochs):
        epoch_start = perf_counter()
        epoch_loss = float(_train_one_epoch(torch, model, train_loader, optimizer, loss_weights, device))
        _sync_if_cuda(torch, device)
        losses.append(epoch_loss)

        val_metrics, _ = _evaluate(torch, model, val_loader, data.y[splits.val], device)
        val_score = float(_validation_objective(val_metrics))
        val_history.append(
            {
                "epoch": float(epoch + 1),
                "train_loss": epoch_loss,
                "val_auc_fail": float(val_metrics["auc_fail"]),
                "val_recall_fail": float(val_metrics["recall_fail"]),
                "val_mae_risk": float(val_metrics["mae_risk"]),
                "val_score": val_score,
                "lr": float(optimizer.param_groups[0]["lr"]),
            }
        )
        scheduler.step(val_score)

        improved = (val_score - best_score) > float(cfg.final_min_delta)
        if improved:
            best_score = val_score
            best_epoch = epoch + 1
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            no_improve = 0
        else:
            no_improve += 1

        _log(
            cfg,
            f"[final] epoch {epoch + 1}/{cfg.final_epochs} train_loss={epoch_loss:.5f} "
            f"val_auc={val_metrics['auc_fail']:.4f} val_recall={val_metrics['recall_fail']:.4f} "
            f"val_mae={val_metrics['mae_risk']:.4f} val_score={val_score:.4f} "
            f"best={best_score:.4f} wait={no_improve}/{cfg.final_patience} "
            f"lr={optimizer.param_groups[0]['lr']:.2e} elapsed={perf_counter() - epoch_start:.1f}s",
        )

        if (epoch + 1) >= max(int(cfg.final_min_epochs), 1) and no_improve >= max(int(cfg.final_patience), 1):
            stop_reason = f"early_stop_patience={cfg.final_patience}"
            _log(cfg, f"[final] early stop at epoch {epoch + 1} (best_epoch={best_epoch})")
            break

    if best_state is not None:
        model.load_state_dict(best_state)
    else:
        best_epoch = len(losses)
        best_score = float("-inf")

    _, y_pred_val = _evaluate(torch, model, val_loader, data.y[splits.val], device)
    thresholds_calibration: dict[str, Any] | None = None
    if bool(cfg.calibrate_thresholds):
        calibrated_thresholds, thresholds_calibration = _calibrate_thresholds(
            y_true=data.y[splits.val],
            y_pred=y_pred_val,
            base_thresholds=thresholds,
            fusion_weights=fusion_weights,
            base_seed=cfg.seed + 1701,
            trials=max(int(cfg.threshold_calibration_trials), 1),
            num_tasks=max(int(cfg.threshold_calibration_tasks), 4),
            eval_seeds=max(int(cfg.threshold_calibration_eval_seeds), 1),
        )
        _log(
            cfg,
            "[final] threshold calibration "
            f"score={thresholds_calibration['best_score']:.4f} "
            f"edge={calibrated_thresholds['edge']:.3f} "
            f"path_max={calibrated_thresholds['path_max']:.3f} "
            f"path_avg={calibrated_thresholds['path_avg']:.3f}",
        )
        thresholds = calibrated_thresholds

    metrics, y_pred = _evaluate(torch, model, test_loader, data.y[splits.test], device)
    planner = _evaluate_proxy_stable(
        y_true=data.y[splits.test],
        y_pred=y_pred,
        fusion_weights=fusion_weights,
        thresholds=thresholds,
        base_seed=cfg.seed + 77,
        eval_seeds=max(int(cfg.proxy_eval_seeds_final), 1),
        num_tasks=max(int(cfg.proxy_tasks_final), 4),
    )
    best_val_metrics = val_history[max(best_epoch - 1, 0)] if val_history else {}
    model_path = output_dir / "best_model.pt"
    torch.save(
        {
            "state_dict": model.state_dict(),
            "model_config": model_cfg.__dict__,
            "channel_stats": channel_stats,
            "fusion_weights": fusion_weights,
            "thresholds": thresholds,
            "label_order": ["y_fail", "q_roll", "q_pitch", "q_slip", "q_lift", "p_bottom", "p_stuck"],
        },
        model_path,
    )
    metrics["planner"] = planner
    return {
        "model_path": str(model_path),
        "model_config": model_cfg.__dict__,
        "train_config": train_cfg,
        "loss_weights": loss_weights,
        "fusion_weights": fusion_weights,
        "thresholds": thresholds,
        "train_losses": losses,
        "val_history": val_history,
        "best_epoch": int(best_epoch),
        "best_val_score": float(best_score),
        "best_val_metrics": best_val_metrics,
        "stop_reason": stop_reason,
        "thresholds_calibration": thresholds_calibration,
        "test_metrics": metrics,
    }


def _train_one_epoch(torch, model, loader, optimizer, loss_weights: dict[str, float], device) -> float:
    model.train()
    total = 0.0
    n = 0
    for x_map, param, y in loader:
        x_map = x_map.to(device)
        param = param.to(device)
        y = y.to(device)
        optimizer.zero_grad(set_to_none=True)
        logits = model(x_map, param)
        loss = _loss(torch, logits, y, loss_weights)
        loss.backward()
        optimizer.step()
        total += float(loss.detach().cpu()) * x_map.shape[0]
        n += x_map.shape[0]
    return total / max(n, 1)


def _loss(torch, logits, y, weights: dict[str, float]):
    bce = torch.nn.functional.binary_cross_entropy_with_logits
    pred = torch.sigmoid(logits)
    fail_loss = bce(logits[:, 0], y[:, 0])
    bottom_loss = bce(logits[:, 5], y[:, 5])
    stuck_loss = bce(logits[:, 6], y[:, 6])
    risk_loss = torch.nn.functional.l1_loss(pred[:, 1:5], y[:, 1:5])
    return (
        weights["fail"] * fail_loss
        + weights["bottom"] * bottom_loss
        + weights["stuck"] * stuck_loss
        + weights["risk"] * risk_loss
    )


def _evaluate(torch, model, loader, y_true_np: np.ndarray, device) -> tuple[dict[str, Any], np.ndarray]:
    model.eval()
    preds = []
    start = perf_counter()
    with torch.no_grad():
        for x_map, param, _ in loader:
            logits = model(x_map.to(device), param.to(device))
            preds.append(torch.sigmoid(logits).cpu().numpy())
    _sync_if_cuda(torch, device)
    elapsed = perf_counter() - start
    y_pred = np.concatenate(preds, axis=0)
    y_fail_true = y_true_np[:, 0].astype(int)
    y_fail_pred = (y_pred[:, 0] >= 0.5).astype(int)
    try:
        auc = float(roc_auc_score(y_fail_true, y_pred[:, 0]))
    except ValueError:
        auc = 0.5
    metrics = {
        "auc_fail": auc,
        "f1_fail": float(f1_score(y_fail_true, y_fail_pred, zero_division=0)),
        "precision_fail": float(precision_score(y_fail_true, y_fail_pred, zero_division=0)),
        "recall_fail": float(recall_score(y_fail_true, y_fail_pred, zero_division=0)),
        "mae_risk": float(np.mean(np.abs(y_pred[:, 1:] - y_true_np[:, 1:]))),
        "mae_all": float(np.mean(np.abs(y_pred - y_true_np))),
        "infer_ms_per_sample": float(1000.0 * elapsed / max(y_true_np.shape[0], 1)),
    }
    return metrics, y_pred


def _fitness(metrics: dict[str, Any], planner: dict[str, float]) -> float:
    return float(
        metrics["auc_fail"]
        + 0.30 * metrics["recall_fail"]
        - 0.80 * metrics["mae_risk"]
        - 0.01 * metrics["infer_ms_per_sample"]
        + 0.45 * planner["plan_success_rate"]
        + 0.20 * planner["oracle_safe_rate"]
        - 0.12 * max(planner["mean_length_ratio"] - 1.0, 0.0)
    )


def _validation_objective(metrics: dict[str, Any]) -> float:
    return float(
        metrics["auc_fail"]
        + 0.30 * metrics["recall_fail"]
        - 0.80 * metrics["mae_risk"]
    )


def _evaluate_proxy_stable(
    *,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    fusion_weights: list[float],
    thresholds: dict[str, float],
    base_seed: int,
    eval_seeds: int,
    num_tasks: int,
) -> dict[str, float]:
    plan_sr = 0.0
    oracle_sr = 0.0
    len_ratio = 0.0
    solved = 0
    total = 0
    for i in range(max(eval_seeds, 1)):
        r = evaluate_proxy_astar(
            y_true=y_true,
            y_pred=y_pred,
            fusion_weights=fusion_weights,
            thresholds=thresholds,
            seed=int(base_seed + 7919 * i),
            num_tasks=max(int(num_tasks), 4),
        )
        plan_sr += float(r.plan_success_rate)
        oracle_sr += float(r.oracle_safe_rate)
        len_ratio += float(r.mean_length_ratio)
        solved += int(r.solved_tasks)
        total += int(r.total_tasks)
    denom = float(max(eval_seeds, 1))
    return {
        "plan_success_rate": plan_sr / denom,
        "oracle_safe_rate": oracle_sr / denom,
        "mean_length_ratio": len_ratio / denom,
        "solved_tasks": int(solved),
        "total_tasks": int(total),
    }


def _calibrate_thresholds(
    *,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    base_thresholds: dict[str, float],
    fusion_weights: list[float],
    base_seed: int,
    trials: int,
    num_tasks: int,
    eval_seeds: int,
) -> tuple[dict[str, float], dict[str, Any]]:
    rng = np.random.default_rng(base_seed)

    def _clip_thresholds(candidate: dict[str, float]) -> dict[str, float]:
        edge = float(np.clip(candidate["edge"], 0.4, 0.9))
        path_max = float(np.clip(candidate["path_max"], 0.4, 0.9))
        path_avg = float(np.clip(candidate["path_avg"], 0.1, 0.5))
        return {"edge": edge, "path_max": path_max, "path_avg": path_avg}

    candidates: list[dict[str, float]] = []
    candidates.append(_clip_thresholds(base_thresholds))
    candidates.append({"edge": 0.75, "path_max": 0.85, "path_avg": 0.45})

    base = _clip_thresholds(base_thresholds)
    for _ in range(max(trials - len(candidates), 0)):
        candidate = {
            "edge": base["edge"] + float(rng.normal(0.0, 0.10)),
            "path_max": base["path_max"] + float(rng.normal(0.0, 0.08)),
            "path_avg": base["path_avg"] + float(rng.normal(0.0, 0.07)),
        }
        if rng.random() < 0.25:
            candidate = {
                "edge": float(rng.uniform(0.4, 0.9)),
                "path_max": float(rng.uniform(0.4, 0.9)),
                "path_avg": float(rng.uniform(0.1, 0.5)),
            }
        candidates.append(_clip_thresholds(candidate))

    best_thresholds = base
    best_score = -np.inf
    best_planner: dict[str, float] | None = None
    for idx, thresholds in enumerate(candidates):
        planner = _evaluate_proxy_stable(
            y_true=y_true,
            y_pred=y_pred,
            fusion_weights=fusion_weights,
            thresholds=thresholds,
            base_seed=base_seed + 113 * idx,
            eval_seeds=eval_seeds,
            num_tasks=num_tasks,
        )
        score = _planner_objective(planner)
        if score > best_score:
            best_score = score
            best_thresholds = thresholds
            best_planner = planner

    report = {
        "trials": len(candidates),
        "best_score": float(best_score),
        "best_planner": best_planner or {},
        "base_thresholds": base,
        "selected_thresholds": best_thresholds,
    }
    return best_thresholds, report


def _planner_objective(planner: dict[str, float]) -> float:
    return float(
        1.00 * planner["plan_success_rate"]
        + 0.80 * planner["oracle_safe_rate"]
        - 0.20 * max(planner["mean_length_ratio"] - 1.0, 0.0)
    )


def _make_loader(
    torch,
    x_map: np.ndarray,
    param: np.ndarray,
    y: np.ndarray,
    indices: np.ndarray,
    batch_size: int,
    shuffle: bool,
    seed: int,
    num_workers: int = 0,
    pin_memory: bool = False,
):
    # PyTorch uses NCHW layout; stored HDF5 uses NHWC.
    x = torch.from_numpy(np.transpose(x_map[indices], (0, 3, 1, 2)).astype(np.float32))
    p = torch.from_numpy(param[indices].astype(np.float32))
    yy = torch.from_numpy(y[indices].astype(np.float32))
    dataset = torch.utils.data.TensorDataset(x, p, yy)
    generator = torch.Generator()
    generator.manual_seed(seed)
    return torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        generator=generator,
        num_workers=max(int(num_workers), 0),
        pin_memory=bool(pin_memory),
    )


def _resolve_device(torch, requested: str):
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)


def _set_torch_seed(torch, seed: int) -> None:
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False


def _subsample_indices(indices: np.ndarray, max_count: int, rng: np.random.Generator) -> np.ndarray:
    if max_count <= 0 or indices.shape[0] <= max_count:
        return indices
    return np.sort(rng.choice(indices, size=max_count, replace=False))


def _log(cfg: TrainConfig, message: str) -> None:
    if cfg.verbose:
        print(message, flush=True)


def _candidate_prefix(iter_idx: int | None, particle_idx: int | None) -> str:
    if iter_idx is None or particle_idx is None:
        return "[pso-candidate]"
    return f"[pso-candidate i{iter_idx + 1} p{particle_idx + 1}]"


def _sync_if_cuda(torch, device) -> None:
    if str(device).startswith("cuda") and torch.cuda.is_available():
        torch.cuda.synchronize(device)


def _clear_cuda_cache(torch, device) -> None:
    if str(device).startswith("cuda") and torch.cuda.is_available():
        torch.cuda.empty_cache()
