from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from time import perf_counter
from typing import Callable

import numpy as np

from .common_types import ActionPrimitive, VehicleParams
from .feature_builder import FeatureBuilder
from .planning_types import PathResult, PlanningScene, PlannerThresholds, State3
from .risk_model_infer import RiskModelInfer


METHOD_BASELINE_1 = "baseline1_2p5d_astar"
METHOD_BASELINE_2 = "baseline2_manual_risk_weighted_astar"
METHOD_BASELINE_3 = "baseline3_ml_risk_weighted_astar"
METHOD_PROPOSED = "proposed_ml_risk_constrained_astar"

ALL_METHODS = [METHOD_BASELINE_1, METHOD_BASELINE_2, METHOD_BASELINE_3, METHOD_PROPOSED]


@dataclass
class PlannerConfig:
    goal_radius_cells: int = 2
    max_expansions: int = 60_000
    max_labels_per_state: int = 4
    manual_risk_lambda: float = 1.0
    ml_risk_lambda: float = 1.0
    # Optional mild risk shaping inside feasible set for proposed method.
    proposed_risk_lambda: float = 0.0
    # Blend in manual geometric risk for proposed constraints/cost to avoid
    # obvious hill/pit crossings when ML edge risk is under-estimated.
    # 0.0: pure ML risk, 1.0: max(ML risk, manual risk).
    proposed_manual_guard_weight: float = 0.0
    default_thresholds: PlannerThresholds = PlannerThresholds(edge_safe=0.75, path_max_safe=0.85, path_avg_safe=0.45)


@dataclass
class _Label:
    state: State3
    parent_idx: int
    action_id: str
    g_cost: float
    path_length_m: float
    risk_sum: float
    risk_max: float
    turns: int
    edge_risk: float
    edge_risk_vec: list[float]
    active: bool = True

    @property
    def risk_avg(self) -> float:
        return float(self.risk_sum / max(self.path_length_m, 1e-6))


def _heading_from_bin(k: int, heading_bins: int) -> float:
    return float((2.0 * math.pi * (k % heading_bins)) / max(heading_bins, 1))


def _wrap_bin(k: int, heading_bins: int) -> int:
    return int(k % heading_bins)


def _goal_reached(state: State3, goal: State3, radius_cells: int) -> bool:
    di = state[0] - goal[0]
    dj = state[1] - goal[1]
    return (di * di + dj * dj) <= (radius_cells * radius_cells)


def _heuristic_m(state: State3, goal: State3, resolution_m: float) -> float:
    di = float(state[0] - goal[0])
    dj = float(state[1] - goal[1])
    return float(math.hypot(di, dj) * resolution_m)


def _dominates_for_proposed(a: _Label, b: _Label, eps: float = 1e-9) -> bool:
    cond_len = a.path_length_m <= b.path_length_m + eps
    cond_rmax = a.risk_max <= b.risk_max + eps
    cond_ravg = a.risk_avg <= b.risk_avg + eps
    strict = (
        (a.path_length_m < b.path_length_m - eps)
        or (a.risk_max < b.risk_max - eps)
        or (a.risk_avg < b.risk_avg - eps)
    )
    return bool(cond_len and cond_rmax and cond_ravg and strict)


def _extract_local_patch(padded_h: np.ndarray, i: int, j: int, patch_size: int) -> np.ndarray:
    return padded_h[i : i + patch_size, j : j + patch_size].astype(np.float32, copy=False)


def _transition(
    state: State3,
    action: ActionPrimitive,
    resolution_m: float,
    heading_bins: int,
) -> tuple[State3, float]:
    i, j, k = state
    theta = _heading_from_bin(k, heading_bins)
    ds = float(action.delta_s_m)
    dpsi = float(math.radians(action.delta_psi_deg))
    if abs(dpsi) < 1e-9:
        dx_body = ds
        dy_body = 0.0
    else:
        radius = ds / dpsi
        dx_body = radius * math.sin(dpsi)
        dy_body = radius * (1.0 - math.cos(dpsi))
    dx = math.cos(theta) * dx_body - math.sin(theta) * dy_body
    dy = math.sin(theta) * dx_body + math.cos(theta) * dy_body
    ni = int(round(i + dx / resolution_m))
    nj = int(round(j + dy / resolution_m))
    delta_bin = int(round(dpsi / (2.0 * math.pi / max(heading_bins, 1))))
    nk = _wrap_bin(k + delta_bin, heading_bins)
    dxy = float(math.hypot((ni - i) * resolution_m, (nj - j) * resolution_m))
    return (ni, nj, nk), dxy


def _manual_edge_risk(
    feature_builder: FeatureBuilder,
    local_patch: np.ndarray,
    heading_rad: float,
    vehicle: VehicleParams,
    action: ActionPrimitive,
) -> tuple[float, list[float]]:
    fmap = feature_builder.build_feature_patch(local_patch, heading_rad, vehicle, action)
    swept = fmap[:, :, 5] > 0.5
    if not np.any(swept):
        swept = np.ones_like(swept, dtype=bool)
    grad = np.sqrt(fmap[:, :, 1] ** 2 + fmap[:, :, 2] ** 2)
    rough = np.abs(fmap[:, :, 3])
    slope_metric = float(np.mean(grad[swept]))
    rough_metric = float(np.mean(rough[swept]))
    slope_score = float(np.clip(slope_metric / 0.35, 0.0, 1.0))
    rough_score = float(np.clip(rough_metric / 0.06, 0.0, 1.0))
    risk = float(np.clip(0.65 * slope_score + 0.35 * rough_score, 0.0, 1.0))
    y_vec = [risk, slope_score, slope_score, rough_score, rough_score, 0.0, 0.0]
    return risk, y_vec


def _baseline_edge_risk() -> tuple[float, list[float]]:
    return 0.0, [0.0] * 7


def _transition_is_valid(state: State3, h: np.ndarray) -> bool:
    i, j, _ = state
    return 0 <= i < h.shape[0] and 0 <= j < h.shape[1]


def _traceback(labels: list[_Label], goal_idx: int) -> tuple[list[State3], list[str], list[float], list[list[float]]]:
    states_rev: list[State3] = []
    actions_rev: list[str] = []
    edge_risk_rev: list[float] = []
    edge_vec_rev: list[list[float]] = []
    idx = goal_idx
    while idx >= 0:
        lab = labels[idx]
        states_rev.append(lab.state)
        if lab.parent_idx >= 0:
            actions_rev.append(lab.action_id)
            edge_risk_rev.append(float(lab.edge_risk))
            edge_vec_rev.append([float(v) for v in lab.edge_risk_vec])
        idx = lab.parent_idx
    return (
        list(reversed(states_rev)),
        list(reversed(actions_rev)),
        list(reversed(edge_risk_rev)),
        list(reversed(edge_vec_rev)),
    )


def _select_thresholds(
    explicit: PlannerThresholds | None,
    model_infer: RiskModelInfer | None,
    defaults: PlannerThresholds,
) -> PlannerThresholds:
    if explicit is not None:
        return explicit
    if model_infer is not None:
        return model_infer.thresholds
    return defaults


def plan_path(
    scene: PlanningScene,
    vehicle: VehicleParams,
    actions: list[ActionPrimitive],
    method: str,
    model_infer: RiskModelInfer | None = None,
    config: PlannerConfig | None = None,
    thresholds: PlannerThresholds | None = None,
) -> PathResult:
    if method not in ALL_METHODS:
        raise ValueError(f"unknown method: {method}")
    cfg = config or PlannerConfig()
    if method in {METHOD_BASELINE_3, METHOD_PROPOSED} and model_infer is None:
        raise ValueError(f"method {method} requires model_infer")
    if scene.heightmap.ndim != 2:
        raise ValueError(f"scene.heightmap must be 2D, got {scene.heightmap.shape}")

    start_t = perf_counter()
    h = np.asarray(scene.heightmap, dtype=np.float32)
    patch_size = int(h.shape[0] if h.shape[0] < 31 else 31)
    # Model training uses 31x31 patches; keep planner extraction consistent.
    patch_size = 31
    half = patch_size // 2
    h_pad = np.pad(h, ((half, half), (half, half)), mode="edge")
    feature_builder = FeatureBuilder(patch_size=patch_size, resolution_m=scene.resolution_m)
    thresholds_eff = _select_thresholds(thresholds, model_infer, cfg.default_thresholds)

    labels: list[_Label] = []
    labels_by_state: dict[State3, list[int]] = {}
    heap: list[tuple[float, float, float, int, int, int]] = []
    push_counter = 0
    expanded = 0
    goal_idx = -1
    risk_cache: dict[tuple[int, int, int, str], tuple[float, list[float]]] = {}

    root = _Label(
        state=scene.start_state,
        parent_idx=-1,
        action_id="",
        g_cost=0.0,
        path_length_m=0.0,
        risk_sum=0.0,
        risk_max=0.0,
        turns=0,
        edge_risk=0.0,
        edge_risk_vec=[0.0] * 7,
        active=True,
    )
    labels.append(root)
    labels_by_state[scene.start_state] = [0]
    h0 = _heuristic_m(scene.start_state, scene.goal_state, scene.resolution_m)
    heapq.heappush(heap, (h0, 0.0, 0.0, 0, push_counter, 0))
    push_counter += 1

    while heap:
        _, _, _, _, _, cur_idx = heapq.heappop(heap)
        cur = labels[cur_idx]
        if not cur.active:
            continue
        if _goal_reached(cur.state, scene.goal_state, cfg.goal_radius_cells):
            goal_idx = cur_idx
            break
        expanded += 1
        if expanded > cfg.max_expansions:
            break

        i, j, k = cur.state
        local_patch = _extract_local_patch(h_pad, i, j, patch_size)
        heading_rad = _heading_from_bin(k, scene.heading_bins)

        for action in actions:
            nxt, dxy = _transition(cur.state, action, scene.resolution_m, scene.heading_bins)
            if dxy <= 1e-6:
                continue
            if not _transition_is_valid(nxt, h):
                continue

            ni, nj, _ = nxt
            dz = float(h[ni, nj] - h[i, j])
            d3d = float(math.sqrt(max(dxy * dxy + dz * dz, 1e-9)))

            cache_key = (i, j, k, action.action_id)
            cached = risk_cache.get(cache_key)
            if cached is None:
                if method == METHOD_BASELINE_1:
                    edge_risk, edge_vec = _baseline_edge_risk()
                elif method == METHOD_BASELINE_2:
                    edge_risk, edge_vec = _manual_edge_risk(feature_builder, local_patch, heading_rad, vehicle, action)
                else:
                    pred = model_infer.predict_edge(local_patch, heading_rad, vehicle, action, scene.friction_mu)  # type: ignore[union-attr]
                    edge_risk = float(pred.edge_risk)
                    edge_vec = pred.y_hat.astype(float).tolist()
                risk_cache[cache_key] = (float(edge_risk), [float(v) for v in edge_vec])
            else:
                edge_risk, edge_vec = cached

            edge_risk_plan = float(edge_risk)
            if method == METHOD_PROPOSED and float(cfg.proposed_manual_guard_weight) > 1e-9:
                manual_risk, _ = _manual_edge_risk(feature_builder, local_patch, heading_rad, vehicle, action)
                w = float(np.clip(cfg.proposed_manual_guard_weight, 0.0, 1.0))
                guarded = float(max(edge_risk, manual_risk))
                edge_risk_plan = float((1.0 - w) * edge_risk + w * guarded)

            new_path_length = float(cur.path_length_m + d3d)
            new_risk_sum = float(cur.risk_sum + d3d * edge_risk_plan)
            new_risk_max = float(max(cur.risk_max, edge_risk_plan))
            new_risk_avg = float(new_risk_sum / max(new_path_length, 1e-6))
            turns = int(cur.turns + (1 if abs(action.delta_psi_deg) > 1e-6 else 0))

            if method == METHOD_PROPOSED:
                if edge_risk_plan > thresholds_eff.edge_safe:
                    continue
                if new_risk_max > thresholds_eff.path_max_safe:
                    continue
                if new_risk_avg > thresholds_eff.path_avg_safe:
                    continue

            if method == METHOD_BASELINE_1:
                step_cost = d3d
            elif method == METHOD_BASELINE_2:
                step_cost = d3d * (1.0 + cfg.manual_risk_lambda * edge_risk)
            elif method == METHOD_BASELINE_3:
                step_cost = d3d * (1.0 + cfg.ml_risk_lambda * edge_risk)
            else:
                step_cost = d3d * (1.0 + max(float(cfg.proposed_risk_lambda), 0.0) * edge_risk_plan)
            new_g = float(cur.g_cost + step_cost)

            candidate = _Label(
                state=nxt,
                parent_idx=cur_idx,
                action_id=action.action_id,
                g_cost=new_g,
                path_length_m=new_path_length,
                risk_sum=new_risk_sum,
                risk_max=new_risk_max,
                turns=turns,
                edge_risk=float(edge_risk_plan),
                edge_risk_vec=[float(v) for v in edge_vec],
                active=True,
            )

            existing = labels_by_state.get(nxt, [])
            active_existing = [idx for idx in existing if labels[idx].active]
            if method == METHOD_PROPOSED:
                dominated = False
                to_drop: list[int] = []
                for idx in active_existing:
                    old = labels[idx]
                    if _dominates_for_proposed(old, candidate):
                        dominated = True
                        break
                    if _dominates_for_proposed(candidate, old):
                        to_drop.append(idx)
                if dominated:
                    continue
                if to_drop:
                    for idx in to_drop:
                        labels[idx].active = False
                    active_existing = [idx for idx in active_existing if idx not in to_drop]
                if len(active_existing) >= cfg.max_labels_per_state:
                    worst_idx = max(
                        active_existing,
                        key=lambda idx: (
                            labels[idx].path_length_m,
                            labels[idx].risk_max,
                            labels[idx].risk_avg,
                        ),
                    )
                    worst = labels[worst_idx]
                    better = (
                        candidate.path_length_m < worst.path_length_m
                        or candidate.risk_max < worst.risk_max
                        or candidate.risk_avg < worst.risk_avg
                    )
                    if not better:
                        continue
                    labels[worst_idx].active = False
                    active_existing = [idx for idx in active_existing if idx != worst_idx]
                labels.append(candidate)
                cand_idx = len(labels) - 1
                labels_by_state[nxt] = active_existing + [cand_idx]
            else:
                # Non-proposed baselines keep one best label per state by g_cost.
                if active_existing:
                    old_idx = min(active_existing, key=lambda idx: labels[idx].g_cost)
                    old = labels[old_idx]
                    if old.g_cost <= candidate.g_cost + 1e-9:
                        continue
                    labels[old_idx].active = False
                    active_existing = [idx for idx in active_existing if idx != old_idx]
                labels.append(candidate)
                cand_idx = len(labels) - 1
                labels_by_state[nxt] = active_existing + [cand_idx]

            heuristic = _heuristic_m(nxt, scene.goal_state, scene.resolution_m)
            priority = float(candidate.g_cost + heuristic)
            heapq.heappush(
                heap,
                (
                    priority,
                    candidate.risk_avg,
                    candidate.risk_max,
                    candidate.turns,
                    push_counter,
                    cand_idx,
                ),
            )
            push_counter += 1

    elapsed_ms = float((perf_counter() - start_t) * 1000.0)
    if goal_idx < 0:
        return PathResult(
            found=False,
            fail_reason="max_expansions_or_no_path",
            states=[],
            actions=[],
            edge_risks=[],
            edge_risk_vectors=[],
            path_length_m=float("nan"),
            risk_max=float("nan"),
            risk_avg=float("nan"),
            expanded_nodes=int(expanded),
            planning_time_ms=elapsed_ms,
        )

    states, actions_seq, edge_risks, edge_vec = _traceback(labels, goal_idx)
    goal_lab = labels[goal_idx]
    return PathResult(
        found=True,
        fail_reason="",
        states=states,
        actions=actions_seq,
        edge_risks=edge_risks,
        edge_risk_vectors=edge_vec,
        path_length_m=float(goal_lab.path_length_m),
        risk_max=float(goal_lab.risk_max),
        risk_avg=float(goal_lab.risk_avg),
        expanded_nodes=int(expanded),
        planning_time_ms=elapsed_ms,
    )
