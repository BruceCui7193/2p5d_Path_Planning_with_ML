from __future__ import annotations

from dataclasses import dataclass
import heapq
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class PlannerEval:
    plan_success_rate: float
    oracle_safe_rate: float
    mean_length_ratio: float
    solved_tasks: int
    total_tasks: int


@dataclass(frozen=True)
class EdgeRisk:
    pred: float
    true: float


def fuse_risk(y: np.ndarray, weights: Iterable[float]) -> np.ndarray:
    weights_arr = np.asarray(list(weights), dtype=np.float32)
    weights_arr = weights_arr / max(float(weights_arr.sum()), 1e-6)
    if y.ndim != 2 or y.shape[1] != 7:
        raise ValueError(f"y must have shape (N,7), got {y.shape}")
    weighted = y @ weights_arr
    conservative = np.max(y[:, 1:], axis=1)
    return np.maximum(weighted, conservative * 0.65).astype(np.float32)


def evaluate_proxy_astar(
    y_true: np.ndarray,
    y_pred: np.ndarray,
    fusion_weights: list[float],
    thresholds: dict[str, float],
    seed: int,
    num_tasks: int = 24,
    grid_size: int = 14,
) -> PlannerEval:
    true_risk = fuse_risk(y_true, fusion_weights)
    pred_risk = fuse_risk(y_pred, fusion_weights)
    if true_risk.shape[0] < grid_size * grid_size:
        num_tasks = max(4, min(num_tasks, true_risk.shape[0] // 8))

    rng = np.random.default_rng(seed)
    success = 0
    oracle_safe = 0
    length_ratios: list[float] = []
    solved = 0

    for task_id in range(num_tasks):
        edge_count = grid_size * grid_size * 4
        edge_true, edge_pred = _sample_task_edges(
            true_risk=true_risk,
            pred_risk=pred_risk,
            thresholds=thresholds,
            rng=rng,
            edge_count=edge_count,
            grid_size=grid_size,
        )

        result = _astar(edge_pred, edge_true, thresholds, grid_size)
        if result is None:
            continue
        solved += 1
        length, path_true = result
        true_max = max(path_true) if path_true else 1.0
        true_avg = float(np.mean(path_true)) if path_true else 1.0
        safe = true_max <= thresholds["path_max"] and true_avg <= thresholds["path_avg"]
        success += int(safe)

        oracle = _astar(edge_true, edge_true, thresholds, grid_size)
        if oracle is not None:
            oracle_safe += 1
            oracle_len = max(float(oracle[0]), 1.0)
            length_ratios.append(float(length) / oracle_len)

    return PlannerEval(
        plan_success_rate=success / max(num_tasks, 1),
        oracle_safe_rate=oracle_safe / max(num_tasks, 1),
        mean_length_ratio=float(np.mean(length_ratios)) if length_ratios else 2.0,
        solved_tasks=solved,
        total_tasks=num_tasks,
    )


def _sample_task_edges(
    true_risk: np.ndarray,
    pred_risk: np.ndarray,
    thresholds: dict[str, float],
    rng: np.random.Generator,
    edge_count: int,
    grid_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    low_thr = float(np.quantile(true_risk, 0.35))
    high_thr = float(np.quantile(true_risk, 0.80))
    low_idx = np.flatnonzero(true_risk <= low_thr)
    med_idx = np.flatnonzero((true_risk > low_thr) & (true_risk <= high_thr))
    high_idx = np.flatnonzero(true_risk > high_thr)

    parts = [
        _choice(rng, low_idx, int(edge_count * 0.50)),
        _choice(rng, med_idx, int(edge_count * 0.30)),
        _choice(rng, high_idx, edge_count - int(edge_count * 0.50) - int(edge_count * 0.30)),
    ]
    idx = rng.permutation(np.concatenate(parts))
    edge_true = true_risk[idx].reshape(grid_size, grid_size, 4).copy()
    edge_pred = pred_risk[idx].reshape(grid_size, grid_size, 4).copy()

    # Ensure each proxy map has at least one physically safe oracle corridor.
    # The learned model still decides whether it recognizes that corridor as safe.
    safe_limit = min(thresholds["edge"] * 0.75, thresholds["path_max"] * 0.75, thresholds["path_avg"] * 0.85)
    corridor_idx = np.flatnonzero(true_risk <= safe_limit)
    if corridor_idx.shape[0] < 2 * grid_size:
        corridor_idx = low_idx

    for x in range(grid_size - 1):
        sample_idx = int(rng.choice(corridor_idx))
        edge_true[x, 0, 0] = true_risk[sample_idx]
        edge_pred[x, 0, 0] = pred_risk[sample_idx]
    for y in range(grid_size - 1):
        sample_idx = int(rng.choice(corridor_idx))
        edge_true[grid_size - 1, y, 2] = true_risk[sample_idx]
        edge_pred[grid_size - 1, y, 2] = pred_risk[sample_idx]

    return edge_true, edge_pred


def _choice(rng: np.random.Generator, values: np.ndarray, size: int) -> np.ndarray:
    if values.shape[0] == 0:
        raise ValueError("cannot sample from empty risk bin")
    return rng.choice(values, size=size, replace=values.shape[0] < size)


def _astar(
    edge_pred: np.ndarray,
    edge_true: np.ndarray,
    thresholds: dict[str, float],
    grid_size: int,
) -> tuple[int, list[float]] | None:
    start = (0, 0)
    goal = (grid_size - 1, grid_size - 1)
    moves = [(1, 0, 0), (-1, 0, 1), (0, 1, 2), (0, -1, 3)]

    max_path_len = grid_size * grid_size
    max_expansions = grid_size * grid_size * 12

    heap: list[tuple[float, int, tuple[int, int], float, float, list[float]]] = []
    heapq.heappush(heap, (_heuristic(start, goal), 0, start, 0.0, 0.0, []))
    best_cost: dict[tuple[int, int], float] = {start: 0.0}
    expansions = 0

    while heap:
        _, length, node, pred_max, pred_sum, true_path = heapq.heappop(heap)
        expansions += 1
        if expansions > max_expansions:
            return None
        if node == goal:
            return length, true_path

        x, y = node
        for dx, dy, action_idx in moves:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= grid_size or ny >= grid_size:
                continue

            edge_r = float(edge_pred[x, y, action_idx])
            if edge_r > thresholds["edge"]:
                continue

            new_len = length + 1
            if new_len > max_path_len:
                continue

            new_pred_max = max(pred_max, edge_r)
            new_pred_sum = pred_sum + edge_r
            new_pred_avg = new_pred_sum / max(new_len, 1)
            if new_pred_max > thresholds["path_max"] or new_pred_avg > thresholds["path_avg"]:
                continue

            nxt = (nx, ny)
            # Proxy A* keeps one label per node. This is intentionally less
            # exact than multi-label risk search, but bounded and stable enough
            # for PSO fitness evaluation.
            dominance_value = new_len + 1.5 * new_pred_max + 0.5 * new_pred_avg
            old_cost = best_cost.get(nxt)
            if old_cost is not None and old_cost <= dominance_value:
                continue
            best_cost[nxt] = dominance_value
            new_true_path = true_path + [float(edge_true[x, y, action_idx])]
            priority = new_len + _heuristic(nxt, goal) + 0.05 * dominance_value
            heapq.heappush(heap, (priority, new_len, nxt, new_pred_max, new_pred_sum, new_true_path))

    return None


def _heuristic(node: tuple[int, int], goal: tuple[int, int]) -> int:
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
