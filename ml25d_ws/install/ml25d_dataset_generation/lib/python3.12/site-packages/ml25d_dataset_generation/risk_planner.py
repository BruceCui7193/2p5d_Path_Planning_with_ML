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
        replace = edge_count > true_risk.shape[0]
        idx = rng.choice(true_risk.shape[0], size=edge_count, replace=replace)
        edge_true = true_risk[idx].reshape(grid_size, grid_size, 4)
        edge_pred = pred_risk[idx].reshape(grid_size, grid_size, 4)

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


def _astar(
    edge_pred: np.ndarray,
    edge_true: np.ndarray,
    thresholds: dict[str, float],
    grid_size: int,
) -> tuple[int, list[float]] | None:
    start = (0, 0)
    goal = (grid_size - 1, grid_size - 1)
    moves = [(1, 0, 0), (-1, 0, 1), (0, 1, 2), (0, -1, 3)]

    heap: list[tuple[float, int, tuple[int, int], float, float, list[float]]] = []
    heapq.heappush(heap, (_heuristic(start, goal), 0, start, 0.0, 0.0, []))
    best: dict[tuple[int, int], tuple[int, float]] = {start: (0, 0.0)}

    while heap:
        _, length, node, pred_max, pred_sum, true_path = heapq.heappop(heap)
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
            new_pred_max = max(pred_max, edge_r)
            new_pred_sum = pred_sum + edge_r
            new_pred_avg = new_pred_sum / max(new_len, 1)
            if new_pred_max > thresholds["path_max"] or new_pred_avg > thresholds["path_avg"]:
                continue

            nxt = (nx, ny)
            old = best.get(nxt)
            dominance_value = new_pred_max + new_pred_avg
            if old is not None and old[0] <= new_len and old[1] <= dominance_value:
                continue
            best[nxt] = (new_len, dominance_value)
            new_true_path = true_path + [float(edge_true[x, y, action_idx])]
            priority = new_len + _heuristic(nxt, goal) + 0.05 * dominance_value
            heapq.heappush(heap, (priority, new_len, nxt, new_pred_max, new_pred_sum, new_true_path))

    return None


def _heuristic(node: tuple[int, int], goal: tuple[int, int]) -> int:
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
