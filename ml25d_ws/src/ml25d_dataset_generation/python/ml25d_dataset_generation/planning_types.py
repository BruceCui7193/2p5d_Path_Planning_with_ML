from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


State3 = tuple[int, int, int]


@dataclass(frozen=True)
class PlanningScene:
    scene_id: str
    terrain_class: str
    heightmap: np.ndarray
    resolution_m: float
    friction_mu: float
    start_state: State3
    goal_state: State3
    heading_bins: int


@dataclass(frozen=True)
class PlannerThresholds:
    edge_safe: float
    path_max_safe: float
    path_avg_safe: float


@dataclass
class PathResult:
    found: bool
    fail_reason: str
    states: list[State3]
    actions: list[str]
    edge_risks: list[float]
    edge_risk_vectors: list[list[float]]
    path_length_m: float
    risk_max: float
    risk_avg: float
    expanded_nodes: int
    planning_time_ms: float

    def as_metrics_dict(self) -> dict[str, Any]:
        return {
            "found": int(self.found),
            "fail_reason": self.fail_reason,
            "path_length_m": float(self.path_length_m),
            "risk_max": float(self.risk_max),
            "risk_avg": float(self.risk_avg),
            "expanded_nodes": int(self.expanded_nodes),
            "planning_time_ms": float(self.planning_time_ms),
            "num_states": int(len(self.states)),
            "num_actions": int(len(self.actions)),
        }
