"""Path-following controller: a look-ahead ("pure-pursuit"-style) heading controller."""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from .config import Config
from .geometry import wrap_angle


class PathFollower:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.path = np.zeros((0, 2))
        self.idx = 0

    def set_path(self, path_xy: np.ndarray) -> None:
        self.path = path_xy
        self.idx = 0

    def _advance_index(self, x: float, y: float) -> None:
        """Move the 'closest point' index forward only (never backwards)."""
        hi = min(len(self.path), self.idx + 40)
        d = np.hypot(self.path[self.idx:hi, 0] - x, self.path[self.idx:hi, 1] - y)
        self.idx += int(np.argmin(d))

    def lookahead_point(self, x: float, y: float) -> Tuple[float, float]:
        self._advance_index(x, y)
        L = self.cfg.lookahead_m
        acc, i = 0.0, self.idx
        while i < len(self.path) - 1 and acc < L:
            acc += float(np.hypot(*(self.path[i + 1] - self.path[i])))
            i += 1
        return float(self.path[i, 0]), float(self.path[i, 1])

    def command(self, pose, goal_dist: float) -> Tuple[float, float]:
        """Return (v, w) that steers toward the look-ahead point."""
        c = self.cfg
        x, y, th = pose
        lx, ly = self.lookahead_point(x, y)
        alpha = wrap_angle(math.atan2(ly - y, lx - x) - th)   # heading error
        w = float(np.clip(2.2 * alpha, -c.max_w, c.max_w))
        if abs(alpha) > 0.9:                       # facing the wrong way: turn on the spot
            return 0.0, w
        v = c.max_v * max(0.0, math.cos(alpha)) ** 2           # slow down in sharp turns
        v = min(v, 0.9 * goal_dist + 0.1)                      # ease in to the goal
        return v, w

    def remaining_length(self, x: float, y: float) -> float:
        if len(self.path) == 0:
            return 0.0
        seg = np.hypot(np.diff(self.path[self.idx:, 0]), np.diff(self.path[self.idx:, 1])).sum()
        return float(seg + np.hypot(self.path[self.idx, 0] - x, self.path[self.idx, 1] - y))
