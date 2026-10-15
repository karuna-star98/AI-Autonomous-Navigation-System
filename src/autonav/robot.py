"""Kinematic model of a differential-drive robot (a.k.a. unicycle model)."""
from __future__ import annotations

import math

import numpy as np

from .config import Config
from .geometry import wrap_angle


class Robot:
    def __init__(self, cfg: Config, x: float, y: float, theta: float,
                 rng: np.random.Generator):
        self.cfg = cfg
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0          # linear velocity  (m/s)
        self.w = 0.0          # angular velocity (rad/s)
        self.rng = rng
        self.odometer = 0.0   # total distance travelled

    @property
    def pose(self):
        return self.x, self.y, self.theta

    def step(self, v_cmd: float, w_cmd: float) -> None:
        """Apply a velocity command with acceleration limits and small actuator noise."""
        c = self.cfg
        dt = c.dt
        v_cmd = float(np.clip(v_cmd, -0.5, c.max_v))
        w_cmd = float(np.clip(w_cmd, -c.max_w, c.max_w))
        self.v += float(np.clip(v_cmd - self.v, -c.max_acc * dt, c.max_acc * dt))
        self.w += float(np.clip(w_cmd - self.w, -c.max_ang_acc * dt, c.max_ang_acc * dt))
        v = self.v * (1 + self.rng.normal(0, c.actuator_noise))
        w = self.w * (1 + self.rng.normal(0, c.actuator_noise))
        self.theta = wrap_angle(self.theta + w * dt)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.odometer += abs(v) * dt
