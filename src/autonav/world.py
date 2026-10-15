"""Ground-truth virtual world.

The world is the *simulator's* secret truth. The robot never reads it directly -
it only sees it through the camera and lidar (see sensors.py), exactly like a
real robot would. That separation is what makes this a real autonomy pipeline
instead of a "path-finding on a known map" demo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from .config import Config
from .geometry import world_to_px

# BGR colours. Obstacle colours are deliberately distinct in HSV so the
# classical detector (perception.py) can segment them from the grey floor.
FLOOR_BGR = (238, 236, 232)
GRIDLINE_BGR = (222, 220, 216)
STATIC_BGR = (30, 140, 255)      # orange  -> static obstacle (shelf, building, wall, cone)
STATIC_EDGE = (10, 95, 205)
DYNAMIC_BGR = (190, 60, 200)     # magenta -> moving obstacle (pedestrian, forklift)
DYNAMIC_EDGE = (140, 30, 150)


@dataclass
class StaticRect:
    """Axis-aligned rectangle; (x, y) is the lower-left corner in metres."""
    x: float
    y: float
    w: float
    h: float


class DynamicObstacle:
    """A circular obstacle that ping-pongs along a polyline at constant speed."""

    def __init__(self, waypoints: Sequence[Tuple[float, float]], speed: float,
                 radius: float, phase_s: float = 0.0):
        assert len(waypoints) >= 2, "need at least two waypoints"
        self.wp = np.array(waypoints, dtype=float)
        self.speed = speed
        self.radius = radius
        self.idx = 0           # index of the last waypoint reached
        self.dir = 1           # +1 forward along the list, -1 backward
        self.pos = self.wp[0].copy()
        self.advance(speed * phase_s)

    def advance(self, dist: float) -> None:
        while dist > 1e-9:
            target = self.wp[self.idx + self.dir]
            d = float(np.linalg.norm(target - self.pos))
            if d <= dist:
                self.pos = target.copy()
                dist -= d
                self.idx += self.dir
                if not (0 <= self.idx + self.dir < len(self.wp)):
                    self.dir = -self.dir
            else:
                self.pos = self.pos + (target - self.pos) / d * dist
                dist = 0.0

    def step(self, dt: float) -> None:
        self.advance(self.speed * dt)


class World:
    def __init__(self, cfg: Config, name: str, rects: List[StaticRect],
                 dynamics: List[DynamicObstacle], start: Tuple[float, float],
                 goal: Tuple[float, float], start_theta: float = 0.0):
        self.cfg = cfg
        self.name = name
        self.dynamics = dynamics
        self.start = start
        self.goal = goal
        self.start_theta = start_theta
        t = 0.4  # boundary wall thickness
        W, H = cfg.width_m, cfg.height_m
        walls = [StaticRect(0, 0, W, t), StaticRect(0, H - t, W, t),
                 StaticRect(0, 0, t, H), StaticRect(W - t, 0, t, H)]
        self.rects = walls + list(rects)
        self._static_img = self._render_static()
        self._static_mask = self._render_static_mask()
        self._dist = None
        self._rebuild_mask()

    # ------------------------------------------------------------------ render
    def _render_static(self) -> np.ndarray:
        cfg = self.cfg
        img = np.full((cfg.H, cfg.W, 3), FLOOR_BGR, np.uint8)
        for xm in range(0, int(cfg.width_m) + 1):          # faint 1 m grid lines
            u = int(xm * cfg.px_per_m)
            cv2.line(img, (u, 0), (u, cfg.H), GRIDLINE_BGR, 1)
        for ym in range(0, int(cfg.height_m) + 1):
            v = int(ym * cfg.px_per_m)
            cv2.line(img, (0, v), (cfg.W, v), GRIDLINE_BGR, 1)
        for r in self.rects:
            u0, v1 = world_to_px(cfg, r.x, r.y)
            u1, v0 = world_to_px(cfg, r.x + r.w, r.y + r.h)
            p0, p1 = (int(round(u0)), int(round(v0))), (int(round(u1)) - 1, int(round(v1)) - 1)
            cv2.rectangle(img, p0, p1, STATIC_BGR, -1)
            cv2.rectangle(img, p0, p1, STATIC_EDGE, 1)
        return img

    def _render_static_mask(self) -> np.ndarray:
        cfg = self.cfg
        m = np.zeros((cfg.H, cfg.W), np.uint8)
        for r in self.rects:
            u0, v1 = world_to_px(cfg, r.x, r.y)
            u1, v0 = world_to_px(cfg, r.x + r.w, r.y + r.h)
            cv2.rectangle(m, (int(round(u0)), int(round(v0))),
                          (int(round(u1)) - 1, int(round(v1)) - 1), 255, -1)
        return m

    def render(self, include_dynamic: bool = True) -> np.ndarray:
        """BGR image of the world (without the robot)."""
        img = self._static_img.copy()
        if include_dynamic:
            for d in self.dynamics:
                u, v = world_to_px(self.cfg, *d.pos)
                rp = int(round(d.radius * self.cfg.px_per_m))
                cv2.circle(img, (int(round(u)), int(round(v))), rp, DYNAMIC_BGR, -1, cv2.LINE_AA)
                cv2.circle(img, (int(round(u)), int(round(v))), rp, DYNAMIC_EDGE, 1, cv2.LINE_AA)
        return img

    def _rebuild_mask(self) -> None:
        m = self._static_mask.copy()
        for d in self.dynamics:
            u, v = world_to_px(self.cfg, *d.pos)
            cv2.circle(m, (int(round(u)), int(round(v))),
                       int(round(d.radius * self.cfg.px_per_m)), 255, -1)
        self.mask = m
        self._dist = None

    # --------------------------------------------------------------- dynamics
    def step(self, dt: float) -> None:
        for d in self.dynamics:
            d.step(dt)
        if self.dynamics:
            self._rebuild_mask()

    # ---------------------------------------------------------- ground truth q
    def collides(self, x: float, y: float, r: float) -> bool:
        """Does a circle of radius r at (x, y) overlap any obstacle?"""
        cfg = self.cfg
        u, v = world_to_px(cfg, x, y)
        if not (0 <= u < cfg.W and 0 <= v < cfg.H):
            return True
        rp = r * cfg.px_per_m
        x0, x1 = max(0, int(u - rp) - 1), min(cfg.W, int(u + rp) + 2)
        y0, y1 = max(0, int(v - rp) - 1), min(cfg.H, int(v + rp) + 2)
        crop = self.mask[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        circ = (xx - u) ** 2 + (yy - v) ** 2 <= rp ** 2
        return bool(np.any(crop[circ]))

    def clearance(self, x: float, y: float) -> float:
        """Distance (m) from point to the nearest obstacle surface."""
        if self._dist is None:
            self._dist = cv2.distanceTransform(255 - self.mask, cv2.DIST_L2, 3)
        u, v = world_to_px(self.cfg, x, y)
        u = min(max(int(u), 0), self.cfg.W - 1)
        v = min(max(int(v), 0), self.cfg.H - 1)
        return float(self._dist[v, u]) / self.cfg.px_per_m

    def raycast(self, x: float, y: float, angles: np.ndarray, max_range: float,
                step_m: float = 0.05) -> np.ndarray:
        """Distance to first obstacle along each world-frame angle (vectorised)."""
        cfg = self.cfg
        ts = np.arange(step_m, max_range + step_m, step_m)
        dx = np.cos(angles)[:, None] * ts[None, :]
        dy = np.sin(angles)[:, None] * ts[None, :]
        u = np.round((x + dx) * cfg.px_per_m).astype(int)
        v = np.round((cfg.height_m - (y + dy)) * cfg.px_per_m).astype(int)
        inside = (u >= 0) & (u < cfg.W) & (v >= 0) & (v < cfg.H)
        hit = ~inside
        uc, vc = np.clip(u, 0, cfg.W - 1), np.clip(v, 0, cfg.H - 1)
        hit |= (self.mask[vc, uc] > 0) & inside
        any_hit = hit.any(axis=1)
        first = hit.argmax(axis=1)
        return np.where(any_hit, ts[first], max_range)

    def occupancy_grid(self) -> np.ndarray:
        """Ground-truth *static* occupancy at planning resolution (bool, row 0 = top)."""
        cfg = self.cfg
        frac = cv2.resize(self._static_mask, (cfg.grid_w, cfg.grid_h),
                          interpolation=cv2.INTER_AREA) / 255.0
        return frac > 0.15
