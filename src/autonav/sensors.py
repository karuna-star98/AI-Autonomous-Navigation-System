"""Simulated sensors: an egocentric camera and a 360-degree range sensor."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np

from .config import Config
from .geometry import world_to_px
from .world import World


@dataclass
class CameraFrame:
    """One camera capture plus the geometry needed to map it back to the world."""
    image: np.ndarray      # BGR, robot-centred view, forward = up
    fov_mask: np.ndarray   # 255 where the pixel is inside FOV/range and inside the map
    M: np.ndarray          # 2x3 affine: world-pixel -> camera-pixel
    pose: Tuple[float, float, float]
    cx: float              # robot position in camera image (pixels)
    cy: float

    def cam_to_world(self, cfg: Config, cu: float, cv_: float) -> Tuple[float, float]:
        """Convert a camera pixel into world metres (inverse of M)."""
        x, y, th = self.pose
        ru, rv = world_to_px(cfg, x, y)
        right = (math.sin(th), math.cos(th))       # robot's right, in image axes
        fwd = (math.cos(th), -math.sin(th))        # robot's forward, in image axes
        d_r, d_f = cu - self.cx, self.cy - cv_
        wu = ru + d_r * right[0] + d_f * fwd[0]
        wv = rv + d_r * right[1] + d_f * fwd[1]
        return wu / cfg.px_per_m, cfg.height_m - wv / cfg.px_per_m


class Camera:
    """Bird's-eye "front camera": sees a wedge ahead of the robot.

    Think of it as a simplified stand-in for a forward camera followed by an
    inverse-perspective transform - the same view autonomous cars use for
    planning. Gaussian noise is added so perception has to be robust.
    """

    def __init__(self, cfg: Config, rng: np.random.Generator):
        self.cfg, self.rng = cfg, rng
        R = cfg.cam_range_m * cfg.px_per_m
        self.w = int(2 * R)
        self.h = int(R) + 10
        self.cx, self.cy = self.w / 2.0, float(self.h - 10)
        # Static field-of-view wedge in camera coordinates.
        mask = np.zeros((self.h, self.w), np.uint8)
        half = math.radians(cfg.cam_fov_deg / 2)
        pts = [(self.cx, self.cy)]
        for a in np.linspace(-half, half, 40):
            pts.append((self.cx + R * math.sin(a), self.cy - R * math.cos(a)))
        cv2.fillPoly(mask, [np.array(pts, np.int32)], 255)
        self._fov = mask

    def capture(self, world: World, pose: Tuple[float, float, float]) -> CameraFrame:
        cfg = self.cfg
        x, y, th = pose
        ru, rv = world_to_px(cfg, x, y)
        s, c = math.sin(th), math.cos(th)
        M = np.array([[s, c, self.cx - (s * ru + c * rv)],
                      [-c, s, self.cy + (c * ru - s * rv)]], dtype=np.float64)
        img = cv2.warpAffine(world.render(include_dynamic=True), M, (self.w, self.h),
                             flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        valid = cv2.warpAffine(np.full((cfg.H, cfg.W), 255, np.uint8), M, (self.w, self.h),
                               flags=cv2.INTER_NEAREST, borderValue=0)
        fov = cv2.bitwise_and(self._fov, valid)
        noisy = img.astype(np.float32) + self.rng.normal(0, cfg.cam_noise_std, img.shape)
        img = np.clip(noisy, 0, 255).astype(np.uint8)
        return CameraFrame(img, fov, M, pose, self.cx, self.cy)


class Lidar:
    """360-degree range finder (like a cheap 2D LiDAR / ring of ultrasonic sensors)."""

    def __init__(self, cfg: Config, rng: np.random.Generator):
        self.cfg, self.rng = cfg, rng
        # Angles relative to robot heading: index 0 = straight ahead.
        self.rel_angles = np.linspace(0, 2 * math.pi, cfg.lidar_beams, endpoint=False)

    def scan(self, world: World, pose: Tuple[float, float, float]) -> np.ndarray:
        x, y, th = pose
        r = world.raycast(x, y, th + self.rel_angles, self.cfg.lidar_range_m)
        r = r + self.rng.normal(0, self.cfg.lidar_noise_std, r.shape)
        return np.clip(r, 0.0, self.cfg.lidar_range_m)
