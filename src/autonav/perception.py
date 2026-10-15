"""Perception: turn camera pixels into obstacles, and obstacles into a map.

Two classes:
  * ObstacleDetector - classical computer vision (HSV colour segmentation +
    contours). It exposes a clean interface (frame -> detections) so a YOLO
    network can be dropped in later without touching the rest of the system.
  * OccupancyBelief  - the robot's *belief* about the world: a log-odds
    occupancy grid built only from what the camera has actually seen.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import cv2
import numpy as np

from .config import Config
from .sensors import CameraFrame

# HSV ranges (OpenCV: H 0-179, S/V 0-255)
STATIC_HSV = ((5, 120, 100), (25, 255, 255))      # orange
DYNAMIC_HSV = ((135, 80, 80), (168, 255, 255))    # magenta
MIN_BLOB_PX = 14


@dataclass
class Detection:
    cls: str                          # "static" | "dynamic"
    bbox: Tuple[int, int, int, int]   # x, y, w, h in camera pixels
    world_xy: Tuple[float, float]     # estimated centre in metres
    distance_m: float                 # range from robot
    area_px: int


@dataclass
class DetectionResult:
    static_mask: np.ndarray
    dynamic_mask: np.ndarray
    detections: List[Detection] = field(default_factory=list)


class ObstacleDetector:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._k = np.ones((3, 3), np.uint8)

    def _segment(self, hsv: np.ndarray, rng, fov: np.ndarray) -> np.ndarray:
        m = cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1]))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._k)   # remove speckle noise
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._k)  # fill small holes
        return cv2.bitwise_and(m, fov)

    def detect(self, frame: CameraFrame) -> DetectionResult:
        blur = cv2.GaussianBlur(frame.image, (3, 3), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        static = self._segment(hsv, STATIC_HSV, frame.fov_mask)
        dynamic = self._segment(hsv, DYNAMIC_HSV, frame.fov_mask)
        res = DetectionResult(static, dynamic)
        x, y, _ = frame.pose
        for cls, mask in (("static", static), ("dynamic", dynamic)):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_BLOB_PX:
                    continue
                bx, by, bw, bh = cv2.boundingRect(cnt)
                wx, wy = frame.cam_to_world(self.cfg, bx + bw / 2, by + bh / 2)
                res.detections.append(Detection(
                    cls, (bx, by, bw, bh), (wx, wy),
                    float(np.hypot(wx - x, wy - y)), int(area)))
        return res


class OccupancyBelief:
    """Log-odds occupancy grid. 0 = unknown, >0 = occupied, <0 = free."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.logodds = np.zeros((cfg.grid_h, cfg.grid_w), np.float32)
        self.observed = np.zeros((cfg.grid_h, cfg.grid_w), bool)
        self.dyn_seen = np.full((cfg.grid_h, cfg.grid_w), -1e9, np.float32)

    def _to_grid(self, cam_mask: np.ndarray, M: np.ndarray) -> np.ndarray:
        """Project a camera-frame mask into the world, then average into grid cells."""
        cfg = self.cfg
        world = cv2.warpAffine(cam_mask, M, (cfg.W, cfg.H),
                               flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP, borderValue=0)
        return cv2.resize(world, (cfg.grid_w, cfg.grid_h), interpolation=cv2.INTER_AREA) / 255.0

    def update(self, frame: CameraFrame, det: DetectionResult, t: float) -> None:
        c = self.cfg
        obs = self._to_grid(cv2.bitwise_or(det.static_mask, det.dynamic_mask), frame.M)
        dyn = self._to_grid(det.dynamic_mask, frame.M)
        fov = self._to_grid(frame.fov_mask, frame.M)
        seen = fov > 0.8                 # only update cells the camera fully covers
        occ = seen & (obs > 0.2)
        free = seen & ~occ
        self.logodds[occ] += c.l_occ
        self.logodds[free] += c.l_free
        np.clip(self.logodds, -c.l_clip, c.l_clip, out=self.logodds)
        self.observed |= seen
        self.dyn_seen[seen & (dyn > 0.15)] = t

    def occupied(self) -> np.ndarray:
        return self.logodds > 0.5

    def dynamic_recent(self, t: float) -> np.ndarray:
        return (t - self.dyn_seen) < self.cfg.dyn_memory_s

    def reset(self) -> None:
        self.logodds[:] = 0
        self.observed[:] = False
        self.dyn_seen[:] = -1e9
