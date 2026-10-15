"""Dynamic-obstacle tracking + prediction + local collision avoidance.

Why this exists: the global planner (A*) only knows where things ARE. A walker
crossing the road will be somewhere else by the time the robot gets there.
So we:
  1. TRACK moving obstacles from camera detections (nearest-neighbour association)
  2. ESTIMATE their velocity (linear fit over the last ~1 s)
  3. PREDICT their positions for the next few seconds (constant velocity)
  4. CHECK the robot's intended motion against those predictions and
       - slow down / yield if a conflict is predicted      ("YIELDING")
       - otherwise search a few safe arcs (dynamic-window)  ("EVADING")
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

from .config import Config
from .perception import DetectionResult
from .sensors import CameraFrame

HORIZON_S = 3.0
STEP_S = 0.2
TAUS = np.arange(0.0, HORIZON_S + 1e-9, STEP_S)
GATE_M = 1.0              # max jump for associating a detection to a track
UNCERTAINTY_GROWTH = 0.12   # m of extra radius per second of prediction (pedestrians can turn around)
TRACK_TTL_S = 1.5         # keep predicting a track this long after it leaves the view


@dataclass
class Track:
    tid: int
    hist: Deque[Tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=6))
    radius: float = 0.4
    last_seen: float = 0.0

    @property
    def pos(self) -> np.ndarray:
        return np.array(self.hist[-1][1:])

    def velocity(self) -> np.ndarray:
        """Least-squares velocity over the last ~0.6 s (short, so direction reversals show up quickly)."""
        if len(self.hist) < 4 or self.hist[-1][0] - self.hist[0][0] < 0.3:
            return np.zeros(2)
        h = np.array(self.hist)
        t = h[:, 0] - h[:, 0].mean()
        denom = float((t ** 2).sum())
        v = np.array([(t * (h[:, 1] - h[:, 1].mean())).sum(), (t * (h[:, 2] - h[:, 2].mean())).sum()]) / denom
        speed = float(np.hypot(*v))
        return v if speed <= 1.8 else v / speed * 1.8        # reject noisy spikes


class DynamicAvoider:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.tracks: List[Track] = []
        self._next_id = 0

    # ---------------------------------------------------------------- tracking
    def update(self, det: DetectionResult, frame: CameraFrame, t: float) -> None:
        cfg = self.cfg
        dets = [d for d in det.detections if d.cls == "dynamic"]
        used = set()
        for d in sorted(dets, key=lambda d: -d.area_px):
            p = np.array(d.world_xy)
            best, best_d = None, GATE_M
            for tr in self.tracks:
                if tr.tid in used:
                    continue
                pred = tr.pos + tr.velocity() * (t - tr.last_seen)
                dist = float(np.hypot(*(pred - p)))
                if dist < best_d:
                    best, best_d = tr, dist
            r = max(d.bbox[2], d.bbox[3]) / 2.0 / cfg.px_per_m
            if best is None:
                best = Track(self._next_id)
                self._next_id += 1
                self.tracks.append(best)
            best.hist.append((t, p[0], p[1]))
            best.radius = 0.7 * best.radius + 0.3 * max(0.25, r)
            best.last_seen = t
            used.add(best.tid)
        self.tracks = [tr for tr in self.tracks if t - tr.last_seen < TRACK_TTL_S]

    def predicted(self, t_now: float) -> Tuple[np.ndarray, np.ndarray]:
        """(n_tracks, n_taus, 2) predicted centres and (n_tracks,) radii."""
        if not self.tracks:
            return np.zeros((0, len(TAUS), 2)), np.zeros(0)
        pts, rad = [], []
        for tr in self.tracks:
            v = tr.velocity()
            base = tr.pos + v * (t_now - tr.last_seen)
            pts.append(base[None, :] + v[None, :] * TAUS[:, None])
            rad.append(tr.radius)
        return np.array(pts), np.array(rad)

    # ------------------------------------------------------------- prediction
    @staticmethod
    def _along_path(path: np.ndarray, idx: int, speeds: float) -> np.ndarray:
        """Robot positions at TAUS if it follows the path at constant speed."""
        seg = path[idx:]
        if len(seg) < 2 or speeds < 1e-3:
            return np.repeat(seg[:1], len(TAUS), axis=0) if len(seg) else np.zeros((len(TAUS), 2))
        cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])))])
        s = np.clip(speeds * TAUS, 0, cum[-1])
        return np.stack([np.interp(s, cum, seg[:, 0]), np.interp(s, cum, seg[:, 1])], axis=1)

    @staticmethod
    def _arc(pose, v: float, w: float) -> np.ndarray:
        x, y, th = pose
        if abs(w) < 1e-3:
            return np.stack([x + v * np.cos(th) * TAUS, y + v * np.sin(th) * TAUS], axis=1)
        thn = th + w * TAUS
        return np.stack([x + v / w * (np.sin(thn) - np.sin(th)),
                         y - v / w * (np.cos(thn) - np.cos(th))], axis=1)

    def _gap(self, robot_pts: np.ndarray, pred: np.ndarray, rad: np.ndarray) -> float:
        """Smallest surface-to-surface distance over all tracks and times."""
        if len(pred) == 0:
            return 99.0
        d = np.hypot(pred[:, :, 0] - robot_pts[None, :, 0], pred[:, :, 1] - robot_pts[None, :, 1])
        grow = UNCERTAINTY_GROWTH * TAUS[None, :]          # further into the future = less certain
        return float((d - rad[:, None] - grow - self.cfg.robot_radius).min())

    def _gap_n(self, robot_pts, pred, rad, n):
        d = np.hypot(pred[:, :, 0] - robot_pts[None, :, 0], pred[:, :, 1] - robot_pts[None, :, 1])
        grow = UNCERTAINTY_GROWTH * TAUS[None, :n]
        return float((d - rad[:, None] - grow - self.cfg.robot_radius).min())

    def _static_hits(self, pts: np.ndarray, occ_robot: np.ndarray) -> float:
        cfg = self.cfg
        col = np.clip((pts[:, 0] / cfg.cell_m).astype(int), 0, cfg.grid_w - 1)
        row = np.clip(((cfg.height_m - pts[:, 1]) / cfg.cell_m).astype(int), 0, cfg.grid_h - 1)
        return float(occ_robot[row, col].mean())

    # ----------------------------------------------------------------- filter
    def filter(self, t: float, pose, v: float, w: float, v_now: float, path: np.ndarray,
               idx: int, occ_robot: np.ndarray) -> Tuple[float, float, str]:
        """Return (v, w, mode) where mode is '', 'yield' or 'evade'."""
        pred, rad = self.predicted(t)
        if len(pred) == 0:
            return v, w, ""
        safe = 0.22 + 0.25 * abs(v_now)          # more room when moving fast (braking distance)
        # Fast reject: nothing within ~7 m of us -> no conflict possible
        if float(np.hypot(pred[:, 0, 0] - pose[0], pred[:, 0, 1] - pose[1]).min()) > 8.0:
            return v, w, ""
        for k in (1.0, 0.6, 0.3, 0.0):
            pts = self._along_path(path, idx, max(v, 0.0) * k)
            if self._gap(pts, pred, rad) > safe:
                return v * k, w, ("" if k == 1.0 else "yield")
        # Even standing still is predicted to be unsafe -> evasive manoeuvre.
        best, best_s = (0.0, w, "evade"), -1e9
        n = 9                                     # evasive arcs only look 1.6 s ahead
        for cv in (-0.7, -0.4, 0.0, 0.6, 1.1):
            for cw in (-1.6, -0.8, 0.0, 0.8, 1.6):
                pts = self._arc(pose, cv, cw)[:n]
                gap = self._gap_n(pts, pred[:, :n], rad, n)
                score = min(gap, 1.5) - 3.0 * self._static_hits(pts, occ_robot) - 0.15 * abs(cv - v)
                if score > best_s:
                    best_s, best = score, (cv, cw, "evade")
        return best
