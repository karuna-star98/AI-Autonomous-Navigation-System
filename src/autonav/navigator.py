"""Navigator: the decision-making brain.

Each control tick it:
  1. (at camera rate) updates the belief map from perception
  2. decides whether the current path is still safe -> re-plans if not
  3. asks the controller for a velocity command
  4. passes that command through a lidar-based SAFETY LAYER that can slow or stop
     the robot regardless of what the planner believes

State machine:
  NAVIGATING -> EMERGENCY_STOP -> (RECOVERING) -> NAVIGATING
  NAVIGATING -> WAITING_FOR_PATH (no route found) -> NAVIGATING | FAILED
  NAVIGATING -> GOAL_REACHED
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import Config
from .avoidance import DynamicAvoider
from .controller import PathFollower
from .geometry import cell_to_world, disk_kernel, world_to_cell
from .perception import DetectionResult, ObstacleDetector, OccupancyBelief
from .planner import PlanResult, astar, smooth_path
from .sensors import CameraFrame


class State(str, Enum):
    NAVIGATING = "NAVIGATING"
    YIELDING = "YIELDING (moving obstacle)"
    EVADING = "EVADING (moving obstacle)"
    EMERGENCY_STOP = "EMERGENCY STOP"
    RECOVERING = "RECOVERING (reverse)"
    WAITING = "WAITING FOR PATH"
    GOAL = "GOAL REACHED"
    FAILED = "FAILED"


@dataclass
class Event:
    t: float
    kind: str          # "replan" | "estop" | "recover"
    detail: str = ""


class Navigator:
    def __init__(self, cfg: Config, goal: Tuple[float, float]):
        self.cfg = cfg
        self.goal = goal
        self.detector = ObstacleDetector(cfg)
        self.belief = OccupancyBelief(cfg)
        self.follower = PathFollower(cfg)
        self.avoider = DynamicAvoider(cfg)
        self.occ_robot = np.zeros((cfg.grid_h, cfg.grid_w), bool)
        self.yields = 0
        self._avoid_mode = ""
        self._kernel_robot = disk_kernel((cfg.robot_radius + 0.05) / cfg.cell_m)
        self.state = State.NAVIGATING
        self.path_xy: Optional[np.ndarray] = None
        self.plan_map = np.zeros((cfg.grid_h, cfg.grid_w), bool)
        self.last_det: Optional[DetectionResult] = None
        self.last_frame: Optional[CameraFrame] = None
        self.last_plan: Optional[PlanResult] = None
        self.events: List[Event] = []
        self.plan_times_ms: List[float] = []
        self.plan_expanded: List[int] = []
        self.replans = 0
        self.estops = 0
        self.detections_total = 0
        self.failure_reason = ""
        self._map_dirty = True
        self._t_last_plan = -1e9
        self._t_wait_start: Optional[float] = None
        self._wait_reset_done = False
        self._t_stop_start: Optional[float] = None
        self._t_recover_end = 0.0
        self._pos_hist: List[Tuple[float, float, float]] = []
        self._kernel_inflate = disk_kernel(cfg.inflation_m / cfg.cell_m)
        self._kernel_dyn = disk_kernel((cfg.inflation_m + cfg.dyn_extra_clearance) / cfg.cell_m)

    # ------------------------------------------------------------ perception
    def perceive(self, frame: CameraFrame, t: float) -> None:
        det = self.detector.detect(frame)
        self.belief.update(frame, det, t)
        self.last_det, self.last_frame = det, frame
        self.avoider.update(det, frame, t)
        self.detections_total += len(det.detections)
        self._map_dirty = True

    # -------------------------------------------------------------- planning
    def _build_plan_map(self, pose, t: float):
        """Occupied cells -> inflate by robot size (+ extra for moving things)."""
        cfg = self.cfg
        occ = self.belief.occupied()
        dyn = occ & self.belief.dynamic_recent(t)
        blocked = cv2.dilate(occ.astype(np.uint8), self._kernel_inflate).astype(bool)
        blocked |= cv2.dilate(dyn.astype(np.uint8), self._kernel_dyn).astype(bool)
        # The robot's own footprint is physically free: don't let inflation trap it.
        r, c = world_to_cell(cfg, pose[0], pose[1])
        k = int(math.ceil(cfg.inflation_m / cfg.cell_m))
        r0, r1, c0, c1 = max(0, r - k), r + k + 1, max(0, c - k), c + k + 1
        blocked[r0:r1, c0:c1] = occ[r0:r1, c0:c1]
        # Cost field: prefer paths with clearance.
        dist_m = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * cfg.cell_m
        prox = np.clip((cfg.proximity_range_m - dist_m) / cfg.proximity_range_m, 0, 1)
        cost = 1.0 + cfg.proximity_weight * prox
        self.plan_map = blocked
        self.occ_robot = cv2.dilate(occ.astype(np.uint8), self._kernel_robot).astype(bool)
        return blocked, cost, dyn.any()

    def _path_blocked(self, pose) -> bool:
        """Is any cell of the next few metres of path now inside an inflated obstacle?"""
        if self.path_xy is None:
            return True
        cfg = self.cfg
        i = self.follower.idx
        acc = 0.0
        while i < len(self.path_xy) - 1 and acc < cfg.horizon_m:
            r, c = world_to_cell(cfg, *self.path_xy[i])
            if self.plan_map[r, c]:
                return True
            acc += float(np.hypot(*(self.path_xy[i + 1] - self.path_xy[i])))
            i += 1
        return False

    def _plan(self, pose, t: float, reason: str, cost) -> bool:
        cfg = self.cfg
        start = world_to_cell(cfg, pose[0], pose[1])
        goal = world_to_cell(cfg, *self.goal)
        blocked = self.plan_map
        if blocked[goal]:                                 # never trap the goal itself
            r, c = goal
            blocked[max(0, r - 2):r + 3, max(0, c - 2):c + 3] = False
        res = astar(blocked, start, goal, cost)
        self._t_last_plan = t
        self.last_plan = res
        self.plan_times_ms.append(res.time_ms)
        self.plan_expanded.append(res.expanded)
        if not res.success:
            self.path_xy = None
            return False
        strict = cv2.dilate(blocked.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        cells = smooth_path(blocked, res.path, strict)
        pts = [np.array([pose[0], pose[1]])] + [np.array(cell_to_world(cfg, r, c))
                                               for r, c in cells[1:-1]] + [np.array(self.goal)]
        self.path_xy = self._densify(np.array(pts), 0.25)
        self.follower.set_path(self.path_xy)
        self.replans += 1
        self.events.append(Event(t, "replan", reason))
        return True

    @staticmethod
    def _densify(pts: np.ndarray, spacing: float) -> np.ndarray:
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(math.ceil(float(np.hypot(*(b - a))) / spacing)))
            for k in range(1, n + 1):
                out.append(a + (b - a) * k / n)
        return np.array(out)

    # --------------------------------------------------------------- control
    def _sector_min(self, ranges: np.ndarray, centre_deg: float, half_deg: float) -> float:
        n = len(ranges)
        ang = (np.arange(n) * 360.0 / n - centre_deg + 180.0) % 360.0 - 180.0
        sel = np.abs(ang) <= half_deg
        return float(ranges[sel].min()) if sel.any() else float(ranges.min())

    def act(self, t: float, pose, ranges: np.ndarray, v_now: float = 0.0) -> Tuple[float, float]:
        """Return the (v, w) command for this tick."""
        cfg = self.cfg
        if self.state in (State.GOAL, State.FAILED):
            return 0.0, 0.0
        x, y, _ = pose
        goal_dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if goal_dist < cfg.goal_tol_m:
            self.state = State.GOAL
            return 0.0, 0.0

        # ---- 1. refresh map + decide whether to (re)plan ----
        need_plan, reason, cost = (self.path_xy is None and self.state != State.WAITING), "initial/no path", None
        if self._map_dirty:
            _, cost, dyn_near = self._build_plan_map(pose, t)
            self._map_dirty = False
            if self.path_xy is not None and self._path_blocked(pose):
                need_plan, reason = True, "path blocked by new obstacle"
            elif dyn_near and t - self._t_last_plan > cfg.replan_period_s:
                need_plan, reason = True, "periodic (moving obstacles nearby)"
        if self.state == State.WAITING and t - self._t_last_plan > 0.5:
            need_plan, reason = True, "retry after no path"
        if self._t_recover_end and t >= self._t_recover_end and self.state == State.RECOVERING:
            need_plan, reason = True, "after recovery"
        if need_plan:
            if cost is None:
                _, cost, _ = self._build_plan_map(pose, t)
            ok = self._plan(pose, t, reason, cost)
            if not ok:
                if self.state != State.WAITING:
                    self.state = State.WAITING
                    self._t_wait_start = t
                    self._wait_reset_done = False
                waited = t - (self._t_wait_start if self._t_wait_start is not None else t)
                if waited > 2 * cfg.no_path_timeout_s:
                    self.state = State.FAILED
                    self.failure_reason = "no path to goal"
                elif waited > cfg.no_path_timeout_s and not self._wait_reset_done:
                    self.belief.reset()          # last resort: forget stale map, re-explore
                    self._wait_reset_done = True
                    self.events.append(Event(t, "recover", "belief reset"))
                return 0.0, 0.0
            if self.state in (State.WAITING, State.RECOVERING):
                self.state = State.NAVIGATING

        if self.path_xy is None:
            return 0.0, 0.0

        # ---- 2. recovery manoeuvre ----
        rear_gap = self._sector_min(ranges, 180, 40) - cfg.robot_radius
        if self.state == State.RECOVERING:
            if t < self._t_recover_end and rear_gap > 0.25:
                return -0.3, 0.0
            self.state = State.NAVIGATING

        # ---- 3. path following ----
        v, w = self.follower.command(pose, goal_dist)

        # ---- 3a. lidar slow-down factor (applied at the end as a MIN with the avoider) ----
        front_gap = self._sector_min(ranges, 0, cfg.front_sector_deg) - cfg.robot_radius
        scale = float(np.clip((front_gap - cfg.stop_gap_m) / (cfg.slow_gap_m - cfg.stop_gap_m), 0.15, 1.0))
        v_scaled = v * scale
        v_plan = v

        # ---- 3b. moving-obstacle prediction: yield or evade if a conflict is predicted ----
        # (predict with the controller's intended speed: slowing down *because* a mover is
        #  close would otherwise make the robot predict that it lingers in the mover's lane)
        v_f, w, mode = self.avoider.filter(t, pose, v_plan, w, v_now, self.path_xy,
                                           self.follower.idx, self.occ_robot)
        v = v_f if v_f < 0 else min(v_f, v_scaled)
        if mode and mode != self._avoid_mode:
            self.yields += 1
            self.events.append(Event(t, "yield" if mode == "yield" else "evade", mode))
        self._avoid_mode = mode

        # ---- 4. safety layer (never trust the planner blindly) ----
        reversing_away = (mode == "evade" and v < 0 and rear_gap > 0.2)   # moving AWAY is always allowed
        if front_gap < cfg.stop_gap_m and not reversing_away:
            if self.state != State.EMERGENCY_STOP:
                self.estops += 1
                self._t_stop_start = t
                self.events.append(Event(t, "estop", f"front gap {front_gap:.2f} m"))
            self.state = State.EMERGENCY_STOP
            if t - (self._t_stop_start or t) > 3.0 and rear_gap > 0.6:   # stuck: back off
                self.state = State.RECOVERING
                self._t_recover_end = t + 1.2
                self.events.append(Event(t, "recover", "reversing"))
            # Forward motion is blocked, but turning on the spot is always safe
            # for a round robot, so we keep steering toward the path.
            return 0.0, w
        if self.state == State.EMERGENCY_STOP:
            self.state = State.NAVIGATING
            self._map_dirty = True
        if self.state in (State.NAVIGATING, State.YIELDING, State.EVADING):
            self.state = {"yield": State.YIELDING, "evade": State.EVADING}.get(mode, State.NAVIGATING)

        # ---- 5. stuck detection (progress watchdog) ----
        self._pos_hist.append((t, x, y))
        while self._pos_hist and t - self._pos_hist[0][0] > 8.0:
            self._pos_hist.pop(0)
        if (self.state == State.NAVIGATING and self._pos_hist and t - self._pos_hist[0][0] > 7.5
                and math.hypot(x - self._pos_hist[0][1], y - self._pos_hist[0][2]) < 0.15):
            self.path_xy = None            # forces a fresh plan next tick
            self._pos_hist.clear()
            self.events.append(Event(t, "recover", "no progress -> replan"))
        return v, w
