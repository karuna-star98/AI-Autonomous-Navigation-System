"""Dashboard renderer (pure OpenCV, so it works headless and in a live window)."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from .config import Config
from .geometry import world_to_px
from .navigator import Navigator, State
from .world import World

BG = (34, 30, 30)
PANEL_BG = (46, 42, 42)
WHITE = (240, 240, 240)
GREY = (170, 170, 170)
PATH_C = (200, 70, 20)
TRAIL_C = (80, 175, 60)
ROBOT_C = (225, 150, 20)
GOAL_C = (60, 180, 60)
STATE_COLOR = {
    State.NAVIGATING: (90, 200, 90), State.EMERGENCY_STOP: (60, 60, 235),
    State.YIELDING: (40, 190, 250), State.EVADING: (40, 120, 250),
    State.RECOVERING: (50, 160, 250), State.WAITING: (50, 200, 240),
    State.GOAL: (90, 220, 90), State.FAILED: (60, 60, 235),
}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, s, org, scale=0.5, color=WHITE, thick=1):
    cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)


def _px(cfg: Config, x: float, y: float) -> Tuple[int, int]:
    u, v = world_to_px(cfg, x, y)
    return int(round(u)), int(round(v))


def _draw_robot(img, cfg: Config, pose, color=ROBOT_C):
    x, y, th = pose
    c = _px(cfg, x, y)
    r = int(round(cfg.robot_radius * cfg.px_per_m))
    cv2.circle(img, c, r, color, -1, cv2.LINE_AA)
    cv2.circle(img, c, r, (40, 40, 40), 1, cv2.LINE_AA)
    tip = _px(cfg, x + 0.55 * math.cos(th), y + 0.55 * math.sin(th))
    cv2.arrowedLine(img, c, tip, (30, 30, 30), 2, cv2.LINE_AA, tipLength=0.45)


def _draw_path(img, cfg, path_xy, color=PATH_C, thick=2):
    if path_xy is None or len(path_xy) < 2:
        return
    pts = np.array([_px(cfg, *p) for p in path_xy], np.int32)
    cv2.polylines(img, [pts], False, color, thick, cv2.LINE_AA)


def _draw_goal(img, cfg, goal):
    c = _px(cfg, *goal)
    cv2.circle(img, c, int(cfg.goal_tol_m * cfg.px_per_m), GOAL_C, 2, cv2.LINE_AA)
    cv2.circle(img, c, 4, GOAL_C, -1, cv2.LINE_AA)
    _text(img, "GOAL", (c[0] - 40, c[1] - 12), 0.45, (40, 130, 40), 1)


def world_panel(cfg: Config, world: World, pose, nav: Navigator, ranges, trail) -> np.ndarray:
    img = world.render()
    x, y, th = pose
    # camera field-of-view wedge
    half = math.radians(cfg.cam_fov_deg / 2)
    pts = [_px(cfg, x, y)] + [_px(cfg, x + cfg.cam_range_m * math.cos(th + a),
                                  y + cfg.cam_range_m * math.sin(th + a))
                              for a in np.linspace(-half, half, 30)]
    overlay = img.copy()
    cv2.fillPoly(overlay, [np.array(pts, np.int32)], (120, 220, 255))
    img = cv2.addWeighted(overlay, 0.22, img, 0.78, 0)
    # lidar rays
    n = len(ranges)
    for i, r in enumerate(ranges):
        a = th + 2 * math.pi * i / n
        end = _px(cfg, x + r * math.cos(a), y + r * math.sin(a))
        hit = r < cfg.lidar_range_m - 0.05
        cv2.line(img, _px(cfg, x, y), end, (60, 60, 230) if hit else (200, 180, 120), 1, cv2.LINE_AA)
    if len(trail) > 1:
        cv2.polylines(img, [np.array([_px(cfg, a, b) for a, b in trail], np.int32)],
                      False, TRAIL_C, 2, cv2.LINE_AA)
    _draw_path(img, cfg, nav.path_xy)
    cv2.circle(img, _px(cfg, *world.start), 5, (200, 120, 40), -1, cv2.LINE_AA)
    _text(img, "START", (_px(cfg, *world.start)[0] - 18, _px(cfg, *world.start)[1] + 22), 0.4, (150, 90, 30))
    _draw_goal(img, cfg, world.goal)
    _draw_robot(img, cfg, pose)
    return img


def belief_panel(cfg: Config, nav: Navigator, pose, trail) -> np.ndarray:
    b = nav.belief
    img = np.full((cfg.grid_h, cfg.grid_w, 3), 175, np.uint8)       # unknown = grey
    free = b.observed & (b.logodds <= 0.5)
    img[free] = (250, 250, 250)
    inflated = nav.plan_map & ~b.occupied()
    img[inflated & b.observed] = (205, 205, 255)                      # keep-out margin
    img[inflated & ~b.observed] = (190, 190, 225)
    img[b.occupied()] = (40, 40, 40)
    img = cv2.resize(img, (cfg.W, cfg.H), interpolation=cv2.INTER_NEAREST)
    if len(trail) > 1:
        cv2.polylines(img, [np.array([_px(cfg, a, b_) for a, b_ in trail], np.int32)],
                      False, TRAIL_C, 2, cv2.LINE_AA)
    _draw_path(img, cfg, nav.path_xy)
    _draw_goal(img, cfg, nav.goal)
    _draw_robot(img, cfg, pose)
    return img


def camera_panel(nav: Navigator, scale: float = 1.6) -> np.ndarray:
    fr, det = nav.last_frame, nav.last_det
    if fr is None:
        return np.full((int(170 * scale), int(280 * scale), 3), 60, np.uint8)
    img = fr.image.copy()
    dim = (img * 0.35).astype(np.uint8)
    m = fr.fov_mask > 0
    dim[m] = img[m]
    img = cv2.resize(dim, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    cv2.circle(img, (int(fr.cx * scale), int(fr.cy * scale)), 6, ROBOT_C, -1, cv2.LINE_AA)
    if det is not None:
        for d in det.detections:
            x0, y0, w, h = [int(v * scale) for v in d.bbox]
            col = (50, 50, 235) if d.cls == "dynamic" else (60, 200, 60)
            cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), col, 2)
            _text(img, f"{d.cls} {d.distance_m:.1f}m", (x0, max(12, y0 - 4)), 0.42, col, 1)
    return img


def _sparkline(w: int, h: int, ts: Sequence[float], vs: Sequence[float], vmax: float, window=20.0):
    img = np.full((h, w, 3), PANEL_BG, np.uint8)
    cv2.rectangle(img, (0, 0), (w - 1, h - 1), (90, 90, 90), 1)
    if len(ts) > 2:
        t1 = ts[-1]
        pts = [(int((t - (t1 - window)) / window * (w - 6)) + 3, int(h - 6 - (v / vmax) * (h - 14)))
               for t, v in zip(ts, vs) if t >= t1 - window]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, (90, 200, 90), 2, cv2.LINE_AA)
    _text(img, "speed (m/s), last 20 s", (6, 14), 0.4, GREY)
    return img


def compose(cfg: Config, world: World, pose, nav: Navigator, ranges, trail, t: float,
            hist_t, hist_v, stats: dict, scenario: str) -> np.ndarray:
    pw, ph, gap, top = cfg.W, cfg.H, 10, 38
    bottom_h = 270
    W = 2 * pw + 3 * gap
    H = top + ph + gap + bottom_h + gap
    canvas = np.full((H, W, 3), BG, np.uint8)
    _text(canvas, "AI-Based Autonomous Navigation System", (gap, 25), 0.7, WHITE, 2)
    _text(canvas, f"scenario: {scenario}   |   perception: OpenCV   planner: A*   safety: lidar",
          (W - 640, 25), 0.5, GREY)

    wp = world_panel(cfg, world, pose, nav, ranges, trail)
    bp = belief_panel(cfg, nav, pose, trail)
    canvas[top:top + ph, gap:gap + pw] = wp
    canvas[top:top + ph, 2 * gap + pw:2 * gap + 2 * pw] = bp
    _text(canvas, "SIMULATOR (ground truth)", (gap + 8, top + 18), 0.5, (60, 60, 60), 1)
    _text(canvas, "ROBOT'S BELIEF MAP (what it has seen)", (2 * gap + pw + 8, top + 18), 0.5, (60, 60, 60), 1)

    y0 = top + ph + gap
    cp = camera_panel(nav)
    ch, cw = cp.shape[:2]
    canvas[y0:y0 + min(ch, bottom_h), gap:gap + cw] = cp[:bottom_h]
    _text(canvas, "ONBOARD CAMERA + DETECTIONS", (gap + 6, y0 + 16), 0.45, WHITE, 1)

    # telemetry text
    tx = 2 * gap + cw + 10
    col = STATE_COLOR.get(nav.state, WHITE)
    _text(canvas, "STATE", (tx, y0 + 22), 0.5, GREY)
    _text(canvas, nav.state.value, (tx + 62, y0 + 22), 0.6, col, 2)
    rows = [
        ("time", f"{t:6.1f} s"),
        ("speed v / w", f"{stats['v']:.2f} m/s  {stats['w']:+.2f} rad/s"),
        ("dist to goal", f"{stats['goal_dist']:.2f} m"),
        ("distance travelled", f"{stats['odometer']:.1f} m"),
        ("re-plans", f"{nav.replans}"),
        ("yields / e-stops", f"{nav.yields} / {nav.estops}"),
        ("min clearance", f"{stats['min_clear']:.2f} m"),
        ("collisions", f"{stats['collisions']}"),
        ("last plan", (f"{nav.last_plan.algorithm} {nav.last_plan.expanded} nodes "
                       f"{nav.last_plan.time_ms:.0f} ms") if nav.last_plan else "-"),
    ]
    for i, (k, v) in enumerate(rows):
        yy = y0 + 52 + i * 24
        _text(canvas, k, (tx, yy), 0.47, GREY)
        _text(canvas, v, (tx + 165, yy), 0.5, (255, 80, 80) if (k == "collisions" and stats['collisions']) else WHITE)

    sx = tx + 440
    sw = W - sx - gap
    spark = _sparkline(sw, 110, hist_t, hist_v, cfg.max_v * 1.1)
    canvas[y0 + 8:y0 + 8 + 110, sx:sx + sw] = spark
    # tiny legend
    ly = y0 + 150
    for i, (name, color) in enumerate([("static obstacle", (30, 140, 255)), ("moving obstacle", (190, 60, 200)),
                                       ("planned path", PATH_C), ("robot trail", TRAIL_C),
                                       ("keep-out margin", (205, 205, 255))]):
        cv2.rectangle(canvas, (sx, ly + i * 22), (sx + 14, ly + 12 + i * 22), color, -1)
        _text(canvas, name, (sx + 22, ly + 11 + i * 22), 0.45, WHITE)
    return canvas
