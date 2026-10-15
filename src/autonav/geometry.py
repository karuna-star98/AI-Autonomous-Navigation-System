"""Small geometry helpers shared by all modules.

Conventions (important - most beginner bugs come from mixing these up!):
  * WORLD  : metres, x to the right, y UP, heading theta counter-clockwise from +x.
  * PIXEL  : image coordinates (u right, v DOWN) of the rendered world.
  * CELL   : planning grid (row, col); row 0 is the TOP of the image, so grids
             line up exactly with images and cv2.resize works without flipping.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from .config import Config


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def world_to_px(cfg: Config, x: float, y: float) -> Tuple[float, float]:
    return x * cfg.px_per_m, (cfg.height_m - y) * cfg.px_per_m


def px_to_world(cfg: Config, u: float, v: float) -> Tuple[float, float]:
    return u / cfg.px_per_m, cfg.height_m - v / cfg.px_per_m


def world_to_cell(cfg: Config, x: float, y: float) -> Tuple[int, int]:
    col = int(x / cfg.cell_m)
    row = int((cfg.height_m - y) / cfg.cell_m)
    return (min(max(row, 0), cfg.grid_h - 1), min(max(col, 0), cfg.grid_w - 1))


def cell_to_world(cfg: Config, row: int, col: int) -> Tuple[float, float]:
    return (col + 0.5) * cfg.cell_m, cfg.height_m - (row + 0.5) * cfg.cell_m


def bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    """All grid cells on the straight line between two cells (inclusive)."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


def disk_kernel(radius_cells: float) -> np.ndarray:
    """Boolean disk used to inflate obstacles by the robot size."""
    k = int(math.ceil(radius_cells))
    yy, xx = np.ogrid[-k:k + 1, -k:k + 1]
    return ((xx ** 2 + yy ** 2) <= radius_cells ** 2 + 0.25).astype(np.uint8)
