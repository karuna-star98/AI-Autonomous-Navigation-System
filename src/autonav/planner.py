"""Global path planning on a 2D occupancy grid.

Three classic graph-search planners share one interface so they can be
benchmarked fairly:
  * BFS       - fewest *steps*, ignores step length and costs
  * Dijkstra  - optimal cost, explores uniformly in all directions
  * A*        - optimal cost, but guided by a heuristic -> expands far fewer nodes
"""
from __future__ import annotations

import heapq
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .geometry import bresenham

Cell = Tuple[int, int]
SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PlanResult:
    success: bool
    path: List[Cell] = field(default_factory=list)
    cost: float = math.inf
    expanded: int = 0
    time_ms: float = 0.0
    algorithm: str = ""


def _octile(a: Cell, b: Cell) -> float:
    """Admissible, consistent heuristic for 8-connected grids."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def _walk_back(parent, goal: Cell) -> List[Cell]:
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def _valid_move(blocked: np.ndarray, r: int, c: int, dr: int, dc: int) -> bool:
    """In-bounds, free target, and no corner cutting through diagonal gaps."""
    H, W = blocked.shape
    nr, nc = r + dr, c + dc
    if not (0 <= nr < H and 0 <= nc < W) or blocked[nr, nc]:
        return False
    if dr != 0 and dc != 0 and (blocked[r + dr, c] or blocked[r, c + dc]):
        return False
    return True


def astar(blocked: np.ndarray, start: Cell, goal: Cell,
          cost_mult: Optional[np.ndarray] = None, use_heuristic: bool = True) -> PlanResult:
    """A* (or Dijkstra when use_heuristic=False).

    blocked   : bool grid, True = cannot enter
    cost_mult : optional float grid >= 1 scaling the cost of entering a cell
                (used to keep paths away from obstacles)
    """
    t0 = time.perf_counter()
    name = "astar" if use_heuristic else "dijkstra"
    if blocked[start] or blocked[goal]:
        return PlanResult(False, algorithm=name, time_ms=(time.perf_counter() - t0) * 1e3)
    H, W = blocked.shape
    g = np.full((H, W), np.inf)
    g[start] = 0.0
    parent = {start: None}
    closed = np.zeros((H, W), bool)
    heap = [(_octile(start, goal) if use_heuristic else 0.0, 0.0, start)]
    expanded = 0
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        closed[cur] = True
        expanded += 1
        if cur == goal:
            return PlanResult(True, _walk_back(parent, goal), float(gc), expanded,
                              (time.perf_counter() - t0) * 1e3, name)
        r, c = cur
        for dr, dc, step in MOVES:
            if not _valid_move(blocked, r, c, dr, dc):
                continue
            nxt = (r + dr, c + dc)
            if closed[nxt]:
                continue
            ng = gc + step * (cost_mult[nxt] if cost_mult is not None else 1.0)
            if ng < g[nxt]:
                g[nxt] = ng
                parent[nxt] = cur
                h = _octile(nxt, goal) if use_heuristic else 0.0
                heapq.heappush(heap, (ng + h, ng, nxt))
    return PlanResult(False, expanded=expanded, algorithm=name,
                      time_ms=(time.perf_counter() - t0) * 1e3)


def dijkstra(blocked, start, goal, cost_mult=None) -> PlanResult:
    return astar(blocked, start, goal, cost_mult, use_heuristic=False)


def bfs(blocked: np.ndarray, start: Cell, goal: Cell) -> PlanResult:
    t0 = time.perf_counter()
    if blocked[start] or blocked[goal]:
        return PlanResult(False, algorithm="bfs")
    parent = {start: None}
    q = deque([start])
    expanded = 0
    while q:
        cur = q.popleft()
        expanded += 1
        if cur == goal:
            path = _walk_back(parent, goal)
            cost = sum(SQRT2 if (a[0] != b[0] and a[1] != b[1]) else 1.0
                       for a, b in zip(path, path[1:]))
            return PlanResult(True, path, cost, expanded, (time.perf_counter() - t0) * 1e3, "bfs")
        r, c = cur
        for dr, dc, _ in MOVES:
            nxt = (r + dr, c + dc)
            if nxt not in parent and _valid_move(blocked, r, c, dr, dc):
                parent[nxt] = cur
                q.append(nxt)
    return PlanResult(False, expanded=expanded, algorithm="bfs",
                      time_ms=(time.perf_counter() - t0) * 1e3)


def smooth_path(blocked: np.ndarray, path: List[Cell],
                strict: Optional[np.ndarray] = None) -> List[Cell]:
    """Greedy line-of-sight shortcutting ("string pulling").

    Grid paths zig-zag in 45-degree steps. If we can see a farther waypoint in
    a straight line over free cells, we skip the ones in between.

    `strict` is an optional *more conservative* map (e.g. obstacles inflated by
    one extra cell). Shortcuts are checked against it so that straight segments
    keep a safety buffer instead of grazing the keep-out boundary - the robot
    cannot track a path perfectly, so a little slack matters.
    """
    if len(path) <= 2:
        return path
    check = strict if strict is not None else blocked
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            line = bresenham(*path[i], *path[j])[1:]          # the start cell is where we already are
            if not any(check[r, c] for r, c in line):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out
