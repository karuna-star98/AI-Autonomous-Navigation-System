"""Ready-made virtual environments.

  warehouse : shelf aisles + a moving forklift      (indoor logistics robot)
  street    : city blocks + crossing pedestrians    (delivery robot / AV)
  random    : seeded random clutter + movers        (for batch evaluation)
"""
from __future__ import annotations

from typing import Callable, Dict, List

import numpy as np

from .config import Config
from .planner import astar
from .geometry import world_to_cell
from .world import DynamicObstacle, StaticRect, World
import cv2


def warehouse(cfg: Config, seed: int = 0) -> World:
    rng = np.random.default_rng(seed)
    S = StaticRect
    rects = [
        S(5.0, 0.4, 1.2, 8.6), S(5.0, 12.0, 1.2, 7.6),       # shelf column 1 (gap y 9..12)
        S(10.0, 4.0, 1.2, 12.0),                               # shelf column 2
        S(15.0, 0.4, 1.2, 10.0), S(15.0, 13.0, 1.2, 6.6),     # shelf column 3 (gap y 10..13)
        S(20.0, 5.0, 1.2, 11.0),                               # shelf column 4
        S(24.5, 0.4, 1.2, 7.0), S(24.5, 11.0, 1.2, 8.6),      # shelf column 5 (gap y 7.4..11)
        S(2.5, 6.0, 1.5, 1.0), S(12.8, 17.0, 1.0, 1.0),       # loose pallets
        S(17.8, 2.5, 1.0, 1.0), S(7.8, 14.0, 1.0, 1.0),
    ]
    dyn = [
        DynamicObstacle([(8.1, 2.0), (8.1, 17.5)], 0.7, 0.55, phase_s=rng.uniform(0, 10)),   # forklift
        DynamicObstacle([(12.8, 1.5), (12.8, 10.0)], 0.6, 0.35, phase_s=rng.uniform(0, 10)),  # worker
        DynamicObstacle([(17.8, 18.0), (17.8, 6.0)], 0.6, 0.35, phase_s=rng.uniform(0, 10)),  # worker
    ]
    return World(cfg, "warehouse", rects, dyn, start=(1.8, 2.0), goal=(28.0, 17.8), start_theta=0.0)


def street(cfg: Config, seed: int = 0) -> World:
    rng = np.random.default_rng(seed)
    S = StaticRect
    rects = [
        S(3.5, 2.0, 5.0, 5.0), S(3.5, 13.0, 5.0, 5.0),        # building blocks
        S(11.0, 2.0, 6.0, 5.0), S(11.0, 13.0, 6.0, 5.0),
        S(20.0, 2.0, 4.0, 5.0), S(20.0, 13.0, 5.0, 5.0),
        S(26.0, 2.0, 2.5, 5.0),
        S(13.5, 9.3, 0.5, 0.5), S(15.8, 10.8, 0.5, 0.5),       # traffic cones (kept off the pedestrian lanes)
        S(26.0, 9.5, 0.5, 0.5),
    ]
    dyn = [
        DynamicObstacle([(9.8, 7.4), (9.8, 12.6)], 0.8, 0.3, phase_s=rng.uniform(0, 6)),     # pedestrians
        DynamicObstacle([(18.8, 12.6), (18.8, 7.4)], 0.7, 0.3, phase_s=rng.uniform(0, 6)),
        DynamicObstacle([(23.0, 7.4), (23.0, 12.6)], 0.9, 0.3, phase_s=rng.uniform(0, 6)),
    ]
    return World(cfg, "street", rects, dyn, start=(1.6, 10.0), goal=(28.3, 15.5), start_theta=0.0)


def random_field(cfg: Config, seed: int = 0, n_static: int = 20, n_dynamic: int = 3) -> World:
    """Random clutter; retries until the ground-truth map is actually solvable."""
    rng = np.random.default_rng(1000 + seed)
    start, goal = (1.8, 1.8), (cfg.width_m - 1.8, cfg.height_m - 1.8)
    for _ in range(300):
        rects: List[StaticRect] = []
        for _ in range(n_static):
            w, h = rng.uniform(0.8, 2.8), rng.uniform(0.8, 2.8)
            x = rng.uniform(1.0, cfg.width_m - 1.0 - w)
            y = rng.uniform(1.0, cfg.height_m - 1.0 - h)
            if (np.hypot(x + w / 2 - start[0], y + h / 2 - start[1]) < 3.2 or
                    np.hypot(x + w / 2 - goal[0], y + h / 2 - goal[1]) < 3.2):
                continue
            rects.append(StaticRect(x, y, w, h))
        dyn = []
        for _ in range(n_dynamic):
            p0 = np.array([rng.uniform(4, cfg.width_m - 4), rng.uniform(3, cfg.height_m - 3)])
            ang = rng.uniform(0, 2 * np.pi)
            p1 = p0 + rng.uniform(3, 6) * np.array([np.cos(ang), np.sin(ang)])
            p1 = np.clip(p1, [2, 2], [cfg.width_m - 2, cfg.height_m - 2])
            dyn.append(DynamicObstacle([tuple(p0), tuple(p1)], rng.uniform(0.4, 0.8), 0.3,
                                       phase_s=rng.uniform(0, 8)))
        world = World(cfg, f"random_s{seed}", rects, dyn, start, goal, 0.0)
        if is_solvable(world):
            return world
    raise RuntimeError("could not generate a solvable random world")


def is_solvable(world: World) -> bool:
    """Ground-truth reachability check used to build fair test worlds."""
    cfg = world.cfg
    occ = world.occupancy_grid().astype(np.uint8)
    r = cfg.inflation_m / cfg.cell_m
    from .geometry import disk_kernel
    blocked = cv2.dilate(occ, disk_kernel(r)).astype(bool)
    s, g = world_to_cell(cfg, *world.start), world_to_cell(cfg, *world.goal)
    return astar(blocked, s, g).success


SCENARIOS: Dict[str, Callable[..., World]] = {
    "warehouse": warehouse,
    "street": street,
    "random": random_field,
}


def build(name: str, cfg: Config, seed: int = 0) -> World:
    if name not in SCENARIOS:
        raise ValueError(f"unknown scenario '{name}'. choose from {list(SCENARIOS)}")
    return SCENARIOS[name](cfg, seed)


def optimal_path_length(world: World) -> float:
    """Length (m) of the best path a robot with a PERFECT map would take.

    Used as the baseline for "path efficiency": a robot that has to discover
    the map by driving around can never beat this number.
    """
    from .geometry import cell_to_world, disk_kernel
    from .planner import smooth_path
    cfg = world.cfg
    occ = world.occupancy_grid().astype(np.uint8)
    blocked = cv2.dilate(occ, disk_kernel(cfg.inflation_m / cfg.cell_m)).astype(bool)
    s, g = world_to_cell(cfg, *world.start), world_to_cell(cfg, *world.goal)
    res = astar(blocked, s, g)
    if not res.success:
        return float("nan")
    pts = [np.array(world.start)] + [np.array(cell_to_world(cfg, r, c))
                                     for r, c in smooth_path(blocked, res.path)[1:-1]] + [np.array(world.goal)]
    return float(sum(np.hypot(*(b - a)) for a, b in zip(pts[:-1], pts[1:])))
