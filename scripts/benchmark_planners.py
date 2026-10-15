#!/usr/bin/env python3
"""Compare BFS, Dijkstra and A* on random obstacle grids.

    python scripts/benchmark_planners.py
Produces outputs/benchmarks/planner_comparison.png and prints a table.
Takeaway for interviews: A* and Dijkstra find the SAME optimal cost, but A* expands
far fewer nodes; BFS is fast but ignores diagonal step cost, so it is not optimal.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                  # noqa: E402
import numpy as np                               # noqa: E402
from autonav.planner import astar, bfs, dijkstra  # noqa: E402


def main(n_instances: int = 40, size: int = 120, density: float = 0.25):
    rng = np.random.default_rng(42)
    stats = {k: dict(exp=[], ms=[], cost=[]) for k in ("BFS", "Dijkstra", "A*")}
    used = 0
    while used < n_instances:
        g = rng.random((size, size)) < density
        g[0, 0] = g[-1, -1] = False
        s, t = (0, 0), (size - 1, size - 1)
        results = {"BFS": bfs(g, s, t), "Dijkstra": dijkstra(g, s, t), "A*": astar(g, s, t)}
        if not all(r.success for r in results.values()):
            continue
        used += 1
        for k, r in results.items():
            stats[k]["exp"].append(r.expanded); stats[k]["ms"].append(r.time_ms); stats[k]["cost"].append(r.cost)
    print(f"{size}x{size} grids, {int(density*100)}% obstacles, {n_instances} instances\n")
    print(f"{'planner':<10}{'nodes expanded':>16}{'time (ms)':>12}{'avg path cost':>15}")
    for k, v in stats.items():
        print(f"{k:<10}{np.mean(v['exp']):>16.0f}{np.mean(v['ms']):>12.1f}{np.mean(v['cost']):>15.1f}")
    out = ROOT / "outputs" / "benchmarks"
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))
    names = list(stats)
    cols = ["#9aa5b1", "#5b8def", "#2ca25f"]
    for a, key, title in zip(ax, ("exp", "ms", "cost"),
                             ("Nodes expanded (lower = better)", "Planning time, ms (lower = better)",
                              "Mean path cost (lower = better)")):
        a.bar(names, [np.mean(stats[k][key]) for k in names], color=cols)
        a.set_title(title, fontsize=10)
    fig.suptitle(f"Planner comparison - {size}x{size} grid, {int(density*100)}% obstacles")
    fig.tight_layout(); fig.savefig(out / "planner_comparison.png", dpi=130)
    print(f"\nsaved {out/'planner_comparison.png'}")


if __name__ == "__main__":
    main()
