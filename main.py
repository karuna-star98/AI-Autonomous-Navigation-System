#!/usr/bin/env python3
"""AI-Based Autonomous Navigation System - command line entry point.

Examples
--------
  python main.py                                   # warehouse scenario, headless, saves results
  python main.py --scenario street --show          # watch it live in a window
  python main.py --scenario random --seed 7 --video --gif
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from autonav.scenarios import SCENARIOS          # noqa: E402
from autonav.simulation import Simulation        # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System (virtual simulation)")
    p.add_argument("--scenario", "-s", default="warehouse", choices=list(SCENARIOS),
                   help="virtual environment to run (default: warehouse)")
    p.add_argument("--seed", type=int, default=0, help="random seed (changes sensor noise / movers)")
    p.add_argument("--show", action="store_true", help="open a live window (press q to quit)")
    p.add_argument("--video", action="store_true", help="save navigation_demo.mp4")
    p.add_argument("--gif", action="store_true", help="save navigation_demo.gif")
    p.add_argument("--out", default=None, help="output folder (default: outputs/<scenario>_seed<N>)")
    p.add_argument("--quiet", action="store_true", help="less terminal output")
    return p.parse_args()


def main():
    a = parse_args()
    out = a.out or f"outputs/{a.scenario}_seed{a.seed}"
    print("=" * 66)
    print(" AI-Based Autonomous Navigation System")
    print(f" scenario={a.scenario}  seed={a.seed}  output -> {out}/")
    print("=" * 66)
    sim = Simulation(a.scenario, a.seed, out_dir=out, show=a.show, save_video=a.video,
                     save_gif=a.gif, render=True, verbose=not a.quiet)
    m = sim.run()
    print("-" * 66)
    print(f" RESULT : {'GOAL REACHED' if m.success else 'FAILED (' + m.failure_reason + ')'}")
    print(f" time {m.sim_time_s}s | path {m.path_length_m} m (perfect-map optimum {m.optimal_path_m} m, "
          f"efficiency {m.path_efficiency})")
    print(f" re-plans {m.replans} | emergency stops {m.emergency_stops} | collisions {m.collisions} "
          f"| min clearance {m.min_clearance_m} m")
    print(f" planner: avg {m.avg_plan_ms} ms, max {m.max_plan_ms} ms, avg {m.avg_nodes_expanded} nodes")
    print(f" compute time {m.wall_time_s}s (faster than real-time = deployable)")
    print(f" files saved in {out}/")
    print("-" * 66)
    return 0 if m.success else 1


if __name__ == "__main__":
    sys.exit(main())
