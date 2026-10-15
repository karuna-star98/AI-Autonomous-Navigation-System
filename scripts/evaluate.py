#!/usr/bin/env python3
"""Batch evaluation: run many seeds per scenario and report success / safety stats.

    python scripts/evaluate.py --seeds 5            # quick
    python scripts/evaluate.py --seeds 20 --workers 4
Results: outputs/benchmarks/evaluation.csv + evaluation_summary.png
"""
import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np                                   # noqa: E402
from autonav.simulation import Simulation            # noqa: E402


def run_one(args):
    scenario, seed = args
    return asdict(Simulation(scenario, seed).run())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--scenarios", nargs="+", default=["warehouse", "street", "random"])
    p.add_argument("--workers", type=int, default=2)
    a = p.parse_args()
    jobs = [(s, k) for s in a.scenarios for k in range(a.seeds)]
    print(f"running {len(jobs)} simulations on {a.workers} workers ...")
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_one, jobs))
    out = ROOT / "outputs" / "benchmarks"
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "evaluation.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader(); wr.writerows(rows)

    print(f"\n{'scenario':<10}{'runs':>5}{'success':>9}{'collis.':>9}{'time(s)':>9}{'eff.':>7}"
          f"{'replans':>9}{'estops':>8}{'min clr(m)':>11}")
    summary = {}
    for sc in a.scenarios:
        r = [x for x in rows if x["scenario"] == sc]
        ok = [x for x in r if x["success"]]
        summary[sc] = dict(
            n=len(r), succ=100 * len(ok) / len(r), col=sum(x["collisions"] for x in r),
            t=np.mean([x["sim_time_s"] for x in ok]) if ok else float("nan"),
            eff=np.mean([x["path_efficiency"] for x in ok]) if ok else float("nan"),
            rp=np.mean([x["replans"] for x in r]), es=np.mean([x["emergency_stops"] for x in r]),
            clr=min(x["min_clearance_m"] for x in r))
        s = summary[sc]
        print(f"{sc:<10}{s['n']:>5}{s['succ']:>8.0f}%{s['col']:>9}{s['t']:>9.1f}{s['eff']:>7.2f}"
              f"{s['rp']:>9.1f}{s['es']:>8.1f}{s['clr']:>11.3f}")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.8))
    names = list(summary)
    ax[0].bar(names, [summary[n]["succ"] for n in names], color="tab:green"); ax[0].set_ylim(0, 105)
    ax[0].set_title("Success rate (%)")
    ax[1].bar(names, [summary[n]["eff"] for n in names], color="tab:blue"); ax[1].set_ylim(0, 1.05)
    ax[1].set_title("Path efficiency (optimal / actual)")
    ax[2].bar(names, [summary[n]["col"] for n in names], color="tab:red")
    ax[2].set_title(f"Total collisions ({a.seeds} seeds each)")
    fig.tight_layout(); fig.savefig(out / "evaluation_summary.png", dpi=130)
    print(f"\nsaved {out}/evaluation.csv and evaluation_summary.png")


if __name__ == "__main__":
    main()
