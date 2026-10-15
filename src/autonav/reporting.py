"""Matplotlib figures saved after every run (good README / portfolio material)."""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")                      # headless-safe backend
import matplotlib.pyplot as plt
import numpy as np

STATE_COLORS = {"EMERGENCY_STOP": "#e04040", "WAITING": "#f0b030", "RECOVERING": "#f08030"}


def save_plots(sim, out: Path) -> None:
    cfg, world, nav = sim.cfg, sim.world, sim.nav
    log = sim.log
    t = np.array([r["t"] for r in log])
    x = np.array([r["x"] for r in log])
    y = np.array([r["y"] for r in log])
    v = np.array([r["v"] for r in log])
    clr = np.array([r["clearance"] for r in log])

    # ---- 1. trajectory over the map (ground-truth, final dynamic positions) ----
    fig, ax = plt.subplots(figsize=(10, 6.8))
    img = world.render(include_dynamic=True)[:, :, ::-1]
    ax.imshow(img, extent=[0, cfg.width_m, 0, cfg.height_m])
    sc = ax.scatter(x, y, c=t, s=4, cmap="viridis", zorder=3)
    ax.plot(*world.start, "o", c="tab:blue", ms=9, zorder=4, label="start")
    ax.plot(*world.goal, "*", c="tab:green", ms=16, zorder=4, label="goal")
    for e in nav.events:
        if e.kind == "estop":
            i = min(int(e.t / cfg.dt), len(x) - 1)
            ax.plot(x[i], y[i], "x", c="red", ms=9, mew=2.5, zorder=5)
    ax.plot([], [], "x", c="red", label="emergency stop")
    fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01, label="time (s)")
    ax.set_title(f"Robot trajectory - {sim.scenario} (seed {sim.seed})")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(loc="upper right")
    fig.tight_layout(); fig.savefig(out / "trajectory.png", dpi=130); plt.close(fig)

    # ---- 2. telemetry: speed + clearance over time with event markers ----
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    a1.plot(t, v, c="tab:green"); a1.set_ylabel("speed (m/s)"); a1.grid(alpha=.3)
    a2.plot(t, clr, c="tab:blue"); a2.set_ylabel("clearance to nearest\nobstacle (m)")
    a2.axhline(0, c="red", lw=1, ls="--"); a2.grid(alpha=.3); a2.set_xlabel("time (s)")
    for e in nav.events:
        col = {"replan": "tab:purple", "estop": "tab:red", "recover": "tab:orange"}.get(e.kind, "grey")
        for ax in (a1, a2):
            ax.axvline(e.t, c=col, alpha=.25, lw=1)
    a1.plot([], [], c="tab:purple", alpha=.5, label="re-plan")
    a1.plot([], [], c="tab:red", alpha=.5, label="emergency stop")
    a1.legend(loc="upper right", ncol=2)
    a1.set_title("Navigation telemetry")
    fig.tight_layout(); fig.savefig(out / "telemetry.png", dpi=130); plt.close(fig)
