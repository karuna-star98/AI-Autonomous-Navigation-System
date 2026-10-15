"""Central configuration. Every tunable number lives here so experiments are easy."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Config:
    # ---- world / simulation -------------------------------------------------
    width_m: float = 30.0          # map width in metres
    height_m: float = 20.0         # map height in metres
    px_per_m: int = 20             # rendering resolution (pixels per metre)
    cell_m: float = 0.25           # planning-grid resolution (metres per cell)
    dt: float = 0.05               # simulation step (s)
    max_time_s: float = 180.0      # give up after this much simulated time

    # ---- robot (differential drive / unicycle model) ------------------------
    robot_radius: float = 0.35
    max_v: float = 1.3             # m/s
    max_w: float = 2.2             # rad/s
    max_acc: float = 1.6           # m/s^2
    max_ang_acc: float = 7.0       # rad/s^2
    actuator_noise: float = 0.01   # multiplicative noise on executed v and w

    # ---- camera (simulated, egocentric bird's-eye view) ---------------------
    cam_range_m: float = 7.0
    cam_fov_deg: float = 120.0
    cam_hz: float = 10.0
    cam_noise_std: float = 6.0     # gaussian pixel noise (0-255 scale)

    # ---- lidar / ultrasonic-style range sensor ------------------------------
    lidar_beams: int = 72
    lidar_range_m: float = 5.0
    lidar_noise_std: float = 0.02  # metres

    # ---- belief map (log-odds occupancy grid) -------------------------------
    l_occ: float = 1.2
    l_free: float = -0.8
    l_clip: float = 4.0
    dyn_memory_s: float = 1.2      # how long a "dynamic" cell keeps extra clearance
    dyn_extra_clearance: float = 0.35

    # ---- planning -----------------------------------------------------------
    inflation_margin: float = 0.35  # safety margin added to robot radius
    proximity_weight: float = 2.5   # extra cost for hugging obstacles
    proximity_range_m: float = 1.2
    replan_period_s: float = 1.0    # periodic replan while dynamic obstacles are near
    horizon_m: float = 6.0          # how far ahead we check the path for blockage
    no_path_timeout_s: float = 25.0

    # ---- control & safety ---------------------------------------------------
    lookahead_m: float = 1.0
    goal_tol_m: float = 0.4
    stop_gap_m: float = 0.30        # emergency-stop if front gap (edge-to-obstacle) below this
    slow_gap_m: float = 1.50        # start slowing below this
    front_sector_deg: float = 35.0

    # ---- derived ------------------------------------------------------------
    @property
    def W(self) -> int:
        return int(round(self.width_m * self.px_per_m))

    @property
    def H(self) -> int:
        return int(round(self.height_m * self.px_per_m))

    @property
    def grid_w(self) -> int:
        return int(round(self.width_m / self.cell_m))

    @property
    def grid_h(self) -> int:
        return int(round(self.height_m / self.cell_m))

    @property
    def inflation_m(self) -> float:
        return self.robot_radius + self.inflation_margin
