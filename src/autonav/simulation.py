"""Simulation loop: ties world, sensors, navigator, robot and visualiser together."""
from __future__ import annotations

import json
import math
import shutil
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np

from .config import Config
from .navigator import Navigator, State
from .robot import Robot
from .scenarios import build, optimal_path_length
from .sensors import Camera, Lidar
from . import visualizer


@dataclass
class RunMetrics:
    scenario: str
    seed: int
    success: bool
    failure_reason: str
    sim_time_s: float
    path_length_m: float
    optimal_path_m: float       # best path with a perfect map (baseline)
    path_efficiency: float      # optimal / actual path length (1.0 = as good as a perfect-map robot)
    mean_speed_mps: float
    replans: int
    emergency_stops: int
    yields: int
    collisions: int
    min_clearance_m: float      # smallest robot-edge-to-obstacle distance during the run
    avg_plan_ms: float
    max_plan_ms: float
    avg_nodes_expanded: float
    detections_total: int
    wall_time_s: float


class Simulation:
    def __init__(self, scenario: str = "warehouse", seed: int = 0, cfg: Optional[Config] = None,
                 render: bool = False, show: bool = False, out_dir: Optional[str] = None,
                 save_video: bool = False, save_gif: bool = False, render_every: int = 2,
                 verbose: bool = False):
        self.cfg = cfg or Config()
        self.scenario, self.seed = scenario, seed
        self.rng = np.random.default_rng(seed)
        self.world = build(scenario, self.cfg, seed)
        self.robot = Robot(self.cfg, *self.world.start, self.world.start_theta, self.rng)
        self.camera = Camera(self.cfg, self.rng)
        self.lidar = Lidar(self.cfg, self.rng)
        self.nav = Navigator(self.cfg, self.world.goal)
        self.render = render or show or save_video or save_gif
        self.show, self.save_video, self.save_gif = show, save_video, save_gif
        self.render_every = render_every
        self.verbose = verbose
        self.out_dir = Path(out_dir) if out_dir else None
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)
        # logs
        self.trail: List[tuple] = []
        self.log: List[dict] = []
        self.snapshots: Dict[str, np.ndarray] = {}
        self._frames_gif: List[np.ndarray] = []
        self._writer = None
        self._video_tmp: Optional[Path] = None

    # ------------------------------------------------------------------ run
    def run(self) -> RunMetrics:
        cfg, world, robot, nav = self.cfg, self.world, self.robot, self.nav
        wall0 = time.perf_counter()
        n_steps = int(cfg.max_time_s / cfg.dt)
        cam_every = max(1, int(round(1.0 / (cfg.cam_hz * cfg.dt))))
        collisions, in_collision = 0, False
        min_clear = float("inf")
        ranges = self.lidar.scan(world, robot.pose)
        hist_t, hist_v = [], []
        t = 0.0
        t_final = 0.0
        for step in range(n_steps):
            t = step * cfg.dt
            # --- sense ---
            if step % cam_every == 0:
                nav.perceive(self.camera.capture(world, robot.pose), t)
            ranges = self.lidar.scan(world, robot.pose)
            # --- think + act ---
            v_cmd, w_cmd = nav.act(t, robot.pose, ranges, robot.v)
            robot.step(v_cmd, w_cmd)
            world.step(cfg.dt)
            # --- bookkeeping ---
            hit = world.collides(robot.x, robot.y, cfg.robot_radius)
            if hit and not in_collision:
                collisions += 1
            in_collision = hit
            clear = world.clearance(robot.x, robot.y) - cfg.robot_radius
            min_clear = min(min_clear, clear)
            if step % 2 == 0:
                self.trail.append((robot.x, robot.y))
            hist_t.append(t); hist_v.append(robot.v)
            self.log.append(dict(t=t, x=robot.x, y=robot.y, theta=robot.theta, v=robot.v,
                                 w=robot.w, clearance=clear, state=nav.state.name))
            if self.render and step % self.render_every == 0:
                stats = dict(v=robot.v, w=robot.w, odometer=robot.odometer,
                             goal_dist=math.hypot(world.goal[0] - robot.x, world.goal[1] - robot.y),
                             min_clear=min_clear, collisions=collisions)
                frame = visualizer.compose(cfg, world, robot.pose, nav, ranges, self.trail, t,
                                           hist_t[-400:], hist_v[-400:], stats, self.scenario)
                self._on_frame(frame, step, t)
                if self.show and not self._show(frame):
                    break
            if self.verbose and step % int(5 / cfg.dt) == 0:
                print(f"  t={t:6.1f}s  pos=({robot.x:5.1f},{robot.y:5.1f})  v={robot.v:.2f}  "
                      f"state={nav.state.value:<18} replans={nav.replans} estops={nav.estops}")
            t_final = t
            if nav.state in (State.GOAL, State.FAILED):
                break
        # a few extra frames so the video ends on the goal
        if self.render:
            self._finish_render(ranges, hist_t, hist_v, collisions, min_clear, t_final)
        success = nav.state == State.GOAL
        reason = "" if success else (nav.failure_reason or "timeout")
        plan_ms = nav.plan_times_ms or [0.0]
        optimal = optimal_path_length(world)
        m = RunMetrics(
            scenario=self.scenario, seed=self.seed, success=success, failure_reason=reason,
            sim_time_s=round(t_final, 2), path_length_m=round(robot.odometer, 2),
            optimal_path_m=round(optimal, 2),
            path_efficiency=round(min(1.0, optimal / max(robot.odometer, 1e-6)), 3),
            mean_speed_mps=round(robot.odometer / max(t_final, 1e-6), 3),
            replans=nav.replans, emergency_stops=nav.estops, yields=nav.yields, collisions=collisions,
            min_clearance_m=round(min_clear, 3), avg_plan_ms=round(float(np.mean(plan_ms)), 2),
            max_plan_ms=round(float(np.max(plan_ms)), 2),
            avg_nodes_expanded=round(float(np.mean(nav.plan_expanded or [0])), 1),
            detections_total=nav.detections_total,
            wall_time_s=round(time.perf_counter() - wall0, 2))
        if self.out_dir:
            self._save_outputs(m)
        return m

    # ------------------------------------------------------- frame handling
    def _on_frame(self, frame: np.ndarray, step: int, t: float) -> None:
        nav = self.nav
        snaps = self.snapshots
        if "01_environment" not in snaps:
            snaps["01_environment"] = frame.copy()
        det = nav.last_det
        if det and "02_obstacle_detection" not in snaps:
            kinds = {d.cls for d in det.detections}
            if {"static", "dynamic"} <= kinds and t > 1.0:
                snaps["02_obstacle_detection"] = frame.copy()
        if nav.replans >= 1 and "03_path_planning" not in snaps and t > 0.2:
            snaps["03_path_planning"] = frame.copy()
        if nav.replans >= 3 and "04_replanning" not in snaps:
            snaps["04_replanning"] = frame.copy()
        if nav.estops >= 1 and "05_emergency_stop" not in snaps:
            snaps["05_emergency_stop"] = frame.copy()
        self._write_video(frame)
        if self.save_gif and (step // self.render_every) % 2 == 0:
            small = cv2.resize(frame, None, fx=0.55, fy=0.55, interpolation=cv2.INTER_AREA)
            self._frames_gif.append(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))

    def _show(self, frame: np.ndarray) -> bool:
        try:
            cv2.imshow("AI Autonomous Navigation - press q to quit", frame)
            return (cv2.waitKey(1) & 0xFF) != ord("q")
        except cv2.error:
            print("[warn] cannot open a window here (headless?). Continuing without --show.")
            self.show = False
            return True

    def _write_video(self, frame: np.ndarray) -> None:
        if not (self.save_video and self.out_dir):
            return
        if self._writer is None:
            self._video_tmp = self.out_dir / "_raw.mp4"
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(str(self._video_tmp), cv2.VideoWriter_fourcc(*"mp4v"),
                                           20, (w, h))
        self._writer.write(frame)

    def _finish_render(self, ranges, hist_t, hist_v, collisions, min_clear, t) -> None:
        robot, nav, world, cfg = self.robot, self.nav, self.world, self.cfg
        stats = dict(v=robot.v, w=robot.w, odometer=robot.odometer,
                     goal_dist=math.hypot(world.goal[0] - robot.x, world.goal[1] - robot.y),
                     min_clear=min_clear, collisions=collisions)
        final = visualizer.compose(cfg, world, robot.pose, nav, ranges, self.trail, t,
                                   hist_t[-400:], hist_v[-400:], stats, self.scenario)
        self.snapshots["06_final_run"] = final
        for _ in range(30):                       # hold last frame ~1.5 s
            self._write_video(final)
        if self.save_gif:
            small = cv2.resize(final, None, fx=0.55, fy=0.55, interpolation=cv2.INTER_AREA)
            self._frames_gif += [cv2.cvtColor(small, cv2.COLOR_BGR2RGB)] * 8
        if self.show:
            cv2.waitKey(1500)
            cv2.destroyAllWindows()

    # -------------------------------------------------------------- outputs
    def _save_outputs(self, m: RunMetrics) -> None:
        from .reporting import save_plots
        out = self.out_dir
        (out / "metrics.json").write_text(json.dumps(asdict(m), indent=2))
        import csv
        with open(out / "trajectory.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(self.log[0].keys()))
            wr.writeheader()
            wr.writerows(self.log[::2])
        save_plots(self, out)
        for name, img in self.snapshots.items():
            cv2.imwrite(str(out / f"{name}.png"), img)
        if self._writer is not None:
            self._writer.release()
            final = out / "navigation_demo.mp4"
            if shutil.which("ffmpeg"):
                r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(self._video_tmp),
                                    "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-crf", "24",
                                    str(final)])
                if r.returncode == 0:
                    self._video_tmp.unlink()
                else:
                    self._video_tmp.rename(final)
            else:
                self._video_tmp.rename(final)
        if self.save_gif and self._frames_gif:
            from PIL import Image
            frames = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=96)
                      for f in self._frames_gif]
            frames[0].save(out / "navigation_demo.gif", save_all=True, append_images=frames[1:],
                           duration=100, loop=0, optimize=True)
