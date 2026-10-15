# GitHub Guide: publish this project and build real proof of work

> **Be honest and it will hold up in interviews.** This repository is a finished, tested project. Your job is to
> *understand it, run it, change it, and make it yours*. Recruiters and interviewers will ask about the code.
> Do **not** back-date commits or claim features you can't explain. The 7-day plan below is a *study-and-commit*
> plan: each day you read one part, run it, tweak something, and commit **your** change with **your** notes.

---------------------------------------------------------------------

## 1. One-time setup

```bash
# Install Git: https://git-scm.com/downloads, then tell Git who you are
git config --global user.name  "Your Name"
git config --global user.email "your-github-email@example.com"
```

## 2. Prepare the folder

```bash
cd AI-Autonomous-Navigation-System
python -m venv .venv                 # (Windows: py -3.11 -m venv .venv)
source .venv/bin/activate            # (Windows: .venv\Scripts\activate)
pip install -r requirements.txt
python -m unittest discover -s tests # everything should say OK
python main.py --scenario warehouse --video --gif   # creates outputs/warehouse_seed0/
```

Edit `LICENSE` (your name) and the **Author** section of `README.md` before the first commit.

## 3. Create the repository on GitHub (web)

1. Go to <https://github.com/new>
2. **Repository name:** `ai-autonomous-navigation-system`
3. **Description (copy-paste):**
   `Simulation-based autonomous robot navigation in Python: OpenCV perception, occupancy mapping, A* planning, moving-obstacle prediction and lidar safety layer. No hardware needed.`
4. Public. **Do NOT** tick "Add a README", ".gitignore" or "license" (we already have them - ticking causes push conflicts).
5. Click **Create repository**. Copy the HTTPS URL it shows.

**Topics** (Settings → gear next to *About*): `autonomous-navigation` `robotics` `path-planning` `a-star`
`computer-vision` `opencv` `obstacle-avoidance` `python` `simulation` `self-driving-car` `occupancy-grid` `mobile-robot`

## 4. Connect and push

```bash
git init
git add .
git commit -m "feat: initial commit - autonomous navigation simulation"
git branch -M main
git remote add origin https://github.com/<your-username>/ai-autonomous-navigation-system.git
git push -u origin main
```

Login problems? Run `gh auth login` (GitHub CLI) or use a Personal Access Token as the password.
See `docs/TROUBLESHOOTING.md` → Git section.

## 5. Add your proof assets (screenshots, GIF)

After `python main.py --scenario warehouse --video --gif` copy the best results into the repo:

```bash
cp outputs/warehouse_seed0/navigation_demo.gif   images/demo_warehouse.gif
cp outputs/warehouse_seed0/01_environment.png     images/01_simulation_environment.png
cp outputs/warehouse_seed0/02_obstacle_detection.png images/02_obstacle_detection.png
cp outputs/warehouse_seed0/03_path_planning.png   images/03_path_planning.png
cp outputs/warehouse_seed0/04_replanning.png      images/04_replanning.png
cp outputs/warehouse_seed0/06_final_run.png       images/06_final_run.png
cp outputs/warehouse_seed0/trajectory.png         images/trajectory.png
cp outputs/warehouse_seed0/telemetry.png          images/telemetry.png
git add images && git commit -m "docs: add demo GIF and result screenshots" && git push
```
(Windows: use `copy` instead of `cp`.) Large videos: keep the `.mp4` under ~25 MB, or upload to YouTube and link it in the README.

## 6. Pin it

Your profile → **Customize your pins** → tick this repo. Put it first.

---------------------------------------------------------------------

## 7. Seven-day study-and-commit plan

Each day: **read** the file, **run** it, **change one thing**, **write 2-3 lines of notes** (these become LinkedIn/resume material).

| Day | Focus | Commit (add these files) | Commit message | Proof to save | Notes to keep |
|---|---|---|---|---|---|
| 1 | Setup + structure | `README.md requirements.txt .gitignore LICENSE src/autonav/{__init__,config,geometry}.py` | `chore: project structure, config and coordinate helpers` | terminal screenshot of `pip install` + tests `OK` → `images/00_terminal_setup.png` | Why 3 coordinate frames (world/pixel/cell)? |
| 2 | Simulation world | `world.py robot.py scenarios.py tests/test_world.py` | `feat(sim): ground-truth world, moving obstacles, unicycle robot` | `images/01_simulation_environment.png` | How a ground-truth world differs from what the robot knows |
| 3 | Perception | `sensors.py perception.py tests/test_perception.py` | `feat(perception): OpenCV obstacle detection and occupancy belief map` | `images/02_obstacle_detection.png` | HSV thresholding, why noise needs morphology |
| 4 | Path planning | `planner.py tests/test_planner.py scripts/benchmark_planners.py` | `feat(planning): A*, Dijkstra, BFS with cost map and smoothing` | `outputs/benchmarks/planner_comparison.png`, `images/03_path_planning.png` | Why A* expands fewer nodes than Dijkstra (run the benchmark) |
| 5 | Obstacle avoidance | `avoidance.py controller.py` | `feat(safety): moving-obstacle tracking, prediction, yield/evade` | `images/04_replanning.png` | Run the street scenario with and without `avoidance` - note the collision difference |
| 6 | Integration | `navigator.py simulation.py visualizer.py reporting.py main.py tests/test_integration.py` | `feat: navigator state machine, dashboard and end-to-end tests` | `images/06_final_run.png`, `images/demo_warehouse.gif` | The 3 bugs you hit and how you fixed them |
| 7 | Evaluation + docs | `scripts/evaluate.py docs/ README.md` | `docs: evaluation results, architecture and README` | `images/trajectory.png`, `images/telemetry.png`, GitHub repo + README preview screenshots | Final metrics table |

Concrete commands for Day 2 as an example (repeat the pattern):
```bash
git add src/autonav/world.py src/autonav/robot.py src/autonav/scenarios.py tests/test_world.py
git commit -m "feat(sim): ground-truth world, moving obstacles, unicycle robot"
git push
```
> If you pushed everything in one go on Day 1, that's fine too. What matters is that **later commits are real changes of your own**
> (a new scenario, tuned parameters, a YOLO detector, a bug fix).

## 8. Good commit messages

Use `type(scope): what and why` - `feat`, `fix`, `docs`, `test`, `refactor`, `chore`.
Good: `fix(safety): only stop for obstacles in the direction of travel` · Bad: `update`, `final`, `asdf`.

## 9. Screenshot / proof checklist

| # | What | Ideal file name | Folder | Command / how |
|---|---|---|---|---|
| 1 | Terminal: install + tests OK | `00_terminal_setup.png` | `images/` | screenshot of `python -m unittest discover -s tests` |
| 2 | Terminal: final run summary | `00_terminal_run.png` | `images/` | screenshot after `python main.py` |
| 3 | Simulation environment | `01_simulation_environment.png` | `images/` | auto-saved |
| 4 | Obstacle detection (boxes on camera) | `02_obstacle_detection.png` | `images/` | auto-saved |
| 5 | Path planning | `03_path_planning.png` | `images/` | auto-saved |
| 6 | Re-planning / avoidance | `04_replanning.png` | `images/` | auto-saved |
| 7 | Emergency stop (if triggered) | `05_emergency_stop.png` | `images/` | auto-saved when it happens |
| 8 | Final successful run | `06_final_run.png` | `images/` | auto-saved |
| 9 | Trajectory + telemetry plots | `trajectory.png`, `telemetry.png` | `images/` | auto-saved |
| 10 | Demo GIF | `demo_warehouse.gif` | `images/` | `--gif` |
| 11 | Demo video | `navigation_demo.mp4` | `videos/` | `--video` |
| 12 | Evaluation + planner charts | `evaluation_summary.png`, `planner_comparison.png` | `images/` | `scripts/` |
| 13 | GitHub repo page | `github_repo.png` | local only | screenshot for LinkedIn |
| 14 | README preview | `readme_preview.png` | local only | screenshot for LinkedIn |

Reference an image in the README (the path is relative to the README):
```markdown
![Obstacle detection](images/02_obstacle_detection.png)
```
