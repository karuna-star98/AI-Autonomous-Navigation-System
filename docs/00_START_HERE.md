# Start here: where everything is

This maps your original 17-section brief (A-P) to the files in this repository.

| Brief section | Where |
|---|---|
| **A.** Project explanation | `README.md` (overview, problem, relevance), `docs/ARCHITECTURE.md` |
| **B.** Tech-stack options | Below |
| **C.** Selected approach | Below |
| **D.** Architecture | `docs/ARCHITECTURE.md` |
| **E.** Folder structure | `README.md` |
| **F.** Installation | `README.md` → Installation |
| **G.** Full code, file by file | `src/autonav/*.py`, `main.py`, `scripts/` (each file has a docstring explaining its job) |
| **H.** Virtual simulation workflow | `README.md` → Simulation workflow; `docs/ARCHITECTURE.md` |
| **I.** How to run | `README.md` → How to run |
| **J.** GitHub upload steps | `docs/GITHUB_GUIDE.md` §1-6 |
| **K.** README | `README.md` |
| **L.** Commit strategy | `docs/GITHUB_GUIDE.md` §7-8 |
| **M.** Screenshots / proof checklist | `docs/GITHUB_GUIDE.md` §9 |
| **N.** Resume / LinkedIn / interview | `docs/CAREER_KIT.md` |
| **O.** Future improvements | `docs/ROADMAP.md` |
| **P.** Troubleshooting | `docs/TROUBLESHOOTING.md` |

## B. Three versions of the project

| | Option A - Easiest | Option B - Intermediate (**chosen**) | Option C - Advanced |
|---|---|---|---|
| Tools | Python, NumPy, Matplotlib | Python, NumPy, OpenCV, Matplotlib (custom 2D simulator) | CARLA or Gazebo + ROS 2, PyTorch/YOLO |
| What you build | A\* on a known grid, animated | Unknown map, camera + lidar perception, occupancy mapping, A\*, moving-obstacle prediction, safety layer | Full simulator integration with trained detector, SLAM, ROS nodes |
| Difficulty | ★☆☆ | ★★☆ | ★★★ |
| GPU | No | **No** | Yes (CARLA needs a strong GPU) |
| Install effort | 5 min | 10 min | Hours, many version conflicts |
| Proof value | Looks like a tutorial | Strong, and finishes in days | Very strong, but most students never finish |

## C. Why Option B

It runs on any laptop, shows every stage of the real pipeline (perception → mapping → planning → control → safety), can be *measured* (success/collision/clearance across seeds), and leaves a clear upgrade path (YOLO → ROS 2 → CARLA). Option C is where the roadmap leads.

## About the original brief

* The brief's hardware chapter is real-robot oriented; this repository implements the **virtual simulation** path it also asks for. `hardware/` contains an optional Raspberry Pi stub.
* The brief's wiring guide has two errors (GPIO 23 used twice; 5 V HC-SR04 echo into a 3.3 V GPIO). See `hardware/README.md`.
* The brief's YOLO snippet only displays the camera feed (it never runs the network). Here the detector is real but classical; `docs/ROADMAP.md` explains how to add YOLO.
