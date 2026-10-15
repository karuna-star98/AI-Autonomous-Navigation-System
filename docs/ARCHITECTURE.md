# Architecture

## Block diagram

```
 ┌──────────────────────── SIMULATOR (ground truth, hidden from the robot) ────────────────────────┐
 │  World: static obstacles (shelves, buildings, cones, walls)  +  moving obstacles (people, forklift) │
 └───────────────┬───────────────────────────────────────────────────────────┬──────────────────────┘
                 │ renders                                                    │ ray-casts
                 ▼                                                            ▼
        ┌─────────────────┐                                           ┌──────────────┐
        │ Camera (BEV)    │  120° FOV, 7 m, Gaussian noise             │ Lidar 72 rays │ 5 m, noise
        └────────┬────────┘                                           └──────┬───────┘
                 ▼                                                           │
 ┌───────────────────────────── ROBOT SOFTWARE (src/autonav) ────────────────┼───────────────────┐
 │ PERCEPTION                                                                │                   │
 │  ObstacleDetector: blur → HSV threshold → morphology → contours           │                   │
 │       → detections {class, bbox, world xy, distance}                      │                   │
 │       │                                                                   │                   │
 │       ├──► OccupancyBelief (log-odds grid): "what I have seen"            │                   │
 │       └──► DynamicAvoider tracks: velocity estimate + 3 s prediction      │                   │
 │                                                                           │                   │
 │ PLANNING                                                                  │                   │
 │  belief → inflate by robot size → cost map (stay clear of obstacles)      │                   │
 │       → A* (8-connected) → line-of-sight smoothing → waypoints            │                   │
 │       re-plan when: path blocked │ moving obstacles near │ no progress    │                   │
 │                                                                           │                   │
 │ CONTROL & SAFETY (runs every 50 ms)                                       ▼                   │
 │  PathFollower (look-ahead heading controller) ─► (v, w)                                       │
 │     ─► DynamicAvoider.filter : yield / evade if a predicted conflict                          │
 │     ─► Lidar safety layer     : slow down, emergency-stop in the travel direction             │
 └───────────────────────────────────────────┬──────────────────────────────────────────────────┘
                                             ▼  (v, w)
                              Robot (unicycle model, accel limits, actuator noise)
                                             │ pose
                                             └──────────► back to the simulator
```

## Data flow, one tick (50 ms)

1. **Sense** - every 100 ms the camera renders an image of what is ahead; the lidar scans every tick.
2. **Perceive** - the detector finds orange (static) and magenta (moving) blobs and converts pixel
   positions to world metres. The belief grid is updated: cells the camera fully covers get *free* evidence
   unless an obstacle is detected there (*occupied* evidence). Unseen cells stay *unknown*.
3. **Decide** - the navigator builds a planning map from the belief (obstacles inflated by robot radius + margin,
   moving obstacles inflated extra). If the current path is blocked, it re-plans with A*.
4. **Act** - the controller steers toward a look-ahead point; the avoidance layer predicts where moving obstacles
   will be and yields/evades; the lidar safety layer has the last word.
5. **Move** - the robot integrates the velocity command with acceleration limits and noise.

## Module guide

| File | Responsibility | Key idea |
|------|----------------|----------|
| `config.py` | all tunable numbers | one place to experiment |
| `geometry.py` | coordinate frames, Bresenham, disk kernel | world / pixel / cell conventions |
| `world.py` | ground truth, rendering, collision, ray-casting | robot never reads it directly |
| `robot.py` | unicycle kinematics | accel limits + noise |
| `sensors.py` | `Camera`, `Lidar` | partial, noisy observations |
| `perception.py` | `ObstacleDetector`, `OccupancyBelief` | CV → detections → map |
| `planner.py` | BFS / Dijkstra / A*, smoothing | octile heuristic, no corner cutting |
| `avoidance.py` | tracking, prediction, yield/evade | constant-velocity prediction, dynamic-window arcs |
| `controller.py` | look-ahead path follower | speed drops in turns, eases into the goal |
| `navigator.py` | state machine + safety | the "brain" |
| `visualizer.py`, `reporting.py` | dashboard, plots | portfolio-ready outputs |
| `simulation.py` | main loop + metrics + video/GIF | headless or live |

## Key design decisions (good interview material)

* **Partial observability.** The robot starts with an *empty* map (unknown = free, "free-space assumption")
  and discovers obstacles as it drives. Re-planning is therefore essential, not decoration.
* **Belief map instead of ground truth.** Log-odds updates make the map robust to noise and let moving
  obstacles fade out when they leave a cell.
* **Layered safety.** The planner can be wrong; the avoider can be wrong; the lidar layer is simple and independent.
* **Cost-aware A\*.** Entering a cell costs more near obstacles, so paths keep their distance without extra logic.
* **Perception behind an interface.** `ObstacleDetector.detect(frame) → detections`. Swapping HSV for YOLO means
  rewriting only that class.

## Known limitations (stated honestly)

* Robot pose is perfect (no localisation / SLAM). Real robots must estimate it.
* The camera is a simulated bird's-eye view; real cameras need an inverse-perspective transform or depth.
* Detection is colour-based, so it only works in this simulator - a trained model is the real-world upgrade.
* Moving-obstacle prediction assumes constant velocity and non-reactive pedestrians.
* Planning is 2D and the robot is circular.
