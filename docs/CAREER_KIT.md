# Resume, LinkedIn and Interview Kit

> Use only what you can explain. Run the project, read each module, and change something before you present it.
> Numbers below are from `docs/results/evaluation.csv` (30 runs). If you change the code, re-run `scripts/evaluate.py` and update them.

## 1. Resume bullets (pick 3)

* Built an **autonomous navigation system in Python** (OpenCV, NumPy) simulated end-to-end: noisy camera + lidar perception, log-odds occupancy mapping, A\* planning with re-planning, and a layered safety controller; reached the goal in **30/30** simulated runs across 3 environments.
* Implemented **moving-obstacle tracking and 3-second trajectory prediction** with yield/dynamic-window evasion, cutting collisions in a pedestrian scenario from 9 (before) to 3 events in 30 runs, and achieving **0 collisions** in warehouse and cluttered scenarios.
* Designed a **modular, tested codebase** (21 unit/integration tests, batch evaluation across seeds, live dashboard, auto-generated metrics and plots); A\* planning averaged **8.6 ms** per plan and explored **~56 % fewer nodes** than Dijkstra at identical path cost.

*(Short versions)* "Autonomous navigation simulation: OpenCV perception, A\* planning, obstacle avoidance - Python" · "Benchmarked A\*, Dijkstra, BFS; 56 % fewer node expansions with optimal paths."

## 2. LinkedIn project descriptions

**Version A - short post**
> 🚗 I built an AI-based autonomous navigation system - entirely in simulation.
> A robot starts with *no map*, sees through a noisy camera + lidar, builds an occupancy map, plans with A\*, predicts where pedestrians will walk, and stops itself when something gets too close.
> Result: 30/30 runs reached the goal; 0 collisions in warehouse and cluttered worlds. I also documented where it still fails (pedestrians who turn around suddenly) - and what I'd do about it.
> Python · OpenCV · NumPy · Matplotlib | Code + demo: <github link> #Robotics #AutonomousVehicles #ComputerVision #Python #PathPlanning

**Version B - what I learned**
> Biggest lessons from building a self-navigating robot in simulation: (1) the planner is not the hard part - *safety and perception* are; (2) one successful run proves nothing - I evaluated over 30 seeded runs; (3) replaying failures taught me more than any tutorial (a "safety" rule that froze the robot next to a wall was my favourite bug). <github link>

**Version C - technical**
> Project: sense → perceive → plan → act pipeline for a differential-drive robot. Perception: HSV segmentation + contours → detections → log-odds occupancy grid. Planning: 8-connected A\* with clearance cost map and line-of-sight smoothing. Local layer: velocity estimation, 3 s constant-velocity prediction with growing uncertainty, yield/dynamic-window evasion, and an independent lidar emergency stop. 21 tests, batch evaluation, metrics export. <github link>

## 3. Explaining the project

**To HR (30 seconds):** "I built a virtual self-driving robot. It has to travel from A to B in an environment it has never seen, avoiding walls and moving people. It looks through a simulated camera, builds a map of what it has seen, picks the best route, and slows or stops when something gets close. I tested it on 30 runs and wrote down both what works and what doesn't."

**To a technical interviewer (60 seconds):** "It's a modular sense-perceive-plan-act stack in Python. The robot has no prior map: a noisy bird's-eye camera is processed with HSV segmentation and contours into classified detections, which update a log-odds occupancy grid. Planning is 8-connected A\* on the inflated belief map with a clearance cost field and string-pulling smoothing; it re-plans when the path is blocked. Moving obstacles are tracked, their velocity estimated by a short least-squares fit, and predicted for 3 s with growing uncertainty; the local layer yields or runs a small dynamic-window search. An independent lidar layer can always slow or stop. I evaluate over seeded runs: 30/30 reach the goal, 0 collisions in two scenarios, and residual collisions in the pedestrian scenario where people reverse direction."

**Industry relevance:** "Every mobile robot - warehouse AMRs, delivery robots, autonomous cars - runs a version of this loop. The production versions use learned perception, SLAM, and more sophisticated planners, but the architecture and the failure modes (partial observability, moving agents, layered safety) are the same."

## 4. Interview questions and answers

**1. What is the objective?**
To move a robot from a start to a goal in an unknown environment containing static and moving obstacles, using only onboard sensing, without collisions.

**2. Why simulation instead of hardware?**
It lets me test hundreds of scenarios quickly and safely, repeat them with seeds, and measure results. The robot only sees the world through simulated sensors, so the software structure matches a real robot; the hardware layer would just replace the sensors and motors (I included a stub and noted untested wiring fixes).

**3. Does the robot know the map?**
No. It starts with an empty belief map, assumes unseen space is free (free-space assumption), and updates a log-odds occupancy grid from camera observations. That's why re-planning is essential: the first path is usually wrong.

**4. How does it detect obstacles?**
The simulated camera image is blurred, converted to HSV, thresholded by colour for static (orange) and moving (magenta) obstacles, cleaned with morphological opening/closing, and contours give bounding boxes. Pixel positions are converted to world coordinates. *It is classical CV tuned to the simulator; the detector sits behind an interface so YOLO can replace it.*

**5. Why A\* and not Dijkstra or BFS?**
All work on grids. Dijkstra is optimal but explores in every direction; BFS ignores step costs so diagonals aren't optimal. A\* with an admissible heuristic (octile distance) is optimal and focused toward the goal. I benchmarked it: same cost as Dijkstra, ~56 % fewer node expansions.

**6. What makes the heuristic admissible?**
Octile distance never overestimates the true cost on an 8-connected grid with step costs 1 and √2 and cost multipliers ≥ 1. A unit test confirms A\* and Dijkstra give identical costs on random grids.

**7. How do you handle moving obstacles?**
A\* only knows where things are *now*. I track moving detections, estimate velocity with a short least-squares fit, predict positions for 3 s with a radius that grows with time, and check the robot's intended path against them. If a conflict is predicted the robot slows/yields; if even standing still is unsafe, it scores a few arcs (dynamic-window style) and moves away.

**8. What failed, and what did you do about it?** *(best question to prepare)*
Three real ones: (a) a safety rule that stopped the robot whenever anything was within 12 cm *in any direction* froze it beside a wall - I changed it to stop only in the travel direction and always allow rotation. (b) The evasive manoeuvre reversed away from a pedestrian, but the front-obstacle emergency stop overrode it - I exempted reversing-away. (c) Collisions in a pedestrian scenario - I added tracking/prediction and measured: 9 collisions → 3 over 30 runs. I also found that my own scenario had a cone on a pedestrian's lane and fixed the scenario. Remaining failures are pedestrians reversing direction abruptly.

**9. How would you improve it?**
Replace the colour detector with YOLO; add localisation noise and an EKF/SLAM; wrap it as a ROS 2 node (`/scan` in, `/cmd_vel` out) and test in Gazebo; use a learned or probabilistic pedestrian predictor; try an RL local planner and compare against the classical one.

**10. Is this really "AI"?**
It's classical AI and robotics: heuristic search, probabilistic mapping, state estimation and computer vision - not deep learning. That's actually how most production navigation stacks are built; learning is typically used in perception and prediction. My roadmap adds a trained detector.

**11. How do you know it works?**
Unit tests (planner optimality, perception accuracy, collision geometry), end-to-end tests, and a batch evaluation over seeds with metrics: success, collisions, minimum clearance, path efficiency vs the perfect-map optimum, and planning time.

**12. Why a belief map instead of ground truth?**
Using ground truth would be cheating: no real robot has it. The belief map is noisy and incomplete, which is what makes planning and safety non-trivial.

**13. What is path efficiency?**
Optimal length with a full map divided by the distance the robot actually drove. 0.8-0.94 in my scenarios; it can't be 1.0 because the robot spends distance discovering obstacles.

**14. How would this run on a real robot?**
Swap the simulated camera/lidar for real drivers, supply a pose from odometry/SLAM, map `(v, w)` to wheel PWM (see `hardware/`). The brief's original wiring had a GPIO 23 conflict and a 5 V echo line into a 3.3 V pin; I documented fixes.

## 5. Honest answers to hard follow-ups

* *"Did you write all of this yourself?"* - Be truthful. If you used an AI assistant/template, say you used it as a tool, then show you understand and have modified the code (new scenario, tuned parameters, a detector, a bug fix).
* *"What's the weakest part?"* - Prediction of abrupt pedestrian turns, and colour-based perception.
