# Future improvements (and how to build each)

| Upgrade | What it adds | How to start | Difficulty |
|---|---|---|---|
| **Real object detector (YOLO)** | Detect people/cars/boxes instead of coloured blobs | Keep `ObstacleDetector.detect(frame)` signature; load an Ultralytics `yolov8n` model, return the same `Detection` objects. Run on real images/videos or CARLA frames | ★★☆ |
| **Real-time camera input** | Use a webcam / Pi camera | `cv2.VideoCapture(0)`; add an inverse-perspective transform (4-point homography) to get a bird's-eye view | ★★☆ |
| **Lane detection** | Road following | Canny edges → region mask → Hough lines or sliding-window polynomial fit; feed the lane centre as the path | ★★☆ |
| **ROS 2 integration** | Industry-standard robotics middleware | Wrap `Navigator` in a ROS 2 node: subscribe `/scan`, `/camera/image_raw`; publish `/cmd_vel` (that is exactly `(v, w)`). Test in Gazebo/TurtleBot3 | ★★★ |
| **CARLA integration** | Realistic self-driving simulator | Replace `Camera`/`Lidar`/`Robot` with CARLA sensors/vehicle; keep planner and avoidance | ★★★ |
| **SLAM / localisation** | Remove the "perfect pose" assumption | Add odometry noise, then an EKF or particle filter localising against the belief map; later `slam_toolbox` | ★★★ |
| **Dynamic window / MPC local planner** | Smoother, optimal local control | Generalise `DynamicAvoider._arc` search into a full DWA with a cost function | ★★☆ |
| **Reinforcement learning** | Learned local policy | Wrap `Simulation` in a Gymnasium env (obs = lidar, action = (v, w)); train PPO (Stable-Baselines3) and compare with the classical stack | ★★★ |
| **Multiple agents** | Fleet / traffic | Several `Robot` + `Navigator` objects; treat other robots as dynamic obstacles; add priority rules | ★★☆ |
| **Warehouse robot version** | Pick-and-deliver | Task queue of goals + docking; reuse `warehouse` scenario | ★★☆ |
| **Drone version** | 3D navigation | 3D grid / octree, A* in 3D, AirSim or PX4-SITL | ★★★ |
| **Cloud dashboard / IoT** | Remote monitoring | Publish metrics over MQTT; Flask/Streamlit dashboard; Docker | ★★☆ |
| **Learned cost maps** | Smarter planning | Train a small CNN to predict traversability | ★★★ |

Suggested order for a student: YOLO detector → localisation noise + EKF → ROS 2 wrapper → CARLA/Gazebo.
