<div align="center">

# Cognitive Robotics — Group Coursework

### Simulated Jetbot navigation with traffic-sign perception

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-FF6F00?style=for-the-badge)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-navigation-3b82f6?style=for-the-badge)](https://navigation.ros.org/)
[![YOLO](https://img.shields.io/badge/YOLO-ultralytics-111827?style=for-the-badge)](https://github.com/ultralytics/ultralytics)
[![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)

<br/>

[![NTU](https://img.shields.io/badge/Nottingham%20Trent%20University-Cognitive%20Robotics-008080?style=flat-square)](https://www.ntu.ac.uk/)
[![Simulation](https://img.shields.io/badge/simulation-maze%20world-22c55e?style=flat-square)]()
[![Perception](https://img.shields.io/badge/perception-traffic%20signs-f59e0b?style=flat-square)]()

<br/>

[![GitHub last commit](https://img.shields.io/github/last-commit/hackerbotfz/Cognitive-CW?style=flat-square&logo=github)](https://github.com/hackerbotfz/Cognitive-CW/commits)
[![GitHub repo size](https://img.shields.io/github/repo-size/hackerbotfz/Cognitive-CW?style=flat-square&logo=github)](https://github.com/hackerbotfz/Cognitive-CW)
[![GitHub stars](https://img.shields.io/github/stars/hackerbotfz/Cognitive-CW?style=flat-square&logo=github)](https://github.com/hackerbotfz/Cognitive-CW/stargazers)

<br/>

**[Faiz Lawan](https://github.com/hackerbotfz)**

</div>

---

ROS 2 workspace for an NTU cognitive robotics group project: a **Jetbot** robot in an **Ignition Gazebo** maze with **Nav2** autonomous navigation, **AMCL** localisation, and a **YOLO** node for detecting stop / slow / fast traffic-sign posters on the walls.

## Packages

| Package | Role |
|---------|------|
| **ntu_robotsim** | Worlds, models, bridges, Nav2 launch files, YOLO detector node |
| **odom_to_tf_ros2** | Publishes `odom` → `tf` for the simulated robot frame |

## Architecture

```mermaid
flowchart TB
    subgraph Sim["Ignition Gazebo"]
        MAZE[maze.sdf]
        JET[jetbot + RGB-D camera]
        SIGNS[traffic-sign posters]
    end

    subgraph Bridge["ros_gz_bridge"]
        BR[parameter_bridge]
    end

    subgraph ROS2["ROS 2 stack"]
        RSP[robot_state_publisher]
        NAV[Nav2 + AMCL]
        YOLO[yolo_detector]
        ODOM[odom_to_tf]
    end

    MAZE --> JET
    SIGNS --> JET
    JET --> BR
    BR --> RSP
    BR --> NAV
    BR --> YOLO
    BR --> ODOM
```

## Workspace layout

```
Cognitive_CW/
├── src/
│   ├── ntu_robotsim/       # simulation, navigation, perception
│   └── odom_to_tf_ros2/    # odometry TF helper
├── requirements.txt        # Python deps for YOLO node
└── README.md
```

## Build

```bash
cd Cognitive_CW
pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

Trained weights ship as `src/ntu_robotsim/models/best.pt.gz` (Git LFS). After clone: `git lfs pull`. The detector decompresses them on first run. Without LFS or weights, it falls back to `yolov8n.pt`.

To add or refresh weights from a local `best.pt`:

```bash
python scripts/compress_model.py
git add src/ntu_robotsim/models/best.pt.gz
git commit -m "Add trained YOLO weights"
```

## Run

**Maze simulation**

```bash
ros2 launch ntu_robotsim maze.launch.py
```

**Robot + ROS ↔ Gazebo bridge**

```bash
ros2 launch ntu_robotsim single_robot_sim.launch.py
```

**Navigation (Nav2 + RViz)**

```bash
ros2 launch ntu_robotsim navigation.launch.py
```

**Traffic-sign detection**

```bash
ros2 run ntu_robotsim yolo_detector.py
```

**Odometry TF**

```bash
ros2 launch odom_to_tf_ros2 atlas_odom_to_tf.launch.py
```

## License

[LICENSE](LICENSE) (Apache-2.0) · [NOTICE](NOTICE) · `odom_to_tf_ros2`: [BSD 3-Clause](src/odom_to_tf_ros2/LICENSE)
