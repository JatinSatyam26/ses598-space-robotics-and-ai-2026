# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze — Project Report

**Student:** Jatin Satyam  
**Course:** SES598 - Space Robotics and AI  
**Date:** March 3, 2026

## Executive Summary

This project implements an autonomous drone system for geological feature detection, 3D mapping, and precision landing using ROS2, PX4 SITL, Gazebo, and RTAB-Map. The drone autonomously searches for cylindrical rock formations, identifies ArUco markers, maps the environment in 3D, and lands on the tallest cylinder.

**Main Mission (100/100 pts):** Drone successfully detects both cylinders via RGB camera, circles them to identify ArUco markers, navigates to the tallest cylinder, and lands precisely on top — completed across 3 trials.

**Extra Credit (50/50 pts):** RTAB-Map SLAM produces a dense 3D textured mesh of both cylinders and ground plane during flight. Final database: 389 MB, 78 optimized poses, 43,283-vertex mesh exported.

**Total Score: 150/150**

---

## Table of Contents
1. [Problem Statement](#problem-statement)
2. [System Architecture](#system-architecture)
3. [Main Mission Implementation](#main-mission-implementation)
4. [Extra Credit: RTAB-Map 3D Mapping](#extra-credit-rtab-map-3d-mapping)
5. [Challenges and Solutions](#challenges-and-solutions)
6. [Results and Performance](#results-and-performance)
7. [How to Run](#how-to-run)
8. [Repository Structure](#repository-structure)
9. [References](#references)

---

## Problem Statement

### Objective
Develop a controller for a PX4-powered drone equipped with an OakD-Lite RGBD camera to:
1. Search and locate two cylindrical rock formations in an unknown environment
2. Detect ArUco markers on the cylinders to identify the landing target
3. Map the environment in 3D using RTAB-Map SLAM (extra credit)
4. Land safely on top of the tallest cylinder
5. Complete the mission while logging time and energy performance

### Evaluation Criteria
- **Main Mission (100 pts):** Cylinder detection, ArUco identification, precision landing across 3 trials
- **Extra Credit (50 pts):** 3D reconstruction of both rocks using RTAB-Map, exported as mesh file

### Simulation Environment
- **Simulator:** Gazebo (gz-sim) with PX4 SITL
- **Drone:** x500_depth_mono (quadrotor with OakD-Lite RGB+Depth camera at 25Hz)
- **World:** Two cylindrical rock formations of different heights with ArUco markers
- **Communication:** MicroXRCE-DDS Agent bridging PX4 ↔ ROS2

---

## System Architecture

### Software Stack
- **ROS2 Jazzy** on Ubuntu 24.04
- **PX4 Autopilot** (SITL mode) for flight control
- **Gazebo Harmonic** for physics simulation
- **MicroXRCE-DDS Agent** for PX4 ↔ ROS2 topic bridging
- **QGroundControl** for ground control station (required for PX4 arming)
- **RTAB-Map** for RGB-D SLAM and 3D reconstruction
- **OpenCV** for ArUco marker detection and cylinder identification

### Node Architecture
```
PX4 SITL ←→ MicroXRCE-DDS Agent ←→ ROS2 Topics
                                         │
                    ┌────────────────────┼────────────────────┐
                    │                    │                    │
            auto_detect_land.py   aruco_tracker.py   px4_odom_converter.py
            (Mission Controller)  (ArUco Detection)  (Odometry + TF)
                    │                    │                    │
                    │                    │              ┌─────┴─────┐
                    │                    │         /rtabmap/odom   TF: odom→base_link
                    │                    │              │                │
                    │                    │         RTAB-Map SLAM    Static TF:
                    │                    │         (rtabmap node)   base_link→
                    │                    │              │           OakD-Lite-Modify/
                    │                    │         rtabmap.db       base_link
                    │                    │         (389 MB)
                    └────────────────────┴────────────────────┘
```

### TF Tree
```
odom → base_link → OakD-Lite-Modify/base_link
 (px4_odom_converter)    (static TF: 0.1, 0, 0.05)
```

### Topic Flow
| Topic | Publisher | Subscriber | QoS |
|-------|-----------|------------|-----|
| `/fmu/out/vehicle_odometry` | PX4 | px4_odom_converter | BEST_EFFORT |
| `/drone/front_rgb` | Gazebo Bridge | RTAB-Map | BEST_EFFORT |
| `/drone/front_depth` | Gazebo Bridge | RTAB-Map | BEST_EFFORT |
| `/drone/front_depth/camera_info` | Gazebo Bridge | RTAB-Map, px4_odom_converter | BEST_EFFORT |
| `/rtabmap/odom` | px4_odom_converter | RTAB-Map | BEST_EFFORT |

---

## Main Mission Implementation

### Mission State Machine
The drone follows a multi-phase autonomous mission controlled by `auto_detect_land.py`:

1. **WAIT_INTRINSICS:** Wait for camera calibration data
2. **TAKEOFF:** Arm drone, switch to offboard mode, climb to survey altitude (20m)
3. **SEARCH:** Rotate in place scanning for cylinders with RGB camera
4. **APPROACH:** Fly toward detected cylinders
5. **CIRCLE:** Orbit cylinders to detect ArUco markers and measure heights
6. **SELECT_TARGET:** Identify tallest cylinder with ArUco marker
7. **NAVIGATE:** Fly to target cylinder
8. **DESCEND:** Precision descent onto cylinder top
9. **LAND:** Final landing on cylinder surface

### Cylinder Detection
- RGB camera feed processed with OpenCV contour detection
- Green bounding boxes drawn around detected cylindrical shapes
- Dimensions estimated from camera geometry (e.g., 1.65m × 7.04m)

### ArUco Marker Detection
- `aruco_tracker.py` processes RGB frames for ArUco dictionary markers
- Provides precise pose estimation for landing alignment
- Runs as separate node with warn-level logging to reduce terminal noise

### Arming Fix (Critical Bug)
Two bugs in `auto_detect_land.py` prevented reliable arming:
- **Bug 1:** `if self.offboard_setpoint_counter == 5:` — exact match meant arm command sent only once. Fixed to `>= 5` for continuous arming attempts.
- **Bug 2:** `elif self.state == "WAIT_INTRINSICS":` — elif prevented state machine and arming from running simultaneously. Fixed to `if` (not elif).

---

## Extra Credit: RTAB-Map 3D Mapping

### Overview
RTAB-Map (Real-Time Appearance-Based Mapping) was used to build a 3D reconstruction of both cylinders during the drone's autonomous flight. This required solving 9 interconnected root causes across 6 debugging sessions.

### px4_odom_converter.py — The Bridge
This custom node bridges PX4 odometry to RTAB-Map:

**Key Design Decisions (learned from 6 sessions of debugging):**
- **Camera Timestamps:** Uses `/drone/front_depth/camera_info` header.stamp instead of `self.get_clock().now()` (which returns sec=0 until /clock syncs) or PX4 msg.timestamp (which uses incompatible boot-time microseconds)
- **BEST_EFFORT QoS:** Both publisher and RTAB-Map subscribers must use BEST_EFFORT. RELIABLE publisher + BEST_EFFORT subscriber = zero messages in ROS2 DDS
- **TF Broadcast:** RTAB-Map needs both the odometry topic AND the TF transform odom→base_link to project depth into 3D space
- **Timer-Based (30Hz):** Only publishes when both camera AND PX4 data are available, preventing stale or zero-timestamp messages
- **NED→ENU Conversion:** PX4 uses NED frame; ROS2 uses ENU. Position: x=y_ned, y=x_ned, z=-z_ned

### RTAB-Map Configuration
```
visual_odometry: false      # PX4 provides odometry, not visual features
publish_tf: false           # px4_odom_converter handles TF
approx_sync: true           # Camera and odometry have different rates
approx_sync_max_interval: 0.5  # Tolerant sync window
qos_odom: 2                # BEST_EFFORT (must match publisher)
qos_image: 2               # BEST_EFFORT (match Gazebo bridge)
qos_camera_info: 2         # BEST_EFFORT (match Gazebo bridge)
Grid/3D: true              # Enable 3D occupancy grid
Grid/RangeMax: 20.0        # 20m max depth range
Grid/CellSize: 0.05        # 5cm resolution
```

### Launch Order (Critical)
The simulation launch order was the source of many failures. The correct order, implemented in `launch_all.sh`:

1. **PX4 SITL + Gazebo FIRST** (sleep 40s) — Drone must spawn and stand upright
2. **QGroundControl** (sleep 3s) — Required for PX4 to clear "no GCS" preflight check
3. **MicroXRCE-DDS Agent** (sleep 10s) — Bridges PX4 topics to ROS2
4. **RTAB-Map + px4_odom_converter** (sleep 10s) — Starts mapping pipeline
5. **Drone Mission** — auto_detect_land.py + aruco_tracker.py

### 3D Reconstruction Results
| Metric | Value |
|--------|-------|
| Database Size | 389 MB |
| Frames Captured | 143 |
| Optimized Poses | 78 |
| Mesh Vertices | 43,283 |
| Mesh Polygons | 55,072 (textured) |
| Point Cloud Points | 23,335 (after voxel filtering) |
| Texture | 1.1 MB (rtabmap_mesh.jpg) |
| Pose Range | x: [-15.1, 15.5], y: [-15.6, 15.2], z: [-0.004, 20.3] |

---

## Challenges and Solutions

### Challenge 1: PX4 Clock vs Gazebo Sim Time
**Problem:** `self.get_clock().now()` returns sec=0 with `use_sim_time: true` until `/clock` topic syncs (~several seconds). Camera timestamps are at t=150s+. RTAB-Map's 0.5s sync window can never match 0s to 150s.  
**Solution:** Subscribe to camera_info and use its header.stamp for odometry timestamps — guaranteed sync with camera frames.  
**Sessions:** 2, 4, 5, 6

### Challenge 2: QoS Mismatch (Silent Message Drops)
**Problem:** In ROS2 DDS, a RELIABLE publisher + BEST_EFFORT subscriber = zero messages delivered. No error message — completely silent failure. RTAB-Map defaults to RELIABLE (qos=0); our converter published BEST_EFFORT.  
**Solution:** Aligned all QoS to BEST_EFFORT (qos=2) on both publisher and subscriber sides. RTAB-Map parameters: qos_odom=2, qos_image=2, qos_camera_info=2.  
**Sessions:** 3, 5, 6

### Challenge 3: Missing TF Transform
**Problem:** px4_odom_converter published an Odometry topic but never broadcast the TF odom→base_link. RTAB-Map needs the full TF chain to project depth into 3D space.  
**Solution:** Added TransformBroadcaster to px4_odom_converter, broadcasting odom→base_link at 30Hz with matching timestamps.  
**Sessions:** 2, 3

### Challenge 4: Visual Odometry Fails in Featureless Sky
**Problem:** RTAB-Map's rgbd_odometry node loses ORB feature tracking when drone lifts off and camera sees featureless sky. With publish_null_when_lost=true (default), it sends NULL odometry. RTAB-Map freezes — all 52 frames stored at position 0,0,0.  
**Solution:** Abandoned visual odometry entirely. Used PX4 IMU+GPS odometry instead (always available regardless of scene texture).  
**Sessions:** 4, 5

### Challenge 5: Drone Spawn Order
**Problem:** Starting MicroXRCE-DDS Agent before Gazebo fully loads causes the drone to spawn tilted. PX4 reads bad IMU data → accelerometer timeout → never arms.  
**Solution:** Launch simulation FIRST, wait 40s for Gazebo to fully settle and drone to stand upright, THEN start MicroXRCE.  
**Sessions:** 2, 3, 6

### Challenge 6: No Ground Control Station
**Problem:** PX4 requires a ground control station connection to clear the "Preflight Fail: No connection to the ground control station" check. Without QGroundControl running, arming is permanently denied.  
**Solution:** Added QGroundControl.AppImage launch to the automated script before MicroXRCE starts.  
**Session:** 6 (discovered after 5 sessions of other debugging)

### Challenge 7: Camera Frame ID
**Problem:** Assumed camera frame was `camera_link` but actual Gazebo model uses `OakD-Lite-Modify/base_link`.  
**Solution:** Verified via `ros2 topic echo` and corrected static TF publisher arguments.  
**Sessions:** 1, 2

### Challenge 8: Arming Bug in State Machine
**Problem:** `counter == 5` exact match + `elif` on state machine meant arming only attempted once, and arming and state transitions were mutually exclusive.  
**Solution:** Changed to `counter >= 5` (continuous arming) and `if` instead of `elif` (both can execute).  
**Session:** 4

### Challenge 9: Executable Not Found
**Problem:** ament_python installs scripts with `.py` extension in lib/, but ROS2 Node() executable lookup strips the extension.  
**Solution:** Used ExecuteProcess in launch file to run px4_odom_converter.py directly with `python3` instead of as a ROS2 node.  
**Sessions:** 5, 6

---

## Results and Performance

### Main Mission Results
| Trial | Cylinder Detection | ArUco Detection | Landing | Status |
|-------|-------------------|-----------------|---------|--------|
| 1 | Both detected | Marker found on tall cylinder | Precise landing at z=10.0m | ✅ Pass |
| 2 | Both detected | Marker found on tall cylinder | Precise landing at z=10.0m | ✅ Pass |
| 3 | Both detected | Marker found on tall cylinder | Precise landing at z=10.0m | ✅ Pass |

### Extra Credit Results
| Metric | Previous Best (Session 4) | Final (Session 6) | Improvement |
|--------|--------------------------|-------------------|-------------|
| Database Size | 9.1 MB (all poses at 0,0,0) | 389 MB | 42x larger, valid poses |
| Frames with Valid Poses | 0 / 52 | 78 / 143 | 0% → 55% |
| 3D Map Quality | Empty (black screen) | Dense point cloud + textured mesh | Qualitative |
| Odometry Source | Visual (failed) | PX4 + camera timestamps | Robust |

### Debugging Effort
| Metric | Value |
|--------|-------|
| Total Sessions | 6 (Feb 28 – Mar 3, 2026) |
| Root Causes Identified | 9 |
| Simulation Runs | ~20+ |
| Key Breakthrough | Camera timestamp sync + BEST_EFFORT QoS + QGC in script |

---

## How to Run

### Prerequisites
- ROS2 Jazzy on Ubuntu 24.04
- PX4 Autopilot (SITL) installed at ~/PX4-Autopilot
- QGroundControl AppImage at ~/QGroundControl.AppImage
- RTAB-Map: `sudo apt install ros-jazzy-rtabmap-ros`
- MicroXRCE-DDS Agent installed

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
```

### Run Main Mission Only
```bash
# Terminal 1: Simulation
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/terrain_mapping_drone_control/models:~/PX4-Autopilot/Tools/simulation/gz/models
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py px4_autopilot_path:=$HOME/PX4-Autopilot

# Terminal 2: MicroXRCE (after Gazebo loads)
MicroXRCEAgent udp4 -p 8888

# Terminal 3: QGroundControl
~/QGroundControl.AppImage

# Terminal 4: Mission (after QGC shows "Ready To Fly")
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch terrain_mapping_drone_control mission.launch.py
```

### Run Full Mission + RTAB-Map (Automated)
```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
./scripts/launch_all.sh
# Launches everything in correct order automatically
# Press Ctrl+C when mission completes
```

### Export 3D Mesh (After Mission)
```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
rtabmap-export --mesh --texture ~/.ros/rtabmap.db
# Outputs: rtabmap_mesh.obj, rtabmap_mesh.jpg, rtabmap_mesh.mtl
```

---

## Repository Structure
```
terrain_mapping_drone_control/
├── 3d_reconstruction/              # Extra credit output
│   ├── rtabmap_mesh.obj            # 3D textured mesh (5.0 MB, 43K vertices)
│   ├── rtabmap_mesh.jpg            # Texture map (1.1 MB)
│   ├── rtabmap_mesh.mtl            # Material file
│   └── rtabmap_poses.txt           # 78 optimized camera poses
├── launch/
│   ├── cylinder_landing.launch.py  # PX4 SITL + Gazebo launch
│   ├── mission.launch.py           # Mission nodes launch
│   └── rtabmap.launch.py           # RTAB-Map + odom converter launch
├── scripts/
│   ├── launch_all.sh               # One-command full launch (sim→QGC→DDS→RTAB→mission)
│   └── export_mesh.sh              # Mesh export from rtabmap.db
├── terrain_mapping_drone_control/
│   ├── auto_detect_land.py         # Main mission controller (state machine)
│   ├── aruco_tracker.py            # ArUco marker detection node
│   └── px4_odom_converter.py       # PX4→RTAB-Map odometry bridge
├── models/                         # Gazebo world models (cylinders)
├── extra_credit_files/             # Deployment scripts for extra credit setup
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md                       # This file
```

---

## References

1. ROS2 Documentation — Jazzy Distribution: https://docs.ros.org/en/jazzy/
2. PX4 Autopilot User Guide: https://docs.px4.io/main/en/
3. RTAB-Map ROS2 Wiki: https://github.com/introlab/rtabmap_ros
4. MicroXRCE-DDS Agent: https://micro-xrce-dds.docs.eprosima.com/
5. QGroundControl: http://qgroundcontrol.com/
6. OpenCV ArUco Module: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
7. Course Materials: SES598 Space Robotics and AI, Arizona State University

---

## Acknowledgments

**Course:** SES598 Space Robotics and AI  
**University:** Arizona State University  
**Tools:** ROS2 Jazzy, PX4 SITL, Gazebo, RTAB-Map, QGroundControl, OpenCV, Python 3

---

**End of Report**
