# SES 598 Midterm — Aerial-Ground Cooperative Autonomy

**Uncertainty-Aware 3D Gaussian Splatting for Drone Terrain Mapping and Rover Navigation**

**Student:** Jatin Satyam (`jsatyam@asu.edu`)
**Course:** SES 598 — Space Robotics and AI, Spring 2026, Arizona State University

---

## Overview

A complete two-agent autonomy pipeline running in Gazebo Harmonic / ROS 2 Jazzy:

1. **Drone survey** — PX4-SITL quadrotor flies a boustrophedon (lawnmower) pattern over
   a simulated Mars terrain at 5 m altitude, capturing downward-facing RGB images and
   forward-facing depth frames.
2. **3D reconstruction** — COLMAP builds a sparse SfM model; Nerfstudio `splatfacto`
   (3D Gaussian Splatting) trains a dense scene representation; a point cloud is exported.
3. **Traversability costmap** — The 3DGS point cloud is projected to a 2-D occupancy
   grid: cells with high Z-variance or tall obstacles → obstacle (cost 100); flat cells
   with adequate coverage → free (cost 0); poorly-observed cells → unknown (−1).
4. **Rover navigation** — A differential-drive Mars rover spawns in the same Gazebo
   world; Nav2 (DWB local planner + NavFn global planner) navigates it from (8, 0) to
   (−5, −5) using the drone-generated costmap, avoiding uncertain terrain.
5. **Evaluation** — Quantitative metrics, bird's-eye visualisation, and an
   uncertainty-awareness ablation study.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  PHASE A — Drone Survey (real-time)                                  │
│                                                                       │
│  PX4-SITL ──► Gazebo Harmonic                                        │
│     (x500 depth+mono)       │                                        │
│                              ├─ /drone/down_mono ──► image_saver ──► │
│  MicroXRCE-DDS ◄────────────┤                         ~/midterm_    │
│  QGroundControl              ├─ /drone/front_depth ──► flight_data/  │
│                              │                                        │
│  control_node ──► /fmu/in/trajectory_setpoint                        │
│  (boustrophedon survey ±10 m, 3 m row spacing, 5 m AGL)             │
└─────────────────────────────────────────────────────────────────────┘
                    │  1999 RGB frames + poses.json
                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  PHASE B — Offline Reconstruction                                     │
│                                                                       │
│  COLMAP automatic_reconstructor                                       │
│    └─► sparse/0/ {cameras,images,points3D}.bin                       │
│                                                                       │
│  nerfstudio splatfacto (15 000 iters, RTX 4060 ~30 min)             │
│    └─► 3DGS model + point_cloud.ply (500 k pts)                      │
│                                                                       │
│  generate_costmap.py                                                  │
│    └─► costmap.{pgm,yaml,debug.png}  (0.1 m/cell, inflation 0.2 m)  │
└─────────────────────────────────────────────────────────────────────┘
                    │  costmap.yaml
                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  PHASE C — Rover Navigation (real-time)                               │
│                                                                       │
│  Gazebo Harmonic (standalone, no PX4)                                 │
│    └─► mars_rover model (DiffDrive, 5 kg, 4 wheels)                  │
│                                                                       │
│  ros_gz_bridge:  /model/mars_rover/cmd_vel  ◄──► /rover/cmd_vel      │
│                  /model/mars_rover/odometry  ──►  /rover/odom         │
│                                                                       │
│  Nav2 bringup (DWB + NavFn + static costmap)                         │
│    map_server ──► global_costmap                                      │
│    bt_navigator ──► /navigate_to_pose                                 │
│                                                                       │
│  rover_navigator.py                                                   │
│    setInitialPose(8,0) ──► goToPose(-5,-5)                           │
│    └─► rover_path.json                                                │
└─────────────────────────────────────────────────────────────────────┘
                    │  rover_path.json + metrics
                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  PHASE D — Evaluation                                                 │
│                                                                       │
│  compute_metrics.py  ──► metrics.json                                 │
│  visualize_results.py ──► birdseye_result.png                         │
│  run_ablation.py      ──► ablation_{results.json,figure.png}          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

### System (Ubuntu 24.04 LTS)

```bash
# ROS 2 Jazzy + Gazebo Harmonic
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-nav2-dwb-core ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox colmap

# Python tools
pip install \
  nerfstudio==1.1.5 \
  pycolmap open3d plyfile numpy matplotlib Pillow \
  --break-system-packages
```

### Hardware / simulation

| Component | Notes |
|-----------|-------|
| GPU | NVIDIA RTX 4060 8 GB (CUDA 12.8) — for nerfstudio |
| PX4-Autopilot | Cloned to `~/PX4-Autopilot`, built with `make px4_sitl gz_x500_depth_mono` |
| QGroundControl | AppImage at `~/QGroundControl.AppImage` |
| Micro-XRCE-DDS | `MicroXRCEAgent` binary on `$PATH` |

### ROS workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/JatinSatyam26/ses598-space-robotics-and-ai-2026.git
cd ..
colcon build --packages-select midterm_project --symlink-install
source install/setup.bash
```

---

## Quick Start — Full Pipeline

```bash
cd ~/ses598-space-robotics-and-ai-2026/midterm_project/scripts
chmod +x run_full_pipeline.sh
./run_full_pipeline.sh
```

The script runs all four phases sequentially with live progress output.
Logs are written to `~/midterm_logs/`.

**Skip the 30-min 3DGS training** (use existing output):
```bash
SKIP_3DGS=1 ./run_full_pipeline.sh --phase B
```

**Use OpenSplat instead of nerfstudio** (if CUDA setup is problematic):
```bash
USE_OPENSPLAT=1 ./run_full_pipeline.sh --phase B
```

---

## Individual Phase Instructions

### Phase A — Drone Survey

Open **four terminals** (or use tmux). They must be started in order:

```bash
# Term 1 — PX4 + Gazebo
./scripts/term1_px4_gazebo.sh

# Term 2 — QGroundControl + DDS agent (after Gazebo is open)
./scripts/term2_qgc_dds.sh

# Term 3 — Bridge + control_node (after DDS shows heartbeat)
./scripts/term3_bridge_control.sh

# Term 4 — image_saver (after drone takes off)
./scripts/term4_perception.sh
```

The drone flies 10 waypoints in a ±10 m boustrophedon at 5 m AGL, hovers 3 s at
each waypoint, then lands. ~1999 images are saved to `~/midterm_flight_data/rgb/`.

### Phase B — Reconstruction

```bash
# COLMAP sparse SfM
./scripts/run_colmap.sh ~/midterm_flight_data/rgb/

# If COLMAP fails to register images (sim textures are low-frequency),
# use the pose-injected fallback:
python3 ./scripts/create_colmap_from_poses.py

# 3DGS training (~30 min on RTX 4060)
./scripts/train_3dgs.sh ~/midterm_reconstruction

# Dense point cloud export
./scripts/export_pointcloud.sh

# Traversability costmap
python3 ./scripts/generate_costmap.py ~/midterm_3dgs_export/point_cloud.ply \
    --output-dir ~/midterm_reconstruction \
    --resolution 0.1 --inflation-radius 0.2
```

### Phase C — Rover Navigation

```bash
# Requires Gazebo running + costmap at ~/midterm_reconstruction/costmap.yaml
./scripts/run_rover_nav.sh

# Override goal or costmap:
./scripts/run_rover_nav.sh -p goal_x:=-8.0 -p goal_y:=5.0
COSTMAP_YAML=~/my/costmap.yaml ./scripts/run_rover_nav.sh
```

### Phase D — Evaluation

```bash
python3 ./scripts/compute_metrics.py
python3 ./scripts/visualize_results.py
python3 ./scripts/run_ablation.py
```

---

## Results

### Bird's-Eye View

<!-- After running the pipeline, replace with actual output image -->

![Bird's-eye result](../../midterm_results/birdseye_result.png)

*Point cloud (plasma colormap by height) + traversability costmap + rover path (blue)
+ drone survey pattern (dashed gray)*

### Ablation Study

![Ablation figure](../../midterm_results/ablation_figure.png)

*Left: uncertainty-aware navigation (unknown cells blocked). Right: uncertainty-ignoring
navigation (unknown cells treated as free). The uncertainty-aware path is longer but
avoids poorly-observed terrain.*

### Metrics Summary

<!-- Generated by compute_metrics.py — shown here after a run -->

| Metric | Value |
|--------|-------|
| 3D points | — |
| Coverage area | — m² |
| Point density | — pts/m² |
| Free cells | — % |
| Obstacle cells | — % |
| Unknown cells | — % |
| Path length | — m |
| Path smoothness | — rad |
| Mission time | — s |

---

## File Structure

```
midterm_project/
├── config/
│   ├── mission_config.yaml       drone survey parameters
│   └── nav2_params.yaml          Nav2 DWB + NavFn configuration
├── launch/
│   ├── planetary_landing.launch.py   PX4+Gazebo launch
│   └── rover_nav.launch.py           rover spawn + Nav2 launch
├── midterm_project/              ROS2 Python package
│   ├── control_node.py           drone state machine (offboard + survey)
│   ├── image_saver.py            RGB + depth image capture node
│   └── rover_navigator.py        Nav2 BasicNavigator mission node
├── models/
│   ├── mars_rover/               differential-drive rover SDF
│   └── planetary_terrain/        heightmap terrain model
├── scripts/
│   ├── run_full_pipeline.sh      ← MASTER SCRIPT
│   ├── term{1..4}_*.sh           individual terminal scripts (Phase A)
│   ├── run_colmap.sh             COLMAP sparse reconstruction (Phase B)
│   ├── create_colmap_from_poses.py  pose-injected COLMAP fallback (Phase B)
│   ├── train_3dgs.sh             nerfstudio splatfacto training (Phase B)
│   ├── export_pointcloud.sh      ns-export pointcloud (Phase B)
│   ├── generate_costmap.py       3DGS → traversability costmap (Phase B)
│   ├── run_rover_nav.sh          rover launch + navigation (Phase C)
│   ├── compute_metrics.py        quantitative metrics (Phase D)
│   ├── visualize_results.py      bird's-eye figure (Phase D)
│   └── run_ablation.py           uncertainty ablation (Phase D)
└── worlds/
    └── planetary_landing.sdf     Mars terrain with boulders and spires
```

---

## Key Design Decisions

**Why downward camera for RGB?**
The front-facing camera at 5 m altitude sees near-identical horizon frames.
The downward mono camera (`/drone/down_mono`) captures varied ground texture,
giving COLMAP sufficient baseline for feature matching.

**Why 3-second waypoint hover?**
The DiffDrive plugin has a 50 Hz odometry publish rate. At 1 m/s survey speed,
a 3 s dwell ensures 150 new odometry messages per waypoint — sufficient for
image_saver to capture 30+ unique frames (at 5-frame subsampling).

**Why A\* for the ablation?**
Re-running Nav2 twice would require two live Gazebo sessions. A Python A\*
on the same costmap grid mirrors NavFn's behaviour faithfully enough to
demonstrate the uncertainty-awareness tradeoff: the uncertainty-aware path
is longer but crosses zero unknown cells.

**Why `localizer='robot_localization'` in BasicNavigator?**
This is the one magic string in nav2_simple_commander 1.3.10 that bypasses
both `_waitForNodeToActivate(localizer)` and `_waitForInitialPose()`. Since
AMCL is not running (we use Gazebo odometry directly), passing any other
string causes the navigator to hang waiting for AMCL to activate.

---

## References

1. **3D Gaussian Splatting** — Kerbl et al., *3D Gaussian Splatting for Real-Time
   Radiance Field Rendering*, SIGGRAPH 2023.
   https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/

2. **MASt3R** — Leroy et al., *Grounding Image Matching in 3D with MASt3R*,
   CVPR 2025. https://github.com/naver/mast3r

3. **Nav2** — Macenski et al., *The Marathon 2: A Navigation System*,
   IROS 2020. https://navigation.ros.org

4. **PX4 Autopilot** — Meier et al., *PX4: A Node-Based Multithreaded Open
   Source Robotics Framework for Deeply Embedded Platforms*, ICRA 2015.
   https://px4.io

5. **Nerfstudio** — Tancik et al., *Nerfstudio: A Modular Framework for
   Neural Radiance Field Development*, SIGGRAPH 2023.
   https://docs.nerf.studio
