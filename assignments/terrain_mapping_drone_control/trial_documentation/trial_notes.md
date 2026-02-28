# Cylinder Landing Mission - Trial Documentation

## Hardware Configuration
- Laptop: OMEN by HP Gaming Laptop 16
- CPU: Intel Core (12th Gen)
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU (8GB VRAM)
- RAM: 15.6 GB
- OS: Ubuntu 24.04 LTS

## Software Environment
- ROS2: Jazzy
- PX4-Autopilot commit: 9ac03f03eb
- Gazebo: Harmonic
- px4_msgs: matched to PX4 firmware (msg/versioned/)
- QGroundControl: v4.4.4
- NVIDIA Driver: 590.48.01 (installed during trial process)
- MicroXRCE-DDS Agent: UDP port 8888

## GPU Configuration Per Trial
| Trial | GPU Used | Gazebo Real-Time % | Notes |
|-------|----------|-------------------|-------|
| Trial 1 | Intel UHD Graphics (integrated) | ~63% | NVIDIA driver not installed at time of trial |
| Trial 2 | NVIDIA GeForce RTX 4060 | ~98% | NVIDIA driver 590 installed and configured |
| Trial 3 | NVIDIA GeForce RTX 4060 | ~98% | Same config as Trial 2 |

## Results Summary
| Trial | GPU | Duration | Battery Used | Real-Time % | Result |
|-------|-----|----------|--------------|-------------|--------|
| Trial 1 | Intel UHD | 163.49 sec | 0.000% | ~63% | Success |
| Trial 2 | NVIDIA RTX 4060 | 111.41 sec | 0.000% | ~98% | Success |
| Trial 3 | NVIDIA RTX 4060 | 110.98 sec | 0.000% | ~98% | Success |

## Average Results (NVIDIA GPU trials only - Trials 2 and 3)
- Average Mission Duration: 111.20 seconds
- Average Battery Used: 0.000%
- Success Rate: 100%

## Launch Commands Used

### Terminal 1 - MicroXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

### Terminal 2 - PX4 SITL + Gazebo + Cylinders
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/terrain_mapping_drone_control/models:~/PX4-Autopilot/Tools/simulation/gz/models && ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py px4_autopilot_path:=/home/jatin-satyam/PX4-Autopilot

### Terminal 3 - QGroundControl (satisfies PX4 GCS preflight check)
~/QGroundControl.AppImage

### Terminal 4 - Mission Node with logging
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch terrain_mapping_drone_control mission.launch.py 2>&1 | tee ~/ses598-space-robotics-and-ai-2026/assignments/terrain_mapping_drone_control/trial_documentation/trialX/logs/mission_log.txt

## Key Observations
1. Intel integrated GPU caused simulation to run at ~63% real-time, increasing mission duration by ~47% compared to NVIDIA GPU trials
2. Two failed landing attempts occurred during Intel GPU trials due to insufficient real-time performance causing imprecise landing
3. NVIDIA RTX 4060 brought simulation to ~98% real-time, significantly improving landing precision and consistency
4. Mission duration was highly consistent across NVIDIA GPU trials (111.41s vs 110.98s, difference of only 0.43s)
5. Battery consumption was 0.000% in all trials (simulation battery behavior)
6. QGroundControl is required before mission launch to satisfy PX4 GCS preflight check
7. ArUco marker detection worked reliably on the tallest cylinder in all successful trials

## Failed Attempts
- 2 failed landing attempts during Intel GPU phase
- Root cause: Low real-time factor (~45-63%) caused timing issues in precision landing sequence
- Fix: Installing NVIDIA driver 590 and configuring PRIME render offload resolved the issue

## Documentation Structure
trial_documentation/
├── trial_notes.md          (this file)
├── trial1/
│   ├── logs/mission_log.txt
│   ├── screenshots/
│   └── Screencast from 2026-02-27 13-09-37.mp4
├── trial2/
│   ├── logs/mission_log.txt
│   ├── screenshots/
│   └── (screen recording)
└── trial3/
    ├── logs/mission_log.txt
    ├── screenshots/
    └── (screen recording)
