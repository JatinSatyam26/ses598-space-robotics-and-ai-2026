#!/bin/bash
# TERMINAL 4: Perception + Image Saver
# Run AFTER control node is running and drone has taken off.
echo "=== Terminal 4: Starting image saver ==="
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

mkdir -p ~/ses598-space-robotics-and-ai-2026/midterm_project/data/images

ros2 run midterm_project image_saver \
  --ros-args \
  --params-file ~/ses598-space-robotics-and-ai-2026/midterm_project/config/mission_config.yaml
