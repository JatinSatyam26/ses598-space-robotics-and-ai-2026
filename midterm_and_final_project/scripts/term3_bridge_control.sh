#!/bin/bash
# TERMINAL 3: ROS2 Bridge + Control Node
# Run AFTER Terminal 2 DDS agent is running and showing heartbeat.
#
# Bridge topics:
#   /drone/down_mono      — downward-facing mono camera (RGB input for image_saver)
#   /drone/front_rgb      — forward RGB camera (kept for reference)
#   /drone/front_depth    — forward depth camera (depth input for image_saver)
#   /drone/front_depth/camera_info — depth camera intrinsics
#
# NOTE: /clock is intentionally NOT bridged — it causes time sync jumps
#       that interfere with PX4 EKF and prevent arming.
echo "=== Terminal 3: Starting ROS2 bridge ==="
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /drone/down_mono@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/front_depth@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  --ros-args -r /mono_camera:=/drone/down_mono &

BRIDGE_PID=$!
echo "Bridge PID: $BRIDGE_PID"

echo "Waiting 5s for bridge to settle..."
sleep 5

echo "=== Starting control node ==="
ros2 run midterm_project control_node \
  --ros-args \
  --params-file ~/ses598-space-robotics-and-ai-2026/midterm_and_final_project/config/mission_config.yaml
