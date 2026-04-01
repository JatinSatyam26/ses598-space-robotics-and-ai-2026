#!/bin/bash
# TERMINAL 3: ROS2 Bridge + Control Node
# Run AFTER Terminal 2 DDS agent is running and showing heartbeat.
echo "=== Terminal 3: Starting ROS2 bridge ==="
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /drone/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/front_depth@sensor_msgs/msg/Image[gz.msgs.Image \
  /drone/front_rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /drone/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo &

BRIDGE_PID=$!
echo "Bridge PID: $BRIDGE_PID"

echo "Waiting 3s for bridge to settle..."
sleep 5

echo "=== Starting control node ==="
ros2 run midterm_project control_node \
  --ros-args \
  --params-file ~/ses598-space-robotics-and-ai-2026/midterm_project/config/mission_config.yaml
