#!/bin/bash
# TERMINAL 1: PX4 + Gazebo
# Wait for Gazebo to fully open and drone to spawn before starting Terminal 2.
echo "=== Terminal 1: Starting PX4 + Gazebo ==="
source /opt/ros/jazzy/setup.bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=planetary_landing make px4_sitl gz_x500_depth_mono
