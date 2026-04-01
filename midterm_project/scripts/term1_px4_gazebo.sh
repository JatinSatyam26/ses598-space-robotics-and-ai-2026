#!/bin/bash
# TERMINAL 1: PX4 + Gazebo
# Run this first. Wait for Gazebo to fully open before starting Terminal 2.
echo "=== Terminal 1: Starting PX4 + Gazebo ==="
source /opt/ros/jazzy/setup.bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=planetary_landing ./build/px4_sitl_default/bin/px4 ./build/px4_sitl_default/etc -s etc/init.d-posix/rcS
