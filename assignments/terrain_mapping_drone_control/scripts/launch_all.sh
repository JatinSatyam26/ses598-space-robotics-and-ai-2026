#!/bin/bash
set -e

WS="$HOME/ros2_ws"
PKG="terrain_mapping_drone_control"
DB_PATH="$HOME/.ros/rtabmap.db"

echo "=== Cleaning old RTAB-Map database ==="
rm -f "$DB_PATH"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "=== Building workspace ==="
cd "$WS"
colcon build --packages-select "$PKG" --symlink-install
source "$WS/install/setup.bash"

echo ""
echo "=== Step 1: Launching PX4 SITL + Gazebo FIRST ==="
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:$WS/src/$PKG/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models
ros2 launch "$PKG" cylinder_landing.launch.py px4_autopilot_path:=$HOME/PX4-Autopilot &
PID_SIM=$!
echo "PID (sim): $PID_SIM"

echo "Waiting 40s for Gazebo to fully load and drone to stand upright..."
sleep 40

echo ""
echo "=== Step 2: Starting MicroXRCE-DDS Agent (AFTER sim) ==="
# Start QGroundControl in background
~/QGroundControl.AppImage &> /dev/null &
PID_QGC=$!
echo "PID (QGC): $PID_QGC"
sleep 3
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
PID_DDS=$!
echo "PID (dds agent): $PID_DDS"

echo "Waiting 10s for PX4 topics to appear..."
sleep 10

echo ""
echo "=== Step 3: Launching RTAB-Map ==="
ros2 launch "$PKG" rtabmap.launch.py &
PID_RTAB=$!
echo "PID (rtabmap): $PID_RTAB"

echo "Waiting 10s for RTAB-Map to initialize..."
sleep 10

echo ""
echo "=== Step 4: Launching Drone Mission ==="
ros2 launch "$PKG" mission.launch.py &
PID_MISSION=$!
echo "PID (mission): $PID_MISSION"

echo ""
echo "============================================"
echo "  All launched! Watch the RTAB-Map window."
echo "  Press Ctrl+C when mission completes."
echo "  Then run: ./scripts/export_mesh.sh"
echo "============================================"
echo ""

wait $PID_MISSION 2>/dev/null || true

echo ""
echo "=== Shutting down... ==="
kill $PID_RTAB 2>/dev/null || true
kill $PID_DDS 2>/dev/null || true
kill $PID_SIM 2>/dev/null || true
kill $PID_QGC 2>/dev/null || true
wait 2>/dev/null || true

echo "=== Done. Database at: $DB_PATH ==="
echo "=== Run: ./scripts/export_mesh.sh ==="
