#!/bin/bash
#
# launch_all.sh — Runs the full extra credit mission:
#   1. PX4 SITL + Gazebo + Bridge (cylinder_landing.launch.py)
#   2. RTAB-Map + Odom Converter (rtabmap.launch.py)
#   3. Drone Mission (mission.launch.py)
#
# After mission completes, run export_mesh.sh to extract the 3D map.
#
set -e

# ── Configuration ──────────────────────────────────────────────
WS="$HOME/ros2_ws"
PKG="terrain_mapping_drone_control"
DB_PATH="$HOME/.ros/rtabmap.db"

# ── Clean slate ────────────────────────────────────────────────
echo "=== Cleaning old RTAB-Map database ==="
rm -f "$DB_PATH"

# ── Source workspace ───────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# ── Build ──────────────────────────────────────────────────────
echo "=== Building workspace ==="
cd "$WS"
colcon build --packages-select "$PKG" --symlink-install
source "$WS/install/setup.bash"

# ── Launch Step 1: PX4 SITL + Gazebo + Bridge ─────────────────
echo ""
echo "=== Step 1: Launching PX4 SITL + Gazebo ==="
ros2 launch "$PKG" cylinder_landing.launch.py &
PID_SIM=$!
echo "PID (sim): $PID_SIM"

# Wait for simulation to initialize
echo "Waiting 15s for simulation to start..."
sleep 15

# ── Launch Step 2: RTAB-Map + Odom Converter ──────────────────
echo ""
echo "=== Step 2: Launching RTAB-Map ==="
ros2 launch "$PKG" rtabmap.launch.py &
PID_RTAB=$!
echo "PID (rtabmap): $PID_RTAB"

# Wait for RTAB-Map to initialize
echo "Waiting 10s for RTAB-Map to initialize..."
sleep 10

# ── Launch Step 3: Drone Mission ──────────────────────────────
echo ""
echo "=== Step 3: Launching Drone Mission ==="
ros2 launch "$PKG" mission.launch.py &
PID_MISSION=$!
echo "PID (mission): $PID_MISSION"

# ── Wait for mission ──────────────────────────────────────────
echo ""
echo "=== All launched. Press Ctrl+C to stop everything. ==="
echo "=== After mission completes, run: ./scripts/export_mesh.sh ==="
echo ""

# Wait for any process to exit, then clean up all
wait $PID_MISSION 2>/dev/null || true

echo ""
echo "=== Mission process ended. Shutting down... ==="
kill $PID_RTAB 2>/dev/null || true
kill $PID_SIM 2>/dev/null || true
wait 2>/dev/null || true

echo ""
echo "=== Done. Database at: $DB_PATH ==="
echo "=== Run ./scripts/export_mesh.sh to export the 3D mesh. ==="
