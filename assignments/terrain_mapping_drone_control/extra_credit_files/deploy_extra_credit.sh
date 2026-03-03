#!/bin/bash
#
# deploy_extra_credit.sh — Deploy all extra credit files
#
# This script:
#   1. Installs RTAB-Map (ros-jazzy)
#   2. Copies new/updated files to the project
#   3. Applies the arming bug fixes to auto_detect_land.py
#   4. Rebuilds the workspace
#
set -e

WS="$HOME/ros2_ws"
PKG_SRC="$WS/src/terrain_mapping_drone_control"
PKG_PY="$PKG_SRC/terrain_mapping_drone_control"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Step 1: Install RTAB-Map ==="
sudo apt update
sudo apt install -y \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-util \
    ros-jazzy-rtabmap-viz
echo "RTAB-Map installed."

echo ""
echo "=== Step 2: Copy new files ==="

# px4_odom_converter.py
cp "$SCRIPT_DIR/px4_odom_converter.py" "$PKG_PY/px4_odom_converter.py"
echo "  → px4_odom_converter.py"

# rtabmap.launch.py (overwrite original)
cp "$SCRIPT_DIR/rtabmap.launch.py" "$PKG_SRC/launch/rtabmap.launch.py"
echo "  → rtabmap.launch.py"

# launch_all.sh
cp "$SCRIPT_DIR/launch_all.sh" "$PKG_SRC/scripts/launch_all.sh"
chmod +x "$PKG_SRC/scripts/launch_all.sh"
echo "  → launch_all.sh"

# export_mesh.sh
cp "$SCRIPT_DIR/export_mesh.sh" "$PKG_SRC/scripts/export_mesh.sh"
chmod +x "$PKG_SRC/scripts/export_mesh.sh"
echo "  → export_mesh.sh"

echo ""
echo "=== Step 3: Fix arming bug in auto_detect_land.py ==="

ADL="$PKG_PY/auto_detect_land.py"

# Fix 1: == 5 → >= 5 (arm continuously, not just once)
sed -i 's/if self\.offboard_setpoint_counter == 5:/if self.offboard_setpoint_counter >= 5:/' "$ADL"

# Fix 2: elif self.state == "WAIT_INTRINSICS" → if self.state == "WAIT_INTRINSICS"
# (arming AND state machine must both run each tick, not either/or)
sed -i 's/elif self\.state == "WAIT_INTRINSICS":/if self.state == "WAIT_INTRINSICS":/' "$ADL"

echo "  → Fixed arming logic (>=5, elif→if)"

# Verify fixes applied
echo ""
echo "  Verifying fixes:"
grep -n "offboard_setpoint_counter >= 5" "$ADL" && echo "    ✓ >= 5 fix applied" || echo "    ✗ >= 5 fix FAILED"
grep -n 'if self.state == "WAIT_INTRINSICS"' "$ADL" && echo "    ✓ elif→if fix applied" || echo "    ✗ elif→if fix FAILED"

echo ""
echo "=== Step 4: Build workspace ==="
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source "$WS/install/setup.bash"

echo ""
echo "============================================"
echo "  Extra credit deployment complete!"
echo "============================================"
echo ""
echo "To run the mission:"
echo "  cd $PKG_SRC"
echo "  ./scripts/launch_all.sh"
echo ""
echo "After mission completes, export the mesh:"
echo "  ./scripts/export_mesh.sh"
echo ""
