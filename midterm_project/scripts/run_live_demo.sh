#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
#  Mars Scout Mission — Live Demo Launch Script
#  Physics-based drone (MulticopterVelocityControl) + 6-wheel DiffDrive rover
#  Drone surveys terrain, builds occupancy grid, rover plans A* path to dome.
#
#  Usage: bash scripts/run_live_demo.sh
# ═══════════════════════════════════════════════════════════════════════════════
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && \
  source "$HOME/ros2_ws/install/setup.bash" || true

# Set Gazebo model path (include PX4 models for x500_base)
export PROJ_IGNORE_CELESTIAL_BODY=YES
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "=========================================="
echo "  MARS SCOUT MISSION — LIVE DEMO"
echo "  $(date)"
echo "=========================================="
echo ""
echo "  Project: ${PROJECT_DIR}"
echo "  GZ resource path: ${GZ_SIM_RESOURCE_PATH:0:80}"
echo ""

WORLD_SDF="${WORLD_SDF:-${PROJECT_DIR}/worlds/mars_mission.sdf}"
[[ "$WORLD_SDF" != /* ]] && WORLD_SDF="${PROJECT_DIR}/${WORLD_SDF}"
[[ ! -f "$WORLD_SDF" ]] && { echo "ERROR: $WORLD_SDF not found"; exit 1; }

# ── Process tracking ──────────────────────────────────────────────────────────
PIDS=()

cleanup() {
  echo ""
  echo "=========================================="
  echo "  SHUTTING DOWN"
  echo "=========================================="
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  sleep 1
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "parameter_bridge" 2>/dev/null || true
  pkill -9 -f "smart_flight_node" 2>/dev/null || true
  pkill -9 -f "smart_rover_node" 2>/dev/null || true
  echo "  Done."
}
trap cleanup EXIT

# ── 1. Launch Gazebo ──────────────────────────────────────────────────────────
echo "Launching Gazebo world..."
gz sim "${WORLD_SDF}" -r >/tmp/gz_stdout.log 2>/tmp/gz_stderr.log &
GZ_PID=$!
PIDS+=($GZ_PID)
echo "  Gazebo PID: ${GZ_PID} — waiting 10s for startup..."
sleep 10

# Enable the multicopter velocity controller
# Actual GZ topic: /flying_drone/enable (from robotNamespace in drone SDF)
echo "Enabling drone velocity controller..."
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5

# ── 2. List GZ topics for debugging ──────────────────────────────────────────
echo ""
echo "Available Gazebo topics:"
gz topic -l 2>/dev/null | sort || true
echo ""

# ── 3. Start ROS-Gazebo bridge ────────────────────────────────────────────────
echo "Starting ROS-Gazebo bridge..."
#
# Verified GZ topic names (from gz topic -l with mars_mission.sdf loaded):
#   Drone cmd_vel:    /flying_drone/gazebo/command/twist  (ROS→GZ ])
#   Drone odom:       /model/drone/odometry               (GZ→ROS [)
#   Drone rangefinder:/rangefinder                        (GZ→ROS [)
#   Drone depth cam:  /depth_camera                       (GZ→ROS [)
#   Rover cmd_vel:    /cmd_vel                            (ROS→GZ ])
#   Rover odom:       /rover_odom                         (GZ→ROS [)
#   Rover lidar:      /rover_lidar                        (GZ→ROS [)
#
ros2 run ros_gz_bridge parameter_bridge \
  "/flying_drone/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/rangefinder@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/rover_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/rover_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  --ros-args \
  -r /flying_drone/gazebo/command/twist:=/drone/cmd_vel \
  -r /model/drone/odometry:=/drone/odom \
  -r /rangefinder:=/drone/rangefinder \
  -r /depth_camera:=/drone/front_depth \
  -r /cmd_vel:=/rover/cmd_vel \
  -r /rover_odom:=/rover/odom \
  -r /rover_lidar:=/rover/lidar \
  2>/tmp/bridge.log &
BRIDGE_PID=$!
PIDS+=($BRIDGE_PID)
echo "  Bridge PID: ${BRIDGE_PID}"
sleep 3

# ── 4. Start drone smart flight node ─────────────────────────────────────────
echo "Starting smart flight node (drone brain)..."
ros2 run midterm_project smart_flight_node 2>&1 | tee /tmp/smart_flight.log &
DRONE_PID=$!
PIDS+=($DRONE_PID)
echo "  Drone node PID: ${DRONE_PID}"
sleep 1

# ── 5. Start rover smart node ─────────────────────────────────────────────────
echo "Starting smart rover node (rover brain)..."
ros2 run midterm_project smart_rover_node 2>&1 | tee /tmp/smart_rover.log &
ROVER_PID=$!
PIDS+=($ROVER_PID)
echo "  Rover node PID: ${ROVER_PID}"

echo ""
echo "=========================================="
echo "  ALL SYSTEMS RUNNING"
echo "  Drone: TAKEOFF → SURVEY → LAND"
echo "  Rover: WAITING → PLANNING → NAVIGATING"
echo "  Logs: /tmp/smart_flight.log"
echo "        /tmp/smart_rover.log"
echo "        /tmp/bridge.log"
echo "  Press Ctrl+C to stop"
echo "=========================================="

# Monitor progress
elapsed=0
while true; do
  sleep 10
  elapsed=$((elapsed + 10))

  DRONE_STATE=$(grep -oE "TAKEOFF|SURVEY|PUBLISH_MAP|LAND|DONE" \
    /tmp/smart_flight.log 2>/dev/null | tail -1 || echo "?")
  ROVER_STATE=$(grep -oE "WAITING|PLANNING|NAVIGATING|ARRIVED" \
    /tmp/smart_rover.log 2>/dev/null | tail -1 || echo "?")
  DRONE_WPS=$(grep -c "Reached waypoint" /tmp/smart_flight.log 2>/dev/null || echo 0)
  ROVER_WPS=$(grep -c "Reached waypoint" /tmp/smart_rover.log 2>/dev/null || echo 0)

  echo "  [${elapsed}s] Drone: ${DRONE_STATE} (WP ${DRONE_WPS}/8)  Rover: ${ROVER_STATE} (WP ${ROVER_WPS})"

  if grep -q "ARRIVED at habitat dome" /tmp/smart_rover.log 2>/dev/null; then
    echo ""
    echo "=========================================="
    echo "  MISSION COMPLETE!"
    echo "  Rover reached the habitat dome."
    echo "  $(date)"
    echo "=========================================="
    sleep 5
    break
  fi

  # Exit if either node crashed
  if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "WARNING: Gazebo process ended"
    break
  fi
done

# Wait
wait -n $GZ_PID $DRONE_PID $ROVER_PID 2>/dev/null || true
