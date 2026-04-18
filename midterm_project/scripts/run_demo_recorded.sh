#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
#  Mars Scout Mission — Screen-Recorded Demo
#  Runs the full mission while capturing a video with ffmpeg x11grab.
#
#  Usage: bash scripts/run_demo_recorded.sh
# ═══════════════════════════════════════════════════════════════════════════════
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && \
  source "$HOME/ros2_ws/install/setup.bash" || true

# Set Gazebo model path (include PX4 models for x500_base)
PX4_MODELS="$HOME/PX4-Autopilot/Tools/simulation/gz/models"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PX4_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"

RESULTS_DIR="${HOME}/midterm_results"
mkdir -p "$RESULTS_DIR"

DISPLAY_VAL="${DISPLAY:-:0}"
RESOLUTION=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
VIDEO_OUT="${RESULTS_DIR}/mars_scout_demo_${TIMESTAMP}.mp4"

echo "=========================================="
echo "  MARS SCOUT MISSION — RECORDED DEMO"
echo "  $(date)"
echo "=========================================="
echo ""
echo "  Recording to: ${VIDEO_OUT}"
echo "  Resolution:   ${RESOLUTION}"
echo "  Display:      ${DISPLAY_VAL}"
echo ""

WORLD_SDF="${PROJECT_DIR}/worlds/mars_mission.sdf"
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

  # Stop ffmpeg gracefully (send q to stdin)
  if [[ -n "${FFMPEG_PID:-}" ]]; then
    kill -SIGINT "$FFMPEG_PID" 2>/dev/null || true
    sleep 3
  fi

  echo "  Video saved to: ${VIDEO_OUT}"
  echo "  Done."
}
trap cleanup EXIT

# ── Start ffmpeg recording ────────────────────────────────────────────────────
echo "Starting screen recording..."
ffmpeg -y \
  -f x11grab \
  -framerate 15 \
  -video_size "${RESOLUTION}" \
  -i "${DISPLAY_VAL}" \
  -c:v libx264 \
  -preset ultrafast \
  -crf 28 \
  "${VIDEO_OUT}" \
  >/tmp/ffmpeg_record.log 2>&1 &
FFMPEG_PID=$!
PIDS+=($FFMPEG_PID)
echo "  ffmpeg PID: ${FFMPEG_PID}"
sleep 2

# ── 1. Launch Gazebo ──────────────────────────────────────────────────────────
echo "Launching Gazebo world..."
gz sim "${WORLD_SDF}" -r >/tmp/gz_stdout.log 2>/tmp/gz_stderr.log &
GZ_PID=$!
PIDS+=($GZ_PID)
echo "  Gazebo PID: ${GZ_PID} — waiting 12s for startup..."
sleep 12

# Enable the multicopter velocity controller
# Actual GZ topic: /flying_drone/enable (from robotNamespace in drone SDF)
echo "Enabling drone velocity controller..."
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5

# ── 2. Start ROS-Gazebo bridge ────────────────────────────────────────────────
echo "Starting ROS-Gazebo bridge..."
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

# ── 3. Start drone smart flight node ─────────────────────────────────────────
echo "Starting smart flight node (drone brain)..."
ros2 run midterm_project smart_flight_node 2>&1 | tee /tmp/smart_flight.log &
DRONE_PID=$!
PIDS+=($DRONE_PID)
sleep 1

# ── 4. Start rover smart node ─────────────────────────────────────────────────
echo "Starting smart rover node (rover brain)..."
ros2 run midterm_project smart_rover_node 2>&1 | tee /tmp/smart_rover.log &
ROVER_PID=$!
PIDS+=($ROVER_PID)

echo ""
echo "=========================================="
echo "  ALL SYSTEMS RUNNING — RECORDING"
echo "  Video: ${VIDEO_OUT}"
echo "  Drone: TAKEOFF → SURVEY → LAND"
echo "  Rover: WAITING → PLANNING → NAVIGATING"
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
    sleep 8  # extra time for recording to capture the final state
    break
  fi

  if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "WARNING: Gazebo process ended"
    break
  fi
done

echo "Stopping recording..."
