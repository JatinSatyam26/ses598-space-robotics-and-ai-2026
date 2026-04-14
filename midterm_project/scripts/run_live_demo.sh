#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
#  SES 598 Live Demo — Aerial-Ground Cooperative Autonomy
#  Physics-based drone (MulticopterVelocityControl) + DiffDrive rover
#
#  ONE COMMAND: bash scripts/run_live_demo.sh
# ═══════════════════════════════════════════════════════════════════════════════
set -o pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORLD_SDF="${PROJECT_DIR}/worlds/demo_world.sdf"
ROVER_SDF="${PROJECT_DIR}/models/mars_rover/model.sdf"
FLIGHT_DATA="$HOME/midterm_flight_data"
RESULTS="$HOME/midterm_results"
WORLD_NAME="demo_world"
PX4_MODELS="$HOME/PX4-Autopilot/Tools/simulation/gz/models"

# ── Gazebo resource path ──────────────────────────────────────────────────────
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PX4_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"

# ── Timeouts ──────────────────────────────────────────────────────────────────
PHASE_A_TIMEOUT=360    # 6 min
PHASE_B_TIMEOUT=180    # 3 min
GZ_BOOT_WAIT=15        # Gazebo startup

# ── Process tracking ──────────────────────────────────────────────────────────
PIDS=()

cleanup() {
  echo ""
  echo "════════════════════════════════════════"
  echo "  CLEANUP"
  echo "════════════════════════════════════════"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  sleep 1
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "parameter_bridge" 2>/dev/null || true
  pkill -9 -f "flight_control_node" 2>/dev/null || true
  pkill -9 -f "rover_driver" 2>/dev/null || true
  pkill -9 -f "demo_image_saver" 2>/dev/null || true
  echo "  Done."
}
trap cleanup EXIT

# ── Source ROS2 ───────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && \
  source "$HOME/ros2_ws/install/setup.bash" || true

echo ""
echo "╔═══════════════════════════════════════════════════════════════════════╗"
echo "║     SES 598 — Aerial-Ground Cooperative Autonomy Live Demo            ║"
echo "╚═══════════════════════════════════════════════════════════════════════╝"
echo "  $(date)"
echo ""
echo "  GZ_PATH: ${GZ_SIM_RESOURCE_PATH:0:80}…"
echo ""

[[ ! -f "$WORLD_SDF" ]] && { echo "ERROR: $WORLD_SDF not found"; exit 1; }
mkdir -p "${FLIGHT_DATA}/rgb" "${FLIGHT_DATA}/depth" "${RESULTS}"

# ════════════════════════════════════════════════════════════════════════════
echo "════════════════════════════════════════"
echo "  PHASE A — DRONE SURVEY"
echo "  $(date '+%H:%M:%S')"
echo "════════════════════════════════════════"

# ── 1. Launch Gazebo ─────────────────────────────────────────────────────────
echo "[A1] Launching Gazebo (${WORLD_NAME})…"
GZ_GUI=0 gz sim -r "${WORLD_SDF}" >/tmp/gz_stdout.log 2>/tmp/gz_stderr.log &
GZ_PID=$!
PIDS+=($GZ_PID)
echo "     Gazebo PID: ${GZ_PID} — booting (${GZ_BOOT_WAIT}s)…"
sleep "${GZ_BOOT_WAIT}"

# ── 2. Enable MulticopterVelocityControl ─────────────────────────────────────
echo "[A2] Enabling velocity controller…"
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5
gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
  2>/dev/null || true
sleep 0.5

# ── 3. Start ROS-Gazebo bridge ────────────────────────────────────────────────
echo "[A3] Starting ROS2-Gazebo bridge…"
# Direction: ] = ROS2→GZ (publish from ROS2 to GZ)
#            [ = GZ→ROS2 (subscribe from GZ to ROS2)
ros2 run ros_gz_bridge parameter_bridge \
  "/flying_drone/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/flying_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/mono_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args \
  -r /flying_drone/gazebo/command/twist:=/drone/cmd_vel \
  -r /model/flying_drone/odometry:=/drone/odom \
  -r /mono_camera:=/drone/down_mono \
  -r /rgb_camera:=/drone/front_rgb \
  -r /depth_camera:=/drone/front_depth \
  2>/tmp/drone_bridge.log &
BRIDGE_PID=$!
PIDS+=($BRIDGE_PID)
sleep 4
echo "     Drone bridge PID: ${BRIDGE_PID}"

# ── 4. Start image saver ──────────────────────────────────────────────────────
echo "[A4] Starting image saver…"
ros2 run midterm_project demo_image_saver 2>/tmp/img_saver.log &
IMG_PID=$!
PIDS+=($IMG_PID)
sleep 2

# ── 5. Start flight control ───────────────────────────────────────────────────
echo "[A5] Starting flight controller…"
echo "     Takeoff → lawnmower survey → land"
ros2 run midterm_project flight_control_node 2>&1 | tee /tmp/flight_ctrl.log &
FLIGHT_PID=$!
PIDS+=($FLIGHT_PID)

echo ""
echo "  Drone flying survey. Waiting for 'Mission complete'…"
echo "  Phase A timeout: ${PHASE_A_TIMEOUT}s"
echo ""

elapsed=0
while [[ $elapsed -lt $PHASE_A_TIMEOUT ]]; do
  if grep -qE "Mission complete|mission DONE" /tmp/flight_ctrl.log 2>/dev/null; then
    echo "  ✓ Drone survey COMPLETE — $(date '+%H:%M:%S')"
    break
  fi
  sleep 5
  elapsed=$((elapsed + 5))
  FRAMES=$(ls "${FLIGHT_DATA}/rgb/" 2>/dev/null | wc -l)
  WPS=$(grep -c "Waypoint.*reached" /tmp/flight_ctrl.log 2>/dev/null || true)
  ALT=$(grep "Odometry\|odom\|pos" /tmp/flight_ctrl.log 2>/dev/null | tail -1 || true)
  echo "  [${elapsed}s] WPs:${WPS}/10  Frames:${FRAMES}"
done

[[ $elapsed -ge $PHASE_A_TIMEOUT ]] && \
  echo "  WARNING: Phase A timed out — continuing"

FRAME_COUNT=$(ls "${FLIGHT_DATA}/rgb/" 2>/dev/null | wc -l)
echo ""
echo "  Frames captured: ${FRAME_COUNT}"

# Stop flight controller
kill $FLIGHT_PID 2>/dev/null || true
sleep 1

# Stop image saver gracefully so it writes poses.json
echo "  Saving poses.json…"
kill -TERM $IMG_PID 2>/dev/null || true
sleep 3   # allow 3s for poses.json to be written

# ════════════════════════════════════════════════════════════════════════════
echo ""
echo "════════════════════════════════════════"
echo "  PHASE B — ROVER NAVIGATION"
echo "  $(date '+%H:%M:%S')"
echo "════════════════════════════════════════"

# Rover is already in demo_world.sdf at (10,-4) — no spawn needed.
# ── 1. Bridge rover topics ────────────────────────────────────────────────────
# Rover DiffDrive uses global /cmd_vel and /odom (no model namespace in GZ)
echo "[B1] Starting rover bridge…"
ros2 run ros_gz_bridge parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  --ros-args \
  -r /cmd_vel:=/rover/cmd_vel \
  -r /odom:=/rover/odom \
  2>/tmp/rover_bridge.log &
ROVER_BRIDGE_PID=$!
PIDS+=($ROVER_BRIDGE_PID)
sleep 3
echo "     Rover bridge PID: ${ROVER_BRIDGE_PID}"

# ── 2. Start rover driver ─────────────────────────────────────────────────────
echo "[B2] Starting rover driver (following A* path around obstacles)…"
ros2 run midterm_project rover_driver 2>&1 | tee /tmp/rover_driver.log &
ROVER_PID=$!
PIDS+=($ROVER_PID)

echo ""
echo "  Rover navigating. Waiting for 'Navigation complete'…"
echo "  Phase B timeout: ${PHASE_B_TIMEOUT}s"
echo ""

elapsed=0
while [[ $elapsed -lt $PHASE_B_TIMEOUT ]]; do
  if grep -q "Navigation complete" /tmp/rover_driver.log 2>/dev/null; then
    echo "  ✓ Rover navigation COMPLETE — $(date '+%H:%M:%S')"
    break
  fi
  sleep 5
  elapsed=$((elapsed + 5))
  WPS=$(grep -c "Waypoint.*reached" /tmp/rover_driver.log 2>/dev/null || true)
  echo "  [${elapsed}s] Rover WPs completed: ${WPS}"
done

[[ $elapsed -ge $PHASE_B_TIMEOUT ]] && \
  echo "  WARNING: Phase B timed out"

# ════════════════════════════════════════════════════════════════════════════
echo ""
echo "════════════════════════════════════════"
echo "  PHASE C — SUMMARY"
echo "  $(date '+%H:%M:%S')"
echo "════════════════════════════════════════"

FRAME_COUNT=$(ls "${FLIGHT_DATA}/rgb/" 2>/dev/null | wc -l)
POSE_COUNT=$(python3 -c \
  "import json; print(len(json.load(open('${FLIGHT_DATA}/poses.json'))))" \
  2>/dev/null || echo 0)
ROVER_PTS=$(python3 -c \
  "import json; d=json.load(open('${RESULTS}/rover_actual_path.json')); print(len(d))" \
  2>/dev/null || echo 0)

echo ""
echo "  Drone frames  : ${FRAME_COUNT}"
echo "  Pose records  : ${POSE_COUNT}"
echo "  Rover points  : ${ROVER_PTS}"
echo ""
[[ $FRAME_COUNT -ge 50 ]] && \
  echo "  ✓ Frame count OK (${FRAME_COUNT} ≥ 50)" || \
  echo "  ✗ Frame count LOW (${FRAME_COUNT} < 50)"
echo ""
echo "  Data: ${FLIGHT_DATA}/rgb/  ${FLIGHT_DATA}/poses.json"
echo "  Data: ${RESULTS}/rover_actual_path.json"
echo ""
echo "════════════════════════════════════════"
