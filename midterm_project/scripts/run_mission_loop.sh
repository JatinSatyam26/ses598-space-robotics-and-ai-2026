#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
#  Mars Scout Mission — Self-Improving Loop
#
#  Each iteration:
#    1. Cleanly kills any running Gazebo / ROS nodes
#    2. Records the run with ffmpeg (x11grab)
#    3. Runs the full mission (smart_flight_node + smart_rover_node)
#    4. Sends video + logs to Claude API for analysis
#    5. Reads the brief to learn what happened
#    6. Continues until all success criteria are met
#
#  Usage: bash scripts/run_mission_loop.sh [max_runs]
#  Default max_runs = 5
# ═══════════════════════════════════════════════════════════════════════════════
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MAX_RUNS="${1:-5}"
RESULTS_DIR="${HOME}/midterm_results"
mkdir -p "$RESULTS_DIR"

# Source ROS2
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && \
  source "$HOME/ros2_ws/install/setup.bash" || true

PX4_MODELS="$HOME/PX4-Autopilot/Tools/simulation/gz/models"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PX4_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"

WORLD_SDF="${PROJECT_DIR}/worlds/mars_mission.sdf"
DISPLAY_VAL="${DISPLAY:-:0}"

# ─── Cleanup function ─────────────────────────────────────────────────────────
kill_all() {
  echo "[loop] Killing all mission processes..."
  pkill -SIGINT -f "gz sim" 2>/dev/null || true
  sleep 2
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "parameter_bridge" 2>/dev/null || true
  pkill -9 -f "smart_flight_node" 2>/dev/null || true
  pkill -9 -f "smart_rover_node" 2>/dev/null || true
  pkill -9 -f "ruby.*gz" 2>/dev/null || true
  sleep 2
  echo "[loop] Clean."
}
trap 'kill_all; pkill -SIGINT -f ffmpeg 2>/dev/null; echo "Loop interrupted."; exit 0' INT TERM

# ─── Main loop ────────────────────────────────────────────────────────────────
RUN=0
while [[ $RUN -lt $MAX_RUNS ]]; do
  RUN=$((RUN + 1))
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  VIDEO_OUT="${RESULTS_DIR}/run_$(printf '%03d' $RUN)_${TIMESTAMP}.mp4"

  echo ""
  echo "╔══════════════════════════════════════════════════════════════╗"
  echo "║  MARS SCOUT MISSION — RUN ${RUN}/${MAX_RUNS}   $(date '+%H:%M:%S')"
  echo "╚══════════════════════════════════════════════════════════════╝"
  echo ""

  # ── 0. Clean slate ──────────────────────────────────────────────────────────
  kill_all
  > /tmp/smart_flight.log
  > /tmp/smart_rover.log
  > /tmp/bridge.log

  # ── 1. Start screen recording ───────────────────────────────────────────────
  RESOLUTION=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")
  echo "[run $RUN] Recording to: ${VIDEO_OUT}"
  ffmpeg -y -f x11grab -framerate 15 -video_size "${RESOLUTION}" \
    -i "${DISPLAY_VAL}" -c:v libx264 -preset ultrafast -crf 28 \
    "${VIDEO_OUT}" >/tmp/ffmpeg_${RUN}.log 2>&1 &
  FFMPEG_PID=$!
  sleep 2

  # ── 2. Launch Gazebo ─────────────────────────────────────────────────────────
  echo "[run $RUN] Launching Gazebo..."
  gz sim "${WORLD_SDF}" -r >/tmp/gz_stdout.log 2>/tmp/gz_stderr.log &
  GZ_PID=$!
  sleep 12

  if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "[run $RUN] ERROR: Gazebo crashed at startup. Check /tmp/gz_stderr.log"
    cat /tmp/gz_stderr.log | tail -20
    kill $FFMPEG_PID 2>/dev/null
    sleep 5
    continue
  fi
  echo "[run $RUN] Gazebo running (PID $GZ_PID)"

  # Enable drone velocity controller
  gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
    2>/dev/null || true
  sleep 0.5
  gz topic -t /flying_drone/enable --msgtype gz.msgs.Boolean -p 'data: true' \
    2>/dev/null || true

  # ── 3. Start bridge ──────────────────────────────────────────────────────────
  echo "[run $RUN] Starting ROS-Gazebo bridge..."
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
  sleep 3

  # ── 4. Start robot nodes ─────────────────────────────────────────────────────
  echo "[run $RUN] Starting drone brain..."
  ros2 run midterm_project smart_flight_node 2>&1 | tee /tmp/smart_flight.log &
  DRONE_PID=$!
  sleep 1

  echo "[run $RUN] Starting rover brain..."
  ros2 run midterm_project smart_rover_node 2>&1 | tee /tmp/smart_rover.log &
  ROVER_PID=$!

  echo ""
  echo "[run $RUN] Mission running — monitoring..."
  echo "           Flight log: /tmp/smart_flight.log"
  echo "           Rover  log: /tmp/smart_rover.log"
  echo ""

  # ── 5. Monitor until completion or timeout ────────────────────────────────────
  TIMEOUT=300  # 5 minutes max per run
  elapsed=0
  MISSION_DONE=false

  while [[ $elapsed -lt $TIMEOUT ]]; do
    sleep 10
    elapsed=$((elapsed + 10))

    DRONE_STATE=$(grep -oE "TAKEOFF|SURVEY|RETURN_HOME|LAND|DONE" \
      /tmp/smart_flight.log 2>/dev/null | tail -1 || echo "?")
    ROVER_STATE=$(grep -oE "WAITING|PLANNING|NAVIGATING|ARRIVED" \
      /tmp/smart_rover.log 2>/dev/null | tail -1 || echo "?")
    DRONE_WPS=$(grep -c "Waypoint.*reached" /tmp/smart_flight.log 2>/dev/null || echo 0)
    ROVER_WPS=$(grep -c "Waypoint.*reached" /tmp/smart_rover.log 2>/dev/null || echo 0)

    echo "  [${elapsed}s] Drone:${DRONE_STATE}(wp${DRONE_WPS}) Rover:${ROVER_STATE}(wp${ROVER_WPS})"

    # Success: rover arrived AND drone landed
    if grep -q "ARRIVED at habitat dome" /tmp/smart_rover.log 2>/dev/null && \
       grep -q "LANDED\|mission complete" /tmp/smart_flight.log 2>/dev/null; then
      MISSION_DONE=true
      echo ""
      echo "  ★ MISSION COMPLETE at ${elapsed}s ★"
      sleep 5  # let recording capture the final state
      break
    fi

    # Also exit if rover arrived (even if drone still flying)
    if grep -q "ARRIVED at habitat dome" /tmp/smart_rover.log 2>/dev/null; then
      MISSION_DONE=true
      echo "  ★ Rover ARRIVED (drone may still be active) ★"
      sleep 8
      break
    fi

    # Check Gazebo still alive
    if ! kill -0 $GZ_PID 2>/dev/null; then
      echo "  [run $RUN] Gazebo died unexpectedly"
      break
    fi
  done

  [[ $elapsed -ge $TIMEOUT ]] && echo "  [run $RUN] Timeout after ${TIMEOUT}s"

  # ── 6. Stop recording, clean up processes ─────────────────────────────────────
  echo "[run $RUN] Stopping recording..."
  kill -SIGINT $FFMPEG_PID 2>/dev/null || true
  sleep 4  # allow ffmpeg to finalize the mp4

  kill_all

  # ── 7. Analyze this run ───────────────────────────────────────────────────────
  echo ""
  echo "[run $RUN] Analyzing results..."
  if [[ -f "${VIDEO_OUT}" ]]; then
    VIDEO_SIZE=$(du -sh "${VIDEO_OUT}" | cut -f1)
    echo "[run $RUN] Video: ${VIDEO_OUT} (${VIDEO_SIZE})"
  else
    echo "[run $RUN] Warning: no video file found at ${VIDEO_OUT}"
    VIDEO_OUT=""
  fi

  python3 "${SCRIPT_DIR}/analyze_run.py" \
    "${VIDEO_OUT:-/dev/null}" "$RUN" "$RESULTS_DIR"
  ANALYZE_EXIT=$?

  # Read the brief back
  BRIEF_FILE="${RESULTS_DIR}/run_$(printf '%03d' $RUN)_brief.txt"
  STATUS_FILE="${RESULTS_DIR}/run_$(printf '%03d' $RUN)_status.json"

  if [[ -f "$STATUS_FILE" ]]; then
    ALL_PASSED=$(python3 -c \
      "import json; d=json.load(open('$STATUS_FILE')); print(d.get('all_passed', False))" \
      2>/dev/null || echo "False")
  else
    ALL_PASSED="False"
  fi

  echo ""
  echo "══ RUN $RUN COMPLETE ════════════════════════════════════"
  [[ -f "$BRIEF_FILE" ]] && cat "$BRIEF_FILE" || echo "(no brief)"
  echo "═══════════════════════════════════════════════════════"
  echo ""

  # ── 8. Check exit condition ───────────────────────────────────────────────────
  if [[ "$ALL_PASSED" == "True" ]]; then
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║  ALL SUCCESS CRITERIA MET — LOOP COMPLETE!               ║"
    echo "║  Total runs: $RUN                                         ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    break
  fi

  if [[ $RUN -ge $MAX_RUNS ]]; then
    echo "[loop] Maximum runs ($MAX_RUNS) reached."
    break
  fi

  echo "[loop] Starting run $((RUN + 1)) in 5 seconds..."
  sleep 5
done

echo ""
echo "[loop] All results in: ${RESULTS_DIR}/"
ls -la "${RESULTS_DIR}/"*.txt "${RESULTS_DIR}/"*.json 2>/dev/null | tail -20
