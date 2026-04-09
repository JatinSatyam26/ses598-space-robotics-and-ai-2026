#!/bin/bash
# Comprehensive mission launcher for SES 598 midterm
# Fix C: Proper sequenced startup to avoid DDS time sync death spiral
# Fix D already applied: UXRCE_DDS_SYNCT=0 in airframe config
set -euo pipefail

ATTEMPT="${1:-1}"
LOG_DIR=~/midterm_debug
LOG="$LOG_DIR/attempt_${ATTEMPT}.log"
FLIGHT_DATA=~/midterm_flight_data
ROS2_WS=~/ros2_ws

echo "=== MISSION ATTEMPT $ATTEMPT ===" | tee "$LOG"
echo "Start time: $(date)" | tee -a "$LOG"

# ── Cleanup helper ──────────────────────────────────────────────────────────
cleanup() {
    echo "=== CLEANUP ===" | tee -a "$LOG"
    # Kill process groups to take down all children
    for pid_file in /tmp/midterm_*.pid; do
        [ -f "$pid_file" ] || continue
        pid=$(cat "$pid_file")
        name=$(basename "$pid_file" .pid)
        if kill -0 "$pid" 2>/dev/null; then
            echo "Killing $name (pgid $pid)..." | tee -a "$LOG"
            kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
        fi
        rm -f "$pid_file"
    done
    # Catch stragglers by name
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gz sim\|gzserver\|gzclient" 2>/dev/null || true
    pkill -f "MicroXRCEAgent\|micro_ros_agent" 2>/dev/null || true
    pkill -f "QGroundControl" 2>/dev/null || true
    pkill -f "ros2 run midterm_project" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

# ── Kill any existing processes first ──────────────────────────────────────
echo "Killing any existing simulation processes..." | tee -a "$LOG"
pkill -f "px4|gz sim|MicroXRCEAgent|QGroundControl" 2>/dev/null || true
pkill -f "ros2 run midterm_project|parameter_bridge" 2>/dev/null || true
sleep 3

# ── Source ROS2 ──────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

# ── PHASE 1: Start PX4 + Gazebo ──────────────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 1: Starting PX4 + Gazebo ===" | tee -a "$LOG"
mkdir -p "$FLIGHT_DATA/rgb" "$FLIGHT_DATA/depth"

PX4_LOG="$LOG_DIR/px4_attempt_${ATTEMPT}.log"
cd ~/PX4-Autopilot
# Use setsid to create a new process group; save the group leader PID
setsid bash -c '
    source /opt/ros/jazzy/setup.bash
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=planetary_landing make px4_sitl gz_x500_depth_mono 2>&1
' >> "$PX4_LOG" 2>&1 &
PX4_PID=$!
echo "$PX4_PID" > /tmp/midterm_px4.pid
echo "PX4 PID: $PX4_PID, log: $PX4_LOG" | tee -a "$LOG"

# Wait for "Startup script returned successfully" — up to 3 minutes
echo "Waiting for PX4 startup..." | tee -a "$LOG"
STARTUP_TIMEOUT=180
STARTUP_SEEN=false
for i in $(seq 1 $STARTUP_TIMEOUT); do
    if grep -q "Startup script returned successfully" "$PX4_LOG" 2>/dev/null; then
        echo "PX4 started after ${i}s" | tee -a "$LOG"
        STARTUP_SEEN=true
        break
    fi
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "ERROR: PX4 process died after ${i}s" | tee -a "$LOG"
        cat "$PX4_LOG" | tail -30 | tee -a "$LOG"
        exit 1
    fi
    sleep 1
done

if [ "$STARTUP_SEEN" = false ]; then
    echo "ERROR: PX4 never reported startup success after ${STARTUP_TIMEOUT}s" | tee -a "$LOG"
    tail -50 "$PX4_LOG" | tee -a "$LOG"
    exit 1
fi

# Wait additional 30s for physics clock to stabilize before starting DDS
echo "Physics stabilization wait (30s)..." | tee -a "$LOG"
sleep 30

# ── PHASE 2: Start QGroundControl ────────────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 2: Starting QGroundControl ===" | tee -a "$LOG"
setsid ~/QGroundControl.AppImage --no-sandbox >> "$LOG_DIR/qgc_attempt_${ATTEMPT}.log" 2>&1 &
QGC_PID=$!
echo "$QGC_PID" > /tmp/midterm_qgc.pid
echo "QGC PID: $QGC_PID" | tee -a "$LOG"
sleep 10

# ── PHASE 3: Start MicroXRCE-DDS Agent ───────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 3: Starting MicroXRCE-DDS Agent ===" | tee -a "$LOG"
DDS_LOG="$LOG_DIR/dds_attempt_${ATTEMPT}.log"
setsid MicroXRCEAgent udp4 -p 8888 >> "$DDS_LOG" 2>&1 &
DDS_PID=$!
echo "$DDS_PID" > /tmp/midterm_dds.pid
echo "DDS PID: $DDS_PID, log: $DDS_LOG" | tee -a "$LOG"

# Wait for DDS session established — up to 60s
echo "Waiting for DDS session..." | tee -a "$LOG"
DDS_TIMEOUT=60
DDS_SEEN=false
for i in $(seq 1 $DDS_TIMEOUT); do
    if grep -qiE "session established|create_session|ProxyClient" "$DDS_LOG" 2>/dev/null; then
        echo "DDS session established after ${i}s" | tee -a "$LOG"
        DDS_SEEN=true
        break
    fi
    sleep 1
done

if [ "$DDS_SEEN" = false ]; then
    echo "WARNING: DDS session not confirmed, proceeding anyway..." | tee -a "$LOG"
fi

# Wait for time sync to stabilize (15s)
echo "DDS time sync stabilization (15s)..." | tee -a "$LOG"
sleep 15

# Check if time sync is causing issues
if grep -c "time jump detected" "$PX4_LOG" 2>/dev/null | grep -qv "^0$"; then
    TIMEJUMP_COUNT=$(grep -c "time jump detected" "$PX4_LOG" 2>/dev/null || echo "0")
    echo "WARNING: $TIMEJUMP_COUNT time jump events detected in PX4 log" | tee -a "$LOG"
    # Check if it's still happening (new ones in last 10s)
    sleep 10
    TIMEJUMP_COUNT2=$(grep -c "time jump detected" "$PX4_LOG" 2>/dev/null || echo "0")
    if [ "$TIMEJUMP_COUNT2" -gt "$TIMEJUMP_COUNT" ]; then
        echo "ERROR: Time sync still cycling ($TIMEJUMP_COUNT -> $TIMEJUMP_COUNT2). Fix D may not have applied." | tee -a "$LOG"
        echo "Checking UXRCE_DDS_SYNCT value in PX4 log..." | tee -a "$LOG"
        grep "UXRCE_DDS_SYNCT\|DDS_SYNCT" "$PX4_LOG" | tee -a "$LOG"
    else
        echo "Time sync appears stable." | tee -a "$LOG"
    fi
fi

# ── PHASE 4: Start ROS2 Bridge ───────────────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 4: Starting ROS2 Bridge ===" | tee -a "$LOG"
BRIDGE_LOG="$LOG_DIR/bridge_attempt_${ATTEMPT}.log"
setsid bash -c '
    source /opt/ros/jazzy/setup.bash
    source '"$ROS2_WS"'/install/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
      /drone/down_mono@sensor_msgs/msg/Image[gz.msgs.Image \
      /drone/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image \
      /drone/front_depth@sensor_msgs/msg/Image[gz.msgs.Image \
      /drone/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
      --ros-args -r /mono_camera:=/drone/down_mono 2>&1
' >> "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > /tmp/midterm_bridge.pid
echo "Bridge PID: $BRIDGE_PID" | tee -a "$LOG"
sleep 5

# ── PHASE 5: Start Control Node ──────────────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 5: Starting Control Node ===" | tee -a "$LOG"
CTRL_LOG="$LOG_DIR/control_attempt_${ATTEMPT}.log"
setsid bash -c '
    source /opt/ros/jazzy/setup.bash
    source '"$ROS2_WS"'/install/setup.bash
    ros2 run midterm_project control_node \
      --ros-args \
      --params-file ~/ses598-space-robotics-and-ai-2026/midterm_project/config/mission_config.yaml 2>&1
' >> "$CTRL_LOG" 2>&1 &
CTRL_PID=$!
echo "$CTRL_PID" > /tmp/midterm_control.pid
echo "Control node PID: $CTRL_PID, log: $CTRL_LOG" | tee -a "$LOG"
sleep 5

# ── PHASE 6: Start Image Saver ────────────────────────────────────────────
echo "" | tee -a "$LOG"
echo "=== PHASE 6: Starting Image Saver ===" | tee -a "$LOG"
SAVER_LOG="$LOG_DIR/saver_attempt_${ATTEMPT}.log"
setsid bash -c '
    source /opt/ros/jazzy/setup.bash
    source '"$ROS2_WS"'/install/setup.bash
    ros2 run midterm_project image_saver \
      --ros-args \
      --params-file ~/ses598-space-robotics-and-ai-2026/midterm_project/config/mission_config.yaml \
      -p output_dir:='"$FLIGHT_DATA"' 2>&1
' >> "$SAVER_LOG" 2>&1 &
SAVER_PID=$!
echo "$SAVER_PID" > /tmp/midterm_saver.pid
echo "Image saver PID: $SAVER_PID, log: $SAVER_LOG" | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "=== ALL PROCESSES STARTED — MONITORING ===" | tee -a "$LOG"
echo "Monitoring for up to 30 minutes..." | tee -a "$LOG"

# ── MONITORING LOOP ───────────────────────────────────────────────────────
MISSION_TIMEOUT=1800  # 30 min
SUCCESS=false
FAILURE_REASON=""
START_MONITOR=$(date +%s)

while true; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_MONITOR))

    if [ $ELAPSED -ge $MISSION_TIMEOUT ]; then
        FAILURE_REASON="timeout after ${MISSION_TIMEOUT}s"
        break
    fi

    # Check success: survey complete
    if grep -q "Survey complete\|all.*waypoints reached" "$CTRL_LOG" 2>/dev/null; then
        SUCCESS=true
        echo "SUCCESS: Survey complete!" | tee -a "$LOG"
        break
    fi

    # Check frame count for partial success
    FRAME_COUNT=$(ls "$FLIGHT_DATA/rgb/" | wc -l 2>/dev/null || echo 0)

    # Check if poses are being written (odometry flowing)
    POSE_COUNT=$(python3 -c "
import json, os
f = os.path.expanduser('~/midterm_flight_data/poses.json')
try:
    d = json.load(open(f))
    print(len(d))
except:
    print(0)
" 2>/dev/null || echo 0)

    # Check for arming
    ARMED=$(grep -c "arming\|ARMED\|Offboard mode" "$CTRL_LOG" 2>/dev/null || echo 0)

    # Status update every 30s
    if [ $((ELAPSED % 30)) -eq 0 ]; then
        echo "[${ELAPSED}s] Frames: $FRAME_COUNT | Poses: $POSE_COUNT | Armed/Offboard: $ARMED" | tee -a "$LOG"
        # Show last control log line
        tail -1 "$CTRL_LOG" 2>/dev/null | tee -a "$LOG" || true
    fi

    # Success condition: 150+ new frames AND poses are being recorded
    if [ "$FRAME_COUNT" -gt 150 ] && [ "$POSE_COUNT" -gt 100 ]; then
        SUCCESS=true
        echo "SUCCESS: $FRAME_COUNT frames + $POSE_COUNT poses captured!" | tee -a "$LOG"
        break
    fi

    # Failure detection: repeated TIMEOUT after DDS connected + no arming in 10 min
    if [ $ELAPSED -gt 600 ] && [ "$ARMED" -eq 0 ]; then
        # Check for persistent timeout cycling
        TIMEOUT_COUNT=$(grep -c "TIMEOUT\|fail: TIMEOUT" "$PX4_LOG" 2>/dev/null || echo 0)
        if [ "$TIMEOUT_COUNT" -gt 50 ]; then
            FAILURE_REASON="PX4 sensor TIMEOUT cycling ($TIMEOUT_COUNT events), never armed after ${ELAPSED}s"
            break
        fi
    fi

    sleep 5
done

echo "" | tee -a "$LOG"
if [ "$SUCCESS" = true ]; then
    echo "=== MISSION SUCCESS ===" | tee -a "$LOG"
    FRAME_COUNT=$(ls "$FLIGHT_DATA/rgb/" | wc -l)
    echo "Final frame count: $FRAME_COUNT" | tee -a "$LOG"
    # Force image_saver to save poses on shutdown
    if [ -f /tmp/midterm_saver.pid ]; then
        kill -INT $(cat /tmp/midterm_saver.pid) 2>/dev/null || true
        sleep 3
    fi
else
    echo "=== MISSION FAILED: $FAILURE_REASON ===" | tee -a "$LOG"
    echo "Control log tail:" | tee -a "$LOG"
    tail -30 "$CTRL_LOG" 2>/dev/null | tee -a "$LOG" || true
    echo "PX4 log tail:" | tee -a "$LOG"
    tail -20 "$PX4_LOG" 2>/dev/null | tee -a "$LOG" || true
fi

echo "End time: $(date)" | tee -a "$LOG"
# cleanup called via trap
