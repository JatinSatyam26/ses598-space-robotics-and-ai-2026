#!/bin/bash
# Full drone survey launch — one command, no interaction needed.
# Run after a fresh PC boot. Handles all timing and health checks automatically.

set -e

FLIGHT_DIR="$HOME/midterm_flight_data"
PX4_DIR="$HOME/PX4-Autopilot"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"

log() { echo "[$(date +%H:%M:%S)] $*"; }
die() { echo "FATAL: $*" >&2; exit 1; }

# ── Step 0: Clear old data ────────────────────────────────────────────────────
log "Clearing old flight data..."
rm -f "$FLIGHT_DIR/rgb/"* "$FLIGHT_DIR/depth/"* 2>/dev/null || true
echo '[]' > "$FLIGHT_DIR/poses.json"

# ── Step 1: Delete bson param store (airframe defaults must apply fresh) ──────
log "Deleting PX4 param store so airframe defaults apply..."
rm -f "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson"

# ── Step 2: Start DDS agent FIRST (before PX4) ───────────────────────────────
# Starting DDS before PX4 means the 80-topic handshake completes during PX4
# boot — not mid-flight — so it never spikes CPU while lockstep is running.
log "Starting DDS agent..."
gnome-terminal --title="DDS" -- bash -c \
    'MicroXRCEAgent udp4 -p 8888 2>&1 | tee /tmp/dds_log.txt; exec bash'
sleep 5

# ── Step 3: Start PX4 + Gazebo ───────────────────────────────────────────────
log "Launching PX4 + Gazebo (planetary_landing world)..."
gnome-terminal --title="PX4+Gazebo" -- bash -c \
    'cd ~/PX4-Autopilot \
     && source /opt/ros/jazzy/setup.bash \
     && source ~/ros2_ws/install/setup.bash \
     && PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" \
        PX4_GZ_WORLD=planetary_landing \
        make px4_sitl gz_x500_depth_mono 2>&1 | tee /tmp/px4_log.txt; exec bash'

# ── Step 4: Wait for PX4 startup + DDS session ───────────────────────────────
log "Waiting for PX4 startup and DDS session (up to 120s)..."
DEADLINE=$(( $(date +%s) + 120 ))
while [ $(date +%s) -lt $DEADLINE ]; do
    sleep 5
    STARTUP=$(grep -c "Startup script returned successfully" /tmp/px4_log.txt 2>/dev/null || echo 0)
    DDS_OK=$(grep -c "session established" /tmp/dds_log.txt 2>/dev/null || echo 0)
    ACCEL_FAIL=$(grep -c "Accel.*TIMEOUT\|Gyro.*TIMEOUT\|BARO.*TIMEOUT" /tmp/px4_log.txt 2>/dev/null || echo 0)
    log "  startup=$STARTUP  dds_session=$DDS_OK  sensor_timeouts=$ACCEL_FAIL"
    if [ "$STARTUP" -gt 0 ] && [ "$DDS_OK" -gt 0 ]; then
        log "PX4 started and DDS connected."
        break
    fi
done
grep -c "Startup script returned successfully" /tmp/px4_log.txt | grep -q "^0$" && \
    die "PX4 did not start. Check /tmp/px4_log.txt"

# ── Step 5: Start QGC (for GCS preflight check) ──────────────────────────────
log "Starting QGroundControl..."
gnome-terminal --title="QGC" -- bash -c \
    '~/QGroundControl.AppImage > /tmp/qgc_log.txt 2>&1; exec bash'

# ── Step 6: Wait for EKF to converge (system reaches STANDBY) ────────────────
log "Waiting for EKF to converge (up to 120s)..."
source /opt/ros/jazzy/setup.bash 2>/dev/null
DEADLINE=$(( $(date +%s) + 120 ))
EKF_OK=0
while [ $(date +%s) -lt $DEADLINE ]; do
    sleep 5
    # Check via MAVLink heartbeat
    STATUS=$(python3 -c "
from pymavlink import mavutil; import sys
try:
    m = mavutil.mavlink_connection('udp:localhost:14550')
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=4)
    if hb: print(hb.system_status)
    else: print(-1)
except: print(-1)
" 2>/dev/null)
    # MAV_STATE_STANDBY=3, MAV_STATE_ACTIVE=4
    log "  MAVLink system_status=$STATUS (need 3=STANDBY or 4=ACTIVE)"
    if [ "$STATUS" = "3" ] || [ "$STATUS" = "4" ]; then
        EKF_OK=1
        log "EKF converged — system is ready to arm!"
        break
    fi
    # Also check preflight fails in log for progress
    FAILS=$(grep "Preflight Fail" /tmp/px4_log.txt 2>/dev/null | tail -5)
    [ -n "$FAILS" ] && log "  Recent preflight: $(echo "$FAILS" | tail -1)"
done
if [ "$EKF_OK" -eq 0 ]; then
    log "WARNING: EKF did not converge in 120s — proceeding anyway (may still arm)"
fi

# ── Step 7: Launch ROS2 bridge ────────────────────────────────────────────────
log "Starting ROS2 bridge..."
gnome-terminal --title="Bridge" -- bash -c \
    'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash \
     && ros2 run ros_gz_bridge parameter_bridge \
        /mono_camera@sensor_msgs/msg/Image@gz.msgs.Image \
        /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
        /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
        --ros-args \
        -r /mono_camera:=/drone/down_mono \
        -r /depth_camera:=/drone/front_depth \
        -r /camera_info:=/drone/front_depth/camera_info \
        -p use_sim_time:=true 2>&1 | tee /tmp/bridge_log.txt; exec bash'
sleep 5

# ── Step 8: Launch control node ───────────────────────────────────────────────
log "Starting control node (drone survey)..."
CFG=$(ros2 pkg prefix midterm_project 2>/dev/null)/share/midterm_project/config/mission_config.yaml
gnome-terminal --title="Control" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash \
     && ros2 run midterm_project control_node \
        --ros-args --params-file $CFG -p use_sim_time:=true \
        2>&1 | tee /tmp/control_log.txt; exec bash"
sleep 3

# ── Step 9: Launch image saver ────────────────────────────────────────────────
log "Starting image saver..."
gnome-terminal --title="ImageSaver" -- bash -c \
    'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash \
     && ros2 run midterm_project image_saver \
        2>&1 | tee /tmp/saver_log.txt; exec bash'

# ── Step 10: Monitor until survey completes ───────────────────────────────────
log "Survey running. Monitoring progress (Ctrl+C to stop monitoring, survey continues)..."
sleep 10
while true; do
    FRAMES=$(ls "$FLIGHT_DIR/rgb/" 2>/dev/null | wc -l)
    POSES=$(python3 -c "import json; d=json.load(open('$FLIGHT_DIR/poses.json')); print(len(d))" 2>/dev/null || echo "?")
    STATE=$(grep -oP "(?<=state: )\w+" /tmp/control_log.txt 2>/dev/null | tail -1 || echo "?")
    log "  frames=$FRAMES  poses=$POSES  state=$STATE"
    if grep -q "DONE\|survey complete\|RTL\|HOVER" /tmp/control_log.txt 2>/dev/null; then
        log "Survey complete!"
        break
    fi
    sleep 30
done
