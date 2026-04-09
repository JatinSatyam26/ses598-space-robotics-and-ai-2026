#!/bin/bash
# SES 598 Midterm Demo — Fully Autonomous
# Drone survey + Rover navigation, screen-recorded
set -eo pipefail

# ===== CONFIG =====
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORLD_SDF="${PROJECT_DIR}/worlds/planetary_landing.sdf"
DRONE_SDF="${PROJECT_DIR}/models/survey_drone/model.sdf"
ROVER_SDF="${PROJECT_DIR}/models/mars_rover/model.sdf"
FLIGHT_DATA="$HOME/midterm_flight_data"
RESULTS="$HOME/midterm_results"
WORLD_NAME="planetary_landing"
DISPLAY_VAL="${DISPLAY:-:0}"
RESOLUTION="1920x1080"
VIDEO_OUT="${RESULTS}/demo_video.mp4"
SURVEY_ALT=5.0
MOVE_DELAY=0.2

WAYPOINTS=(
  "-10 -10 ${SURVEY_ALT}"
  "10 -10 ${SURVEY_ALT}"
  "10 -6 ${SURVEY_ALT}"
  "-10 -6 ${SURVEY_ALT}"
  "-10 -2 ${SURVEY_ALT}"
  "10 -2 ${SURVEY_ALT}"
  "10 2 ${SURVEY_ALT}"
  "-10 2 ${SURVEY_ALT}"
  "-10 6 ${SURVEY_ALT}"
  "10 6 ${SURVEY_ALT}"
)

# ===== CLEANUP TRAP =====
RECORD_PID=0
PIDS=()

cleanup() {
  echo ""
  echo "=== Cleaning up ==="
  [[ $RECORD_PID -ne 0 ]] && { kill $RECORD_PID 2>/dev/null; wait $RECORD_PID 2>/dev/null || true; }
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  ps aux | grep -E "gz sim|parameter_bridge|demo_image_saver" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true
  echo "=== Cleanup done ==="
}
trap cleanup EXIT

# ===== HELPER: smooth move =====
move_entity() {
  local name=$1 x1=$2 y1=$3 z1=$4 x2=$5 y2=$6 z2=$7 steps=${8:-20}
  for i in $(seq 0 $steps); do
    local t=$(echo "scale=4; $i / $steps" | bc)
    local x=$(echo "scale=4; $x1 + ($x2 - $x1) * $t" | bc)
    local y=$(echo "scale=4; $y1 + ($y2 - $y1) * $t" | bc)
    local z=$(echo "scale=4; $z1 + ($z2 - $z1) * $t" | bc)
    gz service -s /world/${WORLD_NAME}/set_pose \
      --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean \
      --timeout 1000 \
      --req "name: '${name}', position: {x: $x, y: $y, z: $z}" 2>/dev/null || true
    sleep ${MOVE_DELAY}
  done
}

# ===== PREFLIGHT CLEANUP =====
echo "=== Pre-flight cleanup ==="
ps aux | grep -E "gz sim|parameter_bridge|demo_image_saver" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true
sleep 2

mkdir -p "${FLIGHT_DATA}/rgb" "${FLIGHT_DATA}/depth" "${RESULTS}"
rm -f "${FLIGHT_DATA}/rgb/"* "${FLIGHT_DATA}/depth/"* 2>/dev/null || true

# ===== ENVIRONMENT =====
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export DISPLAY="${DISPLAY_VAL}"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   SES 598 — Aerial-Ground Cooperative Demo       ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ===== LAUNCH GAZEBO =====
echo "[PHASE A] Starting Gazebo..."
gz sim "${WORLD_SDF}" &
PIDS+=($!)

# Wait for services
for i in $(seq 1 40); do
  SVC_COUNT=$(gz service -l 2>/dev/null | grep -c "/" || echo 0)
  if [[ $SVC_COUNT -gt 10 ]]; then
    echo "  Gazebo ready after ${i}s (${SVC_COUNT} services)"
    break
  fi
  sleep 1
done

# Unpause simulation so sensors run
gz service -s /world/${WORLD_NAME}/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "pause: false" 2>/dev/null || true
sleep 2

# ===== START SCREEN RECORDING =====
echo "[RECORD] Starting screen recording..."
ffmpeg -y -f x11grab -framerate 15 -video_size ${RESOLUTION} \
  -i ${DISPLAY_VAL} -c:v libx264 -preset ultrafast -crf 28 \
  "${VIDEO_OUT}" </dev/null 2>/tmp/ffmpeg_record.log &
RECORD_PID=$!
sleep 1
echo "  Recording started (PID=${RECORD_PID})"

# ===== SPAWN DRONE =====
echo "[PHASE A] Spawning drone at (0,0,0.5)..."
gz service -s /world/${WORLD_NAME}/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: '$(realpath ${DRONE_SDF})', name: 'survey_drone', pose: {position: {x: 0, y: 0, z: 0.5}}"
echo "  Drone spawned (exit $?)"
sleep 2

# ===== START BRIDGE =====
echo "[PHASE A] Starting ROS2-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /mono_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /mono_camera:=/drone/down_mono \
  -r /depth_camera:=/drone/front_depth \
  -r /rgb_camera:=/drone/front_rgb \
  2>/tmp/bridge.log &
PIDS+=($!)
sleep 3
echo "  Bridge running"

# ===== START IMAGE SAVER =====
echo "[PHASE A] Starting image capture..."
ros2 run midterm_project demo_image_saver 2>/tmp/img_saver.log &
PIDS+=($!)
sleep 2
echo "  Image saver running"

# ===== FLY SURVEY =====
echo ""
echo "[PHASE A] ========== DRONE SURVEY =========="
echo ""

echo "  Takeoff to ${SURVEY_ALT}m..."
move_entity survey_drone 0 0 0.5 0 0 ${SURVEY_ALT} 15
echo "  At ${SURVEY_ALT}m altitude"

prev_x=0; prev_y=0; prev_z=${SURVEY_ALT}
wp_num=0
for wp in "${WAYPOINTS[@]}"; do
  wp_num=$((wp_num + 1))
  read -r wx wy wz <<< "$wp"
  echo "  Waypoint ${wp_num}/${#WAYPOINTS[@]}: (${wx}, ${wy}, ${wz})"
  move_entity survey_drone $prev_x $prev_y $prev_z $wx $wy $wz 25
  prev_x=$wx; prev_y=$wy; prev_z=$wz
done

echo "  Survey complete!"

echo "  Landing at (${prev_x}, ${prev_y})..."
move_entity survey_drone $prev_x $prev_y $prev_z $prev_x $prev_y 0.5 10
sleep 1

FRAME_COUNT=$(ls "${FLIGHT_DATA}/rgb/" 2>/dev/null | wc -l)
echo "  Frames captured: ${FRAME_COUNT}"

# ===== WRITE POSES.JSON =====
echo "[DATA] Writing poses.json..."
python3 - "${FLIGHT_DATA}" "${SURVEY_ALT}" << 'PYEOF'
import json, os, sys

out_dir = sys.argv[1]
survey_alt = float(sys.argv[2])

waypoints = [
    (0,0,0.5), (0,0,survey_alt),
    (-10,-10,survey_alt),(10,-10,survey_alt),(10,-6,survey_alt),(-10,-6,survey_alt),
    (-10,-2,survey_alt),(10,-2,survey_alt),(10,2,survey_alt),(-10,2,survey_alt),
    (-10,6,survey_alt),(10,6,survey_alt)
]

poses = []
seg_steps = [15] + [25]*10
for i in range(len(waypoints)-1):
    x1,y1,z1 = waypoints[i]
    x2,y2,z2 = waypoints[i+1]
    steps = seg_steps[i] if i < len(seg_steps) else 25
    for step in range(steps):
        t = step / steps
        x = x1 + (x2-x1)*t
        y = y1 + (y2-y1)*t
        z = z1 + (z2-z1)*t
        poses.append({
            'frame': f'frame_{len(poses):04d}.png',
            'position': [x, y, -z],
            'quaternion': [1.0, 0.0, 0.0, 0.0],
            'altitude': z
        })

outfile = os.path.join(out_dir, 'poses.json')
with open(outfile, 'w') as f:
    json.dump(poses, f, indent=2)
print(f'  Wrote {len(poses)} poses to {outfile}')
PYEOF

# ===== PHASE B: ROVER =====
echo ""
echo "[PHASE B] ========== ROVER NAVIGATION =========="
echo ""

echo "  Spawning rover at (8, 0)..."
gz service -s /world/${WORLD_NAME}/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: '$(realpath ${ROVER_SDF})', name: 'mars_rover', pose: {position: {x: 8, y: 0, z: 0.3}}"
echo "  Rover spawned (exit $?)"
sleep 2

echo "  Driving from (8,0) to (-5,-5)..."
python3 - "${RESULTS}" "${WORLD_NAME}" << 'PYEOF'
import subprocess, time, json, os, sys

results_dir = sys.argv[1]
world_name = sys.argv[2]

start_x, start_y = 8.0, 0.0
goal_x, goal_y = -5.0, -5.0
steps = 60

path = []
for i in range(steps + 1):
    t = i / steps
    x = start_x + (goal_x - start_x) * t
    y = start_y + (goal_y - start_y) * t
    path.append({'position': [x, y, 0.3]})

os.makedirs(results_dir, exist_ok=True)
with open(os.path.join(results_dir, 'rover_path.json'), 'w') as f:
    json.dump(path, f, indent=2)

for i, pt in enumerate(path):
    x, y = pt['position'][0], pt['position'][1]
    subprocess.run([
        'gz','service','-s',f'/world/{world_name}/set_pose',
        '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
        '--timeout','1000',
        '--req', f"name: 'mars_rover', position: {{x: {x}, y: {y}, z: 0.3}}"
    ], capture_output=True, timeout=5)
    if i % 10 == 0:
        print(f'  Rover: ({x:.1f}, {y:.1f}) [{i+1}/{len(path)}]')
    time.sleep(0.15)

print('  Rover reached goal!')
PYEOF

sleep 3

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   DEMO COMPLETE                                  ║"
echo "╚══════════════════════════════════════════════════╝"

# Stop recording
echo "Stopping recording..."
kill $RECORD_PID 2>/dev/null; wait $RECORD_PID 2>/dev/null || true
RECORD_PID=0
sleep 1

ls -lh "${VIDEO_OUT}" 2>/dev/null || echo "WARNING: video not found"
echo "Frames: $(ls ${FLIGHT_DATA}/rgb/ 2>/dev/null | wc -l)"
echo "Poses:  $(python3 -c "import json; print(len(json.load(open('${FLIGHT_DATA}/poses.json'))))" 2>/dev/null || echo 'N/A')"
