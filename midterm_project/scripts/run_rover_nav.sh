#!/bin/bash
# run_rover_nav.sh — Launch rover navigation end-to-end
#
# Prerequisites (must already be running in other terminals):
#   Term 1: PX4 + Gazebo   (planetary_landing.launch.py)
#   Term 2: DDS agent       (MicroXRCEAgent)
#   Term 3: bridge + drone control (term3_bridge_control.sh)
#
# This script:
#   1. Launches rover_nav.launch.py (spawn rover + bridge + Nav2)
#   2. Waits for Nav2 to initialise (10 s)
#   3. Runs rover_navigator node  (waits for nav, sends goal, monitors)
#   4. On exit (success or failure): kills the launch process
#
# Optional arguments passed through to rover_navigator via --ros-args:
#   run_rover_nav.sh -p goal_x:=-8.0 -p goal_y:=5.0
#
# Costmap path (default: ~/midterm_reconstruction/costmap.yaml):
#   run_rover_nav.sh costmap_yaml:=/path/to/costmap.yaml

set -e

# ── Environment ───────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# ── Costmap check ─────────────────────────────────────────────────────────────
COSTMAP_YAML="${COSTMAP_YAML:-$HOME/midterm_reconstruction/costmap.yaml}"
if [ ! -f "$COSTMAP_YAML" ]; then
    echo "WARNING: costmap.yaml not found at $COSTMAP_YAML"
    echo "  Generate it first:"
    echo "  python3 ~/ses598-space-robotics-and-ai-2026/midterm_project/scripts/generate_costmap.py \\"
    echo "      ~/midterm_3dgs_export/point_cloud.ply"
    echo ""
    echo "  Proceeding without a map — Nav2 will use an empty global costmap."
    echo "  Set COSTMAP_YAML=/path/to/costmap.yaml to override."
fi

# ── Launch rover_nav (background) ────────────────────────────────────────────
echo "=== Launching rover_nav.launch.py ==="
LAUNCH_LOG="$HOME/midterm_results/rover_launch.log"
mkdir -p "$HOME/midterm_results"

ros2 launch midterm_project rover_nav.launch.py \
    "costmap_yaml:=$COSTMAP_YAML" \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID  (log: $LAUNCH_LOG)"

# Trap to kill the launch on exit (Ctrl-C, success, or failure)
cleanup() {
    echo ""
    echo "=== Shutting down ==="
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "  Killing launch group (PID $LAUNCH_PID)…"
        # Kill the whole process group so child processes (Nav2, bridge, RSP) die too
        kill -- -"$LAUNCH_PID" 2>/dev/null || kill "$LAUNCH_PID" 2>/dev/null || true
    fi
    echo "  Done."
}
trap cleanup EXIT

# ── Wait for Nav2 init ────────────────────────────────────────────────────────
echo ""
echo "=== Waiting 10s for Nav2 to initialise ==="
for i in $(seq 10 -1 1); do
    printf "\r  %2ds remaining…" "$i"
    sleep 1
done
echo ""

# Verify the launch is still alive
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: launch process died. Check $LAUNCH_LOG"
    cat "$LAUNCH_LOG" | tail -30
    exit 1
fi

# ── Run navigator ─────────────────────────────────────────────────────────────
echo ""
echo "=== Starting rover navigator ==="
echo "  Start : (8.0, 0.0)"
echo "  Goal  : (-5.0, -5.0)"
echo "  Results will be saved to ~/midterm_results/rover_path.json"
echo ""

# Forward any extra CLI arguments as ROS args
EXTRA_ARGS=("$@")
if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    ros2 run midterm_project rover_navigator \
        --ros-args "${EXTRA_ARGS[@]}"
else
    ros2 run midterm_project rover_navigator
fi

NAV_EXIT=$?

echo ""
if [ "$NAV_EXIT" -eq 0 ]; then
    echo "=== Navigation SUCCEEDED ==="
else
    echo "=== Navigation FAILED (exit code $NAV_EXIT) ==="
fi

echo "  rover_path.json : $HOME/midterm_results/rover_path.json"
echo "  Launch log      : $LAUNCH_LOG"

exit $NAV_EXIT
