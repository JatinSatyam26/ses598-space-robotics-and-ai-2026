#!/bin/bash
# =============================================================================
# run_full_pipeline.sh — SES 598 Midterm: End-to-End Pipeline
#
# Phases (can be run individually with --phase <A|B|C|D>):
#   A  Drone survey   — PX4+Gazebo, bridge, control_node, image_saver
#   B  Reconstruction — COLMAP → 3DGS → point cloud → costmap
#   C  Navigation     — Standalone Gazebo + rover + Nav2
#   D  Evaluation     — metrics, visualisations, ablation
#
# Usage:
#   ./run_full_pipeline.sh             # full pipeline A→D
#   ./run_full_pipeline.sh --phase B   # single phase
#   ./run_full_pipeline.sh --phase C --costmap ~/my/costmap.yaml
#
# Environment overrides:
#   COSTMAP_YAML          override costmap path used in Phase C
#   SURVEY_TIMEOUT        seconds to wait for drone survey (default: 1200)
#   NAV_TIMEOUT           seconds to wait for rover mission (default: 600)
#   SKIP_3DGS             set to 1 to skip nerfstudio training (use existing)
#   USE_OPENSPLAT         set to 1 to use OpenSplat instead of nerfstudio
# =============================================================================

set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
PROJECT_DIR="$HOME/ses598-space-robotics-and-ai-2026/midterm_project"
SCRIPTS_DIR="$PROJECT_DIR/scripts"
LOG_DIR="$HOME/midterm_logs"
DATA_DIR="$HOME/midterm_flight_data"
RECON_DIR="$HOME/midterm_reconstruction"
GS_OUTPUT="$HOME/midterm_3dgs_output"
GS_EXPORT="$HOME/midterm_3dgs_export"
RESULTS_DIR="$HOME/midterm_results"
COSTMAP_YAML="${COSTMAP_YAML:-$RECON_DIR/costmap.yaml}"

SURVEY_TIMEOUT="${SURVEY_TIMEOUT:-1200}"
NAV_TIMEOUT="${NAV_TIMEOUT:-600}"
SKIP_3DGS="${SKIP_3DGS:-0}"
USE_OPENSPLAT="${USE_OPENSPLAT:-0}"

# ── Colours ───────────────────────────────────────────────────────────────────
C_RESET='\033[0m'
C_BOLD='\033[1m'
C_CYAN='\033[0;36m'
C_GREEN='\033[0;32m'
C_YELLOW='\033[1;33m'
C_RED='\033[0;31m'

log()  { echo -e "${C_CYAN}[$(date +%H:%M:%S)]${C_RESET} $*"; }
ok()   { echo -e "${C_GREEN}[$(date +%H:%M:%S)] ✓ $*${C_RESET}"; }
warn() { echo -e "${C_YELLOW}[$(date +%H:%M:%S)] ⚠ $*${C_RESET}"; }
die()  { echo -e "${C_RED}[$(date +%H:%M:%S)] ✗ $*${C_RESET}"; exit 1; }

phase_banner() {
    local label="$1" title="$2"
    echo ""
    echo -e "${C_BOLD}${C_CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    printf  "║  PHASE %-54s║\n" "$label: $title"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${C_RESET}"
}

phase_done() {
    echo -e "${C_GREEN}${C_BOLD}"
    echo "  ──── Phase $1 complete ────"
    echo -e "${C_RESET}"
}

# ── Parse arguments ───────────────────────────────────────────────────────────
RUN_PHASE="ALL"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --phase) RUN_PHASE="${2^^}"; shift 2 ;;
        --costmap) COSTMAP_YAML="$2"; shift 2 ;;
        --help|-h)
            grep '^#' "$0" | head -20 | sed 's/^# \?//'
            exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

# ── Setup ─────────────────────────────────────────────────────────────────────
mkdir -p "$LOG_DIR" "$RESULTS_DIR"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# Track PIDs for cleanup
PIDS=()
cleanup() {
    if [[ ${#PIDS[@]} -gt 0 ]]; then
        warn "Cleaning up background processes: ${PIDS[*]}"
        for pid in "${PIDS[@]}"; do
            kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
        done
    fi
}
trap cleanup EXIT

# ── Helpers ───────────────────────────────────────────────────────────────────

# Start a background command, log to file, register PID
bg_run() {
    local name="$1"; shift
    local logfile="$LOG_DIR/${name}.log"
    log "Starting $name → $logfile"
    setsid bash -c "$*" > "$logfile" 2>&1 &
    local pid=$!
    PIDS+=("$pid")
    echo "$pid" > "$LOG_DIR/${name}.pid"
    echo "$pid"
}

# Wait for a string to appear in a log file (with timeout)
wait_for_log() {
    local logfile="$1" pattern="$2" timeout_s="${3:-120}" label="${4:-$pattern}"
    log "Waiting for '$label' in $(basename "$logfile") (timeout: ${timeout_s}s)…"
    local t=0
    while ! grep -q "$pattern" "$logfile" 2>/dev/null; do
        sleep 2; t=$((t+2))
        if [[ $t -ge $timeout_s ]]; then
            warn "Tail of $logfile:"
            tail -20 "$logfile" 2>/dev/null || true
            die "Timeout after ${timeout_s}s waiting for: $label"
        fi
        if [[ $((t % 30)) -eq 0 ]]; then
            log "  … still waiting ($t/${timeout_s}s)"
        fi
    done
    ok "Detected: $label"
}

kill_phase() {
    local name="$1"
    local pidfile="$LOG_DIR/${name}.pid"
    if [[ -f "$pidfile" ]]; then
        local pid; pid=$(cat "$pidfile")
        kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
        rm -f "$pidfile"
    fi
}

# ══════════════════════════════════════════════════════════════════════════════
# PHASE A — DRONE SURVEY
# ══════════════════════════════════════════════════════════════════════════════
run_phase_a() {
    phase_banner "A" "Drone Survey"

    # Pre-flight check
    if [[ ! -d "$HOME/PX4-Autopilot" ]]; then
        die "PX4-Autopilot not found at ~/PX4-Autopilot"
    fi
    if ! command -v MicroXRCEAgent &>/dev/null; then
        die "MicroXRCEAgent not found. Install Micro-XRCE-DDS-Agent."
    fi

    mkdir -p "$DATA_DIR/rgb" "$DATA_DIR/depth"

    # ── Step A1: PX4 + Gazebo ────────────────────────────────────────────────
    log "A1: Launching PX4 + Gazebo (planetary_landing world)…"
    PX4_PID=$(bg_run "px4_gazebo" \
        "cd ~/PX4-Autopilot && \
         PX4_GZ_WORLD=planetary_landing \
         make px4_sitl gz_x500_depth_mono 2>&1")

    log "Waiting 30s for Gazebo + drone to spawn…"
    sleep 30
    wait_for_log "$LOG_DIR/px4_gazebo.log" "Ready for takeoff" 120 "PX4 ready"

    # ── Step A2: QGC + DDS agent ─────────────────────────────────────────────
    log "A2: Starting QGroundControl + MicroXRCE-DDS Agent…"
    QGC_PID=$(bg_run "qgc" \
        "$HOME/QGroundControl.AppImage --no-sandbox 2>&1")
    sleep 5

    DDS_PID=$(bg_run "dds_agent" \
        "MicroXRCEAgent udp4 -p 8888 2>&1")
    wait_for_log "$LOG_DIR/dds_agent.log" "RTPS" 30 "DDS agent running"

    # ── Step A3: ROS2 Bridge + control_node ──────────────────────────────────
    log "A3: Launching ROS2 bridge + control_node…"
    sleep 3
    BRIDGE_PID=$(bg_run "bridge_control" \
        "source /opt/ros/jazzy/setup.bash && \
         source ~/ros2_ws/install/setup.bash && \
         ros2 run ros_gz_bridge parameter_bridge \
           /drone/down_mono@sensor_msgs/msg/Image[gz.msgs.Image \
           /drone/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image \
           /drone/front_depth@sensor_msgs/msg/Image[gz.msgs.Image \
           /drone/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
           --ros-args -r /mono_camera:=/drone/down_mono 2>&1")
    sleep 5

    CONTROL_PID=$(bg_run "control_node" \
        "source /opt/ros/jazzy/setup.bash && \
         source ~/ros2_ws/install/setup.bash && \
         ros2 run midterm_project control_node \
           --ros-args \
           --params-file $PROJECT_DIR/config/mission_config.yaml 2>&1")

    # ── Step A4: image_saver ─────────────────────────────────────────────────
    log "A4: Starting image_saver…"
    IMAGE_PID=$(bg_run "image_saver" \
        "source /opt/ros/jazzy/setup.bash && \
         source ~/ros2_ws/install/setup.bash && \
         ros2 run midterm_project image_saver \
           --ros-args \
           -p output_dir:=$DATA_DIR \
           -p save_every_nth:=5 2>&1")

    # ── Wait for survey ───────────────────────────────────────────────────────
    log "Waiting for drone survey to complete (timeout: ${SURVEY_TIMEOUT}s)…"
    wait_for_log "$LOG_DIR/control_node.log" "Mission DONE\|STATE: DONE\|DONE.*total time" \
        "$SURVEY_TIMEOUT" "Survey complete"

    # Brief extra wait for final images to flush
    log "Survey done. Waiting 5s for image_saver flush…"
    sleep 5

    # ── Teardown A ────────────────────────────────────────────────────────────
    log "Shutting down drone processes…"
    for name in image_saver control_node bridge_control dds_agent qgc px4_gazebo; do
        kill_phase "$name"
    done
    PIDS=()

    # Count captured images
    local n_imgs; n_imgs=$(ls "$DATA_DIR/rgb/"*.png 2>/dev/null | wc -l || echo 0)
    ok "Phase A complete — $n_imgs images saved to $DATA_DIR/rgb/"
    phase_done "A"
}

# ══════════════════════════════════════════════════════════════════════════════
# PHASE B — OFFLINE RECONSTRUCTION
# ══════════════════════════════════════════════════════════════════════════════
run_phase_b() {
    phase_banner "B" "3D Reconstruction"

    local n_imgs; n_imgs=$(ls "$DATA_DIR/rgb/"*.png 2>/dev/null | wc -l || echo 0)
    if [[ $n_imgs -lt 10 ]]; then
        die "Only $n_imgs images in $DATA_DIR/rgb/ — run Phase A first."
    fi
    log "Found $n_imgs survey images in $DATA_DIR/rgb/"

    # ── Step B1: COLMAP sparse reconstruction ────────────────────────────────
    log "B1: Running COLMAP sparse reconstruction…"
    if [[ -d "$RECON_DIR/sparse/0" ]] && ls "$RECON_DIR/sparse/0/cameras."* &>/dev/null; then
        warn "COLMAP output exists — skipping. Delete $RECON_DIR/sparse/ to re-run."
    else
        bash "$SCRIPTS_DIR/run_colmap.sh" "$DATA_DIR/rgb" \
            2>&1 | tee "$LOG_DIR/colmap.log"
        [[ -d "$RECON_DIR/sparse/0" ]] || \
            die "COLMAP failed — check $LOG_DIR/colmap.log"
    fi

    # Fallback: build sparse model directly from known poses
    if ! ls "$RECON_DIR/sparse/0/cameras."* &>/dev/null 2>&1; then
        warn "COLMAP sparse/0 empty — using pose-based fallback…"
        python3 "$SCRIPTS_DIR/create_colmap_from_poses.py" \
            2>&1 | tee "$LOG_DIR/colmap_from_poses.log"
    fi
    ok "B1: COLMAP complete"

    # ── Step B2: 3DGS training ────────────────────────────────────────────────
    if [[ "$SKIP_3DGS" == "1" ]]; then
        warn "B2: Skipping 3DGS (SKIP_3DGS=1)"
    elif [[ "$USE_OPENSPLAT" == "1" ]]; then
        log "B2: Training 3DGS with OpenSplat…"
        if [[ ! -x "$HOME/OpenSplat/build/opensplat" ]]; then
            die "OpenSplat binary not found at ~/OpenSplat/build/opensplat"
        fi
        mkdir -p "$GS_EXPORT"
        "$HOME/OpenSplat/build/opensplat" \
            "$RECON_DIR" \
            -o "$GS_EXPORT/splat.ply" \
            -n 15000 \
            2>&1 | tee "$LOG_DIR/opensplat.log"
        ok "B2: OpenSplat complete → $GS_EXPORT/splat.ply"
    else
        log "B2: Training 3DGS with nerfstudio splatfacto…"
        if ! command -v ns-train &>/dev/null; then
            die "ns-train not found. Run: pip install nerfstudio --break-system-packages"
        fi
        bash "$SCRIPTS_DIR/train_3dgs.sh" "$RECON_DIR" \
            2>&1 | tee "$LOG_DIR/train_3dgs.log"
        ok "B2: nerfstudio training complete"

        log "B3: Exporting dense point cloud…"
        bash "$SCRIPTS_DIR/export_pointcloud.sh" \
            2>&1 | tee "$LOG_DIR/export_pointcloud.log"
        ok "B3: Point cloud exported to $GS_EXPORT/"
    fi

    # ── Step B4: Traversability costmap ──────────────────────────────────────
    log "B4: Generating traversability costmap…"
    local ply_path
    if   [[ -f "$GS_EXPORT/point_cloud.ply" ]]; then ply_path="$GS_EXPORT/point_cloud.ply"
    elif [[ -f "$GS_EXPORT/splat.ply" ]];       then ply_path="$GS_EXPORT/splat.ply"
    else die "No point cloud found in $GS_EXPORT/. Check Phase B output."; fi

    python3 "$SCRIPTS_DIR/generate_costmap.py" \
        "$ply_path" \
        --output-dir "$RECON_DIR" \
        --resolution 0.1 \
        --inflation-radius 0.2 \
        2>&1 | tee "$LOG_DIR/costmap.log"

    COSTMAP_YAML="$RECON_DIR/costmap.yaml"
    [[ -f "$COSTMAP_YAML" ]] || die "costmap.yaml not generated — check $LOG_DIR/costmap.log"
    ok "B4: Costmap saved to $RECON_DIR/"
    phase_done "B"
}

# ══════════════════════════════════════════════════════════════════════════════
# PHASE C — ROVER NAVIGATION
# ══════════════════════════════════════════════════════════════════════════════
run_phase_c() {
    phase_banner "C" "Rover Navigation"

    if [[ ! -f "$COSTMAP_YAML" ]]; then
        warn "Costmap not found at $COSTMAP_YAML"
        warn "Nav2 will use an empty global costmap. Run Phase B first."
    fi

    # ── Step C1: Standalone Gazebo ────────────────────────────────────────────
    log "C1: Launching standalone Gazebo (no PX4 drone)…"
    GAZEBO_PID=$(bg_run "gazebo_rover" \
        "source /opt/ros/jazzy/setup.bash && \
         gz sim '$PROJECT_DIR/worlds/planetary_landing.sdf' -r 2>&1")

    log "Waiting 20s for Gazebo to load world…"
    sleep 20

    # ── Step C2: Spawn rover + bridge + Nav2 + navigator ─────────────────────
    log "C2: Launching rover nav stack (spawn + bridge + Nav2)…"
    COSTMAP_YAML="$COSTMAP_YAML" LAUNCH_LOG="$LOG_DIR/rover_launch.log" \
    bash "$SCRIPTS_DIR/run_rover_nav.sh" \
        2>&1 | tee "$LOG_DIR/rover_nav.log" &
    NAV_PID=$!
    PIDS+=("$NAV_PID")

    log "Waiting for rover mission to finish (timeout: ${NAV_TIMEOUT}s)…"
    local t=0
    while kill -0 "$NAV_PID" 2>/dev/null; do
        sleep 5; t=$((t+5))
        if [[ $t -ge $NAV_TIMEOUT ]]; then
            warn "Navigation timed out after ${NAV_TIMEOUT}s — killing."
            kill "$NAV_PID" 2>/dev/null || true
            break
        fi
        if [[ $((t % 30)) -eq 0 ]]; then
            log "  Navigation running… ($t/${NAV_TIMEOUT}s)"
        fi
    done
    wait "$NAV_PID" 2>/dev/null || true

    # ── Teardown C ────────────────────────────────────────────────────────────
    kill_phase "gazebo_rover"
    PIDS=()

    if [[ -f "$RESULTS_DIR/rover_path.json" ]]; then
        local success; success=$(python3 -c \
            "import json; d=json.load(open('$RESULTS_DIR/rover_path.json')); \
             print(d.get('result','UNKNOWN'))" 2>/dev/null || echo "UNKNOWN")
        ok "Phase C complete — rover result: $success"
    else
        warn "rover_path.json not found — navigation may have failed."
    fi
    phase_done "C"
}

# ══════════════════════════════════════════════════════════════════════════════
# PHASE D — EVALUATION
# ══════════════════════════════════════════════════════════════════════════════
run_phase_d() {
    phase_banner "D" "Evaluation & Visualisation"

    # ── Step D1: Metrics ──────────────────────────────────────────────────────
    log "D1: Computing quantitative metrics…"
    local ply_path=""
    for p in "$GS_EXPORT/point_cloud.ply" "$GS_EXPORT/splat.ply"; do
        [[ -f "$p" ]] && { ply_path="$p"; break; }
    done

    python3 "$SCRIPTS_DIR/compute_metrics.py" \
        ${ply_path:+--ply "$ply_path"} \
        --costmap-pgm "$RECON_DIR/costmap.pgm" \
        --rover-path  "$RESULTS_DIR/rover_path.json" \
        --output      "$RESULTS_DIR/metrics.json" \
        2>&1 | tee "$LOG_DIR/metrics.log"
    ok "D1: Metrics → $RESULTS_DIR/metrics.json"

    # ── Step D2: Bird's-eye visualisation ─────────────────────────────────────
    log "D2: Generating bird's-eye result figure…"
    python3 "$SCRIPTS_DIR/visualize_results.py" \
        ${ply_path:+--ply "$ply_path"} \
        --costmap-pgm "$RECON_DIR/costmap.pgm" \
        --rover-path  "$RESULTS_DIR/rover_path.json" \
        --poses       "$DATA_DIR/poses.json" \
        --output      "$RESULTS_DIR/birdseye_result.png" \
        2>&1 | tee "$LOG_DIR/visualize.log"
    ok "D2: Figure → $RESULTS_DIR/birdseye_result.png"

    # ── Step D3: Ablation study ───────────────────────────────────────────────
    log "D3: Running uncertainty ablation study…"
    python3 "$SCRIPTS_DIR/run_ablation.py" \
        --costmap-pgm "$RECON_DIR/costmap.pgm" \
        --rover-path  "$RESULTS_DIR/rover_path.json" \
        --output-dir  "$RESULTS_DIR" \
        2>&1 | tee "$LOG_DIR/ablation.log"
    ok "D3: Ablation → $RESULTS_DIR/ablation_{results.json,figure.png}"

    # ── Summary ───────────────────────────────────────────────────────────────
    echo ""
    echo -e "${C_BOLD}${C_GREEN}═══════════════════════════════════════════════════${C_RESET}"
    echo -e "${C_BOLD}${C_GREEN}  ALL PHASES COMPLETE${C_RESET}"
    echo -e "${C_BOLD}${C_GREEN}═══════════════════════════════════════════════════${C_RESET}"
    echo ""
    echo "  Output files:"
    for f in \
        "$RESULTS_DIR/metrics.json" \
        "$RESULTS_DIR/birdseye_result.png" \
        "$RESULTS_DIR/ablation_results.json" \
        "$RESULTS_DIR/ablation_figure.png" \
        "$RESULTS_DIR/rover_path.json"; do
        if [[ -f "$f" ]]; then
            local size; size=$(du -sh "$f" | cut -f1)
            echo "    ✓  $(basename "$f")  ($size)"
        fi
    done
    echo ""
    phase_done "D"
}

# ══════════════════════════════════════════════════════════════════════════════
# DISPATCH
# ══════════════════════════════════════════════════════════════════════════════
case "$RUN_PHASE" in
    A)   run_phase_a ;;
    B)   run_phase_b ;;
    C)   run_phase_c ;;
    D)   run_phase_d ;;
    ALL)
        run_phase_a
        run_phase_b
        run_phase_c
        run_phase_d
        ;;
    *)   die "Unknown phase: $RUN_PHASE  (valid: A B C D ALL)" ;;
esac
