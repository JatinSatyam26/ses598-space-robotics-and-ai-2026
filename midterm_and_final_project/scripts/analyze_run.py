#!/usr/bin/env python3
"""
Analyze a Mars Scout Mission run: reads terminal logs + extracts video frames,
sends to Claude API for a concise explanatory brief, and outputs success/fail
criteria with specific improvement suggestions.

Usage:
  python3 analyze_run.py <video_path> <run_number> [output_dir]

Outputs:
  <output_dir>/run_<N>_brief.txt  — human-readable analysis
  <output_dir>/run_<N>_status.json — machine-readable pass/fail flags
"""

import sys
import os
import json
import base64
import subprocess
import re
import tempfile
from datetime import datetime

# Try importing Anthropic SDK
try:
    import anthropic
except ImportError:
    print("ERROR: anthropic SDK not found. Install with: pip install anthropic")
    sys.exit(1)


# ── Success criteria ─────────────────────────────────────────────────────────
SUCCESS_CRITERIA = {
    "drone_surveyed_all_waypoints": "Drone completed all survey waypoints",
    "drone_landed": "Drone landed successfully (LANDED message)",
    "grid_meaningful": "Occupancy grid had > 15 occupied cells",
    "rover_avoided_obstacles": "Rover showed obstacle avoidance behavior",
    "rover_arrived": "Rover arrived at habitat dome",
}


def read_log(path):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def extract_frames(video_path, num_frames=8):
    """Extract N evenly-spaced frames from video as base64 JPEG strings."""
    if not os.path.exists(video_path):
        return []

    frames = []
    with tempfile.TemporaryDirectory() as tmpdir:
        # Get video duration
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "default=noprint_wrappers=1:nokey=1", video_path],
            capture_output=True, text=True)
        try:
            duration = float(probe.stdout.strip())
        except Exception:
            duration = 60.0

        interval = duration / (num_frames + 1)
        for i in range(1, num_frames + 1):
            ts = interval * i
            frame_path = os.path.join(tmpdir, f"frame_{i:03d}.jpg")
            subprocess.run(
                ["ffmpeg", "-y", "-ss", str(ts), "-i", video_path,
                 "-vframes", "1", "-q:v", "5", "-vf", "scale=960:-1",
                 frame_path],
                capture_output=True)
            if os.path.exists(frame_path):
                with open(frame_path, "rb") as f:
                    frames.append(base64.standard_b64encode(f.read()).decode())
    return frames


def analyze_logs(flight_log, rover_log):
    """Parse log files and return status flags."""
    status = {k: False for k in SUCCESS_CRITERIA}
    notes = []

    # Drone survey
    wps = len(re.findall(r'Waypoint \d+/\d+ reached', flight_log))
    total_match = re.search(r'Waypoint \d+/(\d+) reached', flight_log)
    total_wps = int(total_match.group(1)) if total_match else 10
    if wps >= total_wps:
        status["drone_surveyed_all_waypoints"] = True
    else:
        notes.append(f"Drone only completed {wps}/{total_wps} waypoints")

    # Drone landing
    if "LANDED" in flight_log:
        status["drone_landed"] = True
    elif "LAND" in flight_log:
        notes.append("Drone entered LAND state but didn't confirm touch-down")
    else:
        notes.append("Drone did not attempt landing")

    # Occupancy grid
    grid_match = re.search(r'OccupancyGrid.*?(\d+) occupied cells', flight_log)
    if grid_match:
        cells = int(grid_match.group(1))
        if cells > 15:
            status["grid_meaningful"] = True
        else:
            notes.append(f"Occupancy grid had only {cells} occupied cells (need >15)")
    else:
        notes.append("No occupancy grid published in logs")

    # Rover obstacle avoidance — accept both reactive (LiDAR) and proactive (A*) avoidance
    lidar_avoidance = len(re.findall(r'Obstacle at|avoiding|LOW CLEARANCE', rover_log)) > 0
    astar_avoidance = bool(re.search(r'Goal in obstacle zone', rover_log))
    path_match = re.search(r'Path planned: (\d+) waypoints', rover_log)
    nontrivial_path = bool(path_match and int(path_match.group(1)) > 10)
    if lidar_avoidance or astar_avoidance or nontrivial_path:
        status["rover_avoided_obstacles"] = True
        methods = []
        if lidar_avoidance:
            methods.append("LiDAR reactive")
        if astar_avoidance:
            methods.append("A* reroute")
        if nontrivial_path:
            methods.append(f"non-trivial path ({path_match.group(1)} waypoints)")
        notes.append(f"Avoidance via: {', '.join(methods)}")

    # Rover arrival
    if "ARRIVED" in rover_log:
        status["rover_arrived"] = True
    else:
        wp_match = re.search(r'Waypoint (\d+)/(\d+) reached', rover_log)
        if wp_match:
            notes.append(
                f"Rover reached WP {wp_match.group(1)}/{wp_match.group(2)} but didn't arrive")

    return status, notes


def call_claude(flight_log, rover_log, frames, run_number, status, notes):
    """Send logs and frames to Claude for analysis."""
    client = anthropic.Anthropic()

    # Truncate logs to avoid token limits
    flight_excerpt = flight_log[-4000:] if len(flight_log) > 4000 else flight_log
    rover_excerpt = rover_log[-3000:] if len(rover_log) > 3000 else rover_log

    status_str = "\n".join(
        f"  {'PASS' if v else 'FAIL'}: {SUCCESS_CRITERIA[k]}"
        for k, v in status.items())

    notes_str = "\n".join(f"  - {n}" for n in notes) if notes else "  (none)"

    system_prompt = """You are analyzing a ROS2 + Gazebo Harmonic robotics simulation run.
The scenario: a drone (MulticopterVelocityControl quadcopter) surveys Mars terrain at 2m AGL,
builds an occupancy grid from its rangefinder, publishes it to a 6-wheel rover, which then
uses A* pathfinding to navigate around rocks to a habitat dome 15m away.
Write a concise explanatory brief (under 300 words) covering: what happened, what worked,
what failed, and the single most important fix for the next run."""

    # Build content blocks
    content = []

    # Add frames if available
    for i, frame_b64 in enumerate(frames[:6]):  # max 6 frames
        content.append({
            "type": "image",
            "source": {"type": "base64", "media_type": "image/jpeg", "data": frame_b64}
        })

    # Add log text
    content.append({
        "type": "text",
        "text": f"""=== RUN {run_number} ANALYSIS REQUEST ===

SUCCESS CRITERIA STATUS:
{status_str}

ADDITIONAL NOTES:
{notes_str}

=== DRONE (smart_flight_node) LOG (last 4000 chars) ===
{flight_excerpt}

=== ROVER (smart_rover_node) LOG (last 3000 chars) ===
{rover_excerpt}

Please write the concise explanatory brief now."""
    })

    response = client.messages.create(
        model="claude-sonnet-4-6",
        max_tokens=600,
        system=system_prompt,
        messages=[{"role": "user", "content": content}]
    )

    return response.content[0].text


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <video_path> <run_number> [output_dir]")
        sys.exit(1)

    video_path = sys.argv[1]
    run_number = int(sys.argv[2])
    output_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/midterm_results")
    os.makedirs(output_dir, exist_ok=True)

    print(f"[analyze_run] Run #{run_number} — reading logs...")
    flight_log = read_log("/tmp/smart_flight.log")
    rover_log = read_log("/tmp/smart_rover.log")

    print(f"[analyze_run] Parsing log metrics...")
    status, notes = analyze_logs(flight_log, rover_log)

    passed = sum(1 for v in status.values() if v)
    total = len(status)
    print(f"[analyze_run] Criteria: {passed}/{total} passed")

    print(f"[analyze_run] Extracting video frames from {video_path}...")
    frames = extract_frames(video_path, num_frames=8)
    print(f"[analyze_run] Got {len(frames)} frames")

    print(f"[analyze_run] Calling Claude API for analysis...")
    try:
        brief = call_claude(flight_log, rover_log, frames, run_number, status, notes)
    except Exception as e:
        brief = f"Claude API call failed: {e}\n\nLog-based status:\n"
        for k, v in status.items():
            brief += f"  {'PASS' if v else 'FAIL'}: {SUCCESS_CRITERIA[k]}\n"
        if notes:
            brief += "\nNotes:\n" + "\n".join(f"  - {n}" for n in notes)

    # Save outputs
    brief_path = os.path.join(output_dir, f"run_{run_number:03d}_brief.txt")
    status_path = os.path.join(output_dir, f"run_{run_number:03d}_status.json")

    with open(brief_path, "w") as f:
        f.write(f"=== RUN {run_number} BRIEF — {datetime.now().isoformat()} ===\n\n")
        f.write(brief)
        f.write("\n\n=== CRITERIA ===\n")
        for k, v in status.items():
            f.write(f"  {'PASS' if v else 'FAIL'}: {SUCCESS_CRITERIA[k]}\n")
        if notes:
            f.write("\n=== NOTES ===\n")
            for n in notes:
                f.write(f"  - {n}\n")

    status_data = {
        "run": run_number,
        "timestamp": datetime.now().isoformat(),
        "video": video_path,
        "passed": passed,
        "total": total,
        "all_passed": passed == total,
        "criteria": status,
        "notes": notes,
        "brief": brief,
    }
    with open(status_path, "w") as f:
        json.dump(status_data, f, indent=2)

    print(f"\n{'='*60}")
    print(f"RUN {run_number} BRIEF:")
    print('='*60)
    print(brief)
    print('='*60)
    print(f"\nSaved to: {brief_path}")
    print(f"Status:   {status_path}")
    print(f"Result:   {'ALL PASS — MISSION SUCCESSFUL' if passed == total else f'{passed}/{total} criteria met'}")

    # Exit 0 if all criteria passed (loop termination signal)
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
