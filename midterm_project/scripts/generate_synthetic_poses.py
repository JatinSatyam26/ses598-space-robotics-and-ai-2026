#!/usr/bin/env python3
"""
Generate synthetic poses.json for the existing frames.
Uses the known lawnmower survey pattern from control_node.py.

Survey params:
  SURVEY_ALTITUDE_M = 5.0  (AGL)
  _SURVEY_OFFSETS (dx, dy) in NED:
    (-10, 0), (10, 0), (10, 3), (-10, 3), (-10, 6), (10, 6),
    (10, 9), (-10, 9), (-10, 12), (10, 12)

Home position assumed: (0, 0, 0) NED → survey at z = -5.0 NED
Quaternion: [w, x, y, z] with yaw based on flight direction.
"""

import json, math, os, glob, re

OUTPUT_PATH  = os.path.expanduser("~/midterm_flight_data/poses.json")
RGB_DIR      = os.path.expanduser("~/midterm_flight_data/rgb/")
DEPTH_DIR    = os.path.expanduser("~/midterm_flight_data/depth/")

# Mission params from control_node.py
HOME_NED      = [0.0, 0.0, 0.0]
SURVEY_ALT    = 5.0

SURVEY_OFFSETS = [
    (-10.0,  0.0),
    ( 10.0,  0.0),
    ( 10.0,  3.0),
    (-10.0,  3.0),
    (-10.0,  6.0),
    ( 10.0,  6.0),
    ( 10.0,  9.0),
    (-10.0,  9.0),
    (-10.0, 12.0),
    ( 10.0, 12.0),
]

z_survey = HOME_NED[2] - SURVEY_ALT  # -5.0 in NED
WAYPOINTS = [(HOME_NED[0] + dx, HOME_NED[1] + dy, z_survey)
             for dx, dy in SURVEY_OFFSETS]

def yaw_to_quat(yaw_rad):
    """NED yaw → quaternion [w, x, y, z] (level flight)."""
    half = yaw_rad / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]

def segment_yaw(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])

# Build path
segments = []
total_length = 0.0
for i in range(len(WAYPOINTS) - 1):
    a, b = WAYPOINTS[i], WAYPOINTS[i + 1]
    L = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
    segments.append({'a': a, 'b': b, 'length': L, 'yaw': segment_yaw(a, b)})
    total_length += L

print(f"Survey: {len(WAYPOINTS)} waypoints, total path {total_length:.1f}m")

# Collect RGB frames — sort numerically by the integer in the filename
rgb_files = glob.glob(os.path.join(RGB_DIR, "frame_*.png"))
def parse_frame_id(path):
    m = re.search(r'frame_0*(\d+)\.png$', os.path.basename(path))
    return int(m.group(1)) if m else -1

# Sort by numeric frame_id, deduplicate (keep shorter name = canonical)
frame_map = {}
for f in rgb_files:
    fid = parse_frame_id(f)
    if fid >= 0:
        # If duplicate, keep the one with shorter filename (e.g. frame_0000 over frame_00000)
        if fid not in frame_map or len(os.path.basename(f)) < len(os.path.basename(frame_map[fid])):
            frame_map[fid] = f

frames = [(fid, frame_map[fid]) for fid in sorted(frame_map.keys())]
n_frames = len(frames)
print(f"Unique frames: {n_frames}")

# Build depth file map similarly
depth_files = glob.glob(os.path.join(DEPTH_DIR, "frame_*.png"))
depth_map = {}
for f in depth_files:
    fid = parse_frame_id(f)
    if fid >= 0:
        if fid not in depth_map or len(os.path.basename(f)) < len(os.path.basename(depth_map[fid])):
            depth_map[fid] = f

print(f"Depth frames available: {len(depth_map)}")

def interpolate_path(t_total):
    """Return (pos, quat) at fractional distance t_total along the survey path."""
    target_dist = t_total * total_length
    cum = 0.0
    for seg in segments:
        if cum + seg['length'] >= target_dist or seg is segments[-1]:
            local_t = ((target_dist - cum) / seg['length']
                       if seg['length'] > 0 else 0.0)
            local_t = max(0.0, min(1.0, local_t))
            a, b = seg['a'], seg['b']
            pos = [
                a[0] + local_t * (b[0] - a[0]),
                a[1] + local_t * (b[1] - a[1]),
                a[2] + local_t * (b[2] - a[2]),
            ]
            return pos, yaw_to_quat(seg['yaw'])
        cum += seg['length']
    return list(WAYPOINTS[-1]), yaw_to_quat(segments[-1]['yaw'])

# Generate poses
poses = []
for i, (fid, rgb_path) in enumerate(frames):
    t = i / max(n_frames - 1, 1)
    pos, q = interpolate_path(t)

    depth_path = depth_map.get(fid, os.path.join(DEPTH_DIR, f"frame_{fid:04d}.png"))

    poses.append({
        'frame_id':  fid,
        'ros_frame': fid * 5,
        'rgb_path':  rgb_path,
        'depth_path': depth_path,
        'odom': {
            'position':   pos,
            'quaternion': q,
        }
    })

# Validate
xs = [p['odom']['position'][0] for p in poses]
ys = [p['odom']['position'][1] for p in poses]
print(f"Poses: {len(poses)}")
print(f"X range: {min(xs):.2f} to {max(xs):.2f}")
print(f"Y range: {min(ys):.2f} to {max(ys):.2f}")
print(f"Sample[0]:    {poses[0]['odom']['position']}")
print(f"Sample[500]:  {poses[500]['odom']['position']}")
print(f"Sample[1000]: {poses[1000]['odom']['position']}")
print(f"Sample[-1]:   {poses[-1]['odom']['position']}")

with open(OUTPUT_PATH, 'w') as f:
    json.dump(poses, f, indent=2)
print(f"\nWritten: {OUTPUT_PATH}")
