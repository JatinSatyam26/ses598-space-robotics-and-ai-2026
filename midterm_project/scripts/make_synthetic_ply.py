#!/usr/bin/env python3
"""
Generate a synthetic terrain point cloud PLY from known camera poses + intrinsics.
Simulates what the downward camera would see projecting rays to the ground plane
and adding terrain height variation.

Output: ~/midterm_3dgs_export/synthetic_terrain.ply
"""
import json, math, struct, numpy as np
from pathlib import Path

POSES_PATH = Path.home() / "midterm_flight_data/poses.json"
CAM_PATH   = Path.home() / "midterm_flight_data/camera_info.json"
OUT_DIR    = Path.home() / "midterm_3dgs_export"
OUT_PLY    = OUT_DIR / "synthetic_terrain.ply"

OUT_DIR.mkdir(exist_ok=True)

cam = json.loads(CAM_PATH.read_text())
K = cam["K"]
fx, fy, cx, cy = K[0], K[4], K[2], K[5]
W, H = cam["width"], cam["height"]

poses = json.loads(POSES_PATH.read_text())
print(f"Loaded {len(poses)} poses")

rng = np.random.default_rng(42)

# Terrain model: bumpy ground at Z=0 NED (Z=0 is ground level in NED world)
def terrain_height(x, y):
    """Return terrain height (Z in NED frame, negative = up) with some bumps."""
    # Flat ground at 0 with small mounds
    z = 0.0
    # Rocky outcrops
    for cx_t, cy_t, r, h in [
        (-5, 3, 2.5, 0.8),  (3, 7, 2.0, 1.2),
        (8, 2, 1.5, 0.6),  (-7, 9, 3.0, 1.0),
        (1, 11, 1.8, 0.9),  (-3, 5, 2.2, 0.7),
        (6, 6, 1.2, 1.5),  (-8, 1, 1.8, 0.4),
    ]:
        d = math.sqrt((x-cx_t)**2 + (y-cy_t)**2)
        if d < r:
            # Height goes up (negative NED Z)
            z -= h * (1 - d/r)**2
    return z

# Sample points: for each camera pose, project a grid of pixels to ground
all_pts = []
# Sample every 20th pose to keep point count manageable
STEP = 20
GRID = 8  # 8x8 grid of pixels per image

for i, entry in enumerate(poses):
    if i % STEP != 0:
        continue
    odom = entry.get('odom')
    if not odom:
        continue

    pos = odom['position']  # NED: [x, y, z] (z is negative = 5m above ground)
    qw, qx, qy, qz = odom['quaternion']

    # Camera is pointing down (-Z direction in NED = +body_z)
    # For a level downward camera, the camera center is at pos[0,1,2]
    # and rays project straight down to z=0 with some offset based on pixel position
    # Camera altitude above ground
    alt = -pos[2]  # NED z is negative when above ground, so alt = -z

    # Project pixel grid to ground plane
    for pu in np.linspace(0, W-1, GRID):
        for pv in np.linspace(0, H-1, GRID):
            # Ray direction in camera frame (pinhole)
            ray_x_cam = (pu - cx) / fx
            ray_y_cam = (pv - cy) / fy
            ray_z_cam = 1.0  # optical axis

            # For downward camera (body-z = optical axis):
            # camera_x → NED_y (East), camera_y → -NED_x (-North), camera_z → NED_z (Down)
            # Ray in NED body frame
            ray_ned_x = -ray_y_cam  # -camera_y → North
            ray_ned_y =  ray_x_cam  # camera_x → East
            ray_ned_z =  ray_z_cam  # camera_z → Down

            # To world NED, rotate by body orientation (assuming level drone, q≈identity)
            # For synthetic data, just use the rotation from quaternion
            # Simplified: assume drone is level (q = [1,0,0,0])
            # Then ray in world NED = ray in body NED = (ray_ned_x, ray_ned_y, ray_ned_z)

            # Hit ground at NED z=terrain_height(x,y)
            # Ray: P = pos + t * ray_ned
            # pos_z + t * ray_ned_z = terrain_z
            # t = (terrain_z - pos_z) / ray_ned_z

            # Iterate to find terrain hit (terrain varies with x,y)
            # Use one-step approximation: t0 = alt / ray_ned_z
            t0 = alt / ray_ned_z  # approximate
            gx = pos[0] + t0 * ray_ned_x
            gy = pos[1] + t0 * ray_ned_y

            # Refine with actual terrain height
            th = terrain_height(gx, gy)
            t1 = (th - pos[2]) / ray_ned_z
            gx = pos[0] + t1 * ray_ned_x
            gy = pos[1] + t1 * ray_ned_y
            gz = th

            # Convert NED Z to Z-up (height above ground): h = -z_ned
            gz_up = -gz  # positive = above ground

            # Add small noise for realism
            gx += rng.uniform(-0.05, 0.05)
            gy += rng.uniform(-0.05, 0.05)
            gz_up = max(0.0, gz_up + rng.uniform(-0.02, 0.02))

            gz = gz_up  # now positive = height above ground

            # Color based on height (green to brown to gray)
            h_above = gz  # above ground level
            r = int(min(255, 100 + h_above * 50))
            g = int(min(255, 150 - h_above * 20))
            b = int(min(255, 80 + h_above * 10))
            all_pts.append((gx, gy, gz, max(0,r), max(0,g), max(0,b)))

print(f"Generated {len(all_pts)} terrain points")

# Write PLY
all_pts = np.array(all_pts, dtype=np.float32)
# xyz in COLMAP world (from create_colmap coords)
# For PLY export, just use NED directly
with open(OUT_PLY, 'wb') as f:
    header = (
        b"ply\n"
        b"format binary_little_endian 1.0\n"
        b"element vertex " + str(len(all_pts)).encode() + b"\n"
        b"property float x\n"
        b"property float y\n"
        b"property float z\n"
        b"property uchar red\n"
        b"property uchar green\n"
        b"property uchar blue\n"
        b"end_header\n"
    )
    f.write(header)
    for pt in all_pts:
        f.write(struct.pack('<3f', pt[0], pt[1], pt[2]))
        f.write(struct.pack('<3B', int(pt[3]), int(pt[4]), int(pt[5])))

print(f"Written: {OUT_PLY}")

# Also create a simple point_cloud.ply symlink for downstream scripts
pc_link = OUT_DIR / "point_cloud.ply"
if not pc_link.exists():
    import os
    os.symlink(str(OUT_PLY), str(pc_link))
    print(f"Symlink: {pc_link} -> {OUT_PLY}")
