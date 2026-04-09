#!/usr/bin/env python3
"""
create_colmap_from_poses.py — Build a COLMAP sparse model from known PX4 poses.

Skips COLMAP feature matching entirely: directly writes cameras/images/points3D
from the camera_info.json intrinsics and poses.json odometry.  Useful for
Gazebo simulation where COLMAP's matcher often struggles with low-texture ground.

Input files (~/midterm_flight_data/):
  camera_info.json  — K matrix + width/height from /drone/front_depth/camera_info
  poses.json        — per-frame odom from image_saver.py:
    [{"frame_id": N, "ros_frame": N, "rgb_path": "...",
      "odom": {"position": [x,y,z], "quaternion": [qw,qx,qy,qz]}}, ...]

Output (~/midterm_reconstruction/sparse/0/):
  cameras.bin / images.bin / points3D.bin   (pycolmap)
  cameras.txt / images.txt / points3D.txt   (fallback text format)

Coordinate conventions
-----------------------
PX4 world  : NED  — X=North, Y=East, Z=Down (right-handed)
PX4 body   : FRD  — X=Forward, Y=Right, Z=Down
PX4 q odom : rotates FRD body → NED world  (q_world_body)

Downward mono camera (mounted level, lens pointing down, top of image = North):
  camera X (right in image)  ←→  body Y  (East/Right)
  camera Y (down in image)   ←→  body -X (-North/Forward)  [image rows increase southward]
  camera Z (optical / into ground) ←→  body Z  (Down)

  R_body_to_cam = [[0,  1, 0],
                   [-1, 0, 0],
                   [0,  0, 1]]

COLMAP expects world-to-camera rotation R_cw and translation t = -R_cw @ C_world.

  R_cw = R_body_to_cam @ R_ned_to_body
       = R_bc @ R_nb.T

Adjust R_BODY_TO_CAM below if your Gazebo camera mount differs.
"""

import json
import math
import os
import sys
import struct
from pathlib import Path

# ── Configuration ──────────────────────────────────────────────────────────────

DATA_DIR  = Path.home() / "midterm_flight_data"
OUT_DIR   = Path.home() / "midterm_reconstruction" / "sparse" / "0"

# Downward camera: top=North, right=East.  Change if camera mount differs.
# Rows = body-to-camera rotation matrix (maps body-frame vector to camera-frame).
R_BODY_TO_CAM = [
    [ 0.0,  1.0,  0.0],   # cam_x = +body_y (East = right in image)
    [-1.0,  0.0,  0.0],   # cam_y = -body_x (South = downward in image)
    [ 0.0,  0.0,  1.0],   # cam_z = +body_z (Down = optical axis toward ground)
]

# ── Math helpers ───────────────────────────────────────────────────────────────

def quat_to_rotmat(qw, qx, qy, qz):
    """Quaternion → 3×3 rotation matrix (row-major list of lists)."""
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return [
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ]

def matmul33(A, B):
    """3×3 matrix multiply."""
    return [
        [sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]

def transpose33(M):
    return [[M[j][i] for j in range(3)] for i in range(3)]

def matvec3(M, v):
    return [sum(M[i][k]*v[k] for k in range(3)) for i in range(3)]

def rotmat_to_quat(R):
    """3×3 rotation matrix → quaternion (qw, qx, qy, qz)."""
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2][1] - R[1][2]) * s
        qy = (R[0][2] - R[2][0]) * s
        qz = (R[1][0] - R[0][1]) * s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s
    return qw, qx, qy, qz

# ── Load inputs ────────────────────────────────────────────────────────────────

def load_camera_info(path):
    with open(path) as f:
        info = json.load(f)
    K   = info['K']          # 9-element row-major 3×3
    fx  = K[0]; fy = K[4]
    cx  = K[2]; cy = K[5]
    w   = info['width']
    h   = info['height']
    return w, h, fx, fy, cx, cy

def load_poses(path):
    """
    Supports both formats:
      A) image_saver.py: {"frame_id": N, "rgb_path": "...", "odom": {"position": [...], "quaternion": [...]}}
      B) legacy:         {"frame": "name.png", "position": [...], "quaternion": [...]}
    Returns list of (image_name, pos_ned, quat_wxyz).
    """
    with open(path) as f:
        entries = json.load(f)

    result = []
    for e in entries:
        # Extract image filename
        if 'rgb_path' in e:
            name = os.path.basename(e['rgb_path'])
        elif 'frame' in e:
            name = e['frame']
        else:
            print(f"  WARNING: skipping entry with no image name: {e}")
            continue

        # Extract position and quaternion
        if 'odom' in e:
            pos  = e['odom']['position']    # [x, y, z] NED
            quat = e['odom']['quaternion']  # [qw, qx, qy, qz]
        else:
            pos  = e['position']
            quat = e['quaternion']

        if pos is None or quat is None:
            print(f"  WARNING: {name} has no odometry — skipping")
            continue

        result.append((name, pos, quat))

    return result

# ── Pose conversion ────────────────────────────────────────────────────────────

def ned_pose_to_colmap(pos_ned, quat_wxyz, R_bc):
    """
    Convert a PX4 NED body pose to COLMAP world-to-camera rotation + translation.

    Returns (qw, qx, qy, qz), (tx, ty, tz) for COLMAP images format.
    """
    qw, qx, qy, qz = quat_wxyz

    # R_nb: body → NED (from PX4 odometry quaternion)
    R_nb = quat_to_rotmat(qw, qx, qy, qz)

    # R_bn: NED → body
    R_bn = transpose33(R_nb)

    # R_cw: NED world → camera  =  R_bc @ R_bn
    R_cw = matmul33(R_bc, R_bn)

    # Translation: camera center in world (NED), then project to camera frame
    # t = -R_cw @ p_NED
    t = [-sum(R_cw[i][k] * pos_ned[k] for k in range(3)) for i in range(3)]

    q_out = rotmat_to_quat(R_cw)
    return q_out, t

# ── COLMAP text-format writer ──────────────────────────────────────────────────

def write_text_model(out_dir, w, h, fx, fy, cx, cy, image_poses):
    """Write cameras.txt, images.txt, points3D.txt in COLMAP text format."""
    out_dir.mkdir(parents=True, exist_ok=True)

    # cameras.txt
    with open(out_dir / 'cameras.txt', 'w') as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write(f"1 PINHOLE {w} {h} {fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f}\n")

    # images.txt
    with open(out_dir / 'images.txt', 'w') as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
        for img_id, (name, (qw,qx,qy,qz), (tx,ty,tz)) in enumerate(image_poses, start=1):
            f.write(f"{img_id} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} "
                    f"{tx:.9f} {ty:.9f} {tz:.9f} 1 {name}\n")
            f.write("\n")  # empty POINTS2D line

    # points3D.txt (empty — no 3D points from poses alone)
    with open(out_dir / 'points3D.txt', 'w') as f:
        f.write("# 3D point list (empty — poses only, no triangulation)\n")
        f.write("# Run colmap mapper or triangulator to add 3D points.\n")

    print(f"  Text model written to {out_dir}")

# ── COLMAP binary-format writer ────────────────────────────────────────────────

def write_binary_model(out_dir, w, h, fx, fy, cx, cy, image_poses):
    """
    Write cameras.bin, images.bin, points3D.bin (COLMAP binary format).
    Does not require pycolmap — implements the binary spec directly.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    # cameras.bin
    # Format: num_cameras (uint64), then per camera:
    #   camera_id (uint32), model_id (int32), width (uint64), height (uint64),
    #   params[] (double * num_params)
    PINHOLE_MODEL_ID = 1
    PINHOLE_NUM_PARAMS = 4  # fx, fy, cx, cy
    with open(out_dir / 'cameras.bin', 'wb') as f:
        f.write(struct.pack('<Q', 1))  # 1 camera
        f.write(struct.pack('<i', 1))  # camera_id = 1
        f.write(struct.pack('<i', PINHOLE_MODEL_ID))
        f.write(struct.pack('<Q', w))
        f.write(struct.pack('<Q', h))
        f.write(struct.pack('<4d', fx, fy, cx, cy))

    # images.bin
    # Format: num_images (uint64), then per image:
    #   image_id (uint32), qvec (4×double: qw,qx,qy,qz), tvec (3×double),
    #   camera_id (uint32), name (null-terminated string),
    #   num_points2D (uint64), then per point2D: x (double), y (double), point3D_id (int64)
    with open(out_dir / 'images.bin', 'wb') as f:
        f.write(struct.pack('<Q', len(image_poses)))
        for img_id, (name, (qw,qx,qy,qz), (tx,ty,tz)) in enumerate(image_poses, start=1):
            f.write(struct.pack('<I', img_id))
            f.write(struct.pack('<4d', qw, qx, qy, qz))
            f.write(struct.pack('<3d', tx, ty, tz))
            f.write(struct.pack('<I', 1))  # camera_id
            f.write(name.encode() + b'\x00')
            f.write(struct.pack('<Q', 0))  # 0 points2D

    # points3D.bin — empty
    with open(out_dir / 'points3D.bin', 'wb') as f:
        f.write(struct.pack('<Q', 0))

    print(f"  Binary model written to {out_dir}")

# ── pycolmap writer (optional) ─────────────────────────────────────────────────

def write_pycolmap_model(out_dir, w, h, fx, fy, cx, cy, image_poses):
    import pycolmap
    import numpy as np

    reconstruction = pycolmap.Reconstruction()

    cam = pycolmap.Camera(
        model='PINHOLE',
        width=w,
        height=h,
        params=[fx, fy, cx, cy],
    )
    cam.camera_id = 1
    reconstruction.add_camera(cam)

    for img_id, (name, (qw,qx,qy,qz), (tx,ty,tz)) in enumerate(image_poses, start=1):
        # pycolmap Image uses Rigid3d for cam_from_world
        try:
            # pycolmap >= 0.6
            img = pycolmap.Image(
                image_id=img_id,
                name=name,
                camera_id=1,
                cam_from_world=pycolmap.Rigid3d(
                    rotation=pycolmap.Rotation3d([qw, qx, qy, qz]),
                    translation=np.array([tx, ty, tz]),
                ),
            )
        except (AttributeError, TypeError):
            # older pycolmap: use tvec + qvec directly
            img = pycolmap.Image(name=name, camera_id=1)
            img.image_id = img_id
            img.qvec = np.array([qw, qx, qy, qz])
            img.tvec = np.array([tx, ty, tz])

        img.registered = True
        reconstruction.add_image(img)

    out_dir.mkdir(parents=True, exist_ok=True)
    reconstruction.write_binary(str(out_dir))
    print(f"  pycolmap binary model written to {out_dir}")

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    camera_info_path = DATA_DIR / 'camera_info.json'
    poses_path       = DATA_DIR / 'poses.json'

    print("=== create_colmap_from_poses ===")
    print(f"  Data dir : {DATA_DIR}")
    print(f"  Out dir  : {OUT_DIR}")

    if not camera_info_path.exists():
        sys.exit(f"ERROR: {camera_info_path} not found. Run the drone survey first.")
    if not poses_path.exists():
        sys.exit(f"ERROR: {poses_path} not found. Run the drone survey first.")

    # Load
    w, h, fx, fy, cx, cy = load_camera_info(camera_info_path)
    print(f"  Camera   : {w}×{h}  fx={fx:.2f} fy={fy:.2f}  cx={cx:.2f} cy={cy:.2f}")

    raw_poses = load_poses(poses_path)
    print(f"  Poses    : {len(raw_poses)} frames")

    if not raw_poses:
        sys.exit("ERROR: No valid poses found.")

    # Convert poses
    converted = []
    for name, pos_ned, quat_wxyz in raw_poses:
        q_out, t_out = ned_pose_to_colmap(pos_ned, quat_wxyz, R_BODY_TO_CAM)
        converted.append((name, q_out, t_out))

    # Write model — prefer pycolmap binary, fall back to our own binary, then text
    try:
        import pycolmap
        print(f"  Using pycolmap {pycolmap.__version__}")
        write_pycolmap_model(OUT_DIR, w, h, fx, fy, cx, cy, converted)
    except ImportError:
        print("  pycolmap not available — writing binary format directly")
        write_binary_model(OUT_DIR, w, h, fx, fy, cx, cy, converted)

    # Always write text format alongside for human inspection
    text_dir = OUT_DIR.parent / "0_text"
    write_text_model(text_dir, w, h, fx, fy, cx, cy, converted)

    print()
    print("=== Done ===")
    print(f"  {len(converted)} camera poses written")
    print(f"  Binary: {OUT_DIR}")
    print(f"  Text  : {text_dir}")
    print()
    print("Next steps:")
    print(f"  # Triangulate 3D points from the known poses:")
    print(f"  colmap feature_extractor --database_path {OUT_DIR.parent.parent}/database.db \\")
    print(f"      --image_path ~/midterm_flight_data/rgb/")
    print(f"  colmap exhaustive_matcher --database_path {OUT_DIR.parent.parent}/database.db")
    print(f"  colmap point_triangulator \\")
    print(f"      --database_path {OUT_DIR.parent.parent}/database.db \\")
    print(f"      --image_path ~/midterm_flight_data/rgb/ \\")
    print(f"      --input_path {OUT_DIR} --output_path {OUT_DIR}")

if __name__ == '__main__':
    main()
