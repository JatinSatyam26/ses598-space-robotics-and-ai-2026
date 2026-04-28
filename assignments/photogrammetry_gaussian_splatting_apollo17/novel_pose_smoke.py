import json
import numpy as np
import torch
import struct
from pathlib import Path
import cv2

SPLAT_DIR = Path('/home/jatin-satyam/ses598-apollo17/splats_n15/apollo17_n15/splatfacto/2026-04-25_144254')
SPARSE_DIR = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense/sparse')
OUT_DIR = Path('/home/jatin-satyam/ses598-apollo17/metrics_n15')

# Read dataparser transforms to understand coordinate mapping
transforms_path = SPLAT_DIR / 'dataparser_transforms.json'
with open(transforms_path) as f:
    dp = json.load(f)
print('Dataparser transforms:')
print(json.dumps(dp, indent=2))

# Interpolate between camera 0 and camera 6 (first and last mag137 views) at t=0.5
# We'll do this by reading the transforms.json that nerfstudio produces
# Actually nerfstudio COLMAP dataparser uses the sparse model directly.
# The dataparser_transforms.json has: transform (4x4) and scale

# Read COLMAP cameras from dense/sparse (PINHOLE format)
CAMERAS_PATH = SPARSE_DIR / 'cameras.bin'
IMAGES_PATH = SPARSE_DIR / 'images.bin'

def read_cameras_bin(path):
    cameras = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            cam_id = struct.unpack('I', f.read(4))[0]
            model_id = struct.unpack('i', f.read(4))[0]
            w = struct.unpack('Q', f.read(8))[0]
            h = struct.unpack('Q', f.read(8))[0]
            nparams = {0:3, 1:4, 2:4, 3:5, 4:8}[model_id]
            params = struct.unpack(f'{nparams}d', f.read(8*nparams))
            cameras[cam_id] = {'model_id': model_id, 'w': w, 'h': h, 'params': params}
    return cameras

def read_images_bin(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            img_id = struct.unpack('I', f.read(4))[0]
            qw, qx, qy, qz = struct.unpack('4d', f.read(32))
            tx, ty, tz = struct.unpack('3d', f.read(24))
            cam_id = struct.unpack('I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00':
                    break
                name += c
            n_pts = struct.unpack('Q', f.read(8))[0]
            f.read(n_pts * 24)
            images[img_id] = {
                'qvec': (qw, qx, qy, qz),
                'tvec': (tx, ty, tz),
                'cam_id': cam_id,
                'name': name.decode(),
            }
    return images

cameras = read_cameras_bin(CAMERAS_PATH)
images = read_images_bin(IMAGES_PATH)

def qvec_to_R(qvec):
    qw, qx, qy, qz = qvec
    R = np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ])
    return R

def slerp(q0, q1, t):
    q0 = np.array(q0); q1 = np.array(q1)
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1; dot = -dot
    dot = min(dot, 1.0)
    theta = np.arccos(dot)
    if theta < 1e-8:
        return q0
    return (np.sin((1-t)*theta) * q0 + np.sin(t*theta) * q1) / np.sin(theta)

img_list = sorted(images.values(), key=lambda x: x['name'])
mag137 = [img for img in img_list if 'mag137' in img['name']]
i0, i1 = mag137[0], mag137[-1]
print(f'Interpolating between {i0["name"]} and {i1["name"]}')

t = 0.5
q_interp = slerp(i0['qvec'], i1['qvec'], t)
t_interp = tuple((1-t)*np.array(i0['tvec']) + t*np.array(i1['tvec']))

R = qvec_to_R(q_interp)
t_vec = np.array(t_interp)
c2w = np.eye(4)
c2w[:3, :3] = R.T
c2w[:3, 3] = -R.T @ t_vec

cam = cameras[i0['cam_id']]
fx, fy, cx, cy = cam['params']
w, h = cam['w'], cam['h']

# Apply dataparser transform
T = np.array(dp['transform'])
scale = dp['scale']
c2w_ns_34 = T @ c2w
c2w_ns_34[:3, 3] *= scale
c2w_ns = np.vstack([c2w_ns_34, [0, 0, 0, 1]])

print(f'Interpolated camera center (nerfstudio coords): {c2w_ns[:3, 3]}')
print(f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  {w}x{h}')

# Write a cameras_path JSON for ns-render camera-path
# nerfstudio camera_path format
camera_path = {
    'camera_type': 'perspective',
    'render_height': h,
    'render_width': w,
    'fps': 1,
    'seconds': 1.0,
    'camera_path': [{
        'camera_to_world': c2w_ns.flatten().tolist(),
        'fov': float(2 * np.degrees(np.arctan(0.5 * h / fy))),
        'aspect': float(w / h),
    }],
}

cam_path_file = OUT_DIR / 'novel_camera_path.json'
with open(cam_path_file, 'w') as f:
    json.dump(camera_path, f, indent=2)
print(f'Saved camera path: {cam_path_file}')
