import sqlite3
import numpy as np
import cv2
import struct
import shutil
from pathlib import Path

DB_PATH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/database.db')
SPARSE_IN = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/sparse/0')
SPARSE_OUT = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/sparse/0_with21036')
SPARSE_OUT.mkdir(parents=True, exist_ok=True)

IMG36_ID = 14
IMG37_ID = 15

db = sqlite3.connect(str(DB_PATH))
cur = db.cursor()

def read_keypoints(image_id):
    cur.execute('SELECT rows, cols, data FROM keypoints WHERE image_id=?', (image_id,))
    row = cur.fetchone()
    n, cols, data = row
    kps = np.frombuffer(data, dtype=np.float32).reshape(n, cols)
    return kps[:, :2]

def read_matches(id1, id2):
    pair_id = id1 * 2147483647 + id2
    cur.execute('SELECT rows, cols, data, H FROM two_view_geometries WHERE pair_id=?', (pair_id,))
    row = cur.fetchone()
    n_rows, n_cols, data, H_bytes = row
    matches = np.frombuffer(data, dtype=np.uint32).reshape(n_rows, 2)
    H = np.frombuffer(H_bytes, dtype=np.float64).reshape(3, 3) if H_bytes else None
    return matches, H

def read_cameras_bin(path):
    cameras = {}
    model_nparams = {0:3, 1:4, 2:4, 3:5, 4:8, 5:8, 6:12, 7:5}
    with open(path, 'rb') as f:
        num = struct.unpack('<Q', f.read(8))[0]
        for _ in range(num):
            cid = struct.unpack('<I', f.read(4))[0]
            model = struct.unpack('<I', f.read(4))[0]
            w = struct.unpack('<Q', f.read(8))[0]
            h = struct.unpack('<Q', f.read(8))[0]
            np_ = model_nparams.get(model, 4)
            params = list(struct.unpack(f'<{np_}d', f.read(8*np_)))
            cameras[cid] = {'model': model, 'w': w, 'h': h, 'params': params}
    return cameras

def camera_K_dist(cam):
    model = cam['model']
    p = cam['params']
    if model == 0:
        f, cx, cy = p[0], p[1], p[2]
        K = np.array([[f,0,cx],[0,f,cy],[0,0,1]])
        dist = np.zeros(5)
    elif model == 1:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        dist = np.zeros(5)
    elif model == 2:
        f, cx, cy, k1 = p[0], p[1], p[2], p[3]
        K = np.array([[f,0,cx],[0,f,cy],[0,0,1]])
        dist = np.array([k1, 0, 0, 0, 0])
    elif model == 3:
        f, cx, cy, k1, k2 = p[0], p[1], p[2], p[3], p[4]
        K = np.array([[f,0,cx],[0,f,cy],[0,0,1]])
        dist = np.array([k1, k2, 0, 0, 0])
    else:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        dist = np.array(p[4:8] + [0]*(8-len(p[4:8])))[:5] if len(p)>4 else np.zeros(5)
    return K.astype(np.float64), dist.astype(np.float64)

def read_images_bin(path):
    images = {}
    model_nparams_kp = {0:3, 1:4, 2:4, 3:5, 4:8, 5:8, 6:12, 7:5}
    with open(path, 'rb') as f:
        num = struct.unpack('<Q', f.read(8))[0]
        for _ in range(num):
            iid = struct.unpack('<I', f.read(4))[0]
            qvec = np.array(struct.unpack('<4d', f.read(32)))
            tvec = np.array(struct.unpack('<3d', f.read(24)))
            cid = struct.unpack('<I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            name = name.decode()
            n = struct.unpack('<Q', f.read(8))[0]
            xys = np.array(struct.unpack(f'<{2*n}d', f.read(16*n))).reshape(-1,2) if n>0 else np.zeros((0,2))
            pt3d = np.array(struct.unpack(f'<{n}q', f.read(8*n)), dtype=np.int64) if n>0 else np.array([], dtype=np.int64)
            w,x,y,z = qvec
            R = np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],[2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],[2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])
            images[iid] = {'qvec': qvec, 'tvec': tvec, 'cam_id': cid, 'name': name, 'R': R, 'xys': xys, 'pt3d': pt3d}
    return images

def qvec_from_R(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])

kps36 = read_keypoints(IMG36_ID)
kps37 = read_keypoints(IMG37_ID)
matches, H_colmap = read_matches(IMG36_ID, IMG37_ID)

pts36 = kps36[matches[:, 0]].astype(np.float64)
pts37 = kps37[matches[:, 1]].astype(np.float64)

cameras_orig = read_cameras_bin(SPARSE_IN / 'cameras.bin')
images_orig = read_images_bin(SPARSE_IN / 'images.bin')

img37 = images_orig[15]
R37 = img37['R']
t37 = img37['tvec']
cam_id37 = img37['cam_id']
K37, dist37 = camera_K_dist(cameras_orig[cam_id37])
K36, dist36 = camera_K_dist(cameras_orig[2])

print(f'K36: f={K36[0,0]:.2f} cx={K36[0,2]:.2f} cy={K36[1,2]:.2f}')
print(f'K37: f={K37[0,0]:.2f} cx={K37[0,2]:.2f} cy={K37[1,2]:.2f}')

print(f'\nHcolmap:\n{H_colmap}')
print(f'\nDecomposing homography...')
retval, rotations, translations, normals = cv2.decomposeHomographyMat(H_colmap, K36)
print(f'{retval} solutions found')

pts36_u = cv2.undistortPoints(pts36.reshape(-1,1,2), K36, dist36).reshape(-1,2)
pts37_u = cv2.undistortPoints(pts37.reshape(-1,1,2), K37, dist37).reshape(-1,2)

best_sol = None
best_inliers = -1

for i, (R_rel, t_rel, n_vec) in enumerate(zip(rotations, translations, normals)):
    t_r = t_rel.flatten()
    P1 = np.hstack([np.eye(3), np.zeros((3,1))])
    P2 = np.hstack([R_rel, t_r.reshape(3,1)])
    pts_in_front = 0
    sample = min(100, len(pts36_u))
    for pt1, pt2 in zip(pts36_u[:sample], pts37_u[:sample]):
        A = np.array([
            pt1[0]*P1[2] - P1[0],
            pt1[1]*P1[2] - P1[1],
            pt2[0]*P2[2] - P2[0],
            pt2[1]*P2[2] - P2[1]
        ])
        _, _, Vt = np.linalg.svd(A)
        X = Vt[-1]
        X = X / X[3]
        z1 = X[2]
        z2 = (R_rel @ X[:3] + t_r)[2]
        if z1 > 0 and z2 > 0:
            pts_in_front += 1
    print(f'  sol {i}: t_rel={np.round(t_r,3)} pts_in_front={pts_in_front}/{sample}')
    if pts_in_front > best_inliers:
        best_inliers = pts_in_front
        best_sol = (R_rel, t_r)

db.close()

if best_sol is None:
    print('No valid solution found - giving up on 21036')
    exit(1)

R_rel, t_rel = best_sol
print(f'\nSelected: pts_in_front={best_inliers}')

R36_abs = R37 @ R_rel.T
C36 = -R37.T @ t37 + R_rel.T @ t_rel
t36_abs = -R36_abs @ (-R37.T @ t37 - R_rel.T @ t_rel)

t36_abs2 = t37 + R37 @ (-R_rel.T @ t_rel)

print(f't36_abs from 21037 pose: {np.round(t36_abs,4)}')
print(f't36_abs alternative:     {np.round(t36_abs2,4)}')
print(f't37 for reference:       {np.round(t37,4)}')

C37 = -R37.T @ t37
C36_calc = C37 + R37.T @ (R_rel.T @ t_rel)
t36_final = -R36_abs @ C36_calc

print(f'C37: {np.round(C37,4)}')
print(f'C36: {np.round(C36_calc,4)}')
print(f't36_final: {np.round(t36_final,4)}')

qvec36 = qvec_from_R(R36_abs)
print(f'qvec36: {np.round(qvec36,6)}')

shutil.copy(SPARSE_IN / 'cameras.bin', SPARSE_OUT / 'cameras.bin')
shutil.copy(SPARSE_IN / 'points3D.bin', SPARSE_OUT / 'points3D.bin')

with open(SPARSE_IN / 'images.bin', 'rb') as fin, open(SPARSE_OUT / 'images.bin', 'wb') as fout:
    num_orig = struct.unpack('<Q', fin.read(8))[0]
    fout.write(struct.pack('<Q', num_orig + 1))
    fout.write(fin.read())

    new_img_id = 16
    fout.write(struct.pack('<I', new_img_id))
    fout.write(struct.pack('<4d', *qvec36))
    fout.write(struct.pack('<3d', *t36_final))
    fout.write(struct.pack('<I', 2))
    fout.write(b'mag138/AS17-138-21036HR.png\x00')

    n_kp = len(kps36)
    fout.write(struct.pack('<Q', n_kp))
    for kp in kps36:
        fout.write(struct.pack('<2d', float(kp[0]), float(kp[1])))
        fout.write(struct.pack('<q', -1))

print(f'\nWrote extended sparse model to {SPARSE_OUT}')
print(f'21036 pose: C=[{C36_calc[0]:.3f},{C36_calc[1]:.3f},{C36_calc[2]:.3f}]')
