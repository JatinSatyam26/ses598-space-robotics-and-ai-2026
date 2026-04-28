import numpy as np
import struct
import json
from pathlib import Path

SPARSE = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/sparse/0')
SPLAT_DIR = Path('/home/jatin-satyam/ses598-apollo17/splats_n15/apollo17_n15/splatfacto/2026-04-25_144254')
OUT_DIR = Path('/home/jatin-satyam/ses598-apollo17/novel_poses')
OUT_DIR.mkdir(parents=True, exist_ok=True)

with open(SPLAT_DIR / 'dataparser_transforms.json') as f:
    dp = json.load(f)
T34 = np.array(dp['transform'], dtype=np.float64)
scale = float(dp['scale'])
print('Transform T34:')
print(T34)
print(f'Scale: {scale}')

def read_cameras(path):
    cameras = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            cid = struct.unpack('I', f.read(4))[0]
            mid = struct.unpack('i', f.read(4))[0]
            w = struct.unpack('Q', f.read(8))[0]
            h = struct.unpack('Q', f.read(8))[0]
            nparams = {0:3,1:4,2:4,3:5,4:8}[mid]
            params = struct.unpack(f'{nparams}d', f.read(8*nparams))
            cameras[cid] = {'model_id':mid,'w':w,'h':h,'params':params}
    return cameras

def read_images(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack('I', f.read(4))[0]
            qw,qx,qy,qz = struct.unpack('4d', f.read(32))
            tx,ty,tz = struct.unpack('3d', f.read(24))
            cid = struct.unpack('I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            npts = struct.unpack('Q', f.read(8))[0]
            f.read(npts*24)
            images[iid] = {'qvec':(qw,qx,qy,qz),'tvec':(tx,ty,tz),'cam_id':cid,'name':name.decode()}
    return images

def qvec_to_R(qvec):
    qw,qx,qy,qz = qvec
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx**2+qy**2)]
    ])

def read_images_with_pts(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack('I', f.read(4))[0]
            qw,qx,qy,qz = struct.unpack('4d', f.read(32))
            tx,ty,tz = struct.unpack('3d', f.read(24))
            cid = struct.unpack('I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            npts = struct.unpack('Q', f.read(8))[0]
            pt_ids = []
            for _ in range(npts):
                struct.unpack('2d', f.read(16))
                pid = struct.unpack('Q', f.read(8))[0]
                if pid != 2**64-1:
                    pt_ids.append(pid)
            images[iid] = {'name': name.decode(), 'cam_id': cid,
                           'qvec': (qw,qx,qy,qz), 'tvec': (tx,ty,tz),
                           'pt3d_ids': set(pt_ids)}
    return images

def read_all_pts(path, filter_box):
    pts = {}
    x0,x1,y0,y1,z0,z1 = filter_box
    with open(path, 'rb') as f:
        n = struct.unpack('Q', f.read(8))[0]
        for _ in range(n):
            pid = struct.unpack('Q', f.read(8))[0]
            x,y,z = struct.unpack('3d', f.read(24))
            f.read(3); f.read(8)
            ntrack = struct.unpack('Q', f.read(8))[0]
            f.read(ntrack*8)
            if x0<=x<=x1 and y0<=y<=y1 and z0<=z<=z1:
                pts[pid] = np.array([x,y,z])
    return pts

bbox = (-7.5, 1.0, -2.5, 4.5, 0.5, 11.0)
all_pts = read_all_pts(SPARSE / 'points3D.bin', bbox)

cameras = read_cameras(SPARSE / 'cameras.bin')
images = read_images_with_pts(SPARSE / 'images.bin')

mag137_imgs = [img for img in images.values() if 'mag137' in img['name']]
mag138_imgs = [img for img in images.values() if 'mag138' in img['name']]

def rock_centroid(imgs):
    ptids = set()
    for img in imgs:
        ptids.update(img['pt3d_ids'])
    pts = np.array([all_pts[pid] for pid in ptids if pid in all_pts])
    return np.median(pts, axis=0) if len(pts) > 0 else None

T_lookat137 = rock_centroid(mag137_imgs)
T_lookat138 = rock_centroid(mag138_imgs)
print(f'Rock centroid mag137: {T_lookat137.round(4)}')
print(f'Rock centroid mag138: {T_lookat138.round(4)}')

def cluster_geometry(imgs):
    centers, forwards = [], []
    for img in imgs:
        R = qvec_to_R(img['qvec'])
        t = np.array(img['tvec'])
        C = -R.T @ t
        fwd = R.T[:, 2]
        centers.append(C)
        forwards.append(fwd)
    centers = np.array(centers)
    forwards = np.array(forwards)
    centroid = centers.mean(0)
    radii = np.linalg.norm(centers - centroid, axis=1)
    radius = float(np.median(radii))
    mean_fwd = forwards.mean(0)
    mean_fwd = mean_fwd / np.linalg.norm(mean_fwd)
    return centroid, radius, mean_fwd, centers

C137, r137, F137, centers137 = cluster_geometry(mag137_imgs)
C138, r138, F138, centers138 = cluster_geometry(mag138_imgs)

T_formula137 = C137 + F137 * r137 * 1.5
T_formula138 = C138 + F138 * r138 * 1.5

print(f'\nmag137: C={C137.round(4)}  r={r137:.4f}  F={F137.round(4)}')
print(f'  lookat_target(formula)={T_formula137.round(4)}  lookat_target(actual)={T_lookat137.round(4)}')
print(f'mag138: C={C138.round(4)}  r={r138:.4f}  F={F138.round(4)}')
print(f'  lookat_target(formula)={T_formula138.round(4)}  lookat_target(actual)={T_lookat138.round(4)}')

def lookat_colmap(eye, target):
    world_up = np.array([0., 1., 0.])
    Z = target - eye
    Z = Z / np.linalg.norm(Z)
    X = np.cross(world_up, Z)
    X = X / np.linalg.norm(X)
    Y = np.cross(X, Z)
    c2w = np.eye(4)
    c2w[:3, 0] = X
    c2w[:3, 1] = Y
    c2w[:3, 2] = Z
    c2w[:3, 3] = eye
    return c2w

col_flip = np.diag([1., -1., -1., 1.])

def to_nerfstudio(c2w_44):
    c2w_34 = (T34 @ (c2w_44 @ col_flip)).copy()
    c2w_34[:, 3] *= scale
    return np.vstack([c2w_34, [0., 0., 0., 1.]])

def arc_poses(C, r, F, T_lookat, n=5, span_deg=30.0):
    F_xz = np.array([F[0], 0., F[2]])
    F_xz = F_xz / np.linalg.norm(F_xz)
    P_xz = np.array([-F_xz[2], 0., F_xz[0]])
    angles = np.linspace(-span_deg, span_deg, n)
    results = []
    for a in angles:
        th = np.radians(a)
        d = np.cos(th) * F_xz + np.sin(th) * P_xz
        eye = C + r * d
        eye[1] = C[1]
        c2w_colmap = lookat_colmap(eye, T_lookat)
        c2w_ns = to_nerfstudio(c2w_colmap)
        results.append({
            'angle_deg': float(a),
            'eye_colmap': eye.tolist(),
            'c2w_colmap': c2w_colmap.tolist(),
            'c2w_ns': c2w_ns.tolist(),
        })
    return results

poses137 = arc_poses(C137, r137, F137, T_formula137, n=5, span_deg=30.0)
poses138 = arc_poses(C138, r138, F138, T_formula138, n=5, span_deg=30.0)

cam1 = cameras[1]
f_train = cam1['params'][0] / 2.0
h_train = cam1['h'] / 2.0
vfov_deg = float(2.0 * np.degrees(np.arctan(h_train / 2.0 / f_train)))
print(f'\nVertical FOV: {vfov_deg:.4f} degrees  (f_train={f_train:.2f}, h_train={h_train:.0f})')

RENDER_W, RENDER_H = 2000, 2000

camera_path_json = {
    'camera_type': 'perspective',
    'render_height': RENDER_H,
    'render_width': RENDER_W,
    'fps': 1,
    'seconds': float(10),
    'camera_path': [],
}

all_entries = [(p, 'mag137', i+1) for i, p in enumerate(poses137)] + \
              [(p, 'mag138', i+1) for i, p in enumerate(poses138)]

for p, cluster, idx in all_entries:
    camera_path_json['camera_path'].append({
        'camera_to_world': np.array(p['c2w_ns']).flatten().tolist(),
        'fov': vfov_deg,
        'aspect': float(RENDER_W) / float(RENDER_H),
    })

cam_path_file = OUT_DIR / 'novel_camera_path.json'
with open(cam_path_file, 'w') as f:
    json.dump(camera_path_json, f, indent=2)
print(f'\nCamera path saved: {cam_path_file} ({len(camera_path_json["camera_path"])} poses)')

print('\nPose definitions:')
for p, cluster, idx in all_entries:
    eye = np.array(p['eye_colmap'])
    c_ns = np.array(p['c2w_ns'])[:3, 3]
    print(f'  {cluster}_{idx:02d}  angle={p["angle_deg"]:+6.1f}  eye_colmap={eye.round(3)}  t_ns={c_ns.round(4)}')
