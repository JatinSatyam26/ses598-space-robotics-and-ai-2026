import numpy as np
import open3d as o3d
import json
import struct
import cv2
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

SPARSE14 = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense/sparse')
N14_MESH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense/meshed-poisson.ply')
N24_MESH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n24/dense/meshed-poisson.ply')
IMAGES14 = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense/images')
METRICS_JSON = Path('/home/jatin-satyam/ses598-apollo17/comparison/quantitative_metrics.json')
OUT_DIR = Path('/home/jatin-satyam/ses598-apollo17/comparison')

with open(METRICS_JSON) as f:
    metrics = json.load(f)
T_icp = np.array(metrics['icp_transform'])

def read_cameras_bin(path):
    cameras = {}
    nparams = {0:3, 1:4, 2:4, 3:5, 4:8, 5:8, 6:12, 7:5}
    with open(path, 'rb') as f:
        num = struct.unpack('<Q', f.read(8))[0]
        for _ in range(num):
            cid = struct.unpack('<I', f.read(4))[0]
            model = struct.unpack('<I', f.read(4))[0]
            w = struct.unpack('<Q', f.read(8))[0]
            h = struct.unpack('<Q', f.read(8))[0]
            n = nparams.get(model, 4)
            params = list(struct.unpack(f'<{n}d', f.read(8*n)))
            cameras[cid] = {'model': model, 'w': w, 'h': h, 'params': params}
    return cameras

def read_images_bin(path):
    images = {}
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
            w_q, x_q, y_q, z_q = qvec
            R = np.array([
                [1-2*y_q*y_q-2*z_q*z_q, 2*x_q*y_q-2*z_q*w_q, 2*x_q*z_q+2*y_q*w_q],
                [2*x_q*y_q+2*z_q*w_q, 1-2*x_q*x_q-2*z_q*z_q, 2*y_q*z_q-2*x_q*w_q],
                [2*x_q*z_q-2*y_q*w_q, 2*y_q*z_q+2*x_q*w_q, 1-2*x_q*x_q-2*y_q*y_q]
            ])
            C = -R.T @ tvec
            images[iid] = {'qvec': qvec, 'tvec': tvec, 'cam_id': cid, 'name': name,
                           'R': R, 'C': C, 'xys': xys, 'pt3d': pt3d}
    return images

def read_points3d_bin(path):
    pts = {}
    with open(path, 'rb') as f:
        num = struct.unpack('<Q', f.read(8))[0]
        for _ in range(num):
            pid = struct.unpack('<Q', f.read(8))[0]
            xyz = np.array(struct.unpack('<3d', f.read(24)))
            rgb = np.array(struct.unpack('<3B', f.read(3)))
            err = struct.unpack('<d', f.read(8))[0]
            n = struct.unpack('<Q', f.read(8))[0]
            f.read(n * 8)
            pts[pid] = (xyz, err)
    return pts

cameras = read_cameras_bin(SPARSE14 / 'cameras.bin')
images = read_images_bin(SPARSE14 / 'images.bin')
pts3d = read_points3d_bin(SPARSE14 / 'points3D.bin')

def compute_reproj_error(img_info):
    cam = cameras[img_info['cam_id']]
    p = cam['params']
    if cam['model'] == 1:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
    else:
        fx = fy = p[0]; cx, cy = p[1], p[2]
    R = img_info['R']
    t = img_info['tvec']
    errors = []
    for i, pid in enumerate(img_info['pt3d']):
        if pid < 0 or pid not in pts3d:
            continue
        xyz, _ = pts3d[pid]
        Xc = R @ xyz + t
        if Xc[2] <= 0:
            continue
        u_proj = fx * Xc[0] / Xc[2] + cx
        v_proj = fy * Xc[1] / Xc[2] + cy
        u_obs, v_obs = img_info['xys'][i]
        errors.append(np.sqrt((u_proj - u_obs)**2 + (v_proj - v_obs)**2))
    return float(np.mean(errors)) if errors else float('inf')

img_errors = {}
for iid, img_info in images.items():
    err = compute_reproj_error(img_info)
    img_errors[iid] = err
    print(f'  {img_info["name"]}: reproj_error={err:.3f}px')

sorted_imgs = sorted(img_errors.items(), key=lambda x: x[1])
n_imgs = len(sorted_imgs)
pick_indices = [n_imgs//5, n_imgs*2//5, n_imgs*3//5, n_imgs*4//5]
chosen_ids = [sorted_imgs[i][0] for i in pick_indices]
print(f'\nChosen cameras (near-median reprojection error):')
for iid in chosen_ids:
    print(f'  img {iid}: {images[iid]["name"]}  error={img_errors[iid]:.3f}px')

print('\nLoading meshes...')
mesh14 = o3d.io.read_triangle_mesh(str(N14_MESH))
mesh14.compute_vertex_normals()
mesh24 = o3d.io.read_triangle_mesh(str(N24_MESH))
mesh24.transform(T_icp)
mesh24.compute_vertex_normals()

def render_mesh_matplotlib(mesh, R_cam, t_cam, cam_info, W=800, H=800, title=''):
    verts = np.asarray(mesh.vertices)
    colors_v = np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None

    Xc = (R_cam @ verts.T + t_cam.reshape(3,1)).T
    mask = Xc[:, 2] > 0.1

    p = cam_info['params']
    if cam_info['model'] == 1:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
    else:
        fx = fy = p[0]; cx, cy = p[1], p[2]

    scale_x = W / cam_info['w']
    scale_y = H / cam_info['h']

    u = (fx * Xc[:,0] / Xc[:,2].clip(1e-6) + cx) * scale_x
    v = (fy * Xc[:,1] / Xc[:,2].clip(1e-6) + cy) * scale_y

    in_img = mask & (u >= 0) & (u < W) & (v >= 0) & (v < H)

    img = np.ones((H, W, 3), dtype=np.float32) * 0.95

    depths = Xc[:, 2]
    d_min, d_max = depths[in_img].min() if in_img.any() else 1.0, depths[in_img].max() if in_img.any() else 10.0

    valid_u = u[in_img].astype(int).clip(0, W-1)
    valid_v = v[in_img].astype(int).clip(0, H-1)
    valid_d = depths[in_img]

    if colors_v is not None:
        valid_c = colors_v[in_img]
    else:
        norm_d = (valid_d - d_min) / (d_max - d_min + 1e-10)
        valid_c = np.stack([norm_d, norm_d, norm_d], axis=1)

    sort_order = np.argsort(-valid_d)
    for idx in sort_order:
        uu, vv = valid_u[idx], valid_v[idx]
        img[vv, uu] = valid_c[idx]

    img_uint8 = (img * 255).clip(0, 255).astype(np.uint8)
    return img_uint8

sidebyside_paths = []
for view_idx, iid in enumerate(chosen_ids, start=1):
    img_info = images[iid]
    cam = cameras[img_info['cam_id']]
    R_cam = img_info['R']
    t_cam = img_info['tvec']
    img_name = img_info['name']

    print(f'\nRendering view {view_idx}: {img_name}')

    render14 = render_mesh_matplotlib(mesh14, R_cam, t_cam, cam)
    render24 = render_mesh_matplotlib(mesh24, R_cam, t_cam, cam)

    r14_path = OUT_DIR / f'render_n14_view{view_idx}.png'
    r24_path = OUT_DIR / f'render_n24_view{view_idx}.png'
    Image.fromarray(render14).save(str(r14_path))
    Image.fromarray(render24).save(str(r24_path))
    print(f'  Saved: {r14_path.name}, {r24_path.name}')

    photo_name = img_name
    photo_path = IMAGES14 / photo_name
    has_photo = photo_path.exists()

    if has_photo:
        photo = cv2.imread(str(photo_path))
        photo = cv2.cvtColor(photo, cv2.COLOR_BGR2RGB)
        photo_resized = cv2.resize(photo, (800, 800))
        fig, axes = plt.subplots(1, 3, figsize=(24, 8))
        axes[0].imshow(render14)
        axes[0].set_title(f'N=14 Mesh', fontsize=13, color='steelblue')
        axes[0].axis('off')
        axes[1].imshow(photo_resized)
        axes[1].set_title(f'Original Photo: {Path(photo_name).name}', fontsize=13, color='goldenrod')
        axes[1].axis('off')
        axes[2].imshow(render24)
        axes[2].set_title(f'N=24 Mesh (ICP-aligned)', fontsize=13, color='mediumvioletred')
        axes[2].axis('off')
    else:
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        axes[0].imshow(render14)
        axes[0].set_title(f'N=14 Mesh', fontsize=13, color='steelblue')
        axes[0].axis('off')
        axes[1].imshow(render24)
        axes[1].set_title(f'N=24 Mesh (ICP-aligned)', fontsize=13, color='mediumvioletred')
        axes[1].axis('off')

    fig.suptitle(f'View {view_idx}: {Path(photo_name).stem}  reproj_err={img_errors[iid]:.2f}px', fontsize=14)
    fig.tight_layout()
    sbs_path = OUT_DIR / f'sidebyside_view{view_idx}.png'
    plt.savefig(str(sbs_path), dpi=100, bbox_inches='tight')
    plt.close()
    sidebyside_paths.append(sbs_path)
    print(f'  Saved: {sbs_path.name}')

print('\nGenerating qualitative grid (2x2 of side-by-sides)...')
fig, axes = plt.subplots(2, 2, figsize=(28, 18))
for i, sbs_path in enumerate(sidebyside_paths):
    row, col = i // 2, i % 2
    img = np.array(Image.open(sbs_path))
    axes[row, col].imshow(img)
    axes[row, col].set_title(f'View {i+1}', fontsize=12)
    axes[row, col].axis('off')
fig.suptitle('Qualitative Comparison: N=14 vs N=24 Mesh (4 Viewpoints)', fontsize=16, fontweight='bold')
plt.tight_layout()
grid_path = OUT_DIR / 'qualitative_grid.png'
plt.savefig(str(grid_path), dpi=80, bbox_inches='tight')
plt.close()
print(f'Saved: {grid_path}')
print('\n===== STAGE: QUALITATIVE COMPARISON COMPLETE =====')
