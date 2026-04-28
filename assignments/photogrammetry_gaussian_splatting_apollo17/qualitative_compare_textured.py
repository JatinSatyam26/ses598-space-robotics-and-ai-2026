import numpy as np
import open3d as o3d
import struct
import json
import os
os.environ['PYOPENGL_PLATFORM'] = 'egl'
import pyrender
import trimesh
from pathlib import Path
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SPARSE14 = Path('colmap_n15/sparse/0')
SPARSE24 = Path('colmap_n24/sparse/0')
PLY14 = Path('colmap_n15/dense/textured/textured.ply')
PLY24 = Path('colmap_n24/dense/textured/textured.ply')
METRICS_JSON = Path('comparison/quantitative_metrics.json')
IMAGES14 = Path('data/rgb')
OUT_DIR = Path('comparison')

RENDER_W, RENDER_H = 800, 800

with open(METRICS_JSON) as f:
    T_icp = np.array(json.load(f)['icp_transform'])

def read_cameras_bin(path):
    cameras = {}
    nparams = {0:3,1:4,2:4,3:5,4:8}
    with open(path, 'rb') as f:
        n = struct.unpack('<Q', f.read(8))[0]
        for _ in range(n):
            cid = struct.unpack('<I', f.read(4))[0]
            mid = struct.unpack('<I', f.read(4))[0]
            w = struct.unpack('<Q', f.read(8))[0]
            h = struct.unpack('<Q', f.read(8))[0]
            np_ = nparams[mid]
            params = list(struct.unpack(f'<{np_}d', f.read(8*np_)))
            cameras[cid] = dict(model=mid, W=w, H=h, params=params)
    return cameras

def read_images_bin(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('<Q', f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack('<I', f.read(4))[0]
            qvec = np.array(struct.unpack('<4d', f.read(32)))
            tvec = np.array(struct.unpack('<3d', f.read(24)))
            cid = struct.unpack('<I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            n_pts = struct.unpack('<Q', f.read(8))[0]
            f.read(n_pts * 24)
            qw, qx, qy, qz = qvec
            R = np.array([
                [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)]
            ])
            images[iid] = dict(name=name.decode(), cam_id=cid, R=R, tvec=tvec)
    return images

def build_c2w_opengl(R, tvec):
    c2w = np.eye(4)
    c2w[:3, :3] = R.T
    c2w[:3, 3] = -R.T @ tvec
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    return c2w @ flip

def colmap_intrinsics_to_pyrender(cam_data, out_W, out_H):
    p = cam_data['params']
    m = cam_data['model']
    if m == 1:
        fx, fy = p[0], p[1]; cx, cy = p[2], p[3]
    elif m == 2:
        fx = fy = p[0]; cx = p[1]; cy = p[2]
    else:
        fx = fy = p[0]; cx = p[1]; cy = p[2]
    sx = out_W / cam_data['W']; sy = out_H / cam_data['H']
    return fx*sx, fy*sy, cx*sx, cy*sy

def load_vertex_colored_mesh(ply_path, transform=None):
    mesh_o3d = o3d.io.read_triangle_mesh(str(ply_path))
    if transform is not None:
        mesh_o3d.transform(transform)
    mesh_o3d.compute_vertex_normals()
    verts = np.asarray(mesh_o3d.vertices)
    tris = np.asarray(mesh_o3d.triangles)
    colors = (np.asarray(mesh_o3d.vertex_colors) * 255).astype(np.uint8)
    tm = trimesh.Trimesh(vertices=verts, faces=tris, vertex_colors=colors)
    return pyrender.Mesh.from_trimesh(tm, smooth=False)

def render_mesh(mesh_pr, pose, fx, fy, cx, cy, W=RENDER_W, H=RENDER_H):
    scene = pyrender.Scene(bg_color=[0.15, 0.15, 0.15, 1.0], ambient_light=[1.0, 1.0, 1.0])
    scene.add(mesh_pr)
    camera = pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=cx, cy=cy, znear=0.01, zfar=1000.0)
    scene.add(camera, pose=pose)
    light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=3.0)
    scene.add(light, pose=pose)
    renderer = pyrender.OffscreenRenderer(W, H)
    color, _ = renderer.render(scene, flags=pyrender.RenderFlags.FLAT)
    renderer.delete()
    return color

print('Loading cameras from N=14 sparse model...')
cams14 = read_cameras_bin(SPARSE14 / 'cameras.bin')
imgs14 = read_images_bin(SPARSE14 / 'images.bin')

chosen_names = [
    'mag137/AS17-137-20904HR.png',
    'mag137/AS17-137-20907HR.png',
    'mag138/AS17-138-21030HR.png',
    'mag138/AS17-138-21033HR.png',
]
chosen_imgs = [next(v for v in imgs14.values() if v['name'] == nm) for nm in chosen_names]

print('Loading N=14 textured mesh...')
mesh14_pr = load_vertex_colored_mesh(PLY14)

print('Loading N=24 textured mesh (ICP-aligned to N=14 frame)...')
mesh24_pr = load_vertex_colored_mesh(PLY24, transform=T_icp)

sidebyside_paths = []
for vi, (img_info, img_name) in enumerate(zip(chosen_imgs, chosen_names), start=1):
    print(f'Rendering view {vi}: {img_name}')

    cam_data = cams14[img_info['cam_id']]
    pose = build_c2w_opengl(img_info['R'], img_info['tvec'])
    fx, fy, cx, cy = colmap_intrinsics_to_pyrender(cam_data, RENDER_W, RENDER_H)

    color14 = render_mesh(mesh14_pr, pose, fx, fy, cx, cy)
    color24 = render_mesh(mesh24_pr, pose, fx, fy, cx, cy)

    r14_path = OUT_DIR / f'textured_render_n14_view{vi}.png'
    r24_path = OUT_DIR / f'textured_render_n24_view{vi}.png'
    Image.fromarray(color14).save(str(r14_path))
    Image.fromarray(color24).save(str(r24_path))
    print(f'  N14: mean={color14.mean():.1f} std={color14.std():.1f}')
    print(f'  N24: mean={color24.mean():.1f} std={color24.std():.1f}')

    photo_path = IMAGES14 / img_name
    if photo_path.exists():
        photo = np.array(Image.open(photo_path).convert('RGB').resize((RENDER_W, RENDER_H), Image.LANCZOS))
        fig, axes = plt.subplots(1, 3, figsize=(24, 8))
        axes[0].imshow(color14)
        axes[0].set_title(f'N=14 Textured Mesh', fontsize=13, color='steelblue')
        axes[0].axis('off')
        axes[1].imshow(photo)
        axes[1].set_title(f'Source Photo: {Path(img_name).stem}', fontsize=13, color='goldenrod')
        axes[1].axis('off')
        axes[2].imshow(color24)
        axes[2].set_title(f'N=24 Textured Mesh (ICP-aligned)', fontsize=13, color='mediumvioletred')
        axes[2].axis('off')
    else:
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        axes[0].imshow(color14)
        axes[0].set_title('N=14 Textured Mesh', fontsize=13, color='steelblue')
        axes[0].axis('off')
        axes[1].imshow(color24)
        axes[1].set_title('N=24 Textured Mesh (ICP-aligned)', fontsize=13, color='mediumvioletred')
        axes[1].axis('off')

    fig.suptitle(f'Textured Mesh Comparison View {vi}: {Path(img_name).stem}', fontsize=14, fontweight='bold')
    fig.tight_layout()
    sbs_path = OUT_DIR / f'textured_sidebyside_view{vi}.png'
    plt.savefig(str(sbs_path), dpi=100, bbox_inches='tight')
    plt.close()
    sidebyside_paths.append(sbs_path)
    print(f'  Side-by-side: {sbs_path}')

print('\nGenerating 2x2 qualitative grid...')
fig, axes = plt.subplots(2, 2, figsize=(28, 18))
for i, sbs_path in enumerate(sidebyside_paths):
    row, col = i // 2, i % 2
    img = np.array(Image.open(sbs_path))
    axes[row, col].imshow(img)
    axes[row, col].set_title(f'View {i+1}: {Path(chosen_names[i]).stem}', fontsize=12)
    axes[row, col].axis('off')
fig.suptitle('Textured Mesh Comparison: N=14 vs N=24 Photogrammetry (4 Viewpoints)', fontsize=16, fontweight='bold')
plt.tight_layout()
grid_path = OUT_DIR / 'textured_qualitative_grid.png'
plt.savefig(str(grid_path), dpi=80, bbox_inches='tight')
plt.close()
print(f'Saved: {grid_path}')
print('\n===== STAGE: TEXTURED QUALITATIVE COMPARISON COMPLETE =====')
