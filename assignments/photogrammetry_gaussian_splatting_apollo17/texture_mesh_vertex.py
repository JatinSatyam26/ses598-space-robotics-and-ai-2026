import numpy as np
import open3d as o3d
import struct
import json
import time
from pathlib import Path
from PIL import Image

def read_cameras_bin(path):
    cameras = {}
    nparams = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8}
    with open(path, 'rb') as f:
        n = struct.unpack('<Q', f.read(8))[0]
        for _ in range(n):
            cid = struct.unpack('<I', f.read(4))[0]
            mid = struct.unpack('<I', f.read(4))[0]
            w = struct.unpack('<Q', f.read(8))[0]
            h = struct.unpack('<Q', f.read(8))[0]
            np_ = nparams[mid]
            params = list(struct.unpack(f'<{np_}d', f.read(8 * np_)))
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
            images[iid] = dict(name=name.decode(), cam_id=cid, R=R, tvec=tvec, pos=-R.T @ tvec)
    return images

def build_cameras(sparse_dir, images_dir):
    sparse_dir = Path(sparse_dir)
    images_dir = Path(images_dir)
    raw_cams = read_cameras_bin(sparse_dir / 'cameras.bin')
    raw_imgs = read_images_bin(sparse_dir / 'images.bin')
    cameras = []
    for iid, img in raw_imgs.items():
        cam = raw_cams[img['cam_id']]
        p = cam['params']
        m = cam['model']
        if m == 1:
            f = p[0]; cx = p[2]; cy = p[3]; k = 0.0
        elif m == 2:
            f = p[0]; cx = p[1]; cy = p[2]; k = p[3]
        else:
            f = p[0]; cx = p[1]; cy = p[2]; k = 0.0
        img_path = images_dir / img['name']
        if not img_path.exists():
            continue
        src = np.array(Image.open(img_path).convert('RGB'), dtype=np.uint8)
        cameras.append(dict(
            name=img['name'], R=img['R'], t=img['tvec'], pos=img['pos'],
            f=f, cx=cx, cy=cy, k=k, W=cam['W'], H=cam['H'], img=src, model=m
        ))
    return cameras

def project_pts(pts3d, cam):
    Xc = (cam['R'] @ pts3d.T + cam['t'][:, None]).T
    valid = Xc[:, 2] > 0.1
    eps = 1e-8
    xn = Xc[:, 0] / Xc[:, 2].clip(eps)
    yn = Xc[:, 1] / Xc[:, 2].clip(eps)
    if cam['k'] != 0.0:
        r2 = xn**2 + yn**2
        sc = 1.0 + cam['k'] * r2
        xd = xn * sc; yd = yn * sc
    else:
        xd = xn; yd = yn
    u = cam['f'] * xd + cam['cx']
    v = cam['f'] * yd + cam['cy']
    in_img = valid & (u >= 0) & (u < cam['W']) & (v >= 0) & (v < cam['H'])
    return u, v, in_img, Xc[:, 2]

def bilinear_sample(img, u_px, v_px):
    H, W = img.shape[:2]
    u0 = np.floor(u_px).astype(np.int32).clip(0, W - 2)
    v0 = np.floor(v_px).astype(np.int32).clip(0, H - 2)
    fu = (u_px - u0).astype(np.float32)[:, None]
    fv = (v_px - v0).astype(np.float32)[:, None]
    c00 = img[v0, u0].astype(np.float32)
    c01 = img[v0, u0 + 1].astype(np.float32)
    c10 = img[v0 + 1, u0].astype(np.float32)
    c11 = img[v0 + 1, u0 + 1].astype(np.float32)
    return (1-fu)*(1-fv)*c00 + fu*(1-fv)*c01 + (1-fu)*fv*c10 + fu*fv*c11

def color_vertices(mesh, cameras, scene, batch=200000):
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
    N = len(verts)

    accum_color = np.zeros((N, 3), dtype=np.float64)
    accum_weight = np.zeros(N, dtype=np.float64)

    cam_positions = np.array([c['pos'] for c in cameras], dtype=np.float64)

    for start in range(0, N, batch):
        end = min(start + batch, N)
        vb = verts[start:end]
        nb = normals[start:end]
        M = end - start

        if start % 500000 == 0:
            print(f'  Vertex {start:,}/{N:,}')

        for ci, cam in enumerate(cameras):
            u_px, v_px, in_img, depths = project_pts(vb, cam)

            if not in_img.any():
                continue

            cam_dir = cam['pos'] - vb
            dists = np.linalg.norm(cam_dir, axis=1).clip(1e-10)
            cam_dir_n = cam_dir / dists[:, None]
            cos_a = (cam_dir_n * nb).sum(axis=1)

            good = in_img & (cos_a > 0)
            if not good.any():
                continue

            check_idx = np.where(good)[0]
            origins = vb[check_idx].astype(np.float32)
            dirs = (cam['pos'] - origins).astype(np.float32)
            d_to_cam = np.linalg.norm(dirs, axis=1, keepdims=True).clip(1e-10)
            dirs_n = (dirs / d_to_cam).astype(np.float32)
            offset_o = origins + dirs_n * 0.005

            rays = np.concatenate([offset_o, dirs_n], axis=1)
            rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
            hit = scene.cast_rays(rays_t)
            hit_d = hit['t_hit'].numpy()
            unoccluded = hit_d >= d_to_cam.flatten() * 0.98

            if not unoccluded.any():
                continue

            final_idx = check_idx[unoccluded]
            w = cos_a[final_idx] / dists[final_idx]**2
            sampled = bilinear_sample(cam['img'], u_px[final_idx], v_px[final_idx])

            global_idx = np.arange(start, end)[final_idx]
            np.add.at(accum_color, global_idx, sampled * w[:, None])
            np.add.at(accum_weight, global_idx, w)

    has_color = accum_weight > 0
    vertex_colors = np.zeros((N, 3), dtype=np.float32)
    vertex_colors[has_color] = (accum_color[has_color] / accum_weight[has_color, None]).clip(0, 255) / 255.0
    vertex_colors[~has_color] = 0.5

    coverage = has_color.mean()
    return vertex_colors, coverage

def texture_mesh_vertex(label, mesh_path, sparse_dir, images_dir, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'\n{"="*60}')
    print(f'Vertex-color texturing {label}: {mesh_path}')

    t0 = time.time()
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    mesh.compute_vertex_normals()
    verts = np.asarray(mesh.vertices)
    N_verts = len(verts)
    N_faces = len(np.asarray(mesh.triangles))
    print(f'  Mesh: {N_verts:,} verts, {N_faces:,} faces')

    cameras = build_cameras(sparse_dir, images_dir)
    print(f'  {len(cameras)} cameras loaded')

    print('  Building raycasting scene...')
    scene = o3d.t.geometry.RaycastingScene()
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene.add_triangles(mesh_t)

    print('  Computing vertex colors...')
    vertex_colors, coverage = color_vertices(mesh, cameras, scene)
    print(f'  Coverage: {coverage:.1%}  time: {time.time()-t0:.1f}s')

    mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
    out_ply = out_dir / 'textured.ply'
    o3d.io.write_triangle_mesh(str(out_ply), mesh)
    print(f'  Saved: {out_ply}  ({out_ply.stat().st_size/1e6:.1f} MB)')

    summary = dict(
        label=label,
        tier='B',
        format='per-vertex-colored PLY',
        output_vertex_count=N_verts,
        output_face_count=N_faces,
        coverage_rate=round(float(coverage), 4),
        out_dir=str(out_dir),
        files=['textured.ply']
    )
    with open(out_dir / 'summary.json', 'w') as f:
        json.dump(summary, f, indent=2)
    return summary

t_all = time.time()

summary_n14 = texture_mesh_vertex(
    'N=14',
    'colmap_n15/dense/meshed-poisson.ply',
    'colmap_n15/sparse/0',
    'data/rgb',
    'colmap_n15/dense/textured'
)

summary_n24 = texture_mesh_vertex(
    'N=24',
    'colmap_n24/dense/meshed-poisson.ply',
    'colmap_n24/sparse/0',
    'data/rgb_n24',
    'colmap_n24/dense/textured'
)

combined = dict(
    tier='B',
    output_format='per-vertex-colored PLY',
    note='Per-vertex RGB coloring. Recognized as textured mesh in photogrammetry tools (Metashape, CloudCompare). Chosen because xatlas UV parametrization exceeded 45-minute per-mesh limit on 2.1M-face input.',
    meshes=[summary_n14, summary_n24]
)

with open('comparison/textured_summary.json', 'w') as f:
    json.dump(combined, f, indent=2)
print(json.dumps(combined, indent=2))
print(f'\nTotal time: {(time.time()-t_all)/60:.1f} minutes')
print('\n===== STAGE: VERTEX TEXTURING COMPLETE =====')
