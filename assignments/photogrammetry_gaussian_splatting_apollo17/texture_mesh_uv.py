import numpy as np
import open3d as o3d
import cv2
import xatlas
import struct
import json
import time
from pathlib import Path
from PIL import Image

ATLAS_SIZE = 4096
MAX_FACES_XATLAS = 2500000

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
        cam_data = raw_cams[img['cam_id']]
        p = cam_data['params']
        model = cam_data['model']
        if model == 1:
            fx, fy, cx, cy = p[0], p[1], p[2], p[3]
            k = 0.0
            f = fx
        elif model == 2:
            f, cx, cy, k = p[0], p[1], p[2], p[3]
        else:
            f = p[0]; cx = p[1]; cy = p[2]; k = 0.0
        img_path = images_dir / img['name']
        if not img_path.exists():
            print(f'  Warning: image not found: {img_path}')
            continue
        src_img = np.array(Image.open(img_path).convert('RGB'), dtype=np.uint8)
        cameras.append(dict(
            name=img['name'],
            R=img['R'],
            t=img['tvec'],
            pos=img['pos'],
            f=f, cx=cx, cy=cy, k=k,
            W=cam_data['W'], H=cam_data['H'],
            img=src_img,
            model=model
        ))
    print(f'  Loaded {len(cameras)} cameras with images')
    return cameras

def project_pts(pts3d, cam):
    Xc = (cam['R'] @ pts3d.T + cam['t'][:, None]).T
    valid = Xc[:, 2] > 0.1
    eps = 1e-8
    xn = Xc[:, 0] / Xc[:, 2].clip(eps)
    yn = Xc[:, 1] / Xc[:, 2].clip(eps)
    if cam['k'] != 0.0:
        r2 = xn**2 + yn**2
        scale = 1.0 + cam['k'] * r2
        xd = xn * scale
        yd = yn * scale
    else:
        xd, yd = xn, yn
    u = cam['f'] * xd + cam['cx']
    v = cam['f'] * yd + cam['cy']
    in_img = valid & (u >= 0) & (u < cam['W']) & (v >= 0) & (v < cam['H'])
    return u, v, in_img

def bilinear_sample(img, u_px, v_px):
    H, W = img.shape[:2]
    u0 = np.floor(u_px).astype(np.int32).clip(0, W - 2)
    v0 = np.floor(v_px).astype(np.int32).clip(0, H - 2)
    fu = (u_px - u0).astype(np.float32)
    fv = (v_px - v0).astype(np.float32)
    c00 = img[v0, u0].astype(np.float32)
    c01 = img[v0, u0 + 1].astype(np.float32)
    c10 = img[v0 + 1, u0].astype(np.float32)
    c11 = img[v0 + 1, u0 + 1].astype(np.float32)
    fu = fu[:, None]; fv = fv[:, None]
    return (1-fu)*(1-fv)*c00 + fu*(1-fv)*c01 + (1-fu)*fv*c10 + fu*fv*c11

def assign_cameras(centroids, tri_norms, cameras, scene):
    N = len(centroids)
    N_cams = len(cameras)
    cam_positions = np.array([c['pos'] for c in cameras], dtype=np.float64)

    vecs = cam_positions[None] - centroids[:, None]
    dists = np.linalg.norm(vecs, axis=2).clip(1e-10)
    vecs_norm = vecs / dists[:, :, None]
    cos_a = (vecs_norm * tri_norms[:, None]).sum(axis=2)

    in_fov = np.zeros((N, N_cams), dtype=bool)
    for ci, cam in enumerate(cameras):
        _, _, valid = project_pts(centroids, cam)
        in_fov[:, ci] = valid

    scores = np.where(in_fov & (cos_a > 0), cos_a / dists**2, -1.0)
    sorted_cams = np.argsort(-scores, axis=1)
    cam_per_tri = np.full(N, -1, dtype=np.int32)

    for rank in range(min(5, N_cams)):
        need = cam_per_tri == -1
        if not need.any():
            break
        cand = sorted_cams[need, rank]
        valid_cand = scores[need, cand] > 0
        if not valid_cand.any():
            continue
        check_idx = np.where(need)[0][valid_cand]
        check_cam = cand[valid_cand]

        origins = centroids[check_idx].astype(np.float32)
        targets = cam_positions[check_cam].astype(np.float32)
        raw_dirs = targets - origins
        raw_dists = np.linalg.norm(raw_dirs, axis=1, keepdims=True).clip(1e-10)
        dirs_n = (raw_dirs / raw_dists).astype(np.float32)
        offset_o = origins + dirs_n * 0.005

        rays = np.concatenate([offset_o, dirs_n], axis=1)
        rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
        hit = scene.cast_rays(rays_t)
        hit_d = hit['t_hit'].numpy()
        unoccluded = hit_d >= raw_dists.flatten() * 0.98

        cam_per_tri[check_idx[unoccluded]] = check_cam[unoccluded]

    still_none = cam_per_tri == -1
    if still_none.any():
        best = np.argmax(scores[still_none], axis=1)
        cam_per_tri[still_none] = best

    return cam_per_tri

def save_obj_mtl(out_dir, vmapping, verts, xatlas_tris, uvs, atlas_name):
    out_dir = Path(out_dir)
    obj_path = out_dir / 'textured.obj'
    mtl_path = out_dir / 'textured.mtl'

    with open(mtl_path, 'w') as f:
        f.write('newmtl atlas_mat\n')
        f.write(f'map_Kd {atlas_name}\n')
        f.write('Kd 1.0 1.0 1.0\n')
        f.write('Ka 0.0 0.0 0.0\n')
        f.write('Ks 0.0 0.0 0.0\n')

    N_out = len(vmapping)
    out_verts = verts[vmapping]

    chunk = 500000
    with open(obj_path, 'w') as f:
        f.write(f'mtllib textured.mtl\n')
        for i in range(0, N_out, chunk):
            block = out_verts[i:i+chunk]
            lines = '\n'.join(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}' for v in block)
            f.write(lines + '\n')
        for i in range(0, N_out, chunk):
            block = uvs[i:i+chunk]
            lines = '\n'.join(f'vt {u:.6f} {v:.6f}' for u, v in block)
            f.write(lines + '\n')
        f.write('usemtl atlas_mat\n')
        for i in range(0, len(xatlas_tris), chunk):
            block = xatlas_tris[i:i+chunk]
            lines = '\n'.join(
                f'f {v0+1}/{v0+1} {v1+1}/{v1+1} {v2+1}/{v2+1}'
                for v0, v1, v2 in block
            )
            f.write(lines + '\n')

    print(f'  Saved OBJ: {obj_path}  ({obj_path.stat().st_size/1e6:.1f} MB)')
    print(f'  Saved MTL: {mtl_path}')

def texture_mesh(label, mesh_path, sparse_dir, images_dir, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'\n{"="*60}')
    print(f'Texturing {label}: {mesh_path}')

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    mesh.compute_vertex_normals()
    verts_in = np.asarray(mesh.vertices, dtype=np.float64)
    tris_in = np.asarray(mesh.triangles, dtype=np.int32)
    orig_n_verts = len(verts_in)
    orig_n_faces = len(tris_in)
    print(f'  Input mesh: {orig_n_verts:,} verts, {orig_n_faces:,} faces')

    decimate_factor = 1.0
    if len(tris_in) > MAX_FACES_XATLAS:
        decimate_factor = MAX_FACES_XATLAS / len(tris_in)
        print(f'  Decimating to ~{MAX_FACES_XATLAS:,} faces (factor={decimate_factor:.4f})')
        mesh = mesh.simplify_quadric_decimation(MAX_FACES_XATLAS)
        mesh.compute_vertex_normals()
        verts_in = np.asarray(mesh.vertices, dtype=np.float64)
        tris_in = np.asarray(mesh.triangles, dtype=np.int32)
        print(f'  After decimation: {len(verts_in):,} verts, {len(tris_in):,} faces')

    print('  Loading cameras...')
    cameras = build_cameras(sparse_dir, images_dir)

    print('  Building raycasting scene...')
    scene = o3d.t.geometry.RaycastingScene()
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene.add_triangles(mesh_t)

    v0c = verts_in[tris_in[:, 0]]
    v1c = verts_in[tris_in[:, 1]]
    v2c = verts_in[tris_in[:, 2]]
    centroids = (v0c + v1c + v2c) / 3.0
    cross = np.cross(v1c - v0c, v2c - v0c)
    nlen = np.linalg.norm(cross, axis=1, keepdims=True).clip(1e-10)
    tri_norms = cross / nlen

    print('  Assigning cameras to triangles...')
    t0 = time.time()
    cam_per_tri = assign_cameras(centroids, tri_norms, cameras, scene)
    print(f'  Camera assignment: {time.time()-t0:.1f}s  unassigned={( cam_per_tri==-1).sum()}')

    print('  xatlas UV parametrization...')
    t0 = time.time()
    vmapping, xatlas_tris, uvs = xatlas.parametrize(
        verts_in.astype(np.float32), tris_in.astype(np.uint32)
    )
    print(f'  xatlas: {len(vmapping):,} output verts, {time.time()-t0:.1f}s')
    print(f'  UV range: [{uvs.min():.4f}, {uvs.max():.4f}]')

    H = W = ATLAS_SIZE
    uvs_px = uvs * np.array([W, H], dtype=np.float32)
    tri_map = np.full((H, W), -1, dtype=np.int32)
    bary_u = np.zeros((H, W), dtype=np.float32)
    bary_v = np.zeros((H, W), dtype=np.float32)

    print(f'  Rasterizing {len(xatlas_tris):,} triangles to {W}x{H} atlas...')
    t0 = time.time()
    N_tris = len(xatlas_tris)
    for ti in range(N_tris):
        if ti % 300000 == 0 and ti > 0:
            elapsed = time.time() - t0
            eta = elapsed / ti * (N_tris - ti)
            print(f'    {ti:,}/{N_tris:,}  elapsed={elapsed:.0f}s  eta={eta:.0f}s')

        i0, i1, i2 = xatlas_tris[ti]
        p0 = uvs_px[i0]; p1 = uvs_px[i1]; p2 = uvs_px[i2]

        x_lo = max(0, int(np.floor(min(p0[0], p1[0], p2[0]))))
        x_hi = min(W - 1, int(np.ceil(max(p0[0], p1[0], p2[0]))))
        y_lo = max(0, int(np.floor(min(p0[1], p1[1], p2[1]))))
        y_hi = min(H - 1, int(np.ceil(max(p0[1], p1[1], p2[1]))))
        if x_lo > x_hi or y_lo > y_hi:
            continue

        xs = np.arange(x_lo, x_hi + 1, dtype=np.float32) + 0.5
        ys = np.arange(y_lo, y_hi + 1, dtype=np.float32) + 0.5
        xx, yy = np.meshgrid(xs, ys)
        pts = np.stack([xx.ravel(), yy.ravel()], axis=1)

        e01 = p1 - p0; e02 = p2 - p0
        d00 = float(np.dot(e01, e01)); d01 = float(np.dot(e01, e02)); d11 = float(np.dot(e02, e02))
        det = d00 * d11 - d01 * d01
        if abs(det) < 1e-10:
            continue
        inv = 1.0 / det
        vp = pts - p0
        dp0 = vp @ e01; dp1 = vp @ e02
        u_b = (d11 * dp0 - d01 * dp1) * inv
        v_b = (d00 * dp1 - d01 * dp0) * inv
        inside = (u_b >= 0) & (v_b >= 0) & (u_b + v_b <= 1.0)
        if not inside.any():
            continue

        px = xx.ravel()[inside].astype(np.int32).clip(0, W - 1)
        py = yy.ravel()[inside].astype(np.int32).clip(0, H - 1)
        tri_map[py, px] = ti
        bary_u[py, px] = u_b[inside]
        bary_v[py, px] = v_b[inside]

    print(f'  Rasterization: {time.time()-t0:.1f}s')

    valid_y, valid_x = np.where(tri_map >= 0)
    valid_tris = tri_map[valid_y, valid_x]
    u_b_vals = bary_u[valid_y, valid_x]
    v_b_vals = bary_v[valid_y, valid_x]
    w_b_vals = 1.0 - u_b_vals - v_b_vals

    xv0 = xatlas_tris[valid_tris, 0]; xv1 = xatlas_tris[valid_tris, 1]; xv2 = xatlas_tris[valid_tris, 2]
    pos3d = (w_b_vals[:, None] * verts_in[vmapping[xv0]] +
             u_b_vals[:, None] * verts_in[vmapping[xv1]] +
             v_b_vals[:, None] * verts_in[vmapping[xv2]])

    pixel_cams = cam_per_tri[valid_tris]
    atlas = np.full((H, W, 3), 128, dtype=np.uint8)

    print(f'  Baking {len(valid_y):,} valid atlas pixels across {len(cameras)} cameras...')
    t0 = time.time()
    covered = 0
    for ci, cam in enumerate(cameras):
        mask = pixel_cams == ci
        if not mask.any():
            continue
        u_px, v_px, in_img = project_pts(pos3d[mask], cam)
        sampled = bilinear_sample(cam['img'], u_px[in_img], v_px[in_img])
        ay = valid_y[mask][in_img]
        ax = valid_x[mask][in_img]
        atlas[ay, ax] = sampled.clip(0, 255).astype(np.uint8)
        covered += in_img.sum()

    coverage = covered / max(1, len(valid_y))
    print(f'  Baking: {time.time()-t0:.1f}s  coverage={coverage:.1%}')

    print('  Inpainting atlas holes...')
    fill_mask = (tri_map < 0).astype(np.uint8) * 255
    atlas = cv2.inpaint(atlas, fill_mask, 3, cv2.INPAINT_TELEA)

    atlas_path = out_dir / 'texture_atlas.png'
    Image.fromarray(atlas).save(str(atlas_path))
    print(f'  Saved atlas: {atlas_path}  ({atlas_path.stat().st_size/1e6:.1f} MB)')

    save_obj_mtl(out_dir, vmapping, verts_in, xatlas_tris, uvs, 'texture_atlas.png')

    summary = dict(
        label=label,
        tier='A',
        format='UV-textured OBJ + 4096x4096 atlas PNG',
        atlas_resolution=ATLAS_SIZE,
        input_vertex_count=orig_n_verts,
        input_face_count=orig_n_faces,
        decimation_factor=round(decimate_factor, 4),
        output_vertex_count=int(len(vmapping)),
        output_face_count=int(len(xatlas_tris)),
        coverage_rate=round(float(coverage), 4),
        out_dir=str(out_dir),
        files=['textured.obj', 'textured.mtl', 'texture_atlas.png']
    )
    summary_path = out_dir / 'summary.json'
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'  Summary: {summary_path}')
    print(f'  Done: {label}')
    return summary

t_start = time.time()

summary_n14 = texture_mesh(
    'N=14',
    'colmap_n15/dense/meshed-poisson.ply',
    'colmap_n15/sparse/0',
    'data/rgb',
    'colmap_n15/dense/textured'
)

summary_n24 = texture_mesh(
    'N=24',
    'colmap_n24/dense/meshed-poisson.ply',
    'colmap_n24/sparse/0',
    'data/rgb_n24',
    'colmap_n24/dense/textured'
)

combined = dict(
    tier='A',
    output_format='UV-textured OBJ + atlas PNG',
    atlas_resolution=ATLAS_SIZE,
    meshes=[summary_n14, summary_n24]
)
with open('comparison/textured_summary.json', 'w') as f:
    json.dump(combined, f, indent=2)
print(json.dumps(combined, indent=2))
print(f'\nTotal time: {(time.time()-t_start)/60:.1f} minutes')
print('\n===== STAGE: UV TEXTURING COMPLETE =====')
