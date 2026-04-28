import struct
import numpy as np
import cv2
import torch
import torch.nn.functional as F
import open3d as o3d
from pathlib import Path

DEVICE = torch.device('cuda')
torch.backends.cudnn.benchmark = True

DENSE_DIR = Path('/home/jatin-satyam/ses598-apollo17/colmap_n24/dense')
SPARSE_DIR = DENSE_DIR / 'sparse'
IMAGES_DIR = DENSE_DIR / 'images'
OUT_PLY = DENSE_DIR / 'fused.ply'
OUT_MESH = DENSE_DIR / 'meshed-poisson.ply'

N_DEPTHS = 128
N_NEIGHBORS = 6
DEPTH_CHUNK = 24
IMG_SCALE = 0.6

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
            w, x, y, z = qvec
            R = np.array([
                [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
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
            pts[pid] = xyz
    return pts

def get_K(cam, scale=1.0):
    p = cam['params']
    if cam['model'] == 1:
        K = np.array([[p[0]*scale, 0, p[2]*scale],
                      [0, p[1]*scale, p[3]*scale],
                      [0, 0, 1]], dtype=np.float64)
    else:
        K = np.array([[p[0]*scale, 0, p[1]*scale],
                      [0, p[0]*scale, p[2]*scale],
                      [0, 0, 1]], dtype=np.float64)
    return K

def get_depth_range(img_info, pts3d, margin=0.3):
    pt_ids = img_info['pt3d']
    valid_ids = pt_ids[pt_ids >= 0]
    if len(valid_ids) == 0:
        return 1.0, 30.0
    R = img_info['R']
    t = img_info['tvec']
    depths = []
    for pid in valid_ids:
        if pid in pts3d:
            Xw = pts3d[pid]
            Xc = R @ Xw + t
            if Xc[2] > 0:
                depths.append(Xc[2])
    if len(depths) < 5:
        return 1.0, 30.0
    depths = np.array(depths)
    d_min = np.percentile(depths, 5) * (1 - margin)
    d_max = np.percentile(depths, 95) * (1 + margin)
    return max(0.05, d_min), d_max

def select_neighbors(ref_id, images, n=6):
    ref = images[ref_id]
    C_ref = ref['C']
    R_ref = ref['R']
    z_ref = R_ref[2, :]
    cands = []
    for iid, img in images.items():
        if iid == ref_id:
            continue
        C_src = img['C']
        dist = np.linalg.norm(C_src - C_ref)
        if dist < 1e-6:
            continue
        z_src = img['R'][2, :]
        cos_ang = np.clip(np.dot(z_ref, z_src), -1, 1)
        score = dist * (2.0 - cos_ang)
        cands.append((score, iid))
    cands.sort()
    return [iid for _, iid in cands[:n]]

def compute_depth_map(ref_img, src_data, K_ref_s, d_min, d_max, sH, sW):
    ref_small = cv2.resize(ref_img, (sW, sH))
    ref_t = torch.from_numpy(ref_small).float().to(DEVICE).permute(2,0,1).unsqueeze(0) / 255.0

    K_ref_inv = torch.from_numpy(np.linalg.inv(K_ref_s)).float().to(DEVICE)

    u = torch.arange(sW, device=DEVICE).float()
    v = torch.arange(sH, device=DEVICE).float()
    vv, uu = torch.meshgrid(v, u, indexing='ij')
    pixels = torch.stack([uu, vv, torch.ones(sH, sW, device=DEVICE)], dim=-1).reshape(-1, 3)

    depths = torch.linspace(d_min, d_max, N_DEPTHS, device=DEVICE)

    cost_vol = torch.zeros(N_DEPTHS, sH, sW, device=DEVICE)
    count_vol = torch.zeros(N_DEPTHS, sH, sW, device=DEVICE)

    for src_img, K_src_s, R_ref, t_ref, R_src, t_src in src_data:
        src_small = cv2.resize(src_img, (sW, sH))
        src_t = torch.from_numpy(src_small).float().to(DEVICE).permute(2,0,1).unsqueeze(0) / 255.0
        src_H, src_W = sH, sW

        K_src_t = torch.from_numpy(K_src_s).float().to(DEVICE)
        R_ref_t = torch.from_numpy(R_ref).float().to(DEVICE)
        t_ref_t = torch.from_numpy(t_ref).float().to(DEVICE)
        R_src_t = torch.from_numpy(R_src).float().to(DEVICE)
        t_src_t = torch.from_numpy(t_src).float().to(DEVICE)

        R_rel = R_src_t @ R_ref_t.T
        t_rel = t_src_t - R_rel @ t_ref_t

        A = K_src_t @ R_rel @ K_ref_inv
        b_vec = K_src_t @ t_rel

        for start in range(0, N_DEPTHS, DEPTH_CHUNK):
            end = min(start + DEPTH_CHUNK, N_DEPTHS)
            d_chunk = depths[start:end]
            nc = len(d_chunk)

            H_batch = A.unsqueeze(0).expand(nc, -1, -1).clone()
            col_add = b_vec.unsqueeze(0) / d_chunk.unsqueeze(1)
            H_batch = H_batch.clone()
            H_batch[:, :, 2] = H_batch[:, :, 2] + col_add

            proj = torch.einsum('nij,pj->nip', H_batch, pixels)

            z_proj = proj[:, 2, :]
            valid_z = z_proj > 0.01

            proj_xy = proj[:, :2, :] / z_proj.unsqueeze(1).clamp(min=1e-6)
            proj_grid = proj_xy.permute(0, 2, 1).reshape(nc, sH, sW, 2)

            proj_grid_norm = proj_grid.clone()
            proj_grid_norm[..., 0] = proj_grid[..., 0] / (src_W / 2) - 1
            proj_grid_norm[..., 1] = proj_grid[..., 1] / (src_H / 2) - 1

            in_bounds = ((proj_grid_norm[..., 0] >= -1) & (proj_grid_norm[..., 0] <= 1) &
                         (proj_grid_norm[..., 1] >= -1) & (proj_grid_norm[..., 1] <= 1) &
                         valid_z.reshape(nc, sH, sW))

            src_exp = src_t.expand(nc, -1, -1, -1)
            warped = F.grid_sample(src_exp, proj_grid_norm, mode='bilinear',
                                   padding_mode='zeros', align_corners=False)

            ref_exp = ref_t.expand(nc, -1, -1, -1)
            diff = (ref_exp - warped).abs().mean(dim=1)

            cost_vol[start:end] += diff * in_bounds.float()
            count_vol[start:end] += in_bounds.float()

    valid = count_vol > 0
    avg_cost = torch.where(valid, cost_vol / count_vol.clamp(min=1),
                           torch.full_like(cost_vol, 1e6))

    best_idx = avg_cost.argmin(dim=0)
    depth_map = depths[best_idx]
    valid_mask = count_vol.max(dim=0)[0] > 1

    return depth_map.cpu().numpy().astype(np.float32), valid_mask.cpu().numpy()

cameras = read_cameras_bin(SPARSE_DIR / 'cameras.bin')
images = read_images_bin(SPARSE_DIR / 'images.bin')
pts3d = read_points3d_bin(SPARSE_DIR / 'points3D.bin')

print(f'Cameras: {len(cameras)}, Images: {len(images)}, Sparse pts: {len(pts3d)}')

all_pts = []
all_colors = []

depth_maps = {}
valid_masks = {}
ref_poses = {}

for ref_id, ref_info in images.items():
    img_path = IMAGES_DIR / ref_info['name']
    ref_img = cv2.imread(str(img_path))
    if ref_img is None:
        print(f'Could not load {img_path}')
        continue
    ref_img = cv2.cvtColor(ref_img, cv2.COLOR_BGR2RGB)

    H_orig, W_orig = ref_img.shape[:2]
    sW = int(W_orig * IMG_SCALE)
    sH = int(H_orig * IMG_SCALE)

    K_ref = get_K(cameras[ref_info['cam_id']])
    K_ref_s = get_K(cameras[ref_info['cam_id']], scale=IMG_SCALE)

    d_min, d_max = get_depth_range(ref_info, pts3d)
    print(f'\n[{ref_id}] {ref_info["name"]} depth=[{d_min:.2f},{d_max:.2f}]')

    neighbor_ids = select_neighbors(ref_id, images, n=N_NEIGHBORS)
    print(f'  neighbors: {neighbor_ids}')

    src_data = []
    for src_id in neighbor_ids:
        src_info = images[src_id]
        src_path = IMAGES_DIR / src_info['name']
        src_img = cv2.imread(str(src_path))
        if src_img is None:
            continue
        src_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2RGB)
        K_src_s = get_K(cameras[src_info['cam_id']], scale=IMG_SCALE)
        src_data.append((src_img, K_src_s, ref_info['R'], ref_info['tvec'],
                         src_info['R'], src_info['tvec']))

    if not src_data:
        print(f'  No source images, skipping')
        continue

    depth_map, valid_mask = compute_depth_map(ref_img, src_data, K_ref_s, d_min, d_max, sH, sW)
    print(f'  valid pixels: {valid_mask.sum()} / {sH*sW}')

    depth_maps[ref_id] = depth_map
    valid_masks[ref_id] = valid_mask
    ref_poses[ref_id] = (ref_img, K_ref, ref_info['R'], ref_info['tvec'], sH, sW, H_orig, W_orig)

print('\nApplying geometric consistency filtering...')
for ref_id, depth_map in depth_maps.items():
    ref_img, K_ref, R_ref, t_ref, sH, sW, H_orig, W_orig = ref_poses[ref_id]
    valid_mask = valid_masks[ref_id].copy()
    K_ref_s = K_ref.copy()
    K_ref_s[0] *= IMG_SCALE
    K_ref_s[1] *= IMG_SCALE
    K_ref_s[0,2] *= IMG_SCALE
    K_ref_s[1,2] *= IMG_SCALE

    consistent = np.zeros(valid_mask.shape, dtype=np.int32)
    neighbor_ids = select_neighbors(ref_id, images, n=N_NEIGHBORS)

    for src_id in neighbor_ids:
        if src_id not in depth_maps:
            continue
        src_depth = depth_maps[src_id]
        _, _, R_src, t_src, sH_s, sW_s, _, _ = ref_poses[src_id]
        K_src_s = get_K(cameras[images[src_id]['cam_id']], scale=IMG_SCALE)

        v_idx, u_idx = np.where(valid_mask)
        if len(v_idx) == 0:
            continue

        d_vals = depth_map[v_idx, u_idx]
        u_norm = (u_idx.astype(np.float64) - K_ref_s[0,2]) / K_ref_s[0,0]
        v_norm = (v_idx.astype(np.float64) - K_ref_s[1,2]) / K_ref_s[1,1]

        Xc = np.stack([u_norm * d_vals, v_norm * d_vals, d_vals], axis=1)
        Xw = (R_ref.T @ (Xc.T - t_ref.reshape(3,1))).T
        Xc_src = (R_src @ Xw.T + t_src.reshape(3,1)).T

        valid_z = Xc_src[:, 2] > 0
        u_src = K_src_s[0,0] * Xc_src[:,0] / Xc_src[:,2].clip(1e-6) + K_src_s[0,2]
        v_src = K_src_s[1,1] * Xc_src[:,1] / Xc_src[:,2].clip(1e-6) + K_src_s[1,2]

        u_src_i = np.round(u_src).astype(int)
        v_src_i = np.round(v_src).astype(int)

        in_img = (u_src_i >= 0) & (u_src_i < sW_s) & (v_src_i >= 0) & (v_src_i < sH_s)
        valid_proj = valid_z & in_img

        for idx in np.where(valid_proj)[0]:
            d_src = src_depth[v_src_i[idx], u_src_i[idx]]
            if d_src > 0:
                d_reproj = Xc_src[idx, 2]
                if abs(d_reproj - d_src) / (d_src + 1e-6) < 0.1:
                    consistent[v_idx[idx], u_idx[idx]] += 1

    final_mask = valid_mask & (consistent >= 2)
    print(f'  [{ref_id}] consistency: {valid_mask.sum()} -> {final_mask.sum()} pts')
    valid_masks[ref_id] = final_mask

print('\nBackprojecting depth maps to 3D...')
for ref_id, depth_map in depth_maps.items():
    ref_img, K_ref, R_ref, t_ref, sH, sW, H_orig, W_orig = ref_poses[ref_id]
    valid_mask = valid_masks[ref_id]

    K_ref_s = K_ref.copy()
    K_ref_s[0] *= IMG_SCALE
    K_ref_s[1] *= IMG_SCALE
    K_ref_s[0,2] *= IMG_SCALE
    K_ref_s[1,2] *= IMG_SCALE

    v_idx, u_idx = np.where(valid_mask)
    if len(v_idx) == 0:
        continue

    d_vals = depth_map[v_idx, u_idx].astype(np.float64)
    u_norm = (u_idx.astype(np.float64) - K_ref_s[0,2]) / K_ref_s[0,0]
    v_norm = (v_idx.astype(np.float64) - K_ref_s[1,2]) / K_ref_s[1,1]

    Xc = np.stack([u_norm * d_vals, v_norm * d_vals, d_vals], axis=1)
    Xw = (R_ref.T @ (Xc.T - t_ref.reshape(3,1))).T

    ref_img_small = cv2.resize(ref_img, (sW, sH))
    colors = ref_img_small[v_idx, u_idx].astype(np.float32) / 255.0

    all_pts.append(Xw.astype(np.float32))
    all_colors.append(colors)
    print(f'  [{ref_id}] contributed {len(Xw)} pts')

pts_all = np.concatenate(all_pts, axis=0)
colors_all = np.concatenate(all_colors, axis=0)
print(f'\nTotal merged pts before filtering: {len(pts_all):,}')

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_all)
pcd.colors = o3d.utility.Vector3dVector(colors_all)

print('Statistical outlier removal...')
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
print(f'After outlier removal: {len(pcd.points):,}')

voxel_size = 0.0015
pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
print(f'After voxel downsample (voxel={voxel_size}): {len(pcd_down.points):,}')

if len(pcd_down.points) < 500000:
    voxel_size = 0.0010
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f'Retry voxel={voxel_size}: {len(pcd_down.points):,}')

print(f'Saving fused.ply ({len(pcd_down.points):,} points)...')
o3d.io.write_point_cloud(str(OUT_PLY), pcd_down)
print(f'Saved: {OUT_PLY}')

print('\nEstimating normals...')
pcd_down.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd_down.orient_normals_consistent_tangent_plane(100)

print('Poisson reconstruction (depth=10)...')
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_down, depth=10)
print(f'Raw mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')

densities = np.asarray(densities)
density_thresh = np.percentile(densities, 5)
vertices_to_remove = densities < density_thresh
mesh.remove_vertices_by_mask(vertices_to_remove)
print(f'Cleaned mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')

if len(mesh.vertices) < 100000:
    print('WARNING: mesh has fewer than 100k vertices, retrying with depth=9')
    mesh2, densities2 = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_down, depth=9)
    densities2 = np.asarray(densities2)
    thresh2 = np.percentile(densities2, 3)
    mesh2.remove_vertices_by_mask(np.asarray(densities2) < thresh2)
    print(f'depth=9 mesh: {len(mesh2.vertices):,} vertices, {len(mesh2.triangles):,} faces')
    if len(mesh2.vertices) > len(mesh.vertices):
        mesh = mesh2

mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(OUT_MESH), mesh)
print(f'\nSaved mesh: {OUT_MESH}')
print(f'Final vertices: {len(mesh.vertices):,}  faces: {len(mesh.triangles):,}')
print(f'fused.ply size: {OUT_PLY.stat().st_size / 1e6:.1f} MB')
print(f'meshed-poisson.ply size: {OUT_MESH.stat().st_size / 1e6:.1f} MB')
print('\n===== STAGE: DENSE RECONSTRUCTION N=24 COMPLETE =====')
