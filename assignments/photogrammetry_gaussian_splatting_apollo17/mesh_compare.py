import numpy as np
import open3d as o3d
import json
import struct
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm

N14_MESH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense/meshed-poisson.ply')
N24_MESH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n24/dense/meshed-poisson.ply')
OUT_DIR = Path('/home/jatin-satyam/ses598-apollo17/comparison')
OUT_DIR.mkdir(parents=True, exist_ok=True)

N_SAMPLE = 100000
VOXEL_REG = 0.05

print('Loading meshes...')
mesh14 = o3d.io.read_triangle_mesh(str(N14_MESH))
mesh24 = o3d.io.read_triangle_mesh(str(N24_MESH))
print(f'N=14 mesh: {len(mesh14.vertices):,} vertices, {len(mesh14.triangles):,} faces')
print(f'N=24 mesh: {len(mesh24.vertices):,} vertices, {len(mesh24.triangles):,} faces')

print('Sampling point clouds...')
pcd14 = mesh14.sample_points_uniformly(number_of_points=N_SAMPLE)
pcd24 = mesh24.sample_points_uniformly(number_of_points=N_SAMPLE)

print('Computing FPFH features for global registration...')
pcd14_d = pcd14.voxel_down_sample(VOXEL_REG)
pcd24_d = pcd24.voxel_down_sample(VOXEL_REG)

pcd14_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_REG*2, max_nn=30))
pcd24_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_REG*2, max_nn=30))

fpfh14 = o3d.pipelines.registration.compute_fpfh_feature(
    pcd14_d, o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_REG*5, max_nn=100))
fpfh24 = o3d.pipelines.registration.compute_fpfh_feature(
    pcd24_d, o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_REG*5, max_nn=100))

print('RANSAC global registration...')
result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    pcd24_d, pcd14_d, fpfh24, fpfh14,
    mutual_filter=True,
    max_correspondence_distance=VOXEL_REG*2,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    ransac_n=3,
    checkers=[
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(VOXEL_REG*2)
    ],
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
)
print(f'RANSAC fitness: {result_ransac.fitness:.4f}, inlier rmse: {result_ransac.inlier_rmse:.4f}')

print('ICP refinement...')
pcd14_fine = pcd14.voxel_down_sample(0.01)
pcd24_fine = pcd24.voxel_down_sample(0.01)
pcd14_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
pcd24_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

result_icp = o3d.pipelines.registration.registration_icp(
    pcd24_fine, pcd14_fine,
    max_correspondence_distance=0.02,
    init=result_ransac.transformation,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
)
T_icp = result_icp.transformation
print(f'ICP fitness: {result_icp.fitness:.4f}, inlier rmse: {result_icp.inlier_rmse:.6f}')
print(f'ICP transform:\n{T_icp}')

pcd24_aligned = pcd24
pcd24_aligned_t = o3d.geometry.PointCloud(pcd24)
pcd24_aligned_t.transform(T_icp)

pts14 = np.asarray(pcd14.points)
pts24 = np.asarray(pcd24_aligned_t.points)

print('Computing nearest-neighbor distances...')
tree14 = o3d.geometry.KDTreeFlann(pcd14)
tree24 = o3d.geometry.KDTreeFlann(pcd24_aligned_t)

dists_24to14 = np.array([np.sqrt(tree14.search_knn_vector_3d(p, 1)[2][0]) for p in pts24])
dists_14to24 = np.array([np.sqrt(tree24.search_knn_vector_3d(p, 1)[2][0]) for p in pts14])

chamfer = float((dists_24to14.mean() + dists_14to24.mean()) / 2)
hausdorff_p99 = float(np.percentile(np.maximum(dists_24to14, dists_14to24[:len(dists_24to14)]), 99))

pts_all = np.concatenate([pts14, pts24])
bbox_diag = float(np.linalg.norm(pts_all.max(0) - pts_all.min(0)))

def fscore(d1, d2, thresh):
    p = (d2 < thresh).mean()
    r = (d1 < thresh).mean()
    if p + r == 0:
        return 0.0
    return float(2 * p * r / (p + r))

f_scores = {}
for t_frac in [0.01, 0.05, 0.1]:
    t_abs = bbox_diag * t_frac
    f_scores[f'f_score_at_{t_frac}'] = fscore(dists_14to24, dists_24to14, t_abs)
    f_scores[f'threshold_abs_{t_frac}'] = t_abs

metrics = {
    'chamfer_distance': chamfer,
    'hausdorff_p99': hausdorff_p99,
    'bbox_diagonal': bbox_diag,
    'icp_fitness': float(result_icp.fitness),
    'icp_inlier_rmse': float(result_icp.inlier_rmse),
    'icp_transform': T_icp.tolist(),
    'n14_vertices': len(mesh14.vertices),
    'n14_faces': len(mesh14.triangles),
    'n24_vertices': len(mesh24.vertices),
    'n24_faces': len(mesh24.triangles),
}
metrics.update(f_scores)

with open(OUT_DIR / 'quantitative_metrics.json', 'w') as f:
    json.dump(metrics, f, indent=2)
print(f'Saved: {OUT_DIR / "quantitative_metrics.json"}')
print(f'Chamfer: {chamfer:.6f}')
print(f'Hausdorff p99: {hausdorff_p99:.6f}')
for t in [0.01, 0.05, 0.1]:
    print(f'F-score @{t}: {f_scores[f"f_score_at_{t}"]:.4f}')

print('\nGenerating per-vertex distance heatmap...')
mesh24_aligned = o3d.io.read_triangle_mesh(str(N24_MESH))
mesh24_aligned.transform(T_icp)
mesh24_aligned.compute_vertex_normals()

verts24 = np.asarray(mesh24_aligned.vertices)
pcd_vert = o3d.geometry.PointCloud()
pcd_vert.points = o3d.utility.Vector3dVector(verts24)
tree14_full = o3d.geometry.KDTreeFlann(pcd14)

vert_dists = np.array([np.sqrt(tree14_full.search_knn_vector_3d(v, 1)[2][0]) for v in verts24])

d_min_v, d_max_v = np.percentile(vert_dists, 2), np.percentile(vert_dists, 98)
norm_dists = np.clip((vert_dists - d_min_v) / (d_max_v - d_min_v + 1e-10), 0, 1)

cmap = cm.get_cmap('coolwarm')
colors_rgb = cmap(norm_dists)[:, :3]

mesh24_colored = o3d.geometry.TriangleMesh()
mesh24_colored.vertices = mesh24_aligned.vertices
mesh24_colored.triangles = mesh24_aligned.triangles
mesh24_colored.vertex_colors = o3d.utility.Vector3dVector(colors_rgb)
mesh24_colored.compute_vertex_normals()

ply_path = OUT_DIR / 'per_vertex_distance.ply'
o3d.io.write_triangle_mesh(str(ply_path), mesh24_colored)
print(f'Saved: {ply_path}')

fig, ax = plt.subplots(1, 1, figsize=(10, 7))
sc = ax.scatter(verts24[:, 0], verts24[:, 2], c=vert_dists, cmap='coolwarm',
                s=0.5, vmin=d_min_v, vmax=d_max_v, alpha=0.6)
plt.colorbar(sc, ax=ax, label='Distance to N=14 mesh')
ax.set_title('Per-vertex Distance: N=24 mesh to N=14 baseline (top-down view)', fontsize=12)
ax.set_xlabel('X (COLMAP world)')
ax.set_ylabel('Z (COLMAP world)')
ax.set_aspect('equal')
fig.tight_layout()
png_path = OUT_DIR / 'per_vertex_distance.png'
plt.savefig(str(png_path), dpi=120, bbox_inches='tight')
plt.close()
print(f'Saved: {png_path}')

print('\n===== STAGE: MESH COMPARISON COMPLETE =====')
