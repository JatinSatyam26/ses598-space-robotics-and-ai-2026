import numpy as np
import open3d as o3d
from pathlib import Path

DENSE_DIR = Path('/home/jatin-satyam/ses598-apollo17/colmap_n15/dense')
OUT_PLY = DENSE_DIR / 'fused.ply'
OUT_MESH = DENSE_DIR / 'meshed-poisson.ply'

pcd_raw = o3d.io.read_point_cloud(str(OUT_PLY))
pts = np.asarray(pcd_raw.points)
colors = np.asarray(pcd_raw.colors)

print(f'Raw cloud: {len(pts):,} points')
print(f'  X: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}]')
print(f'  Y: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}]')
print(f'  Z: [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]')

x_min, x_max = -7.5, 1.0
y_min, y_max = -2.5, 4.5
z_min, z_max = 0.5, 11.0

mask = ((pts[:,0] >= x_min) & (pts[:,0] <= x_max) &
        (pts[:,1] >= y_min) & (pts[:,1] <= y_max) &
        (pts[:,2] >= z_min) & (pts[:,2] <= z_max))

pts_crop = pts[mask]
colors_crop = colors[mask]
print(f'\nAfter crop [{x_min},{x_max}]x[{y_min},{y_max}]x[{z_min},{z_max}]:')
print(f'  {len(pts_crop):,} points ({100*mask.mean():.1f}%)')

pcd_crop = o3d.geometry.PointCloud()
pcd_crop.points = o3d.utility.Vector3dVector(pts_crop)
pcd_crop.colors = o3d.utility.Vector3dVector(colors_crop)

print('\nStatistical outlier removal on cropped cloud...')
pcd_crop, _ = pcd_crop.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
print(f'After outlier removal: {len(pcd_crop.points):,}')

print('Saving updated fused.ply...')
o3d.io.write_point_cloud(str(OUT_PLY), pcd_crop)
print(f'Saved fused.ply: {len(pcd_crop.points):,} points')

print('\nEstimating normals...')
pcd_crop.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=50))
pcd_crop.orient_normals_consistent_tangent_plane(100)

for depth in [11, 10, 9]:
    print(f'\nPoisson reconstruction depth={depth}...')
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_crop, depth=depth)
    print(f'  Raw mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')
    densities_np = np.asarray(densities)
    thresh_pct = 5
    thresh = np.percentile(densities_np, thresh_pct)
    mesh.remove_vertices_by_mask(densities_np < thresh)
    print(f'  After pruning p{thresh_pct}: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')
    if len(mesh.vertices) >= 100000:
        break

mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(OUT_MESH), mesh)

ply_size = OUT_PLY.stat().st_size
mesh_size = OUT_MESH.stat().st_size
print(f'\nFinal fused.ply: {len(pcd_crop.points):,} points, {ply_size/1e6:.1f} MB')
print(f'Final meshed-poisson.ply: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces, {mesh_size/1e6:.1f} MB')
