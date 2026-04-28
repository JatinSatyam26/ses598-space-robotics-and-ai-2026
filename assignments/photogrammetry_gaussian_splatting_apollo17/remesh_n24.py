import numpy as np
import open3d as o3d
from pathlib import Path

IN_PLY = Path('/home/jatin-satyam/ses598-apollo17/colmap_n24/dense/fused.ply')
OUT_MESH = Path('/home/jatin-satyam/ses598-apollo17/colmap_n24/dense/meshed-poisson.ply')

print('Loading fused.ply...')
pcd = o3d.io.read_point_cloud(str(IN_PLY))
pts = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)
print(f'Total points: {len(pts):,}')
print(f'Bounding box: min={pts.min(0).round(3)}, max={pts.max(0).round(3)}')

c137 = np.array([-3.07, -0.12, -1.70])
c138 = np.array([4.13, 0.16, 3.10])
d137 = np.linalg.norm(pts - c137, axis=1)
d138 = np.linalg.norm(pts - c138, axis=1)
d_min = np.minimum(d137, d138)

mask = d_min < 10.0
pts_clean = pts[mask]
colors_clean = colors[mask]
print(f'After cluster-proximity filter (<10 units): {len(pts_clean):,} points')

pcd_clean = o3d.geometry.PointCloud()
pcd_clean.points = o3d.utility.Vector3dVector(pts_clean)
pcd_clean.colors = o3d.utility.Vector3dVector(colors_clean)

print('Statistical outlier removal on filtered cloud...')
pcd_clean, _ = pcd_clean.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
print(f'After outlier removal: {len(pcd_clean.points):,}')

print('Estimating normals...')
pcd_clean.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
pcd_clean.orient_normals_consistent_tangent_plane(100)

for depth in [10, 11]:
    print(f'\nPoisson reconstruction (depth={depth})...')
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_clean, depth=depth)
    print(f'Raw mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')
    densities = np.asarray(densities)
    thresh = np.percentile(densities, 3)
    mesh.remove_vertices_by_mask(densities < thresh)
    print(f'Cleaned mesh (3% density threshold): {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces')
    if len(mesh.vertices) >= 100000:
        print(f'Target met at depth={depth}')
        break

mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(OUT_MESH), mesh)
print(f'\nSaved: {OUT_MESH}')
print(f'Final vertices: {len(mesh.vertices):,}  faces: {len(mesh.triangles):,}')
print(f'File size: {OUT_MESH.stat().st_size / 1e6:.1f} MB')
print('\n===== REMESH COMPLETE =====')
