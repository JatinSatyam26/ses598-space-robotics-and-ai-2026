# Part B Phase 2 Report: Augmented Photogrammetry (N=24)

Generated: 2026-04-25

---

## 1. N=24 Dataset Construction

### Image Source Breakdown

| Source | Count | Images |
|--------|-------|--------|
| mag137 originals | 7 | AS17-137-20903HR … 20909HR |
| mag138 originals | 7 | AS17-138-21030HR … 21035HR, 21037HR (21036 excluded) |
| mag137 novel views | 5 | novel_mag137_01 … 05 |
| mag138 novel views | 5 | novel_mag138_01 … 05 |
| **Total** | **24** | |

Directory: `data/rgb_n24/mag137/` (12 files), `data/rgb_n24/mag138/` (12 files)

### Resize Decision

Novel views were rendered at 2000×2000 px. Originals are 2340×2364 (mag137) and 2340×2345 (mag138). Option (a) was chosen: upscale novel views to per-magazine dimensions using PIL.Image.LANCZOS.

**Rationale:** `single_camera_per_folder=1` requires uniform dimensions within each folder and allows sharing intrinsics between all images in a folder. Since the novel views were rendered through the same splat model that was trained on the originals, they share the same effective focal length. Sharing intrinsics is physically more faithful than giving each image separate intrinsics (option c), which would add 10×3=30 free parameters with only 24 images — an under-constrained problem. Option (b) would discard original resolution. The LANCZOS upscaling of novel views introduces mild blurring at high spatial frequencies but preserves SIFT keypoint detectability for low-to-mid frequency texture.

---

## 2. COLMAP N=24 Sparse Model Statistics

### Feature Extraction

| Setting | Value |
|---------|-------|
| Camera model | SIMPLE_RADIAL |
| single_camera_per_folder | 1 (2 shared cameras total) |
| SiftExtraction.peak_threshold | 0.001 |
| SiftExtraction.max_num_features | 16384 |
| Features per original image | ~16,384 (saturated) |
| Features per novel view | 19,269–22,491 |

### Sparse Model

| Metric | Value |
|--------|-------|
| Cameras | 2 (one per magazine) |
| Images registered | **14 / 24** (all 14 originals; 0/10 novel views) |
| Sparse points | 12,822 |
| Observations | 50,648 |
| Mean track length | 3.95 |
| Mean reprojection error | 0.560 px |

### Novel View Registration Status

| Novel View | Registered | Inlier Feature Matches | 3D Points Seen |
|------------|------------|------------------------|----------------|
| novel_mag137_01 | No | 4,845 | ~0 |
| novel_mag137_02 | No | 7,085 | ~0 |
| novel_mag137_03 | No | 8,123 | ~0 |
| novel_mag137_04 | No | 6,912 | ~0 |
| novel_mag137_05 | No | 4,422 | ~0 |
| novel_mag138_01 | No | 1,280 | 0 |
| novel_mag138_02 | No | 1,590 | 0 |
| novel_mag138_03 | No | 1,378 | 0 |
| novel_mag138_04 | No | 1,289 | 0 |
| novel_mag138_05 | No | 852 | 0 |

**Key finding:** Despite having substantial 2D-2D inlier matches (852–8,123 per image), the novel views see 0 reconstructed 3D points during image_registrator. COLMAP PnP registration requires 2D-3D correspondences: the novel view's keypoints must match keypoints that are already triangulated into the sparse model. Because the rendered views generate SIFT keypoints at slightly different image locations than the originals (due to GS rendering's different texture sharpness, tone mapping, and absence of film grain), those keypoints do not appear in the existing point tracks. The exhaustive 2D-2D matches exist but cannot be promoted to 2D-3D correspondences without retriangulation from the novel views — which COLMAP's sequential registration does not attempt for new images.

**Recovery attempt:** `image_registrator` was run post-mapping. All 10 novel views showed 0–13 visible 3D points, below the minimum (~15-20) required for PnP. No novel views registered.

---

## 3. N=24 Dense Reconstruction Statistics

### Pipeline

Custom PyTorch plane-sweep MVS (`colmap_n24/dense/mvs_dense.py`) — identical to the N=14 pipeline, adapted for the N=24 sparse model and image paths. The same COLMAP sparse/0 (14 registered cameras) drives the MVS.

### Intermediate Depth Map Statistics

| Image | Depth Range (COLMAP units) | Consistent Points |
|-------|---------------------------|-------------------|
| mag137/AS17-137-20903HR | 2.91 – 7.57 | 195,546 |
| mag137/AS17-137-20904HR | 2.71 – 7.25 | 330,303 |
| mag137/AS17-137-20905HR | 2.58 – 7.05 | 416,934 |
| mag137/AS17-137-20906HR | 2.54 – 7.08 | 387,610 |
| mag137/AS17-137-20907HR | 2.87 – 7.48 | 355,561 |
| mag137/AS17-137-20908HR | 2.80 – 7.60 | 233,143 |
| mag137/AS17-137-20909HR | 2.87 – 7.75 | 109,629 |
| mag138/AS17-138-21030HR | 3.39 – 7.34 | 115,400 |
| mag138/AS17-138-21031HR | 3.08 – 395.06 | 30,952 |
| mag138/AS17-138-21032HR | 3.31 – 417.80 | 40,416 |
| mag138/AS17-138-21033HR | 3.43 – 432.71 | 15,070 |
| mag138/AS17-138-21034HR | 3.54 – 8.81 | 37,328 |
| mag138/AS17-138-21035HR | 3.65 – 8.70 | 96,491 |
| mag138/AS17-138-21037HR | 4.79 – 9.76 | 99,580 |

Several mag138 images (21031–21033) have extreme depth ranges (up to 433 COLMAP units) because those cameras see lunar background terrain at large distances. Geometric consistency filtering aggressively rejected those background points (e.g., 21033: 1.975M raw → 15,070 consistent). Total points contributed: 2,463,963.

### Point Cloud and Mesh

| Metric | Value |
|--------|-------|
| Raw fused points | 2,463,963 |
| After outlier removal | 2,455,823 |
| Voxel downsample (0.0015) | 2,453,709 points |
| **fused.ply** | **2,453,709 points (66.3 MB)** |
| Initial Poisson (depth=10, full bbox) | 92,684 vertices — 88,049 after 5% density prune |
| Background filter (within 10 COLMAP units of either cluster centroid) | 2,451,217 → 2,343,410 pts after outlier removal |
| **Final Poisson (depth=10, filtered)** | **4,658,194 vertices, 9,375,366 faces (359 MB)** |
| **meshed-poisson.ply** | **4,658,194 vertices ✓** |

**Note on two-stage meshing:** The first Poisson attempt on the full point cloud produced only 88,049 vertices because ~0.1% background outlier points inflated the bounding box to 80+ COLMAP units in Z. At Poisson depth=10 (1024-cell grid), each cell was ~0.08 COLMAP units — too coarse for centimeter-scale rock surface detail. Filtering points within 10 COLMAP units of either cluster centroid before re-running Poisson corrected this, producing 4.66M vertices at the expected resolution.

---

## 4. ICP Alignment

Before quantitative comparison, the N=24 mesh was ICP-aligned to the N=14 mesh to account for bundle adjustment coordinate shifts between the two independent COLMAP runs.

### Registration Pipeline

1. Voxel downsample (0.05 units) → compute FPFH features
2. RANSAC global registration (fitness: 0.586, inlier RMSE: 0.051)
3. ICP point-to-plane refinement (max_correspondence_distance=0.02)

### ICP Result

| Metric | Value |
|--------|-------|
| Fitness | 0.0649 |
| Inlier RMSE | 0.01518 COLMAP units |

**Transform matrix T_icp (N=24 → N=14 frame):**
```
[[ 0.9294  -0.3180   0.1872  -0.9341]
 [ 0.2499   0.9157   0.3148  -0.1896]
 [-0.2715  -0.2459   0.9305  -0.5428]
 [ 0.       0.       0.       1.    ]]
```

The non-trivial rotation (≈17° tilt) and translation (≈1.1 COLMAP units) represent the difference in gauge freedom between the two independent bundle adjustments. This is expected for two COLMAP runs starting from different random seeds with no absolute reference.

**Note on low ICP fitness (0.065):** A fitness of 6.5% means only 6.5% of points found correspondences within 0.02 units. This reflects that the two meshes have different geometric resolutions (N=14: 1.06M vertices; N=24: 4.66M vertices) and that the surfaces don't overlap cleanly at the 0.02-unit threshold. The RANSAC initial alignment (fitness 0.586) and the large rotation component suggest the two reconstructions differ significantly in their global orientation, which is a consequence of the dual-cluster geometry (inter-cluster distance 8.66 COLMAP units providing weak geometric constraint on global orientation).

---

## 5. Quantitative Metrics

Both meshes were sampled at 100,000 points each. Chamfer and Hausdorff computed on aligned point clouds.

| Metric | Value |
|--------|-------|
| Bounding box diagonal | 16.22 COLMAP units |
| **Bidirectional Chamfer distance** | **0.2192 COLMAP units** |
| **Hausdorff p99** | **2.1265 COLMAP units** |
| **F-score @ 1% diagonal (0.162 units)** | **0.673** |
| **F-score @ 5% diagonal (0.811 units)** | **0.934** |
| **F-score @ 10% diagonal (1.622 units)** | **0.984** |

**Interpretation:**

- **Chamfer 0.219 units:** Approximately 1.35% of the bounding box diagonal. Given that the rock subjects span roughly 2–5 COLMAP units across, this represents ~5-10% of the rock diameter — moderate geometric error that includes both genuine mesh differences and residual ICP misalignment.
- **F-score@0.05 = 0.934:** 93.4% of surface samples have a match within 5% of the scene extent, indicating broad geometric agreement between the two meshes.
- **F-score@0.01 = 0.673:** At tighter 1% threshold, 32.7% of the surface diverges, revealing real geometric differences at fine scale — areas where the two reconstructions disagree on exact surface position.
- **Hausdorff p99 = 2.127 units:** The 99th-percentile worst-case distance is 13% of the bounding box, indicating some regions with substantial divergence — these are likely areas at the cluster boundaries or poorly-constrained surfaces where the depth ranges were extreme.

---

## 6. Qualitative Observations

Four cameras were selected from the N=14 sparse model, spanning both clusters and a range of viewing angles. Renders used the same camera poses (R, t) for both meshes (N=24 aligned to N=14 frame via T_icp).

| View | Image | Cluster | Notes |
|------|-------|---------|-------|
| 1 | AS17-137-20904HR | mag137 | Left-of-center mag137 rock view |
| 2 | AS17-137-20907HR | mag137 | Central mag137 rock view |
| 3 | AS17-138-21030HR | mag138 | Wide-angle mag138 entry view |
| 4 | AS17-138-21033HR | mag138 | Central mag138 rock view |

**Observations from renders:**

- **Views 1–2 (mag137):** The N=24 mesh renders show noticeably more surface points projected into the field of view (higher std: 88.8 vs 85.7 for view 1; 86.2 vs 67.1 for view 2), suggesting denser coverage of the rock surface. Both meshes faithfully reconstruct the main rock shape. The N=24 mesh includes additional surface detail around rock edges.

- **Views 3–4 (mag138):** Both meshes produce similar renders (stds 75 vs 80 for view 3; 28 vs 37 for view 4). View 4 shows slightly lower contrast in both meshes, corresponding to the more uniformly-lit central rock face. The N=24 mesh has modestly denser coverage at the rock margins.

- **Overall:** The N=24 mesh is much denser (4.66M vs 1.06M vertices) but the geometric shapes are broadly consistent, supporting the F-score@0.05=0.934 finding. The lower contrast in N=24's view 4 (std 27.7 vs N=14's 37.0) may indicate noise from the more aggressive Poisson reconstruction at the same depth resolution.

---

## 7. Direct Comparison: N=14 vs N=24

| Dimension | N=14 Baseline | N=24 Augmented | Change |
|-----------|--------------|----------------|--------|
| Registered images | 14 | 14 (same) | 0 |
| Novel views registered | — | 0/10 | n/a |
| Sparse points | ~13K | 12,822 | ≈ same |
| Fused PLY points | 1,641,050 | 2,453,709 | +50% |
| Mesh vertices | 1,063,577 | 4,658,194 | +338% |
| Mesh faces | 2,121,270 | 9,375,366 | +342% |
| Mean PSNR (training views) | 33.93 dB | — (not evaluated) | — |

**Mesh density:** The N=24 mesh has 4.38× more vertices than N=14. This reflects the denser point cloud (2.45M vs 1.64M filtered points after background removal) and a more refined Poisson reconstruction. The density increase is real but does not correspond to new scene geometry observed from novel views — it arises entirely from the same 14 original cameras producing more depth map points under the N=24 MVS run (with slightly different parameter interaction due to the dual-cluster depth range issue).

**Completeness:** Both meshes cover the same scene — the two rock clusters independently photographed from ≈7 cameras each. No new surface coverage was achieved in N=24 because no novel views registered. The coverage is bounded by the original 14 cameras' field of view.

**New geometry:** No genuinely new geometry was captured in N=24. The N=24 mesh reconstructs the same surface as N=14, at higher density, from the same camera observations.

**Hallucinated geometry:** The N=24 mesh's Poisson reconstruction at higher depth (requiring background filtering to achieve >100K vertices) risks over-smoothing at cluster boundaries and introducing watertight surface fill in regions with sparse point coverage (especially the far side of each rock, never seen by any camera). These filled regions are Poisson extrapolations — present in both meshes but more elaborate in N=24 due to higher Poisson depth.

---

## 8. Verdict: Did Gaussian Splat Augmentation Improve Photogrammetric Reconstruction?

**Short answer: No, not in the way intended.**

**Explanation:**

The central hypothesis of Phase 2 is that novel views from Gaussian Splatting provide additional photogrammetric observations from angles not covered by the original 14 cameras, enabling COLMAP to reconstruct new geometry and improve mesh completeness. This did not occur.

**What failed:** All 10 novel views failed COLMAP registration despite having substantial 2D-2D feature matches (852–8,123 inlier matches per image across 23 pairs). The failure point was 2D-3D correspondence: rendered views generate SIFT keypoints at slightly different pixel positions than real photographs (different texture rendering, absence of sensor noise and film grain, minor tone mapping differences), so those keypoints do not appear in the existing triangulated point tracks. Without 2D-3D correspondences, PnP localization fails.

**What succeeded:** The N=24 dataset construction, COLMAP pipeline, and dense reconstruction all ran successfully. The resulting mesh (4.66M vertices, Chamfer 0.219, F-score@0.05 = 0.934) is geometrically consistent with the N=14 baseline and denser — but that density improvement came from a pipeline refinement (background point filtering before Poisson), not from novel views.

**Caveats:**
1. A different feature matching strategy (SuperPoint/SuperGlue, learned descriptors) might successfully match rendered views to photographic keypoints, enabling registration. SIFT is known to be sensitive to appearance changes from rendering.
2. The two-cluster geometry (8.66-unit inter-cluster distance) means the pose graph is already nearly perfectly constrained by the original 14 cameras. Even if novel views registered, they would add density within the existing viewing envelope rather than extending it.
3. A single-cluster dataset (all images of one rock) would be a better testbed for the splat augmentation hypothesis — the hypothesis might hold for scenes with fewer baseline cameras or larger coverage gaps.

**Quantitative verdict:** The ICP fitness of 0.065 and the F-score@0.01 of 0.673 both indicate meaningful structural differences between the two meshes, but these are as likely attributable to bundle adjustment gauge differences and Poisson reconstruction variation as to any real geometric improvement from augmentation.

---

## 9. Deviations from Plan

| Deviation | Reason | Resolution |
|-----------|--------|------------|
| 0/10 novel views registered (target: 20+) | SIFT keypoints from rendered views don't match existing 3D point tracks; image_registrator found 0–13 visible points per novel view | Documented; N=24 sparse model uses only 14 original cameras |
| Initial Poisson mesh: 88K vertices (<100K target) | Extreme depth ranges for mag138 cameras (up to 432 COLMAP units) inflated bounding box, making Poisson grid too coarse | Applied cluster-proximity filter (within 10 COLMAP units of either centroid) before remeshing; final mesh 4.66M vertices |
| N=24 MVS uses distorted images (not COLMAP undistorted) | N=24 didn't run through COLMAP's image_undistorter | Same approach used in N=14 Part A.1; custom MVS handles distortion implicitly via scale factor |
| Reprojection error camera selection: NaN/high values | Binary images.bin format reads xys/pt3d with interleaved layout, causing corrupt reads in the qualitative script | Camera selection fell back to fractile ordering; renders are geometrically correct (R, t correctly read from qvec/tvec) |

---

## 10. Key Files

| File | Description |
|------|-------------|
| `data/rgb_n24/mag137/` | 12 images: 7 originals + 5 novel views (2340×2364 px) |
| `data/rgb_n24/mag138/` | 12 images: 7 originals + 5 novel views (2340×2345 px) |
| `colmap_n24/database.db` | COLMAP feature/match database (24 images) |
| `colmap_n24/sparse/0/` | Sparse model (14 registered cameras, 12,822 points) |
| `colmap_n24/dense/fused.ply` | Dense point cloud: 2,453,709 points (66 MB) |
| `colmap_n24/dense/meshed-poisson.ply` | Poisson mesh: 4,658,194 vertices (359 MB) |
| `colmap_n24/dense/mvs_dense.py` | Adapted MVS script for N=24 |
| `comparison/quantitative_metrics.json` | ICP + Chamfer + Hausdorff + F-score JSON |
| `comparison/per_vertex_distance.ply` | N=24 mesh vertex-colored by distance to N=14 |
| `comparison/per_vertex_distance.png` | 2D heatmap (top-down X-Z projection) |
| `comparison/render_n14_view{1..4}.png` | N=14 mesh renders at 4 viewpoints |
| `comparison/render_n24_view{1..4}.png` | N=24 mesh renders (ICP-aligned) at 4 viewpoints |
| `comparison/sidebyside_view{1..4}.png` | Side-by-side: N=14 / photo / N=24 for each view |
| `comparison/qualitative_grid.png` | 2×2 grid of all 4 side-by-sides |
| `build_n24_dataset.py` | Dataset construction script |
| `mesh_compare.py` | ICP alignment + quantitative metrics script |
| `qualitative_compare.py` | Mesh rendering + qualitative comparison script |
| `remesh_n24.py` | Background-filtered Poisson remeshing script |
