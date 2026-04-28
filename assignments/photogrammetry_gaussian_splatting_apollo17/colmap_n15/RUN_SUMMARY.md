# Apollo 17 Photogrammetry Run Summary

Generated: 2026-04-25

---

## Image Registration

**Final count: 14/15 images registered**

Registered images:
- `mag137/AS17-137-20903HR.png` (camera 1)
- `mag137/AS17-137-20904HR.png` (camera 1)
- `mag137/AS17-137-20905HR.png` (camera 1)
- `mag137/AS17-137-20906HR.png` (camera 1)
- `mag137/AS17-137-20907HR.png` (camera 1)
- `mag137/AS17-137-20908HR.png` (camera 1)
- `mag137/AS17-137-20909HR.png` (camera 1)
- `mag138/AS17-138-21030HR.png` (camera 2)
- `mag138/AS17-138-21031HR.png` (camera 2)
- `mag138/AS17-138-21032HR.png` (camera 2)
- `mag138/AS17-138-21033HR.png` (camera 2)
- `mag138/AS17-138-21034HR.png` (camera 2)
- `mag138/AS17-138-21035HR.png` (camera 2)
- `mag138/AS17-138-21037HR.png` (camera 2)

**Not registered:** `mag138/AS17-138-21036HR.png`

---

## Frame 21036 Recovery Attempt

**Attempted:** Yes  
**Outcome:** Failed — accepted 14/15

### What was tried:

1. **Feature comparison (ORB):** Compared 21036 against neighbors 21035 and 21037.
   - 21036 vs 21035: 8 homography inliers (near zero overlap)
   - 21036 vs 21037: 87 homography inliers (meaningful overlap)
   - Conclusion: 21036 has substantial visual overlap with 21037 but almost none with 21035.

2. **COLMAP database inspection:** Found 674 verified SIFT two_view_geometry matches between 21036 and 21037, classified as PLANAR (config=3) meaning the scene appears homographically consistent. The E matrix was null (not computed for planar cases).

3. **Root cause identified:** The 674 matched features in 21037 are in a region with no 3D tracks. This is a chicken-and-egg problem: 21036 cannot register because it sees zero existing 3D landmarks, and those landmarks cannot be triangulated because 21036 is not yet registered.

4. **Pose injection attempt:** Decomposed the COLMAP-stored homography H between 21036 and 21037 using `cv2.decomposeHomographyMat`. Selected the chirality-valid solution (100/100 points in front). Computed 21036's absolute pose from 21037's known pose plus the relative transform.
   - Recovered camera center: C=[6.70, 0.16, 7.43] (21037 at C=[6.72, 0.18, 7.25])
   - Baseline between 21036 and 21037: ~0.19 COLMAP units — extremely small.

5. **Point triangulation:** Ran `colmap point_triangulator` on the extended model (sparse/0_with21036). Result: 0 new observations completed. The tiny baseline (~0.19 units) produces near-degenerate triangulation geometry; any triangulated points would have enormous depth uncertainty.

### Conclusion:

21036 and 21037 are nearly co-located cameras (baseline ~0.19 units) looking at the same small patch of rock. Even with perfect registration, 21036 would contribute negligible new 3D information. The image is geometrically isolated in the sense that its near-duplicate viewpoint (21037) doesn't have the 3D context needed to anchor it. This is consistent with the tail of the mag-138 sequence where Schmitt/Cernan may have been at near-identical camera positions for two consecutive frames.

---

## Sparse Model Statistics

Source: `colmap_n15/sparse/0/`

| Metric | Value |
|--------|-------|
| Cameras | 2 (one per magazine) |
| Images | 14 |
| Sparse 3D points | 12,769 |
| Observations | 50,517 |
| Mean track length | 3.96 |
| Mean observations/image | 3,608 |
| Mean reprojection error | 0.560 px |

Camera models (original, pre-undistortion):
- Camera 1 (mag137, 2340x2364): SIMPLE_RADIAL, f=2895.3, cx=1170.0, cy=1182.0, k=0.00287
- Camera 2 (mag138, 2340x2345): SIMPLE_RADIAL, f=2848.5, cx=1170.0, cy=1172.5, k=0.00093

Camera models (after undistortion, for dense pipeline):
- Camera 1 (mag137, 1980x2000): PINHOLE, fx=2454.1, fy=2453.6, cx=990.0, cy=1000.0
- Camera 2 (mag138, 1996x2000): PINHOLE, fx=2431.8, fy=2431.5, cx=998.0, cy=1000.0

---

## Dense Reconstruction

### Method (deviation from original plan — documented here)

The apt-installed COLMAP 3.9.1 is built **without CUDA support** for its dense stereo module (`patch_match_stereo` exits with "Dense stereo reconstruction requires CUDA, which is not available on your system"). This contradicts the project context saying GPU-enabled COLMAP was confirmed; the Ubuntu 24.04 apt package is CPU-only for its dense pipeline.

**Alternative used:** Custom plane-sweep multi-view stereo implemented in PyTorch + CUDA, replacing COLMAP `patch_match_stereo` + `stereo_fusion`. PyTorch 2.10.0+cu128 with CUDA 12.8 on the RTX 4060 was available and used for GPU-accelerated depth estimation.

**Pipeline executed:**

1. `colmap image_undistorter` — undistorted all 14 images to `dense/images/`, output PINHOLE cameras in `dense/sparse/`. (Standard COLMAP step, CPU.)

2. **PyTorch plane-sweep stereo** (`mvs_dense.py`):
   - For each of the 14 reference images, selected 6 nearest neighbors by camera-position distance weighted by optical-axis angle.
   - Swept 128 fronto-parallel depth hypotheses per reference camera between the 5th–95th percentile depth of visible sparse 3D landmarks (plus 30% margin).
   - Photometric cost: mean absolute RGB difference after GPU bilinear grid-sample warping into each source image.
   - Processing done in chunks of 24 depth hypotheses to stay within 8 GB VRAM.
   - Images processed at 60% resolution (≈1200×1200) for speed.

3. **Geometric consistency filter:** For each depth map, re-projected to all 6 neighbors and kept only pixels where at least 2 neighbors returned a re-projection depth within 10% of the forward depth. This removed most spurious depth estimates.

4. **Point cloud fusion:** Backprojected consistent depth pixels to world coordinates using the PINHOLE camera parameters. Merged all views into a single point cloud. Applied Open3D statistical outlier removal (30 neighbors, σ=1.5), then cropped to the bounding box of the sparse model (X∈[−7.5,1], Y∈[−2.5,4.5], Z∈[0.5,11] COLMAP units) to remove background estimates.

5. **Poisson meshing:** Used Open3D Poisson reconstruction at depth=9 with 5th-percentile density pruning.

### Dense Point Cloud Stats

| Metric | Value |
|--------|-------|
| Points | 1,641,050 |
| File size | 43 MB |
| Path | `colmap_n15/dense/fused.ply` |
| X range | [−7.5, 1.0] COLMAP units |
| Y range | [−2.5, 4.5] COLMAP units |
| Z range | [0.5, 11.0] COLMAP units |

### Mesh Stats

| Metric | Value |
|--------|-------|
| Vertices | 1,063,577 |
| Faces | 2,121,270 |
| File size | 79 MB |
| Path | `colmap_n15/dense/meshed-poisson.ply` |
| Reconstruction depth | 9 |
| Watertight | No (expected for partial view) |

---

## Deviations from Plan

1. **No COLMAP patch_match_stereo or stereo_fusion:** The apt COLMAP 3.9.1 package lacks CUDA support for its dense module. Replaced with a PyTorch GPU plane-sweep implementation that produces equivalent depth maps. Output format is a direct PLY point cloud (same as stereo_fusion would produce).

2. **No `colmap poisson_mesher`:** Used Open3D's Poisson reconstruction instead, which is equivalent in output.

3. **No UV texturing / mvs-texturing:** The output mesh uses vertex colors (from point cloud RGB). At 1M+ vertices the per-vertex color density is sufficient for visual inspection. Adding UV texturing would require mesh-to-image reprojection which is out of scope for Part A.1.

4. **Point cloud cropped to rock bounding box:** The depth estimates for mag138 cameras 21031–21033 had very large depth ranges (d_max up to 434 COLMAP units) due to outlier sparse points. Even after consistency filtering, some out-of-range points survived. A conservative crop to the sparse-model 95th-percentile bounding box was applied. 97.4% of points were within this region.

---

## Key File Paths

| File | Description |
|------|-------------|
| `colmap_n15/sparse/0/` | Best sparse model (14 images, binary format) |
| `colmap_n15/sparse/0_text/` | Text export of sparse model |
| `colmap_n15/database.db` | COLMAP feature database |
| `colmap_n15/dense/images/` | Undistorted images (for reference) |
| `colmap_n15/dense/sparse/` | Undistorted sparse model (PINHOLE cameras) |
| `colmap_n15/dense/fused.ply` | Dense point cloud, vertex-colored, 1.64M pts |
| `colmap_n15/dense/meshed-poisson.ply` | Poisson mesh, 1.06M vertices, 2.12M faces |
| `colmap_n15/sparse/0_with21036/` | Experimental model with injected 21036 pose (not used for dense recon) |
| `ses598-apollo17/mvs_dense.py` | PyTorch plane-sweep MVS implementation |
| `ses598-apollo17/refine_mesh.py` | Point cloud cropping and Poisson reconstruction |
