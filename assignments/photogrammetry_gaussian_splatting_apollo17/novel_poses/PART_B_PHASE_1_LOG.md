# Part B Phase 1 Log: Novel View Generation

Generated: 2026-04-25

---

## 1. Dataparser Transform

**File:** `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/dataparser_transforms.json`

**Transform matrix T34 (3×4):**
```
[[ 0.98847  -0.15134   0.00504  -0.52542]
 [ 0.00504   0.06611   0.99780  -0.70265]
 [-0.15134  -0.98627   0.06611   0.05325]]
```

**Scale:** 0.152103746238842

**Interpretation:**
- COLMAP +Z maps to Nerfstudio +Y (up): the scene's gravity axis is COLMAP +Z
- COLMAP +X maps to Nerfstudio +X (right)
- T34 is the composition of (a) the row-swap/negate applied_transform and (b) the auto-orient T_auto

**Critical finding:** The colmap dataparser additionally applies `c2w[0:3, 1:3] *= -1` (negating Y and Z columns, converting COLMAP OpenCV to OpenGL camera convention) BEFORE folding into T34. This column flip is NOT included in T34. Novel camera poses must therefore be passed through:

```
novel_pose_ns = T34 @ (c2w_colmap @ diag(1, -1, -1, 1))
```

where `diag(1,-1,-1,1)` negates the Y (camera down→up) and Z (camera forward→backward) columns, converting COLMAP OpenCV convention to OpenGL convention expected by nerfstudio.

---

## 2. Cluster Geometry

**mag137** (7 cameras, frames 20903–20909):
| Property | Value |
|----------|-------|
| Centroid C | (-3.066, -0.118, -1.703) |
| Median radius r | 0.871 COLMAP units |
| Mean forward F | (-0.056, 0.115, 0.992) |
| Look-at target T = C + F×r×1.5 | (-3.139, 0.032, -0.406) |
| F_xz (XZ projection) | (-0.057, 0, 0.998) |

**mag138** (7 cameras, frames 21030–21037):
| Property | Value |
|----------|-------|
| Centroid C | (4.128, 0.157, 3.103) |
| Median radius r | 1.636 COLMAP units |
| Mean forward F | (-0.817, 0.233, 0.527) |
| Look-at target T = C + F×r×1.5 | (2.121, 0.729, 4.396) |
| F_xz (XZ projection) | (-0.841, 0, 0.542) |

---

## 3. Novel Pose Definitions

Arc formula: `eye = C + r × (cos(θ) × F_xz + sin(θ) × P_xz)`, Y clamped to C.y.
`P_xz = (-F_xz.z, 0, F_xz.x)` (90° CCW rotation of F_xz in XZ plane).

**mag137 poses** (look-at target: (-3.139, 0.032, -0.406)):

| Pose | θ (deg) | Eye (COLMAP world) | t_ns (Nerfstudio) |
|------|---------|---------------------|-------------------|
| mag137_01 | -30 | (-2.673, -0.118, -0.925) | (-0.480, -0.250, 0.078) |
| mag137_02 | -15 | (-2.888, -0.118, -0.849) | (-0.512, -0.239, 0.084) |
| mag137_03 |   0 | (-3.115, -0.118, -0.833) | (-0.546, -0.237, 0.089) |
| mag137_04 | +15 | (-3.338, -0.118, -0.875) | (-0.580, -0.243, 0.094) |
| mag137_05 | +30 | (-3.543, -0.118, -0.974) | (-0.611, -0.259, 0.098) |

**mag138 poses** (look-at target: (2.121, 0.729, 4.396)):

| Pose | θ (deg) | Eye (COLMAP world) | t_ns (Nerfstudio) |
|------|---------|---------------------|-------------------|
| mag138_01 | -30 | (3.380, 0.157, 4.558) | (0.428, 0.589, -0.048) |
| mag138_02 | -15 | (3.029, 0.157, 4.315) | (0.375, 0.552, -0.042) |
| mag138_03 |   0 | (2.752, 0.157, 3.989) | (0.333, 0.502, -0.039) |
| mag138_04 | +15 | (2.570, 0.157, 3.603) | (0.306, 0.444, -0.038) |
| mag138_05 | +30 | (2.493, 0.157, 3.183) | (0.294, 0.380, -0.041) |

---

## 4. Validation Table

| Frame | Mean | Std | Verdict |
|-------|------|-----|---------|
| mag137_01 | 98.19 | 76.16 | OK |
| mag137_02 | 92.43 | 76.87 | OK |
| mag137_03 | 90.23 | 73.58 | OK |
| mag137_04 | 108.08 | 59.88 | OK |
| mag137_05 | 103.97 | 53.08 | OK |
| mag138_01 | 151.74 | 48.99 | OK |
| mag138_02 | 159.76 | 43.46 | OK |
| mag138_03 | 148.83 | 49.72 | OK |
| mag138_04 | 156.64 | 46.58 | OK |
| mag138_05 | 141.09 | 50.48 | OK |

All 10 frames non-blank (std > 5). Threshold for OK: std > 5.

---

## 5. Debug Iterations

**Iteration 1** (initial — wrong convention, formula look-at target):
Applied `novel_pose = T34 @ c2w_lookat` without column flip. Result: 9/10 non-blank (mag138_02 std=4.49 BLANK). Root cause: without `@ diag(1,-1,-1,1)`, novel cameras looked in the *opposite* direction from training cameras. 9 happened to see stray Gaussian floaters; mag138_02 looked into empty space.

**Iteration 2** (wrong convention, 3D-point median look-at target):
Changed T_lookat to median of in-bbox 3D points visible to each cluster (mag137: (-3.227, 0.577, 2.993); mag138: (-0.639, 1.145, 4.999)). Still missing column flip. Result: still 1 blank frame (mag138_02 std=3.87). Confirmed bug is the convention flip, not the target.

**Iteration 3** (correct convention, formula look-at target):
Applied full convention: `novel_pose = T34 @ (c2w_lookat @ diag(1,-1,-1,1))`. Reverted T_lookat to formula. Result: all 10 OK (std range 43–87). ✓

---

## 6. Renders

| Property | Value |
|----------|-------|
| Resolution | 2000×2000 pixels |
| Vertical FOV | 44.42° (from camera 1: f=1447.65 px, h=1182 px at downscale 2) |
| Output | `novel_poses/novel_views.mp4` (10 frames at 1 fps) |
| Frames | `novel_mag137_01.png` … `novel_mag138_05.png` |
| Contact sheet | `novel_poses/contact_sheet.png` |
| Config used | `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/config.yml` |
| Checkpoint | `step-000029999.ckpt` |

---

## 7. Key Files

| File | Description |
|------|-------------|
| `gen_novel_poses.py` | Pose generation + camera-path JSON writer |
| `make_contact_sheet.py` | 2×5 contact sheet generator |
| `novel_poses/novel_camera_path.json` | Camera-path JSON fed to ns-render |
| `novel_poses/novel_views.mp4` | Raw render output (10 frames) |
| `novel_poses/novel_mag137_01..05.png` | Extracted individual renders (mag137) |
| `novel_poses/novel_mag138_01..05.png` | Extracted individual renders (mag138) |
| `novel_poses/contact_sheet.png` | 2×5 grid of all 10 thumbnails |
