# Apollo 17 Gaussian Splatting Run Summary (Part A.2)

Generated: 2026-04-25

---

## Framework and Installation

**Method:** Nerfstudio 1.1.5 `splatfacto` (3D Gaussian Splatting via gsplat 1.4.0 backend)

**Environment:**
- Python 3.12, PyTorch 2.10.0+cu128
- CUDA 12.8, RTX 4060 Laptop 8 GB VRAM
- Nerfstudio installed via pip (`~/.local/bin/ns-train`, `~/.local/bin/ns-render`)
- gsplat 1.4.0 (CUDA Gaussian rasterizer)

**Training command:**
```
ns-train splatfacto \
  --output-dir splats_n15 \
  --experiment-name apollo17_n15 \
  --max-num-iterations 30000 \
  --vis tensorboard \
  colmap \
  --data colmap_n15 \
  --colmap-path sparse/0 \
  --images-path images \
  --eval-mode all \
  --downscale-factor 2
```

`--eval-mode all` treats all 14 registered images as training views (no held-out test set), ensuring all views are used for training and rendering.

---

## Input Data

| Metric | Value |
|--------|-------|
| Images | 14 (all registered frames; AS17-138-21036HR not registered) |
| Camera model (sparse) | SIMPLE_RADIAL (2 cameras, one per magazine) |
| Training resolution | 1170×1182 (mag137), 1170×1172 (mag138) — 2× downscale from undistorted |
| Dataparser | COLMAP (`colmap_n15/sparse/0/`) |

**Resolution handling:** `colmap_n15/images_2/` was pre-computed (OpenCV INTER_AREA half-resolution) before training to bypass Nerfstudio's interactive downscale prompt, which cannot run non-interactively.

The symlink `colmap_n15/images → data/rgb` points to the original full-resolution images; Nerfstudio reads from `images_2/` at downscale factor 2.

---

## Training Details

| Setting | Value |
|---------|-------|
| Iterations | 30,000 (completed at step 29999) |
| Initial Gaussians | ~2,000 (from sparse SfM points) |
| Final Gaussians | ~993,000 (after densification + pruning) |
| Densification schedule | Steps 0–15,000: split/duplicate every 100 steps |
| Checkpoint | `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/nerfstudio_models/step-000029999.ckpt` (683 MB) |
| Approx. wall time | ~44 minutes on RTX 4060 Laptop |

Densification grew from ~2K to a peak of ~1.0M Gaussians; pruning kept count near 990K–1.0M during the densification phase. After step 15,000 no further split/duplicate/prune events occurred.

---

## Rendering

**Command:**
```
ns-render dataset \
  --load-config splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/config.yml \
  --output-path renders_n15/ \
  --split train \
  --image-format png
```

All 14 training-view renders saved to `renders_n15/train/rgb/mag137/` and `renders_n15/train/rgb/mag138/` at training resolution (1170×1182 / 1170×1172).

---

## Quantitative Metrics

PSNR computed with `skimage.metrics.peak_signal_noise_ratio(data_range=255)`.  
SSIM computed with `skimage.metrics.structural_similarity(channel_axis=-1, data_range=255)`.  
Originals downsampled to render resolution with `cv2.INTER_AREA` before comparison.

### Aggregate (14 training views)

| Metric | Mean | Median | Min | Max | Std |
|--------|------|--------|-----|-----|-----|
| PSNR (dB) | 33.93 | 34.11 | 30.18 | 37.36 | 2.24 |
| SSIM | 0.9430 | 0.9455 | 0.9069 | 0.9688 | 0.0187 |

### Per-Image Results

| Image | PSNR (dB) | SSIM |
|-------|-----------|------|
| AS17-137-20903HR | 30.22 | 0.9425 |
| AS17-137-20904HR | 30.18 | 0.9312 |
| AS17-137-20905HR | 32.42 | 0.9382 |
| AS17-137-20906HR | 31.08 | 0.9273 |
| AS17-137-20907HR | 34.52 | 0.9590 |
| AS17-137-20908HR | 33.12 | 0.9675 |
| AS17-137-20909HR | 33.70 | 0.9686 |
| AS17-138-21030HR | 36.51 | 0.9688 |
| AS17-138-21031HR | 35.30 | 0.9485 |
| AS17-138-21032HR | 33.46 | 0.9186 |
| AS17-138-21033HR | 34.97 | 0.9069 |
| AS17-138-21034HR | 36.19 | 0.9263 |
| AS17-138-21035HR | 35.96 | 0.9497 |
| AS17-138-21037HR | 37.36 | 0.9493 |

**Best PSNR:** AS17-138-21037HR = 37.36 dB (SSIM 0.9493)  
**Worst PSNR:** AS17-137-20904HR = 30.18 dB (SSIM 0.9312)  
**Median PSNR:** AS17-137-20907HR = 34.52 dB

The mag137 sequence (frames 20903–20909) consistently scores lower than mag138 (21030–21037). Both groups share a single COLMAP camera model (SIMPLE_RADIAL) per magazine; the mag137 camera has a slightly higher radial distortion coefficient (k=0.00287 vs k=0.00093 for mag138). This may contribute a small alignment error at the image edges, slightly depressing PSNR. The two lowest-scoring frames (20903, 20904) are consecutive and may also have the least coverage from other training views for the Gaussian densification.

---

## Visual Outputs

| File | Description |
|------|-------------|
| `metrics_n15/per_image_metrics.png` | Bar charts: PSNR (steelblue) and SSIM (mediumvioletred) per image with goldenrod mean lines |
| `metrics_n15/comparison_grid.png` | 4-row grid: worst/median/random/best PSNR images, original vs rendered side-by-side |
| `metrics_n15/per_image_metrics.csv` | Per-image CSV (14 rows): image name, paths, resolutions, PSNR, SSIM |
| `metrics_n15/aggregate.json` | Aggregate statistics JSON (mean/median/min/max/std for PSNR and SSIM) |

---

## Novel Pose Smoke Test

A single novel view was rendered at a pose interpolated (t=0.5) between the first (AS17-137-20903HR) and last (AS17-137-20909HR) frames of the mag137 sequence. The interpolation uses:
- **Rotation:** SLERP between the two COLMAP quaternions
- **Translation:** Linear interpolation of COLMAP translation vectors
- **Coordinate transform:** Applied dataparser_transforms.json affine + scale to map COLMAP world coordinates to Nerfstudio scene coordinates

The rendered frame shows a coherent view of the lunar rock with expected perspective midway between the two training cameras. No catastrophic floater artifacts or black regions were observed.

| File | Description |
|------|-------------|
| `metrics_n15/novel_pose_smoke.mp4` | 1-frame video of interpolated novel view |
| `metrics_n15/novel_pose_frame.png` | Still extracted from the video (1980×2000 px) |
| `metrics_n15/novel_camera_path.json` | Camera path JSON used for rendering |

---

## Deviations from Plan

1. **Pre-computed images_2/ directory:** Nerfstudio's COLMAP dataparser prompts interactively when downscaling images at runtime. Since training runs non-interactively (background), `colmap_n15/images_2/` was pre-populated with OpenCV-downscaled images before launching `ns-train`.

2. **Renders in subdirectories:** `ns-render dataset` outputs to `renders_n15/train/rgb/mag137/` and `renders_n15/train/rgb/mag138/` (preserving the COLMAP image subdirectory structure). The `compute_metrics.py` script was updated to use `rglob('*.png')` instead of `glob('*.png')` to search recursively.

3. **Two aborted training runs:** A first training run (2026-04-25_144104) was launched but terminated immediately before the pre-computed images_2/ directory was ready. The second run (2026-04-25_144254) completed successfully to step 29999.

---

## Key File Paths

| File | Description |
|------|-------------|
| `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/config.yml` | Nerfstudio training config |
| `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/nerfstudio_models/step-000029999.ckpt` | Final model checkpoint (683 MB) |
| `splats_n15/apollo17_n15/splatfacto/2026-04-25_144254/dataparser_transforms.json` | COLMAP-to-Nerfstudio affine transform + scale |
| `renders_n15/train/rgb/mag137/` | 7 rendered training views (mag137 sequence) |
| `renders_n15/train/rgb/mag138/` | 7 rendered training views (mag138 sequence) |
| `metrics_n15/per_image_metrics.csv` | Per-image PSNR/SSIM results |
| `metrics_n15/aggregate.json` | Aggregate PSNR/SSIM statistics |
| `metrics_n15/per_image_metrics.png` | Bar chart visualization |
| `metrics_n15/comparison_grid.png` | Original vs rendered comparison grid |
| `metrics_n15/novel_pose_frame.png` | Novel interpolated view still frame |
| `compute_metrics.py` | PSNR/SSIM computation script |
| `make_comparison_grid.py` | Comparison grid generation script |
| `novel_pose_smoke.py` | Novel pose interpolation + camera path generation |
| `splats_n15/train.log` | Full training stdout/stderr log |
