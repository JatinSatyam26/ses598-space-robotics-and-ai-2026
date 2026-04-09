# Midterm Pipeline Results

## Aerial Survey (Phase 1)
- **2000 RGB frames** captured at 800×600, frame diff = 25.31 (excellent variation)
- **Survey pattern**: 10-waypoint boustrophedon lawnmower, ±10m extent, 3m spacing, 5m AGL
- **Camera**: fx = fy = 582.2 px, cx = 400, cy = 300

## 3D Gaussian Splatting Reconstruction (Phase 2)
- **Tool**: nerfstudio `splatfacto`, 7000 iterations, RTX 4060
- **Output**: 14,156 Gaussians (`~/midterm_3dgs_export/splat.ply`, 3.4 MB)
- **COLMAP input**: 1999 poses, 4820 initial ground-plane points

## Navigation Costmap (Phase 3)
- **Size**: 55×42 cells @ 0.5 m/cell = 27.5×21.0 m
- **Coverage**: 66.2% free | 2.0% obstacle | 27.4% unknown
- **Largest free region**: 354.75 m²
- **Terrain height range**: 0.00–0.74 m

## Rover Navigation (Phase 4 — A* on costmap)
- **Method**: A* planner on drone-generated costmap (nav2 not installed → scripts/run_ablation.py)
- **Route**: (8.0, 0.0) → (−5.0, −5.0)
- **Path length**: 14.66 m | **Time**: 29.3 s @ 0.5 m/s
- **Smoothness**: 0.346 rad mean |Δθ|
- **Min obstacle clearance**: 6.0 m

## Ablation Study (Phase 5)
| Metric | WITH uncertainty | WITHOUT uncertainty |
|---|---|---|
| Path found | **NO** | Yes |
| Path length | — | 14.66 m |
| Unknown cells crossed | — | 9 |
| Planner time | 7 ms | 1 ms |

**Key finding**: The uncertainty-aware planner correctly refuses to navigate to (−5,−5) because that location is outside the drone's survey coverage (unknown territory). The uncertainty-ignoring planner finds a path but crosses 9 poorly-observed cells — demonstrating the safety benefit of drone-first cooperative mapping.

## Output Files
| File | Location |
|---|---|
| RGB frames | `~/midterm_flight_data/rgb/` |
| Poses (synthetic) | `~/midterm_flight_data/poses.json` |
| Camera info | `~/midterm_flight_data/camera_info.json` |
| COLMAP model | `~/midterm_reconstruction/sparse/0/` |
| Costmap | `~/midterm_reconstruction/costmap.{pgm,yaml}` |
| 3DGS checkpoint | `~/midterm_3dgs_output/.../step-6999.ckpt` |
| Gaussian splat | `~/midterm_3dgs_export/splat.ply` |
| Rover path | `~/midterm_results/rover_path.json` |
| Metrics | `~/midterm_results/metrics.json` |
| Bird's-eye view | `~/midterm_results/birdseye_result.png` |
| Ablation figure | `~/midterm_results/ablation_figure.png` |

## Fixes Applied
1. **PX4 time-sync death spiral**: Added `param set-default UXRCE_DDS_SYNCT 0` to
   `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_mono`
2. **gsplat CUDA OOM**: Set `MAX_JOBS=2` before `ns-train` (default MAX_JOBS=10 OOM-kills nvcc on 16 GB)
3. **nerfstudio PyTorch 2.6 compat**: Patched `eval_utils.py` with `weights_only=False`
4. **Synthetic poses**: `scripts/generate_synthetic_poses.py` generates lawnmower-pattern poses
   when PX4 odometry is unavailable (e.g., poses.json empty after flight)
