#!/usr/bin/env python3
"""
generate_costmap.py — Convert a 3DGS / point-cloud .ply into a Nav2 costmap.

Usage
-----
  python3 generate_costmap.py <input.ply> [options]

Options
  --resolution M        Metres per grid cell (default: 0.1)
  --output-dir DIR      Where to write costmap.{pgm,yaml,png} (default: same dir as .ply)
  --x-min / --x-max     Manual map bounds in X (metres).  Default: auto from point cloud.
  --y-min / --y-max     Manual map bounds in Y (metres).  Default: auto from point cloud.
  --padding M           Extra margin to add around auto-detected bounds (default: 1.0 m)
  --inflation-radius M  Obstacle inflation in metres (default: 0.2 m = 2 cells at 0.1 m)
  --z-min / --z-max     Clip points outside this Z range before processing (default: no clip)

Classification thresholds (all heights in metres, variance in m²)
  --free-height     MAX Z for a FREE cell            (default: 0.5)
  --free-variance   MAX Z-variance for FREE          (default: 0.1)
  --obs-height      MIN Z to call a cell OBSTACLE    (default: 1.0)
  --obs-variance    MIN Z-variance for OBSTACLE      (default: 0.5)
  --lethal-height   MIN Z for LETHAL (also 100)      (default: 2.0)
  --min-count       MIN points for any classification (default: 3)
  --free-count      MIN points for FREE              (default: 5)
"""

import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image

# ── helpers ────────────────────────────────────────────────────────────────────

def load_ply(path: Path):
    """
    Load a .ply file and return arrays:
      xyz   : (N, 3) float32
      scales: (N, 3) float32 or None   — Gaussian scale_0/1/2 if present
      opacs : (N,)   float32 or None   — Gaussian opacity if present

    Tries open3d first (fast), then plyfile (more flexible for non-standard fields).
    """
    print(f"Loading {path} …")
    xyz = scales = opacs = None

    # ── open3d fast path ──────────────────────────────────────────────────────
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(path))
        pts = np.asarray(pcd.points, dtype=np.float32)
        if pts.shape[0] == 0:
            raise ValueError("open3d returned 0 points")
        xyz = pts
        print(f"  open3d: {len(xyz):,} points")
    except Exception as e:
        print(f"  open3d failed ({e}), trying plyfile …")

    # ── plyfile path (handles Gaussian fields) ────────────────────────────────
    try:
        from plyfile import PlyData
        ply = PlyData.read(str(path))
        vtx = ply['vertex']
        names = {p.name for p in vtx.properties}

        x = vtx['x'].astype(np.float32)
        y = vtx['y'].astype(np.float32)
        z = vtx['z'].astype(np.float32)
        xyz = np.stack([x, y, z], axis=1)

        # Gaussian scale fields  (nerfstudio exports scale_0 / scale_1 / scale_2)
        scale_fields = ['scale_0', 'scale_1', 'scale_2']
        if all(f in names for f in scale_fields):
            s0 = vtx['scale_0'].astype(np.float32)
            s1 = vtx['scale_1'].astype(np.float32)
            s2 = vtx['scale_2'].astype(np.float32)
            # Nerfstudio stores log-scale; exponentiate to get real scale
            scales = np.exp(np.stack([s0, s1, s2], axis=1))
            print(f"  Gaussian scales found (log-space, converted to metres)")

        # Opacity  (nerfstudio: 'opacity' field, sigmoid-activated during export)
        if 'opacity' in names:
            opacs = 1.0 / (1.0 + np.exp(-vtx['opacity'].astype(np.float32)))
            print(f"  Gaussian opacity found")
        elif 'f_dc_0' in names:
            print(f"  Spherical-harmonic DC fields found (not used for costmap)")

        print(f"  plyfile: {len(xyz):,} points, fields: {sorted(names)}")
    except ImportError:
        if xyz is None:
            sys.exit("ERROR: neither open3d nor plyfile could load the .ply. "
                     "Install with:  pip install open3d plyfile --break-system-packages")
    except Exception as e:
        if xyz is None:
            sys.exit(f"ERROR loading {path}: {e}")

    return xyz, scales, opacs


def make_grid(xyz, args):
    """
    Project points onto XY plane and return per-cell statistics.
    Returns: (grid_count, grid_z_max, grid_z_var, grid_scale_mean, meta)
    """
    res = args.resolution

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    # Determine bounds
    x_min = args.x_min if args.x_min is not None else float(x.min()) - args.padding
    x_max = args.x_max if args.x_max is not None else float(x.max()) + args.padding
    y_min = args.y_min if args.y_min is not None else float(y.min()) - args.padding
    y_max = args.y_max if args.y_max is not None else float(y.max()) + args.padding

    cols = max(1, math.ceil((x_max - x_min) / res))
    rows = max(1, math.ceil((y_max - y_min) / res))
    print(f"  Grid: {cols} × {rows} cells  ({cols*res:.1f} × {rows*res:.1f} m)  "
          f"origin=({x_min:.2f}, {y_min:.2f})")

    meta = dict(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max,
                cols=cols, rows=rows, res=res)

    # Cell indices for each point
    ci = np.clip(((x - x_min) / res).astype(np.int32), 0, cols - 1)
    ri = np.clip(((y - y_min) / res).astype(np.int32), 0, rows - 1)
    flat = ri * cols + ci                                    # 1-D index
    n_cells = rows * cols

    # ── count (keep flat until all bincount ops are done) ─────────────────────
    cnt_flat = np.bincount(flat, minlength=n_cells)          # shape (n_cells,)

    # ── z_max ─────────────────────────────────────────────────────────────────
    grid_z_max_flat = np.full(n_cells, -np.inf, dtype=np.float32)
    np.maximum.at(grid_z_max_flat, flat, z)
    grid_z_max_flat[cnt_flat == 0] = 0.0

    # ── z_variance (E[Z²] - E[Z]²) ────────────────────────────────────────────
    zf = z.astype(np.float64)
    z_sum  = np.bincount(flat, weights=zf,    minlength=n_cells)
    z_sum2 = np.bincount(flat, weights=zf**2, minlength=n_cells)
    with np.errstate(invalid='ignore', divide='ignore'):
        z_mean = np.where(cnt_flat > 0, z_sum / cnt_flat, 0.0)
        z_var  = np.where(cnt_flat > 1, z_sum2 / cnt_flat - z_mean**2, 0.0)
        z_var  = np.maximum(z_var, 0.0)

    # ── reshape everything to (rows, cols) ─────────────────────────────────────
    grid_count = cnt_flat.reshape(rows, cols)
    grid_z_max = grid_z_max_flat.reshape(rows, cols)
    grid_z_var = z_var.reshape(rows, cols).astype(np.float32)

    # ── mean scale (Gaussian uncertainty proxy) ────────────────────────────────
    grid_scale_mean = None
    # (populated by caller if scales array is provided)

    return grid_count, grid_z_max, grid_z_var, grid_scale_mean, meta


def classify(grid_count, grid_z_max, grid_z_var, args):
    """
    Returns costmap in nav2 cost units (0=free, 100=obstacle, -1/255=unknown).
    Uses int16 internally; caller converts to uint8.

    Encoding (matches nav2 map_server):
      0   = FREE
      1-99 = gradient cost
      100  = obstacle / lethal
      -1  = unknown (will be written as 205 in .pgm)
    """
    rows, cols = grid_count.shape
    cost = np.full((rows, cols), -1, dtype=np.int16)   # start: all unknown

    cnt  = grid_count
    zmax = grid_z_max
    zvar = grid_z_var

    fh = args.free_height
    fv = args.free_variance
    fc = args.free_count
    oh = args.obs_height
    ov = args.obs_variance
    lh = args.lethal_height
    mc = args.min_count

    has_data = cnt >= mc

    # LETHAL (100)
    lethal_mask = has_data & (zmax > lh)
    cost[lethal_mask] = 100

    # OBSTACLE (100)
    obs_mask = has_data & ~lethal_mask & ((zmax > oh) | (zvar > ov))
    cost[obs_mask] = 100

    # FREE (0) — flat ground, enough observations
    free_mask = has_data & ~lethal_mask & ~obs_mask & (zmax < fh) & (zvar < fv) & (cnt >= fc)
    cost[free_mask] = 0

    # GRADIENT: cells between free and obstacle thresholds get interpolated cost
    grad_mask = has_data & ~lethal_mask & ~obs_mask & ~free_mask

    # Height contribution 0–100
    h_cost = np.clip((zmax - fh) / (oh - fh + 1e-9) * 99, 0, 99).astype(np.int16)
    # Variance contribution 0–100
    v_cost = np.clip((zvar - fv) / (ov - fv + 1e-9) * 99, 0, 99).astype(np.int16)
    cost[grad_mask] = np.maximum(h_cost, v_cost)[grad_mask]

    # UNCERTAIN (-1) — too few points (already set as default)

    return cost


def inflate(cost, inflation_cells):
    """
    Expand every obstacle cell outward by inflation_cells using a square kernel.
    Inflated zone gets cost = max(existing_cost, 99) so it's "near obstacle".
    Does not overwrite FREE cells that are beyond the inflation radius of any obstacle.
    """
    if inflation_cells <= 0:
        return cost

    obs = (cost == 100).astype(np.uint8)

    # Simple distance transform via repeated dilation (small radius — fast enough)
    from PIL import Image as PILImage
    img = PILImage.fromarray(obs * 255, mode='L')
    # Expand obstacle mask by inflation_cells pixels
    from PIL.ImageFilter import MaxFilter
    kernel_size = inflation_cells * 2 + 1
    dilated = np.array(img.filter(MaxFilter(kernel_size))) > 0

    # Apply inflation cost where the dilation is True but original cost is not already 100
    inflate_mask = dilated & (cost != 100) & (cost >= 0)
    cost[inflate_mask] = np.maximum(cost[inflate_mask], 99)

    return cost


def write_pgm(cost, path):
    """
    Write Nav2 map_server .pgm (P5 binary, 8-bit).

    Nav2 conventions:
      0   → 254 (free, white)
      100 → 0   (obstacle, black)
      -1  → 205 (unknown, mid-grey)
      1-99 → interpolate 254 → 1
    """
    rows, cols = cost.shape
    pgm = np.empty((rows, cols), dtype=np.uint8)

    # Unknown
    pgm[cost < 0] = 205

    # Free → 254
    free_m = cost == 0
    pgm[free_m] = 254

    # Gradient 1-99 → 253 down to 2
    grad_m = (cost > 0) & (cost < 100)
    pgm[grad_m] = (254 - cost[grad_m] * 253 // 99).astype(np.uint8)

    # Obstacle / lethal → 0 (black)
    pgm[cost >= 100] = 0

    # Nav2 map_server expects top-row = north (y increasing upward in world).
    # Our grid row 0 = y_min (south). Flip vertically.
    pgm = np.flipud(pgm)

    with open(path, 'wb') as f:
        f.write(f"P5\n{cols} {rows}\n255\n".encode())
        f.write(pgm.tobytes())


def write_yaml(meta, resolution, output_dir, pgm_name):
    path = output_dir / 'costmap.yaml'
    content = f"""\
image: {pgm_name}
resolution: {resolution}
origin: [{meta['x_min']:.6f}, {meta['y_min']:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    path.write_text(content)
    return path


def write_debug_png(cost, path):
    """
    Color visualization:
      green  = free (0)
      red    = obstacle (100)
      orange = gradient (1-99)
      grey   = unknown (-1)
    """
    rows, cols = cost.shape
    rgb = np.zeros((rows, cols, 3), dtype=np.uint8)

    rgb[cost < 0]   = (128, 128, 128)   # unknown → grey
    rgb[cost == 0]  = (80,  200,  80)   # free    → green

    grad_m = (cost > 0) & (cost < 100)
    # interpolate green→red as cost rises 1→99
    t = cost[grad_m] / 99.0
    rgb[grad_m, 0] = (t * 220).astype(np.uint8)
    rgb[grad_m, 1] = ((1 - t) * 180).astype(np.uint8)
    rgb[grad_m, 2] = 20

    rgb[cost >= 100] = (220, 50, 50)    # obstacle → red

    # Flip so y increases upward (north = up)
    rgb = np.flipud(rgb)
    Image.fromarray(rgb, 'RGB').save(path)


def print_stats(cost):
    total   = cost.size
    free    = int((cost == 0).sum())
    obs     = int((cost >= 100).sum())
    grad    = int(((cost > 0) & (cost < 100)).sum())
    unknown = int((cost < 0).sum())
    print(f"\n  Grid statistics:")
    print(f"    Total cells : {total:,}")
    print(f"    Free        : {free:,}  ({100*free/total:.1f}%)")
    print(f"    Obstacle    : {obs:,}  ({100*obs/total:.1f}%)")
    print(f"    Gradient    : {grad:,}  ({100*grad/total:.1f}%)")
    print(f"    Unknown     : {unknown:,}  ({100*unknown/total:.1f}%)")


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('ply', type=Path, help='Input .ply file')
    p.add_argument('--resolution',       type=float, default=0.1,  metavar='M')
    p.add_argument('--output-dir',       type=Path,  default=None, metavar='DIR')
    p.add_argument('--x-min',            type=float, default=None, metavar='M')
    p.add_argument('--x-max',            type=float, default=None, metavar='M')
    p.add_argument('--y-min',            type=float, default=None, metavar='M')
    p.add_argument('--y-max',            type=float, default=None, metavar='M')
    p.add_argument('--padding',          type=float, default=1.0,  metavar='M')
    p.add_argument('--inflation-radius', type=float, default=0.2,  metavar='M')
    p.add_argument('--z-min',            type=float, default=None, metavar='M')
    p.add_argument('--z-max',            type=float, default=None, metavar='M')
    # Classification thresholds
    p.add_argument('--free-height',   type=float, default=0.5)
    p.add_argument('--free-variance', type=float, default=0.1)
    p.add_argument('--obs-height',    type=float, default=1.0)
    p.add_argument('--obs-variance',  type=float, default=0.5)
    p.add_argument('--lethal-height', type=float, default=2.0)
    p.add_argument('--min-count',     type=int,   default=3)
    p.add_argument('--free-count',    type=int,   default=5)
    return p.parse_args()


def main():
    args = parse_args()

    if not args.ply.exists():
        sys.exit(f"ERROR: {args.ply} not found")

    output_dir = args.output_dir or args.ply.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    # ── Load ──────────────────────────────────────────────────────────────────
    xyz, scales, opacs = load_ply(args.ply)

    # Optional Z clip (e.g. remove ground plane noise or high-altitude artefacts)
    if args.z_min is not None or args.z_max is not None:
        mask = np.ones(len(xyz), dtype=bool)
        if args.z_min is not None:
            mask &= xyz[:, 2] >= args.z_min
        if args.z_max is not None:
            mask &= xyz[:, 2] <= args.z_max
        xyz    = xyz[mask]
        if scales is not None: scales = scales[mask]
        if opacs  is not None: opacs  = opacs[mask]
        print(f"  After Z clip: {len(xyz):,} points remain")

    if len(xyz) == 0:
        sys.exit("ERROR: No points remain after filtering.")

    print(f"  Z range: [{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}] m")
    print(f"  X range: [{xyz[:,0].min():.3f}, {xyz[:,0].max():.3f}] m")
    print(f"  Y range: [{xyz[:,1].min():.3f}, {xyz[:,1].max():.3f}] m")

    # ── Build grid ────────────────────────────────────────────────────────────
    print("\nBuilding 2-D grid …")
    grid_count, grid_z_max, grid_z_var, _, meta = make_grid(xyz, args)

    # Optional: add per-cell mean scale into grid (for future use / debug)
    if scales is not None:
        scale_mag = np.linalg.norm(scales, axis=1)  # (N,) scalar size
        x = xyz[:, 0]; y = xyz[:, 1]
        cols_n = meta['cols']; rows_n = meta['rows']; res = meta['res']
        ci = np.clip(((x - meta['x_min']) / res).astype(np.int32), 0, cols_n - 1)
        ri = np.clip(((y - meta['y_min']) / res).astype(np.int32), 0, rows_n - 1)
        flat = ri * cols_n + ci
        n_cells = rows_n * cols_n
        cnt_flat = grid_count.ravel()
        scale_sum = np.bincount(flat, weights=scale_mag.astype(float), minlength=n_cells)
        with np.errstate(divide='ignore', invalid='ignore'):
            scale_mean = np.where(cnt_flat > 0, scale_sum / cnt_flat, 0.0)
        grid_scale_mean = scale_mean.reshape(rows_n, cols_n).astype(np.float32)
        print(f"  Gaussian scale mean range: [{grid_scale_mean.max():.4f}] m (max occupied cell)")

    # ── Classify ──────────────────────────────────────────────────────────────
    print("Classifying cells …")
    cost = classify(grid_count, grid_z_max, grid_z_var, args)

    # ── Inflate obstacles ─────────────────────────────────────────────────────
    inflation_cells = max(0, round(args.inflation_radius / args.resolution))
    if inflation_cells > 0:
        print(f"Inflating obstacles by {inflation_cells} cell(s) "
              f"({inflation_cells * args.resolution:.2f} m) …")
        cost = inflate(cost, inflation_cells)

    print_stats(cost)

    # ── Write outputs ─────────────────────────────────────────────────────────
    pgm_name  = 'costmap.pgm'
    pgm_path  = output_dir / pgm_name
    yaml_path = output_dir / 'costmap.yaml'
    png_path  = output_dir / 'costmap_debug.png'

    print(f"\nWriting outputs to {output_dir} …")
    write_pgm(cost, pgm_path)
    write_yaml(meta, args.resolution, output_dir, pgm_name)
    write_debug_png(cost, png_path)

    rows, cols = cost.shape
    print(f"  costmap.pgm      ({cols} × {rows} px)")
    print(f"  costmap.yaml     (origin=[{meta['x_min']:.3f}, {meta['y_min']:.3f}], "
          f"res={args.resolution} m)")
    print(f"  costmap_debug.png")
    print("\nDone.")
    print(f"\nTo use in Nav2:")
    print(f"  ros2 run nav2_map_server map_server --ros-args "
          f"-p yaml_filename:={yaml_path.resolve()}")


if __name__ == '__main__':
    main()
