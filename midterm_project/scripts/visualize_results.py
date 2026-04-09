#!/usr/bin/env python3
"""
visualize_results.py — Bird's-eye-view composite of the full midterm pipeline.

Layers (bottom to top):
  1. 3DGS / point-cloud projected to XY, coloured by height
  2. Costmap as semi-transparent RGBA overlay (green/red/gray)
  3. Drone survey path (dashed gray, from poses.json)
  4. Rover path (solid blue, from rover_path.json)
  5. Start (green ●) and goal (red ★) markers

All inputs are optional — missing files produce a warning and the layer is skipped
so the figure is always generated even before the full pipeline has run.

Usage
-----
  python3 visualize_results.py [options]

Options
  --ply FILE            3DGS / point-cloud .ply  (default: ~/midterm_3dgs_export/point_cloud.ply)
  --costmap-pgm FILE    costmap.pgm               (default: ~/midterm_reconstruction/costmap.pgm)
  --costmap-yaml FILE   costmap.yaml              (default: same dir as pgm)
  --rover-path FILE     rover_path.json           (default: ~/midterm_results/rover_path.json)
  --poses FILE          poses.json                (default: ~/midterm_flight_data/poses.json)
  --output FILE         output PNG                (default: ~/midterm_results/birdseye_result.png)
  --dpi N               figure DPI                (default: 150)
  --no-pointcloud       skip point-cloud layer (faster for large PLYs)
  --max-points N        max points to plot        (default: 50000)
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')   # headless
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import numpy as np


# ── Data loaders ───────────────────────────────────────────────────────────────

def load_ply_xyz(path, max_points=50_000):
    """Return (N,3) float32 array of XYZ, sub-sampled to max_points."""
    try:
        from plyfile import PlyData
        ply = PlyData.read(str(path))
        vtx = ply['vertex']
        x = vtx['x'].astype(np.float32)
        y = vtx['y'].astype(np.float32)
        z = vtx['z'].astype(np.float32)
        pts = np.stack([x, y, z], axis=1)
    except Exception:
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(str(path))
            pts = np.asarray(pcd.points, dtype=np.float32)
        except Exception as e:
            print(f'  WARNING: could not load {path}: {e}')
            return None
    if len(pts) > max_points:
        idx = np.random.default_rng(0).choice(len(pts), max_points, replace=False)
        pts = pts[idx]
    print(f'  Loaded {len(pts):,} points from {path.name}')
    return pts


def load_costmap(pgm_path, yaml_path):
    """Return (img_array, meta_dict) where img_array is uint8 (rows×cols)."""
    # Parse YAML for origin and resolution
    meta = {'resolution': 0.1, 'origin': [0.0, 0.0, 0.0],
            'occupied_thresh': 0.65, 'free_thresh': 0.196,
            'negate': 0}
    if yaml_path.exists():
        for line in yaml_path.read_text().splitlines():
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if k == 'resolution':
                meta['resolution'] = float(v)
            elif k == 'origin':
                meta['origin'] = [float(x) for x in v.strip('[]').split(',')]
            elif k == 'negate':
                meta['negate'] = int(v)
            elif k == 'occupied_thresh':
                meta['occupied_thresh'] = float(v)
            elif k == 'free_thresh':
                meta['free_thresh'] = float(v)

    # Parse PGM (binary P5)
    with open(pgm_path, 'rb') as f:
        magic = f.readline().decode().strip()
        if magic not in ('P5', 'P2'):
            raise ValueError(f'Not a PGM file: {magic}')
        # Skip comments
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()
        cols, rows = map(int, line.split())
        maxval = int(f.readline().decode().strip())
        raw = f.read()
    img = np.frombuffer(raw, dtype=np.uint8).reshape(rows, cols)
    print(f'  Costmap: {cols}×{rows} cells, res={meta["resolution"]}m, '
          f'origin=({meta["origin"][0]:.2f},{meta["origin"][1]:.2f})')
    return img, meta


def costmap_to_rgba(img, meta):
    """
    Convert PGM values to an RGBA array for overlay.
    Nav2 map_server convention (negate=0):
      254        = free     → transparent green
      0          = obstacle → semi-transparent red
      205        = unknown  → semi-transparent grey
      1..253     = gradient → orange, opacity proportional
    """
    rows, cols = img.shape
    rgba = np.zeros((rows, cols, 4), dtype=np.float32)

    free_m  = img == 254
    obs_m   = img == 0
    unk_m   = img == 205
    grad_m  = (img > 0) & (img < 205)

    # free: light green, low alpha
    rgba[free_m]  = [0.2, 0.8, 0.2, 0.25]
    # obstacle: red, higher alpha
    rgba[obs_m]   = [0.9, 0.1, 0.1, 0.65]
    # unknown: grey, medium alpha
    rgba[unk_m]   = [0.5, 0.5, 0.5, 0.30]
    # gradient: orange, alpha proportional to cost
    t = (254 - img[grad_m].astype(np.float32)) / 253.0   # 0=free,1=obs
    rgba[grad_m, 0] = 0.9
    rgba[grad_m, 1] = 0.6 * (1 - t)
    rgba[grad_m, 2] = 0.0
    rgba[grad_m, 3] = 0.2 + 0.45 * t

    return rgba


def load_poses_path(poses_path):
    """Return (N,2) XY array of drone survey waypoints from poses.json."""
    with open(poses_path) as f:
        entries = json.load(f)
    if not entries:
        return None
    coords = []
    for e in entries:
        if 'odom' in e and e['odom'] is not None:
            pos = e['odom']['position']
            coords.append([pos[0], pos[1]])
        elif 'position' in e and e['position'] is not None:
            pos = e['position']
            coords.append([pos[0], pos[1]])
    if not coords:
        return None
    return np.array(coords, dtype=np.float32)


def load_rover_path(rover_path_json):
    """Return (waypoints_array, meta_dict) from rover_path.json."""
    with open(rover_path_json) as f:
        data = json.load(f)
    pts = data.get('path', [])
    if not pts:
        return None, data
    arr = np.array([[p['x'], p['y']] for p in pts], dtype=np.float32)
    return arr, data


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    home = Path.home()
    p.add_argument('--ply',           type=Path,
                   default=home / 'midterm_3dgs_export' / 'point_cloud.ply')
    p.add_argument('--costmap-pgm',   type=Path,
                   default=home / 'midterm_reconstruction' / 'costmap.pgm')
    p.add_argument('--costmap-yaml',  type=Path, default=None)
    p.add_argument('--rover-path',    type=Path,
                   default=home / 'midterm_results' / 'rover_path.json')
    p.add_argument('--poses',         type=Path,
                   default=home / 'midterm_flight_data' / 'poses.json')
    p.add_argument('--output',        type=Path,
                   default=home / 'midterm_results' / 'birdseye_result.png')
    p.add_argument('--dpi',           type=int,  default=150)
    p.add_argument('--no-pointcloud', action='store_true')
    p.add_argument('--max-points',    type=int,  default=50_000)
    return p.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    if args.costmap_yaml is None:
        args.costmap_yaml = args.costmap_pgm.with_suffix('.yaml')

    fig, ax = plt.subplots(figsize=(12, 10), facecolor='#1a1a2e')
    ax.set_facecolor('#1a1a2e')
    ax.set_aspect('equal')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#444')

    legend_handles = []
    x_min = y_min = x_max = y_max = None

    def _expand_bounds(xs, ys):
        nonlocal x_min, y_min, x_max, y_max
        x_min = min(x_min, xs.min()) if x_min is not None else xs.min()
        x_max = max(x_max, xs.max()) if x_max is not None else xs.max()
        y_min = min(y_min, ys.min()) if y_min is not None else ys.min()
        y_max = max(y_max, ys.max()) if y_max is not None else ys.max()

    # ── Layer 1: Point cloud ───────────────────────────────────────────────────
    pts = None
    if not args.no_pointcloud and args.ply.exists():
        print(f'Loading point cloud: {args.ply}')
        pts = load_ply_xyz(args.ply, args.max_points)

    if pts is not None and len(pts) > 0:
        z = pts[:, 2]
        z_norm = (z - z.min()) / max(z.ptp(), 0.01)
        colors = plt.cm.plasma(z_norm)
        ax.scatter(pts[:, 0], pts[:, 1], c=colors, s=0.5, alpha=0.4,
                   linewidths=0, rasterized=True, zorder=1)
        sm = plt.cm.ScalarMappable(cmap='plasma',
                                   norm=mcolors.Normalize(z.min(), z.max()))
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=ax, fraction=0.025, pad=0.02)
        cbar.set_label('Height (m)', color='white', fontsize=9)
        cbar.ax.yaxis.set_tick_params(color='white')
        plt.setp(cbar.ax.yaxis.get_ticklabels(), color='white')
        legend_handles.append(mpatches.Patch(color=plt.cm.plasma(0.5),
                                             label=f'Point cloud ({len(pts):,} pts)'))
        _expand_bounds(pts[:, 0], pts[:, 1])
    elif not args.ply.exists():
        print(f'  SKIP point cloud — {args.ply} not found')

    # ── Layer 2: Costmap overlay ───────────────────────────────────────────────
    costmap_meta = None
    if args.costmap_pgm.exists():
        print(f'Loading costmap: {args.costmap_pgm}')
        try:
            img, costmap_meta = load_costmap(args.costmap_pgm, args.costmap_yaml)
            rgba = costmap_to_rgba(img, costmap_meta)
            res  = costmap_meta['resolution']
            ox, oy = costmap_meta['origin'][0], costmap_meta['origin'][1]
            rows, cols = img.shape
            extent = [ox, ox + cols * res, oy, oy + rows * res]
            # PGM is stored with row 0 = north (y_max) due to our flipud in write_pgm
            ax.imshow(rgba, extent=extent, origin='upper',
                      interpolation='nearest', zorder=2)
            _expand_bounds(np.array([ox, ox + cols * res]),
                           np.array([oy, oy + rows * res]))
            legend_handles += [
                mpatches.Patch(facecolor=(0.2, 0.8, 0.2, 0.5), label='Free'),
                mpatches.Patch(facecolor=(0.9, 0.1, 0.1, 0.7), label='Obstacle'),
                mpatches.Patch(facecolor=(0.5, 0.5, 0.5, 0.4), label='Unknown'),
            ]
        except Exception as e:
            print(f'  WARNING: costmap load failed: {e}')
    else:
        print(f'  SKIP costmap — {args.costmap_pgm} not found')

    # ── Layer 3: Drone survey path ─────────────────────────────────────────────
    if args.poses.exists():
        print(f'Loading drone poses: {args.poses}')
        try:
            drone_xy = load_poses_path(args.poses)
            if drone_xy is not None and len(drone_xy) > 1:
                ax.plot(drone_xy[:, 0], drone_xy[:, 1],
                        '--', color='#aaaaaa', linewidth=0.8, alpha=0.6,
                        zorder=3, label='Drone survey')
                ax.scatter(drone_xy[0, 0], drone_xy[0, 1],
                           marker='^', color='#cccccc', s=80, zorder=4)
                legend_handles.append(
                    plt.Line2D([0], [0], linestyle='--', color='#aaaaaa',
                               linewidth=1.2, label='Drone survey path'))
                _expand_bounds(drone_xy[:, 0], drone_xy[:, 1])
                print(f'  {len(drone_xy)} drone waypoints')
            else:
                print('  SKIP drone path — poses.json has no valid odom data')
        except Exception as e:
            print(f'  WARNING: drone poses failed: {e}')
    else:
        print(f'  SKIP drone path — {args.poses} not found')

    # ── Layer 4: Rover path ────────────────────────────────────────────────────
    rover_meta = None
    if args.rover_path.exists():
        print(f'Loading rover path: {args.rover_path}')
        try:
            rover_xy, rover_meta = load_rover_path(args.rover_path)
            if rover_xy is not None and len(rover_xy) > 0:
                ax.plot(rover_xy[:, 0], rover_xy[:, 1],
                        '-', color='#4fc3f7', linewidth=2.0, zorder=5,
                        solid_capstyle='round')
                legend_handles.append(
                    plt.Line2D([0], [0], color='#4fc3f7', linewidth=2,
                               label=f'Rover path ({len(rover_xy)} pts)'))
                _expand_bounds(rover_xy[:, 0], rover_xy[:, 1])

            # Start / goal from metadata
            sx = rover_meta.get('start', {}).get('x', 8.0)
            sy = rover_meta.get('start', {}).get('y', 0.0)
            gx = rover_meta.get('goal',  {}).get('x', -5.0)
            gy = rover_meta.get('goal',  {}).get('y', -5.0)
        except Exception as e:
            print(f'  WARNING: rover path failed: {e}')
            sx, sy, gx, gy = 8.0, 0.0, -5.0, -5.0
    else:
        print(f'  SKIP rover path — {args.rover_path} not found')
        sx, sy, gx, gy = 8.0, 0.0, -5.0, -5.0
        rover_meta = None

    # ── Layer 5: Start / goal markers ─────────────────────────────────────────
    ax.scatter([sx], [sy], marker='o', s=200, color='#00e676',
               edgecolors='white', linewidths=1.5, zorder=6)
    ax.scatter([gx], [gy], marker='*', s=350, color='#ff1744',
               edgecolors='white', linewidths=1.0, zorder=6)
    ax.annotate('START', (sx, sy), textcoords='offset points', xytext=(8, 6),
                color='#00e676', fontsize=8, fontweight='bold')
    ax.annotate('GOAL',  (gx, gy), textcoords='offset points', xytext=(8, 6),
                color='#ff1744', fontsize=8, fontweight='bold')
    legend_handles += [
        mpatches.Patch(color='#00e676', label=f'Start ({sx:.1f}, {sy:.1f})'),
        mpatches.Patch(color='#ff1744', label=f'Goal  ({gx:.1f}, {gy:.1f})'),
    ]
    _expand_bounds(np.array([sx, gx]), np.array([sy, gy]))

    # ── Axes / title ───────────────────────────────────────────────────────────
    if x_min is not None:
        pad = 2.0
        ax.set_xlim(x_min - pad, x_max + pad)
        ax.set_ylim(y_min - pad, y_max + pad)

    ax.set_xlabel('X (m)', color='white', fontsize=11)
    ax.set_ylabel('Y (m)', color='white', fontsize=11)
    ax.grid(True, color='#333355', linewidth=0.5, alpha=0.5)

    # Title: embed mission result if available
    title = 'SES 598 Midterm — Bird\'s-Eye View'
    if rover_meta:
        result = rover_meta.get('result', '?')
        dist   = rover_meta.get('total_dist_traveled_m', 0)
        t_sec  = rover_meta.get('elapsed_sec', 0)
        title += f'\nRover: {result}  |  {dist:.1f}m traveled  |  {t_sec:.0f}s'
    ax.set_title(title, color='white', fontsize=12, pad=12)

    leg = ax.legend(handles=legend_handles, loc='upper left',
                    facecolor='#2a2a3e', edgecolor='#555', labelcolor='white',
                    fontsize=8, framealpha=0.85)

    plt.tight_layout()
    fig.savefig(args.output, dpi=args.dpi, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'\nSaved: {args.output}')


if __name__ == '__main__':
    main()
