#!/usr/bin/env python3
"""
compute_metrics.py — Quantitative evaluation of the full midterm pipeline.

Reconstruction metrics (from .ply):
  • Total 3D points
  • Bounding box and coverage area (XY footprint in m²)
  • Mean point density (pts/m²)
  • Z range and height distribution

Costmap metrics (from costmap.pgm + .yaml):
  • Grid size and physical dimensions
  • Free / obstacle / unknown cell percentages
  • Largest free-space connected component (reachable area in m²)

Navigation metrics (from rover_path.json):
  • Total path length (m)
  • Path smoothness — mean absolute heading change (rad)
  • Mean / min distance to nearest obstacle along path
  • Time to complete mission

Outputs
  ~/midterm_results/metrics.json   — machine-readable
  Formatted table printed to stdout

Usage
-----
  python3 compute_metrics.py [options]
  python3 compute_metrics.py --ply ~/midterm_3dgs_export/point_cloud.ply
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np


# ── Costmap / PLY loaders (same convention as other scripts) ──────────────────

def load_pgm_meta(pgm_path, yaml_path):
    meta = {'resolution': 0.1, 'origin': [0.0, 0.0, 0.0]}
    if yaml_path and yaml_path.exists():
        for line in yaml_path.read_text().splitlines():
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if k == 'resolution': meta['resolution'] = float(v)
            elif k == 'origin':   meta['origin'] = [float(x)
                                                     for x in v.strip('[]').split(',')]
    with open(pgm_path, 'rb') as f:
        _ = f.readline()  # magic
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()
        cols, rows = map(int, line.split())
        _ = f.readline()
        raw = f.read()
    img = np.frombuffer(raw, dtype=np.uint8).reshape(rows, cols)
    meta['rows'] = rows
    meta['cols'] = cols
    return img, meta


def load_ply_xyz(path):
    try:
        from plyfile import PlyData
        ply  = PlyData.read(str(path))
        vtx  = ply['vertex']
        x = vtx['x'].astype(np.float32)
        y = vtx['y'].astype(np.float32)
        z = vtx['z'].astype(np.float32)
        return np.stack([x, y, z], axis=1)
    except Exception:
        pass
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(path))
        return np.asarray(pcd.points, dtype=np.float32)
    except Exception as e:
        print(f'  WARNING: could not load PLY: {e}')
        return None


# ── Reconstruction metrics ────────────────────────────────────────────────────

def reconstruction_metrics(pts):
    if pts is None or len(pts) == 0:
        return {}
    n = len(pts)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    x_range = float(x.max() - x.min())
    y_range = float(y.max() - y.min())
    z_range = float(z.max() - z.min())
    bbox_area = x_range * y_range   # XY footprint
    density = n / max(bbox_area, 1e-6)

    # Estimate actual covered area with a coarse grid
    res = 0.5   # 0.5 m grid for coverage estimate
    xi = ((x - x.min()) / res).astype(int)
    yi = ((y - y.min()) / res).astype(int)
    occupied = set(zip(xi.tolist(), yi.tolist()))
    coverage_area = len(occupied) * res * res

    # Height percentiles
    z_pcts = np.percentile(z, [10, 50, 90]).tolist()

    return {
        'total_points':       int(n),
        'bbox_x_range_m':     round(x_range, 3),
        'bbox_y_range_m':     round(y_range, 3),
        'bbox_z_range_m':     round(z_range, 3),
        'bbox_xy_area_m2':    round(bbox_area, 2),
        'coverage_area_m2':   round(coverage_area, 2),
        'point_density_per_m2': round(density, 2),
        'z_p10_m':            round(z_pcts[0], 3),
        'z_median_m':         round(z_pcts[1], 3),
        'z_p90_m':            round(z_pcts[2], 3),
    }


# ── Costmap metrics ───────────────────────────────────────────────────────────

def _largest_connected_component(mask):
    """BFS to find largest True-connected region in a bool mask."""
    rows, cols = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best = 0
    for r0 in range(rows):
        for c0 in range(cols):
            if mask[r0, c0] and not visited[r0, c0]:
                # BFS
                queue = [(r0, c0)]
                visited[r0, c0] = True
                size = 0
                while queue:
                    r, c = queue.pop()
                    size += 1
                    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            if mask[nr, nc] and not visited[nr, nc]:
                                visited[nr, nc] = True
                                queue.append((nr, nc))
                best = max(best, size)
    return best


def costmap_metrics(img, meta):
    rows, cols = img.shape
    total = rows * cols
    res   = meta['resolution']

    n_free  = int((img == 254).sum())
    n_obs   = int((img ==   0).sum())
    n_unk   = int((img == 205).sum())
    n_grad  = int(((img > 0) & (img < 205)).sum())

    # Largest free connected component (4-connected)
    free_mask = (img == 254)
    t0 = time.perf_counter()
    lcc = _largest_connected_component(free_mask)
    lcc_time = time.perf_counter() - t0

    return {
        'grid_cols':             cols,
        'grid_rows':             rows,
        'physical_width_m':      round(cols * res, 2),
        'physical_height_m':     round(rows * res, 2),
        'resolution_m':          res,
        'total_cells':           total,
        'free_cells':            n_free,
        'obstacle_cells':        n_obs,
        'gradient_cells':        n_grad,
        'unknown_cells':         n_unk,
        'free_pct':              round(100 * n_free  / total, 2),
        'obstacle_pct':          round(100 * n_obs   / total, 2),
        'gradient_pct':          round(100 * n_grad  / total, 2),
        'unknown_pct':           round(100 * n_unk   / total, 2),
        'free_area_m2':          round(n_free  * res * res, 2),
        'obstacle_area_m2':      round(n_obs   * res * res, 2),
        'unknown_area_m2':       round(n_unk   * res * res, 2),
        'largest_free_cc_cells': lcc,
        'largest_free_cc_m2':    round(lcc * res * res, 2),
    }


# ── Navigation metrics ────────────────────────────────────────────────────────

def _path_to_array(path_list):
    return np.array([[p['x'], p['y']] for p in path_list], dtype=np.float64)


def _obstacle_distance_along_path(path_xy, img, meta):
    """
    For each path waypoint, find the distance (m) to the nearest obstacle cell.
    Returns array of distances.
    """
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    rows = meta['rows']

    # Build obstacle coordinate arrays (in world space)
    obs_mask = (img == 0)
    rr, cc = np.where(obs_mask)
    if len(rr) == 0:
        return np.full(len(path_xy), np.inf)

    # World coords of obstacle cell centres
    row_from_bottom = rows - 1 - rr
    obs_x = ox + (cc + 0.5) * res
    obs_y = oy + (row_from_bottom + 0.5) * res
    obs_xy = np.stack([obs_x, obs_y], axis=1)   # (M, 2)

    # For each path point, find min dist to any obstacle
    dists = []
    for px, py in path_xy:
        d = np.linalg.norm(obs_xy - np.array([px, py]), axis=1).min()
        dists.append(float(d))
    return np.array(dists)


def navigation_metrics(rover_data, img=None, meta=None):
    path = rover_data.get('path', [])
    if not path:
        return {'error': 'no path data'}

    xy = _path_to_array(path)
    n  = len(xy)

    # Path length
    if n >= 2:
        deltas = np.diff(xy, axis=0)
        seg_lengths = np.linalg.norm(deltas, axis=1)
        total_length = float(seg_lengths.sum())
    else:
        total_length = 0.0

    # Smoothness: mean |Δheading|
    if n >= 3:
        angles  = np.arctan2(deltas[:, 1], deltas[:, 0])
        d_angle = np.diff(angles)
        d_angle = np.where(d_angle >  math.pi, d_angle - 2*math.pi, d_angle)
        d_angle = np.where(d_angle < -math.pi, d_angle + 2*math.pi, d_angle)
        smoothness = float(np.abs(d_angle).mean())
        max_turn   = float(np.abs(d_angle).max())
    else:
        smoothness = 0.0
        max_turn   = 0.0

    # Timestamps
    t_start = path[0].get('timestamp', 0.0)
    t_end   = path[-1].get('timestamp', rover_data.get('elapsed_sec', 0.0))
    duration = t_end - t_start

    # Goal distance at completion
    final_dist = path[-1].get('dist_remaining', None)

    result = {
        'waypoints_recorded': n,
        'path_length_m':      round(total_length, 3),
        'smoothness_mean_rad':round(smoothness, 5),
        'max_turn_rad':       round(max_turn, 5),
        'elapsed_sec':        round(rover_data.get('elapsed_sec', duration), 2),
        'avg_speed_m_s':      round(total_length / max(duration, 0.01), 3),
        'recoveries':         rover_data.get('recoveries', 0),
        'success':            rover_data.get('success', False),
        'result':             rover_data.get('result', 'UNKNOWN'),
        'final_dist_remaining_m': round(final_dist, 3) if final_dist is not None else None,
    }

    # Obstacle proximity (requires costmap)
    if img is not None and meta is not None:
        dists = _obstacle_distance_along_path(xy, img, meta)
        result['obstacle_dist_mean_m'] = round(float(dists.mean()), 3)
        result['obstacle_dist_min_m']  = round(float(dists.min()),  3)
        result['obstacle_dist_p10_m']  = round(float(np.percentile(dists, 10)), 3)

    return result


# ── Pretty-printer ────────────────────────────────────────────────────────────

def _section(title):
    print(f'\n{"━"*58}')
    print(f'  {title}')
    print(f'{"━"*58}')


def _row(label, value, unit=''):
    vs = str(value) if not isinstance(value, float) else f'{value:g}'
    print(f'  {label:<36} {vs:>12}  {unit}')


def print_table(rec_m, cost_m, nav_m):
    _section('RECONSTRUCTION (3DGS Point Cloud)')
    if rec_m:
        _row('Total 3D points',      f'{rec_m["total_points"]:,}')
        _row('Coverage area',        rec_m['coverage_area_m2'],  'm²')
        _row('Bounding-box XY area', rec_m['bbox_xy_area_m2'],   'm²')
        _row('Point density',        rec_m['point_density_per_m2'], 'pts/m²')
        _row('Z range',
             f'{rec_m["bbox_z_range_m"]:.2f}',                    'm')
        _row('Z median',             rec_m['z_median_m'],         'm')
    else:
        print('  (no PLY data)')

    _section('COSTMAP')
    if cost_m:
        _row('Grid dimensions',
             f'{cost_m["grid_cols"]} × {cost_m["grid_rows"]}',   'cells')
        _row('Physical size',
             f'{cost_m["physical_width_m"]:.1f} × '
             f'{cost_m["physical_height_m"]:.1f}',               'm')
        _row('Resolution',           cost_m['resolution_m'],      'm/cell')
        _row('Free',
             f'{cost_m["free_cells"]:,}  ({cost_m["free_pct"]:.1f}%)',  '')
        _row('Obstacle',
             f'{cost_m["obstacle_cells"]:,}  ({cost_m["obstacle_pct"]:.1f}%)', '')
        _row('Unknown',
             f'{cost_m["unknown_cells"]:,}  ({cost_m["unknown_pct"]:.1f}%)', '')
        _row('Largest free region',  cost_m['largest_free_cc_m2'], 'm²')
    else:
        print('  (no costmap data)')

    _section('NAVIGATION')
    if nav_m and 'error' not in nav_m:
        _row('Result',               nav_m['result'])
        _row('Path length',          nav_m['path_length_m'],     'm')
        _row('Elapsed time',         nav_m['elapsed_sec'],        's')
        _row('Avg speed',            nav_m['avg_speed_m_s'],     'm/s')
        _row('Smoothness (mean |Δθ|)', nav_m['smoothness_mean_rad'], 'rad')
        _row('Max single turn',      nav_m['max_turn_rad'],      'rad')
        _row('Recovery attempts',    nav_m['recoveries'])
        if 'obstacle_dist_min_m' in nav_m:
            _row('Min obstacle distance',  nav_m['obstacle_dist_min_m'],  'm')
            _row('Mean obstacle distance', nav_m['obstacle_dist_mean_m'], 'm')
    else:
        print('  (no navigation data)')

    print(f'\n{"━"*58}')


# ── Entry point ────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    home = Path.home()
    p.add_argument('--ply',          type=Path,
                   default=home / 'midterm_3dgs_export' / 'point_cloud.ply')
    p.add_argument('--costmap-pgm',  type=Path,
                   default=home / 'midterm_reconstruction' / 'costmap.pgm')
    p.add_argument('--costmap-yaml', type=Path, default=None)
    p.add_argument('--rover-path',   type=Path,
                   default=home / 'midterm_results' / 'rover_path.json')
    p.add_argument('--output',       type=Path,
                   default=home / 'midterm_results' / 'metrics.json')
    return p.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    if args.costmap_yaml is None:
        args.costmap_yaml = args.costmap_pgm.with_suffix('.yaml')

    all_metrics = {}

    # ── Reconstruction ──────────────────────────────────────────────────────
    rec_m = {}
    if args.ply.exists():
        print(f'Loading PLY: {args.ply}')
        pts = load_ply_xyz(args.ply)
        if pts is not None and len(pts) > 0:
            print(f'  {len(pts):,} points loaded')
            rec_m = reconstruction_metrics(pts)
    else:
        print(f'SKIP reconstruction — {args.ply} not found')
    all_metrics['reconstruction'] = rec_m

    # ── Costmap ─────────────────────────────────────────────────────────────
    cost_m = {}
    img, meta = None, None
    if args.costmap_pgm.exists():
        print(f'Loading costmap: {args.costmap_pgm}')
        img, meta = load_pgm_meta(args.costmap_pgm, args.costmap_yaml)
        print(f'  {meta["cols"]}×{meta["rows"]} cells')
        cost_m = costmap_metrics(img, meta)
    else:
        print(f'SKIP costmap — {args.costmap_pgm} not found')
    all_metrics['costmap'] = cost_m

    # ── Navigation ──────────────────────────────────────────────────────────
    nav_m = {}
    if args.rover_path.exists():
        print(f'Loading rover path: {args.rover_path}')
        with open(args.rover_path) as f:
            rover_data = json.load(f)
        nav_m = navigation_metrics(rover_data, img, meta)
    else:
        print(f'SKIP navigation — {args.rover_path} not found')
    all_metrics['navigation'] = nav_m

    # ── Print ────────────────────────────────────────────────────────────────
    print_table(rec_m or None, cost_m or None, nav_m or None)

    # ── Save ─────────────────────────────────────────────────────────────────
    with open(args.output, 'w') as f:
        json.dump(all_metrics, f, indent=2)
    print(f'\nSaved metrics to: {args.output}')


if __name__ == '__main__':
    main()
