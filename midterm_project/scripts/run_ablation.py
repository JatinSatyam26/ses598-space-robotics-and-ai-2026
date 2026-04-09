#!/usr/bin/env python3
"""
run_ablation.py — Uncertainty-awareness ablation study.

Compares two navigation strategies on the same costmap:
  A) WITH uncertainty  — unknown cells (cost -1) kept as unknown (Nav2 treats
                         them as obstacles by default via track_unknown_space=true)
  B) WITHOUT uncertainty — unknown cells treated as FREE (cost 0), rover ignores
                           areas with insufficient 3DGS coverage

Uses a Python A* planner on the costmap grid so no live Nav2 session is needed.
The A* mirrors Nav2's NavFn behaviour: diagonal moves allowed, obstacle threshold
matches occupied_thresh=0.65 (PGM value ≤ ~96 = occupied).

Inputs  (all auto-detected with defaults)
  --costmap-pgm FILE     (default: ~/midterm_reconstruction/costmap.pgm)
  --costmap-yaml FILE
  --rover-path FILE      (default: ~/midterm_results/rover_path.json, optional)
  --output-dir DIR       (default: ~/midterm_results/)

Outputs
  ablation_results.json   — metrics for both runs
  ablation_figure.png     — side-by-side paths on the two costmaps
"""

import argparse
import heapq
import json
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ── Costmap loader (same as visualize_results) ────────────────────────────────

def load_costmap(pgm_path, yaml_path):
    meta = {'resolution': 0.1, 'origin': [0.0, 0.0, 0.0],
            'occupied_thresh': 0.65, 'free_thresh': 0.196, 'negate': 0}
    if yaml_path.exists():
        for line in yaml_path.read_text().splitlines():
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if k == 'resolution':   meta['resolution'] = float(v)
            elif k == 'origin':     meta['origin'] = [float(x) for x in v.strip('[]').split(',')]
            elif k == 'negate':     meta['negate'] = int(v)
            elif k == 'occupied_thresh': meta['occupied_thresh'] = float(v)
            elif k == 'free_thresh':     meta['free_thresh'] = float(v)
    with open(pgm_path, 'rb') as f:
        magic = f.readline().decode().strip()
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()
        cols, rows = map(int, line.split())
        _ = f.readline()
        raw = f.read()
    img = np.frombuffer(raw, dtype=np.uint8).reshape(rows, cols)
    return img, meta


def pgm_to_cost_grid(img, treat_unknown_as_free=False):
    """
    Convert PGM pixel values to a traversal cost grid.
    Returns int16 grid:  0=free, 1-99=gradient, 100=obstacle, -1=unknown
    PGM convention (negate=0, write_pgm flipud):
      254 = free,  0 = obstacle,  205 = unknown,  1-253 gradient
    """
    rows, cols = img.shape
    grid = np.full((rows, cols), -1, dtype=np.int16)

    grid[img == 254] = 0              # free
    grid[img == 0]   = 100           # obstacle / lethal
    # gradient 1-253 (excluding 205 unknown): map to cost 1-99
    grad = (img > 0) & (img < 205) & (img != 0)
    grid[grad] = ((254 - img[grad].astype(np.int16)) * 99 // 253).clip(1, 99)
    # unknown (205)
    if treat_unknown_as_free:
        grid[img == 205] = 0
    else:
        grid[img == 205] = -1   # -1 = blocked for navigation

    return grid


# ── A* path planner ───────────────────────────────────────────────────────────

def world_to_cell(x, y, meta):
    """Convert world (x, y) metres to (row, col) in cost grid."""
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    rows = meta['_rows']
    col = int((x - ox) / res)
    row_from_bottom = int((y - oy) / res)
    # PGM row 0 = top (y_max); grid row 0 = bottom (y_min) → flip
    row = rows - 1 - row_from_bottom
    return row, col


def cell_to_world(row, col, meta):
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    rows = meta['_rows']
    row_from_bottom = rows - 1 - row
    x = ox + (col + 0.5) * res
    y = oy + (row_from_bottom + 0.5) * res
    return x, y


def astar(grid, start_rc, goal_rc, obstacle_thresh=99):
    """
    A* on grid (rows, cols).
    Cells with cost >= obstacle_thresh or cost == -1 are impassable.
    Returns list of (row, col) or None if no path found.
    """
    rows, cols = grid.shape
    sr, sc = start_rc
    gr, gc = goal_rc

    def h(r, c):
        return math.hypot(r - gr, c - gc)

    def neighbors(r, c):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    yield nr, nc, math.sqrt(dr*dr + dc*dc)

    open_heap = [(h(sr, sc), 0.0, sr, sc)]
    g_cost = {(sr, sc): 0.0}
    parent = {(sr, sc): None}

    while open_heap:
        _, g, r, c = heapq.heappop(open_heap)

        if (r, c) == (gr, gc):
            # Reconstruct path
            path = []
            node = (gr, gc)
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path

        if g > g_cost.get((r, c), float('inf')) + 1e-9:
            continue   # stale entry

        for nr, nc, move_cost in neighbors(r, c):
            cell_val = int(grid[nr, nc])
            if cell_val < 0 or cell_val >= obstacle_thresh:
                continue
            # Penalise high-cost cells (near obstacles)
            step = g + move_cost * (1.0 + cell_val / 50.0)
            if step < g_cost.get((nr, nc), float('inf')):
                g_cost[(nr, nc)] = step
                parent[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (step + h(nr, nc), step, nr, nc))

    return None   # no path found


# ── Path metrics ──────────────────────────────────────────────────────────────

def path_length(xy_path):
    """Total Euclidean length of a path given as (N,2) world coords."""
    if len(xy_path) < 2:
        return 0.0
    d = np.diff(xy_path, axis=0)
    return float(np.linalg.norm(d, axis=1).sum())


def path_smoothness(xy_path):
    """Mean absolute heading change (radians) — lower = smoother."""
    if len(xy_path) < 3:
        return 0.0
    vectors = np.diff(xy_path, axis=0)
    angles = np.arctan2(vectors[:, 1], vectors[:, 0])
    changes = np.abs(np.diff(angles))
    # Wrap to [-π, π]
    changes = np.minimum(changes, 2 * math.pi - changes)
    return float(changes.mean())


def unknown_cells_crossed(path_rc, grid):
    """Count cells on the path that were originally unknown (cost == -1 in base grid)."""
    return sum(1 for r, c in path_rc if grid[r, c] == -1)


# ── Visualisation ─────────────────────────────────────────────────────────────

def pgm_to_display(img):
    """Convert PGM to grayscale float [0,1] for imshow."""
    return img.astype(np.float32) / 255.0


def draw_path_on_ax(ax, path_world, color, label, linewidth=2.0):
    if path_world is None or len(path_world) < 2:
        ax.text(0.5, 0.5, 'No path found', transform=ax.transAxes,
                ha='center', va='center', color='red', fontsize=14)
        return
    arr = np.array(path_world)
    ax.plot(arr[:, 0], arr[:, 1], '-', color=color,
            linewidth=linewidth, solid_capstyle='round', label=label, zorder=3)


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    home = Path.home()
    p.add_argument('--costmap-pgm',  type=Path,
                   default=home / 'midterm_reconstruction' / 'costmap.pgm')
    p.add_argument('--costmap-yaml', type=Path, default=None)
    p.add_argument('--rover-path',   type=Path,
                   default=home / 'midterm_results' / 'rover_path.json')
    p.add_argument('--output-dir',   type=Path,
                   default=home / 'midterm_results')
    p.add_argument('--start-x', type=float, default=8.0)
    p.add_argument('--start-y', type=float, default=0.0)
    p.add_argument('--goal-x',  type=float, default=-5.0)
    p.add_argument('--goal-y',  type=float, default=-5.0)
    return p.parse_args()


def _run_astar(label, grid, base_grid, meta, start_xy, goal_xy):
    """Run A* and collect metrics. Returns dict."""
    sr, sc = world_to_cell(*start_xy, meta)
    gr, gc = world_to_cell(*goal_xy,  meta)
    rows, cols = grid.shape

    # Clamp to grid
    sr = max(0, min(sr, rows - 1))
    sc = max(0, min(sc, cols - 1))
    gr = max(0, min(gr, rows - 1))
    gc = max(0, min(gc, cols - 1))

    t0 = time.perf_counter()
    path_rc = astar(grid, (sr, sc), (gr, gc))
    elapsed = time.perf_counter() - t0

    if path_rc is None:
        print(f'  [{label}] A* found NO PATH in {elapsed:.3f}s')
        return {
            'label': label, 'found': False, 'elapsed_sec': round(elapsed, 4),
            'path_length_m': None, 'smoothness_rad': None,
            'unknown_cells_crossed': None, 'path_cells': 0,
        }

    # Convert to world coords
    path_world = [cell_to_world(r, c, meta) for r, c in path_rc]
    xy = np.array(path_world)
    length = path_length(xy)
    smooth = path_smoothness(xy)
    unk    = unknown_cells_crossed(path_rc, base_grid)

    print(f'  [{label}] A* path: {len(path_rc)} cells, '
          f'{length:.2f}m, smoothness={smooth:.4f} rad, '
          f'unknown cells crossed={unk}, time={elapsed:.3f}s')

    return {
        'label': label,
        'found': True,
        'elapsed_sec': round(elapsed, 4),
        'path_length_m': round(length, 3),
        'smoothness_rad': round(smooth, 6),
        'unknown_cells_crossed': unk,
        'path_cells': len(path_rc),
        'path_world': [[round(x, 3), round(y, 3)] for x, y in path_world],
    }


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.costmap_yaml is None:
        args.costmap_yaml = args.costmap_pgm.with_suffix('.yaml')

    # ── Load costmap ──────────────────────────────────────────────────────────
    if not args.costmap_pgm.exists():
        print(f'ERROR: costmap not found at {args.costmap_pgm}')
        print('  Run generate_costmap.py or run_colmap.sh first.')
        # Generate a synthetic 200×200 costmap for demonstration
        print('  Generating synthetic costmap for demonstration …')
        img, meta = _synthetic_costmap()
    else:
        print(f'Loading costmap: {args.costmap_pgm}')
        img, meta = load_costmap(args.costmap_pgm, args.costmap_yaml)

    rows, cols = img.shape
    meta['_rows'] = rows
    meta['_cols'] = cols

    # ── Load actual rover path if available ───────────────────────────────────
    actual_path_world = None
    if args.rover_path.exists():
        with open(args.rover_path) as f:
            rp = json.load(f)
        path_pts = rp.get('path', [])
        if path_pts:
            actual_path_world = [[p['x'], p['y']] for p in path_pts]
            args.start_x = rp.get('start', {}).get('x', args.start_x)
            args.start_y = rp.get('start', {}).get('y', args.start_y)
            args.goal_x  = rp.get('goal',  {}).get('x', args.goal_x)
            args.goal_y  = rp.get('goal',  {}).get('y', args.goal_y)
            print(f'  Loaded actual rover path: {len(path_pts)} waypoints')

    start_xy = (args.start_x, args.start_y)
    goal_xy  = (args.goal_x,  args.goal_y)

    # ── Run A (with uncertainty = unknown blocked) ────────────────────────────
    print('\n--- Run A: WITH uncertainty (unknown = blocked) ---')
    grid_a    = pgm_to_cost_grid(img, treat_unknown_as_free=False)
    result_a  = _run_astar('WITH uncertainty', grid_a, grid_a, meta,
                            start_xy, goal_xy)

    # ── Run B (without uncertainty = unknown free) ────────────────────────────
    print('\n--- Run B: WITHOUT uncertainty (unknown = free) ---')
    grid_b    = pgm_to_cost_grid(img, treat_unknown_as_free=True)
    base_grid = pgm_to_cost_grid(img, treat_unknown_as_free=False)  # for unk count
    result_b  = _run_astar('WITHOUT uncertainty', grid_b, base_grid, meta,
                            start_xy, goal_xy)

    # ── Comparison ────────────────────────────────────────────────────────────
    print('\n=== Ablation Comparison ===')
    print(f"{'Metric':<30} {'WITH unc':>12} {'WITHOUT unc':>14}")
    print('-' * 58)
    metrics_to_compare = [
        ('Path found',          'found',                    '',  ''),
        ('Path length (m)',     'path_length_m',           '.2f', '.2f'),
        ('Smoothness (rad)',    'smoothness_rad',           '.4f', '.4f'),
        ('Unknown cells crossed','unknown_cells_crossed',   'd', 'd'),
        ('Planner time (s)',    'elapsed_sec',              '.4f', '.4f'),
    ]
    for name, key, fmtA, fmtB in metrics_to_compare:
        va = result_a.get(key)
        vb = result_b.get(key)
        sa = f'{va:{fmtA}}' if (va is not None and fmtA) else str(va)
        sb = f'{vb:{fmtB}}' if (vb is not None and fmtB) else str(vb)
        print(f'  {name:<28} {sa:>12}   {sb:>12}')

    # Interpretation
    print()
    if result_a['found'] and result_b['found']:
        len_a = result_a['path_length_m']
        len_b = result_b['path_length_m']
        unk_a = result_a['unknown_cells_crossed']
        unk_b = result_b['unknown_cells_crossed']
        if len_b < len_a:
            print(f'  ✓ WITHOUT uncertainty found a {len_a-len_b:.2f}m shorter path')
            print(f'    but crossed {unk_b} unknown cells vs {unk_a} for WITH.')
        else:
            print(f'  ✓ WITH uncertainty path is {len_b-len_a:.2f}m longer but safer')
            print(f'    (avoids {unk_b - unk_a} additional unknown-coverage cells).')
        if unk_b > 0:
            print(f'  → WITHOUT-uncertainty rover may enter poorly-observed terrain!')

    # ── Save JSON ─────────────────────────────────────────────────────────────
    ablation_data = {
        'start': {'x': args.start_x, 'y': args.start_y},
        'goal':  {'x': args.goal_x,  'y': args.goal_y},
        'costmap': str(args.costmap_pgm),
        'with_uncertainty':    {k: v for k, v in result_a.items() if k != 'path_world'},
        'without_uncertainty': {k: v for k, v in result_b.items() if k != 'path_world'},
        'actual_nav2_path_available': actual_path_world is not None,
    }
    json_out = args.output_dir / 'ablation_results.json'
    with open(json_out, 'w') as f:
        json.dump(ablation_data, f, indent=2)
    print(f'\n  Saved: {json_out}')

    # ── Figure ────────────────────────────────────────────────────────────────
    print('Generating comparison figure …')
    fig, axes = plt.subplots(1, 2, figsize=(16, 7), facecolor='#1a1a2e')
    fig.suptitle('Ablation Study: Uncertainty-Aware vs Uncertainty-Ignoring Navigation',
                 color='white', fontsize=13, y=0.99)

    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    extent = [ox, ox + cols * res, oy, oy + rows * res]

    for ax, result, grid, title, path_color in [
        (axes[0], result_a, grid_a, 'A) WITH Uncertainty\n(unknown = obstacle)', '#4fc3f7'),
        (axes[1], result_b, grid_b, 'B) WITHOUT Uncertainty\n(unknown = free)',   '#ff8a65'),
    ]:
        ax.set_facecolor('#1a1a2e')
        ax.set_aspect('equal')
        ax.tick_params(colors='white')

        # Draw costmap
        disp = pgm_to_display(img)
        # Color map: 0=black (obs), 205=grey (unk), 254=white (free)
        cmap_cm = plt.cm.gray
        ax.imshow(disp, extent=extent, origin='upper',
                  cmap=cmap_cm, alpha=0.6, interpolation='nearest', zorder=1)

        # Highlight unknown cells differently per variant
        unk_mask = (img == 205).astype(np.float32)
        unk_rgba = np.zeros((*unk_mask.shape, 4), dtype=np.float32)
        if result is result_a:   # blocked
            unk_rgba[unk_mask > 0] = [0.9, 0.5, 0.0, 0.45]   # orange = blocked
        else:                    # free
            unk_rgba[unk_mask > 0] = [0.3, 0.9, 0.3, 0.25]   # green = passable
        ax.imshow(unk_rgba, extent=extent, origin='upper',
                  interpolation='nearest', zorder=2)

        # A* path
        if result['found']:
            path_w = result.get('path_world', [])
            if path_w:
                arr = np.array(path_w)
                ax.plot(arr[:, 0], arr[:, 1], '-', color=path_color,
                        linewidth=2.5, solid_capstyle='round', zorder=4,
                        label=f'A* path ({result["path_length_m"]:.1f}m)')

        # Actual Nav2 path
        if actual_path_world and result is result_a:
            ap = np.array(actual_path_world)
            ax.plot(ap[:, 0], ap[:, 1], '--', color='#e040fb',
                    linewidth=1.5, alpha=0.8, zorder=5, label='Actual Nav2 path')

        # Start / goal
        sx, sy = args.start_x, args.start_y
        gx, gy = args.goal_x, args.goal_y
        ax.scatter([sx], [sy], marker='o', s=150, color='#00e676',
                   edgecolors='white', linewidths=1.2, zorder=6)
        ax.scatter([gx], [gy], marker='*', s=250, color='#ff1744',
                   edgecolors='white', linewidths=1.0, zorder=6)
        ax.annotate('S', (sx, sy), xytext=(4, 4), textcoords='offset points',
                    color='#00e676', fontsize=9, fontweight='bold')
        ax.annotate('G', (gx, gy), xytext=(4, 4), textcoords='offset points',
                    color='#ff1744', fontsize=9, fontweight='bold')

        # Stats annotation
        if result['found']:
            stats = (f"Length: {result['path_length_m']:.2f}m\n"
                     f"Smooth: {result['smoothness_rad']:.4f} rad\n"
                     f"Unk cells: {result['unknown_cells_crossed']}\n"
                     f"Time: {result['elapsed_sec']*1000:.1f}ms")
        else:
            stats = 'NO PATH FOUND'
        ax.text(0.02, 0.02, stats, transform=ax.transAxes,
                color='white', fontsize=8, va='bottom',
                bbox=dict(boxstyle='round', facecolor='#2a2a3e', alpha=0.8))

        # Legend patches
        leg_handles = [
            mpatches.Patch(color='white', alpha=0.6, label='Free'),
            mpatches.Patch(color='black', alpha=0.6, label='Obstacle'),
            mpatches.Patch(
                color='#e07820' if result is result_a else '#30e830',
                alpha=0.6,
                label='Unknown (blocked)' if result is result_a else 'Unknown (free)'),
        ]
        if result['found']:
            leg_handles.append(plt.Line2D([0], [0], color=path_color,
                                          linewidth=2, label='A* path'))
        if actual_path_world and result is result_a:
            leg_handles.append(plt.Line2D([0], [0], color='#e040fb',
                                          linewidth=1.5, linestyle='--',
                                          label='Actual Nav2 path'))
        ax.legend(handles=leg_handles, loc='upper right',
                  facecolor='#2a2a3e', edgecolor='#555',
                  labelcolor='white', fontsize=7.5, framealpha=0.85)

        ax.set_xlabel('X (m)', color='white', fontsize=10)
        ax.set_ylabel('Y (m)', color='white', fontsize=10)
        ax.set_title(title, color='white', fontsize=11, pad=8)
        ax.grid(True, color='#333355', linewidth=0.4, alpha=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    fig_out = args.output_dir / 'ablation_figure.png'
    fig.savefig(fig_out, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'  Saved: {fig_out}')


def _synthetic_costmap():
    """Generate a synthetic 200×200 costmap for demonstration when none exists."""
    rng = np.random.default_rng(42)
    rows, cols = 200, 200
    img = np.full((rows, cols), 205, dtype=np.uint8)  # start: all unknown

    # Ground plane (free) patches
    for _ in range(20):
        r = rng.integers(10, rows - 10)
        c = rng.integers(10, cols - 10)
        h = rng.integers(15, 40)
        w = rng.integers(15, 40)
        img[max(0,r-h):r+h, max(0,c-w):c+w] = 254

    # Obstacles (rocks)
    for _ in range(8):
        r = rng.integers(20, rows - 20)
        c = rng.integers(20, cols - 20)
        rad = rng.integers(3, 10)
        rr, cc = np.ogrid[:rows, :cols]
        mask = (rr - r)**2 + (cc - c)**2 <= rad**2
        img[mask] = 0

    # Flip so top=north
    img = np.flipud(img)

    res = 0.15
    meta = {
        'resolution': res,
        'origin': [-cols * res / 2, -rows * res / 2, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0,
    }
    print('  (using synthetic 200×200 costmap for demonstration)')
    return img, meta


if __name__ == '__main__':
    main()
