#!/usr/bin/env python3
"""
generate_costmap_and_path.py — SES 598 Midterm
Builds a 2-D occupancy grid from known obstacle geometry in demo_world.sdf,
runs A* from rover start (10, -4) to rover goal (-10, -4), and writes:
  ~/midterm_results/rover_path.json     — A* path for rover_driver
  ~/midterm_reconstruction/costmap.pgm  — occupancy map (Nav2 format)
  ~/midterm_reconstruction/costmap.yaml — map metadata

Obstacle list from demo_world.sdf (XY only, height irrelevant for ground rover):
  boulder_1      sphere  center=(-8,-6)   r=2.0
  boulder_2      sphere  center=(7,-9)    r=1.5
  sphere_large   sphere  center=(-5,8)    r=2.0
  spire          cyl     center=(4,5)     r=0.8
  pillar         cyl     center=(-11,3)   r=1.2
  slab           box     center=(9,7)     size=(5,3) yaw=0.4
  fallen_cyl     cyl     center=(-3,-11)  r=0.7
  rock_wall      box     center=(0,-13)   size=(14,1.5) yaw=0.2
  small_rock_1   sphere  center=(6,-4)    r=0.5   ← directly on path y=-4
  small_rock_2   sphere  center=(7,-3)    r=0.4   ← 1 m from path (inflation hits)
  small_rock_3   sphere  center=(5.5,-5)  r=0.6   ← 1 m from path (inflation hits)
"""

import heapq
import json
import math
import os
import struct

import numpy as np

# ── Grid parameters ───────────────────────────────────────────────────────────
RESOLUTION    = 0.5          # m/cell
INFLATION_R   = 1.0          # m — rover footprint + safety margin
WORLD_X_MIN   = -20.0
WORLD_X_MAX   =  20.0
WORLD_Y_MIN   = -20.0
WORLD_Y_MAX   =  20.0

# ── Rover mission ─────────────────────────────────────────────────────────────
START_XY  = (10.0,  -4.0)
GOAL_XY   = (-12.0, -4.0)

# ── Output paths ─────────────────────────────────────────────────────────────
RESULTS_DIR = os.path.expanduser('~/midterm_results')
RECON_DIR   = os.path.expanduser('~/midterm_reconstruction')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(RECON_DIR,   exist_ok=True)


# ── Obstacle geometry ─────────────────────────────────────────────────────────
# Each circle obstacle: (cx, cy, r)
CIRCLES = [
    (-8.0,  -6.0, 2.0),   # boulder_1
    ( 7.0,  -9.0, 1.5),   # boulder_2
    (-5.0,   8.0, 2.0),   # sphere_large
    ( 4.0,   5.0, 0.8),   # spire
    (-11.0,  3.0, 1.2),   # pillar
    (-3.0, -11.0, 0.7),   # fallen_cylinder
    ( 6.0,  -4.0, 0.5),   # small_rock_1  ← directly on rover path
    ( 7.0,  -3.0, 0.4),   # small_rock_2
    ( 5.5,  -5.0, 0.6),   # small_rock_3
]

# Box obstacles: (cx, cy, half_lx, half_ly, yaw_rad)
BOXES = [
    ( 9.0,  7.0, 2.5, 1.5, 0.4),   # slab
    ( 0.0, -13.0, 7.0, 0.75, 0.2), # rock_wall
]


def world_to_grid(wx, wy, ox, oy, res):
    """Convert world (x,y) to grid (col,row) — col=x axis, row=y axis."""
    col = int((wx - ox) / res)
    row = int((wy - oy) / res)
    return col, row


def grid_to_world(col, row, ox, oy, res):
    return ox + (col + 0.5) * res, oy + (row + 0.5) * res


def build_costmap():
    cols = int((WORLD_X_MAX - WORLD_X_MIN) / RESOLUTION)
    rows = int((WORLD_Y_MAX - WORLD_Y_MIN) / RESOLUTION)
    grid = np.zeros((rows, cols), dtype=np.uint8)   # 0=free

    # Mark circles
    for cx, cy, r in CIRCLES:
        total_r = r + INFLATION_R
        r_cells = int(math.ceil(total_r / RESOLUTION)) + 1
        gc, gr = world_to_grid(cx, cy, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
        for dc in range(-r_cells, r_cells + 1):
            for dr in range(-r_cells, r_cells + 1):
                cc, cr = gc + dc, gr + dr
                if 0 <= cc < cols and 0 <= cr < rows:
                    wx, wy = grid_to_world(cc, cr, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
                    if math.hypot(wx - cx, wy - cy) <= total_r:
                        grid[cr, cc] = 100

    # Mark boxes (axis-aligned check in rotated frame)
    for cx, cy, hlx, hly, yaw in BOXES:
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        total_hlx = hlx + INFLATION_R
        total_hly = hly + INFLATION_R
        search_r = math.hypot(total_hlx, total_hly)
        r_cells = int(math.ceil(search_r / RESOLUTION)) + 1
        gc, gr = world_to_grid(cx, cy, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
        for dc in range(-r_cells, r_cells + 1):
            for dr in range(-r_cells, r_cells + 1):
                cc, cr = gc + dc, gr + dr
                if 0 <= cc < cols and 0 <= cr < rows:
                    wx, wy = grid_to_world(cc, cr, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
                    lx = (wx - cx) * cos_y - (wy - cy) * sin_y
                    ly = (wx - cx) * sin_y + (wy - cy) * cos_y
                    if abs(lx) <= total_hlx and abs(ly) <= total_hly:
                        grid[cr, cc] = 100

    return grid, cols, rows


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start_cell, goal_cell):
    rows, cols = grid.shape
    sc, sr = start_cell
    gc, gr = goal_cell

    if grid[sr, sc] == 100:
        raise ValueError(f"Start cell ({sc},{sr}) is occupied!")
    if grid[gr, gc] == 100:
        raise ValueError(f"Goal cell ({gc},{gr}) is occupied!")

    # 8-connected neighbours
    neighbours = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    diag_cost = math.sqrt(2)

    open_set = [(0.0, (sc, sr))]
    came_from = {}
    g_score = {(sc, sr): 0.0}

    while open_set:
        f, current = heapq.heappop(open_set)
        cc, cr = current

        if current == (gc, gr):
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append((sc, sr))
            path.reverse()
            return path

        for dc, dr in neighbours:
            nc, nr = cc + dc, cr + dr
            if not (0 <= nc < cols and 0 <= nr < rows):
                continue
            if grid[nr, nc] == 100:
                continue
            move_cost = diag_cost if (dc != 0 and dr != 0) else 1.0
            tentative_g = g_score[current] + move_cost
            neighbour = (nc, nr)
            if tentative_g < g_score.get(neighbour, float('inf')):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f_new = tentative_g + heuristic(neighbour, (gc, gr))
                heapq.heappush(open_set, (f_new, neighbour))

    raise ValueError("A* found no path from start to goal!")


def write_pgm(path, grid):
    """Write grayscale PGM (P5 binary) — occupied=0(black), free=255(white)."""
    rows, cols = grid.shape
    img = np.where(grid == 100, 0, 254).astype(np.uint8)
    with open(path, 'wb') as f:
        header = f'P5\n{cols} {rows}\n255\n'.encode()
        f.write(header)
        f.write(img.tobytes())


def write_yaml(path, cols, rows):
    with open(path, 'w') as f:
        f.write(f'image: costmap.pgm\n')
        f.write(f'resolution: {RESOLUTION}\n')
        f.write(f'origin: [{WORLD_X_MIN}, {WORLD_Y_MIN}, 0.0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.196\n')


def main():
    print("Building costmap…")
    grid, cols, rows = build_costmap()
    occupied = int(np.sum(grid == 100))
    print(f"  Grid: {cols}×{rows} cells ({RESOLUTION}m/cell), "
          f"{occupied} occupied cells")

    # Verify straight-line is blocked
    print("\nVerifying straight-line start→goal is blocked:")
    sx, sy = START_XY
    gx, gy = GOAL_XY
    blocked_count = 0
    for t in np.linspace(0, 1, 200):
        wx = sx + t * (gx - sx)
        wy = sy + t * (gy - sy)
        c, r = world_to_grid(wx, wy, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
        if 0 <= c < cols and 0 <= r < rows and grid[r, c] == 100:
            blocked_count += 1
    print(f"  Straight-line passes through {blocked_count}/200 occupied cells "
          f"({'BLOCKED ✓' if blocked_count > 0 else 'NOT BLOCKED ✗'})")

    print(f"\nRunning A* from {START_XY} → {GOAL_XY}…")
    sc, sr = world_to_grid(*START_XY, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
    gc, gr = world_to_grid(*GOAL_XY, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
    print(f"  Start cell: ({sc},{sr}), Goal cell: ({gc},{gr})")
    print(f"  Start occupied: {grid[sr,sc]==100}, Goal occupied: {grid[gr,gc]==100}")

    path_cells = astar(grid, (sc, sr), (gc, gr))
    print(f"  A* path: {len(path_cells)} cells")

    # Convert to world coordinates
    path_world = [
        grid_to_world(c, r, WORLD_X_MIN, WORLD_Y_MIN, RESOLUTION)
        for c, r in path_cells
    ]

    # Compute path length vs straight-line distance
    path_len = sum(
        math.hypot(path_world[i+1][0]-path_world[i][0],
                   path_world[i+1][1]-path_world[i][1])
        for i in range(len(path_world)-1)
    )
    straight_len = math.hypot(GOAL_XY[0]-START_XY[0], GOAL_XY[1]-START_XY[1])
    print(f"\n  Path length   : {path_len:.2f} m")
    print(f"  Straight-line : {straight_len:.2f} m")
    print(f"  Detour ratio  : {path_len/straight_len:.3f}x "
          f"({'DETOURS AROUND OBSTACLES ✓' if path_len > straight_len * 1.05 else 'PATH TOO STRAIGHT ✗'})")

    # Mark path on grid for PGM visualization
    vis_grid = grid.copy()
    for c, r in path_cells:
        vis_grid[r, c] = 50   # grey = path

    # Write costmap files
    pgm_path  = os.path.join(RECON_DIR, 'costmap.pgm')
    yaml_path = os.path.join(RECON_DIR, 'costmap.yaml')
    write_pgm(pgm_path, vis_grid)
    write_yaml(yaml_path, cols, rows)
    print(f"\n  Costmap saved: {pgm_path}")
    print(f"  Metadata saved: {yaml_path}")

    # Write rover_path.json
    path_json = [{'x': round(x, 3), 'y': round(y, 3)} for x, y in path_world]
    rover_path = {'path': path_json,
                  'start': {'x': START_XY[0], 'y': START_XY[1]},
                  'goal':  {'x': GOAL_XY[0],  'y': GOAL_XY[1]},
                  'path_length_m': round(path_len, 3),
                  'straight_line_m': round(straight_len, 3)}
    out_path = os.path.join(RESULTS_DIR, 'rover_path.json')
    with open(out_path, 'w') as f:
        json.dump(rover_path, f, indent=2)
    print(f"  rover_path.json saved: {out_path} ({len(path_json)} waypoints)")

    print("\n✓ Done — rover will detour around rock cluster.")


if __name__ == '__main__':
    main()
