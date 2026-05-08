#!/usr/bin/env python3
"""
Phase 4: Generate Golombek-Rapp scatter rock meshes for jezero_c.sdf.

Creates 3 distinct rock shapes via trimesh convex hull of perturbed spheres.
Exports OBJ to worlds/jezero_c/meshes_scatter/.
Ray-casts terrain mesh to get accurate z at each placement position.
Prints SDF <model> fragments ready to paste into jezero_c.sdf.

Placement: rig y=-2 (world y=-22) only — rover path is rig y=0..1.5,
so this lane has 2+ m clearance from rover. All rocks are on the drone's
outermost survey lane and will be detected by the downward rangefinder.

Golombek-Rapp distribution (simplified): 3 large / 3 medium / 2 small-medium.
Box collision geometry throughout — no mesh-collision RTF penalty.

Usage: python3 scripts/generate_rocks_phase4.py
"""

import os
import sys
import numpy as np
import trimesh

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
SCATTER_DIR = os.path.join(PROJECT_DIR, 'worlds', 'jezero_c', 'meshes_scatter')
TERRAIN_OBJ = os.path.join(PROJECT_DIR, 'worlds', 'jezero_c', 'terrain_mesh', 'jezero_terrain.obj')


def make_rock_mesh(seed, width_ratio=0.90, depth_ratio=0.75):
    """Convex hull of a perturbed unit-height sphere — base at z=0, centroid at xy=0."""
    rng = np.random.RandomState(seed)
    sphere = trimesh.creation.icosphere(subdivisions=2)  # 162 verts
    v = sphere.vertices.copy()
    # Scale to unit bounding box in z, with aspect ratios
    v[:, 2] *= 0.5
    v[:, 0] *= width_ratio * 0.5
    v[:, 1] *= depth_ratio * 0.5
    # Add noise to break icosphere symmetry
    v += rng.uniform(-0.10, 0.10, v.shape)
    hull = trimesh.convex.convex_hull(v)
    hull.vertices -= [hull.centroid[0], hull.centroid[1], hull.vertices[:, 2].min()]
    hull.fix_normals()
    return hull


def terrain_z_at(terrain_mesh, world_x, world_y):
    """Single downward ray cast; returns terrain surface z at (world_x, world_y)."""
    locs, _, _ = terrain_mesh.ray.intersects_location(
        ray_origins=np.array([[world_x, world_y, 10.0]]),
        ray_directions=np.array([[0.0, 0.0, -1.0]]),
        multiple_hits=False,
    )
    return float(locs[0][2]) if len(locs) else 0.0


def sdf_model(name, mesh_file, wx, wy, tz, height, width, depth, yaw):
    """SDF <model> block: mesh visual + box collision."""
    pz = tz + height / 2.0
    bw, bd, bh = width * 0.88, depth * 0.88, height * 0.88  # slightly inset box
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{wx:.3f} {wy:.3f} {pz:.4f} 0 0 {yaw:.2f}</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="col">\n'
        f'          <geometry><box><size>{bw:.3f} {bd:.3f} {bh:.3f}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <visual name="vis">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>file://jezero_c/meshes_scatter/{mesh_file}</uri>\n'
        f'              <scale>{width:.4f} {depth:.4f} {height:.4f}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.42 0.28 0.17 1</ambient>\n'
        f'            <diffuse>0.53 0.36 0.22 1</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def main():
    os.makedirs(SCATTER_DIR, exist_ok=True)

    # --- 3 distinct rock shapes (unit height = 1.0 m in z) ---
    shapes = {
        'rock_A': make_rock_mesh(seed=42,  width_ratio=0.92, depth_ratio=0.78),  # wide/squat
        'rock_B': make_rock_mesh(seed=137, width_ratio=0.85, depth_ratio=0.70),  # balanced
        'rock_C': make_rock_mesh(seed=271, width_ratio=0.80, depth_ratio=0.65),  # tall/narrow
    }

    print('Generating OBJ files...')
    for sname, mesh in shapes.items():
        path = os.path.join(SCATTER_DIR, f'{sname}.obj')
        mesh.export(path)
        bb = mesh.bounding_box.extents
        print(f'  {sname}.obj  {len(mesh.vertices)} verts  {len(mesh.faces)} faces'
              f'  unit bbox: {bb[0]:.3f} x {bb[1]:.3f} x {bb[2]:.3f} m')

    print(f'\nLoading terrain mesh from {TERRAIN_OBJ}...')
    terrain = trimesh.load(TERRAIN_OBJ, process=False)
    print(f'  {len(terrain.vertices)} verts, {len(terrain.faces)} faces')

    # --- Rock placement: all at rig y=-2 (world y=-22) ---
    # Rig offset (5, -20): world_x = rig_x + 5, world_y = rig_y - 20
    # Golombek-Rapp sizing: 3 large / 3 medium / 2 small-medium (power-law from large)
    WY = -22.0   # world y for all rocks
    rocks = [
        # rig_x  shape      height  yaw   label
        (3,  'rock_A', 0.62, 0.00, 'large'),
        (5,  'rock_B', 0.45, 0.70, 'medium'),
        (7,  'rock_C', 0.34, 1.20, 'small_med'),
        (9,  'rock_A', 0.65, 0.30, 'large'),
        (11, 'rock_B', 0.48, 1.80, 'medium'),
        (13, 'rock_C', 0.36, 0.90, 'small_med'),
        (15, 'rock_A', 0.60, 2.10, 'large'),
        (17, 'rock_B', 0.42, 1.40, 'medium'),
    ]

    print('\nLooking up terrain z at each rock position...')
    fragments = []
    for i, (rig_x, sname, height, yaw, label) in enumerate(rocks):
        wx = rig_x + 5.0
        tz = terrain_z_at(terrain, wx, WY)
        mesh = shapes[sname]
        bb = mesh.bounding_box.extents  # unit bbox
        width, depth = bb[0] * height, bb[1] * height
        rname = f'scatter_{i+1:02d}'
        frag = sdf_model(rname, f'{sname}.obj', wx, WY, tz, height, width, depth, yaw)
        fragments.append(frag)
        print(f'  {rname}: rig({rig_x},-2) world({wx:.0f},-22)'
              f'  tz={tz:.4f}  h={height}m [{label}]')

    # --- Print SDF fragment ---
    print()
    print('=' * 72)
    print('SDF fragment — paste before </world> in jezero_c.sdf:')
    print('=' * 72)
    print()
    header = (
        '    <!-- Phase 4: Golombek-Rapp scatter rocks (Apr 22 2026).\n'
        '         8 rocks at rig y=-2 (world y=-22), rig x=3..17 (2m spacing).\n'
        '         3 shapes (A squat / B balanced / C narrow), 3 size classes.\n'
        '         Golombek-Rapp: 3 large (h=0.60-0.65m) / 3 medium (h=0.42-0.48m)\n'
        '                        / 2 small-med (h=0.34-0.36m).\n'
        '         Survey lane rig y=-2: drone rangefinder flies directly overhead.\n'
        '         Rover path is rig y=0..1.5: 2+ m clearance, no collision risk.\n'
        '         Visual: trimesh convex-hull OBJ (scaled per instance).\n'
        '         Collision: inset box hull — zero mesh-collision RTF penalty. -->'
    )
    print(header)
    for frag in fragments:
        print()
        print(frag)
    print()
    print('=' * 72)
    print(f'Meshes written to: {SCATTER_DIR}')


if __name__ == '__main__':
    main()
