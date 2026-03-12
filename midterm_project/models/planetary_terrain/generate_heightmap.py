#!/usr/bin/env python3
"""Generate a planetary-style heightmap PNG for Gazebo terrain."""

import numpy as np
from PIL import Image
import os

def generate_heightmap(size=513, output_dir='.'):
    """
    Generate a heightmap with craters, hills, and varied roughness.
    Size should be 2^n + 1 for Gazebo compatibility (e.g., 129, 257, 513).
    Pixel values: 0=lowest, 255=highest.
    """
    np.random.seed(42)  # Reproducible terrain
    terrain = np.ones((size, size), dtype=np.float64) * 128.0  # Base at mid-height

    # --- Layer 1: Broad hills (low frequency) ---
    for _ in range(4):
        cx, cy = np.random.randint(50, size - 50, 2)
        radius = np.random.randint(80, 180)
        height = np.random.uniform(15, 40)
        y, x = np.ogrid[:size, :size]
        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        mask = dist < radius
        terrain[mask] += height * np.cos(np.pi * dist[mask] / (2 * radius))

    # --- Layer 2: Craters (depressions with raised rims) ---
    for _ in range(6):
        cx, cy = np.random.randint(40, size - 40, 2)
        radius = np.random.randint(20, 60)
        depth = np.random.uniform(10, 30)
        y, x = np.ogrid[:size, :size]
        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        # Depression inside crater
        inner = dist < radius * 0.7
        terrain[inner] -= depth * (1 - dist[inner] / (radius * 0.7))
        # Raised rim
        rim = (dist >= radius * 0.7) & (dist < radius)
        terrain[rim] += depth * 0.3 * np.cos(
            np.pi * (dist[rim] - radius * 0.7) / (radius * 0.3))

    # --- Layer 3: Rocky noise (high frequency roughness) ---
    for octave in range(3):
        freq = 2 ** (octave + 3)
        amplitude = 8.0 / (octave + 1)
        noise = np.random.randn(freq, freq) * amplitude
        # Upsample noise to full resolution
        from scipy.ndimage import zoom
        upsampled = zoom(noise, size / freq, order=1)
        # Crop to exact size (zoom can produce off-by-one)
        upsampled = upsampled[:size, :size]
        terrain += upsampled

    # --- Layer 4: Flat landing zone (clear area for safe landing) ---
    # Create a relatively flat region near center
    flat_cx, flat_cy = size // 2 + 30, size // 2 - 20
    flat_radius = 35
    y, x = np.ogrid[:size, :size]
    dist = np.sqrt((x - flat_cx)**2 + (y - flat_cy)**2)
    flat_mask = dist < flat_radius
    # Smooth the terrain toward the average height in this region
    avg_h = terrain[flat_mask].mean()
    blend = np.clip(1.0 - dist / flat_radius, 0, 1)
    terrain = terrain * (1 - blend * 0.8) + avg_h * blend * 0.8

    # --- Normalize to 0-255 ---
    terrain = np.clip(terrain, 0, 255)
    terrain = ((terrain - terrain.min()) / (terrain.max() - terrain.min()) * 255).astype(np.uint8)

    # Save
    img = Image.fromarray(terrain, mode='L')
    output_path = os.path.join(output_dir, 'heightmap.png')
    img.save(output_path)
    print(f'Heightmap saved: {output_path} ({size}x{size})')
    print(f'  Height range: {terrain.min()} - {terrain.max()}')
    print(f'  Flat zone near center at ({flat_cx}, {flat_cy}), radius={flat_radius}px')
    return output_path

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    meshes_dir = os.path.join(script_dir, 'meshes')
    os.makedirs(meshes_dir, exist_ok=True)
    generate_heightmap(size=513, output_dir=meshes_dir)
