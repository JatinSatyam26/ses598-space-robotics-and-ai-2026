#!/usr/bin/env python3
"""
Phase 21: Procedural planetary terrain heightmap generator
513x513 pixels (2^n+1 required by Gazebo)
4 layers: broad hills, craters with raised rims, rocky noise, flat landing zone
Seed=42 for reproducibility
"""
import numpy as np
from PIL import Image
import os

def generate_heightmap(size=513, seed=42):
    np.random.seed(seed)
    h = np.zeros((size, size), dtype=np.float32)
    x = np.linspace(0, 1, size)
    y = np.linspace(0, 1, size)
    xx, yy = np.meshgrid(x, y)

    # Layer 1: broad hills
    for _ in range(6):
        cx, cy = np.random.rand(), np.random.rand()
        amp = np.random.uniform(0.2, 0.5)
        width = np.random.uniform(0.1, 0.3)
        h += amp * np.exp(-((xx - cx)**2 + (yy - cy)**2) / (2 * width**2))

    # Layer 2: craters with raised rims
    for _ in range(8):
        cx, cy = np.random.rand(), np.random.rand()
        r = np.sqrt((xx - cx)**2 + (yy - cy)**2)
        depth = np.random.uniform(0.1, 0.3)
        radius = np.random.uniform(0.03, 0.08)
        h -= depth * np.exp(-r**2 / (2 * radius**2))
        h += depth * 0.4 * np.exp(-(r - radius * 1.5)**2 / (2 * (radius * 0.5)**2))

    # Layer 3: rocky noise
    noise = np.random.randn(size, size)
    from scipy.ndimage import gaussian_filter
    h += 0.05 * gaussian_filter(noise, sigma=2)

    # Layer 4: flat landing zone (center)
    cx, cy = size // 2, size // 2
    r = np.sqrt((np.arange(size)[:, None] - cx)**2 + (np.arange(size)[None, :] - cy)**2)
    landing_mask = np.clip(1.0 - r / 60.0, 0, 1)
    h = h * (1 - landing_mask * 0.8)

    # Normalize to 0-255
    h -= h.min()
    h /= h.max()
    h = (h * 255).astype(np.uint8)
    return h

if __name__ == '__main__':
    out_dir = os.path.join(os.path.dirname(__file__), 'meshes')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'heightmap.png')
    hm = generate_heightmap(513, seed=42)
    img = Image.fromarray(hm, mode='L')
    img.save(out_path)
    print(f"Heightmap saved: {out_path}")
    print(f"Size: {hm.shape}, min={hm.min()}, max={hm.max()}")
