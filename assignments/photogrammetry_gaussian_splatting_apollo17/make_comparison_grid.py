import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

RENDERS_DIR = Path('/home/jatin-satyam/ses598-apollo17/renders_n15/train/rgb')
RGB_DIR = Path('/home/jatin-satyam/ses598-apollo17/data/rgb')
METRICS_DIR = Path('/home/jatin-satyam/ses598-apollo17/metrics_n15')

IMAGE_MAP = {
    'AS17-137-20903HR': 'mag137/AS17-137-20903HR.png',
    'AS17-137-20904HR': 'mag137/AS17-137-20904HR.png',
    'AS17-137-20905HR': 'mag137/AS17-137-20905HR.png',
    'AS17-137-20906HR': 'mag137/AS17-137-20906HR.png',
    'AS17-137-20907HR': 'mag137/AS17-137-20907HR.png',
    'AS17-137-20908HR': 'mag137/AS17-137-20908HR.png',
    'AS17-137-20909HR': 'mag137/AS17-137-20909HR.png',
    'AS17-138-21030HR': 'mag138/AS17-138-21030HR.png',
    'AS17-138-21031HR': 'mag138/AS17-138-21031HR.png',
    'AS17-138-21032HR': 'mag138/AS17-138-21032HR.png',
    'AS17-138-21033HR': 'mag138/AS17-138-21033HR.png',
    'AS17-138-21034HR': 'mag138/AS17-138-21034HR.png',
    'AS17-138-21035HR': 'mag138/AS17-138-21035HR.png',
    'AS17-138-21037HR': 'mag138/AS17-138-21037HR.png',
}

SELECTED = [
    ('AS17-137-20904HR', 30.18, 0.9312, 'Worst PSNR'),
    ('AS17-137-20907HR', 34.52, 0.9590, 'Median PSNR'),
    ('AS17-138-21031HR', 35.30, 0.9485, 'Random'),
    ('AS17-138-21037HR', 37.36, 0.9493, 'Best PSNR'),
]

def load_pair(key):
    rel = IMAGE_MAP[key]
    mag = rel.split('/')[0]
    render_path = RENDERS_DIR / mag / f'{key}.png'
    orig_path = RGB_DIR / rel
    render = cv2.cvtColor(cv2.imread(str(render_path)), cv2.COLOR_BGR2RGB)
    orig = cv2.cvtColor(cv2.imread(str(orig_path)), cv2.COLOR_BGR2RGB)
    rh, rw = render.shape[:2]
    orig_r = cv2.resize(orig, (rw, rh), interpolation=cv2.INTER_AREA)
    return orig_r, render

fig, axes = plt.subplots(4, 2, figsize=(14, 22))
fig.suptitle('Gaussian Splatting Reconstruction: Original vs Rendered\n(Training Views)', fontsize=14, fontweight='bold')

for row, (key, psnr, ssim, label) in enumerate(SELECTED):
    orig, render = load_pair(key)
    short = key.replace('AS17-', '').replace('HR', '')
    axes[row, 0].imshow(orig)
    axes[row, 0].set_title(f'{short} — Original', fontsize=10)
    axes[row, 0].axis('off')
    axes[row, 1].imshow(render)
    axes[row, 1].set_title(f'{short} — Rendered  [{label}]  PSNR={psnr:.2f} dB  SSIM={ssim:.4f}', fontsize=10)
    axes[row, 1].axis('off')

plt.tight_layout()
out = METRICS_DIR / 'comparison_grid.png'
plt.savefig(str(out), dpi=120, bbox_inches='tight')
plt.close()
print(f'Saved: {out}')
