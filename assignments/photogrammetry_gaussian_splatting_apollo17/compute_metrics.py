import json
import csv
import numpy as np
import cv2
from pathlib import Path
from skimage.metrics import peak_signal_noise_ratio, structural_similarity
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RENDERS_DIR = Path('/home/jatin-satyam/ses598-apollo17/renders_n15/train/rgb')
RGB_DIR = Path('/home/jatin-satyam/ses598-apollo17/data/rgb')
METRICS_DIR = Path('/home/jatin-satyam/ses598-apollo17/metrics_n15')
METRICS_DIR.mkdir(parents=True, exist_ok=True)

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

render_files = sorted(RENDERS_DIR.rglob('*.png')) + sorted(RENDERS_DIR.rglob('*.jpg')) + sorted(RENDERS_DIR.rglob('*.jpeg'))
print(f'Found {len(render_files)} renders in {RENDERS_DIR}')

rows = []
for render_path in render_files:
    stem = render_path.stem
    matched_key = None
    for key in IMAGE_MAP:
        if key in stem or stem in key:
            matched_key = key
            break
    if matched_key is None:
        for key in IMAGE_MAP:
            short = key.split('-')[-1].replace('HR', '')
            if short in stem:
                matched_key = key
                break
    if matched_key is None:
        print(f'  WARNING: no match for render {stem}')
        continue
    orig_path = RGB_DIR / IMAGE_MAP[matched_key]
    if not orig_path.exists():
        print(f'  WARNING: original not found: {orig_path}')
        continue

    render_img = cv2.imread(str(render_path))
    orig_img = cv2.imread(str(orig_path))
    render_img = cv2.cvtColor(render_img, cv2.COLOR_BGR2RGB)
    orig_img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2RGB)

    render_h, render_w = render_img.shape[:2]
    orig_h, orig_w = orig_img.shape[:2]

    if (render_h, render_w) != (orig_h, orig_w):
        orig_resized = cv2.resize(orig_img, (render_w, render_h), interpolation=cv2.INTER_AREA)
        resize_note = f'original downsampled {orig_w}x{orig_h} to {render_w}x{render_h}'
    else:
        orig_resized = orig_img
        resize_note = 'same resolution'

    psnr_val = peak_signal_noise_ratio(orig_resized, render_img, data_range=255)
    ssim_val = structural_similarity(orig_resized, render_img, channel_axis=-1, data_range=255)

    rows.append({
        'image_name': matched_key,
        'render_path': str(render_path),
        'original_path': str(orig_path),
        'render_resolution': f'{render_w}x{render_h}',
        'original_resolution': f'{orig_w}x{orig_h}',
        'resize_note': resize_note,
        'psnr_db': round(float(psnr_val), 4),
        'ssim': round(float(ssim_val), 6),
    })
    print(f'  {matched_key}: PSNR={psnr_val:.2f}dB  SSIM={ssim_val:.4f}')

if not rows:
    print('ERROR: no rows computed, check renders directory')
    exit(1)

csv_path = METRICS_DIR / 'per_image_metrics.csv'
fieldnames = ['image_name', 'render_path', 'original_path', 'render_resolution', 'original_resolution', 'resize_note', 'psnr_db', 'ssim']
with open(csv_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
print(f'\nSaved: {csv_path} ({len(rows)} rows)')

psnr_vals = [r['psnr_db'] for r in rows]
ssim_vals = [r['ssim'] for r in rows]

agg = {
    'n_images': len(rows),
    'psnr_db': {
        'mean': round(float(np.mean(psnr_vals)), 4),
        'median': round(float(np.median(psnr_vals)), 4),
        'min': round(float(np.min(psnr_vals)), 4),
        'max': round(float(np.max(psnr_vals)), 4),
        'std': round(float(np.std(psnr_vals)), 4),
    },
    'ssim': {
        'mean': round(float(np.mean(ssim_vals)), 6),
        'median': round(float(np.median(ssim_vals)), 6),
        'min': round(float(np.min(ssim_vals)), 6),
        'max': round(float(np.max(ssim_vals)), 6),
        'std': round(float(np.std(ssim_vals)), 6),
    }
}

agg_path = METRICS_DIR / 'aggregate.json'
with open(agg_path, 'w') as f:
    json.dump(agg, f, indent=2)
print(f'Saved: {agg_path}')
print(f'Mean PSNR: {agg["psnr_db"]["mean"]:.2f} dB   Mean SSIM: {agg["ssim"]["mean"]:.4f}')

names = [r['image_name'].replace('AS17-', '').replace('HR', '') for r in rows]
psnr_arr = np.array(psnr_vals)
ssim_arr = np.array(ssim_vals)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
fig.suptitle('Per Image Gaussian Splatting Reconstruction Quality', fontsize=14, fontweight='bold')

x = np.arange(len(names))
bars1 = ax1.bar(x, psnr_arr, color='steelblue', edgecolor='midnightblue', linewidth=0.5)
ax1.axhline(y=agg['psnr_db']['mean'], color='goldenrod', linestyle='--', linewidth=1.5,
            label=f'Mean {agg["psnr_db"]["mean"]:.2f} dB')
ax1.set_ylabel('PSNR (dB)', fontsize=11)
ax1.set_title('Peak Signal to Noise Ratio per Training View', fontsize=12)
ax1.set_xticks(x)
ax1.set_xticklabels(names, rotation=45, ha='right', fontsize=9)
ax1.legend(fontsize=10)
ax1.set_ylim(0, max(psnr_arr) * 1.18)
for bar, val in zip(bars1, psnr_arr):
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.2,
             f'{val:.1f}', ha='center', va='bottom', fontsize=8)

bars2 = ax2.bar(x, ssim_arr, color='mediumvioletred', edgecolor='darkred', linewidth=0.5)
ax2.axhline(y=agg['ssim']['mean'], color='goldenrod', linestyle='--', linewidth=1.5,
            label=f'Mean {agg["ssim"]["mean"]:.4f}')
ax2.set_ylabel('SSIM', fontsize=11)
ax2.set_title('Structural Similarity Index per Training View', fontsize=12)
ax2.set_xticks(x)
ax2.set_xticklabels(names, rotation=45, ha='right', fontsize=9)
ax2.legend(fontsize=10)
ax2.set_ylim(0, 1.05)
for bar, val in zip(bars2, ssim_arr):
    ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
             f'{val:.3f}', ha='center', va='bottom', fontsize=8)

plt.tight_layout()
plot_path = METRICS_DIR / 'per_image_metrics.png'
plt.savefig(str(plot_path), dpi=150, bbox_inches='tight')
plt.close()
print(f'Saved: {plot_path}')

rows_sorted = sorted(rows, key=lambda r: r['psnr_db'])
print(f'\nWorst PSNR: {rows_sorted[0]["image_name"]} = {rows_sorted[0]["psnr_db"]:.2f} dB (SSIM {rows_sorted[0]["ssim"]:.4f})')
print(f'Best PSNR:  {rows_sorted[-1]["image_name"]} = {rows_sorted[-1]["psnr_db"]:.2f} dB (SSIM {rows_sorted[-1]["ssim"]:.4f})')
median_idx = len(rows_sorted) // 2
print(f'Median PSNR: {rows_sorted[median_idx]["image_name"]} = {rows_sorted[median_idx]["psnr_db"]:.2f} dB')
