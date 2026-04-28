import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path

OUT_DIR = Path('/home/jatin-satyam/ses598-apollo17/novel_poses')

frames = [
    ('novel_mag137_01.png', 'mag137_01  -30 deg'),
    ('novel_mag137_02.png', 'mag137_02  -15 deg'),
    ('novel_mag137_03.png', 'mag137_03    0 deg'),
    ('novel_mag137_04.png', 'mag137_04  +15 deg'),
    ('novel_mag137_05.png', 'mag137_05  +30 deg'),
    ('novel_mag138_01.png', 'mag138_01  -30 deg'),
    ('novel_mag138_02.png', 'mag138_02  -15 deg'),
    ('novel_mag138_03.png', 'mag138_03    0 deg'),
    ('novel_mag138_04.png', 'mag138_04  +15 deg'),
    ('novel_mag138_05.png', 'mag138_05  +30 deg'),
]

fig, axes = plt.subplots(2, 5, figsize=(22, 9))
fig.suptitle('Novel View Synthesis: 10 Rendered Poses (5 per Magazine Cluster)', fontsize=14, fontweight='bold')

for idx, (fname, label) in enumerate(frames):
    row = idx // 5
    col = idx % 5
    ax = axes[row, col]
    img = np.array(Image.open(OUT_DIR / fname).convert('RGB'))
    thumb_h = 400
    scale = thumb_h / img.shape[0]
    thumb_w = int(img.shape[1] * scale)
    from PIL import Image as PILImage
    thumb = np.array(PILImage.fromarray(img).resize((thumb_w, thumb_h), PILImage.LANCZOS))
    ax.imshow(thumb)
    ax.set_title(label, fontsize=10, color='steelblue' if 'mag137' in fname else 'mediumvioletred')
    ax.axis('off')

plt.tight_layout()
out = OUT_DIR / 'contact_sheet.png'
plt.savefig(str(out), dpi=120, bbox_inches='tight')
plt.close()
print(f'Saved: {out}')
