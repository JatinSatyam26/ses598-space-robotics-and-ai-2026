import shutil
from pathlib import Path
from PIL import Image

SRC_137 = Path('/home/jatin-satyam/ses598-apollo17/data/rgb/mag137')
SRC_138 = Path('/home/jatin-satyam/ses598-apollo17/data/rgb/mag138')
NOVEL_DIR = Path('/home/jatin-satyam/ses598-apollo17/novel_poses')
OUT_137 = Path('/home/jatin-satyam/ses598-apollo17/data/rgb_n24/mag137')
OUT_138 = Path('/home/jatin-satyam/ses598-apollo17/data/rgb_n24/mag138')

TARGET_SIZE_137 = (2340, 2364)
TARGET_SIZE_138 = (2340, 2345)

for src in sorted(SRC_137.glob('*.png')):
    dst = OUT_137 / src.name
    shutil.copy2(src, dst)
    print(f'Copied {src.name} -> mag137/')

for src in sorted(SRC_138.glob('*.png')):
    if '21036' in src.name:
        print(f'Skipping excluded {src.name}')
        continue
    dst = OUT_138 / src.name
    shutil.copy2(src, dst)
    print(f'Copied {src.name} -> mag138/')

for i in range(1, 6):
    src = NOVEL_DIR / f'novel_mag137_{i:02d}.png'
    img = Image.open(src).convert('RGB')
    img_resized = img.resize(TARGET_SIZE_137, Image.LANCZOS)
    dst = OUT_137 / f'novel_mag137_{i:02d}.png'
    img_resized.save(dst)
    print(f'Resized+saved novel_mag137_{i:02d}.png -> mag137/ ({img.size} -> {TARGET_SIZE_137})')

for i in range(1, 6):
    src = NOVEL_DIR / f'novel_mag138_{i:02d}.png'
    img = Image.open(src).convert('RGB')
    img_resized = img.resize(TARGET_SIZE_138, Image.LANCZOS)
    dst = OUT_138 / f'novel_mag138_{i:02d}.png'
    img_resized.save(dst)
    print(f'Resized+saved novel_mag138_{i:02d}.png -> mag138/ ({img.size} -> {TARGET_SIZE_138})')

print(f'\nmag137 total: {len(list(OUT_137.glob("*.png")))}')
print(f'mag138 total: {len(list(OUT_138.glob("*.png")))}')
print(f'Grand total: {len(list(OUT_137.glob("*.png"))) + len(list(OUT_138.glob("*.png")))}')
