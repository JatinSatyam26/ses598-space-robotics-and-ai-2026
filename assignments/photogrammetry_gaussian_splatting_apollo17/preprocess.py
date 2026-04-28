from PIL import Image
from pathlib import Path

src = Path.home() / "ses598-apollo17" / "data" / "raw"
dst = Path.home() / "ses598-apollo17" / "data" / "rgb"
dst.mkdir(parents=True, exist_ok=True)

for p in sorted(src.glob("*.png")):
    img = Image.open(p)
    if img.mode != "RGB":
        img = img.convert("RGB")
    img.save(dst / p.name)
    print(f"{p.name}: mode={img.mode}, size={img.size}")
