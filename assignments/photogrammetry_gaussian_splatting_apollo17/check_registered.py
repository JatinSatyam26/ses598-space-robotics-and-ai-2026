from pathlib import Path

base = Path.home() / "ses598-apollo17"
rgb_root = base / "data" / "rgb"
all_images = sorted(str(p.relative_to(rgb_root)) for p in rgb_root.rglob("*.png"))

images_txt = base / "colmap_n15" / "sparse" / "0_text" / "images.txt"
registered = set()
with open(images_txt) as f:
    for line in f:
        line = line.strip()
        if line.startswith("#") or not line:
            continue
        parts = line.split()
        if len(parts) >= 10 and parts[-1].endswith(".png"):
            registered.add(parts[-1])

print(f"Total images: {len(all_images)}")
print(f"Registered: {len(registered)}")
print()
print("Registered:")
for img in sorted(registered):
    print(f"  {img}")
print()
print("NOT registered:")
missing = set(all_images) - registered
for img in sorted(missing):
    print(f"  {img}")
