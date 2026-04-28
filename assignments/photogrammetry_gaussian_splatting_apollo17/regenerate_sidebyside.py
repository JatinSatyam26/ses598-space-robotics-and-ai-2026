from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

COMP_DIR = Path.home() / "ses598-space-robotics-and-ai-2026/assignments/photogrammetry_gaussian_splatting_apollo17/comparison"
PHOTO_DIR = Path.home() / "ses598-apollo17/data/rgb"

PANEL_SIZE = 800
OUTER_PAD = 20
H_GAP = 10
TITLE_H = 44
GAP1 = 8
LABEL_H = 32
GAP2 = 10

views = [
    (1, "mag137/AS17-137-20904HR.png", "AS17-137-20904HR"),
    (2, "mag137/AS17-137-20907HR.png", "AS17-137-20907HR"),
    (3, "mag138/AS17-138-21030HR.png", "AS17-138-21030HR"),
    (4, "mag138/AS17-138-21033HR.png", "AS17-138-21033HR"),
]

total_w = OUTER_PAD + PANEL_SIZE + H_GAP + PANEL_SIZE + H_GAP + PANEL_SIZE + OUTER_PAD
total_h = OUTER_PAD + TITLE_H + GAP1 + LABEL_H + GAP2 + PANEL_SIZE + OUTER_PAD

print("Canvas size per side-by-side", total_w, "x", total_h)

bold_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
regular_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
title_font = ImageFont.truetype(bold_path, 20)
label_font = ImageFont.truetype(bold_path, 15)

x_cols = [
    OUTER_PAD,
    OUTER_PAD + PANEL_SIZE + H_GAP,
    OUTER_PAD + 2 * (PANEL_SIZE + H_GAP),
]

panel_y = OUTER_PAD + TITLE_H + GAP1 + LABEL_H + GAP2

for vi, photo_rel, photo_stem in views:
    r14 = Image.open(COMP_DIR / f"textured_render_n14_view{vi}.png").convert("RGB")
    r24 = Image.open(COMP_DIR / f"textured_render_n24_view{vi}.png").convert("RGB")
    photo = Image.open(PHOTO_DIR / photo_rel).convert("RGB").resize((PANEL_SIZE, PANEL_SIZE), Image.LANCZOS)

    canvas = Image.new("RGB", (total_w, total_h), "white")
    draw = ImageDraw.Draw(canvas)

    title_text = f"Textured Mesh Comparison View {vi}: {photo_stem}"
    bbox = draw.textbbox((0, 0), title_text, font=title_font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((total_w - tw) // 2, OUTER_PAD + (TITLE_H - th) // 2), title_text, fill="black", font=title_font)

    panel_labels = [
        ("N=14 Textured Mesh", "steelblue"),
        (f"Source Photo: {photo_stem}", "goldenrod"),
        ("N=24 Textured Mesh (ICP-aligned)", "mediumvioletred"),
    ]
    label_top = OUTER_PAD + TITLE_H + GAP1
    for xi, (label_text, label_color) in zip(x_cols, panel_labels):
        bbox = draw.textbbox((0, 0), label_text, font=label_font)
        lw, lh = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((xi + (PANEL_SIZE - lw) // 2, label_top + (LABEL_H - lh) // 2), label_text, fill=label_color, font=label_font)

    for xi, panel in zip(x_cols, [r14, photo, r24]):
        canvas.paste(panel, (xi, panel_y))

    out_path = COMP_DIR / f"textured_sidebyside_view{vi}.png"
    canvas.save(str(out_path))
    print(f"Saved view {vi}", out_path)

print("All four side-by-sides regenerated")
