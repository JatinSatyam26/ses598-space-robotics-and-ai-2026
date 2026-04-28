from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

WORK_DIR = Path.home() / "ses598-space-robotics-and-ai-2026/assignments/photogrammetry_gaussian_splatting_apollo17"
COMPARISON_DIR = WORK_DIR / "comparison"
OUTPUT_PATH = COMPARISON_DIR / "textured_qualitative_grid.png"

sidebyside_paths = [
    COMPARISON_DIR / "textured_sidebyside_view1.png",
    COMPARISON_DIR / "textured_sidebyside_view2.png",
    COMPARISON_DIR / "textured_sidebyside_view3.png",
    COMPARISON_DIR / "textured_sidebyside_view4.png",
]

panels = [Image.open(p).convert("RGB") for p in sidebyside_paths]
print("Loaded panels", [(p.width, p.height) for p in panels])

H_PAD = 40
V_PAD_TITLE = 40
V_PAD_ROW = 60
TITLE_HEIGHT = 70
MAX_LONG_DIM = 4000

cell_width = (MAX_LONG_DIM - H_PAD) // 2
scale = cell_width / panels[0].width
cell_height = round(panels[0].height * scale)

cells = [p.resize((cell_width, cell_height), Image.LANCZOS) for p in panels]
print("Cell size", cell_width, "x", cell_height)

total_width = cell_width * 2 + H_PAD
total_height = TITLE_HEIGHT + V_PAD_TITLE + cell_height + V_PAD_ROW + cell_height
print("Grid size", total_width, "x", total_height)

grid = Image.new("RGB", (total_width, total_height), "white")
draw = ImageDraw.Draw(grid)

font_paths = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
]
font = None
for fp in font_paths:
    if Path(fp).exists():
        font = ImageFont.truetype(fp, 34)
        break
if font is None:
    font = ImageFont.load_default()

title_text = "Textured Mesh Comparison: N=14 vs N=24 Photogrammetry (4 Viewpoints)"
bbox = draw.textbbox((0, 0), title_text, font=font)
text_w = bbox[2] - bbox[0]
text_h = bbox[3] - bbox[1]
title_x = (total_width - text_w) // 2
title_y = (TITLE_HEIGHT - text_h) // 2
draw.text((title_x, title_y), title_text, fill="black", font=font)

y_row1 = TITLE_HEIGHT + V_PAD_TITLE
y_row2 = y_row1 + cell_height + V_PAD_ROW

grid.paste(cells[0], (0, y_row1))
grid.paste(cells[1], (cell_width + H_PAD, y_row1))
grid.paste(cells[2], (0, y_row2))
grid.paste(cells[3], (cell_width + H_PAD, y_row2))

grid.save(str(OUTPUT_PATH), optimize=True)
print("Grid saved", OUTPUT_PATH)
