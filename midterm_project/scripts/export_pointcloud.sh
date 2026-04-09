#!/bin/bash
# export_pointcloud.sh — Export dense point cloud from a trained nerfstudio model
#
# Usage:
#   ./export_pointcloud.sh [config_path] [num_points]
#
# If config_path is omitted, the script finds the most recent config.yml
# under ~/midterm_3dgs_output/.
#
# Default num_points: 500000 (500k — reasonable for 8GB VRAM + RTX 4060)
# Output: ~/midterm_pointcloud/point_cloud.ply

set -e

OUTPUT_DIR="$HOME/midterm_3dgs_output"
POINTCLOUD_DIR="$HOME/midterm_pointcloud"
NUM_POINTS="${2:-500000}"

# ── Find config ────────────────────────────────────────────────────────────────

if [ -n "$1" ] && [ -f "$1" ]; then
    CONFIG_PATH="$1"
else
    CONFIG_PATH=$(find "$OUTPUT_DIR" -name "config.yml" | sort | tail -1)
    if [ -z "$CONFIG_PATH" ]; then
        echo "ERROR: No config.yml found under $OUTPUT_DIR"
        echo "  Run train_3dgs.sh first, or pass the config path as argument:"
        echo "  $0 /path/to/config.yml [num_points]"
        exit 1
    fi
    echo "Auto-detected config: $CONFIG_PATH"
fi

if ! command -v ns-export &>/dev/null; then
    echo "ERROR: ns-export not found. Install nerfstudio:"
    echo "  pip install nerfstudio --break-system-packages"
    exit 1
fi

echo "=== Point Cloud Export ==="
echo "  Config     : $CONFIG_PATH"
echo "  Num points : $NUM_POINTS"
echo "  Output dir : $POINTCLOUD_DIR"
echo ""

mkdir -p "$POINTCLOUD_DIR"

ns-export pointcloud \
    --load-config "$CONFIG_PATH" \
    --output-dir "$POINTCLOUD_DIR" \
    --num-points "$NUM_POINTS"

echo ""
echo "=== Export complete ==="
if ls "$POINTCLOUD_DIR"/*.ply &>/dev/null; then
    for f in "$POINTCLOUD_DIR"/*.ply; do
        SIZE=$(du -sh "$f" | cut -f1)
        POINTS=$(python3 -c "
import struct, sys
with open('$f', 'rb') as fh:
    hdr = b''
    while b'end_header' not in hdr:
        hdr += fh.readline()
    for line in hdr.decode().split('\n'):
        if line.startswith('element vertex'):
            print(line.split()[-1])
            break
" 2>/dev/null || echo "?")
        echo "  $f  ($SIZE, ~$POINTS points)"
    done
else
    echo "  Output dir : $POINTCLOUD_DIR"
fi
echo ""
echo "Open in CloudCompare, MeshLab, or Open3D:"
echo "  python3 -c \\"
echo "    \"import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('$POINTCLOUD_DIR/point_cloud.ply')])\""
