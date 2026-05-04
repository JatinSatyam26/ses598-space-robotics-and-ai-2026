#!/bin/bash
# run_colmap.sh — Sparse 3D reconstruction from drone survey images
#
# Usage:
#   ./run_colmap.sh [image_dir]
#
# Default image_dir: ~/midterm_flight_data/rgb/
# Output:            ~/midterm_reconstruction/sparse/0/
#
# Uses COLMAP automatic_reconstructor in sparse-only mode with a PINHOLE
# camera model (matching the Gazebo simulated camera, 640x480).

set -e

IMAGE_DIR="${1:-$HOME/midterm_flight_data/rgb}"
WORKSPACE="$HOME/midterm_reconstruction"

if [ ! -d "$IMAGE_DIR" ]; then
    echo "ERROR: Image directory not found: $IMAGE_DIR"
    exit 1
fi

IMAGE_COUNT=$(ls "$IMAGE_DIR"/*.png 2>/dev/null | wc -l)
if [ "$IMAGE_COUNT" -eq 0 ]; then
    echo "ERROR: No PNG images found in $IMAGE_DIR"
    exit 1
fi

echo "=== COLMAP Sparse Reconstruction ==="
echo "  Image dir  : $IMAGE_DIR ($IMAGE_COUNT images)"
echo "  Workspace  : $WORKSPACE"
echo "  Camera     : PINHOLE (single camera)"
echo "  Dense      : disabled (sparse only)"
echo ""

mkdir -p "$WORKSPACE"

colmap automatic_reconstructor \
    --workspace_path "$WORKSPACE" \
    --image_path "$IMAGE_DIR" \
    --camera_model PINHOLE \
    --single_camera 1 \
    --dense 0

echo ""
echo "=== Reconstruction complete ==="

# Print summary
SPARSE_DIR="$WORKSPACE/sparse/0"
if [ -d "$SPARSE_DIR" ]; then
    echo "  Output: $SPARSE_DIR"
    for f in cameras.bin images.bin points3D.bin cameras.txt images.txt points3D.txt; do
        if [ -f "$SPARSE_DIR/$f" ]; then
            SIZE=$(du -sh "$SPARSE_DIR/$f" | cut -f1)
            echo "    $f  ($SIZE)"
        fi
    done
    # Count registered images and 3D points if text files exist
    if [ -f "$SPARSE_DIR/images.txt" ]; then
        REG=$(grep -c "^[0-9]" "$SPARSE_DIR/images.txt" 2>/dev/null || echo "?")
        echo "  Registered images : $((REG / 2))"
    fi
    if [ -f "$SPARSE_DIR/points3D.txt" ]; then
        PTS=$(grep -c "^[0-9]" "$SPARSE_DIR/points3D.txt" 2>/dev/null || echo "?")
        echo "  3D points         : $PTS"
    fi
else
    echo "  WARNING: sparse/0/ not found — reconstruction may have failed."
    echo "  Check $WORKSPACE/sparse/ for partial results."
    ls "$WORKSPACE/sparse/" 2>/dev/null || true
fi
