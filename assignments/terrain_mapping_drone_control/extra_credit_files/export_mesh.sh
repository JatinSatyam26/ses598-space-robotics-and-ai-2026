#!/bin/bash
#
# export_mesh.sh — Export 3D mesh from RTAB-Map database
#
# Produces:
#   - rtabmap_mesh.obj  (3D mesh file)
#   - rtabmap_mesh.mtl  (material file)
#   - rtabmap_cloud.ply  (point cloud)
#
set -e

DB_PATH="$HOME/.ros/rtabmap.db"
OUTPUT_DIR="$HOME/ros2_ws/src/terrain_mapping_drone_control/mesh_output"

# Check database exists and has content
if [ ! -f "$DB_PATH" ]; then
    echo "ERROR: No database found at $DB_PATH"
    echo "Run the mission first with launch_all.sh"
    exit 1
fi

DB_SIZE=$(stat -c%s "$DB_PATH" 2>/dev/null || stat -f%z "$DB_PATH" 2>/dev/null)
echo "Database size: $DB_SIZE bytes"

if [ "$DB_SIZE" -lt 100000 ]; then
    echo "WARNING: Database is very small ($DB_SIZE bytes). It may be empty."
    echo "The mission may not have recorded enough data."
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

# Source ROS
source /opt/ros/jazzy/setup.bash

echo ""
echo "=== Exporting point cloud (PLY) ==="
rtabmap-export \
    --poses \
    --output_dir "$OUTPUT_DIR" \
    "$DB_PATH" || echo "Point cloud export had issues (may still have partial output)"

echo ""
echo "=== Exporting 3D mesh (OBJ) ==="
rtabmap-export \
    --mesh \
    --texture \
    --output_dir "$OUTPUT_DIR" \
    "$DB_PATH" || echo "Mesh export had issues (may still have partial output)"

echo ""
echo "=== Database info ==="
rtabmap-info "$DB_PATH" 2>/dev/null || echo "Could not read database info"

echo ""
echo "=== Export complete ==="
echo "Output directory: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR"
echo ""
echo "Upload the mesh_output/ folder to your repo for submission."
echo "If files are large, use: git lfs track '*.obj' '*.ply' '*.mtl'"
