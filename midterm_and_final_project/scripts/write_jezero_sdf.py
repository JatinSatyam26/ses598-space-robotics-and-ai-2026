"""
Write (or rewrite) worlds/jezero_c.sdf with the mesh-based terrain configuration.

Canonical record of the SDF structure used in the Tier-3 Jezero world build.
Re-run this if the SDF has been corrupted or you want to regenerate it from
the authoritative source.

Key design decisions baked in:
  - Uses dartsim default physics engine (no <engine> override).
    Rationale: neither dartsim, classic Bullet, nor bullet-featherstone build
    heightmap collision from SDF in Gazebo Harmonic / gz-physics7. Mesh-based
    terrain works with all three. Dartsim is the reference implementation
    with the most-exercised mesh path.
  - Terrain is loaded as a triangulated .obj mesh (see scripts/dtm_to_obj.py).
  - Static terrain model, zero inertia contribution, collision + visual point
    at the same mesh URI.
  - PROJ_IGNORE_CELESTIAL_BODY=YES env var is still needed at launch time
    (Gazebo's DEM loader hardcodes EPSG:4326 as target CRS, PROJ 8+ blocks
    Mars-to-Earth coordinate transforms). This script does not set that env var;
    that's a launch concern.

Uses the two-step Python write methodology (paste-discipline).

Usage:
  python3 scripts/write_jezero_sdf.py
"""

import os
import shutil
import xml.etree.ElementTree as ET
from datetime import datetime

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TARGET = os.path.join(PROJECT_ROOT, "worlds/jezero_c.sdf")

SDF = '''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="jezero_c">

    <!-- Physics: standard real-time, 1ms step -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Core plugins (required for Gazebo Harmonic simulation) -->
    <!-- Using dartsim default physics engine: mesh collision is the reference
         implementation path, well-tested across all gz-physics-7 engines.
         No <engine> override needed. -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Mars sky: butterscotch ambient, low dust -->
    <scene>
      <ambient>0.55 0.35 0.25 1</ambient>
      <background>0.75 0.55 0.40 1</background>
      <grid>false</grid>
    </scene>

    <!-- Sun: low angle to get good shadowing on the terrain -->
    <light name="mars_sun" type="directional">
      <pose>0 0 50 0 0.7 0.3</pose>
      <diffuse>0.95 0.85 0.75 1</diffuse>
      <specular>0.4 0.35 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 -0.3 -0.8</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Jezero_C terrain as triangulated mesh.
         Source: 65x65 Float32 DTM converted via scripts/dtm_to_obj.py
         4225 vertices, 8192 triangles, 64x64 m extent, 0-3.08 m relief.
         UV-mapped to ortho texture (Option A: full-terrain mapping). -->
    <model name="jezero_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <mesh>
              <uri>file://jezero_c/terrain_mesh/jezero_terrain.obj</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <mesh>
              <uri>file://jezero_c/terrain_mesh/jezero_terrain.obj</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
'''

# Pre-write validation
try:
    ET.fromstring(SDF)
    print(f"SDF string parses: {len(SDF)} bytes, XML-valid")
except ET.ParseError as e:
    print(f"FATAL: SDF string does NOT parse: {e}")
    raise SystemExit(1)

# Back up existing if present
if os.path.exists(TARGET):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = f"{TARGET}.backup_{ts}"
    shutil.copy2(TARGET, backup_path)
    print(f"Backed up existing file to {backup_path}")
else:
    print("No existing file to back up (writing fresh)")

# Write
with open(TARGET, "w") as f:
    f.write(SDF)
print(f"Wrote {TARGET}")

# Post-write validation
with open(TARGET) as f:
    disk_content = f.read()
try:
    ET.fromstring(disk_content)
    print(f"Post-write XML validation: PASS ({len(disk_content)} bytes on disk)")
except ET.ParseError as e:
    print(f"FATAL: file on disk does NOT parse: {e}")
    raise SystemExit(1)
