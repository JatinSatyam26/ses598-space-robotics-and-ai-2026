# SES 598 — Technical Invariants Cheat Sheet
**As of end of April 17, 2026 session. Read this AFTER the transcript, keep it open during work.**

---

## Project identity

- **Student:** Jatin Satyam, ASU, SES 598 Space Robotics and AI, Spring 2026
- **Stack:** Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic 8.10.0, native plugins only (NO PX4, NO MicroXRCE, NO QGC)
- **Drone plugin:** `MulticopterVelocityControl` + `MulticopterMotorModel`
- **Rover plugin:** `DiffDrive`
- **Working model:** Claude Opus 4.7 with extended thinking
- **Machine:** OMEN HP Gaming Laptop 16, RTX 4060, CUDA 12.8

---

## Epistemic rules (non-negotiable)

1. **Real physics only. No shortcuts. No deception of any kind.**
2. **Terminal truth and visual truth must always agree** before declaring anything "working."
3. **No hardcoded constants that encode world assumptions.** If the value depends on the world file, derive it at runtime.
4. **Ground-truth pose is for validation only**, never operational decisions.
5. **No cosmetic fixes.** Fix the physics, or report honest physics. Never massage numbers.
6. **Every SUCCESS/COMPLETE/ARRIVED message must be backed by a physical simulator condition**, not just a code path.

---

## Terminal paste discipline (codified Apr 17, 2026)

**Problem observed:** Paste buffer corrupts heredocs larger than ~30 lines. Symptom is characters from *later* in the command block get injected *into* the heredoc before its closing delimiter, silently producing truncated/malformed files. Direct `cat > file.sdf <<'SDF' ... SDF` is especially vulnerable — the delimiter gets swallowed and the heredoc absorbs whatever comes next.

**Rule for any file creation >30 lines or any heredoc (especially XML/SDF):**

1. **Use a two-step Python write, not a direct heredoc.**
   - **Step A (small safe paste):** `cat > /tmp/write_step.py <<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` ... Python script with target file content as triple-quoted string ... `PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE`
   - **Step B (trivial paste):** `python3 /tmp/write_step.py`
   - If Step A paste corrupts, Python fails with SyntaxError — loud failure, not silent corruption.

2. **Always pre-validate inside the script.** For XML/SDF: `ET.fromstring(SDF_STRING)` BEFORE writing to disk.

3. **Always post-validate after writing.** Read back from disk and re-validate.

4. **Use improbable delimiters.** Not `<<'SDF'` or `<<'EOF'`. Use `<<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` or similar that cannot possibly appear in content.

5. **Wait 1-2 seconds between pastes.** Give the terminal's input buffer time to flush. Do not chain pastes.

6. **Gazebo log to file first, dump via `tail` after.** `timeout N gz sim ... > /tmp/log 2>&1; tail -60 /tmp/log`. Killed processes truncate piped output unreliably; files capture everything.

This methodology was tested on April 17 and survived paste corruption in the very next command after codification.

---

## Current system state

### Working (verified)
- Drone takeoff, survey (10 waypoints), return home, land at z≈0.2 m with skids on ground
- Controller disable on landing via `gz topic /flying_drone/enable data:false`
- Rover A* path planning with goal-replanner when goal cell is occupied
- Rover waypoint navigation through all planned waypoints
- **HiRISE Jezero_C DTM + ortho downloaded, cropped, prepared as Gazebo heightmap assets (NEW Apr 17)**
- **Mars-ochre orthoimage texture verified visually (NEW Apr 17)**

### In progress
- **Jezero_C world loads in Gazebo headless but 3 issues surfaced (NEW Apr 17)** — see "Next actions"

### Broken (diagnosed, not yet fixed)
- **Rover localization:** reads start-relative DiffDrive odom as world-frame. Rover is always offset by the spawn pose `(0, +1.5)` from its true world position. Next thing to fix with full TRN stack, not with a hardcoded offset.
- Status printer in launch script shows `WP 0/8` always. Cosmetic, dishonest, known.
- Simulation does not self-terminate after mission complete. Operational nuisance.

### Not yet built
- Ground-truth pose logger (validation baseline)
- Scan-to-map matcher
- Pose fusion node
- Modified rover node consuming fused world-frame pose
- Mastcam-Z hero rock meshes (Phase 3 next)
- Poly Haven scatter with Golombek-Rapp distribution (Phase 4)

---

## Tier-3 world build state (NEW Apr 17)

### Decisions locked in
- **Tile:** Jezero_C (ESP_045994_1985 + ESP_046060_1985 stereo pair). Rationale: landing site area, hazard-validated flat floor, DiffDrive-safe, same DTM used in Perseverance's onboard TRN flight map.
- **Patch:** 40 × 40 m, 300 m east + 20 m south of Octavia E. Butler Landing. Avoids landing scar while staying in mission neighborhood.
- **Product form:** Form B (USGS/JPL TRN mosaic individual tile, not full 3.2 GB mosaic). GeoTIFF, affine-aligned to CTX reference.
- **Rock strategy:** (c) Mastcam-Z photogrammetry for hero outcrops (Brac, Caille, Cheiron, Hogwallow-Rockytop) + (b) Poly Haven scatter for density. 100-300 rocks after size-threshold cull, Golombek-Rapp k≈0.05 q≈2.0.

### Measured terrain characteristics (from HiRISE data)
- Full Jezero_C tile: 7.06 × 14.44 km, elevation -2665 to -2396 m (below Mars 2000 Sphere ref, correct for Jezero basin)
- Our 40 × 40 m patch: **relief 3.08 m**, mean slope ~4.4% (~2.5°), StdDev 0.888 m — broadly distributed gentle undulation, safe for DiffDrive

### Asset files
```
~/ros2_ws/src/midterm_project/worlds/jezero_c/
├── dtm/
│   ├── DTM_MOLAtopography_..._Jezero_C_..._1m_Eqc_....tif        [120 MB, full tile, keep]
│   ├── jezero_c_patch_40m_raw.tif                                 [7.0 KB, cropped pre-norm]
│   ├── jezero_c_patch_40m_normalized.tif                          [5.2 KB, Float32 0-3.08 m]
│   ├── jezero_c_heightmap_65.tif                                  [8.5 KB, PRIMARY Gazebo input]
│   └── jezero_c_heightmap_65.png                                  [993 B, 8-bit fallback]
└── ortho/
    ├── ESP_045994_1985_..._25cm_Eqc_....tif                       [1.6 GB, full tile, keep]
    ├── jezero_c_patch_40m.tif                                     [26 KB, cropped grayscale]
    └── jezero_c_texture_257.png                                   [30 KB, Mars-ochre RGB]
```

### World SDF
- `~/ros2_ws/src/midterm_project/worlds/jezero_c.sdf` (2231 bytes, XML-valid)
- Contains: physics + plugins + Mars sky/sun + heightmap model pointing at 65.tif + 257.png texture
- **Needs physics engine fix (dartsim → bullet-featherstone) before it will collide correctly**
- Launch requires `export PROJ_IGNORE_CELESTIAL_BODY=YES`

---

## Key spawn poses (from Tier-1/2 world SDF, not yet migrated to jezero_c.sdf)

| Entity | Spawn pose | Frame |
|---|---|---|
| Drone | (0, 0, 0.2) | world |
| Rover | (0, 1.5, 0.12) | world |
| Dome | (15, 0, 0), radius 1.0 m sphere | world |

---

## Key file paths

- Project root: `~/ros2_ws/src/midterm_project/`
- Flight node: `~/ros2_ws/src/midterm_project/midterm_project/smart_flight_node.py`
- Rover node: `~/ros2_ws/src/midterm_project/midterm_project/smart_rover_node.py`
- Flight node log (runtime): `/tmp/smart_flight.log`
- Rover node log (runtime): `/tmp/smart_rover.log`
- Bridge log: `/tmp/bridge.log`
- Launch script: `~/ros2_ws/src/midterm_project/scripts/run_live_demo.sh`
- Tier-1/2 world: `~/ros2_ws/src/midterm_project/worlds/mars_mission.sdf`
- **Tier-3 world (new Apr 17):** `~/ros2_ws/src/midterm_project/worlds/jezero_c.sdf`
- **Tier-3 asset dir (new Apr 17):** `~/ros2_ws/src/midterm_project/worlds/jezero_c/`
- x500_base (landing gear reference): `~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`

---

## Key numbers

| Value | Current | Why |
|---|---|---|
| Drone LAND threshold | `pos[2] < 0.23` | x500 skids touch ground when base_link at z=0.227 |
| Rover goal | `(15.0, 0.0)` | Dome center from world SDF |
| Rover arrival method | Path completion (not proximity) | Approach B, after proximity kept producing false positives |
| Rover arrival sanity | Final WP within 2.0 m of dome | Dome radius 1.0 + grid cell 0.25 + margin |
| Occupancy grid resolution | 0.25 m | Sets floor on scan-match accuracy |
| Grid extent (world-frame) | x=[-2, 18], y=[-5, 5] | From flight node config |
| LiDAR stop distance | 0.55 m | From `obstacle_stop_dist` |
| **Jezero_C patch size** | 64 × 64 m (padded from 40 × 40) | **65×65 pixel heightmap (2^6+1)** |
| **Jezero_C patch relief** | **3.08 m** | **Max − min from real HiRISE DTM** |
| **Mars 2000 Sphere radius** | **3,396,190 m** | **For lat/lon ↔ projection coord math** |
| **Butler Landing projection XY** | **(4590878.7, 1093298.1) m** | **Derived: R × radians(lon), R × radians(lat)** |
| **Patch center (projection)** | **(4591197.8, 1093298.0) m** | **300 m east, 20 m south of Butler Landing** |

---

## Topic map

| Topic | Type | Direction | Source/Sink |
|---|---|---|---|
| `/drone/cmd_vel` | geometry_msgs/Twist | ROS→GZ | flight node → `/flying_drone/gazebo/command/twist` |
| `/drone/odom` | nav_msgs/Odometry | GZ→ROS | drone OdometryPublisher |
| `/drone/occupancy_grid` | nav_msgs/OccupancyGrid | ROS internal | flight node → rover node |
| `/rover/cmd_vel` | geometry_msgs/Twist | ROS→GZ | rover node → `/model/rover/cmd_vel` (via bridge) |
| `/rover/odom` | nav_msgs/Odometry | GZ→ROS | rover DiffDrive plugin (`rover_odom` on GZ side, remapped to `/rover/odom`) **START-RELATIVE** |
| `/rover_lidar` | sensor_msgs/LaserScan | GZ | rover 360° LiDAR |
| `/flying_drone/enable` | gz.msgs.Boolean | GZ only | controller enable/disable |
| `/world/mars_mission/pose/info` | gz.msgs.Pose_V | GZ only | ground truth for ALL models — bridge NOT yet wired |

---

## Environment gotchas (observed, reproducible)

| Issue | Symptom | Workaround |
|-------|---------|------------|
| **NumPy 2.4.4 vs python3-gdal 3.8.4 ABI** | `gdal_calc.py` fails with `_ARRAY_API not found`, `ImportError: numpy.core.multiarray failed to import` | Use pure `osgeo.gdal` + `struct` instead of `osgeo.gdal_array`. Do NOT downgrade NumPy — it'll break ROS2 Jazzy deps. |
| **Paste heredocs >30 lines corrupt** | Characters from later in block injected mid-heredoc, silent file truncation | Two-step Python write approach (see Terminal paste discipline section) |
| **Gazebo PROJ Mars→Earth rejection** | `Source and target ellipsoid do not belong to the same celestial body` during DEM load | `export PROJ_IGNORE_CELESTIAL_BODY=YES` before launching `gz sim`. This is the PROJ-documented escape hatch. |
| **Dartsim heightmap unimplemented** | `Heightmap construction from an SDF has not been implemented yet for dartsim` | Explicit `<plugin filename="gz-physics-bullet-featherstone-plugin" .../>` in SDF to override dartsim. ODE also works. |
| **Canonical mirror flakiness** | `apt install` returns DNS/timeout after `apt update` succeeds | Retry with `--fix-missing`. Usually transient (minutes, not hours). |
| **`convert: Request did not return an image.`** | ImageMagick with no args | This is IM's idiomatic "no args" message. NOT an error. Ignore. |

---

## Tier-3 phase roadmap

- [x] **Phase 0:** Environment diagnostic
- [x] **Phase 0B:** GDAL + Blender install (Ubuntu 24.04 python3-gdal + libgdal-dev + gdal-bin 3.8.4, blender 4.0.2)
- [x] **Phase 1A:** HiRISE DTM + ortho download (Jezero_C, ~1.7 GB)
- [x] **Phase 1B:** Metadata extraction + Butler Landing projection coord calc
- [x] **Phase 1C:** Crop 40×40 m patch + normalize DTM to 0-based elevation
- [x] **Phase 1D:** Pad DTM to 65×65 (2^6+1), generate Mars-ochre texture at 257×257
- [x] **Phase 1E:** Visual sanity check (ortho color passed, heightmap gradient passed)
- [x] **Phase 2 (partial):** World SDF written, XML-valid, loads headless. **3 issues surfaced — PROJ, dartsim, DEM size cascade.**
- [ ] **Phase 2 (complete):** Fix PROJ env var, swap to bullet-featherstone, re-launch headless, launch GUI, visually confirm terrain
- [ ] **Phase 3:** Download Mastcam-Z hero meshes (Sketchfab CC-Attribution), chop in Blender
- [ ] **Phase 4:** Download Poly Haven scatter meshes, write Golombek-Rapp scatter script
- [ ] **Phase 5:** Integrate habitat dome, drone, rover into Jezero world
- [ ] **Phase 6:** Verify drone survey + rover navigation still work on new terrain
- [ ] **Phase 7:** Ground-truth logger
- [ ] **Phase 8:** Confidence-Rich Grid Mapping on drone
- [ ] **Phase 9:** Scan-to-map matcher (hierarchical correlation + branch-and-bound)
- [ ] **Phase 10:** Pose fusion EKF
- [ ] **Phase 11:** Modified rover node consuming fused pose
- [ ] **Phase 12:** Integration run + localization error curve

---

## Next-action checklist — IMMEDIATE

**Before anything else, test that Gazebo loads the Jezero world with collision:**

1. Rewrite `jezero_c.sdf` to override physics engine. Use two-step Python write. Add explicit `<plugin filename="gz-physics-bullet-featherstone-plugin" name="gz::physics::bullet_featherstone::BulletFeatherstonePlugin"/>` (verify exact plugin name from `/usr/lib/x86_64-linux-gnu/gz-physics-*`).
2. Launch headless with `PROJ_IGNORE_CELESTIAL_BODY=YES`. Verify log shows no Dem.cc:390 error and no dartsim heightmap warning.
3. Launch GUI, confirm terrain visible with Mars-ochre texture, walk camera around.
4. Drop a test box onto the terrain at (0, 0, 5) and confirm it collides/rests on the surface — proves collision works.
5. ONLY THEN proceed to Phase 3 (Mastcam-Z mesh download).

**If bullet-featherstone also fails:** fallback to mesh-based terrain. Convert 65×65 DTM to triangulated .obj with UV mapping to ortho PNG. Guaranteed to work because meshes are core Gazebo geometry.

---

## Bug archaeology (one-line per historical bug for context)

- **Phase 60 (Apr 3):** QoS mismatch, topic names, TrajectorySetpoint NaNs, `/clock` death spiral, stale Gazebo — all PX4-era, obsoleted by Apr 9 pivot
- **Apr 9 pivot:** abandoned PX4, went native Gazebo plugins, gained simplicity, lost disarm primitive
- **Apr 15 prior session:** identified landing bug hypotheses, locked 5-day plan
- **Apr 16 drone:** landing bug was velocity-controller-has-no-disarm; fixed via controller disable + threshold correction
- **Apr 16 rover:** cascade of arrival-too-loose → proximity-fires-early → coord-frame-offset
- **Apr 16 localization pivot:** rejected hardcoded fix; building real TRN stack
- **Apr 17 DTM preparation:** NumPy ABI blocked gdal_calc.py, pivoted to pure osgeo.gdal Python
- **Apr 17 paste disaster:** heredoc corruption silently produced broken SDF, codified two-step Python write methodology in response
- **Apr 17 Gazebo launch:** PROJ Mars rejection + dartsim heightmap unimplemented — diagnosed, not yet fixed

---

## Working preferences

- One command block at a time. No "run all of these."
- When asking user to run something, keep it to 1-3 commands max.
- For anything requiring file creation >30 lines or heredocs, use the two-step Python approach. Mandatory.
- User will paste output in a text file. Read it carefully, not skimmed.
- Catch logical inconsistencies. Don't hand-wave.
- Don't declare things fixed until visually verified in Gazebo.
- Don't offer deceptive/cosmetic options.
- Patch scripts should be idempotent with timestamped backups.
- When I suspect I need to re-verify something I've been told, say so and re-verify. Don't assume.
