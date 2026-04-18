# SES 598 — Technical Invariants Cheat Sheet
**Last updated: April 17, 2026 end-of-day. Read after the transcript, keep open during work.**

---

## Project identity

- **Student:** Jatin Satyam, ASU, SES 598 Space Robotics and AI, Spring 2026
- **Stack:** Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic 8.10.0, native plugins only (NO PX4, NO MicroXRCE, NO QGC)
- **Drone plugin:** `MulticopterVelocityControl` + `MulticopterMotorModel`
- **Rover plugin:** `DiffDrive` (new: 6-wheel Perseverance-inspired design)
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
7. **(New, codified Apr 17)** Scene-graph introspection (`scene/info`) reports rendering components, not physics components. Collision geometry existence must be verified by dynamic test (e.g. drop a box), not by reading scene/info.
8. **(New, codified Apr 17)** Confidence calibration discipline: when a hypothesis is supported by multiple independent sources but has no *affirmative* test evidence, cap confidence at 60%, not 85%. The sanity check is what pushes confidence higher — not consensus of adjacent documentation.

---

## Terminal paste discipline (codified Apr 17, 2026)

**Problem observed:** Paste buffer corrupts heredocs larger than ~30 lines. Symptom is characters from *later* in the command block get injected *into* the heredoc before its closing delimiter, silently producing truncated/malformed files.

**Rule for any file creation >30 lines or any heredoc (especially XML/SDF):**

1. **Use a two-step Python write, not a direct heredoc.**
   - Step A: `cat > /tmp/write_step.py <<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` ... Python script with target file content as triple-quoted string ... `PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE`
   - Step B: `python3 /tmp/write_step.py`

2. **For large files (>1 KB) or during repeated cycles, prefer download-and-move** over any form of paste. Ask Claude to generate the file in its sandbox, download it, `mv ~/Downloads/FILE /tmp/` and run. This is the gold standard — no paste corruption risk at all.

3. **Always pre-validate inside scripts.** For XML/SDF: `ET.fromstring(SDF_STRING)` BEFORE writing to disk.

4. **Always post-validate after writing.** Read back from disk and re-validate.

5. **Use improbable delimiters.** Not `<<'SDF'` or `<<'EOF'`. Use `<<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` or similar.

6. **Gazebo log to file first, dump via `tail` after.** `timeout N gz sim ... > /tmp/log 2>&1; tail -60 /tmp/log`. Killed processes truncate piped output unreliably; files capture everything.

7. **NEVER paste `exit 1` inside a command block.** When pasted into an interactive shell (not inside a script file), it terminates the shell and closes the terminal. Use `exit 0` or restructure without explicit exit.

---

## Current system state

### Working (verified on real data)
- Drone takeoff, survey (10 waypoints), return home, land at z≈0.2 m with skids on ground
- Controller disable on landing via `gz topic /flying_drone/enable data:false`
- Rover A* path planning with goal-replanner when goal cell is occupied
- Rover waypoint navigation through all planned waypoints
- **HiRISE Jezero_C DTM + ortho downloaded, cropped, prepared as Gazebo terrain assets (Apr 17)**
- **Mars-ochre orthoimage texture verified visually (Apr 17)**
- **Jezero_C mesh-based terrain loads in Gazebo, renders correctly, collision verified by box-drop test (Apr 17 end-of-day)**
- **PROJ_IGNORE_CELESTIAL_BODY=YES env var handles Mars CRS issue (Apr 17)**
- **New: 6-wheel Perseverance-inspired rover model (mars_rover/model.sdf)**
- **New: downward rangefinder on drone for altitude hold (flying_drone/model.sdf)**
- **New: habitat dome model (habitat_dome/)**

### In progress
- Nothing actively in progress. Phase 2 complete. Next session starts fresh on Phase 3 or 5.

### Broken (diagnosed, not yet fixed)
- **Rover localization:** reads start-relative DiffDrive odom as world-frame. Rover is always offset by the spawn pose `(0, +1.5)` from its true world position. Fix via TRN stack (Phases 8-10), not hardcoded offset.
- Status printer in launch script shows `WP 0/8` always. Cosmetic, dishonest, known.
- Simulation does not self-terminate after mission complete. Operational nuisance.

### Known cosmetic issues (Apr 17)
- **Terrain visually darker than expected in Gazebo GUI.** Ortho PNG is confirmed ochre stand-alone, but terrain in-scene reads closer to dark chocolate. Possibly a Gazebo tone-mapping / ogre2 auto-exposure interaction with the bright butterscotch sky background. Not blocking. Address in Phase 2.5 polish pass if time.

### Not yet built
- Ground-truth pose logger (validation baseline)
- Scan-to-map matcher
- Pose fusion node
- Modified rover node consuming fused world-frame pose
- Mastcam-Z hero rock meshes (Phase 3)
- Poly Haven scatter with Golombek-Rapp distribution (Phase 4)
- Drone/rover integration into jezero_c.sdf (Phase 5)
- Atmospheric fog for horizon fade (Phase 2.5 polish, optional)

---

## Tier-3 world build state (finalized Apr 17)

### Final approach: mesh-based terrain
Heightmap (`<heightmap>` SDF) approach **abandoned** after diagnosing that neither dartsim nor classic Bullet nor bullet-featherstone implement `ConstructSdfCollision` for heightmaps in gz-physics7 / Harmonic. This is a system-wide gap, not an engine-specific limitation. Diagnosed via the sequence:
1. Tier A (PROJ-only, dartsim default): no SDFFeatures.cc:864 complaint gone, but scene/info showed visual only (no collision). Dartsim's ODE-level Heightfield AABB path orphans the geometry.
2. Tier C (classic Bullet engine override): engine loaded correctly, but still no collision attached — same gap.
3. Tier E (mesh-based terrain): works. Requires per-vertex normals or dartsim segfaults in `OdeMesh::fillArrays`.

### Terrain pipeline (authoritative)
```
Raw HiRISE TIFFs (120MB + 1.6GB, gitignored)
  ↓ gdal_translate -projwin
Cropped 40x40m patch
  ↓ pure osgeo.gdal + struct normalization (no gdal_array, NumPy ABI issue)
Normalized 0-3.08m relief, Float32 GeoTIFF
  ↓ symmetric nearest-edge padding
65x65 (2^6+1) heightmap.tif  [committed]
  ↓ scripts/dtm_to_obj.py
jezero_terrain.obj (4225 verts, 8192 tris, UVs, per-vertex normals)  [committed]
jezero_terrain.mtl (Mars-ochre material + ortho texture)  [committed]
```

### Decisions locked in
- **Tile:** Jezero_C (ESP_045994_1985 + ESP_046060_1985 stereo pair)
- **Patch:** 40 × 40 m, 300 m east + 20 m south of Octavia E. Butler Landing
- **Terrain representation:** `.obj` mesh, UV-mapped to 257×257 ortho PNG (full-terrain / Option A UV mapping)
- **Physics engine:** dartsim default (no <engine> override)
- **Launch env:** `PROJ_IGNORE_CELESTIAL_BODY=YES` (required)

### Measured terrain characteristics (from HiRISE data)
- Full Jezero_C tile: 7.06 × 14.44 km, elevation -2665 to -2396 m (below Mars 2000 Sphere ref, correct for Jezero basin)
- Our 40 × 40 m patch: **relief 3.08 m**, mean slope ~4.4% (~2.5°), StdDev 0.888 m

### Asset files (after Apr 17)
```
~/ros2_ws/src/midterm_project/worlds/jezero_c/
├── dtm/
│   ├── DTM_...tif                              [120 MB, GITIGNORED, see jezero_c/README.md]
│   ├── jezero_c_patch_40m_raw.tif              [7 KB, committed]
│   ├── jezero_c_patch_40m_normalized.tif       [5 KB, committed]
│   ├── jezero_c_heightmap_65.tif               [9 KB, committed, legacy from heightmap attempt]
│   └── jezero_c_heightmap_65.png               [1 KB, committed, legacy]
├── ortho/
│   ├── ESP_045994_1985_...tif                  [1.6 GB, GITIGNORED, see jezero_c/README.md]
│   ├── jezero_c_patch_40m.tif                  [26 KB, committed]
│   └── jezero_c_texture_257.png                [30 KB, committed, used as texture]
└── terrain_mesh/
    ├── jezero_terrain.obj                      [718 KB, committed]
    └── jezero_terrain.mtl                      [280 B, committed]
```

### World SDF
- `~/ros2_ws/src/midterm_project/worlds/jezero_c.sdf` (2465 bytes, XML-valid)
- Contains: physics (dartsim default) + plugins + Mars sky/sun + `<mesh>` model pointing at .obj
- **Box-drop collision test verified Apr 17 21:20 local** — 0.5m cube dropped at (0,0,5) came to rest at z≈1.60m on real terrain topography

---

## Key spawn poses (from Tier-1/2 world SDF, not yet migrated to jezero_c.sdf)

| Entity | Spawn pose | Frame |
|---|---|---|
| Drone | (0, 0, 0.2) | world |
| Rover | (0, 1.5, 0.12) | world |
| Dome | (15, 0, 0), radius 1.0 m sphere | world |

For Phase 5 (drone/rover integration in jezero_c.sdf), spawn z-values must account for terrain elevation at (x,y). Mean terrain height at origin is ~1.35 m. Spawn drone at e.g. z=2.0, rover at z=1.7 to start on-surface.

---

## Key file paths

- Project root: `~/ros2_ws/src/midterm_project/`
- Flight node: `midterm_project/smart_flight_node.py`
- Rover node: `midterm_project/smart_rover_node.py`
- Flight node log (runtime): `/tmp/smart_flight.log`
- Rover node log (runtime): `/tmp/smart_rover.log`
- Bridge log: `/tmp/bridge.log`
- Launch script: `scripts/run_live_demo.sh`
- Tier-1/2 world: `worlds/mars_mission.sdf`
- **Tier-3 world:** `worlds/jezero_c.sdf`
- **Tier-3 asset dir:** `worlds/jezero_c/` (see `worlds/jezero_c/README.md` for data sources)
- **Terrain mesh converter:** `scripts/dtm_to_obj.py`
- **SDF rewriter:** `scripts/write_jezero_sdf.py`
- x500_base (landing gear reference): `~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`
- GitHub: `github.com/JatinSatyam26/ses598-space-robotics-and-ai-2026`

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
| **Jezero_C patch size** | 64 × 64 m (padded from 40 × 40) | **Mesh is 64x64m, data is 40x40m center** |
| **Jezero_C patch relief** | **3.08 m** | **Max - min from real HiRISE DTM** |
| **Mars 2000 Sphere radius** | **3,396,190 m** | **For lat/lon ↔ projection coord math** |
| **Butler Landing projection XY** | **(4590878.7, 1093298.1) m** | **Derived: R × radians(lon), R × radians(lat)** |
| **Patch center (projection)** | **(4591197.8, 1093298.0) m** | **300 m east, 20 m south of Butler Landing** |
| **Terrain mesh vertex count** | **4225** | **65×65 grid** |
| **Terrain mesh triangle count** | **8192** | **64×64 quads × 2 triangles** |
| **Box-drop verification z** | **1.60 m** | **Box rested on real terrain topography (Apr 17)** |

---

## Topic map

| Topic | Type | Direction | Source/Sink |
|---|---|---|---|
| `/drone/cmd_vel` | geometry_msgs/Twist | ROS→GZ | flight node → `/flying_drone/gazebo/command/twist` |
| `/drone/odom` | nav_msgs/Odometry | GZ→ROS | drone OdometryPublisher |
| `/drone/occupancy_grid` | nav_msgs/OccupancyGrid | ROS internal | flight node → rover node |
| `/rover/cmd_vel` | geometry_msgs/Twist | ROS→GZ | rover node → `/model/rover/cmd_vel` (via bridge) |
| `/rover/odom` | nav_msgs/Odometry | GZ→ROS | rover DiffDrive plugin (**START-RELATIVE**) |
| `/rover_lidar` | sensor_msgs/LaserScan | GZ | rover 360° LiDAR |
| `/flying_drone/enable` | gz.msgs.Boolean | GZ only | controller enable/disable |
| `/world/*/pose/info` | gz.msgs.Pose_V | GZ only | all-entity ground truth poses |

---

## Environment gotchas (observed, reproducible)

| Issue | Symptom | Workaround |
|-------|---------|------------|
| **NumPy 2.4.4 vs python3-gdal 3.8.4 ABI** | `gdal_calc.py` fails with `_ARRAY_API not found`, `ImportError: numpy.core.multiarray failed to import`. Also: `gdal.UseExceptions()` triggers the same import failure as a side effect. | Use pure `osgeo.gdal` + `struct.unpack()` for raster IO. Avoid `gdal_array`, `gdal.UseExceptions()`, and `gdal_calc.py`. Do NOT downgrade NumPy — it'll break ROS2 Jazzy deps. |
| **Paste heredocs >30 lines corrupt** | Characters from later in block injected mid-heredoc, silent file truncation | Two-step Python write; for >1KB files, download-and-move pattern |
| **Gazebo PROJ Mars→Earth rejection** | `Source and target ellipsoid do not belong to the same celestial body` during DEM load | `export PROJ_IGNORE_CELESTIAL_BODY=YES` before `gz sim` (required, not optional) |
| **Gazebo heightmap collision unsupported in gz-physics7** | `SDFFeatures.cc:318 Heightmap construction from an SDF has not been implemented yet for dartsim`. Collision shape silently dropped, `scene/info` shows visual only. Classic Bullet & bullet-featherstone have same gap. | Convert DTM to triangulated mesh (`.obj`) — `scripts/dtm_to_obj.py` |
| **Dartsim mesh loader requires vertex normals** | `CustomMeshShape.cc:144 One of the submeshes ... does not have a normal count [0] that matches its vertex count [N]. This submesh will be ignored!` → then segfault in `OdeMesh::fillArrays` | Write OBJ with explicit `vn` lines + `f v/vt/vn v/vt/vn v/vt/vn` face syntax. `s off` not `s 1`. |
| **gz service multi-line protobuf text format** | `String literals cannot cross line boundaries` when passing inline SDF as `sdf:` field | Write SDF to disk, pass `sdf_filename:` instead of `sdf:` |
| **scene/info reports rendering only** | `collision` blocks absent from scene/info even when collision is present in simulation | Verify collision via dynamic test (drop a box, watch z), not scene/info |
| **exit 1 inside pasted command block** | Terminal closes | Use `exit 0` in scripts, or restructure without explicit exit |
| **Canonical mirror flakiness** | `apt install` returns DNS/timeout after `apt update` succeeds | Retry with `--fix-missing`. Usually transient. |
| **`convert: Request did not return an image.`** | ImageMagick with no args | Not an error, IM's idiomatic message. Ignore. |

---

## Tier-3 phase roadmap

- [x] **Phase 0:** Environment diagnostic
- [x] **Phase 0B:** GDAL + Blender install
- [x] **Phase 1A:** HiRISE DTM + ortho download (Jezero_C, ~1.7 GB)
- [x] **Phase 1B:** Metadata extraction + Butler Landing projection coord calc
- [x] **Phase 1C:** Crop 40×40 m patch + normalize DTM
- [x] **Phase 1D:** Pad DTM to 65×65, generate Mars-ochre texture
- [x] **Phase 1E:** Visual sanity check
- [x] **Phase 2 (heightmap):** Tried heightmap, diagnosed system-wide gap, pivoted
- [x] **Phase 2 (mesh):** Mesh-based terrain, collision verified via box-drop test (Apr 17 end-of-day)
- [ ] **Phase 2.5 (optional polish):** Atmospheric fog for horizon fade
- [ ] **Phase 3:** Mastcam-Z hero meshes (Sketchfab CC-Attribution), chop in Blender
- [ ] **Phase 4:** Poly Haven scatter meshes, Golombek-Rapp distribution
- [ ] **Phase 5:** Integrate drone, rover, dome into jezero_c.sdf
- [ ] **Phase 6:** Verify survey + navigation on new terrain
- [ ] **Phase 7:** Ground-truth logger
- [ ] **Phase 8:** Confidence-Rich Grid Mapping on drone
- [ ] **Phase 9:** Scan-to-map matcher (hierarchical correlation + branch-and-bound)
- [ ] **Phase 10:** Pose fusion EKF
- [ ] **Phase 11:** Modified rover node consuming fused pose
- [ ] **Phase 12:** Integration run + localization error curve

---

## Next-action checklist — START OF NEXT SESSION

State at start of next session: Phase 2 complete, committed, pushed. World loads cleanly, terrain renders, collision works.

**Suggested next moves, in order of value:**

1. **Phase 5 (integration) first, Phase 3 (pretty rocks) later.** Integrating the drone + rover + dome into jezero_c.sdf gets end-to-end testing going on the new terrain. Rocks are cosmetic; integration is functional.
2. Copy the drone/rover/dome model includes from `mars_mission.sdf` into `jezero_c.sdf`. Adjust spawn poses for terrain height (~1.5m at origin).
3. Run `scripts/run_live_demo.sh` with `WORLD_SDF=worlds/jezero_c.sdf` override — see if survey + navigation work on uneven mesh terrain.
4. Expect issues: DiffDrive on triangulated mesh may behave differently than on flat plane. LaserScan returns from angled terrain may cause false positives in occupancy grid.

**Alternative next move (if looks-matter):** Phase 2.5 polish — add fog, debug the dark-terrain rendering. 30-60 min.

---

## Bug archaeology (one-line per historical bug for context)

- **Phase 60 (Apr 3):** QoS mismatch, topic names, TrajectorySetpoint NaNs, `/clock` death spiral — all PX4-era, obsoleted by Apr 9 pivot
- **Apr 9 pivot:** abandoned PX4, went native Gazebo plugins
- **Apr 15 prior session:** identified landing bug hypotheses
- **Apr 16 drone:** landing bug was velocity-controller-has-no-disarm; fixed
- **Apr 16 rover:** cascade of arrival-too-loose → proximity-fires-early → coord-frame-offset
- **Apr 16 localization pivot:** rejected hardcoded fix; building real TRN stack
- **Apr 17 AM DTM preparation:** NumPy ABI blocked gdal_calc.py, pivoted to pure osgeo.gdal Python
- **Apr 17 midday paste disaster:** heredoc corruption silently produced broken SDF; codified two-step Python write methodology in response
- **Apr 17 heightmap dead-end:** PROJ Mars rejection fixed via env var, but heightmap-from-SDF unsupported in gz-physics7 across all three engines (dartsim, classic Bullet, bullet-featherstone)
- **Apr 17 late:** pivoted to mesh-based terrain, first OBJ missing normals → dartsim segfault → v2 with per-vertex normals works
- **Apr 17 verification:** scene/info misleadingly reports rendering only; box-drop dynamic test confirmed collision works (box rested at z=1.60m on real HiRISE topography)

---

## Working preferences

- One command block at a time. No "run all of these."
- Keep command blocks to 1-3 commands max.
- For file creation >30 lines or heredocs, use two-step Python or download-and-move.
- User pastes output in a text file. Read it carefully, not skimmed.
- Catch logical inconsistencies. Don't hand-wave.
- Don't declare things fixed until visually verified in Gazebo OR via dynamic test.
- Don't offer deceptive/cosmetic options.
- Patch scripts should be idempotent with timestamped backups.
- Re-verify when suspicious. Don't assume.
- Calibrate confidence honestly: ~55% for "supported by adjacent docs with no direct test", ~85%+ only after empirical confirmation.
