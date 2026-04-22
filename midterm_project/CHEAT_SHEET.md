# SES 598 — Technical Invariants Cheat Sheet
**Last updated: April 22, 2026. Phases 8–10 and 12 complete. Baseline re-verified Apr 22 (88 occupied, 0 obstacle events, ARRIVED 0.73m, ~145s). shadows=false in jezero_c.sdf recovers ~5% RTF. Phase 3/4 rock attempts aborted — RTF drops to 60-65% with any static collision objects added. Rock strategy needs rethink before retry.**

---

## Project identity

- **Student:** Jatin Satyam, ASU, SES 598 Space Robotics and AI, Spring 2026
- **Stack:** Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic 8.10.0, native plugins only (NO PX4 runtime, NO MicroXRCE, NO QGC). **x500_base airframe is now vendored locally (BSD-3), project is PX4-install-free.**
- **Drone plugin:** `MulticopterVelocityControl` + `MulticopterMotorModel`
- **Rover plugin:** `DiffDrive` (6-wheel Perseverance-inspired design)
- **Working model:** Claude Opus 4.7 with extended thinking
- **Machine:** OMEN HP Gaming Laptop 16, RTX 4060, CUDA 12.8

---

## Timeline (as of Apr 18)

- **May 4, 2026** — OTHER assignment due (not started yet as of Apr 18). Expect to lose ~1 week of midterm time to it.
- **May 10, 2026** — Midterm project due. 22 days out. All phases in scope, nothing cut. Phase 2.5 cosmetic polish is reserved for last-if-time.
- **Work budget:** 12 hrs/day target. ~40–70 focused hours estimated to complete Phases 5–12.

---

## Epistemic rules (non-negotiable)

1. **Real physics only. No shortcuts. No deception of any kind.**
2. **Terminal truth and visual truth must always agree** before declaring anything "working."
3. **No hardcoded constants that encode world assumptions.** If the value depends on the world file, derive it at runtime.
4. **Ground-truth pose is for validation only**, never operational decisions.
5. **No cosmetic fixes.** Fix the physics, or report honest physics. Never massage numbers.
6. **Every SUCCESS/COMPLETE/ARRIVED message must be backed by a physical simulator condition**, not just a code path.
7. **(Apr 17)** Scene-graph introspection (`scene/info`) reports rendering components, not physics components. Collision geometry existence must be verified by dynamic test (e.g. drop a box), not by reading scene/info.
8. **(Apr 17)** Confidence calibration discipline: when a hypothesis is supported by multiple independent sources but has no *affirmative* test evidence, cap confidence at 60%, not 85%. The sanity check is what pushes confidence higher — not consensus of adjacent documentation.
9. **(New, codified Apr 18)** Log noise vs. real failure — on this machine, `libEGL warning: egl: failed to create dri2 screen` appears on every Gazebo launch. It is cosmetic. Gazebo runs fine despite these warnings. **Truth-criterion for "world loaded"** is: (a) GUI window shows the scene, AND (b) `gz model --list` reports all expected entities. **NOT** log's line count, **NOT** presence of `libEGL` warnings.
10. **(New, codified Apr 18)** Grep discipline for log triage — libEGL emits lowercase `warning`, not `Warning`. Case-sensitive `grep Err|Warning` will miss these. Always use `grep -iE "err|warn|fail|unable"` (case-insensitive) for first-pass log inspection.
11. **(New, codified Apr 18)** Equilibrium verification method — to verify a spawned entity has settled into a stable physical rest pose (not still drifting), query its pose twice separated by >5 simulated seconds after initial settling window. Delta < 10 μm in all components = stable equilibrium. Used to confirm Phase 5 rig stability.

---

## Terminal paste discipline (codified Apr 17, expanded Apr 18)

**Problem observed Apr 17:** Paste buffer corrupts heredocs larger than ~30 lines. Symptom is characters from *later* in the command block get injected *into* the heredoc before its closing delimiter, silently producing truncated/malformed files.

**Rule for any file creation >30 lines or any heredoc (especially XML/SDF):**

1. **Use a two-step Python write, not a direct heredoc.**
   - Step A: `cat > /tmp/write_step.py <<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` ... Python script with target file content as triple-quoted string ... `PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE`
   - Step B: `python3 /tmp/write_step.py`

2. **For large files (>1 KB) or during repeated cycles, prefer download-and-move** over any form of paste. Ask Claude to generate the file in its sandbox, download it, `mv ~/Downloads/FILE /tmp/` and run. Gold standard — no paste corruption risk at all.

3. **Always pre-validate inside scripts.** For XML/SDF: `ET.fromstring(SDF_STRING)` BEFORE writing to disk.

4. **Always post-validate after writing.** Read back from disk and re-validate.

5. **Use improbable delimiters.** Not `<<'SDF'` or `<<'EOF'`. Use `<<'PY_EOF_MARKER_XYZZY_DO_NOT_REMOVE'` or similar.

6. **Gazebo log to file first, dump via `tail` after.** `timeout N gz sim ... > /tmp/log 2>&1; tail -60 /tmp/log`. Killed processes truncate piped output unreliably; files capture everything.

7. **NEVER paste `exit 1` inside a command block.** When pasted into an interactive shell (not inside a script file), it terminates the shell and closes the terminal. Use `exit 0` or restructure without explicit exit.

8. **Pastes to the terminal must stay under ~300 characters.** The interactive shell's paste buffer is unreliable above this threshold even for single-line commands. For longer commands, use the download-and-move pattern — the 300-char limit only applies to the commands *invoking* the script, not the script contents.

9. **(New, codified Apr 18)** When modifying an existing XML file by adding elements, **prefer targeted text insertion over ElementTree parse-reserialize**. ElementTree's round-trip normalizes formatting — changes XML declaration, self-closing tag spacing, indentation — even for lines you didn't touch. Produces noisy diffs that obscure the real change. Surgical string-splice (find the literal closing tag line, insert before it) is cleaner when appending. ElementTree is correct when reshaping structure or when source formatting is already normalized.

10. **(New, codified Apr 18)** **`--` inside XML comments is illegal** per XML spec 2.5. `ET.Comment(" -- divider -- ")` will construct successfully but fail on round-trip parse. Use single hyphens in comment text: `ET.Comment(" divider ")` or `ET.Comment(" - Mission entities - ")`.

---

## Current system state

### Working (verified on real data)
- Drone takeoff, survey (10 waypoints), return home, land at z≈0.2 m with skids on ground (verified on flat Tier-1/2 world as of Apr 16)
- Controller disable on landing via `gz topic /flying_drone/enable data:false`
- Rover A* path planning with goal-replanner when goal cell is occupied
- Rover waypoint navigation through all planned waypoints (on flat Tier-1/2 world)
- **HiRISE Jezero_C DTM + ortho downloaded, cropped, prepared as Gazebo terrain assets (Apr 17)**
- **Mars-ochre orthoimage texture verified visually (Apr 17)**
- **Jezero_C mesh-based terrain loads in Gazebo, renders correctly, collision verified by box-drop test (Apr 17)**
- **PROJ_IGNORE_CELESTIAL_BODY=YES env var handles Mars CRS issue (Apr 17)**
- **6-wheel Perseverance-inspired rover model (mars_rover/model.sdf) (Apr 16)**
- **Downward rangefinder on drone for altitude hold (flying_drone/model.sdf) (Apr 16)**
- **Habitat dome model (habitat_dome/) (Apr 16)**
- **(Phase 5 — Apr 18)** x500_base model vendored locally (`models/x500_base/`, BSD-3 licensed). Project is PX4-install-free.
- **(Phase 5 — Apr 18)** Drone + rover + habitat_dome integrated into `jezero_c.sdf` as `<include>` blocks. Rig at offset (5, -20) in world frame, worst-case slope 2.14° across all spawn and path-sample points.
- **(Phase 5 — Apr 18)** All three entities reach stable zero-drift equilibrium within 10s of simulated physics on real HiRISE mesh terrain. Verified by 20s settle test: Δpose(t=10, t=20) < 10 μm in all components.

- **(Phase 6 — Apr 20)** End-to-end integration test on jezero_c.sdf: PARTIAL on first run (coord frame mismatch). Three run_live_demo.sh infrastructure bugs fixed. Drone altitude hold confirmed working (world-z modulated to maintain ~3m AGL, max z=4.55m over variable terrain). No crash, no rollover.
- **(Phase 7 — Apr 20)** Ground-truth pose logger `scripts/groundtruth_logger.py` built and verified. Logs world + rig-frame poses for drone/rover/dome at ~0.5 Hz to `/tmp/phase7_groundtruth.csv`. Integrated into run_live_demo.sh. First full-mission CSV: 217 rows, 133.9s, all 3 entities, zero gaps.
- **(Failure A fixed — Apr 20)** `smart_flight_node.py` now subtracts `odom_origin` (first odom reading) from all XY readings. Drone now surveys correct region: rig x[−2,18] y[−5,5] = world x[3,23] y[−25,−15].
- **(Phase 6 clean baseline — Apr 20)** Full end-to-end re-run on jezero_c.sdf: drone surveyed correct region (249 occupied cells), landed at z=0.134m with controller disabled, rover planned 19-WP A* path and reported "ARRIVED at habitat dome" (1.08m from dome center per odom). Ground-truth: rover physical displacement only 1.523m (Failure C persists — 90% wheel slip). Drone max world-z=3.747m.
- **(Failure C fixed — Apr 20)** Rover traction fixed: added terrain surface μ=3.0 + contact params to jezero_c.sdf; added 6WD (front + rear DiffDrive plugins with separate odom topics) to mars_rover/model.sdf. Physical displacement: 1.523m → 13.783m (9x improvement). Rover now physically traverses terrain to dome.

- **(Phase 8 — Apr 20)** Bayesian log-odds occupancy grid replaces count-based grid. L_FREE=−0.847, L_OCC=2.197, clamp ±10. Three-value output: −1 unknown | 0 free | 51–100 occupied. Threshold L>1.5 requires ≥1 clean rangefinder hit. TRANSIENT_LOCAL QoS eliminates sleep loop. A* penalises unknown cells ×3. Verified: 86 occupied, 765 free on jezero_c.sdf.
- **(Phase 9 — Apr 20)** TRN scan-to-map matcher node (`trn_node.py`). Builds Gaussian-blurred likelihood field from occupancy grid (pure numpy, no scipy). Vectorized (K×M) candidate scoring at 2 Hz, K=294 candidates (±0.75m XY, ±15° yaw). Dead-reckons from odom deltas between updates. Publishes PoseWithCovarianceStamped on `/rover/trn_pose`. Verified live: fix #1 improve=43.9%, fix #11 improve=88.6%, covariance converged 1.6→0.05 by fix #21.
- **(Phase 9b — Apr 20)** TRN feedback loop in `smart_rover_node`. Subscribes to `/rover/trn_pose`; blends correction at α=0.3 when cov<0.5 and magnitude<0.8m. Guards against divergence. Logs every 10th blend.
- **(Phase 10 — Apr 20)** TRN-triggered A* re-planning. Accumulates applied TRN correction; when >0.6m AND cooldown=0 AND >2 WPs remain, re-runs A* from corrected position. 5s cooldown (50 ticks at 10 Hz). Keeps old path if A* fails. Verified live: no re-plan fired (small corrections, original path valid — correct healthy outcome).
- **(Phase 12 — Apr 20)** Localization error analysis. `scripts/phase12_logger.py` + `scripts/phase12_analysis.py`. Odom mean error 0.965m, TRN mean 2.699m (bias-corrected). TRN benefit confirmed at t=97–108s (+15–24%). trn_node.py params tightened: SEARCH_YAW 0.087 rad, MIN_VALID_RAYS 8, MIN_SCORE_IMPROVE 0.05.
- **(Baseline re-verified — Apr 22)** Clean run after Phase 3/4 rock attempts abandoned. shadows=false in jezero_c.sdf. Result: 88 occupied, 761 free, 0 obstacle events, ARRIVED at 0.73m from dome, 19-WP path, ~145s wall-clock. Gazebo cache cleared. This is the confirmed healthy baseline.

### In progress
- Nothing. Baseline re-verified Apr 22 with shadows=false. Phase 3/4 rock enrichment deferred — needs RTF strategy.

### Broken (diagnosed, not yet fixed)
- Status printer in launch script shows `WP 0/8` always. Cosmetic, dishonest, known.
- Simulation does not self-terminate after mission complete. Operational nuisance.

### Previously broken — now fixed
- **Rover localization (Failures A+C):** FIXED via odom_origin subtraction (Phase 6), terrain μ=3.0 + 6WD (Failure C), and TRN feedback loop (Phases 9–10).
- **Drone survey coord frame (Failure A, Apr 20):** FIXED (commit c829a03).
- **Rover odom 90% wheel slip (Failure C, Apr 20):** FIXED (commit 543effb). Physical displacement 1.523m → 13.783m.

### Known cosmetic issues (Apr 17-18)
- **Terrain visually darker than expected in Gazebo GUI.** Ortho PNG is confirmed ochre stand-alone, but terrain in-scene reads closer to dark chocolate. Gazebo tone-mapping / ogre2 auto-exposure interacting with the bright butterscotch sky background. Not blocking. Address in Phase 2.5 polish pass if time (reserved for last).

### Not yet built
- Rock feature enrichment (Phase 3/4) — blocked by RTF; see roadmap for options
- Atmospheric fog for horizon fade (Phase 2.5 polish, optional, last)

---

## Tier-3 world build state (finalized Apr 18)

### Final approach: mesh-based terrain + native-plugin entities
Heightmap SDF approach abandoned Apr 17 (gz-physics7 system-wide gap across dartsim, classic Bullet, bullet-featherstone). Mesh approach succeeded after adding per-vertex normals. Three entities integrated Apr 18.

### Terrain pipeline (authoritative)
```
Raw HiRISE TIFFs (120MB + 1.6GB, gitignored)
  ↓ gdal_translate -projwin
Cropped 40x40m patch
  ↓ pure osgeo.gdal + struct normalization (no gdal_array, NumPy ABI issue)
Normalized 0-3.08m relief, Float32 GeoTIFF
  ↓ symmetric nearest-edge padding
65x65 (2^6+1) heightmap.tif  [committed]
  ↓ scripts/dtm_to_obj.py (v2, with per-vertex normals)
jezero_terrain.obj (4225 verts, 8192 tris, UVs, per-vertex normals)  [committed]
jezero_terrain.mtl (Mars-ochre material + ortho texture)  [committed]
```

### Rig integration (Phase 5, finalized Apr 18)
```
jezero_c.sdf [committed, 3144 bytes]
  ├── <world name="jezero_c">
  ├── physics: dartsim default (no <engine> override)
  ├── sun, sky, atmosphere
  ├── <model name="jezero_terrain"> (mesh collision + visual)
  ├── <include> model://flying_drone → name=drone @ (5, -20, 0.223)
  ├── <include> model://mars_rover → name=rover @ (5, -18.5, 0.161)
  └── <include> model://habitat_dome → name=habitat_dome @ (20, -20, 0.230)
```

### Rig offset decision
- **Rig offset: (5, -20)** in world frame. Chosen by exhaustive 1-m grid search over candidate origins (ox ∈ [-14, +14], oy ∈ [-20, +20]) minimizing worst-case slope across 12 query points: 3 spawns + 9 samples of rover's straight-line path to dome.
- **Worst-case slope at the chosen rig: 2.14°** across all 12 query points (well inside DiffDrive's 10-15° stability limit).
- **Compare to the naïve (0,0) origin:** rover spawn slope was 16.58°, drone 10.85°. Catastrophic roll-on-spawn behavior.
- **Net rover elevation change to dome:** 0.19 m uphill over 15 m (0.72° average).

### Rig-relative vs. world-frame coordinate convention
- **Rover/drone node code continues using rig-relative coordinates.** Rover goal: (15, 0) rig-frame (= (20, -20) world frame). Survey extent: x ∈ [-2, 18], y ∈ [-5, 5] rig-frame.
- **The offset lives ONLY in `jezero_c.sdf`** in the three `<include>` `<pose>` tags. Nowhere else.
- **This is consistent with the experimental design:** the rover's DiffDrive odom starts at (0,0) in its own frame and never knows the rig offset. The TRN stack (Phases 8-10) is what will eventually bridge rig-frame to world-frame — exactly the problem the research is set up to solve.

### Vendored dependency: x500_base
- `models/x500_base/` (27 MB) — vendored copy of the PX4 x500 airframe.
- **License: BSD-3-Clause**, Copyright Rudis Laboratories 2022, author Benjamin Perseghetti.
- `LICENSE` file preserved in the directory. BSD-3 requires: (a) copyright notice retained in source redistribution [done], (b) copyright notice in documentation [the LICENSE file satisfies this], (c) no use of copyright-holder names to endorse [trivially satisfied].
- **Source:** NXP HoverGames Drone Development Kit (KIT-HGDRONEK66) reference model.
- `flying_drone/model.sdf` uses `<include merge="true"><uri>model://x500_base</uri></include>` — resolves against the project's own `models/` directory, no PX4 install needed.

### Decisions locked in
- **Tile:** Jezero_C (ESP_045994_1985 + ESP_046060_1985 stereo pair)
- **Patch:** 40 × 40 m, 300 m east + 20 m south of Octavia E. Butler Landing
- **Terrain representation:** `.obj` mesh, UV-mapped to 257×257 ortho PNG
- **Physics engine:** dartsim default (no `<engine>` override)
- **Launch env:** `PROJ_IGNORE_CELESTIAL_BODY=YES` (required)
- **GZ_SIM_RESOURCE_PATH:** `$PROJECT_DIR/models:$GZ_SIM_RESOURCE_PATH:-` (no longer includes `$PX4_MODELS`)

### Measured terrain characteristics (from HiRISE data)
- Full Jezero_C tile: 7.06 × 14.44 km, elevation −2665 to −2396 m
- Our 40 × 40 m patch (center): relief 3.08 m, mean slope ~4.4%, StdDev 0.888 m
- **At Phase 5 rig (5, -20):** terrain z = 0.023 m (drone), 0.041 m (rover), 0.230 m (dome)
- **Mesh world extent:** x ∈ [-32, +32], y ∈ [-32, +32], z ∈ [0.000, 3.080], mean z = 1.164 m

### Asset files (as of Apr 18)
```
~/ros2_ws/src/midterm_project/worlds/jezero_c/
├── dtm/
│   ├── DTM_...tif                              [120 MB, GITIGNORED, see jezero_c/README.md]
│   ├── jezero_c_patch_40m_raw.tif              [7 KB, committed]
│   ├── jezero_c_patch_40m_normalized.tif       [5 KB, committed]
│   ├── jezero_c_heightmap_65.tif               [9 KB, committed, legacy]
│   └── jezero_c_heightmap_65.png               [1 KB, committed, legacy]
├── ortho/
│   ├── ESP_045994_1985_...tif                  [1.6 GB, GITIGNORED]
│   ├── jezero_c_patch_40m.tif                  [26 KB, committed]
│   └── jezero_c_texture_257.png                [30 KB, committed, texture]
└── terrain_mesh/
    ├── jezero_terrain.obj                      [718 KB, committed]
    └── jezero_terrain.mtl                      [280 B, committed]

~/ros2_ws/src/midterm_project/models/x500_base/
├── LICENSE                                      [1.5 KB, BSD-3]
├── model.config                                 [773 B]
├── model.sdf                                    [17 KB]
├── materials/textures/                          [1.6 MB]
├── meshes/
│   ├── NXP-HGD-CF.dae                           [22 MB, frame mesh]
│   ├── 5010Base.dae, 5010Bell.dae               [motor components]
│   ├── 1345_prop_ccw.stl, 1345_prop_cw.stl      [propellers]
│   └── CF.png                                   [1.6 MB texture]
└── thumbnails/                                  [168 KB, editor previews, committed]
```

### World SDF
- `worlds/jezero_c.sdf` (3144 bytes, XML-valid)
- Contains: physics + plugins + Mars sky/sun + jezero_terrain mesh + 3 `<include>` blocks
- **Box-drop collision test verified Apr 17 21:20** — 0.5m cube dropped at (0,0,5) came to rest at z≈1.60m
- **Phase 5 integration verified Apr 18 ~05:35** — all 3 entities reach stable equilibrium in 10s, render correctly, physics stable

---

## Key spawn poses (world frame, Phase 5)

| Entity | World pose | Rig-relative | Clearance | Terrain z | Local slope |
|---|---|---|---|---|---|
| Drone | (5, -20, 0.223) | (0, 0) | 0.2 m skids | 0.023 m | 1.92° |
| Rover | (5, -18.5, 0.161) | (0, 1.5) | 0.12 m wheels | 0.041 m | 2.03° |
| Dome | (20, -20, 0.230) | (15, 0) | 0 (static) | 0.230 m | 3.96° |

### Stable post-settling poses (after 10s physics)
| Entity | Final XYZ | Final RPY (rad) |
|---|---|---|
| Drone | (4.9995, -20.0000, 0.0115) | (0.00, 0.033, 0.00) |
| Rover | (5.2299, -18.4987, 0.1017) | (0.018, 0.023, 0.006) |
| Dome | (20.0000, -20.0000, 0.2300) | (0, 0, 0) |

**Note on rover settling offset:** rover spawned at x=5.0, settled at x=5.23 — drifted 23 cm east into a stable rest against a mesh facet during first ~8s of physics. Then 0-drift thereafter. This is a one-time settling offset, not ongoing motion.

---

## Key file paths

- Project root (git): `/home/jatin-satyam/ses598-space-robotics-and-ai-2026/`
- Midterm dir: `~/ros2_ws/src/midterm_project/`
- Flight node: `midterm_project/smart_flight_node.py`
- Rover node: `midterm_project/smart_rover_node.py`
- Flight node log (runtime): `/tmp/smart_flight.log`
- Rover node log (runtime): `/tmp/smart_rover.log`
- Bridge log: `/tmp/bridge.log`
- Launch script: `scripts/run_live_demo.sh` (no longer references PX4; includes ground-truth logger)
- Ground-truth logger: `scripts/groundtruth_logger.py` — output `/tmp/phase7_groundtruth.csv` (CSV: wall_time, entity, world_xyz, rig_xy, rpy)
- Phase 6 report: `PHASE6_REPORT.md`
- Tier-1/2 world: `worlds/mars_mission.sdf` (flat ground, legacy)
- **Tier-3 world:** `worlds/jezero_c.sdf` (Phase 5 integrated)
- **Tier-3 asset dir:** `worlds/jezero_c/` (see `worlds/jezero_c/README.md`)
- **Terrain mesh converter:** `scripts/dtm_to_obj.py`
- **SDF rewriter:** `scripts/write_jezero_sdf.py`
- **Vendored x500:** `models/x500_base/` (BSD-3)
- x500_base upstream reference: `~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/` (no longer a runtime dependency — kept locally as upstream source of truth for future diffs)
- GitHub: `github.com/JatinSatyam26/ses598-space-robotics-and-ai-2026`

---

## Key numbers

| Value | Current | Why |
|---|---|---|
| Drone LAND threshold | `pos[2] < 0.23` | x500 skids touch ground when base_link at z=0.227 |
| Rover goal (rig-frame) | `(15.0, 0.0)` | Dome center in rig frame |
| Rover arrival method | Path completion (not proximity) | Approach B, after proximity kept producing false positives |
| Rover arrival sanity | Final WP within 2.0 m of dome | Dome radius 1.0 + grid cell 0.25 + margin |
| Occupancy grid resolution | 0.25 m | Sets floor on scan-match accuracy |
| Grid extent (rig-frame) | x=[-2, 18], y=[-5, 5] | From flight node config |
| LiDAR stop distance | 0.55 m | From `obstacle_stop_dist` |
| **Log-odds L_FREE** | **−0.847** | log(0.3/0.7) — free cell update |
| **Log-odds L_OCC** | **+2.197** | log(0.9/0.1) — obstacle update |
| **Log-odds clamp** | **±10.0** | Prevents saturation lock |
| **Occupied publish threshold** | **L > 1.5** | Requires ≥1 clean rangefinder hit |
| **TRN blur sigma** | **1.5 cells = 0.375 m** | Gaussian spread on likelihood field |
| **TRN candidates K** | **294** | ±0.75m XY at 0.25m step, ±15° yaw at 5° step |
| **TRN update rate** | **2 Hz** | `_update` timer period |
| **TRN covariance accept** | **cov[0] < 0.5** | σ < ~0.7m before blending into rover |
| **TRN blend alpha** | **0.3** | Fraction of correction applied per tick |
| **TRN re-plan threshold** | **0.6 m** | Accumulated applied correction before A* re-plan |
| **TRN re-plan cooldown** | **50 ticks = 5 s** | Minimum gap between re-plans at 10 Hz |
| **Jezero_C patch size** | 64 × 64 m (padded from 40 × 40) | Mesh is 64×64m, data is 40×40m center |
| **Jezero_C patch relief** | **3.08 m** | Max - min from real HiRISE DTM |
| **Mars 2000 Sphere radius** | **3,396,190 m** | For lat/lon ↔ projection coord math |
| **Butler Landing projection XY** | **(4590878.7, 1093298.1) m** | R × radians(lon), R × radians(lat) |
| **Patch center (projection)** | **(4591197.8, 1093298.0) m** | 300 m east, 20 m south of Butler Landing |
| **Terrain mesh vertex count** | **4225** | 65×65 grid |
| **Terrain mesh triangle count** | **8192** | 64×64 quads × 2 triangles |
| **Box-drop verification z** | **1.60 m** | Box rested on terrain topography (Apr 17) |
| **Phase 5 rig offset** | **(5, -20)** | Min worst-case slope from grid search |
| **Phase 5 worst-case slope** | **2.14°** | Max across 12 query points at chosen rig |
| **Phase 5 settle time** | **<10s** | All entities at 0-drift equilibrium |

---

## Topic map

| Topic | Type | Direction | Source/Sink |
|---|---|---|---|
| `/drone/cmd_vel` | geometry_msgs/Twist | ROS→GZ | flight node → `/flying_drone/gazebo/command/twist` |
| `/drone/odom` | nav_msgs/Odometry | GZ→ROS | drone OdometryPublisher |
| `/drone/occupancy_grid` | nav_msgs/OccupancyGrid | ROS internal | flight node → rover node |
| `/rover/cmd_vel` | geometry_msgs/Twist | ROS→GZ | rover node → `/model/rover/cmd_vel` (via bridge) |
| `/rover/odom` | nav_msgs/Odometry | GZ→ROS | rover DiffDrive plugin (**START-RELATIVE, rig-frame**) |
| `/rover/lidar` | sensor_msgs/LaserScan | GZ→ROS | rover 360° LiDAR (bridged from `/rover_lidar`) |
| `/rover/trn_pose` | geometry_msgs/PoseWithCovarianceStamped | ROS internal | TRN node → rover node (corrected rig-frame pose) |
| `/flying_drone/enable` | gz.msgs.Boolean | GZ only | controller enable/disable |
| `/world/*/pose/info` | gz.msgs.Pose_V | GZ only | all-entity ground truth poses |

---

## Environment gotchas (observed, reproducible)

| Issue | Symptom | Workaround |
|-------|---------|------------|
| **NumPy 2.4.4 vs python3-gdal 3.8.4 ABI** | `gdal_calc.py` fails with `_ARRAY_API not found`. `gdal.UseExceptions()` triggers same failure. | Use pure `osgeo.gdal` + `struct.unpack()`. Avoid `gdal_array`, `gdal.UseExceptions()`, `gdal_calc.py`. Do NOT downgrade NumPy. |
| **Paste heredocs >30 lines corrupt** | Characters from later in block injected mid-heredoc, silent file truncation | Two-step Python write; for >1KB files, download-and-move pattern |
| **Pastes >300 chars unreliable** | Interactive shell truncates or corrupts commands above this threshold | Download-and-move; keep terminal commands ≤300 chars |
| **Gazebo PROJ Mars→Earth rejection** | `Source and target ellipsoid do not belong to the same celestial body` during DEM load | `export PROJ_IGNORE_CELESTIAL_BODY=YES` before `gz sim` (required, not optional) |
| **Gazebo heightmap collision unsupported in gz-physics7** | `SDFFeatures.cc:318 Heightmap construction from an SDF has not been implemented yet`. Classic Bullet & bullet-featherstone same gap. | Convert DTM to triangulated mesh — `scripts/dtm_to_obj.py` |
| **Dartsim mesh loader requires vertex normals** | `CustomMeshShape.cc:144 One of the submeshes ... does not have a normal count [0]` → segfault in `OdeMesh::fillArrays` | Write OBJ with explicit `vn` lines + `f v/vt/vn ...` face syntax. `s off` not `s 1`. |
| **gz service multi-line protobuf text format** | `String literals cannot cross line boundaries` when passing inline SDF as `sdf:` field | Write SDF to disk, pass `sdf_filename:` |
| **scene/info reports rendering only** | `collision` blocks absent from scene/info even when collision is present | Verify collision via dynamic test (drop a box), not scene/info |
| **exit 1 inside pasted command block** | Terminal closes | Use `exit 0` in scripts, or restructure |
| **(NEW Apr 18) libEGL warnings on RTX 4060 + Wayland** | Every `gz sim` launch emits `libEGL warning: egl: failed to create dri2 screen` plus `pci id for fd N: 10de:28e0, driver (null)` repeatedly. | **These are cosmetic, not errors.** Gazebo runs fine. Ignore the warnings. Criterion for "world loaded" is GUI window contents + `gz model --list`, not log noise. |
| **(NEW Apr 18) Headless `gz sim -s` stalls on this setup** | With `-s` (server-only) flag, Gazebo hangs on EGL init. `timeout 30` kills it, log is 12 lines of libEGL warnings, no world load. | Drop the `-s` flag. Use GUI-attached `gz sim -r worlds/*.sdf` and ignore the window. State queries (`gz model`, `gz topic`, `gz service`) work identically with GUI attached. |
| **(NEW Apr 18) `LIBGL_ALWAYS_SOFTWARE=1` triggers segfault in Gazebo** | Ogre2 explicitly requests hardware device; software-rendering override conflicts and SIGSEGV. | Not a valid workaround for GPU issues. Use GUI mode (see above) or fix the underlying driver. |
| **(NEW Apr 18) ElementTree XML round-trip reformats the file** | Parse→modify→serialize loses exact XML declaration style, self-closing tag whitespace, and indentation consistency. Diff shows unrelated changes. | For appending to existing XML, use targeted string splice (find `</world>`, insert before). Parse-reserialize is for reshaping, not appending. |
| **(NEW Apr 18) `--` in XML comments** | `<!-- -- foo -- -->` accepted at construct time, rejected on parse. XML spec 2.5 forbids double-hyphen inside comments. | Use single hyphens or em-dash unicode. ASCII single-dash is safest. |
| **(NEW Apr 18) Case-sensitive log grep misses `libEGL warning:`** | `grep Err|Warning` catches uppercase messages; Mesa emits lowercase `warning`. | Use `grep -iE "err\|warn\|fail\|unable"` for first-pass triage. |
| **Canonical mirror flakiness** | `apt install` DNS/timeout after `apt update` succeeds | Retry with `--fix-missing`. Usually transient. |
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
- [x] **Phase 2 (mesh):** Mesh-based terrain, collision verified via box-drop test (Apr 17)
- [x] **Phase 5:** Integrate drone + rover + habitat_dome into jezero_c.sdf (Apr 18). Rig offset (5, -20), stable equilibrium verified.
- [x] **Phase 6:** Integration test on jezero_c.sdf (Apr 20). PARTIAL — drone coord frame bug + rover 90% slip. Three infra bugs fixed. Full findings in PHASE6_REPORT.md.
- [x] **Phase 7:** Ground-truth pose logger `scripts/groundtruth_logger.py` (Apr 20). Integrated into run_live_demo.sh. Verified: 217-row CSV over full mission.
- [x] **Phase 8:** Bayesian log-odds occupancy grid (Apr 20). Replaces count-based grid. TRANSIENT_LOCAL QoS, A* unknown-cell ×3 cost. Verified: 86 occupied, 765 free, rover ARRIVED at 0.73m from dome.
- [x] **Phase 9:** TRN scan-to-map matcher (Apr 20). `trn_node.py` — likelihood field + vectorized K×M scoring at 2 Hz. Verified: 41+ fixes, covariance 1.6→0.05 by fix #21. Publishes `/rover/trn_pose`.
- [x] **Phase 9b:** TRN feedback loop in rover (Apr 20). Soft-blend α=0.3, cov guard, magnitude guard.
- [x] **Phase 10:** TRN-triggered A* re-planning (Apr 20). 0.6m threshold, 5s cooldown, near-goal skip, A* failure fallback. Verified live: no re-plan fired (small corrections = original path valid — healthy).
- [ ] **Phase 3:** Rock feature enrichment — ATTEMPTED Apr 22, ABORTED. Box rocks at rig y=+2 caused rover collision; RTF dropped to 65% with any static collision objects on HiRISE mesh. Root cause: static bodies add broad-phase collision pairs against 8192-triangle terrain. Options for retry: (a) use ≤2 rocks at rig y=−2 only, (b) accept 65% RTF, (c) skip entirely.
- [ ] **Phase 4:** Procedural scatter — ATTEMPTED Apr 22, ABORTED. 8 OBJ-visual + box-collision rocks at rig y=−2: RTF ~60%, A* rerouting caused 212 dome-collision events. `scripts/generate_rocks_phase4.py` + `worlds/jezero_c/meshes_scatter/*.obj` on disk (untracked). Same RTF root cause as Phase 3.
- [x] **Phase 12:** Localization error curve (Apr 20). `scripts/phase12_logger.py` + `scripts/phase12_analysis.py`. Odom 0.965m mean error, TRN 2.699m bias-corrected. TRN benefit t=97–108s (+15–24%). trn_node params tightened.
- [ ] **Phase 2.5 (optional polish, LAST):** Atmospheric fog, dark-terrain debug

Note: Phase 3/4 rock enrichment blocked by RTF degradation on HiRISE mesh. TRN works with 86–88 occupied cells from bare terrain (covariance floors at 0.05). Rocks are a nice-to-have, not a must-have. Proceed to report + demo recording if deadline pressure rises.

---

## Next-action checklist — START OF NEXT SESSION

State at start of next session: Phases 8–10 and 12 complete (Apr 20–22). Baseline re-verified Apr 22: 88 occupied, 0 obstacle events, ARRIVED 0.73m, ~145s. shadows=false committed. 18 days to May 10 (May 4 other assignment pending).

**Suggested next moves, in order of value:**

1. **Record demo video.** Full pipeline working. Run `WORLD_SDF=worlds/jezero_c.sdf bash scripts/run_live_demo.sh` with screen recording. This is the primary deliverable.
2. **Write/complete report.** Phase 12 quantitative results (odom vs TRN error curve) are the key technical contribution. `scripts/phase12_analysis.py` already produces the plot.
3. **Phase 3/4 retry (optional).** Only if demo video looks thin on visual richness. Try ≤2 rocks at rig y=−2, accept RTF ~80%, verify no dome-circling before committing.
4. **Phase 2.5 polish (last, if time).** Fix dark terrain tone-mapping, atmospheric fog.

---

## Bug archaeology (one-line per historical bug for context)

- **Phase 60 (Apr 3):** QoS mismatch, topic names, TrajectorySetpoint NaNs, `/clock` death spiral — all PX4-era, obsoleted by Apr 9 pivot
- **Apr 9 pivot:** abandoned PX4 runtime, went native Gazebo plugins. But x500 airframe file still PX4-sourced until Apr 18 vendoring.
- **Apr 15 prior session:** identified landing bug hypotheses
- **Apr 16 drone:** landing bug was velocity-controller-has-no-disarm; fixed
- **Apr 16 rover:** cascade of arrival-too-loose → proximity-fires-early → coord-frame-offset
- **Apr 16 localization pivot:** rejected hardcoded fix; building real TRN stack
- **Apr 17 AM DTM preparation:** NumPy ABI blocked gdal_calc.py, pivoted to pure osgeo.gdal Python
- **Apr 17 midday paste disaster:** heredoc corruption silently produced broken SDF; codified two-step Python write
- **Apr 17 heightmap dead-end:** PROJ Mars rejection fixed via env var, but heightmap-from-SDF unsupported in gz-physics7 across all three engines
- **Apr 17 late:** pivoted to mesh-based terrain, first OBJ missing normals → dartsim segfault → v2 with per-vertex normals works
- **Apr 17 verification:** scene/info misleadingly reports rendering only; box-drop dynamic test confirmed collision
- **Apr 18 AM Phase 5 rig:** naïve (0,0) spawn hit 16.58° slope. 1-m grid search picked (5,-20) with 2.14° max. Rig offset lives only in SDF, node code uses rig-relative coordinates.
- **Apr 18 x500 vendoring:** discovered `flying_drone/model.sdf` depended on `model://x500_base` which lived in `~/PX4-Autopilot/`. Vendored it locally (BSD-3), now project is PX4-install-free.
- **Apr 18 integration script churn:** v1 aborted on `--` in XML comment (caught by pre-validate). v2 wrote but ElementTree reformatted file cosmetically. v3 switched to targeted string splice — clean diff, surgical change. Lesson: parse-reserialize for reshaping, string splice for appending.
- **Apr 18 libEGL false alarm:** spent ~1h debugging libEGL warnings as a driver problem, including a reboot. Warnings were cosmetic all along; GUI-mode launches worked fine, headless `-s` mode is what's actually broken on this setup. Lesson: truth-criterion is "did the world load in the GUI," not "is the log short."
- **Apr 20 Phase 6 infra bugs (3):** run_live_demo.sh missing PROJ_IGNORE_CELESTIAL_BODY, missing worlds/ in GZ_SIM_RESOURCE_PATH, WORLD_SDF hardcoded. All fixed in same session.
- **Apr 20 Failure A (drone coord frame):** OdometryPublisher gives world-frame; flight node treats odom as rig-relative without initial-pose subtraction. Drone surveyed central terrain instead of rig region. FIXED via odom_origin capture + subtraction.
- **Apr 20 Failure C (rover wheel slip):** DiffDrive odom 10:1 vs physical on HiRISE mesh. Rover physically stationary while node declared ARRIVED. FIXED via terrain μ=3.0 + 6WD.
- **Apr 20 Phase 8 false positives:** First run had 591 occupied cells (over-marking). Root causes: radius=2 in _mark_obstacle (25 cells per event) + depth_cb forward camera at 3m AGL producing false obstacle hits from sloped terrain. Fixed: radius=1, threshold L>1.5, depth_cb disabled. Result: 591 → 75 occupied.
- **Apr 20 Phase 8 sleep loop → TRANSIENT_LOCAL:** Original map publish used `sleep(2)` + republish loop. Replaced with TRANSIENT_LOCAL QoS so subscriber always receives the map regardless of timing. Both publisher and subscriber must match.
- **Apr 20 Phase 9 TRN covariance:** `trn_cov` stored as scalar (not matrix diagonal). Published as `cov[0] = trn_cov²`. Rover guard checks `cov[0] < 0.5` → σ < ~0.7m. Confirmed: covariance floors at 0.0025 (σ=0.05m) after ~21 fixes.
- **Apr 20 Phase 10 re-plan not firing (healthy):** In validated run, accumulated TRN correction never reached 0.6m threshold. This is correct — TRN corrections were small (drift well-controlled by 6WD + μ=3.0), so original A* path remained valid. Re-plan guard working correctly.
- **Apr 22 Phase 3 RTF crash:** 6 static box rocks on HiRISE mesh dropped RTF from ~100% to 65%. Root cause: static bodies still participate in broad-phase collision detection; adding shapes to an 8192-triangle mesh terrain multiplies collision-pair work. Reverted.
- **Apr 22 Phase 3 rover collision:** Rocks at rig y=+2 (world y=−18) placed 0.5m from rover start (y=+1.5). Rock edge at y=1.725m, rover body ~0.5m wide. Effective zero clearance despite A* avoiding grid cells. Rover crashed physically.
- **Apr 22 Phase 4 RTF + dome-circling:** 8 OBJ-visual + box-collision rocks at rig y=−2 (safe from rover). RTF ~60% (shadows=false helped but mesh rendering adds GPU load). A* path rerouted around rocks, altering final dome approach angle → 212 dome-collision events (rover circling dome). Mission technically completed (ARRIVED 0.73m) but visually wrong. Root RTF cause same as Phase 3. Reverted. generate_rocks_phase4.py + meshes_scatter/*.obj kept on disk as reference (untracked).

---

## Working preferences

- One command block at a time. No "run all of these."
- Keep terminal command blocks to 1-3 commands max, under 300 chars total.
- For file creation >30 lines or heredocs, use two-step Python or download-and-move.
- User pastes output in a text file. Read it carefully, not skimmed.
- Catch logical inconsistencies. Don't hand-wave.
- Don't declare things fixed until visually verified in Gazebo OR via dynamic test OR via the 10-20s equilibrium check.
- Don't offer deceptive/cosmetic options.
- Patch scripts should be idempotent with timestamped backups.
- Re-verify when suspicious. Don't assume.
- Calibrate confidence honestly: ~55% for "supported by adjacent docs with no direct test", ~85%+ only after empirical confirmation.
- **(NEW Apr 18)** Hour counter is tracked across the session and reported at milestone transitions. Driver debugging and other environmental friction is called out but not counted against phase budgets.
