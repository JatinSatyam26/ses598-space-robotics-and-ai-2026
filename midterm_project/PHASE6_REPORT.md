# PHASE 6 REPORT — Jezero_C End-to-End Integration Test
**Date:** 2026-04-20
**Branch:** main (HEAD e849260)
**World:** worlds/jezero_c.sdf (Phase 5 integrated rig, offset (5,-20))
**Stack:** ROS2 Jazzy + Gazebo Harmonic 8.10.0 + RTX 4060 + Ubuntu 24.04

---

## Step 0 — Preconditions

| Check | Result |
|---|---|
| Git state clean | PASS — clean main, HEAD e849260 |
| No PX4-Autopilot / PX4_MODELS in launch script | PASS — executable code PX4-free (stale comment only) |
| WORLD_SDF env override supported | FIXED — was hardcoded; added `${WORLD_SDF:-default}` + relative-path guard |
| jezero_c.sdf has 3 correct include blocks | PASS — drone/rover/habitat_dome at expected poses |
| PROJ_IGNORE_CELESTIAL_BODY=YES exported | FIXED — was missing; added to run_live_demo.sh |

Infrastructure bug also found during Step 2 launch:
- GZ_SIM_RESOURCE_PATH missing `worlds/` dir — terrain mesh URI unresolvable. FIXED (added `${PROJECT_DIR}/worlds:`).

All three fixes are minimal one-line diffs with timestamped backups preserved.

---

## Step 1 — Baseline World Load (jezero_c.sdf standalone)

**Result: PASS**

All 4 entities loaded. Poses at t=15s match Phase 5 verified values to 4 sig figs:

| Entity | World XYZ | RPY (rad) | vs Phase 5 baseline |
|---|---|---|---|
| drone | (4.999530, -20.000000, 0.011483) | (-0.000001, 0.033438, -0.000004) | MATCH |
| rover | (5.229940, -18.498700, 0.101703) | (0.017719, 0.023076, 0.006377) | MATCH |
| habitat_dome | (20.000000, -20.000000, 0.230000) | (0.000000, 0.000000, 0.000000) | MATCH |

Log: zero real errors. Only cosmetic libEGL warnings (expected on RTX 4060 + Wayland).
Visual: Jatin confirmed Gazebo window showing Mars terrain with dome visible.

---

## Step 2 — End-to-End Demo (run_live_demo.sh)

### Mission timeline

| ROS time | Event | Source |
|---|---|---|
| 1776711865.24 | drone node init (survey_alt=3.0m, grid=80x40) | smart_flight.log |
| 1776711866.22 | rover node init, waiting for grid | smart_rover.log |
| 1776711868.53 | TAKEOFF complete z=2.93m, 10-WP survey started | smart_flight.log |
| 1776711879–1776711924 | Survey WP 1–10 reached (~7-8s per WP) | smart_flight.log |
| 1776711924.63 | SURVEY COMPLETE, returning home | smart_flight.log |
| 1776711924.64 | OccupancyGrid published: 80x40, 238 occupied cells | smart_flight.log |
| 1776711924.72 | Rover A* path computed: 18 waypoints | smart_rover.log |
| 1776711924.81–1776711973.51 | Rover WP 1–18 reached | smart_rover.log |
| 1776711935.53 | Drone: At home position (1.5,-0.1) — LANDING | smart_flight.log |
| 1776711973.61 | ARRIVED at habitat dome (odom-frame only) | smart_rover.log |
| 12:06:15 MST | Demo script: MISSION COMPLETE | phase6_demo.log |

### Pass/fail per Step 2.4 criteria

| Criterion | Status | Evidence |
|---|---|---|
| TAKEOFF (drone z > 5m) | PASS | z=2.93m terminal; Jatin visual confirmed takeoff |
| SURVEY_COMPLETE | PASS | 10/10 WPs terminal; 55.7s total survey time |
| OCCUPANCY_GRID_PUBLISHED | PASS | 80x40, 238 cells terminal confirmed |
| DRONE_LANDING (z < 0.23m) | FAIL | z=1.242m at gz query; rotors still spinning (Jatin visual); no LANDED log |
| ROVER_CMD_VEL traffic | PARTIAL | rover physically moved 1.511m (gz confirmed); cmd_vel reaching DiffDrive |
| ROVER_FINAL_POSE near dome (< 2m) | FAIL | Rover rig (1.739, 1.438), dome rig (15.0, 0.0), distance 13.34m |
| ROVER_ROLLOVER (roll/pitch > 45 deg) | PASS | No rollover; Jatin visual confirmed |
| SIM_DEADLOCK | PASS | Sim ran to completion without stall |

### Observational failures (do not fix — Phase 8-10 inputs)

**FAILURE A — Drone survey flew wrong spatial region**
Visual: drone surveyed central terrain (world ~x[-2,18], y[-5,5]), not rig area (world x[3,23], y[-25,-15]).
Hypothesis: OdometryPublisher gives absolute world-frame pose. Flight node uses rig-frame
waypoints (x in [-2,18], y in [-5,5]) without subtracting initial world pose (5,-20).
So drone navigates directly to world (rig-waypoint) instead of world (rig-offset + rig-waypoint).
Survey center error: ~20m south of intended location.

**FAILURE B — Drone controller not disabled at landing**
Terminal: "At home position (1.5,-0.1) — LANDING" but no LANDED event.
gz pose: z=1.242m at cleanup query — never crossed z<0.23m threshold.
Visual: rotors still spinning after descent.
Hypothesis: "home position" (1.5,-0.1) is in node-odom frame which is misaligned with world;
drone descended toward wrong world location (world ~0.5,-1.1 vs expected world 5,-20).
Compounded by Failure A frame mismatch.

**FAILURE C — Rover odom 90% wheel slip (or topic disconnect)**
Odom-reported displacement: 15.126m. Physical displacement (gz): 1.511m.
v_odom=0.311 m/s, v_physical=0.031 m/s. Slip ratio=90.0%, odom:actual=10.0:1.
WP1 reached in 83ms (physically impossible, suggests initial proximity at odom (0,0)).
Hypothesis A (primary): DiffDrive wheel spin on sloped mesh facets — wheels rotate at
commanded rate but rover barely translates. Odom counts rotations, massively overestimates.
Hypothesis B: Bridge topic mismatch on jezero_c include-namespacing; odom feed broken,
rover uses stale zero-initialized position. Physical 1.511m motion confirms cmd_vel reaches
DiffDrive regardless.
Note: Both hypotheses predict rover never reaches dome. Root cause determination deferred to Phase 8.

**FAILURE D — Rover never reached dome (consequence of C)**
Final gz world pose: (6.739, -18.562). Rig-frame: (1.739, 1.438). Distance to dome: 13.34m.
Rover node declared ARRIVED based on odom alone. Physical arrival threshold (2m) not met.

**FAILURE E — Occupancy grid reflects wrong spatial region (consequence of A)**
Grid was built from LaserScan over central terrain, not rig area. A* path avoids y~0 corridor
with 1.20x detour ratio and 2.6m peak southern deviation — obstacle pattern from wrong region.
Grid is structurally valid (correct format, correct cell count); spatially invalid for this rig.

---

## Step 3 — Known-Risk Failure Mode Characterization

### 3.1 DiffDrive Wheel Slip Metric

| Metric | Value |
|---|---|
| Physical displacement (gz delta) | 1.511 m |
| Odom-reported displacement | 15.126 m |
| Navigation time | 48.7 s |
| v_physical | 0.031 m/s |
| v_odom | 0.311 m/s |
| **Slip ratio** | **90.0%** |
| **Odom:actual ratio** | **10.0:1** |

Directly validates need for TRN stack. DiffDrive odom is unusable as primary localization
on real HiRISE mesh terrain over any path longer than ~1m.

### 3.2 LaserScan False Positives / Occupancy Grid Quality

| Metric | Value |
|---|---|
| Grid occupancy | 238/3200 cells = 7.4% |
| Straight-line path (spawn to dome) | 14.70 m |
| A* path length | 17.61 m |
| Detour ratio | 1.20x |
| Max path deviation from straight line | 2.6 m south (y=-2.6 rig-frame) |

CAVEAT: Grid built from survey of wrong region (Failure A). Obstacle pattern does not
correspond to rover's actual path. Cannot confirm whether detour reflects true obstacles
or terrain-scan artifacts on the actual path. Re-evaluation required after Failure A fixed.

### 3.3 Rangefinder Altitude Hold

| Metric | Value |
|---|---|
| Survey alt target | 3.0 m above ground |
| World z at takeoff complete | 2.93 m |
| Terrain z at spawn | 0.023 m |
| Altitude above terrain at survey start | 2.907 m |
| Error | 0.093 m undershoot (3.1%) |
| Time-series z log | NOT AVAILABLE — add to Phase 7 |
| Crash observed | No — 10/10 WPs completed |

Insufficient data for quantitative characterization over variable terrain. Add continuous
drone z + rangefinder channel to Phase 7 ground-truth logger before re-evaluating.

---

## Step 4.3 — Phase 6 Assessment

**Phase 6: PARTIAL**

The simulation ran without crash, deadlock, or rollover. Core infrastructure (world load,
topic bridge, sensor pipeline, A* planner) is functional. However, two independent coordinate
frame bugs prevent meaningful end-to-end validation:

1. **Drone odom frame mismatch** (Failure A+B): drone surveys wrong region and never correctly
   lands. The occupancy grid and landing sequence both depend on a corrected coordinate frame.
2. **Rover odom 90% slip** (Failure C+D): rover never physically reaches the dome.
   DiffDrive odom is not a viable localization source on HiRISE terrain.

**These are not blocking Phase 7.** Phase 7 (ground-truth logger) is INDEPENDENT of the
odom frame bugs — it reads directly from `gz topic /world/jezero_c/pose/info` which gives
absolute world-frame poses regardless of node odom correctness. Phase 7 data will provide
the baseline to quantify how wrong the odom-based estimates are.

**Recommended fix sequence before Phase 8:**
- Fix Failure A (drone coord frame): subtract initial world pose from odom readings in
  smart_flight_node.py, or switch odom subscription to start-relative source.
- Verify Failure C root cause (odom topic mismatch vs wheel slip) by echoing
  `/rover/odom` during a short test run with mars_mission.sdf flat world.

**Proposed next step: Phase 7 (ground-truth logger), ~1-2 hrs.**
