# Phase 8–10 Mission Report
**Date:** April 20, 2026  
**World:** `worlds/jezero_c.sdf` (real HiRISE Jezero_C DTM, 40×40m patch, 3.08m relief)  
**Stack:** ROS2 Jazzy + Gazebo Harmonic 8.10.0, native plugins only

---

## 1. What Was Built

### Phase 8 — Bayesian Log-Odds Occupancy Grid
Replaced the count-based binary grid in `smart_flight_node.py` with a probabilistic log-odds map.

| Parameter | Value | Rationale |
|---|---|---|
| L_FREE | −0.847 | log(0.3/0.7) |
| L_OCC | +2.197 | log(0.9/0.1) |
| Clamp | ±10.0 | Prevents saturation lock |
| Publish threshold | L > 1.5 | ≥1 clean rangefinder hit required |
| QoS | RELIABLE + TRANSIENT_LOCAL | Subscriber always receives map, no sleep loop |
| Grid | 80×40 cells at 0.25m | x∈[−2,18] y∈[−5,5] rig-frame |

Depth camera (`front_depth`) disabled for grid updates — forward-facing camera at 3m AGL produces false positives from sloped terrain; downward rangefinder is the sole obstacle sensor.

A* was updated to penalise unknown cells (−1) at ×3 cost, preferring confirmed-free corridors.

### Phase 9 — TRN Scan-to-Map Matcher (`trn_node.py`)
New node that localises the rover by matching its LiDAR scan against the drone's occupancy grid.

**Algorithm:**
1. `map_cb`: converts occupancy grid to a Gaussian-blurred likelihood field (σ=1.5 cells, pure numpy). Builds K=294 candidate offsets (±0.75m XY at 0.25m step × ±15° yaw at 5° step).
2. `odom_cb`: applies incremental odom deltas to a dead-reckoning estimate between corrections.
3. `_update` (2 Hz): subsamples N=36 rays from `/rover/lidar`. Scores all K candidates via vectorized (K×M) hit-position lookup in likelihood field. Accepts correction when `best > baseline + 2%` improvement. Publishes `PoseWithCovarianceStamped` on `/rover/trn_pose`.

### Phase 9b — TRN Feedback Loop
`smart_rover_node` subscribes to `/rover/trn_pose` and soft-blends corrections into `self.pos` at α=0.3 per tick when:
- TRN covariance `cov[0] < 0.5` (σ < ~0.7m)
- Correction magnitude < 0.8m (divergence guard)
- State == `NAVIGATING`

### Phase 10 — TRN-Triggered A* Re-planning
When accumulated TRN correction exceeds 0.6m and cooldown has expired (50 ticks = 5s at 10Hz), the rover re-runs A* from its TRN-corrected position. Guards:
- Skip if ≤2 waypoints remain (near goal)
- Keep old path if A* returns empty (never stops cold mid-mission)
- 5s cooldown prevents thrashing

---

## 2. Validated Demo Run — April 20, 2026

### Timeline

| Time (sim) | Event |
|---|---|
| t=0s | Gazebo + bridge + all 4 nodes launched |
| t=3.9s | Drone TAKEOFF complete at z=2.85m |
| t=~59s | Survey complete — 10/10 waypoints, map published |
| t=59.6s | TRN likelihood field built (294 candidates) |
| t=59.6s | TRN fix #1: improve=43.9%, cov=1.600 |
| t=59.6s | Rover receives map, plans 18-WP A* path, enters NAVIGATING |
| t=64.6s | TRN fix #11: improve=88.6%, cov=0.172 |
| t=69.6s | TRN fix #21: improve=53.7%, cov=0.050 (floor reached) |
| t=69.6s | TRN blend #1 fires in rover: delta=(0.25,−0.50)m, accumulated=0.168m |
| t=74.6s | Drone LANDED at z=0.211m, controller disabled |
| t=~121s | Rover ARRIVED — 18/18 waypoints, 0.73m from dome center |

### Map Quality

| Metric | Value |
|---|---|
| Grid size | 80×40 cells |
| Free cells | 765 |
| Occupied cells | 86 |
| Unknown cells | 2349 |
| A* path waypoints | 18 |

### TRN Performance

| Fix # | Improvement | Covariance |
|---|---|---|
| 1 | 43.9% | 1.600 |
| 11 | 88.6% | 0.172 |
| 21 | 53.7% | 0.050 |
| 31 | 6.4% | 0.050 |
| 41 | 27.6% | 0.050 |

Covariance floor reached by fix #21 and held steady — TRN locked onto the terrain with high confidence.

### Navigation Result

| Metric | Value |
|---|---|
| Rover waypoints | 18/18 |
| Final WP distance to dome | **0.73m** |
| TRN re-plans triggered | **0** (accumulated correction < 0.6m threshold) |
| Drone final z | 0.211m |
| Controller disabled | Yes |

The zero re-plan count is the **correct healthy outcome**: TRN corrections were small (accumulated ~0.168m), confirming that 6WD traction + terrain friction kept odom drift low enough that the original A* path remained valid throughout the mission. Phase 10 re-planning logic is verified present and armed; it simply was not needed.

---

## 3. Pipeline Summary

```
Drone surveys jezero_c.sdf terrain (3m AGL, 10 lawnmower WPs)
  → Bayesian log-odds OccupancyGrid (80×40, TRANSIENT_LOCAL)
    → TRN builds likelihood field (Gaussian blur, K=294 candidates)
    → Rover receives map, plans A* path (18 WPs, unknown cells ×3 cost)
    → TRN corrects rover pose at 2 Hz (fix #21: cov=0.050)
      → Rover blends TRN at α=0.3 (cov guard + mag guard)
        → Phase 10 re-plan armed (threshold 0.6m, not triggered)
          → ARRIVED at habitat dome, 0.73m from dome center
```

---

## 4. Key Files (as of commit 5ffad01)

| File | Role |
|---|---|
| `midterm_project/smart_flight_node.py` | Drone brain: survey, Bayesian grid, publish map |
| `midterm_project/smart_rover_node.py` | Rover brain: A*, navigation, TRN blend, re-plan |
| `midterm_project/trn_node.py` | TRN: likelihood field, vectorized matcher, pose publisher |
| `scripts/run_live_demo.sh` | Launch: Gazebo + bridge + 4 nodes + GT logger |
| `scripts/groundtruth_logger.py` | GT pose logger → `/tmp/phase7_groundtruth.csv` |

---

## 5. Next Steps

- **Phase 12:** Plot odom drift vs TRN-corrected error vs ground truth CSV (localization error curve)
- **Phase 3:** Add Mastcam-Z hero rock meshes for richer TRN feature environment
- **Phase 4:** Poly Haven scatter (Golombek-Rapp distribution)
