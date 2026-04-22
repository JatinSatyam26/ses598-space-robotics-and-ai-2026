# Mars Scout Demo — Recording Checklist

## Before you start
- [ ] Close any prior Gazebo / ROS processes: `pkill -9 -f "gz sim"; pkill -9 -f "parameter_bridge"`
- [ ] Start your screen recorder (OBS, Kazam, etc.) — record the full desktop
- [ ] Open two terminals side by side

---

## Terminal 1 — launch the demo

```bash
cd ~/ros2_ws/src/midterm_project
source ~/ros2_ws/install/setup.bash
WORLD_SDF=worlds/jezero_c.sdf bash scripts/run_live_demo.sh
```

> **Critical:** without `WORLD_SDF=worlds/jezero_c.sdf` the script loads the flat
> legacy world silently. Always use this prefix.

## Terminal 2 — watch mission logs (open after ~15 s)

```bash
tail -f /tmp/smart_flight.log /tmp/smart_rover.log /tmp/trn.log
```

---

## What you should see in Gazebo — in order

| Time | Gazebo visual | Log confirmation |
|---|---|---|
| 0–10 s | Mars terrain, drone on pad, rover behind it, dome far right | — |
| ~10 s | Drone lifts off | `smart_flight.log: TAKEOFF complete` |
| 10–70 s | Drone flies lawnmower grid at ~3 m AGL | `Reached waypoint 1 … 10` |
| ~70 s | Drone descends back to pad, rotors spin down | `LANDING … z=0.2` |
| ~70 s | Rover begins moving toward dome | `smart_rover.log: NAVIGATING` |
| ~70 s | TRN covariance drops 1.6 → 0.05 | `trn.log: fix #21 cov=0.050` |
| ~130 s | Rover stops near dome | `ARRIVED at habitat dome … 0.73m` |
| ~135 s | Terminal 1 prints MISSION COMPLETE | Script banner |

---

## Successful run criteria

- Drone completes 10/10 survey waypoints without crashing
- `smart_flight.log` reports 86–88 occupied cells
- Rover plans 18 waypoints and reaches all 18
- Final log line: `ARRIVED at habitat dome` with distance < 2.0 m
- `trn.log` shows covariance reaching 0.050 by fix #21

---

## If something goes wrong

| Symptom | Fix |
|---|---|
| Drone stuck hovering | Bridge started before Gazebo loaded. Ctrl+C, rerun. |
| Rover not moving | Check `smart_rover.log` for "Received occupancy grid". If missing, TRANSIENT_LOCAL handshake failed. Ctrl+C, rerun. |
| RTF below 50% | `pgrep -a gz` — a prior Gazebo is still running. Kill it, then rerun. |

---

## After a successful run — generate the Phase 12 error plot

```bash
python3 scripts/phase12_analysis.py /tmp/phase12_poses.csv
# Plot saved to: /tmp/phase12_error.png
```

Include `/tmp/phase12_error.png` in your submission alongside the report.
