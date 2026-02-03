# First-Order Boustrophedon Navigator - Project Report

**Student:** Jatin Satyam  
**Course:** SES598 - Space Robotics and AI  
**Date:** February 2, 2026

## Executive Summary

This project successfully implemented a precise boustrophedon (lawnmower) pattern controller for a first-order dynamical system using ROS2 and Turtlesim. Through systematic analysis and controller architecture redesign, the final implementation achieved an average cross-track error of **0.027 units**, significantly exceeding the target requirement of < 0.2 units.

**Key Achievement:** 96.5% reduction in cross-track error through control structure improvements rather than simple parameter tuning.

---

## Table of Contents
1. [Problem Statement](#problem-statement)
2. [Initial Analysis](#initial-analysis)
3. [Methodology](#methodology)
4. [Implementation Details](#implementation-details)
5. [Results and Performance](#results-and-performance)
6. [Challenges and Solutions](#challenges-and-solutions)
7. [Conclusion](#conclusion)
8. [References](#references)

---

## Problem Statement

### Objective
Modify and tune a ROS2 + Turtlesim controller to execute a precise boustrophedon (lawnmower) coverage pattern with minimal cross-track error.

### Requirements
- Average cross-track error < 0.2 units (25 points)
- Maximum cross-track error < 0.5 units (15 points)
- Smooth motion (10 points)
- Clean cornering behavior (10 points)
- Even spacing and complete coverage (20 points)

### Initial State
The provided controller implemented basic PD control with go-to-point tracking, resulting in:
- Large curved arcs between waypoints
- Diagonal transitions at row edges
- Average cross-track error ≈ 0.76 units
- Poor pattern geometry

---

## Initial Analysis

### Baseline Performance (Before Modifications)

**Quantitative Metrics:**
- Average cross-track error: **0.760 units**
- Maximum cross-track error: **~2.0 units**
- Pattern type: Curved spiral-like trajectory

**Observed Behaviors:**
1. **Curved Row Traversal:** The turtle followed curved paths instead of straight lines
2. **Large Looping Turns:** Transitions between rows exhibited wide, inefficient arcs
3. **Continuous Angular Correction:** Angular velocity never reached zero during straight segments
4. **Coupled Motion:** Linear and angular velocities were always active simultaneously

**Root Cause Identification:**
The controller implemented **go-to-point tracking** rather than **path following**. The turtle continuously adjusted heading while moving forward, chasing target points rather than following line segments.

---

## Methodology

### Diagnostic Approach

1. **Environment Setup Verification**
   - Confirmed ROS2 Jazzy on Ubuntu 24.04
   - Fixed import issue: `turtlesim.msg.Pose` → `turtlesim_msgs.msg.Pose`
   - Established rebuild workflow: `colcon build` + `source install/setup.bash`

2. **Controller Analysis**
   - Reviewed existing PD control implementation
   - Identified lack of state separation (no TURN vs DRIVE modes)
   - Recognized waypoint structure issues

3. **Problem Classification**
   - **Not a tuning problem:** Recognized that PD gain adjustment alone couldn't fix geometric issues
   - **Control structure problem:** Required behavioral separation and proper waypoint sequencing

### Solution Strategy

**Phase 1: Stability Improvements**
- Added divide-by-zero guards for cross-track error calculation
- Implemented dt floor (minimum 1e-3) to prevent derivative spikes
- Added angular velocity clamping to reduce oscillation
- Reset derivative states on mode transitions

**Phase 2: State Machine Implementation**
- Introduced TURN and DRIVE modes
- Set linear velocity to zero during TURN mode
- Computed turn target angles using `atan2`

**Phase 3: Waypoint Structure Correction**
- Original waypoints forced diagonal transitions: `(9, 8.0) → (2, 7.5)`
- Corrected to orthogonal structure: `(9, 8.0) → (9, 7.5) → (2, 7.5)`
- Ensured horizontal rows with vertical step-downs at edges

**Phase 4: Critical Architecture Fix**
- **Problem:** TURN mode only triggered for vertical transitions (Y-coordinate changes)
- **Solution:** Modified logic to enforce TURN mode at the start of EVERY segment
- **Implementation:** Removed conditional check, always compute turn angle and enter TURN mode before driving

---

## Implementation Details

### Final Controller Architecture

**State Machine:**
```
TURN Mode:
- Linear velocity = 0 (rotate in place)
- Angular velocity = Kp_angular × angle_error (clamped to ±2.0)
- Exit condition: |angle_error| < 0.05 radians
- Purpose: Align with next segment before driving

DRIVE Mode:
- PD control for both linear and angular velocity
- Linear: Kp × distance + Kd × distance_derivative
- Angular: Kp × angle_error + Kd × angle_error_derivative
- Purpose: Follow straight line to next waypoint
```

### Final Parameter Values
```python
# PD Controller Gains
Kp_linear = 1.0    # Proportional gain for forward motion
Kd_linear = 0.1    # Derivative gain for forward motion
Kp_angular = 3.0   # Proportional gain for rotation (increased from 1.0)
Kd_angular = 0.1   # Derivative gain for rotation

# Pattern Parameters
spacing = 0.5      # Distance between parallel rows (units)
```

**Parameter Justification:**
- **Kp_linear = 1.0:** Provides adequate forward acceleration without overshooting
- **Kd_linear = 0.1:** Minimal damping sufficient for first-order system
- **Kp_angular = 3.0:** Increased to enable faster alignment during TURN mode
- **Kd_angular = 0.1:** Prevents rotational oscillation
- **spacing = 0.5:** Balances coverage density with efficiency

---

## Results and Performance

### Final Performance Metrics

**Quantitative Results:**
- **Average cross-track error: 0.027 units** ✅ (Target: < 0.2)
- **Maximum cross-track error: ~0.10 units** ✅ (Target: < 0.5)
- **Pattern completion time: ~2.5 minutes**
- **Waypoints successfully tracked: 22/22**

**Performance Improvement:**
- Cross-track error reduction: **96.5%** (0.760 → 0.027)
- Maximum error reduction: **95%** (2.0 → 0.10)

### Comparison Table

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Avg Cross-Track Error | 0.760 | 0.027 | 96.5% |
| Max Cross-Track Error | ~2.0 | ~0.10 | 95.0% |
| Pattern Geometry | Curved spiral | Straight lines | Qualitative |
| Motion Coupling | Always coupled | Separated phases | Qualitative |

### Visual Evidence

Performance plots are saved in `~/ros2_ws/`:
- `cross_track_error.png` - Error over time (before: peaks at 2.0, after: peaks at 0.10)
- `trajectory.png` - Path visualization (before: curved spiral, after: straight lines)
- `velocity_profiles.png` - Motion analysis (before: coupled, after: separated phases)

---

## Challenges and Solutions

### Challenge 1: Import Errors
**Problem:** `turtlesim.msg.Pose` not found in ROS2 Jazzy  
**Solution:** Updated to `turtlesim_msgs.msg.Pose`  
**Lesson:** ROS2 distros have different message package structures

### Challenge 2: Understanding the Root Cause
**Problem:** Initial assumption that tuning PD gains would fix the pattern  
**Investigation:** 
- Tried increasing Kp_angular from 1.0 to 3.0
- Error barely changed (0.760 → 0.761)
- Realized geometry was fundamentally wrong

**Solution:** Stepped back to analyze the control architecture  
**Lesson:** "Tuning can't fix incorrect geometry" - must address structural issues first

### Challenge 3: TURN Mode Not Activating for Horizontal Segments
**Problem:** Even with TURN mode implemented, error remained high (~0.76)  
**Root Cause:** TURN mode only triggered when Y-coordinate changed (vertical transitions)  
**Solution:** Removed the conditional - enforce TURN mode for ALL waypoint transitions  
**Result:** Immediate dramatic improvement (0.76 → 0.027)  
**Lesson:** Every segment requires proper alignment, not just vertical transitions

### Challenge 4: Rebuild Workflow
**Problem:** Code changes not taking effect  
**Solution:** Established consistent workflow:
```bash
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```
**Lesson:** Always rebuild and re-source after code modifications

---

## Key Insights

### 1. Control Structure vs Parameter Tuning
**The fundamental lesson:** This assignment demonstrated that control problems must be diagnosed correctly before attempting solutions. Parameter tuning alone cannot compensate for architectural flaws.

**Evidence:**
- Increasing Kp_angular had negligible effect when go-to-point logic remained
- Enforcing turn-first behavior achieved 96.5% improvement with same base parameters

### 2. First-Order System Characteristics
Turtlesim's first-order dynamics (instantaneous velocity response) simplified some aspects but highlighted control logic importance:
- No momentum/inertia effects
- Direct velocity control
- Made geometric issues more visible (no natural damping)

### 3. State Machine Design
Proper state separation (TURN vs DRIVE) is essential for precise trajectory following:
- **TURN:** Zero linear velocity ensures pure rotation
- **DRIVE:** Minimal angular velocity allows straight-line motion
- **Transition:** Hard stop + reset prevents velocity coupling

### 4. Waypoint Topology
Path planning and control are interdependent:
- Diagonal waypoint segments force curved motion
- Orthogonal waypoints enable straight-line following
- Corner waypoints provide natural turn locations

---

## Conclusion

This project successfully demonstrated the importance of proper control architecture design in achieving precise trajectory tracking. The key finding—that a 96.5% error reduction was achieved through structural improvements rather than parameter tuning—reinforces fundamental principles in robotics and control systems engineering.

**Final Performance:**
- ✅ Average cross-track error: 0.027 units (Target: < 0.2)
- ✅ Maximum cross-track error: 0.10 units (Target: < 0.5)
- ✅ Smooth, efficient motion with clean cornering
- ✅ Uniform coverage pattern with proper spacing

**Key Takeaway:** "This was not a tuning problem. It was a control-structure problem." Understanding the nature of control challenges is essential before attempting solutions.

---

## How to Run

### Build and Launch
```bash
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```

### Real-Time Parameter Tuning
In a separate terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Monitor Performance
```bash
ros2 topic echo /cross_track_error
```

---

## References

1. ROS2 Documentation - Jazzy Distribution: https://docs.ros.org/en/jazzy/
2. Turtlesim Package: http://wiki.ros.org/turtlesim
3. PD Control Theory and Applications
4. Boustrophedon Path Planning for Coverage Tasks
5. Course Materials: SES598 Space Robotics and AI, Arizona State University

---

## Acknowledgments

**Course:** SES598 Space Robotics and AI  
**Tools:** ROS2 Jazzy, Turtlesim, Python 3, Matplotlib

---

**End of Report**
