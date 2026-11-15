# Task 3A/3B Implementation Plan

> **For agentic workers:** executed natively in the session that wrote it (user instruction: "do the work now"). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Four nodes that score a perfect Task 3A run and two scoring Task 3B runs.

**Architecture:** Task 2A arm nodes and Task 2B eBot nodes, extended with a package job, a hopper drop and a two-topic handover (`/handover/ebot`, `/handover/arm`). One code set, copied under the 3A and 3B names.

**Tech Stack:** ROS 2 Jazzy, rclpy, OpenCV, NumPy, SciPy, Gazebo Harmonic (in distrobox `eyrc`).

**Spec:** `docs/superpowers/specs/2026-11-15-task3-architecture-design.md`

## Global Constraints

- Script names: `task3A_perception.py`, `task3A_manipulation.py`, `task3A_planning.py`, `task3A_navigation.py`; 3B the same with `task3B_`. All in `algorithms/setup.py` SCRIPTS.
- No MoveIt, Nav2 or any ready-made planner; no terminal topic/service calls during evaluation.
- Arm caps (Task 3): twist ≤ 0.25 m/s and 0.60 rad/s, joints ≤ 0.60 rad/s; a command over a cap is discarded whole. Use 0.22 / 0.50 / 0.50.
- Leg path: first/last pose exact, inside `/map`, no occupied cell; follow within 0.5 m; end still within 0.3 m and ±0.15 rad.
- TF names `<kind>_ore_<n>` and `ore_package`, parent `base_link`.
- Coding Standards header block, class-based, no dead code, no placeholder logic.

## Recon results (measured 15 Nov, base_link unless stated)

| Item | Value |
|---|---|
| Hopper rim | x[−1.251, −0.648], y[−0.31, +0.30], z +0.207; funnel floor z +0.04 |
| Arm stop pose for the eBot | world (0.0, 1.9), yaw 0 or π (Task 2B arm pose; free on every map seen) |
| Package top centre with the eBot there | (0.671, 0.048, 0.256) |
| Package top colour | HSV H 154, S 255, V 246; black rim; the only magenta in view |
| Package hold force | ≈ 2.94 N |

## Review Focus

1. Arm reaches for the package while the eBot is still rolling in → package job only starts on `AT_ARM`, which the planner sends only after 1 s standing still on the stop pose.
2. Package TF stale after the package is lifted → perception stops publishing `ore_package` once `/handover/arm` reads `PACKAGE_CLEAR`.
3. Hopper drop out of reach → release point chosen from a reach test in the sim, inside the rim by ≥ 0.05 m.
4. Late-starting node misses a handover message → TRANSIENT_LOCAL on both handover topics.
5. 3B unmapped rock on a leg → LiDAR rocks added to the planning grid before each path is published; guard stops before contact.

---

### Task 1: Scaffold the four 3A scripts and register them
**Files:** Create `algorithms/scripts/task3a/task3A_{perception,manipulation,planning,navigation}.py` (copied from Task 2); Modify `algorithms/setup.py` SCRIPTS.
- [ ] Copy, fix header (Filename, Classes, topics), chmod +x, add to SCRIPTS, `colcon build --packages-select algorithms`, `ros2 pkg executables algorithms` lists all four.

### Task 2: Handover protocol
**Interfaces (produces):** topics `/handover/ebot` and `/handover/arm`, `std_msgs/String`, QoS reliable, TRANSIENT_LOCAL, depth 1. Tokens: ebot `EN_ROUTE | AT_ARM | LEAVING`; arm `IDLE | TAKING | PACKAGE_CLEAR`. Each node defines its tokens as an `Enum` whose values are the strings.
- [ ] Planning: publishes `EN_ROUTE` at start, `AT_ARM` after leg 2 finishes, waits for `PACKAGE_CLEAR` before planning leg 3, then `LEAVING`.
- [ ] Manipulation: publishes `IDLE` at start, `TAKING` when the package job starts, `PACKAGE_CLEAR` after the hold check passes with the tool ≥ 0.10 m above the deck.
- [ ] Verify with `ros2 topic echo` (reading only) during a practice run.

### Task 3: Package detection (perception)
**Interfaces:** consumes `/handover/ebot`, `/handover/arm`; produces TF `base_link → ore_package` (top-face centre minus 0.0381 m).
- [ ] `PackageDetector.detect(image_bgr) -> list[Detection]`: magenta HSV (145,150,150)–(163,255,255), area 150–4000 px, dark-ring test as for ores.
- [ ] Offline check on the saved frame `arm_2.npz`: one detection, centre within 0.02 m of (0.671, 0.048, 0.218).
- [ ] Tracker: established after 10 sightings, published while `AT_ARM` and not `PACKAGE_CLEAR`.

### Task 4: Package job and hopper drop (manipulation)
**Interfaces:** consumes TF `ore_package`, `/handover/ebot`; produces `/handover/arm`.
- [ ] Constants: `package_height_m = 0.0762`, `package_holding_force_n = (2.4, 3.6)`, `hopper_drop_point = (x, y, z)` from the reach test, `package_travel_height_m` clearing the rim (0.207) by ≥ 0.08 m with the package hanging below.
- [ ] Job selection: at each `SELECT`, the package goes first once `AT_ARM` and its TF exist.
- [ ] Route to the hopper across the left (+y, rover side) at package travel height, never across the conveyor.
- [ ] Speed caps raised to 0.22 m/s / 0.50 rad/s / 0.50 rad/s.
- [ ] Sim check: package lifted, `PACKAGE_CLEAR` sent, package rests in the hopper, `arm_status` stays 0.

### Task 5: eBot changes (planning, navigation)
- [ ] Planning: wait at the arm for `PACKAGE_CLEAR` (no timeout that drives off with the package); LiDAR points not explained by `/map` added as rocks before each plan; path checked against `/map` rules before publishing.
- [ ] Navigation: `/odom` smoothing for control (3B noise); obstacle guard slows and stops before contact.
- [ ] Sim check: legs 1–3, package request, stop pose, home.

### Task 6: Full 3A practice runs
- [ ] Headless runs with the evaluator until two clean runs in a row (score, T, penalties logged).

### Task 7: 3B
- [ ] Copy to `task3b/` with `task3B_` names, register, build.
- [ ] Measure 3B noise; tune; headless two-run evaluation until both runs score.

### Task 8: Hand-over
- [ ] Recorded evaluations by the user (screen recording), zips `SC#4372_task3A.zip`, `SC#4372_task3B.zip`, push to `team`.
