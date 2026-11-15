# Task 3 (3A + 3B) system architecture — Team SC#4372

Status: draft for approval, 15 Nov 2026 23:30 IST. Hard deadline for 3A, 3B and 3C: 16 Nov 2026.

## 1. Intent

One set of four nodes that runs both Task 3A (one run, ×0.6 → 30 marks) and Task 3B
(two consecutive runs with noisier sensors and a map that may not match the world, averaged → 50
marks). The same code is copied under both name sets (`task3A_*.py`, `task3B_*.py`), so 3B
robustness is designed in from the start rather than retrofitted.

Success = a **perfect run that earns the bonus**: the bonus ends the run instantly, which also
stops T at its minimum. Code is marked against the e-Yantra Coding Standards and read by
plagiarism software: class-based, no dead code, no placeholder logic, honest logs.

### Goals
- Every run scores: one ore in its own container and the eBot on the ore drop pose are the
  zero-floor conditions; both must happen early and with no single point of failure.
- All 5 items detected (CD 5), all 5 dropped (SD 20), legs 1–2 followed (EN 10), bonus (3).
- Zero collisions and zero protective stops (−2.5 each).
- Low T: arm and eBot work in parallel; the arm takes the package as soon as the eBot arrives.

### Non-goals
- Reinforcement learning, MoveIt, Nav2 or any ready-made planner (rule-banned or too risky).
- Custom message packages: coordination uses `std_msgs`, so the build stays one ament_python package.
- Task 3C (content is on the portal only; handled separately once known).

### Anti-goals (would fail even if it "works")
- A topic or service call from a terminal during evaluation (scores 0).
- A path republished after the eBot leaves 0.3 m of a leg start (the judged path is the one
  before move-off; replanning mid-leg silently invalidates following).
- The eBot leaving the arm before the package is off its deck.
- A 3B node set that differs between run 1 and run 2.

## 2. Facts the design rests on

| Fact | Value | Source |
|---|---|---|
| Score | (600−T)×0.02 + CD×1 + SD×4 + EN×5 + B×3 − P×2.5, out of 50 | 3A Instructions |
| Zero floor | run = 0 unless an ore is in its own container AND the eBot reached the ore drop pose | 3A Instructions |
| Home / ore drop pose | (0,0) yaw 0 / (6.0, 1.9) yaw 0 or ±3.14 | 3A Instructions |
| Leg rules | path = last published before 0.3 m move-off; first/last pose exact; inside map; no occupied cell; stay within 0.5 m; end still within 0.3 m and ±0.15 rad | 3A Instructions |
| Legs scored | home→drop, drop→arm (EN 2×5). arm→home not scored but "eBot home" is needed for the bonus | Instructions + evaluator strings |
| Frames | `map` = `world` (identity static TF). Arm `base_link` at world (−0.5973, 1.8653, 0.8825), identity rotation | launch file, Task 2A probe |
| TF names | `<kind>_ore_<n>` (1-based per kind); `ore_package`, published while the package is on the eBot beside the arm | 3A Instructions |
| Package | 177.8 × 139.7 × 76.2 mm, 0.30 kg, held force ≈ 2.94 N | 3A Instructions, model.sdf |
| Arm limits (Task 3) | twist 0.25 m/s, 0.60 rad/s; joints 0.60 rad/s, 0.8 rad/s²; over-limit command discarded whole | 3 Overview |
| /odom | now injected by the arena plugin (eBot odometry plugin removed); evaluator compares it to sim truth | release diff, evaluator |
| Ore package service | `/spawn_ore_package` succeeds only while the eBot stands still on the drop pose with no package | launch header |

### Unknowns, resolved by recon in the simulator before coding the parts that need them
1. Crusher hopper opening: position and rim height relative to `base_link` (mesh only, no
   numbers in the SDF).
2. The eBot stop pose beside the arm: free on `/map`, reachable by the arm (deck within
   ~0.80 m of the shoulder), seen by the camera, clear of the arm's swept volume.
3. eBot top-deck height and the package's colours as the RealSense sees them.
4. 3B noise magnitudes on camera depth, force, `/odom` and `/scan`, and how far rocks move off
   the map.
5. Planner leg times on a few rock maps (sets the expected T).

## 3. Approaches considered

| | Approach | Trade-off |
|---|---|---|
| **A (chosen)** | Four required nodes; the handover is two latched `std_msgs/String` topics between `task3X_manipulation` and `task3X_planning` | Fewest processes and files; each node keeps one job; no extra file to submit |
| B | Add a fifth coordinator node `task3X_handover.py` that owns the mission | Cleaner single place for the mission, but one more process that must start, one more failure point, one more file under review; the coordinator still needs both robots' state, so it duplicates subscriptions |
| C | One process per robot (merge perception+manipulation, planning+navigation) | Not allowed: four named scripts are required |

A is chosen because the handover is a two-message exchange; a coordinator node adds more
surface than it removes.

## 4. Architecture

```
            /camera/* ───────────────┐
                                     ▼
   ┌───────────────────── task3X_perception ─────────────────────┐
   │ OreDetector + PackageDetector → DepthProjector → ItemTracker │
   │ TF: base_link → <kind>_ore_<n>, base_link → ore_package     │
   └──────────────────────────────────────────────────────────────┘
                 │ TF                        ▲ /handover/ebot (AT_ARM gates package search)
                 ▼                           │
   ┌──────────────────── task3X_manipulation ────────────────────┐
   │ ArmMission (job queue) · CartesianServo · RouteFollower      │
   │ GripMonitor · RecoveryManager · DropTargets(containers,hopper)│
   └──────────────────────────────────────────────────────────────┘
        │ /handover/arm  (PACKAGE_CLEAR)        ▲ /handover/ebot (AT_ARM)
        ▼                                       │
   ┌───────────────────── task3X_planning ───────────────────────┐
   │ EbotMission (legs, package request) · PlanningGrid (map ⊕    │
   │ LiDAR rocks) · ClearanceField · AStarSearch · PathSmoother   │
   │ publishes /ebot_path per leg, /handover/ebot                 │
   └──────────────────────────────────────────────────────────────┘
        │ /ebot_path, /ebot_leg (leg id)       ▲ /ebot_leg_status (ARRIVED)
        ▼                                       │
   ┌───────────────────── task3X_navigation ─────────────────────┐
   │ PoseFilter(/odom) · PathFollower TURN/TRACK/ALIGN · ObstacleGuard(/scan, ultrasonic) │
   │ publishes /cmd_vel                                           │
   └──────────────────────────────────────────────────────────────┘
```

### 4.1 Handover protocol (own topics, all `std_msgs/String`, QoS reliable + TRANSIENT_LOCAL depth 1)

| Topic | Publisher | Values | Meaning |
|---|---|---|---|
| `/handover/ebot` | planning | `EN_ROUTE`, `AT_ARM`, `LEAVING` | `AT_ARM` only once the eBot stands still on the stop pose with the package on board |
| `/handover/arm` | manipulation | `IDLE`, `TAKING`, `PACKAGE_CLEAR` | `PACKAGE_CLEAR` only once the magnet holds ≈2.94 N and the tool has lifted ≥ 0.10 m above the deck |
| `/ebot_leg` | planning → navigation | `HOME_TO_DROP`, `DROP_TO_ARM`, `ARM_TO_HOME` | which path is being followed |
| `/ebot_leg_status` | navigation → planning | `ARRIVED <leg>`, `BLOCKED <leg>` | arrival = within tolerance and still for 0.5 s |

Latched topics make the protocol order-independent: a node that restarts or starts late reads
the current state at once. Each message is a single uppercase token from an `Enum`, parsed in
one place per node; unknown tokens are logged and ignored.

### 4.2 Arm mission (manipulation)

Job queue, re-planned after every completed job:
1. Ores in the order Task 2A uses (untried first, then by reach), **except** that when
   `/handover/ebot = AT_ARM` the package job jumps to the head of the queue at the next job
   boundary (never aborting a pick in progress, never carrying an ore over the eBot).
2. Package job: hover over `ore_package` TF, two-stage descent, touch on force (threshold
   re-tuned for the package's spring response), hold check 2.4–3.5 N, lift ≥ 0.10 m, publish
   `PACKAGE_CLEAR`, transit to the hopper, release above the opening, verify the force falls
   to free.
3. Remaining ores.

Reused from Task 2A unchanged in behaviour: JointUnfolder, controller switch, CartesianServo,
RouteFollower, ServiceCaller, protective-stop recovery, moved-ore guard. Changes: speed caps
raised within Task 3 limits with margin (linear 0.22 m/s, angular 0.50 rad/s, joint 0.50 rad/s),
`DropTargets` gains the hopper, routes gain a keep-out volume around the parked eBot.

### 4.3 Perception

Reused: per-kind HSV ore detection, dark-ring test, depth projection, tracker with fixed
names. Added: `PackageDetector` (colour/shape signature measured in recon; size gate from the
known 177.8 × 139.7 mm footprint), active only while `/handover/ebot = AT_ARM` and until the
package is lifted; `ore_package` TF is the tracked median of the package top-face centre
minus half its height, matching the ore convention. 3B: longer median window (15 → 25
sightings), depth patch 9×9 with outlier rejection (median absolute deviation), HSV ranges
re-checked on noisy frames.

### 4.4 eBot mission (planning)

Sequence: plan leg 1 → wait for navigation `ARRIVED` → stand still → call `/spawn_ore_package`
(retry while it reports not ready, bounded) → plan leg 2 to the stop pose → `AT_ARM` →
wait for `PACKAGE_CLEAR` → `LEAVING` → plan leg 3 home.

Planning per leg, all before move-off:
- `PlanningGrid` = `/map` occupancy ⊕ rocks seen by the current LiDAR scan that the map does
  not explain (3B), inflated by the eBot radius.
- `ClearanceField`: one distance transform over the grid. A* cost = length + penalty that
  grows as clearance falls, so paths keep to the middle of free space. This replaces the fixed
  clearance-radius ladder from 2B with one smooth cost and is what absorbs rocks that sit
  slightly off the map.
- The published path is validated with the evaluator's own rules (exact start/end poses,
  inside map, no segment through an occupied `/map` cell) before it is published; a failed
  check re-plans with a smaller clearance weight rather than publishing a bad path.

### 4.5 Navigation

Reused from 2B: FollowPhase TURN/TRACK/ALIGN/DONE, corridor checks, turn-first logic. Added:
- `PoseFilter`: smooths `/odom` (3B noise) for control; arrival is declared on the filtered
  pose with tolerances well inside the judged 0.3 m / 0.15 rad.
- `ObstacleGuard` with LiDAR and the rear ultrasonics: slows, then stops before contact; may
  shift the tracking target sideways by at most 0.25 m (half the 0.5 m corridor) to pass a rock
  the map did not show; never republishes the path mid-leg.

## 5. Failure handling (edge cases)

| Case | Handling |
|---|---|
| Package service refuses (not still, already has a package) | stand still 1 s, retry up to 5 times, then log and keep retrying at 2 s; never leave the drop pose without the package |
| Package not seen at the arm | perception keeps searching; manipulation keeps sorting ores meanwhile; eBot waits (the rule) |
| Package slips off during lift | no `PACKAGE_CLEAR` is ever sent until the hold check passes; retry the pick from the new TF |
| eBot blocked by an unmapped rock | guard stops it; sidestep within corridor; if still blocked, report `BLOCKED`, wait, then creep |
| Protective stop | Task 2A recovery (release, unlock, lift); ≤ 3 recoveries |
| A node restarts | latched handover topics give it the current state |
| 3B second run | nodes are stateless across runs; each start reads only the fresh world |

## 6. Delivery plan (time-boxed to the deadline)

1. Merge `v3.0.0` into the workspace (`git stash -u` / `merge tags/v3.0.0` / `stash pop`), build.
2. Recon run in the 3A world: hopper position, stop pose, deck height, package look, leg times.
3. Copy Task 2 nodes to `task3a/`, implement the changes above, offline checks.
4. 3A practice runs until two clean runs in a row → recorded 3A evaluation → package.
5. Same code as `task3b/` → 3B practice in the 3B world → recorded two-run evaluation → package.
6. Push to the team repo; zips and videos into `~/eyantra/task3a|task3b/`.

## 7. Left to the builder

Exact thresholds (touch force for the package, hold windows, guard distances, clearance
weight) come from recon and practice runs; each is a named constant with its unit, and the
log of each run records the values it used.
