# Task 3C viva — study guide, Team SC#4372 (Strata-Cobot)

Prepared 16 Nov 2026. Read it as a team, then run the drill in section 7 at least twice.

---

## 1. What the viva is (from the portal page)

- 5 questions "about your own work". Every team gets different questions.
- One recording, max 10 minutes, made on the portal page (it stamps each question, a
  clock and a watermark into the video). 15 minutes from Start to finish recording.
  Then 3 hours to upload to YouTube (Unlisted) and paste the link.
- One attempt only. Last time to press Start: **16 Nov 2026, 11:59 PM**.
- All members in one frame (or one Google Meet with cameras on). **Every member answers
  at least one question.**
- 20 marks.

**What this tells us about the questions.** "About your own work" means the questions come
from *your submitted tasks and code*: why you chose an approach, how a part works,
what happens in a failure case, what you would change. They are not general robotics
trivia. The practice page shows sample questions; **open it first and note the style**
(length, whether they name a task, whether they ask "why" or "how").

**Time budget.** 10 minutes ÷ 5 questions = **about 1 min 45 s per answer**. Aim for
60–90 seconds. Long answers cost the last question.

**Research note.** We searched GitHub, Reddit, YouTube and participant blogs. No eYRC
viva questions are published: e-Yantra forbids sharing theme material, and blogs only
describe the tasks. The question bank below is therefore built from what evaluators of
code-based robotics vivas ask, applied to *our* code.

---

## 2. How to answer every question: the 4-step shape

Say the answer in this order. It fits 60–90 seconds and shows understanding.

1. **What** — one sentence: what the part does.
2. **How** — the method, in plain words (2–3 sentences).
3. **Why this way** — the decision and what you rejected.
4. **Proof** — one number or result: "it delivered all 4 ores in 4.5 minutes with no
   protective stop", "50 out of 50".

If you do not know: say what you *do* know, then "we would check X to confirm". Never
invent.

Example: *"How does your eBot avoid rocks?"*
> "The planner reads the map as a grid and runs A* on it. Before searching, we grow every
> rock by the eBot's half-width plus a margin, so the path keeps the whole body clear,
> not just the centre. We also add a cost near rocks so the path stays in the middle of
> free space. In 2B this scored 50/50 with zero collisions."

---

## 3. Our work in one minute (learn this pitch)

> "Strata-Cobot has two robots: a UR7e arm with a magnetic tool and a RealSense camera,
> and an eBot rover with a LiDAR. In Task 1 we learned each piece: colour detection of
> ores, moving the arm through waypoints, and the eBot following a path. In Task 2A the
> arm detected four ores with the camera, published their positions as TF frames, and
> sorted them into containers with the magnet — 50/50. In Task 2B the eBot planned its
> own path with A* on the map and drove it, fetching the ore package — 50/50. In Task 3
> both robots work together: the eBot brings the package to the arm, the two nodes
> coordinate the handover over our own topics, the arm drops it in the crusher and the
> eBot goes home. Task 3B adds sensor noise and a map that does not match the world, so we
> made every decision robust to noise and added rocks the LiDAR sees to the planning map."

---

## 4. Task-by-task facts to know

| Task | What we built | Key facts |
|---|---|---|
| 0 | Setup | ROS 2 Jazzy + Gazebo Harmonic in an Ubuntu 24.04 distrobox (laptop runs 26.04) |
| 1A | Ore detection | OpenCV HSV colour masks per ore kind, contours, centroid, depth → 3D, TF broadcast |
| 1B | Arm waypoints (40/40) | Servo through `/delta_twist_cmds`, proportional control on tool position, hold each waypoint ≥ 2 s; caps 0.15 m/s, 0.35 rad/s (we used 0.12 for margin) |
| 1C | eBot path following (40/40) | Pure pursuit with lookahead, turn on the spot when facing far off the path, reach radius inside the 0.5 m scoring distance |
| 2A | Ore sorting (50/50) | Perception: HSV + "dark ring" test (ore sides are black) + 9×9 depth median + tracker (15 sightings, names fixed once 4 are stable). Manipulation: 16-state machine, unfold in joint space first, Cartesian servo, two-stage descent, touch by force > 0.8 N, hold check 0.9–2.2 N (ore ≈ 1.47 N), transit on an arc in front of the base, protective-stop recovery |
| 2B | eBot planning + navigation (50/50) | A* on a coarse grid (4× cells), rocks grown by clearance radius tried from 0.36 m down to 0.26 m, path "pulled tight" like a string, resampled every 0.10 m; navigation TURN/TRACK/ALIGN/DONE, lidar obstacle guard |
| 3A | Both robots, one run | Package detection (magenta top, H≈154), handover topics, package pick (≈2.94 N), hopper drop, eBot waits beside arm, then home |
| 3B | Noise + map mismatch, 2 runs | Averaged stillness/speed/force, LiDAR returns not on the map added as rocks before each leg, same nodes both runs |

---

## 5. Task 3 architecture (most likely viva area)

```
camera ──► perception ──TF: <kind>_ore_<n>, ore_package──► manipulation ──► arm servo
                ▲  /handover/ebot (AT_ARM)                    │ /handover/arm (TAKING, PACKAGE_CLEAR)
                │                                             ▼
/map,/odom,/scan ──► planning ──/ebot_path──► navigation ──► /cmd_vel
                     │ /handover/ebot (EN_ROUTE, AT_ARM, LEAVING)
```

- **Four nodes**, one job each: perception (see), manipulation (arm), planning (eBot
  mission + paths), navigation (drive the path).
- **Handover protocol** — our own two topics, `std_msgs/String`:
  - planning publishes `AT_ARM` only after the eBot stands still on the stop pose;
  - manipulation publishes `PACKAGE_CLEAR` only after the magnet holds ≈2.94 N **and** the
    package is lifted 0.12 m off the deck;
  - planning drives home only after `PACKAGE_CLEAR`.
- **QoS: reliable + TRANSIENT_LOCAL (latched).** A node that starts late, or restarts,
  immediately receives the last state. That removes start-order bugs.
- **Stop pose beside the arm:** map (0.0, 1.9). It is inside the arena wall, free on every
  rock map, puts the package ≈0.67 m from the arm base (inside reach) and in the camera's view.
- **Hopper:** found from the crusher's collision mesh — rim x −1.25…−0.65, y −0.31…+0.30,
  z +0.21 (arm frame). We release at (−0.78, 0.02, 0.33): inside the rim, within reach.
- **Arm job order:** ores one by one; at each job boundary, if the eBot is `AT_ARM` and the
  package is seen, the package goes first, so the eBot can leave early (time marks).
- **Collision safety with the eBot:** when the eBot is near the arm, every arm route in front
  of the base travels at 0.40 m height, above the eBot deck (0.18) and package (0.26).

---

## 6. Question bank with model answers

Probability: ★★★ very likely, ★★ likely, ★ possible. Each answer fits the 4-step shape.

### A. Big picture and design decisions

**A1 ★★★ Explain the overall architecture of your Task 3 solution.**
Four nodes: perception, manipulation, planning, navigation. Perception turns camera
images into TF frames for each ore and the package. Manipulation runs a state machine
that picks items and drops them. Planning plans each eBot leg with A* and runs the eBot's
mission. Navigation follows the published path. The robots coordinate through two
latched topics of our own, `/handover/ebot` and `/handover/arm`.

**A2 ★★★ How do the arm and the eBot coordinate the handover? Why that design?**
The eBot says `AT_ARM` only when it is standing still on the stop pose. The arm takes the
package at its next job boundary and says `PACKAGE_CLEAR` only when the force shows it is
holding it and it has lifted it off the deck. Only then does the eBot leave. We used two
simple topics instead of a fifth coordinator node: fewer processes, fewer failure points.
They are latched (TRANSIENT_LOCAL), so start order does not matter.

**A3 ★★ Why did you not use MoveIt or Nav2?**
They are not allowed, and writing our own also lets us control every limit the evaluator
checks: the arm's speed caps, the path rules, the stop tolerances.

**A4 ★★ What was the hardest problem, and how did you solve it?**
Pick one and tell it as a story. Good options:
- 3B: the planner never started, because odometry noise made the eBot look like it was
  always moving. We averaged the last ~12 odometry readings before deciding it is still.
- 2A: orange ground and the orange rover were detected as vanadinite. We added the
  "dark ring" test: an ore's coloured top is surrounded by black sides; the ground is not.
- 3A: the arm's normal travel arc went through the space where the eBot parks. We raise
  the arc in front of the base while the eBot is near.

**A5 ★★ If you had more time, what would you improve?**
Speed: the arm spends most time in transit, so shorter routes and smoother motion.
Robustness: re-plan during the first turn of a leg with fresh LiDAR data.
(Being honest about limits scores better than claiming perfection.)

### B. Perception (camera, OpenCV, TF)

**B1 ★★★ How do you detect the ores and the package?**
Convert the image to HSV, threshold each kind's colour range, clean the mask, find
contours. A region counts only if its size is right and a ring of black surrounds it (ore
sides are black). The package is the only magenta object. The centre pixel plus the
aligned depth gives a 3D point.

**B2 ★★★ Why HSV and not RGB?**
In HSV the colour (hue) is separated from brightness. A shadow changes V a lot but H
little, so one hue range works across lighting. In RGB all three channels change with light.

**B3 ★★★ How do you convert a pixel to a 3D position in the arm's frame?**
Read depth at that pixel (median of a 9×9 patch, ignoring NaN/inf). Pinhole model:
x = (u − cx)·z/fx, y = (v − cy)·z/fy, with fx, fy, cx, cy read from `camera_info`.
Then transform the point from the camera optical frame to `base_link` with TF. We
subtract half the item height (38 mm) so the TF names the middle of the block.

**B4 ★★ What is TF and why publish ores as TF frames?**
TF keeps every coordinate frame and the transforms between them over time. Publishing
`azurite_ore_1` etc. lets any node ask "where is this ore in base_link" without knowing the
camera. The evaluator also reads them for the detection marks.

**B5 ★★ How do you keep ore names stable (azurite_ore_1 stays azurite_ore_1)?**
A tracker matches each sighting to the nearest known ore of the same kind (within 6 cm).
An ore is named once it has 10 sightings, so a one-off false detection never gets a name.
After all four are named, the set is fixed, and a carried ore keeps its name.

**B6 ★ Why a median and not a mean?**
The median ignores outliers: edge pixels that fall on the floor behind the ore, or a noisy
reading. A mean would be pulled by them.

### C. Manipulation (UR7e arm, magnet, force)

**C1 ★★★ How does your arm pick an ore?**
Hover 12 cm above the top face. Descend at 25 mm/s to 2 cm above, then at 10 mm/s.
Contact = force above 0.8 N while within 2 cm of the top. Magnet on, wait 0.8 s, lift.
Then check the hold: an ore adds ≈1.47 N, the package ≈2.94 N. If the force is wrong,
release and retry (up to 3 times).

**C2 ★★★ How do you control the arm without a trajectory planner?**
Servoing: every 50 ms we send a tool velocity on `/delta_twist_cmds`. Velocity = gain ×
position error, scaled as a whole vector so the direction is kept, and kept below the
caps (0.22 m/s; the limit is 0.25). Orientation is held tool-down the same way.
Routes are lists of waypoints; corner waypoints are passed at full speed.

**C3 ★★★ What is a protective stop and how do you avoid / recover from one?**
The arm latches (`/arm_status` 20–23) when a limit is broken: too fast, too much force,
collision. Avoid: speeds below the caps, slow final descent, overpress check at 3 N, high
routes over obstacles. Recover: release the magnet, call the unlock service, lift for 3 s
inside the recovery window, then continue; give up after 3 because each stop costs 2.5 marks.

**C4 ★★ Why unfold in joint space before moving in Cartesian space?**
The start pose is folded close to a joint limit, and Cartesian motion from there can
hit singular configurations. A short joint-space move to a comfortable pose first
avoids that.

**C5 ★★ How do you know the magnet is really holding the item?**
The magnet service only reports the coil, not contact. So we use the force sensor: tool
weight is already removed, so the reading equals the held item's weight. We check the
mean of 10 samples is in the right window.

**C6 ★★ How did you find where the crusher hopper is?**
From the crusher's collision mesh in the world model: we converted its vertices into the
arm's frame and found the rim (x −1.25…−0.65, z +0.21). We release inside the rim on the
arm's side, where the arm can still reach.

**C7 ★ Why does the arm travel on an arc and not a straight line?**
Ores are on the right, containers on the left. A straight line passes near the base axis,
a singular region where small tool moves need huge joint speeds. The arc keeps a safe radius.

### D. eBot planning and navigation

**D1 ★★★ Explain your path planner.**
The map is an occupancy grid (5 mm cells). We grow rocks by the body clearance and run
A* on a coarser grid (4× wider cells) for speed. A* uses f = g + h, with h the straight-line
distance, which never overestimates, so the path is the shortest. Then we "pull the path
tight" like a string and resample it every 10 cm.

**D2 ★★★ Why A* and not Dijkstra / RRT / RL?**
A* finds the optimal path on a grid and is faster than Dijkstra because the heuristic
focuses the search. RRT gives random, not shortest, paths. RL is not reliable enough: a
failed run scores zero, and the evaluator checks the path against the map.

**D3 ★★★ How do you make sure the published path is valid?**
The first and last poses are exactly the leg's start and end. Every segment is checked
cell by cell against blocked cells, which include every occupied map cell grown by the
clearance. So it never crosses an occupied cell.

**D4 ★★ How does the eBot follow the path?**
Pure pursuit: aim at a point a lookahead distance ahead on the path (longer when faster),
steer by the heading error. If the error is large, stop and turn on the spot first. Near
the end, slow down, then turn to the final yaw. Arrival tolerance is tighter than the 0.3 m
and 0.15 rad the evaluator allows.

**D5 ★★★ In 3B the map may not match the world. What did you do?**
Before each leg, while standing still, we take 5 LiDAR scans and use the median of each
beam to remove noise. Returns that are more than 15 cm from any mapped rock, and have
neighbours (so they are not noise), are added to the planning map as rocks. The path then
avoids them and still never crosses a /map rock. In testing it found 12 such returns before
leg 1 and planned round them. During driving, the LiDAR guard slows and stops before contact.

**D6 ★★ Why does the eBot face west at the ore drop pose?**
Leg 2 goes west. Turning at the end of leg 1 costs the same time as turning at the start of
leg 2, but its LiDAR then sees leg 2's way before we plan it.

**D7 ★ How do you request the ore package?**
Call `/spawn_ore_package` (a Trigger service) from our node, only after standing still on
the drop pose; retry every second if refused; wait 2 s for the package to settle before driving.

### E. Noise and robustness (3B)

**E1 ★★★ How did you make your nodes robust to sensor noise?**
We measured it first: odometry ±1.2 cm and ±0.02 m/s, LiDAR ±4 cm per beam, depth ±8 mm,
force ±0.12 N. Then averaged every decision: stillness over 12 odometry readings, speed
over 8, touch over 3 force samples, hold over 10, ore position as the median of 15
sightings, LiDAR as the median of 5 scans.

**E2 ★★ What happens if the package is not detected when the eBot arrives?**
The arm keeps sorting ores; it re-checks at every job boundary. If all ores are done it
waits above the stop pose. The eBot never leaves while the package is on its deck, because
it waits for `PACKAGE_CLEAR`.

**E3 ★★ What if a node crashes or starts late?**
The handover topics are latched, so a restarted node gets the current state at once. Each
node also has timeouts that stop it cleanly with a logged reason instead of hanging.

**E4 ★ Why must both 3B runs use the same nodes?**
The task checks robustness, not tuning for one map. Our nodes read everything from the
fresh world each run and hold no state between runs.

### F. ROS 2 fundamentals (asked to check understanding)

**F1 ★★★ Topic vs service — where did you use each?**
Topic: continuous one-way data (camera, odometry, velocity commands, handover state).
Service: one request, one reply (magnet on/off, package spawn, controller switch, unlock).

**F2 ★★ What is QoS? Why TRANSIENT_LOCAL for /map, /ebot_path and the handover topics?**
Quality of Service sets reliability, history and durability. TRANSIENT_LOCAL keeps the last
message for late subscribers. /map is published once, so a node that starts later still gets it.

**F3 ★★ What is `use_sim_time` and why set it?**
Nodes use the simulation clock (`/clock`) instead of wall time, so timers, timeouts and
timestamps match the simulation even if it runs slower than real time. We found that a
timer must start on the first cycle, not at time 0, because the world may already have run.

**F4 ★★ Why must you rebuild with colcon after editing a script?**
`ros2 run` runs the installed copy in `install/`, not the file in `src/`. `colcon build`
copies it; then `source install/setup.bash`.

**F5 ★ Why never call a service synchronously inside a callback?**
The node is already spinning; waiting inside a callback blocks the executor and can
deadlock. We use `call_async` and check the result in the timer.

### G. Team and process

**G1 ★★ How did you split the work in the team?**
Answer truthfully with names: who did perception, arm, eBot, testing, video. Each member
should be able to explain their part.

**G2 ★ How did you test before submitting?**
We ran full runs in the simulator headless, read the logs with timestamps, timed every
phase, and only then ran the official evaluator. Every bug was reproduced, fixed and re-run.

---

## 7. The drill (do this twice before pressing Start)

1. Decide **who answers which area**: e.g. member 1 = architecture + handover (A, E),
   member 2 = perception (B), member 3 = arm (C), member 4 = eBot (D). F and G: anyone.
2. Each member reads their section, then explains each ★★★ question aloud **without
   reading**, in 60–90 s, using the 4-step shape. Others time it.
3. Random round: one person reads 5 random questions from the bank; the right member answers;
   total must stay under 9 minutes.
4. Do the portal **practice page** once with the real camera and mic. Check that the person
   farthest from the laptop is heard clearly.
5. On the day: sit together, test the mic, open the page, press Start, read each question
   aloud first, then answer. Say "This is <name>, I'll answer this one" so the evaluator
   sees that every member answered.

## 8. Rules to remember during the recording

- Only record on the portal page; any other recorder or an edited video scores zero.
- Everyone's face visible the whole time.
- If a question is about something another member did, that member answers.
- Upload to YouTube as **Unlisted**, paste the link within 3 hours.
