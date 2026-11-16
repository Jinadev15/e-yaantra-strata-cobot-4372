# Strata-Cobot: The Complete Project Handbook

**e-Yantra Robotics Competition (eYRC) 2026-27 · Theme: Strata-Cobot (SC) · Team SC#4372**

This handbook explains everything our team built in Stage 1, from Task 0 to Task 3, for a
reader who knows Python but has never seen ROS, Gazebo or this project. Read it from the
top. Every chapter builds on the one before it.

---

## How to use this handbook

| If you want to... | Read |
|---|---|
| Understand the whole project in 5 minutes | Chapter 1 |
| Learn the robotics ideas we use (ROS 2, frames, cameras, control, search) | Chapter 2 |
| Set up the computer and run anything | Chapter 3 |
| Understand one task | Chapters 4 to 9 (one per task) |
| Understand how the final system fits together | Chapter 10 |
| Revise for the viva | Chapter 12 (quiz) and the separate `Task3C_viva_guide.md` |
| Look up a word | Chapter 13 (glossary) |

Boxes marked **Why** explain a decision: what we chose, and what we rejected. These are the
parts examiners ask about most.

---

# Chapter 1 — The project in five minutes

## 1.1 The story of the theme

Strata-Cobot is set in a mining colony on a planet called Kepler. Two robots work together:

- A **robot arm** (a UR7e, a real industrial arm made by Universal Robots) stands at a
  processing station. It has a **magnet** on its end instead of a gripper, and a
  **RealSense depth camera** on a mast that looks down at the station.
- A **rover** called the **eBot** drives across a field of rocks. It has a **LiDAR** (a
  spinning laser that measures distances) and knows its own position from **odometry**.

Ore blocks arrive on a **conveyor**. There are three kinds, each with a coloured top:
**azurite (blue)**, **malachite (green)** and **vanadinite (orange)**. The arm must sort
them into a **cargo rover** that carries one container per kind. The eBot fetches an
**ore package** from a far point of the field and brings it to the arm, which drops it
into the **ore crusher**.

Everything runs in **simulation**: a 3D physics world (Gazebo) on our laptop, not real
hardware. Our job is to write the programs that make the robots do the work by
themselves. An **evaluator program** from e-Yantra watches each run and gives the marks.

## 1.2 What we built, task by task

```
 Task 0   Install everything, prove the setup works
 Task 1A  See:   find the ores in camera images, say where they are in 3D
 Task 1B  Move:  drive the arm's tool through 5 points in space
 Task 1C  Drive: make the eBot follow a route without hitting rocks
 Task 2A  See + move: detect 4 ores and sort them with the magnet      (50/50)
 Task 2B  Plan + drive: plan the eBot's own paths, fetch the package   (50/50)
 Task 3A  Both robots in one world: the eBot brings the package to the arm,
          they coordinate the handover, the arm sorts ores and crushes the package
 Task 3B  Task 3A with noisy sensors and a map that does not match the world, twice
 Task 3C  Viva: a 10-minute video answering 5 questions about our work
```

| Task | Result |
|---|---|
| 1B | 40 / 40 |
| 1C | 40 / 40 |
| 2A | 50 / 50 |
| 2B | 50 / 50 |
| 3A, 3B | Official evaluator (16 Nov 2026): 3A 28.24 / 30 (run score 47.06, T 147 s); 3B 47.00 / 50 (47.02 and 46.97, T 149 s and 151 s). Every item detected and dropped, both eBot legs, the bonus, no penalties. |

## 1.3 The final system on one page

```
                         ┌───────────────── THE WORLD (Gazebo simulation) ─────────────────┐
                         │  arm + magnet + camera   conveyor + ores   crusher   cargo rover │
                         │  eBot + LiDAR            rocks             ore package            │
                         └──────────────────────────────────────────────────────────────────┘
         camera images │            ▲ arm velocity            /map /odom /scan │     ▲ wheel velocity
                       ▼            │ commands                                 ▼     │ commands
               ┌──────────────┐  ┌──────────────┐                     ┌──────────────┐ ┌──────────────┐
               │  PERCEPTION  │  │ MANIPULATION │                     │   PLANNING   │ │  NAVIGATION  │
               │ "what is     │─►│ "pick it,    │◄── handover ───────►│ "where to go,│►│ "drive along │
               │  where"      │TF│  drop it"    │    messages         │  which path" │ │  the path"   │
               └──────────────┘  └──────────────┘                     └──────────────┘ └──────────────┘
                    arm side                                                 eBot side
```

Four programs (called **nodes**) run at the same time. Each has one job. They talk to each
other and to the simulation by sending messages. Chapter 10 explains this picture in detail.

---

# Chapter 2 — The ideas you need first

This chapter teaches the robotics ideas the rest of the handbook uses. You do not need to
memorise it; come back when a later chapter uses a word you do not know.

## 2.1 Simulation: Gazebo

A **simulator** is a program that pretends to be the real world. **Gazebo Harmonic** draws
the robots and objects in 3D and computes physics: gravity, collisions, friction, motors.
It also produces fake sensor data (camera images, laser distances) as if real sensors were
there. We write the same programs we would write for real robots, and Gazebo answers as the
real world would.

e-Yantra gives us the world: the arena, the robots, the sensors and a **launch file** that
starts it all (`task3a.launch.py`). We are not allowed to change those files; we only write
our own programs.

## 2.2 ROS 2: how robot programs talk

**ROS 2** (Robot Operating System 2) is not an operating system. It is a set of libraries
and tools that let many small programs work together as one robot. We use the version
called **Jazzy**.

| Idea | Plain meaning | Example in our project |
|---|---|---|
| **Node** | One running program with one job | `task3A_perception.py` is a node |
| **Topic** | A named channel. Some nodes *publish* messages on it, others *subscribe* and receive them. One-way, continuous | `/camera/camera/color/image_raw` carries camera images, 10 per second |
| **Message** | One packet of data with a fixed shape (type) | `geometry_msgs/Twist` = a linear and an angular velocity |
| **Service** | A request-and-reply call, like a function call to another node | `/magnet` (turn the magnet on/off), `/spawn_ore_package` |
| **QoS** (Quality of Service) | Rules for a topic: must every message arrive? Should a late subscriber get the last message? | `/map` is published once and *kept* (TRANSIENT_LOCAL), so late nodes still get it |
| **Timer** | Calls a function every N seconds | our arm control loop runs every 0.05 s (20 times a second) |
| **Callback** | A function ROS calls when a message arrives | `odomcb()` runs every time a new odometry message comes |
| **Package** | A folder of code ROS knows how to build | `algorithms` is our package |
| **colcon** | The build tool | `colcon build --packages-select algorithms` |
| **Launch file** | A Python file that starts many nodes at once | `task3a.launch.py` starts the whole world |

**Topic vs service — when to use which.** Use a topic for data that keeps flowing (images,
positions, velocity commands). Use a service for one action with one answer ("turn the
magnet on" → "done").

A tiny ROS 2 node in Python looks like this:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class talker(Node):
    def __init__(self):
        super().__init__('talker_node')                              # register with ROS
        self.pub = self.create_publisher(String, '/chatter', 10)     # a topic we publish on
        self.create_timer(1.0, self.tick)                            # call tick() every second

    def tick(self):
        self.pub.publish(String(data='hello'))

rclpy.init()
rclpy.spin(talker())                                                 # keep running, call callbacks
```

Every node we wrote follows this pattern: create publishers, subscribers and timers in
`__init__`, keep the latest data in variables when callbacks fire, and do the real work in
a timer.

**Simulation time.** Gazebo has its own clock, published on `/clock`. It can run slower than
real time when the computer is busy. Every node we write sets `use_sim_time = True`, so its
timers and timeouts follow the simulation's clock, not the wall clock.

## 2.3 Coordinate frames and TF

A position only means something if you say *measured from where*. Each important part of the
world has its own **frame**: an origin point and three axes (x, y, z).

| Frame | Where it is | Who uses it |
|---|---|---|
| `world` / `map` | Fixed corner of the arena. In our world they are identical | the eBot's position, the map |
| `base_link` | The base of the arm. **+x forward, +y left, +z up** | all arm positions; the ore positions we publish |
| `camera_color_optical_frame` | Inside the camera. **+z looks forward out of the lens**, +x right, +y down | raw camera measurements |
| `base_footprint` | Centre of the eBot on the ground | eBot driving |
| `laser` | Inside the LiDAR | LiDAR measurements |

The arm's `base_link` sits at world position (−0.5973, 1.8653, 0.8825) with no rotation. So a
point at world x = 0.0 is at base_link x = 0.5973.

**TF** is the ROS system that knows how all frames relate to each other at every moment. Any
node can ask: "where is frame B, seen from frame A?" and TF answers with a **transform**
(a shift plus a rotation). Nodes can also *broadcast* new frames. Our perception node
broadcasts a frame for each ore it finds, for example `azurite_ore_1`, as a child of
`base_link`. The arm node then simply asks TF "where is `azurite_ore_1` in `base_link`?".
The evaluator does the same to award detection marks.

**Rotations** are stored as **quaternions** (four numbers x, y, z, w). You never need to read
them by hand; the library `scipy.spatial.transform.Rotation` converts them. For the eBot we
only need the **yaw** (the heading angle around the vertical axis):
`yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²))`.

## 2.4 The camera: from a pixel to a 3D point

The RealSense gives two pictures 10 times per second, both 640 × 480 pixels:

- a **colour image** (what you would see), and
- an **aligned depth image**: for each pixel, the distance in **metres** along the camera's
  forward axis. "Aligned" means pixel (u, v) in the depth image is the same spot as pixel
  (u, v) in the colour image. Pixels with no reading are NaN or infinity and must be ignored.

The **pinhole camera model** turns a pixel (u, v) with depth z into a 3D point in the camera
frame:

```
x = (u − cx) · z / fx          fx, fy = focal lengths in pixels  (457.65 here)
y = (v − cy) · z / fy          cx, cy = the image centre         (320, 240 here)
z = depth
```

These four numbers (the **intrinsics**) are read from the topic `camera_info`, never typed
into the code. Then TF moves the point from the camera frame into `base_link`.

The camera sits on a mast at base_link (−1.152, −0.02, 1.238), tilted about 48° down, looking
over the crusher at the arm's station.

## 2.5 Finding coloured objects: HSV colour segmentation

Computers store colour as **BGR** (blue, green, red), 0–255 each. The trouble: a shadow
changes all three numbers at once. **HSV** splits colour differently:

- **H (hue)**: which colour, 0–179 in OpenCV (red ≈ 0, orange ≈ 10, green ≈ 70, blue ≈ 105,
  magenta ≈ 150);
- **S (saturation)**: how strong the colour is;
- **V (value)**: how bright.

A shadow mostly changes V, so a range on H finds an object under any lighting. With OpenCV:

```python
hsv  = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (100, 130, 120), (110, 255, 220))   # white where the pixel is "azurite blue"
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

Each **contour** is the outline of one coloured blob. Its centre comes from **image
moments**: `cx = m10 / m00`, `cy = m01 / m00`. We then filter blobs by size (too small =
noise, too big = ground) and by shape tests explained in Chapter 4.

## 2.6 Moving the arm: velocity control (servoing)

There are two ways to command a robot arm:

1. **Position / trajectory control**: "go to these joint angles" and a planner works out the
   motion. MoveIt does this. **e-Yantra does not allow MoveIt or any ready-made planner.**
2. **Velocity control (servoing)**: every few milliseconds, send "move the tool at this
   velocity right now". The arm's controller turns that into joint speeds.

We use velocity control through two topics:

- `/delta_twist_cmds` (`TwistStamped`): tool velocity in `base_link` — linear (m/s) and angular (rad/s);
- `/delta_joint_cmds` (`JointJog`): one velocity per joint (rad/s).

Only one of them is active at a time; we switch with the service
`/ur_arm_controller/switch_controller`.

**Safety rules of this arm controller:**
- **Caps.** Task 3 allows 0.25 m/s, 0.60 rad/s, 0.60 rad/s per joint. A command *over* a cap is
  thrown away completely, not reduced. So we always stay below with a margin (we use 0.22).
- **Dead-man timer.** If no command arrives for 0.15 s, the arm stops. So we send commands
  20 times a second, even "zero velocity" to stand still.
- **Protective stop.** If the arm moves too fast, presses too hard, hits something or reaches a
  joint limit, it **latches**: it refuses all motion. `/arm_status` then reads 20–23. Each
  protective stop costs 2.5 marks. It can be released with the service
  `/ur_arm_controller/unlock_protective_stop`; for 5 s afterwards only motions that make the
  situation better are accepted.

**The proportional (P) controller** — the heart of all our arm motion:

```
error    = target_position − current_tool_position          (a 3D vector)
velocity = Kp × error, then shrunk as a whole if longer than the speed cap
```

Far from the target the tool moves at the cap; near it, it slows down smoothly and stops
exactly there. "Shrunk as a whole" matters: if we clipped x, y and z separately, the
direction would change. The same idea holds the tool's orientation (pointing straight down).

**Singularities.** At some arm poses, a small tool motion needs huge joint speeds (for
example, the tool right above the base axis, or the arm fully stretched). Near those, the
controller latches. Our routes keep away from them: Chapter 5 and 7 show how.

## 2.7 The magnet and the force sensor

The tool is a magnetic disc 81 mm across. `/magnet` (a `SetBool` service) switches it on or
off. It grips only when its face lies **flat and within a few millimetres** of the top of an
ore, and the whole disc is on the ore's face.

The magnet service only reports the coil state, not whether it holds anything. To *know* it
holds something we use `/ur7e/end_effector/force_magnitude`: the force at the tool, with the
tool's own weight already removed. In free air it reads about 0 N. Holding an ore (0.15 kg)
it reads ≈ 1.47 N; holding the package (0.30 kg), ≈ 2.94 N (weight = mass × 9.81). Pressing
on something also raises it, which is how we detect **touch**.

## 2.8 The eBot: odometry, LiDAR, map

- **`/odom`** (`Odometry`): where the eBot thinks it is (x, y, heading) and how fast it moves.
- **`/scan`** (`LaserScan`): 360 distance readings across the front half (−90° to +90°), up to 8 m.
- **`/ultrasonic_rl`, `/ultrasonic_rr`**: two rear distance sensors.
- **`/map`** (`OccupancyGrid`): a top-down grid of the arena, 5 mm per cell, 1784 × 1380
  cells. Each cell is 0 (free) or 100 (rock or wall). Published once when the world starts.
- **`/cmd_vel`** (`Twist`): we publish forward speed (`linear.x`, up to 0.5 m/s) and turn
  rate (`angular.z`, up to 1.0 rad/s). The eBot is a **differential drive**: it cannot move
  sideways, only forward/back and turn.

Important poses: **home** (0, 0) facing yaw 0; **ore drop pose** (6.0, 1.9), where the
package is handed out; our **stop beside the arm** (0.0, 1.9).

## 2.9 Path planning: A* search on a grid

To drive from A to B without hitting rocks, we treat the map as a grid of squares and search
for the cheapest chain of free squares. **A\*** ("A-star") is the standard method:

- Keep a **priority queue** of cells to explore, ordered by **f = g + h**:
  - **g** = cost of the best known way from the start to this cell;
  - **h** = a guess of the remaining cost: the straight-line distance to the goal.
- Repeatedly take the cell with the lowest f, look at its 8 neighbours, and update their g if
  this way is cheaper.
- Stop when the goal is taken from the queue; follow the "parent" links back to the start.

Because h never **overestimates** (a straight line is the shortest possible), the first path
found is the **shortest**. A* is faster than Dijkstra's algorithm because h steers the search
towards the goal instead of spreading in every direction.

**The robot is not a point.** We **inflate** every rock by the robot's size: a cell is
"blocked" if a rock is within the clearance radius (for the eBot, 0.26–0.36 m). Then a path
that is free for the robot's *centre* is free for its whole *body*. A **distance
transform** (`scipy.ndimage.distance_transform_edt`) gives, for every cell, the distance to
the nearest rock, which makes inflation one comparison.

## 2.10 Following a path: pure pursuit

The planned path is a list of points every 10 cm. **Pure pursuit** follows it like a person
walking to a point a few steps ahead on a line:

1. Find the path point nearest the robot.
2. Pick an **aim point** a **lookahead distance** further along the path (0.45 m plus more at
   higher speed).
3. Steer to face it: turn rate = gain × heading error. Drive slower when the error is large.

If the robot faces far away from the path (more than ~0.8 rad), it stops and turns on the
spot first. At the end it slows down, stops on the last point and turns to the final heading.

## 2.11 State machines

A long job (find ore → move above it → go down → grab → lift → check → carry → drop) is
easiest to write as a **state machine**: a list of named **phases**, the current phase, and
one small function per phase that runs every timer tick and decides when to move to the next
phase. Every phase has a timeout, so the robot never hangs silently. Both our arm node and
our eBot nodes are state machines.

## 2.12 Noise and why we average

Real (and Task 3B's simulated) sensors are **noisy**: the same quantity measured twice gives
slightly different numbers. We measured the 3B noise ourselves (Chapter 9): odometry jumps by
about ±1.2 cm and ±0.02 m/s, LiDAR by ±4 cm, depth by ±8 mm, force by ±0.12 N. A decision
based on **one** reading can be wrong. A **mean** (average) of several readings shrinks random
noise; a **median** (the middle value) also ignores wild outliers. Every decision our final
code makes uses one of these.

---

# Chapter 3 — Our setup and the way we worked

## 3.1 Why a "box" (distrobox)

e-Yantra's software targets **Ubuntu 24.04 with ROS 2 Jazzy**. The team leader's laptop runs
**Ubuntu 26.04**, which ships a newer ROS 2 (Lyrical) that broke e-Yantra's packages at build
time. Re-installing the laptop with 24.04 would have cost days. So we used **distrobox**: it
runs a complete Ubuntu 24.04 system inside a **container** on the same laptop, sharing the
home folder, screen and graphics card. Inside it we installed ROS 2 Jazzy and Gazebo Harmonic.
The container is called `eyrc`.

> **Why** distrobox and not a virtual machine: a VM would need its own memory and a slow
> virtual graphics card; Gazebo needs the real GPU (an RTX 4050). Distrobox uses the real
> GPU and the same files, with no copying.

A teammate on native Ubuntu 24.04 needs no box: he installs Jazzy directly.

## 3.2 Opening a "box terminal"

Every command that runs ROS must run inside the box. In a normal terminal:

```bash
distrobox enter eyrc
```

then paste these three lines (they clear leftovers of the laptop's own ROS and load ours):

```bash
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_jazzy/ros2_ws/install/setup.bash
```

Messages about `/opt/ros/lyrical` while entering are harmless.

## 3.3 The workspace

```
~/ros2_ws_jazzy/ros2_ws/            the ROS 2 workspace
├── src/                            source code — this folder is our git repository
│   ├── algorithms/                 OUR package: every script we write lives here
│   │   ├── setup.py                lists which scripts get installed (SCRIPTS = [...])
│   │   └── scripts/task1a … task3b/
│   ├── eyantra_kepler_colony/      e-Yantra's world: arena, ores, launch files   (do not edit)
│   ├── ur_description/             the UR7e arm, camera, magnet                  (do not edit)
│   ├── ebot_description/           the eBot                                     (do not edit)
│   ├── eyrc-sc-evaluator           e-Yantra's evaluator program
│   └── requirements.sh             installs the dependencies
├── build/  install/  log/          made by colcon; never edit, never commit
```

**Git.** `src/` is a clone of e-Yantra's repository (remote `origin`). Each new task is
released as a **tag** (v2.1.0, v3.0.0) that we merge in with `git merge tags/v3.0.0`. Our own
work is pushed to the team's private repository (remote `team`,
github.com/Jinadev15/e-yaantra-strata-cobot-4372).

## 3.4 The edit → build → run loop

1. Write the script in `src/algorithms/scripts/taskXX/`, first line `#!/usr/bin/env python3`.
2. Make it executable: `chmod +x <file>`.
3. Add its path to `SCRIPTS` in `src/algorithms/setup.py`.
4. Build and load: `cd ~/ros2_ws_jazzy/ros2_ws && colcon build --packages-select algorithms && source install/setup.bash`.
5. Run: `ros2 run algorithms <file name>`.

`ros2 run` starts the **installed copy** in `install/`, not the file you edited. Forgetting
step 4 means running the old code. This is the most common beginner mistake.

## 3.5 How a task is evaluated and submitted

Each task ships with the **evaluator** (`./eyrc-sc-evaluator --task 3A --team-id 4372`). It
checks that the world is untouched and freshly launched, prints **Ready**, records the whole
run, and at the end prints the marks and writes a signed **`result.zip`** (which must never be
opened or changed).

Rules that score a run 0 if broken: starting our nodes before Ready; typing `ros2 topic pub`
or `ros2 service call` in a terminal; pausing the simulation; using MoveIt/Nav2; running
programs the evaluator cannot inspect (we met this: a GNOME network helper `wsdd` and another
Python program had to be closed before the evaluator would start).

Submission = a zip named `SC#4372_task<ID>.zip` with `result.zip` and our scripts at the top
level, plus a **screen recording** of the same run uploaded to YouTube as **Unlisted**.

## 3.6 How we worked (our method)

Every task followed the same cycle:

1. **Read** the task pages and the manuals completely.
2. **Measure** before guessing: a small "probe" script saves a camera frame, the map, sensor
   readings and the key transforms, so positions come from data, not from guessing.
3. **Design** on paper: which nodes, which states, which numbers.
4. **Build** in small steps; each class does one thing.
5. **Test** in the simulator with timestamps in every log line; time every phase.
6. **Fix the cause**, not the symptom; re-run the whole task after each fix.
7. **Evaluate** with the official evaluator, record the video, package, push to git.

Our code standard, because e-Yantra analyses every submission: object-oriented, one job per
class, every number a named constant with its unit and reason, no dead code, no fake logic,
every failure handled and logged honestly.

---

# Chapter 4 — Task 0 and Task 1A: setting up, and seeing the ores

## 4.1 Task 0 — setup

Goal: install ROS 2 Jazzy, Gazebo Harmonic and e-Yantra's workspace, build it, launch the
demo world and run the evaluator once. The evaluator produced a signed result file
(`result-SC-4372-task-0-2026-09-12.json`) proving the setup works. Chapter 3 describes the
setup we ended up with.

## 4.2 Task 1A — the problem

Six ores (two of each kind) sit in the arm's cell. Using only the camera, publish a TF frame
for each ore at its position in `base_link`, named `<kind>_ore_<n>` (for example
`azurite_ore_1`, `azurite_ore_2`). The evaluator compares our frames with the true positions.

## 4.3 How we solved it

```
colour image ──► HSV mask per kind ──► contours ──► size filter ──► centre pixel (u, v)
depth image  ──────────────────────────────────────────────────────► depth z at (u, v)
camera_info  ──► fx, fy, cx, cy ─────► pinhole model ──► point in camera frame
TF           ──► camera → base_link ─► point in base_link ──► broadcast <kind>_ore_<n>
```

Step by step:

1. **Subscribe** to the colour image, the aligned depth image and `camera_info`; store the
   latest of each.
2. **Every 0.2 s** (a timer), convert the colour image to HSV and threshold one range per
   kind. Ranges, read off a saved frame:

   | Kind | H | S | V |
   |---|---|---|---|
   | azurite (blue) | 100–110 | 130–255 | 120–220 |
   | malachite (green) | 68–80 | 90–255 | 190–255 |
   | vanadinite (orange) | 7–13 | 110–255 | 150–255 |

3. **Find contours**, drop blobs smaller than 100 px (noise) or bigger than 5000 px (ground).
4. **Centre** of each blob from image moments.
5. **Depth**: take the 9 × 9 pixel patch around the centre, keep only valid readings (finite,
   above 0), use the **median**.
6. **Pinhole model** → point in the camera frame → TF → point in `base_link`.
7. **Names that stay put**: a sighting within 30 pixels of a known ore of the same kind is that
   ore; otherwise it becomes the next number (at most two per kind).
8. **Broadcast** a `TransformStamped` with parent `base_link` and child `<kind>_ore_<n>`.

> **Why** the median of a patch, not the single centre pixel: the centre pixel can land on an
> edge where depth jumps between the ore and the floor. The median of 81 pixels ignores those.

**Problem met:** the orange ground and parts of the rover passed the vanadinite colour test.
In 1A we rejected them with a size limit and a region limit. In Task 2A we replaced this with a
cleaner rule, the **dark-ring test** (Chapter 7), which works anywhere in the image.

---

# Chapter 5 — Task 1B: moving the arm through waypoints (40/40)

## 5.1 The problem

Drive the arm's tool through five given points in `base_link`, in order, holding each for at
least 2 seconds, without a protective stop:

| # | x | y | z |
|---|---|---|---|
| 1 | −0.4085 | −0.5379 | 0.1967 |
| 2 | −0.8000 | −0.0005 | 0.3967 |
| 3 | −0.7430 | 0.5280 | 0.1967 |
| 4 | −0.4097 | 0.5280 | 0.1967 |
| 5 | −0.0763 | 0.5280 | 0.1967 |

## 5.2 How we solved it

1. **Read feedback**: tool pose (`/tcp_pose_raw`), joint angles (`/joint_states`), arm status
   (`/arm_status`).
2. **Unfold first, in joint space.** The arm starts folded, with the elbow only about 20° from
   its limit and close to a singular pose. Cartesian commands from there latch the arm. So we
   switch to the joint controller and drive three joints to a comfortable pose
   (shoulder_lift −1.20 rad, elbow −1.60 rad, wrist_1 −1.90 rad). All joints are scaled by one
   factor so they arrive together.
3. **Switch to the twist controller** (service call) and move in Cartesian space.
4. **P-control** the tool to each waypoint (Chapter 2.6), speed capped at 0.12 m/s
   (limit 0.15) and 0.35 rad/s.
5. **Hold** each waypoint: once within tolerance, keep sending zero velocity for more than 2 s.

**Problems met and fixes** (from our notes):
- The first unfold commands were ignored → switch to the joint controller *before* sending them.
- A detour up to z = 0.60 stretched the arm straight and latched fault 21 (stretched arm is a
  singularity) → removed the detour.
- Waypoint 2 is far out (0.80 m) → the tool is tilted 25° outward at the waypoints, which
  keeps the wrist away from the reach limit.

Result: **40 / 40**.

> **Lesson carried into every later task:** unfold in joint space first, stay under the caps
> with a margin, and keep away from stretched or folded poses.

---

# Chapter 6 — Task 1C: the eBot follows a route (40/40)

## 6.1 The problem

e-Yantra publishes a **route** on `/ebot_path`: a list of waypoints across the rock field.
The eBot must pass within 0.5 m of each waypoint in order (0.3 m earns an accuracy bonus)
without hitting a rock, and stop at the last one.

## 6.2 How we solved it

The waypoints are far apart and a straight line between two of them can cross a rock. So our
node (`path_follower.py`) **plans** between consecutive waypoints itself, then **follows**:

1. **Map** → grid (`GridMap`), distance transform → distance to the nearest rock per cell.
2. **For each leg** between waypoints: block cells closer to a rock than a clearance radius;
   try radii from wide to narrow (0.52, 0.46, 0.40, 0.32, 0.26 m) and keep the first that
   finds a way. A wide margin is safer; the narrow ones exist for tight gaps.
3. **A\*** with an extra cost near rocks (weight 2.0, fading to zero at 1 m), so paths stay in
   the middle of free space.
4. **Shortcut**: remove points while the straight line between neighbours stays clear.
   **Resample** every 0.10 m.
5. **Follow** with pure pursuit (cruise 0.35 m/s, lookahead 0.45 m + 0.8 s × speed, turn on
   the spot when the heading error exceeds 0.8 rad). A waypoint counts as reached within
   0.28 m (inside the 0.3 m bonus).
6. **LiDAR safety**: points in the corridor in front of the body slow the eBot down; very close
   ones stop it and turn it away.

> **Why** plan between waypoints instead of driving straight to them: a straight line can
> cross a rock. The planned path is guaranteed clear for the body, so collisions are avoided
> by design, and the LiDAR layer is only a safety net.

Result: **40 / 40**.

---

# Chapter 7 — Task 2A: detect and sort four ores with the arm (50/50)

## 7.1 The problem

Four ores lie on the conveyor at random positions each launch (any mix of the three kinds).
Two nodes:

- **`task2A_perception.py`**: detect each ore and broadcast its TF frame `<kind>_ore_<n>` in
  `base_link`.
- **`task2A_manipulation.py`**: pick each ore with the magnet and drop it in the container of
  its kind on the cargo rover.

**Marks** = CD × 3.5 + SP × 4.5 + SD × 4.5 − AC × 2.5, where CD = correct detections,
SP = successful picks, SD = successful drops, AC = protective stops. Our run: CD 14, SP 18,
SD 18, AC 0 → **50 / 50**, all four ores in about 4.5 minutes.

## 7.2 The cell, measured

We wrote a probe script that saved a camera frame, the camera's intrinsics and every key
transform. From it and from the world's model files:

| What | Value (base_link) |
|---|---|
| Ore block | 101.6 × 101.6 × 76.2 mm, coloured top, **black sides** |
| Ore top face height | z ≈ −0.064 (middle z ≈ −0.103) |
| Ores on the conveyor | x −0.45 … +0.10, y −0.60 … −0.80 (to the right of the arm) |
| Containers (world) | azurite (−0.626, 2.550), malachite (−0.932, 2.500), vanadinite (−1.224, 2.430) — to the left |
| Crusher | behind the arm (−x): never travel there with an ore |

## 7.3 Perception, step by step

1. **HSV masks** per kind (same ranges as 1A), small morphological opening to remove specks.
2. **Size gate**: 80–2000 px (an ore top is 150–550 px at this distance).
3. **The dark-ring test.** Draw a ring 3 to 9 pixels around the blob. An ore's coloured top is
   surrounded by its **black sides**; the orange ground and rover are not. The blob counts only
   if at least 70 % of the ring is black (V < 70).
4. **Depth** from a 9 × 9 median patch → pinhole → `base_link` (camera transform looked up
   once and cached: the camera never moves). Lower the point by half the ore height (38.1 mm)
   so the frame marks the ore's **middle**.
5. **Tracking with stable names** (`OreTracker`): a sighting within 6 cm of a known ore of the
   same kind updates it; the ore's position is the **median of its last 15 sightings**. An ore
   is named only after **10 sightings**, so a one-frame false detection never gets a name.
   Once four ores are established the set is **fixed**: later sightings only update the
   nearest ore (within 25 cm), so an ore being carried keeps its name.
6. **Broadcast** every named ore 10 times a second.

> **Why** a tracker, not per-frame detection: the evaluator scores the names. Without a tracker,
> `azurite_ore_1` could swap with `azurite_ore_2` between frames, or a flicker could create a
> fifth ore.

## 7.4 Manipulation: the state machine

```
WAIT_FOR_ORES ─► SWITCH_TO_JOINT ─► UNFOLD ─► SWITCH_TO_TWIST ─► SELECT ◄──────────────────┐
                                                                    │                      │
                     HOVER ─► DESCEND ─► GRAB ─► LIFT ─► VERIFY ─► TRANSIT ─► RELEASE ─────┘
                       ▲                                   │ (not held)
                       └────────────── retry ◄─────────────┘
Any phase ─(protective stop)─► RECOVER_UNLOCK ─► RECOVER_LIFT ─► SELECT
End states: FINISHED, FAILED
```

The node runs at 20 Hz. Each phase is a small method; a dictionary maps phase → method.

| Phase | What happens |
|---|---|
| WAIT_FOR_ORES | wait until perception publishes 4 ores (or start with fewer after 20 s) |
| UNFOLD | joint-space unfold, as in 1B |
| SELECT | choose the next ore: untried first, then the one furthest forward |
| HOVER | travel to 12 cm above the ore's top face |
| DESCEND | two speeds: 25 mm/s down to 2 cm above the top, then 10 mm/s aiming 5 mm below it (the tool's spring absorbs the rest so the face lies flat) |
| GRAB | stop on **touch**: force > 0.8 N *and* within 2 cm of the top. Magnet on, wait 0.8 s |
| LIFT | straight up to the hover height |
| VERIFY | mean force of 10 samples must be 0.9–2.2 N (an ore ≈ 1.47 N). If not: release, retry (max 3) |
| TRANSIT | carry to the container along an arc (below), release height 0.10 m |
| RELEASE | magnet off, wait 1 s, check the force dropped |

**The arc.** Ores are on the right of the base, containers on the left. A straight line between
them passes close to the base axis — a **singularity**. So the tool travels on a **circle round
the base** (radius 0.60 m, waypoints every 30°) at 0.14 m height, across the **front** of the
base, never behind (the crusher is there).

**Touch threshold.** We first used 0.35 N, but a reading of about 0.4 N appears higher up with
nothing touched. 0.8 N plus the "within 2 cm" condition fixed false touches.

**Robustness** built in: timeouts on every motion (unfold 40 s, others 90 s); a check that the
ore has not moved more than 3 cm before descending; retry of a release; recovery from a
protective stop (release magnet, call unlock, lift 3 s, continue; stop after 3); clean stop on
Ctrl-C.

**Classes** (one job each): `ArmFeedback` (latest sensor readings), `Waypoint`,
`CartesianServo` (P-control), `RouteFollower` (drives a list of waypoints), `JointUnfolder`,
`ContainerMap` (drop points via TF), `OreLocator` (reads ore TFs), `ServiceCaller` (one
non-blocking service call at a time).

> **Why** non-blocking service calls (`call_async` + check later): a node that waits for a reply
> inside its own timer callback blocks itself and can hang forever.

---

# Chapter 8 — Task 2B: the eBot plans its own paths (50/50)

## 8.1 The problem

No arm this time. The rocks are in a new random layout each launch. Two nodes:

- **`task2B_planning.py`**: plan a path for each step on `/map`, **publish it on
  `/ebot_path`**, and request the ore package at the drop pose.
- **`task2B_navigation.py`**: drive the eBot along the published path.

Three steps: home (0,0) → ore drop pose (6.0, 1.9) → arm pose (0.0, 1.9) → home. At the drop
pose, call `/spawn_ore_package` (it only works while the eBot stands still there).

**Path rules** (checked by the evaluator): the judged path is the last one published before the
eBot moves 0.3 m; its first and last poses must be exactly the step's start and end; it must
stay inside the map and cross no occupied cell. **Following**: stay within 0.5 m of it, end
standing still within 0.3 m and ±0.15 rad of the last pose.

Our result: **50 / 50** (OP 10, EP 25, EF 15, EC 0 in the evaluator printout — package, paths,
following, and zero collisions). Benchmark path length ≈ 14.4 m for the whole lap.

## 8.2 Planning, step by step (`PathPlanner`)

1. **`OccupancyMap`**: the map as an array + distance transform (distance to the nearest rock
   for every 5 mm cell).
2. **Blocked cells** = closer to a rock than a clearance radius. Radii tried in order
   **0.36, 0.34, 0.32, 0.30, 0.28, 0.26 m**; the widest that finds a way wins. Near the start
   and end poses the clearance is relaxed so a pose that sits next to a rock is still reachable.
3. **Coarse grid**: the search runs on cells 4 × wider (a coarse cell is blocked if any fine
   cell inside it is). 16 × fewer cells → much faster. If it finds nothing, the full grid is used.
4. **A\*** on the coarse grid, 8 neighbours, never cutting a blocked corner. Run with three
   "rock cost" weights (0, 1, 2): each may pick a different way round the rocks.
5. **Straighten**: replace the staircase of cells with the fewest straight lines that stay clear.
6. **Tighten like a string**: repeatedly remove a corner if its neighbours can see each other,
   else slide it towards their straight line as far as the rocks allow (bisection).
7. Keep the **shortest** of the candidate paths, **resample** every 0.10 m.
8. **End yaw**: of the allowed headings (0 or π), the one needing the least turning.

The planner also runs the lap: it waits until the eBot stands still near a step's start,
plans, publishes, waits for arrival (within 0.25 m and 0.10 rad, still for 1 s), requests the
package after step 1 (retry every second), waits 2 s for it to settle, then plans the next step.

## 8.3 Navigation (`PathFollower`)

Phases **TURN → TRACK → ALIGN → DONE**:

- **TURN**: at the start, turn on the spot until facing along the path (so it never sets off
  sideways and leaves the corridor).
- **TRACK**: pure pursuit, cruise 0.35 m/s, lookahead 0.45 m + 0.8 s × speed, heading gain
  1.6, minimum turn command 0.25 rad/s (below it the wheels do not move).
- **ALIGN**: inside 0.08 m of the end, stop and turn to the end yaw (within 0.04 rad).
- **DONE**: stand still.

**`ObstacleGuard`** (LiDAR): looks only at the corridor the body is about to sweep (body
width + 4 cm margin, up to 0.60 m ahead). Something there → slow down in proportion; closer
than 0.12 m → stop and turn away. Inside 0.7 m of the goal it stands down (the planner already
cleared the goal).

> **Why** the planner and navigation are separate nodes: the rules judge the *published* path,
> so planning must finish and publish before moving. Separating them also means the follower
> can be tested alone, and Task 3 could reuse both unchanged.

---

# Chapter 9 — Task 3A and 3B: both robots in one run

## 9.1 The problem

**Task 3A.** One world holds everything from 2A and 2B. In one run:

| Robot | Work |
|---|---|
| Arm | Detect and sort the four ores (as 2A). Detect the **ore package** on the eBot, lift it off and drop it in the **ore crusher's hopper** |
| eBot | Home → ore drop pose, request the package (as 2B). Carry it to a stop **beside the arm** that *we choose*, wait while the arm takes it, then drive home |

The robots must **coordinate the handover themselves**: e-Yantra gives no interface for it.
The eBot must not leave until the package is off its deck. The package's TF must be named
`ore_package` and published while it sits on the eBot beside the arm.

**Marks (one run, out of 50):**

```
Run Score = (600 − T) × 0.02  +  CD × 1  +  SD × 4  +  EN × 5  +  B × 3  −  P × 2.5
Task 3A   = Run Score × 0.6   (out of 30)
```

| | Meaning | Max |
|---|---|---|
| T | run time in seconds; time marks only for a complete run | 12 |
| CD | correct detections: 4 ores + package | 5 |
| SD | successful drops: 4 ores in own containers + package in the hopper | 20 |
| EN | eBot legs followed correctly: home → drop pose, drop pose → arm | 10 |
| B | bonus for a perfect run; it ends the run instantly | 3 |
| P | penalty per collision or protective stop | −2.5 each |

A run scores **0** unless an ore lands in its own container **and** the eBot reaches the drop pose.

**Task 3B** = the same run with **noisier sensors** and a `/map` that **may not match the
rocks** in the world, done **twice in a row with the same, unchanged nodes**. The second world
must be launched within 3 minutes of the end of the first. Marks = (Run 1 + Run 2) ÷ 2, but
**0 if either run scores 0**. Out of 50.

## 9.2 What we measured first (recon)

| Question | How we found out | Answer |
|---|---|---|
| Where is the crusher's hopper? | read the crusher's collision mesh (a `.glb` file), converted its corners to `base_link` | rim x −1.251…−0.648, y −0.31…+0.30, z +0.207; funnel floor z +0.04 — behind the arm |
| Where can the eBot stop beside the arm? | rendered `/map` near the arm | the arena wall is at world x ≈ −0.47, the arm just outside it. Stop at world **(0.0, 1.9)**, inside the wall |
| How high is the eBot's deck? | eBot model file + `/odom` height | deck top at world z 1.061 = base_link **+0.179**, deck 0.35 × 0.29 m |
| What does the package look like on camera? | drove the eBot there with our 2B nodes, saved a frame | **magenta** top (H 154, S 255, V 246), thin black rim, the only magenta in view; top centre at base_link **(0.671, 0.048, 0.256)** |
| How noisy is 3B? | a script recorded sensors with both robots still | odometry ±1.2 cm, ±0.02 m/s, ±0.04 rad/s; LiDAR ±4 cm; depth ±8 mm; force ±0.12 N |

## 9.3 Architecture

```
                 /handover/ebot  (EN_ROUTE ▸ AT_ARM ▸ LEAVING)
          ┌──────────────────────────────────────────────────────────────┐
          ▼                                                              │
 ┌────────────────┐  TF: <kind>_ore_<n>,    ┌──────────────────┐        │
 │  PERCEPTION    │      ore_package         │  MANIPULATION    │        │
 │ ores + package │ ───────────────────────► │ ores + package   │        │
 └────────────────┘                          │ job queue        │        │
          ▲                                  └──────────────────┘        │
          │ /handover/arm                       │ /handover/arm           │
          │ (IDLE ▸ TAKING ▸ PACKAGE_CLEAR)     ▼                         │
          └──────────────────────────────── ┌──────────────────┐ ────────┘
                                            │    PLANNING      │  /ebot_path  ┌──────────────┐
                                            │ legs, package,   │ ───────────► │  NAVIGATION  │──► /cmd_vel
                                            │ wait for handover│              │ follow path  │
                                            └──────────────────┘              └──────────────┘
```

Four required nodes, built from our Task 2 nodes:

| Task 3 node | Started from | New work |
|---|---|---|
| `task3A_perception.py` | 2A perception | package detection, `ore_package` TF, listens to both handover topics |
| `task3A_manipulation.py` | 2A manipulation | package job, hopper drop, handover messages, safe routes over the eBot, Task 3 speeds |
| `task3A_planning.py` | 2B planning | stop beside the arm, wait for `PACKAGE_CLEAR`, LiDAR rocks not on the map |
| `task3A_navigation.py` | 2B navigation | averaged speed for noisy odometry |

The 3B scripts (`task3B_*.py`) are **the same code** with only the file names changed: we
designed for 3B's noise from the start.

## 9.4 The handover protocol

Two topics of our own, type `std_msgs/String`, each carrying one word:

| Topic | Sent by | Words, in order | Meaning |
|---|---|---|---|
| `/handover/ebot` | planning | `EN_ROUTE` → `AT_ARM` → `LEAVING` | `AT_ARM` only once the eBot **stands still** on the stop pose with the package |
| `/handover/arm` | manipulation | `IDLE` → `TAKING` → `PACKAGE_CLEAR` | `PACKAGE_CLEAR` only once the force shows the package **is held** (2.3–3.7 N) **and** it is lifted 12 cm off the deck |

The sequence in time:

```
eBot:  EN_ROUTE ── drives leg 1 ── package ── drives leg 2 ── AT_ARM ··········waits··········· LEAVING ── leg 3 home
arm:   IDLE ── sorts ore 1 ── sorts ore 2 ──(job boundary)── TAKING ── pick ── PACKAGE_CLEAR ── hopper ── ores 3, 4
perception: publishes ore_package only between AT_ARM and PACKAGE_CLEAR
```

**QoS: reliable + TRANSIENT_LOCAL** ("latched"). The last word is kept and given to anyone who
subscribes later. So the nodes can start in any order, and a node that restarts learns the
current state at once.

> **Why** two simple topics and not a fifth "coordinator" node: the handover is two messages.
> A coordinator would be one more process to start, one more thing that can fail, and it would
> still need both robots' states. A custom message type would need a second (CMake) package;
> plain strings with a fixed list of words, each parsed into an `Enum` in one place, are enough.

> **Why** these exact conditions: the eBot must not leave with the package on its deck (rule),
> and the arm must not reach for a package that is still rolling in. "Still on the stop pose"
> and "held and lifted" are the two facts that make each step safe.

## 9.5 Perception changes

- A second detector for the package: magenta HSV (145, 150, 150)–(163, 255, 255). Its black rim
  is thinner than an ore's sides and the eBot's white body shows beyond it, so the dark-ring
  test needs **50 %** black for the package instead of 70 % (we measured 67 %).
- The package is looked for **only while** the eBot reports `AT_ARM`, and published as
  `ore_package` until the arm reports `PACKAGE_CLEAR`. While the arm is `TAKING` (its tool covers
  the package) the position is **frozen** at the last good value.
- The package is 76.2 mm tall like an ore, so the same "top face minus half height" rule names its
  middle. Offline check on a saved frame: detected at (0.670, 0.050, 0.219), within 2 mm of the
  measured position.

## 9.6 Manipulation changes

**The job queue.** At every **job boundary** (after a drop, never in the middle of a pick):

1. if the eBot is `AT_ARM` and the package is seen → take the **package** first (so the eBot can
   leave early and drive home while the arm finishes the ores — this saves time);
2. else the next ore;
3. if no ore is left but the package has not come → wait above the stop pose at
   (0.55, 0.05, 0.45);
4. when all four ores and the package are done → FINISHED.

**The package pick** reuses the ore pick (hover, two-speed descent, touch, grab, lift) with its
own hold window (2.3–3.7 N, package ≈ 2.94 N). After the hold check: send `PACKAGE_CLEAR`, then
carry it to the hopper at **package travel height 0.40 m** (the package hangs 76 mm below the
tool and must clear the hopper rim at 0.207 m). The route goes round the **left** (rover) side,
never over the conveyor. Release point **(−0.78, 0.02, 0.33)**: inside the rim on the arm's
side, within reach; the package falls onto the funnel.

**Not hitting the eBot.** The Task 2A arc passed through (0.6, 0, 0.14) — exactly where the
parked eBot's top box is (x 0.50–0.85, top +0.18) with the package on it (+0.26). Fix: in the
**front sector** of the base (within 50° of straight ahead) the tool travels at **0.40 m**
whenever the eBot is near the arm. "Near" comes from the eBot's own `/odom`: inside the box
x < 2.5 m, y > 1.0 m on the map (its last stretch in, the stop pose and the way out), or
whenever it reports `AT_ARM`, or when its position is unknown.

**Faster within the new limits.** Task 3 raised the caps (0.25 m/s, 0.60 rad/s). We use
0.22 m/s and 0.50 rad/s. Intermediate route points are passed at full speed instead of
slowing down at each one, and the arc radius is 0.50 m (shorter routes).

**A bug found in testing — the clock.** The node first started its "wait for ores, give up
after 90 s" timer at simulation time 0. The world had already been running for minutes, so the
arm gave up at once. Fix: start a phase's timer on the first control cycle, and do nothing
until `/clock` has arrived.

**Recovery with the package.** If a protective stop happens after `PACKAGE_CLEAR`, the magnet
releases and the package is lost somewhere; the node records that it cannot be picked again and
finishes the ores instead of waiting forever.

## 9.7 Planning changes

- **Steps**: home → (6.0, 1.9) → **(0.0, 1.9)** beside the arm → home.
- After step 2: publish `AT_ARM`, then **wait** (phase `HANDOVER`) until `PACKAGE_CLEAR`, then
  publish `LEAVING` and plan step 3. There is deliberately no timeout that drives away with the package.
- At the drop pose the eBot turns to **face west (yaw π)**: step 2 goes west, so the turn costs
  nothing extra, and the LiDAR then sees step 2's way before step 2 is planned.
- **Rocks the map does not show (3B).** Before planning each step, while the eBot stands still:
  1. keep the last **5 scans**; the **median per beam** removes the ±4 cm noise;
  2. convert each return to map coordinates using the eBot's pose and the LiDAR's mount
     (from TF: 0.256 m in front of the centre);
  3. a return more than **0.15 m from every mapped rock** (and within 4 m) is something the
     map does not show;
  4. keep it only if at least **3 others lie within 0.10 m** (a lone return is noise);
  5. add a 5 cm disc of "rock" at each kept return to a **copy** of the map and plan on it.

  Adding rocks can only make the path *more* careful, so the published path still crosses no
  occupied `/map` cell. If no way exists round the new rocks, it plans on `/map` alone. In a 3B
  test this found 12 unmapped returns before step 1 and planned round them.

> **Why** look before each step and not re-plan while driving: the rules judge the path published
> before the eBot moves off. Re-planning mid-step would make the eBot leave the judged path. So we
> look as well as we can **before** moving, and the LiDAR guard handles the rest while driving.

## 9.8 Navigation and noise

- **Stillness** (planner): the 3B odometry speed jumps by ±0.02 m/s even when standing still, so
  "standing still" never became true and the planner **never planned step 1** in our first 3B
  test. Fix: average the last **12** odometry readings (about 0.3 s) before deciding.
- **Speed** (navigation): averaged over the last **8** readings, so the "stuck near the end"
  check works.
- **Touch** (arm): decided on the mean of the last **3** force samples, so one noisy sample is
  not taken as contact (we saw one false touch in a 3B test before this fix).

## 9.9 How we tested Task 3

We ran the simulator **headless** (no window) from scripts, so full runs could repeat without
anyone watching. Each run saved one log per node with timestamps; a script printed a timeline and
the time spent in every arm phase.

| Run | World | What it showed |
|---|---|---|
| 1 | 3A | arm never moved → the clock bug (9.6) |
| 2 | 3A | **first complete run**: 4 ores + package in the hopper, eBot home, no protective stop, 297 s. Each ore took ≈ 60 s, mostly travel |
| 3 | 3A | speed changes: 254 s, clean |
| 3B-1 | 3B | planner never planned → the stillness bug (9.8); one false touch |
| 3B-2 | 3B | after the fixes: complete in **235 s**, 12 unmapped LiDAR returns planned round, no warnings |

Official evaluator runs, 16 Nov 2026 (each change was kept only after a fully scored run with
no penalties):

| Version | 3A | 3B | What changed |
|---|---|---|---|
| first | 27.68 (T 193.6 s) | 46.15 | the code above |
| faster arm | 28.05 (T 162.7 s) | | speed limits near the Task 3 caps (0.24 m/s), lower hover, faster lift |
| shorter routes | 28.24 (T 147.2 s) | | tangent route round the base, one stop before each pick |
| eBot recovery | 28.16 (T 153.1 s) | 47.00 (T 149, 151 s) | rocks off their mapped place planned round; re-plan after 8 s blocked |

The score cannot reach 30 / 30: the run score is 38 + (600 − T) × 0.02 with every item done, so
full marks would need T = 0. Only time is left to gain, and the arm (five picks) sets it.

One 3B run during this tuning showed a failure the practice runs had missed: a rock a few
centimetres off its mapped place stood just inside the path, the eBot stopped in front of it and
the old guard turned it back and forth for seven minutes, which scores the run 0. The planner now
looks for unmapped returns out to 8 m and treats a return 8 cm off a mapped rock as unmapped;
when the eBot gets no closer to its goal for 8 s it plans again from where it stands with every
close return as a rock, and the guard simply stops and waits for that path. A stress test with
the first look switched off forced three such stops: all three were recovered.

---

# Chapter 10 — The whole system, end to end

What happens in a Task 3A run, second by second (from practice run 3B-2):

| Time (s) | Arm side | eBot side |
|---|---|---|
| 0 | perception sees 4 ores, names them; arm unfolds | planner gets the map, takes 5 still scans, plans step 1 round 12 unmapped returns, publishes it |
| 10 | picks ore 1 | drives step 1 |
| 37 | | arrives at (6.0, 1.9) facing west, requests the package → granted |
| 40 | | plans and publishes step 2, drives |
| 47 | ore 1 delivered, picks ore 2 | |
| 66 | | stops beside the arm → **AT_ARM** |
| 94 | ore 2 delivered → job boundary → **TAKING** the package | waits |
| 118 | package held and lifted → **PACKAGE_CLEAR** | **LEAVING**, plans step 3 |
| 133 | package released over the hopper; picks ore 3 | |
| 135 | | home, lap complete |
| 235 | ore 4 delivered → **FINISHED** | |

---

# Chapter 11 — Files, commands and submission

## 11.1 Where everything is

| Path | What |
|---|---|
| `src/algorithms/scripts/task1a/ore_detector.py` | Task 1A |
| `src/algorithms/scripts/task1b/task1b.py` | Task 1B |
| `src/algorithms/scripts/task1c/path_follower.py` | Task 1C |
| `src/algorithms/scripts/task2a/task2A_{perception,manipulation}.py` | Task 2A |
| `src/algorithms/scripts/task2b/task2B_{planning,navigation}.py` | Task 2B |
| `src/algorithms/scripts/task3a/task3A_*.py` | Task 3A (four nodes) |
| `src/algorithms/scripts/task3b/task3B_*.py` | Task 3B (same code) |
| `src/docs/superpowers/specs/…task3-architecture-design.md` | Task 3 design document |
| `~/eyantra/task*/` | submission zips, videos, notes |

## 11.2 Running Task 3A for practice (all box terminals)

1. `ros2 launch eyantra_kepler_colony task3a.launch.py` — wait for "Task 3A world ready".
2. Four more terminals: `ros2 run algorithms task3A_perception.py`, `…_manipulation.py`,
   `…_planning.py`, `…_navigation.py`.

Useful read-only checks: `ros2 topic echo /arm_status`, `ros2 topic echo /odom`,
`ros2 run tf2_ros tf2_echo base_link ore_package`.

## 11.3 Official evaluation (6 terminals, in this order)

1. Close any program the evaluator cannot inspect (other Python programs, the GNOME `wsdd` helper).
2. Start the screen recorder (1080p, Gazebo on one half, terminals on the other).
3. `cd ~/ros2_ws_jazzy/ros2_ws/src && ./eyrc-sc-evaluator --task 3A --team-id 4372` — it waits.
4. Launch the world. Wait for the evaluator's **Ready**.
5. Start the four nodes.
6. When both robots finish, press **q** (3A). For 3B: **x** ends run 1; stop the nodes and the
   world; relaunch within 3 minutes; start the same nodes; **q** ends run 2.
7. Zip `result.zip` + the four scripts as `SC#4372_task3A.zip`; upload the video as Unlisted.

---

# Chapter 12 — Check yourself

Answer aloud without looking. The answer is in the chapter shown.

1. What is the difference between a topic and a service? Give one of each from our project. (2.2)
2. Why does every node set `use_sim_time`? What bug did the clock cause in Task 3? (2.2, 9.6)
3. What is `base_link`, and which way do its axes point? (2.3)
4. Turn pixel (u, v) with depth z into a 3D point. Where do fx, fy, cx, cy come from? (2.4)
5. Why HSV and not RGB? (2.5)
6. What is the dark-ring test and what problem did it solve? (7.3)
7. Why are ore names only given after 10 sightings? (7.3)
8. What happens if an arm command exceeds the speed cap? What is the dead-man timer? (2.6)
9. Why does the arm unfold in joint space before any Cartesian motion? (5.2)
10. How does the arm know it touched an ore, and how does it know it holds it? (2.7, 7.4)
11. Why does the arm travel on an arc? Why does it rise to 0.40 m in front in Task 3? (7.4, 9.6)
12. Explain A* in three sentences. Why is its path the shortest? (2.9)
13. Why inflate the rocks? What clearance radii do we try? (2.9, 8.2)
14. What must a published path satisfy to be valid? (8.1)
15. Explain pure pursuit and the four follower phases. (2.10, 8.3)
16. Describe the handover protocol: the two topics, the words, and the exact conditions. (9.4)
17. Why are the handover topics TRANSIENT_LOCAL? (9.4)
18. Why does the arm take the package first once the eBot arrives? (9.6)
19. How do we find rocks the map does not show, and why is the path still valid? (9.7)
20. Which three decisions did 3B's noise break, and how did we fix each? (9.8)
21. Why does the eBot face west at the ore drop pose? (9.7)
22. What scores a whole run 0 in Task 3? In Task 3B? (9.1)

---

# Chapter 13 — Glossary

| Word | Meaning |
|---|---|
| A\* | Grid search for the shortest path using cost so far + estimated cost to go |
| Aligned depth | Depth image whose pixels match the colour image's pixels |
| `base_link` | The arm's base frame: +x forward, +y left, +z up |
| Callback | Function ROS calls when a message arrives |
| colcon | ROS 2 build tool |
| Contour | Outline of a white region in a mask |
| Dead-man timer | Arm stops if no command arrives within 0.15 s |
| Differential drive | Two-sided wheel drive: forward/back and turn, never sideways |
| Distance transform | For every grid cell, the distance to the nearest obstacle |
| distrobox | Runs another Linux system in a container on the same machine |
| Evaluator | e-Yantra's program that checks, records and scores a run |
| Frame | An origin and three axes that positions are measured from |
| HSV | Hue, saturation, value: colour space where colour and brightness are separate |
| Inflation | Growing obstacles by the robot's size so the centre can be planned as a point |
| Intrinsics | Camera numbers fx, fy, cx, cy for the pinhole model |
| Latched (TRANSIENT_LOCAL) | Topic that keeps its last message for late subscribers |
| LiDAR | Laser sensor measuring distances in many directions |
| Median | Middle value of a sorted list; ignores outliers |
| Node | One running ROS program with one job |
| Occupancy grid | Map as a grid of free/occupied cells |
| Odometry | The robot's own estimate of its position and speed |
| P-controller | Command = gain × error |
| Protective stop | Arm safety latch after a violation; costs 2.5 marks |
| Pure pursuit | Path following by steering toward a point a lookahead distance ahead |
| QoS | Delivery rules of a topic (reliability, durability, history) |
| Quaternion | Four numbers that store a 3D rotation |
| Service | Request–reply call between nodes |
| Singularity | Arm pose where small tool motions need huge joint speeds |
| State machine | Program organised as named phases with rules to move between them |
| TF | ROS system that tracks all frames and their transforms |
| Topic | Named one-way message channel |
| Twist | Linear + angular velocity |
| Yaw | Heading angle around the vertical axis |
