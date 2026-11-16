# StrataCobot (eYRC 2026-27), Team 4372

Our code for the e-Yantra Robotics Competition theme **StrataCobot**: a UR7e robot arm sorts
ores from a conveyor into a rover, and an eBot (a small mobile robot) drives across a rocky arena,
collects an ore package and brings it to the arm, which drops it into a crusher.

This page tells you everything you need to set the project up on a fresh computer, run it, run
the official evaluator, record the video and pack the submission. Follow it from top to bottom
the first time. You do not need to know ROS 2 beforehand: every command is given in full.

| Task | What it is | Our best official result |
|---|---|---|
| 0 | Setup check | done |
| 1A | Ore detection with OpenCV | done |
| 1B | UR7e arm waypoints | 40 / 40 |
| 1C | eBot path following | 40 / 40 |
| 2A | Arm picks and sorts 4 ores | 50 / 50 |
| 2B | eBot plans and drives a 3-step lap | 50 / 50 |
| **3A** | Arm and eBot together, one run | **28.24 / 30** (all items, time 147 s) |
| **3B** | Same with noisy sensors and shifted rocks, two runs | **47.00 / 50** (all items, 149 s and 151 s) |

Why not 30 / 30 and 50 / 50: in Task 3 every item we can score is already scored. What is left is
time marks, `(600 − T) × 0.02`, which would need T = 0 seconds for full marks.

---

## Contents

0. [For the person doing the final recordings](#0-for-the-person-doing-the-final-recordings)
1. [What you need](#1-what-you-need)
2. [One-time setup](#2-one-time-setup)
3. [Opening a terminal the right way](#3-opening-a-terminal-the-right-way)
4. [Before every run: the checklist](#4-before-every-run-the-checklist)
5. [A practice run (no evaluator)](#5-a-practice-run-no-evaluator)
6. [Official evaluation: Task 3A](#6-official-evaluation-task-3a)
7. [Official evaluation: Task 3B (two runs)](#7-official-evaluation-task-3b-two-runs)
8. [Recording and packing the submission](#8-recording-and-packing-the-submission)
9. [What the messages mean](#9-what-the-messages-mean)
10. [When something goes wrong](#10-when-something-goes-wrong)
11. [How the code is organised](#11-how-the-code-is-organised)
12. [Rules we never break](#12-rules-we-never-break)

---

## 0. For the person doing the final recordings

Your job: set the project up once, then record **one Task 3A evaluation** and **one Task 3B
evaluation** (which is two runs), check both results, pack them and send them to Jinadev. In order:

1. Section 2, once: install, clone, build, check the graphics card. About 1 hour, mostly downloads.
2. Section 5: one practice run, to see that everything works on your laptop.
3. Section 4 checklist, then section 6 (3A) while recording the screen (section 8).
4. Section 4 checklist again, then section 7 (3B) while recording.
5. Section 8.4: check the results, the zips and the videos.
6. Section 8.5: send everything to Jinadev.

Expected results on a working setup: 3A about 28 / 30, 3B about 47 / 50, each run lasting about
2.5 to 3 minutes. Every run should show `Detected 5 of 5, dropped 5 of 5, eBot legs 2 of 2, and
the bonus`.

## 1. What you need

- **Ubuntu 24.04** installed on the computer (not in a virtual machine: Gazebo needs the real
  graphics card).
- **16 GB RAM** recommended (the runs use about 10 GB), and **about 20 GB of free disk**.
- An internet connection for the first setup.
- Access to this private GitHub repository. Ask Jinadev to add your GitHub account as a
  collaborator, then accept the invitation e-mail.

If your computer runs a different Ubuntu version, stop here and ask: the e-Yantra software only
supports Ubuntu 24.04 with ROS 2 Jazzy. (Jinadev's laptop runs it inside a `distrobox` container
named `eyrc`; see section 3.)

## 2. One-time setup

Do this once per computer. Use a normal terminal (Ctrl + Alt + T). Paste each block, press
Enter, and **wait until it finishes** before pasting the next one. When a command asks for your
password, type it (nothing is shown while you type) and press Enter. When it asks `[Y/n]`, press
Enter.

### 2.1 Install ROS 2 Jazzy and Gazebo Harmonic

Block 1, the ROS 2 package source (one paste):

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Block 2, ROS 2 itself with Gazebo, and the build tool (one paste; this downloads a few GB and can
take 20 to 40 minutes):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop-full python3-colcon-common-extensions git zip unzip ffmpeg mesa-utils
```

(`zip` packs the submission, `ffmpeg` converts the video, `mesa-utils` gives the graphics check
of section 2.6.)

Check it worked (one line):

```bash
source /opt/ros/jazzy/setup.bash && ros2 --help | head -3
```

It should print `usage: ros2 [-h] ...`. If it says `command not found`, block 2 did not finish:
run it again and read the last error lines.

### 2.2 Get this repository

The workspace folder is `~/ros2_ws`, and this repository becomes its `src` folder. One paste:

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/Jinadev15/e-yaantra-strata-cobot-4372.git src
```

Git asks for a username and password. The password is **not** your GitHub password: use a
personal access token (GitHub → Settings → Developer settings → Personal access tokens → Tokens
(classic) → Generate new token, tick `repo`, copy the token and paste it as the password).
Alternatively install the GitHub CLI (`sudo apt install -y gh`), run `gh auth login`, follow the
questions, then run the `git clone` line again.

### 2.3 Install the competition's extra packages

One paste, wait for it to finish:

```bash
cd ~/ros2_ws/src && ./requirements.sh
```

It installs what the arena needs and prints what it did. To only check, without installing:
`./requirements.sh --check`.

### 2.4 Build the workspace

One paste, takes 1 to 3 minutes:

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build
```

It must end with `Summary: N packages finished` and no `Failed` line. Then check that our scripts
are there (one line):

```bash
source ~/ros2_ws/install/setup.bash && ros2 pkg executables algorithms | grep task3
```

You should see 8 lines: `algorithms task3A_manipulation.py` and so on, four for 3A and four for 3B.

### 2.5 Getting updates later

When the team pushes new code, update and rebuild (one paste):

```bash
cd ~/ros2_ws/src && git pull && cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select algorithms
```

### 2.6 Check the graphics card

Gazebo must draw on the real graphics card, or it becomes very slow. One line in a normal terminal:

```bash
glxinfo -B | grep -E "OpenGL renderer"
```

- It names your **NVIDIA** or **AMD Radeon** card, or an Intel **Iris/UHD** card on an Intel-only
  laptop: good.
- It says **`llvmpipe`**: the graphics driver is missing. For an NVIDIA card run
  `sudo ubuntu-drivers install`, restart the laptop, and check again.
- On a laptop with **both** Intel and NVIDIA graphics, check the NVIDIA card is usable:

  ```bash
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B | grep -E "OpenGL renderer"
  ```

  If that names the NVIDIA card, paste these two lines into the **world terminal (T1)** each time,
  before the `ros2 launch` line, so Gazebo draws on it:

  ```bash
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  ```

## 3. Opening a terminal the right way

Every ROS 2 command below runs in a terminal that has been **prepared** with these two lines.
Open a new terminal (Ctrl + Alt + T) and paste them together (one paste):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Nothing is printed: that is normal. A terminal prepared like this is called a **ROS terminal**
below. Every new terminal must be prepared again; preparing it twice does no harm.

To save typing, you can add both lines to the end of `~/.bashrc` once, and every new terminal is
prepared by itself (one paste, only ever once):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**On Jinadev's laptop only** (Ubuntu 26.04 with the `eyrc` container): first `distrobox enter eyrc`,
wait for the prompt, then paste these three lines, and use `~/ros2_ws_jazzy/ros2_ws` wherever this
page says `~/ros2_ws`:

```bash
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_jazzy/ros2_ws/install/setup.bash
```

## 4. Before every run: the checklist

The evaluator refuses to start (and the attempt counts as failed) if another Python program is
running that it cannot inspect, and Gazebo slows down badly when the computer is busy.

1. **Close** the web browser, music and video apps, and anything else you do not need. Keep
   these instructions on another device.
2. **Plug in the charger** and set the power mode to **Performance** (Settings → Power).
3. **Check for Python programs** (normal terminal, one line):

   ```bash
   pgrep -a python | grep -v -E "networkd-dispatcher|unattended-upgrade"
   ```

   It should print **nothing**. If it prints a line, close that program. A line with `wsdd` in it
   is a network helper that Ubuntu starts by itself; end it with `kill <number>`, using the first
   number on that line.
4. **Nothing left over from an earlier run**: every terminal from the last run is closed, or was
   stopped with Ctrl + C. If unsure, restart the laptop.
5. **Enough disk space** for the recording (one line; `Avail` should be at least 5G):

   ```bash
   df -h ~ | tail -1
   ```

6. **Another ROS computer on the same Wi-Fi?** (for example a teammate running the simulation at
   the same time) ROS 2 programs on one network find each other, and two simulations would mix.
   Then either use a different network, or paste this line into **every** terminal of the run
   (all six), right after preparing it:

   ```bash
   export ROS_DOMAIN_ID=43
   ```

## 5. A practice run (no evaluator)

Use this to see the robots work. Nothing is scored or saved.

You need **5 ROS terminals**, called T1 to T5 here. Prepare all five first (section 3).

**T1**, start the world (one line). Wait until it prints `Task 3A world ready` (about 1 minute;
the Gazebo window opens before that):

```bash
ros2 launch eyantra_kepler_colony task3a.launch.py
```

Then start our four programs, one line in each terminal, one right after the other:

T2:
```bash
ros2 run algorithms task3A_perception.py
```
T3:
```bash
ros2 run algorithms task3A_manipulation.py
```
T4:
```bash
ros2 run algorithms task3A_planning.py
```
T5:
```bash
ros2 run algorithms task3A_navigation.py
```

The arm unfolds and starts picking ores; the eBot drives away to collect the package. After about
3 minutes T3 prints `Done: 4 ore(s) delivered: [...], package in the hopper` and T4 prints
`Lap complete: home, ore drop pose, arm pose, home`. To stop, press **Ctrl + C** in T2, T3, T4,
T5, then T1. For the 3B world use `task3b.launch.py` and the `task3B_` programs.

## 6. Official evaluation: Task 3A

The evaluator is e-Yantra's program that watches the run and writes the signed `result.zip`.
**Start the screen recording before step 1** if this run is for submission (section 8).

You need **6 ROS terminals**: T1 world, T2 evaluator, T3 to T6 our programs. Prepare all six first.

1. **T2**, the evaluator. It must start **before** the world (one line):

   ```bash
   cd ~/ros2_ws/src && ./eyrc-sc-evaluator --task 3A --team-id 4372
   ```

   It prints a list of checks and then waits for the world.

2. **T1**, the world (one line):

   ```bash
   ros2 launch eyantra_kepler_colony task3a.launch.py
   ```

3. Wait until **T2** prints `Ready` (about 1 to 2 minutes). Then start the four programs, one
   right after the other, without waiting in between:

   T3:
   ```bash
   ros2 run algorithms task3A_perception.py
   ```
   T4:
   ```bash
   ros2 run algorithms task3A_manipulation.py
   ```
   T5:
   ```bash
   ros2 run algorithms task3A_planning.py
   ```
   T6:
   ```bash
   ros2 run algorithms task3A_navigation.py
   ```

4. Wait. The run takes about 2.5 to 3 minutes. When everything is done, T2 says the bonus is
   earned and prints the marks **by itself**. If it ever says `Press q` instead, press `q` in T2
   once T4 shows `Done: ...` and T5 shows `Lap complete`.

5. Keep recording until the marks are on screen, then stop the recording. Press **Ctrl + C** in
   T3, T4, T5, T6 and T1.

6. Save the result right away (normal terminal, one paste):

   ```bash
   mkdir -p ~/eyantra/task3a && cp ~/ros2_ws/src/SC-4372-task-3A/result.zip ~/eyantra/task3a/
   ```

   The same folder also holds `summary.txt`, a readable copy of the marks.

## 7. Official evaluation: Task 3B (two runs)

Task 3B is **two runs in one evaluation and one recording**, with the same programs both times.
Both runs must score, or the task scores 0. Start the screen recording first.

1. **T2** (one line), wait for it to wait for the world:

   ```bash
   cd ~/ros2_ws/src && ./eyrc-sc-evaluator --task 3B --team-id 4372
   ```

2. **T1** (one line):

   ```bash
   ros2 launch eyantra_kepler_colony task3b.launch.py
   ```

3. When T2 prints `Ready`, start the four **3B** programs, one right after the other:

   T3:
   ```bash
   ros2 run algorithms task3B_perception.py
   ```
   T4:
   ```bash
   ros2 run algorithms task3B_manipulation.py
   ```
   T5:
   ```bash
   ros2 run algorithms task3B_planning.py
   ```
   T6:
   ```bash
   ros2 run algorithms task3B_navigation.py
   ```

4. **End of run 1.** When the robots finish, T2 says the bonus is earned and then
   `Ready for the second run: launch the world again`. (If it does not end by itself, press `x`
   in T2 once T4 shows `Done: ...` and T5 shows `Lap complete`.) **Leave T2 running.** Then, in
   this order:
   1. Press **Ctrl + C** in T3, T4, T5 and T6, then in T1. Wait until T1 is back at its prompt.
   2. In T1, start the world again, **within 3 minutes**: press the Up arrow key and Enter, or
      paste `ros2 launch eyantra_kepler_colony task3b.launch.py` again.
   3. When T2 prints `Ready` again, start the same four `task3B_` programs in T3 to T6 again (Up
      arrow and Enter in each).

5. **End of run 2.** T2 ends by itself at the bonus and prints the marks of both runs (press `q`
   if it asks). Keep recording until the marks are on screen, then stop.

6. Ctrl + C in T3 to T6 and T1, then save the result (normal terminal, one paste):

   ```bash
   mkdir -p ~/eyantra/task3b && cp ~/ros2_ws/src/SC-4372-task-3B/result.zip ~/eyantra/task3b/
   ```

## 8. Recording and packing the submission

### 8.1 Recording the screen

Ubuntu 24.04 has a screen recorder built in:

1. Arrange the windows first: the Gazebo window big enough to see the robots, and the evaluator
   terminal (T2) visible next to it, so its `Ready` and the marks are on screen.
2. Press the **Print Screen** key (`PrtSc`). A bar appears at the bottom.
3. Click the **video camera** icon (to record video instead of a picture), choose **Screen**, and
   click the round **record** button. A red dot with a timer appears in the top-right corner.
4. Do the evaluation (section 6 or 7).
5. When the marks are printed, click the **red dot** in the top-right corner to stop.

The video is saved in `~/Videos/Screencasts/` as a `.webm` file. It must show, without cuts: the
evaluator starting, the world launching, `Ready`, the robots working, and the printed marks.

### 8.2 Converting the video

List the recordings to get the file name (one line):

```bash
ls -lt ~/Videos/Screencasts/
```

Then convert, replacing `<file>` with the recording's name without `.webm` (one paste, wait until
it finishes; it takes a few minutes):

```bash
cd ~/Videos/Screencasts && ffmpeg -y -i "<file>.webm" -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -crf 26 -preset medium -pix_fmt yuv420p -an task3a_run.mp4
```

(For 3B, name the output `task3b_run.mp4`.) Upload the mp4 to YouTube as **Unlisted** if you are
the one submitting.

### 8.3 Packing the zips

Each zip holds exactly five files at its top level: the evaluator's
`result.zip` (never opened or changed) and our four scripts. One paste for 3A:

```bash
cd ~/eyantra/task3a && cp ~/ros2_ws/src/algorithms/scripts/task3a/task3A_*.py . && zip "SC#4372_task3A.zip" result.zip task3A_perception.py task3A_manipulation.py task3A_planning.py task3A_navigation.py && unzip -l "SC#4372_task3A.zip"
```

One paste for 3B:

```bash
cd ~/eyantra/task3b && cp ~/ros2_ws/src/algorithms/scripts/task3b/task3B_*.py . && zip "SC#4372_task3B.zip" result.zip task3B_perception.py task3B_manipulation.py task3B_planning.py task3B_navigation.py && unzip -l "SC#4372_task3B.zip"
```

The last part lists what is inside: five files each.

### 8.4 Checking that everything is right

1. **The marks** (one line each):

   ```bash
   cat ~/ros2_ws/src/SC-4372-task-3A/summary.txt
   ```
   ```bash
   cat ~/ros2_ws/src/SC-4372-task-3B/summary.txt
   ```

   Good: 3A shows `Detected 5 of 5, dropped 5 of 5, eBot legs 2 of 2, and the bonus` and a result
   near 28 / 30. 3B shows the same line for **both** runs and a result near 47 / 50. If a run says
   `scores 0` or misses items, read the reason it gives, fix it (section 10), and do that whole
   evaluation again.

2. **The zips match the run.** The `result.zip` inside each zip must be the one from the
   recorded run. Compare them (one paste; both lines of each pair must be the same):

   ```bash
   unzip -p ~/eyantra/task3a/"SC#4372_task3A.zip" result.zip | md5sum; md5sum ~/ros2_ws/src/SC-4372-task-3A/result.zip
   unzip -p ~/eyantra/task3b/"SC#4372_task3B.zip" result.zip | md5sum; md5sum ~/ros2_ws/src/SC-4372-task-3B/result.zip
   ```

3. **The videos play** and have the right length (one paste; 3A about 4 to 6 minutes, 3B about
   8 to 12):

   ```bash
   cd ~/Videos/Screencasts && for v in task3a_run.mp4 task3b_run.mp4; do echo "$v: $(ffprobe -v error -show_entries format=duration -of csv=p=0 $v | cut -d. -f1) s"; done
   ```

   Then open each mp4 and skim it: the start, `Ready`, the robots and the marks must all be visible.

4. **Same run everywhere.** The marks in the video must be the marks in `summary.txt`. If you ran
   an evaluation twice, only the last one's `result.zip` is kept, so the video must be of the last
   run.

### 8.5 Sending the results to Jinadev

Send these files (Google Drive or a pen drive):

- `~/eyantra/task3a/SC#4372_task3A.zip` and `~/Videos/Screencasts/task3a_run.mp4`
- `~/eyantra/task3b/SC#4372_task3B.zip` and `~/Videos/Screencasts/task3b_run.mp4`
- both `summary.txt` files (`~/ros2_ws/src/SC-4372-task-3A/summary.txt` and `.../SC-4372-task-3B/summary.txt`)

Never open or re-zip a `result.zip` on the way: copy the files as they are.

## 9. What the messages mean

These are normal:

| Where | Message | Meaning |
|---|---|---|
| T1 | many yellow `Warning` lines while loading | the world's model files; harmless |
| T1 | `Task 3A world ready` | the world is up |
| T3 (perception) | `Ores fixed in base_link: ...` | the camera has found all four ores |
| T4 (arm) | `SELECT -> HOVER`, `DESCEND`, `GRAB`, ... | the arm's steps for each ore |
| T4 | `Descent ended (touch): ... 1.1 N` | the magnet touched the ore gently |
| T4 | `Handover: arm PACKAGE_CLEAR` | the arm has lifted the package off the eBot |
| T5 (planner) | `Step 1: 40 lidar returns added as rocks` | rocks not on the map were seen and planned round |
| T5 | `Lap complete: ...` | the eBot is back home |
| T5 | `no progress for 8 s ... planning again from here` | a rock blocked the eBot; it plans a new way round. The run continues (this step may lose its eBot marks, but the run still scores) |

## 10. When something goes wrong

| What you see | What to do |
|---|---|
| `ros2: command not found`, or `Package 'algorithms' not found` | The terminal is not prepared: paste the two lines of section 3 |
| Evaluator: `A program is running that this evaluator cannot inspect: process <n> ...` | Close that program (or `kill <n>` in a normal terminal), close everything from the run, and start again from step 1 |
| Evaluator: `a program connected to the simulation ... besides the world's own` | A Gazebo from an earlier run is still running. Close every terminal, open fresh ones, start again |
| Evaluator never prints `Ready` | Wait 3 minutes. If still nothing: Ctrl + C in T1 and T2, close leftovers (section 4) and start again |
| Gazebo is very slow or the window freezes | The computer is too busy: close the browser and other apps, plug in the charger, Performance mode. The score uses simulation time, so a slow run still scores the same |
| `colcon build` fails | Read the first error line. Usually a missing package: run `./requirements.sh` again (section 2.3) |
| The arm or eBot does nothing after a minute | Check that all four programs are running and none printed `Traceback`. Stop everything and start again |
| A run went wrong during an official evaluation | Stop everything (Ctrl + C in all terminals) and do the whole evaluation again from step 1. Only the last `result.zip` is kept |

## 11. How the code is organised

```
src/                                  <- this repository
├── algorithms/                       <- OUR package: all the code we wrote
│   ├── setup.py                      <- lists every script ros2 run can start
│   └── scripts/
│       ├── task1a/ ... task2b/       <- earlier tasks
│       ├── task3a/                   <- task3A_perception / manipulation / planning / navigation
│       └── task3b/                   <- the same four files under the task3B_ names
├── eyantra_kepler_colony/            <- e-Yantra's world: launch files, models (do not edit)
├── ebot_description/, ur_description/ <- the robots (do not edit)
├── eyrc-sc-evaluator                 <- e-Yantra's evaluator program (do not edit)
├── requirements.sh                   <- e-Yantra's installer
└── docs/
    ├── eyantra_quickstart.md         <- e-Yantra's original short instructions
    ├── superpowers/                  <- our Task 3 design and plan
    └── team/
        ├── StrataCobot_Project_Handbook.md   <- the whole project explained from zero
        └── Task3C_viva_guide.md               <- viva preparation
```

The four programs and what they do in Task 3:

- **perception**: finds the ores and the package in the camera image and publishes where they
  are (`<kind>_ore_<n>`, `ore_package`).
- **manipulation**: drives the arm: picks each ore with the magnet and drops it in its container
  on the rover, then lifts the package off the eBot and drops it into the crusher.
- **planning**: plans the eBot's paths round the rocks (A* search) and runs its lap: home → ore
  drop pose → beside the arm → home.
- **navigation**: steers the eBot along each path and stops it before anything in its way.

The arm and the eBot talk through two small topics, `/handover/ebot` and `/handover/arm`, so the
eBot only leaves the arm once the package is safely lifted. Task 3A and 3B use the same code; only
the file names differ. To change the code, edit the `task3a` file, copy the change into the
matching `task3b` file, then rebuild (section 2.5) before running.

The full explanation of every task, every design decision and the problems we solved is in
[docs/team/StrataCobot_Project_Handbook.md](docs/team/StrataCobot_Project_Handbook.md).

## 12. Rules we never break

- **Never** run `ros2 topic pub` or `ros2 service call` during an evaluation: it fails the run.
- **Never** pause Gazebo (Space in its window) or press Ctrl + Z in a terminal during a run: the
  run scores 0.
- **Never** open, unzip or change a `result.zip`: it is signed, and a changed one is rejected.
- **Never** commit `result.zip`, `build/`, `install/` or `log/` (the `.gitignore` already
  prevents it).
- Rebuild after every code change: `ros2 run` starts the built copy, not the file you edited.

---

The UR7e description in `ur_description` is modified by e-Yantra from the upstream Universal
Robots ROS 2 description; see `ur_description/LICENSE`.
