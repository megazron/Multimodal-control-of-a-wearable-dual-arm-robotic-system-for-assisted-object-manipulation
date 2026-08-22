# HOW TO RUN IT

One page. If you read nothing else, read the first two lines and the table.

```
bash scripts/check_channels.sh      # first action of every lab session, ~3 min
bash scripts/start_gui.sh           # THE ONE COMMAND. Everything is in the window.
```

The window opens on the **RUN** tab and the first panel is **OPERATE — how the
arms are driven**. That panel is the answer to "what can this thing do":

Each mode has **three** buttons, and they are a sequence, not a choice of
one launch:

| mode | SIM ONLY | SIM + MOCK | SIM + REAL |
| --- | --- | --- | --- |
| **1 MASTER TELEOP** (01) | the sim stack | + mock hardware | + the physical arms |
| **2 VR / DESK TELEOP** (02) | sim stack, then VR transport | + mock | + real |
| **3 SHARED AUTONOMY** (03/04) | the autonomy stack | + mock | + real |
| **4 FULL AUTONOMY** (06) | the autonomy stack | + mock | + real |

`SIM + REAL` launches the stack, **waits for it to actually be up**, then
cascades — because the cascade needs a running stack and firing both at once
is a refusal every time. The panel says which step it is on and gives up
loudly after 90 s rather than stalling.

**Rehearse on SIM + MOCK first.** It is the identical sequence against
`mock_real.launch.py` and nothing physical moves.

Under the modes: **TYPE WHAT YOU WANT → Instruct** (the full-autonomy
prompt), the **EXPERIMENTS** panel, `START VR TELEOP`, the real-arm sequence,
and `STOP everything this window launched`.

---

## RUNNING A TRIAL

**EXPERIMENTS — run a trial and keep the data**, on the RUN tab. Every field
has a working default: press `RUN TRIAL` with nothing filled in and it runs
T1 under full autonomy as PILOT.

| field | default | what it does |
| --- | --- | --- |
| task | `m1` (T1 pick and place) | which task |
| mode | `06_full_autonomy` | what it is recorded under |
| participant | `PILOT` | goes in the manifest — `write_manifest()` **refuses** a name, email, DOB, address or phone |
| session | blank → a timestamp | groups trials |
| trial no. | 1, and it **advances after each run** | it was always 0 from this window, so repeat runs overwrote each other's identity |
| seed | blank → the task's default | stage-2 layouts are drawn from it |

Toggles: `scripted` (on — the runner drives the waypoints, which is what
every recorded clip used), `dry run` (**press this first** on anything you
have not run today), and the recorders — `CSV` (on: `full_state_recorder`,
one row per tick) and `video` (off: `record_rviz.py` on a virtual display,
slow; x11grab of this desktop records black).

`RUN FULL EXPERIMENT` runs the same task under every mode it is *allowed* to
run in — T1 and T1S2 are locked to 06 by the task itself — spaced 8 s apart,
sharing one session name. Never simultaneous: two runs against one
`move_group` is HARD CONSTRAINT 3 one level down.

`open the recordings folder` goes to `recordings/sessions/<session>/`.

**`live mode:` under those buttons is read from live publishers, not from
which button you pressed.** "I launched it" and "it is driving the arm" are
different facts and the gap between them is where a run goes wrong.

---

## THE THREE THINGS THAT WILL BITE YOU FIRST

1. **Source the environment in every terminal.** `start_gui.sh` does it for
   you; a bare `ros2 run` does not.
   ```
   source /opt/ros/jazzy/setup.bash && source ~/kortex_ws/install/setup.bash
   export FASTDDS_BUILTIN_TRANSPORTS=SHM        # NOT optional. HARD CONSTRAINT 5
   ```
2. **Never run two stacks.** Two `master_pose_node` split the serial stream
   and cost a full day of measurements. The GUI refuses and names the PIDs.
   HARD CONSTRAINT 3.
3. **SIGINT the bridge, never SIGKILL.** The arm permits exactly one Kortex
   session and a killed one leaks until it times out. HARD CONSTRAINT 2.

---

## FULL AUTONOMY, END TO END

The thing most people actually want. Nothing moves until you press CONFIRM.

1. `bash scripts/start_gui.sh`
2. **RUN → 4 FULL AUTONOMY**. The middle column switches to **Instruct**.
3. **Press LOOK first.** The destination is chosen by the colour the *camera*
   sees; until it has looked there is nothing to plan against.
4. Type what you want — "put the red cube on the red pad", "pick up the blue
   one". 75 phrasings are scored: 34 correct / 14 ask / 27 refused / 0
   misunderstood.
5. Read the three things it shows you **before** anything moves: what the
   parser understood, what the camera detected (with the classifier's own HSV
   margin per cube), and the announced intention.
6. **CONFIRM.** The runner is handed the task, mode 06, the detections it was
   planned from, and your sentence verbatim.

Without the GUI:
```
python3 scripts/instruct_t1.py "put the red cube on the red pad"
bash scripts/run_experiment.sh m1 --mode 06_full_autonomy --participant PILOT --scripted
```

---

## HOW MUCH THE ARMS CAN MOVE, AND WHAT CHANGED

Since 2026-08-22 the IK follower defaults to a **15 degree orientation cone**
instead of a hard-pinned wrist, in every mode.

Measured through the follower's own candidate list, from the work point, with
the 0.15 m wearer floor enforced throughout:

| wrist policy | mean reach, 12 directions |
| --- | --- |
| pinned (before) | 0.065 m — 11 of 12 directions blocked |
| **cone 15 deg (now)** | **0.304 m** |
| cone 45 deg | 0.325 m |
| position only | 0.362 m |

**It cannot change a motion that already worked.** The commanded orientation
is always tried first, and a tilt is only reached after every redundancy seed
has failed — so the cone can rescue a motion that fails outright and can do
nothing else. Every tilt it uses is logged as `[ORIENT] solved by tilting the
tool axis N deg` and published on `/ik_status_<arm>`.

To reproduce a recording made before that change:
```
ros2 run srl_teleop ik_follower_node --ros-args -p orientation_policy:=exact
```

To go wider (measured, not guessed):
```
--ros-args -p orientation_policy:=cone -p orientation_cone_deg:=45.0
```

Full write-up: `docs/system/21_what_makes_the_workspace_small.md`.

---

## HOW THE ARMS MOVE, AND WHAT CHANGED

**SET UP tab → `How the arms move (next launch)`.**

Since 2026-08-23 the follower generates motion with **Ruckig**: time-optimal,
jerk-limited, and synchronised across all seven joints. Before that it was a
per-joint step clamp, and the difference is not subtle.

| left arm, realistic slew, 50 Hz | before | now |
| --- | --- | --- |
| how far the HAND leaves the path it was asked for | **51.3 mm** | **0.0 mm** |
| joints arrive over | 2 cycles | 1 cycle |
| peak joint speed against `joint_limits.yaml` | **12.5x the limit** | 1.00x |

The 30 mm figure to compare that first row against is the grasp capture gate.
The excursion is not a tuning value: made the step size 35x smaller and it
*settles* at 68 mm rather than going away, because the joints are at
different fractions of their own travel however small the step is.

**Which one is running is on the STATUS tab, under `Motion`** — read from the
follower's own status topic, not from which item is selected. `ruckig` in
green, `sync clamp` or `LEGACY` in amber.

The chooser applies to the **next launch**, not to a running follower: every
parameter in `ik_follower_node` except `motion_enabled` is read once at
startup. The panel says so, and pressing `measure what it does to the hand`
runs the whole comparison offline, with no stack up, in a few seconds.

To reproduce a recording made before 2026-08-23, choose
`legacy clamp_towards` — or:
```
bash scripts/run_teleop.sh motion_generator:=legacy
```

**One caveat, stated plainly.** The velocity limits are the robot's own, read
from `src/srl_moveit_config/config/joint_limits.yaml`. The acceleration and
jerk limits are **assumed** — that file declares none for any of the fourteen
joints — and are set as two ramp times (`accel_ramp_s`, `jerk_ramp_s`). They
change how smoothly the velocity limit is approached and can never exceed it.

Full write-up: `docs/system/24_motion_planning.md`.

---

## HOW ACCURATE IS THE GRASP, AND WHAT MOVED

**Every figure here is against the 30 mm capture gate a grasp has to hit.**
`python3 scripts/measure_control_budget.py` prints the whole budget and
refuses to report if its own controls fail.

| | before 2026-08-23 | now |
| --- | --- | --- |
| the hand's excursion off the commanded path | 51.3 mm | **0.0 mm** |
| declared vs measured pad offset (T0, T2, T3) | 13.45 mm | **0.05 mm** |
| shell bias, on the live pick path | 10.5 mm | **0.0 mm** with a support height |
| T1's grasp, built at the wrong gripper opening | 11.43 mm | **0.00 mm** |
| **RSS of the measured terms** | 18.64 mm | **12.91 mm** |
| **worst case, if they all add** | 33.39 mm | **19.99 mm** |

**The pad midpoint is not a constant.** The Robotiq's fingers swing on a
four-bar linkage, so the distance from the wrist to the finger pads depends on
how open the hand is: 0.09833 m wide open, 0.10976 m closed on a 40 mm cube.
Quoting one without the other is what put T1's grasp 11.43 mm out.
`python3 scripts/measure_pad_mid_ee.py --by-width` prints the table; it runs
offline and needs no stack.

**Two things are still open and are worth knowing before you read a number:**

* T1's exact grasp pose is refused by T1's own table, so the follower tilts
  the tool axis 5 deg to reach it. That is inside the cone it is allowed and
  it costs **9.58 mm** of the 30 mm gate.
* `camera_link` against the physical camera module has **never been
  measured**. It is the largest unknown left, and it needs the hardware.

---

## WHAT IS ON THE TABLE

**RUN tab → Vision → `WHAT IS ON THE TABLE?`**, or:

```
python3 scripts/pick_from_table.py --self-test     # no ROS, no robot
python3 scripts/pick_from_table.py --arm left      # DRY RUN, the default
```

It segments the support surface from the wrist camera's depth, then every
cluster standing on it, then a plane-constrained grasp for each — and reports
the ones it **cannot** pick up, with the reason. Objects it can and cannot
grasp both appear; "there is a thing here I cannot pick up" is an answer.

**It needs no camera calibration.** Points come through the wrist camera's
pose, which is forward kinematics. The scene camera's extrinsic still has 0%
coverage on real recordings and this path deliberately does not use it.

To pick a *named* object, `--want "the red cube"`: the detector's mask is
intersected with the 3-D clusters and the one with the largest fraction of
its own pixels inside the detection wins. Below 15% overlap it **refuses** —
it will not substitute the nearest thing.

The chain then checks reach for both the pregrasp and the grasp, and the
**whole densified sweep** between them, not just the endpoints. A move whose
ends are both clear and whose middle goes through the wearer is refused and
says so.

---

## SEEING WHAT THE ARMS ARE DOING

```
ros2 run srl_teleop rviz_master          # markers only; commands nothing
```

Five layers, one topic each, so you can switch one off without losing the
rest:

| topic | what |
| --- | --- |
| `/viz/states` | COMMANDED, ACTUAL, and PLANNED-BUT-REFUSED **with its reason** |
| `/viz/wearer` | the body inflated by the 0.15 m floor, from the body the guard is *enforcing* |
| `/viz/path` | the densified sweep, one point per sample, coloured by that sample's clearance |
| `/viz/liveness` | joint-state age per arm, IK failures, whether the wrist was unpinned |
| `/viz/envelope` | the measured workspace as three volumes |

**The ACTUAL panel is 3-D.** It opens on `3-D (drag to orbit)` — drag inside
it to turn the scene, `reset 3-D view` to go back to the shipped angles. The
axis-aligned `FRONT` / `SIDE` / `TOP` views are still there and deliberately
do **not** orbit: being able to read a coordinate straight off them is the
point of having them.

The GUI embeds RViz in the COMMANDED panel automatically where a window
manager exists. If it cannot, the panel says **why** and RViz's own output is
in `.scratch/rviz_<panel>.log`.

---

## BEFORE HARDWARE

```
python3 scripts/real_calibration/check_all.py     # 4 modules, must pass
bash scripts/diagnostics.sh                        # before blaming the code
```

`check_all.py` proves the **instruments** work. It says nothing about the arms.

Then, in the GUI's RUN tab, the real-arm sequence in order:
`1. START REAL ARMS` → `2. HOME BOTH ARMS` → `3. OBSERVER STATION` →
`4. START REAL ARM TELEOP`.

**The real arms are still at the legacy Kortex home** and the sim is at the
presentation pose. The bridge refuses to enable on the ~1.9 rad difference
rather than commanding it — that refusal is correct. Capture the new pose on
the arms first; the values in Kortex degrees are in `docs/NEXT_SESSION.md`.
HARD CONSTRAINT 0.

---

## WHEN IT GOES WRONG

| symptom | first thing to try |
| --- | --- |
| `ros2 topic list` returns nothing against a running stack | the CLI daemon wedged. Use a small rclpy script, or `ros2 daemon stop` |
| the GUI refuses to open | one is already open. Use it, or `--anyway` |
| RViz panel is empty | read `.scratch/rviz_<panel>.log`. The panel also states the exit code |
| a launched job did nothing | `.scratch/launch_<key>.log`. Every refusal names its reason |
| discovery finds nothing | `FASTDDS_BUILTIN_TRANSPORTS=SHM`, then clear `/dev/shm/fastrtps_*` **with the stack stopped** |
| the arm stops 7 mm short | measured and expected: 0.305 deg per joint. `docs/system/22_grasping.md` |
| a grasp is refused as "unreachable" | it now quotes the distance the search reached. If it names the wearer, the floor fired — that check was dead until 2026-08-22 |

Keyed by symptom: `docs/system/06_troubleshooting.md`.
Lab-day fault table: `docs/system/03_real_robot_bringup.md`.

---

## HOW GOOD IS IT

**SET UP → `HOW GOOD IS THE SYSTEM?`**, or
`python3 scripts/measure_control_budget.py`.

It composes the recorded baselines into one budget: where the hand ends up
against the 30 mm grasp gate, how far the wearer moves before the guard hears
about it, what losing tracking costs, and how free the arms are. It measures
nothing new and prints **UNMEASURED** where nothing has been measured.

The full reading is `docs/system/23_how_good_is_it.md`. The three-line
version: the geometry is good, the positioning is marginally over budget with
two fixes switched off, and **everything that is genuinely hard is about a
person moving faster than a 0.5 s perception chain can follow.**

---

## WHAT IS STILL BLOCKED, SO YOU DO NOT WASTE A SESSION ON IT

* **7 of 14 master channels are incoherent** — a soldering problem. Master
  teleop is the mode most likely to refuse.
* **No microphone in WSL** (`/dev/snd` has only `timer`), so every voice
  number in this repository is a *parser* score, never a transcription score.
* **The camera→robot extrinsic has 0% coverage on real recordings.** Grasping
  from the scene camera alone is not calibrated. See `22_grasping.md`.
* **No human data has been collected at all** — ethics, not code.
