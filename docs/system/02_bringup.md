# Bring-up, in order

## THE WRIST CAMERAS DO NOT SEE THE WORK SURFACE FROM HOME. Read this first.

**Measured, in sim, before anyone had a real camera:** at the home pose the
bench projects to pixel **(954, 539)** and the table to **(1115, 765)** for the
right camera — both **in front of** the camera and both outside a 640-wide
image. The left camera is parked looking up and back (elevation +38.3 deg;
"sees nothing at table height from home").

**So a correctly-working camera returns an empty scene from home**, and an
empty scene is indistinguishable from a dead camera, a bad cable or a driver
that never started. That ambiguity has cost this project days before, in the
frozen `/real/joint_states`, the dead j7 pot and the 0.000 noise floor.

**THE PINNED WRIST CANNOT FIX IT.** The camera's optical axis IS the gripper
approach axis, and teleop pins that axis at (-0.153, +0.846, +0.511) — 30.7 deg
**above** horizontal. A camera on that axis looks up and forward from wherever
the hand is, so **no position** makes it look down at a table.

**WHAT TO DO: drive to the SCAN POSE first.** It commands its own orientation,
which is legitimate because scanning is not teleoperation — one posture, taken
before the trial, no operator in the loop, same collision-aware IK.

    python3 scripts/find_scan_pose.py            # search, report
    python3 scripts/verify_scan_view.py          # what the camera then sees
    recordings/baselines/scan_pose.json          # the stored answer

| | camera position | IK | work-region corners in frame |
| --- | --- | --- | --- |
| left | (+0.600, -0.225, 1.370) | **10/10** | **100%** |
| right | (-0.600, -0.225, 1.370) | **10/10** | **100%** |

Verified through the mock RGB-D publisher from those poses: left renders the
bench/table across **71%** of the frame with all four T1 objects visible
(17546 blue px, 19098 green px); right renders it across **83%** with **no T1
objects, by design** — T1 is a left-arm task and all four cubes sit at
positive x.

`scene_fingerprint_node` now WARNS BY NAME at sweep start when a camera is
more than 0.15 m from its scan pose, rather than accumulating nothing quietly.

**Appearance is the mock's, not the real camera's — no detection rate may be
read off any of this.** Detection at working distance remains UNMEASURED.


Everything here assumes `source install/setup.bash`.

## 0a. FIRST ACTION OF EVERY LAB SESSION — re-capture the channel baseline

```bash
bash scripts/check_channels.sh          # ~3 min, block A only, 14 sweeps
```

**Do this before anything else that involves the master arm, every session,
whether or not anyone touched a soldering iron.**

`master_pose_node` decides which channels to freeze from a **stored file**,
not from live hardware. If that file is older than the wiring, the software
freezes channels that now work and **nothing downstream disagrees** — the arm
simply behaves as though the repair never happened, and the only symptom is
motion that was already expected to be missing.

The runtime reads the **newest** `recordings/baselines/channels_*.json`, which
is the same file `check_channels.sh` diffs against. So making a repair take
effect is exactly one command:

```bash
python3 src/srl_experiments/trajectory_capture/channel_report.py \
    <csv> --save-baseline recordings/baselines/channels_$(date +%Y%m%d).json
```

(the script prints this line for you at the end). **Target: 12 or more of 14
coherent**, which is where degraded mode stops engaging.

If degraded mode engages, the startup banner carries a boxed warning naming
the file and its age, and `master_pose_node` re-issues it every 30 s. Seeing
that warning with a freshly captured baseline is fine; seeing it with a
baseline older than your last repair means the repair is being discarded.

## 0b. Before anything

```bash
bash scripts/diagnostics.sh
```

Checks the things that have actually gone wrong: controllers active,
`/joint_states` at ~100 Hz, the master serial port present, stale Fast DDS
shared-memory segments, arm network, e-stop.

## 1. Simulation only

```bash
bash scripts/run_teleop.sh                    # everything
bash scripts/run_teleop.sh dashboard:=true    # + status dashboard
```

Brings up MoveIt/RViz/controllers, `master_pose_node`, an `ik_follower_node`
per arm, and the e-stop. Sequencing has two independent guarantees:
`startup_delay` (12 s) holds the teleop nodes back while `move_group` loads,
and `ik_follower_node` blocks in `wait_for_service("/compute_ik")` regardless.

## 2. Add IMU-primary sensing

```bash
ros2 launch srl_teleop sensing.launch.py tau_s:=0.5
ros2 topic echo /master_pointing_left
ros2 topic echo /master_capability_left      # what is live, and what each loss costs
```

## 3. Add perception

```bash
ros2 launch srl_perception perception.launch.py
ros2 topic echo /perception/objects_info
```

## 4. Add autonomy

```bash
bash scripts/run_autonomy.sh
ros2 topic echo /autonomy/state_left         # DIRECT | ASSIST | GRASPED
ros2 topic echo /autonomy/arbiter_left       # and WHY it is in that state
```

## 5. Real hardware

See `03_real_robot_bringup.md`. In short, three terminals:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh --mock     # drop --mock when ready
```

`start_real.sh` refuses, with a named reason and exit 1, when the sim stack is
not up, the e-stop is latched, homing finishes outside tolerance, or the
bridge will not enable.

## 6. A participant session

```bash
ros2 run srl_teleop participant_safety_node
# ... physically check the e-stop is in the participant's reach ...
ros2 topic pub --once /estop_reachable_confirmed std_msgs/Bool "{data: true}"
ros2 service call /participant/arm_session std_srvs/srv/Trigger
bash scripts/run_experiment.sh e2 --participant P01 --participant-index 0
```

The full checklist is in `docs/research/03_ethics_and_safety.md` §6.
