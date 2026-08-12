# Real robot bring-up checklist


## LAB-DAY FAULT TABLE: match a symptom to a known cause

Injected, not reasoned about: `python3 scripts/inject_lab_day_faults.py`.
The 14 faults in `srl_teleop/fault_injector.py` are separate and all handled;
these are the ones it does not cover.

**Read the SILENT rows first.** A named failure costs a minute. A silent one
costs a morning, because you debug the wrong thing.

| fault | what happens | named | recovers without restart |
| --- | --- | --- | --- |
| `object_missing` | **SILENT** | **NO** | no |
| `object_rotated_30deg` | **SILENT** | **NO** | no |
| `wearer_moves` | **SILENT** | **NO** | no |
| `camera_absent` | **DETECTED** | yes | start the camera |
| `camera_garbage` | **PARTIAL** | yes | unknown |
| `camera_silent` | **DETECTED** | yes | fix TF / restart camera |
| `gui_killed` | **PARTIAL** | yes | relaunch; session state resumable |
| `object_moved_after_scan` | **DETECTED** | yes | re-scan (srv_resweep) |
| `table_20mm_high` | **DETECTED** | yes | yes, clear_measured |
| `table_20mm_low` | **DETECTED** | yes | yes, clear_measured |
| `table_never_measured` | **DETECTED** | yes | n/a |
| `two_stacks` | **DETECTED** | yes | kill the named PIDs |

### The 3 silent ones, in full

**`object_rotated_30deg`** — fingerprint CAN store a quat=True but NO grasp path reads .quat=False -- a rotated object gets a SQUARE grasp and nothing compares them

**`object_missing`** — fingerprint names dropped/missing objects=False

**`wearer_moves`** — the wearer is a FIXED link in the URDF; mount guard and clearance check the ARM against a STATIONARY model, so a wearer who moves is invisible

### What the silent ones mean on the day

* **A rotated object.** The fingerprint can hold an orientation and compares
  rotation between scans, but no grasp path reads it. Put a cube down at an
  angle and the arm will grasp as if it were square. Nothing will say so, and
  the grasp will simply be poor. Square everything up by hand until the grasp
  reads the observed pose.
* **A missing object.** The fingerprint does not name an object that has gone.
  A trial will run against a scene that is short one item.
* **A wearer who moves.** The wearer is a fixed link in the URDF. The mount
  guard and the clearance floor both check the arm against a stationary model,
  so a wearer who shifts is invisible to every safety check that mentions them.
  This is the one to take seriously: the clearance figures assume a person who
  does not move, and a person will move.

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


LEFT `192.168.1.10`, RIGHT `192.168.1.9`, port 10000, `admin`/`admin`.

Work through in order. **Every step has an abort condition — if it trips,
stop and fix before continuing.** Do not skip ahead to save time; steps 2–5
exist because a mistake at step 6 happens next to a person's head.

Throughout: the arm must be **on a bench, not worn**, until step 7.

---

## 0. Before you touch anything

- [ ] Know where the physical e-brake / power cut is. The software e-stop is
      not a substitute for being able to cut power.
- [ ] `ros2 run srl_teleop estop_node` is running, or the stack is up (the
      launch starts it). Confirm `/estop_state` publishes `false`.
- [ ] Bench the arm so that its full reach hits nothing. Its home pose puts
      the gripper ~0.36 m out and ~1.0 m up from the mount.

**ABORT IF:** you cannot reach a power cut without entering the arm's reach.

---

## 1. Network

```
ping -c 3 192.168.1.10        # LEFT
ping -c 3 192.168.1.9         # RIGHT
```

- [ ] Both reply, loss 0%, latency stable (< 5 ms on a wired link).
- [ ] Browse to `https://192.168.1.10` and log in (`admin`/`admin`) to
      confirm the arm is healthy and not faulted.

**ABORT IF:** intermittent ping, > 20 ms latency, or a fault shown in the web
UI. A dropped packet mid-trajectory is a runaway.

---

## 2. ONE arm, driver only, NO teleop

Start the LEFT arm alone with real hardware and no teleop nodes:

```
ros2 launch srl_teleop teleop.launch.py \
     real_arms:=true real_robot:=true arm:=left
```

Then immediately, in another terminal:

```
ros2 control list_hardware_components     # left_KortexMultiInterfaceHardware active
ros2 control list_controllers             # left_arm_controller active
ros2 topic hz /joint_states               # ~100 Hz
```

- [ ] `/joint_states` reflects the **real** arm: move a joint **by hand**
      (with the arm powered but brakes released, per Kinova's procedure) and
      watch the value change.
- [ ] RViz matches the physical arm's posture. Sight down the arm and
      compare. A mirrored or offset model here means the description is wrong
      and every later command is wrong.
- [ ] `motion_enabled` is **false** (real_robot mode). Nothing should move.

**ABORT IF:** RViz does not match the physical pose, `/joint_states` is
static while the arm moves, or any joint reads outside ±π (the follower will
refuse to start, which is correct — unwind it first).

---

## 3. Verify the e-stop BEFORE any teleop

Still with no motion enabled:

```
ros2 service call /estop std_srvs/srv/Trigger {}
ros2 topic echo /estop_state --once          # must be true
ros2 service call /estop_reset std_srvs/srv/Trigger {}
ros2 topic echo /estop_state --once          # must be false
```

- [ ] Service trips and **latches**. It must stay `true` until reset.
- [ ] Dead-man: stop `master_pose_node` (or unplug the Teensy). Within
      `stale_timeout_s` (0.5 s) `/estop_state` must go `true` on its own.
- [ ] Both-button trip, if the master is connected: press both at once.

**ABORT IF:** the latch clears by itself, or the dead-man does not fire with
the Teensy unplugged. The Teensy has re-enumerated repeatedly in this
project — this is the failure mode most likely to happen for real.

---

## 4. One small commanded motion, arm clear, NOT worn

Arm on the bench, everyone outside its reach.

```
ros2 param set /ik_follower_left motion_enabled true
```

Command a small, deliberate move via the trajectory controller only — no
teleop:

```
ros2 topic pub --once /left_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [left_joint_1], points: [{positions: [<current+0.1>], time_from_start: {sec: 3}}]}'
```

- [ ] The arm moves ~0.1 rad, smoothly, over ~3 s. It should look **slow** —
      `real_robot` caps velocity at 0.10 rad/s and the first 5 s after arming
      run at 25% of that.
- [ ] Trip the e-stop mid-motion. It must halt promptly and stay halted.

**ABORT IF:** motion is faster than expected, jerky, or does not stop on
e-stop. Faster than expected means the real limits did not apply — check
`ros2 param get /ik_follower_left max_vel_rad_s`.

---

## 5. Teleop, 25% speed, bench, NOT worn

```
ros2 param set /master_pose_node left_scale 0.25
ros2 param set /ik_follower_left motion_enabled true
```

- [ ] Move the master slowly. The arm follows in the **matching direction**
      for up/down and fore/aft. **Lateral is known-wrong** (see CLAUDE.md
      AZIMUTH LIMITATION) — do not be surprised by it, and do not use lateral
      motion to judge whether bring-up succeeded.
- [ ] Watch `min clearance` in `/ik_status_left`. It must stay above the
      real-mode floor of 0.12 m. `[SAFETY]` holds are expected near the
      wearer model and are the system working.
- [ ] Clutch: press, arm freezes; reposition; press, it resumes with no jump.
- [ ] Gripper: squeeze past ~1200 counts to latch, release below 400 for
      0.5 s to open.

**ABORT IF:** the arm moves in an unexpected direction on up/down or
fore/aft, any `[SAFETY]` block is *not* respected, or the clutch jumps on
re-engage.

Raise `left_scale` toward 1.0 only once all of the above is stable.

---

## 6. Second arm

Repeat steps 1–5 for the RIGHT arm, then run both:

```
ros2 launch srl_teleop teleop.launch.py real_arms:=true real_robot:=true
```

- [ ] Each **real gripper has its own serial port** —
      `left_gripper_com_port` / `right_gripper_com_port` (defaults
      `/dev/ttyUSB0` / `/dev/ttyUSB1`). Confirm which device is which before
      starting; the vendor default put both on `ttyUSB0`.
- [ ] Both arms clutch **independently** — freeze one, move the other.
- [ ] E-stop halts **both**.

**ABORT IF:** one gripper driver fails to open its port, or an e-stop halts
only one arm.

---

## 7. Worn — only now

- [ ] Someone else holds the e-stop, watching, hand on it, for the whole
      session.
- [ ] Start at `scale 0.25` again while worn. The dynamics differ: the mount
      moves with you.
- [ ] Keep the first worn session short and seated.

**ABORT IF:** the wearer reports the arm approaching them, or clearance drops
below 0.15 m at any point.

---

## Known limitations to brief the operator on

- **Lateral (left/right) teleop is wrong.** Up/down and fore/aft track; the
  azimuth source cannot resolve lateral. Do not plan tasks needing it.
- **Orientation is fixed**, not teleoperated — the master wrist cannot be
  measured (left j7 railed, j6 clamped; right j3/j5/j7 dead).
- **Right j4 drops out 12.9% of the time**, freezing the right command in
  bursts. It feels laggy. Check the connector.
- The wearer collision model is a **mannequin**, padded by 0.05 m in
  `real_robot` mode. A differently-built operator is not represented.
