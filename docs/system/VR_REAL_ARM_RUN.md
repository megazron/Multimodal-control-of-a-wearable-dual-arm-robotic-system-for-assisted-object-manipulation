# VR ON THE REAL ARMS — the procedure for the day

**Follow this in order. Do not skip forward to step 12 because the arms are
already powered.**

Read once before starting: `docs/system/vr_desk_operation.md`. This document
assumes that setup — you sit across the room FACING the wearer, holding the
controllers as motion capture, watching the real robot with your own eyes.
Nobody wears the headset; it stands on a shelf as the tracking reference with
its proximity sensor taped.

**Nothing in this repository has ever run against a real arm.** Every step
below is derived from the code. Section 14 lists what could still surprise
you — read it before the arms are powered, not while you are debugging.

---

## THE SHAPE OF THE DAY

| | | roughly |
| --- | --- | --- |
| A | steps 1–5: the machine talks to itself | 15 min |
| B | steps 6–9: VR measured, **sim only, arms cold** | 30 min |
| C | steps 10–13: one real arm, e-stop proven, no teleop | 25 min |
| D | steps 14–16: VR drives the real arm | as long as you like |

**You do not have to reach D.** Stopping at the end of C is a good day: a
homed arm, a proven e-stop, and a measured VR chain.

---

# PART A — THE MACHINE TALKS TO ITSELF

## Step 1. Open the operations window

```
bash scripts/start_gui.sh
```

It sources everything, refuses a second window, and opens on **SET UP →
Connect the real arms**: eight checks in plain words, each with the fix as a
button.

**PASS:** the panel reads `READY TO CONNECT THE ARMS`, or names exactly what
is wrong.

**Work down the panel and clear every row before going on.** Each of the
following is one of those rows; they are written out here because you will
also meet them from a terminal.

### 1a. This window and the running system disagree about how to find each other

*What it looks like:* something is running and `ros2 node list` shows nothing,
or shows it from one terminal and not another.

*Why:* `FASTDDS_BUILTIN_TRANSPORTS=SHM` is HARD CONSTRAINT 5 — UDP discovery
is dead on this host. Measured: with it unset, `ros2 service list` hung past
30 s against a stack that was running and answering. A shell that never
sourced `scripts/env.sh` is a shell on a different network.

*Fix:* the panel's **Restart this window to match what is running**. From a
terminal, every shell starts with:

```
source ~/kortex_ws/scripts/env.sh
```

That one line pins `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `FASTDDS_BUILTIN_TRANSPORTS=SHM`, and
unsets any inherited `ROS_LOCALHOST_ONLY`.

*Do not "fix" this by changing the domain.* Changing it in one terminal and
not another recreates the same partition under a new name.

### 1b. An earlier run left blocks of memory behind

*What it looks like:* discovery goes intermittent — a service visible to one
client and not another, `wait_for_service` timing out on a service the graph
can see. It reads as a broken node.

*Fix:* the panel's **Clear the leftovers**, or with everything stopped:

```
source ~/kortex_ws/scripts/env.sh
srl_clear_stale_shm
```

**Two globs, not one.** `rm /dev/shm/fastrtps_*` leaves the `sem.fastrtps_*`
semaphores; 188 segments once cleared to 39 "remaining", which read as a live
writer and sent a session hunting a process that did not exist.

**THEN WAIT FIVE SECONDS BEFORE STARTING ANYTHING.** Clearing removes the
segments; the participants that were using them do not find out instantly.
Starting a stack into a half-torn-down transport gives a stack that comes up
and cannot see itself — which looks exactly like the problem you just fixed,
so you clear again, and go round.

*It refuses while anything is running,* deliberately. Removing a segment a
live participant is using is worse than leaving it.

### 1c. The helper that lists what is running has hung

*What it looks like:* `ros2 node list` returns **an empty string with exit
status 0**, or does not come back. It has not failed, it is stuck.

This is the most common one on this box. It has already happened twice while
this documentation was being written.

*Fix:* the panel's **Restart that helper**, or:

```
srl_ros_daemon_reset
```

*Tell-tale:* `ros2 node list --no-daemon` sees the graph and the ordinary one
does not.

### 1d. Something is running twice

Two `master_pose_node` split the serial stream and cost a full day of
measurements (HARD CONSTRAINT 3). A stray `robot_state_publisher` does the
same damage and is **not** caught by a process count — it wins
`/robot_description`, `ros2_control_node` reads it, throws "no 'ros2_control'
tag found in the URDF" and dies, and every joint reads a plausible 0.000.

*Fix:* the panel's **Stop the extra copy** / **Stop the leftover**.

### 1e. Parts of an earlier run are still going

Stopping a launch leaves its children re-parented to init and still
publishing. Not a second stack, not a stray description — they simply answer
alongside whatever you start next. On 2026-08-20 this box had **seven** of
them from the previous day.

*Fix:* the panel's **Stop the leftovers**. It names them first.

### 1f. The master arm moved to a different socket

Not needed for VR — VR does not use the Teensy at all — so a red row here
does **not** block the day. It is listed because the panel shows it.

---

## Step 2. Can this machine reach the arms?

```
bash scripts/check_arm_network.sh
```

**PASS:** both arms answer. Left `192.168.1.10`, right `192.168.1.9`, RTT
around 1.4–1.7 ms, 0% loss.

**FAIL — `eth0` is on `192.168.3.x` or a `172.x` NAT subnet.** WSL is in NAT
mode, so this box is behind a translation the arms cannot answer through.

*Fix, on the Windows side,* in `%USERPROFILE%\.wslconfig`:

```
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown` and reopen. In mirrored mode this box's LAN address IS
Windows' LAN address, which is also what makes the headset able to reach the
VR page.

**After switching, expect step 1c.** The ros2 daemon caches network state and
will still be holding the pre-mirrored interface. That is precisely the outage
`env.sh` was written for: the graph was fine the whole time and the daemon was
not. Run `srl_ros_daemon_reset` and carry on.

*Do not skip this because "the sim works".* The sim never leaves this machine.

---

## Step 3. Rehearse the whole real-arm sequence with nothing at risk

```
bash scripts/start_real.sh --mock arm:=both
```

**PASS:** reaches `REAL ARMS LIVE`.

**If `arm:=both` aborts the hardware load,** you are single-arm today: use
`arm:=left` from here on. There is an unresolved contradiction in the notes
about whether both arms' drivers can coexist (`kortex_driver` exports
`tcp/twist.*` without an arm prefix), and this is the cheap way to settle it
before anything is powered.

Stop it with **Ctrl-C**, and read the next step before you do it again.

---

## Step 4. Learn the one that ruins the afternoon: the leaked session

**The arm permits exactly ONE Kortex session.** SIGINT closes it. SIGKILL
leaks it, and the next connection is refused until the arm times the old one
out — which can be minutes, and looks exactly like a dead arm.

* Stop the bridge with **Ctrl-C**, once, and let it finish.
* Confirm it finished: the log must contain

```
kortex session closed cleanly
```

* **Never** `pkill -9`, and never `pkill -f` a broad pattern — that matches
  the shell running it and has killed three working terminals in one session
  here.

*If you think you have leaked one:* the connection panel's **Release the
connection** asks the bridge to drop the stale session and open a fresh one
in place, without relaunching. A relaunch re-homes the arm, and a participant
session cannot absorb that.

---

## Step 5. Start the sim stack, and only the sim stack

```
ros2 launch srl_teleop teleop.launch.py gate:=false
```

**PASS:** `ros2 node list` shows `move_group`, `controller_manager`, an
`ik_follower` per arm, `mount_guard_node`, `estop_node`.

**Order matters.** `vr_pose_mapper` refuses to engage without robot TF, and
the symptom is a clutch that silently does nothing.

---

# PART B — VR, MEASURED, WITH THE ARMS COLD

Nothing in Part B touches the real arms. Do all of it before powering them.

## Step 6. Bring up the VR chain

Headset on its shelf, lenses toward your hands, 1–1.5 m away, chest height,
on something nobody will lean on. Proximity sensor taped — a folded scrap of
paper, reversible, no settings changed on a borrowed headset.

```
bash scripts/start_vr_wifi.sh
```

Open the printed `https://<ip>:8765` in the **headset's own browser**, accept
the one certificate warning, and press **ENTER VR**.

**PASS:** the page shows `immersive-vr supported`, the socket connects, and
`pose rate` settles near 72–90 Hz.

**Passthrough is also offered now** — **ENTER WITH PASSTHROUGH** — see
section 15. For desk operation you do not need it.

**FAIL — the page loads and the socket dies silently.** The certificate
exception is per ORIGIN and a WebSocket cannot prompt. Both are served from
one port for exactly this reason; if you have split them, don't.

**FAIL — `navigator.xr absent`.** Not a secure context. Use the `https://`
URL, not the IP with `http`.

## Step 7. The status monitor, on the desk beside you

```
python3 scripts/vr_desk_monitor.py
```

Link, latency, per-arm clutch, scale, gripper, lag, and **TRACKED / OCCLUDED**
per controller. You cannot see an in-headset panel; this is where the state
lives.

## Step 8. MEASURE THE YAW. FIRST. NOT OPTIONAL.

```
bash scripts/vr_measure_session.sh yaw
```

You face the wearer, so your "away from me" is the robot's "toward its own
front". Until this angle is measured the mapper adds your displacement **raw**
and the arm moves in a direction nobody chose.

The script asks for two motions: ~40 cm straight **towards the wearer**, then
~40 cm to **your own right**. The second is a cross-check, not an input — if
the two do not come out perpendicular and horizontal, it **refuses to write a
number** rather than writing a confident wrong one. It also refuses motions
under 100 mm or more than 35° off horizontal.

**PASS:** it prints a yaw and writes it.

**FAIL — it refuses:** your frame is not a pure yaw. A tilted shelf, or the
motion was not the one asked for. Level the shelf and repeat.

**Why an angle and never a mirror:** "they're facing me, so mirror it" is a
reflection, det = −1. Applied to position it looks right; applied to the pose
it mirrors every **orientation**, so the gripper rolls the wrong way while
the positions look correct. That is the hardest thing to see in a video. No
value of `align_yaw_deg` can produce a reflection, and a test pins that over
±720°.

**A yaw near 180° is expected and is the point** — you are facing the wearer.

## Step 9. The rest of the measurements, one command each

```
bash scripts/vr_measure_session.sh rate
bash scripts/vr_measure_session.sh freeze
bash scripts/vr_measure_session.sh clutch
bash scripts/vr_measure_session.sh scale
bash scripts/vr_measure_session.sh gripper
bash scripts/vr_measure_session.sh reset
bash scripts/vr_measure_session.sh report
```

Each prints its instruction, records while you do it, and closes when you
press ENTER. Each merges into one report, so stopping half way keeps the half
you did.

**On `freeze`, read the verdict, not the arm.** `/vr/tracking_ok` **defaults
FALSE and is derived from frame ARRIVAL**, so covering one controller while
the headset streams on does not move it — measured with a real Quest: **zero**
transitions at 90 Hz. The step therefore prefers
`/vr/controller_valid_<hand>`, and it reports `was_true_before`. If that is
false the verdict is **NOT MEASURED**, whatever the arm did: an arm that does
not move because the topic was silent and an arm that does not move because
the freeze worked are the same observation.

| | pass |
| --- | --- |
| rate | 72–90 Hz, worst gap well under 200 ms |
| freeze | `was_true_before: true`, then a loss, latency ≤ 0.200 s |
| clutch | worst re-engage jump < 5 mm over three cycles |
| scale | the thumbstick moves scale and the command stays continuous |
| gripper | the gripper follows the trigger and latches |
| reset | references, anchors and filter cleared; scale back to 1.0; the reset counter advanced |

---

# PART C — ONE REAL ARM, AND THE E-STOP, BEFORE ANY TELEOP

**The wearer is out of the rig for all of Part C.**

## Step 10. Post the observer

The observer is **not** you. They stand where they can see the wearer and the
arms and reach the physical e-stop. On their terminal:

```
python3 scripts/observer_estop.py
```

They confirm three things and press ENTER. It then beats at 5 Hz for as long
as they keep confirming.

**PASS:** `observer PRESENT`, and `vr_safety_node` logs `observer e-stop
CONFIRMED present for this session`.

**This is a heartbeat, not a checkbox.** Close the terminal, sleep the laptop
or walk away with it and the beats stop; past 2 s that is treated as
**withdrawal** and the arm freezes. Until 2026-08-20 it was a latch, so a
publisher that simply stopped left the system believing somebody was standing
there with their hand on a button — the one state this interlock exists to
detect was the one state it could not see.

**Do not set `require_observer_estop:=false`.** If you are tempted, the
answer is to find a second person, not a parameter.

## Step 11. Connect ONE arm

```
bash scripts/start_real.sh arm:=left
```

Ten-second countdown, one line per second; Ctrl-C aborts.

**PASS:** the Kortex session opens, homing runs with live per-joint progress,
and it prints `REAL ARMS LIVE`.

## Step 12. THE HOME GAP — expect a refusal, and welcome it

The bridge will very likely **REFUSE to enable**, naming a joint and about
**1.94 rad**.

**That is correct and it is the designed behaviour.** The sim home was moved
to the presentation pose on 2026-08-15 and the physical arms were never
recaptured. The bridge replays sim angles starting at home, so enabling would
command that difference as a jump. It refuses; it does not move.

**You have two honest options.**

**(a) Do not recapture today.** VR on the real arm needs the bridge, so this
ends Part D. Part C still stands and is worth having. This is the right choice
if the day is already long.

**(b) Recapture, deliberately, with the wearer out and a hand on the e-stop.**
Every number is in `docs/NEXT_SESSION.md` §"CAPTURE THE NEW HOME ON THE REAL
ARMS" — Kortex degrees, ROS degrees and ROS radians, both arms. In outline:

1. wearer out of the rig — it is a ~312° total move, worst single joint 111°;
2. one arm at a time;
3. confirm the bridge refuses **before** you start. If it does **not** refuse,
   **stop**: either the arm is already at the new pose or something is reading
   a different home, and both need explaining before anything moves;
4. home with `real_homing_node`. It uses a velocity law, so it is immune to
   the joint_5 seam. The move exceeds its 120° refusal, so it needs
   `allow_large_move:=true` — set deliberately, once, for this session;
5. watch joint_7, the 111° one. Kortex should settle at **265.75**;
6. read the arm back, compare against the table in Kortex degrees, and record
   the live reading in `config/real_home_reference.txt` — no code reads it, so
   it cannot break anything, and it is the only record of what the hardware
   actually did;
7. re-run the enable gate. It must now succeed. If it still refuses, the arm
   is not where the readback says.

**If you abandon half way, say so in the notes.** The arms are then at neither
home. That is safe — the bridge refuses on any gap over 0.05 rad — but the
next person's first refusal will otherwise look like this same known issue and
be waved through.

## Step 13. PROVE THE E-STOP ON THE REAL ARM — before any teleop

**Do this before VR touches the arm. Not after.**

1. Confirm the arm is powered, homed and holding.
2. The observer presses **e + ENTER** at their station — or hits the physical
   button, which is the real test.
3. **PASS:** the arm halts immediately and `/estop_state` latches true. It
   stays halted. `/estop_reset` is the only way out and is a deliberate act.
4. Reset, and prove it a second time from the GUI's red **E-STOP**.

**FAIL — the arm keeps holding but does not halt:** stop the session. Do not
proceed to teleop with an unproven e-stop.

**A note on the order of operations inside the e-stop:** it publishes the halt
before it talks to any driver. `wait_for_service` in that path once blocked
the e-stop for 4.0 s (HARD CONSTRAINT 6). If you add anything to this path,
put the halt first.

---

# PART D — VR DRIVES THE REAL ARM

## Step 14. Arm the real-arm path, explicitly

```
ros2 service call /vr/enable_real_arm std_srvs/srv/Trigger {}
```

**PASS:** `real-arm control enabled for this VR session`.

**FAIL — `REFUSED: no observer e-stop confirmed`:** step 10 is not satisfied.
Look at the observer's terminal; they may have stopped re-confirming.

Also confirm, on the GUI's bottom bar and in `live_monitor`, that
`motion_enabled` has been armed by hand — HARD CONSTRAINT 8 holds the arm
until it is, and it defaults false in `real_robot` mode.

## Step 15. First motion: small, low, and away from the wearer

1. Set scale **low** — thumbstick down until the monitor reads ~0.25.
2. Grip, move your hand **5 cm**, release.
3. Watch the arm, not the screen.

**PASS:** the arm moves in the direction you moved, scaled down, and stops
when you release.

**FAIL — the arm moves the WRONG WAY:** release the grip. Do not adjust
anything by hand. Re-run step 8. A wrong heading is a wrong heading and the
only fix is to measure it again.

**FAIL — the arm does not move at all:** check, in order — the clutch (grip
held?), `/vr/controller_valid_<hand>` (is the controller visible to the
headset?), `/vr/freeze` (has the safety node frozen?), and the follower's
`motion_enabled`.

**FAIL — the arm jumps on engage:** release immediately. The re-engage jump
should be under 5 mm (step 9). A large jump means the anchor is stale — reset
the mapper (`/vr/reset`) and re-engage.

## Step 16. Between runs, reset the mapper

```
ros2 service call /vr/reset std_srvs/srv/Trigger {}
```

Clears references, anchors, the filter and the thumbstick scale.
`align_yaw_deg` is deliberately **not** cleared — it is where you are sitting,
not per-run state.

---

# 17. THE SCENE CAMERA, AND WHAT VR DOES WITHOUT IT

**VR runs without it, unchanged, and says so.** The scene camera measures the
wearer's actual body so the clearance floor is enforced against a person
rather than a mannequin. It is not on the VR command path and its absence is
not a refusal.

The ladder, per part of the body:

| rung | what enforces the floor |
| --- | --- |
| measured | the tracked body, where the camera can see that part |
| **mannequin** | `wearer_posture.wearer_model()` — everywhere else, and everywhere always if there is no camera |

**The camera may only ever make the wearer BIGGER.** `fuse()` keeps whichever
of the tracked and mannequin primitive is closer to the robot, per part. A
camera that says the wearer stepped back changes nothing; a dead camera leaves
exactly today's model.

**Where to look:** `/wearer/enforced` says which body the clearance floor is
using, and the GUI's **Wearer** tab shows it in its top line.

If you want it today: `bash scripts/attach_scene_camera.sh` prints the two
`usbipd` commands. If you do not, do nothing — the arms run on the mannequin,
which is what every clearance figure in this project was measured against
anyway.

---

# 18. WHEN SOMETHING GOES WRONG MID-RUN

| symptom | first thing to check |
| --- | --- |
| arm stops dead, nothing on screen changed | the observer stopped re-confirming — 2 s of silence is a freeze |
| arm frozen, poses still flowing at 90 Hz | the tracking reference was knocked. `/vr/rebase_reference`, then **re-run step 8** |
| one arm frozen, the other fine | that controller is occluded. `/vr/controller_valid_<hand>` |
| everything looks up, nothing responds | step 1c — ask `ros2 node list --no-daemon` |
| the graph half-works | step 1b, then **wait five seconds** |
| the bridge refuses to enable | step 12, and it is telling you the truth |
| the next connect is refused | step 4 — a leaked session |

---

# 19. TEARDOWN, IN THIS ORDER

1. Release the clutch. Do not stop anything while an arm is being driven.
2. `ros2 service call /vr/reset std_srvs/srv/Trigger {}`
3. **Ctrl-C the real bridge, once**, and wait for `kortex session closed
   cleanly`.
4. Stop the VR chain, then the sim stack.
5. Observer stands down last: **q + ENTER**.
6. `srl_clear_stale_shm`, so tomorrow does not start at step 1b.
7. Write down what actually happened, including what you skipped.

---

# 14. WHAT IS UNVERIFIED — READ BEFORE THE ARMS ARE POWERED

*(Numbered 14 because it is the section you should have read at step 1. It is
at the end because it is long.)*

**Nothing in this repository has ever run against a real Kinova arm.**
Everything below has only ever executed against `mock_real_stack`, which is
the real code path with the driver swapped — so the logic is exercised and the
HARDWARE is not. These are ranked by how likely they are to bite you today.

## Very likely

1. **The bridge refuses to enable.** The 1.94 rad home gap. Step 12. This is
   not a surprise, it is the expected state, and it is the single most likely
   thing to end Part D.

2. **The ros2 daemon hangs at least once.** It did twice while this was being
   written. Step 1c.

3. **`arm:=both` aborts the hardware load.** `kortex_driver` exports
   `tcp/twist.*` without an arm prefix, so two real components collide. The
   notes contradict themselves on whether patches 0001-0003 fixed it. Step 3
   settles it with nothing at risk.

4. **The observer e-stop has never been satisfied by anything.** Nothing in
   this repository published `/vr/observer_estop_present` until 2026-08-20;
   `start_vr_wifi.sh` checked for a publisher that could not exist and printed
   a warning. `observer_estop.py` is new and has never run in a lab.

## Likely

5. **Homing speed and behaviour on real joints.** `real_homing_node` uses a
   velocity law with `kp 0.5, ki 0.4, vmax 0.15 rad/s`, tuned against a mock
   whose dynamics are a first-order model. A real Gen3 has gearbox friction, a
   17 kg payload and gravity. Expect the approach to be slower and the settle
   longer; watch for oscillation about the target and stop it if you see any.

6. **The lag trip on a real arm.** `lag_trip_rad` is 0.15–0.50 depending on
   the launch. On the mock the real arm follows within microseconds because it
   IS the command. A real arm lags by its own dynamics, and the first thing
   the trip may catch is normal following error. If it trips immediately on
   enable with no motion commanded, that is the number, not a fault.

7. **The gripper on real Kortex.** `fsr_gripper_node` publishes to
   `/<arm>_gripper_controller/joint_trajectory` and the bridge subscribes over
   the same session. The transport is proven; the real gripper's force,
   travel, latching and what "closed on nothing" reads as are not.

8. **Ctrl-C during homing.** The teardown path is written to stop the arm and
   close the session, and has only ever been exercised against a mock that
   stops instantly. Try it deliberately at step 11, before you need it.

## Possible

9. **The VR yaw at 180°.** `calibrate_operator_yaw.py` is verified at 0/90/180
   through the whole chain — in simulation, with a synthetic controller. It has
   never been run with a human hand and a real Quest. It refuses rather than
   guessing, so the failure mode is "it will not write a number", not "it
   writes the wrong one".

10. **`vr_pose_mapper` HOLDS THE ARMS.** Documented and still open: the mapper
    holds the arms so a staging move cannot execute while it runs. The reset
    half is fixed; this half is not. If a staging move appears to do nothing,
    this is why — stop the mapper for it.

11. **The re-engage jump is not measured on this setup.** The chain test drove
    the target into a region where IK fails (47/64 solves), so the arm sat
    still and every clutch number read 0.000 — degenerate, not good. Step 9's
    `clutch` is the first real measurement of it.

12. **Which way is the wearer's right.** Genuinely open: the docs say world
    +x is the wearer's right, and the arms sit the other way round
    (`left_end_effector_link` at x = +0.117, `right_` at x = −0.160). Either
    +x is the wearer's left, or the arm named `right_` is physically on their
    left. **This is why step 15 says move 5 cm and watch the arm.** The yaw
    calibration measures the heading empirically and does not depend on
    settling this — but the ARM NAMING might still surprise you, so confirm
    which physical arm moves when you drive the one you think you are driving.

13. **The clearance floor against a real person.** Every clearance figure was
    measured against the mannequin. With the wearer out of the rig for Parts C
    and D this does not bite today — but the moment somebody is in the harness
    it does, and the scene camera (§17) is the answer, not a bigger margin.

14. **Two Kortex sessions from a crashed process.** Step 4 covers the clean
    case. A process that segfaults does not close its session, and nothing in
    this repository has ever seen that happen on real hardware.

## Known-unknowable today

15. **Real camera intrinsics and extrinsics**, if you attach the scene camera.
    No camera has ever been attached to this machine. Everything optical is
    unmeasured — `docs/system/19_scene_camera_as_built.md` §6.

16. **Speech.** `/dev/snd` has only `timer`, so no microphone can be opened in
    WSL. Every voice number in this project came from synthesised audio. Do
    not attempt a voice-driven run today.
