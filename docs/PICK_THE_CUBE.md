# Picking the cube with the left arm

Two commands. Nothing is hand-solved; no saved pose is required.

```bash
bash scripts/bringup_arm.sh left      # bridge + gripper camera, idempotent
python3 scripts/srl_pick_cube.py      # find viewpoint, measure, servo, grasp, lift, verify
```

Add `--dry-run` to the second to find the viewpoint and measure the cube
without touching it.

Everything that talks to the arm must be on the right domain. `bringup_arm.sh`
sets it internally; if you run a script by hand:

```bash
export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=SHM
export PYTHONPATH=~/kortex_ws/scripts:$PYTHONPATH
```

**`ROS_DOMAIN_ID=7`, not the usual 0.** Domain 0's FastDDS shared memory is
polluted and no *new* process can join it — proved with a minimal pub/sub: 0 of
3 messages on domain 0, 3 of 3 on domain 7. Existing nodes keep working, which
is what makes it so confusing. Getting this wrong publishes into the void and
moves nothing, silently. Clearing it needs `/dev/shm/fastrtps_*` removed with
the whole stack stopped (HARD CONSTRAINT 5).

## What the pipeline does

| step | script | what it does |
| --- | --- | --- |
| bring-up | `bringup_arm.sh` | one Kortex session, camera on the URDF frames |
| viewpoint | `auto_observe.py` | searches for a pose that actually SEES the cube |
| measure | `servo_pick_left.measure` | table plane + cube, in the camera's own frame |
| grasp | `srl_pick_cube.py` | servos the pads onto the cube, closes, lifts, verifies |

## The seven things that make it work

These are all failures that happened, each of which produced a confident wrong
answer before it was found.

1. **The camera extrinsic is wrong by ~8.5°, and it rotates with the wrist.**
   The table cannot move, yet the table normal measured from four arm poses
   spanned **8.45°** — 34 mm of error at 0.23 m, 59 mm at 0.40 m. So the grasp
   is *not* planned open-loop through that transform. The pad-to-cube error is
   measured in the camera frame, where the data is good (0.54 mm plane RMS, a
   40 mm cube measured at 40.9 mm), and nulled by iteration. **This is the one
   that matters most**: closed loop converges through a wrong extrinsic, open
   loop cannot, and did not.

2. **The pad midpoint moves when the hand closes.** The Robotiq's fingers swing
   on a four-bar: wrist-to-pad is 0.09833 m open, 0.10976 m closed on a 40 mm
   cube. Solving with the hand open and then closing drives the pads **11.43 mm**
   further along the tool axis — into the table. Poses are solved at
   `CUBE_GRIP`. (docs/ENGINEERING_LOG.md already records this as "T1's grasp was built
   11.43 mm short".)

3. **The wrist camera cannot see its own grasp.** The last ~80 mm pushes the
   cube out of the bottom of the frame. That is geometry, not failure: the last
   good fix is carried in world and the remainder completed from it.

4. **Joints 3, 5, 7 are continuous.** −179.05° and +181.02° are the same pose;
   subtracting gives 360.08°. That "error" aborted a run that had *arrived*.
   All joint arithmetic goes through `ang_wrap`.

5. **A publish loop starves its own subscription.** Spinning with `timeout 0`
   while publishing at 20 Hz let the cached joint vector go seconds stale; a
   motion check read 0.032° on a move that was really running and aborted it.
   Motion is judged only through `fresh_q()`.

6. **Arrival is waited for, not timed.** The bridge closes error proportionally
   (kp = 0.5, ~2 s time constant), so a fixed 2.5 s settle leaves a third of the
   error standing and reports a miss.

7. **Paths are checked against the table over the whole hand.** A straight line
   in joint space says nothing about where the hand goes — that is how the
   gripper was driven into the table.

## Known-bad instruments — do not trust these

* **The wearer clearance check is not usable.** Asked about the pose in which
  the arm was physically against the mannequin, `ClearanceModel` returned
  **367.0 mm — the same value it returns for a visibly clear pose.** It samples
  link *origins* only, so it misses the arm's body between joints. Until it
  measures link *geometry*, treat a "pass" from it as meaningless. This is the
  outstanding safety gap; HARD CONSTRAINT 11 is currently unenforced in code.
* **`/real_status_*` can be a frozen republish.** It is published by
  `real_homing_node`, not the bridge. When its input dies it keeps publishing
  the last snapshot with fresh timestamps. A pose read from it matched a saved
  file exactly while the arm was somewhere else. Key on **distinct source
  stamps**, which is why every capture here reports them.
* **`in_contact` / `force_n` has no free-space baseline**, so it cannot tell
  contact from gravity and payload. It read 25–41 N with the arm in free space.

## When it goes wrong

| symptom | cause |
| --- | --- |
| preflight: `arm_subs=0`, no joint states | wrong `ROS_DOMAIN_ID`, or the bridge died |
| bridge: `Broken pipe` / `read failed` | the Kortex session dropped; `bringup_arm.sh left --restart` |
| bridge: `Connection timed out` while the arm pings | a previous session leaked; wait ~30 s, retry `--restart` |
| arm pings 100% loss | the arm is off the network — power/cabling, not software |
| cube not found by the scan | it is outside the reachable viewpoints; move it toward the arm |
| `MISSED` at step 5/5 | the cube stayed on the table; re-run, the servo starts from wherever it is |

Never `SIGKILL` a bridge: it leaks the Kortex session and the next run cannot
connect at all, while the arm still pings and its API port still answers
(HARD CONSTRAINT 2). `bringup_arm.sh` refuses to do it.

---

# Scanning the table with both arms

```bash
bash scripts/bringup_arm.sh left
bash scripts/bringup_arm.sh right
python3 scripts/scan_table.py              # -> recordings/baselines/table_model.json
```

## Camera-in-hand vs camera-for-hand, and which job each does

**Camera-in-hand (eye-in-hand)** — the gripper cameras. The camera rides on the
wrist, so its view is dense and close, but every measurement needs the hand-eye
transform to become a robot coordinate, and ours is wrong by 8.45°. It also
loses the target in the last ~80 mm of a grasp.

**Camera-for-hand (eye-to-hand)** — a fixed camera watching the workspace. It
sees the object *and* the gripper, so you can servo on their relative error and
calibration largely cancels. We do not have a usable one: `/scene_camera` is
colour-only with an all-zero `K`.

So we get the eye-to-hand benefit a different way. The pads and the camera are
**one rigid body**: their relative pose is FK down a single chain and never
touches the mount error. That makes

```
gap = (pad midpoint in the CAMERA frame) · n_camera + d_camera
```

exact regardless of calibration. `srl_scene`'s self-test proves it: injecting an
8.45° frame error leaves the reported height at 149.9 mm, unchanged.

**The division of labour:**

| question | use | carries mount error? |
| --- | --- | --- |
| what is on the table, roughly where? | `scan_table.py` object poses in world | yes — that is the error bar |
| how do I get near it? | IK to the object pose | yes, but closed-loop servoing absorbs it |
| **when do I stop descending?** | **`srl_scene.pad_height_above`** | **no — this is the safe one** |

## The modules

| script | job | self-test |
| --- | --- | --- |
| `srl_scene.py` | plane fit, object segmentation, extrinsic-free pad height | `--self-test` |
| `handeye_from_table.py` | recovers the mount-rotation error from the table | `--self-test` |
| `guarded_descent.py` | closed-loop descent that stops on a measured gap | `--self-test` |
| `scan_table.py` | both arms, many views, fused table + object model | on hardware |

All three self-tests pass on constructed geometry, where the answer is known:

* plane normal recovered to **0.01°**, offset to **0.15 mm**
* a 40 mm and a 60 mm cube recovered at 41.2 mm and 61.2 mm
* a **known 8.45° mount error recovered to 0.09°** from six simulated views,
  collapsing view disagreement from 15.75° to 0.31°
* descent converges to a 20 mm target **through a 12% scale error and a 6 mm
  per-step bias**, and refuses to move when it cannot see the surface

## Why the descent used to go into the table

It was open-loop. The stopping height was computed in robot coordinates through
the bad mount, and nothing re-measured on the way down, so the error was only
discovered by contact. `guarded_descent` re-measures every step, stops on the
measured gap rather than a commanded pose, and raises `Blind` rather than
descending when the surface is not visible.

One control detail worth knowing: a plain proportional step **cannot** converge
against a constant execution bias — the step shrinks as the gap closes until the
bias exactly cancels it, and the descent parks short. The self-test reproduces
that (stalls at 26.8 mm), which is why the gain adapts when a step fails to
deliver the motion it asked for.
