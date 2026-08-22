# NEXT SESSION — 2026-08-24

Supersedes `docs/NEXT_SESSION_2026_08_23.md`, which is still the right
description of the workspace, sim-to-real and LeRobot work. This page records
what 2026-08-23 built and what it changed about the plan.

## THE ONE-PARAGRAPH VERSION

**Teleoperation had no motion generator, and now it has one.** What stood in
for one was `clamp_towards`: a per-joint step clamp that moved each joint
independently, so the joints sat at different fractions of their own travel
and the hand left the straight line the IK solution implies by **51.3 mm** —
on a rig whose grasp capture gate is 30 mm. It was also 12.53x over the
velocity limits in `joint_limits.yaml`, which nothing in this repository had
ever read. It is Ruckig now: jerk-limited, phase-synchronised, per-joint
limits, and **0.0 mm** off the commanded path with all seven joints arriving
on one cycle. The change is `srl_teleop/motion_generator.py`, it is pure and
self-tested, `motion_generator:=legacy` reproduces any earlier recording, and
the chooser and the live read-out are both in the window.

---

## 1. WHAT WAS MEASURED

Instrument: `scripts/measure_teleop_motion.py` — validated against `srl_fk`'s
own control and four constructed known answers before it reports anything.
Baseline: `recordings/baselines/teleop_motion.json`.
Case: the left arm's shipped home + `[0.50 -0.30 0.10 0.25 -0.05 0.15 0.02]`
at 50 Hz, which is the case the previous session's diagnosis used. It was
reproduced to the decimal place (135.3 mm / 45.1 mm) before anything changed.

| left arm | off-path max | peak joint speed | arrives over |
| --- | --- | --- | --- |
| `clamp_towards`, as shipped | **51.3 mm** | **12.53x the limit** | 2 cycles |
| **ruckig, the default now** | **0.0 mm** | 1.00x | 1 cycle |
| synchronised clamp (fallback) | 0.0 mm | 1.00x | 1 cycle |

Right arm: 39.2 → 0.0 mm.

**The metric changed, deliberately, and both are in the file.** The diagnosis
metric — where the hand is at cycle k against where it would be if every
joint were at fraction k/N — conflates leaving the path with not being at k/N
along it. A jerk-limited generator is deliberately not at k/N, so that metric
charges it for the feature (it reads 69.8 mm and means nothing). The reported
figure is the distance from each commanded hand position to the CURVE the
straight joint-space line traces: parameterisation-free, and the fault
isolated.

**It was never a tuning value.** Swept against `max_step` the excursion
settles at 68.0 mm rather than tending to zero, because the desynchronisation
is proportional.

## 2. WHAT IS ASSUMED, AND IT IS THE ONE THING TO GO AND MEASURE

`src/srl_moveit_config/config/joint_limits.yaml` declares
`has_acceleration_limits: false` and `max_acceleration: 0` for **all fourteen
joints**. So:

* **velocity limits** — the robot's own, read from that file, never widened;
* **acceleration and jerk** — ASSUMED, derived from two named ramp times
  (`accel_ramp_s` 0.25 s, `jerk_ramp_s` 0.10 s) and labelled `ASSUMED` in
  `Limits.provenance` for as long as the file declares none. A test writes a
  yaml that DOES declare one and requires the label to follow the file.

The assumption is bounded — acceleration and jerk shape how the velocity
limit is approached and can never exceed it — but it is still a guess about
the hardware. **Measuring the real acceleration limit is a lab-day job and it
would remove the only unlabelled unknown in the motion path.**

## 3. WHAT IS UNTESTED ON HARDWARE, RANKED

Nothing in this work ran on a real arm. In the order it is likely to bite:

1. **`time_from_start` is now one cycle, not `travel / max_vel`.** The
   generator has already shaped the motion, and telling the controller to
   take longer than a cycle makes it interpolate part of the way before the
   next point replaces it. Correct in principle; unobserved on a controller.
2. **The arming ramp changed mechanism.** It used to stretch
   `time_from_start`; it now LOWERS the generator's velocity limit, which
   also makes the ramp jerk-limited. Same intent, different lever.
3. **`generator_resync_rad` (0.35 rad) has never met a real tracking error.**
   The generator integrates its own output; it snaps back to the measured arm
   past this. Too tight and it fights the controller every cycle and becomes
   the old clamp; too loose and a genuine desync persists. Watch
   `/ik_status_<arm>[16]`, which counts resyncs, and the `[MOTION]` stats
   line.
4. **The cascade and `real_robot` caps now reach the generator.** They used
   to cap `max_step` only. Verify the real arm is actually slower, not just
   told it is.

## 3b. THE ACCURACY PASS, AND THE RECORDING PIPELINE IT UNBLOCKED

Full account: `docs/system/findings.md`, 2026-08-23 (later).

    RSS of the measured terms   18.64 mm -> 12.91 mm
    WORST CASE (they add)       33.39 mm -> 19.99 mm   against a 30 mm gate

**The pad midpoint is a curve, not a constant.** The Robotiq's fingers swing
on a four-bar, so the wrist-to-pad distance depends on the opening: 0.09833 m
wide open, 0.10976 on a 40 mm cube, 0.11179 on a 20 mm one. Both numbers this
repository has argued about are points on it. T1's grasp was built on the
open-hand value, 11.43 mm short. `pad_mid_ee_for(width_mm)` is the table and
`srl_fk` can place the fingers offline now.

**Staging went to a sixth copy of the home pose** —
`recordings/baselines/presentation_pose.json`, written before home became the
presentation pose, 3.06 rad away on joint_4. `require_home()` refused every
run, so **the recording sweep had been unable to record a single cell since
that check was added**. Staging goes to `config/home_positions_*.txt` now.

**Six verifiers were asking a question the robot does not ask** — the home
wrist instead of the anchor (42.9 deg apart), and `exact` instead of the
follower's 15 deg cone. T1 stage 1 went 32 IK failures to 0 without anything
about the robot changing.

### WHAT IS STILL OPEN FROM IT

1. **T1's exact grasp pose is refused by T1's own table.** It solves 10/10
   with no furniture. The follower's cone supplies 5.0 deg, costing 9.58 mm of
   the 30 mm gate. Either move T1's table or accept and record the tilt.
2. **Dance d2 and d3 have 68 unreachable waypoints** at the anchor with the
   cone (d1 is clean). `verify_dance_paths` refuses to certify them for
   filming, correctly.
3. **`camera_link` vs the physical module is UNMEASURED** and is now the
   largest unknown in the budget. There is no CAD of the Kinova gripper here.
4. **~18 other scripts still read `ee_quat()` and call it the anchor.** Two
   were fixed; the rest produce figures at a wrist no task commands. Sweeping
   them is a session on its own and every number they have produced since the
   home changed is suspect until then.

## 4. WHAT TO DO NEXT, IN ORDER

Unchanged from `docs/system/24_motion_planning.md` except that item 1 is done:

1. ~~Wire Ruckig into the follower~~ — **done**.
2. **Record a free-space wrench baseline on each arm** — one minute of motion
   — and turn contact detection on. `srl_teleop/arm_telemetry.py` is built
   and tested; until a baseline exists `in_contact` compares against a fixed
   gate and its own reason string says the gate is uncalibrated.
3. **Use contact to ABORT a grasp** rather than driving through it.
4. **Feed the `VoxelWorld` from the wrist camera in the live pick path.**
5. **Solve the scene-camera extrinsic** (re-capture with the arms stowed).
6. **Evaluate MoveIt Servo** against the follower — now a fairer comparison,
   because the follower is no longer a naive step clamp.
7. **Revisit cuRobo if the GPU changes.**

And, new, from this session: **measure the arms' real acceleration limits**
and fill in `joint_limits.yaml`, so the motion path has no assumed number
left in it.

## 5. GATES, AS LEFT

```
python3 scripts/check_tests.py          1216 passed, 2 failed (both allowed), 0 NEW
python3 scripts/verify_gui_buttons.py   268 checks, 0 failed
python3 -m srl_teleop.dependency_check  0 problems
.venv_vision/bin/python scripts/real_calibration/check_all.py   4 of 4
```

`ruckig` is in the **system user site**, deliberately, because the followers
run in the system interpreter:

```
pip install --user --no-deps --break-system-packages ruckig
```

It has no dependencies at all — one compiled `.so` — so it cannot repeat the
`.venv_vision` numpy incident. `dependency_check` carries a row for it, and
without it the follower degrades to the synchronised clamp and says so rather
than silently returning to the per-joint one.

## 6. WHAT IS STILL TRUE AND UNCHANGED

* **Nothing in this repository has ever run against a real arm.**
* HARD CONSTRAINT 0 stands: the real arms are at the legacy Kortex home and
  the presentation pose has never been captured on them.
* 7 of 14 master channels were INCOHERENT at the last channel check.
* All the standing traps in `NEXT_SESSION_2026_08_23.md` still apply.
