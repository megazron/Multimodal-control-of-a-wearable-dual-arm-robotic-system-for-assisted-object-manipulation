# Findings and diagnostic history

Every measurement, wrong turn and worked example, in the order it happened. Split out of CLAUDE.md on 2026-08-12 because it was 98k tokens and 10% of a session's context before any work started. NOTHING HERE WAS DELETED. Sections marked SUPERSEDED are kept for their diagnosis, not their conclusion.

CLAUDE.md carries the one-line rules; this file carries why each one exists.

# Historical log

Everything below is the original log, kept in chronological order. The
`teleop.launch.py` invocations in it are still correct — `scripts/run_teleop.sh`
is a thin wrapper around exactly that launch.

## Does the robot actually FOLLOW? — not yet verified under motion

The gain matrices above are `d(command)/d(master)`: they measure the
**software mapping**, not whether the arm moves. Measured from
`teleop_20260731_125706.csv`, the one recording with real EE and a moving
master (right arm, 2097 samples, old settings):

```
   d(COMMAND)/d(master)          d(ACTUAL ROBOT EE)/d(master)
     +1.034  -0.009  -0.044        +0.100  -0.040  -0.114
     -0.001  -0.989  -0.001        -0.018  +0.014  +0.021
     -0.047  +0.060  -0.920        -0.038  +0.003  +0.050
   tracking |cmd-actual|: RMS 0.0396 m, max 0.2956 m
```

The command swept correctly; **the robot barely moved** (EE gain ~0.1 or
less). So "commanded pose is a good proxy for EE" is **false under motion** —
the earlier sub-millimetre tracking figures were taken with a *stationary*
master and prove nothing about tracking.

That recording predates the current fixes (it ran with `anchored`
orientation, where IK was failing 90%+ of the time, so the follower had
little to publish).

**Re-measured 2026-07-31 with the current settings and real tf2 EE, 40 s:**

| arm | IK success | GUARD | tracking RMS / max |
| --- | --- | --- | --- |
| left | 99.9% (1914/1915) | 1914 published, all direct, 0 rejected | 0.0004 m / 0.0015 m |
| right | 100% (1824/1824) | 1824 published, all direct, 0 rejected | 0.0000 m / 0.0000 m |

So the robot **does** reproduce the command now. But the master was at rest,
so this proves the loop is closed and stable, **not** that it tracks a moving
input. The right arm's 0.00000 m is trivially perfect because its command
never changed. **A gain matrix against real EE still requires a capture with
the master actually moving** — that is the one measurement nobody has taken.

## NOT done — home-pose optimisation

The home poses are still the originals (left 150/-94/-53/-65/-123/41/121,
right 62/-109/-93/34/-175/-31/2) and are **0.373 m from mirror symmetry**.
Optimising them was not attempted this pass: it requires sampling IK success
across a task volume for many candidate poses at ~50 ms per call, and the
prerequisite measurement (does the robot track a *moving* master) is still
outstanding. Because `offset = P_HOME`, changing home also forces re-measuring
the anchor and re-deriving the orientation lock — so it should be done in one
deliberate pass, not bolted on.

---

# Final build (2026-07-31, later session)

## Collision safety — the wearer is now in the collision model

**This was the important one.** `avoid_collisions=True` was already set on the
IK request, but `human_backpack.xacro` carried **27 `<visual>` and 1
`<collision>`**, and no human link appeared anywhere in the SRDF. MoveIt
cannot avoid what it cannot see, so the flag was doing nothing for the
operator.

Added `<collision>` to torso, head, hips, both legs, both human arms and
hands (13 collision elements). Paired with **36 SRDF exclusions** for the
mount-adjacent proximal links (`{left,right}_{base_link, shoulder_link,
half_arm_1_link}` vs torso/harness/backpack/mount_plate/mount pads) — the arm
bases are bolted to the harness and are permanently adjacent, so without
those exclusions every pose is trivially "in collision". **Everything distal
of `half_arm_1` is left enabled** — that is what must actually avoid the
wearer.

**Redundancy search.** The Gen3 has one redundant DOF: `joint_3` is the
upper-arm roll, and turning it swings the ELBOW around the shoulder-wrist
axis without moving the gripper. On IK failure (which now includes collision
failures) the follower re-seeds `REDUNDANT_IDX` and retries up to
`redundancy_samples` (6) times, fanning out either side of the natural
posture. Live, the left arm used **163 retries in one window** — poses that
would previously have been dropped.

**Hard floor.** `min_clearance_m` = 0.05. Before publishing, clearance from
the arm's distal links to the wearer is measured from **TF** (where the robot
really is, not where it was asked to go) via `clearance.py`, using capsule/box
primitives taken straight from the xacro. Under the floor: hold, log, publish
nothing. Reported in `/ik_status_<arm>` fields 6–8 (min clearance, blocks,
redundancy retries). TF unavailable returns `inf` and does NOT block — an
interlock that fires on missing data would make the arm undriveable.

## Right gripper — root-caused and FIXED

`right_robotiq_85_left_knuckle_joint/position` was `[unavailable]
[unclaimed]`. The URDF was correct — both gripper blocks declared the command
interface identically. The fault was in the vendor macro
`robotiq_description/urdf/2f_85.ros2_control.xacro`:

```xml
<gpio name="reactivate_gripper">      <!-- UNPREFIXED -->
```

On a dual-arm robot both grippers instantiate this macro, so both hardware
components registered the same GPIO interface names. The second to load
(right) then exported **zero** command interfaces — visible as an empty
"command interfaces" list in `list_hardware_components -v` while the left had
both the knuckle and the GPIO. The earlier vendor patch prefixed the hardware
*name* but missed the GPIO. Fixed to `${prefix}reactivate_gripper`.

After the fix: all five controllers active, both knuckle interfaces
`[available] [claimed]`, and both grippers track commanded positions exactly
(0.60 / 0.00 rad).

## Clutch

Button mapping **measured**: left → `btn2`, right → `btn1`, each gating only
its own arm. Three glitch fixes, all three causes:
`clutch_debounce_s` 0.05 (bounce double-toggle), a **quasi-static gate**
(`clutch_static_m` 2 mm over `clutch_static_window_s` 100 ms) so a moving
reference is never latched, an **averaged reference** over
`clutch_ref_frames` 5, and **EMA reset** (`tip_filt = None`) on engage so no
pre-disengage state blends into the first frames.

## Gripper latch

`latch_close_counts` **1200**, `latch_open_counts` **400**,
`latch_release_hold_s` 0.5. Rationale from the measured ranges (rest 6–191,
squeezed 3603/3842): 1200 is ~33% of full squeeze — a deliberate grasp, well
clear of the 250 deadband — while 400 is just above the rest ceiling, so a
tiring grip sagging toward mid-range does **not** release. Proportional
control still applies on the way in; latched grip holds the firmest value
seen. Latch state is independent of the clutch, so freezing the arm never
opens the hand.

## real_robot mode

`ros2 launch srl_teleop teleop.launch.py real_robot:=true` — caps
`max_vel_rad_s` to 0.15 (from 0.6), raises the clearance floor to 0.10 m
(from 0.05), sets `motion_enabled=false` so nothing moves until
`ros2 param set /ik_follower_<arm> motion_enabled true`, and **refuses to
start** if a continuous joint is wound beyond ±π (an unwind is a large
unattended motion with a person wearing the rig). Sim defaults unchanged.

## Final measured state — clean single stack

```
                     LEFT                      RIGHT
IK success           100.0% (2201/2201)        100.0% (2138/2138)
GUARD                2201 direct, 0 rejected   2138 direct, 0 rejected
min clearance        0.367 m, 0 blocks         0.222 m, 0 blocks
tracking vs tf2      RMS 0.42 mm, max 2.01 mm  RMS 0.38 mm, max 1.43 mm
dropouts             none                      j3/j5/j7 (dead, unused)
```

Axes: **up/down and fore/aft track correctly on both arms. Lateral does
not** — unchanged, and still limited by azimuth-from-j1 (see AZIMUTH
LIMITATION). Gyro azimuth stays disabled (`azimuth_mode` = `j1`); the
validating capture with a moving master still does not exist, and the right
gyro drifts 44 °/min regardless.

## STILL NOT DONE — home pose / vision orientation

The home poses remain the originals and are still 0.373 m from mirror
symmetry. Not attempted again this pass: it needs IK sampling across a
task volume for many candidate poses, and because `offset = P_HOME` it also
forces re-measuring the anchor and re-deriving the locked orientation. It
should be one deliberate pass, and it should come after the moving-master
measurement, since that is what decides whether the mapping is trustworthy
enough to build a grasp pose on.

---

# Real-robot bring-up build (2026-07-31, final)

See `docs/REAL_ROBOT_BRINGUP.md` for the staged checklist to follow when the
arms arrive. Real arms: LEFT 192.168.1.10, RIGHT 192.168.1.9.

## Home pose — right arm re-derived, left kept

**The task volume in the brief is not reachable.** x −0.25..0.25, y 0.40..0.70,
z 0.75..0.95 measures **5.6%** IK with collisions and **11.1%** with them off,
so it is a REACH limit, not the wearer: a back-mounted arm at z=1.25 cannot
work a table at z=0.75..0.95 across the body centreline. The same
0.5×0.3×0.2 m box moved to the arm's OWN side at chest height
(left x 0.05..0.55, y 0.25..0.55, z 1.00..1.20; right mirrored) measures
**77.8–100%**. All home scoring uses that volume.

**LEFT home unchanged** — it already scores 94.4% IK, 0.349 m home clearance,
44.1° headroom, 0.52 rad seam margin. Changing it to satisfy a "all four
files must change" rule would have made it worse.

**RIGHT home replaced.** Old: seam margin **0.087 rad** — joint_5 sat 5° from
the ±π seam, where a wrap executes as a 358° rotation; and headroom 29.1°,
under the 30° target. New home is the LEFT pose mirrored about x=0 (mirror
the POSE and solve IK, not negate joints — both arms use the same
non-mirrored Gen3 macro):

| | old right | new right |
| --- | --- | --- |
| joint angles (deg) | 62, −109, −93, 34, −175, −31, 2 | 28.9, −103.3, −107.3, 62.0, −78.9, 53.6, 66.4 |
| IK over volume | 88.9% | **100.0%** |
| seam margin | 0.087 rad | **1.268 rad** |
| headroom | 29.1° | **34.8°** |
| manipulability | 0.1335 | 0.1272 |
| home clearance | 0.549 m | 0.349 m |
| P_HOME | (−0.220, 0.657, 1.254) | (−0.363, 0.407, 1.017) |
| \|right − mirror(left)\| | **0.373 m** | **0.000 m** |

Changed in `config/home_positions_right.txt`, `srl_dual.urdf.xacro` line 53,
and `WORKSPACE_CENTRE["right"]` (offset = P_HOME). Left's three equivalents
are deliberately unchanged.

## Second unprefixed-resource collision found

After the `reactivate_gripper` GPIO fix, one more would have bitten on real
hardware: **both real Robotiq drivers defaulted to `COM_port=/dev/ttyUSB0`**
and would have fought over one device. Added `left_gripper_com_port` /
`right_gripper_com_port` xacro args (defaults `ttyUSB0` / `ttyUSB1`).
A full scan of `<gpio>`, `<joint>` and `<sensor>` names across all four
ros2_control components now shows **no duplicates**.

## E-stop (`estop_node`, started by the launch)

Three independent triggers, any of which **latches** until `/estop_reset`:
`/estop` service or Bool topic; both master buttons within 0.4 s; and a
**dead-man** on `/master_arm_pose_*` going stale past `stale_timeout_s`
(0.5 s). Halting publishes each arm's *measured* joint positions as a short
trajectory, republished while latched, so a follower that keeps commanding is
continuously overridden. The followers also subscribe to `/estop_state` and
refuse to publish while it is true.

**It does not depend on the master arm working** — the service path and the
dead-man both function with the Teensy unplugged, and the dead-man fires
*because* of it. Verified: trips, stays latched across seconds, clears only
on reset, and the followers logged the override.

## real_robot limits

| | sim | real_robot |
| --- | --- | --- |
| `max_vel_rad_s` | 0.6 | **0.10** |
| `max_step_rad` | 0.35 | **0.10** |
| clearance floor | 0.05 m | **0.12 m** |
| wearer padding | 0 | **0.05 m** |
| motion | on | **must arm `motion_enabled`** |
| wound-up arm | unwinds | **refuses to start** |
| first 5 s after arming | full | **25% of cap** |

`time_from_start` is stretched by the reduced velocity, so trajectories are
genuinely slower rather than clamped. `real_arms` (description: real drivers)
is a SEPARATE argument from `real_robot` (follower limits) on purpose — you
want the slow, strict follower while still on mock hardware during bring-up.

## Final sim state

```
                  LEFT                       RIGHT
IK success        100.0% (1983/1983)         100.0% (1930/1930)
GUARD             1983 direct, 0 rejected    1930 direct, 0 rejected
min clearance     0.387 m, 0 blocks          0.251 m, 0 blocks
tracking RMS      0.40 mm (max 1.82 mm)      0.39 mm (max 2.39 mm)
dropouts          none                       j3/j5/j6/j7 (dead, unused)
```

## NOT done: orientation "tilt" mode

Not implemented. It needs an `OrientationConstraint` path through
`/compute_ik` plus a fixed-vs-tilt IK comparison, and I ran out of room after
the home-pose work. `orientation_mode` stays `fixed`. The scaffolding it
would need already exists: the home-pose scorer measures IK with yaw free by
sampling yaw, which is the same question a ~π yaw tolerance asks.

---

# Pre-hardware pass (2026-07-31, final)

## Can these arms reach a table? NO — mechanical finding

LEFT arm, 0.5×0.3×0.2 m volume on its own side (x 0.05..0.55, y 0.25..0.55),
top-down grasp with **yaw free**, mount at (+0.15, −0.12, 1.25):

| z centre | IK % | collisions off |
| --- | --- | --- |
| 1.10 | 77.8 | 83.3 |
| 1.05 | 72.2 | 77.8 |
| 1.00 | 66.7 | 72.2 |
| 0.95 | 61.1 | 61.1 |
| 0.90 | 44.4 | 50.0 |
| 0.85 | 38.9 | 44.4 |
| 0.80 | 33.3 | 38.9 |
| 0.75 | 16.7 | 22.2 |
| 0.70 | 16.7 | 16.7 |

**No height reaches 80%.** Best is 77.8% at chest height (1.10 m); table
height (0.75) is **16.7%**. Turning collisions off barely moves it, so this
is **reach geometry, not the wearer** — the mount sits at z=1.25 and a table
at 0.75 is 0.5 m below and 0.5 m forward, which the Gen3 can span by distance
(0.74 m vs ~0.90 m reach) but not with a top-down wrist inside joint limits.

**This is mechanical, not software.** Options, in increasing order of effort:
1. **Wearer sits or leans forward** — cheapest, effectively lowers the mount
   relative to the table. Leaning ~20° forward moves the mount ~0.25 m down
   and forward, which from the table above should recover most of the gap.
2. **Pitch the mounts further down.** They are currently `rpy 0 ±0.6 0`
   (34.4°). Steeper pitch aims the workspace downward. UNTESTED — it needs a
   URDF change and a move_group restart per angle.
3. **Lower the mount on the harness.** Largest gain, largest mechanical
   change, and it competes with the wearer's own shoulders.

## Posture bias (Task 2c) — shipped, w = 0.02

`solve_type: Distance` seeds from the live joint state, so the arm keeps
whatever IK branch it drifted into and never returns. The seed (NOT the
solution) is now blended toward the nominal home:
`seed = current + w·angdiff(home, current)`. Only the seed moves, so the
solution still has to hit the commanded pose exactly — this steers *which*
7-DOF solution is chosen and cannot move the gripper off target.

**Drift check (the thing that would make it worse than the problem):** master
held still for 30 s, max EE displacement **0.00000 m on both arms** — well
inside the 1 mm limit.

## Orientation "tilt" — implemented, NOT shipped

`orientation_mode: tilt` adds roll+pitch from the IMU gravity vector, holds
the last tilt while accelerating, and leaves yaw uncommanded. Measured over
135 poses spanning the master ball with ±0.3 rad roll/pitch:

| arm | exact pose (fixed) | OrientationConstraint 0.2/0.2/π |
| --- | --- | --- |
| left | 79.3% | **79.3%** |
| right | 68.9% | **68.9%** |

**Identical to the digit** — this MoveIt `/compute_ik` setup **ignores the
`constraints` field** entirely; the plugin is not constraint-aware. So the
OrientationConstraint route does not work here, and both figures are below
the 90% bar. **`orientation_mode` stays `fixed`.** Making tilt viable needs
either a constraint-aware IK plugin or explicit yaw sampling in the follower.

## Final sim state

```
              IK        GUARD                     clearance   tracking RMS
left    100.0% (1472)   1472 direct, 0 rejected   0.374 m     0.40 mm
right   100.0% (1441)   1441 direct, 0 rejected   0.243 m     0.48 mm
```
Exactly one move_group, RViz, master_pose_node, e-stop and FSR node; two
followers (`ik_follower_left`, `ik_follower_right`).

## NOT done this pass

**Tasks 4, 5, 6, 7 — the cascade architecture** (namespaced `/real` stack,
`sim_to_real_bridge.py`, sim rate-limiting to the real cap, lag monitor,
cascade-specific e-stop paths, home-pose match check, and the bring-up doc
rewrite) were **not started**. `docs/REAL_ROBOT_BRINGUP.md` remains the v1
single-stack version and does NOT describe the cascade.

Task 5's slower limits (0.05 rad/s etc.) were also not applied — `real_robot`
still carries the previous 0.10 rad/s / 0.12 m values.

---

# Pre-hardware pass 2 (2026-07-31)

## Table reachability: mount ANGLE is not the answer

Swept extra mount pitch without touching the URDF (rotating the base by δ is
exactly equivalent to rotating targets by −δ about the mount origin, so the
running move_group could be reused). LEFT arm, table volume centred z=0.80,
chest volume z=1.10:

| extra pitch δ | absolute pitch | table % | chest % | binding constraint at table height |
| --- | --- | --- | --- | --- |
| −1.20 | −34.4° | 5.6 | 50.0 | JOINT/IK 13, ORIENT 2, COLL 1, REACH 1 |
| −0.90 | −17.2° | 0.0 | 61.1 | JOINT/IK 13, ORIENT 2, COLL 2, REACH 1 |
| −0.60 | 0.0° | 11.1 | 72.2 | JOINT/IK 12, ORIENT 1, COLL 2, REACH 1 |
| −0.30 | +17.2° | 16.7 | 66.7 | JOINT/IK 11, ORIENT 2, COLL 1, REACH 1 |
| **0.00** | **+34.4° (current)** | **33.3** | **77.8** | JOINT/IK 9, ORIENT 1, COLL 1, REACH 1 |
| **+0.30** | **+51.6°** | **44.4** | **83.3** | JOINT/IK 6, ORIENT 3, REACH 1 |

**Tilting the mount DOWN makes it worse, not better.** The arm is on the
wearer's back; aiming the base downward points it into the body, so the arm
must reach even further around.

**THE BINDING CONSTRAINT IS JOINT-LIMIT / KINEMATIC FEASIBILITY**, not reach.
Of 18 table-height samples at the current angle: **JOINT/IK 9**, orientation
1, collision 1, **reach only 1**. The Gen3 spans the distance easily (0.74 m
required vs 0.902 m reach) but cannot *fold* into the required configuration
inside the ±2.41 / ±2.66 / ±2.23 rad limits on joints 2/4/6. No mount
orientation fixes that, because orientation does not change how far the arm
has to fold — only the target's *position relative to the mount* does.

So the actionable fixes are the ones that move the target relative to the
mount: **the wearer sitting or leaning forward**, or **relocating the mount
lower/more lateral** — not re-angling it. The one angle worth noting is
**+0.3 rad (51.6°), which improves BOTH table (33.3→44.4%) and chest
(77.8→83.3%)** — still far short of 80% at table height. NOT APPLIED, as
instructed; applying it would invalidate P_HOME, the anchor and every
clearance figure.

## Continuous reachability replaces pointwise IK — and the right home FAILS it

26 directions, 1 cm steps, each step seeded from the previous solution
exactly as `ik_follower_node` does, over the 0.22 m master ball:

| | LEFT | RIGHT |
| --- | --- | --- |
| reach min / median / max | 0.11 / 0.22 / 0.22 m | 0.07 / 0.22 / 0.22 m |
| isotropy (min/max) | **0.50** (FAIL, aim >0.5) | **0.32** (FAIL) |
| directions reaching the full ball | 19/26 = **73%** (PASS) | 15/26 = **58%** (FAIL) |
| bounded by | IK-failure 7, step 0, full 19 | IK-failure 10, step 1, full 15 |

**This vindicates the objection to pointwise scoring.** The current right home
was chosen because it scored **100% pointwise**, and it is the **worst** of
the two on the metric that actually matters — 58% usable, isotropy 0.32.
Pointwise IK asks "could the arm get there from anywhere?"; teleop asks "can
it get there from where it already is, without a branch switch". Only the
second is a usable workspace.

**Re-optimising the right home against this metric was NOT done** — see below.

## Posture bias — shipped, w = 0.02, drift verified

Seed (not solution) blended toward nominal home:
`seed = current + w·angdiff(home, current)`. Master held still 30 s: max EE
displacement **0.00000 m on both arms**, inside the 1 mm limit.

## Tilt orientation — implemented, NOT shipped

`OrientationConstraint` (0.2/0.2/π) gave **exactly the same** IK success as an
exact pose — left 79.3% both ways, right 68.9% both ways, over 270 calls.
This `/compute_ik` setup **ignores the `constraints` field**; the plugin is
not constraint-aware. Both below the 90% bar, so `orientation_mode` stays
`fixed`. Viable tilt needs a constraint-aware IK plugin or explicit yaw
sampling inside the follower.

## NOT done

- **Task C(b)** — re-optimising the right home against continuous
  reachability. It fails (58% / 0.32) and should be redone against this
  metric, not pointwise.
- **Tasks E, F, G, H** — the whole cascade architecture (namespaced `/real`
  stack, `sim_to_real_bridge.py`, sim rate-limiting to the real cap, lag
  monitor, cascade e-stop paths, home-pose match check) and the bring-up doc
  rewrite. `docs/REAL_ROBOT_BRINGUP.md` is still the v1 single-stack version.
- **Task F** slow-motion values (0.05 rad/s, 30 s ramp, independent
  displacement ceiling) — `real_robot` still carries 0.10 rad/s / 0.12 m.

---

## 2026-07-31 (later): homes unified, velocity homing, gated cascade

### The home is now the LEGACY REAL home, in all four places
`config/home_positions_{left,right}.txt` and both `initial_positions` in
`srl_dual.urdf.xacro`. Reason: the bridge replays SIM joint angles onto the
real arm, so if the two homes differ, enabling the bridge commands the
difference **as a jump**. They must be the same pose.

    LEFT  Kortex 259.03 277.69 267.74 286.14 194.10 27.48  55.26
    RIGHT Kortex 303.65  77.06  98.57  58.57 317.14 36.39 154.71

All 42 values verified self-consistent through `kortex_convention`, and every
limited joint (2/4/6) is inside its hard stop.

**Cost of the change, measured, not assumed:**

| | left | right |
|---|---|---|
| P_HOME | (0.814, −0.492, 1.232) | (0.173, 0.117, 1.987) |
| IK over ±0.15 m volume | 81.5% | 63.0% |
| clearance | 0.413 m (torso) | **0.126 m (HEAD)** |

The right figure is **6 mm above the 0.12 m floor** — the tightest margin in
the project. It does not collide, but it is not comfortable either, and the
right arm's previous pose had 100% IK and 1.27 rad of seam margin. Mirror
symmetry is given up by this change. Right stays MOCK, so this is not yet
exercised on hardware.

`WORKSPACE_CENTRE`/`WORKSPACE_ORIENT` are **derived** from the home (offset =
P_HOME) and were re-measured. Changing the home again means re-measuring them.

**SEAM RISK, left joint_5.** −165.90° is 14.10° (0.246 rad) from the ±180
seam, inside the 0.3 rad margin used elsewhere. The velocity homing path is
immune (it uses `angle_diff`, never a position setpoint), but any future
position-mode code touching joint_5 must go through `pose_delta_rad`.
Flagged and deliberately left at the given value.

### Homing is now a VELOCITY law
`speed[j] = clip(kp * angle_diff(target, actual), -vmax, +vmax)`, kp 0.5,
vmax 0.05 rad/s, deadband 1°. Error magnitude cannot produce a fast move, and
it decelerates smoothly as `kp*err` drops under the cap (observed: 0.050 →
0.033 → 0.020 rad/s). The 120° refusal is **gone** — there is no position
setpoint to jump to, which was the only thing it was guarding.

**The Gen3 7-DOF macro exports 8 command interfaces, ALL "position".** There
is no velocity command interface (velocity is a *state* interface only), so
the rate is delivered as `q_cmd = q_actual + speed*dt` recomputed each cycle
off the **measured** position — never more than ~2.5 mrad ahead of the arm.

### Two real bugs found and fixed
1. **`ik_follower_node` crashed in `real_robot` mode.** `self.max_vel` was
   read at line 201 but not assigned until line 211 → `AttributeError`.
   **`real_robot:=true` had therefore never successfully run.** Worse, once
   the crash was fixed, line 211 would have *overwritten* the 0.10 rad/s
   safety cap the block had just applied. A clamp that is discarded one line
   later is worse than no clamp, because the warning still prints.
2. **New launch files were not installed.** `setup.py` listed them by hand;
   `mock_real.launch.py` and `real_arms.launch.py` silently did not install.
   Now a `glob('launch/*.launch.py')`.

### Kortex driver: deactivation is ONE-WAY
Deactivating the hardware component tears down the API router;
`on_activate` then fails with
`KBasicException: Router is not active. Unable to execute send.`
**The driver-level e-stop cannot be undone in-process — recovery is a
relaunch.** The gate is built around this (it *starts* drivers, never
reactivates them).

### `/real/joint_states` is FROZEN while the component is inactive
313 samples over 5 s → **1 distinct value per joint, peak-to-peak 0.000e+00**,
identical to the reading taken 40 min earlier while active. The broadcaster
keeps publishing at 100 Hz but the content is a cache. Any hold/limp test
done through this topic while inactive **cannot detect motion** and will
report "holding" no matter what the arm does. Same zero-variance signature
that identified the dead j7 pot. **The hold-vs-limp question is still
unanswered.**

### Verified end-to-end against `mock_real.launch.py` (no hardware)
- gate prompt appears; with no tty it stays **SIM-ONLY** (correct default-no)
- homing: all 7 joints within 0.05 rad, max error 0.0174 rad (1.00°, = the
  deadband); joint_5 took the +60° short way, not the naive −300°
- bridge enables, prints `REAL ARMS LIVE`
- **preview delay 1.04 s measured** (configured 1.00), spread 0.02 s across
  three crossing levels; real reproduced sim travel 0.3000 rad exactly
- **lag monitor trips at 0.152 rad** on a stalled follower → e-stop latched,
  bridge disabled
- min clearance during homing 0.373 m
- real EE displacement for a 0.30 rad sim joint_1 move: **0.2005 m**

### Still not done
- hold-vs-limp on the real arm (needs live feedback, i.e. component active)
- right arm real (two-arm `tcp/twist.linear.x` collision unfixed)
- `docs/REAL_ROBOT_BRINGUP.md` still describes the v1 single-stack flow
- the gated flow has **never been run against real hardware**

# ARM MOUNT ORIENTATION — FIXED (2026-08-05)

## The symptom

At the legacy home joint angles the real arms point FORWARD, in front of the
wearer. In sim the same angles put both arms up and back over the shoulders:
left P_HOME `(0.814, -0.492, 1.232)` — 0.49 m **behind** — and right
`(0.173, 0.117, 1.987)` — 0.24 m **above** a 1.75 m wearer's head.

## HOME JOINT ANGLES ARE GROUND TRUTH — never change them to fix geometry

`config/home_positions_{left,right}.txt` and the two `initial_positions` in
`srl_dual.urdf.xacro` hold the LEGACY REAL home. They match the real arm, and
the sim→real bridge replays sim joint angles onto the real arm, so any
difference is commanded **as a jump**. When the sim's geometry looks wrong,
the mount is the suspect; the joint angles are not. They were not touched by
this fix and must not be touched by the next one.

## (A) vs (B): it is (A), MOUNT ROTATION. Evidence.

**The arm's SHAPE is bit-identical to the stock Gen3.** Every link's pose
relative to its own `base_link`, at the home angles, was compared against
`ros2_kortex`'s stock `gen3.xacro dof:=7` at the same angles:

```
                        MAX deviation, srl_dual vs stock gen3
  LEFT   9 links        position 0.000000000 m,  rotation 0.0000024 deg
  RIGHT  9 links        position 0.000000000 m,  rotation 0.0000024 deg
```

Float noise. Both arms instantiate the same vendor macro and neither modifies
it, so only the world-frame orientation differed → hypothesis (A).

Three further checks rule out (B), a joint-zero-convention error:

- **The URDF's zero IS Kinova's zero.** At all joints zero the stock URDF puts
  the EE at `(0, -0.0249, 1.1874)` in the base frame — the fully-extended
  vertical arm, 1.187 m tall, with only the known 24.9 mm wrist offset off
  axis. That is Kinova's documented zero pose.
- **The vendor driver applies no offset and no sign flip.**
  `kortex_driver/src/hardware_interface.cpp:693` is
  `arm_positions_[i] = wrapRadiansFromMinusPiToPi(toRad(actuator_position_deg))`
  and `kortex_math_util.cpp:9` is `degree * M_PI / 180.0`. Degrees → radians
  plus a wrap, nothing else — exactly what `kortex_convention.py` does.
- **A rigid base rotation alone explains the symptom**, and one was found that
  satisfies every criterion. If the shape were wrong, no rotation could.

So **Task 0.3 (per-joint offsets) does not apply. No offsets are needed, in
the URDF or in `kortex_convention.py`.**

## Frame convention — verified against the humanoid's own geometry

`x` lateral, `y` fore/aft with the wearer facing `+y`, `z` up. Confirmed
independently three ways from the model itself: the backpack link sits at
`y = -0.12` (behind), the harness strap visuals at `y = +0.10` (chest side),
and the foot visuals are offset `+0.06` in `y` (toes forward).

The frame is right-handed, so `right = forward × up = ŷ × ẑ = +x̂` and **+x is
the wearer's RIGHT**, as assumed.

**But the naming is viewer-perspective, not anatomical.** `left_leg`,
`left_mount` and `human_left_upper_arm` all sit at **positive** x — i.e. on the
wearer's anatomical **right**. Kinematically this changes nothing (the
sagittal plane is x=0 either way) but it matters when matching a sim arm to a
physical arm in the lab. Check which side is which before trusting a label.

## Root cause: the angle was in the wrong rpy slot

Old value: left `rpy="0 0.6 0"`, right `rpy="0 -0.6 0"`.

In this file's convention `x` is lateral and `y` is fore/aft, so **a rotation
about y is a LATERAL SPLAY, not a forward pitch**. The old mount splayed both
arms 34.4° outward with **zero** forward tilt. Someone thinking in the usual
ROS x-forward convention would write exactly this for "pitch the mounts down".

The tell is the hand-written sign flip on the right arm. A pure forward tilt
is a roll, and `mirror(r,0,0) = (r,0,0)` — it needs no flip. The old value
needed one only because the angle was in the pitch slot.

## New mount

Parameterised by the two angles a real bracket actually has, with the right
mount **derived** as the exact sagittal mirror so an asymmetric pair is
unrepresentable:

```
mount_tilt_deg  =  60     forward tilt of the plate NORMAL from vertical
mount_clock_deg = -150    clocking of the LEFT arm on its bolt circle
                          (the right arm is clocked by +150)

R_left  = Rx(-60 deg) @ Rz(-150 deg)      R_right = M R_left M,  M = diag(-1,1,1)
```

which the xacro emits, in closed form, as

```
              roll                pitch                yaw
  left    0.982793723247329  -0.447832396928932  -2.860557752086980
  right   0.982793723247329  +0.447832396928932  +2.860557752086980
```

Exact mirrors: roll equal, pitch and yaw exact negatives,
`|| M·R_left·M − R_right ||_max = 0.000e+00`, `|| M·xyz_left − xyz_right || =
0.000e+00`. `xyz` is unchanged at `(±0.15, 0, 0.18)` — the position was
already correct.

**Why round angles are the roundness here.** `tilt` and `clock` are physical;
the rpy triple is only the ZYX parameterisation of the same rotation and its
three numbers have no separate meaning in this frame. Both chosen values are
multiples of 30°, and `tilt` sits on the reachability plateau (see below). No
rpy triple that is a multiple of 45° satisfies all six criteria — the closest,
`(-90, 30, 0)`, fails only on the project's own real-robot clearance floor
(0.105 m padded, against a 0.12 m floor).

## Top 3 candidates, all six criteria

All are exact-mirror mounts evaluated at the unchanged legacy home angles.
"home clr" is `clearance.py`'s own metric (distal link origins vs
torso/head/hips), with the 0.05 m `real_robot` wearer pad in brackets.

| | **#1 CHOSEN** tilt 60 / clock −150 | #2 tilt 65 / clock −155 | #3 tilt 65 / splay 10 / clock −165 |
| --- | --- | --- | --- |
| left rpy (deg) | (56.31, −25.66, −163.90) | (62.77, −22.52, −168.85) | (63.01, −23.50, −179.53) |
| 1 EE forward, y | L +0.214 R +0.317 **PASS** | L +0.250 R +0.297 PASS | L +0.365 R +0.158 PASS |
| 2 EE height, z | L 1.393 R 1.346 **PASS** | L 1.310 R 1.364 PASS | L 1.318 R 1.349 PASS |
| 3 rpy exact mirror | **yes** | yes | yes |
| 3 \|ΔP_HOME\| | 1.384 m **FAIL** (see below) | 1.384 m FAIL | 1.384 m FAIL |
| 4 home clearance | 0.224 m (0.173) **PASS** | 0.224 m (0.174) PASS | 0.263 m (0.213) PASS |
| 5 through-range clearance | 0.071 m **PASS** | 0.083 m PASS | 0.101 m PASS |
| 6 IK, offline | 89.6% / 91.7% | 91.7% / 89.6% | 85.4% / 91.7% |
| 6 IK, **live `/compute_ik`** | **93.8% / 81.2% PASS** | 91.7% / **72.9% FAIL** | not measured |
| roundness (tilt, clock) | **both multiples of 30** | neither | neither |

**#2 was measured live and lost.** Its offline score was marginally the best,
but against the real TRAC-IK plugin its right arm drops to 72.9% — below the
80% bar — while #1 holds 81.2%. Offline ranking did not survive contact with
the actual solver; that is why the shortlist was re-measured live rather than
decided on the model.

### Criterion 3's 0.02 m clause is unsatisfiable, and this is a proof, not an excuse

With mirror-symmetric mounts, `|right_P_HOME − mirror(left_P_HOME)|` **does not
depend on the mount rotation at all**:

```
  mirror(left_P_HOME) = M(p_L + R_L v_L) = p_R + (M R_L M)(M v_L) = p_R + R_R (M v_L)
  right_P_HOME        = p_R + R_R v_R
  difference          = |R_R (v_R − M v_L)| = |v_R − M v_L|        <- R cancels
```

where `v` is the EE in the arm's own base frame. Numerically
`|v_R − M v_L| = 1.3837 m` for **every** mirror-symmetric mount, old or new.

The cause is the home angles, not the geometry: the two arms are simply
**parked asymmetrically**. `home_positions_right.txt` already records that the
legacy right pose is not the mirror of the legacy left one. Most of the gap is
`joint_1`, the shoulder roll — clock the right arm by −165.5° and the residual
drops to **0.0834 m**, with joints 2–7 then within 10° of an exact mirror. A
`joint_1` difference is a parking choice on a *continuous* joint and carries no
geometric meaning.

Two consequences worth knowing:

- **Both hands land on the same side of the body.** `v_L` and `M v_R` are
  119.6° apart, a fixed property of the home angles. To put both hands forward
  the mount must straddle that 119.6°, which necessarily throws both EEs to
  one side. No mirror-symmetric mount avoids it. In the chosen mount both are
  at negative x (the wearer's left).
- Fixing it needs a **re-parked right arm**, not a different mount — and that
  is a hardware action, since the home angles are ground truth.

`config/home_positions_right.txt` and `config/real_home_reference.txt` both
record that the right arm's legacy values were **never read from hardware**
("RIGHT ... NOT READ ... UNVERIFIED"). Worth re-reading them at the next lab
session before drawing conclusions from the asymmetry.

## The reachability plateau — why tilt ≈ 60°

Forward tilt is the parameter that matters, and it matters enormously. IK over
a frontal working volume (0.4 × 0.3 × 0.4 m on each arm's own side, table-to-
chest height, top-down grasp with yaw free, collision-filtered), swept over
plate tilt with zero splay:

| tilt | 0 | 20 | 30 | 40 | **45** | 50 | **60** | 70 | 80 | 90 | 120 | 150 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| IK left | 0.188 | 0.646 | 0.812 | 0.958 | **0.958** | 0.958 | **0.896** | 0.917 | 0.833 | 0.854 | 0.729 | 0.375 |
| IK right | 0.354 | 0.771 | 0.896 | 0.979 | **0.979** | 0.979 | **0.917** | 0.875 | 0.875 | 0.812 | 0.938 | 0.792 |

The peak is 40–50°; **the old mount sits at tilt 0**, the worst end of the
curve, which is the whole story of the 47.9% / 58.3% it scored. Tilt 45 is the
peak but its home EEs come out at 1.32–1.62 m — too high — so the home-pose
criteria push the answer to 60, still at 0.896/0.917.

**A key simplification, verified to 1e-6 m:** `joint_1` is continuous and
coaxial with the plate normal, so spinning the mount about that normal is
absorbed exactly by `q1`. Reachability and wearer clearance therefore depend
only on the plate NORMAL (2 DOF); the clocking moves only the home pose. That
factorisation is what made an exhaustive search tractable, and it also means
**the clocking is pinned only by the home-pose criteria — the shakiest data in
this problem — while the tilt rests on reachability, which is robust.**

## THE PROXIMAL COLLISION — a real mechanical finding, not a modelling nuisance

The first candidate was applied and `/compute_ik` then returned
**NO_IK_SOLUTION (−31) for 48/48 poses on both arms**, including a self-test
asking IK for the pose the arm was already standing in.
`/check_state_validity` named it immediately:

```
  left_shoulder_link  <-> human_left_upper_arm    depth 0.0240
  right_shoulder_link <-> human_right_upper_arm   depth 0.0007
```

The offline search had missed it because it scored clearance from **distal
link origins** — `clearance.py`'s metric — which cannot see a proximal link,
and cannot see a contact that is nowhere near a link origin. The offline model
was rebuilt on the links' **actual collision meshes** (COLLADA for the Gen3,
STL for the Robotiq) and now agrees with MoveIt.

Sweeping the *entire* space of mount rotations with that model:

```
  base_link vs the wearer's upper arm, best over ALL rotations : +0.0443 m
                                       worst                   : -0.0490 m
  best among rotations that ALSO satisfy the home criteria      : -0.0154 m
```

The geometry: the mount origin is `hypot(0.06, 0.12) = 0.1342 m` from the
wearer's upper-arm axis, i.e. **38 mm** of surface clearance, and the Gen3 base
tube is 46 mm in radius and **171 mm long**. Almost any forward tilt swings it
into contact. `base_link` is moved by no joint, so this is a **static
interference set by the mount POSITION** and by the mannequin's arms being
rigid cylinders at x = ±0.21 — not something the rotation, or any planner, can
avoid.

Handled the way the project already handles the 36 mount-adjacent exclusions:
`{base_link, shoulder_link, half_arm_1_link}` × `human_{left,right}_upper_arm`
are now `disable_collisions` in `srl_dual.srdf`, with the reasoning in the
file. **Everything distal of `half_arm_1` stays enabled** — that is what
actually protects the operator, and its measured home clearance is 0.224 m.

**MECHANICAL ACTION, recorded rather than hidden:** on the real rig the bracket
must stand the arm base off the wearer's shoulder, or the mounts must move
outboard/up. The sim will not warn you about it again, because the pair is now
excluded.

## Everything re-derived — old vs new

| | OLD (rpy 0 0.6 0) | NEW (tilt 60, clock −150) |
| --- | --- | --- |
| P_HOME left | (0.8142, −0.4922, 1.2324) | **(−0.5195, 0.2138, 1.3928)** |
| P_HOME right | (0.1726, 0.1173, 1.9870) | **(−0.8596, 0.3169, 1.3456)** |
| anchor quat left | (0.0182, 0.8958, 0.2062, 0.3933) | **(0.4283, −0.0809, −0.6071, 0.6644)** |
| anchor quat right | (0.5803, −0.1462, 0.7789, 0.1875) | **(0.4942, 0.8189, −0.0536, −0.2870)** |
| home clearance left | 0.482 m torso (0.413 padded) | 0.224 m torso (0.173 padded) |
| home clearance right | 0.176 m **head** (0.126 padded) | 0.366 m torso (0.302 padded) |
| IK, frontal volume | 47.9% / 58.3% collision-free | **93.8% / 91.7%** offline, **93.8% / 81.2%** live |
| through-range clearance | 0.006 m / 0.005 m | **0.110 m / 0.128 m** |
| continuous reach, left | min 0.14 / med 0.22 / max 0.22, isotropy 0.64, 19/26 full | min 0.14 / med 0.22 / max 0.22, isotropy 0.64, **20/26** full |
| continuous reach, right | min 0.08 / med 0.22 / max 0.22, isotropy 0.36, 15/26 full | min 0.09 / med 0.22 / max 0.22, isotropy **0.41**, 14/26 full |
| \|ΔP_HOME\| vs mirror | 1.3837 m | 1.3837 m (invariant — see proof above) |

`WORKSPACE_CENTRE` and `WORKSPACE_ORIENT` in `master_calibration.py` are
**derived** from P_HOME (offset = P_HOME) and were updated to the new values.
The old right-arm figure of 0.126 m padded clearance was the tightest margin in
the project; it is now 0.302 m.

The right arm's home clearance improving from 0.176 m to 0.366 m while the
left's drops from 0.482 m to 0.224 m is the asymmetric parking again, not a
regression: both are far above the 0.12 m real-robot floor, which the old
right arm cleared by only 6 mm.

## Verification, all without hardware

- `xacro` parses; `check_urdf` reports a valid single-rooted tree.
- Left and right mount rpy printed side by side and proved exact mirrors to
  0.000e+00 in both the rotation matrix and the position.
- **The sim boots into the new pose.** `/robot_description` was read back off
  the topic (not the file) and its `backpack_to_*_mount` origins match the
  edited xacro to 3.89e-16 — so a stale install or stale process would have
  been caught. `/joint_states` sits at the home angles to 0.003°, and tf2
  `world → *_end_effector_link` reproduces the offline P_HOME to 4 dp.
- `/check_state_validity` returns `valid=True, contacts=0` for both arm groups
  at home.

### Follower regression, with the master MOVING — the measurement nobody had

No master arm is attached (`/dev/ttyACM*` absent), so `/master_arm_pose_*` was
driven from a script: a ±0.10 m Lissajous inside the master ball about the new
anchor, 50 Hz for 40 s, with the real `ik_follower_node` for each arm.

```
            IK success        GUARD                      min clearance  tracking |cmd - tf2 EE|
  left      95.3% (1668/1750) 1668 direct, 0 rejected    0.205 m, 0 blocks   RMS 0.0164 m, max 0.0734 m
  right     91.1% (1435/1575) 1431 direct, 3 slewed, 1 rejected  0.365 m, 0 blocks   RMS 0.0694 m, max 0.2230 m
```

**There is no prior moving-master number to regress against** — every earlier
tracking figure in this file was taken with the master at rest and is
sub-millimetre for that reason. Read these as the first real measurement, not
as a regression. The right arm's 69 mm RMS is lag, not error: the guard shows
1431 direct commits and only 3 slews, and clearance never blocked.

Redundancy retries were heavily used (492 left, 878 right), which is the
`joint_3` re-seed doing its job now that collisions are genuinely being
checked.

### Wrist cameras at home

```
  left   at (−0.476, +0.209, +1.429), optical axis (−0.628, −0.471, +0.620), elevation +38.3 deg
         points UPWARD - sees nothing at table height from home
  right  at (−0.814, +0.345, +1.327), optical axis (−0.523, +0.196, −0.830), elevation −56.1 deg
         hits z = 0.80 m at (−1.146, +0.469), range 0.64 m, which IS in front of the wearer
```

The camera frames are `rpy = (pi, pi, 0)` from `end_effector_link`, i.e. a
180° spin about z, so the optical axis is the gripper approach axis.

**Only the right camera sees table height from home, and only off to one
side.** This is a consequence of the home *joint angles*, not the mount — the
left wrist is parked looking up and back. It is fine for teleop, where the
operator moves the arm before looking, but **any perception work that assumes a
useful view at the home pose will not get one.** Point the arms first.

### What you should SEE in RViz

- Both arm bases still bolted at `(±0.15, −0.12, 1.25)`, on the pack, unmoved.
- Each base plate **tilted 60° forward** — the arm leaves the pack heading
  up-and-forward over the shoulder, not sideways-and-up as before.
- Both arms come **over the shoulders to the front**: `half_arm_1` crosses
  `y ≈ +0.13`, i.e. in front of the chest, at about shoulder height
  (`z ≈ 1.39`).
- Both hands end up **in front of the wearer at chest height** (`y = +0.21`
  and `+0.33`, `z = 1.39` and `1.35`) and — expected, not a bug — **both on
  the wearer's left**, at `x = −0.52` and `−0.86`, with the arms crossing.
  That asymmetry is the asymmetric parking of the home angles, proved above to
  be independent of the mount.
- Nothing above head height. Nothing behind the back.

If instead you see the arms splayed sideways and reaching up and back over the
shoulders, with one hand above the head, the edit did not take — check for a
stale `move_group` process rather than a stale install (`srl_description` is
symlink-installed, so the file is live).

## Debris found on the way

- **`/dev/shm` fills with stale `fastrtps_*` segments** from killed stacks, and
  Fast DDS then logs `RTPS_TRANSPORT_SHM Error ... Failed init_port
  fastrtps_port7002`. Discovery goes intermittent: services appear for one
  client and not another, and `wait_for_service` times out on a service that
  `get_service_names_and_types()` can see. Clear `/dev/shm/fastrtps_*` **with
  the stack stopped**, then relaunch.
- **Killing a launch by its `ros2 launch` PID alone leaves orphans.** A
  half-killed stack shows `controller_manager: Waiting for data on
  'robot_description'` forever because `robot_state_publisher` is gone. Kill
  the explicit PID list — `move_group`, `ros2_control_node`,
  `robot_state_publisher`, `spawner` — and check the count is zero before
  relaunching.
- `static_transform_publisher --frame-id world --child-frame-id world` in
  `generate_demo_launch` dies with `exit code -6` every launch. Harmless
  (a self-referential transform), but it is not evidence of a problem you
  caused.

---

## Part 4 — perception

AprilTag 36h11 via OpenCV's ArUco module (no extra dependency, no separate
process). Characterised on **synthetic images through the real detector code
path**, 200 trials per distance, blur σ 0.8 px, pixel noise σ 3.0:

| dist (m) | detection | pos mean / p95 (mm) | rot mean / p95 (°) | latency (ms) |
| --- | --- | --- | --- | --- |
| 0.15 | 100% | 0.19 / 0.23 | 0.06 / 0.15 | 5.8 |
| 0.25 | 100% | 0.40 / 0.71 | 0.36 / 0.88 | 5.7 |
| **0.35** | **100%** | **0.74 / 1.60** | **0.75 / 1.94** | **6.4** |
| 0.50 | 100% | 1.63 / 4.01 | 1.33 / 3.03 | 7.1 |
| 1.00 | 100% | 9.48 / 22.12 | 8.60 / 47.06 | 5.9 |

**STUDY GATE at 0.35 m working distance: 100%, PASSED — in simulation only.**

**Corner refinement: SUBPIX is the shipped default, and the measurement is why.**

| method | latency | pos err | rot err |
| --- | --- | --- | --- |
| none | 4.2 ms | 2.43 mm | **27.7°** (unusable — the planar ambiguity flips) |
| **subpix** | **3.3 ms** | **0.65 mm** | **0.92°** |
| apriltag | 141.7 ms | 0.46 mm | 0.82° |

APRILTAG refinement costs **43×** the time for 0.2 mm and 0.1°. At 141.7 ms
the detector runs at 7 Hz, slower than the arm moves.

### Two bugs in the measurement harness, both silent

1. **The renderer produced a MIRRORED tag** — 0/120 detections at every
   distance. OpenCV's camera has +y **down**, so pairing ArUco object corners
   (+y up) with the marker image's own TL,TR,BR,BL reverses the winding.
   AprilTag decoding is not mirror-invariant; a vertical flip of the same crop
   decoded immediately.
2. **The ground-truth rotation was 180° out**, giving a flat ~22° error that
   looked like the planar-pose ambiguity. The winding fix flips the rendered
   tag's own frame in y, which for a planar target is Rx(π) applied in
   **OBJECT** space: `R_render @ Rx(π)`, not `Rx(π) @ R_render`. **The two
   forms COMMUTE for a tilt about x or about y**, so checking those axes
   cannot tell them apart — only a general in-plane axis can. Measured over
   200 random axes: wrong form 21.79° mean, right form **1.06°**. The wrong
   form was also noise-INDEPENDENT and scaled with tilt, which is how it was
   distinguished from the genuine ambiguity.

**Colour/shape is a clearly-secondary fallback**, off by default, hard-capped
at confidence 0.45 so it can never outrank a tag in the tracker's fusion. It
fails by being *confidently wrong*, which AprilTag does not.

`object_pose_tracker` **drops** objects unseen for 1.5 s rather than holding
them. A held pose is indistinguishable from a live one downstream — the same
class of bug as the frozen `/real/joint_states` and the dead j7 pot.

## Part 5 — shared autonomy

**Grasps** come from a lookup table (top-down at the centroid, gripper aligned
to the object's minor axis), and **every candidate is validated with
/compute_ik — both the grasp and its pre-grasp standoff — before it is
offered.** An offered-then-failed grasp is worse than no offer: the
participant attributes the failure to the autonomy and the trial is
contaminated in a way analysis cannot undo.

**Intent** is driven **primarily by the IMU pointing direction**, not by
commanded EE velocity. Velocity is a derivative of the pot chain and inherits
every pot fault; a rejected frame FREEZES the command, so velocity reads zero
exactly when the operator is moving hardest. Velocity is a secondary cue at
weight 0.25. `test_pointing_is_used_when_velocity_is_frozen` encodes this.

### The three hard cases — MEASURED, and tested

| case | behaviour |
| --- | --- |
| **no objects** | empty distribution, `state="no_objects"`, `top=None`. Never a uniform distribution over nothing, never stale. Objects disappearing clears the estimate. |
| **equally likely** | `top_p = 0.5000`, `margin = 0.0000`, `ambiguous = True`. Three-way tie holds 1/3. **The arbiter must not enter ASSIST on a tie** — assisting toward the wrong one of two adjacent objects is worse than not assisting. |
| **changes mind mid-reach** | switches in **2 frames = 0.10 s at 20 Hz** |

The change-of-mind case is the one that matters, and the **forgetting factor**
is what makes it possible:

| forget | peak P(A) | frames to switch | seconds |
| --- | --- | --- | --- |
| **0.000** | 1.0000 | **201** | **10.05** ← latches |
| 0.005 | 0.9947 | 2 | 0.10 |
| **0.020 (default)** | **0.9788** | **2** | **0.10** |
| 0.100 | 0.8946 | 1 | 0.05 |

With `forget = 0` the log-odds run away, confidence saturates at 1.0 and the
estimate takes 10 s to abandon a goal. `test_zero_forgetting_latches_...`
documents the failure mode the default avoids.

**Handover is discrete and operator-confirmed.** DIRECT → ASSIST requires
P > threshold **AND** proximity **AND** not-ambiguous. In ASSIST only wrist
ORIENTATION servos (SLERP over 0.5 s, never a snap); **position stays 100% the
operator's in every state** — that line in `handover_arbiter._publish()` is the
safety argument of the whole design. Cancel by moving away or by button, with
**no jump**: the blend is frozen where it is and released, so the commanded
pose is continuous. **The gripper is never closed by this node.**

## Part 6 — gravity compensation

See `docs/system/05_gravity_and_load.md`. Three different problems:

**(a) Robot payload.** `payload_manager` sets the Kinova payload on every
GRASPED and clears it on release, and **publishes what it used so it lands in
the trial log** even when the hardware call is a no-op. Why it matters for the
data, not just the robot: an unconfigured payload produces a steady-state
tracking error that VARIES WITH THE OBJECT, and object type is crossed with
condition — a confound baked in.

**The sim CANNOT answer the with/without question, and saying so is the honest
result.** `mock_components/GenericSystem` echoes commands with no dynamics, so
payload has *exactly zero* effect: 0.0164 m tracking RMS both ways, difference
0.0000 m. That is a property of the mock, not evidence payload does not
matter. **UNVERIFIED WITHOUT HARDWARE.**

The hardware path is a **stub** and says so: it needs a live Kortex session,
the arm permits exactly one, and `kortex_highlevel_bridge` owns it.

**(b) Master fatigue.** No gravity compensation; the operator's arm carries
it. Fatigue is collinear with condition order, so NASA-TLX is contaminated in
every condition without intervention. Mitigations, in order of effect:
counterbalance (converts bias to variance), **blocks ≤ 8 min with 2-min
rests**, an arm rest between trials, a counterweight at the master shoulder,
session ≤ 45 min. **A Borg CR10 probe between every block** turns fatigue from
an unmeasured confound into a covariate.

**(c) Worn mass.** 8.2 kg per Gen3, >17 kg with harness. **E1 and E5 are
bench-mounted; E2–E4 should be worn only if the claim is about wearable SRLs**,
and the choice is recorded per session as `mounting: worn | bench`. Worn
operation is **not recommended** until the mount's proximal interference is
fixed mechanically.

## Part 7 — the experiments

Five folders under `src/srl_experiments/experiments/`, each with `protocol.md`,
`config.yaml`, `run_*.py`, `analyse_*.py`, `scenarios/` and `results/`.

Shared infrastructure, in one place so the experiments cannot drift apart:
`trial_logger.py` (one CSV per trial + a session manifest, flushed per sample),
`conditions.py` (**Williams** squares, which balance immediate sequence as well
as position), `task_trial.py` (one reach-and-grasp trial), `runner.py`,
`scripted_operator.py`.

**Anonymity is enforced in code**: `write_manifest()` raises if passed `name`,
`email`, `dob`, `address` or `phone`.

E2's counterbalancing at n=12: position imbalance 0, sequence imbalance 0,
perfectly balanced.

## Part 9 — pilot

All five experiments run **end to end in sim with scripted input**, and every
metric is captured (verified by a column-by-column audit of every trial file:
zero unexplained-empty columns in all five). All five analysers run and emit
plots.

The pilot found **four real bugs that a reading would not have**:

1. **`grasp_generator` died at startup** with `Executor is already spinning` —
   `spin_until_future_complete()` inside a timer callback. From outside,
   nothing looked broken: the arbiter simply sat in DIRECT reporting
   "P=1.00, no grasp". Fixed with a `ReentrantCallbackGroup` and a
   `MultiThreadedExecutor`.
2. **Every trial ran to the timeout.** No completion condition for
   `direct`/`shared`, so "completion time" was the timeout for every
   condition — E2 and E3 would have measured nothing.
3. **Every trial after the first logged 0.00 s.** The arbiter latches GRASPED
   until the gripper opens, and the mock gripper boots at 0.79 rad — already
   past any "closed" threshold. Now GRASPED requires a *closing transition*,
   and the trial only ends on a transition observed *inside* that trial.
4. **`clutch` and `intent_top` were empty in every log.** Nothing published
   `/master_status_<arm>` in scripted mode, and nothing published objects, so
   the whole intent → grasp → handover path was never exercised and the pilot
   would have passed with those columns silently blank.

Teleop standalone verified by **physically removing** `srl_autonomy`,
`srl_perception` and `srl_experiments` from `install/`: `ros2 pkg executables
srl_teleop` still lists 29, `srl_autonomy` is not found, and
`ik_follower_node` still reaches "Tracking enabled".

36 unit tests pass across the workspace.

## What is still NOT verified without hardware

- **Perception on real cameras.** Detection rate, pose accuracy and latency
  are from synthetic images. Real lighting, motion blur, rolling shutter and
  the Kinova cameras' true intrinsics are unmodelled. The ≥95% gate has been
  passed **in simulation only**.
- **Payload compensation on a real arm.** The Kortex call is a stub.
- **The complementary filter against a real gyro.** The gyro stream in the
  measurement is synthesised from measured noise parameters.
- **The two-IMU path.** No second sensor exists yet.
- **The gated real-arm flow.** Exercised only against `mock_real.launch.py`.
- **The right arm's home joint values**, never read from hardware — and
  P_HOME for that arm depends on them.
- **The mount's proximal interference** is real and mechanical, not a
  modelling artefact.

---

# MOUNT, SECOND FIX (2026-08-05) — DERIVED, and three constraints named

## What was wrong with the first fix

The first fix searched the mount rpy against ENDPOINT criteria only — where
the end effector lands. It had **no path or intermediate-link constraint**.
It found rotations that satisfied the endpoint by routing the arm THROUGH the
wearer: measured, `base_link` and `shoulder_link` were **75 mm inside the
torso** on both arms, and ten link pairs were in collision.

Nothing objected, because the offending pairs had been SRDF-excluded and the
offline check that approved the mount excluded the same pairs by the same
argument. **An exclusion silences the alarm; it does not move the metal.**

Correcting one thing in the brief: the two mounts WERE exact sagittal mirrors
(`||M R_left M − R_right|| = 0.000e+00`). The arms look unmirrored because the
HOME JOINT ANGLES are not mirror poses — `|v_R − M·v_L| = 1.3837 m`, and that
residual is **provably independent of the mount**.

## The derivation

`|EE − mount| = |v|` is forced by the joint angles, so the reachable EE set is
a SPHERE. The target DIRECTION fixes two rotational DOF exactly (shortest
arc); the third is a spin about that direction which **does not move the EE at
all** — it rotates the arm's PATH — and it is chosen by the minimum clearance
of every intermediate link. Endpoint and path are thereby separated instead of
competing.

## What binds — three constraints, none fixable by rotation

**1. Mirror-symmetric mounts cannot work at all.** `v_L` and `M·v_R` are
**119.6° apart**. Two vectors rigidly 119.6° apart cannot both point forward.
Resolution: the plate NORMAL is mirrored (one plate) but the **CLOCKING** —
which hole on the arm's circular base flange — is a per-arm assembly choice
and is not. That drops the mirror residual from 1.3837 m to **0.0889 m**.

**2. The old mount POSITION made 0.15 m clearance impossible.** At
(±0.15, −0.12) the Gen3 base tube starts 0.010 m from the torso box, so over
EVERY rotation the ceiling for `base_link` — a link no joint moves — was
**+0.0097 m**. Infeasible by a factor of 15 before any rotation was chosen.
The mount was moved onto the BACK of the pack.

**3. Clearance and reach are in DIRECT CONFLICT.** Moving the mount back to
gain clearance pushes the task volume toward the edge of the arm's 0.902 m
reach:

| mount y | max home clearance | task-volume centre, as % of reach |
| --- | --- | --- |
| −0.12 (old) | 0.026 m | 67% |
| −0.24 | 0.132 m | 80% |
| **−0.32 (applied)** | **0.211 m** | **88%** |
| −0.36 | 0.250 m | 93% |

## APPLIED

```
mount xyz (backpack-relative)  (±0.20, −0.20, 0.18)  -> world (±0.20, −0.32, 1.25)
left  rpy   ( 1.247666407, −0.232161321,  1.508064663)
right rpy   (−1.256457931, −0.033197491,  1.699631975)
```

NOT exact rpy mirrors, deliberately — the plate normal is mirrored, the
clocking is not. The mount POSITION is an exact mirror.

## The seven constraints

| | | |
| --- | --- | --- |
| 1 | ZERO colliding pairs at home | **PASS** — MoveIt `valid=True, contacts=0`, both arms |
| 2 | ≥0.15 m every arm link vs every wearer link | **PASS** — 0.1664 m (mesh), 0.1610 m (guard capsule) |
| 3 | no link through head or torso | **PASS** |
| 4 | \|ΔP_HOME\| < 0.02 m | **FAIL — 0.0889 m. BINDS.** Root cause is the home joint angles, which are ground truth and were not changed. |
| 5 | both EEs in front, 0.9–1.3 m | **PASS** — (0.697, 0.248, 1.146) and (−0.763, 0.298, 1.179) |
| 6 | clear through the working range | **PASS** — +0.0495 / +0.0544 m over reachable poses |
| 7 | IK ≥ 80% over a frontal task volume | **FAIL — 18.8% / 18.8%. BINDS.** Live `/compute_ik`. |

Constraints 2 and 7 are the two ends of the trade-off above; they cannot both
be satisfied at this scale. **The rig needs a longer-reach arm, a task volume
closer to the body, or the mount raised above the shoulder rather than pushed
back.** That is a mechanical decision, not a software one.

Also found: the wearer model has **static self-collisions** (head↔torso 0.040 m,
hips↔legs 0.050 m). Two fixed links overlapping by construction — almost
certainly the orange link in the screenshot, and nothing to do with the arms.

## A5 — the regression guard

`mount_guard_node` runs on every `teleop.launch.py`, measures the arm as a
chain of capsules against every wearer primitive from TF, **ignores the SRDF**,
and fails loudly. It caught its own first version being too conservative
(bounding spheres fired at 0.113 m on a geometry with 0.166 m of real
clearance) — a guard that cries wolf gets switched off, which is worse than no
guard.

# HARDENING PASS (2026-08-05) — fault injection, and what it found

## The central defence: `block_monitor` + `blocking_aggregator`

Four mechanisms have historically stopped all motion **silently**. The common
failure was never the blocking — blocking is usually correct — it was that
blocking was **indistinguishable from working**. So every mechanism that can
stop motion now registers with a `BlockMonitor` that enforces:

- loud and **by name** on the first block, repeating every 5 s;
- **> 3 s escalates to ERROR** — sustained blocking is never normal;
- a **recovery string is mandatory at registration**; a blocker that cannot
  say how it clears is a latch, and `register()` raises;
- one published state on `/blocking`, aggregated to `/blocking_summary`.

Nine blockers registered in `ik_follower_node`: `estop`, `startup_unwind`,
`home_gate`, `motion_disarmed`, `ik_inflight`, `ik_failed`, `clearance_floor`,
`flip_reject`, `no_joint_state`.

`blocking_aggregator` adds the two things a per-node monitor cannot see:
a **unit that has gone silent** (a crashed node looks identical to a healthy
one on a dashboard), and **motion unexplained** — commands flowing, joints
not moving, and *nothing* blocking. That last is the exact silent-stall
signature and it now has its own topic.

## Fault injection results — 8/10  [SUPERSEDED: now 14/14, see the end of this file]

`ros2 run srl_teleop fault_injector --all`

| fault | result | evidence |
| --- | --- | --- |
| channel_dropout | **handled** | j4 auto-disabled, capability loss named, abort published |
| wrong_object_latch | **handled** | switched target in **0.069 s**, peak P 0.978 (bounded, never saturates) |
| ambiguous_flicker | **handled** | top_p **0.50000**, margin 1.3e-15, **0 flips in 199 samples**, arbiter stayed DIRECT |
| stale_object | **handled** | object dropped, `no_objects`, autonomy DIRECT |
| detection_failure | **handled** | yields to DIRECT rather than assisting toward something it cannot see |
| logging_failure | **handled** | raises on an unwritable path instead of dropping samples |
| identifying_data | **handled** | manifest refuses `name`/`email`/`dob` in code |
| nonmonotonic_clock | **handled** | 0 wall-clock interval uses remain |
| teensy_disconnect | **NOT handled** | dead-man did not trip on 10 s of silence |
| estop_during_motion | **NOT handled** | e-stop latched and recovered correctly, but the follower's *named* block did not appear |

## What injection found that reading would not

1. **`ik_inflight` latched "active" forever** — my own instrumentation. The
   clear() sat on a line only reached when `pending` was already False, which
   never happens on a healthy system. Held 65 s reporting "waiting 0.00 s".
   Moved the clear into the response callback.
2. **`/estop_reset` is a SERVICE, not a topic.** The first injection run
   published a Bool at it, nothing listened, the e-stop stayed latched, and
   **six subsequent faults reported NOT HANDLED for that one reason**. Tests
   must reset to a known-good state between faults.
3. **`channel_manager`'s auto-disable defaulted to OFF** (`0` frames). A
   channel could drop out for a whole trial and nothing would mark it dead.
   Now 25 frames — 0.5 s at 50 Hz, the measured length of a real j4 burst —
   and a required-channel dropout now publishes a trial ABORT.
4. **`estop_node` latched `deadman_enabled` at construction.** Setting it at
   runtime silently did nothing, so `participant_safety_node`'s attempt to
   force the dead-man on for a participant session would have failed silently.
   Now re-read every cycle.
5. **`ros2 param set` hangs on this box** (the daemon). Anything that shells
   out to it fails silently. The injector uses a parameter client.
6. **Duplicate node instances corrupt the blocking view.** Three stale
   followers published under the same unit key and overwrote each other. Added
   to the symptom-keyed troubleshooting table; `scripts/recover.sh` clears it.
7. **Six files still used the wall clock for INTERVALS** — the same bug class
   that produced a −2321 ms latency, fixed once in `kortex_highlevel_bridge`
   and left everywhere else, including `sim_to_real_bridge` (the sim→real
   command path) and `real_homing_node` (which drives the real arm). All
   converted to `time.monotonic()`; wall-clock timestamps deliberately kept.

## Still NOT handled, and honestly  [BOTH RESOLVED — see "Dead-man and blocking latch pass"]

**Kept because the diagnosis below was wrong in an instructive way.** Both
were closed later the same day. `teensy_disconnect` did not fail because the
dead-man was broken; it failed because the dead-man keyed on message ARRIVAL
while `master_pose_node` republishes stale data at 50 Hz, so the topic never
went silent. `estop_during_motion`'s observability gap was the follower
setting its blocker only inside `on_pose`, which stops being called at the
very moment the dead-man fires.

- **teensy_disconnect**: the dead-man is now runtime-settable and the injector
  enables it, but 10 s of total master silence still produced no trip. Not
  isolated before the end of the pass.
- **estop_during_motion**: the safety FUNCTION is verified — `/estop_state`
  goes true, the arm halts, and `/estop_reset` clears it. What did not appear
  is the follower's *named block* in the aggregated view, despite the code path
  being correct on inspection and having produced `[BLOCKED:estop] still held,
  46.2 s` in an earlier run. **The e-stop works; its observability is the open
  item.** Do not read this as "the e-stop is broken".

## New operational tooling

    ros2 run srl_teleop preflight [--participant]   # refuses to start, names why
    ros2 run srl_teleop fault_injector --all        # inject and verify
    ros2 run srl_teleop blocking_aggregator         # one place to look
    bash scripts/recover.sh                         # known-good from anything

`docs/system/06_troubleshooting.md` is now keyed by SYMPTOM, with "the arm is
not moving" listing every cause seen in this project and its check.

---

# Dead-man and blocking latch pass (2026-08-05)

## The dead-man's real hole was the SOURCE TIMESTAMP, and it is now closed

`master_pose_node` does **not** go silent when the master degrades. On a
validation failure it substitutes `last_good` and **keeps publishing at
50 Hz** — and it used to stamp that substituted frame `now`. So on the
failure mode that actually happens (a Teensy emitting garbage rather than
vanishing) the topic never went quiet and the timestamp never went stale, and
a dead-man keyed on **message arrival** could not fire at all. Only a clean
unplug produced silence, which is why the mechanism looked correct in every
test that pulled the cable.

Two changes, one at each end:

- `master_pose_node` stamps a substituted pose with **the time of the data it
  was built from** (`src_time`), not publication time, and publishes the data
  age in `status[7]`.
- `estop_node`'s dead-man keys on `msg.header.stamp`. An empty stamp falls
  back to arrival and **warns**, because an e-stop that cannot be cleared is
  its own hazard.

Measured, `stale 0.50 s + 5 checks x 0.10 s = 1.00 s` deadline:

| case | result |
| --- | --- |
| healthy master, fresh stamps, 243 frames | no trip |
| **degraded: publishing @50 Hz, FROZEN source stamp** | **trip 0.94–0.99 s** |
| clean unplug, topic silent | trip 0.97 s |

The trip line now names which it was: `source age 0.98 s, last message arrived
0.00 s ago -- master STILL PUBLISHING stale data`.

### The e-stop HALT was 4 s late, and detection never was

`trip()` called `halt_real_driver()` first, which did two
`wait_for_service(timeout_sec=2.0)` calls on `/real/controller_manager`. On
any sim or mock stack that service does not exist, so **every e-stop blocked
the whole node for 4.0 s** — inside a 10 Hz timer on a SingleThreadedExecutor
— before parking the arms, before logging, and before `/estop_state` could
change. Measured: decision at 0.97 s, trip logged at 4.98 s.

Fixed by parking first (`halt_once()` + publish `/estop_state`, both local
publishes) and making the real-driver path non-blocking (`service_is_ready()`,
the calls were already `call_async`). Do **not** reintroduce
`wait_for_service` there.

### Two more holes the trace exposed

- **An arm that has never published used to exempt itself from its own
  dead-man** (`if t is None: continue`). Only the left master usually
  publishes, so the right arm's dead-man was permanently disarmed. Age is now
  measured from when the dead-man was armed.
- **A reset was measured against staleness predating it**, so a latch caused
  by a brief gap re-tripped on the next tick and could not be cleared. Age is
  now measured from the later of "last fresh data" and "armed/reset". A truly
  dead master simply trips again one full deadline later.

### `/estop_deadman` — the dead-man is now observable

Part 4b requires verifying the e-stop every participant session, and a
mechanism that cannot be read cannot be verified. `estop_node` publishes
`armed`, `latched`, `age_s`, `arrival_age_s`, `stale_count`, `deadline_s`,
`never_published` and `unstamped_frames` at 10 Hz. Comparing `age_s` against
`arrival_age_s` is what distinguishes a stale republisher from a silent one.

## Follower e-stop block: invisible exactly when it mattered

The `estop` blocker was set and cleared **only inside `on_pose`**. The
commonest reason for the e-stop to latch is the dead-man, and the dead-man
fires *because* the master went silent — so `on_pose` stops being called at
the same moment and the blocker announcing "stopped: e-stop latched" was never
published. From the aggregated view the follower just went quiet, which is
indistinguishable from a crashed node. It is now maintained by
`publish_status` on a timer. Measured: named block appears in **0.11 s** with
the master publishing and **0.50 s** with the master silent (limit 1.0 s) —
the second case could not have passed before.

## The latch PATTERN, and a systemic fix

`scripts/audit_unreachable_clear.py` looks for the shape rather than the
instances: state set in one branch and cleared only where it cannot be reached
while set. Across **152 files** it reports **6 occurrences, all in
`ik_follower_node`, 0 LATCH-RISK** — every blocker is written as

```
if <bad>: blocks.block(N); return
blocks.clear(N)
```

so the clear always sits on the fall-through path of the guard it reports on.
That is fine while the guard is mutated elsewhere, and latches the moment the
guard also stops the callback running.

Rather than grade each instance, `BlockMonitor` now **auto-expires** any
blocker not re-asserted for `EXPIRE_S` (2.0 s, comfortably above the slowest
10 Hz asserting loop). A held condition is re-asserted every cycle by design,
so a blocker nobody is asserting is *abandoned*, not held. The expiry is
logged loudly — an abandoned blocker still means the unit asserting it stopped
running. This converts the whole class from a silent permanent lie into a
logged, self-correcting event.

## Injection results — 10/10  [SUPERSEDED: now 14/14, see the end of this file]

| fault | result |
| --- | --- |
| channel_dropout | handled (j4 auto-disabled, abort published) |
| **teensy_disconnect** | **handled — frozen 0.94 s, silent 0.97 s, follower block 0.28 s** |
| **estop_during_motion** | **handled — blocked, latched, recovered after reset** |
| wrong_object_latch | handled |
| ambiguous_flicker | handled |
| stale_object | handled |
| detection_failure | handled |
| logging_failure | handled |
| identifying_data | handled |
| nonmonotonic_clock | handled (0 offenders) |

### Harness lesson worth keeping

Four consecutive `teensy_disconnect` runs reported NOT HANDLED for reasons
that had nothing to do with the system: **every blocking call in the harness
publishes nothing while it waits**, and `reset_estop`/`set_param` wait longer
than the dead-man deadline — so the setup injected a master outage and the
resulting trip was attributed to the fault under test. `Harness.master_on()`
now runs the master on a background thread, as the real one does. Test any
dead-man with the stream established FIRST and the arming CONFIRMED
(`/estop_deadman`) before injecting.

Also: `pkill -f "lib/srl_teleop/estop_node"` **kills the shell running it**,
because the pattern matches that shell's own command line. Kill explicit PIDs.

## NOT DONE this pass  [BOTH DONE LATER THE SAME DAY — see "Acceptance test complete"]

- **Part 2 recovery paths (b)–(e)** — Teensy auto-reconnect for the next
  trial, Kortex session-loss recovery without relaunch, camera/zero-detection
  abort *before* the trial, network-dropout freeze-both. Specified, still not
  implemented.
- **The six experiments with faults injected mid-trial** — not run, so
  invalid-trial marking and pre-registered exclusion are still unverified end
  to end.

---

# Acceptance test complete (2026-08-05)

## Part 2 recovery paths (b)-(e) — implemented and injected

`recovery_manager` (new) owns all four, in one shape:
**DETECT → FREEZE → ABORT (trial INVALID, cause recorded) → RECOVER**, and
recover never means relaunch — a participant session cannot absorb one, since
the arm re-homes, the condition order is lost and the block is discarded.
`arm_link_monitor` (new) supplies real ping-based link health.

| path | detected | frozen | abort cause recorded | recovered |
| --- | --- | --- | --- | --- |
| (b) teensy disconnect | 1.16 s | 0.0 s | source data stale on left,right | 1.2 s, port re-DETECTED |
| (c) kortex session loss | 0.0 s | 0.01 s | session reported LOST | fresh session, no relaunch |
| (d) camera / zero detections | before the trial | — (correctly no freeze) | scan saw 0 objects, need >= 1 | allowed after fix |
| (e) network dropout | 0.01 s | 0.0 s | link DOWN to right, BOTH arms frozen | both links up |

Points worth keeping:

- **(b) recovery is re-DETECTION, not reopening.** The board moves between
  ACM0 and ACM1, so `find_port()` runs again; the next trial is gated until
  the master has been publishing FRESH data for `reconnect_confirm_s`.
- **(c) the session is CLOSED before a new one is opened**, because the arm
  permits exactly one and a leaked one refuses the next connect. The hardware
  component is deliberately NOT deactivated — that tears down the router and
  `on_activate` then fails permanently with `Router is not active`.
  `/real/session_recover` on the bridge does zero speeds → Stop() →
  CloseSession() → CreateSession().
- **(e) freezes BOTH arms on either link failing.** One arm live and one
  frozen on a wearer is worse than both frozen and cannot be told apart by
  looking.
- **`expect_real_stack` (default false)** stops the session and link paths
  firing on *silence*. On a sim stack those topics simply do not exist, and
  treating an absent optional subsystem as a fault would abort every sim
  trial. They still fire on an explicit `connected:false` / `up:false`.
- **`freeze_on_master_loss` (default false)**, for the same reason the
  dead-man defaults off: in sim the master goes stale constantly and freezing
  on it makes a healthy stack look like dead hardware. **The trial is marked
  INVALID either way** — that is data integrity and is never optional.
  Stopping the arm on master loss is the dead-man's job.

## Six experiments with faults injected mid-trial — ALL PASS

`python3 scripts/inject_experiment_faults.py` — 8 trials each, scripted, arm
link dropped while **trial 3 is running**.

| exp | ran | trials | invalid | continued | analyser excluded | all-invalid fails loudly |
| --- | --- | --- | --- | --- | --- | --- |
| E1 | yes | 8 | 1 | yes | yes | yes |
| E2 | yes | 8 | 1 | yes | yes | yes |
| E3 | yes | 8 | 1 | yes | yes | yes |
| E4 | yes | 8 | 1 | yes | yes | yes |
| E5 | yes | 8 | 1 | yes | yes | yes |
| E6 | yes | 8 | 1 | yes | yes | yes |

Cause recorded in every case:
`aborted mid-trial: link DOWN to right -- BOTH arms frozen, ...`

**E6 had no run/analyse scripts at all** — only a config, protocol and
scenarios. `run_vr_vs_mannequin.py` and `analyse_vr_vs_mannequin.py` were
written and `run_experiment.sh` now accepts `e6`.

`srl_experiments/validity.py` is the one definition of "analysable", shared by
all six so they cannot drift: it excludes on the `valid` column, prints the
count and every cause, and **exits non-zero** when no trials survive or when
more than `MAX_INVALID_FRACTION` (0.5) were excluded. Before this, a session
in which every trial was invalidated analysed cleanly to `0 valid trials`
followed by empty means and NaN intervals — indistinguishable from a null
finding by anyone reading the output.

### Four harness bugs, all of which made a failed injection look like a pass

Each produced a table of clean runs while testing nothing:

1. **A fixed delay is not "mid-trial".** The six experiments have different
   startup costs and trial lengths (0.4–2.5 s), so one offset landed inside a
   trial in some runs and in the gap between trials in others.
2. **The publisher must already be discovered.** Building the injector node
   after the trigger cost ~1 s of DDS discovery — longer than a scripted
   trial — so the fault arrived after the trial it was meant to hit.
3. **`/trial_state` persisted across experiments.** The previous run's final
   trial index still satisfied "trial 3 is running", so the fault fired during
   the *next* experiment's startup, before any trial existed. The trigger now
   requires the state to name the current experiment.
4. **`/participant/abort` is a ONE-SHOT event.** A fault raised between two
   trials was seen by neither. The runner now also consumes `/recovery_state`
   (continuous, 5 Hz) and **accumulates** faults seen during the trial, since
   a fault that recovers mid-trial would be gone from the current set by the
   time the trial ended. Invalidation is scoped to faults that appear *while*
   the trial runs — a pre-existing standing fault means the trial should never
   have started, which is what `/recovery/ready` is for.

The table also asserts the invalid trial names the **injected** cause. Without
that, an unrelated standing fault invalidated a trial and scored as a pass for
an injection that never landed.

## Item 3 — an expired blocker is NOT permission to move

**Answer: it never was able to grant motion, and it can no longer read as an
all-clear either.**

Motion is gated by the guards themselves (`estopped`, `pending`, the clearance
floor); **nothing anywhere gates motion on BlockMonitor state**, verified by
search. So an expiry could not resume motion.

But the expiry did set `active=False`, and every *consumer* read that as
clear — `preflight`'s "no active blockers" check would have PASSED on a
crashed guard and let a participant session start. Expiry is now a **third
state**:

| state | meaning | `blocked` |
| --- | --- | --- |
| active | the condition is held | True |
| **expired** | **the asserting loop STOPPED; condition UNKNOWN** | **True** |
| clear | live code says the condition is gone | False |

- `blocked` stays True while expired — "nobody is asserting it" is not
  evidence the condition went away.
- `blocking_aggregator` counts expired as blocking and reports `state_unknown`,
  so "nothing is blocking" can never be produced by a crashed asserting loop.
- `preflight` fails on an expired blocker.
- `ik_follower_node` registers `state_unknown` and **refuses to publish a
  trajectory** while any other blocker is expired. The check sits at the
  PUBLISH, not the callback entry: every `clear()` above it is what resolves
  an expiry, so refusing at entry would prevent the re-assertion that clears
  the unknown and would deadlock exactly as the original latch did.
  `state_unknown` is excluded from its own trigger, or it re-arms itself
  forever — the same latch in a new costume.
- An expiry resolves when the blocker is **asserted again** (the loop is
  running) or **explicitly cleared** by live code.

`src/srl_teleop/test/test_block_expiry_is_not_permission.py` pins all six
properties, including that a blocker re-asserted at 10 Hz for 5 s is never
expired. The tests use a fake clock: BlockMonitor stamps assertions with
`time.monotonic()` internally, so advancing only the argument to
`_expire_stale()` ages every blocker artificially and "proves" that a held
blocker expires.

## Injection results — 14/14

| fault | result |
| --- | --- |
| channel_dropout | handled |
| teensy_disconnect | handled (frozen 0.94 s, silent 0.97 s) |
| estop_during_motion | handled |
| wrong_object_latch | handled |
| ambiguous_flicker | handled |
| stale_object | handled |
| detection_failure | handled |
| logging_failure | handled |
| identifying_data | handled |
| nonmonotonic_clock | handled |
| **teensy_reconnect** | **handled** |
| **kortex_session_loss** | **handled** |
| **camera_zero_detections** | **handled** |
| **network_dropout** | **handled** |

32 unit tests pass. `test_flake8` and `test_pep257` fail, and **failed before
this work too** — the package uses double quotes throughout, against the ROS
style default. Pre-existing, not a regression.

## Still not verified without hardware

The Kortex session-recovery SEQUENCE is exercised through
`/real/session_state` and `/real/session_recover`, but the actual
close-then-reconnect against a real arm is untested — there is no hardware.
The one-session constraint is what makes it risky, and it is the part a real
run will exercise first.

---

# HARDENING PASS — CONSOLIDATED SUMMARY (2026-08-05)

**Read this section first if you are starting fresh.** It is the state of the
safety layer as of the end of the pass. Earlier sections marked SUPERSEDED
above are kept for their diagnoses, not their conclusions.

## The two defects that mattered, and why they hid for so long

### 1. The dead-man could not see the failure that actually happens

`master_pose_node` does **not** go silent when the master degrades. On a
validation failure it substitutes `last_good` and **keeps publishing at
50 Hz** — and it stamped that substituted frame `now`. So on the failure mode
that actually occurs (a Teensy emitting garbage rather than vanishing) the
topic never went quiet and the timestamp never went stale.

A dead-man keyed on **message arrival** therefore could not fire at all. Only
a clean unplug produced silence — which is why **every cable-pull test
passed**, and why the mechanism looked correct for months. The test and the
failure mode were different failures.

**Fixed at both ends.** `master_pose_node` stamps a substituted pose with the
time of the DATA it was built from (`src_time`), never publication time, and
publishes the data age in `status[7]`. `estop_node`'s dead-man keys on
`msg.header.stamp`. An empty stamp falls back to arrival and **warns** — an
e-stop that cannot be cleared is its own hazard.

| case (deadline = stale 0.50 s + 5 x 0.10 s = 1.00 s) | result |
| --- | --- |
| healthy master, fresh stamps, 243 frames | no trip |
| **degraded: publishing @50 Hz, FROZEN source stamp** | **trip 0.94–0.99 s** |
| clean unplug, topic silent | trip 0.97 s |

The trip line names which it was:
`source age 0.98 s, last message arrived 0.00 s ago -- master STILL PUBLISHING
stale data`. Comparing `age_s` against `arrival_age_s` on `/estop_deadman` is
what distinguishes a stale republisher from a silent one.

### 2. The e-stop HALT was 4 s late; detection never was

`trip()` called `halt_real_driver()` first, which did two
`wait_for_service(timeout_sec=2.0)` calls on `/real/controller_manager` — a
service that **does not exist on any sim or mock stack**. Both timed out in
full, inside the e-stop's own 10 Hz timer on a SingleThreadedExecutor. So
every e-stop blocked the whole node for **4.0 s** before parking the arms,
before logging, and before `/estop_state` could change.

Measured: decision at 0.97 s, trip logged at 4.98 s. Detection was always on
time. The halt was not, and nothing said so — the log line that would have
revealed it was itself behind the block.

**Fixed** by parking first (`halt_once()` + publish `/estop_state`, both local
publishes costing microseconds) and making the real-driver path non-blocking
(`service_is_ready()`; the calls were already `call_async`).
**Do not reintroduce `wait_for_service` there.**

Three smaller holes from the same trace: an arm that had never published
exempted itself from its own dead-man forever (only the left master usually
publishes, so the right arm's was permanently disarmed); a reset was measured
against staleness predating it, so a latch from a brief gap re-tripped
instantly and could not be cleared; and the follower's `estop` blocker was set
only inside `on_pose`, which stops being called at the exact moment the
dead-man fires.

## BlockMonitor auto-expiry — the structural fix, and how to read an expiry

Every blocker in this system is written as

```
if <bad>: blocks.block(N); return
blocks.clear(N)
```

so the `clear()` always sits on the fall-through path of the guard it reports
on. That is fine while the guard is mutated elsewhere, and **latches the
moment the guard also stops the callback running**. It happened five times,
the fifth inside the code written to catch the first four.

`scripts/audit_unreachable_clear.py` looks for the SHAPE, not the instances:
across 152 files it reports 6 occurrences, all in `ik_follower_node`,
0 LATCH-RISK. Rather than grade each one, `BlockMonitor` now **auto-expires**
any blocker not re-asserted for `EXPIRE_S` (2.0 s, comfortably above the
slowest 10 Hz asserting loop). A held condition is re-asserted every cycle by
design, so a blocker nobody is asserting is *abandoned*, not held.

**AN EXPIRY IS NOT AN ALL-CLEAR, AND A FOLLOWER MUST NOT READ IT AS ONE.**
An expiry means the loop asserting the blocker STOPPED RUNNING, so the guarded
condition is UNKNOWN — not gone. Reading it as clear would turn a crashed
guard into a green light, which is worse than the latch it replaced. Expiry is
therefore a **third state**:

| state | meaning | `blocked` |
| --- | --- | --- |
| active | the condition is held | True |
| **expired** | **the asserting loop stopped; condition UNKNOWN** | **True** |
| clear | live code says the condition is gone | False |

- `blocked` stays True while expired.
- `blocking_aggregator` counts expired as blocking and publishes
  `state_unknown`, so "nothing is blocking" can never be produced by a crashed
  asserting loop.
- `preflight` FAILS on an expired blocker — starting a participant session on
  an unknown safety state is the failure this whole mechanism exists to stop.
- `ik_follower_node` registers `state_unknown` and **refuses to publish a
  trajectory** while any other blocker is expired.
- Expiry resolves when the blocker is **asserted again** (the loop is running)
  or **explicitly cleared** by live code.

**Two placement rules, both load-bearing.** The follower's refusal sits at the
PUBLISH, not at callback entry: every `clear()` above it is what resolves an
expiry, so refusing at entry would prevent the re-assertion that clears the
unknown and would deadlock exactly as the original latch did. And
`state_unknown` is excluded from its own trigger, or it re-arms itself
forever — the same latch in a new costume.

Nothing anywhere gates motion on BlockMonitor state (verified by search);
motion is gated by the guards themselves. So an expiry never could resume
motion — the hole was that every *consumer* read it as clear.

## Six experiments, faults injected mid-trial — ALL PASS

`python3 scripts/inject_experiment_faults.py` — 8 scripted trials each, arm
link to the right arm dropped **while trial 3 is running**. E1–E6 all: trial
marked INVALID with the cause recorded, session continued to the next trial,
analyser excluded as pre-registered, and an all-invalid session fails loudly.

Cause recorded in every case:
`aborted mid-trial: link DOWN to right -- BOTH arms frozen, ...`

`srl_experiments/validity.py` is the single definition of "analysable",
shared so the six cannot drift. It excludes on the `valid` column, prints
every cause, and **exits non-zero** when no trials survive or more than
`MAX_INVALID_FRACTION` (0.5) were excluded. Before it, a session in which
every trial was invalidated analysed cleanly to `0 valid trials` followed by
empty means and NaN intervals — indistinguishable from a null finding.

**E6 had no run or analyse scripts at all**, only a config and protocol. Both
were written; `run_experiment.sh` now accepts `e6`.

### The harness bugs are the transferable lesson

Four separate bugs each produced a table of clean runs while testing nothing.
Any future injection harness will hit them:

1. **A fixed delay is not "mid-trial"** — trials run 0.4–2.5 s with gaps, so
   one offset landed inside a trial in some runs and in a gap in others.
2. **The publisher must already be discovered** — building the injector node
   after the trigger costs ~1 s of DDS matching, longer than a trial.
3. **State persists across runs** — the previous experiment's final
   `/trial_state` still satisfied "trial 3 is running", so the fault fired
   into the next run's startup. The trigger must name the current experiment.
4. **One-shot events are missable** — `/participant/abort` fires once, so a
   fault raised between trials was seen by neither. The runner now also
   consumes `/recovery_state` (continuous, 5 Hz) and ACCUMULATES faults seen
   during a trial, since a fault that recovers mid-trial would be gone from
   the current set by the time the trial ended.

And the assertion must require the **injected** cause: without that, an
unrelated standing fault invalidated a trial and scored as a pass for an
injection that never landed.

## Recovery paths (b)–(e)

`recovery_manager` (new) owns all four in one shape — **detect → freeze →
abort (trial INVALID, cause recorded) → recover** — and recover never means
relaunch, because a participant session cannot absorb one: the arm re-homes,
the condition order is lost and the block is discarded. `arm_link_monitor`
(new) supplies ping-based link health, separate so recovery can be tested
without faking a network outage.

Two parameters default OFF for sim, and the reasoning is the same as the
dead-man's: `expect_real_stack` (session/link paths fire only on an explicit
`connected:false` / `up:false`, never on silence — those topics simply do not
exist on a sim stack) and `freeze_on_master_loss` (in sim the master goes
stale constantly and freezing on it makes a healthy stack look like dead
hardware). **The trial is marked INVALID either way** — that is data
integrity and is never optional. Stopping the arm on master loss is the
dead-man's job.

## VERIFIED against mocks vs UNVERIFIED pending hardware

### VERIFIED, in sim / against mocks / by injection

- Dead-man trips on a **frozen-but-publishing** master (0.94 s) and on a
  silent one (0.97 s); no false positive over 243 fresh frames.
- E-stop halt latency: parks and publishes `/estop_state` immediately;
  the 4 s block is gone.
- Follower's named `estop` block appears in the aggregated view in 0.11 s with
  the master publishing and **0.50 s with the master silent** — the second
  case could not have passed before.
- BlockMonitor three-state semantics, 6 unit tests, including that a blocker
  re-asserted at 10 Hz for 5 s is never expired.
- **14/14 fault injections handled.**
- **6/6 experiments** invalidate correctly mid-trial, continue, exclude, and
  fail loudly when wholly invalid.
- Recovery paths (b)–(e) detect, freeze, abort with cause, and recover.
- 32 unit tests pass. `test_flake8` / `test_pep257` fail and **failed before
  this work too** (the package uses double quotes throughout, against the ROS
  style default) — pre-existing, not a regression.

### UNVERIFIED — needs the real arms

- **The Kortex close-then-reconnect against a real arm.** The sequence is
  exercised through `/real/session_state` and `/real/session_recover`, but
  never against hardware. The arm permits exactly ONE session, so a leaked one
  refuses the next connect — this is the part a real run will hit first, and
  the riskiest thing in the recovery layer.
- **Network dropout against real links.** `arm_link_monitor` pings
  192.168.1.10 / .9; only the injected topic has been exercised.
- **The dead-man against a real Teensy re-enumeration.** The frozen-data case
  was reproduced synthetically; the real board's behaviour on re-attach
  (ACM0 ↔ ACM1) has not been observed through this path.
- **Everything in "What is still NOT verified without hardware"** further up
  remains true: perception on real cameras, payload compensation, the
  complementary filter against a real gyro, the two-IMU path, the gated
  real-arm flow, and the right arm's home joint values.
- **The mount fix at the legacy home angles on the real arms.**
- **The 18.8% IK figure** that drives the "these arms cannot reach a table"
  conclusion. See `docs/NEXT_SESSION.md` item 1 — two harnesses disagreed
  wildly and this may be a measurement bug, not a mechanical fact.

**Nothing in this pass has ever run against a real arm.**

---

# GUI, dual arm, and trajectory capture (2026-08-06)

## `teleop_gui` — one terminal for everything

`ros2 run srl_teleop teleop_gui`. Curses, 5 Hz, works over SSH and in WSL with
no X server. **Topics only — it never opens the serial port** (verified: no
serial import, and no `/dev/ttyACM*` fd while running). `master_pose_node`
resolves the port once and holds it; a second reader would steal frames.

BLOCKERS is the first panel, because "the arm is not moving and nothing says
why" is the failure this rig actually has. An EXPIRED blocker is shown as
loudly as an active one — it means the asserting loop stopped, so the
condition is UNKNOWN, not clear.

Pot colouring is **mode-aware**: a channel the current `position_mode` does
not consume can never be red. Reddening an unused dead j5 trains you to ignore
red, which is how a real dropout gets missed.

Controls go through **parameter clients**, never `ros2 param set` — that goes
via the daemon, which hangs on this box and reports success it has not earned.
Every change is read back and echoed `old -> new`; a failure is logged loudly.
Verified live: `max_vel_rad_s 0.6 -> 0.48` and `max_step_rad 0.35 -> 0.28` on
both followers, confirmed by an independent read-back, plus E-STOP and reset.

## Dual arm — a FIFTH collision found, and it was invisible in sim

`tcp/twist.*` and `reset_fault/*` were **hardcoded string literals in the
driver C++**, not in any xacro, so no URDF-level check could see them. Fixed
by composing a new `prefix` hardware parameter (`patches/0001`, `0002`);
`prefix` defaults to empty so single-arm behaviour is bit-identical upstream.
Miss the four comparisons in `prepare/perform_command_mode_switch` and the
interfaces export correctly but are never matched.

> `tf_prefix` already exists as a hardware parameter and looks like the natural
> place for this. It is **declared in the xacro and never read by the driver**.

**The fifth, and the one worth remembering:**
`robotiq_driver/src/hardware_interface.cpp` exports `reactivate_gripper` as a
literal too. The earlier fix prefixed only the GPIO **declaration** in the
xacro, so after it the URDF declared `left_reactivate_gripper` while the
driver exported `reactivate_gripper` — they disagreed. It went unnoticed
because `robotiq_driver` is `COLCON_IGNORE`d (missing `serial`) and **every
gripper test to date ran against `mock_components/GenericSystem`, which reads
the URDF and never runs that code**. Patched (`0003`); **it cannot be compiled
here**, so treat its C++ half as written and reviewed, never executed.

`scripts/audit_dual_arm_collisions.py` now checks all five shapes statically,
including C++ literals. Currently PASS. The sixth should be found by the tool.

Also fixed: `real_arms_highlevel.launch.py` passed `left_robot_ip`
unconditionally, so `arm:=right` would have opened a session against the LEFT
arm and homed it.

`start_real.sh` takes `arm:=left|right|both`, defaulting to **both**, and now
waits for one `HOMING COMPLETE` **per arm** — the first arm finishing used to
end the wait and enable the bridge while the second was still moving.

**Verified against mocks, `--mock arm:=both`:** six per-arm nodes started
(`mock_real_stack_*`, `real_homing_node_*`, `sim_to_real_bridge_*`), both arms
`HOMING COMPLETE ... WITHIN`, `REAL ARMS LIVE`.
**Two real Kortex sessions have never been opened.**

### The homing "stall" was a wrong exit test

Completion required every joint inside the **1 deg deadband**, while the
acceptance criterion is `home_tolerance_rad` = 0.05 rad (**2.86 deg**). A joint
resting at 2.5 deg is HOMED by the project's own definition and could never
satisfy the exit test, so homing ran on until the stall timer fired — exactly
the observed left-arm behaviour (under 1 deg at t=66 s, then j6/j7 held
2.4-2.7 deg and it hunted there). **The deadband's job is to suppress the
CORRECTION, not to decide when the arm is home.** Now completes on the
tolerance.

Three more, all in `real_homing_node`:

- **Hysteresis** — enter the band at `deadband_deg` (1.0), leave only past
  `exit_deadband_deg` (2.0). One threshold makes a joint stop just inside,
  drift out, get kicked, and overshoot back in: the 2.2-3.0 deg hunting.
- **OSCILLATING is reported separately from STALLED**, from error SIGN
  CHANGES. The arm IS moving; saying "stalled" sends you looking for a seized
  joint.
- **BELOW VELOCITY FLOOR** is a third diagnosis. At 2.5 deg error
  `kp*err = 0.5 * 0.0436 = 0.0218 rad/s`; if the arm cannot execute that, the
  error can never close. Detected as "commanded non-zero and not moving" and
  named, rather than timing out. `min_commandable_rad_s` is a parameter
  because the true floor is a hardware number nobody has measured yet.

Mock run: both arms completed at **0.0491 rad (2.81 deg), WITHIN** the
0.050 rad tolerance — outside the old 1 deg exit test, so the old code would
have declared STALLED there.

### Right arm with j3/j5/j7 dead — position survives, orientation does not

`src/srl_teleop/test/test_right_arm_spherical_survives_dead_rolls.py`, 6 tests:
spherical validates only j1/j2/j4 so the dead rolls never reject a frame (fk
mode rejects the same frame, which is why the right arm is unusable there);
reach responds to the j2/j4 bends; and the rolls move the tip by **at most
20.4 mm over the full roll range** against a 0.272 m master reach — not zero,
so "free" is the wrong word, but small enough that position survives.
**What it lacks is WRIST ORIENTATION**: j5 and j7 are the forearm and wrist
rolls, so wrist rotation is unobservable and `orientation_mode` cannot leave
`fixed` for the right arm whatever the IK does.

### FSR grippers

`fsr_gripper_node` survived the restructure intact (deadband 250, latch
1200/400, hold 0.5 s all unchanged) but **was not in any launch** — it had
only ever been started by hand, so a launched stack silently had no grippers.
Now started by `teleop.launch.py` under `grippers:=true`.

**Right gripper interface CONFIRMED exporting** with both patches applied:
`right_robotiq_85_left_knuckle_joint/position [available] [claimed]`, matching
the left, and `left_`/`right_reactivate_gripper` now separately prefixed. This
is against mock hardware; `tcp/*` and `reset_fault/*` do not appear at all
under `mock_components`, so **the 0001 patch cannot be verified in sim**.

## Trajectory capture — `src/srl_experiments/trajectory_capture/`

`capture_protocol.py` prints the plan and costs it without a running stack:
**80 segments, ~35 min** of recording (A isolation 14, B IMU 8, C directional
12, D combinational 16, E repeatability 24, F speed 6).

The four known-dead channels are recorded as **NEGATIVE CONTROLS**
(`A_left_j7`, `A_right_j3`, `A_right_j5`, `A_right_j7`). A flat trace is
positive evidence the CHANNEL is dead rather than the recorder broken — the
ambiguity that made a dead `/tf` look like a recorder bug — and re-running
after the wiring repair proves the fix against an identical protocol.

`record_trajectories.py` is button-gated, **each arm gated by the OPPOSITE
arm's button** (an arm's own button also toggles its clutch), with a
pre-capture check that refuses to start on dead `/joint_states`, absent
buttons or dead tf2.

`analyse_capture.py` emits the CALIBRATION REPORT. **Validated against
synthetic data with known ground truth**, which caught three real bugs:

1. **`C_left_up` matched the direction `left` before `up`** — the arm names
   collide with the direction names, so every up/down segment was filed under
   left (visible as `n=20` where it should be 3). Direction now comes from the
   CSV column, never from the label.
2. **Correlation included dropout zeros**, so a dropout-ridden channel
   correlated with anything that moved — measuring the fault, not crosstalk.
3. **The linearity R² was meaningless** (0.000 for every healthy channel): it
   regressed against sample index, measuring the shape of the operator's
   sweep, not the sensor. Replaced with `rail%` and `mono`, which are
   computable without an independent angle reference.

After the fixes it recovers ground truth exactly: gain `diag(+0.3,-0.3,-0.3)`,
the injected 10% off-axis leak, repeatability n=3 per direction, rate
dependence 0.18/0.30/0.42 m for slow/medium/fast, dropouts 13.7% on the
intermittent channel, and `fsr1 vs fsr2 r=+0.9990` flagged HIGH.

## What is NOT verified

- **Two real Kortex sessions.** All dual-arm work is against mocks.
- **`patches/0003` C++ half cannot even be compiled here** — `robotiq_driver`
  is `COLCON_IGNORE`d for a missing `serial` package.
- **`patches/0001`** builds, but `tcp/*` and `reset_fault/*` are not exported
  by `mock_components`, so the prefix fix is unexercised in sim.
- **The velocity floor** — `min_commandable_rad_s` defaults to 0 (infer-only)
  because the arm's true minimum commandable speed has never been measured.
- **The capture protocol has never been run with a Teensy attached**; the
  analyser is validated on synthetic data only.

---

# CORRECTIONS AND FIXES (2026-08-06, post-capture)

## THE RECORDER OVERSAMPLED THE SENSOR, AND THAT FAKED EVERY PER-ROW STATISTIC

**Rows at 50.3 Hz; the pots update at 14.7–17 Hz.** So **82–88% of adjacent
rows are bit-identical**. Any statistic over consecutive rows is therefore
dominated by duplicates:

- median |Δ| over rows = **0.000 on all 14 channels**. That is the recorder,
  not the sensor. Over DISTINCT updates the real noise floor is **0.2–5.4°**.
- the "spread exactly 0.0 over 40 samples" signature that was used for years
  to identify a dead channel is **this artefact**, not evidence.

Every channel statistic must be computed between **distinct, time-adjacent**
sensor updates. `channel_report.py` does; nothing else may be trusted.

## TWO PRIORS THAT WERE WRONG FOR WEEKS

- **left j7 is the CLEANEST channel on the left arm** — 0% dropouts, 0%
  implausible jumps. It is NOT railed and NOT dead. Its range is only **26°**,
  so it is stiff or mechanically restricted, but it tracks. The old "75%
  railed / spread 0.0" verdict came from the oversampling above.
- **half the left arm is INCOHERENT** — j2, j3, j4, j5, j6. Worse than
  documented. j1 is INTERMITTENT (2.2% dropouts).

## INCOHERENT: a verdict that did not exist before, and the one that matters

A failing pot **does not go quiet**. It returns values spanning a wide range
that are **not a trajectory**: consecutive updates unrelated, jumping tens of
degrees in 60 ms. Range alone calls that healthy — which is how a broken
channel keeps getting trusted.

Rule: **>5% of updates jumping >60°** (≈2500°/s at the sensor's 15 Hz).

Verdicts, in order — DEAD (circular range <5° or dropouts >90%), INCOHERENT,
INTERMITTENT (dropouts >2%), ALIVE (coherent, range ≥20°, dropouts ≤2%).

**RANGE MUST BE CIRCULAR** — the smallest arc containing every sample, bounded
by 360 by construction. Unwrap-and-subtract is invalid for a rotary sensor:
across a dropout gap the unwrap cannot know how far the joint moved and the
offset runs away. That produced **314363°** on a 360° sensor.

**Reject outside [0,360] first.** 0.2–0.8% of samples are serial parse
glitches, in bursts up to 27 samples.

## BASELINE — 2026-08-06, 6 of 14 channels usable

| | verdict | | | verdict |
| --- | --- | --- | --- | --- |
| l_j1 | INTERMITTENT 2.2% drop | | r_j1 | ALIVE 195° |
| l_j2 | **INCOHERENT** 5% | | r_j2 | ALIVE 261° |
| l_j3 | **INCOHERENT** 8% | | r_j3 | **INCOHERENT** 14%, 80% drop |
| l_j4 | **INCOHERENT** 6%, 22% drop | | r_j4 | ALIVE 291° (5.0%, borderline) |
| l_j5 | **INCOHERENT** 12%, 40% drop | | r_j5 | DEAD 100% drop |
| l_j6 | **INCOHERENT** 8%, 70% drop | | r_j6 | ALIVE 250° |
| l_j7 | **ALIVE** 26° | | r_j7 | DEAD 100% drop |

Saved as `recordings/baselines/channels_20260806.json`.
Re-test after each repair: `bash scripts/check_channels.sh` (~3 min, block A
only, diffs against the baseline automatically).

**This is a soldering problem, not a software one.** Do not try to filter an
incoherent channel into usefulness — the sim is a RELAY, and whatever the
pots feed in the sim reproduces and the real arm copies a second later.

## THE 2026-08-06 CAPTURE IS SIM-ONLY AND MOSTLY CLUTCHED OUT

**No gain matrix, smoothing parameter, scale or lateral/gyro conclusion may be
derived from it.** Specifically:

- **The real arms never moved.** Capture 13:02:28–13:34:56; the newest bridge
  log was written 12:45:57 and no bridge was alive. The CSV has **no `real_*`
  columns at all**.
- **41 of 42 directional segments recorded with the clutch DISENGAGED** on the
  arm under test, sim EE span 0.000 m. Cause: `GATE_INDEX` in the recorder
  mapped each arm to its OWN clutch button, so every gating press disengaged
  the clutch being measured. The comment above it stated the correct rule and
  the code did the opposite.
- **The e-stop was latched for all of block F** and much of E-right, because
  the both-button trigger fired on any two button EDGES within 0.4 s and
  gating taps supplied them.
- Block A (per-joint) does NOT depend on the clutch and IS valid. That is the
  only part of the capture that supports conclusions.

## FIXED THIS PASS

- **`teleop_gui` crashed on launch** with no tty (`cbreak() returned ERR`,
  then the cleanup's `nocbreak()` ALSO failed and replaced the traceback) and
  under `TERM=dumb` (`curs_set()`). Both now refuse with an actionable
  message. **The old test passed on broken code** because it drove the program
  through `script -qec`, which allocates a pty, and set `TERM=xterm` — it
  constructed exactly the environment where the bug cannot occur, then checked
  only for panel captions, never the exit code or a traceback.
  `test_teleop_gui_launches.py` runs the real entry point and asserts on exit
  status and the absence of a traceback; verified to FAIL on the old code.
- **GUI control keys blocked the draw loop** for seconds when a target node
  was absent, so the console appeared hung exactly when you were using it to
  find out why something was hung. Control actions now run on a worker thread.
- **Recorder**: `real_*` joint and EE columns added; out-of-range pot values
  rejected at capture time with `n_rejected_raw` recorded; per-arm
  `master_seq_*` counters that only increment when the payload CHANGES, plus
  `t_mono`, so duplicates can never again masquerade as samples.
- **Recorder gating**: refuses to start, and aborts mid-segment, on a
  disengaged clutch, a latched e-stop, a missing bridge when `--need-real`, or
  a dead required channel. The 2026-08-06 run lost 35 minutes to exactly these
  with nothing stopping it.
- **Both-button e-stop now needs a sustained simultaneous HOLD**
  (`both_hold_s`, 0.30 s) instead of two edges in a window. Gating taps can no
  longer fire it; squeeze-and-hold still does, and it is strictly harder to
  trigger accidentally.
- **Incoherence rejection at the master node** (`max_channel_rate_deg_s`,
  **800 °/s**). Derived from the four coherent channels (n=7126 updates):
  p50 0.40°, p90 6.25° (≈93 °/s), p99 102.6° (≈1531 °/s — already the glitch
  tail). 800 °/s sits between them. A rejected value is marked as a DROPOUT
  and flows through the existing mode-scoped validation — nothing is invented.

## RECOMMENDED: log on sensor CHANGE, not at a fixed rate

Between the two options, **event-driven on `/master_arm_raw_*`** is right:
a fixed 20 Hz still aliases against a 14.7–17 Hz sensor and reintroduces
duplicates, whereas logging on change makes every row an independent sample so
per-row statistics are valid with no dedup step. Keep a heartbeat row every
0.5 s so gaps stay visible, and keep `t_mono` for intervals.

## NOT DONE / INVALID

- **The reachability measurement in `scripts/measure_workspace.py` ran from
  the wrong pose.** Both sim arms were parked **155–166° from home** (left
  over from the capture), so "from home" was not measured. Left reported
  isotropy 0.15 / mean 0.251 m and right reported 0.000 m in all 26
  directions; **treat both as invalid** and re-run with the sim freshly
  launched and the followers stopped so the arm actually sits at home.
- Graduated collision response and indefinite clutch indexing were **not**
  verified this pass.

---

# WORKSPACE, COLLISION AND CLUTCH — MEASURED FROM A CLEAN SIM (2026-08-06)

Supersedes the invalid reachability figures noted above. Those were taken with
both arms parked **155–166° from home**; these were taken with the sim freshly
launched (`srl_moveit_config demo.launch.py`, no `srl_teleop` nodes at all, so
nothing commands the arms) and **both arms verified at 0.0000 rad from home**.

`scripts/measure_workspace.py` now **refuses to sample** unless every joint is
within `--home-tol-rad` (0.05) of home, and prints the check. That assertion is
what was missing; without it the tool happily measured the wrong question and
reported it under the right name.

## Continuous reachability from home — 26 directions, 1 cm steps

Each step seeded from the previous solution, exactly as `ik_follower_node`
does. Full per-direction results: `recordings/baselines/workspace_20260806.json`.

| | LEFT | RIGHT |
| --- | --- | --- |
| front | 0.160 (collision/IK) | 0.100 (collision/IK) |
| back | **0.500 (max range)** | **0.500 (max range)** |
| left | 0.450 (collision/IK) | 0.140 (collision/IK) |
| right | 0.240 (collision/IK) | **0.500 (max range)** |
| up | **0.500 (max range)** | **0.500 (max range)** |
| down | 0.230 (collision/IK) | 0.190 (collision/IK) |
| worst of all 26 | 0.060 m | 0.090 m |
| mean of all 26 | 0.354 m | 0.325 m |
| **isotropy (min/max)** | **0.12** | **0.18** |
| binding | collision/IK ×16, max range ×9, step guard ×1 | collision/IK ×16, max range ×9, step guard ×1 |

**Collision/IK binds in 16 of 26 directions on both arms** — reach is limited
by the wearer and by joint feasibility, not by arm length. Nine directions ran
out of sweep range (0.50 m) without binding at all, so the true reach in those
is larger and untested.

**The usable fraction is the number that matters for scaling.** At scale 1.0
against a 0.272 m master ball, only the GUARANTEED sphere counts — the radius
reachable in *every* direction:

| | guaranteed sphere | usable fraction at scale 1.0 | scale mapping master → guaranteed |
| --- | --- | --- | --- |
| left | 0.060 m | 22% | **0.22** |
| right | 0.090 m | 33% | **0.33** |

Those scales are what maps the operator's full range onto the sphere reachable
in every direction. They are far below the shipped 1.0 — but note this is the
*conservative* reading: the mean reach is 0.33–0.35 m, so a scale near 1.0 is
right for most directions and wrong only near the worst ones.

**RUN-TO-RUN VARIANCE IS REAL.** Two consecutive runs from the identical home
pose gave left isotropy 0.28 then 0.12, and left worst 0.140 m then 0.060 m.
TRAC-IK uses random restarts, so a direction near the feasibility boundary can
fall either way. Treat single-run figures as approximate and repeat before
acting on a small change.

## Graduated collision response — HELD, no breach, no lockup

`scripts/probe_collision_response.py` drives the commanded EE straight at the
head (world 0,0,1.295) and torso (0,0,1.220) centres, holds it inside the body
for 6 s, then withdraws it.

| arm | zone | min clearance | floor | clearance-floor blocks | IK after retreat |
| --- | --- | --- | --- | --- | --- |
| left | head | 0.0791 m | 0.05 | 0 | +189 |
| left | torso | 0.0752 m | 0.05 | 0 | +188 |
| right | head | 0.0730 m | 0.05 | 0 | +173 |
| right | torso | 0.0722 m | 0.05 | 0 | +173 |

**The hard floor was never reached.** Zero clearance-floor blocks: the earlier
stages — collision-aware IK and redundancy re-seeding — stopped the arm
0.072–0.079 m away, so the floor stayed a backstop rather than the mechanism.
And every zone **resumed** on retreat (+173 to +189 IK successes), so blocking
degraded rather than latching.

## Indefinite clutch indexing — UNBOUNDED

`scripts/probe_clutch_indexing.py`: 8 cycles of 0.12 m, re-basing the reference
to wherever the arm actually is after each cycle, which is what a re-engage
does.

| arm | total EE travel, 8 cycles | per-cycle gain | verdict |
| --- | --- | --- | --- |
| left | **0.914 m** | 0.120 m (0.074 on cycle 1) | UNBOUNDED |
| right | **0.960 m** | 0.120 m every cycle | UNBOUNDED |

Travel is ~8× a single ball and every cycle in the second half still advances,
so the reachable set is bounded by the ROBOT, not by one ball of master travel.

**What this probe does NOT cover:** the clutch state machine itself — button
edges, the quasi-static gate, the 5-frame averaged reference — lives in
`master_pose_node` and needs a Teensy and physical presses, so it cannot be
verified unattended. What is verified is the downstream property that makes
indexing worth having: successive anchored offsets accumulate instead of
snapping back.

---

# REACHABILITY, N=10 — the measurement IS repeatable; the STATISTIC was not

`scripts/measure_workspace.py --repeats 10 --max 0.90`, both arms verified at
**0.0000 rad from home** (the assertion refused a first attempt where the
collision/clutch probes had left the arms elsewhere — it works).

**IQR = 0.000 m on every direction, both arms.** Ten sweeps give identical
quartiles. The factor-of-two instability seen earlier was **not** measurement
noise: it was the choice of statistic.

**What actually varies.** 9 of 26 left directions and 3 of 26 right show
`min << median` — a rare early IK failure on 1–2 sweeps of 10 (e.g. left
`-1-1-1` median 0.700, min 0.220; right `+1-1+1` median 0.490, min 0.000).
Isotropy computed as *min-over-directions of a single sweep* is the statistic
most sensitive to exactly that outlier, which is why it read 0.28 then 0.12.
**On medians it is stable.** Use medians for anything that sets a parameter.

| | LEFT | RIGHT |
| --- | --- | --- |
| isotropy on medians | **0.16** | **0.10** |
| worst direction (median) | 0.140 m | 0.090 m |
| median of medians | 0.400 m | 0.340 m |
| mean of medians | 0.443 m | 0.403 m |
| still truncated at 0.90 m | 2 dirs | 3 dirs |

Truncation is now immaterial: the Gen3's reach is **0.902 m**, so a direction
reaching 0.90 is bound by the arm itself. Isotropy remains a slight lower
bound but the bias is small.

## Scale — and why "scale to the median" is not the answer either

| scale | left: 1-sweep / needs indexing | right: 1-sweep / needs indexing |
| --- | --- | --- |
| 0.50 | 26 / 0 | 21 / 5 |
| 0.75 | 21 / 5 | 16 / 10 |
| **1.00** | **17 / 9** | **14 / 12** |
| 1.25 | 15 / 11 | **13 / 13** |
| 1.47 | **13 / 13** | 12 / 14 |

Scaling to the worst direction gives 0.51 (left) / 0.33 (right) and wastes
most of the reach, as noted.

**But scaling to the median gives 1.47 / 1.25 and leaves 13 of 26 directions
needing a second sweep — 50% by construction**, because the median is by
definition the half-way point. "Let the clutch handle the few that fall short"
does not describe half of them.

**Recommendation: 1.0, unchanged, for now.** It reaches 17/26 (left) and 14/26
(right) in a single sweep, and there is a second reason not to raise it:
**scale multiplies the master signal, and 7 of 14 master channels are
currently INCOHERENT.** Amplifying by 1.47 amplifies the garbage too. Revisit
scale after the wiring repair, when the input is worth magnifying — at that
point 1.25–1.47 is defensible, with indexing covering the rest.

Full per-direction medians/min/max/IQR:
`recordings/baselines/workspace_n10_20260806.json`; with retry (below),
`workspace_n10_retry_20260806.json`.

## The "arm sticks" question — measured, and it is NOT operator-visible

TRAC-IK's random restarts do produce false negatives, but the rate is tiny and
the follower already absorbs them.

**Per-call false-negative rate on a KNOWN-solvable target** (same target, same
seed, 20-30 calls each, all 26 directions):

| reach fraction | left | right |
| --- | --- | --- |
| 50% of median | 0.19% | 0.00% |
| 80% | 0.00% | 0.00% |
| 95% | 0.19% | 0.00% |
| 100% | 0.00% | 0.40% |

**0.0-0.4%, sporadic, and NOT concentrated** — never the same direction twice,
and no rise toward the reach boundary. The worst any single direction showed
was 5% (1 call in 20).

**Raising the solver timeout does nothing.** Requested 5 / 20 / 50 / 100 ms all
returned a median solve time of **6.5-7.0 ms**. The request timeout is ignored
on this path; `kinematics_solver_timeout: 0.005` in `kinematics.yaml` governs,
and 5 ms is already sufficient.

**The real mechanism is COMPOUNDING, and it was in the measuring tool.**
A sweep is a chain of up to 90 calls and any one failure ends the direction
early: at p=0.002 per call, P(at least one failure in 70 calls) is about 13%,
which matches the 9-of-26 early terminations observed over 10 sweeps.
`ik_follower_node` never suffers this because on failure it re-seeds the
redundant DOF (joint_3) and retries up to `redundancy_samples` (6) -- so the
effective rate the operator sees is ~0.002^7, i.e. never.
`scripts/measure_workspace.py` had no retry, making it MORE brittle than the
system it characterises.

**Fixed by giving the sweep the follower's retry.** Early terminations
**left 9 -> 0, right 3 -> 0**, and the medians barely moved (isotropy 0.16 and
0.10 unchanged; only left `-1+0+1` rose 0.830 -> 0.900, to the arm's own
limit). That the medians were already right is the confirmation that the
outliers were chain-compounding, not geometry.

**One residual cost worth knowing.** When the retry does fire it can spend up
to 7 x 6.7 = 47 ms, which exceeds the 20 ms budget at 50 Hz and consumes the
whole 47.6 ms at 21 Hz. At a 0.2% rate that is roughly once every 10 s. The
follower calls IK ASYNCHRONOUSLY behind the `pending` guard, so this appears
as one or two skipped poses rather than a stall -- a hitch, not a stick.

---

# PART 3 — TASK SCENE IN SIM, AND A FINDING THAT STOPS THE BIMANUAL SET (2026-08-07)

## Built

`srl_experiments/task_scene.py` publishes each task's objects to
`/planning_scene` as **CollisionObjects**, not markers. A marker is
decoration: `avoid_collisions=True` cannot see it, so the arm sweeps through
a table that looks solid on screen, and rehearsing against decoration teaches
a motion that will collide on the real rig.

Grasped objects are **attached to the gripper link**, so a carried block is
collision-checked against the wearer instead of passing through their chest.
The trigger is a gripper CLOSING TRANSITION into the holding band, never a
closed gripper — `gripper_state()` calls a fully-closed gripper `free_air`
because closing on nothing is not a grasp, and the mock boots at 0.79 rad,
which is what made every trial after the first log an instant grasp in the
original pilot.

`layout.yaml` per task carries the protocol's numbers verbatim, plus every
grasp point and placement target with the arm the protocol assigns to it.

## THE FINDING: THE TWO ARMS SHARE NO WORKSPACE AT ALL

`python3 scripts/verify_task_scenes.py` — arms verified at home to 0.0000 rad.

    t1 0/2   t2 0/5   t3 0/6   t4 0/3   t5 0/3      -- 0 of 19 targets

Not a scene-collision problem and not an orientation problem: every target
failed with collisions OFF and with yaw sampled. So it is reach.

**Home EE: left (+0.697, +0.248, +1.146), right (-0.763, +0.298, +1.179).**
The hands park **1.46 m apart**, far out to the sides at chest height, while
every task layout is centred in front of the wearer and spans 0.62 m in x.
14 of 19 targets are beyond the arm's 0.902 m reach outright (mean distance
0.979 m).

Translating the whole layout does not rescue it — best 3/5 for T2, at
dx = -0.55 m, because a shift that helps one arm moves the other's targets
further away.

**The decisive measurement, a 7 x 3 x 3 grid over the frontal volume:**

| | |
| --- | --- |
| cells reachable by BOTH arms | **0 of 63** |
| left only | 16 (all at x >= +0.4) |
| right only | 18 (all at x <= -0.4) |
| neither | 29 — including **the entire centreline, x in [-0.2, +0.2]** |

    z = 1.10 m
        y\x   -0.6   -0.4   -0.2   +0.0   +0.2   +0.4   +0.6
       +0.2    R      R      .      .      .      L      L
       +0.4    R      R      R      .      .      L      L

**No bimanual task is performable in this configuration.** T2, T3, T4 and T5
all require either a shared workspace or an inter-arm handover, and the two
reachable sets are disjoint with a dead band between them. Only T1
(independent reach, one target per arm) could run, and only with its targets
moved out to each arm's own side.

This also explains the operator's original symptom directly: **"does not
reach toward the centre" is not a control fault — the centreline is reachable
by neither arm.**

The cause is the home JOINT ANGLES, which are ground truth and were not
touched: CLAUDE.md already records `|v_R - M v_L| = 1.3837 m` and that the
two arms are parked asymmetrically, with the residual **provably independent
of the mount rotation**. Fixing this needs the right arm re-parked in
hardware, not a mount change and not a layout change.

**Do not re-run the bimanual protocols until the arms share a workspace.**
The task scenes and the verifier are built and will answer the question again
in one command once the parking is fixed.

---

# MOUNT GEOMETRY IS BLOCKING REAL TASKS (2026-08-07)

## First, a correction to my own earlier framing

The "0 of 63 cells reachable by BOTH arms" result was used to conclude the
bimanual programme was blocked. That test asks whether both arms can reach
**the same point** — which is what T4 handover needs, and nothing else does.
T3 needs two **distinct** points 300 mm apart, one per arm, which is a
strictly weaker requirement. Measured properly, with collisions ON:

| task | verdict | evidence |
| --- | --- | --- |
| T1 bimanual reach | **FEASIBLE** | symmetric pair reachable at x = +/-0.30, y 0.1, z 1.0 |
| T2 hold and fill | **BLOCKED** | no (hold, release-100 mm-above) pair exists anywhere in a 128-point search. It needs one arm holding and the other reaching directly above — effectively a shared region |
| T3 coordinated carry | **FEASIBLE, RELOCATED** | two grips 300 mm apart ARE reachable, in a band xc -0.05..+0.05, y 0.35..0.40, **z 1.10..1.30** |
| T4 inter-arm handover | **BLOCKED** | 0 of 16 transfer points reachable by both arms; 0 of 63 grid cells |
| T5 handover to wearer | **FEASIBLE** | right arm reaches (-0.16, 0.35, 1.05) |

**T3's feasible band is at CHEST height (1.10-1.30 m), not the protocol's
table height (0.885 m).** The tray needs a stand, not a table. Nine feasible
placements were found, so it is a band and not a knife edge.

## Is the disjointness the wearer or the kinematics? BOTH, in that order

    collisions ON  : both  0/63   left 16/63   right 18/63
    collisions OFF : both  3/63   left 26/63   right 28/63

Only 3 shared points exist kinematically, and **the wearer removes all
three**. So the mount matters, and a sweep is worth running.

## MOUNT SWEEP — 108 candidates, REPORT ONLY, nothing applied

`scripts/sweep_mount_overlap.py`. Moving a base by T is exactly equivalent,
for reachability, to moving the target by T^-1, so one running move_group
answers every candidate with no URDF edit.

**The trick does NOT preserve wearer collision** (the wearer does not move
with the mount), so the sweep is purely KINEMATIC — an upper bound. Any
candidate worth pursuing needs a real URDF check before its clearance is
trusted, and the best candidates move the arms INBOARD, FORWARD and LOWER,
i.e. **towards the person**. That tension is unresolved and is the first
thing to check if anyone acts on this.

| | separation | both/63 | front L/R | T3 tray |
| --- | --- | --- | --- | --- |
| **current** | 0.40 m | 3/63 | 0.16 / 0.10 | yes (kinematic) |
| **best buildable** dx -0.10, dy +0.15, dz -0.15, tilt -60 | **0.20 m** | **22/63** | **0.42 / 0.30** | yes |
| best overall dx -0.20 | **0.00 m** | 29/63 | 0.20 / 0.06 | **NOT BUILDABLE** — two Gen3 bases (~92 mm radius plus bracket) would occupy the same space |

Each parameter alone, from the current mount:

    forward tilt    +0 -> 3    -30 -> 6    -60 -> 12
    mounts inboard  +0 -> 3   -0.10 -> 6  -0.20 -> 14
    mount forward   +0 -> 3   +0.15 -> 4  +0.30 -> 6
    mount lower     +0 -> 3   -0.15 -> 5
    splay INWARD    +0 -> 3    -25 -> 0     <- HURTS, do not splay

**So yes, the mount can be improved**: 3 -> 22 of 63 shared points and front
reach 0.16/0.10 -> 0.42/0.30 m, within a buildable 0.20 m separation. It is
not a rescue of T4 (22/63 is kinematic, before the wearer) but it is a large
change and it makes T2 worth re-testing.

**Not applied.** A mount change invalidates P_HOME, the workspace anchor and
every clearance figure.

# PERCEPTION MODELS — INSTALLED AND FITTED, DETECTION STILL UNMEASURED

`.percep_venv` (use `--ignore-installed sympy`; `--system-site-packages`
otherwise collides with the system sympy and matplotlib's numpy ABI).

| | measured |
| --- | --- |
| GPU | RTX A500 Laptop, **4096 MiB**, 783 MiB baseline |
| faster-whisper small/int8 | load+warm **6.0 s**, VRAM **1222 MiB** |
| YOLO-World-s | load+warm 70 s (first run downloads 338 MB) |
| **both resident** | **2012 MiB of 4096** — fits, ~2 GB headroom |
| detector latency | ~23 ms/frame on GPU |

## The detection number is NOT available, and the 1% figure is my harness

`scripts/measure_detection.py` returned **0-4%** on the task objects. Before
reporting that, I checked the model against a real photograph:

    REAL PHOTO  ['bus','person']      -> 7 detections, conf 0.89-0.91
    SYNTHETIC   ['green cube']        -> 0 detections
    SYNTHETIC   ['box','square'] @0.01 -> 0 detections

**The model is fine. My renderer is out of distribution** — flat coloured
rectangles on a flat grey background are nothing like the photographs
YOLO-World was trained on. This is the AprilTag mirrored-renderer bug in a
new costume: a synthetic harness that measures itself.

So **detection rate at working distance remains UNMEASURED**, and the 95%
gate is neither passed nor failed. It needs photorealistic rendering or the
real Kinova cameras. **Mode 6 is not participant-ready and this is why.**

Also found: ultralytics 8.4.116 + torch 2.13 has a device-placement bug —
calling `set_classes()` after the model is on CUDA raises
`Expected all tensors to be on the same device`. Set classes BEFORE the first
GPU predict, or run on CPU.

# SAFETY BUGS — both fixed structurally

**(a) Forbidden targets can no longer be candidates.** The keep-out moved
INTO `WorldModel.match(exclude=...)`, so a forbidden object is never returned
to any consumer. Filtering after resolution left the ambiguous branch free to
offer a position the robot must never reach, and asking the operator to
choose an option that will then be refused invites them to say "the far one"
and trust it. `count_excluded()` lets the refusal still name the real reason
rather than saying "I can't see anything". 9 tests, four forbidden positions
across three distinct reasons (behind, head, torso).

**(b) The wake word gates properly.** A no-wake utterance is silent; only an
utterance carrying the wake word earns "Sorry, I didn't understand". Verified
with the executive PROVEN ALIVE first — the harness now sends a probe command
and refuses to report at all if nothing answers, because the previous version
passed this check while nothing was running.

---

# STANDING RULE: VALIDATE THE INSTRUMENT BEFORE REPORTING A BAD MEASUREMENT

**Four times now a "system failure" has turned out to be the measuring tool.**

| reported | actually |
| --- | --- |
| noise floor 0.000 on all 14 channels | the recorder oversampled a 15 Hz sensor at 50 Hz, so 82-88% of adjacent rows were bit-identical |
| isotropy swinging 0.28 -> 0.12 between runs | min-over-directions is the statistic most sensitive to a rare tail failure; on medians it is stable and IQR = 0.000 |
| detection 0-4% on the task objects | flat-shaded primitives are out of distribution; the same model scores 0.89-0.91 on a real photograph |
| T3 carry path 0 of 19 placements | the path template carried toward the wearer, the one direction that leaves the feasible band |

Each was caught ONLY by checking the instrument against a known-good
reference. None announced itself.

## The rule

**Before reporting a bad measurement, validate the measuring tool against
ground truth.** A surprising failure is evidence about the instrument until
the instrument has been cleared.

Concretely:

1. **Every analysis script gets a known-answer test.** Feed it an input whose
   answer you know and confirm it comes back.
2. **Prefer REAL data with known ground truth.** Replaying a recorded capture
   whose properties were established independently beats anything generated.
3. **Synthetic ONLY where the ground truth is CONSTRUCTED, not RENDERED.**
   Two points 300 mm apart really are 300 mm apart — arithmetic and geometry
   are trustworthy. A *rendered* image is only as good as the renderer, and
   that is exactly what produced the 0-4% detection figure.
4. **A result that contradicts an earlier measurement is an instrument check,
   not a finding**, until the two are reconciled. "0 of 19 paths" against "19
   working grip pairs" was the tell.
5. **Repeat before believing.** TRAC-IK has random restarts; a single-sample
   IK result is not a measurement. Three repeats minimum, and report the
   spread.

## COROLLARY: A POSE THAT PASSES ONE IK CALL IS NOT A REACHABLE POSE

**Verification is N repeats over the WHOLE PATH.** Both halves are load-bearing
and both were learned the hard way.

**N repeats, because TRAC-IK restarts randomly.** A pose at the edge of the
feasible set is a coin flip and one call is one flip. Measured on this rig:

| pose | true per-call feasibility | passes a 5/5 check |
| --- | --- | --- |
| T2 at x = +/-0.10 | 0-80%, varying with height | sometimes |
| (0.15, 0.35, 1.36) | **72-82%** | **19-38% of the time** |
| (0.15, 0.35, 1.34) | 100% (40/40) | always |

Every scenario in this project was verified at **k=2**, which lets a 60% pose
through 36% of the time -- often enough to be written into a protocol, rare
enough to look like bad luck when it fails on the day. N is now **10**, with
the check SHORT-CIRCUITING on the first failure so raising it costs almost
nothing on poses that already pass.

**The whole path, because the arm flies the gaps.** Endpoints being reachable
says nothing about the straight segment between them. T3's S3 declared four
waypoints spanning 200 mm and left 150 mm unexamined between each pair; T5 was
verified at two points and never had a transit; T7's marker traces a Lissajous
on a SPHERE and only the centre and two points on a vertical line through it
had ever been checked. Every path is now densified to **20 mm** and T7 is
checked against all 26 directions of its amplitude shell.

**N IS NOT A SUBSTITUTE FOR MARGIN.** No finite N proves a pose reachable; it
only bounds how often the check lies. What saves this rig is that feasibility
falls off a CLIFF rather than degrading -- 100% at z=1.34, 82% at 1.36 -- so
the real defence is to keep every declared figure **at least 20 mm inside the
last pose that passed N/N**, and to use N to find that boundary.

    python3 scripts/audit_scenario_reachability.py --repeats 10

grades every pose SOLID (N/N) / MARGINAL (1..N-1) / FAIL (0), and **MARGINAL
is not usable** -- that is the T2 failure by name.

### And validate the audit before believing it

The first run of this audit reported every T3 waypoint unreachable. The
geometry was fine: the audit assigned the "left" arm to the -x end, and
`left_*` links sit at POSITIVE x in this model. Two more instrument bugs
followed -- re-rolling a static hold pose once per waypoint, which turns a
60/60 pose into a random failure somewhere along the path, and a bare
`safe_dump` in `verify_scenarios.py` that silently deleted the T2 scenarios
merged in by a different script. Fifth, sixth and seventh instances of the
standing rule above.

## Applied

`src/srl_experiments/test/test_coupled_metrics_known_answers.py` (21 tests)
and `test_t7_targets_known_answers.py` (7) validate every metric that will
touch participant data:

- sling sag reproduces the closed form the object spec was cut from
- tilt is verified height-independent (0.885 m table vs 1.15 m stand)
- path efficiency returns exactly sqrt(2)/2 for a known right-angle detour
- **the phase-lag estimator recovers an injected 140 ms delay**
- **the interference fit recovers an injected b_cross of 7.5 from 7.5**
- yoked speeds are shown to be UNIDENTIFIABLE, so the design cannot be
  quietly simplified into uninterpretability

The T7 target test caught a real bug before it reached a participant: the
Lissajous exceeded its amplitude by sqrt(1+0.35^2) = 1.06, so the marker could
leave the sphere that had been VERIFIED reachable — the "scenario that fails
IK on the day and wastes a participant" failure.

---

# MOUNT SWEEP — RECORDED, NOT APPLIED (2026-08-07)

Left unapplied deliberately. The sweep is **purely kinematic**: the
base-transform trick does not move the wearer, so these are UPPER BOUNDS, and
the best candidates move the arms INBOARD, FORWARD and LOWER — i.e. **toward
the person**. Any candidate needs a real URDF clearance check before its
numbers mean anything.

| | separation | both/63 | front L/R | T3 tray |
| --- | --- | --- | --- | --- |
| **current (applied)** | 0.40 m | 3/63 | 0.16 / 0.10 | yes |
| best buildable: dx -0.10, dy +0.15, dz -0.15, tilt -60 | 0.20 m | 22/63 | 0.42 / 0.30 | yes |
| best overall: dx -0.20 | 0.00 m | 29/63 | 0.20 / 0.06 | **NOT BUILDABLE** — two Gen3 bases would occupy the same space |

Each parameter alone, from the current mount:

    forward tilt    +0 -> 3    -30 -> 6    -60 -> 12
    mounts inboard  +0 -> 3   -0.10 -> 6  -0.20 -> 14
    mount forward   +0 -> 3   +0.15 -> 4  +0.30 -> 6
    mount lower     +0 -> 3   -0.15 -> 5
    splay INWARD    +0 -> 3    -25 -> 0     <- HURTS, do not splay

Full table: `recordings/baselines/mount_overlap_sweep.json`.

# THE EXPERIMENT PROGRAMME AFTER THE GEOMETRY (2026-08-07)

| task | verdict | evidence |
| --- | --- | --- |
| **T7 pursuit** | **IN** (new) | needs no shared point, no objects, no vision. 6/6 scenarios verified |
| **T3 rigid carry** | **IN, respec'd** | band is z 1.10-1.30, NOT the 0.885 m table. Transport must be **VERTICAL** -- lateral and fore-aft leave the band |
| **T6 compliant carry** | **IN** (new) | same paths, 350 mm sling, nominal separation 310 mm |
| **T5 handover** | **IN** | 3/3 scenarios verified |
| **T2 hold and fill** | **IN if the object is redesigned** | see below |
| T1 | subsumed by T7 | |
| T4 | **BLOCKED** | 0 of 16 transfer points; geometry, not orientation |

**17 of 17 scenarios verified** against a live `/compute_ik`
(`scenarios_verified.yaml`). Two were caught failing and fixed before they
could waste a participant: S3's detour passed through (0.0, 0.40, 1.26),
unreachable; and its start at z=1.10 fails at the 310 mm nominal separation
though it passes at 300 mm.

## T2 is NOT blocked — my earlier verdict assumed the object

The BLOCKED verdict came from requiring the release point 100 mm **directly
above** the hold point. That is a design choice. Sweeping the real design
space finds feasible pairs, e.g. left holds (+0.10, 0.35, 1.15) while right
releases (-0.10, 0.35, 1.20).

**Implied object:** an open-top container whose **opening centre is ~200 mm
horizontally from its grasp handle and ~50 mm above it** — a box on a side
handle, like a dustpan or saucepan, not a box gripped at the lip. The holding
gripper must be clear of the opening, and that clearance is exactly what
accommodates the arms' separation.

**It does not fit the 75 min session.** If wanted, it DISPLACES P6 (T5).
Do not extend the session: fatigue drifts monotonically because the master arm
has no gravity compensation.

## T3's transport is a LIFT, not a carry

Measured, 3 repeats per waypoint: grips and vertical lift 20/20; carrying
toward the wearer (y 0.35 -> 0.25) **0/20**. The feasible band is only
0.35-0.40 m deep in y, so a fore-aft carry leaves it immediately. Four
vertical carry paths verified, spanning z 1.10 -> 1.35.

## Pilot — 252 trials, all six blocks

`python3 scripts/pilot_bimanual.py`

    t3   72 trials    t6   72 trials    t7  108 trials
    counterbalancing  direct/assisted/shared each [2,2,2] across positions -> balanced
    validity          3 invalid, all with the cause recorded, excluded as pre-registered
    T3 tilt RMS       direct 1.097 -> assisted 0.608 -> shared 0.275 deg   (-75%)
    T6 sep err RMS    direct 7.698 -> assisted 4.219 -> shared 1.911 mm    (-75%)
    T7 interference   b_self 358, b_cross 289 mm per (m/s), R2 = 1.000
    dual-task cost    0.06 (unimanual 66.4 mm -> bimanual 70.5 mm)

The pilot's first version claimed the fit should recover the constants it
injected (90 / 60). It does not, and **the claim was wrong, not the metric**:
the generator injects speed sensitivity in the error amplitude AND in a
tracking lag whose positional error also grows with speed, so the fit
recovers TOTAL speed sensitivity. Exact recovery is pinned separately by a
unit test that feeds the fit a purely linear model and gets 7.5 back from 7.5.

---

# T7 b_cross — DEFINITION SETTLED, PIPELINE VALIDATED (2026-08-07)

The pilot injected 90/60 and the fit returned 358/289. That gap made the
headline T7 metric uninterpretable in absolute terms, so it is now resolved
two ways at once.

## 1. What b_cross measures, in words

**The TOTAL speed sensitivity of arm A's RMS tracking error to arm B's target
speed**, in mm of RMS error per (m/s).

"Total" is load-bearing. Tracking error has at least two components and BOTH
grow with target speed:

- an **amplitude** component — the operator tracks less accurately when
  attention is divided;
- a **lag** component — any fixed reaction delay `L` becomes positional error
  `v*L`, because the target has moved that far by the time the hand arrives.

**From RMS error alone these are not separable** — here or in any other
experiment; they produce the same statistic. So b_cross is DEFINED as the
total, and reading it as "the attentional cost" alone would overstate it by
whatever the lag contributes. **`phase_lag_s` is reported beside it for
exactly this reason:** b_cross large with a phase lag flat in the other arm's
speed means the effect is attentional; both rising together means it is not.

## 2. The pipeline now recovers a known answer END TO END

A unit test recovering 7.5 from 7.5 proves the ARITHMETIC. It does not prove
the PIPELINE, and the pilot is what must. `scripts/pilot_bimanual.py` gained a
CALIBRATION block whose RMS error is linear in the two speeds by
construction — fixed offset, no lag, no noise — run through the real
`summarise_pursuit -> interference_coefficient` path:

    injected   b_self = 90.0   b_cross = 60.0   mm per (m/s)
    recovered  b_self = 90.0   b_cross = 60.0   R2 = 1.0000   (n=16)

The realistic-operator generator in the same pilot still fits 358/289, and
that is now stated as expected rather than explained away: its error combines
an amplitude term and a lag term NONLINEARLY, so a linear fit returns the
total, which is the defined quantity. The pilot checks the ordering the
design predicts (b_cross > 0, b_self > b_cross) and fails the run if either
the calibration or the ordering does not hold.

**The rule this follows:** a metric that cannot recover a known answer end to
end is not ready, even if a unit test recovers it in isolation. That is the
measurement rule applied to the measurement rule.

# T2 vs T5 FOR THE P6 SLOT — DECISION RECORDED, AND I DISAGREED

Adding T2 exceeds the 75 min session, so it can only displace T5.

**I do not agree T2 is clearly the better use of the slot, and the protocol
keeps T5.** The study's framing is *wearable* supernumerary limbs with
embodiment as a headline measure. T5 is the only task in which the robot
SERVES THE WEARER — the defining use case — and the only one where the
ownership/agency measures have a functional referent rather than an aesthetic
one. It is also the only single-arm block, so it is the only one that still
runs if the bimanual geometry degrades further.

T2's genuine advantage is a DISCRETE success/failure outcome where T3, T6 and
T7 are all continuous. But T3 and T6 already log "object retained" as a
binary, so that measure is not absent without T2.

**Revisit if the paper's claim turns out to be about coordination under
divided attention rather than about embodiment** — then T2 is the better
block. The trade is written into `docs/research/04_protocol.md` so the
reversal, if it happens, is visible as a deliberate one.

---

# TWO-PERSON REFRAMING (2026-08-08) — operator and wearer are different people

**The arms are worn by one person and driven by a DIFFERENT person.** This is
not a change of emphasis; it changes who is a participant, what is measured,
and which literature the work sits in. Engineering is unaffected — the arms,
the master, the safety stack and every coordinate are the same — but anything
describing "the operator" as the person wearing the pack is now wrong.

Research docs, in reading order:
`docs/research/01_literature_review.md` §7 (Fusion, SRL Proxemics, the gap),
`02_baseline_and_hypotheses.md` §6 (H1-H4 for the dyad),
`05_two_person_measures.md`, `06_task_set_two_person.md`,
`07_session_order_two_person.md`, `03_ethics_and_safety.md`.

## The gap, corrected twice — do not overstate it

Fusion (SIGGRAPH 2018 E-Tech) is the direct predecessor and its architecture
is ours. Two corrections to the obvious novelty claim, both of which must
survive into any write-up:

- **Fusion's Direct / Induced / Enforced are levels of PHYSICAL COUPLING to
  the wearer's own arms, not levels of machine autonomy.** In all three the
  robot decides nothing. It is also a 2-page demo with **no user study and no
  numbers** — cite it for existence, never for performance.
- **The gap is NOT untouched.** SRL Proxemics (CHI 2026) already ran a
  separate concealed operator, varied autonomy, and measured wearer safety and
  trust — finding **higher autonomy made both WORSE** (SCR Mdn 0.85 vs 0.25,
  p=.002; trust 4.18 vs 5.42). Its autonomy was a hidden human and its wearer
  had no task, so what remains open is performance-and-experience together,
  not "nobody has looked".

Likewise **T9's disturbance compensation is partly solved already**: Zhang et
al. (2024) report 1.37 ± 0.58 mm with the human shoulder as floating base.
What is untested is the **two-person** case, where the disturbance comes from
a nervous system the operator has no efference copy of.

## New tasks, verified

| | |
| --- | --- |
| **T8 wearer-assisted reach** | 4/4 verified, **2 per arm, 3 distinct stances**. A target unreachable at nominal stance and reachable after the wearer repositions |
| **T9 reach under wearer motion** | 4/4 verified at (±0.35, 0.35, **1.15**). **IV capped at 100 mm** — 160 mm ("step in place") leaves the reachable set at every centre probed and was REMOVED rather than left to lose trials to geometry |

`scripts/verify_t8_t9_scenarios.py`. Wearer stance is simulated by
transforming TARGETS, which is exact kinematically but does **not** move the
wearer's collision geometry — an upper bound on reach, a lower bound on
clearance.

## THE CLEARANCE FINDING — IK-valid is not clearance-valid

Recording every task surfaced something no amount of IK verification would:

> **T3/T6 carry paths bring the arms within 48-62 mm of the wearer's torso.**
> Contact-free, IK-valid, and **below the 120 mm floor `real_robot` enforces.**

MoveIt's `avoid_collisions` is **binary contact**; the real-robot floor is a
**margin**. A scenario can pass one and be refused by the other, and this pair
does. Consistent with CLAUDE.md's own "through-range clearance +0.0495 /
+0.0544 m" — the number was known, but had never been connected to the task
scenarios.

**RESOLVED by measurement** (`scripts/probe_task_band_clearance.py`):

- the closest link is **`spherical_wrist_1_link`** (9/12 probes), not the
  forearm -- so the TARGETS are too close to the torso, not the path;
- **forward in y does NOT work**: y=0.45 clears 0.131 m but only 1 of 6 poses
  is reachable, and 0.50+ is unreachable. Clearance and reach conflict in y,
  exactly as they do for the mount;
- **outboard in x DOES work**: half-span 0.155 -> **0.25 m** gives 0.134 m
  clearance at **6/6 reachable**, and every value from 0.25 to 0.40 clears.
  Corroborated by the recordings: T7/T8 work at |x| >= 0.40 and measured
  0.158-0.187 m over 30 clips, none below the floor.

**NOT APPLIED.** A 0.25 m half-span is a **500 mm tray**, and that propagates:
T6's 350 mm sling retains only to 340.7 mm separation so the ball would be
gone before the trial starts (needs L >= 507 mm); T3's tilt threshold
rescales, since 60 mm over 500 mm is 6.8 deg not 11.3; and T2's container
opening moves from 300 mm to 500 mm from the handle. One deliberate pass with
full re-verification, as with the mount.

**T5 is exempt and must stay so.** It measured 0.068-0.095 m, but T5 is a
handover TO the wearer -- the arm is meant to reach their waist. Low clearance
there is the task, not a defect, and it needs a scoped exemption rather than a
moved target. Lowering the floor globally is not available: it is the last
thing between the arms and the chest of someone who did not choose the
motion.

## Verification recordings

    python3 scripts/record_verification.py --all     # every task x scenario x condition
    python3 scripts/make_clip_index.py               # -> recordings/verification/INDEX.md

Two panels per clip: a 3-D view and a **front view**, because the 3-D view
cannot show whether two grippers are level and that is the whole of the T3/T6
metric. The wearer is drawn from `human_backpack.xacro`, the object is drawn
(rigid tray as a straight line, sling with its closed-form sag and a ball that
turns red when released), and clearance is delegated to the project's OWN
`srl_teleop/clearance.py` so the clip and the numbers cannot disagree.

**The conditions in these clips are smoothing levels, not the autonomy stack.**
They verify geometry, motion and clearance. No autonomy claim may be read off
them, and INDEX.md says so at the top.

### Four instrument bugs, all caught before their output was believed

1. **My clearance model was invented** — hardcoded world-frame boxes reporting
   0.064 m where the shipped guard reports far more. Two clearance metrics are
   worse than one; it now calls `clearance.py`.
2. **The tracking metric included the approach from home**, where the command
   is deliberately far ahead of the arm: 224 mm on a run that tracks to
   **2.1 mm** once moving. Frames are now phase-tagged and scored on the task
   phase only.
3. **Paths were too short and too fast** — a 3-frame clip of an arm asked to
   cross half a metre in 0.1 s.
4. **A per-frame matplotlib 3-D axes** put the full sweep at three hours;
   reusing one figure brought it to ~38 s per run.

Fifth instance of the standing rule, and the reason it is a rule.

---


---

# APPENDIX: the CLAUDE.md header as it stood before the 2026-08-12 split

Kept verbatim because the split promised nothing would be deleted. The new
CLAUDE.md rewrites this material rather than copying it, so the COUNTS below
are the ones that were current on 2026-08-10 and are NOT maintained. Re-measure
with the commands in the table rather than quoting them.

# kortex_ws — shared autonomy for a wearable supernumerary limb

## HOW TO RUN THINGS

Source the workspace first (every terminal):

```
source /opt/ros/jazzy/setup.bash && source ~/kortex_ws/install/setup.bash
```

## WHICH GUI TO RUN

```
ros2 run srl_teleop gui           # Qt operations GUI  <-- RUN THIS
ros2 run srl_teleop teleop_gui    # curses            <-- only over SSH / no display
```

Not sourced? `python3 ~/kortex_ws/scripts/srl_gui.py`

**`gui` IS PRIMARY. `console` IS NOT, AND SAYING OTHERWISE COST WEEKS.** This
file named `console` as primary from 2026-08-08 to 2026-08-10 while every
piece of GUI work went into `srl_gui.py` -- the master-arm schematic, the
robot schematic, the commanded-vs-actual divergence readout, the real-robot
view, the dark HUD, the A/B/C task buttons and the indicator self-test. None
of it is in `console`, which has not been touched since the restructure. A
reader following this file ran a GUI that received nothing, reported the new
panels as missing, and was right.

    gui         Qt5. Master-arm and robot schematics, divergence, both camera
                feeds, 25 launch buttons, indicator self-test, RViz with the
                commanded arm ghosted over the actual one, and Charts /
                Session / Event-log tabs, and the holographic master-arm and
                robot schematics. Frame time with a full sim stack running,
                measured after the panel settled: median **2.80 ms**, p95
                **9.57**, max 27.57, against a 100 ms budget (n=300). The
                schematics cost ~0.5 ms of median over the pre-Part-4 figure
                (2.26 / 9.14 / 16.66).
    teleop_gui  curses, no display needed. The SSH fallback and nothing more.
    console     Dear PyGui. SUPERSEDED and fully absorbed -- its Charts, its
                session/trial view and its event log are now tabs in `gui`.
                Nothing is left in it that `gui` does not have. Do not start
                new work in it.
    launcher    tkinter. SUPERSEDED by gui.

**25 buttons, not 41.** The GUI used to offer three generations of task set at
once -- t1-t9 (300/310 mm span), e1-e6 (the superseded E-series) and a/b/c --
so a button labelled "T3 rigid carry" ran a 300 mm tray against a 500 mm
current spec. The old sets are archived under
`src/srl_experiments/experiments/_archive/` with a README naming what
superseded them, `run_experiment.sh` refuses `t*`/`e*` BY NAME rather than
silently, and the task buttons are now exactly 3 tasks x 5 modes = 15.


Everything else -- every mode, experiment, calibration and diagnostic -- is a
button inside them. `gui` refuses to start a second stack and says why:
two `master_pose_node` instances split the serial stream and invalidated a
full day of measurements.

## CURRENT COUNTS -- the one place with live numbers

**The historical log below quotes counts that were true on their date.** Do
not read "36 unit tests pass" from a 2026-08-05 section as a statement about
today; that is what the date is for. This block is the only one that claims to
be current, and it is re-measured whenever it is touched.

| | measured 2026-08-10 | command |
| --- | --- | --- |
| unit tests | **376 pass, 2 fail, 1 skipped** | `python3 -m pytest -q src/*/test` |
| the 2 failures | `test_flake8`, `test_pep257` -- **pre-existing**, the package uses double quotes against the ROS style default | |
| executables | teleop 42, experiments 24, vr_teleop 9, autonomy 7, perception 6, vr_autonomy 2 | `ros2 pkg executables <pkg>` |
| GUI launch specs | **25** (15 task = 3 x 5 modes, plus modes and diagnostics) | `verify_gui_buttons.py` -- 46 checks |
| RViz render ceiling | **16.0 fps** llvmpipe, 16.3 d3d12, at 800x500 | `measure_render_rate.py` |
| clip delivered rate | **5.41 fps** over 11.3 s (was 1.29 over 28.4 s) | `measure_capture_rate.py` |
| verification clips | **15** = 3 tasks x 5 modes, 8 angles each | `record_abc_sweep.py` |
| clip object placement | **0 mm from target** in all 5 modes (task A) | `scene_events.json` per clip |

## FIRST ACTION OF EVERY LAB SESSION

```
bash scripts/check_channels.sh          # ~3 min, 14 sweeps, block A only
```

Which master channels are used and which are FROZEN is decided from the
**newest** `recordings/baselines/channels_*.json` -- a file on disk, not live
hardware. A baseline older than the wiring makes the software freeze channels
that now work, and **nothing downstream disagrees**: the arm behaves exactly as
it did before the repair. Target is **12 or more of 14 coherent**, where
degraded mode stops engaging. Save the new reference at the end (the script
prints the command) and the next launch picks it up automatically, because the
runtime and the script now select the same file by mtime.

When degraded mode engages, the startup banner carries a boxed warning naming
the baseline file and its age, and it is re-issued every 30 s.


Two Kinova Gen3 (7-DOF) arms on a backpack frame, teleoperated from a master
mannequin arm instrumented with potentiometers and IMUs, read by a Teensy 4.1
over USB serial, with a shared-autonomy layer. ROS 2 Jazzy.

**This file is the engineering log: every measured number, every trap, and why
each decision was made.** For orientation start with `README.md`; for how the
pieces fit together, `docs/system/01_architecture.md`.

**IF YOU ARE STARTING A FRESH SESSION:** read the final section of this file,
"HARDENING PASS — CONSOLIDATED SUMMARY", then `docs/NEXT_SESSION.md`, which
lists the lab work in priority order. Sections marked SUPERSEDED are kept for
their diagnoses, not their conclusions. **Nothing in the hardening pass has
ever run against a real arm.**

## Layout (restructured 2026-08-05)

| Package | Role | Depends on |
| --- | --- | --- |
| `srl_teleop` | **teleoperation ONLY** — master sensing, IK follower, clutch, scaling, sim→real bridge, e-stop | nothing in-repo |
| `srl_perception` | AprilTag detection, 6-DOF object pose | nothing in-repo |
| `srl_autonomy` | grasp generation, intent inference, handover arbitration | teleop, perception |
| `srl_experiments` | E1–E5, logging, conditions, analysis | all of the above |
| `srl_description` | `srl_dual.urdf.xacro` — arms, mount, and the wearer | — |
| `srl_moveit_config` | MoveIt config; TRAC-IK kinematics, controllers, SRDF | description |
| `ros2_kortex`, `ros2_kortex_vision`, `ros2_robotiq_gripper` | vendor deps | — |

**THE DEPENDENCY ARROW RUNS ONE WAY AND THIS IS LOAD-BEARING.** `srl_teleop`
is the baseline condition of every experiment; if it imported anything from
`srl_autonomy` the comparison would be circular. Verified two ways, not
assumed: no `import srl_autonomy|srl_perception` anywhere in `srl_teleop`, and
`ik_follower_node` still reaches "Tracking enabled" with those packages
physically removed from `install/`.

Build: `colcon build --symlink-install` — 16 packages, no flags needed.
`robotiq_driver` and `robotiq_hardware_tests` are `COLCON_IGNORE`d because
they need a `serial` CMake package that is not available here; see
`src/ros2_robotiq_gripper/README_BUILD.md`. Nothing in `srl_*` depends on them.

All `srl_*` packages are symlink-installed, so edits to `*.py` take effect
**on node restart**, without a rebuild. A rebuild is only needed when
`setup.py` entry points or `data_files` change. If a code change appears not
to apply, suspect a stale *process* — the node loaded the old module at start
— not a stale install tree.

## Run it

```
bash scripts/run_teleop.sh                    # teleoperation only
bash scripts/run_autonomy.sh                  # + perception + shared autonomy
bash scripts/run_experiment.sh e1 --participant PILOT --scripted
bash scripts/calibrate.sh zero|gyro|imu-mount|max-position
bash scripts/diagnostics.sh                   # before blaming the code
```

Three-terminal split for real work:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh          # --mock to rehearse
```

---

---

# 2026-08-15 — THE WORKSPACE HAD NEVER BEEN MEASURED AGAINST THE WEARER

The brief was four changes to T1 and one question about the workspace. The
question turned out to be the whole session, because the answer to "what is
limiting it" was, in part, "nobody has ever asked the arm how close it gets to
the person."

## The gap

Every workspace and layout figure in this repository comes from
`/compute_ik` with `avoid_collisions`. `srl_dual.srdf` permanently excludes
`torso`, `harness` and `backpack` against each arm's `base_link`,
`shoulder_link` and `half_arm_1_link` — the pairs a shoulder-mounted arm
actually threatens — so MoveIt returns `valid` for poses with the tube inside
the person. `find_presentation_pose.py` already knew this and checks clearance
GEOMETRICALLY for that one pose; nothing else did, and no task coordinate,
survey cell or workspace marking had ever been checked at all.

Measured with the mount guard's own capsule model
(`scripts/measure_clearance_region.py`, `scripts/verify_t1_paths.py`):

| | |
| --- | --- |
| shipped right-arm T1 layout | **70 of 143 waypoints inside the 150 mm floor**, worst 58.7 mm, at **0 IK failures** |
| the idle arm's park pose | **30.6 mm**, held for the whole clip |
| stage 2, seed 0, left arm | 21 of 71 waypoints inside the floor |
| left arm's IK-reachable cells | 72 of 265 inside the floor, worst **−2.7 mm** |
| right arm's IK-reachable cells | 68 of 314 inside the floor, worst −10.5 mm |

Negative means the metal is inside the person. Those cells were inside the
workspace marking a participant is told to work in.

## What binds, per direction, with the mechanism named

`scripts/measure_what_binds.py` — a four-rung ladder (collision-aware with
furniture → without furniture → collisions off → orientation free), the first
rung that succeeds naming the constraint, with the wearer split from
self-collision by measuring the collision-free solution against the capsule
model. Four controls, including a box placed on a cell just called REACHABLE,
which must then read FURNITURE.

    forward (+y)   ORIENTATION -- the pinned wrist, at y = 0.425
    outboard       ORIENTATION -- the pinned wrist, at |x| = 0.950
    inboard        THE WEARER  -- 0 mm inside the left forearm, 8 mm the right
    down (-z)      FURNITURE   -- the table top
    up, back       not bounded within 0.700 m

**No direction is bound by a joint limit or by arm length.** And the IK
boundary is not the safe boundary: forward, the arm is at 132 mm and 116 mm
against a 150 mm floor while IK still solves for another 25 mm.

## The height claim, settled both ways

"The band is y 0.06–0.20 and the same at every table height 0.90–1.30" is half
right, and the half that holds is the useful one. Objects ON the table, table
moved to each height, probed 20 mm above its top: **0.00–0.05 at every height
without exception.** Raising the table buys nothing, because what limits it is
the table — the approach axis is 30.7 deg ABOVE horizontal, so the forearm
trails below the fingertip and over a slab that volume is inside the slab.

With the table fixed and only the work plane moved, the band is strongly
height-dependent: 0.025 m at z = 0.95, 0.175 at 1.05, **0.425 at 1.10**, 0.500
at 1.30. It is a function of how far the work is ABOVE the table, saturating
near 0.50 m at about 200 mm of clearance. The current fixtured layout at
z = 1.12, 170 mm clear, already sits on that plateau.

## The free 300 mm

The old region boundary at |x| = 0.70 was **the survey box**, not the arm.
`survey_work_surface.py` has always defaulted to `--x -0.70 0.70`. Beyond it
there are 108 more left cells and 125 more right cells out to |x| = 1.000, and
**every one of them is 100% clear of the wearer**. The marking a participant
is shown was understating the usable space by 300 mm per side while
overstating it by 175 mm at the inboard end, where it is not safe.

## Four instrument defects found on the way

1. **`survey_work_surface.py` passed no `arm` to `ee_for()`.** It defaults to
   the LEFT arm's wrist-to-pad offset, so every right-arm cell in every survey
   was tested at a wrist pose 48.3 mm from where the right hand closes. The
   2026-08-12 decision to move T1 to the right arm was taken on that survey.

2. **`positions[:7]` on an IK response is always the LEFT arm.**
   `res.solution.joint_state` is the whole robot — 26 joints, left arm first —
   so slicing it gives the left arm whatever arm was asked about, and for a
   right-arm query those seven are the left arm sitting at its seed. The right
   arm's clearance therefore read ONE value, 0.1610 m, across 80 cells while
   the left varied from −0.003 to 0.161. Caught by the "zero variance" row of
   CLAUDE.md's table before the number was used for anything.
   `Solver.solve_arm_joints()` selects by name now, and
   `measure_clearance_region` carries a per-arm differential control that
   fails if an arm's clearance does not fall as the target comes inboard.

3. **`verify_msc_tasks.py` carried its own copy of T1's layout**, written down
   on 2026-08-11 and never touched. It verified those coordinates through
   three layout changes and reported zero failures each time. It imports now.

4. **`run_abc` wrote `--seed` into the manifest and never read it.** The
   layout came from `spec["build"]()` with no arguments, so stage 2 — whose
   entire definition is random positions — ran the same four cubes in every
   trial with a different seed number recorded beside them. A recorded seed
   that nothing reads is worse than no seed: it is a claim of randomisation
   in the data file.

## The centre, asked directly

The brief asked for T1's coloured planes in the centre of the table.
`scripts/measure_centre_reach.py`, scanning inboard at every forward distance
because the wearer's own arms hang at y = 0 and reaching inboard 300 mm IN
FRONT of them is a different question from reaching inboard beside them:

    left  innermost workable column, any y            x = 0.250
    left  ... that also keeps the clearance floor     x = 0.425
    left  x = 0.00 and +/-0.10                        NO y from 0.10 to 0.55
    right x = -0.10                                   y 0.425..0.475 only,
                                                      at the forward limit

The planes sit at x = 0.450 and 0.610 — the innermost clearance-safe column
plus the project's 20 mm margin — and they are **450 mm off centre**. That
number is the size of the finding: the centre of a person's own workspace is
the one place a shoulder-mounted supernumerary arm cannot go.

Its first version had a real bug worth recording: the inward scan STOPPED at
the first column that failed, which assumed the reachable set is an interval
in x. It is not — the left arm's row at y = 0.30 fails at x = 0.55 and
succeeds at 0.25..0.50 — so the scan reported "nothing at this y" for a row
with eleven working columns in it. Same family as the 62%-of-its-bounding-box
result that made the marking be drawn as cells.

---

# 2026-08-15 (later) — THE GAP HAD A THIRD CONSUMER, AND THE TABLE POINTED AT THE WRONG COLUMN

Two contradictions found while writing `docs/system/clearance_gap_ledger.md`,
which is the ledger of which earlier numbers the SRDF clearance gap
invalidated. `docs/TASK_SPEC.md` says a contradiction gets fixed and recorded
here, so both are here.

## 1. `named_places.py` still resolved spoken commands against the IK set

`clip_scene.py` and `msc_clip_tasks.py` were switched from `cells` to
`clear_cells` when the clearance region was measured. `srl_autonomy/
named_places.py` was not, and it is the module that turns **"move to the left
side"** into a pose for `autonomy_executive` — the whole of mode 06's input
path. Three consumers read that survey and two were changed.

It was safe, and it was safe **by luck**. The centroid of the IK set,
(0.625, 0.175), happens to clear the floor; the centroid of the clear set is
(0.725, 0.175). Nothing in the code preferred the safe one — a re-survey that
moved the centroid inboard would have returned a pose inside the wearer and
no check anywhere would have disagreed, because the only check the pose ever
faced was the `avoid_collisions` one that cannot see it. The `n_cells` it
reported alongside overstated the usable region by exactly the 72 left and 68
right cells that breach the floor.

It now reads `clear_cells` and **refuses** a file without them rather than
falling back, which the 2026-08-13 survey still on disk demonstrates:

    work_surface_region_PRE20260815.json has no `clear_cells`: it predates
    the wearer-clearance measurement ... I will not resolve a spoken place
    against an unchecked region.

Resolved poses move outboard 100 mm (left) and 75 mm (right); reported counts
drop 265 -> 193 and 314 -> 246. The **"front centre is unreachable"** answer
is unchanged, because it is 0 cells at |x| <= 0.10 in both sets — which is
worth saying, because it means that finding never depended on the gap.

## 2. TASK_SPEC's region table quoted the safe boundary against the unsafe count

The table in section 2A read

    left   265 cells, x +0.425...+1.000   |  193 cells

Measured from the file, the IK set runs from x = **0.250** and the clear set
from 0.425; on the right, −0.225 and −0.400. So the x-range printed in the
IK column was the CLEAR set's range. The row therefore quoted the safe
boundary while counting the unsafe cells, and the 175 mm inboard strip that is
the entire subject of the measurement did not appear in the table at all.

This is the "everything matches" row of CLAUDE.md's instrument table wearing
a new hat: two numbers from the same measurement, printed side by side,
looking consistent because one of them was copied from the other's source.

## 3. And the sweep had only one witness for a moving arm

Not a clearance finding, but found in the same pass. `record_abc_sweep`
decided a clip was good from `run_abc`'s exit code, which carries its own
`--min-travel-m` gate. Both measure EE travel, in different processes, off
different tf2 listeners — so the runner can see motion the scene node never
received, and on 2026-08-15 that is what got filed: clips reporting
`TRAVEL L 0.00 R 0.00` with four cubes carried 0.000 m, recorded OK.

`scene_travel_verdict()` is a second gate reading the scene node's own
`ee_travel_m`, extracted as a function so it can be handed a broken input —
the old gate lived inside a 300-line loop that needs a stack, an Xvfb and four
minutes to reach, which is the same reason a check nobody can break is a check
nobody has tested. 13 known answers, including the two a naive version gets
wrong: **one arm still is correct** for the three one-armed tasks, and a clip
with no travel field is **UNANSWERED, not failed** — by_design.py's rule.

Floor 0.05 m, from measurement rather than taste: the good clip recorded
minutes earlier reads left 3.0205 m, right 0.6010 m, and the failure reads
0.0000 on both. There is nothing between them.

---

# 2026-08-15 (recording) — WHAT LOOKING FOUND THAT THE CHECKS DID NOT

The clips were re-recorded on the current geometry and then examined frame by
frame against each task's MUST BE TRUE list. Everything below passed every
automatic check in this repository.

## 1. T2's TRAY IS ELASTIC, AND THAT MAKES T2-1 AND T2-2 UNFALSIFIABLE

`clip_scene.py` draws T2's tray as a CUBE spanning between the two grippers:

    span = math.dist(gl, gr)
    add(Marker.CUBE, mid, (round(span + 0.06, 4), dep, th), TAN, ns="tray")

So the tray's length is *the hand separation plus 60 mm*, recomputed every
tick. Measured over the recorded clips:

| | gripper separation | tray as DRAWN | spec |
| --- | --- | --- | --- |
| 01_master_teleop | 0.427 … 1.151 m | **0.487 … 1.211 m** | one RIGID tray, 0.560 m, grips 0.500 m apart |
| 06_full_autonomy | 0.456 … 1.151 m | **0.516 … 1.211 m** | " |

**A 2.5x stretch.** The consequences are not cosmetic:

* **T2-1, "tray held by BOTH grippers", cannot fail.** The tray IS the segment
  between the grippers, so every frame shows it held whatever the arms do.
  It is not evidence of coupling; it is a definition.
* **T2-2, "ball visible ON the tray", cannot fail either.** The ball is
  redrawn at the tray's midpoint every tick and "carried WITH it".
* **The declared failure mode never happens.** TASK_SPEC: "tilt past 6.8
  degrees drops the ball". Scoped to the CARRY -- the samples where the hands
  are within 100 mm of the 500 mm tray, which is the only window in which the
  word "carry" means anything:

  | | 01_master_teleop | 06_full_autonomy |
  | --- | --- | --- |
  | carry window | 44.6 - 56.7 s | 60.0 - 68.1 s |
  | tilt max | 6.85 deg | **23.04 deg** |
  | tilt rms | 2.28 deg | **20.18 deg** |
  | share of the carry above the 6.8 deg drop tilt | 1% | **89%** |

  Under full autonomy the tray is past the ball-drop angle for **89% of the
  carry** and the ball never moves. Scoping it to the carry made the result
  stronger, not weaker: the unscoped number (10.77 s) mixed in the approach.
* **The coupling premise is not represented.** T2 is "bimanual by COUPLING:
  one body, two grips, neither arm's pose free given the other's." A body
  that resizes to fit the hands constrains neither arm.

**THE TASK IS NOT AT FAULT, AND THIS IS THE PART THAT HAD TO BE CHECKED.**
`msc_clip_tasks` commands a hand separation of **exactly 0.500 m at every one
of the 21 waypoint pairs** -- the coupling IS commanded. What varies is what
the arms ACHIEVE, and the renderer draws the tray at the achieved separation.
So the stretch is a TRACKING ERROR being rendered as elasticity instead of as
error. Inside the carry window the achieved separation error still reaches
**81.6 mm (01)** and **67.9 mm (06)** on a rigid 500 mm body.

The previous bug here was the opposite error -- a rigid 560 mm tray pinned to
ONE arm's pads, so the right arm held air -- and the fix replaced a
wrong-but-rigid tray with a right-looking elastic one.

**A correction to an earlier reading in this same pass, recorded because the
instrument rule applies to me as well as to the scripts.** The front view of
06/T2 shows the tray crossing the wearer's head, and it does not: at the
highest waypoint pair the tray segment passes at y = 0.35, clearing the head
sphere by **248 mm**. The overlap is projection. It was checked because a
marker is not a collision object and `avoid_collisions` would not have
objected if it HAD been true -- which is the same gap as the SRDF one, and
worth re-checking whenever a marker and a person share a frame.

**T2 also records no GRASPED or RELEASED events at all** (`items` is `{}`;
tray and ball are fixtures), which is deliberate and documented -- but it
means the carry series is not anchored to a carry. `tilt_max_deg` and
`sep_err_max_mm` are computed over the WHOLE clip, approach and retreat
included, so they are not carry statistics and must not be quoted as ones.

## 2. THE WORKSPACE MARKING RUNS 25 mm PAST THE TABLE'S NEAR EDGE

`work_surface_region.json`'s clear cells start at **y = 0.075**.
`clip_scene.TABLE_NEAR_Y` is **0.100**. So the whole nearest row of the
marking -- **43 of 439 cells, 9.8%** -- is drawn over air, on the wearer's
side of the table edge.

This is the same defect the table widening fixed in x and left unfixed in y.
TASK_SPEC's own words for why the table went to +-1.05: *a marking drawn over
air is a marking that lies.*

**And stage 2 puts objects there.** In all three modes recorded, `cube_left_0`
came to rest at **y = 0.076**, 24 mm beyond the table's front edge, because
the stage-2 sampler draws from `clear_cells` and nothing in that pool knows
where the furniture is. It is visible from the front view.

Not fixed here, deliberately: moving the table's near edge is a scene change,
and changing scenery half way through a recorded set is worse than the defect.
The edge at y = 0.100 was measured to cost 0 waypoint failures at a 0.95 top;
0.075 has NOT been measured and must be before it is moved.

## 3. ACHIEVED MOTION IS NOT THE SAME ACROSS MODES, WITH NO OPERATOR PRESENT

TASK_SPEC section 1 says: *"the same waypoints go out under every mode, so
robot performance is identical across modes and every measured difference
comes from the operator."*

The first clause is true and is enforced in code. **The second does not
follow, and these clips disprove it** -- they are scripted, with no operator
anywhere in the loop, and the achieved motion still differs by mode:

| task | quantity | 01_master_teleop | 03_shared_autonomy | 06_full_autonomy |
| --- | --- | --- | --- | --- |
| T0 | left EE path | 1.407 m | 2.034 m | 2.032 m |
| T1 | left EE path | 3.106 m | 3.695 m | 3.682 m |
| T3 | circuit box carried | **0.021 m** | — | **0.161 m** |
| T3 | both objects held at once | **0.29 s** | — | **4.49 s** |

Run-to-run noise is bounded by the one cell recorded twice on the same day:
T1 under 01 read 3.021 m and 3.106 m, a 3% spread. T0's 44% and T3's 7.7x are
far outside it.

The mechanism is not mysterious -- the follower, the assist node and the VR
mapper are different command paths with different tracking lag, and the scene
node measures what the arm ACHIEVED, not what was commanded. That is a
reasonable thing for the platform to do. What is not reasonable is the
inference: **a mode difference in a trial cannot be attributed to the operator
until the no-operator difference has been subtracted**, and it has never been
measured. These fifteen clips are the first measurement of it.

The claim should read: the same waypoints are COMMANDED under every mode. It
should not say the robot performs identically, because it does not.

## 4. T3's MEASUREMENT HOLD IS 0.29 s UNDER 01

T3's card says both arms hold still while a reading is taken. Under
01_master_teleop the window in which BOTH objects are held at once is
**0.29 s**, and the circuit box travels **21 mm** -- it is never held up. Under
06 the same waypoints give 4.49 s and 161 mm. The clip is not wrong about
anything it claims; it simply does not show the thing the card describes.

## 5. THE VR MAPPER HOLDS THE ARMS, AND IT COST SEVEN CLIPS THEIR OPENING POSE

Seven of the twenty-five clips opened on the HOME pose instead of the
presentation pose: **all five of 04_vr_shared and two of 02_vr_teleop** -- the
only two modes that run `vr_pose_mapper`. Modes 01, 03 and 06 staged first
time on the same stack.

**Isolated by experiment, one variable, with a control on either side.** Same
stack, same arms, same target pose, nothing else touched:

| | `vr_pose_mapper` | result |
| --- | --- | --- |
| 1 | absent | `--home` **ARRIVED**, worst joint error 0.0000 / 0.0000 rad |
| 2 | **started, scale 1.0, exactly as `isolate()` starts it** | **DID NOT ARRIVE**, 2.076 / 1.924 rad |
| 3 | killed again, nothing else changed | **ARRIVED**, 0.0000 / 0.0000 rad |

**The arm does not move at all while the mapper is up.** In the sweep the
reported joint error was identical at the start and the end of the whole
deadline, and identical again on the next clip -- 2.6506 rad then 2.6462 rad
for 04's right arm. That is a held arm, not a slow one.

The follower's own pause is not enough. `stage_presentation_pose` sets
`motion_enabled` false on both followers and restores it -- the log shows both
-- and the arm still does not move. So the publisher that wins is not one the
follower's pause silences. This is the project's one-source-at-a-time rule at
the CONTROLLER level, which `docs/NEXT_SESSION.md` already records in a
different guise.

**Fixed in the sweep, not in the follower**: the mapper is stopped for the
staging move and `isolate()` is called again to restart it and RE-COUNT the
publishers on the follower input, so the run still begins from a verified
graph. If that re-isolation fails the clip is skipped rather than filmed
through an unverified graph. The underlying contention is untouched and is
still the right thing to fix at the source.

### A wrong hypothesis, recorded because it was acted on

The first explanation was that the staging trajectory was a single point with
a FIXED 2.5 s duration whatever the distance, so a 2.23 rad move demanded
0.89 rad/s and could not arrive. That defect is real and is fixed --
`move_seconds()` scales the duration from the distance, with seven known
answers -- **and it is not what was happening.** With the scaling in, mode 04's
right arm was given 8.8 s and a 12.8 s deadline and finished exactly where it
started. A fix that removes a real defect can still leave the symptom, and
reporting the symptom as fixed because a plausible cause was addressed is how
this project's standing rule gets broken from the inside.

## 6. 02_vr_teleop CANNOT GRASP, AND IT IS EVERY GRASPING TASK

Three of twenty-five cells failed. **All three are 02_vr_teleop, and they are
exactly its three tasks that record a grasp**: T1, T1 stage 2 and T3. Its
other two -- T0, which has nothing to grasp, and T2, whose tray is a fixture
with no grasp event by design -- pass.

T1, closest the pads ever came to each cube, against a **30 mm** capture gate:

| cube | x | 01 | 03 | 06 | **02** |
| --- | --- | --- | --- | --- | --- |
| 0 | 0.56 | 0.0 mm | 0.0 | 0.0 | **88.1 mm** |
| 1 | 0.62 | 0.0 mm | 0.0 | 0.0 | **115.4** |
| 2 | 0.68 | 0.0 mm | 0.0 | 0.0 | **156.2** |
| 3 | 0.74 | 0.0 mm | 0.0 | 0.0 | **205.9** |

The other three modes close on all four cubes at **0.0000 m** and place each
on the mat of its colour. 02 never touches one, and all four cubes are still
in their starting row in the final frame.

**The miss ACCUMULATES: increments of 27.3, 40.8, 49.7 mm.** That is the
diagnostic. A constant offset repeats one number -- which is how the 2026-08-13
"three cubes reporting an identical 0.0314" was traced to a perpendicular
offset, and how the earlier mapper `scale:=0.5` bug was traced to a constant
factor by the SAME 0.189 m appearing on two runs. A growing miss is neither:
the arm falls further behind on every cube.

### AND IT IS THE MAPPER'S LIFETIME. FIXED, AND THE FIX IS THE EVIDENCE

Re-recorded with one change -- `vr_pose_mapper` is now stopped for the staging
move and restarted by `isolate()` before each run, so every clip gets a FRESH
mapper instead of one shared across the mode's five clips. **All five of
02_vr_teleop then passed, including all three that had failed**, and T1's four
cubes closed at **0.0000 m** and landed on the mat of their own colour:

| | first run, one mapper for the mode | re-run, a fresh mapper per clip |
| --- | --- | --- |
| T1 | NO GRASP; 88.1 / 115.4 / 156.2 / 205.9 mm | **0.0 / 0.0 / 0.0 / 0.0 mm**, placed 32 mm from target |
| T1 stage 2 | NO GRASP | OK |
| T3 | NO GRASP | OK |
| left EE travel on T1 | 2.760 m | 3.703 m |

**So the mapper does not reset between runs, and what it carries over is
enough to lose every grasp.** The ordering in the failed run says the same
thing: its FIRST clip (T0, on a mapper seconds old) passed, and every clip
after it failed. The miss growing WITHIN a run -- 27.3, then 40.8, then
49.7 mm per cube -- is the same accumulation on a shorter timescale.

It also explains the shape of the 2026-08-15 (earlier) failure that this
session inherited, where 02 lost only the LAST cube by 44 mm: less had
accumulated by then.

**What is fixed and what is not.** The sweep now restarts the mapper per clip,
so the CLIPS are sound. The mapper itself still accumulates, and the DATA path
(`run_abc`, which a study session drives) shares `mode_upstreams.isolate()`
but starts the mapper once per session -- so a real trial block under 02 would
walk into exactly this. That is the next thing to fix and it belongs in
`vr_pose_mapper`, not in the harness.

---

# 2026-08-15 (later still) — THE OPENING POSE, THE APPROACH, AND A WHITE THAT WAS NEVER WHITE

Three changes were asked for. Two of them contradicted measurements already in
this repository, so both were re-measured before anything moved. One of the
two survived the re-measurement and was therefore refused; the other turned
out to be measuring the wrong quantity.

## 1. THE OPENING FRAME IS THE PRESENTATION POSE, AND ITS WRISTS WERE ALREADY LEVEL

"Wrists up and elbows splayed" names TWO poses, and they cost different
things:

| what is seen | which pose | where |
| --- | --- | --- |
| elbows splayed, hands low and wide | **PRESENTATION** | the opening frame of all 25 clips; `opened_on: "presentation"` in every `scene_events.json` |
| wrists up +85 / +79 deg | **HOME** | `docs/img/rviz/home_pose_labelled.png`, RViz/GUI before staging, and any clip where the staging move did not execute |

`presentation_pose.json` recorded `elevation_deg: 0.0` for BOTH arms before
this session touched anything. The presentation pose's wrists have been level
since it was built. So the half of the complaint that is about wrists is about
HOME, and the half that is about splay is about the presentation pose — which
is free to change, because it is a camera decision that no coordinate depends
on.

### What changed, and a claim from 2026-08-13 that does NOT survive measurement

The 2026-08-13 commit left three faults recorded and unfixed: the hands were
LOW, the posture was WIDE, and — it said — "the upper arms ride over the
shoulder line ... the limb still rises above the shoulder before descending".
The elbow term could not see the third, because it measured one joint
(`shoulder_link.z - forearm_link.z`) rather than the limb, so a term was added
for the APEX of the whole mount-guard capsule chain against the wearer's
shoulder line at z = 1.46 (the top face of the 0.36 x 0.22 x 0.48 torso box).
It was free: `clearance()` already FKs every link on that chain.

**The new term found nothing, and that is the result.** Scored on the same
instrument (`scripts/score_pose.py`, all three controls correct):

| | hand z | wrist | elbow below shoulder | limb apex | clearance |
| --- | --- | --- | --- | --- | --- |
| OLD left | 1.100 | -0.0 deg | 188 mm | 1.342, **118 mm below** 1.46 | 0.1610 |
| NEW left | **1.180** | -0.0 deg | 79 mm | 1.342, 118 mm below | 0.1610 |
| OLD right | 1.100 | -0.0 deg | 292 mm | 1.341, **119 mm below** | 0.1610 |
| NEW right | **1.180** | +0.0 deg | 272 mm | 1.341, 119 mm below | 0.1610 |

The apex is IDENTICAL before and after and it was already 118 mm under the
shoulder line. So the limb was never riding over the shoulders: that bullet
was an impression taken off a render, the geometry does not support it, and it
is withdrawn. Inflating the chain by the 50 mm tube radius still tops out
around 1.39, under the 1.46 line. What reads as "over the shoulder" in the
picture is the arm being nearer the camera than the wearer is.

**The one thing that did change is the one thing that was really wrong: the
hands are 80 mm higher**, 1.100 (hip) to 1.180 (chest). Wrists stay level at
0.0 deg and wearer clearance stays 0.1610 m, which is the ceiling — nothing on
the arm can be further from the wearer than the arm's own base, so 0.1610 is
"as clear as this platform gets" rather than a middling score.

**The cost, stated:** total joint travel from home rises 279.1 -> 311.9 deg
(left) and 263.6 -> 283.0 (right), against a 320 deg cap, so the left arm now
has 8 deg of margin. The travel that the file says actually matters — the
HAND's path through space, because that is the motion a person sees sweeping
over them — went DOWN, 0.190 -> 0.188 m and 0.235 -> 0.222 m.

### WHAT DID NOT CHANGE, AND WHY IT CANNOT

The hand span is still 1.100 m against a 0.360 m torso, 3.1x. That is HARD
CONSTRAINT 11 and not a search failure: measured geometrically, a hand at
|x| = 0.18 puts the arm 16 mm INSIDE the torso and |x| = 0.45 is the nearest
either hand comes with the 150 mm floor intact. Requiring one mirrored target
both arms can hold pushes that to 0.55 — narrower and lopsided, or wider and
symmetric, and the floor does not allow both.

## 2. THE HOME ANGLES WERE NOT TOUCHED, AND HERE IS THE BILL

Refused pending an explicit decision, and the cost is larger than the file
already records:

* the stored home IS the legacy real-robot home, so changing it starts with a
  physical recapture on arms this repository has never run;
* `P_HOME`, `WORKSPACE_CENTRE` (offset = P_HOME) and `WORKSPACE_ORIENT` —
  and `WORKSPACE_ORIENT` is the quaternion `run_abc.send()` writes into
  **every waypoint of every task under every mode**, so it is not only the
  anchor, it is the study's independent variable;
* `PAD_OFFSET_BY_ARM`, measured off TF at home, with three consumers
  including the gate that decides when the hand closes;
* every T0-T3 coordinate through `ee_for()`, every clearance figure, all 579
  region cells, and the presentation pose itself, which is stored as a DELTA
  from home;
* **and the jump is measurable.** The presentation pose is a level-wrist,
  hands-forward posture and it sits 311.9 deg (left) and 283.0 deg (right) of
  total joint travel from home. That is the order of what the sim-to-real
  bridge would command in one step if the sim home moved there and the real
  arm were not recaptured. `home_positions_left.txt` additionally records
  joint_5 at 14.10 deg from the +/-180 seam.

## 3. GRASP APPROACH: MEASURED PER OBJECT, AND TOP-DOWN IS WORSE FOR T1

`scripts/measure_grasp_approach.py`, N=10 over the full path, furniture in
scene, wearer measured geometrically, eight controls including the pad offset
reproducing `PAD_OFFSET_BY_ARM` to 0.1 mm.

**Every grasp in the set uses the pinned near-side approach**, because
`run_abc.send()` writes the anchor into every waypoint. There is no per-grasp
approach in the code at all.

### T1, and this REPLACES the 0/4-vs-0/4 row in `09_task2_grasping_finding.md`

That row was taken on the old RIGHT-arm layout with pedestals, which no longer
exists. Re-taken on the layout T1 actually runs:

| | pinned near-side | top-down |
| --- | --- | --- |
| 4 cubes | **4/4, clearance 0.1610** | 4/4, clearance 0.1610 |
| 4 place points | **4/4, clearance 0.1610** | 4/4 reachable, clearance **0.012 / 0.042 / 0.085 / 0.115** |

**Top-down keeps the picks and loses every one of the places on wearer
clearance** — down to 12 mm against a 150 mm floor. So top-down is not the
better approach for T1; it is strictly worse, and the approach is unchanged.

### Does the wrist present the fingers to the object? Measured from FK on the two pads

The closing axis is read from the two finger-tip links, and the object's
extent ALONG that axis is compared with the hand's 85 mm opening:

| task | object | pinned | top-down | verdict |
| --- | --- | --- | --- | --- |
| T1 | 40 mm cube | 49.0 mm | 40.0 mm | both fit; the pinned hand closes across a DIAGONAL of the cube, not a face |
| T2 | tray, left grip | 83.4 mm | no yaw solves it | fits by **1.6 mm** |
| T2 | tray, right grip | **113.9 mm** | no yaw solves it | **does not fit an 85 mm hand** |
| T3 | circuit box | **195.0 mm** | **170.0 mm** | **does not fit under either approach** |
| T3 | multimeter | 42.4 mm | 30.0 mm | both fit |

T3's box is the clean case: it is reachable and clear of the wearer under both
approaches and the hand is aimed squarely at its 170 mm width. What it needs
is a ROLL about the approach axis to close on the 50 mm height — not a
different approach, and not a different position.

**An instrument defect on the way, and it is this project's own pattern.** The
first version of the presentation test asked only whether the pads were
CENTRED on the object and duly reported 195 mm of circuit box lying across an
85 mm hand as PRESENTED. A verdict that cannot fail on an ungraspable object
is not a verdict; the aperture is now part of it and a 200 mm control object
proves it can fail.

### AND THE TASKS' OWN PATHS: THREE OF FOUR BREACH THE WEARER FLOOR

Only T1 has ever had the geometric wearer pass. Taken for the others, on the
waypoint list the recorder actually sends:

| task | arm | IK failures | worst clearance | |
| --- | --- | --- | --- | --- |
| T0 | left / right | **4 / 4** | 0.0086 / 0.0176 | breaches by 141 / 132 mm |
| T1 | left / right | 0 / 0 | 0.1610 / 0.1610 | clean |
| T2 | left / right | 0 / 0 | 0.0046 / 0.0904 | breaches by 145 / 60 mm |
| T3 | left / right | 0 / 0 | 0.0618 / 0.1610 | left breaches by 88 mm |

Reported, not fixed: each is a layout change with its own re-verification, and
T2's is entangled with the elastic-tray finding above.

## 4. THE PADS STAY 450 mm OFF CENTRE, AND HEIGHT DOES NOT RESCUE THEM

`scripts/measure_centre_vs_height.py`. The standing objection to
`centre_reach.json` was fair — it was taken at ONE work height, z = 1.120,
while `band_vs_work_height.json` shows forward reach climbing from 0.025 m to
0.500 m as the plane rises. So the two were crossed: ten heights from 1.12 to
1.55, both arms, stage 1 grasp-pose-only (optimistic) and stage 2 the full
pick path at N=10 with the wearer measured geometrically.

**At |x| <= 0.10 there is no height that works.** IK reaches the centreline
easily — the right arm solves at x = 0.025 — and not one of those cells clears
the 150 mm floor at any height. Asked again with a TOP-DOWN wrist, in case the
pinned wrist was the thing closing it: same answer, and every stage-1 survivor
at |x| = 0.300 turned out to be reach-only once the whole path was walked.

The innermost SAFE column, confirmed over the full path:

| work plane | left | right | height above the table |
| --- | --- | --- | --- |
| **1.120 (now)** | **0.425** | **0.400** | 170 mm |
| 1.250 | 0.400 | 0.350 | 300 mm |
| 1.350 | 0.400 | 0.300 | 400 mm |
| 1.550 | 0.350 | 0.250 (reach only) | 600 mm |

Height buys 75 mm of inboard reach for the left arm and 150 mm for the right,
and it costs 430 mm of standing the work up in the air. The pads stay at
x = 0.450 and 0.610. The 1.120 row reproduces `centre_reach.json` exactly,
which is the cross-check that matters: two instruments, built for different
questions, agreeing on the boundary.

## 5. THE TABLE WAS NEVER WHITE IN THE PICTURE, AND NOTHING COULD HAVE NOTICED

TASK_SPEC T1-8 says the table is WHITE. `clip_scene.OAK` asked for
(0.94, 0.94, 0.95). Sampled out of the shipped 2026-08-15 T1 clip, the top
renders at **RGB(118,118,118)** — mid grey — while the arm's own meshes in the
same frame reach (208,207,210).

**The audit passed it every time**, because `audit_task_spec` read `CS.OAK`
and asked whether the REQUESTED colour was neutral and >= 0.80. It never
looked at a pixel. That is CLAUDE.md's "matched requested RGB, not RENDERED
colour" row arrived at from the authoring side instead of the verifying one.

`scripts/probe_marker_shading.py` measures the renderer's actual response, one
slab where the table is, one frame per colour, three controls:

| requested | up-facing face | camera-facing face |
| --- | --- | --- |
| 0.00 | 0 | 0 |
| 0.94 | 119 | 208 |
| 1.00 | 127 | 221 |
| 1.50 | 190 | 255 |
| 2.00 | 253 | 255 |

Two things fall out. A marker's material takes **ambient = 0.5 x colour** and
the only light is a headlight on the camera, so an UP-FACING face — which is
what a table top is — gets the ambient term alone. And **RViz does not clamp
the colour before it reaches the material**: 1.5 and 2.0 continue the same
line to 190 and 253, so the ambient term can be driven to full and a white
table top IS available. It just cannot be written down as "white".

The table top is now (1.88, 1.88, 1.89), which renders ~240. If a renderer
ever does clamp at 1.0 this degrades to the old 127 rather than to anything
broken. T1-8 now applies the measured coefficient and asks for a white PIXEL,
and the audit's 13-probe self-test still detects every break.

**And the table lost two things that were never furniture.** Two 110 mm posts
stood on it in every T1, T1S2, T2 and T3 clip, carrying the bench — which only
tasks a/b/c have. They held nothing, stopped 60 mm short of the objects above
them, and invited exactly the reading this scene must not invite: that the
objects rest on something. They are drawn only when a bench is. The legs are
also inset now so the top overhangs, and the apron runs all four sides instead
of two.

## 6. AND ONE THING THAT LOOKED LIKE A DEFECT AND WAS A CAPTURE ORDERING FAULT

The first render of the white table came back with the table present and the
workspace marking, the cubes and the mats ALL MISSING. The obvious reading was
that an out-of-range colour had broken the marker array — plausible, and it
would have made the whole approach unusable.

It is not what happened. `clip_scene` was publishing 220 markers at 10 Hz
throughout: 9 `scene` (one table, four legs, four apron rails), 197
`workspace`, 4 `planes`, 8 `item`, measured by subscribing and counting. What
was wrong was the ORDER: the scene node was started before RViz existed, so
"the marking is missing" and "the renderer was not up yet" produced the same
picture. Displays first, then the scene, then the grab, and the same colours
render the whole scene correctly — table top at 238 of 255 against the old
118, near edge 255, legs 170.

Recorded because it is a fresh instance of an old pattern: the render is an
instrument too, and a missing thing in a picture is evidence about the
capture until the capture has been cleared.

**A CONSEQUENCE FOR THE RECORDED SET.** The 25 clips were recorded before any
of this. Every one of them shows a grey table, two risers holding nothing, and
an opening pose with the hands 80 mm low. They are stale against the scene and
need re-recording before they are shown to anyone.

---

# 2026-08-15 (last) — THE PRESENTATION POSE BECAME HOME, AND THE ANCHOR DID NOT

The request was to make the presentation pose the actual home rather than a
pose clips open on. The standing objection was this project's own: the
wrist-up orientation is the approach that reaches the table. **That objection
turns out to be about a different quantity, and the difference is the whole
result.**

## THE CHANGE IS TWO CHANGES, AND ONLY ONE OF THEM IS DANGEROUS

Home is the origin of two things that sound like one:

* **where the arm rests** — `config/home_positions_*.txt` and the URDF's
  `initial_positions`. This is the picture.
* **the pinned ANCHOR** — `master_calibration.WORKSPACE_ORIENT`, the
  quaternion `run_abc.send()` writes into every waypoint of every task under
  every mode. It is the approach direction, 30.7 deg (left) / 22.1 (right)
  above horizontal, and it was MEASURED at home.

**They are separable in the code, and nothing had noticed.** `WORKSPACE_ORIENT`
and `WORKSPACE_CENTRE` are read ONCE, in `master_pose_node.__init__`, to seed
ROS parameters. Nothing recomputes them from the live home. So a level home
does not force a level grasp: the arm rests level and rotates its wrist to the
pinned approach on the way to the work — exactly what
`home_wrist_is_real.md` had always claimed about the two numbers, now
demonstrated by moving one and watching the other stay.

## MEASURED, NOT ARGUED

`scripts/measure_home_change.py`, N=10 over every distinct waypoint of each
task's OWN builder, wearer and furniture in scene, wearer clearance from the
mount guard's capsule model. The arms are STAGED into each pose and arrival is
verified off `/joint_states`, not off the staging script's report. IK failures:

| task | home STORED / anchor STORED | home PRES / anchor STORED | home PRES / anchor LEVEL |
| --- | --- | --- | --- |
| T0 | 8 | 8 | 6 |
| T1 | 0 | **0** | 0 |
| T1 stage 2 | 0 | **0** | 1 |
| T2 | 0 | **0** | **4 — the right arm is lost** |
| T3 | 0 | **0** | 0 |

**With the anchor kept, every task performs exactly as it does today.** T0's
eight failures are not caused by this change — they are there in the shipped
platform too, and they are the first time anyone has walked T0's own waypoint
list against collision-aware IK.

**With the anchor re-derived to level, T2 loses its right arm.** The near-side
approach is what reaches the tray, and 0/2 under top-down was already on
record. So the approach is load-bearing and the resting pose is not.

The level-anchor paths are not a rotation applied to the old wrist poses: the
pad offset is only valid at the anchor orientation, so every waypoint is
shifted by `(R_stored - R_level) @ pad_offset_in_ee_frame` — exact, because
the orientation is constant along a path — which holds the finger PADS on the
trajectory the task declares and moves the wrist to wherever the new
orientation puts it.

**THE KNOWN-ANSWER CONTROL IS WHAT MAKES THE ABOVE WORTH READING.** The
stored/stored cell has to reproduce the committed 2026-08-15 numbers on eight
task/arm pairs, and it does.

## THE ONE THING THAT GOT WORSE

**T1 stage 2's RIGHT arm loses 59 mm of wearer clearance**, 0.1610 → 0.1016
against a 150 mm floor, measured worst-over-N. It still solves every waypoint.
The mechanism is the IK SEED: `ik_follower_node` seeds from home and the
solver seeds from the live joint state, so a different resting posture lands
the solver in a different null-space branch. T2's right arm moves 0.0799 →
0.0027, but T2 was already 70 mm inside the floor, so that is worse-of-a-bad-
thing rather than new. T1, T3 and T1 stage 2's left arm do not move.

## AN INSTRUMENT DEFECT, CAUGHT BY A CONTROL, FIXED, AND WORTH RECORDING TWICE

The first version took the clearance of whichever solve happened to run LAST
per waypoint — one sample of a null space TRAC-IK re-seeds randomly, and the
seed is precisely what this experiment changes. Corrected to the worst over
all N. The known-answer control then flagged T2's right arm as DISAGREEING at
0.0799 against a committed 0.0904 — which is the correction working, not a
fault, because worst-over-N can only be lower. The control was made one-sided
rather than the measurement being tuned to pass it.

And on one run the middle cell was REFUSED outright: the staging move from the
stored home to the presentation pose is 1.94 rad and did not arrive inside the
default deadline, so the script reported nothing rather than reporting numbers
for a pose the arms were not in. That refusal is the reason the numbers above
can be believed; the transition is intermittent and is now retried at a longer
deadline.

## WHAT MOVED, WHAT DID NOT, AND WHAT IS OWED

**Moved:** `config/home_positions_{left,right}.txt`; both `initial_positions`
blocks of `srl_dual.urdf.xacro`. A new test,
`test_home_lives_in_two_places.py`, fails if the two ever drift apart or if a
continuous joint is stored outside ±π — it was a comment in both files saying
"keep the two in step", and a comment cannot fail.

**The right arm's joint_7 is stored WRAPPED**, -94.25 deg rather than the
265.75 the search returned. Same physical pose — joint_7 is continuous — but
`ik_follower_node` refuses to start on real hardware with a continuous joint
outside ±π and unwinds 360 deg in sim. The Kortex value is 265.75 either way,
so the lab capture is unambiguous. The left arm's joint_5 seam margin
IMPROVED, 14.10 → 28.01 deg.

**Did not move, and must not:** `WORKSPACE_ORIENT`, per the table above. No
task coordinate, no clearance figure, none of the 579 region cells —
they derive from the anchor, and nothing computes them from home at runtime.
The coupling to home is by derivation HISTORY, not live computation.

**Owed:** `WORKSPACE_CENTRE` is documented as `offset = P_HOME` and is now
stale by 0.188 m (left) / 0.222 m (right). `master_pose_node` seeds
`pos_anchor`, `anchor_ref` and `last_pos` from it, so the first commanded
teleop frame would move the arm that far. The replacement values are in the
file, ready to paste, and deliberately not applied: changing it changes what
live teleoperation DOES, which is the study's baseline condition.

**And the real arms have not moved.** They are still at the legacy Kortex
home. `sim_to_real_bridge.enable()` compares the real arm against the loaded
home (`require_homed`, 0.05 rad) and REFUSES, naming the joint and the size;
the gap is ~1.9 rad, forty times the tolerance, and a second gate refuses on
the sim-to-real gap behind it. **It refuses; it does not silently move.**
Getting the arms there is ~312 deg (left) / ~283 (right) of total joint
travel — a large unattended motion over a person, and a decision to take in
the room rather than from a script.

---

# 2026-08-15 (applied) — THE HOME POSE HAS FIVE STORAGE SITES, NOT TWO

The "update both places" trap is quoted repeatedly in this project. **It
undercounts, and the two sites nobody names are the dangerous ones.**

| site | live? | who reads it |
| --- | --- | --- |
| `config/home_positions_{arm}.txt` | **THE SOURCE, and the most live thing here** | `sim_to_real_bridge` (the enable gate), `real_homing_node` (where the REAL arm is driven), `ik_follower_node` (the IK seed), `mock_real_stack`, `session_manager` |
| `srl_dual.urdf.xacro` x2 blocks | live | where the SIM arm spawns; baked at launch |
| `pot_bridge.py` | **live, and it had its own hardcoded copy** | publishes to the arm controllers |
| `srl_teleop_node.py` | **live, and it had its own hardcoded copy** | publishes to the arm controllers |
| `config/real_home_reference.txt` | **reference only — no code reads it** | nothing |

**Both hardcoded copies are installed executables that no launch file or
script references.** `ros2 run srl_teleop pot_bridge` and `ros2 run srl_teleop
srl_teleop_node` both work and both publish joint trajectories. Nothing
exercises them, so nothing would have caught the drift, and after the home
change each was one command away from driving an arm to the superseded pose.
They now load `home_positions` like everything else.

`real_home_reference.txt` is the one that genuinely *is* reference-only, and it
had gone stale in the way that matters: it carried a `left_sim_home_deg` line
and per-joint deltas computed against a sim home superseded TWICE since, so it
read like a homing estimate and was arithmetic against a pose that no longer
exists. The deltas are removed rather than updated — the live reading is real
data worth keeping, the deltas are recomputable in one line — and the header
now says plainly that no code reads the file.

## THE GUARD, AND TWO GOES AT MAKING IT ABLE TO FAIL

`test_home_has_one_source.py` covers all of it: URDF against config, no node
carrying its own copy, every arm-commanding node actually loading the source,
continuous joints inside ±π, and `--` never appearing inside an XML comment.

Getting the "no hardcoded copy" check to work took two corrections, both found
by trying to break it:

* **v1** anchored on `home... = [` and MISSED the dict form the real code used
  — `self.home = {"left": [ ...seven... ], "right": [...]}`. It passed a
  deliberately broken input, which makes it not a check.
* **v2** matched any seven-float list with "home" within 200 characters and
  FALSE-POSITIVED on `mock_real_stack.start_offset_deg`, which is an offset
  *from* home sitting under a comment that mentions it.
* **v3** parses the file and asks the only question that matters: is a literal
  seven-element pose assigned to something named `home`? That is a question
  about the syntax tree, not about characters, and it fails on both the dict
  form and the flat form while leaving the offset alone.

## AND THE XML COMMENT RULE, WHICH COST THE WHOLE DESCRIPTION

`--` cannot appear inside an XML comment. A comment added beside the home
change used it the way every Python file here does, and xacro refused
`srl_dual.urdf.xacro` outright: "not well-formed (invalid token)". That takes
out the arms, the wearer, MoveIt and everything reading `/robot_description`,
and the error names a line and column rather than the mistake. Caught by
generating the URDF and diffing all fourteen `initial_value` entries against
the config — which is also the check that confirms the change took.

## WHAT THE HOME CHANGE ACTUALLY COST, VERIFIED ON THE APPLIED TREE

Re-verified after applying, N=10 full path, wearer and furniture, clearance
geometric, arms confirmed at the loaded home to 0.0001 rad:

| task | IK failures | worst clearance | vs the legacy home |
| --- | --- | --- | --- |
| T0 | 8 (4 per arm) | 0.0078 / 0.0126 | unchanged; pre-existing |
| T1 stage 1 | **0** | 0.1571 (174 waypoints), **0 below the floor** | unchanged, clean |
| T1 stage 2 | 0 | seed 0 right **0.1135** | **REGRESSED: 25 of 98 waypoints inside the floor.** Seeds 1 and 2 clean |
| T2 | **1** (left) | 0.0000 / 0.0026 | **REGRESSED: left arm now fails a waypoint** |
| T3 | 0 | 0.0618 / 0.1610 | unchanged |

**So the change is not free, and the earlier "every task performs exactly as
before" needs the qualifier it now has.** That statement was measured on IK
failures alone and it holds for T1, T1s2 and T3; T2's left arm has since
turned up one marginal failure, and T1 stage 2's seed 0 right arm has lost its
clearance margin.

**THE MECHANISM IS THE IK SEED, AND THE JOINT_7 WRAP IS PART OF IT.** The
solver seeds from the live joint state, so the resting posture selects which
null-space branch it lands in. The right arm's joint_7 is additionally stored
WRAPPED (-94.25 deg) where the pose search returned 265.75 — the same physical
pose, 2π apart as a seed. Both effects push the right arm into a different
branch, and on T1 stage 2's seed-0 layout that branch runs 37 mm inside the
wearer floor.

**Neither is fixed here, because both are layout decisions rather than bugs.**
The options for stage 2 are to re-draw seed 0, or to filter the sampling pool
against the floor at the new home; the option for T2 is the same conversation
its elastic tray already needs. Both are in `docs/NEXT_SESSION.md`. What must
not happen is the numbers being quoted from the pre-change run: `verify_t1_paths`
exits non-zero on the floor breach and says which waypoints.

---

# 2026-08-15 — CAN THE WORK BE IN THE CENTRE, ON THE SURFACE? SEARCHED PROPERLY.

`scripts/search_centre_on_surface.py`. The earlier answer swept the WORK PLANE
above a table fixed at 0.95 and lifted the objects into the air, which is what
the brief does not want. This sweeps the TABLE and requires the objects to
REST ON it: table top 0.70–1.10, object distance out 0.10–0.55, overhang 0.00
or 0.05, |x| 0.00–0.45, both arms, pinned and top-down. 3360 cells screened at
the grasp pose, survivors re-walked over the full pick path at N=10 with the
wearer measured geometrically. Three controls, all correct — including an
object BURIED in the slab, which must fail or the table is not in the scene.

"The wearer stands further back" and "the table moves forward" are the same
degree of freedom, so the y sweep covers both. A NARROWER table is free and is
not swept: nothing about the approach depends on how far the table extends
sideways past the object.

## THE ANSWER: NO. AND THE CLOSEST IS 350 mm.

**No configuration puts |x| ≤ 0.10 on the surface.** 22 of 3360 cells survive
the screen, and the structure of those 22 is the finding:

| | |
| --- | --- |
| nearest the centreline, LEFT arm | **x = 0.350**, table top 1.00, near edge 0.530, confirmed over the full path at N=10, clearance 0.1524 |
| nearest the centreline, RIGHT arm | **x = 0.450**, and it is *reach only* — 0.1381 against a 0.150 floor |
| **every survivor has overhang 0.00** | the object must sit at the VERY FRONT EDGE. Not one cell with the object 50 mm back survives |
| table height | barely matters — 0.80, 0.90, 0.95, 1.00 and 1.10 all appear |
| what does matter | how far FORWARD the table is: every survivor is at y 0.400–0.550, near edge 0.380–0.530 |
| **top-down** | **0 of 840 cells**, at any table height, either arm |

**The overhang result is the mechanism, and it is the same one this repository
already has.** The pinned wrist's tool axis is 30.7° ABOVE horizontal, so the
hand arrives from the near side and from BELOW; over a slab that trailing
volume is inside the slab. The only place an object on a table can be grasped
is where there is no table under the approach — the front edge.

## TWO THINGS THIS CHANGES

**1. T1-1 may be unblockable after all, and not by the route that was tried.**
The audit records T1-1 (cubes rest on the table) as BLOCKED on the evidence
that raising a slab under the objects costs 46 of 54 waypoints. That held the
table where it is. Moving the table FORWARD to a near edge of ~0.53 and
putting the objects at its front edge gives a cell that is reachable and clear
over the full path at N=10. **This is one cell, not a layout** — four cubes and
two pads all have to fit along that front edge and all have to verify — so it
is a lead, not a result. It is the first evidence in this project that objects
on the surface are reachable at all.

**2. Top-down with objects on a surface is not available anywhere.** Not at
any table height from 0.70 to 1.10, not at any distance, not on either arm.
So "top-down and centre-on-surface" is not a trade between two achievable
things: centre-on-surface is achievable at 350–450 mm off centre, and
top-down-on-surface is not achievable at all. Combined with the earlier
measurement — top-down reaches T1's four place points but breaches the wearer
floor at all four — the pinned near-side approach is the only one that both
reaches and stays clear.

---

# 2026-08-15 — PREDICTIVE AVOIDANCE: THE NULL-SPACE HALF DOES NOT WORK, MEASURED

The brief asked for an arm that sees a near-collision coming and moves the
ELBOW out of the way through the null space while the gripper pose is
unchanged, instead of refusing and holding. **Built, tested, and the
null-space half is measurably useless on this arm.** Recording that is worth
more than shipping it.

## WHAT WAS BUILT

`srl_teleop/predictive_avoidance.py`, 16 unit tests on constructed inputs:

* **LOOKAHEAD** — straight-line extrapolation of the commanded pose over a
  0.30 s horizon, with a staleness guard so a paused master predicts NO
  motion rather than flying on its last velocity. "No estimate" and "an
  estimate of no motion" are different returns, deliberately.
* **NULL-SPACE SAMPLER** — the same EE pose requested from a symmetric fan of
  seeds around joints 3, 4 and 6, every solution scored by wearer clearance,
  the clearest chosen. Not a Jacobian projection: this follower has a
  `/compute_ik` service, not a local Jacobian, and the docstring says so
  rather than borrowing the name.
* **choose()** — clear of the trigger, use the unperturbed solution so the
  operator feels nothing; inside it, take the clearest; refuse only when
  nothing clears the floor.

## AND THEN IT WAS DRIVEN AT THE WEARER, WHICH IS THE POINT

`scripts/verify_predictive_avoidance.py` marches the commanded EE from the
home hand position straight into the head and then the torso, 15 steps, with
avoidance off and on, against the real solver and the real wearer model.

| | min clearance OFF | min clearance ON | best gain anywhere |
| --- | --- | --- | --- |
| head | −0.1550 | **−0.1550** | **+0.0044 m** |
| torso | −0.1574 | **−0.1574** | **+0.0001 m** |

**The fan is not the problem.** Its own control passed: 90 and 82 DISTINCT
joint solutions were found and scored. The null space is being searched; it
simply does not contain anything better. At 12 of 15 steps (head) and 13 of 14
(torso) the chosen solution's clearance equals the baseline's to four decimal
places.

## WHY, AND WHAT WOULD ACTUALLY WORK

Two mechanisms, and both matter:

1. **Where the EE itself is inside the wearer, no null space can help** — by
   construction, since the family holds the EE pose fixed. Refusing is the
   correct answer there and it is what happened. The harness drives the hand
   INTO the head, so the late steps could never have been rescued.
2. **Where the hand is still clear, re-seeding TRAC-IK does not select a
   different ELBOW branch.** The returned solutions differ in joint space —
   90 of them — while the limiting link stays put, so they differ in the
   wrist rather than in the swivel. Seed sampling is not a redundancy
   resolution.

**The real implementation is an explicit elbow-swivel parameterisation**:
compute the elbow circle about the shoulder-wrist axis analytically for the
Gen3, choose the swivel angle that maximises wearer clearance, and solve the
remaining joints from it. That is a different and larger piece of work, and it
is the honest next step rather than a tuning of this one.

## WHAT IS TRUE OF IT TODAY

* the EE residual is **0.00001 m** — the operator could not feel the
  avoidance, which is trivially true because nothing meaningful moved;
* it **never hung**: every step returned a decision. It refused 11 of 15 while
  the hand was being driven into a person's head, which is the right answer,
  not a lockup;
* `avoidance never reduces clearance` — its own control — held at every step;
* the clearance floor is untouched and is still the last resort.

**NOT WIRED INTO `ik_follower_node`.** That node gates motion near a person's
chest, and wiring in a strategy measured to add 4 mm would be worse than
leaving it out. `srl_console`'s "tangential" and "null-space" display labels
were already removed in an earlier pass for naming stages that did not exist;
nothing here re-adds them.

---

# 2026-08-15 — THE CENTRE, ASKED AGAIN WITH THE WEARER'S ARMS MOVED. IT IS THE TORSO.

The project's answer to "why can the work not be in the centre, in front of the
person" has been **the wearer's own forearm**, and that answer was measured
against a mannequin whose arms hang rigidly at its sides. That is a MODELLING
CHOICE and not a fact about people: a person standing in a rig with two robot
arms working in front of their chest will move their arms, and asking them to
is an ordinary operating instruction rather than a platform change.

So the posture was made a variable and the sweep re-run. **The centre does not
open in any posture, and with the wearer's arms deleted entirely the binding
constraint is the TORSO.**

## THE POSTURE IS NOW A VARIABLE, WITH ONE SOURCE

`src/srl_teleop/srl_teleop/wearer_posture.py` holds the table. Two consumers
read it and they share no file: `human_backpack.xacro`, which is what MoveIt
plans against, and `mount_guard_node.WEARER`, which is what the 150 mm floor is
measured against geometrically. A posture applied to one and not the other does
not error — it produces a sweep that runs five times, changes nothing, and
reports the shipped answer under five new labels. So the xacro's arm block is
GENERATED (`scripts/gen_wearer_posture_xacro.py`) and
`test_wearer_posture_has_one_source` regenerates it and fails on drift.

    SRL_WEARER_ARMS=folded ros2 launch srl_moveit_config demo.launch.py

`down` reproduces the shipped centres to the millimetre, which is the
regression control; an unknown posture name stops the xacro build rather than
falling back to the default. `dist_point` gained a rotation argument, which it
did not need while every wearer link was axis-aligned and needed the moment a
forearm could lie on its side.

## THE ANSWER, z = 1.120, floor 150 mm, N=2 grasp pose and N=10 over the full path

| posture | left, min \|x\| | right | home clearance | what happens |
| --- | --- | --- | --- | --- |
| **down** (shipped) | **0.325** | **0.450** | 0.1610 | left binds on the TORSO from 0.175 out; right binds on its own forearm and upper arm |
| **behind** | **nothing anywhere** | **nothing anywhere** | **0.1145, inside the floor** | the upper arm rotates back into the MOUNT |
| **folded** | 0.400 | 0.450 | 0.1610 | worse, as it must be: the forearms are in the work volume |
| **out** | **nothing anywhere** | **nothing anywhere** | **0.0601 / −0.0837** | the hands sit in the arms' own outboard space; the RIGHT arm is INSIDE the wearer's hand at the home pose |
| **none** *(no arms at all)* | **0.300** | **0.375** | 0.1610 | every column 0.125–0.300 binds on the TORSO |

`recordings/baselines/centre_vs_wearer_posture.json` and one
`centre_posture_<name>.json` per posture.

**Read the `none` row as the whole of the result.** It is not a posture, it is
the LIMIT: the wearer has no arms, so nothing about their arms can bind. The
centre is still shut, min \|x\| is still 300 mm (left) and 375 mm (right), and
at \|x\| = 0.10 the clearance is **−0.007 m (left) and −0.013 m (right)** — the
arm is inside the person's chest. Holding the arms clear is worth **25 mm on
the left and 75 mm on the right**. Real, and nowhere near the 300 mm needed.

**So the constraint is the torso, and the answer to "arms or torso or mount" is:
the torso for the working end, the mount for the floor itself.**

## THE MOUNT SETS A CEILING ON EVERY CLEARANCE FIGURE, AND IT IS 0.1610 m

`base_link` is a link no joint moves. Measured directly against each posture's
wearer:

| posture | base_link's nearest wearer part | clearance |
| --- | --- | --- |
| down / folded / out | torso | **0.1610** |
| behind | the wearer's own upper arm | **0.1145** |

Under the shipped wearer, **the 150 mm floor has 11 mm of headroom before any
joint moves at all**, which is why so many clearance figures in this project
read exactly 0.1610: that is not the arm's clearance, it is the mount's, and
it is the minimum over the whole chain. Under `behind` the same constant falls
to 0.1145 and every pose in the workspace is inside the floor, including home.
No software can move it. A mount change can, and that is the re-derivation
priced in TASK_SPEC §2A.

## TWO INSTRUMENT FAULTS FOUND ON THE WAY, AND BOTH WERE LIVE

**1. The runtime wearer had no arms.** `srl_teleop/clearance.py` is what
`ik_follower_node`, `real_homing_node` and `sim_to_real_bridge` enforce, and it
modelled a torso, a head and a pair of hips. The offline survey has always
included the wearer's arms, and on the inboard side those arms are exactly what
binds the right arm. **The planner's wearer and the follower's wearer were
different people, and the follower's had no arms.** Fixed: the six arm
primitives are expressed in their own link frames, so the table needs no
posture — the posture moves the LINK and TF carries it. `_clearance_from` now
iterates `ClearanceModel.PARTS` instead of a hardcoded triple, because a fixed
list there would have kept the arms out of the live check while the model
claimed them. Known-answer test added.

**2. `record_verification.py` drew the wearer's arms 150 mm low.** Its
`WEARER_SPEC` applied a (0, 0, −0.15) offset to the upper arm and (0, 0, −0.13)
to the forearm, which assumes the link frame sits at the shoulder. The xacro
puts the collision at the link origin with no offset, so the drawn upper arm
sat 150 mm below the one the score used, and the hands were absent entirely.
Both corrected, and the scoring path now includes the arms as well.

## THE PUBLISHED 0.425 / 0.400 IS STALE, AND THE RECONCILIATION SAYS SO

This sweep's `down` row returned 0.325 / 0.450 where
`recordings/baselines/centre_vs_height.json` records **0.425 / 0.400** at the
same height. A result that contradicts an earlier measurement is an instrument
check until the two are reconciled, so `measure_centre_vs_height.py` was re-run
UNCHANGED at z = 1.120 over the same x range: it returns **left 0.325 (y =
0.525), right 0.450 (y = 0.100)**, confirmed over the full path at N=10.

The two instruments agree with each other and disagree with the stored file.
The likeliest cause is the 2026-08-15 home change: the solver seeds from the
live joint state, so moving where the arm RESTS moves which null-space branch
it lands in, which is the same mechanism that cost T1 stage 2 seed 0 its
clearance. **The left arm gained 100 mm inboard and the right arm lost 50 mm,
and neither was noticed because nothing re-ran the sweep after the home moved.**
TASK_SPEC §2 is corrected accordingly.

## TWO CONTROLS I GOT WRONG, RECORDED BECAUSE THEY COST TWO POSTURES A REPORT

* **A control whose ground truth moves with the thing under test is an outcome
  in disguise.** The fixed-wearer sweep requires the outboard cell to CLEAR the
  floor and requires clearance to FALL as x goes to zero. Both hold for a
  wearer whose arms hang down; under `behind` and `out` the wearer's own arm
  moves toward that cell, so both failed and two postures refused to report —
  on exactly the postures the sweep exists to measure. They are results now.
  What survives is: the cell SOLVES (the loop ran), and the clearance MAP has
  at least three distinct values (it is not a dead channel). Asked of the map
  rather than of two hand-picked cells, because under `out` the wearer's hand
  sits at \|x\| = 0.655 and one of those cells stops solving.
* **Two stacks ran at once and a measurement talked to the wrong one.** A
  malformed shell line left an orphaned runner, so a second `move_group` came
  up beside the queue's. The `out` run then read a `robot_description` holding
  the `down` posture while its own process reported `out`. **The URDF control
  caught it** — it compares the loaded description's human-arm origins against
  the posture table — and that is the whole reason it exists. HARD CONSTRAINT 3
  is about measurements as much as about serial ports.

## WHAT THIS MEANS FOR THE BRIEF

The work cannot be in the centre of the table in front of the person, and no
instruction to the wearer changes that. The nearest the shipped configuration
gets is **325 mm off centre on the left arm and 450 mm on the right**, and the
best any posture can offer is 300 / 375 with the wearer having no arms at all.
Asking the wearer to hold their arms clear is worth 25–75 mm, is worth stating
as an operating requirement for the RIGHT arm specifically, and is not a route
to the centre.

`folded` is the posture people actually adopt when told to keep their arms out
of the way, and it is the WORST of the usable ones. `behind` and `out` are not
merely unhelpful: with a BACK-mounted rig they put the wearer's own limbs into
the robot's mounting envelope, and both breach the floor at the home pose
before the robot has been asked to do anything. If a wearer instruction is
given, it must be "arms down at your sides", which is what the model already
assumed and what nobody had checked was the best of the options.

---

# 2026-08-15 — THE HOME POSE: THE RENDER WAS RIGHT AND THE NUMBERS WERE ABOUT SOMETHING ELSE

Reported: "both elbows 0.14 m below the shoulders, wrists level, symmetric to
1e-16." Rendered: both elbows at shoulder height and the wrists angled down.
Both cannot be true, so the thing the picture is DRAWN FROM was measured.

## WHAT THE STACK ACTUALLY BOOTS INTO, read from /tf

`scripts/measure_home_render.py`, three RViz stills from the same boot of the
same stack (`recordings/baselines/home_render.json`):

| | left | right |
| --- | --- | --- |
| elbow (`forearm_link`) below its shoulder | **0.0793 m** | **0.2715 m** |
| elbow (`half_arm_2`) below its shoulder | 0.0181 m | 0.1125 m |
| elbow OUTBOARD of its own hand | **+0.188 m** | −0.018 m |
| tool axis elevation | **−0.01 deg** | **−0.00 deg** |
| hand | (0.55, 0.36, 1.18) | (−0.55, 0.36, 1.18) |

Mirror residual, the left arm reflected through x = 0 against the right:
**0.2826 m at the forearm**, 0.1416 at half_arm_2, 0.0955 at wrist_1, and
**0.0000 m at the shoulder, the bracelet and the hand**.

**The wrists were never the problem** — the tool axis is level to a hundredth
of a degree, pointing straight forward. **The elbows are**: not symmetric, not
down, and on the left arm the elbow is 188 mm further out than the hand it
belongs to. The hands are 1.10 m apart against a 0.36 m torso.

## IT IS NOT A CACHED CONFIG, WHICH IS WHAT THIS LOOKED LIKE

The live joint state matches `config/home_positions_*.txt` to **7.4e-05 rad**.
The stack is at the pose the source file stores. The stored pose is the
asymmetric one.

## WHERE THE "0.14 m, SYMMETRIC TO 1e-16" CAME FROM

Two quantities, each renamed on the way out:

* `presentation_pose.json` records `apex_above_shoulder_m = -0.1176` — the
  limb APEX below the **WEARER'S shoulder line at z = 1.46**. That is the
  0.14-ish number, and it is not the elbow, and it is not measured against the
  robot's shoulder.
* The exact symmetry is of the **HAND TARGETS**, (±0.55, 0.36, 1.18), which do
  mirror to 0.0000 m. The same file's own `elbow_mirror_residual_m` says
  **0.2825**, and that number was in the file the whole time.

A pose is not symmetric because its hands are. Nothing here was fabricated;
two measurements were reported under names that belong to different things,
which is the same class of error as quoting an IK boundary as a clearance
boundary.

## CAN IT BE FIXED? PARTLY, AND THE REST IS THE PLATFORM

`scripts/find_symmetric_home.py`. Mirrored hand targets, wrist held at the
level-forward orientation the pose already has, each target solved from a fan
of 31 seeds over the shoulder and elbow joints, every solution scored on
wearer clearance geometrically. Controls: the shipped home reproduces exactly
under this scorer (0.0793 / 0.2715 / 0.161), a target inside the torso finds
nothing, and the fan returns 26 to 28 DISTINCT elbow heights, so it is really
searching.

**Only three hand columns work at all** — |x| = 0.450, 0.500 and 0.550. Inboard
of 0.450 no pose is both reachable by both arms and clear of the 150 mm floor.

| hand column | left elbow drop | right elbow drop | best mirror residual | branches L x R |
| --- | --- | --- | --- | --- |
| 0.450 | **0.1701 m** | 0.2667 m | **0.1633 m** | 31 x 4 |
| 0.500 | 0.1583 | 0.2626 | 0.1644 | 31 x 28 |
| 0.550 *(shipped)* | 0.1443 | 0.2543 | 0.1616 | 30 x 31 |
| shipped pose itself | 0.0793 | 0.2715 | 0.2826 | — |

**Two of the five things asked for are not achievable on this rig:**

1. **"Hands in front of the chest at torso width" — no.** The innermost column
   that works is **450 mm off centre** against a torso half-width of 0.18 m.
   This is the same constraint Part 1 found: the TORSO, not the wearer's arms.
2. **"Both arms symmetric" — no.** The best mirror residual is **0.161 to
   0.164 m and it does not improve with more choices**: at x = 0.500 the right
   arm offers 28 distinct clearing branches instead of 4 and the answer moves
   by 1 mm. That is a structural floor, not a search that gave up. The two
   mounts' base axes ARE exact mirrors, and their full rotations differ by
   **168 deg about that axis**; joint_1 can absorb a rotation about the base
   axis, but the arm's link offsets perpendicular to it do not mirror, so two
   identical (non-mirrored) arms on mirrored mounts cannot hold mirror-image
   postures.

**Three of them can be had, and one was already true:**

* wrists level and forward — **already true**, and was before this started;
* elbows DOWN — the left elbow drops from 0.0793 m to **0.1701 m** below its
  shoulder, a 91 mm improvement; the right barely moves (0.2715 to 0.2667);
* elbows IN — the hands come in 100 mm per side, 0.550 to 0.450.

**NOT APPLIED AS HOME.** HARD CONSTRAINT 1 says the task set is re-measured
BEFORE home moves, which is what was done for the 2026-08-15 change, and the
improvement here does not meet the acceptance criteria it was asked to meet.
The candidate is recorded in `recordings/baselines/symmetric_home.json` and
photographed (`x450_front.png`) so the decision can be taken on the evidence
rather than re-derived. Moving home to it is a re-verification of every task,
for hands 100 mm nearer the chest and one elbow 91 mm lower.

## FOUR INSTRUMENT FAULTS FOUND ON THE WAY, AND ALL FOUR WOULD HAVE HIT THE RECORDING

1. **`record_rviz.py`'s scratch directory was a dead path.** `SCRATCH` was
   hardcoded to one session's `/tmp` directory, which no longer exists, so
   every render check, cached config and frame grab wrote into nothing. It is
   derived now and overridable with `SRL_SCRATCH`.
2. **A blank white frame was reported as captured.** The first still came back
   as a path to an empty viewport and the code called it a shot. The check now
   measures the fraction of viewport pixels differing from the modal colour and
   refuses below 2%; the known-blank frame scores **0.0003**, and the chrome
   has to be cropped first because the whole window scores 0.132 on menus and
   the status bar alone.
3. **A TF read reported "mirror residual 0.0000 m" from sixteen failed
   lookups.** Perfect symmetry and an empty loop are the same picture. It
   spins and retries now, and the missing-frame control is what caught it.
4. **`scripts/env.sh` never exported `FASTDDS_BUILTIN_TRANSPORTS=SHM`** — the
   one variable HARD CONSTRAINT 5 calls "NOT optional". It was set only in the
   shells `sim_session.py` opens for the launch, so a stack started that way
   and any process started by hand could not see each other. Measured: with it
   unset, `ros2 service list` hung past 30 s and a measurement reported "no
   /compute_ik" while `move_group` was up and answering; with it exported the
   same query returned immediately. **A transport mismatch reads exactly like
   a dead stack**, and this cost two full measurement runs before it was found.

---

# 2026-08-15 (later) — PREDICTIVE AVOIDANCE: THE NULL SPACE IS NOW REAL, AND IT STILL CANNOT HELP

An earlier pass built a seed-fan "null-space" sampler, measured it worth 4.4 mm
at the head, and said plainly that seed sampling is not redundancy resolution
and that the real answer is an explicit parameterisation. **That was built.**
The conclusion did not change, and it is now a much stronger statement.

## WHAT WAS BUILT

`srl_teleop/predictive_avoidance.py`, additions:

* `jacobian(fk, q)` — a 6 x n end-effector Jacobian by central differences from
  ANY forward-kinematics callable, so the same code runs against `/compute_fk`
  offline and a local KDL chain in a follower;
* `null_projector(J)` — N = I − J⁺J, verified idempotent and symmetric;
* `clearance_gradient(...)` — d(clearance)/dq, returning None if ANY probe is
  None, because half a gradient points somewhere nobody chose;
* `NullSpaceRetreat.retreat(...)` — integrates q̇ = N ∇c with a **task-space
  correction each step**;
* `tangential_target(...)` — strips the INTO-the-wearer component of the
  operator's step and keeps the across component.

**THE TASK-SPACE CORRECTION IS NOT COSMETIC.** N ∇c is tangent to the
constraint, so a FINITE step along it leaves the manifold. Measured on the
constructed planar arm: **5.5 mm of hand drift** over eight steps of 0.05 rad,
which an operator would feel. One Newton step of −J⁺·pose_error per iteration
takes the residual to **under 0.1 mm**, and the test asserts it.

**TESTED ON A CONSTRUCTED ARM, NOT ON THE ROBOT.** A three-link planar arm has
a known one-dimensional null space, so "J q̇ = 0 leaves the end effector alone"
is checkable to machine precision. If the retreat moved the elbow on the Gen3
that would be evidence about the Gen3, not about the arithmetic. 27 tests.

## AND THEN IT WAS DRIVEN AT THE WEARER, WHICH IS THE POINT

`scripts/verify_predictive_avoidance.py`, THREE targets and FOUR conditions.
The third target is new and it exists to be FAIR: `head` and `torso` drive the
hand straight at the wearer, which is the right worst case for the floor and
the worst possible case for a stage that works by keeping the across-the-body
part of a motion. `across` sweeps over the front of the chest instead.

| target | off | seed fan | **null space** | tangential |
| --- | --- | --- | --- | --- |
| head | 15/15 | 4/15 | **4/15** | 5/15 |
| torso | 14/15 | 4/15 | **4/15** | 5/15 |
| across | 9/15 | 3/15 | **3/15** | 4/15 |
| best single-step clearance gain | — | — | **0.0045 / 0.0000 / 0.0056 m** | — |

**The genuine projection is worth the same as the sampler it replaced: nothing.**
Between 0.0 and 5.6 mm.

## AND HERE IS WHY, WHICH IS THE PART WORTH KEEPING

Every refusal was made to name the ARM LINK, not only the wearer part. Across
all three targets and 28 refusals, the closest link is the same one **every
single time**:

    left_end_effector_link  <->  head    11 refusals, worst -0.1550 m
    left_end_effector_link  <->  torso   17 refusals, worst -0.1343 m

**The HAND is what is inside the person. Not the elbow, not the forearm.** And
the null space is, by definition, the family of postures that hold the HAND'S
POSE FIXED. So no amount of correct redundancy resolution can help: the
offending link is the one the method holds still. Refusing is the right answer
and it is right for the right reason.

This retires the earlier hypothesis. It was not that TRAC-IK re-seeding failed
to find the elbow circle; the elbow circle is not where the problem is.

## WHAT DOES HELP, AND BY HOW LITTLE

The tangential stage moves the thing the null space cannot: the commanded
target. It rescues **exactly one step per target**, sliding the hand 47 to
57 mm across the wearer instead of stopping. In these scenarios the operator's
motion ends up purely inward, and a purely inward step has no across component
to keep, so the stage runs out. That is a real limit of the method and not an
implementation defect.

## THE NUMBERS THE BRIEF ASKS FOR BY NAME

* **lookahead**: 0.30 s, straight-line, with a staleness guard so a paused
  master predicts NO motion rather than flying on its last velocity;
* **achieved loop rate**: median **0.131 to 0.153 s per commanded pose**, so
  about **7 Hz**, for the whole decision including the fan, the projection and
  the clearance evaluation, through the SERVICE-based solver. A follower with a
  local IK library would be faster and this figure does not pretend otherwise;
* **EE residual**: **0.00001 m**. The operator cannot feel the avoidance,
  which in this case is trivially true because nothing meaningful moved;
* **never locks up**: every step returned a decision under every avoidance
  condition. The only "no decision at all" steps in the whole run were under
  **avoidance OFF**, where the baseline solver simply failed to solve;
* **the floor stayed last**: unchanged, and it caught a bug of mine — see below.

## A BUG OF MINE THAT THE NUMBERS CAUGHT

The first integration accepted the projection's result whenever it beat the
fan's clearance and marked the step COMPLETED. That bypassed the floor.
Measured, it reported **10 of 15 torso steps completed while the projection's
best clearance gain for that target was 0.0000 m** — a completion rate produced
by the bookkeeping rather than by the robot. A result that improves clearance
without reaching the floor is still a refusal, and it is recorded as one now.
The corrected number is 4 of 15.

The summary also printed "lockups N" where N was the count of REFUSED steps,
against this file's own definition, and compared only two of the conditions.
Both fixed.

## STILL NOT WIRED INTO `ik_follower_node`, AND NOW FOR A BETTER REASON

Not because it is unvalidated — it is validated. Because it is **measured to be
worth nothing on the failure it was built for**. `srl_console` names three
stages (collision-aware IK, redundancy re-seeding, the hard floor) and those
are still exactly the three the live stack runs; nothing here re-adds a label
for a stage the follower does not execute.

**What would actually help is a smaller claim than redundancy resolution:** the
hand is the problem, so the useful mitigations are ones that act on the
COMMANDED POSE — the tangential slide, a speed limit that scales with
clearance, or a hard stop that the operator can feel through the master. The
elbow was never the thing in the way.

---

# 2026-08-15 (later still) — T2'S TRAY WAS ELASTIC AND T1'S TWO STAGES WERE TWO TASKS

## THE TRAY DEFORMED TO FIT WHATEVER THE ARMS WERE DOING

`clip_scene` drew T2's tray between the grippers with length **`span + 0.06`**,
where `span` is the LIVE distance between them. Over the shipped clip set that
ran **0.487 to 1.211 m against a 0.560 m spec** — from 73 mm short to 651 mm
long, and held at both ends the whole way.

That is not cosmetic. T2's two headline criteria are "tray held by BOTH
grippers" and "ball visible ON the tray", and **neither could fail**: the board
resized to reach whichever hands existed, and the ball rode a surface that was
redefined every frame to stay underneath it. The same clips reached **23 deg of
tilt with the ball still on**, against a declared drop angle of **6.8 deg**,
which is 60 mm of height difference over 500 mm and is the entire failure
criterion of the task.

Fixed:

* the tray is drawn at the SPEC length from `tasks.TASK_B` and **oriented along
  the line between the grippers**, so a separation error is visible as what it
  is — a rigid board that does not reach one of the hands, or one whose end
  sticks out past it. `tray_fit_err_m` is recorded per tick beside the tilt;
* the ball **falls off** past `fail_tilt_deg` and the drop **LATCHES**. A ball
  that climbs back on when the tray levels turns a carry that failed in the
  middle into a carry that passed, which is the same defect with the opposite
  sign.

**AND THE AUDIT'S OWN CHECKS COULD NOT FAIL EITHER.** T2-1 asked "is the tray
drawn between both grippers", which an elastic tray satisfies by construction,
and T2-2 asked "is a ball drawn on the tray top". Both now check the property
that makes the criterion capable of failing: the tray must be RIGID at the spec
length and must not contain the elastic expression, and the ball must have a
tilt-conditioned, latching drop. 12 known-answer tests; the audit's own
self-test still detects all 13 deliberate breaks.

## T1'S TWO STAGES HAD DIFFERENT TARGETS, AND STAGE 2 HAD NO COLOUR RULE

Stage 1 places four cubes onto two coloured pads: blue to blue, green to green.
Stage 2 was placing onto **coordinates drawn from each arm's own surveyed
cells** — arbitrary points, no colour, and **nothing drawn on screen to place
onto**, because `clip_scene`'s pad block ran for `t1` only. The half of T1 that
exists to demonstrate divided attention had no colour matching in it at all.

**The pads cannot literally be shared, and that is geometry.** Both sit at
positive x, and the right arm's innermost column that is reachable AND clear is
\|x\| = 0.450 — it cannot reach +0.450, which is 900 mm across the far side of
a person. So the pair is MIRRORED for the arm that has to reach it: same two
colours, same 160 mm spacing, same y. A cube goes to the pad of **its own
colour on the side it landed on**, which is stage 1's rule with the side made
random.

One subtlety worth the test it has: the colour follows the cube's place in the
**whole draw**, not its place within one arm's share. Keyed on the per-arm
index, a 3/1 split gives three blue and one green, which is a different task
from the one T1 declares. 10 tests, including that split.

## WHAT REMAINS BLOCKED IN T1, UNCHANGED AND RESTATED

* **T1-1, cubes ON the table: still BLOCKED.** They float 150 mm above it.
  Every support geometry was measured fatal, and the only lead is the
  front-edge cell found on 2026-08-15 (x = 0.350, table top 1.00, near edge
  0.530) which is one cell and not a layout for six objects.
* **Top-down and table-height grasping remain mutually exclusive**, so the
  brief's "grasp from above if the geometry permits" resolves to near-side:
  measured **0 of 840 cells** for top-down on a surface at any table height
  0.70–1.10 for either arm. The pinned wrist arrives from below, and every
  teleop mode pins it.
* **Cubes and pads in the CENTRE remains unreachable**, now for the third time
  and by a third method: Part 1's posture sweep put the limit at the torso, not
  the wearer's arms, with min \|x\| 0.300 even with the wearer's arms deleted.

Audit after these changes: **45 PRESENT, 0 MISSING, 1 BLOCKED (T1-1), 1
pixels-only.**

---

# 2026-08-15 (later still) — FINDING 3 SETTLED: THE MODE DIFFERENCE IS REAL, ITS SIZE IS NOT KNOWN

Finding 3 above reports that scripted clips with no operator show different
achieved motion by mode, on a SINGLE run per cell against a 3% same-mode spread
taken from the one cell recorded twice. A single repeat is not a measurement,
so the two ways it could have been an artefact were tested directly against the
recorded set. `scripts/analyse_mode_difference.py`,
`recordings/baselines/mode_difference.json`.

## THE TWO ALTERNATIVES, BOTH REFUTED

**1. It is the recording length.** `ee_travel_m` is a sum of \|dp\| over ticks,
so a run watched for longer accumulates more path over the same motion. Refuted
by runs of IDENTICAL duration:

| task | duration | modes | travel | difference |
| --- | --- | --- | --- | --- |
| T2 | 15.83 s | 01 vs 03 | 1.075 vs 1.365 | **27%** |
| T0 | 19.83 s | 03 vs 04 | 2.034 vs 1.519 | **34%** |
| T3 | 20.25 s | 03 vs 06 | 1.335 vs 1.255 | **6.4%** |

**2. It is run-to-run noise.** Refuted by exact agreement. On T1S2, modes 01,
02, 03 and 06 share a net displacement of **(0.1901, 0.1374) to four decimal
places**; on T2, 01 and 02 share (0.6286, 0.6565); on T3, 01 and 02 share
(0.6018, 0.5948). Four modes agreeing to 0.1 mm is not what independent noise
looks like — it is what running the same thing looks like.

**So the difference is real, and finding 3 stands. TASK_SPEC section 1 keeps
its correction.**

## AND HERE IS WHAT THE SET CANNOT SETTLE, WHICH IS THE SIZE

Both metrics are properties of the OBSERVATION WINDOW as much as of the motion:

* `ee_travel_m` sums over recorder ticks, so it counts sensor noise at a
  stationary arm and grows with how often and how long the node watched;
* `ee_net_m` is `dist(ee_first, ee_track)` where **`ee_first` is the first pose
  the scene node happened to see** — not the pose the task started from. That
  is why several modes share it exactly: those runs caught the same window, and
  the number is partly about the recorder starting up.

That is enough to say the difference is not noise and not duration. It is not
enough to give the difference a trustworthy magnitude, and the magnitude is
what a baseline subtraction needs.

Fixed here: `ee_samples` is written into `scene_events.json` beside the two
metrics, together with a caveat naming this problem, so the next comparison can
normalise by sample count rather than assume two runs are comparable. Not fixed
here: anchoring the path metric on the first COMMANDED waypoint rather than the
first observed pose, which changes what every recorded clip measured and so
belongs with the re-record rather than half way through one.

## WHAT IS STILL REQUIRED, AND IT IS NOT BLOCKED ON ANYTHING

N >= 5 scripted repeats per (mode, task) cell, no operator, same seed; a metric
that starts when the task starts; and the WITHIN-mode spread reported beside
the BETWEEN-mode difference. It needs no hardware, no participants and no
ethics approval. `docs/research/02_baseline_and_hypotheses.md` section 5.0 now
carries it as a threat to validity ahead of fatigue, because it bears on every
hypothesis that compares conditions.

---

# 2026-08-15 (part 6, before recording) — T1 STAGE 2 SEED 0 WAS A STALE POOL, NOT A BAD DRAW

The pre-recording re-verification at N=10 reproduced the known regression:
**stage 2, seed 0, RIGHT arm, 26 waypoints inside the 150 mm wearer floor,
worst 0.1135 m, with ZERO IK failures.** Seeds 1 and 2 were clean, which is
what made it look like one unlucky draw.

**It was the pool.** `work_surface_region.json` was surveyed BEFORE the
2026-08-15 home change and its right-arm `clear_cells` reach \|x\| = 0.400. The
right arm's innermost column that is reachable AND clear at the CURRENT home is
**0.450** — re-measured over the full path at N=10 in part 1, and the same
staleness that made `centre_vs_height.json`'s published columns wrong. Seed 0
drew a right-arm cube from the 50 mm of pool that the arm can no longer work,
and the breach is on the path rather than at the grasp pose, which is why an
IK-clean survey passed it.

`INNERMOST_SAFE_X = {"left": 0.325, "right": 0.450}` is applied in `_region()`
and **11 stale right-arm cells are dropped**; the left's measured limit is
inboard of the survey's own 0.425 and costs nothing. Re-verified:

| | waypoints | IK failures | worst clearance | below floor |
| --- | --- | --- | --- | --- |
| seed 0 left | 90 | 0 | 0.1571 | **0** |
| seed 0 right | 90 | 0 | 0.1610 | **0** |
| seed 1 both | 97 | 0 | 0.1571 / 0.1610 | 0 |
| seed 2 both | 88 | 0 | 0.1571 / 0.1610 | 0 |

`verify_t1_paths.py` exits 0. **This is the class fix rather than the one-draw
fix**, which is the choice NEXT_SESSION §5 left open: re-drawing seed 0 would
have moved one cube and left the other stale cells in the pool for the next
seed to find.

## AND A CONTROL FAILED IN A VERIFIER I DID NOT CHANGE

`verify_msc_tasks.py` refuses to report, because its fourth control — "a
known-good pick", which is task A's own pick in the LEGACY A/B/C scene — reads
UNREACHABLE. The other three controls pass. That verifier is not what the MSc
task set is checked with (`verify_t1_paths.py` is), and nothing in this
session's changes touches `clip_tasks.A_PICK` or scene "a".

It is recorded here rather than fixed, because the honest reading is that the
legacy A/B/C coordinates have not been re-derived since the home change and
the refusal is that verifier telling the truth about itself. **Its refusal is
correct behaviour**: it will not print a count when a control says the
instrument is wrong. What it needs is A_PICK re-derived at the current home,
which is a job for whoever next needs the A/B/C set.

---

# 2026-08-15 (part 6) — T2 CARRIES THE TRAY WITH ITS HANDS OPEN

Three verifiers had been failing for the whole life of the MSc clip set:
`verify_gripper_motion`, `verify_grasp_quality`, `verify_object_attachment`.
Their own source carried the right diagnosis — *"NOTHING IN THE CLIP PATH
WRITES grip_trace.json AT ALL, 0 files on disk across 25 recorded cells"* — and
they refused to report a pass on zero inputs, which is correct and was ignored
because it looked like tooling noise.

**The missing piece was the PRODUCER, not the check.** `clip_scene` now writes
`grip_trace.json` beside `scene_events.json`: the knuckle angle per arm per
tick, the same signal the attach gate already uses. Two more things had to be
fixed before the verifiers could speak:

* their glob knew only the legacy FOUR-level path
  (`<mode>/<task>/<scenario>/<condition>`); the MSc sweep writes THREE;
* they unpacked four path components and **crashed** on a three-level path;
* and `GRASP_TASKS` held only retired names in lower case while the path
  spells the task in CAPITALS, so `task in GRASP_TASKS` was False for every
  MSc clip. That is how "checking gripper motion in 5 runs" and "grasp clips:
  0" appeared together.

## AND THEN IT SPOKE

| task | arm | knuckle range | open | close | release | pixels |
| --- | --- | --- | --- | --- | --- | --- |
| t1 | left | 0.00–0.42 | yes | yes | yes | 186879 → 16265 **OK** |
| t1s2 | left | 0.00–0.42 | yes | yes | yes | 188853 → 26690 **OK** |
| t3 | left | 0.00–0.52 | yes | yes | yes | 183622 → 10346 **OK** |
| **t2** | — | **0.00–0.00** | **no** | **no** | **no** | — **CHECK** |

**T2's grippers are fully OPEN for all 992 samples of the carry, on both arms.**

The task's own schedule commands the opposite: `grip=lambda n: {"left":
[CT.grip_for(30)] * n, "right": [CT.grip_for(30)] * n}` — closed to the tray's
grip block width on **every** waypoint, with a comment saying "both grippers
are already closed on the tray and STAY closed". The other three tasks cycle
their grippers correctly through the same command path, so the path works; T2
specifically is commanded closed and recorded open.

**So T2-1, "tray held by BOTH grippers", is false in the pixels.** It has been
false for as long as the task has existed, and it was invisible because the
tray was DRAWN between the two grip points whether or not anything was
gripping — the elastic-tray defect fixed earlier today. Rigid or elastic, a
board drawn between two open hands still looks held.

Not fixed here. The schedule is right, the command path works for three other
tasks, and the defect is somewhere between `run_abc`'s gripper publishing and a
constant-valued schedule — plausibly an only-on-change publish, since T2 is the
one task whose commanded grip never varies. It is named, evidenced and left for
the person who owns that path rather than guessed at.

## ALSO FIXED: T0 WROTE AN EMPTY TRACE

`self.t0` was set inside the T2 branch, so a task with no carry never started
its clock and appended nothing. An empty file and a stationary gripper are
precisely the two things these verifiers exist to tell apart. The clock starts
for every task now.

---

# 2026-08-15 (part 6) — "0 OF 10 GENUINE GRASPS" WAS A MISSING FILE AND A PINNED WRIST

With the grip trace produced, `verify_grasp_quality` ran for the first time on
the MSc set and reported **0 of 10 clips show a GENUINE grasp by all four
criteria**. The scene's own record says otherwise, and it is not ambiguous. T1
under 03, from `scene_events.json`:

    GRASPED cube_0 knuckle 0.383 (needed 0.3812)  ... RELEASED carried 0.2675 m
    GRASPED cube_1 knuckle 0.3945 ...              RELEASED carried 0.2310 m
    GRASPED cube_2 knuckle 0.3912 ...              RELEASED carried 0.4026 m
    GRASPED cube_3 ...                             RELEASED carried 0.3445 m

with `min_pad_obj_m = 0.0` on every cube — the pads touching the object — and
all four finishing on their pads. Four grasps, four carries, four places.

## CRITERION 4 READ A FILE THAT DOES NOT EXIST

`attach` is `cap.get("grasp_planned") and the knuckle entered the holding
band`, where `cap` is `rviz_capture.json` — the LEGACY recorder's file. The MSc
sweep does not write it, so `cap` was `{}` and criterion 4 was **False for
every clip whatever the fingers did**.

It falls back to the scene's own record now, reading "planned" as THE TASK
DECLARES A GRASPABLE OBJECT. That is a statement about the task; reading it
from the GRASPED events instead would make the criterion test itself. After
the fix: `attach` is True for t1, t1s2 and t3, and False for t0 (no graspable
object, which is by design) and for t2 (the open-gripper defect above).

## AND CRITERION 2 ASKS THE PINNED WRIST FOR SOMETHING IT CANNOT DO

t1 and t1s2 still fail, on **wrist**, which requires the gripper's principal
axis to rotate more than 8 degrees during the clip. Measured: **0.3 to 2.1
degrees.**

That is not a defect, it is the platform. `run_abc.send()` writes the pinned
anchor into every waypoint of every task under every mode, so the wrist does
not rotate during a T1 pick and place — it is held at the 30.7 degree approach
for the whole path. A criterion that treats wrist rotation as evidence of a
genuine grasp cannot be satisfied by a system whose defining feature is a
pinned wrist, and the file's own comment already suspected it: *"stationary by
construction and its measured axis barely moves -- it reported 2.9 deg on a
grasp whose wrist was correctly aligned."*

**Read the score with that attached**: 2 of 10 GENUINE is t3 twice, where the
meter presentation does turn the wrist; t1 and t1s2 satisfy fingers, approach
and attach and fail only the criterion the platform forbids; t2 fails on a real
defect; t0 has no grasp at all and should not be counted.

Not silently weakened. The criterion is right for an arm that can rotate its
wrist during a grasp and wrong for this one, and which of those is true is a
platform decision rather than a verifier tuning parameter.

---

# 2026-08-15 (part 6) — THE DANCE ROUTINES COULD NOT BE FILMED, AND THE CHECK THAT KNEW IT CRASHED

`verify_dance_paths.py` exits non-zero on the current geometry, and for two
reasons stacked on top of each other.

**First, it crashed instead of reporting.** The line that prints an example
failure is `"   e.g. %s" % worst[0]`, where `worst[0]` is a 3-tuple, so Python
tries to fill one conversion from three arguments and raises TypeError. That
line only runs WHEN THERE IS A FAILURE TO REPORT, so the verifier threw an
exception exactly when it had something to say and printed a clean table the
rest of the time. Fixed with `% (worst[0],)`.

**Then it spoke, and the envelope was wrong:**

| routine | before | e.g. |
| --- | --- | --- |
| d1 flow | **45 of 382 waypoints FAIL** | right arm at x = −0.28 |
| d2 pulse | 0 of 356 | — |
| d3 play | **49 of 554 FAIL** | left arm at x = +0.272 |

The choreography's box was `|x| ∈ [0.26, 0.50]`, taken from task0's free-space
anchors with NO FURNITURE and no wearer clearance floor. The columns each arm
can actually work — reachable AND clear of the 150 mm floor at the current home
— are **0.325 on the left and 0.450 on the right**. Most of that box is inboard
of the right arm's limit, which is why the right arm failed at −0.28 and the
left at +0.272.

**One symmetric band, not two.** A per-arm band would let the left arm work
125 mm further in than the right, and these routines are built on unison, canon
and mirroring, so an asymmetric envelope makes the arms trace visibly different
shapes and the choreography stops reading. Both arms use the RIGHT arm's limit
plus the project's 20 mm margin, and the outer edge moves out to keep the span:
the measured clear region runs to |x| = 1.00 on both sides.

    X_IN, X_MID, X_OUT   0.28, 0.38, 0.48  ->  0.48, 0.60, 0.72
    BOX |x|              0.26 .. 0.50      ->  0.47 .. 0.76

Re-verified: **d1 0 of 382, d2 0 of 356, d3 0 of 560, over 3898 IK calls, 0
failures.** The routines are filmable now and were not before.

---

# 2026-08-16 (part 7) — WHAT THE NEW CLIPS SHOW, LOOKED AT

Frames pulled from the re-recorded set and read against the MUST BE TRUE lists.
Stills kept in `docs/img/home_render/`. Answers below are from the PIXELS
first, with the measurement that agrees beside them.

## THE SIX THINGS THE BRIEF ASKED TO SEE

| asked | seen | |
| --- | --- | --- |
| symmetric home, elbows down | **no** | elbows 0.0793 (L) and 0.2715 (R) below their shoulders, forearm mirror residual 0.2826 m. Best achievable 0.1633 m, and it does not improve with more IK branches |
| objects ON the table | **no** | they float. Visible in T1, T1S2 and T3: the markings and every object sit above the white top |
| pads and cubes in the CENTRE | **no** | they are at the two ends. min \|x\| 0.325 (L) / 0.450 (R), and 0.300 / 0.375 even with the wearer's arms deleted |
| arms working in FRONT of the person | **no** | T1S2 shows both arms out at the table's ends, for the same reason |
| the tray RIGID | **yes** | and see below, because making it rigid showed something |
| the elbow moving aside near the wearer | **no** | not wired into the follower, and measured worth 0.0 to 5.6 mm because the HAND is what breaches, not the elbow |

## THE TRAY WAS THE POINT OF THE EXERCISE

T2's frame shows a **rigid slab of fixed length across the wearer with NEITHER
GRIPPER ON IT** — the left hand is down beside the left mat, the right is up
near the shoulder, and the tray floats between them touching nothing. The ball
is lying on the table where it fell.

That is the change working exactly as the brief intended: *"draw it rigid at
spec length so separation error becomes visible as the tray not fitting, which
is what the task measures."* The elastic tray hid this by stretching to meet
whichever hands existed. The grip trace says the same thing in numbers — 0.00
knuckle for all 992 samples on both arms — and the picture now agrees with the
data instead of contradicting it.

## WHAT IS RIGHT, AND WORTH SAYING BECAUSE IT WAS NOT ALWAYS

* **the table is WHITE in the pixels**, not merely requested;
* **the card comes first and reads like a person wrote it**: "Pick up four
  cubes and place each one on the mat of its colour. Watch the fingers close on
  the cube, not above it." No em dashes, no banned words, no tricolon;
* **every clip opens on the presentation pose** — `opened_on: presentation`
  recorded per cell and confirmed in the opening frames;
* **one marking per arm in a one-arm task and two in a both-arm task**, with
  every object inside the marked cells;
* **stage 2 now has coloured pads on BOTH sides**, which it never had: the
  frame shows a blue and a green mat in each arm's region;
* **T1 grasps for real** — four GRASPED and four RELEASED events, knuckle
  0.383 to 0.395 against a needed 0.3812, pad-to-object distance 0.0 m,
  carried 0.23 to 0.40 m, all four landing on their pads;
* **the routines read as choreography again**: D1's frame has both arms in the
  same mirrored shape, reaching, with no table in the scene. Before today's
  envelope move, d1 and d3 had 94 unreachable waypoints between them and would
  have stalled on camera.

## THE HONEST SUMMARY

Four of the six things asked for cannot be shown, and none of the four is a
recording fault. Three of them — objects on the table, work in the centre, arms
in front of the person — are the same measured constraint arriving from three
directions, and part 1 established that it is the WEARER'S TORSO and the MOUNT
rather than anything software can move. The fourth, the elbow retreat, is a
strategy measured to be worth nothing on this arm because the offending link is
the one it holds still.

The two that can be shown are shown, and the rigid tray immediately exposed a
defect that had been invisible for the life of the task.

# 2026-08-17 — T1 CANNOT BE RE-RECORDED: THE OBSERVE POSE ARRIVES INSIDE A TOLERANCE, AND VISION DEPROJECTS FROM THE POSE IT DID NOT REACH

**The T1 re-record is BLOCKED, and the block is in the vision path rather than
in the task, the layout or the home.** Two clips were recorded under
`06_full_autonomy` and both were DISCARDED. Neither is on disk; the committed
2026-08-15 clip is left in place, exactly as `d3` was left when
`verify_dance_paths` disagreed with a sweep that had reported OK.

## THE GEOMETRY IS CLEAN, SO THIS IS NOT THE LAYOUT

Run first, as the pre-record gate, against a live `/compute_ik` at the
2026-08-16 home. `recordings/baselines/t1_paths.json` had been stale since
2026-08-15 — it predates the solved home — and is refreshed by this run:

    both controls correct    a waypoint driven into the wearer FAILS
                             home clears the floor at 0.1596 m
    stage 1                  171 waypoints, 0 IK failures, worst clearance
                             0.1610 m, 0 inside the 150 mm floor
    stage 2, seeds 0/1/2     both arms, 0 IK failures, 0 inside the floor
    TOTAL                    0 failures over 7522 IK calls

Stage 2 seed 0's right arm — 25 of 98 waypoints inside the floor at the
2026-08-15 home — is CLEAN at this home. That regression is closed.

## WHAT THE CLIPS ACTUALLY SHOWED: THREE CUBES OF FOUR

Both discarded runs recorded a genuinely moving arm (left travel 5.2709 m and
5.3016 m, staged on the presentation pose, capture gated on motion) and both
placed three cubes. `cube_3` was never grasped:

    cube_0/1/2   min_pad_obj_closed_m  0.0000      captured
    cube_3       min_pad_obj_closed_m  0.0309      gate 0.030 -- 0.9 mm outside

The sweep reported `OK  run exited 0` for the first run. The second run's own
verifiers caught it: `verify_gripper_motion` and `verify_object_attachment`
both FAILED. **The run gate and the verifiers disagree, and the verifiers
win** — the same rule that discarded the d3 clip.

## IT IS NOT LAG, AND THE NUMBER SAYS SO TO A TENTH OF A MILLIMETRE

T1 has been built from DETECTED cube positions since 2026-08-16 (`cubes=None`
keeps the declared layout; the sweep passes what `observe_and_detect` saw).
The scene still draws the cubes where they really are, so a detection error is
a grasp error. Measured against ground truth (`T1_CUBES`):

    cube      detected in-sweep        dx        dy      error    30 mm gate
    cube_0    (0.5459, 0.0954)     -14.0 mm  -24.3 mm   28.0 mm   ok
    cube_1    (0.6062, 0.0927)     -12.8 mm  -25.8 mm   28.8 mm   ok
    cube_2    (0.6681, 0.0921)     -11.7 mm  -27.6 mm   30.0 mm   ok
    cube_3    (0.7294, 0.0907)     -10.6 mm  -29.1 mm   31.0 mm   MISS

**cube_3's 31.0 mm detection error and its 30.9 mm recorded miss are the same
number.** Every cube is displaced in the SAME direction, which is a constant
camera-pose error and not accumulating follower lag — the identical-value
signature this repository already learned to read when three cubes reported
0.0314.

## THE INSTRUMENT WAS CLEARED BEFORE THE MEASUREMENT WAS BELIEVED

The standing rule says a surprising failure is evidence about the instrument
until the instrument is cleared. It was, and the detector is FINE:

    condition                                    detection error
    standalone, clean stack, settle 6            1.3 - 3.0 mm
    standalone, clean stack, settle 20           1.3 - 3.0 mm
    standalone, detections #1, #2, #3            IDENTICAL to 4 decimal places
    inside the sweep (twice)                     28.0 - 31.0 mm
    after a clip has run                         REFUSED, did not reach observe

Three consecutive standalone detections returned
`(0.5592, 0.121) (0.6205, 0.1217) (0.6817, 0.1211) (0.7428, 0.1212)` — the
committed `vision_drives_grasp.json` baseline, reproduced exactly. **The first
detection after a fresh camera is not the biased one**; that hypothesis was
tested with a three-detection control on a clean stack and REFUTED. Settle
time is not the variable either: settle 6 gave both 4.0 mm and 25.5 mm.

## THE CAUSE: `stage()` ACCEPTS 0.02 rad PER JOINT, AND THE FOLLOWER IS ON THE SAME CONTROLLER

`verify_colour_vision.Vision.stage()` verifies arrival off `/joint_states`
with `tol=0.02` — per joint, on all seven. `observe_and_detect` then reads
`n.cam_pose()` from TF and deprojects the image through it.

Standalone the follower has no target, the arm lands ON the observe pose, and
the deprojection is right. **Inside the sweep `ik_follower_node` is streaming
position commands to the SAME controller**, so the arm settles at the EDGE of
that tolerance instead of on the pose. 0.02 rad on the proximal joints of a
0.7 m arm is centimetres at the camera, and the whole scene shifts with it.
The third row of the table is the same fault with the volume turned up: once a
clip has run and the follower is holding the arm, the observe move does not
arrive at all and stage-detect refuses by name.

This is the project's one-publisher-per-controller rule — already recorded for
`stage_presentation_pose.py` and for `vr_pose_mapper` — arriving in the VISION
path, where it is quieter, because nothing here fails. It returns four cubes,
in the right order, with the right colours, and every coordinate is wrong by
about the width of the capture window.

## WHAT TO DO, AND WHAT NOT TO

**Do not widen the 30 mm gate.** It is not too tight; the detection is off. A
wider gate would grasp cube_3 from a pose that is still 31 mm wrong and hide
the fault in exactly the clips meant to be evidence for it.

**Do not tighten `tol` alone either.** A tighter tolerance against a follower
that is actively holding the arm turns a wrong answer into a timeout, which is
the third row of the table.

The fix belongs where the other two instances were fixed — one source at a
time on that controller: pause the follower for the observe move as the sweep
already does for the staging move (`bridge_enable` / `bridge_disable` exist),
or have the camera stamp each frame with the pose it was RENDERED from and
have `observe_and_detect` refuse a frame that predates arrival. The second is
the stronger fix, because it makes the failure impossible to record silently
rather than merely unlikely.

Either way it needs a known-answer test: a deliberately displaced observe pose
must produce a detection error of the size that displacement predicts. A check
that cannot fail on a broken input is not a check.

## WHAT THIS DOES NOT TOUCH

T0, T3, D1 and D2 do not look before grasping — `t1` is the only task the
sweep runs `stage_observe_and_detect` for — so the five modes recorded on
2026-08-16 are unaffected. T1S2 and T2 are still on the 2026-08-15 geometry
and still need re-recording; T1S2 uses the same vision path and is blocked
behind the same fix.

# 2026-08-17 (later) — "THE EXPERIMENT STARTS FROM THE OLD HOME": IT DOES NOT, AND WHAT DOES HAPPEN IS WORSE FOR THE DATA

The report was "RViz boots into the new pose, but a task run starts from the
old one." It is half right, and the half that is wrong matters, because the
place the pose is actually lost is not the place the report points at.

## WHAT WAS MEASURED

A watcher subscribed to `/joint_states` and to the mode's own entry topic, and
recorded the joint state at the instant the FIRST waypoint was published on it
— the actual start of a run, not the xacro. Compared against
`config/home_positions_{arm}.txt` with the difference taken WRAPPED, so a
continuous joint a whole turn out does not read as 6.28 rad.

Fresh teleop stack, T1 under `01_master_teleop`, `--vision` with a staged
detection, worst per-joint distance from home:

| | left | right |
| --- | --- | --- |
| at boot | **0.0000** | **0.0000** |
| after `stage_presentation_pose.py` | 0.0000 | 0.0000 |
| after `stage_observe_and_detect.py` | 0.0000 | 0.0000 |
| **run 1, first commanded waypoint** | **0.0000** | **0.0000** |
| between runs — nothing else ran | **1.2338** (joint_6) | **0.6248** (joint_4) |
| **run 2, first commanded waypoint** | **1.2338** | **0.6248** |

Run 1 was measured twice, once through the recording sweep's full staging
sequence and once with no staging at all, and both read 0.0000.

## SO THE STRETCH THE REPORT NAMED IS CLEAN, AND EVERY CANDIDATE IN IT WAS CLEARED

* the URDF's two `initial_positions` blocks hold the solved pose and the sim
  spawns on it — the boot reading is the proof, and it matches the source file
  to 0.0000 rad;
* `config/home_positions_*.txt` is read by five runtime consumers and all five
  read the same file. **It is not the file "reported READ BY NOTHING" — that
  is `config/real_home_reference.txt`, and it is still read by nothing**, in
  `src/` or in `scripts/`;
* `recordings/baselines/presentation_pose.json` — the pose the sweep stages to
  — was already regenerated from the solved home on 2026-08-16 and matches it
  to 1e-4 rad. It was the obvious suspect and it is not the cause;
* `stage_observe_and_detect.py` returns the arm home and REFUSES if it cannot,
  and it reported 0.0000 rad;
* `ik_follower_node`'s startup unwind only wraps continuous joints into ±π; it
  cannot move the arm to another pose;
* `srl_moveit_config/config/initial_positions.yaml` is inert — the
  `initial_positions` key was removed on 2026-08-16 and the ros2_control macro
  that read it is no longer invoked.

## WHERE IT IS ACTUALLY LOST

**Between trials.** Nothing returns the arms to home when a run ends, and
nothing looked at where they were when the next one started.
`record_abc_sweep.py` stages before every clip, so the recorded set was never
affected — which is exactly why this survived. Every run driven from the GUI
or from `run_experiment.sh` starts wherever the previous task stopped, from
the second run of a session onward.

**And it is not only the working arm.** T1 is a left-arm task and it left the
RIGHT arm 0.6248 rad out, because the runner parks the idle arm at
`park(±PARK_X)` and never brings it back. "The task did not use that arm" is
not the same as "that arm is where it started".

## THE FIX, AND WHY IT IS IN THE RUNNER

`run_abc.require_home()` runs before the task commands anything. It is:

* **before `--isolate`**, and that order is not cosmetic — `isolate()` starts
  `vr_pose_mapper`, which holds the arms against a joint-space trajectory
  (measured 2026-08-15, with a control either side), so staging after it would
  lose to it;
* **idempotent** — an arm within 0.05 rad is left alone, no subprocess and no
  publisher, so the sweep pays nothing;
* **not its own staging move.** It shells out to
  `scripts/stage_presentation_pose.py`, which is this repository's one
  staging move: it pauses the followers by lowering `motion_enabled`, scales
  the duration from the distance, waits for arrival rather than sleeping,
  restores the followers on every exit path and refuses to touch an arm in
  real_robot mode. A second copy would drift from it;
* **a refusal, not a guess.** `--allow-unhomed` is the deliberate override and
  says so loudly. The measured start error goes into the run summary and the
  trial manifest either way, so a run that started off home is identifiable
  rather than assumed comparable — the same reason the sweep records
  `opened_on`.

Proved live: with the arms at 1.2345 / 0.6248 rad, the next run staged and its
first commanded waypoint read **0.0000 / 0.0000**.

## THE SECOND DEFECT, FOUND ON THE WAY, AND IT IS THE MORE INSTRUCTIVE ONE

`session_manager._js()` read `from srl_experiments import home_positions`.
**There is no such module.** The home source is
`config/home_positions_{arm}.txt` and it is loaded by path. So every
`/joint_states` callback raised ImportError, the bare `except` set
`home_err = None`, and the readiness panel reported "home reference not
loaded" for the life of the node.

It is the G-3 failure the panel exists to prevent, and it survived because it
never went green on a wrong answer. UNKNOWN is a legitimate reading for a gate
with no data yet; this gate had no data ever, and nothing distinguishes those
two states from the outside. **A gate that can only ever say UNKNOWN is worth
a test of its own**, and it now has one: a constructed joint state displaced
by a known amount must come back as that amount.

## WHAT IS STILL OPEN

The arms are still left off home at the END of a run. Fixing the start was the
smaller and safer change — a homing move after the task would run inside the
capture window of any clip that has not stopped recording yet — but it means
every second run pays a staging move it did not used to. If that becomes a
cost, the place to fix it is the end of `run_abc`, not the sweep.


# 2026-08-17 (later still) — T1 FROM A TYPED SENTENCE, AND WHAT THE SHIPPED GRAMMAR DID WITH ITS OWN TASK

T1 is "each cube goes on the plane of its own colour". Asked to say that in
English, the shipped parser could not — and one of the three phrasings failed
in the dangerous direction.

| utterance | shipped parser |
| --- | --- |
| "pick up the blue cube and put it on the blue pad" | `grab` / target `blue cube` / **destination None** — the place clause SILENTLY DROPPED |
| "put the green ones on the green mat" | refused, "no known verb" |
| "move that blue block to its colour" | refused, "heard 'move to' but no place I know" |

The first is the shape this repository's language sweep calls MISUNDERSTOOD:
the grammar matched a verb and a noun that were really there and discarded the
words that said what to do with them. Executing it grabs a cube and stops, and
the operator has heard an acknowledgement.

## WHAT WAS BUILT, AND THE THREE THINGS THAT HAD TO BE RIGHT

**One verb, `put_on`, not a second parser.** It sits above `place` and `goto`
in the pattern list because both would otherwise claim these sentences, and it
only fires when a destination can actually be EXTRACTED — so "put it down"
still reaches `place` and "move to the front centre" still reaches `goto` and
its measured refusal.

**1. The target must be read from the HEAD, not the sentence.**
`extract_target` takes the FIRST colour it finds. On "put it on the blue pad"
that is the DESTINATION's colour, and the arm would have gone looking for a
blue object nobody named. The place clause is split off first and the target is
extracted from what is left.

**2. The two-targets check had to be scoped to the head as well.** It counts
distinct colours and nouns and refuses on more than one. "pick up the blue cube
and put it on the GREEN pad" names one object and one place, and counting the
place's colour as a second target refused an ordinary instruction. Surfaces
(`pad`, `mat`, `plane`, `square`, …) are deliberately NOT added to `NOUNS` for
the same reason.

**3. Typo repair, and the mistake that made it interesting.** A token is
repaired only when it is not already vocabulary, is four characters or more,
and is one Damerau-Levenshtein edit from EXACTLY ONE candidate. Damerau rather
than Levenshtein because `bleu` → `blue` is a transposition, which is one edit
there and two under plain Levenshtein, and it is the commonest colour typo
there is. A TIE is left alone and named back to the operator: `gren` is one
edit from both `green` and `grey`, and breaking that tie would be a confident
answer to a question nobody asked.

**Verbs are never repaired.** That is this repository's own measured rule —
at distance 1, `top` is a STOP and `crop` is a DROP, and
`test_verbs_are_matched_EXACTLY_and_here_is_why` pins it. **The first attempt
implemented it by dropping verbs from the vocabulary entirely, and that broke
ten tests at once**: a CORRECTLY spelled `grab` is one edit from `gray`, so
with verbs no longer counted as words it was repaired into a colour. "Never
repair a verb" and "never touch a verb" are different rules and both are
needed. Two sets now: `_known()` (leave alone) and `_repair_targets()` (a
strict subset, may repair into).

## THE GROUNDING LAYER NEVER READS THE DECLARED COLOUR

`experiments/abc/t1_instruction.py` is the join between the sentence and the
detections and the only place they meet. `msc_clip_tasks.T1_PAIR` — the
declared colour of each cube — does not appear in it, checked behaviourally
and by a source assertion.

Live, against the mock wrist camera, with **every declaration in T1_PAIR
flipped**: "put every cube where it belongs" produced a byte-identical plan and
sent each cube to the pad of its RENDERED colour. The pre-existing mislabel
control on the direct path also holds — declared and vision paths place the
mislabelled cube 0.230 m apart, which is the pad separation.

## ASK IS AN OUTCOME, AND THE SINGULAR IS LOAD-BEARING

"put the blue cube on the blue pad" against **two** blue cubes must ASK. First,
nearest and leftmost are all defensible and all guesses, and this arm is bolted
to a person. "put the blue ONES on the blue pad" must not ask. The difference
lives in `Intent.quantity`, read off the words, not in a heuristic about how
many things happen to match — which is why the sweep's scene has two cubes of
each colour rather than one.

A destination that is NOT the cube's own colour ("put the blue ones on the
green pad") is HONOURED rather than quietly colour-matched. Overruling the
operator there would be the same class of error as dropping a negation.

## MEASURED, 40 PHRASINGS

`scripts/sweep_t1_instructions.py`, five harness controls that must pass before
anything is reported (including one that proves the scorer can SAY
misunderstood):

| | shipped | now |
| --- | --- | --- |
| CORRECT | 0 | **16** |
| ASKED | 2 | 5 |
| REFUSED | 38 | 19 |
| **MISUNDERSTOOD** | **0** | **0** |

"shipped" is the same 40 cases with `put_on` removed and nothing else changed,
so the columns differ by the feature and by nothing else.

The adversarial and misspelling blocks were written from the PARSER'S
STRUCTURE rather than alongside it, which is the lesson
`sweep_language_vision.py` already records: negation before and after the verb,
two targets, two actions, relational reference, stacking, a colour with no pad,
an object not in the scene, a question, a statement, a bare `stop`, an unknown
place, a misspelled VERB, a two-edit misspelling, and the `gren` tie.

**It measures GROUNDING, not detection.** The scene is arithmetic, so the
ground truth is constructed rather than rendered. Whether the camera can see
four cubes at working distance is a different measurement and remains
UNMEASURED on real hardware.

The 40-phrase `sweep_language_vision.py` reproduces its committed
10 / 6 / 24 / 0 exactly, which is the control that says the shipped grammar was
not disturbed.

---

# 2026-08-17 (later still) — THE INSTRUMENT WAS AT THE WRONG WRIST, AND THAT IS WHY THE OBJECTS FLOAT

## The fault, and how far it reaches

`measure_what_binds.Rig` set `self.quat` to the LIVE end-effector orientation
read off TF at construction — the HOME wrist. The tasks never command that:
`run_abc.send()` writes `master_calibration.WORKSPACE_ORIENT` into every
waypoint of every mode. Measured at the current home:

| arm | home tool-axis elevation | anchor | angle between |
| --- | --- | --- | --- |
| left | −1.47 deg | +30.77 | **32.26 deg** |
| right | −1.42 | +22.10 | 24.14 |

The left arm is the one T1 runs on. **Every reachability sweep in the repo
solves through `Rig`**, so every one of them measured a pose the robot is not
asked to reach. TASK_SPEC §9 records this fault for `search_centre_on_surface`,
which worked around it by passing its own anchor to `solve()` — but the
work-around read `as_tuple(rig.quat[arm])`, i.e. it *was* the home wrist, so the
work-around did not work. `solve_joints()` had no `quat` argument at all, so
`verify_t1_paths` — the instrument CLAUDE.md cites for T1 — could not have asked
for the right orientation even in principle.

`Rig(anchor="workspace")` is now the default; `anchor="home"` asks for the old
behaviour by name.

**What the fix did NOT overturn.** T1's shipped stage-1 layout re-verifies at
the anchor: 0 IK failures, 0 below the floor, clearance 0.1610, N=10, 171
waypoints, reproduced twice. The layout was right; the measurement was about the
wrong thing.

**What it did overturn.** `recordings/baselines/centre_on_surface.json`'s 22
hits, and with them the TASK_SPEC §9 lead "A SEARCH THAT MOVED THE TABLE INSTEAD
FOUND A CELL". Re-run at the anchor, all three controls correct: **0 of 3360
cells**.

## T1-1: the objects cannot rest on the surface, measured three ways

1. `search_centre_on_surface.py` — x 0.00–0.45, y 0.10–0.55, top 0.70–1.10,
   overhang 0 and 0.05, both arms, pinned AND top-down, objects RESTING:
   **0 of 3360** survive the full pick path.
2. `search_t1_layout_on_surface.py` — T1's own x = 0.560, tops 0.900–1.100,
   object 20–60 mm behind the edge: **every cell fails**, including a near edge
   at y = 0. It is not the edge; the slab occupies the volume the ARM needs.
3. `measure_objects_on_the_table.py` — objects held at 1.120, slab swept:
   0.950/1.000/1.020 cost **0 of 54**, 1.040 costs 12, 1.100 costs 38.

### And then (3) turned out to be measuring the wrong path

It walks T1's **six pick paths**. The path T1 sends is **171 waypoints** and
also contains the two pad placements, the transits and the standoffs — and a
surface can delete a placement while costing every pick nothing. Acting on
"1.020 costs 0 of 54" put `verify_t1_paths` at **36 of 171**.

`sweep_surface_vs_t1_path.py` walks the whole path, height and near edge
together, N=10, controls correct (no slab 0, slab through the objects 171,
shipped 0.950/0.100 zero):

| top \ near edge | 0.062 | 0.080 | 0.100 |
| --- | --- | --- | --- |
| 0.950 | 0 | 0 | 0 |
| 0.965 | 14 | 0 | 0 |
| 0.980 | 14 | 14 | **0** |

**Height and forward reach trade against each other.** A one-axis sweep of a
two-axis constraint reads the best cell off the wrong axis. The best free pair
is **0.980 / 0.100**, so the surface rises 30 mm and the gap goes 150 → 120 mm.
The cubes still do not rest on it.

### An instrument note that cost an hour

One `verify_t1_paths` run at 0.980/0.100 read **18 of 171** where the sweep read
0. It did not reproduce — two later runs read 0 — and the cause is that the run
overlapped the still-running sweep, which was applying and removing its own slab
in the same planning scene. HARD CONSTRAINT 3 is about two stacks; this is the
same hazard one level down: **two clients mutating one planning scene**. Do not
run a sweep and a verifier against one `move_group` at the same time.

## Two surface constants, 150 mm apart, and nothing compared them

`clip_tasks.BENCH_TOP` was the literal `1.10`; `clip_scene.TABLE_TOP` was the
literal `0.950`. Every cube, pad and marking tile is positioned against the
first; the only surface with geometry is drawn against the second. The raised
bench that closed the gap was deleted as scenery ("THE BENCH IS GONE. ONE
TABLE.") and the objects were left in the air. It survived a code read because
it was *written down*: `clip_scene.py` carried the comment "it is 170 mm below
the work plane and nothing rests on it".

Both now read `srl_experiments.work_surface`, which owns `WORK_PLANE_M` (1.100,
where the objects are), `DECLARED_M` (0.980, the drawn surface),
`DECLARED_NEAR_Y` (0.100) and `FLOAT_GAP_M` (0.120).
`test_one_work_surface_height.py` fails if either consumer goes back to its own
copy, and pins the gap **in both directions** — a smaller gap fails too, because
every number behind it was measured against this pair.

A first attempt collapsed the two into ONE owner. That is wrong in the other
direction: it makes the objects rest on the table by definition and the render
disagrees. They are two facts and the gap between them is a third.

## The check that should have caught it did not exist

`scripts/verify_objects_on_table.py`. Pure geometry over the same two functions
the scene draws from, so it runs offline with no stack, no render and no IK: each
object registered to the work plane, each footprint inside a solid, the marking
painted on the surface, and the one scene-level gap equal to the documented
value. **9 known answers**, including two DRIFT cases (85 mm and 70 mm against a
documented 80) because a tolerance that excuses any gap binds nothing. It reports
`PASS WITH A KNOWN BLOCK` and states in words that T1-1 is not satisfied — the
`by_design` argument applied to a geometric impossibility.

A first version compared every object's base to the surface and called the pads
DRIFT for being 76 mm up instead of 80. They are 4 mm thick and drawn TOP-flush
to the work plane on purpose. That is registration, not drift, and the gap is one
scene fact rather than a per-object one.

## The marking, and the render check that could not fail

The marking was **192 filled 21 mm tiles** at 30% alpha plus a bounding-box
outline, drawn at `BENCH_TOP` so it floated with everything else, with 20 cells
entirely off the table. The comment above the loop still claimed "four thin bars
rather than a filled patch" — true two rewrites earlier.

It is now `region_outline()`: the **same set**, an edge drawn wherever a cell
adjoins a non-cell, so concave stays concave, collinear runs merged. **20 bars**
instead of 196 markers. The bounding box is gone — it was the only real
simplification, asserting a third of the right arm's box that was never
measured. It is painted on the surface and clipped to it; **40 of 192 cells are
dropped** for having no surface under them and the count is printed, not
absorbed.

**Removing the bounding box left the label reading its corners.** `tick()` raised
`NameError` on its first frame, so `clip_scene` came up with no table, no cubes,
no pads and no marking — and all three views were filed **OK at 18% ink**,
because the WEARER is 18% of the frame. With the scene publishing, the same views
read 46–50%. An ink fraction answers "is anything drawn", not "is the scene
drawn". `render_t1_scene.py` now SUBSCRIBES to `/task_objects` and refuses to
shoot unless the task's own namespaces are present. The negative control is that
real failure: `tick()` raised before the `planes` and `item` blocks.

## T1S2 seed 2 has 16 IK failures and always did

Stage 2, seed 2, LEFT arm: **16 of 91** at N=10. Measured at the raised surface
AND at the old 0.950 — **identically 16** — so it is the anchor fix, not the
surface. Seeds 0 and 1 are clean on both arms, and the recording sweep runs
`--seed 0`, so no recorded clip is affected. CLAUDE.md's "0 IK failures … stage 1
AND all three stage-2 seeds" was measured at the home wrist and is corrected.

## A run's home pose, at the moment it matters

`require_home()` measures and stages BEFORE the run. That is not the same claim
as "the run started from home": between it and the first published waypoint the
scene node comes up, the trial manifest is written, and for mode 06
`Vision.stage()` drives the arm to an observe pose. A clip recorded at 09:30 —
two hours AFTER `require_home()` landed at 07:24 — shows arms that are visibly
not at home, and the pre-run check had passed.

`run_abc` now measures the home error **at the first commanded waypoint**,
records it in the trial summary as `first_wp_home_err_rad` /
`first_wp_at_home`, and prints one greppable line. Note also that the clip's
`scene_events.json` carried `opened_on: null`, so it recorded no evidence of the
pose it opened on at all.

## The arms are NOT at home at the first commanded waypoint — measured, 2026-08-17

The instrumentation above found it on its first run. Recording T1 under
`01_master_teleop`, everything upstream reporting success:

* `stage_presentation_pose.py` succeeded → the clip records
  `opened_on: "presentation"`;
* the staged look reported `home to 0.0000 rad`;
* `run_abc.require_home()` returned homed and the run exited 0;
* `verify_rviz_clips` passed 37 of 37 clips.

And at the first commanded waypoint:

    left 0.7955 rad from home, right 0.6249 rad, tol 0.05  ->  NOT AT HOME

**So the operator's reading of the frame was right and the clip was not stale.**
The clip at 09:30 postdates `require_home()` (07:24) by two hours, and the arms
in it really are off home — every check between the two was measuring something
adjacent to the question. `opened_on` records whether STAGING SUCCEEDED, which
it did; it says nothing about where the arms were when the task first commanded
anything.

Note also that the right arm's 0.6249 matches the 0.6248 recorded for "run 2
began off home" to a tenth of a milliradian, and 0.6249 is about what
`park(0.32)` — where T1's idle right arm is held — sits from home. So at least
the right arm's number looks like the PARK pose rather than drift.

**The leading hypothesis, not yet confirmed.** `require_home()` stages through
`stage_presentation_pose.py`, which pauses the followers for its duration. When
that subprocess exits and `ik_follower_node` resumes, it streams toward the
target it still holds — which is wherever the arm was before staging. Staging
would then be undone between `require_home()` returning True and the loop's
first publish, with nothing in between measuring. The instrument to settle it
already exists (`scripts/verify_follower_pause.py`) and the fix, if this is
right, is to re-target or re-pause the followers across that boundary rather
than to move the check.

**The sweep now FAILS a clip that is not at home at its first waypoint**, so
this cannot be recorded silently again. It is why no re-recorded T1 set was
committed this session: the mode 01 clip was recorded, failed this check, and
the previously committed clip was restored rather than replaced with one whose
arms are 0.80 rad off home.

---

# 2026-08-17 (later still, 4) — THE MARKING WAS BELOW THE WORK IT BOUNDS

## My own regression, and the argument that caused it

The previous session moved the workspace marking from the work plane DOWN to the
table, on the argument that "paint goes on the thing it is painted on". That is
a good argument about paint and the wrong argument about this marking. It is the
boundary a participant is told to keep the **work** inside, and drawing it under
the work put the whole task outside its own boundary. Measured, both arms, both
tasks: marking **0.9815**, pad top **1.1000** — the boundary sat **116.5 mm**
below the thing it encloses. The frame showed it plainly.

It is back at the work plane and derived **per arm** through
`clip_scene.work_top_for(arm, task)`, from that arm's own pads rather than from a
shared constant. Both arms come out equal today; they are equal *because they are
measured from the same thing* rather than by coincidence. The label follows the
marking — a label floating 116 mm under the boundary it names is the same defect.
Now: marking 1.1015 against a pad top of 1.1000, 1.5 mm proud of the plane it
encloses, both arms, T1 and T1S2.

**A number I could not reproduce, said so rather than rounded to.** The report
was "left 0.945 against a pad at 0.987, right the same" — a 42 mm gap. The code
has ONE marking height and ONE pad height and they measured 0.9815 and 1.0980, so
the gap is 116.5 mm and identical for both arms. The only 42 in the geometry is
`PAD_OFFSET_BY_ARM["right"][2]` = 0.0421. The defect was real regardless.

## The check's SUBJECT was wrong for T1S2

`verify_objects_on_table.objects_for` read

    cubes = MCT.T1_CUBES if task == "t1" else MCT.T1_CUBES

— a ternary with the same answer on both sides. So asking it about T1S2 measured
stage **one's** four cubes and stage one's single-arm pad pair. Stage 2 draws over
BOTH arms from its own seed and has a pad pair per arm; none of it had ever been
looked at. One task credited with another task's geometry, in a file written the
same day as the `status_table` fix for the same fault. It now reads stage 2's real
layout and follows the seed the sweep records.

## The check can fail on the real scene, and can pass

"It reports BLOCKED today" is not evidence that it can report anything else.
`--inject-float-mm` displaces every object and the marking with them, so one
instrument answers three ways about one scene:

| displacement | gap | verdict | exit |
| --- | --- | --- | --- |
| 0 mm | 120.0 mm | BLOCKED | 3 |
| +5 mm | 125.0 mm | DRIFT | 1 |
| −120 mm | 0.0 mm | **PASS** | 0 |

The last row is the one that matters: if the scene were ever fixed, this check
would notice. Pinned for t1, t1s2 and t3 in
`test_objects_on_table_can_fail.py`.

Two ordering bugs in my own control, both caught by running it: the work plane
was shifted *after* the objects were checked, so +5 mm reported BAD_GEOMETRY
instead of DRIFT and −120 mm reported BAD_GEOMETRY instead of PASS.

## Support geometry, re-priced at the anchor: FIVE TIMES WORSE

`SUPPORTS_ENABLED`'s note invited a re-measurement, and it needed one — every
number in it was solved through `Rig` at the HOME wrist. With the lips really in
the planning scene, `verify_t1_paths --part path --repeats 10` at the anchor over
the full 171-waypoint path:

    cube lips + plane lips, 75% of depth     109 of 171 IK failures
    no supports                                0 of 171

The old table called the same geometry 22. The conclusion is unchanged and the
margin is five times larger: a shelf whose top is flush with the object's base is
exactly where the fingers close, and the cube lips are 20 mm wider than the cube
in x — 10 mm each side, which is the pads.

**So T1-1 is now blocked by four independent anchor measurements**, all listed at
`SUPPORTS_ENABLED`: 0 of 3360 resting cells; every (top, edge) cell failing for
T1's own x including an edge at y = 0; best free pair 0.980/0.100 leaving 120 mm;
and supports at 109 of 171. The objects cannot rest on the table, and the
verifier says so at exit 3 instead of calling it a pass.

## T1S2 re-recorded, all five modes

AT HOME 0.0000 rad on both arms at the pre-approach instant in every cell, 4 of 4
closures at 0.0000 m in every cell, 37 of 37 clips passing `verify_rviz_clips`.
One mode aborted first time: the virtual display would not render after three
restarts and `record_rviz` REFUSED rather than filing a black view — two orphaned
`rviz2` processes were holding the display. The per-mode cleanup now kills
viewers as well as displays.

## The centre of the table, all five levers priced — 2026-08-18

**The question.** Can the two coloured pads sit in the CENTRE of the table,
directly in front of the wearer, with BOTH arms working there — full pick path,
N=10, wearer and table in the planning scene, the 150 mm floor held
geometrically? "Centre" is `|x| <= 0.10`: two 100 mm pads straddling the
centreline put each inner slot at `|x| = 0.025`.

**Instruments.** `scripts/search_centre_geometry.py` (the scorer is
`measure_centre_gap.classify_cell`, unchanged), `scripts/sweep_mount_geometry.py`
(translation and rotation of the mounts by the target-transform trick),
`scripts/probe_centre_limit.py` (best-of-K branches, the upper bound),
`scripts/apply_mount_candidate.py` (the same change put into the URDF for real),
`scripts/verify_centre_layout.py` (the layout point by point).

### What each lever bought, as built

| lever | innermost `\|x\|`, L / R | what blocks the centre |
| --- | --- | --- |
| as built | 0.350 / 0.275 | `bracelet -> end_effector` **20 mm inside the torso** |
| (a) wearer stands back, near edge 0.380 | 0.250 / 0.250 | FLOOR, and the body changes: **the wearer's own forearm**, not the torso |
| (a) near edge >= 0.480 | none | IK — forward reach is spent |
| (b) table width 0.15 … 1.05 | 0.225 … 0.400 / 0.225 … 0.375 | **null.** The verdict is never TABLE; the scatter is the solver |
| (c) rotated, wearer at the short end | 0.275 / 0.300 | **null**, and it is (b) at a narrow width by construction |
| (d) mount, 32 candidates | nothing better than 0.325 / 0.275 | FLOOR at every candidate |
| table height 0.95 … 1.25 | best 0.225 / 0.225 at 1.15 | FLOOR |

**Levers (b) and (c) are dead for a reason, not for a number.** In 18 table
geometries the centre column never once returned TABLE. The slab is under the
object at `x = 0` at every width and every rotation, so no table geometry can
move the thing that is actually in the way.

**Lever (a) changes the mechanism and that is what makes the rest work.** At the
as-built distance the gripper is inside the torso. Past 0.330 m the torso is
clear and the binding body is the wearer's own upper arm against the robot's
`forearm_link -> spherical_wrist_1_link`. The brief's premise — "deleting the
wearer's arms changes nothing, it is the torso" — is true at 0.280 and false at
0.380, and the two answers were being read as one.

### What the combination bought, and the certification

Single levers all leave the centre 110–150 mm inside the floor. **Crossed, two of
them clear it**: the mounts moved 150 mm OUTBOARD and yawed 15 deg outboard, over
a table at 1.25 with its near edge at 0.430, approached at 65 deg inboard.

    best-of-40-branch clearance at |x| <= 0.10   L +0.220 / R +0.196  (floor 0.150)
    certified, N=10, worst branch, full path     innermost |x| = 0.225 (L) / 0.100 (R)

The gap between those two rows is the null-space branch scatter, not geometry:
the pose is admissible and TRAC-IK will not reliably find it. `t1_task` already
records the same risk under `independent` seeding.

**Two things that looked promising and are not.** Heading 70–75 deg reads best of
all on the geometric probe and returns COLLISION:TABLE at every column on the
relaunched stack — the probe's slab test stops at `bracelet_link` and cannot see
the gripper, exactly as `sweep_gap_vs_mount` warns. And making the two mounts
EXACT MIRRORS made the LEFT arm **worse** (0.225 -> 0.350), which is the
repository's own note holding up: two identical arms on mirrored mounts do not
mirror.

### Where the pads ended up

`T1_PLANES = [[0.320, PAD_Y], [-0.165, PAD_Y]]`, every slot and every cube
REACHABLE at N=10 over the full pick path with 0.157–0.220 m of clearance
(`recordings/baselines/centre_layout.json`). Off the centreline: **320 mm (blue,
left) and 165 mm (green, right)**, against 530 and 350 before. The pair is
centred 78 mm left of the centreline and its inner edges are 385 mm apart.

**It is not the centre.** `|x| <= 0.10` is met by the RIGHT arm and missed by the
LEFT by 125 mm. The left arm has been the worse of the two in every certified run
in this session and the mount asymmetry is not the cause.

### What it costs, unmeasured

The mount change moves the anchor, so `WORKSPACE_ORIENT`, `PAD_OFFSET_BY_ARM`,
`P_HOME` and every T0–T3 coordinate derived through `ee_for()` are re-derivations
— TASK_SPEC section 2A lists them. Home clearance itself IMPROVES, 0.1610 ->
0.2202 m, because the mount is no longer the closest thing to the wearer.
`test_t1_layout_is_verified` fails on all three of its assertions: it compares the
shipped constants against `t1_paths.json`, which records the walk of the OLD
layout on the OLD mount. Regenerating it means walking the whole composed 203
waypoint path on the new geometry, which was outside this brief and has NOT been
done.

---

# 2026-08-18 (later) — T1 COMMANDABLE FROM A TYPED SENTENCE, AND THE GRASP THAT HAD BEEN 13 mm SHORT SINCE IT WAS WRITTEN

## The order things were found in, because it matters

The brief was language, the GUI and a re-record. None of it could be recorded
until the layout verified, and walking the layout is what found the two faults
below. Both had been live for as long as the code they sat in.

## 1. `PAD_MID_EE` was 13.47 mm too long, and it had never been measured

`grasp_frames.PAD_MID_EE` is the vector every T1 grasp is built on: the task
declares where the OBJECT is and subtracts this to get the wrist pose. It was
DERIVED — as the magnitude of `clip_tasks.PAD_OFFSET_BY_ARM`, a world-frame
vector recorded at the anchor — and `test_pad_offset_has_one_source` asserted
the two agreed to 0.1 mm, which they did, because one was arithmetic on the
other. A test that derives its expectation from the thing it is checking
cannot fail.

`/compute_fk` on the two finger-tip links and the end-effector link, from one
solution, on both arms:

| | |
| --- | --- |
| end_effector_link -> midpoint of the finger tips | **(0, 0, 0.09833) m** |
| the two arms agree to | **0.000 mm** |
| `\|PAD_OFFSET_BY_ARM\|` | 0.11178 / 0.11184 m |
| disagreement | **13.45 / 13.51 mm** |

The symptom was there to be read the whole time: `verify_t1.py` reported a pad
miss of **13.48 mm on all four cubes of both arms, identically**, which is the
signature of a constant and not of a path. With the measured value it is
**0.00 to 0.01 mm**.

`scripts/measure_pad_mid_ee.py` is the instrument, with a round-trip control;
`recordings/baselines/pad_mid_ee.json` the record. `PAD_OFFSET_BY_ARM` is
deliberately NOT moved: T0, T2 and T3 declare their coordinates through it and
`clip_scene` draws their objects through it, so task and picture agree with
each other. What it means is that in those three tasks the declared coordinate
is 13.45 mm from where the object is drawn and grasped — written down now
instead of unknown.

**Correcting it broke the approach, which is the useful part.** At the shipped
heading of -65 deg the gripper base fouls the table by 5.2 mm and **0 of 40**
grasp attempts solve. Swept at N=10 per cell over the four cubes, elevation -10
solves **40 of 40 from heading -45 to -57.5 and 0 of 40 outside it**. The
heading is now -50: the middle of that window, and the only one that also
solves at elevation -7.5, so it has margin in both directions rather than
merely working.

## 2. The right pad's outer slot was in the wearer's own upper arm

Walked three times at N=10, sequential seeding, the same waypoint read

    0.1611 m     0.1787 m     0.1483 m

against a 0.150 m floor — **one run in three inside it, with 0 IK failures
every time.** That is the null-space branch scatter this file already records
for the left arm, landing on a column with no margin to absorb it.

`scripts/measure_pad_columns.py` sweeps the place pose per column at N=10 and
names the closest body:

| \|x\| | left | right |
| --- | --- | --- |
| 0.120 | 0.1624 (L-upperarm) | 0.1398 (R-upperarm) |
| 0.195 | 0.2202 (torso) | 0.1902 (R-upperarm) |
| 0.230 | 0.2202 | 0.2035 (R-upperarm) |
| 0.255 and out | 0.2202 | 0.2202 (torso) |

0.2202 m is the MOUNT — `base_link`, which no joint moves — so there is a band
where the closest thing to the person is the mount and a bad branch cannot make
it worse. The left arm is mount-limited from 0.225 and the right from 0.255.

**The pads are now symmetric at ±0.290**, both slots of both pads inside that
band, and the cubes moved to ±0.420 and ±0.480 because a pad occupies
\|x\| = 0.210 to 0.370 and a cube at 0.300 would start ON the pad it is meant
to be delivered to.

Walked three times sequential and once independent — the pessimistic branch
case T1 has never survived before — all clean: 0 IK failures, 0 breaches, worst
clearance 0.1920 (left) and 0.2148 (right).

## 3. NEITHER ARM CROSSES THE CENTRELINE, and it decides what stage 2 can be

Measured at N=10 with the wearer and the table in the scene: **0 of 10 IK
solutions at every cross-side pad slot and every cross-side cube position, on
both arms.** Not marginal — nothing solves at all.

So the arm that reaches a cube is fixed by the side it starts on, the arm that
reaches a pad is fixed by the pad's side, and a cube can only be delivered to
the pad on its own side. Stage 2 draws the SIDE; the colour follows from it.
Drawing colour and side independently would be drawing trials the rig cannot
perform, and that is why the old stage 2 needed a pad of each colour on each
side and could not share stage 1's two.

`t1_task.build()` REFUSES such a cube by name rather than emitting a path whose
every waypoint fails IK for a reason nobody would connect to colour. The
refusal is exercised by `test_a_cube_whose_colour_is_on_the_other_side_is_REFUSED`
and by an audit probe.

## 4. Stage 2 is stage 1's geometry now, and eight seeds were walked to say so

Same table, same two pads, same row, same approach, same builder —
`t1_task.build()` with a drawn layout instead of a fixed one, so stage 2 cannot
acquire a dwell, a standoff or a slot spacing stage 1 does not have. Eight
seeds over the composed path at N=10: **0 IK failures, 0 floor breaches, every
one clean** (`recordings/baselines/t1_stage2_paths.json`). Splits seen: 1/3,
2/2, 3/1.

**The pads are 290 mm off centre and not 260 because of a stage 2 seed.** At a
260 mm centre the innermost of three slots lands at \|x\| = 0.200, which the
per-column sweep calls clear FROM HOME and the composed path does not: at seed
3, three cubes left, the left arm read **0.1219 m to the wearer's own upper arm
on 15 waypoints**. Solving from home and solving from the carry are different
questions and the second is the one the follower asks.

## 5. THE ONE MISUNDERSTANDING: a compound instruction executed half of itself

"put the blue ones on the blue pad and the green ones on the green pad" —
two complete instructions sharing a verb — **planned the first clause and
dropped the second in silence: two cubes moved of four, reported as a
success.** Three of the 75 phrasings did it, all compounds.

The mechanism: `split_destination` cuts at the FIRST destination cue, so the
head was "put the blue ones", and the two-targets check counts targets in the
head — one target, passed. Every part behaved as designed.

`voice_intent.split_clauses` now names the clauses; `t1_instruction` plans each
and merges them, refusing when a clause does not stand on its own or when the
two halves send the same cube to two pads.

**Measured, 75 phrasings, against the grammar as committed at b782a9b** — the
same cases, checked out of git into a temporary tree and run in their own
interpreter, so the two columns differ by the code and by nothing else:

| | at b782a9b | now |
| --- | --- | --- |
| CORRECT | 21 | **34** |
| ASKED | 14 | 14 |
| REFUSED | 37 | 27 |
| **MISUNDERSTOOD** | **3** | **0** |

and every one of the 14 asks resolves to the right plan on one reply (one takes
two, because it is a two-part question). Those are reported on their own line
and NOT counted as correct: a grammar that asks about everything must not be
able to look like one that understands everything.

**THE BEFORE COLUMN CAME BACK IDENTICAL THE FIRST TIME IT WAS RUN**, and that
is worth recording because it is indistinguishable from "the change did
nothing": the temporary tree was appended to `sys.path` after the real one, so
`import t1_instruction` found the current file. The runner now asserts the
module it imported came from the tree it wrote.

## 6. The other thing the phrase set found: politeness became a second action

"could you please just put the blue ones on the blue pad for me when you get a
chance" was REFUSED as "more than one action in one instruction". `when` is one
Damerau edit from `then`, the repair rule took it, and `then` is a SEQUENCE
word. A word that decides whether a sentence is refused is as load-bearing as
the verb, so negations, sequence words, conjunctions and relational words are
no longer repair TARGETS — the same rule that already stopped `top` becoming
`stop`. They stay in the vocabulary, so a correctly spelled `then` still
refuses.

A related tie: `padd` is one edit from both `pad` and `pads`, and asking "did
you mean pad or pads?" is asking about a distinction the sentence does not
have. Ties are collapsed by LEMMA before being counted, so `green`/`grey` stays
a tie and `pad`/`pads` does not.

## 7. What "universally commandable" turned out to need

Each of these exists because a phrasing needed it, not because it was designed:

* **selectors**, resolved against what the CAMERA saw — "the leftmost blue
  cube", "the second", "either". A selector the scene cannot settle ("the
  biggest", against four identical 40 mm cubes) is REFUSED BY NAME rather than
  dropped, because dropping it turns a qualified instruction into an
  unqualified one;
* **a bare pick is a whole instruction** — "pick up the blue cube" is planned
  as a pick and a lift, not silently completed with a destination nobody named;
* **"closest to me" is a superlative about the person**, not a relational
  reference to a second object, and is no longer refused as one;
* **"tidy up" offers its reading back** as a question rather than assuming it;
* **an ASK is answerable** — it carries the candidates it was asked about, and
  a reply as short as "the left one", "both" or "yes" resolves it against the
  set the operator was SHOWN. A reply that is itself a complete instruction is
  treated as one.

## 8. The audit that TASK_SPEC section 8 requires had been dead since the rebuild

`scripts/audit_task_spec.py` calls `msc_clip_tasks.t1()`. The rebuild moved the
builder to `t1_task` and the alias went with it, so the pre-recording pass —
the thing that exists to catch "reported done and later found absent" — died
with `AttributeError` the first time it was run afterwards. `check_t1` was also
still written for the one-armed T1: `T1_ARM`, `t1_grip`, `SLOT_DY`, and a
cube-to-pad pairing keyed on index parity.

Rewritten against the task as it is. **46 PRESENT, 0 MISSING, 0 BLOCKED**, and
T1-1 — "cubes rest ON the table" — is satisfied from the shipped scene for the
first time since it was written. TASK_SPEC's T1-9 said "stage 1 runs on the
LEFT arm with all four cubes on the LEFT"; the document has been changed rather
than the check bent to fit it, and the old sentence is left in place above the
correction.

## 9. The GUI prompt panel, and the check it has to pass

A tab in the existing window: text box, voice button on the existing
`/voice_transcript` path, what the parser understood shown BEFORE anything
moves, what the camera detected with the classifier's own HSV margin per cube,
the announced intention and a confirm click, live state from the runner's own
`[progress]` lines, and refusals and questions in plain words — answerable in
the same box.

The failure it is written against is "a prompt box that looks right and sends
nothing". `scripts/verify_prompt_panel.py` drives the real GUI offscreen and
intercepts exactly one thing, the `Popen`: everything before it runs for real.
The check that matters is that CONFIRM hands the runner an argv carrying this
task, mode 06, the detections it was planned from, and the operator's own
sentence verbatim.

`verify_gui_buttons` presses every button in the window, so the panel's two
process launches go through one interceptable method each — pressing LOOK for
real would move the arm in the middle of a button audit.

## 2026-08-18, later: two of T1's four cubes stayed on the table, and the run said exit 0

The first mode-06 T1 re-record after the layout was fixed **picked two cubes
and left two**, and every gate the run had reported success. The clip failed
only because the cube that was missed happened to be `items[0]`, which is the
only item `record_abc_sweep`'s placement check ever read.

`clip_scene` records closest approach whether or not it becomes a grasp, and
that is what separated the two failures — they are **not the same defect**:

| cube | detected off truth | pads got to | knuckle there | fingers closed at |
| --- | --- | --- | --- | --- |
| cube_0 (left, 0.420) | 10.3 mm | **11.0 mm** | **0.0007 — OPEN** | 48.8 mm away |
| cube_1 (left, 0.480) | 2.8 mm | 0.0 | 0.4055 | on the cube |
| cube_2 (right, −0.420) | 21.4 mm | **42.3 mm** | 0.201 | never |
| cube_3 (right, −0.480) | 8.1 mm | 0.0 | 0.3989 | on the cube |

So cube_0 was **reached and lost to timing** — the pads were 11 mm from it
with the hand still open, and the fingers did not read closed until the arm
had moved 48.8 mm on. cube_2 was **never reached**: it stopped 42.3 mm short
of a 100 mm standoff, 58% of the way down its own descent.

**The same run, launched by hand with nothing recording, picked all four.**
That is the whole finding. The dwell at the pick is fourteen waypoints, tuned
without the recording load; under eight ffmpeg captures and RViz the follower
lags past it. A schedule whose result depends on what else is running on the
box is not measuring the task, and it had been passing only because the lag
had been smaller than the dwell.

### What was actually wrong

The grip COMMAND was already gated on arrival — that was fixed twice before,
and the comment block in `run_abc` records both. What was never gated is **the
schedule itself**. `steps = hold_s / 0.05` is a fixed number of ticks per
waypoint; the arm's lag is not fixed. So the close was commanded correctly and
then the schedule walked away while the fingers were still moving.

Two different quantities were being confused, and they fail differently:

* **arrival** — the pads are not on the object yet. Waiting fixes it if the
  arm is slow, and cannot fix it if the pose is unreachable.
* **the grip** — the change was commanded and the FINGERS have not done it.
  `clip_scene` decides a grasp on the knuckle, so this is the quantity that
  decides whether the cube moves at all.

A waypoint that is waiting for either now holds until it gets it, bounded by
`WP_SETTLE_S` (8 s per waypoint) and `RUN_SETTLE_BUDGET_S` (90 s per run),
and **says so out loud** when it gives up. An object that has been given up on
is not paid for again — the pick dwell is 14 waypoints at one pose, so without
that one unreachable cube would cost 112 s and switch the waiting off for
every cube after it.

The settle test is `gripper_state.holding()` **imported, not reimplemented**.
The defect being fixed is two instruments disagreeing about whether the hand
was closed; a local copy of the 0.90 fraction would have been the third copy
of a constant this repo has already had to unify once.
`test_grip_waits_for_the_fingers.py` sweeps every knuckle from 0.000 to 0.800
and asserts the run and the scene agree at all of them, with the two measured
values above as its known answers.

### And the check that could not fail

`record_abc_sweep` compared `items[0]["final"]` against `place_target`. For a
one-object task that is the task; for T1's four cubes it is a quarter of it,
and **the other three could not fail**. Had cube_1 rather than cube_0 been the
one missed, this clip would have been filed as good. It now names every object
that was never carried, and every object still in the hand at the end.
Re-run against the rejected clip on disk it answers
`NEVER MOVED: ['cube_0', 'cube_2']`.

---

# 2026-08-19: THE VR PATH OVER WIFI, AND WHAT RUNNING IT FOR THE FIRST TIME FOUND

The question that started it was narrow: does the VR path need `adb`? A
borrowed lab headset showed `unauthorized` and the in-headset authorisation
prompt never appeared, which on a device you may not reconfigure is terminal
-- Developer Mode needs the owner's Meta account.

**The answer is that it depends which route, and the doc named only one.**
`vr_bringup.md` listed `adb` as prerequisite 1 and called it "the ONE hard
blocker", which is true of the route it documents (a sideloaded Unity client
reaching `ws://127.0.0.1:8766` through `adb reverse`) and not true of the
repo's own WebXR fallback, which `wsl.md` had recorded and then set aside. So
`adb` blocked ONE route and was listed as blocking VR.

## 1. THE WEBXR BRIDGE HAD NEVER BEEN STARTED. NOT ONCE.

    self.clients = set()        # quest_bridge_node.__init__

`rclpy.node.Node.clients` is a read-only PROPERTY -- the node's service
clients. Assigning it raises `AttributeError` during construction, so
`quest_bridge_node` has never come up since it was written on 2026-08-07. It
was carried in the tree, in the launch file, and in two documents as "the
fallback for a headset without USB access", and the failure is on the first
statement of `__init__`: a single attempt to run it would have found it.

This is the "feature present but does nothing" row of the instrument-failure
table, one level further out than usual. Nothing checked that the node was
STORED correctly, let alone that it RAN.

## 2. AND ITS AXIS MAP WAS FOR A DIFFERENT FRAME

    quest_to_ros_p(p) = [-z, -x, y]        # as shipped

That lands in the ROS convention -- x forward, y left, z up. **This repo's
world frame is x = the wearer's RIGHT, y = FORWARD, z = UP**, which
`vr_bringup.md` section 6 states explicitly and which
`quest_vendor_bridge.quest_to_world_*` implements for the Unity route.

It matters because `vr_pose_mapper` adds the controller displacement STRAIGHT
onto the robot's world-frame pose:

    p_cmd = p_anchor + scale * (p_controller - p_ref)

The docstring above that line promises an `R_align`; there is none in the
code. So the controller frame IS the world frame by assumption, and a
mismatch is a rotation applied to every command the operator gives.

Measured through the real node, real socket, real topic:

| the hand does | as shipped, the arm was commanded | after the fix |
| --- | --- | --- |
| 3 m forward, 1 m right, 2 m up | **3 m RIGHT, 1 m BACKWARD**, 2 m up | 1 m right, 3 m forward, 2 m up |

The two transports genuinely need different conversions and must not share
one: **WebXR is right-handed** (+z toward the viewer), **Unity is
left-handed** (+z forward). The Unity map carries its handedness flip in a
negated `w`; the WebXR map is a pure -90 deg rotation about x with det = +1
and `w` must be left alone. Copying one into the other inverts every rotation.
`test_webxr_frames.py` pins both, including one test whose only job is to fail
if someone "unifies" them. Its first two tests fail against the shipped
mapping -- checked by putting the shipped mapping back.

## 3. ONE ORIGIN. THIS IS THE PART THAT WOULD HAVE COST A LAB DAY.

The 2026-08-07 decision to drop WebXR listed the HTTPS cost correctly: a
self-signed cert with the IP in `subjectAltName`, a firewall rule, and a
warning accepted inside a headset. All of that is still real. What it did not
say is the trap underneath:

**a certificate exception is granted per ORIGIN, and a WebSocket cannot
prompt for one.** Serve the page on 8443 and the socket on 8765 and the
operator accepts one warning, gets a page that looks completely healthy, and a
socket that fails with no reason shown anywhere. From inside a headset that is
indistinguishable from a bridge that is down.

`quest_bridge_node` now serves the client page from the SAME port and the same
TLS context as the socket, via `websockets`' `process_request`. One warning,
one exception, both work. (`websockets`' `Headers` is a multidict whose
`__setitem__` APPENDS, so the response headers must be deleted before being
set; two `Content-Length` values drop the connection.)

## 4. PROVE THE NETWORK BEFORE THE CERTIFICATE

`scripts/vr_reachability.py` serves a page over **plain HTTP**, deliberately.
Lab wifi commonly has AP client isolation -- both clients reach the internet,
neither reaches the other -- and from inside a headset that failure looks
exactly like a certificate problem, so the natural reaction is to go and fight
the cert. If the plain page loads, policy allows it and every later failure is
TLS or firewall. If it does not, no certificate will help.

It also runs a WebSocket echo on a second port: some networks pass HTTP
through a proxy that eats the `Upgrade`, and the VR path is a socket, so HTTP
alone is not proof. Both hits print in the terminal, so nothing must be read
inside the headset. `--selftest` probes a live port and a dead one and
requires different answers.

## 5. THE MAPPER RESETS NOW, IN THE NODE

`NEXT_SESSION.md` item 1 -- `vr_pose_mapper` does not reset between runs, which
cost 02_vr_teleop every grasping task at 88/115/156/206 mm and GROWING -- is
fixed where the state is:

* `reset()` clears references, anchors, `filt`, `last_cmd`, the jump history
  and **`scale`**, which the thumbstick moves live and which is per-run state
  as much as anything else;
* a `/vr/reset` Trigger service and a `/vr/reset_request` topic;
* `mode_upstreams.isolate()` calls it EVERY run -- not only when it did not
  just start the process -- and REFUSES the mode if it does not answer, because
  "we think we just started it" is exactly the assumption that produced the
  session-lifetime bug;
* **the accumulating term is named and published.** It is `filt`: the EMA is
  rate-limited to `max_speed_mps`, so whenever the command moves faster than
  that it falls behind and cannot catch up while the motion continues. That is
  a LAG, not an offset, which is precisely why the four cube misses grew by
  27, 41 and 50 mm instead of repeating one number. It is now `lag_m` on
  `/vr/mapper_<hand>` with a warning past 30 mm -- the capture gate.

Also fixed while in there: **tracking loss now drops the clutch** instead of
only pausing publication. Staying engaged across a dropout means resuming from
a filter latched before the loss against a controller that has since moved,
and the clutch's "zero jump by construction" guarantee is void for exactly the
motion nobody saw. Dropping it costs a re-grip, which `_on_joy` retries
automatically while the grip is still held.

All eight tests in `test_vr_mapper_resets_between_runs.py` fail against the
node as it was.

## 6. AN ANALYSER THAT WAS WRONG, CAUGHT BY ITS OWN SELF-TEST

`vr_headset_check.py` scores the gripper by correlating the trigger against
the commanded angle. Its self-test fed it a known-GOOD latching trace and the
analyser returned r = 0.033 and called the gripper broken.

The gripper was fine. A latching gripper deliberately HOLDS the firmest value
seen while the trigger falls away, so a correlation over the whole trace is
near zero for a perfectly working one. The statistic was wrong, not the
signal. It now correlates only over the pre-latch region, where proportional
control is actually in force, and reads 0.989 on the same trace.

That is the standing rule doing its job on this session's own instrument,
before it was ever pointed at a headset.

## 7. What is measured and what is still not

Verified end to end, no headset needed: `scripts/verify_vr_wifi_route.py`,
seven checks, plus two negative controls (`--break-cert` serves a certificate
for an address this host does not hold; `--break-frame` restores the shipped
axis map through an external launcher, so the production node carries no
"break me" switch). The script exits non-zero if either negative control
passes.

Still unmeasured, and only a physical headset can close them -- pose rate,
end-to-end latency, freeze-on-real-dropout, re-engage jump, live scaling,
gripper, overlay legibility. `scripts/vr_headset_check.py` is the instrument
and it is guided; its analysers are validated against constructed truth and
against deliberately broken traces. **An item it reports as NOT MEASURED is
not a pass**, and it says so.

---

# 2026-08-19 (later): THE SETUP IS NOT VR. IT IS MOTION CAPTURE.

The operator sits across the room **facing the wearer**, holds the two
controllers as a 6-DOF input device, and watches the real robot with their own
eyes. **Nobody wears the headset** -- it stands on a shelf as the tracking
reference for the controllers. Every head-worn assumption in `vr_teleop.md`
was wrong about who is where, and two of them were load-bearing.

## 1. THE ROUTE DECISION, ON FACTS RATHER THAN PREFERENCE

`SteamVR / OpenVR over Quest Link` was evaluated properly and rejected:

| | checked on this machine |
| --- | --- |
| Meta Quest Link PC app | **not installed** |
| SteamVR | **not installed** (Steam is; SteamVR is not) |
| OpenXR runtime | **none registered** |
| Quest on USB | **not attached** |

Those are all installable. The one that is not is the account: **Link pairs
the headset to the PC app through a Meta account**, and the headset is on its
owner's. Air Link additionally needs enabling in the headset's own Settings →
Beta, which is a settings change on borrowed hardware. It is the adb blocker
again wearing a different hat.

It also buys **no rate**: OpenVR poses are bounded below by the headset's frame
period exactly as WebXR's are, and WebXR is already delivering **89.9 Hz**.

**What it would carry over is worth writing down**, because it makes the port
cheap if the lab ever buys its own headset: `vr_pose_mapper`,
`vr_gripper_node`, `vr_safety_node`, the clutch, the scaling and the reset all
consume `/vr/controller_pose_<hand>`, `/vr/controller_joy_<hand>` and
`/vr/controller_valid_<hand>`. **Only the source node changes.** OpenVR even
shares WebXR's handedness (+x right, +y up, −z forward), so
`webxr_to_world_*` would apply unchanged. The topic contract is the seam, and
it held.

## 2. R_ALIGN WAS PROMISED FOR MONTHS AND NEVER EXISTED

`vr_pose_mapper`'s docstring has always said

    p_cmd = p_anchor + scale * R_align * (p_controller - p_ref)

and the code added the displacement **raw**. So the controller frame *was* the
world frame by assumption -- true only for an operator standing behind the
wearer facing the same way, which is not how the rig is driven.

**It is a YAW, and it can never be a mirror.** "They're facing me, so mirror
it" is the intuitive fix and it is a reflection, det = −1. Applied to position
alone it looks right; applied to the pose it mirrors every **orientation**, so
the gripper rolls the wrong way while the positions look correct. That is the
hardest class of bug to see in a video, and it is why the alignment is exposed
as **one angle** rather than per-axis sign flips: no value of `align_yaw_deg`
can produce a reflection, pinned over ±720°. The yaw reaches the orientation
too, conjugated as `qz · dq · qz⁻¹` -- rotating position and not orientation
puts them in two frames and IK is asked for a pose that does not exist.

**Verified through the whole real chain** -- wss client → `quest_bridge_node`
→ `vr_pose_mapper` → `/master_arm_pose_right` -- five cases, every one
**0.0 deg off**:

| yaw | hand moves | commanded |
| --- | --- | --- |
| 0 | forward (WebXR −z) | world +y |
| 0 | right (WebXR +x) | world +x |
| 180 | forward | world −y |
| 180 | right | world −x |
| 90 | right | world +y |

**Calibrated, not guessed.** `scripts/calibrate_operator_yaw.py` has the
operator move ~40 cm straight towards the robot -- a direction both people in
the room can identify -- and one known direction fixes one unknown angle. A
second motion is a cross-check, not an input: after the solved yaw the two
must still be perpendicular and horizontal, and if they are not the script
**refuses to write a number**. It also refuses motions under 100 mm or more
than 35 deg off horizontal.

### An ambiguity this uncovered and did NOT settle

The docs say world **+x is the wearer's right**. The arms sit the other way
round: `left_end_effector_link` at x = +0.117, `right_end_effector_link` at
x = −0.160, and T1 places "the green pad at x = −0.290 on the RIGHT arm".
Either +x is the wearer's *left*, or the arm named `right_` is physically on
the wearer's left. Those need different fixes and only one is an axis-map
change. **Still open.** It is why the heading is measured rather than argued.

## 3. TWO SAFETY GAPS THAT DESK OPERATION TURNS FROM RARE INTO ROUTINE

### 3.1 Controller tracking loss froze nothing

`/vr/tracking_ok` is derived from `last_rx`, stamped on **every frame
arrival**, at line 285 -- *before* the per-controller validity check at line
296. So it caught a network dropout, a sleeping headset and a backgrounded app
and **not the case it is named after**. Measured with the real Quest: covering
a controller for 5 s produced **zero** tracking-loss transitions while the
stream ran on at 90 Hz, and the mapper kept commanding from the last pose.

Head-worn this is nearly harmless, because the operator's hands are in the
headset's cameras. Off-head, with the headset on a shelf, occlusion is
routine. `/vr/controller_valid_<hand>` now exists, the mapper freezes that arm
and **refuses to engage it** while it is occluded -- latching a reference off
an emulated pose would anchor the whole run to a position the runtime invented.

### 3.2 Nothing consumed `/vr/freeze`

`vr_safety_node` computed freezes and published them and **no node listened**.
So "the observer e-stop was withdrawn" and "the tracking reference moved" were
both states the system could be in while still commanding the arm. The mapper
honours it now, drops the clutch, and refuses to re-engage while it stands.
It caught its own author immediately: the first run of the chain test came
back "clutch did not engage" and the log said
`engage REFUSED: the safety node is holding a freeze`, because the observer
e-stop publisher had been killed in a restart.

## 4. THE TRACKING REFERENCE IS A PHYSICAL OBJECT, AND A NUDGE IS SILENT

This is the failure mode desk operation *introduces*. Every other failure in
this system announces itself -- a dropout stops the frames, an occlusion
invalidates a pose, a dead link stalls the rate. A knocked headset leaves the
poses valid, the rate at 90 Hz and the controllers tracked, and silently
rotates the frame every subsequent pose is expressed in.

`vr_safety_node` latches the first HMD pose and freezes past **20 mm or
2 deg**. The rotation threshold is the one that matters: a 10 deg twist barely
moves the headset and puts every command 10 deg wrong. The freeze **survives
the ordinary unfreeze path**, because poses flow the whole time it is wrong --
the generic "poses are fine again, observer present" condition is true
throughout. Clearing it is deliberate (`/vr/rebase_reference`) and the service
tells you to re-run the yaw calibration, because the heading you measured
belonged to the old reference.

## 5. THE MAPPER WAS REPORTING FOLLOWER LAG AS A CLUTCH JUMP

`mean_reengage_jump_m` was `|last_cmd − pr|`. At engage the command is set to
`pr`, the arm's **own** pose from TF, so the jump the ARM makes is zero by
construction and there is nothing to measure there. `last_cmd − pr` is a
different quantity: how far the **follower** had fallen behind its last
target. A real run reported **150 mm of "re-engage jump" across an engage on
which the arm moved zero**. Published under that name it would have gone into
the write-up as evidence the clutch is bad. Renamed to
`max_follower_lag_at_engage_m`, with `reengage_jump_m: 0.0` and the reasoning
in `_engage()`.

## 6. THREE HARNESSES THAT RECORDED EXACTLY ZERO, AND WHY

Three successive measurement scripts came back with 0 commands and 0 TF
samples on a system that was working. The cause was the same each time and it
was mine: `tf2_ros.TransformListener(buffer, node)` joins the node's default
**mutually-exclusive** callback group, so a lookup inside a timer callback
blocks *every other callback on that node* -- including the subscriptions.
`spin_thread=True` gives the listener its own executor and it recorded 2146
samples immediately. Worth knowing before the next analysis script is written.

## 7. AND A RESULT THAT WAS 0.000 EVERYWHERE, WHICH IS NOT A RESULT

With TF finally recording, every clutch quantity read **0.000 mm** -- which is
the first row of the instrument table. The data said why: the commands were
moving correctly (123 mm in world +y, exactly 0.5 scale × 0.25 m of hand) and
the **end effector never moved at all**, one unique row in 2146 samples. The
follower log had it: `[BLOCKED:ik_failed]`, **47/64 IK solves succeeded
(73%)** -- the synthetic drive had pushed the target into a region where IK
fails, so the arm sat at the first commanded pose.

That makes the clutch numbers **degenerate**, not good: 0.000 mm because
nothing moved, which cannot distinguish a sound clutch from a frozen arm.
**The re-engage jump is therefore still NOT measured on this setup**, and it
needs a drive that stays inside the reachable set. The mapping, the rate and
the reset are measured; that one is not, and saying so is the point.

---

## 2026-08-20 — the first real-arm VR session, and four bugs in our own tooling

**The VR path worked.** A Quest in the operator's hands drove a real Kinova
Gen3 over wifi, in passthrough, with the gripper live. Almost none of the day
was spent on that path; it was spent on discovery partitions that we caused.

### 1. The launch swept the ros2 daemon's own shared memory

`run_teleop.sh` line 12 calls `srl_clear_stale_shm`, which removed
`/dev/shm/fastrtps_*` unconditionally. The `ros2 daemon` is a participant and
its segments are in there. It does not die when they vanish — **it goes
deaf**. `ros2 node list` then returns nothing against a fully healthy stack,
and `start_real.sh` reports `DISCOVERY PROBLEM, not a missing stack`: true,
specific, confident, and pointing at the wrong thing, because the sweep two
seconds earlier caused it.

Measured: removed 14 segments, and every subsequent preflight failed. Four
full teardown-and-restart cycles were spent before the sweep was suspected —
the message was so specific that it read as evidence about the network.

The sweeper now stops the daemon, sweeps, and restarts it. Its live-process
guard also named only three sim nodes, so the VR stack, the real-arm bridge
and `observer_estop.py` were all invisible to it: a sweep "with the stack
stopped" could still be a sweep under a live observer e-stop. All four are
now counted.

**A guard that names processes by an explicit list will be wrong the moment a
new process exists. This one was wrong for the observer e-stop.**

### 2. `timeout N ros2 …` sends SIGTERM

A hard-killed DDS participant holding shared memory poisons discovery for
everything that joins afterwards. `env.sh` and `start_real.sh` had ten such
call sites, including `ros2 topic hz`, which **never exits on its own and is
therefore always killed**. All now `timeout -s INT`, so rclpy releases its
segments on the way out.

### 3. `require_observer_estop` did not gate the thing it is named after

`vr_safety_node` tested `self.observer_ok` directly in the freeze/unfreeze
path, with no parameter. The parameter gated only `_srv_enable`. Setting it
false therefore disabled the real-arm request and left the clutch held shut
anyway.

In the lab this presented as a **broken clutch**: grip pressed, gripper moved,
arm dead, and the only explanation in a log on a machine the operator could
not see while wearing the headset. `_observer_required()` /
`_observer_satisfied()` now carry it. `_srv_enable` is unchanged and still
refuses unconditionally.

### 4. Two arms cannot share the Kortex high-level path over WSL

Measured on the same box in the same session:

```
one arm    21.0 Hz sustained,  send latency 12-33 ms
two arms    8.1-12.2 Hz,       send latency SPIKING TO 519 ms
```

519 ms is past the bridge's 0.5 s watchdog, so it zeroes the speed. The joints
stop **while still being commanded**, and `real_homing_node` reports `HOMING
BELOW VELOCITY FLOOR` — a true statement about a symptom two layers down, and
one that sends you to the homing gains instead of the network. Both arms
froze simultaneously, which is the giveaway.

This is HARD CONSTRAINT 4 appearing on the high-level path, not just the
cyclic one. `KORTEX_RATE` defaults to 12 Hz per arm for `arm:=both`.
**Untested — it is a hypothesis derived from the latency, not a result.**

### 5. The wearer clearance check was measuring almost nothing

`real_homing_node.measure_clearance()` iterated `("torso", "head", "hips")`
against a model holding **twelve** primitives — it never checked the neck, the
thighs, or the wearer's own upper arms, forearms and hands, which CLAUDE.md
records as the geometry that binds the right arm inboard.

It also swept `DISTAL_LINKS`, which starts at the forearm. The shoulder and
both half-arm tubes — the segments a shoulder mount actually swings across a
person's head and chest — were never measured. **The hand can be a metre clear
while the upper tube is inside somebody's neck, and the check reports the
hand.** This is the SRDF trap of hard constraint 11 reproduced one layer down,
in the node whose whole job is that check.

Both fixed; a part the model has but TF cannot resolve is now NAMED in a
warning rather than skipped in silence. **Not re-measured** — every clearance
figure from a real-arm run before this date was taken with the proximal half
of the arm invisible.

### 6. And the one that is not a code bug

Every real-arm run so far, including this one, used `SRL_WEARER_PRESENT=0`,
which deletes the wearer from the model entirely and prints `clear inf m` on
every homing tick. That is correct for a bench-mounted rig with an empty
harness and wrong the instant anybody stands in the arms' volume. It was left
set after the situation changed. **No real-arm run has ever happened with a
wearer in the model.**

---

# 2026-08-23 — the teleoperation motion generator, and five instrument bugs on the way to it

Instrument: `scripts/measure_teleop_motion.py`.
Module: `srl_teleop/motion_generator.py` (pure, self-tested, no ROS).
Baseline: `recordings/baselines/teleop_motion.json`.
Tests: `src/srl_teleop/test/test_motion_generator.py` (36).

## 1. THE DIAGNOSIS, REPRODUCED BEFORE IT WAS ACTED ON

`ik_follower_node.clamp_towards()` was the entire motion generation for
teleoperation. It takes the IK solution and walks toward it, clamping **each
joint independently** to `max_step_rad` per control cycle. The previous
session recorded the consequence and this one reproduced it to the decimal
place before changing anything, which is the point: a baseline you cannot
reproduce is not a baseline.

Case: the left arm's shipped home + `[0.50 -0.30 0.10 0.25 -0.05 0.15 0.02]`,
50 Hz, `max_step_rad` 0.35.

```
same-cycle EE deviation from a synchronised move:  max 135.3 mm, mean 45.1 mm
per-joint arrival spread:                          1 cycle of 2
```

Three separable faults, and only the first was named before:

| | |
| --- | --- |
| **not synchronised** | a joint needing 0.10 rad arrives in one cycle, one needing 0.50 takes two, so the joints sit at different fractions of their own travel at every instant and the hand leaves the straight line the IK solution implies |
| **not continuous** | from rest the first cycle commands a whole `max_step`. Its implied acceleration step is **1343x** the jerk-limited one |
| **it never read the joint limits** | `max_step_rad` / dt is 17.5 rad/s against the 1.3963 and 1.2218 rad/s in `src/srl_moveit_config/config/joint_limits.yaml` — **12.53x**, and the limits DIFFER per joint, which a single `max_step` cannot express |

## 2. THE METRIC IN THE DIAGNOSIS MIXES TWO THINGS, AND IT MATTERS

"Where the hand is at cycle k against where it would be if every joint were
at fraction k/N" conflates *leaving the path* with *not being at k/N along
it*. A jerk-limited generator is DELIBERATELY not at k/N — that is what a
velocity profile is — so scoring one on that metric charges it for the
feature. Ruckig reads 69.8 mm on it, and the number means nothing.

So the reported figure is **off-path**: the distance from each commanded hand
position to the CURVE the straight joint-space line traces,
parameterisation-free. Both are in the baseline file; only off-path is
comparable.

| left arm, 50 Hz | off-path max | same-cycle max | peak joint speed | arrives |
| --- | --- | --- | --- | --- |
| `clamp_towards` (as shipped) | **51.3 mm** | 135.3 mm | **12.53x limit** | 2 cycles |
| **ruckig (now the default)** | **0.0 mm** | 69.8 mm | **1.00x** | 1 cycle |
| synchronised clamp (fallback) | 0.0 mm | 22.7 mm | 1.00x | 1 cycle |

51.3 mm of unrequested Cartesian path on a rig whose grasp capture gate is
30 mm. Right arm: 39.2 mm → 0.0 mm.

**And it is not a tuning value.** Swept against its own step size, the
excursion does not tend to zero — it settles:

```
max_step 0.35 rad ->  2 cycles,  51.3 mm off the path
max_step 0.20 rad ->  3 cycles,  34.4 mm
max_step 0.10 rad ->  5 cycles,  68.0 mm
max_step 0.01 rad -> 50 cycles,  68.0 mm
```

Each row is a genuinely different path (densifying at 0.02 rad changes none
of these figures, which was checked rather than assumed). The
desynchronisation is proportional, so it survives any step size. It is the
shape of the algorithm.

## 3. WHAT WAS BUILT

[Ruckig](https://github.com/pantor/ruckig) — time-optimal, jerk-limited,
per-DoF limits, MIT, 19.8 us mean for 7 DoF, and it takes a non-zero TARGET
velocity, which is what a streaming teleop target actually is
([paper](https://arxiv.org/pdf/2105.04830)). Installed into the SYSTEM user
site, deliberately, because the followers run in the system interpreter:

```
pip install --user --no-deps --break-system-packages ruckig
```

It has **no dependencies at all** — one compiled `.so` — so it cannot repeat
the `.venv_vision` numpy incident. `python3 -m srl_teleop.dependency_check`
read 0 problems before and after, and
`scripts/real_calibration/check_all.py` still reads 4 of 4.

`srl_teleop/motion_generator.py` is pure: no ROS, no robot, no display, like
`orientation_policy` and `joint_planner`. `ik_follower_node` and
`measure_teleop_motion.py` consult the same module, so the measurement
describes the code the robot runs.

**`Synchronization.Phase`, not `Time`.** Both make every joint ARRIVE
together; only phase synchronisation keeps them on the straight line BETWEEN
the endpoints, and that line is the whole point. Measured on this case:
phase 0.02 mm of path deviation, time 9.5 mm, clamp 51.3 mm.

**The velocity limits are the robot's own; the acceleration and jerk limits
are ASSUMED and say so.** `joint_limits.yaml` declares
`has_acceleration_limits: false` and `max_acceleration: 0` for all fourteen
joints, so there is nothing to read. They are derived from two named ramps —
"reach the velocity limit in 0.25 s", "reach that acceleration in 0.10 s" —
and `Limits.provenance` reads `ASSUMED` for as long as that is true. A test
writes a yaml that DOES declare one and requires the label to follow the
file, so the honesty is a fact rather than a hardcoded string. The assumption
is bounded: acceleration and jerk shape HOW the velocity limit is approached
and can never exceed it.

**The fallback is synchronised too, and it names itself.** Without ruckig the
generator does not quietly become `clamp_towards` again: `SynchronisedClamp`
scales the WHOLE joint vector by one factor, so the path stays on the line
and every joint still arrives together, with the per-joint velocity limits
enforced by that same factor. It is not jerk-limited and it says so —
`backend` reads `synchronised-clamp`, `/ik_status_<arm>[15]` reads 1 rather
than 2, and the GUI's STATUS tab shows it in amber.

## 4. FIVE THINGS THAT WERE WRONG IN MY OWN WORK, EACH CAUGHT BY A CHECK

Recorded because four of the five would have passed a test suite.

### 4a. Re-planning every cycle destroys phase synchronisation

The first version called Ruckig's `calculate()` every control cycle. That
re-plans from a mid-profile state, and a re-plan cannot reproduce the
phase-synchronised profile it is halfway through: the joints' fractions of
travel spread to 0.07 at cycle 10 and re-converged, leaving the commanded
path **0.0264 rad** off the straight line. Riding one trajectory through
`update()` gives **8.9e-17 rad**. Every other check passed either way;
the self-test's "is the path the straight line" check is the only thing that
saw it.

### 4b. And what forced the re-plan was floating-point noise

Even after switching to `update()` it still re-planned on all 36 cycles.
`update()` recomputes only when its input changed — and the target was
recomputed each cycle as `pos + wrap_pi(target - pos)`, which is not
bit-identical to `target`. The goal moved by ~1e-17 every cycle, Ruckig
compares by value, and every cycle looked like a new target. The fold is now
cached while the incoming target is unchanged.

### 4c. The fallback claimed to be jerk-limited by reporting zeros

`SynchronisedClamp` returned an acceleration of `[0.0] * 7` because it does
not model one. The continuity check reads that field, so the fallback passed
it — by fabricating the quantity being checked. It reports its real implied
acceleration now (**62.5x** the jerk-limited step), and a test requires it to
be over 10x: this backend must never be able to claim smoothness.

### 4d. The off-path metric had a 0.05 mm floor and reported it as behaviour

The reference curve was 4001 points over a 0.4 m path — a 0.1 mm grid — so
the nearest-sample distance bottomed out at half a grid step and the
phase-synchronised generator, which is on the line to 2e-16 rad, measured
0.0509 mm off it. The instrument's resolution read as the robot's behaviour.
The nearest sample is now refined by bisection on the real FK and the same
path reads 9.4e-09 mm.

### 4e. The arrival-cycle test used an absolute tolerance on a per-joint quantity

A phase-synchronised move reported arrivals of `[36, 36, 35, 36, 35, 36, 35]`
under a 1e-6 rad gate: every joint was at the same FRACTION throughout, but
the ones moving 0.02 rad were inside 1e-6 of the end a cycle before the one
moving 0.50 rad. The travels span 25x, so an absolute gate measures the
instrument's scale. It is relative to each joint's own travel now, and this
does NOT loosen the control — the per-joint clamp lands on its targets
exactly, so its arrivals are unchanged by any tolerance.

## 5. `CONTINUOUS_IDX` HAD SIX DEFINITIONS

Found by the "one source" test written for this change.
`motion_generator`, `ik_follower_node`, `sim_to_real_bridge`,
`sim_to_real_gap`, `real_homing_node`, `mock_real_stack` and
`kortex_highlevel_bridge` each declared `CONTINUOUS_IDX = (0, 2, 4, 6)`.

They agreed, which is luck rather than design. "Which joints are
`type="continuous"` and can therefore wind up" is a fact about the URDF, and
seven copies of it is the same shape as two home poses: the day one is edited
the others describe a different robot. One source now, in
`motion_generator`, with a test that walks the package's ASTs and requires
exactly one assignment.

## 6. WHAT THIS DOES NOT CLAIM

* **Nothing here ran on a real arm.** Every figure is offline FK against the
  URDF, through `srl_fk`, whose own control (compiled FK against the URDF
  walker) is run before any measurement is reported.
* **The acceleration and jerk limits have never been measured.** They are
  assumed, labelled, and parameterised as ramp times.
* **The `time_from_start` change is untested on hardware.** With the
  generator the published point is one cycle ahead and is timed as one cycle,
  because telling the controller to take longer than a cycle makes it
  interpolate part of the way before the next point replaces it. The legacy
  path's `travel / max_vel` stretch is kept for `motion_generator:=legacy`.
* **The arming ramp changed shape.** It used to stretch `time_from_start`;
  under the generator it LOWERS the velocity limit instead, which also makes
  the ramp jerk-limited. Same intent, different mechanism, unmeasured on an
  arm.

---

# 2026-08-23 (later) — the accuracy pass: a constant that was a curve, and six verifiers asking the wrong question

Asked to improve the accuracy as far as it would go and then check that
everything works. The error budget in `scripts/measure_control_budget.py` was
the map, and every term it named turned out to be worth chasing.

    RSS of the measured terms   18.64 mm  ->  12.91 mm
    WORST CASE (they add)       33.39 mm  ->  19.99 mm   against a 30 mm gate

The worst case is inside the capture gate for the first time.

## 1. THE PAD MIDPOINT IS NOT A CONSTANT. IT IS A FUNCTION OF THE OPENING

**The Robotiq 85 is a four-bar linkage: its fingers SWING, they do not
translate.** So the distance from the wrist to the midpoint of the finger tips
depends on how open the hand is. Measured from the URDF's own mimic chain by
`scripts/measure_pad_mid_ee.py --by-width`, offline:

| the hand | knuckle | tip span | pad from wrist |
| --- | --- | --- | --- |
| wide open | 0.0000 | 135.5 mm | **0.09833 m** |
| on a 40 mm cube | 0.4235 | 93.3 mm | **0.10976 m** |
| on a 20 mm object | 0.6118 | 72.2 mm | **0.11179 m** |

**Both numbers this repository has argued about are points on that curve.**
`clip_tasks.PAD_OFFSET_BY_ARM` had magnitude 0.11178 — the hand almost shut.
`grasp_frames.PAD_MID_EE` is 0.09833 — the hand wide open. The 2026-08-18 note
calls the first "13.47 mm too long" against the second and treats the
difference as an error in one of them. Neither was wrong. They are two gripper
states, and the quantity was compared as though it did not have one.

**T1's grasp has been built on the open-hand value ever since, so the wrist
sits 11.43 mm too close and the fingers close past the cube's centre.**

### And it explains why the evidence flipped

`verify_t1.py` reads the finger tips out of `/compute_fk`, which uses the
simulation's CURRENT gripper joints — whatever the last thing to touch the
hand left them at. On 2026-08-18 that was an open hand and the pad miss read
0.00 mm, which is the measurement quoted as the evidence for the constant. On
2026-08-23, same geometry, same code, it read **13.52 mm**, because the hand
was nearly shut. A measurement whose answer depends on leftover state.

Both sides are fixed. `grasp_frames.pad_mid_ee_for(width_mm)` is the table;
`t1_task.ee_for` builds the grasp at the opening its 40 mm cube needs; and
`verify_t1` reads the tips through `srl_fk` at a STATED opening instead of
through the live stack. T1's pad miss at zero tilt is now 0.0000 mm by
construction, and the measured miss went **13.52 → 9.58 mm** — what is left is
the tilt, below.

### The instrument could not do it before

`scripts/srl_fk.py` set every gripper joint to 0.0 and ignored the Robotiq's
`mimic` tags entirely, so the finger tips — the only part of the robot that
touches anything — could not be placed anywhere except one arbitrary opening.
It walks the mimic chain now and takes a `gripper=` angle.

## 2. THE DECLARED PAD OFFSET, 13.45 mm, AND WHY IT WAS INVISIBLE

`clip_tasks.PAD_OFFSET_BY_ARM` is DERIVED from the measurement now rather than
hand-recorded, so T0, T2 and T3 put their pads on the coordinate they declare:
**13.45 mm → 0.05 mm**, which is the 0.1 mm grid `ee_for` rounds to.

It was invisible because it cancelled: `clip_scene` DRAWS each object at
`ee_for(obj) + the same offset`, so the picture was right whatever the
constant was. What did not cancel is where the fingers went.

Re-verified because moving three tasks' coordinates is exactly the kind of
change that is fine until it is not — `verify_msc_tasks`, N=10, both arms,
each task's own furniture, every clip waypoint the recorder drives:
**6024 IK calls, 0 failures**.

## 3. THE SHELL BIAS WAS FIXED IN THE SCENE READER AND NOT IN THE PICK PATH

`table_scene` has corrected it since 2026-08-21. `grasp_pipeline.plan_grasp`
— the function `find_object.py` and the recording path actually plan grasps
with — never had it: 10.5 mm in the budget, and two readers of the same scene
disagreeing. One `shell_corrected_centre` now, in `rgbd_grasp`, used by both.

Without a support height the plan SAYS the centre is the shell's rather than
reporting a biased number as a measurement, and an object whose top is below
the surface it is said to rest on is REFUSED rather than corrected into
nonsense. The test records the assumption the correction really rests on: the
TOP has to be observed, and with the top third unseen the centre reads 10 mm
low.

**T1 uses its plane instead of only gating on it.** `vision_grasp` had already
established that an accepted detection is a cube RESTING on the work plane;
its centre is then at `T1_Z` exactly, while the deprojected z is that same
quantity through a depth pixel, an intrinsic, a TF lookup and a half-a-cube
ray correction the function itself calls approximate. The snap runs AFTER the
gate — a cube off the plane is still rejected and named — and x and y are
untouched.

## 4. SIX VERIFIERS WERE ASKING A QUESTION THE ROBOT DOES NOT ASK

This is the "check that everything works" half, and almost everything that
looked broken was the instrument.

### 4a. `verify_msc_tasks.py` had not run since T1 became two-armed

It read `msc_clip_tasks.T1_ARM`, which was deleted when T1 moved to both arms
on 2026-08-16, and has **raised AttributeError on import ever since**. It did
not verify T1 wrongly; it did not run at all, and nothing said so. It is the
second time this file has held its own stale copy of T1's layout. T1 is
verified by `verify_t1.py` now and this file REFUSES BY NAME if a T1 layout is
reintroduced.

### 4b. The verifiers solve at the HOME wrist, not the anchor the tasks command

`run_abc.send()` writes `master_calibration.WORKSPACE_ORIENT` into every
waypoint. Around twenty scripts read `Solver.ee_quat()` — the live home wrist
— and call it the anchor. Measured on 2026-08-23 against the shipped home:

    left  42.94 deg apart      right  27.29 deg apart

CLAUDE.md already records this fault and it was fixed in
`measure_what_binds.Rig` on 2026-08-17 **and nowhere else**. What it costs,
measured with A/B/C's own furniture applied: task A's own pick solves **10 of
10 at the anchor and 0 of 10 at the home wrist**. `verify_msc_tasks`'s
"known-good pick" control was therefore failing on a task that runs — the
first thing it said when it was finally able to run at all.

`verify_task_scenes.task_anchor()` is one source for it now, and
`anchor_gap_deg()` prints the gap in the verifier's own output so the choice
is visible rather than implicit.

### 4c. The verifiers ask for EXACT while the follower has run a 15 deg cone since 2026-08-22

`ik_follower_node` defaults to `orientation_policy:=cone`,
`orientation_cone_deg:=15.0` in every mode. The verifiers asked `/compute_ik`
for the commanded orientation only, so a waypoint the robot solves by tilting
two degrees read as unreachable. `docs/NEXT_SESSION_2026_08_23.md` asked for
exactly this change.

| | asking EXACT | asking what the robot asks |
| --- | --- | --- |
| T1 stage 1 | 32 IK failures per arm | **0** |
| T1 stage 2, seed 3 | 48 left / 16 right | **0 / 0** |
| dance d1 | 64 of 382 | **0** |
| dance d2 | 135 of 356 | 62 |
| dance d3 | 13 of 560 | 6 |

The commanded orientation is candidate 0, so nothing that solved before stops
solving; `--orientation-policy exact` reproduces every earlier figure.

### 4d. `normalise()` returns RADIANS while its two siblings take DEGREES

`describe(*normalise("cone", 15.0))` prints "within a 0 deg cone" — 0.2618 rad
formatted as degrees. `verify_t1` printed exactly that while correctly solving
inside a 15 deg cone, and `verify_dance_paths` did worse: it passed the radian
value on to `candidates()`, so its first "cone" run was a **0.26 deg** cone
and rescued almost nothing. The value was right in one place and wrong in the
other, and the sentence was wrong in both.

### 4e. `verify_t1 --stage2 N` overwrote the STAGE 1 baseline

Same output file for both stages, so a stage-2 run silently replaced the
record `test_t1_layout_is_verified` reads — and did, turning three green tests
red with a result from a different stage. One file per stage now.

## 5. WHAT IS STILL OPEN, WITH NUMBERS

* **T1's exact grasp pose is refused by T1's own table.** It solves 10 of 10
  with no furniture and needs a 5.0 deg tilt with the table in the scene. The
  follower's cone supplies it, and the tilt costs `2·|pad|·sin(tilt/2)` =
  8.58 mm, measured 9.58 mm — inside the 30 mm capture gate, and not zero.
  `verify_t1` now bounds the two claims separately: 2 mm at zero tilt, because
  there the pads land on the cube BY CONSTRUCTION, and the capture gate when a
  tilt was needed. Loosening the first to cover the second would be a
  tolerance that binds nothing.
* **The dance routines d2 and d3 have 68 unreachable waypoints** between them
  at the anchor with the 15 deg cone (down from 171 at the home wrist). d1 is
  clean. `verify_dance_paths` still refuses to certify them for filming.
* **camera_link against the physical module is UNMEASURED** and is now the
  largest unknown in the budget. There is no CAD of the Kinova gripper in this
  repository — only the master arm's — so it needs the hardware.
* **The terminal joint error stays UNCOMPENSATED at 7.24 mm.** The model says
  1.88 mm with `terminal_overshoot` on, and it stays off: it and the
  `deadband_deg` 1.0 → 0.10 change are two corrections for one error and
  neither has been tried on hardware.
