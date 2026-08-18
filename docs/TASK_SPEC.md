# TASK_SPEC.md — the single source of truth

Written 2026-08-13 from the session brief. **If anything in this repository
contradicts this document, this document wins and the contradiction gets
fixed.** Record the contradiction and its fix in `docs/system/findings.md`.

Status column meanings used throughout: `MUST BE TRUE` items are the
acceptance criteria. They are checked twice — once **in the code, before any
recording**, and once **in the pixels, after**. A check passing is necessary
and has never been sufficient.

---

## 1. THE SYSTEM

Two Kinova Gen3 arms on a backpack worn by one person, operated remotely by a
second person. Four tasks, five control modes.

The task layer is **mode-independent by design**: the same waypoints are
COMMANDED under every mode. `run_abc` builds them from the task spec with no
mode argument, and that is enforced in code and tested.

**It does not follow that the robot performs identically, and it does not.**
This document said so until 2026-08-15, when the scripted clips — no operator
anywhere in the loop — measured the achieved motion per mode and found T0's
left-arm path at 1.407 m under 01 against 2.034 m under 03, and T3's circuit
box carried 21 mm under 01 against 161 mm under 06, against a same-mode
run-to-run spread of 3%. The command paths differ (follower, assist node, VR
mapper) and they track with different lag, so what the arm ACHIEVES differs.

The consequence for the study is the part that matters: **a difference
between modes in a trial cannot be attributed to the operator until the
no-operator difference has been subtracted.** It has never been measured;
`docs/system/findings.md`, 2026-08-15, is the first measurement and it is
five tasks deep, not a characterisation.

**SETTLED 2026-08-15 (later): the difference is REAL, and its SIZE is still
unknown.** The two ways it could have been an artefact were tested against the
recorded set (`scripts/analyse_mode_difference.py`):

* **not recording length** — runs of IDENTICAL duration differ in path length:
  27% between 01 and 03 at 15.83 s on T2, 34% between 03 and 04 at 19.83 s on
  T0, 6.4% between 03 and 06 at 20.25 s on T3;
* **not run-to-run noise** — on T1S2 four modes share a net displacement of
  (0.1901, 0.1374) **exactly**, and two modes do the same on T2 and on T3.
  Independent noise does not agree to four decimal places.

What the archived set **cannot** settle is the magnitude, for two reasons that
are properties of the instrument rather than of the robot: every cell is a
single run, and both metrics depend on the observation window —
`ee_travel_m` sums |dp| over recorder TICKS, and `ee_net_m` is anchored on the
first pose the scene node happened to see, not on the pose the task started
from. The sample count is now written into `scene_events.json` beside them,
with that caveat, so the next comparison can be normalised instead of assumed
comparable. The magnitude needs N >= 5 repeats per cell and a metric that
starts when the TASK starts; `docs/research/02_baseline_and_hypotheses.md`
section 5.0 carries the requirement.

### Five modes. The naming is 01, 02, 03, 04, 06. There is no 05.

| key | name | input | assistance |
| --- | --- | --- | --- |
| 01 | `01_master_teleop` | mannequin master arm, robot copies | none |
| 02 | `02_vr_teleop` | Quest controller, full 6-DOF input | none |
| 03 | `03_shared_autonomy` | master input | autonomy supplies wrist orientation |
| 04 | `04_vr_shared` | VR input | autonomy supplies wrist orientation |
| 06 | `06_full_autonomy` | spoken or typed goal | system does the rest |

---

## 2. THE FOUR TASKS

### T0 — TARGET REACHING

Six spheres, three per arm, labelled **L1–L3** and **R1–R3**.

Positions: front-high above shoulder, front-out at chest height, front-low
below chest. Visibly far apart, so a viewer sees the arm travel between
genuinely different places.

No table, no objects, no grasping. It runs in every mode. Under 06 all six are
visible and the command is **"move the left arm to L2"**.

**MUST BE TRUE**

| # | criterion |
| --- | --- |
| T0-1 | all six spheres drawn |
| T0-2 | all six coloured |
| T0-3 | all six labelled (L1–L3, R1–R3) |
| T0-4 | arm visibly moves between distinct positions |
| T0-5 | no bench in the scene |

### T1 — COLOUR-MATCHED PICK AND PLACE

Four cubes, two blue and two green. One blue plane and one green plane. Each
cube goes on the plane of its own colour.

**The table is WHITE.** Everything else about it is unchanged: same top height
(0.95), same near edge (y = 0.10), same legs and apron. It is 1.05 m
half-width rather than 0.90, because the marked region now reaches |x| = 1.00
and a marking drawn over air is a marking that lies. White is *safer* for the
clip verifier than the old oak: every detector keys on channel differences
(`_tan` needs R−B > 45, `_green` needs G−R > 45) and a neutral has R = G = B,
so it cannot fire any of them.

**"White" here is a claim about the PIXEL, and it did not use to be.** The
table asked for (0.94, 0.94, 0.95) and rendered at RGB(118,118,118) in every
shipped clip, because an RViz marker's material takes ambient = 0.5 × colour
and the only light is a headlight on the camera — so an up-facing face, which
is what a table top is, gets the ambient term alone. Measured in
`scripts/probe_marker_shading.py`; the audit now applies that coefficient and
asks for a white pixel rather than a white request. See
`docs/system/findings.md`, 2026-08-15 (later still).

**The two risers are gone from T1, T1S2, T2 and T3.** They carried the bench,
which only tasks a/b/c have, and stood on the table holding nothing in every
MSc clip — 60 mm short of the objects above them, inviting the one reading
this scene must not invite. The legs are inset so the top overhangs, and the
apron runs all four sides.

**Stage 1** (`m1`) — **the LEFT arm only, with all four cubes on the LEFT
side.** This reverses the 2026-08-12 mirror to the right arm. That mirror was
decided on `t1_layout_options.json` (left 2/4 cubes, right 4/4) and both of
its inputs have since been found wrong: the survey behind it applied the
**left** arm's wrist-to-pad offset to **both** arms, and the layout it scored
sat inside the wearer clearance floor. Re-measured on the left arm against
its own cells, N=10 over the full path with wearer and furniture in scene:
**4 of 4 cubes and 2 of 2 planes, 0 waypoint failures, 0 waypoints inside the
clearance floor.**

**Stage 2** (`m1s2`) — both arms working at once, four cubes, **and the SIDE
of each cube is part of the random draw**, not fixed at two per arm. Positions
are drawn only from surveyed cells that are reachable over the whole pick path
*and* clear of the wearer. The draw is re-taken per trial from the trial's own
seed, and that seed is stored in the manifest and read by both the task path
and the scene. At least one cube falls on each side; a draw that puts all four
on one side is rejected and re-taken, because "both arms working" is the task
definition and not a preference.

#### Where the planes are, and why they are not in the centre

The brief asks for the two planes in the **centre** of the table, directly in
front of the person. **That is not reachable, and the shortfall is 450 mm.**

> **CORRECTED 2026-08-15 (later). Two numbers in this section are stale and one
> conclusion in it was never tested.**
>
> **The columns moved when home moved.** Re-running `measure_centre_vs_height.py`
> unchanged at z = 1.120 today returns **left 0.325, right 0.450**, confirmed
> over the full path at N=10, against the 0.425 / 0.400 recorded below. The
> solver seeds from the live joint state, so the 2026-08-15 home change moved
> which null-space branch it lands in. The left arm gained 100 mm inboard and
> the right lost 50 mm, and nothing re-ran the sweep after home moved.
>
> **"The thing occupying the centre is the wearer" is right, and it is not
> their arms.** The wearer's arm posture is now a variable
> (`wearer_posture.py`), and the sweep was re-run in four postures plus the
> limit case of no arms at all: `recordings/baselines/centre_vs_wearer_posture.json`.
> With the wearer's arms **deleted entirely**, min \|x\| is still 0.300 (left)
> and 0.375 (right), every column from 0.125 to 0.300 binds on the **TORSO**,
> and at \|x\| = 0.10 the clearance is −0.007 / −0.013 m, which is the arm
> inside the person's chest. Holding the arms clear buys 25 mm on the left and
> 75 mm on the right. Folding them across the chest costs 75 mm; clasping them
> behind the back or holding them out to the sides puts the wearer's own limbs
> in the BACK-mounted robot's envelope and breaches the floor at the **home
> pose**, before anything has been asked of the arm.
>
> **And `base_link` caps every clearance figure at 0.1610 m.** It is a link no
> joint moves, and against the shipped wearer it sits 0.1610 m from the torso.
> That is why so many clearance numbers here read exactly 0.1610: it is the
> mount's clearance, not the arm's, and the 150 mm floor has 11 mm of headroom
> before any joint moves. See `docs/system/findings.md`, 2026-08-15.
Measured (`recordings/baselines/centre_reach.json`, z = 1.120, N=3, wearer and
furniture in scene, four controls correct):

| | left arm | right arm |
| --- | --- | --- |
| innermost workable column, any forward distance | **x = 0.250** | x = 0.075 |
| … that also keeps the 150 mm wearer clearance floor | **x = 0.425** | x = 0.400 |
| x = 0.00, asked directly | no y from 0.10 to 0.55 | no y from 0.10 to 0.55 |
| x = ±0.10, asked directly | no y from 0.10 to 0.55 | y 0.425–0.475 only |
| what stops it going further in | **the wearer's own arm**, 23 mm inside | the wearer / orientation |

**Asked again across every work height, and the answer does not move.** The
fair objection to the table above is that it was taken at ONE height,
z = 1.120, while `band_vs_work_height.json` shows forward reach climbing from
0.025 m to 0.500 m as the plane rises. Crossed (`measure_centre_vs_height.py`,
ten heights 1.12–1.55, both arms, stage 1 grasp-pose-only and stage 2 the full
pick path at N=10 with the wearer measured geometrically, all five controls
correct):

| work plane | innermost SAFE column, left | right | height above the table |
| --- | --- | --- | --- |
| **1.120 (now)** | **0.425** | **0.400** | 170 mm |
| 1.250 | 0.400 | 0.350 | 300 mm |
| 1.350 | 0.400 | 0.300 | 400 mm |
| 1.550 | 0.350 | 0.250 (reach only) | 600 mm |

At \|x\| ≤ 0.10 **no height works.** IK reaches the centreline easily — the
right arm solves at x = 0.025 — and not one of those cells clears the 150 mm
floor at any height. Asked again with the wrist UNPINNED to top-down, in case
the pinned wrist was what closed it: same answer, and every stage-1 survivor
at \|x\| = 0.300 turned out to be reach-only once the whole path was walked.
Height buys 75 mm inboard for the left arm and 150 mm for the right, at the
price of standing the work 430 mm further up in the air. The 1.120 row
reproduces `centre_reach.json` exactly, which is the cross-check that matters.

> **SUPERSEDED 2026-08-18 BY A MOUNT CHANGE AND A REBUILD, AND THE OLD
> SENTENCE IS LEFT ABOVE ON PURPOSE.** Everything from "The planes therefore
> sit at x = 0.450 and 0.610" onward described a T1 that ran on ONE arm with
> both pads beside the person. It is gone. What replaced it, and what each
> number now is:
>
> * **the mounts moved** 150 mm outboard and 15 deg of yaw outboard, over a
>   table at 1.250 with its near edge at 0.430, approached at 50 deg inboard
>   and 10 deg below horizontal. That is a re-derivation of the platform, and
>   it is what bought the centre;
> * **the pads are SYMMETRIC, 290 mm either side of the centreline**, one per
>   arm, straddling it. The pair is centred on the person and its inner edges
>   are 420 mm apart. Not `|x| <= 0.10` — the centre is still not reachable —
>   but the work now happens IN FRONT of the wearer with an arm either side of
>   it rather than beside them;
> * **the objects REST on the table.** T1-1 is satisfied, for the first time,
>   from the shipped scene rather than by an injected displacement;
> * **the grasp lands on the cube.** `grasp_frames.PAD_MID_EE` was 13.47 mm
>   too long — derived from the magnitude of a world-frame vector rather than
>   measured — and `verify_t1.py` now reads the finger tips off FK at the pose
>   the task commands and reports a 0.00 to 0.01 mm pad miss;
> * **290 and not 260**, because stage 2 can put THREE cubes on one pad and
>   the innermost of three slots at a 260 mm centre lands at `|x| = 0.200`,
>   which the per-column sweep calls clear from HOME and the composed path does
>   not: 0.1219 m to the wearer's own upper arm on 15 waypoints.
>
> The result about shoulder-mounted arms below still stands and is still the
> point: the thing occupying the centre of the workspace is the wearer.

This is a result about shoulder-mounted arms, not a layout failure. The thing
occupying the centre of the workspace is the wearer, and moving the mounts
150 mm outboard bought 290 mm rather than 0 — the centre itself is still shut.

#### NEITHER ARM CROSSES THE CENTRELINE, AND THAT DECIDES WHAT STAGE 2 CAN BE

Measured 2026-08-18, N=10, wearer and table in the planning scene: **0 of 10 IK
solutions at every cross-side pad slot and every cross-side cube position, on
both arms.** So the arm that can reach a cube is fixed by the side it starts
on, the arm that can reach a pad is fixed by the pad's side, and **a cube can
only be delivered to the pad on its own side.**

Stage 2 draws the SIDE of each cube, so its colour follows from that side. A
stage 2 that drew colour and side independently would be drawing trials the rig
cannot perform. What varies per trial is the SPLIT — 1/3, 2/2 or 3/1 — and the
columns each cube occupies, which is the asymmetric bimanual load stage 2
exists to create. `t1_task.build()` REFUSES a cube whose colour names the other
side's pad, rather than emitting a path whose every waypoint fails IK for a
reason nobody would connect to colour.

**MUST BE TRUE**

| # | criterion |
| --- | --- |
| T1-1 | cubes rest ON the table, not floating |
| T1-2 | cubes start clear of the planes — no cube shares a position with a plane |
| T1-3 | four separate close/open cycles, not one |
| T1-4 | each cube moves only while gripped |
| T1-5 | pads on the cube and not inside the table |
| T1-6 | each cube ends on the correct colour |
| T1-7 | workspace markings drawn on the correct arms and enclosing all objects |
| T1-8 | the table is WHITE |
| T1-9 | stage 1 runs on BOTH arms with the two pads straddling the centreline, and a cube is run by the arm that can reach the pad of its colour. **This criterion said the opposite until 2026-08-18** — "the LEFT arm only, with all four cubes on the LEFT" — which was the pre-rebuild T1 with both pads 595 and 825 mm off centre beside the person |
| T1-10 | the pads sit at the innermost column that is reachable AND clear of the wearer with the project's 20 mm margin, measured per column, and the distance from the centreline is stated, not hidden: **290 mm either side** |
| T1-11 | **every T1 and t1s2 waypoint keeps the 150 mm wearer clearance floor**, measured geometrically and not by `avoid_collisions` |
| T1-12 | stage 2's cube SIDES are drawn at random, both arms always get work, and the seed is stored in the manifest and read by the task |

### T2 — BIMANUAL COORDINATED CARRY

Both arms grip one rigid **500 mm** tray with a loose ball on it. Lift,
transport, place.

Bimanual by **COUPLING**: one body, two grips, neither arm's pose free given
the other's. Tilt past **6.8 degrees** drops the ball.

**MUST BE TRUE**

| # | criterion |
| --- | --- |
| T2-1 | tray held by BOTH grippers |
| T2-2 | ball visible ON the tray |
| T2-3 | tilt logged continuously through the carry, not pass/fail at the end |
| T2-4 | separation logged continuously through the carry, not pass/fail at the end |

#### T2 AND THE 2026-08-16 HOME CHANGE — the honest record, including a scare

The home pose was re-solved from geometric constraints on 2026-08-16. Every
figure below was measured **on the same day with the same instrument**, N=10
over the full path, wearer and furniture in the scene, clearance geometric —
the old home was re-measured rather than quoted, because a regression against
a number from a different run is not a regression.

| | old home (2026-08-15) | intermediate candidate | **shipped 2026-08-16** |
| --- | --- | --- | --- |
| T2 left | **2** of 11 unsolvable | 0 | 0 |
| T2 right | 0 | **8** of 11 | **2** of 11 |
| T2 total | **2** | **8** | **2 — unchanged** |

**An intermediate candidate did regress T2 from 2 to 8, and the pose that
shipped does not.** The cause is the IK seed: `/compute_ik` seeds from the
live joint state, so moving home moves which null-space branch the solver
lands in. Two candidates that satisfy the same geometric constraints can
therefore differ by six waypoints on a task neither of them touches. That is
worth knowing on its own: **T2's IK count is a property of the seed, not of
the task**, and any future home change must re-measure it rather than reason
about it.

The candidate that regressed it had a continuous joint resting **on the ±π
seam** (left joint_5 at exactly 180.00 deg, 0.00 rad of margin against the
0.30 rad this project keeps). A seam-margin constraint was added to the solver
and the re-solved pose both clears the seam by 0.57 rad and leaves T2 where it
was.

**READ BOTH NUMBERS ON TOP OF THE OPEN-GRIPPER DEFECT.** T2's grip trace reads
0.00 knuckle for all 992 samples on both arms while the schedule commands
`grip_for(30)` on every waypoint: **the task has never actually held the
tray**, under either home. Its right grip also presents 113.9 mm to an 85 mm
hand, which cannot close, and its wearer clearance is 0.0026 m against a
0.150 m floor — a breach of 147 mm — under the old home as well as the new.

So T2 was not performable before this change and is not performable after it,
and its unchanged IK count says nothing about whether it works. T2 is tracked
as its own task: the gripper must actually close, the right-hand grip must
fit, and the wearer floor must be respected, before its IK failure count means
anything.

#### What the 2026-08-16 home change did to the other four, measured

Same run, same instrument, old home re-measured the same day:

| task | old home | new home | |
| --- | --- | --- | --- |
| T0 | 9 IK failures (5 L / 4 R) | **7** (4 L / 3 R) | improved; still not performable, and it was not performable before either |
| T1 | 0 failures, clearance 0.1610 | **0**, 0.1610 | unchanged |
| T1 stage 2 | 0 failures, right-arm worst clearance **0.1405** | **0**, right-arm worst clearance **0.1024** | IK unchanged; the clearance breach **deepened by 38 mm** |
| T3 | 0 failures, 0.0618 / 0.1610 | **0**, 0.0618 / 0.1610 | unchanged |

**T1 stage 2's right arm is the one thing that got worse and it must not be
buried.** It has 0 IK failures under both homes, so every IK-based check calls
it clean; measured geometrically it sits 0.1024 m from the wearer against a
0.150 m floor, where before it sat at 0.1405 m. Both are breaches. This is the
SRDF gap again — `avoid_collisions` cannot see these pairs — and it is exactly
the class of defect `docs/system/clearance_gap_ledger.md` exists to track.

### T3 — CIRCUIT BOX AND MULTIMETER

One circuit box with four measurement points, one multimeter
(**30 x 70 x 45 mm**). One arm holds the box steady, the other presents the
meter to be read.

Bimanual **BY ROLE**, not coupling — two objects, two places.

Approach differs per object: the box needs **top-down**, the meter needs
**near-side**. That is measured, not a preference.

**And neither is what the code does.** `run_abc.send()` writes the pinned
anchor into every waypoint of every task under every mode, so there is no
per-grasp approach anywhere in the code — every grasp in the set is a
near-side one. Measured per object over the full path at N=10, with the
closing axis read from FK on the two finger-tip links
(`scripts/measure_grasp_approach.py`, eight controls including the pad offset
reproducing `PAD_OFFSET_BY_ARM` to 0.1 mm):

| object | pinned | top-down | the hand opens 85 mm |
| --- | --- | --- | --- |
| T1 cube (40 mm) | 49.0 mm across the closing axis | 40.0 mm | both fit; pinned closes across a DIAGONAL, not a face |
| T2 tray, left grip | 83.4 mm | no yaw solves it | fits by 1.6 mm |
| T2 tray, right grip | **113.9 mm** | no yaw solves it | **does not fit** |
| **T3 circuit box** | **195.0 mm** | **170.0 mm** | **does not fit under either approach** |
| T3 multimeter | 42.4 mm | 30.0 mm | both fit |

The box is reachable and clear of the wearer under both approaches, and the
hand is aimed squarely at its 170 mm width. What it needs is a ROLL about the
approach axis to close on the 50 mm height — not a different approach and not
a different position. **No reachability check in this repository asks this
question**, which is why a grasp that cannot physically close has verified
clean for as long as it has existed.

**MUST BE TRUE**

| # | criterion |
| --- | --- |
| T3-1 | exactly one box |
| T3-2 | four measurement points drawn |
| T3-3 | each object held by the correct arm |

---

## 2A. THE WORKSPACE — what it is, and what limits it

Everything in this section was measured on 2026-08-15 against the current
one-table scene, with the wearer and the furniture in the planning scene, at
the T1 work plane z = 1.120, with the wrist pinned at the anchor as every
teleop mode commands it. Each script carries its own controls and prints
nothing if one fails.

### The region

| | IK-reachable, full pick path | **also clear of the wearer** |
| --- | --- | --- |
| left | 265 cells, x +0.250…+1.000 | **193 cells, x +0.425…+1.000** |
| right | 314 cells, x −1.000…−0.225 | **246 cells, x −1.000…−0.400** |
| both arms | 0 | 0 |
| \|x\| ≤ 0.10 (front centre) | **0** | **0** |

**Read the x-ranges as the whole of the difference.** The two columns stop in
the same place outboard and 175 mm apart inboard, and that inboard strip is
the strip the SRDF could not see. An earlier version of this table printed
the clearance-safe range against the IK column, which quoted the safe
boundary while counting the unsafe cells; corrected 2026-08-15 against
`work_surface_region.json` and recorded in `docs/system/findings.md`.

`recordings/baselines/work_surface_region.json`, 25 mm cells, full pick path,
merged from three clearance surveys. **Two things about it are new and both
change what the marking means.**

**1. The clearance floor is now part of the region, and it never was before.**
Every workspace number this project has ever quoted is an IK number, and IK
cannot see the wearer where it matters: the SRDF permanently excludes
torso/harness/backpack against each arm's base, shoulder and half_arm_1 —
exactly the pairs a shoulder-mounted arm threatens — so `/compute_ik` returns
`valid` for poses with the tube inside the person. Measured geometrically with
the mount guard's own capsule model, **72 of the left arm's 265 IK-reachable
cells and 68 of the right's 314 are inside the 150 mm floor, the worst at
−2.7 mm.** The shipped right-arm T1 layout spent **70 of its 143 waypoints**
inside the floor with **zero IK failures**, and so did the idle arm's park
pose, at 30.6 mm. Every check this project had called that layout clean.

**2. The old boundary at |x| = 0.70 was the survey box, not the arm.** Beyond
it there are 108 more left cells and 125 more right cells out to |x| = 1.000,
and **every one of them is 100% clear of the wearer.** The marking a
participant is shown was understating the usable space by 300 mm per side
while overstating it by 175 mm at the inboard end, where it is not safe.

### What binds, per direction

`recordings/baselines/what_binds.json`. The ladder is: collision-aware with
furniture → collision-aware without furniture → collisions off → orientation
free. The first rung that succeeds names the constraint. Four controls, all
correct, including a box placed on a cell just called reachable, which must
then read FURNITURE.

| direction | reach from a surveyed seed | **what binds** | clearance at the last reachable pose |
| --- | --- | --- | --- |
| forward (+y) | 0.300 m, to y = 0.425 | **ORIENTATION** — the pinned wrist | 0.132 (L) / 0.116 (R) — **already under the floor** |
| outboard | 0.600 m, to \|x\| = 0.950 | **ORIENTATION** — the pinned wrist | 0.161 / 0.161 |
| inboard | 0.125 m (L) / 0.150 m (R) | **THE WEARER** — see the correction below | 0.023 / 0.016 |
| down (−z) | 0.100 m (L) / 0.125 m (R) | **FURNITURE** — the table top | 0.121 / 0.136 |
| up (+z) | not bounded within 0.700 m | — | 0.161 / 0.068 |
| back (−y) | not bounded within 0.700 m | — (behind the person; not usable space) | 0.161 / 0.161 |

**No direction is bound by a joint limit or by arm length.** Nothing returned
KINEMATIC except the 1.6 m control. Forward and outboard are bound by the
pinned wrist, inboard by the wearer on **both** arms — 0 mm inside the left
forearm and 8 mm inside the right — and down by the table.

**"The wearer" inboard means the TORSO on the left arm and the ARM on the
right, and that distinction was never made because the arms could not move.**
Re-measured 2026-08-15 with the wearer's arm posture as a variable: the left
arm's limiting wearer part is the torso at every column from 0.175 to 0.300,
the right arm's is its own forearm and upper arm. Deleting the wearer's arms
outright moves the left limit by 25 mm and the right by 75 mm and leaves the
centre shut. Full table in `docs/system/findings.md`, and the operating
consequence is in the corrected block under T1 above: the useful wearer
instruction is "arms down at your sides", which is what the model already
assumed.

**Read the last column with the third one.** The IK boundary and the safe
boundary are not the same boundary, and in four of the six directions the
clearance floor is reached first. Forward, the arm is at 132 mm and 116 mm
against a 150 mm floor while IK still solves for another 25 mm. The usable
workspace is the smaller of the two everywhere, and until this measurement it
had only ever been quoted as the larger.

### The band, and the height claim

The remembered claim is "the reachable band on the work surface is
y = 0.06–0.20 and is the SAME at every table height from 0.90 to 1.30 m."
**Half of it holds and the half that holds is the important half.**

Objects **on the table**, probed 20 mm above its top, table moved to each
height (`band_vs_table_height.json`):

| table top | 0.90 | 0.95 | 1.00 | 1.05 | 1.10 | 1.15 | 1.20 | 1.25 | 1.30 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| band, y | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 | 0.00–0.05 |

**Invariant, exactly as remembered — and 50 mm wide, not 140.** Raising or
lowering the table buys nothing at all, because what limits it is the table
itself: the approach axis points 30.7° *above* horizontal, so the forearm
trails below and behind the fingertip, and over a slab that trailing volume is
inside the slab. The band rises with the surface and never moves forward.

The same probe with the table left at 0.95 and only the work plane moved
(`band_vs_work_height.json`) shows the other half:

| work plane z | 0.90 | 0.95 | 1.00 | 1.05 | 1.10 | 1.15 | 1.20 | 1.30 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| left, y max | none | 0.025 | 0.100 | 0.175 | **0.425** | 0.450 | 0.475 | 0.500 |
| right, y max | none | 0.025 | 0.125 | 0.200 | **0.450** | 0.475 | 0.475 | 0.500 |

So the band is **not** the same at every height and it is **not** 0.06–0.20.
It is a function of how far the work is **above** the table, saturating near
0.50 m at about 200 mm of clearance. The current layout already exploits this:
the objects are fixtured 170 mm above the table and get 0.425 m of forward
reach. The old "0.06–0.20 everywhere" figure was measured with the bench in
the scene, and the bench is what made it look height-invariant.

### The mount tilt, priced again

`recordings/baselines/forward_reach.json`, option A, and the price has not
changed:

| | left y max | right y max |
| --- | --- | --- |
| baseline (mount as built, tilt 60°) | 0.050 | 0.050 |
| tilt −30° | 0.125 | 0.150 |
| **tilt −45°** | **0.200** | **0.225** |
| translate forward 0.30 m | 0.050 | 0.050 |

That is the ~0.15 m of forward reach, and **it is an upper bound, not an
offer.** Option A is implemented as the inverse transform on the *target*,
which is exact for kinematics and wrong for wearer collision, because the
wearer does not tilt with the mount. Every clearance figure in this document
would have to be re-measured before a single number of it could be believed.

What it invalidates, with the current geometry: `P_HOME` for both arms; the
pinned anchor orientation and therefore `PAD_OFFSET_BY_ARM`, which is measured
off TF at the anchor; every T0–T3 coordinate, all of them derived through
`ee_for()` from that anchor; the home pose itself, now the presentation pose
with the wrists level and 0.1610 m of clearance against a 0.15 floor; and all
579 cells of the region above. It is a re-derivation of the whole platform,
and the arms have never been run.

**A mount change is not what the 2026-08-15 home change was, and the two must
not be confused.** Moving home moved where the arm RESTS and nothing else:
the anchor is a stored constant that nothing recomputes from home, so no task
coordinate and no clearance figure moved, and the task set was re-measured to
prove it. A mount change moves the anchor itself, which is what makes it a
re-derivation. See `docs/system/home_wrist_is_real.md`.

### The cheap options, priced

| option | what it buys | what it costs |
| --- | --- | --- |
| **use the space already measured** (|x| to 1.00) | **+300 mm per side, 233 new cells, all 100% clear of the wearer** | the table widens to ±1.05 (scenery, no verified coordinate); the marking and stage-2 pool change; **no task coordinate moves** |
| raise the work plane 1.12 → 1.20 | +50 mm forward | `T1_Z` moves, so the whole layout is re-derived and re-verified; objects sit 250 mm above the table |
| move the table | **nothing** — measured invariant at every height 0.90–1.30 | — |
| wearer stands further back | **nothing, and it is the wrong sign** — the work moves away from the arm, not toward it | — |
| lower the clearance floor below 0.15 | the inboard 175 mm (x 0.25–0.425) | HARD CONSTRAINT 11. It is the last thing between the arms and a person's chest, and the cells it would buy are the ones measured at 23 mm and −2.7 mm |
| unpin the wrist | forward and outboard, both of which bind on ORIENTATION | changes what the modes ARE; the pinned wrist is the thing the study compares |
| tilt the mount −45° | ~0.15 m forward, as an upper bound | the whole re-derivation above |

### RECOMMENDATION

**Take the space that is already there: extend the usable region to
|x| = 1.000 and drop the inboard cells that breach the clearance floor.** It
is the only option that buys real space at no cost to anything verified — 233
new cells, every one of them measured clear of the wearer, no task coordinate
moved, no mount touched, no floor lowered. It is done: the region file, the
marking, the stage-2 sampling pool and T1's own layout all now run on it.

**And say the rest plainly, because it is a finding and not a failure.** The
workspace of a shoulder-mounted supernumerary arm is bounded *inboard by the
wearer* and *forward by the pinned wrist*, and neither can be widened from
software. The centre of the person's own workspace is the one place these arms
cannot go, and the 450 mm the planes sit off centre is the size of that
result. Widening it needs a mount change, which invalidates every coordinate
and every clearance figure in this repository, on a platform that has never
been run against a real arm.

---

## 3. THE DANCE

Three routines. What exists reads as waypoint traversal. It must read as
**performance**, not a motion test.

| # | requirement |
| --- | --- |
| D-1 | a beat — motion on a tempo with phrasing: anticipate, accent, hold, release. Not constant velocity between points |
| D-2 | anticipation at **0.30 s** so it survives 5–13 fps capture. A 0.10 s anticipation is one frame and invisible |
| D-3 | the two arms in relationship: unison, canon, call-and-response, opposition. Not just mirrored waypoints |
| D-4 | each routine a distinct character, recognisable without being told which is which |
| D-5 | use the whole envelope. With the bench gone there is far more room than the current routines use |
| D-6 | hits and holds — a pose held on a beat reads as intentional; arriving and stopping dead reads as a machine hitting a setpoint |
| D-7 | every waypoint through `/compute_ik` with `avoid_collisions`, normal velocity caps, e-stop abortable |

Film it and **watch it**. If it does not look like dancing, iterate before
showing the user.

---

## 4. THE UNIVERSAL SOLUTION

Language works. Perception does not. The gaps:

| # | gap |
| --- | --- |
| U-1 | no camera publisher in sim, so the detection path has never run end to end |
| U-2 | the fingerprint records position but **not YAW**, so an angled object gets a square grasp |
| U-3 | table height is declared, never measured; `set_measured()` exists and nothing calls it |

Build the mock camera path so the whole pipeline is exercised **to the camera
boundary**, wire yaw into the fingerprint **and the grasp**, wire
`set_measured` from depth, and then **list explicitly what still needs real
cameras**.

---

## 4A. THE PERCEPTION-DRIVEN GRASP — measured and built, 2026-08-16.

**The robot cannot pick what it has not seen.** Every grasp in this repository
used to be computed from a coordinate written into the task file. This section
records what was measured and what now works.

### THREE CLAIMS THAT WENT ROUND AND ARE FALSE. Do not design against them.

They were relayed from an earlier report, checked, and contradicted by
measurement. They are written down here **because a wrong constraint that
survives gets designed around later**, which costs more than the original
error.

| claim | measured |
| --- | --- |
| "the camera is mounted 88.4 deg off the tool axis" | the mount OFFSET direction is **93.10 deg** — and that is where the camera SITS, not where it LOOKS |
| "camera-on-top and wrist-level are mutually exclusive" | **both hold at once in the shipped home**: approach elevation −1.47 / −1.42 deg with the camera **12.05 deg off vertical**, on both arms |
| "look-then-grasp is not available on this rig" | **both arms have a verified-USABLE observe pose** — `/compute_ik` 10 of 10, clearance 0.1610 m, transit clean at 0.1587 / 0.1599 m via a via point |

`camera_color_frame` sits at rpy (pi, pi, 0) from `end_effector_link`, which is
diag(−1, −1, 1): the optical axis IS the tool axis, to **0.00 deg**. The camera
looks exactly where the gripper points, by construction.

### BUILT AND MEASURED, 2026-08-16: look-then-grasp runs end to end

`scripts/verify_look_then_grasp.py`, left arm, mock wrist camera, N=10 on every
planned waypoint:

| step | result |
| --- | --- |
| home -> via -> observe | 24.1 s, arrives to 0.017 rad |
| detect + classify | **0.31 s**, 4 cubes, 2 pad-blobs rejected by the size gate |
| classification | **4 of 4 correct**, 0 wrong colour, 0 missed, worst localisation **3.6 mm** |
| plan derived from detections | 4 picks, destination pad chosen by the SEEN colour |
| **control P-B** | declared cube coordinates shifted **0.25 m** in memory -> **plan unchanged**, so it is reading the detection and not the file |
| added time | **48.6 s per look**, **12.1 s per pick** (one look serves all four cubes) |

**THREE THINGS HAD TO BE FIXED AND ONLY ONE WAS THE OBVIOUS ONE.**

1. **Size at range.** The pads are 210 x 130 mm in the cubes' own colours and
   `colour_shape_detector` had a `min_area_px` FLOOR with no upper bound. It
   locked onto the pads: 0 of 4. A blob is now rejected unless it is the size
   that object would be at the range it appears to be at -- 170 px where a cube
   would be 31 px.
2. **A depth step, because COLOUR ALONE CANNOT DO IT.** The cubes touch the
   same-coloured pads in projection, so 8-connectivity merges them into one
   blob. Two attempts failed first and are recorded in the code: a single depth
   threshold recovered one of two cubes (the pad is tilted and spans ~40 mm of
   depth across its own width), and a morphological closing was worse still,
   because the cubes protrude past the pad's EDGE rather than sitting inside
   its silhouette. What works is cutting the mask at the depth DISCONTINUITY.
   **This means the separation REQUIRES DEPTH**; `colour_shape_detector`
   subscribes to colour only, so on the real rig it needs the depth image or
   pads in non-cube colours.
3. **A scorer bug of my own.** The match tolerance was 80 mm against a 60 mm
   cube pitch, so each cube was given its NEIGHBOUR's detection -- and since
   the colours alternate, two detections sitting 3.5 mm from truth with the
   right colours scored as wrong-colour. Matching is now globally nearest-pair
   and capped at 30 mm, half the pitch.

**THE GRASP MUST BE PLANNED AFTER RETURNING HOME, NOT FROM THE OBSERVE POSE.**
`/compute_ik` seeds from the live joint state, so where the arm stands when the
grasp is planned decides which null-space branch comes back. Same 24 waypoints,
same targets, both with 0 IK failures:

| planned from | worst wearer clearance |
| --- | --- |
| the observe pose (where the arm is when the frame is taken) | **0.0017 m — a 148 mm breach** |
| home, after returning | **0.1610 m — clean** |

The obvious sequencing (look, then grasp from where you looked) is the unsafe
one, and no IK-based check catches it: both plans solve every waypoint.

### What the wrist camera actually does today, measured

`camera_link` hangs off `end_effector_link` at (0, 0.05639, −0.00305) with
rpy (π, π, 0), which makes the camera frame `diag(−1, −1, 1)` in the EE frame.
So **the optical axis is the tool axis** — the camera looks exactly where the
gripper is going, 0.7 deg off on the left arm and 0.0 on the right. That part
is right by construction and needs no work.

**But the approach is not along the tool axis, and that is the problem.**
`run_abc.send()` pins the wrist at the anchor for every waypoint, so the
orientation is CONSTANT through a reach while the PATH is a vertical descent
from a 0.10 m standoff (`msc_clip_tasks.STANDOFF`). The anchor points 30.8 deg
(left) / 22.1 (right) above horizontal. Measured against T1's own geometry:

| | left | right |
| --- | --- | --- |
| camera to cube, at the pre-grasp standoff | 0.155 m | 0.167 m |
| camera to cube, at the grasp | 0.128 m | 0.128 m |
| **cube's angle off the optical axis at the standoff** | **65.8 deg** | **62.5 deg** |

**So the answer to "is the cube in frame at the pre-grasp standoff" is NO.**
At 62–66 deg off-axis it is outside the horizontal field of view of any
RealSense-class module (roughly ±32 deg), and it only enters the frame as the
hand drops onto it. And for the whole of that descent the object is nearer
than **0.25 m**, the minimum range of the depth module the Gen3 vision head
uses — so depth is unusable exactly where the object finally becomes visible.
The cube is never simultaneously in frame and in depth range.

**None of this was a designed choice.** The camera points wherever the pinned
wrist lands, and the pinned wrist was chosen for reach, not for seeing.

### What it would take

| # | requirement |
| --- | --- |
| P-1 | a **LOOK pose** per pick, distinct from the pre-grasp standoff: object on the optical axis to within the FOV margin, and at **≥ 0.25 m** so depth is valid. It is a new pose to solve for, with the same wearer-clearance floor as everything else, and it must be reachable over the full path |
| P-2 | the sequence becomes **look → DETECT → compute the grasp from what was seen → approach → grasp**. Today it is approach → grasp, with the coordinate read from a file |
| P-3 | the grasp must be computed from the DETECTION, not from the task file. `grasp_generator` already reports `yaw_source`; the pose it consumes has to come from the tracker rather than from `msc_clip_tasks` |
| P-4 | a **fallback when detection fails**, stated and logged, because a blind pick that silently falls back to the file coordinate is indistinguishable from a working perception path — the exact failure mode this document's audit log is full of |
| P-5 | the descent must not lose the object: either re-detect from the LOOK pose and move open-loop, or accept that no visual servoing is possible inside 0.25 m and say so |

### What already exists, and what is missing

| piece | state |
| --- | --- |
| mock RGB-D publisher | EXISTS, wired behind `mock_camera:=true` (U-1, closed) |
| detector + tracker + yaw | EXISTS, `verify_detection_path.py` runs the chain (U-2, closed) |
| table height from depth | EXISTS, `work_surface_node` (U-3, closed) |
| known-dimension depth fit | EXISTS |
| **a pose from which the object is visible AND in depth range** | **MISSING** |
| **any consumer that computes a grasp from a detection** | **MISSING** |
| **a detection-failure branch** | **MISSING** |

**MUST BE TRUE, when it is built**

| # | criterion |
| --- | --- |
| P-A | the object is in frame and within depth range at the LOOK pose, measured from the rendered camera, not asserted |
| P-B | the grasp pose used is provably the DETECTED one — change the file coordinate and the arm still goes to the object |
| P-C | detection failure is visible in the trial record and does not silently become a file-coordinate pick |

---

## 5. THE GUI — a session with no terminal

| # | requirement |
| --- | --- |
| G-1 | launch every mode |
| G-2 | run every task |
| G-3 | readiness gate that names each failing check and is **never green on unknown** |
| G-4 | master arm and robot schematics with per-joint health |
| G-5 | divergence between commanded and actual |
| G-6 | both cameras |
| G-7 | active blockers |
| G-8 | live parameters |
| G-9 | e-stop |
| G-10 | session control with redo, skip, abort and resume |

**Click every button and confirm it does what its label says.**

---

## 6. HOW TO RECORD

| # | rule |
| --- | --- |
| R-1 | archive the existing set first with a note on what changed. `mode06_GOOD` is **never** touched |
| R-2 | clear the `rvizcfg` cache before any framing test. A stale cached config is why the last session did not converge |
| R-3 | eight angles: front, back, left, right, iso, gripper, quad, plus the card |
| R-4 | **the work must be CENTRED and FULLY IN FRAME.** It is currently at the right edge and cut off. The front view follows the task, `record_rviz.py` ~line 121, and the cache holds `front_t1.rviz` beside `front.rviz` |
| R-5 | information card **FIRST**, held long enough to read, then the footage. Not an overlay. States mode, task, what is expected, what to watch for |
| R-6 | card text written as a person would write it. No em dashes. No "leverage", "robust", "seamless", "delve". No tricolon lists. Read it back and cut anything that sounds generated |
| R-7 | every clip opens on the **presentation pose** if it exists. Settle whether it does — it has been reported both built and absent |
| R-8 | identify the tall box at bottom-left of frame. An unidentified object in every shot is either undeclared scene geometry or a leftover |
| R-9 | run **DETACHED with a waiter**. Polling kills runs |
| R-10 | push after each completed mode, verify from `git ls-remote` |

---

## 7. THEN INSPECT — and this is the part that matters

Extract frames and **LOOK** at every clip. For each, answer from the pixels
against the MUST BE TRUE list for that task.

**Report what you SEE, not what the checks return. Every superseded set passed
its checks.**

Anything wrong: fix, re-record, re-check. Repeat until nothing is wrong.

---

## 8. BEFORE YOU START

Three things have been reported done and later found absent:

1. the sweep's task set,
2. the cube-plane overlap fix,
3. possibly the presentation pose.

So **verify each MUST BE TRUE item exists in the code BEFORE recording, and
report which were missing. One pass, not one clip at a time.**

The mechanical form of this pass is `scripts/audit_task_spec.py`, which reads
this document's criteria IDs and checks each against the code and the task
specs without needing a stack. Its findings are recorded in
`docs/system/findings.md` and in the audit table below.

---

## 9. AUDIT LOG — code-side verification of every MUST BE TRUE

Filled in by the pre-recording pass. `PRESENT` = the code does it.
`MISSING` = specified here and absent in code. `RECORD` = to be answered from
the pixels only.

Run `python3 scripts/audit_task_spec.py` for the live table, and
`--self-test` to confirm the audit can still fail (12 probes, each breaking
one input on purpose).

**Result of the pre-recording pass, 2026-08-13: 40 PRESENT, 0 MISSING,
1 BLOCKED, 1 pixels-only.** Nine items were missing or wrong when the pass
started. All nine are now built; the one that is BLOCKED is blocked by a
measurement, not by unfinished work.

### What the pass found missing, and what was done

| id | found | fixed by |
| --- | --- | --- |
| T1-7 | the marking excluded **both** coloured planes by 70–130 mm in y. `WORKSPACE` was a hardcoded box surveyed **with the bench in the scene**, and the bench is gone | re-surveyed against the current scene at 25 mm, both sides, full path, all controls correct, 14751 IK calls. `clip_scene` now **reads** the survey and refuses if it is absent; the marking is drawn as the measured **cells**, because at 50 mm the right arm's cells fill only 62% of their own bounding box and a drawn box would claim a third of a region nobody measured |
| T1-7 | both arms' markings were drawn in a one-arm task, putting a large empty rectangle mid-frame | `marked_arms(task)` — T1 draws only `T1_ARM` |
| T2-3, T2-4 | `tasks.TASK_B` had declared `log_continuously = (tilt_deg, sep_err_mm, height_diff_mm)` since the task was written and **nothing read it**. The carry could only be scored pass/fail at the end | `clip_scene.tick()` samples both every tick into `scene_events.carry_series`; `_carry_summary()` reduces it; 6 known-answer tests, including a tray that swings to 9 deg and recovers, which every end-state check scores identically to one that never moved |
| D-2 | anticipation was 20% of a move's waypoint count. It lands at 0.45–0.60 s today by luck of the beats; a 1-beat key would render 0.10 s, which is one frame at capture rate | `ANTICIPATE_S = 0.30` as a floor in **time**; a move too short to carry it gets **no** wind-up rather than an invisible one, and says so |
| U-1 | `mock_rgbd_camera` existed but no launch file or run script started it, so camera → detector → tracker had never run end to end | wired into `perception.launch.py` behind `mock_camera:=true`; `scripts/verify_detection_path.py` runs the chain — 3 raw detections, 2 tracked, yaw measured for both |
| U-2 | an identity quaternion means *nobody measured the orientation* and every consumer read it as *zero degrees*, so an object at 30 deg got a square grasp | identity is `None` throughout; the fallback detector publishes the angle it was already measuring and discarding; yaw is first-class and averaged circularly inside the object's symmetry; `grasp_generator` reports `yaw_source` |
| U-3 | `set_measured()` had **no producer** — the fault injector was its only caller | `work_surface_node` measures the height from depth (mode over 2 mm bins, not a mean, so objects standing on the table cannot drag it); `work_surface.subscribe()` carries it across the process boundary. Live: **0.9503 m against a rendered 0.950**, and a blind region refuses |
| R-7 | the presentation pose was **never built**. Reported present twice; `git log -S` finds nothing | derived, not guessed: `find_presentation_pose.py` sweeps one wrist joint, reads the tool axis from FK, and checks wearer clearance **geometrically** — `/check_state_validity` is not the authority here, because the offending pairs are SRDF-excluded. Tool axis +30.8 → **+6.5 deg** (left) and +22.1 → **+7.5** (right), clearance 0.1610 m against a 0.15 floor. The sweep commands it, **waits for arrival**, and records `opened_on` per clip |
| — | `msc_clip_tasks` said T1 runs on the **LEFT** arm in its `expect` text and its section comment, while `T1_ARM` is `right` | `expect` is now derived from `T1_ARM` |
| — | T3's information card described the meter touching each test point, which the path never does | rewritten to what the path does |
| — | `clip_scene.py` used the leaked loop variable `_arm` in the yaw check, so every left-arm item was measured against the **right** gripper's axis | uses the item's own arm |
| — | `verify_mock_camera.py` printed **FAIL** for a deprojection error of 0.2 mm against a 5 mm tolerance, because it also required 2 objects in view and reported "too few to judge" as failure | three states: OK / FAIL / `??` insufficient, and the insufficient case names the scan pose |

### T1-1 is BLOCKED, and here is the measurement

The spec says the cubes must rest on the table. They float 150 mm above it.
The repository's existing answer was that support geometry is impossible —
pedestals 0 of 4 cubes reachable, lips 16–22 waypoint failures, footprint pads
26, side posts 65. But every one of those asked whether a support can go
**under** objects at z = 1.12 with the table left at 0.95. The spec asks
something else: **raise the table so its top is what they stand on.**

Measured directly, on the arm T1 actually runs on, both controls correct
(`scripts/measure_objects_on_the_table.py`, 2486 IK calls, N=5 over T1's six
pick paths densified to 30 mm):

| slab top | waypoint failures, of 54 |
| --- | --- |
| no slab *(control, must be 0)* | **0** |
| 0.950 *(shipped)* | **0** |
| 1.000 / 1.020 | 4 / 4 |
| 1.040 / 1.050 | 16 / 24 |
| 1.060 / 1.070 | 28 / 28 |
| 1.080 | 40 |
| **1.100 — cubes resting on it** | **46** |
| slab through the objects *(control, must be > 0)* | 54 |

**So T1-1 cannot be satisfied WITH THE TABLE WHERE IT IS**, and that
qualifier turns out to matter. The highest slab that costs nothing is exactly
where the table already is — but every candidate above holds the table at its
current distance and raises it under the objects.

**A SEARCH THAT MOVED THE TABLE INSTEAD FOUND A CELL, 2026-08-15.**
`scripts/search_centre_on_surface.py` swept table height AND distance with the
objects RESTING ON the surface: 3360 cells, three controls correct including
an object buried in the slab that must fail. The survivors all share one
property — **overhang 0.00, the object at the very front edge** — because the
pinned wrist arrives from the near side and below, and the only place with no
table under the approach is the edge. The best confirmed cell is x = 0.350,
table top 1.00, near edge 0.530, reachable and clear over the full pick path
at N=10 with clearance 0.1524 m.

**That is a lead, not a result.** One cell is not a layout: four cubes and two
pads all have to fit along that front edge and all have to verify. But it is
the first evidence in this project that objects on the surface are reachable
at all, and it means T1-1 is blocked by the table's POSITION rather than by
the platform. See `docs/system/findings.md`.

And the cause is now separated from the symptom. With the wrist positions
re-derived for a **top-down** hand, the same slab at 1.100 costs **12 of 54**
against the pinned wrist's 46, while both reach every waypoint with no slab at
all. **The pinned wrist is what blocks it** — the anchor axis sits 30.7 deg
above horizontal, so the hand arrives from the near side and from below,
through the volume a table top occupies. Every teleop mode pins the wrist, so
this is the platform and not the layout. Unpinning it would change what the
modes *are*, which is the thing the study compares — and would not fully close
it either, since top-down still loses 12.

Recorded as `BLOCKED` by the audit rather than as a silent gap, and the clip
caption already carries the related cost: an object that cannot fall cannot be
dropped, so `drops` is not a measurable outcome for T1.
