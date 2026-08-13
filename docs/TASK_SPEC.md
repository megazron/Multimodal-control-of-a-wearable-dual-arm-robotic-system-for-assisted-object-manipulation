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

The task layer is **mode-independent by design**: the same waypoints go out
under every mode, so robot performance is identical across modes and every
measured difference comes from the operator.

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

Four cubes, two blue and two green, on the table. One blue plane and one green
plane. Each cube goes on the plane of its own colour.

* **Stage 1** (`m1`): one arm.
* **Stage 2** (`m1s2`): both arms at once, positions randomised.

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

### T3 — CIRCUIT BOX AND MULTIMETER

One circuit box with four measurement points, one multimeter
(**30 x 70 x 45 mm**). One arm holds the box steady, the other presents the
meter to be read.

Bimanual **BY ROLE**, not coupling — two objects, two places.

Approach differs per object: the box needs **top-down**, the meter needs
**near-side**. That is measured, not a preference.

**MUST BE TRUE**

| # | criterion |
| --- | --- |
| T3-1 | exactly one box |
| T3-2 | four measurement points drawn |
| T3-3 | each object held by the correct arm |

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

**So T1-1 cannot be satisfied.** The highest slab that costs nothing is
exactly where the table already is.

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
