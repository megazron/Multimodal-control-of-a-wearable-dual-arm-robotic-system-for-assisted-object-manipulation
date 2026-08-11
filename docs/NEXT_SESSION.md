# RESUME POINT — wiring DONE. Mode 06 recorded 32 files and ALL FOUR FAILED.

## THE STATE, PLAINLY

**No mode is complete.** `recordings/verification/06_full_autonomy/` holds 32
mp4s (4 tasks x 8 angles) and **every task failed the sweep's own check**:

    FAIL  run exited 1; no scene_events.json -- the scene never ran

`find recordings/verification -name scene_events.json` returns **0**. The
clips render, but nothing has confirmed an object was ever in them, so none
of them may be cited. Do not treat the 32 files as a recorded mode.

## DIAGNOSE THIS FIRST

`clip_scene.py --task t1` never wrote its events file. Two candidates, and
they are distinguishable:

  1. **`run_abc.py` exited 1** — the sweep says so ("run exited 1"). Check the
     GUI job log for the launched button; the runner may be failing before
     the scene matters, in which case the scene not running is a SYMPTOM.
  2. **the scene itself failed** — my `_items()` tables import
     `msc_clip_tasks` and `task3` *inside* the function. If that import
     raises inside the node, the timer callback dies quietly.

Run it directly and read the error:

```
python3 scripts/sim_session.py -- \
  "timeout 45 python3 scripts/clip_scene.py --task t1 --out /tmp/se.json; echo X=$?"
```

**Note for t0 specifically:** T0 has NO objects, so `_items()` returns `{}`
and `tick()` returns early by design — which means it will *never* write a
scene_events.json. The sweep must not require one for a task with no objects,
or T0 can never pass. That is a sweep fix, not a scene fix.

## WHAT IS DONE AND PUSHED

  * **wiring complete**: `run_experiment.sh` m0-m3, `run_abc.py --taskset
    msc`, 20 GUI specs `msc_<m0-m3>_<mode>`, `record_abc_sweep.py --taskset`.
    All four dry-run correctly: m0 25 waypoints, m1 113, m2 14, m3 38.
  * **clip paths verified**: 103 distinct waypoints, N=10, both arms, bench in
    scene, 0 failures, 5264 IK calls.
  * **the MSc names collide with an archived refusal.** run_experiment.sh
    refuses t1..t9 BY NAME as the 300/310 mm set. The MSc tasks are therefore
    keyed **m0-m3** in the dispatcher and displayed T0-T3. Do not "tidy" this.

## BUGS FIXED THIS SESSION, all found by running

  * the GUI key was built from the taskset key (`msc_t0_...`) and matched no
    button; it now uses the dispatcher key (`msc_m0_...`).
  * `run_one`'s four error paths returned a 2-tuple where the caller unpacks
    5, so the FIRST real failure surfaced as `ValueError: not enough values to
    unpack` with the actual reason nowhere on screen.
  * `gui_launch_specs`' accepted-key whitelist was `[te]\d|[abc]` and silently
    disabled all 20 MSc buttons with a message that was WRONG.
  * `sim_session.kill_stack()` counted ZOMBIES as survivors and refused to
    start on a machine that was clean.

## CLEAN UP BEFORE RE-RECORDING — the sweep LEAKS ONE Xvfb PER ANGLE

Found after the failed mode-06 run: **8 orphaned Xvfb servers** (one per
angle) and **8 stale `/tmp/.X9*-lock` files**, left behind because the sweep
failed before its teardown. They were killed by explicit PID and the locks
removed.

This matters for the next run, not just for tidiness: Xvfb refuses a display
number whose lock file exists, so the leak is self-worsening — 8 per mode
across 5 modes is 40 dead servers and 40 held display numbers, and the
failure it produces is a recording that renders black on a display that was
never actually created. Black frames are exactly what the verifier's known
blind spot (globbing only `rviz_front`) let through once before.

So before any re-record:

```
pgrep -x Xvfb                      # expect none
ls /tmp/.X9*-lock                  # expect none
```

and kill/remove by EXPLICIT PID and path if not — never a broad pattern, which
has killed the working shell three times in this project.

Worth fixing properly in `record_rviz.py` / `record_abc_sweep.py`: the Xvfb
teardown should run on the FAILURE path too, not only on success.

## STILL NOT DONE

  * part 3 (re-verify by looking) — nothing to look at yet
  * part 4 (the report)
  * modes 01, 02, 03, 04 — not attempted

# RESUME POINT — 2026-08-11 (third session)

## ⚠ PHYSICALLY CHECK THIS BEFORE ANY PARTICIPANT SEES THE RIG ⚠

**T3's circuit box is at x = −0.540. That puts it about 0.41 m from the
wearer's shoulder, AT THEIR SIDE rather than in front of them.**

The geometry is sound and measured: 0.41 m is well inside a seated arm's
reach, and |x| ≥ 0.51 is the *only* band where a pinned wrist can hold the box
at all — 37 of 156 candidate hold poses, all right-arm, all outboard of 0.51.
There is no inboard alternative.

**What is NOT measured, and cannot be, is whether a person can comfortably
probe a circuit board at their own side while their hands are occupied and
they are wearing a 17 kg pack.** That is a working-posture judgement and it
needs a body in the rig, not a solver.

Check it before the first session. If it is not workable, the finding is that
**no box position satisfies both the pinned-wrist reach and a natural working
posture** — which is a result about the platform, and a publishable one, not a
layout failure to be engineered around.

---

## WHERE PART 5 STANDS

DONE this session:
  * the coupling-vs-role distinction recorded in BOTH task specs, so the two
    claims cannot be merged in the write-up (`bimanual_kind` on each)
  * the 15 superseded clips archived to
    `archive/recordings/verification_20260811/` with GEOMETRY_NOTE.md naming
    both reasons: T2's band was inside the bench slab, and T3's box has moved
    190 mm outboard

# RESUME POINT — 2026-08-11 (second session), MSc experiment build

**Parts 3 and 4 are DONE, committed and pushed (`23c3a25`). Part 5 —
recording — is NOT STARTED.** Nothing is half-built: no capture script has
been modified, no clip archived.

## WHERE TO PICK UP: PART 5, THE RECORDING SWEEP

    part 5  record 4 tasks x 5 modes, 7 angles + quad     <-- START HERE
    part 6  re-verify the recordings BY LOOKING
    part 7  the report

### What part 5 has to do, and everything already known about it

1. **ARCHIVE FIRST**, with a geometry note. `recordings/verification/` holds
   the 15 clips of the A/B/C set. They are recorded against **the old T2 band
   (z 1.10-1.30), which is now known to be inside the bench**, so they are
   superseded geometry, not merely old. Move them to
   `archive/recordings/verification_20260811/` with a `GEOMETRY_NOTE.md`
   saying exactly that.
2. **Xvfb :99, never WSLg's :0.** `x11grab` on `:0` records BLACK -- measured,
   mean pixel value 0.0 with RViz plainly visible, because WSLg composites in
   Wayland and the pixels never reach the X root window. Xvfb has no
   compositor. `LIBGL_ALWAYS_SOFTWARE=1` is required; llvmpipe reports GL 4.5.
3. **`scripts/record_abc_sweep.py` is the existing sweep** (3 tasks x 5
   modes). It needs a 4th task and re-pointing at the MSc set. EXTEND it --
   do not write a second sweep.
4. **ONE MODE OWNS THE GRAPH AT A TIME.** Measured: after a VR run, modes
   01/04/06 all recorded 0.0000 m of travel until `vr_pose_mapper` was killed.
   `mode_adapters.conflicting_modes()` now encodes which pairs collide; use it
   rather than remembering.
5. **Preconditions, enforced not remembered**: stray `robot_state_publisher`
   owning `/robot_description` (a CAD URDF with no ros2_control tag kills the
   whole stack and every joint reads a plausible 0.000), more than one
   `master_pose_node`, zero or duplicate `move_group`. `procscan` and
   `preconditions()` already do this.
6. **Motion-gated capture start**, captions burnt in: mode, task, scenario,
   expected, result. `burn_caption()` in `record_abc_sweep.py` exists.

### Part 6 -- what the verifier must do, and its known blind spots

* It must **catch its constructed broken clips before printing anything**.
* It must check **EVERY angle**. It once globbed only `rviz_front` and seven
  black gripper views passed.
* Then **extract frames and look**: object between the finger pads, arm clear
  of the wearer, nothing through the bench, gripper opening and closing at the
  right moments, motion continuous, object visibly placed.
* Detection thresholds must be calibrated on **RENDERED** colour, not the RGB
  set on the marker -- RViz shades everything. And the **HUD text is detected
  as the object** unless the top 24% of the frame is ignored (measured: 587
  "ball" pixels of which 529 were letters).

## WHAT PARTS 3 AND 4 SETTLED

### Blocker A -- the free-space check was broken, and hid more than T2

Three independent lines, bench in scene:

  * ARITHMETIC: two declared T2 grip poses lie INSIDE the bench slab.
  * PER WAYPOINT, N=10, both assignments: 4+9+8 = **21 failures**, exactly
    what the regression block reported.
  * CLEAR BAND: feasible from z=1.28 (y=0.35) and z=1.30 (y=0.38).

**T2 re-spec'd to z 1.32-1.40** (+20 mm margin). Tilt threshold unaffected --
6.8 deg is 60 mm over the SPAN, not the height.

`verify_abc_scenarios` now applies furniture to the STUDY tasks and carries a
**control that must fail**: a pose inside the bench slab must be unreachable.

**IT ALSO EXPOSED TWO LEGACY FAILURES, recorded and NOT fixed:** with the
bench in, the legacy **Task A** scores 1/3 and 0/3 targets with 59 transit
failures (its targets at z=1.05 are under the slab) and **Task C** 5/26 and
0/26 shell directions (its shell reaches z=1.07). Those belong to the A/B/C
set the MSc four supersede. If anyone revives them, that is the first thing to
fix.

### Blocker B -- my check was wrong, and the right answer moved the box

Asked at the near-edge pose, both arms, N=10: the pinned wrist can PLACE ONTO
and PROBE the box at x=-0.35 but cannot HOLD it. A sweep of 156 hold poses
found 37 a pinned wrist can reach, **all right-arm, all |x| >= 0.51**.

**Box moved to (-0.540, 0.170), held by the RIGHT arm; meter presented by the
LEFT at (0.340, 0.190).** The wearer's shoulder is ~0.41 m away -- inside a
seated reach, but at their SIDE. **Whether that is a workable working posture
is an experimenter judgement and must be checked with a person before the
first session.** If it is not, the finding is that no position satisfies both
the pinned-wrist reach and a natural posture, which is a platform result.

## VERIFIED STATE OF THE FOUR MSc TASKS

`python3 scripts/verify_msc_tasks.py --repeats 10` -- N=10, full densified
paths, bench in scene, both arm assignments, refuses on tested==0 and on any
failed control:

    T0  0 sphere failures, 0 transit failures, 120 sampled poses 0 unreachable
    T1  6 items 0 failures, 8 transports 0 failures
    T2  S1/S2/S3 all 0 failures
    T3  box 13 waypoints 0, meter 13 waypoints 0
    4234 IK calls, TOTAL FAILURES 0, all four controls correct

BUILT: `task_actions.py`, `mode_adapters.py` (+ the 7-test mode-independence
proof), `task0.py`, `task3.py`, T1's layout, T2's re-spec. 128 tests pass.

## STILL UNVERIFIED, NAMED

* **No clip has been recorded against any of this geometry.** Every existing
  clip predates the T2 re-spec and the T3 box move.
* T3's wearer posture (above) -- a judgement, not a measurement.
* Legacy Task A and Task C fail with the bench in scene.
* Nothing in this repository has ever run against a real arm.
* Every figure is IK feasibility in simulation against a mock that echoes
  commands. Mock and physics figures must never share a column.

---

# RESUME POINT — 2026-08-11, MSc experiment build

**Stopped at a clean boundary: parts 1 and 2 of the build brief are DONE and
committed. Parts 3-7 are NOT STARTED.** Nothing is half-built.

## WHERE TO PICK UP

    part 3  build T2 (coordinated carry), then T3     <-- START HERE
    part 4  verify every task N=10, both arm assignments, bench in scene
    part 5  record 4 tasks x 5 modes, 7 angles + quad, Xvfb :99
    part 6  re-verify the recordings BY LOOKING, frame by frame
    part 7  the report

But **two blockers surfaced in part 2 and must be settled before part 4**, see
"WHAT THE FIXED CHECK FOUND" below.

## PART 1 — T1 IS SETTLED: OPTION 4, and the layout is verified

Options were tested in the order the brief set. Both of the preferred ones
fail, and the failures are informative.

**Option 2 (reduce standoff/lift) — cannot even be tested.** At the current
bench edge FIXED has zero supported cells on the GRASP POSE alone, so there is
nothing for a smaller standoff to rescue. With a support rail at y=0.19 the
grasp pose is *still* zero — the rail occupies exactly the volume the fingers
enter. `--sweep-standoff` REFUSED rather than reporting a ladder with no rungs.

**Option 1 (bench edge forward) — dead by deduction, stated as such.** A bench
edge at 0.19 is the same obstruction 40 mm thick where the rail was 15 mm,
with the same top face at z=1.10. Strictly worse than the rail, which gave 0.
NOT separately measured; if anyone doubts the deduction, measure it.

Rail sweep, bench in scene, full densified path:

    rail edge   FIXED cells   TOPDOWN cells   6-item layout
    none            0             18          no (18564 combos)
    0.230           0             19          no (27132)
    0.210           0             20          no (38760)
    0.190           0             22          no (74613)
    0.170        CONTROL FAILED -- Task A's own pick became unreachable

**OPTION 4 TAKEN, and it FITS.** Objects fixtured rather than resting:

    FIXED    45 cells, layout FITS at N=10 over the full path
    TOPDOWN  94 cells, layout FITS

    VERIFIED T1 LAYOUT (left arm, pinned wrist, bench in scene, N=10):
      cubes   (0.280, 0.230)  (0.340, 0.230)  (0.380, 0.170)  (0.400, 0.230)
      planes  (0.300, 0.130)  (0.460, 0.130)

**WHAT OPTION 4 COSTS, plainly: a cube that cannot fall cannot be dropped, so
`drops` stops being a measurable outcome for T1.** Grasp success, placement
success and wrong-colour placement all survive; `drops` does not. It is
recorded in the result JSON as `cost_of_option_4`, not left implicit.

## PART 2 — REGRESSION BLOCK FIXED, and it found two real problems

It forced left=+x / right=-x at k=1. Now it tries BOTH arm assignments at
N=10, the way verify_abc_scenarios does. Re-baselined:

    T0 spheres (8)                    0 unreachable        OK
    T2 carry waypoints, N=10          21 FAILURES          <-- BLOCKER
    T3 circuit_box                    reachable by NEITHER ARM  <-- BLOCKER
    T3 multimeter                     reachable by left only

The old 37/2 figures were the broken check, confirmed against a no-rail
baseline of exactly 37 and 2.

### BLOCKER A — T2's carry has 21 waypoint failures WITH THE BENCH IN SCENE

verify_abc_scenarios reports 0 failures for the same path, because it checks
the study tasks in FREE SPACE and applies furniture only to the clip tasks.
So T2's carry band has never been checked against the bench it is carried
over. **Settle this before building T2** — it may need the band raised, or it
may be my check asking at the wrong z.

### BLOCKER B — T3's circuit box is reachable by NEITHER arm

Caveat before acting: my check asks for `ee_for(BOX_OBJ)`, the wrist pose for
grasping the box at its centre. T3 may never grasp it there — clip_tasks has
B_PLACE and C_PROBE_ON, which are different poses. **Check what T3 actually
commands before concluding the box is unreachable.** This is exactly the
"asked the wrong question" shape that has bitten twice already.

## WHAT IS BUILT AND PROVEN (parts of the earlier build order)

    1/5  task_actions.py + mode_adapters.py + the mode-independence PROOF
         7 tests. All five modes produce identical task traces; the proof
         refuses a vacuous pass, requires the five adapter logs to DIFFER,
         and catches a deliberately mode-leaking task layer.
         FOUND: refusal is only expressible in 3 of 5 modes -- a refusal
         count is structurally zero in two cells and must not be compared
         across all five.
    2/5  T0 -- seeded sampler, region-bounded, RegionExhausted rather than
         silent degradation. 240 sampled poses over 40 seeds, 0 unreachable,
         N=10, bench in scene. 9 unit tests. min separation 0.06 m set from a
         measured acceptance curve.

## STANDING TRAPS RE-CONFIRMED THIS SESSION

* **Stale /dev/shm/fastrtps_\* degrade discovery.** Two verification runs died
  on the Solver's absent-data guard with /joint_states present but not
  arriving. Clear with the stack STOPPED.
* **The planning scene outlives the process.** `clip_scene.remove_furniture()`
  only removes ITS OWN ids, so a `support_rail` added by another script
  survives it and the next run fails its own control for an unrelated reason.
  That happened twice. `verify_t1_layout.py` now removes what it adds, on
  every exit path including the refusals.
* **A number with no control means nothing.** The rail "damaging" T2 and T3
  was the check, not the rail.

## UNVERIFIED, NAMED

* T2's carry against the bench (blocker A).
* T3's real grasp poses (blocker B).
* Nothing in this repository has ever run against a real arm.
* All figures here are IK feasibility in simulation, against a mock that
  echoes commands. Mock and physics figures must never share a column.

---

# RE-RECORD 2026-08-10 — STOPPED AT 3.5 OF 5 MODES, AND WHY

Old clips moved to `archive/recordings/verification_20260810/` with
`GEOMETRY_NOTE.md`. New clips in `recordings/verification/<mode>/<TASK>/<scen>/`,
8 angles each, against the verified geometry.

| mode | A | B | C |
| --- | --- | --- | --- |
| 06_full_autonomy | OK, placed 0 mm | OK | OK |
| 01_master_teleop | OK, placed 0 mm | OK | OK |
| 03_shared_autonomy | OK, placed 0 mm | OK | OK |
| 02_vr_teleop | OK, placed 0 mm | **FAIL** no grasp | **FAIL** no grasp, exit 1 |
| 04_vr_shared | **NOT RUN** | | |

Stack evidence over the recorded runs: **54 of 54 IK windows at 100%**, mount
guard PASS at **0.1610 m** worst clearance (floor 0.15), **0 clearance-floor
blocks**. The one block was a 0.38 s `flip_reject` that self-cleared.

## THE OPEN ITEM — VR grasps, and the two possibilities

Task A grasps and places at 0 mm on the VR path, so the transport works. B and
C do not grasp at all. From the frames, B's right arm travels to the part and
stops beside it; the part never moves.

**Not yet separable, and this decides the fix:**

  (a) the VR path lands the pads **more than 30 mm** from the object, so
      `clip_scene`'s new proximity gate correctly refuses to call it a grasp; or
  (b) **30 mm is too tight** for this path's tracking error.

**The measurement that settles it: log the pad-to-object distance at closest
approach.** `clip_scene._grip(arm)` already computes the pad position in world
and `it["pos"]` is the object, so it is a `math.dist` and one field in the
event dump. Do that before touching either the gate or the mapper — changing
one on a guess is how a threshold gets tuned to hide a real offset.

C is a THIRD failure (exit 1 — the runner itself errored) and C uses the LEFT
arm, the arm task A grasped with successfully, so "the right arm's VR mapping"
does not explain both.

## Two grasp bugs the re-record found, both fixed

1. `gripper_state.holding(knuckle, width_mm)` had **only a lower bound** — the
   no-width branch bounds both ends, the width branch had lost the upper. A
   gripper closed on nothing (the mock boots at 0.7929 rad) satisfied it for
   every object, so task A logged GRASPED at t=0.0 with the arm at home.
   A regression of a failure CLAUDE.md already records from the pilot.
2. `clip_scene` had **no proximity test at all** — any closure anywhere counted
   as grasping the object. `record_rviz` gates on `near_pick`; the file the
   sweep judges completion from did not.

## Preconditions are enforced now, not remembered

`preconditions()` refuses to record, and re-checks before every mode, on: a
stray `robot_state_publisher` owning `/robot_description`, more than one
`master_pose_node`, and zero or duplicate `move_group`.

The isolation check asks WHICH nodes publish, not how many — a count was
measuring `master_pose_node`'s respawn cycle, not isolation.

---

# DETECTOR ACCURACY -- THE LAB MEASUREMENT (added 2026-08-10)

**This is the only route that measures OUR objects in OUR lighting. Everything
else -- constructed clouds, public datasets -- measures something adjacent.**

WHAT IS ALREADY KNOWN, so you are not re-measuring it:

  * The POSE accuracy is NOT the detector's job. Fitting the object's KNOWN
    DIMENSIONS to the cropped depth gives 1.2 mm (40 mm block), 1.8 mm (45 mm
    part), 8.0 mm (50 mm multimeter) against a constructed cloud, and it is
    insensitive to a sloppy detection box (1.0-1.2 mm at 0-50% contamination).
  * Taking the CENTROID of the visible points instead is wrong by 13-45 mm.
    Do not do that, and do not let a library do it for you.
  * Range barely matters: 1.2 mm at 0.10 m and at 0.35 m. A close-range look
    is NOT justified by depth precision.

SO WHAT THE LAB SESSION MUST MEASURE IS THE DETECTION RATE AND THE BOX, not
the pose:

```
1.  Mark 9 positions on the bench with a rule: 3 x 3, 100 mm spacing,
    centred on the task band. Record each position to +/-1 mm.
2.  For each object (40 mm block, 45 mm part, 50 mm multimeter) and each
    position:
      ros2 run srl_perception vlm_object_locator          # detector running
      ros2 topic echo /perception/objects --once          # one detection
    Capture the RGB and the depth frame alongside, so the run can be redone
    offline without re-occupying the lab.
3.  30 frames per (object, position) -- 9 x 3 x 30 = 810 frames. At 5 s each
    that is about 70 minutes including handling.
4.  Report, per object:
      DETECTION RATE      fraction of frames with a detection at all
                          <95% BLOCKS A PARTICIPANT SESSION
      BOX IoU             against a hand-labelled box on 30 sampled frames
      POSE ERROR          known-dimension fit vs the marked position, RMS
      SIGMA               the std of that error -- this is the number the
                          scene fingerprint needs at <= 2 mm
5.  Repeat the whole thing under the session's actual lighting. Detection is
    lighting-sensitive in a way geometry is not.
```

**TUNE THE PROMPTS FIRST -- it is the largest single effect measured.**
Wording alone changed confident detections per frame by 3x on real images,
against 7.1 objects actually visible:

    specific phrases ("hole puncher")      2.1 / frame
    generic single nouns ("box", "can")    6.4 / frame   <- DEFAULT
    one phrase for the scene               1.0 / frame

The system defaults to GENERIC SINGLE NOUNS, one per expected object class.
Prefer recall and filter afterwards: over-detection is handled by the
confidence threshold, the fingerprint's association gate and the depth
segmentation, while a missed detection cannot be recovered. Before the 810
frames, spend ten minutes trying 3-4 wordings per object and keep the best --
the wording that suits "can" may not suit "multimeter".

**A detection rate under 95% is a stop.** It does not matter how good the
fit is: the fit only runs on frames where something was detected.

**If the rate is good but sigma is above 2 mm**, the fingerprint's
"has the scene changed" decision is what fails, not the grasp. In that case
fall back to (b) -- objects at marked positions, vision confirming presence
only -- and state in the write-up that goal poses are known a priori, because
that weakens the intent-inference claim and must not be left implicit.

---

# LAB PLACEMENT REQUIREMENT: +/- 10 mm

**Every object must be placed within 10 mm of its registered position.**

THE LIMIT IS THE GRIPPER'S CAPTURE WINDOW, NOT IK. Measured: every grasp pose
survives +/-20 mm of displacement in IK terms -- all six axis directions, N=10,
with the furniture in the planning scene. What fails is the fingers closing
beside the object. Capture half-windows, from the measured jaw map:

    40 mm block        22.5 mm
    45 mm part         20.0 mm
    50 mm multimeter   17.5 mm   <- the binding one

At 20 mm the block is still caught, the part sits exactly on its limit, and
the multimeter is MISSED. So the requirement is set by the widest object,
not by the arm, and it is 10 mm.

`pos_tol` in the fingerprint store was tightened 15 mm -> 10 mm to match
(2026-08-10). At 15 mm an object could be declared UNCHANGED and then not be
grasped -- the tolerance was looser than the thing it protects. Tightening is
free on the AprilTag path (0.00% false-changed at both) and requires it: the
colour/shape fallback was already 77% false-changed at 15 mm. To hold
false-changed under 1% at 10 mm a detector needs sigma <= 2 mm; AprilTag
measures 0.74 mm at 0.35 m.

---

# SCENE CALIBRATION -- WHAT TO RUN, IN WHAT ORDER (added 2026-08-10)

**Before any data-collection session. Sim-verified; the arm-driving step of
the sweep does not exist yet, so today this is a partly manual procedure and
step 4 is where it stops being automatic.**

```
1.  bash scripts/check_channels.sh          # ~3 min. FIRST, ALWAYS.
                                            # 12+/14 coherent or degraded mode
                                            # engages and the master is unusable
2.  bash scripts/run_teleop.sh gate:=false  # one stack, and only one
3.  ros2 run srl_teleop preflight --participant
                                            # FAILS on: empty planning scene,
                                            # wearer missing from TF, blockers
                                            # active or EXPIRED, e-stop latched
4.  python3 scripts/clip_scene.py --task <a|b|c>
                                            # applies the bench/bin/box and
                                            # CONFIRMS them in move_group:
                                            # "7 collision objects applied,
                                            #  7 confirmed"
                                            # If it says fewer, STOP -- the arm
                                            # will plan through the furniture.
5.  ros2 run srl_perception scene_fingerprint_node
    ros2 topic echo /scene/state --once     # unchanged -> calibration skipped
                                            # moved     -> only that object is
                                            #              re-registered
    # THE ARMS DO NOT SWEEP THEMSELVES YET. Until they do, put each object in
    # view of a wrist camera by hand, or drive the arms with the GUI, before
    # trusting the fingerprint.
6.  ros2 service call /scene/resweep std_srvs/srv/Trigger
                                            # after moving anything on purpose
7.  python3 scripts/verify_abc_scenarios.py --repeats 10
                                            # 0 failures with the furniture in
8.  python3 scripts/measure_task_margin.py  # every pose >= 20 mm of margin
```

**The tolerance you are calibrating to: +/-10 mm, not +/-20 mm.** At 20 mm the
40 mm block is still caught but the 50 mm multimeter is not -- 20 mm is outside
its 17.5 mm capture half-window and the fingers close beside it. IK is fine at
either; capture is what breaks. See `docs/system/08_scene_calibration.md`.

**`pos_tol` in the fingerprint store is 15 mm, which is LOOSER than the grasp
survives on the multimeter.** An object can be declared UNCHANGED at 15 mm and
then not be grasped. Tighten it to 10 mm, or narrow that object -- but the real
detector's noise floor has to be measured first, because a tolerance below
2 x pose_noise_sd cannot be separated from noise.

**Detection rate is still UNMEASURED and under 95% blocks a participant
session.** The synthetic renderer is out of the detector's distribution (0-4%
on our primitives; 0.89-0.91 on a real photograph). Either put AprilTags on the
objects -- measured 0.74 mm at 0.35 m, comfortably inside +/-10 mm -- or
measure detection on real camera frames before scheduling anyone.

---

# PART 4 DONE (2026-08-09) -- the divergence is visible, the two panels are not

Commit `95adcf4`.

## WHAT SHIPPED

`scripts/srl_gui.py`, rebuilt around the question the brief asks: what is the
gap between what was commanded and what the arms actually did?

* **Divergence readout**, per joint and end-effector, coloured against
  `lag_trip_rad` -- **read from the running bridge**, not hardcoded. A panel
  that colours against 0.5 while the bridge runs at 0.3 says "fine" up to the
  moment the arm stops. The header names its source, and says
  "default -- bridge not read" when it could not get one.
* **Both wrist cameras**, subscribed and never opened, with NO CAMERA shown
  explicitly and a stale frame never painted.
* **Controls**: precision/speed dial (with the settings it implies shown),
  force-clutch, per-arm scale, re-base anchor, per-arm grip release, e-stop
  and e-stop reset. All parameter writes go through parameter CLIENTS with
  read-back; `ros2 param set` goes via the daemon, which hangs on this box.
* **26 launch specs**, 23 live and 3 disabled with the reason ON the button.

## THE FINDING: TWO EMBEDDED RVIZ PANELS DO NOT WORK ON THIS HOST

X11 reparenting (`QWindow.fromWinId` + `createWindowContainer`) **embeds and
positions but does not CLIP**. Measured from screenshots, not return codes:

| | |
| --- | --- |
| Qt layout underneath | **correct** -- the GUI's own geometry dump put controls at (8,49)-(478,983), commanded (487,75)-(1180,857), actual (1194,75)-(1887,857), divergence (482,868)-(1892,983) |
| where rviz2 actually painted | **(0,0) at 1595x995** -- over the banner, the cameras, every indicator and the divergence readout |
| with two instances | they paint over each other as well |

Positioning the foreign window in top-level coordinates instead of (0,0) moved
it and raised the second panel's content from 36 to 292 distinct colours, so
the handle is real -- but no amount of positioning produces clipping. Clipping
a reparented client is the **window manager's** job and **no window manager is
installed** (twelve checked; `sudo` needs a password so none can be added).

**Shipped instead: one RViz with the COMMANDED arm solid and the ACTUAL arm
translucent in the same scene** (`TF Prefix: real_`, alpha 0.45). This is the
fallback the brief asked to be proposed, and it is the better answer: the
operator sees the GAP in one 3-D view instead of comparing two viewports by
eye, and it costs 248 MB less. `--embed-rviz --dual-rviz` opts back in.

## `/real/tf` DOES NOT EXIST

The brief specifies the actual arms come from "/real/joint_states and
/real/tf". The first exists; the second does not. Checked against a running
mock stack, the only `real` topics are `/real/joint_states`, the two
controller topics, `/real_status_*` and `/realmock/robot_description`.

Both real launches run `robot_state_publisher` with `namespace="real"` and
`frame_prefix="real_"`, so the real arm's transforms are in the **SHARED /tf**
under prefixed frames joined to `world` by a static transform -- one tree with
two robots, not two trees. Subscribing to `/real/tf` would have produced a
permanently empty ACTUAL panel that looked exactly like an embedding failure.

## NEGATIVE CONTROLS, WHICH IS WHAT THIS PART WAS REALLY ABOUT

**The indicator self-test.** Three synthetic snapshots -- GOOD, BAD, ABSENT --
are pushed through the SAME `refresh()` the live data uses, and every
indicator must render healthy differently from NOT CHECKED, and healthy
differently from abnormal. Compared on (value, colour, NOTE): dropping the
note was the first version's mistake, because "--" purple "NO ATTEMPTS --
follower up, no poses in" and "--" purple "no ik_status published" differ only
there, and that difference is the whole point.

**Result: 17/17.** Two indicators legitimately treat absence AS the
abnormality (the detectors have no other way to be wrong), and that is
reported rather than asserted away.

**It found two real bugs on its first run**, neither of which reading would
have caught:

1. **`"ENGAGED" in "DISENGAGED"` is TRUE.** The clutch indicator reported a
   disengaged clutch as engaged. Same shape as the `/blocking` substring match
   that reported every registered blocker as active. Now word-boundary
   matched, DISENGAGED tested first.
2. **The mode indicator had no abnormal state at all** -- it could not be
   shown to be reading anything. It now reports CONFLICT when more than one
   source claims the arm, which is a real fault by this project's own
   one-source-at-a-time rule.

**Divergence carries the discipline in its type.** Seven statuses and only
`OK` carries a number: a real arm that is not publishing reads **NO REAL ARM**,
not 0.000 rad. `REAL_FROZEN` is separate again, because `/real/joint_states`
publishes a CACHE at full rate when the hardware component goes inactive, so
arrival-rate freshness cannot see it -- the Side tracker keeps arrival and
content-change apart for exactly that.

**Buttons: 55/55**, in three levels, because one check cannot cover it
honestly. Level 1 RUNS the dispatcher for all 13 tasks with `--dry-run` and
requires exit 0 -- the direct regression for the five buttons that exited 2
while appearing to launch. Level 2 constructs the GUI offscreen and presses
every button, requiring an observable outcome from each (silence fails, and so
does a traceback), with launching intercepted so the check cannot start
stacks. Level 3 requires every refusal to name a reason.

**Tasks A/B/C are DISABLED, deliberately.** They have a verified spec and no
runner. Pointing them at t3/t6/t7 would run the superseded 300/310 mm span and
log it under a 500 mm name -- a button that launches something DIFFERENT from
what it claims is worse than one that refuses.

## MEASURED

| configuration | peak RSS | processes |
| --- | --- | --- |
| indicators only (`--no-rviz`) | **182 MB** | 2 |
| default: one RViz + ghost | **506 MB** | 3 |
| `--embed-rviz --dual-rviz` | **754 MB** | 4 |

Frame time with the cameras subscribed and the divergence live, READ FROM THE
GUI'S OWN STATUS BAR in the screenshots:

    indicators only   frame 18.64 ms   median 2.41   p95 18.64   max 35.05
    default + ghost   frame  4.38 ms   median 2.88   p95 24.81   max 66.22

against a **100 ms budget**, so roughly 3% of it at the median and two thirds
at the worst sample. Stated honestly: the harness's programmatic capture of
that line came back EMPTY (the GUI's stdout was not captured through the
subprocess pipe), so these are read from the status bar rather than parsed.
That is a gap in the harness, not in the numbers -- the status bar is what the
operator sees -- but it should be closed before anyone regresses on frame time.

No OOM. The machine reported 7.8-11 GB available throughout, so the two
earlier OOM kills were not this stack.

The ghost is proved DIFFERENTIALLY -- the same scene rendered with the ghost
display on and off differs by **36,651 pixels**. "It looks right" is not a
check; if the ghost were absent the difference would be zero, which is the
outcome that had to be reportable.

## STILL OPEN

* Two embedded panels need a window manager on the host. Not a code fix.
* The GUI has never driven a REAL Kortex session; everything above is sim plus
  the mock real stack.
* Part 2's "NOT FIXED, AND WHY" list is unchanged.

---

# PART 3 DONE (2026-08-09) — four items, and three of them were worse than reported

Commits `c0ff406` (3.1), `759dfe6` (3.2), `c51ee0f` (3.3), `d5bbec8` (3.4).

## 3.1 LANGUAGE AND VISION SWEEP — 7 MISUNDERSTOOD, all seven closed

40 phrasings, 13 categories, written against the parser's STRUCTURE rather
than alongside it. That distinction is the whole reason the sweep was worth
running: `measure_freeform_language.py` already reported **0 MISUNDERSTOOD**
over 30 phrases, and every one of those phrases was written with the grammar
it tests. Zero is the EXPECTED output of a self-test, not evidence of safety.

    before   10 CORRECT   7 ASKED  16 REFUSED   7 MISUNDERSTOOD
    after    10 CORRECT   6 ASKED  24 REFUSED   0 MISUNDERSTOOD

"before" is the same 40 cases with the fix stubbed out, so the rows differ by
the fix and nothing else. **CORRECT did not fall**, so no capability was
traded for the safety.

**All seven shared one shape**: the grammar matched a real verb and a real
noun and discarded the word that reversed or qualified them.

| n | category | said | would have done |
| --- | --- | --- | --- |
| 4 | negation | "don't grab the blue cube" | grabbed the blue cube |
| 1 | relational | "the cube to the left of the red block" | grabbed the red cube — grounded to the LANDMARK |
| 1 | conjunction | "the blue cube AND the green ball" | grabbed one, silently |
| 1 | sequence | "pick up the red block THEN put it down" | parsed as PLACE — it would have opened the hand |

**Negation is scoped by POSITION, and that is load-bearing.** Only a negation
BEFORE the verb negates the command; "grab the blue cube but not the red one"
is an exclusion clause and still grabs the blue cube. A keyword search would
have thrown that away for no safety at all.

**Verbs stay exact, and the reason is measured.** Fuzzy-matching the closed
verb set at Damerau-Levenshtein 1 makes **"top" a STOP**, "crop" a DROP, and
let/yet/net/bet all a GET or SET. So a misspelled VERB refuses outright while
a misspelled NOUN degrades to a question — the module's closed-verb/open-noun
principle, now visible in the data (verb 3/3 REFUSED; noun 1 ASKED, 2 CORRECT).

**The harness has a negative control.** The headline is a count of zero, and
zero is also what a harness prints when it has stopped grading. `self_test()`
swaps in a deliberately broken parser and REFUSES TO PRINT A RESULT unless all
four negation cases come back MISUNDERSTOOD.

## 3.2 GRIPPER — two real bugs found by RUNNING it

The 2F-85 holds position when its commanding process dies, so a node killed
mid-grasp leaves the fingers part-closed and the next start cannot tell a grip
from a leftover. Neither default is safe: opening drops the object, never
opening leaves the travel range wrong all session.

**The marker is written when the GRIP IS TAKEN, not when the process exits.**
`kill -9` runs no exit code — atexit, signal handlers and `finally:` are all
equally useless against it — so the claim "I may be holding something" has to
already be on disk before the kill lands. Recovery then needs BOTH halves:
marker + knuckle in the holding band means still holding; marker + `free_air`
means the object is gone and the marker is stale. The knuckle alone cannot do
it (a hand closed on nothing looks closed) and the marker alone cannot (it
outlives its object).

**An inherited grip cannot be released by a resting pad.** After a crash the
operator's hand is by definition off the pad, so the ordinary 0.5 s low
release would have fired and dropped the object half a second after recovery.

Two bugs neither reading nor unit tests would have found:

1. **The shutdown opened the hand, then re-closed it.** Setpoints after
   SIGTERM were `[0.0, 0.166]` — spinning to flush the publish fired `tick()`
   once more, which re-read a pad the operator was still touching. Two writers
   on one gripper in one cycle, the same collision as the 2026-08-08 scripted
   grasp vs frac schedule.
2. **The startup verdict was re-decided every tick**, so it read the hand
   WHILE IT WAS MOVING UNDER ITS OWN OPEN COMMAND. A gripper correctly found
   at 0.800 rad (free_air, safe to open) was re-judged 0.2 s later at 0.55 rad
   — mid-ramp, inside the holding band — and concluded it was gripping
   something. It latched onto its own open.

**16/16 by actually SIGKILLing the node mid-grasp** against a fake 2F-85 that
stops on the object and OUTLIVES the node. Two harness bugs were caught first:
the rig published nothing while waiting for the node to die (same shape as the
four teensy_disconnect runs), and `free_air` is unreachable at a 2500-count
grasp because that is not a full squeeze — **a real limit now recorded: a
PARTIAL squeeze on an empty hand is indistinguishable from a grip.**

## 3.3 DEGRADED MODE — the repair could not take effect, and the warning had never fired

Neither defect was "the warning is too quiet".

**1. The runtime read a different file from the one the lab writes.**
`check_channels.sh` saves `channels_<today>.json` and diffs against
`ls -t channels_*.json | head -1`. `default_baseline_path()` returned the
literal string `channels_20260806.json`. A lab session could repair every pot,
re-capture, watch the script confirm 14/14 — and `master_pose_node` would go
on reading the 08-06 file and freezing eight working channels for ever.

**2. The Job A mitigation had never fired once.** `dg.staleness_warning(baseline)`
— `baseline` is not a name in that scope — inside a bare
`except Exception: _stale = None`. Every run raised NameError and swallowed
it. Confirmed by running the node: no `[CHANNELS]` line existed at all. Job A
verified the FUNCTION and never its CALL SITE.

Fixed: newest-by-mtime with a test asserting the shell script and the runtime
pick the SAME file; warning moved INSIDE the banner (a separate log call is a
thing that can go missing separately, which is exactly what happened), boxed,
naming the file and its age, and re-issued every 30 s. Three analysis tools
were reading the same hardcoded date, including the thesis channel-health
FIGURE.

**Verified end to end**: dropping a newer 14/14 baseline in and launching the
real node now gives "full channel set", 14/14 coherent, all seven channels in
use on both arms. Before this that file was ignored entirely.

`check_channels.sh` is documented as the FIRST ACTION OF EVERY LAB SESSION in
CLAUDE.md, README.md, docs/system/02_bringup.md (step 0a) and the lab list
above, where it is now step 1 — before the soldering, because it is both the
reference every repair is diffed against and the only thing that makes a
repair visible to the software.

## 3.4 TASKS A, B, C — verified N=10 over the full path, 116-minute session

`src/srl_experiments/experiments/abc/`. Five tasks to three, both merges
already justified by recorded findings rather than by the clock: T3/T4 were
never two mechanisms, and T2 carried no comparative measure at all (no DIRECT,
no VR, by geometry).

    A  POSITIONING        uncoupled control; attention contrast; Fitts
    B  COORDINATED CARRY  physical coupling, rigid AND compliant
    C  DUAL PURSUIT       simultaneity across the disjoint sets; b_cross

**2743 IK calls, 0 failures**, both arms at 0.0000 rad from home. Task A also
verifies every TRANSIT between targets — segments the spec does not name but
the operator traverses. Task B checks both grippers over the densified path
AND the sling arithmetic (sag 102.0 mm vs a 40 mm ball; s_max 534.0 mm
matching the declared threshold to 0.1 mm, 34 mm of margin). Task C checks the
26-direction amplitude shell plus the radial interior.

**Instrument controls, because the headline is a zero.** A pose past the
0.902 m reach and a pose inside the wearer must both come back unreachable and
a declared target must come back reachable; the script refuses to print a
count if any control is wrong. All three correct.

**The session is data plus a checker**, and the checker found a real error on
its first run: Task A at 3 repeats is 9.1 min against blocks of 8 and 7. **The
spec was changed, not the check** — A now runs 2 repeats, and the cost is
stated where it lands (per-participant Fitts throughput is thin at 2 repeats
per ID, so the regression is a GROUP-level fit and may not be reported per
participant).

    116 min against a 120 cap
    pack-on 36 min total (cap 48), 9 min continuous (cap 12)

Task A is BENCH-mounted, which is what makes it fit — its claim is about
pointing, not about wearing. A test asserts the WEARER stays the binding
constraint rather than the clock, so if that flips someone notices.

## STILL OPEN AFTER PART 3

* Part 2's "NOT FIXED, AND WHY" list is unchanged: five wall-clock intervals,
  four identity-quaternion drive sites, two audit blind spots, D2.
* **Nothing in Tasks A/B/C has been driven by a human through the master arm.**
  Every coordinate is IK feasibility in simulation, and the DIRECT condition
  of all three depends on master channels that are currently incoherent.
* The optional pick-and-place block remains non-comparative by geometry.
* `test_flake8` / `test_pep257` still fail, as they did before this work.

---

# JOB A DONE (2026-08-09) - and the pot repair does NOT take effect yet

## THE HEADLINE: degraded mode will STILL ENGAGE after the repair

`degraded_mode.decide()` reads a **stored baseline file**, not live hardware.
`recordings/baselines/channels_20260806.json` still says **6/14 coherent**, so
`master_pose_node` will engage degraded mode at startup and **freeze eight
now-working channels**, silently.

    stale baseline    6/14 coherent  ->  degraded engages: True
    repaired arm     14/14 coherent  ->  degraded engages: False   (verified)

**Nothing in Job B's capability list is actually available until that baseline
is re-captured**, which needs the lab: `bash scripts/check_channels.sh`.
Until then the software behaves exactly as it did with broken pots.

This is the "state left from a previous run" instrument mechanism, in the one
place where it silently discards a hardware repair.

**Mitigation shipped:** `staleness_warning()` fires on EVERY engage, naming
the file and its age. Age was tried as the trigger first and was WRONG: the
file is 3.1 days old, which is recent, and its content is still completely
wrong. A re-capture would also reset the age and hide the problem it caused.
Verified both directions: warns on the stale baseline, silent on a healthy one.

## Audit totals
    A wall-clock interval        0   (the 1 hit was the ARCHIVED prior work,
                                      now excluded: it is a historical record)
    B block() with no clear()    0
    C unprefixed C++ resource    3   all VENDOR, already patch targets
    D can start a second stack   0
    E1 entry point unresolved    0
    F1 missing referenced path   0
    F2 task arg runner rejects   0
    H check passes on no data    0   (was 4, fixed last session)
    E2 module with main() unregistered  3
    G defined and never called   248

E2: `boundary_feedback_node` (deliberately withdrawn) and
`kortex_highlevel_bridge` (runs via its own venv) are correct as they stand.
**`analyse_sensing` remains a genuine omission.**

## JOBS B-G NOT STARTED
Job B's first question (does a working j7 make orientation_mode:tilt viable)
is pure sim and IS answerable without the lab. The rest of Job B needs a fresh
master capture.

---

# C4 PART TWO (2026-08-09) - CAD render works; ONE RVIZ CLAIM WAS WRONG

## CORRECTION to the previous C4 commit
I wrote that the RViz capture recipe was "one command per view". **It is not.**
`shoot.sh` takes yaw/pitch/distance arguments and IGNORES them: the camera
pose comes from the `.rviz` config file, so every invocation produces the SAME
view. Verified by md5 -- two "different" captures were byte-identical.

Getting the other views needs a per-view `.rviz` config with the camera pose
written into it, which is what `record_rviz.py:ensure_display()` already does
and what `shoot.sh` should reuse. That is the fix; it was not attempted here.

## CAD RENDERING WORKS, and it is vector
`scripts/render_cad_figures.py` projects the B-rep through cadquery's SVG
exporter: resolution-independent, no GL context, no display, no compositor.
That last matters on this machine, where x11grab on a composited window
records black.

`potarm_iso.svg` is rendered and inspected. It shows the repeating joint
module clearly -- three modules and the base -- which **visually corroborates
the 79.0 mm module pitch measured in C2**.

**It is SLOW.** Hidden-line projection over a 400-solid assembly takes several
minutes per view, and the remaining views were still rendering. Zero-byte
partial files are deleted rather than committed.

## STILL TO DO
* remaining CAD views (front/side/top for both parts) -- just run the script
  and let it finish;
* the annotated CAD figures: pot/IMU/FSR placement, link-length diagram,
  exploded view, mount with bases. These need annotation on top of the
  renders, like the RViz figure;
* the other RViz stills, after fixing `shoot.sh` to write a per-view config.

---

# C5 DONE - the CAD is not the kinematic source (2026-08-09)

83 pages, 0 errors, 0 undefined references.

New section in the master arm chapter, `\label{sec:cad-not-kinematic}`. Three
statements, each CHECKED rather than asserted:

1. **The CAD carries no articulated joints.** Both STEP files declare AP214,
   a schema that CAN carry kinematics. Searching for every entity that would
   express it (`KINEMATIC_JOINT`, `REVOLUTE_PAIR`, `PRISMATIC_PAIR`,
   `KINEMATIC_LINK`, `MECHANISM`) returns **zero** in both.
2. **No robot description derives from them.** No URDF or xacro references any
   `.step` file. Stronger: **the master arm has NO URDF at all** -- it is not
   modelled as a robot description in any form.
3. **The kinematics are seven measured constants**, `LINK_LENGTHS` in
   `master_calibration.py`, originating in the prior work's C++ header as
   "shaft-center to shaft-center, measured directly on the physical master
   arm".

The section also states why the 1.2% CAD agreement does NOT make the CAD the
source: it was established by measuring the CAD *against* the constants, and
if the two ever disagree the question must be settled on the physical arm.
And it records that regenerating kinematics from the CAD is impossible anyway
-- the geometry gives pair pitch and cannot separate the 43 mm roll from the
37 mm bend.

## JOB C REMAINDER
Only C4's outstanding figures are left: more RViz stills (recipe works, one
command per view) and the CAD figures (cadquery installed, can render).

---

# C4 PARTLY DONE (2026-08-09)

82 pages, 0 errors, 0 undefined references.

## DONE
* **The first real RViz figure in the report.** Captured from the running
  system on Xvfb, cropped, and annotated with arrows: left/right arm, gripper,
  backpack mount, and the wearer marked as being IN the collision model. It
  sits in the workspace chapter and shows the disjoint-workspace result rather
  than only asserting it. `docs/img/rviz/home_pose_labelled.png`.
  Two arrow tips were wrong on the first pass -- the "right arm" arrow pointed
  at the torso -- and were corrected against the rendered image.
* **Full participant-Results chapter skeleton**,
  `thesis_report/results_skeleton/participant_results.tex`. Every planned
  figure and table as a labelled placeholder naming the analysis script that
  produces it: completion time by mode, coordination error traces, cross-arm
  interference, NASA-TLX, Borg CR10, trust, embodiment, proprioceptive drift,
  grasp success, and demographics.
  Three constraints are built into the captions so they cannot be lost:
  T2 carries NO mode comparison; TLX and Borg are split by ROLE because only
  the wearer carries 17 kg; and embodiment is decomposed rather than scored,
  because ownership and agency move independently.

## STILL TO DO IN C4
* More RViz stills: task scenes, reachable volume, collision model close-up,
  coordinate frames, and one per control mode in operation. The capture
  recipe works and is one command per view
  (`scratchpad/shoot.sh <display> <name>`); it is repetition, not difficulty.
* **CAD figures: none yet.** Isometric/front/side/top of the master arm with
  dimensions, exploded view, the backpack mount with bases, the pot/IMU/FSR
  placement diagram, and the link-length diagram. cadquery is installed and
  can render, so these are now feasible.

## C5 NOT STARTED
State in the report that the CAD has no articulated joints, that no URDF
derives from it, and that the working URDFs come from measured link lengths.

---

# C3 DONE - prior work recovered and read (2026-08-09)

Source archived at `thesis_report/_source/prior_work/`. The development
chapter is no longer reconstructed: it now cites the actual programs, and
file dates give the calendar directly (7 June to 27 July 2026).

## What the prior work actually was
Five programs import MuJoCo. **None contains an inverse-kinematics call of any
kind** -- searching the whole set for "inverse" or "jacobian" returns nothing.
The mapping is a literal joint-space assignment:
`data.qpos[k1_qpos[j]] = k1_target[j]`. So the phase was joint-space
throughout, and the later move to a task-space command is a change of KIND.

## TWO CORRECTIONS to what the chapter previously claimed

**1. The prior system ALREADY drove the real arm.** `srl_teleop.py` (35 KB,
the largest) imports the Kinova API directly and calls
`SendJointSpeedsCommand` behind a `USE_REAL` gate -- the SAME high-level
velocity interface the delivered system uses, arrived at independently about
two months earlier. **The move to ROS 2 was therefore NOT driven by needing
vendor drivers**, which is what an earlier draft supposed. It was driven by
the collision model, the solver and the controller lifecycle. Narrower claim,
and the honest one.

**2. The FK link lengths have a documented origin.** They were not first
written in Python. `basic_control/kinematics.h` carries them as C++ constants:

    // Physical link lengths (meters), shaft-center to shaft-center, measured
    // directly on the physical master arm.
    L1 0.043  L2 0.037  L3 0.043  L4 0.037  L5 0.043  L6 0.036  L7 0.033
    // J1 roll, J2 bend, J3 roll, J4 bend, J5 roll, J6 bend, J7 roll

**This settles what the C2 CAD comparison can claim.** "Shaft-centre to
shaft-centre" IS the joint-module pitch measured off the STEP, so the two are
commensurable and their 1.2% agreement is a genuine cross-check rather than
similar-looking numbers. It also explains why 43 and 37 cannot be separated
from the CAD: neither the header nor the geometry distinguishes them, only
their sum.

76 pages, 0 errors.

## STILL TO DO: C4, C5

---

# C2 FINISHED (2026-08-09)

## Library: cadquery, not pythonocc-core
**pythonocc-core is not on PyPI at all** -- it ships through conda, and this
workspace has no conda. `cadquery` bundles OCP, a binding to the SAME
OpenCascade kernel, and installs with pip. Identical kernel, different
packaging. For overall bounding extents no kernel is needed at all: STEP is
text and `scripts/measure_cad_step.py` does it with a regex.

## Parts and dimensions

    PotArm.step      75 solids, 15 structural   extent 113.6 x  94.3 x 363.2 mm
    backpack.step   418 solids                  extent 161.2 x 176.6 x  69.5 mm
    total solid volume, arm: 60.1 cm^3

**MASS NOT REPORTED.** The STEP carries no density. Volume times an assumed
density is an invented number.

## LINK LENGTHS: THE CAD AGREES WITH THE FK, to 1.2%

The arm decomposes into a REPEATING JOINT MODULE. Filtering to structural
solids (>= 2000 mm^3, so the ~400 screws and pins are excluded), the large
25.0 x 40.0 x 45.5 mm bodies sit at z = 49.8, 128.8, 207.8 mm:

    CAD joint-module pitch      79.0, 79.0 mm      (perfectly regular)
    FK roll+bend pair sums      80, 80, 79 mm      (43+37, 43+37, 43+36)
    difference                  -1.0 mm per pair, -2.0 mm over two

**This validates the measured FK link lengths against the designed geometry
for the first time.** The seven FK values were measured off the built arm and
had never been checked against the CAD. They agree to 1.2%.

**What it does NOT do**: separate 43 from 37 within a pair. Centre-to-centre
spacing gives the PAIR pitch, not the individual links, so the alternating
roll/bend split remains measured-only.

The earlier 91.2 mm "difference" (CAD 363.2 vs FK 272.0) is now explained and
was correctly withheld: the arm's Z extent includes the base boss and the tip
fixture, neither of which is a kinematic link.

## THE MOUNT ANGLE: NOT AVAILABLE, and this is the honest answer

`backpack.step` is 418 solids and **nothing in the STEP labels which are the
arm mounting faces.** With no label there is no placement to read, so the URDF
rpy (71.5, -13.3, 86.4 deg, solved analytically and never checked against the
bracket) CANNOT be verified from this file.

Picking solids by size and deriving an angle from them would be a guess
dressed as a measurement. **The mount check remains open** -- it needs either
a named/coloured mounting face in the CAD, or a physical measurement of the
bracket in the lab.

## STILL TO DO: C3, C4, C5

---

# JOB C: C1 DONE, C2 STARTED, C3/C4/C5 NOT STARTED (2026-08-09)

## C1 /mnt/c: WORKS. No action needed.
It recovered on its own; `ls /mnt/c` and the project directory both read
fine. The mount entry is the same 9p one that was returning EIO earlier in
the session, so that was transient. No Windows-side command required.

Prior work confirmed present at `/mnt/c/Users/Gausms/Desktop/MSc_Project`:
`mujoco_menagerie`, `MUJOCO_LOG.TXT`, `singlekinova.py`, `srl_teleop.py`,
`testjoints.py`, `com_read.py`, `basic_control/`, `3dprint/`, `BOM/`.

## C2 CAD: FIRST MEASUREMENT TAKEN, no library needed

**Recommendation: no CAD library.** STEP is a TEXT format and every vertex is
a `CARTESIAN_POINT` with literal coordinates, so a bounding box needs a regex
rather than a kernel. cadquery and pythonocc-core are each a large install
with a compiled OCCT dependency, and neither is needed to answer "how big is
this part". `scripts/measure_cad_step.py` does it with the standard library.

Reach for pythonocc-core ONLY for what the regex genuinely cannot do: mass
from material properties, the assembly tree, and the pose of a subpart inside
an assembly. Those are exactly what C2's mount-angle question needs, so that
install is still coming.

    PotArm.step     20804 points, mm   extent X 113.61  Y 94.28  Z 363.15
    backpack.step   39958 points, mm   extent X 161.24  Y 176.58  Z 69.50

**FIRST COMPARISON, and it is NOT yet a finding.** The FK chain sums to
272.0 mm over seven links; the CAD arm's Z extent is 363.2 mm, a difference of
91.2 mm. That is plausibly the base boss plus the tip fixture, neither of
which is a kinematic link. **It cannot be resolved without decomposing the
STEP into per-link solids, which needs the kernel.** Do not report 91 mm as a
disagreement until the parts are separated.

## STILL TO DO IN JOB C
* **C2 the mount geometry** -- the actual question. Where the arm bases attach
  to `backpack.step`, at what angle, against the URDF mount rpy that was
  solved analytically and NEVER measured against the bracket. Needs
  pythonocc-core for the assembly poses.
* **C2 per-link lengths** vs the FK values. Needs the same.
* **C3** read the MuJoCo and prior programs for the development-path section.
* **C4** RViz screenshots, CAD figures, and the Results chapter skeleton.
* **C5** state in the report that the CAD has no articulated joints and that
  no URDF derives from it.

---

# VACUOUS CHECKS FIXED + POST-HOC WIRED (2026-08-09)

## The four vacuous checks: 0 remaining
`verify_final5.py` now RAISES if densify returns no waypoints, rather than
reporting "TOTAL FAILURES: 0" on a path it never checked. Same guard on the
place path, on `verify_scenarios`'s centres dict (all() over an empty dict is
True) and on `verify_autonomy`'s expected-words list (an empty expectation
passes anything the robot says, including nothing).

## holm: the gap was not a missing call
`holm()` had NOTHING TO CORRECT. Each analyser ran one omnibus test and
stopped; an omnibus says something differs, not WHICH PAIR. The missing piece
was the post-hoc stage holm was written for.

`srl_experiments/posthoc.py` now provides it once: paired Wilcoxon over all
pairs, Holm-corrected, reporting RAW and ADJUSTED p side by side plus a
rank-biserial effect size. A pair with too few observations is excluded from
the family rather than counted, because including it would weaken every other
comparison on the strength of a test that never ran. `holm([])` now raises.
Known-answer checked: [0.01,0.04,0.03] -> [0.03,0.06,0.06], monotone.
The five duplicate copies now delegate to it.

## WORSE THAN THE AUDIT REPORTED, and this is the finding
**Three of the five E-series analysers run NO inferential test at all.** They
define `friedman_or_rm()` AND `holm()` and call neither:

    analyse_divided_attention   omnibus calls 0
    analyse_dof_recovery        omnibus calls 0
    analyse_intent_inference    omnibus calls 0
    analyse_autonomy_level      omnibus 1, post-hoc 1   (wired)
    analyse_vr_vs_mannequin     omnibus 1, post-hoc 1   (wired)

Those three now END with a loud DESCRIPTIVE ONLY warning. I did NOT wire the
omnibus in blind: the correct test depends on each experiment's design and on
which series pair with which, and guessing that is how a wrong p-value enters
a thesis. **That wiring is a real outstanding task, not a formality.**

67 tests pass.

## JOB C: NOT STARTED

---

# JOB B DONE - workspace audit. REPORT ONLY, nothing fixed (2026-08-09)

`scripts/audit_workspace.py` -- repeatable, 232 python files scanned. Each
check exists because reading did NOT catch that bug in this project.

## CLEAN: the five recurring bug classes are essentially gone

    A  wall-clock used as an interval          0    the conversion held
    B  block() with no clear() anywhere        0
    D  can start a second stack                0
    E1 entry point does not resolve            0
    F1 referenced path does not exist          0
    F2 task arg the runner rejects             0    (the class that made five
                                                     GUI buttons do nothing)

C  unprefixed resource in C++: **3, all VENDOR** -- `reactivate_gripper` in
robotiq_driver and `reset_fault` twice in kortex_driver. All upstream files
already addressed through `patches/`, so these are the patch TARGETS, not
unpatched defects. Worth re-checking after any vendor bump.

## OUTSTANDING, and NOT fixed because you asked for a report first

**H  a check that can pass on NO DATA: 4.** The most serious class, because it
is how a known-answer test passed on zero parsed rows.

    scripts/verify_final5.py:67   good  = all(ok(arm, w) for w in path)
    scripts/verify_final5.py:70   goodb = all(ok(arm, w) for w in place)
    scripts/verify_scenarios.py:165  ok7 = all(centres.values())
    scripts/verify_autonomy.py:117  ok_words = all(...)

An empty `path` makes `good` True. **The five-task verification reporting
"0 failures" would be indistinguishable from it having checked nothing.**
That is the headline finding of this audit.

**E2  module with main() not registered: 3.** Two are correct as they stand:
`boundary_feedback_node` was deliberately withdrawn, and
`kortex_highlevel_bridge` runs through its own venv interpreter via
ExecuteProcess rather than as an entry point. **`analyse_sensing` looks like a
genuine omission** -- it has a main() and nothing runs it.

**G  defined and never called: 253.** Verified by sampling, not assumed: three
picked at random (`holm`, `prim_distance`, `elapsed_columns_present`) each have
**0 non-definition mentions anywhere**. `holm` is a multiple-comparison
correction defined identically in five analysers and called in none, so every
E-series analyser reports uncorrected p-values. That one is a RESULT-AFFECTING
finding, not tidiness.

## FOUR FALSE POSITIVES IN MY OWN AUDIT, found and fixed before reporting
1. The audit scanned ITSELF; every check contains its own pattern as a string.
2. Dead-code detection counted only `name(` calls, so a function in a dispatch
   list read as dead. 293 -> 253 after counting mentions.
3. A comment and a docstring example counted as "can start a second stack".
4. A path quoted at the end of a sentence carried the full stop, so
   `verify_scenarios.py.` read as missing.

## JOB C: NOT STARTED

---

# JOB A DONE - recordings restructured by control mode (2026-08-09)

    recordings/verification/<mode>/<task>/<scenario>/<condition>/

Six numbered mode dirs (01_master_teleop .. 06_full_autonomy), created and
**empty**, plus two honestly-named buckets. All 125 existing clips migrated
with `git mv`, none orphaned.

## THE FINDING THAT SHAPED THE MIGRATION
**No clip in the repository was recorded through a control mode.** Every one
was produced by `record_rviz.py`, which calls `/compute_ik` DIRECTLY and never
publishes `/master_arm_pose_*` -- no follower, clutch, anchor or orientation
lock in the path. Verified two ways: no clip's metadata carries a mode field,
and the recorder has no publisher for the pose topic. So the mode axis is
EMPTY, and clips went to:

    00_unclassified_legacy_geometry    87 clips, retired nine-task geometry
    00_unclassified_scripted_playback  38 clips, current geometry, still scripted

Calling the f-clips "legacy geometry" would have been a false label; giving
either a mode would have been a guess.

## FIVE SILENT BREAKAGES THE EXTRA PATH LEVEL CAUSED, all found by running
1. `verify_gripper_motion` reported **0 grasp clips** -- the glob was fixed
   but the path PARSING was not, and 0-of-0 reads exactly like 0-of-N.
2. Its `GRASP_TASKS` list carried only the RETIRED names (t2,t3,t5,t6), so
   every current f-clip classified as "not a grasp clip". Now covers both
   naming generations.
3. `make_clip_index` globs `summary.json` only, so it dropped all 38
   screen-capture clips and **rewrote INDEX.md with the 87 retired ones,
   discarding the Task 2 caveat**. Now reads both record types.
4. Merging two record shapes then produced four successive KeyErrors, each
   patched field-by-field until I normalised once at load instead.
5. That normaliser defaulted `ee_travel_m` to 0.0, which classified all 38
   screen-capture clips as **"no motion"** -- absence read as a value, in the
   patch written to prevent exactly that. Unmeasured fields now print `--`.

**The Task 2 caveat is now GENERATED by `make_clip_index.py`**, not
hand-written, because a hand-written warning was silently overwritten on the
next run.

Verified after: 39/39 clips pass, 65 grasp clips found (was 0), 125 runs
indexed, 0 false "no motion".

## JOBS B and C: NOT STARTED

---

# LAB SESSION: DO THESE, IN THIS ORDER

Everything below needs hardware. Nothing above this line does. The order is
not arbitrary: each step's result changes whether the next one is worth doing.

---

## 0. BEFORE YOU TOUCH ANYTHING: exactly one stack

Two `master_pose_node` instances split the serial stream and invalidated a
full day of measurements. It is now impossible at the file-descriptor level,
but confirm the refusal never appears:

```bash
pgrep -fc "lib/srl_teleop/master_pose_node"     # must be 0 or 1, never 2
ls /dev/shm | grep -c fastrtps                  # if in the hundreds, see below
```

Stale shared-memory segments from killed stacks silently kill NEWLY LAUNCHED
nodes: empty logs, no processes, no error at all. This cost hours on 08-09.
Clear them **with the stack stopped**:

```bash
rm -f /dev/shm/fastrtps_*
```

Kill by explicit PID, never `pkill -f <pattern>` -- the pattern matches the
shell running it, which killed the working shell three times in one session.

---

## 1. RE-CAPTURE THE CHANNEL BASELINE -- FIRST, AND AGAIN AT THE END

```bash
bash scripts/check_channels.sh          # ~3 min, block A only
```

Do this BEFORE the soldering iron comes out, not only after. Two reasons: it
is the reference every later repair is diffed against, and it is the only
thing that makes a repair visible to the software at all.

`master_pose_node` decides which channels to freeze from the NEWEST
`recordings/baselines/channels_*.json`. Until 2026-08-09 it read a hardcoded
`channels_20260806.json` instead, so a re-capture could not take effect at
all -- every pot could be repaired, the script could confirm 14/14, and the
runtime would go on freezing eight working channels for ever. That is fixed,
and the runtime now selects the same file the script does (`ls -t`).

Save the new reference at the end of the session; the script prints the exact
command. **Target: 12 or more of 14 coherent**, where degraded mode stops
engaging.

## 2. REPAIR LEFT j2 AND j4. Nothing else on the master is worth soldering.

**These two, and only these two.** Two independent routes agree, which is why
this is stated flatly rather than as a suggestion:

* the observability regression: with j1 and j7 alone the left arm retains
  **no reach observable at all**, $R^2 = 0.133$ with a residual that is
  \SI{93}{\percent} of the true spread. The right arm needs no help
  ($R^2 = 0.986$).
* `capability.regain()`, which was never told the above, independently names
  **j2 and j4 as WORTH IT** and j3, j5 and j6 as **no change**.

Fixing a roll buys nothing while both bends are dead, because a roll carries
no radial information. That is why the health table alone cannot tell you what
to repair, and the regain map can.

Expected gain: the left arm moves from rung **SPH_RATE** (radius driven as a
rate off the wrist, cost \SI{0.076}{\metre} mean) back to **SPHERICAL**
(radius measured, cost \SI{0.000}{\metre}).

## 3. Re-test AFTER EACH ATTEMPT, not once at the end

```bash
bash scripts/check_channels.sh          # ~3 min, block A only
```

It diffs automatically against `recordings/baselines/channels_20260806.json`.
**Target: 12 or more of 14 coherent**, which is where `degraded_mode:=auto`
stops engaging.

Run it after *each* attempt. A repair that breaks a working channel should be
caught immediately, not discovered at the end of the session when you cannot
tell which attempt did it.

Watch for the verdict **INCOHERENT** specifically. A failing pot does not go
quiet: it returns a wide spread of values that are not a trajectory, and range
alone calls that healthy. That is how a broken channel keeps getting trusted.

## 4. ONE 35-minute recapture, gate fix live

```bash
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash run_teleop_capture.sh
```

The 2026-08-06 capture lost **41 of 42 directional segments** to a clutch
gating bug and latched the e-stop through the whole of block F. Both are
fixed, but confirm from the recorder's own output rather than assuming:

* each segment must record with the clutch **ENGAGED** on the arm under test
  (each arm is gated by the OPPOSITE arm's button, because an arm's own button
  also toggles its clutch);
* the e-stop must not latch. The both-button trigger now needs a sustained
  \SI{0.30}{\second} hold, so gating taps can no longer fire it.

This capture is what unblocks: the gyro-azimuth decision, any smoothing
parameter, and the first real gain matrix with a **moving** master. Every
tracking figure in the thesis was taken with the master at rest.

## 5. Verify the cascade rate limit end to end

`clamp_towards` now cascades `max_step` as well as `max_vel`. It is
unit-tested and has **never been measured with a stack up**.

```bash
bash scripts/start_real.sh --mock
# then vary the trip threshold and watch the divergence
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.3
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.5
ros2 param set /sim_to_real_bridge_left lag_trip_rad 0.8
```

**EXPECTED: divergence stays roughly CONSTANT as `lag_trip_rad` changes.**
If it SCALES with the threshold, the fix is wrong: that would mean the sim is
still running ahead and only the trip point moved, which is the bug the
cascade was meant to remove.

Record the actual numbers either way. This is a prediction with a clear
falsifier, which is the only kind worth testing.

## 6. Unblock VR: platform-tools on WINDOWS

```
https://dl.google.com/android/repository/platform-tools-latest-windows.zip
```

Unzip to `C:\platform-tools`. No install, no admin rights. Verify from WSL:

```bash
/mnt/c/platform-tools/adb.exe devices
```

`unauthorized` instead of `device` means the **Allow USB debugging** prompt
has not been accepted, and that prompt appears **inside the headset** -- put
it on to answer it. This traps everyone.

It must be the **Windows** build: the headset enumerates as a Windows USB
device, so `apt install adb` inside WSL gives a binary that cannot see it.

Then the whole connection is one command, which also supervises the tunnel:

```bash
bash scripts/vr_connect.sh
```

## 7. While you are there, two cheap measurements

* **Read the right arm's home joint angles from hardware.** They have NEVER
  been read (`config/home_positions_right.txt` says so), and $P_{\text{HOME}}$
  for that arm, the workspace anchor and every clearance figure depend on
  them.
* **Photograph the task objects**, 25 images each, for the detection measure.
  The current detection figure characterises a renderer, not a detector, so
  the \SI{95}{\percent} gate is neither passed nor failed.

---

# ITEM 3 - THESIS: instrument-validation chapter written

The master arm chapter (2799 words), the MuJoCo-to-ROS development path
(1631), and all four control-mode flowcharts ALREADY EXISTED from the earlier
rewrite. Checked before writing rather than duplicated.

The genuinely missing piece was the methods contribution, now written:
`thesis_report/methods/instrument.tex`, Chapter 6, "When the instrument is the
fault". Sixteen cases in one table (looked like / actually was / caught by),
four mechanisms, and the finding that **9 of 16 were caught by an
independently known quantity and only 3 by inspection** -- which is the
argument for building the cross-check in rather than trying harder.

76 pages, 0 errors, 0 undefined references.

### Thesis work still outstanding
* The Results chapter predates Parts 1-9 and does NOT yet carry: the aperture
  curve validated against the vendor stroke, the capability ladder costs, the
  scene-fingerprint numbers, the broken-then-fixed VR path, the free-form
  language results, or the 39/39 clip sweep.
* Placeholders: 5 images, 2 reproduced figures needing permission.
* 13 todo markers (10 CONFIRM, 2 MEASURE, 1 FIGURE).

---

# ITEM 2 - WRIST CAMERA RELAY: LOGIC MEASURED, GUI WIDGET UNCONFIRMED

## Measured and working
`srl_teleop/camera_relay.py` holds the staleness rule, shared by the GUI and
the VR publisher so the two cannot drift apart. Against a synthetic 15 Hz
camera on both arms:

    measured rate      14.2 Hz  (reported from ARRIVALS, not configured)
    transport latency  median 2.7 / 3.1 ms, p95 7.6 ms  (header stamp -> receipt)
    after the source stops:  live -> NO SIGNAL, paint disabled

**It degrades visibly by construction**: past `stale_after_s` the widget is
told NOT to paint at all, so a frozen frame can never be shown as live. That
is the failure class this project keeps meeting (frozen /real/joint_states
reading as "holding", the dead pot reading as "steady").

`camera_vr_publisher` relays the same streams as JPEG on
`/vr_camera_<arm>/compressed` with state on `/vr_camera_state`, rate-capped,
and publishes NO FRAME when the channel is not live.

## CONFIRMED FROM PIXELS (2026-08-09)
Both states screenshotted and kept in `docs/img/`:

* **live**: both feeds painting, labelled LEFT / RIGHT, captions
  `live 13.8 Hz 320x240` in green -- `gui_cameras_live.png`
* **source stopped**: both panels black with **NO SIGNAL** in red and
  `last frame 5.9 s ago` -- `gui_cameras_no_signal.png`

The capture failed before because the embedded RViz is a separate top-level X
window until it is reparented, and during that window it covers the host.
`--no-rviz` exists for capture and headless checks.

Two defects fixed on the way: the panel was LAST in the column and fell below
the fold on a 950 px display, which for a remote operator's only view of the
workspace is a real defect rather than a cosmetic one; and two different
things were both called "camera" and disagreed on screen -- one tracks the
IMAGE topic, the other the DETECTOR topic. The second is now "detector".

Also fixed on the way: `srl_gui.py` referenced `cap` and `cr` with NO IMPORT
AT ALL. It survived only because the branch dereferencing `cap` never ran with
no data. It would have crashed the moment a capability message arrived.

---

# ITEM 1 COMPLETE - 2026-08-09 overnight run

## ALL 38 CLIPS RE-RECORDED, 39/39 PASS THE VALIDATED VERIFIER

Five-task geometry (500 mm tray, 540 mm sling), single-owner gripper fix,
7 angles per clip, 266 files, 132 MB, largest file 5.3 MB.
`recordings/verification/INDEX.md` carries the Task 2 caveat at the TOP.

### THE VERIFIER WAS MISCALIBRATED FOUR TIMES. Every one caught BEFORE any
### clip was reported as failed.
1. `block_orange` fired 495 px on a frame with no orange object: RViz lighting
   pulls the tan tray into the orange band. Fixed with R-B > 155.
2. `tray_tan` fires 220 px and `ball_yellow` 21 px on the WEARER'S SKIN. At the
   shared 12 px floor every f3/f4 clip would have passed whether or not the
   object rendered.
3. Per-object floors calibrated on 800 px captures, while `frames()` rescales
   every sample to **600 px** -- wrong by (600/800)^2 = 0.56, which rejected
   nine sling clips whose object I had already confirmed by eye.
4. My own expectation was wrong twice, not the detector: the f2 scene
   legitimately renders a tan table.

### A REAL STALE-GEOMETRY BUG, which is what the re-record existed to find
`record_rviz.SLING_L` was still the retired 350 mm while the spec is 540 mm.
At the respec'd 500 mm separation a 350 mm sling is GEOMETRICALLY IMPOSSIBLE
((L/2)^2 - (s/2)^2 = -0.032), so it rendered as a flat line with the ball
instantly fallen, and all nine f4 clips failed correctly. The length now comes
from the scenario. Confirmed from pixels afterwards: `sag=102mm`, matching the
predicted 0.102 m, with the ball retained in the V.

### KNOWN AND ON SCREEN
f3/f4 show a CARRY, not a grasp: `GRASP REFUSED: pre-grasp standoff not
solvable`. Correct -- the tray edge sits at |x| = 0.25 m and top-down grasping
is infeasible below 0.30 m (0/9 against 9/9). The refusal is in the overlay.

## ITEMS 2, 3, 4: NOT STARTED
2 wrist camera relay, 3 thesis sections, 4 lab list.

---

# STOPPED HERE - 2026-08-09 overnight run

## ITEM 1 RE-RECORD CLIPS: INFRASTRUCTURE DONE + VERIFIER VALIDATED.
## THE SWEEP ITSELF HAS NOT BEEN RUN. 4 of 38 clips exist.

### What is finished and committed
* `scripts/final5_runs.py` builds the sweep from the CURRENT five-task spec:
  **14 runs, 38 clips, 266 files at 7 angles**. Output under `f1..f5` so it
  cannot collide with the 87 stale nine-task clips.
* `record_rviz.py --final5 --resume`: per-run conditions (f2 is shared-only),
  progress written to `recordings/verification/sweep_progress.json` AFTER
  EVERY CLIP, and `--resume` skips what is already done.
* **The Task 2 caveat is burned into the overlay in red** and confirmed
  visible in a captured frame: "DIRECT-IK: not operator-commandable. Grasp
  needs 169.7/164.6 deg from the pinned wrist anchor."
* Overlay label bug fixed: clips were labelling themselves by the RENDERER
  code, so an f3 clip said "T3". A clip that misnames itself looks
  authoritative; it now shows the run's own name.

### THE VERIFIER WAS MISCALIBRATED IN TWO WAYS. Both caught BEFORE any clip
### was reported as failed, which is what the brief required.
1. `block_orange` fired **495 px on an f3 frame with no orange object** -- RViz
   lighting brings the tan tray into the orange band. Now requires R-B > 155,
   calibrated against a confirmed positive AND a confirmed negative.
2. Worse: **`tray_tan` fires 220 px and `ball_yellow` 21 px on the WEARER'S
   SKIN** with no task object present. At the shared 12 px floor, every f3 and
   f4 clip would have passed whether or not the object rendered. Per-object
   floors now sit above the skin and far below the object:
   tray 220 (absent) vs 828 (present), floor 400; ball 21 vs 649, floor 120.

### TO RESUME: one command
```
python3 scripts/record_rviz.py --final5 --all --resume
```
It skips the 4 clips already recorded and writes progress after each. Then
`python3 scripts/verify_rviz_clips.py` and rebuild INDEX.md with the Task 2
caveat at the TOP.

### Known, unresolved
* **f3/f4 grasp is REFUSED**: "pre-grasp standoff not solvable". Correct and
  expected -- the tray edge sits at |x| = 0.25 and Part 1 measured top-down
  grasping infeasible below |x| = 0.30. The clips show the carry, not a grasp.
  Decide whether to widen the tray or accept a carry-without-grasp clip.
* **Repo size not yet checked**: 266 files at roughly 1.3 MB each is about
  350 MB. Check before committing the sweep; consider tracking quad+gripper
  only and keeping the rest local.

## ITEMS 2, 3, 4: NOT STARTED
2 wrist camera relay, 3 thesis sections, 4 lab list. Stopped at the item
boundary rather than half-doing item 1's sweep.

---

# SESSION 2026-08-09 - NINE-PART BRIEF, PARTS 1-2 DONE

Sim and mock only. No lab.

## PART 1 - grasp investigation: DONE, committed ae68df1
Headline: **the grasp is NOT real under teleoperation.** It is real under the
scripted and autonomous paths. 169.7 deg (left) / 164.6 deg (right) of wrist
rotation is needed from the anchor that `orientation_mode: fixed` pins to, and
no channel commands it. The clips work because the recorder calls
`/compute_ik` directly, bypassing the teleop orientation lock.

Also: the approach IS a real straight-line approach from a 100 mm standoff
(0.01 mm max deviation); t2/t5 close on their objects, **t3 and t6 stop
1.5 mm short**; precision is NOT measurable against the mock and is reported
as such; dexterity is 5 of 8 approach directions per arm, mirror-symmetric,
with no from-behind approach on either.

Three instrument bugs were found before any number was trusted. See the
commit message.

## PART 2 - dynamic degradation as architecture: DONE, this commit

`srl_teleop/capability.py` (pure, testable) + `capability_node`.

Six rungs, selected live from whatever channels pass health:

    L0 FK        7 healthy                       cost 0.000 m
    L1 SPHERICAL j1 + at least one of j2/j4      cost 0.000 m
    L2 SPH_RATE  j1 + any spare alive channel    cost 0.076 m mean / 0.114 p95
    L3 SHELL     j1 only, radius FROZEN          cost 0.151 m mean / 0.232 p95
    L4 DIR_ONLY  IMU only                        NO POSITION
    L5 NONE      IMU down                        NO POSITION

Costs are LOADED from `recordings/baselines/capability_ladder.json`, measured
on 20440 recorded frames. A rung with no measurement prints "unmeasured"
rather than a plausible number.

Verified live against a synthetic master: DIR_ONLY at startup (conservative,
"unknown" is not "healthy") -> SPHERICAL -> FK -> fell to SPH_RATE when both
bends went incoherent -> RECOVERED to SPHERICAL when j4 came back. Recovery
is automatic and needs no restart.

`scripts/verify_capability_degradation.py` sweeps all 128 channel subsets.
From the measured left-arm health it independently names **j2 and j4** as the
only repairs worth making, and j3/j5/j6 as no change, which matches the
observability regression (R^2 = 0.133 on j1+j7 alone).

18 known-answer tests.

## PART 3 - scene fingerprinting: DONE, this commit

`srl_perception/scene_fingerprint.py` (pure matcher) + `scene_fingerprint_node`.

Verified end to end against synthetic detections, four stages:

    1st sweep   FIRST FINGERPRINT: 3 objects in 0.9 s
    2nd sweep   SCENE MATCHES (max delta 0.0 mm) -- calibration SKIPPED
    3rd sweep   block_b moved 60.0 mm -> re-registered 1, kept 2 unchanged
    4th sweep   block_b vanished -> dropped 1, store now holds 2

Measured (matcher only; the DETECTOR is not measurable here):
  * decision accuracy at AprilTag noise (sigma 0.8 mm): 100% on all four of
    same / moved / appeared / vanished, 2000 trials each.
  * smallest detected displacement: 20 mm at 95% (model predicts 15 mm).
  * compare() costs 0.12 ms for 5 objects, 6.7 ms for 50. The decision is
    free; a sweep's cost is arm motion.

**The fingerprint is only useful with a tag-grade detector.** At the
colour/shape fallback's 10 mm noise the "unchanged" verdict is right 2.6% of
the time -- it would re-register every start. Widening the tolerance fixes the
false alarms and costs sensitivity:

    pos_tol   false "changed"   min detected move
     15 mm       97.3%            30 mm
     30 mm       13.0%            50 mm
     40 mm        0.8%            60 mm

## PART 4 - four-mode path verification: DONE, this commit

**THE VR COMMAND PATH WAS BROKEN AND IS NOW FIXED.** `quest_vendor_bridge` --
the transport the project adopted -- published `/vr_pose_<side>` and
`/vr_gripper_<side>`, which NOTHING in the workspace subscribes to.
`vr_pose_mapper`, the stage that turns a controller pose into
`/master_arm_pose_<arm>`, listens on `/vr/controller_pose_<side>` and
`/vr/controller_joy_<side>`. Measured on a live graph: `/vr_pose_left` had
1 publisher and 0 subscribers; `/vr/controller_pose_left` had neither. VR
could not reach an arm at all through the vendor transport.

Fixed and verified: all three hops now `pub=1 sub=1`, and the mapper publishes
`/master_arm_pose_left` as a result.

Other findings:
  * **two detectors write one topic.** apriltag_detector and
    colour_shape_detector both publish `/perception/detections/<arm>`.
  * **no camera-device contention inside `srl_*`.** Nothing opens a device;
    every consumer subscribes. The device owner is the vendor
    `ros2_kortex_vision` node; two instances of THAT would contend.
  * **the e-stop is a designed second writer** on the arm controller topics.
    The first version of the checker flagged it as contention wrongly.
  * `/blocking` had 1 publisher, 0 subscribers: `blocking_aggregator` is not
    started by `teleop.launch.py`.

NEEDS THE LAB: real Kortex sessions, the vision driver on real cameras, and
mode switching with the real cascade up.

## PART 5 - free-form language: DONE, this commit

`srl_autonomy/referring.py`. **Opens the NOUN, not the VERB.** The recorded
decision against an LLM still holds: mis-hearing a verb makes the robot do the
wrong ACTION, while mis-grounding a noun makes it do the right action to the
wrong THING, which reachability, the keep-out and confirmation all bound.

Measured on 30 phrasings, most never designed for (politeness, filler,
misspellings, superlatives, compound clauses, relational, and phrases that
sound specific while constraining nothing):

    CORRECT        17   56.7%
    ASKED           7   23.3%
    REFUSED         6   20.0%
    MISUNDERSTOOD   0    0.0%   <-- the dangerous category

Verb handling unchanged and still closed: 0 mismatches over 8 cases, with
"stop" winning even inside "grab it but stop if it slips".

TWO REAL BUGS, found by the undesigned phrasings:
  * **relational references grounded to the ANCHOR.** "the one behind the red
    block" returned the red block with full confidence -- the only dangerous
    outcome in the set. Relational phrases are now REFUSED by name.
  * **verb words leaked into the noun list**, so "grab the leftmost object"
    was scored against the word "grab" and refused.

Superlatives are DEFINITE (argmax, not graded scoring), which is semantics
rather than threshold tuning and converted four wrong ASKs into CORRECT.

Reachability is checked BEFORE announcing and names why:
"I can see the far cup, but I can't reach it: it is 1.40 m in front of me,
past my reach." Keep-out objects are filtered BEFORE grounding, so a forbidden
target is never offered as an option.

12 known-answer tests.

## PART 6 - GUI: DONE (485fad5)
Qt5, Helvetica via a user-level fontconfig alias, **RViz embedded as a panel**
by X11 reparenting (verified from pixels, 31 fps), five indicator groups, and
a frame time of 0.45 ms median / 0.76 p95 against a 100 ms budget. Three
embedding routes costed in `docs/system/07_gui_rviz_embedding.md`.

## PART 7 - VR setup and README: DONE, this commit
`scripts/vr_connect.sh` does the whole connection in one command and then
SUPERVISES the tunnel, because `adb reverse` dies with the USB connection and
opening it once leaves the headset silently unable to reach a running bridge.
Verified: with no adb it refuses with the exact install instruction and exits
non-zero.

`docs/img/vr_controls.svg` is the control diagram, drawn rather than tabulated.
Trigger = gripper, grip = clutch, thumbstick y = live scale, face buttons
deliberately unbound. README section added.

**BLOCKED ON: Android Platform-Tools, installed on the WINDOWS side.** Not
present. Nothing in this path has run against a headset.

## PART 8 - operator aids: PARTIALLY DONE, this commit
Six candidates evaluated in `docs/system/08_operator_aids.md`; four rejected
with rig-specific reasons (FSR conflicts with the gripper twice; the IMU is
the primary direction sensor and one unit spikes 72 deg/s while stationary;
snap-to-object would make the DIRECT baseline not a baseline).

BUILT: `boundary_feedback_node` -- predicts the workspace wall ahead along the
direction of travel and names which wall it is. **Live verification did NOT
complete**: /compute_ik was not being served when the test ran. The node
correctly published nothing rather than inventing a boundary, but that is not
evidence it works. FIRST JOB NEXT: one run against a healthy stack to measure
how far the warning leads the wall.

NOT BUILT: the wrist camera relay. Evaluated as the second-best candidate and
still is; it is a subscriber and a widget.

## PART 9 - audit DONE, re-recording NOT DONE, this commit

`docs/system/09_results_audit.md` audits every number in Parts 1-8: how it was
measured, whether the instrument was validated, verdict.

    VALIDATED  14      SOUND 15      UNVERIFIED 4      STRUCTURAL 1

The four UNVERIFIED, none of which is reported as a result anywhere:
end-to-end fingerprint accuracy (no cameras), smallest grippable object (the
mock cannot produce positioning error), anything against a real headset (no
adb), and the boundary warning's lead distance (below).

**BOUNDARY NODE STILL NOT VERIFIED.** It publishes and warns, but reports the
wall at 0 mm along a line a direct IK probe reaches at 6/6 with the same
orientation. Two measurements contradict; at least one instrument is wrong.
Candidates: the node probes before its cached orientation is populated (and
identity is unreachable at 0/6 on that line), or the velocity estimate is
still near zero when the first probe fires. FIRST JOB NEXT.

**CLIP RE-RECORDING NOT DONE.** The 87 clips still use the OLD nine-task
geometry and predate the single-owner gripper fix. Re-recording needs a
healthy stack for hours and was not attempted rather than attempted badly.


6 GUI with embedded RViz, 7 VR setup + README, 8 new operator features,
9 re-record + self-audit. Each is a session's work; they were not begun
rather than begun badly.

---

# SESSION IN PROGRESS — 2026-08-08 (five-task verification + grasp)

Scope of this session, deliberately narrow: finish the N=10 verification,
confirm the five-task set, finish the grasp sequence, commit. **Clip
re-recording, figures and the thesis are NOT in scope** — each has run the
context out when bundled with anything else.

## 1. N=10 full-path verification — DONE, 0 failures

Both arms verified at **0.0000 rad from home** before any sample was taken
(the script refuses otherwise, and that refusal is what invalidated the
previous attempt at 0.87/0.59 rad off home).

    task 1 positioning     left 3/3, right 3/3 targets
    task 2 pick and place  left 3/3 approaches + 3/3 places, right the same
    task 3/4 coupled carry S1 4 wp, S2 11 wp, S3 10 wp -- all VERIFIED
    task 5 dual pursuit    26/26 shell directions, both arms
    TOTAL FAILURES: 0

**No figure changed.** The result is byte-identical to the run committed in
`abd0c74` (`detail` compares equal), which is the first time a reachability
figure in this project has been reproduced across runs rather than merely
re-asserted. The reason it reproduces is that the 500 mm respec moved every
coupled waypoint off the feasibility boundary; the earlier 310 mm geometry
failed 10 of 25 and would not have.

## 2. Grasp sequence — a real defect found in the RECORDED TRACE

The previous session proved the gripper was commanded (109 samples,
0.050–0.424 rad). That was true and still insufficient: **two owners were
writing the gripper on the same cycle.** The scripted grasp called
`send_gripper()`, then `tick()` re-issued the frac schedule immediately after,
so the recorded knuckle sawed through the lift and reached **0.05 (fully
open) at the start of transit** — the object was grasped, dropped, and
re-grasped in mid-air when the schedule's fraction crossed its own threshold.

    BEFORE  lift  0.42 -> 0.275 -> 0.181 -> 0.238 -> ... -> 0.116
            task  0.05 at t=8.79   (dropped)   0.424 at t=10.44 (re-grasped)

    AFTER   lift  0.424 flat, n=18, min == max
            task  0.424 held to t=18.50, then 0.05 once, at the placement

Fixed by giving the gripper a single owner (`grip_hold`): the plan owns it
from pre-grasp to the release fraction, the schedule owns it after, so the
place still happens in one place in the code.

Second defect, same area: the marker attached on the 0.10–0.74 band, so a
40 mm block (needing 0.42) snapped to the gripper at knuckle **0.157**, while
the fingers were still visibly open. `holding()` now takes the object's width
and requires 0.90 of it.

## 3. Five-task set — confirmed, with two honest limits

All five are specified, internally consistent and verified at N=10. The
arithmetic reproduces: sling `s_max = 0.5340` matches the spec exactly, the
33.6 mm length margin is real, and `fail_tilt_deg = 6.8` is `atan(60/500)`.
Two stale figures were found and corrected — the Task 3 docstring still
described a 310 mm span, and `time_above_11_3_s` still named the tilt
threshold of that superseded span.

**LIMIT 1: there are two genuinely bimanual MECHANISMS, not three.** Tasks 3
and 4 are the same mechanism — a coupled object spanning the dead band — run
with a rigid and a compliant object. Task 5 is the second mechanism, two
simultaneous targets in the disjoint sets. That is what the geometry supports.
It is not a shortfall: rigid-versus-compliant is the scientific contrast, so
two objects on one mechanism is deliberate. But the set should not be
described as three independent bimanual demands, because it is not.

**LIMIT 2: nothing runs this spec.** `final5/tasks.py` is consumed by
`verify_final5.py` and by nothing else — no runner, no analyser, no recorder.
The five-task set is verified as a GEOMETRY and unbuilt as an EXPERIMENT. The
superseded nine-task package under `experiments/bimanual/` is what the runner
and the recorded clips still use, and it carries a different span (300/310 mm
against 500 mm) — see the trap comment in `tasks.py`.

## 4. Wrist alignment under `orientation_mode: fixed` — stated plainly

**Under direct teleoperation the wrist cannot align to a grasp, and the
grasp clips do not demonstrate that it can.**

`master_pose_node` declares `orientation_mode` default `"fixed"`: the
commanded orientation is pinned to the home anchor for the whole trial.
Measured, for the Task 2 block at (0.32, 0.35, 1.15):

    anchor quaternion (home EE, what "fixed" pins to)  (-0.0896, 0.4861, 0.8693, 0.0032)
    top-down grasp quaternion required                 ( 1.0,    0.0,    0.0,    0.0)
    WRIST ROTATION NEEDED                              169.7 deg

169.7 deg is very nearly a reversal. There is no channel to command it: the
master's wrist is unmeasurable (left j7 railed and j6 clamped, right j3/j5/j7
dead), which is *why* the mode defaults to fixed, and mode 3
(ORIENTATION_ASSIST) — which exists to supply it — is a STUB because this
`/compute_ik` plugin ignores `OrientationConstraint`.

The grasp sequence works in the recordings because the recorder is a
**scripted path that calls `/compute_ik` directly** with the grasp
quaternion, bypassing the teleop orientation lock entirely. So the clips
demonstrate the ROBOT can execute an aligned grasp; they do **not**
demonstrate an OPERATOR can command one. The three recorded conditions
(direct / assisted / shared) differ only in trajectory smoothing and all use
the same scripted wrist, so they do not distinguish this either.

This is the one place where the autonomy earns a categorical rather than a
quantitative advantage, and it should be claimed exactly that narrowly.

---

# CHECKPOINT — 2026-08-08 (updated after the reachability audit)

Repo: https://github.com/megazron/dococthefinal (`main`). Verified: a clean
clone plus the README's vendor steps builds **21 packages, 0 failures**.

## THE STANDING CAVEAT — read before trusting any number below

**Every figure in the protocols is IK feasibility in SIMULATION.** Nothing in
the bimanual programme has been driven by a human through the master arm. The
DIRECT condition of every task — the baseline the whole design rests on —
depends on channels that are currently **INCOHERENT**: 7 of 14 fail the
coherence test, and degraded mode freezes `l_j2` and `l_j4`, which costs the
left arm its entire radial dimension.

The reachability figures, the verified scenarios, the sling geometry and the
pilot all say what the ROBOT can do. None of them say what an OPERATOR can do
through this master arm today.

## WHAT WORKS — measured, not assumed

| | evidence |
| --- | --- |
| **Real dual-arm control** | two simultaneous Kortex sessions, **21.4 Hz per arm**, no halving |
| **Homing** | all 7 joints inside tolerance; residuals scatter 0.037–0.126 deg after the integral + settle fix (was 7 joints parked at the 0.99 deg deadband edge) |
| **Grippers** | on the Kinova **internal bus**, driven over the existing session — no second session opened |
| **Clutch indexing** | **unbounded**: 338.8 mm over 6 cycles, re-engage jump **mean 0.29 mm / max 0.35 mm**, drift while frozen 0.000 mm |
| **Boot reference** | latches at the master's ACTUAL pose — no calibrated neutral needed |
| **Collision avoidance** | graduated; held at 0.072–0.079 m with **zero** hard-floor blocks, and resumed on retreat |
| **E-stop** | trips on a frozen-but-publishing master in 0.94 s and a silent one in 0.97 s; halt is immediate |
| **Tracking** | up/down and fore/aft correct on both arms |
| **Autonomy pipeline** | modes 1–6, **10/10** mode-6 checks: refusals, ambiguity, wearer keep-out, spoken confirmation, voice stop in **2 ms** |
| **Console** | Dear PyGui, frame time **median 0.80 ms / p95 6.76 ms** against a 33 ms budget |
| **Experiment runner** | `run_bimanual.py` — T2/T3/T5/T6/T7 with real `--participant/--condition/--scenario` |
| **Every protocol coordinate** | re-verified at **N=10 repeats over the whole densified path**: **17/17 scenarios and 17/17 protocol-table figures SOLID**, 0 MARGINAL. `scripts/audit_scenario_reachability.py` |
| **T2 discrete outcome** | blocks placed / missed / dropped, from gripper TRANSITIONS with the opening tracked off the holding arm's live pose. 15 known-answer tests |

## WHAT IS BLOCKED, AND ON WHAT

| blocked | on |
| --- | --- |
| **Lateral tracking** | azimuth comes from j1 alone and couples with arm bend. Gyro azimuth halves the residual (25.6° → 13.5°) but was NOT shipped — three directions from one segment each is not a triad |
| **Left-arm radial motion** | `l_j2` + `l_j4` incoherent. Measured: **no reach observable survives** (R² = 0.133, residual = 93% of the true spread). The j7 rate fallback is a workaround, not a fix |
| **T4 inter-arm handover** | **geometry**, not orientation: 0 of 16 transfer points reachable by both arms |
| ~~**T2 scenarios**~~ | **DONE** — 4 graded scenarios, verified whole-path at N=10, and the discrete outcome is instrumented |
| **Mode 3 orientation assist** | this `/compute_ik` plugin ignores `OrientationConstraint` (measured: identical IK success with and without). Needs a constraint-aware plugin or explicit yaw sampling |
| **Mode 6 participant-readiness** | **detection rate unmeasured.** Models fit (2012/4096 MiB) but the synthetic renderer is out of distribution and measures itself |
| **Voice input** | `/dev/snd` holds only `timer`. The Windows-side UDP sender is written, never exercised |
| **VR on hardware** | `adb` is installed on neither WSL nor Windows |
| **T5 handover success rate** | needs the WEARER'S BUTTON (a foot pedal). Correctly not inferred from joint states — only the wearer knows whether they have the tool |
| **Grasp success in modes 5/6** | unmeasured — execution stops at PLAN |

## THE LAB ORDER

**0. Nothing here needs re-verifying first.** Every protocol coordinate was
re-audited on 2026-08-08 at N=10 over the whole path and all 17 are SOLID.
Re-run `python3 scripts/audit_scenario_reachability.py` only if the mount,
the home pose or a layout changes — all three invalidate it.

**1. Repair `l_j2` and `l_j4`.** These two buy the most: with both frozen the
left arm's commanded set collapses from a 135 mm-thick shell to a *surface*.
Right arm needs no help (R² = 0.986).

**2. `bash scripts/check_channels.sh` after EACH attempt.** Diffs
automatically against `recordings/baselines/channels_20260806.json`.
**Target 12+ of 14 coherent**, where `degraded_mode:=auto` stops engaging.
Run it after each attempt, not once at the end — a repair that breaks a
working channel should be caught immediately.

**3. One 35-minute recapture, gate fix live, EXACTLY ONE STACK.** The
2026-08-06 capture lost 41 of 42 directional segments to a clutch-gating bug
and latched the e-stop through block F. Both are fixed. Two `master_pose_node`
instances are now impossible at the fd level, but confirm the refusal never
appears.

**4. Verify the cascade rate limit end to end.** `clamp_towards` now cascades
`max_step` as well as `max_vel`; unit-tested, **never measured with a stack
up**. **Expected: divergence stays roughly CONSTANT as `lag_trip_rad`
changes rather than scaling with it.** If it still scales, the fix is wrong.

**5. Photograph the task objects** for the detection measure — 25 images per
object at 0.25 / 0.35 / 0.50 m. Pass = **≥95% at working distance**. Until
then mode 6 is not participant-ready.

---

# NEXT SESSION — LAB ORDER

## READ THIS FIRST

**Every number in the protocols is IK FEASIBILITY IN SIMULATION.** Nothing in
the bimanual programme has been driven by a human through the master arm.
The DIRECT condition of every task — the baseline the whole design rests on —
depends on channels that are currently **INCOHERENT**: 7 of 14 fail the
coherence test, and degraded mode freezes l_j2 and l_j4, which costs the left
arm its entire radial dimension.

So the reachability, the scenarios, the sling geometry and the pilot all say
what the ROBOT can do. None of them say what an OPERATOR can do through this
master arm today.

## In order

### 1. Repair l_j2 and l_j4 — these two buy the most
Degraded mode freezes them, and with both frozen the left arm's commanded set
collapses from a 135 mm-thick shell to a SURFACE: no in/out motion at all.
Measured: regressing true reach on every still-coherent channel plus the IMU
gives **R² = 0.133, residual 51 mm = 93% of the 55 mm true spread** — there is
no reach observable left on that arm. The right arm needs no help (R² = 0.986).

### 2. `bash scripts/check_channels.sh` after EACH repair attempt
Diffs automatically against `recordings/baselines/channels_20260806.json`.
**Target: 12+ of 14 coherent**, which is where `degraded_mode:=auto` stops
engaging. Re-test after each attempt, not once at the end — a repair that
breaks a working channel should be caught immediately.

### 3. ONE 35-minute recapture, gate fix live, exactly ONE stack
The 2026-08-06 capture lost 41 of 42 directional segments to a clutch-gating
bug (each arm gated by its OWN button) and the e-stop latched through block F.
Both are fixed. **Exactly one stack**: two `master_pose_node` instances split
the serial stream and invalidated a whole day; that is now impossible at the
fd level (`claim_exclusive`), but confirm the refusal message never appears.

### 4. Cascade rate limit, end to end with a stack up
`clamp_towards` now cascades `max_step` as well as `max_vel`; it is
unit-tested and **has never been measured with a stack running**.
**Expected result: divergence stays roughly CONSTANT as `lag_trip_rad`
changes, rather than scaling with it.** If divergence still scales with the
threshold, the cascade is still not rate-limiting and the fix is wrong.

### 5. Photograph the task objects — the mode-6 detection gate
Per the Part 9 decision below. 25 images per object at 0.25 / 0.35 / 0.50 m.
Pass = **≥95% detection at working distance**. Until then mode 6 is not
participant-ready.

---

# NEXT SESSION

# ============================================================
# BLOCKER — NO PARTICIPANT MAY BE RUN UNTIL THIS CLEARS
# ============================================================

**7 of 14 master channels are INCOHERENT.** Only 6 of 14 are usable
(5 ALIVE + 1 INTERMITTENT). An incoherent channel returns values that span a
wide range but are not a trajectory — consecutive updates jump tens of degrees
in 60 ms, which no hand produces.

**Running participants on that measures the harness, not the research
question.** Every number a session produced would be a property of the wiring.

**Acceptance test — the objective gate:**

```bash
bash scripts/check_channels.sh          # ~3 min, block A only
```

**Target: 12+ of 14 coherent.** It diffs automatically against
`recordings/baselines/channels_20260806.json`, so each repair attempt shows as
a verdict change. Re-run it as often as you like with the iron in hand.

This is a soldering problem. Do not attempt to filter an incoherent channel
into usefulness: the sim is a RELAY, so whatever the pots feed in the sim
reproduces and the real arm copies a second later.

## SECOND BLOCKER — the task layouts are not reachable

`python3 scripts/check_layout_reachable.py` reports **12 of 12 layout points
UNREACHABLE** as the protocols currently specify them. This is independent of
the wiring and must be fixed before T2/T3/T5 can run at all. See
"Front reach" below.

---

Priority order. Item 1 is first because a number derived from it currently
shapes the whole experiment design and **may be a measurement bug**.

Read `CLAUDE.md`'s final section — "HARDENING PASS — CONSOLIDATED SUMMARY" —
before touching anything. The single most important line in it:
**nothing in the hardening pass has ever run against a real arm.**

## Before anything, every session

```
bash scripts/recover.sh                  # known-good from any state
bash scripts/diagnostics.sh              # before blaming the code
ros2 run srl_teleop preflight            # refuses to start, and names why
```

`preflight` now fails on an EXPIRED blocker as well as an active one. An
expiry means the unit asserting it stopped running, so the condition is
unknown — do not start a session on it. See the consolidated summary for the
three-state semantics.

If the graph looks empty, suspect the **ros2 daemon**, not the network:
`ros2 daemon stop && ros2 daemon start`, and use
`ros2 node list --no-daemon` as ground truth.

---

## 1. Reconcile the two IK harnesses — 95.8% vs 0%

**Why this is first.** The 18.8% figure in "The seven constraints" is what
supports the conclusion that *these arms cannot reach a table*, and that
conclusion drives the task volume, the home poses, the grasp library and every
experiment's scenario set. If it is a measurement artefact, a large part of
the design is built on it. Two harnesses disagreed by essentially the entire
range, which is not the signature of a marginal geometry — it is the
signature of a harness bug.

**START HERE, and it is not what you expect: neither harness exists in the
repo.** Both numbers came from ad-hoc scripts written into a scratchpad and
discarded. `grep -rl compute_ik` over `src/` returns only production nodes
(`ik_follower_node`, `grasp_generator`, `master_calibration`) — there is no
committed IK-sweep tool. So this cannot begin by "re-running" either side.

### What to run

1. **Write ONE harness and commit it**, e.g. `scripts/ik_sweep.py`, and make
   it print its own configuration in the header of every run: pose list, seed
   policy, `avoid_collisions`, orientation constraint, group name, timeout,
   and which frame the target is expressed in. Every disagreement below is a
   configuration difference that a printed header would have made obvious.
2. Sample the frontal task volume on each arm's own side and report, per
   sample, the FAILURE REASON, not just a pass rate:

```
bash scripts/run_teleop.sh gate:=false          # move_group + TRAC-IK live
python3 scripts/ik_sweep.py --arm left  --volume frontal --report-reasons
python3 scripts/ik_sweep.py --arm right --volume frontal --report-reasons
```

3. Check these five differences explicitly, because they are the ones that
   can produce 95.8% vs 0% from the same geometry:
   - **`avoid_collisions` true vs false**, and whether the wearer links are in
     the planning scene at all when it is true;
   - **seeded from the current joint state vs from a fixed seed** — the
     continuous-reachability work already showed pointwise IK and
     seeded-from-previous IK answer different questions;
   - **orientation exact vs yaw-free vs an `OrientationConstraint`** — this
     `/compute_ik` setup **ignores the `constraints` field entirely** (measured:
     identical to the digit, 79.3%/68.9% both ways), so a harness that thinks
     it relaxed orientation did not;
   - **target frame** — `world` vs the arm's `base_link`. A frame error gives
     0% cleanly and looks like a reach limit;
   - **a stale `move_group`** holding the pre-mount-fix URDF. Read
     `/robot_description` off the TOPIC and compare the
     `backpack_to_*_mount` origins against the xacro, as was done for the
     mount fix.

### What a pass looks like

One committed harness, run twice with the two configurations, reproducing
BOTH numbers and explaining the gap in one sentence — e.g. "95.8% was
collisions-off with yaw free; 18.8% was collisions-on with an exact pose."
Then a single agreed figure, with its configuration recorded, replacing the
18.8% in `CLAUDE.md`'s constraint 7.

A pass is **not** "the new number is 60%". It is knowing which of the two old
numbers was wrong and why.

### What it blocks

The table-reach conclusion, the mount trade-off between constraints 2 and 7,
the task volume, the home-pose work, and the scenario sets in E1–E6. Do not
re-optimise the mount or the homes until this is settled — both previous
mount searches were invalidated by scoring the wrong thing.

---

## 2. Verify the mount fix against the real arms at the legacy home angles

**Why.** The mount was DERIVED, not searched, and verified only in sim — the
xacro parses, the rpy pair are exact mirrors to 0.000e+00, `/robot_description`
read back off the topic matches to 3.89e-16, and MoveIt reports
`valid=True, contacts=0` at home. None of that is evidence about the physical
bracket.

**GROUND TRUTH, and do not negotiate with it:** at the legacy Kortex home
angles the REAL arms point FORWARD. The home joint angles in
`config/home_positions_{left,right}.txt` and the two `initial_positions` in
`srl_dual.urdf.xacro` are correct and **must not be changed to fix geometry**.
If sim and real disagree, the mount is the suspect.

### What to run

```
bash scripts/check_arm_network.sh                    # latency, loss, ports
bash scripts/start_real.sh --mock                    # rehearse first
ros2 launch srl_teleop real_drivers_readonly.launch.py   # no motion at all
ros2 topic echo /real/joint_states --once
ros2 run tf2_ros tf2_echo world left_end_effector_link
ros2 run tf2_ros tf2_echo world right_end_effector_link
```

Compare the real joint angles against
`config/home_positions_{left,right}.txt` through `kortex_convention`, and the
real arm's physical pose against the sim's P_HOME:

```
left  P_HOME (-0.5195, 0.2138, 1.3928)
right P_HOME (-0.8596, 0.3169, 1.3456)
```

**Read the right arm's home angles off the hardware while you are there.**
`config/home_positions_right.txt` and `config/real_home_reference.txt` both
record that they were NEVER READ and are UNVERIFIED, and right P_HOME depends
on them.

### What a pass looks like

Both arms come over the shoulders to the FRONT at roughly chest height, with
`half_arm_1` crossing in front of the chest — matching the "What you should
SEE in RViz" list. Both hands land on the wearer's left with the arms
crossing; **that is expected**, not a bug — it follows from the home angles
and is provably independent of the mount (`|v_R − M·v_L| = 1.3837 m` for every
mirror-symmetric mount).

Fail signature: arms splayed sideways, reaching up and back over the
shoulders, one hand above the head. That is the OLD mount — suspect a stale
`move_group` process, not a stale install (`srl_description` is
symlink-installed).

**Also check the thing sim can no longer warn you about:** the Gen3 base tube
is 46 mm in radius and starts ~38 mm from the wearer's upper-arm surface, so
`{base_link, shoulder_link, half_arm_1_link}` vs `human_*_upper_arm` are
SRDF-excluded. On the real rig the bracket must stand the base off the
shoulder, or the mounts move outboard/up. **An exclusion silences the alarm;
it does not move the metal.**

### What it blocks

Every clearance figure, the workspace anchor, all grasp poses, and wearing the
rig at all. Worn operation is **not recommended** until the proximal
interference is resolved mechanically.

---

## 3. The wiring repair — seven compromised channels

**Why.** Seven of fourteen pot channels are dead, railed, clamped or
intermittent, and they are the reason the whole system is built around
IMU-primary sensing:

| arm | live | compromised |
| --- | --- | --- |
| LEFT | j1 j2 j3 j4 | **j5** (2.8% zeros, 14% railed), **j6** (12.8% clamped 0), **j7** (75% railed) |
| RIGHT | j1 j2 j6 | **j3** (70% zeros), **j5** (99% zeros), **j7** (99% zeros), **j4 intermittent (12.9% zeros)** |

Two priorities, in this order:

1. **Right j4 — the most consequential.** It is a channel spherical mode
   USES, and its dropouts are bursty: 51.1% of `right_A_back_to_forward`.
   The validator handles it correctly (`valid==0` tracks `j4==0` to within
   0.1%), so no fabricated data reaches the command — the damage is
   **staleness**, a frozen command for half of one sweep. **Check the j4
   connector first.**
2. **All three dead right channels are ROLL joints** (j3, j5, j7). Suspect
   ONE wiring fault fatigued through the rotation, not three independent pot
   failures. Look for a single common-mode cause before replacing pots.

Repairing the wrist channels (left j7/j6, right j3/j5/j7) is what unlocks
`orientation_mode: anchored` — currently `fixed`, i.e. position-only teleop,
because the wrist cannot be measured at all.

### What to run — the gain matrix IS the acceptance test

```
terminal 1:  ros2 launch srl_teleop teleop.launch.py
terminal 2:  bash scripts/run_teleop_capture.sh
             ros2 run srl_teleop analyse_teleop <csv>
```

The recorder is **button-gated** — it waits, you press to start, sweep at your
own pace, press to end. Every segment is gated by the **OPPOSITE** arm's
button, because an arm's own button also toggles that arm's clutch and would
disengage it exactly when the sweep starts. A press under 0.5 s discards the
segment and re-prompts.

**Before capturing, check** `ros2 control list_controllers` shows
`joint_state_broadcaster` and both arm controllers ACTIVE, and
`ros2 topic hz /joint_states` is ~100 Hz. A recording with 0/20440 EE rows has
happened, and the cause was dead `/tf`, not the recorder.

### What a pass looks like

`d(ACTUAL ROBOT EE)/d(master)` — not `d(command)/d(master)` — approaching
`diag(+1,+1,+1)` with a **moving** master. The distinction is not pedantic:
the one existing moving-master measurement showed the command sweeping
correctly at gain ~1.0 while the robot EE gain was **~0.1**.

Specific targets:
- **lateral finally works.** Up/down and fore/aft already track; lateral does
  not, and AXIS_MAP cannot fix it — azimuth comes from j1 alone, and j1 only
  produces lateral motion once a downstream bend offsets the tip from the roll
  axis, so azimuth and elevation are not independent. The reversal test is the
  cleanest metric (it assumes only that you reversed each motion): left is
  12.6° mean error from 180°, right 26.2°.
- right j4 dropouts gone: `valid==0` fraction near zero on the right arm.
- **Do not re-try hybrid azimuth** (`atan2(p_fk.y, p_fk.x)`) until the pot
  ANGLE calibration is fixed — it was tested and made every axis worse
  (left 12.6° → 33.3°), because FK azimuth compounds four suspect angles while
  j1 is one directly-measured zero-referenced quantity. Calibrated q3 spanned
  336° and q6 ±130–170°, which is physically impossible.

### What it blocks

`orientation_mode: anchored`, the two-IMU elbow path (needs a second I²C bus —
`Wire1` on Teensy pins 16/17 — because 0x68 and 0x69 are both already taken),
E4's DOF-recovery manipulation, and any claim that the teleop baseline is a
fair comparison rather than a broken one.

---

## 4. `start_real.sh` end to end

**Why.** It reached `REAL ARMS LIVE` once, then the lag monitor tripped at
0.152 rad. The tuning issue behind that was fixed and **never retested**, so
the gated flow has never completed against real hardware.

### What to run

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh --mock      # rehearse the identical sequence
terminal 3:  bash scripts/start_real.sh             # then for real
```

`start_real.sh` refuses with a NAMED reason and exit 1 when the sim stack is
down, the e-stop is latched, homing finishes outside tolerance, or the bridge
will not enable.

### What a pass looks like

- Homing: all 7 joints within tolerance (mock reference: ≤0.05 rad, max error
  0.0174 rad = the 1° deadband), and joint_5 taking the short way round.
- `REAL ARMS LIVE`, and the lag monitor **does not trip** through a full
  sequence of deliberate moves.
- On exit, grep for `kortex session closed cleanly`. **The arm permits exactly
  one session and a leaked one blocks the next run.** SIGINT the bridge
  directly if you kill things by hand; SIGKILL leaks the session.
- Read the LOGGED achieved rate, never assume `rate_hz`: expect ~18.4–18.7 Hz,
  not 30. A cycle is two sequential round trips and back-to-back RPCs cost
  ~26 ms each. That is fine — at vmax 0.05 rad/s a joint moves 2.8 mrad per
  cycle against a 17 mrad deadband.

### Expectations worth setting

- **The cyclic path is unusable over WSL** and this is settled, not worth
  re-litigating: every cyclic write is a ~10 ms network round trip, which
  consumes the entire 100 Hz budget. Use the HIGH-LEVEL API
  (`kortex_highlevel_bridge`, `real_arms_highlevel.launch.py`). If you find
  yourself debugging `ros2_control`'s Kortex driver, stop.
- **Mirrored networking is mandatory** (`networkingMode=mirrored` in
  `%UserProfile%\.wslconfig`, then `wsl --shutdown`). Under NAT, TCP 10000
  connects fine and the UDP realtime channel on 10001 times out — 3–6 SECOND
  writes, feedback frozen, controllers reporting "active" while the hardware
  deactivates underneath them.
- **A fast read is the dead-channel signature**, not an improvement. Once a
  write error deactivates the component, `read()` returns a cache. Same
  zero-variance pattern as the frozen `/real/joint_states` and the dead j7 pot.

### What it blocks

Every real-arm experiment, and the **untested Kortex session recovery** —
`/real/session_recover` closes the one permitted session and opens a fresh
one, and that sequence has only ever run against a mock. It is the riskiest
thing in the recovery layer and a real run will hit it first. Exercise it
deliberately once the bridge is up:

```
ros2 service call /real/session_recover std_srvs/srv/Trigger
```

Pass: `fresh session created without a relaunch (recovery #1)`, the arm still
responds, and no leaked session on the next start.

---

## Things that will waste your day if you forget them

- **Never `pkill -f` a node name.** The pattern matches the shell running the
  command and kills your own terminal. Kill an explicit PID list.
- **Never clean up with broad name patterns while a stack you want is
  running.** That is what killed `joint_state_broadcaster` mid-capture and
  produced a recording with zero EE rows.
- **`/dev/shm` fills with stale `fastrtps_*` segments** from killed stacks and
  discovery goes intermittent. Clear them with the stack STOPPED.
- **`ros2 param set` hangs on this box** (the daemon). Use a parameter client.
- **Never use `time.time()` for intervals** — the WSL wall clock steps
  backwards and once produced a send latency of −2321 ms. `time.monotonic()`.
  The audit found six files still doing it; all fixed, keep it that way.
- **`/estop_reset` is a SERVICE, not a topic.** Publishing a Bool at it does
  nothing, which once made six consecutive fault injections report NOT HANDLED
  for that single reason.
- Shell scripts sourcing ROS must wrap it in `set +u` / `set -u`.

---

# FRONT REACH — measured, and it constrains every task

`python3 scripts/diagnose_front_reach.py --arm both`

The arms work far better BEHIND the wearer than in front, and in front is
where every task volume is.

| direction | LEFT | RIGHT | binding constraint |
| --- | --- | --- | --- |
| front | **0.160 m** | **0.100 m** | **IK INFEASIBLE** |
| back | 0.900 m (sweep limit) | 0.900 m | wearer collision |

**(a) The front limit is NOT the wearer, and NOT orientation.**

- Re-solved with `avoid_collisions=False`: **still fails**. So it is not the
  wearer.
- Re-solved with yaw sampled over 8 angles: gains only **+0.010 m** (left) and
  **+0.040 m** (right). So it is not `orientation_mode: fixed` either.
- The limited joints are nowhere near their stops at the failure point:
  **joint_2 at 62% / 57%**, joint_6 at 41%. So it is not a joint limit.

It is genuine kinematic infeasibility for that arm base pose. By contrast
**back** IS wearer collision, and there orientation matters enormously —
yaw-free extends it +0.390 m on both arms.

**(b) Mount rotation DOES trade in our favour, and back does not pay for it.**
`python3 scripts/sweep_mount_tradeoff.py --arm both` (pitch about the mount
x-axis; **nothing applied**):

| delta | left front | left back | left down | right front | right back |
| --- | --- | --- | --- | --- | --- |
| **−40°** | **0.280** | 0.900 | 0.400 | **0.200** | 0.900 |
| −20° | 0.220 | 0.900 | 0.320 | 0.160 | 0.900 |
| **0° (current)** | **0.160** | 0.900 | 0.220 | **0.100** | 0.900 |
| +20° | 0.080 | 0.900 | 0.140 | 0.020 | 0.900 |
| +40° | 0.000 | 0.900 | 0.020 | 0.000 | 0.000 |

−40° nearly doubles front reach (left +75%, right +100%) and improves down
reach (0.220 → 0.400 / 0.180 → 0.360) while **back stays pinned at 0.900**.

**Caveat, and it matters:** back is measured only to the 0.90 m sweep limit,
so it is saturated and cannot show degradation. The trade-off is measured
against a truncated back. Re-run with a larger `--max` before acting.

**NOT APPLIED.** A mount change invalidates P_HOME, the workspace anchor and
every clearance figure, and would force re-deriving the orientation lock.

**(c) 0.160 / 0.100 m is NOT enough. The layouts do not work.**

`python3 scripts/check_layout_reachable.py` → **12 of 12 points UNREACHABLE**,
walking from home exactly as the follower does:

| point | arm | needed | reached |
| --- | --- | --- | --- |
| T2 box lip | right | 0.556 | 0.520 |
| T2 block A/B/C | left | 0.59 / 0.51 / 0.44 | 0.48 / 0.45 / 0.41 |
| T3 tray grips | both | 0.68 / 0.61 | 0.56 / 0.46 |
| T3 place | both | 0.69 / 0.61 | 0.56 / 0.46 |
| T5 cradle, receive | left | 0.48 / 0.56 | 0.41 / 0.44 |

Raising the work surface by 0.15 m and 0.25 m was tested: **still 12 of 12**.

### What has to change — layout, not mount or posture

Three separate problems, and only the first is about height:

1. **The stations are simply too far.** 0.44–0.69 m required against ~0.41–0.63 m
   achieved. Stations must sit within roughly **0.35 m of each arm's home EE**:
   left `(0.697, 0.248, 1.146)`, right `(−0.763, 0.298, 1.179)`.
2. **The naming quirk was applied wrongly in my own protocols.** `left_*` links
   sit at **+x**. The T2 draft put the box at x=−0.28 and assigned it to the
   left arm, i.e. reaching across the body. Corrected in the checker; the
   protocol layout tables still need updating.
3. **T3 is geometrically impossible as specified.** The two home EEs are
   **1.46 m apart in x**. A 300 mm tray on the midline requires each arm to come
   ~0.65 m inward — far beyond the ~0.4 m typical reach. T3 needs either the
   tray placed off-midline within one arm's reach of each grip point, or a
   different home parking pose. **Do not attempt T3 until this is resolved.**

Mount change and wearer posture are NOT the first lever here: the mount buys
+0.12 m of front reach at best, and the shortfall is 0.03–0.23 m across the
points, so layout alone can close it for T2 and T5. T3 needs more.

---

# PARTICIPANT SESSION ORDER

**Gated on the blocker above. Do not run until `check_channels.sh` shows 12+
of 14 coherent, and the layouts are re-sited.**

| | phase | min | notes |
| --- | --- | --- | --- |
| **P0** | Consent, briefing, baselines | 10 | embodiment questionnaire + proprioceptive drift, **BEFORE any system use** |
| **P1** | Familiarisation and practice | 10 | free play, then practice to a stated criterion. **Log whether the criterion was met** |
| **P2** | **T1 Bimanual Reach** | 10 | no objects; doubles as extended familiarisation while producing real characterisation data |
| **P3** | **T3 Coordinated Carry** | 15 | **PRIMARY TASK, deliberately early on the freshest participant** |
| **P4** | Break — harness OFF | 5 | **Borg CR10** fatigue probe |
| **P5** | **T2 Hold and Fill** | 15 | less demanding, forgiving of moderate fatigue |
| **P6** | **T5 Handover to Wearer** | 8 | **OPTIONAL — cut this first if running long** |
| **P7** | Post-session measures | 10 | embodiment + drift repeated, NASA-TLX per condition, trust scale, demand-characteristics question |

**Full session 83 min, which is over the ceiling for wearing unpowered mass.
Without P6 it is 75 min. Treat 75 as the target and P6 as a stretch.**

**T3 goes early on purpose.** The master arm has no gravity compensation, so
the operator's own arm carries its weight throughout and fatigue drifts
monotonically through the session. The headline measure must not be taken
last.

**T4 Inter-Arm Handover does not appear: it is BLOCKED** pending wrist
orientation.

## Counterbalancing and fatigue

- Three autonomy conditions — **DIRECT / ASSISTED / SHARED** — Latin-square
  counterbalanced **WITHIN each task, not across tasks**. Each task is its own
  square, so a task's three conditions are order-balanced against each other.
- **Log elapsed wear time per trial** (`wear_time_s` from harness-on) so
  fatigue enters the model as a covariate.
- **Order cannot remove fatigue, only measure it.** Counterbalancing converts
  a systematic bias into variance; the Borg probe at P4 and the wear-time
  covariate are what make it estimable.

---

# BLOCKED-ON-WHAT (2026-08-07)

| item | blocked by | note |
| --- | --- | --- |
| **T4 inter-arm handover** | **GEOMETRY, not orientation** | corrects the earlier note. 0 of 16 transfer points and 0 of 63 grid cells are reachable by both arms. Wrist angle was never the blocker; there is nowhere to hand anything over. Needs the right arm re-parked in hardware |
| **T2 hold and fill** | geometry | no (hold, release-above) pair exists |
| **T3 coordinated carry** | layout only | feasible at **z 1.10-1.30 m**, not the protocol's 0.885 m table. Put the tray on a stand |
| **mode 3 orientation assist** | this `/compute_ik` plugin **ignores `OrientationConstraint`** | measured: identical IK success with and without it, 79.3%/68.9% both ways over 270 calls. To make it real needs EITHER a constraint-aware IK plugin (e.g. bio_ik or a TRAC-IK build with constraint support) OR explicit yaw sampling inside `ik_follower_node` — sample N yaw values about the commanded approach axis and accept the first that solves. The scaffolding for the second already exists in the home-pose scorer |
| **mode 6 participant-readiness** | **detection rate unmeasured** | models install and fit (2012/4096 MiB) but the synthetic harness is out of distribution and measures itself. Needs the real cameras |
| **voice input** | audio routing, needs the lab | `/dev/snd` holds only `timer`. `scripts/win_mic_sender.py` (Windows-side UDP) is written and NOT exercised with a real microphone |
| **grasp / handover success rates** | the above | unmeasured because execution stops at PLAN: the objects that would be grasped are not reachable by the arms that would need to cooperate, and there is no trustworthy detection to drive them |

## DETECTION RATE — the gate on mode 6 (decision: option 1)

**Chosen: photograph the real objects on the real table and run detection on
those images.** Justified against the alternatives:

- A photorealistic renderer would take a physically-based pipeline, HDRI
  lighting and real material parameters to be worth more than flat shading —
  and it would still be a model of the Kinova camera, not the camera. The
  effort buys a number that must be re-measured on hardware anyway.
- "Accept as unmeasurable" is where it already stands and yields nothing.
- Photographs need only the lab, a phone or the wrist camera itself, and 20
  minutes. It is the honest measurement.

### Procedure

1. Place the task objects on the table in the verified positions.
2. Capture **≥ 25 images per object** at each of 0.25, 0.35 and 0.50 m,
   varying pose and lighting; use the **wrist camera** if the arm is live,
   otherwise a phone at the same working distance.
3. Label the true bounding box once per image (a rectangle drag).
4. Run `scripts/measure_detection.py --images <dir>` (the renderer is
   replaced by a file loader; the metric code is unchanged and already
   known-answer tested).

### What a PASS looks like

| | threshold |
| --- | --- |
| detection rate at 0.25–0.35 m | **≥ 95%** per object |
| position error | ≤ 15 mm mean, ≤ 30 mm p95 |
| latency | ≤ 100 ms per frame |
| false positives on an empty table | ≤ 1 in 100 frames |

**Below 95% at working distance, mode 6 stays not-participant-ready.** It is
there now, and this is the only measurement that moves it.

### Already established, so it need not be redone

- models install and coexist: faster-whisper small/int8 + YOLO-World-s =
  **2012 MiB of 4096**, ~23 ms/frame
- the model is not the problem: **0.89–0.91 confidence on a real photograph**
- ultralytics 8.4.116 + torch 2.13: call `set_classes()` BEFORE the first
  CUDA predict, or it raises a device-placement error

## JOB B.1 — WRIST ORIENTATION AFTER THE POT REPAIR (2026-08-09)

`scripts/measure_wrist_capability.py`, both arms verified at home to 0.0000 rad.

**(a) The master can now express the rotation.** Sweeping j5/j6/j7 over their
full travel gives **180.0 deg** of tip reorientation, against the **169.7 deg
(left) / 164.6 deg (right)** a top-down grasp needs from the pinned anchor.
With the wrist channels dead this was unavailable at any angle, so the repair
does restore the input side.

**(b) The ROBOT is now the binding constraint, and it was not before.**
27-pose grid per arm (+/-0.10 m, the master ball's half-span), N=5, live
`/compute_ik`:

| arm | fixed (ships today) | tilt | top-down grasp |
| --- | --- | --- | --- |
| left | 74.1% | **73.5%** | 55.6% |
| right | 85.2% | **81.2%** | 55.6% |

**TILT IS NEARLY FREE, BUT THE 90% BAR IS NOT MET BY EITHER MODE.** Tilt costs
0.6 pp on the left and 4.0 pp on the right against fixed. Per the stated rule
(ship if tilt holds above 90%) the answer is **DO NOT SHIP** -- but note the
rule as written can never fire, because `fixed`, the mode that ships today,
is itself at 74-85% over the same volume. The comparison that carries
information is tilt-vs-fixed, and on that tilt is within sampling noise on the
left.

This does NOT contradict the earlier "tilt is not viable" finding. That one was
about `OrientationConstraint`, which this `/compute_ik` plugin ignores. This
commands an EXACT tilted pose instead, and an exact tilted pose solves at
essentially the same rate as the anchor.

**Top-down grasp is 55.6% over the volume** (it is 100% at the centre pose
alone). Carried into Job C.

**LIMIT:** this measures what the GEOMETRY permits. Master ACCURACY at those
orientations depends on the repaired pots and every recording here predates
the repair. A fresh capture is the only thing that closes it.

### Two instrument bugs, both caught before the number was reported

1. **Tilt was built as an ABSOLUTE master-frame rotation** and never used the
   anchor, despite the comment saying it did. The roll=pitch=0 cell therefore
   tested the identity quaternion, not the anchor, and the whole mode scored
   **0.0%** -- which would have been reported as "tilt is dead". Fixed to
   `anchor (x) rpy(roll,pitch,0)`, with the zero cell kept as a permanent
   known-answer check: with no tilt applied, tilt IS fixed.
2. **That check then demanded exact equality on a randomised solver** and
   flagged a 0.7 pp difference as a bug. Two independent draws at this sample
   size have a binomial sd of ~3.7 pp. The band is now 3 sigma. A guard that
   cries wolf is the failure mode a guard must never have.

**ALSO STANDING FROM JOB A:** degraded mode still reads the stored
`channels_20260806.json` (6/14 coherent) and will still freeze eight working
channels until `bash scripts/check_channels.sh` is run in the lab.

## JOB B.2 — CAPABILITY DELTA, BEFORE AND AFTER (2026-08-09)

`scripts/measure_capability_delta.py`. Exact arithmetic over `degraded_mode`
and the master FK, no hardware needed.

| | coherent | degraded | left radial extent | right radial extent |
| --- | --- | --- | --- | --- |
| BEFORE (stored baseline) | 6/14 | ENGAGES, freezes l_j23456 / r_j357 | **0.000 m** (0.272 only) | 0.139 m |
| AFTER (all 14 repaired) | 14/14 | does not engage | **0.139 m** | 0.139 m |

The left arm's commandable set goes from a **SURFACE back to a SHELL**: with
j2 and j4 frozen the FK magnitude is a constant, so radius was not commandable
at all and the radial fallback on j7 existed only to substitute for it. The
right arm's frozen channels were all rolls, which carry no radial information,
so its extent never changed -- which is why the repair is worth far more to
the left arm than to the right.

Validated against an independently established figure: this reproduces the
documented left-collapse and the right's unchanged extent without being told
either.

### The instrument bug, again

The first run reported **"extent unchanged, 0.139 -> 0.139 on both arms"** -- a
clean null result, entirely manufactured. `decide()` returns
`{arm: [joint index]}` and I had guessed a set of `(arm, index)` tuples, so
the extraction silently produced `[]` and froze nothing in the BEFORE case.
The tell was the output contradicting itself: degraded mode ENGAGES while
freezing no channels.

## JOB B — WHAT IS BLOCKED ON THE LAB, AND THE EXACT CAPTURE NEEDED

Three of Job B's questions are fits to RECORDED FRAMES, and **every recording
in this repository predates the repair**. Re-running them now would measure
the broken pots and report the answer as the repaired capability:

- **azimuth / lateral** (the 54.8 / 54.7 deg pairwise-angle residual; FK vs
  gyro vs j1-alone). CLAUDE.md's own instruction was "do not re-try the hybrid
  without first fixing the pot angle calibration" -- that precondition is now
  met, but the re-test needs new data.
- **the R0-R5 position ladder** against full 7-DOF FK.
- **the smoothing re-tune** from freshly measured noise.

**THE CAPTURE THAT UNBLOCKS ALL THREE** (one session):

    bash scripts/check_channels.sh          # FIRST -- refreshes the baseline,
                                            # without which degraded mode still
                                            # freezes eight working channels
    ros2 launch srl_teleop teleop.launch.py
    bash run_teleop_capture.sh              # blocks A-F, ~35 min

Block C (directional) and block E (repeatability) are the ones the azimuth
question needs; block A alone is enough to refresh the channel baseline.

## JOB C — THE GRIPPER PENETRATES OBJECTS (2026-08-09)

`scripts/measure_grasp_penetration.py`, live `/compute_ik`, geometry from TF.

### The fault, and the number

`grasp_library.candidates()` returned `position = p.copy()` -- the bare object
CENTROID -- and `grasp_generator` commanded `<arm>_end_effector_link` there.
But the fingertips sit **0.1118 m along the tool's +z** (measured from TF,
both arms agreeing to 4 dp) and the grasp quaternion aims that axis **straight
down**. So the tool reached the centroid and the fingers were a full finger-
length below it, through the object and into the table.

| object | size_z | penetration BEFORE | AFTER |
| --- | --- | --- | --- |
| flat_plate | 0.012 | **+105.8 mm** | -5.0 mm |
| wide_block | 0.035 | +94.3 mm | -5.0 mm |
| small_cube | 0.040 | +91.8 mm | -5.0 mm |
| cylinder | 0.080 | +71.8 mm | -5.0 mm |
| tall_box | 0.090 | +66.8 mm | -5.0 mm |
| narrow_rod | 0.120 | +51.8 mm | -5.0 mm |

Positive = fingertip that far BELOW the underside. **Every object in the
catalogue, without exception.** The gripper never closed on anything.

**FIX:** `grasp_offset()` raises the tool by `FINGERTIP_REACH_M - size_z/2 +
pad` -- the object's half-height plus a 5 mm pad, offset by the finger length
that made the correction necessary. Derived, not tuned; a taller object needs
LESS offset, which is the sign check. The measurement script now reads the
LIBRARY's output rather than recomputing the formula, so it is a real
regression rather than the script agreeing with itself.

### Why IK did not refuse -- THREE separate things, not one

| probe | accepted |
| --- | --- |
| centroid pose, object NOT in the planning scene | **100.0%** |
| centroid pose, object IS in the planning scene | 0.0% |
| corrected pose, object present | 0.0% |
| corrected pose, object ABSENT (control) | 100.0% |
| pre-grasp standoff, object ABSENT (control) | **0.0%** |

1. **Collision-aware IK WOULD have caught it. The object is never in the
   scene.** `grasp_generator` sets `avoid_collisions=True`, but nothing in the
   autonomy path publishes a CollisionObject -- `world_model` and
   `grasp_generator` publish none at all, only `scripted_pick_place` and the
   experiment scene do. The check ran against a world where the object does
   not exist and passed 100%. This is the project's own "a marker is
   decoration" failure one layer up.
2. **Adding the object to the scene is NOT sufficient on its own.** The
   CORRECTED grasp is reachable with no object (100%) and refused with one
   (0%), because the fingers envelop the object -- which is what a grasp IS.
   Collision-aware IK cannot tell a grasp from a crash. The target must be
   excluded from the gripper's ACM, or attached, or only the standoff
   validated with collisions on.
3. **The pre-grasp standoff is UNREACHABLE, and that is reach not collision** --
   0% with the object ABSENT. The offset raises the whole approach by ~0.10 m
   and pushes the standoff out of the arm's volume at z=1.15. **Objects must
   be placed lower to pay for the correction.** This is a protocol change, not
   a code one, and it is the one Job C finding that costs something.

Without the object-absent controls, findings 2 and 3 would both have been
reported as "collision refused it". A 0% means nothing until the same pose has
been tried with the object removed.

### Position tolerance the design assumes

| object | lateral | vertical |
| --- | --- | --- |
| narrow_rod | +/-32.5 mm | +/-65.0 mm |
| tall_box | +/-25.0 mm | +/-50.0 mm |
| small_cube | +/-22.5 mm | +/-25.0 mm |
| wide_block | +/-22.5 mm | +/-22.5 mm |
| cylinder | +/-20.0 mm | +/-45.0 mm |

**At +/-10 mm and at +/-20 mm of real-world object error, every graspable
object still fits between the pads.** Lateral is the binding axis and the
tightest is the cylinder at +/-20.0 mm, so **20 mm is the placement precision
the lab needs**, and 10 mm carries margin on every object.

`flat_plate` is EXCLUDED, not a tolerance failure: its 90 mm minor extent
exceeds the 85 mm stroke, so it is ungraspable at 0 mm error and is already
refused upstream by `is_graspable()`. Reporting it as "misses at +/-10 mm"
would imply it works at 0 mm.

### Regression

`src/srl_autonomy/test/test_grasp_offset.py`, 5 tests, **negative-control
checked**: 3 of the 5 fail on the pre-fix code and all 5 pass after. One of
them exists solely to catch the formula being right while nothing calls it.

## JOB D — ARE THE MODES REAL? PARTIAL (2026-08-09)

`scripts/verify_mode_paths.py` (live graph) and `scripts/drive_mode_paths.py`
(end-to-end injection).

### PROVEN, three independent ways: modes 4, 5 and 6 CANNOT COMMAND THE ARM

**`/autonomy/assist_pose_<arm>` has ZERO subscribers.** `handover_arbiter`
publishes the blended assist pose and **nothing in any package consumes it** --
confirmed by the live graph (pub 1, sub 0, with the arbiter running) and by an
exhaustive grep across `src/` and `scripts/` that returns only the arbiter
itself and my own new probes. Mode 4's entire contribution is computed
correctly and then dropped on the floor.

**`autonomy_executive` publishes no motion topic at all.** Its complete
publisher list is `/robot_speech`, `/autonomy_stage`, `/autonomy_decision`,
`/estop`. No pose, no trajectory, no action client. The only client anywhere
in `srl_autonomy` is `grasp_generator`'s `/compute_ik`, which VALIDATES a
grasp and never commands one. Modes 5 and 6 run a state machine that narrates
("Handing over to the other arm"), logs decisions and can e-stop -- and the
arm never receives a command from any of it.

| mode | verdict |
| --- | --- |
| 1 DIRECT_MANNEQUIN | path connected (`/master_arm_pose_left` pub 1 sub 1); end-to-end drive INCONCLUSIVE, see below |
| 2 DIRECT_VR | path connected via `/vr/controller_pose_<s>`; `/vr_pose_<s>` is a legacy topic with no subscriber |
| 3 ORIENTATION_ASSIST | same path as 1; already documented a STUB |
| 4 SHARED_AUTONOMY | **DOES NOT WORK** -- output has no consumer |
| 5 SUPERVISED_AUTO | **DOES NOT WORK** -- no motion path exists |
| 6 FULL_AUTONOMY | **DOES NOT WORK** -- no motion path exists |

This is the answer to "or a bluff". Modes 4-6 are a bluff in the specific
sense that every stage before the last one is real, measured and tested, and
the last hop was never connected.

### NOT ESTABLISHED, and I am not claiming it

The end-to-end drive for modes 1/2/3 returned **0 trajectories**, and I do
NOT report that as "mode 1 does not work" -- this project has measured mode 1
tracking to 0.4 mm RMS. The harness is contaminated: `master_pose_node` runs
in the stack and publishes to the SAME `/master_arm_pose_<arm>` topic I was
injecting on, in degraded mode with reach frozen, so the follower saw
interleaved poses and `[BLOCKED:ik_failed]` held 8.9 s. Two publishers on one
topic is the same class as two readers on one Teensy.

To finish it: relaunch with `master_pose_node` disabled (or point the follower
at a test topic) and re-run `drive_mode_paths.py`.

Three harness bugs were fixed on the way, each of which produced a confident
wrong answer first: the first run had no stack up at all and reported five
modes broken; mode 2's chain was keyed on the legacy `/vr_pose_<s>` rather
than the real `/vr/controller_pose_<s>`; and the injection used a WORLD-frame
position where the topic carries a MASTER-frame displacement, so IK correctly
refused every frame.

### NOT DONE

The language/vision prompt sweep (correct / asked / refused / MISUNDERSTOOD)
was not run. Given modes 5 and 6 cannot move the arm, the sweep would measure
the parser in isolation rather than the mode -- worth doing, but it should
follow the missing command path, not precede it.

## JOB E — THE CALIBRATION SWEEP IS NOW VISIBLE (2026-08-09)

`srl_perception/scene_markers.py` (new) + wired into `scene_fingerprint_node`,
publishing `MarkerArray` on **`/scene/markers`**.

Before this the sweep published only `/scene/state` and `/scene/objects` as
JSON strings and logged "SCENE CHANGED" with a list of names. The operator had
no way to see WHICH object the diff meant, and the log scrolls.

Per object: a coloured box at its pose, plus a TEXT_VIEW_FACING label carrying
**name, verdict, confidence, displacement and rotation**.

| verdict | colour | rationale |
| --- | --- | --- |
| SAME | desaturated grey-green, alpha 0.45 | normal is most of the scene; if it shouts, nothing else can |
| MOVED | amber | |
| MOVED_OR_SWAPPED | orange | association itself is in doubt |
| APPEARED | blue | new to the scene |
| RECLASSIFIED | magenta | identity changed |
| VANISHED | red, alpha 0.55 | transient, see below |

Three decisions worth keeping:

- **A dropped object is SHOWN before it goes.** Removing it instantly is
  indistinguishable from one that was never there: the operator sees a scene
  with one fewer object and no reason. It is drawn at its **stored** pose (the
  only pose it has -- there is no observation, that being what vanished means)
  for `drop_hold_s` (3.0 s default), then stops being republished.
- **A move is drawn as an ARROW from where it was**, because a displacement is
  only meaningful against its origin and the operator should not have to
  remember.
- **Text is always fully opaque** even when its box is faint. A desaturated
  box still reads as an object; desaturated text is simply unreadable, and the
  label is the part carrying the information.

Every frame begins with **DELETEALL** -- ids are per-frame, and without it the
previous sweep's markers stay on screen underneath the current one. That bug
has already been paid for once in this project.

### Verification

`src/srl_perception/test/test_scene_markers.py`, **10 known-answer tests**:
DELETEALL is first; all six verdicts are visually distinct; SAME is the least
saturated; a dropped object is shown at 0.5 s and gone at 9.0 s; VANISHED is
drawn at its stored pose; the label carries name and confidence; MOVED gets an
arrow from stored to observed and SAME gets none; text stays opaque; ids are
unique within a frame.

Live on a real graph, driven by a synthetic detector: **16 marker frames
received, DELETEALL first, 4 boxes + 4 labels**, e.g.
`red_cube  APPEARED  conf 0.92` at rgba 0.20 0.65 0.95.

**HONEST LIMIT:** the live run exercised only the APPEARED path. My fake
detector called `/scene/resweep` before the first fingerprint had settled, so
the second sweep compared against an incomplete store and every object came
back APPEARED rather than the SAME/MOVED/VANISHED mix intended. The colour and
lifetime logic for the other five verdicts is proven by the known-answer tests
against constructed verdicts, **not** by a live sweep. Re-run with a longer
settle before the resweep to close that.

**NOT DONE:** an RViz pixel capture of the markers. The `/scene/markers`
topic is verified to carry the right content; nobody has yet confirmed from
pixels that RViz draws it as intended. Use the Xvfb route -- x11grab on the
WSLg `:0` records black.

## JOB F — PRECISION/SPEED DIAL: MODULE DONE, SAFETY MEASUREMENT NOT (2026-08-09)

`src/srl_teleop/precision_speed.py` (new). One dial in [0, 1]:

| dial | | scale | smoothing | max_vel | max_step |
| --- | --- | --- | --- | --- | --- |
| 0.00 | PRECISION | 0.20 | 0.08 | 0.10 rad/s | 0.10 rad |
| 0.50 | BALANCED | 0.45 | 0.29 | 0.35 rad/s | 0.22 rad |
| 1.00 | SPEED | 1.00 | 0.50 | 0.60 rad/s | 0.35 rad |

**Scale is GEOMETRIC, not linear.** Perceived precision goes as motion per unit
of hand travel and the useful range is bunched low: 0.2 to 0.4 is a large
change in feel, 0.8 to 1.0 is barely noticeable. A linear map would waste most
of the dial.

### The safety invariant is enforced, not documented

`SAFETY_PARAMS` names the nine things the dial must never touch (clearance
floor, wearer pad, `avoid_collisions`, redundancy samples, dead-man timeout
and enable, e-stop enable, flip-reject limit, IK watchdog).
`assert_safety_unconditional()` RAISES on any of them, and the test that
matters walks **all 101 dial positions** and asserts the emitted settings dict
never contains one -- so a future edit that adds a key to `RANGE` fails the
test rather than silently shipping.

At dial 1.0 the arm moves faster and therefore reaches the floor sooner, but
the floor is in the same place. **Speed changes how quickly a wrong motion
happens, not how far it is allowed to go.**

### No jump on change

`rebase_anchor()` absorbs a scale change into the anchor:
`pos_anchor += (old - new) * (current - reference)`. Proven continuous to
**1e-12** across 1.0->0.2, 0.2->1.0 and 0.6->0.35, with the operator well away
from the engage point -- which is exactly where the uncorrected jump is worst
and least predictable. It reuses the correction the motion-scale parameter
already had rather than inventing a second one that could disagree.

9 tests pass.

### NOT DONE, and NOT to be assumed

**The safety measurement did not run.** `scripts/probe_dial_safety.py` is
written and drives the commanded pose straight at the wearer at both dial
extremes, but it recorded **zero clearance samples** and correctly refused to
conclude anything ("NO CLEARANCE SAMPLES -- nothing may be concluded"). Cause:
`/ik_status_<arm>` publishes an **EMPTY data array** in this state, so the
clearance field CLAUDE.md documents at index 5 is not there to read. Fix that
first -- either the follower only fills the array after activity, or the
documented layout has drifted from the code -- then re-run the probe.

**So the headline claim of Job F ("safety does not scale with the dial") is
proven in CODE and NOT YET IN MOTION.** Do not report it as measured.

**Also not done:** wiring the dial into the GUI as a slider and onto the VR
thumbstick, and the tracking-error-at-each-end measurement. The module is the
single source both would call.

## BLOCKER 1 — AUTONOMY CAN NOW REACH THE ARM (2026-08-09)

### The fix

`ik_follower_node` now subscribes to **`/autonomy/assist_pose_<arm>`**, which
previously had **zero subscribers anywhere** -- handover_arbiter and (now)
autonomy_executive computed a correct pose and dropped it, which is how three
of the four study modes shipped without ever driving the arm.

**IT ENTERS AT `request_ik`, THE SAME DOOR TELEOP USES.** `on_pose` maps a
MASTER-frame displacement through the anchor and scale and then calls
`request_ik`; an autonomy pose is already world-frame, so it skips the mapping
and *nothing else*. Collision-aware IK, the redundancy re-seed, the clearance
floor, the step guard, the flip reject, the e-stop and every BlockMonitor
blocker are literally the same code. **There is deliberately no second path to
the controller.**

`autonomy_executive` gained `command(arm, xyz, quat)` / `release()` and a
20 Hz pump. Its complete publisher list was speech, stage, decision and estop:
it narrated a pick and the arm never heard a word of it. The pose is
republished at a steady rate rather than once per stage, because the follower
treats autonomy as driving only while poses keep arriving.

**One source at a time.** A new `autonomy_has_control` blocker suppresses the
master while autonomy is live. Interleaving two sources on one arm is the
two-publishers-on-one-topic bug wearing a different hat.

### Verified live

| check | result |
| --- | --- |
| `/autonomy/assist_pose_left` has a subscriber | **yes** (pub 1, sub 1) |
| autonomy pose reaches the follower | **yes** -- `autonomy_has_control` is asserted, and only `on_pose` can set it, which requires a received autonomy pose |
| the SAME clearance floor applies | **yes** -- aiming inside the wearer names `clearance_floor` and `ik_failed`, and produced 0 trajectories |
| autonomy moves the arm | **NOT CONFIRMED -- see below** |

### NOT CONFIRMED, and not to be assumed

**0 trajectories at a reachable target.** The path is connected and the safety
stack demonstrably fires, but no motion was observed. Do not read this as
"autonomy now works". Two things point at the harness rather than the fix:
the stack logged `RTPS_TRANSPORT_SHM Error ... Failed init_port
fastrtps_port7004` (the documented stale `/dev/shm` segment problem), and a
second graph read of the same topic returned sub 0 while blockers from that
very subscription were arriving -- so discovery on this stack is degraded.

Also unresolved from Job F and likely related: `/ik_status_<arm>` publishes an
**empty data array** on this stack, so clearance cannot be read from it.

**Next session, in order:** stop the stack, clear `/dev/shm/fastrtps_*`,
relaunch, confirm the arms are at home, then re-run
`scripts/verify_autonomy_command_path.py`. It uses the arm's OWN anchor
orientation (an identity quaternion is not a neutral choice -- it is a
specific, unreachable one, and using it produced a false negative once here
and once in Job D).

### NOT DONE

The language/vision sweep (vague, relational, superlative, compound,
misspelled, absent objects; correct / asked / refused / **MISUNDERSTOOD**) was
not run. Blockers 2 and 3, the three task specifications, the session
timeline, and the dual-view GUI were not started.

## PART 1 — YES, AUTONOMY MOVES THE ARM (2026-08-09)

**Answer: YES.** 1092 trajectories at a reachable target, and — the measurement
that matters — **0.0700 m and 0.1204 m of real tf2 end-effector displacement
over two legs, residual 0.0000 m to target on both.** Three of four study modes
are no longer blocked at the last hop.

### The real bug: `ik_follower_node` had no `import time`

Not a discovery problem. `ik_follower_node.py`'s import block was
`sys, os, math, home_positions, rclpy, …` — **`time` was never imported**,
while the autonomy path added last session calls `time.monotonic()` twice:
line 713 in `on_autonomy_pose` and line 733 in `autonomy_is_driving`.

A missing name is only evaluated when the code path RUNS. So the node imported
cleanly, launched cleanly, logged `Tracking enabled`, and then died on the
**first `/autonomy/assist_pose_<arm>` message it ever received**:

```
File ".../ik_follower_node.py", line 713, in on_autonomy_pose
    now = time.monotonic()
NameError: name 'time' is not defined
[ERROR] [ik_follower_node-13]: process has died [pid 682827, exit code 1]
```

The followers launch **without `respawn`** (only `master_pose_node` has it), so
it stayed dead for the remainder of the session. Every autonomy pose after the
first went to a topic with no live subscriber. That is the whole of "0
trajectories", across two sessions.

The crash is at line 713, **before** `self.autonomy_pose_t = now` at line 720 —
the only assignment that can make `autonomy_is_driving()` return True. So last
session's row *"`autonomy_has_control` is asserted, and only `on_pose` can set
it"* could not have been true as written, and is now retracted.

### Discovery WAS also degraded — and it was a separate, non-causal fault

Both were real; only one mattered.

- **Four concurrent stacks were running**, not one — 35 stack PIDs across four
  `ros2 launch` trees, plus three orphaned `ros2 bag record` processes left from
  the 2026-08-08 verification sweep.
- **188 stale `/dev/shm/fastrtps_*` entries**, oldest dated Aug 5.
- The **`ros2` daemon was hung**, not merely stale: `ros2 daemon stop` blocked
  and died with `TimeoutError: [Errno 110] Connection timed out` after 120 s.

Cleared all of it, relaunched a single stack, confirmed both arms at home
(**max error 0.0000 rad, all 7 joints, both arms**) — and the re-run *still*
reported `autonomy moves arm : False`. **Fixing discovery changed nothing.**
That is what promoted the remaining failure from "environment" to "bug".

**Trap worth keeping:** `rm /dev/shm/fastrtps_*` leaves **39 `sem.fastrtps_*`
semaphores** behind — the glob does not match them. A count that greps only
`fastrtps` then reads "39 still there" and sends you hunting for a live writer
that does not exist. Remove both patterns.

### THE INSTRUMENT WAS LYING ABOUT THE SAFETY CHECK

`verify_autonomy_command_path.py` decided which blockers fired with a
**substring match over the raw `/blocking` payload**. That payload is JSON and
carries `blockers` — the list of every blocker the unit **registered** at
construction, active or not. So `"clearance_floor" in msg.data` was true in
**every message the follower ever published**, including at a perfectly
reachable target and including with the arm doing nothing.

It measured registration, not assertion. Last session's row *"aiming inside the
wearer names `clearance_floor` and `ik_failed`"* was therefore not evidence of
anything, and is retracted.

Fixed by parsing the JSON and reading `active` (plus `expired`, which is not an
all-clear) only, with the reachable drive kept as an explicit **control**:

| | trajectories | ACTIVE blockers |
| --- | --- | --- |
| reachable target (0.32, 0.35, 1.15) | **1092** | **none** |
| inside the wearer (0.0, −0.10, 1.25) | **0** | **`ik_failed`** |

Ten names would have been reported as "fired" by the old match and were not
active: `autonomy_has_control, clearance_floor, estop, flip_reject, home_gate,
ik_inflight, motion_disarmed, no_joint_state, startup_unwind, state_unknown`.

**The discriminating blocker is `ik_failed`, NOT `clearance_floor`.** The
clearance floor never fired — collision-aware IK refused to solve first. That
is a *cross-check passing*, not a new finding: it reproduces the graduated
collision response already measured here (`probe_collision_response.py`, "the
hard floor was never reached ... the earlier stages stopped the arm
0.072–0.079 m away"). Two independent measurements agreeing is the reason to
believe this one.

### Regression test

`src/srl_teleop/test/test_no_unresolved_module_names.py`, **4 tests**.

The obvious test — bring a node up and publish a pose — is the bug class this
project keeps paying for (#5: a test that constructs the environment where the
bug cannot occur). It needs `/compute_ik`, tf, a controller and home files.
This test parses the **shipped source** with `ast` and needs no environment at
all: any name used in attribute position (`time.monotonic`) that is never
imported or bound is reported, across **every module in every `srl_*`
package**. The next missing import is caught by the same test rather than by a
dead node.

**Negative-control checked both ways**, which is the only reason to trust it:
deleting the `import time` line makes 2 of the 4 fail with the real message,
and two further tests assert the *analysis itself* both detects a constructed
missing import and does not flag correct source.

Current sweep across all six `srl_*` packages: **no other unresolved module
names.**

### Still open, carried forward

- The followers have **no `respawn`**. A follower that dies and stays dead is
  the silent-stop class this project has paid for repeatedly. Not changed here
  — respawn also masks crashes, so it is a deliberate decision, not a default.
- `/ik_status_<arm>` publishing an empty data array (blocks the clearance
  readout and the Job F safety measurement) is untouched and is Part 2 work.

## PART 2 — FULL DIAGNOSIS (2026-08-09)

Report first, then fixes, then what was deliberately left. **Sim only.**

### THE HEADLINE: one bug was wearing three costumes

Part 1's missing `import time` was not one fault. It killed the left follower
on the first autonomy pose, and a dead node publishes nothing — so three
separately-filed problems were the same corpse seen from three angles:

| filed as | actually |
| --- | --- |
| "autonomy does not move the arm — 0 trajectories" | the follower was dead |
| "`/ik_status_<arm>` publishes an empty data array" (Part 2D.3, Job F) | a dead node publishes nothing. Measured live after the fix: **11 fields, index 5 = 0.1277 m**. Never empty |
| "`probe_dial_safety.py` recorded zero clearance samples" | it was subscribing to a dead publisher |

**Part 2D.3 is closed by the Part 1 fix, not by separate work.** The documented
field layout had not drifted.

### A. THE FIVE RECURRING BUG CLASSES

**A1 silent blocking — 1 real instance, fixed.** `probe_dial_safety.py` could
report a safety pass from a run in which the arm never moved (below). The nine
`ik_follower_node` blockers all carry mandatory recovery strings and are
covered by BlockMonitor auto-expiry; no new instance found there.

**A2 unreachable `clear()` — 7 occurrences, 0 latch-risk, but THE AUDIT IS
BLIND IN TWO PLACES.** All 7 are in `ik_follower_node` in the tolerated
fall-through form. The two "SET-WITHOUT-CLEAR" hits reported in
`quest_vendor_bridge.py` are **false positives**:

- `self.clutch` *is* cleared, at line 319 — but via `c.clutch = False` on a
  `Ctl` instance, not through `self`. The audit only searches within the
  declaring class.
- `self.frozen` *is* cleared, at line 379 — but by tuple unpacking,
  `self.frozen, self.freeze_reason = frozen, why`, which the audit does not
  parse as an assignment.

So `audit_unreachable_clear.py`'s standing "0 LATCH-RISK across 152 files" is
weaker than it reads: **it cannot see attribute assignment through a
non-`self` instance, or tuple-unpacked assignment, anywhere in the
workspace.** Two more instrument blind spots for the register.

**A3 wall-clock intervals — 10 found.** `time.time()` used for a duration:

```
scripts/drive_mode_paths.py:45,77,78          scripts/probe_dial_safety.py:60,78,103,104
scripts/measure_grasp_penetration.py:69,136   src/srl_teleop/degraded_mode.py:147
```

`degraded_mode.py:147` is **correct and deliberately left**: it is
`(time.time() - os.path.getmtime(path)) / 86400.0`, a file age in days.
A filesystem mtime is wall-clock by definition and cannot be differenced
against a monotonic clock. The rest are measurement harnesses, where a
backwards clock step corrupts the number silently — the fault that once
produced a **−2321 ms** latency. `probe_dial_safety.py`'s four are fixed as
part of its rewrite; the other five are noted, not fixed (see "not fixed").

Note `fault_injector.py`'s existing `nonmonotonic_clock` check reports "0
offenders" — it scans `src/` only, so **every one of these lives in a
directory it does not look at.** The check is true and useless.

**A4 unprefixed dual-arm resources — clean.** `audit_dual_arm_collisions.py`
PASSes all five shapes including C++ literals: no duplicate interfaces, no
shared device endpoints, "every exported resource name is composed, not
literal", 30 prefixed resources seen.

**A5 tests that construct the environment where the bug cannot occur — 1 real
instance, fixed.** `verify_autonomy_command_path.py`; see below. The new
`test_no_unresolved_module_names.py` was written specifically to avoid this
class: it parses shipped source and needs no running system.

### B. INSTRUMENT FAILURES — 3 NEW, ALL FOUND BY CROSS-CHECK

**B1. The autonomy probe measured registration, not assertion.**
`verify_autonomy_command_path.py` decided which blockers fired with a
**substring match over the raw `/blocking` payload**, which is JSON carrying
`blockers` — every blocker the unit ever *registered*. `"clearance_floor" in
msg.data` was true in **every message the follower ever published**. Ten names
would have been falsely reported as fired. *Mechanism: absence read as a
value.* Fixed: parse JSON, read `active` (+`expired`, which is not an
all-clear), and keep the reachable drive as an explicit control.

**B2. `probe_dial_safety.py` reported a frozen number as a measurement.** The
follower assigns `min_clearance` at the *end* of the publish path, **after IK
succeeds**. Aimed inside the wearer, IK fails and that line is never reached,
so the field holds the clearance from the last successful pose indefinitely.

Measured: both dial ends returned **bit-identical 0.1277 m**, and tf2 showed
the arm at **0.0000 m** from where an unrelated earlier test had left it. It
had not moved at all. *Mechanisms: absence read as a value, plus state left
from a previous run.* The zero-variance signature is the same one that
identified the dead j7 pot and the frozen `/real/joint_states`.

**Job F's headline claim is therefore RETRACTED as previously stated** — and
then re-measured honestly (below). It also published that number to
`recordings/baselines/dial_safety.json`, so a fabricated figure was persisted
as a baseline; overwritten by the corrected run.

**B3. The same probe published world coordinates to a master-frame topic.**
`/master_arm_pose_<arm>` carries a MASTER-FRAME DISPLACEMENT which the
follower maps through anchor and scale. Feeding it a world position asks for a
pose nowhere near the intended one — the identical mistake that produced Job
D's false negative. *Mechanism: order of operations inside the measurement.*

**B4 (my own, caught before reporting).** My first entry-point audit printed
`checked 0 entry points / all resolve` — the regex matched nothing and the
absence read as a pass. My first `C3` run flagged `config/kinematics.yaml` and
two others as missing; they are **package-relative vendor paths** resolved via
`FindPackageShare` at runtime. Both corrected before use.

### THE CORRECTED JOB F MEASUREMENT

A single target deep inside the body is refused outright, so the arm never
advances and a correctly-refused run is indistinguishable from a broken
harness. The approach is now **incremental** — interpolated toward the wearer
in 2 cm steps, each individually reachable — so the arm genuinely walks in and
stops at the last pose it can hold. Clearance comes from tf2 through the
project's **own** `srl_teleop.clearance`, so probe and floor cannot disagree.

| dial | vel | step | clearance | stopped from wearer | floor blocks | EE advanced |
| --- | --- | --- | --- | --- | --- | --- |
| 1.0 | 0.60 rad/s | 0.35 | **0.0857 m** | 0.4209 m | 0 | **0.1403 m** |
| 0.0 | 0.10 rad/s | 0.10 | **0.0857 m** | 0.4209 m | 0 | **0.1403 m** |

**SAFETY UNCONDITIONAL: HOLDS.** Identical stopping geometry at 6× the speed,
now with 0.1403 m of demonstrated motion behind it rather than none.

**Cross-check, and the reason to believe it:** 0.0857 m sits alongside
`probe_collision_response.py`'s independently measured 0.072–0.079 m, and both
report **0 clearance-floor blocks** — collision-aware IK stops the arm first
and the hard floor stays a backstop. Two instruments, one geometry, agreeing.

The probe now refuses to conclude when the arm does not move — verified: the
pre-incremental version correctly printed `INCONCLUSIVE — the arm did not move`
rather than passing.

### C. STALE REFERENCES

- **70 of 70 `console_scripts` entry points resolve** to a module that exists
  and a function that exists. No orphans.
- **14 baseline artefacts are written by exactly one script and read by
  nothing.** Not a defect on its own — a baseline's reader is git and a human —
  but it matters for Part 6: `make_thesis_figures.py` consumes only
  `mount_overlap_sweep.json` and `workspace_n10_20260806.json`. The other
  twelve feed no figure yet.

### D. KNOWN SPECIFIC FAULTS

| | status |
| --- | --- |
| D1 identity quaternion as "neutral" | **8 files.** Marker/scene uses are fine (orientation genuinely arbitrary). **Five drive IK and are not fine:** `probe_collision_response.py:79`, `probe_clutch_indexing.py:70`, `measure_grasp_penetration.py:94`, `drive_mode_paths.py:53`, `scripted_operator.py:111`. Fixed in `probe_dial_safety.py` only |
| D2 anything that can start a second stack | **not audited this pass** |
| D3 `/ik_status` empty array | **CLOSED** — was the dead follower |
| D4 stale SHM at startup | **FIXED** — `srl_clear_stale_shm()` in `scripts/env.sh`, called by `run_teleop.sh` and `run_autonomy.sh` |

`srl_clear_stale_shm()` clears **both** `fastrtps_*` and `sem.fastrtps_*`. The
second glob is the point: `rm /dev/shm/fastrtps_*` alone left 39 semaphores of
188 entries behind, which reads as a live writer and sends you hunting a
process that does not exist. It **refuses while a stack is running** (verified:
`3 stack process(es) running -- NOT clearing`, exit 1) because removing a
segment a live participant holds is worse than leaving it.

### E. NUMBERS NOT TRACEABLE TO A VALIDATED MEASUREMENT

1. **Job F's "safety does not scale with the dial"** — was fabricated from a
   frozen field. **Now genuinely measured** (table above).
2. **`probe_collision_response.py`'s 0.0722–0.0791 m** — provenance in doubt,
   not withdrawn. It drives with an **identity quaternion**, so a refusal there
   may be orientation-infeasibility rather than collision. The arm did visibly
   approach, and the numbers agree with today's independent 0.0857 m, so they
   are probably sound — but they should be re-run with the anchor orientation
   before being quoted again.
3. **`/ik_status` right-arm clearance reads `-1.0000`** with all counters zero.
   That is the documented sentinel for non-finite, correct here (nothing has
   driven the right arm), but it is exactly the shape of "absence read as a
   value" and any consumer must treat `-1.0` as *unknown*, never as a distance.

### NOT FIXED, AND WHY

- **Five wall-clock intervals** in `drive_mode_paths.py` and
  `measure_grasp_penetration.py`. Mechanical, but each needs its script re-run
  to confirm nothing else depended on the timing, and neither is on the
  critical path today. **`fault_injector.py`'s clock check must be widened to
  `scripts/` at the same time** — it currently reports 0 offenders because it
  only scans `src/`.
- **Four identity-quaternion drive sites.** Each needs the anchor lookup and a
  re-run of the measurement it feeds; changing them without re-measuring would
  leave numbers in CLAUDE.md whose provenance nobody could state.
- **`audit_unreachable_clear.py`'s two blind spots.** Worth fixing before its
  clean bill is relied on again.
- **D2, second-stack prevention** — not audited.
- **Follower `respawn`.** `master_pose_node` has it, the followers do not, so a
  follower that dies stays dead — which is exactly what hid this bug for two
  sessions. Deliberately NOT added: respawn would have masked the NameError
  behind a 5 s restart loop instead of a clean traceback. The right fix is that
  a dead follower be *loud*, which `blocking_aggregator`'s silent-unit
  detection already covers. Recorded as a decision, not an oversight.
