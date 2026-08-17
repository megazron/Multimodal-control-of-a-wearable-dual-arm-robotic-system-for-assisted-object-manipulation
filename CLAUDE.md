# kortex_ws — shared autonomy for a wearable supernumerary limb

Two Kinova Gen3 (7-DOF) on a backpack frame, driven from an instrumented
master mannequin arm over a Teensy 4.1, with a shared-autonomy layer.
ROS 2 Jazzy. **The arms are worn by one person and driven by a different
person**; both are participants.

**This file is the standing rules and the current state. It is deliberately
short.** The history, the worked examples and the reasoning behind every rule
live in `docs/system/`; each rule below points at where its detail is.

## READ THIS BEFORE YOU START READING

* **Read with `offset` and `limit`. Do not re-read a file you have already
  seen this session.** Read results have hit a third of the context window,
  which is time and money spent on text already in front of you.
* Prefer `grep -n` to opening a file. Prefer opening 40 lines to opening 4000.
* This file was 98k tokens on 2026-08-12 and is now under 15k. Keep it that
  way: new findings go in `docs/system/findings.md`, not here.

---

## HOW TO RUN THINGS

Source first, in every terminal:

```
source /opt/ros/jazzy/setup.bash && source ~/kortex_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=SHM        # NOT optional. See wsl.md
```

```
ros2 run srl_teleop gui                 # Qt operations GUI  <-- USE THIS
ros2 run srl_teleop teleop_gui          # curses, SSH / no display only
bash scripts/run_teleop.sh              # teleoperation only
bash scripts/run_autonomy.sh            # + perception + shared autonomy
bash scripts/run_experiment.sh m1 --participant PILOT --scripted
bash scripts/calibrate.sh zero|gyro|imu-mount|max-position
bash scripts/diagnostics.sh             # before blaming the code
python3 scripts/sim_session.py --stack teleop --keep-up -- true
```

Three-terminal split for real work:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh          # --mock to rehearse
```

**`gui` is primary.** `console` and `launcher` are superseded and absorbed;
do not start new work in them. Task keys are `m0 m1 m1s2 m2 m3` (displayed
T0–T3); `t*`/`e*` are the archived sets and are refused by name.

### First action of every lab session

```
bash scripts/check_channels.sh          # ~3 min, block A only
```

Which master channels are FROZEN is decided from the **newest**
`recordings/baselines/channels_*.json`. A baseline older than the wiring makes
the software freeze channels that now work, and nothing downstream disagrees.
Target: 12 or more of 14 coherent.

---

## CURRENT STATE, 2026-08-15

Re-measure rather than trust this table; each row names its command.

| | state | check |
| --- | --- | --- |
| unit tests | 614 pass, 2 allowed failures (`test_flake8`, `test_pep257`, pre-existing: the package uses double quotes). `check_tests.py` is the gate and knows the allowlist; `python3 -m pytest -q src/*/test` is the raw run | `python3 scripts/check_tests.py` |
| **home pose** | **the presentation pose since 2026-08-15**: wrists level, hands in front of the chest. Task set re-measured first — with the ANCHOR unchanged every task performs exactly as before. **The real arms are still at the legacy home**; the bridge refuses to enable at 1.94 rad and says why | `recordings/baselines/home_change.json` |
| **where home is stored** | **FIVE places, not two.** `config/home_positions_*.txt` is the SOURCE (5 live consumers incl. the bridge gate and the real homing target); the URDF's two blocks are where the sim spawns; `pot_bridge.py` and `srl_teleop_node.py` had their own hardcoded copies and now load it; `real_home_reference.txt` is reference-only and read by nothing | `test_home_has_one_source.py` |
| **a run starts from HOME** | **it did not, from the SECOND run of a session onward.** Nothing returned the arms to home when a run ended and nothing checked at the start, so run 2 began 1.2338 rad (left) / 0.6248 (right) out — measured at the first commanded waypoint. Boot and run 1 both read 0.0000. `run_abc.require_home()` now stages before the task and REFUSES rather than running from an unknown pose; `--allow-unhomed` overrides, loudly. The sweep always staged, so the clips were never affected | `test_run_starts_from_home.py` |
| **T1 from a typed sentence** | **works, and the colour is the CAMERA'S.** `run_abc --instruct "put the green ones on the green mat"`, or `scripts/instruct_t1.py` for the whole look-parse-ground-run path in one command. 40 phrasings: **16 correct / 5 asked / 19 refused / 0 MISUNDERSTOOD**, against 0/2/38/0 for the shipped grammar. A singular request matching two cubes ASKS; it never chooses | `scripts/sweep_t1_instructions.py` |
| task layer | the same waypoints are **COMMANDED** under every mode — `run_abc` builds them with no mode argument. **The robot does NOT perform identically**: measured 2026-08-15 with no operator, T0's left path 1.407 m under 01 against 2.034 m under 03 | `docs/system/findings.md` |
| **02_vr_teleop** | **cannot grasp.** All three of its grasping tasks fail; the pads miss by 88 → 206 mm against a 30 mm gate, and the miss ACCUMULATES. The other four modes close at 0.0000 m | `recordings/verification/02_vr_teleop/T1/*/scene_events.json` |
| **the VR mapper** | `vr_pose_mapper` **holds the arms**, so the staging move cannot execute while it runs. Isolated with a control either side. The sweep stops it for staging and re-isolates | `docs/system/findings.md` |
| **T2's tray** | **FIXED 2026-08-16.** It was elastic — drawn as the LIVE `separation + 0.06`, so it ran 0.487–1.211 m against a 0.560 m spec and T2-1 and T2-2 could not fail. Now drawn at the spec length and oriented along the gripper line, and the ball falls off past 6.8 deg and STAYS off | `src/srl_experiments/test/test_t2_tray_is_rigid.py` |
| accuracy table | reproducible from committed data; grasp NOT uniformly 100% (06 reads 50%, VR 75%) | `scripts/accuracy_table.py` |
| status | 25 clip dirs, 19/19 planned data cells | `scripts/status_table.py` |
| clips | **RE-RECORDED 2026-08-16**: 25 cells (5 tasks x 5 modes) plus the three routines, eight angles and a card each, every one opening on the presentation pose, on the current geometry. 31 clip dirs, 19/19 planned data cells, no real gaps. The old set is in `archive/recordings/verification_20260815_part6` and is still the evidence for finding 3 | `scripts/status_table.py` |
| **T2 in the pixels** | **the tray is rigid now and NEITHER GRIPPER IS ON IT.** The grip trace reads 0.00 knuckle for all 992 samples on both arms while the schedule commands `grip_for(30)` on every waypoint. T2-1 "held by BOTH grippers" is false and has been for the life of the task; the elastic tray hid it by stretching to meet whichever hands existed | `recordings/verification/*/T2/*/grip_trace.json` |
| **the dance** | **d1 and d3 could not be filmed** until 2026-08-16: 45 of 382 and 49 of 554 waypoints unreachable, because the envelope was \|x\| 0.26–0.50 against per-arm limits of 0.325 and 0.450. Moved to 0.47–0.76 as ONE symmetric band; re-verified 0 of 1298 failing | `scripts/verify_dance_paths.py` |
| **the table** | **WHITE in the picture, not just in the request**: renders 238 of 255 against the old 118. A marker's ambient is 0.5 x colour and RViz does not clamp, so the top asks for 1.88. The risers are gone from every MSc task | `scripts/probe_marker_shading.py` |
| **grasp approach** | every grasp uses the **pinned near-side** wrist — `run_abc.send()` writes the anchor into every waypoint of every mode. Top-down is WORSE for T1 (loses all four place points on wearer clearance). **T3's circuit box presents 195 mm of box to an 85 mm hand and cannot close**; T2's right grip 114 mm | `recordings/baselines/grasp_approach.json` |
| **wearer clearance, per task** | only T1 stage 1 is clean. **T0 breaches by ~142/137 mm (and 4 IK failures per arm), T2 by 150/147 and its LEFT arm now fails 1 waypoint, T3's left arm by 88**. T1 stage 2 seed 0's right arm is CLEAN as of 2026-08-17 | `recordings/baselines/home_change_applied.json` |
| **predictive avoidance** | **the null space is now REAL** — Jacobian by finite differences, N = I − J⁺J, clearance gradient, closed-loop task-space correction (open-loop drifted the hand 5.5 mm; closed-loop is under 0.1 mm). **And it is still worth 0.0 to 5.6 mm.** Every one of 28 refusals names `end_effector_link`: the HAND is inside the person, and the null space holds the hand fixed by definition. Lookahead 0.30 s, decision rate ~7 Hz through the services, EE residual 0.00001 m, never locked up. Tangential reroute rescues 1 step per target, sliding 47–57 mm | `recordings/baselines/predictive_avoidance.json` |
| **the home pose, in PIXELS** | **the render was right and the numbers were about something else.** Measured from TF at boot: elbows **0.0793 (L) / 0.2715 (R)** below their shoulders, the LEFT elbow **188 mm outboard of its own hand**, hands 1.10 m apart against a 0.36 m torso, forearm mirror residual **0.2826 m**. The wrists ARE level (−0.01 deg) and always were. NOT a cached config: the joint state matches the source file to 7e-5 rad | `recordings/baselines/home_render.json` |
| **can the pose be fixed** | **partly.** Innermost hand column that works for both arms: **\|x\| = 0.450** (torso half-width 0.18), so "hands at torso width" is the TORSO again. Best mirror residual **0.161–0.164 m and it does not improve with more branches** (4 vs 31 changes it by 1 mm), so symmetry is structural: the mounts' base axes mirror exactly but differ by **168 deg of roll**, and two identical arms on mirrored mounts cannot mirror. Left elbow can drop 91 mm. **NOT applied** — home moves only after the task set is re-measured | `recordings/baselines/symmetric_home.json` |
| **the centre, and WHY** | **the TORSO, not the wearer's arms.** The arm posture is now a variable (`SRL_WEARER_ARMS`, five settings, one source). With the wearer's arms deleted entirely the centre is still shut: min \|x\| 0.300 (L) / 0.375 (R), every column 0.125–0.300 binding on the torso, and −0.007/−0.013 m at \|x\| = 0.10. Arms clear buys 25 mm (L) and 75 mm (R). `folded` costs 75 mm; `behind` and `out` breach the floor at HOME | `recordings/baselines/centre_vs_wearer_posture.json` |
| **the mount caps clearance** | `base_link` sits **0.1610 m** from the torso and no joint moves it, so the 150 mm floor has 11 mm of headroom before the arm does anything. A clearance reading of exactly 0.1610 is the MOUNT, not the arm | `wearer_posture.py` |
| **innermost safe column** | **left 0.325, right 0.450** at z = 1.120, N=10 over the full path. The stored 0.425/0.400 in `centre_vs_height.json` is STALE: re-running that script unchanged returns 0.325/0.450 today, because the home change moved the IK seed | `scripts/measure_centre_vs_height.py` |
| **the runtime wearer had no arms** | `clearance.py` — what the follower, homing node and bridge enforce — modelled torso, head and hips only. The wearer's arms are what bind the RIGHT arm inboard. Added, with a known-answer test | `test_wearer_posture_has_one_source.py` |
| **centre, ON the surface** | **searched properly, 3360 cells, table height AND distance swept with objects resting on it: no configuration puts \|x\| <= 0.10 on the surface.** Closest confirmed at N=10: **x = 0.350** (left arm, table top 1.00, near edge 0.530, clearance 0.1524). Right arm reaches only 0.450, reach-only. **Every survivor needs the object at the very FRONT EDGE** — overhang 0.00 — because the pinned wrist arrives from below | `scripts/search_centre_on_surface.py` |
| **top-down on a surface** | **0 of 840 cells**, any table height 0.70–1.10, either arm. Not a trade against centre-on-surface: centre-on-surface is achievable at 350–450 mm off centre, top-down-on-surface is not achievable at all | `recordings/baselines/centre_on_surface.json` |
| workspace marking | **an OUTLINE, not 192 filled tiles** — same set, not a bounding box: an edge wherever a cell adjoins a non-cell, 20 bars against 196 markers. **It is drawn at the plane it BOUNDS, per arm** (`work_top_for`): painting it on the table instead put it 116.5 mm below the pads and the frame showed the task outside its own boundary. Now 1.1015 against a pad top of 1.1000 | `clip_scene.region_outline`, `marking_z` |
| **T1-1, and the fourth measurement** | **objects cannot rest on the table, measured four ways AT THE ANCHOR.** 0 of 3360 resting cells; every (top, edge) cell fails for T1's own x incl. an edge at y = 0; best free pair 0.980/0.100 leaving **120 mm**; and **support lips cost 109 of 171 waypoints** against 0 with none — the old home-wrist note said 22, so supports are 5x worse than recorded, not better | `SUPPORTS_ENABLED` note in `clip_scene.py` |
| **the on-table check can fail AND pass** | `--inject-float-mm` displaces the scene so one instrument answers three ways: 0 → BLOCKED exit 3, +5 mm → DRIFT exit 1, **−120 mm → PASS exit 0**. Exit 0 is reserved for RESTING. It also read T1's cubes when asked about T1S2 (`T1_CUBES if task=="t1" else T1_CUBES`) and now reads stage 2's own seeded layout | `test_objects_on_table_can_fail.py` |
| **a render check that could not fail** | removing the bounding box left the label reading its corners, so `tick()` raised on frame 1 and the scene came up with **no table, cubes, pads or marking — filed OK at 18% ink, because the WEARER is 18% of the frame** (46–50% when the scene is really there). `render_t1_scene` now SUBSCRIBES to `/task_objects` and refuses to shoot without the task's namespaces | `scripts/render_t1_scene.py` |
| **two clients, one planning scene** | a verifier run that overlapped a still-running sweep read **18 of 171** where two clean runs read 0 — the sweep was applying and removing its own slab in the same scene. HARD CONSTRAINT 3 one level down: never run a sweep and a verifier against one `move_group` | `docs/system/findings.md` |
| T1 | stage 1 is the **LEFT** arm, cubes on the left. **Re-verified 2026-08-17 AT THE ANCHOR on the raised surface: stage 1 clean — 0 IK failures, 0 below the floor, clearance 0.1610, 171 waypoints at N=10, reproduced twice, plus every per-object pick and both pad slots.** Stage 2 seeds 0 and 1 clean; **seed 2's left arm is 16 of 91** (see its own row). The earlier "all three stage-2 seeds" was measured at the home wrist | `scripts/verify_t1_paths.py` |
| **T1 cannot be re-recorded** | **the clip loses cube_3 and the cause is the VISION path, not the task.** T1 builds its path from DETECTED cubes; in-sweep those are 28–31 mm off truth against a 30 mm gate, standalone on the same stack 1.3–3.0 mm (baseline reproduced to 4 dp, 3 runs). `Vision.stage()` accepts 0.02 rad PER JOINT and `ik_follower_node` is on the same controller, so the arm settles at the edge of the tolerance and vision deprojects from a pose it never reached. Two clips recorded and DISCARDED | `docs/system/findings.md`, 2026-08-17 |
| wearer clearance | **the region and every T1 coordinate are now checked against the 150 mm floor GEOMETRICALLY.** `avoid_collisions` cannot see it — the SRDF excludes the pairs that matter — and the previous layout spent 70 of 143 waypoints inside it at 0 IK failures | `scripts/measure_clearance_region.py` |
| workspace marking | re-surveyed 2026-08-15: the marking is now the **clearance-safe** cells, and the box runs to \|x\| = 1.00 (the old 0.70 was the survey box, not the arm) | `recordings/baselines/work_surface_region.json` |
| what limits the workspace | forward and outboard: the **pinned wrist**. Inboard: **the wearer**. Down: the **table**. No direction is bound by a joint limit | `docs/TASK_SPEC.md` §2A |
| grasp pose | pad offset IS applied — `PAD_OFFSET` in `clip_tasks.py`, tips +0.098 m along the tool axis | `recordings/baselines/pad_clearance.json` |
| **the wrist every sweep measured at** | **it was the HOME wrist, not the anchor the task sends — 32.26 deg apart on T1's arm.** `measure_what_binds.Rig` read the live EE orientation off TF at construction; `run_abc.send()` writes `WORKSPACE_ORIENT`. EVERY reachability sweep in the repo solves through `Rig`. Now `anchor="workspace"` by default. T1's stage-1 layout survives the fix (0 failures, N=10, twice); `centre_on_surface.json`'s 22 hits do not | `docs/system/findings.md`, 2026-08-17 (later still) |
| **two surface heights, 150 mm apart** | **`BENCH_TOP` 1.10 positioned every object, `TABLE_TOP` 0.950 was the only geometry, and nothing compared them.** One owner now: `work_surface.WORK_PLANE_M` 1.100 / `DECLARED_M` 0.980 / `DECLARED_NEAR_Y` 0.100 / `FLOAT_GAP_M` 0.120. Surface raised 30 mm, gap 150 → 120 mm | `test_one_work_surface_height.py` |
| **T1-1, objects on the table** | **BLOCKED, and measured three ways at the anchor.** 0 of 3360 cells with objects RESTING; T1's own x fails at every top 0.900–1.100 including a near edge at y = 0; and height vs near edge TRADE — best free pair 0.980/0.100. The pick-only sweep called 1.020 free and it costs the FULL path 18–36 of 171 | `scripts/sweep_surface_vs_t1_path.py`, `scripts/verify_objects_on_table.py` |
| **T1S2 seed 2** | **16 of 91 IK failures on the LEFT arm, and always did** — identical at 0.980 and at the old 0.950, so it is the anchor fix not the surface. Seeds 0 and 1 clean; the sweep records `--seed 0`, so no clip is affected | `scripts/verify_t1_paths.py --part stage2` |
| table height | owner exists and detects ±20 mm; **nothing calls `set_measured()` from depth** | `srl_experiments/work_surface.py` |
| silent faults | 3–4 open, listed at the top of `03_real_robot_bringup.md` | `scripts/inject_lab_day_faults.py` |
| real hardware | **nothing in this repo has ever run against a real arm** | |

**Blocked on the lab:** 7 of 14 master channels incoherent (a soldering
problem; `l_j2` and `l_j4` named by regression); the two arms' reachable sets
are disjoint so T4 handover needs the right arm re-parked in hardware; the
right arm's home joint values were never read from hardware; two real Kortex
sessions have never been opened; detection rate at working distance is
unmeasured; no `adb`; this machine is off the lab network.

**Blocked on ethics:** no human data has been collected at all. Operator and
wearer are different people and consent separately. Worn operation is >17 kg
with no gravity compensation on someone who did not choose the motion.

---

## HARD CONSTRAINTS. Violating one of these causes real damage.

0. **SIM AND REAL HOME DISAGREE ON PURPOSE, since 2026-08-15.** Sim home is
   now the presentation pose (wrists LEVEL, hands in front of the chest); the
   physical arms are still at the legacy Kortex home and the wrists there
   point UP by +85/+79 deg. The bridge REFUSES to enable on the ~1.9 rad
   difference rather than commanding it — verified in `enable()`. **Capture
   the new pose on the arms before any real session**; values in Kortex
   degrees are in `NEXT_SESSION.md`. → `home_wrist_is_real.md`
1. **Home joint angles are ground truth, and the REAL arm's are the ones that
   count.** The sim's were changed deliberately on 2026-08-15, after
   re-measuring the whole task set; the arms' were not. Never change either to
   make a pose look right without re-measuring first — the sim→real bridge
   replays sim angles onto the real arm and any difference it does not catch
   is commanded as a jump. **The ANCHOR is a DIFFERENT quantity**: the 30.7
   deg approach direction is a stored constant that nothing recomputes from
   home. Moving home cost nothing measurable; re-deriving `WORKSPACE_ORIENT`
   to match a level home costs **T2 its right arm**, so never do it.
   → `home_wrist_is_real.md`
2. **The arm permits exactly ONE Kortex session.** SIGINT the bridge; SIGKILL
   leaks the session and the next run cannot connect. Grep for
   `kortex session closed cleanly`. → `wsl.md`
3. **Never run two stacks.** Two `master_pose_node` split the serial stream
   and invalidated a full day of measurements. The GUI refuses and names the
   PIDs. A stray `robot_state_publisher` does the same damage and is not
   caught by the count. → `architecture.md`
4. **The Kortex cyclic path is unusable over WSL.** Use the high-level API
   (`kortex_highlevel_bridge`). Every cyclic write is a network round trip.
   Do not spend another day on the ros2_control driver. → `wsl.md`
5. **`FASTDDS_BUILTIN_TRANSPORTS=SHM`.** UDP discovery is dead on this host.
   Clear `/dev/shm/fastrtps_*` with the stack STOPPED, then relaunch.
6. **Never `wait_for_service` in `estop_node.trip()`.** It once blocked the
   e-stop for 4.0 s. Park first, then talk to the driver. → `findings.md`
7. **The dependency arrow runs one way.** `srl_teleop` imports nothing
   in-repo. It is the baseline condition of every experiment; if it imported
   `srl_autonomy` the comparison would be circular.
8. **`real_robot` mode must be armed.** `motion_enabled=false` until set by
   hand, and it refuses to start with a continuous joint wound beyond ±π.
9. **Never `pkill` broad patterns while a stack you want is running.** Kill an
   explicit PID list, and kill the process GROUP.
10. **Wrap ROS `setup.bash` in `set +u` / `set -u`** — it reads unbound
    variables and dies otherwise.
11. **The wearer is in the collision model and the clearance floor is the last
    thing between the arms and a person's chest.** Do not lower it globally.
    An SRDF exclusion silences an alarm; it does not move the metal.
    **`avoid_collisions` IS NOT THE WEARER CHECK** — the SRDF excludes the 44
    proximal pairs a shoulder mount actually threatens, so a `valid` pose can
    have the tube inside the person. Measure clearance geometrically, and do
    not cite a workspace figure without checking
    `docs/system/clearance_gap_ledger.md` for whether it survived.
    **The WEARER'S POSTURE is part of the model and it is a variable now**
    (`SRL_WEARER_ARMS`, default `down`). It has ONE source, `wearer_posture.py`,
    which writes the URDF's arm block and builds `mount_guard_node.WEARER`; a
    posture that reaches one and not the other measures the old wearer under a
    new name. Do not tell a wearer to clasp their arms behind their back or
    hold them out to the sides: with a BACK-mounted rig both put their own
    limbs inside the floor at the HOME pose. → `findings.md`, 2026-08-15
12. **Anonymity is enforced in code.** `write_manifest()` raises on `name`,
    `email`, `dob`, `address`, `phone`.
13. **A demonstration is not evidence.** Demo clips carry that caveat in the
    caption and produce no trial data, no condition and no metric.

---

## THE STANDING RULE

**Before reporting a bad measurement, validate the measuring tool against
ground truth.** A surprising failure is evidence about the INSTRUMENT until
the instrument has been cleared. This has been the fault **seventeen times**.

* Every analysis script gets a known-answer test.
* **A check that cannot fail on a deliberately broken input is not a check.**
* Prefer real data with known ground truth. Synthetic only where the ground
  truth is CONSTRUCTED (arithmetic, geometry), never RENDERED.
* A result that contradicts an earlier measurement is an instrument check, not
  a finding, until the two are reconciled.
* Repeat before believing. TRAC-IK restarts randomly; one call is one flip.
* **N repeats over the WHOLE PATH.** A pose passing one IK call is not
  reachable. N=10, densified to 20 mm, and stay 20 mm inside the last pose
  that passed N/N. MARGINAL is not usable.

### Instrument failure modes seen here. Check this list first.

| symptom | mechanism |
| --- | --- |
| statistic is 0.000 everywhere | recorder oversamples a slower sensor; adjacent rows identical |
| zero variance | dead channel, or a cached value republished |
| data fresh but never changes | stale republished with a NEW timestamp; key on the SOURCE stamp |
| learned model scores near zero | synthetic renderer out of distribution |
| metric swings between runs | min-over-directions; use medians |
| everything matches | substring/prefix matching (`t0` inside `t001`; `t1s2`.startswith(`t1`)) |
| results depend on run order | scene or state persisted; `remove_furniture()` clears only ITS ids |
| every pose returns one value | FK evaluating the CURRENT state, ignoring the solution given |
| feature "present" but does nothing | checked that a field is STORED, not that a consumer READS it |
| a gap that is not a gap | by-design absence rendered identically to a defect; use `by_design.py` |
| clip verifier fails a good clip | matched requested RGB, not RENDERED colour; RViz shades everything |
| test breaks with no behaviour change | test sliced source "before the first def" |
| success count identical either way | a tolerance so loose it binds nothing; measure the ACHIEVED value |

---

## LAYOUT

| package | role |
| --- | --- |
| `srl_teleop` | teleoperation ONLY: master sensing, IK follower, clutch, scaling, sim→real bridge, e-stop |
| `srl_perception` | AprilTag, 6-DOF object pose, mock RGB-D |
| `srl_autonomy` | grasp generation, intent inference, handover arbitration |
| `srl_experiments` | tasks, logging, conditions, analysis, work-surface owner |
| `srl_description` | `srl_dual.urdf.xacro` — arms, mount, wearer |
| `srl_moveit_config` | TRAC-IK, controllers, SRDF |

Build: `colcon build --symlink-install`, 16 packages, no flags.
`robotiq_driver` and `robotiq_hardware_tests` are `COLCON_IGNORE`d (missing
`serial`); nothing in `srl_*` depends on them. All `srl_*` are
symlink-installed, so `.py` edits take effect **on node restart** — if a change
seems not to apply, suspect a stale PROCESS, not a stale install.

---

## WHERE THE DETAIL IS

| file | holds |
| --- | --- |
| `docs/system/findings.md` | the diagnostic history and every worked example: mount geometry, hardening passes, the reachability and clearance measurements, the experiment programme, the two-person reframing |
| `docs/system/hardware.md` | Teensy and serial, channel health, spherical position and AXIS_MAP, gyro azimuth, calibration, clutch, grippers, degraded mode, the virtual Teensy |
| `docs/system/wsl.md` | mirrored networking, the cyclic-path finding, `/mnt/c`, `/dev/shm`, the Quest transport, x11grab and Xvfb capture |
| `docs/system/architecture.md` | packages and topics, the IK follower, operating modes, VR stack, the three GUIs |
| `docs/system/clearance_gap_ledger.md` | **which earlier workspace and clearance numbers the SRDF gap invalidated and which stand. Read before citing any workspace figure.** |
| `docs/system/home_wrist_is_real.md` | **the home pose CHANGED on 2026-08-15.** What moved, what deliberately did not, why the real arms are still at the old pose and what stops that becoming a jump |
| `docs/system/03_real_robot_bringup.md` | **the lab-day fault table and the camera framing note. Read before hardware.** |
| `docs/system/06_troubleshooting.md` | keyed by SYMPTOM |
| `docs/NEXT_SESSION.md` | what to do next, in order |
| `docs/WORK_BRIEF.md` | the standing multi-part brief |
| `docs/research/` | literature, hypotheses, protocol, ethics, the grasping and approach-geometry findings |

**Starting fresh?** This file, then `docs/NEXT_SESSION.md`. Go to the others
only when a rule above sends you.
