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
| unit tests | 509 pass, 2 allowed failures (`test_flake8`, `test_pep257`, pre-existing: the package uses double quotes). `check_tests.py` is the gate and knows the allowlist; `python3 -m pytest -q src/*/test` is the raw run | `python3 scripts/check_tests.py` |
| task layer | the same waypoints are **COMMANDED** under every mode — `run_abc` builds them with no mode argument. **The robot does NOT perform identically**: measured 2026-08-15 with no operator, T0's left path 1.407 m under 01 against 2.034 m under 03 | `docs/system/findings.md` |
| **02_vr_teleop** | **cannot grasp.** All three of its grasping tasks fail; the pads miss by 88 → 206 mm against a 30 mm gate, and the miss ACCUMULATES. The other four modes close at 0.0000 m | `recordings/verification/02_vr_teleop/T1/*/scene_events.json` |
| **the VR mapper** | `vr_pose_mapper` **holds the arms**, so the staging move cannot execute while it runs. Isolated with a control either side. The sweep stops it for staging and re-isolates | `docs/system/findings.md` |
| **T2's tray** | **elastic** — drawn as `separation + 0.06` between the grippers, so it runs 0.487–1.211 m against a rigid 0.560 m spec. T2-1 and T2-2 cannot fail, and the ball never drops | `scripts/clip_scene.py` ~1318 |
| accuracy table | reproducible from committed data; grasp NOT uniformly 100% (06 reads 50%, VR 75%) | `scripts/accuracy_table.py` |
| status | 25 clip dirs, 19/19 planned data cells | `scripts/status_table.py` |
| clips | **re-recorded 2026-08-15 on the current geometry**: 25 cells, 5 tasks x 5 modes, eight angles and a card each | `scripts/status_table.py` |
| workspace marking vs the table | the marking's nearest row (y = 0.075) is **25 mm in front of the table's near edge (0.100)** — 43 of 439 cells drawn over air, and stage 2 places cubes there | `scripts/clip_scene.py` `TABLE_NEAR_Y` |
| T1 | stage 1 is the **LEFT** arm, cubes on the left; N=10 full path, 0 IK failures, 0 waypoints inside the wearer clearance floor | `scripts/verify_t1_paths.py` |
| wearer clearance | **the region and every T1 coordinate are now checked against the 150 mm floor GEOMETRICALLY.** `avoid_collisions` cannot see it — the SRDF excludes the pairs that matter — and the previous layout spent 70 of 143 waypoints inside it at 0 IK failures | `scripts/measure_clearance_region.py` |
| workspace marking | re-surveyed 2026-08-15: the marking is now the **clearance-safe** cells, and the box runs to \|x\| = 1.00 (the old 0.70 was the survey box, not the arm) | `recordings/baselines/work_surface_region.json` |
| what limits the workspace | forward and outboard: the **pinned wrist**. Inboard: **the wearer**. Down: the **table**. No direction is bound by a joint limit | `docs/TASK_SPEC.md` §2A |
| grasp pose | pad offset IS applied — `PAD_OFFSET` in `clip_tasks.py`, tips +0.098 m along the tool axis | `recordings/baselines/pad_clearance.json` |
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

0. **The home wrist points UP by +85 deg (left) and +79 deg (right). That is
   REAL, read from the physical arms, and it is not a rendering fault.** The
   anchor's 30.7 deg is a DIFFERENT quantity measured from the shoulder and
   the two are not comparable. → `home_wrist_is_real.md`
1. **Home joint angles are ground truth.** When the geometry looks wrong, the
   mount is the suspect. Never change the home angles to fix a pose; the
   sim→real bridge replays sim angles onto the real arm and any difference is
   commanded as a jump. → `findings.md`
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
| `docs/system/home_wrist_is_real.md` | why the home wrist points up, what changing it would cost, and why 30.7 deg is a different number |
| `docs/system/03_real_robot_bringup.md` | **the lab-day fault table and the camera framing note. Read before hardware.** |
| `docs/system/06_troubleshooting.md` | keyed by SYMPTOM |
| `docs/NEXT_SESSION.md` | what to do next, in order |
| `docs/WORK_BRIEF.md` | the standing multi-part brief |
| `docs/research/` | literature, hypotheses, protocol, ethics, the grasping and approach-geometry findings |

**Starting fresh?** This file, then `docs/NEXT_SESSION.md`. Go to the others
only when a rule above sends you.
