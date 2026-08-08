# kortex_ws — shared autonomy for a wearable supernumerary robotic limb

## HOW TO RUN THINGS

```bash
source /opt/ros/jazzy/setup.bash && source ~/kortex_ws/install/setup.bash

ros2 run srl_teleop console       # Dear PyGui dashboard  -- PRIMARY
ros2 run srl_teleop launcher      # tkinter launcher      -- simpler
ros2 run srl_teleop teleop_gui    # terminal console      -- SSH / no display
```

Everything else — every mode, experiment, calibration and diagnostic — is a
button inside them. The console refuses to start a second stack and says why:
two `master_pose_node` instances split the serial stream and invalidated a
full day of measurements.

Experiments from the command line:

```bash
scripts/run_experiment.sh t3 --participant P01 --condition direct --scenario S1
scripts/run_experiment.sh t7 --participant P01 --dry-run
```

## The six operating modes

Two axes: what drives the arm, and how much the robot decides.

| # | mode | input | robot decides | max vel | state |
| --- | --- | --- | --- | --- | --- |
| 1 | DIRECT_MANNEQUIN | master arm | nothing | 0.60 | works |
| 2 | DIRECT_VR | Quest | nothing | 0.60 | works against the mock |
| 3 | ORIENTATION_ASSIST | master + vision | wrist | 0.60 | **stub** — this `/compute_ik` plugin ignores `OrientationConstraint` |
| 4 | SHARED_AUTONOMY | master + vision | wrist, target, approach | 0.60 | works |
| 5 | SUPERVISED_AUTO | point / voice | + grasp, transport, place | **0.25** | works |
| 6 | FULL_AUTONOMY | voice | + target | **0.15** | works; **perception models unmeasured** |

Autonomy runs **slower** than teleop by design: nobody is watching, so the
only bound on a wrong motion is how long it takes to happen. All six share one
safety stack — collision-aware IK, clearance floor, graduated avoidance, joint
wrapping, e-stop, dead-man — and `assert_safety_invariant()` raises on any
transition that cannot prove it.

## What works

Real dual-arm control at **21.4 Hz per arm** (two simultaneous Kortex
sessions, no halving). Homing to within tolerance with residuals scattering
0.037–0.126°. Grippers on the Kinova **internal bus**, over the existing
session. Clutch indexing **unbounded** — 338.8 mm over 6 cycles, re-engage
jump 0.29 mm mean. Graduated collision avoidance, held at 0.072–0.079 m with
zero hard-floor blocks. E-stop trips on a frozen-but-publishing master in
0.94 s. Up/down and fore/aft tracking. The full autonomy pipeline, 10/10
mode-6 checks. The Dear PyGui console at 0.80 ms median frame time.

## What does not

**Lateral tracking** — azimuth comes from j1 alone and couples with arm bend.
**Left-arm radial motion** — `l_j2` and `l_j4` are incoherent, and no reach
observable survives them (R² = 0.133). **T4 inter-arm handover** — blocked by
geometry, 0 of 16 transfer points reachable by both arms. **Mode 3.**
**Detection rate** — unmeasured, so mode 6 is not participant-ready.
**Voice input** — `/dev/snd` holds only `timer`. **VR on hardware** — `adb`
is installed nowhere yet.

## Hardware state

7 of 14 master channels are **INCOHERENT** (over 5% of updates jumping >60°,
faster than any hand). Degraded mode freezes them and runs on what is left.
Baseline: `recordings/baselines/channels_20260806.json`. **Repairing `l_j2`
and `l_j4` buys the most** — see `docs/NEXT_SESSION.md`.

> **Every number in the protocols is IK feasibility in simulation.** Nothing
> in the bimanual programme has been driven by a human through the master arm,
> and the DIRECT condition of every task depends on those incoherent channels.

## WSL specifics that cost days

**Mirrored networking is MANDATORY.** In `%UserProfile%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown`. The Kortex driver opens TCP 10000 for config **and UDP
10001 for the realtime cyclic channel**. Under WSL2's default NAT the UDP
channel times out: writes take **3–6 seconds**, the controller manager
overruns permanently, and feedback freezes at one distinct value per joint
while `ros2 control list_controllers` still reports every controller
"active". The arm looks faulted and is not.

**The cyclic path is unusable here regardless — use the high-level API.**
Cyclic control is built for a 1 kHz loop on a dedicated link; over WSL every
write is a network round trip costing ~10 ms, so at 100 Hz the write alone
consumes the entire cycle budget. `SendJointSpeedsCommand` over the TCP
session is a *velocity* command — it holds a motion between sends, so it does
not need 1 kHz. Measured end to end: **18.4–18.7 Hz**, send latency ~26 ms,
tracking error 0.01–0.15°. That is far more than the motion needs.

Two traps already paid for: never apply the velocity law twice (the bridge
uses feedforward + feedback, and the deadband suppresses the *correction*
only), and never use `time.time()` for intervals — the WSL wall clock steps
backwards on host resync and once produced a **−2321 ms** latency.

**One Kortex session only.** The arm permits exactly one; a leaked one blocks
the next run. SIGINT the bridge, never SIGKILL.


Two Kinova Gen3 7-DOF arms on a backpack frame, teleoperated from an
instrumented mannequin arm, with a shared-autonomy layer that supplies the
degrees of freedom the wearable master physically cannot measure. ROS 2 Jazzy.

**If you have never seen this project before, read this page and then
`docs/system/01_architecture.md`.**

---


## Getting a buildable checkout

This repo tracks what we wrote. Three categories are deliberately **not**
committed, and this is how to get them back.

### 1. Vendor ROS packages — clone, then patch

```bash
cd src
git clone -b main   https://github.com/Kinovarobotics/ros2_kortex.git
git clone -b main   https://github.com/Kinovarobotics/ros2_kortex_vision.git
git clone -b main   https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
cd ..
# From the REPO ROOT. The patch paths already begin with src/, so
# `git apply --directory=src` doubles it into src/src/ and every hunk fails.
# --fuzz absorbs upstream line drift: these are cut against a moving `main`.
for p in patches/000*.patch; do patch -p1 --forward --fuzz=3 -i "$p"; done
```

**Verified from a clean clone on 2026-08-08.** 0001 and 0002 applied exactly;
0003 needed `fuzz 3 (offset -10 lines)` because `ros2_robotiq_gripper` `main`
has moved since it was cut. If a hunk ever fails outright, the change each
patch makes is one line of prose in `patches/README.md` and can be redone by
hand — they are small.

Confirm they landed:

```bash
grep -n 'prefix}reactivate_gripper' \
  src/ros2_robotiq_gripper/robotiq_description/urdf/2f_85.ros2_control.xacro
grep -n 'prefix + "reactivate_gripper"' \
  src/ros2_robotiq_gripper/robotiq_driver/src/hardware_interface.cpp
```

The patches are **required** for dual-arm operation — without them both arms
register the same unprefixed hardware resources and the second gripper
exports no command interface. See `patches/README.md`.

### 2. `src/serial` — already here, vendored on purpose

`src/serial` is committed as **plain files**, not a submodule. It is a
612 KB checkout of `tylerjw/serial` branch `ros2` at commit `d8d1606`, with
its `.git` removed. Reasoning and the command to refresh it are in
`src/serial/VENDORED.md`.

It is a build dependency of `robotiq_driver`, which is currently
`COLCON_IGNORE`d for unrelated reasons, so nothing in the default build
needs it today.

### 2b. Build it

```bash
colcon build --symlink-install
```

**Verified from a clean clone on 2026-08-08: 21 packages, 0 failures.**

Note that a fresh clone builds *more* than the original working tree did.
`robotiq_driver` and `robotiq_hardware_tests` were `COLCON_IGNORE`d there
because the `serial` CMake package was missing; vendoring `src/serial` into
this repo removed that obstacle, so both now build. Those `COLCON_IGNORE`
files were never tracked (they live inside a gitignored vendor package), so
they do not come across.

### 3. Python environments and model weights — rebuild, do not clone

Both venvs are gitignored. `.percep_venv` alone is 5.6 GB.

```bash
# Kinova API. protobuf 3.5.1 is broken on py3.12 and shadows the one ROS
# needs, so it lives in its own venv built with --system-site-packages.
python3 -m venv --system-site-packages .kortex_venv
.kortex_venv/bin/pip install protobuf==3.20.3
.kortex_venv/bin/pip install --no-deps   https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl

# Perception. --ignore-installed sympy, or it collides with the system copy.
python3 -m venv --system-site-packages .percep_venv
.percep_venv/bin/pip install --ignore-installed sympy matplotlib
.percep_venv/bin/pip install faster-whisper ultralytics
```

**Model weights download themselves on first use** and are gitignored:

| file | size | note |
| --- | --- | --- |
| `weights/clip/ViT-B-32.pt` | 338 MB | **over GitHub's 100 MB limit.** Pulled by ultralytics the first time YOLO-World's `set_classes()` runs |
| `yolov8s-worldv2.pt` | 25 MB | the detector itself |

### 4. Research data — committed

`recordings/` (98 MB) **is** in the repo: the 2026-07-31 teleop captures and
the 2026-08-06 trajectory capture, plus the analysis PNGs and the channel
and workspace baselines under `recordings/baselines/`. The largest single
file is 30 MB, inside GitHub's limits, so **nothing was omitted**.

## The one-paragraph version

A body-mounted master arm must be light, wearable and leave the operator's
hands free. That rules out a grounded 6-DOF measurement chain, so a wearable
master will always measure fewer degrees of freedom than a desk-mounted haptic
device — gravity gives roll and pitch but **never yaw**, and integrating
acceleration for position diverges. This project treats that as a design
constraint rather than a defect: the autonomy supplies the missing wrist
orientation during approach, and the operator keeps position control
throughout. The research question is whether that helps most when the
operator's attention is divided, which is the situation a supernumerary limb
actually creates.

---

## Layout

| package | role | depends on |
| --- | --- | --- |
| `srl_teleop` | **teleoperation only** — master sensing, IK follower, clutch, scaling, sim→real bridge, e-stop | nothing in this repo |
| `srl_perception` | AprilTag detection, 6-DOF object pose | nothing in this repo |
| `srl_autonomy` | grasp generation, intent inference, handover arbitration | `srl_teleop`, `srl_perception` |
| `srl_experiments` | E1–E5, logging, conditions, analysis | all of the above |
| `srl_description` | `srl_dual.urdf.xacro` — the arms, the mount, and the wearer | — |
| `srl_moveit_config` | MoveIt config, TRAC-IK, the SRDF | `srl_description` |

**The dependency arrow runs one way.** `srl_teleop` is the baseline condition
of every experiment, so it must run with `srl_autonomy` absent — and that is
tested, not assumed (Part 9 of `CLAUDE.md`).

---

## Quick start

```bash
# build (16 packages; two vendor gripper packages are COLCON_IGNOREd — see
# src/ros2_robotiq_gripper/README_BUILD.md)
colcon build --symlink-install
source install/setup.bash

# teleoperation only
bash scripts/run_teleop.sh

# teleoperation + perception + shared autonomy
bash scripts/run_autonomy.sh

# an experiment, with no human and no hardware
bash scripts/run_experiment.sh e1 --participant PILOT --scripted

# when something is wrong, before blaming the code
bash scripts/diagnostics.sh
```

Three-terminal split for real work:

```
terminal 1:  bash scripts/run_teleop.sh gate:=false
terminal 2:  ros2 run srl_teleop live_monitor
terminal 3:  bash scripts/start_real.sh          # --mock to rehearse
```

---

## Documentation

| | |
| --- | --- |
| `CLAUDE.md` | **the engineering log.** Every measured number, every trap, and why each decision was made. Long, and worth it. |
| `docs/system/01_architecture.md` | how the pieces fit together and which way the dependencies point |
| `docs/system/02_bringup.md` | bringing the system up, in order |
| `docs/system/03_real_robot_bringup.md` | the staged checklist for real hardware |
| `docs/system/04_calibration.md` | every calibration, in the order that matters |
| `docs/system/05_gravity_and_load.md` | robot payload, master fatigue, worn mass — three different problems |
| `docs/system/06_troubleshooting.md` | symptoms that have actually happened, and what they meant |
| `docs/research/01_literature_review.md` | 55 references, and an honest assessment of what is novel |
| `docs/research/02_baseline_and_hypotheses.md` | pre-registered hypotheses and the reviewer objection, answered |
| `docs/research/03_ethics_and_safety.md` | protocol, risks, consent, data handling |

---

## Status — what is real and what is not

**Verified in simulation:**
IK 93.8% / 91.7% over a frontal working volume; tracking under a *moving*
master 16 mm RMS (left) / 69 mm (right); the mount fix; the full experiment
pipeline end to end for all five studies.

**Verified against real hardware:** the master arm's serial path and the
Kortex high-level velocity bridge (~18 Hz, 0.01–0.15° tracking).

**NOT verified without hardware, and it matters:**
- perception's detection rate on real cameras (characterised on synthetic
  images only — the ≥95% study gate has been passed in simulation only);
- payload compensation on a real arm (the Kortex call is a stub);
- the gated real-arm flow (exercised only against `mock_real.launch.py`);
- the right arm's home joint values, which were never read from hardware.

**A known mechanical problem:** the arm bases statically interfere with the
wearer's own upper arms in the collision model. It is excluded in the SRDF and
recorded; **worn operation is not recommended until the bracket is changed.**
See `docs/research/03_ethics_and_safety.md` §2.

---

## The rules this repo is built on

1. **Measure, don't assert.** Every number in `CLAUDE.md` has a procedure
   behind it. Where a figure could not be measured, it says so.
2. **Degrade, never fail.** A dead sensor channel reduces capability and says
   which; it does not stop the node or fabricate a value.
3. **Never invent data.** A frozen command, a held pose and a cached reading
   are all indistinguishable from live ones downstream. Several bugs in this
   project's history were exactly that.
4. **The operator keeps position control.** In every autonomy state. That line
   in `handover_arbiter.py` is the safety argument of the whole design.
