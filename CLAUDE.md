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
                Session / Event-log tabs. Frame time median 2.26 ms, p95 9.14,
                max 16.66 against a 100 ms budget (n=300).
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
| unit tests | **347 pass, 2 fail, 1 skipped** | `python3 -m pytest -q src/*/test` |
| the 2 failures | `test_flake8`, `test_pep257` -- **pre-existing**, the package uses double quotes against the ROS style default | |
| executables | teleop 42, experiments 24, vr_teleop 9, autonomy 7, perception 6, vr_autonomy 2 | `ros2 pkg executables <pkg>` |
| GUI launch specs | **25** (15 task = 3 x 5 modes, plus modes and diagnostics) | `verify_gui_buttons.py` -- 46 checks |
| RViz render ceiling | **16.0 fps** llvmpipe, 16.3 d3d12, at 800x500 | `measure_render_rate.py` |
| clip delivered rate | **5.41 fps** over 11.3 s (was 1.29 over 28.4 s) | `measure_capture_rate.py` |

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

# Historical log

Everything below is the original log, kept in chronological order. The
`teleop.launch.py` invocations in it are still correct — `scripts/run_teleop.sh`
is a thin wrapper around exactly that launch.

## Hardware / serial

**The port is auto-detected — do not hardcode it.** The Teensy moves between
`/dev/ttyACM0` and `/dev/ttyACM1` on every `usbipd` re-attach, which caused
several failed runs. `srl_teleop/serial_port.py` globs `/dev/ttyACM*` and
`/dev/ttyUSB*` (preferring `/dev/teensy` if a udev rule provides it), opens
each and sniffs for a `k1j1` frame. `serial_port` defaults to `"auto"` in
`master_pose_node`, `pot_bridge` and `srl_teleop_node`; an explicit path
bypasses detection.

On failure it **raises** `PortNotFound` naming the re-attach command:

```
usbipd attach --wsl --hardware-id 16c0:0483
```

It no longer sets `self.ser = None` and spins silently, which made a wrong
port indistinguishable from a dead board.

The port is resolved **once**, in the constructor. `teleop.launch.py` gives
`master_pose_node` `respawn=True, respawn_delay=5.0`, so a Teensy attached
after launch is picked up automatically without restarting the stack.

## Channel health (measured 2026-07-31, 20440 frames)

| | live | dead / unusable |
| --- | --- | --- |
| LEFT | j1 j2 j3 j4 | j5 (2.8% zeros, 14% railed), j6 (12.8% clamped 0), j7 (75% railed) |
| RIGHT | j1 j2 j6 | j3 (70% zeros), j5 (99% zeros), j7 (99% zeros), **j4 12.9% zeros** |

All three dead right channels are roll joints — suspect one wiring fault
fatigued through the rotation, not three independent pot failures.

**Left j1 is ALIVE.** It had spread exactly 0.0 over 40 samples in a zero
capture, which is the signature that identified j7 as broken, so it was
suspected. During the shoulder-rotation segment it showed **109 distinct
values over 193.5°** (right j1: 131 values, 109.6°). The zero-spread was a
short-window artifact. No wiring fault.

**Right j4 is intermittent and is the most consequential remaining fault** —
12.9% exact-zero dropouts on a channel spherical mode *uses*, concentrated in
bursts: 51.1% of `right_A_back_to_forward`, 27.4% of `right_A_left_to_right`,
15.5% of `right_A_up_to_down`.

The validator handles it correctly — `valid==0` tracks `j4==0` to within 0.1%
(13.0% vs 12.9%), so **no fabricated data reaches the command**. The damage is
**staleness**: a rejected frame holds the last good joint vector, so the right
command *freezes* for half of one sweep and a quarter of another. That
flattens measured displacement, biases the gain toward zero and adds apparent
lag. Had the dropout been used instead of rejected it would have been far
worse — reach error 0.124 m mean / 0.189 m max (against a 0.272 m arm) and FK
azimuth error 55.9° mean / 180° max. **Check the j4 connector.**

## Position: SPHERICAL mode (the default)

`position_mode` is `spherical`. The old 7-joint FK path (`fk`) is kept intact
and switchable live for comparison.

The full FK collapsed directionally: SVD of five gesture positions put
93.4/4.2/2.3% of variance on one axis and put "down" *higher* than "up". So
position is now built in spherical form about the master shoulder:

- **ELEVATION** from the wrist IMU. `a_hat` (arm long axis in the sensor
  frame) is captured once with the arm hanging down, and
  `elevation = asin(dot(a_hat, normalize(accel)))`. Mount-independent,
  drift-free, no integration.
- **AZIMUTH** from j1 alone, zero-referenced.
- **REACH** from `|fk([j1,j2,j3,j4,0,0,0])|` — magnitude only, which is
  dominated by the j2/j4 bends. A dead j3 is passed as 0.

Validation is mode-scoped: spherical only rejects on **j1, j2, j4**. A dead
j3/j5/j6/j7 no longer rejects a frame — that is the entire point.

### a_hat sign convention — do not "fix" it

Hanging down = **−90°**, horizontal = 0°, straight up = **+90°**.
`capture_zero.py` asserts this at capture time. The original brief specified
both `a_hat = -normalize(accel_rest)` *and* `elevation = asin(-dot(...))`;
those compound and put the hanging rest pose at +90°, flipping z alone.
Only one negation is correct.

### NEUTRAL vs NEUTRAL_SPHERICAL — keep them separate

`NEUTRAL = to_robot_frame((0,0,+FULL_EXT))` is the FK path's straight-out
neutral. `NEUTRAL_SPHERICAL = to_robot_frame((0,0,-FULL_EXT))` is the
spherical rest, arm hanging **down**. Using the FK neutral in spherical mode
offsets every commanded pose by 0.544 m in z — twice the arm's whole reach.
Both are asserted to give exactly zero rest displacement.

## AXIS_MAP — `("+x","+y","+z")`, measured

**det(_axis_matrix()) == +1.0**, asserted at import by `validate_axis_map()`.

Evidence, from regressing commanded pose against the master's physical
spherical position over all Phase-A sweeps (left, 6443 samples, R² = 0.999):

```
            phys_x    phys_y    phys_z
  cmd_x     +0.992    -0.004    +0.008
  cmd_y     +0.002    -0.993    -0.010
  cmd_z     +0.005    -0.003    -1.011
```

That is exactly `diag(+1,−1,−1)`, i.e. the old `("+x","-y","-z")` — so the
pipeline was faithfully doing what AXIS_MAP said, and AXIS_MAP was wrong.
Raising the master drove the robot **down** (`cmd_z = −1.011·phys_z`) and
moving forward drove it **back**.

**The four maps ending in `"-z"` are invalid in spherical mode.** They were
derived for the FK path, where +z runs along the hanging arm. In spherical
mode the elevation convention already carries the vertical sign, so a `"-z"`
entry double-negates it and *no* such map can produce correct up/down.
`LEGAL_AXIS_MAPS` now lists the four det=+1 maps with a positive vertical.

Identity fixes vertical and fore/aft. Verified: raising the master 30° now
gives `robot dz = +0.036` (was negative).

### AZIMUTH LIMITATION — lateral is still wrong, and AXIS_MAP cannot fix it

**Mechanism (corrected).** j1's axis does *not* tilt as the arm raises — j1 is
the first joint and its axis is fixed in the mount. The coupling is the
**lever arm**: j1 is a roll about the arm's own axis, so with a straight arm
(q=0) rolling j1 moves the tip 0.000 m. j1 only produces lateral motion once
a downstream bend (j2/j4) offsets the tip from the roll axis. So
azimuth-per-degree-of-j1 scales with how bent the arm is, and azimuth and
elevation are **not** independent — which is what the spherical model assumes.

**Hybrid azimuth was tested offline and REJECTED.** Replacing
`azimuth = j1` with `azimuth = atan2(p_fk.y, p_fk.x)` (FK's horizontal angle,
which accounts for the bend) made things *worse* on the cleanest available
metric — the reversal test, which assumes only that the operator reversed
each motion, not that they hit the axes:

| arm | model | down↔up | left↔right | fwd↔back | mean err from 180° |
| --- | --- | --- | --- | --- | --- |
| left | j1 (shipped) | 166.8 | 159.7 | 175.7 | **12.6°** |
| left | hybrid | 157.1 | 108.1 | 175.1 | 33.3° |
| right | j1 (shipped) | 163.2 | 137.8 | 160.3 | **26.2°** |
| right | hybrid | 146.3 | 134.6 | 140.5 | 39.5° |

**How much of the right arm's extra error is j4?** Re-regressing with the
stale (`valid==0`) frames dropped moves the right arm's reversal error only
23.3° → 21.3° and Kabsch rms 54.7° → 51.4°, against 12.3° for the healthy
left arm. So j4 accounts for roughly **2° of the ~9–11° gap** — it is a real
fault worth fixing (it costs smoothness, freezing the command for 13% of
frames) but it is **not** why the right arm's directions are wrong. The right
arm is as usable as the left for up/down and fore/aft; both fail laterally
for the same azimuth reason.

Likely reason: FK's horizontal angle inherits the absolute-joint-angle errors
that the IMU mount calibration already exposed (calibrated q3 spanning 336°,
q6 ±130–170° — physically impossible). j1 alone is a single directly-measured
zero-referenced quantity; FK azimuth compounds four suspect angles. **Do not
re-try the hybrid without first fixing the pot angle calibration.**

Fitting the best rotation from measured sweep directions to the operator's
intended directions (Kabsch) leaves **54.8° RMS residual (left), 54.7°
(right)**. The rotation-invariant pairwise-angle test shows why: intended
left↔right and forward↔back are 90° apart, but measured **137.7°** apart
(off by +47.7°). A rotation preserves angles, so **no rotation and no
permutation can map one triad onto the other** — the measured directions are
not an orthogonal triad at all.

Cause: azimuth comes from **j1 alone**. j1 is the shoulder roll. With the arm
hanging, rotating j1 rotates the arm about the vertical, so j1 *is* azimuth.
As the arm is raised, j1's axis tilts away from vertical and stops
corresponding to azimuth — and every sweep in the protocol raises the arm.

**Up/down and forward/back are correct. Lateral is not.** Fixing it needs a
better azimuth observable (e.g. j1 combined with j2/j4 to resolve the
shoulder cone, or a second IMU), not a different AXIS_MAP.

## Gyro-aided azimuth — built, measured, NOT enabled by default

`azimuth_mode` = `j1` (default) or `gyro`. The gyro path is implemented and
the plumbing is live (`gx,gy,gz` are now published on `/master_arm_raw_*` and
recorded), but it is **off** until the orthogonality gain is demonstrated on a
capture with a *moving* master.

Design: `yaw_rate = dot(omega - bias, u_hat)` where `u_hat = normalize(accel)`
is world-up in the sensor frame. This needs **no mount rotation** — both
vectors are already in the sensor frame — so the failed IMU mount calibration
is irrelevant. It observes rotation about vertical directly, so it is immune
both to the lever-arm coupling that defeats azimuth-from-j1 and to the chain
error that defeated FK azimuth. Bounded three ways: reset to the j1 estimate
at **every clutch engage**, a slow pull toward j1
(`azimuth_blend_tau_s` = 20 s), and per-arm bias subtraction. When `|accel|`
leaves the gate the last trusted `u_hat` is **held** and integration
continues — that fast motion is exactly what the gyro is for.

### Measured drift, arms stationary, 60 s

| arm | drift | residual yaw rate | verdict |
| --- | --- | --- | --- |
| left | **−5.0 °/min** | mean −0.075 °/s, std 1.57 | usable; a 20 s segment drifts ~1.7° |
| right | **+44.0 °/min** | mean +0.574 °/s, std 9.74, max 72.3 | **unusable** |

The right IMU shows large intermittent spikes (72 °/s while stationary) —
same signature as the rest of the right-arm harness fault. Left is clean.

### Gyro bias capture — a trap worth knowing

Bias must be captured from a **settled** stream. Capturing it through
`capture_gyro_bias.py` while something else is repeatedly reopening the serial
port gives garbage, because **reopening the port resets the Teensy** and the
IMU emits nonsense while settling. The first capture that way read
std 4.85 °/s (left) / 27.9 °/s (right) and a right bias of 23.4 °/s; sampled
from the settled ROS topic instead, the same gyros read std ~0.11 °/s. That
one mistake was the difference between 1492 °/min and 44 °/min of drift.
Stop the publisher cleanly, or sample `/master_arm_raw_*` rather than the port.

## Orientation: `orientation_mode` — default `fixed`

- `fixed` (default) — orientation pinned to the anchor. **Position-only teleop.**
- `anchored` — `anchor_quat * relative rotation from the master wrist`.
- `relative` — the original: the master's relative rotation raw.

The master wrist cannot currently be measured (left j7 railed + j6 clamped;
right j3/j5/j7 dead), so commanded orientation swung with roll/pitch while
yaw stayed frozen, making position and orientation jointly unreachable.
Measured over the recorded trajectory with `/compute_ik`:

| arm | scale | orientation from wrist | held at anchor |
| --- | --- | --- | --- |
| left | 0.6 | 10% | **100%** |
| left | 1.0 | 8% | **92%** |
| right | 0.6 | 62% | **98%** |
| right | 1.0 | 75% | **95%** |

Earlier, `relative` → `anchored` alone took IK from 5.3% → 92.6% (left) and
0.8% → 62.6% (right). Move to `anchored` once the wrist channels are
repaired.

## Workspace anchor — MEASURED

```
WORKSPACE_CENTRE = {"left": (0.363, 0.407, 1.017), "right": (-0.220, 0.657, 1.254)}
WORKSPACE_ORIENT = {"left": (0.899, 0.118, -0.375, -0.191),
                    "right": (-0.068, 0.617, 0.572, 0.537)}
```

From `tf2_echo world <arm>_end_effector_link` with both arms verified at home
to 4 dp. The old guess `(0.3, 0.0, 1.2)` returned **NO_IK_SOLUTION (−31)** for
the left arm — it was genuinely unreachable, and is what drove the IK
failures. Backpack mounts verified unchanged at (±0.15, −0.12, 1.25).

## Motion scaling

`left_scale` / `right_scale` default to **1.0**, chosen from measured IK
reachability (92%/95% at 1.0 with orientation fixed; ~0.44 m commanded span
against a master range of 0.39–0.47 m per axis).

Scale is **re-read every frame**, so `ros2 param set` retunes live. On a
change the anchor is **re-based** so the commanded pose is continuous:

```
pos_anchor += (old_scale - new_scale) * (current - reference_at_engage)
```

Without that, changing scale jumps the arm by `(new−old)×displacement`,
worst exactly when the operator is far from the engage point. Verified live:
scale 1.0→2.0→0.5→1.5→1.0 moved the EE ≤1.8 mm.

## IK follower

Joint wrapping **is implemented** (this used to say otherwise):
`CONTINUOUS_IDX = (0,2,4,6)` for joints 1/3/5/7, `wrap_continuous()` on both
the IK seed and the published solution, `time_from_start` from the true
unwrapped delta over `max_vel_rad_s` 0.6 floored at `min_time_s` 0.05, plus a
startup unwind. `right_joint_5` is −175° in both `home_positions_right.txt`
and the URDF (−3.0543 rad), clear of the ±π seam.

### The guard deadlock, and its fix

Symptom: `reject_count` exactly equalled `success_count`, with 100% IK
success and the arm never leaving home. The step guard discarded every
solution further than `max_step_rad` from the current state — but a discarded
solution means the arm does not move, so the next solution is exactly as far.
It could never converge.

Fix: **slew, don't discard.** `clamp_towards()` commits a bounded step toward
the solution, converging in `ceil(distance / max_step)` cycles. A solution is
only *rejected* when the target pose barely moved (pose deadband) yet the
solution jumped — a redundancy flip — and even then `flip_reject_limit`
force-accepts after 10 consecutive rejections so rejection can never deadlock
either. `[GUARD]` reports `direct / slewed / rejected` separately; conflating
them made a working solver look like a failing one.

Regression tests: `src/srl_teleop/test/test_ik_guard.py` (6 tests, includes
convergence from 2.0 rad within `ceil(2.0/0.35)` cycles).

### In-flight watchdog

`pending` guards against overlapping IK calls and is cleared in the response
callback. If a call never returns — `/compute_ik` restarted, `move_group`
disturbed — it stayed `True` forever and every later pose was skipped in
silence. Observed exactly that: node alive, poses at 50 Hz, zero IK calls.
`pending_timeout_s` (1.0) now force-releases it.

## Clutch

Starts **ENGAGED** and only flips on a button *change*, so it cannot latch
disengaged at startup — logged explicitly at startup. `force_clutch_engaged`
pins it engaged and ignores the buttons entirely.

**Button mapping, MEASURED 2026-07-31: left button → `btn2`, right button →
`btn1`.** This confirms the suspected reversal (`BUTTON1_PIN=2` is the left
pin). The recorder re-measures it at the start of every run and writes it to
a `<csv>.meta.json` sidecar rather than assuming it.

Re-engage jump measured over 20 transitions: **≤9.2 mm, most under 3 mm.**

## Recording a calibration pass

```
terminal 1:  ros2 launch srl_teleop teleop.launch.py
terminal 2:  bash ~/kortex_ws/run_teleop_capture.sh
```

`teleop_recorder` is **button-gated**: it prints an instruction and waits
indefinitely, you press to start, move at your own pace, press to end. Rows
land only between the presses, so repositioning is never regressed as signal.
A press-and-release under 0.5 s discards the segment and re-prompts; the rows
are buffered and only committed on save, so a discarded take leaves no trace.

Every segment is gated by the **opposite** arm's button — an arm's own button
also toggles that arm's clutch, and gating a left sweep with the left button
disengages the clutch exactly when the sweep starts.

Two things that are easy to get wrong here:

- **Do not put the recorder inside the launch.** `ros2 launch` merges all
  stdout and line-prefixes it, so RViz/MoveIt/followers bury the countdown
  and prefixes break in-place redraw. Suppressing the recorder's own output
  cannot help — the noise is from other processes. Hence no `record:=true`.
- **The recorder spins on a background executor.** Calling `spin_once()` once
  per row is thin for eight subscriptions plus `/tf` and `/tf_static`. Fixed
  as a precaution — but note this was **not** why the 13:33 recording had
  empty `*_ee_*` columns (see below).

### Why a recording can silently lose robot EE

`teleop_20260731_133310.csv` has **0/20440** EE rows; the two earlier
recordings that day have 8529/8531 and 4556/4561 with the *same* recorder
code. So the cause was not the recorder: **`/tf` was dead**, because
`joint_state_broadcaster` and `left_arm_controller` had gone `unconfigured`.
That in turn was almost certainly caused by an over-broad process cleanup
(`pkill -f "ros2 launch"` plus a grep-based `kill -9` sweep including
`spawner`) run ~4 minutes earlier while two stacks were up.

**Before any capture, check:** `ros2 control list_controllers` shows
`joint_state_broadcaster` and both arm controllers **active**, and
`ros2 topic hz /joint_states` returns ~100 Hz. Recover with:

```
ros2 control set_controller_state <name> inactive   # configures
ros2 control set_controller_state <name> active
```

Never clean up processes with broad name patterns while a stack you want to
keep is running — kill an explicit PID list instead.

Shell scripts sourcing ROS must wrap it in `set +u` / `set -u` —
`setup.bash` reads unbound variables internally and dies under `set -u`.

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

## Grippers

`ros2 run srl_teleop fsr_gripper_node`. fsr1 → left, fsr2 → right, mapped
proportionally onto the Robotiq 2F-85 driven knuckle (0 rad open, 0.8 closed).

Measured pads: unsqueezed 6–191 counts, squeezed 3603 (left) / 3842 (right),
cross-talk 3.7% / 0.4% under load only — cleanly separable, so no mixing
matrix is needed. Three corrections are: a **deadband at 250 counts** (rest
reaches 191, so without it the gripper creeps shut on noise), **per-arm range**
(the pads differ ~7% at full squeeze), and **output hysteresis** of 0.02
(a new setpoint every frame at 50 Hz makes the fingers buzz).

**Left gripper works** — commanded 0.40 / 0.80 / 0.00 rad, tracked exactly.

**Right gripper is BLOCKED, and not by the controller.**
`right_robotiq_85_left_knuckle_joint/position` shows **`[unavailable]`** in
`ros2 control list_hardware_interfaces`, while the left equivalent is
`[available] [claimed]`. The right `RobotiqGripperHardwareInterface` component
is `active` but never exports its command interface, so
`right_gripper_controller` cannot activate — `set_controller_state` and a
fresh `spawner` both fail with "Failed to activate controller". This is a
URDF / ros2_control config fault in how the right gripper is instantiated, not
something the controller layer can fix.

## Analysis

`ros2 run srl_teleop analyse_teleop <csv>` — gain matrix, effective scale,
lag, tracking error, clutch jump, dropouts, and four plots.

## Status dashboard

`ros2 run srl_teleop dashboard` (own terminal, full-screen) or
`teleop.launch.py dashboard:=true` (plain blocks in the log). Plain ANSI, no
toolkit, works over SSH. Pot colouring is **mode-aware**: exact-0.0 is red
only on channels the current `position_mode` consumes, so a dead-but-unused
j5 shows yellow rather than training you to ignore red.

## Tuned parameters

| parameter | value | why |
| --- | --- | --- |
| `ema_alpha` | 0.3 | unchanged; tracking error is already ~0 |
| `accel_gate_g` | 0.15 | unchanged; 0.0% of frames exceeded it in 20440 rows |
| `max_step_rad` | 0.35 | unchanged; 0 rejections and 0 slews at 50 Hz |
| `left_scale` / `right_scale` | 1.0 | measured reachability |
| `pending_timeout_s` | 1.0 | new watchdog |

Nothing needed loosening: the accel gate was never hit, and the guard is
publishing every solution directly.

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

## WSL networking — MIRRORED, and the arms are reachable

- `eth0 192.168.1.20/24` → **mirrored mode** is active
  (`networkingMode=mirrored` in the Windows-side `%USERPROFILE%\.wslconfig`).
  Both arms answer: 192.168.1.10 (left) at ~1.35–1.7 ms RTT, 0% loss.
- Mirrored mode shares **all** Windows adapters, so WSL now sees `eth0`,
  `eth3` and loopback rather than the single NAT interface. That is normal.
  It does **not** partition ROS 2 discovery — see the ros2 daemon note below.
- `scripts/check_arm_network.sh`: 500-ping latency/jitter/loss, TCP 10000 and
  443, UDP 10001 probe, PASS/MARGINAL/FAIL verdict.

### "No ROS 2 nodes at all" is usually a STALE ros2 DAEMON

After the switch to mirrored mode, terminal 3 reported *"No ROS 2 nodes at
all"* while terminal 1 ran a healthy stack. There was **no discovery
partition**: `ros2 node list --no-daemon` saw all 28 nodes the whole time.
The **ros2 daemon** caches network state, did not survive the interface
change, and *hung* rather than failing — so `timeout 20 ros2 node list`
returned an empty string, which read as "nothing is running".

Fix and prevention:

```
ros2 daemon stop && ros2 daemon start
ros2 node list --no-daemon      # ground truth, bypasses the daemon
```

With a healthy daemon the default settings give 28/28 nodes on 5/5 runs
across eth0 + eth3 + lo. **Do not pin discovery to loopback** to "fix" this —
`ROS_LOCALHOST_ONLY` is deprecated in Jazzy and, applied to only some
processes, causes the very partition it is meant to prevent (the daemon caches
the env of whichever shell spawned it first). `scripts/env.sh` is the single
source of environment truth: it pins `ROS_DOMAIN_ID=0`, actively *unsets* any
inherited `ROS_LOCALHOST_ONLY`, and provides daemon-aware graph helpers.
Every script sources it, and `teleop.launch.py` applies the same settings
in-process because terminal 1 is launched by hand and never sources it.

`start_real.sh`'s preflight distinguishes the three states rather than
conflating them: nothing running / running but not discoverable (names the
daemon) / healthy.

## THE CYCLIC PATH IS UNUSABLE OVER WSL — use the high-level API

**This is the single most important operational fact about running the real
arm from this machine.** Do not spend another day on the ros2_control driver.

The Kortex **cyclic** path (what `ros2_control`'s
`KortexMultiInterfaceHardware` commands through) is built for a **1 kHz** loop
on a dedicated link. Over WSL every cyclic write is a **network round trip
costing ~10 ms**. Measured, in a read-only launch with no controller even
capable of commanding:

```
Read time: 217 us, Update: 282 us, Write time: 10365 us
```

At 100 Hz the whole cycle budget is 10 ms, so the write alone consumes it.
The controller manager overran permanently, **no command ever reached the
arm**, and the arm sat still while serving perfectly live feedback. Lowering
the CM rate does not help: the write is a round trip at any rate.

**How this misleads you.** The symptom is a frozen-looking arm:
`/real/joint_states` flat to 16 decimal places, homing errors identical for
15 s, then `HOMING STALLED`. Everything points at a dead driver or a faulted
arm. Both are wrong. Checks that ruled them out, worth repeating before
believing any new theory:

- `left_arm_controller` was **active**, with `Subscription count: 1`. Not a
  spawner-lock problem.
- No leaked session — a fresh single driver connected cleanly
  (`Session created`, no lock retry), and read a *different* angle than the
  previous session, so the feedback was genuine and not a cache.
- Arm not faulted: `ARMSTATE_SERVOING_READY`, all 7 actuators
  `fault=0x0/0x0`, 24.3 V, `SINGLE_LEVEL_SERVOING`.
- The feedback was flat **because nothing was commanding it**, not the
  reverse.

**The supported route here is the HIGH-LEVEL API**:
`SendJointSpeedsCommand` over the TCP session. Same ~10 ms round trip, but it
is a *velocity* command — it holds a motion between sends, so it does not need
1 kHz. `srl_teleop/kortex_highlevel_bridge.py` implements it and
`real_arms_highlevel.launch.py` is the launch that uses it. `start_real.sh`
points at that launch. Measured end to end, homing the real arm to within
tolerance on all 7 joints:

| Metric | Value |
|---|---|
| Loop rate | **18.4–18.7 Hz** (target 30) |
| Send latency | min ~17 ms, **avg ~26 ms**, max 33 ms |
| Tracking error | **0.01–0.15 deg** (setpoint vs actual) |
| Homing result | all 7 joints ≤ 0.98 deg, `WITHIN` tolerance |

30 Hz is not achievable: a cycle is **two** sequential round trips (read, then
write) and back-to-back RPCs cost ~26 ms each rather than the ~10 ms an
isolated call costs. ~18 Hz is still far more than the motion needs — at
vmax 0.05 rad/s a joint moves 2.8 mrad per cycle against a 1 deg (17 mrad)
deadband. **Always read the logged achieved rate; never assume `rate_hz`.**

### Two traps in that node, both already paid for

1. **Do not apply the velocity law twice.** `real_homing_node` and
   `sim_to_real_bridge` both publish an *incremental* setpoint (`cur + v*dt`,
   ~1.7 mrad ahead of measured). A second position law on top sees an error an
   order of magnitude *inside* the 1 deg deadband and commands exactly zero —
   the arm never moves, the setpoint never advances because it tracks measured
   position, and homing stalls at its starting error. The bridge therefore uses
   **feedforward + feedback**: `speed = clip(v_ff + kp*err, ±vmax)`, taking
   `v_ff` from the trajectory's `velocities[]` when present (homing fills it)
   and differentiating the setpoint stream otherwise. The deadband suppresses
   the **correction only**.
2. **Never use `time.time()` for intervals in WSL.** The wall clock steps
   backwards on host resync; it produced a send latency of **−2321 ms**. Use
   `time.monotonic()`. This same stepping is what makes
   `real.robot_state_publisher` log *"Moved backwards in time, re-publishing
   joint transforms"* every ~30 s — a clock artefact, **not** a driver fault.

### kortex_api must stay isolated

The Kinova wheel pins `protobuf==3.5.1`, which is **broken on Python 3.12**
(`collections.MutableMapping`) and, installed to user site, **shadows the
protobuf ROS needs**. It lives in `~/kortex_ws/.kortex_venv`, built with
`--system-site-packages` so one interpreter has both `rclpy` and `kortex_api`:

```
python3 -m venv --system-site-packages ~/kortex_ws/.kortex_venv
~/kortex_ws/.kortex_venv/bin/pip install protobuf==3.20.3
~/kortex_ws/.kortex_venv/bin/pip install --no-deps \
  https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl
```

`real_arms_highlevel.launch.py` runs the bridge through that interpreter via
`ExecuteProcess`. Do **not** `pip install kortex_api` into user site.

### One session, and closing it

The arm permits exactly **one** session and a leaked one blocks the next run.
The bridge shares a single session between read and write and closes it on
every exit path (`zero speeds → Stop() → CloseSession`), logging
`kortex session closed cleanly`. Grep for that line after any run. SIGINT the
bridge process directly if you have to kill things by hand — SIGKILL leaks
the session.

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

## /mnt/c is broken and CANNOT be fixed from inside WSL

`/proc/mounts` still shows the mount — `C:\134 /mnt/c 9p rw,aname=drvfs;path=C:\;...trans=fd,rfd=6,wfd=6` — but every access returns **EIO**. The
9p transport behind those file descriptors is dead; the mount entry is a
corpse. `sudo` on this machine **requires a password**, so the remount cannot
even be attempted unattended, and a remount would not help anyway: the 9p
channel is established by the WSL host process, not by the guest.

**Fix (must be done by a human, from Windows):**
```
wsl --shutdown          # in Windows PowerShell
```
then reopen the WSL terminal.

**Everything downstream stays PENDING until then:** Windows host LAN address,
`wsl --version`, the Windows build, whether this machine qualifies for
mirrored networking, and any `usbipd` check. Note the Teensy still works —
`usbipd` was attached earlier in the session, and `/run/usbipd-win` is a
*separate* 9p mount that is still alive.

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

## Three-terminal split (2026-07-31)

    terminal 1:  ros2 launch srl_teleop teleop.launch.py gate:=false
    terminal 2:  ros2 run srl_teleop live_monitor
    terminal 3:  bash scripts/start_real.sh          (--mock to rehearse)

`gate:=false` skips the y/N prompt; sim teleop is live immediately and the
real driver stack is not running at all. `monitor:=false` is now the default
in teleop.launch.py, because live_monitor is a full-screen redraw whose
cursor-home escapes corrupt a shared launch log, and running it in both
places would duplicate the node name.

`live_monitor` was rewritten: it previously coloured dropouts against j1-j5
(the **FK-mode** channels) while the default mode is spherical, which uses
only j1/j2/j4 -- so it reddened j3/j5 for being idle. Now mode-aware, matching
dashboard.py, and it shows gyro as well as accel.

`scripts/start_real.sh` refuses with a named reason and exit 1 when: the sim
stack is not up, the e-stop is latched, homing finishes outside tolerance, or
the bridge will not enable. `--mock` runs the identical sequence against
mock_real.launch.py, which is how it is tested without hardware.

**Operational fact worth knowing:** after the real arm homes, the bridge will
not enable until the SIM also sits near home, i.e. until the operator is
holding the master at neutral. With the master resting on the bench the sim
sits ~0.19 rad off and the bridge refuses (correctly). This is not a fault.

**Two script bugs found by running it, not by reading it:** `grep -c` prints
`0` AND exits 1 when there is no match, so a `|| echo 0` fallback produced
`"0\n0"` and broke an integer test; and `ros2 topic hz` needs several samples
before printing, so a short timeout reported a perfectly healthy master as
"silent". Both fixed.

## /real controller_manager rate, and the REAL blocker (2026-07-31)

`srl_teleop/config/real_controllers.yaml` now configures the /real CM
separately from the sim (keyed `/real/controller_manager`, 20 Hz). The sim's
`srl_moveit_config/config/ros2_controllers.yaml` is UNCHANGED at 100 Hz.
Spawner timeout raised to 120 s; `real_arms.launch.py` gained `home:=false`
and `bridge:=false` so the driver can be brought up without motion.

**The rate was a red herring.** Measured at both 30 Hz and 20 Hz:

    Read time  :      50 - 160 us       <- fast
    Write time : 3000000 - 6008589 us   <- 3 to 6 SECONDS, then
                 "timeout detected: BaseCyclicClient::RefreshFeedback"
    22 write timeouts, 47 overruns, achieved joint_states 4.0 Hz,
    feedback FROZEN (1 distinct value/joint, std 0.000e+00)

No update_rate absorbs a 3-second blocking call. From the driver banner:

    Port used          '10000'   TCP config  -> session created, 7 actuators
    Realtime port used '10001'   UDP cyclic  -> times out

TCP 10000 opens fine and ping is 1.5 ms. It is the **UDP realtime channel on
10001** that WSL2 NAT breaks. `/etc/wsl.conf` has no `networkingMode`, so this
box is on NAT.

**MIRRORED NETWORKING IS MANDATORY.** In Windows `%UserProfile%\.wslconfig`:

    [wsl2]
    networkingMode=mirrored

then `wsl --shutdown`. Until then the real arm CANNOT be commanded, and
`ros2 control list_controllers` showing "active" is misleading -- the
controllers activate and then the hardware deactivates underneath them.

**Trap:** the "fast" 50-160 us read is NOT an improvement over the original
11479 us. Once a write error deactivates the component, read() returns a
cache. A fast read here is the dead-channel signature, the same zero-variance
pattern as the frozen `/real/joint_states` and the dead j7 pot.

---

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

# Restructure and research build (2026-08-05)

## Part 2 — six packages, one-way dependencies

Moved out of `srl_teleop`: `vlm_locate_node` → `srl_perception/vlm_object_locator`,
`pick_place_node` → `srl_autonomy/scripted_pick_place`, `move_to_pose_node` →
`srl_autonomy/move_to_pose`, `spawn_cubes` → `srl_experiments/scene_spawner`.
Deleted `master_pose_node.py.bak`. Root clutter (`read_teensy.py`,
`srl_gui_wsl.py`) moved to `scripts/legacy/`.

`colcon build` with no flags now builds 16 packages clean. Two vendor gripper
packages are `COLCON_IGNORE`d for a missing `serial` dependency — pre-existing
and already noted in this file; `robotiq_description`, which is the part the
URDF needs, still builds.

**One real defect found by auditing entry points against modules:**
`capture_zero` had no `main()` — its whole body was under
`if __name__ == "__main__"`, so `ros2 run srl_teleop capture_zero` had never
worked. Wrapped in `main(argv=None)`.

## Part 3 — IMU-primary sensing

### (a) The pot reduction ladder — MEASURED on 20 430 real frames

Commanded master-tip error against the shipped 4-pot reach, over the
2026-07-31 recording. **This is the cost to the SHIPPED pipeline**: in
spherical mode direction comes from the IMU, so a pot reduction degrades the
REACH MAGNITUDE, not the direction. Measuring raw 7-DOF FK error instead would
charge every reduction for direction error the pipeline never incurs.

| rung | pots | mean | p95 | max | LEFT |
| --- | --- | --- | --- | --- | --- |
| R0 j1 j2 j3 j4 (shipped) | 4 | 0.0000 | 0.0000 | 0.0000 | reference |
| R1 drop j3 | 3 | **0.0416** | 0.0800 | 0.0842 | |
| R2 drop j3, j4 | 2 | 0.0661 | 0.1672 | 0.2399 | elbow lost |
| R3 drop j2, j3 | 2 | **0.0292** | 0.0653 | 0.0859 | shoulder lost |
| R4 j1 + 2nd IMU elbow | 1 | 0.0638 | 0.1517 | 0.2358 | |
| R5 no pots | 0 | — | — | — | **POSITION UNAVAILABLE** |

RIGHT arm: R1 0.0045, R2 0.0695, R3 0.0267, R4 0.0648 m.

**Reading it.** Dropping **j3 costs 4 cm on the left and 4.5 mm on the right** —
it is nearly free, which is why a dead j3 is already passed through as 0.
Dropping **j2 (R3) costs LESS than dropping j3+j4 (R2)** on both arms, because
the elbow dominates reach magnitude and the shoulder does not. A second IMU
supplying the elbow (R4) gets to **6.4 cm on one pot**, comparable to keeping
two. And with **no pots there is no position at all** — the node says so rather
than inventing one, which is the single most dangerous failure mode on a robot
bolted to a person.

### (c) Complementary filter — MEASURED

Tilt error against a static gravity reference. **The gyro stream is
SYNTHESISED** from the frame-to-frame change in accel-derived attitude plus
each arm's **measured** bias and noise (left −0.075 ± 1.57 °/s, right
+0.574 ± 9.74 °/s) — the recordings predate the gyro plumbing. The noise is
real and the motion is real; only the pairing is synthetic.

| tau (s) | LEFT mean / p95 | RIGHT mean / p95 |
| --- | --- | --- |
| **0.2** | **0.131° / 0.279°** | **0.574° / 1.131°** |
| 0.5 | 0.191° / 0.380° | 0.934° / 1.875° |
| 1.0 | 0.245° / 0.584° | 1.415° / 2.912° |
| 2.0 | 0.355° / 0.764° | 2.285° / 4.515° |
| 5.0 | 0.707° / 1.539° | 5.044° / 7.603° |

The right arm is **~4× worse at every tau**, tracking its 6× noisier gyro.

**The accelerometer's own noise CANNOT be measured from these recordings**:
89.9% of consecutive rows repeat the accel triple **bit-identically**, because
the 50 Hz recorder oversamples the sensor. Any within-run figure is 0.000° by
construction. An earlier version of the analysis reported 37.8° of "noise" by
pooling all static frames against one global mean — which measured how many
different poses the arm rested in, not the sensor.

### Two bugs in my own analysis, both caught by the numbers looking wrong

1. **The filter's correction sign.** `cross(v_pred, v_meas)` the wrong way
   round does not diverge loudly — it settles at a stable **176–180°** error,
   which reads like a coordinate convention problem. Correct is
   `cross(v_meas, v_pred)`: the correction is applied as `q ← q·dq`, so the
   rotation must carry `v_meas` onto `v_pred`. `test_filter_converges_and_does_not_invert`
   fails hard on the wrong sign.
2. **Euler wrapping in the metric.** Differencing roll/pitch reported ~200° of
   error purely from `atan2` wrapping near ±π with the arm hanging. The metric
   is now the angle between predicted and measured gravity — representation-
   free, and immune to yaw, which gravity cannot observe anyway.

### (b) Two-IMU path — built, DEFAULT OFF

`master_imu_node two_imu:=true`. Elbow angle from the two segments' relative
orientation, no pot. The shared yaw ambiguity **cancels** in a relative
measurement — that is why two IMUs can replace the elbow pot and one cannot,
and it is asserted in `test_two_imu_elbow_is_yaw_invariant`.

**WIRING — 0x68 and 0x69 are both taken.** The MPU-6050/9250 family has
exactly two I²C addresses, selected by AD0, and this rig already uses both
(one IMU per arm). A third sensor cannot join the same bus. Options:

1. **Second I²C bus.** The Teensy 4.1 has three (`Wire`, `Wire1` on pins
   16/17, `Wire2` on 24/25). Put both upper-arm IMUs on `Wire1` at 0x68/0x69.
   No extra parts, short buses. **Recommended.**
2. **TCA9548A multiplexer** on the existing bus: one part at 0x70–0x77, eight
   channels, each carrying a 0x68/0x69 pair. One extra transaction per read,
   irrelevant at 50 Hz.
3. A sensor with more addresses (BNO055 at 0x28/0x29), which also moves fusion
   onto the sensor.

The frame must gain the second sensor's six fields at indices 13–18 of
`/master_arm_raw_<arm>`. With `two_imu:=true` and a short frame the node says
so and falls back to the j4 pot, rather than silently using the first sensor
twice.

### (e) Every channel individually disableable — `channel_manager`

```
ros2 param set /channel_manager left_j4_enabled false
ros2 topic echo /master_capability_left
```

Publishes what is live and **what each loss costs**, per channel. Policy is
**degrade, never fail**. `disable_all_pots` is the endpoint of the argument:
the rig becomes IMU-only and **position is reported UNAVAILABLE**, not
invented. `auto_disable_after_frames` retires a channel that has been reading
a firmware-clamped 0.0 for N consecutive frames.

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

# VR TELEOPERATION (2026-08-05)

Two new packages, `srl_vr_teleop` and `srl_vr_autonomy`, plus `quest_app/`.
Verified: **no import of `srl_teleop` in either direction**, and `srl_teleop`
still lists 30 executables and runs with all five other srl packages removed
from `install/`.

**Transport: WebXR over a WebSocket.** The dominant latency term is the
headset's own frame period (13.9 ms at 72 Hz), identical for Unity; the
WebSocket adds single digits on a LAN. Unity would buy a few ms in exchange
for a licence, adb sideloading and a rebuild per iteration.

Measured against the desktop mock: **controller 72.0 Hz** (requirement ≥60),
**robot command 100 Hz**, ROS-side lag below the 5 ms measurement resolution.
Clutch engage anchors exactly on the robot's current EE, so the re-engage jump
is **zero by construction**.

**Assistance differs from the mannequin, and that is the interesting part.**
The mannequin's autonomy supplies wrist orientation the input cannot express —
capability recovered, a categorical result. VR already has 6-DOF, so its
autonomy supplies PRECISION: a funnel that blends toward the validated grasp
over the last 3 cm, and **yields entirely if the operator starts rotating the
controller**.

Two bugs the mock caught that reading would not: the clutch consumed its
rising edge on a refused engage and could never engage again; and the mock
moved from frame zero, so the quasi-static engage check correctly refused it
forever.

---

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

# QUEST TRANSPORT — the vendor servers, and the decision (2026-08-07)

Two RAR5 archives from `/mnt/c/Users/Gausms/Downloads`. No `unrar` and `sudo`
needs a password, but **`libarchive.so.13` is already present**, so
`libarchive-c` in a throwaway venv extracts RAR5 with no install and no root.

## What they are

| | hand pose | controller signal |
| --- | --- | --- |
| port | `ws://127.0.0.1:8765` | `ws://127.0.0.1:8766` |
| protocol tag | `wen.quest.handpose.v1` (validated) | none; requires `left`+`right` |
| transport | **`adb reverse tcp:8765 tcp:8765` over USB** | same, 8766 |
| payload | `left/right`: `tracked`, `confidence`, `position{xyz}`, `rotation{xyzw}`, plus `space`, `sequence` | `left/right`: `connected`, `positionTracked`, `rotationTracked`, `position`, `orientation`, `trigger`, `grip`, `thumbstick{x,y,pressed,touched}`, `buttons{x,y|a,b}`, plus `schemaVersion`, `sequence`, `timestamp` |
| rate | not set by the server — it MEASURES it (`packet_rate_hz`) | measures `receive_hz` |
| frame | **Unity: x right, y UP, z forward** (`unity_to_plot` maps Unity XYZ to plot X,Z,Y "so Unity Y is vertical") | same |
| direction | **receive-only — no `send()` anywhere** | receive-only |
| host | Windows: conda + `.bat`, `adb` from Android Platform-Tools | same |

The client is a **Unity app**, not a browser.

## THE TRANSPORT DECISION: adopt the vendor PROTOCOL, drop WebXR

**`adb reverse` over USB removes the entire HTTPS problem.** WebXR needs a
secure context, which on a LAN means a self-signed cert with the IP in
subjectAltName, a Windows firewall rule, and the operator accepting a warning
in the headset. With `adb reverse` the Quest connects to `ws://127.0.0.1`,
which is a secure origin by definition. No cert, no LAN IP, no firewall rule,
no warning. It is also USB, so it does not share the wifi with the arms.

**MEASURED, and it is what makes this work:** under `networkingMode=mirrored`,
**Windows can connect to a socket bound on 127.0.0.1 INSIDE WSL** — verified
by binding a listener in WSL and connecting from Windows PowerShell
(`WINDOWS->WSL localhost: CONNECTED`, accepted from 127.0.0.1:61830). So
`adb reverse` run on Windows lands directly on a ROS node in WSL. No relay
process, no second hop.

So: **speak the vendor wire protocol from a ROS node in WSL**, on the same
ports, with the same JSON. The user's existing Unity client then works
unmodified. What is NOT reused is the vendor process itself — it binds
127.0.0.1 in a Tkinter/matplotlib GUI on Windows under conda, and it is
receive-only, so it cannot carry the haptics or the state feedback the
in-headset overlay needs. The 31 KB file is mostly GUI; the protocol part is
~50 lines.

WebXR is kept as a fallback for a headset without USB access.

## Environment facts found while doing this

- **WSL interop works only via the lowercase path** `/mnt/c/windows/...`.
  `/mnt/c/Windows/...` returns `Invalid argument` and an empty listing, which
  reads exactly like the dead-9p-mount failure and is not one.
- **`adb` is installed neither in WSL nor on Windows** (`Get-Command adb`
  returns nothing). Android Platform-Tools is a prerequisite.
- **The machine is no longer on the lab network.** `eth0` is DOWN and `eth3`
  holds `192.168.3.146/22`; `ping 192.168.1.10` is 100% loss. Nothing that
  needs a real arm can run here.

---

# DEGRADED MODE, RADIAL FALLBACK, AND THE VIRTUAL TEENSY (2026-08-07)

## The movement problem, root-caused by measurement

**Is there any reach signal without l_j2/l_j4?** Regressed the true 4-pot
reach on EVERY channel that still passes coherence, plus the IMU, over the
2026-08-06 capture:

| arm | live predictors | R^2 | residual RMS |
| --- | --- | --- | --- |
| left | j1, j7 + IMU | **0.133** | 51 mm = **93% of the 55 mm true spread** |
| right | j1, j2, j4, j6 + IMU | **0.986** | 9.1 mm |

**The left arm has NO reach observable and the right needs no help.** That is
expected physically: reach is set by shoulder flexion (j2) and elbow (j4);
j1 and j7 are ROLLS about the arm's own axis and carry zero radial
information, and one WRIST IMU cannot see the elbow without a second IMU on
the upper arm. **This names the repair exactly: l_j2 and l_j4.**

## The radial fallback — j7, and why not the FSR or a button

With j2 and j4 frozen the FK magnitude is a constant, so the operator drives
radius directly on a channel that is **alive, coherent, and unused by
spherical position**: `l_j7` (right arm needs none). It conflicts with
nothing — unlike the FSR (the gripper) or the buttons (the clutch), the
operator can extend while gripping and while indexing.

**RATE, not absolute.** l_j7 spans only 26 deg, so an absolute map would give
26 deg of resolution over the whole radial range and would peg reach to a
wrist angle the operator must hold. Deflection past a 4 deg deadband commands
d(reach)/dt up to 0.12 m/s, clamped to 0.04..0.272 m.

## Cost of degraded mode — radial extent, NOT convex hull

A convex hull cannot see this: the hull of a spherical shell is a solid ball
and reported an identical 0.0387 m^3 whether the shell had thickness or none.
Azimuth and elevation are angular and survive every exclusion, so the only
dimension a frozen pot can remove is the RADIUS.

| arm | reach range OFF | ON | |
| --- | --- | --- | --- |
| left | 0.137-0.272 m (extent 0.135 m) | 0.272 only (**0.000 m**) | collapses to a SURFACE |
| right | 0.133-0.272 m (0.139 m) | unchanged | **0.4%** volume loss |

## THE VIBRATION NUMBER

`scripts/prove_degraded_mode.py` replays the recorded incoherent traces with
every coherent channel and the accelerometer pinned to its median — a
perfectly steady hand on perfect wiring, so every millimetre of output is
contributed by the broken channels alone.

| arm | degraded OFF | ON |
| --- | --- | --- |
| left | **54.527 mm \|std\|** (p-p 177 x, 170 z) | 0.000 |
| right | 0.014 mm | 0.000 |

The ON column is 0.000 by construction. The OFF column is the vibration.

## VISIBILITY — "I could not tell degraded mode was on"

`master_pose_node` now logs a loud startup banner AND publishes
`/master_channel_state` (String, TRANSIENT_LOCAL so a late subscriber still
gets it). Measured live:

    DEGRADED | 6/14 coherent | left: use j17 frozen j23456 reach=j7 @ 0.156 m
                             | right: use j1246 frozen j357 reach=pots

## TWO READERS ON ONE TEENSY IS NOW IMPOSSIBLE, NOT MERELY DETECTABLE

`serial_port.claim_exclusive()` takes `LOCK_EX|LOCK_NB` on the open fd, and
`sniff()` refuses to probe a locked port — **the port SEARCH itself used to
read, so probing a port a live node owned stole bytes from its frame stream:
the same corruption, caused by the code looking for the device.** The refusal
names the holding PID and the recovery. 4 tests against a real pty.

## THE VIRTUAL TEENSY — `scripts/virtual_teensy.py`

A pty streaming real frames in the exact wire format, so the whole master
path is testable between lab visits with **known ground truth**, which
hardware cannot give. `--mode idle` (every channel at its zero reference),
`--mode replay` (real recorded rows, faults and all), `--mode script`
(scripted button presses), `--reach-sweep`. Sensor rate is deliberately
slower than the frame rate, reproducing the 82-88% duplicate-row aliasing.

**Measured with one stack, master IDLE, 60 s, n=2981/2980:**

| | left | right |
| --- | --- | --- |
| commanded EE std | **0.0000 mm** | **0.0000 mm** |
| p-p | 0.000 mm | 0.000 mm |
| worst sim joint p-p | 0.0051 deg (right_joint_7) | |

So **the software manufactures no motion from a stationary input.** Read this
together with the 54.5 mm above: idle mode proves the pipeline is clean, the
replay proves what the real pots inject.

## Two logging bugs, one latent crash in the SAFETY layer

rclpy caches log severity **per call site**. `lg = (error if X else warn)`
followed by `lg(...)` is ONE call site, and the second severity raises
`ValueError: Logger severity cannot be changed between calls`.

- `master_pose_node` hit it immediately (left arm warn, right arm info on one
  line) and **respawned every 6.3 s**, which in the log looked like a serial
  fault.
- **`block_monitor.py:142` had the same shape**, so the first time ANY
  blocker was held past `escalate_s` (3 s) the repeat-log would have thrown
  inside the block monitor's own timer — in the one component whose job is to
  make blocking visible. Verified to raise; fixed to two call sites and
  pinned by `test_escalation_does_not_crash_the_logger`.

Note the ternary-INSIDE-the-call form is tolerated while the
select-then-call form raises; only measurement distinguishes them.

## Memory — the OOM kills are NOT the steady-state stack

One sim stack: **821 MB total RSS** (move_group 188, each ik_follower 85,
estop 76, mount_guard 75, master_pose 74, controller_manager 73, fsr 71,
rsp 34) against a **15.7 GB** WSL allocation with 12 GB free. So the stack is
~5% of available memory and no `.wslconfig` change is warranted. The OOM
kills need a different explanation — the most likely contributor is
**duplicate stacks plus RViz**, and duplicate stacks are now blocked at the
serial layer. NOT root-caused; do not treat this as closed.

---

# CLUTCH, BOOT REFERENCE AND INDEXING — BUILT AND MEASURED (2026-08-07)

## The boot reference was the master-frame ORIGIN

`self.d_ref = {a: np.zeros(3)}`. The first command was therefore
`pos_anchor + scale*(tip - 0) = home + tip`, so the arm jumped by the
master's whole tip vector (~0.22 m) in whatever direction the master happened
to be lying. **That is why the operator had to hold the master at a
calibrated neutral: the only master pose that did not jump was tip == 0.**

Now `d_ref` starts as `None` and the first VALID frame latches it — validity
required, because latching from a frame that failed validation offsets every
later command, the same failure the re-engage path already refuses. Measured
at startup:

    [REF:left]  BOOT REFERENCE latched at the master's ACTUAL pose
                (+0.128, +0.000, +0.182)
    [REF:right] ... (+0.265, +0.000, +0.212)

`/master_rebase` (Trigger) clears it so the next valid frame re-latches, and
**`sim_to_real_bridge` calls it when it enables** — the bridge seeds from the
real arm's actual position, so the master reference must be re-latched at the
same instant or the operator's current pose maps to wherever it mapped at
node startup, and they will have moved in between. The call is
`service_is_ready()` + `call_async`, never `wait_for_service`: that is the
4 s e-stop stall all over again.

## Button mapping is now LIVE, and there is a lab check

`{left,right}_clutch_button` is re-read every frame, so `ros2 param set`
swaps it with no rebuild; a change resyncs `btn_last` so the swap itself
cannot be read as a press. `ros2 run srl_teleop check_buttons` prompts for
one button at a time, reports WHICH INDEX moved, and prints the exact
`ros2 param set` lines. It is **read-only** — a tool that silently "fixes" a
safety-relevant mapping is worse than one that reports.

## A second press now CANCELS a pending engage

Previously every further press while an engage was deferred just re-armed the
same pending state, so a refused engage (frame invalid, or master still
moving) looked exactly like a dead button with no way out.

## MEASURED: 10 index cycles against the virtual board

`python3 scripts/verify_indexing.py --cycles 10`

| | |
| --- | --- |
| re-engage jump | **mean 0.29 mm, max 0.35 mm** |
| drift while frozen | **0.000 mm** — the arm truly ignores master motion |
| total travel | **338.8 mm over 6 cycles**, ~56.6 mm each |
| second half | 169.8 mm — **UNBOUNDED, still advancing** |

So indexing works: freeze is absolute, resume is jump-free, and the
reachable set is bounded by the ROBOT, not by the master's range.

### The first run showed four cycles of 0.00 mm, and that was a real bug

Cycles 2-5 moved nothing. Instrumenting reach showed why: the radial
fallback had integrated to its **clamp**, so pushing the wrist further did
nothing and **said nothing**. That is precisely the silent-blocking class —
a check that can only say no. The clamp now warns by name and states the
recovery. Do not remove it: at either limit the radial axis stops responding
in that direction, and to the operator that is indistinguishable from the arm
sticking.

## PART 1.4 — the binding constraint is IK INFEASIBILITY, so DO NOT sweep the mount

`scripts/diagnose_front_reach.py`, right arm at home:

| dir | reached | cause | limited joints at the stop | yaw free |
| --- | --- | --- | --- | --- |
| front | 0.100 m | **IK INFEASIBLE** | joint_2 57%, joint_6 44% of limit | +0.040 m |
| down | 0.190 m | **IK INFEASIBLE** | joint_2 58%, joint_6 58% | +0.050 m |

Those reproduce the documented baseline (right front 0.100, down 0.190)
**exactly**, which is what says the arm really was at home for this run.

It is **not wearer collision** — the failure persists with collisions off —
and it is **not a joint limit**: the limited joints sit at 44-58% of travel.
The brief's mount-rpy sweep was conditional on wearer collision, so it is
**not indicated**, and a mount change would invalidate P_HOME and the anchor
for no gain. Freeing yaw buys only 40-60 mm.

**The left arm's numbers in this run are INVALID** — it read 0.000 m in both
directions because the indexing test had left it away from home.
`measure_workspace.py` asserts on home for exactly this reason;
`diagnose_front_reach.py` does not, and should.

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

# PART 4 — VR ON THE VENDOR PROTOCOL, BUILT AND MEASURED (2026-08-07)

Full checklist: `docs/system/vr_bringup.md`.

`srl_vr_teleop/quest_vendor_bridge.py` is a WebSocket server on 8766 speaking
the vendor's exact JSON, so the user's existing Unity client works
unmodified. `quest_vendor_mock.py` is a desktop client emitting the same
schema, with scenarios for each safety path.

## Measured against the mock, no headset

| | |
| --- | --- |
| packet rate in | **68.5 Hz** (822 packets / 12.0 s) |
| status frames pushed back | 242 |
| **round-trip latency** | **median 8.66 ms, p95 16.5 ms** |
| tracking-loss freeze (limit 0.20 s) | fired at **0.24 s** |
| link-loss freeze (limit 0.30 s) | fired at **0.32-0.38 s** |
| observer e-stop absent | **refuses to drive at all** |
| clutch cycle | DISENGAGED then ENGAGED with the reference latched at the controller's CURRENT pose |

Round trip is measured by the CLIENT, because only the client has one clock
at both ends; the bridge echoes `timestamp` untouched. Measuring it in the
bridge would need the two clocks to agree, which they do not.

## Frames

    p_world = (x_q, z_q, y_q)          Unity x right, y UP, z fwd
    q_world = (qx, qz, qy, -qw)        -> world x right, y fwd, z up

Swapping two axes flips handedness, which is the conversion required; the
rotation carries it in the negated scalar. **A yaw about Unity's up axis
comes out NEGATED about world z, and that is correct** — +theta in a
left-handed frame is the same physical turn as -theta in a right-handed one.
My first test asserted the sign was preserved and failed; the assertion was
wrong, not the code. The property that actually matters — position and
orientation ending up in the SAME frame — is pinned separately, because if
they disagree IK is asked for a pose that does not exist. 7 tests.

## Two bugs found by running it

1. **`self.clients` collides with `rclpy.Node.clients`** (its service
   clients), a property with no setter. The node died at construction.
2. **The freeze log flooded at 839 ERROR lines in one 11 s run**, because the
   "has it changed" test compared a message containing the elapsed time, so
   every tick looked like a new event. Now compares the freeze CATEGORY, with
   a 5 s heartbeat: 839 -> 21 lines. A flood buries the transition it reports.

## First-person view

`wearer_view` publishes TF `head -> wearer_eyes` (+90 mm forward, +60 mm up,
along the wearer's +y) and a CameraInfo. Verified with `tf2_echo`. Not
cosmetic: whether the arms feel attached or external is what the experiments
measure, and a third-person camera answers that before the study starts.

## NOT built, deliberately

**The Unity-side overlay rendering.** The data channel is built, running and
republished on `/vr_state`; the vendor server cannot do this at all (it is
receive-only, no `send()` anywhere). Writing a Unity scene that cannot be
compiled or tested here would be worse than saying so. The panel spec is in
the bring-up doc.

## Blocker

**`adb` is installed neither in WSL nor on Windows.** Android Platform-Tools
is the one hard prerequisite before any headset test.

---

# FULL AUTONOMY — THE SIX MODES (2026-08-07)

## The mode registry

`srl_teleop/operating_modes.py` — in the package that depends on nothing
in-repo, so every other package can import it without inverting the
dependency arrow the experimental design rests on.

| # | mode | input | robot decides | max_vel | status |
| --- | --- | --- | --- | --- | --- |
| 1 | DIRECT_MANNEQUIN | master | nothing | 0.60 | implemented |
| 2 | DIRECT_VR | Quest | nothing | 0.60 | implemented |
| 3 | ORIENTATION_ASSIST | master + vision | wrist | 0.60 | **STUB** — this `/compute_ik` plugin ignores `OrientationConstraint` (measured: identical IK success with and without it) |
| 4 | SHARED_AUTONOMY | master + vision | wrist, target, approach | 0.60 | implemented |
| 5 | SUPERVISED_AUTO | point/voice | + grasp, transport, place | **0.25** | implemented |
| 6 | FULL_AUTONOMY | voice | + target | **0.15** | implemented; **perception models not installed** |

**Autonomy runs SLOWER than teleop**, by design: nobody is watching, so the
only bound on a wrong motion is how long it takes to happen.

**One safety stack, no per-mode opt-out.** `assert_safety_invariant()` raises
on every transition. 36 ordered mode pairs tested; dropping ANY one of the
six shared mechanisms is refused in ALL six modes (36 x 6 assertions).

## Model selection — decided by a 4 GB VRAM ceiling

This machine is an **RTX A500 Laptop, 4096 MiB**, 15 GB RAM, no torch
installed. That, not benchmark rank, decides most of it.

| need | chosen | why |
| --- | --- | --- |
| STT | **faster-whisper `small`, int8, CTranslate2** | ~1 GB, 3.4% WER. int8 cuts VRAM ~40% and gives ~4x throughput over the reference implementation. `large-v3-turbo` is better English but needs ~6 GB and would leave nothing for the detector, which must be resident at the same time |
| detection | **YOLO-World** (Grounding DINO Swin-T as an optional higher-accuracy backend) | YOLO-World is ~5x smaller and ~3x faster (35 FPS vs 11 FPS on a 4090); on an A500 the accuracy leader would be far too slow, and autonomy is deliberately slow-moving anyway |
| pointing | **Molmo — RULED OUT** | smallest Molmo 2 is **4B** (~8 GB fp16). It does not fit. And its value would be resolving deictic "that thing over there", which mode 6 CANNOT USE: voice-only autonomy has no pointing input to ground against. Weight for a capability we cannot exercise |
| segmentation | **depth inside the detected box**; SAM 2.1-tiny optional | we need a grasp axis, not a pixel-perfect mask, and the RGB-D stream already gives the 3D structure. SAM 2 with visual prompts has been reported at ~10 GB on high-end GPUs; tiny is far less but still competes for a budget that is already spent |
| intent | **deterministic grammar, NO LLM** | see below |

### Why a grammar and not a local LLM

The command set is closed and tiny: grab, handover, place, stop. A 7B model
quantised to fit would take 3-4 GB of the 4 GB that EXISTS, add 200-800 ms,
and put non-determinism in the component that decides whether a robot beside
a person's head starts moving.

The failure modes differ in KIND. A grammar that does not recognise an
utterance returns `unparsed` and the robot asks again. An LLM asked for JSON
can emit a **different, plausible** verb, and nothing downstream can tell
that from a correct parse. For five verbs that is a bad trade.

### WHY NOT AN END-TO-END VLA — do not reverse this later

1. **Embodiment.** OpenVLA, pi0 and similar are trained on Open X-Embodiment,
   which contains no backpack-mounted dual-Kinova SRL. Zero-shot transfer to
   a novel embodiment is poor, and fine-tuning needs demonstration data this
   project does not have.
2. **It bypasses the safety stack.** A VLA emits actions directly. Every gate
   this repo has — collision-aware IK, the clearance floor, graduated
   avoidance, the step guard — sits between a *pose* and the arm. A model
   that emits joint targets goes around all of it. On a robot mounted beside
   a person's head that is the wrong trade.
3. **Unexplainable failure.** The modular pipeline fails at a NAMED stage
   with its inputs logged (`/autonomy_decision`). A VLA failure is a number
   that was wrong.

## MEASURED — mode 6, 10/10 checks

`python3 scripts/verify_autonomy.py`

| check | result |
| --- | --- |
| no matching object | REFUSED, "I can't see anything matching 'green cube'" |
| ambiguous (2 matches) | **ASKS** — "on the left and on the right. Which one?" and picks NOTHING |
| below confidence floor | REFUSED |
| target BEHIND the wearer | REFUSED — "I will not plan a path there" |
| deictic with no pointing input | REFUSED |
| unknown verb | "Sorry, I didn't understand that" |
| no wake word | **silent** |
| accept | announces, waits, then moves. **voice -> motion 2008 ms** (2 s of that is the configured confirmation wait) |
| voice "stop" mid-sequence | **2 ms**, `/estop` published |
| decision log | intent, resolve, pick_target, voice_stop — all with inputs |

### Two real bugs found by running it

1. **The wearer keep-out was evaluated too late.** It sat in `start_pick`,
   i.e. AFTER disambiguation, so a target behind the wearer became one of
   three candidates and the robot asked *"which one -- left, right, near or
   far?"*, offering a position it must never reach. Asking the operator to
   choose an option the robot would then refuse is worse than not offering
   it: it invites them to say "the far one" and trust the answer. The
   keep-out is now a FILTER applied before resolution.
2. **A no-wake utterance was answered** with "Sorry, I didn't understand",
   which defeats the wake word — the robot was replying to conversations it
   was never addressed in. Now silent unless the wake word was present.

Also: `self.handle` collided with `rclpy.Node.handle` and killed the node at
construction — the same class as `self.clients` in the VR bridge. rclpy
reserves more names on Node than is obvious.

## T4 INTER-ARM HANDOVER — THE PREMISE IS FALSE, AND THAT IS THE FINDING

The brief expected: teleop cannot hand over because `orientation_mode: fixed`
gives both wrists the same approach direction, but modes 5/6 command full
6-DOF and can give each arm its own wrist angle — "a task impossible under
teleoperation and possible under autonomy".

**Measured in sim, arms at home, 16 candidate transfer points including the
protocol's own transfer station (-0.05, 0.28, 1.060):**

    points reachable by BOTH arms: 0 of 16
    protocol transfer station: left=False right=False

**A handover point does not exist.** The blocker was never wrist
orientation — the two arms' reachable sets are DISJOINT (see the Part 3
finding: 0 of 63 frontal cells reachable by both, entire centreline reachable
by neither). Giving each arm its own wrist angle cannot help, because there
is nowhere to hand anything over.

So T4 is blocked for **both** teleop and autonomy, by geometry, until the
right arm is re-parked in hardware. The "impossible under teleop, possible
under autonomy" result is NOT available on this rig as configured — and
claiming it without this check would have been wrong in print.

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

# GUI LAUNCHER — `scripts/srl_launcher.py` (2026-08-07)

**Primary interface.** `teleop_gui` remains the terminal fallback for SSH.

    python3 scripts/srl_launcher.py
    ros2 run srl_teleop launcher          # same thing, via a shim

## WSLg works — verified before anything was built

    DISPLAY=:0   WAYLAND_DISPLAY=wayland-0   /mnt/wslg present
    Tk 8.6, window created, screen 1920x1200, mainloop entered and exited

## What it does

Four tabs — Teleoperation, Autonomy, Experiments, Calibration & diagnostics —
plus an always-visible live status pane, live parameter sliders, and a large
red E-STOP. Every button launches a managed subprocess in its own process
GROUP, and its output is pumped to a scrollable console. The only second
window needed is RViz, which opens its own.

## The four things that make it more than a button board

**1. It refuses to start a second stack.** Preflight lists every running
stack process and names why: two `master_pose_node` instances split the
serial stream and invalidated a full day of measurements. Measured:
`refused: True`, `job created: False`, with the reason shown.

**2. Preflight shows what failed instead of starting and dying.** With no
Teensy and off the lab network, `Mannequin -> real` refuses with BOTH causes
listed, and **the confirmation dialog is never reached** — preflight runs
first.

**3. Live controls use parameter CLIENTS, never `ros2 param set`.** The CLI
goes through the ros2 daemon, which hangs on this box and has reported
success it had not earned. Every slider calls SetParameters directly and
reads the value back:

    [PARAM OK] /ik_follower_left max_vel_rad_s: 0.6000 -> 0.3000
    [PARAM FAILED] /no_such_node x -> 1.0000 : no set_parameters service

A failure raises a dialog saying **the value on the robot is UNCHANGED**.

**4. Clean shutdown.** SIGINT to the process group FIRST (the Kortex bridge
closes its session on SIGINT and LEAKS it on SIGKILL, and the arm permits
exactly one session), then SIGKILL after a grace period, then an
explicit-PID sweep of orphans. Verified: **0 real stack processes after
close.**

Disabled buttons say why on hover: Mode 3 (`OrientationConstraint` ignored by
this plugin), T2 (needs the redesigned container and would displace T5).

## Two threading bugs found by clicking, not by reading

1. **`log()` from a worker thread** raised `main thread is not in main loop`
   and killed the worker mid-flight, so a failed parameter write produced
   NEITHER an echo NOR a dialog — a silent failure in the component built to
   make failures loud.
2. **`self.after()` from a worker thread raises the same error.** The fix is
   a `queue.Queue` drained by the main-thread tick; workers never touch Tk.

## And one in the verification, worth recording

The first "leftover processes after close" check grepped for bare names
(`move_group`, `master_pose_node`) and **matched its own heredoc**, which
quoted those words — reporting 2-3 leftovers when there were none. The check
now matches installed executable PATHS
(`lib/moveit_ros_move_group/move_group`), which a shell command line cannot
accidentally contain. Fourth instance of the measuring instrument being the
fault, and the reason the standing rule exists.

## Verified by clicking through every path

| | result |
| --- | --- |
| window opens under WSLg | yes, ROS link connected |
| Sim only | 6 stack processes, **546 lines captured** in the console |
| second stack | REFUSED by name, no job created |
| Mannequin with no Teensy | refused, cause shown |
| Mannequin -> real, no Teensy + off-network | refused with BOTH causes, confirm never reached |
| Autonomy mode 4 | started, banner captured |
| param apply / param failure | echoed / dialog raised |
| pilot button | ran, produced `t3 72 trials` |
| E-STOP | published, status flips to `E-STOP LATCHED` |
| status pane | STACK, SAFETY, ACTIVE BLOCKERS, CHANNELS, FOLLOWERS, REAL ARM |
| window close | **0 real stack processes** |

## teleop_gui hung on quit — found by the launcher work, not by the GUI's own tests

While verifying the launcher, the terminal console's pty tests began failing
with exit status 250. It was not a crash: **`teleop_gui` never exited at
all** on `q`. `ex.shutdown()` on a MultiThreadedExecutor blocks indefinitely
if a callback is still running, and the test's wait then expired.

From the outside a hung exit is indistinguishable from a crash — the same
class as every other silent-stop in this project. The teardown is now
time-boxed (`ex.shutdown(timeout_sec=2.0)`, each step guarded) and ends with
`os._exit(0)`: curses has already restored the terminal, so there is nothing
left to clean up in that process, and waiting on an rclpy thread that may
never return is the worse failure. 5/5 pty tests pass.

---

# SRL CONSOLE — VISUAL REDESIGN (2026-08-08)

Three principles taken from operator-display practice (ISA-101, NASA display
standard, annunciator panels):

1. **Colour is a scarce alarm channel.** Low-saturation grey base; saturated
   colour reserved for the abnormal. The corollary I had wrong: a NORMAL
   state is largely UNCOLOURED, not green. The old safety bar painted five
   items red at once, which is the same as painting none.
2. **Hierarchy from size and weight, not boxes and lines.** Three type levels
   only, monospace for numerics so digits are fixed-width, grouping by
   whitespace.
3. **A sentinel must never look measured, and the payload must never be
   truncated.** `-1.000 m` reads as a measurement; `--` reads as absent. The
   REASON is the actionable content -- shorten the label instead.

## What changed

| defect | fix |
| --- | --- |
| pipeline overflowed, last stage cut mid-word | **VERTICAL list**, full-width rows |
| status floated outside the box it described | status colour tints the ROW; label, status and reason are one unit |
| reasons truncated mid-word | `wrap=`, never truncated |
| one font size | three levels (30 px mono bold / 17 px mono / 13-15 px sans) |
| two thirds empty | pipeline fills it; recent events below |
| undifferentiated red safety bar | ONE focal readout; observer amber; caps and frame time plain grey |
| clearance `-1.000 m` | `fmt()` renders every sentinel as `--` |
| 8 stages BLOCKED vs "no blockers reported" | reconciled -- see below |

## The contradiction, reconciled

The two panels answer different questions and one said so badly. `/blocking`
is BlockMonitor **inside ik_follower_node**: it reports guards the FOLLOWER
registered, not whether data is reaching it. When the follower is not running
there is no publisher at all, which is NOT "nothing is blocking". The panel
now states which, and carries the pipeline's own count.

## FIRST-OUT ANNUNCIATION — the change that mattered most

The first redesign still rendered **nine of eleven rows red**. Those were not
nine problems; they were one problem and eight consequences. Annunciator
panels solve this with first-out logic: only the ORIGINATING alarm shows at
full severity. Now exactly one row is red (the root cause), downstream stages
render grey as "consequence of <stage> -- fix that first", and the blocker
text says "root cause Teensy, 9 downstream" so the number matches the
display.

Measured frame time after the redesign: **median 0.01 ms, p95 0.94 ms,
max 2.70 ms** against a 33 ms budget.

# SRL CONSOLE — Dear PyGui operations GUI (2026-08-08)

    python3 scripts/srl_console.py        or  ros2 run srl_teleop console

Three interfaces now, deliberately:

| | use |
| --- | --- |
| **`srl_console`** | **primary.** Dear PyGui, real-time, full instrumentation |
| `srl_launcher` | tkinter. Simple launch + status, no plotting |
| `teleop_gui` | terminal fallback for SSH with no display |

## Framework chosen by measurement

    Dear PyGui under WSLg, 110 frames with a live plot:
        median 1.85 ms   p95 2.29 ms   max 2.89 ms

WSLg's D3D12 layer caps hardware OpenGL below 4.0; Dear PyGui targets 3.3 and
renders fine. **tkinter+matplotlib rejected** — a canvas redraw costs tens of
ms and Tk's threading model already bit twice in the launcher. **A FastAPI web
UI is the better long-term answer** (phone, Quest browser, SSH port-forward)
and is deliberately deferred: it is a second process, a second failure mode
and a websocket to debug, when the point of this tool is to debug everything
else.

## Architecture

ROS owns one thread and every subscription, service call and parameter write.
The draw loop reads ONE immutable snapshot dict, rebound atomically — it never
calls a service, never waits, never locks. Parameter writes are QUEUED to the
ROS thread and answered by callback. Tiered refresh: charts 30 Hz, numerics
10 Hz, verdicts 1 Hz, process/disk 0.2 Hz.

**MEASURED FRAME TIME UNDER FULL LOAD** (stack running, autonomy node,
subprocess streaming, a node killed underneath it):

    median 0.01 ms    p95 3.13 ms    max 4.85 ms     budget 33 ms

## The two panels that justify the whole thing

**PIPELINE FLOW** — eleven stages from Teensy to real arm, each FLOWING /
STALE / BLOCKED with the reason, clickable for detail.

**[WHY IS NOTHING MOVING?]** walks the chain and names the FIRST stage not
passing data. Verified: with no Teensy attached it answers
`FIRST STAGE NOT PASSING DATA: Teensy [BLOCKED] no /dev/ttyACM* and no frames`.

**The master-arm panel leads with STALENESS, not value.** Per channel:
update count, **time since last DISTINCT value**, and a >60 deg jump counter.
A channel unchanged for more than 2 s renders RED even while data arrives —
the dead-man bug was stale data republished with fresh timestamps, and the
0.000 noise floor was rows duplicating faster than the sensor updated. A value
that looks live but has not changed is the failure this panel exists to show.

## Verified by clicking every path

| | result |
| --- | --- |
| Dear PyGui renders under WSLg | yes |
| Sim only | 3 stack processes, pipeline goes FLOWING |
| Mode 4 alongside a running stack | starts (add-ons are not second stacks) |
| second STACK | refused by name |
| param apply / failure | `0.6000 -> 0.6000` / `[error] /nope ... FAILED` |
| **kill move_group underneath the GUI** | **GUI keeps rendering**, count drops 3 -> 2 |
| E-STOP | `E-STOP LATCHED` in the always-visible bar |
| pilot / diagnostics buttons | ran, output streamed |
| window close | **0 stack processes** |

## Two bugs found by clicking

1. **IK reported `BLOCKED — success 0%` with ZERO attempts.** No attempts is
   not failure; it pointed the operator at the solver instead of at the
   missing input. Now `STALE — follower up, NO poses received yet`. Same
   no-data/bad-data conflation this GUI exists to prevent.
2. **The second-stack refusal was too broad** — it blocked autonomy nodes and
   diagnostics, which are ADD-ONS to a running stack. Refusing those whenever
   a stack exists makes the refusal useless in the only situation where you
   would use them. Now scoped by `starts_stack`.

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

# SCREEN-RECORDING RVIZ UNDER WSLg (2026-08-08) — READ THIS BEFORE TRYING

## x11grab ON `:0` RECORDS BLACK. This will cost you a day.

`ffmpeg -f x11grab -i :0` with RViz plainly visible on screen produces a
**black video**. Measured, not inferred:

| grab | mean pixel value |
| --- | --- |
| full screen 1920x1200 on `:0`, RViz visible | **0.0** |
| the RViz window region on `:0` | **0.0** |
| the same RViz on **Xvfb `:99`** | **126.8** (std 110.4, 40742 distinct colours) |

**Why.** WSLg runs a Wayland compositor with XWayland. Window contents are
composited by Wayland and **never land in the X root window** that x11grab
reads. There is nothing wrong with the ffmpeg command; the pixels are not
there to be read. Nothing on the X side can fix it — not window ids, not
`-window_id`, not region offsets. All were tried.

## THE FIX: a virtual display with its own RViz

```bash
Xvfb :99 -screen 0 1600x1000x24 &
DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb \
  rviz2 -d src/srl_experiments/config/verification_capture.rviz &
ffmpeg -f x11grab -video_size 1600x1000 -framerate 12 -i :99.0 out.mp4
```

Xvfb has **no compositor**, so its root window really does hold the rendered
pixels. `LIBGL_ALWAYS_SOFTWARE=1` is required — llvmpipe renders RViz's
OpenGL fine and reports GL 4.5. This is a **second** RViz instance subscribing
to the same ROS graph; it is not a second stack and does not violate the
one-stack rule (no `master_pose_node` involved).

`scripts/record_rviz.py` does all of this itself and is idempotent about it.

**ffmpeg is not installed system-wide.** A static build was obtained without
root via `pip install --target <dir> imageio-ffmpeg` and copied to
`~/.local/bin/ffmpeg`. **`ffprobe` was NOT** — a verifier that shells out to
`ffprobe` silently reports "0 frames" for every clip, which is what the first
version of `verify_rviz_clips.py` did. Sample frames by TIME (`-ss`) instead
of by frame index; it needs no frame count.

## THE CLIP VERIFIER MUST BE CALIBRATED ON *RENDERED* COLOUR

`verify_rviz_clips.py` proves an object is on screen by counting pixels of its
colour. **Matching against the RGB set on the marker fails**, because RViz
lights and shades every surface:

| object | requested RGB | actually rendered |
| --- | --- | --- |
| ball | 242, 191, 26 | **189, 165, 74** |
| sling | 140, 89, 46 | 136, 111, 63 |
| target green | 26, 230, 51 | **68, 151, 63** |
| container teal | 13, 191, 179 | **45, 141, 140** |
| block orange | 255, 115, 0 | **207, 138, 35** |

The first version failed **11 clips whose objects I had just looked at**.
Detection now uses RELATIONS between channels (`R - B > 65`, `G - R > 45`),
which survive shading, with thresholds taken from pixel counts measured on
frames confirmed by eye. Validated both ways: every confirmed-good frame
detects its object (38-860 px), a black frame detects none.

**FOUR more calibration traps in the same file, all found by checking the
instrument against clips I had already looked at:**

- **The overlay text is detected as the object.** The HUD is white and
  orange-yellow — the same channel relations the ball and sling detectors
  match. Measured on one T6 frame: **587 "ball" pixels, of which 58 were the
  ball and 529 were the letters.** Because the text never moves it dragged
  every centroid toward a fixed point and made carried objects read as
  static. Every detector now ignores the top 24% of the frame.
- **Four temporal samples is UNDERSAMPLING.** Every clip carries ~1.5 s of
  approach and ~1.4 s of hold, so on a short scenario the moving part is a
  thin slice and four probes land mostly in the static hold. T3 S1 (a 50 mm
  lift) read 7.1 px and was called static; with nine samples it reads
  25.9 px. T7 S1 went 0.0 -> 27.5 px the same way.
- **Two targets share one centroid.** T7 and T9 draw a target per arm; their
  COMBINED centroid sits between them and barely moves even when both are
  orbiting. Measure each half of the frame separately.
- **A target sphere gets OCCLUDED BY THE GRIPPER** once the arm arrives on
  it, so the colour vanishes from the later frames of a perfectly good clip.
  Pursuit and reach tasks (T7/T8/T9) have no carried object at all and are
  judged by the arm's own travel and tracking error instead. The target
  marker was also enlarged to 100 mm and made translucent so it reads as a
  halo around the gripper rather than disappearing inside it.

**Two structural traps:**

- **"Nothing moved" is not a failure.** T9's zero-sway scenario is a
  stationary arm holding a world-fixed point ON PURPOSE. The check must fail
  only a genuinely FROZEN capture (frame delta < 0.05), not a still one.
- **Objects must be gated to the phase that owns them.** Drawing the sling
  during the approach — when the grippers are still ~1.46 m apart, because
  they start at home — makes a 350 mm sling read as taut and the ball "falls"
  before the task begins. S4 reported a drop on a scenario whose 58 mm sag
  clears the 40 mm ball comfortably.
- **MarkerArray must begin with DELETEALL every frame.** The topic is
  TRANSIENT_LOCAL and ids are reassigned per frame, so without it the previous
  run's container and blocks stay on screen underneath the next task's
  objects — two scenes in one picture.

## Two recordings per run, and they answer different questions

| file | what it is |
| --- | --- |
| `rviz.mp4` | **real screen capture.** What you would see at the machine. Watch this |
| `clip.mp4` | TF-rendered 3-D + front view. Ugly, but drawn from exactly the samples that produced `summary.json`, so picture and numbers cannot disagree |

