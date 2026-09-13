# Hardware: Teensy, channels, calibration, grippers

Master arm sensing, channel health, the spherical position model, calibration procedures and the gripper stack. Split out of docs/ENGINEERING_LOG.md on 2026-08-12. Nothing deleted.

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
