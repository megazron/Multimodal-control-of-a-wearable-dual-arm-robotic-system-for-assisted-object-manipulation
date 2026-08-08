# Calibration

One script, `scripts/calibrate.sh`, and an order that matters.

## 1. Pot zeros and the arm axis — `calibrate.sh zero`

Captures the pot zero offsets and `a_hat`, the arm's long axis in the wrist
IMU's frame, with the arm **hanging straight down and still**.

`a_hat` is what makes elevation mount-independent and drift-free.

**SIGN CONVENTION — do not "fix" it.** Hanging = **−90°**, horizontal = 0,
straight up = **+90°**. `capture_zero.py` asserts this at capture time. The
original brief specified both `a_hat = -normalize(accel_rest)` *and*
`elevation = asin(-dot(...))`; those compound and put the resting arm at +90°,
flipping z alone. Only one negation is correct.

The capture self-checks and refuses on: `|accel|` outside 1 ± 0.05 g (the arm
was moving), accel spread too large, gyro residual > 1 °/s, and a rest
elevation that is not −90°.

## 2. Gyro bias — `calibrate.sh gyro`

**The trap that cost a factor of 34.** Bias must be captured from a **settled**
stream. Reopening the serial port **resets the Teensy**, and the IMU emits
nonsense while settling. Captured through a port reopen, the same gyros read
std 4.85 / 27.9 °/s and a bias of 23.4 °/s; sampled from the settled ROS topic
they read std ~0.11 °/s. That one mistake was the difference between
1492 °/min and 44 °/min of measured drift.

**Stop the publisher cleanly, or sample `/master_arm_raw_*` rather than the
port.**

## 3. IMU mount — `calibrate.sh imu-mount`

Historically this produced physically impossible results (calibrated q3
spanning 336°, q6 ±130–170°). It is **not** required by the shipped pipeline:
elevation uses `a_hat` and gyro-aided azimuth uses `dot(omega, u_hat)` with
both vectors already in the sensor frame, so neither needs a mount rotation.
Run it only if you intend to revisit FK-based azimuth.

## 4. Master travel — `calibrate.sh max-position`

Records the master's reachable range per channel. Feeds the 0.22 m ball figure
that E1's target distances are chosen against.

## 5. The complementary filter time constant

Not a capture — a choice, and it has been measured:

| tau (s) | tilt error, left arm | tilt error, right arm |
| --- | --- | --- |
| **0.2** | **0.13° mean, 0.28° p95** | **0.57° mean, 1.13° p95** |
| 0.5 | 0.19° / 0.38° | 0.93° / 1.88° |
| 1.0 | 0.25° / 0.58° | 1.42° / 2.91° |
| 2.0 | 0.35° / 0.76° | 2.29° / 4.52° |
| 5.0 | 0.71° / 1.54° | 5.04° / 7.60° |

Smaller tau is better **against a static reference**, because the
accelerometer is the accurate sensor when nothing is moving. Do not read this
as "always use 0.2": tau also sets how much the gyro is trusted through fast
motion, where the accelerometer is contaminated by the arm's own acceleration.
0.2–0.5 s is the useful range; the right arm's gyro is 6× noisier and pushes
the choice toward the smaller value.

## What must be re-derived when the MOUNT changes

`offset = P_HOME`, so a mount change invalidates:

- `WORKSPACE_CENTRE` and `WORKSPACE_ORIENT` in `master_calibration.py`
- every clearance figure
- the continuous-reachability sweep
- E1–E5's `home_xyz` in each `config.yaml`

There is a script for the first three: see the mount section of `CLAUDE.md`.
