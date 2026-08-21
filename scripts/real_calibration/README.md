# Real-arm environment calibration

Built 2026-08-21, after a session in which a planned pick commanded a path
through the mannequin and nothing in the code objected. Everything here exists
because something specific went wrong; each module's docstring names it.

## Run this first, every session

```
python3 scripts/real_calibration/check_all.py
```

Four modules, each of which must recover a known answer **and** fail on a
deliberately broken input. If this does not pass, do not run a live session.

## Then, with the arms up

```
# 1. plan and check everything, move nothing
python3 scripts/real_calibration/sweep_workspace.py --arm both --dry-run

# 2. the real sweep: walks outward in 26 directions, records every point
.venv_vision/bin/python scripts/real_calibration/sweep_workspace.py --arm left

# 3. solve the camera pose from the same recording
.venv_vision/bin/python scripts/real_calibration/solve_extrinsic.py --arm left
```

## What each module is for

| module | what it does | the defect it exists because of |
| --- | --- | --- |
| `safe_motion.py` | densifies a move, checks the WHOLE path against the wearer, drives it, verifies arrival, watches the joint stream | a pick checked only its three waypoints; the bridge interpolates in joint space, so the sweep between two clear poses went through the wearer. Also: `goto()` printed "reached" after a timeout on a 39.89 deg error, and nothing noticed the arm had dropped off the network |
| `arm_ik.py` | position IK seeded from a sampled table, reporting the achieved residual | `grasp_pipeline.reachable()` seeds from uniform-random joint vectors and reports "outside the arm's 902 mm reach" when they all fail. A target it refused at 301 restarts was walked to 42 mm by plain random sampling |
| `scene_cameras.py` | the RealSense and the webcam at settings measured to work over usbip | depth at 640x480 **silently freezes** — librealsense re-delivers the last frame, so it reads as 25 fps while the content never changes. And the RealSense is mounted upside down, where rotating the image without rotating the principal point moves a ray 69 mm at 2.5 m |
| `solve_extrinsic.py` | camera → robot base by ICP against the arm's own model | two centroid-matching attempts failed at 267 mm and 354 mm. A centroid throws away the shape; the camera sees a front surface and the model is an axis, and that offset is not constant across poses |

## The numbers that were measured, not assumed

* **RealSense over usbip**: depth 424x240 + colour 640x480 @15 gives 60/60
  distinct depth frames. 640x480 depth gives 4–33 of 60 with timeouts, or
  looks perfect and is frozen. 848x480 gives nothing.
* **Rotated principal point**: `cx' = 320.13, cy' = 247.78` against a native
  `318.87, 231.22`.
* **Mount cap**: the whole-chain wearer clearance reads a constant
  **0.2202 m** for every pose — that is the immobile stub, and it binds
  nothing. Always use `moving_chain_m`.
* **Wearer frames are prefixed**: the real stack publishes `real_torso`,
  `real_head`, `real_human_left_hand`. `real_homing_node` was looking up the
  arm link as `real_<arm>_<link>` and the wearer part as a bare `torso`, so
  the guard resolved **0 of 9 parts in every run that has ever been made**.
  Fixed by naming the prefix once and applying it to both operands.

## What is still unverified

The extrinsic solver is validated on **synthetic** data only (0.124 deg,
1.6 mm). It has never been run against a real recording, because the sweep it
consumes has not been run — the left arm dropped off the network three times
on the day this was written. Treat its first real output as a measurement to
be checked, not a result.
