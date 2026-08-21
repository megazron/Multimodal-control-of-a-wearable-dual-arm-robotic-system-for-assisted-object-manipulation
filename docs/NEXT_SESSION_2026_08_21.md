# RESUME POINT 2026-08-21 — REAL ARMS, VISION, AND FOUR SAFETY DEFECTS

## THE ONE-PARAGRAPH VERSION

Both real arms were driven for the first time: homed, e-stop proven on each,
and a new **wide home** written to all four carriers. The vision stack is
built and tested (FastSAM + colour ensemble, geometric grasp, 132 tests). The
cube was **not** picked up, for one reason only: nothing here can say where it
is in robot coordinates. Four defects were found on real hardware, all of
which would have bitten a lab day, and all are fixed and pinned by tests.

## STATE AT SHUTDOWN

* Both Kortex sessions **closed cleanly** (logged twice). No leak.
* Arms: **left at the wide home**, right wherever homing last left it.
* Sim stack down. SHM cleared. Nothing running.
* Test gate **green**: 1030 passed, 2 failed (both allowed), 0 NEW.

## THE FOUR DEFECTS FOUND ON REAL HARDWARE

1. **The arm can be stuck in LOW_LEVEL_SERVOING and everything looks fine.**
   Session created, joint states at full rate, homing accepted -- and every
   speed command silently rejected with "Invalid command for the current
   servoing mode". The arm never moved while the homing node reported 176 deg
   of error as though it had. The servoing mode is ARM state and survives a
   crashed session. `kortex_highlevel_bridge` now reads it, sets
   SINGLE_LEVEL_SERVOING, reads it back, and refuses if it did not take.

2. **The wearer clearance guard returned "infinitely clear" when it had no
   data.** A 176 deg sweep ran with `0 of 9 parts checked` and `clear inf m`
   on every line. `if not pts: return inf` meant the guard was inert and the
   comparison against the floor always passed. Now returns NaN and HALTS --
   demonstrated on the real arm, which refused to move.

3. **`vmax_rad_s` and friends were read once at start-up.** `ros2 param set`
   answered "successful", the store held the new value, and the control loop
   used the old one forever. A 36-run speed calibration swept 0.10/0.25/0.50
   and every run actually executed at 0.40. Live parameters now apply, and
   structural ones are refused WITH A REASON.

4. **`verify_gui_buttons.py` could move the robot.** It stubbed `on_launch`
   but not `_run_raw`, so pressing `START REAL ARMS` during an audit ran
   `start_real.sh` for real -- whose first act is a teardown -- and it closed
   both live Kortex sessions mid-session. An audit that can move a robot is
   not an audit. Now stubbed at the same seam.

## THE NEW HOME

`|x| = 0.740 m`. Seam margins **0.4576 (L) / 0.3477 (R)** against the 0.30
rule -- the previous candidate sat at 0.2963. In all four carriers; all nine
consumers read `load_home_radians`, so every control mode has it.

Homing reaches **0.03-0.12 deg** per joint. Do not add an integral term to the
bridge for that path -- `real_homing_node` already has one and two in series
made it WORSE (0.19-0.88 deg, measured). The bridge's integral is suppressed
whenever the sender supplies velocities, and applies only to the position path
`sim_to_real_bridge` uses.

## WHY THE CUBE WAS NOT PICKED UP

Detection is solved. **FastSAM on the scene frame: 89 segments, exactly one
green, at pixel (647, 316), zero false positives** -- the room's teal robots
get segmented and rejected on region-mean colour, where three earlier
per-pixel filters had been fooled (their shadowed hue is 76, the cube's 74).

What is missing is 3D:

* the **scene camera** sees the cube perfectly and has no calibration, so its
  pixel is a direction. Arm-as-ruler gave 24 px reprojection = +-75 mm against
  a 30 mm gate, because a motion-diff centroid is not a fixed point on the
  gripper. **A marker fixes this** -- `scripts/calibrate_scene_camera.py` is
  written, dry-run clean (10 stations, clearance >=0.200, seam >=0.35), and
  refuses above 2.5 px.
* the **wrist camera** has depth and works, but twelve guessed viewpoints all
  looked at the mannequin's chest or the teal robot. It needs a starting
  direction.
* the **RealSense D435i** enumerates but will not stream over usbip
  (isochronous transfers). It negotiated USB 2.0 -- max 6-15 fps, where USB3
  gives 90 -- so suspect the cable first. One stream works briefly, two never,
  one attempt dumped core.

**The next session should do ONE of:** put a magenta marker on the gripper and
run the calibrator; or be told roughly where the cube sits and point the wrist
camera there; or bridge the RealSense from the Windows side over TCP.

## WHAT IS NEW AND WHERE

| file | what |
| --- | --- |
| `srl_perception/rgbd_grasp.py` | geometry: deproject, cloud, min-width frame, grasp, pregrasp |
| `srl_perception/prompt_detector.py` | prompt -> detections; FastSAM + YOLO-World + colour ensemble |
| `srl_perception/srl_cameras.py` | scene (MJPG) and gripper (ffmpeg colour + GStreamer depth) |
| `srl_perception/grasp_pipeline.py` | detect -> depth -> grasp -> reach, each refusing BY NAME |
| `srl_perception/scene_calibration.py` | marker-based scene camera solve, with a verdict |
| `scripts/find_object.py` | the CLI |
| `scripts/calibrate_scene_camera.py` | the calibrator (needs a marker) |
| GUI **Real arms** + **Vision** panels | RUN tab, all modes. Audit 203/203 |
| `docs/system/20_vision_and_grasp_as_built.md` | the full write-up |
| `recordings/baselines/arm_reach_extents.json` | measured envelope, 253-1512 mm |
| `recordings/baselines/workspace_waypoints.json` | 48 safety-checked waypoints, never recorded |

## TWO NUMBERS WORTH REMEMBERING

* **Home already sits 771 mm out of a 902 mm arm.** Most directions run out of
  arm, not out of permission. The room has metres; the arm does not.
* **Neither arm crosses the centreline.** Best case is a 480 mm dead band at
  y=0.45. A single object reachable by BOTH arms does not exist in this
  geometry -- confirmed with a 41-seed solver, not quoted from the docs.

## STILL OPEN

* the cube (above)
* the 48-waypoint workspace recording -- generated and checked, never run
* right joint_5 rests at 0.976 deg where left settles to 0.091
* the RealSense, if the cable turns out to be fine
