# What the first real runs measured, and what was wrong with them

Runs on 2026-08-30 with `scripts/cv_pickpose_visuals.py`, four cameras, 33
stages each. This file is the honest read of them: what the numbers say, and
which of the failures were the rig's and which were the program's.

## The short version

| run | left_gripper | right_gripper | scene_rs | scene_hd |
| --- | --- | --- | --- | --- |
| `20260830_065146` | NO FRAME (vision module wedged) | 28/33 | NO FRAME (stream mode) | 12/33 |
| `20260830_070529` | 27/33 | 29/33 | 17/33 | 12/33 |

**0 stages crashed in any run.** Every non-result is either a refusal that
names its reason or a by-design absence.

## Four things were wrong. Three were the program, one was the rig.

### 1. The depth working band was the wrist camera's, applied to every camera

`PICK_Z_MIN, PICK_Z_MAX = 0.08, 1.5` is the pick path's own gate and is
correct for a camera on the hand. The RealSense sits across the room: its
nearest return is **0.741 m** and its median is **3.84 m**, so the gate kept
**0 of 85 061 returns** and sixteen of that camera's stages refused for want
of depth the camera had plenty of.

A gate is part of a measurement, so it belongs to the thing being measured.
The band is now per source: wrist `0.08-1.5 m`, scene `0.15-8.0 m`, and every
figure states the band it used.

This is the instrument failure this repository keeps paying for -- a
surprising failure that was evidence about the INSTRUMENT, not the camera.

### 2. The aligned depth was stipple, so "what is standing on the surface" saw nothing

`on_surface` refused on both gripper cameras -- *nothing stands more than 5 mm
above the fitted plane* -- while `boxes3d`, on the same frame, posed **6
objects, the nearest at 0.536 m**. Both cannot be true.

Cause: depth is 480x270 and colour is 1280x720, so forward-projecting one into
the other lands one sample per ~7 colour pixels. Measured: **7.22% of colour
pixels got a depth**. The above-plane mask was therefore a dot pattern, every
connected component fell under the 40 px floor, and the stage that needs
connectivity reported nothing while the stage that works per mask reported six
objects.

Fixed by splatting each projected sample over the gap it spans, with the
radius derived from the resolution ratio rather than chosen.

### 3. The left arm's vision module was wedged (this one was the rig)

Port 554 on `192.168.1.10` was not listening while the arm pinged, its Kortex
API answered on 10000, and the camera driver retried for ever. Recovered with
`reboot_vision_module.py left` -- **RTSP back after 38 s** -- after stopping
the bridge with SIGINT (`kortex session closed cleanly`).

The RealSense had two faults of its own: it enumerated at **USB 2.1**, where
the stereo module does not offer 424x240 at all, so the hardcoded request
returned "Couldn't resolve requests" on a camera that was working perfectly at
a mode nobody asked for; and a transient `errno 16` during negotiation
demoted it to its worst mode (640x480@6, **13%** of pixels valid) when
480x270@15 was available (**27%**). Both fixed: the device is asked what it
has, and each mode gets more than one attempt.

### 4. THE ARMS WERE NOT AT THE PICK POSE, and that changes what the figures show

| run | left | right |
| --- | --- | --- |
| `20260830_065146` | **82.27 deg out** | **66.10 deg out** |
| `20260830_070529` | **79.99 deg out** | 1.19 deg -- at the pick pose |

The right arm returned to the pick pose between the two runs; the left did
not. So every `left_gripper` figure in these runs is a picture from
**somewhere else**, and must not be captioned "at the pick pose".

This is not a defect. It is the check working: a figure captioned "at the pick
pose" is worth nothing if nobody measured the pose, and nothing else in the
run would have noticed.

## What the good stages actually measured

`20260830_070529`, right arm at the pick pose:

| quantity | left_gripper | right_gripper | scene_rs | scene_hd |
| --- | --- | --- | --- | --- |
| depth returns | 115 256 px | 104 751 px | 85 061 px | no depth sensor |
| median range | 2.214 m | 1.838 m | 3.836 m | - |
| plane residual RMS | 1.408 mm | 1.597 mm | (band bug) | - |
| surface extent | 0.579 m | 0.431 m | (band bug) | - |
| surface distance | 0.277 m | 0.290 m | (band bug) | - |
| FastSAM regions | 30 | 44 | 80 | 114 |
| objects posed in 3-D | 6 | 12 | (band bug) | no intrinsics |
| nearest object | 0.536 m | 0.338 m | - | - |
| people found | 0 | 0 | **1** | **1** |

A 1.4-1.6 mm plane RMS on a real table at 0.28 m is the number the pick path
depends on, and it is close to the 0.54 mm this repository recorded for the
same sensor.

## What is still limited, and is not a bug

* **`scene_hd` has no intrinsics.** It has never been calibrated in this
  repository, so 6 stages refuse rather than assume `cx = w/2`. Run
  `scripts/calibrate_scene_camera.py` and they all start working.
* **YOLO-World found nothing on any camera.** It was asked for
  `['green cube']` -- that is the whole class list it was given, so it is not
  looking for people or furniture. Not a fault; change `--prompt`.
* **No AprilTag in any frame.** A tag either decodes or it does not.
* **The grasp planner refused on both wrist cameras** -- objects measured 142
  mm and 113 mm across their narrowest axis against a 75 mm usable jaw. That
  is an answer about the scene, not a failure of the planner.
* **The robot is not detected in any image, and no stage claims it is.**
  Nothing in this repository finds an arm in a picture; the robot's pose comes
  from its encoders through the URDF, and drawing it into a camera image needs
  an extrinsic measured at ~8.5 deg wrong on the wrist.

## Reproducing the figures without the rig

    .venv_vision/bin/python scripts/cv_pickpose_visuals.py --replay recordings/vision_thesis/<run>

The raw colour, depth (`.npy`, metres) and intrinsics are kept for exactly
this, so a figure can be redrawn or a constant changed without another lab
session.
