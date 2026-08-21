# The vision and grasp stack, as built 2026-08-21

**Prompt in, grasp out.** `scripts/find_object.py "the green cube"` looks with
a camera, detects what you named, builds a parallel-jaw grasp and says whether
an arm can reach it. It moves nothing.

## What is actually working, and what it cost to find out

| | state |
| --- | --- |
| **the gripper camera** | **live for the first time in this project.** Kinova wrist module, `rtsp://<ip>/color`, 1280x720 H.264, read with ffmpeg |
| **the gripper DEPTH camera** | **live, and it is why `rtsp://<ip>/depth` "does not work".** The depth stream is NOT H.264 -- it is a GStreamer-payloaded RAW stream (`rtpgstdepay`), 480x270 `GRAY16_LE`, millimetres. ffmpeg and OpenCV will not open it. Measured: 95% valid pixels, 1.43-6.34 m |
| **intrinsics** | **read from the robot over the Kortex API**, not assumed. Colour `fx 1297.67 fy 1298.63 cx 620.91 cy 238.28`; depth `fx fy 342.21 cx 233.07 cy 132.48`; depth->colour baseline 27 mm. **The colour principal point is 122 px above the image centre** -- anyone assuming `cy = h/2` throws every ray by 122 px, which at 1 m is 94 mm, three times the grasp tolerance |
| **detection** | an ENSEMBLE, because the two halves fail at different things |
| **grasp** | geometric, no learned model. Minimum-width frame + antipodal jaw, GPG-style |

## Why the detector is an ensemble and not one model

Measured on one real frame from this rig:

* **YOLO-World** finds `person` at 0.725 and `laptop` at 0.739 and **cannot
  find the target green cube at all** -- not at conf 0.02, not upscaled 4x.
  The cube is ~19 px and very dark (HSV value 26-63).
* The **colour+depth** backend finds that cube every time and cannot name a
  laptop.

Neither is "the good one". `backend="both"` runs the model first and falls
through to colour when it returns nothing, and **every detection reports which
backend produced it** -- a detection that does not say how it was made invites
the reader to assume the open-vocabulary model was running when it was not.

YOLO-World was chosen over GroundingDINO and OWLv2 for one measured reason:
this machine has an RTX A500 with **4 GB**. YOLO-World is ~60M parameters
against GroundingDINO's 218M and OWLv2's 428M. The bigger models score higher
and do not fit.

## Range is a discriminator that has nothing to do with colour

This room contains teal robots whose **shadowed hue is 76**; the target green
cube's is **74**. Two apart, inside noise. Colour-only filters picked the
robots three separate times, including one that passed a "tight" filter on
hue AND saturation. What separates them is that the robots sit at **2.8 m**
and anything on the rig is under **1.6 m**, so the range gate is applied at
detection time rather than left for the caller to remember.

## The defect that would have driven the jaw through the object

A depth camera sees the **front surface only** -- the cloud is a shell, not a
solid, so it has almost no thickness along the line of sight. An
unconstrained minimum-width search picks exactly that direction: measured on
a constructed 50 mm cube it reported **0.0 mm wide**. Acting on it would
command the jaw to close along the line of sight, squeezing through a surface
whose far side was never observed.

With one depth view the only honest grasp closes in the plane **perpendicular
to the view axis**, where both contact points lie on measured surface. Pinned
by `test_a_SHELL_from_one_depth_view_is_not_gripped_along_the_line_of_sight`.

A second, separate defect: **PCA alone is wrong for a cube, and a cube is the
target.** When extents are equal the covariance is degenerate and the
eigensolver returns an arbitrary orientation, so the "short axis" came back
along a face DIAGONAL -- 43.0 mm for a 40 mm cube. Three millimetres is
survivable; the direction is not, because a jaw told to close on a diagonal
approaches a corner and the cube rolls out. Fixed with rotating calipers.

## Reachability, and an instrument failure worth remembering

The reachability check is **multi-start**. An earlier version used a single
seed at home plus a term pulling solutions back toward home -- a local
optimiser told to stay put. It reported "unreachable" for points the operator
reached by hand. Every "cannot reach" it produced meant "I did not look".

With 41 seeds the answer held for cross-side targets: those are genuine
kinematic limits. **Home already sits 771 mm out of a 902 mm arm**, so most
directions run out of arm rather than hitting a rule.

## Layout

| file | holds |
| --- | --- |
| `srl_perception/rgbd_grasp.py` | pure geometry: deproject, cloud, min-width frame, grasp, pregrasp. No ROS, no model, fully unit-tested |
| `srl_perception/prompt_detector.py` | prompt -> detections. YOLO-World + colour/depth ensemble |
| `srl_perception/srl_cameras.py` | the two cameras, with MJPG and GStreamer-depth baked in |
| `srl_perception/grasp_pipeline.py` | detect -> depth -> grasp -> reach, each stage able to refuse BY NAME |
| `scripts/find_object.py` | the CLI |
| GUI **Vision** panel | the same thing from the RUN tab, in every mode |

Open-vocabulary detection needs `.venv_vision`; without it the colour backend
still works and still says so.
