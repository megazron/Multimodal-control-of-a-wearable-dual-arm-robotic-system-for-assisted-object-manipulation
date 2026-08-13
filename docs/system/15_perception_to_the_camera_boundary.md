# The perception pipeline, and exactly where it stops

Written 2026-08-13 closing U-1, U-2 and U-3 of `docs/TASK_SPEC.md`.

The pipeline now runs end to end **in simulation, to the camera boundary**.
This file says what that establishes, and — the part that matters — what it
does not, so nobody quotes a number from it that it cannot support.

## What was wrong, in one line each

| | was | now |
| --- | --- | --- |
| U-1 | no camera publisher in sim, so image → detector → tracker → fingerprint had never run **at all**; every node was tested alone against a fixture and not one of the joins was | `mock_rgbd_camera` is wired into `perception.launch.py` behind `mock_camera:=true`, and `scripts/verify_detection_path.py` runs the whole chain |
| U-2 | an identity quaternion means "nobody measured the orientation" and every consumer read it as "zero degrees", so an object at 30 deg got a square grasp and no check anywhere could tell that from a square object | identity is `None` throughout; the fallback detector publishes the angle it already measured; the fingerprint carries yaw as a first-class averaged field; `grasp_generator` reports `yaw_source` |
| U-3 | `work_surface.set_measured()` had **no producer** — the fault injector was its only caller — so the table height was declared and never measured | `work_surface_node` measures it from depth and publishes it; `work_surface.subscribe()` carries it across the process boundary |

## What was actually run, and what came out

Live, against the stack, from the scan pose:

```
scripts/verify_work_surface_from_depth.py       4 of 4 checks pass
    a height is published at all                 z = 0.9503 m
    it matches the height the mock RENDERS       +0.3 mm against a 10 mm tolerance
    work_surface.check() sees it                 source = measured
    CONTROL: a blind region REFUSES              names the scan pose

scripts/verify_detection_path.py                4 of 4 checks pass
    camera -> detector                           3 raw detections
    detector -> tracker                          2 tracked objects
    yaw at the consumer                          2 measured, 0 unknown
    detector diagnostic agrees with the wire     yes
```

Offline, against constructed depth frames whose true answer was written down
first: `src/srl_perception/test/test_surface_from_depth.py`, 9 checks
including three refusals; `src/srl_perception/test/test_fingerprint_yaw.py`,
9 checks.

## WHAT THIS ESTABLISHES

The geometry is **constructed**, so these are arithmetic against answers known
in advance and they count:

* the topics exist with the driver's own encodings, and the detectors match
  them — `16UC1` in millimetres, which is the branch that will actually run;
* `CameraInfo` intrinsics reach the consumer and are used;
* deprojection lands on the true world position through the real TF chain,
  with a control that must fail (the transposed rotation) and does;
* the surface estimator resolves the ±20 mm error the fault table injects,
  is not dragged by objects standing on the table, and refuses rather than
  answering when the camera is not looking at the region;
* yaw survives detector → tracker → consumer, and "not measured" survives as
  a third state rather than collapsing to zero.

## WHAT STILL NEEDS REAL CAMERAS

**Nothing below can be moved by any amount of further work on the mock.**
This is the list the spec asked for.

| | why the mock cannot reach it |
| --- | --- |
| **detection rate at working distance** | the one number the whole "scene understanding without fiducials" claim rests on. Flat-shaded primitives on a flat background score 0–4% for a learned detector that scores 0.89–0.91 on a real photograph, so the mock's near-perfect result is an artefact of rendering, not evidence. The ≥95% gate is neither passed nor failed. |
| **detection accuracy and the bounding box** | same reason. Pose accuracy from fitting known dimensions to cropped depth is separately measured (1.2–8.0 mm) and is *not* the detector's job; the box is. |
| **the real intrinsics, distortion and the colour↔depth extrinsic** | the mock uses a plumb-bob model with zero distortion because that is what makes the arithmetic checkable. The real values come off the device. |
| **depth holes, multipath and specular dropout** | the mock's depth is a rendered plane and never returns zero where a real sensor would. The estimator's `0 is no return` branch is written but has never seen a real hole. |
| **whether a real table surface returns depth at all** | a matte oak top usually does; a gloss bench often does not. Unknown for this table. |
| **exposure, motion blur, rolling shutter** | the arm moves and the camera is on it. None of this exists in a rendered frame. |
| **whether the Kinova vision module streams over this machine's network path** | untested. `eth0`/`eth1` are down and this host is off the lab network. |
| **the detector topic names the real driver publishes** | the audit already records this as WILL FAIL as configured: detector topics default to `/camera/...` and the Kinova driver publishes elsewhere, and nothing asserts the subscribed topic has a publisher. A wrong name is a silent empty scene. |

## The one convention this loses, stated rather than discovered later

An object **measured at exactly 0.000000 degrees** encodes as an identity
quaternion and is read back as unmeasured. `vision_msgs` has no field for
"not observed", so the sentinel has to be a pose, and identity is the only
honest choice available.

The cost is nil in practice: the objects are 90-degree symmetric, so a grasp
planned for "unknown, assume square" and one planned for "measured, zero
degrees" are the same grasp. Half a degree encodes as `z = 0.0044`, four
thousand times the tolerance. Where the distinction has to survive exactly,
pass `yaw_deg=` to `Observation()`; it is believed over the quaternion.

## How to run it

```
ros2 launch srl_perception perception.launch.py \
    mock_camera:=true fallback:=true work_surface:=true task:=t1

python3 scripts/verify_detection_path.py
python3 scripts/verify_work_surface_from_depth.py
python3 -m pytest -q src/srl_perception/test
```

**Drive to the scan pose first.** From the home pose the wrist cameras do not
frame the work surface at all, so a correctly working camera returns an empty
region — and an empty region is indistinguishable from a dead camera unless
you already know which pose you are in.
