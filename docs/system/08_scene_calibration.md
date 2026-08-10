# Scene calibration — the sim/real distance problem

**The question this answers:** the real object is not where the sim thinks it
is. How far out can it be before the grasp fails, and how do we find out where
it actually is before a session starts?

## What `scene_fingerprint_node` does today

`ros2 run srl_perception scene_fingerprint_node`

| | |
| --- | --- |
| consumes | `/perception/objects` (`Detection3DArray`) — it does **not** open a camera |
| publishes | `/scene/state`, `/scene/objects`, `/scene/markers` |
| services | `/scene/resweep`, `/scene/forget` |
| store | `config/scene_fingerprint.json` |
| match tolerance | `pos_tol` 15 mm, `rot_tol` 15°, gate 250 mm |

The logic is built and tested: it loads a stored fingerprint, accumulates
detections during a sweep window, compares, and either reports calibration
skipped or re-registers **only what moved**. When stored and observed agree it
republishes the **stored** pose, deliberately — two poses that agree within
tolerance are equally valid, but the stored one has been accumulated over more
views, and swapping to a fresh single-sweep estimate every start would inject a
random walk into a quantity the rest of the system treats as fixed. Drift
during running is **flagged, never acted on**.

It does not open a camera, by design: two processes on one camera is the same
fault class as two readers on one serial port.

### What it needs on real hardware, and does not have

1. **Nothing drives the arms.** `begin_sweep()` opens an accumulation window
   and logs. It does not command a single look pose. On real hardware the
   sweep has to move both wrist cameras through a set of poses that see the
   bench, and that motion does not exist. Today the "sweep" only accumulates
   whatever the cameras happen to already see.
2. **`pos_tol` = 15 mm is a placeholder.** The module's own
   `min_detectable_displacement()` says the floor is
   `max(pos_tol, 2 × pose_noise_sd)`, and the real detector's pose noise has
   never been measured on real images. Until it is, 15 mm is a guess that
   happens to sit between the AprilTag figure (0.8 mm σ, synthetic) and the
   colour/shape fallback (10 mm σ, synthetic).
3. **The look poses must be verified.** Every pose the sweep drives to needs
   the same N=10 full-path treatment as the task poses, with the bench in the
   planning scene. Otherwise the calibration sweep is the one motion in the
   session that was never checked.

## The sweep procedure

```
startup
  └─ load config/scene_fingerprint.json      (if absent: full registration)
  └─ drive each arm through the look poses   ← NOT IMPLEMENTED
  └─ accumulate /perception/objects into the sweep window
  └─ compare each observation against the store
       within 15 mm and 15°  → UNCHANGED, republish the STORED pose
       beyond that, inside the 250 mm gate → MOVED, re-register THIS object
       outside the gate      → APPEARED / VANISHED, re-register
  └─ publish /scene/state and /scene/objects
running
  └─ every detection drift-checked against the store; a misfit is FLAGGED
```

Re-registering only what moved is the point: a full recalibration costs the
sweep time on every start and injects fresh noise into objects that did not
move.

## What tolerance does the grasp survive?

Two independent limits, and the smaller wins.

**1. Capture — is the object still between the fingers when they close?**

Jaw separation was measured against knuckle angle from live TF:

| knuckle (rad) | measured tip separation | `grip_for` model | delta |
| --- | --- | --- | --- |
| 0.000 | 135.5 mm | 85.0 mm | +50.5 |
| 0.200 | 116.8 mm | 63.8 mm | +53.0 |
| 0.4235 | 93.3 mm | 40.0 mm | +53.3 |
| 0.800 | 50.7 mm | 0.0 mm | +50.7 |

**The model's slope is right and its offset is not the model's problem.** Over
the full range the measurement falls 84.8 mm against the model's 85.0 mm — a
0.2 mm disagreement in gradient. The constant ~52 mm offset is simply that
`finger_tip_link` sits about 26 mm outboard of each contact surface, so the
frames are not the pads. `grip_for()` is therefore sound, and the capture
half-window at full open is `(85 − width) / 2`:

| object | width | capture half-window |
| --- | --- | --- |
| task A block | 40 mm | **22.5 mm** |
| task B part | 45 mm | **20.0 mm** |
| task C multimeter | 50 mm | **17.5 mm** |

**2. IK — is the displaced grasp pose still reachable, furniture in?**
Measured at N=10, all six axis directions:

| pose | arm | ±10 mm | ±20 mm |
| --- | --- | --- | --- |
| A pick | left | all 6 ok | all 6 ok |
| B pick | right | all 6 ok | all 6 ok |
| C present | left | all 6 ok | all 6 ok |

### The answer

| real object error | verdict |
| --- | --- |
| **±10 mm** | **Grasp succeeds on all three objects.** 10 mm is inside every capture window (worst 17.5 mm) and IK is unaffected. |
| **±20 mm** | **Grasp succeeds on the 40 mm block only.** 20 mm is inside its 22.5 mm window; it sits exactly ON the 45 mm part's 20.0 mm limit, and **outside** the 50 mm multimeter's 17.5 mm window — that grasp fails, the fingers close beside the object. IK is fine at 20 mm; capture is what breaks. |

So the usable requirement is **±10 mm**, not ±20 mm, and it is set by the
widest object rather than by the arm. The 15 mm `pos_tol` in the fingerprint
store is therefore *looser than the grasp can survive on the multimeter* — an
object can be declared UNCHANGED at 15 mm of error and then not be grasped.
**`pos_tol` should be tightened to 10 mm, or the multimeter narrowed**, and
that decision needs the real detector's noise floor first, because a tolerance
below `2 × pose_noise_sd` cannot be separated from noise.

## Is the detector good enough? No — and it is still unmeasured

**Detection rate at working distance remains UNMEASURED, and under 95% blocks
any participant session.**

The synthetic harness cannot answer it. `scripts/measure_detection.py`
returned 0–4% on the task objects, and the instrument was checked before that
was believed: the same model scores 0.89–0.91 confidence on a real photograph
and 0/0 on flat-shaded primitives. The renderer is out of the detector's
distribution — the AprilTag mirrored-renderer bug in a new costume.

What that leaves:

- **AprilTag path**: 100% detection at 0.35 m in simulation, 0.74 mm mean
  position error. Simulation only.
- **Colour/shape fallback**: capped at confidence 0.45 so it can never outrank
  a tag, and it fails by being confidently wrong.
- **YOLO-World**: fits alongside Whisper in 4 GB (2012 MiB resident, ~23 ms per
  frame), and has never been scored on anything in this scene that was not
  rendered by us.

**Mode 6 is not participant-ready and this is the reason.** Either the objects
carry AprilTags — in which case the measured 0.74 mm at 0.35 m is comfortably
inside the ±10 mm requirement — or detection must be measured on real camera
frames before a session is scheduled.
