# 15. Eye-in-hand observation on a shoulder-mounted arm

**Measured 2026-08-16.** Two results about where a wrist camera can and cannot
see the work on this rig. Both were produced by `scripts/measure_vision_options.py`
against the current layout, with the FK checked against live TF (0.06 mm) and
the projection checked against a hand-computed pixel before either was run.
Raw output: `recordings/baselines/vision_options.json`.

They are recorded here as **results**, not as implementation notes, because
each closes or opens a design option that is otherwise easy to assume either
way.

---

## Result 1. The approach never has the object in frame. A separate observe pose is the only place detection can occur.

The wrist camera's optical axis **is** the tool axis. `camera_color_frame`
sits at rpy (pi, pi, 0) from `end_effector_link`, which is diag(-1, -1, 1), so
camera +z = EE +z to **0.00 deg**. The camera looks exactly where the gripper
points, by construction. That is the fact people reason from, and on its own it
suggests detection during the reach should be easy.

**It is not, and the reason is the PATH, not the camera.** `run_abc.send()`
pins the wrist at the anchor for every waypoint, so orientation is constant
through a reach while the path is a **vertical descent** from a 0.10 m
standoff. The anchor points 30.8 deg (left) / 22.1 (right) **above** horizontal,
so the object sits well off the optical axis for the whole descent.

Every waypoint of T1's own pick leg, both arms, all four cubes — **32 poses**:

| waypoint | off-axis, left / right | range, left / right | in frame | in depth range |
| --- | --- | --- | --- | --- |
| standoff | 65.8 / 62.5 deg | 0.155 / 0.167 m | no | no |
| mid-descent | 47.9 / 46.8 deg | 0.133 / 0.140 m | no | no |
| **grasp** | **26.1 / 26.1 deg** | **0.128 m** | no | no |
| lift | 59.3 / 56.9 deg | 0.145 / 0.155 m | no | no |

**0 of 32 in frame at any range. 0 of 32 in depth range at any angle.**

The two requirements move in opposite directions along the path, which is why
no waypoint satisfies both. Descending **improves** the angle (65.8 -> 26.1
deg) and **worsens** the range (0.155 -> 0.128 m), and the whole path lies
inside 0.167 m while the depth module is blind below 0.25 m. Even the best
waypoint — the grasp, at 26.1 deg — is still outside the frame, because 26.1
deg clears the horizontal half-FOV of 27.5 deg but not the **vertical** half of
21.3 deg.

**Consequence, stated plainly so it is not simplified away later:** a separate
OBSERVE pose is not a convenience or an optimisation. It is the only place on
this rig where detection can happen at all. Removing it does not make the
pipeline simpler; it makes it blind.

The observe pose is verified usable on both arms —
`/compute_ik` 10 of 10, wearer clearance 0.1610 m — but only via a **via
point**: a straight joint-space move from home dips to 0.1199 m (left) and
0.1264 (right) against a 0.150 m floor, because a joint interpolation is
straight in JOINT space and the elbow sweeps an arc through the wearer. See
`scripts/verify_observe_pose.py`.

---

## Result 2. Cross-arm observation is OPEN, not closed.

**This corrects a claim that went round in the opposite direction.** It was
reported that "two arms mounted 0.90 m apart facing outward cannot see each
other's work, 0 of 47". Measured, neither the geometry nor the conclusion
holds:

* the arms are **0.6946 m** apart at the shoulders and **0.8025 m** at the
  hands in the home pose, not 0.90 m;
* both directions **find** an observing pose.

| watcher | range to the other arm's objects | wearer clearance | straight transit from home | joint travel |
| --- | --- | --- | --- | --- |
| right watches LEFT's work | 0.62 - 0.85 m | 0.1523 m | **-0.1176 m — needs a via** | 621 deg |
| left watches RIGHT's work | 0.58 - 0.86 m | 0.1610 m | **0.1596 m — clean** | 600 deg |

Note the asymmetry: left-watching-right needs no via and right-watching-left
does. That is the mounts again — reflecting the left mount through x = 0 leaves
it 168.00 deg of roll from the right — and it is the same structural asymmetry
that stops the two arms holding mirrored postures.

**The real limitation is availability, not geometry.** Cross-arm observation
only works while an arm is idle:

* **T1 stage 1** — the left arm works and the right is idle, so the right arm
  can watch. Usable.
* **T1 stage 2** — both arms work simultaneously. There is no idle camera, so
  cross-arm observation is unavailable *for that task*, not for the rig.

So the option is open and its cost is 600-621 deg of joint travel on the
watching arm, which is a real cost to weigh against the 48.6 s that the
working arm's own observe move takes.

---

## What is actually built

`vision_grasp.observe_and_detect()` and `run_abc --vision`. The working arm
looks, detects, returns home, and the path is then built from the SEEN
position and the SEEN colour. Measured on T1, left arm:

| | |
| --- | --- |
| classification | **4 of 4 correct**, 0 wrong colour, 0 missed |
| localisation | worst **3.6 mm** |
| added time | **48.6 s per look**, **12.2 s per pick** (one look serves four cubes) |
| **vision drives the grasp** | a cube mislabelled in the scene definition is placed by its **detected** colour, 230 mm from where the declaration would have put it |

Three things had to be right for that, and only the first is obvious:

1. **Size at range.** The pads are 210 x 130 mm in the cubes' own colours; a
   colour-only detector locks onto them. A blob is rejected unless it is the
   size that object would be at the range it appears to be at.
2. **A depth step.** The cubes touch the same-coloured pads in projection, so
   connectivity merges them. **Colour alone cannot separate them** — this
   requires the depth image, which `colour_shape_detector` does not currently
   subscribe to. On the real rig it needs depth, or pads in non-cube colours.
3. **Plan the grasp AFTER returning home.** `/compute_ik` seeds from the live
   joint state. The same 24 waypoints solve with 0 failures either way, at
   **0.0017 m** of wearer clearance when planned from the observe pose and
   **0.1610 m** when planned from home. The obvious sequencing is the unsafe
   one and no IK-based check catches it.

---

## What still needs real cameras

Everything appearance-dependent. The classification above is a statement about
RENDERED colour: flat-shaded primitives under a headlight are not a photograph,
and the same pipeline has scored 0-4% on rendered primitives against 0.89-0.91
on real images for a learned detector. The HSV thresholds will need
recalibrating before they see a photograph — the tightest channel measured on
the render is **hue, 17 counts of margin** on both colours, which is not much
to lose to white balance.

What DOES transfer is the plumbing: the frame chain, the intrinsics, the
deprojection arithmetic and a consumer that reads a detection rather than a
file. Those are constructed geometry with a known answer.
