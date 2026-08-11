# A pinned-wrist master cannot choose grasp yaw, and VR can

**The gap appears in kinematics, before any participant is involved.**

**Run it:** `python3 scripts/measure_t1_cubes_t2_transfer.py --repeats 10`
→ `recordings/baselines/t1_cubes_t2_transfer.json`

---

## The result

Grasping a 40 mm cube, measured on identical poses in each arm's own measured
graspable strip, bench in the planning scene, N = 10 for the fixed policy:

| arm | fixed (pinned anchor) | cube, best of 4 yaws | cylinder, best of 8 | top-down, 4 yaws |
| --- | --- | --- | --- | --- |
| left | 12/12 — **100 %** | 12/12 | 12/12 | 12/12 |
| right | 5/12 — **42 %** | 11/12 — **92 %** | 11/12 — **92 %** | 12/12 — 100 % |

Mapped onto the control modes, whose only relevant difference is what they can
do with the wrist:

| mode | wrist | left | right |
| --- | --- | --- | --- |
| **MASTER_TELEOP** | pinned; `orientation_mode: fixed`, and nothing in the master measures the wrist | 100 % | **42 %** |
| **VR_TELEOP** | full 6-DOF from the controller; the operator chooses | 100 % | 92–100 % |
| MASTER_SHARED / VR_SHARED / FULL_AUTONOMY | autonomy supplies the wrist on approach | 100 % | 100 % |

## Why it is a capability result and not a layout problem

The measurement was made to decide a layout question, and the answer turned
out to be about the input device instead. Three things make that reading the
right one:

**1. It is not the object.** A cube admits four grasp yaws — its 4-fold
symmetry about the vertical; a cylinder admits all of them. If shape were the
binding factor those two would differ. They do not: **11/12 either way.** Four
yaws already recover everything eight do. This corrects the natural reading of
the earlier Gate 2 numbers: the 5/12-versus-11/12 gap was *fixed versus
yaw-free*, not *cube versus cylinder*, and the object was left as a cube
because changing it would have bought nothing measurable.

**2. It is not the arm.** Both arms carry the same Gen3 and the same 2F-85,
and the left arm scores 100 % under every policy including the pinned one.
What differs is where each arm's graspable strip sits — the left's at
|x| 0.30–0.39, the right's at |x| 0.54 — which follows from the asymmetric
home parking (`|v_R − M·v_L| = 1.3837 m`, provably independent of the mount).
The pinned wrist is simply a worse fit for the right arm's strip.

**3. It is not the solver.** Same solver, same poses, same repeats, same
bench. The only variable across the four columns is how much of the
orientation the mode may choose.

So the statement is:

> **On this platform a pinned-wrist master arm loses 58 % of the right arm's
> graspable poses that a 6-DOF input recovers, and it loses them purely
> because it cannot select the grasp yaw.**

## What it predicts, and what it does not

**Predicts:** VR_TELEOP should outperform MASTER_TELEOP on any right-arm
grasping task, and the effect should be *largest* where grasp orientation is
most constrained. That is a directional hypothesis derived from geometry
before data collection, which is the strongest form a pre-registered
prediction can take. It is also a mechanism, not just a direction: the
difference is yaw selection, so it should vanish on tasks with no grasp — and
T0 is exactly that task, which makes T0 the control for this effect rather
than only the baseline instrument.

**Does not predict** anything about completion time, workload or preference.
Kinematic feasibility is a ceiling, not a performance measure: an operator can
fail at a pose that is perfectly feasible. The 42 % is the fraction of poses
where the pinned mode has *no solution at all* — participants will do worse
than that, never better.

## Consequence for the study design, kept separate on purpose

T1 is run **left-arm only**, which removes the affected cell from the matrix.
That is a design decision recorded in `docs/research/12_...` and in the T1
spec; it is *not* the reason this finding exists, and this finding does not
depend on it. The 42 % stands whether or not any task is ever run on the right
arm under a pinned wrist.

The within-T1 arm contrast is not lost from the study: **T0 measures both
arms in every mode**, has no grasp, and is therefore the clean place to see
whether the input-device difference is specific to grasping or general to
pointing. If VR beats the master on T0 as well, the effect is not about yaw.

## What would change it

Repairing the master's wrist channels. `orientation_mode` is pinned because
left j7 is railed and right j3/j5/j7 are dead, so wrist rotation is
unobservable; with the pots repaired the master can express roughly 180° and
the mode could move to `anchored`. `orientation_mode: tilt` would **not** fix
it — tilt buys nothing over yaw-free here (right arm 26.7 % against 33.3 %)
and this `/compute_ik` setup ignores the `constraints` field entirely, which
is why tilt was never shipped.

Until then the 42 % is a property of the shipped system and should be reported
as one.
