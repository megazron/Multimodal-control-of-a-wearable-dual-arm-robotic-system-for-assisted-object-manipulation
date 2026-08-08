# E4 — DOF recovery

**Type:** categorical result. **Participants:** REQUIRED.

## Question
Are there objects that direct teleoperation **cannot** grasp and assist
**can**?

## Why the result is categorical
"37% faster" invites the reader to imagine a continuum. The claim this
experiment supports is different in kind: *impossible becomes possible*. That
is what a DOF deficit produces — not slower performance but an unreachable
region of the task space.

## Design
Position is held CONSTANT across scenarios; only the REQUIRED APPROACH
ORIENTATION varies (0°, 30°, 60°, 90°, 120°). Any difference between
scenarios is therefore attributable to orientation alone.

Conditions: `direct` vs `shared`. 4 repeats × 5 scenarios × 2 conditions.

## THE CONTROL THAT ANSWERS THE REVIEWER

The objection is: *"you added autonomy to compensate for a broken input
device."*

The answer is that the deficit is **structural**, not incidental:

- *Incidental* faults are this rig's specific dead potentiometers. They are
  repairable and they are not the point.
- *Structural* limits are properties of any wearable master: it must be light,
  wearable and leave the hands free, so it cannot carry a grounded 6-DOF
  force-feedback measurement chain. Gravity gives roll and pitch but **never
  yaw** — a rank deficiency, not a calibration problem. Integrating a gyro
  drifts (measured −5.0 and +44.0 °/min). Double-integrating acceleration
  diverges as ½bt².

To make that empirical rather than rhetorical, this experiment:

1. **Masks orientation IN SOFTWARE** on otherwise-healthy channels, so the
   deficit under test is the structural one;
2. **Records which channels were live in every trial**;
3. **Excludes any trial with >2% dropout on a required channel** — a threshold
   set from the MEASURED separation between healthy channels (0%) and the
   faulty right j4 (12.9%).

If the effect survives with healthy channels and a software-imposed
orientation deficit, the objection is answered by the data rather than by the
discussion section.

## Pre-registered hypotheses
- **H4.1** Grasp success in `direct` falls to ≈0 for `side_facing` and
  `inverted`, and stays high in `shared`. Test: **McNemar** on the paired
  binary outcome per scenario; Holm across the five scenarios.
- **H4.2** For `upright_easy` there is NO difference between conditions. This
  is the control that shows the effect is orientation and not "assist is
  simply better at everything".
- **H4.3** The crossover is monotone in tilt angle.
- **H4.4** *Falsifiable prediction that tests the whole argument:* the size of
  the assist benefit does NOT depend on which incidental channels were live
  (given the 2% exclusion). If it does, the deficit being measured is the
  broken rig after all, and the structural claim fails.

## Stop / invalidate
45 s timeout. INVALID on dropout >2% of required channels, or e-stop.
