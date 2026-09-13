# SUPERSEDED 2026-08-15 — recorded against geometry that has since changed

These are the T0, T2 and T3 clips from every mode, moved here under the
never-delete rule before the 2026-08-15 re-record. **Do not cite them for any
layout, workspace or clearance figure.** They are kept because they are the
only footage of the scene as it was, and because a report that cites evidence
which is not on disk is worse than one that cites nothing.

The demonstration routines D1–D3 under `06_full_autonomy` were NOT moved. They
are a different taskset, they were not re-recorded on 2026-08-15, and moving
them would have taken the only copy out of the live tree for a change that
does not affect what they show.

T1 and T1S2 are not here. They were deleted rather than archived on
2026-08-15: the ten clips recorded that day were invalidated by two fixes
found by looking at them, and a clip that is known wrong is not evidence of
anything.

## What changed after these were recorded

| | then | now |
| --- | --- | --- |
| the table | oak, half-width 0.90 | **WHITE**, three shades, half-width **1.05** |
| the workspace marking | the IK-reachable cells, \|x\| ≤ 0.70 | the **clearance-safe** cells, out to \|x\| = 1.00 |
| T1 stage 1 | right arm, cubes on the right | **left arm, cubes on the left** |
| T1 stage 2 place targets | one formula per arm — both of an arm's cubes went to ONE point | one destination per cube, minimum travel 0.150 m |
| the close dwell | 8 waypoints | **14** — 02_vr_teleop missed the last cube by 44.0 mm against a 30 mm gate at 8 |
| the front camera for T1 | aimed at x = −0.38, the old right-arm cube row | aimed at x = +0.54 |
| the wearer clearance floor | **never measured against anything** | measured geometrically, per waypoint, 150 mm |

## The one that matters most

Every workspace figure behind these clips came from `/compute_ik` with
`avoid_collisions`, and the SRDF permanently excludes the wearer pairs a
shoulder-mounted arm actually threatens — torso, harness and backpack against
each arm's base, shoulder and half_arm_1. So MoveIt returned `valid` for poses
with the tube inside the person, and nothing measured the distance.

Measured afterwards: the shipped right-arm T1 layout spent **70 of its 143
waypoints inside the 150 mm floor at zero IK failures**, and the idle arm's
park pose sat at 30.6 mm for whole clips. The marking drawn in these clips —
the region a participant is told to work in — contained 72 left and 68 right
cells inside that floor, the worst at −2.7 mm.

The full ledger of which numbers this invalidated and which stand is
`docs/system/clearance_gap_ledger.md`.
