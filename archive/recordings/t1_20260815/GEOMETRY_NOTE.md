# Why these T1 and T1S2 clips are superseded — 2026-08-15

Ten clip directories, five modes, T1 and T1S2. They were recorded against a
layout that has since changed in four ways. **No number from any of them may
be quoted for T1 or stage 2.** T0, T2 and T3 are unaffected and stay in
`recordings/verification/`.

## What changed

1. **The table is white.** It was oak. Cosmetic on its own, but every frame
   differs.

2. **Stage 1 moved from the RIGHT arm to the LEFT, and the cubes with it.**
   Cubes were at x = −0.320…−0.500, y = 0.190; they are now at
   x = +0.560…+0.740, y = 0.120.

3. **The coloured planes moved.** From (−0.340, 0.300) and (−0.470, 0.300) to
   (+0.450, 0.240) and (+0.610, 0.240).

4. **Stage 2 randomises the SIDE of each cube, not just its position**, and
   the trial's seed now actually reaches the layout. In these clips it did
   not: `run_abc` wrote `--seed` into the manifest and built the layout with
   no seed at all, so every stage-2 clip shows the SAME four cubes under a
   different seed number in its own metadata.

## And the reason the layout changed, which is the part worth keeping

The right-arm layout in these clips was verified by every instrument this
project had, at **zero IK failures**. Measured geometrically against the mount
guard's own capsule model of the wearer, it spends **70 of its 143 waypoints
inside the 150 mm wearer clearance floor**, worst case 58.7 mm from the
wearer's upper arm — and the idle arm's park pose sits at 30.6 mm for the
whole clip.

Nothing had ever checked. Every workspace and layout number in this repository
came from `/compute_ik` with `avoid_collisions`, and the SRDF permanently
excludes torso/harness/backpack against each arm's base, shoulder and
half_arm_1 — the pairs a shoulder-mounted arm actually threatens — so MoveIt
returns `valid` for poses with the tube inside the person.

See `recordings/baselines/t1_paths_BEFORE.json` for the measurement on this
layout, `docs/TASK_SPEC.md` section 2A for what replaced it.
