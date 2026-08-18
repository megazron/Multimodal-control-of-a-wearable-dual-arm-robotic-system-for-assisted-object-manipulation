# The T1S2 clip archived on 2026-08-18, and what changed under it

This is the last recording of stage 2 as a SEPARATE TASK. It was filmed on the
2026-08-15 geometry: objects on the shared work plane 120 mm above the table,
a pad PAIR per side sized from `PLANE_SIZE_BY_ARM`, the wrist pinned at
`master_calibration.WORKSPACE_ORIENT`, and the cubes coloured by their place in
the draw.

None of that survives. Stage 2 now shares stage 1's geometry entirely -- same
table at 1.250 with its near edge at 0.430, the same two pads at +/-0.290
straddling the centreline, the same row, the same approach (elevation -10 deg,
heading -50 inboard), and the same builder. What the seed draws is which SIDE
each cube starts on and which measured column it stands in.

The colour follows the side, and that is a measurement rather than a
simplification: neither arm can cross the centreline -- 0 of 10 IK solutions at
every cross-side pad slot and every cross-side cube, on both arms, with the
wearer and the table in the scene -- so a cube can only be delivered to the pad
on its own side.

Eight seeds were walked over the composed path at N=10 before this was
replaced: 0 IK failures, 0 waypoints inside the 150 mm floor, every one clean
(`recordings/baselines/t1_stage2_paths.json`).

The clip is kept because it is the evidence for what stage 2 looked like while
the two stages had diverged -- which `test_t1_stages_agree` asserted at the
time, and now asserts the opposite of.
