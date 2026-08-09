# Verification clips: the CURRENT five-task set

> ## READ THIS FIRST: TASK 2 IS NOT OPERATOR-COMMANDABLE
>
> **The f2 clips do not show teleoperated grasping, because teleoperated
> grasping is not achievable on this platform.** A top-down grasp needs
> **169.7 deg** (left arm) and **164.6 deg** (right) of wrist rotation from the
> orientation `orientation_mode: fixed` pins the commanded wrist to, and
> nothing in the master measures the wrist to command it.
>
> Those clips work because the recorder calls `/compute_ik` **directly** with
> the grasp quaternion, bypassing the teleoperation orientation lock. They
> demonstrate that the ROBOT can execute an aligned grasp. They do **not**
> demonstrate that an OPERATOR can command one.
>
> The caveat is also burned into the overlay of every f2 clip in red, so a
> viewer who never opens this file still cannot draw the wrong conclusion.
> f2 therefore has no DIRECT and no VR condition: it is recorded in the shared
> condition only. See `docs/research/09_task2_grasping_finding.md`.


Recorded against the five-task spec in
`src/srl_experiments/experiments/final5/tasks.py` (500 mm tray span, 540 mm
sling), with the single-owner gripper fix. These supersede the 87 clips under
`t2..t9`, which use the retired nine-task geometry.

Seven angles per clip: `rviz_front` (the only one carrying the overlay),
`rviz_back`, `rviz_left`, `rviz_right`, `rviz_iso`, `rviz_top`,
`rviz_gripper` (tight on the fingers), plus `rviz_quad` as a review tile.

| task | scenario | condition | dur | grip | grasp | notes |
| --- | --- | --- | --- | --- | --- | --- |
| f1 | S1_left_only | assisted | 7.9 s | 0.05..0.05 | n/a |  |
| f1 | S1_left_only | direct | 8.2 s | 0.05..0.05 | n/a |  |
| f1 | S1_left_only | shared | 8.2 s | 0.05..0.05 | n/a |  |
| f1 | S2_right_only | assisted | 7.8 s | 0.05..0.05 | n/a |  |
| f1 | S2_right_only | direct | 7.9 s | 0.05..0.05 | n/a |  |
| f1 | S2_right_only | shared | 7.9 s | 0.05..0.05 | n/a |  |
| f1 | S3_both | assisted | 8.1 s | 0.05..0.05 | n/a |  |
| f1 | S3_both | direct | 8.0 s | 0.05..0.05 | n/a |  |
| f1 | S3_both | shared | 8.0 s | 0.05..0.05 | n/a |  |
| f2 | S1_near_pick | shared | 14.6 s | 0.05..0.42 | planned | **DIRECT-IK, not operator-commandable**, blocks=1 |
| f2 | S2_far_pick | shared | 14.4 s | 0.05..0.42 | planned | **DIRECT-IK, not operator-commandable**, blocks=1 |
| f3 | S1_short_lift | assisted | 6.3 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S1_short_lift | direct | 6.2 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S1_short_lift | shared | 6.1 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S2_full_lift | assisted | 8.3 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S2_full_lift | direct | 8.4 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S2_full_lift | shared | 8.4 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S3_detour | assisted | 10.9 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S3_detour | direct | 10.5 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f3 | S3_detour | shared | 10.5 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S1_short_lift | assisted | 6.2 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S1_short_lift | direct | 6.2 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S1_short_lift | shared | 6.4 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S2_full_lift | assisted | 8.6 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S2_full_lift | direct | 8.3 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S2_full_lift | shared | 8.4 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S3_detour | assisted | 10.9 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S3_detour | direct | 10.8 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f4 | S3_detour | shared | 10.6 s | 0.05..0.61 | pre-grasp standoff not solvable |  |
| f5 | S1_both_slow | assisted | 10.7 s | 0.05..0.05 | n/a |  |
| f5 | S1_both_slow | direct | 10.4 s | 0.05..0.05 | n/a |  |
| f5 | S1_both_slow | shared | 10.5 s | 0.05..0.05 | n/a |  |
| f5 | S2_one_fast | assisted | 10.7 s | 0.05..0.05 | n/a |  |
| f5 | S2_one_fast | direct | 11.0 s | 0.05..0.05 | n/a |  |
| f5 | S2_one_fast | shared | 10.6 s | 0.05..0.05 | n/a |  |
| f5 | S3_both_fast | assisted | 10.5 s | 0.05..0.05 | n/a |  |
| f5 | S3_both_fast | direct | 10.5 s | 0.05..0.05 | n/a |  |
| f5 | S3_both_fast | shared | 10.7 s | 0.05..0.05 | n/a |  |

## What the automatic check covers, and what it does not

`scripts/verify_rviz_clips.py`: **39 of 39 clips pass**. It confirms the task
object is on screen and that the arm moves, from PIXELS.

It does **not** confirm that a grasp is physically correct, that the motion is
what an operator would command, or that any number in the overlay is right.
Those come from the measurements in `docs/system/09_results_audit.md`.

**The f3 and f4 clips show a carry, not a grasp.** Their overlay reads
`GRASP REFUSED: pre-grasp standoff not solvable`, which is correct: the tray
edge sits at |x| = 0.25 m and top-down grasping is infeasible below
|x| = 0.30 m (measured, 0/9 against 9/9). The refusal is on screen rather
than hidden.
