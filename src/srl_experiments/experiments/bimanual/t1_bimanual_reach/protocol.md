# T1 — BIMANUAL REACH

**Both arms to separate targets simultaneously. Fitts, done bimanually.**

Measures the **attention bottleneck**: throughput with one arm versus two.
E1 already characterises single-arm Fitts throughput on this rig; T1 is the
bimanual comparison, and the difference between them is the cost of dividing
attention across two limbs.

**Feasible with position-only control.** Reaching only, no grasping, no
orientation. Runs on the current rig.

## Targets

No object. Targets are 40 mm and 20 mm circles marked on two boards, one in
front of each arm, at three amplitudes each — the same index-of-difficulty
range as E1 so the two are directly comparable.

```
   ID = log2(2A / W)      A = amplitude,  W = target width

   W = 40 mm,  A = 100 / 200 / 300 mm   ->  ID = 2.32 / 3.32 / 3.91
   W = 20 mm,  A = 100 / 200 / 300 mm   ->  ID = 3.32 / 4.32 / 4.91
```

## Fixed layout — NO APRILTAGS

Two boards, each 400 × 300 mm, at:

| board | x | y | z |
| --- | --- | --- | --- |
| left board centre | −0.26 | 0.30 | 0.980 |
| right board centre | +0.26 | 0.30 | 0.980 |

Target centres are at the board centre ± the amplitude along **y** (fore/aft),
deliberately avoiding the unresolved lateral axis.

## Trial structure

One trial = one simultaneous double reach. Both arms are given a target at the
same instant; the trial ends when BOTH have dwelled 400 ms inside their target.

- 6 conditions per arm (2 widths × 3 amplitudes), 5 repeats = 30 trials
- **Plus a single-arm baseline block**: the identical targets, one arm at a
  time. Without it there is no bottleneck to measure.

**Conditions:** DIRECT / ASSISTED (autonomy drives one arm to its target, the
operator drives the other) / SHARED (autonomy assists both final approaches).

## Metrics

- `mt_s` per arm — movement time to dwell
- **`throughput_bits_s` = ID / MT**, per arm, and summed across arms
- **`bottleneck_ratio` = bimanual summed throughput / single-arm throughput.**
  1.0 means two arms are free; 0.5 means the operator is strictly serialising.
- `stagger_s` — |t_arrive_left − t_arrive_right|. A large stagger IS
  serialisation, visible directly.
- `overshoot_mm`, `path_ratio` per arm

## Success detection — TF only

A reach succeeds when the EE is within W/2 of the target centre in the board
plane and dwells 400 ms. No gripper, no object, so no presence check is needed
— but a **coarse sanity check** applies: if an EE never leaves its start pose
by more than 20 mm, the arm was not driven and the trial is INVALID
(`arm_not_driven`) rather than scored as an infinitely slow reach.

## Analysis

`analyse_t1.py`. Fits MT = a + b·ID per arm per condition, reports throughput,
and the bimanual/single-arm ratio with a paired test. Reports stagger
distribution — if stagger is large and throughput ratio near 0.5, the operator
is time-slicing rather than truly parallelising, and that is the headline
finding.
