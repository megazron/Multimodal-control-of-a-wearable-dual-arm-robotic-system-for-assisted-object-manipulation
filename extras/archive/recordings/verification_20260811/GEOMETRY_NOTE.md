# SUPERSEDED GEOMETRY — archived 2026-08-11

These 15 clips (3 tasks × 5 modes) are **evidence of a geometry that is now
known to be wrong**. They are kept because they are a record of what was run,
not because anything in them can still be cited.

## Why they are superseded — two independent reasons

### 1. Task B's carry band was INSIDE THE BENCH

Every T2/Task-B clip here was recorded against the declared band
**z 1.10 – 1.30**. That band had never been checked with collision geometry
present: `verify_abc_scenarios.py` verified the study tasks in **free space**
and applied the bench only to the clip tasks, so it reported 0 failures for a
path whose first grips are inside the slab.

Settled three ways on 2026-08-11, bench in the planning scene:

| | |
| --- | --- |
| **arithmetic** (no solver) | two declared grip poses — S2's start at (±0.25, 0.35, 1.10) — lie **inside** the slab, which occupies z 1.06–1.10, y 0.245–0.63, \|x\| ≤ 0.85 |
| **per waypoint**, N=10, both arm assignments | S1 4/4 fail, S2 9/11, S3 8/10 = **21 failures** |
| **clear band** | feasible only from z = 1.28 (y = 0.35) and z = 1.30 (y = 0.38) |

The band is now **z 1.32 – 1.40**, held 20 mm inside the lowest clear z at
each y. So the carry these clips show is a carry through the bench.

### 2. T3's circuit box has moved 190 mm outboard

The box sat at x = −0.35. T3 requires it to be **held** — grasped and kept
still — which no previous scene ever did: `task_b()` places a part *on top of*
it and `task_c()` probes its top. Asked properly, a pinned wrist cannot grasp
it there at all; a sweep of 156 candidate hold poses found 37 it can reach,
**all right-arm and all at \|x\| ≥ 0.51**. The box is now at
**(−0.540, 0.170)**.

## What is also wrong with them, from earlier

They predate nothing else of substance — the grasp fixes and the 500 mm span
are in them — but note that the whole **A/B/C task set** they record has since
been superseded by the MSc four (T0/T1/T2/T3), and that the same free-space
bug was hiding failures in **Task A** (1/3 and 0/3 targets, 59 transit
failures; its targets at z = 1.05 are under the slab) and **Task C** (5/26 and
0/26 shell directions; its amplitude shell reaches z = 1.07).

## What they may still be used for

* As a record that the sweep, the isolation rule, the caption burner and the
  clip verifier all worked end to end.
* As a before/after pair against the re-recorded clips, **if and only if** the
  geometry difference is stated alongside.

## What they may NOT be used for

* Any statement about whether a task is feasible, reachable or collision-free.
* Any measured placement figure. The 0 mm placement results in these clips
  were achieved against furniture that the study tasks were not being checked
  against.
* Any comparison with the re-recorded clips that does not name the band
  change.

Replaced by the sweep recorded against `recordings/baselines/msc_verification.json`
(4234 IK calls, 0 failures, N=10, bench in scene, both arm assignments).
