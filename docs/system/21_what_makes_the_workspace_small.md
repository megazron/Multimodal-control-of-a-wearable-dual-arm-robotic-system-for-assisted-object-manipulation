# What makes the workspace small — measured, per direction, per policy

Measured 2026-08-22, offline, no move_group, no running stack.
`scripts/measure_orientation_cost.py` is the instrument;
`recordings/baselines/orientation_cost*.json` are the results.

## THE ANSWER IN ONE TABLE

Mean reach, walking outward in 25 mm steps until the first refusal, with the
0.15 m wearer floor enforced under **every** policy:

| policy | from the arm's HOME EE (0.744, 0.339, 1.180) | from the WORK POINT (0.400, 0.175, 1.120) |
| --- | --- | --- |
| **pinned** — today | **0.460 m**, 6 of 28 walks wearer-bound | **0.065 m**, **11 of 12 walks wearer-bound** |
| spin — roll free, axis fixed | 0.463 m (+3 mm) | 0.065 m (**+0 mm**) |
| cone 15 deg | 0.492 m (+32 mm) | 0.319 m (+254 mm) |
| cone 45 deg | 0.530 m (+71 mm) | 0.440 m (+375 mm) |
| free — position only | 0.549 m (+89 mm) | 0.450 m (+385 mm) |

**The pinned wrist costs 89 mm where the arm has room and a factor of seven
where the work is.**

## WHY THE TWO COLUMNS DISAGREE, AND WHY BOTH ARE TRUE

They walk from different points, and a reach is meaningless without the point
it was measured from. This repository already had two baselines that
disagreed for exactly this reason and nobody had reconciled them:

* `recordings/baselines/arm_reach_extents.json` walks from the home EE, which
  is 0.744 m outboard, where the arm has room to choose an elbow.
* `recordings/baselines/what_binds.json` walks from (0.4, 0.175, 1.12) —
  `BENCH_TOP + 0.02`, the plane T1 runs on — and reported four of twelve
  directions bound by ORIENTATION with the detail string *"reachable only
  with the wrist unpinned"*. It named the mechanism and never gave a size.

`measure_orientation_cost.py --start what_binds` reproduces the second and
sizes it. Both baselines stand.

## THE MECHANISM, WHICH IS NOT "THE WEARER IS IN THE WAY"

At the work point, under the pin, **eleven of twelve directions are refused
by the wearer floor** and only four are under a free wrist. The arm can put
its hand at those points. What it cannot do is put its hand there *and* hold
the commanded wrist orientation *and* keep its elbow out of the person —
because a fixed 6-DOF pose on a 7-DOF arm spends the entire redundancy, and
the redundancy is the single joint whose whole purpose is to move the elbow
while the hand stays put.

The floor is not the problem and must not be touched (HARD CONSTRAINT 11).
It was 0.15 m under every policy in every run above, and a policy that
widened the workspace by letting the arm nearer the wearer would be a bug,
not a result.

## FREEING THE ROLL IS WORTH NOTHING. THIS IS THE USEFUL NEGATIVE RESULT

`spin` — let the gripper rotate about its own approach axis, hold the axis
fixed — is the relaxation that *sounds* safest, because roll only matters at
the instant of a grasp. It buys **+0 mm at the work point** and +3 mm at
home.

It is the tool **axis** that has to tilt. `orientation_policy.SPIN` therefore
exists in order to be **refused by name**, with the measurement in the
refusal string, so that nobody re-invents it, ships it, and measures zero.

## WHAT WAS BUILT

`src/srl_teleop/srl_teleop/orientation_policy.py` — one source, consulted by
`ik_follower_node` at runtime and by the measuring script, so the number
describes the policy the robot actually runs.

* `exact` (**the default**) yields exactly one candidate: the commanded
  orientation. Byte-for-byte today's behaviour. Every recorded mode
  comparison depends on that and none of them changes.
* `cone(deg)` yields the commanded orientation **first**, then tilts of the
  tool axis in rings, nearest first, never exceeding the tolerance.
* `free` is position only.

In `ik_follower_node` the escalation order is load-bearing and tested:

```
commanded pose
  -> redundancy re-seeds (the elbow swings, the pose does not move)
    -> orientation candidates, nearest first        <- new, off by default
      -> refuse, naming the policy and how many candidates were tried
```

A pose that solves at the commanded orientation is **always** solved at the
commanded orientation. Relaxation is only ever reached after the strict
answer has failed every redundancy seed.

**A relaxation is never silent.** It logs `[ORIENT] solved by tilting the
tool axis N deg`, and `/ik_status_<arm>` gained three fields — how many
solves needed a tilt, the last tilt, the worst so far — so the RViz master
shows it. An operator watching a gripper arrive at an angle they did not ask
for must be able to find out why from the screen.

## HARD CONSTRAINT 1 IS UNTOUCHED

`WORKSPACE_ORIENT` is not re-derived, not moved, and not recomputed from
home. Re-deriving it to match a level home costs T2 its right arm and that is
measured. What changed is *how strictly the constant is insisted upon*, it
defaults to insisting completely, and it is opt-in per node.

## ONE DEFECT THE TEST FOUND ON THE WAY

`WORKSPACE_ORIENT` **is not stored as a unit quaternion**: the left anchor's
norm is 0.99995844, the right's 1.00000561. Four parts in 100 000 sounds like
nothing. Build a rotation matrix from it and the matrix is not orthonormal,
so a "25.0 deg" tilt about it comes out 24.9996 deg and a cone that is
supposed to be a **bound** is exceeded by a candidate reporting that it is
not. Caught by the test that requires the reported tilt to equal the achieved
tilt, run against the real anchors instead of identity.
`orientation_policy` normalises on the way in. The constant is not rewritten.

## WHAT IS STILL NOT MEASURED

* **Nothing above ran on a real arm.** It is offline FK/IK against the URDF
  with the guard's own wearer model. The mechanism is geometric and should
  transfer; the exact millimetres will not.
* The walks stop at a 0.90 m cap and several directions hit it, so those
  reaches are lower bounds.
* `--as-follower` solves through the discrete candidate list the follower
  actually uses rather than a continuous cone bound. Where both have been
  run they agree; it has not been run for every direction.
* No task has been re-recorded under a cone. The measured 385 mm is what the
  IK can reach, not a claim that any task performs better — and under
  HARD CONSTRAINT 1's reasoning, the task set must be re-measured before any
  mode adopts a cone as its default.
