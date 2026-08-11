# Two-arm object transfer is not achievable on this platform

**A measured result about supernumerary-limb geometry, not a shortcoming of
this implementation.**

**Run it:** `python3 scripts/measure_t1_cubes_t2_transfer.py --repeats 10`
→ `recordings/baselines/t1_cubes_t2_transfer.json`

---

## The claim, in one number

> **The closest the two end effectors can get to each other, anywhere in the
> reachable workspace, is 0.200 m. A cube crossing between two container rims
> needs about 0.115 m. Transfer fails by a factor of 1.7.**

That single figure settles rim-to-rim, tilted-toward-each-other, vertical
stack, and every other arrangement at once — because all of them require the
two grippers to be *near each other*, and none of them can be.

## Why it was measured this way

Gate 3 tested a **vertical stack** — source cup directly above destination —
and found 0 of 6975 candidate pairs. That was a legitimate refutation of
pouring, but not of transfer in general: two containers brought rim-to-rim,
or tilted toward each other, is a different relative pose and could in
principle have been feasible where a stack is not.

So the question was generalised to the thing every transfer geometry has in
common:

> what is the smallest distance between an end-effector pose the **left** arm
> can reach and one the **right** arm can reach?

A reachability table is built over 31 × 6 × 8 = **1488 poses per arm per
orientation policy**, with the bench in the planning scene, then every
left-pose/right-pose pair is measured.

| left policy | right policy | pairs tested | **minimum separation** |
| --- | --- | --- | --- |
| fixed (pinned anchor) | fixed | 66 402 | 0.240 m |
| fixed | top-down | 49 028 | 0.253 m |
| top-down | fixed | 62 775 | 0.233 m |
| top-down | top-down | 46 350 | **0.200 m** |

**Controls, so the number is the robot's and not the sweep's:** all four
reachable sets are non-empty — left/fixed 238, left/top-down 225,
right/fixed 279, right/top-down 206, of 1488 each. The script refuses to
report a minimum distance from an empty set.

## What it means

The two arms' reachable sets are **disjoint with a 200 mm gap at their
closest approach**. The task the gap forbids is not "pouring" — it is
*any* two-arm operation on a shared object at a shared place:

* container-to-container transfer, in any orientation;
* inter-arm handover (independently measured: 0 of 16 candidate transfer
  points, 0 of 63 shared frontal grid cells);
* one arm presenting an object into the other's grasp;
* ALOHA's task set entire — slot battery, open cup, thread velcro — all of
  which need both grippers within centimetres.

## Why this is a property of the configuration

The cause is on record and is not a tuning failure:

* the arms are mounted on the **wearer's back**, so both must reach forward
  over a shoulder before they can do anything;
* the wearer's torso occupies the centreline, which is exactly where a
  bench-top bimanual workspace would be;
* the residual `|v_R − M·v_L| = 1.3837 m` is a property of the home joint
  angles and is **provably independent of the mount rotation**. Fixing it
  means re-parking an arm in hardware.

A sweep of 108 mount candidates raises the shared-cell count from 3 to 22 of
63 *kinematically*, and the best candidates move the arms inboard, forward and
lower — **towards the person**. Clearance and shared reach are in direct
conflict on a worn platform in a way they never are on a bench.

## What can still be bimanual, and which kind

The repository distinguishes two, and only one survives:

| | requires | status here |
| --- | --- | --- |
| **simultaneity** | two arms doing separate things at once; no coupling | **available** — the two targets sit in the two disjoint sets (Task C, dual pursuit) |
| **physical coupling** | one body, two grips at distinct points, neither arm's pose free given the other's | **available** — the grips are 500 mm apart and *span* the gap rather than meeting inside it |
| **transfer / handover** | both arms at or near one place | **impossible**, by 0.200 m against 0.115 m |

**T2 is therefore the coordinated carry, and it is the stronger of the two
available claims.** A rigid 500 mm tray with a loose ball, held at two points,
verified band z 1.10–1.30, failure is tilt past 6.8°. Removing one arm makes
the task **impossible**, not slower — whereas under simultaneity a participant
working sequentially merely degrades, which contaminates the dependent
variable with strategy. Tilt RMS and separation error are joint metrics no
single arm can produce, and the pilot shows they discriminate across
conditions: **7.698 → 4.219 → 1.911 mm**.

Note that the 500 mm span is not a workaround; it is the direct consequence of
the same geometry. Each grip sits at |x| = 0.25, outside the dead band, which
is exactly why a *spanning* object works where a *shared* one cannot.

## How to state it in the write-up

Report the 0.200 m minimum separation with its controls and the pair counts.
Without them it reads as an excuse; with them it is a characterisation of the
configuration, and it explains — without any appeal to control quality — why
the SRL literature's bimanual demonstrations are overwhelmingly *bracing* and
*support* tasks rather than the fine in-hand coordination a bench-top pair
performs.

**Do not** describe this as a limitation of the controller, the solver or the
gripper. It is measured with collision-aware IK at ten repeats over a table of
1488 poses per arm, and it is a statement about where the two arms can be.
