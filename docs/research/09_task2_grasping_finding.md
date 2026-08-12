# Teleoperated grasping is not achievable with this master

**Decision, 2026-08-09: Task 2 loses its DIRECT and VR conditions.** They are
not replaced with easier objects. The absence is the result.

## The measurement

A top-down grasp of the task block requires the wrist to rotate
**\SI{169.7}{\degree}** (left arm) and **\SI{164.6}{\degree}** (right) from the
orientation `orientation_mode: fixed` pins the commanded wrist to. Both are
very nearly a reversal.

There is no channel that commands it:

* the master's wrist is unmeasurable — left j7 is restricted to
  \SI{26}{\degree} and j6 is incoherent; right j3, j5 and j7 are dead or
  incoherent. That is *why* the mode defaults to pinned;
* ORIENTATION_ASSIST, which exists to supply it, is a stub — **but NOT for
  the reason this document gave until 2026-08-12.** The earlier claim was that
  the `/compute_ik` plugin ignores `OrientationConstraint`, on the evidence of
  identical IK success with and without over 270 calls. **That was a tolerance
  artefact and the claim is WITHDRAWN.** The tolerance used (0.2 / 0.2 / π rad)
  was loose enough not to bind at the poses tested, so identical counts were
  the expected result either way — a success COUNT cannot distinguish "honoured
  but not binding" from "ignored".

  Re-measured by FK on the solution rather than by counting successes, at T1's
  four real grasp poses, with the decisive case being a request whose POSE and
  whose CONSTRAINT disagree:

  | request | result |
  | --- | --- |
  | exact pinned, no constraint | achieved **0.00°** from pinned |
  | pose PINNED + constraint TOP-DOWN 0.05 rad | **REFUSED, 4/4** |
  | exact top-down, no constraint | achieved **0.00°** from top-down |
  | pose TOP-DOWN + constraint TOP-DOWN 0.05 rad | achieved **0.00°** from top-down |

  Row 2 is the discriminator. Had the constraint been ignored the solver would
  have returned the pinned solution and reported success, exactly as row 1
  does. It refused instead, at every pose: **the constraint binds.** So a
  constrained yaw-free IK path is available and ORIENTATION_ASSIST can stop
  being a stub. Raw output: `recordings/baselines/orientation_constraint_fk.txt`.

  **The conclusion of this document is unchanged**, because it never rested on
  the solver. It rests on the MASTER: a 169.7° wrist rotation cannot be
  commanded by an input device whose wrist channels are dead. A solver that
  honours a constraint does not give the operator a wrist.
* the VR controller does report 6-DOF, but it drives the same follower through
  the same pinned-orientation path, so it inherits the same limit.

The recorded grasp clips execute correctly because the recorder calls
`/compute_ik` **directly** with the grasp quaternion, bypassing the
teleoperation orientation lock. They demonstrate that the *robot* can execute
an aligned grasp. They do not demonstrate that an *operator* can command one,
and the three recorded conditions differ only in trajectory smoothing, so they
do not distinguish it either.

## The wearer does NOT forbid top-down — the 0/128 measured something else

**Resolved 2026-08-12.** The 0/128 poses sat **240 mm inboard of T1's grasp
points**, so that sweep measured LATERAL REACH, not approach direction. The
wearer is not the ceiling, and the general claim that was proposed on the back
of it — *"a supernumerary arm cannot approach from above, because its own
wearer is above the workspace"* — is **FALSE at these grasp points and must not
appear anywhere.** It would have been a wrong claim generalised to every SRL.

The original null result also does not reproduce at T1's grasp points.
Measured there with `avoid_collisions=True` and the wearer in the scene,
top-down succeeds **4/4** and FK confirms the achieved orientation is 0.00°
from top-down. The 128-pose set is not in this repository — no script writes
it and no baseline records it — so the two cannot be reconciled by rerunning
the other probe.

### So is top-down the better approach? MEASURED, AND NO.

Once the premise dissolved, top-down was tested as a working approach for the
first time — N=10, full path, wearer AND furniture in the scene, constrained
yaw-free IK at 0.05 rad, i.e. exactly the mechanism modes 3/4/6 would use:

| task | pinned near-side | top-down, constrained |
| --- | --- | --- |
| T1 | 0/4 reachable | 0/4 reachable |
| T2 | **2/2, lift ≥ 300 mm** | 0/2 reachable |
| T3 | 0/2 reachable | **1/2, lift ≥ 300 mm** |

**The two approaches are COMPLEMENTARY, not ordered.** Each rescues a task the
other cannot do, so neither is the right global default and "switch to
top-down" would have traded T2 away for part of T3. The approach belongs in
the TASK definition, chosen per grasp, not in a global setting.

**And the lift premise does not survive either.** The near-side approach was
believed to cap the lift because it enters from below; measured, wherever the
grasp is reachable at all the lift is ≥ 300 mm — the sweep's own cap, never
the arm's — for BOTH approaches. There is no 40 mm ceiling to raise. Dropping
`LIFT_M` from 80 mm was premised on a limit that does not exist.

T1 fails under both approaches for a reason unrelated to orientation: the
object pedestals block the left arm (see the one-table layout work), and T1's
cubes are all on the side of the arm that cannot reach them.

## Why this is a finding rather than a gap

The thesis argues that autonomy's contribution here is **categorical**: it
supplies degrees of freedom the input device cannot measure, rather than
merely doing the same job faster. Task 2 is where that argument is
demonstrated instead of asserted.

Making the objects easier to grasp would have hidden it. A grasp that succeeds
because the object was chosen to suit a pinned wrist measures the object, not
the interface.

**Stated generally, and this is the transferable claim:** a body-worn master
that does not measure wrist orientation cannot support teleoperated grasping
of arbitrarily-oriented objects, whatever its position mapping does. The
limit is in what the input observes, not in the control law.

## What the mode comparison can and cannot claim for Task 2

**CAN claim.** That grasping is achievable in modes 4 and above and not in
modes 1-2 on this platform, and *why* — a specific, measured orientation
requirement against a specific, measured sensing limit. This is a capability
result and it is the strongest thing Task 2 produces.

**CANNOT claim.** Anything comparative. With no DIRECT condition there is no
baseline, so Task 2 yields **no completion-time comparison, no error
comparison, no workload comparison, and no learning-curve comparison** between
teleoperation and autonomy. Reporting a shared-autonomy completion time for
Task 2 without a baseline beside it would invite exactly the comparison the
design cannot support.

**CANNOT claim either**, and this one is easy to overstate: that autonomy is
*better* at grasping. Nothing was measured against. The claim is that
teleoperation cannot do it at all, which is a different and narrower
statement.

## Effect on the programme

The four remaining tasks are unaffected: Task 1 positioning, Task 3 rigid
carry, Task 4 compliant carry and Task 5 dual pursuit all command position
only, and all keep their full mode set. **The two-mode comparison therefore
rests on four tasks rather than five**, which costs statistical power and is
recorded as a cost rather than absorbed.

Task 2 becomes a single-condition capability demonstration in modes 4 and
above, plus the refusal in modes 1-2. It still earns its place: it is the only
task with a discrete success or failure outcome, and it is the only one that
demonstrates the categorical argument.

## If the wrist is ever repaired

Repairing left j6/j7 and right j3/j5/j7 would restore wrist measurement and
the DIRECT condition could return. The measurement to redo first is the one
above: the required rotation from the anchor, per arm, per object orientation.
Nothing else in this document survives that repair unchanged.
