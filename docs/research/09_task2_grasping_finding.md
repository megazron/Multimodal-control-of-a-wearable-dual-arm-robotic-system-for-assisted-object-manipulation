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
* ORIENTATION_ASSIST, which exists to supply it, is a stub because this
  `/compute_ik` plugin ignores `OrientationConstraint` (measured: identical IK
  success with and without, over 270 calls);
* the VR controller does report 6-DOF, but it drives the same follower through
  the same pinned-orientation path, so it inherits the same limit.

The recorded grasp clips execute correctly because the recorder calls
`/compute_ik` **directly** with the grasp quaternion, bypassing the
teleoperation orientation lock. They demonstrate that the *robot* can execute
an aligned grasp. They do not demonstrate that an *operator* can command one,
and the three recorded conditions differ only in trajectory smoothing, so they
do not distinguish it either.

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
