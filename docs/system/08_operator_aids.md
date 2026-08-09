# What else the existing hardware could give the operator

Six candidates, each judged on feasibility with the sensors already fitted
(14 potentiometers, 2 inertial units, 2 force-sensitive resistors, 2 buttons,
2 wrist cameras) and on value in the **two-person** configuration this system
is built around. Two were built. Four were rejected, and the reasons are
specific to this rig rather than general.

## Built

### 1. Workspace-boundary feedback  — `boundary_feedback_node`

**The problem it solves is the project's own recurring failure.** The
reachable set from home is strongly anisotropic: measured isotropy 0.16 on the
left arm and 0.10 on the right, with a worst direction of \SI{0.14}{m} against
a median of \SI{0.40}{m}. An operator meets that boundary by the arm *stopping*
— which is exactly the silent-blocking class this project has spent months
eliminating everywhere else. The arm not moving looks identical whether it is
blocked, unreachable, or receiving nothing.

**It predicts rather than reacts.** Reacting is useless: by the time IK fails
the operator is already at the wall. The node probes AHEAD along the current
direction of travel and reports how far the boundary is and, by name, what
kind of boundary it is: inverse-kinematics infeasibility, the wearer clearance
floor, or the step guard.

Feasibility is the reason this is cheap: everything needed already exists. The
follower runs IK anyway, `clearance.py` is the shipped metric, and the
commanded pose stream gives the direction. Nothing new is sensed.

### 2. Wrist camera relay  — DESIGNED, NOT BUILT

**Value is specific to two people.** When operator and wearer are the same
person, the operator can look at the task. When they are different people the
operator has no view of the workspace at all, and the arms are on somebody
else's back. This is the single largest information gap in the configuration.

It is also nearly free: the image topics exist, the detector already
subscribes to them, and adding a viewer adds a subscriber rather than a
consumer of the device. Two subscribers on one topic is the point of
publish/subscribe, not the serial-port bug.

The measured caveat travels with it: at the home pose the **left** wrist camera
points up and back and sees nothing at table height, and only the right sees
the working volume. A relay shows the operator what the camera sees, which at
home is not much. That is a property of the home joint angles, not of the
relay.

**STATUS: not built.** The evaluation stands and it remains the second choice,
but it was not implemented in this pass. It is a subscriber and a widget, so
it is a small job; it is listed in `docs/NEXT_SESSION.md`.

## Status of what was built

`boundary_feedback_node` is implemented and its arithmetic is right, but
**its live verification did not complete**: `/compute_ik` was not being served
by the time the test ran, because the sim stack had been torn down during the
Part 4 shared-memory cleanup and did not come back. The node behaved correctly
under that condition — it guards on `service_is_ready()` and published
nothing rather than inventing a boundary — but "it correctly did nothing" is
not evidence that it correctly does something.

What IS established:

* the executor deadlock described below was real, was found by measurement,
  and is fixed;
* bisection costs 5 inverse-kinematics calls per arm per tick against 10 for a
  linear walk, arithmetic that does not depend on a running stack.

What is NOT established: that the warning actually leads the wall, and by how
much. That is one run against a healthy stack and it is the first thing to do
next.

## Rejected

### 3. Variable scaling driven by force-sensor pressure  — REJECTED, conflict

The force sensors **are** the gripper, and that is load-bearing: proportional
close, a deadband at 250 counts because a resting hand reaches 191, and a
latch at 1200. Driving scale from the same signal means the operator cannot
change scale without changing grip, or grip without changing scale. Using the
opposite arm's sensor instead couples the two arms, which is worse in a task
set whose whole point is bimanual independence.

There is no third pressure channel to move it to.

### 4. Gesture shortcuts from inertial patterns  — REJECTED, sensor conflict

The inertial unit is the **primary direction sensor**: elevation comes from
gravity, and it is the one part of the position mapping that works on both
arms. A gesture recogniser would have to separate intentional gestures from
ordinary teleoperation motion using the same signal that carries the command.
A false positive fires an action mid-task.

The measurement that settles it: the right inertial unit drifts
\SI{44}{\degree\per\minute} and shows \SI{72}{\degree\per\second} spikes while
**stationary**. Gesture detection on that channel would misfire on noise, and
the failure would be a command the operator did not give.

### 5. Force-sensor dead-man  — REJECTED, same conflict as (3)

Requiring continuous pressure to permit motion means the gripper is held
closed whenever the arm is allowed to move. The light-touch band below the
gripper deadband is not usable either: rest already reaches 191 against a
deadband of 250, so the separable window is under 60 counts of a
\num{3600}-count range.

**But the concern behind it is real and is recorded here rather than
dismissed.** The clutch is a *toggle*, so it latches engaged; a true dead-man
requires continuous action and cannot latch. The right place to fix that is
the clutch's own state machine or a foot switch, not the force sensors.

### 6. Snap-to-object assist in direct mode  — REJECTED, experimental design

Feasible, and it would help. That is the problem: direct teleoperation is the
**baseline condition** of every planned comparison. Adding target-aware
assistance to it makes the baseline not a baseline, and the shared-autonomy
condition would then be measured against something that already contains
assistance.

This capability already exists as its own mode. Keeping it there is what makes
the comparison mean anything.
