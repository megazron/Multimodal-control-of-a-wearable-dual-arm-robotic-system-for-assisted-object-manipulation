# THE PHYSICS GATE — result, and the architecture decision it settles

**Run it:** `python3 scripts/gazebo_gate/gate.py` (and `control_check.py`
first — see "the command path" below).

**The question.** Can the Robotiq 2F-85, as *this workspace* actually
describes it, close on a 40 mm cube in Gazebo, hold it without jitter, and
drop it when released? PASS was defined before anything ran: held without
jitter, and falls on release.

**The answer: FAIL.** Not because Gazebo cannot do it in principle, but
because getting the *jaw contact* to behave took a sustained sequence of
engine swaps, a physics constraint substituted in software, and gain tuning —
and it still does not pass. Object dynamics, by contrast, worked first time
and never needed touching. That asymmetry is the whole finding, and it points
at exactly one of the three architectures.

---

## What the rig is

One real Gen3 7-DOF + real 2F-85, from the shipped description, hung **upside
down** from a fixed post above a bench, every arm joint at zero. That makes
the approach top-down by construction with **no IK call anywhere**, and the
lift is a *fold* — `joint_2 = +a`, `joint_4 = −2a`, `joint_6 = +a` — which
with the three rolls at zero keeps the tool orientation exactly constant,
because for a planar chain the tool angle is the SUM of the bends and that sum
is zero for every `a`. `joint_1` is coaxial with the inverted base, so
rotating it swings the held object without moving the approach axis by a
degree: a second disturbance, free.

No ros2_control, no MoveIt, no ROS. The gate is a physics question and every
layer between the setpoint and the joint is somewhere the answer can come out
wrong for a reason that is not physics.

## The measured result

| check | result | evidence |
| --- | --- | --- |
| lifted | **yes** | cube rose 0.256 m with the hand |
| survives a 90° arm rotation | **yes** | 0.24–0.48 mm RMS relative motion, ≤1.9 mm p2p |
| steady in the hand | **no** | 32 mm p2p in x through the hold window |
| falls on release | **no** | fingers reopened fully (0.1355 m); the cube stayed at z = 1.019 and crept 0.08 mm/s |

(That column is the run at the *shipped* fingertip friction. At a realistic
friction it fails differently and worse — see "why the pads will not let go".)

And the two numbers that decide it:

* **Closing the gripper shoved the 50 g cube 88 mm sideways** before capturing
  it — x went 0.022 → 0.088 during the close.
* **The fingers stalled 17 mm too wide.** Free-air separation at the same
  command is 0.079 m and a 40 mm cube should stop them at 0.091 m; they
  stopped at **0.108 m**. Whatever is being held, it is not being pinched on
  its faces.

So the object is captured by jamming and wedging rather than by a grasp, and
it is then held on release rather than dropped.

## Why the pads will not let go: `mu = 100000`

`robotiq_description`'s fingertip links ship

```xml
<surface><friction><ode><mu1>100000.0</mu1><mu2>100000.0</mu2></ode></friction></surface>
```

That is not a friction coefficient, it is a way of writing "never slip". Any
contact at all becomes a weld, which is consistent with a cube that hangs
motionless in a fully-open hand. **A gate that only passes at mu = 1e5 has not
answered the question**, which is why `--tip-mu` exists.

**Repeated at mu = 1.0, it fails the other way, and that is what settles it:**

| | shipped, mu = 1e5 | realistic, mu = 1.0 |
| --- | --- | --- |
| lifted | yes, 0.256 m | **no** — cube rise −0.150 m |
| where the cube ended | 1.019 m, motionless in an open hand | **0.620 m — the bench top**, i.e. knocked off the pedestal and never picked up |
| in the hand during "hold" | 21 mm below the pads | **422 mm below the pads**, rolling (55 mm RMS, 133 mm p2p) |
| fell on release | **no** | yes (it was already on the bench) |

So the two ends bracket the behaviour and **both fail, in opposite
directions**: at the shipped friction the object cannot be let go of, and at a
physical friction it cannot be picked up. Making it work means finding a
friction, a closing angle, a contact stiffness and a set of finger gains that
are simultaneously right — with no external reference to say when you have
arrived, because there is no real-arm measurement of any of them.

**That is sustained tuning of contact, which was the pre-agreed FAIL
condition. It is a FAIL.**

## The command path, and three traps that each produced confident nonsense

Before any gate number is believed, `scripts/gazebo_gate/control_check.py`
must pass: command a joint, read it back, require it to be where it was sent.
It exists because the gate produced a complete run of plausible numbers while
**every setpoint was landing on a topic with no subscriber**.

1. **`<topic>` is not the parameter.** gz-sim's `JointPositionController`
   takes `<sub_topic>`, and with neither it subscribes to
   `/model/<model>/joint/<joint>/0/cmd_pos`. A `<topic>` element is dropped
   in silence. During that run the gripper *appeared* to track a closing
   sweep — the fingers were drooping under gravity at roughly the rate the
   sweep was stepping. A monotone curve that is not the thing you commanded
   is the worst kind of false positive.
2. **The controllers are inert until their first message,** and publishing
   eight setpoints with `gz topic -p` takes seconds. During those seconds the
   arm is free: it collapses, `joint_4` and `joint_6` end past their limits,
   and the continuous rolls spin at 17–27 rad/s. Fixed by starting the world
   **paused**, loading every setpoint, then running. `<initial_position>` is a
   real parameter of this plugin and does **not** substitute for it.
3. **A link's pose is reported relative to its MODEL, and gz's JSON omits
   zero-valued fields.** So `cube_link` reads as exactly `(0, 0, 0)` in every
   sample for ever, and parsed with a `0.0` default that is a
   plausible-looking measurement rather than a gap. One entire gate run scored
   the cube as lying on the floor from t = 0 while the gripper was in fact
   holding it. Read the **model** entry; the arm's links are safe only because
   its model sits at the world origin. `evaluate()` now refuses to report if
   the cube is exactly at the origin in every sample.

## The engine fork — neither engine can do both halves

This is the structural finding, and it is not a tuning problem.

| | dartsim (**default**) | bullet-featherstone |
| --- | --- | --- |
| mimic constraints | **none.** No symbols; says so at load: *"the chosen physics engine does not support mimic constraints, so no constraint will be created"* | **yes** — `SetJointMimicConstraint` |
| controlling this arm | **holds every joint to 0.002 rad** | at the same gains the arm collapses; joint values are **identical with and without commands** (`joint_4` −2.5691 vs −2.5701) |
| raising the gains | not needed | fixes the bends, destabilises the load-free rolls: worst \|q\| 0.21 → 0.43 → 0.86 rad at 50× / 200× / 600× |
| world joint types | any | **fixed only** — a prismatic joint to the world fails the *whole* kinematic tree, not just that joint |

The 2F-85 is a parallel gripper *only* because four follower joints mimic the
driven knuckle. So the rig runs on dartsim with the coupling done in
**software**: one position controller per finger joint, driven together with
the multipliers taken from the URDF's own `<mimic>` tags.

**That is not the same thing as a constraint.** A constraint is enforced by
the solver and cannot be violated; six controllers can each be pushed off
their setpoint by contact, independently. The 17 mm stall above is what that
looks like. Every grip-force number from this rig inherits it.

## Cost

| | |
| --- | --- |
| real-time factor, full arm + gripper, 1 ms step | **0.75–0.98** |
| gz sim, headless, no GUI, llvmpipe | works; `-s`, no rendering needed |
| a bare falling cube (no arm) | RTF ≈ 7 |

Object dynamics are cheap. The 17 mesh collision geometries of the arm and
hand are what costs.

---

# THE DECISION: option (c)

> **Objects fall, settle and stack physically. Grasping stays scripted.**

Recommended on the gate result, not on preference.

**Why not (b), full Gazebo.** The gate is the cheapest possible version of the
grasping question — one arm, no IK, a top-down approach, a light symmetric
cube, a pure vertical lift — and it still fails, on a hand whose parallel
linkage no available engine can both constrain *and* control. Making it pass
would mean tuning contact stiffness, friction, closing speed and finger gains
against each other, and the shipped `mu = 1e5` means the obvious "fix" is to
make the pads stickier until it works. That is a tuning loop with no natural
end and no external reference, on a property no participant measurement
depends on.

**Why not (a), markers only.** The measurement that colour-matched placement
actually rests on is where an object *ends up* — a cube landing in the bin, a
misplaced cube tipping off a paper plane, a stack that does or does not stand.
Markers cannot produce any of that: placement always "succeeds", and a
kinematic object cannot fall over. The gate showed this half of the physics is
the half that works, so declining it costs something real for no saving.

**What (c) buys, and what it costs.**

* Real placement outcomes: settling, toppling, rolling, stacking, and
  therefore a genuine binary success/failure at the moment of release.
* No jaw contact anywhere in the loop, so none of the fork above applies and
  the collision meshes of the fingers can be simplified or dropped, which is
  where the real-time factor goes.
* The workspace already has the scripted half: `clip_scene.py` attaches an
  object to the gripper link when the fingers reach the object's width, and
  releases it where it is put. Option (c) replaces only the *kinematic* object
  with a gz-physics one and keeps that trigger.
* **What it does not buy, and must be stated in the write-up:** grasp
  *acquisition* is assumed, not simulated. Slip, squeeze force, in-hand
  rotation and grasp failure are outside the model. Any claim about grasp
  robustness needs the real arm.

## KEEP MOCK AND PHYSICS FIGURES APART

Every tracking, clearance and placement number currently in this repository
was measured against `mock_components/GenericSystem`, **which echoes commands
and has no dynamics**. docs/ENGINEERING_LOG.md already records the consequence: payload made
*exactly zero* difference to tracking RMS (0.0164 m both ways, difference
0.0000 m) — a property of the mock, not evidence about payload.

So a physics figure and a mock figure are not comparable and must never be
placed in the same column. Under (c), every measurement acquires a `dynamics:
mock | physics` field, and any table mixing the two is wrong on its face.

---

## Reproducing

```
python3 scripts/gazebo_gate/control_check.py     # command path: MUST pass first
python3 scripts/gazebo_gate/gate.py --probe      # pads, lift calibration, jaw map
python3 scripts/gazebo_gate/gate.py              # the gate
python3 scripts/gazebo_gate/gate.py --tip-mu 1.0 # again without the shipped mu=1e5
python3 scripts/gazebo_gate/gate.py --engine bullet-featherstone   # the fork
```

Every run writes `result.json`, the generated world and the raw pose stream to
its own directory, so a later run is diffed against it rather than remembered.
