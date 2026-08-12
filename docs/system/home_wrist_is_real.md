# The home wrist points up by +85 and +79 degrees. This is REAL. Do not "fix" it.

At the home pose the wrists sit **+85 degrees (left) and +79 degrees (right)
above horizontal**. The grippers point almost straight up. On screen it looks
like a rendering fault or a quaternion convention error, and it is neither.

**The home joint angles are ground truth read from the physical arms.** They
are the legacy real home, recorded in `config/home_positions_{left,right}.txt`
and in the two `initial_positions` blocks of `srl_dual.urdf.xacro`. The
sim-to-real bridge replays sim joint angles onto the real arm, so any
difference between the stored home and the real one is commanded as a JUMP.
That is why the file says the angles must not be edited to make geometry look
right.

## What changing it would cost

Not a config edit. A physical recapture, and then:

* `P_HOME` for the affected arm, which is derived from the angles;
* `WORKSPACE_CENTRE` and `WORKSPACE_ORIENT`, since `offset = P_HOME`;
* every task coordinate, all verified N=10 against the current anchor;
* every clearance figure, measured against the current pose.

So it is a deliberate pass with a re-verification bill, not a tidy-up.

## The 30.7 degree anchor is a DIFFERENT number

Do not compare the two. They measure different things from different origins:

| figure | what it measures | from |
| --- | --- | --- |
| **+85 / +79 deg** | the WRIST's elevation at the home pose | the horizontal |
| **30.7 deg** | the teleop approach axis, the tool direction the commanded pose is pinned to | the shoulder |

Seeing 30.7 in one place and 85 in another and concluding one of them is wrong
is an easy mistake and it has nearly been made. They are consistent: the arm is
parked with the wrist high, and teleoperation pins the approach axis 30.7
degrees above horizontal, which is a claim about the commanded tool direction
and not about where the wrist happens to rest when idle.

## What was done instead

A PRESENTATION POSE, used only for the opening frame of a clip. The stored
home stays ground truth, nothing downstream is invalidated, and the footage
starts on something that reads as an arm ready to work. It is a camera
decision, not a kinematic one, and it is the only thing about the home pose
that should ever be changed for the sake of a picture.

## If you are here because the arms look wrong in RViz

Check in this order:

1. a stray `robot_state_publisher` owning `/robot_description` (this has
   happened, and it makes every joint read 0.000, which is also plausible);
2. a stale `move_group` from a previous stack;
3. the mount, which is the usual suspect when the geometry looks wrong.

The home angles are the last thing to suspect, not the first.
