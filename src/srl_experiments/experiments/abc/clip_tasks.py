#!/usr/bin/env python3
"""The three CLIP tasks: A pick and place, B bimanual hold and place, C multimeter.

THESE ARE NOT THE STUDY TASKS. `tasks.py` holds the participant spec (A
positioning, B coordinated carry, C dual pursuit), verified N=10 over the
densified full path. This module holds the three tasks the RECORDING set
demonstrates, which is a different question: what the rig can be shown doing.

EVERY COORDINATE BELOW IS TAKEN FROM SOMETHING ALREADY VERIFIED, and where a
task needs a shape the study spec does not have, it is built from verified
points rather than chosen freshly. Inventing a nearby coordinate is how a
protocol acquires a pose that fails IK on the day, and this project has paid
for that once already.

  A  pick and place    from OPTIONAL_PICK_PLACE -- picks at |x| >= 0.30, which
                       is where a TOP-DOWN grasp is kinematically feasible
                       (measured 0/9 at x = 0.25, 9/9 at x = 0.30), with the
                       0.10 m standoff and the bin, 3/3 approaches and 3/3
                       places verified.
  B  hold and place    both arms, on the 500 mm span of TASK_B, whose whole
                       path is verified at N=10. LEFT HOLDS STILL while RIGHT
                       traverses -- that asymmetry is the task, and it is
                       drawn from the same verified band.
  C  multimeter        a two-handed instrument task on the same verified band:
                       left presents the body, right brings the probe to it
                       and holds contact.

WHAT "MULTIMETER" MEANS HERE, STATED PLAINLY. There is no multimeter in the
scene and no multimeter model in this repository. Task C is the two-handed
PRESENT-AND-PROBE MOTION that such a task consists of, executed on verified
coordinates. The clip shows the motion; it does not show an instrument, and
the caption on every C clip says so. Filming an empty gripper and captioning
it "multimeter" without that sentence would be the kind of claim this project
exists to avoid.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tasks as T                                            # noqa: E402
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "srl_teleop"))
from srl_teleop import gripper_state as _gs                  # noqa: E402

Y = T.Y                       # 0.35, the only fore/aft band both arms work in
SEP = T.TRAY_SEP              # 0.500, verified clear of the dead band


def _dense(path, step=0.03):
    out = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        for k in range(n):
            f = k / float(n)
            out.append([a[j] + f * (b[j] - a[j]) for j in range(3)])
    out.append(list(path[-1]))
    return out


def _hold(pt, n):
    return [list(pt) for _ in range(n)]


# ---------------------------------------------------------------- TASK A
# Single arm. The other arm holds its start pose so the clip shows ONE arm
# working, which is the point of the task.
A_PICK = [0.32, Y, 1.15]
A_BIN = [0.30, Y, 1.02]
A_STANDOFF = 0.10


def task_a():
    pre = [A_PICK[0], A_PICK[1], A_PICK[2] + A_STANDOFF]
    lift = [A_PICK[0], A_PICK[1], A_PICK[2] + 0.14]
    over = [A_BIN[0], A_BIN[1], A_BIN[2] + 0.16]
    left = _dense([pre, A_PICK]) + _hold(A_PICK, 6) + \
        _dense([A_PICK, lift, over, A_BIN]) + _hold(A_BIN, 4)
    return {"left": left, "right": _hold([-0.32, Y, 1.15], len(left))}


# ---------------------------------------------------------------- TASK B
# Bimanual. LEFT holds the work still; RIGHT brings a part to it and places.
B_HOLD = [SEP / 2.0, Y, 1.15]
B_START = [-SEP / 2.0, Y, 1.28]
B_PLACE = [-SEP / 2.0, Y, 1.13]


def task_b():
    right = _dense([B_START, B_PLACE]) + _hold(B_PLACE, 8)
    return {"left": _hold(B_HOLD, len(right)), "right": right}


# ---------------------------------------------------------------- TASK C
# Two-handed instrument motion: left presents, right probes and holds contact.
C_PRESENT = [SEP / 2.0, Y, 1.18]
C_PROBE_UP = [-SEP / 2.0, Y, 1.30]
C_PROBE_ON = [-SEP / 2.0, Y, 1.19]


def task_c():
    right = _dense([C_PROBE_UP, C_PROBE_ON]) + _hold(C_PROBE_ON, 10) + \
        _dense([C_PROBE_ON, C_PROBE_UP])
    left = _hold(C_PRESENT, len(right))
    return {"left": left, "right": right}


# THE ONE DEFINITION, imported. This was a third copy of the same map -- kept
# "so this module has no dependency on the recorder", which was the right
# instinct pointed at the wrong owner: the dependency belongs on srl_teleop,
# which depends on nothing in-repo, not on a script under scripts/. Three
# copies of the threshold that decides whether a grasp happened is three
# chances for a study's trials to be reclassified by a typo.
grip_for = _gs.grip_for
OPEN = _gs.CMD_OPEN_RAD


def _sched(n, arm, close_at, open_at, width_mm):
    """A per-waypoint gripper schedule: OPEN, close to the object's WIDTH,
    hold, open again.

    Closing to the WIDTH rather than fully is what makes the grasp legible and
    what makes the attachment honest -- the scene attaches only once the
    fingers reach 0.90 of this value, so a gripper that merely left the open
    position never picks anything up.
    """
    g = grip_for(width_mm)
    out = []
    for k in range(n):
        if k < close_at:
            out.append(OPEN)
        elif k < open_at:
            out.append(g)
        else:
            out.append(OPEN)
    return {arm: out, ("right" if arm == "left" else "left"): [OPEN] * n}


TASKS = {
    "a": dict(name="pick and place",
              scenario="S1_single_arm",
              build=task_a,
              # descend (open) -> close on the 40 mm block at the pick ->
              # carry -> open over the bin
              grip=lambda n: _sched(n, "left", 5, n - 4, 40),
              width_mm=40,
              expect="LEFT arm descends 0.10 m to the pick, closes, lifts "
                     "0.14 m, carries to the bin and releases. RIGHT arm "
                     "holds still throughout.",
              caveat=""),
    "b": dict(name="hold and place",
              scenario="S1_bimanual",
              build=task_b,
              # RIGHT already holds the part, carries it down, releases at the
              # placement; LEFT stays open, holding the work steady.
              grip=lambda n: _sched(n, "right", 1, n - 5, 45),
              width_mm=45,
              expect="LEFT arm HOLDS the work still at x=+0.25. RIGHT arm "
                     "brings the part down 0.15 m and places it. Both arms "
                     "engaged at once.",
              caveat=""),
    "c": dict(name="multimeter",
              scenario="S1_present_probe",
              build=task_c,
              # LEFT grips the multimeter for the whole task and never lets
              # go -- presenting it IS the task. RIGHT stays open: the probe
              # is the gripper itself.
              grip=lambda n: _sched(n, "left", 0, n, 50),
              width_mm=50,
              expect="LEFT presents the body and holds it steady. RIGHT "
                     "brings the probe down 0.11 m, holds contact, retracts.",
              caveat="The multimeter is a DUMMY BODY -- a coloured box of "
                     "the right size, not an instrument model. It is grasped, "
                     "carried and presented for real."),
}
