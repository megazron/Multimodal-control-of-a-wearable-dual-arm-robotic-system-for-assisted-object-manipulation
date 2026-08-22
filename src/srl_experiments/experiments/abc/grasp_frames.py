#!/usr/bin/env python3
"""WHERE THE FINGER PADS ARE, AND HOW TO POINT THE HAND SOMEWHERE ELSE.

    from grasp_frames import PAD_MID_EE, q_from_axis, axis_for, wrist_for

WHY THIS EXISTS. `clip_tasks.ee_for()` converts an object position into the
wrist pose that puts the pads on it, and it does so through
`PAD_OFFSET_BY_ARM` -- a WORLD-frame vector, measured off TF, and therefore
only correct at the one orientation it was measured at: the pinned anchor
`master_calibration.WORKSPACE_ORIENT`. Every task in this repository commands
that anchor, so a world-frame offset has always been enough.

A task built for `06_full_autonomy` alone does not command it. Under 06 the
system supplies the whole pose, and the 2026-08-17 centre search measured that
the anchor is one of the 53 of 55 approach orientations that CANNOT touch an
object resting on a table. So T1 now carries its own approach, and the offset
has to rotate with it.

THE OFFSET IS THE SAME ON BOTH ARMS, AND THE 48.3 mm WAS THE ANCHOR
-------------------------------------------------------------------
`PAD_OFFSET_BY_ARM` records the two arms' wrist-to-pad vectors as 48.3 mm
apart and attributes it to the arms being parked asymmetrically. Rotated back
into each arm's own END-EFFECTOR frame by its own anchor quaternion, they are:

    left    (+0.00003, +0.00004, +0.11177)
    right   (+0.00004, +0.00002, +0.11184)

-- the same vector to 0.07 mm, purely along the tool axis. So the hardware is
symmetric and the 48.3 mm is entirely the two arms' different anchor
ORIENTATIONS. That is worth writing down because it means a per-arm pad table
is not needed once the offset is expressed in the frame it belongs to, and
because `test_pad_offset_has_one_source.py` reproduces both world vectors from
this one constant, which is the check that they have not drifted apart.
"""
import math

import numpy as np

# Finger-pad midpoint in the END EFFECTOR frame, metres. Along the tool axis
# and nothing else.
#
# MEASURED FROM FK, 2026-08-18, NOT DERIVED. It was 0.1118 -- the MAGNITUDE of
# `clip_tasks.PAD_OFFSET_BY_ARM`, which is a world-frame vector recorded at the
# anchor -- and `/compute_fk` on the two finger-tip links and the end-effector
# link, from one solution, on both arms, says 0.09833 with the two arms
# agreeing to 0.000 mm. The derivation was 13.47 mm long.
#
# IT SHOWED UP AS A GRASP THAT MISSED. `verify_t1.py` solves the pose the task
# commands and then asks FK where the tips landed: with 0.1118 the pad midpoint
# sat 13.48 mm from the cube centre on all four cubes of both arms, identically,
# which is the signature of a constant rather than of a path. With the measured
# value it sits on the cube. `scripts/measure_pad_mid_ee.py` is the instrument
# and `recordings/baselines/pad_mid_ee.json` the record.
#
# `PAD_OFFSET_BY_ARM` IS DELIBERATELY NOT CHANGED. T0, T2 and T3 declare their
# coordinates through `clip_tasks.ee_for` and `clip_scene` DRAWS their objects
# through the same offset, so task and picture agree with each other; moving it
# moves every coordinate in three tasks and every scene that draws them. What it
# means is that in those tasks the declared coordinate is 13.45 mm from where
# the object is drawn and grasped -- written down now instead of unknown.
PAD_MID_EE = (0.0, 0.0, 0.09833)

# =============================================================================
# AND IT IS THE WIDE-OPEN HAND, WHICH IS NOT WHERE A GRASP HAPPENS
# =============================================================================
# THE ROBOTIQ 85 IS A FOUR-BAR LINKAGE. Its fingers SWING; they do not
# translate. So the distance from the wrist to the midpoint of the finger tips
# is a function of how open the hand is, and the constant above is one point on
# that curve -- the fully open one, knuckle 0.0000 rad, tip span 135.5 mm.
#
# Measured from the URDF's own mimic chain by
# `scripts/measure_pad_mid_ee.py --by-width`, offline:
#
#     wide open (0 mm)      0.09833 m     <- PAD_MID_EE
#     a 40 mm cube          0.10976 m     +11.43 mm
#     a 20 mm object        0.11179 m     +13.46 mm
#
# BOTH OF THE NUMBERS THIS FILE HAS ARGUED ABOUT ARE ON THAT CURVE. The
# 0.11178 that `clip_tasks.PAD_OFFSET_BY_ARM` carried and that the 2026-08-18
# note calls "13.47 mm long" is the hand almost SHUT -- a 20 mm object. The
# 0.09833 that replaced it is the hand WIDE OPEN. The disagreement was never an
# error in either measurement; it was two gripper states being compared as
# though the quantity did not depend on one.
#
# AND IT EXPLAINS WHY THE EVIDENCE FLIPPED. `verify_t1.py` reads the finger
# tips out of FK at whatever opening the simulation's gripper was left at. On
# 2026-08-18 that was the open hand and the pad miss read 0.00 mm; on
# 2026-08-23, same geometry, same code, it read 13.52 mm. A measurement whose
# answer depends on leftover state was the evidence for the constant.
#
# SO THE QUANTITY TAKES A WIDTH. `pad_mid_ee_for(width_mm)` is the pad midpoint
# at `gripper_state.grip_for(width_mm)` -- the opening the fingers will be at
# when they are ON the object, which is the only opening at which "the pads are
# on the object" means anything.
_BY_WIDTH = None


def _by_width():
    global _BY_WIDTH
    if _BY_WIDTH is None:
        import json
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
        path = os.path.join(root, "recordings/baselines/pad_mid_ee_by_width.json")
        if not os.path.exists(path):
            raise RuntimeError(
                "no pad_mid_ee_by_width.json -- run `python3 "
                "scripts/measure_pad_mid_ee.py --by-width` (offline, no stack "
                "needed). The pad midpoint depends on the gripper opening and "
                "this module will not guess it.")
        _BY_WIDTH = json.load(open(path))
    return _BY_WIDTH


def pad_mid_ee_for(width_mm, arm="left"):
    """Pad midpoint in the EE frame, at the opening a `width_mm` object needs.

    Linear between tabulated widths; the table spans 0-85 mm, which is the
    whole range the jaws have, so this never extrapolates. `width_mm=0` is the
    wide-open hand and returns `PAD_MID_EE` exactly, which is the control that
    says the table and the constant are the same measurement.
    """
    rows = _by_width()["arms"][arm]
    ws = [r["width_mm"] for r in rows]
    zs = [r["along_axis_m"] for r in rows]
    w = max(ws[0], min(ws[-1], float(width_mm)))
    for i in range(1, len(ws)):
        if w <= ws[i]:
            t = (w - ws[i - 1]) / float(ws[i] - ws[i - 1] or 1)
            return (0.0, 0.0, zs[i - 1] + t * (zs[i] - zs[i - 1]))
    return (0.0, 0.0, zs[-1])


def q_matrix(q):
    """(x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def q_from_matrix(R):
    """Rotation matrix -> (x, y, z, w), branch-safe at every trace."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def axis_for(elev_deg, head_deg):
    """Unit approach direction. Heading 0 is +y, away from the wearer.

    Elevation is measured from horizontal, POSITIVE UP, so the pinned anchor
    is +30.8 deg (left) and a top-down hand is -90.
    """
    e, h = math.radians(elev_deg), math.radians(head_deg)
    return (math.sin(h) * math.cos(e), math.cos(h) * math.cos(e), math.sin(e))


def q_from_axis(axis, roll=0.0):
    """A wrist orientation whose tool axis (+z of the EE frame) is `axis`.

    `roll` spins the hand about that axis and decides which pair of faces the
    pads close on.
    """
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.95 else \
        np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, a)
    x = x / np.linalg.norm(x)
    y = np.cross(a, x)
    c, s = math.cos(roll), math.sin(roll)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return q_from_matrix(np.column_stack([x, y, a]) @ Rz)


def tool_axis(q):
    return q_matrix(q)[:, 2]


def elev_head_of(q):
    a = tool_axis(q)
    return (math.degrees(math.asin(max(-1.0, min(1.0, a[2])))),
            math.degrees(math.atan2(a[0], a[1])))


def wrist_for(obj_xyz, q, pad_mid_ee=PAD_MID_EE):
    """WRIST pose that puts the finger pads on `obj_xyz` at orientation `q`.

    The same construction `clip_tasks.ee_for` performs, with the offset
    rotated into the world by the orientation actually being commanded rather
    than baked in at the anchor.
    """
    return [float(v) for v in (np.asarray(obj_xyz, float)
                               - q_matrix(q) @ np.asarray(pad_mid_ee, float))]


def approach_path(obj_xyz, q, standoff, pad_mid_ee=PAD_MID_EE):
    """(pre-grasp wrist, grasp wrist): straight in ALONG the approach axis.

    NOT a vertical descent. A vertical standoff over a level approach would
    drop the hand onto the table from above and then slide it forward, which
    is a different motion from the one the reachability sweep measured.
    """
    ee = wrist_for(obj_xyz, q, pad_mid_ee)
    a = tool_axis(q)
    return [float(ee[i] - standoff * a[i]) for i in range(3)], ee
