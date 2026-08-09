#!/usr/bin/env python3
"""
grasp_library.py — the lookup table. No learned predictor, on purpose.

A learned grasp predictor would be a second thing that can fail during a user
study, and its failures would be silent and correlated with object appearance —
exactly the sort of confound that cannot be removed in analysis. A table of
top-down grasps at the centroid, with the gripper aligned to the object's
minor axis, is boring, inspectable, and fails in ways an experimenter can see.

The one rule that matters: a candidate is not a grasp until /compute_ik has
accepted it. `grasp_generator` enforces that; this module only proposes.
"""
import math

import numpy as np

# Robotiq 2F-85: 85 mm stroke. Anything wider cannot be grasped at all, and
# saying so early is better than offering a grasp that will fail in front of a
# participant.
GRIPPER_MAX_WIDTH_M = 0.085
GRIPPER_SAFE_WIDTH_M = 0.075          # leave clearance for pose error

# HOW FAR THE FINGERTIPS REACH along the tool's +z from end_effector_link.
# MEASURED from TF on the live model (both arms agree to 4 dp), not taken from
# a datasheet: scripts/measure_grasp_penetration.py reads
# <arm>_robotiq_85_{left,right}_finger_tip_link and averages them.
#
# THIS CONSTANT IS WHY GRASPS PENETRATED. The grasp quaternion aims the tool's
# +z straight DOWN, so commanding end_effector_link to an object's centroid
# put the fingertips 111.8 mm BELOW that centroid -- through the object and
# into the table under it. The gripper never closed on anything.
FINGERTIP_REACH_M = 0.1118
# Clearance left between the fingertip and the object's underside.
GRASP_PAD_M = 0.005


def grasp_offset(object_id, pad=GRASP_PAD_M):
    """How far ABOVE the centroid to put the tool for a top-down grasp.

    Derived rather than tuned. With the tool at centroid + d, the fingertip
    lands at centroid + d - FINGERTIP_REACH_M, and we want it `pad` above the
    object's underside at centroid - size_z/2:

        d = FINGERTIP_REACH_M - size_z/2 + pad

    i.e. the object's half-height plus a pad, offset by the finger length that
    made the correction necessary. A taller object needs LESS offset, which is
    the sanity check on the sign.
    """
    return FINGERTIP_REACH_M - describe(object_id)["size"][2] / 2.0 + pad

# Object catalogue, keyed by the AprilTag id the object carries.
#   size      (x, y, z) extents in the object's own frame, metres
#   grasp     "top_down" (the only style implemented)
#   approach  standoff along -z before closing, metres
OBJECTS = {
    "tag_0": dict(name="small_cube", size=(0.040, 0.040, 0.040), approach=0.10),
    "tag_1": dict(name="tall_box", size=(0.035, 0.070, 0.090), approach=0.12),
    "tag_2": dict(name="wide_block", size=(0.100, 0.040, 0.035), approach=0.10),
    "tag_3": dict(name="cylinder", size=(0.045, 0.045, 0.080), approach=0.11),
    "tag_4": dict(name="flat_plate", size=(0.090, 0.090, 0.012), approach=0.08),
    "tag_5": dict(name="narrow_rod", size=(0.020, 0.020, 0.120), approach=0.12),
}
DEFAULT_OBJECT = dict(name="unknown", size=(0.045, 0.045, 0.045), approach=0.10)


def describe(object_id):
    return OBJECTS.get(object_id, DEFAULT_OBJECT)


def graspable_width(object_id):
    """Narrowest horizontal extent — what the fingers must span."""
    s = describe(object_id)["size"]
    return min(s[0], s[1])


def is_graspable(object_id):
    w = graspable_width(object_id)
    return w <= GRIPPER_SAFE_WIDTH_M, w


def _quat_from_z_and_x(z_axis, x_hint):
    """Build a quaternion whose +z is z_axis and whose +x is as close to
    x_hint as orthogonality allows."""
    z = np.asarray(z_axis, float)
    z /= np.linalg.norm(z)
    x = np.asarray(x_hint, float) - np.dot(x_hint, z) * z
    n = np.linalg.norm(x)
    if n < 1e-6:                      # hint parallel to z; pick anything stable
        x = np.array([1.0, 0, 0]) - z[0] * z
        n = np.linalg.norm(x)
    x /= n
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
             (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
             (R[1, 0] - R[0, 1]) / s]
    q = np.array(q, float)
    return q / np.linalg.norm(q)


def candidates(object_id, position, yaw_object=0.0, n_yaw=8):
    """Top-down grasp candidates at the object centroid.

    The gripper's approach axis (+z of end_effector_link) points DOWN, and the
    finger-opening axis is aligned with the object's MINOR horizontal axis —
    that is the direction the fingers have to span, so aligning to it is what
    makes the object fit in the 85 mm stroke.

    Several yaw variants are returned around the ideal one, because a grasp
    that is geometrically perfect but kinematically unreachable is useless;
    the caller tries them in order and keeps the first /compute_ik accepts.
    Returned best-first, not in arbitrary order.
    """
    spec = describe(object_id)
    sx, sy, _ = spec["size"]
    # OFFSET ALONG THE RETREAT DIRECTION. This used to be `p = position`, the
    # bare centroid, which drove the fingers through the object every time.
    p = np.asarray(position, float) + np.array([0.0, 0.0,
                                                grasp_offset(object_id)])

    # Minor horizontal axis in the object frame, rotated into world by the
    # object's own yaw. Fingers close ALONG this direction.
    minor_in_obj = np.array([1.0, 0.0, 0.0]) if sx <= sy else np.array([0.0, 1.0, 0.0])
    c, s = math.cos(yaw_object), math.sin(yaw_object)
    minor = np.array([c * minor_in_obj[0] - s * minor_in_obj[1],
                      s * minor_in_obj[0] + c * minor_in_obj[1], 0.0])

    out = []
    # Ideal first, then symmetric alternatives at increasing yaw offset. A
    # 180 deg flip is free (the gripper is symmetric), so offsets are folded
    # into [-90, 90].
    offsets = [0.0]
    step = math.pi / max(2, n_yaw)
    for k in range(1, n_yaw // 2 + 1):
        offsets += [k * step, -k * step]
    for off in offsets:
        co, so = math.cos(off), math.sin(off)
        axis = np.array([co * minor[0] - so * minor[1],
                         so * minor[0] + co * minor[1], 0.0])
        q = _quat_from_z_and_x(np.array([0.0, 0.0, -1.0]), axis)
        out.append(dict(position=p.copy(), quat=q, yaw_offset=off,
                        approach=spec["approach"],
                        width=min(sx, sy), name=spec["name"]))
    return out


def pregrasp(grasp):
    """Standoff pose: same orientation, `approach` metres above the grasp.

    NOTE the standoff now stacks on top of the grasp offset, so a standoff
    that was reachable from the centroid may not be from the corrected pose.
    Measured: for an object at z=1.15 the corrected grasp is reachable (100%)
    while the standoff 0.10 m above it is NOT (0%). That is a real constraint
    on where objects may be placed, not a defect in this function.
    """
    p = grasp["position"].copy()
    p[2] += grasp["approach"]
    return dict(grasp, position=p)
