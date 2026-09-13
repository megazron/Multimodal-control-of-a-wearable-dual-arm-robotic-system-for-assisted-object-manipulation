#!/usr/bin/env python3
"""KNOWN ANSWERS FOR THE YAW PATH. TASK_SPEC.md U-2.

The fault being fixed, stated once: an identity quaternion means "nobody
measured the orientation", and every consumer read it as "the object is at
zero degrees". So an object at 30 degrees got a square grasp, and no check
anywhere could tell that apart from a correct grasp on a square object. It is
listed as `object_rotated_30deg`, SILENT, in the lab-day fault table.

These are arithmetic against angles written down here, which is the
constructed ground truth docs/ENGINEERING_LOG.md permits.
"""

import math
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))

from srl_perception import scene_fingerprint as sf                # noqa: E402


def q_yaw(deg):
    a = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(a), math.cos(a))


IDENT = (0.0, 0.0, 0.0, 1.0)


# ---- the distinction the whole fix is about ------------------------------
def test_identity_is_unknown_not_zero():
    assert sf.yaw_from_quat(IDENT) is None
    assert sf.yaw_from_quat(None) is None
    assert abs(sf.yaw_from_quat(q_yaw(30.0)) - 30.0) < 1e-6


def test_observation_reports_whether_yaw_was_measured():
    unknown = sf.Observation("cube", (0.3, 0.2, 1.1), IDENT)
    known = sf.Observation("cube", (0.3, 0.2, 1.1), q_yaw(30.0))
    assert unknown.yaw_known is False and unknown.yaw_deg is None
    assert known.yaw_known is True and abs(known.yaw_deg - 30.0) < 1e-6


def test_the_unmeasured_case_survives_a_round_trip():
    """as_dict/from_dict must keep None as None.

    Writing the key only when it is known would make an unmeasured object and
    an old file indistinguishable to every reader.
    """
    o = sf.Observation("cube", (0.3, 0.2, 1.1), IDENT)
    d = o.as_dict()
    assert "yaw_deg" in d and d["yaw_deg"] is None and d["yaw_known"] is False
    assert sf.Observation.from_dict(d).yaw_known is False


# ---- symmetry, because a cube is not a wrench ----------------------------
def test_a_cube_turned_90_degrees_is_not_turned():
    """The gripper cannot tell 0 from 90 on a square object.

    Without folding, every cube nudged past 45 degrees would report a large
    rotation and every scan would flag a rotated object.
    """
    assert abs(sf.wrap_symmetric(90.0, 90.0)) < 1e-9
    assert abs(sf.wrap_symmetric(89.0, 90.0) - (-1.0)) < 1e-9
    assert abs(sf.wrap_symmetric(30.0, 90.0) - 30.0) < 1e-9
    # a 180-degree-symmetric object folds differently, and must
    assert abs(sf.wrap_symmetric(179.0, 180.0) - (-1.0)) < 1e-9


def test_circular_mean_does_not_average_across_the_fold():
    """89 and -89 are one degree apart, not 178.

    An arithmetic mean gives 0, which is wrong by the whole symmetry, and it
    would be wrong QUIETLY -- the number looks perfectly reasonable.
    """
    m = sf.mean_yaw_deg([89.0, -89.0], 90.0)
    assert abs(abs(m) - 90.0) < 1e-6 or abs(m) < 1e-6
    assert abs(sf.mean_yaw_deg([29.0, 31.0], 90.0) - 30.0) < 1e-6
    assert sf.mean_yaw_deg([None, None], 90.0) is None
    assert abs(sf.mean_yaw_deg([None, 30.0], 90.0) - 30.0) < 1e-6


def test_yaw_is_averaged_over_views_not_taken_from_the_first():
    """The position has always been a running mean; the orientation was
    first-come. One bad view fixed the yaw for a whole sweep."""
    o = sf.Observation("cube", (0.3, 0.2, 1.1), q_yaw(28.0))
    for v in (30.0, 31.0, 31.0):
        o.add_yaw(v)
    assert 29.0 < o.yaw_deg < 31.0
    assert len(o.yaw_views) == 4


# ---- what compare() now sees ---------------------------------------------
def test_a_rotated_object_is_reported_as_changed():
    """THE CASE THAT USED TO PASS SILENTLY.

    Same place, turned 30 degrees. Position alone calls it SAME.

    `yaw_deg=0.0` is passed EXPLICITLY rather than as q_yaw(0.0), because
    q_yaw(0.0) IS the identity quaternion and identity means unmeasured --
    see yaw_from_quat. That is the one angle the wire format cannot carry,
    and the test says so instead of tripping over it.
    """
    stored = [sf.Observation("cube", (0.30, 0.20, 1.12), yaw_deg=0.0)]
    same = [sf.Observation("cube", (0.30, 0.20, 1.12), q_yaw(0.5))]
    turned = [sf.Observation("cube", (0.30, 0.20, 1.12), q_yaw(30.0))]

    v_same, _ = sf.compare(stored, same)
    v_turn, _ = sf.compare(stored, turned)
    assert v_same[0]["verdict"] == sf.SAME
    assert v_turn[0]["verdict"] != sf.SAME
    assert v_turn[0]["rot_deg"] > 25.0


def test_an_unmeasured_yaw_does_not_fabricate_a_verdict():
    """If either side never measured yaw, no yaw verdict is produced.

    Comparing a measured 30 against an unmeasured object as though the
    unmeasured one were at 0 would invent a rotation that was never seen.
    """
    stored = [sf.Observation("cube", (0.30, 0.20, 1.12), IDENT)]
    obs = [sf.Observation("cube", (0.30, 0.20, 1.12), q_yaw(30.0))]
    v, _ = sf.compare(stored, obs)
    assert v[0]["verdict"] == sf.SAME
    assert v[0]["rot_deg"] is None


def test_a_cube_turned_90_still_compares_same():
    stored = [sf.Observation("cube", (0.30, 0.20, 1.12), q_yaw(0.0))]
    obs = [sf.Observation("cube", (0.30, 0.20, 1.12), q_yaw(90.0))]
    v, _ = sf.compare(stored, obs)
    assert v[0]["verdict"] == sf.SAME, v[0]
