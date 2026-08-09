#!/usr/bin/env python3
"""The grasp pose must not drive the fingers through the object.

Pins the fault found in Job C: `candidates()` returned the bare object
centroid, and because the grasp quaternion aims the tool's +z straight down
the fingertips landed a full finger-length below it -- 51.8 to 105.8 mm
inside the object and the table under it, on every object in the catalogue.
"""
import numpy as np

from srl_autonomy import grasp_library as gl


def tip_depth_below_underside(oid):
    """Positive = the fingertip is that far BELOW the object's underside."""
    sz = gl.describe(oid)["size"][2]
    cand = gl.candidates(oid, np.zeros(3))[0]
    tip_z = float(cand["position"][2]) - gl.FINGERTIP_REACH_M
    return (-sz / 2.0) - tip_z


def test_no_object_is_penetrated():
    for oid in gl.OBJECTS:
        d = tip_depth_below_underside(oid)
        assert d < 0, "%s penetrated by %.1f mm" % (oid, 1000 * d)


def test_clearance_is_exactly_the_pad():
    for oid in gl.OBJECTS:
        d = tip_depth_below_underside(oid)
        assert abs(-d - gl.GRASP_PAD_M) < 1e-9


def test_taller_object_needs_less_offset():
    """The sign check. A taller object is gripped higher up, so the tool has
    to be raised LESS. If this inverts, the offset is being subtracted."""
    tall = gl.grasp_offset("tag_5")        # narrow_rod, 120 mm
    short = gl.grasp_offset("tag_4")       # flat_plate, 12 mm
    assert tall < short


def test_offset_is_actually_applied_by_candidates():
    """Guards against the formula being right while nothing calls it."""
    for oid in gl.OBJECTS:
        c = gl.candidates(oid, np.array([1.0, 2.0, 3.0]))[0]
        assert abs(c["position"][2] - (3.0 + gl.grasp_offset(oid))) < 1e-12
        assert abs(c["position"][0] - 1.0) < 1e-12   # lateral untouched
        assert abs(c["position"][1] - 2.0) < 1e-12


def test_pregrasp_stacks_above_the_corrected_grasp():
    g = gl.candidates("tag_0", np.zeros(3))[0]
    p = gl.pregrasp(g)
    assert p["position"][2] > g["position"][2]
