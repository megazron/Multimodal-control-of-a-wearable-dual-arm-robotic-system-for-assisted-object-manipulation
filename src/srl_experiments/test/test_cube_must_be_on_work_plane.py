"""A CUBE IS ON THE WORK PLANE; A FRAGMENT OF A PAD IS NOT.

WHY THE HEIGHT TEST EXISTS AT ALL. T1's coloured pads are the SAME COLOURS as
its cubes -- deliberately, because "each cube ended on the pad of its own
colour" has to be judgeable from a frame. The consequence is that the only
thing separating a cube from a piece of pad in the detector is blob SIZE at
range, and that is not enough: two 40 mm cubes standing in front of a
210 x 130 mm pad split it into several connected fragments. The largest is
rejected as too big and the leftovers are cube-sized.

Measured inside the recording sweep, which is where it bites: T1 was refused
twice, once with "saw 8 cubes, the task expects 4" and once with 9, while the
same code standalone on the same stack saw exactly 4. It looked intermittent; it
is a sensitivity to precisely where the arm settled at the observe pose.

WHAT THE REFUSAL'S OWN DUMP SHOWED, and it is unambiguous:

    4 detections   z 1.1118 .. 1.1191     work plane (T1_Z) = 1.1200
    5 detections   z 0.9576 .. 0.9601     160 mm low

The real cubes are within 9 mm of the plane they rest on. The spurious ones are
160 mm off. `PLANE_WINDOW_M` is 40 mm -- one cube -- which is an order of
magnitude clear of the real spread and four times clear of the spurious group.

WHAT THIS FILE ASSERTS. That the window admits what it must and rejects what it
must, in both directions, and that it is derived from the OWNED work-plane
constant rather than from a literal. The arithmetic is constructed, so this is
the standing rule's allowed use of synthetic data: no rendering is involved.

The gate is deliberately capable of rejecting a REAL cube that has been moved
off the plane, and that is correct behaviour, not a false positive: a cube
160 mm below the work plane is not one this arm can pick, and saying so beats
picking at it.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, "experiments", "abc"))

import vision_grasp as VG                                # noqa: E402
import msc_clip_tasks as M                               # noqa: E402

M_T1_Z = M.T1_Z
from srl_experiments import work_surface as WS           # noqa: E402

# The two groups exactly as the sweep's refusal dumped them, EXPRESSED AS
# OFFSETS FROM THE PLANE THEY WERE MEASURED AT.
#
# They were absolute heights -- 1.1118 and friends, against a work plane of
# 1.1200 -- and T1's plane moved on 2026-08-17 when the task was rebuilt with
# its objects RESTING on the table at 0.9700 rather than floating 120 mm above
# it. Absolute numbers would have made these tests fail for the one reason a
# test must never fail: the world moved and the property did not.
#
# What the measurement is actually about is the SEPARATION -- real cubes
# within 9 mm of the plane they rest on, pad fragments 160 mm below it -- and
# that is a property of the scene's geometry, not of where the scene sits. So
# the offsets are recorded and re-anchored on whatever plane T1 declares.
_PLANE_WHEN_MEASURED = 1.1200
MEASURED_CUBE_OFFSETS = tuple(round(z - _PLANE_WHEN_MEASURED, 4) for z in
                              (1.1118, 1.1149, 1.1150, 1.1153, 1.1191))
MEASURED_FRAGMENT_OFFSETS = tuple(round(z - _PLANE_WHEN_MEASURED, 4) for z in
                                  (0.9576, 0.9580, 0.9596, 0.9601))
MEASURED_CUBES = tuple(round(M_T1_Z + d, 4) for d in MEASURED_CUBE_OFFSETS)
MEASURED_PAD_FRAGMENTS = tuple(round(M_T1_Z + d, 4)
                               for d in MEASURED_FRAGMENT_OFFSETS)


def _accepts(z):
    """The gate's decision for one deprojected z, as vision_grasp applies it."""
    return abs(float(z) - M.T1_Z) <= VG.PLANE_WINDOW_M


def test_the_window_is_one_cube():
    assert VG.PLANE_WINDOW_M == pytest.approx(M.CUBE_M, abs=1e-9), (
        "the window is meant to be one cube wide; a window unrelated to the "
        "object size is a number nobody can re-derive.")


def test_the_work_plane_comes_from_its_owner():
    """T1_Z is a surface plus half a cube, and that surface has ONE owner.

    IT IS NO LONGER `work_surface.work_plane()`, AND THE DIVERGENCE IS THE
    2026-08-17 REBUILD. T1's objects used to float 120 mm above the table on
    the shared work plane; they now REST on T1's own table, so its plane is
    `t1_task.TABLE_TOP`. Every other task still works on the shared plane.
    Both halves are asserted, because a T1 that silently drifted back onto
    `work_plane()` would put its cubes 120 mm in the air and pass every check
    that does not look at the pixels.
    """
    import t1_task as T1M
    assert M.T1_Z == pytest.approx(T1M.TABLE_TOP + M.CUBE_M / 2.0, abs=1e-9)
    assert M.T1_Z != pytest.approx(WS.work_plane() + M.CUBE_M / 2.0, abs=1e-6)
    # STAGE 2 CAME ACROSS ON 2026-08-18, and it came across with the walk
    # behind it: `recordings/baselines/t1_stage2_paths.json` records eight
    # seeds walked over the composed path at N=10, every one clean. Until then
    # this asserted the opposite, because stage 2 on stage 1's plane without
    # that walk would have been cubes placed onto pads nobody had reached.
    assert M.T1S2_Z == pytest.approx(M.T1_Z, abs=1e-9), (
        "stage 2 has drifted off stage 1's work plane again; the two stages "
        "are one geometry and test_t1_stages_agree says so")


@pytest.mark.parametrize("z", MEASURED_CUBES)
def test_every_real_cube_is_accepted(z):
    assert _accepts(z), (
        "z = %.4f is one of the four cubes the sweep actually detected "
        "(worst 9 mm from the plane); rejecting it would lose a real grasp."
        % z)


@pytest.mark.parametrize("z", MEASURED_PAD_FRAGMENTS)
def test_every_pad_fragment_is_rejected(z):
    assert not _accepts(z), (
        "z = %.4f is one of the spurious pad fragments (160 mm low). "
        "Accepting it is what made the detector report 8 and 9 cubes." % z)


def test_the_two_groups_are_separated_by_an_order_of_magnitude():
    """The margin, stated as a number, so shrinking it is visible."""
    worst_cube = max(abs(z - M.T1_Z) for z in MEASURED_CUBES)
    best_frag = min(abs(z - M.T1_Z) for z in MEASURED_PAD_FRAGMENTS)
    assert worst_cube < VG.PLANE_WINDOW_M < best_frag
    assert best_frag / worst_cube > 10.0, (
        "the real and spurious groups are only %.1fx apart; the window is no "
        "longer comfortably between them and the threshold needs re-measuring."
        % (best_frag / worst_cube))


def test_it_can_fail_on_a_deliberately_broken_input():
    """A CHECK THAT CANNOT FAIL IS NOT A CHECK. A cube exactly on the plane is
    accepted; the same cube dropped by two windows is not."""
    assert _accepts(M.T1_Z)
    assert not _accepts(M.T1_Z - 2 * VG.PLANE_WINDOW_M)
    assert not _accepts(M.T1_Z + 2 * VG.PLANE_WINDOW_M)


def test_the_boundary_is_inclusive_and_symmetric():
    eps = 1e-9
    assert _accepts(M.T1_Z + VG.PLANE_WINDOW_M - eps)
    assert _accepts(M.T1_Z - VG.PLANE_WINDOW_M + eps)
    assert not _accepts(M.T1_Z + VG.PLANE_WINDOW_M + 1e-4)
    assert not _accepts(M.T1_Z - VG.PLANE_WINDOW_M - 1e-4)


# ===========================================================================
#  THE DUPLICATE RULE, against the two cases actually observed
# ===========================================================================
# The work-plane window cannot remove a pad fragment that is LEVEL with the
# plane, and those exist: the blue pad sits 95 mm behind the cube row and 20 mm
# below it, and from the observe pose those offsets nearly cancel in range. Both
# refusals contained exactly one such pair, and in both the correct cube had the
# LARGER blob -- which is the rule.
def _dedupe(dets, cube_m=None):
    """The rule as vision_grasp applies it: largest blob first, drop anything
    within one cube width of something already kept."""
    import math
    cube_m = M.CUBE_M if cube_m is None else cube_m
    kept = []
    for d in sorted(dets, key=lambda z: -z["blob_px"]):
        if not any(math.dist(d["xy"], k["xy"]) < cube_m for k in kept):
            kept.append(d)
    return sorted(kept, key=lambda z: z["xy"][0])


# "saw 5 cubes": the blue sliver beside the green cube at x = 0.620.
CASE_FIVE = [
    dict(colour="blue", xy=(0.5587, 0.1215), blob_px=45.0),
    dict(colour="blue", xy=(0.6222, 0.1431), blob_px=33.0),   # pad sliver
    dict(colour="blue", xy=(0.6806, 0.1206), blob_px=44.0),
    dict(colour="green", xy=(0.6198, 0.1179), blob_px=39.0),  # the real cube
    dict(colour="green", xy=(0.7422, 0.1213), blob_px=43.0),
]
# "saw 9 cubes", after the work-plane window has removed the five low ones.
CASE_NINE_ON_PLANE = [
    dict(colour="blue", xy=(0.5563, 0.1157), blob_px=45.0),
    dict(colour="blue", xy=(0.6187, 0.1391), blob_px=33.0),   # pad sliver
    dict(colour="blue", xy=(0.6784, 0.1143), blob_px=43.0),
    dict(colour="green", xy=(0.6159, 0.1124), blob_px=47.0),  # the real cube
    dict(colour="green", xy=(0.7396, 0.1142), blob_px=43.0),
]
# What the layout says is true, by x order.
TRUTH_COLOURS = ("blue", "green", "blue", "green")


@pytest.mark.parametrize("case,label", [(CASE_FIVE, "saw 5"),
                                        (CASE_NINE_ON_PLANE, "saw 9")])
def test_dedupe_recovers_exactly_four_cubes(case, label):
    out = _dedupe(case)
    assert len(out) == 4, (
        "%s: dedupe left %d detections, not 4 -- %s"
        % (label, len(out), [(d["colour"], d["xy"]) for d in out]))


@pytest.mark.parametrize("case,label", [(CASE_FIVE, "saw 5"),
                                        (CASE_NINE_ON_PLANE, "saw 9")])
def test_dedupe_keeps_the_right_colour_in_x_order(case, label):
    """The pad sliver is BLUE and sits beside the GREEN cube, so choosing
    wrongly here would send that cube to the wrong pad -- a visible,
    scoreable error rather than a missing detection."""
    got = tuple(d["colour"] for d in _dedupe(case))
    assert got == TRUTH_COLOURS, "%s: got %s, truth %s" % (label, got,
                                                           TRUTH_COLOURS)


@pytest.mark.parametrize("case", [CASE_FIVE, CASE_NINE_ON_PLANE])
def test_the_four_survivors_are_on_the_declared_pitch(case):
    """A 60 mm pitch, so consecutive survivors are ~60 mm apart. If dedupe
    ever kept a sliver instead of a cube this spacing would break."""
    xs = [d["xy"][0] for d in _dedupe(case)]
    gaps = [b - a for a, b in zip(xs, xs[1:])]
    assert all(0.045 < g < 0.075 for g in gaps), gaps


def test_dedupe_does_not_merge_genuinely_separate_cubes():
    """THE OTHER DIRECTION. Four cubes on the real 60 mm pitch must survive
    untouched -- a rule that collapses the layout would be worse than the
    duplicates it removes."""
    real = [dict(colour="blue", xy=(x, 0.120), blob_px=40.0)
            for x in (0.56, 0.62, 0.68, 0.74)]
    assert len(_dedupe(real)) == 4


def test_the_window_is_below_the_layout_pitch():
    """The rule is only safe because one cube width is smaller than the pitch
    the layout uses. Assert that relationship rather than trusting it."""
    xs = sorted(c[0] for c in M.T1_CUBES)
    pitch = min(b - a for a, b in zip(xs, xs[1:]))
    assert M.CUBE_M < pitch, (
        "the cubes are %.3f m apart and one cube is %.3f m wide; the duplicate "
        "rule would merge adjacent cubes." % (pitch, M.CUBE_M))
