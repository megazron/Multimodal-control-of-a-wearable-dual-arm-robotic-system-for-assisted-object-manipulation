"""THE SCENE HAS ONE OWNER FOR EACH SURFACE HEIGHT, AND THE GAP IS PINNED.

WHAT WENT WRONG. `clip_tasks.BENCH_TOP` was the literal 1.10 and
`clip_scene.TABLE_TOP` was the literal 0.950. Every cube, coloured pad and
workspace-marking tile in T1 is positioned against the first; the only surface
with geometry is drawn against the second. The raised bench that used to close
the 150 mm between them was deleted as scenery in the "THE BENCH IS GONE. ONE
TABLE." pass and the objects were left in the air.

Nothing noticed for two months, because nothing compared the two numbers.
There was no test, and the floating was written down as a design fact in a
source comment ("it is 170 mm below the work plane and nothing rests on it"),
which is how a defect survives a code read.

WHAT THIS FILE ASSERTS.

  1. Both consumers READ `srl_experiments.work_surface`. Not "have the same
     value" -- two literals that happen to agree today is the state that
     produced the bug. If either stops reading the owner, this fails.
  2. The gap between the work plane and the drawn surface is EXACTLY the
     documented, measured value. Not "small", not "less than X": exactly, so
     that changing either height is a deliberate act with a measurement behind
     it. Both the direction and the size are pinned -- a SMALLER gap fails too,
     because the numbers behind it were measured against this pair.
  3. The surface is at the highest value measured to cost T1's pick paths
     nothing, and the measurement that says so is named in the assertion
     message rather than left for someone to go and find.

WHY THE GAP IS NOT ZERO, so nobody "fixes" it by editing a constant. Measured
at `WORKSPACE_ORIENT` -- the anchor `run_abc.send()` actually commands, which
is 32.26 deg from the home wrist that the previous sweeps used by mistake:

  * `search_centre_on_surface.py`: x 0.00..0.45, y 0.10..0.55, top 0.70..1.10,
    overhang 0 and 0.05, both arms, pinned AND top-down, objects RESTING on the
    surface -- 0 of 3360 cells survive the full pick path.
  * `search_t1_layout_on_surface.py`: T1's own x = 0.560, tops 0.900..1.100 --
    every cell fails, including a near edge at y = 0. It is not the edge; the
    slab occupies the volume the arm needs.
  * `measure_objects_on_the_table.py`: objects held at 1.120, slab swept --
    0.950 / 1.000 / 1.020 cost 0 of 54 waypoints, 1.040 costs 12, 1.100 costs
    38. Controls: no slab 0, slab through the objects 46.

So T1-1 is BLOCKED, the block is the pinned wrist, and 1.020 is the closest a
surface can come. `scripts/verify_objects_on_table.py` reports it as a known
block rather than as a pass or a failure.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, "experiments", "abc"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from srl_experiments import work_surface as WS          # noqa: E402

# THE MEASURED NUMBERS, WRITTEN DOWN ONCE. If a future measurement moves the
# surface, these change WITH the measurement and in the same commit.
DOCUMENTED_WORK_PLANE_M = 1.100
DOCUMENTED_SURFACE_M = 0.980
DOCUMENTED_GAP_M = 0.120
DOCUMENTED_NEAR_Y = 0.100

# FROM THE FULL-PATH SWEEP, NOT THE PICK SWEEP, and the difference is the whole
# reason this constant is spelled out here. `sweep_surface_vs_t1_path.py`,
# 171 waypoints, N=3, controls correct -- failures by (top, near edge):
#
#     top \ edge   0.050   0.062   0.080   0.100
#     0.950            7       0       0       0
#     0.980           16      14      14       0
#     1.000           33      18      14      14
#     1.020           36      36      32      18
#
# Height and forward reach trade against each other, so the surface is a PAIR
# and is pinned as one. `measure_objects_on_the_table.py` walks only the six
# pick paths and called 1.020 free; on the full path 1.020 costs 18 even at its
# best edge, and 36 at the edge the marking wanted. A one-axis sweep of a
# two-axis constraint reads the best cell off the wrong axis.
FULL_PATH_FAILURES = {
    (0.950, 0.050): 7, (0.950, 0.062): 0, (0.950, 0.080): 0, (0.950, 0.100): 0,
    (0.980, 0.050): 16, (0.980, 0.062): 14, (0.980, 0.080): 14, (0.980, 0.100): 0,
    (1.000, 0.050): 33, (1.000, 0.062): 18, (1.000, 0.080): 14, (1.000, 0.100): 14,
    (1.020, 0.050): 36, (1.020, 0.062): 36, (1.020, 0.080): 32, (1.020, 0.100): 18,
}


def test_work_plane_is_the_documented_measured_value():
    assert WS.WORK_PLANE_M == pytest.approx(DOCUMENTED_WORK_PLANE_M, abs=1e-9)


def test_surface_is_the_highest_free_pair_on_the_full_path():
    """The (top, edge) pair must be free, and no free pair may be higher.

    Asserted against the whole measured grid rather than against a single
    remembered number, so the claim "this is the highest one that costs
    nothing" is checked rather than asserted."""
    free = [(t, e) for (t, e), f in FULL_PATH_FAILURES.items() if f == 0]
    here = (WS.DECLARED_M, WS.DECLARED_NEAR_Y)
    assert here in FULL_PATH_FAILURES, (
        "the surface pair %s was never measured. Add it to "
        "sweep_surface_vs_t1_path.py's grid and re-run before shipping it."
        % (here,))
    assert FULL_PATH_FAILURES[here] == 0, (
        "the surface pair %s costs T1 %d of 171 waypoints. That is not a "
        "scenery change; it deletes waypoints."
        % (here, FULL_PATH_FAILURES[here]))
    assert WS.DECLARED_M == max(t for t, _e in free), (
        "a free pair exists at a HIGHER surface (%s free); the objects are "
        "floating further above the table than they need to."
        % sorted(free, reverse=True)[:3])


def test_the_near_edge_is_as_far_forward_as_this_height_allows():
    free_at_h = [e for (t, e), f in FULL_PATH_FAILURES.items()
                 if f == 0 and t == WS.DECLARED_M]
    assert WS.DECLARED_NEAR_Y == min(free_at_h), (
        "at a top of %.3f the near edge may come to %.3f for free; it is at "
        "%.3f. Bringing the surface forward is what stops the workspace "
        "marking being clipped."
        % (WS.DECLARED_M, min(free_at_h), WS.DECLARED_NEAR_Y))


def test_the_gap_is_exactly_the_documented_one_in_both_directions():
    assert WS.FLOAT_GAP_M == pytest.approx(DOCUMENTED_GAP_M, abs=1e-9), (
        "the work plane stands %.1f mm above the drawn surface; the measured, "
        "documented value is %.1f mm. A SMALLER gap fails this too, on "
        "purpose: every number behind it was measured against this pair, so "
        "moving either height needs a re-measurement in the same commit."
        % (WS.FLOAT_GAP_M * 1000.0, DOCUMENTED_GAP_M * 1000.0))
    assert WS.FLOAT_GAP_M == pytest.approx(
        WS.WORK_PLANE_M - WS.DECLARED_M, abs=1e-9)


def test_the_gap_is_open_so_t1_1_is_not_claimed_satisfied():
    """The honest direction. If this ever fails, T1-1 may have been SOLVED --
    and then the audit log, TASK_SPEC section 9 and this file all have to be
    rewritten together, which is exactly why it is asserted rather than left
    as a comment."""
    assert WS.FLOAT_GAP_M > 0.0, (
        "the objects now rest ON the surface. That would be a real result, "
        "and it must arrive with the measurement that supports it plus "
        "updates to TASK_SPEC section 9 and the audit log -- not by editing a "
        "constant.")


def test_bench_top_reads_the_owner_and_is_not_its_own_literal():
    import clip_tasks as CT
    assert CT.BENCH_TOP == pytest.approx(WS.work_plane(), abs=1e-9)
    # AND IT MUST BE READING IT, not agreeing by coincidence. Move the owner
    # and the consumer has to move with it; a literal will not.
    old = WS.WORK_PLANE_M
    try:
        WS.WORK_PLANE_M = old + 0.037
        import importlib
        importlib.reload(CT)
        assert CT.BENCH_TOP == pytest.approx(old + 0.037, abs=1e-9), (
            "clip_tasks.BENCH_TOP did not follow work_surface.work_plane() -- "
            "it is carrying its own copy again, which is the 150 mm defect "
            "returning.")
    finally:
        WS.WORK_PLANE_M = old
        import importlib
        importlib.reload(CT)
    assert CT.BENCH_TOP == pytest.approx(old, abs=1e-9)


def test_table_top_reads_the_owner_and_is_not_its_own_literal():
    import clip_scene as CS
    assert CS.TABLE_TOP == pytest.approx(WS.table_top(), abs=1e-9)
    old = WS.DECLARED_M
    try:
        WS.DECLARED_M = old - 0.041
        import importlib
        importlib.reload(CS)
        assert CS.TABLE_TOP == pytest.approx(old - 0.041, abs=1e-9), (
            "clip_scene.TABLE_TOP did not follow work_surface.table_top().")
    finally:
        WS.DECLARED_M = old
        import importlib
        importlib.reload(CS)
    assert CS.TABLE_TOP == pytest.approx(old, abs=1e-9)


def test_the_two_heights_are_different_and_that_is_the_point():
    """They were collapsed into one owner at first, which was wrong in the
    other direction: it made the objects rest on the table by definition and
    the render disagreed. They are two facts."""
    import clip_scene as CS
    import clip_tasks as CT
    assert CT.BENCH_TOP != CS.TABLE_TOP
    assert CT.BENCH_TOP - CS.TABLE_TOP == pytest.approx(WS.FLOAT_GAP_M,
                                                        abs=1e-9)
