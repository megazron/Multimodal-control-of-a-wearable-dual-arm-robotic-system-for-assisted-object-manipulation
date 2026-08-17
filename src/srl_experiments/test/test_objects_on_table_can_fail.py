"""THE OBJECTS-ON-TABLE CHECK MUST BE ABLE TO FAIL, AND TO PASS.

WHY THIS FILE EXISTS. The check was written to catch objects floating above the
table and its first version got the comparison backwards: it required the gap to
EQUAL the documented `FLOAT_GAP_M`, so a scene with the objects floating 120 mm
up PASSED and a scene with them resting on the table FAILED. It certified the
defect and would have rejected the fix.

That was corrected, but "it reports BLOCKED today" is not evidence that it can
report anything else. A check with one reachable answer is the same as no check,
and this repo has paid for that shape repeatedly -- CLAUDE.md: a check that
cannot fail on a deliberately broken input is not a check.

So this drives the REAL scene three ways through one displacement knob and pins
all three verdicts:

    displacement    gap        verdict     exit
    0 mm            120.0 mm   BLOCKED       3    the measured geometric block
    +5 mm           125.0 mm   DRIFT         1    it CAN fail
    -120 mm           0.0 mm   PASS          0    it CAN pass, resting

The third row is the important one: it says that if the scene were ever fixed,
this check would notice, which is the only reason to keep it.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(PKG, "experiments", "abc"))
sys.path.insert(0, PKG)

import verify_objects_on_table as V                            # noqa: E402
from srl_experiments import work_surface as WS                 # noqa: E402

TASKS = ["t1", "t1s2", "t3"]


@pytest.mark.parametrize("task", TASKS)
def test_the_real_scene_is_blocked_not_passing(task):
    """As shipped: every object registered and inside the footprint, the gap
    exactly the documented one, and NOT a pass."""
    r = V.check_task(task)
    state, code = V.verdict_for(r["work_plane_to_surface_m"],
                               r["geometry_ok"], r["documented_gap_m"])
    assert r["geometry_ok"], r["objects"]
    assert state == V.BLOCK and code == 3
    assert r["work_plane_to_surface_m"] == pytest.approx(WS.FLOAT_GAP_M,
                                                         abs=1e-9)
    assert not r["resting"]


@pytest.mark.parametrize("task", TASKS)
def test_a_deliberately_floated_scene_FAILS(task):
    """+5 mm is not the documented gap, so it is drift and must not be excused
    -- this is the direction the inverted version got wrong."""
    r = V.check_task(task, inject_float_m=0.005)
    state, code = V.verdict_for(r["work_plane_to_surface_m"],
                               r["geometry_ok"], r["documented_gap_m"])
    assert state == V.DRIFT and code == 1, (
        "floating the scene an extra 5 mm produced %s; the check is not "
        "sensitive to the thing it exists to measure" % state)


@pytest.mark.parametrize("task", TASKS)
def test_a_scene_whose_objects_REST_passes(task):
    """Displace by exactly the documented gap and the objects land on the
    table. This must be the only PASS, and it must return 0."""
    r = V.check_task(task, inject_float_m=-WS.FLOAT_GAP_M)
    state, code = V.verdict_for(r["work_plane_to_surface_m"],
                               r["geometry_ok"], r["documented_gap_m"])
    assert r["resting"], r["work_plane_to_surface_m"]
    assert state == V.PASS and code == 0


def test_exit_zero_is_reserved_for_resting():
    for s in (V.BLOCK, V.DRIFT, V.BADGEOM):
        assert V.CODES[s] != 0
    assert V.CODES[V.PASS] == 0


def test_t1s2_is_checked_against_its_OWN_layout():
    """It was not. `objects_for` read `T1_CUBES if task == "t1" else T1_CUBES`
    -- the same answer on both sides -- so asking about T1S2 measured stage
    ONE's cubes and stage one's single-arm pad pair. Stage 2 draws from its own
    seed over BOTH arms and has a pad pair per arm."""
    names = [o[0] for o in V.objects_for("t1s2")]
    assert any(n.startswith("cube_left_") for n in names), names
    assert any(n.startswith("cube_right_") for n in names), names
    assert any(n.startswith("plane_left_") for n in names), names
    assert any(n.startswith("plane_right_") for n in names), names
    # and it must NOT be stage 1's list
    assert [o[0] for o in V.objects_for("t1")] != names


def test_t1s2_follows_its_seed():
    """The scene node and the task are driven from ONE seed so the picture and
    the path cannot disagree; this check has to follow the same seed or it is
    measuring a layout nothing ran."""
    a = [tuple(o[1]) for o in V.objects_for("t1s2", seed=0)
         if o[0].startswith("cube_")]
    b = [tuple(o[1]) for o in V.objects_for("t1s2", seed=3)
         if o[0].startswith("cube_")]
    assert a != b, "the layout did not change with the seed"


def test_the_marking_is_registered_to_the_plane_it_bounds():
    """A boundary a participant keeps the WORK inside belongs at the work
    plane. Drawn on the table it sat 116.5 mm below the pads and the frame
    showed the task outside its own boundary."""
    import clip_scene as CS
    import clip_tasks as CT
    for task in ("t1", "t1s2"):
        for arm in CS.marked_arms(task):
            z = CS.marking_z(arm, task)
            assert z == pytest.approx(CT.BENCH_TOP + CS.MARK_T / 2.0,
                                      abs=1e-9)
            assert z > CT.BENCH_TOP - CS.PLANE_T, (
                "%s/%s marking at %.4f is below the pads it encloses"
                % (task, arm, z))
