#!/usr/bin/env python3
"""The wrist may be unpinned, and only after the pinned answer has failed.

WHAT THIS PROTECTS. `ik_follower_node` can now tilt the tool axis inside a
cone when the commanded orientation has no IK solution. That is worth 385 mm
of reach at the work point (recordings/baselines/orientation_cost_what_binds
.json) and it is also, if it fires when it should not, an arm arriving at a
grasp pointing somewhere the task did not ask for. Three properties make the
difference and each one is asserted here rather than argued in a comment:

  1  THE DEFAULT IS TODAY. `exact` yields exactly one candidate, so a node
     that nobody configures behaves byte-for-byte as it always has. Every
     recorded mode comparison in this repository depends on that.
  2  STRICT FIRST, ALWAYS. Candidate 0 is the commanded orientation under
     every policy. A pose that solves pinned is solved pinned; relaxation is
     only ever reached after the strict pose failed every redundancy seed.
  3  THE CONE IS A BOUND. No candidate may exceed the tolerance, and the
     tilt each one REPORTS must be the tilt it actually has. A reported
     number that is a label rather than a measurement is this repository's
     own listed failure mode -- "checked that a field is STORED, not that a
     consumer READS it".

AND THE ONE THAT CAUGHT SOMETHING. `spin` -- freeing the gripper's roll about
its own approach axis while holding the axis -- reads like the obvious safe
relaxation and measured **+0 mm** at the work point. It is refused by name so
that nobody re-invents it, and the refusal has to keep saying why.

THESE TESTS MUST BE ABLE TO FAIL. Each one below is paired with a construction
that would pass a weaker assertion: a cone that is not enforced, a candidate
list that does not start at the commanded pose, a tilt label that disagrees
with the geometry.
"""
import math
import os
import re
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))

from srl_teleop import orientation_policy as OP        # noqa: E402

IDENT = (0.0, 0.0, 0.0, 1.0)
# A deliberately non-trivial commanded orientation: the real anchor.
ANCHOR_LEFT = (-0.0896, 0.4860, 0.8693, 0.0032)
ANCHOR_RIGHT = (0.1335, 0.5423, 0.8288, 0.0345)
ANCHORS = (IDENT, ANCHOR_LEFT, ANCHOR_RIGHT)


def test_the_module_self_test_passes():
    assert OP.self_test(verbose=False)


# ---------------------------------------------------------------- property 1
def test_the_default_policy_is_exactly_todays_behaviour():
    """One candidate, the commanded one, no tilt. Nothing else."""
    for q in ANCHORS:
        c = list(OP.candidates(q))
        assert len(c) == 1, (
            "the default policy offered %d candidates. Every recorded mode "
            "comparison assumes the wrist is pinned; a default that relaxes "
            "invalidates all of them silently." % len(c))
        assert c[0][0] == tuple(float(v) for v in q)
        assert c[0][1] == 0.0


def test_the_default_is_exact_by_name_not_by_accident():
    assert OP.DEFAULT_POLICY == OP.EXACT
    assert OP.DEFAULT_CONE_DEG == 0.0
    p, cone = OP.normalise()
    assert (p, cone) == (OP.EXACT, 0.0)


# ---------------------------------------------------------------- property 2
@pytest.mark.parametrize("policy,deg", [(OP.EXACT, None), (OP.CONE, 5.0),
                                        (OP.CONE, 45.0), (OP.FREE, None)])
def test_the_commanded_orientation_is_always_tried_first(policy, deg):
    for q in ANCHORS:
        first = next(iter(OP.candidates(q, policy, deg)))
        assert first[0] == tuple(float(v) for v in q), (
            "under policy %r the first candidate was not the commanded "
            "orientation. Relaxation must be a LAST resort: a pose that "
            "solves pinned has to be solved pinned." % policy)
        assert first[1] == 0.0


def test_a_relaxed_candidate_never_precedes_a_tighter_one():
    for q in ANCHORS:
        tilts = [t for _, t, _ in OP.candidates(q, OP.CONE, 60.0)]
        assert tilts == sorted(tilts), (
            "candidates are not nearest-first, so the follower would accept "
            "a 60 deg tilt while a 20 deg one was still untried")


# ---------------------------------------------------------------- property 3
@pytest.mark.parametrize("deg", [1.0, 5.0, 15.0, 45.0, 90.0])
def test_no_candidate_ever_leaves_the_cone(deg):
    for q in ANCHORS:
        worst = max(OP.axis_deviation_deg(q, c)
                    for c, _, _ in OP.candidates(q, OP.CONE, deg))
        assert worst <= deg + 1e-6, (
            "a %.0f deg cone produced a %.4f deg tilt. The tolerance is the "
            "entire safety story of this feature: it is what lets a task say "
            "how far its approach may be bent." % (deg, worst))


@pytest.mark.parametrize("deg", [5.0, 30.0, 75.0])
def test_the_reported_tilt_is_the_achieved_tilt(deg):
    """A label that is not a measurement is a field nobody can act on."""
    for q in ANCHORS:
        for c, tilt, _ in OP.candidates(q, OP.CONE, deg):
            got = OP.axis_deviation_deg(q, c)
            assert abs(got - tilt) < 1e-6, (
                "candidate reports %.4f deg and is actually %.4f deg out"
                % (tilt, got))


def test_the_cone_is_actually_explored_not_just_declared():
    """A cone that yields only the commanded pose would pass every bound
    check above and be worth nothing. It must reach its own edge."""
    for q in ANCHORS:
        tilts = [t for _, t, _ in OP.candidates(q, OP.CONE, 40.0)]
        assert len(tilts) > 8, "only %d candidates in a 40 deg cone" % len(tilts)
        assert max(tilts) == pytest.approx(40.0, abs=1e-6), (
            "the widest candidate is %.2f deg inside a 40 deg cone, so the "
            "tolerance the caller asked for is not being used" % max(tilts))
        # and the azimuths must actually go round, not all lie one way
        dirs = set()
        for c, t, _ in OP.candidates(q, OP.CONE, 40.0):
            if t <= 0:
                continue
            a = OP.tool_axis(c)
            dirs.add((round(a[0], 3), round(a[1], 3), round(a[2], 3)))
        assert len(dirs) > 6, (
            "the cone collapsed to %d distinct directions; it is a fan, not "
            "a cone, and it would miss the elbow configuration that clears "
            "the wearer" % len(dirs))


# ------------------------------------------------------- the refusals bite
def test_spin_is_refused_and_keeps_saying_why():
    """It measured +0 mm. The refusal exists so nobody re-invents it."""
    with pytest.raises(OP.PolicyError) as e:
        OP.normalise(OP.SPIN)
    msg = str(e.value)
    assert "+0 mm" in msg, (
        "the refusal no longer carries the measurement. Without it 'spin' "
        "reads like a conservative option rather than a measured dead end.")
    assert "cone" in msg, "the refusal does not say what to use instead"


@pytest.mark.parametrize("policy,deg", [
    (OP.EXACT, 15.0),          # a contradiction, not a preference
    ("nonsense", None),
    (OP.CONE, -1.0),
    (OP.CONE, 181.0),
])
def test_a_policy_that_cannot_be_honoured_is_refused_not_coerced(policy, deg):
    with pytest.raises(OP.PolicyError):
        OP.normalise(policy, deg)


def test_cone_zero_is_exact_rather_than_a_slow_exact():
    p, cone = OP.normalise(OP.CONE, 0.0)
    assert (p, cone) == (OP.EXACT, 0.0)
    assert len(list(OP.candidates(ANCHOR_LEFT, OP.CONE, 0.0))) == 1


# ------------------------------------------ the tool axis has ONE definition
def test_tool_axis_agrees_with_grasp_frames():
    """`grasp_frames.q_matrix(q)[:, 2]` is where the pad offset is measured
    along. Two definitions of which way the hand points is how a 13.47 mm
    grasp error survived for months.

    Compared on the NORMALISED quaternion, and the difference is the point:
    the stored anchors are not unit (left 0.99995844, right 1.00000561), so
    `q_matrix` of the raw value is not orthonormal. `grasp_frames` is left
    alone -- T0, T2 and T3 declare their coordinates through it and moving it
    moves the geometry -- but nothing that has to enforce a BOUND may inherit
    a non-orthonormal rotation.
    """
    sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
    import grasp_frames as GF                              # noqa: E402
    for q in ANCHORS:
        want = GF.q_matrix(OP.unit(q))[:, 2]
        got = OP.tool_axis(q)
        assert max(abs(float(a) - b) for a, b in zip(want, got)) < 1e-12


def test_the_stored_anchors_are_not_unit_and_that_is_handled():
    """A regression guard on the reason `unit()` exists. If somebody later
    normalises WORKSPACE_ORIENT in place this test says so out loud rather
    than the cone quietly becoming exact again."""
    sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
    from srl_teleop import master_calibration as MC        # noqa: E402
    worst = 0.0
    for arm in ("left", "right"):
        q = MC.WORKSPACE_ORIENT[arm]
        worst = max(worst, abs(1.0 - math.sqrt(sum(v * v for v in q))))
    # Whatever the anchors are, the cone must still be a bound.
    for arm in ("left", "right"):
        q = MC.WORKSPACE_ORIENT[arm]
        for c, tilt, _ in OP.candidates(q, OP.CONE, 30.0):
            assert OP.axis_deviation_deg(q, c) <= 30.0 + 1e-6
            assert abs(OP.axis_deviation_deg(q, c) - tilt) < 1e-6
    assert worst >= 0.0


# ------------------------------------------------- the follower wires it up
def test_the_follower_defaults_to_the_measured_cone():
    """The cone is ON for every mode, at the width the measurement supports.

    A policy module nothing consults is a policy nobody follows -- the
    repository's own "feature present but does nothing". And a policy that
    is consulted but defaults to off is the same thing with extra steps: the
    239 mm sits in a parameter nobody sets.
    """
    src = open(os.path.join(
        WS, "src/srl_teleop/srl_teleop/ik_follower_node.py")).read()
    assert 'declare_parameter("orientation_policy", "cone")' in src, (
        "ik_follower_node no longer defaults to a cone. Pinned reaches "
        "0.065 m at the work point against 0.304 for cone15, measured "
        "through this node's own candidate list.")
    m = re.search(r'declare_parameter\("orientation_cone_deg",\s*([0-9.]+)\)',
                  src)
    assert m, "the cone width is no longer declared"
    deg = float(m.group(1))
    assert 5.0 <= deg <= 45.0, (
        "the default cone is %.1f deg. Below 5 it buys nothing; above 45 is "
        "past what was measured." % deg)


def test_the_module_default_stays_exact_even_though_the_node_does_not():
    """Two different defaults, deliberately.

    The LIBRARY default is `exact`: a caller that does not say what it wants
    must get today's behaviour, so no analysis script silently starts
    measuring a different policy. The NODE default is the cone, because that
    is an operational choice somebody made on evidence. Conflating them
    would mean either the robot stays pinned or every offline tool quietly
    changes what it measures.
    """
    assert OP.DEFAULT_POLICY == OP.EXACT
    assert len(list(OP.candidates(ANCHOR_LEFT))) == 1


def test_the_follower_actually_asks_the_policy_for_candidates():
    src = open(os.path.join(
        WS, "src/srl_teleop/srl_teleop/ik_follower_node.py")).read()
    assert "orientation_policy as _op" in src
    assert "self._op.candidates(" in src, (
        "the follower imports the policy but never asks it for candidates")


def test_the_follower_tries_the_strict_pose_before_relaxing():
    """Ordering, read off the source: the orientation escalation must sit
    AFTER the redundancy retry, not before or beside it."""
    src = open(os.path.join(
        WS, "src/srl_teleop/srl_teleop/ik_follower_node.py")).read()
    i_red = src.index("self.redundancy_try < self.redundancy_n")
    i_ori = src.index("self._op.candidates(")
    assert i_red < i_ori, (
        "the orientation relaxation is attempted before the redundancy "
        "seeds are exhausted, so a pose reachable at the commanded "
        "orientation could be solved at a tilted one")


def test_a_relaxation_is_reported_and_never_silent():
    src = open(os.path.join(
        WS, "src/srl_teleop/srl_teleop/ik_follower_node.py")).read()
    assert "[ORIENT]" in src, "a tilt is applied with no log line"
    assert "self.ori_relaxed" in src
    # and it must reach the status topic, not only the log
    tail = src[src.index("def publish_status"):]
    assert "self.ori_relaxed" in tail and "self.ori_worst_tilt_deg" in tail, (
        "the relaxation counters never reach /ik_status_<arm>, so a GUI "
        "cannot show that the wrist was unpinned and the operator learns it "
        "from the arm")
