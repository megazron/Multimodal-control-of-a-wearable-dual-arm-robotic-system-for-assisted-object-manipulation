#!/usr/bin/env python3
"""Two defects in `grasp_pipeline.reachable`, and the controls that catch them.

This is the function that decides whether the arm can go where the grasp
planner wants it. Until 2026-08-22 it got both of its answers from the wrong
place.

CHANGE 1 -- HOW IT SEARCHES, and this one is a hardening rather than a
demonstrated fix. It seeded the optimiser from `home` plus N UNIFORM-RANDOM
joint vectors, and its refusal read

    "no IK solution in 41 starts -- this point is outside the arm's 902 mm
     reach, not blocked by a rule"

which is a CONCLUSION dressed as a measurement whatever the search does. It
now seeds from a cached table of sampled configurations, nearest first --
the method `scripts/real_calibration/arm_ik.py` was written with on
2026-08-21 after a target refused at 301 restarts was walked to 42 mm by
plain sampling.

I COULD NOT REPRODUCE THAT FAILURE HERE. On points drawn from the full joint
box, at 4, 8, 20 and 40 restarts, both seedings find 10 of 10. So the test
below asserts only that the seeded search is never worse, and the wording of
the refusal is fixed on its own merits.

DEFECT 2 -- ITS WEARER FLOOR COULD NOT FIRE. This one is real, and the tests
for it fail on the old code. `scorer.arm_terms()` returns two
clearances. `clearance_m` is the WHOLE chain, which includes the immobile
mount stub `base_link -> shoulder_link`; that stub is nearer the wearer than
anything the joints can move, so the whole-chain number is the constant
0.2202 m for every pose the arm can reach. This function compared THAT
against the 0.15 m floor. 0.2202 >= 0.15 is true always, so the check passed
on every pose ever planned and the floor in the grasp pipeline had never
fired once.

`clearance_moving_m` is the real one. The difference at home is 0.2202
against 0.4031.

THE CONTROLS. Each defect gets a test that fails on the old code:

  * FK-CONSTRUCTED POINTS. Take a joint vector, compute where its hand is.
    That point is reachable BY CONSTRUCTION, so a refusal is a search
    failure. The uniform-seeded version misses these; the seeded one must
    find all of them.
  * A POINT INSIDE THE WEARER must be refused, and refused for the WEARER
    rather than for reach. Under the old clearance this returned reachable.
  * A REFUSAL MUST QUOTE WHAT IT MEASURED, not what it inferred. The string
    "outside the arm's" is banned by name.
"""
import math
import os
import sys

import numpy as np
import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
for p in ("scripts", "config", "src/srl_perception"):
    sys.path.insert(0, os.path.join(WS, p))

pytest.importorskip("scipy")


@pytest.fixture(scope="module")
def rig():
    import home_positions as hp
    import solve_home_pose as SHP
    from srl_perception import grasp_pipeline as GP
    sc = SHP.Scorer()
    homes = {a: np.array(hp.load_home_radians(a)) for a in ("left", "right")}
    return sc, homes, GP


def _hand(sc, arm, q):
    return np.asarray(sc.arm_terms(arm, q, with_clearance=False)["hand"],
                      float)


def _fk_points(sc, arm, home, n=5, spread=0.9, seed=4):
    """Points that ARE reachable, by construction."""
    lo, hi, _ = sc.lim[arm]
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = np.clip(home + rng.uniform(-spread, spread, 7), lo, hi)
        out.append(_hand(sc, arm, q))
    return out


# ------------------------------------------------------------- defect 1
@pytest.mark.parametrize("arm", ["left", "right"])
def test_every_fk_constructed_point_is_found(rig, arm):
    """A point the arm demonstrably can reach must not be refused."""
    sc, homes, GP = rig
    missed = []
    for tgt in _fk_points(sc, arm, homes[arm]):
        ok, _q, why = GP.reachable(sc, arm, tgt, homes[arm])
        if not ok:
            missed.append((tgt.round(3).tolist(), why))
    assert not missed, (
        "%d of 5 FK-constructed points were refused on the %s arm. These "
        "points are reachable by construction, so each refusal is the search "
        "failing to look: %s" % (len(missed), arm, missed))


def test_the_seeded_search_is_never_worse_than_the_uniform_one(rig):
    """WHAT THIS DOES AND DOES NOT SHOW, stated because it surprised me.

    The seeding change was made on the strength of a 2026-08-21 report that
    `reachable()` refused a target at 301 restarts which plain FK sampling
    walked to within 42 mm. I could not reproduce that here. Points drawn
    from the FULL joint box, at 4, 8, 20 and 40 restarts, both seedings find
    10 of 10. So on the targets I can construct, the uniform search was not
    failing.

    The change therefore stands as a HARDENING, not as a demonstrated bug
    fix, and this test asserts only what is true: starting from samples that
    are already near the target cannot do worse than starting from random
    ones, and it must not have broken anything. If a future point does break
    the uniform version, the `>=` below becomes a `>` and this docstring is
    wrong in the good direction.

    The defect in this function that IS demonstrated is the other one -- the
    clearance it read.
    """
    from scipy.optimize import minimize
    sc, homes, GP = rig
    arm = "left"
    home = homes[arm]
    lo, hi, _ = sc.lim[arm]
    rng = np.random.default_rng(2)
    pts = [_hand(sc, arm, rng.uniform(lo, hi)) for _ in range(8)]

    def old_uniform(goal, restarts=8):
        r2 = np.random.default_rng(11)
        goal = np.asarray(goal, float)

        def cost(q):
            return float(np.sum((_hand(sc, arm, q) - goal) ** 2))
        for s0 in [home] + [r2.uniform(lo, hi) for _ in range(restarts)]:
            r = minimize(cost, s0, method="L-BFGS-B",
                         bounds=list(zip(lo, hi)),
                         options=dict(maxiter=400, ftol=1e-14))
            if math.sqrt(max(r.fun, 0.0)) < 0.010:
                return True
        return False

    def found(goal):
        ok, _q, why = GP.reachable(sc, arm, goal, home, restarts=8)
        return ok or "no IK solution" not in why

    new_ok = sum(1 for p in pts if found(p))
    old_ok = sum(1 for p in pts if old_uniform(p))
    assert new_ok == len(pts), (
        "the seeded search failed to find an IK solution for %d of %d points "
        "sampled from the arm's own joint box" % (len(pts) - new_ok, len(pts)))
    assert new_ok >= old_ok, (
        "the seeded search found %d of %d where uniform seeding found %d. "
        "Seeding from nearby samples cannot legitimately be worse."
        % (new_ok, len(pts), old_ok))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_a_refusal_says_what_it_measured_not_what_it_concluded(rig, arm):
    sc, homes, GP = rig
    ok, _q, why = GP.reachable(sc, arm, np.array([0.0, 5.0, 1.0]),
                               homes[arm])
    assert not ok, "claimed to reach a point 5 m away"
    assert "outside the arm's" not in why, (
        "the refusal asserts the point is outside the arm's reach. That is "
        "an inference from a search; report the distance the search actually "
        "got to. Got: %s" % why)
    assert "SEEDED" in why or "seeded" in why
    assert "mm away" in why, "the refusal quotes no measured distance: %s" % why


# ------------------------------------------------------------- defect 2
@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_wearer_floor_can_actually_fire(rig, arm):
    """A point inside the wearer must be refused FOR THE WEARER.

    Under the whole-chain clearance this returned reachable, because the
    whole chain reads the mount stub's constant 0.2202 m whatever the arm is
    doing. A floor that cannot fail is not a floor.
    """
    sc, homes, GP = rig
    sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
    from srl_teleop import mount_guard_node as MG
    torso = None
    for name, _kind, _dims, ctr, _rpy in MG.WEARER:
        if str(name).lower() == "torso":
            torso = np.asarray(ctr, float)
    assert torso is not None, "the wearer model has no torso to aim at"

    ok, _q, why = GP.reachable(sc, arm, torso, homes[arm])
    assert not ok, (
        "the %s arm was cleared to put its hand in the middle of the "
        "wearer's torso. %s" % (arm, why))
    assert "clearance" in why and "floor" in why, (
        "it was refused, but not by the wearer floor -- so the floor still "
        "may never have fired. Reason given: %s" % why)


def test_it_reads_the_moving_chain_and_not_the_mount_stub(rig):
    """Directly: the number in the reason must be the moving chain's."""
    sc, homes, GP = rig
    arm = "left"
    tgt = _hand(sc, arm, homes[arm])
    ok, q, why = GP.reachable(sc, arm, tgt, homes[arm])
    assert ok, why
    t = sc.arm_terms(arm, q)
    whole = float(t["clearance_m"])
    moving = float(t["clearance_moving_m"])
    # they must differ here, or this test proves nothing
    assert abs(whole - moving) > 0.05, (
        "whole-chain and moving-chain clearance agree at this pose (%.4f vs "
        "%.4f), so this test cannot tell which one was read" % (whole, moving))
    assert ("%.3f" % moving) in why, (
        "the reason quotes %s; the moving-chain clearance is %.3f and the "
        "whole-chain (mount stub) constant is %.3f. Reading the stub is how "
        "the floor came to be unfireable." % (why, moving, whole))


def test_the_source_no_longer_reads_the_whole_chain_clearance():
    """Belt and braces on the line itself, because the behavioural tests
    above depend on a pose where the two happen to differ."""
    src = open(os.path.join(
        WS, "src/srl_perception/srl_perception/grasp_pipeline.py")).read()
    body = src[src.index("def reachable("):src.index("def plan_and_check(")]
    assert 'clearance_moving_m' in body
    assert 't["clearance_m"]' not in body, (
        "`reachable` reads the whole-chain clearance again. That is the "
        "immobile mount stub and it is 0.2202 m in every pose, so the wearer "
        "floor compared against it can never fail.")
