#!/usr/bin/env python3
"""
test_ik_guard.py — the guard must converge, not deadlock.

Regression test for the failure that kept the arm frozen at home: the step
guard discarded every IK solution that was further than max_step_rad from
the current state. Because a discarded solution means the arm does not move,
the next solution was exactly as far away, forever. It showed up in the logs
as reject_count == success_count with a 100% IK success rate.

No ROS required -- the decision logic is pure functions in ik_follower_node.
"""
import math

from srl_teleop.ik_follower_node import (
    CONTINUOUS_IDX, clamp_towards, joint_steps, pose_delta, quat_angle,
    wrap_continuous, wrap_pi,
)

MAX_STEP = 0.35


def _sim_converge(start, target, max_step=MAX_STEP, budget=None):
    """Drive clamp_towards repeatedly, as the node does cycle to cycle."""
    if budget is None:
        budget = 4 * int(math.ceil(2.0 / max_step))
    cur = list(start)
    for n in range(1, budget + 1):
        cur = clamp_towards(target, wrap_continuous(cur), max_step)
        if max(abs(s) for s in joint_steps(target, wrap_continuous(cur))) < 1e-9:
            return n, cur
    return None, cur


def test_converges_from_2_rad():
    """2.0 rad away must converge in ceil(2.0/max_step) cycles, not never."""
    target = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    start = [0.0] * 7
    expected = int(math.ceil(2.0 / MAX_STEP))          # 6
    n, final = _sim_converge(start, target)
    assert n is not None, "guard never converged -- the deadlock is back"
    assert n <= expected, f"took {n} cycles, expected <= {expected}"
    assert abs(final[0] - 2.0) < 1e-9, final


def test_every_step_is_bounded():
    """A clamped command must never exceed max_step in one cycle."""
    target = [2.0, -1.5, 2.5, 1.0, -2.0, 0.9, 3.0]
    cur = [0.0] * 7
    for _ in range(int(math.ceil(2.5 / MAX_STEP)) + 2):
        nxt = clamp_towards(target, wrap_continuous(cur), MAX_STEP)
        for i, (a, b) in enumerate(zip(nxt, wrap_continuous(cur))):
            d = abs(wrap_pi(a - b)) if i in CONTINUOUS_IDX else abs(a - b)
            assert d <= MAX_STEP + 1e-12, f"joint {i} stepped {d}"
        cur = nxt


def test_converges_across_the_pi_seam():
    """A short move across +/-pi must not be inflated into a 2pi traverse."""
    target = [wrap_pi(math.pi - 0.05)] + [0.0] * 6      # just below +pi
    start = [wrap_pi(-math.pi + 0.05)] + [0.0] * 6      # just above -pi
    n, final = _sim_converge(start, target)
    assert n is not None and n <= 1, f"seam crossing took {n} cycles"
    assert abs(wrap_pi(final[0] - target[0])) < 1e-9


def test_limited_joints_are_not_wrapped():
    """Only 1/3/5/7 are continuous; the others must not be wrapped."""
    target = [0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0]
    cur = [0.0] * 7
    for _ in range(20):
        cur = clamp_towards(target, wrap_continuous(cur), MAX_STEP)
    for i in (1, 3, 5):
        assert abs(cur[i] - 3.0) < 1e-9, f"joint {i} = {cur[i]}, wanted 3.0"


def test_pose_delta_and_flip_discrimination():
    """The deadband must separate 'target moved' from 'solution flipped'."""
    a = ((0.30, 0.0, 1.20), (0.0, 0.0, 0.0, 1.0))
    same = ((0.30, 0.0, 1.20), (0.0, 0.0, 0.0, 1.0))
    moved = ((0.35, 0.0, 1.20), (0.0, 0.0, 0.0, 1.0))

    d_m, d_rad = pose_delta(a, same)
    assert d_m == 0.0 and d_rad < 1e-6

    d_m, _ = pose_delta(a, moved)
    assert abs(d_m - 0.05) < 1e-9

    # unseeded first cycle must read as "moved", so nothing is ever
    # classified as a flip before a baseline exists
    assert pose_delta(a, None) == (float("inf"), float("inf"))

    assert quat_angle((0, 0, 0, 1), (0, 0, 0, 1)) < 1e-9
    assert abs(quat_angle((0, 0, 0, 1),
                          (0, 0, math.sin(0.25), math.cos(0.25))) - 0.5) < 1e-9


def test_the_original_deadlock_scenario():
    """The exact observed failure: static target 2.5 rad from a parked arm.

    Old behaviour rejected all of these and the arm never moved. The guard
    must now commit bounded steps and arrive.
    """
    target = [0.0, 0.0, 0.0, 0.0, 0.0, -2.5, 0.0]      # j6, a limited joint
    cur = [0.0] * 7
    published = 0
    for _ in range(int(math.ceil(2.5 / MAX_STEP)) + 1):
        steps = joint_steps(target, wrap_continuous(cur))
        cur = clamp_towards(target, wrap_continuous(cur), MAX_STEP)
        published += 1
        if max(abs(s) for s in steps) <= 1e-9:
            break
    assert published > 1
    assert abs(cur[5] - (-2.5)) < 1e-9, cur[5]


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS %s" % name)
            except AssertionError as e:
                fails += 1
                print("FAIL %s: %s" % (name, e))
    raise SystemExit(1 if fails else 0)
