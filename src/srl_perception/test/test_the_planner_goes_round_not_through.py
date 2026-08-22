#!/usr/bin/env python3
"""The planner must go ROUND, and must never mark its own homework.

WHY THIS EXISTS. This project had no motion planner: `ik_follower_node`
solves one IK per commanded pose and clamps the joint step per cycle, which
is pose streaming with a rate limiter. For teleoperation that is right. For a
pick it is not, and the cost is measured -- a pregrasp and a grasp that are
both clear, streamed as a straight joint-space line, breach the wearer floor
at 0.1462 m at step 29 of 64. `safe_motion.check_path` was written to CATCH
that, and catching it means the run stops.

The properties below are the ones that make a planner safe to trust on a
wearable rig, and each is asserted rather than argued:

  * every edge of a RETURNED path is safe, densified, not just its nodes;
  * a clear move is a straight line, not a search that takes seconds;
  * an unsafe START and an unsafe GOAL are DIFFERENT refusals;
  * the continuous seam takes the short way -- defect 5 of 2026-08-21 would
    make the planner travel 340 degrees where 20 will do;
  * shortcutting never introduces a breach;
  * resampling never exceeds the follower's own step, or the follower
    invents the intermediate motion with an unchecked interpolation and the
    whole exercise is undone at the last step;
  * and an observed obstacle world actually blocks, and CANNOT be accepted
    and silently ignored.
"""
import math
import os
import sys

import numpy as np
import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
for p in ("scripts", "config", "src/srl_perception", "src/srl_teleop",
          "scripts/real_calibration"):
    sys.path.insert(0, os.path.join(WS, p))

from srl_perception import joint_planner as JP                # noqa: E402

LO = -np.ones(7) * 2.0
HI = np.ones(7) * 2.0
CONT = [False] * 7


def _slab(q):
    """Everything with |q0| < 0.35 and q1 < 0.5 is 'inside the wearer'."""
    q = np.asarray(q, float)
    return 0.0 if (abs(q[0]) < 0.35 and q[1] < 0.5) else 1.0


START = np.array([-1.0, 0.0, 0, 0, 0, 0, 0.0])
GOAL = np.array([1.0, 0.0, 0, 0, 0, 0, 0.0])


def test_the_module_self_test_passes():
    assert JP.self_test(verbose=False)


def test_the_straight_line_really_is_unsafe_or_this_proves_nothing():
    assert not JP.Checker(_slab, LO, HI, CONT).edge_ok(START, GOAL)


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_it_goes_round_and_every_edge_is_safe(seed):
    ch = JP.Checker(_slab, LO, HI, CONT)
    path, info = JP.plan(START, GOAL, ch, seed=seed)
    assert JP.dist(path[0], START) < 1e-9
    assert JP.dist(path[-1], GOAL) < 1e-9
    check = JP.Checker(_slab, LO, HI, CONT)
    assert all(check.edge_ok(a, b) for a, b in zip(path[:-1], path[1:])), (
        "a returned path has an unsafe edge (%s)" % info["method"])


def test_no_sample_on_the_final_path_is_unsafe():
    """The whole reason it densifies. A path whose NODES are clear and whose
    middle is not is the defect this planner replaces."""
    ch = JP.Checker(_slab, LO, HI, CONT)
    path, _ = JP.plan(START, GOAL, ch, seed=3)
    dense = JP.resample(path, step=0.02)
    bad = [q for q in dense if not JP.Checker(_slab, LO, HI, CONT).ok(q)]
    assert not bad, "%d of %d samples unsafe" % (len(bad), len(dense))


def test_a_clear_move_is_a_straight_line_not_a_search():
    ch = JP.Checker(lambda q: 1.0, LO, HI, CONT)
    path, info = JP.plan(START, GOAL, ch)
    assert info["method"] == "straight line" and len(path) == 2


def test_an_unsafe_start_and_an_unsafe_goal_are_different_refusals():
    inside = np.zeros(7)
    with pytest.raises(JP.PlanRefusal) as e1:
        JP.plan(inside, GOAL, JP.Checker(_slab, LO, HI, CONT))
    assert "START" in str(e1.value)
    with pytest.raises(JP.PlanRefusal) as e2:
        JP.plan(START, inside, JP.Checker(_slab, LO, HI, CONT))
    assert "GOAL" in str(e2.value)
    assert "not a planning failure" in str(e2.value), (
        "an unsafe goal is not the planner failing; it is the goal being "
        "inside the person, and the two need different actions")


def test_a_sealed_goal_is_refused_with_what_it_tried():
    def sealed(q):
        return 0.0 if float(np.asarray(q, float)[0]) > 0.5 else 1.0
    with pytest.raises(JP.PlanRefusal) as e:
        JP.plan(START, GOAL, JP.Checker(sealed, LO, HI, CONT), max_iter=200)
    assert "GOAL" in str(e.value) or "iterations" in str(e.value)


@pytest.mark.parametrize("a_deg,b_deg,short_deg", [
    (170.0, -170.0, 20.0), (-170.0, 170.0, 20.0), (179.0, -179.0, 2.0),
])
def test_the_continuous_seam_takes_the_short_way(a_deg, b_deg, short_deg):
    cont = [True] + [False] * 6
    ch = JP.Checker(lambda q: 1.0, -np.ones(7) * 4, np.ones(7) * 4, cont)
    a = np.array([math.radians(a_deg)] + [0.0] * 6)
    b = np.array([math.radians(b_deg)] + [0.0] * 6)
    path, _ = JP.plan(a, b, ch)
    assert abs(math.degrees(JP.path_length(path)) - short_deg) < 1e-6


def test_resampling_never_exceeds_the_followers_step():
    """The follower clamps to max_step_rad per cycle. Waypoints further
    apart than that mean it invents the motion between them with an
    unchecked straight interpolation."""
    ch = JP.Checker(_slab, LO, HI, CONT)
    path, _ = JP.plan(START, GOAL, ch, seed=2)
    for step in (0.05, 0.10, 0.35):
        rs = JP.resample(path, step=step)
        worst = max(float(np.abs(JP.delta(a, b)).max())
                    for a, b in zip(rs[:-1], rs[1:]))
        assert worst <= step + 1e-9, "step %.2f, worst %.4f" % (step, worst)
        assert JP.dist(rs[0], path[0]) < 1e-9
        assert JP.dist(rs[-1], path[-1]) < 1e-9


# ------------------------------------------------- the observed world
def _one_d_links(q):
    return [np.array([float(np.asarray(q, float)[0]), 0.0, 0.0])]


def _wall():
    return np.array([[0.0, y * 0.01, z * 0.01]
                     for y in range(-8, 9) for z in range(-8, 9)])


def test_the_observed_world_actually_blocks():
    """Before this the planner's ONLY obstacle was the wearer, so a path
    that went round a person would sweep the arm through the bench."""
    w = JP.VoxelWorld(_wall(), voxel_m=0.03)
    assert len(w) > 0
    free = JP.Checker(lambda q: 1.0, LO, HI, CONT)
    blocked = JP.Checker(lambda q: 1.0, LO, HI, CONT, world=w,
                         link_points=_one_d_links, tube_r=0.05)
    a = np.array([-1.0] + [0.0] * 6)
    b = np.array([1.0] + [0.0] * 6)
    assert free.edge_ok(a, b), "without a world the wall must be invisible"
    assert not blocked.edge_ok(a, b), "with a world it must block"
    assert blocked.last_hit is not None
    assert abs(float(blocked.last_hit[0])) < 0.10, (
        "the refusal must say WHERE, and it said %s" % blocked.last_hit)


def test_a_world_that_cannot_be_checked_is_refused_not_ignored():
    """Accepting obstacles and not using them is a planner that reports
    avoidance it is not doing."""
    with pytest.raises(ValueError):
        JP.Checker(lambda q: 1.0, LO, HI, CONT,
                   world=JP.VoxelWorld(_wall()))


def test_an_empty_world_blocks_nothing():
    ch = JP.Checker(lambda q: 1.0, LO, HI, CONT,
                    world=JP.VoxelWorld(np.zeros((0, 3))),
                    link_points=_one_d_links)
    assert ch.edge_ok(np.array([-1.0] + [0.0] * 6),
                      np.array([1.0] + [0.0] * 6))


def test_the_self_filter_keeps_the_gripper_out_of_the_obstacle_set():
    """The wrist camera sees the gripper. Voxelising it puts a permanent
    obstacle exactly where the hand is and nothing can ever be planned."""
    near = np.array([[0.02, 0.0, 0.0], [0.5, 0.0, 0.0]])
    w = JP.VoxelWorld(near, voxel_m=0.03, ignore_within_m=0.10,
                      origin=[0.0, 0.0, 0.0])
    assert w.n_points == 1
    assert not w.occupied([0.02, 0.0, 0.0])
    assert w.occupied([0.5, 0.0, 0.0])
