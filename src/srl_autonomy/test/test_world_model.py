#!/usr/bin/env python3
"""World model: persistence, re-verification, and refusing to trust memory."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_autonomy.world_model import (                    # noqa: E402
    CONFLICT, FRESH, STALE, WorldModel, scan_waypoints)


class Clk:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t
    def advance(self, d): self.t += d


def wm(**kw):
    c = Clk()
    return WorldModel(clock=c, **kw), c


def det(label, xyz, conf=0.9):
    return {"label": label, "position": list(xyz), "confidence": conf}


def test_objects_persist_when_out_of_view():
    """The whole point of an eye-in-hand world model."""
    m, c = wm()
    m.observe([det("green cube", [0.2, 0.3, 0.9])])
    c.advance(30.0)
    assert len(m.objects) == 1
    assert m.state(1, expected_visible=False) == STALE


def test_low_confidence_never_enters_the_model():
    m, _ = wm(min_confidence=0.5)
    m.observe([det("ghost", [0, 0.3, 0.9], conf=0.2)])
    assert not m.objects
    assert any(e[0] == "reject_low_conf" for e in m.log)


def test_repeat_detections_merge_not_duplicate():
    m, _ = wm()
    m.observe([det("cube", [0.20, 0.30, 0.90])])
    m.observe([det("cube", [0.21, 0.30, 0.90])])
    assert len(m.objects) == 1


def test_distinct_objects_stay_distinct():
    m, _ = wm()
    m.observe([det("cube", [0.20, 0.30, 0.90])])
    m.observe([det("cube", [0.50, 0.30, 0.90])])
    assert len(m.objects) == 2


def test_expected_but_unseen_is_a_CONFLICT_then_DROPPED():
    """An object that is gone must not remain in the model: a remembered
    pose is as precise, and as wrong, as a live one."""
    m, c = wm(max_misses=2)
    m.observe([det("cube", [0.2, 0.3, 0.9])])
    c.advance(5.0)
    dropped, conflicts = m.reverify(seen_ids=[], expected_ids=[1])
    assert conflicts == [1] and not dropped
    assert m.state(1, expected_visible=True) == CONFLICT
    dropped, _ = m.reverify(seen_ids=[], expected_ids=[1])
    assert dropped and 1 not in m.objects


def test_seen_again_clears_the_misses():
    m, c = wm(max_misses=2)
    m.observe([det("cube", [0.2, 0.3, 0.9])])
    m.reverify(seen_ids=[], expected_ids=[1])
    m.reverify(seen_ids=[1], expected_ids=[1])
    assert m.objects[1].misses == 0
    assert m.state(1, expected_visible=True) == FRESH


def test_not_expected_is_not_penalised():
    """Looking away must not delete the world."""
    m, c = wm(max_misses=1)
    m.observe([det("cube", [0.2, 0.3, 0.9])])
    for _ in range(10):
        m.reverify(seen_ids=[], expected_ids=[])
    assert 1 in m.objects


def test_dual_arm_observer_is_the_other_arm():
    assert WorldModel.observer_arm("left") == "right"
    assert WorldModel.observer_arm("right") == "left"


def test_frustum_respects_direction_and_range():
    f = WorldModel.expected_visible
    cam = (0.0, 0.0, 1.0)
    fwd = (0.0, 1.0, 0.0)
    assert f(cam, fwd, (0.0, 0.5, 1.0)) is True          # straight ahead
    assert f(cam, fwd, (0.0, -0.5, 1.0)) is False        # behind
    assert f(cam, fwd, (0.0, 5.0, 1.0)) is False         # too far
    assert f(cam, fwd, (0.0, 0.02, 1.0)) is False        # too close
    assert f(cam, fwd, (0.9, 0.5, 1.0)) is False         # outside the fov


def test_scan_waypoints_are_in_front_and_differ_per_arm():
    L = scan_waypoints("left")
    R = scan_waypoints("right")
    assert len(L) == 5 and all(p[1] > 0 for p in L)      # +y is FORWARD
    assert np.sign(L[0][0]) != np.sign(R[0][0])


def test_match_requires_every_content_word():
    m, _ = wm()
    m.observe([det("green cube", [0.2, 0.3, 0.9]),
               det("red cube", [0.4, 0.3, 0.9])])
    assert len(m.match("green cube")) == 1
    assert len(m.match("cube")) == 2
    assert len(m.match("blue cube")) == 0
