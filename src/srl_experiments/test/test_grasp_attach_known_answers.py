#!/usr/bin/env python3
"""Known answers for the grasp attach test and the single-owner gripper.

STANDING RULE (docs/ENGINEERING_LOG.md): validate the instrument before believing it. Both
defects these tests pin were invisible to inspection and visible only in a
recorded trace, so they get a test that fails on the old behaviour.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))), "scripts"))


def _mod():
    import importlib
    return importlib.import_module("record_rviz")


def test_grip_for_is_monotonic_and_bounded():
    """A wider object must leave the fingers further open."""
    m = _mod()
    assert m.grip_for(85) < m.grip_for(40) < m.grip_for(10)
    for w in (0, 10, 40, 85, 200):
        assert 0.12 <= m.grip_for(w) <= 0.70


def test_a_barely_closed_gripper_is_not_holding_a_40mm_block():
    """The defect: 0.157 attached a block that needs 0.424.

    This is the assertion that fails on the old band-only holding().
    """
    m = _mod()
    assert not m.holding(0.157, 40)
    assert not m.holding(m.GRIP_OPEN, 40)


def test_fingers_at_the_object_width_are_holding():
    m = _mod()
    target = m.grip_for(40)
    assert m.holding(target, 40)
    assert m.holding(0.95 * target, 40), "must tolerate tracking lag"


def test_release_is_detected_once_the_fingers_leave_the_width():
    m = _mod()
    assert not m.holding(0.5 * m.grip_for(40), 40)


def test_a_narrow_object_needs_a_tighter_close_than_a_wide_one():
    """A knuckle that holds a wide object need not hold a narrow one."""
    m = _mod()
    wide = m.grip_for(60)
    assert m.holding(wide, 60)
    assert not m.holding(wide, 10)


def test_band_form_still_works_where_no_width_is_known():
    """t7-style tasks hold nothing and pass no width; keep that path intact."""
    m = _mod()
    assert m.holding(0.40) and not m.holding(0.05) and not m.holding(0.80)
    assert not m.holding(None) and not m.holding(float("nan"))


def test_release_fraction_is_defined_only_for_tasks_that_place():
    """A carry task must never hand the gripper back mid-transit."""
    m = _mod()
    assert set(m.RELEASE_FRAC) == {"t2", "t5"}
    for t in ("t3", "t6"):
        assert m.RELEASE_FRAC.get(t, 2.0) > 1.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
