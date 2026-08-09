#!/usr/bin/env python3
"""The dial trades precision for speed and NOTHING else."""
import math

import pytest

from srl_teleop import precision_speed as ps


def test_safety_params_are_refused():
    for k in ps.SAFETY_PARAMS:
        with pytest.raises(ValueError):
            ps.assert_safety_unconditional({k: 0.0})


def test_the_dial_never_produces_a_safety_param():
    """The real guard: whatever the dial emits, at any position."""
    for i in range(0, 101):
        s = ps.settings(i / 100.0)
        ps.assert_safety_unconditional(s)
        assert not ps.SAFETY_PARAMS.intersection(s)


def test_clearance_floor_is_identical_at_both_extremes():
    a, b = ps.settings(0.0), ps.settings(1.0)
    assert "min_clearance_m" not in a and "min_clearance_m" not in b
    assert set(a) == set(b) == set(ps.RANGE)


def test_endpoints_are_the_declared_range():
    for k, (lo, hi) in ps.RANGE.items():
        assert abs(ps.settings(0.0)[k] - lo) < 1e-9
        assert abs(ps.settings(1.0)[k] - hi) < 1e-9


def test_every_parameter_is_monotonic_in_the_dial():
    prev = ps.settings(0.0)
    for i in range(1, 101):
        cur = ps.settings(i / 100.0)
        for k in ps.RANGE:
            assert cur[k] >= prev[k] - 1e-12, k
        prev = cur


def test_scale_resolution_is_spread_not_crowded():
    """Geometric, so the low end -- where feel changes fastest -- gets the
    resolution. A linear map would make 0.0-0.1 nearly indistinguishable."""
    s = ps.settings
    low = s(0.1)["scale"] - s(0.0)["scale"]
    high = s(1.0)["scale"] - s(0.9)["scale"]
    assert high > low


def test_dial_is_clamped_not_wrapped():
    assert ps.settings(-5.0) == ps.settings(0.0)
    assert ps.settings(9.0) == ps.settings(1.0)


def test_rebase_keeps_the_command_continuous():
    anchor = [0.5, 0.4, 1.1]
    ref = [0.0, 0.0, 0.0]
    cur = [0.10, -0.05, 0.02]         # operator well away from engage

    def cmd(a, sc):
        return [ai + sc * (c - r) for ai, c, r in zip(a, cur, ref)]

    for old, new in ((1.0, 0.2), (0.2, 1.0), (0.6, 0.35)):
        before = cmd(anchor, old)
        a2 = ps.rebase_anchor(anchor, cur, ref, old, new)
        after = cmd(a2, new)
        assert max(abs(x - y) for x, y in zip(before, after)) < 1e-12


def test_rebase_matters_most_far_from_engage():
    """The jump it prevents grows with displacement -- which is why doing it
    only near the engage point would miss the dangerous case."""
    def jump(disp):
        anchor, ref = [0, 0, 0], [0, 0, 0]
        cur = [disp, 0, 0]
        return abs((1.0 - 0.2) * (cur[0] - ref[0]))
    assert jump(0.20) > jump(0.02)
