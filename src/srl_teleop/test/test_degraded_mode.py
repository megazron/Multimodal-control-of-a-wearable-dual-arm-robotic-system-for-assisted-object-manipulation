#!/usr/bin/env python3
"""Degraded mode: freeze what is broken, and never invent a value.

The properties pinned here are the ones whose violation would be silent.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_teleop import degraded_mode as dg          # noqa: E402

BASE = {
    "l_j1": {"verdict": "INTERMITTENT"}, "l_j2": {"verdict": "INCOHERENT"},
    "l_j3": {"verdict": "INCOHERENT"}, "l_j4": {"verdict": "INCOHERENT"},
    "l_j5": {"verdict": "INCOHERENT"}, "l_j6": {"verdict": "INCOHERENT"},
    "l_j7": {"verdict": "ALIVE"},
    "r_j1": {"verdict": "ALIVE"}, "r_j2": {"verdict": "ALIVE"},
    "r_j3": {"verdict": "INCOHERENT"}, "r_j4": {"verdict": "ALIVE"},
    "r_j5": {"verdict": "DEAD"}, "r_j6": {"verdict": "ALIVE"},
    "r_j7": {"verdict": "DEAD"},
}
HEALTHY = {"%s_j%d" % (a, i + 1): {"verdict": "ALIVE"}
           for a in ("l", "r") for i in range(7)}


def test_counts_the_measured_baseline():
    assert dg.n_coherent(BASE) == 6           # l_j1, l_j7, r_j1/2/4/6


def test_intermittent_is_usable_but_incoherent_is_not():
    """An INTERMITTENT channel drops out -- a KNOWN loss the validator
    already rejects. An INCOHERENT one returns plausible wrong values, which
    nothing downstream can detect. Only the second must be frozen."""
    exc = dg.excluded_channels(BASE)
    assert 0 not in exc["left"]               # l_j1 INTERMITTENT -> kept
    assert 1 in exc["left"]                   # l_j2 INCOHERENT   -> frozen
    assert 4 in exc["right"]                  # r_j5 DEAD         -> frozen


def test_auto_engages_below_threshold_and_clears_above():
    on, _, _ = dg.decide(BASE, "auto", min_coherent=12)
    assert on is True
    off, exc, _ = dg.decide(HEALTHY, "auto", min_coherent=12)
    assert off is False
    assert exc == {"left": [], "right": []}


def test_unknown_health_is_not_treated_as_healthy():
    """An absent baseline must not read as an all-clear -- that is the same
    error as treating an EXPIRED BlockMonitor entry as clear."""
    on, exc, why = dg.decide({}, "auto")
    assert on is True
    assert "NO channel baseline" in why
    # ...but it must not freeze channels it has no evidence against either.
    assert exc == {"left": [], "right": []}


def test_off_never_freezes_anything():
    on, exc, _ = dg.decide(BASE, "off")
    assert on is False and exc == {"left": [], "right": []}


def test_left_loses_reach_and_right_does_not():
    """The measured cost. Left j2/j4 are the reach pots and both are
    incoherent, so the left commanded set collapses to a spherical shell.
    Right freezes only j3/j5/j7, none of which spherical reach depends on
    materially (j3's measured contribution is 4.5 mm)."""
    exc = dg.excluded_channels(BASE)
    assert "reach FROZEN" in dg.position_capability("left", exc["left"])
    assert "reach LIVE" in dg.position_capability("right", exc["right"])
    # azimuth survives on both -- j1 is coherent on both arms
    assert "azimuth LIVE" in dg.position_capability("left", exc["left"])


def test_banner_names_every_frozen_channel():
    on, exc, why = dg.decide(BASE, "auto")
    txt = dg.banner(BASE, on, exc, why)
    for i in exc["left"]:
        assert "j%d" % (i + 1) in txt
    assert "DEGRADED MODE ACTIVE" in txt
    assert "check_channels.sh" in txt         # the recovery path is stated


@pytest.mark.parametrize("mode", ["auto", "on", "off"])
def test_decide_is_total(mode):
    on, exc, why = dg.decide(BASE, mode)
    assert isinstance(on, bool) and why
    assert set(exc) == {"left", "right"}
