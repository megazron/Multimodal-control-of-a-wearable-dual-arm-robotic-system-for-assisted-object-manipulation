"""The orientation sweep's monotonicity control, after it was relaxed.

WHAT WAS RELAXED, AND WHY. A tighter orientation policy admits a subset of
the poses a looser one admits, so a looser policy can never reach less far.
The sweep asserts that per walk. It failed on one walk -- `right -1-1+1,
pinned 0.900 > spin 0.875` -- and the failure reproduced IDENTICALLY at ten
and at twenty random seeds, which is what proves it was never a sampling
problem: 0.900 is the search CAP, so the pinned walk ran to the end of the
search without ever being refused and its reach is a LOWER BOUND, not a
boundary. Comparing a lower bound against a boundary is not a monotonicity
test.

Capped comparisons are therefore excluded. A relaxed control is exactly the
kind of change that quietly stops being a control, so this pins both halves:
it must still FAIL on a real violation, and it must only skip when the
TIGHTER policy is the capped one.
"""
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))

np = pytest.importorskip("numpy")
import measure_orientation_cost as moc          # noqa: E402

POLS = ["pinned", "spin", "cone15", "cone45", "free"]


def _row(reach, why=None):
    r = dict(zip(POLS, reach))
    w = dict(zip(POLS, why or ["UNRE"] * len(POLS)))
    return r, w


def test_a_monotone_walk_reports_nothing():
    r, w = _row([0.40, 0.42, 0.45, 0.48, 0.50])
    assert list(moc.monotonicity(r, w, POLS)) == []


def test_a_real_violation_still_fails():
    """The half that matters. Nothing here is capped, so a looser policy
    reaching less far is the arithmetic impossibility the control exists
    for."""
    r, w = _row([0.50, 0.42, 0.45, 0.48, 0.50])
    out = list(moc.monotonicity(r, w, POLS))
    assert len(out) == 1
    kind, tight, loose, rt, rl = out[0]
    assert kind == "fail"
    assert (tight, loose) == ("pinned", "spin")
    assert rt > rl


def test_a_capped_tighter_policy_is_skipped_not_failed():
    """The exact case measured on the rig: pinned ran to the cap."""
    r, w = _row([0.900, 0.875, 0.90, 0.90, 0.90],
                ["CAP", "UNRE", "CAP", "CAP", "CAP"])
    out = list(moc.monotonicity(r, w, POLS))
    assert [o[0] for o in out] == ["skip"]
    assert out[0][1:3] == ("pinned", "spin")


def test_a_capped_LOOSER_policy_does_not_excuse_a_violation():
    """The asymmetry is the whole point. Only the TIGHTER side being a lower
    bound makes the comparison meaningless; if the looser one is capped and
    still reached less far, that is a genuine violation and must fail."""
    r, w = _row([0.50, 0.42, 0.45, 0.48, 0.50],
                ["UNRE", "CAP", "UNRE", "UNRE", "UNRE"])
    out = list(moc.monotonicity(r, w, POLS))
    assert [o[0] for o in out] == ["fail"]


def test_the_exclusion_cannot_swallow_every_comparison():
    """A control that skips everything is not a control. With every walk
    capped and a violation present, the skips must be exactly the capped
    pairs and no more."""
    r, w = _row([0.90, 0.80, 0.70, 0.60, 0.50], ["CAP"] * 5)
    out = list(moc.monotonicity(r, w, POLS))
    assert len(out) == 4 and {o[0] for o in out} == {"skip"}
    # and with none capped, the same reaches are four hard failures
    r2, w2 = _row([0.90, 0.80, 0.70, 0.60, 0.50])
    out2 = list(moc.monotonicity(r2, w2, POLS))
    assert len(out2) == 4 and {o[0] for o in out2} == {"fail"}
