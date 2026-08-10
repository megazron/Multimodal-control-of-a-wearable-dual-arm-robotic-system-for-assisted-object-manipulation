#!/usr/bin/env python3
"""The one property that matters: NOT CHECKED can never read as HEALTHY.

This project has been bitten by silent acceptance three times in a week -- a
volatile QoS, a misplaced RViz config key, a missing mesh install rule -- and
in every case something absent looked exactly like something fine. These tests
exist so that cannot happen to the go/no-go indicator a participant session
depends on.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from srl_experiments import readiness as R                      # noqa: E402


class Probe:
    """A probe that answers OK to everything, with holes punched into it."""

    def __init__(self, missing=(), failing=(), raising=()):
        self.missing, self.failing, self.raising = (set(missing), set(failing),
                                                    set(raising))

    def _a(self, name):
        if name in self.raising:
            raise RuntimeError("sensor exploded")
        if name in self.failing:
            return R.FAIL, "broken"
        if name in self.missing:
            return R.UNKNOWN, "no data"
        return R.OK, "fine"

    def __getattr__(self, name):
        if name in R.REQUIRED:
            return lambda: self._a(name)
        raise AttributeError(name)


def test_all_ok_is_ready():
    r = R.evaluate(Probe())
    assert r.ready
    assert r.summary() == "READY"


def test_a_single_unknown_blocks():
    """An UNKNOWN is exactly as blocking as a FAIL. This is the rule."""
    for name in R.REQUIRED:
        r = R.evaluate(Probe(missing=[name]))
        assert not r.ready, "%s UNKNOWN did not block" % name
        assert name in [c.name for c in r.unknown]


def test_unknown_and_fail_render_differently():
    """A red light says fix it; a grey light says the instrument is not
    looking. Conflating them is how you 'fix' a check that never ran."""
    rf = R.evaluate(Probe(failing=["cameras"]))
    ru = R.evaluate(Probe(missing=["cameras"]))
    assert not rf.ready and not ru.ready
    assert "FAILING" in rf.summary() and "NOT CHECKED" not in rf.summary()
    assert "NOT CHECKED" in ru.summary() and "FAILING" not in ru.summary()


def test_a_check_that_raises_is_unknown_not_ok():
    """A probe that throws must not leave the previous verdict, or OK."""
    r = R.evaluate(Probe(raising=["estop"]))
    assert not r.ready
    c = r.checks["estop"]
    assert c.state == R.UNKNOWN
    assert "raised" in c.detail


def test_one_dead_sensor_does_not_blind_the_others():
    r = R.evaluate(Probe(raising=["scene"]))
    assert r.checks["cameras"].state == R.OK
    assert r.checks["disk"].state == R.OK


def test_a_check_that_is_never_reported_stays_unknown():
    """THE STRUCTURAL GUARANTEE. Readiness is built from REQUIRED with every
    entry UNKNOWN; a check that is never wired writes nothing and therefore
    cannot write OK. Adding a name to REQUIRED and forgetting to implement it
    makes the system NOT READY, which is the correct direction to fail."""
    partial = [R.Check(n, R.OK, "fine") for n in R.REQUIRED if n != "recovery"]
    r = R.Readiness(partial)
    assert not r.ready
    assert [c.name for c in r.unknown] == ["recovery"]


def test_state_cannot_be_a_bool():
    """No boolean anywhere: a bool cannot represent 'nobody asked'."""
    for bad in (True, False, 1, 0, None, "yes"):
        try:
            R.Check("cameras", bad)
        except ValueError:
            continue
        raise AssertionError("Check accepted a non-tri-state: %r" % bad)


def test_every_failing_check_is_named():
    r = R.evaluate(Probe(failing=["disk", "homed"], missing=["scene"]))
    s = r.summary()
    assert "disk space" in s and "arms at home" in s
    assert "scene calibrated" in s
    assert not r.ready
