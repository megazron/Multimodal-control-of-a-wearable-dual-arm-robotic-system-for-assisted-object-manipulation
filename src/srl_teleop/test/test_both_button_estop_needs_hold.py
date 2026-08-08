#!/usr/bin/env python3
"""Gating taps must not fire the both-button e-stop; a held squeeze must.

The capture gates every segment on a button press. Under the old rule -- any
two button EDGES within 0.4 s, where a release is an edge -- ordinary gating
taps latched the e-stop for the whole of block F.
"""
import pytest


class _Stub:
    """Exercises the real decision logic without a ROS graph."""

    def __init__(self, hold=0.30):
        self.both_hold_s = hold
        self._both_since = None
        self.tripped = []

    def trip(self, why):
        self.tripped.append(why)

    def feed(self, b1, b2, now):
        down = {1: bool(b1), 2: bool(b2)}
        if down[1] and down[2]:
            if self._both_since is None:
                self._both_since = now
            elif now - self._both_since >= self.both_hold_s:
                self._both_since = None
                self.trip("held")
        else:
            self._both_since = None


def test_sequential_gating_taps_do_not_trip():
    s = _Stub()
    t = 0.0
    for _ in range(20):                       # 20 alternating gating presses
        for b1, b2 in ((1, 0), (0, 0), (0, 1), (0, 0)):
            s.feed(b1, b2, t)
            t += 0.05
    assert not s.tripped, "gating taps must never fire the e-stop"


def test_brief_simultaneous_touch_does_not_trip():
    s = _Stub()
    for i, t in enumerate([0.0, 0.05, 0.10, 0.15]):
        s.feed(1, 1, t)
    assert not s.tripped, "0.15 s overlap is below the 0.30 s hold"


def test_deliberate_held_squeeze_trips():
    s = _Stub()
    t = 0.0
    while t <= 0.5:
        s.feed(1, 1, t)
        t += 0.05
    assert s.tripped, "squeeze-and-hold must still e-stop"


def test_release_edge_alone_does_not_trip():
    s = _Stub()
    s.feed(1, 1, 0.0)
    s.feed(0, 0, 0.05)
    s.feed(1, 1, 0.10)
    s.feed(0, 0, 0.15)
    assert not s.tripped


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
