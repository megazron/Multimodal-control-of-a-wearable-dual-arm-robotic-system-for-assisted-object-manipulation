#!/usr/bin/env python3
"""The both-button e-stop must survive TOGGLE buttons.

The master buttons are firmware toggles (teensy_final.ino,
updateButtonToggle): /master_fsr_buttons carries a LATCHED 0/1 that flips
once per press and stays there. Reading those latches as levels meant a
button left toggled on from an earlier session turned the operator's FIRST
gating press into "both buttons held together", and the e-stop tripped 0.30 s
into segment 1 of the 2026-09-01 capture and latched for the rest of it.

THIS TEST BINDS THE NODE'S OWN on_buttons, not a copy of it. The version it
replaces reimplemented the decision in a local stub, so the stub and the node
could -- and did -- disagree: the stub passed every day while the shipped
rule was tripping the rig.
"""
import sys
import pathlib

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from srl_teleop.estop_node import EStop                       # noqa: E402


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        outer = self

        class _T:
            nanoseconds = 0

        _T.nanoseconds = int(outer.t * 1e9)
        return _T


class _Msg:
    def __init__(self, b1, b2):
        self.data = [0.0, 0.0, float(b1), float(b2)]


class Rig:
    """The real on_buttons, with only the state it touches."""

    def __init__(self, window=0.40, gap=1.0):
        self.both_window_s = window
        self.btn_gap_s = gap
        self.btn_rx = None
        self.btn_last = {1: None, 2: None}
        self.btn_edge = {1: None, 2: None}
        self.tripped = []
        self._clock = _Clock()

    def get_clock(self):
        return self._clock

    def trip(self, why):
        self.tripped.append(why)

    def feed(self, b1, b2, t):
        self._clock.t = t
        EStop.on_buttons(self, _Msg(b1, b2))


def test_a_device_reboot_clearing_both_toggles_does_not_trip():
    """THE 2026-09-01 REATTACH REGRESSION.

    The Teensy was detached and reattached to clear its dead IMUs. Both
    toggles had been latched at 1; it came back with both at 0. That is two
    edges in the first message after the gap, and it tripped the e-stop with
    nobody near a button.
    """
    r = Rig()
    t = 0.0
    for _ in range(10):                    # both latched on, stream healthy
        r.feed(1, 1, t); t += 0.02
    t += 30.0                              # detached: 30 s of silence
    for _ in range(10):                    # back, both toggles cleared by reboot
        r.feed(0, 0, t); t += 0.02
    assert not r.tripped, (
        "a stream gap is not a gesture -- the values either side of a reboot "
        "describe different device states")


def test_after_a_gap_the_stream_still_works_normally():
    """The resync must not deafen it: a real squeeze after a gap still trips."""
    r = Rig()
    r.feed(1, 1, 0.0)
    t = 40.0                               # gap, then a fresh stream
    r.feed(0, 0, t); t += 0.02             # adopted as history
    r.feed(1, 0, t); t += 0.10             # then a genuine two-handed squeeze
    r.feed(1, 1, t)
    assert r.tripped, "a real squeeze after a reattach must still e-stop"


def test_a_button_left_latched_does_not_arm_the_estop():
    """THE 2026-09-01 REGRESSION. btn1 toggled on in an earlier session; the
    operator presses the LEFT button once to start segment 1."""
    r = Rig()
    t = 0.0
    for _ in range(20):                    # btn1 sitting latched at 1
        r.feed(1, 0, t)
        t += 0.05
    r.feed(1, 1, t)                        # the single LEFT gating press
    t += 0.05
    for _ in range(40):                    # and it stays latched, as toggles do
        r.feed(1, 1, t)
        t += 0.05
    assert not r.tripped, (
        "a stale toggle plus ONE gating press must not e-stop; this is the "
        "fault that cost the 2026-09-01 capture")


def test_sequential_gating_taps_do_not_trip():
    r = Rig()
    t = 0.0
    for _ in range(20):                    # alternating single presses, 1 s apart
        r.feed(1, 0, t); t += 1.0
        r.feed(1, 1, t); t += 1.0
        r.feed(0, 1, t); t += 1.0
        r.feed(0, 0, t); t += 1.0
    assert not r.tripped, "gating presses seconds apart must never e-stop"


def test_bounce_on_one_button_does_not_trip():
    """Two edges in a window are not a squeeze if they are the same button."""
    r = Rig()
    for i, t in enumerate([0.0, 0.02, 0.04, 0.06, 0.08]):
        r.feed(i % 2, 0, t)
    assert not r.tripped


def test_deliberate_two_handed_squeeze_trips():
    r = Rig()
    r.feed(0, 0, 0.0)
    r.feed(1, 0, 0.10)                     # both pressed within the window
    r.feed(1, 1, 0.20)
    assert r.tripped, "squeezing both must still e-stop"


def test_squeeze_trips_from_any_toggle_parity():
    """Toggles flip DOWN as readily as up; the gesture must work either way."""
    r = Rig()
    for _ in range(5):
        r.feed(1, 1, 0.0)                  # both latched on, adopted as history
    r.feed(0, 1, 0.10)                     # squeeze: both flip off
    r.feed(0, 0, 0.20)
    assert r.tripped, "a squeeze from both-latched-on must e-stop too"


def test_first_message_is_history_not_a_press():
    """Whatever the toggles are at when the node starts is not a gesture."""
    r = Rig()
    r.feed(1, 1, 0.0)
    r.feed(1, 1, 0.05)
    assert not r.tripped


def test_presses_further_apart_than_the_window_do_not_trip():
    r = Rig()
    r.feed(0, 0, 0.0)
    r.feed(1, 0, 0.10)
    r.feed(1, 1, 0.90)                     # 0.80 s apart, window is 0.40
    assert not r.tripped


def test_one_gesture_is_one_trip():
    r = Rig()
    r.feed(0, 0, 0.0)
    r.feed(1, 0, 0.10)
    r.feed(1, 1, 0.20)
    n = len(r.tripped)
    for i in range(20):
        r.feed(1, 1, 0.30 + i * 0.05)
    assert len(r.tripped) == n, "the edges must be consumed, not re-fired"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
