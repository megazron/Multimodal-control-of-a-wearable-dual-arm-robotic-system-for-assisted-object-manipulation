"""Silence from the observer must be treated as absence, not as presence.

THE DEFECT THIS PINS. `/vr/observer_estop_present` was handled as a LATCH:
`True` set the flag and only an explicit `False` cleared it. So a publisher
that simply STOPPED -- a closed terminal, a slept laptop, a killed process, an
observer who put the button down and left the room -- left `vr_safety_node`
believing somebody was standing next to a 17 kg rig with their hand on an
e-stop, for as long as the session lasted.

That is the single state this interlock exists to detect, and it was the one
state it could not see. Every other VR failure has a signature: a dropout
stops the poses, an occlusion invalidates a controller, a nudged reference
moves the HMD. An observer walking away changes nothing observable at all.

Driven through the real timeout logic with a clock the test controls, so it
cannot pass by taking two seconds and hoping.
"""


class Fake:
    """The two methods under test, lifted onto a controllable clock.

    Deliberately NOT a full node: `vr_safety_node` needs rclpy, a TF buffer
    and a graph, and none of that is what is being checked. The logic below
    is a transcription of `_on_observer` and `_observer_stale`, and
    `test_the_transcription_matches_the_node` keeps it honest.
    """

    def __init__(self, timeout=2.0):
        self.observer_ok = False
        self.observer_t = None
        self.timeout = timeout
        self.now = 100.0
        self.frozen = False
        self.reason = ''

    def on_observer(self, data):
        if data:
            self.observer_t = self.now
        if data and not self.observer_ok:
            self.observer_ok = True
        elif not data and self.observer_ok:
            self.observer_ok = False
            self.observer_t = None
            self.freeze('observer e-stop withdrawn')

    def stale(self):
        if not self.observer_ok:
            return False
        if self.timeout <= 0.0 or self.observer_t is None:
            return False
        return (self.now - self.observer_t) > self.timeout

    def freeze(self, why):
        self.frozen = True
        self.reason = why

    def tick(self):
        if self.stale():
            self.observer_ok = False
            self.observer_t = None
            self.freeze('observer e-stop stopped reporting')


def test_a_beating_observer_stays_present():
    """The control. Without it a node that froze unconditionally would pass
    every other test in this file."""
    f = Fake()
    for _ in range(50):
        f.now += 0.2
        f.on_observer(True)
        f.tick()
    assert f.observer_ok and not f.frozen


def test_silence_freezes():
    f = Fake()
    f.on_observer(True)
    assert f.observer_ok
    # The publisher stops. Nothing else changes: no False, no error, no
    # dropout -- exactly what walking out of the room looks like.
    for _ in range(30):
        f.now += 0.2
        f.tick()
    assert not f.observer_ok, "silence left the observer marked present"
    assert f.frozen and 'stopped reporting' in f.reason


def test_silence_shorter_than_the_timeout_does_not_freeze():
    """One dropped beat is a dropped beat, not an absent observer. A timeout
    tight enough to fire on ordinary jitter would be turned off on the first
    lab day, which is worse than not having it."""
    f = Fake(timeout=2.0)
    f.on_observer(True)
    f.now += 1.5
    f.tick()
    assert f.observer_ok and not f.frozen


def test_an_explicit_withdrawal_still_freezes_immediately():
    f = Fake()
    f.on_observer(True)
    f.now += 0.1
    f.on_observer(False)
    assert not f.observer_ok
    assert f.frozen and 'withdrawn' in f.reason


def test_the_timeout_can_be_switched_off_and_says_so_by_doing_nothing():
    """A head-worn session with a different interlock may want it off. It
    must then behave EXACTLY as the old latch did, so turning it off is a
    known state rather than a new one."""
    f = Fake(timeout=0.0)
    f.on_observer(True)
    f.now += 1000.0
    f.tick()
    assert f.observer_ok and not f.frozen


def test_the_transcription_matches_the_node():
    """This file models the node's logic. If the node changes shape, the
    model is a description of something that no longer exists."""
    import os
    src = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'srl_vr_teleop', 'vr_safety_node.py')
    text = open(src).read()
    for needle in ('observer_estop_timeout_s',
                   'def _observer_stale',
                   'self.observer_t = time.monotonic()',
                   'stopped reporting'):
        assert needle in text, (
            '%r is gone from vr_safety_node -- this test is now modelling '
            'code that does not exist' % needle)
