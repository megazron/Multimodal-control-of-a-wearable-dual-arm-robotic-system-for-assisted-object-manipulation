#!/usr/bin/env python3
"""An EXPIRED blocker must never read as an all-clear.

BlockMonitor auto-expires a blocker that nobody has re-asserted, which is what
stops the unreachable-clear() pattern latching a block forever. The danger in
that fix is the inverse failure: an expiry means the loop asserting the
blocker STOPPED RUNNING, so the guarded condition is UNKNOWN, not gone.
Treating it as "clear, resume motion" would turn a crashed guard into a green
light -- worse than the latch it replaced.

These tests pin the three-state semantics: active / expired(unknown) / clear.
"""
import time

import pytest

from srl_teleop import block_monitor
from srl_teleop.block_monitor import BlockMonitor, EXPIRE_S


class _Clock:
    """Deterministic clock. BlockMonitor stamps assertions with
    time.monotonic() internally, so a test that advances only the argument to
    _expire_stale() would age every blocker artificially and 'prove' that a
    held blocker expires."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


@pytest.fixture(autouse=True)
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(block_monitor.time, 'monotonic', c)
    return c


class _Log:
    def __init__(self):
        self.msgs = []

    def warn(self, m, **k):
        self.msgs.append(('warn', m))

    def error(self, m, **k):
        self.msgs.append(('error', m))

    def info(self, m, **k):
        self.msgs.append(('info', m))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class _Node:
    """Minimal stand-in: BlockMonitor only needs a logger, a publisher, a timer."""

    def __init__(self):
        self._log = _Log()
        self.pub = _Pub()
        self.timers = []

    def get_logger(self):
        return self._log

    def create_publisher(self, *a, **k):
        return self.pub

    def create_timer(self, period, cb):
        self.timers.append(cb)
        return None


def _monitor():
    n = _Node()
    b = BlockMonitor(n, 'unit_under_test')
    b.register('guard', 'a guard that stops motion',
               recovery='the asserting loop must run again')
    return n, b


def test_expiry_reports_unknown_not_clear(clock):
    n, b = _monitor()
    b.block('guard', 'condition is bad')
    assert b.blocked and b.active_names == ['guard']
    assert not b.state_unknown

    # Nobody re-asserts it: the asserting loop has stopped.
    b._expire_stale(clock.advance(EXPIRE_S + 0.1))

    assert b.active_names == [], 'must not stay ACTIVE -- that is the latch'
    assert b.expired_names == ['guard'], 'must be EXPIRED, not cleared'
    assert b.state_unknown is True
    assert b.blocked is True, (
        'blocked must stay True while the state is unknown: "nobody is '
        'asserting it" is not evidence the condition went away')


def test_expiry_is_logged_as_a_fault_not_a_recovery(clock):
    n, b = _monitor()
    b.block('guard', 'bad')
    b._expire_stale(clock.advance(EXPIRE_S + 0.1))
    errs = [m for lvl, m in n._log.msgs if lvl == 'error']
    assert any('AUTO-EXPIRED' in m and 'NOT an all-clear' in m for m in errs), \
        'an expiry means a unit stopped running and must be loud about it'


def test_re_assertion_resolves_the_unknown(clock):
    n, b = _monitor()
    b.block('guard', 'bad')
    b._expire_stale(clock.advance(EXPIRE_S + 0.1))
    assert b.state_unknown
    b.block('guard', 'still bad')          # the loop is running again
    assert not b.state_unknown
    assert b.active_names == ['guard']


def test_explicit_clear_from_live_code_resolves_the_unknown(clock):
    """The unit is demonstrably alive and says the condition is gone."""
    n, b = _monitor()
    b.block('guard', 'bad')
    b._expire_stale(clock.advance(EXPIRE_S + 0.1))
    assert b.state_unknown
    b.clear('guard')
    assert not b.state_unknown
    assert not b.blocked


def test_held_blocker_is_never_expired(clock):
    """A genuinely held condition is re-asserted every cycle and must survive."""
    n, b = _monitor()
    for _ in range(50):                    # 10 Hz for 5 s, the slowest loop
        b.block('guard', 'still bad')
        b._expire_stale(clock.advance(0.1))
    assert b.active_names == ['guard']
    assert not b.state_unknown, 'a re-asserted blocker must never expire'


def test_published_state_distinguishes_all_three(clock):
    import json
    n, b = _monitor()
    b.block('guard', 'bad')
    n.timers[0]()                          # BlockMonitor._tick
    d = json.loads(n.pub.sent[-1].data)
    assert d['active'] == ['guard'] and d['expired'] == []
    assert d['state_unknown'] is False

    b._expire_stale(clock.advance(EXPIRE_S + 0.1))
    n.timers[0]()
    d = json.loads(n.pub.sent[-1].data)
    assert d['active'] == [], 'expired must not be reported as active'
    assert d['expired'] == ['guard']
    assert d['state_unknown'] is True
    assert d['blocked'] is True, 'downstream must not see an all-clear'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))


def test_escalation_does_not_crash_the_logger():
    """A blocker held past escalate_s must log at ERROR after logging at WARN.

    rclpy caches log severity per CALL SITE. block_monitor used to select the
    method into a variable and call it from one line, so the first time
    anything blocked for more than escalate_s the repeat-log raised
    ValueError inside the monitor's own timer -- in the component whose only
    job is to make blocking visible. Verified to raise before the fix.

    Driven by re-asserting block() on a real BlockMonitor: that is the path
    the follower actually takes at 50 Hz while a condition is held.
    """
    import rclpy
    from rclpy.node import Node
    from srl_teleop.block_monitor import BlockMonitor
    import srl_teleop.block_monitor as bmod

    rclpy.init()
    try:
        n = Node("escalation_probe")
        bm = BlockMonitor(n, "probe", escalate_s=0.05)
        bm.register("thing", "a thing is blocking", "clear the thing")
        bm.block("thing")                      # first WARN
        # Age the blocker explicitly instead of sleeping: other tests in this
        # file drive BlockMonitor with an artificial clock, and a wall-clock
        # sleep here is both slower and less deterministic.
        b = bm._b["thing"]
        b.since = time.monotonic() - 10.0
        bm.block("thing")                      # crosses escalate_s -> ERROR
        assert b.escalated
        # Now force the REPEAT path at both severities. This is the line
        # that used to raise.
        for esc in (False, True, False, True):
            b = bm._b["thing"]
            b.escalated = esc
            b.last_log = time.monotonic() - (bmod.REPEAT_S + 1.0)
            bm.block("thing", "still here")    # must not raise
        n.destroy_node()
    finally:
        rclpy.shutdown()
