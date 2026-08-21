#!/usr/bin/env python3
"""A stalled stream must not keep reporting the rate it used to have.

MEASURED 2026-08-19, with a real Quest. The headset was taken off and set on
a table. Within ONE second the WebXR session suspended and frames stopped --
the frame counter froze at 21343 and did not move again. The bridge went on
reporting:

    rate_hz : 89.8      (unchanged for minutes)
    clients : 1
    rtt     : 15.12     (unchanged)

Both of those fields lied, for two different reasons:

  * `rate_hz` was (n-1)/(last-first) over a window of ARRIVAL times. When
    arrivals stop the window stops changing, so the expression returns the
    last healthy value forever.
  * `clients` counts OPEN SOCKETS. A suspended browser page still answers
    WebSocket pings at the network layer, so the socket never closed.

Only `frames` told the truth. This matters more in desk operation than
anywhere else: the operator is not wearing the headset and cannot see an
overlay, so `vr_desk_monitor` is their ONLY status display -- and it would
have shown a healthy green link with nothing arriving.

This is the "data fresh but never changes" row of CLAUDE.md's instrument
table, in the instrument this project uses to watch itself.
"""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")
from srl_vr_teleop.quest_bridge_node import QuestBridge            # noqa: E402


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def bridge(ros, monkeypatch):
    """The transport is stubbed out, deliberately.

    _report() is pure arithmetic over rate_win/last_rx and needs no socket.
    Letting __init__ start the real server makes this test bind port 8765,
    so it would pass only when something else already holds the port -- a
    test whose result depends on what else is running is not a test.
    """
    monkeypatch.setattr(QuestBridge, '_serve', lambda self: None)
    n = QuestBridge()
    yield n
    n.destroy_node()


def _published(n):
    """Run _report and capture what it put on /vr/bridge_status."""
    import json
    grabbed = {}
    real = n.stat_pub.publish
    n.stat_pub.publish = lambda m: grabbed.update(json.loads(m.data))
    try:
        n._report()
    finally:
        n.stat_pub.publish = real
    return grabbed


def test_a_live_stream_reports_its_rate(bridge):
    now = time.monotonic()
    for i in range(180):
        bridge.rate_win.append(now - 2.0 + i / 90.0)
    bridge.last_rx = bridge.rate_win[-1]
    bridge.n_frames = 180
    out = _published(bridge)
    assert out['live'] is True
    assert 80 < out['rate_hz'] < 100, out


def test_a_STOPPED_stream_reports_ZERO_not_its_last_value(bridge):
    """The whole point. Same window, just old."""
    now = time.monotonic()
    for i in range(180):
        bridge.rate_win.append(now - 30.0 + i / 90.0)   # all 28+ s ago
    bridge.last_rx = bridge.rate_win[-1]
    bridge.n_frames = 21343
    out = _published(bridge)
    assert out['live'] is False, out
    assert out['rate_hz'] == 0.0, (
        'a stalled stream reported %s Hz -- this is exactly the number that '
        'made a dead link look healthy on the operator display' % out['rate_hz'])
    assert out['age_s'] > 25.0


def test_the_frame_counter_is_still_published_because_it_never_lied(bridge):
    now = time.monotonic()
    for i in range(180):
        bridge.rate_win.append(now - 30.0 + i / 90.0)
    bridge.last_rx = bridge.rate_win[-1]
    bridge.n_frames = 21343
    out = _published(bridge)
    assert out['frames'] == 21343


def test_clients_alone_cannot_be_trusted_so_live_is_published_beside_it(bridge):
    """An open socket is not a sending client. A suspended page keeps the
    socket up and answers pings, which is why `clients` read 1 throughout."""
    now = time.monotonic()
    for i in range(180):
        bridge.rate_win.append(now - 30.0 + i / 90.0)
    bridge.last_rx = bridge.rate_win[-1]

    class _FakeWs:
        pass
    bridge.ws_clients.add(_FakeWs())            # socket open...
    out = _published(bridge)
    assert out['clients'] == 1                  # ...and it still says so...
    assert out['live'] is False                 # ...but live tells the truth
    bridge.ws_clients.clear()


def test_no_frames_ever_is_not_reported_as_live(bridge):
    bridge.last_rx = 0.0
    bridge.rate_win.clear()
    out = _published(bridge)
    assert out['live'] is False
    assert out['rate_hz'] == 0.0
    assert out['age_s'] is None, 'no frames at all has no age, not age 0'


def test_the_desk_monitor_reads_live_and_not_clients():
    """The consumer, not the field. Publishing `live` and having the display
    keep reading `clients` would fix nothing."""
    import re
    p = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                     'scripts', 'vr_desk_monitor.py')
    p = os.path.normpath(p)
    if not os.path.exists(p):
        pytest.skip('desk monitor not in this checkout')
    src = open(p).read()
    assert "b.get('live')" in src
    assert re.search(r"live\s*else", src), 'the LIVE/NO FRAMES branch'
