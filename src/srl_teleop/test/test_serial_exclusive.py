#!/usr/bin/env python3
"""Two readers on one master port must be IMPOSSIBLE, not merely detectable.

On 2026-08-06 two teleop stacks ran with both master_pose_node instances
holding /dev/ttyACM0. Nothing errored -- POSIX allows many readers on one tty
-- but the kernel gives each byte to exactly one of them, so each parsed
fragments. The raw topic ran at 68 Hz with 100% of rows distinct where a
healthy single stream gives ~15 Hz of distinct updates inside 50 Hz rows, and
a full day of channel-health measurement had to be discarded.

Run against a real pty, so the test exercises the same fcntl path as a real
/dev/ttyACM*. It does NOT construct an environment where the bug cannot occur
-- the whole point is that the bug CAN occur here and the lock stops it.
"""
import os
import sys
import threading
import time

import pytest

serial = pytest.importorskip("serial")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_teleop.serial_port import (                         # noqa: E402
    PortNotFound, claim_exclusive, sniff)


def feed(master_fd, stop, line=b"k1j1:100 j2:100 j3:100\n"):
    """A pty writer that keeps going, because pyserial's open() calls
    reset_input_buffer() and DISCARDS anything written before the open. A
    real Teensy streams continuously; a one-shot write does not model it and
    made the first version of this test fail for a reason that had nothing to
    do with the code under test."""
    def run():
        while not stop.is_set():
            try:
                os.write(master_fd, line)
            except OSError:
                return
            time.sleep(0.02)
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


@pytest.fixture
def pty_port():
    master, slave = os.openpty()
    name = os.ttyname(slave)
    yield master, name
    for fd in (master, slave):
        try:
            os.close(fd)
        except OSError:
            pass


def test_second_claim_is_refused(pty_port):
    _, name = pty_port
    a = serial.Serial(name, 115200, timeout=0.05)
    claim_exclusive(a)
    b = serial.Serial(name, 115200, timeout=0.05)
    try:
        with pytest.raises(PortNotFound) as ei:
            claim_exclusive(b)
        # The refusal must NAME the cause and the recovery, not just fail.
        msg = str(ei.value)
        assert "already held" in msg
        assert "master_pose_node" in msg      # states how to find the holder
        assert "SPLIT" in msg or "split" in msg
    finally:
        b.close()
        a.close()


def test_lock_is_released_on_close(pty_port):
    """Otherwise a crashed node would lock the port until reboot, which is a
    worse failure than the one being prevented."""
    _, name = pty_port
    a = serial.Serial(name, 115200, timeout=0.05)
    claim_exclusive(a)
    a.close()
    b = serial.Serial(name, 115200, timeout=0.05)
    try:
        claim_exclusive(b)          # must not raise
    finally:
        b.close()


def test_sniff_refuses_to_probe_a_held_port(pty_port):
    """The sniffer READS. Probing a port a live node owns steals bytes from
    its frame stream -- the same corruption, caused by the port search."""
    master, name = pty_port
    stop = threading.Event()
    feed(master, stop)
    a = serial.Serial(name, 115200, timeout=0.05)
    claim_exclusive(a)
    try:
        ok, why = sniff(name, tries=1, settle=0.05)
        assert ok is False
        assert "IN USE" in why
        assert "steal" in why
    finally:
        stop.set()
        a.close()


def test_sniff_still_works_on_a_free_port(pty_port):
    """The guard must not break detection -- a lock that also prevents the
    legitimate first open would make the rig undriveable."""
    master, name = pty_port
    stop = threading.Event()
    feed(master, stop)
    try:
        ok, why = sniff(name, tries=30, settle=0.1, timeout=0.2)
        assert ok is True, why
    finally:
        stop.set()
