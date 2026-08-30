#!/usr/bin/env python3
"""The scene camera must RE-PROBE when it stops reading, not fail for ever.

Under WSL the camera arrives over usbip and the service drops it on its own;
a re-attach can hand the SAME camera back as a different /dev/videoN. The node
used to `return` on every failed read and never try again, so it sat holding a
dead handle -- still reporting "open on /dev/video0" while that node no longer
existed -- until a person noticed and restarted it.

These tests do not need a camera. They drive the recovery logic directly,
which is the only part that was missing.
"""
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(WS, "src/srl_perception"))


class _Fake:
    """The node's recovery surface, without ROS or a camera."""
    REOPEN_AFTER_FAILURES = 30
    REOPEN_EVERY_S = 2.0

    def __init__(self, reopen_succeeds=True):
        self.n_fail = 0
        self.cap = None
        self.dev = "/dev/video0"
        self.reason = "open on /dev/video0"
        self.opens = 0
        self._reopen_ok = reopen_succeeds
        self._logged = []

    def get_logger(self):
        return types.SimpleNamespace(info=self._logged.append,
                                     warn=self._logged.append,
                                     error=self._logged.append)

    def _repair_usb(self):
        """Recorded, never performed.

        The node calls this from `_maybe_reopen` when a re-probe found no
        device, and it shells out to usbipd to detach and re-attach a real
        camera. A stand-in that actually did that would make these tests
        disruptive to whatever else on the machine is holding a camera.

        It was missing entirely, so all three tests here died with
        `AttributeError: '_Fake' object has no attribute '_repair_usb'` --
        the node grew a recovery step and its stand-in did not follow. The
        count is kept because "did the back-off stop us reaching the
        expensive repair" is exactly what these tests are about.
        """
        self.repairs = getattr(self, "repairs", 0) + 1

    def _open(self):
        self.opens += 1
        if self._reopen_ok:
            self.cap = object()
            self.dev = "/dev/video4"      # deliberately a DIFFERENT node
            self.reason = "open on /dev/video4, 1280x720"


def _bind():
    from srl_perception.scene_camera_node import SceneCamera
    return SceneCamera._maybe_reopen


def test_it_does_not_reopen_on_a_single_dropped_frame():
    m = _bind()
    f = _Fake()
    f.n_fail = 1
    m(f)
    assert f.opens == 0, "one bad read must not tear down a working camera"


def test_it_reopens_after_a_sustained_failure():
    m = _bind()
    f = _Fake()
    f.n_fail = _Fake.REOPEN_AFTER_FAILURES
    m(f)
    assert f.opens == 1
    assert f.n_fail == 0, "a successful re-open must clear the failure count"


def test_recovery_accepts_a_DIFFERENT_device_number():
    """The whole point. A re-attach can renumber the camera, so recovery must
    re-run the probe rather than reopen the remembered path."""
    m = _bind()
    f = _Fake()
    f.n_fail = 40
    m(f)
    assert f.dev == "/dev/video4", \
        "recovery must take whatever node now delivers, not the old number"
    assert any("RECOVERED" in str(x) for x in f._logged)


def test_it_backs_off_rather_than_reopening_every_tick():
    """Re-opening a V4L2 device at the tick rate is its own way of keeping a
    camera unusable."""
    m = _bind()
    f = _Fake(reopen_succeeds=False)
    f.n_fail = 50
    m(f)
    m(f)
    m(f)
    assert f.opens == 1, "back-off must suppress the 2nd and 3rd attempt"


def test_it_retries_again_after_the_backoff_expires():
    m = _bind()
    f = _Fake(reopen_succeeds=False)
    f.n_fail = 50
    m(f)
    assert f.opens == 1
    f._last_reopen = time.monotonic() - (_Fake.REOPEN_EVERY_S + 0.5)
    m(f)
    assert f.opens == 2, "after the back-off it must try again"


def test_a_failed_reopen_leaves_the_failure_count_alone():
    """So the state topic keeps saying it is still broken."""
    m = _bind()
    f = _Fake(reopen_succeeds=False)
    f.n_fail = 50
    m(f)
    assert f.n_fail == 50
    assert f.cap is None


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
