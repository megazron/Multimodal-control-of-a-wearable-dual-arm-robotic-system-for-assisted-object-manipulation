"""The relay's auto-enable thread must survive its own inputs.

MEASURED ON THE RIG 2026-08-29, in `.scratch/launch_real_cascade.log`:

    [WARN] waiting to enable: REFUSED: sim is 2.892 rad (165.69 deg) from the
           real arm on joint_7, over the 1.00 rad ENABLE gap ...
    Exception in thread Thread-1 (_auto_enable):
      File ".../sim_to_real_bridge.py", line 251, in sim_delayed
        for t, q in self.sim_hist:
    RuntimeError: deque mutated during iteration

`sim_hist` is a BOUNDED deque appended by `on_sim` on the ROS executor thread
and iterated by `sim_delayed()`, which `_auto_enable` calls from a thread of
its own. A bounded deque evicts as it appends, so an append landing mid-scan
raises -- and that exception ends the thread.

Why that is fatal rather than untidy: `_auto_enable` is the ONLY thing that
will ever enable the relay on this path. Once it dies the relay stops
retrying for good. The refusals it had been printing were asking the operator
to bring the arm to home -- and after they did, nothing was left alive to
notice. The operator sees a relay that is running, whose refusal was correct,
whose cure was applied, and which is permanently dead. Nothing in the node's
state says so.

Two fixes, and both are tested here: `sim_delayed` iterates a SNAPSHOT taken
under a lock, and the retry loop catches, SAYS SO, and keeps retrying rather
than unwinding.
"""
import os
import re
import threading
import time
import unittest
from collections import deque

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
SRC = open(os.path.join(
    WS, "src", "srl_teleop", "srl_teleop", "sim_to_real_bridge.py")).read()


class _Hist:
    """Just the history half of the bridge, with the real `sim_delayed`."""

    def __init__(self, delay=1.0, maxlen=50):
        self.sim_hist = deque(maxlen=maxlen)
        self._hist_lock = threading.Lock()
        self.delay = delay
        self.names = ["j%d" % i for i in range(7)]
        ns = {"time": time, "threading": threading}
        body = SRC[SRC.index("    def sim_delayed(self):"):
                   SRC.index("    # ---------------- enable / disable")]
        exec("class _M:\n" + body, ns)         # noqa: S102 -- code under test
        self.sim_delayed = ns["_M"].sim_delayed.__get__(self)


class TestSimDelayedSurvivesAConcurrentWriter(unittest.TestCase):

    def test_it_does_not_raise_while_the_deque_is_being_appended(self):
        h = _Hist()
        for i in range(60):                    # full, so appends EVICT
            h.sim_hist.append((time.monotonic() - 5 + i * 0.01, [0.0] * 7))
        stop = threading.Event()
        errors = []

        def writer():
            while not stop.is_set():
                with h._hist_lock:
                    h.sim_hist.append((time.monotonic(), [0.0] * 7))

        t = threading.Thread(target=writer, daemon=True)
        t.start()
        try:
            for _ in range(3000):
                try:
                    h.sim_delayed()
                except RuntimeError as e:      # the exact failure
                    errors.append(e)
                    break
        finally:
            stop.set(); t.join(timeout=2)
        self.assertEqual(errors, [],
                         "sim_delayed must iterate a snapshot: an append on "
                         "the ROS thread mid-scan killed the only thread "
                         "that ever enables the relay")

    def test_it_still_returns_the_delayed_sample(self):
        # The snapshot must not change what the method is FOR.
        h = _Hist(delay=1.0)
        now = time.monotonic()
        h.sim_hist.append((now - 2.0, [1.0] * 7))     # older than the delay
        h.sim_hist.append((now, [9.0] * 7))           # the present
        self.assertEqual(h.sim_delayed(), [1.0] * 7,
                         "the delayed sample, not the present one -- the "
                         "present is exactly what the delay avoids")

    def test_empty_history_is_still_none(self):
        self.assertIsNone(_Hist().sim_delayed())


class TestTheWriterTakesTheLockToo(unittest.TestCase):
    """A snapshot under a lock only helps if the writer honours it."""

    def test_on_sim_appends_under_the_lock(self):
        body = SRC[SRC.index("    def on_sim(self, m):"):
                   SRC.index("    def on_real(self, m):")]
        self.assertIn("with self._hist_lock:", body)

    def test_the_lock_exists(self):
        self.assertIn("self._hist_lock = threading.Lock()", SRC)


class TestTheRetryLoopCannotDieSilently(unittest.TestCase):

    def test_the_attempt_is_wrapped(self):
        loop = SRC[SRC.index("    def _auto_enable(self):"):
                   SRC.index("    def _enable_attempt(self):")]
        self.assertIn("try:", loop)
        self.assertIn("except Exception", loop)
        self.assertIn("continue", loop,
                      "catching and then falling out of the loop is the same "
                      "defect with a log line")

    def test_the_failure_says_the_relay_is_still_disabled(self):
        loop = SRC[SRC.index("    def _auto_enable(self):"):
                   SRC.index("    def _enable_attempt(self):")]
        self.assertIn("still DISABLED", loop,
                      "the operator's question is 'will the arm move', so "
                      "the answer has to be in the message")

    def test_a_precondition_is_not_reported_as_a_refusal(self):
        # (None, None) means "inputs not ready", which is not the same as
        # "refused" and must not be printed as one or stored as `last`.
        att = SRC[SRC.index("    def _enable_attempt(self):"):
                  SRC.index("    # ---------------- inputs")]
        self.assertIn("return (None, None)", att)
        loop = SRC[SRC.index("    def _auto_enable(self):"):
                   SRC.index("    def _enable_attempt(self):")]
        self.assertIn("if ok is not None:", loop)


class TestTheCheckCanFail(unittest.TestCase):
    """The concurrency test must fail on the code as it was."""

    def test_iterating_the_live_deque_raises(self):
        # Reproduce the OLD sim_delayed -- iterate the deque itself -- and
        # require the RuntimeError. If this stops raising, the first test
        # above proves nothing and should not be trusted.
        h = _Hist()
        for i in range(60):
            h.sim_hist.append((time.monotonic() - 5 + i * 0.01, [0.0] * 7))
        stop = threading.Event()
        seen = []

        def writer():
            while not stop.is_set():
                h.sim_hist.append((time.monotonic(), [0.0] * 7))

        t = threading.Thread(target=writer, daemon=True)
        t.start()
        try:
            for _ in range(200000):
                try:
                    for _t, _q in h.sim_hist:      # the old code, verbatim
                        pass
                except RuntimeError:
                    seen.append(1)
                    break
        finally:
            stop.set(); t.join(timeout=2)
        self.assertTrue(seen, "the race did not reproduce, so the fix above "
                              "is untested -- do not trust it")


if __name__ == "__main__":
    unittest.main()
