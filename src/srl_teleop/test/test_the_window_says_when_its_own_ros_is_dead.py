"""A window that outlives its ROS node must say so, not blame the arm.

MEASURED ON THE RIG 2026-08-29. The operations window was drawn, responsive
to clicks and showing a full set of readings. Its ROS node was dead:

    /vr_safety_node                  ANSWERS
    /kortex_highlevel_bridge_left    ANSWERS
    /ik_follower_left                ANSWERS
    /srl_gui                         NO ANSWER

`ros2 param list` is served from a node's own executor, so three controls
answering and this one not is the executor, not the daemon. At the same
moment `/real/joint_states` was publishing at 11.9 Hz with `srl_gui` listed
as its subscriber -- the data was arriving at a process that had stopped
collecting it.

What the operator saw was `bridge up, NO DATA` on the arm panel. That label
is chosen when `fresh` is false, and `fresh` is false both when the ARM stops
publishing and when THIS WINDOW stops receiving. The two rendered
identically, so the reading pointed at the arm, which was fine. Every service
button was inert at the same time and for the same reason -- `bus.submit`
appends to a queue drained by a timer in the dead executor -- so pressing
harder produced nothing and logged nothing.

A frozen picture of a rig is worse than a blank one: it is a reading the
operator has no reason to distrust.

`Bus._snapshot` runs on a 0.1 s timer INSIDE that executor, so `snap["t"]`
stops advancing the moment the executor does. These tests hold that the
window reads its own liveness off that stamp, that it says so in the banner
ahead of every other state, and that the arm label names the window rather
than the arm. Nothing here starts a stack or opens a window.
"""
import os
import re
import time
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
SRC = open(os.path.join(WS, "scripts", "srl_gui.py")).read()


def _slice(head, tail):
    a = SRC.index(head)
    b = SRC.index(tail, a)
    assert b > a, "the slice bounds moved -- this test reads nothing"
    return SRC[a:b]


class _Bus:
    def __init__(self, snap):
        self.snap = snap


def _gui(age_s):
    """A stand-in carrying the real `ros_stale_s`, over a snapshot of a
    chosen age. Sliced rather than imported: `srl_gui` needs PyQt and a
    display, and this has to run in CI."""
    body = _slice("    ROS_DEAD_S = ", "    def refresh(self):")
    ns = {"time": time}
    exec("class _G:\n" + body, ns)              # noqa: S102 -- code under test
    g = ns["_G"]()
    g.bus = _Bus({} if age_s is None
                 else {"t": time.monotonic() - age_s})
    return g


class TestLivenessComesFromTheSnapshotStamp(unittest.TestCase):

    def test_a_live_executor_is_not_reported_stale(self):
        self.assertIsNone(_gui(0.0).ros_stale_s())
        self.assertIsNone(_gui(1.0).ros_stale_s())

    def test_a_stopped_executor_is_reported_with_its_age(self):
        g = _gui(30.0)
        age = g.ros_stale_s()
        self.assertIsNotNone(age, "a snapshot 30 s old is a dead executor")
        self.assertGreater(age, 29.0)

    def test_the_threshold_is_above_the_snapshot_period(self):
        # _snapshot runs at 0.1 s. A threshold near that would flag every
        # scheduling hiccup as a dead node, and an alarm that cries wolf is
        # an alarm somebody learns to ignore.
        g = _gui(0.0)
        self.assertGreaterEqual(g.ROS_DEAD_S, 1.0)

    def test_no_snapshot_yet_is_not_a_death(self):
        # A window in its first moments has no stamp. Reporting that as a
        # dead node would put the slab up on every launch.
        self.assertIsNone(_gui(None).ros_stale_s())


class TestTheWindowSaysIt(unittest.TestCase):

    def test_the_banner_branch_outranks_the_e_stop_slab(self):
        banner = _slice("        if getattr(self, \"_narration\", None):",
                        "        # charts")
        i_dead = banner.index("ros_stale_s")
        i_es = banner.index("elif es:")
        self.assertLess(i_dead, i_es,
                        "with the executor stopped, `es` is a value read "
                        "some time ago like every other reading -- the "
                        "banner has to say the window is blind before it "
                        "reports what the rig was doing")
        self.assertIn("FROZEN", banner)
        self.assertIn("no button will send", banner)

    def test_the_arm_label_names_the_window_not_the_arm(self):
        panel = _slice("        js = s.get(\"real_js_arms\") or {}",
                       "    def _ping_arms")
        self.assertIn("deaf = self.ros_stale_s(s)", panel,
                      "the arm panel has to ask whether the WINDOW is "
                      "receiving before it interprets `fresh`")
        i_deaf = panel.index("elif deaf is not None:")
        i_nodata = panel.index('lbl.setText("bridge up, NO DATA")')
        self.assertLess(i_deaf, i_nodata,
                        "`bridge up, NO DATA` blames the arm and must not "
                        "be reachable while this window is deaf")
        self.assertIn("WINDOW NOT RECEIVING", panel,
                      "and it stays SHORT: these status chips are not "
                      "word-wrapped and the control column is fitted before "
                      "any press, so a long one overflows rather than "
                      "widening it -- the audit's own cut-off check caught "
                      "exactly that. The age goes in the banner.")


class TestTheCheckCanFail(unittest.TestCase):
    """A check that cannot fail on a deliberately broken input is not a
    check. Both of these assert the OLD behaviour is gone."""

    def test_the_old_unconditional_label_is_no_longer_reachable_first(self):
        panel = _slice("        js = s.get(\"real_js_arms\") or {}",
                       "    def _ping_arms")
        # The condition guarding the old label must still be `not fresh` --
        # if someone deletes the deaf branch this reverts and the test above
        # fails. Here we pin that the two branches are DISTINCT states, not
        # one renamed.
        self.assertIn("elif not fresh:", panel)
        self.assertEqual(
            panel.count('lbl.setText("bridge up, NO DATA")'), 1,
            "one branch sets it, and the comment above the deaf branch "
            "quotes it -- count the CALL, not the words")

    def test_ros_stale_s_returns_none_not_zero_when_live(self):
        # `None` and `0.0` are both falsey, and a caller writing
        # `if age:` would work either way -- but the banner formats the value
        # with %.0f, so returning 0.0 when live would put a slab up saying
        # "DEAD (0 s)". Pin the type.
        self.assertIs(_gui(0.5).ros_stale_s(), None)


if __name__ == "__main__":
    unittest.main()
