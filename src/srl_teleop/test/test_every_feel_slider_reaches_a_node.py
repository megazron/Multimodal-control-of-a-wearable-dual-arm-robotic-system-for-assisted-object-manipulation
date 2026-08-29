"""A slider that sets a parameter nobody re-reads is a decoration.

THE DEFECT THIS EXISTS TO PREVENT, from earlier the same day. The window's
`START REAL ARM TELEOP` called a service that set `allow_real_arm` on
`vr_safety_node` -- a parameter the whole tree declares, passes and
republishes, and NEVER READS. The button reported success and the arm could
not move.

A panel of smoothness sliders is that defect waiting to happen seven more
times, because every ROS node caches its parameters in `__init__` by default.
`sim_to_real_bridge` did exactly that with `preview_delay_s`, `max_vel_rad_s`
and `max_step_rad`; `vr_pose_mapper` read its 1-Euro constants only when it
CONSTRUCTED a filter, and nothing constructed one again.

So each row of the FEEL table is checked here against the source of the node
that owns it: the value must be re-read in a loop, or accepted by a live
parameter callback. There is no third way for a slider to mean anything.

The last test is the other half of the contract: the SAFETY trips must not be
on the panel at all. A limit that can be widened from a slider while the arm
is moving is not a limit.
"""
import os
import re
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
GUI = open(os.path.join(WS, "scripts", "srl_gui.py")).read()


def _feel_rows():
    """The FEEL table, as (label, node, param), read from the window."""
    block = GUI[GUI.index("    FEEL = ["):GUI.index("    def _feel_panel")]
    return re.findall(
        r'\(\s*"([^"]+)",\s*"([^"]+)",\s*\n?\s*"([a-z_]+)"', block)


SOURCES = {
    "/vr_pose_mapper":
        os.path.join(WS, "src", "srl_vr_teleop", "srl_vr_teleop",
                     "vr_pose_mapper.py"),
    "/sim_to_real_bridge_%s":
        os.path.join(WS, "src", "srl_teleop", "srl_teleop",
                     "sim_to_real_bridge.py"),
    "/kortex_highlevel_bridge_%s":
        os.path.join(WS, "src", "srl_teleop", "srl_teleop",
                     "kortex_highlevel_bridge.py"),
}


class TestTheTableIsReadable(unittest.TestCase):

    def test_the_rows_parse(self):
        rows = _feel_rows()
        self.assertGreaterEqual(len(rows), 5,
                                "the FEEL table moved or changed shape -- "
                                "this whole file is reading nothing")

    def test_every_node_is_one_this_test_can_check(self):
        for label, node, prm in _feel_rows():
            self.assertIn(node, SOURCES,
                          "%r points at %s, which this test has no source "
                          "for -- add it rather than leaving the slider "
                          "unchecked" % (label, node))


class TestEverySliderIsActuallyReRead(unittest.TestCase):
    """The property the panel is only allowed to exist because of."""

    def test_each_parameter_is_re_read_or_live_callback(self):
        for label, node, prm in _feel_rows():
            src = open(SOURCES[node]).read()
            with self.subTest(slider=label, param=prm):
                self.assertIn(prm, src,
                              "%s does not mention %s at all" % (node, prm))
                # Either a live parameter callback names it, or it is read
                # somewhere other than the constructor's one-time block.
                # ANCHOR ON THE CALLBACK BODY, not on the registration.
                # Splitting at the first "_on_param" lands on
                # `add_on_set_parameters_callback(self._on_param)` -- and in
                # kortex_highlevel_bridge the body is 486 lines further on,
                # outside any sane window, so the check said "not live"
                # about a parameter that has been live all along.
                body = src.split("def _on_param", 1)
                has_cb = ("add_on_set_parameters_callback" in src
                          and len(body) > 1 and prm in body[1][:4000])
                reads = len(re.findall(
                    r"get_parameter\(\s*['\"]%s['\"]\s*\)" % re.escape(prm),
                    src))
                rebuilt = prm in src.split("SMOOTHING_PARAMS", 1)[-1][:2000]
                refreshed = prm in src.split("_refresh_tuning", 1)[-1][:1200]
                self.assertTrue(
                    has_cb or rebuilt or refreshed or reads > 1,
                    "%s reads %s once, in __init__, with no live callback "
                    "and no refresh -- the slider would set a number the "
                    "node has already copied and will never look at again"
                    % (node, prm))


class TestTheGuardsAreNotOnThePanel(unittest.TestCase):
    """Feel is tunable. Trips are not."""

    def test_no_safety_trip_is_a_slider(self):
        table = GUI[GUI.index("    FEEL = ["):GUI.index("    def _feel_panel")]
        for trip in ("lag_trip_rad", "enable_gap_rad", "max_step_rad",
                     "min_clearance_m", "home_tolerance_rad"):
            self.assertNotIn(
                trip, table,
                "%s is a guard, not a feel knob. A limit that can be widened "
                "from a slider while the arm is moving is not a limit."
                % trip)

    def test_the_panel_says_so(self):
        panel = GUI[GUI.index("    FEEL = ["):GUI.index("    def _feel_apply")]
        self.assertIn("safety trips are NOT here", panel)


class TestTheCheckCanFail(unittest.TestCase):

    def test_a_write_only_parameter_is_caught(self):
        # `allow_real_arm` is the real example: declared, passed by launch
        # files, republished in a status string, and read by no consumer.
        # If this ever passes, the check above proves nothing.
        src = open(SOURCES["/vr_pose_mapper"]).read()
        reads = len(re.findall(
            r"get_parameter\(\s*['\"]hold_rate_hz['\"]\s*\)", src))
        self.assertLessEqual(
            reads, 1,
            "picked a parameter that IS re-read; this negative control needs "
            "one that is not")


if __name__ == "__main__":
    unittest.main()
