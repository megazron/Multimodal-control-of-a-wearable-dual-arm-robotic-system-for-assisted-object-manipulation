"""The headset panel must not sit in front of the work.

THE DEFECT, reported from the headset 2026-08-30: "that pass through panel has
such a huge banner the user is unable to keep his headset properly, affecting
the teleoperation".

It was 0.84 m wide and 0.42 m tall at one metre -- about 45 degrees of the
horizontal field -- centred 10 degrees BELOW the eye line, which is exactly
where an operator looks to see their own hands and the arm they are driving.
And it is HEAD-LOCKED: the model matrix is expressed in view space on purpose,
so turning to look past it brings it along. The operator's only remaining move
was to shift the headset on their face, which is what they were doing, and a
headset that is being held rather than worn is a tracking problem as well as a
comfort one.

A telemetry panel earns its space while you are SETTING UP and costs you while
you are REACHING, so it shrinks the moment the clutch closes and returns when
it opens.

The last group is the one that must not regress: a FREEZE is exempt. This
file's own comment in vr_client.html says freeze_reason is "the only field an
operator who cannot see the arm must never miss", and shrinking that to get a
better view would trade away the reason the panel exists.
"""
import os
import re
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
HTML = open(os.path.join(WS, "src", "srl_vr_teleop", "web",
                         "vr_client.html")).read()


def _geom(name):
    m = re.search(
        r"const %s\s*=\s*\{w:\s*([\d.]+),\s*h:\s*([\d.]+),\s*"
        r"y:\s*(-?[\d.]+),\s*a:\s*([\d.]+)\}" % name, HTML)
    assert m, "%s is not declared in the form this test reads" % name
    return dict(w=float(m.group(1)), h=float(m.group(2)),
                y=float(m.group(3)), a=float(m.group(4)))


class TestTheAButtonHidesIt(unittest.TestCase):
    """SHOW/HIDE IS A DECISION, NOT A CONTINUOUS SIGNAL.

    It was first wired to the clutch -- shrink while driving -- and the
    operator corrected it the same day: "make it disappear when A is pressed,
    not the clutch button". They are right. The clutch changes many times a
    minute, so tying the view to it makes the panel flicker in and out while
    working, and the moment you most want telemetry is often mid-reach with
    the grip closed. Visibility is decided once and stays decided.
    """

    def test_hidden_means_hidden(self):
        self.assertEqual(_geom("PANEL_OFF")["a"], 0.0,
                         "the operator asked for it GONE -- a shrunken panel "
                         "is still a thing in the way")

    def test_the_toggle_is_edge_triggered(self):
        self.assertIn("aDown && !aWasDown", HTML,
                      "a held button must toggle ONCE, not sixty times a "
                      "second")

    def test_the_clutch_no_longer_controls_visibility(self):
        self.assertNotIn("PANEL_DRIVE", HTML)
        self.assertNotIn("GRIP_ENGAGED", HTML,
                         "the clutch threshold has no business in a "
                         "visibility decision any more")

    def test_a_guided_step_keeps_its_own_use_of_A(self):
        self.assertIn("!state.instruction) panelHidden", HTML,
                      "a guided step owns the panel and prints 'press A when "
                      "finished'; the toggle must stand aside there rather "
                      "than fight it for the button")

    def test_the_alpha_actually_reaches_the_shader(self):
        self.assertIn("uniform float alpha", HTML)
        self.assertIn("c.a*alpha", HTML,
                      "declared and unused would be this session's own "
                      "recurring bug, in a shader")
        self.assertIn("uniform1f(gl.getUniformLocation(quadProg, 'alpha')",
                      HTML)


class TestItReactsInTheSameFrame(unittest.TestCase):
    """Read the LOCAL gamepad, not the robot state coming back over the
    socket. The panel must respond in the frame the button is pressed;
    waiting for the bridge to echo the button back puts a network round trip
    between pressing and seeing."""

    def test_the_toggle_reads_the_local_buttons(self):
        self.assertIn("const aDown = !!(L.a || R.a);", HTML)

    def test_either_hand_works(self):
        # The operator's other hand is often holding something, or the
        # controller is the one not currently clutched.
        self.assertIn("L.a || R.a", HTML)


class TestAFreezeIsNeverShrunk(unittest.TestCase):
    """The one thing an operator who cannot see the arm must never miss."""

    def test_frozen_forces_the_full_panel(self):
        fn = HTML[HTML.index("function panelGeom()"):
                  HTML.index("// Head-locked panel")]
        self.assertIn("state.frozen", fn)
        self.assertIn("return PANEL_IDLE", fn)
        self.assertLess(fn.index("state.frozen"), fn.index("panelHidden ?"),
                        "the freeze test must come FIRST, or an operator who "
                        "hid the panel never sees the one message they must "
                        "not miss")

    def test_a_guided_instruction_is_also_full_size(self):
        fn = HTML[HTML.index("function panelGeom()"):
                  HTML.index("// Head-locked panel")]
        self.assertIn("state.instruction", fn,
                      "a guided step owns the whole panel by design -- the "
                      "operator cannot read a terminal")


if __name__ == "__main__":
    unittest.main()
