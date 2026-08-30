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


class TestItShrinksWhileDriving(unittest.TestCase):

    def test_the_driving_panel_is_much_smaller(self):
        idle, drive = _geom("PANEL_IDLE"), _geom("PANEL_DRIVE")
        self.assertLess(drive["w"], idle["w"] / 2.0,
                        "halving is not enough to clear the field an "
                        "operator reaches through")
        self.assertLess(drive["h"], idle["h"] / 2.0)

    def test_it_moves_out_of_the_eye_line(self):
        idle, drive = _geom("PANEL_IDLE"), _geom("PANEL_DRIVE")
        self.assertLess(drive["y"], idle["y"],
                        "further DOWN, not up -- the work is straight ahead")

    def test_it_fades(self):
        self.assertLess(_geom("PANEL_DRIVE")["a"], 1.0)
        self.assertGreater(_geom("PANEL_DRIVE")["a"], 0.2,
                           "still readable at a glance; invisible telemetry "
                           "is the opposite problem")

    def test_the_alpha_actually_reaches_the_shader(self):
        self.assertIn("uniform float alpha", HTML)
        self.assertIn("c.a*alpha", HTML,
                      "declared and unused would be this session's own "
                      "recurring bug, in a shader")
        self.assertIn("uniform1f(gl.getUniformLocation(quadProg, 'alpha')",
                      HTML)


class TestItReactsInTheSameFrame(unittest.TestCase):
    """Read the LOCAL gamepad, not the robot state coming back."""

    def test_driving_comes_from_the_local_grip(self):
        self.assertIn("driving = (L.grip > GRIP_ENGAGED)", HTML,
                      "waiting for the mapper to report engagement puts a "
                      "network round trip between squeezing and being able "
                      "to see, which is the opposite of the fix")

    def test_the_threshold_matches_the_clutch(self):
        m = re.search(r"const GRIP_ENGAGED = ([\d.]+)", HTML)
        self.assertIsNotNone(m)
        self.assertEqual(float(m.group(1)), 0.6,
                         "vr_pose_mapper._on_joy engages at 0.6; a panel "
                         "that clears at a different threshold than the "
                         "clutch is a panel that lies about the clutch")


class TestAFreezeIsNeverShrunk(unittest.TestCase):
    """The one thing an operator who cannot see the arm must never miss."""

    def test_frozen_forces_the_full_panel(self):
        fn = HTML[HTML.index("function panelGeom()"):
                  HTML.index("// Head-locked panel")]
        self.assertIn("state.frozen", fn)
        self.assertIn("return PANEL_IDLE", fn)
        self.assertLess(fn.index("state.frozen"), fn.index("driving ?"),
                        "the freeze test must come FIRST, or a driving "
                        "operator gets a shrunken freeze banner")

    def test_a_guided_instruction_is_also_full_size(self):
        fn = HTML[HTML.index("function panelGeom()"):
                  HTML.index("// Head-locked panel")]
        self.assertIn("state.instruction", fn,
                      "a guided step owns the whole panel by design -- the "
                      "operator cannot read a terminal")


if __name__ == "__main__":
    unittest.main()
