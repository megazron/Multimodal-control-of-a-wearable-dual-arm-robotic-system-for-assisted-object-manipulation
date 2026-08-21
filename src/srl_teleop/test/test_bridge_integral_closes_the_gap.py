"""The bridge's integral term: it must exist, be bounded, and be reset.

WHY THIS TEST EXISTS. The high-level bridge had proportional feedback only.
Measured on the real arms 2026-08-21, both at rest at home: joints 2-6 all sat
0.10-0.11 deg from target with the SAME SIGN, joint_1 opposite, and joint_7
exact to 0.001 deg -- joint_7 being the wrist roll, the one joint that carries
no gravity moment. That is gravity sag, and proportional control cannot remove
a constant offset. Teleoperation therefore tracked the simulation to ~1 deg per
joint while `real_homing_node`, which has ki, closed to 0.09 deg.

The bound and the reset are tested as hard as the term itself: an unbounded
integrator winds up whenever the arm cannot follow and dumps it as a lurch the
moment motion is permitted -- next to somebody's head, on this rig.
"""
import pathlib
import re

SRC = (pathlib.Path(__file__).resolve().parents[1] / "srl_teleop"
       / "kortex_highlevel_bridge.py").read_text()


def test_the_bridge_has_an_integral_term_at_all():
    assert 'self.declare_parameter("ki"' in SRC
    assert "self.ki * self._integ[i]" in SRC, (
        "the integral state must actually reach the commanded speed; storing "
        "it and not adding it is the 'feature present but does nothing' shape")


def test_the_integrator_is_bounded():
    assert "integ_max_rad_s" in SRC
    assert re.search(r"self\._integ\[i\]\s*=\s*clip\(self\._integ\[i\]", SRC), \
        "an unbounded integrator dumps stored error as a lurch"


def test_the_integrator_keeps_working_INSIDE_the_deadband():
    """The deadband is exactly where the last tenth of a degree lives.

    If the integral is frozen inside the deadband it cannot close the gap the
    deadband creates, and the term is decorative.
    """
    body = SRC.split("dt_i = 1.0 / max(self.rate, 1.0)", 1)[1].split("try:", 1)[0]
    # The integral update now sits in the `if use_i:` block, which runs for
    # every joint regardless of the deadband -- the deadband only gates the
    # PROPORTIONAL term. What must be true is that nothing about the deadband
    # can skip the integral update.
    blk = body.split("if use_i:", 1)[1]
    assert "self._integ[i] +=" in blk
    prop = body.split("if abs(delta[i]) <= self.deadband:", 1)[1].split("if use_i:", 1)[0]
    assert "self._integ" not in prop, (
        "the integral must not be gated by the deadband -- the deadband is "
        "exactly where the last tenth of a degree lives")


def test_every_zero_speed_path_RESETS_the_integrator():
    """estop, no-target and watchdog must all clear it."""
    for marker in ('reason = "estop"',
                   'reason = "no target yet"',
                   'reason = "watchdog'):
        idx = SRC.index(marker)
        window = SRC[idx:idx + 400]
        assert "self._integ = [0.0] * NJ" in window, (
            "the %r path does not reset the integrator" % marker)


def test_ki_is_live_tunable_like_the_other_gains():
    live = SRC.split("live = {", 1)[1].split("}", 1)[0]
    assert '"ki"' in live and '"integ_max_rad_s"' in live


def test_changing_ki_clears_the_accumulated_integral():
    """A new gain applied to an old integral is a step the operator did not ask
    for."""
    idx = SRC.index('elif prm.name == "ki":')
    assert "self._integ = [0.0] * NJ" in SRC[idx:idx + 260]


def test_the_integral_is_SUPPRESSED_when_the_sender_supplies_velocities():
    """TWO INTEGRATORS IN SERIES MADE THE ARM WORSE.

    `real_homing_node` computes its own kp/ki law and fills velocities[]; this
    node is meant to be a pure executor for that path. Adding a second
    integral integrated an error another integrator was already closing.
    Measured on the real arm: homing settled to 0.19-0.88 deg where the same
    node had reached 0.03-0.12 deg before the term was added.
    """
    assert "use_i = (t_vel is None) and self.ki > 0.0" in SRC
    i = SRC.index("use_i = (t_vel is None)")
    window = SRC[i:i + 260]
    assert "self._integ = [0.0] * NJ" in window, (
        "the integral state must be cleared when it is not in use, or it "
        "returns stale on the next position-mode command")
    body = SRC.split("dt_i = 1.0 / max(self.rate, 1.0)", 1)[1].split("try:", 1)[0]
    assert "if use_i:" in body, "the integral is applied unconditionally"
