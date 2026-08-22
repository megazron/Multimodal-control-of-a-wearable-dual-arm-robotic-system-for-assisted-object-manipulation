#!/usr/bin/env python3
"""Everything the Kinova arm already reports, and the gate that needs a baseline.

WHY. `kortex_highlevel_bridge` read exactly one thing from the arm --
`GetMeasuredJointAngles()`, seven numbers -- and that was the entire sensory
input this project has ever had from real hardware.

That is most of "everything works in simulation and then real hardware messes
up". In simulation there is no friction, no gravity sag, no contact and no
thermal derating; on the arm there are all four and nothing could see any of
them. A system that cannot tell its gripper has hit something keeps pushing.

`BaseCyclic.RefreshFeedback()` costs the SAME ONE ROUND TRIP and returns, per
actuator, position, velocity, torque, motor current, voltage and temperature
-- and at the base, the arm's own external-wrench estimate at the tool, the
IMU, and the fault banks.

THE HARD PART IS THE WRENCH, and these tests are mostly about it. The Gen3's
tool wrench includes the arm's own gravity model and whatever the gripper
weighs, so its zero is NOT zero and depends on pose. A fixed threshold either
fires on gravity or never fires at all, and the code has to SAY which
situation it is in.
"""
import math
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))

from srl_teleop import arm_telemetry as AT                   # noqa: E402

BRIDGE = os.path.join(WS, "src/srl_teleop/srl_teleop/"
                          "kortex_highlevel_bridge.py")


def test_the_module_self_test_passes():
    assert AT.self_test(verbose=False)


def test_every_actuator_field_is_parsed():
    t = AT.from_feedback(AT._frame(pos_deg=10.0, torque=2.5, temp=30.0))
    for name in ("position", "velocity", "torque", "current", "voltage",
                 "temperature"):
        assert len(getattr(t, name)) == AT.NJ, name
    assert abs(t.torque[0] - 2.5) < 1e-9
    assert abs(t.velocity[0] - math.radians(0.5)) < 1e-12, (
        "velocity must be converted to rad/s; Kortex reports deg/s")


def test_a_firmware_that_omits_a_field_says_which():
    """A firmware that does not publish temperature is a DIFFERENT situation
    from an arm that is cold, and defaulting to zero conflates them."""
    bare = AT._FakeFeedback([AT._FakeAct(position=0.0)
                             for _ in range(AT.NJ)], AT._FakeBase())
    t = AT.from_feedback(bare)
    assert "torque" in t.missing_fields
    assert "tool_external_wrench_force_x" in t.missing_fields


@pytest.mark.parametrize("temp,want", [(25.0, "ok"), (62.0, "warn"),
                                       (80.0, "fault")])
def test_overheating_is_reported_because_a_derated_arm_looks_like_a_bug(
        temp, want):
    """The arm derates itself BEFORE it faults, and a derated arm presents
    as an arm that has stopped keeping up for no visible reason."""
    st, why = AT.from_feedback(AT._frame(temp=temp)).overheating()
    assert st == want, why


def test_a_fault_bank_is_reported_with_its_value():
    f, why = AT.from_feedback(AT._frame(faults=(0x40, 0))).faulted()
    assert f and "0x40" in why
    assert "refuse to servo" in why, (
        "from upstream a fault bank looks exactly like an arm that has "
        "stopped following, so the reason has to say what it really is")


# ------------------------------------------------------------ the wrench
def test_without_a_baseline_the_gate_says_it_is_uncalibrated():
    c, why = AT.from_feedback(AT._frame(fx=1.0)).in_contact()
    assert not c
    assert "nobody has calibrated" in why
    c, why = AT.from_feedback(AT._frame(fx=20.0)).in_contact()
    assert c
    assert "may be gravity" in why, (
        "a fixed gate cannot tell contact from the arm's own gravity model, "
        "and it must say so rather than assert contact")


def test_a_baseline_separates_a_constant_load_from_a_real_push():
    """THE CHECK THAT MATTERS. A heavy but CONSTANT gravity load must not
    read as contact; a small change on top of it must."""
    free = [AT.from_feedback(AT._frame(fx=12.0 + 0.05 * (i % 5), fz=-9.0))
            for i in range(200)]
    bl = AT.contact_baseline(free)

    steady, why = AT.from_feedback(
        AT._frame(fx=12.1, fz=-9.0)).in_contact(bl)
    assert not steady, "a 15 N constant load read as contact: %s" % why

    pushed, why = AT.from_feedback(
        AT._frame(fx=12.1, fz=-16.0)).in_contact(bl)
    assert pushed, "7 N on top of the baseline was missed: %s" % why

    # and the fixed gate cannot make that distinction, which is the point
    assert AT.from_feedback(AT._frame(fx=12.1, fz=-9.0)).in_contact()[0], (
        "if the fixed gate did NOT fire on the steady load, this test "
        "proves nothing about why the baseline is needed")


def test_a_baseline_from_a_handful_of_samples_is_refused():
    few = [AT.from_feedback(AT._frame(fx=1.0)) for _ in range(10)]
    with pytest.raises(ValueError) as e:
        AT.contact_baseline(few)
    assert "at least" in str(e.value)


# ------------------------------------------------------- the bridge reads it
def test_the_bridge_asks_for_the_full_feedback():
    """A parser nothing calls is dead code, and the point was to stop
    reading seven numbers."""
    src = open(BRIDGE).read()
    assert "RefreshFeedback()" in src, (
        "the bridge does not call RefreshFeedback, so it is still reading "
        "joint angles only")
    assert "arm_telemetry" in src
    assert "BaseCyclicClient" in src


def test_the_bridge_still_has_a_fallback_and_says_when_it_uses_it():
    src = open(BRIDGE).read()
    assert "GetMeasuredJointAngles()" in src, (
        "the old call must remain as a fallback -- an arm whose cyclic "
        "client fails should still report joint angles")
    assert "joint angles ONLY" in src or "joint angles only" in src, (
        "falling back silently means reading a tenth of the data with no "
        "sign that anything changed")


def test_the_bridge_publishes_the_telemetry():
    src = open(BRIDGE).read()
    assert "/real/telemetry_" in src
    assert "tel_pub.publish" in src, (
        "reading the data and not publishing it is the 'feature present but "
        "does nothing' shape")


def test_the_high_level_write_path_is_untouched():
    """HARD CONSTRAINT 4: the Kortex CYCLIC path is unusable over WSL. This
    change reads feedback through the high-level API and must not have
    introduced a cyclic write."""
    src = open(BRIDGE).read()
    assert "SendJointSpeedsCommand" in src
    for banned in ("BaseCyclic_pb2.Command", "self._cyclic.Refresh(",
                   "ActuatorCommand"):
        assert banned not in src, (
            "%s appears in the bridge -- that is the cyclic WRITE path, "
            "which HARD CONSTRAINT 4 forbids over WSL" % banned)
