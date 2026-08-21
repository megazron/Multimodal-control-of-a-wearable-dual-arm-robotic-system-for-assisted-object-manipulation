"""The bridge must SET the servoing mode, not assume it.

The servoing mode is ARM state and survives a session ending, including a
crashed one. Measured 2026-08-21: the left arm was found in
LOW_LEVEL_SERVOING. The bridge connected cleanly, streamed joint states at
full rate, accepted a homing request -- and every speed command was rejected
with "Invalid command for the current servoing mode". The arm never moved,
while the homing node reported 176 deg of error as though it had.

Everything downstream looked healthy. That is the whole reason this has to be
checked and corrected at connect time rather than diagnosed later.
"""
import pathlib

SRC = (pathlib.Path(__file__).resolve().parents[1] / "srl_teleop"
       / "kortex_highlevel_bridge.py").read_text()


def test_the_bridge_reads_the_servoing_mode_on_connect():
    assert "GetServoingMode()" in SRC


def test_it_SETS_single_level_servoing_when_it_is_wrong():
    assert "SetServoingMode(" in SRC
    assert "SINGLE_LEVEL_SERVOING" in SRC


def test_it_reads_the_mode_back_and_refuses_if_it_did_not_take():
    """Setting and not checking is the 'feature present but does nothing'
    shape: the call can succeed while the arm stays where it was."""
    i = SRC.index("SetServoingMode(m)")
    window = SRC[i:i + 700]
    assert "GetServoingMode()" in window, "no read-back after the set"
    assert "raise RuntimeError" in window, "no refusal if the set did not take"


def test_the_correction_is_announced_rather_than_silent():
    i = SRC.index("if mode != Base_pb2.SINGLE_LEVEL_SERVOING:")
    window = SRC[i:i + 500]
    assert "get_logger().warn" in window, (
        "silently correcting arm state hides the fact that something left "
        "the arm in the wrong mode")


def test_the_check_happens_BEFORE_the_arm_state_is_reported():
    """Order matters: reporting ARMSTATE_SERVOING_LOW_LEVEL and then carrying
    on is exactly what happened, and it read as informational."""
    assert (SRC.index("GetServoingMode()")
            < SRC.index('self.get_logger().info("session created'))
