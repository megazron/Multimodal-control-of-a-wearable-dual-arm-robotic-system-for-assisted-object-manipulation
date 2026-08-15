"""The sim home moved and the real arms did not. The bridge must REFUSE.

On 2026-08-15 `config/home_positions_*.txt` became the presentation pose while
the physical arms stayed at the legacy Kortex home. `sim_to_real_bridge`
replays SIM joint angles onto the real arm starting from home, so that
disagreement is exactly the situation its `require_homed` gate exists for: a
~1.9 rad difference commanded in one step would be a jump on a robot worn by a
person.

WHAT THIS TESTS AND WHAT IT DOES NOT. It exercises the gate's PREDICATE --
`pose_delta_rad(home, real, CONTINUOUS_IDX)` against `home_tolerance_rad`,
which is the arithmetic `enable()` performs -- using the real constants: the
home this repository now loads, and the legacy Kortex angles the arms are
actually parked at. It does not stand up the node, so it is not a test of the
node's lifecycle; it is a test that the numbers the node compares fall the
refusing side of its threshold, and that is the claim being made in
docs/system/home_wrist_is_real.md.

It is written to fail if the home is ever quietly moved back, too: if someone
restores the legacy pose the gap goes to zero and the assertion that it
EXCEEDS the tolerance fires.
"""
import math
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "config"))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))

from srl_teleop.kortex_convention import (                    # noqa: E402
    kortex_list_to_ros, pose_delta_rad)

# The gate's own default, from sim_to_real_bridge's declare_parameter.
HOME_TOLERANCE_RAD = 0.05
CONTINUOUS_IDX = (0, 2, 4, 6)

# WHERE THE PHYSICAL ARMS ARE PARKED. The legacy real-robot home, in Kortex
# degrees, straight off the hardware and recorded in the header of both
# config/home_positions_*.txt files before the 2026-08-15 change.
LEGACY_KORTEX = {
    "left": [259.03, 277.69, 267.74, 286.14, 194.10, 27.48, 55.26],
    "right": [303.65, 77.06, 98.57, 58.57, 317.14, 36.39, 154.71],
}


def _gap(arm):
    import home_positions as hp
    sim_home = hp.load_home_radians(arm)
    real_now = kortex_list_to_ros(LEGACY_KORTEX[arm])
    d = pose_delta_rad(sim_home, real_now, CONTINUOUS_IDX)
    return max(abs(x) for x in d), d


def test_the_gap_is_real_and_large():
    for arm in ("left", "right"):
        worst, _ = _gap(arm)
        assert worst > HOME_TOLERANCE_RAD, (
            "%s arm: the loaded home is %.4f rad from where the physical arm "
            "is parked, INSIDE the bridge's %.2f rad tolerance. Either the "
            "home was moved back to the legacy pose, or the arms were "
            "recaptured and LEGACY_KORTEX in this test is stale -- update it "
            "and say so." % (arm, worst, HOME_TOLERANCE_RAD))


def test_the_gap_is_big_enough_that_it_cannot_be_missed():
    """Not a near-miss on the threshold: it is over a radian on both arms."""
    for arm in ("left", "right"):
        worst, _ = _gap(arm)
        assert worst > 1.0, (
            "%s arm gap is only %.4f rad (%.1f deg). The documented figure is "
            "~1.9 rad, forty times the gate's tolerance; if it has shrunk, the "
            "claim in home_wrist_is_real.md needs re-stating."
            % (arm, worst, math.degrees(worst)))


def test_a_recaptured_arm_would_pass():
    """The gate is not simply always-refuse -- the control for the above.

    An arm parked AT the loaded home must fall inside the tolerance, or the
    two tests above would pass for a gate that can never open and they would
    prove nothing about the home change.
    """
    import home_positions as hp
    for arm in ("left", "right"):
        sim_home = hp.load_home_radians(arm)
        d = pose_delta_rad(sim_home, list(sim_home), CONTINUOUS_IDX)
        worst = max(abs(x) for x in d)
        assert worst <= HOME_TOLERANCE_RAD, (
            "%s: an arm sitting exactly at the loaded home measures %.4f rad "
            "from it, so the gate could never open." % (arm, worst))
