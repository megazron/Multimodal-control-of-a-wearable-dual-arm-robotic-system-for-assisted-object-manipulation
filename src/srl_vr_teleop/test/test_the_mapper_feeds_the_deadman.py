#!/usr/bin/env python3
"""The mapper must stay a LIVE source on /master_arm_pose_* while it runs.

THE MECHANISM THIS PINS. On any real-arm launch, estop_node latches an
e-stop when /master_arm_pose_<arm> goes stale for 0.5 s on 5 consecutive
checks -- including for an arm that has NEVER published. The mapper used to
publish only inside the engaged branch, so on a real-arm VR run:

  * releasing the grip            -> topic silent -> LATCHED e-stop
  * any safety freeze             -> topic silent -> LATCHED e-stop
  * driving with one hand         -> the OTHER arm's topic silent -> e-stop

and the latch outlives the cause: the operator re-grips into a system that
now refuses for a different reason than the one that froze it, clearable
only by /estop_reset from a terminal.

The fix is not to weaken the dead-man; it is for the mapper to say something
true while it is not driving: the arm's OWN live pose, i.e. a hold-in-place
command with zero motion by construction. A freeze still stops the arm.
The dead-man still catches the mapper process dying.

Each test fails against the node as it was (no publish outside the engaged
branch).
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")
from srl_vr_teleop.vr_pose_mapper import VrPoseMapper                # noqa: E402


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mapper(ros):
    n = VrPoseMapper()
    yield n
    n.destroy_node()


HOLD_P = np.array([0.40, 0.50, 1.10])
HOLD_Q = np.array([0.0, 0.0, 0.0, 1.0])


def _instrument(n):
    """Fixed robot TF, and a counter on each hand's command publisher."""
    n._robot_pose = lambda arm: (HOLD_P.copy(), HOLD_Q.copy())
    sent = {h: [] for h in n.hands}

    class _Rec:
        def __init__(self, hand):
            self.hand = hand

        def publish(self, m):
            sent[self.hand].append(m)

    for h in n.hands:
        n.cmd_pub[h] = _Rec(h)
    # The rate limiter inside _publish_hold must not swallow the calls the
    # tests make back to back.
    n.set_parameters([rclpy.parameter.Parameter(
        'hold_rate_hz', rclpy.Parameter.Type.DOUBLE, 1e6)])
    return sent


def test_a_disengaged_hand_still_feeds_its_arm_topic(mapper):
    sent = _instrument(mapper)
    assert not any(mapper.engaged.values())
    mapper._tick()
    for h in mapper.hands:
        assert sent[h], (
            "hand %r is disengaged and published NOTHING: on a real-arm "
            "launch that is a latched e-stop 2.5 s after every clutch "
            "release, and from the first check for a hand never engaged" % h)


def test_the_hold_command_is_the_arms_own_pose(mapper):
    """Zero motion by construction: target == where the arm already is."""
    sent = _instrument(mapper)
    mapper._tick()
    m = sent["left"][0]
    got = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
    assert np.allclose(got, HOLD_P, atol=1e-12), (
        "the hold command must be the robot's LIVE pose -- anything else is "
        "a motion nobody commanded")


def test_a_freeze_commands_hold_rather_than_going_silent(mapper):
    sent = _instrument(mapper)
    h = "left"
    mapper.engaged[h] = True
    mapper.ctrl[h] = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    mapper.p_ref[h] = np.zeros(3)
    mapper.q_ref[h] = np.array([0.0, 0.0, 0.0, 1.0])
    mapper.p_anchor[h] = HOLD_P.copy()
    mapper.q_anchor[h] = HOLD_Q.copy()
    mapper.safety_frozen = True
    mapper._tick()
    assert sent[h], (
        "a safety freeze went SILENT: the arm was already going to hold "
        "(the follower keeps its last command); the only thing silence adds "
        "is a latched e-stop that outlives the freeze")
    m = sent[h][-1]
    got = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
    assert np.allclose(got, HOLD_P, atol=1e-12), (
        "under a freeze the command must be hold-in-place, never the "
        "operator's hand")


def test_no_tf_means_no_publish_not_a_guess(mapper):
    """With no robot TF there is nothing TRUE to say; a made-up pose would
    be commanded as a jump the moment the follower comes up."""
    sent = _instrument(mapper)
    mapper._robot_pose = lambda arm: (None, None)
    mapper._tick()
    assert not any(sent.values()), (
        "published a hold with no TF to base it on -- that pose is invented")


def test_the_feed_can_be_switched_off_and_says_so(mapper):
    sent = _instrument(mapper)
    mapper.set_parameters([rclpy.parameter.Parameter(
        'hold_when_idle', rclpy.Parameter.Type.BOOL, False)])
    mapper._tick()
    assert not any(sent.values())
    # And the idle heartbeat must SAY the feed is off, because an operator
    # cannot otherwise tell this mapper from one that feeds the dead-man.
    import json
    beats = {h: [] for h in mapper.hands}

    class _Rec:
        def __init__(self, hand):
            self.hand = hand

        def publish(self, m):
            beats[self.hand].append(json.loads(m.data))

    for h in mapper.hands:
        mapper.st_pub[h] = _Rec(h)
    mapper._idle_state()
    for h in mapper.hands:
        assert beats[h] and beats[h][-1]["feeding_deadman"] is False
