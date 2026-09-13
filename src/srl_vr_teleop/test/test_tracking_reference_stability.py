#!/usr/bin/env python3
"""The tracking reference is a physical object and it can be knocked.

DESK OPERATION. The operator sits across the room facing the wearer, holds the
controllers as a 6-DOF input, and watches the real robot. The headset is worn
by nobody -- it stands on a shelf and is the ONLY thing giving the controllers
an absolute position.

WHY THIS NEEDS ITS OWN CHECK. Every other failure in this system announces
itself: a dropout stops the frames, an occlusion invalidates a pose, a dead
link stalls the rate. A nudged reference announces nothing at all. The poses
stay valid, the rate stays at 90 Hz, the controllers stay tracked -- and every
pose after the nudge is expressed in a frame that has silently rotated, so the
operator's calibrated yaw is now wrong by however far it turned. It is
indistinguishable from correct operation from inside ROS, which is exactly the
"data fresh but never changes" family in docs/ENGINEERING_LOG.md's instrument table, one
level out.

Each test below fails against the node as it was, which had no notion of a
reference at all.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped                        # noqa: E402
from std_msgs.msg import Bool                                    # noqa: E402
from srl_vr_teleop.vr_safety_node import VrSafety                # noqa: E402
from srl_vr_teleop.vr_pose_mapper import VrPoseMapper            # noqa: E402


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def safety(ros):
    n = VrSafety()
    yield n
    n.destroy_node()


def hmd(x=0.0, y=0.0, z=1.2, q=(0.0, 0.0, 0.0, 1.0)):
    m = PoseStamped()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    (m.pose.orientation.x, m.pose.orientation.y,
     m.pose.orientation.z, m.pose.orientation.w) = q
    return m


def yaw_q(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


# ------------------------------------------------------------- the latch
def test_the_first_hmd_pose_becomes_the_reference(safety):
    assert safety.ref_hmd is None
    safety._on_hmd(hmd(0.1, 0.2, 1.3))
    assert safety.ref_hmd is not None
    assert np.allclose(safety.ref_hmd[0], [0.1, 0.2, 1.3])


def test_a_stationary_headset_never_trips_it(safety):
    """It must survive ordinary tracking noise or it will cry wolf and be
    switched off, which is worse than not having it."""
    safety._on_hmd(hmd())
    rng = np.random.default_rng(0)
    for _ in range(200):
        n = rng.normal(scale=0.002, size=3)          # 2 mm of jitter
        safety._on_hmd(hmd(n[0], n[1], 1.2 + n[2]))
    assert not safety._reference_bad(), safety.ref_moved
    assert not safety.frozen or 'REFERENCE' not in safety.freeze_reason


# ------------------------------------------------------------ the trip
def test_a_nudge_in_TRANSLATION_freezes(safety):
    safety._on_hmd(hmd())
    safety.frozen = False
    safety._on_hmd(hmd(0.0, 0.0, 1.2 + 0.05))        # 50 mm
    assert safety._reference_bad()
    assert safety.frozen
    assert 'REFERENCE MOVED' in safety.freeze_reason


def test_a_nudge_in_ROTATION_freezes(safety):
    """The one that matters most: a 10 deg twist barely moves the headset but
    rotates the operator's whole frame, so the arm goes 10 deg wrong in every
    subsequent command."""
    safety._on_hmd(hmd())
    safety.frozen = False
    safety._on_hmd(hmd(q=yaw_q(10.0)))
    assert safety._reference_bad()
    assert 'REFERENCE MOVED' in safety.freeze_reason
    d, a = safety.ref_moved
    assert d < 0.001, "it barely moved in position -- that is the point"
    assert 9.0 < a < 11.0, a


def test_the_ordinary_unfreeze_path_cannot_clear_it(safety):
    """The trap this guards. Poses keep flowing the whole time the reference
    is wrong, so the generic 'poses are fine again, observer present' unfreeze
    is true throughout and would silently resume."""
    safety._on_hmd(hmd())
    safety.observer_ok = True
    safety._on_hmd(hmd(0.0, 0.10, 1.2))
    assert safety.frozen
    import time as _t
    safety.last_pose_t = _t.monotonic()              # poses ARE flowing
    safety._tick()
    assert safety.frozen, "a moved reference must survive the unfreeze path"
    assert safety._reference_bad()


def test_rebase_is_deliberate_and_demands_recalibration(safety):
    """Re-latching automatically would absorb the very shift being detected."""
    from std_srvs.srv import Trigger
    safety._on_hmd(hmd())
    safety._on_hmd(hmd(0.0, 0.20, 1.2))
    assert safety._reference_bad()
    res = safety._srv_rebase(Trigger.Request(), Trigger.Response())
    assert res.success
    assert safety.ref_hmd is None and safety.ref_moved is None
    assert 'calibrate_operator_yaw' in res.message, (
        'the measured heading belonged to the OLD reference; saying so is the '
        'whole point of making this deliberate')
    assert not safety._reference_bad()


def test_it_can_be_switched_off_for_a_worn_session(safety):
    """Head-worn operation moves the HMD constantly and by design."""
    safety.set_parameters([rclpy.parameter.Parameter(
        'watch_tracking_reference', rclpy.Parameter.Type.BOOL, False)])
    safety._on_hmd(hmd())
    safety._on_hmd(hmd(0.0, 0.9, 1.6))
    assert not safety._reference_bad()


# --------------------------------------------- the mapper actually stops
def test_the_mapper_honours_the_safety_freeze(ros):
    """vr_safety_node computed a freeze and published it, and NOTHING
    consumed it. So 'the observer e-stop was withdrawn' and 'the tracking
    reference moved' were both states the system could be in while still
    commanding the arm."""
    m = VrPoseMapper()
    try:
        assert m.safety_frozen is False, 'must default off for stacks with ' \
                                         'no vr_safety_node running'
        for h in ('left', 'right'):
            m.engaged[h] = True
            m.filt[h] = np.zeros(3)
        b = Bool()
        b.data = True
        m._on_safety_freeze(b)
        assert m.safety_frozen
        for h in ('left', 'right'):
            assert m.engaged[h] is False, 'freeze must drop the clutch'
            assert m.filt[h] is None
        import inspect
        src = inspect.getsource(VrPoseMapper._tick)
        assert 'self.safety_frozen or not self.tracking_ok' in src
        eng = inspect.getsource(VrPoseMapper._engage)
        assert 'safety node is holding a freeze' in eng, \
            're-gripping must not defeat it'
    finally:
        m.destroy_node()
