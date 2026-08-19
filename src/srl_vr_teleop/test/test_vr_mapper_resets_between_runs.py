#!/usr/bin/env python3
"""vr_pose_mapper must begin every run in the state it begins its life in.

THE MEASURED FAILURE THIS PINS. 02_vr_teleop lost every grasping task while
the other four modes closed on all four cubes at 0.0000 m. The misses were
88.1, 115.4, 156.2 and 205.9 mm against a 30 mm capture gate, and the
increments -- 27.3, 40.8, 49.7 mm -- are the diagnostic: a constant scale
error repeats ONE number (that is how the earlier `scale:=0.5` bug was
identified, by the same 0.189 m appearing twice), a growing miss does not.
Re-recording with a fresh mapper PROCESS per clip fixed all five cells. So the
carried state was the cause, and the fix belonged in the node.

Each test below is written so it FAILS against the node as it was: no
reset(), no service, no disengage on tracking loss, no lag measurement.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")
from srl_vr_teleop.vr_pose_mapper import VrPoseMapper                # noqa: E402
from std_msgs.msg import Bool                                        # noqa: E402


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


def _dirty(n, hand="left"):
    """Put the node into the state a finished run leaves it in."""
    n.engaged[hand] = True
    n.p_ref[hand] = np.array([0.10, 0.20, 0.30])
    n.q_ref[hand] = np.array([0.0, 0.0, 0.0, 1.0])
    n.p_anchor[hand] = np.array([0.40, 0.50, 0.60])
    n.q_anchor[hand] = np.array([0.0, 0.0, 0.0, 1.0])
    n.filt[hand] = np.array([0.41, 0.52, 0.63])
    n.last_cmd[hand] = np.array([0.41, 0.52, 0.63])
    n.ctrl[hand] = (np.array([0.5, 0.5, 0.5]), np.array([0., 0., 0., 1.]))
    n.ctrl_hist[hand] = [(0.0, np.zeros(3))]
    n.jump[hand] = [0.004]
    n.lag[hand] = 0.087
    n.lag_max[hand] = 0.206
    n.scale = 1.7                       # the thumbstick moved it


# --------------------------------------------------------------- the reset
def test_reset_clears_every_piece_of_per_run_state(mapper):
    _dirty(mapper)
    mapper.reset("test")
    for h in ("left", "right"):
        assert mapper.engaged[h] is False
        assert mapper.p_ref[h] is None, "a stale reference re-bases the whole run"
        assert mapper.q_ref[h] is None
        assert mapper.p_anchor[h] is None, "a stale anchor is the run's origin"
        assert mapper.q_anchor[h] is None
        assert mapper.filt[h] is None, (
            "filt is the one that ACCUMULATES: it is rate limited, so it "
            "cannot catch up while the motion continues")
        assert mapper.last_cmd[h] is None
        assert mapper.ctrl_hist[h] == []
        assert mapper.jump[h] == []
        assert mapper.lag[h] == 0.0
        assert mapper.lag_max[h] == 0.0


def test_reset_restores_the_declared_scale_not_the_operators(mapper):
    """Scale is per-run state too. The thumbstick moves it live, and a scale
    of 1.7 inherited by the next participant is a different experiment."""
    declared = float(mapper.get_parameter("scale").value)
    mapper.scale = 1.7
    mapper.reset("test")
    assert mapper.scale == declared


def test_reset_is_counted_and_reported(mapper):
    before = mapper.n_resets
    mapper.reset("test")
    mapper.reset("test")
    assert mapper.n_resets == before + 2


def test_the_reset_service_exists_and_answers(mapper):
    """The harness calls this by name. If it is renamed, mode_upstreams'
    _reset_mapper() refuses the mode, so the failure is loud -- but only if
    the service is really there."""
    from std_srvs.srv import Trigger
    names = dict(mapper.get_service_names_and_types())
    assert "/vr/reset" in names, sorted(names)
    assert "std_srvs/srv/Trigger" in names["/vr/reset"]
    _dirty(mapper)
    res = mapper._srv_reset(Trigger.Request(), Trigger.Response())
    assert res.success
    assert mapper.p_anchor["left"] is None


# ------------------------------------------------------ the fault it fixes
def test_a_second_run_on_a_carried_filter_misses_and_the_miss_GROWS(mapper):
    """The mechanism, arithmetic only, no ROS, no renderer.

    Ground truth is constructed: a hand moving at a constant speed above the
    rate limit. The commanded pose must fall behind by a distance that
    increases every tick, and reset() must return the miss to zero.
    """
    h = "left"
    dt = 1.0 / float(mapper.get_parameter("rate_hz").value)
    mx = float(mapper.get_parameter("max_speed_mps").value) * dt
    hand_speed = 4.0 * float(mapper.get_parameter("max_speed_mps").value)

    mapper.engaged[h] = True
    mapper.tracking_ok = True
    mapper.p_ref[h] = np.zeros(3)
    mapper.q_ref[h] = np.array([0., 0., 0., 1.])
    mapper.p_anchor[h] = np.zeros(3)
    mapper.q_anchor[h] = np.array([0., 0., 0., 1.])
    mapper.filt[h] = np.zeros(3)
    mapper.scale = 1.0

    lags = []
    for i in range(1, 60):
        want = np.array([hand_speed * i * dt, 0.0, 0.0])
        step = want - mapper.filt[h]
        n = float(np.linalg.norm(step))
        if n > mx:
            step *= mx / n
        mapper.filt[h] = mapper.filt[h] + step
        lags.append(float(np.linalg.norm(want - mapper.filt[h])))

    assert lags[-1] > lags[0], "the lag must GROW; a constant offset is a " \
                               "different fault with a different fix"
    assert lags[-1] > 0.030, ("59 ticks above the rate limit should exceed the "
                              "30 mm capture gate, got %.4f m" % lags[-1])
    # increments strictly positive: this is accumulation, not an offset
    inc = np.diff(lags)
    assert (inc > 0).all()

    mapper.lag[h] = lags[-1]
    mapper.reset("test")
    assert mapper.filt[h] is None and mapper.lag[h] == 0.0


# --------------------------------------------------- tracking loss + clutch
def test_tracking_loss_drops_the_clutch_rather_than_only_pausing(mapper):
    """Freezing while STAYING engaged means resuming from a filter latched
    before the dropout against a controller that moved during it -- a jump
    the operator did not command and cannot see coming. Dropping the clutch
    forces a re-latch, which is zero by construction."""
    for h in ("left", "right"):
        _dirty(mapper, h)
    ok = Bool(); ok.data = True
    mapper._on_tracking(ok)
    assert mapper.engaged["left"] is True

    lost = Bool(); lost.data = False
    mapper._on_tracking(lost)
    for h in ("left", "right"):
        assert mapper.engaged[h] is False, "clutch must drop on tracking loss"
        assert mapper.filt[h] is None, "and the stale filter must go with it"
    assert mapper.tracking_ok is False


def test_regripping_after_a_dropout_relatches_at_the_current_pose(mapper, monkeypatch):
    """The re-engage jump the study reports must be zero BY CONSTRUCTION, and
    it stays zero across a dropout only because the clutch was dropped."""
    h = "left"
    robot = (np.array([0.30, 0.40, 1.10]), np.array([0., 0., 0., 1.]))
    monkeypatch.setattr(mapper, "_robot_pose", lambda arm: robot)
    monkeypatch.setattr(mapper, "_quiet", lambda hand: True)

    mapper.ctrl[h] = (np.array([0.9, 0.1, 0.2]), np.array([0., 0., 0., 1.]))
    mapper.last_cmd[h] = robot[0].copy()
    mapper._engage(h)
    assert mapper.engaged[h]

    lost = Bool(); lost.data = False
    mapper._on_tracking(lost)
    # the hand wandered 40 cm while nothing was tracked
    mapper.ctrl[h] = (np.array([1.3, 0.1, 0.2]), np.array([0., 0., 0., 1.]))
    ok = Bool(); ok.data = True
    mapper._on_tracking(ok)
    mapper._engage(h)

    assert np.allclose(mapper.p_ref[h], [1.3, 0.1, 0.2]), (
        "the reference must re-latch at where the hand IS now")
    assert np.allclose(mapper.p_anchor[h], robot[0])
    assert max(mapper.jump[h]) < 1e-9, (
        "re-engage jump must be zero; got %s" % mapper.jump[h])


# --------------------------------------------------------- the measurement
def test_the_engage_metric_is_not_called_a_clutch_jump(mapper):
    """It was, and it is not one.

    At engage the command is set to `pr` -- the arm's OWN pose from TF -- so
    the jump the ARM makes is zero by construction. `last_cmd - pr` measures
    something else: how far the FOLLOWER had fallen behind its last target.
    Publishing that as `max_reengage_jump_m` puts a number in the write-up
    that says the clutch is bad when the clutch is fine. Measured on a real
    run: 150 mm of "re-engage jump" on an engage across which the arm moved
    zero.
    """
    import inspect
    src = inspect.getsource(VrPoseMapper._tick)
    assert 'mean_follower_lag_at_engage_m' in src
    assert 'max_follower_lag_at_engage_m' in src
    assert 'max_reengage_jump_m=' not in src, (
        'the follower-lag figure must not be published under a name that '
        'reads as a clutch discontinuity')
    eng = inspect.getsource(VrPoseMapper._engage)
    assert 'TRACKING ERROR' in eng, 'say what the number is, where it is taken'


def test_lag_is_published_so_the_next_one_is_visible_not_inferred(mapper):
    """The 2026-08-16 diagnosis needed four cube misses and a re-record. The
    number itself is now on /vr/mapper_<hand>."""
    import inspect
    src = inspect.getsource(VrPoseMapper._tick)
    assert "lag_m=round(self.lag[hand]" in src
    assert "max_lag_m=round(self.lag_max[hand]" in src
    assert "self.lag[hand] = float(np.linalg.norm(p_cmd - p_out))" in src
