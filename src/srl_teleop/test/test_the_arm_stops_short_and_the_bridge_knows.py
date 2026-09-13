#!/usr/bin/env python3
"""The 0.305 deg terminal error: the model, the fix, and the trap in between.

WHAT WAS MEASURED. On the rig, 2026-08-21, 36 runs, both arms, six
directions: the end effector lands 7.2 mm RMS from where it was sent. In
joint space, 211 of 252 per-joint terminal errors sit at +/-0.25..0.30 deg
with the sign a coin flip. That is not a spread, it is a BOUND -- a joint
parking just inside a band, on the side it arrived from.

WHY THE SHAPE MATTERS MORE THAN THE SIZE. Every recorded move is 100 mm, so
"the arm travels 93.5% of what it is told" and "the arm stops 7.2 mm short"
fit the Cartesian data identically. On a 20 mm move they differ by a factor
of five, and a gain fitted here and applied there would be confidently wrong.
The joint data discriminates -- the recorded per-joint errors pushed through
the Jacobian reproduce the Cartesian error to 90%, r = 0.923, and a Jacobian
does not know how far the arm travelled.

So the model is one number and the compensation is "overshoot each joint by
EPS in the direction of travel". The tests below are almost all about the
ways that can be wrong:

  * the overshoot going the WRONG WAY, which doubles the error and is
    invisible on hardware;
  * the continuous seam, where +170 -> -170 deg travels +20 deg the short
    way and naive subtraction says -340 -- defect 5 of 2026-08-21, and here
    it would put the overshoot on the wrong side of every seam crossing;
  * a joint that barely moved being "corrected" in a direction it was not
    going;
  * and the one that would be worst: the compensation switched ON while
    silently applying zero.

THIS FILE REPLACED `test_sim_to_real_gap_refuses_outside_its_domain.py`,
which tested a Cartesian 3x3 correction and the elaborate set of refusals it
needed -- off-axis, off-length, off-pose. Those refusals were correct for
that model and they made it useless: almost no real motion satisfied them.
They were the symptom of fitting nine parameters to six axis-aligned
directions. The joint model needs none of them, so the tests for them are
gone rather than adapted.

AND THE TRAP THIS FILE EXISTS TO PIN. The recording is labelled with three
speeds and is NOT a speed sweep: `vmax_rad_s` was read once at bridge
start-up, and 7 of 36 runs moved FASTER than the limit they were commanded
with. An earlier version of this work reported the error as
"speed-independent" on the strength of it. How the error varies with speed
has never been measured, and `test_the_speed_sweep_is_still_known_to_be_inert`
keeps that from being forgotten.
"""
import json
import math
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
os.environ.setdefault("SRL_WS", WS)
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))

from srl_teleop import sim_to_real_gap as G                 # noqa: E402

MODEL = os.path.join(WS, "recordings/baselines/sim_to_real_gap.json")
SRC = os.path.join(WS,
                   "recordings/baselines/arm_directional_calibration.json")
BRIDGE = os.path.join(WS, "src/srl_teleop/srl_teleop/kortex_highlevel_bridge.py")

have_model = pytest.mark.skipif(
    not os.path.exists(MODEL),
    reason="no sim_to_real_gap.json; run scripts/measure_sim_to_real_gap.py")


@pytest.fixture(scope="module")
def model():
    return G.load(MODEL)


@have_model
def test_the_module_self_test_passes():
    assert G.self_test(verbose=False)


# ------------------------------------------------------------- the model
@have_model
def test_eps_is_about_a_third_of_a_degree_on_both_arms(model):
    e = {a: G.eps_rad(a, model) for a in model["joint_terminal"]}
    assert set(e) == {"left", "right"}
    for arm, v in e.items():
        assert 0.2 < math.degrees(v) < 0.5, (
            "%s EPS is %.4f deg. The measurement says 0.305; a value far "
            "from that means the model file was refitted on different data "
            "and every number in the docstrings is stale."
            % (arm, math.degrees(v)))
    assert abs(e["left"] - e["right"]) < math.radians(0.01), (
        "the two arms no longer agree on EPS. One number for both is what "
        "makes this credible as a property of the controller rather than of "
        "one machine.")


@have_model
def test_the_joint_model_beats_the_cartesian_matrix_where_it_counts(model):
    """On an axis neither was fitted on. This is the whole argument."""
    for arm, jt in model["joint_terminal"].items():
        cart = model["arms"][arm]["held_out_axis_rms_mm"]
        assert jt["held_out_axis_rms_mm"] < 0.1 * cart, (
            "%s: joint model %.2f mm, Cartesian 3x3 %.2f mm on the same "
            "hold-out. The joint model is used BECAUSE it generalises; if "
            "that stops being true, stop using it."
            % (arm, jt["held_out_axis_rms_mm"], cart))
        assert jt["held_out_axis_rms_mm"] < 0.4 * jt["identity_rms_mm"]


@have_model
def test_scrambling_the_signs_destroys_the_model(model):
    """The sign structure is the model. If a scrambled version scored as
    well, EPS would just be a fitted fudge factor."""
    for arm, jt in model["joint_terminal"].items():
        assert jt["scrambled_signs_rms_mm"] > 0.9 * jt["identity_rms_mm"], (
            "%s: scrambled signs score %.2f mm against identity %.2f. The "
            "model is supposed to be carried by WHICH WAY each joint parks."
            % (arm, jt["scrambled_signs_rms_mm"], jt["identity_rms_mm"]))


# -------------------------------------------------------- the compensation
@have_model
@pytest.mark.parametrize("q_from,q_to", [
    ([0.0] * 7, [0.5] * 7),
    ([0.0] * 7, [-0.5] * 7),
    ([0.3, -0.2, 0.1, 0.4, -0.6, 0.2, -0.1],
     [-0.4, 0.5, -0.7, 0.9, 0.2, -0.3, 0.8]),
])
def test_the_overshoot_always_goes_the_way_the_joint_is_going(q_from, q_to,
                                                              model):
    for arm in model["joint_terminal"]:
        e = G.eps_rad(arm, model)
        cmd, applied, _ = G.correct_joints(arm, q_from, q_to, model=model)
        trav = G.joint_travel(q_from, q_to)
        for i in range(7):
            if not applied[i]:
                continue
            over = cmd[i] - q_to[i]
            assert abs(abs(over) - e) < 1e-12
            assert (over > 0) == (trav[i] > 0), (
                "joint_%d travels %+.3f rad and was overshot %+.5f -- the "
                "wrong way. This DOUBLES the error and there is no sign of "
                "it on the arm." % (i + 1, trav[i], over))


@have_model
def test_a_joint_that_barely_moved_is_left_alone(model):
    for arm in model["joint_terminal"]:
        e = G.eps_rad(arm, model)
        q0 = [0.0] * 7
        q1 = [e * 0.1, e * 0.9, e * 1.1, 0.0, -e * 0.9, -e * 1.1, 0.0]
        cmd, applied, _ = G.correct_joints(arm, q0, q1, model=model)
        assert applied == [False, False, True, False, False, True, False], (
            "a joint moving less than EPS must not be overshot: 'the way it "
            "was going' is not defined for something that was not going "
            "anywhere. got %s" % applied)
        for i, a in enumerate(applied):
            if not a:
                assert cmd[i] == q1[i]


@have_model
@pytest.mark.parametrize("a_deg,b_deg,short_deg", [
    (170.0, -170.0, +20.0),      # up through 180, comes out negative
    (-170.0, 170.0, -20.0),
    (179.0, -179.0, +2.0),
    (10.0, 30.0, +20.0),
])
def test_the_continuous_seam_takes_the_short_way(a_deg, b_deg, short_deg,
                                                 model):
    """joint_1 is continuous. 170 -> -170 is +20 deg, not -340."""
    q0 = [math.radians(a_deg)] + [0.0] * 6
    q1 = [math.radians(b_deg)] + [0.0] * 6
    t = G.joint_travel(q0, q1)
    assert abs(math.degrees(t[0]) - short_deg) < 1e-9, (
        "%.0f -> %.0f deg reported as %+.1f, want %+.1f"
        % (a_deg, b_deg, math.degrees(t[0]), short_deg))
    for arm in model["joint_terminal"]:
        cmd, _, _ = G.correct_joints(arm, q0, q1, model=model)
        over = cmd[0] - q1[0]
        assert (over > 0) == (short_deg > 0), (
            "the seam overshoot went the wrong way on %s" % arm)


@have_model
def test_a_joint_with_hard_stops_does_not_wrap(model):
    """joint_2 has real limits, so 170 -> -170 really is -340 for it. Wrapping
    it would invent a path through a mechanical stop."""
    q0 = [0.0, math.radians(170.0)] + [0.0] * 5
    q1 = [0.0, math.radians(-170.0)] + [0.0] * 5
    t = G.joint_travel(q0, q1)
    assert abs(math.degrees(t[1]) + 340.0) < 1e-9


@have_model
def test_correct_then_predict_is_the_identity(model):
    for arm in model["joint_terminal"]:
        q0 = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        q1 = [1.1, -0.9, 0.8, -1.2, 0.4, -0.1, 1.5]
        cmd, _, _ = G.correct_joints(arm, q0, q1, model=model)
        got = G.predict_joints(arm, q0, cmd, model=model)
        assert max(abs(a - b) for a, b in zip(got, q1)) < 1e-12


# ------------------------------------------------------------- the bridge
def test_the_bridge_deadband_is_narrower_than_the_measured_park():
    """1.0 deg suppressed the correction over a band 3.3x wider than the
    error it left behind. The whole point of narrowing it is that the
    proportional term stays alive where the arm was actually parking."""
    src = open(BRIDGE).read()
    import re
    m = re.search(r'declare_parameter\("deadband_deg",\s*([0-9.]+)\)', src)
    assert m, "the bridge no longer declares deadband_deg"
    band = float(m.group(1))
    park = 0.305
    assert band < park, (
        "the deadband is %.2f deg and joints were measured parking at "
        "%.3f deg. A deadband wider than the residual it causes means the "
        "proportional term is off exactly where it is needed." % (band, park))


def test_the_bridge_applies_the_overshoot_rather_than_only_declaring_it():
    """docs/ENGINEERING_LOG.md's own 'feature present but does nothing'. A parameter that
    is stored and never read is the defect this repository lists first."""
    src = open(BRIDGE).read()
    assert 'declare_parameter("terminal_overshoot", False)' in src
    assert "def _overshoot(self, target):" in src
    assert "target = self._overshoot(target)" in src, (
        "the bridge declares the overshoot and never applies it to the "
        "setpoint")
    # and it must be applied BEFORE the control law reads the target
    i_apply = src.index("target = self._overshoot(target)")
    i_use = src.index("delta = pose_delta_rad(target, actual")
    assert i_apply < i_use, (
        "the overshoot is applied after the control law has already used the "
        "raw target, so it does nothing")


def test_the_overshoot_defaults_off_and_refuses_to_run_at_zero():
    """Three states, and the dangerous one is 'on, applying nothing'. The
    log would say compensated and the arm would not be."""
    src = open(BRIDGE).read()
    assert 'declare_parameter("terminal_overshoot", False)' in src, \
        "the overshoot must default OFF: two corrections for one error is " \
        "one too many, and the deadband change may already have closed it"
    assert "Refusing to run" in src and "overshoot of zero" in src, (
        "the bridge no longer refuses 'on with zero overshoot'. That state "
        "logs as compensated and moves the arm as if it were not.")
    assert "eps_rad" in src, (
        "the bridge does not read the MEASURED eps, so the number in the "
        "control loop is a second copy that can drift from the measurement")


# ----------------------------------------------- the trap that caught me
@pytest.mark.skipif(not os.path.exists(SRC), reason="no calibration source")
def test_the_speed_sweep_is_still_known_to_be_inert():
    """`vmax_rad_s` was read once at bridge start-up, so the 36 runs labelled
    with three speeds all ran at one. Runs exceeding their own commanded
    limit is the proof.

    This test PASSES on the broken data on purpose. If it ever fails, speed
    was genuinely varied in a newer recording -- which is good news, and
    means every 'how the error varies with speed is unmeasured' sentence in
    this repository has to be rewritten. That is why it is a test and not a
    comment.
    """
    with open(SRC) as f:
        d = json.load(f)
    over = [r for r in d["runs"]
            if r["peak_joint_speed_rad_s"] > r["vmax_rad_s"] * 1.02]
    assert over, (
        "no run exceeds its own commanded vmax any more. Either the speed "
        "limit is now applied -- in which case a real speed sweep is "
        "possible and the 'unmeasured' claims are stale -- or this file was "
        "replaced. Go and look before deleting this test.")


@have_model
def test_the_model_file_does_not_claim_speed_was_varied(model):
    s = model.get("speed_sweep_was_inert")
    assert s is not None, (
        "the model file no longer records that the speed sweep was inert. "
        "That fact is the difference between 'the error does not depend on "
        "speed' and 'nobody has measured whether it does'.")
    assert s["runs_exceeding_their_own_vmax"] > 0
    assert "UNMEASURED" in s["verdict"]
