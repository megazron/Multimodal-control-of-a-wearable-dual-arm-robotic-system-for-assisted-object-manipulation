#!/usr/bin/env python3
"""The measured sim-to-real correction: one number, applied in JOINT space.

    python3 -m srl_teleop.sim_to_real_gap        # the self-test

WHAT IS WRONG. The real arms stop short. Measured on the rig 2026-08-21, 36
runs, both arms, six directions, 100 mm each: the end effector lands **7.2 mm
RMS** from where it was told to go. Against a 30 mm grasp gate that is a
quarter of the budget, and nothing in this repository was subtracting it.

(The recording is labelled with three speeds and is not a speed sweep:
`vmax_rad_s` was read once at bridge start-up, and 7 of the 36 runs moved
faster than the limit they were commanded with. How this error varies with
speed is UNMEASURED. It does not affect the model below, whose hold-out is
by axis.)

WHAT SHAPE IT IS, WHICH IS THE WHOLE POINT
------------------------------------------
Every run is a 100 mm step, so "the arm travels 93.5% of what it is told" and
"the arm stops 7.2 mm short" fit the Cartesian data identically. They are not
the same model. On a 20 mm move the first predicts 1.3 mm of error and the
second predicts 7 mm -- a third of the move. A gain fitted here and applied
to short moves would make them worse, confidently.

The JOINT data settles it. Pushing each run's recorded per-joint terminal
error through the arm's own Jacobian reproduces the observed Cartesian error
to 90%, r = 0.923. A Jacobian does not know how far the arm travelled, so
neither does the error. And the joint errors are bimodal -- 211 of 252 at
+/-0.25..0.30 deg with the sign a coin flip -- which is a joint parking just
inside a band on the side it arrived from, not a random stop.

So the model is ONE NUMBER:

    every joint parks EPS radians short, on the side it came from
    EPS = 0.3052 deg (left), 0.3059 deg (right)

and the compensation is **overshoot each joint by EPS in the direction of
travel**. No Cartesian matrix, no direction basis, no step length. Measured:
7.24 -> 1.83 mm and 7.21 -> 1.68 mm, and -- the part that matters -- 1.88 /
1.72 mm HELD OUT on an axis the fit never saw, where a Cartesian 3x3 fitted
to the same data scores 93.5 mm.

WHY THIS ONE IS ON AND THE 3x3 IS NOT
--------------------------------------
An earlier version of this file carried the 3x3 and refused any move that was
more than 10 deg off an axis, more than 20% from 100 mm, or more than 100 mm
from one pose. Those refusals were correct for that model and they made it
useless: almost no real motion satisfies them. The refusals were a symptom of
fitting nine parameters to six axis-aligned directions.

The joint model has no such domain because it makes no directional claim. It
is applied to every move, and the only thing it refuses is a joint whose
travel is SMALLER than EPS -- where "overshoot in the direction of travel" is
not a correction, it is a guess about which way a joint that has barely moved
is going.

The 3x3 is still in the model file, still measured, and is available through
`cartesian_matrix()` for anyone comparing. It is not used.

WHAT IS MEASURED AND WHAT IS INFERRED
--------------------------------------
MEASURED: the magnitude of EPS, that it is the same on both arms to 0.0007
deg, and that a model built from it generalises to an axis it was not fitted
on. NOT measured: how EPS varies with speed -- the recording that looks like
a speed sweep is not one.

INFERRED, and one hardware run settles it: that the sign is always "the side
the joint came from". The recording does not store the commanded joint
trajectory, so the validation takes each joint's sign from the sign of its own
recorded error. The scrambled-sign control in
`scripts/measure_sim_to_real_gap.py` is what stops that being free -- destroy
the sign structure and the model is worth nothing (7.19 mm, i.e. identity).

THE REAL FIX IS UPSTREAM OF THIS FILE. A terminal offset of 0.3 deg per joint
is a controller that stops before it arrives. `kortex_highlevel_bridge`
suppresses its proportional term inside a 1.0 deg deadband -- three times
wider than the error it leaves behind -- so inside that band only the
integrator acts. Compensating for it here is a workaround that works; closing
it there is the fix, and it is `terminal_deadband_deg` in that node.
"""
from __future__ import annotations

import json
import os

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")
MODEL = os.path.join(WS, "recordings/baselines/sim_to_real_gap.json")

# How far outside the measured step length the model may be used. 20% is a
# judgement, it is small, and it is named here rather than buried at a call
# site so that widening it is a visible edit.
STEP_TOLERANCE = 0.20
# How far from the measured start pose. The runs are all from one pose; this
# says "near it", and near is 100 mm.
START_TOLERANCE_M = 0.10
# How far off an axis a direction may be and still be inside the measured
# set. The six directions are axis-aligned, so this is tight on purpose.
AXIS_TOLERANCE_DEG = 10.0


class OutsideDomain(Exception):
    """The correction was asked for where it was never measured."""


class NotCalibrated(Exception):
    """No model file. Run scripts/measure_sim_to_real_gap.py."""


def load(path=MODEL):
    """The fitted model, or `NotCalibrated` naming what to run."""
    if not os.path.exists(path):
        raise NotCalibrated(
            "%s does not exist. Run scripts/measure_sim_to_real_gap.py, "
            "which will refuse to write one if its own controls fail."
            % path)
    with open(path) as f:
        d = json.load(f)
    if not d.get("controls_passed"):
        raise NotCalibrated(
            "%s exists but does not record controls_passed. A correction "
            "whose controls did not run is a systematic error nobody "
            "checked." % path)
    return d


def _axis_angle_deg(v):
    """Degrees from `v` to the nearest coordinate axis."""
    import math
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        return 0.0
    best = 0.0
    for i in range(3):
        c = abs(v[i]) / n
        best = max(best, c)
    return math.degrees(math.acos(max(-1.0, min(1.0, best))))


# ===================================================================== JOINT
# The model that is ON. One number per arm, applied to every move.
CONTINUOUS_IDX = (0, 2, 4, 6)


def eps_rad(arm, model=None):
    """The terminal parking error for one arm, radians. Always positive."""
    d = model or load()
    jt = (d.get("joint_terminal") or {}).get(arm)
    if jt is None:
        raise NotCalibrated(
            "no joint-terminal model for arm %r in %s. Re-run "
            "scripts/measure_sim_to_real_gap.py -- an older model file "
            "carries only the Cartesian tiers, and those are not usable for "
            "an arbitrary move." % (arm, MODEL))
    return float(jt["eps_rad"])


def _wrap(a):
    import math as _m
    return (a + _m.pi) % (2 * _m.pi) - _m.pi


def joint_travel(q_from, q_to, continuous=CONTINUOUS_IDX):
    """q_to - q_from, wrapped on the continuous joints.

    Joints 1/3/5/7 of a Gen3 are continuous. Differencing them naively across
    the +/-pi seam reports 288 deg for a 69.5 deg move, which is defect 5 of
    2026-08-21 and would put the overshoot on the wrong SIDE -- the one thing
    this correction must never do.
    """
    out = []
    for i, (a, b) in enumerate(zip(q_from, q_to)):
        d = float(b) - float(a)
        out.append(_wrap(d) if i in continuous else d)
    return out


def correct_joints(arm, q_from, q_to, model=None, min_travel=None):
    """The joint target to COMMAND so the arm ARRIVES at `q_to`.

    Overshoots each joint by EPS in its own direction of travel. Returns
    (q_command, applied, why) where `applied` is a per-joint bool -- a joint
    that barely moved is left alone and says so, because "the direction it
    was going" is not defined for a joint that was not going anywhere.

    This is the correction that is ON. It applies to any move, any distance,
    any pose: it makes no claim about direction in Cartesian space, which is
    exactly why it survives the axis hold-out that kills the 3x3.
    """
    e = eps_rad(arm, model)
    floor = e if min_travel is None else float(min_travel)
    trav = joint_travel(q_from, q_to)
    out, applied = [], []
    for i, (t, target) in enumerate(zip(trav, q_to)):
        if abs(t) <= floor:
            out.append(float(target))
            applied.append(False)
        else:
            out.append(float(target) + (e if t > 0 else -e))
            applied.append(True)
    n = sum(applied)
    why = ("overshot %d of 7 joints by %.4f deg in the direction of travel; "
           "%d moved less than that and were left alone"
           % (n, __import__("math").degrees(e), 7 - n))
    return out, applied, why


def predict_joints(arm, q_from, q_commanded, model=None, min_travel=None):
    """Where the arm will actually STOP if commanded `q_commanded`.

    The forward direction of `correct_joints`, and the thing a divergence
    display should compare the real arm against -- otherwise a perfectly
    healthy arm reads as 0.3 deg off on every joint forever.
    """
    e = eps_rad(arm, model)
    floor = e if min_travel is None else float(min_travel)
    trav = joint_travel(q_from, q_commanded)
    out = []
    for t, target in zip(trav, q_commanded):
        if abs(t) <= floor:
            out.append(float(target))
        else:
            out.append(float(target) - (e if t > 0 else -e))
    return out


def cartesian_matrix(arm, model=None):
    """The 3x3 Cartesian fit. MEASURED, and deliberately NOT used.

    Kept because it is what the error looks like from outside and because its
    93 mm leave-one-AXIS-out is the evidence that a Cartesian fit to six
    axis-aligned directions must not be trusted off them. If you are reaching
    for this, read the module docstring first.
    """
    d = model or load()
    return d["arms"][arm]["M"]


# ---------------------------------------------------------------- self-test
def self_test(verbose=True):
    """Against the RECORDED runs, and against the model's own physics.

    Inverting a matrix and multiplying it back is a check on numpy. These
    check the two things that can actually be wrong: whether the overshoot
    goes the RIGHT WAY, and whether it would have made the real arm land
    closer on data the fit did not see.
    """
    import math
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-54s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    try:
        d = load()
    except NotCalibrated as e:
        if verbose:
            print("  no model: %s" % e)
        return False

    arms = sorted(d.get("joint_terminal") or {})
    if not arms:
        check("the model file carries a joint-terminal model", False,
              "re-run scripts/measure_sim_to_real_gap.py")
        return False

    # 1 -- THE OVERSHOOT GOES THE RIGHT WAY. A sign error here doubles the
    #      error instead of removing it, and on hardware it is invisible.
    for arm in arms:
        e = eps_rad(arm, d)
        q0 = [0.0] * 7
        q1 = [0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5]
        cmd, applied, _ = correct_joints(arm, q0, q1, model=d)
        good = all(
            (cmd[i] > q1[i] if q1[i] > 0 else cmd[i] < q1[i])
            for i in range(7))
        check("%-5s overshoot is in the direction of travel" % arm, good,
              "eps %.4f deg" % math.degrees(e))
        check("%-5s overshoot is exactly EPS" % arm,
              all(abs(abs(cmd[i] - q1[i]) - e) < 1e-12 for i in range(7)))
        check("%-5s every joint of a large move is corrected" % arm,
              all(applied))

    # 2 -- ROUND TRIP. correct then predict must return the target.
    for arm in arms:
        q0 = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6, -0.7]
        q1 = [0.9, -0.8, 0.7, -0.6, 0.5, -0.4, 0.3]
        cmd, _, _ = correct_joints(arm, q0, q1, model=d)
        got = predict_joints(arm, q0, cmd, model=d)
        check("%-5s correct() then predict() lands on the target" % arm,
              max(abs(a - b) for a, b in zip(got, q1)) < 1e-12,
              "worst %.2e rad" % max(abs(a - b) for a, b in zip(got, q1)))

    # 3 -- A JOINT THAT BARELY MOVED IS LEFT ALONE. Overshooting a joint
    #      whose travel is under EPS is not a correction, it is a guess about
    #      which way something that did not move was going.
    for arm in arms:
        e = eps_rad(arm, d)
        q0 = [0.0] * 7
        q1 = [e * 0.5, e * 2.0, 0.0, -e * 0.5, -e * 2.0, 0.0, 1.0]
        cmd, applied, why = correct_joints(arm, q0, q1, model=d)
        check("%-5s a joint moving less than EPS is not touched" % arm,
              applied == [False, True, False, False, True, False, True]
              and cmd[0] == q1[0] and cmd[2] == q1[2],
              why)

    # 4 -- THE CONTINUOUS SEAM. +170 -> -170 deg is +20 deg the short way:
    #      the joint keeps INCREASING, passes 180 and comes out at -170. So
    #      the overshoot must be POSITIVE, and the commanded value ends up
    #      LESS negative than the target -- which reads backwards until you
    #      remember the joint is travelling the other way round.
    #
    #      Naive subtraction says -340 and would put the overshoot on the
    #      wrong side of a seam crossing. That is defect 5 of 2026-08-21 in
    #      a new costume, and here it would be a 0.3 deg error in the worst
    #      possible direction on every move that crosses.
    #
    #      This check FAILED on its first run and the code was right: the
    #      assertion had the sign of the short way backwards. Left in with
    #      both directions asserted so it cannot be got wrong again.
    for arm in arms:
        e = eps_rad(arm, d)
        for a_deg, b_deg, want in ((170.0, -170.0, +20.0),
                                   (-170.0, 170.0, -20.0),
                                   (10.0, 30.0, +20.0),
                                   (30.0, 10.0, -20.0)):
            q0 = [math.radians(a_deg)] + [0.0] * 6
            q1 = [math.radians(b_deg)] + [0.0] * 6
            t = joint_travel(q0, q1)
            check("%-5s %+7.1f -> %+7.1f deg travels %+5.1f the short way"
                  % (arm, a_deg, b_deg, want),
                  abs(math.degrees(t[0]) - want) < 1e-9,
                  "got %+.2f deg" % math.degrees(t[0]))
            cmd, _, _ = correct_joints(arm, q0, q1, model=d)
            over = cmd[0] - q1[0]
            check("%-5s %+7.1f -> %+7.1f overshoots the same way"
                  % (arm, a_deg, b_deg),
                  abs(abs(over) - e) < 1e-12
                  and (over > 0) == (want > 0),
                  "overshoot %+.4f deg" % math.degrees(over))
        # a NON-continuous joint must NOT wrap: joint_2 has real stops
        q0 = [0.0, math.radians(170.0)] + [0.0] * 5
        q1 = [0.0, math.radians(-170.0)] + [0.0] * 5
        t = joint_travel(q0, q1)
        check("%-5s a NON-continuous joint does not wrap" % arm,
              abs(math.degrees(t[1]) + 340.0) < 1e-9,
              "travel %+.1f deg -- joint_2 has hard stops, so the short way "
              "round does not exist for it" % math.degrees(t[1]))

    # 5 -- HELD OUT, ON THE REAL RUNS. Would this have landed the arm closer
    #      on an AXIS the parameter was not fitted on? This is the claim.
    src = os.path.join(WS, d["source"])
    if os.path.exists(src):
        for arm in arms:
            jt = d["joint_terminal"][arm]
            check("%-5s beats identity, HELD OUT on an unseen axis" % arm,
                  jt["held_out_axis_rms_mm"] < 0.4 * jt["identity_rms_mm"],
                  "%.2f mm -> %.2f mm (%.0f%% removed); the Cartesian 3x3 "
                  "scores %.0f mm on the same hold-out"
                  % (jt["identity_rms_mm"], jt["held_out_axis_rms_mm"],
                     jt["removed_pct"],
                     d["arms"][arm]["held_out_axis_rms_mm"]))
            check("%-5s scrambling the signs destroys it" % arm,
                  jt["scrambled_signs_rms_mm"] > 0.9 * jt["identity_rms_mm"],
                  "%.2f mm, i.e. back to doing nothing"
                  % jt["scrambled_signs_rms_mm"])

    # 6 -- THE TWO ARMS AGREE, which is why one number is credible
    if len(arms) == 2:
        a, b = (eps_rad(x, d) for x in arms)
        check("both arms measure the same EPS",
              abs(a - b) < math.radians(0.01),
              "%.4f vs %.4f deg" % (math.degrees(a), math.degrees(b)))

    # 7 -- an unknown arm is refused rather than defaulted to zero
    try:
        eps_rad("middle", d)
        check("an unknown arm is refused", False)
    except NotCalibrated:
        check("an unknown arm is refused", True)

    # 8 -- the model file must still carry what is NOT validated
    check("the model file states what is NOT validated",
          "NOT_validated" in d.get("what_is_and_is_not_validated", {}))

    if verbose:
        print("sim_to_real_gap self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
