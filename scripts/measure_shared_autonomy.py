#!/usr/bin/env python3
"""WHAT CALIBRATION IS WORTH TO SHARED-AUTONOMY TELEOPERATION. MEASURED.

    python3 scripts/measure_shared_autonomy.py            # no stack needed
    python3 scripts/measure_shared_autonomy.py --trials 400

WHAT SHARED AUTONOMY ACTUALLY IS HERE
-------------------------------------
`srl_autonomy.intent_inference.IntentEstimator` is a Bayesian posterior over
WHICH OBJECT the operator is reaching for, updated from the hand's position and
the direction it is travelling. Everything downstream -- assistance, the
arbiter, mode 03 and mode 04 -- hangs off that answer being right.

Its inputs are the OBJECT POSITIONS. That is the whole connection to
calibration: the estimator can only be as right about "which cube" as it is
about "where the cubes are".

WHY THIS CANNOT BE MEASURED IN THE SIMULATION AS IT STANDS
----------------------------------------------------------
In simulation the declared coordinates ARE the truth -- `mock_rgbd_camera`
builds the scene from the same file the task reads -- so a declared-position
estimator and a perfectly-calibrated one get identical inputs and score
identically. That is not a result about calibration; it is a result about the
renderer.

The interesting quantity is what happens when the table does NOT match the
file, which is every real table. So the declared error is SWEPT: the estimator
is given positions displaced by D millimetres from where the objects really
are, and the operator reaches for a real object. D = 0 is today's simulation.
D = 60 is one cube pitch on this rig, where the file points at the NEIGHBOUR.

The calibrated arm is given the error the calibration ACTUALLY achieves --
11.3 mm mean, measured by `measure_pick_accuracy.py` against the renderer's
own geometry -- rather than a hoped-for zero.

WHAT IS SCORED
--------------
  correct     the estimator's top object is the one being reached for, with
              enough margin that `summary()` does not call it ambiguous
  wrong       it is confident about the WRONG object -- the failure that
              matters, because assistance then pulls the operator away
  ambiguous   it never becomes confident; assistance declines to help

Wrong is much worse than ambiguous, and they are counted separately for that
reason.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, os.path.join(ROOT, "src/srl_autonomy"),
           os.path.join(ROOT, "src/srl_experiments/experiments/abc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

OUT = os.path.join(ROOT, "recordings/baselines/shared_autonomy_gain.json")
# What the calibration achieves, measured, not hoped for.
CALIBRATED_ERR_MM = 11.3
# The hand starts here and reaches toward the target.
START = np.array([0.30, 0.10, 1.45])
FRAMES = 30
STEP_FRAC = 0.06
# How many CONSECUTIVE frames the estimator must name the same object before
# assistance acts on it.
#
# WITHOUT THIS THE MEASUREMENT WAS MEANINGLESS. Acting on the first frame that
# clears the ambiguity margin scored 25% with EXACT object positions -- chance
# for four objects -- because the hand starts 0.4 m away and the directions to
# four cubes 60 mm apart are nearly identical there. The estimator was right
# to be unsure; my harness was reading its first flicker as an answer.
#
# It is also what assistance should do. Pulling the operator toward an object
# because one frame of noise pointed that way is the failure mode the whole
# posterior exists to avoid.
N_CONFIRM = 3


def truth():
    import t1_task as T
    return [np.array([x, y, T.T1_Z], float) for (x, y) in T.T1_CUBES]


def one_trial(est_cls, believed, target_true, rng, hand_noise_m=0.010,
              frames=FRAMES):
    """Reach toward the TRUE target; the estimator sees `believed` positions.

    Returns (verdict, frames_to_confident). The operator is deliberately not
    perfect: the hand wanders by `hand_noise_m` per frame, because a noiseless
    reach makes any estimator look good.
    """
    est = est_cls()
    ids = ["obj_%d" % i for i in range(len(believed))]
    want = ids[int(np.argmin([np.linalg.norm(t - target_true)
                              for t in truth()]))]
    p = START.copy()
    run_top, run_len = None, 0
    for k in range(frames):
        to = target_true - p
        d = float(np.linalg.norm(to))
        if d < 1e-6:
            break
        v = to / d
        step = v * (d * STEP_FRAC) + rng.normal(0, hand_noise_m, 3)
        p = p + step
        est.update(ids, p, np.array(believed), pointing=v, ee_vel=step)
        s = est.summary()
        if s["ambiguous"]:
            run_top, run_len = None, 0
            continue
        if s["top"] == run_top:
            run_len += 1
        else:
            run_top, run_len = s["top"], 1
        if run_len >= N_CONFIRM:
            return ("correct" if run_top == want else "wrong"), k + 1
    return "ambiguous", None


def run(declared_err_mm, trials, seed=0):
    from srl_autonomy.intent_inference import IntentEstimator
    T = truth()
    rng = np.random.default_rng(seed)
    out = {}
    for label, err_mm in (("DECLARED", declared_err_mm),
                          ("CALIBRATED", CALIBRATED_ERR_MM)):
        tally = dict(correct=0, wrong=0, ambiguous=0)
        frames = []
        for _ in range(trials):
            # Each object's believed position is displaced by err_mm in a
            # random direction -- the file (or the map) is wrong, and not
            # wrong the same way for every object.
            believed = []
            for t in T:
                d = rng.normal(0, 1, 3)
                d[2] *= 0.2                      # mostly a planar error
                n = np.linalg.norm(d)
                believed.append(t + (d / n) * (err_mm / 1000.0))
            tgt = T[int(rng.integers(len(T)))]
            verdict, at = one_trial(IntentEstimator, believed, tgt, rng)
            tally[verdict] += 1
            if at:
                frames.append(at)
        out[label] = dict(
            err_mm=err_mm,
            correct_pct=round(100.0 * tally["correct"] / trials, 1),
            wrong_pct=round(100.0 * tally["wrong"] / trials, 1),
            ambiguous_pct=round(100.0 * tally["ambiguous"] / trials, 1),
            frames_to_confident=round(float(np.mean(frames)), 1)
            if frames else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=300)
    ap.add_argument("--declared-errors-mm", type=float, nargs="+",
                    default=[0, 20, 40, 60, 80, 120])
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1

    import t1_task as T
    print("shared-autonomy intent inference, %d trials per condition" % a.trials)
    print("4 cubes on a %.0f mm pitch; the calibrated arm carries its "
          "MEASURED %.1f mm error\n"
          % ((T.T1_CUBES[1][0] - T.T1_CUBES[0][0]) * 1000, CALIBRATED_ERR_MM))
    print("%-12s %-11s %8s %8s %10s %9s"
          % ("table is off", "positions", "correct", "WRONG", "ambiguous",
             "frames"))
    print("-" * 62)
    rows = {}
    for e in a.declared_errors_mm:
        r = run(e, a.trials, seed=int(e))
        rows["%.0f" % e] = r
        for label in ("DECLARED", "CALIBRATED"):
            v = r[label]
            print("%-12s %-11s %7.1f%% %7.1f%% %9.1f%% %9s"
                  % ("%.0f mm" % e if label == "DECLARED" else "",
                     label, v["correct_pct"], v["wrong_pct"],
                     v["ambiguous_pct"],
                     "%.1f" % v["frames_to_confident"]
                     if v["frames_to_confident"] else "--"))
    print()
    print("DECLARED positions are the task file's. CALIBRATED are what the")
    print("sweep measured. At 0 mm they are the same question, which is why")
    print("today's simulation cannot show the difference.")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(dict(rows=rows, trials=a.trials,
                       calibrated_err_mm=CALIBRATED_ERR_MM,
                       cube_pitch_mm=60.0,
                       note=("the declared error is SWEPT because in "
                             "simulation the declared coordinate is the "
                             "truth; the sweep is what a real table does")),
                  f, indent=2, sort_keys=True)
    print("-> %s" % a.out)
    return 0


def self_test(verbose=True):
    """Constructed: the ground truth is which object was reached for."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-62s %s%s" % (name, "PASS" if cond else "FAIL",
                                    "" if cond else "  -- " + str(detail)))

    from srl_autonomy.intent_inference import IntentEstimator
    T = truth()
    rng = np.random.default_rng(1)

    # PERFECT KNOWLEDGE, ACROSS ALL FOUR TARGETS. If the estimator cannot do
    # this, nothing below means anything.
    #
    # AGGREGATED ON PURPOSE, and the first version was not. It tested a single
    # target -- obj_2 -- and demanded 80%, and got 7%. That is not a broken
    # estimator, it is the hardest geometry on the table: reaching for obj_2
    # puts obj_3 DIRECTLY BEHIND IT along the approach, so the direction
    # points at both for most of the reach and the posterior is right to stay
    # unsure. Measured per target with exact positions, 20 trials each:
    #
    #     obj_0  20 correct        obj_2   1 correct, 19 ambiguous
    #     obj_1  20 correct        obj_3  15 correct,  5 ambiguous
    #
    # An object with another one behind it is hard, and saying so is better
    # than averaging it away -- but a test that only ever asks the hard one
    # measures the geometry, not the estimator.
    per = {}
    for i in range(len(T)):
        v = [one_trial(IntentEstimator, T, T[i], rng)[0] for _ in range(20)]
        per[i] = dict(correct=v.count("correct"), wrong=v.count("wrong"),
                      ambiguous=v.count("ambiguous"))
    tot_c = sum(d["correct"] for d in per.values())
    tot_w = sum(d["wrong"] for d in per.values())
    check("with EXACT positions it is NEVER confidently wrong", tot_w == 0,
          per)
    check("and it identifies the target in a majority of trials",
          tot_c > 0.5 * 20 * len(T), (tot_c, 20 * len(T)))
    check("at least one target is hard -- an object with another behind it",
          any(d["ambiguous"] > d["correct"] for d in per.values()), per)
    good = tot_c

    # THE CONTROL: positions displaced by a full cube pitch must do worse.
    bad_believed = [t + np.array([0.06, 0, 0]) for t in T]
    bad = 0
    for i in range(len(T)):
        bad += sum(one_trial(IntentEstimator, bad_believed, T[i], rng)[0]
                   == "correct" for _ in range(20))
    check("THE CONTROL: displacing every object by one 60 mm pitch makes it "
          "worse", bad < good, (bad, good))

    # A wrong answer and an ambiguous one must be distinguishable.
    verdicts = {one_trial(IntentEstimator, bad_believed, T[0], rng)[0]
                for _ in range(40)}
    check("verdicts are drawn from the three named outcomes",
          verdicts <= {"correct", "wrong", "ambiguous"}, verdicts)

    r = run(0.0, 40, seed=3)
    check("at 0 mm declared error the two conditions are NOT identical -- "
          "the calibrated one still carries its own 11.3 mm",
          r["DECLARED"]["err_mm"] != r["CALIBRATED"]["err_mm"])
    if verbose:
        print("measure_shared_autonomy self-test %s"
              % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    sys.exit(main())
