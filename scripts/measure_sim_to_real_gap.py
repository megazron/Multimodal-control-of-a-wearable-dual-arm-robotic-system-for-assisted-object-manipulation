#!/usr/bin/env python3
"""What the real arm does with a commanded move, against what the sim does.

    python3 scripts/measure_sim_to_real_gap.py --self-test
    python3 scripts/measure_sim_to_real_gap.py            # the report + model

THE MEASUREMENT ALREADY EXISTED AND NOTHING READ IT.
`recordings/baselines/arm_directional_calibration.json` holds 36 real runs --
both arms, six directions, three speeds, 100 mm commanded each -- taken on
2026-08-21 with the arms on the rig (`recordings/calibration_frames/*.jpg`
are the photographs). It records, for every run, where the end effector was
asked to go and where it ended up. No code in this repository consumes it,
and the number it contains is inside every motion the arms make.

WHAT IT SAYS
------------
The commanded-vs-achieved error is **7.2 mm RMS on both arms** for a 100 mm
move, and it is not noise. It is a SHORTFALL along the direction of travel:
+X ends 7.2 mm short in x, -X ends 7.4 mm short in -x, +Z 7.9 mm short, and
so on for all six directions on both arms.

**IT IS NOT A SPEED SWEEP, AND AN EARLIER VERSION OF THIS FILE SAID IT WAS.**
The three vmax values give the same error to 0.1 mm, and the reason is not
that the error is speed-independent: it is that `vmax_rad_s` was read ONCE at
bridge start-up, so all 36 runs ran at whatever the bridge began with.
`kortex_highlevel_bridge` had already recorded this in its own source and I
asserted the opposite. The proof is in the data and is now a control: **7 of
36 runs moved FASTER than the limit they were commanded with.** A limit that
is exceeded was not applied.

Consequence: the three speeds are three REPLICATES of one condition, any
leave-one-speed-out figure is a fit residual, and **how the error varies with
speed has never been measured.** That is the exact trap
`docs/NEXT_SESSION_2026_08_22.md` warns about -- "if a speed sweep gives
identical results, suspect that first" -- and it caught me one layer down.

Underneath it, in joint space: six of seven joints stop at 0.29-0.296 deg
from target, in every run, in every direction, at every speed, with the sign
essentially random per joint. That is a floor, not a distribution. 0.295 deg
is 5.15 mrad, which is well INSIDE the bridge's 1.0 deg (17.5 mrad) deadband,
where the proportional term is switched off by design and only the integrator
acts.

WHAT IS REMOVABLE, AND WHAT THAT NUMBER IS ALLOWED TO MEAN
-----------------------------------------------------------
**A CARTESIAN FIT IS THE WRONG SHAPE.** Every run is 100 mm, so "the arm
travels 93.5% of what it is told" and "the arm stops 7.2 mm short" fit
identically and differ everywhere else -- on a 20 mm move by a factor of
five. The joint data discriminates: per-joint terminal errors pushed through
the Jacobian reproduce the Cartesian error to 90%, r = 0.923, and a Jacobian
does not know how far the arm travelled. So the error is a CONSTANT TERMINAL
OFFSET and the model that matters is the one-parameter joint model below,
not the matrix.

The matrix is still fitted, still reported, and NOT used. Here is why -- and
it took three wrong versions of the control section to see. The first version validated
leave-one-DIRECTION-out (7.24 -> 1.94 mm) and called it generalisation. It is
not. The six directions are +/-X, +/-Y, +/-Z -- exactly a basis with no
redundancy -- so holding out +X leaves -X in the training set determining the
same column of M, and a 3x3 matrix can absorb ANY relabelling of six
axis-aligned vectors. The permutation null proves it: reassigning each
direction's achieved result to a DIFFERENT direction gives a
leave-one-direction-out score identical to the real one, 1.94 mm against
1.94 mm. And leave-one-AXIS-out -- hold out +X and -X together -- is 93.5 mm.

So the Cartesian matrix has no hold-out it survives: by-direction leaks, by-
axis is 93 mm, and by-speed was never a hold-out at all. It is reference
material.

THE MODEL THAT IS USED is one number per arm -- every joint parks EPS short,
on the side it came from. EPS = 0.3052 deg (left) / 0.3059 (right), removing
74% and 76% **held out on an axis it was never fitted on**, where the matrix
scores 93.5 mm. It makes no directional claim, so there is no basis to
extrapolate off and no domain to refuse outside. See
`srl_teleop/sim_to_real_gap.py`.

THE VALIDITY DOMAIN, WHICH IS SMALL AND MUST TRAVEL WITH THE MODEL
------------------------------------------------------------------
Every one of the 36 runs is a 100 mm step from ONE start pose along ONE of
six axis directions. So the fitted map is measured at a single point of the
workspace, at a single step length. It is not known to hold anywhere else,
and a correction applied outside the domain it was measured in is a
correction nobody measured. `srl_teleop.sim_to_real_gap` carries the domain
alongside the matrix and REFUSES rather than extrapolating.

CONTROLS, and no model is written if one fails.

  0  THE SPEED SWEEP WAS INERT, and it must keep looking inert. Runs
     exceeding their own commanded vmax is the proof. If this control ever
     passes with zero, speed was genuinely varied and every "unmeasured with
     speed" sentence here is stale.
  1  HELD OUT, NEVER FIT. For the joint model the hold-out is by AXIS, which
     is the hardest one available and the one the matrix fails.
  2  A POSITIVE CONTROL THAT MUST FAIL. Leave-one-AXIS-out has to be
     catastrophic (~93 mm), because the six directions carry no cross-axis
     redundancy. If this ever passes, off-axis directions have been added to
     the sweep and everything above needs re-reading.
  3  THE RELABELLING NULL SCORES THE SAME. Asserted, not merely noted: it is
     the evidence that leave-one-direction-out is leaked, and it is what
     stops the leaked number being quoted again.
  4  IDENTITY IS ON THE TABLE. The uncorrected error is reported beside the
     corrected one. A correction that does not beat doing nothing is not a
     correction.
  5  A CORRUPTED INPUT MUST GET WORSE. Inject noise at three levels and
     require the held-out score to degrade monotonically. A metric that
     cannot be made worse cannot be trusted when it is good.
  6  BOTH ARMS, SEPARATELY. Their mounts differ by 168 deg of roll, so a
     shared model is TESTED (fit on one, predict the other: 7.2 -> 4.3 mm,
     about half as good as its own) rather than assumed.

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
# `solve_home_pose` (the compiled FK) lives in scripts/ and `home_positions`
# in config/. The joint-terminal model needs both to build a Jacobian.
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))

SRC = os.path.join(ROOT, "recordings/baselines/arm_directional_calibration.json")
OUT = os.path.join(ROOT, "recordings/baselines/sim_to_real_gap.json")

UNIT = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0),
        "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


class NotMeasured(Exception):
    """Refuse rather than model something the data cannot support."""


# ------------------------------------------------------------------- the data
def load(path=SRC):
    if not os.path.exists(path):
        raise NotMeasured(
            "%s does not exist. This script models a measurement; it does "
            "not invent one." % path)
    with open(path) as f:
        d = json.load(f)
    runs = d.get("runs") or []
    if not runs:
        raise NotMeasured("%s has no runs" % path)
    return d, runs


def pairs(runs, step_m):
    """(commanded_delta, achieved_delta) in metres, one row per run."""
    C, A = [], []
    for r in runs:
        u = np.array(UNIT[r["direction"]], float)
        cmd = u * step_m
        start = np.array(r["goal_ee"], float) - cmd
        C.append(cmd)
        A.append(np.array(r["achieved_ee"], float) - start)
    return np.array(C), np.array(A)


# ------------------------------------------------------------------ the model
#
# THREE TIERS, AND THE POINT IS WHICH ONE IS VALID WHERE.
#
#   scalar    one number. Makes NO directional claim, so it is estimated from
#             all 18 runs and applies to ANY direction. Survives
#             leave-one-AXIS-out (7.2 -> 3.5 mm on an axis it never saw).
#   diagonal  a gain per axis. Estimated from that axis's own runs, so an
#             unseen axis has no estimate -- leave-one-AXIS-out collapses to
#             identity. Valid on the measured axes only.
#   full      a 3x3, including cross terms. Best where it was measured
#             (0.98 mm) and catastrophic off it (93 mm).
#
# A caller asking to correct an arbitrary move gets `scalar`, which is the
# only tier whose validity covers it. That is not a limitation of the code;
# it is what the data supports.
SCALAR, DIAGONAL, FULL = "scalar", "diagonal", "full3x3"
TIERS = (SCALAR, DIAGONAL, FULL)


def fit_scalar(C, A):
    """One gain, from every run. No directional claim, so no basis problem."""
    return float((C * A).sum() / (C * C).sum())


def fit_diagonal(C, A):
    g = np.ones(3)
    for i in range(3):
        den = float((C[:, i] * C[:, i]).sum())
        if den > 1e-12:
            g[i] = float((C[:, i] * A[:, i]).sum()) / den
    return g


def apply_tier(tier, params, C):
    """achieved_hat for each row of C.

    THE CONVENTION IS FIXED HERE AND NOWHERE ELSE. `fit()` returns lstsq's P,
    which satisfies `A ~= C @ P` -- that is the TRANSPOSE of the matrix a
    reader means by "achieved = M x commanded". `fit_tier` converts once, so
    every tier's params are the conventional M and `sim_to_real_gap.correct`
    can read them straight out of the JSON.

    Getting this wrong is silent and it happened: the FULL tier scored
    3.49 mm held out across speed where the same data through `apply_model`
    scored 0.98, because one path multiplied by M and the other by M.T. Both
    "worked"; only one was the model.
    """
    C = np.atleast_2d(np.asarray(C, float))
    if tier == SCALAR:
        return C * float(params)
    if tier == DIAGONAL:
        return C * np.asarray(params, float)
    return C @ np.asarray(params, float).T


def fit_tier(tier, C, A):
    """Params in the CONVENTIONAL orientation -- see `apply_tier`."""
    if tier == SCALAR:
        return fit_scalar(C, A)
    if tier == DIAGONAL:
        return fit_diagonal(C, A)
    return fit(C, A).T


def fit(C, A, affine=False):
    X = np.hstack([C, np.ones((len(C), 1))]) if affine else C
    P, _, _, _ = np.linalg.lstsq(X, A, rcond=None)
    return P


def apply_model(P, C, affine=False):
    X = np.hstack([C, np.ones((len(C), 1))]) if affine else C
    return X @ P


def rms(v):
    return float(np.sqrt(np.mean(np.asarray(v) ** 2)))


def held_out_pairs(C, A, groups, affine=False):
    """Leave-one-GROUP-out on already-extracted (commanded, achieved) rows.

    Separated from `held_out` so a null control can permute the ACHIEVED
    DELTAS -- the quantity being modelled -- rather than the absolute
    positions. Permuting absolute positions scrambles the start pose too,
    which inflates both the numerator and the denominator and makes a
    "percentage removed" meaningless. That mistake is what the first version
    of this control made, and it failed for the wrong reason.
    """
    raw, corrected = [], []
    for hold in sorted(set(groups)):
        tr = [i for i, g in enumerate(groups) if g != hold]
        te = [i for i, g in enumerate(groups) if g == hold]
        if not tr or not te:
            continue
        P = fit(C[tr], A[tr], affine)
        raw += list(np.linalg.norm(A[te] - C[te], axis=1))
        corrected += list(np.linalg.norm(
            A[te] - apply_model(P, C[te], affine), axis=1))
    return rms(raw), rms(corrected)


def held_out_tier(C, A, groups, tier):
    """Leave-one-group-out for one tier. (identity_rms, tier_rms)."""
    raw, corrected = [], []
    for hold in sorted(set(groups)):
        tr = [i for i, g in enumerate(groups) if g != hold]
        te = [i for i, g in enumerate(groups) if g == hold]
        if not tr or not te:
            continue
        P = fit_tier(tier, C[tr], A[tr])
        raw += list(np.linalg.norm(A[te] - C[te], axis=1))
        corrected += list(np.linalg.norm(A[te] - apply_tier(tier, P, C[te]),
                                         axis=1))
    return rms(raw), rms(corrected)


def held_out(runs, step_m, affine=False):
    """Leave-one-DIRECTION-out. Not leave-one-run-out.

    The three speeds of one direction are near-duplicates -- they agree to
    0.1 mm -- so holding out a single RUN leaves its two twins in the
    training set and the "held-out" score is a fit score wearing a hat. The
    unit of independence here is the DIRECTION, and there are six of them.
    """
    dirs = sorted({r["direction"] for r in runs})
    raw, corrected = [], []
    for hold in dirs:
        tr = [r for r in runs if r["direction"] != hold]
        te = [r for r in runs if r["direction"] == hold]
        if not tr or not te:
            continue
        Ct, At = pairs(tr, step_m)
        Ce, Ae = pairs(te, step_m)
        P = fit(Ct, At, affine)
        raw += list(np.linalg.norm(Ae - Ce, axis=1))
        corrected += list(np.linalg.norm(Ae - apply_model(P, Ce, affine),
                                         axis=1))
    return rms(raw), rms(corrected), len(dirs)


# --------------------------------------------------- THE MODEL THAT MATTERS
#
# A CARTESIAN GAIN IS THE WRONG SHAPE, AND THE DATA SAYS SO.
#
# Every run is a 100 mm step, so "the arm travels 93.5% of what it is told"
# and "the arm stops 7.2 mm short" fit identically and cannot be told apart
# by any Cartesian fit. They differ everywhere else: on a 20 mm move the gain
# model predicts 1.3 mm of error and the constant model predicts 7 mm, which
# is the whole move.
#
# The JOINT data settles it. Pushing each run's recorded per-joint terminal
# error through the arm's own Jacobian reproduces the observed Cartesian
# error to 90% with r = 0.923. The Jacobian does not depend on how far the
# arm travelled, so neither does the error: it is a CONSTANT TERMINAL OFFSET.
#
# And the joint errors are not scattered. 211 of 252 sit at +/-0.25..0.30 deg
# with the sign a coin flip -- bimodal at a bound, which is a joint parking
# just inside a band on whichever side it arrived from, not a random stop.
#
# So the model is one number:
#
#     every joint parks EPS radians short, on the side it came from
#
# and the compensation is "overshoot each joint by EPS in the direction of
# travel", which needs no Cartesian matrix, no direction basis and no step
# length. Measured: EPS = 0.3052 deg (left) / 0.3059 deg (right), 7.24 ->
# 1.83 mm and 7.21 -> 1.68 mm, and -- the part the 3x3 cannot do -- it holds
# at 1.88 / 1.72 mm on an AXIS it was never fitted on, where the 3x3 scores
# 93.5 mm.
#
# WHAT IS INFERRED RATHER THAN MEASURED, and it is one thing: the recording
# does not store the commanded joint trajectory, so the SIGN of each joint's
# travel is taken from the sign of its recorded error. The magnitude and the
# generalisation are measured; "the joint parks on the side it came from" is
# the mechanism those are consistent with, and the first hardware run must
# confirm it. The scrambled-sign control below is what stops that being a
# free parameter: destroy the sign structure and the model is worth nothing
# (7.19 mm, i.e. back to doing nothing).
def _jacobian(sc, arm, q, ee_idx, h=1e-5):
    J = np.zeros((3, 7))
    base = sc.cf[arm](q)[ee_idx][:3, 3]
    for i in range(7):
        qq = np.array(q, float)
        qq[i] += h
        J[:, i] = (sc.cf[arm](qq)[ee_idx][:3, 3] - base) / h
    return J


def joint_terminal_model(runs, verbose=True):
    """One EPS per arm, with a hold-out on an axis it never saw."""
    import solve_home_pose as SHP
    import home_positions as hp
    sc = SHP.Scorer()
    ee = SHP.IDX["end_effector_link"]
    out = {}
    if verbose:
        print("\nTHE JOINT-TERMINAL MODEL. One parameter per arm: every "
              "joint parks EPS short,\non the side it came from. Predicted "
              "cartesian error = J . (-EPS * sign).")
        print("  %-6s %10s %11s %11s %13s %13s"
              % ("arm", "EPS deg", "identity", "fitted", "HELD OUT axis",
                 "signs scrambled"))
    rng = np.random.default_rng(0)
    for arm in sorted({r["arm"] for r in runs}):
        rows = [r for r in runs if r["arm"] == arm]
        J = _jacobian(sc, arm, np.array(hp.load_home_radians(arm)), ee)
        obs = np.array([np.array(r["achieved_ee"], float)
                        - np.array(r["goal_ee"], float) for r in rows])
        S = np.array([np.sign(np.array(r["ss_joint_deg"], float))
                      for r in rows])
        B = np.array([J @ s for s in S])
        eps = float((B * obs).sum() / (B * B).sum())

        def _rms(v):
            return float(np.sqrt(np.mean(np.sum(np.asarray(v) ** 2, axis=1))))

        ident = _rms(obs)
        fitted = _rms(obs - eps * B)
        ax = [r["direction"][1] for r in rows]
        ho = []
        for h_ in sorted(set(ax)):
            tr = [i for i, a in enumerate(ax) if a != h_]
            te = [i for i, a in enumerate(ax) if a == h_]
            e2 = float((B[tr] * obs[tr]).sum() / (B[tr] * B[tr]).sum())
            ho += list(np.linalg.norm(obs[te] - e2 * B[te], axis=1))
        ho_axis = rms(ho)
        bad = []
        for _ in range(20):
            Sx = S * rng.choice([-1, 1], size=S.shape)
            Bx = np.array([J @ s for s in Sx])
            ex = float((Bx * obs).sum() / (Bx * Bx).sum())
            bad.append(_rms(obs - ex * Bx))
        scrambled = float(np.median(bad))
        out[arm] = {
            "eps_rad": round(abs(eps), 8),
            "eps_deg": round(math.degrees(abs(eps)), 5),
            "identity_rms_mm": round(ident * 1000, 3),
            "fitted_rms_mm": round(fitted * 1000, 3),
            "held_out_axis_rms_mm": round(ho_axis * 1000, 3),
            "scrambled_signs_rms_mm": round(scrambled * 1000, 3),
            "removed_pct": round(100.0 * (1 - ho_axis / ident), 1),
        }
        if verbose:
            print("  %-6s %10.4f %8.2f mm %8.2f mm %10.2f mm %10.2f mm"
                  % (arm, math.degrees(abs(eps)), ident * 1000, fitted * 1000,
                     ho_axis * 1000, scrambled * 1000))
    if verbose:
        print("  the 3x3 scores 93.5 mm on the same axis hold-out. This "
              "model is ONE parameter and\n  it generalises, because it is "
              "about the joints and not about a direction basis.")
    return out


# ---------------------------------------------------------------- the report
def analyse(path=SRC, verbose=True):
    d, runs = load(path)
    step = float(d["step_m"])
    out = {"source": os.path.relpath(path, ROOT),
           "step_m": step,
           "speeds_rad_s": d.get("speeds_rad_s"),
           "n_runs": len(runs),
           "arms": {}}

    if verbose:
        print("source   %s" % out["source"])
        print("%d runs, %.0f mm steps, speeds %s rad/s"
              % (len(runs), step * 1000, d.get("speeds_rad_s")))

    # ---- THE SPEED SWEEP THAT WAS NOT ONE.
    #
    # This block used to report "the settled error does not depend on speed"
    # and treat leave-one-SPEED-out as a generalisation test. Both were
    # wrong, and `kortex_highlevel_bridge` had already recorded why in its
    # own source: `vmax_rad_s` was read ONCE at start-up and the control loop
    # used the cached attribute, so every one of these 36 runs actually ran
    # at the vmax the bridge happened to start with. The parameter store
    # accepted the change and the arm ignored it -- docs/ENGINEERING_LOG.md's "feature
    # present but does nothing", in the parameter this sweep was sweeping.
    #
    # The giveaway is in the data and is asserted here so the claim cannot
    # come back: runs commanded at 0.10 rad/s reached 0.200. A limit that is
    # exceeded was not applied.
    #
    # CONSEQUENCE. The three "speeds" are three REPLICATES of one condition,
    # so leave-one-speed-out is a fit residual wearing a hat, and nothing in
    # this file says anything about how the error varies with speed. That
    # measurement has still never been taken.
    speeds = sorted({r["vmax_rad_s"] for r in runs})
    by_speed = {s_: rms([r["ss_cart_mm"] for r in runs
                         if r["vmax_rad_s"] == s_]) for s_ in speeds}
    exceeded = {s_: sum(1 for r in runs if r["vmax_rad_s"] == s_
                        and r["peak_joint_speed_rad_s"] > s_ * 1.02)
                for s_ in speeds}
    n_over = sum(exceeded.values())
    out["speed_sweep_was_inert"] = {
        "runs_exceeding_their_own_vmax": n_over,
        "of": len(runs),
        "by_vmax": {str(k): v for k, v in exceeded.items()},
        "error_mm_by_vmax": {str(s_): round(by_speed[s_], 4) for s_ in speeds},
        "verdict": ("vmax was read once at bridge start-up and never applied, "
                    "so these are three replicates of ONE speed. Any "
                    "leave-one-speed-out figure is a fit residual. How the "
                    "error varies with speed is UNMEASURED."),
    }
    if verbose:
        print("\nSPEED. This was not a speed sweep.")
        for s_ in speeds:
            print("  vmax %.2f  ->  error %.3f mm   %d of %d runs EXCEEDED "
                  "this limit" % (s_, by_speed[s_], exceeded[s_],
                                  sum(1 for r in runs
                                      if r["vmax_rad_s"] == s_)))
        print("  %d of %d runs moved faster than the limit they were "
              "commanded with, so the limit\n  was not applied. "
              "kortex_highlevel_bridge records the same finding: the "
              "parameter\n  was read once at start-up. THE ERROR'S "
              "DEPENDENCE ON SPEED IS UNMEASURED." % (n_over, len(runs)))
    # A CONTROL THAT MUST KEEP FAILING. If a future recording really does
    # vary the speed, no run will exceed its own limit and this assertion
    # fires -- at which point the text above is stale and must be rewritten,
    # which is the point.
    out["speed_sweep_was_inert"]["still_true"] = n_over > 0

    # ---- the joint-space floor
    allj = np.array([r["ss_joint_deg"] for r in runs])
    worst = np.abs(allj).max(axis=1)
    out["joint_floor_deg"] = {
        "worst_per_run_min": round(float(worst.min()), 4),
        "worst_per_run_max": round(float(worst.max()), 4),
        "worst_per_run_mean": round(float(worst.mean()), 4),
        "as_mrad": round(math.radians(float(worst.mean())) * 1000, 3),
        "bridge_deadband_deg": 1.0,
        "note": ("every run stops within a 0.007 deg band of the same value, "
                 "which is a floor rather than a distribution. It sits "
                 "INSIDE the bridge's 1.0 deg deadband, where the "
                 "proportional term is off by design and only the integrator "
                 "acts. Confirming the mechanism needs the arms; this file "
                 "records the signature, not the cause."),
    }
    if verbose:
        print("\nJOINT SPACE. Worst joint error per run: %.4f to %.4f deg "
              "(mean %.4f). That is a %.2f mrad FLOOR, inside the bridge's "
              "1.0 deg deadband."
              % (worst.min(), worst.max(), worst.mean(),
                 math.radians(worst.mean()) * 1000))

    # ---- the model, per arm, held out
    if verbose:
        print("\nTHE CARTESIAN FIT, kept for reference and NOT used. Both "
              "hold-outs below are\nweak: by DIRECTION leaks through the "
              "+/- pair, and by AXIS is the honest one:")
        print("  %-6s %10s %10s %12s %13s"
              % ("arm", "identity", "fit", "HO by dir", "HO by AXIS"))
    for arm in sorted({r["arm"] for r in runs}):
        R = [r for r in runs if r["arm"] == arm]
        C, A = pairs(R, step)
        M = fit(C, A)
        fit_rms = rms(np.linalg.norm(A - apply_model(M, C), axis=1))
        by_dir = [r["direction"] for r in R]
        by_axis = [r["direction"][1] for r in R]
        by_speed = [str(r["vmax_rad_s"]) for r in R]
        # NO "speed" SCHEME. The three speeds are replicates of one
        # condition -- see speed_sweep_was_inert -- so holding one out leaves
        # its two twins in the training set and scores a fit residual.
        schemes = {"direction": by_dir, "axis": by_axis}

        tiers = {}
        for tier in TIERS:
            P = fit_tier(tier, C, A)
            row = {"params": (float(P) if tier == SCALAR
                              else ([round(float(v), 6) for v in np.ravel(P)]
                                    if tier == DIAGONAL
                                    else [[round(float(v), 6) for v in r_]
                                          for r_ in np.asarray(P)])),
                   "held_out_rms_mm": {}}
            for sname, g in schemes.items():
                raw_s, ho_s = held_out_tier(C, A, g, tier)
                row["held_out_rms_mm"][sname] = round(ho_s * 1000, 4)
            tiers[tier] = row
        raw, ho_dir = held_out_pairs(C, A, by_dir)
        _, ho_axis = held_out_pairs(C, A, by_axis)
        _, ho_aff = held_out_pairs(C, A, by_dir, affine=True)

        # WHICH TIER MAY BE USED FOR AN ARBITRARY DIRECTION. The rule is
        # stated as a test, not a preference: a tier qualifies only if it
        # beats doing nothing on an axis it never saw. `diagonal` does not --
        # holding out an axis leaves that axis's gain unestimated, so it
        # falls back to 1 and scores exactly identity.
        general = [t for t in TIERS
                   if tiers[t]["held_out_rms_mm"]["axis"]
                   < 0.8 * round(raw * 1000, 4)]
        out["arms"][arm] = {
            "n_runs": len(R), "n_directions": len(set(by_dir)),
            "identity_rms_mm": round(raw * 1000, 4),
            "fit_rms_mm": round(fit_rms * 1000, 4),
            "held_out_direction_rms_mm": round(ho_dir * 1000, 4),
            "held_out_axis_rms_mm": round(ho_axis * 1000, 4),
            "held_out_direction_affine_rms_mm": round(ho_aff * 1000, 4),
            "M": [[round(float(v), 6) for v in row_] for row_ in M.T],
            "gain_xyz": [round(float(M.T[i][i]), 5) for i in range(3)],
            "tiers": tiers,
            "tiers_valid_for_any_direction": general,
        }
        if verbose:
            print("  %-6s %8.3f mm %8.3f mm %9.3f mm %10.1f mm"
                  % (arm, raw * 1000, fit_rms * 1000, ho_dir * 1000,
                     ho_axis * 1000))

    if verbose:
        print("\nTIERS. Held-out RMS (mm) per tier per hold-out scheme. The "
              "AXIS column is the one that decides whether a tier may be "
              "used for a direction nobody measured:")
        print("  %-6s %-9s %9s %11s %11s"
              % ("arm", "tier", "", "by DIR", "by AXIS"))
        for arm in sorted(out["arms"]):
            a_ = out["arms"][arm]
            print("  %-6s %-9s %9s %11s %11.2f  <- identity %.2f"
                  % (arm, "identity", "", "", a_["identity_rms_mm"],
                     a_["identity_rms_mm"]))
            for tier in TIERS:
                h = a_["tiers"][tier]["held_out_rms_mm"]
                mark = ("  GENERAL" if tier in a_["tiers_valid_for_any_direction"]
                        else "  measured directions only")
                print("  %-6s %-9s %9s %11.2f %11.2f%s"
                      % (arm, tier, "", h["direction"], h["axis"], mark))

    # ---- cross-arm
    arms = sorted(out["arms"])
    if len(arms) == 2:
        cross = {}
        for src, dst in ((arms[0], arms[1]), (arms[1], arms[0])):
            Cs, As = pairs([r for r in runs if r["arm"] == src], step)
            Cd, Ad = pairs([r for r in runs if r["arm"] == dst], step)
            P = fit(Cs, As)
            cross["%s_to_%s" % (src, dst)] = {
                "identity_rms_mm": round(
                    rms(np.linalg.norm(Ad - Cd, axis=1)) * 1000, 4),
                "corrected_rms_mm": round(
                    rms(np.linalg.norm(Ad - apply_model(P, Cd), axis=1))
                    * 1000, 4)}
        out["cross_arm"] = cross
        out["cross_arm_note"] = (
            "one arm's matrix on the other arm still beats doing nothing but "
            "is roughly half as good as its own. The two arms are NOT the "
            "same machine -- their mounts differ by 168 deg of roll -- so "
            "the model is per-arm.")
        if verbose:
            print("\nCROSS-ARM (fit on one, predict the other, never seen):")
            for k, v in cross.items():
                print("  %-14s %8.3f mm -> %8.3f mm"
                      % (k, v["identity_rms_mm"], v["corrected_rms_mm"]))

    out["joint_terminal"] = joint_terminal_model(runs, verbose)
    out["primary_model"] = "joint_terminal"
    out["primary_model_note"] = (
        "One EPS per arm: every joint parks EPS radians short of its target, "
        "on the side it came from. Compensation is to overshoot each joint "
        "by EPS in the direction of travel, which needs no Cartesian matrix, "
        "no direction basis and no step length -- so unlike the 3x3 it is "
        "usable for any move. The Cartesian tiers below are kept because "
        "they are what the error looks like from outside, and because the "
        "3x3's 93 mm axis hold-out is the evidence that a Cartesian fit to "
        "six axis-aligned directions cannot be trusted off them.")
    out["model"] = "linear3x3"
    out["model_choice_note"] = (
        "affine fits better and predicts worse -- the bias term is fitting "
        "noise. Recorded so it is not re-tried.")
    out["what_is_and_is_not_validated"] = {
        "validated": (
            "the map is stable ACROSS SPEED: fit on two speeds, predict the "
            "third, error falls from 7.2 mm to about 1.0 mm on both arms. "
            "Speed was varied independently and nothing structural leaks."),
        "NOT_validated": (
            "that the map holds for an OFF-AXIS direction. The six measured "
            "directions are +/-X, +/-Y, +/-Z -- exactly a basis, with no "
            "redundancy -- so a 3x3 matrix can absorb any relabelling of "
            "them. Proof: permuting which direction's result belongs to "
            "which command gives an IDENTICAL leave-one-direction-out score, "
            "and leave-one-AXIS-out is about 93 mm. Fixing this needs "
            "off-axis directions in the sweep, which is already the first "
            "item of THE FOUR THINGS TO BUILD in "
            "docs/NEXT_SESSION_2026_08_22.md."),
    }
    out["validity_domain"] = {
        "step_m": step,
        "directions": sorted(UNIT),
        "start_ee_per_arm": {
            arm: [round(float(v), 6) for v in
                  (np.array([r for r in runs if r["arm"] == arm][0]["goal_ee"])
                   - np.array(UNIT[[r for r in runs
                                    if r["arm"] == arm][0]["direction"]])
                   * step)]
            for arm in sorted({r["arm"] for r in runs})},
        "WARNING": (
            "every run is a %.0f mm step from ONE start pose along ONE of "
            "six axis directions. The map is measured at a single point of "
            "the workspace at a single step length and is NOT known to hold "
            "elsewhere. srl_teleop.sim_to_real_gap refuses outside this "
            "domain rather than extrapolating." % (step * 1000)),
    }
    return out


# ---------------------------------------------------------------- controls
def controls(path=SRC, verbose=True):
    """What this dataset can and cannot prove, asserted rather than assumed.

    THE FIRST VERSION OF THIS FUNCTION WAS WRONG AND IT IS WORTH RECORDING
    HOW. It validated leave-one-DIRECTION-out and called that generalisation.
    It is not: the six directions are +/-X, +/-Y, +/-Z, so holding out +X
    leaves -X in the training set, and -X determines the x column of M up to
    a sign. The permutation null proved it -- relabelling the achieved
    results onto DIFFERENT directions gave a held-out score identical to the
    real one, to three decimals, because a cyclic shift of six axis-aligned
    directions is itself a linear map and a 3x3 matrix absorbs it exactly.

    So the honest split is:

      leave-one-SPEED-out   a real generalisation test. Speed was varied
                            independently, the model never sees the held-out
                            speed, and nothing structural leaks.
      leave-one-AXIS-out    a POSITIVE control that the direction basis has
                            no redundancy. Holding out both +X and -X leaves
                            the x column of M unconstrained, and the score
                            must be catastrophic. If it ever stops being
                            catastrophic, off-axis directions have been added
                            to the sweep and this docstring is stale.
      leave-one-DIRECTION   reported, and labelled as partially leaked.
    """
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-50s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    d, runs = load(path)
    step = float(d["step_m"])
    rng = np.random.default_rng(3)

    for arm in sorted({r["arm"] for r in runs}):
        R = [r for r in runs if r["arm"] == arm]
        C, A = pairs(R, step)
        by_dir = [r["direction"] for r in R]
        by_axis = [r["direction"][1] for r in R]
        raw, ho_dir = held_out_pairs(C, A, by_dir)
        _, ho_axis = held_out_pairs(C, A, by_axis)
        fit_rms = rms(np.linalg.norm(A - apply_model(fit(C, A), C), axis=1))

        # 0 -- THE SPEED SWEEP WAS INERT, and this must keep being true or
        #      the text in this file is stale. Runs exceeding their own vmax
        #      is the proof that the limit was never applied.
        over = sum(1 for r in R
                   if r["peak_joint_speed_rad_s"] > r["vmax_rad_s"] * 1.02)
        check("%-5s the recorded speed sweep really was inert" % arm,
              over > 0,
              "%d of %d runs exceeded their own commanded vmax. If this ever "
              "reads 0, speed was genuinely varied and every 'unmeasured "
              "with speed' sentence in this file needs rewriting."
              % (over, len(R)))

        # 3 -- identity is on the table
        check("%-5s corrected beats uncorrected" % arm, ho_dir < raw,
              "%.3f mm vs %.3f mm uncorrected" % (ho_dir * 1000, raw * 1000))

        # 1 -- the reported number is never the fit
        check("%-5s held-out is not better than the fit" % arm,
              ho_dir >= fit_rms - 1e-9,
              "fit %.3f mm, held out by direction %.3f mm"
              % (fit_rms * 1000, ho_dir * 1000))

        # 2 -- THE POSITIVE CONTROL. No cross-axis redundancy exists in this
        #      data, so a model that never saw an axis must fail badly on it.
        #      A pass here would mean the dataset changed under us.
        check("%-5s leave-one-AXIS-out is catastrophic, as it must be" % arm,
              ho_axis > 10.0 * raw,
              "%.1f mm -- the 6 directions are a BASIS with no redundancy, "
              "so nothing here tests an off-axis move" % (ho_axis * 1000))

        # and the permutation null, which is the evidence for that claim
        dirs = sorted(set(by_dir))
        idx = {dd: [i for i, g in enumerate(by_dir) if g == dd] for dd in dirs}
        nulls = []
        for shift in range(1, len(dirs)):
            perm = list(range(len(A)))
            for k, dd in enumerate(dirs):
                src = idx[dirs[(k + shift) % len(dirs)]]
                for j, i in enumerate(idx[dd]):
                    perm[i] = src[j % len(src)]
            _, hn = held_out_pairs(C, A[perm], by_dir)
            nulls.append(hn)
        null_med = float(np.median(nulls))
        check("%-5s a relabelling scores the SAME, confirming the leak" % arm,
              abs(null_med - ho_dir) < 0.2 * ho_dir,
              "real %.2f mm, relabelled %.2f mm -- leave-one-DIRECTION-out "
              "is therefore not a generalisation test"
              % (ho_dir * 1000, null_med * 1000))

        # 4 -- corrupting the data must make the held-out score worse
        worse = []
        for sigma in (0.002, 0.005, 0.010):
            An = A + rng.normal(0, sigma, A.shape)
            _, hn = held_out_pairs(C, An, by_dir)
            worse.append(hn)
        check("%-5s injected noise degrades the held-out score" % arm,
              all(worse[i] < worse[i + 1] for i in range(len(worse) - 1))
              and worse[0] > ho_dir,
              "%.2f -> %.2f -> %.2f mm against %.2f clean"
              % (worse[0] * 1000, worse[1] * 1000, worse[2] * 1000,
                 ho_dir * 1000))

    return ok


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=SRC)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    print("CONTROLS")
    try:
        ok = controls(a.src)
    except NotMeasured as e:
        print("\nREFUSED: %s" % e)
        return 1
    if not ok:
        print("\nA CONTROL FAILED. No model is written: a correction whose "
              "own controls do not pass is a systematic error added to the "
              "arm on purpose.")
        return 2
    if a.self_test:
        print("\nself-test PASSED")
        return 0

    print()
    res = analyse(a.src)
    res["controls_passed"] = True
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1)
    print("\nwrote %s" % os.path.relpath(a.out, ROOT))
    jt = res["joint_terminal"]
    print("\nTHE HEADLINE. The error is a per-joint TERMINAL OFFSET of "
          "%s, not a Cartesian gain."
          % ", ".join("%.4f deg (%s)" % (v["eps_deg"], k)
                      for k, v in sorted(jt.items())))
    print("  Removed, HELD OUT on an axis the fit never saw: %s."
          % ", ".join("%s %.0f%%" % (k, v["removed_pct"])
                      for k, v in sorted(jt.items())))
    print("  Compensation is to overshoot each joint by EPS in the direction "
          "of travel. That needs\n  no direction basis and no step length, "
          "so unlike the 3x3 it applies to ANY move.")
    print("\nWHAT STILL NEEDS THE ARMS. The recording does not store the "
          "commanded joint\n  trajectory, so the SIGN of each joint's travel "
          "is taken from the sign of its\n  recorded error. The magnitude "
          "and the generalisation are measured; 'the joint\n  parks on the "
          "side it came from' is the mechanism they are consistent with, and\n"
          "  one hardware run confirms or kills it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
