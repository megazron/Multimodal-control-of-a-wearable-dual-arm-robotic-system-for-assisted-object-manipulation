#!/usr/bin/env python3
"""IK that looks before it says no.

WHY NOT grasp_pipeline.reachable(). That function seeds L-BFGS-B from `home`
plus N UNIFORM-RANDOM joint vectors. A uniform random 7-vector is almost
never in the basin of a given Cartesian target, so on a hard point every
start fails and the function reports:

    "no IK solution in 301 starts -- this point is outside the arm's 902 mm
     reach, not blocked by a rule"

which is a CONCLUSION, not a measurement. Measured on 2026-08-21: a target it
refused at 301 restarts was walked to within 42 mm by plain random FK
sampling. CLAUDE.md already records this failure for this function -- "every
'cannot reach' it produced meant 'I did not look'" -- and it was still the
default path.

The cure is seeding. Sample the joint space once, keep the table, and start
the optimiser from samples that are ALREADY NEAR the target. Then a refusal
means the search looked.

WHAT IS REPORTED. The achieved residual, always, in millimetres. "Reachable"
against a 10 mm tolerance and "solved to 0.2 mm" are different facts and a
caller deciding whether to drive a gripper somewhere needs the second one.
"""
from __future__ import annotations

import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "config"))

import numpy as np

SEED_TABLE = 60000
N_SEEDS = 60
CONT_IDX = (0, 2, 4, 6)


class ArmIK:
    """Position IK for one arm, with a cached seed table.

    `point` selects what is being placed: "hand" is the finger-pad midpoint
    (what a grasp cares about), "ee" is end_effector_link (what the reach
    tables were measured with). They differ by about 98 mm along the tool
    axis, which is three times a grasp tolerance, so the choice is explicit
    and never defaulted silently.
    """

    def __init__(self, scorer, arm, point="ee", seed=11,
                 table=SEED_TABLE):
        import solve_home_pose as SHP
        self.SHP = SHP
        self.sc = scorer
        self.arm = arm
        self.point = point
        self.lo, self.hi, _ = scorer.lim[arm]
        rng = np.random.default_rng(seed)
        self.Q = rng.uniform(self.lo, self.hi, size=(table, 7))
        self.P = np.array([self._pt(q) for q in self.Q])

    def _pt(self, q):
        M = self.sc.cf[self.arm](q)
        if self.point == "hand":
            f1 = M[self.SHP.IDX["robotiq_85_left_finger_tip_link"]][:3, 3]
            f2 = M[self.SHP.IDX["robotiq_85_right_finger_tip_link"]][:3, 3]
            return (f1 + f2) * 0.5
        return M[self.SHP.IDX["end_effector_link"]][:3, 3]

    def solve(self, target, tol=0.005, n_seeds=N_SEEDS, floor=None,
              seam=None, extra_seeds=()):
        """(q, residual_m, why). q is None when nothing met the tolerance.

        `floor` and `seam`, when given, are applied AFTER the geometric solve
        so a refusal can distinguish "the arm cannot get there" from "it can,
        but not safely" -- two different answers that a single boolean hides.
        """
        from scipy.optimize import minimize
        target = np.asarray(target, float)
        d = np.linalg.norm(self.P - target, axis=1)
        order = np.argsort(d)[:n_seeds]
        seeds = [np.asarray(s, float) for s in extra_seeds] + \
                [self.Q[i] for i in order]

        def cost(q):
            return float(np.sum((self._pt(q) - target) ** 2))

        sols = []
        for s0 in seeds:
            r = minimize(cost, s0, method="Nelder-Mead",
                         options=dict(maxiter=2500, xatol=1e-6, fatol=1e-12))
            q = np.clip(r.x, self.lo, self.hi)
            e = float(np.linalg.norm(self._pt(q) - target))
            if e <= tol:
                sols.append((e, q))
        if not sols:
            best_raw = float(d[order[0]])
            return None, best_raw, (
                "no solution within %.1f mm; %d seeded starts, closest raw "
                "sample %.1f mm. This is a SEARCHED refusal, not an assumed "
                "one." % (tol * 1000, len(seeds), best_raw * 1000))
        sols.sort(key=lambda t: t[0])
        if floor is None and seam is None:
            e, q = sols[0]
            return q, e, "solved to %.2f mm" % (e * 1000)
        best = None
        for e, q in sols:
            c = self.sc.clearance_parts(self.arm, q)["moving_chain_m"]
            sm = min(np.pi - abs(float(q[i])) for i in CONT_IDX)
            okc = floor is None or c >= floor
            oks = seam is None or sm >= seam
            key = (okc and oks, c)
            if best is None or key > best[0]:
                best = (key, e, q, c, sm)
        (good, _), e, q, c, sm = best
        if not good:
            return None, e, (
                "reachable geometrically (%.2f mm) but clearance %.4f m / "
                "seam %.2f rad fails the limits" % (e * 1000, c, sm))
        return q, e, ("solved to %.2f mm, clearance %.4f m, seam %.2f rad"
                      % (e * 1000, c, sm))


def self_test(verbose=True):
    """Must hit points known to be reachable, and must refuse a silly one."""
    import solve_home_pose as SHP
    import home_positions as hp
    sc = SHP.Scorer()
    ok_all = True
    for arm in ("left", "right"):
        ik = ArmIK(sc, arm, point="ee", table=20000)
        qh = np.array(hp.load_home_radians(arm))
        # (1) the home EE itself -- must solve to ~0
        home_ee = ik._pt(qh)
        q, e, why = ik.solve(home_ee, tol=0.005)
        assert q is not None and e < 0.005, \
            "%s: could not re-solve its OWN home EE (%s)" % (arm, why)
        if verbose:
            print("%-5s home EE      %s" % (arm, why))
        # (2) a point sampled from real FK -- reachable by construction
        rng = np.random.default_rng(5)
        qr = np.clip(qh + rng.uniform(-0.6, 0.6, 7), ik.lo, ik.hi)
        tgt = ik._pt(qr)
        q, e, why = ik.solve(tgt, tol=0.005)
        assert q is not None, "%s: missed an FK-constructed point (%s)" \
            % (arm, why)
        if verbose:
            print("%-5s FK-sampled   %s" % (arm, why))
        # (3) far outside the arm -- must refuse, and say it searched
        q, e, why = ik.solve(np.array([0.0, 5.0, 1.0]), tol=0.005)
        assert q is None, "%s: claimed to reach a point 5 m away" % arm
        assert "SEARCHED" in why
        if verbose:
            print("%-5s 5 m away     REFUSED -- %s" % (arm, why.split(";")[0]))
    if verbose:
        print("arm_ik self-test PASSED")
    return ok_all


if __name__ == "__main__":
    self_test(verbose=True)
