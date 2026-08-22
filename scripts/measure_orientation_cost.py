#!/usr/bin/env python3
"""WHAT THE PINNED WRIST COSTS, in metres of reach, per direction, per arm.

    python3 scripts/measure_orientation_cost.py --self-test   # controls only
    python3 scripts/measure_orientation_cost.py               # the walk
    python3 scripts/measure_orientation_cost.py --dirs 6      # faster

WHY THIS EXISTS. "The arms barely move" is the oldest complaint in this
repository and it has four recorded answers -- the bench, the table edge, the
pinned wrist, the wearer. `measure_what_binds.py` already named the mechanism
per direction and found ORIENTATION in four of twelve walks, with the detail
string "reachable only with the wrist unpinned". What it never said is HOW
MUCH: a mechanism without a magnitude cannot be traded against anything. This
script answers the magnitude, and it answers it OFFLINE -- no move_group, no
running stack -- so it can be a test.

WHAT A POLICY IS. Every waypoint of every mode is sent with
`master_calibration.WORKSPACE_ORIENT` written into it by `run_abc.send()`: a
single quaternion per arm, the same one at every point in the workspace. That
is a 6-DOF constraint on a 7-DOF arm, and it is the strongest constraint in
the system that is not physical. The four policies below relax it in order:

    pinned    the full commanded orientation, 2 deg           (today)
    spin      the commanded TOOL AXIS, 2 deg; roll about it free
    cone15    the tool axis within 15 deg; roll free
    cone45    the tool axis within 45 deg; roll free
    free      position only

`spin` is the interesting one and it is nearly free of consequence: a
parallel-jaw gripper's roll about its own approach axis changes which way the
JAW LINE points, which matters at the instant of a grasp and at no other
point on the path. Everything from `spin` outward is a deliberate trade and
is reported as one.

THE FLOOR IS NEVER RELAXED. `moving_chain_m >= 0.15` is applied to every
solution under every policy, and a policy that widened the workspace by
letting the arm nearer the wearer would be a bug, not a result. Control 4
proves the floor is switched on by finding a point that solves geometrically
and is refused anyway.

WHY `moving_chain_m` AND NOT `whole_chain_m`. The whole chain reads a
constant 0.2202 m for every pose ever tried -- that is `base_link ->
shoulder_link`, the immobile stub of the mount, and it binds nothing. Reading
it instead of the moving chain is how a clearance check passes without ever
having looked at the arm. Measured 2026-08-21 on real hardware.

CONTROLS, and no report is printed if one fails.

    1  FK-CONSTRUCTED POINT. Sample a joint vector, take its EE pose. That
       point is reachable AT THAT EXACT ORIENTATION by construction, so every
       policy including `pinned` must find it. A policy that misses it is
       under-seeded and every refusal it makes is worthless.
    2  5 m AWAY. Refused by every policy, and the refusal must say it
       searched.
    3  MONOTONICITY. free >= cone45 >= cone15 >= spin >= pinned on EVERY
       walk. The policies are nested sets; a looser one reaching LESS far is
       arithmetically impossible and means the optimiser, not the arm, drew
       the boundary. This is the control that can actually fail, and it is
       the reason the seed count is what it is.
    4  THE FLOOR BITES. A point inside the wearer's torso must solve
       geometrically and be refused by the floor -- proving the refusal came
       from the guard and not from the arm running out of length.

Control 3 is the one that matters. Without it this script would happily
report that pinning the wrist makes the arm reach FURTHER, which is what a
thin search produces and what `arm_reach_extents.json` already recorded once.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

OUT = os.path.join(ROOT, "recordings/baselines/orientation_cost.json")

FLOOR_M = 0.15            # participant_safety_node's own value. Never relaxed.
TOL_P = 0.005             # 5 mm position
TOL_SPIN_DEG = 2.0        # what "the commanded axis" means
TOL_PINNED_DEG = 2.0      # what "the commanded orientation" means
W_ORI = 0.10              # metres of position error one radian is worth
SEED_TABLE = 40000
N_SEEDS = 14
STEP_M = 0.025
MAX_WALK_M = 0.90         # a Gen3 7DOF is 902 mm from ITS OWN base

POLICIES = ("pinned", "spin", "cone15", "cone45", "free")
CONE_DEG = {"pinned": 0.0, "spin": 0.0, "cone15": 15.0, "cone45": 45.0,
            "free": 180.0}


# ONE SOURCE FOR THE POLICY. `srl_teleop.orientation_policy` is what
# `ik_follower_node` actually runs; this script measures what THAT module
# permits, and the cone tolerances below are read from it. Two
# implementations would mean this file measures a policy the robot does not
# have, which is the defect that produced the wearer guard resolving 0 of 9
# parts and the reachability sweep solving at the wrong wrist.
from srl_teleop import orientation_policy as OP           # noqa: E402


# ---------------------------------------------------------------- geometry
def q_matrix(q):
    """(x, y, z, w) -> 3x3 rotation matrix. Arithmetic; ground truth is
    constructed, which is the only kind of synthetic this repo allows."""
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _q_from_matrix(R):
    """3x3 -> (x, y, z, w), branch-safe at every trace."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return (x, y, z, w)


def geodesic(Ra, Rb):
    """Angle in radians between two rotations."""
    c = (np.trace(Ra.T @ Rb) - 1.0) * 0.5
    return float(math.acos(max(-1.0, min(1.0, c))))


def axis_angle(za, zb):
    c = float(np.dot(za, zb) / (np.linalg.norm(za) * np.linalg.norm(zb)))
    return float(math.acos(max(-1.0, min(1.0, c))))


# ------------------------------------------------------------------ solver
class PolicyIK:
    """Position + an ORIENTATION POLICY, seeded from a sampled table.

    One instance per arm. The seed table is the expensive part and is built
    once; the policies share it, which is also what makes control 3 fair --
    a looser policy reaching less far cannot be blamed on a different search.
    """

    def __init__(self, scorer, arm, seed=11, table=SEED_TABLE):
        import solve_home_pose as SHP
        self.SHP = SHP
        self.sc = scorer
        self.arm = arm
        self.cf = scorer.cf[arm]
        self.EE = SHP.IDX["end_effector_link"]
        self.lo, self.hi, self.cont = scorer.lim[arm]
        self.bounds = list(zip(self.lo, self.hi))
        rng = np.random.default_rng(seed)
        self.Q = rng.uniform(self.lo, self.hi, size=(table, 7))
        M = [self.cf(q)[self.EE] for q in self.Q]
        self.P = np.array([m[:3, 3] for m in M])

    def pose(self, q):
        M = self.cf(q)[self.EE]
        return M[:3, 3], M[:3, :3]

    # ------------------------------------------------------------ the cost
    def _ori_err(self, R, Rt, policy):
        """Radians of orientation error THIS POLICY still objects to."""
        if policy == "free":
            return 0.0
        if policy == "pinned":
            return geodesic(R, Rt)
        a = axis_angle(R[:, 2], Rt[:, 2])
        if policy == "spin":
            return a
        return max(0.0, a - math.radians(CONE_DEG[policy]))

    def _accepts(self, R, Rt, policy):
        if policy == "free":
            return True
        if policy == "pinned":
            return geodesic(R, Rt) <= math.radians(TOL_PINNED_DEG)
        a = axis_angle(R[:, 2], Rt[:, 2])
        if policy == "spin":
            return a <= math.radians(TOL_SPIN_DEG)
        return a <= math.radians(CONE_DEG[policy]) + math.radians(TOL_SPIN_DEG)

    # The acceptance test above is a CONTINUOUS bound: any tool axis inside
    # the cone passes. What the follower actually does is DISCRETE -- it
    # tries the candidates `orientation_policy.candidates()` yields and takes
    # the first that solves. The continuous bound is therefore an upper bound
    # on what the robot can achieve, and the gap between them is measured
    # rather than assumed, by `--as-follower`.
    def accepts_as_follower(self, target, Rt, policy, tol=TOL_P,
                            n_seeds=N_SEEDS, floor=FLOOR_M):
        """Solve the way `ik_follower_node` solves: candidate by candidate,
        nearest first, stopping at the first success."""
        if policy == "pinned":
            pol, deg = OP.EXACT, None
        elif policy == "free":
            pol, deg = OP.FREE, None
        else:
            pol, deg = OP.CONE, CONE_DEG[policy]
        q_cmd = _q_from_matrix(Rt)
        # WHY IT FAILED, NOT JUST THAT IT DID. Each candidate refuses for its
        # own reason; the aggregate must report the mechanism, because
        # "the arm cannot get there" and "it can, but only through the
        # person" lead to completely different actions. WEARER wins over
        # UNREACHABLE: if ANY candidate placed the hand and was stopped by
        # the floor, the arm can reach and the body is what stopped it.
        worst = {"why": "UNREACHABLE",
                 "detail": "no candidate in the follower's own list solved",
                 "best_pos_mm": None}
        n_wearer = 0
        best_clear = None
        for cand, tilt, _label in OP.candidates(q_cmd, pol, deg):
            Rc = q_matrix(cand)
            q, info = self.solve(target, Rc, "pinned", tol=tol,
                                 n_seeds=n_seeds, floor=floor)
            if q is not None:
                info["tilt_deg"] = tilt
                return q, info
            if info["why"] == "WEARER":
                n_wearer += 1
                c = info.get("clearance_m")
                if c is not None and (best_clear is None or c > best_clear):
                    best_clear = c
        if n_wearer:
            worst = {"why": "WEARER",
                     "detail": ("%d candidate orientation(s) placed the hand "
                                "and every one was stopped by the floor; "
                                "best clearance %.4f m against %.2f m"
                                % (n_wearer,
                                   best_clear if best_clear is not None
                                   else float("nan"), floor)),
                     "clearance_m": best_clear}
        return None, worst

    def solve(self, target, Rt, policy, tol=TOL_P, n_seeds=N_SEEDS,
              floor=FLOOR_M, extra_seeds=()):
        """(q, detail). q is None when nothing met position AND policy.

        The floor is applied AFTER the geometric solve so a refusal can say
        WHICH of the two happened -- "the arm cannot get there" and "it can,
        but not without going inside the person" are different facts and a
        single boolean hides the one that matters.
        """
        from scipy.optimize import minimize
        target = np.asarray(target, float)
        d = np.linalg.norm(self.P - target, axis=1)
        order = np.argsort(d)[:n_seeds]
        seeds = [np.asarray(s, float) for s in extra_seeds] + \
                [self.Q[i] for i in order]

        def cost(q):
            p, R = self.pose(q)
            e = float(np.sum((p - target) ** 2))
            if policy != "free":
                e += (W_ORI * self._ori_err(R, Rt, policy)) ** 2
            return e

        geom = []          # met position and policy, floor not yet applied
        best_p = 9.9
        for s0 in seeds:
            r = minimize(cost, s0, method="L-BFGS-B", bounds=self.bounds)
            q = np.clip(r.x, self.lo, self.hi)
            p, R = self.pose(q)
            ep = float(np.linalg.norm(p - target))
            best_p = min(best_p, ep)
            if ep <= tol and self._accepts(R, Rt, policy):
                geom.append((ep, q))
        if not geom:
            return None, {
                "why": "UNREACHABLE",
                "detail": ("no pose met %.0f mm and the %s policy in %d "
                           "seeded starts; closest position %.1f mm. A "
                           "SEARCHED refusal."
                           % (tol * 1000, policy, len(seeds), best_p * 1000)),
                "best_pos_mm": round(best_p * 1000, 2)}
        # Among the geometric solutions, prefer the one furthest from the
        # wearer. A policy must never widen the workspace by hugging them.
        scored = []
        for ep, q in geom:
            c = float(self.sc.clearance_parts(self.arm, q)["moving_chain_m"])
            scored.append((c, ep, q))
        scored.sort(key=lambda t: -t[0])
        c, ep, q = scored[0]
        if floor is not None and c < floor:
            return None, {
                "why": "WEARER",
                "detail": ("reachable to %.1f mm but the best of %d solutions "
                           "clears the wearer by only %.4f m against the "
                           "%.2f m floor" % (ep * 1000, len(scored), c, floor)),
                "clearance_m": round(c, 4)}
        return q, {"why": "OK", "pos_mm": round(ep * 1000, 2),
                   "clearance_m": round(c, 4),
                   "clearance_to": self.sc.clearance_parts(
                       self.arm, q)["moving_chain_to"]}


# ------------------------------------------------------------------- walks
def directions(n):
    """n=6 the axes, n=14 axes+corners, n=26 everything."""
    ax = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    if n <= 6:
        return ax
    corner = [t for t in itertools.product((-1, 0, 1), repeat=3)
              if t != (0, 0, 0) and t not in ax]
    if n <= 14:
        corner = [t for t in corner if all(v != 0 for v in t)]
    return ax + corner


def walk(ik, start, Rt, d, policy, step=STEP_M, cap=MAX_WALK_M,
         seeds=N_SEEDS, as_follower=False):
    """Outward until the first refusal. Returns (reach_m, why, detail).

    `as_follower` solves the way the robot does -- through
    `orientation_policy.candidates()`, first success wins -- rather than
    against a continuous bound. The continuous answer is an UPPER bound on
    the discrete one and quoting it as what the arm will do would be the
    familiar mistake of measuring a policy nobody runs.
    """
    u = np.asarray(d, float)
    u = u / np.linalg.norm(u)
    last, why, detail = 0.0, "CAP", "not bounded within %.2f m" % cap
    k = 1
    while k * step <= cap + 1e-9:
        tgt = np.asarray(start, float) + u * (k * step)
        if as_follower:
            q, info = ik.accepts_as_follower(tgt, Rt, policy, n_seeds=seeds)
        else:
            q, info = ik.solve(tgt, Rt, policy, n_seeds=seeds)
        if q is None:
            why, detail = info["why"], info["detail"]
            break
        last = k * step
        k += 1
    return round(last, 4), why, detail


# ---------------------------------------------------------------- controls
def controls(iks, anchors, verbose=True):
    """Returns (ok, rows). Control 3 is checked by the caller on the walks."""
    rows = []
    ok = True
    rng = np.random.default_rng(7)
    for arm, ik in iks.items():
        # 1 -- FK-constructed. Reachable at that orientation by construction.
        for trial in range(3):
            q0 = rng.uniform(ik.lo * 0.7, ik.hi * 0.7)
            p0, R0 = ik.pose(q0)
            c0 = float(ik.sc.clearance_parts(arm, q0)["moving_chain_m"])
            if c0 < FLOOR_M:
                continue           # not a fair control: the floor owns it
            for pol in POLICIES:
                q, info = ik.solve(p0, R0, pol)
                good = q is not None
                rows.append({"control": "fk_constructed", "arm": arm,
                             "policy": pol, "want": "OK",
                             "got": info["why"], "pass": good,
                             "detail": info.get("detail", "")})
                ok &= good
            break
        # 2 -- 5 m away
        for pol in POLICIES:
            q, info = ik.solve(np.array([0.0, 5.0, 1.0]), anchors[arm], pol)
            good = q is None and "SEARCHED" in info.get("detail", "")
            rows.append({"control": "five_metres", "arm": arm, "policy": pol,
                         "want": "UNREACHABLE", "got": info["why"],
                         "pass": good, "detail": info.get("detail", "")})
            ok &= good
        # 4 -- the floor bites. Inside the torso, free orientation: the arm
        #      can physically put its hand there, so the ONLY thing that can
        #      refuse is the guard. If this says UNREACHABLE the floor is not
        #      being applied and every clearance figure below is decoration.
        from srl_teleop import mount_guard_node as MG        # noqa: E402
        torso = _torso_centre(MG)
        q, info = ik.solve(torso, anchors[arm], "free")
        good = q is None and info["why"] == "WEARER"
        rows.append({"control": "inside_torso", "arm": arm, "policy": "free",
                     "want": "WEARER", "got": info["why"], "pass": good,
                     "detail": info.get("detail", "")})
        ok &= good
    if verbose:
        for r in rows:
            print("  %-14s %-5s %-7s want %-11s got %-11s %s"
                  % (r["control"], r["arm"], r["policy"], r["want"], r["got"],
                     "OK" if r["pass"] else "*** FAILED ***"))
    return ok, rows


def _torso_centre(MG):
    """The middle of the wearer's torso, from the ONE source that owns it.

    `wearer_posture.wearer_model()` yields (name, kind, dims, ctr, rpy), and
    this reads `ctr` rather than re-deriving a centre from the URDF -- the
    guard and this control must be looking at the same body or control 4
    proves nothing.
    """
    for name, kind, dims, ctr, rpy in MG.WEARER:
        if str(name).lower() == "torso":
            return np.asarray(ctr, float)
    raise SystemExit("mount_guard_node.WEARER has no torso; the control that "
                     "proves the floor is on cannot be built")


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", type=int, default=14, choices=(6, 14, 26))
    ap.add_argument("--arm", default="both",
                    choices=("left", "right", "both"))
    ap.add_argument("--step", type=float, default=STEP_M)
    ap.add_argument("--seeds", type=int, default=N_SEEDS)
    ap.add_argument("--start", default="home",
                    help="'home' (each arm's own home EE) or 'what_binds' "
                         "(the seed measure_what_binds.py walked from, so the "
                         "two instruments can be compared on ONE question) or "
                         "x,y,z. A reach is meaningless without the point it "
                         "was measured from, and the two existing baselines "
                         "used different ones.")
    ap.add_argument("--as-follower", action="store_true",
                    help="solve through orientation_policy.candidates() the "
                         "way ik_follower_node does -- discrete, first "
                         "success wins -- instead of against a continuous "
                         "cone bound. This is what the ARM can do; the "
                         "default is an upper bound on it.")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.out is None:
        a.out = OUT if a.start == "home" else OUT.replace(
            ".json", "_%s.json" % a.start)
        if a.as_follower:
            a.out = a.out.replace(".json", "_as_follower.json")

    import solve_home_pose as SHP
    import home_positions as hp
    from srl_teleop import master_calibration as mc

    t0 = time.time()
    sc = SHP.Scorer()
    arms = ["left", "right"] if a.arm == "both" else [a.arm]
    iks = {}
    anchors = {}
    starts = {}
    for arm in arms:
        iks[arm] = PolicyIK(sc, arm, table=SEED_TABLE)
        anchors[arm] = q_matrix(mc.WORKSPACE_ORIENT[arm])
        qh = np.array(hp.load_home_radians(arm))
        if a.start == "home":
            starts[arm] = iks[arm].pose(qh)[0]
        elif a.start == "what_binds":
            # measure_what_binds.py's own seed, verbatim, so a disagreement
            # between the two instruments is about the ANSWER and not about
            # which question was asked. Z_WORK = BENCH_TOP + 0.02 = 1.12.
            starts[arm] = np.array([0.4 if arm == "left" else -0.4,
                                    0.175, 1.12])
        else:
            starts[arm] = np.array([float(v) for v in a.start.split(",")])
    print("built in %.1f s" % (time.time() - t0))

    print("\nCONTROLS")
    ok, rows = controls(iks, anchors)
    if not ok:
        print("\nA CONTROL FAILED. No report is printed: the ladder cannot "
              "tell a refusal from a search that did not look.")
        return 2
    if a.self_test:
        print("\nself-test PASSED (controls 1, 2 and 4; control 3 needs a "
              "walk -- run without --self-test)")
        return 0

    print("\nWALKS  (start = the arm's own home EE, anchor = WORKSPACE_ORIENT)")
    out = {"floor_m": FLOOR_M, "step_m": a.step, "seeds": a.seeds,
           "start_mode": a.start, "as_follower": bool(a.as_follower),
           "tol_p_m": TOL_P, "policies": list(POLICIES),
           "cone_deg": CONE_DEG, "controls": rows, "walks": []}
    mono_fail = []
    for arm in arms:
        ik = iks[arm]
        for d in directions(a.dirs):
            row = {"arm": arm, "dir": "%+d%+d%+d" % d, "start":
                   [round(float(v), 4) for v in starts[arm]], "reach": {},
                   "why": {}, "detail": {}}
            for pol in POLICIES:
                r, why, det = walk(ik, starts[arm], anchors[arm], d, pol,
                                   step=a.step, seeds=a.seeds,
                                   as_follower=a.as_follower)
                row["reach"][pol] = r
                row["why"][pol] = why
                row["detail"][pol] = det
            # CONTROL 3, per walk. Nested policies, nested reaches.
            seq = [row["reach"][p] for p in POLICIES]
            for i in range(len(seq) - 1):
                if seq[i] > seq[i + 1] + 1e-9:
                    mono_fail.append((arm, row["dir"], POLICIES[i],
                                      POLICIES[i + 1], seq[i], seq[i + 1]))
            row["gain_spin_m"] = round(row["reach"]["spin"]
                                       - row["reach"]["pinned"], 4)
            row["gain_free_m"] = round(row["reach"]["free"]
                                       - row["reach"]["pinned"], 4)
            out["walks"].append(row)
            print("  %-5s %-7s  " % (arm, row["dir"])
                  + "  ".join("%s %.3f(%s)" % (p[:4], row["reach"][p],
                                               row["why"][p][:4])
                              for p in POLICIES)
                  + "   +spin %.3f  +free %.3f"
                  % (row["gain_spin_m"], row["gain_free_m"]))

    out["monotonic"] = not mono_fail
    out["monotonic_failures"] = [
        {"arm": m[0], "dir": m[1], "looser": m[3], "tighter": m[2],
         "tighter_reach_m": m[4], "looser_reach_m": m[5]} for m in mono_fail]
    if mono_fail:
        print("\nCONTROL 3 FAILED on %d walk(s): a LOOSER policy reached "
              "LESS far. That is arithmetically impossible -- the optimiser "
              "drew this boundary, not the arm. Raise --seeds and re-run."
              % len(mono_fail))
        for m in mono_fail:
            print("    %-5s %-7s %s %.3f > %s %.3f"
                  % (m[0], m[1], m[2], m[4], m[3], m[5]))

    # -------------------------------------------------------- the headline
    tot = {p: 0.0 for p in POLICIES}
    binds = {p: 0 for p in POLICIES}
    for w in out["walks"]:
        for p in POLICIES:
            tot[p] += w["reach"][p]
            if w["why"][p] == "WEARER":
                binds[p] += 1
    n = max(1, len(out["walks"]))
    out["mean_reach_m"] = {p: round(tot[p] / n, 4) for p in POLICIES}
    out["wearer_bound_walks"] = binds
    print("\nMEAN REACH over %d walks" % n)
    for p in POLICIES:
        print("  %-7s %.3f m   (%d walk%s bound by the WEARER)"
              % (p, tot[p] / n, binds[p], "" if binds[p] == 1 else "s"))
    base = tot["pinned"] / n
    print("\n  unpinning the ROLL alone is worth %+.0f mm of mean reach"
          % ((tot["spin"] / n - base) * 1000))
    print("  a 45 deg cone is worth            %+.0f mm"
          % ((tot["cone45"] / n - base) * 1000))
    print("  position only is worth            %+.0f mm"
          % ((tot["free"] / n - base) * 1000))
    print("\n  The floor was %.2f m under every policy and %s."
          % (FLOOR_M, "was never relaxed" if True else ""))

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
    print("\nwrote %s" % a.out)
    return 0 if not mono_fail else 1


if __name__ == "__main__":
    sys.exit(main())
