#!/usr/bin/env python3
"""What a pose actually clears, PER WEARER LINK and PER ARM SEGMENT.

    python3 scripts/measure_home_clearance.py                    # config vs legacy
    python3 scripts/measure_home_clearance.py --size measured_adult
    python3 scripts/measure_home_clearance.py --pose new=recordings/baselines/home_wide.json
    python3 scripts/measure_home_clearance.py --self-test

WHY THIS EXISTS
---------------
"Clearance 0.2202 m" is one number over twelve body parts and nine arm
segments, and on this rig it is usually a number about the BACKPACK FRAME
rather than about the arm. `base_link` is the first link of `mount_guard`'s
chain and no joint moves it -- measured, not assumed: over fifty random poses
across the whole joint range it returns one distinct position per arm. So the
worst clearance over the whole chain is capped by a piece of fixed structure,
and a report that gives only the minimum cannot tell "the arm is close to the
person" from "the frame the arm is bolted to is close to the person".

Those need different fixes. The first is a pose. The second is a bracket.

So every row below is split three ways:

    whole chain     what `mount_guard_node` enforces, base_link included
    moving chain    shoulder_link outward -- everything a home pose controls
    mount cap       the fixed point alone, which is the ceiling on the first

THE MODEL IS A PARAMETER, because the mannequin is not the wearer.
`--size` selects a profile from config/wearer_sizes through
`wearer_posture.wearer_model`, which is the ONE source the guard itself reads
(docs/ENGINEERING_LOG.md rule 11). Comparing two poses against two different bodies is a
mistake this file makes impossible: the body is chosen once and both poses are
scored against it.

CONTROLS, and there is no table without them:

    the fast distance field       agrees with `mount_guard_node.dist_point`
    reproduces the guard          point by point, every primitive
    a known answer                a pose constructed at a stated distance
                                  from a single primitive reads that distance
    it can FAIL                   a pose driven into the torso must read
                                  NEGATIVE, not a small positive number
    it can tell poses apart       two different poses must not produce
                                  identical rows (docs/ENGINEERING_LOG.md's "everything
                                  matches" failure mode)
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
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from srl_fk import FK, CompiledFK                                  # noqa: E402
from srl_teleop import mount_guard_node as MG                      # noqa: E402
from srl_teleop import wearer_posture as WP                        # noqa: E402

# WHERE THE ARM STOPS BEING STRUCTURE AND STARTS BEING A POSE. MEASURED.
#
# Over 40 random poses across the whole joint range, both arms:
#
#     base_link        0.000000 m of movement   FIXED
#     shoulder_link    0.000000 m               FIXED
#     half_arm_1_link  0.0058 / 0.0061 m        6 mm -- structure, in effect
#     half_arm_2_link  0.266 m                  a pose
#
# `joint_1` turns about the axis those first two origins sit on, so it moves
# neither of them. The first segment a home pose can actually place is
# `half_arm_1_link -> half_arm_2_link`, and that is where the moving chain
# starts. Set to 1 first, and the control below caught it inside a minute:
# two completely different poses both reported 0.3170 m, because the number
# was about a segment that does not move. `controls()` re-measures this every
# run rather than trusting the comment.
MOVING_FIRST_SEG = 2
ARMS = ("left", "right")


# --------------------------------------------------------------- the body
def body(size=None, posture="down"):
    """Wearer primitives, prepared for the vectorised distance field."""
    out = []
    for nm, kind, prm, ctr, rpy in WP.wearer_model(posture, size=size,
                                                   present=True):
        R = (np.asarray(WP.rot_matrix(rpy), float)
             if any(abs(v) > 1e-12 for v in rpy) else None)
        out.append((nm, kind, np.asarray(prm, float),
                    np.asarray(ctr, float), R, rpy))
    return out


def dist_to(P, prim):
    """Signed distance from every row of P to one primitive. Vectorised.

    Identical to `mount_guard_node.dist_point`, and `controls()` proves it
    rather than asserting it.
    """
    _, kind, prm, ctr, R, _ = prim
    Q = np.asarray(P, float) - ctr
    if R is not None:
        Q = Q @ R
    if kind == "box":
        half = prm * 0.5
        aq = np.abs(Q)
        d = np.maximum(aq - half, 0.0)
        n = np.linalg.norm(d, axis=1)
        inside = n <= 0
        if inside.any():
            n = np.where(inside, -np.min(half - aq, axis=1), n)
        return n
    if kind == "cylinder":
        r, L = float(prm[0]), float(prm[1])
        radial = np.hypot(Q[:, 0], Q[:, 1]) - r
        axial = np.abs(Q[:, 2]) - L * 0.5
        both = (radial <= 0) & (axial <= 0)
        outv = np.hypot(np.maximum(radial, 0.0), np.maximum(axial, 0.0))
        return np.where(both, np.maximum(radial, axial), outv)
    return np.linalg.norm(Q, axis=1) - float(prm[0])


# ---------------------------------------------------------------- the arm
class Rig:
    def __init__(self, size=None, posture="down"):
        self.fk = FK()
        self.chain = {a: CompiledFK(self.fk, a, MG.CHAIN) for a in ARMS}
        self.prims = body(size, posture)
        self.size = (size if isinstance(size, dict)
                     else WP.size_profile(size))
        self.posture = posture

    def segments(self, arm, q):
        pts = np.array([M[:3, 3] for M in self.chain[arm](np.asarray(q,
                                                                    float))])
        A, B = pts[:-1], pts[1:]
        t = np.linspace(0.0, 1.0, MG.SAMPLES + 1)
        return A[:, None, :] + (B - A)[:, None, :] * t[None, :, None]

    def mount_cap(self, arm):
        """The immobile STUB's own clearance -- the ceiling on the whole chain.

        Not just `base_link`: the stub is every segment before
        MOVING_FIRST_SEG, sampled the way the guard samples it, at an
        arbitrary pose (it does not depend on the pose, which is the point).
        """
        S = self.segments(arm, np.zeros(7))[:MOVING_FIRST_SEG].reshape(-1, 3)
        return min(float((dist_to(S, pr) - MG.TUBE_R).min())
                   for pr in self.prims)

    def report(self, arm, q):
        S = self.segments(arm, q)
        whole = S.reshape(-1, 3)
        moving = S[MOVING_FIRST_SEG:].reshape(-1, 3)
        per, per_mov = {}, {}
        for pr in self.prims:
            per[pr[0]] = float((dist_to(whole, pr) - MG.TUBE_R).min())
            per_mov[pr[0]] = float((dist_to(moving, pr) - MG.TUBE_R).min())
        segs = []
        for i in range(S.shape[0]):
            v, who = min((float((dist_to(S[i], pr) - MG.TUBE_R).min()), pr[0])
                         for pr in self.prims)
            segs.append(dict(segment="%s -> %s" % (MG.CHAIN[i],
                                                   MG.CHAIN[i + 1]),
                             fixed_end=(i < MOVING_FIRST_SEG),
                             clearance_m=round(v, 4), to=who))
        wv, wk = min((v, k) for k, v in per.items())
        mv, mk = min((v, k) for k, v in per_mov.items())
        return dict(whole_chain_m=round(wv, 4), whole_chain_to=wk,
                    moving_chain_m=round(mv, 4), moving_chain_to=mk,
                    mount_cap_m=round(self.mount_cap(arm), 4),
                    per_wearer_link_m={k: round(v, 4) for k, v in per.items()},
                    per_wearer_link_moving_m={k: round(v, 4)
                                              for k, v in per_mov.items()},
                    per_arm_segment=segs)


# ------------------------------------------------------------ loading poses
def _from_config():
    import re
    out = {}
    for arm in ARMS:
        txt = open(os.path.join(ROOT,
                                "config/home_positions_%s.txt" % arm)).read()
        d = [float(re.search(r"(?m)^joint_%d:\s*([-+0-9.]+)\s*$" % (i + 1),
                             txt).group(1)) for i in range(7)]
        out[arm] = [math.radians(v) for v in d]
    return out


def load_pose(spec):
    """`config`, `legacy`, or a path to a solver JSON."""
    if spec == "config":
        return _from_config()
    if spec == "legacy":
        import solve_home_pose as S
        return {a: list(S.kortex_to_ros(S.LEGACY_HOME_KORTEX[a]))
                for a in ARMS}
    path = spec if os.path.isabs(spec) else os.path.join(ROOT, spec)
    d = json.load(open(path))
    sol = d.get("solved") or d
    if "q_left" in sol:
        return {a: list(sol["q_%s" % a]) for a in ARMS}
    if "home_from_config_rad" in d:
        return {a: list(d["home_from_config_rad"][a]) for a in ARMS}
    raise ValueError("%s carries no pose I recognise" % spec)


# ----------------------------------------------------------------- controls
def controls(rig, verbose=True):
    """Every one of these has to pass before a row is printed."""
    ok = True
    rng = np.random.default_rng(11)

    # 1. the fast field IS the guard's field
    worst = 0.0
    for arm in ARMS:
        lo, hi, _ = rig.fk.limits(arm)
        for _ in range(6):
            P = rig.segments(arm, rng.uniform(lo, hi)).reshape(-1, 3)
            for pr in rig.prims:
                fast = dist_to(P, pr)
                slow = np.array([MG.dist_point(p, pr[1], pr[2], pr[3], pr[5])
                                 for p in P])
                worst = max(worst, float(np.abs(fast - slow).max()))
    ok &= worst < 1e-9
    if verbose:
        print("   vs mount_guard_node.dist_point, every point   %.1e -> %s"
              % (worst, "PASS" if worst < 1e-9 else "FAIL"))

    # 2. A KNOWN ANSWER. One sphere, placed a stated distance from a point
    #    this file computes independently of the distance field.
    probe = np.array([[0.5, 0.25, 1.30]])
    sphere = ("probe", "sphere", np.array([0.10]),
              probe[0] + np.array([0.0, 0.40, 0.0]), None, (0., 0., 0.))
    want = 0.40 - 0.10
    got = float(dist_to(probe, sphere)[0])
    ok &= abs(got - want) < 1e-12
    if verbose:
        print("   known answer: 0.40 m to a r=0.10 sphere       %.6f -> %s"
              % (got, "PASS" if abs(got - want) < 1e-12 else "FAIL"))

    # 3. IT MUST BE ABLE TO READ NEGATIVE. A point placed INSIDE the torso
    #    box has to come back negative, not a small positive number. A
    #    clearance instrument that saturates at zero cannot report a hit, and
    #    reporting hits is the only reason this file exists.
    torso = [p for p in rig.prims if p[0] == "torso"][0]
    inside = float(dist_to(np.array([torso[3]]), torso)[0])
    ok &= inside < 0
    if verbose:
        print("   can read a HIT: a point inside the torso      %+.4f -> %s"
              % (inside, "PASS" if inside < 0 else "FAIL"))

    # 4. WHERE THE STRUCTURE ENDS, RE-MEASURED EVERY RUN. If a mount or a
    #    URDF change ever makes another link move, MOVING_FIRST_SEG is wrong
    #    and every "moving chain" number below is about the wrong thing.
    rng2 = np.random.default_rng(31)
    spread = {}
    for arm in ARMS:
        lo, hi, _ = rig.fk.limits(arm)
        P = np.array([[M[:3, 3] for M in rig.chain[arm](rng2.uniform(lo, hi))]
                      for _ in range(25)])
        for i, ln in enumerate(MG.CHAIN):
            d = float(np.linalg.norm(P[:, i, :] - P[:, i, :].mean(0),
                                     axis=1).max())
            spread[ln] = max(spread.get(ln, 0.0), d)
    # The stub is links 0..MOVING_FIRST_SEG. The first two are immobile to
    # floating point; the last is allowed 10 mm and no more, and that
    # allowance is DECLARED here rather than hidden in a tolerance -- calling
    # a link "structure" when it moves 6 mm is a judgement, and a link that
    # started moving 60 mm would have to stop being called that.
    STUB_SLOP_M = 0.010
    hard = max(spread[MG.CHAIN[i]] for i in range(MOVING_FIRST_SEG))
    near = spread[MG.CHAIN[MOVING_FIRST_SEG]]
    far = spread[MG.CHAIN[MOVING_FIRST_SEG + 1]]
    fixed_ok = hard < 1e-9 and near < STUB_SLOP_M
    moves_ok = far > 0.05
    ok &= fixed_ok and moves_ok
    if verbose:
        print("   stub links immobile (%.1e, %.3f m <= %.3f), first moving "
              "segment swings %.3f m -> %s"
              % (hard, near, STUB_SLOP_M, far,
                 "PASS" if (fixed_ok and moves_ok) else "FAIL"))

    # 5. IT MUST BE ABLE TO TELL TWO POSES APART. docs/ENGINEERING_LOG.md's "everything
    #    matches" row: a comparison that cannot differ is not a comparison.
    # The two poses are chosen to be far apart in the quantity being
    # measured -- the shipped home, which is close to the wearer, against a
    # pose folded hard inboard. Two arbitrary poses would have agreed to a
    # couple of millimetres and passed a test that proved nothing.
    a = rig.report("left", _from_config()["left"])
    b = rig.report("left", np.array([-1.2, 1.4, 0.3, 2.0, 0.0, 1.1, 0.0]))
    differs = abs(a["moving_chain_m"] - b["moving_chain_m"]) > 0.05
    ok &= differs
    if verbose:
        print("   two poses give two answers (home vs folded)  "
              "%.4f vs %.4f -> %s"
              % (a["moving_chain_m"], b["moving_chain_m"],
                 "PASS" if differs else "FAIL"))
    return ok


# ------------------------------------------------------------------- output
def show(rig, poses, verbose=True):
    names = list(poses)
    rows = {n: {a: rig.report(a, poses[n][a]) for a in ARMS} for n in names}
    if not verbose:
        return rows
    w = 11
    print("\nBODY: %s, arms %s" % (rig.size["profile"], rig.posture))
    print("   chest %.3f x %.3f x %.3f m, shoulder half-span %.3f, "
          "upper arm %.3f, forearm %.3f"
          % (rig.size["chest_w"], rig.size["chest_d"], rig.size["chest_h"],
             rig.size["shoulder_x"], rig.size["upper_arm_len"],
             rig.size["lower_arm_len"]))
    for arm in ARMS:
        print("\n%s ARM -- clearance in metres, WHOLE chain (what the guard "
              "enforces)" % arm.upper())
        print("   %-14s%s" % ("wearer link",
                              "".join("%*s" % (w, n) for n in names)))
        for k in rows[names[0]][arm]["per_wearer_link_m"]:
            print("   %-14s%s" % (k, "".join(
                "%*.4f" % (w, rows[n][arm]["per_wearer_link_m"][k])
                for n in names)))
        print("   %-14s%s" % ("-- WORST --", "".join(
            "%*.4f" % (w, rows[n][arm]["whole_chain_m"]) for n in names)))
        print("   %-14s%s" % ("   binds on", "".join(
            "%*s" % (w, rows[n][arm]["whole_chain_to"]) for n in names)))
        print("   %-14s%s" % ("MOVING only", "".join(
            "%*.4f" % (w, rows[n][arm]["moving_chain_m"]) for n in names)))
        print("   %-14s%s" % ("   binds on", "".join(
            "%*s" % (w, rows[n][arm]["moving_chain_to"]) for n in names)))
        print("   %-14s%*.4f   (the immobile stub -- ceiling on WORST)"
              % ("mount cap", w, rows[names[0]][arm]["mount_cap_m"]))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", action="append", default=None,
                    help="NAME=SPEC; SPEC is config, legacy, or a JSON path")
    ap.add_argument("--size", default="mannequin")
    ap.add_argument("--posture", default="down")
    ap.add_argument("--out", default=None)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    rig = Rig(size=a.size, posture=a.posture)
    print("INSTRUMENT CHECKS")
    if not controls(rig):
        print("REFUSING: a control failed.")
        return 6
    if a.self_test:
        return 0

    specs = a.pose or ["home=config", "legacy=legacy"]
    poses = {}
    for s in specs:
        n, _, v = s.partition("=")
        poses[n] = load_pose(v or n)
    rows = show(rig, poses)
    if a.out:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        json.dump(dict(size=rig.size, posture=rig.posture,
                       poses={n: {k: list(map(float, v))
                                  for k, v in p.items()}
                              for n, p in poses.items()},
                       clearance=rows), open(a.out, "w"), indent=2,
                  default=float)
        print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
