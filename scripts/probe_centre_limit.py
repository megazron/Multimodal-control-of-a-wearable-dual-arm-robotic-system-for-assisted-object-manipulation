#!/usr/bin/env python3
"""HOW CLOSE TO THE CENTRE CAN THE ARM GET AT ALL, AND WHAT IS THE LAST THING
IN THE WAY?

    python3 scripts/probe_centre_limit.py --self-test
    python3 scripts/probe_centre_limit.py --levers

WHY THIS EXISTS AND WHY IT IS NOT ANOTHER COLUMN SWEEP. The column sweeps
(`search_centre_geometry.py`, `sweep_mount_geometry.py`) report the INNERMOST
column that passes, and at N=2 that number carries +/-75 mm of scatter, because
TRAC-IK returns a different null-space branch on every call and the sweep keeps
the WORST of the repeats. That is the correct way to certify a layout and the
wrong way to answer "could this ever work": a lever that moved the boundary by
50 mm is indistinguishable from noise, and a lever that moved it by 300 mm
would be obvious.

So this asks the question the levers are actually about, at the centreline
itself, and it asks it in the direction that CANNOT be beaten by a luckier
solver:

    take the grasp pose at |x| = 0.00, 0.05, 0.10
    solve it K times with collisions OFF -- every branch the solver will offer
    keep the BEST clearance any branch achieved, not the worst

If the best of K branches is still inside the 150 mm floor, no seeding, no
null-space projection and no solver change can rescue that configuration, and
the lever is dead for a reason that is geometric rather than statistical. If
the best branch clears, the configuration is worth certifying properly.

WHAT IS VARIED. The same five levers, and their combinations:

    table top and near edge   the wearer's standing distance and the surface
    mount transform          translation and rotation of BOTH mounts, mirrored,
                             applied exactly (see sweep_mount_geometry)
    approach heading         the wrist orientation, which under mode 06 is a
                             task variable and costs nothing to change
    wearer arm posture       swapped in the CAPSULE MODEL only, which is legal
                             here because collisions are off and the wearer is
                             measured geometrically -- this is the diagnostic
                             that says whether the wearer's own arms or the
                             torso is the binding body, and it is not a
                             proposal

CONTROLS, and no report without them:

    a pose driven into the torso reads NEGATIVE, names the torso, names a link
    a pose 1.6 m out does not solve at all
    the as-built centre reproduces the column sweeps' verdict for the centre
      column (COLLISION:WEARER, i.e. best branch still negative)
    deleting the wearer's arms CHANGES the answer at a distance where the
      forearm was named and does NOT change it where the torso was named --
      a posture switch that changes nothing anywhere is a posture switch that
      never reached the model
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from sweep_gap_vs_mount import chain_points                   # noqa: E402
from sweep_mount_geometry import Cand                         # noqa: E402
from search_t1_centre import CUBE                             # noqa: E402
from srl_teleop import mount_guard_node as MG                 # noqa: E402
from srl_teleop import wearer_posture as WP                   # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_limit.json")


def worst_wearer(pts, model):
    worst, who, seg = 1e9, None, None
    for si, (a, b) in enumerate(zip(pts, pts[1:])):
        for k in range(MG.SAMPLES + 1):
            t = k / float(MG.SAMPLES)
            p = a + (b - a) * t
            for name, kind, prm, ctr, rpy in model:
                d = MG.dist_point(list(p), kind, prm, ctr, rpy) - MG.TUBE_R
                if d < worst:
                    worst, who = d, name
                    seg = "%s -> %s" % (MG.CHAIN[si], MG.CHAIN[si + 1])
    return worst, who, seg


def best_branch(node, fkc, arm, obj, q, pad_mid, cand, model, k):
    """Solve the grasp pose k times; keep the BEST clearance any branch got."""
    w = GF.wrist_for(obj, q, pad_mid)
    w2 = cand.to_target(arm, w)
    qm = as_msg(q)
    best, who, seg, solved = -1e9, None, None, 0
    for _ in range(k):
        j = node.solve_arm_joints(arm, list(w2), qm, avoid=False, tries=4)
        if j is None:
            continue
        solved += 1
        pts = chain_points(fkc, node, arm, j)
        if pts is None:
            continue
        pts = [cand.to_world(arm, p) for p in pts]
        c, kk, s = worst_wearer(pts, model)
        if c > best:
            best, who, seg = c, kk, s
    if solved == 0:
        return dict(solved=0, best=None, wearer=None, link=None)
    return dict(solved=solved, best=round(best, 4), wearer=who, link=seg)


def probe(node, fkc, pad_mid, name, top, near_y, cand, q_of, model, k, floor,
          xs, log):
    row = dict(name=name, top=top, near_y=near_y, mount=cand.as_dict())
    z = round(top + CUBE / 2.0, 4)
    y = round(near_y + CUBE / 2.0, 4)
    worst_of_arms = 1e9
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        per = {}
        armbest = -1e9
        for x in xs:
            r = best_branch(node, fkc, arm, [round(sgn * x, 4), y, z],
                            q_of(arm), pad_mid[arm], cand, model, k)
            per["%.3f" % x] = r
            if r["best"] is not None and r["best"] > armbest:
                armbest = r["best"]
        row[arm] = per
        row["%s_best" % arm] = None if armbest <= -1e9 else round(armbest, 4)
        worst_of_arms = min(worst_of_arms, armbest)
    row["both_clear_floor"] = worst_of_arms >= floor
    lb, rb = row["left_best"], row["right_best"]
    # name the limiter at the centreline itself, which is what the levers move
    lc = row["left"]["0.000"]
    rc = row["right"]["0.000"]
    log("  %-46s  L %-8s %-26s  R %-8s %-26s  %s"
        % (name,
           "no IK" if lb is None else "%+.4f" % lb,
           "%s / %s" % (lc["wearer"], (lc["link"] or "")[:14]),
           "no IK" if rb is None else "%+.4f" % rb,
           "%s / %s" % (rc["wearer"], (rc["link"] or "")[:14]),
           "CLEARS" if row["both_clear_floor"] else ""))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--k", type=int, default=40)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--xs", default="0.00,0.05,0.10")
    ap.add_argument("--levers", action="store_true")
    ap.add_argument("--approach-grid", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    xs = [float(v) for v in a.xs.split(",")]

    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)."
                  % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    rig.set_furniture(False)          # collisions are OFF; the wearer is
    fk = FK(n)                        # geometric, and the slab is not the
    fkc = fk.cli                      # question this file asks
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    DOWN = WP.wearer_model("down")
    NONE = WP.wearer_model("none")
    ident = Cand("as built")
    t1q = lambda arm: T1.APPROACH[arm]              # noqa: E731

    # ------------------------------------------------------------ controls
    ctl = {}
    ql = as_msg(T1.APPROACH["left"])
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22], ql, avoid=False,
                            tries=8)
    if jt is None:
        log("REFUSING: the torso control pose does not solve at all.")
        return 6
    pts = chain_points(fkc, n, "left", jt)
    c, who, seg = worst_wearer([np.asarray(p) for p in pts], DOWN)
    ctl["inside_torso_negative"] = "%.4f / %s / %s" % (c, who, seg)
    ok = c < 0 and who and seg
    ctl["far_1.6m_unreachable"] = n.solve_arm_joints(
        "left", [1.60, 0.35, 1.15], ql, avoid=False, tries=6) is None
    ok = ok and ctl["far_1.6m_unreachable"]
    # the as-built centre must still be NEGATIVE on the best of k branches --
    # the column sweeps call it COLLISION:WEARER, which is the same statement
    built = probe(n, fkc, pad_mid, "control: as built, best of %d" % a.k,
                  0.950, 0.280, ident, t1q, DOWN, a.k, a.floor, [0.0],
                  lambda _s: None)
    ctl["as_built_centre_best_branch"] = built["left_best"]
    ok = ok and built["left_best"] is not None and built["left_best"] < 0.0
    # THE POSTURE SWAP MUST REACH THE MEASUREMENT, and this control is
    # CONSTRUCTED rather than argued. The first version asserted that deleting
    # the wearer's arms changes the best branch at the centre, which is a
    # CLAIM ABOUT THE ANSWER and not about the instrument: it duly failed,
    # because at that distance the best branch is limited by the torso and the
    # arms are not the binding body at all. A control cannot be allowed to
    # encode the result it is checking.
    #
    # So: a segment laid along the wearer's own left forearm -- shoulder
    # (0.21, 0, 1.43), hanging straight down, forearm centre near
    # (0.21, 0, 1.00) -- must read NEGATIVE under `down` and must read the
    # TORSO, further away, under `none`. Ground truth is the geometry, not a
    # solver.
    seg_pts = [np.array([0.21, 0.0, 1.05]), np.array([0.21, 0.0, 0.95])]
    cd, wd, _ = worst_wearer(seg_pts, DOWN)
    cn, wn, _ = worst_wearer(seg_pts, NONE)
    ctl["posture_swap_reaches_the_model"] = (
        "on the wearer's own forearm: down %.4f (%s) -> none %.4f (%s)"
        % (cd, wd, cn, wn))
    ok = ok and cd < 0.0 and "forearm" in (wd or "") and cn > cd + 0.05 \
        and (wn or "").startswith("torso")
    log("CONTROLS")
    for k2, v in ctl.items():
        log("   %-34s %s" % (k2, v))
    if not ok:
        log("\nREFUSING: a control failed.")
        return 6
    if a.self_test:
        log("\nself-test only.")
        return 0

    res = dict(tag=a.tag, k=a.k, floor=a.floor, xs=xs, controls=ctl, rows=[])
    log("\n" + "=" * 132)
    log("BEST CLEARANCE ANY BRANCH ACHIEVES AT |x| <= %.2f, floor %.3f m "
        "(best of %d solves, collisions OFF)" % (max(xs), a.floor, a.k))
    log("=" * 132)

    if a.approach_grid:
        # THE APPROACH IS A FREE VARIABLE UNDER MODE 06 and it costs nothing:
        # `run_abc.set_orient()` sends a per-task orientation and rebuilds the
        # finger-pad offset from it. Heading has been swept; ELEVATION never
        # has, and the two are not independent -- a hand that points further
        # down puts its wrist further back over the table, which is the same
        # 70 mm of trail the heading is being used to remove.
        cases = []
        for top, e in ((1.15, 0.380), (1.25, 0.430)):
            for elev in (-30.0, -20.0, -10.0, 0.0, 10.0):
                for h in (50.0, 65.0, 75.0):
                    def qh(arm, h=h, elev=elev):
                        s2 = -1.0 if arm == "left" else 1.0
                        return GF.q_from_axis(GF.axis_for(elev, s2 * h), 0.0)
                    cases.append(("elev %+.0f, head %+.0f, top %.2f near %.3f"
                                  % (elev, h, top, e), top, e, ident, qh,
                                  DOWN))
        for name, top, e, cand, qof, model in cases:
            res["rows"].append(probe(n, fkc, pad_mid, name, top, e, cand, qof,
                                     model, a.k, a.floor, xs, log))

    if a.levers:
        cases = []
        # (a) standing distance, on the as-built surface
        for e in (0.280, 0.330, 0.380, 0.430, 0.480):
            cases.append(("(a) near edge %.3f" % e, 0.950, e, ident, t1q,
                          DOWN))
        # table height, crossed with the best distance
        for t in (1.05, 1.15, 1.25):
            cases.append(("(a+h) top %.2f, near 0.380" % t, t, 0.380, ident,
                          t1q, DOWN))
        # (d) the mount, at the as-built table and at the best one
        mounts = [Cand("outboard +0.15", dx=0.15),
                  Cand("outboard +0.30", dx=0.30),
                  Cand("up +0.15", dz=0.15),
                  Cand("up +0.30", dz=0.30),
                  Cand("outboard +0.15, up +0.15", dx=0.15, dz=0.15),
                  Cand("outboard +0.30, up +0.30", dx=0.30, dz=0.30),
                  Cand("tilt -45", tilt=-45.0),
                  Cand("yaw inboard +30", yaw_in=30.0),
                  Cand("yaw inboard -30", yaw_in=-30.0),
                  Cand("forward +0.15", dy=0.15)]
        for m in mounts:
            cases.append(("(d) %s, table as built" % m.name, 0.950, 0.280, m,
                          t1q, DOWN))
        for m in mounts:
            cases.append(("(e) %s, top 1.15 near 0.380" % m.name, 1.15, 0.380,
                          m, t1q, DOWN))
        # the approach heading, free under mode 06, at the best table
        for h in (30.0, 50.0, 65.0, 80.0, 90.0):
            def qh(arm, h=h):
                s = -1.0 if arm == "left" else 1.0
                return GF.q_from_axis(GF.axis_for(T1.APPROACH_ELEV_DEG,
                                                  s * h), 0.0)
            cases.append(("(f) heading %+.0f inboard, top 1.15 near 0.380"
                          % h, 1.15, 0.380, ident, qh, DOWN))
        # THE COMBINATIONS. The single levers above name two that move the
        # number by more than the +/-5 mm the branch sampling carries -- the
        # mount's YAW and the approach HEADING -- so they are crossed here,
        # with and without an outboard shift, at the two best tables.
        for top, e in ((1.15, 0.380), (1.15, 0.430), (1.25, 0.430)):
            for yaw in (0.0, -15.0, -30.0, -45.0):
                for dx in (0.0, 0.15):
                    for h in (65.0, 80.0):
                        def qh(arm, h=h):
                            s2 = -1.0 if arm == "left" else 1.0
                            return GF.q_from_axis(
                                GF.axis_for(T1.APPROACH_ELEV_DEG, s2 * h), 0.0)
                        cases.append((
                            "(e) yaw %+.0f, out %+.2f, head %+.0f, top %.2f "
                            "near %.3f" % (yaw, dx, h, top, e),
                            top, e, Cand("yaw%+.0f/out%+.2f" % (yaw, dx),
                                         dx=dx, yaw_in=yaw), qh, DOWN))
        # the limit: no wearer arms at all, at the best table and best mount
        cases.append(("LIMIT: wearer arms deleted, top 1.15 near 0.380",
                      1.15, 0.380, ident, t1q, NONE))
        cases.append(("LIMIT: arms deleted + outboard 0.30 + up 0.30",
                      1.15, 0.380, Cand("out+up", dx=0.30, dz=0.30), t1q,
                      NONE))
        for name, top, e, cand, qof, model in cases:
            res["rows"].append(probe(n, fkc, pad_mid, name, top, e, cand, qof,
                                     model, a.k, a.floor, xs, log))

    res["log"] = lines
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    log("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
