#!/usr/bin/env python3
"""Choose T0's THREE DISTINCT REACHING DIRECTIONS per arm, by measurement.

    python3 scripts/choose_t0_directions.py [--repeats 10]

WHY THIS EXISTS
---------------
T0's targets were drawn from a single 200 x 80 mm band (`task0.BAND`,
x 0.30..0.50, y 0.35, z 1.36..1.44).  That band is correct for what it was
derived for -- it is the largest rectangle BOTH ARMS can work with THE BENCH
IN THE SCENE -- and it has two consequences nobody wanted:

  * three targets inside 200 x 80 mm are not three directions.  Sampled at the
    clip seed they land within 0.16 m of each other, so the clip shows an arm
    twitching between three points a hand's breadth apart.
  * the band is bench-constrained, and T0 HAS NO BENCH.  T0 is reaching only:
    no objects, no grasp, nothing to rest on a surface.  Carrying the bench
    into it imports another task's furniture and, with it, another task's
    limits.

So the region is re-derived here with T0's OWN scene -- free space -- and
split into the three directions the task is actually about:

    FRONT-UP     in front of the wearer, ABOVE the shoulder line (z > 1.46,
                 the top of the torso box in human_backpack.xacro)
    FRONT-OUT    directly in front at CHEST height, as far from the body as
                 the arm reaches
    FRONT-DOWN   in front, BELOW chest height

WHAT "MEASURED" MEANS HERE, and it is this project's own rule
-------------------------------------------------------------
  * N repeats per pose (TRAC-IK restarts randomly; one call is one coin flip);
  * the WHOLE PATH, densified, not just the endpoints -- the arm flies the
    gaps;
  * MARGIN, not just N: a chosen cell must have all six +/-20 mm neighbours
    feasible too, because feasibility on this rig falls off a cliff rather
    than degrading, and N only locates the edge.

Nothing here decides anything about the other three tasks: it writes
recordings/baselines/t0_directions.json and prints the block to paste into
task0.py.
"""
import argparse
import json
import math
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t0_directions.json")

# The wearer, from human_backpack.xacro: torso box 0.36 x 0.22 x 0.48 centred
# at (0, 0, 1.22), so the chest front face is y = +0.11 and the shoulder line
# is z = 1.46.  Every region below is stated against those two numbers rather
# than against a round figure, so a change to the wearer model shows up here.
CHEST_FRONT_Y = 0.11
SHOULDER_Z = 1.46
CHEST_Z = 1.22

# The search grid.  Coarse enough to sweep in minutes, fine enough that the
# 20 mm margin test is a real test.
STEP = 0.04
MARGIN_M = 0.02

REGIONS = {
    # name          x range        y range        z range          score
    "FRONT_UP":   dict(x=(0.24, 0.56), y=(0.20, 0.50),
                       z=(SHOULDER_Z, 1.62),
                       score=lambda p: (p[2], p[1])),
    "FRONT_OUT":  dict(x=(0.24, 0.56), y=(0.30, 0.66),
                       z=(1.14, CHEST_Z + 0.06),
                       score=lambda p: (p[1], -abs(p[0]))),
    "FRONT_DOWN": dict(x=(0.24, 0.56), y=(0.20, 0.50),
                       z=(0.92, 1.12),
                       score=lambda p: (-p[2], p[1])),
}
ORDER = ("FRONT_UP", "FRONT_OUT", "FRONT_DOWN")
# Two targets closer than this are not two directions.  0.25 m is more than
# four times the 0.06 m separation the old single-band sampler used, and it is
# what "visibly far apart in the clip" has to mean at this scale.
MIN_SEP_M = 0.25


def frange(lo, hi, step):
    n = int(math.floor((hi - lo) / step + 1e-9)) + 1
    return [round(lo + i * step, 4) for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--coarse", type=int, default=3,
                    help="repeats during the grid sweep")
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home. A reachability sweep "
                  "from the wrong pose measures the wrong question." % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    # T0'S OWN SCENE IS EMPTY. Any furniture left loaded by an earlier run
    # would silently constrain this sweep exactly as the bench constrained the
    # band being replaced.
    import clip_scene as CS
    CS.remove_furniture(n)

    calls = {"n": 0}

    def ok(arm, p, k):
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    # ---------------------------------------------------------- CONTROLS
    ctl_far = ok("left", [1.60, 0.35, 1.15], 1)
    ctl_wearer = ok("left", [0.0, -0.10, 1.25], 1)
    ctl_good = ok("left", [0.35, 0.35, 1.40], 1)
    print("CONTROLS  1.6 m out -> %s | inside the wearer -> %s | "
          "a known-good T0 pose -> %s"
          % ("REACHABLE" if ctl_far else "unreachable",
             "REACHABLE" if ctl_wearer else "unreachable",
             "reachable" if ctl_good else "UNREACHABLE"))
    if ctl_far or ctl_wearer or not ctl_good:
        print("REFUSING TO REPORT: a control failed.")
        n.destroy_node()
        rclpy.shutdown()
        return 6

    out = {"regions": {k: {kk: vv for kk, vv in v.items() if kk != "score"}
                       for k, v in REGIONS.items()},
           "repeats": a.repeats, "margin_m": MARGIN_M,
           "min_sep_m": MIN_SEP_M, "furniture": "none -- T0 has no objects",
           "arms": {}}

    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        chosen, per = {}, {}
        for name in ORDER:
            R = REGIONS[name]
            cands = []
            for x in frange(*R["x"], STEP):
                for y in frange(*R["y"], STEP):
                    for z in frange(*R["z"], STEP):
                        p = [round(sgn * x, 4), y, z]
                        if not ok(arm, p, a.coarse):
                            continue
                        cands.append(p)
            # MARGIN: keep only cells whose six +/-20 mm neighbours also
            # solve. A cell that passes alone is on the cliff edge.
            solid = []
            for p in sorted(cands, key=R["score"], reverse=True):
                nb = [[p[0] + d, p[1], p[2]] for d in (MARGIN_M, -MARGIN_M)] + \
                     [[p[0], p[1] + d, p[2]] for d in (MARGIN_M, -MARGIN_M)] + \
                     [[p[0], p[1], p[2] + d] for d in (MARGIN_M, -MARGIN_M)]
                if all(ok(arm, q, a.coarse) for q in nb):
                    solid.append(p)
                if len(solid) >= 3:
                    break
            per[name] = dict(feasible_cells=len(cands),
                             with_margin=len(solid))
            if not solid:
                print("   %-5s %-11s NO CELL with %.0f mm margin (%d feasible)"
                      % (arm, name, MARGIN_M * 1000, len(cands)))
                continue
            chosen[name] = solid[0]
            print("   %-5s %-11s %d feasible, chose (%+.3f, %.3f, %.3f)"
                  % (arm, name, len(cands), *solid[0]))
        out["arms"][arm] = dict(regions=per, chosen=chosen)
        if len(chosen) < 3:
            continue
        # ------------------------------------------------ the triple
        pts = [chosen[k] for k in ORDER]
        seps = {"%s-%s" % (ORDER[i], ORDER[j]): round(math.dist(pts[i], pts[j]), 4)
                for i in range(3) for j in range(i + 1, 3)}
        out["arms"][arm]["separations_m"] = seps
        print("   %-5s separations: %s" % (arm, seps))
        # ------------------------------------------------ N=10, full path
        bad_pose = [k for k in ORDER if not ok(arm, chosen[k], a.repeats)]
        transit_bad, transit_n = 0, 0
        for i in range(3):
            for j in range(3):
                if i == j:
                    continue
                path = densify([pts[i], pts[j]], 0.02)
                transit_n += len(path)
                transit_bad += sum(1 for w in path if not ok(arm, w, a.repeats))
        out["arms"][arm].update(
            pose_failures=bad_pose, transit_waypoints=transit_n,
            transit_failures=transit_bad,
            min_separation_m=round(min(seps.values()), 4))
        print("   %-5s N=%d: %d pose failures, %d/%d transit failures"
              % (arm, a.repeats, len(bad_pose), transit_bad, transit_n))

    out["ik_calls"] = calls["n"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("\n%d IK calls -> %s" % (calls["n"], OUT))
    print("\nPASTE INTO task0.py:")
    for arm in ("left", "right"):
        c = out["arms"][arm].get("chosen", {})
        for k in ORDER:
            if k in c:
                print("    %-5s %-11s %r" % (arm, k, c[k]))
    n.destroy_node()
    rclpy.shutdown()
    bad = any(out["arms"][a2].get("pose_failures")
              or out["arms"][a2].get("transit_failures")
              or len(out["arms"][a2].get("chosen", {})) < 3
              or out["arms"][a2].get("min_separation_m", 0) < MIN_SEP_M
              for a2 in ("left", "right"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
