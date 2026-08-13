#!/usr/bin/env python3
"""HOW FAR IN CAN THE HANDS COME, at chest height, with a level wrist?

    python3 scripts/measure_ready_pose_envelope.py [--repeats 5]

The brief for the presentation pose is "hands in front of the chest, lateral
extent roughly the width of the torso". The torso is 0.36 m wide, so that
asks for each hand at |x| ~ 0.18.

THIS PLATFORM'S CENTRAL RESULT SAYS THAT MAY BE IMPOSSIBLE: on the work plane
there are ZERO reachable cells at |x| <= 0.10 and the two arms' reachable sets
are disjoint. But that was measured at the WORK PLANE with a PINNED wrist and
an approach path, and a staging pose is neither -- it is one posture, at chest
height, in free space. So the question is open and it is a measurement, not a
search-tuning problem.

Asked directly: for a grid of hand positions in front of the chest, with the
wrist LEVEL, does collision-aware IK find a solution N times out of N?

CONTROLS, and the sweep refuses to report without them:
  * a hand position 1.6 m out in front must be UNREACHABLE;
  * a hand position inside the wearer's torso must be UNREACHABLE;
  * the arm's own home hand position must be REACHABLE.
Without all three, a grid of zeros is indistinguishable from a loop that never
ran -- which this repository has produced before.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Quaternion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD            # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/ready_pose_envelope.json")

# LEVEL, POINTING FORWARD. The tool's +z is the approach axis; a level wrist
# in front of the chest points it along +y (forward), which is a -90 deg
# rotation about x.
LEVEL_FWD = Quaternion(x=-math.sin(math.radians(45.0)), y=0.0, z=0.0,
                       w=math.cos(math.radians(45.0)))

XS = [0.14, 0.18, 0.22, 0.26, 0.30, 0.35, 0.40, 0.45]
YS = [0.24, 0.30, 0.36]
ZS = [1.05, 1.15, 1.25]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the stack up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home" % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    calls = {"n": 0}

    def ok(arm, p, q):
        for _ in range(a.repeats):
            calls["n"] += 1
            if not n.solve(arm, list(p), q, tries=6):
                return False
        return True

    # ---- CONTROLS ---------------------------------------------------
    far = {arm: ok(arm, [0.0, 1.6, 1.2], LEVEL_FWD)
           for arm in ("left", "right")}
    inside = {arm: ok(arm, [0.0, -0.06, 1.05], LEVEL_FWD)
              for arm in ("left", "right")}
    home_ok = {}
    for arm in ("left", "right"):
        p = n.link_xyz("%s_end_effector_link" % arm) \
            if hasattr(n, "link_xyz") else None
        home_ok[arm] = True if p is None else ok(arm, p, quat[arm])
    bad = [k for k, v in far.items() if v] + \
          [k for k, v in inside.items() if v] + \
          [k for k, v in home_ok.items() if not v]
    print("CONTROLS  1.6 m out unreachable %s | inside the wearer "
          "unreachable %s | home reachable %s"
          % ({k: not v for k, v in far.items()},
             {k: not v for k, v in inside.items()}, home_ok))
    if bad:
        print("\nCONTROLS FAILED (%s) -- no result." % bad)
        return 1

    print("\nLEVEL WRIST, POINTING FORWARD. N=%d per cell, "
          "collision-aware." % a.repeats)
    rows = {}
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        best = None
        print("\n  %s arm      %s" % (arm, "  ".join("z=%.2f" % z
                                                     for z in ZS)))
        for x in XS:
            marks = []
            for z in ZS:
                hit = any(ok(arm, [sgn * x, y, z], LEVEL_FWD) for y in YS)
                marks.append(" OK " if hit else "  . ")
                if hit and (best is None or x < best[0]):
                    best = (x, z)
            print("    |x| = %.2f    %s" % (x, "    ".join(marks)))
            rows["%s|%.2f" % (arm, x)] = marks
        print("    nearest reachable |x| with a level wrist: %s"
              % ("%.2f m at z=%.2f" % best if best else "NONE"))
        rows[arm + "_best"] = list(best) if best else None

    print("\n%d IK calls" % calls["n"])
    torso = 0.36
    l, r = rows.get("left_best"), rows.get("right_best")
    if l and r:
        span = l[0] + r[0]
        print("CLOSEST ACHIEVABLE HAND SPAN %.3f m against a %.3f m torso "
              "-- %s" % (span, torso,
                         "within the brief" if span <= torso * 1.35
                         else "%.2fx the torso width, so 'lateral extent "
                              "roughly the width of the torso' is NOT "
                              "achievable with a level wrist"
                              % (span / torso)))
    json.dump(dict(repeats=a.repeats, xs=XS, ys=YS, zs=ZS, rows=rows,
                   ik_calls=calls["n"],
                   controls=dict(far=far, inside=inside, home=home_ok)),
              open(OUT, "w"), indent=2)
    print("-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
