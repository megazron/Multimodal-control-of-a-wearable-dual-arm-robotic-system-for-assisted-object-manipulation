#!/usr/bin/env python3
"""Generate and VERIFY four graded T2 scenarios, then merge them into
scenarios_verified.yaml.

T2 is "hold and fill": one arm HOLDS an open-top container by a side handle,
the other arm PICKS a block and RELEASES it into the opening.

THE OBJECT IS WHY THIS IS POSSIBLE AT ALL. The earlier BLOCKED verdict assumed
the release point sits directly above the hold point. Sweeping the real design
space found feasible pairs ~200 mm apart horizontally:

    left holds  (+0.10, 0.35, 1.15)     right releases (-0.10, 0.35, 1.20)

so the container is a box on a SIDE HANDLE -- like a dustpan or a saucepan --
whose opening centre is ~200 mm from the grasp and ~50 mm above it. The
holding gripper is therefore well clear of the opening, and that clearance is
exactly what accommodates the two arms' separation.

WHAT IS VERIFIED. Not just the endpoints. Every scenario carries:
  * the HOLD pose, checked as a static hold for the holding arm;
  * the full FILL PATH for the working arm -- pick, lift, transit, release,
    retreat -- with the holding arm required to be simultaneously solvable at
    the hold pose at EVERY waypoint.
A scenario whose endpoints solve but whose transit does not is exactly the
kind that fails on the day and wastes a participant.
"""
import math, os, sys
import numpy as np, rclpy, yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver

BIM = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src/srl_experiments/experiments/bimanual")
Y = 0.35
# RE-DERIVED 2026-08-08 with both arms VERIFIED at home (0.0000 rad offset).
# The protocol's pair -- "left holds (+0.10, 0.35, 1.15), right releases
# (-0.10, 0.35, 1.20)" -- is WRONG in two ways: the arm assignment is
# swapped, and NOBODY reaches (+0.10, 0.35, 1.15). That figure came from a
# sweep taken with the arms displaced from home, which is the same
# measured-from-the-wrong-pose error that once invalidated a reachability run.
#
# Measured reachable spans at y = 0.35:   left  x  0.10 .. 0.55
#                                          right x -0.55 .. -0.10
# x = +/-0.10 is the MARGIN of each arm's span and is not usable: repeated
# probing gives 0/5 to 4/5 depending on height. x = +/-0.15 is 5/5 at every
# height tested. A single IK call passes at the margin and a repeated one
# fails, which is precisely how a scenario gets written down that then fails
# on the day.
HOLD = [-0.15, Y, 1.10]         # RIGHT arm holds the handle
OPEN = [0.15, Y, 1.20]          # opening centre: 300 mm across, 100 mm up


def fill_path(pick, release, lift=0.06, retreat=0.06):
    """pick -> lift -> transit -> above release -> release -> retreat."""
    p, r = list(pick), list(release)
    return [p,
            [p[0], p[1], p[2] + lift],
            [(p[0] + r[0]) / 2.0, p[1], max(p[2], r[2]) + lift],
            [r[0], r[1], r[2] + lift],
            r,
            [r[0], r[1], r[2] + retreat]]


def build():
    """Four graded scenarios. Difficulty comes from the TRANSIT and the
    tolerance, as it does for T3/T6 -- not from how hard a block is to grip."""
    return {
      "S1_short_reach": dict(
        hold=HOLD, hold_arm="right", fill_arm="left",
        pick=[0.30, Y, 1.15], release=OPEN, block_mm=40, tol_mm=40,
        why="pick close to the opening, generous release tolerance"),
      "S2_long_reach": dict(
        hold=HOLD, hold_arm="right", fill_arm="left",
        pick=[0.45, Y, 1.15], release=OPEN, block_mm=40, tol_mm=35,
        why="pick further out and lower, so the transit is longer"),
      "S3_height_change": dict(
        hold=[-0.15, Y, 1.20], hold_arm="right", fill_arm="left",
        pick=[0.35, Y, 1.15], release=[0.15, Y, 1.30], block_mm=40,
        tol_mm=35,
        why="container held 100 mm higher; the fill path must climb"),
      "S4_tight_tolerance": dict(
        hold=HOLD, hold_arm="right", fill_arm="left",
        pick=[0.35, Y, 1.25], release=OPEN, block_mm=26, tol_mm=18,
        why="26 mm block into the same opening: half the placement margin"),
    }


def main():
    rclpy.init()
    n = Solver(); n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim"); return 2
    q = {a: n.ee_quat(a) for a in ("left", "right")}

    def ok(arm, p, k=2):
        return all(n.solve(arm, p, q[arm], tries=6) for _ in range(k))

    print("T2 SCENARIO VERIFICATION")
    print("container: opening %.0f mm across / %.0f mm above the handle\n"
          % (1000 * abs(OPEN[0] - HOLD[0]), 1000 * (OPEN[2] - HOLD[2])))
    out = {}
    for name, s in build().items():
        ha, fa = s["hold_arm"], s["fill_arm"]
        path = fill_path(s["pick"], s["release"])
        hold_ok = ok(ha, s["hold"])
        bad = []
        for w in path:
            # the working arm must reach it AND the holding arm must still be
            # able to hold, simultaneously
            if not ok(fa, w) or not ok(ha, s["hold"], k=1):
                bad.append([round(v, 3) for v in w])
        good = hold_ok and not bad
        out[name] = dict(
            **{k: v for k, v in s.items() if k != "path"},
            fill_path=[[round(float(v), 3) for v in w] for w in path],
            verified=bool(good),
            reason="" if good else (
                "hold pose unreachable" if not hold_ok
                else "unreachable fill waypoints: %s" % bad))
        print("  %-20s %-9s hold %-9s path %d/%d  %s"
              % (name, "VERIFIED" if good else "FAILED",
                 "ok" if hold_ok else "FAIL",
                 len(path) - len(bad), len(path), s["why"]))
        if bad:
            print("        unreachable: %s" % bad)

    p = os.path.join(BIM, "scenarios_verified.yaml")
    spec = yaml.safe_load(open(p))
    spec.setdefault("tasks", {})["T2_hold_fill"] = out
    yaml.safe_dump(spec, open(p, "w"), sort_keys=False)
    v = sum(1 for s in out.values() if s["verified"])
    print("\n  %d of %d verified -> merged into %s" % (v, len(out), p))
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
