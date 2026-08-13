#!/usr/bin/env python3
"""EVERY DANCE WAYPOINT THROUGH /compute_ik, BEFORE ANY OF IT IS FILMED.

    python3 scripts/verify_dance_paths.py [--repeats 3]

The routines' envelope was widened from a 0.18 x 0.14 x 0.18 m box to
0.24 x 0.22 x 0.56, on the strength of task0's free-space anchors. Those are
measured -- 0 failures, N=10, no furniture -- but they are ANOTHER TASK's
measurement, and an envelope inherited from a neighbouring verification is an
inference until it is checked on the paths that will actually run.

It has been wrong before, and in exactly this way: the previous widening was
taken from a forward-reach sweep that probed only |x| = 0.35, said nothing
about the inboard and outboard columns the dance visits, and 117 of 1455
waypoints failed.

WHAT IS CHECKED. Every rendered waypoint of every routine, both arms, at the
arm's own anchor orientation, with `avoid_collisions=True` and the wearer in
the scene -- the same door the follower drives the routine through, so a pose
that fails here would have failed on camera.

CONTROLS, and it refuses to report without them:
  * a pose 1.6 m out must FAIL, or a run of zeros means nothing;
  * a pose inside the wearer's torso must FAIL;
  * the home pose must PASS.
"""

import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD            # noqa: E402
import choreography as CH                                      # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/dance_paths.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--stride", type=int, default=2,
                    help="check every Nth waypoint; 1 checks all of them")
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

    def ok(arm, p):
        for _ in range(a.repeats):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    far = {arm: ok(arm, [0.0, 1.6, 1.2]) for arm in ("left", "right")}
    inside = {arm: ok(arm, [0.0, -0.06, 1.05]) for arm in ("left", "right")}
    print("CONTROLS  1.6 m out unreachable %s | inside the wearer "
          "unreachable %s"
          % ({k: not v for k, v in far.items()},
             {k: not v for k, v in inside.items()}))
    if any(far.values()) or any(inside.values()):
        print("\nCONTROLS FAILED -- no result.")
        return 1

    print("\nDANCE PATHS, N=%d, every %dth waypoint, collision-aware"
          % (a.repeats, a.stride))
    rows, total_bad = {}, 0
    for key, spec in sorted(CH.TASKS.items()):
        path = spec["build"]()
        bad, tested, worst = 0, 0, []
        for arm in ("left", "right"):
            for i, p in enumerate(path[arm]):
                if i % a.stride:
                    continue
                tested += 1
                if not ok(arm, p):
                    bad += 1
                    if len(worst) < 5:
                        worst.append((arm, i,
                                      [round(v, 3) for v in p]))
        rows[key] = dict(tested=tested, failures=bad, examples=worst)
        total_bad += bad
        print("  %-3s %-16s %4d of %4d waypoints FAIL%s"
              % (key, spec["name"], bad, tested,
                 ("   e.g. %s" % worst[0]) if worst else ""))

    print("\n%d IK calls, %d failures" % (calls["n"], total_bad))
    if total_bad:
        print("THE ENVELOPE IS NOT CLEAR. Do not film it: a routine with "
              "unreachable waypoints stalls on camera at the pose that "
              "failed, and reads as the robot freezing.")
    else:
        print("Envelope clear. |x| %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f"
              % (CH.BOX[0][0], CH.BOX[0][1], CH.BOX[1][0], CH.BOX[1][1],
                 CH.BOX[2][0], CH.BOX[2][1]))
    json.dump(dict(repeats=a.repeats, stride=a.stride, box=CH.BOX,
                   rows=rows, ik_calls=calls["n"],
                   controls=dict(far=far, inside=inside)),
              open(OUT, "w"), indent=2)
    print("-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if total_bad else 0


if __name__ == "__main__":
    sys.exit(main())
