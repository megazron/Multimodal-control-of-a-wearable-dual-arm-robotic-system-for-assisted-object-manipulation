#!/usr/bin/env python3
"""Verify every coordinate of the five-task set at N repeats over the FULL path.

A pose that passes one IK call is not a reachable pose (docs/ENGINEERING_LOG.md standing
rule): TRAC-IK restarts randomly, so a pose at the edge of the feasible set is
a coin flip. And the arm FLIES the segments between declared waypoints, so
endpoints alone prove nothing. N=10 over paths densified to 20 mm.
"""
import json, math, os, sys
import numpy as np, rclpy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src/srl_experiments/experiments/final5"))
from verify_task_scenes import Solver, HOME_TOL_RAD
from audit_scenario_reachability import densify
import tasks as T

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N, STEP = 10, 0.02


def main():
    rclpy.init(); n = Solver(); n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik"); return 2
    # REFUSE unless the arms are at home. This measures reach FROM HOME, and a
    # run taken with the arms parked elsewhere answers a different question
    # under the same name -- which has already invalidated one reachability
    # run and half a front-reach run in this project.
    bad_home = False
    for a in ("left", "right"):
        w, j = n.home_ok(a)
        print("  %-5s home offset %.4f rad %s" % (a, w, "ok" if w <= HOME_TOL_RAD else "NOT AT HOME"))
        bad_home |= w > HOME_TOL_RAD
    if bad_home:
        print("\n  REFUSING: arms are not at home. Send them home and re-run.")
        return 3
    q = {a: n.ee_quat(a) for a in ("left", "right")}

    def ok(arm, p, k=N):
        return all(n.solve(arm, list(p), q[arm], tries=6) for _ in range(k))

    def pair(xc, y, z, sep):
        """Both grippers on the two ends. Left works +x -- the naming is
        viewer-perspective, so the assignment is tried both ways."""
        a = [xc - sep/2, y, z]; b = [xc + sep/2, y, z]
        return ((ok("left", b) and ok("right", a)) or
                (ok("left", a) and ok("right", b)))

    out, fails = {}, 0
    print("\nTASK 1 positioning")
    t1 = {}
    for arm, pts in T.TASK1["targets"].items():
        res = [ok(arm, p) for p in pts]
        t1[arm] = [dict(p=p, ok=bool(r)) for p, r in zip(pts, res)]
        print("   %-5s %d/%d targets" % (arm, sum(res), len(res)))
        fails += len(res) - sum(res)
    out["task1"] = t1

    print("TASK 2 pick and place (pick, standoff, and the approach between)")
    t2 = {}
    for arm, picks in T.TASK2["picks"].items():
        rows = []
        for p in picks:
            pre = [p[0], p[1], p[2] + T.TASK2["standoff_m"]]
            path = densify([pre, p], STEP)
            # AN EMPTY PATH MUST NOT VERIFY. all() over nothing is True, so a
            # densify() that returned no waypoints would make this report
            # "TOTAL FAILURES: 0" -- indistinguishable from having checked
            # every waypoint. That is the same mechanism that let a
            # known-answer check pass on zero parsed rows, sitting under the
            # result this project leans on hardest.
            if not path:
                raise RuntimeError(
                    "densify returned no waypoints for the approach to %s; "
                    "refusing to report a pass on an empty path" % (p,))
            good = all(ok(arm, w) for w in path)
            binp = T.TASK2["bins"][arm]
            place = densify([[binp[0], binp[1], binp[2] + 0.12], binp], STEP)
            if not place:
                raise RuntimeError(
                    "densify returned no waypoints for the place at %s"
                    % (binp,))
            goodb = all(ok(arm, w) for w in place)
            rows.append(dict(pick=p, approach_ok=bool(good), place_ok=bool(goodb),
                             n_way=len(path) + len(place)))
            fails += (not good) + (not goodb)
        t2[arm] = rows
        print("   %-5s %d/%d picks with a clear approach, %d/%d places"
              % (arm, sum(r["approach_ok"] for r in rows), len(rows),
                 sum(r["place_ok"] for r in rows), len(rows)))
    out["task2"] = t2

    print("TASK 3/4 coupled carry (both grippers, whole path)")
    t3 = {}
    for name, path in T.TASK3["paths"].items():
        dense = densify(path, STEP)
        bad = [w for w in dense if not pair(w[0], w[1], w[2], T.TRAY_SEP)]
        t3[name] = dict(waypoints=len(dense), bad=len(bad), ok=not bad)
        fails += len(bad)
        print("   %-16s %2d waypoints  %s" % (name, len(dense),
                                              "VERIFIED" if not bad else "%d FAIL" % len(bad)))
    out["task3_4"] = t3

    print("TASK 5 dual pursuit (the amplitude shell, 26 directions each arm)")
    t5 = {}
    import itertools
    dirs = [np.array(d, float) / np.linalg.norm(d)
            for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
    for arm, c in T.TASK5["centres"].items():
        c = np.asarray(c, float); amp = T.TASK5["amplitude_m"]
        bad = sum(1 for d in dirs if not ok(arm, list(c + amp * d)))
        t5[arm] = dict(centre=list(c), shell_fail=bad, ok=bad == 0)
        fails += bad
        print("   %-5s centre %s  %d/26 shell directions" % (arm, list(c), 26 - bad))
    out["task5"] = t5

    p = os.path.join(ROOT, "recordings/baselines/final5_verification.json")
    json.dump(dict(repeats=N, step_m=STEP, failures=fails, detail=out),
              open(p, "w"), indent=2)
    print("\n  TOTAL FAILURES: %d   -> %s" % (fails, p))
    n.destroy_node(); rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
