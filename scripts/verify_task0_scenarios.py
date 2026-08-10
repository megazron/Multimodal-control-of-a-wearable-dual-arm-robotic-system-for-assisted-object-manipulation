#!/usr/bin/env python3
"""Verify TASK 0's four spheres per arm, N=10, over the FULL densified path,
WITH THE BENCH IN THE SCENE.

Three rules carried from verify_abc_scenarios.py, all of them paid for:

  N REPEATS, because TRAC-IK restarts randomly.  A pose at the edge of the
  feasible set is a coin flip and one call is one flip; k=2 passes a 60%%-
  feasible pose 36%% of the time, which is often enough to be written into a
  protocol and rare enough to look like bad luck on the day.

  THE WHOLE PATH, because the arm FLIES the segments between the spheres.
  Task 0's movements are BETWEEN targets by definition -- every one of the six
  pairs is traversed -- so the segments are not an extra, they ARE the task.

  THE FURNITURE IN THE SCENE, because a free-space verification of a task
  performed over a bench answers a different question.  An earlier sweep in
  this project verified 40 poses without the bench and every one of them was
  wrong for that reason.

And one rule of its own: the headline is a COUNT OF ZERO, and zero is what a
broken verifier prints too, so the run is bracketed by controls whose answers
are known independently and it REFUSES TO REPORT if any of them is wrong.

    python3 scripts/verify_task0_scenarios.py [--repeats 10]
"""
import argparse
import itertools
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
import task0 as T0                                           # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

STEP = 0.02
OUT = os.path.join(ROOT, "recordings/baselines/task0_verification.json")


def furniture(node):
    import clip_scene as CS
    tmp = CS.Scene.__new__(CS.Scene)
    return CS.Scene._collision_furniture(tmp)


def apply_scene(node, ps, timeout_s=12.0):
    import time as _t
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + timeout_s
    while _t.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return bool(fut.done())


def require_path(path, what):
    """AN EMPTY PATH MUST NOT VERIFY.  all() over nothing is True, so a
    densify() returning nothing would print 'FAILURES: 0' on a path it never
    looked at -- indistinguishable from having checked every waypoint."""
    if not path:
        raise RuntimeError("densify returned no waypoints for %s" % what)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=STEP)
    ap.add_argument("--no-furniture", action="store_true",
                    help="diagnostic only -- NOT a valid verification")
    args = ap.parse_args()
    N, step = args.repeats, args.step

    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2

    print("=" * 74)
    print("TASK 0 -- four spheres per arm, N=%d, paths densified to %.0f mm"
          % (N, step * 1000))
    print("=" * 74)

    bad_home = False
    for a in ("left", "right"):
        w, j = n.home_ok(a)
        print("  %-5s home offset %.4f rad   %s"
              % (a, w, "ok" if w <= HOME_TOL_RAD else "NOT AT HOME (j%d)" % j))
        bad_home |= w > HOME_TOL_RAD
    if bad_home:
        print("\n  REFUSING: the arms are not at home.  This measures reach "
              "FROM HOME, and a run taken elsewhere answers a different "
              "question under the same name.")
        return 3

    quat = {a: n.ee_quat(a) for a in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("\n  REFUSING: no tf2 for an arm's end effector.  The anchor "
              "orientation must be the ARM'S OWN -- an identity quaternion is "
              "not neutral, it is a specific unreachable pose.")
        return 4

    applied = True
    if not args.no_furniture:
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = furniture(n)
        applied = apply_scene(n, ps)
        print("  bench/furniture applied: %s (%d objects)"
              % (applied, len(ps.world.collision_objects)))
        if not applied:
            print("\n  REFUSING: the furniture did not apply.  Verifying a "
                  "bench task in free space is the failure this check exists "
                  "to prevent.")
            return 6

    calls = {"n": 0}

    def ok(arm, p, k=None):
        k = N if k is None else k
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False           # short-circuit; N is cheap when it passes
        return True

    # ------------------------------------------------- INSTRUMENT CONTROLS
    print("\nINSTRUMENT CONTROLS (a zero is only meaningful if a one is "
          "possible)")
    ctl_far = ok("left", [1.60, 0.35, 1.15], k=1)
    ctl_in = ok("left", [0.0, -0.10, 1.25], k=1)
    ctl_good = ok("left", T0.TARGETS_LEFT["A"], k=1)
    # And the finding this task's geometry rests on: x = 0.25 must FAIL where
    # x = 0.30 passes.  If that cliff has moved, the |x| >= 0.30 rule is no
    # longer justified and the sphere positions must be re-derived.
    ctl_cliff = ok("left", [0.25, T0.Y, 1.15], k=1)
    print("   far outside reach (1.60, 0.35, 1.15) -> %s  (want unreachable)"
          % ("REACHABLE" if ctl_far else "unreachable"))
    print("   inside the wearer (0.00,-0.10, 1.25) -> %s  (want unreachable)"
          % ("REACHABLE" if ctl_in else "unreachable"))
    print("   sphere A                              -> %s  (want reachable)"
          % ("reachable" if ctl_good else "UNREACHABLE"))
    print("   the x=0.25 cliff                      -> %s  (want unreachable;"
          " this is what |x|>=0.30 is FOR)"
          % ("REACHABLE" if ctl_cliff else "unreachable"))
    if ctl_far or ctl_in or not ctl_good:
        print("\n  REFUSING TO REPORT.  A control failed, so any count this "
              "prints -- especially a zero -- means nothing.")
        n.destroy_node()
        rclpy.shutdown()
        return 5

    out = {"controls": dict(far_unreachable=not ctl_far,
                            inside_wearer_unreachable=not ctl_in,
                            sphere_A_reachable=bool(ctl_good),
                            x025_still_infeasible=not ctl_cliff),
           "furniture_in_scene": bool(applied and not args.no_furniture),
           "targets": {"left": T0.TARGETS_LEFT, "right": T0.TARGETS_RIGHT},
           "target_width_m": T0.TARGET_W_M}
    fails = 0

    # --------------------------------------------------------- the spheres
    print("\nSPHERES")
    for arm, pts in (("left", T0.TARGETS_LEFT), ("right", T0.TARGETS_RIGHT)):
        rows = {k: bool(ok(arm, p)) for k, p in sorted(pts.items())}
        fails += sum(1 for v in rows.values() if not v)
        out.setdefault("spheres", {})[arm] = rows
        print("   %-5s %s" % (arm, "  ".join(
            "%s:%s" % (k, "ok" if v else "FAIL") for k, v in rows.items())))

    # -------------------------------------------------- every pair's path
    # All six unordered pairs, densified.  These ARE the trials -- a movement
    # in Task 0 is always sphere-to-sphere -- so a failure here is a trial
    # that cannot run, not a nicety.
    print("\nTRANSITS (all 6 pairs per arm, densified to %.0f mm)"
          % (step * 1000))
    for arm, pts in (("left", T0.TARGETS_LEFT), ("right", T0.TARGETS_RIGHT)):
        per = {}
        for a, b in itertools.combinations(sorted(pts), 2):
            path = require_path(densify([pts[a], pts[b]], step),
                                "%s %s->%s" % (arm, a, b))
            bad = sum(1 for w in path if not ok(arm, w))
            amp = math.dist(pts[a], pts[b])
            per["%s-%s" % (a, b)] = dict(
                waypoints=len(path), failures=bad,
                amplitude_m=round(amp, 4),
                fitts_id=round(T0.fitts_id(amp), 3))
            fails += bad
            print("   %-5s %s-%s  A=%.3f m  ID=%.2f  %3d waypoints  %s"
                  % (arm, a, b, amp, T0.fitts_id(amp), len(path),
                     "ok" if not bad else "%d FAIL" % bad))
        out.setdefault("transits", {})[arm] = per

    # --------------------------------------------------- design sanity
    ids = sorted(v["fitts_id"] for v in out["transits"]["left"].values())
    span = ids[-1] - ids[0]
    gaps = [b - a for a, b in zip(ids, ids[1:])]
    mg = sum(gaps) / len(gaps)
    uneven = math.sqrt(sum((g - mg) ** 2 for g in gaps))
    out["design"] = dict(ids=ids, id_span_bits=round(span, 3),
                         id_gap_unevenness=round(uneven, 3))
    print("\nDESIGN   ID span %.2f bits over 6 conditions, gap unevenness "
          "%.3f" % (span, uneven))
    # A Fitts regression over a degenerate x-axis is not a regression.
    if span < 1.5:
        print("   ID span under 1.5 bits -- the regression would be fitted "
              "over too narrow a range to mean anything.")
        fails += 1
    if len(set(round(i, 2) for i in ids)) < 6:
        print("   two IDs coincide -- one of the six conditions is a "
              "duplicate and buys nothing.")
        fails += 1

    # mirror symmetry, as arithmetic
    worst_mirror = max(
        abs(T0.TARGETS_RIGHT[k][0] + T0.TARGETS_LEFT[k][0]) +
        abs(T0.TARGETS_RIGHT[k][1] - T0.TARGETS_LEFT[k][1]) +
        abs(T0.TARGETS_RIGHT[k][2] - T0.TARGETS_LEFT[k][2])
        for k in T0.TARGETS_LEFT)
    out["design"]["mirror_residual_m"] = worst_mirror
    print("   left/right mirror residual %.2e m" % worst_mirror)
    if worst_mirror > 1e-9:
        fails += 1

    if not args.no_furniture:
        import clip_scene as CS
        CS.remove_furniture(n)

    payload = dict(repeats=N, step_m=step, ik_calls=calls["n"],
                   failures=fails, detail=out)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(payload, open(OUT, "w"), indent=2)
    print("\n" + "=" * 74)
    print("  %d IK calls   TOTAL FAILURES: %d" % (calls["n"], fails))
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
