#!/usr/bin/env python3
"""Verify the FOUR MSc tasks -- T0, T1, T2, T3 -- N=10, full densified path,
bench in the scene, BOTH arm assignments.

This is the one verifier for the MSc set. It replaces nothing: the legacy
A/B/C set keeps verify_abc_scenarios.py, and the two report separately so a
failure in one cannot be mistaken for the other.

THE GUARDS, as code:
  * a sweep REFUSES when tested == 0 -- a zero from a loop that never ran is
    not a measurement;
  * a failed control means NO REPORT AT ALL, not a footnote;
  * one of the controls is a pose deliberately INSIDE the bench slab, which
    must come back unreachable. That is the check that would have caught the
    free-space bug on the day it was introduced -- verify_abc_scenarios
    reported 0 failures for a carry path whose grips were inside the bench.

    python3 scripts/verify_msc_tasks.py [--repeats 10]
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import tasks as TSK                                          # noqa: E402
import task0 as T0                                           # noqa: E402
import task3 as T3                                           # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/msc_verification.json")
STEP = 0.02
STANDOFF, LIFT = 0.10, 0.08

# T1's layout, verified 2026-08-11 under option 4 (fixtured objects).
T1_CUBES = [[0.280, 0.230], [0.340, 0.230], [0.380, 0.170], [0.400, 0.230]]
T1_PLANES = [[0.300, 0.145], [0.460, 0.145]]
T1_ARM = "left"
CUBE_M = 0.040


def require(seq, what):
    if not seq:
        raise RuntimeError("REFUSING: %s produced nothing to test. A zero "
                           "from a loop that never ran is not a "
                           "measurement." % what)
    return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--sampled-trials", type=int, default=20)
    a = ap.parse_args()
    N = a.repeats

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s %.4f rad from home" % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    import clip_scene as CS
    import time as _t
    tmp = CS.Scene.__new__(CS.Scene)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)

    def use(task):
        """Load exactly THIS task's furniture, having removed every other.

        EACH TASK IS CHECKED AGAINST ITS OWN SCENE, and that is a correction.
        One scene was applied for the whole run, so T0 -- which has no
        objects, no grasp and therefore no furniture -- was verified against
        another task's bench, and its target band was then derived inside the
        limits that bench imposes. A task's reachable volume is a property of
        the task's OWN scene.
        """
        CS.remove_furniture(n)
        objs = CS.Scene._collision_furniture(tmp, task)
        if not objs:
            return []
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = objs
        for _ in range(3):
            fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
            end = _t.time() + 20.0
            while _t.time() < end and not fut.done():
                rclpy.spin_once(n, timeout_sec=0.05)
            if fut.done():
                return [o.id for o in objs]
        raise RuntimeError("REFUSING: %s furniture did not apply." % task)

    calls = {"n": 0}

    def ok(arm, p, k=None):
        k = N if k is None else k
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    def pair_ok(p, sep, k=None):
        lo = [p[0] - sep / 2.0, p[1], p[2]]
        hi = [p[0] + sep / 2.0, p[1], p[2]]
        return ((ok("left", hi, k) and ok("right", lo, k)) or
                (ok("left", lo, k) and ok("right", hi, k)))

    def pick_path(obj_xyz):
        ee = CT.ee_for(obj_xyz)
        pre = [ee[0], ee[1], ee[2] + STANDOFF]
        up = [ee[0], ee[1], ee[2] + LIFT]
        return require(densify([pre, ee], STEP) + densify([ee, up], STEP),
                       "pick path for %s" % (obj_xyz,))

    print("=" * 74)
    print("MSc TASKS T0 / T1 / T2 / T3 -- N=%d, bench in scene, both arm "
          "assignments" % N)
    print("=" * 74)

    # ------------------------------------------------ CONTROLS
    # Run with the LEGACY A/B/C scene loaded, and that matters now that each
    # task carries its own furniture. Two of the four controls are poses whose
    # expected answer is only defined against a particular scene: "inside the
    # bench slab" needs a bench, and "a known-good pick" is task A's own pick,
    # verified against task A's own furniture. Asking either of them inside
    # T1's scene measures T1's supports, not the solver.
    use("a")
    ctl_far = ok("left", [1.60, 0.35, 1.15], k=1)
    ctl_wearer = ok("left", [0.0, -0.10, 1.25], k=1)
    ctl_bench = ok("left", [0.30, 0.40, 1.08], k=1)
    ctl_good = ok("left", CT.A_PICK, k=1)
    print("\nCONTROLS")
    print("   1.6 m out            -> %s (want unreachable)"
          % ("REACHABLE" if ctl_far else "unreachable"))
    print("   inside the wearer    -> %s (want unreachable)"
          % ("REACHABLE" if ctl_wearer else "unreachable"))
    print("   INSIDE THE BENCH     -> %s (want unreachable)"
          % ("REACHABLE" if ctl_bench else "unreachable"))
    print("   a known-good pick    -> %s (want reachable)"
          % ("reachable" if ctl_good else "UNREACHABLE"))
    if ctl_far or ctl_wearer or ctl_bench or not ctl_good:
        CS.remove_furniture(n)
        print("\n  REFUSING TO REPORT: a control failed. Any count here -- "
              "especially a zero -- would mean nothing.")
        n.destroy_node()
        rclpy.shutdown()
        return 6
    out = {"controls": dict(far=not ctl_far, wearer=not ctl_wearer,
                            bench=not ctl_bench, known_good=bool(ctl_good)),
           "repeats": N,
           # PER TASK, not one scene for the run -- see use().
           "furniture_per_task": {t: [f[0] for f in CS.furniture_boxes(t)]
                                  for t in MCT.ORDER}}
    fails = 0

    # ------------------------------------------------ T0
    # T0 HAS NO FURNITURE. It is reaching only -- no objects, no grasp,
    # nothing to rest on a surface -- so it is verified in the free space it
    # actually runs in, not against another task's bench.
    print("\nT0  sphere pointing -- NO FURNITURE (%s), transits, and %d "
          "sampled trials" % (use("t0") or "free space", a.sampled_trials))
    t0 = {}
    for arm, pts in (("left", T0.TARGETS_LEFT), ("right", T0.TARGETS_RIGHT)):
        bad = [k for k, p in sorted(pts.items()) if not ok(arm, p)]
        seg = 0
        for x, y in itertools.combinations(sorted(pts), 2):
            path = require(densify([pts[x], pts[y]], STEP),
                           "T0 transit %s-%s" % (x, y))
            seg += sum(1 for w in path if not ok(arm, w))
        t0[arm] = dict(sphere_failures=len(bad), transit_failures=seg)
        fails += len(bad) + seg
        print("   %-5s %d sphere failures, %d transit failures"
              % (arm, len(bad), seg))
    checked = 0
    sbad = []
    for seed in range(a.sampled_trials):
        tgt, _ = T0.sample_trial(seed)
        for label, p in sorted(tgt.items()):
            checked += 1
            if not ok("left" if label.startswith("L") else "right", p):
                sbad.append((seed, label))
    if checked == 0:
        raise RuntimeError("REFUSING: 0 sampled poses checked.")
    t0["sampled"] = dict(checked=checked, unreachable=len(sbad))
    fails += len(sbad)
    print("   sampled: %d poses, %d unreachable" % (checked, len(sbad)))
    out["T0"] = t0

    # ------------------------------------------------ T1
    print("\nT1  pick and place -- 4 cubes + 2 planes, %s arm, full path "
          "(scene: %s)" % (T1_ARM, ", ".join(use("t1"))))
    zc = CT.BENCH_TOP + CUBE_M / 2.0
    t1, t1bad = {}, 0
    for kind, items in (("cube", T1_CUBES), ("plane", T1_PLANES)):
        for i, (x, y) in enumerate(items):
            path = pick_path([x, y, zc])
            b = sum(1 for w in path if not ok(T1_ARM, w))
            t1["%s_%d" % (kind, i)] = dict(xy=[x, y], waypoints=len(path),
                                           failures=b)
            t1bad += b
    fails += t1bad
    print("   %d items, %d waypoint failures" % (len(T1_CUBES) + len(T1_PLANES),
                                                 t1bad))
    # every cube -> every plane transport, densified
    tbad = 0
    for (cx, cy) in T1_CUBES:
        for (px, py) in T1_PLANES:
            src = CT.ee_for([cx, cy, zc])
            dst = CT.ee_for([px, py, zc])
            path = require(densify([[src[0], src[1], src[2] + LIFT],
                                    [dst[0], dst[1], dst[2] + LIFT]], STEP),
                           "T1 transport")
            tbad += sum(1 for w in path if not ok(T1_ARM, w))
    t1["transport_failures"] = tbad
    fails += tbad
    print("   %d transports, %d waypoint failures" % (8, tbad))
    out["T1"] = t1

    # ------------------------------------------------ T2
    print("\nT2  coordinated carry -- both grippers, %d mm span, both "
          "assignments (scene: %s)"
          % (TSK.TRAY_SEP * 1000, ", ".join(use("t2"))))
    t2, t2bad = {}, 0
    for name, path in TSK.TASK_B["paths"].items():
        pts = require(densify(path, STEP), "T2 %s" % name)
        b = sum(1 for p in pts if not pair_ok(p, TSK.TRAY_SEP))
        t2[name] = dict(waypoints=len(pts), failures=b)
        t2bad += b
        print("   %-16s %2d waypoints, %d failures" % (name, len(pts), b))
    fails += t2bad
    t2["band_z"] = list(TSK.TASK_B["band_z"])
    out["T2"] = t2

    # ------------------------------------------------ T3
    print("\nT3  circuit box and multimeter -- hold poses and the full picks "
          "(scene: %s)" % ", ".join(use("t3")))
    t3, t3bad = {}, 0
    for arm, p in T3.poses_to_verify():
        good = ok(arm, CT.ee_for(p))
        t3["%s_%s" % (arm, "_".join("%.3f" % v for v in p))] = bool(good)
        if not good:
            t3bad += 1
    for label, obj, arm in (("circuit_box", T3.BOX_OBJ, T3.BOX_ARM),
                            ("multimeter", T3.METER_OBJ, T3.METER_ARM)):
        path = pick_path(obj)
        b = sum(1 for w in path if not ok(arm, w))
        t3["%s_pick_failures" % label] = b
        t3bad += b
        print("   %-12s %-5s arm, %2d waypoints, %d failures"
              % (label, arm, len(path), b))
    fails += t3bad
    t3["min_expected_repositioning_requests"] = T3.MIN_EXPECTED_REQUESTS
    out["T3"] = t3

    # ------------------------------------------------ THE CLIP PATHS
    # The recorder drives THESE, not the specs above. A spec verified and a
    # clip path unverified is EXACTLY how the archived set came to record a
    # carry through the bench: the two are different lists and only one had
    # ever been checked.
    print("\nCLIP PATHS  the waypoints record_abc_sweep actually drives")
    clips, cbad, ctested = {}, 0, 0
    for key in MCT.ORDER:
        spec = MCT.TASKS[key]
        use(key)                    # this task's own scene, again
        path = spec["build"]()
        per = {}
        for arm in ("left", "right"):
            pts = require(path[arm], "clip path %s/%s" % (key, arm))
            # De-duplicate: a HOLD repeats one pose and re-solving it N times
            # per repeat costs a great deal and tells you nothing new.
            uniq, seen = [], set()
            for w in pts:
                kk = tuple(round(v, 4) for v in w)
                if kk not in seen:
                    seen.add(kk)
                    uniq.append(list(w))
            b = sum(1 for w in uniq if not ok(arm, w))
            ctested += len(uniq)
            per[arm] = dict(waypoints=len(pts), distinct=len(uniq),
                            failures=b)
            cbad += b
        clips[key] = per
        print("   %-3s %-28s L %3d distinct %d fail   R %3d distinct %d fail"
              % (key, spec["name"], per["left"]["distinct"],
                 per["left"]["failures"], per["right"]["distinct"],
                 per["right"]["failures"]))
    if ctested == 0:
        raise RuntimeError(
            "REFUSING: 0 clip waypoints were tested. A zero from a loop that "
            "never ran is not a measurement -- and this is the very section "
            "that was silently absent from the previous run.")
    print("   %d distinct clip waypoints tested at N=%d" % (ctested, N))
    fails += cbad
    out["clip_paths"] = clips
    out["clip_waypoints_tested"] = ctested

    CS.remove_furniture(n)
    out["ik_calls"] = calls["n"]
    out["failures"] = fails
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("\n" + "=" * 74)
    print("  %d IK calls   TOTAL FAILURES: %d" % (calls["n"], fails))
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
