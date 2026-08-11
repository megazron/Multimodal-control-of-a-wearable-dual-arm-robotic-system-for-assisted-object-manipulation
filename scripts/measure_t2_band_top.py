#!/usr/bin/env python3
"""How far UP does T2's coordinated-carry band actually go?

    python3 scripts/measure_t2_band_top.py [--repeats 10]

WHY. T2's whole commanded motion is 80 mm -- `S2_full_lift` runs z 1.32 to
1.40 -- which is not enough travel to see in a clip. The band was re-spec'd
on 2026-08-11 from a survey that established its BOTTOM ("feasible from
z = 1.28 at y = 0.35, and from 1.30 at y = 0.38") and recorded its top only as
"up to at least 1.40", because 1.40 was as high as that survey looked.

This looks higher, the same way and with the same scene: both grippers, 500 mm
apart, BOTH arm assignments, N repeats, T2's own furniture. It reports the
highest z that passes and the +20 mm margin this project keeps off a boundary.
It CHANGES NOTHING -- extending the study path is a spec decision.
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
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import tasks as TSK                                          # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t2_band_top.json")
MARGIN_M = 0.02


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--max-z", type=float, default=1.70)
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
    CS.remove_furniture(n)
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(tmp, "t2")
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + 20.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)

    sep = TSK.TRAY_SEP
    y = TSK.Y
    calls = {"n": 0}

    def ok(arm, p):
        for _ in range(a.repeats):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    def pair_ok(z):
        lo = [-sep / 2.0, y, z]
        hi = [+sep / 2.0, y, z]
        return ((ok("left", hi) and ok("right", lo)) or
                (ok("left", lo) and ok("right", hi)))

    band = TSK.TASK_B["band_z"]
    print("T2 band top -- span %.0f mm at y=%.2f, N=%d, T2's own scene"
          % (sep * 1000, y, a.repeats))
    print("  declared band %s" % (band,))
    rows, top = {}, None
    z = band[0]
    while z <= a.max_z + 1e-9:
        good = pair_ok(round(z, 4))
        rows["%.3f" % z] = bool(good)
        print("   z=%.3f  %s" % (z, "ok" if good else "FAIL"))
        if not good:
            break
        top = round(z, 4)
        z += a.step
    # CONTROL: a height nothing can reach, so a run of `ok` everywhere is not
    # reported as a discovery.
    ctl = pair_ok(2.20)
    print("  control z=2.20 -> %s (want FAIL)" % ("REACHABLE" if ctl else "fail"))
    out = dict(declared_band=list(band), rows=rows, highest_passing=top,
               recommended_top=(None if top is None
                                else round(top - MARGIN_M, 4)),
               margin_m=MARGIN_M, repeats=a.repeats, ik_calls=calls["n"],
               control_far_unreachable=(not ctl))
    CS.remove_furniture(n)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("\n  highest passing z %s, recommended top %s (+%.0f mm margin)"
          % (top, out["recommended_top"], MARGIN_M * 1000))
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if (top is not None and not ctl) else 1


if __name__ == "__main__":
    sys.exit(main())
