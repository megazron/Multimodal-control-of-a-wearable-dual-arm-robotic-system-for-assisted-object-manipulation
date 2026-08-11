#!/usr/bin/env python3
"""Settle blocker A (T2's carry vs the bench) and blocker B (T3's box).

BLOCKER A.  Two checks disagree on the same path: verify_abc_scenarios says 0
failures, the re-baselined regression block says 21.  The difference is the
bench.  This resolves it three ways so the answer does not rest on one:

  (i)   ARITHMETIC.  Is the failing waypoint geometrically inside the bench?
        That needs no solver and cannot be wrong.
  (ii)  PER-WAYPOINT.  Which waypoints fail, with the bench in the scene, at
        N=10 and both arm assignments.
  (iii) A CLEAR BAND.  The lowest z at which the whole span is feasible, so
        T2 can be re-specified rather than merely condemned.

BLOCKER B.  The earlier check asked for `ee_for(BOX_OBJ)` -- the wrist pose to
grasp the circuit box at its CENTRE.  clip_tasks' own comment says that is the
wrong pose: "Targeting an object's CENTRE puts the wrist inside that object's
own footprint -- the wrist trails the pads by 95 mm -- so the gripper body
clips the near face on the way in."  The file therefore defines BOX_NEAR_Y and
uses it everywhere.  So the question is re-asked at the near-edge pose, with
both arm assignments and both approach policies, before anything is concluded.

    python3 scripts/settle_blockers_ab.py [--repeats 10]
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import tasks as TSK                                          # noqa: E402
from srl_autonomy.grasp_library import _quat_from_z_and_x    # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/blockers_ab.json")


def inside_bench(p, margin=0.0):
    """Pure arithmetic: is this point inside the bench slab?"""
    return (abs(p[0]) <= CT.BENCH_HALF_X + margin and
            CT.BENCH_NEAR_Y - margin <= p[1] <= CT.BENCH_FAR_Y + margin and
            CT.BENCH_TOP - CT.BENCH_THICK - margin <= p[2]
            <= CT.BENCH_TOP + margin)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--quick", type=int, default=3)
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
    anchor = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in anchor.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4
    qt = _quat_from_z_and_x([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    topdown = Quaternion(x=float(qt[0]), y=float(qt[1]), z=float(qt[2]),
                         w=float(qt[3]))

    import clip_scene as CS
    tmp = CS.Scene.__new__(CS.Scene)
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(tmp)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)
    import time as _t
    applied = False
    for _ in range(3):
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if fut.done():
            applied = True
            break
    if not applied:
        print("REFUSING: bench did not apply")
        return 5
    print("bench in scene: True   slab z %.3f..%.3f, y %.3f..%.3f, |x|<=%.2f"
          % (CT.BENCH_TOP - CT.BENCH_THICK, CT.BENCH_TOP, CT.BENCH_NEAR_Y,
             CT.BENCH_FAR_Y, CT.BENCH_HALF_X))

    calls = {"n": 0}

    def ok(arm, p, q, k):
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), q, tries=6):
                return False
        return True

    def pair_ok(p, sep, k):
        """Both grips, BOTH arm assignments -- the naming is viewer
        perspective, not anatomical."""
        lo = [p[0] - sep / 2.0, p[1], p[2]]
        hi = [p[0] + sep / 2.0, p[1], p[2]]
        return ((ok("left", hi, anchor["left"], k) and
                 ok("right", lo, anchor["right"], k)) or
                (ok("left", lo, anchor["left"], k) and
                 ok("right", hi, anchor["right"], k)))

    ctl_good = ok("left", CT.A_PICK, anchor["left"], 1)
    ctl_far = ok("left", [1.6, 0.35, 1.15], anchor["left"], 1)
    print("CONTROLS  known-good -> %s;  1.6 m out -> %s"
          % ("ok" if ctl_good else "UNREACHABLE",
             "SOLVED" if ctl_far else "none"))
    if not ctl_good or ctl_far:
        CS.remove_furniture(n)
        print("REFUSING TO REPORT: a control failed.")
        return 6

    res = {"repeats": a.repeats, "bench": dict(
        top=CT.BENCH_TOP, thick=CT.BENCH_THICK, near_y=CT.BENCH_NEAR_Y,
        far_y=CT.BENCH_FAR_Y, half_x=CT.BENCH_HALF_X)}

    # ================================================== BLOCKER A
    print("\n" + "=" * 72)
    print("BLOCKER A  T2's carry, with the bench in the scene")
    print("=" * 72)
    sep = TSK.TRAY_SEP

    # (i) arithmetic -- no solver involved, so it cannot be a solver artefact
    geo = []
    for name, path in TSK.TASK_B["paths"].items():
        for p in densify(path, 0.02):
            for s in (-0.5, +0.5):
                g = [p[0] + s * sep, p[1], p[2]]
                if inside_bench(g):
                    geo.append({"path": name, "grip": [round(v, 4) for v in g]})
    print("  (i) ARITHMETIC: %d declared grip poses lie INSIDE the bench slab"
          % len(geo))
    for g in geo[:6]:
        print("        %s  %s" % (g["path"], g["grip"]))
    res["A_grips_inside_bench"] = len(geo)
    res["A_examples"] = geo[:10]

    # (ii) per waypoint, N=10, both assignments
    print("  (ii) PER WAYPOINT, N=%d, both arm assignments:" % a.repeats)
    perpath, total_bad, total_wp = {}, 0, 0
    for name, path in TSK.TASK_B["paths"].items():
        pts = densify(path, 0.02)
        if not pts:
            raise RuntimeError("densify returned nothing for %s" % name)
        bad = [p for p in pts if not pair_ok(p, sep, a.repeats)]
        perpath[name] = {"waypoints": len(pts), "failures": len(bad),
                         "failed_z": sorted({round(p[2], 3) for p in bad})}
        total_bad += len(bad)
        total_wp += len(pts)
        print("      %-16s %2d waypoints, %2d fail   failing z: %s"
              % (name, len(pts), len(bad), perpath[name]["failed_z"]))
    if total_wp == 0:
        print("      REFUSING: 0 waypoints tested.")
        res["A"] = {"verdict": "INVALID -- 0 waypoints"}
    else:
        res["A_per_path"] = perpath
        res["A_total_failures"] = total_bad
        res["A_total_waypoints"] = total_wp

    # (iii) the clear band
    print("  (iii) CLEAR BAND: lowest z at which the span is feasible")
    band = {}
    for z in [round(1.10 + 0.02 * i, 3) for i in range(16)]:   # 1.10 .. 1.40
        band[z] = pair_ok([0.0, TSK.Y, z], sep, a.quick)
    clear = [z for z, v in band.items() if v]
    print("      feasible z: %s" % (clear if clear else "NONE"))
    res["A_band"] = {str(k): bool(v) for k, v in band.items()}
    res["A_lowest_clear_z"] = min(clear) if clear else None
    # And confirm at the detour's y too, since S3 leaves y = 0.35
    band38 = {}
    for z in [round(1.10 + 0.02 * i, 3) for i in range(16)]:
        band38[z] = pair_ok([0.0, 0.38, z], sep, a.quick)
    clear38 = [z for z, v in band38.items() if v]
    print("      at y=0.38 (S3's detour): %s" % (clear38 if clear38 else "NONE"))
    res["A_band_y038"] = {str(k): bool(v) for k, v in band38.items()}
    res["A_lowest_clear_z_y038"] = min(clear38) if clear38 else None

    # ================================================== BLOCKER B
    print("\n" + "=" * 72)
    print("BLOCKER B  T3's circuit box -- asked at the RIGHT pose this time")
    print("=" * 72)
    print("  box object centre %s, size %.2f deep x %.2f high"
          % ([round(v, 4) for v in CT.BOX_OBJ], CT.BOX_D, CT.BOX_H))
    cands = {
        "centre (the WRONG pose, kept as the control)": CT.ee_for(CT.BOX_OBJ),
        "near-edge, box mid-height": CT.ee_for(
            [CT.BOX_OBJ[0], CT.BOX_NEAR_Y, CT.BOX_OBJ[2]]),
        "near-edge, box top": CT.ee_for(
            [CT.BOX_OBJ[0], CT.BOX_NEAR_Y, CT.BENCH_TOP + CT.BOX_H]),
        "B_PLACE (what task_b actually commands)": CT.B_PLACE,
        "C_PROBE_ON (what task_c actually commands)": CT.C_PROBE_ON,
    }
    bres = {}
    for label, ee in cands.items():
        row = {}
        for pol, qq in (("fixed", None), ("topdown", topdown)):
            arms = []
            for arm in ("left", "right"):
                q = anchor[arm] if qq is None else qq
                if ok(arm, ee, q, a.repeats):
                    arms.append(arm)
            row[pol] = arms
        bres[label] = {"ee": [round(v, 4) for v in ee], **row}
        print("   %-42s fixed:%-14s topdown:%s"
              % (label[:42], ",".join(row["fixed"]) or "NEITHER",
                 ",".join(row["topdown"]) or "NEITHER"))
    res["B_candidates"] = bres

    # If no candidate works under `fixed`, sweep for a HOLD pose that does.
    works = any(v["fixed"] for k, v in bres.items() if "WRONG" not in k)
    res["B_near_edge_reachable_fixed"] = bool(works)
    if not works:
        print("\n   No declared pose is holdable with a pinned wrist. "
              "Sweeping for one:")
        hits = []
        for x in [round(-0.24 - 0.03 * i, 3) for i in range(12)]:
            for y in (0.13, 0.17, 0.21, 0.25):
                for z in (CT.BENCH_TOP + CT.BOX_H / 2.0,
                          CT.BENCH_TOP + CT.BOX_H):
                    ee = CT.ee_for([x, y, z])
                    for arm in ("left", "right"):
                        if ok(arm, ee, anchor[arm], a.quick):
                            hits.append({"x": x, "y": y, "z": round(z, 4),
                                         "arm": arm})
        res["B_sweep_hits"] = hits[:20]
        res["B_sweep_count"] = len(hits)
        print("      %d (x, y, z, arm) hold poses found" % len(hits))
        for h in hits[:8]:
            print("        %s" % h)
    else:
        print("\n   A declared pose IS holdable -- no sweep needed.")

    CS.remove_furniture(n)
    res["ik_calls"] = calls["n"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n%d IK calls -> %s" % (calls["n"], OUT))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
