#!/usr/bin/env python3
"""GATES 1-3 for the MSc experiment brief, measured in one sim session.

Each gate is a question the brief says must be answered BEFORE building,
because the answer changes what gets built.  Nothing here designs anything.

  GATE 1  How long do the four tasks actually take, at what velocity cap?
          Measured as TRAJECTORY DURATION over the declared task paths --
          max joint delta per segment divided by the cap, which is exactly
          how ik_follower_node computes time_from_start -- not guessed from
          "about 3 rad".
  GATE 2  Can the PINNED wrist grasp?  Four orientation policies compared on
          the same poses: fixed anchor, anchor with roll/pitch tilt, anchor
          with YAW FREE (what a sphere or a vertical cylinder allows), and a
          true top-down grasp.
  GATE 3  Does a cup-over-cup pose pair exist AT ALL -- one arm above the
          other, N=10 over the full path, bench in the scene?  If it does
          not, grasp stability and pour angle are moot.

    python3 scripts/gate_check_experiment.py [--repeats 10]

Every gate is bracketed by instrument controls and REFUSES to report if one
fails, because three of the numbers it is likely to print are zeros and a
broken solver prints zeros too.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import tasks as T                                            # noqa: E402
from srl_autonomy.grasp_library import _quat_from_z_and_x    # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/experiment_gates.json")

# Caps, read from where they actually live rather than from the brief.
#   teleop.launch.py: real_max_vel_rad_s default 0.15, real_max_step_rad 0.05
#   ik_follower_node: max_vel_rad_s default 0.6 (sim)
CAPS = {"real (shipped, 0.15)": 0.15, "sim default (0.6)": 0.60,
        "brief's assumption (0.05)": 0.05, "candidate 0.30": 0.30}


def tilt_variants(q, n=4, amp=0.30):
    """Anchor rotated by +/-amp about the tool's own x and y.

    This is what `orientation_mode: tilt` would command.  It is sampled
    EXPLICITLY rather than passed as an OrientationConstraint, because this
    /compute_ik setup ignores the constraints field entirely -- measured, and
    the reason tilt was never shipped on the strength of a constraint test.
    """
    out = [q]
    for axis in (0, 1):
        for s in (+1, -1):
            h = s * amp / 2.0
            c, sn = math.cos(h), math.sin(h)
            d = [0.0, 0.0, 0.0, c]
            d[axis] = sn
            x1, y1, z1, w1 = q.x, q.y, q.z, q.w
            x2, y2, z2, w2 = d
            out.append(Quaternion(
                x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))
    return out[:1 + n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=0.02)
    a = ap.parse_args()
    N = a.repeats

    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home" % (arm, w))
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
    if not cli.wait_for_service(timeout_sec=15.0):
        print("REFUSING: no /apply_planning_scene")
        return 5
    import time as _t
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + 15.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    bench_in = bool(fut.done())
    print("bench in scene: %s" % bench_in)

    res = {"bench_in_scene": bench_in, "repeats": N, "caps": CAPS}
    calls = {"n": 0}

    def sol(arm, p, q, k=1, avoid=True):
        """Return the joint solution, or None.  Repeated k times; the LAST
        solution is returned so the caller can chain seeds."""
        got = None
        for _ in range(k):
            calls["n"] += 1
            got = n.solve_joints(arm, list(p), q, avoid=avoid, tries=6)
            if not got:
                return None
        return got

    # ------------------------------------------------ instrument controls
    ctl_far = sol("left", [1.60, 0.35, 1.15], anchor["left"])
    ctl_good = sol("left", CT.A_PICK, anchor["left"])
    print("CONTROLS  1.6 m out -> %s (want none);  Task A pick -> %s (want ok)"
          % ("SOLVED" if ctl_far else "none", "ok" if ctl_good else "NONE"))
    if ctl_far or not ctl_good:
        print("REFUSING TO REPORT: a control failed.")
        return 6
    res["controls"] = dict(far_unreachable=not bool(ctl_far),
                           known_good_reachable=bool(ctl_good))

    # =====================================================  GATE 1
    # Trajectory duration over the DECLARED task paths.  ik_follower_node sets
    # time_from_start from the largest joint delta over max_vel_rad_s, floored
    # at min_time_s, so that is what is summed here.
    print("\n" + "=" * 72)
    print("GATE 1  how long the declared paths actually take")
    print("=" * 72)

    def duration(arm, waypoints, cap, min_time_s=0.05):
        """Sum of per-segment times, and total joint travel."""
        prev, total, travel, miss = None, 0.0, 0.0, 0
        for p in waypoints:
            q = sol(arm, p, anchor[arm], k=1)
            if q is None:
                miss += 1
                continue
            q = np.asarray(q[:7], float)   # arm joints only; the solution
            #                                 carries gripper joints too, and
            #                                 including them would charge the
            #                                 trajectory for finger motion
            if prev is not None:
                d = float(np.max(np.abs(q - prev)))
                travel += float(np.sum(np.abs(q - prev)))
                total += max(d / cap, min_time_s)
            prev = q
        return total, travel, miss

    g1 = {}
    # One representative pick-and-place: Task A's own clip path.
    a_path = CT.task_a()
    for label, arm in (("T1 pick-and-place (Task A clip path)", "left"),):
        pts = [w for w in a_path[arm]]
        uniq = []
        for p in pts:
            if not uniq or max(abs(x - y) for x, y in zip(p, uniq[-1])) > 1e-6:
                uniq.append(list(p))
        dense = densify(uniq, a.step) if len(uniq) > 1 else uniq
        row = {"waypoints": len(dense)}
        for cname, cap in CAPS.items():
            t, travel, miss = duration(arm, dense, cap)
            row[cname] = round(t, 1)
            row["joint_travel_rad"] = round(travel, 2)
            row["unsolved_waypoints"] = miss
        g1[label] = row
        print("  %-38s %3d wp, travel %.2f rad" %
              (label, row["waypoints"], row["joint_travel_rad"]))
        for cname in CAPS:
            print("      %-28s %6.1f s" % (cname, row[cname]))

    # Task B's carry, both arms, for the coupled task.
    for name, path in T.TASK_B["paths"].items():
        dense = densify(path, a.step)
        left = [[p[0] + T.TRAY_SEP / 2, p[1], p[2]] for p in dense]
        row = {"waypoints": len(dense)}
        for cname, cap in CAPS.items():
            t, travel, miss = duration("left", left, cap)
            row[cname] = round(t, 1)
            row["joint_travel_rad"] = round(travel, 2)
        g1["T2/B carry %s" % name] = row
        print("  %-38s %3d wp, travel %.2f rad, real cap %.1f s"
              % ("T2/B carry " + name, row["waypoints"],
                 row["joint_travel_rad"], row["real (shipped, 0.15)"]))
    res["gate1"] = g1

    # =====================================================  GATE 2
    print("\n" + "=" * 72)
    print("GATE 2  can the PINNED wrist grasp?  four orientation policies")
    print("=" * 72)
    # Poses: the graspable strip measured in 12_randomisation_region.md, so
    # this is asked where objects actually are, not over an abstract cube.
    poses = []
    for x in (0.30, 0.33, 0.36, 0.39, 0.42):
        for y in (0.15, 0.19, 0.23):
            poses.append([x, y, CT.BENCH_TOP + 0.02])
    g2 = {}
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        pol = {k: 0 for k in ("fixed", "tilt", "yaw_free", "topdown")}
        for p in poses:
            pp = [sgn * p[0], p[1], p[2]]
            ee = CT.ee_for(pp)
            if sol(arm, ee, anchor[arm], k=N):
                pol["fixed"] += 1
            if any(sol(arm, ee, q, k=1) for q in tilt_variants(anchor[arm])):
                pol["tilt"] += 1
            if n.solve_yaw_free(arm, ee, anchor[arm]):
                pol["yaw_free"] += 1
            if sol(arm, ee, topdown, k=1):
                pol["topdown"] += 1
        g2[arm] = {k: dict(ok=v, of=len(poses),
                           pct=round(100.0 * v / len(poses), 1))
                   for k, v in pol.items()}
        print("  %-5s  " % arm + "   ".join(
            "%s %d/%d (%.1f%%)" % (k, v["ok"], v["of"], v["pct"])
            for k, v in g2[arm].items()))
    res["gate2"] = g2

    # =====================================================  GATE 3
    print("\n" + "=" * 72)
    print("GATE 3  does a cup-OVER-cup pose pair exist at all?")
    print("=" * 72)
    # Pouring needs the source lip above the destination opening: the two EE
    # poses share x and y and differ in z.  That is not the same POINT, so it
    # is not ruled out by the 0-of-16 transfer-point result and has to be
    # measured on its own.
    g3 = {"dz_tested": [0.10, 0.15, 0.20], "hits": []}
    tested = 0
    for dz in g3["dz_tested"]:
        for x in [round(-0.5 + 0.05 * i, 3) for i in range(21)]:
            for y in (0.30, 0.35, 0.40):
                for zd in (1.10, 1.15, 1.20, 1.25):
                    lo = [x, y, zd]
                    hi = [x, y, zd + dz]
                    tested += 1
                    for a1, a2 in (("left", "right"), ("right", "left")):
                        if (sol(a1, hi, anchor[a1], k=1) and
                                sol(a2, lo, anchor[a2], k=1)):
                            g3["hits"].append(
                                dict(dz=dz, x=x, y=y, z_dest=zd,
                                     above=a1, below=a2))
    g3["pairs_tested"] = tested
    print("  %d (x, y, z, dz) stacks tested, %d admitted a pair"
          % (tested, len(g3["hits"])))
    if g3["hits"]:
        print("  first few:", g3["hits"][:5])
    else:
        print("  NO cup-over-cup pose pair exists anywhere in the swept "
              "volume.  Grasp stability, pour angle and repeatability are "
              "moot -- the pose does not exist.")
    # Control for THIS gate: the two arms must each be able to reach
    # SOMETHING in the swept band on their own, or the zero is the sweep's.
    solo = {arm: sum(1 for x in (-0.45, -0.35, 0.35, 0.45)
                     for zd in (1.10, 1.20)
                     if sol(arm, [x, 0.35, zd], anchor[arm], k=1))
            for arm in ("left", "right")}
    g3["solo_reachable_in_band"] = solo
    print("  CONTROL: single-arm reachable poses in the same band -- "
          "left %d, right %d (both must be > 0 for the zero to mean anything)"
          % (solo["left"], solo["right"]))
    if not (solo["left"] and solo["right"]):
        print("  REFUSING to report gate 3: the sweep found nothing for "
              "either arm alone, so it is not measuring what it claims.")
        g3["verdict"] = "INVALID -- control failed"
    else:
        g3["verdict"] = ("POSSIBLE" if g3["hits"] else
                         "IMPOSSIBLE in the swept volume")
    res["gate3"] = g3

    CS.remove_furniture(n)
    res["ik_calls"] = calls["n"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n%d IK calls  ->  %s" % (calls["n"], OUT))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
