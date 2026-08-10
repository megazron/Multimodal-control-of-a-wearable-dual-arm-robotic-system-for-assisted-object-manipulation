#!/usr/bin/env python3
"""How much room does each task pose actually have, with the furniture in?

WHY THIS AND NOT ANOTHER PASS/FAIL. The clip paths verify at N=10 with the
bench loaded -- 0 failures. But the reachable fore/aft band at a FIXED probe
height collapsed from 15/19 samples to 1/19 for the right arm once the
furniture was present, and "it passes" plus "the envelope collapsed" are not
contradictory: they are the same rig measured at different heights. What
neither answers is the question that decides whether a study can run on this
arm -- HOW CLOSE TO THE EDGE IS EVERY POSE.

So this perturbs each declared pose along +/-x, +/-y, +/-z until it fails,
and reports the surviving margin. A pose with 5 mm of room is a pose that
will fail on the day; a pose with 60 mm is a pose that will not. The
project's own rule is to keep declared figures at least 20 mm inside the last
pose that passed N/N, so 20 mm is the bar.

    python3 scripts/measure_task_margin.py [--repeats 10]
"""
import argparse
import json
import os
import sys
import time as T

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
import clip_scene as CS                                          # noqa: E402
import clip_tasks as CT                                          # noqa: E402
from moveit_msgs.msg import PlanningScene                        # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene                   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/task_margin.json")
STEP = 0.01
MAX = 0.12
BAR = 0.02          # the project's own margin rule


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    a = ap.parse_args()
    N = a.repeats

    import rclpy
    if not rclpy.ok():
        rclpy.init()
    n = Solver()
    n.spin(4.0)
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.3f rad from home." % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 for an end effector.")
        return 4

    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=12.0):
        print("REFUSING: no /apply_planning_scene -- the furniture cannot be "
              "loaded, and a margin measured against an empty world is the "
              "optimistic number this whole exercise exists to stop quoting.")
        return 5
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(
        CS.Scene.__new__(CS.Scene))
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = T.time() + 12
    while T.time() < end and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)

    calls = {"n": 0}

    def ok(arm, p):
        for _ in range(N):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    POSES = [
        ("A pick", "left", CT.A_PICK),
        ("A release", "left", CT.A_BIN),
        ("A idle", "right", CT.park(-0.32)),
        ("B pick", "right", CT.B_START),
        ("B place", "right", CT.B_PLACE),
        ("B idle", "left", CT.park(0.32)),
        ("C present", "left", CT.C_PRESENT),
        ("C probe", "right", CT.C_PROBE_ON),
    ]
    print("MARGIN AROUND EVERY DECLARED POSE, furniture IN, N=%d" % N)
    print("  bar is %.0f mm (the project's own rule)\n" % (BAR * 1000))
    print("  %-11s %-6s %7s %7s %7s %7s %7s %7s   %s"
          % ("pose", "arm", "-x", "+x", "-y", "+y", "-z", "+z", "worst"))
    rows = []
    for name, arm, p in POSES:
        if not ok(arm, p):
            print("  %-11s %-6s  POSE ITSELF FAILS" % (name, arm))
            rows.append(dict(pose=name, arm=arm, ok=False))
            continue
        m = {}
        for axis in (0, 1, 2):
            for sgn in (-1, +1):
                d = 0.0
                while d < MAX:
                    q = list(p)
                    q[axis] += sgn * (d + STEP)
                    if not ok(arm, q):
                        break
                    d += STEP
                m["%s%s" % ("-+"[sgn > 0], "xyz"[axis])] = round(d, 3)
        worst = min(m.values())
        rows.append(dict(pose=name, arm=arm, ok=True, margins=m,
                         worst_m=worst))
        print("  %-11s %-6s %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f   %5.0f mm %s"
              % (name, arm, m["-x"] * 1000, m["+x"] * 1000, m["-y"] * 1000,
                 m["+y"] * 1000, m["-z"] * 1000, m["+z"] * 1000, worst * 1000,
                 "" if worst >= BAR else "<-- UNDER THE BAR"))

    tight = [r for r in rows if r.get("ok") and r["worst_m"] < BAR]
    per_arm = {}
    for r in rows:
        if r.get("ok"):
            per_arm.setdefault(r["arm"], []).append(r["worst_m"])
    print()
    for arm in sorted(per_arm):
        v = per_arm[arm]
        print("  %-6s worst margin %.0f mm, median %.0f mm, over %d poses"
              % (arm, min(v) * 1000,
                 sorted(v)[len(v) // 2] * 1000, len(v)))
    print("\n  poses under the %.0f mm bar: %d" % (BAR * 1000, len(tight)))
    for r in tight:
        print("     %s (%s) %.0f mm" % (r["pose"], r["arm"],
                                        r["worst_m"] * 1000))
    CS.remove_furniture(n)
    json.dump(dict(rows=rows, repeats=N, bar_m=BAR, ik_calls=calls["n"]),
              open(OUT, "w"), indent=2)
    print("\n  %d IK calls -> %s" % (calls["n"], OUT))
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass
    sys.exit(rc)
