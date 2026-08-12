#!/usr/bin/env python3
"""HOW MUCH LIFT is available at a grasp point, with the pinned wrist?

    python3 scripts/measure_lift.py

T1 IS CALLED PICK AND PLACE. If the achievable lift at a cube is zero, the
task is push-and-place under the pinned wrist, and that belongs in the task
DEFINITION rather than being discovered by a viewer watching a cube skate
across a table. If it is 20 mm, a low but real lift is still a pick and the
number simply needs stating.

MEASURED, NOT ADJUSTED. This does not tune the approach to improve the answer.
It asks one question at each real grasp point: starting from the grasp pose,
how far straight UP can the gripper go before IK fails, with the scene's own
collision geometry loaded and the wrist pinned exactly as teleop pins it?

Reported three ways, because they are three different claims:

  PINNED, scene loaded    what T1 actually gets today
  PINNED, no furniture    how much of the limit is the furniture
  FREE wrist, scene       how much of the limit is the wrist lock

N=10 at every height, short-circuiting on the first failure, and the lift is
the last height that passed N/N -- so a height that solves 7 times out of 10
does NOT count, per the rule that a MARGINAL pose is not a usable pose.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))

from geometry_msgs.msg import Quaternion                      # noqa: E402
from moveit_msgs.msg import PlanningScene                     # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene                # noqa: E402
from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402

N = 10
DZ = 0.005          # 5 mm steps: a "20 mm lift" claim needs mm resolution
DZ_MAX = 0.30
OUT = os.path.join(WS, "recordings", "baselines", "lift.json")


def _fan():
    """A spread of wrist orientations, for the FREE-wrist comparison."""
    import math
    out = []
    for tilt in (0.0, 20.0, 40.0, 60.0):
        for yaw in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
            t, y = math.radians(tilt), math.radians(yaw)
            z = np.array([math.sin(t) * math.sin(y), math.sin(t) * math.cos(y),
                          -math.cos(t)])
            z = z / np.linalg.norm(z)
            up = np.array([0.0, 1.0, 0.0])
            if abs(float(z @ up)) > 0.95:
                up = np.array([1.0, 0.0, 0.0])
            x = np.cross(up, z)
            x = x / np.linalg.norm(x)
            R = np.column_stack([x, np.cross(z, x), z])
            tr = R[0, 0] + R[1, 1] + R[2, 2]
            if tr > 0:
                s = math.sqrt(tr + 1.0) * 2
                q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                     (R[1, 0] - R[0, 1]) / s, 0.25 * s)
            else:
                i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
                if i == 0:
                    s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    q = (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
                elif i == 1:
                    s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
                else:
                    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                         0.25 * s, (R[1, 0] - R[0, 1]) / s)
            m = Quaternion()
            m.x, m.y, m.z, m.w = (float(v) for v in q)
            out.append(m)
    return out


def main():
    import clip_scene as CS
    import msc_clip_tasks as MCT

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s %.4f rad from home" % (arm, w))
            return 3
    pinned = {a: n.ee_quat(a) for a in ("left", "right")}
    fan = _fan()

    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)
    tmp = CS.Scene.__new__(CS.Scene)

    def scene(on):
        CS.remove_furniture(n)
        n.spin(0.8)
        objs = CS.Scene._collision_furniture(tmp, "t1") if on else []
        if objs:
            ps = PlanningScene()
            ps.is_diff = True
            ps.world.collision_objects = objs
            f = cli.call_async(ApplyPlanningScene.Request(scene=ps))
            end = time.time() + 20.0
            while time.time() < end and not f.done():
                rclpy.spin_once(n, timeout_sec=0.05)
        n.spin(1.0)
        return len(objs)

    def lift(arm, x, y, z0, quats):
        """Last height above z0 that solves N/N. MARGINAL does not count."""
        best = None
        dz = 0.0
        while dz <= DZ_MAX + 1e-9:
            ok = any(all(n.solve(arm, [x, y, z0 + dz], q, tries=6)
                         for _ in range(N)) for q in quats)
            if not ok:
                break
            best = dz
            dz += DZ
        return best

    res = {}
    print("grasp height z=%.3f, steps of %d mm, N=%d\n" % (MCT.T1_Z,
                                                           int(DZ * 1000), N))
    for label, on, free in (("PINNED, scene", True, False),
                            ("PINNED, no furniture", False, False),
                            ("FREE wrist, scene", True, True)):
        nobj = scene(on)
        row = {}
        for arm in ("left", "right"):
            sx = 1.0 if arm == "left" else -1.0
            vals = []
            for i, (cx, cy) in enumerate(MCT.T1_CUBES):
                q = fan if free else [pinned[arm]]
                v = lift(arm, sx * abs(cx), cy, MCT.T1_Z, q)
                vals.append(v)
            row[arm] = vals
            pretty = ", ".join("--" if v is None else "%d" % round(v * 1000)
                               for v in vals)
            got = [v for v in vals if v is not None]
            print("   %-22s %-5s  per cube (mm): %-22s  min %s"
                  % (label, arm, pretty,
                     "NONE REACHABLE" if not got
                     else "%d mm" % round(min(got) * 1000)))
        res[label] = dict(objects=nobj, per_cube=row)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(results=res, grasp_z=MCT.T1_Z, cubes=MCT.T1_CUBES,
                   repeats=N, step_m=DZ), open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
