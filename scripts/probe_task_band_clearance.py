#!/usr/bin/env python3
"""WHICH arm link is inside the 120 mm clearance floor, and where does the
task band have to move to clear it?

    python3 scripts/probe_task_band_clearance.py

Recording every task showed every chest-band scenario sitting at 48-67 mm of
clearance -- IK-valid, contact-free, and below the 120 mm margin `real_robot`
mode enforces. That is a finding about the BAND, not about any one scenario,
so it deserves its own measurement rather than an inference from 87 clips.

This answers three questions the clips cannot:

  1. WHICH LINK is closest? A forearm brushing the torso is a path problem;
     a gripper doing it is a target problem, and they have different fixes.
  2. How far forward must the band move to clear 120 mm?
  3. Is the band still REACHABLE once it has moved? A clearance fix that
     leaves the task unreachable is not a fix.

Uses the project's own srl_teleop/clearance.py, not a second model.
"""
import os
import sys

import numpy as np
import rclpy
import rclpy.time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver, HOME_TOL_RAD             # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop.clearance import (TORSO_BODY, HEAD_BODY, HIPS_BODY,   # noqa: E402
                                  DISTAL_LINKS, point_clearance)

FLOOR_REAL = 0.12
FLOOR_SIM = 0.05
# The band every chest-height task uses, and the candidate replacements.
Y_CANDIDATES = (0.35, 0.40, 0.45, 0.50, 0.55)
X_PROBE = 0.155        # T3/T6 tray half-span; also close to T2's hold/opening
Z_PROBE = (1.10, 1.20, 1.30)
# Moving the band FORWARD does not work -- y=0.45 clears the floor but only
# 1 of 6 poses is reachable, and 0.50+ is unreachable. The closest link is the
# WRIST, not the forearm, which says the targets are too close to the torso
# LATERALLY rather than the path being wrong. T7 and T8 work at |x| >= 0.40
# and clear 0.16-0.19 m, so x is the axis with headroom. This sweep tests it.
X_CANDIDATES = (0.155, 0.20, 0.25, 0.30, 0.35, 0.40)


class P(Solver):
    def worst_link(self, arm):
        worst, who, link = float("inf"), "", ""
        bodies = {"torso": TORSO_BODY, "head": HEAD_BODY, "hips": HIPS_BODY}
        for ln in DISTAL_LINKS:
            for wf, prims in bodies.items():
                try:
                    t = self.buf.lookup_transform(
                        wf, "%s_%s" % (arm, ln), rclpy.time.Time())
                except Exception:                              # noqa: BLE001
                    continue
                v = t.transform.translation
                d = point_clearance((v.x, v.y, v.z), prims)
                if d < worst:
                    worst, who, link = float(d), wf, ln
        return worst, who, link


def main():
    rclpy.init()
    n = P()
    n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim")
        return 2
    q = {a: n.ee_quat(a) for a in ("left", "right")}
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    pub = {a: n.create_publisher(
        JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 5)
        for a in ("left", "right")}

    def drive(arm, xyz):
        """Solve and actually move there, then read clearance from TF."""
        from moveit_msgs.msg import PositionIKRequest, RobotState
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import Pose, PoseStamped
        import time as _t
        seed = [n.js.get(k, 0.0) for k in n.names(arm)]
        for k in range(6):
            s2 = list(seed)
            if k:
                s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request()
            r = PositionIKRequest()
            r.group_name = "%s_arm" % arm
            rs = RobotState()
            rs.joint_state.name = n.names(arm)
            rs.joint_state.position = [float(x) for x in s2]
            r.robot_state = rs
            r.avoid_collisions = True
            ps = PoseStamped()
            ps.header.frame_id = "world"
            pz = Pose()
            pz.position.x, pz.position.y, pz.position.z = (float(v) for v in xyz)
            pz.orientation = q[arm]
            ps.pose = pz
            r.pose_stamped = ps
            r.timeout.nanosec = 100_000_000
            req.ik_request = r
            fut = n.ik.call_async(req)
            t0 = _t.monotonic()
            while not fut.done() and _t.monotonic() - t0 < 3.0:
                rclpy.spin_once(n, timeout_sec=0.002)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                d = dict(zip(res.solution.joint_state.name,
                             res.solution.joint_state.position))
                sol = [d[j] for j in n.names(arm)]
                m = JointTrajectory()
                m.joint_names = n.names(arm)
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in sol]
                pt.time_from_start.sec = 1
                m.points = [pt]
                pub[arm].publish(m)
                n.spin(1.2)
                return True
        return False

    print("TASK-BAND CLEARANCE PROBE")
    print("floors: sim %.2f m, real_robot %.2f m\n" % (FLOOR_SIM, FLOOR_REAL))
    print("  %-6s %-5s %-6s %10s  %-34s %s"
          % ("y", "z", "arm", "clearance", "closest pair", "verdict"))
    results = []
    for y in Y_CANDIDATES:
        for z in Z_PROBE:
            for arm, sx in (("left", +1), ("right", -1)):
                p = [sx * X_PROBE, y, z]
                if not drive(arm, p):
                    print("  %-6.2f %-5.2f %-6s %10s  %-34s %s"
                          % (y, z, arm, "-", "-", "UNREACHABLE"))
                    results.append((y, z, arm, float("nan"), "", False))
                    continue
                d, who, link = n.worst_link(arm)
                ok = d >= FLOOR_REAL
                print("  %-6.2f %-5.2f %-6s %9.3f m  %-34s %s"
                      % (y, z, arm, d, "%s <-> %s_%s" % (who, arm, link),
                         "clears 120 mm" if ok else
                         "BELOW the real_robot floor"))
                results.append((y, z, arm, d, link, ok))

    print("\nSUMMARY -- minimum clearance over the three heights, per y")
    for y in Y_CANDIDATES:
        vals = [r[3] for r in results if r[0] == y and r[3] == r[3]]
        reach = sum(1 for r in results if r[0] == y and r[3] == r[3])
        tot = sum(1 for r in results if r[0] == y)
        if vals:
            print("  y = %.2f   worst %.3f m   reachable %d/%d   %s"
                  % (y, min(vals), reach, tot,
                     "CLEARS" if min(vals) >= FLOOR_REAL else "below floor"))
        else:
            print("  y = %.2f   nothing reachable" % y)

    # ------------------------------------------------------------ x sweep
    print("\nLATERAL SWEEP -- half-span x at y=0.35, the reachable band")
    print("  %-7s %-5s %-6s %10s  %s" % ("x", "z", "arm", "clearance", "verdict"))
    xres = []
    for xx in X_CANDIDATES:
        for z in (1.10, 1.20, 1.30):
            for arm, sx in (("left", +1), ("right", -1)):
                if not drive(arm, [sx * xx, 0.35, z]):
                    xres.append((xx, z, arm, float("nan"), False))
                    print("  %-7.3f %-5.2f %-6s %10s  UNREACHABLE"
                          % (xx, z, arm, "-"))
                    continue
                d, who, link = n.worst_link(arm)
                ok = d >= FLOOR_REAL
                xres.append((xx, z, arm, d, ok))
                print("  %-7.3f %-5.2f %-6s %9.3f m  %s"
                      % (xx, z, arm, d,
                         "clears 120 mm" if ok else "below floor"))
    print("\n  half-span   worst clearance   reachable   verdict")
    for xx in X_CANDIDATES:
        v = [r[3] for r in xres if r[0] == xx and r[3] == r[3]]
        tot = sum(1 for r in xres if r[0] == xx)
        if v:
            print("   %.3f m     %.3f m           %d/%d         %s"
                  % (xx, min(v), len(v), tot,
                     "CLEARS" if min(v) >= FLOOR_REAL else "below floor"))
        else:
            print("   %.3f m     -                 0/%d         unreachable"
                  % (xx, tot))
    print("\n  A tray span is 2x the half-span: x=0.30 means a 600 mm tray.")

    links = {}
    for r in results:
        if r[4]:
            links[r[4]] = links.get(r[4], 0) + 1
    print("\nCLOSEST LINK, counted over all probes")
    for k, v in sorted(links.items(), key=lambda kv: -kv[1]):
        print("  %-28s %d" % (k, v))
    print("\nA forearm brushing the torso is a PATH problem -- the band is too")
    print("close to the body. A gripper doing it would be a TARGET problem.")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
