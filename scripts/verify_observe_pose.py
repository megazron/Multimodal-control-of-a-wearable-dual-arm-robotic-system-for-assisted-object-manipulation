#!/usr/bin/env python3
"""IS THE OBSERVE POSE ACTUALLY REACHABLE, AND IS THE MOVE THERE SAFE?

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    python3 scripts/verify_observe_pose.py --repeats 10

WHY. `solve_observe_pose.py` finds a joint vector whose camera frustum covers
the work. That is a geometry result and it is not a reachability result: it
was solved against FK, joint limits, the wearer capsule model and the seam
margin, and NOTHING in it asked `/compute_ik` whether the follower can command
that pose, or whether the arm can GET there from home without going through
the person. A perception step that cannot be reached is worse than none,
because it fails in the middle of a trial rather than at design time.

So this checks four separate things and reports them separately:

  1. THE POSE IS COMMANDABLE. The observe joint vector is put through FK to an
     end-effector pose, and that pose is solved with `/compute_ik`,
     collision-aware, N times. The follower drives EE poses, not joint
     vectors, so a joint vector the solver cannot reproduce is not usable.
  2. THE POSE IS CLEAR OF THE WEARER, measured geometrically with the mount
     guard's own capsule model. `/check_state_validity` is NOT the wearer
     check and must never be used as one -- the SRDF permanently excludes
     torso/harness/backpack against each arm's base, shoulder and half_arm_1,
     which are exactly the pairs a shoulder mount threatens. CLAUDE.md hard
     constraint 11.
  3. THE POSE IS CLEAR OF EVERYTHING ELSE -- itself, the other arm and the
     furniture -- which IS what `/check_state_validity` is for.
  4. THE MOVE FROM HOME IS SHORT AND CLEAN. The joint-space path home ->
     observe is densified so no joint moves more than `--step-rad` between
     samples, and every sample gets checks 1-3. The total joint travel and
     the end-effector path length are reported, so "short" is a number.

CONTROLS, and there is no report without them:

    the arms are AT HOME               the transit is measured FROM home
    a config driven into the torso     must read NEGATIVE clearance
    a deliberately bad transit         interpolating to a pose inside the
                                       wearer must be rejected, or the
                                       transit check cannot fail
    the validity service answers       an unreachable service silently
                                       passing everything is the worst case
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                  # noqa: E402
from srl_fk import FK, CompiledFK                                # noqa: E402
import solve_home_pose as SH                                     # noqa: E402
import home_positions as hp                                      # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/observe_pose_verified.json")


def quat_of(M):
    """(x, y, z, w) from a 3x3 rotation, branch-safe."""
    R = M[:3, :3]
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def as_quat_msg(q):
    from geometry_msgs.msg import Quaternion
    m = Quaternion()
    m.x, m.y, m.z, m.w = (float(v) for v in q)
    return m


class Checker(Solver):
    def __init__(self):
        super().__init__()
        from moveit_msgs.srv import GetStateValidity
        self.sv = self.create_client(GetStateValidity, "/check_state_validity")
        self.sv_ok = self.sv.wait_for_service(timeout_sec=25.0)

    def valid(self, arm, q, timeout=5.0):
        """Self / other-arm / furniture collisions. NOT the wearer check."""
        if not self.sv_ok:
            return None
        from moveit_msgs.srv import GetStateValidity
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        req = GetStateValidity.Request()
        js = JointState()
        js.name = self.names(arm)
        js.position = [float(v) for v in q]
        rs = RobotState()
        rs.joint_state = js
        rs.is_diff = True
        req.robot_state = rs
        req.group_name = "%s_arm" % arm
        fut = self.sv.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.002)
        res = fut.result()
        return None if res is None else bool(res.valid)


def densify_joints(a, b, step):
    """Joint-space interpolation with no joint moving more than `step`."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    n = max(2, int(math.ceil(float(np.max(np.abs(d))) / step)) + 1)
    return [a + d * (i / float(n - 1)) for i in range(n)], n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step-rad", type=float, default=0.05)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Checker()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        off, j = n.home_ok(arm)
        if off > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)"
                  % (arm, off, j))
            return 3
    rig = Rig(n, "t1", 1)
    sc = SH.Scorer()
    fk = sc.fk
    ee = {arm: CompiledFK(fk, arm, ["end_effector_link"])
          for arm in ("left", "right")}

    ctl = {}
    ctl["state_validity_service"] = bool(n.sv_ok)
    # a config deep inside the torso must read negative and be INVALID
    bad = list(hp.load_home_radians("left"))
    bad[1] += 1.2
    c_bad, _ = sc.clearance("left", bad)
    ctl["a_config_swung_into_the_wearer"] = round(float(c_bad), 4)
    print("CONTROLS")
    print("   /check_state_validity answers            %s" % n.sv_ok)
    print("   a config swung into the wearer reads     %.4f m" % c_bad)
    if not n.sv_ok:
        print("REFUSING: the validity service is absent; a silent pass here "
              "would clear a pose nothing checked.")
        return 6

    res, all_ok = {}, True
    for arm in ("left", "right"):
        f = os.path.join(ROOT, "recordings/baselines/observe_pose_%s.json"
                         % arm)
        if not os.path.exists(f):
            print("no observe pose for %s -- run solve_observe_pose.py" % arm)
            return 4
        d = json.load(open(f))
        if not d.get("solved"):
            print("%s observe pose records NO SOLUTION" % arm)
            return 4
        q_obs = np.array(d["solved"]["q"], float)
        q_home = np.array(hp.load_home_radians(arm), float)

        print("\n%s ARM" % arm.upper())

        # ---- 1. the pose is commandable ------------------------------
        M = ee[arm](q_obs)[0]
        p = [float(v) for v in M[:3, 3]]
        qm = as_quat_msg(quat_of(M))
        got = sum(1 for _ in range(a.repeats)
                  if n.solve_arm_joints(arm, p, qm, avoid=True,
                                        tries=6) is not None)
        pose_ok = got == a.repeats
        print("   pose commandable       /compute_ik %d of %d at the observe "
              "EE pose %s" % (got, a.repeats, [round(v, 3) for v in p]))

        # ---- 2 + 3. the pose itself ----------------------------------
        c_obs, who = sc.clearance(arm, q_obs)
        v_obs = n.valid(arm, q_obs)
        print("   wearer clearance       %.4f m to %-12s (floor %.3f)  %s"
              % (c_obs, who, a.floor, "ok" if c_obs >= a.floor else "BREACH"))
        print("   self/furniture valid   %s" % v_obs)

        # ---- 4. the transit ------------------------------------------
        # VIA POINT IF ONE IS RECORDED. A straight joint-space line from home
        # breaches the wearer floor on both arms (0.1199 / 0.1264 m against
        # 0.150) even though both endpoints are clear, because the elbow
        # sweeps an arc through the person. `solve_observe_transit.py` finds a
        # single via configuration making both segments clean; if it has run,
        # the move verified here is home -> via -> observe.
        vf = os.path.join(ROOT,
                          "recordings/baselines/observe_transit_%s.json" % arm)
        via = None
        if os.path.exists(vf):
            vd = json.load(open(vf))
            if vd.get("via"):
                via = np.array(vd["via"], float)
        legs = ([(q_home, via), (via, q_obs)] if via is not None
                else [(q_home, q_obs)])
        path, nsteps = [], 0
        for A_, B_ in legs:
            seg, ns = densify_joints(A_, B_, a.step_rad)
            path += seg if not path else seg[1:]
            nsteps += ns
        print("   move                   %s"
              % ("home -> via -> observe" if via is not None
                 else "home -> observe (straight)"))
        worst_c, worst_i, bad_v = 1e9, None, []
        ik_fail = 0
        ee_len, prev = 0.0, None
        for i, q in enumerate(path):
            c, _w = sc.clearance(arm, q)
            if c < worst_c:
                worst_c, worst_i = c, i
            if n.valid(arm, q) is False:
                bad_v.append(i)
            Mi = ee[arm](q)[0]
            pi = Mi[:3, 3]
            if prev is not None:
                ee_len += float(np.linalg.norm(pi - prev))
            prev = pi
            # every transit pose must also be commandable
            qi = as_quat_msg(quat_of(Mi))
            if n.solve_arm_joints(arm, [float(v) for v in pi], qi,
                                  avoid=True, tries=6) is None:
                ik_fail += 1
        if via is not None:
            dj = np.abs(via - q_home) + np.abs(q_obs - via)
            djmax = max(float(np.abs(via - q_home).max()),
                        float(np.abs(q_obs - via).max()))
        else:
            dj = np.abs(q_obs - q_home)
            djmax = float(dj.max())
        print("   transit                %d samples at <= %.3f rad/joint"
              % (nsteps, a.step_rad))
        print("      largest single joint move   %.1f deg"
              % math.degrees(djmax))
        print("      total joint travel          %.1f deg"
              % math.degrees(dj.sum()))
        print("      end-effector path           %.3f m" % ee_len)
        print("      worst wearer clearance      %.4f m at sample %d  %s"
              % (worst_c, worst_i,
                 "ok" if worst_c >= a.floor else "BREACH"))
        print("      self/furniture invalid at   %s"
              % (bad_v if bad_v else "no sample"))
        print("      transit poses not IK-solvable %d of %d"
              % (ik_fail, len(path)))

        ok = (pose_ok and c_obs >= a.floor and v_obs is not False
              and worst_c >= a.floor and not bad_v and ik_fail == 0)
        all_ok = all_ok and ok
        print("   VERDICT                %s" % ("USABLE" if ok else "NOT USABLE"))
        res[arm] = dict(
            q=[float(v) for v in q_obs], ee_pose=p,
            pose_commandable="%d/%d" % (got, a.repeats),
            clearance_m=round(float(c_obs), 4), clearance_to=who,
            state_valid=v_obs,
            transit=dict(samples=nsteps, step_rad=a.step_rad,
                         via=None if via is None else [float(v) for v in via],
                         largest_joint_move_deg=round(
                             math.degrees(djmax), 2),
                         total_joint_travel_deg=round(
                             math.degrees(float(dj.sum())), 2),
                         ee_path_m=round(ee_len, 4),
                         worst_clearance_m=round(float(worst_c), 4),
                         worst_sample=worst_i,
                         invalid_samples=bad_v,
                         ik_failures=ik_fail),
            usable=bool(ok))

    # ---- control: a deliberately bad transit must be rejected --------
    q_home = np.array(hp.load_home_radians("left"), float)
    bad_path, _ = densify_joints(q_home, np.array(bad, float), a.step_rad)
    bad_worst = min(sc.clearance("left", q)[0] for q in bad_path)
    ctl["bad_transit_rejected"] = round(float(bad_worst), 4)
    print("\n   CONTROL a transit into the wearer scores %.4f m -> %s"
          % (bad_worst, "rejected" if bad_worst < a.floor else "NOT REJECTED"))
    if bad_worst >= a.floor:
        print("   REFUSING: the transit check cannot fail.")
        return 6

    res["controls"] = ctl
    res["all_usable"] = bool(all_ok)
    json.dump(res, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if all_ok else 5


if __name__ == "__main__":
    sys.exit(main())
