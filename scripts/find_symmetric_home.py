#!/usr/bin/env python3
"""A HOME POSE THAT LOOKS RIGHT: elbows down and in, hands in front, symmetric.

    python3 scripts/sim_session.py --stack moveit -- \
        python3 scripts/find_symmetric_home.py

WHAT IS WRONG WITH THE ONE WE HAVE, measured from TF and confirmed in the
render (`recordings/baselines/home_render.json`):

    hands        at |x| = 0.550, so 1.10 m apart against a 0.36 m torso
    elbows       left 0.0793 m below its shoulder and 0.188 m OUTBOARD of its
                 own hand; right 0.2715 m below. Winged out, not down and in
    wrists       level, and that part was always true: tool axis -0.01 deg
    symmetry     the HANDS mirror to 0.0000 m and the FOREARMS to 0.2826 m,
                 which is how a pose gets reported as symmetric while looking
                 nothing of the sort

So the two things to fix are the hand SEPARATION and the elbow. This searches
for both at once, with the two hand targets constructed as exact mirrors so
symmetry is a property of the QUESTION rather than something hoped for in the
answer, and then reports how symmetric the ARMS came out, which is a different
matter: the two arms are identical hardware on mirrored mounts, so a mirrored
target does not guarantee a mirrored posture.

THE ELBOW IS CHOSEN, NOT ACCEPTED. `/compute_ik` returns one solution per call
and seeds from the live state, so asking once gets whichever branch that seed
falls into. Each target is asked many times from a fan of seeds spread over
joints 1, 2 and 4 -- the joints that actually raise and lower the elbow -- and
the solution kept is the one with the LOWEST elbow that still clears the
wearer. `docs/system/findings.md` records that seed sampling does not move the
elbow much when the EE pose is pinned mid-motion; that is a different question
from this one, where the target itself is being chosen.

CONTROLS, and there is no report without them:

    the CURRENT home reproduces     the shipped pose must come back with the
                                    clearance and elbow heights TF just
                                    measured, or the scorer is not measuring
                                    the same thing the render showed
    a hand target inside the torso  must find nothing that clears the floor
    the seed fan finds MORE THAN    if every seed returns one elbow height the
    ONE elbow height                fan is not searching, and "the lowest
                                    elbow" means nothing
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver                              # noqa: E402
from srl_teleop import mount_guard_node as MG                      # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/symmetric_home.json")
FLOOR = 0.15
SHOULDER = "shoulder_link"
ELBOW = "forearm_link"
CHAIN = MG.CHAIN


def level_forward_quat():
    """Tool axis along +y, level. The wrist orientation the pose already has.

    Read off the shipped home rather than constructed from scratch: it is
    already exactly what "wrists level and forward" asks for, and rebuilding
    it by hand is a chance to get a sign wrong for no gain.
    """
    return None


class Finder(Solver):
    def __init__(self):
        super().__init__()
        self.fk = self.create_client(
            __import__("moveit_msgs.srv", fromlist=["GetPositionFK"])
            .GetPositionFK, "/compute_fk")
        self.fk.wait_for_service(timeout_sec=25.0)
        self.calls = 0

    def fk_points(self, arm, q, links):
        """World positions of `links` for joint vector q, from /compute_fk."""
        from moveit_msgs.srv import GetPositionFK
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        import time as _t
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, ln) for ln in links]
        js = JointState()
        js.name = self.names(arm)
        js.position = list(q)
        rs = RobotState()
        rs.joint_state = js
        req.robot_state = rs
        fut = self.fk.call_async(req)
        end = _t.time() + 5.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.002)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        return [np.array([p.pose.position.x, p.pose.position.y,
                          p.pose.position.z]) for p in res.pose_stamped]

    def clearance_of(self, arm, q):
        """Worst clearance over the whole capsule chain, and which part."""
        pts = self.fk_points(arm, q, CHAIN)
        if pts is None:
            return None, None
        worst, who = 1e9, None
        for a, b in zip(pts, pts[1:]):
            for k in range(MG.SAMPLES + 1):
                t = k / float(MG.SAMPLES)
                p = [a[i] + (b[i] - a[i]) * t for i in range(3)]
                for nm, kind, prm, ctr, rpy in MG.WEARER:
                    d = MG.dist_point(p, kind, prm, ctr, rpy) - MG.TUBE_R
                    if d < worst:
                        worst, who = d, nm
        return worst, who

    def solve_ik(self, arm, xyz, quat, seed, avoid=True):
        # NOT `ik`: Solver.ik is the /compute_ik CLIENT, and a
        # method of the same name shadows it silently until
        # something calls it and gets 'Client object is not
        # callable'.
        from moveit_msgs.msg import PositionIKRequest, RobotState
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        import time as _t
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = "%s_arm" % arm
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) for v in xyz)
        # `Solver.ee_quat` hands back a geometry_msgs Quaternion, not a
        # four-tuple. Assign the message.
        ps.pose.orientation = quat
        r.pose_stamped = ps
        r.ik_link_name = "%s_end_effector_link" % arm
        r.timeout.sec = 1
        r.avoid_collisions = avoid
        js = JointState()
        js.name = self.names(arm)
        js.position = list(seed)
        rs = RobotState()
        rs.joint_state = js
        r.robot_state = rs
        req.ik_request = r
        self.calls += 1
        fut = self.ik.call_async(req)
        end = _t.time() + 5.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.002)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        got = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [got[k] for k in self.names(arm)]


def seed_fan(base, n_per=5):
    """Seeds spread over joints 1, 2 and 4, the ones that move the elbow.

    Joint 3 and 6 were the fan in `predictive_avoidance`, chosen there to keep
    a FIXED EE pose while hunting the null space. Here the EE pose is the thing
    being chosen, so the fan is over the shoulder and elbow pitches, which is
    where elbow height lives.
    """
    out = [list(base)]
    for j in (0, 1, 3):
        for k in range(1, n_per + 1):
            for sgn in (1, -1):
                s = list(base)
                s[j] += sgn * 0.45 * k
                out.append(s)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, nargs="*", default=None)
    ap.add_argument("--y", type=float, nargs="*", default=[0.30, 0.36, 0.42])
    ap.add_argument("--z", type=float, nargs="*", default=[1.12, 1.18, 1.24])
    ap.add_argument("--floor", type=float, default=FLOOR)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    xs = a.x if a.x else [round(0.15 + 0.025 * i, 3) for i in range(19)]

    rclpy.init()
    n = Finder()
    n.spin(10.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2

    home = {arm: [n.js.get(k, 0.0) for k in n.names(arm)]
            for arm in ("left", "right")}
    quat = {}
    for arm in ("left", "right"):
        quat[arm] = n.ee_quat(arm)
        if quat[arm] is None:
            print("no TF for the %s end effector, so the wrist orientation "
                  "being searched at is unknown. Refusing." % arm)
            return 4

    ctl = {}
    # the shipped home, scored by THIS scorer
    shipped = {}
    for arm in ("left", "right"):
        pts = n.fk_points(arm, home[arm], [SHOULDER, ELBOW, "end_effector_link"])
        c, who = n.clearance_of(arm, home[arm])
        shipped[arm] = dict(
            elbow_below_shoulder_m=None if pts is None
            else round(float(pts[0][2] - pts[1][2]), 4),
            hand=None if pts is None else [round(float(v), 4) for v in pts[2]],
            clearance_m=None if c is None else round(c, 4), to=who)
    ctl["shipped_home_reproduces"] = dict(
        want="left elbow 0.0793 below, right 0.2715, clearance 0.1610",
        got=json.dumps(shipped))

    print("WHAT THE SHIPPED HOME SCORES, under this scorer")
    for arm in ("left", "right"):
        d = shipped[arm]
        print("   %-5s elbow %+.4f m below the shoulder, hand %s, "
              "clearance %s to %s"
              % (arm, d["elbow_below_shoulder_m"] or 0.0, d["hand"],
                 d["clearance_m"], d["to"]))

    def all_at(arm, xyz):
        """EVERY solution that clears the floor, not just the lowest elbow.

        Picking the lowest elbow per arm INDEPENDENTLY is what left the two
        arms 0.2166 m from being mirror images: the left arm's lowest-elbow
        branch and the right arm's are simply different branches. Symmetry is
        a property of the PAIR, so the pair has to be chosen together, and
        that needs the whole candidate set rather than each arm's own winner.
        """
        out = []
        for seed in seed_fan(home[arm]):
            q = n.solve_ik(arm, xyz, quat[arm], seed, avoid=True)
            if q is None:
                continue
            pts = n.fk_points(arm, q, [SHOULDER, ELBOW])
            if pts is None:
                continue
            c, who = n.clearance_of(arm, q)
            if c is None or c < a.floor:
                continue
            out.append(dict(q=[round(v, 6) for v in q],
                            elbow_below_shoulder_m=round(
                                float(pts[0][2] - pts[1][2]), 4),
                            elbow=[round(float(v), 4) for v in pts[1]],
                            clearance_m=round(c, 4), to=who))
        # DE-DUPLICATE. The fan returns the same branch from several seeds and
        # a pair search over duplicates just does the same comparison twice.
        seen, uniq = set(), []
        for r in out:
            k = tuple(round(v, 3) for v in r["q"])
            if k in seen:
                continue
            seen.add(k)
            uniq.append(r)
        return uniq

    def best_at(arm, xyz):
        """Lowest elbow that clears the floor, over the seed fan.

        The clearance evaluation walks nine capsule segments against twelve
        wearer primitives and is by far the most expensive step, so the
        candidates are collected first, sorted by elbow drop, and scored from
        the best downwards -- the first one that clears the floor is the
        answer and the rest are never evaluated.
        """
        cands, heights = [], set()
        for seed in seed_fan(home[arm]):
            q = n.solve_ik(arm, xyz, quat[arm], seed, avoid=True)
            if q is None:
                continue
            pts = n.fk_points(arm, q, [SHOULDER, ELBOW])
            if pts is None:
                continue
            drop = float(pts[0][2] - pts[1][2])
            heights.add(round(drop, 3))
            cands.append((drop, q, pts[1]))
        for drop, q, elbow in sorted(cands, key=lambda t: -t[0]):
            c, who = n.clearance_of(arm, q)
            if c is None or c < a.floor:
                continue
            return dict(q=[round(v, 6) for v in q],
                        elbow_below_shoulder_m=round(drop, 4),
                        elbow=[round(float(v), 4) for v in elbow],
                        clearance_m=round(c, 4), to=who), len(heights)
        return None, len(heights)

    # ------------------------------------------------------------- controls
    inside, _ = best_at("left", [0.05, 0.0, 1.22])
    ctl["target_inside_the_torso_finds_nothing"] = dict(
        want="None", got="None" if inside is None else "FOUND")

    _b, nh = best_at("left", [0.45, 0.36, 1.18])
    ctl["seed_fan_finds_more_than_one_elbow"] = dict(want="> 1", got=nh)

    print("\nCONTROLS")
    for k, v in ctl.items():
        print("   %-38s want %-26s got %s"
              % (k, v["want"][:26], str(v["got"])[:90]))
    if inside is not None or nh <= 1:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    # ---------------------------------------------------------------- sweep
    print("\nHOW FAR IN CAN THE HANDS COME, with the wrist level and forward")
    print("   %-6s %-6s %-6s %-9s %-9s %-9s %s"
          % ("x", "y", "z", "L elbow", "R elbow", "clear", "mirror residual"))
    rows = []
    for z in a.z:
        for y in a.y:
            for x in xs:
                bl, _ = best_at("left", [x, y, z])
                br, _ = best_at("right", [-x, y, z])
                if bl is None or br is None:
                    rows.append(dict(x=x, y=y, z=z, ok=False))
                    continue
                pl = n.fk_points("left", bl["q"], CHAIN)
                pr = n.fk_points("right", br["q"], CHAIN)
                res = None
                if pl is not None and pr is not None:
                    res = max(float(np.linalg.norm(
                        np.array([-p[0], p[1], p[2]]) - r))
                        for p, r in zip(pl, pr))
                rows.append(dict(
                    x=x, y=y, z=z, ok=True,
                    left=bl, right=br,
                    clearance_m=round(min(bl["clearance_m"],
                                          br["clearance_m"]), 4),
                    mirror_residual_m=None if res is None else round(res, 4)))
                print("   %-6.3f %-6.2f %-6.2f %+9.4f %+9.4f %9.4f %s"
                      % (x, y, z, bl["elbow_below_shoulder_m"],
                         br["elbow_below_shoulder_m"],
                         min(bl["clearance_m"], br["clearance_m"]),
                         "----" if res is None else "%.4f" % res))

    # ------------------------------------------------- the SYMMETRIC pair
    #
    # For every column that worked, take both arms' full candidate sets and
    # choose the pair whose links mirror best. Elbow drop is the tie-break,
    # not the objective: a pose that looks wrong symmetrically looks wrong
    # however low the elbow is.
    pairs = []
    for r in [x for x in rows if x.get("ok")]:
        xl, y, z = r["x"], r["y"], r["z"]
        ls = all_at("left", [xl, y, z])
        rs = all_at("right", [-xl, y, z])
        if not ls or not rs:
            continue
        best = None
        for cl in ls:
            pl = n.fk_points("left", cl["q"], CHAIN)
            if pl is None:
                continue
            for cr in rs:
                pr = n.fk_points("right", cr["q"], CHAIN)
                if pr is None:
                    continue
                resid = max(float(np.linalg.norm(
                    np.array([-p[0], p[1], p[2]]) - q2))
                    for p, q2 in zip(pl, pr))
                drop = min(cl["elbow_below_shoulder_m"],
                           cr["elbow_below_shoulder_m"])
                key = (round(resid, 4), -drop)
                if best is None or key < best[0]:
                    best = (key, cl, cr, resid, drop)
        if best is None:
            continue
        pairs.append(dict(x=xl, y=y, z=z,
                          mirror_residual_m=round(best[3], 4),
                          worst_elbow_drop_m=round(best[4], 4),
                          n_left=len(ls), n_right=len(rs),
                          left=best[1], right=best[2]))
        print("   PAIR x=%.3f  mirror residual %.4f m  elbows %+.4f / %+.4f  "
              "(%d x %d candidates)"
              % (xl, best[3], best[1]["elbow_below_shoulder_m"],
                 best[2]["elbow_below_shoulder_m"], len(ls), len(rs)))

    good = [r for r in rows if r.get("ok")]
    print("\n%d of %d targets are reachable by BOTH arms and clear the floor"
          % (len(good), len(rows)))
    if good:
        inner = min(r["x"] for r in good)
        print("   innermost hand column that works: |x| = %.3f  (torso "
              "half-width is 0.18)" % inner)
        best_sym = min(good, key=lambda r: (r["mirror_residual_m"] or 9e9))
        print("   most symmetric: x=%.3f y=%.2f z=%.2f, mirror residual "
              "%.4f m, elbows %+.4f / %+.4f"
              % (best_sym["x"], best_sym["y"], best_sym["z"],
                 best_sym["mirror_residual_m"] or -1,
                 best_sym["left"]["elbow_below_shoulder_m"],
                 best_sym["right"]["elbow_below_shoulder_m"]))
    json.dump(dict(controls=ctl, shipped=shipped, rows=rows, pairs=pairs,
                   floor=a.floor, ik_calls=n.calls),
              open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (n.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
