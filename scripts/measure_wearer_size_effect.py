#!/usr/bin/env python3
"""What changes when the wearer is a person instead of the mannequin.

    python3 scripts/measure_wearer_size_effect.py                 # needs a stack
    python3 scripts/measure_wearer_size_effect.py --ee-only       # no stack, weaker

THE QUESTION. Every clearance figure in this project is measured against ONE
body: a 300 mm upper arm, a 260 mm forearm, a 360 mm chest, arms hanging
rigidly at the sides. The 150 mm floor, the innermost safe columns of 0.325
(L) and 0.450 (R), T0's ~142 mm breach and T2's 150/147 are all statements
about that body. A different person is a different set of numbers, and until
the wearer's size became a variable there was no way to ask how different.

THE INSTRUMENT CHECK THAT SHAPED THIS FILE. The first version measured the
END-EFFECTOR POINT only, because that needs no IK -- and reported that the
centre is OPEN, with 0.19 m of clearance at |x| = 0. The repository has
measured the centre SHUT at 780 configurations, min |x| 0.300 (L) / 0.375 (R).
Both are correct and they are about different things: at 0.30 m forward the
HAND clears a 0.22 m deep torso easily, while the FOREARM and WRIST links
swing inboard and do not. A result that contradicts an earlier measurement is
an instrument check until the two are reconciled (CLAUDE.md), so the default
here is now the whole arm, through IK, exactly as the published number was.

`--ee-only` keeps the cheap version because it is genuinely useful -- every
one of the 28 predictive-avoidance refusals names `end_effector_link` -- but
it prints what it is and refuses to answer the centre question.
"""
import argparse
import json
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
sys.path.insert(0, os.path.join(WS, "scripts"))

from srl_teleop import wearer_posture as WP                  # noqa: E402
from srl_teleop.clearance import point_clearance, DISTAL_LINKS  # noqa: E402

FLOOR = 0.150
IK_COLUMN = {"left": 0.325, "right": 0.450}    # quoted from the baselines


def named_prims(size, posture="down"):
    return [(n, k, d, c) for n, k, d, c, _r in WP.wearer_model(posture, size)]


def clearance_at(p, prims):
    best, who = float("inf"), None
    for name, kind, dims, ctr in prims:
        d = point_clearance(p, [(kind, dims, ctr)])
        if d < best:
            best, who = d, name
    return best, who


def arm_clearance(points, prims):
    """Worst clearance over a whole set of arm link points."""
    best, who = float("inf"), None
    for p in points:
        d, w = clearance_at(p, prims)
        if d < best:
            best, who = d, w
    return best, who


# ---------------------------------------------------------------- with IK
class Rig:
    """IK for a target point, then FK for every distal link."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from moveit_msgs.srv import GetPositionIK
        import srl_fk
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("wearer_size_effect")
        self.cli = self.node.create_client(GetPositionIK, "/compute_ik")
        if not self.cli.wait_for_service(timeout_sec=20.0):
            raise SystemExit(
                "no /compute_ik -- start the sim stack first:\n"
                "    bash scripts/run_teleop.sh gate:=false\n"
                "or run with --ee-only, which needs nothing but answers a "
                "weaker question.")
        self.fk = srl_fk.FK()
        self.GetPositionIK = GetPositionIK
        # THE ANCHOR THE TASKS ACTUALLY SEND. Solving at a different wrist
        # orientation measures a different robot -- `measure_what_binds` was
        # reading the HOME wrist, 32.26 deg from the anchor, and every
        # reachability sweep in the repository went through it.
        import importlib
        try:
            ct = importlib.import_module("clip_tasks")
        except Exception:                                     # noqa: BLE001
            sys.path.insert(0, os.path.join(
                WS, "src/srl_experiments/experiments/abc"))
            ct = importlib.import_module("clip_tasks")
        self.orient = getattr(ct, "WORKSPACE_ORIENT", None)

    def solve(self, arm, xyz, tries=3):
        from geometry_msgs.msg import PoseStamped
        req = self.GetPositionIK.Request()
        req.ik_request.group_name = "%s_arm" % arm
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        q = self.orient
        if q is not None:
            ps.pose.orientation.x, ps.pose.orientation.y = float(q[0]), float(q[1])
            ps.pose.orientation.z, ps.pose.orientation.w = float(q[2]), float(q[3])
        else:
            ps.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = ps
        for _ in range(tries):
            fut = self.cli.call_async(req)
            self.rclpy.spin_until_future_complete(self.node, fut,
                                                  timeout_sec=4.0)
            r = fut.result()
            if r is None or r.error_code.val != 1:
                continue
            names = list(r.solution.joint_state.name)
            pos = list(r.solution.joint_state.position)
            q7 = []
            for i in range(7):
                nm = "%s_joint_%d" % (arm, i + 1)
                if nm not in names:
                    q7 = []
                    break
                q7.append(pos[names.index(nm)])
            if len(q7) == 7:
                return q7
        return None

    def link_points(self, arm, q7):
        return [tuple(float(c) for c in p)
                for p in self.fk.points(arm, q7, list(DISTAL_LINKS))]

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", nargs="*",
                    default=["mannequin", "measured_adult"])
    ap.add_argument("--postures", nargs="*", default=["down", "folded"])
    ap.add_argument("--y", type=float, default=0.30)
    ap.add_argument("--z", type=float, default=1.10)
    ap.add_argument("--step", type=float, default=0.025)
    ap.add_argument("--x-max", type=float, default=0.80)
    ap.add_argument("--repeats", type=int, default=3,
                    help="TRAC-IK restarts randomly; one call is one flip")
    ap.add_argument("--ee-only", action="store_true")
    ap.add_argument("--out", default=os.path.join(
        WS, "recordings/baselines/wearer_size_effect.json"))
    a = ap.parse_args(argv)

    print("\n== WHAT THE WEARER'S SIZE COSTS ==")
    print("floor %.3f m, work point %.2f m forward at z = %.2f"
          % (FLOOR, a.y, a.z))
    print("measuring: %s\n"
          % ("the END-EFFECTOR POINT ONLY -- not comparable with the "
             "published columns" if a.ee_only else
             "EVERY DISTAL ARM LINK, through IK, as the published columns were"))

    rig = None if a.ee_only else Rig()
    result = dict(floor=FLOOR, y=a.y, z=a.z, ik_column=IK_COLUMN,
                  mode="ee_only" if a.ee_only else "whole_arm", rows=[])

    print("INNERMOST COLUMN THAT CLEARS THE WEARER (|x|, metres)")
    print("  %-15s %-8s %8s %8s   %s"
          % ("size", "posture", "left", "right", "what binds"))
    for size in a.sizes:
        for posture in a.postures:
            prims = named_prims(size, posture)
            got, binds, clr = {}, {}, {}
            for arm in ("left", "right"):
                sgn = 1.0 if arm == "left" else -1.0
                x, found = 0.0, None
                while x <= a.x_max + 1e-9:
                    tgt = (sgn * x, a.y, a.z)
                    if a.ee_only:
                        d, who = clearance_at(tgt, prims)
                        ok = d >= FLOOR
                    else:
                        ok, d, who = True, float("inf"), None
                        for _ in range(a.repeats):
                            q = rig.solve(arm, tgt)
                            if q is None:
                                ok, who, d = False, "NO IK", None
                                break
                            dd, ww = arm_clearance(rig.link_points(arm, q),
                                                   prims)
                            if dd < d:
                                d, who = dd, ww
                        ok = ok and d is not None and d >= FLOOR
                    if ok:
                        found = (x, d, who)
                        break
                    x += a.step
                got[arm] = None if found is None else found[0]
                clr[arm] = None if found is None else found[1]
                binds[arm] = None if found is None else found[2]
            print("  %-15s %-8s %8s %8s   L:%s R:%s"
                  % (size, posture,
                     "--" if got["left"] is None else "%.3f" % got["left"],
                     "--" if got["right"] is None else "%.3f" % got["right"],
                     binds["left"], binds["right"]))
            result["rows"].append(dict(size=size, posture=posture,
                                       left=got["left"], right=got["right"],
                                       clear_left=clr["left"],
                                       clear_right=clr["right"],
                                       binds_left=binds["left"],
                                       binds_right=binds["right"]))

    print("\nWHAT CHANGED, mannequin -> measured_adult")
    base = {r["posture"]: r for r in result["rows"] if r["size"] == "mannequin"}
    worst = 0.0
    any_move = False
    for r in result["rows"]:
        if r["size"] == "mannequin":
            continue
        b = base.get(r["posture"])
        if not b:
            continue
        for arm in ("left", "right"):
            if b[arm] is None or r[arm] is None:
                print("  %-8s %-6s  %s -> %s" % (r["posture"], arm, b[arm],
                                                 r[arm]))
                any_move = True
                continue
            dx = r[arm] - b[arm]
            worst = max(worst, abs(dx))
            if abs(dx) >= 0.0049:
                any_move = True
                print("  %-8s %-6s  %.3f -> %.3f  (%+.0f mm)"
                      % (r["posture"], arm, b[arm], r[arm], 1000 * dx))
    if not any_move:
        print("  nothing moved by more than 5 mm")
    result["worst_shift_m"] = worst

    if not a.ee_only:
        # NOT COMPARED WITH THE PUBLISHED COLUMNS, and that is deliberate.
        #
        # The published 0.325 / 0.450 were certified over the WHOLE DENSIFIED
        # PATH at N = 10, and CLAUDE.md is explicit that a pose passing one IK
        # call is not reachable and that MARGINAL is not usable. This is a
        # single-point probe. The two are different quantities, and printing
        # AGREES or DIFFERS against them would compare a spot reading with a
        # certified floor -- the substitution the standing rule exists to
        # prevent. The first version of this file did print it, and duly
        # reported DIFFERS on three of four rows, which was a fact about the
        # comparison and not about the robot.
        #
        # What IS valid is the comparison this file exists for: the same
        # probe, the same operating point, the same repeats, with only the
        # BODY changed. That is printed above, and it has its own control --
        # the mannequin row must reproduce the shipped geometry exactly.
        print("\nAGAINST THE PUBLISHED COLUMNS (%.3f L / %.3f R): NOT COMPARED."
              % (IK_COLUMN["left"], IK_COLUMN["right"]))
        print("  Those were certified over the whole densified path at N=10;")
        print("  this is a single-point probe at N=%d. Different quantities."
              % a.repeats)
        print("  The valid comparison is mannequin vs person, same probe,")
        print("  same point, printed above.")
    else:
        print("\nThe centre question is NOT answered by --ee-only: the hand "
              "clears a 0.22 m torso at 0.30 m forward while the forearm and "
              "wrist do not. Run without --ee-only.")

    if rig:
        rig.close()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(result, open(a.out, "w"), indent=1)
    print("\nwrote %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
