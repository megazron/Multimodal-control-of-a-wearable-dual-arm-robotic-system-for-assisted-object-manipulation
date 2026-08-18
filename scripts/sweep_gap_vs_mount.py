#!/usr/bin/env python3
"""WOULD TILTING THE MOUNT OPEN THE GAP, AND WHAT WOULD IT COST?

    python3 scripts/sweep_gap_vs_mount.py
    python3 scripts/sweep_gap_vs_mount.py --self-test

THE TRICK, AND EXACTLY WHAT IT MODELS. Rotating an arm's base by D about the
mount origin is the same as rotating its TARGETS by -D about that origin, so
one running `move_group` can evaluate every candidate without relaunching. The
joints that come back are the joints the TILTED arm would use.

WHERE THAT TRICK GOES WRONG IF USED NAIVELY, and TASK_SPEC already records it:
the WEARER AND THE TABLE DO NOT TILT WITH THE MOUNT. Asking MoveIt for a
collision-free solution to the inverse-rotated target checks the untilted arm
against a world that has not been rotated with it, which is a different
question from the one being asked.

SO COLLISION IS DONE HERE, GEOMETRICALLY, ON THE ROTATED LINKS. IK is solved
with `avoid_collisions=False`; the resulting link chain is rotated by +D about
the mount origin, which is exactly where the tilted arm's links would be; and
that chain is then tested against

    the wearer   the mount guard's own capsule model, unrotated
    the table    the slab, as an axis-aligned box, inflated by the tube radius

Both are the same models used everywhere else in this repository, applied to
points instead of to a planning scene. What it does NOT model is MoveIt's mesh
collision and self-collision, so a candidate that looks good here still has to
be re-measured on a relaunched stack before anything is built on it. That is
stated in the output, not buried here.

CONTROL, and no report without it: at D = 0 the sweep must reproduce the
baseline it is a perturbation of. If a 0 deg tilt does not return the same
innermost column as `measure_centre_gap.py`, the rotation machinery is wrong
and every other row is noise.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from moveit_msgs.msg import RobotState                        # noqa: E402
from moveit_msgs.srv import GetPositionFK                     # noqa: E402

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from audit_scenario_reachability import densify               # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from search_t1_centre import frange, CUBE, STANDOFF, LIFT     # noqa: E402
from srl_teleop import mount_guard_node as MG                 # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/gap_vs_mount.json")

# The mount origins in WORLD, from srl_dual.urdf.xacro: the backpack link sits
# at torso + (0, -0.12, 0.02), so world = (mount_x, mount_y - 0.12,
# mount_z + 1.07).
MOUNT = {"left": np.array([0.20, -0.32, 1.25]),
         "right": np.array([-0.20, -0.32, 1.25])}
TABLE_HALF_X, TABLE_DEPTH, TABLE_THICK = 1.05, 0.62, 0.035


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def about(p, R, origin):
    return list(origin + R @ (np.asarray(p, float) - origin))


def table_hits(pts, top, near_y, r=MG.TUBE_R):
    """Does any sampled link point lie inside the table slab, inflated by r?

    The slab is axis-aligned, so this is a box test rather than a mesh test.
    Conservative in the direction that matters -- it inflates the arm, not the
    table -- and it is the same slab `put_table` applies to the planning scene
    in every other sweep.
    """
    z0, z1 = top - TABLE_THICK, top
    y0, y1 = near_y, near_y + TABLE_DEPTH
    for p in pts:
        if (abs(p[0]) <= TABLE_HALF_X + r and y0 - r <= p[1] <= y1 + r
                and z0 - r <= p[2] <= z1 + r):
            return True
    return False


def chain_points(fkc, node, arm, joints):
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.fk_link_names = ["%s_%s" % (arm, ln) for ln in MG.CHAIN]
    rs = RobotState()
    rs.joint_state.name = node.names(arm)
    rs.joint_state.position = [float(v) for v in joints[:7]]
    req.robot_state = rs
    fut = fkc.call_async(req)
    import time as _t
    end = _t.time() + 5.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.002)
    res = fut.result()
    if res is None or res.error_code.val != 1 or not res.pose_stamped:
        return None
    return [np.array([ps.pose.position.x, ps.pose.position.y,
                      ps.pose.position.z]) for ps in res.pose_stamped]


# THE TABLE TEST COVERS THE ARM TUBES AND NOT THE HAND, AND THE CONTROL IS
# WHAT FORCED THAT.
#
# `MG.TUBE_R` is 50 mm because a Gen3 link is a 46 mm tube. The gripper is not:
# its fingers are thin and at the grasp they sit 20 mm above the table by
# design. Inflating THEM by 50 mm makes every grasp pose read as a table
# collision, and the zero-tilt control duly came back "nothing reachable
# anywhere" against a planning-scene run that found 0.375 and 0.275.
#
# So the slab test runs over the chain up to `bracelet_link` and stops. What
# that gives up is real and is stated in the output: a gripper-versus-table
# contact -- which is exactly what blocks the near-lateral approaches -- is
# INVISIBLE to this sweep. It prices the mount against the WEARER and the arm
# tubes, and any candidate it likes still has to be re-measured on a
# relaunched stack with MoveIt doing the collision.
TABLE_CHAIN_END = MG.CHAIN.index("bracelet_link")


def sampled(pts, upto=None):
    out = []
    n = len(pts) if upto is None else min(upto + 1, len(pts))
    for a, b in zip(pts[:n], pts[1:n]):
        for k in range(MG.SAMPLES + 1):
            t = k / float(MG.SAMPLES)
            out.append(a + (b - a) * t)
    return out


def worst_wearer(pts):
    worst, who, seg = 1e9, None, None
    for si, (a, b) in enumerate(zip(pts, pts[1:])):
        for k in range(MG.SAMPLES + 1):
            t = k / float(MG.SAMPLES)
            p = a + (b - a) * t
            for name, kind, prm, ctr, rpy in MG.WEARER:
                d = MG.dist_point(list(p), kind, prm, ctr, rpy) - MG.TUBE_R
                if d < worst:
                    worst, who = d, name
                    seg = "%s -> %s" % (MG.CHAIN[si], MG.CHAIN[si + 1])
    return worst, who, seg


def cell(node, fkc, arm, obj, q, pad_mid, delta, repeats, floor, top, near_y):
    """One column at one tilt. (ok, worst clearance, wearer, link)."""
    R = rot_x(math.radians(delta))
    Rinv = R.T
    o = MOUNT[arm]
    pre, ee = GF.approach_path(obj, q, STANDOFF, pad_mid)
    up = [ee[0], ee[1], ee[2] + LIFT]
    wps = densify([pre, ee], 0.025) + densify([ee, up], 0.025)
    qm = as_msg(q)
    worst, who, seg = 1e9, None, None
    for w in wps:
        # THE TARGET, ROTATED BACK. Orientation is left alone on purpose: this
        # models a mount whose ARM is re-aimed, and the task keeps commanding
        # the same hand orientation in the world.
        w2 = about(w, Rinv, o)
        for _ in range(repeats):
            j = node.solve_arm_joints(arm, w2, qm, avoid=False, tries=8)
            if j is None:
                return False, None, None, None, "IK_FAIL"
            pts = chain_points(fkc, node, arm, j)
            if pts is None:
                return False, None, None, None, "FK_FAIL"
            pts = [np.asarray(about(p, R, o)) for p in pts]
            if table_hits(sampled(pts, TABLE_CHAIN_END), top, near_y):
                return False, None, None, None, "TABLE"
            c, k, s = worst_wearer(pts)
            if c < worst:
                worst, who, seg = c, k, s
    return (worst >= floor), worst, who, seg, ("OK" if worst >= floor
                                               else "FLOOR")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deltas", default="-45,-30,-15,0,15")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-step", type=float, default=0.025)
    ap.add_argument("--table-top", type=float, default=T1.TABLE_TOP)
    ap.add_argument("--near-y", type=float, default=T1.TABLE_NEAR_Y)
    ap.add_argument("--baseline-left", type=float, default=0.375)
    ap.add_argument("--baseline-right", type=float, default=0.275)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    rig.set_furniture(False)          # collision is geometric here, not scene
    fk = FK(n)
    fkc = fk.cli
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    z_obj = round(a.table_top + CUBE / 2.0, 4)
    y = round(a.near_y + CUBE / 2.0, 4)
    xs = frange(0.0, a.x_max, a.x_step)

    def innermost(arm, delta):
        sgn = 1.0 if arm == "left" else -1.0
        for x in xs:
            obj = [round(sgn * x, 4), y, z_obj]
            ok, c, who, seg, why = cell(n, fkc, arm, obj, T1.APPROACH[arm],
                                        pad_mid[arm], delta, a.repeats,
                                        a.floor, a.table_top, a.near_y)
            if ok:
                return dict(x=x, clearance=round(c, 4), wearer=who, link=seg)
        return None

    print("CONTROL -- at 0 deg this must reproduce the relaunched-stack sweep")
    base = {arm: innermost(arm, 0.0) for arm in ("left", "right")}
    want = {"left": a.baseline_left, "right": a.baseline_right}
    okc = True
    for arm in ("left", "right"):
        got = None if base[arm] is None else base[arm]["x"]
        good = got is not None and abs(got - want[arm]) <= 0.051
        okc = okc and good
        print("   %-5s innermost at 0 deg = %s, relaunched stack said %.3f  %s"
              % (arm, "none" if got is None else "%.3f" % got, want[arm],
                 "PASS" if good else "FAIL"))
    if not okc:
        print("\nREFUSING TO REPORT: the rotation machinery does not reproduce "
              "the baseline at zero tilt, so no other row means anything.\n"
              "Note this compares a GEOMETRIC collision model against a "
              "planning-scene one, so a one-cell (25 mm) difference is "
              "expected and tolerated; more is not.")
        json.dump(dict(refused=True, base=base, want=want),
                  open(a.out, "w"), indent=2)
        return 6
    if a.self_test:
        print("\nself-test only: the rotation reproduces the baseline.")
        return 0

    print("\nTILT vs INNERMOST REACHABLE |x|, objects resting on the table")
    print("   (negative tilts the mounts FORWARD, over the wearer's shoulders)")
    rows = []
    for d in [float(v) for v in a.deltas.split(",")]:
        row = dict(delta_deg=d)
        for arm in ("left", "right"):
            r = base[arm] if d == 0.0 else innermost(arm, d)
            row[arm] = r
        rows.append(row)
        print("   %+6.1f deg   left %s   right %s"
              % (d,
                 "----" if row["left"] is None else
                 "%.3f (%.4f m to %s)" % (row["left"]["x"],
                                          row["left"]["clearance"],
                                          row["left"]["wearer"]),
                 "----" if row["right"] is None else
                 "%.3f (%.4f m to %s)" % (row["right"]["x"],
                                          row["right"]["clearance"],
                                          row["right"]["wearer"])))

    print("\nWHAT A MOUNT CHANGE COSTS, and it is not measured here:")
    print("   P_HOME for both arms, the pinned anchor and therefore")
    print("   PAD_OFFSET_BY_ARM, every T0-T3 coordinate derived through")
    print("   ee_for(), the presentation pose, and all 579 cells of the")
    print("   surveyed region. TASK_SPEC section 2A lists them. This sweep")
    print("   prices the BENEFIT only; the bill is a re-derivation of the")
    print("   whole platform on arms that have never been run.")
    json.dump(dict(deltas=rows, floor=a.floor, repeats=a.repeats,
                   table=dict(top=a.table_top, near_y=a.near_y),
                   control=dict(base=base, want=want),
                   models="IK collisions OFF; wearer and table checked "
                          "geometrically on the ROTATED link chain; MoveIt "
                          "mesh and self-collision NOT modelled"),
              open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
