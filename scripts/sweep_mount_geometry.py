#!/usr/bin/env python3
"""WHERE WOULD THE MOUNTS HAVE TO BE FOR THE CENTRE TO BE REACHABLE?

    python3 scripts/sweep_mount_geometry.py --self-test
    python3 scripts/sweep_mount_geometry.py --grid
    python3 scripts/sweep_mount_geometry.py --cands "out+0.10,dx=0.10"

WHAT THIS IS AND WHAT IT IS NOT. `sweep_gap_vs_mount.py` prices ONE mount
change -- tilt about the world x axis -- and does it by rotating the TARGETS
instead of relaunching a stack. The same trick covers translation and yaw
exactly, and neither has ever been swept:

    mount moved by t          solve the target at p - t, then put the solved
                              link chain back at +t
    mount rotated by R        solve the target at R^-1 (p - o) + o, then put
                              the chain back at R (q - o) + o

Both are EXACT for kinematics -- it is the same rigid transform on both sides
-- and both are WRONG if the collision check is left to `move_group`, because
the wearer and the table do not move with the mount. So, exactly as in the
tilt sweep, IK is solved with collisions OFF and the wearer and the slab are
checked GEOMETRICALLY on the transformed chain, against the same capsule model
`mount_guard_node` runs.

WHAT IT DOES NOT MODEL, stated here and again in the output: MoveIt's mesh
collision, self-collision, and the gripper-versus-table contact (the slab test
stops at `bracelet_link`, for the reason `sweep_gap_vs_mount` records). A
candidate this file likes is a SHORTLIST ENTRY, not a result. It has to be
re-measured on a relaunched stack with the URDF actually changed, which is
what `scripts/apply_mount_candidate.py` is for.

THE CONTROL, and no report without it: at the identity transform the sweep
must reproduce the innermost column the relaunched-stack scorer reports for
the same table. If it does not, the transform machinery is wrong and every
other row is noise.
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

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from audit_scenario_reachability import densify               # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from search_t1_centre import frange, CUBE, STANDOFF, LIFT     # noqa: E402
from sweep_gap_vs_mount import (MOUNT, chain_points, sampled,  # noqa: E402
                                worst_wearer, TABLE_CHAIN_END)
from srl_teleop import mount_guard_node as MG                 # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/mount_geometry.json")

TABLE = dict(half_x=1.05, depth=0.62, thick=0.035)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def table_hits(pts, top, near_y, r=MG.TUBE_R):
    z0, z1 = top - TABLE["thick"], top
    y0, y1 = near_y, near_y + TABLE["depth"]
    for p in pts:
        if (abs(p[0]) <= TABLE["half_x"] + r and y0 - r <= p[1] <= y1 + r
                and z0 - r <= p[2] <= z1 + r):
            return True
    return False


class Cand:
    """One mount candidate: a translation and a rotation, mirrored per arm.

    `dx` is OUTBOARD (positive moves each mount away from the centreline),
    `dy` is FORWARD, `dz` is UP. `tilt` rotates about the world x axis, which
    is what `sweep_gap_vs_mount` swept -- negative pitches the arms forward
    over the wearer's shoulders. `yaw_in` rotates about the world z axis
    TOWARD the centreline, which is the "angle them inward" lever and has
    never been swept at all.
    """

    def __init__(self, name, dx=0.0, dy=0.0, dz=0.0, tilt=0.0, yaw_in=0.0):
        self.name = name
        self.dx, self.dy, self.dz = dx, dy, dz
        self.tilt, self.yaw_in = tilt, yaw_in

    def t(self, arm):
        s = 1.0 if arm == "left" else -1.0
        return np.array([s * self.dx, self.dy, self.dz])

    def R(self, arm):
        s = 1.0 if arm == "left" else -1.0
        # Mirroring a rotation through x = 0 negates pitch and yaw and keeps
        # roll, so an inboard yaw is +z on the right arm and -z on the left.
        return rot_z(math.radians(-s * self.yaw_in)) @ \
            rot_x(math.radians(self.tilt))

    def origin(self, arm):
        return MOUNT[arm]

    def to_target(self, arm, p):
        """World point -> the point the UNMOVED arm must solve for."""
        o, t, R = self.origin(arm), self.t(arm), self.R(arm)
        return list(R.T @ (np.asarray(p, float) - t - o) + o)

    def to_world(self, arm, q):
        """A solved link point -> where the MOVED arm would really put it."""
        o, t, R = self.origin(arm), self.t(arm), self.R(arm)
        return R @ (np.asarray(q, float) - o) + o + t

    def as_dict(self):
        return dict(name=self.name, dx=self.dx, dy=self.dy, dz=self.dz,
                    tilt=self.tilt, yaw_in=self.yaw_in)


def cell(node, fkc, arm, obj, q, pad_mid, cand, repeats, floor, top, near_y):
    pre, ee = GF.approach_path(obj, q, STANDOFF, pad_mid)
    up = [ee[0], ee[1], ee[2] + LIFT]
    wps = densify([pre, ee], 0.025) + densify([ee, up], 0.025)
    qm = as_msg(q)
    worst, who, seg = 1e9, None, None
    for w in wps:
        w2 = cand.to_target(arm, w)
        for _ in range(repeats):
            j = node.solve_arm_joints(arm, w2, qm, avoid=False, tries=8)
            if j is None:
                return False, None, None, None, "IK_FAIL"
            pts = chain_points(fkc, node, arm, j)
            if pts is None:
                return False, None, None, None, "FK_FAIL"
            pts = [cand.to_world(arm, p) for p in pts]
            if table_hits(sampled(pts, TABLE_CHAIN_END), top, near_y):
                return False, None, None, None, "TABLE"
            c, k, s = worst_wearer(pts)
            if c < worst:
                worst, who, seg = c, k, s
    return (worst >= floor), worst, who, seg, ("OK" if worst >= floor
                                               else "FLOOR")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-step", type=float, default=0.025)
    ap.add_argument("--table-top", type=float, default=T1.TABLE_TOP)
    ap.add_argument("--near-y", type=float, default=T1.TABLE_NEAR_Y)
    ap.add_argument("--half-x", type=float, default=1.05)
    ap.add_argument("--depth", type=float, default=0.62)
    ap.add_argument("--baseline-left", type=float, default=0.375)
    ap.add_argument("--baseline-right", type=float, default=0.275)
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--cands", default="")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    TABLE["half_x"], TABLE["depth"] = a.half_x, a.depth

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)."
                  % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    rig.set_furniture(False)          # collision is geometric here
    fk = FK(n)
    fkc = fk.cli
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    z_obj = round(a.table_top + CUBE / 2.0, 4)
    y = round(a.near_y + CUBE / 2.0, 4)
    xs = frange(0.0, a.x_max, a.x_step)

    def innermost(arm, cand):
        sgn = 1.0 if arm == "left" else -1.0
        first_why = None
        for x in xs:
            obj = [round(sgn * x, 4), y, z_obj]
            ok, c, who, seg, why = cell(n, fkc, arm, obj, T1.APPROACH[arm],
                                        pad_mid[arm], cand, a.repeats,
                                        a.floor, a.table_top, a.near_y)
            if first_why is None:
                first_why = why
            if ok:
                return dict(x=x, clearance=round(c, 4), wearer=who, link=seg,
                            centre_blocked_by=first_why)
        return dict(x=None, centre_blocked_by=first_why)

    print("CONTROL -- the identity transform must reproduce the "
          "relaunched-stack scorer")
    ident = Cand("identity")
    base = {arm: innermost(arm, ident) for arm in ("left", "right")}
    want = {"left": a.baseline_left, "right": a.baseline_right}
    okc = True
    for arm in ("left", "right"):
        got = base[arm]["x"]
        good = got is not None and abs(got - want[arm]) <= 0.051
        okc = okc and good
        print("   %-5s identity innermost = %s, planning-scene scorer said "
              "%.3f  %s" % (arm, "none" if got is None else "%.3f" % got,
                            want[arm], "PASS" if good else "FAIL"))
    if not okc:
        print("\nREFUSING TO REPORT: the transform machinery does not "
              "reproduce the baseline at identity.")
        json.dump(dict(refused=True, base=base, want=want),
                  open(a.out, "w"), indent=2)
        return 6
    if a.self_test:
        print("\nself-test only: the transform reproduces the baseline.")
        return 0

    cands = [ident]
    if a.grid:
        for v in (0.05, 0.10, 0.15, 0.20):
            cands.append(Cand("outboard +%.2f" % v, dx=v))
            cands.append(Cand("inboard  -%.2f" % v, dx=-v))
            cands.append(Cand("forward  +%.2f" % v, dy=v))
            cands.append(Cand("up       +%.2f" % v, dz=v))
            cands.append(Cand("down     -%.2f" % v, dz=-v))
        for d in (-45.0, -30.0, -15.0, 15.0, 30.0):
            cands.append(Cand("tilt %+.0f" % d, tilt=d))
        for d in (-45.0, -30.0, -15.0, 15.0, 30.0, 45.0):
            cands.append(Cand("yaw inboard %+.0f" % d, yaw_in=d))
    for spec in [s for s in a.cands.split(";") if s.strip()]:
        head, _, rest = spec.partition(",")
        kw = {}
        for part in rest.split(","):
            if not part.strip():
                continue
            k, _, v = part.partition("=")
            kw[k.strip()] = float(v)
        cands.append(Cand(head.strip(), **kw))

    print("\nMOUNT CANDIDATE vs INNERMOST REACHABLE |x| (table top %.3f, near "
          "edge %.3f)" % (a.table_top, a.near_y))
    rows = []
    for c in cands:
        row = c.as_dict()
        for arm in ("left", "right"):
            row[arm] = base[arm] if c is ident else innermost(arm, c)
        both = [row[arm]["x"] for arm in ("left", "right")]
        row["both_reach_centre"] = all(v is not None and v <= 0.1001
                                       for v in both)
        rows.append(row)
        print("   %-22s  left %-7s  right %-7s   %s"
              % (c.name,
                 "----" if both[0] is None else "%.3f" % both[0],
                 "----" if both[1] is None else "%.3f" % both[1],
                 "CENTRE" if row["both_reach_centre"] else
                 "centre: L %s / R %s" % (row["left"]["centre_blocked_by"],
                                          row["right"]["centre_blocked_by"])))

    json.dump(dict(tag=a.tag, cands=rows, floor=a.floor, repeats=a.repeats,
                   table=dict(top=a.table_top, near_y=a.near_y,
                              half_x=a.half_x, depth=a.depth),
                   control=dict(base=base, want=want),
                   models="IK collisions OFF; wearer and slab checked "
                          "geometrically on the TRANSFORMED chain; MoveIt "
                          "mesh, self-collision and gripper-vs-table NOT "
                          "modelled. A row here is a shortlist entry, not a "
                          "result."),
              open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
