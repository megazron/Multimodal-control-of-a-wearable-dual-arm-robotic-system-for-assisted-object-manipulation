#!/usr/bin/env python3
"""WHAT BINDS THE WORKSPACE, per direction, per arm, with the mechanism named.

    python3 scripts/measure_what_binds.py                  # the direction walk
    python3 scripts/measure_what_binds.py --heights        # band vs height
    python3 scripts/measure_what_binds.py --self-test      # controls only

WHY THIS EXISTS. "The arms barely move" has been answered four different ways
in this repository -- the bench, the table edge, the pinned wrist, the wearer
-- and each answer was true of the measurement that produced it and silent
about the others. A reachable-cell count says WHERE the boundary is and never
says WHAT PUT IT THERE, so every proposed fix has been a guess about the
mechanism.

THE LADDER. At a point the arm cannot reach, four solves in order, and the
FIRST one that succeeds names the binding constraint:

    collision-aware, furniture in scene      -> REACHABLE, nothing binds
    collision-aware, furniture REMOVED       -> FURNITURE binds
    collisions OFF, pinned orientation       -> a BODY binds (see below)
    collisions OFF, orientation FREE         -> ORIENTATION binds
    nothing succeeds                         -> KINEMATIC (arm length/limits)

"A BODY binds" is split further, because MoveIt reports a collision without
saying with what: the collision-free IK solution is measured against the
mount guard's own capsule model of the wearer. Negative clearance means the
metal is inside the person -- WEARER. Non-negative means the arm collided
with ITSELF or with a wearer pair the capsule model does not carry, and it is
reported as SELF/OTHER rather than as a wearer result nobody measured.

THE CLEARANCE FLOOR IS NOT IN THAT LADDER AND MUST NOT BE. It is a runtime
guard, not an IK constraint: /compute_ik will happily return a pose the
follower would then refuse to publish. So it is measured SEPARATELY, on the
solutions that DID solve, and reported as "the floor bites at N mm before IK
does" or "it does not bite at all inside the IK-reachable set". Conflating
the two is how a workspace gets widened by lowering a guard.

CONTROLS, and no report is printed if one fails. A ladder that cannot tell
furniture from wearer would classify everything as whichever rung it tests
first, and the map would look authoritative:

    1.6 m out                    must be KINEMATIC
    inside the wearer's torso    must be WEARER, with clearance < 0
    inside the table slab        must be FURNITURE
    a surveyed cell              must be REACHABLE

Control 3 is the one that matters. It is the only thing standing between this
script and a report that blames the wearer for the furniture.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import CollisionObject, PlanningScene   # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402
from moveit_msgs.srv import GetPositionFK                    # noqa: E402
from moveit_msgs.msg import RobotState                       # noqa: E402

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from srl_teleop import mount_guard_node as MG                # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/what_binds.json")
Z_WORK = CT.BENCH_TOP + 0.02          # 1.12, the work plane T1 runs on
STEP = 0.025
MAX_WALK = 0.70
CLEAR_FLOOR = 0.15                    # participant_safety_node's own value


# --------------------------------------------------------------- the scene
def _apply(node, cli, objs):
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = objs
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = time.time() + 20.0
    while time.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.spin(0.4)


def _remove(node, cli, ids):
    objs = []
    for i in ids:
        o = CollisionObject()
        o.id = i
        o.header.frame_id = "world"
        o.operation = CollisionObject.REMOVE
        objs.append(o)
    _apply(node, cli, objs)


class Rig:
    """One place that knows how to ask each of the four questions."""

    def __init__(self, node, scene="t1", repeats=3):
        self.n = node
        self.repeats = repeats
        self.calls = 0
        self.quat = {a: node.ee_quat(a) for a in ("left", "right")}
        self.cli = node.create_client(ApplyPlanningScene,
                                      "/apply_planning_scene")
        self.cli.wait_for_service(timeout_sec=20.0)
        self.fk = node.create_client(GetPositionFK, "/compute_fk")
        self.fk.wait_for_service(timeout_sec=20.0)
        import clip_scene as CS
        self.CS = CS
        tmp = CS.Scene.__new__(CS.Scene)
        CS.remove_furniture(node)
        self.objs = CS.Scene._collision_furniture(tmp, scene)
        self.ids = [o.id for o in self.objs]
        self.furniture = False
        self.set_furniture(True)

    def control_block(self, p, on):
        """A box placed ON a pose known to be reachable, then taken away.

        THE FURNITURE CONTROL HAS TO BE CONSTRUCTED, and the first version was
        not. It named a point inside the table slab -- which the pinned wrist
        cannot reach with the table gone either, so the ladder correctly fell
        through to ORIENTATION and the control failed for being wrong rather
        than for the ladder being wrong. A control whose ground truth is
        argued from the scene is not a control. This one is CONSTRUCTED: take
        a cell the ladder has just called REACHABLE, put a 0.24 m box on it,
        and the only thing that can have changed is the furniture.
        """
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        cid = "control_block"
        if not on:
            _remove(self.n, self.cli, [cid])
            self.ids = [i for i in self.ids if i != cid]
            self.objs = [o for o in self.objs if o.id != cid]
            return
        o = CollisionObject()
        o.id = cid
        o.header.frame_id = "world"
        o.operation = CollisionObject.ADD
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.24, 0.24, 0.24]
        po = Pose()
        po.position.x, po.position.y, po.position.z = (float(v) for v in p)
        po.orientation.w = 1.0
        o.primitives = [sp]
        o.primitive_poses = [po]
        self.objs.append(o)
        self.ids.append(cid)
        _apply(self.n, self.cli, [o])
        self.furniture = True

    def set_furniture(self, on):
        if on == self.furniture:
            return
        if on:
            _apply(self.n, self.cli, self.objs)
        else:
            _remove(self.n, self.cli, self.ids)
        self.furniture = on

    # ------------------------------------------------------- the four rungs
    def solve(self, arm, p, quat=None, avoid=True, k=None):
        q = self.quat[arm] if quat is None else quat
        for _ in range(self.repeats if k is None else k):
            self.calls += 1
            if not self.n.solve(arm, list(p), q, avoid=avoid, tries=6):
                return False
        return True

    def solve_joints(self, arm, p, avoid=True):
        """THIS ARM'S seven, by name -- never positions[:7].

        The first version of this method sliced the response, which is the
        LEFT arm's joints for every query. The right arm therefore measured
        its clearance from the left arm sitting at home: 80 cells, ONE
        distinct value, 0.1610 m, in a map whose left half varied from
        -0.003 to 0.161. A single value across a whole map is the "zero
        variance" row of CLAUDE.md's table and it is why this was caught
        before the number was used, rather than after.
        """
        self.calls += 1
        return self.n.solve_arm_joints(arm, list(p), self.quat[arm],
                                       avoid=avoid, tries=6)

    def orient_free(self, arm, p, avoid=False):
        """Any of a fan of approach orientations, including straight down."""
        for q in self._fan(arm):
            self.calls += 1
            if self.n.solve(arm, list(p), q, avoid=avoid, tries=4):
                return True
        return False

    def _fan(self, arm):
        out = [self.quat[arm]]
        base = self.quat[arm]
        for yaw in np.linspace(-math.pi, math.pi, 8, endpoint=False):
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            q = Quaternion()
            q.x = base.x * cy - base.y * sy
            q.y = base.x * sy + base.y * cy
            q.z = base.z * cy + base.w * sy
            q.w = base.w * cy - base.z * sy
            out.append(q)
        # straight down, and down yawed -- the TOP-DOWN hand this project has
        # priced twice and never adopted, carried here so "orientation binds"
        # can name which orientation would have worked.
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            h = math.pi / 2
            qd = Quaternion(x=math.sin(h / 1.0) * 0.0, y=0.0, z=0.0, w=1.0)
            # tool +z down = rotate -90 deg about world x, then yaw about z
            a2 = -math.pi / 2
            qx = (math.sin(a2 / 2), 0.0, 0.0, math.cos(a2 / 2))
            qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            x1, y1, z1, w1 = qz
            x2, y2, z2, w2 = qx
            qd.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            qd.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            qd.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            qd.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            out.append(qd)
        return out

    # ------------------------------------------------- the geometric wearer
    def clearance(self, arm, joints):
        """Worst arm-to-wearer distance for a joint vector, mount-guard model.

        THE SAME MODEL THE GUARD RUNS, on purpose. /check_state_validity is
        not the authority here: the proximal wearer pairs are SRDF-excluded,
        so MoveIt can call a pose valid with the tube inside the torso.
        """
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, ln) for ln in MG.CHAIN]
        rs = RobotState()
        rs.joint_state.name = self.n.names(arm)
        rs.joint_state.position = [float(v) for v in joints[:7]]
        req.robot_state = rs
        fut = self.fk.call_async(req)
        end = time.time() + 5.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.n, timeout_sec=0.002)
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None, None
        pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
               for ps in res.pose_stamped]
        worst, who = 1e9, None
        for a, b in zip(pts, pts[1:]):
            for k in range(MG.SAMPLES + 1):
                t = k / float(MG.SAMPLES)
                p = [a[i] + (b[i] - a[i]) * t for i in range(3)]
                for name, kind, prm, ctr in MG.WEARER:
                    d = MG.dist_point(p, kind, prm, ctr) - MG.TUBE_R
                    if d < worst:
                        worst, who = d, name
        return worst, who

    # ------------------------------------------------------- the classifier
    def classify(self, arm, p):
        """The ladder. Returns (verdict, detail, clearance_m, who)."""
        self.set_furniture(True)
        j = self.solve_joints(arm, p, avoid=True)
        if j is not None and self.solve(arm, p, avoid=True):
            c, who = self.clearance(arm, j)
            return "REACHABLE", "", c, who
        self.set_furniture(False)
        if self.solve(arm, p, avoid=True):
            self.set_furniture(True)
            return "FURNITURE", "the scene blocks it; free with furniture out", \
                None, None
        jf = self.solve_joints(arm, p, avoid=False)
        self.set_furniture(True)
        if jf is not None:
            c, who = self.clearance(arm, jf)
            if c is not None and c < 0.0:
                return "WEARER", "arm inside the wearer by %.0f mm at %s" % (
                    -c * 1000.0, who), c, who
            return "SELF/OTHER", (
                "a collision, but NOT the capsule wearer model "
                "(clearance %s) -- self-collision or an SRDF pair"
                % ("None" if c is None else "%.3f m" % c)), c, who
        if self.orient_free(arm, p, avoid=False):
            return "ORIENTATION", \
                "reachable only with the wrist unpinned", None, None
        return "KINEMATIC", "no configuration reaches it at all", None, None


# ------------------------------------------------------------------ walks
DIRS = {
    "forward (+y)":  (0.0, 1.0, 0.0),
    "back (-y)":     (0.0, -1.0, 0.0),
    "inboard":       None,               # toward x = 0, sign per arm
    "outboard":      None,               # away from x = 0
    "up (+z)":       (0.0, 0.0, 1.0),
    "down (-z)":     (0.0, 0.0, -1.0),
}


def _dirvec(name, arm):
    if name == "inboard":
        return (-1.0 if arm == "left" else 1.0, 0.0, 0.0)
    if name == "outboard":
        return (1.0 if arm == "left" else -1.0, 0.0, 0.0)
    return DIRS[name]


def walk(rig, arm, seed, name):
    """Step outward until the arm cannot work, then name what stopped it."""
    d = _dirvec(name, arm)
    last, last_c, last_who, n_ok = None, None, None, 0
    k = 0
    while k * STEP <= MAX_WALK:
        p = [seed[i] + d[i] * k * STEP for i in range(3)]
        v, why, c, who = rig.classify(arm, p)
        if v != "REACHABLE":
            return dict(direction=name, arm=arm, seed=list(seed),
                        last_ok=last, cells=n_ok,
                        reach_m=round(n_ok * STEP, 4) if n_ok else 0.0,
                        first_bad=[round(x, 4) for x in p],
                        binds=v, detail=why,
                        last_ok_clearance_m=(None if last_c is None
                                             else round(last_c, 4)),
                        last_ok_clearance_to=last_who)
        last = [round(x, 4) for x in p]
        last_c, last_who, n_ok = c, who, n_ok + 1
        k += 1
    return dict(direction=name, arm=arm, seed=list(seed), last_ok=last,
                cells=n_ok, reach_m=round(n_ok * STEP, 4),
                first_bad=None, binds="NOT BOUNDED WITHIN %.2f m" % MAX_WALK,
                detail="", last_ok_clearance_m=(None if last_c is None
                                                else round(last_c, 4)),
                last_ok_clearance_to=last_who)


# ---------------------------------------------------------------- controls
def controls(rig, seed):
    out = {}
    v, why, c, who = rig.classify("left", [1.60, 0.35, 1.15])
    out["far_1.6m"] = dict(want="KINEMATIC", got=v, detail=why)
    v, why, c, who = rig.classify("left", [0.05, -0.02, 1.22])
    out["inside_torso"] = dict(want="WEARER", got=v, detail=why,
                               clearance_m=None if c is None else round(c, 4))
    v, why, c, who = rig.classify("left", seed["left"])
    out["surveyed_cell"] = dict(want="REACHABLE", got=v, detail=why,
                                clearance_m=None if c is None else round(c, 4))
    # THE SAME POSE, WITH A BOX ON IT. Nothing else changes, so a verdict
    # other than FURNITURE is the ladder failing and not the scene.
    ee = CT.ee_for(seed["left"], "left")
    rig.control_block(ee, True)
    v, why, c, who = rig.classify("left", seed["left"])
    rig.control_block(None, False)
    out["box_on_a_good_cell"] = dict(want="FURNITURE", got=v, detail=why)
    ok = (out["far_1.6m"]["got"] == "KINEMATIC"
          and out["inside_torso"]["got"] == "WEARER"
          and out["box_on_a_good_cell"]["got"] == "FURNITURE"
          and out["surveyed_cell"]["got"] == "REACHABLE")
    return out, ok


# ------------------------------------------------------------ height sweep
def height_band(rig, arm, z, x_probe=0.35, y0=0.0, y1=0.70, dy=0.025):
    x = x_probe if arm == "left" else -x_probe
    good = []
    y = y0
    while y <= y1 + 1e-9:
        if rig.solve(arm, [x, y, z]):
            good.append(round(y, 4))
        y += dy
    if not good:
        return None, None, 0
    return min(good), max(good), len(good)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--scene", default="t1")
    ap.add_argument("--z", type=float, default=Z_WORK)
    ap.add_argument("--heights", action="store_true",
                    help="band vs work height instead of the direction walk")
    ap.add_argument("--table-heights", action="store_true",
                    help="MOVE the table and probe 20 mm above its top")
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
            print("REFUSING: %s arm %.4f rad from home (joint_%d)."
                  % (arm, w, j))
            return 3
    rig = Rig(n, a.scene, a.repeats)
    if any(q is None for q in rig.quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    # The seed must be a cell the SURVEY says works, per arm, so the walk
    # starts inside the region rather than at a point nobody measured.
    seed = {"left": [0.400, 0.175, a.z], "right": [-0.400, 0.175, a.z]}
    ctl, ok = controls(rig, seed)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-14s want %-10s got %-11s %s"
              % (k, v["want"], v["got"], v.get("detail", "")[:60]))
    if not ok:
        print("\nREFUSING TO REPORT: a control failed. The ladder cannot be "
              "trusted to tell these apart.")
        n.destroy_node()
        rclpy.shutdown()
        return 6
    if a.self_test:
        print("\nself-test only: the ladder separates all four.")
        n.destroy_node()
        rclpy.shutdown()
        return 0

    res = dict(z=a.z, scene=a.scene, repeats=a.repeats, step=STEP,
               controls=ctl, seed=seed)

    if a.heights or a.table_heights:
        rows = {}
        zs = [round(0.90 + 0.05 * i, 3) for i in range(9)]     # 0.90..1.30
        if a.table_heights:
            print("\nBAND vs TABLE HEIGHT -- the table is MOVED and the probe "
                  "sits 20 mm above its top")
        else:
            print("\nBAND vs WORK HEIGHT -- nothing is moved but the probe")
        for z in zs:
            if a.table_heights:
                objs = []
                for o in rig.objs:
                    import copy
                    o2 = copy.deepcopy(o)
                    if o2.id == "table":
                        for pp in o2.primitive_poses:
                            pp.position.z += (z - rig.CS.TABLE_TOP)
                    objs.append(o2)
                _remove(n, rig.cli, rig.ids)
                _apply(n, rig.cli, objs)
                rig.furniture = True
                probe_z = z + 0.02
            else:
                probe_z = z
            row = {}
            for arm in ("left", "right"):
                lo, hi, k = height_band(rig, arm, probe_z)
                row[arm] = dict(y_min=lo, y_max=hi, cells=k)
            rows["%.2f" % z] = row
            print("   %s %.2f  probe z=%.3f   L y %s..%s (%d)   R y %s..%s (%d)"
                  % ("table" if a.table_heights else "work ", z, probe_z,
                     "----" if row["left"]["y_min"] is None
                     else "%.3f" % row["left"]["y_min"],
                     "----" if row["left"]["y_max"] is None
                     else "%.3f" % row["left"]["y_max"], row["left"]["cells"],
                     "----" if row["right"]["y_min"] is None
                     else "%.3f" % row["right"]["y_min"],
                     "----" if row["right"]["y_max"] is None
                     else "%.3f" % row["right"]["y_max"],
                     row["right"]["cells"]))
        res["table_heights" if a.table_heights else "work_heights"] = rows
        if a.table_heights:
            _remove(n, rig.cli, rig.ids)
            _apply(n, rig.cli, rig.objs)
    else:
        print("\nWHAT BINDS, PER DIRECTION  (seed = a surveyed cell, %.0f mm "
              "steps, N=%d)" % (STEP * 1000, a.repeats))
        walks = []
        for arm in ("left", "right"):
            print("\n  %s arm, seed %s" % (arm.upper(), seed[arm]))
            for name in ("forward (+y)", "back (-y)", "inboard", "outboard",
                         "up (+z)", "down (-z)"):
                w = walk(rig, arm, seed[arm], name)
                walks.append(w)
                print("    %-13s %5.3f m to %-24s BINDS: %-11s %s"
                      % (name, w["reach_m"], str(w["last_ok"]), w["binds"],
                         w["detail"][:52]))
                if w["last_ok_clearance_m"] is not None:
                    print("                  clearance at the last reachable "
                          "pose %.4f m to %s  (floor %.2f)"
                          % (w["last_ok_clearance_m"],
                             w["last_ok_clearance_to"], CLEAR_FLOOR))
        res["walks"] = walks

    res["ik_calls"] = rig.calls
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
