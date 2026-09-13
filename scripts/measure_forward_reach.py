#!/usr/bin/env python3
"""WHY IS EVERYTHING AT THE WEARER'S WAIST, and what moves it onto the table?

    python3 scripts/measure_forward_reach.py            # all four options
    python3 scripts/measure_forward_reach.py --only b   # one of a/b/c/d

Measures FORWARD REACH -- how far out in y the arm can work -- under four
interventions, so the layout can be moved onto a table on evidence rather
than on preference.

THE FINDING THIS EXISTS TO ANSWER. The surveyed band is y = 0.05..0.20 on the
work plane. The bench surface spans y = 0.245..0.630, so it overlaps the band
by EXACTLY ZERO; the table spans y = 0.100..0.720 at z = 0.950 and overlaps by
0.100 m. Every task coordinate was placed in cells that verified, and those
cells are all at the body. The layout is geometrically valid and physically
unusable.

FOUR MEASUREMENTS, and they are NOT equally trustworthy -- read the caveats:

  (a) MOUNT: rotate/translate the mount. Implemented as the inverse transform
      on the TARGET, which is exact for KINEMATICS and WRONG for wearer
      collision, because the wearer does not move with the mount. So (a) is
      an UPPER BOUND and any candidate needs a real URDF clearance check.
  (b) HEIGHT: sweep z. Nothing is transformed, so this one is exact.
  (c) WEARER: the same sweep with avoid_collisions off. The DIFFERENCE from
      (b) is the wearer's contribution, separated from the kinematics.
  (d) POSTURE: lean forward about the hips. Here the target transform is
      VALID FOR BOTH kinematics and collision, because the wearer rotates
      WITH the mount -- they are one rigid body when the wearer leans. That
      makes (d) the only intervention whose collision result can be believed
      without re-checking the URDF.

WHAT NONE OF THEM ESTABLISH. This is SINGLE-POSE IK at N=3: an envelope, not
a verified coordinate. docs/ENGINEERING_LOG.md's own corollary applies -- a pose that passes
one IK call is not a reachable pose, and a task coordinate needs N=10 over the
full densified PATH. Use these numbers to CHOOSE a layout; verify the layout
you choose with audit_scenario_reachability.py before recording anything.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))

from geometry_msgs.msg import Quaternion                      # noqa: E402
from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402

N = 3
Y0, Y1, DY = 0.00, 0.80, 0.025
X_PROBE = 0.35
Z_WORK = 1.12
OUT = os.path.join(WS, "recordings", "baselines",
                   "forward_reach.json")


def _Rx(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Rig:
    def __init__(self, node):
        self.n = node
        self.quat = {}
        self.mount = {}
        for arm in ("left", "right"):
            self.quat[arm] = node.ee_quat(arm)
            self.mount[arm] = self._tf("%s_base_link" % arm)
        # `or` on a numpy array raises -- ambiguous truth value. An explicit
        # None test, and the fallback is the wearer model's hip height.
        h = self._tf("hips")
        self.hip = np.array([0.0, 0.0, 0.95]) if h is None else h

    def _tf(self, frame):
        for _ in range(120):
            try:
                t = self.n.buf.lookup_transform("world", frame,
                                                rclpy.time.Time())
                tr = t.transform.translation
                return np.array([tr.x, tr.y, tr.z])
            except Exception:
                rclpy.spin_once(self.n, timeout_sec=0.05)
        return None

    def quats(self, arm, mode):
        """Orientations to try. `pinned` is one; `any` is a fan of them."""
        if mode == "pinned":
            return [self.quat[arm]]
        out = []
        for R in self._fan(mode):
            q = Quaternion()
            t = R[0, 0] + R[1, 1] + R[2, 2]
            if t > 0:
                sc = math.sqrt(t + 1.0) * 2
                v = ((R[2, 1] - R[1, 2]) / sc, (R[0, 2] - R[2, 0]) / sc,
                     (R[1, 0] - R[0, 1]) / sc, 0.25 * sc)
            else:
                i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
                if i == 0:
                    sc = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    v = (0.25 * sc, (R[0, 1] + R[1, 0]) / sc,
                         (R[0, 2] + R[2, 0]) / sc, (R[2, 1] - R[1, 2]) / sc)
                elif i == 1:
                    sc = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    v = ((R[0, 1] + R[1, 0]) / sc, 0.25 * sc,
                         (R[1, 2] + R[2, 1]) / sc, (R[0, 2] - R[2, 0]) / sc)
                else:
                    sc = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    v = ((R[0, 2] + R[2, 0]) / sc, (R[1, 2] + R[2, 1]) / sc,
                         0.25 * sc, (R[1, 0] - R[0, 1]) / sc)
            q.x, q.y, q.z, q.w = (float(c) for c in v)
            out.append(q)
        return out

    def _fan(self, mode):
        """Tool axis DOWN, then yawed about vertical; `any` adds tilts."""
        base = []
        down = np.array([0.0, 0.0, -1.0])
        yaws = [0.0] if mode == "topdown" else \
            [math.radians(d) for d in (0, 45, 90, 135, 180, 225, 270, 315)]
        tilts = [0.0] if mode == "topdown" else \
            [math.radians(d) for d in (0, 20, 40, 60)]
        for ti in tilts:
            ax = _Rx(ti) @ down
            for ya in yaws:
                zc = _Rz(ya) @ ax
                zc = zc / np.linalg.norm(zc)
                up = np.array([0.0, 1.0, 0.0])
                if abs(float(zc @ up)) > 0.95:
                    up = np.array([1.0, 0.0, 0.0])
                xc = np.cross(up, zc)
                xc = xc / np.linalg.norm(xc)
                base.append(np.column_stack([xc, np.cross(zc, xc), zc]))
        return base

    def reach(self, arm, z, xform=None, avoid=True, orient="pinned"):
        """Reachable y values at (x = +-X_PROBE, z), N=3 each.

        Returns (y_min, y_max, count). y_max is the number that matters:
        it is how far in FRONT of the wearer the arm can work.
        """
        x = X_PROBE if arm == "left" else -X_PROBE
        good = []
        y = Y0
        while y <= Y1 + 1e-9:
            p = np.array([x, y, z])
            q = self.quat[arm]
            if xform is not None:
                p, q = xform(arm, p, q)
            cands = [q] if orient == "pinned" else self.quats(arm, orient)
            hit = any(all(self.n.solve(arm, list(p), c, avoid=avoid, tries=6)
                          for _ in range(N)) for c in cands)
            if hit:
                good.append(round(y, 4))
            y += DY
        if not good:
            return (None, None, 0)
        return (min(good), max(good), len(good))


def mount_xform(rig, dR=None, dt=None):
    """Target transform equivalent to MOVING THE MOUNT.

    Moving a base by T is exactly equivalent, for reachability, to moving the
    target by T^-1. The wearer does NOT move, so this is kinematics only.
    """
    def f(arm, p, q):
        m = rig.mount[arm]
        pp = np.asarray(p, float)
        if dt is not None:
            s = np.array([dt[0] if arm == "left" else -dt[0], dt[1], dt[2]])
            pp = pp - s
        if dR is not None:
            pp = dR.T @ (pp - m) + m
        return pp, q
    return f


def lean_xform(rig, theta):
    """Target transform for the WEARER LEANING FORWARD about the hips.

    VALID FOR COLLISION TOO, unlike the mount transform: when the wearer
    leans, the mount and the wearer's own geometry rotate together as one
    rigid body, so rotating the target by the inverse reproduces both.
    """
    R = _Rx(theta)
    def f(arm, p, q):
        h = rig.hip
        return R.T @ (np.asarray(p, float) - h) + h, q
    return f


def shift_xform(dy):
    """The wearer standing CLOSER to (dy<0) or FURTHER from (dy>0) the table."""
    def f(arm, p, q):
        return np.asarray(p, float) - np.array([0.0, dy, 0.0]), q
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="abcd")
    ap.add_argument("--z-probe", type=float, default=0.97,
                    help="height to probe for the (A) mount measurement; "
                         "default is 20 mm above the table top")
    ap.add_argument("--furniture", default="",
                    help="comma list of furniture NAMES to publish; empty "
                         "means all of the scene's. Isolates WHICH piece "
                         "blocks -- the bench in front or the table below.")
    ap.add_argument("--bench-dy", type=float, default=0.0,
                    help="move the furniture toward (-) or away from (+) the "
                         "wearer. Answers where a work surface must SIT.")
    ap.add_argument("--orient", default="pinned",
                    choices=["pinned", "topdown", "any"],
                    help="pinned = the teleop wrist lock the tasks use. "
                         "topdown = tool axis straight down. any = accept if "
                         "ANY of a sampled set solves. The DIFFERENCE between "
                         "pinned and any is the cost of the wrist lock, which "
                         "is the one term nothing else here isolates.")
    ap.add_argument("--bench-dz", type=float, default=0.0,
                    help="RAISE the furniture by this much and measure just "
                         "above the new surface. Answers the question the "
                         "plain z sweep cannot: a taller work surface moves "
                         "the arm AND the obstacle together, so reach at a "
                         "higher z with the bench left at 1.10 is not the "
                         "same experiment.")
    ap.add_argument("--scene", default="",
                    help="publish this task's FURNITURE as collision objects "
                         "before measuring. The survey ran with scene 't1' "
                         "and this script originally did not, which is the "
                         "whole reason the two disagreed.")
    a = ap.parse_args()

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
    # CLEAR FIRST, ALWAYS. remove_furniture() used to run only when --scene
    # was given, so a run WITHOUT a scene inherited the previous run's
    # furniture and reported it as the no-furniture baseline. That produced
    # 0.025 m where a clean stack gives 0.325 m, i.e. a fabricated result
    # that looked like a finding.
    import clip_scene as _CS0
    _CS0.remove_furniture(n)
    n.spin(1.0)
    if a.scene:
        # THE FURNITURE IS A COLLISION OBJECT, and that turns out to be the
        # entire story -- see the header. Published the same way the survey
        # publishes it, from the same definition, so the two cannot differ.
        import clip_scene as CS
        import time as _t
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        tmp = CS.Scene.__new__(CS.Scene)
        cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
        cli.wait_for_service(timeout_sec=15.0)
        CS.remove_furniture(n)
        objs = CS.Scene._collision_furniture(tmp, a.scene)
        if a.furniture:
            want = set(a.furniture.split(","))
            objs = [o for o in objs if o.id in want
                    or o.id.split("_")[0] in want]
        if a.bench_dy:
            for o in objs:
                for pp in o.primitive_poses:
                    pp.position.y += a.bench_dy
        if a.bench_dz:
            for o in objs:
                for pp in o.primitive_poses:
                    pp.position.z += a.bench_dz
        if objs:
            ps = PlanningScene()
            ps.is_diff = True
            ps.world.collision_objects = objs
            fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
            end = _t.time() + 20.0
            while _t.time() < end and not fut.done():
                rclpy.spin_once(n, timeout_sec=0.05)
        print("scene %r published: %d collision objects"
              % (a.scene, len(objs)))
        n.spin(2.0)

    rig = Rig(n)
    print("mount left %s  right %s  hip %s"
          % (np.round(rig.mount["left"], 3), np.round(rig.mount["right"], 3),
             np.round(rig.hip, 3)))

    res = {}

    def row(label, z, xform=None, avoid=True):
        out = {}
        for arm in ("left", "right"):
            lo, hi, k = rig.reach(arm, z, xform, avoid, a.orient)
            out[arm] = dict(y_min=lo, y_max=hi, cells=k)
        L, R = out["left"], out["right"]
        print("   %-34s z=%.2f  L y %s..%s  R y %s..%s"
              % (label, z,
                 "----" if L["y_min"] is None else "%.3f" % L["y_min"],
                 "----" if L["y_max"] is None else "%.3f" % L["y_max"],
                 "----" if R["y_min"] is None else "%.3f" % R["y_min"],
                 "----" if R["y_max"] is None else "%.3f" % R["y_max"]))
        return out

    if "b" in a.only or "c" in a.only:
        print("\n(b) HEIGHT -- exact, nothing transformed")
        res["b_height"] = {}
        zs = (0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.40)
        if a.bench_dz:
            # ON the raised surface, plus a 20 mm working clearance.
            zs = (1.10 + a.bench_dz + 0.02,)
        for z in zs:
            res["b_height"]["%.2f" % z] = row(
                "bench top %.2f" % (1.10 + a.bench_dz), z)

    if "c" in a.only:
        print("\n(c) WEARER -- same sweep, collisions OFF. The DIFFERENCE "
              "is the wearer.")
        res["c_nocoll"] = {}
        for z in (0.95, 1.05, 1.12, 1.20, 1.30):
            res["c_nocoll"]["%.2f" % z] = row("collisions OFF", z, None, False)

    if "A" in a.only:
        # THE HONEST MOUNT MEASUREMENT. Transforming only the target leaves
        # the furniture where it was, so the arm is asked about a displaced
        # point while colliding against undisplaced obstacles -- not an upper
        # bound, just wrong. Here the FURNITURE is transformed by the same
        # inverse, per arm, and republished before that arm is measured.
        # The wearer still does not move, so this remains optimistic about
        # wearer clearance -- but the furniture, which is what actually
        # binds, is now handled correctly.
        import clip_scene as CS2
        import time as _t2
        from moveit_msgs.msg import PlanningScene as PS2
        from moveit_msgs.srv import ApplyPlanningScene as APS2
        tmp2 = CS2.Scene.__new__(CS2.Scene)
        cli2 = n.create_client(APS2, "/apply_planning_scene")
        cli2.wait_for_service(timeout_sec=15.0)

        def publish_xformed(arm, dR, dt):
            CS2.remove_furniture(n)
            n.spin(0.6)
            objs = CS2.Scene._collision_furniture(tmp2, a.scene or "t1")
            if a.furniture:
                want = set(a.furniture.split(","))
                objs = [o for o in objs if o.id in want
                        or o.id.split("_")[0] in want]
            m = rig.mount[arm]
            for o in objs:
                for pp in o.primitive_poses:
                    v = np.array([pp.position.x, pp.position.y,
                                  pp.position.z])
                    if dt is not None:
                        v = v - np.array([dt[0] if arm == "left" else -dt[0],
                                          dt[1], dt[2]])
                    if dR is not None:
                        v = dR.T @ (v - m) + m
                    pp.position.x, pp.position.y, pp.position.z = (
                        float(v[0]), float(v[1]), float(v[2]))
            ps = PS2()
            ps.is_diff = True
            ps.world.collision_objects = objs
            fut = cli2.call_async(APS2.Request(scene=ps))
            end = _t2.time() + 20.0
            while _t2.time() < end and not fut.done():
                rclpy.spin_once(n, timeout_sec=0.05)
            n.spin(1.0)

        print("\n(A) MOUNT with the FURNITURE TRANSFORMED TOO, per arm")
        res["A_mount_real"] = {}
        for lab, dR, dt in [("baseline", None, None),
                            ("tilt -30 deg", _Rx(math.radians(-30)), None),
                            ("tilt -45 deg", _Rx(math.radians(-45)), None),
                            ("fwd 0.15 + tilt -30", _Rx(math.radians(-30)),
                             (0.0, 0.15, 0.0)),
                            ("fwd 0.30", None, (0.0, 0.30, 0.0))]:
            cell = {}
            for arm in ("left", "right"):
                publish_xformed(arm, dR, dt)
                lo, hi, k = rig.reach(arm, a.z_probe,
                                      mount_xform(rig, dR, dt), True,
                                      a.orient)
                cell[arm] = dict(y_min=lo, y_max=hi, cells=k)
            res["A_mount_real"][lab] = cell
            print("   %-24s z=%.2f  L y_max %s   R y_max %s"
                  % (lab, a.z_probe,
                     "----" if cell["left"]["y_max"] is None
                     else "%.3f" % cell["left"]["y_max"],
                     "----" if cell["right"]["y_max"] is None
                     else "%.3f" % cell["right"]["y_max"]))

    if "a" in a.only:
        print("\n(a) MOUNT -- KINEMATIC UPPER BOUND, wearer does not move")
        res["a_mount"] = {}
        cands = [("tilt -15 deg", _Rx(math.radians(-15)), None),
                 ("tilt -30 deg", _Rx(math.radians(-30)), None),
                 ("tilt -45 deg", _Rx(math.radians(-45)), None),
                 ("tilt +30 deg", _Rx(math.radians(30)), None),
                 ("inboard 0.10", None, (-0.10, 0.0, 0.0)),
                 ("inboard 0.20", None, (-0.20, 0.0, 0.0)),
                 ("forward 0.15", None, (0.0, 0.15, 0.0)),
                 ("forward 0.30", None, (0.0, 0.30, 0.0)),
                 ("lower 0.15", None, (0.0, 0.0, -0.15)),
                 ("fwd 0.15 + tilt -30", _Rx(math.radians(-30)),
                  (0.0, 0.15, 0.0))]
        for lab, dR, dt in cands:
            res["a_mount"][lab] = row(lab, Z_WORK,
                                      mount_xform(rig, dR, dt))

    if "d" in a.only:
        print("\n(d) POSTURE -- lean is VALID for collision too (rigid body)")
        res["d_posture"] = {}
        for deg in (10, 20, 30):
            for sign in (-1, 1):
                th = math.radians(sign * deg)
                m = rig.mount["left"]
                mm = _Rx(th) @ (m - rig.hip) + rig.hip
                if mm[1] <= m[1]:
                    continue          # not forward; skip the wrong sign
                res["d_posture"]["lean %d deg" % deg] = row(
                    "lean %d deg (mount +%.3f m y)" % (deg, mm[1] - m[1]),
                    Z_WORK, lean_xform(rig, th))
        for dy in (-0.10, -0.20):
            res["d_posture"]["closer %.2f" % dy] = row(
                "stand %.2f m closer" % -dy, Z_WORK, shift_xform(dy))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(results=res, repeats=N, x_probe=X_PROBE,
                   scene=a.scene,
                   y_range=[Y0, Y1], dy=DY,
                   caveat="SINGLE-POSE IK at N=3. An envelope, not a "
                          "verified coordinate. Verify any chosen layout "
                          "with audit_scenario_reachability.py at N=10 over "
                          "the full path before recording."),
              open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
