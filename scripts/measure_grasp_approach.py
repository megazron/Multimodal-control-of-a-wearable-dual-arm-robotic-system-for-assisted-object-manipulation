#!/usr/bin/env python3
"""IS THE APPROACH RIGHT FOR EACH GRASP? Per task, per object, per arm.

    python3 scripts/measure_grasp_approach.py [--repeats 10]

WHAT THIS ANSWERS, and it is three questions rather than one:

  1. WHICH APPROACH IS USED. Every teleop mode pins the wrist to the anchor
     orientation, so every grasp in the set is a NEAR-SIDE approach whether or
     not that suits the object. The tool axis is 30.7 deg ABOVE horizontal
     (left) and 22.1 (right) -- measured off TF, not assumed -- so the hand
     arrives from the near side and from below.

  2. WHETHER IT IS THE BEST AVAILABLE. The same grasp is re-measured with a
     TOP-DOWN wrist over the whole path, N repeats, furniture in the scene and
     the wearer measured GEOMETRICALLY. `09_task2_grasping_finding.md` records
     that the two approaches are complementary rather than ordered -- T2 needs
     near-side, T3's box needs top-down -- but its T1 row was taken on the OLD
     right-arm layout with pedestals, which no longer exists. T1 has since
     moved to the left arm on cells re-surveyed against the clearance floor,
     so that row has to be re-taken before it can be quoted.

  3. WHETHER THE FINGERS ARE ACTUALLY PRESENTED TO THE OBJECT. This is the
     question no reachability check asks. A pose can solve IK, clear the
     wearer and still close the hand across the wrong axis of the object, or
     with the object outside the pads entirely. Measured here from FK on the
     two finger-tip links: where the pads are, how wide the object is ALONG
     THE CLOSING AXIS, and how far the object centre sits off the line between
     them.

THE PAD OFFSET IS DERIVED PER APPROACH, AND THAT IS NOT A DETAIL.
`clip_tasks.PAD_OFFSET_BY_ARM` is the wrist-to-pad vector IN WORLD at the
anchor orientation. It is only valid at that orientation. Rotating the wrist
top-down moves the pads somewhere else entirely, so a top-down grasp built
with `ee_for()` puts the wrist where the near-side pads would have been and
the fingers nowhere near the object. Here the pad midpoint is read in the
END EFFECTOR frame -- where it is a constant of the hardware -- and rotated by
whichever approach is being tested. The control below checks that doing this
at the anchor reproduces PAD_OFFSET_BY_ARM, which was measured independently.

CONTROLS, and no report without them:

    pad offset at the anchor        must reproduce PAD_OFFSET_BY_ARM (< 5 mm)
    the top-down quaternion         FK must confirm the tool axis is within
                                    1 deg of straight down -- constructing a
                                    quaternion and believing it is how a
                                    "top-down" sweep ends up pointing forward
    a pose inside the torso         clearance must be NEGATIVE
    T1 cube 0, pinned, as commanded must be REACHABLE (it is the shipped path,
                                    verified 4/4 at N=10)
    a target 2.5 m out              must be unreachable under BOTH approaches
    the pad-axis extent             a 30 x 70 mm object must measure 30 mm
                                    across one pad axis and 70 across the
                                    perpendicular one -- constructed
                                    arithmetic, so the ground truth is known
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as M                                   # noqa: E402
import task3 as T3M                                          # noqa: E402
import tasks as TSK                                          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/grasp_approach.json")
STANDOFF, LIFT = 0.10, 0.08
APERTURE_M = 0.085                  # 2F-85 open span


# ------------------------------------------------------------------ quaternion
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def as_msg(q):
    m = Quaternion()
    m.x, m.y, m.z, m.w = (float(v) for v in q)
    return m


def as_tuple(m):
    return (m.x, m.y, m.z, m.w)


def top_down(yaw):
    """Tool +z pointing at world -z, yawed about world z.

    ROTATE 180 DEG ABOUT X, NOT 90. R_x(t) sends (0,0,1) to (0,-sin t, cos t),
    so -90 deg sends the tool axis to +y -- FORWARD, horizontal, not down.
    `measure_what_binds._fan` builds its "straight down" that way and it is
    therefore a horizontal fan; recorded rather than silently worked around.
    """
    qx = (math.sin(math.pi / 2), 0.0, 0.0, math.cos(math.pi / 2))
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return q_mul(qz, qx)


def tool_axis(q):
    return q_matrix(q)[:, 2]


def deg_from_down(q):
    a = tool_axis(q)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(-a[2])))))


# ------------------------------------------------------------------- the set
def grasp_set():
    """Every grasp the recorded task set actually performs, in world.

    `ee` items are declared as WRIST poses already (T2's tray is held rather
    than approached, so `tasks.py` gives the EE path directly); everything
    else is an OBJECT pose and the wrist is derived per approach.
    """
    g = []
    z = M.T1_Z
    for i, (cx, cy) in enumerate(M.T1_CUBES):
        g.append(dict(task="T1", name="cube_%d" % i, arm=M.T1_ARM,
                      obj=[cx, cy, z], size=(0.04, 0.04, 0.04), kind="pick"))
    slot = {0: -M.SLOT_DY, 2: +M.SLOT_DY, 1: -M.SLOT_DY, 3: +M.SLOT_DY}
    for i in range(4):
        px, py = M.T1_PLANES[M.T1_PAIR[i]]
        g.append(dict(task="T1", name="place_%d" % i, arm=M.T1_ARM,
                      obj=[px, round(py + slot[i], 4), z],
                      size=(0.04, 0.04, 0.04), kind="place"))
    # T2 -- the tray, held at two points TRAY_SEP apart. EE poses.
    p0 = TSK.TASK_B["paths"]["S2_full_lift"][0]
    half = TSK.TRAY_SEP / 2.0
    tray = TSK.TASK_B.get("objects", {}).get("tray", {}).get("size",
                                                             (0.56, 0.26, 0.02))
    # THE TRAY IS AT THE PADS, NOT AT THE WRIST, and getting that wrong is
    # worth 119 mm. `tasks.py` declares T2's path in EE coordinates because
    # the tray is held rather than approached, and `clip_scene._grip()` draws
    # the tray between the two arms' FINGER PADS. So the grip point is the
    # wrist waypoint plus the pad offset at whatever orientation is under
    # test -- `obj_from_ee` below -- and the first version of this file put
    # the tray at the wrist, which reported the pads 111.9 mm off the object
    # and called it a finding. 111.9 mm is the pad offset.
    for arm, sgn in (("left", 1.0), ("right", -1.0)):
        g.append(dict(task="T2", name="tray_%s" % arm, arm=arm,
                      ee=[p0[0] + sgn * half, p0[1], p0[2]],
                      obj_from_ee=True,
                      obj=[p0[0] + sgn * half, p0[1], p0[2]],
                      size=(0.06, tray[1], tray[2]), kind="hold"))
    # T3 -- two objects, one task, and the repository's own evidence that
    # approach is a per-grasp property.
    g.append(dict(task="T3", name="circuit_box", arm=T3M.BOX_ARM,
                  obj=list(T3M.BOX_OBJ), size=T3M.BOX_SIZE, kind="pick"))
    g.append(dict(task="T3", name="multimeter", arm=T3M.METER_ARM,
                  obj=list(T3M.METER_OBJ), size=T3M.METER_SIZE, kind="pick"))
    return g


# --------------------------------------------------------------------- FK
class FK:
    def __init__(self, node):
        self.n = node
        self.cli = node.create_client(GetPositionFK, "/compute_fk")
        self.cli.wait_for_service(timeout_sec=20.0)

    def poses(self, arm, joints, links):
        import time
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = list(links)
        rs = RobotState()
        rs.joint_state.name = self.n.names(arm)
        rs.joint_state.position = [float(v) for v in joints[:7]]
        req.robot_state = rs
        fut = self.cli.call_async(req)
        end = time.time() + 5.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.n, timeout_sec=0.002)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        out = {}
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            out[name] = (np.array([ps.pose.position.x, ps.pose.position.y,
                                   ps.pose.position.z]),
                         (ps.pose.orientation.x, ps.pose.orientation.y,
                          ps.pose.orientation.z, ps.pose.orientation.w))
        return out


def link_names(arm):
    return ["%s_end_effector_link" % arm,
            "%s_robotiq_85_left_finger_tip_link" % arm,
            "%s_robotiq_85_right_finger_tip_link" % arm]


def pad_mid_in_ee(fk, arm, joints):
    """Pad midpoint expressed in the END EFFECTOR frame -- a hardware constant.

    Read from FK rather than TF so it comes from the same solution everything
    else in this script is measured on.
    """
    p = fk.poses(arm, joints, link_names(arm))
    if p is None:
        return None, None
    ee_p, ee_q = p["%s_end_effector_link" % arm]
    R = q_matrix(ee_q)
    tips = [p["%s_robotiq_85_%s_finger_tip_link" % (arm, s)][0]
            for s in ("left", "right")]
    mid = (tips[0] + tips[1]) / 2.0
    return R.T @ (mid - ee_p), (tips[0], tips[1])


# --------------------------------------------------------- the presentation
def presentation(tips, obj, size):
    """Does the hand present its fingers to this object?

    Everything here is geometry on an axis-aligned box, so the ground truth is
    CONSTRUCTED and the self-test can check it against arithmetic.
    """
    tl, tr = np.asarray(tips[0], float), np.asarray(tips[1], float)
    span = float(np.linalg.norm(tl - tr))
    if span < 1e-6:
        return None
    u = (tl - tr) / span
    mid = (tl + tr) / 2.0
    c = np.asarray(obj, float)
    h = np.asarray(size, float) / 2.0
    # extent of an axis-aligned box along u
    extent = 2.0 * float(np.abs(u) @ h)
    d = mid - c
    along = float(d @ u)
    perp = float(np.linalg.norm(d - along * u))
    centred = bool(abs(along) <= max(h @ np.abs(u), 0.005) + 0.010
                   and perp <= 0.015)
    fits = bool(extent <= APERTURE_M)
    # PRESENTED MEANS BOTH, AND THE FIRST VERSION OF THIS ONLY CHECKED ONE.
    # It reported the T3 circuit box as PRESENTED with 195 mm of box lying
    # across a hand that opens to 85: the pads were centred on the object, so
    # the centring test passed, and nothing asked whether the object FITS
    # between them. A hand aimed perfectly at something it cannot close on is
    # not presenting its fingers to it.
    #
    # The pad span is NOT the aperture and must not be read as one: the IK
    # solution carries whatever gripper joints the seed had, so the tips come
    # back ~52 mm apart whatever the hand is about to do. The span is here to
    # fix the closing AXIS, which is the direction that matters; the aperture
    # is the hardware's 85 mm.
    return dict(pad_span_mm=round(span * 1000.0, 1),
                closing_axis=[round(float(v), 4) for v in u],
                object_extent_along_closing_axis_mm=round(extent * 1000.0, 1),
                pad_mid_offset_along_mm=round(along * 1000.0, 1),
                pad_mid_offset_perp_mm=round(perp * 1000.0, 1),
                fits_in_aperture=fits, pads_centred=centred,
                aperture_mm=round(APERTURE_M * 1000.0, 1),
                presented=bool(fits and centred),
                why=("" if fits and centred else
                     ("the object is %.0f mm across the closing axis against "
                      "an %.0f mm hand" % (extent * 1000.0, APERTURE_M * 1000.0)
                      if not fits else
                      "the pads are %.0f mm off the object" % (perp * 1000.0))))


# ------------------------------------------------------------------- paths
def wrist_for(obj, q, pad_ee):
    """WRIST pose that puts the pads on `obj` at approach orientation `q`."""
    return list(np.asarray(obj, float) - q_matrix(q) @ np.asarray(pad_ee))


def path_as_commanded(ee):
    pre = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def path_along_axis(ee, q):
    a = tool_axis(q)
    pre = list(np.asarray(ee, float) - STANDOFF * a)
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def run_path(rig, fk, arm, wps, q, repeats):
    """(all solved, worst geometric clearance over every solution, who).

    Clearance is taken on EVERY solution, not on the last one: TRAC-IK
    restarts randomly, so the N repeats of a waypoint are N different postures
    for the same hand pose and the worst of them is the one that matters.
    """
    worst, who = 1e9, None
    qm = as_msg(q)
    for w in wps:
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                return False, None, None
            c, k = rig.clearance(arm, j)
            if c is None:
                return False, None, None
            if c < worst:
                worst, who = c, k
    return True, worst, who


def grasp_joints(rig, arm, ee, q, repeats):
    """The joint vector at the GRASP waypoint itself, or None."""
    qm = as_msg(q)
    j = None
    for _ in range(repeats):
        j = rig.n.solve_arm_joints(arm, list(ee), qm, avoid=True, tries=6)
        rig.calls += 1
        if j is None:
            return None
    return j


def evaluate(rig, fk, g, q, pad_ee, repeats, shape):
    """One (grasp, approach, path shape) cell."""
    arm = g["arm"]
    ee = g["ee"] if "ee" in g else wrist_for(g["obj"], q, pad_ee)
    obj = (list(np.asarray(ee, float) + q_matrix(q) @ np.asarray(pad_ee))
           if g.get("obj_from_ee") else g["obj"])
    wps = (path_as_commanded(ee) if shape == "as_commanded"
           else path_along_axis(ee, q))
    ok, worst, who = run_path(rig, fk, arm, wps, q, repeats)
    res = dict(shape=shape, waypoints=len(wps),
               ee=[round(float(v), 4) for v in ee],
               object_at=[round(float(v), 4) for v in obj],
               reachable=ok,
               worst_clearance_m=None if worst is None else round(worst, 4),
               clearance_to=who,
               clears_floor=bool(ok and worst is not None
                                 and worst >= CLEAR_FLOOR))
    if ok:
        j = grasp_joints(rig, arm, ee, q, 1)
        if j is not None:
            p = fk.poses(arm, j, link_names(arm))
            if p is not None:
                tips = [p["%s_robotiq_85_%s_finger_tip_link" % (arm, s)][0]
                        for s in ("left", "right")]
                res["achieved_tool_deg_from_down"] = round(
                    deg_from_down(p["%s_end_effector_link" % arm][1]), 2)
                res["presentation"] = presentation(tips, obj, g["size"])
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--yaws", type=int, default=4)
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
    rig = Rig(n, "t1", 1)
    fk = FK(n)

    ctl, good = {}, True

    # ---- 1. the pad offset at the anchor must reproduce the shipped number
    pad_ee, pad_err = {}, {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        v, _ = pad_mid_in_ee(fk, arm, home)
        pad_ee[arm] = v
        qa = as_tuple(rig.quat[arm])
        world = q_matrix(qa) @ v
        want = np.asarray(CT.PAD_OFFSET_BY_ARM[arm], float)
        pad_err[arm] = float(np.linalg.norm(world - want))
        ctl["pad_offset_%s" % arm] = dict(
            want="< 0.005 m from PAD_OFFSET_BY_ARM",
            got="%.4f m  (measured %s)"
                % (pad_err[arm], [round(float(x), 4) for x in world]))
        good = good and pad_err[arm] < 0.005

    # ---- 2. the top-down quaternion must actually be top-down, by FK
    td_ctl = None
    for arm in ("left",):
        q = top_down(0.0)
        ee = wrist_for([0.60, 0.175, 1.20], q, pad_ee[arm])
        j = n.solve_arm_joints(arm, ee, as_msg(q), avoid=False, tries=8)
        if j is None:
            td_ctl = "no IK at the probe pose"
        else:
            p = fk.poses(arm, j, link_names(arm))
            td_ctl = round(deg_from_down(p["%s_end_effector_link" % arm][1]), 3)
    ctl["topdown_is_down"] = dict(want="< 1 deg from straight down",
                                  got=td_ctl)
    good = good and isinstance(td_ctl, float) and td_ctl < 1.0

    # ---- 3. a pose inside the torso must measure negative clearance
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22], rig.quat["left"],
                            avoid=False, tries=8)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = dict(
        want="< 0", got=None if c_t is None else round(c_t, 4))
    good = good and c_t is not None and c_t < 0.0

    # ---- 4. the shipped T1 grasp must still be reachable as commanded
    gset = grasp_set()
    c0 = [x for x in gset if x["name"] == "cube_0"][0]
    r0 = evaluate(rig, fk, c0, as_tuple(rig.quat[c0["arm"]]),
                  pad_ee[c0["arm"]], 1, "as_commanded")
    ctl["shipped_t1_cube0_reachable"] = dict(want="True", got=r0["reachable"])
    good = good and r0["reachable"]

    # ---- 5. a target 2.5 m out must fail under both approaches
    far = dict(task="ctl", name="far", arm="left", obj=[2.5, 0.2, 1.2],
               size=(0.04, 0.04, 0.04), kind="pick")
    f1 = evaluate(rig, fk, far, as_tuple(rig.quat["left"]), pad_ee["left"],
                  1, "as_commanded")["reachable"]
    f2 = evaluate(rig, fk, far, top_down(0.0), pad_ee["left"], 1,
                  "as_commanded")["reachable"]
    ctl["far_target_unreachable"] = dict(want="False / False",
                                         got="%s / %s" % (f1, f2))
    good = good and not f1 and not f2

    # ---- 6. the pad-axis extent is arithmetic, so check it against arithmetic
    tips_x = (np.array([0.03, 0.0, 0.0]), np.array([-0.03, 0.0, 0.0]))
    tips_y = (np.array([0.0, 0.03, 0.0]), np.array([0.0, -0.03, 0.0]))
    ex = presentation(tips_x, [0, 0, 0], (0.030, 0.070, 0.045))
    ey = presentation(tips_y, [0, 0, 0], (0.030, 0.070, 0.045))
    ctl["extent_along_pad_axis"] = dict(
        want="30.0 mm across x, 70.0 across y",
        got="%.1f / %.1f" % (ex["object_extent_along_closing_axis_mm"],
                             ey["object_extent_along_closing_axis_mm"]))
    good = good and abs(ex["object_extent_along_closing_axis_mm"] - 30.0) < 0.1 \
        and abs(ey["object_extent_along_closing_axis_mm"] - 70.0) < 0.1
    # ---- 7. an object WIDER THAN THE HAND must not read as presented.
    # The first version of `presentation()` only asked whether the pads were
    # centred, and duly reported 195 mm of circuit box lying across an 85 mm
    # hand as PRESENTED. A verdict that cannot fail on an ungraspable object
    # is not a verdict.
    wide = presentation(tips_x, [0, 0, 0], (0.200, 0.040, 0.040))
    ctl["too_wide_is_not_presented"] = dict(
        want="presented False, pads_centred True",
        got="%s / %s" % (wide["presented"], wide["pads_centred"]))
    good = good and (not wide["presented"]) and wide["pads_centred"]

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-30s want %-40s got %s" % (k, v["want"], v["got"]))
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    # ------------------------------------------------------------ the sweep
    print("\nAPPROACH PER GRASP -- N=%d over the full path, furniture in "
          "scene,\nwearer measured geometrically against a %.0f mm floor.\n"
          % (a.repeats, CLEAR_FLOOR * 1000.0))
    yaws = [i * 2.0 * math.pi / a.yaws for i in range(a.yaws)]
    rows = []
    for g in gset:
        arm = g["arm"]
        entry = dict(task=g["task"], name=g["name"], arm=arm,
                     obj=[round(v, 4) for v in g["obj"]],
                     size=[round(v, 4) for v in g["size"]],
                     kind=g["kind"], approaches={})
        qa = as_tuple(rig.quat[arm])
        entry["anchor_deg_from_down"] = round(deg_from_down(qa), 2)
        cands = [("pinned_near_side", qa)]
        # top-down: pick the yaw that solves the grasp pose, then run the
        # whole path on it. Trying every yaw over every path is 4x the cost
        # for an answer the grasp pose already selects.
        best = None
        for k, yw in enumerate(yaws):
            qd = top_down(yw)
            ee = wrist_for(g["obj"], qd, pad_ee[arm])
            if n.solve_arm_joints(arm, ee, as_msg(qd), avoid=True,
                                  tries=6) is not None:
                best = ("top_down_yaw%d" % k, qd)
                break
            rig.calls += 1
        if best is None:
            entry["approaches"]["top_down"] = dict(
                reachable=False, note="no yaw of straight down solves the "
                                      "grasp pose at all")
        else:
            cands.append(best)
        for label, q in cands:
            entry["approaches"][label] = {}
            for shape in ("as_commanded", "along_axis"):
                entry["approaches"][label][shape] = evaluate(
                    rig, fk, g, q, pad_ee[arm], a.repeats, shape)
        rows.append(entry)

        def fmt(lbl, shp):
            d = entry["approaches"].get(lbl, {})
            d = d.get(shp) if isinstance(d.get(shp), dict) else None
            if d is None:
                return "%-22s" % "--"
            return "%-9s clr %-8s" % (
                "OK" if d["clears_floor"] else
                ("reach" if d["reachable"] else "FAIL"),
                "----" if d["worst_clearance_m"] is None
                else "%.3f" % d["worst_clearance_m"])
        print("   %-3s %-13s %-5s  pinned[cmd] %s pinned[axis] %s  "
              "topdown[cmd] %s topdown[axis] %s"
              % (g["task"], g["name"], arm,
                 fmt("pinned_near_side", "as_commanded"),
                 fmt("pinned_near_side", "along_axis"),
                 fmt(best[0] if best else "top_down", "as_commanded"),
                 fmt(best[0] if best else "top_down", "along_axis")))
        for label in entry["approaches"]:
            d = entry["approaches"][label].get("as_commanded")
            if isinstance(d, dict) and d.get("presentation"):
                p = d["presentation"]
                print("        %-18s object %.1f mm across the closing axis "
                      "(hand opens %.0f), pads %.1f mm off centre -- %s%s"
                      % (label,
                         p["object_extent_along_closing_axis_mm"],
                         p["aperture_mm"], p["pad_mid_offset_perp_mm"],
                         "PRESENTED" if p["presented"] else "NOT presented",
                         "" if p["presented"] else ": " + p["why"]))

    # ------------------------------------- THE TASKS' OWN COMMANDED PATHS
    # A grasp pose is not a task. `verify_t1_paths.py` walks T1's real
    # waypoint list against the geometric wearer model; T0, T2 and T3 have
    # never had that pass, and this is the cheapest place to take it, because
    # the clearance machinery is already up. It is REPORTED, not fixed here:
    # a layout change is a separate decision with its own re-verification.
    print("\nTHE TASKS' OWN COMMANDED PATHS, wearer measured geometrically\n"
          "   (every waypoint the recorder sends, at the pinned anchor, "
          "N=%d)" % min(a.repeats, 3))
    paths = {}
    try:
        walks = [("T0", M.t0()), ("T1", M.t1()), ("T2", M.t2()), ("T3", M.t3())]
    except Exception as e:                                     # noqa: BLE001
        walks = []
        print("   could not build the task paths: %s" % e)
    for tname, per_arm in walks:
        paths[tname] = {}
        for arm, wps in per_arm.items():
            # Consecutive duplicates are HOLDS -- the same pose repeated so an
            # idle arm stays put. Measuring each one again buys nothing.
            uniq, seen = [], set()
            for w in wps:
                k = tuple(round(v, 4) for v in w)
                if k not in seen:
                    seen.add(k)
                    uniq.append(list(w))
            worst, who, fails, where = 1e9, None, 0, None
            for w in uniq:
                j = None
                for _ in range(min(a.repeats, 3)):
                    j = rig.n.solve_arm_joints(arm, w, rig.quat[arm],
                                               avoid=True, tries=6)
                    rig.calls += 1
                    if j is None:
                        break
                if j is None:
                    fails += 1
                    continue
                c, k2 = rig.clearance(arm, j)
                if c is not None and c < worst:
                    worst, who, where = c, k2, list(w)
            paths[tname][arm] = dict(
                waypoints=len(uniq), ik_failures=fails,
                worst_clearance_m=None if worst > 1e8 else round(worst, 4),
                to=who, at=where,
                clears_floor=bool(worst <= 1e8 and worst >= CLEAR_FLOOR))
            print("   %-3s %-5s %3d distinct waypoints, %d IK failures, "
                  "worst clearance %s to %s  %s"
                  % (tname, arm, len(uniq), fails,
                     "----" if worst > 1e8 else "%.4f m" % worst, who,
                     "" if worst >= CLEAR_FLOOR
                     else "<-- BREACHES the %.0f mm floor by %.0f mm"
                          % (CLEAR_FLOOR * 1000.0,
                             (CLEAR_FLOOR - worst) * 1000.0)))

    out = dict(repeats=a.repeats, floor=CLEAR_FLOOR, aperture_m=APERTURE_M,
               controls=ctl, grasps=rows, task_paths=paths,
               ik_calls=rig.calls,
               pad_offset_in_ee_frame={k: [round(float(x), 4) for x in v]
                                       for k, v in pad_ee.items()},
               method="full path (standoff, grasp, lift) at N=repeats, "
                      "avoid_collisions with the task furniture applied, "
                      "wearer clearance from the mount guard's capsule model "
                      "on every returned solution; pad offset rotated per "
                      "approach rather than taken from the anchor")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
