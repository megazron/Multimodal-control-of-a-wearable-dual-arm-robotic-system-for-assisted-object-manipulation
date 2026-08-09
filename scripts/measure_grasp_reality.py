#!/usr/bin/env python3
"""PART 1: is the grasp real, and how precise is it? Measure, do not assert.

Every number printed carries how it was obtained. Where a quantity rests on an
assumption, the assumption is measured first and validated against a known
answer before anything is built on it.

  [0] INSTRUMENT   true aperture against knuckle angle, validated against the
                   vendor's published 85 mm stroke
  (a) finger separation over time against the object's width
  (b) is the approach a straight line along an approach vector from a standoff
  (c) is the wrist aligned to the object, or pinned to the anchor
  (d) positioning error at the grasp point, repeat variance, smallest object
  (e) achievable grasp orientations per arm

Run against a bare sim stack (demo.launch.py). Needs no master arm.
"""
import glob
import json
import math
import os
import sys

import numpy as np
import rclpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from record_verification import Driver          # noqa: E402
from geometry_msgs.msg import Quaternion        # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/grasp_reality.json")
KNUCKLE = "%s_robotiq_85_left_knuckle_joint"
PAD_L = "%s_robotiq_85_left_finger_tip_link"
PAD_R = "%s_robotiq_85_right_finger_tip_link"
EE = "%s_end_effector_link"
VENDOR_STROKE_MM = 85.0        # Robotiq 2F-85 published maximum opening


def attach_gripper_publisher(dr):
    """record_rviz patches send_gripper on inside its own main, so a bare
    Driver does not have it. Same topic, same message."""
    from rclpy.qos import QoSProfile
    from trajectory_msgs.msg import JointTrajectory as JT
    from trajectory_msgs.msg import JointTrajectoryPoint as JTP
    pubs = {a: dr.create_publisher(
        JT, "/%s_gripper_controller/joint_trajectory" % a, QoSProfile(depth=4))
        for a in ("left", "right")}

    def send_gripper(arm, angle):
        m = JT()
        m.joint_names = [KNUCKLE % arm]
        pt = JTP()
        pt.positions = [float(angle)]
        pt.time_from_start.nanosec = 250_000_000
        m.points = [pt]
        pubs[arm].publish(m)
    dr.send_gripper = send_gripper


# ============================================================== [0] instrument
def aperture_curve(dr, arm):
    """Measured pad-to-pad aperture against knuckle angle.

    THE TRAP THIS AVOIDS. Reading the two finger-tip frames straight out of TF
    gives the distance between LINK ORIGINS, and the origin sits behind the pad
    face. Taken raw it says the gripper never closes below 50.7 mm and that
    grip_for()'s linear model is 52 mm wrong. Both are artefacts of the
    instrument. Subtracting the fully-closed origin gap gives the true
    aperture, and the check that this is right is that full open then lands on
    the vendor's published 85 mm rather than on nothing in particular.
    """
    rows = []
    for a in np.linspace(0.0, 0.80, 17):
        dr.send_gripper(arm, float(a))
        for _ in range(14):
            dr.spin(0.05)
        pl, pr = dr.link_pose(PAD_L % arm), dr.link_pose(PAD_R % arm)
        act = dr.js.get(KNUCKLE % arm)
        if pl is None or pr is None or act is None:
            continue
        rows.append(dict(cmd=float(a), knuckle=float(act),
                         origin_gap_mm=float(1000.0 * np.linalg.norm(
                             np.asarray(pl) - np.asarray(pr)))))
    if not rows:
        return None, None
    off = min(r["origin_gap_mm"] for r in rows)
    for r in rows:
        r["aperture_mm"] = r["origin_gap_mm"] - off
    return rows, off


def aperture_of(curve, knuckle):
    k = [r["knuckle"] for r in curve]
    a = [r["aperture_mm"] for r in curve]
    return float(np.interp(knuckle, k, a))


def knuckle_for(curve, aperture_mm):
    a = [r["aperture_mm"] for r in curve][::-1]
    k = [r["knuckle"] for r in curve][::-1]
    return float(np.interp(aperture_mm, a, k))


# ------------------------------------------------------------------- helpers
def q_of(v):
    q = Quaternion()
    q.x, q.y, q.z, q.w = (float(x) for x in v)
    return q


def qang(a, b):
    a = np.asarray([a.x, a.y, a.z, a.w] if hasattr(a, "x") else a, float)
    b = np.asarray([b.x, b.y, b.z, b.w] if hasattr(b, "x") else b, float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return 2.0 * math.degrees(math.acos(min(1.0, abs(float(a @ b)))))


def main():
    rclpy.init()
    dr = Driver()
    attach_gripper_publisher(dr)
    dr.spin(4.0)
    if not dr.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start a sim stack first")
        return 2
    out = {}
    P = print
    # THE ANCHOR IS THE HOME ORIENTATION, AND THE ARMS MUST ACTUALLY BE AT
    # HOME WHEN IT IS READ. ee_quat() returns the CURRENT end-effector
    # orientation, so two separate mistakes give the same wrong answer of
    # 0.0 deg: reading it after the approach test, and reading it at startup
    # when a PREVIOUS run left the arms parked on the grasp pose. Both
    # happened. Send home, wait, verify, then read.
    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as hp
    for a in ("left", "right"):
        dr.send(a, hp.load_home_radians(a), 2.5)
    for _ in range(70):
        dr.spin(0.05)
    off_home = {}
    for a in ("left", "right"):
        tgt = hp.load_home_radians(a)
        cur = [dr.js.get("%s_joint_%d" % (a, i + 1), 0.0) for i in range(7)]
        off_home[a] = max(abs((cur[i] - tgt[i] + math.pi) % (2 * math.pi)
                              - math.pi) for i in range(7))
    if max(off_home.values()) > 0.05:
        print("  REFUSING: arms not at home (left %.4f, right %.4f rad)."
              % (off_home["left"], off_home["right"]))
        print("  The anchor read here would be whatever pose they are in.")
        return 4
    home_anchor = {a: dr.ee_quat(a) for a in ("left", "right")}
    P("=" * 76)
    P("PART 1   IS THE GRASP REAL?   bare sim stack, mock hardware")
    P("=" * 76)

    # ------------------------------------------------------- [0] instrument
    P("\n[0] INSTRUMENT  aperture against knuckle angle")
    curve, off = aperture_curve(dr, "left")
    if curve is None:
        P("    FAILED: no finger-tip transforms.")
        return 3
    open_mm = max(r["aperture_mm"] for r in curve)
    P("    method   command angle, read both finger-tip frames from TF,")
    P("             subtract the fully-closed origin gap (%.2f mm)." % off)
    P("    VALIDATION  full open measures %.2f mm against the vendor's"
      % open_mm)
    P("             published %.0f mm stroke: agreement %.2f mm."
      % (VENDOR_STROKE_MM, abs(open_mm - VENDOR_STROKE_MM)))
    errs = [abs(85.0 * (1 - r["knuckle"] / 0.8) - r["aperture_mm"])
            for r in curve]
    P("    grip_for() linear model error: mean %.2f mm, max %.2f mm"
      % (np.mean(errs), np.max(errs)))
    out["instrument"] = dict(origin_offset_mm=off, open_mm=open_mm,
                             vendor_stroke_mm=VENDOR_STROKE_MM,
                             linear_model_err_mean=float(np.mean(errs)),
                             linear_model_err_max=float(np.max(errs)),
                             curve=curve)

    # ---------------------------------------------- (a) closing on the object
    P("\n(a) IS THE GRIPPER CLOSING ON THE OBJECT?")
    P("    method   recorded knuckle traces -> aperture via the curve above,")
    P("             compared against each clip's declared object width.")
    rows = []
    for f in sorted(glob.glob(os.path.join(
            ROOT, "recordings/verification/*/*/*/*/grip_trace.json"))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        parts = f.split(os.sep)
        task, scen, cond = parts[-4], parts[-3], parts[-2]
        vals = [g[a] for g in d for a in ("left", "right")
                if g.get(a) is not None]
        if not vals:
            continue
        rows.append(dict(task=task, scen=scen, cond=cond,
                         open_mm=aperture_of(curve, min(vals)),
                         closed_mm=aperture_of(curve, max(vals))))
    out["closure"] = rows
    if rows:
        wid = {"t2": 40.0, "t3": 20.0, "t5": 32.0, "t6": 20.0}
        P("    %d clips with a knuckle trace" % len(rows))
        P("    task  n   min aperture   max aperture   object   closes past it?")
        for t in sorted({r["task"] for r in rows}):
            rs = [r for r in rows if r["task"] == t]
            mn = np.mean([r["closed_mm"] for r in rs])
            mx = np.mean([r["open_mm"] for r in rs])
            w = wid.get(t)
            verdict = "n/a (carries nothing)" if w is None else (
                "YES, by %.1f mm" % (w - mn) if mn <= w else
                "NO, stops %.1f mm short" % (mn - w))
            P("    %-4s %2d   %6.1f mm      %6.1f mm     %s   %s"
              % (t, len(rs), mn, mx,
                 ("%4.0f mm" % w) if w else "  --   ", verdict))
    else:
        P("    NO TRACES FOUND -- cannot answer (a) from recorded data.")

    # ------------------------------------------------- (b) straight approach
    P("\n(b) IS THE APPROACH A REAL GRASP?")
    P("    method   drive the pre-grasp -> grasp path through the same IK the")
    P("             recorder uses, read the EE from TF at each waypoint, and")
    P("             measure perpendicular deviation from the straight line.")
    import importlib
    rr = importlib.import_module("record_rviz")
    dev_rows = []
    for arm, pick in (("left", [0.32, 0.35, 1.15]),
                      ("right", [-0.32, 0.35, 1.15])):
        plan = rr.plan_grasp(dr, arm, pick, "block")
        if not plan.get("ok"):
            dev_rows.append(dict(arm=arm, refused=plan.get("refused")))
            P("    %-5s GRASP REFUSED: %s" % (arm, plan.get("refused")))
            continue
        p0 = np.asarray(plan["pregrasp"], float)
        p1 = np.asarray(plan["grasp"], float)
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        q = q_of(plan["quat"])
        devs = []
        for w in plan["path"]:
            sol = dr.solve_joints(arm, list(w), q)
            if sol is None:
                continue
            dr.send(arm, sol, 0.45)
            for _ in range(11):
                dr.spin(0.05)
            ee = dr.link_pose(EE % arm)
            if ee is None:
                continue
            v = np.asarray(ee, float) - p0
            devs.append(float(np.linalg.norm(v - np.dot(v, u) * u)))
        dev_rows.append(dict(arm=arm, standoff_mm=1000 * plan["approach_m"],
                             n=len(devs),
                             dev_mean_mm=float(1000 * np.mean(devs)),
                             dev_max_mm=float(1000 * np.max(devs))))
        P("    %-5s standoff %.0f mm, %d waypoints, deviation from the "
          "straight line: mean %.2f mm, max %.2f mm"
          % (arm, 1000 * plan["approach_m"], len(devs),
             1000 * np.mean(devs), 1000 * np.max(devs)))
    out["approach"] = dev_rows

    # ------------------------------------------------------ (c) wrist aligned
    P("\n(c) IS THE WRIST ALIGNED TO THE OBJECT, OR PINNED?")
    P("    method   compare the grasp quaternion the generator asks for")
    P("             against the home end-effector orientation that")
    P("             orientation_mode 'fixed' pins the commanded wrist to.")
    wr = []
    for arm, pick in (("left", [0.32, 0.35, 1.15]),
                      ("right", [-0.32, 0.35, 1.15])):
        anchor = home_anchor.get(arm)
        plan = rr.plan_grasp(dr, arm, pick, "block")
        if not plan.get("ok") or anchor is None:
            continue
        a = qang(anchor, plan["quat"])
        wr.append(dict(arm=arm, rotation_deg=a))
        P("    %-5s grasp needs %.1f deg from the pinned anchor" % (arm, a))
    out["wrist"] = wr

    # --------------------------------------------------------- (d) precision
    P("\n(d) HOW PRECISE IS IT?")
    P("    method   command the grasp pose N times from a randomised start,")
    P("             read the achieved EE from TF, report error and spread.")
    P("    WHAT THIS CAN AND CANNOT MEASURE. mock_components echoes commanded")
    P("    joint positions with no dynamics, so the arm lands exactly where")
    P("    the solver put it. The residual below is therefore the IK solver's")
    P("    convergence and the redundancy branch's repeatability, NOT the")
    P("    robot's positioning accuracy, which is structurally unmeasurable")
    P("    here. Real accuracy needs the arm.")
    prec = {}
    for arm, pick in (("left", [0.32, 0.35, 1.15]),
                      ("right", [-0.32, 0.35, 1.15])):
        plan = rr.plan_grasp(dr, arm, pick, "block")
        if not plan.get("ok"):
            continue
        q = q_of(plan["quat"])
        tgt = np.asarray(plan["grasp"], float)
        got = []
        for i in range(10):
            jitter = np.asarray(plan["pregrasp"], float) + \
                np.array([0.02 * math.sin(i), 0.02 * math.cos(i), 0.02])
            s0 = dr.solve_joints(arm, list(jitter), q)
            if s0 is not None:
                dr.send(arm, s0, 0.7)
                for _ in range(16):
                    dr.spin(0.05)
            s = dr.solve_joints(arm, list(tgt), q)
            if s is None:
                continue
            dr.send(arm, s, 0.7)
            for _ in range(20):
                dr.spin(0.05)
            ee = dr.link_pose(EE % arm)
            if ee is not None:
                got.append(np.asarray(ee, float))
        if len(got) < 3:
            continue
        G = np.array(got)
        err = np.linalg.norm(G - tgt, axis=1)
        spread = np.linalg.norm(G - G.mean(axis=0), axis=1)
        prec[arm] = dict(n=len(got),
                         err_mean_mm=float(1000 * err.mean()),
                         err_max_mm=float(1000 * err.max()),
                         repeat_sd_mm=float(1000 * spread.std()),
                         repeat_max_mm=float(1000 * spread.max()))
        P("    %-5s n=%d  IK residual mean %.4f mm / max %.4f mm,"
          "  branch repeatability sd %.4f mm / max %.4f mm"
          % (arm, len(got), 1000 * err.mean(), 1000 * err.max(),
             1000 * spread.std(), 1000 * spread.max()))
    out["precision"] = prec
    if prec:
        P("    SMALLEST RELIABLE OBJECT: NOT DERIVABLE FROM THIS MEASUREMENT.")
        P("      The lower bound is set by positioning error, which is exactly")
        P("      the quantity the mock cannot produce. What CAN be bounded")
        P("      here is the aperture side: the fingers span %.1f mm at full"
          % open_mm)
        P("      open, and the command resolution set by the linear model's")
        P("      %.2f mm error is the floor on how tightly a width can be"
          % out["instrument"]["linear_model_err_max"])
        P("      commanded. Object size must be re-derived on hardware.")
        out["object_window_mm"] = dict(
            min="UNVERIFIED -- needs hardware", max=open_mm,
            note="mock echoes commands; positioning error is structurally 0")

    # -------------------------------------------------------- (e) dexterity
    P("\n(e) WHAT GRASP ORIENTATIONS ARE ACHIEVABLE?")
    P("    method   at each pick point, sample approach directions over a")
    P("             hemisphere x yaw and count which solve, N=3 each.")
    dex = {}
    dirs = {"top-down": (0, 0, -1), "front": (0, 1, 0), "back": (0, -1, 0),
            "from +x": (1, 0, 0), "from -x": (-1, 0, 0),
            "45 front-down": (0, 0.707, -0.707),
            "45 side-down +x": (0.707, 0, -0.707),
            "45 side-down -x": (-0.707, 0, -0.707)}
    for arm, pick in (("left", [0.32, 0.35, 1.15]),
                      ("right", [-0.32, 0.35, 1.15])):
        ok = {}
        for name, d in dirs.items():
            hits = 0
            for yaw in (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4):
                qq = approach_quat(d, yaw)
                if all(dr.solve_joints(arm, list(pick), q_of(qq)) is not None
                       for _ in range(3)):
                    hits += 1
            ok[name] = hits
        dex[arm] = ok
        got = [k for k, v in ok.items() if v > 0]
        P("    %-5s solvable approach directions: %d of %d"
          % (arm, len(got), len(dirs)))
        for k, v in ok.items():
            P("        %-16s %d/4 yaw settings" % (k, v))
    out["dexterity"] = dex

    dr.destroy_node()
    rclpy.shutdown()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    P("\n  -> %s" % OUT)
    return 0


def approach_quat(approach, yaw):
    """Quaternion whose z axis points along `approach`, rolled by `yaw`.

    The gripper's approach axis is its own +z, so building the frame from the
    desired approach direction and then spinning about it enumerates exactly
    the wrist freedom that matters for a grasp.
    """
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(tmp, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    c, s = math.cos(yaw), math.sin(yaw)
    xr = c * x + s * y
    yr = -s * x + c * y
    R = np.column_stack([xr, yr, z])
    t = np.trace(R)
    if t > 0:
        w = math.sqrt(1 + t) / 2
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                (R[1, 0] - R[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    r = math.sqrt(max(1e-12, 1 + R[i, i] - R[j, j] - R[k, k]))
    v = [0.0, 0.0, 0.0]
    v[i] = r / 2
    v[j] = (R[j, i] + R[i, j]) / (2 * r)
    v[k] = (R[k, i] + R[i, k]) / (2 * r)
    return (v[0], v[1], v[2], (R[k, j] - R[j, k]) / (2 * r))


if __name__ == "__main__":
    sys.exit(main())
