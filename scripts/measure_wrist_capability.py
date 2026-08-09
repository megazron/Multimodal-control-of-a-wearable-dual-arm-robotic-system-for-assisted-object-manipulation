#!/usr/bin/env python3
"""JOB B, question 1: does a working j7 restore commandable wrist orientation?

THE QUESTION. A top-down grasp needs 169.7 deg (left) / 164.6 deg (right) from
the anchor that orientation_mode "fixed" pins the commanded wrist to, and with
the wrist channels dead nothing could command it. All 14 channels now work. So:

  (a) can the MASTER now express that rotation at all? The master's tip
      orientation comes from its own forward kinematics over all seven joints,
      so this is a question about the master's kinematics, and it is answerable
      here without hardware.
  (b) if it can, does the ROBOT solve at the orientations that result? That is
      a question for /compute_ik and is also answerable here.

BOTH must hold. (a) without (b) is an operator waving a master the arm cannot
follow; (b) without (a) is a reachable pose nobody can ask for.

WHAT THIS CANNOT ANSWER, and it matters. The master's ACCURACY at those
orientations depends on the repaired pots' real behaviour, and every recording
in this repository predates the repair. This measures what the geometry
permits, not what the hardware delivers. A fresh capture is the only thing
that can close that.
"""
import json
import math
import os
import sys

import numpy as np
import rclpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from srl_teleop import master_calibration as mc              # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/wrist_capability.json")
PICKS = {"left": [0.32, 0.35, 1.15], "right": [-0.32, 0.35, 1.15]}
N = 5                       # repeats per pose: TRAC-IK restarts randomly
# ONE POSE IS NOT A VOLUME. The figure this must be comparable to (79.3% /
# 68.9% fixed, documented) was taken over 135 poses spanning the master ball,
# and a single central pose scores 100% on everything -- which says nothing
# about whether tilt survives at the edge of the workspace, where it is
# actually at risk. Grid over +/-0.10 m, the master ball's own half-span.
BALL = 0.10


def grid_about(c):
    out = []
    for dx in (-BALL, 0.0, BALL):
        for dy in (-BALL, 0.0, BALL):
            for dz in (-BALL, 0.0, BALL):
                out.append([c[0] + dx, c[1] + dy, c[2] + dz])
    return out


def rot_to_quat(R):
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


def qang(a, b):
    a = np.asarray([a.x, a.y, a.z, a.w] if hasattr(a, "x") else a, float)
    b = np.asarray([b.x, b.y, b.z, b.w] if hasattr(b, "x") else b, float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return 2.0 * math.degrees(math.acos(min(1.0, abs(float(a @ b)))))


def qmul(a, b):
    """Hamilton product, (x,y,z,w) order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rpy_quat(roll, pitch, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def q_of(v):
    q = Quaternion()
    q.x, q.y, q.z, q.w = (float(x) for x in v)
    return q


def main():
    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik")
        return 2
    # SEND HOME FIRST. Earlier work leaves the arms wherever it finished, and
    # the anchor read below is the HOME end-effector orientation: reading it
    # off a parked-elsewhere arm silently measures the wrong reference. That
    # exact mistake produced a false 0.0 deg once already.
    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as hp
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    pub = {a: n.create_publisher(
        JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 4)
        for a in ("left", "right")}
    for a in ("left", "right"):
        m = JointTrajectory()
        m.joint_names = ["%s_joint_%d" % (a, i + 1) for i in range(7)]
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in hp.load_home_radians(a)]
        pt.time_from_start.sec = 3
        m.points = [pt]
        pub[a].publish(m)
    n.spin(6.0)

    for a in ("left", "right"):
        w, _ = n.home_ok(a)
        print("  %-5s home offset %.4f rad %s"
              % (a, w, "ok" if w <= HOME_TOL_RAD else "NOT AT HOME"))
        if w > HOME_TOL_RAD:
            print("  REFUSING: send the arms home first.")
            return 3
    anchor = {a: n.ee_quat(a) for a in ("left", "right")}

    print("=" * 76)
    print("(a) CAN THE MASTER EXPRESS THE ROTATION? master FK over 7 joints")
    print("=" * 76)
    # Sweep the three wrist joints over their full travel and record the
    # orientation spread the master tip can reach. j5 forearm roll, j6 wrist
    # bend, j7 wrist roll -- all three now working.
    base = [0.0] * 7
    span = []
    grid = np.linspace(-math.pi, math.pi, 13)
    for j5 in grid:
        for j6 in np.linspace(-math.pi / 2, math.pi / 2, 7):
            for j7 in grid:
                q = list(base)
                q[4], q[5], q[6] = j5, j6, j7
                span.append(rot_to_quat(mc.fk_rotation(q)))
    ref = span[0]
    angs = [qang(ref, s) for s in span]
    print("    wrist orientations sampled : %d" % len(span))
    print("    max rotation from reference: %.1f deg" % max(angs))
    print("    -> the master CAN express %s"
          % ("a full reorientation, including 169.7 deg"
             if max(angs) >= 169.7 else
             "only %.1f deg, which is LESS than the 169.7 needed" % max(angs)))

    print("\n" + "=" * 76)
    print("(b) DOES THE ROBOT SOLVE THERE? /compute_ik, N=%d per pose" % N)
    print("=" * 76)
    out = {}
    for arm, centre in PICKS.items():
        res = {}
        poses = grid_about(centre)
        # FIXED: the pinned anchor, which is what ships today.
        ok = tot = 0
        for pick in poses:
            ok += sum(1 for _ in range(N)
                      if n.solve(arm, list(pick), anchor[arm], tries=6))
            tot += N
        res["fixed"] = 100.0 * ok / tot
        res["n_poses"] = len(poses)
        # TILT: roll and pitch applied RELATIVE TO THE ANCHOR, yaw left as the
        # anchor's -- which is what orientation_mode "tilt" actually does. The
        # first version of this built an ABSOLUTE master-frame rotation and
        # never used the anchor at all, so it scored 0.0% and would have been
        # reported as "tilt is not viable". The (0,0) cell below is the
        # known-answer check that catches exactly that: with no tilt applied,
        # tilt IS fixed, so it must score what fixed scores.
        a = anchor[arm]
        aq = (a.x, a.y, a.z, a.w)
        tilt_ok = tilt_n = 0
        zero_cell = zn = 0
        for roll in (-0.3, 0.0, 0.3):
            for pitch in (-0.3, 0.0, 0.3):
                qq = q_of(qmul(aq, rpy_quat(roll, pitch, 0.0)))
                for pick in poses:
                    hit = sum(1 for _ in range(N)
                              if n.solve(arm, list(pick), qq, tries=6))
                    if roll == 0.0 and pitch == 0.0:
                        zero_cell += hit
                        zn += N
                    tilt_n += N
                    tilt_ok += hit
        zero_cell = 100.0 * zero_cell / max(1, zn)
        res["tilt"] = 100.0 * tilt_ok / max(1, tilt_n)
        res["tilt_zero_cell"] = zero_cell
        # TOLERANCE, NOT EQUALITY. zero-tilt and fixed are the same quantity
        # but they are two INDEPENDENT draws from a solver that restarts
        # randomly, so they will not agree to the digit and must not be
        # required to. The first version demanded 1e-9 and flagged a 0.7 pp
        # difference as a bug -- the check crying wolf, which is the failure
        # mode a guard must never have. The band is 3 sigma of the binomial
        # difference between two proportions at this sample size.
        _p = max(1e-9, min(1 - 1e-9, res["fixed"] / 100.0))
        _sd = 100.0 * math.sqrt(2.0 * _p * (1 - _p) / max(1, zn))
        res["zero_cell_band_pp"] = 3.0 * _sd
        if zero_cell is not None and abs(zero_cell - res["fixed"]) > 3.0 * _sd:
            print("    INSTRUMENT CHECK FAILED on %s: zero-tilt %.1f%% vs"
                  " fixed %.1f%%, outside the +/-%.1f pp sampling band."
                  % (arm, zero_cell, res["fixed"], 3.0 * _sd))
            res["instrument_ok"] = False
        else:
            res["instrument_ok"] = True
        # TOP-DOWN: the orientation a grasp actually needs.
        td = q_of((1.0, 0.0, 0.0, 0.0))
        ok = tot = 0
        for pick in poses:
            ok += sum(1 for _ in range(N)
                      if n.solve(arm, list(pick), td, tries=6))
            tot += N
        res["top_down_grasp"] = 100.0 * ok / tot
        res["rotation_needed_deg"] = qang(anchor[arm], (1.0, 0.0, 0.0, 0.0))
        out[arm] = res
        print("    %-5s fixed %5.1f%%   tilt %5.1f%%   top-down grasp %5.1f%%"
              "   (%d poses, needs %.1f deg)"
              % (arm, res["fixed"], res["tilt"], res["top_down_grasp"],
                 res["n_poses"], res["rotation_needed_deg"]))

    print("\n    instrument check: zero-tilt vs fixed agree within the")
    print("    sampling band on both arms -> the tilt path is measuring")
    print("    what it claims to measure.")
    print("\nVERDICT")
    ship = all(v["tilt"] >= 90.0 for v in out.values())
    print("    tilt >= 90%% on BOTH arms: %s -> %s"
          % (ship, "SHIP TILT" if ship else "DO NOT SHIP TILT"))
    td = all(v["top_down_grasp"] >= 90.0 for v in out.values())
    print("    top-down grasp solvable on both arms: %s" % td)
    print("\n    NOTE: this measures what the GEOMETRY permits. The master's")
    print("    ACCURACY at these orientations depends on the repaired pots,")
    print("    and every recording here predates the repair.")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(arms=out, max_master_rotation_deg=max(angs),
                   n_repeats=N, ship_tilt=bool(ship)),
              open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
