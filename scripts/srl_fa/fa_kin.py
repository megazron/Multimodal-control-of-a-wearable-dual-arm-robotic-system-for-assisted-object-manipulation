#!/usr/bin/env python3
"""IK on ANY link, and a transit check that walks the whole hand.

    python3 scripts/srl_fa/fa_kin.py --self-test

WHY NOT plan_pick_left.ik
-------------------------
That solver is excellent and it is used unchanged for grasps: it puts the PAD
MIDPOINT at a position with the end effector at an orientation.  Scanning asks
a different question -- put the CAMERA there, looking that way -- and the
camera is not the pads.  Rather than convert back and forth through a fixed
offset (and get the sign wrong once), the same damped-least-squares loop is
written here over an arbitrary link.  It is the same algorithm, the same
finite-difference Jacobian and the same FK, so a grasp solved by either lands
in the same place; the self-test requires exactly that.

WHY THE TRANSIT CHECK EXISTS
----------------------------
A straight line in joint space says nothing about where the hand goes.  That
is how the gripper was driven into the table on 2026-08-25.  `safe_goto_left`
already checks this, against a table plane read from a SAVED file.  Full
autonomy measures the table every time it looks, so the check here takes a
plane as an argument and additionally clears the OBJECT COLUMNS -- the
cardboard box is 120 mm of obstacle that a table-plane check waves straight
through.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_fk import FK  # noqa: E402

# Every part of the hand that could touch something.  The forearm is included
# because a wrist-down pose puts it lower than the fingers.
HAND_LINKS = [
    "end_effector_link",
    "robotiq_85_base_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_right_knuckle_link",
    "spherical_wrist_2_link",
]
PAD_LINKS = ["robotiq_85_left_finger_tip_link",
             "robotiq_85_right_finger_tip_link"]
CAM_LINK = "camera_depth_frame"
CAM_COLOUR_LINK = "camera_color_frame"


# ---------------------------------------------------------------------- maths
def rot_err(R_cur, R_des):
    """Rotation vector taking R_cur to R_des."""
    E = R_des @ R_cur.T
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    s = np.linalg.norm(v)
    c = (np.trace(E) - 1) / 2
    if s < 1e-9:
        return np.zeros(3) if c > 0 else np.array([math.pi, 0.0, 0.0])
    return v / s * math.atan2(s, c)


def look_at(eye, target, up_hint=(0.0, 0.0, 1.0)):
    """Camera rotation whose +Z (optical axis) points from `eye` at `target`.

    The convention is OpenCV's, which is what the depth deprojection above
    assumes: +Z forward, +X right, +Y down.
    """
    z = np.asarray(target, float) - np.asarray(eye, float)
    nz = np.linalg.norm(z)
    if nz < 1e-9:
        raise ValueError("look_at: eye and target coincide")
    z = z / nz
    up = np.asarray(up_hint, float)
    if abs(float(up @ z)) > 0.98:                 # degenerate: pick another up
        up = np.array([0.0, 1.0, 0.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)


# ------------------------------------------------------------------------ IK
def ik_link(fk, arm, link, p_des, R_des, q0, grip=0.0, iters=300,
            tol_p=3e-4, tol_r=math.radians(0.5), damping=0.05, max_dq=0.15):
    """Damped least squares putting `link` at (p_des, R_des). Returns
    (q, pos_err_m, rot_err_rad, iterations).

    `R_des=None` solves POSITION ONLY, which is what a reach wants: fixing an
    orientation a 7-DOF arm does not need spends the redundancy that keeps the
    elbow out of the wearer (CLAUDE.md, "what the pinned wrist COSTS").
    """
    q = np.array(q0, float)
    lo, hi, cont = fk.limits(arm)
    rows = 6 if R_des is not None else 3

    def pose(qq):
        T, = fk.poses(arm, qq, [link], gripper=grip)
        return T[:3, 3], T[:3, :3]

    for it in range(iters):
        p, R = pose(q)
        ep = np.asarray(p_des, float) - p
        er = rot_err(R, R_des) if R_des is not None else np.zeros(3)
        if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
            return q, float(np.linalg.norm(ep)), float(np.linalg.norm(er)), it
        J = np.zeros((rows, 7))
        h = 1e-6
        for j in range(7):
            qd = q.copy()
            qd[j] += h
            p2, R2 = pose(qd)
            J[:3, j] = (p2 - p) / h
            if R_des is not None:
                J[3:, j] = rot_err(R, R2) / h
        e = np.r_[ep, er] if R_des is not None else ep
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(rows), e)
        nrm = np.linalg.norm(dq)
        if nrm > max_dq:
            dq *= max_dq / nrm
        q = q + dq
        for j in range(7):
            if not cont[j]:
                q[j] = min(hi[j], max(lo[j], q[j]))
    p, R = pose(q)
    er = rot_err(R, R_des) if R_des is not None else np.zeros(3)
    return (q, float(np.linalg.norm(np.asarray(p_des, float) - p)),
            float(np.linalg.norm(er)), iters)


# -------------------------------------------------------------- hand geometry
def pads_world(fk, arm, q, grip):
    """(left tip, right tip, midpoint) in world."""
    Tl, Tr = fk.poses(arm, q, PAD_LINKS, gripper=grip)
    return Tl[:3, 3], Tr[:3, 3], (Tl[:3, 3] + Tr[:3, 3]) / 2.0


def pads_in_cam(fk, arm, q, grip):
    """(left tip, right tip, midpoint) in the DEPTH CAMERA frame.

    Camera and pads are one rigid chain, so this carries NO mount error --
    which is the entire reason the descent guards are computed here.
    """
    Tc, Tl, Tr = fk.poses(arm, q, [CAM_LINK] + PAD_LINKS, gripper=grip)
    R, t = Tc[:3, :3], Tc[:3, 3]
    f = lambda p: R.T @ (p - t)                              # noqa: E731
    a, b = f(Tl[:3, 3]), f(Tr[:3, 3])
    return a, b, (a + b) / 2.0


def T_colour_from_depth(fk, arm, q):
    """4x4 taking depth-frame points into the colour frame. Rigid chain."""
    Td, Tc = fk.poses(arm, q, [CAM_LINK, CAM_COLOUR_LINK])
    return np.linalg.inv(Tc) @ Td


HAND_EXCLUDE_R = 0.075       # radius of the spheres that hide the gripper


def hand_spheres(fk, arm, q, grip, radius=HAND_EXCLUDE_R):
    """Camera-frame spheres covering the robot's own hand.

    THE ROBOT IS IN ITS OWN PICTURE.  At a scan pose the fingers hang into the
    bottom of the frame, stand 100-200 mm proud of the table and segment
    beautifully as a tall grey object.  FK puts them in the camera frame down
    one rigid chain, so masking them carries no mount error -- and nothing
    else can do the job: they are the same colour and the same height as
    things that really are on the table.
    """
    links = [CAM_LINK, "robotiq_85_base_link"] + PAD_LINKS
    Ts = fk.poses(arm, q, links, gripper=grip)
    R, t = Ts[0][:3, :3], Ts[0][:3, 3]
    return [(R.T @ (T[:3, 3] - t), radius) for T in Ts[1:]]


class Surface:
    """A plane in WORLD, plus the object columns standing on it.

    Built from a scan.  Everything here carries the residual mount error, so
    it is used for TRANSIT margins (tens of millimetres) and never to decide
    when to stop descending -- that is fa_perception's camera-frame job.
    """

    def __init__(self, n_w, d_w, columns=()):
        n = np.asarray(n_w, float)
        self.n = n / np.linalg.norm(n)
        self.d = float(d_w)
        # (centre_xy_world, radius_m, top_height_above_plane_m)
        self.columns = list(columns)

    def height(self, p):
        return float(self.n @ np.asarray(p, float) + self.d)

    def obstacle_height(self, p):
        """Height of the tallest column whose footprint contains p."""
        best = 0.0
        p = np.asarray(p, float)
        for c, r, top in self.columns:
            foot = p - self.n * self.height(p)
            if np.linalg.norm(foot - np.asarray(c, float)) < r:
                best = max(best, top)
        return best

    def clearance(self, p):
        """Height of p above whatever is actually beneath it."""
        return self.height(p) - self.obstacle_height(p)


def min_hand_clearance(fk, arm, q, surf, grip=0.0, links=HAND_LINKS):
    Ts = fk.poses(arm, q, links, gripper=grip)
    return min(surf.clearance(T[:3, 3]) for T in Ts)


def path_clear(fk, arm, q0, q1, surf, margin, grip=0.0, steps=60):
    """Walk the joint-space straight line and report the worst clearance.

    Returns (ok, worst_m, worst_fraction).
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    worst, worst_a = float("inf"), 0.0
    for k in range(steps + 1):
        a = k / steps
        h = min_hand_clearance(fk, arm, q0 + (q1 - q0) * a, surf, grip)
        if h < worst:
            worst, worst_a = h, a
    return worst >= margin, worst, worst_a


def lift_first_detour(fk, arm, q0, q1, surf, margin, grip=0.0, lift_m=0.12):
    """A raise-traverse-descend detour when the straight path does not clear.

    Returns a list of waypoints (excluding q0), or None if even the detour
    cannot be made safe.  Refusing is a valid answer; commanding a path that
    grazes the table is not.
    """
    out = []
    for lift in (lift_m, lift_m * 1.6):
        wp = []
        okall = True
        for q_end in (q0, q1):
            _, _, mid = pads_world(fk, arm, q_end, grip)
            qh, ep, _, _ = ik_link(fk, arm, "robotiq_85_left_finger_tip_link",
                                   mid + surf.n * lift, None, q_end, grip=grip)
            if ep > 0.008:
                okall = False
                break
            wp.append(qh)
        if not okall:
            continue
        legs = [(q0, wp[0]), (wp[0], wp[1]), (wp[1], q1)]
        if all(path_clear(fk, arm, a, b, surf, margin, grip)[0] for a, b in legs):
            out = [wp[0], wp[1], q1]
            return out
    return None


# ----------------------------------------------------------------- self-test
def self_test():                                              # noqa: C901
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("  %-58s %s %s" % (name, "OK" if cond else "FAIL", detail))

    fk = FK()
    q0 = np.array([0.3, 0.4, 1.2, -1.3, -0.2, -0.5, -1.1])

    print("1. look_at builds the rotation it claims to")
    R = look_at([0.5, 0.0, 1.4], [0.5, 0.3, 1.0])
    axis = R @ np.array([0.0, 0.0, 1.0])
    want = np.array([0.0, 0.3, -0.4]); want /= np.linalg.norm(want)
    chk("optical axis points at the target",
        float(np.linalg.norm(axis - want)) < 1e-9,
        "(%.3f,%.3f,%.3f)" % tuple(axis))
    chk("it is a proper rotation (det +1, not a mirror)",
        abs(float(np.linalg.det(R)) - 1.0) < 1e-9
        and float(np.abs(R.T @ R - np.eye(3)).max()) < 1e-9)

    print("\n2. IK on the CAMERA link, round-tripped through FK")
    Tc, = fk.poses("left", q0, [CAM_LINK])
    tgt_p = Tc[:3, 3] + np.array([0.03, -0.02, 0.04])
    tgt_R = Tc[:3, :3] @ look_at([0, 0, 0], [0.10, 0.05, 1.0])
    q, ep, er, it = ik_link(fk, "left", CAM_LINK, tgt_p, tgt_R, q0)
    chk("camera position solved", ep < 0.0005, "(%.3f mm, %d iters)" % (ep * 1000, it))
    chk("camera orientation solved", math.degrees(er) < 0.5,
        "(%.3f deg)" % math.degrees(er))
    Tc2, = fk.poses("left", q, [CAM_LINK])
    chk("FK of the solution reproduces the target",
        float(np.linalg.norm(Tc2[:3, 3] - tgt_p)) < 0.0005)

    print("\n3. position-only IK leaves the wrist free and still lands")
    q3, ep3, _, _ = ik_link(fk, "left", CAM_LINK, tgt_p, None, q0)
    chk("position-only reaches the point", ep3 < 0.0005, "(%.3f mm)" % (ep3 * 1000))

    print("\n4. it agrees with the grasp solver the repository already trusts")
    import plan_pick_left as ppl
    ppl.set_arm("left")
    Tl, Tr, Tee = fk.poses("left", q0, PAD_LINKS + ["end_effector_link"],
                           gripper=0.447)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    tgt = mid + np.array([0.02, 0.01, -0.03])
    qa, epa, era, _ = ppl.ik(tgt, Tee[:3, :3], q0, grip=0.447)
    # the same question, asked of this module: put the EE where plan_pick_left
    # would have put it, and check the PADS land in the same place
    Tl2, Tr2, _ = fk.poses("left", qa, PAD_LINKS + ["end_effector_link"],
                           gripper=0.447)
    mid_a = (Tl2[:3, 3] + Tr2[:3, 3]) / 2.0
    qb, epb, erb, _ = ik_link(fk, "left", "end_effector_link",
                              fk.poses("left", qa, ["end_effector_link"])[0][:3, 3],
                              Tee[:3, :3], q0)
    Tl3, Tr3 = fk.poses("left", qb, PAD_LINKS, gripper=0.447)
    mid_b = (Tl3[:3, 3] + Tr3[:3, 3]) / 2.0
    chk("plan_pick_left reached its own target", epa < 0.001,
        "(%.3f mm)" % (epa * 1000))
    chk("both solvers put the pads within 1 mm of each other",
        float(np.linalg.norm(mid_a - mid_b)) < 0.001,
        "(%.3f mm)" % (np.linalg.norm(mid_a - mid_b) * 1000))

    print("\n5. the transit check, against a CONSTRUCTED surface")
    # The known answer is computed from the link positions directly, NOT
    # guessed from the pad midpoint: the lowest part of the hand at a given
    # pose is whichever link happens to hang lowest, and asserting "about
    # 300 mm" would only be testing that guess.
    Ts = fk.poses("left", q0, HAND_LINKS, gripper=0.0)
    zs = np.array([T[2, 3] for T in Ts])
    plane_z = zs.min() - 0.150                          # 150 mm under the hand
    flat = Surface([0, 0, 1.0], -plane_z)
    want = float(zs.min() - plane_z)
    got = min_hand_clearance(fk, "left", q0, flat)
    chk("clearance is the LOWEST link's height above the plane",
        abs(got - want) < 1e-9, "(%.4f m, want %.4f)" % (got, want))
    high = Surface([0, 0, 1.0], -(zs.min() + 0.05))     # surface ABOVE the hand
    chk("a surface above the hand reads negative",
        min_hand_clearance(fk, "left", q0, high) < 0)
    okp, worst, _ = path_clear(fk, "left", q0, q0, flat, 0.05)
    chk("a null path is clear", okp, "(worst %.3f m)" % worst)
    okp2, _, _ = path_clear(fk, "left", q0, q0, flat, 0.50)
    chk("... and is REFUSED against a margin it cannot meet", not okp2)

    print("\n6. an object COLUMN raises the floor, which a plane check misses")
    low = Ts[int(np.argmin(zs))][:3, 3]
    foot = np.array([low[0], low[1], plane_z])
    col = Surface([0, 0, 1.0], -plane_z, columns=[(foot, 0.25, 0.100)])
    want_col = min(float(z - plane_z
                         - (0.100 if np.linalg.norm(T[:2, 3] - foot[:2]) < 0.25
                            else 0.0))
                   for T, z in zip(Ts, zs))
    got_col = min_hand_clearance(fk, "left", q0, col)
    chk("a 100 mm column subtracts 100 mm under its own footprint",
        abs(got_col - want_col) < 1e-9,
        "(%.4f m, plane-only %.4f)" % (got_col, got))
    chk("... and that is strictly worse than the plane alone", got_col < got)
    far = Surface([0, 0, 1.0], -plane_z,
                  columns=[(foot + np.array([2.0, 0, 0]), 0.25, 0.100)])
    chk("a column somewhere else changes nothing",
        abs(min_hand_clearance(fk, "left", q0, far) - got) < 1e-12)

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else 0)
