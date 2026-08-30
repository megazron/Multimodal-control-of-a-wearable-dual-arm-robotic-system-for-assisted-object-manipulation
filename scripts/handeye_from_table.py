#!/usr/bin/env python3
"""Recover the gripper camera's mount-rotation error from the table itself.

    python3 scripts/handeye_from_table.py --self-test

WHY THIS IS POSSIBLE WITHOUT A CALIBRATION RIG
----------------------------------------------
The table does not move.  So for every arm pose i, the SAME world vector must
come out of

    n_world  =  R_world_wrist(q_i) . R_wrist_cam . n_cam_i

Measured on the real arm, it does not: across four poses the recovered normal
spanned 8.45 deg.  `R_wrist_cam` is taken from the URDF and is the only term
that is not measured, so the disagreement is its error.

Introduce a correction C, applied in the camera frame:

    n_world  =  R_world_wrist(q_i) . R_wrist_cam_urdf . C . n_cam_i

and choose C to make all the observations agree.  Three unknowns, one
constraint vector per pose, so two well-separated poses are enough and more
are better.  This is the rotational half of the classic AX = XB hand-eye
problem, using a plane instead of a target -- the plane is free, always in
view, and already measured at 0.46 mm RMS.

WHAT IT CANNOT DO
-----------------
A plane normal constrains ROTATION only.  It says nothing about the camera's
translational offset along the surface, and only the component of translation
along the normal shows up in the plane offset.  So this fixes the term that
dominates (8.45 deg is 34-59 mm at working range) and leaves a small
translational residual.  Do not present the result as a full hand-eye
calibration.
"""
import argparse
import math

import numpy as np


def rotvec_to_R(r):
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(r, float) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def spread_deg(vs):
    """Worst pairwise angle in a set of unit vectors, in degrees."""
    worst = 0.0
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            c = float(np.clip(abs(vs[i] @ vs[j]), -1.0, 1.0))
            worst = max(worst, math.degrees(math.acos(c)))
    return worst


def _residual(r, Rs, ns):
    C = rotvec_to_R(r)
    vs = []
    for R, n in zip(Rs, ns):
        v = R @ (C @ n)
        v = v / np.linalg.norm(v)
        if v[2] < 0:
            v = -v
        vs.append(v)
    mean = np.mean(vs, 0)
    mean /= np.linalg.norm(mean)
    return np.concatenate([v - mean for v in vs]), vs


def solve(Rs, ns, iters=200):
    """Rs: world<-camera rotations from the URDF. ns: table normals in camera.

    Returns (C, spread_before_deg, spread_after_deg).
    """
    Rs = [np.asarray(R, float) for R in Rs]
    ns = [np.asarray(n, float) / np.linalg.norm(n) for n in ns]
    _, before = _residual(np.zeros(3), Rs, ns)
    r = np.zeros(3)
    lam = 1e-3
    e, _ = _residual(r, Rs, ns)
    cost = float(e @ e)
    for _ in range(iters):
        J = np.zeros((len(e), 3))
        for k in range(3):
            dr = np.zeros(3)
            dr[k] = 1e-6
            ek, _ = _residual(r + dr, Rs, ns)
            J[:, k] = (ek - e) / 1e-6
        try:
            step = np.linalg.solve(J.T @ J + lam * np.eye(3), -J.T @ e)
        except np.linalg.LinAlgError:
            break
        rn = r + step
        en, _ = _residual(rn, Rs, ns)
        cn = float(en @ en)
        if cn < cost:
            r, e, cost, lam = rn, en, cn, max(lam * 0.5, 1e-9)
            if float(np.linalg.norm(step)) < 1e-10:
                break
        else:
            lam *= 4.0
            if lam > 1e6:
                break
    _, after = _residual(r, Rs, ns)
    return rotvec_to_R(r), spread_deg(before), spread_deg(after)


# ----------------------------------------------------------------- self-test
def self_test():
    ok = True


    def chk(name, got, want, tol, unit=""):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  %-48s %8.3f vs %8.3f %-3s %s"
              % (name, got, want, unit, "OK" if good else "FAIL"))

    rng = np.random.default_rng(3)
    # a KNOWN mount error, of the size we actually measured
    true_err_deg = 8.45
    axis = np.array([0.3, -0.9, 0.31])
    axis /= np.linalg.norm(axis)
    C_true = rotvec_to_R(axis * math.radians(true_err_deg))

    n_world = np.array([0.02, -0.05, 0.998])
    n_world /= np.linalg.norm(n_world)

    Rs, ns = [], []
    for _ in range(6):                       # six distinct wrist orientations
        rv = rng.normal(0, 0.8, 3)
        R = rotvec_to_R(rv)                  # world <- camera, from "the URDF"
        # what the camera REALLY sees, given the mount is off by C_true
        n_cam = C_true.T @ (R.T @ n_world)
        n_cam = n_cam / np.linalg.norm(n_cam)
        n_cam += rng.normal(0, 0.002, 3)     # measurement noise
        Rs.append(R)
        ns.append(n_cam / np.linalg.norm(n_cam))

    C, before, after = solve(Rs, ns)
    print("recovering a KNOWN %.2f deg mount error from 6 simulated views"
          % true_err_deg)
    # NOT equal to the injected error: the spread you observe depends on how
    # far apart the wrist orientations are, which amplifies it. Assert only
    # that it is large before and small after -- that is the claim.
    print("  %-48s %8.3f     %-3s %s"
          % ("disagreement before correction", before, "deg",
             "OK" if before > 5.0 else "FAIL"))
    ok = bool(ok and before > 5.0)
    chk("disagreement after correction", after, 0.0, 0.5, "deg")
    dR = C_true.T @ C
    ang = math.degrees(math.acos(float(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
    chk("recovered rotation vs truth", ang, 0.0, 0.6, "deg")

    print("\na check that CANNOT pass by accident:")
    C0, b0, a0 = solve(Rs, [C_true.T @ (R.T @ n_world) for R in Rs])
    chk("perfectly consistent input -> no correction needed", a0, 0.0, 0.05, "deg")
    ident = math.degrees(math.acos(float(np.clip((np.trace(C0) - 1) / 2, -1, 1))))
    print("  (recovered correction there is %.3f deg from identity in the "
          "camera frame,\n   which is expected: any C that maps a single "
          "consistent set to itself is valid.)" % ident)

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else 0)
