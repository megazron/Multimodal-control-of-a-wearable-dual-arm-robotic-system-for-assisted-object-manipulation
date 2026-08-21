#!/usr/bin/env python3
"""Camera -> robot base, by projecting the ARM and comparing depth.

    python3 scripts/real_calibration/solve_extrinsic.py --self-test
    python3 scripts/real_calibration/solve_extrinsic.py --dir recordings/...

WHY NOT CENTROIDS. Two centroid-matching attempts were made on 2026-08-21 and
both failed, at 267 mm and 354 mm residual against a 30 mm grasp tolerance:

  * matching "pixels nearer than the background" to "model points far from
    other poses" compared two DIFFERENT selections and their centroids were
    never going to correspond;
  * matching them pairwise with one consistent rule fixed the definition and
    still failed, because the camera sees the arm's VISIBLE FRONT SURFACE and
    the model is the capsule AXIS. How much of the arm is visible, and which
    side, changes with every pose -- so the offset between those two centroids
    is not constant and does not absorb into t.

A centroid throws away the shape. This uses it. The arm's capsule chain is
projected into the image and the depth it PREDICTS is compared with the depth
that was MEASURED, pixel by pixel, over every pose at once. Thousands of
constraints per pose instead of one, and the residual is in the same units as
the thing we care about.

WHAT IS SOLVED: six numbers, as a rotation vector and a translation. Reported
with the achieved per-pixel depth residual, and cross-validated by holding
poses out -- a fit residual alone cannot tell over-fitting from agreement.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "config"))

import numpy as np

TUBE_R = 0.05          # the capsule radius the guard models the arm with
FG_THRESH = 0.08       # metres nearer than background to count as "arm"


def rodrigues(r):
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def project(P_robot, rvec, t, K):
    """Model points -> (u, v, z_cam)."""
    R = rodrigues(rvec)
    Pc = P_robot @ R.T + t
    fx, fy, cx, cy, w, h = K
    z = Pc[:, 2]
    good = z > 0.15
    u = np.full(len(Pc), -1.0)
    v = np.full(len(Pc), -1.0)
    u[good] = Pc[good, 0] * fx / z[good] + cx
    v[good] = Pc[good, 1] * fy / z[good] + cy
    return u, v, z, good


def render(model, rvec, t, K):
    """Z-buffer the model into a predicted-depth image.

    A DEPTH IMAGE HOLDS THE NEAREST SURFACE, one value per pixel. Many model
    points land on the same pixel at different ranges, so comparing each of
    them against that single value scores the occluded ones as errors even
    when the transform is exactly right -- which is what the self-test caught:
    the cost was not zero at the known answer. Taking the per-pixel MINIMUM is
    what the sensor actually reports.
    """
    fx, fy, cx, cy, w, h = K
    pred = np.full((h, w), np.inf, np.float32)
    u, v, z, good = project(model, rvec, t, K)
    if not good.any():
        return pred
    iu = np.round(u[good]).astype(int)
    iv = np.round(v[good]).astype(int)
    ok = (iu >= 0) & (iu < w) & (iv >= 0) & (iv < h)
    if not ok.any():
        return pred
    np.minimum.at(pred, (iv[ok], iu[ok]), z[good][ok] - TUBE_R)
    return pred


def residuals(poses, rvec, t, K, huber=0.10):
    """Robust per-PIXEL depth residual over every pose."""
    tot, n = 0.0, 0
    for pz in poses:
        pred = render(pz["model"], rvec, t, K)
        use = np.isfinite(pred) & (pz["depth"] > 0) & pz["fg"]
        if use.sum() < 20:
            continue
        d = np.abs(pred[use] - pz["depth"][use])
        d = np.where(d < huber, d ** 2 / (2 * huber), d - huber / 2)
        tot += float(d.sum())
        n += int(use.sum())
    if n == 0:
        return 1e6, 0
    return tot / n, n


def coverage(poses, rvec, t, K):
    """Fraction of observed arm pixels the model actually lands on.

    A transform can get a tiny residual by projecting the arm off the image
    entirely and scoring the handful of points that remain. Coverage is what
    stops that being called a fit.
    """
    hit = tot = 0
    for pz in poses:
        m = np.isfinite(render(pz["model"], rvec, t, K))
        hit += int((m & pz["fg"]).sum())
        tot += int(pz["fg"].sum())
    return hit / max(tot, 1)


def rvec_of(R):
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-8:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def umeyama(X, Y):
    Xc, Yc = X.mean(0), Y.mean(0)
    U, S, Vt = np.linalg.svd((X - Xc).T @ (Y - Yc))
    V = Vt.T
    R = V @ np.diag([1, 1, np.sign(np.linalg.det(V @ U.T))]) @ U.T
    return R, Yc - R @ Xc


def measured_clouds(poses, K):
    """Per pose: the ARM pixels, as camera-frame points."""
    fx, fy, cx, cy, w, h = K
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    out = []
    for pz in poses:
        m = pz["fg"] & (pz["depth"] > 0)
        z = pz["depth"][m]
        out.append(np.stack([(uu[m] - cx) * z / fx,
                             (vv[m] - cy) * z / fy, z], 1))
    return out


def _visible(Pc, K, tol=0.02):
    """Which camera-frame points are the NEAREST along their own pixel ray."""
    fx, fy, cx, cy, w, h = K
    z = Pc[:, 2]
    ok = z > 0.15
    u = np.full(len(Pc), -1.0)
    v = np.full(len(Pc), -1.0)
    u[ok] = Pc[ok, 0] * fx / z[ok] + cx
    v[ok] = Pc[ok, 1] * fy / z[ok] + cy
    iu = np.round(u).astype(int)
    iv = np.round(v).astype(int)
    inim = ok & (iu >= 0) & (iu < w) & (iv >= 0) & (iv < h)
    zbuf = np.full((h, w), np.inf, np.float32)
    np.minimum.at(zbuf, (iv[inim], iu[inim]), z[inim])
    vis = np.zeros(len(Pc), bool)
    vis[inim] = z[inim] <= zbuf[iv[inim], iu[inim]] + tol
    return vis


def icp(poses, clouds, rvec, t, K, iters=40, trim=0.75):
    """Refine (rvec, t) by matching model points to measured points.

    THE RASTERISED DEPTH COST CANNOT BE OPTIMISED DIRECTLY. Rounding to pixels
    makes it piecewise constant, so Nelder-Mead sat still and the self-test
    recovered a transform 117 deg wrong from a known answer. Nearest-neighbour
    distance is continuous in the pose, which is what a descent needs.

    Trimmed: the model is a capsule AXIS chain and the measurement is a
    partial FRONT SURFACE, so a fraction of points have no honest partner and
    including them drags the fit.
    """
    from scipy.spatial import cKDTree
    trees = [cKDTree(c) for c in clouds if len(c) > 10]
    use = [(p, c) for p, c in zip(poses, clouds) if len(c) > 10]
    if not trees:
        return rvec, t, float("inf")
    R = rodrigues(rvec)
    last = float("inf")
    for _ in range(iters):
        A, B = [], []
        for (pz, _c), tree in zip(use, trees):
            P = pz["model"] @ R.T + t
            # ONLY THE SURFACE THE CAMERA COULD SEE.
            #
            # The model is a solid capsule chain; the measurement is the front
            # skin. Matching the whole model against a front-only cloud pulls
            # the fit backwards by roughly half the arm's thickness, and no
            # amount of trimming removes a bias that is present in every
            # correspondence. Recovered 8.60 deg / 180 mm from a KNOWN
            # transform before this cull, which is the self-test earning its
            # keep. So: keep the point nearest the camera along each ray, and
            # step it toward the camera by the tube radius, because the skin
            # is TUBE_R in front of the axis.
            vis = _visible(P, K)
            if vis.sum() < 20:
                continue
            Pv = P[vis]
            ray = Pv / np.maximum(np.linalg.norm(Pv, axis=1, keepdims=True),
                                  1e-9)
            Pv = Pv - ray * TUBE_R
            d, idx = tree.query(Pv, k=1)
            if len(d) == 0:
                continue
            keep = d <= np.quantile(d, trim)
            src = pz["model"][vis][keep]
            # the same TUBE_R step, expressed back in the model frame, so the
            # Umeyama solve sees a consistent pair
            A.append(src - (ray[keep] * TUBE_R) @ R)
            B.append(_c[idx[keep]])
        if not A:
            break
        A = np.concatenate(A)
        B = np.concatenate(B)
        R, t = umeyama(A, B)
        err = float(np.sqrt(np.mean(np.sum(
            ((A @ R.T + t) - B) ** 2, axis=1))))
        if abs(last - err) < 1e-7:
            break
        last = err
    return rvec_of(R), t, last


def solve(poses, K, x0=None, verbose=True):
    """Coarse global guesses, then ICP. Scored by the DEPTH residual."""
    clouds = measured_clouds(poses, K)
    starts = []
    if x0 is not None:
        starts.append(np.asarray(x0, float))
    rng = np.random.default_rng(0)
    for _ in range(60):
        ang = rng.uniform(0, 2 * np.pi)
        rad = rng.uniform(1.5, 5.0)
        cam = np.array([rad * np.cos(ang), rad * np.sin(ang),
                        rng.uniform(0.8, 3.0)])
        fwd = -cam / np.linalg.norm(cam)
        up = np.array([0, 0, -1.0])
        right = np.cross(up, fwd)
        nr = np.linalg.norm(right)
        if nr < 1e-6:
            continue
        right /= nr
        up2 = np.cross(fwd, right)
        Rrc = np.stack([right, up2, fwd], 1).T
        starts.append(np.concatenate([rvec_of(Rrc), -Rrc @ cam]))

    # COARSE INIT FROM CENTROIDS, THEN ICP.
    #
    # ICP is a local method. On clean synthetic data the random ring of
    # guesses happens to contain a good basin and it converges; on the real
    # recording every one of them missed, and the "best" answer projected
    # onto 20 pixels with 0% coverage. Centroid matching is far too crude to
    # BE the calibration -- it was measured at 267 mm earlier today -- but it
    # is entirely good enough to say roughly where the camera is, which is
    # all ICP needs to start from.
    cen_mod = np.array([p["model"].mean(0) for p in poses])
    cen_obs = np.array([c.mean(0) for c in clouds if len(c) > 10])
    if len(cen_mod) == len(cen_obs) and len(cen_mod) >= 3:
        Rc, tc = umeyama(cen_mod, cen_obs)
        starts.insert(0, np.concatenate([rvec_of(Rc), tc]))
        if verbose:
            cam = -Rc.T @ tc
            print("centroid init: camera near %s"
                  % np.round(cam, 2).tolist())

    best = None
    for s in starts:
        rv, t, err = icp(poses, clouds, s[:3], s[3:], K)
        r, n = residuals(poses, rv, t, K)
        cov = coverage(poses, rv, t, K)
        # COVERAGE IS A GATE, NOT A TIE-BREAK. A transform that throws the
        # model off the image scores a tiny residual on whatever few pixels
        # remain, and that is how a 0%-coverage answer won the last round.
        if cov < 0.10:
            continue
        score = r + 2.0 * (1.0 - cov)
        if best is None or score < best[0]:
            best = (score, np.concatenate([rv, t]), r, n, cov)
    if best is None:
        return None
    return best


def self_test(verbose=True):
    """Generate depth from a KNOWN transform, then recover it."""
    rng = np.random.default_rng(3)
    K = (600.0, 600.0, 320.0, 240.0, 640, 480)
    fx, fy, cx, cy, w, h = K
    rv_true = np.array([0.15, -2.6, 0.35])
    t_true = np.array([0.4, -0.2, 3.2])
    poses = []
    # A CHAIN, not a blob. The first version of this test used spherical
    # Gaussian clouds, and a sphere's front surface is rotationally symmetric
    # about the view axis -- so the rotation was barely observable and the
    # solver "failed" at 8 deg on data that did not constrain the answer. A
    # real arm is a long thin linkage, which is exactly what pins rotation, so
    # the synthetic model is now three connected segments per pose.
    for p in range(8):
        origin = np.array([0.35, -0.30, 1.25])
        pts = [origin]
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        cur = origin.copy()
        for seg in range(3):
            d = d + rng.normal(scale=0.7, size=3)
            d /= np.linalg.norm(d)
            nxt = cur + d * rng.uniform(0.25, 0.35)
            pts.append(nxt)
            cur = nxt
        pts = np.array(pts)
        M = []
        for i in range(len(pts) - 1):
            ts = np.linspace(0, 1, 120)[:, None]
            axis = pts[i] * (1 - ts) + pts[i + 1] * ts
            M.append(axis + rng.normal(scale=0.012, size=axis.shape))
        M = np.concatenate(M)
        pred = render(M, rv_true, t_true, K)
        fg = np.isfinite(pred)
        depth = np.where(fg, pred, 0.0).astype(np.float32)
        poses.append(dict(model=M, depth=depth, fg=fg))
    r0, n0 = residuals(poses, rv_true, t_true, K)
    if verbose:
        print("at the TRUE transform: residual %.6f over %d pixels" % (r0, n0))
    assert r0 < 1e-3, "the cost is not near zero at the true answer"

    best = solve(poses, K, verbose=verbose)
    assert best is not None, "solver returned nothing"
    score, x, r, n, cov = best
    R_t = rodrigues(rv_true)
    R_f = rodrigues(x[:3])
    ang = np.degrees(np.arccos(np.clip(
        (np.trace(R_f @ R_t.T) - 1) / 2, -1, 1)))
    dt = float(np.linalg.norm(x[3:] - t_true))
    if verbose:
        print("recovered: rotation err %.3f deg, translation err %.1f mm, "
              "residual %.5f, coverage %.0f%%"
              % (ang, dt * 1000, r, cov * 100))
    assert ang < 2.0 and dt < 0.05, (
        "SOLVER CANNOT RECOVER A KNOWN TRANSFORM (%.2f deg, %.1f mm)"
        % (ang, dt * 1000))

    # and it must SCORE BADLY on a wrong transform
    rbad, _ = residuals(poses, rv_true + 0.25, t_true + 0.3, K)
    if verbose:
        print("a deliberately wrong transform scores %.4f (vs %.6f) -> %s"
              % (rbad, r0, "detected" if rbad > 10 * max(r0, 1e-6)
                 else "NOT DETECTED"))
    assert rbad > 10 * max(r0, 1e-6)
    if verbose:
        print("solve_extrinsic self-test PASSED")
    return True


def load(dirpath, arm):
    d = os.path.join(dirpath, arm)
    bg = os.path.join(d, "background.npz")
    if not os.path.exists(bg):
        sys.exit("no background.npz in %s -- run the sweep first" % d)
    B = np.load(bg)["depth"]
    Kfound = None
    import solve_home_pose as SHP
    sc = SHP.Scorer()
    MOV = SHP.MOVING_FIRST_SEG
    poses = []
    for f in sorted(glob.glob(os.path.join(d, "*.npz"))):
        if f.endswith("background.npz"):
            continue
        z = np.load(f)
        depth = z["depth"]
        q = z["q"]
        fg = (depth > 0) & (B > 0) & ((B - depth) > FG_THRESH)
        if fg.sum() < 300:
            continue
        model = sc.segments(arm, q)[MOV:].reshape(-1, 3)
        if Kfound is None and "K" in z.files:
            kv = [float(v) for v in z["K"]]
            # width and height are COUNTS, not measurements. They come back
            # from the npz as floats and every array allocation downstream
            # wants ints.
            Kfound = (kv[0], kv[1], kv[2], kv[3], int(kv[4]), int(kv[5]))
        poses.append(dict(model=model, depth=depth, fg=fg, file=f))
    # THE INTRINSICS TRAVEL WITH THE DATA. A sidecar K.json can be lost,
    # edited, or -- as here -- simply never written, and a depth image with
    # the wrong K deprojects into a plausible, wrong cloud. Reading it out of
    # the same file as the pixels makes that impossible.
    return poses, Kfound


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(
        WS, "recordings", "real_calibration"))
    ap.add_argument("--arm", default="left")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test(True) else 1
    print("validating the solver first:")
    self_test(verbose=True)
    print()
    poses, K = load(a.dir, a.arm)
    print("loaded %d poses with usable arm pixels" % len(poses))
    if len(poses) < 6:
        sys.exit("too few poses (%d) -- need at least 6" % len(poses))
    if K is None:
        sys.exit("no camera intrinsics found in the recording")
    print("intrinsics from the recording: %s" % (np.round(K, 2).tolist(),))
    best = solve(poses, tuple(K))
    if best is None:
        sys.exit("NO TRANSFORM cleared the 10% coverage gate. The model never "
                 "lands on the observed arm pixels, so nothing here is a "
                 "calibration -- refusing to write one.")
    score, x, r, n, cov = best
    print("fit: residual %.4f m over %d pixels, coverage %.0f%%"
          % (r, n, cov * 100))
    # hold-out
    k = max(1, len(poses) // 4)
    tr, te = poses[k:], poses[:k]
    b2 = solve(tr, tuple(K), x0=x)
    rte, nte = residuals(te, b2[1][:3], b2[1][3:], tuple(K))
    print("held-out residual %.4f m over %d pixels" % (rte, nte))
    R = rodrigues(x[:3])
    cam = -R.T @ x[3:]
    print("camera in robot frame %s (%.2f m from base)"
          % (np.round(cam, 3).tolist(), np.linalg.norm(cam)))
    json.dump(dict(rvec=x[:3].tolist(), t=x[3:].tolist(),
                   R_robot_to_cam=R.tolist(), camera_in_robot=cam.tolist(),
                   residual_m=r, heldout_residual_m=rte, coverage=cov,
                   n_poses=len(poses)),
              open(os.path.join(a.dir, "extrinsic_%s.json" % a.arm), "w"),
              indent=1)
    print("wrote extrinsic_%s.json" % a.arm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
