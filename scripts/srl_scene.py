#!/usr/bin/env python3
"""Table and object geometry from an EYE-IN-HAND depth camera.

    python3 scripts/srl_scene.py --self-test

THE ONE IDEA THIS MODULE IS BUILT AROUND
----------------------------------------
The gripper camera's mount rotation is wrong by ~8.5 deg and it turns with the
wrist, so anything expressed in robot coordinates inherits 34-59 mm of error.
But two things are rigidly attached to each other and to the camera: the
camera and the FINGER PADS.  Their relative pose comes from FK down one rigid
chain and does NOT pass through the mount error.

So the quantity that decides when to stop descending --

    pad_height = (pad position IN THE CAMERA FRAME) . n_cam + d_cam

-- is computed entirely inside the camera frame and is exact to the quality of
the plane fit (0.46 mm RMS measured).  The arm can be badly mis-calibrated and
this number is still right.  That is why descent is guarded by THIS and not by
a table height in base coordinates, which is what let the gripper be driven
into the table.

Robot-frame object poses ARE still produced, because you need them to plan
approaches; they carry the extrinsic error and are labelled as estimates.
Use them to get CLOSE; use pad_height to decide when to STOP.
"""
import argparse
import math

import numpy as np


# --------------------------------------------------------------------- plane
def fit_plane(P, iters=400, tol=0.006, seed=0):
    """RANSAC plane, refined on its inliers. Returns (n, d, inlier_mask).

    The normal is oriented to point TOWARDS THE CAMERA ORIGIN, so that
    `P @ n + d` is positive for anything standing proud of the surface.  The
    obvious alternative -- orient by the mean point -- is degenerate here,
    because the mean of a table's own points lies ON the plane and the sign
    comes out arbitrary.  That bug silently produced an empty object set.
    """
    P = np.asarray(P, float)
    rng = np.random.default_rng(seed)
    best, best_count = None, -1
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        p = P[idx]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n = n / L
        d = -float(n @ p[0])
        c = int((np.abs(P @ n + d) < tol).sum())
        if c > best_count:
            best_count, best = c, (n, d)
    n, d = best
    m = np.abs(P @ n + d) < tol
    Q = P[m]
    cen = Q.mean(0)
    C = np.cov((Q - cen).T)
    _, V = np.linalg.eigh(C)
    n = V[:, 0] / np.linalg.norm(V[:, 0])
    d = -float(n @ cen)
    if d < 0:                      # point the normal at the camera origin
        n, d = -n, -d
    return n, d, np.abs(P @ n + d) < tol


def plane_rms_mm(P, n, d, mask):
    return float(np.sqrt(((np.asarray(P)[mask] @ n + d) ** 2).mean()) * 1000)


# ------------------------------------------------------------------- objects
def segment_objects(P, n, d, min_h=0.012, max_h=0.30, min_pts=60,
                    cluster_gap=0.030):
    """Clusters standing proud of the plane. All geometry in the INPUT frame.

    Objects are cut by HEIGHT ABOVE THE MEASURED SURFACE, not by colour, so a
    scan finds whatever is on the table rather than only things somebody wrote
    an HSV range for.  Height also rejects the failure that colour alone
    cannot: a coloured mark painted ON the table has no height.
    """
    P = np.asarray(P, float)
    h = P @ n + d
    sel = (h > min_h) & (h < max_h)
    pts = P[sel]
    if len(pts) < min_pts:
        return []
    # cluster in the plane, using a grid so this stays O(n)
    e1 = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    uv = np.stack([pts @ e1, pts @ e2], 1)
    key = np.floor(uv / cluster_gap).astype(int)
    buckets = {}
    for i, k in enumerate(map(tuple, key)):
        buckets.setdefault(k, []).append(i)
    seen, groups = set(), []
    for k in buckets:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.extend(buckets[c])
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    nb = (c[0] + du, c[1] + dv)
                    if nb in buckets and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(comp) >= min_pts:
            groups.append(np.array(comp))
    out = []
    for g in groups:
        Q = pts[g]
        hq = Q @ n + d
        a, b = Q @ e1, Q @ e2
        foot = Q.mean(0) - n * float(Q.mean(0) @ n + d)
        out.append({
            "n_pts": int(len(Q)),
            "top_mm": float(hq.max() * 1000),
            "size_mm": (float((a.max() - a.min()) * 1000),
                        float((b.max() - b.min()) * 1000),
                        float(hq.max() * 1000)),
            "centre": foot + n * (hq.max() / 2.0),   # middle of the body
            "top_centre": foot + n * hq.max(),
        })
    out.sort(key=lambda o: -o["n_pts"])
    return out


# ------------------------------------------------- the extrinsic-free height
def pad_height_above(pad_in_cam, n, d):
    """Height of the pad midpoint above the surface, IN THE CAMERA FRAME.

    No hand-eye transform appears here.  This is the number to descend on.
    """
    return float(np.asarray(pad_in_cam) @ n + d)


def descent_step(pad_in_cam, target_h, n, d, max_step=0.02, floor=0.0):
    """How far to move the pads ALONG -n to reach `target_h`, clipped.

    Returns a vector in the camera frame.  Positive `target_h` keeps the pads
    above the surface; the step never asks to go below `floor`.
    """
    h = pad_height_above(pad_in_cam, n, d)
    want = max(target_h, floor) - h
    want = float(np.clip(want, -max_step, max_step))
    return np.asarray(n, float) * want, h


# ----------------------------------------------------------------- self-test
def _synth(n_true, d_true, cubes, noise=0.0005, seed=1):
    """A synthetic table with cubes on it, in camera coordinates."""
    rng = np.random.default_rng(seed)
    e1 = np.cross(n_true, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n_true, e1)
    origin = -n_true * d_true
    pts = []
    for _ in range(9000):                       # the table
        u, v = rng.uniform(-0.35, 0.35, 2)
        pts.append(origin + e1 * u + e2 * v)
    for (cu, cv, size) in cubes:                # cubes standing on it
        s = size / 2.0
        for _ in range(1400):
            u = rng.uniform(-s, s)
            v = rng.uniform(-s, s)
            face = rng.integers(0, 3)
            if face == 0:
                p = origin + e1 * (cu + u) + e2 * (cv + v) + n_true * size
            elif face == 1:
                p = origin + e1 * (cu + s) + e2 * (cv + v) + n_true * (rng.uniform(0, size))
            else:
                p = origin + e1 * (cu + u) + e2 * (cv + s) + n_true * (rng.uniform(0, size))
            pts.append(p)
    P = np.array(pts)
    return P + rng.normal(0, noise, P.shape)


def self_test():
    ok = True

    def chk(name, got, want, tol, unit=""):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("  %-46s %9.3f vs %9.3f %-3s %s"
              % (name, got, want, unit, "OK" if good else "FAIL"))

    print("plane and object recovery on CONSTRUCTED geometry")
    n_true = np.array([0.10, -0.25, 0.96])
    n_true /= np.linalg.norm(n_true)
    d_true = 0.42
    cubes = [(0.05, -0.02, 0.040), (-0.14, 0.11, 0.060)]
    P = _synth(n_true, d_true, cubes)

    n, d, m = fit_plane(P)
    ang = math.degrees(math.acos(min(1.0, abs(float(n @ n_true)))))
    chk("plane normal error", ang, 0.0, 0.5, "deg")
    chk("plane offset error", abs(d - d_true) * 1000, 0.0, 2.0, "mm")
    chk("plane RMS", plane_rms_mm(P, n, d, m), 0.0, 1.5, "mm")

    objs = segment_objects(P, n, d)
    chk("objects found", len(objs), 2, 0)
    if len(objs) >= 2:
        tops = sorted(o["top_mm"] for o in objs)
        chk("shorter cube height", tops[0], 40.0, 3.0, "mm")
        chk("taller cube height", tops[1], 60.0, 3.0, "mm")

    print("\nthe extrinsic-free descent guard")
    # a pad 150 mm above the surface must be reported as 150 mm...
    pad = -n_true * d_true + n_true * 0.150
    chk("pad height", pad_height_above(pad, n, d) * 1000, 150.0, 2.0, "mm")
    # ...and must remain right when the CAMERA POSE is unknown/wrong, because
    # nothing here uses a hand-eye transform. Rotate the whole scene: the
    # measured plane rotates with it, so the height is invariant.
    th = math.radians(8.45)                     # our real mount error
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    n2, d2, _ = fit_plane(P @ R.T)
    chk("pad height after an 8.45 deg frame error",
        pad_height_above(R @ pad, n2, d2) * 1000, 150.0, 2.0, "mm")

    step, h = descent_step(pad, 0.020, n, d, max_step=0.02)
    chk("descent step is clipped", float(np.linalg.norm(step)) * 1000, 20.0, 0.1, "mm")
    chk("descent step points DOWN", float(step @ n), -0.02, 1e-6)
    pad_low = -n_true * d_true + n_true * 0.010
    step2, _ = descent_step(pad_low, 0.020, n, d, max_step=0.02)
    chk("below target, step points UP", float(step2 @ n), 0.010, 1e-3)

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else 0)
