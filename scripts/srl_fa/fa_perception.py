#!/usr/bin/env python3
"""RGB-D -> the table, what is standing on it, and where the hand may go.

    python3 scripts/srl_fa/fa_perception.py --self-test

WHAT IS HERE THAT IS NOT ALREADY IN srl_scene.py
================================================
`srl_scene` fits the plane and cuts objects out of the cloud, and it is
imported here rather than copied.  Three things full autonomy needs are added:

1.  **Point membership, so an object can be given a COLOUR and a NAME.**
    `srl_scene.segment_objects` returns geometry only.  "Pick the green cube"
    needs the cluster's pixels, so the clustering here returns indices and
    every object carries the median hue/saturation/value of its own colour
    pixels, projected through the depth->colour baseline.

2.  **`support_under_pads` -- the descent guard that generalises "do not hit
    the table" to "do not hit ANYTHING".**  The existing guard descends on the
    gap to the fitted PLANE.  That is right over bare table and wrong over the
    cardboard box: the plane is still the table, 120 mm below the rim the
    fingers are about to hit.  This asks a different question -- how high is
    the tallest thing directly beneath each finger pad -- and the descent
    floor is the answer plus a clearance.  It is computed in the CAMERA FRAME,
    so like the plane guard it carries no hand-eye error.

    THE RADIUS IS PER PAD AND SMALL ON PURPOSE.  A radius around the pad
    MIDPOINT would see the object being grasped -- which sits between the pads
    by definition -- and refuse to descend onto it.  Asking about each pad
    separately, over a 20 mm disc, sees the table beside the cube and not the
    cube.

3.  **Naming and matching**, so a scan can be talked about in a sentence:
    stable ids, a colour word, a shape word, and nearest-match lookup.

EVERYTHING GEOMETRIC IN HERE IS IN THE CAMERA'S OWN FRAME unless a function
name says `world`.  That is the same division of labour the rest of the
repository uses: camera-frame quantities are trustworthy to the quality of the
depth data; world-frame quantities carry the ~8.5 deg mount error and are
labelled ESTIMATE.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_scene import fit_plane, plane_rms_mm  # noqa: E402
from srl_scene import segment_objects as _srl_segment  # noqa: E402

# ----------------------------------------------------------------- constants
MIN_OBJ_H = 0.012        # below this it is a mark on the table, not an object
MAX_OBJ_H = 0.40
MIN_OBJ_PTS = 60
CLUSTER_GAP = 0.030      # points closer than this in the plane are one object
PAD_PROBE_R = 0.020      # radius of the disc probed beneath each finger pad
PAD_PROBE_MIN_PTS = 8    # fewer than this and the probe says "I cannot see"


# ------------------------------------------------------------- deprojection
def deproject(depth_m, K, z_min=0.08, z_max=2.0):
    """Depth image -> (points Nx3 in the camera frame, pixel v, pixel u).

    The pixel indices come back because colour is looked up through them.
    Dropping them is what forces a second, inconsistent pass over the image.
    """
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    v, u = np.nonzero((depth_m > z_min) & (depth_m < z_max))
    if len(v) == 0:
        return np.zeros((0, 3)), v, u
    Z = depth_m[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    return P, v, u


def plane_basis(n):
    """Two unit vectors spanning the plane with normal `n`."""
    e1 = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [0.0, 1.0, 0.0])
    e1 = e1 / np.linalg.norm(e1)
    return e1, np.cross(n, e1)


# ------------------------------------------------------------- segmentation
def cluster_indices(P, n, d, min_h=MIN_OBJ_H, max_h=MAX_OBJ_H,
                    min_pts=MIN_OBJ_PTS, gap=CLUSTER_GAP):
    """Indices into P of each blob standing proud of the plane.

    Same grid-flood clustering as `srl_scene.segment_objects` -- and the
    self-test requires the two to agree on a constructed scene, so this cannot
    drift away from the implementation the rest of the repository trusts.
    What it adds is the INDICES, which is the whole reason it exists.
    """
    P = np.asarray(P, float)
    if len(P) == 0:
        return []
    h = P @ n + d
    sel = np.nonzero((h > min_h) & (h < max_h))[0]
    if len(sel) < min_pts:
        return []
    e1, e2 = plane_basis(n)
    uv = np.stack([P[sel] @ e1, P[sel] @ e2], 1)
    key = np.floor(uv / gap).astype(int)
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
            groups.append(sel[np.array(comp)])
    groups.sort(key=lambda g: -len(g))
    return groups


# -------------------------------------------------------------------- colour
_HUE_NAMES = [(0, 8, "red"), (8, 22, "orange"), (22, 33, "yellow"),
              (33, 45, "lime"), (45, 85, "green"), (85, 100, "cyan"),
              (100, 130, "blue"), (130, 160, "purple"), (160, 180, "red")]


def colour_name(h, s, v):
    """OpenCV HSV (H 0-179, S/V 0-255) -> one plain word.

    Achromatic FIRST.  Hue is meaningless at low saturation, and the white
    table reads as a confident random colour if hue is consulted first --
    which is how a table edge becomes "the blue object" in a list the operator
    is meant to trust.
    """
    if v < 55:
        return "black"
    if s < 45:
        return "white" if v > 165 else "grey"
    # Cardboard is a desaturated orange and calling it "orange" next to an
    # orange cube would be a real confusion, so it gets its own word.
    if 8 <= h < 26 and s < 150 and v < 215:
        return "brown"
    for lo, hi, name in _HUE_NAMES:
        if lo <= h < hi:
            return name
    return "unknown"


def shape_name(size_mm, n_pts):
    """A cautious word for the object's form. Never claims more than it knows."""
    w, dpt, hgt = size_mm
    a, b = max(w, dpt), min(w, dpt)
    if hgt < 15:
        return "flat object"
    if b > 1e-6 and a / b < 1.35 and 0.65 < hgt / a < 1.5:
        return "cube" if a < 120 else "box"
    if hgt < 0.45 * a:
        return "tray" if a > 150 else "block"
    if a > 150 or hgt > 150:
        return "box"
    return "block"


# --------------------------------------------------------------------- scene
class Scene:
    """One RGB-D frame, understood. Camera frame throughout.

    Attributes
    ----------
    P, n, d          cloud, plane normal (pointing at the camera) and offset
    inlier_frac      fraction of the cloud on the plane -- the honesty check
    rms_mm           plane fit residual
    standoff_m       perpendicular camera-to-plane distance == d
    incidence_deg    angle between the optical axis and the plane NORMAL.
                     0 deg is straight down at the table; 90 deg is edge on.
    objects          list of dicts, biggest first
    """

    def __init__(self, P, vpix, upix, n, d, inl, objects, colour_shape,
                 stamp=None, q=None, arm=None):
        self.P, self.v, self.u = P, vpix, upix
        self.n, self.d = n, d
        self.inlier_frac = float(inl.mean()) if len(P) else 0.0
        self.rms_mm = plane_rms_mm(P, n, d, inl) if len(P) else float("nan")
        self.objects = objects
        self.colour_shape = colour_shape
        self.stamp, self.q, self.arm = stamp, q, arm

    # -- viewing geometry, all extrinsic-free -----------------------------
    @property
    def standoff_m(self):
        return float(self.d)

    @property
    def incidence_deg(self):
        return math.degrees(math.acos(min(1.0, abs(float(self.n[2])))))

    @property
    def plane_ok(self):
        return (self.inlier_frac >= 0.15 and np.isfinite(self.rms_mm)
                and self.rms_mm < 6.0)

    def surface_point(self):
        """Where the optical axis meets the plane, in the camera frame.

        The centre of what the camera is looking AT, which is the point a
        viewing-geometry correction has to be organised around.
        """
        z = np.array([0.0, 0.0, 1.0])
        denom = float(z @ self.n)
        if abs(denom) < 1e-6:
            return None
        t = -self.d / denom
        return z * t if t > 0 else None

    def describe(self):
        out = ["plane: %.1f%% inliers, %.2f mm RMS, standoff %.3f m, "
               "incidence %.1f deg" % (self.inlier_frac * 100, self.rms_mm,
                                       self.standoff_m, self.incidence_deg)]
        for o in self.objects:
            out.append("  %-6s %-12s %5.0f x %5.0f x %5.0f mm  %6d pts"
                       % (o["colour"], o["shape"], *o["size_mm"], o["n_pts"]))
        return "\n".join(out)


def drop_near(P, centres_radii):
    """Boolean keep-mask removing points close to given camera-frame spheres.

    THE ROBOT'S OWN HAND IS IN ITS OWN PICTURE.  At a top-down scan pose the
    fingers hang into the bottom of the frame, stand 100-200 mm proud of the
    table, and segment beautifully as "a tall grey object".  Excluding them by
    FK -- the pads' position in the camera frame is a rigid chain, so this
    carries no mount error -- is the only way to tell the gripper apart from
    something on the table, and colour cannot do it.
    """
    P = np.asarray(P, float)
    keep = np.ones(len(P), bool)
    for c, r in centres_radii:
        keep &= np.linalg.norm(P - np.asarray(c, float), axis=1) > r
    return keep


def understand(depth_m, depth_K, colour_bgr=None, colour_K=None, T_col_dep=None,
               q=None, arm=None, stamp=None, z_max=2.0,
               min_footprint_mm=25.0, exclude=()):
    """One RGB-D frame -> a Scene. Colour is optional; geometry is not.

    `T_col_dep` is the 4x4 depth->colour transform from FK (a rigid chain, so
    it carries NO mount error).  Without it the colour lookup is skipped
    rather than guessed, and every object comes back colour "unknown" --
    a missing colour must never be silently invented, because the whole point
    of naming is that the operator can check the name against the room.
    """
    P, vpix, upix = deproject(depth_m, depth_K, z_max=z_max)
    if len(P) < 400:
        return None
    n, d, inl = fit_plane(P)
    if d < 0:
        n, d = -n, -d
    # The plane is fitted on EVERYTHING -- excluding the hand first would
    # throw away good table points it happens to sit over.  Only the object
    # segmentation is masked.
    seg_keep = drop_near(P, exclude) if exclude else np.ones(len(P), bool)
    groups = [g[seg_keep[g]] for g in cluster_indices(P, n, d)]
    groups = [g for g in groups if len(g) >= MIN_OBJ_PTS]
    e1, e2 = plane_basis(n)

    hsv = None
    if colour_bgr is not None and colour_K is not None and T_col_dep is not None:
        import cv2
        hsv = cv2.cvtColor(colour_bgr, cv2.COLOR_BGR2HSV)
        Pc = (T_col_dep @ np.c_[P, np.ones(len(P))].T).T[:, :3]
        with np.errstate(divide="ignore", invalid="ignore"):
            uu = Pc[:, 0] * colour_K[0] / Pc[:, 2] + colour_K[2]
            vv = Pc[:, 1] * colour_K[4] / Pc[:, 2] + colour_K[5]
        ok = (np.isfinite(uu) & np.isfinite(vv) & (Pc[:, 2] > 0.02)
              & (uu >= 0) & (uu < hsv.shape[1]) & (vv >= 0) & (vv < hsv.shape[0]))
        cpix = np.full((len(P), 2), -1, int)
        cpix[ok, 0] = vv[ok].astype(int)
        cpix[ok, 1] = uu[ok].astype(int)
    else:
        cpix = None

    objects = []
    for gi, g in enumerate(groups):
        Q = P[g]
        hq = Q @ n + d
        a, b = Q @ e1, Q @ e2
        foot = Q.mean(0) - n * float(Q.mean(0) @ n + d)
        size = (float((a.max() - a.min()) * 1000),
                float((b.max() - b.min()) * 1000),
                float(hq.max() * 1000))
        # A SLIVER IS NOT AN OBJECT.  A depth edge along a table lip clusters
        # into a 4 x 2 x 194 mm "grey box" -- measured on the real arm on
        # 2026-08-30 -- and an operator asked to pick it would be sent at a
        # discontinuity.  Something with no footprint cannot be grasped and
        # must not be listed.
        if max(size[0], size[1]) < min_footprint_mm:
            continue
        col, hsv_med = "unknown", None
        if cpix is not None:
            sel = cpix[g]
            m = sel[:, 0] >= 0
            if m.sum() >= 12:
                px = hsv[sel[m, 0], sel[m, 1]]
                hsv_med = [float(np.median(px[:, 0])), float(np.median(px[:, 1])),
                           float(np.median(px[:, 2]))]
                col = colour_name(*hsv_med)
        objects.append({
            "idx": len(objects),
            "n_pts": int(len(Q)),
            "size_mm": size,
            "height_mm": float(hq.max() * 1000),
            "centre": foot + n * (hq.max() / 2.0),   # mid-height of the body
            "top_centre": foot + n * hq.max(),
            "foot": foot,
            "points": g,
            "colour": col,
            "hsv": hsv_med,
            "shape": shape_name(size, len(Q)),
        })
    return Scene(P, vpix, upix, n, d, inl, objects, cpix is not None,
                 stamp=stamp, q=q, arm=arm)


# ------------------------------------------------------- the descent guards
def pad_height_above(pad_in_cam, n, d):
    """Gap from a point to the fitted plane, in the camera frame."""
    return float(np.asarray(pad_in_cam) @ n + d)


def support_under_pads(P, n, d, pad_points, radius=PAD_PROBE_R,
                       min_pts=PAD_PROBE_MIN_PTS, ignore_idx=None):
    """Height of the tallest thing directly beneath the finger pads.

    THIS IS THE ANSWER TO "HOW DO I DESCEND WITHOUT BANGING SOMETHING".
    Not the table height -- the height of whatever is actually under the pad,
    which over the cardboard box is the box, and over bare table is the table.

    Parameters
    ----------
    pad_points : the finger-pad positions, camera frame, ONE PER PAD.
    ignore_idx : point indices to exclude -- the object being grasped, whose
                 own top must not stop the descent that is aiming at it.

    Returns (height_m, n_seen). A height of None means NOTHING WAS VISIBLE
    under a pad, which is a refusal to descend, not a clear floor.  Reporting
    an unseen column as clear is exactly the failure mode this guard exists to
    remove.
    """
    P = np.asarray(P, float)
    if len(P) == 0:
        return None, 0
    keep = np.ones(len(P), bool)
    if ignore_idx is not None and len(ignore_idx):
        keep[np.asarray(ignore_idx, int)] = False
    e1, e2 = plane_basis(n)
    h = P @ n + d
    uv = np.stack([P @ e1, P @ e2], 1)
    best, seen = -np.inf, 0
    for pad in np.atleast_2d(np.asarray(pad_points, float)):
        p_uv = np.array([pad @ e1, pad @ e2])
        m = keep & (np.linalg.norm(uv - p_uv, axis=1) < radius)
        k = int(m.sum())
        if k < min_pts:
            continue
        seen += k
        best = max(best, float(np.percentile(h[m], 95)))
    if seen == 0 or not np.isfinite(best):
        return None, 0
    return best, seen


def descent_floor(P, n, d, pad_points, clearance_m=0.004, ignore_idx=None):
    """Lowest gap-to-plane the pad midpoint may be commanded to.

    Combines the two guards: never below the fitted surface, and never within
    `clearance_m` of whatever the pad probe actually sees underneath.
    """
    sup, seen = support_under_pads(P, n, d, pad_points, ignore_idx=ignore_idx)
    if sup is None:
        return None, 0
    return max(0.0, sup) + clearance_m, seen


# --------------------------------------------------------------- world / ids
def to_world(T_world_cam, p_cam):
    p = np.asarray(p_cam, float)
    return (T_world_cam @ np.r_[p, 1.0])[:3]


def label(obj, used):
    """A short unique name: 'green cube', then 'green cube 2' if repeated."""
    base = ("%s %s" % (obj["colour"], obj["shape"])).strip()
    if base not in used:
        used[base] = 1
        return base
    used[base] += 1
    return "%s %d" % (base, used[base])


def match(objects, phrase):
    """Best object for a phrase like 'the green cube'. None if ambiguous.

    Returns (object, reason).  Ambiguity is REPORTED, never resolved by
    picking the first: a wrong object picked confidently is worse than a
    question asked.
    """
    if not objects:
        return None, "nothing has been scanned yet"
    words = [w for w in phrase.lower().replace(",", " ").split()
             if w not in ("the", "a", "an", "that", "this", "please", "up",
                          "object", "item", "thing", "one")]
    if not words:
        return None, "no object named"
    scored = []
    for o in objects:
        name = o.get("name", "").lower()
        toks = set(name.split()) | {o["colour"], o["shape"]}
        hits = sum(1 for w in words if w in toks or any(w in t for t in toks))
        if hits:
            scored.append((hits, o))
    if not scored:
        return None, "nothing on the table matches %r" % phrase
    top = max(s for s, _ in scored)
    best = [o for s, o in scored if s == top]
    if len(best) > 1:
        return None, ("that matches %d objects (%s) -- say which"
                      % (len(best), ", ".join(o["name"] for o in best)))
    return best[0], "matched %r" % best[0]["name"]


# ----------------------------------------------------------------- self-test
def _synth_scene(n_true, d_true, blobs, noise=0.0005, seed=1):
    """A table with blocks on it, in camera coordinates. Ground truth known."""
    rng = np.random.default_rng(seed)
    e1, e2 = plane_basis(n_true)
    origin = -n_true * d_true
    pts = []
    for _ in range(9000):
        u, v = rng.uniform(-0.35, 0.35, 2)
        pts.append(origin + e1 * u + e2 * v)
    for (cu, cv, size) in blobs:
        s = size / 2.0
        for _ in range(1400):
            u, v = rng.uniform(-s, s), rng.uniform(-s, s)
            face = rng.integers(0, 3)
            if face == 0:
                p = origin + e1 * (cu + u) + e2 * (cv + v) + n_true * size
            elif face == 1:
                p = origin + e1 * (cu + s) + e2 * (cv + v) + n_true * rng.uniform(0, size)
            else:
                p = origin + e1 * (cu + u) + e2 * (cv + s) + n_true * rng.uniform(0, size)
            pts.append(p)
    P = np.array(pts)
    return P + rng.normal(0, noise, P.shape)


def self_test():                                              # noqa: C901
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("  %-58s %s %s" % (name, "OK" if cond else "FAIL", detail))

    print("1. clustering agrees with srl_scene, and adds indices")
    # The normal points at the camera, so the surface itself (-n*d) is IN
    # FRONT of it -- z positive.  srl_scene's own synthetic scene puts the
    # table behind the camera, which is harmless for a pure geometry test and
    # fatal for one that has to render a depth IMAGE, and the first version of
    # this test did exactly that: 0 pixels, and understand() returned None.
    n_true = np.array([0.10, -0.25, -0.96]); n_true /= np.linalg.norm(n_true)
    d_true = 0.42
    P = _synth_scene(n_true, d_true, [(0.06, -0.03, 0.040), (-0.07, 0.05, 0.060)])
    n, d, inl = fit_plane(P)
    if d < 0:
        n, d = -n, -d
    mine = cluster_indices(P, n, d)
    theirs = _srl_segment(P, n, d)
    chk("same number of objects as srl_scene", len(mine) == len(theirs),
        "(%d vs %d)" % (len(mine), len(theirs)))
    if len(mine) == len(theirs) == 2:
        my_pts = sorted(len(g) for g in mine)
        th_pts = sorted(o["n_pts"] for o in theirs)
        chk("same point counts", my_pts == th_pts, "(%s)" % (my_pts,))
        heights = sorted(float((P[g] @ n + d).max()) * 1000 for g in mine)
        chk("40 mm block measured", abs(heights[0] - 40) < 3.0,
            "(%.1f mm)" % heights[0])
        chk("60 mm block measured", abs(heights[1] - 60) < 3.0,
            "(%.1f mm)" % heights[1])

    print("\n2. understand() on the same scene, with no colour available")
    # Rendered finer than the real 480x270 depth image ON PURPOSE: the cloud
    # is synthetic and sparse, so at the real resolution several hundred cloud
    # points collapse into a few dozen pixels and a block falls under the
    # minimum point count.  That is a property of the renderer, not of the
    # segmentation, and it must not be allowed to look like one.
    dummyK = [720.0, 0, 480.0, 0, 720.0, 360.0, 0, 0, 1.0]
    sc = understand(_depth_from_cloud(P, dummyK, 960, 720), dummyK)
    chk("scene built", sc is not None)
    if sc:
        chk("plane inliers sane", sc.plane_ok, "(%.0f%%, %.2f mm)"
            % (sc.inlier_frac * 100, sc.rms_mm))
        chk("objects found", len(sc.objects) >= 2, "(%d)" % len(sc.objects))
        chk("colour is 'unknown', never invented",
            all(o["colour"] == "unknown" for o in sc.objects))
        want = math.degrees(math.acos(abs(float(n_true[2]))))
        chk("incidence recovered", abs(sc.incidence_deg - want) < 1.5,
            "(%.1f vs %.1f deg)" % (sc.incidence_deg, want))
        chk("standoff recovered", abs(sc.standoff_m - d_true) < 0.004,
            "(%.3f vs %.3f m)" % (sc.standoff_m, d_true))

    print("\n3. colour naming, on constructed HSV")
    chk("green", colour_name(60, 200, 180) == "green")
    chk("blue", colour_name(115, 200, 180) == "blue")
    chk("red at the wrap", colour_name(175, 200, 180) == "red")
    chk("cardboard is brown, not orange", colour_name(15, 110, 170) == "brown")
    chk("white table is white, not a hue", colour_name(15, 12, 220) == "white")
    chk("dark is black whatever the hue", colour_name(60, 240, 20) == "black")

    print("\n4. the pad probe -- THE DESCENT GUARD")
    e1, e2 = plane_basis(n_true)
    origin = -n_true * d_true
    # a pad over bare table 150 mm up
    pad_clear = origin + e1 * 0.22 + n_true * 0.150
    h, seen = support_under_pads(P, n, d, [pad_clear])
    chk("over bare table the support is the table", h is not None and abs(h) < 0.004,
        "(%.1f mm, %d pts)" % (h * 1000 if h is not None else float('nan'), seen))
    # a pad over the 60 mm block
    pad_over = origin + e1 * (-0.07) + e2 * 0.05 + n_true * 0.150
    h2, _ = support_under_pads(P, n, d, [pad_over])
    chk("over the 60 mm block the support RISES to the block",
        h2 is not None and abs(h2 - 0.060) < 0.006,
        "(%.1f mm)" % (h2 * 1000 if h2 is not None else float('nan')))
    chk("so the descent floor over the block is higher than over the table",
        descent_floor(P, n, d, [pad_over])[0]
        > descent_floor(P, n, d, [pad_clear])[0] + 0.050)
    # nothing visible -> refusal, NOT a clear floor
    pad_void = origin + e1 * 5.0 + n_true * 0.150
    h3, _ = support_under_pads(P, n, d, [pad_void])
    chk("an UNSEEN column refuses rather than reading clear", h3 is None)
    chk("descent_floor passes the refusal on",
        descent_floor(P, n, d, [pad_void])[0] is None)
    # the grasped object must not block its own grasp
    tgt = [g for g in mine
           if abs(float((P[g] @ n + d).max()) - 0.060) < 0.006][0]
    h4, _ = support_under_pads(P, n, d, [pad_over], ignore_idx=tgt)
    chk("ignoring the target, the floor drops back to the table",
        h4 is not None and abs(h4) < 0.006,
        "(%.1f mm)" % (h4 * 1000 if h4 is not None else float('nan')))

    print("\n5. the guard survives a wrong camera mount (it must: no extrinsic)")
    th = math.radians(8.45)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    n2, d2, _ = fit_plane(P @ R.T)
    if d2 < 0:
        n2, d2 = -n2, -d2
    a = support_under_pads(P, n, d, [pad_over])[0]
    b = support_under_pads(P @ R.T, n2, d2, [R @ pad_over])[0]
    chk("support height unchanged by an 8.45 deg mount error",
        abs(a - b) < 0.002, "(%.1f vs %.1f mm)" % (a * 1000, b * 1000))

    print("\n6. matching a phrase to an object")
    objs = [{"name": "green cube", "colour": "green", "shape": "cube"},
            {"name": "brown box", "colour": "brown", "shape": "box"},
            {"name": "green cube 2", "colour": "green", "shape": "cube"}]
    chk("unambiguous match", match(objs[:2], "the green cube")[0] is objs[0])
    chk("ambiguity REFUSES rather than guessing",
        match(objs, "the green cube")[0] is None,
        "(%s)" % match(objs, "green cube")[1][:44])
    chk("no match is reported as no match",
        match(objs[:2], "the purple banana")[0] is None)

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def _depth_from_cloud(P, K, w, h):
    """Render a cloud back into a depth image, so understand() can be tested
    on the same constructed geometry the clustering was."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    img = np.zeros((h, w), np.float32)
    z = P[:, 2]
    good = z > 0.05
    u = (P[good, 0] * fx / z[good] + cx).astype(int)
    v = (P[good, 1] * fy / z[good] + cy).astype(int)
    zz = z[good]
    m = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    order = np.argsort(-zz[m])          # nearest wins
    img[v[m][order], u[m][order]] = zz[m][order]
    return img


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else 0)
