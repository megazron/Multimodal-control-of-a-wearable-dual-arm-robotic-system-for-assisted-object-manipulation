#!/usr/bin/env python3
"""How accurately can an object's pose be recovered from RGB-D, without tags?

WHY THIS IS MEASURABLE HERE AND THE DETECTION RATE IS NOT. The project's own
rule: synthetic is legitimate where the ground truth is CONSTRUCTED, not
RENDERED. A rendered image is only as good as the renderer, which is exactly
what made the 0-4% detection figure meaningless. But a point cloud of a box of
known size, sampled from a known viewpoint with a known noise model, is
arithmetic and geometry -- the ground truth is constructed, and the answer does
not depend on any appearance model.

WHAT IS AND IS NOT ANSWERED HERE:

  ANSWERED  -- given that the detector puts a box roughly around the object,
               how accurately does a fit of the KNOWN dimensions to the depth
               points recover the object's centre? And how does that degrade
               with range and with a sloppy detection box?
  NOT       -- whether the detector finds the object at all on real images.
               That is the detection RATE, it needs real frames, and it is
               reported separately as unmeasured.

THE PIPELINE THIS MODELS is option (c): the detection box only has to point at
the right REGION; the pose comes from fitting known dimensions to the cropped
cloud. The detector's own localisation therefore enters only as contamination
-- background pixels pulled into the crop -- not as pose error directly.

THE ERROR TERMS, all included:
  * stereo depth noise, sigma_z = z^2 * sigma_disp / (f * B), the standard
    model. It shrinks as z^2, which is the entire reason a close-range look
    is worth taking.
  * a depth BIAS (systematic scale/offset error) that does NOT average away,
    unlike the random term. Left in because on a real sensor it dominates
    once enough pixels are averaged.
  * PARTIAL VISIBILITY. Only faces turned toward the camera are sampled, so
    the centroid of the visible surface is NOT the centroid of the object.
    This is the term people forget and it is the largest one for a naive
    centroid.
  * CONTAMINATION from a sloppy detection box: a fraction of points drawn
    from the background plane instead of the object.

    python3 scripts/measure_depth_pose_accuracy.py
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/baselines/depth_pose_accuracy.json")

# --- sensor model: Kinova Gen3 vision module (RealSense D4xx class) --------
F_PX = 512.0          # focal length, px, for the depth stream
BASELINE = 0.050      # m, stereo baseline
SIGMA_DISP = 0.15     # px, disparity noise (typical for D4xx sub-pixel)
DEPTH_BIAS_FRAC = 0.004   # 0.4% systematic range error; does NOT average out

RNG = np.random.default_rng(11)


def sigma_z(z):
    """Per-pixel random depth noise at range z."""
    return z * z * SIGMA_DISP / (F_PX * BASELINE)


def sample_box_cloud(size, centre, z, n_px_target, contamination):
    """Points on the VISIBLE faces of an axis-aligned box, plus background.

    The camera looks along +z from the origin. Only the three faces whose
    outward normal has a negative z / matching sign component are visible; for
    a box viewed roughly head-on that is the front face plus, at an angle, up
    to two sides. Sampling is proportional to projected area, which is what a
    depth camera actually delivers.
    """
    sx, sy, sz = size
    cx, cy, cz = centre
    faces = [
        # (normal axis, sign, in-plane extents, offset along normal)
        (2, -1, (sx, sy), sz / 2.0),      # front face, toward camera
        (0, -1, (sz, sy), sx / 2.0),      # -x side
        (1, -1, (sx, sz), sy / 2.0),      # -y side
    ]
    pts = []
    # projected area weights: the front face dominates for a head-on view
    weights = [1.0, 0.22, 0.22]
    tot = sum(weights)
    for (axis, sgn, ext, off), w in zip(faces, weights):
        n = max(4, int(n_px_target * w / tot))
        a = RNG.uniform(-ext[0] / 2.0, ext[0] / 2.0, n)
        b = RNG.uniform(-ext[1] / 2.0, ext[1] / 2.0, n)
        p = np.zeros((n, 3))
        oth = [i for i in range(3) if i != axis]
        p[:, oth[0]] = a
        p[:, oth[1]] = b
        p[:, axis] = sgn * off
        p += np.array([cx, cy, cz])
        pts.append(p)
    P = np.vstack(pts)
    # depth noise: random per point, plus a systematic scale error
    s = sigma_z(z)
    P[:, 2] += RNG.normal(0.0, s, len(P)) + P[:, 2] * DEPTH_BIAS_FRAC
    # lateral error follows from the depth error through the projection
    P[:, 0] += RNG.normal(0.0, s * 0.5, len(P))
    P[:, 1] += RNG.normal(0.0, s * 0.5, len(P))
    # contamination: background plane behind the object, pulled in by a
    # detection box that is too big
    n_bg = int(len(P) * contamination)
    if n_bg > 0:
        bg = np.zeros((n_bg, 3))
        bg[:, 0] = RNG.uniform(cx - sx, cx + sx, n_bg)
        bg[:, 1] = RNG.uniform(cy - sy, cy + sy, n_bg)
        bg[:, 2] = cz + sz / 2.0 + RNG.uniform(0.01, 0.06, n_bg)
        bg[:, 2] += RNG.normal(0.0, s, n_bg)
        P = np.vstack([P, bg])
    return P


def naive_centroid(P, size):
    """The obvious thing, and the wrong thing: mean of the visible points."""
    return P.mean(axis=0)


def fit_known_box(P, size):
    """Fit a box of KNOWN dimensions to the cloud -- option (c).

    SEGMENT FIRST, THEN FIT. The first version of this trimmed percentiles and
    nothing else, and it read 24.5 mm at 10% contamination and 28.5 mm at 50%
    -- SATURATING, which is the signature of an estimator artefact rather than
    a sensor limit. It was: background points inflated the apparent span along
    an axis, the fit concluded both faces were visible, and it centred between
    a real face and a background plane. That is a bug in the estimator, and
    reporting it as the accuracy of RGB-D would have been wrong.

    What a real pipeline does, and what this does now:
      1. the object is the NEAREST surface in the crop, so keep only points
         within the object's own depth extent of the closest return;
      2. re-trim, then place the box against the surfaces that survive.
    """
    # 1. depth segmentation: the object is the nearest thing in the box
    z_near = np.percentile(P[:, 2], 2.0)
    keep = P[:, 2] <= z_near + size[2] + 0.010
    Q = P[keep] if keep.sum() >= 20 else P

    c = np.empty(3)
    for i, s in enumerate(size):
        lo = np.percentile(Q[:, i], 2.0)
        hi = np.percentile(Q[:, i], 98.0)
        # Decide face visibility from the KNOWN size, not from the spread:
        # if the observed span is close to the true extent, both faces are
        # there; otherwise only one is, and the box hangs off it.
        if hi - lo > 0.75 * s:
            c[i] = 0.5 * (lo + hi)
        else:
            c[i] = lo + s / 2.0
    return c


def trial(size, z, contamination, n_trials=300):
    """Return (naive error mm, known-box-fit error mm), RMS over trials."""
    # pixels on target: the front face's projected size at this range
    px_per_m = F_PX / z
    n_px = max(30, int(size[0] * px_per_m * size[1] * px_per_m))
    en, ef = [], []
    for _ in range(n_trials):
        centre = np.array([RNG.uniform(-0.05, 0.05),
                           RNG.uniform(-0.05, 0.05), z])
        P = sample_box_cloud(size, centre, z, n_px, contamination)
        en.append(np.linalg.norm(naive_centroid(P, size) - centre))
        ef.append(np.linalg.norm(fit_known_box(P, size) - centre))
    return (1000 * float(np.sqrt(np.mean(np.square(en)))),
            1000 * float(np.sqrt(np.mean(np.square(ef)))))


def main():
    OBJ = {"40 mm block": (0.040, 0.040, 0.040),
           "45 mm part": (0.045, 0.045, 0.050),
           "50 mm multimeter": (0.050, 0.090, 0.130)}
    RANGES = [0.10, 0.15, 0.20, 0.25, 0.35, 0.50]
    res = {"sensor": dict(f_px=F_PX, baseline_m=BASELINE,
                          sigma_disp_px=SIGMA_DISP,
                          depth_bias_frac=DEPTH_BIAS_FRAC),
           "per_pixel_sigma_mm": {str(z): round(1000 * sigma_z(z), 3)
                                  for z in RANGES},
           "rows": []}

    print("PER-PIXEL DEPTH NOISE (stereo model, sigma_z = z^2 sd/(f B))")
    for z in RANGES:
        print("   %.2f m -> %.2f mm" % (z, 1000 * sigma_z(z)))

    print("\nPOSE ERROR, RMS over 300 trials, clean crop (0%% contamination)")
    print("  %-18s %7s %12s %14s" % ("object", "range", "naive centroid",
                                     "known-box fit"))
    for nm, sz in OBJ.items():
        for z in RANGES:
            a, b = trial(sz, z, 0.0)
            res["rows"].append(dict(obj=nm, range_m=z, contamination=0.0,
                                    naive_mm=round(a, 2), fit_mm=round(b, 2)))
            print("  %-18s %6.2fm %10.1f mm %12.1f mm%s"
                  % (nm, z, a, b, "   <-- over 10 mm" if b > 10 else ""))

    print("\nWITH A SLOPPY DETECTION BOX (background pulled into the crop)")
    print("  %-18s %7s %8s %14s" % ("object", "range", "contam", "known-box fit"))
    for nm, sz in OBJ.items():
        for cont in (0.0, 0.10, 0.25, 0.50):
            _, b = trial(sz, 0.35, cont)
            res["rows"].append(dict(obj=nm, range_m=0.35, contamination=cont,
                                    fit_mm=round(b, 2)))
            print("  %-18s %6.2fm %7.0f%% %12.1f mm%s"
                  % (nm, 0.35, cont * 100, b,
                     "   <-- over 10 mm" if b > 10 else ""))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
