#!/usr/bin/env python3
"""Measure the three things the fingerprint has to be judged on.

  1. ACCURACY of the match/diff decision
  2. SMALLEST DISPLACEMENT it detects
  3. TIME for a full re-registration versus a skip

WHAT CAN AND CANNOT BE MEASURED HERE, STATED UP FRONT. A fingerprint is a
detector feeding a matcher. This machine has no wrist cameras and the
synthetic renderer is known to be out of the detector's training distribution
-- it scores 0.89-0.91 on a real photograph and 0 on the rendered task
objects, so any "accuracy" taken through it characterises the renderer.

So the split is:
  * the MATCHER is measured properly, against constructed ground truth, with
    pose noise injected at the level the detector was characterised at.
  * the DETECTOR's contribution is NOT measured and is reported as unverified.

The end-to-end figure is therefore conditional: "given a detector with pose
noise sigma, the matcher decides correctly at rate X". Quoting a single
unconditional accuracy would be the renderer bug in a new costume.
"""
import json
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_perception"))
from srl_perception import scene_fingerprint as sf     # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/scene_fingerprint.json")
O = sf.Observation

# Pose noise from the AprilTag characterisation at 0.35 m working distance:
# 0.74 mm mean, 1.60 mm p95 position error. sigma ~= 0.8 mm.
SIGMA_TAG_M = 0.0008
# Confidence-gated colour/shape fallback is far worse; carried as a contrast.
SIGMA_FALLBACK_M = 0.010

BASE = [O("block_a", (0.30, 0.35, 1.00)),
        O("block_b", (-0.30, 0.35, 1.00)),
        O("cup", (0.10, 0.40, 1.02)),
        O("tray", (0.00, 0.38, 0.99)),
        O("tool", (-0.15, 0.42, 1.03))]


def jitter(objs, sd, rng):
    return [O(o.label, tuple(v + rng.gauss(0, sd) for v in o.xyz),
              o.quat, o.confidence) for o in objs]


def main():
    rng = random.Random(20260809)
    out = {}
    P = print
    P("=" * 76)
    P("SCENE FINGERPRINT -- measured")
    P("=" * 76)
    P("\nSCOPE. The MATCHER is measured against constructed ground truth.")
    P("The DETECTOR is NOT: no wrist cameras on this machine, and the")
    P("synthetic renderer is out of the detector's distribution. Every figure")
    P("below is therefore conditional on the pose noise stated with it.")

    # ------------------------------------------------- 1. decision accuracy
    P("\n[1] DECISION ACCURACY, 2000 trials per condition")
    P("    A trial: perturb the scene by a known amount, ask the matcher, and")
    P("    check the verdict against what was actually done.")
    acc = {}
    for name, sd in (("AprilTag, sigma 0.8 mm", SIGMA_TAG_M),
                     ("colour/shape, sigma 10 mm", SIGMA_FALLBACK_M)):
        right = {"same": [0, 0], "moved": [0, 0],
                 "appeared": [0, 0], "vanished": [0, 0]}
        for _ in range(2000):
            kind = rng.choice(["same", "moved", "appeared", "vanished"])
            obs = jitter(BASE, sd, rng)
            if kind == "moved":
                i = rng.randrange(len(obs))
                d = rng.uniform(0.05, 0.20)
                th = rng.uniform(0, 2 * math.pi)
                obs[i] = O(obs[i].label,
                           (obs[i].xyz[0] + d * math.cos(th),
                            obs[i].xyz[1] + d * math.sin(th), obs[i].xyz[2]))
            elif kind == "appeared":
                obs.append(O("new_%d" % rng.randrange(99),
                             (rng.uniform(-0.4, 0.4), 0.36,
                              rng.uniform(0.98, 1.05))))
            elif kind == "vanished":
                obs.pop(rng.randrange(len(obs)))
            v, s = sf.compare(BASE, obs, pos_tol=0.015, gate=0.25)
            got = s["counts"]
            if kind == "same":
                ok = s["unchanged"]
            elif kind == "moved":
                ok = (got.get(sf.MOVED, 0) + got.get(sf.MOVED_OR_SWAPPED, 0)
                      ) == 1 and not got.get(sf.VANISHED)
            elif kind == "appeared":
                ok = got.get(sf.APPEARED, 0) == 1 and not got.get(sf.VANISHED)
            else:
                ok = got.get(sf.VANISHED, 0) == 1 and not got.get(sf.APPEARED)
            right[kind][0] += 1 if ok else 0
            right[kind][1] += 1
        P("    %s" % name)
        for k, (a, b) in right.items():
            P("      %-9s %4d/%4d  %5.1f%%" % (k, a, b, 100.0 * a / b))
        acc[name] = {k: dict(correct=a, n=b) for k, (a, b) in right.items()}
    out["decision_accuracy"] = acc

    # ------------------------------------------ 2. detection threshold
    P("\n[2] SMALLEST DETECTED DISPLACEMENT")
    P("    method: displace one object by d, 400 trials, record the fraction")
    P("    called MOVED. The threshold quoted is the 95%-detection point.")
    thr = {}
    for name, sd in (("AprilTag, sigma 0.8 mm", SIGMA_TAG_M),
                     ("colour/shape, sigma 10 mm", SIGMA_FALLBACK_M)):
        P("    %s" % name)
        found = None
        rows = []
        for mm in (2, 5, 10, 15, 20, 25, 30, 40, 60):
            hit = 0
            for _ in range(400):
                obs = jitter(BASE, sd, rng)
                obs[0] = O(obs[0].label, (obs[0].xyz[0] + mm / 1000.0,
                                          obs[0].xyz[1], obs[0].xyz[2]))
                v, s = sf.compare(BASE, obs, pos_tol=0.015, gate=0.25)
                if any(x["label"] == "block_a" and x["verdict"] in
                       (sf.MOVED, sf.MOVED_OR_SWAPPED) for x in v):
                    hit += 1
            rows.append((mm, hit / 400.0))
            P("      %3d mm -> %5.1f%% detected" % (mm, 100.0 * hit / 400))
            if found is None and hit / 400.0 >= 0.95:
                found = mm
        thr[name] = dict(rows=rows, threshold_mm=found,
                         predicted_mm=1000 * sf.min_detectable_displacement(
                             0.015, sd))
        P("      95%% detection at %s mm  (model predicts %.1f mm)"
          % (found, 1000 * sf.min_detectable_displacement(0.015, sd)))
    out["displacement_threshold"] = thr

    # ---------------------------------------------------- 3. timing
    P("\n[3] TIME")
    P("    The matcher's own cost, and what a sweep adds.")
    n = 2000
    t0 = time.monotonic()
    for _ in range(n):
        sf.compare(BASE, jitter(BASE, SIGMA_TAG_M, rng))
    dt = (time.monotonic() - t0) / n
    P("    compare(), 5 objects           : %.3f ms" % (1000 * dt))
    big = [O("o%d" % i, (rng.uniform(-.5, .5), 0.36, 1.0)) for i in range(50)]
    t0 = time.monotonic()
    for _ in range(200):
        sf.compare(big, jitter(big, SIGMA_TAG_M, rng))
    dtb = (time.monotonic() - t0) / 200
    P("    compare(), 50 objects          : %.3f ms" % (1000 * dtb))
    P("    -> the decision is free; the cost of a sweep is ARM MOTION.")
    P("       Settle time per look pose dominates and is a parameter,")
    P("       not a measurement of this code.")
    out["timing"] = dict(compare_5_ms=1000 * dt, compare_50_ms=1000 * dtb,
                         note="sweep duration is arm motion, measured live")

    P("\nUNVERIFIED, AND NOT REPORTED AS A NUMBER:")
    P("  * detection rate and pose accuracy on REAL wrist cameras.")
    P("  * therefore the end-to-end fingerprint accuracy. The figures above")
    P("    are the matcher's, conditional on the stated pose noise.")
    out["unverified"] = ["detector accuracy on real cameras",
                         "end-to-end fingerprint accuracy"]

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2, default=str)
    P("\n  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
