#!/usr/bin/env python3
"""HOW MUCH OF THE CLEARANCE IS THE MANNEQUIN? Sweep the body, not one guess.

    python3 scripts/measure_wearer_size_sweep.py
    python3 scripts/measure_wearer_size_sweep.py --pose new=recordings/baselines/home_wide.json
    python3 scripts/measure_wearer_size_sweep.py --self-test

WHY A SWEEP AND NOT A SECOND PROFILE
------------------------------------
`config/wearer_sizes/measured_adult.json` is one adult from one photograph and
says so. Answering "what does a real body cost" by swapping in that one
profile replaces a number nobody measured with a number one person measured,
and the reader cannot tell which of the six dimensions did the damage.

So this sweeps each dimension of the body ON ITS OWN, from the mannequin's
value outward, and reports where the 150 mm floor is crossed. The output is a
SENSITIVITY, which survives being wrong about any particular wearer: it says
"every 10 mm of chest depth costs the mount N mm", and that statement is true
whatever the wearer turns out to measure.

The dimension that matters most is the one nothing has ever measured. The
scene camera lifts a skeleton, so it gives JOINT CENTRES: lengths and spans.
It cannot see how DEEP a chest is, and `chest_d` has therefore been 0.220 m
since the model was written -- through every clearance figure in the project,
and through `measured_adult`, which does not override it either.

CONTROLS:

    the mannequin reproduces      sweeping to the shipped value must return
                                  the shipped clearance, exactly
    bigger is never better        every dimension increased must leave
                                  clearance the same or WORSE. A body that
                                  grows and clears more is a broken model
    it can cross the floor        the sweep must contain a breach, or the
                                  range swept was too timid to be informative
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from measure_home_clearance import Rig, load_pose, ARMS               # noqa: E402
from srl_teleop import wearer_posture as WP                           # noqa: E402

FLOOR = 0.15

# Each dimension, and the range worth asking about. The mannequin's value is
# always the first entry so the control has something to reproduce.
#
# WHERE THE RANGES COME FROM. These are published adult anthropometry, quoted
# to the nearest 10 mm and used as a RANGE rather than as a value: roughly
# 5th-percentile female at the low end to 95th-percentile male at the high.
# They are NOT a measurement of the wearer and this file never claims they
# are -- the point of a sweep is that it stays useful when every one of them
# is wrong. Measure the actual wearer before a session; `--profile` will then
# score them directly.
SWEEP = {
    "chest_d": (0.220, [0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32],
                "chest depth -- NEVER MEASURED by anything in this project"),
    "chest_w": (0.360, [0.34, 0.36, 0.40, 0.43, 0.46, 0.50, 0.54],
                "chest width across the torso box"),
    "shoulder_x": (0.210, [0.19, 0.21, 0.23, 0.25, 0.27],
                   "half the shoulder-joint span"),
    "upper_arm_len": (0.300, [0.24, 0.27, 0.30, 0.33, 0.36, 0.39],
                      "shoulder to elbow"),
    "upper_arm_rad": (0.050, [0.05, 0.06, 0.07, 0.08],
                      "upper-arm radius -- a camera cannot measure this"),
    "lower_arm_len": (0.260, [0.22, 0.24, 0.26, 0.28, 0.30],
                      "elbow to wrist"),
    "hips_w": (0.320, [0.30, 0.32, 0.36, 0.40, 0.44],
               "hip breadth standing"),
}


def score(pose, size, posture="down"):
    """(mount cap, whole chain, moving chain), worst over both arms."""
    rig = Rig(size=size, posture=posture)
    cap = min(rig.mount_cap(a) for a in ARMS)
    rows = {a: rig.report(a, pose[a]) for a in ARMS}
    whole = min(rows[a]["whole_chain_m"] for a in ARMS)
    moving = min(rows[a]["moving_chain_m"] for a in ARMS)
    binds = rows[min(ARMS, key=lambda a: rows[a]["whole_chain_m"])]
    return cap, whole, moving, binds["whole_chain_to"]


def controls(pose, verbose=True):
    ok = True
    base = dict(WP.SHIPPED_SIZE)
    cap0, w0, m0, _ = score(pose, base)

    # 1. sweeping TO the shipped value reproduces the shipped answer
    probe = dict(base)
    probe["chest_d"] = SWEEP["chest_d"][0]
    cap1, w1, m1, _ = score(pose, probe)
    same = abs(cap1 - cap0) < 1e-12 and abs(w1 - w0) < 1e-12
    ok &= same
    if verbose:
        print("   the mannequin reproduces itself               %.4f/%.4f "
              "-> %s" % (w1, w0, "PASS" if same else "FAIL"))

    # 2. A BIGGER BODY MUST NEVER CLEAR MORE. This is the direction the whole
    #    safety case rests on, so it is checked over every dimension rather
    #    than argued.
    worst_gain, offender = 0.0, None
    for key, (_, vals, _) in SWEEP.items():
        prev = None
        for v in sorted(vals):
            p = dict(base)
            p[key] = v
            _, w, m, _ = score(pose, p)
            if prev is not None and w > prev + 1e-9:
                if w - prev > worst_gain:
                    worst_gain, offender = w - prev, "%s=%s" % (key, v)
            prev = w
    ok &= offender is None
    if verbose:
        print("   a bigger body never clears more               %s -> %s"
              % (offender or "none", "PASS" if offender is None else "FAIL"))

    # 3. THE RANGE MUST REACH A BREACH, or the sweep proves nothing about
    #    where the floor is. It has to be checked on the COMBINED body, and
    #    finding that out was the point: no single dimension at its extreme
    #    crosses 150 mm (the worst, chest_w = 0.54, reads 0.1578). Testing
    #    one dimension at a time reported "the range is too timid" about a
    #    range that is not timid at all -- the geometry simply does not fail
    #    one dimension at a time. It fails on a whole person.
    big = dict(base)
    for key, (_, vals, _) in SWEEP.items():
        big[key] = max(vals)
    _, w_big, _, _ = score(pose, big)
    breach = w_big < FLOOR
    ok &= breach
    if verbose:
        print("   every dimension at its largest breaches       %.4f m -> %s"
              % (w_big, "PASS" if breach else "FAIL -- widen the range"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="config")
    ap.add_argument("--posture", default="down")
    ap.add_argument("--profile", action="append", default=None,
                    help="also score these named size profiles")
    ap.add_argument("--out", default=None)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    pose = load_pose(a.pose)
    print("INSTRUMENT CHECKS")
    if not controls(pose):
        print("REFUSING: a control failed.")
        return 6
    if a.self_test:
        return 0

    base = dict(WP.SHIPPED_SIZE)
    cap0, w0, m0, who0 = score(pose, base, a.posture)
    print("\nPOSE %r AGAINST THE MANNEQUIN" % a.pose)
    print("   mount cap %.4f   whole chain %.4f (%s)   moving chain %.4f"
          % (cap0, w0, who0, m0))
    print("\nONE DIMENSION AT A TIME. Everything else stays the mannequin.")
    print("   %-15s %-8s %-9s %-9s %-9s %-9s %s"
          % ("dimension", "value", "mountcap", "whole", "moving", "d(whole)",
             "floor"))
    out = {}
    for key, (shipped, vals, note) in SWEEP.items():
        print("   %s  (%s)" % (key, note))
        rows = []
        for v in vals:
            p = dict(base)
            p[key] = v
            cap, w, m, who = score(pose, p, a.posture)
            flag = "BREACH" if w < FLOOR else ("cap<floor" if cap < FLOOR
                                               else "ok")
            print("   %-15s %-8.3f %-9.4f %-9.4f %-9.4f %+9.4f %s%s"
                  % ("", v, cap, w, m, w - w0, flag,
                     "   <- mannequin" if abs(v - shipped) < 1e-9 else ""))
            rows.append(dict(value=v, mount_cap_m=round(cap, 4),
                             whole_m=round(w, 4), moving_m=round(m, 4),
                             binds_on=who, breach=bool(w < FLOOR)))
        # the sensitivity, as one number the reader can carry away
        lo, hi = rows[0], rows[-1]
        span = hi["value"] - lo["value"]
        if span > 0:
            print("   %-15s -> %.1f mm of clearance per 10 mm of %s"
                  % ("", 10.0 * (lo["whole_m"] - hi["whole_m"]) / span * 1000
                     / 1000.0, key))
        out[key] = dict(note=note, shipped=shipped, rows=rows)

    # THE COMBINED BODY. One dimension at a time is a sensitivity; a wearer
    # is all of them at once, and the two answers are not close.
    print("\nEVERY DIMENSION AT ONCE -- the thing a person actually is")
    print("   %-22s %-9s %-9s %-9s %s"
          % ("body", "mountcap", "whole", "moving", "binds on"))
    combos = [("mannequin (shipped)", base)]
    small = dict(base)
    big = dict(base)
    for key, (_, vals, _) in SWEEP.items():
        small[key] = min(vals)
        big[key] = max(vals)
    combos += [("every dim smallest", small), ("every dim largest", big)]
    for name, p in combos:
        cap, w, m, who = score(pose, p, a.posture)
        print("   %-22s %-9.4f %-9.4f %-9.4f %s%s"
              % (name, cap, w, m, who,
                 "   BREACHES THE 150 mm FLOOR" if w < FLOOR else ""))
        if cap < FLOOR:
            print("   %-22s the FIXED MOUNT is inside the floor. No pose "
                  "helps; this is a bracket." % "")
        out.setdefault("combined", {})[name] = dict(
            mount_cap_m=round(cap, 4), whole_m=round(w, 4),
            moving_m=round(m, 4), binds_on=who, breach=bool(w < FLOOR))

    if a.profile:
        print("\nNAMED PROFILES, every dimension at once")
        print("   %-18s %-9s %-9s %-9s %s"
              % ("profile", "mountcap", "whole", "moving", "binds on"))
        for name in a.profile:
            prof = WP.size_profile(name)
            cap, w, m, who = score(pose, prof, a.posture)
            print("   %-18s %-9.4f %-9.4f %-9.4f %s%s"
                  % (name, cap, w, m, who, "   BREACH" if w < FLOOR else ""))
            out.setdefault("profiles", {})[name] = dict(
                mount_cap_m=round(cap, 4), whole_m=round(w, 4),
                moving_m=round(m, 4), binds_on=who, breach=bool(w < FLOOR))

    if a.out:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        json.dump(dict(pose=a.pose, posture=a.posture, floor_m=FLOOR,
                       mannequin=dict(mount_cap_m=cap0, whole_m=w0,
                                      moving_m=m0),
                       sweep=out), open(a.out, "w"), indent=2, default=float)
        print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
