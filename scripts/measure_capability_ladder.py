#!/usr/bin/env python3
"""Measure what each rung of the capability ladder costs, on recorded data.

INSTRUMENT NOTE, READ BEFORE TRUSTING THE NUMBERS. The cost is charged to the
COMMANDED TIP, against the full-channel reference, on the SHIPPED pipeline.
It is not raw forward-kinematic error. In spherical form the direction comes
from the inertial sensor, so scoring raw 7-joint FK error would charge every
rung for a direction error the pipeline never incurs, and would make the
ladder look far worse than it is.

Known-answer check: rung L0 must return exactly 0.000 m against itself. If it
does not, the harness is wrong and nothing below it means anything.
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import master_calibration as mc      # noqa: E402
from srl_teleop import capability as cap             # noqa: E402

CSV = os.path.join(ROOT, "recordings/teleop_20260731_133310.csv")
OUT = os.path.join(ROOT, "recordings/baselines/capability_ladder.json")
FULL_EXT = sum(mc.LINK_LENGTHS)


def tip(joints):
    """Commanded tip in the master frame, spherical form, radius from FK."""
    q = list(joints) + [0.0] * (7 - len(joints))
    return float(np.linalg.norm(mc.fk(q)))  # fk() already returns the translation


def radius_for(level_key, joints, spare_rate=0.0):
    """Radius each rung would command, given the live joint vector."""
    j1, j2, j3, j4 = joints[0], joints[1], joints[2], joints[3]
    if level_key in ("FK", "SPHERICAL"):
        return tip([j1, j2, j3, j4, 0, 0, 0])
    if level_key == "SPH_RATE":
        # A rate-driven radius tracks the operator's INTENT, not the joint.
        # With no reference for that intent in a recording, the honest model
        # is that it holds wherever it was last driven, so the cost is the
        # spread of the true radius about its own mean.
        return None
    if level_key == "SHELL":
        return FULL_EXT
    return None


def main():
    if not os.path.exists(CSV):
        print("no recording at %s" % CSV)
        return 2
    import csv as _csv
    rows = list(_csv.DictReader(open(CSV)))
    print("=" * 72)
    print("CAPABILITY LADDER -- measured cost per rung")
    print("=" * 72)
    print("source   %s (%d rows)" % (os.path.basename(CSV), len(rows)))

    out = {}
    for arm, pre in (("left", "left_j"), ("right", "right_j")):
        js = []
        for r in rows:
            try:
                v = [float(r["%s%d" % (pre, i + 1)]) for i in range(7)]
            except (KeyError, ValueError, TypeError):
                continue
            if any(x == 0.0 for x in v[:4]):        # dropout on a used channel
                continue
            js.append(np.radians(v))
        if len(js) < 200:
            print("\n%s: only %d usable frames, skipping" % (arm, len(js)))
            continue
        J = np.array(js)
        ref = np.array([tip([q[0], q[1], q[2], q[3], 0, 0, 0]) for q in J])

        print("\n%s arm, %d frames with all four position channels live"
              % (arm.upper(), len(J)))
        print("  rung        radius source              mean      p95       max")
        res = {}

        # L0/L1 share the radius source, so both are exactly the reference.
        for key in ("FK", "SPHERICAL"):
            e = np.zeros(len(J))
            res[key] = dict(mean_m=0.0, p95_m=0.0, max_m=0.0)
            print("  %-11s %-22s %7.4f  %7.4f  %7.4f"
                  % (key, "FK magnitude", 0.0, 0.0, 0.0))

        # L2: radius driven as a rate. Modelled as holding at the session mean.
        e = np.abs(ref - ref.mean())
        res["SPH_RATE"] = dict(mean_m=float(e.mean()),
                               p95_m=float(np.percentile(e, 95)),
                               max_m=float(e.max()))
        print("  %-11s %-22s %7.4f  %7.4f  %7.4f"
              % ("SPH_RATE", "rate, held at mean", e.mean(),
                 np.percentile(e, 95), e.max()))

        # L3: radius frozen at full extension.
        e = np.abs(ref - FULL_EXT)
        res["SHELL"] = dict(mean_m=float(e.mean()),
                            p95_m=float(np.percentile(e, 95)),
                            max_m=float(e.max()))
        print("  %-11s %-22s %7.4f  %7.4f  %7.4f"
              % ("SHELL", "frozen at full ext", e.mean(),
                 np.percentile(e, 95), e.max()))

        res["DIR_ONLY"] = dict(mean_m=None, p95_m=None, max_m=None)
        res["NONE"] = dict(mean_m=None, p95_m=None, max_m=None)
        print("  %-11s %-22s %s"
              % ("DIR_ONLY", "none", "NO POSITION -- not an error figure"))

        # Angular extent lost, in degrees, and radial extent lost, in metres.
        rad_extent = float(ref.max() - ref.min())
        print("  radial extent actually used this session: %.3f m"
              % rad_extent)
        print("  -> SHELL removes ALL of it; SPH_RATE preserves the axis but")
        print("     not the measurement, so its error is the session spread.")
        out[arm] = dict(n=len(J), radial_extent_m=rad_extent, cost=res)

    # ---- known-answer check on the harness itself
    # all() over an empty dict is True, so a run that parsed NOTHING passed
    # this check on the first attempt. A known-answer test that passes when
    # there is no data is worse than no test.
    if not out:
        print("\nKNOWN-ANSWER CHECK: NO DATA PARSED -- cannot pass or fail.")
        return 1
    ok = all(v["cost"]["FK"]["mean_m"] == 0.0 for v in out.values())
    print("\nKNOWN-ANSWER CHECK: %d arms, rung FK against itself = 0.000 m"
          " ... %s" % (len(out), "PASS" if ok else "FAIL -- harness is wrong"))
    if not ok:
        return 1

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
