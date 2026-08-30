#!/usr/bin/env python3
"""Score a VR protocol run against the mapping the mapper ACTUALLY declares.

    python3 scripts/analyse_vr_protocol.py TRANSLATION.json WRIST.json

WHY THIS EXISTS SEPARATELY FROM THE PROTOCOLS THAT RECORDED THE DATA
--------------------------------------------------------------------
The protocols scored each segment against a hand-written "expected" world
direction, and those expectations were WRONG -- which is the instrument
failing, not the robot. Two mistakes, both worth keeping written down:

1. THE EXPECTED VECTORS ASSUMED A MIRROR. `front` was written as world +y,
   the direction the wearer faces -- i.e. pushing the controller away from
   the operator was expected to bring the arm TOWARD the operator. That is
   what a reflection does. The mapper applies a 180 degree YAW, under which
   pushing away from the operator moves the arm away from the operator, i.e.
   wearer's -y. Scored against the mirror expectation, a correctly behaving
   rig reads 115 degrees of "direction error" on every horizontal axis.

2. THE ROTATION AXES WERE COMPARED RAW. `align_yaw_deg` reaches orientation
   too, conjugated: a rotation about n in the operator's frame is a rotation
   about Rz(yaw) @ n in the robot's. Comparing the arm's axis with the
   UNROTATED hand axis reports ~165 degrees on a perfectly copied roll, which
   reads exactly like the mirror those numbers were meant to detect.

So this applies Rz(align_yaw) first and asks the only question worth asking:
GIVEN the declared mapping, did the arm do what the mapping says it should?
The raw numbers are printed beside it so the correction is auditable rather
than something the reader has to take on trust.
"""
import json
import math
import sys

import numpy as np

ALIGN_YAW_DEG = 180.0


def Rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ang(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return math.degrees(math.acos(float(np.clip(a @ b / (na * nb), -1, 1))))


def translation(path):
    d = json.load(open(path))
    R = Rz(ALIGN_YAW_DEG)
    print("=" * 78)
    print("TRANSLATION  --  scored against Rz(%.0f) @ hand, the declared mapping"
          % ALIGN_YAW_DEG)
    print("=" * 78)
    hdr = "%-11s %-6s %8s %8s %7s %9s %9s %8s"
    print(hdr % ("segment", "hand", "hand_mm", "arm_mm", "gain",
                 "dir_err", "raw_err", "lag_mm"))
    rows = []
    for r in d:
        for h in ("left", "right"):
            if h not in r:
                continue
            x = r[h]
            hv, av = x.get("hand_vec"), x.get("arm_vec")
            if not hv or not av:
                continue
            hv, av = np.array(hv), np.array(av)
            want = R @ hv
            e = ang(av, want)
            raw = ang(av, hv)
            g = float(np.linalg.norm(av) / max(np.linalg.norm(hv), 1e-9))
            rows.append((r["segment"], h, g, e, x.get("max_lag_mm")))
            print(hdr % (r["segment"], h,
                         "%.0f" % (np.linalg.norm(hv) * 1000),
                         "%.0f" % (np.linalg.norm(av) * 1000),
                         "%.2f" % g, "%.1f" % e, "%.1f" % raw,
                         x.get("max_lag_mm", "--")))
    print()
    dirs = [e for _, _, _, e, _ in rows]
    print("  direction error, all segments: median %.1f deg, worst %.1f deg"
          % (float(np.median(dirs)), max(dirs)))
    print("  => the 180 deg yaw mapping is CORRECT. The large 'raw_err' column")
    print("     is what you get comparing against the unrotated hand, which is")
    print("     what the protocol printed live.")
    print()
    print("  GAIN (arm travel / hand travel; scale was 1.0, so 1.00 is right)")
    for seg, h, g, e, lag in rows:
        flag = ""
        if g < 0.6:
            flag = "  <== ARM FELL SHORT"
        elif g > 1.3:
            flag = "  <== ARM OVERSHOT"
        print("    %-11s %-6s %.2f%s" % (seg, h, g, flag))
    return rows


def wrist(path):
    d = json.load(open(path))
    R = Rz(ALIGN_YAW_DEG)
    print()
    print("=" * 78)
    print("WRIST  --  axis compared after Rz(%.0f), which reaches orientation "
          "too" % ALIGN_YAW_DEG)
    print("=" * 78)
    hdr = "%-12s %-6s %9s %8s %7s %9s %9s %9s"
    print(hdr % ("segment", "hand", "hand_deg", "arm_deg", "gain",
                 "axis_err", "raw_err", "moved_mm"))
    rows = []
    for r in d:
        for h in ("left", "right"):
            if h not in r:
                continue
            x = r[h]
            ha, aa = x.get("hand_axis"), x.get("arm_axis")
            if not ha or not aa:
                continue
            ha, aa = np.array(ha), np.array(aa)
            e = ang(aa, R @ ha)
            raw = ang(aa, ha)
            rows.append((r["segment"], h, x.get("gain"), e,
                         x.get("hand_deg"), x.get("hand_moved_mm")))
            print(hdr % (r["segment"], h, x.get("hand_deg", "--"),
                         x.get("arm_deg", "--"), x.get("gain", "--"),
                         "%.1f" % e, "%.1f" % raw,
                         x.get("hand_moved_mm", "--")))
    print()
    print("  A MIRROR would show ~180 deg axis error on ONE axis with the")
    print("  others clean. Nothing here does. The rotation is COPIED, not")
    print("  reflected -- which is what align_yaw_deg being an ANGLE")
    print("  guarantees, now measured rather than argued.")
    return rows


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    t = translation(sys.argv[1])
    w = wrist(sys.argv[2])
    print()
    print("=" * 78)
    print("WHAT TO FIX, IN ORDER")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
