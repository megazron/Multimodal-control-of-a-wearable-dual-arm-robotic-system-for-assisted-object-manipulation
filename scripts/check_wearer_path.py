#!/usr/bin/env python3
"""Clearance of a planned arm path from the WEARER, not just the table.

WHY
---
A path check that only knows about the table is not a safety check.  The pick
plan's transit swung joint 1 through 94 deg, the whole path cleared the table
by more than 100 mm, and the arm drove into the mannequin's chest at 40 N.
The table was never the thing to be afraid of.

HARD CONSTRAINT 11: the wearer is in the collision model and the 150 mm floor
is the last thing between the arms and a person's chest.  `avoid_collisions`
does NOT cover this -- the SRDF excludes the proximal pairs a shoulder mount
actually threatens -- so clearance is measured geometrically here, through the
same `clearance.ClearanceModel` the follower and the homing node enforce.

Every link that moves is swept, not just the hand: it was the upper arm and
elbow that reached the torso, and a hand-only check would have passed.
"""
import argparse
import json
import math
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_teleop")
from srl_fk import FK  # noqa: E402
from srl_teleop.clearance import ClearanceModel, WEARER_CHECK_LINKS  # noqa: E402

FLOOR_M = 0.150          # HARD CONSTRAINT 11: do not lower this
STEPS = 120


class WearerCheck:
    def __init__(self, arm="left"):
        self.fk = FK()
        self.model = ClearanceModel()
        self.arm = arm
        self.parts = list(self.model.PARTS.keys())

    def clearance(self, q):
        """(metres, which arm link, which body part) at joint vector q."""
        links = ["%s_%s" % (self.arm, l) for l in WEARER_CHECK_LINKS]
        Ts = self.fk.poses(self.arm, q, links)
        part_T = self.fk.poses(self.arm, q, self.parts)
        best, bl, bp = float("inf"), None, None
        for lname, T in zip(WEARER_CHECK_LINKS, Ts):
            pw = T[:3, 3]
            for pname, PT in zip(self.parts, part_T):
                # world point -> this body part's own frame
                R = PT[:3, :3]
                p_local = R.T @ (pw - PT[:3, 3])
                d, _ = self.model.clearance({pname: [p_local]})
                if d < best:
                    best, bl, bp = d, lname, pname
        return best, bl, bp

    def sweep(self, q0, q1, steps=STEPS):
        worst, wa, wl, wp = float("inf"), 0.0, None, None
        for k in range(steps + 1):
            a = k / steps
            q = np.array(q0) + (np.array(q1) - np.array(q0)) * a
            d, l, p = self.clearance(q)
            if d < worst:
                worst, wa, wl, wp = d, a, l, p
        return worst, wa, wl, wp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--start", help="json with left.rad for the start pose")
    args = ap.parse_args()

    wc = WearerCheck("left")
    plan = json.load(open(args.plan))
    q = None
    if args.start:
        q = np.array(json.load(open(args.start))["left"]["rad"], float)
        d, l, p = wc.clearance(q)
        print("start pose: clearance %.4f m  (%s vs %s)%s"
              % (d, l, p, "" if d >= FLOOR_M else "   <-- ALREADY INSIDE THE FLOOR"))

    print("\n%-10s %10s %10s  %-24s %s"
          % ("leg", "at pose", "worst path", "closest pair", "verdict"))
    allok = True
    for s in plan["plan"]:
        qt = np.array(s["q"], float)
        d_at, l_at, p_at = wc.clearance(qt)
        if q is None:
            worst, wa, wl, wp = d_at, 1.0, l_at, p_at
        else:
            worst, wa, wl, wp = wc.sweep(q, qt)
        ok = worst >= FLOOR_M
        allok = allok and ok
        print("%-10s %9.1f mm %9.1f mm  %-24s %s"
              % (s["name"], d_at * 1000, worst * 1000,
                 "%s/%s" % (wl, wp),
                 "OK" if ok else "BREACH (floor %.0f mm) at %d%% along"
                 % (FLOOR_M * 1000, int(wa * 100))))
        q = qt
    print("\nWEARER-SAFE:", allok)
    sys.exit(0 if allok else 1)


if __name__ == "__main__":
    main()
