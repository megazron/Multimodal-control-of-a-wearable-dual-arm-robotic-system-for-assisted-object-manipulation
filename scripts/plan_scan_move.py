#!/usr/bin/env python3
"""Plan a SAFE PATH to the scan pose -- every point on it, not just the ends.

    python3 scripts/plan_scan_move.py --arm left --from home
    python3 scripts/plan_scan_move.py --both --save

WHY THIS EXISTS: A POSE IS NOT A MOVE
-------------------------------------
The scan pose was solved, scored, stored and pressed -- and the arm hit the
mannequin. Two separate mistakes, and only the second one is about planning:

  1. THE CLEARANCE WAS MEASURED IN THE WRONG FRAME. `ClearanceModel` takes
     points in each body part's OWN LINK FRAME and it was handed raw world
     coordinates, so it reported 0.597 m where the truth was 0.217 m. Fixed
     in `srl_body_geometry.wearer_clearance`; this file uses that one.

  2. ONLY THE ENDPOINTS WERE EVER CHECKED. The controller is handed a single
     joint target and interpolates in JOINT SPACE. The hand does not travel
     in a straight line in the world while it does that -- it sweeps -- and
     the swept volume is where the person is. Measured on the corrected
     numbers: pose clearance 0.2168 m at both ends of pick -> scan, and
     0.1508 m in the MIDDLE, worst pair finger tip against the mannequin's
     lower arm. The endpoints were fine and the move was not.

So this plans a MOVE: a list of joint waypoints whose every densified sample
holds the margin. Each leg is checked at PATH_STEPS samples, and a leg that
cannot hold the margin is not returned -- the planner says so rather than
handing back something that looks like a plan.

WHY WAYPOINTS AND NOT A REAL PLANNER. MoveIt is available and is the right
long-term answer, but its collision model is blind to exactly this: docs/ENGINEERING_LOG.md
records that the SRDF excludes the 44 proximal pairs a shoulder mount actually
threatens, so `avoid_collisions` can call a pose valid with the tube inside
the person. A search over lift-first waypoints, scored by the geometry that
was just fixed, is smaller and is checkable.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_body_geometry import wearer_clearance  # noqa: E402
from srl_fk import FK  # noqa: E402
import srl_named_poses as NP  # noqa: E402

WS = "/home/gausms/kortex_ws"
sys.path.insert(0, "%s/src/srl_teleop" % WS)
from srl_teleop.clearance import ClearanceModel  # noqa: E402

FLOOR_M = 0.150
MARGIN_M = 0.250
PATH_STEPS = 30


def wrap(d):
    """Joints 3, 5 and 7 are CONTINUOUS: -179 and +181 deg are one degree
    apart, not 360. Every difference here goes through this."""
    return (np.asarray(d, float) + np.pi) % (2 * np.pi) - np.pi


class Planner:
    def __init__(self, arm):
        self.arm = arm
        self.fk = FK()
        self.model = ClearanceModel()
        self.lo, self.hi, self.cont = self.fk.limits(arm)

    def clear(self, q):
        d, link, part = wearer_clearance(self.fk, self.arm, q, 0.0, self.model)
        return d, "%s vs %s" % ((link or "?").replace(self.arm + "_", ""), part)

    def leg(self, q0, q1, steps=PATH_STEPS):
        """(worst clearance, where, n_breaching) over a joint-space leg."""
        d = wrap(np.asarray(q1) - np.asarray(q0))
        worst, where, bad = float("inf"), None, 0
        for k in range(steps + 1):
            c, w = self.clear(np.asarray(q0) + d * (k / steps))
            if c < worst:
                worst, where = c, w
            if c < MARGIN_M:
                bad += 1
        return worst, where, bad

    def candidates(self, q0, q1):
        """Ways of getting from q0 to q1, simplest first.

        A direct joint interpolation, then LIFT-FIRST detours: move the
        shoulder/elbow joints to a raised value, travel there, then come down
        onto the target. Lifting first is what a person does to reach across
        somebody, and it is the motion that takes the hand OUT of the plane
        the torso occupies.
        """
        yield ("direct", [q1])
        for j2 in (-70.0, -50.0, -30.0, 30.0, 50.0, 70.0):
            mid = np.asarray(q0, float).copy()
            mid[1] = math.radians(j2)
            yield ("lift j2=%+.0f" % j2, [mid, q1])
        for j2 in (-60.0, -40.0, 40.0, 60.0):
            for j4 in (-120.0, -90.0, -60.0):
                mid = np.asarray(q0, float).copy()
                mid[1] = math.radians(j2)
                mid[3] = math.radians(j4)
                yield ("lift j2=%+.0f j4=%+.0f" % (j2, j4), [mid, q1])

    def plan(self, q0, q1, verbose=True):
        """The first candidate whose EVERY leg holds the margin."""
        tried = []
        for name, mids in self.candidates(q0, q1):
            pts = [np.asarray(q0, float)] + [np.asarray(m, float) for m in mids]
            worst, where, bad = float("inf"), None, 0
            for a, b in zip(pts[:-1], pts[1:]):
                w, wh, n = self.leg(a, b)
                bad += n
                if w < worst:
                    worst, where = w, wh
            travel = sum(math.degrees(np.abs(wrap(b - a)).max())
                         for a, b in zip(pts[:-1], pts[1:]))
            tried.append((name, worst, bad, travel, where))
            if bad == 0:
                if verbose:
                    print("    %-22s worst %.4f m  travel %5.0f deg   ACCEPTED"
                          % (name, worst, travel))
                return dict(via=name, waypoints=[list(map(float, p))
                                                 for p in pts[1:]],
                            worst_clearance_m=round(worst, 4),
                            worst_pair=where, travel_deg=round(travel, 1))
            if verbose:
                print("    %-22s worst %.4f m  %2d/%d samples under %.2f  (%s)"
                      % (name, worst, bad, PATH_STEPS + 1, MARGIN_M, where))
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--both", action="store_true")
    ap.add_argument("--start", default="home", choices=["home", "pick"])
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()

    arms = ["left", "right"] if a.both else [a.arm]
    print("PLAN A SAFE MOVE TO THE SCAN POSE")
    print("floor %.3f m (hard) | margin %.3f m (required at EVERY sample)"
          % (FLOOR_M, MARGIN_M))
    print("clearance is surface-vs-body, each in the body part's own frame")
    print("-" * 74)
    out, ok_all = {}, True
    for arm in arms:
        pl = Planner(arm)
        q0 = np.array(NP.load(a.start, arm))
        q1 = np.array(NP.load("scan", arm))
        c0, w0 = pl.clear(q0)
        c1, w1 = pl.clear(q1)
        print("\n  %s: %s %.4f m (%s)  ->  scan %.4f m (%s)"
              % (arm, a.start, c0, w0, c1, w1))
        if c1 < MARGIN_M:
            print("    THE SCAN POSE ITSELF holds only %.4f m, under the "
                  "%.3f m margin." % (c1, MARGIN_M))
            print("    Re-solve it: python3 scripts/solve_scan_pose.py "
                  "--both --save")
            ok_all = False
            continue
        r = pl.plan(q0, q1)
        if r is None:
            print("    NO SAFE PATH FOUND from %s to the scan pose for this "
                  "arm." % a.start)
            print("    Refusing to return a plan. Do NOT command the scan "
                  "pose directly -- the direct joint interpolation is what "
                  "hit the mannequin.")
            ok_all = False
            continue
        r.update(arm=arm, start=a.start)
        out[arm] = r
    if a.save and out:
        p = "%s/recordings/baselines/scan_move.json" % WS
        os.makedirs(os.path.dirname(p), exist_ok=True)
        json.dump(out, open(p, "w"), indent=1)
        print("\nwritten to %s" % p)
    return 0 if (out and ok_all) else 1


if __name__ == "__main__":
    sys.exit(main())
