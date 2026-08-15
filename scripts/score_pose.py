#!/usr/bin/env python3
"""SCORE ONE POSTURE. Hand, elbow, limb apex, wearer clearance, wrist level.

    python3 scripts/score_pose.py --q -2.117 -1.322 ... --arm left
    python3 scripts/score_pose.py --from-json recordings/baselines/x.json

WHY THIS IS SEPARATE FROM THE SEARCH. `find_presentation_pose.py` scores the
candidates it generates; it has no way to score a pose someone hands it. So
"the new pose is better than the old one" could only ever be asserted, because
the old pose's numbers were taken under a cost that did not yet measure the
quantity the render was actually failing on -- the APEX of the limb against
the wearer's shoulder line. A before-and-after needs both measured the same
way, on the same instrument, and this is that instrument.

CONTROLS:
    FK must answer                     or nothing is reported
    a pose driven into the torso       clearance must be NEGATIVE
    the home pose                      must clear the floor
"""
import argparse
import json
import math
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))

import find_presentation_pose as FP                            # noqa: E402


def report(n, arm, q, label):
    F = n.frame(arm, q, ["shoulder_link", "forearm_link", "end_effector_link"])
    if F is None:
        print("   %-22s FK did not answer" % label)
        return None
    hand = F["end_effector_link"]["xyz"]
    elbow = F["forearm_link"]["xyz"]
    sh = F["shoulder_link"]["xyz"]
    x, y, z, w = F["end_effector_link"]["quat"]
    ax = (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
          1.0 - 2.0 * (x * x + y * y))
    elev = math.degrees(math.atan2(ax[2], math.hypot(ax[0], ax[1])))
    clear, who = n.clearance(arm, q)
    apex = getattr(n, "last_apex_z", None)
    print("   %-22s hand (%+.3f, %+.3f, %.3f)  wrist %+.1f deg  "
          "elbow %.0f mm below shoulder" % (label, hand[0], hand[1], hand[2],
                                            elev, (sh[2] - elbow[2]) * 1000))
    print("   %-22s limb apex %.3f = %.0f mm %s the shoulder line %.2f   "
          "clearance %.4f m to %s"
          % ("", apex, abs(apex - FP.SHOULDER_LINE_Z) * 1000.0,
             "ABOVE" if apex > FP.SHOULDER_LINE_Z else "below",
             FP.SHOULDER_LINE_Z, clear, who))
    return dict(hand=[round(v, 4) for v in hand],
                elevation_deg=round(elev, 2),
                elbow_below_shoulder_m=round(sh[2] - elbow[2], 4),
                apex_z=round(apex, 4),
                apex_above_shoulder_m=round(apex - FP.SHOULDER_LINE_Z, 4),
                clearance_m=round(clear, 4), clearance_to=who)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", action="append", nargs="+", default=[],
                    help="LABEL ARM q1..q7, repeatable")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    rclpy.init()
    n = FP.Kin()
    n.spin(3.0)
    if not n.fk.wait_for_service(timeout_sec=20.0):
        print("no /compute_fk -- is the sim up?")
        return 2
    n.spin(2.0)

    import home_positions as hp
    print("CONTROLS")
    ok = True
    for arm in ("left", "right"):
        home = hp.load_home_radians(arm)
        c, who = n.clearance(arm, home)
        print("   home %-5s clearance %.4f m to %-12s (floor %.2f)  apex %.3f"
              % (arm, c, who, FP.MIN_CLEARANCE_M, n.last_apex_z))
        ok = ok and c is not None and c >= FP.MIN_CLEARANCE_M
    crashed = list(home)
    crashed[1] = -3.20                     # the same probe the search uses
    c2, who2 = n.clearance("left", crashed)
    print("   driven into the torso: %.4f m to %s  (must be negative)"
          % (c2, who2))
    ok = ok and c2 is not None and c2 < 0.0
    if not ok:
        print("\nREFUSING TO REPORT: a control failed.")
        return 6

    print("\nPOSES")
    out = {}
    for spec in a.pose:
        label, arm = spec[0], spec[1]
        q = [float(v) for v in spec[2:]]
        if len(q) != 7:
            print("   %s: need 7 joint values, got %d" % (label, len(q)))
            continue
        out["%s/%s" % (label, arm)] = report(n, arm, q, "%s %s" % (label, arm))
    if a.out:
        json.dump(out, open(a.out, "w"), indent=2)
        print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
