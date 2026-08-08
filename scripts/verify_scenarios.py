#!/usr/bin/env python3
"""Define every task scenario and CONFIRM its reachability before it is
written into a protocol. A scenario that fails IK on the day wastes a
participant.

Writes src/srl_experiments/experiments/bimanual/scenarios_verified.yaml.
Only scenarios that pass are marked `verified: true`; failures are kept in
the file with the reason, so nobody silently re-adds one.
"""
import math, os, sys, json
import numpy as np, rclpy, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver

Y = 0.35                      # the only y that works (band 0.35-0.40)
NOM = 0.310                   # nominal gripper separation, mid-overlap

def build():
    """Scenarios. Difficulty comes from the PATH and the coupling."""
    def lift(z0, z1, n=3, y=Y, xc=0.0):
        return [(xc, y, z0 + (z1 - z0) * k / (n - 1)) for k in range(n)]
    T3 = {
      "S1_short_straight":  dict(sep=NOM, tol_deg=8.0,  path=lift(1.15, 1.20, 3),
                                 why="short vertical lift, generous tilt tolerance"),
      "S2_long_height":     dict(sep=NOM, tol_deg=6.0,  path=lift(1.10, 1.30, 5),
                                 why="full-band lift, twice the travel"),
      "S3_curved_obstacle": dict(sep=NOM, tol_deg=6.0,
                                 # Re-routed: the y=0.40 band does NOT extend
                                 # to z=1.26 (measured, unreachable). These
                                 # four waypoints are all in the verified
                                 # placement list.
                                 path=[(0.0,0.35,1.15),(0.0,0.40,1.20),
                                       (0.0,0.40,1.25),(0.0,0.35,1.30)],
                                 why="detour in y around a virtual obstacle"),
      "S4_tight":           dict(sep=0.330, tol_deg=3.8, path=lift(1.15, 1.25, 4),
                                 why="separation 330 mm, 10 mm from the 340 mm "
                                     "sling failure threshold; tight tilt tolerance"),
    }
    T7 = {
      "S1_both_slow":   dict(speed_left=0.05, speed_right=0.05),
      "S2_one_fast":    dict(speed_left=0.20, speed_right=0.05),
      "S3_both_fast":   dict(speed_left=0.20, speed_right=0.20),
      "S4_asymmetric":  dict(speed_left=0.30, speed_right=0.08, unpredictable=True),
      # unimanual baselines -- REQUIRED for the conventional dual-task cost
      "B1_left_only":   dict(speed_left=0.20, speed_right=0.0, unimanual="left"),
      "B2_right_only":  dict(speed_left=0.0,  speed_right=0.20, unimanual="right"),
    }
    T5 = {
      "S1_near":  dict(object=[-0.30, 0.35, 1.05], receive=[-0.16, 0.35, 1.05],
                       hands_occupied=False),
      "S2_far":   dict(object=[-0.40, 0.35, 1.15], receive=[-0.20, 0.35, 1.10],
                       hands_occupied=False),
      "S3_busy":  dict(object=[-0.30, 0.35, 1.10], receive=[-0.16, 0.35, 1.05],
                       hands_occupied=True),
    }
    return T3, T7, T5


def main():
    rclpy.init(); n = Solver(); n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik"); return 2
    q = {a: n.ee_quat(a) for a in ("left", "right")}

    def pair(xc, y, z, sep, k=2):
        pl = [xc - sep/2, y, z]; pr = [xc + sep/2, y, z]
        for _ in range(k):
            if not ((n.solve("left", pr, q["left"], tries=6) and
                     n.solve("right", pl, q["right"], tries=6)) or
                    (n.solve("left", pl, q["left"], tries=6) and
                     n.solve("right", pr, q["right"], tries=6))):
                return False
        return True

    def single(arm, p, k=2):
        return all(n.solve(arm, p, q[arm], tries=6) for _ in range(k))

    T3, T7, T5 = build()
    out = {"frame": "world", "note":
           "+x is the wearer's RIGHT, +y FORWARD, +z UP. Verified against a "
           "live /compute_ik with the arms at home; every waypoint solved "
           "twice.", "nominal_separation_m": NOM, "tasks": {}}

    print("SCENARIO VERIFICATION\n")
    print("T3 / T6 coupled transport (identical paths, different coupling)")
    t3o = {}
    for name, s in T3.items():
        bad = [w for w in s["path"] if not pair(w[0], w[1], w[2], s["sep"])]
        ok = not bad
        t3o[name] = dict(**{k: v for k, v in s.items() if k != "path"},
                         path=[list(map(float, w)) for w in s["path"]],
                         verified=bool(ok),
                         reason="" if ok else "unreachable waypoints: %s" % bad)
        print("  %-20s %-9s %d waypoints  %s"
              % (name, "VERIFIED" if ok else "FAILED", len(s["path"]), s["why"]))
        if bad:
            print("        unreachable: %s" % [list(map(float, b)) for b in bad])
    out["tasks"]["T3_rigid"] = t3o
    out["tasks"]["T6_compliant"] = t3o          # same paths, different object

    print("\nT7 pursuit -- target CENTRES must be inside each arm's own set")
    # each arm tracks inside its OWN reachable region; no shared point needed
    centres = {"left": None, "right": None}
    for arm, xs in (("left", (0.40, 0.45, 0.50)), ("right", (-0.40, -0.45, -0.50))):
        for x in xs:
            for z in (1.05, 1.10, 1.15):
                c = [x, Y, z]
                if single(arm, c) and single(arm, [x, Y, z + 0.08]) and \
                   single(arm, [x, Y, z - 0.08]):
                    centres[arm] = c; break
            if centres[arm]: break
    print("  target centres: left %s   right %s" % (centres["left"], centres["right"]))
    amp = 0.08
    ok7 = all(centres.values())
    t7o = {}
    for name, s in T7.items():
        t7o[name] = dict(**s, centre_left=centres["left"],
                         centre_right=centres["right"], amplitude_m=amp,
                         verified=bool(ok7),
                         reason="" if ok7 else "no reachable target centre")
        print("  %-16s left %.2f m/s  right %.2f m/s  %s"
              % (name, s.get("speed_left", 0), s.get("speed_right", 0),
                 "VERIFIED" if ok7 else "FAILED"))
    out["tasks"]["T7_pursuit"] = t7o

    print("\nT5 handover to wearer (single arm)")
    t5o = {}
    for name, s in T5.items():
        a = "right"
        ok = single(a, s["object"]) and single(a, s["receive"])
        t5o[name] = dict(**s, arm=a, verified=bool(ok),
                         reason="" if ok else "object or receive point unreachable")
        print("  %-10s %-9s object %s receive %s"
              % (name, "VERIFIED" if ok else "FAILED", s["object"], s["receive"]))
    out["tasks"]["T5_handover"] = t5o

    p = ("src/srl_experiments/experiments/bimanual/scenarios_verified.yaml")
    yaml.safe_dump(out, open(p, "w"), sort_keys=False)
    tot = sum(1 for t in out["tasks"].values() for s in t.values() if s["verified"])
    all_ = sum(len(t) for t in out["tasks"].values())
    print("\n  %d of %d scenarios VERIFIED -> %s" % (tot, all_, p))
    n.destroy_node(); rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
