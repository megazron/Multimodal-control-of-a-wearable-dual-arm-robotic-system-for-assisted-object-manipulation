#!/usr/bin/env python3
"""Define every task scenario and CONFIRM its reachability before it is
written into a protocol. A scenario that fails IK on the day wastes a
participant.

Writes src/srl_experiments/experiments/bimanual/scenarios_verified.yaml.
Only scenarios that pass are marked `verified: true`; failures are kept in
the file with the reason, so nobody silently re-adds one.

N REPEATS OVER THE WHOLE PATH -- 2026-08-08
-------------------------------------------
This used to check each pose TWICE and only at the declared waypoints. Both
were too weak, and the audit (scripts/audit_scenario_reachability.py) found
what they missed:

  * a pose at the edge of the feasible set is a coin flip, because TRAC-IK
    restarts randomly. T2's x = +/-0.10 scored 0/5 to 4/5 by height. A pose
    that is 60% feasible passes a 2-call check 36% of the time -- often
    enough to be written down, rare enough to look like bad luck later;
  * the straight segment BETWEEN two reachable waypoints is not itself
    guaranteed reachable, and the arm flies it. T3's S3 detour declared 4
    waypoints spanning 200 mm and left 150 mm unchecked between each pair;
  * for T7 the marker traces a Lissajous on a SPHERE, so checking the centre
    and two points on a vertical line through it checks almost none of it.

So: REPEATS = 5, every path densified to 20 mm, and T7 centres chosen by
requiring all 26 directions of the amplitude shell to pass 5/5.
"""
import itertools, math, os, sys, json
import numpy as np, rclpy, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver
from audit_scenario_reachability import densify

Y = 0.35                      # the only y that works (band 0.35-0.40)
NOM = 0.310                   # nominal gripper separation, mid-overlap
REPEATS = 10                  # 5 lets a 72% pose through 19% of the time
STEP = 0.02                   # path densification, metres
AMP = 0.08                    # T7 marker amplitude

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
                                 # RE-ROUTED AGAIN 2026-08-08. y=0.40 is not
                                 # a usable detour at a 310 mm separation --
                                 # measured 0/5 at z 1.15, 1.25 and 1.30 and
                                 # only 2/5 at 1.20. The previous re-route
                                 # moved the TOP of the detour and kept the
                                 # y, which was the part that did not work.
                                 # y=0.38 is 5/5 across z 1.15-1.30, so the
                                 # detour is now 20 mm shallower and real.
                                 path=[(0.0,0.35,1.15),(0.0,0.38,1.20),
                                       (0.0,0.38,1.25),(0.0,0.35,1.30)],
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

    def pair(xc, y, z, sep, k=REPEATS):
        pl = [xc - sep/2, y, z]; pr = [xc + sep/2, y, z]
        for _ in range(k):
            if not ((n.solve("left", pr, q["left"], tries=6) and
                     n.solve("right", pl, q["right"], tries=6)) or
                    (n.solve("left", pl, q["left"], tries=6) and
                     n.solve("right", pr, q["right"], tries=6))):
                return False
        return True

    def single(arm, p, k=REPEATS):
        return all(n.solve(arm, p, q[arm], tries=6) for _ in range(k))

    def shell_ok(arm, centre, r=AMP):
        """Every one of the 26 directions of the amplitude sphere, N times.

        The marker traces a Lissajous ON this sphere. Checking the centre and
        two points on a vertical line through it -- which is what this did --
        leaves the whole equator unchecked, and that is where both arms
        actually fail: at z=1.05 the left shell had 4 of 26 directions
        non-solid and the right had 1. The right arm's locus check PASSED
        anyway, purely because the sampled path missed its one bad direction.
        """
        c = np.asarray(centre, float)
        for d in itertools.product((-1, 0, 1), repeat=3):
            if not any(d):
                continue
            v = np.asarray(d, float)
            if not single(arm, list(c + r * v / np.linalg.norm(v))):
                return False
        return True

    T3, T7, T5 = build()
    out = {"frame": "world", "note":
           "+x is the wearer's RIGHT, +y FORWARD, +z UP. Verified against a "
           "live /compute_ik with both arms VERIFIED at home. Every path is "
           "densified to %.0f mm and every pose solved %d times; a pose that "
           "passes once is NOT a reachable pose." % (1000 * STEP, REPEATS),
           "repeats": REPEATS, "path_step_m": STEP,
           "nominal_separation_m": NOM, "tasks": {}}

    print("SCENARIO VERIFICATION\n")
    print("T3 / T6 coupled transport (identical paths, different coupling)")
    t3o = {}
    for name, s in T3.items():
        dense = densify(s["path"], STEP)
        bad = [w for w in dense if not pair(w[0], w[1], w[2], s["sep"])]
        ok = not bad
        t3o[name] = dict(**{k: v for k, v in s.items() if k != "path"},
                         path=[list(map(float, w)) for w in s["path"]],
                         verified=bool(ok),
                         reason="" if ok else "unreachable waypoints: %s" % bad)
        print("  %-20s %-9s %d waypoints  %s"
              % (name, "VERIFIED" if ok else "FAILED", len(dense), s["why"]))
        if bad:
            print("        unreachable: %s" % [list(map(float, b)) for b in bad])
    out["tasks"]["T3_rigid"] = t3o
    out["tasks"]["T6_compliant"] = t3o          # same paths, different object

    print("\nT7 pursuit -- target CENTRES must be inside each arm's own set")
    # each arm tracks inside its OWN reachable region; no shared point needed
    centres = {"left": None, "right": None}
    for arm, xs in (("left", (0.40, 0.45, 0.50)), ("right", (-0.40, -0.45, -0.50))):
        for x in xs:
            # z=1.05 is EXCLUDED by measurement, not by taste: the shell
            # fails there on both arms. See shell_ok.
            for z in (1.10, 1.15, 1.20):
                c = [x, Y, z]
                if shell_ok(arm, c):
                    centres[arm] = c; break
            if centres[arm]: break
    print("  target centres: left %s   right %s" % (centres["left"], centres["right"]))
    amp = AMP
    # all() over an EMPTY dict is True. centres is built by a search that can
    # find nothing, so require both arms to be present as well as truthy.
    ok7 = (len(centres) == 2 and all(centres.get(a) for a in ("left", "right")))
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
        o, r = list(s["object"]), list(s["receive"])
        # The arm FLIES this. Endpoints alone would pass a delivery whose
        # transit is unreachable, which is the shape that wastes a session.
        wps = [[o[0], o[1], o[2] + 0.08], o, [o[0], o[1], o[2] + 0.08],
               [(o[0] + r[0]) / 2, (o[1] + r[1]) / 2, max(o[2], r[2]) + 0.08],
               [r[0], r[1], r[2] + 0.06], r, [r[0], r[1], r[2] + 0.08]]
        dense = densify(wps, STEP)
        bad = [w for w in dense if not single(a, w)]
        ok = not bad
        t5o[name] = dict(**s, arm=a,
                         path=[[round(float(v), 3) for v in w] for w in dense],
                         verified=bool(ok),
                         reason="" if ok else "unreachable delivery waypoints: "
                                              "%s" % [[round(v, 3) for v in b]
                                                      for b in bad[:4]])
        print("  %-10s %-9s object %s receive %s  path %d/%d"
              % (name, "VERIFIED" if ok else "FAILED", s["object"],
                 s["receive"], len(dense) - len(bad), len(dense)))
    out["tasks"]["T5_handover"] = t5o

    p = ("src/srl_experiments/experiments/bimanual/scenarios_verified.yaml")
    # MERGE, never replace. T2 is generated by verify_t2_scenarios.py and
    # merged into this same file; a bare safe_dump here silently deleted it,
    # so re-running this script made the runner refuse t2 again with no
    # indication of why. Task keys this script does not own are preserved.
    if os.path.exists(p):
        prev = yaml.safe_load(open(p)) or {}
        for k, v in (prev.get("tasks") or {}).items():
            out["tasks"].setdefault(k, v)
    yaml.safe_dump(out, open(p, "w"), sort_keys=False)
    tot = sum(1 for t in out["tasks"].values() for s in t.values() if s["verified"])
    all_ = sum(len(t) for t in out["tasks"].values())
    print("\n  %d of %d scenarios VERIFIED -> %s" % (tot, all_, p))
    n.destroy_node(); rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
