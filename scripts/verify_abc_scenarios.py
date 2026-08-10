#!/usr/bin/env python3
"""Verify every coordinate of TASKS A, B and C at N=10 over the FULL PATH.

TWO RULES, BOTH LEARNED THE HARD WAY (CLAUDE.md, "A POSE THAT PASSES ONE IK
CALL IS NOT A REACHABLE POSE"):

  N REPEATS, because TRAC-IK restarts randomly and a pose at the edge of the
  feasible set is a coin flip. Measured on this rig: (0.15, 0.35, 1.36) is
  72-82% feasible per call, and a k=2 check passes it 19-38% of the time --
  often enough to be written into a protocol, rare enough to look like bad
  luck on the day. N=10, short-circuiting on the first failure, so raising N
  costs almost nothing on poses that already pass.

  THE WHOLE PATH, because the arm FLIES the segments between declared
  waypoints. Endpoints being reachable says nothing about the 150 mm between
  them. Every path is densified to 20 mm.

N IS NOT A SUBSTITUTE FOR MARGIN. No finite N proves a pose reachable; it only
bounds how often the check lies. What saves this rig is that feasibility falls
off a CLIFF rather than degrading -- 100% at z = 1.34, 82% at 1.36 -- so the
real defence is keeping every declared figure well inside the last pose that
passed N/N, and using N to find that boundary.

    python3 scripts/verify_abc_scenarios.py [--repeats 10]

Refuses to run unless BOTH ARMS ARE AT HOME. This measures reach FROM HOME,
and a run taken with the arms parked elsewhere answers a different question
under the same name -- which has already invalidated one full reachability run
and half a front-reach run in this project.
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import tasks as T                                            # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import math                                                  # noqa: E402

STEP = 0.02
OUT = os.path.join(ROOT, "recordings/baselines/abc_verification.json")


def require_path(path, what):
    """AN EMPTY PATH MUST NOT VERIFY.

    all() over nothing is True, so a densify() that returned no waypoints
    would make this report "TOTAL FAILURES: 0" -- indistinguishable from
    having checked every waypoint. That is the mechanism that let a
    known-answer check pass on zero parsed rows, and it would sit directly
    under the result this protocol leans on hardest.
    """
    if not path:
        raise RuntimeError("densify returned no waypoints for %s; refusing to "
                           "report a pass on an empty path" % what)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=STEP)
    args = ap.parse_args()
    N, step = args.repeats, args.step

    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2

    print("=" * 74)
    print("TASKS A / B / C -- N=%d repeats over paths densified to %.0f mm"
          % (N, step * 1000))
    print("=" * 74)
    bad_home = False
    for a in ("left", "right"):
        w, j = n.home_ok(a)
        print("  %-5s home offset %.4f rad   %s"
              % (a, w, "ok" if w <= HOME_TOL_RAD else "NOT AT HOME (j%d)" % j))
        bad_home |= w > HOME_TOL_RAD
    if bad_home:
        print("\n  REFUSING: the arms are not at home. This would measure a "
              "different question under the same name.")
        return 3

    quat = {a: n.ee_quat(a) for a in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("\n  REFUSING: no tf2 for an arm's end effector. The anchor "
              "orientation must be the ARM'S OWN -- an identity quaternion is "
              "not neutral, it is a specific unreachable pose, and it has "
              "produced a false negative three times here.")
        return 4

    calls = {"n": 0}

    def ok(arm, p, k=None):
        k = N if k is None else k
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False            # short-circuit: N is cheap when it passes
        return True

    def pair(xc, y, z, sep):
        """Both grippers on the two ends of a span of `sep`.

        The arm assignment is tried BOTH ways on purpose. In this model the
        links named `left_*` sit at POSITIVE x -- the naming is viewer
        perspective, not anatomical -- and assuming otherwise made an earlier
        audit report every T3 waypoint unreachable when the geometry was fine.
        """
        a = [xc - sep / 2.0, y, z]
        b = [xc + sep / 2.0, y, z]
        return ((ok("left", b) and ok("right", a)) or
                (ok("left", a) and ok("right", b)))

    # ------------------------------------------------- INSTRUMENT CONTROLS
    # THE HEADLINE OF THIS SCRIPT IS A COUNT OF ZERO, and zero is also what a
    # verifier prints when it has stopped verifying -- a solver that says yes
    # to everything, a quaternion that makes every pose trivially fine, a
    # service that returns success on a request it never processed. "0
    # failures" and "0 poses actually tested" are indistinguishable in the
    # output. So the run is bracketed by two poses whose answers are known
    # independently of anything measured here, and it REFUSES TO REPORT if
    # either comes back wrong.
    print("\nINSTRUMENT CONTROLS (a zero is only meaningful if a one is "
          "possible)")
    ctl_far = ok("left", [1.60, 0.35, 1.15], k=1)          # past the 0.902 m reach
    ctl_in = ok("left", [0.0, -0.10, 1.25], k=1)           # inside the wearer
    ctl_good = ok("left", T.TASK_A["targets"]["left"][0], k=1)
    print("   far outside reach (1.60, 0.35, 1.15)   -> %s  (want unreachable)"
          % ("REACHABLE" if ctl_far else "unreachable"))
    print("   inside the wearer (0.00,-0.10, 1.25)   -> %s  (want unreachable)"
          % ("REACHABLE" if ctl_in else "unreachable"))
    print("   a declared Task A target                -> %s  (want reachable)"
          % ("reachable" if ctl_good else "UNREACHABLE"))
    if ctl_far or ctl_in or not ctl_good:
        print("\n  REFUSING TO REPORT. The verifier failed a control, so any "
              "count it prints -- especially a zero -- means nothing.")
        n.destroy_node()
        rclpy.shutdown()
        return 5

    out, fails = {}, 0
    out["controls"] = dict(far_unreachable=not ctl_far,
                           inside_wearer_unreachable=not ctl_in,
                           declared_target_reachable=bool(ctl_good))

    # ---------------------------------------------------------------- A
    print("\nTASK A  positioning -- targets, and the transit BETWEEN them")
    ta = {}
    for arm, pts in T.TASK_A["targets"].items():
        rows = [dict(p=p, ok=bool(ok(arm, p))) for p in pts]
        # The operator moves BETWEEN targets, so the segments are part of the
        # task even though the spec only names the endpoints. Every ordered
        # pair, because the block presents them in a randomised order.
        seg_bad = 0
        for p, q in itertools.combinations(pts, 2):
            path = require_path(densify([p, q], step), "A transit %s->%s" % (p, q))
            seg_bad += sum(1 for w in path if not ok(arm, w))
        ta[arm] = dict(targets=rows, transit_failures=seg_bad)
        fails += sum(1 for r in rows if not r["ok"]) + seg_bad
        print("   %-5s %d/%d targets, %d transit waypoint failures"
              % (arm, sum(r["ok"] for r in rows), len(rows), seg_bad))
    out["A"] = ta

    # ---------------------------------------------------------------- B
    print("\nTASK B  coordinated carry -- both grippers, whole path, %.0f mm span"
          % (T.TRAY_SEP * 1000))
    tb = {}
    for name, path in T.TASK_B["paths"].items():
        dense = require_path(densify(path, step), "B path %s" % name)
        bad = [w for w in dense if not pair(w[0], w[1], w[2], T.TRAY_SEP)]
        tb[name] = dict(waypoints=len(dense), bad=len(bad), ok=not bad)
        fails += len(bad)
        print("   %-16s %2d waypoints   %s"
              % (name, len(dense),
                 "VERIFIED" if not bad else "%d FAIL" % len(bad)))
    # The rigid and compliant objects share these paths exactly, so one
    # verification covers both. What differs is the FAILURE THRESHOLD, and
    # that is arithmetic on the same geometry -- checked below, not on the arm.
    sag = ((T.SLING_L / 2.0) ** 2 - (T.TRAY_SEP / 2.0) ** 2) ** 0.5
    s_max = 2.0 * ((T.SLING_L / 2.0) ** 2 - (2 * T.BALL_R) ** 2) ** 0.5
    tb["object_factor"] = dict(
        sag_at_nominal_m=round(sag, 5),
        ball_diameter_m=2 * T.BALL_R,
        retained_at_nominal=bool(sag >= 2 * T.BALL_R),
        s_max_m=round(s_max, 4),
        declared_fail_sep_m=T.TASK_B["fail_sep_m"],
        margin_mm=round(1000 * (s_max - T.TRAY_SEP), 1),
    )
    print("   rigid/compliant share the paths; only the threshold differs:")
    print("     sag at %.0f mm span = %.1f mm vs a %.0f mm ball -> %s"
          % (T.TRAY_SEP * 1000, sag * 1000, 2000 * T.BALL_R,
             "RETAINED" if sag >= 2 * T.BALL_R else "ALREADY DROPPED"))
    print("     s_max = %.1f mm, declared %.1f mm, margin %.1f mm"
          % (s_max * 1000, T.TASK_B["fail_sep_m"] * 1000,
             1000 * (s_max - T.TRAY_SEP)))
    if sag < 2 * T.BALL_R:
        fails += 1
        print("     FAIL: the ball is gone before the trial starts")
    if abs(s_max - T.TASK_B["fail_sep_m"]) > 5e-4:
        fails += 1
        print("     FAIL: declared fail_sep_m disagrees with the geometry")
    # The threshold must be INSIDE the reachable band, or the task cannot fail.
    if not (T.TRAY_SEP < s_max):
        fails += 1
        print("     FAIL: threshold is not beyond the nominal span")
    out["B"] = tb

    # ---------------------------------------------------------------- C
    print("\nTASK C  dual pursuit -- the amplitude SHELL, 26 directions per arm")
    tc = {}
    dirs = [np.array(d, float) / np.linalg.norm(d)
            for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
    for arm, c in T.TASK_C["centres"].items():
        c = np.asarray(c, float)
        amp = T.TASK_C["shell_radius_m"]
        bad = [list(np.round(c + amp * d, 4)) for d in dirs
               if not ok(arm, list(c + amp * d))]
        # A Lissajous is not a straight line, so the SHELL is the right object
        # to verify: any point the marker can reach lies on or inside it. The
        # radial segment from the centre out is checked too, because the
        # marker crosses the interior continuously.
        radial = require_path(densify([list(c), list(c + amp * dirs[0])], step),
                              "C radial")
        rbad = sum(1 for w in radial if not ok(arm, w))
        tc[arm] = dict(centre=[float(x) for x in c], amplitude_m=amp,
                       shell_fail=len(bad), radial_fail=rbad,
                       failed_points=bad, ok=not bad and not rbad)
        fails += len(bad) + rbad
        print("   %-5s centre %s  %d/26 shell directions, %d radial failures"
              % (arm, [round(float(x), 3) for x in c], 26 - len(bad), rbad))
    out["C"] = tc

    # -------------------------------------------------- optional, not timed
    print("\nOPTIONAL pick and place -- NOT in the timeline, NOT comparative")
    td = {}
    for arm, picks in T.OPTIONAL_PICK_PLACE["picks"].items():
        rows = []
        for p in picks:
            pre = [p[0], p[1], p[2] + T.OPTIONAL_PICK_PLACE["standoff_m"]]
            path = require_path(densify([pre, p], step), "pick approach %s" % p)
            good = all(ok(arm, w) for w in path)
            binp = T.OPTIONAL_PICK_PLACE["bins"][arm]
            place = require_path(
                densify([[binp[0], binp[1], binp[2] + 0.12], binp], step),
                "place at %s" % binp)
            goodb = all(ok(arm, w) for w in place)
            rows.append(dict(pick=p, approach_ok=bool(good),
                             place_ok=bool(goodb),
                             n_way=len(path) + len(place)))
        td[arm] = rows
        print("   %-5s %d/%d approaches, %d/%d places"
              % (arm, sum(r["approach_ok"] for r in rows), len(rows),
                 sum(r["place_ok"] for r in rows), len(rows)))
    # NOT added to `fails`: this block is outside the protocol, and failing
    # the run on an optional demonstration would block the three tasks that
    # are not optional.
    out["optional_pick_place"] = td

    # ------------------------------------------------- THE CLIP TASK PATHS
    # THE RECORDING SET IS A DIFFERENT SPEC AND WAS NEVER VERIFIED HERE.
    # tasks.py is the participant protocol; clip_tasks.py is what the sweep
    # actually drives, and its coordinates were only ever checked by eye. That
    # is how task A shipped with a bin 0.132 m from the pick of which 0.130 m
    # was VERTICAL -- verified geometry running the wrong task. These paths
    # now get the same full-path N treatment as the protocol's own.
    print("\nCLIP TASKS  the paths the recording sweep actually drives")
    tcl = {}
    for key in ("a", "b", "c"):
        wp = CT.TASKS[key]["build"]()
        per = {}
        for arm, pts in wp.items():
            # Collapse the held repeats: a static hold re-rolls the same pose
            # once per waypoint, which turns a 60/60 pose into a random
            # failure somewhere along the path. Verify each DISTINCT pose.
            seen, uniq = set(), []
            for p in pts:
                k = tuple(round(v, 4) for v in p)
                if k not in seen:
                    seen.add(k)
                    uniq.append(list(p))
            dense = require_path(densify(uniq, step) if len(uniq) > 1 else uniq,
                                 "clip %s/%s" % (key, arm))
            bad = [w for w in dense if not ok(arm, w)]
            per[arm] = dict(distinct=len(uniq), waypoints=len(dense),
                            bad=len(bad), ok=not bad)
            fails += len(bad)
            print("   %s %-5s %2d distinct -> %3d waypoints   %s"
                  % (key.upper(), arm, len(uniq), len(dense),
                     "VERIFIED" if not bad else "%d FAIL" % len(bad)))
        tcl[key] = per
    # Task A's transport must be a CARRY. Verified as arithmetic on the
    # declared coordinates, not by eye.
    lat = math.hypot(CT.A_BIN[0] - CT.A_PICK[0], CT.A_BIN[1] - CT.A_PICK[1])
    tcl["a_lateral_m"] = round(lat, 4)
    tcl["a_vertical_m"] = round(abs(CT.A_BIN[2] - CT.A_PICK[2]), 4)
    print("   A transport: %.3f m lateral, %.3f m vertical  %s"
          % (lat, abs(CT.A_BIN[2] - CT.A_PICK[2]),
             "CARRY" if lat >= 0.25 else "FAIL -- a drop, not a carry"))
    if lat < 0.25:
        fails += 1
    out["clip_tasks"] = tcl

    payload = dict(repeats=N, step_m=step, ik_calls=calls["n"],
                   failures=fails, detail=out)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(payload, open(OUT, "w"), indent=2)
    print("\n" + "=" * 74)
    print("  %d IK calls   TOTAL FAILURES: %d" % (calls["n"], fails))
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
