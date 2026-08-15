#!/usr/bin/env python3
"""T1 AND T1 STAGE 2, EVERY WAYPOINT: reachable, and clear of the wearer.

    python3 scripts/verify_t1_paths.py --repeats 10
    python3 scripts/verify_t1_paths.py --candidate cubes=... planes=... arm=left
    python3 scripts/verify_t1_paths.py --stage2-seeds 0 1 2 3 4

IT DRIVES THE TASK'S OWN PATH, not a reconstruction of it. `verify_msc_tasks`
carries its own copy of T1_CUBES, T1_PLANES and T1_ARM, and that copy went
stale: it still reads left-arm cubes at y = 0.230 and planes at y = 0.145
while the task has been running right-arm cubes at y = 0.190 and planes at
0.300. It verified a layout nothing runs and reported zero failures for it.
So this script imports `msc_clip_tasks.t1()` and walks the list the recorder
will actually send.

IT MEASURES TWO THINGS PER WAYPOINT AND THE SECOND ONE IS NEW:

  1. does collision-aware IK solve it, N times over -- the existing question;
  2. how close does the arm get to the WEARER at that solution.

Nothing in this repository has ever asked (2) of a task coordinate. The
survey, the layout search and the scenario audit are all built on
/compute_ik with `avoid_collisions`, and the SRDF permanently excludes
torso/harness/backpack against each arm's base, shoulder and half_arm_1 --
the pairs a shoulder-mounted arm actually threatens. MoveIt therefore returns
`valid` for poses with the tube inside the person, which is why
find_presentation_pose checks clearance GEOMETRICALLY and says so.

Measured before this script existed: 18 of the left arm's 41 IK-reachable
cells and 27 of the right arm's 80 are inside the 150 mm floor, one of them
at -2.7 mm. HARD CONSTRAINT 11 is not a runtime detail to be discovered on
the day; it is a property of the layout, and it belongs here.

CONTROLS, and no report without them:
    a waypoint driven into the wearer   must FAIL both checks
    the home pose                       must clear the floor
    the shipped path with no furniture  must have no MORE failures
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_paths.json")


def walk_path(rig, arm, pts, repeats, floor):
    """(failures, worst_clearance, who, n_below_floor) over a waypoint list."""
    bad, worst, who, below = 0, 1e9, None, 0
    for w in pts:
        j = None
        okall = True
        for _ in range(repeats):
            j = rig.solve_joints(arm, w, avoid=True)
            if j is None:
                okall = False
                break
        if not okall:
            bad += 1
            continue
        c, k = rig.clearance(arm, j)
        if c is None:
            continue
        if c < worst:
            worst, who = c, k
        if c < floor:
            below += 1
    return bad, (None if worst > 1e8 else worst), who, below


def report(name, arm, pts, rig, repeats, floor, acc):
    bad, worst, who, below = walk_path(rig, arm, pts, repeats, floor)
    acc[name] = dict(arm=arm, waypoints=len(pts), ik_failures=bad,
                     worst_clearance_m=None if worst is None else round(worst, 4),
                     clearance_to=who, waypoints_below_floor=below)
    print("   %-22s %-5s %4d wp   IK fail %3d   worst clearance %s to %-11s"
          "   below floor %3d"
          % (name, arm, len(pts), bad,
             "  ----" if worst is None else "%.4f" % worst,
             who or "-", below))
    return bad, below


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--stage2-seeds", type=int, nargs="*", default=[0, 1, 2])
    # RUN IT IN PARTS, because this machine kills processes at about twenty
    # minutes and N=10 over both stages does not fit in one. The parts are
    # separate INVOCATIONS rather than a resume file: each writes its own
    # result and each re-runs its own controls, so a part cannot inherit a
    # control that passed in a process that has since died.
    ap.add_argument("--part", default="all",
                    choices=["all", "path", "objects", "stage2"])
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)

    # ---------------------------------------------------------- controls
    ctl = {}
    bad_c, worst_c, _, below_c = walk_path(
        rig, "left", [[0.05, -0.02, 1.22], [0.02, -0.05, 1.20]], 1, a.floor)
    ctl["driven_into_the_wearer"] = dict(
        want="IK failures > 0 or clearance < 0",
        ik_failures=bad_c,
        worst_clearance_m=None if worst_c is None else round(worst_c, 4))
    home = [n.js.get(k, 0.0) for k in n.names("left")]
    c_h = rig.clearance("left", home)[0]
    ctl["home_clears_floor"] = dict(want=">= %.2f" % a.floor, got=round(c_h, 4))
    ok = ((bad_c > 0 or (worst_c is not None and worst_c < 0.0))
          and c_h >= a.floor)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-24s %s" % (k, v))
    if not ok:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    acc = {}
    print("\nT1 STAGE 1 -- arm %s, %d cubes, %d planes, N=%d, floor %.2f m"
          % (MCT.T1_ARM.upper(), len(MCT.T1_CUBES), len(MCT.T1_PLANES),
             a.repeats, a.floor))
    print("   layout  cubes %s" % MCT.T1_CUBES)
    print("           planes %s" % MCT.T1_PLANES)
    path = MCT.t1()
    tot_bad = tot_below = 0
    if a.part in ("all", "path"):
        b1, u1 = report("t1 full path", MCT.T1_ARM, path[MCT.T1_ARM], rig,
                        a.repeats, a.floor, acc)
        other = "left" if MCT.T1_ARM == "right" else "right"
        b2, u2 = report("t1 idle arm hold", other, path[other][:1], rig,
                        a.repeats, a.floor, acc)
        tot_bad += b1 + b2
        tot_below += u1 + u2

    # PER OBJECT, so a failure names the cube rather than a waypoint index.
    if a.part in ("all", "objects"):
        print("\n   per object:")
        for i, (cx, cy) in enumerate(MCT.T1_CUBES):
            ee = CT.ee_for([cx, cy, MCT.T1_Z], MCT.T1_ARM)
            pts = [[ee[0], ee[1], ee[2] + MCT.STANDOFF], ee,
                   [ee[0], ee[1], ee[2] + MCT.LIFT]]
            b, u = report("cube_%d" % i, MCT.T1_ARM, pts, rig, a.repeats,
                          a.floor, acc)
            tot_bad += b
            tot_below += u
        for i, (px, py) in enumerate(MCT.T1_PLANES):
            for s in (-MCT.SLOT_DY, +MCT.SLOT_DY):
                ee = CT.ee_for([px, round(py + s, 4), MCT.T1_Z], MCT.T1_ARM)
                pts = [[ee[0], ee[1], ee[2] + MCT.LIFT], ee,
                       [ee[0], ee[1], ee[2] + MCT.STANDOFF]]
                b, u = report("plane_%d slot %+.3f" % (i, s), MCT.T1_ARM, pts,
                              rig, a.repeats, a.floor, acc)
                tot_bad += b
                tot_below += u

    s2 = {}
    if a.part in ("all", "stage2"):
        print("\nT1 STAGE 2 -- both arms, random from the surveyed cells, "
              "%d seeds" % len(a.stage2_seeds))
        for sd in a.stage2_seeds:
            p = MCT.t1_stage2(sd)
            tg = MCT._T1S2["targets"]
            row = {}
            for arm in ("left", "right"):
                bad, worst, who, below = walk_path(rig, arm, p[arm],
                                                   a.repeats, a.floor)
                row[arm] = dict(waypoints=len(p[arm]), ik_failures=bad,
                                worst_clearance_m=None if worst is None
                                else round(worst, 4), clearance_to=who,
                                waypoints_below_floor=below,
                                cubes=[[round(v, 4) for v in c]
                                       for c in tg[arm]])
                print("   seed %-3d %-5s %4d wp   IK fail %3d   worst "
                      "clearance %s   below floor %3d"
                      % (sd, arm, len(p[arm]), bad,
                         "  ----" if worst is None else "%.4f" % worst, below))
                tot_bad += bad
                tot_below += below
            s2["seed_%d" % sd] = row

    res = dict(repeats=a.repeats, floor=a.floor, controls=ctl,
               arm=MCT.T1_ARM, cubes=MCT.T1_CUBES, planes=MCT.T1_PLANES,
               z=MCT.T1_Z, stage1=acc, stage2=s2, ik_calls=rig.calls,
               total_ik_failures=tot_bad,
               total_waypoints_below_floor=tot_below)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("\nTOTAL: %d IK failures, %d waypoints inside the %.2f m clearance "
          "floor.  %d IK calls -> %s"
          % (tot_bad, tot_below, a.floor, rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if (tot_bad == 0 and tot_below == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
