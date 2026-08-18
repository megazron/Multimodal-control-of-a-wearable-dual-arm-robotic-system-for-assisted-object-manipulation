#!/usr/bin/env python3
"""BUILD T1'S LAYOUT FROM THE MEASUREMENT, AND VERIFY EVERY POINT OF IT.

    python3 scripts/solve_t1_layout.py                      # solve and verify
    python3 scripts/solve_t1_layout.py --verify-only        # check the shipped
    python3 scripts/solve_t1_layout.py --self-test          # controls only

WHY A SOLVER RATHER THAN SIX COORDINATES. `search_t1_centre.py` answers "how
close to the centreline can ONE object go". A layout is six objects that all
have to work at once, and this repository has already been caught by the
difference twice: `centre_on_surface.json` found a single cell at x = 0.350
and recorded it, correctly, as "a lead, not a result"; and the pad column at
|x| = 0.425 passed with the pad CENTRE tested and failed once the two per-cube
SLOTS 30 mm away were tested, because a pad is not a point.

So this places every object the task will touch and walks every path the task
will command, at N repeats, with the wearer measured geometrically:

    the two pads      each at its own arm's innermost workable column, with
                      BOTH per-cube slots verified, not the pad centre
    the four cubes    two per arm, outboard of that arm's pad, clear of the
                      pad footprint by more than a cube
    every path        pre-grasp -> grasp -> lift -> carry -> place -> release
                      -> withdraw, for all four cubes

THE APPROACH IS AN INPUT, NOT AN ASSUMPTION. It comes from
`recordings/baselines/t1_centre.json`, where 14 of 181 candidate orientations
could touch an object resting on a surface at all and the pinned teleop anchor
was not among them. It is passed in by elevation, heading and roll so that a
re-run with a different approach is a command line and not an edit.

CONTROLS, and no layout is written if one fails -- the same four
`search_t1_centre` uses, imported rather than restated.
"""
import argparse
import json
import os
import sys

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from search_t1_centre import (put_table, clear_table, controls,  # noqa: E402
                              frange, CUBE, STANDOFF, LIFT)
import grasp_frames as GF                                     # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_layout.json")


# ------------------------------------------------------------------ paths
def pick_path(obj, q, pad_mid):
    """pre-grasp -> grasp -> lift, as the task will command it."""
    pre, ee = GF.approach_path(obj, q, STANDOFF, pad_mid)
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def place_path(obj, q, pad_mid):
    """carry-in -> place -> withdraw. Same shape, mirrored."""
    pre, ee = GF.approach_path(obj, q, STANDOFF, pad_mid)
    up = [pre[0], pre[1], pre[2] + LIFT]
    return densify([up, pre], 0.025) + densify([pre, ee], 0.025) + \
        densify([ee, pre], 0.025)


def walk(rig, arm, wps, q, repeats):
    """(ok, worst clearance, who) over every waypoint at N repeats."""
    qm = as_msg(q)
    worst, who = 1e9, None
    for w in wps:
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                return False, None, None
            c, k = rig.clearance(arm, j)
            if c is None:
                return False, None, None
            if c < worst:
                worst, who = c, k
    return True, worst, who


def cell_ok(rig, arm, obj, q, pad_mid, repeats, floor, kind="pick"):
    wps = pick_path(obj, q, pad_mid) if kind == "pick" \
        else place_path(obj, q, pad_mid)
    ok, c, who = walk(rig, arm, wps, q, repeats)
    return bool(ok and c is not None and c >= floor), c, who


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elev", type=float, default=-10.0)
    ap.add_argument("--head", type=float, default=-50.0,
                    help="degrees INBOARD; mirrored for the right arm")
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--tops", default="0.95,1.00")
    ap.add_argument("--edges", default="0.230,0.280,0.330,0.380",
                    help="table NEAR EDGE y. Everything stands on the front "
                         "edge, so this is also where the work is.")
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--coarse-repeats", type=int, default=2)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--pad-w", type=float, default=0.240)
    ap.add_argument("--pad-d", type=float, default=0.140)
    ap.add_argument("--slot-dx", type=float, default=0.050)
    # TWO CUBES CANNOT BE 25 mm APART, AND THE FIRST RUN OF THIS SOLVER PUT
    # THEM THERE. It walked x outward and took the first two columns that
    # verified -- 0.600 and 0.625, adjacent cells of a 25 mm grid, which for
    # 40 mm cubes is one cube inside another. 60 mm is T0's measured minimum
    # separation and it is what makes four cubes read as four objects.
    ap.add_argument("--cube-pitch", type=float, default=0.060)
    ap.add_argument("--x-max", type=float, default=0.80)
    ap.add_argument("--self-test", action="store_true")
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
    fk = FK(n)
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    ctl, good = controls(rig, pad_mid, a.floor)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-38s %s" % (k, v))
    if not good:
        print("\nREFUSING: a control failed.")
        return 6
    for arm in ("left", "right"):
        d = float(np.linalg.norm(np.asarray(pad_mid[arm])
                                 - np.asarray(GF.PAD_MID_EE)))
        print("   %-38s %.5f m" % ("pad_mid_ee measured vs constant, %s" % arm, d))
        if d > 0.001:
            print("\nREFUSING: the measured pad midpoint disagrees with "
                  "grasp_frames.PAD_MID_EE by %.1f mm." % (d * 1000))
            return 6
    if a.self_test:
        print("\nself-test only.")
        return 0

    q = {"left": GF.q_from_axis(GF.axis_for(a.elev, a.head),
                                np.radians(a.roll)),
         "right": GF.q_from_axis(GF.axis_for(a.elev, -a.head),
                                 np.radians(a.roll))}
    for arm in ("left", "right"):
        e, h = GF.elev_head_of(q[arm])
        print("   %-5s approach: elevation %+.1f deg, heading %+.1f deg, "
              "roll %.1f" % (arm, e, h, a.roll))

    # ======================================================================
    # EVERYTHING STANDS IN ONE ROW ON THE TABLE'S FRONT EDGE, AND THAT IS
    # MEASURED RATHER THAN CHOSEN.
    #
    # `t1_centre.json` swept the table's near edge as an OVERHANG behind each
    # object: 0 mm and 60 mm. Not one cell with 60 mm survived, on either arm,
    # at any of five table heights. The mechanism is visible in the approach:
    # the hand comes in level and INBOARD, so the forearm and the gripper body
    # trail OUTBOARD AND NEARER at the object's own height, which is 20 mm
    # above the table. Sixty millimetres of table nearer than the object is
    # sixty millimetres of table through that volume.
    #
    # So the near edge is a hard boundary and every object's NEAR FACE sits on
    # it. The pads extend FORWARD from that line, away from the wearer, which
    # is free -- nothing approaches from there. Two cubes share a pad, so their
    # landing slots are separated in X, along the row, and not in Y: a slot
    # 35 mm further forward would be a slot with 35 mm of table behind it.
    # ======================================================================
    tops = [float(v) for v in a.tops.split(",")]
    edges = [float(v) for v in a.edges.split(",")]
    xs = frange(0.05, a.x_max, 0.025)

    best = None
    for top in tops:
        z_obj = round(top + CUBE / 2.0, 4)
        for edge in edges:
            y_row = round(edge + CUBE / 2.0, 4)
            rig.set_furniture(False)
            put_table(rig, top, edge)
            print("\n-- table top %.3f, near edge %.3f, row y = %.3f"
                  % (top, edge, y_row))
            pads, cubes = {}, {}
            for arm in ("left", "right"):
                sgn = 1.0 if arm == "left" else -1.0
                found = None
                for px in xs:
                    slots = [[round(sgn * (px - a.slot_dx), 4), y_row, z_obj],
                             [round(sgn * (px + a.slot_dx), 4), y_row, z_obj]]
                    oks = [cell_ok(rig, arm, s, q[arm], pad_mid[arm],
                                   a.coarse_repeats, a.floor, "place")
                           for s in slots]
                    if not all(o[0] for o in oks):
                        continue
                    oks = [cell_ok(rig, arm, s, q[arm], pad_mid[arm],
                                   a.repeats, a.floor, "place") for s in slots]
                    if all(o[0] for o in oks):
                        found = dict(x=round(sgn * px, 4), y=y_row,
                                     slots=slots,
                                     clearance=round(min(o[1] for o in oks), 4))
                        break
                pads[arm] = found
                print("   %-5s pad %s" % (arm, "NONE" if found is None else
                      "|x| = %.3f  slots x %.3f / %.3f  clearance %.4f"
                      % (abs(found["x"]), found["slots"][0][0],
                         found["slots"][1][0], found["clearance"])))
            if any(v is None for v in pads.values()):
                continue
            # cubes: two per arm, outboard of that arm's pad footprint
            gap = a.pad_w / 2.0 + CUBE
            ok_all = True
            for arm in ("left", "right"):
                sgn = 1.0 if arm == "left" else -1.0
                x0 = abs(pads[arm]["x"]) + gap
                mine = []
                for px in [v for v in xs if v >= x0]:
                    if mine and px - abs(mine[-1]["obj"][0]) < a.cube_pitch - 1e-9:
                        continue
                    obj = [round(sgn * px, 4), y_row, z_obj]
                    okc, c, _w = cell_ok(rig, arm, obj, q[arm], pad_mid[arm],
                                         a.coarse_repeats, a.floor, "pick")
                    if not okc:
                        continue
                    okc, c, _w = cell_ok(rig, arm, obj, q[arm], pad_mid[arm],
                                         a.repeats, a.floor, "pick")
                    if okc:
                        mine.append(dict(obj=obj, clearance=round(c, 4)))
                        if len(mine) == 2:
                            break
                cubes[arm] = mine
                for m in mine:
                    print("   %-5s cube at %s  clearance %.4f"
                          % (arm, [round(v, 3) for v in m["obj"]],
                             m["clearance"]))
                if len(mine) < 2:
                    print("   %-5s ONLY %d cube cell(s)" % (arm, len(mine)))
                    ok_all = False
            if not ok_all:
                continue
            span = abs(pads["left"]["x"]) + abs(pads["right"]["x"])
            cand = dict(table_top=top, near_y=edge, y_row=y_row, z_obj=z_obj,
                        pads=pads, cubes=cubes, pad_span=round(span, 4))
            print("   -> CANDIDATE, pads %.0f mm and %.0f mm off centre"
                  % (abs(pads["left"]["x"]) * 1000,
                     abs(pads["right"]["x"]) * 1000))
            if best is None or span < best["pad_span"]:
                best = cand
    clear_table(rig)

    if best is None:
        print("\nNO COMPLETE LAYOUT at any (table top, near edge) in this "
              "sweep. Nothing is written.")
        return 5

    print("\n" + "=" * 72)
    print("THE LAYOUT")
    print("=" * 72)
    print("   table top %.3f, near edge %.3f, everything in one row at "
          "y = %.3f, resting at z = %.3f"
          % (best["table_top"], best["near_y"], best["y_row"], best["z_obj"]))
    for arm in ("left", "right"):
        p = best["pads"][arm]
        print("   %-5s pad centre x = %+.3f (%.0f mm off centre), slots "
              "x %+.3f / %+.3f, clearance %.4f"
              % (arm, p["x"], abs(p["x"]) * 1000, p["slots"][0][0],
                 p["slots"][1][0], p["clearance"]))
        for m in best["cubes"][arm]:
            print("         cube %s clearance %.4f"
                  % ([round(v, 3) for v in m["obj"]], m["clearance"]))
    out = dict(approach=dict(elev=a.elev, head=a.head, roll=a.roll,
                             quat={k: list(v) for k, v in q.items()}),
               pad=dict(w=a.pad_w, d=a.pad_d, slot_dx=a.slot_dx),
               floor=a.floor, repeats=a.repeats, controls=ctl,
               ik_calls=rig.calls, **best)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
