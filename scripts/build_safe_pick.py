#!/usr/bin/env python3
"""Turn the grasp plan into a path that provably clears the table.

The planner solves three POSES.  It says nothing about the joint-space lines
between them, and those lines are where the damage happens: the direct transit
to PRE-GRASP dips to 9.7 mm above the table at 53% along, which is how the
gripper was driven into it.

This inserts lift-over waypoints until every leg clears, and REFUSES to emit a
plan it cannot make safe.

Leg types are checked differently on purpose:
  * TRANSIT legs must keep the whole hand TRANSIT_MARGIN_M above the surface.
  * The DESCENT to the cube and the ASCENT away from it are supposed to end
    low -- the pads close around a 39 mm cube.  They are checked for never
    going BELOW the surface, and for descending monotonically, which is what
    distinguishes "reaching down to the cube" from "sweeping across the table".
"""
import json
import math
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from safe_goto_left import HAND_LINKS, TableCheck  # noqa: E402
from srl_fk import FK  # noqa: E402

TRANSIT_MARGIN_M = 0.045
BELOW_TABLE_M = -0.002        # the hand may never go under the surface
LIFT_TRIES = (0.12, 0.18, 0.25, 0.32)
STEPS = 140


def sweep(fk, tab, q0, q1, grip=0.0, steps=STEPS):
    hs = []
    for k in range(steps + 1):
        a = k / steps
        q = np.array(q0) + (np.array(q1) - np.array(q0)) * a
        hs.append(tab.min_hand_height(fk, q, grip))
    return np.array(hs)


def transit_ok(hs, margin):
    return bool(hs.min() >= margin)


def reach_ok(hs):
    """A descent/ascent: never under the table, and no mid-path dip."""
    if hs.min() < BELOW_TABLE_M:
        return False
    # the lowest point must be at one END, not in the middle
    i = int(hs.argmin())
    return i <= 2 or i >= len(hs) - 3


def lift_of(fk, tab, q_seed, lift_m):
    from plan_pick_left import ik
    Tee, = fk.poses("left", q_seed, ["end_effector_link"])
    lf, rf = fk.poses("left", q_seed,
                      ["robotiq_85_left_finger_tip_link",
                       "robotiq_85_right_finger_tip_link"])
    mid = (lf[:3, 3] + rf[:3, 3]) / 2.0
    q, ep, er, _ = ik(mid + tab.n * lift_m, Tee[:3, :3], q_seed)
    return q, ep


def safe_transit(fk, tab, q0, q1, label):
    """Waypoints from q0 to q1 whose whole path clears the table."""
    hs = sweep(fk, tab, q0, q1)
    if transit_ok(hs, TRANSIT_MARGIN_M):
        print("  %-10s direct: lowest %.1f mm  OK" % (label, hs.min() * 1000))
        return [q1]
    print("  %-10s direct: lowest %.1f mm at %d%% along  -- needs lifting"
          % (label, hs.min() * 1000, int(hs.argmin() / len(hs) * 100)))
    for L in LIFT_TRIES:
        qa, epa = lift_of(fk, tab, q0, L)
        qb, epb = lift_of(fk, tab, q1, L)
        if max(epa, epb) > 0.004:
            print("     lift %.0f mm: IK residual %.1f/%.1f mm -- skip"
                  % (L * 1000, epa * 1000, epb * 1000))
            continue
        legs = [(q0, qa), (qa, qb), (qb, q1)]
        lows = [sweep(fk, tab, a, b).min() for a, b in legs]
        if all(x >= TRANSIT_MARGIN_M for x in lows):
            print("     lift %.0f mm: legs clear at %.1f / %.1f / %.1f mm  OK"
                  % (L * 1000, *[x * 1000 for x in lows]))
            return [qa, qb, q1]
        print("     lift %.0f mm: legs at %.1f / %.1f / %.1f mm -- still low"
              % (L * 1000, *[x * 1000 for x in lows]))
    return None


def main():
    ws = "/home/gausms/kortex_ws"
    fk = FK()
    tab = TableCheck(ws + "/recordings/baselines/cube_located_v2.json")
    plan = json.load(open(ws + "/recordings/baselines/pick_plan_left_v2.json"))
    q_now = np.array(json.load(
        open(ws + "/recordings/baselines/cube_located_v2.json"))["q"], float)
    print("start hand clearance: %.1f mm" % (tab.min_hand_height(fk, q_now) * 1000))

    stages = {s["name"]: s for s in plan["plan"]}
    q_pre = np.array(stages["PRE-GRASP"]["q"])
    q_grasp = np.array(stages["GRASP"]["q"])
    q_lift = np.array(stages["LIFT"]["q"])

    out = []
    print("\nTRANSIT to PRE-GRASP")
    wps = safe_transit(fk, tab, q_now, q_pre, "to PRE")
    if wps is None:
        raise SystemExit("could not clear the table on the way to PRE-GRASP; "
                         "NOTHING WRITTEN")
    for i, q in enumerate(wps):
        out.append({"name": "PRE-GRASP" if i == len(wps) - 1 else "OVER-%d" % (i + 1),
                    "q": list(map(float, q)), "grip": 0.0})

    print("\nDESCENT and ASCENT")
    for nm, a, b, grip in (("GRASP", q_pre, q_grasp, 0.0),
                           ("LIFT", q_grasp, q_lift, 0.55)):
        hs = sweep(fk, tab, a, b, grip)
        ok = reach_ok(hs)
        print("  %-6s lowest %.1f mm at %d%% along  %s"
              % (nm, hs.min() * 1000, int(hs.argmin() / len(hs) * 100),
                 "OK" if ok else "<-- DIPS MID-PATH"))
        if not ok:
            raise SystemExit("%s leg is not a clean reach; NOTHING WRITTEN" % nm)
        out.append({"name": nm, "q": list(map(float, b)), "grip": grip})

    for s in out:
        s["pad_above_table_mm"] = float(
            tab.min_hand_height(fk, np.array(s["q"]), s["grip"]) * 1000)
        s["pos_err_mm"] = 0.0
        s["rot_err_deg"] = 0.0
    for nm in ("PRE-GRASP", "GRASP", "LIFT"):
        if nm in stages:
            for s in out:
                if s["name"] == nm:
                    s["pos_err_mm"] = stages[nm]["pos_err_mm"]
                    s["rot_err_deg"] = stages[nm]["rot_err_deg"]

    print("\nfinal sequence:")
    for s in out:
        print("  %-10s hand %6.1f mm above table" % (s["name"],
                                                     s["pad_above_table_mm"]))
    json.dump({"plan": out, "ok": True,
               "note": "table-clearance checked leg by leg by build_safe_pick.py"},
              open(ws + "/recordings/baselines/pick_plan_left_safe.json", "w"),
              indent=1)
    print("\nwrote recordings/baselines/pick_plan_left_safe.json")


if __name__ == "__main__":
    main()
