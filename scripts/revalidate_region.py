#!/usr/bin/env python3
"""RE-VALIDATE THE SURVEYED REGION FROM THE CURRENT HOME, AT THE ANCHOR.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    python3 scripts/revalidate_region.py --repeats 10

WHY. `recordings/baselines/work_surface_region.json` was surveyed on
2026-08-15. It is the source of TWO things a participant is exposed to:

  * the WORKSPACE MARKING drawn on screen, which a participant is told to work
    inside;
  * the STAGE 2 SAMPLING POOL, which is where t1s2's cubes are drawn from.

Both are stale, for two independent reasons found on 2026-08-16:

  1. THE HOME MOVED. `/compute_ik` seeds from the live joint state, so the
     branch it lands in -- and therefore the clearance of the solution -- moved
     with it.
  2. THE SURVEY SOLVED AT THE WRONG ORIENTATION. It goes through
     `measure_what_binds.Rig`, whose `self.quat` is `node.ee_quat(arm)`, the
     LIVE end-effector orientation read off TF at construction. That is the
     HOME wrist, not `WORKSPACE_ORIENT`, the pinned 30.7 deg near-side anchor
     that `run_abc.send()` writes into every waypoint of every task.

Measured consequence, before this file existed: t1s2 drew a cube on the right
arm's near row and the STANDOFF above it sat 0.0772 m from the wearer against
a 0.150 m floor, with zero IK failures -- so every IK-based check called it
clean. That is HARD CONSTRAINT 11 breached by a marking.

WHAT THIS DOES. It re-walks every cell already in the region file -- the full
pick path, at the anchor, at N=10, with the wearer measured geometrically and
the furniture in the scene -- and keeps only the cells that still clear the
floor. It never ADDS cells, so it cannot widen the marking on a weaker test;
it can only remove cells that no longer hold.

CONTROLS, and there is no report without them:

    the arms are AT HOME              reach is measured from the IK seed
    a cell deep inside the wearer     must be rejected
    a cell far outboard               must be kept
    the clearance model goes NEGATIVE inside the torso
    the survivors are a SUBSET        this file must never invent a cell
"""
import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))

from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                  # noqa: E402
from measure_centre_vs_height import full_path                   # noqa: E402
from measure_grasp_approach import as_msg                        # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import master_calibration as mc                  # noqa: E402

REGION = os.path.join(ROOT, "recordings/baselines/work_surface_region.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--z", type=float, default=1.12)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--region", default=REGION)
    ap.add_argument("--out", default=None,
                    help="default: rewrite the region file in place")
    a = ap.parse_args()
    out = a.out or a.region

    d = json.load(open(a.region))
    cells = {k: [tuple(c) for c in v] for k, v in d["clear_cells"].items()}
    print("region as shipped: %s"
          % {k: len(v) for k, v in cells.items()})

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        off, j = n.home_ok(arm)
        if off > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)"
                  % (arm, off, j))
            return 3
    rig = Rig(n, "t1", 1)
    rig.pad_ee = None
    rig.pad_rot = None
    for _a in ("left", "right"):
        rig.quat[_a] = as_msg(tuple(mc.WORKSPACE_ORIENT[_a]))

    ctl = {}
    ok_in, c_in, _ = full_path(rig, "left", [0.05, 0.30, a.z], 1)
    ctl["deep_inside_the_wearer_rejected"] = "%s / %s" % (
        ok_in, None if c_in is None else round(c_in, 4))
    good_in = (not ok_in) or (c_in is not None and c_in < a.floor)
    ok_out, c_out, _ = full_path(rig, "left", [0.70, 0.15, a.z], 1)
    ctl["far_outboard_kept"] = "%s / %s" % (
        ok_out, None if c_out is None else round(c_out, 4))
    good_out = ok_out and c_out is not None and c_out >= a.floor
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22],
                            as_msg(tuple(mc.WORKSPACE_ORIENT["left"])),
                            avoid=False, tries=8)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = None if c_t is None else round(c_t, 4)
    good_t = c_t is not None and c_t < 0.0

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-34s %s" % (k, v))
    if not (good_in and good_out and good_t):
        print("\nREFUSING: a control failed.")
        return 6

    keep, drop = {}, {}
    for arm in ("left", "right"):
        keep[arm], drop[arm] = [], []
        for i, (x, y) in enumerate(cells[arm]):
            ok, c, who = full_path(rig, arm, [x, y, a.z], a.repeats)
            if ok and c is not None and c >= a.floor:
                keep[arm].append([x, y])
            else:
                drop[arm].append(dict(x=x, y=y, reachable=bool(ok),
                                      clearance_m=None if c is None
                                      else round(c, 4), to=who))
            if (i + 1) % 40 == 0:
                print("   %-5s %3d/%3d  kept %3d"
                      % (arm, i + 1, len(cells[arm]), len(keep[arm])))
        print("   %-5s KEPT %3d of %3d  (dropped %d)"
              % (arm, len(keep[arm]), len(cells[arm]), len(drop[arm])))

    # CONTROL: the survivors must be a strict subset of what came in.
    subset = all(set(map(tuple, keep[a])) <= set(cells[a])
                 for a in ("left", "right"))
    print("\n   survivors are a subset of the input: %s" % subset)
    if not subset:
        print("REFUSING: this file invented a cell.")
        return 6

    for arm in ("left", "right"):
        if keep[arm]:
            xs = [c[0] for c in keep[arm]]
            ys = [c[1] for c in keep[arm]]
            print("   %-5s x %.3f..%.3f  y %.3f..%.3f"
                  % (arm, min(xs), max(xs), min(ys), max(ys)))

    d["clear_cells"] = keep
    d["revalidated"] = dict(
        date="2026-08-16", repeats=a.repeats, z=a.z, floor=a.floor,
        orientation="WORKSPACE_ORIENT (the pinned anchor)",
        dropped={k: len(v) for k, v in drop.items()},
        dropped_cells=drop, controls=ctl,
        note=("re-walked from the 2026-08-16 home at the pinned anchor; the "
              "2026-08-15 survey solved at the live home wrist and is stale "
              "on both counts"))
    json.dump(d, open(out, "w"), indent=2)
    print("\n-> %s" % out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
