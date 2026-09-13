#!/usr/bin/env python3
"""WHICH T1 WAYPOINTS FAIL, and where are they? Per waypoint, not per task.

    python3 scripts/sim_session.py --stack moveit -- \
        python3 scripts/diagnose_t1_waypoints.py --repeats 10

WHY THIS EXISTS. `measure_pad_columns.py` reports a pad cell as reachable and
clear over the full path at N=10, and `measure_home_change.py` reports T1
losing 14 waypoints on the same layout in the same stack. Both cannot be
right, and docs/ENGINEERING_LOG.md's standing rule says a result contradicting an earlier
measurement is an INSTRUMENT CHECK until the two are reconciled.

The two ask different questions and neither says so:

  * `measure_pad_columns` walks the pick path for ONE object cell -- standoff,
    descend, lift -- and nothing else.
  * T1's real path is the whole task: four cubes, two pads, transits between
    them, and the per-cube slot offsets. A waypoint can fail in the transit
    between two cells that are each individually fine.

So this walks T1's OWN builder, waypoint by waypoint, and prints the index,
the world position and the verdict for every one that fails. That is the only
thing that says WHERE the layout is broken rather than THAT it is.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                  # noqa: E402
from measure_grasp_approach import as_msg                        # noqa: E402
import msc_clip_tasks as M                                       # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import master_calibration as mc                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_waypoint_diagnosis.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--task", default="t1", choices=["t1", "t1s2"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

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
    # THE PINNED ANCHOR, NOT THE LIVE END-EFFECTOR ORIENTATION.
    # `Rig.solve_joints` defaults to whatever quaternion the rig was built
    # with, which is read off TF. `run_abc.send()` writes
    # master_calibration.WORKSPACE_ORIENT into EVERY waypoint of every task
    # under every mode, so that is the only orientation a T1 waypoint may be
    # tested at. The first version of this file used the rig default and
    # reported 0 of 179 failures against measure_home_change's 14 -- two
    # instruments answering different questions under one name.
    ANCHOR = {a: as_msg(tuple(mc.WORKSPACE_ORIENT[a]))
              for a in ("left", "right")}

    paths = M.t1() if a.task == "t1" else M.t1_stage2(a.seed)

    # CONTROL: a waypoint driven into the torso must fail or read negative.
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22],
                            as_msg(tuple(mc.WORKSPACE_ORIENT["left"])),
                            avoid=False, tries=8)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    print("CONTROL  a pose inside the torso: %s (must be < 0)"
          % ("None" if c_t is None else "%.4f m" % c_t))
    if c_t is None or c_t >= 0.0:
        print("REFUSING: the clearance model cannot go negative.")
        return 6

    out = {}
    for arm, wps in paths.items():
        if not wps:
            continue
        bad, worst = [], (1e9, None)
        print("\n%s arm -- %d waypoints" % (arm.upper(), len(wps)))
        for i, w in enumerate(wps):
            solved, c = True, None
            for _ in range(a.repeats):
                j = n.solve_arm_joints(arm, list(w), ANCHOR[arm],
                                       avoid=True, tries=6)
                if j is None:
                    solved = False
                    break
                cc, who = rig.clearance(arm, j)
                if cc is not None and (c is None or cc < c):
                    c, kwho = cc, who
            if c is not None and c < worst[0]:
                worst = (c, i)
            if (not solved) or (c is not None and c < a.floor):
                bad.append(dict(index=i, xyz=[round(float(v), 4) for v in w],
                                solved=bool(solved),
                                clearance_m=None if c is None else round(c, 4)))
        print("   %d of %d waypoints fail" % (len(bad), len(wps)))
        for b in bad:
            print("      wp %-4d %-28s solved=%-5s clearance=%s"
                  % (b["index"], b["xyz"], b["solved"],
                     "----" if b["clearance_m"] is None
                     else "%.4f" % b["clearance_m"]))
        if bad:
            xs = sorted({round(b["xyz"][0], 3) for b in bad})
            ys = sorted({round(b["xyz"][1], 3) for b in bad})
            zs = sorted({round(b["xyz"][2], 3) for b in bad})
            print("   failing x: %s" % xs)
            print("   failing y: %s" % ys)
            print("   failing z: %s" % zs)
        out[arm] = dict(n=len(wps), failures=bad,
                        worst_clearance_m=round(worst[0], 4),
                        worst_index=worst[1])

    json.dump(dict(task=a.task, repeats=a.repeats, floor=a.floor, arms=out),
              open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
