#!/usr/bin/env python3
"""WHERE THE FINGER PADS ARE, RELATIVE TO THE WRIST, READ FROM FK.

    python3 scripts/measure_pad_mid_ee.py

WHY THIS EXISTS. `grasp_frames.PAD_MID_EE` is the vector every T1 grasp is
built on: the task declares where the OBJECT is and subtracts this to get the
wrist pose. It was not measured -- it was DERIVED, as the magnitude of
`clip_tasks.PAD_OFFSET_BY_ARM`, a world-frame vector recorded at the anchor
long before T1 carried its own approach.

MEASURED, 2026-08-18, on both arms, with `/compute_fk` on the two finger-tip
links and the end-effector link from the SAME solution:

    end_effector_link -> midpoint of the finger tips   (0, 0, 0.09833) m
    |PAD_OFFSET_BY_ARM|                                       0.11178 m
    disagreement                                               13.45 mm

The two arms agree to 0.01 mm, which is what says it is hardware and not a
posture: rotate each arm's own measurement into its own end-effector frame and
they land on the same vector, purely along the tool axis.

WHICH ONE IS RIGHT IS NOT AN ARGUMENT. `verify_t1.py` solves the grasp pose the
task commands and then asks FK where the finger tips ended up. With 0.11178 the
pads sit 13.48 mm from the cube centre on all four cubes of both arms; with the
measured 0.09833 they sit on it. The check that reads "pad miss must be 0 by
construction" is the whole point of the constant.

WHAT THIS DOES NOT CHANGE. `PAD_OFFSET_BY_ARM` is left exactly as it is. T0, T2
and T3 declare their coordinates through `clip_tasks.ee_for`, and `clip_scene`
DRAWS their objects through the same offset, so task and picture agree with each
other; moving it would move every coordinate in those three tasks and every
scene that draws them, which is a re-derivation and not a fix. What it means is
that in those tasks the declared coordinate is 13.45 mm from where the object
is drawn and grasped, and that is now written down instead of unknown.
"""
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

from verify_task_scenes import Solver                          # noqa: E402
from measure_grasp_approach import FK, link_names, q_matrix    # noqa: E402
import clip_tasks as CT                                        # noqa: E402
import grasp_frames as GF                                      # noqa: E402
from srl_teleop import master_calibration as MC                # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/pad_mid_ee.json")


def main():
    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    fk = FK(n)

    res = {"arms": {}}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        p = fk.poses(arm, home, link_names(arm))
        ee_p, ee_q = p["%s_end_effector_link" % arm]
        tips = [p["%s_robotiq_85_%s_finger_tip_link" % (arm, s)][0]
                for s in ("left", "right")]
        mid = (tips[0] + tips[1]) / 2.0
        world = mid - ee_p
        in_ee = q_matrix(ee_q).T @ world
        stored = np.asarray(CT.PAD_OFFSET_BY_ARM[arm], float)
        res["arms"][arm] = dict(
            pad_mid_in_ee=[round(float(v), 5) for v in in_ee],
            along_tool_axis_m=round(float(in_ee[2]), 5),
            off_axis_mm=round(float(np.linalg.norm(in_ee[:2])) * 1000.0, 3),
            ee_to_one_tip_m=round(float(np.linalg.norm(tips[0] - ee_p)), 5),
            tip_separation_m=round(float(np.linalg.norm(tips[0] - tips[1])), 4),
            stored_world_offset=[round(float(v), 4) for v in stored],
            stored_magnitude_m=round(float(np.linalg.norm(stored)), 5),
            disagreement_mm=round(
                (float(np.linalg.norm(stored)) - float(in_ee[2])) * 1000.0, 2),
            # The stored world vector rotated back through the arm's own
            # ANCHOR -- which is the frame it was recorded in. If the two
            # disagree only in LENGTH, this comes back along +z as well.
            stored_in_ee_at_anchor=[
                round(float(v), 5)
                for v in q_matrix(MC.WORKSPACE_ORIENT[arm]).T @ stored])
        r = res["arms"][arm]
        print("%-5s pad midpoint in the EE frame  %s" % (arm, r["pad_mid_in_ee"]))
        print("      along the tool axis          %.5f m" % r["along_tool_axis_m"])
        print("      off the axis                 %.3f mm" % r["off_axis_mm"])
        print("      stored PAD_OFFSET_BY_ARM     %.5f m long -> %+.2f mm out"
              % (r["stored_magnitude_m"], r["disagreement_mm"]))

    l = np.asarray(res["arms"]["left"]["pad_mid_in_ee"], float)
    rr = np.asarray(res["arms"]["right"]["pad_mid_in_ee"], float)
    gap_mm = float(np.linalg.norm(l - rr)) * 1000.0
    res["both_arms_agree_mm"] = round(gap_mm, 4)
    res["shipped_constant"] = list(GF.PAD_MID_EE)
    res["shipped_matches_measurement_mm"] = round(
        abs(float(GF.PAD_MID_EE[2]) - float(l[2])) * 1000.0, 3)
    print("\nthe two arms agree to %.3f mm, so it is hardware" % gap_mm)
    print("grasp_frames.PAD_MID_EE = %s -> %.3f mm from the measurement"
          % (list(GF.PAD_MID_EE), res["shipped_matches_measurement_mm"]))

    # A CONTROL, because a script that only prints one number cannot be wrong
    # out loud. Rebuild the pad midpoint in WORLD from the constant and the
    # measured EE pose, and compare with the FK midpoint it came from. It is
    # arithmetic, so the ground truth is constructed.
    arm = "left"
    home = [n.js.get(k, 0.0) for k in n.names(arm)]
    p = fk.poses(arm, home, link_names(arm))
    ee_p, ee_q = p["%s_end_effector_link" % arm]
    tips = [p["%s_robotiq_85_%s_finger_tip_link" % (arm, s)][0]
            for s in ("left", "right")]
    mid = (tips[0] + tips[1]) / 2.0
    rebuilt = ee_p + q_matrix(ee_q) @ np.asarray(
        res["arms"][arm]["pad_mid_in_ee"], float)
    err_mm = float(np.linalg.norm(rebuilt - mid)) * 1000.0
    res["round_trip_mm"] = round(err_mm, 4)
    ok = err_mm < 0.05
    print("control: rebuilding the world midpoint from the constant is "
          "%.4f mm out -- %s" % (err_mm, "OK" if ok else "FAILED"))
    if not ok:
        print("REFUSING to write a baseline the round trip does not support.")
        return 3

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("-> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
