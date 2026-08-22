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

=============================================================================
2026-08-23: BOTH NUMBERS ABOVE ARE RIGHT, AND THE COMPARISON BETWEEN THEM WAS
NOT. THE PAD MIDPOINT IS NOT A CONSTANT.
=============================================================================
The Robotiq 85 is a FOUR-BAR LINKAGE. Its fingers swing; they do not
translate. So the distance from the wrist to the midpoint of the finger tips
is a function of HOW OPEN THE HAND IS, and every number above was measured at
one unstated opening. Measured here through the URDF's own mimic chain
(`--by-width`):

    knuckle 0.0000 rad, span 135.5 mm (WIDE OPEN)   0.09833 m
    knuckle 0.4235 rad, span  93.3 mm (a 40 mm cube) 0.10976 m
    knuckle 0.6118 rad, span  ~52 mm  (a 20 mm object) 0.11179 m

The 0.09833 that replaced the 0.11178 in 2026-08-18 is the WIDE OPEN hand.
The 0.11178 it replaced is the hand almost shut. Neither is where the pads are
when they close on T1's 40 mm cube, which is 0.10976 -- **11.43 mm** beyond
the constant every T1 grasp has been built on since.

AND IT EXPLAINS WHY THE CHECK FLIPPED. `verify_t1.py` reads the finger tips
out of FK at whatever opening the simulation's gripper happens to be left at.
On 2026-08-18 that was the open hand and the pad miss read 0.00 mm; on
2026-08-23 it was a nearly shut hand and the same check on the same geometry
read 13.52 mm. A measurement whose answer depends on leftover state is not a
measurement, and this one had been quoted as the evidence for the constant.

So the quantity is `pad_mid_ee_for(width_mm)`, a table measured through the
linkage, and the opening has to be stated with the number. `--by-width` writes
it; `grasp_frames.pad_mid_ee_for` reads it; the fully-open row reproduces the
0.09833 above, which is the control that says the two instruments agree.
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


BY_WIDTH_OUT = os.path.join(ROOT,
                            "recordings/baselines/pad_mid_ee_by_width.json")

# The widths worth tabulating: every object this rig is asked to grasp, plus
# the ends of the range so an interpolation is never an extrapolation.
WIDTHS_MM = (0, 10, 20, 30, 40, 50, 60, 70, 80, 85)


def measure_by_width(verbose=True):
    """The pad midpoint at each gripper opening, from the URDF's own linkage.

    OFFLINE. `scripts/srl_fk.py` walks the URDF, including the Robotiq's mimic
    joints, so this needs no stack and cannot be perturbed by whatever opening
    a running simulation was left at -- which is precisely the fault it exists
    to remove.
    """
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
    from srl_fk import FK as OfflineFK, compiled_matches_urdf
    from srl_teleop import gripper_state as GS
    import numpy as _np

    # THE INSTRUMENT FIRST. srl_fk's own control, before any of its answers
    # are written to a baseline.
    if not compiled_matches_urdf(verbose=False):
        raise SystemExit("REFUSING: srl_fk disagrees with itself; see its "
                         "own control. No table written.")
    fk = OfflineFK()
    rows, ctl = [], {}
    for arm in ("left", "right"):
        per = []
        for w in WIDTHS_MM:
            kn = 0.0 if w >= 85 else float(GS.grip_for(w))
            kn = float(GS.grip_for(w)) if w > 0 else 0.0
            P = fk.poses(arm, [0.0] * 7,
                         ["end_effector_link",
                          "robotiq_85_left_finger_tip_link",
                          "robotiq_85_right_finger_tip_link"], gripper=kn)
            ee, a, b = P[0][:3, 3], P[1][:3, 3], P[2][:3, 3]
            R = P[0][:3, :3]
            mid_ee = R.T @ ((a + b) / 2.0 - ee)
            per.append(dict(width_mm=w, knuckle_rad=round(kn, 6),
                            span_mm=round(float(_np.linalg.norm(a - b)) * 1000, 2),
                            pad_mid_ee=[round(float(v), 6) for v in mid_ee],
                            along_axis_m=round(float(mid_ee[2]), 6),
                            off_axis_mm=round(float(_np.linalg.norm(mid_ee[:2]))
                                              * 1000, 4)))
        rows.append((arm, per))
    left = {r["width_mm"]: r for r in rows[0][1]}
    right = {r["width_mm"]: r for r in rows[1][1]}
    # CONTROLS, and the report is refused if one fails.
    ctl["open_hand_reproduces_PAD_MID_EE"] = (
        abs(left[0]["along_axis_m"] - 0.09833) < 1e-4)
    ctl["the_two_arms_agree"] = all(
        abs(left[w]["along_axis_m"] - right[w]["along_axis_m"]) < 1e-6
        for w in WIDTHS_MM)
    ctl["the_pad_stays_on_the_tool_axis"] = all(
        r["off_axis_mm"] < 0.2 for r in rows[0][1])
    # IT MUST MOVE, or there is nothing to tabulate.
    ctl["the_opening_changes_it"] = (
        abs(left[40]["along_axis_m"] - left[0]["along_axis_m"]) > 0.005)
    res = dict(controls=ctl, widths_mm=list(WIDTHS_MM),
               arms={"left": rows[0][1], "right": rows[1][1]},
               note=("pad midpoint in the END EFFECTOR frame at the gripper "
                     "opening grip_for(width). Measured offline from the URDF "
                     "through the Robotiq's mimic chain; needs no stack, so "
                     "it cannot be perturbed by leftover gripper state."))
    if verbose:
        print("PAD MIDPOINT vs GRIPPER OPENING (left arm, from the URDF)")
        print("  %6s %10s %9s %12s" % ("object", "knuckle", "span", "along axis"))
        for r in rows[0][1]:
            print("  %4d mm %8.4f %7.1f mm %10.5f m"
                  % (r["width_mm"], r["knuckle_rad"], r["span_mm"],
                     r["along_axis_m"]))
        print("\nCONTROLS")
        for k, v in ctl.items():
            print("  %-38s %s" % (k, "OK" if v else "*** FAILED ***"))
    if not all(ctl.values()):
        raise SystemExit("REFUSING to write the table: a control failed.")
    os.makedirs(os.path.dirname(BY_WIDTH_OUT), exist_ok=True)
    json.dump(res, open(BY_WIDTH_OUT, "w"), indent=2, sort_keys=True)
    if verbose:
        print("\n-> %s" % BY_WIDTH_OUT)
    return res


def main():
    if "--by-width" in sys.argv:
        measure_by_width()
        return 0
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
