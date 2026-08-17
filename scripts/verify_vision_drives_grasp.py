#!/usr/bin/env python3
"""IS VISION DRIVING THE GRASP, OR DECORATING IT? One test settles it.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    ros2 run srl_perception mock_rgbd_camera --ros-args -p arm:=left -p task:=t1 &
    python3 scripts/verify_vision_drives_grasp.py

THE TEST. A pipeline that reads a detection and a pipeline that reads a
declaration are indistinguishable from the outside for as long as the two
agree. So make them disagree: MISLABEL a cube in the scene definition --
declare a blue cube green -- and see which pad it is placed on.

    declared path  sends it to the pad its DECLARATION names
    vision path    sends it to the pad its RENDERED COLOUR names

The mock camera renders cube colour from its own index parity, independently
of `T1_PAIR`, so flipping `T1_PAIR` changes what the task BELIEVES without
changing what the camera SEES. That is the whole experiment.

If the two paths place the mislabelled cube in the same place, vision is
decorative and the colour is still coming from the file.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, os.path.join(ROOT, "config"),
          os.path.join(ROOT, "src/srl_experiments/experiments/abc")):
    sys.path.insert(0, p)

OUT = os.path.join(ROOT, "recordings/baselines/vision_drives_grasp.json")


def place_of(wp_path, cube_index, n_cubes=4):
    """The PLACE waypoint of the nth pick, from a built path.

    Each pick contributes an equal share of the sequence, so the place point
    is the lowest waypoint in the second half of that share.
    """
    seq = wp_path["left"] if wp_path.get("left") else wp_path["right"]
    per = len(seq) // n_cubes
    chunk = seq[cube_index * per:(cube_index + 1) * per]
    half = chunk[len(chunk) // 2:]
    return min(half, key=lambda w: w[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    import rclpy
    import msc_clip_tasks as M
    from vision_grasp import observe_and_detect, DetectionUnavailable
    arm = a.arm or M.T1_ARM

    rclpy.init()
    try:
        cubes, info = observe_and_detect(arm, expect=len(M.T1_CUBES))
    except DetectionUnavailable as e:
        print("REFUSED: %s" % e)
        return 5
    finally:
        try:
            rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass

    print("SEEN: %d cubes" % len(cubes))
    for c in cubes:
        print("   x=%.4f y=%.4f -> pad %d (%s)"
              % (c[0], c[1], c[2], M.PLANE_COLOURS[c[2]]))
    print("added %.2f s total, %.2f s per pick"
          % (info["added_total_s"], info["added_per_pick_s"] or 0.0))

    # ---- MISLABEL CUBE 0 IN THE SCENE DEFINITION ---------------------
    saved = dict(M.T1_PAIR)
    truth_pad = M.T1_PAIR[0]
    lie_pad = 1 - truth_pad
    try:
        M.T1_PAIR[0] = lie_pad
        wp_declared = M.t1()                       # believes the lie
        wp_vision = M.t1(cubes=cubes)              # believes the camera
    finally:
        M.T1_PAIR.clear()
        M.T1_PAIR.update(saved)

    p_dec = place_of(wp_declared, 0)
    p_vis = place_of(wp_vision, 0)
    pads = M.T1_PLANES
    print("\nCUBE 0 declared %s (a LIE -- it renders %s)"
          % (M.PLANE_COLOURS[lie_pad], M.PLANE_COLOURS[truth_pad]))
    print("   declared path places it near x=%.3f  (pad %d, %s)"
          % (p_dec[0], lie_pad, M.PLANE_COLOURS[lie_pad]))
    print("   vision   path places it near x=%.3f  (pad %d, %s)"
          % (p_vis[0], truth_pad, M.PLANE_COLOURS[truth_pad]))
    sep = abs(p_dec[0] - p_vis[0])
    # the two pads are 200 mm apart in x; anything near that is a real move
    drives = sep > 0.10
    print("\n   the two paths differ by %.3f m in x -> %s"
          % (sep, "VISION IS DRIVING THE GRASP"
             if drives else "NO DIFFERENCE -- vision is decorative"))

    # ---- AND THE SAME TEST THROUGH THE TYPED INSTRUCTION --------------
    #
    # The path above is built by calling `t1(cubes=...)` directly. A run is
    # driven by a SENTENCE, and a sentence has its own grounding step that
    # could perfectly well have reached for T1_PAIR on its own. So the lie is
    # told again with the instruction layer in the loop: "put every cube where
    # it belongs" must send the mislabelled cube to the pad of its RENDERED
    # colour, and the plan must be byte-identical with the declaration
    # flipped and unflipped.
    import t1_instruction as TI
    instr = "put every cube where it belongs"
    o_truth = TI.plan_from(instr, cubes)
    saved = dict(M.T1_PAIR)
    try:
        for k in list(M.T1_PAIR):
            M.T1_PAIR[k] = 1 - M.T1_PAIR[k]
        o_lied = TI.plan_from(instr, cubes)
    finally:
        M.T1_PAIR.clear()
        M.T1_PAIR.update(saved)
    instr_ok = (o_truth.ok and o_lied.ok
                and o_truth.picks == o_lied.picks)
    print("\nTHE SAME LIE, THROUGH THE TYPED INSTRUCTION %r" % instr)
    print("   truthful declaration -> %s" % o_truth.message)
    print("   every colour flipped -> %s" % o_lied.message)
    for (px, py, pad) in o_lied.picks:
        print("      cube seen at (%.4f, %.4f) -> %s pad"
              % (px, py, M.PLANE_COLOURS[pad]))
    print("   -> %s"
          % ("THE INSTRUCTION IS GROUNDED ON THE CAMERA"
             if instr_ok else
             "THE PLAN MOVED WHEN THE DECLARATION MOVED -- it is reading "
             "the file"))

    res = dict(arm=arm, cubes=cubes, timing=info,
               declared_place_x=round(float(p_dec[0]), 4),
               vision_place_x=round(float(p_vis[0]), 4),
               separation_m=round(float(sep), 4),
               vision_drives_the_grasp=bool(drives),
               instruction=instr,
               instruction_plan=[list(p) for p in o_truth.picks],
               instruction_plan_with_declaration_flipped=[
                   list(p) for p in o_lied.picks],
               instruction_is_grounded_on_the_camera=bool(instr_ok))
    json.dump(res, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    if not drives:
        return 6
    return 0 if instr_ok else 7


if __name__ == "__main__":
    sys.exit(main())
