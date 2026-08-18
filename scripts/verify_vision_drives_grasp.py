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


def pad_near(plan, x, y):
    """(pad index, arm, pad x) for the plan entry NEAREST a given position.

    BY POSITION, NEVER BY INDEX. The declared plan is in `T1_CUBES` order; the
    camera's is sorted by x, because that is the order `observe_and_detect`
    returns. Index 0 therefore names a different physical cube in each, and
    comparing them reported "no difference" while comparing two different
    objects -- the harness agreeing with itself about the wrong thing, which
    is the exact failure mode this script exists to catch in the system.
    """
    import t1_task as _T1
    i = min(range(len(plan)),
            key=lambda k: (plan[k][0] - x) ** 2 + (plan[k][1] - y) ** 2)
    _x, _y, pad, arm = plan[i]
    return pad, arm, _T1.T1_PLANES[pad][0]


def pad_of(plan, cube_index):
    """(pad index, arm, pad x) for one cube in a resolved plan.

    NOT SLICED OUT OF THE WAYPOINTS. The old version cut the path into equal
    shares per cube and took the lowest point of each -- correct while T1 ran
    one arm and every pick contributed the same number of waypoints. The
    rebuilt T1 is two-armed and a cube's ARM follows its colour, so the shares
    are unequal and the slice reads the wrong pick. `t1_task.plan_for()` is
    the resolution itself.
    """
    import t1_task as _T1
    x, _y, pad, arm = plan[cube_index]
    return pad, arm, _T1.T1_PLANES[pad][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    import rclpy
    import msc_clip_tasks as M
    from vision_grasp import observe_and_detect, DetectionUnavailable
    import t1_task as T1M
    arm = a.arm or T1M.LOOK_ARM

    rclpy.init()
    try:
        cubes, info = observe_and_detect(arm, expect=len(T1M.T1_CUBES))
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
    # `t1_task.build`, NOT `msc_clip_tasks.t1`: the builder moved with the
    # 2026-08-17 rebuild. The lie is still told in `T1_PAIR`, which is the
    # DECLARATION, and the camera still paints from `T1_RENDERED`, which is
    # the pixels. Those two being separate constants is what makes this test
    # possible at all -- before the rebuild the mock camera derived colour
    # from index parity, which happened to be independent, by luck.
    saved = dict(T1M.T1_PAIR)
    truth_pad = T1M.T1_PAIR[0]
    lie_pad = 1 - truth_pad
    try:
        T1M.T1_PAIR[0] = lie_pad
        plan_declared = T1M.plan_for()             # believes the lie
        plan_vision = T1M.plan_for(cubes=cubes)    # believes the camera
    finally:
        T1M.T1_PAIR.clear()
        T1M.T1_PAIR.update(saved)

    # THE SAME PHYSICAL CUBE ON BOTH SIDES, found by where it is.
    cx, cy = T1M.T1_CUBES[0]
    d_pad, d_arm, d_x = pad_near(plan_declared, cx, cy)
    v_pad, v_arm, v_x = pad_near(plan_vision, cx, cy)
    print("\nCUBE 0 declared %s (a LIE -- it renders %s)"
          % (M.PLANE_COLOURS[lie_pad], M.PLANE_COLOURS[truth_pad]))
    print("   declared path sends it to the %s pad at x=%+.3f, %s arm"
          % (M.PLANE_COLOURS[d_pad], d_x, d_arm.upper()))
    print("   vision   path sends it to the %s pad at x=%+.3f, %s arm"
          % (M.PLANE_COLOURS[v_pad], v_x, v_arm.upper()))
    sep = abs(d_x - v_x)
    # THE TWO PADS ARE ON OPPOSITE SIDES OF THE CENTRELINE, so a real
    # difference is not a shift of a few centimetres -- it is a different arm.
    drives = (d_pad != v_pad) and (d_arm != v_arm) and sep > 0.30
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
    saved = dict(T1M.T1_PAIR)
    try:
        for k in list(T1M.T1_PAIR):
            T1M.T1_PAIR[k] = 1 - T1M.T1_PAIR[k]
        o_lied = TI.plan_from(instr, cubes)
    finally:
        T1M.T1_PAIR.clear()
        T1M.T1_PAIR.update(saved)
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
               declared_pad_x=round(float(d_x), 4),
               vision_pad_x=round(float(v_x), 4),
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
