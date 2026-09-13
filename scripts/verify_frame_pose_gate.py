#!/usr/bin/env python3
"""KNOWN-ANSWER TEST FOR THE 2026-08-17 DEPROJECTION FAULT.

    python3 scripts/verify_frame_pose_gate.py            # needs a stack + camera

WHAT WENT WRONG, AND WHY A TEST OF IT HAD TO EXIST. Inside the recording sweep
the T1 cubes deprojected 31.7 / 32.7 / 34.0 / 35.7 mm from truth against a
30 mm capture gate, so none of the four was grasped -- and the pad miss at
closure was 31.6 / 32.6 / 33.9 / 35.6 mm, the SAME numbers to 0.1 mm. The same
code standalone on the same stack returned 1.3 - 3.0 mm. Two causes, both now
fixed:

  * `ik_follower_node` streams to the controller `Vision.stage()` publishes
    to, so the arm drifted inside stage()'s 0.02 rad tolerance while the frame
    was taken. The look now PAUSES the followers.
  * the deprojection looked the camera pose up off LIVE TF afterwards, which
    answers "where is the camera now", not "where was it when this pixel was
    captured". `mock_rgbd_camera` now stamps each frame with the pose it was
    RENDERED from and `Vision.cam_pose()` uses it.

THE TEST. Displace the camera by a KNOWN amount and check the error moves by
what that displacement predicts -- which is the only way to tell a fixed
pipeline from a pipeline that happens to be accurate today.

  A. the pipeline at the observe pose            error must be small
  B. the SAME frame deprojected from a pose
     displaced by a known vector                 error must be that vector
  C. the stillness gate refuses a frame whose
     render pose disagrees with live TF          must raise, not return

B is the important one and it is a CONSTRUCTED ground truth: deprojection is
`p_cam + R_wc * ray`, so translating `p_cam` by d translates every world point
by exactly d. If the measured shift is not d, the deprojection is not using
the pose it is given -- the "every pose returns one value" row of docs/ENGINEERING_LOG.md's
instrument table.

C is what makes the failure impossible to record silently, and a check that
cannot fail is not a check, so it is exercised against a deliberately faked
render pose rather than hoped for.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, "scripts"),
          os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "config")):
    if p not in sys.path:
        sys.path.insert(0, p)

OUT = os.path.join(ROOT, "recordings/baselines/frame_pose_gate.json")
DISPLACE = np.array([0.030, -0.020, 0.010])      # 37.4 mm, all three axes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    import rclpy
    import msc_clip_tasks as MCT
    from vision_grasp import (observe_and_detect, DetectionUnavailable,
                              classify, deproject)
    from verify_colour_vision import Vision, truth, score
    arm = a.arm or MCT.T1_ARM

    res = {"arm": arm, "displacement_m": [float(v) for v in DISPLACE]}
    rclpy.init()
    n = Vision(arm)
    try:
        # ---- A. the pipeline, as it runs ------------------------------
        try:
            cubes, info = observe_and_detect(arm, node=n, expect=len(MCT.T1_CUBES),
                                             return_home=False)
        except DetectionUnavailable as e:
            print("A FAILED: %s" % e)
            return 5
        gt = truth()
        errs = []
        for c in cubes:
            d = min(math.dist((c[0], c[1]), (g["xyz"][0], g["xyz"][1]))
                    for g in gt)
            errs.append(d)
        res["A_errors_mm"] = [round(1000 * e, 2) for e in errs]
        res["A_worst_mm"] = round(1000 * max(errs), 2)
        res["frame_pose_is_stamped"] = bool(info.get("frame_pose_is_stamped"))
        res["frame_pose_drift_m"] = info.get("frame_pose_drift_m")
        res["observe_arrival_rad"] = info.get("observe_arrival_rad")
        print("A  worst deprojection error %.2f mm  (frame pose stamped: %s, "
              "camera drift during the frame %s m)"
              % (res["A_worst_mm"], res["frame_pose_is_stamped"],
                 res["frame_pose_drift_m"]))

        # ---- B. the same frame, from a displaced pose -----------------
        #
        # THE FRAME IS NOT RE-TAKEN. Re-taking it would confound the
        # displacement with whatever else changed between two frames; reusing
        # the pixels isolates the deprojection, which is the thing under test.
        img, dep = n.image(), n.depth_m()
        p_cam, R_wc = n.cam_pose()
        dets, _rej = classify(img, dep, n.info)
        base = np.array([deproject(d, n.info, p_cam, R_wc) for d in dets])
        moved = np.array([deproject(d, n.info, p_cam + DISPLACE, R_wc)
                          for d in dets])
        shift = moved - base
        worst = float(np.max(np.abs(shift - DISPLACE)))
        res["B_n_detections"] = int(len(dets))
        res["B_worst_shift_residual_m"] = round(worst, 9)
        res["B_mean_shift_m"] = [round(float(v), 6) for v in shift.mean(axis=0)]
        b_ok = len(dets) > 0 and worst < 1e-9
        print("B  %d detections shifted by %s against a commanded %s -- "
              "residual %.3g m -> %s"
              % (len(dets), res["B_mean_shift_m"], list(DISPLACE), worst,
                 "the deprojection USES the pose it is given"
                 if b_ok else "IT DOES NOT"))

        # ---- C. the stillness gate refuses a moved frame --------------
        #
        # Fake a render pose 50 mm away from where the camera actually is. The
        # gate compares the two and must raise. Without this the gate is a
        # line of code nobody has seen fire.
        # THE GATE IS CALLED DIRECTLY, not through the look.
        #
        # The first version faked `n.render_pose` and re-ran
        # `observe_and_detect`, and the gate did not fire -- because the live
        # camera subscription overwrites `render_pose` during the settle, so
        # the fake was gone before the check ran. The test was measuring the
        # subscription, not the gate. `check_frame_still` exists as its own
        # function precisely so this can be exercised.
        from vision_grasp import check_frame_still
        from geometry_msgs.msg import PoseStamped
        fake = PoseStamped()
        fake.header.frame_id = "world"
        fake.pose.orientation.w = 1.0
        saved = n.render_pose
        n.render_pose = fake
        c_ok, c_msg = False, "no refusal"
        try:
            check_frame_still(n, np.asarray(p_cam) + np.array([0.05, 0.0, 0.0]))
        except DetectionUnavailable as e:
            c_ok, c_msg = ("moved" in str(e) and "still" in str(e)), str(e)
        finally:
            n.render_pose = saved
        # AND THE CONTROL: the SAME gate must PASS on a frame that did not move,
        # or "it refuses" would just mean "it always refuses".
        c_pass = True
        try:
            n.render_pose = fake
            check_frame_still(n, np.asarray(p_cam))
        except DetectionUnavailable as e:
            c_pass, c_msg = False, "refused a STILL frame: %s" % e
        finally:
            n.render_pose = saved
        res["C_accepts_a_still_frame"] = bool(c_pass)
        c_ok = c_ok and c_pass
        res["C_refused"] = bool(c_ok)
        res["C_message"] = c_msg[:200]
        print("C  a frame stamped 50 mm from the live camera -> %s"
              % ("REFUSED, and says why" if c_ok
                 else "NOT REFUSED (%s)" % c_msg[:80]))
    finally:
        try:
            n.destroy_node()
            rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass

    a_ok = res["A_worst_mm"] < 10.0
    res["pass"] = bool(a_ok and b_ok and c_ok)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=1)
    print("\n-> %s" % a.out)
    print("PASS" if res["pass"] else "FAIL")
    return 0 if res["pass"] else 6


if __name__ == "__main__":
    raise SystemExit(main())
