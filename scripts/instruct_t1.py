#!/usr/bin/env python3
"""TYPE AN INSTRUCTION AND WATCH T1 HAPPEN. One command, the whole path.

    python3 scripts/instruct_t1.py "put the green ones on the green mat"
    python3 scripts/instruct_t1.py --dry-run "sort the cubes by colour"

WHAT IT DOES, IN ORDER, and none of the steps are new -- this is the join:

    1. move to the OBSERVE pose            scripts/stage_observe_and_detect.py
    2. detect the cubes and CLASSIFY their colour from the camera
    3. return HOME, because /compute_ik seeds from the live joint state and a
       grasp planned from the observe pose sits 0.0017 m from the wearer
       against a 0.150 m floor (measured, 2026-08-16)
    4. parse the sentence                  srl_autonomy.voice_intent.parse
    5. ground it against what was SEEN     experiments/abc/t1_instruction
    6. refuse, ask, or run                 run_experiment.sh m1 --instruct

WHY THE LOOK CANNOT HAPPEN INSIDE THE RUN. `ik_follower_node` streams position
commands to the arm controller for the whole run. A look would be a second
publisher on that controller, which is this project's one-source-at-a-time rule
at the controller level -- the same rule that makes the staging move pause the
followers. So the look happens BEFORE, as staging, and writes a file.

IT REFUSES ON AMBIGUITY, AND THAT IS THE FEATURE. "pick up the blue cube" with
two blue cubes on the table does not pick one. Every way of choosing -- first,
nearest, leftmost -- is a guess, and this arm is bolted to a person.

PREREQUISITES, and it says which one is missing rather than timing out:
    a stack            python3 scripts/sim_session.py --stack teleop --keep-up -- true
    a wrist camera     ros2 run srl_perception mock_rgbd_camera \\
                           --ros-args -p arm:=left -p task:=t1
    a scene            python3 scripts/clip_scene.py --task t1 --seed 0 \\
                           --out /tmp/scene.json          (optional, for metrics)
"""
import argparse
import json
import os
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(WS, "src/srl_experiments/experiments/abc"),
          os.path.join(WS, "src/srl_autonomy"),
          os.path.join(WS, "config")):
    if p not in sys.path:
        sys.path.insert(0, p)

DETECTIONS = "/tmp/srl_t1_detections.json"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instruction")
    ap.add_argument("--mode", default="01_master_teleop")
    ap.add_argument("--dry-run", action="store_true",
                    help="look, detect and plan, then print the plan and stop "
                         "without moving the arm through the task")
    ap.add_argument("--reuse-detections", action="store_true",
                    help="skip the look and use the last one. For iterating "
                         "on phrasing; NOT for evidence, because the scene "
                         "may have changed since.")
    ap.add_argument("--participant", default="PILOT")
    a = ap.parse_args()

    # ---- 1-3. LOOK ------------------------------------------------------
    if a.reuse_detections and os.path.exists(DETECTIONS):
        print("[look] REUSING %s -- not evidence, the scene may have moved"
              % DETECTIONS)
    else:
        print("[look] moving to the observe pose and detecting...")
        r = subprocess.run(
            [sys.executable, os.path.join(WS, "scripts",
                                          "stage_observe_and_detect.py"),
             "--out", DETECTIONS])
        if r.returncode != 0:
            print("\nREFUSING: the look failed (rc=%d). Planning from the "
                  "task file instead would look exactly like a working "
                  "camera, which is the failure TASK_SPEC P-4 forbids."
                  % r.returncode)
            return 2

    vd = json.load(open(DETECTIONS))
    cubes = [tuple(c) for c in vd["cubes"]]

    # ---- 4-5. PARSE AND GROUND ------------------------------------------
    import t1_instruction as TI
    import msc_clip_tasks as MCT
    o = TI.plan_from(a.instruction, cubes)
    print("\nSEEN: %s" % TI.describe(cubes))
    for c in cubes:
        print("   (%.4f, %.4f) -> %s" % (c[0], c[1],
                                         MCT.PLANE_COLOURS[int(c[2])]))
    print("\nHEARD: %r" % a.instruction)
    if o.intent is not None:
        print("   %s" % o.intent)
    print("\n%s: %s" % (o.kind.upper(), o.message))
    if not o.ok:
        # ASK and REFUSE are both exit non-zero and both are SUCCESSES of the
        # safety property. The distinction is in the word, not the code.
        return 3 if o.kind == TI.ASK else 4
    for px, py, pad in o.picks:
        print("   pick (%.4f, %.4f) -> the %s pad"
              % (px, py, MCT.PLANE_COLOURS[pad]))

    if a.dry_run:
        print("\n--dry-run: not moving.")
        return 0

    # ---- 6. RUN ---------------------------------------------------------
    print("\n[run] %s, through %s's own command path" % ("m1", a.mode))
    r = subprocess.run(
        ["bash", os.path.join(WS, "scripts", "run_experiment.sh"), "m1",
         "--mode", a.mode, "--taskset", "msc", "--participant", a.participant,
         "--scripted", "--vision", DETECTIONS, "--instruct", a.instruction])
    return r.returncode


if __name__ == "__main__":
    raise SystemExit(main())
