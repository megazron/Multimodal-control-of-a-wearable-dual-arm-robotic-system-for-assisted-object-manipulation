#!/usr/bin/env python3
"""THE TOP-DOWN PLAN VIEW: both pads in front of the wearer, both arms on them.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    python3 scripts/render_centre_plan.py

WHAT IT SHOOTS AND WHY IT IS NOT THE `top` VIEW. `record_rviz.VIEWS["top"]` is
the clip set's over-the-shoulder view, pitched 1.35 rad and focused on the
work band; it reads as a high three-quarter, not a plan. The question this
picture has to answer -- are the pads between the two arms and in front of the
person, rather than off at one end -- is a question about x, and only a true
plan view lets anyone measure x off the frame.

BOTH ARMS ARE POSED ON THEIR PADS, not left at home. `pose_arms_at_pads.py`
solves each arm's own pad slot and REFUSES if either does not solve or sits
inside the 150 mm floor, so a frame that exists at all is a frame in which
both arms are demonstrably at the pads.

THE CHECKS, and no image is filed without them, both inherited from
`render_t1_scene` for the reason recorded there: a blank frame once passed as a
render at 18% ink because the WEARER is 18% of the frame.

    the task's own marker namespaces must be present on /task_objects
    the frame must carry more than 2% ink against the modal background
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

import record_rviz as RR                                          # noqa: E402
import render_t1_scene as RT                                      # noqa: E402
import t1_task as T1                                              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="t1")
    ap.add_argument("--dist", type=float, default=1.75)
    ap.add_argument("--slot", default="inner")
    ap.add_argument("--out", default=os.path.join(
        os.environ.get("SRL_SCRATCH", "/tmp"), "centre_plan"))
    a = ap.parse_args()

    # A TRUE PLAN VIEW, focused between the wearer and the row the pads sit
    # on, so the centreline of the person and the centreline of the table are
    # the same vertical line in the frame.
    focus = (0.0, round(T1.TABLE_NEAR_Y + 0.10, 3), T1.TABLE_TOP)
    RR.VIEWS["plan"] = (":97", 1.5708, 1.5620, a.dist, focus, False)

    src = ("set +u; source /opt/ros/jazzy/setup.bash; "
           "source %s/install/setup.bash; set -u; "
           "export FASTDDS_BUILTIN_TRANSPORTS=SHM" % ROOT)
    scene = subprocess.Popen(
        ["bash", "-lc", "%s && exec python3 %s/scripts/clip_scene.py "
         "--task %s --seed 0" % (src, ROOT, a.task)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    pose = None
    try:
        print("clip_scene up for task %s; settling" % a.task)
        time.sleep(12)
        got = RT._scene_namespaces(timeout_s=20.0)
        want = RT._expected_namespaces(a.task)
        print("   /task_objects namespaces: %s" % sorted(got))
        missing = sorted(want - got)
        if missing:
            print("REFUSING: the scene is not publishing %s" % missing)
            return 3
        # POSE THE ARMS, and let it refuse for both of us.
        pose = subprocess.Popen(
            ["bash", "-lc", "%s && exec python3 %s/scripts/pose_arms_at_pads.py "
             "--hold 240 --slot %s" % (src, ROOT, a.slot)],
            preexec_fn=os.setsid)
        time.sleep(20)
        if pose.poll() is not None and pose.returncode != 0:
            print("REFUSING: the arms could not be posed at the pads "
                  "(exit %d)" % pose.returncode)
            return 4
        png, frac = RT.shoot("plan", a.task, a.out, settle=24)
        print("   plan  %-6s ink %.3f%%  %s"
              % ("OK" if png else "BLANK", frac * 100.0, png or "-"))
        return 0 if png else 5
    finally:
        for p in (pose, scene):
            if p is None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:                                     # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
