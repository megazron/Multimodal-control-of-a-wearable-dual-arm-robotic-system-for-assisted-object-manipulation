#!/usr/bin/env python3
"""SHOW ME THE POSE. One still per view, so a posture can be LOOKED at.

    python3 scripts/render_pose.py --pose presentation --out /tmp/pp
    python3 scripts/render_pose.py --pose home --out /tmp/home

Stages a pose and grabs one frame from each capture display. It exists because
a staging pose is a thing a person looks at, and every check this repository
can run on one -- reachable, clear of the wearer, wrist level -- was PASSING
on a pose whose arms were 1.633 m apart in a T. The numbers were right and the
posture was wrong, which is the same lesson the clip inspection keeps
teaching: a check answers the question it was given.

It reuses `record_rviz`'s displays rather than standing up its own, so the
still is framed exactly as the clip will be. Anything that looks right here
and wrong in the clip is a framing difference, not a rendering one.
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import record_rviz as rr                                       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="presentation",
                    choices=["presentation", "home"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--task", default="t1",
                    help="which scene to frame it in")
    ap.add_argument("--views", default="front,iso,left",
                    help="comma separated; must be keys of record_rviz.VIEWS")
    ap.add_argument("--settle-s", type=float, default=4.0)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    rr.ensure_display(os.path.join(a.out, "rviz"), gripper_arm="right",
                      task=a.task)
    time.sleep(3.0)

    cmd = [sys.executable, os.path.join(HERE, "stage_presentation_pose.py")]
    if a.pose == "home":
        cmd.append("--home")
    r = subprocess.run(cmd, capture_output=True, text=True)
    print((r.stdout or r.stderr).strip())
    if r.returncode != 0:
        # A STILL OF A HALF-FINISHED MOVE IS WORSE THAN NO STILL, because it
        # looks like a pose and is not one.
        print("REFUSING to render: the arms did not reach the %s pose "
              "(rc=%d)." % (a.pose, r.returncode))
        return 1
    time.sleep(a.settle_s)

    made = []
    for name in a.views.split(","):
        name = name.strip()
        if name not in rr.VIEWS:
            print("no such view %r; have %s" % (name, sorted(rr.VIEWS)))
            continue
        disp = rr.VIEWS[name][0]
        png = os.path.join(a.out, "%s_%s.png" % (a.pose, name))
        g = subprocess.run(
            [rr.FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", "800x500", "-i", disp, "-frames:v", "1", png],
            capture_output=True, text=True)
        if g.returncode == 0 and os.path.exists(png):
            made.append(png)
            print("   %-6s -> %s" % (name, png))
        else:
            print("   %-6s FAILED: %s" % (name, g.stderr.strip()[:100]))
    print("\n%d still(s). LOOK AT THEM." % len(made))
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
