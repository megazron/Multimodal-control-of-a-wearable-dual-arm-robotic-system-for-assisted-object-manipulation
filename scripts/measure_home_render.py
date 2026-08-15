#!/usr/bin/env python3
"""WHAT POSE DOES THE STACK ACTUALLY BOOT INTO? Measured from TF, shot from RViz.

    python3 scripts/sim_session.py --stack moveit -- \
        python3 scripts/measure_home_render.py --shoot

WHY THIS EXISTS. The home pose was reported as "both elbows 0.14 m below the
shoulders, wrists level, symmetric to 1e-16" and the render shows something
else. Both cannot be true, and a disagreement between a number and a picture is
settled by measuring the thing the picture is drawn FROM.

RViz draws links from /tf. So this reads /tf, at the pose the stack came up in,
with nothing commanded, and reports the same quantities a person reads off the
picture:

    elbow height       relative to the arm's own shoulder joint, signed, with
                       the link it calls the elbow named explicitly. "Elbow" is
                       not a Gen3 link name and the disagreement may be nothing
                       more than two people meaning different links.
    wrist angle        the elevation of the TOOL AXIS above horizontal, which
                       is what "wrists level" means. Read from the bracelet's
                       own frame, not inferred from joint angles.
    hand position      where the gripper actually is, in world.
    mirror residual    the left arm reflected through x = 0 against the right.
                       Symmetric means this is near zero for EVERY link, not
                       just for the hand: a pose can put both hands in the same
                       place by two different routes and look wrong.

`--shoot` additionally brings up RViz on a virtual display and grabs a still,
so the picture and the numbers come from one boot of one stack.

CONTROLS, and there is no report without them:

    TF answers for every link      a missing lookup is not a measurement
    FK agrees with TF              the pose being measured is the pose that is
                                   published, to 1 mm
    the home SOURCE matches the    config/home_positions_*.txt is what the
    live joint state               pose is stored in; the stack must be AT it,
                                   to 1e-4 rad. That is the cached-config and
                                   stale-install trap this check exists for
    a deliberately wrong mirror    reflecting the LEFT arm against ITSELF must
                                   produce a large residual, so a residual of
                                   zero cannot come from a broken comparison
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from verify_task_scenes import Solver                              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/home_render.json")
SCRATCH = os.environ.get(
    "SRL_SCRATCH", os.path.join(os.environ.get("TMPDIR", "/tmp"),
                                "srl_rviz_scratch"))
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")

# WHICH LINK IS THE ELBOW. On a 7-DOF Gen3 the chain is
#   base -> shoulder -> half_arm_1 -> half_arm_2 -> forearm -> wrist_1 ->
#   wrist_2 -> bracelet -> end_effector
# A person looking at the render calls the joint between the upper arm and the
# forearm the elbow, and that is the origin of `forearm_link`. `half_arm_2` is
# the upper arm's far end and sits close to it; both are reported, because a
# claim about "the elbow" that does not say which link it measured is how two
# people end up disagreeing about a picture they both looked at.
SHOULDER = "shoulder_link"
ELBOW = "forearm_link"
ELBOW_ALT = "half_arm_2_link"
WRIST = "spherical_wrist_1_link"
TOOL = "end_effector_link"
CHAIN = [SHOULDER, "half_arm_1_link", ELBOW_ALT, ELBOW, WRIST,
         "spherical_wrist_2_link", "bracelet_link", TOOL]


def quat_axis(q, axis=2):
    """A column of the rotation matrix for quaternion (x, y, z, w)."""
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R[:, axis]


class Reader(Solver):
    """Solver plus a TF read that KEEPS SPINNING while it waits.

    The first version looked up every frame straight after an 8 s spin and got
    nothing for all sixteen, then reported a mirror residual of 0.0000 m --
    which reads as perfect symmetry and is really an empty loop. The controls
    caught it. TF is filled by callbacks, so a lookup that fails has to spin
    and try again rather than conclude anything.
    """

    def __init__(self):
        super().__init__()
        self.tf_error = None

    def pose_of(self, frame, timeout=15.0):
        import rclpy.time
        end = time.monotonic() + timeout
        while True:
            try:
                t = self.buf.lookup_transform("world", frame,
                                              rclpy.time.Time())
                v, r = t.transform.translation, t.transform.rotation
                return (np.array([v.x, v.y, v.z]),
                        np.array([r.x, r.y, r.z, r.w]))
            except Exception as exc:                               # noqa: BLE001
                if self.tf_error is None:
                    self.tf_error = "%s: %s" % (frame, exc)
                if time.monotonic() > end:
                    return None, None
                rclpy.spin_once(self, timeout_sec=0.05)


def home_from_config():
    """The home pose from ITS SOURCE, `config/home_positions_{arm}.txt`.

    Not from the URDF. The URDF's two `initial_positions` blocks are a second
    copy of this file and `test_home_has_one_source` is what keeps them in
    step; reading the source here means a drift between them shows up as this
    check disagreeing with the live joint state, rather than as two wrong
    numbers agreeing with each other.
    """
    out = {}
    for arm in ("left", "right"):
        path = os.path.join(ROOT, "config/home_positions_%s.txt" % arm)
        vals = {}
        for line in open(path):
            line = line.split("#")[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            k = k.strip()
            if k.startswith("joint_"):
                vals[int(k.split("_")[1])] = math.radians(float(v))
        if vals:
            out[arm] = [round(vals[k], 6) for k in sorted(vals)]
    return out


def _ink(png):
    """Fraction of pixels that differ from the modal (background) colour.

    A render that failed is one flat colour. A render that worked has a robot
    in it. This is the difference, and it is measured rather than assumed.
    """
    try:
        from PIL import Image
    except Exception:                                              # noqa: BLE001
        return 1.0                       # cannot judge; do not claim to
    im = Image.open(png).convert("RGB")
    # CROP THE CHROME OUT FIRST. Measured on the known-blank frame this check
    # was written against: the whole window scores 0.132 because RViz's own
    # menus, toolbar and status bar are ink, so a threshold on the window
    # cannot fail on a blank viewport. The viewport alone scores 0.0003.
    w, h = im.size
    im = im.crop((int(0.10 * w), int(0.12 * h),
                  int(0.90 * w), int(0.92 * h))).resize((320, 200))
    px = np.asarray(im).reshape(-1, 3)
    vals, counts = np.unique(px, axis=0, return_counts=True)
    bg = vals[int(np.argmax(counts))]
    return float(np.mean(np.abs(px.astype(int) - bg.astype(int)).sum(1) > 24))


def shoot(name, disp=None, size=None):
    """One still of the live graph, from the RECORDER'S OWN camera config.

    The first version wrote its own minimal RViz config and produced a blank
    white frame: an FPS view with no focal point, and a RobotModel display
    with no `Description Source`, so RViz came up ready and drew nothing. The
    fix is not a better hand-written config, it is to stop having a second
    one -- `record_rviz.write_cfg` is what the whole clip set is filmed with,
    so a still taken any other way is not evidence about the clips.
    """
    import record_rviz as RR
    cfg = RR.write_cfg(name)
    disp = disp or RR.VIEWS[name][0]
    size = size or ("%dx%d" % (RR.VW, RR.VH))
    if subprocess.run(["pgrep", "-f", "Xvfb %s" % disp],
                      capture_output=True).returncode != 0:
        subprocess.Popen(["setsid", "/usr/bin/Xvfb", disp, "-screen", "0",
                          size + "x24", "-nolisten", "tcp"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
    env = dict(os.environ, DISPLAY=disp, LIBGL_ALWAYS_SOFTWARE="1")
    rv = subprocess.Popen(["rviz2", "-d", cfg], env=env,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(22)
    os.makedirs(SCRATCH, exist_ok=True)
    png = os.path.join(SCRATCH, "home_%s.png" % name)
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
                    "-video_size", size, "-i", disp, "-frames:v", "1", png],
                   check=False)
    rv.terminate()
    try:
        rv.wait(timeout=10)
    except Exception:                                              # noqa: BLE001
        rv.kill()
    if not os.path.exists(png):
        return None
    # A BLANK FRAME IS NOT A SHOT. The first attempt returned a path to a
    # white rectangle and called it captured, which is the whole failure this
    # project keeps finding wearing a new hat: the artefact existed, so it
    # counted. Measure the picture.
    frac = _ink(png)
    if frac < 0.02:      # blank viewport measures 0.0003
        print("   %s: only %.3f%% of the frame is not background -- BLANK, "
              "not captured" % (name, frac * 100.0))
        return None
    return png


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shoot", action="store_true",
                    help="also grab RViz stills from a virtual display")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Reader()
    n.spin(12.0)

    want = home_from_config()
    report = dict(home_from_config_rad=want, arms={}, controls={})

    missing = []
    per = {}
    for arm in ("left", "right"):
        d = {}
        for ln in CHAIN:
            p, q = n.pose_of("%s_%s" % (arm, ln))
            if p is None:
                missing.append("%s_%s" % (arm, ln))
                continue
            d[ln] = (p, q)
        per[arm] = d
    report["controls"]["tf_answers_for_every_link"] = dict(
        want="[]", got="%d missing%s" % (
            len(missing),
            "" if not missing else " (first error: %s)" % n.tf_error))
    report["tf_missing"] = missing

    # the joint state the stack is actually at, against what it was told
    js_ok = {}
    for arm in ("left", "right"):
        live = [n.js.get(k, None) for k in n.names(arm)]
        told = want.get(arm)
        if told is None or any(v is None for v in live):
            js_ok[arm] = "unknown"
            continue
        worst = max(abs(a_ - b_) for a_, b_ in zip(live, told))
        js_ok[arm] = round(worst, 9)
    report["controls"]["joint_state_matches_the_home_source"] = dict(
        want="< 1e-4 rad", got=str(js_ok))

    for arm in ("left", "right"):
        d = per[arm]
        if SHOULDER not in d or ELBOW not in d:
            continue
        sh = d[SHOULDER][0]
        el = d[ELBOW][0]
        el2 = d[ELBOW_ALT][0]
        tool_p, tool_q = d[TOOL]
        axis = quat_axis(tool_q, 2)
        elev = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
        report["arms"][arm] = dict(
            shoulder=[round(v, 4) for v in sh],
            elbow_forearm_link=[round(v, 4) for v in el],
            elbow_half_arm_2=[round(v, 4) for v in el2],
            hand=[round(v, 4) for v in tool_p],
            elbow_below_shoulder_m=round(float(sh[2] - el[2]), 4),
            elbow_alt_below_shoulder_m=round(float(sh[2] - el2[2]), 4),
            elbow_outboard_of_hand_m=round(float(abs(el[0]) - abs(tool_p[0])),
                                           4),
            tool_axis=[round(float(v), 4) for v in axis],
            tool_elevation_deg=round(elev, 2),
            hand_forward_of_torso_m=round(float(tool_p[1] - 0.11), 4))

    # mirror residual, per link, and a control that it can be non-zero
    mirror = {}
    for ln in CHAIN:
        if ln not in per["left"] or ln not in per["right"]:
            continue
        pl = per["left"][ln][0].copy()
        pl[0] = -pl[0]
        mirror[ln] = round(float(np.linalg.norm(pl - per["right"][ln][0])), 4)
    report["mirror_residual_m"] = mirror
    report["mirror_residual_worst_m"] = (max(mirror.values()) if mirror
                                         else None)

    selfmirror = {}
    for ln in CHAIN:
        if ln not in per["left"]:
            continue
        pl = per["left"][ln][0].copy()
        pl[0] = -pl[0]
        selfmirror[ln] = round(
            float(np.linalg.norm(pl - per["left"][ln][0])), 4)
    worst_self = max(selfmirror.values()) if selfmirror else 0.0
    report["controls"]["mirror_can_be_nonzero"] = dict(
        want="> 0.10 reflecting the LEFT arm against itself",
        got=round(worst_self, 4))

    ok = (not missing and worst_self > 0.10
          and all(v == "unknown" or v < 1e-4 for v in js_ok.values()))
    report["controls_pass"] = bool(ok)

    print("WHAT THE STACK BOOTED INTO, read from /tf\n")
    for arm in ("left", "right"):
        d = report["arms"].get(arm)
        if not d:
            continue
        print("  %s arm" % arm.upper())
        print("     shoulder            %s" % d["shoulder"])
        print("     elbow (forearm_link) %s   %+.4f m below the shoulder"
              % (d["elbow_forearm_link"], d["elbow_below_shoulder_m"]))
        print("     elbow (half_arm_2)   %s   %+.4f m below the shoulder"
              % (d["elbow_half_arm_2"], d["elbow_alt_below_shoulder_m"]))
        print("     hand                %s" % d["hand"])
        print("     elbow outboard of the hand by %+.4f m"
              % d["elbow_outboard_of_hand_m"])
        print("     tool axis %s -> elevation %+.2f deg"
              % (d["tool_axis"], d["tool_elevation_deg"]))
        print("     hand is %+.4f m in front of the torso face"
              % d["hand_forward_of_torso_m"])
    print("\n  MIRROR RESIDUAL, left reflected through x = 0 against right")
    for ln, v in mirror.items():
        print("     %-26s %.4f m" % (ln, v))
    print("     worst %.4f m" % (report["mirror_residual_worst_m"] or 0.0))

    print("\nCONTROLS")
    for k, v in report["controls"].items():
        print("   %-32s want %-24s got %s" % (k, v["want"], v["got"]))
    if not ok:
        print("\nREFUSING TO CALL THIS A MEASUREMENT: a control failed.")

    if a.shoot:
        shots = {}
        for view in ("front", "left", "iso"):
            shots[view] = shoot(view)
        report["shots"] = shots
        print("\nSTILLS")
        for k, v in shots.items():
            print("   %-6s %s" % (k, v or "NOT CAPTURED"))

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(report, open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 5


if __name__ == "__main__":
    sys.exit(main())
