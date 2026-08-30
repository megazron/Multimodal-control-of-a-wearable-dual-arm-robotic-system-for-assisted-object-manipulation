#!/usr/bin/env python3
"""The named poses the window can send the arms to, and where they come from.

    python3 scripts/srl_named_poses.py            # list them, with the values

ONE SOURCE PER POSE, NAMED IN THE FILE IT COMES FROM. This module exists so
the GUI's buttons and every script agree on what "home" and "the pick pose"
mean, rather than each carrying its own copy. CLAUDE.md records that home was
stored in FIVE places and drifted between them; adding a button that hardcodes
an eighth copy would be repeating exactly that.

HOME is `config/home_positions_<arm>.txt` -- the SOURCE, the file the bridge
gate and the real homing target both load. It is deliberately NOT the URDF's
`initial_positions`, which is only where the sim spawns and which has drifted
from the config in the current working tree (left joint_6 by 0.976 rad). A
button that homed to the URDF would move the arm somewhere no other consumer
believes home to be.

PICK is `config/pick_pose_ideal_<arm>.txt` -- captured 2026-08-25 from the
REAL arms' own encoders after the operator hand-guided both arms to the pose
they wanted. Its header records 60 samples with 60 DISTINCT source stamps,
which matters: an earlier capture was taken from a `real_homing_node` that was
republishing a FROZEN snapshot with fresh timestamps, and it reported a pose
the arm was no longer in.

THESE ARE SIM POSES WHEN SENT TO SIM CONTROLLERS. Sim home and real home
disagree ON PURPOSE (HARD CONSTRAINT 0): the physical arms are still at the
legacy Kortex home and the bridge REFUSES to enable on the ~1.9 rad
difference rather than commanding it. A button in this window drives whatever
controllers are listening; it does not decide whether those are real.
"""
import math
import os

WS = os.environ.get("SRL_WS", "/home/gausms/kortex_ws")

# key -> (label, filename template, one-line description)
POSES = {
    "home": ("HOME",
             "config/home_positions_%s.txt",
             "The presentation pose. THE source every node loads."),
    "pick": ("PICK POSE",
             "config/pick_pose_ideal_%s.txt",
             "Captured from the real arms 2026-08-25, hand-guided by the "
             "operator."),
    # SOLVED, not captured -- and stored as JSON because it carries the
    # scores it was chosen for (surface clearance, outwardness) beside the
    # joint values. A pose whose safety margin is not stored with it is a
    # pose nobody can re-check without re-running the solver.
    "scan": ("SCAN POSE",
             "recordings/baselines/scan_pose.json",
             "Tool straight DOWN, every joint held outward from the body. "
             "Solved against the arm's collision meshes, not its joint "
             "origins."),
}


def _load_json(path, arm, key):
    """A solved pose, stored per arm with the scores it was chosen for."""
    import json
    if not os.path.exists(path):
        raise PoseError("%s does not exist -- run "
                        "`python3 scripts/solve_scan_pose.py --both --save` "
                        "to solve the %s pose first" % (path, key))
    d = json.load(open(path))
    if arm not in d or "q" not in d[arm]:
        raise PoseError("%s has no solved pose for the %s arm" % (path, arm))
    q = [float(x) for x in d[arm]["q"]]
    if len(q) != 7:
        raise PoseError("%s stores %d joints for %s, not 7"
                        % (path, len(q), arm))
    return q


class PoseError(RuntimeError):
    """Raised with the file that was missing or unreadable."""


def load(key, arm):
    """Joint 1..7 in radians for `key` on `arm`, from its one source file."""
    if key not in POSES:
        raise PoseError("unknown pose %r -- expected one of %s"
                        % (key, ", ".join(sorted(POSES))))
    tmpl = POSES[key][1]
    path = os.path.join(WS, tmpl % arm if "%s" in tmpl else tmpl)
    if path.endswith(".json"):
        return _load_json(path, arm, key)
    if not os.path.exists(path):
        raise PoseError("%s does not exist, so %s has no stored value for the "
                        "%s arm" % (path, key, arm))
    d = {}
    with open(path) as fh:
        for ln in fh:
            ln = ln.split("#", 1)[0].strip()      # values carry a kortex-deg
            if not ln.startswith("joint_"):       # comment on the same line
                continue
            k, v = ln.split(":", 1)
            try:
                d[k.strip()] = math.radians(float(v.strip()))
            except ValueError:
                continue
    missing = [i for i in range(1, 8) if "joint_%d" % i not in d]
    if missing:
        raise PoseError("%s is missing joint(s) %s -- refusing to command a "
                        "partial pose" % (path, missing))
    return [d["joint_%d" % i] for i in range(1, 8)]


def source_of(key, arm):
    tmpl = POSES[key][1]
    return os.path.join(WS, tmpl % arm if "%s" in tmpl else tmpl)


def describe(key):
    lbl, _, why = POSES[key]
    return lbl, why


def main():
    for key in sorted(POSES):
        lbl, why = describe(key)
        print("%s -- %s" % (lbl, why))
        for arm in ("left", "right"):
            try:
                q = load(key, arm)
                print("  %-5s %s" % (arm, [round(math.degrees(v), 2) for v in q]))
                print("        %s" % source_of(key, arm))
            except PoseError as e:
                print("  %-5s UNAVAILABLE: %s" % (arm, e))
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
