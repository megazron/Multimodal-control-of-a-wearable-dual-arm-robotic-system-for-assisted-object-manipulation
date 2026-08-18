#!/usr/bin/env python3
"""PUT A MOUNT CANDIDATE INTO THE URDF, OR TAKE IT BACK OUT.

    python3 scripts/apply_mount_candidate.py --show
    python3 scripts/apply_mount_candidate.py --out 0.15 --yaw-out 15
    python3 scripts/apply_mount_candidate.py --restore

WHY A SCRIPT AND NOT AN EDIT. `sweep_mount_geometry.py` and
`probe_centre_limit.py` evaluate a mount candidate by transforming the TARGET
instead of moving the arm, which is exact for kinematics and blind to MoveIt's
mesh collision, to self-collision and to the gripper against the table. Nothing
from those files may be believed until the URDF really carries the change and
the stack has been relaunched on it. This is the one place that performs the
translation from "a transform in a sweep" to "a mount in the robot", so the two
cannot drift.

THE CONVENTION, stated once. `--out` moves BOTH mounts away from the
centreline, mirrored, in metres; the mount origin is `mount_x` in the backpack
frame, which is axis-aligned with world, so this is a change to one number.
`--yaw-out` rotates BOTH mounts away from the centreline about the world z
axis, in degrees, mirrored. Under a reflection through x = 0 a rotation maps
(r, p, y) -> (r, -p, -y), and the pair stays a mirror by construction: the
right arm's matrix is built from the left arm's with the yaw negated, not
edited separately.

`--restore` puts back the values recorded in the backup file this script
writes on its first application, so a swept candidate cannot be left in the
robot by a session that ends early.
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(ROOT, "src/srl_description/urdf/srl_dual.urdf.xacro")
BACKUP = os.path.join(ROOT, "recordings/baselines/mount_as_built.json")

KEYS = ("mount_x", "mount_y", "mount_z", "left_mount_rpy", "right_mount_rpy")


def read():
    src = open(XACRO).read()
    out = {}
    for k in KEYS:
        m = re.search(r'<xacro:arg name="%s"\s*default="([^"]*)"/>' % k, src)
        if not m:
            raise SystemExit("cannot find xacro arg %s" % k)
        out[k] = m.group(1)
    return out


def write(vals):
    src = open(XACRO).read()
    for k, v in vals.items():
        src, n = re.subn(r'(<xacro:arg name="%s"\s*default=")[^"]*("/>)' % k,
                         lambda m: m.group(1) + str(v) + m.group(2), src)
        if n != 1:
            raise SystemExit("expected one %s, replaced %d" % (k, n))
    open(XACRO, "w").write(src)


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    """URDF's fixed-axis convention, R = Rz(y) Ry(p) Rx(r)."""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    if abs(math.cos(p)) < 1e-9:                       # gimbal lock
        return (math.atan2(-R[1, 2], R[1, 1]), p, 0.0)
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return (r, p, y)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=float, default=0.0,
                    help="metres OUTBOARD, mirrored, added to mount_x")
    ap.add_argument("--forward", type=float, default=0.0,
                    help="metres FORWARD, added to mount_y")
    ap.add_argument("--up", type=float, default=0.0,
                    help="metres UP, added to mount_z")
    ap.add_argument("--yaw-out", type=float, default=0.0,
                    help="degrees rotated AWAY from the centreline about z")
    ap.add_argument("--tilt", type=float, default=0.0,
                    help="degrees about world x; negative pitches forward")
    ap.add_argument("--symmetric-from", choices=("left", "right"), default=None,
                    help="make the mounts EXACT MIRRORS by deriving one from "
                         "the other before any delta is applied")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--restore", action="store_true")
    a = ap.parse_args()

    cur = read()
    if a.show:
        print(json.dumps(cur, indent=2))
        if os.path.exists(BACKUP):
            print("backup on file:")
            print(json.dumps(json.load(open(BACKUP)), indent=2))
        return 0

    if a.restore:
        if not os.path.exists(BACKUP):
            print("no backup at %s -- nothing to restore" % BACKUP)
            return 1
        write(json.load(open(BACKUP)))
        print("restored the as-built mount from %s" % BACKUP)
        print(json.dumps(read(), indent=2))
        return 0

    if not os.path.exists(BACKUP):
        os.makedirs(os.path.dirname(BACKUP), exist_ok=True)
        json.dump(cur, open(BACKUP, "w"), indent=2)
        print("recorded the as-built mount -> %s" % BACKUP)
    base = json.load(open(BACKUP))

    # THE PAIR IS NOT A MIRROR AS BUILT, and that is a lever of its own.
    # srl_dual.urdf.xacro records it: reflecting the LEFT mount through x = 0
    # leaves it 168 deg of roll from the RIGHT mount. Every certified sweep in
    # this session has the LEFT arm reaching 100-200 mm less far inboard than
    # the right, and this is the only asymmetry in the model that could
    # produce that. Under a reflection through x = 0 a rotation maps
    # (r, p, y) -> (r, -p, -y), so one side's rpy determines the other's.
    base = dict(base)
    if a.symmetric_from:
        src = "%s_mount_rpy" % a.symmetric_from
        dst = "%s_mount_rpy" % ("right" if a.symmetric_from == "left"
                                else "left")
        r0, p0, y0 = [float(v) for v in base[src].split()]
        base[dst] = "%.9f %.9f %.9f" % (r0, -p0, -y0)
        print("mirrored %s onto %s: %s" % (src, dst, base[dst]))

    vals = dict(base)
    vals["mount_x"] = "%.6g" % (float(base["mount_x"]) + a.out)
    vals["mount_y"] = "%.6g" % (float(base["mount_y"]) + a.forward)
    vals["mount_z"] = "%.6g" % (float(base["mount_z"]) + a.up)
    # THE LEFT MATRIX IS BUILT AND THE RIGHT ONE IS ITS MIRROR, so the pair
    # cannot be edited out of symmetry.
    rl = [float(v) for v in base["left_mount_rpy"].split()]
    Rl = rot_z(math.radians(a.yaw_out)) @ rot_x(math.radians(a.tilt)) \
        @ rpy_to_R(*rl)
    lr, lp, ly = R_to_rpy(Rl)
    rr = [float(v) for v in base["right_mount_rpy"].split()]
    Rr = rot_z(math.radians(-a.yaw_out)) @ rot_x(math.radians(a.tilt)) \
        @ rpy_to_R(*rr)
    rr_, rp_, ry_ = R_to_rpy(Rr)
    vals["left_mount_rpy"] = "%.9f %.9f %.9f" % (lr, lp, ly)
    vals["right_mount_rpy"] = "%.9f %.9f %.9f" % (rr_, rp_, ry_)
    write(vals)
    print("applied: out %+.3f m, forward %+.3f m, up %+.3f m, yaw-out %+.1f "
          "deg, tilt %+.1f deg" % (a.out, a.forward, a.up, a.yaw_out, a.tilt))
    print(json.dumps(read(), indent=2))
    print("\nRELAUNCH THE STACK. Nothing has changed in a running move_group.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
