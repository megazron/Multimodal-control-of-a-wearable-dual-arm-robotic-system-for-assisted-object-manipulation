#!/usr/bin/env python3
"""Forward kinematics for srl_dual, straight off the URDF. No stack required.

WHY THIS EXISTS. Every previous home-pose search in this project ran through
`/compute_ik`, which answers ONE question -- "give me a joint vector that puts
the end effector HERE, in THIS orientation" -- and answers it from whatever
seed it was handed. That is the wrong instrument for choosing a home pose, for
two reasons that between them account for three failed attempts:

  1. IT CANNOT SEARCH THE ROLL. `find_symmetric_home.py` reads each arm's
     CURRENT end-effector quaternion off TF and asks IK to hold it. The roll
     about the approach axis is therefore frozen at whatever the shipped pose
     happened to have, and the roll is exactly the degree of freedom that
     decides which way up the gripper camera points. A search that holds a
     quantity fixed cannot report it as a finding.
  2. THE TWO ARMS WERE ASKED DIFFERENT QUESTIONS. Each arm's target
     orientation came from its OWN TF, and those two quaternions are not
     mirror images of one another. No pair of solutions to a non-mirrored
     pair of targets can mirror, so the "structural asymmetry" that search
     reported is partly a property of the question it asked.

So: build the kinematics here, in the open, over all 14 joints, and let the
optimiser see every constraint at once.

WHAT IT IS. Standard URDF FK -- fixed and revolute joints, origin xyz/rpy
composed in the URDF's own order (translation then R = Rz(y)Ry(p)Rx(r)), joint
rotation about the joint axis. Nothing clever. It is vectorised over poses
only in the sense that it is cheap enough not to need to be.

THE INSTRUMENT IS CHECKED BEFORE IT IS USED. `self_test()` reproduces the
world positions that `recordings/baselines/home_render.json` recorded from
LIVE TF on a booted stack, for the shipped home pose, on eight links per arm.
That file is real data with known ground truth, which is what docs/ENGINEERING_LOG.md's
standing rule asks for -- not a synthetic pose whose "right answer" came from
this same code. A tolerance of 1 mm is used because home_render.json stores 4
decimal places.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
XACRO = os.path.join(ROOT, "src/srl_description/urdf/srl_dual.urdf.xacro")

# The Gen3 chain, shoulder outwards. `end_effector_link` is the frame every
# task pose is expressed in, and it is the frame the gripper and the wrist
# camera both hang off.
ARM_CHAIN = ["shoulder_link", "half_arm_1_link", "half_arm_2_link",
             "forearm_link", "spherical_wrist_1_link",
             "spherical_wrist_2_link", "bracelet_link", "end_effector_link"]


def rpy_matrix(r, p, y):
    """URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_matrix(axis, theta):
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    c, s, C = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c]])


def T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def expand_urdf(path=None, xacro_args=None):
    """Run xacro. The URDF this returns is the one the launch files build."""
    cmd = ["xacro", path or XACRO] + list(xacro_args or [])
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError("xacro failed:\n%s" % out.stderr[-4000:])
    return out.stdout


class FK:
    """Link poses in `world`, for a given left/right joint vector.

    Joint limits come from the URDF too, so a pose this class calls valid is
    one the URDF permits. `joint_7` and `joint_5` and `joint_3` on a Gen3 7DOF
    are CONTINUOUS -- they have no limit element -- and are reported as such
    rather than silently clamped to something.
    """

    def __init__(self, urdf_xml=None):
        from urdf_parser_py.urdf import URDF
        import io
        import contextlib
        xml = urdf_xml if urdf_xml is not None else expand_urdf()
        # urdf_parser_py writes "Unknown tag" chatter to stderr for gazebo and
        # ros2_control blocks it does not model. They are irrelevant to link
        # geometry; silence them so a real error is visible.
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.robot = URDF.from_xml_string(xml)
        self.byname = {j.name: j for j in self.robot.joints}
        self.parent_joint = {j.child: j for j in self.robot.joints}
        self.links = {ln.name for ln in self.robot.links}

    # ------------------------------------------------------------- structure
    def chain_to(self, link):
        """Joints from `world` down to `link`, in order."""
        out = []
        while link in self.parent_joint:
            j = self.parent_joint[link]
            out.append(j)
            link = j.parent
        return out[::-1]

    def limits(self, arm):
        """(lo, hi) per joint. Continuous joints get (-pi, pi) as a SEARCH
        box, and are flagged, because a continuous joint has no limit to
        report and a search still needs a range to sample from."""
        lo, hi, cont = [], [], []
        for i in range(1, 8):
            j = self.byname["%s_joint_%d" % (arm, i)]
            if j.type == "continuous" or j.limit is None:
                lo.append(-math.pi)
                hi.append(math.pi)
                cont.append(True)
            else:
                lo.append(float(j.limit.lower))
                hi.append(float(j.limit.upper))
                cont.append(False)
        return np.array(lo), np.array(hi), cont

    # -------------------------------------------------------------------- FK
    def poses(self, arm, q, links, gripper=None):
        """4x4 world transforms for `links`, with `arm`'s joints set to q.

        `links` are bare names ("forearm_link"); the arm prefix is added. A
        name that already carries the prefix is passed through, so the wearer
        and the mount can be asked for too.

        `gripper` is the knuckle angle in radians. THE HAND WAS UNREACHABLE
        FROM HERE UNTIL 2026-08-23: every gripper joint defaulted to 0.0 and
        the Robotiq's mimic joints were ignored entirely, so this class could
        not put the finger tips anywhere except one arbitrary opening -- and
        the finger tips are the only part of the robot that touches anything.

        The Robotiq 85 is a FOUR-BAR LINKAGE. Its fingers swing rather than
        translate, so the tips' distance ALONG THE TOOL AXIS is a function of
        the opening. "Where the pads are" is therefore not one number unless
        the opening is stated with it, and `grasp_frames.PAD_MID_EE` states
        one number.
        """
        want = set()
        full = []
        for ln in links:
            f = ln if ln.startswith(arm + "_") or ln in self.links \
                else "%s_%s" % (arm, ln)
            full.append(f)
            want.add(f)
        qmap = {"%s_joint_%d" % (arm, i + 1): float(q[i]) for i in range(7)}
        if gripper is not None:
            # The driving joint, then everything that MIMICS it. urdf_parser_py
            # carries the multiplier and offset; applying them is the whole
            # difference between a hand that opens and a hand frozen at 0.
            drive = "%s_robotiq_85_left_knuckle_joint" % arm
            qmap[drive] = float(gripper)
            for j in self.robot.joints:
                m = getattr(j, "mimic", None)
                if m is not None and m.joint in qmap:
                    mult = 1.0 if m.multiplier is None else float(m.multiplier)
                    off = 0.0 if m.offset is None else float(m.offset)
                    qmap[j.name] = qmap[m.joint] * mult + off
        cache = {}
        for f in full:
            if f in cache:
                continue
            M = np.eye(4)
            for j in self.chain_to(f):
                o = j.origin
                xyz = np.array(o.xyz if o and o.xyz else [0, 0, 0], float)
                rpy = o.rpy if o and o.rpy else [0, 0, 0]
                M = M @ T(rpy_matrix(*rpy), xyz)
                if j.type in ("revolute", "continuous"):
                    ang = qmap.get(j.name, 0.0)
                    M = M @ T(axis_matrix(np.array(j.axis, float), ang),
                              np.zeros(3))
                elif j.type == "prismatic":
                    ang = qmap.get(j.name, 0.0)
                    M = M @ T(np.eye(3), np.array(j.axis, float) * ang)
                cache[f] = M
            cache[f] = M
        return [cache[f] for f in full]

    def points(self, arm, q, links):
        return [M[:3, 3] for M in self.poses(arm, q, links)]


class CompiledFK:
    """The same FK, flattened into a tree of 4x4 steps for the optimiser.

    `FK.poses` walks the URDF and rebuilds every transform from scratch on
    every call, which is fine for a handful of queries and far too slow for a
    search that evaluates hundreds of thousands of poses. This precomputes,
    once, the ordered list of (fixed transform, which joint of q rotates here)
    for a set of links, sharing the common prefix between them.

    It is checked against `FK.poses` on random joint vectors rather than
    trusted: same URDF, two independent evaluators, and if they disagree the
    fast one is wrong.
    """

    def __init__(self, fk, arm, links):
        self.fk, self.arm, self.links = fk, arm, links
        self.steps = []          # (parent_index, T_fixed, q_index or -1)
        self.index = {}          # link name -> index into self.steps
        node = {}                # joint name -> step index producing its frame
        node[None] = -1
        for ln in links:
            full = ln if ln.startswith(arm + "_") or ln in fk.links \
                else "%s_%s" % (arm, ln)
            parent = -1
            for j in fk.chain_to(full):
                if j.name in node:
                    parent = node[j.name]
                    continue
                o = j.origin
                xyz = np.array(o.xyz if o and o.xyz else [0, 0, 0], float)
                rpy = o.rpy if o and o.rpy else [0, 0, 0]
                Tf = T(rpy_matrix(*rpy), xyz)
                qi = -1
                if j.type in ("revolute", "continuous"):
                    nm = "%s_joint_" % arm
                    if j.name.startswith(nm) and j.name[len(nm):].isdigit():
                        qi = int(j.name[len(nm):]) - 1
                    self.axis = None
                self.steps.append([parent, Tf,
                                   qi,
                                   np.array(j.axis, float)
                                   if j.type in ("revolute", "continuous")
                                   else None])
                parent = len(self.steps) - 1
                node[j.name] = parent
            self.index[ln] = parent

    def __call__(self, q):
        """World 4x4 for every link, in the order `links` was given."""
        out = [None] * len(self.steps)
        for i, (par, Tf, qi, axis) in enumerate(self.steps):
            M = Tf if par < 0 else out[par] @ Tf
            if axis is not None:
                ang = float(q[qi]) if qi >= 0 else 0.0
                R = axis_matrix(axis, ang)
                M = M.copy()
                M[:3, :3] = M[:3, :3] @ R
            out[i] = M
        return [out[self.index[ln]] for ln in self.links]


# --------------------------------------------------------------- the control
def compiled_matches_urdf(seed=3, n=25, verbose=True):
    """CompiledFK against FK.poses on random joint vectors. Instrument check."""
    rng = np.random.default_rng(seed)
    fk = FK()
    links = ["shoulder_link", "forearm_link", "end_effector_link",
             "camera_link", "robotiq_85_left_finger_tip_link"]
    worst = 0.0
    for arm in ("left", "right"):
        cf = CompiledFK(fk, arm, links)
        lo, hi, _ = fk.limits(arm)
        for _ in range(n):
            q = rng.uniform(lo, hi)
            a = cf(q)
            b = fk.poses(arm, q, links)
            for A, B in zip(a, b):
                worst = max(worst, float(np.abs(A - B).max()))
    ok = worst < 1e-9
    if verbose:
        print("   CompiledFK vs URDF walk, %d random poses/arm: worst "
              "element %.2e -> %s" % (n, worst, "PASS" if ok else "FAIL"))
    return ok, worst


def self_test(verbose=True):
    """Reproduce LIVE TF, from `home_render.json`, for the shipped home.

    Returns (ok, worst_error_m, detail). This is the known-answer test the
    standing rule demands, and its ground truth came off a booted stack rather
    than out of this file.
    """
    import json
    ref_path = os.path.join(ROOT, "recordings/baselines/home_render.json")
    ref = json.load(open(ref_path))
    fk = FK()
    worst, detail = 0.0, []
    for arm in ("left", "right"):
        q = ref["home_from_config_rad"][arm]
        want = ref["arms"][arm]
        got = dict(zip(["shoulder_link", "forearm_link", "half_arm_2_link",
                        "end_effector_link"],
                       fk.points(arm, q, ["shoulder_link", "forearm_link",
                                          "half_arm_2_link",
                                          "end_effector_link"])))
        for key, ln in (("shoulder", "shoulder_link"),
                        ("elbow_forearm_link", "forearm_link"),
                        ("elbow_half_arm_2", "half_arm_2_link"),
                        ("hand", "end_effector_link")):
            e = float(np.linalg.norm(np.array(want[key]) - got[ln]))
            worst = max(worst, e)
            detail.append((arm, key, want[key],
                           [round(float(v), 4) for v in got[ln]], round(e, 5)))
    ok = worst < 1e-3
    if verbose:
        print("FK CONTROL -- this file's FK against LIVE TF recorded in "
              "home_render.json")
        for arm, key, w, g, e in detail:
            print("   %-5s %-20s want %-28s got %-28s  %.5f m"
                  % (arm, key, w, g, e))
        print("   worst %.6f m -> %s" % (worst, "PASS" if ok else "FAIL"))
    return ok, worst, detail


if __name__ == "__main__":
    ok, worst, _ = self_test()
    ok2, _ = compiled_matches_urdf()
    sys.exit(0 if (ok and ok2) else 5)
