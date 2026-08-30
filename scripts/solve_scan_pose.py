#!/usr/bin/env python3
"""Solve a LOOK-STRAIGHT-DOWN scan pose that keeps the whole arm off the body.

    python3 scripts/solve_scan_pose.py --arm left
    python3 scripts/solve_scan_pose.py --both --save

WHAT MAKES THIS DIFFERENT FROM EVERY OTHER POSE SOLVE HERE
-----------------------------------------------------------
It is scored against the arm's SURFACE, not its joint origins.

`docs/PICK_THE_CUBE.md` records the measurement that makes this necessary:
asked about a pose in which the arm was physically jammed against the
mannequin, `ClearanceModel` returned 367.0 mm -- the same value it returns for
a visibly clear pose -- because it samples link origins and the metal between
the joints is invisible to it. Every "clearance" figure in this repository
solved that way is a statement about eight points, not about an arm.

So the wearer term here comes from `srl_body_geometry.wearer_clearance`,
which transforms the collision-mesh vertices the URDF already names and asks
the SAME body model about them. A pose this file calls clear is one whose
tubes are clear, not one whose joints are.

"ALL JOINTS OUTWARD" IS A SCORED TERM, NOT A HOPE
-------------------------------------------------
The operator asked for the arms held away from the body while scanning. That
is not implied by tool-down -- a 7-DOF arm has a null space, and the elbow can
sit inboard or outboard at the same hand pose. The redundancy is exactly the
joint that moves the elbow, so it is free to be spent on this.

`outwardness` is the signed distance of every joint origin from the wearer's
mid-sagittal plane, in the direction away from the body, worst joint first.
Maximising the WORST one (rather than the mean) is deliberate: a mean can be
bought by throwing the wrist far out while the elbow grazes the chest.

WHY TOOL-DOWN AND NOT THE PINNED ANCHOR
---------------------------------------
A scan wants the camera looking at the table, and the wrist camera looks along
the tool axis. CLAUDE.md records top-down-on-a-surface as 0 of 840 cells at
the pinned anchor -- but that was measured for a GRASP, with the pads at the
object. A scan hovers 250-400 mm above the surface, which is a different
question, and it is asked here rather than assumed either way.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_body_geometry import (ARM_LINKS, HAND_LINKS,  # noqa: E402
                               body_points, tool_extent, wearer_clearance)
from srl_fk import FK  # noqa: E402

WS = "/home/gausms/kortex_ws"
sys.path.insert(0, "%s/src/srl_teleop" % WS)
from srl_teleop.clearance import ClearanceModel  # noqa: E402

FLOOR_M = 0.150            # HARD CONSTRAINT 11's floor -- a HARD limit
# THE POSE MUST CLEAR THE FLOOR BY A MARGIN, not merely sit above it.
#
# The first scan pose cleared the floor by 67 mm on the corrected numbers and
# the PATH to it grazed the mannequin's forearm at 0.1508 m -- 0.8 mm of
# headroom, on a MODEL. The model is a nominal mannequin: the real body's
# size, posture and placement all carry error, and none of it is measured.
# 0.8 mm of margin against an unmeasured error is not a margin.
MARGIN_M = 0.250           # what a POSE and every point on its PATH must hold
PATH_STEPS = 30
JOINT_LINKS = ["shoulder_link", "half_arm_1_link", "half_arm_2_link",
               "forearm_link", "spherical_wrist_1_link",
               "spherical_wrist_2_link", "bracelet_link"]


def down_R(yaw):
    """EE +z along world -z (tool pointing down); free yaw about vertical."""
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def rot_err(Rc, Rd):
    E = Rd @ Rc.T
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    s, c = np.linalg.norm(v), (np.trace(E) - 1) / 2
    if s < 1e-9:
        return np.zeros(3) if c > 0 else np.array([np.pi, 0, 0])
    return v / s * math.atan2(s, c)


class ScanSolver:
    def __init__(self, arm):
        self.arm = arm
        self.fk = FK()
        self.model = ClearanceModel()
        self.lo, self.hi, self.cont = self.fk.limits(arm)
        # The arm's own base, so "outward" is measured from the body centre
        # rather than from the world origin, which is inside the wearer.
        self.base = self.fk.poses(arm, np.zeros(7), ["base_link"])[0][:3, 3]
        self.side = 1.0 if self.base[0] > 0 else -1.0

    # ------------------------------------------------------------------ FK
    def ee(self, q, grip=0.0):
        T = self.fk.poses(self.arm, q, ["end_effector_link"], gripper=grip)[0]
        return T[:3, 3], T[:3, :3]

    def ik(self, p_des, R_des, q0, iters=260, tol_p=2e-3, tol_r=math.radians(2)):
        q = np.array(q0, float)
        for _ in range(iters):
            p, R = self.ee(q)
            ep, er = p_des - p, rot_err(R, R_des)
            if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
                return q, np.linalg.norm(ep), np.linalg.norm(er)
            J = np.zeros((6, 7))
            d = 1e-6
            for j in range(7):
                qd = q.copy()
                qd[j] += d
                p2, R2 = self.ee(qd)
                J[:3, j] = (p2 - p) / d
                J[3:, j] = rot_err(R, R2) / d
            e = np.r_[ep, er]
            dq = J.T @ np.linalg.solve(J @ J.T + 0.0025 * np.eye(6), e)
            nn = np.linalg.norm(dq)
            if nn > 0.15:
                dq *= 0.15 / nn
            q = q + dq
            for j in range(7):
                if not self.cont[j]:
                    q[j] = min(self.hi[j], max(self.lo[j], q[j]))
        p, R = self.ee(q)
        return q, np.linalg.norm(p_des - p), np.linalg.norm(rot_err(R, R_des))

    # ----------------------------------------------------------- the scores
    def clearance(self, q, grip=0.0):
        """Worst SURFACE distance from the arm to the wearer.

        DELEGATED, because this file carried its own copy of the bug. It
        passed WORLD points to `ClearanceModel`, which wants them in each
        body part's OWN LINK FRAME -- so it scored the scan pose at 0.597 m
        and the arm hit the mannequin. One implementation now, in
        `srl_body_geometry.wearer_clearance`, so the two cannot disagree
        again.
        """
        d, link, part = wearer_clearance(self.fk, self.arm, q, grip, self.model)
        return d, ("%s vs %s" % (link, part) if link else None)

    def outwardness(self, q):
        """Worst joint's outboard distance from the body's mid-plane.

        Signed by the arm's own side, so a joint that has crossed the
        centreline scores NEGATIVE rather than merely small -- and a pose
        that tucks a joint across the wearer's chest can never win on this
        term by accident.
        """
        T = self.fk.poses(self.arm, q, JOINT_LINKS)
        return min(float(self.side * M[:3, 3][0]) for M in T)

    def score(self, q, grip=0.0):
        c, worst = self.clearance(q, grip)
        return dict(clearance=c, worst=worst, outward=self.outwardness(q))

    # ------------------------------------------------------------- the solve
    def solve(self, target, yaws=None, seeds=None, verbose=True):
        """Best tool-down pose over the free yaw and the null space.

        The yaw is swept because it is free; the redundancy is then spent by
        nudging along the null space toward outwardness, re-solving the task
        each step so the hand does not drift off the point being scanned.
        """
        yaws = yaws if yaws is not None else [math.radians(a)
                                              for a in range(0, 360, 20)]
        seeds = seeds if seeds is not None else [self._home()]
        best = None
        for yaw in yaws:
            R = down_R(yaw)
            for s in seeds:
                q, ep, er = self.ik(np.asarray(target, float), R, s)
                if ep > 2e-3 or er > math.radians(2):
                    continue
                q = self._push_outward(q, np.asarray(target, float), R)
                sc = self.score(q)
                # THE MARGIN, NOT THE FLOOR. The floor is where damage starts;
                # a pose accepted AT the floor has no room for the error in
                # the body model, which is a nominal mannequin whose size,
                # posture and placement are all unmeasured.
                if sc["clearance"] < MARGIN_M:
                    continue
                key = (sc["outward"], sc["clearance"])
                if best is None or key > best[0]:
                    best = (key, q, math.degrees(yaw), sc)
        if best is None:
            return None
        _, q, yaw, sc = best
        # WRAP THE CONTINUOUS JOINTS BACK INSIDE +/-pi.
        #
        # Joints 3, 5 and 7 are continuous, so the null-space push is free to
        # walk one of them past a wrap -- the first solve came out with
        # joint_5 at -286.39 deg. That is the SAME pose as +73.61 and the FK
        # agrees, but it is not the same COMMAND: it is 286 deg of travel from
        # home instead of 74, the bridge sees a huge step, and
        # `real_robot` mode refuses to start with a continuous joint wound
        # beyond +/-pi (HARD CONSTRAINT 8). Wrapping here is exact, not a
        # tolerance -- and the pose is re-scored afterwards so a wrap that
        # somehow changed the geometry would be caught rather than trusted.
        q = np.array(q, float)
        for j in range(7):
            if self.cont[j]:
                q[j] = (q[j] + np.pi) % (2 * np.pi) - np.pi
        sc2 = self.score(q)
        if abs(sc2["clearance"] - sc["clearance"]) > 1e-6:
            print("  !! wrapping a continuous joint CHANGED the clearance "
                  "(%.6f -> %.6f) -- refusing this pose"
                  % (sc["clearance"], sc2["clearance"]))
            return None
        sc = sc2
        if verbose:
            print("  %s: yaw %5.0f deg  clearance %.4f m (%s)  outward %.4f m"
                  % (self.arm, yaw, sc["clearance"], sc["worst"], sc["outward"]))
        return dict(arm=self.arm, q=[float(x) for x in q],
                    q_deg=[round(math.degrees(x), 3) for x in q],
                    yaw_deg=yaw, clearance_m=sc["clearance"],
                    worst_pair=sc["worst"], outward_m=sc["outward"],
                    target=[float(v) for v in target])

    def _push_outward(self, q, p_des, R_des, steps=14, gain=0.05):
        """Spend the null space on getting the joints away from the body.

        N = I - J+J projects out anything that would move the hand, so the
        hand pose is preserved by construction; the task is re-solved after
        each step anyway, because finite-difference null spaces drift.
        """
        for _ in range(steps):
            J = np.zeros((6, 7))
            d = 1e-6
            p, R = self.ee(q)
            for j in range(7):
                qd = q.copy()
                qd[j] += d
                p2, R2 = self.ee(qd)
                J[:3, j] = (p2 - p) / d
                J[3:, j] = rot_err(R, R2) / d
            Jp = np.linalg.pinv(J)
            N = np.eye(7) - Jp @ J
            g = np.zeros(7)
            base = self.outwardness(q)
            for j in range(7):
                qd = q.copy()
                qd[j] += 1e-4
                g[j] = (self.outwardness(qd) - base) / 1e-4
            step = N @ g
            nn = np.linalg.norm(step)
            if nn < 1e-9:
                break
            q_try = q + gain * step / nn
            for j in range(7):
                if not self.cont[j]:
                    q_try[j] = min(self.hi[j], max(self.lo[j], q_try[j]))
            q_try, ep, er = self.ik(p_des, R_des, q_try, iters=60)
            if ep > 2e-3 or er > math.radians(2):
                break
            if self.outwardness(q_try) <= base:
                break
            q = q_try
        return q

    def _home(self):
        d = {}
        for ln in open("%s/config/home_positions_%s.txt" % (WS, self.arm)):
            ln = ln.strip()
            if ln.startswith("joint_"):
                k, v = ln.split(":")
                d[k.strip()] = math.radians(float(v.split("#")[0]))
        return np.array([d["joint_%d" % i] for i in range(1, 8)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--both", action="store_true")
    ap.add_argument("--height", type=float, default=0.35,
                    help="hand height above the work surface, metres")
    ap.add_argument("--surface", type=float, default=0.75)
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()

    arms = ["left", "right"] if a.both else [a.arm]
    z = a.surface + a.height
    out = {}
    print("SCAN POSE  --  tool straight down, hand %.2f m above a %.2f m "
          "surface" % (a.height, a.surface))
    print("clearance is SURFACE-based, per body-part frame; margin %.3f m "
          "(floor %.3f m)" % (MARGIN_M, FLOOR_M))
    print("-" * 74)
    for arm in arms:
        s = ScanSolver(arm)
        # SWEEP WHERE TO STAND OFF FROM, rather than fixing one point.
        # The first version scanned one hard-coded cell (|x| 0.40, y 0.30) and
        # reported "no pose" if that cell was unreachable -- but the operator
        # asked for a scan pose, not for that cell. Further out and further
        # forward both buy clearance, so they are searched, nearest-to-the-
        # table first so the camera stays as close to the work as it can.
        side = 1.0 if s.base[0] > 0 else -1.0
        r = None
        for y in (0.30, 0.38, 0.46):
            for xa in (0.40, 0.48, 0.56):
                r = s.solve([xa * side, y, z], verbose=False)
                if r is not None:
                    print("  %s: standing off at (%+.2f, %.2f, %.2f)"
                          % (arm, xa * side, y, z))
                    print("     yaw %.0f deg  clearance %.4f m (%s)  "
                          "outward %.4f m"
                          % (r["yaw_deg"], r["clearance_m"], r["worst_pair"],
                             r["outward_m"]))
                    break
            if r is not None:
                break
        if r is None:
            print("  %s: NO tool-down pose over this table clears the wearer "
                  "by the %.3f m margin, at any standoff tried." % (arm, MARGIN_M))
            continue
        t = tool_extent(s.fk, arm, np.array(r["q"]), 0.0)
        r["tips_past_pads_mm"] = round(t["tips_past_pads"] * 1000, 2)
        r["hand_reach_along_tool_m"] = round(t["along_tool"], 5)
        r["lowest_point_z"] = round(t["lowest_world_z"], 4)
        print("     hand reaches %.1f mm past the wrist; lowest point z=%.3f"
              % (t["along_tool"] * 1000, t["lowest_world_z"]))
        out[arm] = r
    if a.save and out:
        p = "%s/recordings/baselines/scan_pose.json" % WS
        os.makedirs(os.path.dirname(p), exist_ok=True)
        json.dump(out, open(p, "w"), indent=1)
        print("\nwritten to %s" % p)
    return 0 if out else 1


if __name__ == "__main__":
    sys.exit(main())
