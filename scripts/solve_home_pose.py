#!/usr/bin/env python3
"""SOLVE the home pose from geometric constraints. Do not guess joint angles.

    python3 scripts/solve_home_pose.py                  # solve, report, no writes
    python3 scripts/solve_home_pose.py --apply          # write the pose out
    python3 scripts/solve_home_pose.py --self-test      # can it fail?

WHY THIS EXISTS, AND WHY THE PREVIOUS ATTEMPTS DID NOT TAKE
-----------------------------------------------------------
Three sessions reported a home pose satisfying "wrists level, hands in front
of the chest, symmetric" while the render showed arms winged out at the sides
with the elbows up. The angles were not lost between the file and the render:
`measure_home_render.py` reads the live stack at 7.4e-5 rad from the source
file, and `srl_fk.py` reproduces live TF to 0.06 mm from the same URDF the
launch builds. Nothing is stale, cached or overwritten.

What went wrong is that the three claims were each TRUE OF A QUANTITY THAT DID
NOT CAPTURE THE PICTURE:

    "wrists level"     the TOOL AXIS elevation is 0.00 deg, and always was.
                       The ROLL about that axis was never constrained by
                       anything, and it landed with the wrist camera 56 mm
                       BELOW the fingertips on both arms -- the hand is
                       upside down. A level axis says nothing about which way
                       up the hand is.
    "hands in front"   y = +0.36, genuinely in front. Also |x| = 0.55, so the
                       hands are 1.10 m apart across a 0.36 m torso.
    "symmetric"        the END EFFECTORS mirror to 0.0001 m. The FOREARMS
                       mirror to 0.2826 m. Symmetry measured at one link is
                       not symmetry.

So this file does not propose better angles. It states (a)-(g) as things to
satisfy, searches the joint space for them, and prints each constraint's
ACHIEVED value beside its target -- including the ones it cannot meet.

WHAT IS SEARCHED, AND WHY IT IS A DIFFERENT SEARCH FROM THE LAST ONE
--------------------------------------------------------------------
`find_symmetric_home.py` asked `/compute_ik` to hold each arm's CURRENT
end-effector quaternion. That froze the roll -- the one degree of freedom
constraint (d) is about -- and it handed the two arms targets that are not
mirror images of each other, so no pair of solutions could have mirrored. Its
conclusion that symmetry is "structural" was partly an artefact of the
question.

Here the whole 14-dimensional joint space is searched directly against the
constraints, with no orientation pinned in advance, using FK validated against
live TF. The mounts really do differ by 168 deg of roll and that really does
matter; this measures how much, rather than inheriting it.

CONTROLS, and there is no report without them:

    the FK reproduces live TF          srl_fk.self_test, 0.06 mm on 8 links
    the compiled FK matches the URDF   two independent evaluators, 1e-9
    the SHIPPED pose scores as the     the scorer must reproduce the numbers
    render shows                       the render was measured at, or it is
                                       not scoring what a viewer is looking at
    a deliberately impossible target   hands demanded INSIDE the torso must
                                       return infeasible, not a compromise
    the mirror test can be non-zero    reflecting the LEFT arm against ITSELF
                                       must score large
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from srl_fk import FK, CompiledFK, self_test as fk_self_test        # noqa: E402
from srl_fk import compiled_matches_urdf                            # noqa: E402
from srl_teleop import mount_guard_node as MG                       # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/home_solved.json")

# ---------------------------------------------------------------- the wearer
#
# Taken from mount_guard_node, which is what the follower, the homing node and
# the bridge actually enforce, so a pose this file calls clear is clear by the
# same model that would stop the arm. docs/ENGINEERING_LOG.md rule 11: wearer_posture.py is
# the ONE source, and reaching for it here rather than re-declaring a torso is
# the difference between measuring the wearer and measuring a copy of them.
TORSO = [w for w in MG.WEARER if w[0] == "torso"][0]
TORSO_HALF_W = TORSO[2][0] * 0.5          # 0.18
TORSO_FRONT_Y = TORSO[3][1] + TORSO[2][1] * 0.5   # 0.11
FLOOR = 0.15

# CONTINUOUS JOINTS MUST NOT REST ON THE +/-pi SEAM.
#
# joint_1/3/5/7 of a Gen3 7DOF are continuous, and this project keeps a 0.3 rad
# margin from the wrap point -- findings.md records the left arm's joint_5 at
# 14.10 deg from the seam as a RISK worth writing down. A solver with a +/-pi
# search box will happily park a joint AT the seam if nothing forbids it: the
# first run of the corrected constraint set returned left joint_5 = 180.00 deg,
# which is 0.00 rad of margin, the worst value available. Nothing downstream
# would have caught it -- test_home_has_one_source checks joints are INSIDE
# +/-pi, and pi is inside.
CONTINUOUS_IDX = (0, 2, 4, 6)
SEAM_MARGIN_RAD = 0.30

# THE MOUNT IS NOT A JOINT, AND IT SETS THE CEILING ON THIS WHOLE QUESTION.
#
# `MG.CHAIN` starts at `base_link`, whose origin is FIXED by the backpack
# frame -- measured here rather than believed: over 50 random poses across the
# full joint range it returns ONE distinct position on each arm. So the worst
# clearance over the whole chain can never exceed the clearance of that one
# point, no matter what the seven joints do.
#
# docs/ENGINEERING_LOG.md carries this as "base_link sits 0.1610 m from the torso". THAT
# FIGURE IS STALE: the mounts moved 150 mm outboard and 15 deg of yaw on
# 2026-08-18 and nothing re-measured it. It is 0.2202 m today, computed below
# from the geometry that is actually loaded.
#
# The consequence is the answer to "solve home with a much larger margin":
# against the WHOLE chain there is nothing to solve, because the fixed
# structure already binds. What a home pose can move is everything from
# `shoulder_link` outward, so that is scored as its own number.
MOVING_FIRST_SEG = 2        # see measure_home_clearance.py: base_link AND
#                             shoulder_link are immobile, and half_arm_1_link
#                             moves 6 mm over the whole joint range

LINKS = ["shoulder_link", "forearm_link", "end_effector_link", "camera_link",
         "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
IDX = {n: i for i, n in enumerate(LINKS)}


# ------------------------------------------------------------------ scoring
class Scorer:
    """Achieved value of every constraint, for one arm and for the pair."""

    def __init__(self):
        self.fk = FK()
        self.cf = {a: CompiledFK(self.fk, a, LINKS) for a in ("left", "right")}
        self.chain = {a: CompiledFK(self.fk, a, MG.CHAIN)
                      for a in ("left", "right")}
        self.lim = {a: self.fk.limits(a) for a in ("left", "right")}
        self.prims = self._prims()

    # -- (f) ---------------------------------------------------------------
    #
    # Same capsule model, same tube radius and same sample count as
    # `mount_guard_node`, because a pose cleared by a looser model is a pose
    # the guard will stop. It is vectorised only because the scalar version
    # costs 5 ms per call and the search needs a few million of them; it is
    # checked against `MG.dist_point` point by point in `vectorised_clearance
    # _matches_the_guard()` rather than assumed equivalent.
    @staticmethod
    def _prims():
        out = []
        from srl_teleop import wearer_posture as WP
        for nm, kind, prm, ctr, rpy in MG.WEARER:
            R = (np.asarray(WP.rot_matrix(rpy), float)
                 if any(abs(v) > 1e-12 for v in rpy) else None)
            out.append((nm, kind, np.asarray(prm, float),
                        np.asarray(ctr, float), R))
        return out

    def _dist_to(self, P, prim):
        """Signed distance from every row of P to one primitive."""
        _, kind, prm, ctr, R = prim
        Q = P - ctr
        if R is not None:
            Q = Q @ R                      # R.T @ q, per row
        if kind == "box":
            half = prm * 0.5
            aq = np.abs(Q)
            d = np.maximum(aq - half, 0.0)
            n = np.linalg.norm(d, axis=1)
            inside = n <= 0
            if inside.any():
                n = np.where(inside, -np.min(half - aq, axis=1), n)
            return n
        if kind == "cylinder":
            r, L = float(prm[0]), float(prm[1])
            radial = np.hypot(Q[:, 0], Q[:, 1]) - r
            axial = np.abs(Q[:, 2]) - L * 0.5
            both = (radial <= 0) & (axial <= 0)
            outv = np.hypot(np.maximum(radial, 0.0), np.maximum(axial, 0.0))
            return np.where(both, np.maximum(radial, axial), outv)
        return np.linalg.norm(Q, axis=1) - float(prm[0])

    def segments(self, arm, q):
        """Sample points, kept per segment: (n_segments, SAMPLES+1, 3)."""
        pts = np.array([M[:3, 3] for M in self.chain[arm](q)])
        A, B = pts[:-1], pts[1:]
        t = np.linspace(0.0, 1.0, MG.SAMPLES + 1)
        return A[:, None, :] + (B - A)[:, None, :] * t[None, :, None]

    def sample_points(self, arm, q, first=0):
        """Every sample point from segment `first` onward, flattened.

        `first=0` is the guard's own set and is what `clearance()` scores.
        `first=MOVING_FIRST_SEG` drops the one segment that starts at the
        immobile mount.
        """
        return self.segments(arm, q)[first:].reshape(-1, 3)

    def clearance(self, arm, q, first=0):
        """Worst clearance over the capsule chain, and which wearer part."""
        P = self.sample_points(arm, q, first)
        worst, who = None, None
        for prim in self.prims:
            d = self._dist_to(P, prim) - MG.TUBE_R
            m = float(d.min())
            if worst is None or m < worst:
                worst, who = m, prim[0]
        return float(worst), who

    def mount_cap(self, arm):
        """Clearance of the immobile stub alone. The ceiling on (f).

        Computed from the loaded geometry every time rather than quoted, so a
        mount that moves again cannot leave a stale ceiling behind.
        """
        S = self.segments(arm, np.zeros(7))[:MOVING_FIRST_SEG].reshape(-1, 3)
        return min(float((self._dist_to(S, pr) - MG.TUBE_R).min())
                   for pr in self.prims)

    def clearance_parts(self, arm, q):
        """The full breakdown. Reporting only -- never in the search loop.

        Per WEARER link and per ARM segment, because "0.2202 m of clearance"
        does not say whether the thing that is close is the fixed mount or the
        gripper, and the brief asks for exactly that distinction.
        """
        S = self.segments(arm, q)
        flat = S.reshape(-1, 3)
        moving = S[MOVING_FIRST_SEG:].reshape(-1, 3)
        per_wearer, per_wearer_moving = {}, {}
        for pr in self.prims:
            per_wearer[pr[0]] = float((self._dist_to(flat, pr)
                                       - MG.TUBE_R).min())
            per_wearer_moving[pr[0]] = float((self._dist_to(moving, pr)
                                              - MG.TUBE_R).min())
        per_seg = []
        for i in range(S.shape[0]):
            d = [(float((self._dist_to(S[i], pr) - MG.TUBE_R).min()), pr[0])
                 for pr in self.prims]
            v, who = min(d)
            per_seg.append(dict(segment="%s -> %s" % (MG.CHAIN[i],
                                                      MG.CHAIN[i + 1]),
                                fixed=(i < MOVING_FIRST_SEG),
                                clearance_m=round(v, 4), to=who))
        w_v, w_who = min((v, k) for k, v in per_wearer.items())
        m_v, m_who = min((v, k) for k, v in per_wearer_moving.items())
        return dict(whole_chain_m=w_v, whole_chain_to=w_who,
                    moving_chain_m=m_v, moving_chain_to=m_who,
                    mount_cap_m=self.mount_cap(arm),
                    per_wearer_link_m={k: round(v, 4)
                                       for k, v in per_wearer.items()},
                    per_wearer_link_moving_m={k: round(v, 4)
                                              for k, v in
                                              per_wearer_moving.items()},
                    per_arm_segment=per_seg)

    # -- (a)-(e) -----------------------------------------------------------
    def arm_terms(self, arm, q, with_clearance=True):
        M = self.cf[arm](q)
        sh = M[IDX["shoulder_link"]][:3, 3]
        el = M[IDX["forearm_link"]][:3, 3]
        ee_M = M[IDX["end_effector_link"]]
        ee = ee_M[:3, 3]
        cam = M[IDX["camera_link"]][:3, 3]
        f1 = M[IDX["robotiq_85_left_finger_tip_link"]][:3, 3]
        f2 = M[IDX["robotiq_85_right_finger_tip_link"]][:3, 3]
        fing = (f1 + f2) * 0.5

        # (a) the forearm, as a viewer sees it: elbow to hand.
        v = ee - el
        n = float(np.linalg.norm(v))
        fwd = v / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])
        a_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(fwd[1])))))

        # (c) the approach axis is the end-effector's own z.
        #
        # TWO NUMBERS, NOT ONE, AND THE SECOND WAS MISSING FOR A WHOLE SOLVE.
        # "Level with the ground, not pitched up or down" is an ELEVATION
        # constraint and says nothing about which way round the horizon the
        # hand is aimed. Solved with elevation alone, the answer put both
        # approach axes along x, pointing INWARD: the two grippers faced each
        # other across the wearer's chest, level, symmetric, and useless. The
        # render is what caught it. So the azimuth is a constraint of its own.
        appr = ee_M[:3, 2]
        c_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(appr[2])))))
        c2_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(appr[1])))))

        # (d) THE WRIST CAMERA, AS AN ANGLE, BECAUSE THE HEIGHT TEST WAS TOO
        # WEAK. It asked cam_z > fingertip_z + 0.02, and a camera sitting out
        # to the SIDE of the gripper satisfies that as long as it is a couple
        # of centimetres higher. Measured on the pose that passed it: the
        # camera offset direction was 61.86 deg off vertical on both arms --
        # up and sideways, not on top -- and the render showed it.
        #
        # The quantity that actually pins the roll about the approach axis is
        # the DIRECTION from the end-effector origin to the camera, against
        # world +z. It is 0 deg when the camera is directly on top and 90 when
        # it is flat out to the side, and it is independent of how far forward
        # the fingertips are.
        cam_dir = cam - ee
        d_deg = math.degrees(math.acos(max(-1.0, min(
            1.0, float(cam_dir[2] / max(1e-9, np.linalg.norm(cam_dir)))))))
        # Reported, not constrained: the same idea measured from the GRIPPER
        # CENTRE, which is what a viewer's eye does. It cannot go below about
        # 63 deg while the approach axis is level, because the fingertips are
        # 0.1118 m forward along the approach and the camera only 0.0564 m up
        # from the wrist -- so a <15 deg target on THIS vector would demand a
        # wrist pitched ~60 deg down and would contradict (c). That is the
        # genuine (c)/(d) conflict, and it is stated rather than resolved by
        # preference. Constraining the offset DIRECTION avoids it entirely.
        cf_v = cam - fing
        d_fing_deg = math.degrees(math.acos(max(-1.0, min(
            1.0, float(cf_v[2] / max(1e-9, np.linalg.norm(cf_v)))))))
        d_m = float(cam[2] - fing[2])

        out = dict(
            elbow_below_shoulder_m=float(sh[2] - el[2]),          # (b)
            forearm_off_forward_deg=a_deg,                        # (a)
            approach_elevation_deg=c_deg,                         # (c)
            approach_off_forward_deg=c2_deg,                      # (c2)
            camera_off_top_deg=d_deg,                             # (d)
            camera_from_fingertips_off_top_deg=d_fing_deg,        # reported
            camera_above_fingers_m=d_m,                           # reported
            hand=[float(t) for t in ee],                          # (e)
            hand_abs_x_m=abs(float(ee[0])),
            hand_in_front_of_torso_m=float(ee[1] - TORSO_FRONT_Y),
            shoulder=[float(t) for t in sh],
            elbow=[float(t) for t in el],
            camera=[float(t) for t in cam],
            fingers=[float(t) for t in fing],
            approach_axis=[float(t) for t in appr],
            camera_dir_world_z=float(ee_M[2, 1]),
        )
        if with_clearance:
            c, who = self.clearance(arm, q)
            out["clearance_m"], out["clearance_to"] = c, who
            cm, whom = self.clearance(arm, q, MOVING_FIRST_SEG)
            out["clearance_moving_m"], out["clearance_moving_to"] = cm, whom
        return out

    # -- (g) ---------------------------------------------------------------
    def mirror(self, ql, qr):
        """Left reflected through x = 0 against right: EE, and every link.

        The EE-only figure is the one the brief asks for. The whole-chain
        figure is reported beside it because the shipped pose passes the first
        at 0.0001 m and fails the second at 0.2826 m, and the picture agrees
        with the second.
        """
        pl = [M[:3, 3] for M in self.chain["left"](ql)]
        pr = [M[:3, 3] for M in self.chain["right"](qr)]
        per = {}
        for ln, p, r in zip(MG.CHAIN, pl, pr):
            per[ln] = float(np.linalg.norm(
                np.array([-p[0], p[1], p[2]]) - r))
        ee = float(np.linalg.norm(
            np.array([-pl[MG.CHAIN.index("end_effector_link")][0],
                      pl[MG.CHAIN.index("end_effector_link")][1],
                      pl[MG.CHAIN.index("end_effector_link")][2]])
            - pr[MG.CHAIN.index("end_effector_link")]))
        return ee, max(per.values()), per


    def self_mirror(self, arm, q):
        """The arm reflected through x = 0 against ITSELF.

        The control that a mirror residual of zero means symmetry rather than
        a comparison that cannot move: an arm sitting off the centreline must
        score large against its own reflection.
        """
        p = np.array([M[:3, 3] for M in self.chain[arm](q)])
        r = p * np.array([-1.0, 1.0, 1.0])
        return float(np.linalg.norm(p - r, axis=1).max())


def vectorised_clearance_matches_the_guard(sc, seed=11, n=8, verbose=True):
    """The fast clearance against `mount_guard_node.dist_point`, point by point.

    The fast path exists for speed alone, so it has to give the guard's own
    answer. Random poses across the whole joint range, both arms, every sample
    point against every wearer primitive -- not just the minimum, because two
    different distance fields can share a minimum by accident.
    """
    rng = np.random.default_rng(seed)
    worst = 0.0
    for arm in ("left", "right"):
        lo, hi, _ = sc.lim[arm]
        for _ in range(n):
            q = rng.uniform(lo, hi)
            P = sc.sample_points(arm, q)
            for prim, (nm, kind, prm_, ctr, rpy) in zip(sc.prims, MG.WEARER):
                fast = sc._dist_to(P, prim)
                slow = np.array([MG.dist_point(p, kind, prm_, ctr, rpy)
                                 for p in P])
                worst = max(worst, float(np.abs(fast - slow).max()))
    ok = worst < 1e-9
    if verbose:
        print("   vectorised clearance vs mount_guard.dist_point     "
              "%.1e -> %s" % (worst, "PASS" if ok else "FAIL"))
    return ok, worst


# ------------------------------------------------------------------ targets
class Target:
    """The constraints, as numbers with a tolerance and a direction."""

    def __init__(self, hand_x, hand_y, hand_z,
                 elbow_margin=0.10, forearm_tol_deg=25.0,
                 level_tol_deg=2.0, camera_tol_deg=15.0,
                 approach_tol_deg=25.0,
                 floor=FLOOR, moving_floor=None, mirror_tol=0.02):
        self.hand_x = hand_x
        self.hand_y = hand_y
        self.hand_z = hand_z
        self.elbow_margin = elbow_margin
        self.forearm_tol_deg = forearm_tol_deg
        self.level_tol_deg = level_tol_deg
        self.approach_tol_deg = approach_tol_deg
        self.camera_tol_deg = camera_tol_deg
        self.floor = floor
        # (f2) THE CONSTRAINT THE BRIEF ACTUALLY ASKS FOR. (f) is capped by
        # the mount and cannot be raised; this one is about the seven joints.
        # Defaults to (f) so nothing that does not ask for it changes.
        self.moving_floor = floor if moving_floor is None else moving_floor
        self.mirror_tol = mirror_tol

    def check(self, t, q=None):
        """Per-constraint pass/fail and shortfall, for one arm's terms."""
        if q is None:
            self.seam_worst, self.seam_ok = float("nan"), True
        else:
            m = min(math.pi - abs(float(v)) for i, v in enumerate(q)
                    if i in CONTINUOUS_IDX)
            self.seam_worst, self.seam_ok = m, m >= SEAM_MARGIN_RAD
        return {
            "a_forearm_forward": (
                t["forearm_off_forward_deg"] <= self.forearm_tol_deg,
                t["forearm_off_forward_deg"], "<= %.0f deg"
                % self.forearm_tol_deg),
            "b_elbow_below_shoulder": (
                t["elbow_below_shoulder_m"] >= self.elbow_margin,
                t["elbow_below_shoulder_m"], ">= %.2f m" % self.elbow_margin),
            "c_wrist_level": (
                abs(t["approach_elevation_deg"]) <= self.level_tol_deg,
                t["approach_elevation_deg"], "|elev| <= %.1f deg"
                % self.level_tol_deg),
            "c2_wrist_faces_forward": (
                t["approach_off_forward_deg"] <= self.approach_tol_deg,
                t["approach_off_forward_deg"], "<= %.0f deg"
                % self.approach_tol_deg),
            "d_camera_on_top": (
                t["camera_off_top_deg"] <= self.camera_tol_deg,
                t["camera_off_top_deg"], "<= %.0f deg off +z"
                % self.camera_tol_deg),
            "e_hand_x": (
                abs(t["hand_abs_x_m"] - self.hand_x) <= 0.03,
                t["hand_abs_x_m"], "|x| = %.3f m" % self.hand_x),
            "e_hand_in_front": (
                t["hand_in_front_of_torso_m"] >= 0.10,
                t["hand_in_front_of_torso_m"], ">= 0.10 m"),
            "h_seam_margin": (
                self.seam_ok, self.seam_worst, ">= %.2f rad from +/-pi"
                % SEAM_MARGIN_RAD),
            "f_clearance": (
                t.get("clearance_m", -9) >= self.floor,
                t.get("clearance_m", float("nan")), ">= %.3f m" % self.floor),
            "f2_moving_clearance": (
                t.get("clearance_moving_m", -9) >= self.moving_floor,
                t.get("clearance_moving_m", float("nan")),
                ">= %.3f m" % self.moving_floor),
        }


# SOLVE STRICTLY INSIDE THE CONSTRAINT, CHECK ON IT.
#
# A hinge penalty has zero gradient the instant the constraint is satisfied,
# so the optimiser parks exactly on the boundary: the first run of this sweep
# returned 0.14950 m of clearance against a 0.150 floor and 25.78 deg against
# a 25 deg limit, which is not a solution, it is a rounding error away from
# one. docs/ENGINEERING_LOG.md's own rule says to stay 20 mm inside the last pose that
# passed. So the COST aims for these margins and the CHECK still uses the
# stated constraint -- the reported numbers are therefore true of the
# constraint as written, with room to spare rather than none.
SOLVE_MARGIN = dict(clearance=0.012, forearm_deg=4.0, camera=0.012,
                    elbow=0.025, level_deg=0.6, approach_deg=4.0,
                    camera_deg=3.0, mirror=0.004, seam=0.05)
# (h) HAD NO MARGIN, AND A HINGE WITH NO MARGIN PARKS ON THE BOUNDARY.
# The |x| = 0.780 sweep row came back with the right arm 0.3002 rad from the
# wrap point against a 0.30 rad limit -- 0.2 mrad of room, which is the same
# "rounding error away from a solution" this block exists to forbid. The cost
# now aims 0.05 rad inside; the CHECK is still the stated 0.30.


def cost(q, sc, arm, tg, w=None):
    """Smooth penalty. Zero when every constraint is met WITH MARGIN."""
    t = sc.arm_terms(arm, q, with_clearance=True)
    hinge = lambda v: v * v if v > 0 else 0.0                     # noqa: E731
    m = SOLVE_MARGIN
    ee = np.array(t["hand"])
    want = np.array([math.copysign(tg.hand_x, 1.0 if arm == "left" else -1.0),
                     tg.hand_y, tg.hand_z])
    c = 0.0
    # POSITION IS A CONSTRAINT, NOT A PREFERENCE. At weight 40 the optimiser
    # traded 24 mm of hand position against the forearm-angle penalty and
    # reported a column it was not standing in -- target |x| = 0.290, achieved
    # 0.3145. The sweep's whole purpose is to find WHERE the hands can be, so
    # the hand has to end up where the row says it is.
    c += 400.0 * float(np.sum((ee - want) ** 2))                         # (e)
    c += 2.0e-3 * hinge(t["forearm_off_forward_deg"]
                        - (tg.forearm_tol_deg - m["forearm_deg"]))       # (a)
    c += 60.0 * hinge(tg.elbow_margin + m["elbow"]
                      - t["elbow_below_shoulder_m"])                     # (b)
    c += 4.0e-3 * hinge(abs(t["approach_elevation_deg"])
                        - (tg.level_tol_deg - m["level_deg"]))           # (c)
    c += 2.0e-3 * hinge(t["approach_off_forward_deg"]
                        - (tg.approach_tol_deg - m["approach_deg"]))     # (c2)
    c += 6.0e-3 * hinge(t["camera_off_top_deg"]
                        - (tg.camera_tol_deg - m["camera_deg"]))         # (d)
    c += 400.0 * hinge(tg.floor + m["clearance"] - t["clearance_m"])     # (f)
    c += 400.0 * hinge(tg.moving_floor + m["clearance"]
                       - t["clearance_moving_m"])                        # (f2)
    # Preferences, an order of magnitude below the constraints: as forward as
    # the wrist can be, as low an elbow as the rest allows. They break ties;
    # they never override a constraint.
    # (h) keep every continuous joint clear of the +/-pi seam
    for i in CONTINUOUS_IDX:
        c += 20.0 * hinge(SEAM_MARGIN_RAD + m["seam"]
                          - (math.pi - abs(float(q[i]))))
    c += 1.0e-4 * t["forearm_off_forward_deg"]
    c += 1.0e-3 * max(0.0, 0.25 - t["elbow_below_shoulder_m"])
    return c


def solve_arm(sc, arm, tg, seeds, iters=260):
    """Best joint vector for one arm. Multi-start, then keep every branch."""
    from scipy.optimize import minimize
    lo, hi, _ = sc.lim[arm]
    bounds = list(zip(lo, hi))
    found = []
    for s in seeds:
        s = np.clip(np.asarray(s, float), lo, hi)
        try:
            r = minimize(cost, s, args=(sc, arm, tg), method="L-BFGS-B",
                         bounds=bounds,
                         options=dict(maxiter=iters, ftol=1e-12, eps=1e-6))
        except Exception:                                         # noqa: BLE001
            continue
        q = np.array(r.x)
        found.append((float(cost(q, sc, arm, tg)), q))
    found.sort(key=lambda t: t[0])
    # de-duplicate branches at 0.02 rad, so "many solutions" means many
    # postures rather than one posture found many times
    keep = []
    for c, q in found:
        if all(np.max(np.abs(((q - k[1] + math.pi) % (2 * math.pi)) - math.pi))
               > 0.02 for k in keep):
            keep.append((c, q))
        if len(keep) >= 12:
            break
    return keep


def seed_set(sc, arm, n_random, rng, prior=None):
    """Zeros, the shipped home, any PRIOR solution, then random restarts.

    `prior` lets a re-run start from a previous run's answer, so refining the
    search never costs the ground already covered.
    """
    lo, hi, _ = sc.lim[arm]
    seeds = [np.zeros(7)]
    import json as _j
    # The 2026-08-15 pose, and THE SAME POSE WITH joint_7 TURNED HALF A TURN.
    # Measured: that single half-turn of the continuous wrist roll takes the
    # camera from 56.4 mm BELOW the fingertips to 56.4 mm above, with the
    # approach axis still level (0.00 deg) and still forward (0.004 deg off).
    # The shipped pose was one wrist flip from satisfying (c), (c2) and (d)
    # together. It is far too good a starting point to leave out.
    SHIPPED = {
        "left": [-2.353053, -1.142842, -0.455705, -1.362579, -2.652726,
                 -0.66497, 2.90894],
        "right": [-0.883137, 1.759466, 1.679356, 1.376716, 0.200189,
                  -0.50824, -1.644973]}
    base = np.array(SHIPPED[arm])
    seeds.append(base)
    flip = base.copy()
    flip[6] = math.pi - ((math.pi - (flip[6] + math.pi)) % (2 * math.pi))
    seeds.append(flip)
    ref = os.path.join(ROOT, "recordings/baselines/home_render.json")
    if os.path.exists(ref):
        seeds.append(np.array(_j.load(open(ref))["home_from_config_rad"][arm]))
    if prior is not None:
        seeds += [np.asarray(p, float) for p in prior]
    seeds += [rng.uniform(lo, hi) for _ in range(n_random)]
    return seeds


def priors_from(path, arm):
    """Every joint vector a previous run recorded, as seeds for this one."""
    if not path or not os.path.exists(path):
        return {}
    d = json.load(open(path))
    out = {}
    for row in d.get("sweep") or []:
        q = row.get("q_%s" % arm)
        if q:
            out.setdefault(round(float(row["x"]), 3), []).append(q)
    sol = d.get("solved")
    if sol and sol.get("q_%s" % arm):
        out.setdefault(round(float(sol["hand_abs_x_m"]), 3), []).append(
            sol["q_%s" % arm])
    return out


def refine_pair(sc, tg, ql, qr, iters=900):
    """Polish BOTH arms at once, against the whole-chain mirror residual.

    The column sweep picks the best available PAIR, but each arm was optimised
    against its own target with symmetry only as a tie-break. Constraint (g)
    as the brief states it -- end effectors within 0.02 m -- is then satisfied
    trivially, because the two targets are exact mirrors of each other and any
    arm that reaches its target satisfies it. The shipped pose passes that
    same test at 0.0001 m while looking nothing like a mirror image.

    So the quantity worth minimising is the residual over the WHOLE CHAIN, and
    it needs both arms in one optimisation because it is a property of the
    pair. Everything else stays a constraint; symmetry is the objective.
    """
    from scipy.optimize import minimize
    lo_l, hi_l, _ = sc.lim["left"]
    lo_r, hi_r, _ = sc.lim["right"]
    bounds = list(zip(lo_l, hi_l)) + list(zip(lo_r, hi_r))
    _, ch_old, _ = sc.mirror(ql, qr)

    def make(w):
        def f(v):
            a, b = v[:7], v[7:]
            c = cost(a, sc, "left", tg) + cost(b, sc, "right", tg)
            pl = np.array([M[:3, 3] for M in sc.chain["left"](a)])
            pr = np.array([M[:3, 3] for M in sc.chain["right"](b)])
            res = np.linalg.norm(pl * np.array([-1.0, 1.0, 1.0]) - pr, axis=1)
            # sum of squares drives every link; the max term stops it trading
            # one bad link for eight slightly better ones
            return (c + w * float(np.sum(res ** 2))
                    + w * float(res.max() ** 2))
        return f

    # A SCHEDULE, NOT ONE WEIGHT. Too little and symmetry never moves; too
    # much and the optimiser buys symmetry by walking out of the constraints
    # and the result is thrown away. The first run used a single weight of 6
    # and returned no improvement at all, which is indistinguishable from
    # "already optimal" unless more than one weight is tried.
    best = (ch_old, np.asarray(ql, float), np.asarray(qr, float))
    for w in (0.5, 2.0, 6.0, 20.0, 60.0):
        v0 = np.concatenate([best[1], best[2]])
        try:
            r = minimize(make(w), v0, method="L-BFGS-B", bounds=bounds,
                         options=dict(maxiter=iters, ftol=1e-14, eps=1e-7))
        except Exception:                                         # noqa: BLE001
            continue
        out_l, out_r = np.array(r.x[:7]), np.array(r.x[7:])
        # THE JOINT VECTOR HAS TO GO IN, AND IT DID NOT.
        #
        # `Target.check(t)` with no `q` sets `seam_ok = True` and moves on --
        # it cannot see (h), because (h) is about JOINTS and `t` is about
        # geometry. So the polish was free to walk a continuous joint onto the
        # +/-pi seam and still report every constraint met. Measured: the
        # |x| = 0.740 solve came back with the right arm 0.2963 rad from the
        # wrap point against a 0.30 limit, accepted here, and only caught by
        # `report_pair` at the very end -- which is after the pose has been
        # chosen.
        #
        # This is the same shape as the (g) bug three lines down, which is
        # already commented: a check that scores a SUBSET of the constraints
        # and reads as if it scored all of them.
        okl = all(v[0] for v in tg.check(sc.arm_terms("left", out_l),
                                         out_l).values())
        okr = all(v[0] for v in tg.check(sc.arm_terms("right", out_r),
                                         out_r).values())
        ee_new, ch_new, _ = sc.mirror(out_l, out_r)
        # (g) IS A CONSTRAINT ON THE PAIR AND HAS TO BE CHECKED HERE.
        # tg.check() scores one arm at a time, so a polish that improved the
        # whole-chain residual by pulling the two end effectors apart passed
        # both per-arm checks and broke the constraint the brief actually
        # states. Measured: it returned a pose with 0.0353 m of end-effector
        # mirror error against a 0.020 m limit, and reported it as solved.
        # AND IT IS SOLVED INSIDE THE CONSTRAINT, like everything else here.
        # `SOLVE_MARGIN`'s whole rationale is that a hinge penalty parks the
        # optimiser exactly on the boundary; (g) had no margin, and the
        # |x| = 0.600 solve duly returned 0.0199 m against a 0.020 m limit --
        # 0.1 mm of room, which docs/ENGINEERING_LOG.md calls MARGINAL and not usable.
        ok_g = ee_new <= tg.mirror_tol - SOLVE_MARGIN["mirror"]
        if okl and okr and ok_g and ch_new < best[0]:
            best = (ch_new, out_l, out_r)
    return best[1], best[2], ch_old, best[0]


# ------------------------------------------------------- the physical arms
#
# TWO DIFFERENT POSES IN THIS REPOSITORY BOTH DESCRIBE "THE REAL ARM", and the
# brief names the one that is NOT the home. They are both real and they are
# not the same thing:
#
#   LEGACY_HOME   Kortex 259.03 ... (left) / 303.65 ... (right). The pose the
#                 arms are PARKED at, recorded in findings.md 2026-07-31, and
#                 the pose `sim_to_real_bridge.enable()` compares the live arm
#                 against. Eight files carry it, including
#                 test_bridge_refuses_the_home_change.py.
#   LIVE_READING  Kortex 276.99 ... , LEFT ARM ONLY, the single reading ever
#                 taken off hardware, at 18:21 on 2026-07-31. It is where the
#                 arm HAPPENED to be at that instant, not where it homes, and
#                 `config/real_home_reference.txt` says in its own header that
#                 no code reads it. The right arm has never been read at all.
#
# Both are scored below, because "what pose do the real arms hold" has two
# defensible answers and quoting one without the other is how this project
# ends up with a number nobody can reproduce.
LEGACY_HOME_KORTEX = {
    "left": [259.03, 277.69, 267.74, 286.14, 194.10, 27.48, 55.26],
    "right": [303.65, 77.06, 98.57, 58.57, 317.14, 36.39, 154.71]}
LIVE_READING_KORTEX = {
    "left": [276.99, 309.93, 22.32, 309.49, 44.60, 342.63, 147.39]}


def kortex_to_ros(degs):
    from srl_teleop.kortex_convention import kortex_list_to_ros
    return np.array(kortex_list_to_ros(degs))


def joint_distance(qa, qb, continuous=(0, 2, 4, 6)):
    """Per-joint shortest move, in radians. Continuous joints may wrap."""
    from srl_teleop.kortex_convention import wrap_rad_pi
    out = []
    for i, (t, c) in enumerate(zip(qa, qb)):
        d = float(t) - float(c)
        out.append(wrap_rad_pi(d) if i in continuous else d)
    return np.array(out)


def score_real(sc, tg, result, verbose=True):
    """(a)-(g) for each pose the hardware is on record as holding."""
    out = {}
    for name, table in (("legacy_home", LEGACY_HOME_KORTEX),
                        ("live_reading_20260731", LIVE_READING_KORTEX)):
        qs = {a: kortex_to_ros(v) for a, v in table.items()}
        if "right" not in qs:
            # The right arm has never been read. Scoring it against the left
            # arm's numbers would invent data, so it is left absent and said
            # to be absent.
            terms = {"left": sc.arm_terms("left", qs["left"])}
            chk = {"left": tg.check(terms["left"])}
            out[name] = dict(
                right="NEVER READ FROM HARDWARE",
                q_left=[round(float(v), 6) for v in qs["left"]],
                left=terms["left"],
                checks={k: dict(ok=bool(v[0]), got=round(float(v[1]), 5),
                                want=v[2]) for k, v in chk["left"].items()})
        else:
            tl = sc.arm_terms("left", qs["left"])
            tr = sc.arm_terms("right", qs["right"])
            ee, ch, _ = sc.mirror(qs["left"], qs["right"])
            # ql/qr never existed -- the joint vectors live in `qs`. Passing
            # them matters: `check` derives the SEAM MARGIN (h) from q, and
            # with q=None it silently reports nan and PASSES. So this line
            # both crashed the two-arm branch and, had it not crashed, would
            # have scored the seam constraint as satisfied without looking.
            cl = tg.check(tl, qs["left"])
            cr = tg.check(tr, qs["right"])
            out[name] = dict(
                q_left=[round(float(v), 6) for v in qs["left"]],
                q_right=[round(float(v), 6) for v in qs["right"]],
                left=tl, right=tr, mirror_ee_m=ee,
                mirror_chain_worst_m=ch,
                checks={k: dict(want=cl[k][2],
                                left=round(float(cl[k][1]), 5),
                                right=round(float(cr[k][1]), 5),
                                ok=bool(cl[k][0] and cr[k][0]))
                        for k in cl})
        if verbose:
            print("\n   %s" % name.upper().replace("_", " "))
            d = out[name]
            if "checks" in d and "right" in d.get("checks",
                                                  {}).get("f_clearance", {}):
                for k, v in d["checks"].items():
                    print("      %s%-22s %-18s %10.4f %10.4f"
                          % ("ok " if v["ok"] else "XX ", k, v["want"],
                             v["left"], v["right"]))
                print("      %-25s %-18s %10.4f"
                      % ("   g_mirror_ee", "<= 0.020 m", d["mirror_ee_m"]))
                print("      %-25s %-18s %10.4f"
                      % ("   g_mirror_whole_chain", "(reported)",
                         d["mirror_chain_worst_m"]))
            else:
                print("      RIGHT ARM: %s" % d.get("right"))
                for k, v in d["checks"].items():
                    print("      %s%-22s %-18s %10.4f"
                          % ("ok " if v["ok"] else "XX ", k, v["want"],
                             v["got"]))
    result["real_arms"] = out
    return out


def recapture_table(q_left, q_right):
    """The solved pose in the units the lab capture needs."""
    from srl_teleop.kortex_convention import ros_list_to_kortex
    rows = {}
    for arm, q in (("left", q_left), ("right", q_right)):
        rows[arm] = dict(
            ros_rad=[round(float(v), 6) for v in q],
            ros_deg=[round(math.degrees(float(v)), 2) for v in q],
            kortex_deg=[round(v, 2) for v in ros_list_to_kortex(q)])
    return rows


# -------------------------------------------------------------- writing it
#
# THE POSE LIVES IN MORE PLACES THAN ANYONE NAMES. `test_home_has_one_source`
# knows about four (the two text files and the URDF's two initial_positions
# blocks) and it is right that those are the ones that drive an arm. Two more
# have to move WITH them and neither is in that test:
#
#   master_calibration.WORKSPACE_CENTRE   documented in its own file as
#                                         `offset = P_HOME`, the world point
#                                         the master's rest maps to. Left
#                                         behind, the first commanded teleop
#                                         frame lands wherever the OLD home
#                                         was.
#   srl_moveit_config/config/             a SIXTH copy, all fourteen joints at
#   initial_positions.yaml                ZERO, feeding a duplicate
#                                         ros2_control block. See the note in
#                                         the session report: it is inert only
#                                         because it loads last.
#
# WORKSPACE_ORIENT is deliberately NOT touched. docs/ENGINEERING_LOG.md hard constraint 1
# and home_wrist_is_real.md section 3 both say so, with the measurement:
# re-deriving the anchor to match a level home costs T2 its right arm.
def apply_pose(ql, qr, hand_xyz=None, dry_run=False, sc=None):
    """Write the solved pose into every place that carries it.

    `WORKSPACE_CENTRE` is taken from FORWARD KINEMATICS of the pose being
    written, per arm, not from the nominal target the sweep asked for. The two
    differ by a few millimetres and the constant is documented as
    `offset = P_HOME` -- where the arm ACTUALLY rests.
    """
    import re
    edits = []
    if sc is None:
        sc = Scorer()
    # THE TEXT FILE IS THE SOURCE, SO THE URDF IS DERIVED FROM WHAT THE TEXT
    # FILE ACTUALLY STORES, NOT FROM THE SOLVER'S FULL-PRECISION ANSWER.
    #
    # Rounding each carrier independently -- 2 decimals of degrees in the text
    # file, 4 decimals of radians in the URDF -- lets them drift by up to
    # 1.4e-4 rad. Measured: the first write of this pose put the right arm
    # 1.23e-4 rad from its own source, and `measure_home_render`'s control
    # (which asks for better than 1e-4) refused the measurement. The
    # one-source test's tolerance is 1e-3 and would never have seen it.
    #
    # So the degrees are rounded ONCE, and every other representation is
    # computed from the rounded value. The two carriers now agree to ~1e-6.
    deg = {arm: [round(math.degrees(v), 2) for v in q]
           for arm, q in (("left", ql), ("right", qr))}
    rad = {arm: [math.radians(v) for v in d] for arm, d in deg.items()}
    # P_HOME is read off the pose that will actually be STORED, not the
    # solver's unrounded answer, so WORKSPACE_CENTRE names where the arm rests.
    hands = {arm: sc.arm_terms(arm, rad[arm], with_clearance=False)["hand"]
             for arm in ("left", "right")}

    for arm in ("left", "right"):
        path = os.path.join(ROOT, "config/home_positions_%s.txt" % arm)
        txt = open(path).read()
        for i in range(7):
            txt = re.sub(r"(?m)^joint_%d:\s*[-+0-9.]+\s*$" % (i + 1),
                         "joint_%d: %.2f" % (i + 1, deg[arm][i]), txt)
        edits.append((path, txt))

    path = os.path.join(ROOT, "src/srl_description/urdf/srl_dual.urdf.xacro")
    txt = open(path).read()
    for arm in ("left", "right"):
        want = ("${dict(" + ",".join("joint_%d=%.6f" % (i + 1, rad[arm][i])
                                     for i in range(7)) + ")}")
        # the blocks appear left then right; replace them in that order
        m = list(re.finditer(r'initial_positions="\$\{dict\([^}]*\)\}"', txt))
        idx = 0 if arm == "left" else 1
        s, e = m[idx].span()
        txt = txt[:s] + 'initial_positions="%s"' % want + txt[e:]
    edits.append((path, txt))

    path = os.path.join(ROOT, "src/srl_teleop/srl_teleop/master_calibration.py")
    txt = open(path).read()
    hl, hr = hands["left"], hands["right"]
    new = ('WORKSPACE_CENTRE= {"left":(%.4f,%.4f,%.4f), '
           '"right":(%.4f,%.4f,%.4f)}'
           % (hl[0], hl[1], hl[2], hr[0], hr[1], hr[2]))
    txt2 = re.sub(r"(?m)^WORKSPACE_CENTRE=.*$", new, txt, count=1)
    if txt2 == txt:
        raise RuntimeError("WORKSPACE_CENTRE not found in master_calibration")
    edits.append((path, txt2))

    for path, txt in edits:
        print("   %-58s %s" % (os.path.relpath(path, ROOT),
                               "(dry run)" if dry_run else "written"))
        if not dry_run:
            open(path, "w").write(txt)
    return [p for p, _ in edits]


# --------------------------------------------------------------------- main
def report_pair(sc, tg, ql, qr, label, verbose=True):
    tl = sc.arm_terms("left", ql)
    tr = sc.arm_terms("right", qr)
    ee, chain_worst, per = sc.mirror(ql, qr)
    cl, cr = tg.check(tl, ql), tg.check(tr, qr)
    rows = []
    for k in cl:
        okl, vl, want = cl[k]
        okr, vr, _ = cr[k]
        rows.append((k, want, vl, vr, okl and okr))
    rows.append(("g_mirror_ee", "<= %.3f m" % tg.mirror_tol, ee, ee,
                 ee <= tg.mirror_tol))
    rows.append(("g_mirror_whole_chain", "(reported)", chain_worst,
                 chain_worst, None))
    if verbose:
        print("\n%s" % label)
        print("   %-24s %-18s %12s %12s  %s"
              % ("constraint", "target", "left", "right", ""))
        for k, want, vl, vr, ok in rows:
            mark = "   " if ok is None else ("ok " if ok else "XX ")
            print("   %s%-21s %-18s %12.4f %12.4f"
                  % (mark, k, want, vl, vr))
    all_ok = all(r[4] for r in rows if r[4] is not None)
    return dict(rows=[dict(constraint=k, target=w, left=round(a, 5),
                           right=round(b, 5), ok=o)
                      for k, w, a, b, o in rows],
                clearance_detail={"left": sc.clearance_parts("left", ql),
                                  "right": sc.clearance_parts("right", qr)},
                left=tl, right=tr, mirror_ee_m=ee,
                mirror_chain_worst_m=chain_worst, mirror_per_link=per,
                all_ok=bool(all_ok))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, nargs="*", default=None,
                    help="hand |x| columns to try, innermost first")
    ap.add_argument("--y", type=float, default=0.34)
    ap.add_argument("--z", type=float, default=1.18)
    ap.add_argument("--restarts", type=int, default=140)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--floor", type=float, default=FLOOR)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--apply", action="store_true",
                    help="write the solved pose into every place that "
                         "carries it")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--seed-from", default=None,
                    help="a previous home_solved.json, used as extra seeds")
    ap.add_argument("--forearm-tol-deg", type=float, default=25.0)
    # (a) and (c2) are PRESENTATION constraints -- how the pose LOOKS, not
    # how far it is from the person. They are what stops the hands moving
    # outboard: bringing them out swings the forearm across the body's
    # forward axis. When the ask is distance from the wearer rather than a
    # tidy photograph, these are the two to spend.
    ap.add_argument("--approach-tol-deg", type=float, default=25.0,
                    help="(c2) how far off forward the wrist may face")
    ap.add_argument("--level-tol-deg", type=float, default=2.0,
                    help="(c) how far off level the approach axis may sit")
    # (f2). SEPARATE FROM --floor ON PURPOSE. --floor scores the guard's own
    # chain, which starts at the immobile mount and is therefore capped; this
    # one scores what the seven joints control. Asking for a "much larger
    # margin" is a request about (f2), and stating it as (f) would return
    # INFEASIBLE at every column for a reason that has nothing to do with the
    # pose.
    ap.add_argument("--moving-floor", type=float, default=None,
                    help="(f2) floor on the chain from shoulder_link outward")
    ap.add_argument("--pick", choices=("symmetric", "widest"),
                    default="symmetric",
                    help="which feasible column to take: the most symmetric "
                         "(the 2026-08-16 rule) or the one furthest from the "
                         "wearer")
    a = ap.parse_args()

    print("INSTRUMENT CHECKS")
    ok1, worst1, _ = fk_self_test(verbose=False)
    print("   FK vs live TF (home_render.json, 8 links)          %.6f m -> %s"
          % (worst1, "PASS" if ok1 else "FAIL"))
    ok2, worst2 = compiled_matches_urdf(verbose=False)
    print("   compiled FK vs URDF walk (50 random poses)         %.1e -> %s"
          % (worst2, "PASS" if ok2 else "FAIL"))
    if not (ok1 and ok2):
        print("REFUSING: the instrument is not cleared.")
        return 6

    sc = Scorer()
    rng = np.random.default_rng(a.seed)

    # ------------------------------------------------ control: shipped pose
    ref = json.load(open(os.path.join(
        ROOT, "recordings/baselines/home_render.json")))
    hq = {k: np.array(v) for k, v in ref["home_from_config_rad"].items()}
    tl = sc.arm_terms("left", hq["left"])
    tr = sc.arm_terms("right", hq["right"])
    eem, chm, _ = sc.mirror(hq["left"], hq["right"])

    # THE SCORER AGAINST LIVE TF, FOR WHATEVER POSE IS CURRENT.
    #
    # This used to compare against three hardcoded constants from the
    # 2026-08-15 pose (elbow 0.0793 / 0.2715, chain mirror 0.2826). That was a
    # real known-answer test right up until the pose changed, at which point it
    # failed for the one reason a control must never fail: its reference moved.
    #
    # What it should assert is that THIS FILE'S geometry agrees with what the
    # booted stack measured off /tf, whichever pose that was. So it now
    # compares like for like, per arm, against the quantities
    # `measure_home_render.py` recorded. It is a stronger check than the
    # constants were, and it cannot go stale.
    repro, repro_worst = True, 0.0
    for arm, t in (("left", tl), ("right", tr)):
        rec = ref["arms"].get(arm, {})
        for key, mine, tol in (
                ("elbow_below_shoulder_m", t["elbow_below_shoulder_m"], 1e-3),
                ("camera_above_fingers_m", t["camera_above_fingers_m"], 1e-3),
                ("forearm_off_forward_deg", t["forearm_off_forward_deg"],
                 5e-2),
                ("tool_elevation_deg", t["approach_elevation_deg"], 5e-2)):
            if key not in rec:
                continue          # an older render file; nothing to compare
            d = abs(float(rec[key]) - float(mine))
            repro_worst = max(repro_worst, d / tol)
            if d > tol:
                repro = False
                print("      MISMATCH %s %s: recorded %.4f, computed %.4f"
                      % (arm, key, float(rec[key]), float(mine)))
        if "hand" in rec:
            d = float(np.linalg.norm(np.array(rec["hand"])
                                     - np.array(t["hand"])))
            repro_worst = max(repro_worst, d / 1e-3)
            if d > 1e-3:
                repro = False
    print("   this scorer vs the live TF the render measured     %s"
          % ("PASS" if repro else "FAIL"))
    selfm = sc.self_mirror("left", hq["left"])
    ctl_mirror = selfm > 0.10
    print("   mirror test can be non-zero (left vs itself)       %.4f m -> %s"
          % (selfm, "PASS" if ctl_mirror else "FAIL"))
    ok3, worst3 = vectorised_clearance_matches_the_guard(sc)

    # THE CEILING ON (f), MEASURED. Stated before the sweep, because every
    # "INFEASIBLE -- f_clearance" line below is explained by it and would
    # otherwise read as a fact about the pose.
    caps = {arm: sc.mount_cap(arm) for arm in ("left", "right")}
    print("   the immobile mount clears the wearer by            "
          "L %.4f m / R %.4f m" % (caps["left"], caps["right"]))
    print("      -> no home pose can score (f) above that. docs/ENGINEERING_LOG.md's "
          "0.1610 m is stale (the mounts moved 2026-08-18).")
    # A CONTROL ON THE NEW METRIC, because a number that can only go up is
    # not a measurement. The moving chain is a SUBSET of the whole chain, so
    # its worst case can never be smaller -- checked over random poses, both
    # arms, rather than argued from the definition.
    rngc = np.random.default_rng(23)
    subset_ok, subset_worst = True, 0.0
    for arm in ("left", "right"):
        lo_c, hi_c, _ = sc.lim[arm]
        for _ in range(12):
            qc = rngc.uniform(lo_c, hi_c)
            w_all, _ = sc.clearance(arm, qc)
            w_mov, _ = sc.clearance(arm, qc, MOVING_FIRST_SEG)
            subset_worst = max(subset_worst, w_all - w_mov)
            if w_mov < w_all - 1e-12:
                subset_ok = False
    print("   moving-chain clearance >= whole-chain, 24 poses     "
          "%.1e -> %s" % (subset_worst, "PASS" if subset_ok else "FAIL"))
    if not (repro and ctl_mirror and ok3 and subset_ok):
        print("REFUSING: a control failed.")
        return 6

    tg0 = Target(0.55, 0.36, 1.18, floor=a.floor,
                 moving_floor=a.moving_floor,
                 forearm_tol_deg=a.forearm_tol_deg)
    shipped = report_pair(sc, tg0, hq["left"], hq["right"],
                          "THE SHIPPED HOME, scored against (a)-(g)")

    result_real = {}
    print("\nWHAT THE PHYSICAL ARMS HOLD, scored against (a)-(g)")
    print("   (|x| is not constrained here -- these poses were never chosen "
          "to satisfy it)")
    tg_real = Target(0.55, 0.36, 1.18, floor=a.floor,
                 moving_floor=a.moving_floor,
                 forearm_tol_deg=a.forearm_tol_deg)
    score_real(sc, tg_real, result_real)

    if a.self_test:
        print("\nSELF-TEST: a target demanded INSIDE the torso must fail")
        bad = Target(0.05, 0.0, 1.22, floor=a.floor,
                 moving_floor=a.moving_floor,
                 forearm_tol_deg=a.forearm_tol_deg)
        seeds = seed_set(sc, "left", 40, rng)
        got = solve_arm(sc, "left", bad, seeds)
        t = sc.arm_terms("left", got[0][1])
        chk = bad.check(t)
        nfail = sum(0 if v[0] else 1 for v in chk.values())
        print("   best cost %.4f, %d of %d constraints unmet, clearance "
              "%.4f m -> %s" % (got[0][0], nfail, len(chk), t["clearance_m"],
                                "PASS" if nfail > 0 else "FAIL"))
        if nfail == 0:
            print("   REFUSING: the solver returned a pose inside the person.")
            return 6

    # ------------------------------------------------------------- the sweep
    xs = a.x if a.x else [round(TORSO_HALF_W + 0.02 * i, 3)
                          for i in range(0, 20)]
    print("\nSWEEP: how far IN can the hands come with (a)-(g) all satisfied?")
    print("   torso half-width %.2f m, torso front face y = %.2f m"
          % (TORSO_HALF_W, TORSO_FRONT_Y))
    print("   solved OUTER to INNER, so the rows print widest first")
    print("   %-7s %-9s %-9s %-9s %-9s %-9s %-9s %-8s %s"
          % ("|x|", "L elbow", "R elbow", "L cam deg", "R cam deg", "clear",
             "movclear", "mirrorEE", "all"))
    # SWEPT OUTER TO INNER, CARRYING SOLUTIONS FORWARD.
    #
    # A wide column is easy and a narrow one is hard, and the hard one's
    # answer is a small perturbation of the easy one's. Seeding each column
    # with the previous column's branches finds postures that random restarts
    # from scratch do not, and it is why "infeasible" below can be believed:
    # the search reaches each column having already solved its neighbour.
    # Results are still reported innermost first.
    prior = {arm: priors_from(a.seed_from, arm) for arm in ("left", "right")}
    sweep, best, carry = [], None, {"left": [], "right": []}
    for x in sorted(xs, reverse=True):
        tg = Target(x, a.y, a.z, floor=a.floor,
                 moving_floor=a.moving_floor,
                 forearm_tol_deg=a.forearm_tol_deg,
                 approach_tol_deg=a.approach_tol_deg,
                 level_tol_deg=a.level_tol_deg)
        sl = solve_arm(sc, "left", tg,
                       carry["left"] + seed_set(
                           sc, "left", a.restarts, rng,
                           prior["left"].get(round(x, 3))))
        sr = solve_arm(sc, "right", tg,
                       carry["right"] + seed_set(
                           sc, "right", a.restarts, rng,
                           prior["right"].get(round(x, 3))))
        carry = {"left": [q for _, q in sl], "right": [q for _, q in sr]}
        if not sl or not sr:
            sweep.append(dict(x=x, ok=False, why="no solution"))
            continue
        # choose the PAIR, not two independent winners: symmetry is a property
        # of the pair, and picking each arm's own best is what left the last
        # attempt 0.2166 m from mirroring.
        pick = None
        # THE JOINT VECTOR GOES IN HERE TOO. `Target.check(t)` with no `q`
        # scores nine constraints and silently passes the tenth: (h) is about
        # JOINTS and `t` carries only geometry. So the pair chosen for this
        # column was chosen without (h) ever being evaluated, and the first
        # thing that looked at it was `report_pair`, after the pose had been
        # picked, polished and written. Measured: the |x| = 0.780 row was
        # accepted at 0.3002 rad from the wrap point. Same defect as the one
        # already commented in `refine_pair`; it lived in two places.
        for _, ql in sl:
            fl = tg.check(sc.arm_terms("left", ql), ql)
            if not all(v[0] for v in fl.values()):
                continue
            for _, qr in sr:
                fr = tg.check(sc.arm_terms("right", qr), qr)
                if not all(v[0] for v in fr.values()):
                    continue
                ee, ch, _ = sc.mirror(ql, qr)
                key = (round(ee, 4), round(ch, 4))
                if pick is None or key < pick[0]:
                    pick = (key, ql, qr, ee, ch)
        if pick is None:
            # nothing fully feasible: report the least-bad pair so the
            # shortfall is a number rather than a silence
            ql, qr = sl[0][1], sr[0][1]
            r = report_pair(sc, tg, ql, qr, "", verbose=False)
            unmet = [d["constraint"] for d in r["rows"]
                     if d["ok"] is False]
            sweep.append(dict(x=x, ok=False, unmet=unmet,
                              detail=r))
            # SAY BY HOW MUCH, not just which. A constraint reported as unmet
            # with no number is the same silence this project keeps finding.
            shown = ", ".join(
                "%s %.4f (want %s)" % (d["constraint"], d["left"], d["target"])
                for d in r["rows"] if d["ok"] is False)
            print("   %-7.3f  INFEASIBLE -- %s" % (x, shown))
            continue
        _, ql, qr, ee, ch = pick
        tl2, tr2 = sc.arm_terms("left", ql), sc.arm_terms("right", qr)
        ok = ee <= tg.mirror_tol
        mov = min(tl2["clearance_moving_m"], tr2["clearance_moving_m"])
        print("   %-7.3f %+9.4f %+9.4f %+9.4f %+9.4f %9.4f %9.4f %8.4f %s"
              % (x, tl2["elbow_below_shoulder_m"],
                 tr2["elbow_below_shoulder_m"],
                 tl2["camera_off_top_deg"], tr2["camera_off_top_deg"],
                 min(tl2["clearance_m"], tr2["clearance_m"]), mov, ee,
                 "yes" if ok else "no (mirror)"))
        row = dict(x=x, ok=bool(ok), mirror_ee_m=ee,
                   mirror_chain_worst_m=ch,
                   clearance_m=min(tl2["clearance_m"], tr2["clearance_m"]),
                   clearance_moving_m=mov,
                   q_left=[round(float(v), 6) for v in ql],
                   q_right=[round(float(v), 6) for v in qr])
        sweep.append(row)
        if ok:
            best = (x, ql, qr)   # swept inward, so the last feasible is
            #                      the innermost

    result = dict(controls=dict(
        fk_vs_live_tf_m=round(worst1, 6),
        compiled_fk_vs_urdf=worst2,
        shipped_reproduces_render=bool(repro),
        mirror_can_be_nonzero_m=round(selfm, 4)),
        torso_half_width_m=TORSO_HALF_W, torso_front_y_m=TORSO_FRONT_Y,
        mount_cap_m={k: round(v, 4) for k, v in caps.items()},
        floor_m=a.floor, moving_floor_m=a.moving_floor, pick=a.pick,
        hand_y=a.y, hand_z=a.z,
        shipped=shipped, real_arms=result_real.get("real_arms"),
        sweep=sorted(sweep, key=lambda r: r["x"]))

    if best is None:
        print("\nNO COLUMN SATISFIES EVERY CONSTRAINT in the range swept.")
        result["solved"] = None
    else:
        x, ql, qr = best
        print("\nINNERMOST COLUMN THAT SATISFIES EVERY CONSTRAINT: |x| = %.3f m"
              % x)

        # WHICH FEASIBLE COLUMN TO TAKE, and the rule is stated rather than
        # assumed. "Hands at roughly torso width" is NOT satisfiable at any
        # column (see the sweep), so choosing the innermost one buys a few
        # millimetres of something already conceded. "The two arms exact
        # mirrors" IS one of the constraints, and the whole-chain residual is
        # the only number that tracks what the render shows. So: among the
        # feasible columns, take the most symmetric, and break ties inward.
        cands = [r for r in sweep if r.get("ok")]
        if a.pick == "widest":
            # FURTHEST FROM THE WEARER, and the tie-break is symmetry.
            # Rounded to the millimetre before sorting: two columns that
            # differ by 0.2 mm of clearance are the same answer, and letting
            # that decide would pick on solver noise rather than geometry.
            cands.sort(key=lambda r: (-round(r["clearance_moving_m"], 3),
                                      round(r["mirror_chain_worst_m"], 4),
                                      -r["x"]))
        else:
            cands.sort(key=lambda r: (round(r["mirror_chain_worst_m"], 4),
                                      r["x"]))
        short = [r for r in cands[:3]]
        if not any(r["x"] == x for r in short):
            short.append([r for r in cands if r["x"] == x][0])
        print("\n   SYMMETRY POLISH on the %d most promising feasible columns"
              % len(short))
        polished = []
        for r in short:
            tgr = Target(r["x"], a.y, a.z, floor=a.floor,
                         moving_floor=a.moving_floor,
                         forearm_tol_deg=a.forearm_tol_deg,
                         approach_tol_deg=a.approach_tol_deg,
                         level_tol_deg=a.level_tol_deg)
            pl, pr, cb, ca = refine_pair(sc, tgr, np.array(r["q_left"]),
                                         np.array(r["q_right"]), iters=300)
            print("      |x| = %.3f   whole-chain mirror %.4f -> %.4f m"
                  % (r["x"], cb, ca))
            mv = min(sc.arm_terms("left", pl)["clearance_moving_m"],
                     sc.arm_terms("right", pr)["clearance_moving_m"])
            polished.append((ca, r["x"], pl, pr, mv))
        # THE POLISH OPTIMISES SYMMETRY AND CAN SPEND CLEARANCE BUYING IT.
        # Under --pick widest the clearance is the thing being asked for, so
        # it sorts first here too; refine_pair already refuses any polish that
        # breaks a constraint, so this only chooses among legal ones.
        if a.pick == "widest":
            polished.sort(key=lambda t: (-round(t[4], 3), round(t[0], 4)))
        else:
            polished.sort(key=lambda t: (round(t[0], 4), t[1]))
        ch_after, x, ql, qr, _mv = polished[0]
        print("   TAKING |x| = %.3f m, whole-chain mirror residual %.4f m"
              % (x, ch_after))
        tgx = Target(x, a.y, a.z, floor=a.floor,
                     moving_floor=a.moving_floor,
                     forearm_tol_deg=a.forearm_tol_deg)
        det = report_pair(sc, tgx, ql, qr, "THE SOLVED HOME POSE")
        result["solved"] = dict(
            hand_abs_x_m=x, hand_y=a.y, hand_z=a.z,
            q_left=[round(float(v), 6) for v in ql],
            q_right=[round(float(v), 6) for v in qr],
            detail=det)
        tabs = recapture_table(ql, qr)
        result["solved"]["recapture"] = tabs
        print("\n   THE SOLVED POSE, in the three units the lab needs")
        for arm in ("left", "right"):
            print("      %-6s ROS rad    %s"
                  % (arm, " ".join("%+.6f" % v for v in tabs[arm]["ros_rad"])))
            print("      %-6s ROS deg    %s"
                  % ("", " ".join("%+8.2f" % v for v in tabs[arm]["ros_deg"])))
            print("      %-6s Kortex deg %s"
                  % ("", " ".join("%8.2f" % v
                                  for v in tabs[arm]["kortex_deg"])))

        # HOW FAR IS THIS FROM WHERE THE METAL IS
        print("\n   JOINT-SPACE DISTANCE from the poses the hardware holds")
        dist = {}
        for name, table in (("legacy_home", LEGACY_HOME_KORTEX),
                            ("live_reading_20260731", LIVE_READING_KORTEX)):
            for arm, sol in (("left", ql), ("right", qr)):
                if arm not in table:
                    continue
                d = joint_distance(sol, kortex_to_ros(table[arm]))
                dist["%s.%s" % (name, arm)] = dict(
                    per_joint_rad=[round(float(v), 4) for v in d],
                    worst_rad=round(float(np.abs(d).max()), 4),
                    worst_joint=int(np.argmax(np.abs(d))) + 1,
                    worst_deg=round(math.degrees(float(np.abs(d).max())), 2))
                print("      %-28s %-5s worst %+.4f rad (%.1f deg) on joint_%d"
                      % (name, arm, np.abs(d).max(),
                         math.degrees(np.abs(d).max()),
                         int(np.argmax(np.abs(d))) + 1))
        result["solved"]["distance_from_hardware"] = dist
        print("      the bridge's require_homed tolerance is 0.05 rad, so "
              "every one of these refuses until the arms are recaptured")

        if a.apply or a.dry_run:
            print("\n   WRITING THE POSE")
            apply_pose(ql, qr, dry_run=a.dry_run, sc=sc)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(result, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
