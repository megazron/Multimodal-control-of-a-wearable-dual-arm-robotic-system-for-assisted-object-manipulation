#!/usr/bin/env python3
"""THE PRESENTATION POSE. Derived, checked, and saved. Closes R-7.

    python3 scripts/find_presentation_pose.py --save     # needs the stack

WHY IT DOES NOT ALREADY EXIST, and it is not an oversight. Every clip opens on
the home pose, whose wrist points UP by +85 deg (left) and +79 deg (right).
That is REAL -- read from the physical arms, see
`docs/system/home_wrist_is_real.md` -- and it is not a rendering fault, so it
must not be "fixed" by editing the home angles. HARD CONSTRAINT 1: the sim to
real bridge replays sim angles onto the real arm and any difference is
commanded as a jump.

Nor can the pose come from the task path. The clip runner publishes END
EFFECTOR POSITIONS and the follower PINS the orientation, so a level wrist is
not expressible there at all: there is no position you can command that
changes where the hand points.

SO IT IS A JOINT-SPACE STAGING MOVE, commanded to the arm controllers before
capture starts. That is legitimate for the same reason the scan pose is
legitimate: it is one posture, taken before the trial, with no operator in the
loop, and it is not teleoperation. It is not part of any task and produces no
trial data.

HOW THE ANGLE IS FOUND. Not guessed. Home is the start; one wrist joint is
swept; for each candidate the TOOL AXIS ELEVATION is read from forward
kinematics, and the pose is also required to be collision free against the
same planning scene everything else uses -- wearer included. The candidate
whose tool axis is closest to level, and which is valid, wins.

THREE CONTROLS, because a search over poses that all return the same number
looks exactly like a search that worked:

  * FK MUST RESPOND TO THE STATE IT IS GIVEN. Two different joint vectors
    must produce two different tool axes. This project has already been
    bitten by an FK path that evaluated the CURRENT state and ignored the
    solution handed to it, which made every pose return one value.
  * FK AND TF MUST AGREE ABOUT THE HOME POSE. The arm IS at home while this
    runs, so the tool axis can be measured two independent ways -- through
    /compute_fk from the joint vector, and straight off the TF tree. If they
    disagree the FK path is measuring something other than this robot.
  * A POSE DRIVEN INTO THE WEARER MUST BE REJECTED. Constructed by driving
    joint_2 until the geometric clearance actually goes negative, so the
    control tests the checker rather than testing my guess about geometry.

WHY THE CLEARANCE IS MEASURED HERE AND NOT ASKED OF MoveIt. HARD CONSTRAINT 11
and `mount_guard_node`: a mount rotation once buried both arm bases 75 mm
inside the wearer's torso and `/check_state_validity` returned valid=True,
because the offending pairs are SRDF-excluded. An exclusion silences the alarm;
it does not move the metal. So the wearer check here is the same GEOMETRIC one
the mount guard uses -- the arm as a chain of capsules against the wearer's
primitives, no exclusions -- evaluated on FK for the candidate posture.
/check_state_validity is still asked, for self-collision, but it is not the
authority on the wearer.

WHAT +85 DEGREES IS NOT. CLAUDE.md records the home wrist as pointing up by
+85 (left) and +79 (right), and this script measures the home TOOL AXIS at
about +31 (left) and +22 (right). Those are not in conflict and the difference
is not an error: they are DIFFERENT QUANTITIES, exactly as
`home_wrist_is_real.md` says -- the anchor's ~30.7 deg is measured from the
shoulder and is the elevation of the approach axis, which is the number this
script needs. Comparing the two was the first control here and it failed for
that reason alone.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "srl_teleop"))
from srl_teleop import mount_guard_node as MG                  # noqa: E402

OUT = os.path.join(ROOT, "recordings", "baselines", "presentation_pose.json")

# The wrist joints, in the order they are tried. joint_6 is the wrist bend on
# a Gen3 and is the one that changes where the tool points without swinging
# the whole forearm; joint_5 is the fallback if it cannot get there alone.
WRIST_JOINTS = (6, 5)
SWEEP_DEG = 120.0            # each side of home
STEP_DEG = 2.0
# "Level" is the goal, but the pose also has to stay somewhere a camera can
# see and a person is not. 8 degrees of residual elevation is well under the
# 30.7 deg the pinned wrist sits at and far under home's +85.
GOOD_ENOUGH_DEG = 8.0
# FK and TF must agree about the home tool axis to within this. It is a
# comparison of two instruments on one pose, so the tolerance is tight.
FK_TF_TOL_DEG = 1.0
# The clearance floor, the same 0.15 m the mount guard and the follower use.
# HARD CONSTRAINT 11: it is the last thing between the arms and a person's
# chest and it is not lowered for a staging pose.
MIN_CLEARANCE_M = 0.15


class Kin(Node):
    def __init__(self):
        super().__init__("presentation_pose_finder")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.sv = self.create_client(GetStateValidity,
                                     "/check_state_validity")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def spin(self, secs):
        t = time.time()
        while time.time() - t < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.005)
        return fut.result()

    def names(self, arm):
        return ["%s_joint_%d" % (arm, i + 1) for i in range(7)]

    def _state(self, arm, q):
        """The FULL joint state, with the OTHER arm at its measured position.

        Sending only seven joints leaves MoveIt to fill the rest from its own
        default, so a collision check would be run against an arm that is not
        where this one is. That is the same class of error as checking the
        clearance of the wrong arm.
        """
        rs = RobotState()
        name, pos = list(self.names(arm)), [float(v) for v in q]
        if self.js is not None:
            for n, p in zip(self.js.name, self.js.position):
                if n not in name:
                    name.append(n)
                    pos.append(float(p))
        rs.joint_state.name = name
        rs.joint_state.position = pos
        return rs

    def tool_axis(self, arm, q):
        """The gripper approach axis in world, and its elevation in degrees.

        FK is asked about the state PASSED IN. The control below proves it
        actually is: two different vectors must give two different answers.
        """
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_end_effector_link" % arm]
        req.robot_state = self._state(arm, q)
        res = self._call(self.fk, req)
        if res is None or not res.pose_stamped or res.error_code.val != 1:
            return None, None
        o = res.pose_stamped[0].pose.orientation
        x, y, z, w = o.x, o.y, o.z, o.w
        # The tool's +z in world -- the direction the fingers point.
        ax = (2.0 * (x * z + w * y),
              2.0 * (y * z - w * x),
              1.0 - 2.0 * (x * x + y * y))
        horiz = math.hypot(ax[0], ax[1])
        return ax, math.degrees(math.atan2(ax[2], horiz))

    def tf_elevation(self, arm):
        """The tool axis read straight off TF, for the pose the arm is IN.

        A second, independent instrument for the same quantity. FK takes a
        joint vector and a model; this takes the transform tree the rest of
        the stack is using. They must agree at home.
        """
        try:
            t = self.tfb.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                      # noqa: BLE001
            return None
        o = t.transform.rotation
        x, y, z, w = o.x, o.y, o.z, o.w
        ax = (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
              1.0 - 2.0 * (x * x + y * y))
        return math.degrees(math.atan2(ax[2], math.hypot(ax[0], ax[1])))

    def clearance(self, arm, q):
        """Worst distance from the arm's capsule chain to the wearer, in m.

        The mount guard's model exactly: sample along the segment between
        consecutive link origins and inflate by the Gen3 tube radius. FK
        supplies the origins for the CANDIDATE posture, which is the only
        difference -- the guard reads them from TF for the posture the arm
        is in.
        """
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, ln) for ln in MG.CHAIN]
        req.robot_state = self._state(arm, q)
        res = self._call(self.fk, req)
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None, None
        pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
               for ps in res.pose_stamped]
        worst, who = 1e9, None
        for a, b in zip(pts, pts[1:]):
            for k in range(MG.SAMPLES + 1):
                t = k / float(MG.SAMPLES)
                p = [a[i] + (b[i] - a[i]) * t for i in range(3)]
                for name, kind, prm, ctr in MG.WEARER:
                    d = MG.dist_point(p, kind, prm, ctr) - MG.TUBE_R
                    if d < worst:
                        worst, who = d, name
        return worst, who

    def valid(self, arm, q):
        req = GetStateValidity.Request()
        req.robot_state = self._state(arm, q)
        req.group_name = "%s_arm" % arm
        res = self._call(self.sv, req)
        if res is None:
            return None, []
        return bool(res.valid), [(c.contact_body_1, c.contact_body_2)
                                 for c in res.contacts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()

    import home_positions as hp
    rclpy.init()
    n = Kin()
    n.spin(3.0)
    if not n.fk.wait_for_service(timeout_sec=20.0):
        print("no /compute_fk -- start the sim")
        return 2
    if not n.sv.wait_for_service(timeout_sec=20.0):
        print("no /check_state_validity -- start the sim")
        return 2

    result, controls = {}, {}
    for arm in ("left", "right"):
        home = list(hp.load_home_radians(arm))

        # ---- CONTROL 1: FK responds to the state it is given ---------
        _, e0 = n.tool_axis(arm, home)
        bent = list(home)
        bent[5] += math.radians(45.0)
        _, e1 = n.tool_axis(arm, bent)
        moved = (e0 is not None and e1 is not None
                 and abs(e1 - e0) > 1.0)
        controls["fk_responds_%s" % arm] = moved

        # ---- CONTROL 2: FK and TF agree about the pose the arm is IN --
        tf_elev = n.tf_elevation(arm)
        agree = (e0 is not None and tf_elev is not None
                 and abs(e0 - tf_elev) <= FK_TF_TOL_DEG)
        controls["fk_matches_tf_%s" % arm] = agree
        controls["home_elevation_%s_deg" % arm] = (
            round(e0, 2) if e0 is not None else None)
        controls["home_elevation_tf_%s_deg" % arm] = (
            round(tf_elev, 2) if tf_elev is not None else None)

        # ---- home's own clearance, and CONTROL 3 -----------------------
        home_clear, home_who = n.clearance(arm, home)
        controls["home_clearance_%s_m" % arm] = (
            round(home_clear, 4) if home_clear is not None else None)

        # A pose driven INTO the wearer must be rejected. Swept rather than
        # guessed, so it is the checker being tested and not my geometry.
        crashed = None
        for v in [x * 0.2 for x in range(-16, 17)]:
            q = list(home)
            q[1] = v
            c, who = n.clearance(arm, q)
            if c is not None and c < 0.0:
                crashed = (v, c, who)
                break
        controls["wearer_collision_detected_%s" % arm] = crashed is not None
        controls["wearer_collision_example_%s" % arm] = (
            "joint_2=%.2f -> %.3f m into %s" % crashed if crashed
            else "NONE FOUND -- the clearance model never goes negative, so "
                 "it cannot be trusted to say a pose is clear")

        # ---- the search ------------------------------------------------
        #
        # AMONG THE POSES THAT ARE LEVEL ENOUGH, TAKE THE SMALLEST MOVE.
        # Taking the flattest instead picked joint_6 +120 deg on the right
        # arm for 0.2 deg of elevation, when a much smaller swing reaches
        # the same target -- and a staging move is travel before every clip,
        # over a person, so shorter is strictly better. Flatness is a
        # threshold here, not something to maximise.
        cands = []
        for j in WRIST_JOINTS:
            k = j - 1
            d = -SWEEP_DEG
            while d <= SWEEP_DEG + 1e-9:
                q = list(home)
                q[k] = home[k] + math.radians(d)
                _ax, elev = n.tool_axis(arm, q)
                if elev is not None and abs(elev) <= GOOD_ENOUGH_DEG:
                    clear, who = n.clearance(arm, q)
                    ok, contacts = n.valid(arm, q)
                    if clear is not None and clear >= MIN_CLEARANCE_M and ok:
                        cands.append(dict(joint=j, delta_deg=round(d, 2),
                                          elev=elev, q=q,
                                          clearance_m=round(clear, 4)))
                    else:
                        # A level pose that is too close to the wearer is a
                        # different answer from no level pose existing, so it
                        # is named rather than silently skipped.
                        print("   %s: joint_%d %+6.1f deg is level (%+.1f) "
                              "but clearance %s m to %s / self-collision %s"
                              % (arm, j, d, elev,
                                 "%.3f" % clear if clear is not None else "?",
                                 who, "yes" if ok is False else "no"))
                d += STEP_DEG
            if cands:
                break
        result[arm] = min(cands, key=lambda c: abs(c["delta_deg"])) \
            if cands else None

    print("=" * 72)
    print("PRESENTATION POSE -- a joint-space staging move, not a task pose")
    print("=" * 72)
    bad = 0
    for k in sorted(controls):
        if k.endswith("_deg") or k.endswith("_m") or "_example_" in k:
            continue
        print("   CONTROL %-28s %s" % (k, "OK" if controls[k] else "FAIL"))
        bad += not controls[k]
    for arm in ("left", "right"):
        b = result[arm]
        if b is None:
            print("   %-5s NO valid pose found" % arm)
            bad += 1
            continue
        print("   %-5s home tool axis %+.1f deg (TF %+.1f) -> %+.1f deg by "
              "joint_%d %+.1f deg"
              % (arm, controls["home_elevation_%s_deg" % arm],
                 controls["home_elevation_tf_%s_deg" % arm], b["elev"],
                 b["joint"], b["delta_deg"]))
        print("         wearer clearance %.4f m at the staging pose against "
              "%.4f m at home, floor %.2f"
              % (b["clearance_m"], controls["home_clearance_%s_m" % arm],
                 MIN_CLEARANCE_M))
        print("         %s" % controls["wearer_collision_example_%s" % arm])
        if abs(b["elev"]) > GOOD_ENOUGH_DEG:
            print("         WARNING: %.1f deg is outside the %.0f deg target"
                  % (abs(b["elev"]), GOOD_ENOUGH_DEG))

    if a.save and not bad:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        json.dump(dict(
            note="joint-space staging pose commanded before capture; NOT a "
                 "task pose and not teleoperation. Derived from home by "
                 "sweeping one wrist joint for a level tool axis, checked "
                 "against /check_state_validity with the wearer in scene.",
            good_enough_deg=GOOD_ENOUGH_DEG,
            controls={k: v for k, v in controls.items()},
            min_clearance_m=MIN_CLEARANCE_M,
            poses={arm: dict(joint=result[arm]["joint"],
                             delta_deg=result[arm]["delta_deg"],
                             elevation_deg=round(result[arm]["elev"], 2),
                             clearance_m=result[arm]["clearance_m"],
                             q=[round(v, 6) for v in result[arm]["q"]])
                   for arm in ("left", "right")}),
            open(OUT, "w"), indent=2)
        print("\nsaved -> %s" % OUT)
    elif a.save:
        print("\nNOT SAVED: %d control or search failure(s). A staging pose "
              "derived from an instrument that failed its own controls is "
              "worth less than no staging pose." % bad)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
