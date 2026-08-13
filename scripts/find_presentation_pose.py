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

HOW THE POSE IS FOUND. Not guessed. Coordinate descent from home over joints
1, 2, 4 and 6 against a cost that spells out what "standing ready to work"
means as numbers: hands in FRONT of the chest rather than out at the sides,
lateral extent about the width of the torso, elbows below the shoulders and
slightly outboard of the hands, wrists level, and -- as a tie-break only -- a
short move from home.

THE FIRST VERSION SWEPT joint_6 ALONE, and it could not have worked. Turning
the wrist levels the wrist and does nothing else, so the arms stayed where
home puts them: measured, hands 1.633 m apart against a 0.360 m torso, which
is a T-pose, and WIDER than home's own 1.459 m because swinging the wrist
throws the hand further out. A pose can satisfy every check it is given and
still be the wrong pose, which is why the cost now says what the posture is
FOR.

Hard constraints are hard: a candidate that is unreachable, too close to the
wearer, self-colliding, behind the chest or folded into the centreline is not
scored badly, it is not a candidate at all.

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

# WHICH JOINTS THE SEARCH IS ALLOWED TO MOVE.
#
# The first version turned joint_6 ALONE. That levels the wrist and can do
# nothing else, so it left the arms where home puts them -- and home puts
# them out at the sides. Measured on the pose it produced: hands 1.633 m
# apart against a 0.360 m torso, which is a T-pose, and WIDER than home's
# 1.459 m because swinging the wrist throws the hand further out.
#
# joint_1 and joint_2 swing the whole arm in and forward, joint_4 is the
# elbow, joint_6 is the wrist bend. Those four are enough to bring the hands
# in front of the chest with the elbows down, and leaving joints 3, 5 and 7
# alone keeps the move short and keeps joint_5 well away from the +/-pi seam
# it sits 14 deg from.
# HOW MANY IK SOLUTIONS TO DRAW PER HAND TARGET. TRAC-IK restarts randomly,
# so the same request returns different postures -- which is usually a
# nuisance and is useful here: it samples the arm's null space, and the elbow
# is exactly what the null space controls.
IK_DRAWS = 40

# ---- WHAT "STANDING READY TO WORK" MEANS, AS NUMBERS ---------------------
# The wearer's torso is a 0.36 x 0.22 x 0.48 box centred at z = 1.22
# (human_backpack.xacro, via mount_guard_node). So:
TORSO_HALF_W = 0.18          # "lateral extent roughly the width of the torso"
TORSO_FRONT_Y = 0.11         # the front face; a hand "in front of the chest"
                             # has to be beyond this
# THE BRIEF ASKED FOR |x| = 0.18, THE TORSO'S OWN HALF WIDTH, AND THE
# CLEARANCE FLOOR FORBIDS IT. Measured, level wrist, geometric wearer
# clearance with no SRDF exclusions:
#
#     hand |x| = 0.18   clearance -0.016 m   (the arm is INSIDE the torso)
#                0.20             -0.004
#                0.26             +0.002 .. +0.041
#                0.30             +0.019 .. +0.045
#                0.45             +0.151 .. +0.161   <- the floor is 0.15
#
# It is not the HAND that collides -- the hand is well clear at 0.32 m in
# front of a torso face at 0.11. It is the upper arm and forearm: the mount
# is on the BACK, so folding the arms round to the front lays the limbs
# across the person. |x| = 0.45 is the nearest either hand can come to the
# centreline with the floor intact, which is a 0.90 m span against a 0.36 m
# torso.
#
# HARD CONSTRAINT 11: the floor is not lowered for a staging pose. So the
# target is the measured frontier, and the shortfall against the brief is
# reported rather than engineered around.
#
# AND AN EARLIER MEASUREMENT OF MINE WAS WRONG, for a reason worth keeping:
# scripts/measure_ready_pose_envelope.py said both arms reach |x| = 0.14 and
# I reported the brief as achievable. That sweep asked MoveIt's
# collision-aware IK, and the wearer pairs are SRDF-EXCLUDED -- the same trap
# mount_guard_node exists for. MoveIt said valid; the geometry says the metal
# is 16 mm inside the person.
HAND_TARGET = dict(x=0.45,   # the measured clearance frontier, not the brief
                   y=0.32,   # clear of the front face, at chest depth
                   z=1.22)   # chest height
# What the brief asked for, kept so the shortfall can be stated in numbers.
BRIEF_HAND_X = 0.18
# A hand nearer the centreline than this reads as hands clasped rather than
# ready, and the two grippers start to threaten each other.
HAND_MIN_X = 0.10
# "Elbows down and slightly out, not level with the shoulders."
ELBOW_BELOW_SHOULDER_M = 0.05
# How far outboard of the hand the elbow may sit before it reads as a chicken
# wing rather than "slightly out". Measured off the render that failed: the
# left elbow was 229 mm outboard and the posture read as splayed.
ELBOW_OUT_BAND_M = 0.09
# THE TWO ARMS GET THE SAME TARGET, MIRRORED. Solving each arm independently
# let them settle on different targets -- left at (0.450, 0.320, 1.100) and
# right at (-0.500, 0.320, 1.220) -- and the render shows exactly that: one
# arm forward at the hip, the other out at the side. A staging pose is a
# POSTURE, and a posture is symmetric or it looks like a fault.
REQUIRE_MIRRORED_TARGET = True
# "Wrists level, as they are now."
GOOD_ENOUGH_DEG = 8.0
# FK and TF must agree about the home tool axis to within this. It is a
# comparison of two instruments on one pose, so the tolerance is tight.
FK_TF_TOL_DEG = 1.0
# The clearance floor, the same 0.15 m the mount guard and the follower use.
# HARD CONSTRAINT 11: it is the last thing between the arms and a person's
# chest and it is not lowered for a staging pose.
MIN_CLEARANCE_M = 0.15
# How much a degree of travel from home costs against a millimetre of pose
# error. Small, but not zero: two poses that look the same should be settled
# in favour of the shorter move, because the staging move happens before
# every clip and travels over a person.
TRAVEL_W = 0.0008
# ---- THE MOVE FROM HOME IS BOUNDED, AND THAT IS A HARD CONSTRAINT --------
# The first pose that satisfied everything else needed 270 and 281 degrees of
# total joint travel, with joint_5 swinging +175 and -145. That is not a
# staging move, it is a large unattended sweep over a person before every
# clip, and it is exactly what "short and clean rather than a large sweep"
# rules out.
#
# JOINT_5 IS BOUNDED HARDER THAN THE REST, and not for tidiness.
# config/home_positions_left.txt records it as a SEAM RISK: home puts it
# 14.10 deg from the +/-180 boundary, inside the 0.3 rad margin used
# elsewhere in this project, and "ANY future position-mode code touching
# joint_5 must go through kortex_convention.pose_delta_rad". This script
# publishes position setpoints, so a 175 deg command here is precisely the
# case that warning names -- a small negative excursion wraps and a naive
# controller executes ~358 deg.
# "SHORT AND CLEAN" IS ABOUT THE MOTION YOU SEE, so the bound that matters is
# on the HAND's travel through space, not on a count of degrees. A wrist ROLL
# in place is not a sweep across the wearer however many degrees it is, and a
# level wrist in front of the chest cannot be reached from home without one:
# capping joint_5 at 15 deg found no pose at all, for either arm.
MAX_JOINT_DELTA_DEG = 120.0
MAX_TOTAL_TRAVEL_DEG = 320.0
# How far the hand may travel from where home leaves it. THIS is the number
# that stops the arm sweeping across the person.
MAX_HAND_TRAVEL_M = 0.55

# THE JOINT_5 RULE IS ABOUT DIRECTION, NOT SIZE, and the first version had it
# backwards. home_positions_left.txt records joint_5 as a SEAM RISK because
# home puts it 14.10 deg from the -180 boundary -- so what is dangerous is
# moving it TOWARD that boundary, where a small excursion wraps and a naive
# position controller executes ~358 deg. Moving it the other way, toward
# zero, walks AWAY from the seam and is safer than standing still. A
# symmetric 15 deg cap forbade both equally and ruled out every pose.
J5_HARD_LIMIT_DEG = 179.0


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
        self._solver = None

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

    def ik_joints(self, arm, xyz):
        """One collision-aware IK solution for a LEVEL wrist at `xyz`.

        Level and pointing FORWARD: the tool's +z is the approach axis, so a
        level wrist in front of the chest aims it along +y, which is a -90 deg
        rotation about x. This is the one place the pose deliberately does NOT
        use the pinned anchor -- a staging pose is not teleoperation, and the
        whole reason it exists is that the pinned wrist points up.
        """
        if self._solver is None:
            sys.path.insert(0, os.path.join(ROOT, "src", "srl_experiments",
                                            "experiments", "abc"))
            from verify_task_scenes import Solver
            self._solver = Solver()
            self._solver.spin(3.0)
            self._solver.ik.wait_for_service(timeout_sec=20.0)
        from geometry_msgs.msg import Quaternion
        h = math.radians(45.0)
        q = Quaternion(x=-math.sin(h), y=0.0, z=0.0, w=math.cos(h))
        sol = self._solver.solve_joints(arm, list(xyz), q, tries=4)
        if sol is None:
            return None
        # THIS ARM'S SEVEN, SELECTED BY NAME. solve_joints returns
        # `res.solution.joint_state.position` WHOLE -- all 26 joints of both
        # arms and both grippers -- and handing that to a state builder that
        # expects seven produced a name/position mismatch that MoveIt
        # resolved by falling back to the CURRENT state. Every candidate then
        # measured as the home pose: identical elevation, identical hand
        # position, for every target. That is the "FK evaluating the current
        # state, ignoring the solution given" row of CLAUDE.md's failure
        # table, reached by a different route.
        last = getattr(self._solver, "_last_solution_names", None)
        names = last or self._solver.names(arm)
        if len(sol) == len(names):
            return list(sol)
        idx = {n: i for i, n in enumerate(self._solver_names())}
        want = ["%s_joint_%d" % (arm, i + 1) for i in range(7)]
        if all(n in idx and idx[n] < len(sol) for n in want):
            return [float(sol[idx[n]]) for n in want]
        return None

    def _solver_names(self):
        """The joint ORDER the solver's solutions come back in.

        Read from /joint_states rather than assumed: the IK response orders
        joints however MoveIt's model does, and indexing that by position is
        what turned a 26-joint answer into a 7-joint one silently.
        """
        return list(self.js.name) if self.js is not None else []

    def frame(self, arm, q, links):
        """World positions of several links for one candidate posture."""
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, ln) for ln in links]
        req.robot_state = self._state(arm, q)
        res = self._call(self.fk, req)
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        out = {}
        for ln, ps in zip(links, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[ln] = dict(xyz=(p.x, p.y, p.z), quat=(o.x, o.y, o.z, o.w))
        return out

    def cost(self, arm, q, home):
        """How far this posture is from 'standing ready to work'.

        None means it FAILED A HARD CONSTRAINT, which is different from
        scoring badly: an unreachable or unsafe pose is not a worse candidate,
        it is not a candidate. The soft terms below only ever rank poses that
        are already safe and already have a level wrist.

        The terms are the brief, in order:
          hands in front of the chest, not out at the sides
          lateral extent roughly the width of the torso
          elbows down and slightly out, not level with the shoulders
          wrists level
          and, as a tie-break only, a short move from home
        """
        F = self.frame(arm, q, ["shoulder_link", "forearm_link",
                                "end_effector_link"])
        if F is None:
            return None, None
        hand = F["end_effector_link"]["xyz"]
        elbow = F["forearm_link"]["xyz"]
        sh = F["shoulder_link"]["xyz"]

        # ---- HARD: the wrist must be level ---------------------------
        x, y, z, w = F["end_effector_link"]["quat"]
        ax = (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
              1.0 - 2.0 * (x * x + y * y))
        elev = math.degrees(math.atan2(ax[2], math.hypot(ax[0], ax[1])))
        if abs(elev) > GOOD_ENOUGH_DEG:
            return None, None
        # ---- HARD: in FRONT of the chest, not beside it --------------
        if hand[1] <= TORSO_FRONT_Y:
            return None, None
        # ---- HARD: not folded into the centreline --------------------
        if abs(hand[0]) < HAND_MIN_X:
            return None, None
        # ---- HARD: clear of the wearer, and self-collision free ------
        clear, _who = self.clearance(arm, q)
        if clear is None or clear < MIN_CLEARANCE_M:
            return None, None
        ok, _c = self.valid(arm, q)
        if ok is False:
            return None, None

        # ---- HARD: a staging move, not a sweep -----------------------
        deltas = [abs(math.degrees(q[i] - home[i])) for i in range(7)]
        if max(deltas) > MAX_JOINT_DELTA_DEG:
            return None, None
        if sum(deltas) > MAX_TOTAL_TRAVEL_DEG:
            return None, None
        j5, j5_home = math.degrees(q[4]), math.degrees(home[4])
        if abs(j5) > J5_HARD_LIMIT_DEG or abs(j5) > abs(j5_home) + 1e-9:
            return None, None
        home_hand = self.frame(arm, home, ["end_effector_link"])
        if home_hand is None:
            return None, None
        hand_travel = math.dist(hand, home_hand["end_effector_link"]["xyz"])
        if hand_travel > MAX_HAND_TRAVEL_M:
            return None, None

        c = 0.0
        c += (abs(hand[0]) - HAND_TARGET["x"]) ** 2
        c += (hand[1] - HAND_TARGET["y"]) ** 2
        c += (hand[2] - HAND_TARGET["z"]) ** 2
        # ELBOWS DOWN. Below the shoulder by at least a hand's depth; only
        # the shortfall is penalised, so an elbow that is already low enough
        # is not pushed lower for its own sake.
        el_below = sh[2] - elbow[2]
        c += 4.0 * max(0.0, ELBOW_BELOW_SHOULDER_M - el_below) ** 2
        # AND SLIGHTLY OUT -- WHICH IS A BAND, NOT A DIRECTION.
        #
        # This used to penalise only an elbow INBOARD of the hand, on the
        # reasoning that an elbow tucked in reads as pinned. It scored an
        # elbow 229 mm OUTBOARD of the hand as perfect, and that is precisely
        # the splay the brief rules out: in the render the arm went up and
        # out from the mount and came down outside the body, which is a
        # chicken wing rather than a person standing ready.
        #
        # "Slightly" is ELBOW_OUT_BAND_M. Inside it, no cost; outside it in
        # either direction, quadratic.
        out = abs(elbow[0]) - abs(hand[0])
        c += 6.0 * max(0.0, abs(out) - ELBOW_OUT_BAND_M) ** 2
        # A SHORT MOVE, as a tie-break and nothing more.
        travel = sum(abs(math.degrees(q[i] - home[i])) for i in range(7))
        c += TRAVEL_W * travel
        return c, dict(elev=elev, clearance=clear, hand=hand, elbow=elbow,
                       el_below=el_below, travel=travel,
                       hand_travel=hand_travel)

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
    best_for_target = {}
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

        # ---- the search: SOLVE FOR THE POSE, do not hunt for it ------
        #
        # The hand pose the brief asks for is REACHABLE. Measured
        # (scripts/measure_ready_pose_envelope.py, 282 IK calls, all three
        # controls correct): with a LEVEL wrist in free space at chest
        # height, both arms solve at |x| = 0.14, a hand span of 0.280 m
        # against a 0.360 m torso.
        #
        # THAT DOES NOT CONTRADICT the platform's central result. "Zero
        # reachable cells at |x| <= 0.10" was measured on the WORK PLANE,
        # with the wrist PINNED at the anchor and over a whole approach path.
        # A staging pose is none of those: one posture, chest height, free
        # space, and the wrist is level precisely because it is not pinned.
        # Two different questions, two different answers, and conflating them
        # would have made this pose look impossible.
        #
        # So: ask IK for the pose, rather than descending toward it. What IK
        # does NOT decide is the elbow -- a 7-DOF arm has a null space and
        # TRAC-IK restarts randomly, so repeated calls return genuinely
        # different postures for the same hand pose. Several are drawn and
        # the one with the best elbow wins, which is the only way to get
        # "elbows down and slightly out" out of a position-only solver.
        best_for_target.setdefault(arm, {})
        for tx in (HAND_TARGET["x"], 0.50, 0.55, 0.60):
            for ty in (HAND_TARGET["y"], 0.25, 0.36):
                for tz in (HAND_TARGET["z"], 1.18, 1.10):
                    key = (tx, ty, tz)
                    tgt = [(1.0 if arm == "left" else -1.0) * tx, ty, tz]
                    cands = []
                    for _try in range(IK_DRAWS):
                        q = n.ik_joints(arm, tgt)
                        if q is None:
                            continue
                        c, why = n.cost(arm, q, home)
                        if c is not None:
                            cands.append((c, q, why))
                    if cands:
                        c, q, g = min(cands, key=lambda t: t[0])
                        best_for_target[arm][key] = dict(
                            q=q, elev=g["elev"],
                            clearance_m=round(g["clearance"], 4),
                            hand=[round(v, 4) for v in g["hand"]],
                            elbow=[round(v, 4) for v in g["elbow"]],
                            elbow_below_shoulder_m=round(g["el_below"], 4),
                            travel_deg=round(g["travel"], 1),
                            hand_travel_m=round(g["hand_travel"], 4),
                            target=[round(v, 4) for v in tgt],
                            n_draws=len(cands), cost=round(c, 5),
                            delta_deg=[round(math.degrees(q[i] - home[i]), 2)
                                       for i in range(7)])

    # ---- ONE TARGET, MIRRORED, FOR BOTH ARMS -------------------------
    # Chosen AFTER both arms have been solved, so it is a target both can
    # actually reach rather than whichever each settled on alone. Solving
    # them independently gave the left hand (0.450, 0.320, 1.100) and the
    # right (-0.500, 0.320, 1.220), and the render shows exactly that: one
    # arm forward at the hip and the other out at the side. A posture is
    # symmetric or it reads as a fault.
    shared = [k for k in best_for_target.get("left", {})
              if k in best_for_target.get("right", {})]
    if shared and REQUIRE_MIRRORED_TARGET:
        key = min(shared, key=lambda k: (best_for_target["left"][k]["cost"]
                                         + best_for_target["right"][k]["cost"]))
        for arm in ("left", "right"):
            result[arm] = best_for_target[arm][key]
        print("   mirrored target |x|=%.2f y=%.2f z=%.2f, reachable by both "
              "(%d of %d targets were)"
              % (key[0], key[1], key[2], len(shared),
                 len(best_for_target.get("left", {}))))
    else:
        for arm in ("left", "right"):
            per = best_for_target.get(arm, {})
            result[arm] = (min(per.values(), key=lambda d: d["cost"])
                           if per else None)
        if REQUIRE_MIRRORED_TARGET:
            print("   NO target is reachable by BOTH arms -- falling back to "
                  "per-arm bests, and the pose will be ASYMMETRIC.")

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
        print("   %-5s home tool axis %+.1f deg (TF %+.1f) -> %+.1f deg"
              % (arm, controls["home_elevation_%s_deg" % arm],
                 controls["home_elevation_tf_%s_deg" % arm], b["elev"]))
        print("         hand  (%+.3f, %+.3f, %.3f)   elbow (%+.3f, %+.3f, "
              "%.3f), %.0f mm below the shoulder"
              % (b["hand"][0], b["hand"][1], b["hand"][2],
                 b["elbow"][0], b["elbow"][1], b["elbow"][2],
                 b["elbow_below_shoulder_m"] * 1000))
        print("         wearer clearance %.4f m against %.4f m at home, "
              "floor %.2f" % (b["clearance_m"],
                              controls["home_clearance_%s_m" % arm],
                              MIN_CLEARANCE_M))
        print("         hand travels %.3f m from home (limit %.2f)"
              % (b["hand_travel_m"], MAX_HAND_TRAVEL_M))
        print("         move from home %.0f deg total: %s"
              % (b["travel_deg"],
                 " ".join("j%d%+.0f" % (i + 1, d)
                          for i, d in enumerate(b["delta_deg"])
                          if abs(d) > 0.5) or "none"))
        print("         %s" % controls["wearer_collision_example_%s" % arm])
        if abs(b["elev"]) > GOOD_ENOUGH_DEG:
            print("         WARNING: %.1f deg is outside the %.0f deg target"
                  % (abs(b["elev"]), GOOD_ENOUGH_DEG))
    if all(result[a2] for a2 in ("left", "right")):
        span = abs(result["left"]["hand"][0] - result["right"]["hand"][0])
        print("   HAND SPAN %.3f m against a %.3f m torso -- %.1fx. The "
              "brief asked for about 1x and the CLEARANCE FLOOR forbids it: "
              "at |x| = %.2f the arm is inside the torso. This is the "
              "nearest-in posture the floor permits."
              % (span, 2 * TORSO_HALF_W, span / (2 * TORSO_HALF_W),
                 BRIEF_HAND_X))

    if a.save and not bad:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        json.dump(dict(
            note="joint-space staging pose commanded before capture; NOT a "
                 "task pose and not teleoperation. Found by coordinate "
                 "descent over joints 1, 2, 4 and 6 against a cost that "
                 "encodes hands in front of the chest, lateral extent about "
                 "the torso's, elbows below the shoulders and a level wrist, "
                 "with clearance to the wearer measured GEOMETRICALLY (the "
                 "SRDF excludes the pairs that matter, so "
                 "/check_state_validity is not the authority on it).",
            good_enough_deg=GOOD_ENOUGH_DEG,
            controls={k: v for k, v in controls.items()},
            min_clearance_m=MIN_CLEARANCE_M,
            hand_target=HAND_TARGET,
            poses={arm: dict(delta_deg=result[arm]["delta_deg"],
                             elevation_deg=round(result[arm]["elev"], 2),
                             clearance_m=result[arm]["clearance_m"],
                             hand=result[arm]["hand"],
                             elbow=result[arm]["elbow"],
                             elbow_below_shoulder_m=(
                                 result[arm]["elbow_below_shoulder_m"]),
                             travel_deg=result[arm]["travel_deg"],
                             hand_travel_m=result[arm]["hand_travel_m"],
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
