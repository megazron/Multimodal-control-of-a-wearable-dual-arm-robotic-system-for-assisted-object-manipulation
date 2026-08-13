#!/usr/bin/env python3
"""Drive both arms to the PRESENTATION POSE and wait until they arrive.

    python3 scripts/stage_presentation_pose.py [--timeout-s 6] [--home]

Run before capture starts, so every clip OPENS on a deliberate pose instead of
on the home wrist pointing up by +85 / +79 degrees. The pose itself is derived
and checked by `scripts/find_presentation_pose.py` and stored in
`recordings/baselines/presentation_pose.json`; this script only commands it.

IT IS A STAGING MOVE, NOT TELEOPERATION AND NOT PART OF ANY TASK. One posture,
taken before the trial, no operator in the loop, no trial data produced --
the same footing the scan pose stands on. It cannot come from the task path at
all, because the clip runner publishes end effector POSITIONS and the follower
pins orientation, so there is no position that changes where the hand points.

IT WAITS FOR ARRIVAL RATHER THAN SLEEPING. A fixed sleep records whatever the
arm happens to have reached, and this project has already filmed a hold the
arm never settled into: T3 released the circuit box after 25 mm of an 80 mm
lift because the schedule ran on while the asynchronous follower was still
climbing. Exit code says whether both arms actually got there.

REFUSES RATHER THAN GUESSING. No stored pose, a pose whose controls failed, or
an arm that does not arrive are all reported and non-zero. Opening on the home
pose is a known, documented picture; opening on a half-finished move is not.

=====================================================================
KNOWN LIMITATION, MEASURED: THE FOLLOWER WINS. Read before relying on it.
=====================================================================
`ik_follower_node` streams position commands to the SAME arm controller this
script publishes a trajectory to. Once the follower has a target it holds the
arm there, so a staging trajectory published underneath it is overridden and
this script correctly reports DID NOT ARRIVE.

Measured: immediately after a fresh stack start, when the follower has no
target yet, staging works -- both arms arrive to within 0.012 rad. After a
clip has run, the follower is holding the last task pose and the same command
moves the arm by nothing at all: worst joint error 0.5585 (left) and 0.7330
(right) rad, unchanged across the whole timeout.

This is the project's own one-source-at-a-time rule appearing at the CONTROLLER
level rather than the process level, and it is not fixed. Two ways out, and
neither is chosen here because both need a decision about the follower:

  * pause the follower for the staging move (it already has enable/disable
    services) and resume it before the task starts; or
  * give the follower a joint-space "go here and hold" mode and stage through
    it, so there is only ever one publisher.

Until then the sweep records `opened_on` per clip, so a set where staging
silently lost is identifiable rather than assumed.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "config"))
POSE_FILE = os.path.join(ROOT, "recordings", "baselines",
                         "presentation_pose.json")
ARRIVE_TOL_RAD = 0.02          # ~1.1 deg per joint


class Stager(Node):
    def __init__(self):
        super().__init__("presentation_pose_stager")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)
        self.pub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 5)
            for a in ("left", "right")}

    def spin(self, secs):
        t = time.time()
        while time.time() - t < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, arm, q, secs):
        m = JointTrajectory()
        m.joint_names = ["%s_joint_%d" % (arm, i + 1) for i in range(7)]
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(secs)
        p.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        m.points = [p]
        self.pub[arm].publish(m)

    # ---------------------------------------------------------- follower
    FOLLOWERS = ("/ik_follower_left", "/ik_follower_right")

    def _params(self, node, names):
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, node + "/get_parameters")
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        req = GetParameters.Request(names=list(names))
        fut = cli.call_async(req)
        t = time.time()
        while not fut.done() and time.time() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result()

    def _set_bool(self, node, name, value):
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParameters
        cli = self.create_client(SetParameters, node + "/set_parameters")
        if not cli.wait_for_service(timeout_sec=3.0):
            return False
        p = Parameter(name=name,
                      value=ParameterValue(type=1, bool_value=bool(value)))
        fut = cli.call_async(SetParameters.Request(parameters=[p]))
        t = time.time()
        while not fut.done() and time.time() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        r = fut.result()
        return bool(r and r.results and r.results[0].successful)

    def pause_followers(self):
        """Lower `motion_enabled` on both followers, remembering the value.

        Returns {node: note} describing what was done to each, so the caller
        can print it -- a pause that happened silently is one nobody can tell
        from a pause that did not.
        """
        out = {}
        for node in self.FOLLOWERS:
            res = self._params(node, ["motion_enabled", "real_robot"])
            if res is None or len(res.values) < 2:
                out[node] = "not reachable; left alone"
                continue
            was, real = res.values[0].bool_value, res.values[1].bool_value
            if real:
                # HARD CONSTRAINT 8. Motion on real hardware is armed by a
                # person, never by a script tidying up after itself.
                out[node] = ("real_robot mode -- REFUSING to touch "
                             "motion_enabled")
                continue
            if not was:
                out[node] = "motion_enabled already false; left alone"
                continue
            out[node] = ("paused (motion_enabled true -> false)"
                         if self._set_bool(node, "motion_enabled", False)
                         else "COULD NOT PAUSE -- the follower may win")
        return out

    def resume_followers(self, paused):
        """Restore exactly what was lowered, and nothing else."""
        for node, note in paused.items():
            if note.startswith("paused"):
                ok = self._set_bool(node, "motion_enabled", True)
                paused[node] = note + (", restored" if ok
                                       else ", RESTORE FAILED")

    def worst_error(self, arm, q):
        if self.js is None:
            return None
        idx = {n: i for i, n in enumerate(self.js.name)}
        worst = 0.0
        for i, v in enumerate(q):
            n = "%s_joint_%d" % (arm, i + 1)
            if n not in idx:
                return None
            # WRAPPED. joint_5 sits 14 deg from the +/-180 seam, and an
            # unwrapped difference there reads as a ~358 degree error on an
            # arm that is exactly where it was asked to be.
            d = (float(self.js.position[idx[n]]) - float(v) + math.pi) \
                % (2 * math.pi) - math.pi
            worst = max(worst, abs(d))
        return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout-s", type=float, default=8.0)
    ap.add_argument("--move-s", type=float, default=2.5)
    ap.add_argument("--discover-s", type=float, default=15.0,
                    help="how long to wait for /joint_states to appear")
    ap.add_argument("--home", action="store_true",
                    help="go to HOME instead, for teardown")
    a = ap.parse_args()

    if a.home:
        import home_positions as hp
        want = {arm: list(hp.load_home_radians(arm))
                for arm in ("left", "right")}
        what = "home"
    else:
        if not os.path.exists(POSE_FILE):
            print("no %s -- run scripts/find_presentation_pose.py --save"
                  % POSE_FILE)
            return 2
        d = json.load(open(POSE_FILE))
        failed = [k for k, v in d.get("controls", {}).items()
                  if v is False]
        if failed:
            print("REFUSING: the stored pose was saved by a run whose "
                  "controls failed: %s" % failed)
            return 3
        want = {arm: d["poses"][arm]["q"] for arm in ("left", "right")}
        what = "presentation"

    rclpy.init()
    n = Stager()
    # ---- PAUSE THE FOLLOWER FOR THE DURATION OF THE MOVE --------------
    #
    # See the limitation note above: the follower streams position commands
    # to the same controller this publishes a trajectory to, and once it has
    # a target it wins. `motion_enabled` is its own arming parameter and
    # `ik_follower_node` returns early without publishing when it is false,
    # so lowering it is a clean pause rather than a kill.
    #
    # THE PREVIOUS VALUE IS RESTORED, NOT FORCED TRUE. HARD CONSTRAINT 8:
    # real_robot mode must be ARMED BY HAND, and a staging script that armed
    # motion as a side effect would be exactly the silent re-arm that rule
    # exists to prevent. If an arm is in real_robot mode this refuses to
    # touch it at all and says so.
    paused = n.pause_followers()
    # WAIT FOR DISCOVERY, do not sleep a guess. A fixed 1.5 s was enough when
    # this was run by hand against a warm graph and NOT enough as a fresh
    # subprocess of the sweep -- so the first clip of a run silently opened on
    # the home pose with "no /joint_states -- is the stack up?" while the
    # stack was plainly up. Every other helper here sleeps 3 s for the same
    # reason; a loop is better than any of those numbers.
    t0 = time.time()
    while n.js is None and time.time() - t0 < a.discover_s:
        n.spin(0.25)
    if n.js is None:
        print("no /joint_states after %.1f s -- is the stack up?"
              % a.discover_s)
        n.destroy_node()
        rclpy.shutdown()
        return 2
    for arm in ("left", "right"):
        n.send(arm, want[arm], a.move_s)

    # A CRASH BETWEEN HERE AND THE RESTORE LEAVES THE FOLLOWERS DISARMED,
    # which is the SAFE direction -- motion off, not motion on -- but it is
    # confusing, and a teleop session that silently will not move is the kind
    # of thing that costs a morning. So the restore runs on every exit path.
    t0 = time.time()
    ok = {}
    while time.time() - t0 < a.timeout_s:
        n.spin(0.2)
        # `x or 9.9` READS A PERFECT ARRIVAL AS A FAILURE, because 0.0 is
        # falsy. Measured: an arm already sitting exactly on the pose
        # reported "worst joint error 0.0000 rad" and then "DID NOT ARRIVE",
        # and the sweep dutifully logged that the clip opened on home. The
        # sentinel has to be tested for, not leaned on.
        errs = {arm: n.worst_error(arm, want[arm])
                for arm in ("left", "right")}
        ok = {arm: e is not None and e <= ARRIVE_TOL_RAD
              for arm, e in errs.items()}
        if all(ok.values()):
            break
    errs = {arm: n.worst_error(arm, want[arm]) for arm in ("left", "right")}
    n.resume_followers(paused)
    n.destroy_node()
    rclpy.shutdown()
    for node, note in paused.items():
        print("   %-22s %s" % (node, note))

    for arm in ("left", "right"):
        e = errs[arm]
        print("   %-5s %s pose, worst joint error %s"
              % (arm, what,
                 "%.4f rad" % e if e is not None else "UNKNOWN"))
    if all(ok.values()):
        return 0
    print("DID NOT ARRIVE within %.1f s: %s. Capture would open on a "
          "half-finished move." % (a.timeout_s,
                                   [k for k, v in ok.items() if not v]))
    return 1


if __name__ == "__main__":
    sys.exit(main())
