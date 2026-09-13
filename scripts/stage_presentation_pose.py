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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower_pause as _FP                                 # noqa: E402
POSE_FILE = os.path.join(ROOT, "recordings", "baselines",
                         "presentation_pose.json")
ARRIVE_TOL_RAD = 0.02          # ~1.1 deg per joint

# HOW FAST THE STAGING MOVE IS ALLOWED TO BE, in rad/s per joint.
#
# BOUNDED BY A MEASUREMENT, not chosen. On 2026-08-15 a 1.0315 rad move was
# given 2.5 s -- 0.41 rad/s -- and DID NOT ARRIVE. So whatever the true limit
# is, it is below 0.41, and this sits under it with margin. It is deliberately
# slow anyway: a joint-space move commanded with the followers paused, near a
# person, that is not part of any task.
#
# STILL TO CONFIRM: 2.5 s was measured insufficient, and that bounds the rate
# from above. Nothing here has measured it from BELOW -- what the arm can
# actually sustain -- so this number is a safe bound and not a characterisation.
STAGE_RATE_RAD_S = 0.30


def move_seconds(worst_rad, floor_s=2.5, rate_rad_s=STAGE_RATE_RAD_S,
                 ceiling_s=30.0):
    """How long to give a staging move, FROM HOW FAR IT HAS TO GO.

    THE BUG THIS FIXES. The trajectory was one point with a FIXED 2.5 s
    time_from_start whatever the distance, so a move of 2.23 rad asked for
    0.89 rad/s sustained and did not arrive. MEASURED on the 2026-08-15 sweep,
    printed by this script itself:

        left  presentation pose, worst joint error 1.0315 rad
        right presentation pose, worst joint error 2.2282 rad
        DID NOT ARRIVE within 8.0 s

    A fixed duration is only ever right for a fixed distance, and this one is
    not fixed: the arm starts wherever the previous task left it. So the
    scaling is right on its own terms.

    **IT IS NOT WHAT CAUSED THE 2026-08-15 CLIPS TO OPEN ON HOME, and saying
    so here matters more than the fix does.** With the scaling in, mode 04's
    right arm was given 8.8 s and a 12.8 s deadline and reported the SAME
    2.6506 rad error at the end as at the start -- unchanged to four decimals,
    and identical again on the next clip. An arm that has not moved at all is
    not an arm that was rushed. The real cause is upstream of this file: in
    the two VR modes something holds the arm while the followers are paused,
    which is this project's own one-source-at-a-time rule appearing at the
    CONTROLLER level, exactly as recorded in docs/NEXT_SESSION.md. Modes 01,
    03 and 06 stage first time on the same stack.

    This function therefore removes one real defect and leaves the visible
    symptom untouched. Do not read a green staging line as evidence that the
    controller contention is fixed.

    `worst_rad` may be None -- the joint state has not arrived -- and then the
    floor is used, because guessing a long move from no information would
    make every clip wait.
    """
    if worst_rad is None:
        return floor_s
    return max(floor_s, min(ceiling_s, abs(worst_rad) / rate_rad_s))


# THE SHM REAPER IS REMOVED, AND THE REASONING IS WORTH KEEPING.
#
# The presentation pose kept losing a port race -- Fast DDS leaks a segment
# per participant on this host, the range fills, rclpy.init() fails with
# "Failed init_port fastrtps_portNNNN" and the clip silently opens on home.
# So a reaper was added: delete any /dev/shm/fastrtps_* segment that no live
# process has mapped, read from /proc/*/maps.
#
# IT WAS UNSOUND, in two escalating ways, and both were measured:
#
#   1. it deleted the DISCOVERY PORTS. `fastrtps_port7400` is the rendezvous
#      every participant uses and is not mapped continuously by anybody, so
#      the "orphaned" test called every one of them orphaned. After a clean
#      restart that had just reported READY with 30 joint-state messages, the
#      graph became unjoinable.
#
#   2. excluding the ports was not enough. A participant's own segment is
#      mapped by its PEERS on demand, so a participant with no peer attached
#      at that instant also looks orphaned -- and deleting it makes that
#      participant permanently unreachable. Measured after the exclusion was
#      added: a fresh rclpy node discovered 2 topics and 1 node, itself.
#
# "NOT CURRENTLY MAPPED" IS NOT "ORPHANED", and no amount of refining the
# test fixes that -- the information simply is not in /proc.
#
# So the port pressure is left alone. docs/ENGINEERING_LOG.md's rule stands: clear
# /dev/shm/fastrtps_* only with the stack STOPPED. When staging loses the
# race the sweep records opened_on="home" for that clip, which is a true
# statement about a real clip rather than a graph broken to avoid it.


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
    #
    # THE IMPLEMENTATION MOVED TO scripts/follower_pause.py, unchanged, because
    # the OBSERVE move needs exactly the same thing and a second copy of a
    # routine whose restore failure disarms an arm is not a copy worth having.
    # See that module for the whole account.
    FOLLOWERS = _FP.FOLLOWERS

    def _params(self, node, names):
        return _FP._params(self, node, names)

    def _set_bool(self, node, name, value):
        return _FP._set_bool(self, node, name, value)

    def pause_followers(self):
        """Lower `motion_enabled` on both followers, remembering the value."""
        return _FP.pause(self)

    def resume_followers(self, paused, tries=10):
        """Restore exactly what was lowered, and verify it by read-back."""
        if not _FP.resume(self, paused, tries=tries):
            self._restore_failed = True

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
                    help="go to HOME instead, for teardown. Since 2026-08-16 "
                         "home IS the presentation pose, so this now only "
                         "changes the label.")
    ap.add_argument("--stored-pose", action="store_true",
                    help="stage to recordings/baselines/presentation_pose.json "
                         "instead of the config. That file predates the "
                         "2026-08-16 home change and is 3.06 rad from it; use "
                         "this only to reproduce an older recording.")
    a = ap.parse_args()

    # ==================================================================
    # THE TARGET IS `config/home_positions_*.txt`, AND IT USED NOT TO BE
    # ==================================================================
    # This staged to `recordings/baselines/presentation_pose.json` -- a SIXTH
    # copy of the home pose, in a file `test_home_has_one_source` does not
    # look at. docs/ENGINEERING_LOG.md's own note on `initial_positions.yaml` warns about
    # exactly this shape, in exactly these words, about a different file.
    #
    # HOME BECAME THE PRESENTATION POSE ON 2026-08-16 (HARD CONSTRAINT 0) and
    # that JSON was written before it. Measured 2026-08-23, the stored pose is
    #
    #     left  3.0557 rad from home (joint_4)      175.1 deg
    #     right 3.0805 rad from home (joint_4)      176.5 deg
    #
    # WHAT IT COST, and it is the whole recording pipeline. `run_abc` gained
    # `require_home()` after that date: it stages, re-reads /joint_states and
    # REFUSES to run from an unknown pose. Staging put the arms on the stored
    # pose and reported "worst joint error 0.0000 rad" -- true, against its own
    # target -- and `require_home` then measured 3.08 rad against the source
    # and refused. Every cell of the sweep has failed that way since, with two
    # sentences one line apart saying 0.0000 and 3.0805 about the same arms.
    # Neither was lying; they were staging to two different poses.
    #
    # So the target is the one source. The JSON is kept as the RECORD of the
    # solve that produced the pose -- its controls and achieved constraint
    # values are still the evidence -- and `--stored-pose` still stages to it
    # deliberately, for anyone reproducing a recording made before this date.
    import home_positions as hp
    want = {arm: list(hp.load_home_radians(arm)) for arm in ("left", "right")}
    what = "home" if a.home else "presentation"
    if a.stored_pose:
        if not os.path.exists(POSE_FILE):
            print("no %s -- run scripts/find_presentation_pose.py --save"
                  % POSE_FILE)
            return 2
        d = json.load(open(POSE_FILE))
        failed = [k for k, v in d.get("controls", {}).items() if v is False]
        if failed:
            print("REFUSING: the stored pose was saved by a run whose "
                  "controls failed: %s" % failed)
            return 3
        want = {arm: d["poses"][arm]["q"] for arm in ("left", "right")}
        what = "STORED presentation (pre-2026-08-16)"
    elif os.path.exists(POSE_FILE) and not a.home:
        # SAY SO WHEN THEY DISAGREE. A silent divergence between the record of
        # the solve and the pose that ships is how this happened; naming it
        # every time makes it impossible to acquire again unnoticed.
        try:
            d = json.load(open(POSE_FILE))
            worst = 0.0
            for arm in ("left", "right"):
                q = d["poses"][arm]["q"]
                worst = max(worst, max(
                    abs(((x - y + math.pi) % (2 * math.pi)) - math.pi)
                    for x, y in zip(q, want[arm])))
            if worst > 0.05:
                print("   NOTE: %s is %.4f rad from config/home_positions_*."
                      " Staging to the CONFIG, which is the source."
                      % (os.path.basename(POSE_FILE), worst))
        except Exception:                                      # noqa: BLE001
            pass

    # RETRY THE DDS BRING-UP. Every clip runs this as a fresh process, and
    # Fast DDS leaks a shared-memory segment per participant on this host --
    # 167 of them after one session -- so the port range fills and
    # `rclpy.init()` fails with "Failed init_port fastrtps_portNNNN". Measured
    # on the first full mode: the pose staged on clip 1 and failed on the
    # other four, so four clips silently opened on home.
    #
    # A retry gets a different port. The segments are NOT cleared here on
    # purpose: docs/ENGINEERING_LOG.md is explicit that /dev/shm/fastrtps_* may only be
    # cleared with the stack STOPPED, and a staging script that wiped them
    # mid-sweep would take the running stack's discovery with it.
    last = None
    for attempt in range(1, 6):
        try:
            rclpy.init()
            break
        except Exception as e:                                 # noqa: BLE001
            last = e
            time.sleep(1.5)
    else:
        print("could not initialise DDS after 5 attempts: %s" % last)
        return 2
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
    # DURATION FROM DISTANCE, PER ARM, and the deadline from the duration.
    secs = {}
    for arm in ("left", "right"):
        secs[arm] = move_seconds(n.worst_error(arm, want[arm]), a.move_s)
        n.send(arm, want[arm], secs[arm])
    longest = max(secs.values())
    if longest > a.move_s + 0.01:
        print("   %s move: %.1f s (left) / %.1f s (right) -- scaled from the "
              "distance, not the default %.1f"
              % (what, secs["left"], secs["right"], a.move_s))
    # THE DEADLINE MUST OUTLAST THE MOVE IT IS WAITING FOR. Leaving it at a
    # flat 8 s meant a legitimately 12 s move was reported as a failure to
    # arrive, and the clip opened on home for a move that was still running.
    deadline_s = max(a.timeout_s, longest + 4.0)

    # A CRASH BETWEEN HERE AND THE RESTORE LEAVES THE FOLLOWERS DISARMED,
    # which is the SAFE direction -- motion off, not motion on -- but it is
    # confusing, and a teleop session that silently will not move is the kind
    # of thing that costs a morning. So the restore runs on every exit path.
    t0 = time.time()
    ok = {}
    while time.time() - t0 < deadline_s:
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
    failed = getattr(n, "_restore_failed", False)
    n.destroy_node()
    rclpy.shutdown()
    for node, note in paused.items():
        print("   %-22s %s" % (node, note))
    if failed:
        # NON-ZERO EVEN IF THE POSE ITSELF ARRIVED. A clip that opened on the
        # right pose and left the follower disarmed has broken every clip
        # after it, and that is the more important fact.
        print("REFUSING to report success: a follower was left DISARMED.")
        return 4

    for arm in ("left", "right"):
        e = errs[arm]
        print("   %-5s %s pose, worst joint error %s"
              % (arm, what,
                 "%.4f rad" % e if e is not None else "UNKNOWN"))
    if all(ok.values()):
        return 0
    print("DID NOT ARRIVE within %.1f s: %s. Capture would open on a "
          "half-finished move." % (deadline_s,
                                   [k for k, v in ok.items() if not v]))
    return 1


if __name__ == "__main__":
    sys.exit(main())
