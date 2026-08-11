#!/usr/bin/env python3
"""Run task A, B or C through a NAMED CONTROL MODE's own command path.

WHY THIS EXISTS. Job A's restructure found that **no clip in the repository
had ever been recorded through a control mode**: every one was produced by
`record_rviz.py`, which calls `/compute_ik` directly and never publishes
`/master_arm_pose_*`, so no follower, clutch, anchor, orientation lock or
safety guard was in the path. The mode axis of the recordings tree was empty
because nothing could fill it.

THE ENTRY POINT IS THE POINT. Each mode publishes at the topic its own
upstream publishes at, and everything downstream is the real thing --
`ik_follower_node`'s pose deadband, the collision-aware IK, the redundancy
re-seed, the clearance floor, the step guard, the flip reject, the e-stop and
every BlockMonitor blocker. There is deliberately no second path to the
controller, so entering here means traversing all of it.

    mode                 entry topic                       upstream normally
    01_master_teleop     /master_arm_pose_<arm>            master_pose_node
    02_vr_teleop         /vr/controller_pose_<side>        quest_vendor_bridge
    04_shared_autonomy   /autonomy/assist_pose_<arm>       handover_arbiter
    06_full_autonomy     /autonomy/assist_pose_<arm>       autonomy_executive

WHAT IS SUBSTITUTED, STATED PLAINLY. The OPERATOR is scripted -- there is no
mannequin attached and no headset -- so this stands in for the human's hand,
exactly as `--scripted` has always meant in this repository. What is NOT
substituted is anything between the entry topic and the joint trajectory.

MODES 04 AND 06 SHARE A TOPIC AND MUST NOT SHARE A CLIP. They differ in who
decides, not in how the command travels. 06 therefore goes through
`autonomy_executive` (a spoken instruction, its confirmation wait, then
motion) and 04 is commanded directly, and each clip records which. Recording
them as the same motion under two names would be a fabricated distinction.

THE MOTION IS MEASURED, NOT ASSUMED. Every run reports the commanded span and
the ACTUAL tf2 end-effector displacement, and exits non-zero if the arms did
not move -- a recording of a stationary arm is worse than no recording,
because it looks like a result.

    ros2 run srl_experiments run_abc.py --task b --mode 04_shared_autonomy \\
        --scenario S2_full_lift --scripted
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tasks as T                                            # noqa: E402
import clip_tasks as CT                                      # noqa: E402

ARMS = ("left", "right")

MODES = {
    # EVERY ENTRY TOPIC CARRIES A WORLD POSE. `ik_follower_node.request_ik`
    # assigns the message straight into the IK request
    # (`req.ik_request.pose_stamped = msg`), so /master_arm_pose_<arm> is an
    # ALREADY-MAPPED world pose -- master_pose_node owns the anchor, the scale
    # and the clutch reference, and the follower owns none of them.
    #
    # This was got backwards first, and the failure was silent in an
    # instructive way. Sending a master-frame DISPLACEMENT put the target near
    # the world origin, IK failed every call, and from the outside it looked
    # like the master was being suppressed -- 0.0048 m for a 0.200 m command
    # with no blocker ever active. The diagnosis came from /ik_status, where
    # `fail` climbed by exactly the number of poses sent and `success` never
    # moved.
    "01_master_teleop": dict(
        topic="/master_arm_pose_%s", frame="world",
        note="the world pose master_pose_node would publish once it has "
             "applied the anchor, scale and clutch reference; enters the "
             "follower at on_pose, the live-teleop callback"),
    "02_vr_teleop": dict(
        topic="/vr/controller_pose_%s", frame="world",
        vr=True,
        note="vr_pose_mapper turns a controller pose into "
             "/master_arm_pose_<arm>; it must be running"),
    "04_shared_autonomy": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        note="world-frame pose, entering request_ik at the same door teleop "
             "uses"),
    # The recording set's mode names. 03_shared_autonomy and 04_vr_shared are
    # the same command paths as 04_shared_autonomy / VR-plus-autonomy; the
    # numbering is the one the clip tree uses.
    "03_shared_autonomy": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        note="master present, autonomy commanding -- the arbiter's topic"),
    "04_vr_shared": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        vr_present=True,
        note="VR transport up AND autonomy commanding; the mapper runs but "
             "the arbiter owns the pose"),
    "06_full_autonomy": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        via_executive=True,
        note="same topic as 04, but commanded BY autonomy_executive from a "
             "spoken instruction -- the difference is who decides"),
}


# WHICH KIND OF BIMANUAL EACH TASK IS, carried into every trial row so the
# distinction cannot be lost between the task spec and the analysis. T2 is
# COUPLING, T3 is BY ROLE, T1 stage 2 is SIMULTANEITY, and merging them is the
# error the specs warn about in three separate places.
KIND = {"M0": "single-arm pointing", "M1": "single-arm pick and place",
        "M1S2": "simultaneity", "M2": "physical coupling",
        "M3": "bimanual by role"}


def densify(path, step=0.03):
    """Intermediate waypoints, so the CLIP shows motion rather than two poses.

    The verification densifies to 20 mm because the arm flies the gaps and
    every point between must be reachable. Here the reason is different: a
    recording of two endpoints is a recording of a jump. 30 mm keeps the
    commanded target ahead of the arm without outrunning it.
    """
    out = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        d = math.dist(a, b)
        n = max(1, int(math.ceil(d / step)))
        for k in range(n):
            f = k / float(n)
            out.append([a[j] + f * (b[j] - a[j]) for j in range(3)])
    out.append(list(path[-1]))
    return out


def waypoints(task_key, scenario):
    """The path this task/scenario asks the arms to follow, per arm.

    Taken from the VERIFIED spec, never re-derived here: every coordinate
    below passed N=10 over the densified full path in
    scripts/verify_abc_scenarios.py, and inventing a nearby one would throw
    that away silently.
    """
    t = T.BY_KEY[task_key]
    if task_key == "A":
        pts = t["targets"]
        n = min(len(pts["left"]), len(pts["right"]))
        idx = {"S1_left_only": ("left",), "S2_right_only": ("right",),
               "S3_both": ARMS}.get(scenario, ARMS)
        out = {}
        for a in ARMS:
            if a in idx:
                out[a] = densify([list(p) for p in pts[a][:n]])
            else:
                out[a] = None                       # filled in below
        hold = max(len(v) for v in out.values() if v)
        for a in ARMS:
            if out[a] is None:
                out[a] = [list(pts[a][0])] * hold   # this arm stays put
        return out
    if task_key == "B":
        path = t["paths"][scenario]
        sep = t["grip_sep"]
        # left works +x in this model -- the link names are viewer-
        # perspective, not anatomical, and assuming otherwise once made an
        # audit report every waypoint unreachable on perfectly good geometry.
        dense = densify(path)
        return {"left": [[p[0] + sep / 2.0, p[1], p[2]] for p in dense],
                "right": [[p[0] - sep / 2.0, p[1], p[2]] for p in dense]}
    # C: a Lissajous inside the verified amplitude shell
    c, amp = t["centres"], t["amplitude_m"]
    sp = t["speeds_m_s"].get(scenario, (0.05, 0.05))
    out = {}
    for i, a in enumerate(ARMS):
        base, v = c[a], sp[i]
        pts = []
        for k in range(24):
            u = 2 * math.pi * k / 24.0
            pts.append([base[0] + amp * math.sin(u) * (0.6 + 0.4 * v * 5),
                        base[1],
                        base[2] + amp * 0.35 * math.sin(2 * u)])
        out[a] = pts
    return out


class Runner(Node):
    def __init__(self, args):
        super().__init__("run_abc")
        self.args = args
        self.spec = MODES[args.mode]
        self.pub = {}
        for a in ARMS:
            key = a if "%s" % a in self.spec["topic"] % a else a
            self.pub[a] = self.create_publisher(
                PoseStamped, self.spec["topic"] % key, 10)
        self.state = self.create_publisher(String, "/trial_state", 10)
        # THE GRIPPER. Without this the sweep recorded arms moving past
        # objects: no gripper command was ever sent, so nothing could be
        # grasped and the scene had nothing to attach. fsr_gripper_node owns
        # the pad-driven behaviour; this is the scripted operator's squeeze,
        # published to the same controller topic.
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self._JT, self._JTP = JointTrajectory, JointTrajectoryPoint
        self.grip_pub = {a: self.create_publisher(
            JointTrajectory, "/%s_gripper_controller/joint_trajectory" % a, 10)
            for a in ARMS}
        # VR IS THE ONE GENUINELY RELATIVE PATH, and it needs a held grip.
        # vr_pose_mapper anchors on the clutch ENGAGE and then commands the
        # arm by the controller's motion SINCE that engage -- which is what
        # makes indexing work and the re-engage jump zero. So a controller
        # pose alone drives nothing: without `axes[1] > 0.6` the mapper is
        # disengaged and correctly publishes nothing at all.
        self.joy = None
        if self.spec.get("vr"):
            from sensor_msgs.msg import Joy
            self._Joy = Joy
            self.joy = {a: self.create_publisher(
                Joy, "/vr/controller_joy_%s" % a, 10) for a in ARMS}
            # /vr/tracking_ok DEFAULTS TO FALSE, and the mapper refuses to
            # command anything while it is false -- correctly: a headset that
            # has lost tracking must not keep driving an arm bolted to a
            # person. Nothing in this runner was publishing it, so the clutch
            # engaged perfectly at the task start and the mapper then froze
            # for the whole run. The headset's bridge is what normally
            # publishes this; the scripted operator has to as well.
            from std_msgs.msg import Bool as _B
            self._Bool = _B
            self.track = self.create_publisher(_B, "/vr/tracking_ok", 10)
        self.speech = None
        if self.spec.get("via_executive"):
            self.speech = self.create_publisher(String, "/voice_transcript", 10)
        self.tf = None
        try:
            import tf2_ros
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
        except Exception:                                     # noqa: BLE001
            self.buf = None

    def _wire_logging(self):
        """Subscriptions that exist ONLY to fill the log.

        Kept separate and called explicitly so that nothing in the control
        path depends on them: a data subscription that could change how the
        arm moves would make every trial a measurement of the logger.
        """
        from sensor_msgs.msg import JointState as _JS
        from std_msgs.msg import Float64MultiArray as _F64
        self.js = {}
        self.ikst = {}
        self.create_subscription(_JS, "/joint_states",
                                 lambda m: self.js.update(
                                     zip(m.name, m.position)), 20)
        for _a in ("left", "right"):
            self.create_subscription(
                _F64, "/ik_status_%s" % _a,
                (lambda a: (lambda m: self.ikst.__setitem__(
                    a, list(m.data))))(_a), 10)

    def joints(self, arm):
        return [round(self.js.get("%s_joint_%d" % (arm, i), 0.0), 5)
                for i in range(1, 8)] if self.js else []

    def ee(self, arm):
        if self.buf is None:
            return None
        try:
            t = self.buf.lookup_transform("world", "%s_end_effector_link" % arm,
                                          rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = t.transform.translation
        return (v.x, v.y, v.z)

    def ee_pose(self, arm):
        """Position AND orientation. `ee()` returns position only, which is
        why every ee_q* column was empty in the first logged run."""
        if self.buf is None:
            return None
        try:
            t = self.buf.lookup_transform("world",
                                          "%s_end_effector_link" % arm,
                                          rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v, q = t.transform.translation, t.transform.rotation
        return (v.x, v.y, v.z, q.x, q.y, q.z, q.w)

    def spin(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def hold_grip(self):
        """Keep the VR clutch engaged. Re-sent every cycle, because the
        mapper reads a level, not an edge."""
        if self.joy is None:
            return
        self.track.publish(self._Bool(data=True))
        for a in ARMS:
            j = self._Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.axes = [0.0, 1.0, 0.0, 0.0]      # axes[1] is the grip
            j.buttons = [0, 0, 0, 0]
            self.joy[a].publish(j)

    def grip(self, arm, rad):
        t = self._JT()
        t.joint_names = ["%s_robotiq_85_left_knuckle_joint" % arm]
        p = self._JTP()
        p.positions = [float(rad)]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = 250_000_000
        t.points = [p]
        self.grip_pub[arm].publish(t)

    def send(self, arm, xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        # THE ARM'S OWN ANCHOR ORIENTATION, ON EVERY PATH. NEVER IDENTITY.
        #
        # This used to send identity on the master-frame path, on the belief
        # that the follower would replace it with the anchor. IT DOES NOT --
        # `on_pose` hands the message straight to `request_ik`, orientation
        # included, because in live teleop master_pose_node has already
        # resolved it (orientation_mode: fixed pins it to the anchor there).
        #
        # MEASURED CONSEQUENCE, and it is the project's own documented trap
        # walked into again: mode 01 made 7 IK calls and FAILED ALL SEVEN,
        # burning 42 redundancy retries (7 x redundancy_samples), while every
        # BlockMonitor blocker stayed clear. From outside it looked like the
        # master was being suppressed; it was being asked for a pose that does
        # not exist. An identity quaternion is not neutral -- it is a
        # specific, unreachable orientation, and this is the fourth false
        # negative it has produced here.
        from srl_teleop import master_calibration as mc
        q = mc.WORKSPACE_ORIENT[arm]
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = q
        self.pub[arm].publish(m)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True,
                    choices=["a", "b", "c", "A", "B", "C",
                             "m1s2", "M1S2",
                             "m0", "m1", "m2", "m3",
                             "M0", "M1", "M2", "M3"])
    ap.add_argument("--taskset", default="study",
                    choices=["study", "clip", "msc"],
                    help="'clip' selects the RECORDING tasks (pick and place, "
                         "hold and place, multimeter); 'study' the "
                         "participant spec")
    ap.add_argument("--mode", default="04_shared_autonomy",
                    choices=sorted(MODES))
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--participant", default="PILOT")
    ap.add_argument("--scripted", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    # 0.15 s, LOWERED FROM 0.45 BECAUSE IT WAS THE BINDING LIMIT ON THE CLIPS.
    # At 0.45 s the commanded pose advanced ~2.2 times a second, the arm
    # reached each waypoint and then sat still for the rest of the hold, and
    # the recorded video changed 0.6-1.3 times a second -- well under the 16
    # fps RViz can actually render here. The video looked like a slideshow and
    # raising the capture rate could not have helped, because there was
    # nothing new to capture. 0.15 s is still three 20 Hz publishes per
    # waypoint, so the topic never goes quiet for longer than 0.15 s and
    # vr_pose_mapper's 0.20 s tracking watchdog still cannot fire.
    ap.add_argument("--hold-s", type=float, default=0.15,
                    help="seconds per waypoint; the follower is asynchronous "
                         "so this sets how fast the target leads the arm")
    ap.add_argument("--min-travel-m", type=float, default=0.01)
    # THE DATA PATH. Without these the MSc tasks were driven and filmed and
    # wrote NOTHING a study could analyse: run_abc was the only entry point
    # they had and it never constructed a TrialLogger. Measured end to end:
    # 0 sample rows across 0 files.
    ap.add_argument("--log-root", default=None,
                    help="write per-trial CSVs here; without it, NO DATA is "
                         "written and the run says so")
    # --participant already exists above; do not re-declare it.
    ap.add_argument("--session", default=None)
    ap.add_argument("--trial-index", type=int, default=0)
    ap.add_argument("--block", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--isolate", action="store_true",
                    help="start this mode's upstreams first, exactly as the "
                         "clip sweep does -- see scripts/mode_upstreams.py")
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    key = a.task.upper()

    if a.taskset == "msc":
        # m0-m3 on the command line, t0-t3 inside msc_clip_tasks. The
        # translation lives HERE, in one line, rather than being spread
        # through the sweep and the GUI -- see run_experiment.sh for why the
        # dispatcher key cannot be t0-t3.
        import msc_clip_tasks as MCT
        # m0-m3 -> t0-t3, and m1s2 -> t1s2 for T1 stage 2. The mapping is
        # explicit rather than `"t" + key[1]`, which silently mapped m1s2 to
        # t1 -- the same key as stage 1 -- so a stage 2 request would have
        # run stage 1 and reported success under the wrong name.
        _MSC_KEY = {"M0": "t0", "M1": "t1", "M1S2": "t1s2",
                    "M2": "t2", "M3": "t3"}
        if key not in _MSC_KEY:
            raise SystemExit(
                "unknown msc task key %r -- expected one of %s"
                % (key, ", ".join(sorted(_MSC_KEY))))
        spec = MCT.TASKS[_MSC_KEY[key]]
        scen = a.scenario or spec["scenario"]
        wp = spec["build"]()
        grip_sched = spec["grip"](len(wp["left"]))
    elif a.taskset == "clip":
        spec = CT.TASKS[key.lower()]
        scen = a.scenario or spec["scenario"]
        wp = spec["build"]()
        grip_sched = spec["grip"](len(wp["left"]))
    else:
        default_scen = {"A": "S3_both", "B": "S2_full_lift",
                        "C": "S1_both_slow"}
        scen = a.scenario or default_scen[key]
        wp = waypoints(key, scen)
        grip_sched = None

    if a.dry_run:
        print("task %s / %s / mode %s : %d waypoints per arm, entry topic %s"
              % (key, scen, a.mode, len(wp["left"]),
                 MODES[a.mode]["topic"] % "<arm>"))
        print(MODES[a.mode]["note"])
        return 0

    rclpy.init()
    n = Runner(a)
    n._wire_logging()
    # THE SAME ISOLATION THE CLIP PATH USES, from the same module.
    #
    # 02_vr_teleop's data run recorded 0.0000 m of EE travel on both arms
    # because vr_pose_mapper is a mode UPSTREAM and only the sweep started it.
    # A data run without this silently logs a stationary arm for both VR
    # modes -- rows, columns, timestamps and no motion.
    if a.isolate:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "..", "..", "scripts"))
        import mode_upstreams as MU
        ok, why = MU.isolate_simple(a.mode, n)
        print("[isolate] %s" % why, flush=True)
        if not ok:
            print("REFUSING: this mode is not isolated. A trial recorded now "
                  "would be about whichever publisher happened to win.")
            return 2

    n.spin(2.0)
    start = {arm: n.ee(arm) for arm in ARMS}

    n.state.publish(String(data=json.dumps(
        dict(experiment="abc", task=key, scenario=scen, mode=a.mode,
             participant=a.participant, state="running"))))

    if n.speech is not None:
        # Mode 06 is commanded by an INSTRUCTION, not by a pose. The executive
        # then does the deciding; this waits for it rather than racing it.
        _utt = "hey doc oc grab the blue cube"
        # KEPT ON THE NODE so every sample row carries the utterance that
        # commanded this trial. Mode 06 is driven by an INSTRUCTION, and a
        # trial whose input is not in the data cannot be attributed to it.
        n._said = _utt
        n.speech.publish(String(data=_utt))
        n.spin(2.5)

    # APPROACH THE START, SETTLE, AND ONLY THEN START MEASURING.
    #
    # Travel was measured from wherever the previous run happened to leave the
    # arm, so a second run of the same task reported 0.0118 m instead of
    # 0.4761 m -- not because anything failed, but because the arm was already
    # there. A number that depends on what ran before it is not a measurement
    # of this run, and it would have made every clip in a sweep look worse
    # than the first one.
    # VR: engage the clutch FIRST and let the mapper anchor, then move. The
    # path is sent as a displacement from its own first waypoint, because the
    # mapper commands motion since engage rather than an absolute pose.
    if n.spec.get("vr"):
        # PLACE THE ARM AT THE TASK START BEFORE ENGAGING.
        #
        # VR commands motion SINCE the clutch engage, so wherever the arm is
        # when the operator squeezes becomes the origin of the task path. Left
        # where a previous run finished, the engage anchored at z = 1.3 and
        # the task's +0.20 m lift then asked for z = 1.5, which is outside the
        # reachable set -- 0.0000 m of travel from a clutch that had engaged
        # perfectly well.
        #
        # This setup move is NOT part of the recorded motion and is not
        # claimed as VR: it is the operator walking the arm to the start,
        # done here over the autonomy topic because it is a placement, not a
        # trial. The clip begins after it.
        from geometry_msgs.msg import PoseStamped as _PS
        setup = {arm: n.create_publisher(_PS, "/autonomy/assist_pose_%s" % arm,
                                         10) for arm in ARMS}
        for _ in range(30):
            for arm in ARMS:
                m = _PS()
                m.header.frame_id = "world"
                m.header.stamp = n.get_clock().now().to_msg()
                (m.pose.position.x, m.pose.position.y,
                 m.pose.position.z) = wp[arm][0]
                from srl_teleop import master_calibration as mc
                q = mc.WORKSPACE_ORIENT[arm]
                (m.pose.orientation.x, m.pose.orientation.y,
                 m.pose.orientation.z, m.pose.orientation.w) = q
                setup[arm].publish(m)
            # THE CONTROLLER STREAM MUST NEVER STOP. vr_pose_mapper has a
            # tracking-loss watchdog and freezes if poses stop arriving --
            # correctly, since a headset that has lost tracking must not keep
            # driving an arm. A silent pause during setup tripped it and the
            # mapper spent the whole run FROZEN, publishing nothing.
            for arm in ARMS:
                n.send(arm, [0.0, 0.0, 0.0])
            n.spin(0.1)
        for _ in range(15):  # let the autonomy hold expire, still streaming
            for arm in ARMS:
                n.send(arm, [0.0, 0.0, 0.0])
            n.spin(0.1)
        # HAND STILL FIRST, THEN SQUEEZE -- the order a real operator uses,
        # and the order the mapper requires. It refuses to engage on a moving
        # controller ("the reference would be latched off a moving hand"),
        # which is correct and is the same quasi-static gate the mannequin
        # clutch uses. Sending the grip and the first pose together gave it
        # one sample to judge, so it refused every time and the arm never
        # moved: measured 0.0000 m with the mapper logging the refusal on
        # every cycle.
        # 50 Hz, because the quiet gate needs at least THREE samples inside
        # `quiet_window_s`. At 20 Hz only two landed in the window, `_quiet()`
        # returned False on the `len(h) < 3` line, and the mapper refused
        # every engage -- while the controller was in fact perfectly still.
        # The refusal was correct; the sample rate was not.
        for _ in range(100):                    # ~2 s of a stationary hand
            n.track.publish(n._Bool(data=True))
            for arm in ARMS:
                n.send(arm, [0.0, 0.0, 0.0])
            n.spin(0.02)
        for _ in range(50):                     # then squeeze, and hold
            n.hold_grip()
            for arm in ARMS:
                n.send(arm, [0.0, 0.0, 0.0])
            n.spin(0.02)
        wp = {a: [[p[i] - v[0][i] for i in range(3)] for p in v]
              for a, v in wp.items()}

    # Open both hands before the approach, so the clip starts from a known
    # gripper state rather than wherever the last run left it.
    if grip_sched is not None:
        for _ in range(6):
            for arm in ARMS:
                n.grip(arm, 0.0)
            n.spin(0.1)
    for _ in range(8):
        n.hold_grip()
        for arm in ARMS:
            n.send(arm, wp[arm][0])
        n.spin(0.35)
    n.spin(1.2)
    start = {arm: n.ee(arm) for arm in ARMS}

    # PATH LENGTH, NOT START-TO-END DISPLACEMENT.
    #
    # Travel was `dist(start, end)`, which is ZERO for any path that returns
    # to where it began -- and task C is a present-and-probe round trip by
    # design. It duly reported 0.0000 m and the motion guard refused the run,
    # correctly by its own rule and wrongly about the world: the arm had moved
    # 0.22 m and come back. Accumulating the path integral answers "did this
    # arm move" for every path shape, which is what the guard is actually for.
    track = {arm: [start[arm]] for arm in ARMS}

    # THE GRIP IS GATED ON ARRIVAL, NOT ON WAYPOINT INDEX.
    #
    # It used to be `grip_sched[arm][k]`, published alongside the pose for
    # waypoint k -- which ASSUMES the arm is already at waypoint k. Under a
    # transport that lags, it is not, and the fingers close somewhere along
    # the way.
    #
    # MEASURED, and this is the whole reason clip_scene now logs closest
    # approach. VR task B: the pads reached the part EXACTLY
    # (min_pad_obj 0.000 m) but the knuckle was 0.0 -- fully OPEN -- at that
    # instant, and when the fingers did close they were 0.339 m away, about
    # one segment behind. So the VR path does not land short: it arrives late
    # relative to a schedule that never asked where the arm was. The
    # index-driven schedule worked on the direct paths only because their lag
    # happened to be smaller than a waypoint dwell.
    #
    # Gating on arrival also makes every mode MORE faithful, not just VR: an
    # operator closes their hand when they see the object between the fingers,
    # not after a fixed number of steps. ARRIVE_TOL_M is the capture window --
    # the same quantity the grasp gate uses -- so "arrived" means "close
    # enough that closing here is a grasp".
    ARRIVE_TOL_M = 0.03
    PAD_OFF = CT.PAD_OFFSET
    grip_obj = spec.get("grip_obj")
    # PER-WAYPOINT OBJECT, when the task has more than one.
    #
    # The gate below holds a grip change pending until the pads reach the
    # object -- correct, and written when every task had exactly one. T1 has
    # four cubes and two planes: its first OPEN sat waiting for the pads to
    # return to cube_0 while the arm was at the plane, so the hand never
    # opened, every later cube was collected on the way past, and all four
    # finished in one place still held. `grip_at` lets a task say which object
    # each waypoint is about; tasks that do not declare one are unaffected.
    grip_at = spec.get("grip_at")
    if callable(grip_at):
        grip_at = grip_at(len(wp["left"]))
    held_grip = {arm: 0.0 for arm in ARMS}
    pend = {arm: None for arm in ARMS}      # value awaiting arrival
    pend_obj = {arm: None for arm in ARMS}  # the object it must arrive AT
    late = {arm: 0 for arm in ARMS}

    # ---------------------------------------------------------------- DATA
    # ONE TRIAL, ONE LOGGER. Opened here rather than in a wrapper so that
    # every entry point into the MSc tasks writes data -- a wrapper is
    # something a caller can forget, and forgetting it is exactly how this set
    # came to have a clip path and no data path.
    tl = None
    if a.log_root:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
        from srl_experiments.trial_logger import TrialLogger
        tl = TrialLogger(a.log_root, a.participant, "msc_%s" % key.lower(),
                         session=a.session)
        tl.write_manifest(
            mode=a.mode, task=key, scenario=scen, seed=a.seed,
            taskset=a.taskset, hold_s=a.hold_s,
            entry_topic=MODES[a.mode]["topic"] % "<arm>",
            waypoints=len(wp["left"]),
            isolated=bool(a.isolate),
            sim_only=True,
            note="mock hardware echoes commands with no dynamics; every "
                 "tracking figure here is a property of the mock")
        tl.start(a.trial_index, condition=a.mode, scenario=scen,
                 target_id=key, arm="both", block=a.block)

    n_sent = 0
    for k in range(len(wp["left"])):
        # RE-SEND THE SAME TARGET AT 20 Hz FOR THE WHOLE HOLD, rather than
        # once per waypoint. A single publish followed by a 0.45 s pause is a
        # 0.45 s silence on the topic, and vr_pose_mapper's tracking watchdog
        # freezes after 0.20 s -- so the previous version engaged the clutch
        # correctly at the task start and was then FROZEN for every waypoint
        # of the run. Continuous streaming is also what a real controller and
        # a real master both do, so this makes every mode more faithful, not
        # just VR.
        steps = max(1, int(a.hold_s / 0.05))
        for _ in range(steps):
            n.hold_grip()
            for arm in ARMS:
                tgt = wp[arm][min(k, len(wp[arm]) - 1)]
                n.send(arm, tgt)
                if grip_sched is not None:
                    want = grip_sched[arm][min(k, len(grip_sched[arm]) - 1)]
                    # A CHANGE IS PENDING AGAINST THE WAYPOINT IT WAS ASKED
                    # FOR, not against whatever is current now.
                    #
                    # The first version compared the arm against the CURRENT
                    # waypoint, which is only reachable during the ticks that
                    # waypoint is being commanded. Under lag the arm arrives
                    # later -- while a LATER waypoint is current -- so the
                    # test was never true and the hand never closed at all
                    # (measured: min_pad_obj 0.000 m with the knuckle at 0.0
                    # and no close anywhere in the run). Right diagnosis,
                    # wrong gate: "close when you get to where you were told
                    # to close" needs the target remembered, not resampled.
                    if want != held_grip[arm] and pend[arm] is None:
                        # REMEMBER THE OBJECT THE CHANGE WAS ASKED AGAINST,
                        # for the same reason the value is remembered: under
                        # lag the arm arrives while a LATER waypoint is
                        # current, and resampling would test against the wrong
                        # object as well as the wrong pose.
                        pend[arm] = want
                        # A LIST applies to both arms; a DICT gives each arm
                        # its own object, which T3 needs -- its left arm holds
                        # the meter and its right the box, and one shared
                        # target left the meter ungrasped for the whole clip.
                        if isinstance(grip_at, dict):
                            _seq = grip_at.get(arm)
                        else:
                            _seq = grip_at
                        pend_obj[arm] = (_seq[min(k, len(_seq) - 1)]
                                         if _seq else grip_obj)
                    if pend[arm] is not None:
                        # AGAINST THE OBJECT, IN WORLD -- not against the
                        # waypoint. VR sends CONTROLLER-frame poses that the
                        # mapper transforms, so comparing the arm's world EE
                        # against a commanded waypoint compares two different
                        # frames and is never small: the gate never fired and
                        # the hand never closed at all. The object is the one
                        # thing both frames agree about.
                        here = n.ee(arm)
                        pads = (None if here is None else
                                [here[i] + PAD_OFF[i] for i in range(3)])
                        obj = pend_obj[arm] or grip_obj
                        if (pads is not None and obj is not None
                                and math.dist(pads, obj) <= ARRIVE_TOL_M):
                            held_grip[arm] = pend[arm]
                            pend[arm] = None
                            pend_obj[arm] = None
                        else:
                            late[arm] += 1
                    n.grip(arm, held_grip[arm])
            if tl is not None:
                _pl, _pr = n.ee_pose("left"), n.ee_pose("right")
                _cl = wp["left"][min(k, len(wp["left"]) - 1)]
                _cr = wp["right"][min(k, len(wp["right"]) - 1)]
                _il = n.ikst.get("left") or []
                _ir = n.ikst.get("right") or []
                _src = {0.0: "sim", 1.0: "real"}.get(
                    _il[11] if len(_il) > 11 else None, "unknown")
                _vr = ("autonomy" not in MODES[a.mode]["topic"]
                       and "vr" in a.mode)
                tl.sample(
                    phase=("grasp" if any(v != CT.OPEN
                                          for v in held_grip.values())
                           else "reach"),
                    ee_x=_pl[0] if _pl else "", ee_y=_pl[1] if _pl else "",
                    ee_z=_pl[2] if _pl else "",
                    ee_qx=_pl[3] if _pl else "", ee_qy=_pl[4] if _pl else "",
                    ee_qz=_pl[5] if _pl else "", ee_qw=_pl[6] if _pl else "",
                    ee_r_x=_pr[0] if _pr else "", ee_r_y=_pr[1] if _pr else "",
                    ee_r_z=_pr[2] if _pr else "",
                    ee_r_qx=_pr[3] if _pr else "",
                    ee_r_qy=_pr[4] if _pr else "",
                    ee_r_qz=_pr[5] if _pr else "",
                    ee_r_qw=_pr[6] if _pr else "",
                    joints_left=json.dumps(n.joints("left")),
                    joints_right=json.dumps(n.joints("right")),
                    cmd_x=_cl[0], cmd_y=_cl[1], cmd_z=_cl[2],
                    cmd_r_x=_cr[0], cmd_r_y=_cr[1], cmd_r_z=_cr[2],
                    gripper_rad=held_grip["left"],
                    gripper_rad_right=held_grip["right"],
                    ik_success=_il[0] if _il else "",
                    ik_fail=_il[1] if _il else "",
                    ik_rejected=_il[4] if len(_il) > 4 else "",
                    ik_success_right=_ir[0] if _ir else "",
                    ik_fail_right=_ir[1] if _ir else "",
                    ik_rejected_right=_ir[4] if len(_ir) > 4 else "",
                    clearance_m=_il[5] if len(_il) > 5 else "",
                    clearance_left_m=_il[5] if len(_il) > 5 else "",
                    clearance_right_m=_ir[5] if len(_ir) > 5 else "",
                    clearance_source=_src,
                    # WHAT THIS MODE ACTUALLY SENT, not what its name implies.
                    # A trial is attributable to the input only if the input
                    # is in the row.
                    vr_ctrl_left=(json.dumps(list(_cl)) if _vr else ""),
                    vr_ctrl_right=(json.dumps(list(_cr)) if _vr else ""),
                    autonomy_cmd=(json.dumps(list(_cl))
                                  if "autonomy" in MODES[a.mode]["topic"]
                                  else ""),
                    voice_utterance=getattr(n, "_said", ""),
                    autonomy_state=("ASSIST" if "autonomy" in
                                    MODES[a.mode]["topic"] else "DIRECT"),
                    estop=int(bool(getattr(n, "estopped", False))))
            n.spin(0.05)
        for arm in ARMS:
            pt = n.ee(arm)
            if pt is not None:
                track[arm].append(pt)
        n_sent += len(ARMS)
    n.spin(1.5)

    if grip_sched is not None and any(late.values()):
        # NOT A WARNING TO IGNORE. This counts the 50 ms ticks on which the
        # schedule asked for a grip change the arm had not earned yet. A large
        # number means this transport lags, which is worth knowing even when
        # the gate saved the run.
        print("[GRIP] deferred %s ticks waiting for arrival (left/right)"
              % "/".join(str(late[a]) for a in ARMS), flush=True)

    end = {arm: n.ee(arm) for arm in ARMS}
    travel, net = {}, {}
    for arm in ARMS:
        pts = [p for p in track[arm] if p is not None]
        if len(pts) < 2:
            travel[arm] = None
        else:
            travel[arm] = sum(math.dist(pts[i], pts[i + 1])
                              for i in range(len(pts) - 1))
        net[arm] = (None if (start[arm] is None or end[arm] is None)
                    else math.dist(start[arm], end[arm]))

    n.state.publish(String(data=json.dumps(
        dict(experiment="abc", task=key, scenario=scen, mode=a.mode,
             state="done"))))
    print("task %s / %s / mode %s : %d poses published on %s"
          % (key, scen, a.mode, n_sent, MODES[a.mode]["topic"] % "<arm>"))
    for arm in ARMS:
        print("   %-5s tf2 EE path %s   net %s"
              % (arm,
                 "NO TF -- cannot tell whether it moved"
                 if travel[arm] is None else "%.4f m" % travel[arm],
                 "--" if net[arm] is None else "%.4f m" % net[arm]))
    n.destroy_node()
    rclpy.shutdown()

    # A RECORDING OF A STATIONARY ARM IS WORSE THAN NO RECORDING, because it
    # looks like a result. `None` is a failure too: not knowing whether the
    # arm moved is not evidence that it did.
    moved = [t for t in travel.values() if t is not None and t >= a.min_travel_m]

    if tl is not None:
        import math as _m
        sl = [p for p in track["left"] if p is not None]
        tl.finish(
            completion_time_s=round(len(wp["left"]) * a.hold_s, 3),
            duration_s=round(len(wp["left"]) * a.hold_s, 3),
            samples=tl._rows,
            kind=KIND.get(key, ""),
            path_length_m=round(travel["left"] or 0.0, 4),
            straight_line_m=round(net["left"] or 0.0, 4),
            path_ratio=(round((net["left"] or 0.0) / travel["left"], 4)
                        if travel["left"] else ""),
            grasp_success=int(bool(moved)))
        print("[data] %d sample rows -> %s" % (tl._rows, tl.dir), flush=True)
        # A RUN THAT WROTE NOTHING IS A FAILURE, LOUDLY. Zero rows with exit 0
        # is how a whole study's output goes missing while every check passes.
        if tl._rows == 0:
            print("\nFAILED: the logger wrote ZERO sample rows. A trial that "
                  "produced no data is not a trial, whatever else happened.")
            return 1

    if not moved:
        print("\nFAILED: no arm moved at least %.3f m. Not recording this as "
              "a successful run." % a.min_travel_m)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
