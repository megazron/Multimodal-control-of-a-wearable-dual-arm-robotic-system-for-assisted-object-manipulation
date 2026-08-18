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

import numpy as np
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


# --------------------------------------------------------------------------
# THE RUN MUST START FROM HOME, AND UNTIL 2026-08-17 IT DID NOT.
#
# MEASURED, on a fresh teleop stack, reading /joint_states against
# config/home_positions_{arm}.txt at the instant the FIRST waypoint is
# published on the mode's entry topic:
#
#     at boot                              left 0.0000   right 0.0000 rad
#     run 1, first commanded waypoint      left 0.0000   right 0.0000
#     between runs (nothing else ran)      left 1.2338   right 0.6248
#     run 2, first commanded waypoint      left 1.2338   right 0.6248
#
# So the pose is NOT lost between launch and the first trial -- the URDF, the
# five home carriers and the sim spawn are all correct and were measured to be
# correct. It is lost BETWEEN TRIALS: nothing returns the arms to home when a
# run ends, and nothing checked where they were when the next one started.
# `record_abc_sweep.py` is the only caller that ever staged, and it stages in
# the SWEEP, not in the runner -- so every run driven from the GUI, from
# run_experiment.sh or from a data session after the first one began wherever
# the previous task happened to stop.
#
# NOTE THE RIGHT ARM. T1 is a left-arm task and still leaves the right arm
# 0.62 rad out, because the runner parks the idle arm at park(+/-PARK_X) and
# never brings it back. "The task did not use that arm" is not the same as
# "that arm is where it started".
#
# WHY THE FIX IS HERE AND NOT IN THE SWEEP. The sweep already stages; adding
# it there again would fix the one caller that was not broken. run_abc is the
# ONLY entry point the MSc tasks have, so this is the one place every path
# goes through.
#
# WHY IT SHELLS OUT RATHER THAN COMMANDING THE POSE ITSELF. There is exactly
# one staging move in this repository and `stage_presentation_pose.py` is it:
# it pauses the followers by lowering `motion_enabled`, scales the duration
# from the distance, waits for ARRIVAL rather than sleeping, restores the
# followers on every exit path, and refuses to touch an arm in real_robot
# mode. A second copy here would be a second definition of "go home" and the
# two would drift -- which is the fault this file already carries three
# comments about.
#
# IT REFUSES RATHER THAN RUNNING FROM AN UNKNOWN POSE. A trial that starts
# somewhere nobody recorded cannot be compared with one that did, and the
# whole no-operator mode comparison rests on the runs being alike.
HOME_TOL_RAD = 0.05        # the same tolerance sim_to_real_bridge.enable uses


def home_error(node):
    """Worst wrapped per-joint distance from the loaded home, per arm.

    Returns {arm: (worst_rad, joint_name)} or None if /joint_states has not
    delivered both arms yet -- UNKNOWN, never a quiet zero. A missing joint
    state reading as "0.0000 rad from home" is the exact shape of the silent
    faults this repository keeps finding.
    """
    ws = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
    cfg = os.path.join(ws, "config")
    if cfg not in sys.path:
        sys.path.insert(0, cfg)
    import home_positions as hp
    out = {}
    for arm in ARMS:
        want = list(hp.load_home_radians(arm))
        names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        if not all(nm in node.js for nm in names):
            return None
        d = [((node.js[nm] - w + math.pi) % (2 * math.pi)) - math.pi
             for nm, w in zip(names, want)]
        k = max(range(7), key=lambda i: abs(d[i]))
        out[arm] = (abs(d[k]), names[k])
    return out


# WHERE THE FIRST-WAYPOINT HOME MEASUREMENT IS LEFT FOR THE SWEEP TO FIND.
# Overridable so two runs cannot race, and so a test can point it at a temp dir.
FIRST_WP_FILE = os.environ.get("SRL_FIRST_WP_FILE",
                               "/tmp/srl_first_wp_home.json")


def home_probe(node, label):
    """Record how far each arm is from home, RIGHT NOW, under a label.

    WHY A TRAIL AND NOT ONE READING. `require_home()` returned homed and the
    first commanded waypoint measured 0.7955 rad (left) / 0.6249 (right) out,
    so something between the two moves the arms and nothing was watching. One
    reading at each end tells you that; it does not tell you which step did it.
    The trail is dumped with the first-waypoint record so a failing clip names
    the step rather than the symptom.
    """
    err = home_error(node)
    row = dict(at=label,
               per_arm=(None if err is None
                        else {a: round(v[0], 4) for a, v in err.items()}),
               worst_rad=(None if err is None
                          else round(max(v[0] for v in err.values()), 4)))
    if not hasattr(node, "home_trail"):
        node.home_trail = []
    node.home_trail.append(row)
    print("[home] probe %-22s %s" % (
        label, "UNKNOWN (no /joint_states)" if row["worst_rad"] is None
        else "worst %.4f rad  %s" % (row["worst_rad"], row["per_arm"])),
        flush=True)
    return row


def write_home_record(a, node, pre_row, post_row=None):
    """Leave the home evidence where the recording sweep can find it.

    The sweep launches the runner through the GUI and never sees its stdout, so
    a sidecar file is the channel -- the same one the sweep already uses for the
    staged detections. Written at the pre-approach instant and UPDATED with the
    post-approach reading, so the file is complete even if the run dies mid-clip.

    `at_home` is the PRE-APPROACH reading and nothing else. The post-approach
    numbers are recorded because they are useful for reading a frame, but they
    are not a verdict: the arms are supposed to be at waypoint 0 by then.
    """
    rec = dict(task=a.task, mode=a.mode, tol_rad=HOME_TOL_RAD,
               per_arm=pre_row["per_arm"], worst_rad=pre_row["worst_rad"],
               at_home=bool(pre_row["worst_rad"] is not None
                            and pre_row["worst_rad"] <= HOME_TOL_RAD),
               measured_at="before approach-the-start",
               after_approach=post_row,
               trail=getattr(node, "home_trail", []),
               allow_unhomed=bool(a.allow_unhomed))
    try:
        with open(FIRST_WP_FILE, "w") as fh:
            json.dump(rec, fh, indent=2)
    except Exception as e:                                    # noqa: BLE001
        print("[home] could not write %s: %s" % (FIRST_WP_FILE, e), flush=True)
    return rec


def require_home(node, allow_unhomed=False, tol=HOME_TOL_RAD):
    """Put the arms on home before the task commands anything. (ok, why).

    Idempotent: an arm already within `tol` is left alone and no publisher is
    created, so the recording sweep -- which stages before it calls this --
    pays nothing for it.
    """
    node.spin(2.0)
    err = home_error(node)
    if err is None:
        why = ("no /joint_states for both arms, so where the arms are is "
               "UNKNOWN -- refusing rather than assuming home")
        return (allow_unhomed, why)
    node.start_home_err = {a: round(v[0], 4) for a, v in err.items()}
    worst = max(v[0] for v in err.values())
    if worst <= tol:
        print("[home] arms are at home (worst %.4f rad, tol %.2f)"
              % (worst, tol), flush=True)
        return True, "already home"
    print("[home] NOT at home: %s -- staging before the run"
          % ", ".join("%s %.4f rad (%s)" % (a, v[0], v[1])
                      for a, v in sorted(err.items())), flush=True)
    import subprocess
    ws = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
    r = subprocess.run(
        [sys.executable, os.path.join(ws, "scripts",
                                      "stage_presentation_pose.py")],
        capture_output=True, text=True)
    for ln in (r.stdout or "").strip().splitlines():
        print("      %s" % ln, flush=True)
    node.spin(1.5)
    err = home_error(node)
    if err is None:
        return allow_unhomed, "lost /joint_states during staging"
    node.start_home_err = {a: round(v[0], 4) for a, v in err.items()}
    worst = max(v[0] for v in err.values())
    if worst <= tol:
        print("[home] staged: worst %.4f rad" % worst, flush=True)
        return True, "staged"
    why = ("the arms are %.4f rad from home after staging (%s). The staging "
           "move publishes to the same controller ik_follower_node and "
           "vr_pose_mapper do, so a publisher that holds the arm wins: stop "
           "vr_pose_mapper for VR modes, or pass --allow-unhomed to record a "
           "run that deliberately does not start from home."
           % (worst, ", ".join("%s %.4f (%s)" % (a, v[0], v[1])
                               for a, v in sorted(err.items()))))
    return allow_unhomed, why


class Runner(Node):
    def __init__(self, args):
        super().__init__("run_abc")
        self.args = args
        self.spec = MODES[args.mode]
        # THE ORIENTATION EVERY WAYPOINT IS SENT WITH, PER ARM.
        #
        # `master_calibration.WORKSPACE_ORIENT` unless the TASK declares its
        # own, and only one task does. Every teleop mode pins the wrist at the
        # anchor and the mode comparison depends on that, so this is not a
        # global change and must not become one -- HARD CONSTRAINT 1. It is a
        # task built for `06_full_autonomy` alone saying which way its hand
        # points, which is what 06 means.
        #
        # `set_orient()` fills it from the task spec in main(); until then it
        # is the anchor, so anything that constructs a Runner and sends before
        # the task is resolved behaves exactly as it always did.
        from srl_teleop import master_calibration as _mc
        self.orient = {a: tuple(_mc.WORKSPACE_ORIENT[a]) for a in ARMS}
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
        # THE ARM'S ORIENTATION FOR THIS TASK. `self.orient` is the anchor
        # unless the task declared otherwise; see __init__ and set_orient().
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = self.orient[arm]
        self.pub[arm].publish(m)

    def set_orient(self, orient):
        """Adopt a task's own approach orientation, or keep the anchor.

        THE PAD OFFSET MOVES WITH IT, and that is the half of this change that
        is easy to miss. The arrival gate compares the FINGER PADS against the
        object, and it recovers the pads from the live wrist by adding
        `clip_tasks.PAD_OFFSET_BY_ARM` -- a world-frame vector measured AT THE
        ANCHOR. Under a different orientation that vector points somewhere
        else entirely: at T1's level, inboard-pointing approach it is 111.8 mm
        out in the wrong direction, which is nearly four times the 30 mm
        capture window. The gate would then never fire and the hand would
        never close, which is a failure this file has already recorded twice
        under two other causes.

        So the offset is rebuilt from the orientation actually being sent,
        through the ONE EE-frame constant `grasp_frames.PAD_MID_EE`.
        """
        import grasp_frames as GF
        if orient:
            self.orient = {a: tuple(orient[a]) for a in ARMS if a in orient}
            for a in ARMS:
                self.orient.setdefault(a, tuple(self.orient[ARMS[0]]))
        return {a: [float(v) for v in
                    GF.q_matrix(self.orient[a]) @ np.asarray(GF.PAD_MID_EE)]
                for a in ARMS}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True,
                    choices=["a", "b", "c", "A", "B", "C",
                             # demo routines -- see the "demo" taskset
                             "d1", "d2", "d3", "D1", "D2", "D3",
                             "m1s2", "M1S2",
                             "m0", "m1", "m2", "m3",
                             "M0", "M1", "M2", "M3"])
    ap.add_argument("--taskset", default="study",
                    choices=["study", "clip", "msc", "demo"],
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
    # THE DEFAULT COMES FROM THE ENVIRONMENT so the recording sweep can set
    # ONE seed for a clip and have both the task path and the scene node use
    # it. The GUI buttons are fixed command lines (gui_launch_specs) and
    # cannot carry a per-run seed; an inherited environment variable can, and
    # it is inherited by construction rather than by two places remembering
    # to agree.
    ap.add_argument("--seed", type=int,
                    default=int(os.environ.get("SRL_TASK_SEED", "0")))
    ap.add_argument("--vision", default=None, metavar="CUBES_JSON",
                    help="T1: build the path from DETECTED cubes instead of "
                         "T1_CUBES / T1_PAIR. Takes the file written by "
                         "scripts/stage_observe_and_detect.py, which does the "
                         "looking during STAGING -- the look cannot happen "
                         "inside the run because it would put a second "
                         "publisher on the arm controller alongside "
                         "ik_follower_node. Refuses loudly rather than "
                         "falling back to the declaration.")
    # THE RUN STARTS FROM HOME. See require_home() above for the measurement
    # that made this necessary: run 2 of a session began 1.2338 rad from home
    # because run 1 left it there and nothing brought it back.
    ap.add_argument("--allow-unhomed", action="store_true",
                    help="record a run that does NOT start from home. Off by "
                         "default and loud when used: a trial that starts "
                         "somewhere nobody recorded cannot be compared with "
                         "one that did.")
    # THE TASK FROM A TYPED SENTENCE. M1 only, and only with --vision: the
    # instruction names colours, and the colours have to come from the camera
    # or the sentence is being grounded against the file it is supposed to be
    # independent of.
    ap.add_argument("--instruct", default=None, metavar="TEXT",
                    help="free-form instruction, e.g. \"put the blue ones on "
                         "the blue pad\". Parsed by srl_autonomy.voice_intent "
                         "and grounded against the DETECTED cubes by "
                         "experiments/abc/t1_instruction.py. Refuses on an "
                         "ambiguous instruction rather than choosing.")
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
        # A TASK MAY RESTRICT WHICH MODES IT RUNS UNDER, AND ONE DOES.
        #
        # T1 was rebuilt for `06_full_autonomy` alone: it commands its own
        # approach orientation instead of the pinned anchor, so running it
        # under a teleop mode would put an operator's pinned wrist against a
        # geometry that was never measured for it -- and any difference would
        # then be reported as a MODE effect, which is the one thing the
        # mode-independence property exists to prevent.
        #
        # Refused loudly rather than written in a caveat. The whole point of
        # the caveat field is that nothing enforces it.
        _ok_modes = spec.get("modes")
        if _ok_modes and a.mode not in _ok_modes:
            raise SystemExit(
                "REFUSING: task %s runs under %s only, and --mode is %s.\\n"
                "It commands its own approach orientation rather than the "
                "pinned anchor every teleop mode sends, so a run under %s "
                "would be measuring a different geometry under a mode's name."
                % (key, " and ".join(_ok_modes), a.mode, a.mode))
        scen = a.scenario or spec["scenario"]
        # THE SEED REACHES THE LAYOUT. `--seed` has existed since the runner
        # was written and went straight into the manifest, while the layout
        # was built by `spec["build"]()` with no arguments -- so stage 2,
        # whose entire definition is "positions drawn at random", ran the
        # SAME four cubes in every trial while each trial's manifest recorded
        # a different seed. A recorded seed that nothing reads is worse than
        # no seed: it is a claim of randomisation in the data file.
        #
        # `seeded` is declared on the task rather than inferred from the key,
        # so a future randomised task gets it by saying so.
        wp = (spec["build"](seed=a.seed) if spec.get("seeded")
              else spec["build"]())
        grip_sched = spec["grip"](len(wp["left"]))
    elif a.taskset == "demo":
        # THE CHOREOGRAPHED ROUTINES. They are not tasks: no grasp, no
        # object, no metric. They enter here so they are driven by the same
        # runner, filmed by the same sweep and bound by the same mode safety
        # invariant -- a demo with its own code path is a demo with its own
        # bugs, and worse, its own weaker guarantees.
        import choreography as CH
        if key.lower() not in CH.TASKS:
            raise SystemExit(
                "unknown demo key %r -- expected one of %s"
                % (key, ", ".join(sorted(CH.TASKS))))
        spec = CH.TASKS[key.lower()]
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

    if a.instruct and not a.vision:
        raise SystemExit(
            "--instruct needs --vision: the instruction names COLOURS, and "
            "grounding those against msc_clip_tasks.T1_PAIR instead of the "
            "camera would make a language demo out of a file lookup.")

    if a.dry_run:
        print("task %s / %s / mode %s : %d waypoints per arm, entry topic %s"
              % (key, scen, a.mode, len(wp["left"]),
                 MODES[a.mode]["topic"] % "<arm>"))
        print(MODES[a.mode]["note"])
        return 0

    rclpy.init()

    # ---- LOOK BEFORE GRASPING -------------------------------------------
    #
    # This is the join that was missing. `spec["build"]()` above produced the
    # path from T1_CUBES and T1_PAIR -- a declared position and a declared
    # colour -- so every recorded clip of a COLOUR-MATCHED task was made
    # without a camera being consulted. With --vision the same builder is fed
    # what the camera SAW instead.
    #
    # IT REFUSES RATHER THAN FALLING BACK. A blind pick that quietly reverts to
    # the declared coordinate is indistinguishable from a working perception
    # path, which is exactly the failure TASK_SPEC P-4 forbids.
    vision_info = None
    instruct_info = None
    if a.vision:
        _vt = _MSC_KEY.get(key)
        if a.taskset != "msc" or _vt not in ("t1", "t1s2"):
            raise SystemExit(
                "--vision is implemented for M1 and M1S2 (t1, t1s2) only; "
                "%s has no colour rule for a camera to ground." % key)
        if not os.path.exists(a.vision):
            raise SystemExit(
                "REFUSING TO RECORD: no detections at %s. Run "
                "scripts/stage_observe_and_detect.py during staging. A "
                "fallback to the declared coordinates here would look exactly "
                "like a working camera." % a.vision)
        vd = json.load(open(a.vision))
        cubes = [tuple(c) for c in vd["cubes"]]
        # WHICH LAYOUT THIS RUN IS. Stage 2 draws its cubes from the seed, so
        # "the layout" is a function of the seed and the detections have to
        # have been taken against THAT draw.
        import t1_task as _T1M
        if _vt == "t1s2":
            want = [list(c[:2]) for c in _T1M.stage2_layout(a.seed)]
        else:
            want = [list(c) for c in _T1M.T1_CUBES]
        if len(cubes) != len(want):
            raise SystemExit(
                "REFUSING TO RECORD: the look saw %d cubes, the task expects "
                "%d." % (len(cubes), len(want)))
        # THE DETECTIONS MUST BE OF THIS LAYOUT. A stale file from an earlier
        # layout would build a path to where the cubes USED to be, and every
        # check downstream would pass it.
        if vd.get("layout", {}).get("T1_CUBES") != want:
            raise SystemExit(
                "REFUSING TO RECORD: %s was written against a different "
                "layout (it saw %s, this run is %s). Re-run the staged "
                "detection." % (a.vision,
                                vd.get("layout", {}).get("T1_CUBES"), want))
        if _vt == "t1s2" and int(vd.get("seed", -1)) != int(a.seed):
            raise SystemExit(
                "REFUSING TO RECORD: the detections were taken at seed %s and "
                "this run is seed %s. The cubes are drawn from the seed, so "
                "those are two different tables."
                % (vd.get("seed"), a.seed))
        vision_info = vd.get("timing")
        print("[vision] %d cubes SEEN (staged) -> %s" % (len(cubes), cubes),
              flush=True)
        # ---- THE TASK CAN COME FROM A TYPED SENTENCE ----------------------
        #
        # Without --instruct the whole four-cube routine runs, which is what
        # every recorded clip has been. With it, WHICH cubes and WHICH pads
        # come from the instruction, grounded against what the camera SAW --
        # `t1_instruction.plan_from` is the only place the sentence and the
        # detections meet.
        #
        # IT REFUSES ON ASK AS WELL AS ON REFUSE. An ambiguous instruction
        # resolved by picking the first candidate would be a recorded clip of
        # the robot guessing, and a clip is evidence.
        if a.instruct:
            import t1_instruction as TI
            outcome = TI.plan_from(a.instruct, cubes)
            print("[instruct] %r -> %s: %s"
                  % (a.instruct, outcome.kind.upper(), outcome.message),
                  flush=True)
            if not outcome.ok:
                raise SystemExit(
                    "REFUSING TO RUN: the instruction did not resolve to a "
                    "plan (%s). %s" % (outcome.kind.upper(), outcome.message))
            for px, py, pad in outcome.picks:
                print("   pick (%.4f, %.4f) -> %s pad"
                      % (px, py, MCT.PLANE_COLOURS[pad]), flush=True)
            instruct_info = outcome.as_dict()
            wp = TI.build_path(outcome.picks)
        else:
            wp = _T1M.build(cubes=cubes)
        grip_sched = spec["grip"](len(wp["left"]))

    n = Runner(a)
    n._wire_logging()
    if vision_info is not None:
        n.vision_info = vision_info
    if instruct_info is not None:
        n.instruct_info = instruct_info
    # ---- START FROM HOME --------------------------------------------------
    #
    # BEFORE `--isolate`, and the order is not cosmetic: isolate() starts
    # vr_pose_mapper for the VR modes, and the mapper HOLDS THE ARMS against a
    # joint-space trajectory (measured, with a control either side, 2026-08-15).
    # Staging after it would lose to it. Before it, the controller is free.
    #
    # Idempotent -- an arm already within HOME_TOL_RAD costs nothing and no
    # publisher is created -- which is why the recording sweep, which stages
    # before it launches this, does not pay for it twice.
    _homed, _why = require_home(n, allow_unhomed=a.allow_unhomed)
    if not _homed:
        print("REFUSING TO RUN: %s" % _why)
        return 2
    if _why not in ("already home", "staged"):
        print("[home] --allow-unhomed: RUNNING ANYWAY. %s" % _why, flush=True)
    home_probe(n, "after require_home")

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
        home_probe(n, "after isolate")

    n.spin(2.0)
    # ==================================================================
    # THIS IS THE MOMENT THAT ANSWERS "DID THE RUN START FROM HOME".
    # ==================================================================
    # It is the LAST instant before the block below deliberately drives the
    # arms to waypoint 0 and settles them there, and getting that wrong cost a
    # session. The first version measured at the first iteration of the send
    # loop, called it "the first commanded waypoint", and read 0.7991 rad
    # (left) / 0.6249 (right) from home -- so it FAILED every clip, including
    # good ones, and looked exactly like the defect it was written to catch.
    #
    # The arms were fine. `run_abc` approaches the start on purpose, because
    # travel measured from wherever the previous run left the arm reported
    # 0.0118 m instead of 0.4761 m; the note on that block explains it. By the
    # time the loop runs, the arms are AT waypoint 0 -- 0.80 rad from home for
    # a left-arm task -- and the idle right arm is at `park(0.32)`, which is
    # the 0.6249. Both numbers are correct and neither is a fault.
    #
    # The probe trail is what settled it: 0.0000 at `require_home`, 0.0000
    # here, 0.7991 after the approach. A single reading at the far end cannot
    # tell "never homed" from "homed, then moved on purpose", and this project
    # has now paid for that distinction twice.
    _pre = home_probe(n, "before approach-the-start")
    write_home_record(a, n, _pre)
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
    # PER ARM. This was `CT.PAD_OFFSET`, which is the LEFT arm's wrist-to-pad
    # vector, and it is the THIRD consumer of that constant -- the task path
    # and the clip scene were both fixed to use the per-arm table and this one
    # was missed.
    #
    # It decides when the hand is allowed to close, so on the right arm, whose
    # offset differs by 48.3 mm, the gate compared a pad position that was
    # 48 mm wrong against a 30 mm window and never fired. Measured on the
    # first mode-01 sweep after the other two were fixed: T1's pads reached
    # the cube to within 0.1 mm -- the geometry was right -- and the knuckle
    # stayed at 0.0 for the entire clip. "NO GRASP RECORDED at all" on a run
    # where the arm went exactly where it was told.
    # REBUILT FROM THE ORIENTATION THIS TASK ACTUALLY SENDS. See
    # Runner.set_orient(): at the anchor it reproduces
    # CT.PAD_OFFSET_BY_ARM exactly, and under a task-declared approach
    # it follows the hand instead of pointing where the hand used to.
    PAD_OFF_BY_ARM = n.set_orient(spec.get("orient"))
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
            # WHERE THE ARMS WERE WHEN THE TASK STARTED, per arm, in radians
            # from config/home_positions_{arm}.txt. A trial that started off
            # home is now identifiable in the data rather than assumed
            # comparable -- the same reason the sweep records `opened_on`.
            start_home_err_rad=getattr(n, "start_home_err", None),
            # THE SENTENCE THAT COMMANDED THIS TRIAL, and what it resolved to.
            # A trial driven by language whose language is not in the data
            # cannot be attributed to the language.
            instruction=(a.instruct or None),
            instruction_plan=getattr(n, "instruct_info", None),
            started_unhomed=bool(a.allow_unhomed),
            sim_only=True,
            note="mock hardware echoes commands with no dynamics; every "
                 "tracking figure here is a property of the mock")
        tl.start(a.trial_index, condition=a.mode, scenario=scen,
                 target_id=key, arm="both", block=a.block)

    n_sent = 0
    for k in range(len(wp["left"])):
        if k == 0:
            # WHERE THE ARMS ARE ONCE THE APPROACH HAS PUT THEM ON WAYPOINT 0.
            #
            # RECORDED, NOT JUDGED. The verdict on "did the run start from
            # home" is taken before the approach block above -- see the long
            # note there. By this line the arms are SUPPOSED to be at waypoint
            # 0, so a large number here is the task's own geometry: about
            # 0.80 rad for a left-arm T1 pick, and the idle arm sitting at
            # `park()`. Gating on it failed every good clip.
            #
            # It is still worth having, because it is what a FRAME shows: the
            # opening frame of a clip is the arm at waypoint 0, not at home,
            # and knowing the expected value is how you tell a correct opening
            # frame from a stale one.
            n.spin(0.4)
            _post = home_probe(n, "at waypoint 0 (after the approach)")
            n.first_wp_home_err = _post["per_arm"]
            write_home_record(a, n, _pre, _post)
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
                        _off = PAD_OFF_BY_ARM[arm]
                        pads = (None if here is None else
                                [here[i] + _off[i] for i in range(3)])
                        obj = pend_obj[arm] or grip_obj
                        if (pads is not None and obj is not None
                                and math.dist(pads, obj) <= ARRIVE_TOL_M):
                            # SAY WHICH OBJECT, AS IT HAPPENS.
                            #
                            # A run under 06 is the system deciding what to do
                            # and then doing it, and until now the only visible
                            # trace of the doing was the arm moving. The GUI's
                            # prompt panel reads these lines to say which cube
                            # it is on and which pad it chose; they also land
                            # in the run log, so a clip that went wrong can be
                            # read back without re-deriving the schedule.
                            print("[progress] %s arm %s at (%.3f, %.3f, %.3f)"
                                  % (arm,
                                     "CLOSED on" if pend[arm] else "RELEASED over",
                                     obj[0], obj[1], obj[2]), flush=True)
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
    _she = getattr(n, "start_home_err", None)
    print("   started %s home: %s"
          % ("AT" if _she and max(_she.values()) <= HOME_TOL_RAD else "OFF",
             "UNKNOWN -- no joint state" if not _she else
             ", ".join("%s %.4f rad" % (k, v) for k, v in sorted(_she.items()))))
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
            # THE POSE AT THE FIRST COMMANDED WAYPOINT, in the data rather than
            # only in the console. `start_home_err_rad` in the manifest is the
            # PRE-RUN check; this is the moment the task actually commanded
            # something, which is the only one a clip can be compared against.
            # THE VERDICT IS THE PRE-APPROACH READING. The post-approach one is
            # kept alongside it because it is what the opening FRAME shows, but
            # it is not "did the run start from home" -- by then the approach
            # has deliberately put the arms on waypoint 0.
            start_home_err_rad=getattr(n, "start_home_err", None),
            at_home_before_approach=int(bool(
                _pre.get("worst_rad") is not None
                and _pre["worst_rad"] <= HOME_TOL_RAD)),
            waypoint0_home_err_rad=getattr(n, "first_wp_home_err", None),
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
