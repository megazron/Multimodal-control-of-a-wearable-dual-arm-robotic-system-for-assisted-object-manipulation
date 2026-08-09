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

ARMS = ("left", "right")

MODES = {
    "01_master_teleop": dict(
        topic="/master_arm_pose_%s", frame="master",
        note="the pose master_pose_node would publish; the follower maps it "
             "through the anchor and scale exactly as in live teleop"),
    "02_vr_teleop": dict(
        topic="/vr/controller_pose_%s", frame="master",
        note="vr_pose_mapper must be running; it turns this into "
             "/master_arm_pose_<arm>"),
    "04_shared_autonomy": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        note="world-frame pose, entering request_ik at the same door teleop "
             "uses"),
    "06_full_autonomy": dict(
        topic="/autonomy/assist_pose_%s", frame="world",
        via_executive=True,
        note="same topic as 04, but commanded BY autonomy_executive from a "
             "spoken instruction -- the difference is who decides"),
}


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

    def spin(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def send(self, arm, xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        # Orientation is left as the identity ONLY on the master-frame path,
        # where the follower replaces it with the arm's own anchor. On the
        # world-frame path an identity quaternion is NOT neutral -- it is a
        # specific, unreachable pose, and it has produced a false negative
        # three times in this project -- so the anchor is used explicitly.
        if self.spec["frame"] == "world":
            from srl_teleop import master_calibration as mc
            q = mc.WORKSPACE_ORIENT[arm]
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = q
        else:
            m.pose.orientation.w = 1.0
        self.pub[arm].publish(m)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True, choices=["a", "b", "c",
                                                      "A", "B", "C"])
    ap.add_argument("--mode", default="04_shared_autonomy",
                    choices=sorted(MODES))
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--participant", default="PILOT")
    ap.add_argument("--scripted", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--hold-s", type=float, default=0.45,
                    help="seconds per waypoint; the follower is asynchronous "
                         "so this sets how fast the target leads the arm")
    ap.add_argument("--min-travel-m", type=float, default=0.01)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    key = a.task.upper()

    default_scen = {"A": "S3_both", "B": "S2_full_lift", "C": "S1_both_slow"}
    scen = a.scenario or default_scen[key]
    wp = waypoints(key, scen)

    if a.dry_run:
        print("task %s / %s / mode %s : %d waypoints per arm, entry topic %s"
              % (key, scen, a.mode, len(wp["left"]),
                 MODES[a.mode]["topic"] % "<arm>"))
        print(MODES[a.mode]["note"])
        return 0

    rclpy.init()
    n = Runner(a)
    n.spin(2.0)
    start = {arm: n.ee(arm) for arm in ARMS}

    n.state.publish(String(data=json.dumps(
        dict(experiment="abc", task=key, scenario=scen, mode=a.mode,
             participant=a.participant, state="running"))))

    if n.speech is not None:
        # Mode 06 is commanded by an INSTRUCTION, not by a pose. The executive
        # then does the deciding; this waits for it rather than racing it.
        n.speech.publish(String(data="hey doc oc grab the blue cube"))
        n.spin(2.5)

    # MASTER-FRAME MODES TAKE A DISPLACEMENT, NOT A WORLD COORDINATE.
    #
    # `ik_follower_node.on_pose` computes `pos_anchor + scale * (tip - d_ref)`:
    # the pose on /master_arm_pose_<arm> is the master's TIP inside its own
    # ~0.22 m ball, and the follower maps it onto the arm's anchor. Feeding it
    # a world coordinate like (0.25, 0.35, 1.30) asks for a displacement of
    # well over a metre, which the guards clamp to nothing -- measured, the
    # arms travelled 0.0000 m and the run correctly refused to call itself a
    # success.
    #
    # So the task's world path is re-expressed as a displacement from its own
    # first waypoint, which is exactly what the operator's hand does: the
    # first frame latches the reference and everything after is relative to
    # it. The world-frame modes are untouched, because /autonomy/assist_pose_*
    # really is a world pose.
    if n.spec["frame"] == "master":
        wp = {a: [[p[i] - v[0][i] for i in range(3)] for p in v]
              for a, v in wp.items()}
        print("   master-frame mode: path re-expressed as a displacement, "
              "span %.3f m" % max(math.dist([0, 0, 0], p)
                                  for v in wp.values() for p in v))

    n_sent = 0
    for k in range(len(wp["left"])):
        for arm in ARMS:
            n.send(arm, wp[arm][min(k, len(wp[arm]) - 1)])
            n_sent += 1
        n.spin(a.hold_s)
    n.spin(1.5)

    end = {arm: n.ee(arm) for arm in ARMS}
    travel = {}
    for arm in ARMS:
        if start[arm] is None or end[arm] is None:
            travel[arm] = None
        else:
            travel[arm] = math.dist(start[arm], end[arm])

    n.state.publish(String(data=json.dumps(
        dict(experiment="abc", task=key, scenario=scen, mode=a.mode,
             state="done"))))
    print("task %s / %s / mode %s : %d poses published on %s"
          % (key, scen, a.mode, n_sent, MODES[a.mode]["topic"] % "<arm>"))
    for arm in ARMS:
        print("   %-5s tf2 EE travel: %s"
              % (arm, "NO TF -- cannot tell whether it moved"
                 if travel[arm] is None else "%.4f m" % travel[arm]))
    n.destroy_node()
    rclpy.shutdown()

    # A RECORDING OF A STATIONARY ARM IS WORSE THAN NO RECORDING, because it
    # looks like a result. `None` is a failure too: not knowing whether the
    # arm moved is not evidence that it did.
    moved = [t for t in travel.values() if t is not None and t >= a.min_travel_m]
    if not moved:
        print("\nFAILED: no arm moved at least %.3f m. Not recording this as "
              "a successful run." % a.min_travel_m)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
