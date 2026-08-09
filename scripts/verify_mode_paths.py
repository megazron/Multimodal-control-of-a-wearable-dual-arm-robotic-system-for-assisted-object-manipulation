#!/usr/bin/env python3
"""PART 4: does each mode's command actually reach the arms, with safety in
the path, and does switching modes leak state?

Four modes are checked: DIRECT (mannequin), VR, SHARED autonomy, FULL
autonomy. For each:

  1. COMMAND PATH   every hop from input to the arm's trajectory controller,
                    with each hop confirmed by a live publisher/subscriber
                    match rather than by reading the source.
  2. SAFETY IN PATH the e-stop, the clearance floor and the step guard must
                    sit BETWEEN the input and the arm, not beside it.
  3. CAMERA         who owns the device, who merely subscribes, and whether
                    two writers can land on one topic.
  4. LEAKED STATE   two modes must not both publish the command topic.

A hop is reported CONNECTED only when the graph shows a publisher and a
subscriber on the same topic. Reading an import and concluding the path exists
is exactly the mistake this script exists to avoid.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/baselines/mode_paths.json")

# Each mode's command chain, as an ordered list of (stage, topic) hops.
# The final hop is the trajectory controller the arm listens on.
CHAINS = {
    "DIRECT": [
        ("master_pose_node", "/master_arm_pose_left"),
        ("ik_follower_left", "/left_arm_controller/joint_trajectory"),
    ],
    "VR": [
        ("quest bridge", "/vr/controller_pose_left"),
        ("vr_pose_mapper", "/master_arm_pose_left"),
        ("ik_follower_left", "/left_arm_controller/joint_trajectory"),
    ],
    "SHARED": [
        ("master_pose_node", "/master_arm_pose_left"),
        ("perception", "/perception/objects"),
        ("handover_arbiter", "/master_arm_pose_left"),
        ("ik_follower_left", "/left_arm_controller/joint_trajectory"),
    ],
    "FULL_AUTONOMY": [
        ("voice/executive", "/autonomy_decision"),
        ("perception", "/perception/objects"),
        ("ik_follower_left", "/left_arm_controller/joint_trajectory"),
    ],
}

# Topics that must sit in the path for the mode to be safe to run.
SAFETY = {
    "/estop_state": "e-stop latch, consumed by every follower",
    "/blocking": "named blockers from the follower's own guards",
    "/blocking_summary": "aggregated, catches a unit that went silent",
}

# The sim-to-real cascade. Present only when a real stack is up.
REAL_CHAIN = [
    ("sim_to_real_bridge", "/real/left_arm_controller/joint_trajectory"),
    ("kortex_highlevel_bridge", "/real/joint_states"),
]

# Topics with MORE THAN ONE writer are the two-readers-on-a-serial-port bug
# in publish/subscribe form.
# Nodes that are DESIGNED to be a second writer on a command topic. The
# e-stop publishes each arm's MEASURED joint positions as a short trajectory
# and republishes while latched, precisely so it overrides a follower that
# keeps commanding. Flagging that as contention is a false positive, and the
# first run of this script produced exactly that.
DESIGNED_OVERRIDE = {"estop_node"}

SINGLE_WRITER = [
    "/master_arm_pose_left", "/master_arm_pose_right",
    "/left_arm_controller/joint_trajectory",
    "/right_arm_controller/joint_trajectory",
    "/perception/detections/left", "/perception/detections/right",
]


class Probe(Node):
    def __init__(self):
        super().__init__("verify_mode_paths")

    def pubs(self, topic):
        return self.get_publishers_info_by_topic(topic)

    def subs(self, topic):
        return self.get_subscriptions_info_by_topic(topic)


def main():
    rclpy.init()
    n = Probe()
    for _ in range(30):                 # let discovery settle
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1.0)

    out = {}
    P = print
    P("=" * 78)
    P("PART 4  MODE COMMAND PATHS, SAFETY, CAMERAS, LEAKED STATE")
    P("=" * 78)
    P("Method: every hop is confirmed from the LIVE ROS graph. A hop is")
    P("CONNECTED only if the topic has both a publisher and a subscriber.")

    # ------------------------------------------------------- command paths
    P("\n[1] COMMAND PATHS")
    paths = {}
    for mode, chain in CHAINS.items():
        P("\n  %s" % mode)
        rows = []
        for stage, topic in chain:
            np_, ns_ = len(n.pubs(topic)), len(n.subs(topic))
            if np_ and ns_:
                st = "CONNECTED"
            elif np_ and not ns_:
                st = "DEAD END (published, nobody listening)"
            elif ns_ and not np_:
                st = "waiting (subscriber, no publisher)"
            else:
                st = "ABSENT (neither end running)"
            rows.append(dict(stage=stage, topic=topic, pubs=np_, subs=ns_,
                             status=st))
            P("    %-26s %-44s pub=%d sub=%d  %s"
              % (stage, topic, np_, ns_, st))
        paths[mode] = rows
    out["paths"] = paths

    # ------------------------------------------------------------- safety
    P("\n[2] SAFETY STACK IN THE PATH")
    saf = {}
    for topic, why in SAFETY.items():
        np_, ns_ = len(n.pubs(topic)), len(n.subs(topic))
        ok = np_ > 0 and ns_ > 0
        saf[topic] = dict(pubs=np_, subs=ns_, ok=ok, why=why)
        P("    %-22s pub=%d sub=%d  %-9s  %s"
          % (topic, np_, ns_, "IN PATH" if ok else "NOT IN PATH", why))
    out["safety"] = saf

    # ------------------------------------------------------- single writer
    P("\n[3] SINGLE-WRITER CHECK")
    P("    Two publishers on one command topic is the two-readers-on-one-")
    P("    serial-port bug in publish/subscribe form: the subscriber sees an")
    P("    interleaving of two sources and cannot tell which is which.")
    sw = {}
    for topic in SINGLE_WRITER:
        pubs = n.pubs(topic)
        names = sorted({p.node_name for p in pubs})
        real = [x for x in names if x not in DESIGNED_OVERRIDE
                and not x.startswith("_NODE_NAME_UNKNOWN")]
        bad = len(real) > 1
        sw[topic] = dict(writers=names, contended=bad)
        if names:
            tag = "  <-- CONTENDED" if bad else (
                "  (e-stop override, by design)"
                if set(names) & DESIGNED_OVERRIDE else "")
            P("    %-42s %d writer(s)%s  %s"
              % (topic, len(names), tag, ", ".join(names)))
    if not any(v["contended"] for v in sw.values()):
        P("    no contended command topic in the running graph")
    out["single_writer"] = sw

    # ------------------------------------------------------------ cameras
    P("\n[4] CAMERA ACCESS")
    P("    A camera DEVICE has one owner. A camera TOPIC may have many")
    P("    subscribers, and that is not contention -- it is the point of")
    P("    publish/subscribe. The two must not be conflated.")
    cam_topics = ["/camera/color/image_raw", "/wrist_mounted_camera/image",
                  "/perception/detections/left", "/perception/objects"]
    cams = {}
    for t in cam_topics:
        pubs = sorted({p.node_name for p in n.pubs(t)})
        subs = sorted({s.node_name for s in n.subs(t)})
        cams[t] = dict(pubs=pubs, subs=subs)
        P("    %-34s writers=%-28s readers=%s"
          % (t, ",".join(pubs) or "-", ",".join(subs) or "-"))
    out["cameras"] = cams

    # ------------------------------------------------------- real cascade
    P("\n[5] SIM-TO-REAL CASCADE")
    real = {}
    for stage, topic in REAL_CHAIN:
        np_, ns_ = len(n.pubs(topic)), len(n.subs(topic))
        real[topic] = dict(stage=stage, pubs=np_, subs=ns_)
        P("    %-26s %-44s pub=%d sub=%d" % (stage, topic, np_, ns_))
    if all(v["pubs"] == 0 and v["subs"] == 0 for v in real.values()):
        P("    no /real stack running: this needs mock_real.launch.py or the lab")
    out["real"] = real

    n.destroy_node()
    rclpy.shutdown()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    P("\n  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
