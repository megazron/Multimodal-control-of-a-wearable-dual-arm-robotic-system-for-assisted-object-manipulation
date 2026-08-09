#!/usr/bin/env python3
"""JOB D: are the six modes real, or a bluff? Verified from the LIVE GRAPH.

WHY THE LIVE GRAPH AND NOT THE SOURCE. A node that calls create_publisher on a
topic nothing subscribes to looks perfectly connected in the source: the
publish call is there, the topic name matches the docstring, and every test
that checks "did it publish" passes. The VR bridge did exactly that once.
Only the graph knows whether anything is LISTENING.

A hop is DEAD if it has publishers and zero subscribers. That is the shape
this looks for, per mode, with the mode's nodes actually running.

Reading the result: this establishes whether a command can REACH the arm. It
does not establish that the arm does the right thing when it arrives -- that
is what the task scenarios measure.
"""
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/baselines/mode_paths.json")

# The command chain each mode relies on, LAST HOP FIRST being the one that
# actually moves the arm. Every hop must have >=1 publisher AND >=1 subscriber.
CHAINS = {
    1: ("DIRECT_MANNEQUIN", ["/master_arm_pose_left",
                             "/left_arm_controller/joint_trajectory"]),
    2: ("DIRECT_VR", ["/vr_pose_left", "/master_arm_pose_left",
                      "/left_arm_controller/joint_trajectory"]),
    3: ("ORIENTATION_ASSIST", ["/master_arm_pose_left",
                               "/left_arm_controller/joint_trajectory"]),
    4: ("SHARED_AUTONOMY", ["/detections", "/autonomy/grasp_left",
                            "/autonomy/assist_pose_left",
                            "/left_arm_controller/joint_trajectory"]),
    5: ("SUPERVISED_AUTO", ["/voice_transcript", "/autonomy_stage",
                            "/autonomy/assist_pose_left",
                            "/left_arm_controller/joint_trajectory"]),
    6: ("FULL_AUTONOMY", ["/voice_transcript", "/autonomy_stage",
                          "/autonomy/assist_pose_left",
                          "/left_arm_controller/joint_trajectory"]),
}

# Safety mechanisms that must be LISTENED TO somewhere in the running graph.
SAFETY = ["/estop", "/estop_state", "/blocking"]

# Nodes to bring up per mode, beyond the base sim stack.
EXTRA = {
    2: [("srl_vr_teleop", "vr_pose_mapper"),
        ("srl_vr_teleop", "quest_vendor_bridge")],
    4: [("srl_autonomy", "grasp_generator"), ("srl_autonomy", "intent_inference"),
        ("srl_autonomy", "handover_arbiter")],
    5: [("srl_autonomy", "autonomy_executive")],
    6: [("srl_autonomy", "autonomy_executive"), ("srl_autonomy", "voice_intent")],
}


class Probe(Node):
    def __init__(self):
        super().__init__("mode_path_probe")

    def hop(self, topic):
        return (len(self.get_publishers_info_by_topic(topic)),
                len(self.get_subscriptions_info_by_topic(topic)))


def run(pkg, exe):
    return subprocess.Popen(["ros2", "run", pkg, exe],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            start_new_session=True)


def main():
    rclpy.init()
    p = Probe()
    for _ in range(30):
        rclpy.spin_once(p, timeout_sec=0.1)

    results = {}
    for mode, (name, chain) in CHAINS.items():
        procs = [run(pk, ex) for pk, ex in EXTRA.get(mode, [])]
        if procs:
            time.sleep(9.0)
        for _ in range(30):
            rclpy.spin_once(p, timeout_sec=0.1)
        hops = {}
        for t in chain:
            pub, sub = p.hop(t)
            hops[t] = dict(pub=pub, sub=sub,
                           state=("DEAD END" if pub and not sub else
                                  "no publisher" if not pub else "ok"))
        results[mode] = dict(name=name, hops=hops)
        for q in procs:
            try:
                os.killpg(os.getpgid(q.pid), 15)
            except Exception:
                pass
        if procs:
            time.sleep(2.0)

    print("=" * 78)
    print("MODE COMMAND PATHS, FROM THE LIVE ROS GRAPH")
    print("=" * 78)
    for mode, r in results.items():
        broken = [t for t, h in r["hops"].items() if h["state"] != "ok"]
        verdict = ("WORKS" if not broken else
                   "DOES NOT WORK" if any(
                       r["hops"][t]["state"] == "DEAD END" for t in broken)
                   else "PARTIALLY WORKS")
        print("\nmode %d  %-20s -> %s" % (mode, r["name"], verdict))
        for t, h in r["hops"].items():
            print("    %-42s pub %-3d sub %-3d  %s"
                  % (t, h["pub"], h["sub"], h["state"]))
        r["verdict"] = verdict

    print("\n" + "=" * 78)
    print("SAFETY MECHANISMS -- listened to in the running graph?")
    print("=" * 78)
    saf = {}
    for t in SAFETY:
        pub, sub = p.hop(t)
        saf[t] = dict(pub=pub, sub=sub)
        print("  %-22s pub %-3d sub %-3d  %s"
              % (t, pub, sub, "ok" if sub else "NOBODY LISTENING"))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(modes=results, safety=saf), open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    p.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
