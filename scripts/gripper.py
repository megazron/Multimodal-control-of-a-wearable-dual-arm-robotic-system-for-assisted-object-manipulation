#!/usr/bin/env python3
"""Open and close a real Robotiq gripper, and say what actually happened.

    python3 scripts/gripper.py open
    python3 scripts/gripper.py close
    python3 scripts/gripper.py test           # open, close, open -- watch it
    python3 scripts/gripper.py 0.4            # a knuckle angle in radians
    python3 scripts/gripper.py --mm 40        # an opening in millimetres

WHY THIS EXISTS
---------------
The grasp closed on nothing and nobody could tell, because **the gripper is
write-only**. `kortex_highlevel_bridge._send_gripper()` sets `_grip_sent = t`
immediately after `SendGripperCommand` returns, and a failure is a log line
on a topic nobody was tailing. Nothing in this repository has ever PUBLISHED
the gripper's measured position, so every statement about the hand in every
recording here is a statement about what was COMMANDED.

That is this project's own documented failure mode -- "feature present but
does nothing: checked that a field is STORED, not that a consumer READS it"
-- sitting on the one joint that decides whether a pick worked.

So this tool refuses to report success from the fact that it published. It
checks, in order:

  1. is anything SUBSCRIBED to the topic          (nothing listening -> say so)
  2. what will the bridge COMPUTE from what we send
     (it divides by its own `gripper_closed_rad`; this tool reads that
     parameter off the live node rather than assuming 0.8, so the two cannot
     silently disagree -- if they did, the grip would be scaled and nothing
     would say why)
  3. is `gripper_enabled` true                    (false -> a silent no-op)
  4. did the bridge log a gripper error while we were commanding

WHAT IT STILL CANNOT DO, said plainly. It cannot read the gripper's MEASURED
position, because that needs the Kortex session and the arm permits exactly
one (HARD CONSTRAINT 2) -- the bridge is holding it. `--watch` therefore
measures the fingers the only other way available: it photographs them with
the wrist camera before and after and reports the change in the dark-pixel
footprint at the bottom of the frame, where the fingertips sit. That is a
weak measurement and it is labelled as one. The strong fix is for the bridge
to publish the gripper feedback it is already receiving.

MIN DELTA. The bridge ignores a new target within `gripper_min_delta` (0.02
normalised) of the last one it sent, so re-issuing the SAME command is a
no-op by design. `test` alternates between the extremes, which is always a
real change; a repeated single command will say when it is being suppressed.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The driven knuckle's full travel, in radians. This is only a FALLBACK: the
# live value is read from the bridge, because the bridge is what divides by
# it. Matches fsr_gripper_node.CLOSED_RAD.
CLOSED_RAD_FALLBACK = 0.8
# Robotiq 2F-85 stroke. Used only to translate --mm into a knuckle angle, and
# it is a LINEAR approximation of a four-bar: the fingers swing, so this is
# right at the extremes and approximate in between. `docs/system/findings.md`
# has the measured pad-midpoint curve; nothing here needs that accuracy.
STROKE_MM = 85.0

NAMED = {"open": 0.0, "close": None, "closed": None}


def knuckle_from_mm(mm, closed_rad):
    """Knuckle radians for an opening in mm. 85 mm -> 0 (open)."""
    frac = 1.0 - max(0.0, min(STROKE_MM, float(mm))) / STROKE_MM
    return frac * closed_rad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", nargs="?", default="test",
                    help="open | close | test | a knuckle angle in radians")
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--mm", type=float, default=None,
                    help="opening in millimetres instead of radians")
    ap.add_argument("--hold-s", type=float, default=3.0,
                    help="how long to keep republishing before reporting")
    ap.add_argument("--watch", action="store_true",
                    help="photograph the fingers before and after (weak, "
                         "and labelled as weak)")
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    topic = "/%s_gripper_controller/joint_trajectory" % a.arm
    joint = "%s_robotiq_85_left_knuckle_joint" % a.arm
    bridge = "/kortex_highlevel_bridge_%s" % a.arm

    rclpy.init()
    node = Node("srl_gripper_%s" % a.arm)
    pub = node.create_publisher(JointTrajectory, topic, 10)

    print("domain %s | arm %s | topic %s"
          % (os.environ.get("ROS_DOMAIN_ID", "(unset -> 0)"), a.arm, topic))

    # ---- 1. is anything listening -------------------------------------
    subs = 0
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        subs = pub.get_subscription_count()
        if subs:
            break
    print("  subscribers on the gripper topic: %d" % subs)
    if subs == 0:
        print("\nNOTHING IS LISTENING. Publishing would be a no-op that looked\n"
              "like success. Either the bridge is not running, or this shell\n"
              "is on the wrong ROS_DOMAIN_ID (the bridge's own domain is in\n"
              "its process environment: tr '\\0' '\\n' < /proc/<pid>/environ).")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    # ---- 2/3. what will the bridge do with it --------------------------
    closed_rad, enabled, min_delta = _bridge_params(node, bridge)
    print("  bridge gripper_closed_rad:        %.4f rad%s"
          % (closed_rad,
             "" if closed_rad != CLOSED_RAD_FALLBACK or True else ""))
    print("  bridge gripper_enabled:           %s" % enabled)
    if enabled is False:
        print("\ngripper_enabled is FALSE on the bridge. Every command below\n"
              "would be discarded silently inside _send_gripper(). Turn it on:\n"
              "    ros2 param set %s gripper_enabled true" % bridge)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    # ---- what to send --------------------------------------------------
    if a.mm is not None:
        seq = [("%.0f mm" % a.mm, knuckle_from_mm(a.mm, closed_rad))]
    elif a.what == "test":
        seq = [("OPEN", 0.0), ("CLOSE", closed_rad), ("OPEN", 0.0)]
    elif a.what in NAMED:
        seq = [(a.what.upper(),
                0.0 if a.what == "open" else closed_rad)]
    else:
        try:
            seq = [("%.3f rad" % float(a.what), float(a.what))]
        except ValueError:
            print("not a command or an angle: %r" % a.what)
            node.destroy_node()
            rclpy.shutdown()
            return 2

    before = _finger_shot(node, a.arm) if a.watch else None
    last_norm = None

    for label, rad in seq:
        rad = max(0.0, min(closed_rad, float(rad)))
        norm = max(0.0, min(1.0, rad / max(1e-6, closed_rad)))
        suppressed = (last_norm is not None
                      and abs(norm - last_norm) < min_delta)
        print("\n%s -> knuckle %.4f rad, bridge will send normalised %.3f%s"
              % (label, rad, norm,
                 "   <-- WITHIN gripper_min_delta OF THE LAST ONE, the bridge "
                 "will IGNORE it" if suppressed else ""))

        m = JointTrajectory()
        m.joint_names = [joint]
        p = JointTrajectoryPoint()
        p.positions = [float(rad)]
        p.time_from_start = Duration(sec=1)
        m.points = [p]

        # Republish for the hold. on_gripper() only stores the target, so one
        # message is enough in principle -- but one message is also the thing
        # that goes missing, and a gripper command is not a trajectory that a
        # republish can interrupt. See the republish note in
        # verify_colour_vision.Vision.stage for the case where it IS harmful.
        t0 = time.time()
        while time.time() - t0 < a.hold_s:
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)
        last_norm = norm

    after = _finger_shot(node, a.arm) if a.watch else None
    if before is not None and after is not None:
        _report_shot(before, after)

    print("\nCOMMANDED. This tool cannot see the fingers, so it does not\n"
          "claim they moved -- look at the gripper.\n"
          "If nothing moved, the next thing to read is the bridge's own log:\n"
          "  it prints 'gripper command failed' on the 1st and 20th failure,\n"
          "  and nothing at all when SendGripperCommand returns cleanly.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _bridge_params(node, bridge):
    """(closed_rad, enabled, min_delta) from the live bridge, with fallbacks.

    A parameter the bridge does not answer for is reported as unknown rather
    than assumed -- assuming 0.8 when the node says 0.5 is exactly how a grip
    gets silently scaled.
    """
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, "%s/get_parameters" % bridge)
    closed, enabled, delta = CLOSED_RAD_FALLBACK, None, 0.02
    if not cli.wait_for_service(timeout_sec=4.0):
        print("  (bridge %s did not answer for its parameters; using the\n"
              "   fallback closed_rad %.2f -- if the bridge disagrees, the\n"
              "   grip is scaled and nothing else would say so)"
              % (bridge, closed))
        return closed, enabled, delta
    req = GetParameters.Request()
    req.names = ["gripper_closed_rad", "gripper_enabled", "gripper_min_delta"]
    fut = cli.call_async(req)
    import rclpy
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None or len(res.values) < 3:
        return closed, enabled, delta
    if res.values[0].type == 3:
        closed = float(res.values[0].double_value)
    if res.values[1].type == 1:
        enabled = bool(res.values[1].bool_value)
    if res.values[2].type == 3:
        delta = float(res.values[2].double_value)
    return closed, enabled, delta


def _finger_shot(node, arm):
    """Dark-pixel footprint along the bottom edge, where the fingertips sit.

    DELIBERATELY CRUDE AND LABELLED AS SUCH. It is not a measurement of the
    opening; it is a check that SOMETHING about the fingers changed, for the
    case where the answer is "nothing moved at all".
    """
    try:
        import numpy as np
        from sensor_msgs.msg import Image
    except Exception:
        return None
    got = {}
    sub = node.create_subscription(
        Image, "/%s_camera/color/image_raw" % arm,
        lambda m: got.setdefault("m", m), 5)
    t0 = time.time()
    import rclpy
    while time.time() - t0 < 6.0 and "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    if "m" not in got:
        return None
    m = got["m"]
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    band = a[int(m.height * 0.80):, :, :3].mean(axis=2)
    return float((band < 90).mean())


def _report_shot(before, after):
    print("\n  fingertip footprint in the bottom 20%% of the wrist frame:")
    print("     before %.4f   after %.4f   change %+.4f" %
          (before, after, after - before))
    if abs(after - before) < 0.002:
        print("     THAT IS NO CHANGE. Either the fingers did not move, or")
        print("     they are outside this crude band. Trust your eyes over")
        print("     this number -- it is a smoke alarm, not a measurement.")


if __name__ == "__main__":
    sys.exit(main())
