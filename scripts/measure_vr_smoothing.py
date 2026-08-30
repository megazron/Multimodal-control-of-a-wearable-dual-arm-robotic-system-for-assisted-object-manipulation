#!/usr/bin/env python3
"""Measure the VR teleoperation smoothing THROUGH THE REAL NODE.

    python3 scripts/measure_vr_smoothing.py
    python3 scripts/measure_vr_smoothing.py --self-test

WHY THIS EXISTS AND NOT JUST THE FILTER'S OWN SELF-TEST. `vr_smoothing` proves
the maths on constructed signals. It cannot prove that `vr_pose_mapper` USES
it: the repeated failure mode in this repository is a feature that is present
and does nothing -- checked that a value is STORED, not that a consumer READS
it. So this drives the actual `VrPoseMapper` object, publishes real controller
poses into its real callbacks, and reads what it really commanded on
`/vr/cmd_pose_*`.

WHAT IS STUBBED, AND WHY IT IS HONEST TO STUB IT. Exactly one thing: the robot
pose the clutch latches its anchor against, which normally comes from TF and
therefore from a running simulation. Standing up a whole sim would test MoveIt
and the URDF, not the filter, and would fall foul of HARD CONSTRAINT 3 if
anything else were up. Everything on the path being measured -- the callbacks,
the clutch, the alignment yaw, the filter, the rate limiter, the publisher --
is the shipped code.

GROUND TRUTH IS CONSTRUCTED, so a bad answer is the filter's fault and not a
renderer's:
    STILL   a stationary hand plus Gaussian tremor. The right answer is "the
            command does not move", so output RMS about the mean is the score
            and lower is better.
    REACH   a hand at constant speed. The right answer is a straight line, so
            the lag is exactly computable and lower is better.
The two are scored together because either alone can be won by detuning the
other way, which is the whole reason a fixed EMA could not be tuned out of
this problem.
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_vr_teleop")
from srl_vr_teleop.vr_pose_mapper import VrPoseMapper, q_norm  # noqa: E402

RATE = 72.0
ANCHOR_P = np.array([0.40, 0.20, 1.10])
ANCHOR_Q = q_norm(np.array([0.0, 0.0, 0.0, 1.0]))


class Rig:
    """One mapper, driven by hand, with only the robot pose stubbed."""

    def __init__(self, params):
        self.node = VrPoseMapper()
        # THE ONLY STUB. Returns a fixed pose, so the anchor is exact and any
        # motion in the command is the mapper's doing rather than the sim's.
        self.node._robot_pose = lambda arm: (ANCHOR_P.copy(), ANCHOR_Q.copy())
        for k, v in params.items():
            self.node.set_parameters([rclpy.parameter.Parameter(k, value=v)])
        # Rebuild the smoothers so the parameters just set are the ones in
        # force -- they are read at construction, which is exactly the kind of
        # "set but never read" this script exists to catch.
        for h in ('left', 'right'):
            self.node.pfilt[h] = self.node._make_pfilt()
            self.node.qfilt[h] = self.node._make_qfilt()
        self.got = []
        # SAMPLED AT THE PUBLISHER, not over the wire.
        #
        # Subscribing to /vr/cmd_pose_left from inside the same node received
        # ZERO of the messages the node was demonstrably publishing --
        # `last_cmd` advanced to 0.517 m while the callback never fired. Same
        # node, same process; loopback delivery did not happen under a tight
        # spin_once loop.
        #
        # `last_cmd` is assigned from `p_out` on the line after the publish,
        # so it IS the published value rather than a re-derivation of it, and
        # what is being measured here is the filter and not the transport.
        # Chasing the loopback would have measured DDS.

    def spin(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def link_up(self):
        """Tell the mapper the headset link is alive, through its own callback.

        `tracking_ok` defaults FALSE and the bridge is what sets it, so a rig
        that does not publish it measures the FREEZE path -- correctly, and
        forever: the mapper publishes no command at all while it believes the
        link is down, which reads here as zero samples rather than as an
        error. Driving the real callback rather than assigning the attribute
        keeps the freeze logic itself in the measurement.
        """
        self.node._on_tracking(Bool(data=True))
        for h in ('left', 'right'):
            self.node.hand_tracked[h] = True

    def feed(self, p, grip=1.0):
        m = PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        self.node._on_pose('left', m)
        j = Joy()
        j.axes = [0.0, 0.0]
        j.buttons = [0, 0]
        # The clutch reads `grip` off axes or buttons depending on the client;
        # drive the node's own handler the way the bridge does.
        j.axes = [0.0, float(grip)]
        self.node._on_joy('left', j)

    def run(self, path, settle=1.2):
        """Hold still to latch, then walk `path`. Returns commanded points."""
        dt = 1.0 / RATE
        p0 = path[0]
        self.link_up()
        t_end = time.monotonic() + settle
        while time.monotonic() < t_end:
            self.link_up()
            self.feed(p0, grip=0.0)
            self.spin(dt)
        # engage on a quiet hand
        for _ in range(12):
            self.link_up()
            self.feed(p0, grip=1.0)
            self.spin(dt)
        self.got.clear()
        for p in path:
            # re-assert liveness: the mapper's own watchdog will drop it
            # otherwise, and a mid-run freeze would silently truncate the run
            self.link_up()
            self.feed(p, grip=1.0)
            self.spin(dt)
            c = self.node.last_cmd['left']
            if c is not None:
                self.got.append((time.monotonic(), np.asarray(c).copy()))
        return np.array([g[1] for g in self.got])

    def close(self):
        self.node.destroy_node()


def still_path(n=700, sigma=0.0015, seed=3):
    rng = np.random.default_rng(seed)
    return np.tile(np.array([0.0, 0.0, 0.0]), (n, 1)) + \
        rng.normal(0, sigma, (n, 3))


def reach_path(n=700, v=0.40, sigma=0.0015, seed=3):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / RATE
    return np.stack([v * t, np.zeros(n), np.zeros(n)], 1) + \
        rng.normal(0, sigma, (n, 3))


def score(law, extra=None):
    p = {'smoothing': law, 'align_yaw_deg': 0.0, 'rate_hz': RATE}
    p.update(extra or {})

    r = Rig(p)
    out = r.run(still_path())
    r.close()
    still = (float(np.sqrt(((out - out.mean(0)) ** 2).sum(1).mean())) * 1000
             if len(out) > 20 else float('nan'))

    r = Rig(p)
    path = reach_path()
    out = r.run(path)
    r.close()
    if len(out) > 200:
        # the command trails the hand along +x; compare the last quarter
        k = len(out) // 4
        want = ANCHOR_P[0] + path[-len(out):, 0]
        lag = float(np.mean(want[-k:] - out[-k:, 0])) * 1000
    else:
        lag = float('nan')
    return still, lag, len(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true",
                    help="also assert the ordering the change claims")
    a = ap.parse_args()

    rclpy.init()
    print("VR smoothing, measured through the real vr_pose_mapper")
    print("  still: a stationary hand + 1.5 mm-rms tremor -> command RMS")
    print("  lag:   a 0.40 m/s reach -> how far the command trails the hand")
    print()
    print("%-24s %10s %10s %8s" % ("law", "still_mm", "lag_mm", "cmds"))
    res = {}
    for label, law, extra in (
            ("none (raw)", "none", None),
            ("ema(0.6)  [was]", "ema", {"ema_alpha": 0.6}),
            ("one_euro  [now]", "one_euro", None)):
        s, l, n = score(law, extra)
        res[law] = (s, l)
        print("%-24s %10.2f %10.2f %8d" % (label, s, l, n))
    rclpy.shutdown()

    print()
    ok = True
    if a.self_test:
        def chk(name, cond, detail=""):
            nonlocal ok
            print("  %-52s %s   %s"
                  % (name, "PASS" if cond else "FAIL", detail))
            ok = ok and cond
        e_s, e_l = res["ema"]
        o_s, o_l = res["one_euro"]
        n_s, _ = res["none"]
        chk("the mapper actually applies the filter",
            o_s < n_s * 0.8, "%.2f vs %.2f mm raw" % (o_s, n_s))
        chk("one_euro is steadier than the ema it replaced",
            o_s < e_s, "%.2f vs %.2f mm" % (o_s, e_s))
        chk("one_euro does not pay for it in lag",
            o_l <= e_l + 0.5, "%.2f vs %.2f mm" % (o_l, e_l))
        print()
        print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
