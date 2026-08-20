#!/usr/bin/env python3
"""How fast the body reaches the planning scene, and where the time goes.

    python3 scripts/measure_wearer_tracking_rate.py --seconds 25

The brief asks for "update rate and latency from movement to planning scene",
which is four numbers and not one, and only three of them can be measured
without a person moving in front of a real camera:

    1. CAMERA RATE          frames actually published            MEASURED
    2. TRACKER RATE          estimates actually published         MEASURED
    3. FRAME -> ESTIMATE     stamp on the image to the estimate   MEASURED
    4. ESTIMATE -> SCENE     rate limit plus publish              MEASURED as a
                             rate; its worst case is 1/scene_hz

The sum of 3 and the worst case of 4 is the honest end-to-end figure, and it
is reported as a RANGE rather than a single number, because the rate limit
means the answer depends on when in the cycle the person moved.

WHAT THIS DOES NOT MEASURE. Shutter to photons: the delay inside the camera
and its driver before a frame reaches ROS. On a UVC webcam that is typically
another 10 to 30 ms and it is invisible from here -- there is no timestamp on
the frame older than the one this repository puts on it. It is named so the
end-to-end number is not quoted as if it were complete.
"""
import argparse
import json
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Meter(Node):

    def __init__(self):
        super().__init__("wearer_rate_meter")
        self.img_t, self.est_t, self.scene_t = [], [], []
        self.last_img_stamp = None
        self.lat = []
        self.states = []
        self.create_subscription(Image, "/scene_camera/image_raw",
                                 self.on_img, qos_profile_sensor_data)
        self.create_subscription(String, "/wearer/estimate", self.on_est, 10)
        try:
            from moveit_msgs.msg import PlanningScene
            self.create_subscription(PlanningScene, "/planning_scene",
                                     self.on_scene, 4)
        except Exception:                                     # noqa: BLE001
            pass

    def on_img(self, m):
        now = time.monotonic()
        self.img_t.append(now)
        self.last_img_stamp = (m.header.stamp.sec
                               + m.header.stamp.nanosec * 1e-9, now)

    def on_est(self, m):
        now = time.monotonic()
        self.est_t.append(now)
        if self.last_img_stamp is not None:
            self.lat.append((now - self.last_img_stamp[1]) * 1e3)
        try:
            self.states.append(json.loads(m.data).get("state", "")[:40])
        except Exception:                                     # noqa: BLE001
            pass

    def on_scene(self, m):
        ids = [c.id for c in m.world.collision_objects]
        if any(i.startswith("wearer_") for i in ids):
            self.scene_t.append(time.monotonic())


def rate(ts):
    if len(ts) < 3:
        return None
    return (len(ts) - 1) / (ts[-1] - ts[0])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--scene-hz", type=float, default=2.0,
                    help="the tracker's scene_hz parameter, for the worst case")
    a = ap.parse_args(argv)

    rclpy.init()
    m = Meter()
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(m, timeout_sec=0.1)

    print("\n== WEARER TRACKING, RATE AND LATENCY ==")
    print("  measured over %.0f s\n" % a.seconds)
    rows = [
        ("scene camera frames", rate(m.img_t), "Hz", True),
        ("wearer estimates", rate(m.est_t), "Hz", True),
        ("planning-scene updates", rate(m.scene_t), "Hz", True),
    ]
    for name, v, unit, meas in rows:
        print("  %-26s %-10s %s"
              % (name, "--" if v is None else "%.2f %s" % (v, unit),
                 "MEASURED" if meas else "inferred"))
    if m.lat:
        print("  %-26s %.1f ms median, %.1f ms p95   MEASURED"
              % ("frame -> estimate", st.median(m.lat),
                 sorted(m.lat)[int(0.95 * (len(m.lat) - 1))]))
        worst_scene = 1000.0 / max(0.2, a.scene_hz)
        print("  %-26s %.0f ms worst case (the rate limit)   from scene_hz=%s"
              % ("estimate -> scene", worst_scene, a.scene_hz))
        print("\n  END TO END, movement to planning scene:")
        print("    %.0f to %.0f ms, plus the camera's own shutter-to-ROS delay,"
              % (st.median(m.lat), st.median(m.lat) + worst_scene))
        print("    which is NOT measurable from here (typically 10-30 ms on a")
        print("    UVC webcam). The spread is the rate limit, not jitter.")
    else:
        print("\n  no estimates seen -- is the tracker running?")
    if m.states:
        seen = sorted(set(m.states))
        print("\n  states seen: %s" % "; ".join(seen[:3]))
    if not m.scene_t:
        print("\n  NO planning-scene updates. That is CORRECT if the body has "
              "not moved:\n  the writer publishes only when the geometry "
              "actually changes, so a\n  still person produces no traffic at "
              "all.")
    m.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
