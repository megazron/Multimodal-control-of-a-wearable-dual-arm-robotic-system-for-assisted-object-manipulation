#!/usr/bin/env python3
"""
object_pose_tracker.py — fuse the two wrist cameras into one stable list of
world-frame objects, and DROP STALE ONES.

Why this is a separate node from the detector: the thing autonomy needs is not
"what did camera N see in the last frame" but "what objects exist, where, and
how much should I trust that right now". Those are different questions, and
conflating them is how an object that left the field of view thirty seconds
ago ends up being offered as a grasp target.

Rules, all of them boring on purpose:
  * an object is keyed by its tag id, so fusion is exact and there is no
    data-association step to get wrong;
  * position is a first-order low pass, which suppresses per-frame jitter
    without inventing motion;
  * an object not seen for `stale_after_s` is REMOVED, not held. A held pose
    is indistinguishable from a live one downstream, and that is precisely the
    failure that a frozen /joint_states and a dead pot both had;
  * confidence decays with age, so a briefly-occluded object degrades rather
    than vanishing and reappearing.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class ObjectPoseTracker(Node):
    def __init__(self):
        super().__init__("object_pose_tracker")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("stale_after_s", 1.5)
        self.declare_parameter("position_tau_s", 0.15)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("min_confidence", 0.15)

        self.stale = float(self.get_parameter("stale_after_s").value)
        self.tau = float(self.get_parameter("position_tau_s").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.objs = {}          # id -> dict(p, q, conf, last_seen, seen_count)

        for a in self.get_parameter("arms").value:
            self.create_subscription(
                Detection3DArray, f"/perception/detections/{a}",
                lambda m, a=a: self._on_det(a, m), 10)
        self.pub = self.create_publisher(Detection3DArray, "/perception/objects", 10)
        self.info = self.create_publisher(String, "/perception/objects_info", 10)
        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self._last_msg = None

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_det(self, arm, msg):
        self._last_msg = msg
        now = self._now()
        for d in msg.detections:
            if not d.results:
                continue
            oid = d.id or d.results[0].hypothesis.class_id
            conf = float(d.results[0].hypothesis.score)
            if conf < self.min_conf:
                continue
            p = d.results[0].pose.pose
            pos = np.array([p.position.x, p.position.y, p.position.z])
            quat = np.array([p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w])
            o = self.objs.get(oid)
            if o is None:
                self.objs[oid] = dict(p=pos, q=quat, conf=conf, last=now,
                                      n=1, frame=msg.header.frame_id, src=arm)
            else:
                dt = max(1e-3, now - o["last"])
                k = dt / (self.tau + dt)
                o["p"] = (1 - k) * o["p"] + k * pos
                o["q"] = quat
                o["conf"] = max(o["conf"] * 0.9, conf)
                o["last"] = now
                o["n"] += 1
                o["src"] = arm

    def _tick(self):
        now = self._now()
        dropped = [k for k, o in self.objs.items() if now - o["last"] > self.stale]
        for k in dropped:
            self.get_logger().info(
                "object %s dropped: not seen for %.1f s. Removed rather than "
                "held - a held pose is indistinguishable from a live one." %
                (k, self.stale))
            del self.objs[k]

        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        info = []
        for oid, o in sorted(self.objs.items()):
            from vision_msgs.msg import (BoundingBox3D, Detection3D,
                                         ObjectHypothesisWithPose)
            age = now - o["last"]
            conf = float(o["conf"] * max(0.0, 1.0 - age / self.stale))
            d = Detection3D()
            d.header = out.header
            d.id = oid
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = oid
            h.hypothesis.score = conf
            h.pose.pose.position.x = float(o["p"][0])
            h.pose.pose.position.y = float(o["p"][1])
            h.pose.pose.position.z = float(o["p"][2])
            (h.pose.pose.orientation.x, h.pose.pose.orientation.y,
             h.pose.pose.orientation.z, h.pose.pose.orientation.w) = \
                (float(v) for v in o["q"])
            d.results.append(h)
            d.bbox = BoundingBox3D()
            d.bbox.center = h.pose.pose
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = 0.05
            out.detections.append(d)
            info.append(dict(id=oid, conf=round(conf, 3), age_s=round(age, 3),
                             seen=o["n"], source=o["src"],
                             pos=[round(float(v), 4) for v in o["p"]]))
        self.pub.publish(out)
        m = String()
        m.data = json.dumps({"n": len(info), "objects": info})
        self.info.publish(m)


def main():
    rclpy.init()
    n = ObjectPoseTracker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
