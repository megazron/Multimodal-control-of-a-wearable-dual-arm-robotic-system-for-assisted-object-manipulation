#!/usr/bin/env python3
"""
vr_feedback_node.py — what the operator sees and feels inside the headset.

THIS IS THE VR PATH'S REAL ADVANTAGE and it should be exploited rather than
treated as decoration. The mannequin operator looks at a real robot and a
terminal dashboard; the VR operator sees only what is rendered. There is no
force feedback on a Quest controller, so the visual channel carries everything
and haptics carry the state CHANGES.

Rendered (published as one JSON blob the client draws):
  * the robot's ACTUAL joint state and end-effector pose, from TF, not the
    command - so the operator sees what the arm did, not what was asked;
  * detected objects, each with its inferred intent probability, so the
    operator can SEE what the autonomy believes before it acts;
  * the autonomy state, DIRECT / ASSIST / GRASPED;
  * clutch state and current scale;
  * workspace boundary warnings, which matter more here than anywhere else
    because the operator cannot see the arm approaching its limit;
  * clearance to the wearer.

HAPTICS on state CHANGE only, never continuously: a pulse when ASSIST engages,
a double pulse when it is cancelled, a longer one on freeze. Continuous haptics
become background noise within a minute and then convey nothing.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

import tf2_ros

PULSE = {'assist_on': dict(hand='both', ms=80, amp=0.6),
         'assist_off': dict(hand='both', ms=40, amp=0.4, repeat=2),
         'grasped': dict(hand='both', ms=150, amp=0.9),
         'freeze': dict(hand='both', ms=300, amp=1.0),
         'boundary': dict(hand='both', ms=60, amp=0.5)}


class VrFeedback(Node):
    def __init__(self):
        super().__init__('vr_feedback_node')
        self.declare_parameter('arms', ['left', 'right'])
        self.declare_parameter('rate_hz', 20.0)
        self.arms = list(self.get_parameter('arms').value)
        self.objects = []
        self.intent = {}
        self.autonomy = {a: 'DIRECT' for a in self.arms}
        self.prev_autonomy = dict(self.autonomy)
        self.frozen = False
        self.prev_frozen = False
        # The overlay's single most important field. It was being dropped here:
        # /vr/safety carries the REASON and only the bool was forwarded, so an
        # in-headset panel could say FROZEN and never say why -- and 'tracking
        # lost', 'observer e-stop withdrawn' and 'network dropout' need three
        # different reactions from the operator.
        self.freeze_reason = 'startup'
        self.mapper = {}
        self.boundary = []

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(Detection3DArray, '/perception/objects',
                                 self._on_objects, 10)
        self.create_subscription(String, '/vr/boundary_warning',
                                 lambda m: setattr(self, 'boundary', json.loads(m.data)), 10)
        self.create_subscription(String, '/vr/safety', self._on_safety, 10)
        for a in self.arms:
            self.create_subscription(String, f'/autonomy/intent_debug_{a}',
                                     lambda m, a=a: self.intent.__setitem__(a, json.loads(m.data)), 10)
            self.create_subscription(String, f'/autonomy/state_{a}',
                                     lambda m, a=a: self.autonomy.__setitem__(a, m.data), 10)
        for h in ('left', 'right'):
            self.create_subscription(String, f'/vr/mapper_{h}',
                                     lambda m, h=h: self.mapper.__setitem__(h, json.loads(m.data)), 10)
        self.render_pub = self.create_publisher(String, '/vr/robot_state_json', 10)
        self.haptic_pub = self.create_publisher(String, '/vr/haptics', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)

    def _on_objects(self, m):
        out = []
        for d in m.detections:
            if not d.results:
                continue
            p = d.results[0].pose.pose.position
            out.append(dict(id=d.id or d.results[0].hypothesis.class_id,
                            p=[round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                            conf=round(float(d.results[0].hypothesis.score), 3)))
        self.objects = out

    def _on_safety(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.frozen = bool(d.get('frozen'))
        self.freeze_reason = str(d.get('reason', '') or '')

    def _pulse(self, kind):
        s = String()
        s.data = json.dumps(dict(kind=kind, **PULSE[kind]))
        self.haptic_pub.publish(s)

    def _tick(self):
        for a in self.arms:
            if self.autonomy[a] != self.prev_autonomy[a]:
                if self.autonomy[a] == 'ASSIST':
                    self._pulse('assist_on')
                elif self.autonomy[a] == 'GRASPED':
                    self._pulse('grasped')
                elif self.prev_autonomy[a] == 'ASSIST':
                    self._pulse('assist_off')
                self.prev_autonomy[a] = self.autonomy[a]
        if self.frozen and not self.prev_frozen:
            self._pulse('freeze')
        self.prev_frozen = self.frozen
        if self.boundary:
            self._pulse('boundary')

        arms = {}
        for a in self.arms:
            ee = None
            try:
                t = self.tf_buf.lookup_transform('world', f'{a}_end_effector_link',
                                                 rclpy.time.Time())
                v, r = t.transform.translation, t.transform.rotation
                ee = dict(p=[round(v.x, 4), round(v.y, 4), round(v.z, 4)],
                          q=[round(r.x, 4), round(r.y, 4), round(r.z, 4), round(r.w, 4)])
            except Exception:
                pass
            it = self.intent.get(a, {})
            arms[a] = dict(ee=ee, autonomy=self.autonomy[a],
                           intent_top=it.get('top'), intent_p=it.get('top_p'),
                           intent_ambiguous=it.get('ambiguous'),
                           intent_dist=dict(zip(it.get('ids', []), it.get('p', []))))
        s = String()
        s.data = json.dumps(dict(arms=arms, objects=self.objects,
                                 frozen=self.frozen,
                                 freeze_reason=self.freeze_reason,
                                 boundary=self.boundary,
                                 mapper=self.mapper))
        self.render_pub.publish(s)


def main():
    rclpy.init()
    n = VrFeedback()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
