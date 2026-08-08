#!/usr/bin/env python3
"""
intent_inference.py — P(goal) over the detected objects.

PRIMARY CUE IS THE IMU POINTING DIRECTION, not commanded end-effector
velocity. Pointing is measured directly and its elevation is drift-free;
commanded velocity is a derivative of the potentiometer chain and inherits
every pot fault the master has. On this rig right j4 drops out on 12.9% of
frames in bursts, and a rejected frame FREEZES the command — so velocity reads
zero exactly when the operator is moving hardest. Inferring intent from that
is inferring from the fault. Velocity is kept as a SECONDARY cue with a small
weight, because when it is healthy it disambiguates depth, which pointing
cannot.

The model is a Javdani-style hindsight formulation reduced to its usable core,
not a full POMDP:

    P(g | obs) ∝ P(g) · exp( -(1/beta) * cost_to_go_penalty(g, obs) )

with the penalty built from the angle between the observed direction and the
direction to the goal. A simple recursive Bayesian update over frames, with a
FORGETTING FACTOR, which is what lets the operator change their mind.

THE THREE HARD CASES, all handled explicitly and all tested:

  1. NO OBJECTS. Publish an empty distribution and a state of `no_objects`.
     Never a uniform distribution over nothing, and never a stale one.
  2. MULTIPLE EQUALLY LIKELY. Report the tie honestly: the top probability
     stays near 1/N and `ambiguous` is true. Downstream MUST NOT enter ASSIST
     on a tie, because assisting toward the wrong one of two adjacent objects
     is worse than not assisting.
  3. THE OPERATOR CHANGES THEIR MIND MID-REACH. This is the one that matters.
     Latching onto a goal and not releasing is worse than having no inference
     at all, so the update uses a forgetting factor (a hard floor on every
     probability, equivalently a leak toward uniform) which bounds how
     confident the estimate can become and therefore bounds how long it takes
     to abandon a goal. `switch_latency_s` is measured, not assumed.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

import tf2_ros


class IntentEstimator:
    """Pure logic, no ROS — so the hard cases can be unit-tested."""

    def __init__(self, beta=0.35, forget=0.02, vel_weight=0.25,
                 min_speed=0.02):
        # beta: softness of the pointing likelihood. Smaller = sharper.
        self.beta = float(beta)
        # forget: probability floor per object per update. THIS is what makes
        # a change of mind possible; with forget=0 the log-odds can run away
        # and the estimate becomes unrecoverable.
        self.forget = float(forget)
        self.vel_weight = float(vel_weight)
        self.min_speed = float(min_speed)
        self.ids = []
        self.p = np.zeros(0)

    def _resync(self, ids):
        """Objects appear and disappear; carry over what we knew."""
        if ids == self.ids:
            return
        old = dict(zip(self.ids, self.p))
        self.ids = list(ids)
        if not ids:
            self.p = np.zeros(0)
            return
        # A NEW object starts at the uniform prior, not at zero: an object
        # that has only just been detected must be able to win.
        u = 1.0 / len(ids)
        self.p = np.array([old.get(i, u) for i in ids], float)
        self.p /= self.p.sum()

    def update(self, ids, ee_pos, obj_pos, pointing=None, ee_vel=None):
        """One frame. Returns the posterior over `ids`."""
        self._resync(list(ids))
        if not self.ids:
            return self.p
        n = len(self.ids)
        to_goal = np.asarray(obj_pos, float) - np.asarray(ee_pos, float)
        d = np.linalg.norm(to_goal, axis=1)
        safe = np.maximum(d, 1e-6)
        unit = to_goal / safe[:, None]

        loglik = np.zeros(n)
        used_pointing = False
        if pointing is not None:
            u = np.asarray(pointing, float)
            nu = np.linalg.norm(u)
            if nu > 1e-6:
                used_pointing = True
                cosang = np.clip(unit @ (u / nu), -1.0, 1.0)
                # Penalty is the angular miss. exp(-(1-cos)/beta) is a von
                # Mises-like kernel: cheap, and it degrades gracefully instead
                # of going to zero for a goal behind the operator.
                loglik += -(1.0 - cosang) / self.beta

        if ee_vel is not None:
            v = np.asarray(ee_vel, float)
            sp = float(np.linalg.norm(v))
            if sp > self.min_speed:
                cosv = np.clip(unit @ (v / sp), -1.0, 1.0)
                loglik += -self.vel_weight * (1.0 - cosv) / self.beta

        if not used_pointing and ee_vel is None:
            return self.p                      # no evidence: do not touch it

        post = self.p * np.exp(loglik - loglik.max())
        s = post.sum()
        if s <= 0 or not np.isfinite(s):
            post = np.full(n, 1.0 / n)
        else:
            post /= s
        # FORGETTING: leak toward uniform. Bounds confidence, and therefore
        # bounds how long a wrong goal can persist once the evidence flips.
        post = (1.0 - self.forget * n) * post + self.forget
        self.p = post / post.sum()
        return self.p

    def summary(self):
        if not self.ids:
            return dict(state="no_objects", ids=[], p=[], top=None,
                        top_p=0.0, ambiguous=False, margin=0.0)
        order = np.argsort(-self.p)
        top = int(order[0])
        second = float(self.p[order[1]]) if len(order) > 1 else 0.0
        margin = float(self.p[top] - second)
        return dict(state="ok", ids=list(self.ids),
                    p=[float(x) for x in self.p],
                    top=self.ids[top], top_p=float(self.p[top]),
                    # A tie is reported as a tie. Downstream must not assist.
                    ambiguous=bool(margin < 0.15),
                    margin=margin)


class IntentInferenceNode(Node):
    def __init__(self):
        super().__init__("intent_inference")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("beta", 0.35)
        self.declare_parameter("forget", 0.02)
        self.declare_parameter("vel_weight", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.arms = list(self.get_parameter("arms").value)
        self.est = {a: IntentEstimator(
            float(self.get_parameter("beta").value),
            float(self.get_parameter("forget").value),
            float(self.get_parameter("vel_weight").value)) for a in self.arms}
        self.pointing = {a: None for a in self.arms}
        self.last_ee = {a: None for a in self.arms}
        self.ee_vel = {a: None for a in self.arms}
        self.t_ee = {a: None for a in self.arms}
        self.objects = []

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(Detection3DArray, "/perception/objects",
                                 self._on_objects, 10)
        self.p_pub, self.id_pub, self.dbg = {}, {}, {}
        for a in self.arms:
            self.create_subscription(
                Vector3Stamped, f"/master_pointing_{a}",
                lambda m, a=a: self.pointing.__setitem__(
                    a, np.array([m.vector.x, m.vector.y, m.vector.z])), 10)
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}",
                lambda m, a=a: self._on_cmd(a, m), 10)
            self.p_pub[a] = self.create_publisher(
                Float64MultiArray, f"/autonomy/intent_{a}", 10)
            self.id_pub[a] = self.create_publisher(
                String, f"/autonomy/intent_ids_{a}", 10)
            self.dbg[a] = self.create_publisher(
                String, f"/autonomy/intent_debug_{a}", 10)
        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / hz, self._tick)

    def _on_objects(self, msg):
        objs = []
        for d in msg.detections:
            if not d.results:
                continue
            p = d.results[0].pose.pose.position
            objs.append((d.id or d.results[0].hypothesis.class_id,
                         np.array([p.x, p.y, p.z])))
        self.objects = objs

    def _on_cmd(self, arm, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        if self.last_ee[arm] is not None and self.t_ee[arm] is not None:
            dt = now - self.t_ee[arm]
            if 1e-3 < dt < 0.5:
                self.ee_vel[arm] = (p - self.last_ee[arm]) / dt
        self.last_ee[arm] = p
        self.t_ee[arm] = now

    def _ee(self, arm):
        try:
            t = self.tf_buf.lookup_transform(
                "world", f"{arm}_end_effector_link", rclpy.time.Time())
            v = t.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:
            return self.last_ee[arm]

    def _tick(self):
        ids = [o[0] for o in self.objects]
        pos = np.array([o[1] for o in self.objects]) if self.objects else np.zeros((0, 3))
        for a in self.arms:
            ee = self._ee(a)
            if ee is None:
                continue
            p = self.est[a].update(ids, ee, pos, self.pointing[a], self.ee_vel[a])
            s = self.est[a].summary()
            m = Float64MultiArray()
            m.data = [float(x) for x in p]
            self.p_pub[a].publish(m)
            im = String()
            im.data = json.dumps(s["ids"])
            self.id_pub[a].publish(im)
            dm = String()
            s["cue"] = ("pointing" if self.pointing[a] is not None else "none")
            dm.data = json.dumps(s)
            self.dbg[a].publish(dm)


def main():
    rclpy.init()
    n = IntentInferenceNode()
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
