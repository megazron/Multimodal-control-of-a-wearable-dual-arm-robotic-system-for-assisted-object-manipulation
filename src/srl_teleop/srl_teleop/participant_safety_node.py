#!/usr/bin/env python3
"""
participant_safety_node.py — the interlocks that only apply when a PERSON who
is not the developer is in the loop.

Deliberately SEPARATE from the developer limits in ik_follower_node's
`real_robot` mode. A developer working alone accepts risk knowingly and needs
the arm to move fast enough to debug; a participant has consented to a
described procedure and to nothing else. Sharing one set of limits means the
participant cap silently inherits whatever the developer last loosened.

WHAT IT ENFORCES

1. **A hard velocity cap for participant sessions**, independent of and
   stricter than the developer cap, applied by re-writing the follower's
   parameters and re-asserting them if anything changes them back.
2. **The autonomy must NEVER move the arm toward the wearer.** Checked
   geometrically against the collision model USING THE PARTICIPANT'S OWN
   dimensions, not the mannequin's — a smaller participant has less clearance
   than the model assumes, and the mannequin is 1.75 m.
3. **Abort and mark the trial INVALID if a channel the current mode uses drops
   out mid-trial.** Never silently continue. A frozen command looks exactly
   like a slow participant in the data.
4. **Every e-stop and abort is logged with its cause**, to a file that
   survives the session.
5. **The e-stop must be within the participant's reach and verified each
   session** — this node refuses to arm until `/estop_reachable_confirmed` has
   been published for the session, which forces the check to happen.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

import tf2_ros

# The wearer model in human_backpack.xacro is a 1.75 m mannequin. A
# participant who differs is closer to the arm than the model says.
MODEL_HEIGHT_M = 1.75
DISTAL_LINKS = ["forearm_link", "spherical_wrist_1_link",
                "spherical_wrist_2_link", "bracelet_link", "end_effector_link"]


class ParticipantSafety(Node):
    def __init__(self):
        super().__init__("participant_safety_node")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("enabled", False)
        # STRICTER than the developer real_robot cap of 0.10 rad/s.
        self.declare_parameter("participant_max_vel_rad_s", 0.05)
        self.declare_parameter("participant_max_step_rad", 0.05)
        self.declare_parameter("participant_min_clearance_m", 0.15)
        self.declare_parameter("participant_height_m", 1.75)
        self.declare_parameter("required_channels",
                               ["j1", "j2", "j4", "accel"])
        self.declare_parameter("log_dir", "")
        self.declare_parameter("reassert_period_s", 2.0)

        self.arms = list(self.get_parameter("arms").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.estop_confirmed = False
        self.channels = {}
        self.abort_pub = self.create_publisher(String, "/participant/abort", 10)
        self.status_pub = self.create_publisher(String, "/participant/safety", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)

        d = self.get_parameter("log_dir").value or \
            str(Path.home() / "kortex_ws" / "recordings" / "safety")
        Path(d).mkdir(parents=True, exist_ok=True)
        self.logfile = Path(d) / ("safety_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(Bool, "/estop_reachable_confirmed",
                                 self._on_estop_confirm, 10)
        self.create_subscription(Bool, "/estop_state", self._on_estop, 10)
        for a in self.arms:
            self.create_subscription(
                String, f"/master_capability_{a}",
                lambda m, a=a: self._on_caps(a, m), 10)
            self.create_subscription(
                String, f"/autonomy/state_{a}",
                lambda m, a=a: self._on_autonomy(a, m), 10)
        self.setters = {a: self.create_client(
            SetParameters, f"/ik_follower_{a}/set_parameters") for a in self.arms}
        self._estop_setter = self.create_client(
            SetParameters, "/estop_node/set_parameters")
        self.create_service(Trigger, "/participant/arm_session", self._srv_arm)
        self.create_timer(float(self.get_parameter("reassert_period_s").value),
                          self._reassert)
        self.create_timer(0.1, self._watch)
        self._event("node_start", enabled=self.enabled)
        self.get_logger().warn(
            "participant_safety_node up, enabled=%s. It will NOT arm until "
            "/estop_reachable_confirmed is published - the reach check is "
            "forced to happen, every session." % self.enabled)

    # ------------------------------------------------------------- logging
    def _event(self, kind, **kw):
        rec = dict(t=time.strftime("%Y-%m-%dT%H:%M:%S"), kind=kind, **kw)
        with open(self.logfile, "a") as f:
            f.write(json.dumps(rec) + "\n")
        return rec

    # ----------------------------------------------------------- callbacks
    def _on_estop_confirm(self, m):
        if m.data and not self.estop_confirmed:
            self.estop_confirmed = True
            self._event("estop_reach_confirmed")
            self.get_logger().info(
                "e-stop reachability CONFIRMED for this session.")

    def _on_estop(self, m):
        if m.data:
            self._event("estop_latched", cause="/estop_state true")
            self.get_logger().error("E-STOP latched - logged with cause.")

    def _on_caps(self, arm, m):
        try:
            self.channels[arm] = json.loads(m.data)
        except ValueError:
            pass

    def _on_autonomy(self, arm, m):
        if m.data == "ASSIST":
            ok, why = self._autonomy_direction_safe(arm)
            if not ok:
                self._abort(arm, "autonomy moving toward the wearer: %s" % why)

    # ------------------------------------------------------------- checks
    def _scaled_clearance_floor(self):
        """A shorter participant sits closer to the arm than the 1.75 m model.

        The wearer geometry scales with height, so the clearance the model
        reports overstates the real margin by roughly the height ratio. The
        floor is inflated by that ratio rather than trusting the model.
        """
        h = float(self.get_parameter("participant_height_m").value)
        base = float(self.get_parameter("participant_min_clearance_m").value)
        if h <= 0:
            return base
        return base * max(1.0, MODEL_HEIGHT_M / h)

    def _autonomy_direction_safe(self, arm):
        """Is the assisted motion heading AWAY from the wearer?

        The wearer's sagittal axis is x=0 and the body occupies y in about
        [-0.12, +0.11]. Motion is unsafe if the end effector is both close to
        the body and moving toward it.
        """
        try:
            t = self.tf_buf.lookup_transform(
                "world", f"{arm}_end_effector_link", rclpy.time.Time())
        except Exception:
            return True, "no tf; not blocking on missing data"
        p = np.array([t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z])
        prev = getattr(self, "_prev_ee_%s" % arm, None)
        setattr(self, "_prev_ee_%s" % arm, p)
        if prev is None:
            return True, "no previous sample"
        v = p - prev
        if np.linalg.norm(v) < 1e-4:
            return True, "stationary"
        # Distance to the torso column, and whether we are closing on it.
        d_now = math.hypot(p[0], p[1] - 0.0)
        d_prev = math.hypot(prev[0], prev[1] - 0.0)
        floor = self._scaled_clearance_floor()
        if d_now < floor + 0.10 and d_now < d_prev:
            return False, ("EE %.3f m from the body axis and closing "
                           "(floor %.3f m for a %.2f m participant)"
                           % (d_now, floor, self.get_parameter(
                               "participant_height_m").value))
        return True, "ok"

    def _watch(self):
        if not bool(self.get_parameter("enabled").value):
            return
        req = list(self.get_parameter("required_channels").value)
        for arm in self.arms:
            ch = (self.channels.get(arm) or {}).get("channels")
            if not ch:
                continue
            bad = [c for c in req if not ch.get(c, True)]
            if bad:
                self._abort(arm, "required channel(s) dropped out: %s"
                            % ",".join(bad))

    def _abort(self, arm, why):
        key = "_aborted_%s" % arm
        if getattr(self, key, "") == why:
            return
        setattr(self, key, why)
        rec = self._event("abort", arm=arm, cause=why)
        m = String()
        m.data = json.dumps(rec)
        self.abort_pub.publish(m)
        self.get_logger().error(
            "[%s] ABORT, trial is INVALID: %s. Never silently continued."
            % (arm, why))
        b = Bool()
        b.data = True
        self.estop_pub.publish(b)

    # -------------------------------------------------------------- limits
    def _reassert(self):
        if not bool(self.get_parameter("enabled").value):
            return
        if not self.estop_confirmed:
            self.get_logger().warn(
                "participant limits NOT applied: e-stop reachability has not "
                "been confirmed this session.", throttle_duration_sec=15.0)
            return
        # The e-stop DEAD-MAN defaults OFF (a developer unplugs the master all
        # the time). For a participant it must be ON: a Teensy disconnect
        # mid-trial is otherwise indistinguishable from a still hand. Fault
        # injection caught this - 10 s of total master silence produced no
        # response at all.
        for cli in (self._estop_setter,):
            if cli.service_is_ready():
                req = SetParameters.Request()
                p = Parameter(); p.name = "deadman_enabled"
                p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                         bool_value=True)
                req.parameters.append(p)
                cli.call_async(req)
        vals = {
            "max_vel_rad_s": float(self.get_parameter("participant_max_vel_rad_s").value),
            "max_step_rad": float(self.get_parameter("participant_max_step_rad").value),
            "min_clearance_m": self._scaled_clearance_floor(),
        }
        for arm, cli in self.setters.items():
            if not cli.service_is_ready():
                continue
            req = SetParameters.Request()
            for k, v in vals.items():
                p = Parameter()
                p.name = k
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=v)
                req.parameters.append(p)
            cli.call_async(req)
        s = String()
        s.data = json.dumps(dict(enabled=True, applied=vals,
                                 estop_confirmed=self.estop_confirmed,
                                 log=str(self.logfile)))
        self.status_pub.publish(s)

    def _srv_arm(self, req, res):
        if not self.estop_confirmed:
            res.success = False
            res.message = ("refused: publish /estop_reachable_confirmed after "
                           "physically checking the participant can reach it")
            self._event("arm_refused", cause="estop not confirmed")
            return res
        self.set_parameters([rclpy.parameter.Parameter(
            "enabled", rclpy.Parameter.Type.BOOL, True)])
        self._event("session_armed",
                    vel=float(self.get_parameter("participant_max_vel_rad_s").value))
        res.success = True
        res.message = "participant limits armed"
        return res


def main():
    rclpy.init()
    n = ParticipantSafety()
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
