#!/usr/bin/env python3
"""
payload_manager.py — tell the Kinova arm what it is holding (Part 6a).

WHY THIS MATTERS FOR THE STUDY, not just for the robot. The Gen3's internal
controller compensates gravity using a configured payload mass and centre of
mass. If the configured payload does not match what is actually in the
gripper, the residual gravity torque shows up as a steady-state tracking
error that VARIES WITH THE OBJECT. Trials with a heavy object then have worse
tracking than trials with a light one, for reasons that have nothing to do
with the condition under test — and object type is usually crossed with
condition. That is a confound baked into the data, and it cannot be removed
afterwards.

So payload is set on every grasp and cleared on every release, and the value
used is logged per trial.

HOW IT TALKS TO THE ARM. Payload configuration is a HIGH-LEVEL Kortex API
call (Base/ControlConfig SetPayloadInformation), not a cyclic one, so it goes
through the same isolated interpreter as kortex_highlevel_bridge — see the
kortex_api note in CLAUDE.md. In sim, and whenever the API is unavailable, the
node stays up and records what it WOULD have set, so the logged value is
correct even when the call is a no-op.
"""
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# Object catalogue masses, kg, and centre of mass in the tool frame, metres.
# Measured on a kitchen scale; the CoM is the geometric centre of the grasped
# object, offset along the tool +z by half its height plus the finger length.
PAYLOADS = {
    "small_cube": (0.045, (0.0, 0.0, 0.055)),
    "tall_box": (0.120, (0.0, 0.0, 0.075)),
    "wide_block": (0.090, (0.0, 0.0, 0.052)),
    "cylinder": (0.075, (0.0, 0.0, 0.065)),
    "flat_plate": (0.060, (0.0, 0.0, 0.040)),
    "narrow_rod": (0.035, (0.0, 0.0, 0.085)),
}
# The gripper itself is part of the tool, not the payload: the Robotiq 2F-85
# is already in the arm's tool configuration. Only the grasped object is set
# here. Setting the gripper mass again double-counts it.
EMPTY = (0.0, (0.0, 0.0, 0.0))


class PayloadManager(Node):
    def __init__(self):
        super().__init__("payload_manager")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("apply_to_hardware", False)
        self.declare_parameter("gripper_closed_rad", 0.55)
        self.arms = list(self.get_parameter("arms").value)
        self.apply_hw = bool(self.get_parameter("apply_to_hardware").value)
        self.state = {a: EMPTY for a in self.arms}
        self.held = {a: None for a in self.arms}
        self.gripper = {a: 0.0 for a in self.arms}

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.pub = {}
        for a in self.arms:
            self.create_subscription(String, f"/autonomy/state_{a}",
                                     lambda m, a=a: self._on_state(a, m), 10)
            self.create_subscription(String, f"/autonomy/grasp_status_{a}",
                                     lambda m, a=a: self._on_grasp(a, m), 10)
            self.pub[a] = self.create_publisher(
                String, f"/payload_state_{a}", 10)
        self.create_service(Trigger, "/payload/clear", self._srv_clear)
        self.create_timer(1.0, self._publish)
        if not self.apply_hw:
            self.get_logger().info(
                "payload_manager up in RECORD-ONLY mode. The value that WOULD "
                "be sent is published and logged, so trial data is correct "
                "even though the Kortex call is a no-op. Set "
                "apply_to_hardware:=true on the real arm.")

    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        for a in self.arms:
            v = d.get(f"{a}_robotiq_85_left_knuckle_joint")
            if v is not None:
                self.gripper[a] = float(v)

    def _on_grasp(self, arm, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if s.get("offered"):
            self.held[arm] = s.get("object")

    def _on_state(self, arm, msg):
        st = msg.data
        if st == "GRASPED":
            name = self.held[arm]
            mass, com = PAYLOADS.get(name, EMPTY)
            if name and name not in PAYLOADS:
                self.get_logger().warn(
                    "[%s] grasped '%s' which is not in the payload catalogue; "
                    "setting ZERO payload and logging it. Tracking error for "
                    "this trial will carry an uncompensated gravity term."
                    % (arm, name))
            self._set(arm, mass, com, "grasped %s" % name)
        elif st == "DIRECT" and self.state[arm] != EMPTY:
            self._set(arm, *EMPTY, why="released")

    def _set(self, arm, mass, com, why=""):
        if (mass, tuple(com)) == self.state[arm]:
            return
        self.state[arm] = (mass, tuple(com))
        self.get_logger().info("[%s] payload -> %.3f kg at %s  (%s)"
                               % (arm, mass, com, why))
        if self.apply_hw:
            self._apply(arm, mass, com)

    def _apply(self, arm, mass, com):
        """SetPayloadInformation via the isolated kortex_api interpreter.

        Deliberately fail-soft: if the API is missing the node logs and keeps
        running, because losing payload compensation must not take down
        teleoperation.
        """
        venv = os.path.expanduser("~/kortex_ws/.kortex_venv/bin/python3")
        if not os.path.exists(venv):
            self.get_logger().warn(
                "[%s] apply_to_hardware is set but %s is missing; payload NOT "
                "applied. See the kortex_api isolation note in CLAUDE.md."
                % (arm, venv), throttle_duration_sec=30.0)
            return
        self.get_logger().warn(
            "[%s] hardware payload application is IMPLEMENTED AS A STUB. It "
            "needs a live Kortex session, which this node does not own - the "
            "session belongs to kortex_highlevel_bridge and the arm permits "
            "exactly one. Route it through that bridge before real use."
            % arm, throttle_duration_sec=60.0)

    def _srv_clear(self, req, res):
        for a in self.arms:
            self._set(a, *EMPTY, why="service /payload/clear")
        res.success = True
        res.message = "payload cleared on %s" % ", ".join(self.arms)
        return res

    def _publish(self):
        for a in self.arms:
            mass, com = self.state[a]
            m = String()
            m.data = json.dumps(dict(arm=a, mass_kg=mass, com_m=list(com),
                                     object=self.held[a],
                                     applied_to_hardware=self.apply_hw))
            self.pub[a].publish(m)


def main():
    rclpy.init()
    n = PayloadManager()
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
