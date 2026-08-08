#!/usr/bin/env python3
"""
runner.py — the shared experiment runner every E<n> builds on.

It owns the things that must be identical across experiments, because a
difference in any of them makes the experiments non-comparable:
  * how a trial starts, ends and is timed
  * what is sampled and at what rate
  * when a trial is marked INVALID
  * the safety interlocks that abort it

SCRIPTED MODE. `--scripted` drives the master from `ScriptedOperator` inside
this process, so the whole pipeline runs with no participant and no hardware.
That is how the harness is regression-tested; see Part 9.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from vision_msgs.msg import (BoundingBox3D, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)

import tf2_ros

from srl_experiments.trial_logger import TrialLogger


class ExperimentRunner(Node):
    """Base class. An experiment subclasses this and implements run()."""

    def __init__(self, name, args):
        super().__init__(f"runner_{name}")
        self.exp_name = name
        self.args = args
        self.cfg = self._load_config(args)
        self.arm = self.cfg.get("arm", "left")

        results = Path(args.results or
                       (Path(args.config).parent / "results"))
        results.mkdir(parents=True, exist_ok=True)
        self.log = TrialLogger(results, args.participant, name)

        # --- live state, all updated by callbacks ---
        self.master = None
        self.pointing = None
        self.cmd = None
        self.joints = {}
        self.autonomy_state = "DIRECT"
        self.intent = {}
        self.grasp_offered = False
        self.channels = {}
        self.ik_status = None
        self.payload = 0.0
        self.estop = False
        self.estop_events = 0
        # Aborts raised by the safety layer DURING a trial. recovery_manager,
        # channel_manager and participant_safety_node all publish here. The
        # trial must be marked INVALID with this cause -- a trial that was
        # frozen mid-reach and then continued is not a slow participant, and
        # nothing downstream can tell the difference once it is in the data.
        self.aborts = []
        self.clutch = ""

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        a = self.arm
        self.create_subscription(PoseStamped, f"/master_arm_pose_{a}",
                                 self._on_master, 10)
        self.create_subscription(Vector3Stamped, f"/master_pointing_{a}",
                                 self._on_point, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(String, f"/autonomy/state_{a}",
                                 lambda m: setattr(self, "autonomy_state", m.data), 10)
        self.create_subscription(String, f"/autonomy/intent_debug_{a}",
                                 self._on_intent, 10)
        self.create_subscription(String, f"/autonomy/grasp_status_{a}",
                                 self._on_grasp, 10)
        self.create_subscription(String, f"/master_capability_{a}",
                                 self._on_caps, 10)
        self.create_subscription(Float64MultiArray, f"/ik_status_{a}",
                                 lambda m: setattr(self, "ik_status", list(m.data)), 10)
        self.create_subscription(String, f"/payload_state_{a}", self._on_payload, 10)
        self.create_subscription(Bool, "/estop_state", self._on_estop, 10)
        self.create_subscription(String, "/participant/abort", self._on_abort, 10)
        # /participant/abort is a ONE-SHOT event and is easy to miss: it fires
        # once when a fault is raised, so a fault raised between two trials is
        # never seen by either. /recovery_state is CONTINUOUS state at 5 Hz,
        # so a trial can ask "was any fault outstanding while I ran?" instead
        # of hoping to catch an edge. The abort is still subscribed because it
        # carries the human-readable cause.
        self.recovery_faults = {}
        # ACCUMULATED across the trial, not sampled at the end: a fault that
        # is raised and then recovers mid-trial is exactly the case this has
        # to catch, and it would be gone from the current set by the time the
        # trial finished.
        self.recovery_seen = {}
        # "A trial is running right now, and it is this one." Published from
        # sample_once, which every experiment calls throughout a trial, so it
        # needs no cooperation from the individual experiment loops. Used by
        # the fault-acceptance harness to inject MID-trial rather than into
        # the gap between trials, and useful in its own right for watching a
        # session from another terminal.
        self.trial_pub = self.create_publisher(String, "/trial_state", 10)
        self.create_subscription(String, "/recovery_state",
                                 self._on_recovery, 10)
        # Clutch state rides on /master_status_<arm> field 0 (see
        # master_pose_node). Sampled per row: a trial in which the operator
        # clutched out is a different trial, and without this column that is
        # invisible afterwards.
        self.create_subscription(Float64MultiArray, f"/master_status_{a}",
                                 self._on_master_status, 10)
        # In SCRIPTED mode nothing else publishes objects, so the intent,
        # grasp and handover path would never be exercised and the pilot
        # would silently pass with those columns empty. The runner publishes
        # the scenario's own objects instead.
        self.object_pub = self.create_publisher(
            Detection3DArray, "/perception/objects", 10)
        self.op_cmd = self.create_publisher(
            String, f"/scripted_operator_cmd_{a}", 10)

    # ------------------------------------------------------------- config
    @staticmethod
    def _load_config(args):
        with open(args.config) as f:
            return yaml.safe_load(f) or {}

    # ---------------------------------------------------------- callbacks
    def _on_master(self, m):
        self.master = m

    def _on_point(self, m):
        self.pointing = np.array([m.vector.x, m.vector.y, m.vector.z])

    def _on_js(self, m):
        self.joints = dict(zip(m.name, m.position))

    def _on_intent(self, m):
        try:
            self.intent = json.loads(m.data)
        except ValueError:
            pass

    def _on_grasp(self, m):
        try:
            self.grasp_offered = bool(json.loads(m.data).get("offered"))
        except ValueError:
            pass

    def _on_caps(self, m):
        try:
            self.channels = json.loads(m.data)
        except ValueError:
            pass

    def _on_payload(self, m):
        try:
            self.payload = float(json.loads(m.data).get("mass_kg", 0.0))
        except ValueError:
            pass

    def _on_master_status(self, m):
        if m.data:
            self.clutch = int(m.data[0])

    def publish_objects(self, positions, ids=None):
        """Publish a scenario's objects as if perception had seen them."""
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        for i, pos in enumerate(positions):
            d = Detection3D()
            d.header = msg.header
            d.id = (ids[i] if ids else "tag_%d" % i)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = d.id
            h.hypothesis.score = 1.0
            h.pose.pose.position.x = float(pos[0])
            h.pose.pose.position.y = float(pos[1])
            h.pose.pose.position.z = float(pos[2])
            h.pose.pose.orientation.w = 1.0
            d.results.append(h)
            d.bbox = BoundingBox3D()
            d.bbox.center = h.pose.pose
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = 0.04
            msg.detections.append(d)
        self.object_pub.publish(msg)

    def _on_recovery(self, m):
        try:
            self.recovery_faults = json.loads(m.data).get("faults") or {}
        except (ValueError, TypeError):
            return
        for name, det in self.recovery_faults.items():
            if name not in self.recovery_seen:
                self.recovery_seen[name] = det.get("cause", name)
                self.get_logger().error(
                    "RECOVERY FAULT seen during this trial: %s - %s. The "
                    "trial will be marked INVALID with this cause."
                    % (name, det.get("cause", "?")))

    def _on_abort(self, m):
        try:
            rec = json.loads(m.data)
        except (ValueError, TypeError):
            rec = dict(cause=str(m.data))
        self.aborts.append(rec)
        self.get_logger().error(
            "ABORT during a trial: %s. The trial is marked INVALID with this "
            "cause; the session continues to the next trial."
            % rec.get("cause", "?"))

    def _on_estop(self, m):
        if m.data and not self.estop:
            self.estop_events += 1
            self.get_logger().error(
                "E-STOP during a trial. The trial is marked INVALID; it is "
                "never silently continued.")
        self.estop = bool(m.data)

    # ------------------------------------------------------------ helpers
    def spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def ee(self):
        try:
            t = self.tf_buf.lookup_transform(
                "world", f"{self.arm}_end_effector_link", rclpy.time.Time())
            v, r = t.transform.translation, t.transform.rotation
            return (np.array([v.x, v.y, v.z]),
                    np.array([r.x, r.y, r.z, r.w]))
        except Exception:
            return None, None

    def send_operator(self, p, point_at=None):
        m = String()
        m.data = ",".join("%.5f" % v for v in p)
        if point_at is not None:
            m.data += ";" + ",".join("%.5f" % v for v in point_at)
        self.op_cmd.publish(m)

    def required_channels(self, mode="spherical"):
        """Which channels this mode CANNOT run without.

        Spherical position uses j1 (azimuth) and the j2/j4 bends (reach), plus
        the accelerometer for elevation. A dropout on any of those mid-trial
        invalidates the trial: continuing silently would put a frozen command
        in the data and call it a slow participant.
        """
        return (["j1", "j2", "j4", "accel"] if mode == "spherical"
                else ["j%d" % i for i in range(1, 6)] + ["accel"])

    def channels_ok(self, mode="spherical"):
        ch = self.channels.get("channels")
        if not ch:
            return True, []          # no channel manager running: cannot judge
        bad = [c for c in self.required_channels(mode) if not ch.get(c, True)]
        return (not bad), bad

    # ------------------------------------------------------------- sample
    def sample_once(self, phase=""):
        t = self.log.trial
        if t is not None:
            st = String()
            st.data = json.dumps(dict(running=True, experiment=self.log.experiment,
                                      trial_index=t.get("trial_index"),
                                      condition=t.get("condition"), phase=phase))
            self.trial_pub.publish(st)
        ee_p, ee_q = self.ee()
        m = self.master
        row = dict(phase=phase, autonomy_state=self.autonomy_state,
                   grasp_offered=int(self.grasp_offered), clutch=self.clutch,
                   payload_kg=self.payload, estop=int(self.estop))
        if m is not None:
            row.update(master_x=m.pose.position.x, master_y=m.pose.position.y,
                       master_z=m.pose.position.z,
                       master_qx=m.pose.orientation.x, master_qy=m.pose.orientation.y,
                       master_qz=m.pose.orientation.z, master_qw=m.pose.orientation.w,
                       cmd_x=m.pose.position.x, cmd_y=m.pose.position.y,
                       cmd_z=m.pose.position.z)
        if self.pointing is not None:
            row.update(pointing_x=self.pointing[0], pointing_y=self.pointing[1],
                       pointing_z=self.pointing[2])
        if ee_p is not None:
            row.update(ee_x=ee_p[0], ee_y=ee_p[1], ee_z=ee_p[2],
                       ee_qx=ee_q[0], ee_qy=ee_q[1], ee_qz=ee_q[2], ee_qw=ee_q[3])
        g = self.joints.get(f"{self.arm}_robotiq_85_left_knuckle_joint")
        if g is not None:
            row["gripper_rad"] = g
        if self.intent:
            row.update(intent_top=self.intent.get("top") or "",
                       intent_top_p=self.intent.get("top_p", ""),
                       intent_margin=self.intent.get("margin", ""),
                       intent_ambiguous=int(bool(self.intent.get("ambiguous"))),
                       intent_dist=json.dumps(self.intent.get("p", [])),
                       intent_ids=json.dumps(self.intent.get("ids", [])))
        if self.channels:
            row["channels_live"] = json.dumps(self.channels.get("channels", {}))
        if self.ik_status:
            s = self.ik_status
            row.update(ik_success=s[0], ik_fail=s[1],
                       ik_rejected=s[4] if len(s) > 4 else "")
            if len(s) > 5:
                row["clearance_m"] = s[5]
        self.log.sample(**row)

    # ----------------------------------------------------------- geometry
    @staticmethod
    def path_length(points):
        if len(points) < 2:
            return 0.0
        p = np.asarray(points, float)
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())

    @staticmethod
    def fitts_id(distance, width):
        """Shannon formulation (MacKenzie): ID = log2(D/W + 1)."""
        return math.log2(max(1e-9, distance) / max(1e-9, width) + 1.0)
