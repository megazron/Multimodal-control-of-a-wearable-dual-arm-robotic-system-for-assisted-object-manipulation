#!/usr/bin/env python3
"""Sweep the wrist cameras, fingerprint the scene, and skip calibration if it
has not changed.

    ros2 run srl_perception scene_fingerprint_node
    ros2 service call /scene/resweep std_srvs/srv/Trigger
    ros2 topic echo /scene/state

FLOW
  startup  -> load the stored fingerprint (if any)
           -> sweep: drive each arm through a set of look poses, accumulate
              detections from /perception/objects
           -> compare against the store
           -> if unchanged, publish the STORED poses and report calibration
              skipped; otherwise re-register ONLY what changed
  running  -> every detection is drift-checked against the store. An
              observation that does not fit is FLAGGED, never acted on.

WHY THE STORED POSE IS REPUBLISHED RATHER THAN THE FRESH ONE WHEN THEY MATCH.
Two poses that agree within tolerance are equally valid, but the stored one
has been accumulated over more views. Swapping to a fresh single-sweep
estimate each start would inject a small random walk into a quantity the rest
of the system treats as fixed.

CAMERA CONTENTION. This node does NOT open a camera. It consumes
/perception/objects, which the detector owns. Two processes opening one camera
is the same class of fault as two readers on one serial port, and the fix is
the same: exactly one owner, everything else subscribes.
"""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray

from srl_perception import scene_fingerprint as sf

DEFAULT_STORE = os.path.expanduser("~/kortex_ws/config/scene_fingerprint.json")


class SceneFingerprintNode(Node):

    def __init__(self):
        super().__init__("scene_fingerprint_node")
        self.declare_parameter("store_path", DEFAULT_STORE)
        self.declare_parameter("pos_tol_m", 0.015)
        self.declare_parameter("gate_m", 0.25)
        self.declare_parameter("sweep_settle_s", 0.8)
        self.declare_parameter("min_views", 2)
        self.declare_parameter("auto_sweep_on_start", True)
        # How long to keep waiting for the FIRST detection before
        # declaring the sweep invalid. Covers DDS discovery.
        self.declare_parameter("sweep_max_wait_s", 8.0)

        self.store = sf.Fingerprint.load(
            self.get_parameter("store_path").value)
        self.acc = {}           # label -> accumulated observation
        self.sweeping = False
        self.state = "idle"
        self.last_summary = None
        self.drift_flags = []

        self.pub = self.create_publisher(String, "/scene/state", 10)
        self.pub_obj = self.create_publisher(String, "/scene/objects", 10)
        self.create_subscription(Detection3DArray, "/perception/objects",
                                 self.on_detections, 10)
        self.create_service(Trigger, "/scene/resweep", self.srv_resweep)
        self.create_service(Trigger, "/scene/forget", self.srv_forget)
        self.create_timer(0.5, self.tick)

        if self.store is None:
            self.get_logger().info(
                "no stored fingerprint: the first sweep will create one")
        else:
            self.get_logger().info(
                "loaded fingerprint with %d objects, swept in %s"
                % (len(self.store.objects),
                   ("%.1f s" % self.store.sweep_s) if self.store.sweep_s
                   else "unknown time"))
        if bool(self.get_parameter("auto_sweep_on_start").value):
            self.begin_sweep("startup")

    # ------------------------------------------------------------- sweeping
    def begin_sweep(self, why):
        self.acc = {}
        self.n_msgs = 0
        self.sweeping = True
        self.sweep_t0 = self.get_clock().now().nanoseconds * 1e-9
        self.state = "sweeping"
        self.get_logger().info("sweep started (%s)" % why)

    def on_detections(self, msg):
        if self.sweeping:
            self.n_msgs += 1
        for d in msg.detections:
            lab, score = "", 0.0
            if d.results:
                lab = d.results[0].hypothesis.class_id
                score = float(d.results[0].hypothesis.score)
            p = d.bbox.center.position
            q = d.bbox.center.orientation
            o = sf.Observation(lab, (p.x, p.y, p.z),
                               (q.x, q.y, q.z, q.w), score, 1,
                               (d.bbox.size.x, d.bbox.size.y, d.bbox.size.z))
            if self.sweeping:
                self.accumulate(o)
            elif self.store is not None:
                self.check_drift(o)

    def accumulate(self, o):
        """Running mean over views, weighted by confidence.

        A single view of a tag at a shallow angle is a much worse pose than a
        head-on one, and confidence is the only signal available for that.
        """
        cur = self.acc.get(o.label)
        if cur is None:
            self.acc[o.label] = o
            return
        w0, w1 = cur.confidence * cur.n_views, o.confidence
        tot = w0 + w1 or 1.0
        cur.xyz = tuple((a * w0 + b * w1) / tot
                        for a, b in zip(cur.xyz, o.xyz))
        cur.n_views += 1
        cur.confidence = max(cur.confidence, o.confidence)
        if o.quat and cur.quat is None:
            cur.quat = o.quat

    def end_sweep(self):
        dt = self.get_clock().now().nanoseconds * 1e-9 - self.sweep_t0
        # A SWEEP THAT RECEIVED NO DETECTION MESSAGES AT ALL IS INVALID, NOT A
        # SCENE IN WHICH EVERYTHING VANISHED. Treating the two the same
        # silently destroyed the store the first time this ran: the node
        # starts its sweep before DDS discovery has matched the detector, saw
        # zero messages, concluded every object had gone, and dropped them.
        # No data is not a negative result.
        if self.n_msgs == 0:
            wait = float(self.get_parameter("sweep_max_wait_s").value)
            if dt < wait:
                return                      # keep waiting, do not conclude
            self.sweeping = False
            self.state = "sweep_invalid_no_detections"
            self.get_logger().error(
                "SWEEP INVALID: no detection messages in %.1f s. The stored "
                "fingerprint is UNCHANGED. Check that a detector is running "
                "and publishing /perception/objects." % dt)
            return
        self.sweeping = False
        mv = int(self.get_parameter("min_views").value)
        obs = [o for o in self.acc.values() if o.n_views >= mv]
        dropped = [o.label for o in self.acc.values() if o.n_views < mv]
        if dropped:
            self.get_logger().info(
                "discarded %d single-view detection(s): %s -- one view is not "
                "a pose" % (len(dropped), ", ".join(dropped)))

        if self.store is None:
            self.store = sf.Fingerprint(obs, sf.now(), dt, "first sweep")
            self.store.save(self.get_parameter("store_path").value)
            self.state = "registered"
            self.last_summary = dict(first=True, n=len(obs), sweep_s=dt)
            self.get_logger().info(
                "FIRST FINGERPRINT: %d objects in %.1f s -> %s"
                % (len(obs), dt, self.get_parameter("store_path").value))
            return

        verdicts, summ = sf.compare(
            self.store, obs,
            pos_tol=float(self.get_parameter("pos_tol_m").value),
            gate=float(self.get_parameter("gate_m").value))
        summ["sweep_s"] = dt
        self.last_summary = summ
        if summ["unchanged"]:
            self.state = "match_skip_calibration"
            self.get_logger().info(
                "SCENE MATCHES (%d objects, max delta %.1f mm) -- "
                "calibration SKIPPED, reusing stored poses in %.1f s"
                % (summ["n_stored"],
                   1000 * max([v.get("delta_m") or 0.0 for v in verdicts]
                              or [0.0]), dt))
        else:
            self.state = "changed_reregistered"
            self.get_logger().warn("SCENE CHANGED in %.1f s:" % dt)
            for v in verdicts:
                if v["verdict"] == sf.SAME:
                    continue
                d = ("" if v.get("delta_m") is None
                     else " by %.1f mm" % (1000 * v["delta_m"]))
                self.get_logger().warn("    %-18s %s%s  %s"
                                       % (v["verdict"], v["label"], d,
                                          v.get("note", "")))
            # Re-register only what changed: keep every unchanged stored pose
            # and replace only the ones the diff named.
            keep = {o.label: o for o in self.store.objects
                    if o.label not in summ["reregister"]
                    and o.label not in summ["drop"]}
            for o in obs:
                if o.label in summ["reregister"]:
                    keep[o.label] = o
            self.store = sf.Fingerprint(list(keep.values()), sf.now(), dt,
                                        "partial re-registration")
            self.store.save(self.get_parameter("store_path").value)
            kept = sum(1 for v in verdicts if v["verdict"] == sf.SAME)
            self.get_logger().info(
                "re-registered %d object(s), kept %d unchanged, dropped %d; "
                "store now holds %d"
                % (len(summ["reregister"]), kept, len(summ["drop"]),
                   len(self.store.objects)))

    # ----------------------------------------------------------- drift
    def check_drift(self, o):
        ok, verdict, d = sf.drift_check(
            self.store, o,
            pos_tol=float(self.get_parameter("pos_tol_m").value),
            gate=float(self.get_parameter("gate_m").value))
        if ok:
            return
        key = (o.label, verdict)
        if key in [(f["label"], f["verdict"]) for f in self.drift_flags]:
            return
        f = dict(label=o.label, verdict=verdict,
                 delta_m=d, observed=list(o.xyz), confidence=o.confidence)
        self.drift_flags.append(f)
        # FLAG, do not act. The two live explanations -- the object moved, or
        # this is a misdetection -- call for opposite responses, and one
        # observation cannot separate them.
        self.get_logger().warn(
            "DRIFT FLAGGED: %s %s%s. Stored pose NOT updated and NOT trusted; "
            "call /scene/resweep to resolve."
            % (o.label, verdict,
               "" if d is None else " by %.1f mm" % (1000 * d)))

    # --------------------------------------------------------- services
    def srv_resweep(self, req, resp):
        self.drift_flags = []
        self.begin_sweep("service request")
        resp.success = True
        resp.message = "sweep started"
        return resp

    def srv_forget(self, req, resp):
        p = self.get_parameter("store_path").value
        if os.path.exists(p):
            os.remove(p)
        self.store = None
        resp.success = True
        resp.message = "fingerprint discarded; next sweep will re-create it"
        self.get_logger().warn(resp.message)
        return resp

    # ------------------------------------------------------------- tick
    def tick(self):
        if self.sweeping:
            settle = float(self.get_parameter("sweep_settle_s").value)
            el = self.get_clock().now().nanoseconds * 1e-9 - self.sweep_t0
            if el >= settle:
                self.end_sweep()
        m = String()
        m.data = json.dumps(dict(
            state=self.state, sweeping=self.sweeping,
            n_stored=0 if self.store is None else len(self.store.objects),
            summary=self.last_summary, drift_flags=self.drift_flags,
            store=self.get_parameter("store_path").value))
        self.pub.publish(m)
        if self.store is not None:
            mo = String()
            mo.data = json.dumps([o.as_dict() for o in self.store.objects])
            self.pub_obj.publish(mo)


def main(argv=None):
    rclpy.init(args=argv)
    n = SceneFingerprintNode()
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(
        KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
