#!/usr/bin/env python3
"""recovery_manager.py — freeze, abort, recover. One node, four failure paths.

PART 2 (b)-(e). Each of these has stopped a run on this rig or would stop one
with a participant in the chair. The shared shape is deliberate and is the
whole point:

    DETECT -> FREEZE -> ABORT (trial INVALID, cause recorded) -> RECOVER

and RECOVER never means "relaunch the stack", because a participant session
cannot survive a relaunch: the arm re-homes, the condition order is lost and
the block has to be discarded.

    (b) TEENSY DISCONNECT. The board re-enumerates between /dev/ttyACM0 and
        /dev/ttyACM1 on every usbipd re-attach, so recovery is not "reopen the
        port" but "find the port again". master_pose_node already exits on a
        dead port so launch respawns it with a fresh find_port(); this node
        supplies the half that was missing -- the trial is aborted with a
        cause, and the NEXT trial is gated until the master is demonstrably
        publishing FRESH data again.

    (c) KORTEX SESSION LOSS. The driver cannot be reactivated in-process:
        deactivating tears down the API router and on_activate then fails with
        `KBasicException: Router is not active`. So recovery means a NEW
        session -- and the arm permits exactly ONE, so the old one must be
        closed cleanly first or the next connect is refused. That sequence is
        owned by kortex_highlevel_bridge (it holds the session); this node
        calls its /real/session_recover service and gates on the result.

    (d) CAMERA FAILURE / ZERO DETECTIONS. Handled BEFORE the trial, never
        during: see scan_ok(), which the experiment runner calls in the scan
        phase. Aborting mid-trial here would produce a truncated trial whose
        cause is indistinguishable from a slow participant.

    (e) NETWORK DROPOUT TO EITHER ARM. Freezes BOTH arms, not just the one
        that lost its link. A two-arm rig on a wearer with one arm live and
        one frozen is a worse state than both frozen, and the operator cannot
        tell which is which by looking.

INJECTION. Every path is driven by observable ROS state, so a fault is
injected by publishing the topic the real fault would affect. There are no
test hooks in this file.
"""
import json
import time

import rclpy
from rclpy.node import Node
import rclpy.time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from srl_teleop.block_monitor import BlockMonitor

ARMS = ("left", "right")


class RecoveryManager(Node):
    def __init__(self):
        super().__init__("recovery_manager")
        self.declare_parameter("arms", list(ARMS))
        self.declare_parameter("master_stale_s", 1.0)
        self.declare_parameter("session_stale_s", 2.0)
        self.declare_parameter("link_stale_s", 3.0)
        self.declare_parameter("min_scan_detections", 1)
        self.declare_parameter("scan_settle_s", 1.0)
        self.declare_parameter("reconnect_confirm_s", 1.0)
        self.declare_parameter("serial_port", "auto")
        # The Kortex session and the arm links only EXIST on a real stack. On
        # a sim-only stack their topics are absent, and absence of an optional
        # subsystem is not a fault -- treating it as one would abort every sim
        # trial. With this false, those two paths fire only on an EXPLICIT
        # "connected: false" / "up: false" report, never on silence. The real
        # launch sets it true, where silence genuinely means a crash.
        self.declare_parameter("expect_real_stack", False)
        # Whether losing the master also FREEZES the arm, as opposed to only
        # aborting the trial. Defaults OFF for the same reason estop_node's
        # dead-man does: in sim there is no Teensy, the master goes stale
        # constantly, and freezing on it makes a healthy sim look like dead
        # hardware -- which has cost this project several hours already.
        # The trial is still marked INVALID either way; that is data
        # integrity and is never optional. Stopping the arm on master loss is
        # the DEAD-MAN's job, and teleop.launch.py arms it whenever real_arms
        # or real_robot is set.
        self.declare_parameter("freeze_on_master_loss", False)

        self.arms = list(self.get_parameter("arms").value)
        self.faults = {}            # name -> dict(since, cause, recovered)
        self.deadman = {}
        self.session = {}
        self.link = {}
        self.objects_n = None
        self.master_fresh_since = None

        self.abort_pub = self.create_publisher(String, "/participant/abort", 10)
        self.state_pub = self.create_publisher(String, "/recovery_state", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)

        self.blocks = BlockMonitor(self, "recovery_manager")
        for nm, what, rec in (
            ("teensy_lost", "master serial link is down or publishing stale data",
             "the board re-enumerates; master_pose_node respawns and re-runs "
             "find_port(). The next trial is gated until it publishes fresh."),
            ("kortex_session_lost", "Kortex session is gone or stale",
             "call /real/session_recover: zero speeds, Stop(), CloseSession(), "
             "then a fresh CreateSession. NOT a relaunch."),
            ("arm_link_down", "network link to an arm is down",
             "both arms freeze; clears when /arm_link_status reports both up"),
            ("scan_failed", "scan phase saw no objects",
             "fix the camera or the scene, then re-run the scan. The trial is "
             "refused BEFORE it starts, never aborted during."),
        ):
            self.blocks.register(nm, what, rec)

        # Master health is measured HERE, from the pose stamps directly,
        # rather than read off the dead-man's published state. The dead-man is
        # OFF by default in sim, and its age_s field is only populated while
        # armed -- so keying on it made this path silently inert exactly when
        # nobody had armed the dead-man. The two mechanisms must not share a
        # single point of failure anyway: this one aborts a trial, that one
        # stops the arm.
        self.master_src = {}
        for a in self.arms:
            self.create_subscription(
                PoseStamped, "/master_arm_pose_%s" % a,
                self._pose_cb(a), 20)
        self.create_subscription(String, "/estop_deadman", self._on_deadman, 10)
        self.create_subscription(String, "/real/session_state",
                                 self._on_session, 10)
        self.create_subscription(String, "/arm_link_status", self._on_link, 10)
        self.create_subscription(String, "/perception/objects_info",
                                 self._on_objects, 10)
        self.recover_cli = self.create_client(Trigger, "/real/session_recover")
        self.create_service(Trigger, "/recovery/ready", self._srv_ready)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            "recovery_manager up. Freeze -> abort -> recover, for teensy "
            "loss, Kortex session loss, arm link loss and scan failure. "
            "Recovery never requires a relaunch.")

    # --------------------------------------------------------------- inputs
    def _pose_cb(self, arm):
        def cb(m):
            st = m.header.stamp
            if st.sec == 0 and st.nanosec == 0:
                return
            self.master_src[arm] = rclpy.time.Time.from_msg(st)
        return cb

    def _on_deadman(self, m):
        try:
            self.deadman = json.loads(m.data)
        except ValueError:
            pass

    def _on_session(self, m):
        try:
            self.session = json.loads(m.data)
        except ValueError:
            self.session = {}
        self.session["_rx"] = time.monotonic()

    def _on_link(self, m):
        try:
            self.link = json.loads(m.data)
        except ValueError:
            return
        self.link["_rx"] = time.monotonic()

    def _on_objects(self, m):
        try:
            self.objects_n = int(json.loads(m.data).get("n", 0))
        except (ValueError, TypeError):
            self.objects_n = None

    # ---------------------------------------------------------------- core
    def _raise(self, name, cause, freeze=True):
        """Detect -> freeze -> abort. Idempotent: only the first raises."""
        if name in self.faults:
            return
        self.faults[name] = dict(since=time.monotonic(), cause=cause,
                                 recovered=False)
        self.blocks.block(name, cause)
        if freeze:
            # FREEZE FIRST. The abort is bookkeeping; stopping the arm is not.
            b = Bool()
            b.data = True
            self.estop_pub.publish(b)
        rec = dict(t=time.strftime("%Y-%m-%dT%H:%M:%S"), kind="abort",
                   fault=name, cause=cause, arm="both",
                   recovery_required=True)
        s = String()
        s.data = json.dumps(rec)
        self.abort_pub.publish(s)
        self.get_logger().error(
            "[%s] FROZEN and trial ABORTED (INVALID): %s. Recovery is in "
            "progress; the next trial is gated until it completes."
            % (name, cause))

    def _resolve(self, name, how):
        f = self.faults.get(name)
        if f is None or f["recovered"]:
            return
        f["recovered"] = True
        f["recovered_after_s"] = round(time.monotonic() - f["since"], 2)
        self.blocks.clear(name)
        self.get_logger().warn(
            "[%s] RECOVERED after %.1f s: %s. The stack was NOT relaunched."
            % (name, f["recovered_after_s"], how))
        del self.faults[name]

    # --------------------------------------------------------- (b) teensy
    def _check_teensy(self):
        """Master link health, from the SOURCE stamp of the pose itself.

        Keys on source age, not on whether the topic has traffic:
        master_pose_node republishes the last good frame at 50 Hz when
        validation fails, so a busy topic proves nothing at all.
        """
        if not self.master_src:
            return                       # master has never published: sim run
        now = self.get_clock().now()
        stale = float(self.get_parameter("master_stale_s").value)
        ages = {a: (now - t).nanoseconds / 1e9
                for a, t in self.master_src.items()}
        bad = [a for a, v in ages.items() if v > stale]
        if bad:
            self._raise("teensy_lost",
                        "master source data stale on %s (>%.2f s) -- the board "
                        "has gone or is republishing a frozen frame"
                        % (",".join(sorted(bad)), stale),
                        freeze=bool(self.get_parameter(
                            "freeze_on_master_loss").value))
            self.master_fresh_since = None
            return
        now_m = time.monotonic()
        if self.master_fresh_since is None:
            self.master_fresh_since = now_m
        need = float(self.get_parameter("reconnect_confirm_s").value)
        if now_m - self.master_fresh_since >= need and "teensy_lost" in self.faults:
            port = self._find_port()
            self._resolve("teensy_lost",
                          "master publishing fresh data for %.1f s; port is %s "
                          "(it MOVES between ACM0 and ACM1 on re-enumeration, "
                          "so it was re-detected, not reopened)" % (need, port))

    def _find_port(self):
        try:
            from srl_teleop.serial_port import find_port, PortNotFound
            try:
                return find_port(
                    self.get_parameter("serial_port").value or None)
            except PortNotFound:
                return "not found"
        except Exception as e:                                  # noqa: BLE001
            return "lookup failed: %s" % e

    # -------------------------------------------------- (c) kortex session
    def _check_session(self):
        s = self.session
        if not s:
            return                       # no real stack: nothing to judge
        stale = float(self.get_parameter("session_stale_s").value)
        age = time.monotonic() - s.get("_rx", 0.0)
        real = bool(self.get_parameter("expect_real_stack").value)
        lost = (not s.get("connected", True)) or (real and age > stale)
        if lost:
            self._raise("kortex_session_lost",
                        "session %s (last report %.1f s ago)"
                        % ("reported LOST" if not s.get("connected", True)
                           else "went silent", age))
            self._recover_session()
        elif s.get("connected") and "kortex_session_lost" in self.faults:
            self._resolve("kortex_session_lost",
                          "a FRESH session is up (old one closed first -- the "
                          "arm permits only one)")

    def _recover_session(self):
        f = self.faults.get("kortex_session_lost")
        if f is None or f.get("requested"):
            return
        if not self.recover_cli.service_is_ready():
            self.get_logger().error(
                "[kortex_session_lost] /real/session_recover is NOT available. "
                "Recovery needs the bridge process alive to close the old "
                "session cleanly; the arm permits exactly one session, so a "
                "leaked one blocks every later connect. Start the bridge.")
            return
        f["requested"] = True
        self.get_logger().warn(
            "[kortex_session_lost] requesting in-process recovery: zero "
            "speeds -> Stop() -> CloseSession() -> fresh CreateSession. The "
            "hardware component is NOT deactivated, because deactivating "
            "tears down the router and on_activate then fails with "
            "'Router is not active'.")
        self.recover_cli.call_async(Trigger.Request())

    # ----------------------------------------------------- (e) arm network
    def _check_link(self):
        if not self.link:
            return
        stale = float(self.get_parameter("link_stale_s").value)
        age = time.monotonic() - self.link.get("_rx", 0.0)
        real = bool(self.get_parameter("expect_real_stack").value)
        arms = self.link.get("arms") or {}
        down = [a for a, v in arms.items() if not v.get("up", True)]
        if real and age > stale:
            down = down or ["monitor silent"]
        if down:
            self._raise("arm_link_down",
                        "link DOWN to %s -- BOTH arms frozen, because one arm "
                        "live and one frozen on a wearer is worse than both "
                        "frozen and cannot be told apart by looking"
                        % ",".join(sorted(down)))
        elif arms and "arm_link_down" in self.faults:
            self._resolve("arm_link_down", "both arm links are up again")

    # ------------------------------------------------------- (d) scan gate
    def scan_ok(self):
        n = self.objects_n
        need = int(self.get_parameter("min_scan_detections").value)
        if n is None:
            return False, "no /perception/objects_info -- is the detector running?"
        if n < need:
            return False, "scan saw %d objects, need >= %d" % (n, need)
        return True, "scan saw %d objects" % n

    def _srv_ready(self, req, res):
        """Is it safe to START the next trial? Includes the scan gate.

        The scan is evaluated FIRST and resolves its own fault. Checking the
        outstanding-fault list first made scan_failed unclearable: it is
        itself an outstanding fault, so it short-circuited the very check that
        would have cleared it -- the unreachable-clear pattern again, in a
        service handler.
        """
        ok_scan, why_scan = self.scan_ok()
        if ok_scan:
            self._resolve("scan_failed", why_scan)
        else:
            self._raise("scan_failed", why_scan, freeze=False)

        outstanding = sorted(self.faults)
        if not ok_scan:
            res.success = False
            res.message = ("scan phase FAILED before the trial started: %s"
                           % why_scan)
        elif outstanding:
            res.success = False
            res.message = "recovery outstanding: %s" % ",".join(outstanding)
        else:
            res.success = True
            res.message = why_scan
        return res

    # -------------------------------------------------------------- report
    def _tick(self):
        self._check_teensy()
        self._check_session()
        self._check_link()
        for n in list(self.faults):
            self.blocks.block(n, self.faults[n]["cause"])
        m = String()
        m.data = json.dumps(dict(
            faults={k: dict(cause=v["cause"],
                            held_s=round(time.monotonic() - v["since"], 2))
                    for k, v in self.faults.items()},
            recovery_outstanding=sorted(self.faults),
            ready_for_next_trial=not self.faults,
            scan=dict(zip(("ok", "detail"), self.scan_ok()))))
        self.state_pub.publish(m)


def main():
    rclpy.init()
    n = RecoveryManager()
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
