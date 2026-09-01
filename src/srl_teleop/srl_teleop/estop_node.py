#!/usr/bin/env python3
"""
estop_node.py — emergency stop for both arms. Required before real motion.

Three independent triggers, any of which latches STOPPED:

  1. SERVICE  /estop        (std_srvs/Trigger)  -- halt now
     SERVICE  /estop_reset  (std_srvs/Trigger)  -- explicit reset, the only
                                                   way out of the latch
     TOPIC    /estop        (std_msgs/Bool)     -- same, for scripts

  2. BOTH BUTTONS at once on the master (btn1 AND btn2 changing within
     BOTH_WINDOW_S). Measured mapping is left->btn2, right->btn1, so this is
     "squeeze both". It is a convenience, NOT the primary path.

  3. DEAD-MAN: /master_arm_pose_<arm> going stale beyond `stale_timeout`
     (default 0.5 s). The Teensy has re-enumerated repeatedly; a dead master
     must not leave an arm executing the last trajectory it was handed.

The e-stop DOES NOT depend on the master arm working -- triggers 1 and 3 both
function with the Teensy unplugged, and trigger 3 fires *because* of it.

Halting means: publish the arm's CURRENT measured joint positions as a
trajectory with a short time_from_start. That actively parks it where it is,
which beats letting a stale trajectory run to completion. It republishes
while latched, so a follower that keeps commanding is continuously overridden.

State is published on /estop_state (Bool, latched=stopped).
"""
import json
import sys
import time
import threading

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import (
    SwitchController, SetHardwareComponentState)
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

BOTH_WINDOW_S = 0.4


class EStop(Node):
    def __init__(self):
        super().__init__("estop_node")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("hold_period_s", 0.1)
        # DEAD-MAN DEFAULTS OFF.
        # It exists to stop a REAL arm hurting a person when the master dies.
        # In mock-only sim there is no such hazard -- a stale master just means
        # "hold", not "latch the whole system". Left on by default it made the
        # sim look like dead hardware three times: CPU load from a colcon build
        # or an IK sweep stalls the 50 Hz master publisher past the window, the
        # e-stop latches, and from then on the ONLY thing reaching the
        # controller is the e-stop's own hold-position at 10 Hz.
        # teleop.launch.py turns it ON whenever real_arms or real_robot is set.
        self.declare_parameter("deadman_enabled", False)
        # The dead-man must survive a CPU hiccup. A single late frame is not
        # a dead Teensy: heavy IK sweeps starved /master_arm_pose_* past the
        # 0.5 s window twice and latched the e-stop, which looked exactly
        # like dead hardware and froze the whole sim. Require N CONSECUTIVE
        # stale checks before tripping.
        self.declare_parameter("deadman_consecutive", 5)
        # How close together the two button presses must be to e-stop.
        # A WINDOW BETWEEN TWO EDGES, not a hold, because the firmware
        # buttons are toggles and a hold is not observable. See on_buttons().
        # Long enough to be an instinctive two-handed squeeze, short enough
        # that two deliberate gating presses seconds apart cannot reach it.
        # SET THIS <= 0 TO DISABLE THE BUTTON TRIGGER ENTIRELY.
        #
        # Trigger 2 is documented at the top of this file as a CONVENIENCE and
        # explicitly not the primary path: the service, the topic and the
        # dead-man are unaffected by this and remain the real e-stop. On a rig
        # whose button wiring is faulty -- 2026-09-01, both channels flipping
        # together with nobody touching them, latching mid-session twice --
        # a convenience trigger that fires by itself is not a safety feature,
        # it is an outage that teaches the operator to bypass the e-stop.
        # Disabling it is therefore SAFER than leaving it armed, and it is a
        # parameter rather than a code change so it can be re-armed the moment
        # the wiring is fixed.
        self.declare_parameter("both_window_s", 0.40)
        # Silence longer than this on /master_fsr_buttons means the next
        # message starts a NEW stream, and its values are history rather than
        # edges. Comfortably longer than a dropped frame at 50 Hz, far
        # shorter than any reattach. See on_buttons().
        self.declare_parameter("btn_gap_s", 1.0)
        # DRIVER-LEVEL HALT. Publishing a hold-position trajectory only works
        # if an arm trajectory controller is loaded and active. On the real
        # stack it may not be (the read-only bring-up deliberately loads
        # none), so the e-stop needs a path that does not depend on the
        # trajectory topic at all: deactivate the controllers, then set the
        # hardware component itself INACTIVE so the driver stops commanding.
        self.declare_parameter("real_cm", "/real/controller_manager")
        self.declare_parameter("real_components",
                               ["left_KortexMultiInterfaceHardware"])
        self.declare_parameter("real_controllers",
                               ["left_arm_controller", "right_arm_controller"])
        self.arms = list(self.get_parameter("arms").value)
        self.stale = float(self.get_parameter("stale_timeout_s").value)
        self.deadman = bool(self.get_parameter("deadman_enabled").value)
        self.deadman_n = int(self.get_parameter("deadman_consecutive").value)
        self.stale_count = {a: 0 for a in self.arms}
        self.hold = float(self.get_parameter("hold_period_s").value)
        self.both_window_s = float(self.get_parameter("both_window_s").value)
        self.btn_gap_s = float(self.get_parameter("btn_gap_s").value)

        self.stopped = False
        self.reason = ""
        self.js = {}
        self.last_pose = {a: None for a in self.arms}
        self.unstamped = {a: 0 for a in self.arms}
        self.last_rx = {a: None for a in self.arms}
        # When the dead-man was last armed. Gives an arm that has never
        # published a deadline to be measured against, and gives a grace
        # window on arming so enabling the dead-man does not instantly trip
        # on history from before it was on.
        self.deadman_armed_t = None
        self._deadman_prev = False
        # Per-button latched value and the time it last CHANGED. Both are
        # per-button, so two edges on one bouncing button can never look
        # like a two-handed squeeze. See on_buttons().
        self.btn_last = {1: None, 2: None}
        self.btn_edge = {1: None, 2: None}
        #: When a button message last arrived. A GAP in this stream means the
        #: values on either side of it are not comparable -- see on_buttons().
        self.btn_rx = None

        self.traj_pub = {
            a: self.create_publisher(
                JointTrajectory, f"/{a}_arm_controller/joint_trajectory", 10)
            for a in self.arms
        }
        self.state_pub = self.create_publisher(Bool, "/estop_state", 10)
        self.deadman_pub = self.create_publisher(String, "/estop_deadman", 10)

        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_subscription(Bool, "/estop", self.on_topic, 10)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons", self.on_buttons, 20)
        for a in self.arms:
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}", self._on_pose_factory(a), 20)

        # ONE CLIENT PER ARM for the bridge's own halt. Created here, beside
        # the controller-manager clients, so both layers are set up in one
        # place and neither can be forgotten when the other is edited.
        self._halt_cli = {a: self.create_client(
            Trigger, "/real/emergency_halt_%s" % a)
            for a in ("left", "right")}
        cm = self.get_parameter("real_cm").value
        self.sw_cli = self.create_client(SwitchController, cm + "/switch_controller")
        self.hw_cli = self.create_client(
            SetHardwareComponentState, cm + "/set_hardware_component_state")

        self.create_service(Trigger, "/estop_real_halt", self.srv_real_halt)
        self.create_service(Trigger, "/estop", self.srv_stop)
        self.create_service(Trigger, "/estop_reset", self.srv_reset)
        self.create_timer(float(self.get_parameter("hold_period_s").value),
                          self.tick)

        self.get_logger().warn(
            "E-STOP ACTIVE. /estop (service or Bool topic) halts and LATCHES; "
            "/estop_reset is the only way out. Dead-man %s at %.2f s."
            % ("ON" if self.deadman else "OFF", self.stale))

    # ---------------- inputs ----------------

    def _on_pose_factory(self, arm):
        """Dead-man clock = the SOURCE timestamp, never message arrival.

        Keying on arrival was a latent hole with exactly one symptom and no
        warning. master_pose_node does NOT go silent when the master degrades:
        on a validation failure it republishes the last good joint vector at
        the full 50 Hz. An arrival-based dead-man sees a perfectly healthy
        topic and never fires, so the arm keeps tracking a frozen command --
        with the rig bolted to a person. Only a clean unplug (which exits the
        node) ever produced silence, which is why the mechanism looked correct
        in every test that pulled the cable.

        master_pose_node now stamps a substituted frame with the time of the
        data it was built from, so header.stamp goes stale the moment the
        master stops supplying anything new, whether or not it keeps talking.

        A zero stamp is treated as ABSENT rather than as 1970: a publisher that
        forgets to stamp would otherwise trip the dead-man permanently, and an
        e-stop that cannot be cleared is its own hazard.
        """
        def cb(m):
            st = m.header.stamp
            if st.sec == 0 and st.nanosec == 0:
                self.unstamped[arm] += 1
                if self.unstamped[arm] in (1, 200):
                    self.get_logger().warn(
                        "/master_arm_pose_%s arrives with an empty header "
                        "stamp; the dead-man is falling back to arrival time "
                        "for this arm and CANNOT see a frozen-but-publishing "
                        "master. Fix the publisher." % arm)
                self.last_pose[arm] = self.get_clock().now()
                self.last_rx[arm] = time.monotonic()
                return
            self.last_pose[arm] = rclpy.time.Time.from_msg(st)
            # ARRIVAL time is tracked too, on the monotonic clock, purely so
            # the two can be compared. Source-age and arrival-age agreeing
            # means a healthy master; source-age growing while arrival-age
            # stays near zero is the signature of a master that is
            # republishing stale data at full rate, which is the failure this
            # dead-man exists to catch.
            self.last_rx[arm] = time.monotonic()
        return cb

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def on_topic(self, m):
        if m.data:
            self.trip("/estop topic")

    def on_buttons(self, m):
        """BOTH BUTTONS PRESSED TOGETHER, measured as two FRESH EDGES.

        THE BUTTONS ARE FIRMWARE TOGGLES, NOT MOMENTARY CONTACTS.
        `firmware/master_arm/teensy_final.ino` runs both through
        updateButtonToggle(): the value published on /master_fsr_buttons is a
        LATCHED 0/1 that flips once per physical press and then STAYS there
        until the next press or a board reset. So "held down together" is not
        an observable state at all -- a button toggled on an hour ago and a
        finger resting on one right now publish the identical 1.

        Reading those latches as levels is what broke the 2026-09-01 capture.
        btn1 was still toggled on from an earlier session; the operator
        pressed the LEFT button to start segment 1; both latches then read 1
        and stayed there, so this tripped 0.30 s into the first recording and
        LATCHED. Every retry afterwards refused before it started, and the
        recorder's own message said the latch "came from somewhere else". It
        had come from the gating press, every time.

        An EDGE cannot go stale, which is the whole point: a toggle sitting
        at 1 produces none. So a button left latched arms nothing, the
        deliberate two-handed gesture trips from ANY starting state (each
        press is an edge whichever way it flips), and bounce on a single
        button is one button's edge and can never be two.

        This is not the pre-2026-08-24 rule, which counted any two edges in a
        window and so tripped on ONE arm's gating tap. The edges must be on
        the two DIFFERENT buttons.
        """
        if len(m.data) < 4:
            return
        if self.both_window_s <= 0.0:
            return                      # button trigger disabled; see above
        now = self.get_clock().now().nanoseconds / 1e9

        # A GAP IN THE STREAM IS NOT A GESTURE.
        #
        # Measured 2026-09-01, on the fix above: the Teensy was detached and
        # reattached to clear its dead IMUs. It came back with both toggles
        # reset to 0 -- they had both been latched at 1 before -- so the first
        # message after the gap carried TWO edges in one sample, and this
        # tripped the e-stop on "both buttons pressed together". Nobody
        # touched a button; the device had rebooted.
        #
        # Edges are only meaningful between two CONSECUTIVE observations of a
        # live stream. Across a gap -- a reboot, a reattach, a restarted
        # master node, a dropped USB link -- the previous value describes a
        # device that no longer exists. So the first message after a gap is
        # adopted as history, exactly as the very first message is.
        gap = self.btn_rx is None or (now - self.btn_rx) > self.btn_gap_s
        self.btn_rx = now
        if gap:
            for idx in (1, 2):
                self.btn_last[idx] = bool(m.data[1 + idx] > 0.5)
            self.btn_edge = {1: None, 2: None}
            return

        for idx in (1, 2):
            val = bool(m.data[1 + idx] > 0.5)
            if self.btn_last[idx] is None:
                # FIRST SIGHT: record, do not act. Whatever the toggles
                # happen to be latched at when this node starts is history,
                # not a press, and treating it as one would trip the e-stop
                # on the first message after every launch.
                self.btn_last[idx] = val
                continue
            if val != self.btn_last[idx]:
                self.btn_last[idx] = val
                self.btn_edge[idx] = now
        e1, e2 = self.btn_edge[1], self.btn_edge[2]
        if e1 is not None and e2 is not None and abs(e1 - e2) <= self.both_window_s:
            # CONSUME both edges, so one gesture is one trip and the pair
            # cannot re-fire against a stale partner on the next message.
            self.btn_edge = {1: None, 2: None}
            self.trip("both master buttons pressed together within %.2f s"
                      % self.both_window_s)

    def srv_stop(self, req, resp):
        self.trip("/estop service")
        resp.success = True
        resp.message = "E-STOP LATCHED. /estop_reset to clear."
        return resp

    def srv_reset(self, req, resp):
        was = self.stopped
        self.stopped = False
        self.reason = ""
        # Restart the dead-man's counters. Without this the reset is measured
        # against staleness that PREDATES it, so a latch caused by a brief gap
        # re-trips on the very next tick and the operator cannot clear it --
        # the latch becomes unclearable for a reason nothing reports.
        # This does not weaken the dead-man: if the master really is dead, it
        # simply trips again one full deadline later, which is the same
        # guarantee it makes at every other moment.
        for a in self.arms:
            self.stale_count[a] = 0
        # DROP ANY PENDING BUTTON EDGE. Without this the pair that tripped
        # the e-stop is still sitting in btn_edge, so the very next
        # /master_fsr_buttons message re-trips it and the operator cannot
        # clear the latch -- the same unclearable-latch shape the stale
        # counters above are reset to avoid.
        self.btn_edge = {1: None, 2: None}
        self.deadman_armed_t = self.get_clock().now()
        self.get_logger().warn("E-STOP RESET -- motion permitted again.")
        resp.success = True
        resp.message = "reset (was %s)" % ("STOPPED" if was else "running")
        return resp

    def halt_real_driver(self):
        """Stop the REAL arm without touching the trajectory topic.

        Two independent steps, both best-effort and both logged:
          1. deactivate any arm trajectory controllers, so nothing can send
             new setpoints;
          2. set the Kortex hardware component INACTIVE, which stops the
             driver's own command path at the source.
        Neither commands a motion; both remove the ability to command one.

        NOTE ON BLOCKING -- do not reintroduce wait_for_service() here.
        This runs inside trip(), which runs inside the 10 Hz timer of a
        SingleThreadedExecutor. Two wait_for_service(timeout_sec=2.0) calls
        used to sit in this function, so on any stack WITHOUT a /real
        controller_manager -- i.e. every sim and mock run -- an e-stop blocked
        the whole node for a guaranteed 4.0 s before parking the arms,
        before logging, and before /estop_state could change. Measured: the
        dead-man decided at 0.97 s and the trip was not logged until 4.98 s.
        The detection was never late; the HALT was. service_is_ready() is
        non-blocking and the calls below are already call_async, so when the
        real CM does exist the request still goes out immediately.
        """
        done = []
        ctrls = list(self.get_parameter("real_controllers").value)
        if self.sw_cli.service_is_ready():
            r = SwitchController.Request()
            r.deactivate_controllers = ctrls
            r.strictness = SwitchController.Request.BEST_EFFORT
            f = self.sw_cli.call_async(r)
            done.append("switch_controller(deactivate %s) sent" % ",".join(ctrls))
        else:
            done.append("switch_controller UNAVAILABLE")

        comps = list(self.get_parameter("real_components").value)
        if self.hw_cli.service_is_ready():
            for c in comps:
                r = SetHardwareComponentState.Request()
                r.name = c
                r.target_state.id = State.PRIMARY_STATE_INACTIVE
                r.target_state.label = "inactive"
                self.hw_cli.call_async(r)
                done.append("hardware '%s' -> inactive sent" % c)
        else:
            done.append("set_hardware_component_state UNAVAILABLE")
        # THE HIGH-LEVEL BRIDGE, which is the path a real run actually uses.
        #
        # Everything above targets /real/controller_manager -- the ros2_control
        # CASCADE -- and the supported real path is kortex_highlevel_bridge,
        # which has no controller manager and no hardware component. So on a
        # real high-level run every branch above logged "UNAVAILABLE" and the
        # driver-level halt did nothing, while the arm still stopped for a
        # different reason (the bridge's own /estop_state subscription). A
        # second layer that silently does nothing is worse than none, because
        # it reads as defence in depth.
        for arm in ("left", "right"):
            cli = self._halt_cli.get(arm)
            if cli is not None and cli.service_is_ready():
                cli.call_async(Trigger.Request())
                done.append("bridge %s /real/emergency_halt_%s sent"
                            % (arm, arm))
            else:
                done.append("bridge %s emergency_halt UNAVAILABLE" % arm)

        # AND IF NOTHING WAS REACHABLE, SAY SO ONCE, LOUDLY. Four "UNAVAILABLE"
        # lines are a list; "no driver-level halt exists on this stack" is a
        # fact about the safety layer, and it is the one a reader needs.
        if all("UNAVAILABLE" in d for d in done):
            self.get_logger().error(
                "[E-STOP REAL] NO DRIVER-LEVEL HALT WAS REACHABLE. The arms "
                "stop only because every commander subscribes to "
                "/estop_state; there is no second layer on this stack. On a "
                "SIM or MOCK run that is expected. On a real run it is not.")
        for d in done:
            self.get_logger().error("[E-STOP REAL] %s" % d)
        return done

    def srv_real_halt(self, req, resp):
        self.trip("/estop_real_halt service")
        done = self.halt_real_driver()
        resp.success = True
        resp.message = "; ".join(done)
        return resp

    # ---------------- action ----------------

    def trip(self, why):
        if self.stopped:
            return
        self.stopped = True
        self.reason = why
        # PARK FIRST, and publish the state, before anything that can fail or
        # be slow. Both are local publishes costing microseconds.
        #
        # This used to call halt_real_driver() first, on the reasoning that
        # the real arm matters more than the sim one. The reasoning was right
        # and the ordering was still wrong: that call blocked for 4 s on
        # missing services, and NOTHING was halted during those 4 s -- not the
        # real arm, not the sim arm, and /estop_state still read false, so the
        # followers had no idea either. Doing the cheap certain thing first
        # costs the real path nothing.
        self.halt_once()
        m = Bool(); m.data = True
        self.state_pub.publish(m)
        self.get_logger().error(
            "E-STOP TRIPPED: %s. Both arms halting and LATCHED until "
            "/estop_reset." % why)
        try:
            self.halt_real_driver()
        except Exception as e:
            self.get_logger().error("[E-STOP REAL] driver halt failed: %s" % e)

    def halt_once(self):
        """Park each arm at its measured position."""
        for a in self.arms:
            names = [f"{a}_joint_{i}" for i in range(1, 8)]
            if not all(n in self.js for n in names):
                continue
            t = JointTrajectory()
            t.joint_names = names
            p = JointTrajectoryPoint()
            p.positions = [self.js[n] for n in names]
            p.time_from_start = Duration(sec=0, nanosec=100_000_000)
            t.points = [p]
            self.traj_pub[a].publish(t)

    def tick(self):
        # RE-READ the parameter every cycle. It used to be latched at
        # construction, so `ros2 param set deadman_enabled true` - and
        # participant_safety_node's attempt to force it on for a participant
        # session - silently did nothing. Fault injection caught it: 10 s of
        # total master silence produced no response at all.
        self.deadman = bool(self.get_parameter("deadman_enabled").value)
        self.deadman_n = int(self.get_parameter("deadman_consecutive").value)
        if self.deadman and not self._deadman_prev:
            # Arming transition. Reset the counters and stamp the arming time,
            # so turning the dead-man on does not immediately trip on silence
            # that predates it -- which is precisely what made the injection
            # test unreadable: the harness enabled the dead-man with a blocking
            # service call, that call starved the pose topic for longer than
            # the deadline, and the trip that followed was the harness's own
            # setup rather than the fault under test.
            for a in self.arms:
                self.stale_count[a] = 0
            self.deadman_armed_t = self.get_clock().now()
            self.get_logger().warn(
                "dead-man ARMED: /master_arm_pose_* must stay fresher than "
                "%.2f s by SOURCE timestamp, %d consecutive checks to trip."
                % (self.stale, self.deadman_n))
        elif not self.deadman and self._deadman_prev:
            self.deadman_armed_t = None
            self.get_logger().warn("dead-man DISARMED.")
        self._deadman_prev = self.deadman
        ages = {}
        if self.deadman and not self.stopped:
            now = self.get_clock().now()
            for a in self.arms:
                t = self.last_pose[a]
                if t is None:
                    # An arm that has NEVER published used to `continue`, i.e.
                    # exempt itself from its own dead-man forever. On this rig
                    # only the left master is usually publishing, so the right
                    # arm's dead-man was permanently disarmed and nobody could
                    # see it. Absence is not freshness: age is measured from
                    # when the dead-man was armed, so an arm that never speaks
                    # trips on the same deadline as one that stops speaking.
                    if self.deadman_armed_t is None:
                        self.deadman_armed_t = now
                    t = self.deadman_armed_t
                elif (self.deadman_armed_t is not None
                        and self.deadman_armed_t > t):
                    # Age is measured from the later of "last fresh data" and
                    # "dead-man armed / e-stop reset". Arming or resetting
                    # starts a fresh deadline rather than inheriting staleness
                    # from before it, so the latch is always clearable; a
                    # genuinely dead master just trips again one deadline on.
                    t = self.deadman_armed_t
                age = (now - t).nanoseconds / 1e9
                ages[a] = age
                # Per-tick detail goes to /estop_deadman, NOT the log -- at
                # 10 Hz a log line per tick buries everything else. See
                # `ros2 topic echo /estop_deadman` for the live state.
                if age > self.stale:
                    self.stale_count[a] += 1
                    if self.stale_count[a] >= self.deadman_n:
                        rx = self.last_rx[a]
                        rx_age = (time.monotonic() - rx) if rx else float("nan")
                        self.trip(
                            "dead-man: /master_arm_pose_%s stale > %.2f s for "
                            "%d consecutive checks (source age %.2f s, last "
                            "message arrived %.2f s ago -- %s)"
                            % (a, self.stale, self.stale_count[a], age, rx_age,
                               "master SILENT" if rx_age > self.stale else
                               "master STILL PUBLISHING stale data"))
                        break
                else:
                    self.stale_count[a] = 0
        if self.stopped:
            self.halt_once()
        m = Bool(); m.data = self.stopped
        self.state_pub.publish(m)

        # OBSERVABILITY. Part 4b requires the e-stop to be VERIFIED at the
        # start of every participant session, and a dead-man whose state
        # cannot be read is one that cannot be verified -- it looks identical
        # armed, disarmed, and quietly exempting an arm that never publishes.
        # This publishes exactly what the decision is made on.
        s = String()
        s.data = json.dumps(dict(
            armed=bool(self.deadman), latched=bool(self.stopped),
            reason=self.reason,
            stale_timeout_s=self.stale, consecutive_required=self.deadman_n,
            deadline_s=round(self.stale + self.deadman_n * self.hold, 3),
            age_s={a: round(v, 3) for a, v in ages.items()},
            stale_count=dict(self.stale_count),
            unstamped_frames=dict(self.unstamped),
            arrival_age_s={a: (round(time.monotonic() - r, 3)
                               if r is not None else None)
                           for a, r in self.last_rx.items()},
            never_published=[a for a in self.arms if self.last_pose[a] is None],
            keyed_on="source timestamp (header.stamp)"))
        self.deadman_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    n = EStop()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
