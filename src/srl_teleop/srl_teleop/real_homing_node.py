#!/usr/bin/env python3
"""
real_homing_node.py — move a REAL arm to the home pose under a VELOCITY law.

WHY VELOCITY AND NOT A POSITION TRAJECTORY
------------------------------------------
A position trajectory carries a setpoint that lives in the future: if the arm
is not where the planner thought, the controller closes that gap as fast as
its limits allow. That is the reactivation-jump hazard, and it is the reason
the old implementation needed a blanket "refuse moves over 120 deg" guard --
the guard existed to catch a setpoint that was wrong by a lot.

This version instead commands a RATE:

    speed[j] = clip(kp * angle_diff(target[j], actual[j]), -vmax, +vmax)

Error magnitude cannot produce a fast move, because the clip is on speed, not
on error. As kp*err falls below vmax the joint decelerates smoothly into the
deadband. There is no position setpoint to jump to, so the 120 deg refusal is
not needed and is gone.

HOW THE RATE IS ACTUALLY DELIVERED
----------------------------------
The Gen3 7-DOF ros2_control macro exports EIGHT command interfaces, all of
them "position" -- there is NO velocity command interface (velocity is a
state interface only). Verified against
  kortex_description/arms/gen3/7dof/urdf/kortex.ros2_control.xacro
So a velocity controller has nothing to claim. The rate is delivered as a
position increment off the MEASURED position, recomputed every cycle:

    q_cmd[j] = q_actual[j] + speed[j] * dt

Because q_actual is re-read each cycle, the commanded position is never more
than vmax*dt (about 2.5 mrad at the defaults) ahead of where the arm really
is. That preserves the property that matters: the controller is never handed
a target it must race towards. If the arm stalls, the setpoint stalls with
it rather than running away.

SAFETY PROPERTIES KEPT FROM THE POSITION VERSION
  * SHORTEST PATH via kortex_convention.pose_delta_rad -- continuous joints
    1/3/5/7 may cross the +/-pi seam, limited joints 2/4/6 must not, because
    for them "wrapping" means driving through a hard stop.
  * COLLISION CHECKED against the wearer model every cycle, not just at the
    endpoints.
  * E-STOP ABORTS and leaves the arm exactly where it stopped.
  * EXPLICIT ARMING. Never homes on construction.

Usage:
  ros2 run srl_teleop real_homing_node --ros-args -p arm:=left
  ros2 service call /home_arm std_srvs/srv/Trigger {}
  ros2 service call /home_abort std_srvs/srv/Trigger {}
  # or, for the gated launch, -p auto_home:=true
"""
import math
import os
import sys
import threading
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.expanduser("~/kortex_ws/config"))
import home_positions                                        # noqa: E402

from srl_teleop.kortex_convention import (                   # noqa: E402
    pose_delta_rad, ros_list_to_kortex, wrap_rad_pi)
from srl_teleop.clearance import (                           # noqa: E402
    ClearanceModel, DISTAL_LINKS, REAL_ROBOT_PAD_M)

CONTINUOUS_IDX = (0, 2, 4, 6)


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class RealHoming(Node):
    def __init__(self):
        super().__init__("real_homing_node")
        self.declare_parameter("arm", "left")
        self.declare_parameter("real_ns", "/real")
        # --- the velocity law ---
        self.declare_parameter("kp", 0.5)
        # INTEGRAL. Proportional alone cannot close the last degree on real
        # hardware: at 0.2 deg, kp*err = 0.0017 rad/s, under any plausible
        # minimum commandable speed, so the joint is commanded a speed it
        # will not act on and the error is permanent. Shrinking the deadband
        # alone therefore just moves the stall lower. The integral RAMPS the
        # command until the joint actually moves, which is the correct answer
        # to a velocity floor, and it is what turns the residuals from a
        # boundary into a distribution.
        self.declare_parameter("ki", 0.4)
        # SETTLE. Exiting the instant `worst <= tol` means the SLOWEST joint
        # always stops at the tolerance edge -- the same boundary-parking the
        # deadband caused, one level up, and it put j5 at 2.85 deg of a
        # 2.86 deg tolerance while the other six reached 0.04-0.11 deg. Once
        # inside tolerance the loop keeps correcting until every joint is in
        # the deadband, with this as the backstop so it can never wait
        # forever for a joint the hardware cannot close.
        self.declare_parameter("settle_s", 10.0)
        self.declare_parameter("vmax_rad_s", 0.05)
        # 1.0 deg was NOT a convergence criterion, it was where the joints
        # STOPPED. Measured: six of seven parked at 0.980-0.993 deg -- the
        # latch below setting v=0 the instant a joint crossed the band, so
        # each one froze at whatever error it had on the crossing tick. Seven
        # near-identical residuals is the signature of a boundary, not of a
        # converged loop; a converged loop scatters.
        self.declare_parameter("deadband_deg", 0.15)
        self.declare_parameter("rate_hz", 20.0)
        # --- guards ---
        self.declare_parameter("min_clearance_m", 0.12)
        self.declare_parameter("home_tolerance_rad", 0.05)
        self.declare_parameter("timeout_s", 300.0)
        self.declare_parameter("stall_s", 15.0)
        # HYSTERESIS. A single deadband makes a joint chatter across its own
        # edge: it stops inside, drifts out, gets a kick, overshoots back in.
        # Enter the band at deadband_deg, leave it only past exit_deadband_deg.
        self.declare_parameter("exit_deadband_deg", 0.35)
        # A joint whose commanded speed is below the arm's minimum
        # commandable rate cannot close its error no matter how long it is
        # given. That is a DIFFERENT fault from a stall and needs a different
        # answer, so it is detected and named rather than timing out.
        self.declare_parameter("min_commandable_rad_s", 0.0)
        self.declare_parameter("oscillation_sign_changes", 6)
        self.declare_parameter("auto_home", False)

        self.arm = self.get_parameter("arm").value
        self.ns = self.get_parameter("real_ns").value.rstrip("/")
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.vmax = float(self.get_parameter("vmax_rad_s").value)
        self.deadband = math.radians(
            float(self.get_parameter("deadband_deg").value))
        self.rate = float(self.get_parameter("rate_hz").value)
        self.min_clear = float(self.get_parameter("min_clearance_m").value)
        self.tol = float(self.get_parameter("home_tolerance_rad").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.stall_s = float(self.get_parameter("stall_s").value)
        self.exit_deadband = math.radians(
            float(self.get_parameter("exit_deadband_deg").value))
        self.min_cmd_speed = float(
            self.get_parameter("min_commandable_rad_s").value)
        self.osc_changes = int(
            self.get_parameter("oscillation_sign_changes").value)

        self.names = [f"{self.arm}_joint_{i}" for i in range(1, 8)]
        self.target = list(home_positions.load_home_radians(self.arm))

        self.js = {}
        self.estopped = False
        self.aborted = False
        self.moving = False
        self.done_ok = False

        self.create_subscription(JointState, f"{self.ns}/joint_states",
                                 self.on_js, 20)
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: setattr(self, "estopped", m.data), 10)
        self.pub = self.create_publisher(
            JointTrajectory,
            f"{self.ns}/{self.arm}_arm_controller/joint_trajectory", 10)
        self.status_pub = self.create_publisher(
            Float64MultiArray, f"/real_status_{self.arm}", 10)
        self.done_pub = self.create_publisher(
            Bool, f"/homing_complete_{self.arm}", 10)
        # ACTIVE FLAG. The bridge auto-enables as soon as the real arm is
        # within home tolerance, which happens BEFORE homing has finished
        # correcting. Measured: homing reached tolerance at 21.2 s, the
        # bridge enabled at 22.2 s, and from then on TWO nodes published to
        # /real/<arm>_arm_controller/joint_trajectory -- the homing law
        # pulling to home and the bridge replaying sim angles. The errors
        # stopped converging and hunted at 3-12 deg. Two commanders on one
        # real arm is the fault; the bridge now refuses to enable while this
        # is true.
        self.active_pub = self.create_publisher(
            Bool, "/homing_active_%s" % self.arm,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.active_pub.publish(Bool(data=False))

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.clearance = ClearanceModel()

        # STATE THE EFFECTIVE PARAMETERS, ALWAYS.
        #
        # These were previously printed only by plan(), which main() skips
        # whenever auto_home is true -- i.e. always, under start_real.sh. So
        # the only velocity figure on screen came from the SCRIPT's banner,
        # which printed the BRIDGE's max_vel_rad_s while nothing forwarded
        # anything to this node. "vmax 0.15" in the header and "vmax now
        # 0.050" on every homing line disagreed for months because they were
        # two different parameters and only one of them was this node's.
        self.get_logger().info(
            "%s homing law: kp %.3f  ki %.3f  vmax %.3f rad/s  "
            "deadband %.2f deg (exit %.2f)  tolerance %.2f deg  "
            "settle %.1f s" %
            (self.arm, self.kp, self.ki, self.vmax,
             math.degrees(self.deadband), math.degrees(self.exit_deadband),
             math.degrees(self.tol), self.settle_s))

        self.create_service(Trigger, "/home_arm", self.srv_home)
        self.create_service(Trigger, "/home_abort", self.srv_abort)
        self.create_timer(0.2, self.publish_status)

        if bool(self.get_parameter("auto_home").value):
            threading.Thread(target=self._auto, daemon=True).start()

    def _auto(self):
        """Gated launch path: wait for feedback, then home without a service."""
        t0 = time.monotonic()
        while self.current() is None and time.monotonic() - t0 < 30.0:
            time.sleep(0.2)
        if self.current() is None:
            self.get_logger().error(
                "auto_home: no %s/joint_states after 30 s -- NOT homing."
                % self.ns)
            return
        time.sleep(1.0)
        self.start()

    # ---------------- state ----------------

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def current(self):
        if not all(n in self.js for n in self.names):
            return None
        return [self.js[n] for n in self.names]

    def error(self):
        """Per-joint shortest-path error, target - actual."""
        cur = self.current()
        if cur is None:
            return None, None
        return cur, pose_delta_rad(self.target, cur, CONTINUOUS_IDX)

    def publish_status(self):
        cur, d = self.error()
        if cur is None:
            return
        m = Float64MultiArray()
        at_home = 1.0 if max(abs(x) for x in d) <= self.tol else 0.0
        m.data = (list(cur) + ros_list_to_kortex(cur) + list(d)
                  + [max(abs(x) for x in d), at_home])
        self.status_pub.publish(m)

    def measure_clearance(self):
        pts = {}
        for part in ("torso", "head", "hips"):
            got = []
            for link in DISTAL_LINKS:
                try:
                    t = self.tf_buf.lookup_transform(
                        part, f"real_{self.arm}_{link}",
                        rclpy.time.Time()).transform.translation
                    got.append((t.x, t.y, t.z))
                except Exception:
                    continue
            if got:
                pts[part] = got
        if not pts:
            return float("inf"), None
        return self.clearance.clearance(pts, REAL_ROBOT_PAD_M)

    # ---------------- plan ----------------

    def plan(self):
        cur, d = self.error()
        if cur is None:
            print("  no %s/joint_states yet -- is the real stack up?" % self.ns)
            return None
        worst = max(range(7), key=lambda i: abs(d[i]))
        total = max(abs(x) for x in d)
        # Time under the velocity law: the joint runs at vmax while
        # kp*err > vmax, then decays exponentially inside that band.
        band = self.vmax / self.kp
        if total > band:
            secs = (total - band) / self.vmax + \
                math.log(band / self.deadband) / self.kp
        else:
            secs = math.log(max(total, self.deadband) / self.deadband) / self.kp

        print("\n" + "=" * 72)
        print("HOMING PLAN -- %s arm (%s)   VELOCITY LAW" %
              (self.arm.upper(), self.ns))
        print("=" * 72)
        print("  %-10s %10s %10s %10s %10s" %
              ("joint", "now(deg)", "kortex", "target", "err"))
        kx = ros_list_to_kortex(cur)
        for i in range(7):
            flag = "  <== worst" if i == worst else ""
            print("  joint_%-4d %10.2f %10.2f %10.2f %10.2f%s" %
                  (i + 1, math.degrees(cur[i]), kx[i],
                   math.degrees(self.target[i]), math.degrees(d[i]), flag))
        print("  largest error   : %.2f deg (joint_%d)"
              % (math.degrees(total), worst + 1))
        print("  kp %.2f   vmax %.3f rad/s   deadband %.2f deg"
              % (self.kp, self.vmax, math.degrees(self.deadband)))
        print("  peak speed      : %.4f rad/s (clipped from kp*err = %.4f)"
              % (min(self.vmax, self.kp * total), self.kp * total))
        print("  estimated time  : %.0f s" % secs)
        print("  NO position setpoint exists -- q_cmd tracks measured q + "
              "v*dt,")
        print("  so there is nothing for the controller to race towards.")
        print("=" * 72)
        return d

    # ---------------- execute ----------------

    def srv_home(self, req, resp):
        if self.moving:
            resp.success = False; resp.message = "already homing"; return resp
        ok = self.start()
        resp.success = ok
        resp.message = "homing started" if ok else "could not start -- see log"
        return resp

    def srv_abort(self, req, resp):
        self.aborted = True
        resp.success = True
        resp.message = "abort requested -- arm stops where it is"
        return resp

    def start(self):
        if self.plan() is None:
            return False
        self.aborted = False
        threading.Thread(target=self.execute, daemon=True).start()
        return True

    def execute(self):
        self.moving = True
        dt = 1.0 / self.rate
        t0 = time.monotonic()
        last_progress = time.monotonic()
        best = float("inf")
        tick = 0
        in_band = [False] * 7          # hysteresis latch, per joint
        sign_flips = [0] * 7           # oscillation evidence, per joint
        last_sign = [0] * 7
        prev_pos = None
        stuck_ticks = [0] * 7          # commanded but not moving
        integ = [0.0] * 7              # integral state, per joint
        tol_at = None                  # when tolerance was first met
        self.active_pub.publish(Bool(data=True))
        try:
            while not self.aborted:
                cur, d = self.error()
                if cur is None:
                    self.get_logger().error("lost joint_states -- stopping.")
                    break
                worst = max(abs(x) for x in d)

                # COMPLETE ON THE TOLERANCE, NOT ON THE DEADBAND.
                #
                # This used to exit only when every joint was inside the 1 deg
                # deadband, while the acceptance criterion is the 2.86 deg
                # home_tolerance_rad. A joint resting at 2.5 deg is therefore
                # HOMED by the project's own definition and yet could never
                # satisfy the exit test, so homing ran until it was declared
                # STALLED -- which is exactly what was observed on the left
                # arm: converged under 1 deg at t=66 s, then j6/j7 held
                # 2.4-2.7 deg and it hunted there until the stall timer fired.
                # The deadband's job is to suppress the CORRECTION, not to
                # decide when the arm is home.
                if worst <= self.tol:
                    if tol_at is None:
                        tol_at = time.monotonic()
                    settled = all(in_band)
                    if not settled and \
                            time.monotonic() - tol_at < self.settle_s:
                        pass          # inside tolerance; keep converging
                    else:
                        ok = True
                        self.done_ok = ok
                        self.get_logger().info(
                            "HOMING COMPLETE -- %s max error %.4f rad "
                            "(%.2f deg), WITHIN the %.3f rad (%.2f deg) "
                            "tolerance; %s."
                            % (self.arm, worst, math.degrees(worst), self.tol,
                               math.degrees(self.tol),
                               "every joint settled in the deadband" if settled
                               else "settle window expired with %d of 7 still "
                                    "correcting" % (7 - sum(in_band))))
                        self._report(cur, d)
                        self.done_pub.publish(Bool(data=bool(ok)))
                        break

                if self.estopped:
                    self.get_logger().error(
                        "E-STOP during homing -- stopping HERE. Arm left at "
                        "max error %.3f rad." % worst)
                    break

                if time.monotonic() - t0 > self.timeout:
                    self.get_logger().error(
                        "HOMING TIMEOUT after %.0f s at max error %.3f rad."
                        % (self.timeout, worst))
                    break

                # stall detection: the setpoint tracks measured position, so a
                # stalled arm would otherwise sit here forever making no noise.
                if worst < best - 1e-4:
                    best, last_progress = worst, time.monotonic()
                elif tol_at is not None:
                    # Inside tolerance and settling. `worst` stops improving
                    # here by design, so the stall timer would fire on a
                    # successful home. The settle window is its own bound.
                    pass
                elif time.monotonic() - last_progress > self.stall_s:
                    # "No progress" is three different faults wearing one
                    # name. Reporting them apart is the whole point: they
                    # have different answers and only one is a stall.
                    osc = [i for i in range(7) if sign_flips[i] >= self.osc_changes]
                    floor = [i for i in range(7)
                             if stuck_ticks[i] > self.rate * 2 and
                             abs(self.kp * d[i]) < max(self.min_cmd_speed, 1e-9)]
                    if not floor:
                        # No configured floor: infer it. A joint that is being
                        # commanded a non-zero speed and has not moved for
                        # seconds is below whatever the real floor is.
                        floor = [i for i in range(7)
                                 if stuck_ticks[i] > self.rate * 2
                                 and abs(d[i]) > self.deadband]
                    if osc:
                        self.get_logger().error(
                            "HOMING OSCILLATING (not stalled): joint(s) %s "
                            "changed error sign %s times while max error held "
                            "at %.2f deg. The arm IS moving; it is hunting "
                            "across the deadband edge. Raise "
                            "exit_deadband_deg (now %.2f) or lower kp (now "
                            "%.2f)."
                            % (", ".join("j%d" % (i + 1) for i in osc),
                               ", ".join(str(sign_flips[i]) for i in osc),
                               math.degrees(worst),
                               math.degrees(self.exit_deadband), self.kp))
                    elif floor:
                        self.get_logger().error(
                            "HOMING BELOW VELOCITY FLOOR (not stalled): "
                            "joint(s) %s are being commanded %s rad/s and are "
                            "NOT moving. kp*err at this error is under the "
                            "arm's minimum commandable speed, so the error "
                            "can never close. Raise kp, or accept the "
                            "residual -- it is inside the %.2f deg tolerance."
                            % (", ".join("j%d" % (i + 1) for i in floor),
                               ", ".join("%.4f" % abs(self.kp * d[i])
                                         for i in floor),
                               math.degrees(self.tol)))
                    else:
                        self.get_logger().error(
                            "HOMING STALLED: no progress for %.0f s at max "
                            "error %.3f rad (%.2f deg), and the arm is not "
                            "moving or oscillating. Stopping."
                            % (self.stall_s, worst, math.degrees(worst)))
                    self._report(cur, d)
                    break

                clear, part = self.measure_clearance()
                if clear < self.min_clear:
                    self.get_logger().error(
                        "HOMING HALTED: clearance %.3f m from %s is under the "
                        "%.2f m floor. Reposition by hand."
                        % (clear, part, self.min_clear))
                    break

                # ---- the velocity law ----
                cmd = []
                speeds = []
                for i in range(7):
                    # ANTI-WINDUP, and a reset on ZERO CROSSING. Carrying
                    # integral through a sign change would drive the joint
                    # past the target using error it has already corrected --
                    # overshoot manufactured by the fix.
                    if last_sign[i] and d[i] and \
                            (1 if d[i] > 0 else -1) != last_sign[i]:
                        integ[i] = 0.0
                    integ[i] += d[i] * dt
                    lim = self.vmax / max(self.ki, 1e-9)
                    integ[i] = clip(integ[i], -lim, lim)
                    v = clip(self.kp * d[i] + self.ki * integ[i],
                             -self.vmax, self.vmax)
                    # HYSTERESIS: enter the band at `deadband`, leave it only
                    # past `exit_deadband`. With one threshold the joint stops
                    # just inside, drifts just outside, is kicked, and
                    # overshoots back in -- the 2.2-3.0 deg hunting seen on
                    # the left arm.
                    if in_band[i]:
                        if abs(d[i]) > self.exit_deadband:
                            in_band[i] = False
                    elif abs(d[i]) <= self.deadband:
                        in_band[i] = True
                    if in_band[i]:
                        v = 0.0
                        integ[i] = 0.0
                    speeds.append(v)

                    # Oscillation evidence: sign changes of the ERROR while
                    # outside the band. A hunting joint flips repeatedly; a
                    # stalled one never does.
                    sgn = (1 if d[i] > 0 else (-1 if d[i] < 0 else 0))
                    if sgn and last_sign[i] and sgn != last_sign[i] \
                            and not in_band[i]:
                        sign_flips[i] += 1
                    if sgn:
                        last_sign[i] = sgn

                    # Velocity-floor evidence: commanded to move, not moving.
                    if prev_pos is not None and abs(v) > 1e-9:
                        if abs(cur[i] - prev_pos[i]) < 1e-5:
                            stuck_ticks[i] += 1
                        else:
                            stuck_ticks[i] = 0
                    q = cur[i] + v * dt
                    cmd.append(wrap_rad_pi(q) if i in CONTINUOUS_IDX else q)

                t = JointTrajectory()
                t.joint_names = self.names
                p = JointTrajectoryPoint()
                p.positions = cmd
                p.velocities = speeds
                p.time_from_start = Duration(
                    sec=int(dt), nanosec=int((dt - int(dt)) * 1e9))
                t.points = [p]
                self.pub.publish(t)

                prev_pos = list(cur)
                tick += 1
                if tick % max(1, int(self.rate)) == 0:
                    self._progress(d, speeds, time.monotonic() - t0, clear)
                time.sleep(dt)
        finally:
            self.moving = False
            # Every exit path -- complete, stalled, e-stop, timeout, abort.
            # A flag left raised would block the bridge forever, which is the
            # mirror of the fault it prevents.
            self.active_pub.publish(Bool(data=False))

    def _progress(self, d, speeds, elapsed, clear):
        cells = " ".join(
            "j%d %+7.2f" % (i + 1, math.degrees(d[i])) for i in range(7))
        print("  [%6.1fs] err deg: %s | vmax now %.3f | clear %.3f m"
              % (elapsed, cells, max(abs(v) for v in speeds), clear),
              flush=True)

    def _report(self, cur, d):
        print("\n  final per-joint error:")
        for i in range(7):
            ok = "ok" if abs(d[i]) <= self.tol else "OUT"
            print("    joint_%d  actual %8.2f deg   target %8.2f deg   "
                  "err %+7.3f deg  [%s]"
                  % (i + 1, math.degrees(cur[i]),
                     math.degrees(self.target[i]), math.degrees(d[i]), ok))
        # IS THIS CONVERGENCE, OR THE DEADBAND EDGE?
        #
        # A converged loop leaves residuals scattered by whatever actually
        # limits it -- encoder quantisation, the velocity floor, stiction --
        # and those differ per joint. Residuals that all sit at the SAME
        # magnitude, within a few percent of the deadband, are not a control
        # result: they are the band's own boundary, recorded once per joint.
        # This went unnoticed because every value was inside tolerance and
        # the run reported COMPLETE. It is stated numerically here so it
        # cannot pass silently again.
        mags = sorted(abs(math.degrees(x)) for x in d)
        db = math.degrees(self.deadband)
        spread = mags[-1] - mags[0]
        at_edge = [m for m in mags if db * 0.85 <= m <= db * 1.05]
        print("  residual spread %.3f deg (min %.3f, max %.3f), "
              "deadband %.3f deg" % (spread, mags[0], mags[-1], db))
        if len(at_edge) >= 5 and spread < 0.25 * db:
            print("  WARNING: %d of 7 joints are parked WITHIN 15%% of the "
                  "deadband and the spread is %.1f%% of it. That is the band "
                  "edge, not convergence -- the loop stopped correcting "
                  "rather than reaching the target."
                  % (len(at_edge), 100.0 * spread / db))
        else:
            print("  residuals are scattered (%.1f%% of the deadband), "
                  "consistent with genuine convergence."
                  % (100.0 * spread / max(db, 1e-9)))


def main(args=None):
    rclpy.init(args=args)
    n = RealHoming()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3.0)
    if not bool(n.get_parameter("auto_home").value):
        n.plan()
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
