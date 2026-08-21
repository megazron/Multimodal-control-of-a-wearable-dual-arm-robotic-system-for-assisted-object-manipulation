#!/usr/bin/env python3
"""Move a real arm, or refuse. The primitives every calibration run stands on.

WHY THIS FILE EXISTS -- three defects from 2026-08-21, all in one session,
each of which put metal near a person:

  1. THE PATH WAS NEVER CHECKED, only the endpoints. A pick planned three
     waypoints, cleared every one against the wearer model, and commanded the
     arm straight between them. The bridge interpolates in JOINT space, so a
     safe start and a safe end say nothing about the sweep between: the elbow
     can traverse the wearer while both endpoints read 0.4 m clear. The
     project's own standing rule already said this -- "N repeats over the
     WHOLE PATH ... densified to 20 mm" -- and it was applied to reachability
     sweeps and not to the motion that actually ran.

  2. `goto()` PRINTED "reached" AFTER ITS TIMEOUT, whatever the error. A move
     that stalled 39.89 deg short was logged as success and the next waypoint
     was commanded from it. A mover that cannot fail is not a mover.

  3. NOTHING NOTICED THE ARM WAS GONE. The left arm dropped off the network
     three times; the commanding script kept publishing into a dead session
     and kept believing its own plan. Joint-state staleness is the signal and
     it was not being read.

So: every motion here is densified and checked BEFORE it is sent, every
arrival is verified against a tolerance and RAISES if it is not met, and the
joint stream is watched throughout. The refusals name what failed.

Nothing in this module smooths over a failure. It stops.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "config"))

import numpy as np

# Densify to this joint step. 0.05 rad at a 0.9 m radius is about 45 mm of
# hand travel, and the check is cheap, so there is no reason to go coarser.
MAX_JOINT_STEP_RAD = 0.05
# The wearer floor. The same 0.15 m the guard enforces; a caller may ask for
# MORE margin and may not ask for less.
FLOOR_M = 0.15
# A joint state older than this means the arm is not talking to us.
STALE_S = 1.0
# Arrival tolerance, set to what this controller CAN deliver rather than to
# what would be nice.
#
# The bridge is proportional with a 1.00 deg deadband, and `real_homing_node`
# records the reason it needs an integral term at all: below some error,
# kp*err falls under the arm's MINIMUM COMMANDABLE SPEED, the joint stops
# moving, and the residual can never close. SafeArm sends positions and has
# no integral term, so it inherits that floor.
#
# At 0.030 rad (1.72 deg) a return-to-home sat at 4.62 deg for 41.8 s and was
# correctly refused -- the mover was right and the demand was wrong. 0.05 rad
# is the project's own `home_tolerance_rad`, the number it already accepts as
# home. The ACHIEVED error is recorded on every waypoint regardless, so
# loosening the gate loses no information about where the arm actually went.
ARRIVE_RAD = 0.05


class MotionRefused(RuntimeError):
    """Raised with the reason, and never caught inside this module."""


class ArmVanished(RuntimeError):
    """The joint stream went stale mid-motion."""


# Joints 1, 3, 5 and 7 are CONTINUOUS on the Gen3. They have no stop, so
# +170 deg and -170 deg are 20 deg apart, not 340.
CONTINUOUS_IDX = (0, 2, 4, 6)


def wrap_pi(v):
    """Fold an angle into (-pi, pi]."""
    return (np.asarray(v, float) + np.pi) % (2 * np.pi) - np.pi


def delta(q0, q1, cont=CONTINUOUS_IDX):
    """q1 - q0, taking the SHORT way round on continuous joints.

    A raw subtraction across the +/-pi seam reports a 288 deg journey for a
    72 deg move, and everything downstream believes it: the path is densified
    over the long way, the clearance check validates a sweep the arm was
    never going to make, and the arm is commanded most of a full rotation.
    Observed on this rig -- "at start, 288.39 deg from home" for an arm that
    was 69.5 deg from home.
    """
    d = np.asarray(q1, float) - np.asarray(q0, float)
    d = np.array(d, dtype=float)
    for i in cont:
        d[i] = wrap_pi(d[i])
    return d


def max_error(q0, q1, cont=CONTINUOUS_IDX):
    """Worst per-joint distance, seam-aware. The number a tolerance uses."""
    return float(np.abs(delta(q0, q1, cont)).max())


def densify(q0, q1, step=MAX_JOINT_STEP_RAD):
    """Every intermediate configuration the arm will pass through.

    The bridge drives each joint toward the target independently, so the path
    in joint space is the straight line between the two vectors -- along the
    SHORT arc for a continuous joint. That is what is sampled here, not a
    Cartesian line, which the arm never follows.
    """
    q0 = np.asarray(q0, float)
    d = delta(q0, q1)
    n = int(math.ceil(float(np.abs(d).max()) / step)) + 1
    return np.array([q0 + d * t for t in np.linspace(0.0, 1.0, n)])


def check_path(scorer, arm, q0, q1, floor=FLOOR_M, step=MAX_JOINT_STEP_RAD):
    """(ok, reason, worst_clearance, n_samples) for the WHOLE sweep.

    Uses the MOVING chain. The whole-chain number is dominated by the
    immobile mount stub -- on this rig it reads a constant 0.2202 m for every
    pose ever tried, which is the mount cap and binds nothing.
    """
    P = densify(q0, q1, step)
    lo, hi, cont = scorer.lim[arm]
    # A CONTINUOUS JOINT HAS NO LIMIT TO VIOLATE. srl_fk.limits() hands back
    # (-pi, pi) for those as a SEARCH BOX and flags them as continuous -- it
    # says so in its own docstring. Enforcing that box as a stop rejects any
    # path that crosses the seam, which is a physically fine thing for a
    # jointless-stop axis to do: measured on the real right arm, this refused
    # "joint limit violated at step 10/46" and reported 0.000 m reached for
    # directions the arm walks into without complaint.
    hard = np.array([not c for c in cont], bool)
    worst = float("inf")
    worst_at = None
    for i, q in enumerate(P):
        if np.any(q[hard] < lo[hard] - 1e-6) or \
                np.any(q[hard] > hi[hard] + 1e-6):
            bad = int(np.argmax((q[hard] < lo[hard] - 1e-6)
                                | (q[hard] > hi[hard] + 1e-6)))
            names = [j for j in range(7) if hard[j]]
            return False, ("joint_%d past its limit at step %d/%d"
                           % (names[bad] + 1, i + 1, len(P))), worst, len(P)
        c = scorer.clearance_parts(arm, q)["moving_chain_m"]
        if not np.isfinite(c):
            return False, ("clearance is not finite at step %d/%d -- the "
                           "guard could not measure" % (i + 1, len(P))), \
                   worst, len(P)
        if c < worst:
            worst, worst_at = c, i
        if c < floor:
            return False, ("wearer clearance %.4f m at step %d/%d, under the "
                           "%.3f m floor" % (c, i + 1, len(P), floor)), \
                   worst, len(P)
    return True, ("clear: worst %.4f m at step %d/%d"
                  % (worst, (worst_at or 0) + 1, len(P))), worst, len(P)


class SafeArm:
    """One arm, over the running bridge. Construct inside an rclpy context."""

    def __init__(self, node, arm, scorer, floor=FLOOR_M):
        from trajectory_msgs.msg import JointTrajectory
        from sensor_msgs.msg import JointState
        self.node = node
        self.arm = arm
        self.sc = scorer
        self.floor = floor
        self.names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        self._q = {}
        self._stamp = 0.0
        self._hold = None
        self._stop = threading.Event()
        node.create_subscription(JointState, "/real/joint_states",
                                 self._cb, 50)
        self.pub = node.create_publisher(
            JointTrajectory,
            "/real/%s_arm_controller/joint_trajectory" % arm, 10)
        self.grip_pub = node.create_publisher(
            JointTrajectory,
            "/%s_gripper_controller/joint_trajectory" % arm, 10)
        # CALLBACKS MUST KEEP FLOWING WHILE THE CALLER COMPUTES.
        #
        # This used to pump callbacks from the main thread, inside spin().
        # But the caller blocks that same thread for seconds at a time solving
        # IK, and during a solve nothing spins, so no JointState arrives, so
        # the timestamp ages -- and the staleness check then reports the arm
        # has vanished. Measured: an abort claiming "no right joint state for
        # 2.4 s" while the bridge log had ZERO read failures and both arms
        # answered ping. The liveness monitor was detecting its own process
        # being busy, which is the worst kind of false alarm: it looks exactly
        # like the real fault it exists to catch.
        self._exec = None
        self._spinner = threading.Thread(target=self._spin_forever,
                                         daemon=True)
        self._spinner.start()
        self._feeder = threading.Thread(target=self._feed, daemon=True)
        self._feeder.start()

    # -- plumbing ---------------------------------------------------------
    def _cb(self, m):
        for n, p in zip(m.name, m.position):
            if n in self.names:
                self._q[n] = p
                self._stamp = time.time()

    def _feed(self):
        """Re-send the held target at 10 Hz.

        The bridge commands ZERO SPEED after 0.5 s without a target. A capture
        that pauses to take 25 camera frames is 2 s of silence, which stops the
        arm mid-move -- observed, and it is why an earlier run recorded a
        20.42 deg error it had no other explanation for.
        """
        while not self._stop.is_set():
            if self._hold is not None:
                self._send(self._hold)
            time.sleep(0.1)

    def _send(self, q):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        m = JointTrajectory()
        m.joint_names = list(self.names)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start = Duration(sec=2)
        m.points = [p]
        self.pub.publish(m)

    def _spin_forever(self):
        """The ONLY place this node is spun. See __init__."""
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        self._exec = ex
        while not self._stop.is_set():
            try:
                ex.spin_once(timeout_sec=0.05)
            except Exception:
                break

    def spin(self, t):
        """Wait. Callbacks are serviced by the spinner thread, not here."""
        time.sleep(t)

    # -- state ------------------------------------------------------------
    def q(self, wait_s=6.0):
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if all(n in self._q for n in self.names):
                return np.array([self._q[n] for n in self.names])
            self.spin(0.05)
        return None

    def age(self):
        return time.time() - self._stamp if self._stamp else float("inf")

    def await_fresh(self, timeout_s=3.0):
        """Block until a joint state arrives that is newer than right now.

        WHY A SEPARATE CALL. The staleness clock measures wall time since the
        last message, and this process spends seconds at a stretch inside a
        pure-Python optimiser that holds the GIL -- during which the spinner
        thread barely runs and no message is processed, however healthy the
        arm is. Asserting liveness immediately after a solve therefore fires
        on OUR OWN compute. Measured twice: "no right joint state for 2.5 s"
        with zero bridge errors, both arms answering ping, and joint states
        flowing at 17 Hz the moment the process stopped computing.

        So before a motion starts, wait for proof of life and reset the clock.
        A REAL silence still fails -- this returns False and the caller
        raises -- but a busy CPU no longer looks like a dead arm.
        """
        mark = time.time()
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._stamp > mark:
                return True
            time.sleep(0.02)
        return False

    def assert_live(self, where=""):
        """Raise only if the arm is REALLY silent, not if we were busy.

        A bare "age > STALE_S" test cannot tell a dead arm from a loaded
        process. Everything in this rig runs pure-Python numerics that hold
        the GIL for seconds -- an IK solve, and check_path evaluating the
        wearer model at 46 samples -- and during those the spinner thread
        barely gets scheduled, so the age grows with the arm in perfect
        health. That produced three separate false aborts, each reading
        "no right joint state for 2.4 s" while the bridge logged ZERO errors
        and joint states were flowing at 17 Hz the instant we stopped
        computing.

        So a stale-looking stamp is a QUESTION, not a verdict: stop computing
        and give the arm a fair chance to speak. If nothing arrives while we
        are doing nothing but waiting, it really is gone.
        """
        if self.age() <= STALE_S:
            return
        if self.await_fresh(timeout_s=max(2.0, STALE_S * 2)):
            return
        raise ArmVanished(
            "no %s joint state for %.1f s%s, and none arrived in a further "
            "%.1f s of doing nothing but waiting -- the arm is genuinely not "
            "publishing" % (self.arm, self.age(),
                            (" during " + where) if where else "",
                            max(2.0, STALE_S * 2)))

    # -- motion -----------------------------------------------------------
    def goto(self, target, label="move", timeout_s=60.0,
             tol=ARRIVE_RAD, extra_floor=0.0):
        """Check the whole path, then drive it, then VERIFY arrival.

        Raises MotionRefused if the path is unsafe, ArmVanished if the joint
        stream stops, and MotionRefused if the arm does not converge. It never
        returns having failed.
        """
        target = np.asarray(target, float)
        # Prove the arm is alive BEFORE the clock starts, so a long IK solve
        # in the caller cannot be mistaken for silence from the robot.
        if not self.await_fresh(timeout_s=3.0):
            raise ArmVanished(
                "no fresh %s joint state in 3.0 s before %s -- the arm is "
                "genuinely not publishing (this wait is immune to our own "
                "CPU load)" % (self.arm, label))
        start = self.q()
        if start is None:
            raise ArmVanished("no %s joint state before %s"
                              % (self.arm, label))
        floor = self.floor + extra_floor
        ok, why, worst, n = check_path(self.sc, self.arm, start, target,
                                       floor=floor)
        if not ok:
            raise MotionRefused("%s REFUSED: %s (%d samples checked)"
                                % (label, why, n))
        self._hold = target
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout_s:
            self.spin(0.15)
            self.assert_live(label)
            c = self.q(wait_s=0.5)
            if c is None:
                raise ArmVanished("lost %s joint state during %s"
                                  % (self.arm, label))
            last = max_error(c, target)
            if last < tol:
                break
        self.spin(1.2)
        c = self.q(wait_s=1.0)
        if c is None:
            raise ArmVanished("lost %s joint state settling %s"
                              % (self.arm, label))
        err = max_error(c, target)
        if err >= tol:
            raise MotionRefused(
                "%s DID NOT ARRIVE: worst joint %.4f rad (%.2f deg) after "
                "%.1f s, tolerance %.2f deg. NOT treating this as reached."
                % (label, err, math.degrees(err), time.time() - t0,
                   math.degrees(tol)))
        return c, dict(label=label, worst_clearance_m=worst,
                       path_samples=n, arrive_err_rad=err,
                       seconds=time.time() - t0)

    def hold_here(self):
        c = self.q()
        if c is None:
            raise ArmVanished("no %s joint state to hold" % self.arm)
        self._hold = c
        return c

    def grip(self, rad, label="grip", settle_s=2.5):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        m = JointTrajectory()
        m.joint_names = ["%s_robotiq_85_left_knuckle_joint" % self.arm]
        p = JointTrajectoryPoint()
        p.positions = [float(rad)]
        p.time_from_start = Duration(sec=1)
        m.points = [p]
        for _ in range(12):
            self.grip_pub.publish(m)
            self.spin(0.08)
        self.spin(settle_s)
        return label

    def close(self):
        self._stop.set()
        try:
            self._spinner.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._feeder.join(timeout=1.0)
        except Exception:
            pass


def self_test(verbose=True):
    """Known-answer checks. No ROS, no robot, no arm required.

    A path checker that cannot REFUSE is not a checker, so the third case
    deliberately drives the arm through the wearer and must be caught.
    """
    import solve_home_pose as SHP
    import home_positions as hp
    sc = SHP.Scorer()
    qh = np.array(hp.load_home_radians("left"))

    # SEAM: a continuous joint 20 deg the other side of +/-pi must be a
    # 20 deg move, not a 340 deg one. This is the defect that reported
    # "288.39 deg from home" for an arm 69.5 deg from home.
    a = np.zeros(7)
    a[0] = math.radians(170.0)
    b = np.zeros(7)
    b[0] = math.radians(-170.0)
    raw = float(np.abs(b - a).max())
    wrapped = max_error(a, b)
    assert abs(math.degrees(raw) - 340.0) < 1e-6
    assert abs(math.degrees(wrapped) - 20.0) < 1e-6, \
        "continuous joint seam not handled: %.2f deg" % math.degrees(wrapped)
    dpath = densify(a, b, step=0.05)
    span = float(np.abs(delta(dpath[0], dpath[-1])).max())
    assert abs(math.degrees(span) - 20.0) < 1e-6
    assert len(dpath) <= 10, \
        "densify took the long way: %d samples for a 20 deg move" % len(dpath)
    if verbose:
        print("seam: raw %.0f deg -> wrapped %.0f deg, %d path samples  OK"
              % (math.degrees(raw), math.degrees(wrapped), len(dpath)))

    n = len(densify(qh, qh + 0.2, step=0.05))
    assert n >= 5, "densify returned %d samples for a 0.2 rad move" % n
    d = densify(qh, qh + 0.2, step=0.05)
    assert np.allclose(d[0], qh) and np.allclose(d[-1], qh + 0.2), \
        "densify must include both endpoints"
    step = float(np.abs(np.diff(d, axis=0)).max())
    assert step <= 0.05 + 1e-9, "densify step %.4f exceeds the limit" % step
    if verbose:
        print("densify: %d samples, max step %.4f rad  OK" % (n, step))

    ok, why, worst, ns = check_path(sc, "left", qh, qh, floor=FLOOR_M)
    assert ok, "a zero-length move at home was refused: %s" % why
    if verbose:
        print("stationary at home: %s  OK" % why)

    # KNOWN BAD, FOUND RATHER THAN ASSUMED.
    #
    # An earlier version of this test hard-coded j2=120 j4=60 because that
    # configuration measured 0.0924 m -- but that reading was taken from the
    # arm's DROOPED pose, and the same joint values reached from HOME stay
    # 0.1954 m clear. A "known bad" input that is not actually bad tests
    # nothing and passes silently, which is the exact class of defect this
    # file is here to prevent. So the breach is searched for, and the search
    # failing is itself a failure.
    qbad, cbad = None, None
    for j2 in np.radians(np.arange(-120, 121, 15.0)):
        for j4 in np.radians(np.arange(-140, 141, 15.0)):
            for j6 in np.radians(np.arange(-110, 111, 25.0)):
                t = qh.copy()
                t[1], t[3], t[5] = j2, j4, j6
                c = sc.clearance_parts("left", t)["moving_chain_m"]
                if cbad is None or c < cbad:
                    qbad, cbad = t, c
    assert qbad is not None and cbad < FLOOR_M, (
        "no configuration in the search breaches the %.2f m floor (best "
        "%.4f m) -- cannot prove the checker can refuse anything"
        % (FLOOR_M, cbad if cbad is not None else float("nan")))
    if verbose:
        print("found a breaching pose: moving-chain %.4f m (floor %.2f)"
              % (cbad, FLOOR_M))
    ok, why, worst, ns = check_path(sc, "left", qh, qbad, floor=FLOOR_M)
    assert not ok, ("the path checker PASSED a sweep into the wearer "
                    "(worst %.4f m over %d samples) -- it binds nothing"
                    % (worst, ns))
    if verbose:
        print("sweep into the wearer: REFUSED -- %s  OK" % why)

    # THE ENDPOINTS-ONLY MISTAKE, demonstrated. Both ends clear the floor and
    # the middle does not -- the precise case that endpoint checking misses.
    found = None
    for _ in range(6000):
        a = qh + np.random.default_rng(_).uniform(-1, 1, 7) * 1.4
        lo_, hi_, _c = sc.lim["left"]
        a = np.clip(a, lo_, hi_)
        ca = sc.clearance_parts("left", a)["moving_chain_m"]
        if ca < FLOOR_M:
            continue
        okp, whyp, wp, npp = check_path(sc, "left", qh, a, floor=FLOOR_M)
        if not okp:
            found = (ca, wp, npp)
            break
    assert found is not None, (
        "could not construct a move whose ENDS are both clear and whose PATH "
        "is not -- the endpoint-only failure mode is unproven here")
    if verbose:
        print("both endpoints clear (home %.3f, target %.3f) but the PATH "
              "breaches -> refused over %d samples  OK"
              % (sc.clearance_parts("left", qh)["moving_chain_m"],
                 found[0], found[2]))

    if verbose:
        print("safe_motion self-test PASSED")
    return True


if __name__ == "__main__":
    self_test(verbose=True)
