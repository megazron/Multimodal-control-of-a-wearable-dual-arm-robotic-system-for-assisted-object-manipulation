#!/usr/bin/env python3
"""PART 3.2 -- DOES THE GRIPPER SURVIVE BEING KILLED MID-GRASP?

THE FAILURE THIS EXISTS TO CATCH. The Robotiq 2F-85 holds its position when
the commanding process goes away. So a node killed mid-grasp leaves the
fingers part-closed, and the next start finds them there with nothing in the
software able to say whether that is a grip or a leftover. Two opposite
mistakes follow, and they cannot both be avoided by a default:

    open unconditionally  -> drops whatever the hand was holding
    never open            -> the travel range is wrong for the whole session

THE HARNESS. Two long-lived processes stand in for the parts that OUTLIVE the
node under test, which is the entire point:

    fake hardware -- subscribes to the gripper controller topic, ramps toward
                     the commanded knuckle, STOPS EARLY at `--object-rad` when
                     an object is present (that is what "holding" physically
                     is: the object is the stop), and republishes the position
                     on /joint_states. It never restarts, so the position
                     genuinely persists across the kill.
    fake FSR      -- publishes /master_fsr_buttons at a scriptable raw value.

`fsr_gripper_node` is then started, killed with SIGKILL, and started again,
and the harness asserts on what the SECOND start does.

WHY THIS IS TRUSTWORTHY SYNTHETIC INPUT. Per the standing rule: the ground
truth here is CONSTRUCTED, not rendered. "The knuckle is at 0.42 rad and the
process was killed" is arithmetic and a signal, both exact. Nothing about the
real pads, the real fingers or the real object is being simulated -- only the
one physical property that drives the whole design, which is that the hardware
holds position when nobody is commanding it.

    python3 scripts/verify_gripper_shutdown.py
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import gripper_state as gs             # noqa: E402

ARM = "left"
KNUCKLE = "%s_robotiq_85_left_knuckle_joint" % ARM
REST_RAW = 50.0
# A DELIBERATE grasp: past the 1200 latch threshold, but NOT a full squeeze --
# this is what a normal grip looks like on the measured pads (rest 6-191,
# full 3603).
GRASP_RAW = 2500.0
# A FULL squeeze, at the measured full-scale count. Needed to reach the
# mechanical limit, and that matters: `free_air` can only be observed when the
# operator squeezes all the way. A PARTIAL squeeze on an empty hand parks the
# knuckle inside the `holding` band and is indistinguishable from a grip --
# a real limit of the three-state test, recorded rather than designed around.
FULL_RAW = 3700.0
OBJECT_RAD = 0.42          # a 40 mm block: the fingers stop here
# THE RIG'S MECHANICAL STOP, not the classification threshold. This was called
# MECH_LIMIT_RAD and set to 0.80 while gripper_state.MECH_LIMIT_RAD is 0.74 -- the
# same name for two different quantities, six hundredths apart, in the two
# files that decide whether a hand is holding something. Renamed to what it
# is, imported from the owner, and the ordering
# CMD_OPEN < OPEN < FREE_AIR <= MECH_LIMIT asserted at import time there.
MECH_LIMIT_RAD = gs.MECH_LIMIT_RAD


class Rig(Node):
    """Fake hardware + fake FSR + a reader for everything the node says."""

    def __init__(self, object_rad=OBJECT_RAD):
        super().__init__("gripper_verify_rig")
        self.pos = 0.0
        self.target = 0.0
        self.stop_at = object_rad
        self.raw = REST_RAW
        self.cmds = []                 # every knuckle setpoint ever received
        self.states = []               # every /gripper_reference payload
        self.js = self.create_publisher(JointState, "/joint_states", 10)
        self.fsr = self.create_publisher(
            Float64MultiArray, "/master_fsr_buttons", 10)
        self.create_subscription(
            JointTrajectory, "/%s_gripper_controller/joint_trajectory" % ARM,
            self.on_cmd, 10)
        self.create_subscription(String, "/gripper_reference",
                                 lambda m: self.states.append(json.loads(m.data)),
                                 10)
        self.create_timer(0.02, self.tick)

    def on_cmd(self, m):
        if m.points and m.points[0].positions:
            self.target = float(m.points[0].positions[0])
            self.cmds.append(self.target)

    def tick(self):
        # Ramp toward the target, but never past the object.
        step = 0.05
        want = min(self.target, self.stop_at) if self.target > self.pos \
            else self.target
        if abs(want - self.pos) <= step:
            self.pos = want
        else:
            self.pos += step if want > self.pos else -step
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        j.name = [KNUCKLE]
        j.position = [self.pos]
        self.js.publish(j)
        self.fsr.publish(Float64MultiArray(data=[self.raw, self.raw, 0.0, 0.0]))


def spin(rig, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(rig, timeout_sec=0.02)


def start_node():
    env = dict(os.environ)
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        [sys.executable, "-m", "srl_teleop.fsr_gripper_node"],
        cwd=os.path.join(ROOT, "src/srl_teleop"), env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)


def stop(proc, sig, rig=None):
    """Signal the node, and KEEP THE RIG SPINNING while it dies.

    The rig stands in for the controller, which in the real system is always
    running. A blocking `proc.wait()` with the rig idle would leave the
    shutdown's open setpoint sitting undelivered and score a working shutdown
    as a failure -- a harness that publishes nothing while it waits, which is
    the same bug that made four teensy_disconnect runs report NOT HANDLED.
    """
    if proc.poll() is not None:
        return
    os.killpg(os.getpgid(proc.pid), sig)
    end = time.monotonic() + 8.0
    while proc.poll() is None and time.monotonic() < end:
        if rig is not None:
            rclpy.spin_once(rig, timeout_sec=0.02)
        else:
            time.sleep(0.02)
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5)


RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, bool(ok), detail))
    print("  %-52s %s   %s" % (name, "PASS" if ok else "FAIL", detail))


def case_kill_while_holding(rig):
    print("\n-- CASE 1: SIGKILL while HOLDING an object")
    gs.clear_latched(ARM)
    rig.stop_at, rig.pos, rig.raw = OBJECT_RAD, 0.0, REST_RAW

    p = start_node()
    spin(rig, 5.0)
    check("first start opens and CONFIRMS the reference",
          any(s["startup"][ARM] == "open_confirmed" for s in rig.states),
          "states seen: %s" % sorted({s["startup"][ARM] for s in rig.states}))

    rig.raw = GRASP_RAW                       # squeeze: latch and close
    spin(rig, 3.0)
    check("latch marker written AT LATCH TIME, not at exit",
          gs.was_latched(ARM), gs.marker_path(ARM))
    check("hardware stopped on the object", abs(rig.pos - OBJECT_RAD) < 0.02,
          "knuckle %.3f rad (%s)" % (rig.pos, gs.classify(rig.pos)))

    os.killpg(os.getpgid(p.pid), signal.SIGKILL)   # <-- no exit code runs
    p.wait(timeout=5)
    rig.raw = REST_RAW                        # operator's hand is OFF the pad
    spin(rig, 1.0)
    check("marker SURVIVES SIGKILL", gs.was_latched(ARM),
          "age %.1f s" % (gs.marker_age_s(ARM) or -1))
    check("hardware still holds the object across the kill",
          abs(rig.pos - OBJECT_RAD) < 0.02, "knuckle %.3f rad" % rig.pos)

    rig.states, rig.cmds = [], []
    p2 = start_node()
    spin(rig, 6.0)
    # The node publishes its verdict every tick, so "pending" appears before
    # the first /joint_states arrives. That is correct, and asserting on the
    # SET of states seen was an instrument bug: what matters is where it
    # settled and that it never claimed to have opened.
    st = [s["startup"][ARM] for s in rig.states]
    check("SECOND start settles on HOLDING, never claims open",
          st and st[-1] == "holding" and "open_confirmed" not in st,
          "final %r, seen %s" % (st[-1] if st else None, sorted(set(st))))
    check("SECOND start commands NO open", 0.0 not in rig.cmds,
          "setpoints: %s" % (rig.cmds[:5] or "none"))
    check("OBJECT NOT DROPPED", abs(rig.pos - OBJECT_RAD) < 0.02,
          "knuckle %.3f rad after %ds with the pad at rest" % (rig.pos, 6))

    # The inherited grip must not be released by a resting pad -- but it must
    # still be releasable by someone who takes the pad back.
    rig.raw = GRASP_RAW
    spin(rig, 2.0)
    rig.raw = REST_RAW
    spin(rig, 4.0)
    check("after the operator re-takes the pad, release WORKS",
          rig.pos < gs.OPEN_RAD and not gs.was_latched(ARM),
          "knuckle %.3f rad, marker %s"
          % (rig.pos, "present" if gs.was_latched(ARM) else "cleared"))
    stop(p2, signal.SIGINT, rig)


def case_kill_on_nothing(rig):
    print("\n-- CASE 2: SIGKILL with the hand closed on NOTHING (stale marker)")
    gs.clear_latched(ARM)
    rig.stop_at, rig.pos, rig.raw = MECH_LIMIT_RAD, 0.0, REST_RAW
    p = start_node()
    spin(rig, 4.0)
    rig.raw = FULL_RAW
    spin(rig, 4.0)
    check("closed on nothing reads free_air",
          gs.classify(rig.pos) == gs.FREE_AIR,
          "knuckle %.3f rad -> %s" % (rig.pos, gs.classify(rig.pos)))
    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    p.wait(timeout=5)
    rig.raw = REST_RAW
    rig.states, rig.cmds = [], []
    p2 = start_node()
    spin(rig, 6.0)
    check("stale marker does NOT freeze an empty gripper",
          rig.pos < gs.OPEN_RAD, "knuckle %.3f rad" % rig.pos)
    check("stale marker cleared once the open is confirmed",
          not gs.was_latched(ARM),
          "marker %s, node said %s"
          % ("STILL PRESENT" if gs.was_latched(ARM) else "gone",
             sorted({s["startup"][ARM] for s in rig.states}) or "nothing"))
    stop(p2, signal.SIGINT, rig)


def case_clean_shutdown(rig):
    print("\n-- CASE 3: clean SIGINT / SIGTERM, NOT latched -> must open")
    gs.clear_latched(ARM)
    rig.stop_at, rig.pos, rig.raw = MECH_LIMIT_RAD, 0.0, REST_RAW
    p = start_node()
    spin(rig, 4.0)
    rig.raw = 900.0                    # past the 250 deadband, under the 1200 latch
    spin(rig, 2.0)
    part = rig.pos
    check("hand is part-closed and NOT latched", part > gs.OPEN_RAD,
          "knuckle %.3f rad" % part)
    rig.cmds = []
    stop(p, signal.SIGTERM, rig)       # SIGTERM, the one a launch teardown sends
    spin(rig, 2.0)
    check("SIGTERM shutdown opened the hand", rig.pos < gs.OPEN_RAD,
          "knuckle %.3f rad (was %.3f), setpoints after the signal: %s"
          % (rig.pos, part, rig.cmds))


def case_clean_shutdown_latched(rig):
    print("\n-- CASE 4: clean SIGINT while LATCHED -> must NOT drop the load")
    gs.clear_latched(ARM)
    rig.stop_at, rig.pos, rig.raw = OBJECT_RAD, 0.0, REST_RAW
    p = start_node()
    spin(rig, 4.0)
    rig.raw = GRASP_RAW
    spin(rig, 3.0)
    stop(p, signal.SIGINT, rig)
    spin(rig, 2.0)
    check("SIGINT while latched leaves the grip closed",
          abs(rig.pos - OBJECT_RAD) < 0.02, "knuckle %.3f rad" % rig.pos)
    check("marker kept, so the next start inherits the claim",
          gs.was_latched(ARM), "marker present")
    gs.clear_latched(ARM)


def main():
    rclpy.init()
    rig = Rig()
    print("=" * 78)
    print("GRIPPER SHUTDOWN / STARTUP REFERENCE -- kill it mid-grasp")
    print("=" * 78)
    print("object stop %.2f rad (%s)   free air %.2f rad (%s)"
          % (OBJECT_RAD, gs.classify(OBJECT_RAD),
             MECH_LIMIT_RAD, gs.classify(MECH_LIMIT_RAD)))
    try:
        case_kill_while_holding(rig)
        case_kill_on_nothing(rig)
        case_clean_shutdown(rig)
        case_clean_shutdown_latched(rig)
    finally:
        gs.clear_latched(ARM)
        rig.destroy_node()
        rclpy.shutdown()

    bad = [n for n, ok, _ in RESULTS if not ok]
    print("\n" + "=" * 78)
    print("%d checks, %d PASS, %d FAIL" % (len(RESULTS),
                                           len(RESULTS) - len(bad), len(bad)))
    if not RESULTS:
        print("NO CHECKS RAN -- this is a failure, not a pass.")
        return 2
    for n in bad:
        print("  FAILED: %s" % n)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
