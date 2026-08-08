#!/usr/bin/env python3
"""
fault_injector.py — deliberately break things, and check the system says so.

PART 8. Nothing in this hardening pass counts as done until the failure has
been INJECTED and the response verified. This is the harness that does it.

Every fault here is one that has actually happened on this rig, or one a
participant session would plausibly hit. Each has an EXPECTED RESPONSE, and
the harness checks for it rather than just producing the fault and leaving a
human to squint at a log.

    ros2 run srl_teleop fault_injector --list
    ros2 run srl_teleop fault_injector --fault channel_dropout
    ros2 run srl_teleop fault_injector --all --json results.json

Faults are injected over ROS - by publishing on the topics the real fault
would affect, or by parameter changes - so nothing has to be recompiled and
no test hook is left in production code paths.
"""
import argparse
import json
import threading
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from vision_msgs.msg import (BoundingBox3D, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)


class Harness(Node):
    def __init__(self, arm='left'):
        super().__init__('fault_injector')
        self.arm = arm
        self.blocking = {}
        self.summary = {}
        self.autonomy = ''
        self.intent = {}
        self.aborts = []
        self.estop = False
        self.unexplained = False
        self.capability = {}
        self.js = {}
        self.create_subscription(String, '/blocking', self._on_block, 20)
        self.deadman_state = {}
        self.create_subscription(
            String, '/estop_deadman',
            lambda m: self.deadman_state.update(json.loads(m.data)), 10)
        self.create_subscription(String, '/blocking_summary',
                                 lambda m: setattr(self, 'summary', json.loads(m.data)), 10)
        self.create_subscription(String, f'/autonomy/state_{arm}',
                                 lambda m: setattr(self, 'autonomy', m.data), 10)
        self.create_subscription(String, f'/autonomy/intent_debug_{arm}',
                                 lambda m: setattr(self, 'intent', json.loads(m.data)), 10)
        self.create_subscription(String, '/participant/abort',
                                 lambda m: self.aborts.append(json.loads(m.data)), 10)
        self.create_subscription(Bool, '/estop_state',
                                 lambda m: setattr(self, 'estop', bool(m.data)), 10)
        self.create_subscription(Bool, '/motion_unexplained',
                                 lambda m: setattr(self, 'unexplained', bool(m.data)), 10)
        self.create_subscription(String, f'/master_capability_{arm}',
                                 lambda m: setattr(self, 'capability', json.loads(m.data)), 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.raw = self.create_publisher(Float64MultiArray,
                                         f'/master_arm_raw_{arm}', 20)
        self.pose = self.create_publisher(PoseStamped, f'/master_arm_pose_{arm}', 20)
        # BOTH arms. The dead-man guards each arm independently, and an arm
        # that never publishes trips it -- correctly, since a permanently
        # silent master arm is a fault, not a default. A single-arm harness
        # therefore trips the OTHER arm's dead-man during its own setup and
        # reports it as the injected fault failing. The real master publishes
        # both, so the harness does too.
        self.pose_all = {a: (self.pose if a == arm else self.create_publisher(
            PoseStamped, f'/master_arm_pose_{a}', 20)) for a in ('left', 'right')}
        self.point = self.create_publisher(Vector3Stamped, f'/master_pointing_{arm}', 20)
        self.objects = self.create_publisher(Detection3DArray, '/perception/objects', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        # Recovery-path injection. Each publishes on the topic the REAL fault
        # would affect, so no test hook exists in the production nodes.
        self.session_pub = self.create_publisher(String, '/real/session_state', 10)
        self.link_pub = self.create_publisher(String, '/arm_link_status', 10)
        self.objects_info_pub = self.create_publisher(
            String, '/perception/objects_info', 10)
        self.recovery = {}
        self.create_subscription(
            String, '/recovery_state',
            lambda m: setattr(self, 'recovery', json.loads(m.data)), 10)
        self.ready_cli = self.create_client(Trigger, '/recovery/ready')
        # /estop_reset is a SERVICE (std_srvs/Trigger), not a topic. Publishing
        # a Bool to it silently does nothing, which is exactly what happened on
        # the first injection run: the e-stop latched and every subsequent
        # fault ran against an e-stopped arm.
        self.reset_cli = self.create_client(Trigger, '/estop_reset')

    def call_ready(self):
        """Ask recovery_manager whether the next trial may START."""
        if not self.ready_cli.wait_for_service(timeout_sec=5.0):
            return None
        f = self.ready_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        r = f.result()
        return (bool(r.success), r.message) if r else None

    def reset_estop(self):
        """Clear the latch through the SERVICE, and confirm it cleared."""
        if not self.reset_cli.wait_for_service(timeout_sec=5.0):
            return False
        fut = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.spin(1.0)
        return not self.estop

    def reset_state(self):
        """Return to a known-good state BETWEEN faults.

        Without this the tests are not independent: the first run latched the
        e-stop and every later fault silently ran against a stopped arm, which
        is how six of them came back NOT HANDLED for the same reason.
        """
        self.reset_estop()
        self.set_param('/channel_manager', 'auto_disable_after_frames', 0)
        for _ in range(30):
            self.send_raw([10, 20, 30, 40, 50, 60, 70])
            self.spin(0.02)
        self.send_objects([])
        self.aborts.clear()
        self.spin(1.5)

    def _on_block(self, m):
        d = json.loads(m.data)
        d['_rx'] = time.monotonic()
        self.blocking[d.get('unit', '?')] = d

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    # ------------------------------------------------------------ helpers
    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def master_on(self, p, stamp=None):
        """Run a 50 Hz master stream on a BACKGROUND thread until master_off().

        Every blocking call in this harness -- reset_estop, set_param, any
        service round trip -- publishes nothing while it waits, and several
        of them wait longer than the dead-man deadline. Driving the master
        from the foreground therefore injects a master outage as a side
        effect of the test setup, and the resulting trip gets attributed to
        whichever fault was nominally under test. The real master publishes
        independently of everything else; so does this.

        stamp=None keeps the data fresh; passing a fixed stamp reproduces a
        master that is PUBLISHING while its data is stale.
        """
        self.master_off()
        self._master_stop = threading.Event()

        def loop():
            while not self._master_stop.is_set() and rclpy.ok():
                try:
                    self.send_pose(p, stamp=stamp, all_arms=True)
                except Exception:                                  # noqa: BLE001
                    return
                time.sleep(0.02)
        self._master_thread = threading.Thread(target=loop, daemon=True)
        self._master_thread.start()

    def master_off(self):
        ev = getattr(self, '_master_stop', None)
        if ev is not None:
            ev.set()
        th = getattr(self, '_master_thread', None)
        if th is not None:
            th.join(timeout=1.0)
        self._master_thread = None

    def stream(self, p, dur):
        """Publish /master_arm_pose_* at 50 Hz for `dur` seconds, while spinning.

        Anything that arms or queries the e-stop goes through blocking service
        calls, and a plain spin() publishes NOTHING. Setting up a dead-man test
        with spin() therefore starves the master topic for longer than the
        dead-man deadline, and the trip that follows is caused by the setup
        rather than by the injected fault. Four consecutive attempts at
        `teensy_disconnect` failed for exactly that reason and were recorded as
        the system not handling the fault.
        """
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < dur and rclpy.ok():
            self.send_pose(p, all_arms=True)
            n += 1
            rclpy.spin_once(self, timeout_sec=0.015)
        return n

    def deadman_armed(self):
        d = getattr(self, 'deadman_state', None) or {}
        return bool(d.get('armed')) and not d.get('latched')

    def active_blocks(self):
        out = []
        now = time.monotonic()
        for u, d in self.blocking.items():
            if now - d['_rx'] > 5.0:
                continue
            for b in d.get('blockers', []):
                if b.get('active'):
                    out.append((u, b['name'], b.get('held_s', 0), b.get('reason', '')))
        return out

    def wait_for_block(self, name, timeout=8.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin(0.1)
            for u, n, held, r in self.active_blocks():
                if n == name:
                    return True, (u, n, held, r)
        return False, None

    def send_raw(self, joints_deg, accel=(0.0, 1.0, 0.0)):
        m = Float64MultiArray()
        m.data = [float(v) for v in joints_deg] + list(accel) + [0.0, 0.0, 0.0]
        self.raw.publish(m)

    def send_pose(self, p, q=(0.0, 0.0, 0.0, 1.0), stamp=None,
                  all_arms=False):
        m = PoseStamped()
        # An explicit stamp lets a fault reproduce a master that is PUBLISHING
        # while its data is stale -- the case an arrival-keyed dead-man misses.
        m.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = map(float, q)
        if all_arms:
            for pub in self.pose_all.values():
                pub.publish(m)
        else:
            self.pose.publish(m)

    def send_pointing(self, u):
        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.header.frame_id = 'world'
        u = np.asarray(u, float)
        u = u / max(1e-9, np.linalg.norm(u))
        v.vector.x, v.vector.y, v.vector.z = map(float, u)
        self.point.publish(v)

    def send_objects(self, items):
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for oid, pos in items:
            d = Detection3D()
            d.header = msg.header
            d.id = oid
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = oid
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
        self.objects.publish(msg)

    def set_param(self, node, name, value):
        """Parameter client, NOT the `ros2 param` CLI.

        The CLI goes through the ros2 daemon, which caches network state and
        HANGS on this box - it silently timed out on every call, so a fault
        that depended on a parameter change never actually fired and came back
        NOT HANDLED for a reason that had nothing to do with the system.
        """
        cli = self.create_client(SetParameters, node.rstrip('/') + '/set_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = name
        if isinstance(value, bool):
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=int(value))
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(value))
        req.parameters.append(p)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None

    def ee(self):
        """The robot's ACTUAL end effector, so injected geometry matches what
        intent_inference sees. It reads tf2, not the commanded pose."""
        import tf2_ros
        if not hasattr(self, '_tfb'):
            self._tfb = tf2_ros.Buffer()
            self._tfl = tf2_ros.TransformListener(self._tfb, self)
            self.spin(2.0)
        try:
            t = self._tfb.lookup_transform('world', '%s_end_effector_link' % self.arm,
                                           rclpy.time.Time())
            v = t.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:
            return np.array([0.0, 0.0, 1.0])


# ============================================================== the faults
def f_channel_dropout(h):
    """Right j4 drops out - the real, measured 12.9% burst fault.

    EXPECT: channel_manager reports the channel dead and names the capability
    lost; the trial must be abortable and marked INVALID, never continued."""
    h.set_param('/channel_manager', 'auto_disable_after_frames', 5)
    for _ in range(60):
        h.send_raw([10, 20, 30, 0.0, 50, 60, 70])   # j4 stuck at exactly 0.0
        h.spin(0.02)
    h.spin(2.0)
    cap = h.capability.get('channels', {})
    lost = h.capability.get('capability_lost', [])
    auto = h.capability.get('auto_disabled', [])
    ok = ('j4' in auto) or (cap.get('j4') is False)
    return ok, dict(auto_disabled=auto, j4_enabled=cap.get('j4'),
                    capability_lost=len(lost),
                    position_available=h.capability.get('position_available'))


def f_teensy_disconnect(h):
    """The Teensy stops publishing entirely (re-enumeration, unplug).

    Injected in BOTH forms, because they are not the same fault:

      * SILENT   -- the node exits and the topic stops (a clean unplug).
      * FROZEN   -- the master keeps publishing at 50 Hz while the DATA behind
                    it is stale. This is what a Teensy emitting garbage
                    actually produces: master_pose_node substitutes the last
                    good joint vector on a validation failure and carries on.
                    A dead-man keyed on message ARRIVAL cannot see it at all,
                    and this is the dangerous one -- the arm keeps tracking a
                    frozen command.

    EXPECT: the dead-man fires within its deadline in BOTH forms, and the
    follower reports a NAMED block rather than going quiet."""
    p = h.ee()
    # A live master runs on its own thread throughout, so nothing the setup
    # does can be mistaken for the fault.
    h.master_on(p)
    h.spin(1.5)
    h.set_param('/estop_node', 'deadman_enabled', True)
    h.reset_estop()
    h.spin(2.0)
    if not h.deadman_armed():
        h.master_off()
        return False, dict(setup='dead-man did not arm',
                           state=dict(h.deadman_state))
    deadline = float(h.deadman_state.get('deadline_s', 1.0)) + 0.5

    out = {}
    for form in ('frozen', 'silent'):
        # Master ON *before* the reset: reset_estop blocks for ~1 s and would
        # otherwise starve the topic past the deadline and re-latch instantly.
        h.master_on(p)                       # fresh again
        h.spin(0.5)
        h.reset_estop()
        h.spin(2.0)
        if h.estop:
            out[form] = dict(tripped=False, within_deadline=False,
                             error='e-stop latched before injection; '
                                   'setup starved the master topic',
                             state=dict(h.deadman_state))
            continue
        if form == 'frozen':
            # Keep PUBLISHING at 50 Hz, but freeze the source stamp. This is
            # what master_pose_node does when validation fails: it substitutes
            # the last good joint vector and carries on at full rate.
            h.master_on(p, stamp=h.get_clock().now().to_msg())
        else:
            h.master_off()                   # topic goes silent
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline + 2.0 and not h.estop:
            rclpy.spin_once(h, timeout_sec=0.02)
        lat = time.monotonic() - t0
        out[form] = dict(tripped=bool(h.estop), latency_s=round(lat, 2),
                         topic_still_publishing=(form == 'frozen'),
                         within_deadline=bool(h.estop) and lat <= deadline)
        h.master_off()
    got, info = h.wait_for_block('estop', timeout=6.0)
    h.master_off()
    h.set_param('/estop_node', 'deadman_enabled', False)
    ok = all(v['within_deadline'] for v in out.values()) and got
    return ok, dict(forms=out, follower_block=info, deadline_s=deadline)


def f_estop_during_motion(h):
    """E-stop pressed while the arm is moving.

    EXPECT: latches, the follower names `estop` as the blocker, and it CLEARS
    on /estop_reset - the historical bug was latching with no recovery."""
    e = h.ee()
    for i in range(40):
        h.send_pose(e + np.array([0.001 * i, 0.0, 0.0]))
        h.spin(0.02)
    b = Bool(); b.data = True
    h.estop_pub.publish(b)
    h.spin(1.5)
    for i in range(60):
        h.send_pose(e + np.array([0.001 * i, 0.0, 0.0]))
        h.spin(0.02)
    got, info = h.wait_for_block('estop', timeout=8.0)
    latched = h.estop
    h.reset_estop()
    h.spin(2.0)
    for i in range(60):
        h.send_pose(e)
        h.spin(0.02)
    h.spin(1.5)
    recovered = not any(n == 'estop' for _, n, _, _ in h.active_blocks())
    return (got and latched and recovered), dict(
        blocked=got, latched=latched, recovered_after_reset=recovered, detail=info)


def f_wrong_object_latch(h):
    """Operator changes target mid-reach.

    EXPECT: the estimate RELEASES the old goal. Latching is the failure."""
    e = h.ee()
    A = tuple(e + np.array([0.25, 0.30, -0.05]))
    B = tuple(e + np.array([-0.25, 0.30, -0.05]))
    for _ in range(120):
        h.send_objects([('tag_0', A), ('tag_1', B)])
        h.send_pose(tuple(e))
        h.send_pointing(np.array(A) - e)
        h.spin(0.02)
    first = h.intent.get('top')
    t0 = time.monotonic()
    switched = None
    for _ in range(200):
        h.send_objects([('tag_0', A), ('tag_1', B)])
        h.send_pose(tuple(e))
        h.send_pointing(np.array(B) - e)
        h.spin(0.02)
        if switched is None and h.intent.get('top') != first:
            switched = time.monotonic() - t0
    return (switched is not None), dict(
        first_target=first, now=h.intent.get('top'),
        switch_latency_s=round(switched, 3) if switched else None,
        peak_p=h.intent.get('top_p'))


def f_ambiguous_flicker(h):
    """Two objects exactly equidistant from the pointing ray.

    EXPECT: reported AMBIGUOUS, top_p near 1/N, and the arbiter stays in
    DIRECT. Flickering between them is the failure."""
    e = h.ee()
    A = tuple(e + np.array([0.20, 0.40, 0.0]))
    B = tuple(e + np.array([-0.20, 0.40, 0.0]))
    tops = []
    for _ in range(200):
        h.send_objects([('tag_0', A), ('tag_1', B)])
        h.send_pose(tuple(e))
        h.send_pointing((0.0, 1.0, 0.0))
        h.spin(0.02)
        if h.intent.get('top'):
            tops.append(h.intent['top'])
    flips = sum(1 for a, b in zip(tops, tops[1:]) if a != b)
    amb = bool(h.intent.get('ambiguous'))
    state = h.autonomy
    return (amb and state == 'DIRECT'), dict(
        ambiguous=amb, top_p=h.intent.get('top_p'), margin=h.intent.get('margin'),
        autonomy_state=state, top_flips=flips, samples=len(tops))


def f_stale_object(h):
    """An object is detected, then removed.

    EXPECT: the tracker DROPS it rather than remembering it, and the intent
    estimate reports no_objects rather than a stale distribution."""
    e = h.ee()
    A = tuple(e + np.array([0.25, 0.30, -0.05]))
    for _ in range(60):
        h.send_objects([('tag_0', A)])
        h.send_pose(tuple(e))
        h.send_pointing(np.array(A) - e)
        h.spin(0.02)
    had = h.intent.get('top')
    for _ in range(150):                    # object gone: publish an EMPTY list
        h.send_objects([])
        h.send_pose(tuple(e))
        h.send_pointing(np.array(A) - e)
        h.spin(0.02)
    gone = h.intent.get('state') == 'no_objects' or not h.intent.get('ids')
    return (had is not None and gone), dict(
        had=had, state_now=h.intent.get('state'), ids_now=h.intent.get('ids'),
        autonomy=h.autonomy)


def f_detection_failure(h):
    """Perception stops publishing entirely mid-reach (camera unplugged).

    EXPECT: the tracker's staleness drop empties the object list; the arbiter
    must YIELD to DIRECT rather than holding an assist toward something it can
    no longer see."""
    e = h.ee()
    A = tuple(e + np.array([0.05, 0.05, 0.0]))
    for _ in range(60):
        h.send_objects([('tag_0', A)])
        h.send_pose(tuple(e))
        h.send_pointing(np.array(A) - e)
        h.spin(0.02)
    before = h.autonomy
    for _ in range(150):                    # publish NOTHING at all
        h.send_pose(tuple(e))
        h.spin(0.02)
    return (h.autonomy == 'DIRECT'), dict(before=before, after=h.autonomy,
                                          ids=h.intent.get('ids'))


def f_logging_failure(h):
    """The trial logger cannot write (disk full, read-only path).

    EXPECT: it RAISES rather than silently dropping samples."""
    from srl_experiments.trial_logger import TrialLogger
    try:
        lg = TrialLogger('/proc/nonexistent-readonly', 'PILOT', 'fault')
        lg.start(0, 'x')
        return False, dict(raised=False,
                           note='silently accepted an unwritable path')
    except Exception as e:
        return True, dict(raised=True, error=type(e).__name__)


def f_identifying_data(h):
    """Someone puts a participant NAME in the manifest.

    EXPECT: refused in code, not in a README."""
    from srl_experiments.trial_logger import TrialLogger
    import tempfile
    lg = TrialLogger(tempfile.mkdtemp(), 'P01', 'fault')
    try:
        lg.write_manifest(name='Jane Doe')
        return False, dict(raised=False, note='accepted an identifying field')
    except ValueError as e:
        return True, dict(raised=True, message=str(e)[:80])


def f_nonmonotonic_clock(h):
    """time.time() steps backwards under WSL - it produced a -2321 ms latency
    once. EXPECT: no timing path in the shipped nodes uses it."""
    import pathlib
    bad = []
    roots = ['src/srl_teleop/srl_teleop', 'src/srl_autonomy/srl_autonomy',
             'src/srl_vr_teleop/srl_vr_teleop', 'src/srl_vr_autonomy/srl_vr_autonomy',
             'src/srl_experiments/srl_experiments', 'src/srl_perception/srl_perception']
    for r in roots:
        for f in pathlib.Path(r).rglob('*.py'):
            txt = f.read_text()
            for i, line in enumerate(txt.splitlines(), 1):
                if 'time.time()' not in line or line.strip().startswith('#'):
                    continue
                # A wall-clock TIMESTAMP is correct and stays. Only an
                # INTERVAL - a subtraction, a comparison, or a deadline - is
                # the bug, because that is what goes negative when the WSL
                # host clock steps backwards.
                import re as _re
                if _re.search(r'time\.time\(\)\s*[-<>]|[-<>]\s*time\.time\(\)'
                              r'|=\s*time\.time\(\)\s*\+', line):
                    bad.append('%s:%d' % (f, i))
    return (not bad), dict(offenders=bad)



# --------------------------------------------------------------------------
# PART 2 RECOVERY PATHS (b)-(e). Each asserts the same four things: the fault
# is DETECTED, motion is FROZEN, the trial is ABORTED with a cause recorded,
# and RECOVERY completes without relaunching the stack.
# --------------------------------------------------------------------------

def _recovery_state(h, timeout=6.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and rclpy.ok():
        rclpy.spin_once(h, timeout_sec=0.05)
        if h.recovery:
            return h.recovery
    return {}


def _await(h, pred, timeout=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and rclpy.ok():
        rclpy.spin_once(h, timeout_sec=0.05)
        if pred():
            return round(time.monotonic() - t0, 2)
    return None


def f_teensy_reconnect(h):
    """(b) Teensy disconnect: freeze, abort, mark invalid, reconnect for next.

    The board moves between /dev/ttyACM0 and ACM1 on re-enumeration, so
    recovery is re-DETECTION of the port, not reopening a remembered one.

    EXPECT: recovery_manager raises teensy_lost, freezes, publishes an abort
    with a cause, then reports ready_for_next_trial only once the master is
    publishing FRESH data again."""
    p = h.ee()
    # Freezing on master loss is OFF by default, for the same reason the
    # dead-man is: in sim the master goes stale constantly and freezing on it
    # makes a healthy stack look like dead hardware. A PARTICIPANT session
    # turns it on, and that is the configuration under test here.
    h.set_param('/recovery_manager', 'freeze_on_master_loss', True)
    h.master_on(p)
    h.spin(2.0)
    h.reset_estop()
    h.spin(1.0)
    h.aborts.clear()
    before = _recovery_state(h)

    h.master_off()                                # the board goes away
    raised = _await(h, lambda: 'teensy_lost' in
                    (h.recovery.get('faults') or {}), timeout=12.0)
    aborted = [a for a in h.aborts if a.get('fault') == 'teensy_lost']
    froze = _await(h, lambda: h.estop, timeout=5.0)

    # It comes back -- on a different device node, which is the point.
    h.master_on(p)
    recovered = _await(h, lambda: 'teensy_lost' not in
                       (h.recovery.get('faults') or {}), timeout=15.0)
    ready = bool(h.recovery.get('ready_for_next_trial'))
    h.reset_estop()
    h.master_off()
    h.set_param('/recovery_manager', 'freeze_on_master_loss', False)
    ok = bool(raised is not None and aborted and froze is not None
              and recovered is not None and ready)
    return ok, dict(detected_s=raised, froze_s=froze,
                    abort_cause=(aborted[0].get('cause') if aborted else None),
                    recovered_s=recovered, ready_for_next_trial=ready,
                    was_ready_before=before.get('ready_for_next_trial'))


def f_kortex_session_loss(h):
    """(c) Kortex session loss: freeze, abort, recover WITHOUT a relaunch.

    Injected by publishing /real/session_state with connected=false, which is
    what the bridge itself publishes when its session dies. The driver cannot
    be reactivated in-process (Router is not active), so recovery must be a
    FRESH session -- and the arm permits exactly one, so the old must be closed
    first.

    EXPECT: raised, frozen, aborted with a cause, and recovery_manager calls
    /real/session_recover rather than asking for a relaunch."""
    p = h.ee()
    h.master_on(p)
    h.spin(1.5)
    h.reset_estop()
    h.aborts.clear()

    m = String()
    m.data = json.dumps(dict(arm='left', connected=False,
                             error='router is not active'))
    for _ in range(10):
        h.session_pub.publish(m)
        h.spin(0.1)
    raised = _await(h, lambda: 'kortex_session_lost' in
                    (h.recovery.get('faults') or {}), timeout=10.0)
    froze = _await(h, lambda: h.estop, timeout=5.0)
    aborted = [a for a in h.aborts if a.get('fault') == 'kortex_session_lost']

    # The bridge reports a fresh session.
    m.data = json.dumps(dict(arm='left', connected=True, recoveries=1))
    t0 = time.monotonic()
    while (time.monotonic() - t0 < 8.0 and 'kortex_session_lost' in
           (h.recovery.get('faults') or {})):
        h.session_pub.publish(m)
        rclpy.spin_once(h, timeout_sec=0.05)
    recovered = 'kortex_session_lost' not in (h.recovery.get('faults') or {})
    h.reset_estop()
    h.master_off()
    ok = bool(raised is not None and froze is not None and aborted and recovered)
    return ok, dict(detected_s=raised, froze_s=froze,
                    abort_cause=(aborted[0].get('cause') if aborted else None),
                    recovered_without_relaunch=recovered)


def f_camera_zero_detections(h):
    """(d) Camera failure / zero detections: abort BEFORE the trial starts.

    EXPECT: /recovery/ready REFUSES, naming the scan failure, and it must
    refuse BEFORE the trial rather than aborting one in progress -- a
    truncated trial is indistinguishable from a slow participant."""
    h.master_on(h.ee())
    h.spin(1.0)
    h.reset_estop()

    info = String()
    info.data = json.dumps(dict(n=0))
    for _ in range(10):
        h.objects_info_pub.publish(info)
        h.spin(0.1)
    refused = h.call_ready()
    froze_wrongly = h.estop

    info.data = json.dumps(dict(n=3))
    for _ in range(10):
        h.objects_info_pub.publish(info)
        h.spin(0.1)
    allowed = h.call_ready()
    h.master_off()
    ok = bool(refused and not refused[0] and allowed and allowed[0]
              and not froze_wrongly)
    return ok, dict(refused_before_trial=(not refused[0]) if refused else None,
                    reason=refused[1] if refused else None,
                    allowed_after_fix=allowed[0] if allowed else None,
                    did_not_estop_mid_trial=not froze_wrongly)


def f_network_dropout(h):
    """(e) Network dropout to either arm: freeze BOTH, abort.

    Injected on /arm_link_status, which arm_link_monitor produces from real
    pings. Only the RIGHT link is dropped, to check that BOTH arms freeze --
    one arm live and one frozen on a wearer cannot be told apart by looking."""
    p = h.ee()
    h.master_on(p)
    h.spin(1.5)
    h.reset_estop()
    h.aborts.clear()

    down = String()
    down.data = json.dumps(dict(arms=dict(
        left=dict(ip='192.168.1.10', up=True),
        right=dict(ip='192.168.1.9', up=False, consecutive_failures=3))))
    for _ in range(10):
        h.link_pub.publish(down)
        h.spin(0.1)
    raised = _await(h, lambda: 'arm_link_down' in
                    (h.recovery.get('faults') or {}), timeout=10.0)
    froze = _await(h, lambda: h.estop, timeout=5.0)
    aborted = [a for a in h.aborts if a.get('fault') == 'arm_link_down']
    both = bool(aborted and aborted[0].get('arm') == 'both')

    up = String()
    up.data = json.dumps(dict(arms=dict(
        left=dict(ip='192.168.1.10', up=True),
        right=dict(ip='192.168.1.9', up=True))))
    t0 = time.monotonic()
    while (time.monotonic() - t0 < 8.0 and 'arm_link_down' in
           (h.recovery.get('faults') or {})):
        h.link_pub.publish(up)
        rclpy.spin_once(h, timeout_sec=0.05)
    recovered = 'arm_link_down' not in (h.recovery.get('faults') or {})
    h.reset_estop()
    h.master_off()
    ok = bool(raised is not None and froze is not None and aborted and both
              and recovered)
    return ok, dict(detected_s=raised, froze_s=froze, both_arms_frozen=both,
                    abort_cause=(aborted[0].get('cause') if aborted else None),
                    recovered=recovered)


FAULTS = {
    'channel_dropout': f_channel_dropout,
    'teensy_disconnect': f_teensy_disconnect,
    'estop_during_motion': f_estop_during_motion,
    'wrong_object_latch': f_wrong_object_latch,
    'ambiguous_flicker': f_ambiguous_flicker,
    'stale_object': f_stale_object,
    'detection_failure': f_detection_failure,
    'logging_failure': f_logging_failure,
    'identifying_data': f_identifying_data,
    'nonmonotonic_clock': f_nonmonotonic_clock,
    # PART 2 recovery paths
    'teensy_reconnect': f_teensy_reconnect,
    'kortex_session_loss': f_kortex_session_loss,
    'camera_zero_detections': f_camera_zero_detections,
    'network_dropout': f_network_dropout,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fault', default='')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--arm', default='left')
    ap.add_argument('--json', default='')
    a, _ = ap.parse_known_args()
    if a.list:
        for k in FAULTS:
            print(' ', k)
        return 0
    names = list(FAULTS) if a.all else ([a.fault] if a.fault in FAULTS else [])
    if not names:
        print('pick --fault <name> or --all; --list to see them')
        return 2
    rclpy.init()
    h = Harness(a.arm)
    h.spin(3.0)
    rows = []
    for n in names:
        print('\n=== INJECT %s ===' % n)
        h.reset_state()
        t0 = time.monotonic()
        try:
            ok, detail = FAULTS[n](h)
        except Exception as e:                                  # noqa: BLE001
            ok, detail = False, dict(exception='%s: %s' % (type(e).__name__, e))
        dt = time.monotonic() - t0
        rows.append(dict(fault=n, handled=bool(ok), seconds=round(dt, 1),
                         detail=detail))
        print('  %-22s %s   %s' % (n, 'HANDLED' if ok else '*** NOT HANDLED',
                                   json.dumps(detail)[:150]))
        h.spin(1.0)
    print('\n%-24s %s' % ('FAULT', 'RESULT'))
    for r in rows:
        print('%-24s %s' % (r['fault'], 'handled' if r['handled'] else 'NOT HANDLED'))
    n_ok = sum(1 for r in rows if r['handled'])
    print('\n%d/%d handled' % (n_ok, len(rows)))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(rows, f, indent=2)
    rclpy.shutdown()
    return 0 if n_ok == len(rows) else 1


if __name__ == '__main__':
    sys.exit(main())
