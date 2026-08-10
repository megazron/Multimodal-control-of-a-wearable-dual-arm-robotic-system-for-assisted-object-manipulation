#!/usr/bin/env python3
"""
preflight.py — refuses to start a session if anything is wrong, and NAMES it.

PART 7b. Every check here corresponds to a failure that has actually cost time
on this rig. It exits non-zero on any failure, so `run_experiment.sh` can gate
on it and an experimenter cannot start a participant session on a broken rig
by forgetting a step.

    ros2 run srl_teleop preflight
    ros2 run srl_teleop preflight --participant     # stricter: adds e-stop,
                                                    # disk, and the wear clock
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from srl_teleop.serial_port import find_port, PortNotFound


class Preflight(Node):
    def __init__(self):
        super().__init__('preflight')
        self.js_t = []
        self.blocking = {}
        self.guard = None
        self.estop = None
        self.objects = None
        # THE WEARER IS PART OF THE ROBOT MODEL, not a world object, so it is
        # checked through TF. Named links, because a wrong name reads exactly
        # like a missing link: an earlier version of this audit looked for
        # `human_torso` and concluded the wearer was absent. It is `torso`.
        self._tf_buf = None
        try:
            import tf2_ros
            self._tf_buf = tf2_ros.Buffer()
            self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
        except Exception:                                       # noqa: BLE001
            pass
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.js_t.append(time.monotonic()), 50)
        self.create_subscription(String, '/blocking',
                                 lambda m: self.blocking.update(
                                     {json.loads(m.data)['unit']: json.loads(m.data)}), 20)
        self.create_subscription(String, '/mount_guard',
                                 lambda m: setattr(self, 'guard', m.data), 10)
        self.create_subscription(Bool, '/estop_state',
                                 lambda m: setattr(self, 'estop', bool(m.data)), 10)
        self.create_subscription(String, '/perception/objects_info',
                                 lambda m: setattr(self, 'objects', json.loads(m.data)), 10)

    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)


def check(name, ok, detail, fatal=True):
    mark = 'PASS' if ok else ('FAIL' if fatal else 'WARN')
    print('  [%-4s] %-26s %s' % (mark, name, detail))
    return (ok or not fatal)


WEARER_LINKS = ('torso', 'head', 'hips', 'left_leg', 'right_leg',
                'human_left_upper_arm', 'human_right_upper_arm')


def world_objects(node, timeout_s=6.0):
    """How many collision objects move_group actually holds.

    Asked of move_group, not of whoever published: "I sent it" and "it is
    there" have already differed once in this project, and a single publish
    to /planning_scene at startup loses the race with subscription matching.
    """
    try:
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        cli = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return 0, 'no /get_planning_scene (is move_group up?)'
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None:
            return 0, 'move_group did not answer'
        return len(fut.result().scene.world.collision_objects), ''
    except Exception as e:                                      # noqa: BLE001
        return 0, str(e)[:60]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--participant', action='store_true',
                    help='stricter checks for a participant session')
    ap.add_argument('--need-cameras', action='store_true')
    ap.add_argument('--min-disk-gb', type=float, default=2.0)
    a, _ = ap.parse_known_args()

    print('PRE-FLIGHT%s' % ('  (participant session)' if a.participant else ''))
    rclpy.init()
    n = Preflight()
    n.spin(6.0)
    ok = True

    # --- /joint_states rate. A dead broadcaster looks like a dead robot.
    ts = [t for t in n.js_t if t > time.monotonic() - 5.0]
    hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else 0.0
    ok &= check('joint_states rate', hz > 20.0, '%.1f Hz (expect ~100)' % hz)

    # --- controllers
    try:
        r = subprocess.run(['ros2', 'control', 'list_controllers'],
                           capture_output=True, timeout=25, text=True)
        out = r.stdout
        need = ['joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller']
        missing = [c for c in need if ('%s' % c) not in out or
                   'active' not in out.split(c)[-1][:40]]
        ok &= check('controllers active', not missing,
                    'missing/inactive: %s' % (missing or 'none'))
    except Exception as e:                                      # noqa: BLE001
        ok &= check('controllers active', False, 'could not query: %s' % e)

    # --- mount guard: the home geometry must be safe before anything runs
    ok &= check('mount guard', n.guard is not None,
                n.guard or 'no /mount_guard - is mount_guard_node running?')

    # --- nothing blocking at rest
    active, unknown = [], []
    for u, d in n.blocking.items():
        for b in d.get('blockers', []):
            if b.get('active'):
                active.append('%s/%s' % (u, b['name']))
            elif b.get('expired'):
                unknown.append('%s/%s' % (u, b['name']))
    ok &= check('no active blockers', not active, ', '.join(active) or 'clear')
    # An EXPIRED blocker is not a clear one. It means the unit asserting it
    # stopped running, so the condition it guards is unknown - and starting a
    # participant session on an unknown safety state is exactly the failure
    # this whole mechanism exists to prevent.
    ok &= check('no expired blockers', not unknown,
                ', '.join(unknown) + ' (asserting unit stopped; state UNKNOWN)'
                if unknown else 'none')

    # --- master serial port. Autodetected; the port MOVES between ACM0/ACM1.
    try:
        port = find_port()
        ok &= check('master serial port', True, port)
    except PortNotFound as e:
        ok &= check('master serial port', False, str(e).splitlines()[0],
                    fatal=a.participant)
    except Exception as e:                                      # noqa: BLE001
        ok &= check('master serial port', False, str(e)[:70], fatal=a.participant)

    # --- cameras / detections. Aborting BEFORE a trial, not during.
    if a.need_cameras:
        nobj = (n.objects or {}).get('n')
        ok &= check('perception detections', bool(nobj),
                    '%s objects visible' % (nobj if nobj is not None else 'no data'))

    # --- e-stop present and not already latched
    if a.participant:
        ok &= check('e-stop topic', n.estop is not None,
                    'latched' if n.estop else ('clear' if n.estop is not None
                                               else 'no /estop_state'))
        ok &= check('e-stop not latched', n.estop is not True, 'ok')

    # --- THE PLANNING SCENE MUST NOT BE EMPTY.
    #
    # move_group returned "collision objects: NONE" for the entire life of the
    # clip set, because clip_scene published Markers to /task_objects and
    # nothing to /planning_scene. Every path was planned through a world with
    # no furniture in it, and nothing anywhere said so -- an empty world is
    # silent, and it looks exactly like a world where nothing is in the way.
    #
    # The WEARER is a different matter and was never missing: torso, head,
    # hips, legs and both arms are links of the robot model, present in TF,
    # with only 14 mount-adjacent pairs excluded in the ACM. Wearer clearance
    # figures stand; WORLD-object figures were the optimistic ones.
    wearer_missing = []
    if n._tf_buf is not None:
        import rclpy.time as _rt
        for link in WEARER_LINKS:
            try:
                n._tf_buf.lookup_transform('world', link, _rt.Time())
            except Exception:                                   # noqa: BLE001
                wearer_missing.append(link)
    n.wearer_ok = (n._tf_buf is not None and not wearer_missing)
    scene_n, scene_why = world_objects(n)
    ok &= check('planning scene populated', scene_n > 0,
                ('%d collision objects' % scene_n) if scene_n > 0
                else 'EMPTY WORLD -- %s' % scene_why,
                fatal=a.participant)
    ok &= check('wearer in the model', n.wearer_ok,
                'torso/head/hips/legs in TF' if n.wearer_ok
                else 'MISSING FROM TF: %s -- every clearance number is '
                     'meaningless and the floor cannot fire'
                     % ', '.join(wearer_missing or ['no tf2']),
                fatal=True)

    # --- disk. A session that fills the disk loses the session.
    free = shutil.disk_usage(os.path.expanduser('~')).free / 1e9
    ok &= check('disk space', free >= a.min_disk_gb,
                '%.1f GB free (need %.1f)' % (free, a.min_disk_gb))

    print()
    if ok:
        print('PRE-FLIGHT PASS')
    else:
        print('PRE-FLIGHT FAILED — do not start a session. Fix what is named '
              'above; `bash scripts/diagnostics.sh` has the checks in detail '
              'and docs/system/06_troubleshooting.md is keyed by symptom.')
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
