#!/usr/bin/env python3
"""
vr_guided_session.py -- run the headset measurements WITHOUT reading a terminal.

The instruction for each step is rendered on the in-headset panel; the operator
does the action and presses **A** on the right controller to end that step. The
recording for that step is then closed, analysed, and the next instruction
appears. Nothing has to be typed, and no one has to take the headset off.

WHY IT IS DRIVEN FROM IN-HEADSET. Every earlier attempt at these measurements
missed the action, because the recorder was started AFTER the operator was told
to move -- so the window and the event never lined up, and three separate steps
came back "0 commands" on a system that was working perfectly. The operator is
the only one who knows when the action started and finished, so the operator
ends the step.

HOW THE INSTRUCTION REACHES THE PANEL. quest_bridge_node keeps a `robot_state`
dict that it UPDATES (not replaces) from every message on
/vr/robot_state_json, and pushes to the client at 20 Hz. vr_feedback_node owns
that topic but never writes an `instruction` key, so publishing one here merges
in alongside it and survives. No change to either node.

    python3 scripts/vr_guided_session.py
    python3 scripts/vr_guided_session.py --only axis_right axis_forward
    python3 scripts/vr_guided_session.py --selftest      # no headset, no ROS

The A button is buttons[0] of /vr/controller_joy_<hand> (the bridge publishes
[a, b]). Detected on the RISING edge with a hold-off, so one press ends one
step rather than skipping the next as well.
"""
import argparse
import json
import os
import sys
import threading
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, 'scripts'))
from vr_headset_check import (                                   # noqa: E402
    freeze_latency_s, gripper_follows_trigger, rate_hz,
    reengage_jump_m, scaling_continuity_mm, scaling_response)

# --------------------------------------------------------------------- steps
STEPS = [
    dict(key='m1', label='1 - RATE',
         text='Move BOTH controllers around continuously, at a natural '
              'teleoperation speed. Keep going for about 15 seconds.'),
    dict(key='axis_right', label='2 - RIGHT',
         text='Hold GRIP. Move your hand about 30 cm to your OWN RIGHT and '
              'nothing else. Hold it there, then release GRIP.'),
    dict(key='axis_forward', label='3 - FORWARD',
         text='Hold GRIP. Move your hand about 30 cm straight FORWARD, away '
              'from your chest. Hold, then release GRIP.'),
    dict(key='axis_up', label='4 - UP',
         text='Hold GRIP. Move your hand about 30 cm straight UP. Hold, then '
              'release GRIP.'),
    dict(key='m2', label='5 - FREEZE',
         text='Hold GRIP and drive for 3 seconds. Then, STILL HOLDING GRIP, '
              'cover the controller completely with your other hand for 5 '
              'seconds. Uncover, release GRIP.'),
    dict(key='m3', label='6 - CLUTCH',
         text='Three times: hold GRIP, move about 30 cm, RELEASE grip, move '
              'your hand somewhere else, hold still, then GRIP again.'),
    dict(key='m4', label='7 - SCALE',
         text='Hold GRIP, move 30 cm away from where you gripped, and while '
              'still holding push the THUMBSTICK fully UP for 2 s, then '
              'fully DOWN for 2 s. Release GRIP.'),
    dict(key='m5', label='8 - GRIPPER',
         text='Squeeze the TRIGGER slowly all the way in. Hold it for 2 '
              'seconds. Let it out slowly. Wait 2 seconds.'),
]


# ===================================================================== ANALYSIS
def axis_verdict(seg, axis_name):
    """Which way did the ARM go when the hand went one way?

    Reports the commanded world-frame displacement AND the end-effector
    displacement, because they answer different questions: the command is what
    the mapping produced, the EE is what the robot did with it.
    """
    out = dict(step=axis_name)
    ctrl = np.array([p for _, p in seg['ctrl']]) if seg['ctrl'] else None
    cmd = np.array([p for _, p in seg['cmd']]) if seg['cmd'] else None
    if ctrl is None or len(ctrl) < 5:
        out['error'] = 'no controller poses'
        return out
    if cmd is None or len(cmd) < 5:
        out['error'] = ('no commands -- the clutch never engaged, so the '
                        'mapping was never exercised')
        return out
    # Largest excursion from the start, not last-minus-first: the operator
    # returns their hand, and end-to-end would then read zero.
    d_ctrl = ctrl - ctrl[0]
    d_cmd = cmd - cmd[0]
    i = int(np.argmax(np.linalg.norm(d_ctrl, axis=1)))
    j = int(np.argmax(np.linalg.norm(d_cmd, axis=1)))
    out['controller_moved_m'] = [round(float(x), 4) for x in d_ctrl[i]]
    out['command_moved_m'] = [round(float(x), 4) for x in d_cmd[j]]
    out['controller_dominant_axis'] = 'xyz'[int(np.argmax(np.abs(d_ctrl[i])))]
    out['command_dominant_axis'] = 'xyz'[int(np.argmax(np.abs(d_cmd[j])))]
    out['command_dominant_sign'] = (
        '+' if d_cmd[j][int(np.argmax(np.abs(d_cmd[j])))] > 0 else '-')
    for arm in ('left', 'right'):
        ee = seg['ee'].get(arm) or []
        if len(ee) > 5:
            E = np.array([p for _, p in ee])
            dE = E - E[0]
            k = int(np.argmax(np.linalg.norm(dE, axis=1)))
            out['ee_%s_moved_m' % arm] = [round(float(x), 4) for x in dE[k]]
            out['ee_%s_travel_mm' % arm] = round(
                float(np.linalg.norm(dE[k])) * 1000, 1)
    return out


def analyse(key, seg):
    if key.startswith('axis_'):
        return axis_verdict(seg, key)

    if key == 'm1':
        out = {}
        for h in ('left', 'right'):
            ts = [t for t, _ in (seg['pose_%s' % h] or [])]
            r = rate_hz(ts)
            out['rate_%s_hz' % h] = round(r['hz'], 2) if r['n'] > 10 else None
            out['worst_gap_ms_%s' % h] = (round(r['worst_gap_ms'], 1)
                                          if r['n'] > 10 else None)
        lat = [v for _, v in seg['lat'] if len(v) > 3 and not np.isnan(v[1])]
        if lat:
            a = np.array(lat)
            out['rtt_mean_ms'] = round(float(a[:, 1].mean()), 2)
            out['rtt_p95_ms'] = round(float(a[:, 2].mean()), 2)
            out['bridge_ms'] = round(float(a[:, 3].mean()), 3)
            out['one_way_estimate_ms'] = round(
                out['rtt_mean_ms'] / 2 + out['bridge_ms'], 2)
            out['frames'] = int(a[-1, 5])
            out['dropped'] = int(a[-1, 4])
        return out

    if key == 'm2':
        # WHICH SIGNAL IS BEING TESTED, AND THE PRECONDITION THAT MAKES THE
        # ANSWER MEAN ANYTHING.
        #
        # `/vr/tracking_ok` defaults FALSE and is derived from frame ARRIVAL,
        # so covering ONE controller while the headset streams on does not
        # move it -- measured with the real Quest: zero transitions at 90 Hz.
        # `/vr/controller_valid_<hand>` is the signal that was added for
        # exactly this case, so it is preferred and named in the output.
        #
        # AND: a freeze proves nothing unless the signal was TRUE FIRST. A
        # topic that was silent the whole time, or false from the start,
        # produces the same "arm did not move" as a working freeze. So the
        # verdict reports `was_true_before` and REFUSES to pass without it.
        hand = seg.get('hand', 'right')
        src_name, tr = 'controller_valid_%s' % hand, seg.get(
            'valid_%s' % hand) or []
        if not tr:
            src_name, tr = 'tracking_ok', seg.get('track') or []
        was_true = any(ok for _, ok in tr)
        losses = [t for i, (t, ok) in enumerate(tr)
                  if not ok and i and tr[i - 1][1]]
        cmd_t = [t for t, _ in seg['cmd']]
        out = dict(commands=len(cmd_t), losses=len(losses), signal=src_name,
                   samples=len(tr), was_true_before=bool(was_true),
                   ended_false=bool(tr and not tr[-1][1]))
        if not tr:
            out['verdict'] = ('NOT MEASURED - neither /vr/controller_valid_%s '
                              'nor /vr/tracking_ok published ANYTHING. A '
                              'frozen arm here would prove only that the '
                              'topic was silent.' % hand)
            return out
        if not was_true:
            out['verdict'] = ('NOT MEASURED - %s was never TRUE, so it could '
                              'not go false. The arm not moving proves '
                              'nothing.' % src_name)
            return out
        if not cmd_t:
            out['verdict'] = 'NOT MEASURED - clutch never engaged'
        elif not losses:
            out['verdict'] = (
                'NOT MEASURED - /vr/tracking_ok never went false. The '
                'watchdog is keyed on FRAME ARRIVAL, so covering one '
                'controller while the headset keeps streaming does not reach '
                'it. This is a defect in the bridge, not in the action.')
            # what the client itself said about validity, which DOES change
            inval = seg.get('invalid_right', 0)
            out['frames_client_marked_INVALID'] = inval
        else:
            out['events'] = freeze_latency_s(losses, cmd_t)
            worst = max(e['latency_s'] for e in out['events'])
            out['worst_latency_s'] = round(worst, 3)
            out['verdict'] = 'PASS' if worst <= 0.2 else 'FAIL (> 0.200 s)'
        return out

    if key == 'm3':
        eng = []
        prev = None
        for t, d in seg['map']:
            e = bool(d.get('engaged'))
            if prev is False and e:
                eng.append(t)
            prev = e
        ts = [t for t, _ in seg['map']]
        for i in range(1, len(ts)):
            if ts[i] - ts[i - 1] > 0.4:
                eng.append(ts[i])
        eng = sorted(set(round(x, 2) for x in eng))
        cmd_t = [t for t, _ in seg['cmd']]
        cmd_p = [p for _, p in seg['cmd']]
        out = dict(engages_detected=len(eng))
        if not eng or not cmd_t:
            out['verdict'] = 'NOT MEASURED - no re-engage seen'
            return out
        m = reengage_jump_m(eng, cmd_t, cmd_p)
        js = [e['jump_mm'] for e in m if e.get('jump_mm') is not None]
        out['jumps_mm'] = [round(x, 3) for x in js]
        if js:
            out['worst_jump_mm'] = round(max(js), 3)
            out['verdict'] = 'PASS (< 5 mm)' if max(js) < 5 else 'LARGE'
        if seg['map']:
            out['mapper_self_report'] = seg['map'][-1][1]
        return out

    if key == 'm4':
        sc = [(t, float(d.get('scale', 0))) for t, d in seg['map']]
        stick = [(t, ax[3]) for t, ax in seg['joy'] if len(ax) > 3]
        changes = [(t, v) for i, (t, v) in enumerate(sc)
                   if i and abs(v - sc[i - 1][1]) > 1e-9]
        cont = scaling_continuity_mm(changes, [t for t, _ in seg['cmd']],
                                     [p for _, p in seg['cmd']])
        return dict(response=scaling_response(sc, stick),
                    scale_span=[round(min(v for _, v in sc), 3),
                                round(max(v for _, v in sc), 3)] if sc else None,
                    changes=len(changes),
                    continuity_worst_mm=round(max(cont), 3) if cont else None)

    if key == 'm5':
        tr = [(t, float(d.get('trigger', 0)), float(d.get('command_rad', 0)),
               bool(d.get('latched'))) for t, d in seg['grip']]
        return gripper_follows_trigger(tr)
    return {}


# ===================================================================== SELFTEST
def selftest():
    """Every analyser this script adds, against constructed truth AND against
    a deliberately broken input."""
    fails = []

    def check(name, cond, detail=''):
        print('  %-56s %s %s' % (name, 'PASS' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    def seg(ctrl, cmd, ee_l=None, ee_r=None):
        return dict(ctrl=[(i * .01, p) for i, p in enumerate(ctrl)],
                    cmd=[(i * .01, p) for i, p in enumerate(cmd)],
                    ee={'left': [(i * .01, p) for i, p in enumerate(ee_l or [])],
                        'right': [(i * .01, p) for i, p in enumerate(ee_r or [])]})

    print('\naxis_verdict -- ground truth is arithmetic')
    ctrl = [[0.3 * i / 50, 0, 0] for i in range(51)]          # hand goes +x
    cmd = [[0.15 * i / 50, 0, 0] for i in range(51)]          # arm goes +x
    v = axis_verdict(seg(ctrl, cmd), 'axis_right')
    check('hand +x, arm +x -> dominant x, sign +',
          v['command_dominant_axis'] == 'x' and v['command_dominant_sign'] == '+',
          str(v.get('command_moved_m')))
    cmd_inv = [[-0.15 * i / 50, 0, 0] for i in range(51)]     # arm goes -x
    v2 = axis_verdict(seg(ctrl, cmd_inv), 'axis_right')
    check('hand +x, arm -x -> sign is caught as -',
          v2['command_dominant_sign'] == '-', str(v2.get('command_moved_m')))
    cmd_y = [[0, 0.15 * i / 50, 0] for i in range(51)]        # arm goes +y
    v3 = axis_verdict(seg(ctrl, cmd_y), 'axis_right')
    check('hand +x, arm +y -> WRONG AXIS is caught',
          v3['command_dominant_axis'] == 'y', str(v3.get('command_moved_m')))
    v4 = axis_verdict(seg(ctrl, []), 'axis_right')
    check('no commands -> reported as not exercised, not as 0',
          'error' in v4 and 'clutch' in v4['error'])
    # the hand returns home: end-to-end would read zero, peak must not
    there_back = ([[0.3 * i / 25, 0, 0] for i in range(26)]
                  + [[0.3 * (25 - i) / 25, 0, 0] for i in range(26)])
    v5 = axis_verdict(seg(there_back, there_back), 'axis_right')
    check('a hand that returns still reports the EXCURSION, not 0',
          abs(v5['command_moved_m'][0] - 0.3) < 1e-6,
          str(v5['command_moved_m']))

    print('\nm2 verdict')
    r = analyse('m2', dict(track=[(0.0, True)] * 20, cmd=[(i * .01, [0, 0, 0])
                                                          for i in range(50)],
                           invalid_right=99))
    check('no loss -> NOT MEASURED, and says why', 'NOT MEASURED' in r['verdict'])

    # THE PRECONDITION. A freeze proves nothing unless the signal was TRUE
    # first, and both of the ways it can fail to be look identical from the
    # arm: it does not move either way.
    r_silent = analyse('m2', dict(cmd=[(i * .01, [0, 0, 0]) for i in range(50)]))
    check('a SILENT topic -> NOT MEASURED, not a pass',
          'NOT MEASURED' in r_silent['verdict']
          and r_silent['was_true_before'] is False,
          r_silent['verdict'][:58])
    r_false = analyse('m2', dict(valid_right=[(i * .05, False) for i in range(40)],
                                 cmd=[(i * .01, [0, 0, 0]) for i in range(50)]))
    check('a topic that was NEVER TRUE -> NOT MEASURED',
          'NOT MEASURED' in r_false['verdict']
          and 'never TRUE' in r_false['verdict'], r_false['verdict'][:58])
    r_pref = analyse('m2', dict(
        valid_right=[(i * .05, True) for i in range(40)]
        + [(2.0 + i * .05, False) for i in range(40)],
        track=[(i * .05, True) for i in range(80)],
        cmd=[(i * .01, [0, 0, 0]) for i in range(int(2.05 / .01))]))
    check('per-controller validity is PREFERRED over tracking_ok',
          r_pref['signal'] == 'controller_valid_right'
          and r_pref['verdict'] == 'PASS', r_pref['signal'])
    tr = [(i * .05, True) for i in range(40)] + [(2.0 + i * .05, False)
                                                 for i in range(40)]
    cmds = [(i * .01, [0, 0, 0]) for i in range(int(2.05 / .01))]
    r2 = analyse('m2', dict(track=tr, cmd=cmds, invalid_right=0))
    check('a 0.05 s over-run -> PASS', r2['verdict'] == 'PASS',
          str(r2.get('worst_latency_s')))
    cmds_bad = [(i * .01, [0, 0, 0]) for i in range(int(2.4 / .01))]
    r3 = analyse('m2', dict(track=tr, cmd=cmds_bad, invalid_right=0))
    check('a 0.40 s over-run -> FAIL', r3['verdict'].startswith('FAIL'),
          str(r3.get('worst_latency_s')))

    print('\n' + '-' * 68)
    if fails:
        print('SELFTEST FAILED: %s' % ', '.join(fails))
        return 1
    print('SELFTEST PASS: analysers answer constructed truth and reject '
          'broken input.')
    return 0


# ===================================================================== SESSION
def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Float64MultiArray, String
    import tf2_ros

    hand = args.hand
    steps = [s for s in STEPS if not args.only or s['key'] in args.only]

    class Session(Node):
        def __init__(self):
            super().__init__('vr_guided_session')
            self._hand = hand
            self.t0 = time.monotonic()
            self.lock = threading.Lock()
            self.buf = {}
            self.recording = False
            self.a_edge = threading.Event()
            self._a_prev = False
            self._a_block_until = 0.0
            self.tf_buf = tf2_ros.Buffer()
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
            self.instr = self.create_publisher(String, '/vr/robot_state_json', 10)

            for h in ('left', 'right'):
                self.create_subscription(
                    PoseStamped, '/vr/controller_pose_%s' % h,
                    lambda m, h=h: self._add('pose_%s' % h,
                                             [m.pose.position.x, m.pose.position.y,
                                              m.pose.position.z]), 200)
            self.create_subscription(PoseStamped, '/vr/controller_pose_%s' % hand,
                                     lambda m: self._add('ctrl',
                                                         [m.pose.position.x,
                                                          m.pose.position.y,
                                                          m.pose.position.z]), 200)
            self.create_subscription(PoseStamped, '/master_arm_pose_%s' % hand,
                                     lambda m: self._add('cmd',
                                                         [m.pose.position.x,
                                                          m.pose.position.y,
                                                          m.pose.position.z]), 200)
            self.create_subscription(Joy, '/vr/controller_joy_%s' % hand,
                                     self._on_joy, 200)
            for _h in ('left', 'right'):
                self.create_subscription(
                    Bool, '/vr/controller_valid_%s' % _h,
                    (lambda m, h=_h: self._add('valid_%s' % h, bool(m.data))),
                    20)
            self.create_subscription(Bool, '/vr/tracking_ok',
                                     lambda m: self._add('track', bool(m.data),
                                                         raw=True), 50)
            self.create_subscription(String, '/vr/mapper_%s' % hand,
                                     lambda m: self._add('map',
                                                         json.loads(m.data)), 50)
            self.create_subscription(String, '/vr/gripper_%s' % hand,
                                     lambda m: self._add('grip',
                                                         json.loads(m.data)), 50)
            self.create_subscription(Float64MultiArray, '/vr/latency',
                                     lambda m: self._add('lat', list(m.data)), 20)
            self.create_timer(0.05, self._sample_tf)

        def now(self):
            return time.monotonic() - self.t0

        def _add(self, key, val, raw=False):
            with self.lock:
                if self.recording:
                    self.buf.setdefault(key, []).append((self.now(), val))

        def _sample_tf(self):
            if not self.recording:
                return
            for arm in ('left', 'right'):
                try:
                    t = self.tf_buf.lookup_transform(
                        'world', '%s_end_effector_link' % arm, rclpy.time.Time())
                    v = t.transform.translation
                    with self.lock:
                        self.buf.setdefault('ee', {}).setdefault(arm, []).append(
                            (self.now(), [v.x, v.y, v.z]))
                except Exception:                            # noqa: BLE001
                    pass

        def _on_joy(self, m):
            self._add('joy', list(m.axes))
            a = bool(m.buttons and m.buttons[0])
            # RISING EDGE + hold-off. Without the hold-off one press ends the
            # current step and immediately ends the next one too, which reads
            # as a step that recorded nothing.
            if a and not self._a_prev and time.monotonic() > self._a_block_until:
                self.a_edge.set()
            self._a_prev = a

        def say(self, label, text, progress, rec_s=None):
            s = String()
            s.data = json.dumps(dict(instruction=text, step_label=label,
                                     step_progress=progress,
                                     recording=rec_s is not None,
                                     rec_s=rec_s))
            self.instr.publish(s)

        def clear(self):
            s = String()
            s.data = json.dumps(dict(instruction='', step_label='',
                                     step_progress='', recording=False,
                                     rec_s=None))
            self.instr.publish(s)

        def record_until_enter(self, label, text, progress, timeout):
            """DESK MODE. End the step on ENTER at the keyboard.

            In desk operation NOBODY WEARS THE HEADSET -- it stands on a
            shelf as the tracking reference, lenses pointing at the operator's
            hands. So the in-headset instruction panel this script was built
            around is showing text to an empty headset, and the A button ends
            a step nobody can see. The operator is at a keyboard with a
            monitor, so the keyboard ends the step.

            Everything else is identical: the same recorder, the same
            analysers, the same output file. Only who says "done" changes.
            """
            with self.lock:
                self.buf = {}
                self.recording = True
            t_start = time.monotonic()
            try:
                sys.stdin.readline()
            except KeyboardInterrupt:
                pass
            el = time.monotonic() - t_start
            if el > timeout:
                print('    (%.0f s, past the %.0f s limit)' % (el, timeout))
            with self.lock:
                self.recording = False
                seg = self.buf
                self.buf = {}
            for k in ('ctrl', 'cmd', 'map', 'grip', 'joy', 'lat', 'track',
                      'valid_left', 'valid_right', 'pose_left', 'pose_right'):
                seg.setdefault(k, [])
            seg.setdefault('ee', {'left': [], 'right': []})
            seg['hand'] = self._hand
            seg['recorded_s'] = round(el, 2)
            return seg

        def record_until_a(self, label, text, progress, timeout):
            with self.lock:
                self.buf = {}
                self.recording = True
            self.a_edge.clear()
            self._a_block_until = time.monotonic() + 1.0
            t_start = time.monotonic()
            while not self.a_edge.is_set():
                el = time.monotonic() - t_start
                if el > timeout:
                    print('    (timed out after %.0f s -- closing the step)'
                          % timeout, flush=True)
                    break
                self.say(label, text, progress, rec_s=int(el))
                time.sleep(0.25)
            with self.lock:
                self.recording = False
                seg = self.buf
                self.buf = {}
            self._a_block_until = time.monotonic() + 1.0
            for k in ('ctrl', 'cmd', 'map', 'grip', 'joy', 'lat', 'track',
                      'valid_left', 'valid_right',
                      'pose_left', 'pose_right'):
                seg.setdefault(k, [])
            seg.setdefault('ee', {'left': [], 'right': []})
            return seg

    rclpy.init()
    n = Session()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)

    print('=' * 72)
    if getattr(args, 'desk', False):
        print('VR MEASUREMENT -- DESK MODE. Nobody is wearing the headset.')
        print('Read the instruction here, do it, then press ENTER.')
    else:
        print('GUIDED VR SESSION -- SIM ONLY. Instructions appear IN THE '
              'HEADSET.')
        print('Do each action, then press  A  on the %s controller.' % hand)
    print('=' * 72)

    report = {}
    try:
        for i, st in enumerate(steps, 1):
            prog = '%d/%d' % (i, len(steps))
            print('\n[%s] %s' % (prog, st['label']))
            print('    %s' % st['text'])
            if getattr(args, 'desk', False):
                print('    ... recording. Press ENTER when the action is '
                      'finished.', flush=True)
                seg = n.record_until_enter(st['label'], st['text'], prog,
                                           args.timeout)
            else:
                n.say(st['label'], st['text'], prog, rec_s=0)
                seg = n.record_until_a(st['label'], st['text'], prog,
                                       args.timeout)
            res = analyse(st['key'], seg)
            report[st['key']] = res
            print('    -> %s' % json.dumps(res, default=str)[:400])
    finally:
        n.clear()
        time.sleep(0.5)
        ex.shutdown()
        rclpy.shutdown()

    print('\n' + '=' * 72)
    print(json.dumps(report, indent=2, default=str))
    out = args.out or os.path.join(
        WS, 'recordings', 'baselines', 'vr_headset_session.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if getattr(args, 'append', False) and os.path.exists(out):
        # ONE STEP AT A TIME STILL BUILDS ONE FILE. Without this, running the
        # steps separately -- which is the whole point of desk mode -- leaves
        # only the last one on disk, and the session's evidence is whichever
        # measurement happened to be run last.
        try:
            with open(out) as f:
                prev = json.load(f)
            prev.update(report)
            report = prev
        except Exception:                                     # noqa: BLE001
            pass
    with open(out, 'w') as f:
        json.dump(report, f, indent=2, default=str)
    print('\nwritten: %s' % out)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hand', default='right', choices=['left', 'right'])
    ap.add_argument('--only', nargs='*', default=[],
                    help='step keys: %s' % ' '.join(s['key'] for s in STEPS))
    ap.add_argument('--timeout', type=float, default=180.0,
                    help='seconds before a step closes itself')
    ap.add_argument('--out', default='')
    ap.add_argument('--desk', action='store_true',
                    help='DESK OPERATION: nobody wears the headset, so a '
                         'step ends on ENTER at the keyboard rather than on '
                         'the A button on a controller held by somebody who '
                         'cannot see the panel')
    ap.add_argument('--append', action='store_true',
                    help='merge into the existing report instead of replacing '
                         'it, so one step at a time still builds one file')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    sys.exit(selftest() if a.selftest else run(a))


if __name__ == '__main__':
    main()
