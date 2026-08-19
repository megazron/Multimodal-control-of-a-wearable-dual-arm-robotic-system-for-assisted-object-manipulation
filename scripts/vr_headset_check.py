#!/usr/bin/env python3
"""
vr_headset_check.py -- the measurements that ONLY a physical Quest can give.

Everything in vr_bringup.md section 5 was measured against the desktop mock,
which runs on this host and therefore reports a FLOOR, not a prediction: 68.5
Hz is the mock's loop rate, 8.66 ms is a loopback round trip, and its tracking
dropouts are a clean on/off that no real headset produces. Section 11 lists
what is still open. This script closes it, in sim, with the arms simulated.

SEVEN MEASUREMENTS, each with the operator action that produces it:

  1  pose rate            just wear it and move
  2  end-to-end latency   client-stamped round trip + the ROS-side leg
  3  freeze on tracking   cover a controller / set it down / step out of view
  4  clutch re-engage     grip, move, release, move, grip again
  5  live scaling         thumbstick up and down while driving
  6  gripper on trigger   squeeze in, hold, release
  7  in-headset overlay   read it back to the terminal

USAGE

    ros2 run srl_vr_teleop ...            # or: bash scripts/start_vr_wifi.sh
    python3 scripts/vr_headset_check.py                    # guided, all seven
    python3 scripts/vr_headset_check.py --only 3 4
    python3 scripts/vr_headset_check.py --selftest         # NO headset needed

THE SELF-TEST IS THE POINT OF THIS FILE BEING SPLIT THE WAY IT IS. Every
number below comes out of a pure function over a recorded trace, and
--selftest feeds those functions traces whose answer is CONSTRUCTED
arithmetic: a 72.0 Hz stream really is 72.0 Hz, a 0.31 s freeze really is
0.31 s, a 14 mm jump really is 14 mm. It also feeds each one a trace that is
deliberately wrong and requires the analyser to say so. An analyser that
passes a broken trace is not a measurement, and this project has been caught
by that seventeen times.

WHAT THIS DOES NOT DO: touch a real arm. Sim only. It publishes nothing except
the reset request in step 4.
"""
import argparse
import json
import math
import sys
import time

import numpy as np

# ============================================================== ANALYSERS
# Pure functions over recorded traces. No ROS. Each has a known answer.


def rate_hz(stamps):
    """Sample rate from monotonic stamps, and the jitter that matters."""
    s = np.asarray(sorted(stamps), float)
    if len(s) < 10:
        return dict(n=len(s), hz=float('nan'), p95_gap_ms=float('nan'),
                    worst_gap_ms=float('nan'))
    d = np.diff(s)
    return dict(n=len(s),
                hz=float((len(s) - 1) / (s[-1] - s[0])),
                # A mean rate hides dropped frames. At 72 Hz a run of missed
                # frames shows here and nowhere else, and a 200 ms gap is a
                # freeze whatever the mean says.
                p95_gap_ms=float(np.percentile(d, 95) * 1000.0),
                worst_gap_ms=float(d.max() * 1000.0))


def freeze_latency_s(loss_events, cmd_stamps, window=3.0):
    """How long after tracking is lost does the LAST command go out?

    Not 'when did we notice' -- when did the arm stop being commanded. The
    requirement is 0.2 s, and the thing being measured is the interval during
    which a robot is still moving on poses nobody is producing.
    """
    cmd = np.asarray(sorted(cmd_stamps), float)
    out = []
    for t in sorted(loss_events):
        after = cmd[(cmd >= t) & (cmd <= t + window)]
        out.append(dict(t=t, latency_s=float(after.max() - t) if len(after) else 0.0,
                        commands_after_loss=int(len(after))))
    return out


def reengage_jump_m(engage_times, cmd_stamps, cmd_positions, gap=1.0):
    """Distance between the last commanded point before an engage and the
    first one after it.

    Measured from the COMMAND stream rather than read out of the mapper's own
    self-report, because the mapper computes its jump from the same numbers it
    would be grading. `mean_reengage_jump_m` on /vr/mapper_<hand> is the
    node's opinion; this is the topic.
    """
    st = np.asarray(cmd_stamps, float)
    ps = np.asarray(cmd_positions, float)
    out = []
    for t in sorted(engage_times):
        b = np.where((st < t) & (st > t - gap))[0]
        a = np.where((st >= t) & (st < t + gap))[0]
        if not len(b) or not len(a):
            out.append(dict(t=t, jump_mm=None, note='no command either side'))
            continue
        out.append(dict(t=t,
                        jump_mm=float(np.linalg.norm(ps[a[0]] - ps[b[-1]]) * 1000.0)))
    return out


def scaling_response(scale_series, stick_series):
    """Did the thumbstick actually move the scale, and in the right direction?

    A scale that never changes and a scale that changes on its own look the
    same in a status line. Correlating against the STICK separates them.
    """
    if len(scale_series) < 5 or len(stick_series) < 5:
        return dict(ok=False, why='too few samples')
    st = np.array([t for t, _ in scale_series], float)
    sc = np.array([v for _, v in scale_series], float)
    ss = np.array([v for _, v in stick_series], float)
    sst = np.array([t for t, _ in stick_series], float)
    stick_at = np.interp(st, sst, ss)
    d = np.diff(sc)
    push = stick_at[1:]
    moved = np.abs(d) > 1e-9
    if not moved.any():
        return dict(ok=False, why='scale never changed', span=[float(sc.min()),
                                                              float(sc.max())])
    agree = float(np.mean(np.sign(d[moved]) == np.sign(push[moved])))
    return dict(ok=agree > 0.9, agreement=agree,
                span=[float(sc.min()), float(sc.max())],
                changes=int(moved.sum()))


def scaling_continuity_mm(scale_series, cmd_stamps, cmd_positions, win=0.15):
    """The command must NOT step when the scale changes.

    Changing scale without re-basing the anchor moves the arm by
    (new-old) x displacement, worst exactly when the operator is far from the
    engage point -- so this is measured far from it or it is not measured.
    """
    st = np.asarray(cmd_stamps, float)
    ps = np.asarray(cmd_positions, float)
    out = []
    for t, _v in scale_series:
        b = np.where((st < t) & (st > t - win))[0]
        a = np.where((st >= t) & (st < t + win))[0]
        if len(b) < 2 or len(a) < 2:
            continue
        # predicted next point from the pre-change velocity
        dt = st[b[-1]] - st[b[-2]]
        vel = (ps[b[-1]] - ps[b[-2]]) / dt if dt > 0 else np.zeros(3)
        pred = ps[b[-1]] + vel * (st[a[0]] - st[b[-1]])
        out.append(float(np.linalg.norm(ps[a[0]] - pred) * 1000.0))
    return out


def gripper_follows_trigger(trace):
    """trace: [(t, trigger, command_rad, latched)]. Does the command track the
    trigger, does it LATCH, and does it release only after the hold?"""
    if len(trace) < 20:
        return dict(ok=False, why='too few samples')
    tr = np.array([r[1] for r in trace], float)
    cm = np.array([r[2] for r in trace], float)
    la = np.array([bool(r[3]) for r in trace])
    if tr.max() < 0.35:
        return dict(ok=False, why='trigger never pulled past the latch '
                                  'threshold (0.30); squeeze harder')
    rising = (tr < 0.3) & (np.roll(tr, -1) >= 0.3)
    latched_ever = bool(la.any())
    # CORRELATE ONLY WHERE PROPORTIONAL CONTROL IS IN FORCE -- i.e. before the
    # latch. Once latched the command deliberately HOLDS the firmest value
    # seen while the trigger falls away, so a correlation over the whole trace
    # is near zero for a perfectly working gripper. The self-test caught this
    # analyser scoring a known-good trace at r = 0.033 and calling it broken:
    # the instrument was wrong, not the gripper.
    free = ~la
    if free.sum() >= 5 and cm[free].std() > 1e-9 and tr[free].std() > 1e-9:
        corr = float(np.corrcoef(tr[free], cm[free])[0, 1])
        basis = 'pre-latch (%d samples)' % int(free.sum())
    elif cm.std() > 1e-9:
        corr = float(np.corrcoef(tr, cm)[0, 1])
        basis = 'whole trace -- never unlatched'
    else:
        corr = 0.0
        basis = 'command never moved'
    # while latched the command must never DROP -- that is the whole point of
    # holding the firmest value seen
    drops = 0
    for i in range(1, len(cm)):
        if la[i] and la[i - 1] and cm[i] < cm[i - 1] - 1e-9:
            drops += 1
    return dict(ok=(corr > 0.5 and latched_ever and drops == 0),
                correlation=round(corr, 3), correlation_basis=basis,
                latched=latched_ever,
                drops_while_latched=drops,
                max_trigger=round(float(tr.max()), 3),
                max_command_rad=round(float(cm.max()), 4),
                rising_edges=int(rising.sum()))


# ============================================================== SELF-TEST
def _synth_rate(hz, n, gap_at=None, gap=0.0):
    t = [i / hz for i in range(n)]
    if gap_at is not None:
        t = t[:gap_at] + [x + gap for x in t[gap_at:]]
    return t


def selftest():
    fails = []

    def check(name, cond, detail=''):
        print('  %-58s %s %s' % (name, 'PASS' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    print('\n1. rate_hz -- ground truth is arithmetic')
    r = rate_hz(_synth_rate(72.0, 720))
    check('72.0 Hz stream reads 72.0 Hz', abs(r['hz'] - 72.0) < 0.05,
          '(%.3f)' % r['hz'])
    r2 = rate_hz(_synth_rate(72.0, 720, gap_at=300, gap=0.25))
    check('a 250 ms hole is REPORTED, not averaged away',
          r2['worst_gap_ms'] > 240, '(worst gap %.0f ms)' % r2['worst_gap_ms'])
    check('and the mean rate alone would have HIDDEN it',
          abs(r2['hz'] - 72.0) < 2.0, '(mean still %.1f Hz)' % r2['hz'])
    check('a 30 Hz stream does NOT read 72', abs(rate_hz(
        _synth_rate(30.0, 300))['hz'] - 30.0) < 0.05)

    print('\n2. freeze_latency_s')
    cmds = _synth_rate(100.0, 500)
    # commands stop 0.31 s after a loss at t = 2.0
    loss = 2.0
    cmds_broken = [c for c in cmds if c <= loss + 0.31 or c > loss + 3.5]
    f = freeze_latency_s([loss], cmds_broken)[0]
    check('a 0.31 s over-run measures 0.31 s', abs(f['latency_s'] - 0.31) < 0.011,
          '(%.3f s)' % f['latency_s'])
    check('and 0.31 s FAILS the 0.2 s requirement', f['latency_s'] > 0.2)
    cmds_good = [c for c in cmds if c <= loss + 0.05 or c > loss + 3.5]
    g = freeze_latency_s([loss], cmds_good)[0]
    check('a compliant 0.05 s freeze passes', g['latency_s'] < 0.2,
          '(%.3f s)' % g['latency_s'])
    check('no command at all after loss reads 0.000 s',
          freeze_latency_s([loss], [c for c in cmds if c < loss])[0]
          ['latency_s'] == 0.0)

    print('\n3. reengage_jump_m')
    st = _synth_rate(100.0, 400)
    ps = [[0.0, 0.0, 0.0] for _ in st]
    for i, t in enumerate(st):
        if t >= 2.0:
            ps[i] = [0.014, 0.0, 0.0]          # a 14 mm step at the engage
    j = reengage_jump_m([2.0], st, ps)[0]
    check('a constructed 14 mm step measures 14 mm',
          abs(j['jump_mm'] - 14.0) < 0.01, '(%.3f mm)' % j['jump_mm'])
    j0 = reengage_jump_m([2.0], st, [[0.0, 0.0, 0.0] for _ in st])[0]
    check('a continuous stream measures 0 mm', j0['jump_mm'] < 1e-9)

    print('\n4. scaling_response')
    sc = [(i * 0.1, 0.5 + 0.05 * i) for i in range(20)]
    stick = [(i * 0.1, 1.0) for i in range(20)]
    check('stick up + scale up = agreement', scaling_response(sc, stick)['ok'])
    down = [(i * 0.1, -1.0) for i in range(20)]
    check('stick DOWN + scale up is caught',
          not scaling_response(sc, down)['ok'])
    flat = [(i * 0.1, 0.5) for i in range(20)]
    check('a scale that never moves is caught',
          not scaling_response(flat, stick)['ok'])

    print('\n5. scaling_continuity_mm')
    st = _synth_rate(100.0, 400)
    ps = [[0.4 * t, 0.0, 0.0] for t in st]                 # constant velocity
    cont = scaling_continuity_mm([(2.0, 1.0)], st, ps)
    check('a re-based (continuous) change reads ~0 mm',
          max(cont) < 0.05, '(%.4f mm)' % max(cont))
    ps_jump = [[0.4 * t + (0.02 if t >= 2.0 else 0.0), 0.0, 0.0] for t in st]
    disc = scaling_continuity_mm([(2.0, 1.0)], st, ps_jump)
    check('a 20 mm un-rebased step is caught', max(disc) > 19.0,
          '(%.2f mm)' % max(disc))

    print('\n6. gripper_follows_trigger')
    tr, cmd, lat, out = 0.0, 0.0, False, []
    for i in range(200):
        tr = min(1.0, i / 100.0) if i < 120 else 0.0
        c = 0.0 if tr < 0.06 else (tr - 0.06) / 0.94 * 0.80
        if tr > 0.30:
            lat = True
        if lat:
            cmd = max(cmd, c)
        else:
            cmd = c
        out.append((i * 0.02, tr, cmd, lat))
    g = gripper_follows_trigger(out)
    check('a latching, non-dropping trace passes', g['ok'], str(g))
    bad = [(t, x, 0.0, False) for t, x, _c, _l in out]
    check('a gripper that never moves is caught',
          not gripper_follows_trigger(bad)['ok'])
    dropper = []
    c = 0.0
    for i, (t, x, _cc, _ll) in enumerate(out):
        c = 0.8 * (1.0 - i / 200.0)
        dropper.append((t, x, c, True))
    check('a command that SAGS while latched is caught',
          not gripper_follows_trigger(dropper)['ok'])

    print('\n' + ('-' * 72))
    if fails:
        print('SELFTEST FAILED: %d analyser(s) cannot tell right from wrong: %s'
              % (len(fails), ', '.join(fails)))
        return 1
    print('SELFTEST PASS: every analyser answered a constructed-truth trace '
          'correctly AND rejected a deliberately broken one.')
    return 0


# ============================================================== RECORDING
def run_live(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Empty, Float64MultiArray, String

    class Rec(Node):
        def __init__(self):
            super().__init__('vr_headset_check')
            self.t0 = time.monotonic()
            self.pose_t = {'left': [], 'right': []}
            self.joy = {'left': [], 'right': []}
            self.cmd_t = {'left': [], 'right': []}
            self.cmd_p = {'left': [], 'right': []}
            self.track = []            # (t, ok)
            self.mapper = {'left': [], 'right': []}
            self.grip = {'left': [], 'right': []}
            self.bridge = []
            self.latency = []
            self.state_json = []
            self.reset_pub = self.create_publisher(Empty, '/vr/reset_request', 1)

            for h in ('left', 'right'):
                self.create_subscription(
                    PoseStamped, '/vr/controller_pose_%s' % h,
                    lambda m, h=h: self.pose_t[h].append(self.now()), 50)
                self.create_subscription(
                    Joy, '/vr/controller_joy_%s' % h,
                    lambda m, h=h: self.joy[h].append(
                        (self.now(), list(m.axes), list(m.buttons))), 50)
                self.create_subscription(
                    PoseStamped, '/master_arm_pose_%s' % h,
                    lambda m, h=h: self._cmd(h, m), 50)
                self.create_subscription(
                    String, '/vr/mapper_%s' % h,
                    lambda m, h=h: self.mapper[h].append(
                        (self.now(), json.loads(m.data))), 20)
                self.create_subscription(
                    String, '/vr/gripper_%s' % h,
                    lambda m, h=h: self.grip[h].append(
                        (self.now(), json.loads(m.data))), 20)
            self.create_subscription(Bool, '/vr/tracking_ok',
                                     lambda m: self.track.append((self.now(), m.data)), 20)
            self.create_subscription(String, '/vr/bridge_status',
                                     lambda m: self.bridge.append(
                                         (self.now(), json.loads(m.data))), 10)
            self.create_subscription(Float64MultiArray, '/vr/latency',
                                     lambda m: self.latency.append(
                                         (self.now(), list(m.data))), 10)
            self.create_subscription(String, '/vr/robot_state_json',
                                     lambda m: self.state_json.append(self.now()), 10)

        def now(self):
            return time.monotonic() - self.t0

        def _cmd(self, h, m):
            self.cmd_t[h].append(self.now())
            self.cmd_p[h].append([m.pose.position.x, m.pose.position.y,
                                  m.pose.position.z])

    rclpy.init()
    rec = Rec()

    def spin(seconds, msg=None):
        if msg:
            print('    %s' % msg, flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(rec, timeout_sec=0.05)

    def ask(prompt):
        print('\n  >>> %s' % prompt)
        print('      press ENTER here when you have done it (the headset can '
              'stay on)', flush=True)
        # keep the graph alive while the operator reads the prompt
        import select
        while True:
            rclpy.spin_once(rec, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                return

    steps = args.only or [1, 2, 3, 4, 5, 6, 7]
    report = {}
    hand = args.hand

    print('=' * 72)
    print('VR HEADSET CHECK -- SIM ONLY. Nothing here drives a real arm.')
    print('=' * 72)

    print('\n[0] waiting for the headset to connect...')
    end = time.monotonic() + args.connect_timeout
    while time.monotonic() < end:
        rclpy.spin_once(rec, timeout_sec=0.1)
        if rec.bridge and rec.bridge[-1][1].get('clients', 0) > 0:
            break
    if not (rec.bridge and rec.bridge[-1][1].get('clients', 0) > 0):
        print('  NO CLIENT on the bridge after %.0f s.' % args.connect_timeout)
        print('  Run scripts/vr_reachability.py first: this is a network or a')
        print('  certificate problem, not a measurement problem.')
        rclpy.shutdown()
        return 2
    print('  client connected.')

    # ---------------------------------------------------------------- 1 + 2
    if 1 in steps or 2 in steps:
        print('\n[1,2] POSE RATE AND LATENCY')
        ask('Wear the headset, hold both controllers, and move them around '
            'continuously for %d s at a natural teleoperation speed.'
            % args.dwell)
        spin(args.dwell, 'recording...')
        for h in ('left', 'right'):
            report['rate_%s' % h] = rate_hz(rec.pose_t[h])
        lat = [v for _t, v in rec.latency if len(v) >= 4 and not math.isnan(v[1])]
        if lat:
            report['latency'] = dict(
                client_rtt_mean_ms=round(float(np.mean([v[1] for v in lat])), 2),
                client_rtt_p95_ms=round(float(np.mean([v[2] for v in lat])), 2),
                bridge_proc_ms=round(float(np.mean([v[3] for v in lat])), 3),
                dropped_frames=int(lat[-1][4]), frames=int(lat[-1][5]))
            report['latency']['one_way_estimate_ms'] = round(
                report['latency']['client_rtt_mean_ms'] / 2.0
                + report['latency']['bridge_proc_ms'], 2)
        else:
            report['latency'] = dict(note='client reported no round trip; the '
                                          'page must be echoing echo_t')

    # -------------------------------------------------------------------- 3
    if 3 in steps:
        print('\n[3] TRACKING LOSS MUST FREEZE THE ARM INSIDE 0.2 s')
        print('    A dropout, a sleeping headset and a backgrounded app all')
        print('    look identical from here, and all must freeze.')
        ask('Grip to engage the clutch and drive the arm. Then, WITHOUT '
            'releasing the grip, cover the %s controller completely with '
            'your other hand (or set it face-down on the table) and hold it '
            'there for 3 s.' % hand)
        spin(args.dwell, 'recording...')
        losses = [t for i, (t, ok) in enumerate(rec.track)
                  if not ok and (i == 0 or rec.track[i - 1][1])]
        report['freeze'] = dict(
            losses_seen=len(losses),
            events=freeze_latency_s(losses, rec.cmd_t[hand]))
        if not losses:
            report['freeze']['note'] = (
                'NO tracking loss was observed. The measurement did not '
                'happen -- do not read this as a pass.')

    # -------------------------------------------------------------------- 4
    if 4 in steps:
        print('\n[4] CLUTCH RE-ENGAGE JUMP')
        print('    Resetting the mapper first, so this run does not begin on')
        print('    the previous run\'s anchors.')
        rec.reset_pub.publish(Empty())
        spin(1.0)
        ask('Do this three times: GRIP and hold, move the controller ~30 cm, '
            'RELEASE the grip, move your hand somewhere else, then GRIP '
            'again. Hold still for a moment before each re-grip.')
        spin(args.dwell, 'recording...')
        eng = []
        prev = None
        for t, d in rec.mapper[hand]:
            e = bool(d.get('engaged'))
            if prev is False and e:
                eng.append(t)
            prev = e
        # the mapper only publishes while engaged, so an engage is also the
        # first sample after a gap
        gaps = []
        ts = [t for t, _ in rec.mapper[hand]]
        for i in range(1, len(ts)):
            if ts[i] - ts[i - 1] > 0.5:
                gaps.append(ts[i])
        engages = sorted(set([round(x, 2) for x in eng + gaps]))
        report['reengage'] = dict(
            engages_seen=len(engages),
            measured=reengage_jump_m(engages, rec.cmd_t[hand], rec.cmd_p[hand]),
            mapper_self_report=(rec.mapper[hand][-1][1] if rec.mapper[hand] else None))

    # -------------------------------------------------------------------- 5
    if 5 in steps:
        print('\n[5] LIVE SCALING FROM THE THUMBSTICK')
        ask('GRIP and hold, move ~30 cm away from where you gripped, and '
            'while still holding, push the %s thumbstick fully UP for 2 s, '
            'then fully DOWN for 2 s. Staying far from the engage point is '
            'the point: an un-rebased scale change is worst there.' % hand)
        spin(args.dwell, 'recording...')
        sc = [(t, float(d.get('scale', 0.0))) for t, d in rec.mapper[hand]]
        stick = [(t, ax[3]) for t, ax, _b in rec.joy[hand] if len(ax) > 3]
        changes = [(t, v) for i, (t, v) in enumerate(sc)
                   if i and abs(v - sc[i - 1][1]) > 1e-9]
        cont = scaling_continuity_mm(changes, rec.cmd_t[hand], rec.cmd_p[hand])
        report['scaling'] = dict(
            response=scaling_response(sc, stick),
            continuity_worst_mm=round(max(cont), 3) if cont else None,
            continuity_n=len(cont))

    # -------------------------------------------------------------------- 6
    if 6 in steps:
        print('\n[6] GRIPPER ON THE TRIGGER')
        ask('Squeeze the %s TRIGGER slowly all the way in, hold it for 2 s, '
            'let it out slowly, and wait 2 s. The gripper LATCHES on purpose '
            '-- it should not sag while you hold it, and it should not open '
            'the instant you ease off.' % hand)
        spin(args.dwell, 'recording...')
        tr = [(t, float(d.get('trigger', 0.0)), float(d.get('command_rad', 0.0)),
               bool(d.get('latched'))) for t, d in rec.grip[hand]]
        report['gripper'] = gripper_follows_trigger(tr)

    # -------------------------------------------------------------------- 7
    if 7 in steps:
        print('\n[7] IN-HEADSET OVERLAY')
        spin(2.0)
        st = rate_hz(rec.state_json)
        cl = rec.bridge[-1][1] if rec.bridge else {}
        report['overlay'] = dict(
            robot_state_hz=round(st['hz'], 2) if not math.isnan(st['hz']) else None,
            bridge_clients=cl.get('clients'),
            frames_pushed=cl.get('frames'))
        print('    /vr/robot_state_json at %s Hz, %s client(s) on the bridge.'
              % (report['overlay']['robot_state_hz'],
                 report['overlay']['bridge_clients']))
        print('    The DATA channel is measurable from here; the RENDERING is')
        print('    not. Answer for the panel you can see:')
        for q in ('Is the panel visible and readable inside the headset?',
                  'Does it show the pose rate and the round trip?',
                  'Does per-arm clutch state change when you grip?',
                  'Does FROZEN + the reason appear when you cover a '
                  'controller?'):
            print('      - %s [y/n] ' % q, end='', flush=True)
            report.setdefault('overlay_operator', {})[q] = \
                sys.stdin.readline().strip().lower().startswith('y')

    rclpy.shutdown()

    # ------------------------------------------------------------- verdict
    print('\n' + '=' * 72)
    print(json.dumps(report, indent=2, default=str))
    print('=' * 72)

    fails = []
    for h in ('left', 'right'):
        r = report.get('rate_%s' % h)
        if r and not math.isnan(r['hz']):
            ok = r['hz'] >= 60.0
            print('  %-42s %7.1f Hz   %s' % ('pose rate, %s' % h, r['hz'],
                                             'OK' if ok else 'BELOW 60 Hz'))
            if not ok:
                fails.append('pose rate %s' % h)
    if 'latency' in report and 'one_way_estimate_ms' in report['latency']:
        v = report['latency']['one_way_estimate_ms']
        print('  %-42s %7.1f ms   %s' % ('one-way estimate', v,
                                         'OK' if v < 30 else 'over the 30 ms budget'))
        if v >= 30:
            fails.append('latency')
    fz = report.get('freeze', {})
    if fz.get('events'):
        w = max(e['latency_s'] for e in fz['events'])
        print('  %-42s %7.3f s    %s' % ('freeze after tracking loss', w,
                                         'OK' if w <= 0.2 else 'OVER 0.2 s'))
        if w > 0.2:
            fails.append('freeze latency')
    elif 3 in steps:
        print('  freeze after tracking loss                NOT MEASURED '
              '(no loss occurred)')
        fails.append('freeze NOT MEASURED')
    re_ = report.get('reengage', {})
    js = [e['jump_mm'] for e in re_.get('measured', []) if e.get('jump_mm') is not None]
    if js:
        print('  %-42s %7.2f mm   %s' % ('re-engage jump, worst', max(js),
                                         'OK' if max(js) < 5 else 'LARGE'))
        if max(js) >= 5:
            fails.append('re-engage jump')
    elif 4 in steps:
        print('  re-engage jump                            NOT MEASURED')
        fails.append('re-engage NOT MEASURED')
    sc = report.get('scaling')
    if sc:
        print('  %-42s %s' % ('thumbstick scaling', 'OK' if sc['response'].get('ok')
                              else 'FAILED: %s' % sc['response']))
        if not sc['response'].get('ok'):
            fails.append('scaling')
        if sc.get('continuity_worst_mm') is not None:
            ok = sc['continuity_worst_mm'] < 5.0
            print('  %-42s %7.2f mm   %s' % ('  command step at scale change',
                                             sc['continuity_worst_mm'],
                                             'OK' if ok else 'NOT RE-BASED'))
            if not ok:
                fails.append('scale re-base')
    gp = report.get('gripper')
    if gp:
        print('  %-42s %s' % ('gripper follows trigger',
                              'OK' if gp.get('ok') else 'FAILED: %s' % gp))
        if not gp.get('ok'):
            fails.append('gripper')

    print('\n' + ('ALL MEASURED ITEMS PASS' if not fails else
                  'OPEN: ' + ', '.join(fails)))
    print('\nNOTE: an item that says NOT MEASURED is not a pass. It means the')
    print('operator action did not happen and the number does not exist.')

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        print('\nwritten: %s' % args.out)
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true',
                    help='validate every analyser against constructed truth; '
                         'no headset, no ROS')
    ap.add_argument('--only', type=int, nargs='*', help='run only these steps')
    ap.add_argument('--hand', default='right', choices=['left', 'right'])
    ap.add_argument('--dwell', type=float, default=12.0,
                    help='seconds recorded per step')
    ap.add_argument('--connect-timeout', type=float, default=60.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    sys.exit(selftest() if a.selftest else run_live(a))


if __name__ == '__main__':
    main()
