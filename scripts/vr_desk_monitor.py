#!/usr/bin/env python3
"""
vr_desk_monitor.py -- everything the in-headset panel shows, on the PC screen.

FOR OFF-HEAD OPERATION. The Quest controllers are excellent 6-DOF input and
they need the headset awake to be tracked -- but nobody has to be wearing it.
Put the headset on a box facing the operator, hold the controllers, and read
the state HERE instead of through the lenses.

That inverts the assumption vr_safety_node is built on. Its rule -- an
independent observer e-stop is mandatory because "the operator cannot see the
arm" -- exists for a head-worn operator. Off-head, the operator is looking at
the same RViz window as everyone else. The interlock is NOT removed here; this
is a viewer, it publishes nothing. But the reason for the interlock is weaker
in this mode and that is worth saying out loud rather than quietly relying on.

    python3 scripts/vr_desk_monitor.py

Refreshes in place. Ctrl-C to stop.
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np

ESC = '\033['
CLR = ESC + '2J' + ESC + 'H'


def c(s, col):
    return '%s%sm%s%s0m' % (ESC, col, s, ESC)


GREEN, RED, YELL, DIM, BOLD = '32', '31', '33', '90', '1'


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Float64MultiArray, String
    import tf2_ros

    S = dict(bridge={}, lat=[], safety={}, mapper={}, gripper={},
             joy={}, valid={}, track=None, cmd={}, ee={})

    class M(Node):
        def __init__(self):
            super().__init__('vr_desk_monitor')
            self.tf_buf = tf2_ros.Buffer()
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_subscription(String, '/vr/bridge_status',
                                     lambda m: S.__setitem__('bridge', json.loads(m.data)), 10)
            self.create_subscription(Float64MultiArray, '/vr/latency',
                                     lambda m: S.__setitem__('lat', list(m.data)), 10)
            self.create_subscription(String, '/vr/safety',
                                     lambda m: S.__setitem__('safety', json.loads(m.data)), 10)
            self.create_subscription(Bool, '/vr/tracking_ok',
                                     lambda m: S.__setitem__('track', bool(m.data)), 10)
            for h in ('left', 'right'):
                self.create_subscription(String, '/vr/mapper_%s' % h,
                                         lambda m, h=h: S['mapper'].__setitem__(h, (time.monotonic(), json.loads(m.data))), 20)
                self.create_subscription(String, '/vr/gripper_%s' % h,
                                         lambda m, h=h: S['gripper'].__setitem__(h, json.loads(m.data)), 20)
                self.create_subscription(Joy, '/vr/controller_joy_%s' % h,
                                         lambda m, h=h: S['joy'].__setitem__(h, list(m.axes)), 50)
                self.create_subscription(Bool, '/vr/controller_valid_%s' % h,
                                         lambda m, h=h: S['valid'].__setitem__(h, bool(m.data)), 20)
                self.create_subscription(PoseStamped, '/master_arm_pose_%s' % h,
                                         lambda m, h=h: S['cmd'].__setitem__(h, time.monotonic()), 50)
            self.create_timer(0.1, self.tf)

        def tf(self):
            import rclpy.time
            for a in ('left', 'right'):
                try:
                    t = self.tf_buf.lookup_transform(
                        'world', '%s_end_effector_link' % a, rclpy.time.Time())
                    v = t.transform.translation
                    S['ee'][a] = [v.x, v.y, v.z]
                except Exception:                            # noqa: BLE001
                    pass

    rclpy.init()
    n = M()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    try:
        while True:
            time.sleep(0.35)
            out = []
            out.append(c('  SRL VR -- DESK MONITOR  (off-head operation)', BOLD))
            out.append(c('  the headset is a TRACKER on a box; read state here', DIM))
            out.append('')

            b = S['bridge']
            cl = b.get('clients', 0)
            out.append('  link      %s   %s Hz   frames %s   dropped %s'
                       % (c('CONNECTED', GREEN) if cl else c('NO CLIENT', RED),
                          b.get('rate_hz', '--'), b.get('frames', '--'),
                          b.get('dropped', '--')))
            if S['lat'] and len(S['lat']) > 3 and not math.isnan(S['lat'][1]):
                out.append('  latency   round trip %.1f ms   one-way est %.1f ms'
                           % (S['lat'][1], S['lat'][1] / 2 + S['lat'][3]))

            sf = S['safety']
            if sf.get('frozen'):
                out.append('  state     ' + c('FROZEN: %s' % sf.get('reason', ''), RED))
            else:
                out.append('  state     ' + c('LIVE', GREEN))
            out.append('')

            for h in ('left', 'right'):
                v = S['valid'].get(h)
                ax = S['joy'].get(h) or [0, 0, 0, 0]
                mp = S['mapper'].get(h)
                fresh = mp and (time.monotonic() - mp[0]) < 0.5
                d = mp[1] if mp else {}
                tag = (c('TRACKED', GREEN) if v is True else
                       c('OCCLUDED', RED) if v is False else c('unknown', DIM))
                cl_s = (c('CLUTCH IN ', GREEN) if (fresh and d.get('engaged'))
                        else c('clutch out', DIM))
                out.append('  %-5s  %s  %s  scale %-5s  grip %.2f  trig %.2f'
                           % (h.upper(), tag, cl_s,
                              d.get('scale', '--') if fresh else '--',
                              ax[1] if len(ax) > 1 else 0,
                              ax[0] if len(ax) > 0 else 0))
                lag = d.get('lag_m') if fresh else None
                extra = []
                if lag is not None:
                    extra.append('lag %5.1f mm%s' % (lag * 1000,
                                                     ' <-- OVER GATE' if lag > 0.030 else ''))
                if d.get('max_reengage_jump_m') is not None:
                    extra.append('worst re-engage %5.1f mm'
                                 % (d['max_reengage_jump_m'] * 1000))
                g = S['gripper'].get(h) or {}
                extra.append('gripper %.3f rad%s'
                             % (g.get('command_rad', 0.0),
                                ' LATCHED' if g.get('latched') else ''))
                if extra:
                    out.append('         ' + c('  '.join(extra), DIM))
                ee = S['ee'].get(h)
                if ee:
                    out.append('         ' + c('EE  x %+.3f  y %+.3f  z %+.3f'
                                               % tuple(ee), DIM))
            out.append('')
            out.append(c('  x = wearer right   y = forward   z = up', DIM))
            out.append(c('  Ctrl-C to stop.  This window publishes nothing.', DIM))
            sys.stdout.write(CLR + '\n'.join(out) + '\n')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
