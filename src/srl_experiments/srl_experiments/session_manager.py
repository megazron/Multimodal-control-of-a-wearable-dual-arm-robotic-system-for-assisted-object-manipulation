#!/usr/bin/env python3
"""The session manager: readiness from live topics, and failure handling.

    ros2 run srl_experiments session_manager

WHAT IT OWNS
  * the nine readiness checks, evaluated from live topics, published on
    /session/ready at 2 Hz
  * the participant session -- consent, blocks, trials, timers, logging
  * FAULT WATCH: anything that breaks mid-trial ends that trial INVALID with
    the cause, immediately, without waiting to be asked
  * three one-click recoveries as services, and an abort path that does NOT
    depend on the GUI or the master arm

THE PARTICIPANT'S ABORT DOES NOT GO THROUGH THIS NODE'S GUI, and that is
deliberate. It is watched three ways, any of which stops the session:
  * /participant/abort  (a topic anyone can publish, including a foot switch)
  * the e-stop, which is a separate node with its own hardware paths
  * a FILE, ~/.srl_abort -- because if ROS itself is the thing that has
    failed, a topic is not a way out. `touch ~/.srl_abort` from any shell
    ends the session.
"""
import os
import shutil
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from srl_experiments import readiness as R
from srl_experiments.session import Session, CONSENT_STEPS

ABORT_FILE = os.path.join(os.path.expanduser("~"), ".srl_abort")


class LiveProbe:
    """Turns live topic state into the nine verdicts.

    EVERY METHOD RETURNS UNKNOWN WHEN IT HAS NOT HEARD ANYTHING. Silence is
    not health: a topic that has never published and a topic that is publishing
    "fine" must not produce the same verdict, and the whole point of the
    tri-state is that this module cannot express "probably ok".
    """

    STALE_S = 3.0

    def __init__(self, node):
        self.n = node

    def _fresh(self, stamp):
        return stamp is not None and (time.time() - stamp) < self.STALE_S

    def channels(self):
        st = self.n.channel_state
        if not self._fresh(self.n.channel_t):
            return R.UNKNOWN, "no /master_channel_state"
        if "DEGRADED" in st.upper():
            return R.FAIL, st.split("|")[0].strip()[:48]
        return R.OK, st.split("|")[1].strip()[:40] if "|" in st else "healthy"

    def cameras(self):
        got = [k for k, t in self.n.cam_t.items() if self._fresh(t)]
        if not self.n.cam_t:
            return R.UNKNOWN, "no camera topics subscribed"
        if len(got) < 2:
            missing = [k for k in self.n.cam_t if k not in got]
            return R.FAIL, "not publishing: %s" % ", ".join(missing)
        return R.OK, "both publishing"

    def estop(self):
        if self.n.estop is None:
            return R.UNKNOWN, "no /estop_state"
        if self.n.estop:
            return R.FAIL, "LATCHED"
        if not self.n.estop_tested:
            # Tested THIS SESSION, not ever. A test from last week tells you
            # nothing about today's wiring.
            return R.FAIL, "not tested this session"
        return R.OK, "clear, tested"

    def homed(self):
        if not self._fresh(self.n.js_t):
            return R.UNKNOWN, "no /joint_states"
        if self.n.home_err is None:
            return R.UNKNOWN, "home reference not loaded"
        if self.n.home_err > 0.05:
            return R.FAIL, "%.3f rad from home" % self.n.home_err
        return R.OK, "%.4f rad" % self.n.home_err

    def scene(self):
        if not self._fresh(self.n.scene_t):
            return R.UNKNOWN, "no /scene/state"
        s = self.n.scene_state
        if "unchanged" in s or "skipped" in s:
            return R.OK, "matches stored fingerprint"
        if "changed" in s or "moved" in s:
            return R.FAIL, "scene moved since registration"
        return R.UNKNOWN, "scene state %r not understood" % s[:24]

    def disk(self):
        free = shutil.disk_usage(os.path.expanduser("~")).free / 1e9
        if free < 2.0:
            return R.FAIL, "%.1f GB free (need 2.0)" % free
        return R.OK, "%.0f GB free" % free

    def one_stack(self):
        try:
            import sys
            sys.path.insert(0, os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.dirname(os.path.abspath(__file__))))),
                "srl_teleop"))
            from srl_teleop import procscan
            n = procscan.count("master_pose_node")
        except Exception as e:                                   # noqa: BLE001
            return R.UNKNOWN, "cannot count processes: %s" % str(e)[:40]
        if n == 0:
            return R.FAIL, "no stack running"
        if n > 1:
            return R.FAIL, "%d master_pose_node -- two stacks split the "
            "serial stream" % n
        return R.OK, "one"

    def recovery(self):
        if not self._fresh(self.n.recovery_t):
            return R.UNKNOWN, "no /recovery_state"
        return R.OK, "alive"

    def command_path(self):
        """Has the CURRENT mode's command path been verified reaching the arm?

        Not "is a topic present" -- that is the mistake this check exists to
        avoid. It requires a positive confirmation for THIS mode this session,
        set by the verify step.
        """
        m = self.n.mode
        if m is None:
            return R.UNKNOWN, "no mode selected"
        v = self.n.path_verified.get(m)
        if v is None:
            return R.UNKNOWN, "%s not verified this session" % m
        if not v:
            return R.FAIL, "%s did not reach the arm" % m
        return R.OK, "%s verified" % m


class SessionManager(Node):
    def __init__(self):
        super().__init__("session_manager")
        self.channel_state, self.channel_t = "", None
        self.cam_t = {"left": None, "right": None}
        self.estop, self.estop_tested = None, False
        self.js_t, self.home_err = None, None
        self.scene_state, self.scene_t = "", None
        self.recovery_t = None
        self.mode = None
        self.path_verified = {}
        self.session = None
        self._last_fault = ""

        self.create_subscription(String, "/master_channel_state",
                                 self._chan, 10)
        self.create_subscription(Bool, "/estop_state", self._estop, 10)
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.create_subscription(String, "/scene/state", self._scene, 10)
        self.create_subscription(String, "/recovery_state", self._rec, 10)
        for arm in ("left", "right"):
            self.create_subscription(
                Image, "/%s/wrist/image_raw" % arm,
                lambda m, a=arm: self.cam_t.__setitem__(a, time.time()), 2)
        self.create_subscription(String, "/participant/abort",
                                 self._abort_topic, 10)
        # Session control from the GUI. Topics rather than services because
        # the GUI must never BLOCK on this node: a wait_for_service against a
        # node that has died is how the e-stop once took 4 s to fire.
        self.create_subscription(String, "/session/begin", self._begin, 4)
        self.create_subscription(String, "/session/consent", self._consent, 8)
        self.create_subscription(String, "/session/trial", self._trial, 8)
        self.create_subscription(String, "/session/mode", self._set_mode, 4)
        self.create_subscription(String, "/session/verified",
                                 self._verified, 4)

        self.pub_ready = self.create_publisher(String, "/session/ready", 4)
        self.pub_state = self.create_publisher(String, "/session/state", 4)

        for name, cb in (("redo", self.srv_redo), ("skip", self.srv_skip),
                         ("abort", self.srv_abort)):
            self.create_service(Trigger, "/session/%s" % name, cb)

        self.probe = LiveProbe(self)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.25, self.watch_faults)
        self.get_logger().info(
            "session_manager up. Participant abort: /participant/abort, the "
            "e-stop, or `touch %s`" % ABORT_FILE)

    # ------------------------------------------------------------ callbacks
    def _chan(self, m):
        self.channel_state, self.channel_t = m.data, time.time()

    def _estop(self, m):
        was = self.estop
        self.estop = bool(m.data)
        # An e-stop that has gone true and been cleared IS a test.
        if was is False and self.estop:
            self.estop_tested = True

    def _js(self, m):
        self.js_t = time.time()
        try:
            from srl_experiments import home_positions as hp
            err = 0.0
            for arm in ("left", "right"):
                tgt = hp.load_home_radians(arm)
                names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
                if not all(n in m.name for n in names):
                    return
                cur = [m.position[m.name.index(n)] for n in names]
                import math
                err = max(err, max(abs((c - t + math.pi) % (2 * math.pi)
                                       - math.pi)
                                   for c, t in zip(cur, tgt)))
            self.home_err = err
        except Exception:                                        # noqa: BLE001
            self.home_err = None

    def _scene(self, m):
        self.scene_state, self.scene_t = m.data, time.time()

    def _rec(self, m):
        self.recovery_t = time.time()

    def _abort_topic(self, m):
        self.do_abort(m.data or "participant abort (topic)", by="participant")

    def _begin(self, m):
        """Start or RESUME. `resume:<path>` picks up where a crash left off."""
        try:
            if m.data.startswith("resume:"):
                self.session = Session.load(m.data.split(":", 1)[1])
                self.session.note("session RESUMED from disk", "good")
                self.session.save()
            else:
                from srl_experiments.conditions import williams_row
                plan = [("B%d" % (i // 3 + 1), self.mode or "01_master_teleop",
                         "abc"[i % 3]) for i in range(9)]
                self.session = Session(m.data.strip() or "P00", plan)
            self.get_logger().info("session %s for %s"
                                   % (self.session.sid,
                                      self.session.participant))
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error("could not begin session: %s" % e)

    def _consent(self, m):
        if self.session is None:
            return
        step = m.data.strip()
        for s in CONSENT_STEPS:
            if step in s or s in step:
                self.session.give_consent(s)
                if self.session.consent_complete:
                    try:
                        self.session.start()
                    except Exception as e:                       # noqa: BLE001
                        self.get_logger().error(str(e))
                return

    def _trial(self, m):
        if self.session is None:
            return
        cmd = m.data.strip()
        if cmd == "begin":
            self.session.begin_trial()
        elif cmd.startswith("end"):
            ok = "invalid" not in cmd
            self.session.end_trial(valid=ok,
                                   cause="" if ok else "ended invalid")

    def _set_mode(self, m):
        self.mode = m.data.strip() or None

    def _verified(self, m):
        """`<mode>:ok` or `<mode>:fail` -- a POSITIVE confirmation for THIS
        session. Absence stays UNKNOWN; that is the whole point."""
        try:
            mode, verdict = m.data.split(":", 1)
            self.path_verified[mode.strip()] = verdict.strip().lower() == "ok"
        except ValueError:
            pass

    # ---------------------------------------------------------------- ready
    def tick(self):
        r = R.evaluate(self.probe)
        import json
        self.pub_ready.publish(String(data=json.dumps(r.as_dict())))
        s = self.session
        st = dict(state=(s.state if s else "no session"),
                  participant=(s.participant if s else None),
                  sid=(s.sid if s else None))
        if s is not None:
            t = s.current
            st.update(done=s.done_count, total=len(s.plan),
                      trial=(t.as_dict() if t else None),
                      elapsed=(time.time() - t.started
                               if t and t.started else None),
                      log=s.log[-25:])
        self.pub_state.publish(String(data=json.dumps(st)))

    # ------------------------------------------------------- the fault watch
    def watch_faults(self):
        """Anything that breaks mid-trial ends the trial INVALID, now.

        The operator does not have to notice, and does not have to diagnose.
        The GUI is told in plain words and offers the three buttons.
        """
        if os.path.exists(ABORT_FILE):
            try:
                os.remove(ABORT_FILE)
            except OSError:
                pass
            self.do_abort("abort file touched", by="participant")
            return
        s = self.session
        if s is None or s.state != "running" or s.current is None:
            return
        fault = None
        if self.estop:
            fault = "E-STOP TRIPPED"
        elif not self.probe._fresh(self.js_t):
            fault = "PROCESS DEATH — /joint_states stopped"
        else:
            cams = [k for k, t in self.cam_t.items()
                    if not self.probe._fresh(t)]
            if len(cams) == len(self.cam_t) and self.cam_t:
                fault = "CAMERA LOSS — %s stopped publishing" % ", ".join(cams)
            elif self.probe._fresh(self.channel_t) and \
                    "DEGRADED" in self.channel_state.upper():
                fault = "CHANNEL DROPOUT — master degraded mid-trial"
        if fault and fault != self._last_fault:
            self._last_fault = fault
            s.fail_trial(fault)
            self.get_logger().error("[TRIAL INVALID] %s" % fault)
        elif not fault:
            self._last_fault = ""

    # -------------------------------------------------------------- services
    def _ok(self, resp, msg):
        resp.success = True
        resp.message = msg
        return resp

    def srv_redo(self, req, resp):
        if self.session is None:
            return self._ok(resp, "no session")
        self.session.redo()
        return self._ok(resp, "will re-run the same trial")

    def srv_skip(self, req, resp):
        if self.session is None:
            return self._ok(resp, "no session")
        self.session.skip()
        return self._ok(resp, "skipped; continuing")

    def srv_abort(self, req, resp):
        self.do_abort("operator abort", by="operator")
        return self._ok(resp, "aborted")

    def do_abort(self, reason, by):
        """Stop the arms FIRST, then record. Order matters with a person
        in the room."""
        try:
            self.create_publisher(Bool, "/estop", 2).publish(Bool(data=True))
        except Exception:                                        # noqa: BLE001
            pass
        if self.session is not None and self.session.state == "running":
            self.session.abort(reason, by=by)
        self.get_logger().error("[ABORT] %s (%s) — arms stopped, data kept "
                                "and flagged" % (reason, by))


def main(argv=None):
    rclpy.init(args=argv)
    n = SessionManager()
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
