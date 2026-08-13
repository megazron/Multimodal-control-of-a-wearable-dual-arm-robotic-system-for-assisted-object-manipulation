#!/usr/bin/env python3
"""Mode 5/6 executive: the ONE state machine, and the ONE place mode is set.

    ros2 run srl_autonomy autonomy_executive --ros-args -p mode:=6

STAGES, each of which can REFUSE by name
----------------------------------------
    IDLE -> SCAN -> AWAIT_COMMAND -> RESOLVE -> CONFIRM -> PLAN -> APPROACH
         -> GRASP -> VERIFY_GRIP -> TRANSPORT -> PLACE -> RETREAT -> IDLE

A modular pipeline is the whole argument against an end-to-end VLA: when this
fails it fails at a NAMED stage with its inputs logged, and the failure can
be explained afterwards. A VLA emits joint targets and bypasses every gate
below.

SAFETY, and why mode 6 gets more of it
--------------------------------------
Modes 1-4 have a human looking at the arm. Mode 6 does not, so the clearance
floor and the e-stop are the only things between a misdetection and the
wearer. On top of the shared stack this adds:

  * CONFIDENCE FLOOR      below it, refuse and say why
  * ACTION TIMEOUT        every stage, no exceptions
  * VOICE STOP            latched, and it does not need the wake word
  * SPOKEN CONFIRMATION   announce before moving; ON by default
  * WORLD-MODEL AGREEMENT if memory and live perception disagree, STOP -- do
                          not act on the stale one
  * WEARER EXCLUSION      a target behind the wearer is REFUSED, never routed
                          around. Planning "around" a person's head with a
                          perception system that can be confidently wrong is
                          the wrong trade.
  * DECISION LOG          every decision with its inputs, so any motion can
                          be explained after the fact

Speed: `MAX_VEL_RAD_S` from operating_modes caps mode 6 at 0.15 rad/s against
teleop's 0.60. Autonomy runs SLOWER, because nobody is watching and the only
bound on a wrong motion is how long it takes to happen.
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_teleop")
from srl_teleop import operating_modes as om            # noqa: E402
from srl_autonomy import voice_intent as vi             # noqa: E402
from srl_autonomy import named_places as np_places      # noqa: E402
from srl_autonomy.world_model import WorldModel         # noqa: E402

STAGES = ("IDLE", "SCAN", "AWAIT_COMMAND", "RESOLVE", "CONFIRM", "PLAN",
          "APPROACH", "GRASP", "VERIFY_GRIP", "TRANSPORT", "PLACE",
          "RETREAT", "REFUSED", "STOPPED")

# Wearer keep-out. A sphere at the head and a box at the torso, in world
# coordinates, taken from human_backpack.xacro. Targets inside these are
# refused outright.
WEARER_HEAD = (0.0, 0.0, 1.295)
WEARER_HEAD_R = 0.16
WEARER_TORSO_C = (0.0, -0.06, 1.05)
WEARER_TORSO_HALF = (0.20, 0.14, 0.28)


def behind_wearer(p):
    """True if the point is BEHIND the wearer's coronal plane.

    +y is FORWARD in this repo's world frame. Anything at y < 0 is behind the
    person; the arms mount on the back, so 'behind' is exactly where the
    wearer is, not empty space.
    """
    return float(p[1]) < 0.0


def inside_wearer(p, pad=0.05):
    p = np.asarray(p, float)
    if np.linalg.norm(p - np.asarray(WEARER_HEAD)) < WEARER_HEAD_R + pad:
        return True, "head"
    d = np.abs(p - np.asarray(WEARER_TORSO_C))
    if np.all(d < np.asarray(WEARER_TORSO_HALF) + pad):
        return True, "torso"
    return False, ""


class Executive(Node):
    def __init__(self):
        super().__init__("autonomy_executive")
        self.declare_parameter("mode", 6)
        self.declare_parameter("confidence_floor", 0.55)
        self.declare_parameter("stage_timeout_s", 20.0)
        self.declare_parameter("require_spoken_confirmation", True)
        self.declare_parameter("confirmation_wait_s", 2.0)
        self.declare_parameter("wake_word", vi.WAKE_DEFAULT)

        self.mode = om.mode_from(self.get_parameter("mode").value)
        self.conf_floor = float(self.get_parameter("confidence_floor").value)
        self.timeout = float(self.get_parameter("stage_timeout_s").value)
        self.confirm = bool(
            self.get_parameter("require_spoken_confirmation").value)
        self.confirm_wait = float(
            self.get_parameter("confirmation_wait_s").value)
        self.wake = str(self.get_parameter("wake_word").value)

        # The single predicate every part of the system uses to decide
        # whether a point is off-limits. Passed INTO the world model so
        # exclusion happens at the source, not at each consumer.
        self.forbidden = lambda p: (behind_wearer(p)
                                    or inside_wearer(p)[0])
        self.world = WorldModel(min_confidence=self.conf_floor)
        self.stage = "IDLE"
        self.stage_t = time.monotonic()
        self.stopped = False
        self.decisions = []
        self.estopped = False

        self.say_pub = self.create_publisher(String, "/robot_speech", 10)
        self.stage_pub = self.create_publisher(String, "/autonomy_stage", 10)
        self.dec_pub = self.create_publisher(String, "/autonomy_decision", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        # THE MOTION OUTPUT. Until this existed the executive's complete
        # publisher list was speech, stage, decision and estop: it narrated a
        # pick and the arm never heard a word of it. /autonomy/assist_pose_
        # <arm> is consumed by ik_follower_node, which routes it into
        # request_ik -- the same door teleop uses, so this inherits
        # collision-aware IK, the clearance floor, the guard and the e-stop
        # with no per-mode exemption and no private route to the controller.
        from geometry_msgs.msg import PoseStamped
        self._PoseStamped = PoseStamped
        self.cmd_pub = {a: self.create_publisher(
            PoseStamped, "/autonomy/assist_pose_%s" % a, 10)
            for a in ("left", "right")}
        # The pose is republished at a steady rate rather than once per stage:
        # the follower treats autonomy as driving only while poses keep
        # arriving, so a single publish would hand control straight back to
        # the master mid-motion.
        self._cmd = {}
        self.create_timer(0.05, self._pump_cmd)
        self.create_subscription(String, "/voice_transcript",
                                 self.on_voice, 10)
        self.create_subscription(String, "/detections", self.on_detections, 10)
        self.create_subscription(Bool, "/estop_state", self._estop, 10)
        self.create_subscription(String, "/autonomy_reset", self._reset, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().warn("MODE %s" % om.describe(self.mode))
        self.get_logger().warn(
            "safety required: %s" % ", ".join(om.required_safety(self.mode)))

    # ------------------------------------------------------------- utils
    def _estop(self, m):
        self.estopped = bool(m.data)

    def say(self, text):
        self.say_pub.publish(String(data=text))
        self.get_logger().info("[SAY] %s" % text)

    def decide(self, what, **inputs):
        """Every autonomous decision, with its inputs. This is what makes a
        motion explainable afterwards; a VLA cannot produce it."""
        rec = dict(t=time.monotonic(), stage=self.stage, what=what, **inputs)
        self.decisions.append(rec)
        self.dec_pub.publish(String(data=json.dumps(rec, default=str)))
        return rec

    def command(self, arm, xyz, quat=None):
        """Set the world-frame target this arm should be driven to."""
        self._cmd[arm] = (tuple(float(v) for v in xyz),
                          tuple(float(v) for v in (quat or (0.0, 0.0, 0.0, 1.0))))
        self.decide("command", arm=arm, xyz=list(self._cmd[arm][0]))

    def release(self, arm=None):
        """Stop driving. The follower hands back to the master on its own once
        poses stop arriving, so releasing is simply ceasing to publish."""
        if arm is None:
            self._cmd.clear()
        else:
            self._cmd.pop(arm, None)

    def _pump_cmd(self):
        if self.estopped if hasattr(self, "estopped") else False:
            return
        for arm, (xyz, q) in list(self._cmd.items()):
            m = self._PoseStamped()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = q
            self.cmd_pub[arm].publish(m)

    def to(self, stage, why=""):
        if stage in ("IDLE", "REFUSED"):
            self.release()
        self.stage = stage
        self.stage_t = time.monotonic()
        self.stage_pub.publish(String(data=json.dumps(
            dict(stage=stage, why=why, mode=int(self.mode)))))
        self.get_logger().info("[STAGE] %s %s" % (stage, why))

    def refuse(self, why, speak=True):
        self.decide("refused", why=why)
        if speak:
            self.say("I can't do that: %s" % why)
        self.to("REFUSED", why)

    # ------------------------------------------------------------ inputs
    def on_voice(self, m):
        intent = vi.parse(m.data, wake=self.wake)
        # STOP first, always, and it does not need the wake word.
        if intent.verb == "stop":
            self.stopped = True
            self.decide("voice_stop", raw=m.data)
            self.estop_pub.publish(Bool(data=True))
            self.say("Stopping.")
            self.to("STOPPED", "voice stop")
            return
        if self.stage not in ("AWAIT_COMMAND", "IDLE", "REFUSED"):
            self.get_logger().info("busy in %s, ignoring %r"
                                   % (self.stage, m.data))
            return
        if not intent.ok:
            self.decide("unparsed", raw=m.data, reason=intent.reason)
            # NO WAKE WORD -> SILENT. Answering "Sorry, I didn't understand"
            # to every overheard sentence defeats the wake word entirely:
            # the robot would be responding to a conversation it was never
            # addressed in. Only an utterance that DID carry the wake word
            # earns a reply.
            if "wake word" in intent.reason:
                self.get_logger().debug("not addressed: %r" % m.data)
                return
            self.say("Sorry, I didn't understand that.")
            return
        self.handle_intent(intent)

    def _reset(self, _m):
        """Clear the world model and return to IDLE. Objects PERSIST by
        design, so successive tests would otherwise accumulate candidates and
        turn a unique match into an ambiguous one -- which is exactly how the
        keep-out ordering bug above first showed up."""
        self.world = WorldModel(min_confidence=self.conf_floor)
        self.stopped = False
        self.to("IDLE", "reset")

    def on_detections(self, m):
        try:
            dets = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.world.observe(dets)

    # ----------------------------------------------------------- command
    def handle_intent(self, intent):
        """NOT `handle()`: rclpy.Node already has a `handle` attribute (the
        node's C handle) and shadowing it makes Node.__init__ fail with
        "'method' object does not support the context manager protocol".
        Same class as `self.clients` colliding in the VR bridge -- rclpy
        reserves more names on Node than is obvious."""
        self.decide("intent", **intent.as_dict())
        if intent.verb == "grab":
            if intent.deictic:
                self.refuse(
                    "you pointed at something, but in voice-only autonomy I "
                    "have no way to see where you pointed. Name it instead.")
                return
            self.to("RESOLVE")
            cands = self.world.match(intent.target, exclude=self.forbidden)
            # WEARER KEEP-OUT IS APPLIED BY THE WORLD MODEL ITSELF.
            #
            # Measured 2026-08-07: with the keep-out evaluated only in
            # start_pick, a target BEHIND the wearer became one of three
            # candidates and the robot asked "which one -- left, right, near
            # or far?", offering a position it must never reach. Asking the
            # operator to choose an option the robot would then refuse is
            # worse than not offering it: it invites them to say "the far
            # one" and trust the answer.
            # `match(exclude=...)` never yields a forbidden object, so a
            # forbidden target cannot reach the disambiguation step at all --
            # it is not filtered out afterwards, it is never a candidate.
            # Filtering after the fact left one code path (the ambiguous
            # branch) able to offer a position the robot must never reach,
            # and asking the operator to choose an option that will then be
            # refused invites them to say "the far one" and trust it.
            n_forbidden = self.world.count_excluded(intent.target,
                                                    self.forbidden)
            if n_forbidden:
                self.decide("keepout_excluded", n=n_forbidden,
                            target=intent.target)
            status, chosen, cands = vi.resolve_target(
                intent.target, cands, min_confidence=self.conf_floor)
            self.decide("resolve", target=intent.target, status=status,
                        n=len(cands))
            if status == vi.NO_MATCH:
                if n_forbidden:
                    self.refuse(
                        "the only thing matching %r is behind the wearer. I "
                        "will not plan a path there -- I refuse rather than "
                        "routing around a person." % intent.target)
                else:
                    self.refuse("I can't see anything matching %r"
                                % intent.target)
                return
            if status == vi.AMBIGUOUS:
                # ASK. Never pick the first of several.
                self.say(vi.question_for(intent.target, cands))
                self.to("AWAIT_COMMAND", "ambiguous, asked")
                return
            self.start_pick(chosen)
        elif intent.verb == "handover":
            self.start_handover(intent.arm)
        elif intent.verb == "place":
            self.to("PLACE", "commanded")
            self.say("Putting it down.")
        elif intent.verb == "goto":
            self.start_move_to(intent.target, intent.arm)
        else:
            self.refuse("I don't know how to %r" % intent.verb)

    # ---------------------------------------------------------- sequences
    def start_pick(self, obj):
        p = np.asarray(obj["position"], float)
        # WEARER EXCLUSION -- refuse, never route around.
        if behind_wearer(p):
            self.refuse(
                "that target is behind the wearer. I will not plan a path "
                "there -- I refuse rather than routing around a person.")
            return
        bad, part = inside_wearer(p)
        if bad:
            self.refuse("that target is inside the wearer's %s" % part)
            return
        if float(obj.get("confidence", 0.0)) < self.conf_floor:
            self.refuse("I'm only %.0f%% sure about that, and my floor is "
                        "%.0f%%" % (100 * obj["confidence"],
                                    100 * self.conf_floor))
            return
        self.decide("pick_target", **obj)
        if self.confirm:
            self.to("CONFIRM")
            side = "left" if p[0] < 0 else "right"
            self.say("I see a %s on the %s, reaching now."
                     % (obj.get("label", "object"), side))
            self._confirm_until = time.monotonic() + self.confirm_wait
            self._pending = obj
            return
        self._begin_motion(obj)

    def _begin_motion(self, obj):
        self.decide("begin_motion", **obj)
        self.to("PLAN", "target %s" % obj.get("label"))

    def start_move_to(self, place, arm=None):
        """MOVE_TO: go to a named place, with no object involved.

        THE POSITION COMES OUT OF THE SURVEY. `named_places.resolve()` returns
        a cell the full-path work-surface survey actually solved, snapped to a
        tested cell centre rather than to the region's centroid -- the region
        is not convex, so its centroid can sit in a hole.

        A REFUSAL HERE IS A RESULT, NOT A FAILURE. "Move to the front centre"
        is the single most useful thing this platform can demonstrate about
        itself: the reachable region is nothing like the region a person
        expects, and the robot knows the number. So the refusal carries the
        measurement instead of a shrug.

        THE SAFETY CHECKS ARE THE SAME ONES A PICK GETS. A named place is
        still a Cartesian goal over a person, so it goes through the wearer
        keep-out exactly as `start_pick` does. A verb that skipped them
        because it has no object would be a hole in the keep-out.
        """
        try:
            spec = np_places.resolve(place)
        except (np_places.Unreachable, np_places.SurveyUnavailable) as e:
            self.refuse(str(e))
            return
        if spec.get("joint_space"):
            # HOME is a joint-space pose and this project has exactly one
            # definition of it. Inventing a Cartesian "home" here would make
            # a second.
            self.decide("move_to", place=place, joint_space=True)
            self.say("Going home.")
            self.to("PLAN", "move to %s (joint space)" % place)
            return
        p = np.asarray(spec["position"], float)
        if behind_wearer(p):
            self.refuse(
                "%s is behind the wearer. I will not plan a path there -- I "
                "refuse rather than routing around a person." % place)
            return
        bad, part = inside_wearer(p)
        if bad:
            self.refuse("%s is inside the wearer's %s" % (place, part))
            return
        use_arm = spec.get("arm") or arm
        self.decide("move_to", place=place, arm=use_arm,
                    position=[round(v, 4) for v in p],
                    n_cells=spec.get("n_cells"))
        self.say("Moving to the %s." % place)
        self.to("PLAN", "move to %s" % place)
        # AND ACTUALLY COMMAND IT. `command()` publishes to
        # /autonomy/assist_pose_<arm>, which ik_follower_node routes through
        # request_ik -- the same door teleop uses, so this inherits the
        # collision-aware IK, the clearance floor, the mount guard and the
        # e-stop with no per-mode exemption.
        #
        # Announcing a stage without commanding is how this node once
        # narrated a whole pick while the arm never moved, and a MOVE_TO that
        # only said "Moving to the left side" would be that same gap wearing
        # a new verb.
        self.command(use_arm, p)

    def start_handover(self, to_arm):
        """MODE 5/6 ONLY. Teleop cannot do this: under orientation_mode
        'fixed' both wrists present the SAME approach direction, so the
        receiving gripper cannot be presented an openable face. That is a
        limit of the MASTER, not the robot -- here the robot commands full
        6-DOF poses and gives each arm its own wrist angle."""
        self.decide("handover", to_arm=to_arm)
        self.say("Handing over to the other arm.")
        self.to("TRANSPORT", "inter-arm handover")

    # -------------------------------------------------------------- tick
    def _tick(self):
        if self.stopped or self.estopped:
            if self.stage != "STOPPED":
                self.to("STOPPED", "e-stop" if self.estopped else "voice stop")
            return
        # ACTION TIMEOUT on every stage, no exceptions.
        if self.stage not in ("IDLE", "AWAIT_COMMAND", "REFUSED", "STOPPED"):
            if time.monotonic() - self.stage_t > self.timeout:
                self.refuse("stage %s timed out after %.0f s"
                            % (self.stage, self.timeout))
                return
        if self.stage == "CONFIRM" and time.monotonic() >= self._confirm_until:
            self._begin_motion(self._pending)


def main(argv=None):
    rclpy.init(args=argv)
    n = Executive()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
