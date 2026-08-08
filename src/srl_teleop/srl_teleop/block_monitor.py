#!/usr/bin/env python3
"""
block_monitor.py — every mechanism that can stop motion reports through here.

WHY. Four separate mechanisms have silently stopped all motion on this rig with
no error, and every one of them presented as dead hardware:

  1. the IK step guard rejecting 100% of solutions (reject_count == success_count)
  2. the pot validator's velocity gate holding the last good vector forever
  3. the e-stop latching on a stale master topic
  4. `pending` latching True after an unanswered /compute_ik call

The common failure is not the blocking - blocking is usually correct. It is
that blocking was INDISTINGUISHABLE FROM WORKING. So the rule here is:

    a) LOUD AND BY NAME the first time, and periodically after
    b) every blocker must DEGRADE, never latch permanently
    c) one published state, so there is a single place to look
    d) blocking for > 3 s is an ERROR. Sustained blocking is never normal.
    e) a startup self-test must prove the whole command path moves the arm

A blocker registers once and then calls block()/clear() every cycle. The
monitor does the logging, the escalation and the publishing, so no individual
blocker can forget to.

    self.blocks = BlockMonitor(self, "ik_follower_left")
    self.blocks.register("step_guard", "IK solution too far from current state",
                         recovery="slews toward it instead of discarding")
    ...
    if bad:  self.blocks.block("step_guard", "delta 0.62 rad > 0.35")
    else:    self.blocks.clear("step_guard")
"""
import json
import time

from std_msgs.msg import String

# Blocking longer than this is an ERROR, not a warning. Chosen because a
# human operator notices "the arm isn't moving" in about a second, and three
# seconds of silence is already long enough to start pulling cables.
ESCALATE_S = 3.0
FIRST_LOG_S = 0.0        # log immediately on the first block
REPEAT_S = 5.0
# A blocker not re-asserted for this long is abandoned, not held.
# Comfortably longer than any callback period in the system (the slowest
# asserting loop is 10 Hz) so a genuinely held block is never expired.
EXPIRE_S = 2.0


class _Blocker:
    def __init__(self, name, what, recovery, fatal_if_stuck):
        self.name = name
        self.what = what
        self.recovery = recovery
        self.fatal_if_stuck = fatal_if_stuck
        self.active = False
        self.since = None
        self.last_log = 0.0
        self.count = 0
        self.total_s = 0.0
        self.reason = ""
        self.escalated = False
        self.longest_s = 0.0
        # EXPIRED is a THIRD state, not a synonym for cleared. It means the
        # code that was asserting this blocker stopped running, so the real
        # condition is UNKNOWN. Reporting it as "clear" would tell every
        # consumer that a crashed unit is healthy, which is the exact
        # inversion this whole mechanism exists to prevent.
        self.expired = False
        self.expired_at = None
        self.last_assert = None

    def as_dict(self, now):
        held = (now - self.since) if self.active and self.since else 0.0
        return dict(name=self.name, active=self.active,
                    expired=self.expired,
                    unknown_for_s=(round(now - self.expired_at, 2)
                                   if self.expired and self.expired_at else 0.0),
                    held_s=round(held, 3), count=self.count,
                    total_s=round(self.total_s, 2),
                    longest_s=round(max(self.longest_s, held), 2),
                    reason=self.reason, recovery=self.recovery,
                    escalated=self.escalated)


class BlockMonitor:
    """One per node. Owns every blocker in that node."""

    def __init__(self, node, unit, topic="/blocking", escalate_s=ESCALATE_S,
                 publish_rate_hz=2.0):
        self.node = node
        self.unit = unit
        self.escalate_s = float(escalate_s)
        self._b = {}
        self.pub = node.create_publisher(String, topic, 10)
        node.create_timer(1.0 / publish_rate_hz, self._tick)
        self._never_moved_warned = False

    # ------------------------------------------------------------ registry
    def register(self, name, what, recovery, fatal_if_stuck=True):
        """Declare a blocker BEFORE it can fire.

        `recovery` is not documentation - it is the answer to "what gets the
        arm moving again", and a blocker that cannot answer it is a latch.
        """
        if not recovery:
            raise ValueError(
                "blocker %r has no recovery path. Every blocker must degrade, "
                "never latch permanently - see the module docstring." % name)
        self._b[name] = _Blocker(name, what, recovery, fatal_if_stuck)
        return self

    # -------------------------------------------------------------- state
    def block(self, name, reason=""):
        b = self._b.get(name)
        if b is None:
            raise KeyError("blocker %r was never registered" % name)
        now = time.monotonic()
        b.reason = str(reason)
        b.last_assert = now
        b.expired = False          # asserting again resolves the unknown
        b.expired_at = None
        if not b.active:
            b.active = True
            b.since = now
            b.count += 1
            b.escalated = False
            b.last_log = now
            self.node.get_logger().warn(
                "[BLOCKED:%s] %s — %s. Recovery: %s"
                % (b.name, b.what, reason or "no detail", b.recovery))
        else:
            held = now - b.since
            if held > self.escalate_s and not b.escalated:
                b.escalated = True
                self.node.get_logger().error(
                    "[BLOCKED:%s] STILL BLOCKING after %.1f s — %s. This is "
                    "not normal operation. Recovery: %s"
                    % (b.name, held, reason or b.what, b.recovery))
            elif now - b.last_log > REPEAT_S:
                b.last_log = now
                # NOT `lg = (error if escalated else warn); lg(...)`.
                # rclpy caches severity per CALL SITE, so one call site
                # emitting warn and later error raises "Logger severity
                # cannot be changed between calls" -- INSIDE the block
                # monitor's own timer. Every blocker escalates after 3 s, so
                # this would have thrown the first time anything blocked for
                # more than three seconds, in the one component whose job is
                # to make blocking visible. Verified to raise; the two-branch
                # form below is two call sites and does not.
                text = ("[BLOCKED:%s] held %.1f s — %s"
                        % (b.name, held, reason or b.what))
                if b.escalated:
                    self.node.get_logger().error(text)
                else:
                    self.node.get_logger().warn(text)

    def _expire_stale(self, now):
        """Auto-clear a blocker nobody is still asserting.

        THIS EXISTS TO KILL A PATTERN, NOT A BUG. Every blocker in this system
        is written as

            if <bad>: blocks.block(N); return
            blocks.clear(N)

        so the clear sits on the fall-through path of the very guard the block
        reports on. Whenever the guard also stops the callback from running --
        the master goes silent, an in-flight flag never resets, a callback
        stops being scheduled -- the clear becomes unreachable and the blocker
        latches ACTIVE forever. It has happened five times here, most recently
        inside the code written to catch the previous four.

        A held condition is re-asserted every cycle by design, so a blocker
        that has not been re-asserted for EXPIRE_S is not held; it is
        abandoned. Expiring it converts a silent permanent lie into a logged,
        self-correcting event. The expiry is logged loudly because an
        abandoned blocker still means the unit asserting it stopped running.
        """
        for b in self._b.values():
            if not b.active:
                continue
            last = getattr(b, "last_assert", None)
            if last is None or now - last <= EXPIRE_S:
                continue
            held = now - b.since if b.since else 0.0
            b.active = False
            b.since = None
            b.escalated = False
            # NOT cleared -- moved to EXPIRED, which every consumer must read
            # as "state unknown, do not proceed". Clearing it here would tell
            # the rest of the system that a crashed unit is healthy.
            b.expired = True
            b.expired_at = now
            self.node.get_logger().error(
                "[BLOCKED:%s] AUTO-EXPIRED after %.1f s with no re-assertion "
                "(held %.1f s). This is NOT an all-clear: whatever was "
                "asserting it STOPPED RUNNING, so the real condition is "
                "UNKNOWN and this unit reports state_unknown until the "
                "blocker is asserted or cleared again. Check that unit."
                % (b.name, now - last, held))

    def clear(self, name):
        b = self._b.get(name)
        if b is not None and b.expired:
            # An explicit clear from live code resolves the unknown: the unit
            # is demonstrably running again and says the condition is gone.
            b.expired = False
            b.expired_at = None
            b.last_assert = time.monotonic()
            self.node.get_logger().warn(
                "[BLOCKED:%s] expired state RESOLVED - the asserting unit is "
                "running again and reports the condition clear." % b.name)
        if b is None or not b.active:
            return
        held = time.monotonic() - (b.since or time.monotonic())
        b.total_s += held
        b.longest_s = max(b.longest_s, held)
        b.active = False
        b.since = None
        b.escalated = False
        self.node.get_logger().info(
            "[UNBLOCKED:%s] after %.2f s" % (b.name, held))

    def clear_all(self):
        for n in list(self._b):
            self.clear(n)

    # ------------------------------------------------------------ queries
    @property
    def blocked(self):
        # Expired counts as BLOCKED. "Nobody is asserting it any more" is not
        # evidence the condition went away; it is evidence that whatever was
        # watching stopped. Fail safe.
        return any(b.active or b.expired for b in self._b.values())

    @property
    def active_names(self):
        return [b.name for b in self._b.values() if b.active]

    @property
    def expired_names(self):
        return [b.name for b in self._b.values() if b.expired]

    @property
    def state_unknown(self):
        """True when any blocker's asserting unit has stopped reporting.

        Consumers deciding whether it is SAFE TO PROCEED must treat this as a
        refusal, never as an all-clear."""
        return bool(self.expired_names)

    def held_s(self, name):
        b = self._b.get(name)
        if b is None or not b.active or b.since is None:
            return 0.0
        return time.monotonic() - b.since

    def worst_held_s(self):
        return max([self.held_s(n) for n in self._b] or [0.0])

    # ------------------------------------------------------------ publish
    def _tick(self):
        now = time.monotonic()
        self._expire_stale(now)
        m = String()
        m.data = json.dumps(dict(
            unit=self.unit,
            blocked=self.blocked,
            active=self.active_names,
            expired=self.expired_names,
            state_unknown=self.state_unknown,
            worst_held_s=round(self.worst_held_s(), 3),
            escalated=[b.name for b in self._b.values() if b.escalated],
            blockers=[b.as_dict(now) for b in self._b.values()]))
        self.pub.publish(m)
        # Re-log every escalated blocker so it cannot scroll away.
        for b in self._b.values():
            if b.active and b.escalated and now - b.last_log > REPEAT_S:
                b.last_log = now
                self.node.get_logger().error(
                    "[BLOCKED:%s] still held, %.1f s total this episode"
                    % (b.name, now - b.since))
