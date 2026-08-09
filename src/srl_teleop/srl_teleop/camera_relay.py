#!/usr/bin/env python3
"""Wrist camera relay: what the gripper sees, shown to the operator.

WHY THIS IS THE LARGEST INFORMATION GAP IN THIS SYSTEM. When operator and
wearer are the same person the operator can simply look at the task. When they
are different people the operator has no view of the workspace at all, and the
arms are on somebody else's back. Everything else in the interface tells them
about the ROBOT; this is the only thing that tells them about the WORLD.

IT SUBSCRIBES, IT NEVER OPENS A DEVICE. A camera DEVICE has exactly one owner
and a second opener corrupts both, which is the same fault class as two
readers on one serial port. A camera TOPIC may have as many subscribers as it
likes, and that is the point of publish/subscribe rather than a problem. The
device owner here is the vendor vision driver; this is one more subscriber
beside the detector.

STALENESS IS THE WHOLE DESIGN. A relay that keeps painting the last frame it
received is worse than one showing nothing, because a frozen picture of a
workspace is indistinguishable from a live picture of a workspace that is not
moving. That is the exact failure this project has hit repeatedly: the frozen
/real/joint_states that read as "holding", the dead j7 pot whose zero variance
read as "steady", the master republishing stale data with fresh timestamps.

So a frame is shown as LIVE only while it is younger than `stale_after_s`.
Past that the widget says NO SIGNAL and gives the age. It never dims or
freezes a frame and calls it current.
"""
import time

STALE_AFTER_S = 0.5          # older than this is not a live view
DEAD_AFTER_S = 3.0           # older than this, stop claiming a camera exists

LIVE, STALE, DEAD, ABSENT = "live", "stale", "dead", "absent"


class ChannelState:
    """Rolling state for one arm's camera. No ROS handles: unit-testable."""

    def __init__(self, stale_after_s=STALE_AFTER_S, dead_after_s=DEAD_AFTER_S):
        self.stale_after = stale_after_s
        self.dead_after = dead_after_s
        self.last_t = None          # monotonic arrival of the last frame
        self.n = 0
        self._times = []
        self.width = self.height = 0
        self.encoding = ""
        self.ever = False

    def on_frame(self, width, height, encoding, now=None):
        now = time.monotonic() if now is None else now
        self.last_t = now
        self.n += 1
        self.ever = True
        self.width, self.height, self.encoding = width, height, encoding
        self._times.append(now)
        if len(self._times) > 60:
            self._times = self._times[-60:]

    def hz(self, now=None):
        """Measured arrival rate, not a configured one.

        A configured rate is a claim about intent; the operator needs the rate
        the frames are actually arriving at, because a camera delivering 2 Hz
        while claiming 30 is the case that matters.
        """
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 1e-6 else 0.0

    def age(self, now=None):
        if self.last_t is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_t

    def state(self, now=None):
        if not self.ever:
            return ABSENT
        a = self.age(now)
        if a is None:
            return ABSENT
        if a > self.dead_after:
            return DEAD
        if a > self.stale_after:
            return STALE
        return LIVE

    def caption(self, now=None):
        """One line, and it must never imply a live view when there is none."""
        s = self.state(now)
        if s == ABSENT:
            return "NO CAMERA -- no frame has ever arrived"
        a = self.age(now)
        if s == DEAD:
            return "NO SIGNAL -- last frame %.1f s ago" % a
        if s == STALE:
            return "STALE -- last frame %.2f s ago" % a
        return "live  %.1f Hz  %dx%d" % (self.hz(now), self.width, self.height)

    def show_image(self, now=None):
        """Whether the last frame may be PAINTED at all.

        False past `stale_after`. The alternative -- painting it greyed or
        dimmed -- still puts a picture of the workspace in front of the
        operator, and under time pressure a dim picture is read as a picture.
        """
        return self.state(now) == LIVE
