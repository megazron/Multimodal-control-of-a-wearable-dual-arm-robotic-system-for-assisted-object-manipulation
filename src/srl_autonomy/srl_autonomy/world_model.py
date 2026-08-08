#!/usr/bin/env python3
"""World model for an EYE-IN-HAND camera: scan, remember, re-verify, drop.

The Gen3 Vision module is at the WRIST. It moves with the arm, so:

  * nothing is visible until the arm looks at it;
  * whatever the arm is manipulating is exactly what the camera stops
    seeing, because the gripper is in front of it;
  * an object seen once and then left behind is remembered, not observed.

So the model has to persist detections in the WORLD frame and be honest about
their age. All three of the following are different states and are kept
apart, because collapsing them is how a stale pose gets acted on:

    FRESH      seen within `fresh_s`
    STALE      remembered, not currently observable
    CONFLICT   currently observable and NOT seen -> the world disagrees with
               perception, and the correct action is to STOP

DUAL-ARM OBSERVATION -- the SRL-specific part
---------------------------------------------
Two arms means two wrist cameras. One arm can WATCH while the other
manipulates, which a single-arm eye-in-hand system cannot do at all: it must
choose between seeing and doing. `observer_arm()` picks the arm that is not
executing, and `expected_visible()` is evaluated from THAT arm's camera pose.
This is what makes re-verification possible during an approach rather than
only before it.

WHY "DROP ON FAILED RE-CONFIRMATION" AND NOT "KEEP WITH LOW CONFIDENCE"
-----------------------------------------------------------------------
A remembered object that is no longer there is indistinguishable, downstream,
from one that is -- the pose is just as precise and just as wrong. The
project has been bitten by exactly this shape before (a frozen
/real/joint_states reading as live, a dead pot reading as a steady angle), so
an object that fails re-confirmation is REMOVED, and the caller is told which
and why.
"""
import math
import time

import numpy as np

FRESH = "fresh"
STALE = "stale"
CONFLICT = "conflict"


class Obj:
    def __init__(self, oid, label, position, confidence, t, source="scan"):
        self.id = oid
        self.label = label
        self.position = np.asarray(position, float)
        self.confidence = float(confidence)
        self.first_seen = t
        self.last_seen = t
        self.misses = 0
        self.source = source

    def as_dict(self):
        return dict(id=self.id, label=self.label,
                    position=[round(float(v), 4) for v in self.position],
                    confidence=round(self.confidence, 3),
                    last_seen=self.last_seen, misses=self.misses)


class WorldModel:
    def __init__(self, fresh_s=2.0, merge_m=0.06, max_misses=2,
                 min_confidence=0.40, clock=time.monotonic):
        self.fresh_s = float(fresh_s)
        self.merge_m = float(merge_m)
        self.max_misses = int(max_misses)
        self.min_confidence = float(min_confidence)
        self.clock = clock
        self.objects = {}
        self._next = 1
        self.log = []

    # ------------------------------------------------------------ update
    def observe(self, detections, source="scan"):
        """Fold a batch of WORLD-frame detections in. Returns (added, merged).

        `detections`: [{label, position(xyz, world), confidence}]. Positions
        must ALREADY be transformed via TF -- doing it here would hide which
        camera pose produced them, and a detection is only as good as the TF
        at the instant the image was taken.
        """
        t = self.clock()
        added, merged = [], []
        for d in detections or []:
            c = float(d.get("confidence", 0.0))
            if c < self.min_confidence:
                self.log.append(("reject_low_conf", d.get("label"), c, t))
                continue
            p = np.asarray(d["position"], float)
            hit = self._nearest(d.get("label"), p)
            if hit is not None:
                hit.position = 0.5 * (hit.position + p)
                hit.confidence = max(hit.confidence, c)
                hit.last_seen = t
                hit.misses = 0
                merged.append(hit.id)
            else:
                o = Obj(self._next, str(d.get("label", "?")), p, c, t, source)
                self.objects[self._next] = o
                added.append(self._next)
                self._next += 1
        return added, merged

    def _nearest(self, label, p):
        best, bd = None, self.merge_m
        for o in self.objects.values():
            if label is not None and o.label != label:
                continue
            d = float(np.linalg.norm(o.position - p))
            if d < bd:
                best, bd = o, d
        return best

    # -------------------------------------------------------- re-verify
    def reverify(self, seen_ids, expected_ids):
        """An object EXPECTED to be visible and NOT seen is a conflict.

        Returns (dropped, conflicts). `expected_ids` must come from the
        observing camera's actual pose, not from a guess -- calling
        everything expected would drop the whole model the moment the arm
        looks away.
        """
        t = self.clock()
        dropped, conflicts = [], []
        for oid in list(expected_ids or ()):
            o = self.objects.get(oid)
            if o is None:
                continue
            if oid in (seen_ids or ()):
                o.last_seen = t
                o.misses = 0
                continue
            o.misses += 1
            conflicts.append(oid)
            if o.misses >= self.max_misses:
                self.log.append(("dropped", o.label, o.misses, t))
                dropped.append(self.objects.pop(oid).as_dict())
        return dropped, conflicts

    def state(self, oid, expected_visible=False):
        o = self.objects.get(oid)
        if o is None:
            return None
        fresh = (self.clock() - o.last_seen) <= self.fresh_s
        if fresh:
            return FRESH
        return CONFLICT if expected_visible else STALE

    # --------------------------------------------------------- geometry
    @staticmethod
    def observer_arm(executing_arm):
        """The arm that is NOT executing watches. A single-arm eye-in-hand
        system has to choose between seeing and doing; two arms do not."""
        return "right" if executing_arm == "left" else "left"

    @staticmethod
    def expected_visible(cam_pos, cam_axis, obj_pos, hfov_deg=60.0,
                         max_range=1.2, min_range=0.10):
        """Would this object be in that camera's frustum right now?

        Deliberately conservative: an object judged NOT expected simply is
        not re-verified, whereas a false 'expected' turns a look-away into a
        conflict and stops the robot for nothing.
        """
        v = np.asarray(obj_pos, float) - np.asarray(cam_pos, float)
        d = float(np.linalg.norm(v))
        if d < min_range or d > max_range:
            return False
        a = np.asarray(cam_axis, float)
        n = float(np.linalg.norm(a))
        if n < 1e-9:
            return False
        cos = float(np.dot(v / d, a / n))
        return cos > math.cos(math.radians(hfov_deg / 2.0))

    def snapshot(self):
        return [o.as_dict() for o in self.objects.values()]

    def count_excluded(self, description, exclude):
        """How many matches were withheld because `exclude` forbade them."""
        if exclude is None:
            return 0
        return len(self._raw_match(description)) - len(
            self.match(description, exclude=exclude))

    def _raw_match(self, description):
        return self.match(description, exclude=None)

    def match(self, description, exclude=None):
        """Everything whose label contains every content word asked for.

        `exclude(position) -> bool` removes objects at the SOURCE. A caller
        that has to remember to filter will eventually forget on one path,
        and the path that forgot here was the ambiguous branch -- which then
        offered the operator a target behind the wearer as a choice.
        """
        want = set(str(description or "").lower().split())
        want -= {"the", "a", "an", "object", "thing", "one"}
        out = []
        for o in self.objects.values():
            have = set(o.label.lower().split())
            if not want or want <= have:
                if exclude is not None and exclude(o.position):
                    continue
                out.append(dict(o.as_dict(), position=list(o.position)))
        return out


def scan_waypoints(arm, centre=(0.0, 0.35, 0.95), n=5, radius=0.18,
                   side_offset=0.30):
    """A scripted sweep for the SCAN phase, before anything moves.

    Returned in the world frame. The sweep exists because an eye-in-hand
    camera sees nothing until it is pointed: without a scan, the first thing
    the robot knows about the scene is whatever happens to be under the
    gripper when a command arrives.
    """
    c = np.asarray(centre, float)
    side = 1.0 if arm == "left" else -1.0
    pts = []
    for k in range(n):
        f = (k / max(n - 1, 1)) - 0.5
        # Centred on the arm's OWN side, never on the centreline. Measured
        # 2026-08-07: over a 7x3x3 frontal grid, 0 of 63 cells were reachable
        # by both arms and the entire centreline (x in [-0.2, +0.2]) was
        # reachable by NEITHER. A sweep through the middle would spend its
        # waypoints where the arm cannot go, and the first waypoint of each
        # arm's sweep came out identical -- both exactly at x = 0.
        pts.append((float(c[0] + side * (side_offset + radius * f)),
                    float(c[1] + 0.10 * math.cos(math.pi * f)),
                    float(c[2] + 0.12 * abs(f))))
    return pts
