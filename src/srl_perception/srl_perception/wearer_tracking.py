#!/usr/bin/env python3
"""wearer_tracking.py -- turn observed body keypoints into a wearer model that
is never LESS conservative than the mannequin.

WHAT THIS FILE IS FOR. `wearer_posture.wearer_model()` returns the wearer as a
list of world-frame primitives -- a torso box, a head, neck, hips, two thighs
and, per side, an upper arm, a forearm and a hand. Every clearance figure in
this project is measured against those primitives, and they describe a
MANNEQUIN: fixed radii, fixed lengths, and a posture chosen by an environment
variable. A real person is a different size and stands differently, and 17 kg
of arms is going to be worn by more than one of them.

This module replaces those primitives, ONE AT A TIME, with primitives measured
from a camera -- and it is written around the fact that doing so is dangerous.

    A WRONG BODY MODEL IS MORE DANGEROUS THAN NO BODY MODEL.

A body estimate 100 mm further away than the truth does not fail loudly. It
produces confident clearance numbers that are wrong in the direction that puts
metal into somebody's chest. So the substitution obeys one rule, and the rule
is the whole design:

    THE CAMERA MAY ONLY EVER MAKE THE WEARER BIGGER, NEVER SMALLER.

Implemented in `fuse()` as: for each part, keep whichever of the tracked and
mannequin primitive is CLOSER to the robot. A hallucinated retreat therefore
buys nothing; jitter costs workspace and never safety; and if the tracker dies
the model is exactly what it is today. There is no mode switch, because a mode
switch is a thing that can be in the wrong mode.

FOUR GATES, and each of them is exercised by injection in
test_wearer_tracking.py rather than argued about here:

  1. CONFIDENCE, per segment. A segment whose endpoints are not clearly seen
     reverts to the mannequin ON ITS OWN. This is not cosmetic: measured on
     real photographs, a person with folded arms gives shoulder and hip
     visibility 1.00 while the elbows read 0.09-0.11 -- and the "upper arm"
     it reports is 0.107 m long, which is not an arm. The confidence and the
     nonsense arrive together, which is what makes the gate work.
  2. PLAUSIBILITY, per segment. A segment outside anatomical range is refused
     whatever its confidence, because confidence is the model's opinion of
     its own output and this is a fact about human beings.
  3. SPEED, per endpoint. A joint that moved faster than a person can move
     did not move; the detector jumped to somebody else, or to a coat.
  4. AGE. Past the staleness limit the estimate contributes nothing. A stale
     body pose cannot be told from a live pose of a person standing still --
     CLAUDE.md's "data fresh but never changes" row, with the arms powered.

NO ROS, NO MEDIAPIPE, NO CAMERA. Everything arrives as plain numbers so the
gates can be driven with a deliberately broken body and required to refuse.
"""
import math

# ---------------------------------------------------------------- geometry
# The parts this module can replace, and the mannequin part each one shadows.
# Names match `wearer_posture.wearer_model()` exactly, because the fused list
# is consumed by the same code -- a name that does not match is a part that
# silently keeps the mannequin for ever.
TRACKED_PARTS = ("torso", "head", "hips",
                 "L-upperarm", "L-forearm", "L-hand",
                 "R-upperarm", "R-forearm", "R-hand")

# Anatomical ranges, in metres. Deliberately WIDE -- this gate exists to
# reject a detection that is not a person, not to enforce an average.
# Sources: the 5th-95th centile adult range, rounded outwards.
PLAUSIBLE = {
    "upperarm": (0.20, 0.42),
    "forearm": (0.17, 0.38),
    # THE PALM, NOT THE HAND. The detector's hand landmark is the index
    # knuckle, so the measurable segment is wrist-to-knuckle -- about 80 to
    # 110 mm, not the 180 mm of a whole hand. Checking it against a hand
    # range rejected BOTH hands on the first real photograph, with the
    # perfectly true message "0.079 m is not a hand". It was not a hand; it
    # was a palm, and the gate was right about the number and wrong about the
    # question.
    "palm": (0.055, 0.145),
    "shoulder_width": (0.28, 0.55),
    "torso_height": (0.35, 0.70),
}

# Wrist to fingertip, as a multiple of wrist to index knuckle. The hand has
# to be modelled past the knuckle or the fingers are outside the collision
# model, and the fingers are the part of a person a gripper reaches first.
# 1.9 is the ratio at the LONG end of the adult range, chosen deliberately:
# this number decides how big the hand is, and for a clearance model the
# generous end is the safe end. It is a RATIO, not a measurement -- the palm
# is measured, the extension is assumed.
HAND_EXTENSION = 1.9

# How fast a joint can move, m/s. A wrist in a fast reach does ~2 m/s; 6 m/s
# is a punch. Anything past this is the detector changing its mind about
# which object is a person, not a person moving.
MAX_JOINT_SPEED = 6.0

# Below this, a segment is not used. 0.5 is MediaPipe's own "present and
# visible" boundary; the folded-arms case measured 0.09-0.22 and the clean
# standing cases measured 0.93-1.00, so the threshold sits in a real gap and
# not in the middle of a distribution.
MIN_SEGMENT_CONF = 0.5

# Past this the estimate is not used at all.
MAX_AGE_S = 0.30

# Every tracked cylinder is inflated by this before it is compared with the
# mannequin. A skeleton is a line; a person has a limb around it, and the
# detector gives the bone, not the sleeve.
LIMB_PAD_M = 0.055
TORSO_PAD_M = 0.06


class Segment:
    """One measured body part: a primitive, plus why it should be believed."""

    def __init__(self, name, kind, dims, centre, rpy, conf, reasons=None):
        self.name = name
        self.kind = kind
        self.dims = tuple(dims)
        self.centre = tuple(centre)
        self.rpy = tuple(rpy)
        self.conf = float(conf)
        self.reasons = list(reasons or [])

    @property
    def usable(self):
        return self.conf >= MIN_SEGMENT_CONF and not self.reasons

    def as_primitive(self):
        return (self.name, self.kind, self.dims, self.centre, self.rpy)

    def __repr__(self):
        return "Segment(%s conf=%.2f%s)" % (
            self.name, self.conf,
            "" if not self.reasons else " REJECTED:" + ",".join(self.reasons))


class Estimate:
    """A whole body observation, with the reason for every rejection kept."""

    def __init__(self, segments=None, stamp=0.0, source="", note=""):
        self.segments = {s.name: s for s in (segments or [])}
        self.stamp = float(stamp)
        self.source = source
        self.note = note

    def age(self, now):
        return now - self.stamp

    def usable_names(self, now=None):
        if now is not None and self.age(now) > MAX_AGE_S:
            return set()
        return {n for n, s in self.segments.items() if s.usable}


# ------------------------------------------------------------------ maths
def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        return (0.0, 0.0, 1.0)
    return tuple(c / n for c in v)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def rpy_from_direction(d):
    """Roll-pitch-yaw that points a z-axis primitive along `d`.

    THE SAME CONVENTION `wearer_posture.rpy_for` USES, and it is not the
    obvious one. URDF composes R = Rz(yaw) Ry(pitch) Rx(roll), and the wearer
    model holds YAW AT ZERO and spends roll and pitch instead. Writing the
    natural pitch/yaw form here instead produced primitives rotated 90 degrees
    about the wrong axis -- caught by the pinning test below, which is the
    only reason it is not in the planning scene right now.

    Restated rather than imported so this module needs nothing else present;
    `test_wearer_tracking.py` requires the two to agree to 1e-9, so the
    restatement cannot drift.
    """
    dx, dy, dz = _unit(d)
    roll = math.asin(max(-1.0, min(1.0, -dy)))
    c = math.cos(roll)
    if abs(c) < 1e-9:                       # the limb points straight along y
        return (roll, 0.0, 0.0)
    return (roll, math.atan2(dx / c, dz / c), 0.0)


def capsule_between(a, b, radius):
    """(dims, centre, rpy) for a cylinder spanning a to b."""
    length = _dist(a, b)
    centre = tuple((a[i] + b[i]) * 0.5 for i in range(3))
    d = tuple(b[i] - a[i] for i in range(3))
    return (radius, length), centre, rpy_from_direction(d)


# ------------------------------------------------------------- the gates
def speed_gate(prev, cur, dt, max_speed=MAX_JOINT_SPEED):
    """{joint: reason} for every joint that moved impossibly fast.

    NOT a filter. It does not smooth the jump away -- it NAMES the joint, so
    the segments that depend on it revert to the mannequin and the operator is
    told which. A smoothed impossible jump is an impossible jump you cannot
    see any more.
    """
    if not prev or dt <= 0.0:
        return {}
    bad = {}
    for name, p in cur.items():
        q = prev.get(name)
        if q is None:
            continue
        v = _dist(p, q) / dt
        if v > max_speed:
            bad[name] = "moved %.1f m/s" % v
    return bad


def plausible(kind, length):
    """Is this segment a length a human limb can be?"""
    lo, hi = PLAUSIBLE.get(kind, (0.0, 1e9))
    return lo <= length <= hi


def frame_is_usable(mean_luma, luma_span, n_people, note=""):
    """Reject a frame before any body is read out of it.

    THREE FAILURES THAT PRODUCE CONFIDENT NONSENSE rather than an error:

      * TOO DARK. A learned detector still returns keypoints on a black
        frame, with visibilities that are not obviously wrong.
      * NO CONTRAST. A lens cap, a covered camera or a driver repeating a
        blank buffer all give a nearly constant image. Mean luma alone does
        NOT catch a mid-grey blank, which is why the span is checked too.
      * MORE THAN ONE PERSON. If somebody walks behind the wearer the
        detector may track them instead. Two candidates means this module
        cannot tell which is the wearer, so it declines to guess.

    Returns (ok, reason).
    """
    if mean_luma < 12.0:
        return False, "the scene camera view is too dark to read a body from"
    if luma_span < 15.0:
        return False, ("the scene camera view is blank -- covered lens, or a "
                       "driver repeating one frame")
    if n_people > 1:
        return False, ("more than one person is in view, so which one is "
                       "wearing the arms cannot be established")
    if n_people < 1:
        return False, "nobody is in view of the scene camera"
    return True, note or "frame usable"


# ------------------------------------------------------------------ fuse
def _closest_distance(prim, probe):
    """Distance from `probe` to a primitive tuple (kind, dims, centre, rpy).

    Only the DISTANCE ordering matters here, so the cheap conservative form
    is used: the surface distance of an axis-aligned bound of the shape. A
    cylinder is treated as a capsule about its centre, which under-estimates
    the distance for a long cylinder seen end-on -- and under-estimating is
    the safe direction for a rule that keeps whichever shape is CLOSER.
    """
    kind, dims, centre, _rpy = prim
    d = _dist(probe, centre)
    if kind == "sphere":
        return d - dims[0]
    if kind == "cylinder":
        return d - math.sqrt(dims[0] ** 2 + (dims[1] * 0.5) ** 2)
    half = math.sqrt(sum((c * 0.5) ** 2 for c in dims))
    return d - half


def fuse(mannequin, estimate, now, probe=(0.0, 0.30, 1.30)):
    """The mannequin, with tracked parts substituted ONLY where safe.

    `mannequin` is `wearer_posture.wearer_model()`'s output. `probe` is a
    point standing for where the robot works -- the substitution keeps
    whichever primitive reaches CLOSER to it.

    Returns (primitives, decisions) where decisions is
    {part: (which, reason)} with `which` in {"tracked", "mannequin"}. Every
    part gets an entry, including the ones that were never tracked, so the
    GUI can show the whole body and never an absence.
    """
    out, decisions = [], {}
    usable = estimate.usable_names(now) if estimate is not None else set()
    stale = (estimate is not None and estimate.age(now) > MAX_AGE_S)
    for name, kind, dims, centre, rpy in mannequin:
        man = (kind, dims, centre, rpy)
        if name not in TRACKED_PARTS:
            out.append((name, kind, dims, centre, rpy))
            decisions[name] = ("mannequin", "not a tracked part")
            continue
        seg = estimate.segments.get(name) if estimate is not None else None
        if seg is None:
            out.append((name, kind, dims, centre, rpy))
            decisions[name] = ("mannequin",
                               "the camera did not report this part")
            continue
        if stale:
            out.append((name, kind, dims, centre, rpy))
            decisions[name] = ("mannequin", "the camera reading is too old")
            continue
        if name not in usable:
            out.append((name, kind, dims, centre, rpy))
            decisions[name] = (
                "mannequin",
                "; ".join(seg.reasons) if seg.reasons
                else "not clearly enough seen (%.2f)" % seg.conf)
            continue
        trk = (seg.kind, seg.dims, seg.centre, seg.rpy)
        if _closest_distance(trk, probe) <= _closest_distance(man, probe):
            out.append((name,) + trk)
            decisions[name] = ("tracked", "seen, and closer than assumed")
        else:
            # THE ASYMMETRY, AND IT IS THE WHOLE SAFETY CASE. The camera says
            # this part is further away than the model assumes. It may be
            # right. It may also be a monocular depth error, and there is no
            # way to tell from one camera -- so the assumption stands.
            out.append((name, kind, dims, centre, rpy))
            decisions[name] = ("mannequin",
                               "seen, but further away than assumed -- the "
                               "camera may only make the wearer bigger")
    return out, decisions


def summarise(decisions):
    """(n_tracked, n_total, one sentence) for the GUI and the log."""
    tracked = [k for k, (w, _) in decisions.items() if w == "tracked"]
    n = len(decisions)
    if not tracked:
        return 0, n, "MANNEQUIN -- no part of the body is being measured"
    if len(tracked) == len(TRACKED_PARTS):
        return len(tracked), n, "TRACKED -- every trackable part is measured"
    return (len(tracked), n,
            "PARTLY TRACKED -- %d of %d measured: %s"
            % (len(tracked), len(TRACKED_PARTS), ", ".join(sorted(tracked))))


# =========================================================================
#  FROM JOINTS TO SEGMENTS
# =========================================================================
# The joints this module needs, by the name the caller must use. Deliberately
# NOT MediaPipe's numeric indices: the numbers are a detail of one detector,
# and a second detector with the same joints must be able to feed this without
# pretending to be MediaPipe.
#
# LEFT AND RIGHT ARE THE PERSON'S OWN. That is worth stating because the
# repository's world frame uses +x for the wearer's left and the repo has an
# OPEN contradiction about whether that is really so (CLAUDE.md, desk
# operation). This module never resolves it: it is handed world-frame points
# by the caller and names parts `L-*` / `R-*` to match `wearer_model()`, so
# whichever way that argument settles, the fix is in the caller's extrinsic
# and not in here.
JOINTS = ("nose", "L-shoulder", "R-shoulder", "L-elbow", "R-elbow",
          "L-wrist", "R-wrist", "L-hand-tip", "R-hand-tip",
          "L-hip", "R-hip")


def _mid(a, b):
    return tuple((a[i] + b[i]) * 0.5 for i in range(3))


def segments_from_joints(pts, vis, prev=None, dt=0.0, now=0.0, source=""):
    """Build an Estimate from world-frame joints and per-joint visibility.

    `pts`  {joint name: (x, y, z)} in the ROBOT WORLD frame.
    `vis`  {joint name: 0..1} the detector's own confidence per joint.
    `prev` the previous `pts`, for the speed gate. None on the first frame.

    Every segment carries the reason it was rejected, if it was. A rejected
    segment is still RETURNED -- it is not dropped -- because the GUI has to
    be able to show "this part is not being measured, and here is why", and a
    part that vanishes from a list looks like a part that is fine.
    """
    fast = speed_gate(prev, pts, dt)
    segs = []

    def conf_of(*names):
        return min([vis.get(n, 0.0) for n in names] or [0.0])

    def reasons_of(*names):
        r = [("%s %s" % (n, fast[n])) for n in names if n in fast]
        missing = [n for n in names if n not in pts]
        if missing:
            r.append("no reading for " + ", ".join(missing))
        return r

    # ---- limbs -------------------------------------------------------
    for side in ("L", "R"):
        sh, el = "%s-shoulder" % side, "%s-elbow" % side
        wr, tip = "%s-wrist" % side, "%s-hand-tip" % side
        for label, a, b, kind, pad, extend in (
                ("%s-upperarm" % side, sh, el, "upperarm", LIMB_PAD_M, 1.0),
                ("%s-forearm" % side, el, wr, "forearm", LIMB_PAD_M * 0.9, 1.0),
                ("%s-hand" % side, wr, tip, "palm", LIMB_PAD_M * 0.8,
                 HAND_EXTENSION)):
            r = reasons_of(a, b)
            if a not in pts or b not in pts:
                segs.append(Segment(label, "cylinder", (pad, 0.0),
                                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    0.0, r))
                continue
            length = _dist(pts[a], pts[b])
            if not plausible(kind, length):
                r.append("%.3f m is not a %s" % (length, kind))
            far = pts[b]
            if extend != 1.0:
                # Carry the segment past its far landmark. The knuckle is
                # where the detector stops; the fingers are not.
                far = tuple(pts[a][i] + extend * (pts[b][i] - pts[a][i])
                            for i in range(3))
            dims, centre, rpy = capsule_between(pts[a], far, pad)
            segs.append(Segment(label, "cylinder", dims, centre, rpy,
                                conf_of(a, b), r))

    # ---- torso, hips, head -------------------------------------------
    need = ("L-shoulder", "R-shoulder", "L-hip", "R-hip")
    if all(n in pts for n in need):
        sh_mid = _mid(pts["L-shoulder"], pts["R-shoulder"])
        hip_mid = _mid(pts["L-hip"], pts["R-hip"])
        width = _dist(pts["L-shoulder"], pts["R-shoulder"])
        height = _dist(sh_mid, hip_mid)
        r = reasons_of(*need)
        if not plausible("shoulder_width", width):
            r.append("%.3f m is not a shoulder width" % width)
        if not plausible("torso_height", height):
            r.append("%.3f m is not a torso" % height)
        # A BOX ALIGNED WITH THE WORLD, not with the spine. The mannequin's
        # torso is world-aligned and the fused list is consumed by code that
        # assumes primitives in that convention; a rotated torso would be a
        # second change hiding inside this one.
        segs.append(Segment(
            "torso", "box",
            (width + 2 * TORSO_PAD_M, 0.22 + TORSO_PAD_M, height),
            _mid(sh_mid, hip_mid), (0.0, 0.0, 0.0),
            conf_of(*need), r))
        segs.append(Segment(
            "hips", "box", (width * 0.9, 0.21 + TORSO_PAD_M, 0.18),
            (hip_mid[0], hip_mid[1], hip_mid[2] - 0.02), (0.0, 0.0, 0.0),
            conf_of("L-hip", "R-hip"), reasons_of("L-hip", "R-hip")))
    if "nose" in pts:
        segs.append(Segment("head", "sphere", (0.115,), pts["nose"],
                            (0.0, 0.0, 0.0), conf_of("nose"),
                            reasons_of("nose")))
    return Estimate(segs, stamp=now, source=source)
