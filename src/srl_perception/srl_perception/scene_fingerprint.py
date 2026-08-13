#!/usr/bin/env python3
"""SCENE FINGERPRINTING -- calibrate only when the bed has changed.

THE IDEA. A 3D printer levels its bed before a print, but a good one skips the
probe when nothing has moved. Here the equivalent is: sweep the wrist cameras
once, record what is on the table and where, store it, and on the next start
sweep again and COMPARE. If the scene matches, reuse the stored poses and skip
re-registration entirely. If something moved, appeared or vanished,
re-register only that.

WHAT THIS MODULE IS AND IS NOT. It is the matching and differencing logic, and
it is PURE: no ROS handles, no camera, no I/O beyond an explicit path. That
matters because it is the part that can be tested against CONSTRUCTED ground
truth. Two points 30 mm apart really are 30 mm apart, so a displacement test
built from arithmetic is trustworthy in a way a rendered image is not. The
detection side, which turns pixels into a pose, is NOT tested here and its
accuracy on real cameras remains unmeasured.

THE ASSOCIATION PROBLEM, AND ITS IRREDUCIBLE AMBIGUITY. Comparing two scenes
means deciding which observation corresponds to which stored object. Nearest
neighbour within a gate is used. The gate creates a limit that cannot be
designed away:

    an object that moves FURTHER than the gate is indistinguishable from one
    that vanished while a new one appeared at the destination.

No amount of tuning removes this; it is a property of observing only
positions. Widening the gate trades one error for the other. The gate is
therefore an explicit parameter, its consequence is reported by name in the
diff, and `MOVED_OR_SWAPPED` is a distinct verdict from `MOVED` so a caller is
never told "it moved 400 mm" with more confidence than the data supports.
"""
import json
import math
import os
import time

# Verdicts a comparison can return, per object.
SAME = "same"
MOVED = "moved"
MOVED_OR_SWAPPED = "moved_or_swapped"
APPEARED = "appeared"
VANISHED = "vanished"
RECLASSIFIED = "reclassified"


# AN IDENTITY QUATERNION IS NOT A MEASUREMENT OF ZERO ROTATION.
#
# This is U-2 in TASK_SPEC.md and it is the by_design.py failure wearing
# perception's clothes: the colour/shape detector published w = 1.0 because
# roll is unobservable to it, the grasp path read that as yaw = 0, and an
# object sitting at 30 degrees was grasped square. Nothing anywhere could
# tell "the object is square" from "nobody looked".
#
# So yaw is carried EXPLICITLY, with a flag, and code that wants it has to
# ask whether it is known.
IDENTITY_TOL = 1e-6


def yaw_from_quat(q):
    """Yaw about world z, in degrees, or None if `q` carries no rotation.

    Returns None for identity ON PURPOSE. A caller that wants to treat
    unknown as square must write that down.

    THE ONE CASE THIS CONVENTION LOSES, stated rather than discovered later:
    an object MEASURED at exactly 0.000000 degrees encodes as identity and is
    therefore read back as unmeasured. `vision_msgs` has no field for "not
    observed", so the sentinel has to be a pose, and identity is the only
    honest choice. The cost is nil in practice and here is why: the objects
    are 90-degree symmetric, so a grasp planned for "unknown, assume square"
    and a grasp planned for "measured, 0 degrees" are the SAME grasp. Any
    angle at all -- 0.5 degrees encodes as z = 0.0044, four thousand times
    the tolerance -- comes through as measured. Where the distinction has to
    survive exactly, pass `yaw_deg=` to Observation() explicitly; it is
    believed over the quaternion.
    """
    if q is None:
        return None
    x, y, z, w = q
    if abs(x) < IDENTITY_TOL and abs(y) < IDENTITY_TOL and \
            abs(z) < IDENTITY_TOL and abs(abs(w) - 1.0) < IDENTITY_TOL:
        return None
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def wrap_symmetric(deg, symmetry_deg=90.0):
    """Fold an angle into the object's own symmetry.

    A cube grasped at 0 and at 90 degrees is grasped identically, so the two
    must compare EQUAL. Without this a cube nudged past 45 degrees reports a
    90 degree rotation and every scan looks like a rotated object.
    """
    if deg is None:
        return None
    d = deg % symmetry_deg
    return d - symmetry_deg if d > symmetry_deg / 2.0 else d


def mean_yaw_deg(values, symmetry_deg=90.0):
    """Circular mean inside the symmetry, or None if there is nothing to
    average. An arithmetic mean of 89 and -89 is 0, which is the wrong
    answer by the whole symmetry."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    k = 360.0 / symmetry_deg
    s = sum(math.sin(math.radians(v * k)) for v in vals)
    c = sum(math.cos(math.radians(v * k)) for v in vals)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return wrap_symmetric(vals[0], symmetry_deg)
    return wrap_symmetric(math.degrees(math.atan2(s, c)) / k, symmetry_deg)


class Observation:
    """One object as seen in one sweep."""

    __slots__ = ("label", "xyz", "quat", "confidence", "n_views", "size",
                 "yaw_deg", "symmetry_deg", "yaw_views")

    def __init__(self, label, xyz, quat=None, confidence=1.0, n_views=1,
                 size=None, yaw_deg=None, symmetry_deg=90.0, yaw_views=None):
        self.label = str(label)
        self.xyz = tuple(float(v) for v in xyz)
        self.quat = tuple(float(v) for v in quat) if quat else None
        self.confidence = float(confidence)
        self.n_views = int(n_views)
        self.size = tuple(float(v) for v in size) if size else None
        self.symmetry_deg = float(symmetry_deg)
        # YAW IS FIRST CLASS, not derived on demand from `quat`. An explicit
        # yaw_deg can be None, and None is the answer to "was it measured".
        y = yaw_deg if yaw_deg is not None else yaw_from_quat(self.quat)
        self.yaw_deg = wrap_symmetric(y, self.symmetry_deg)
        # Every yaw this object has contributed, so the running estimate is a
        # circular mean over views rather than whichever view arrived first.
        self.yaw_views = list(yaw_views) if yaw_views is not None else \
            ([self.yaw_deg] if self.yaw_deg is not None else [])

    @property
    def yaw_known(self):
        return self.yaw_deg is not None

    def add_yaw(self, yaw_deg):
        """Fold another view's yaw into the running estimate."""
        y = wrap_symmetric(yaw_deg, self.symmetry_deg)
        if y is None:
            return
        self.yaw_views.append(y)
        self.yaw_deg = mean_yaw_deg(self.yaw_views, self.symmetry_deg)

    def as_dict(self):
        return dict(label=self.label, xyz=list(self.xyz),
                    quat=list(self.quat) if self.quat else None,
                    confidence=self.confidence, n_views=self.n_views,
                    size=list(self.size) if self.size else None,
                    # WRITTEN OUT EVEN WHEN None. A key that vanishes when
                    # unmeasured makes "not measured" and "old file" the same
                    # thing to every reader.
                    yaw_deg=(round(self.yaw_deg, 2)
                             if self.yaw_deg is not None else None),
                    yaw_known=self.yaw_known,
                    yaw_views=len(self.yaw_views),
                    symmetry_deg=self.symmetry_deg)

    @staticmethod
    def from_dict(d):
        return Observation(d["label"], d["xyz"], d.get("quat"),
                           d.get("confidence", 1.0), d.get("n_views", 1),
                           d.get("size"), d.get("yaw_deg"),
                           d.get("symmetry_deg", 90.0))


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def quat_angle_deg(a, b):
    if a is None or b is None:
        return None
    na = math.sqrt(sum(x * x for x in a)) or 1.0
    nb = math.sqrt(sum(x * x for x in b)) or 1.0
    d = abs(sum((x / na) * (y / nb) for x, y in zip(a, b)))
    return 2.0 * math.degrees(math.acos(min(1.0, d)))


class Fingerprint:
    """A stored scene: what was where, with what confidence, and when."""

    # 2: yaw became a first-class field. A version 1 store has no yaw at all,
    # and load() refusing it is right -- reading one would silently give every
    # object yaw_known = False and make a rotated object look unmeasurable
    # rather than unmeasured.
    VERSION = 2

    def __init__(self, objects=None, stamp=None, sweep_s=None, note=""):
        self.objects = list(objects or [])
        self.stamp = stamp
        self.sweep_s = sweep_s
        self.note = note

    # ------------------------------------------------------------ storage
    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(dict(version=self.VERSION, stamp=self.stamp,
                           sweep_s=self.sweep_s, note=self.note,
                           objects=[o.as_dict() for o in self.objects]),
                      f, indent=2)
        os.replace(tmp, path)          # atomic: a crash mid-write cannot
        return path                    # leave a half-written fingerprint

    @staticmethod
    def load(path):
        if not os.path.exists(path):
            return None
        try:
            d = json.load(open(path))
        except Exception:                                    # noqa: BLE001
            return None
        if d.get("version") != Fingerprint.VERSION:
            return None                # a schema change invalidates the store
        return Fingerprint([Observation.from_dict(o) for o in d["objects"]],
                           d.get("stamp"), d.get("sweep_s"), d.get("note", ""))


# PREVIOUS SETTING: pos_tol = 0.015 (15 mm), until 2026-08-10.
#
# CHANGED TO 10 mm because the GRASP does not survive 15 mm. Measured capture
# half-windows are 22.5 mm (40 mm block), 20.0 mm (45 mm part) and 17.5 mm
# (50 mm multimeter), so at 15 mm of real error an object could be declared
# UNCHANGED here and then not be grasped -- the tolerance was looser than the
# thing it protects. The limit is CAPTURE, not IK: every grasp pose survives
# +/-20 mm of displacement in IK terms.
#
# FALSE-CHANGED RATE (unmoved object declared MOVED), 20000 trials through
# this same compare(), noise on BOTH the stored pose and the observation:
#
#     detector                    tol 15 mm    tol 10 mm
#     AprilTag,     sigma 0.8 mm     0.00%        0.00%
#     sigma 1.5 mm                   0.00%        0.01%
#     sigma 2.0 mm                   0.00%        0.53%
#     sigma 2.5 mm                   0.06%        4.63%
#     sigma 3.0 mm                   0.56%       13.14%
#     colour/shape, sigma 10 mm     76.83%       92.03%
#
# So tightening to 10 mm is FREE on the AprilTag path and impossible on the
# colour/shape path -- which was already 77% false-changed at 15 mm, i.e.
# already unusable for this decision and not made unusable by the change.
#
# THE NOISE BUDGET THIS IMPLIES: to hold false-changed under 1% at 10 mm the
# detector needs sigma <= 2 mm. AprilTag at 0.35 m measures 0.74 mm and
# clears it comfortably; nothing else in the project has been measured on
# real images at all.
def compare(stored, observed, pos_tol=0.010, gate=0.25, rot_tol_deg=15.0):
    """Diff two scenes.

    pos_tol   below this, an object counts as unmoved. Set from the detector's
              own pose noise: a tolerance tighter than the noise floor reports
              motion every sweep.
    gate      association radius. Beyond it, a move is indistinguishable from
              a vanish plus an appear (see the module docstring).
    rot_tol   orientation change that counts as a re-orientation.

    Returns (verdicts, summary). `verdicts` is a list of per-object records;
    `summary` carries the counts and whether calibration can be skipped.
    """
    out = []
    used = set()
    # Greedy nearest within the gate, best pairs first. Greedy is adequate
    # because the gate already excludes the far pairings that make greedy
    # differ from an optimal assignment, and it is inspectable in a way the
    # optimal solver is not.
    pairs = []
    for i, s in enumerate(stored.objects if hasattr(stored, "objects")
                          else stored):
        for j, o in enumerate(observed):
            d = dist(s.xyz, o.xyz)
            if d <= gate:
                pairs.append((d, i, j))
    pairs.sort()
    smatch, omatch = {}, {}
    for d, i, j in pairs:
        if i in smatch or j in omatch:
            continue
        smatch[i], omatch[j] = j, i

    sl = stored.objects if hasattr(stored, "objects") else stored
    for i, s in enumerate(sl):
        j = smatch.get(i)
        if j is None:
            out.append(dict(verdict=VANISHED, label=s.label,
                            stored=list(s.xyz), observed=None, delta_m=None,
                            note="not seen within %.0f mm of its stored pose"
                                 % (1000 * gate)))
            continue
        o = observed[j]
        d = dist(s.xyz, o.xyz)
        # ---- HOW MUCH DID IT TURN, and it is not quat_angle_deg alone ----
        #
        # Two things were wrong with taking `quat_angle_deg(s.quat, o.quat)`
        # and nothing else, and both were silent:
        #
        #   1. AN IDENTITY QUATERNION IS NOT AN ORIENTATION. The fallback
        #      detector published identity for everything, so comparing a
        #      stored identity against a measured 30 degrees reported a
        #      30 degree rotation that nobody ever observed -- a MOVED
        #      verdict manufactured out of a missing measurement.
        #   2. A CUBE TURNED 90 DEGREES IS NOT TURNED. quat_angle_deg says
        #      90, so every square object nudged past its own symmetry
        #      flagged as re-oriented and the operator learned to ignore it.
        #
        # So: prefer the yaw comparison, folded into the object's symmetry,
        # whenever BOTH sides measured a yaw. Fall back to the full
        # orientation only when both carry a real, non-identity quaternion --
        # which is the AprilTag path, where pitch and roll are meaningful.
        # Otherwise None, meaning nobody knows, which is a third answer and
        # not a zero.
        s_real_q = s.quat is not None and yaw_from_quat(s.quat) is not None
        o_real_q = o.quat is not None and yaw_from_quat(o.quat) is not None
        if s.yaw_known and getattr(o, "yaw_known", False):
            rot = abs(wrap_symmetric(o.yaw_deg - s.yaw_deg, s.symmetry_deg))
        elif s_real_q and o_real_q:
            rot = quat_angle_deg(s.quat, o.quat)
        else:
            rot = None
        if s.label != o.label:
            out.append(dict(verdict=RECLASSIFIED, label=s.label,
                            new_label=o.label, stored=list(s.xyz),
                            observed=list(o.xyz), delta_m=d,
                            note="same place, different identity"))
        elif d <= pos_tol and (rot is None or rot <= rot_tol_deg):
            out.append(dict(verdict=SAME, label=s.label, stored=list(s.xyz),
                            observed=list(o.xyz), delta_m=d, rot_deg=rot))
        else:
            # Near the gate the association itself is in doubt, so the verdict
            # weakens rather than the number being quoted with false
            # confidence.
            v = MOVED_OR_SWAPPED if d > 0.6 * gate else MOVED
            out.append(dict(verdict=v, label=s.label, stored=list(s.xyz),
                            observed=list(o.xyz), delta_m=d, rot_deg=rot,
                            note=("displacement is a large fraction of the "
                                  "association gate, so a swap cannot be "
                                  "excluded") if v == MOVED_OR_SWAPPED else ""))

    for j, o in enumerate(observed):
        if j not in omatch:
            out.append(dict(verdict=APPEARED, label=o.label, stored=None,
                            observed=list(o.xyz), delta_m=None,
                            confidence=o.confidence))

    counts = {}
    for r in out:
        counts[r["verdict"]] = counts.get(r["verdict"], 0) + 1
    changed = [r for r in out if r["verdict"] != SAME]
    summary = dict(
        counts=counts,
        n_stored=len(sl), n_observed=len(observed),
        n_changed=len(changed),
        unchanged=len(changed) == 0,
        # Only the changed objects need re-registering. That is the whole
        # point: a scene where one block moved costs one re-registration, not
        # a full sweep of everything.
        reregister=[r["label"] for r in changed
                    if r["verdict"] in (MOVED, MOVED_OR_SWAPPED, APPEARED,
                                        RECLASSIFIED)],
        drop=[r["label"] for r in changed if r["verdict"] == VANISHED],
        pos_tol_m=pos_tol, gate_m=gate)
    return out, summary


def min_detectable_displacement(pos_tol, pose_noise_sd):
    """Smallest move that will be called MOVED rather than SAME.

    Two things bound it and the larger wins. The tolerance is a hard floor by
    construction. The detector's own noise is a statistical floor: a move
    smaller than a couple of standard deviations of the pose estimate is not
    separable from noise on a single sweep, whatever the tolerance says.
    """
    return max(pos_tol, 2.0 * pose_noise_sd)


def drift_check(stored, observation, pos_tol=0.015, gate=0.25):
    """During operation: is this observation consistent with the store?

    Returns (ok, verdict, delta). A caller must treat `ok=False` as a reason to
    FLAG rather than to act, because the two live possibilities -- the object
    moved, or this is a misdetection -- have opposite correct responses and a
    single observation cannot separate them.
    """
    sl = stored.objects if hasattr(stored, "objects") else stored
    best, bd = None, None
    for s in sl:
        d = dist(s.xyz, observation.xyz)
        if bd is None or d < bd:
            best, bd = s, d
    if best is None:
        return False, APPEARED, None
    if bd > gate:
        return False, APPEARED, bd
    if best.label != observation.label:
        return False, RECLASSIFIED, bd
    if bd <= pos_tol:
        return True, SAME, bd
    return False, MOVED, bd


def now():
    return time.time()
