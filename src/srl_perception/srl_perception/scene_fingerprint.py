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


class Observation:
    """One object as seen in one sweep."""

    __slots__ = ("label", "xyz", "quat", "confidence", "n_views", "size")

    def __init__(self, label, xyz, quat=None, confidence=1.0, n_views=1,
                 size=None):
        self.label = str(label)
        self.xyz = tuple(float(v) for v in xyz)
        self.quat = tuple(float(v) for v in quat) if quat else None
        self.confidence = float(confidence)
        self.n_views = int(n_views)
        self.size = tuple(float(v) for v in size) if size else None

    def as_dict(self):
        return dict(label=self.label, xyz=list(self.xyz),
                    quat=list(self.quat) if self.quat else None,
                    confidence=self.confidence, n_views=self.n_views,
                    size=list(self.size) if self.size else None)

    @staticmethod
    def from_dict(d):
        return Observation(d["label"], d["xyz"], d.get("quat"),
                           d.get("confidence", 1.0), d.get("n_views", 1),
                           d.get("size"))


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

    VERSION = 1

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
        rot = quat_angle_deg(s.quat, o.quat)
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
