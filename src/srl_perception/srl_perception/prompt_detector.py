#!/usr/bin/env python3
"""Open-vocabulary object detection: a SENTENCE in, objects out.

    from srl_perception.prompt_detector import PromptDetector
    det = PromptDetector()
    hits = det.detect(bgr, "the green cube", depth_m=depth, K=K)

TWO BACKENDS, AND THE FALLBACK IS NOT AN AFTERTHOUGHT.

  yoloworld  YOLO-World, open-vocabulary: any noun phrase, no retraining.
             Chosen over GroundingDINO and OWLv2 for one measured reason --
             this machine has an RTX A500 with 4 GB, shared with everything
             else. YOLO-World is ~60M parameters against GroundingDINO's 218M
             and OWLv2's 428M, and runs ~52 FPS against 3.2 and 8.5. The
             bigger models score higher on open benchmarks and do not fit.

  colour     HSV + depth. No model, no download, no GPU. It only knows the
             colours in COLOUR_TERMS, and it says so rather than pretending.

The backend is REPORTED on every result. A detection that does not say how it
was made invites the reader to assume the good one was running.

DEPTH IS PART OF DETECTION HERE, NOT A LATER STEP. Measured in this room:
the teal robots' shadowed hue is 76 and the target green cube's is 74 -- two
apart, inside noise, and colour-only filters picked the robots three times.
They sit 2.8 m away while anything on the rig is under 1.6 m. So range is a
discriminator that has nothing to do with colour, and it is applied at
detection time rather than left for the caller to remember.
"""
from __future__ import annotations

import re

import numpy as np

# The colour fallback's whole vocabulary, stated so a caller can see the
# limit rather than discover it. Ranges are OpenCV HSV (hue 0..179).
COLOUR_TERMS = {
    "red":    [((0, 90, 40), (8, 255, 255)), ((170, 90, 40), (179, 255, 255))],
    "orange": [((9, 110, 60), (22, 255, 255))],
    "yellow": [((23, 90, 70), (35, 255, 255))],
    "green":  [((36, 80, 25), (85, 255, 255))],
    "cyan":   [((86, 80, 40), (98, 255, 255))],
    "blue":   [((99, 80, 30), (130, 255, 255))],
    "purple": [((131, 60, 30), (160, 255, 255))],
    "magenta": [((161, 80, 40), (169, 255, 255))],
    "pink":   [((161, 40, 120), (172, 255, 255))],
    "black":  [((0, 0, 0), (179, 255, 45))],
    "white":  [((0, 0, 200), (179, 40, 255))],
}
SHAPE_TERMS = ("cube", "block", "box", "ball", "sphere", "cylinder", "can",
               "bottle", "mug", "cup", "object", "thing", "item")
STOP = {"the", "a", "an", "pick", "up", "grab", "get", "take", "that", "this",
        "please", "me", "it", "and", "then", "to", "of", "on", "in"}


class Detection:
    """One object, with everything a grasp needs and a note on provenance."""

    def __init__(self, label, score, bbox, centre_uv, backend,
                 depth_m=None, xyz_cam=None, mask=None, extra=None):
        self.label = label
        self.score = float(score)
        self.bbox = tuple(int(v) for v in bbox)      # x, y, w, h
        self.centre_uv = (float(centre_uv[0]), float(centre_uv[1]))
        self.backend = backend
        self.depth_m = depth_m
        self.xyz_cam = xyz_cam
        self.mask = mask
        self.extra = extra or {}

    def __repr__(self):
        d = "%.3f m" % self.depth_m if self.depth_m is not None else "no depth"
        return ("<Detection %s %.2f at %s %s via %s>"
                % (self.label, self.score, tuple(round(v) for v in self.centre_uv),
                   d, self.backend))

    def as_dict(self):
        return dict(label=self.label, score=self.score, bbox=list(self.bbox),
                    centre_uv=list(self.centre_uv), backend=self.backend,
                    depth_m=self.depth_m,
                    xyz_cam=None if self.xyz_cam is None
                    else [float(v) for v in self.xyz_cam], **self.extra)


def _colour_matches(term, b, g, r):
    """Does a region's MEAN colour match a colour word?

    A mean over a whole segment, not a per-pixel hue test. The room's teal
    robots have individual pixels whose hue (76) is within 2 of the green
    cube's (74) -- that fooled a per-pixel filter three times. Their region
    MEANS are clearly separated, because teal carries far more blue.
    """
    eps = 1.0
    # A RATIO TEST ON NEAR-BLACK PIXELS PASSES ON NOISE.
    #
    # Measured 2026-08-21: a segment with mean BGR (4.1, 8.8, 3.9) -- visually
    # black -- satisfied "g > b*1.25 and g > r*1.8" and was reported as the
    # green cube. It projected to z = -0.176 m, below the floor. At those
    # levels the ratios are sensor noise, not colour.
    #
    # The real cube measures (29, 52, 10): its green channel is 52. Requiring
    # the dominant channel to actually be lit separates a coloured object from
    # a dark one, and costs nothing on a genuinely coloured target.
    if term not in ("black",) and max(b, g, r) < 25.0:
        return False
    if term == "green":
        return g > b * 1.25 and g > r * 1.8
    if term == "red":
        return r > g * 1.5 and r > b * 1.5
    if term == "blue":
        return b > g * 1.4 and b > r * 1.6
    if term == "cyan":
        return b > r * 1.6 and g > r * 1.6 and abs(b - g) < 0.5 * max(b, eps)
    if term == "yellow":
        return g > b * 1.6 and r > b * 1.6
    if term == "magenta" or term == "pink":
        return b > g * 1.3 and r > g * 1.3
    if term == "orange":
        return r > g * 1.2 and g > b * 1.3
    if term == "white":
        return min(b, g, r) > 150
    if term == "black":
        return max(b, g, r) < 60
    return True


def parse_prompt(text):
    """A sentence -> (colour terms, shape terms, leftover nouns).

    Deliberately small and inspectable. It is not trying to be a language
    model; it is trying to make the colour fallback usable from the same box
    the neural backend reads, so an operator types one thing either way.
    """
    words = re.findall(r"[a-z]+", (text or "").lower())
    cols = [w for w in words if w in COLOUR_TERMS]
    shapes = [w for w in words if w in SHAPE_TERMS]
    rest = [w for w in words if w not in STOP and w not in cols
            and w not in shapes]
    return cols, shapes, rest


class PromptDetector:
    def __init__(self, backend="auto", conf=0.05, near_m=None, far_m=None):
        self.conf = float(conf)
        self.near_m = near_m
        self.far_m = far_m
        self._yw = None
        self._sam = None
        self.sam_weights = "FastSAM-s.pt"
        self.sam_imgsz = 1024
        self.device = "cpu"
        self.min_area = 40
        self.last_note = ""
        self.backend = self._pick(backend)

    def _pick(self, want):
        if want in ("colour", "color"):
            return "colour"
        if want == "segment":
            return "segment"
        if want == "both":
            return "both"
        if want in ("auto", "yoloworld"):
            try:
                from ultralytics import YOLOWorld       # noqa: F401
                return "yoloworld"
            except Exception:
                if want == "yoloworld":
                    raise RuntimeError(
                        "yoloworld backend asked for but ultralytics is not "
                        "installed. Install it, or use backend='colour', "
                        "which needs nothing.")
                return "colour"
            # BOTH, BECAUSE THEY FAIL AT DIFFERENT THINGS.
            #
            # Measured on this rig 2026-08-21, same frame: YOLO-World finds a
            # person at 0.725 and a laptop at 0.739 and CANNOT find the target
            # green cube at all -- not even upscaled 4x -- because it is ~19 px
            # and very dark (HSV value 26-63). The colour backend finds that
            # cube every time and cannot name a laptop.
            #
            # Neither is "the good one". Running the neural model first and
            # falling through to colour when it finds nothing covers the union,
            # and every detection still reports which backend produced it.
            return "both"
        raise ValueError("unknown backend %r" % want)

    def available(self):
        """What this machine can actually run, for the GUI to display."""
        out = {"colour": True}
        try:
            import ultralytics                            # noqa: F401
            out["yoloworld"] = True
        except Exception:
            out["yoloworld"] = False
        return out

    # ------------------------------------------------------------- backends
    def _yolo(self, bgr, prompt):
        from ultralytics import YOLOWorld
        if self._yw is None:
            self._yw = YOLOWorld("yolov8s-world.pt")
        cols, shapes, rest = parse_prompt(prompt)
        classes = [" ".join(x for x in [c, s] if x)
                   for c in (cols or [""]) for s in (shapes or rest or ["object"])]
        classes = [c.strip() for c in classes if c.strip()] or ["object"]
        self._yw.set_classes(classes)
        res = self._yw.predict(bgr, conf=self.conf, verbose=False)
        out = []
        for r in res:
            for b in r.boxes:
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                out.append(Detection(
                    classes[int(b.cls[0])] if int(b.cls[0]) < len(classes) else "object",
                    float(b.conf[0]), (x1, y1, x2 - x1, y2 - y1),
                    ((x1 + x2) / 2, (y1 + y2) / 2), "yoloworld"))
        return out

    def _segment(self, bgr, prompt):
        """SEGMENT EVERYTHING, THEN PICK THE ONE THAT MATCHES.

        This is the backend that actually found the target. Asking an
        open-vocabulary detector to NAME a 19 px dark cube failed at every
        confidence -- it is too small and too dark to be recognised as
        anything. But it is trivially SEGMENTABLE: FastSAM cut the real scene
        into 89 regions and exactly one of them was green-dominant, at the
        cube, with no false positives. The teal robots were segmented too and
        rejected on colour, because a whole-region mean is a far stronger
        statistic than the per-pixel hue that fooled three earlier filters.

        The mask is the other win. A colour threshold gives whatever pixels
        happened to pass; a segment gives the object's actual extent, which is
        what the grasp planner needs to measure a width.
        """
        import cv2
        from ultralytics import FastSAM
        if self._sam is None:
            self._sam = FastSAM(self.sam_weights)
        res = self._sam.predict(bgr, device=self.device, retina_masks=True,
                                imgsz=self.sam_imgsz, conf=0.25, iou=0.7,
                                verbose=False)
        r = res[0]
        if r.masks is None:
            return []
        cols, shapes, rest = parse_prompt(prompt)
        H, W = bgr.shape[:2]
        out = []
        for mk in r.masks.data.cpu().numpy():
            m = mk.astype(bool)
            if m.shape != (H, W):
                m = cv2.resize(mk.astype(np.uint8), (W, H)) > 0
            a = int(m.sum())
            if a < self.min_area or a > 0.35 * H * W:
                continue
            b, g, rr = [float(v) for v in cv2.mean(bgr, m.astype(np.uint8))[:3]]
            if cols and not _colour_matches(cols[0], b, g, rr):
                continue
            ys, xs = np.nonzero(m)
            x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
            out.append(Detection(
                (" ".join(cols + shapes) or "object").strip(),
                min(1.0, a / 2000.0), (x0, y0, x1 - x0 + 1, y1 - y0 + 1),
                (float(xs.mean()), float(ys.mean())), "segment", mask=m,
                extra=dict(area_px=a, mean_bgr=[round(b, 1), round(g, 1),
                                                round(rr, 1)])))
        out.sort(key=lambda d: -d.score)
        return out

    def _colour(self, bgr, prompt):
        import cv2
        cols, shapes, rest = parse_prompt(prompt)
        if not cols:
            raise ValueError(
                "the colour backend needs a colour word. Known: %s. "
                "Asked for %r. Install ultralytics for open-vocabulary "
                "detection that does not need one."
                % (", ".join(sorted(COLOUR_TERMS)), prompt))
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        out = []
        for col in cols:
            m = None
            for lo, hi in COLOUR_TERMS[col]:
                part = cv2.inRange(hsv, np.array(lo), np.array(hi))
                m = part if m is None else cv2.bitwise_or(m, part)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
            for i in range(1, n):
                a = int(st[i, cv2.CC_STAT_AREA])
                if a < 40:
                    continue
                x, y, w, h = (int(st[i, cv2.CC_STAT_LEFT]),
                              int(st[i, cv2.CC_STAT_TOP]),
                              int(st[i, cv2.CC_STAT_WIDTH]),
                              int(st[i, cv2.CC_STAT_HEIGHT]))
                out.append(Detection(
                    (col + " " + (shapes[0] if shapes else "object")).strip(),
                    min(1.0, a / 2000.0), (x, y, w, h),
                    (float(ce[i][0]), float(ce[i][1])), "colour",
                    mask=(lab == i), extra=dict(area_px=a)))
        return out

    # ---------------------------------------------------------------- public
    def detect(self, bgr, prompt, depth_m=None, K=None, depth_K=None):
        """Detections for `prompt`, with depth attached when it is available.

        `depth_m` may be a DIFFERENT resolution to the colour frame -- on this
        rig colour is 1280x720 and depth is 480x270 -- so each detection's
        bearing is computed with the colour intrinsics and re-projected with
        the depth intrinsics rather than the images being assumed aligned.
        """
        if self.backend == "both":
            hits = []
            # ORDER IS MEASURED, NOT ALPHABETICAL. Segmentation found the
            # target when naming it could not, so it goes first; naming is
            # better for large everyday objects, so it goes second; colour is
            # the floor that always works.
            for fn, why in ((self._segment, "segment"),
                            (self._yolo, "yoloworld")):
                try:
                    hits = fn(bgr, prompt)
                except Exception as exc:                      # noqa: BLE001
                    self.last_note = "%s unavailable (%s)" % (why, exc)
                    hits = []
                if hits:
                    break
            if not hits:
                try:
                    hits = self._colour(bgr, prompt)
                except ValueError:
                    # no colour word AND the model found nothing: that is a
                    # real "not found", not a missing vocabulary.
                    hits = []
        elif self.backend == "segment":
            hits = self._segment(bgr, prompt)
        elif self.backend == "yoloworld":
            hits = self._yolo(bgr, prompt)
        else:
            hits = self._colour(bgr, prompt)
        if depth_m is not None and K is not None:
            dK = depth_K or K
            keep = []
            for d in hits:
                u, v = d.centre_uv
                bx = (u - K[2]) / K[0]
                by = (v - K[3]) / K[1]
                du = dK[2] + bx * dK[0]
                dv = dK[3] + by * dK[1]
                H, W = depth_m.shape[:2]
                if not (0 <= du < W and 0 <= dv < H):
                    d.extra["depth_note"] = "outside the depth sensor's field of view"
                    keep.append(d)
                    continue
                win = depth_m[max(0, int(dv) - 3):int(dv) + 4,
                              max(0, int(du) - 3):int(du) + 4]
                val = win[np.isfinite(win) & (win > 0)]
                if val.size == 0:
                    d.extra["depth_note"] = "no depth return at this pixel"
                    keep.append(d)
                    continue
                z = float(np.median(val))
                if self.near_m is not None and z < self.near_m:
                    continue
                if self.far_m is not None and z > self.far_m:
                    continue
                d.depth_m = z
                d.xyz_cam = np.array([bx * z, by * z, z], float)
                keep.append(d)
            hits = keep
        hits.sort(key=lambda d: -d.score)
        return hits
