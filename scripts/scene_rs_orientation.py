#!/usr/bin/env python3
"""WHICH WAY UP THE SCENE REALSENSE IS -- measured, not asserted.

    python3 scripts/scene_rs_orientation.py --self-test
    python3 scripts/scene_rs_orientation.py --check recordings/vision_thesis
    python3 scripts/scene_rs_orientation.py --repair recordings/vision_thesis/<run>

WHAT WAS WRONG
==========================================================================
Three separate places carried the sentence "the camera is mounted upside
down" as a hard-coded default -- `cv_pickpose_visuals.grab_scene_rs`
(rotate180=True), `real_calibration.scene_cameras.SceneRealSense`
(rotate180=True), and nothing at all in `srl_realsense_node`, which
therefore disagreed with both.  Nobody had ever measured it.

The camera is NOT mounted upside down.  Measured on 2026-08-30 against the
HD webcam, which watches the same scene from the same side of the room and
is not rotated by anything: over the four recorded captures the un-rotated
RealSense frame agrees with the webcam at r = 0.66--0.70 and the
180-degree-rotated one at r = 0.27--0.34.  Four independent captures, the
same verdict every time, a factor of 2.4 apart.  So every scene_rs figure
in `recordings/vision_thesis` was recorded upside down, and the live node
-- which rotated nothing -- was the only one of the three that was right.

WHY THE IMAGE AND THE INTRINSICS MOVE TOGETHER
==========================================================================
A 180-degree rotation maps pixel (u, v) to (W-1-u, H-1-v).  The principal
point is a pixel, so it maps the same way:

    cx' = W - 1 - cx        cy' = H - 1 - cy

Rotating the picture and leaving K alone puts every deprojected ray on the
wrong side of the optical axis: at cx = 318.87 on a 640-wide frame the
error is 2 * (cx - (W-1)/2) = 2.6 px, which is 13 mm at 3 m -- small, real,
and invisible in a picture.  That is why `rotate()` below takes the image
AND K and returns both: there is no way to call it and get half a rotation.
The map is an involution, so applying it twice is the identity, which is
what `--repair` relies on to undo a rotation that has already been baked
into a recording.

WHY NOT ASK THE IMU
==========================================================================
The D435i has an accelerometer and gravity would settle it in one reading.
It is not used here because the sign convention of librealsense's accel
stream cannot be checked on this box -- the camera reaches it over usbip
and is not attached most of the time -- and a guessed sign is exactly the
kind of unverifiable constant this repository keeps being bitten by.  The
webcam comparison needs no convention: it compares one picture with
another picture of the same room.
"""
from __future__ import annotations

import json
import os

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORIENTATION_FILE = os.path.join(WS, "config", "scene_rs_orientation.json")

#: Used when the config file is absent.  It is the MEASURED value, so a
#: missing file degrades to the truth rather than to the old assumption.
DEFAULT = {
    "rotate180": False,
    "measured_on": "2026-08-30",
    "method": "agreement with the HD webcam, which watches the same scene "
              "and is rotated by nothing",
    "evidence": "recordings/vision_thesis/*: un-rotated r=0.66-0.70, "
                "rotated r=0.27-0.34, over four independent captures",
}


def load_orientation(path=ORIENTATION_FILE):
    """The stored orientation, or the measured default if there is no file."""
    try:
        with open(path) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return dict(DEFAULT, source="built-in default (no %s)" % path)
    if "rotate180" not in d:
        raise ValueError("%s has no 'rotate180' key" % path)
    d = dict(d)
    d["source"] = path
    return d


def rotate180_flag(path=ORIENTATION_FILE):
    return bool(load_orientation(path)["rotate180"])


def rotate_K(K, width, height):
    """Map intrinsics through a 180-degree image rotation.

    K is (fx, fy, cx, cy) and may carry (w, h) after it, which are returned
    unchanged because a 180-degree rotation does not resize anything.
    """
    if K is None:
        return None
    K = list(K)
    K[2] = (width - 1) - K[2]
    K[3] = (height - 1) - K[3]
    return tuple(K)


def rotate(colour, depth=None, K_colour=None, K_depth=None, apply=True):
    """Rotate a frame 180 degrees, image and intrinsics together.

    Returns (colour, depth, K_colour, K_depth).  With apply=False it is the
    identity, so a caller can pass the flag straight through rather than
    writing an `if` that could rotate one half and not the other.
    """
    if not apply:
        return colour, depth, K_colour, K_depth
    import cv2
    h, w = colour.shape[:2]
    out_c = cv2.rotate(colour, cv2.ROTATE_180)
    out_d = None if depth is None else cv2.rotate(depth, cv2.ROTATE_180)
    return (out_c, out_d,
            rotate_K(K_colour, w, h),
            rotate_K(K_depth, w, h) if depth is None
            else rotate_K(K_depth, depth.shape[1], depth.shape[0]))


# ---------------------------------------------------------------- measuring

def _signature(bgr, n=64):
    """A rotation-sensitive, exposure-insensitive signature of a picture.

    Grey, area-averaged to n x n so the two cameras' different fields of
    view and resolutions do not dominate, then z-scored so a brighter
    camera does not score differently from a darker one.
    """
    import cv2
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.resize(g, (n, n), interpolation=cv2.INTER_AREA)
    return (g - g.mean()) / (g.std() + 1e-6)


def agreement(rs_bgr, reference_bgr, n=64):
    """How well the RealSense frame agrees with an upright reference.

    Returns (r_as_given, r_rotated).  The larger wins: if r_rotated is the
    larger, the frame that was passed in is upside down.
    """
    import cv2
    ref = _signature(reference_bgr, n)
    a = float((_signature(rs_bgr, n) * ref).mean())
    b = float((_signature(cv2.rotate(rs_bgr, cv2.ROTATE_180), n) * ref).mean())
    return a, b


#: Below this the two cameras are not looking at the same thing and the
#: comparison decides nothing.  Measured separations are 0.33-0.43.
MIN_MARGIN = 0.10


def verdict(rs_bgr, reference_bgr):
    """UPRIGHT, UPSIDE_DOWN or UNDECIDED, with the numbers behind it."""
    a, b = agreement(rs_bgr, reference_bgr)
    if abs(a - b) < MIN_MARGIN:
        v = "UNDECIDED"
    else:
        v = "UPRIGHT" if a > b else "UPSIDE_DOWN"
    return v, a, b


# ------------------------------------------------------------------ repair

def repair_recording(run_dir, dry_run=False):
    """Turn an already-recorded scene_rs frame the right way up.

    The rotation is an involution, so undoing a baked-in 180-degree
    rotation is the same operation as applying one -- image and intrinsics
    both.  It REFUSES a run it cannot decide about, and a run that is
    already upright, rather than rotating on faith.
    """
    import cv2
    rs_dir = os.path.join(run_dir, "raw", "scene_rs")
    hd_dir = os.path.join(run_dir, "raw", "scene_hd")
    cpath = os.path.join(rs_dir, "colour.png")
    hpath = os.path.join(hd_dir, "colour.png")
    if not os.path.exists(cpath):
        return "no scene_rs frame in %s" % run_dir, False
    if not os.path.exists(hpath):
        return ("no scene_hd frame in %s -- nothing upright to compare "
                "against, so the orientation cannot be decided" % run_dir), False
    R, H = cv2.imread(cpath), cv2.imread(hpath)
    v, a, b = verdict(R, H)
    numbers = "as-saved r=%+.3f, rotated r=%+.3f" % (a, b)
    if v == "UNDECIDED":
        return "%s: UNDECIDED (%s) -- left alone" % (run_dir, numbers), False
    if v == "UPRIGHT":
        return "%s: already upright (%s)" % (run_dir, numbers), False
    if dry_run:
        return "%s: WOULD ROTATE (%s)" % (run_dir, numbers), True

    meta_path = os.path.join(rs_dir, "intrinsics.json")
    with open(meta_path) as fh:
        meta = json.load(fh)
    dpath = os.path.join(rs_dir, "depth.npy")
    depth = np.load(dpath) if os.path.exists(dpath) else None
    c2, d2, Kc, Kd = rotate(R, depth, meta.get("K_colour"),
                            meta.get("K_depth"), apply=True)
    cv2.imwrite(cpath, c2)
    if d2 is not None:
        np.save(dpath, d2.astype(np.float32))
    meta["K_colour"], meta["K_depth"] = (list(Kc) if Kc else None,
                                         list(Kd) if Kd else None)
    notes = [n for n in meta.get("notes", [])
             if "rotated 180" not in n and "not rotated" not in n]
    notes.insert(0,
                 "ORIENTATION CORRECTED %s: this frame had been rotated 180 "
                 "deg on the assumption that the camera is mounted upside "
                 "down. It is not. Measured against the HD webcam, which "
                 "watches the same scene and is rotated by nothing: %s. The "
                 "rotation has been undone on the image, the depth and the "
                 "principal point together (cx=W-1-cx)."
                 % (DEFAULT["measured_on"], numbers))
    meta["notes"] = notes
    meta["orientation_corrected"] = dict(rotated_by_deg=180, evidence=numbers)
    with open(meta_path, "w") as fh:
        json.dump(meta, fh, indent=2)
    return "%s: CORRECTED (%s)" % (run_dir, numbers), True


# --------------------------------------------------------------- self-test

def self_test():
    """Known answers, on constructed inputs. No camera, no recording."""
    import cv2
    ok, fail = 0, 0

    def check(name, cond):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  ok    %s" % name)
        else:
            fail += 1
            print("  FAIL  %s" % name)

    # 1. K maps as the pixel map says, on a constructed asymmetric K.
    W, H = 640, 480
    K = (600.0, 601.0, 318.87, 231.22)
    K2 = rotate_K(K, W, H)
    check("cx' = W-1-cx", abs(K2[2] - (639 - 318.87)) < 1e-9)
    check("cy' = H-1-cy", abs(K2[3] - (479 - 231.22)) < 1e-9)
    check("focal lengths are untouched", K2[0] == K[0] and K2[1] == K[1])
    check("rotating K twice is the identity",
          all(abs(x - y) < 1e-9 for x, y in zip(rotate_K(K2, W, H), K)))

    # 2. A rotation that reaches the image but not K must be impossible:
    #    rotate() returns both or neither.
    img = np.zeros((H, W, 3), np.uint8)
    img[:10, :] = 255                      # a white stripe along the TOP
    dep = np.zeros((H, W), np.float32)
    dep[:10, :] = 1.0
    c, d, kc, kd = rotate(img, dep, K, K, apply=True)
    check("the stripe moved to the bottom", c[-1, 0, 0] == 255 and c[0, 0, 0] == 0)
    check("the depth moved with it", d[-1, 0] == 1.0 and d[0, 0] == 0.0)
    check("K moved with the image", abs(kc[2] - (639 - 318.87)) < 1e-9)
    c0, d0, k0, _ = rotate(img, dep, K, K, apply=False)
    check("apply=False is the identity",
          (c0 == img).all() and (d0 == dep).all() and k0 == K)

    # 3. The discriminator must get a KNOWN-ORIENTATION pair right, both
    #    ways round -- a check that only ever fired one way is not a check.
    ref = np.zeros((480, 640, 3), np.uint8)
    ref[:160] = 20                          # dark ceiling
    ref[160:360] = 200                      # bright wall
    ref[360:] = 60                          # dark floor, but not as dark
    cv2.circle(ref, (320, 250), 60, (255, 255, 255), -1)
    upright = cv2.resize(ref, (424, 240))
    flipped = cv2.rotate(upright, cv2.ROTATE_180)
    v_up, a_up, b_up = verdict(upright, ref)
    v_dn, a_dn, b_dn = verdict(flipped, ref)
    check("an upright frame reads UPRIGHT (%+.2f vs %+.2f)" % (a_up, b_up),
          v_up == "UPRIGHT")
    check("a flipped frame reads UPSIDE_DOWN (%+.2f vs %+.2f)" % (a_dn, b_dn),
          v_dn == "UPSIDE_DOWN")

    # 4. Two pictures of nothing in common must NOT produce a verdict.
    noise = np.random.RandomState(0).randint(0, 255, (240, 320, 3), np.uint8)
    v_n, a_n, b_n = verdict(noise, ref)
    check("unrelated pictures read UNDECIDED (%+.2f vs %+.2f)" % (a_n, b_n),
          v_n == "UNDECIDED")

    # 5. The shipped configuration must say what was measured.
    o = load_orientation()
    check("the stored orientation is upright", o["rotate180"] is False)

    print("\n%d checks, %d failed" % (ok + fail, fail))
    return 1 if fail else 0


def main(argv=None):
    import argparse
    import glob
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--check", default="",
                    help="a vision_thesis directory (or one run in it): "
                         "report the orientation of every scene_rs frame")
    ap.add_argument("--repair", default="",
                    help="same, but turn upside-down frames the right way up")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()

    target = a.check or a.repair
    if not target:
        o = load_orientation()
        print(json.dumps(o, indent=2))
        return 0
    runs = ([target] if os.path.exists(os.path.join(target, "raw"))
            else sorted(d for d in glob.glob(os.path.join(target, "*"))
                        if os.path.isdir(os.path.join(d, "raw"))))
    if not runs:
        print("no runs with a raw/ directory under %s" % target)
        return 1
    changed = 0
    for r in runs:
        msg, did = repair_recording(r, dry_run=a.dry_run or bool(a.check))
        print(msg)
        changed += bool(did)
    print("\n%d run(s), %d %s" % (len(runs), changed,
                                  "would change" if (a.check or a.dry_run)
                                  else "changed"))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
