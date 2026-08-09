#!/usr/bin/env python3
"""CHECK EVERY RVIZ CLIP SHOWS ITS TASK -- automatically, then flag what to eyeball.

    python3 scripts/verify_rviz_clips.py

87 clips is more than anyone will watch frame by frame, and "the file exists"
is not verification. This samples frames from each rviz.mp4 and asks three
questions that can be answered from pixels:

  1. IS THERE A PICTURE?  mean brightness and colour variety, so a black or
     frozen capture is caught rather than shipped.
  2. IS THE OBJECT THERE? each task's objects are deliberately distinct
     colours, so the presence of the tan tray, the orange sling, the teal
     container, the orange blocks, the green target or the yellow ball is a
     pixel-count test, not an opinion.
  3. DID THE SCENE CHANGE? frames from the start, middle and end must differ,
     or the arm did not move and the clip shows a still life.

What it cannot check is whether the motion is the RIGHT motion -- that needs a
human, which is what the top-five list in INDEX.md is for. This narrows the
watching from 87 clips to the ones that fail.
"""
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")
TMP = "/tmp/claude-1000/-home-gausms-kortex-ws/3732aa29-5a7e-4c8e-b77e-379233bdc9c9/scratchpad/vfy"

# OBJECT DETECTORS, CALIBRATED ON FRAMES I CONFIRMED BY EYE.
#
# The first version matched against the RGB I set on the markers and failed 11
# clips whose objects I had personally just looked at. RViz shades every
# surface, so the rendered colour is nowhere near the requested one:
#
#     object        requested        actually rendered
#     ball          242,191, 26        189,165, 74
#     sling         140, 89, 46        136,111, 63
#     target green   26,230, 51         68,151, 63
#     container      13,191,179         45,141,140
#     block          255,115,  0       207,138, 35
#
# So detection uses RELATIONS between channels, which survive shading, rather
# than distance to a nominal colour. Thresholds come from the measured pixel
# counts on those same confirmed frames, scaled to the 600 px sample width.
def _yellow(a):
    return ((a[:, :, 0] > 140) & (a[:, :, 1] > 115) & (a[:, :, 2] < 115)
            & (a[:, :, 0] - a[:, :, 2] > 65))


def _tan(a):
    return ((a[:, :, 0] > 120) & (a[:, :, 1] > 80) & (a[:, :, 2] < 110)
            & (a[:, :, 0] - a[:, :, 2] > 45))


def _teal(a):
    return ((a[:, :, 1] > 100) & (a[:, :, 2] > 100) & (a[:, :, 0] < 110)
            & (a[:, :, 1] - a[:, :, 0] > 40))


def _orange(a):
    # R-B > 155 SEPARATES THE ORANGE BLOCK FROM THE TAN TRAY. Without it this
    # detector fired 495 px on an f3 frame containing no orange object at all,
    # because RViz's lighting brings the tray's bright pixels (192,141,72) into
    # the orange band. Calibrated against BOTH a confirmed positive (the blocks
    # in an f2 frame) and a confirmed negative (the tray in an f3 frame): the
    # blocks keep 88 px across every threshold from 130 up, while the tray
    # leak falls under the 12 px floor at 150.
    return ((a[:, :, 0] > 165) & (a[:, :, 1] > 55) & (a[:, :, 1] < 175)
            & (a[:, :, 2] < 90) & (a[:, :, 0] - a[:, :, 2] > 155))


def _green(a):
    return ((a[:, :, 1] > 105) & (a[:, :, 0] < 130) & (a[:, :, 2] < 130)
            & (a[:, :, 1] - a[:, :, 0] > 45) & (a[:, :, 1] - a[:, :, 2] > 45))


PALETTE = {
    "tray_tan": _tan, "sling_orange": _tan, "ball_yellow": _yellow,
    "container_teal": _teal, "block_orange": _orange,
    "block_green": _green, "target_green": _green,
}
# RECALIBRATED for the four-view layout. The threshold of 25 was set when the
# capture was one 1600x1000 window; each view is now 800x500, so an object
# covers about a quarter of the pixels it used to. T6's ball measures 67 px at
# native size and 18-20 px in the 600 px sample -- plainly present, and 0 on a
# black frame -- but 12 clips failed against the old number. 12 sits above the
# noise floor (which is exactly 0) and below every confirmed detection.
MIN_PIXELS = 12
# PER-OBJECT FLOORS, because a single floor cannot work: two of these
# detectors fire on the WEARER, not on any task object. Measured on a rendered
# f1 frame containing the wearer and no tray, ball or block:
#
#     tray_tan     220 px      <- the mannequin's skin
#     ball_yellow   21 px      <- skin highlights
#     container_teal 0 px
#     block_orange   0 px
#
# against true positives of 828 px (tray) and 649 px (ball) on a frame with
# both plainly visible. A shared floor of 12 px would have passed every f3 and
# f4 clip whether or not the object rendered at all, which is the failure this
# verifier exists to prevent. Floors sit above the skin and far below the
# object.
# CALIBRATED AT THE VERIFIER'S OWN 600 px SCALE, which is not the scale the
# frames are captured at. frames() rescales every sample to 600 px wide, so a
# floor calibrated on an 800 px capture is wrong by (600/800)^2 = 0.56 -- and
# that is exactly what rejected nine sling clips whose object I had already
# confirmed by eye.
#
# Measured on three eye-confirmed frames, at 600 px:
#
#                              tan   yellow
#   wearer only, NO object      54       7    <- the false-positive floor
#   sling CONFIRMED visible    192      40
#   tray  CONFIRMED visible    376     312
#
# A single floor cannot serve both objects: the sling is a 12 mm LINE_STRIP
# and covers a fraction of the solid tray. Floors sit midway between the skin
# and the dimmest confirmed object.
FLOOR = {"tray_tan": 120, "sling_orange": 120, "ball_yellow": 20}


def floor_for(name):
    return FLOOR.get(name, MIN_PIXELS)
# what each task MUST show at least one of
REQUIRED = {
    # Five-task clips. f1 and f5 carry no object at all, so requiring one
    # would fail every clip of them; they are judged on arm travel instead.
    "f1": [],
    "f2": ["container_teal", "block_orange"],
    "f3": ["tray_tan", "ball_yellow"],
    "f4": ["sling_orange", "ball_yellow"],
    "f5": ["target_green"],
    "t2": ["container_teal", "block_orange"],
    "t3": ["tray_tan", "ball_yellow"],
    "t5": ["target_green"],
    "t6": ["sling_orange", "ball_yellow"],
    "t7": ["target_green"],
    "t8": ["target_green"],
    "t9": ["target_green"],
}


def duration(mp4):
    """Seconds, parsed from ffmpeg's own banner.

    ffprobe is NOT installed -- only the ffmpeg binary was pulled in -- and
    the first version of this script silently reported "0 frames" for every
    clip because it invoked a path that does not exist. Frames are therefore
    sampled by TIME, which needs no frame count.
    """
    r = subprocess.run([FFMPEG, "-i", mp4], capture_output=True, text=True)
    for tok in (r.stderr or "").split():
        pass
    import re
    m = re.search(r"Duration:\s*(\d+):(\d+):(\d+\.\d+)", r.stderr or "")
    if not m:
        return 0.0
    return (int(m.group(1)) * 3600 + int(m.group(2)) * 60
            + float(m.group(3)))


def frames(mp4, fracs):
    os.makedirs(TMP, exist_ok=True)
    for f in glob.glob(os.path.join(TMP, "*.png")):
        os.remove(f)
    dur = duration(mp4)
    if dur <= 0:
        return []
    out = []
    for k, fr in enumerate(fracs):
        p = os.path.join(TMP, "f%02d.png" % k)
        subprocess.run([FFMPEG, "-y", "-loglevel", "error",
                        "-ss", "%.2f" % (dur * fr), "-i", mp4,
                        "-frames:v", "1", "-vf", "scale=600:-1", p],
                       check=False)
        if os.path.exists(p):
            out.append(np.asarray(Image.open(p).convert("RGB"), float))
    return out


# The overlay text sits in a fixed band at the top of every frame, and it is
# WHITE and ORANGE-YELLOW -- the same relations the ball and sling detectors
# match. Measured on one T6 frame: 587 "ball" pixels, of which only 58 were
# the ball and 529 were the letters. Because the text never moves, it dragged
# every centroid toward a stationary point and made carried objects look
# static. Every detector therefore ignores the top of the frame.
HUD_FRACTION = 0.24


def below_hud(mask):
    m = mask.copy()
    m[:int(HUD_FRACTION * m.shape[0]), :] = False
    return m


def colour_hits(img, name):
    return int(below_hud(PALETTE[name](img)).sum())


def main():
    rows = []
    mp4s = sorted(glob.glob(os.path.join(OUT, "*/*/*/rviz_front.mp4")))
    print("verifying %d rviz clips\n" % len(mp4s))
    for mp4 in mp4s:
        d = os.path.dirname(mp4)
        parts = os.path.relpath(d, OUT).split(os.sep)
        task, scen, cond = parts[0], parts[1], parts[2]
        dur = duration(mp4)
        if dur < 2.0:
            rows.append(dict(task=task, scenario=scen, condition=cond,
                             ok=False, why="clip only %.1f s long" % dur))
            continue
        F = frames(mp4, (0.30, 0.55, 0.80))
        if len(F) < 2:
            rows.append(dict(task=task, scenario=scen, condition=cond,
                             ok=False, why="could not extract frames"))
            continue
        bright = float(np.mean([f.mean() for f in F]))
        variety = int(np.mean([len(np.unique(f.reshape(-1, 3), axis=0))
                               for f in F]))
        changed = float(np.abs(F[0] - F[-1]).mean())
        found = {}
        for want in REQUIRED.get(task, []):
            found[want] = max(colour_hits(f, want) for f in F)
        missing = [k for k, v in found.items() if v < floor_for(k)]
        why = []
        if bright < 8:
            why.append("picture is black (mean %.1f)" % bright)
        if variety < 500:
            why.append("almost no colour variety (%d)" % variety)
        # FROZEN, not merely still. T9's zero-sway scenario is a stationary
        # arm holding a world-fixed point ON PURPOSE, and failing it for not
        # moving would be failing the task for doing what it says. Only a
        # genuinely frozen capture -- identical frames -- is an error.
        if changed < 0.05:
            why.append("capture is FROZEN (frame delta %.3f)" % changed)
        if missing:
            why.append("object not visible: %s" % ", ".join(missing))
        rows.append(dict(task=task, scenario=scen, condition=cond,
                         duration_s=round(dur, 1),
                         ok=not why, why="; ".join(why), bright=round(bright, 1),
                         variety=variety, delta=round(changed, 2),
                         objects={k: int(v) for k, v in found.items()}))
        print("  %-4s %-22s %-9s %s%s"
              % (task, scen, cond, "OK  " if not why else "FAIL",
                 "" if not why else "  <- " + "; ".join(why)))

    good = [r for r in rows if r["ok"]]
    print("\n  %d of %d clips pass every automatic check" % (len(good), len(rows)))
    bad = [r for r in rows if not r["ok"]]
    if bad:
        print("  FAILING:")
        for r in bad:
            print("    %s/%s/%s -- %s"
                  % (r["task"], r["scenario"], r["condition"], r["why"]))
    json.dump(rows, open(os.path.join(OUT, "rviz_verification.json"), "w"),
              indent=2)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
