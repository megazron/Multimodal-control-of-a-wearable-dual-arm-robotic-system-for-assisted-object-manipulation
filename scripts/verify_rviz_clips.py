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
    return ((a[:, :, 0] > 165) & (a[:, :, 1] > 55) & (a[:, :, 1] < 175)
            & (a[:, :, 2] < 90))


def _green(a):
    return ((a[:, :, 1] > 105) & (a[:, :, 0] < 130) & (a[:, :, 2] < 130)
            & (a[:, :, 1] - a[:, :, 0] > 45) & (a[:, :, 1] - a[:, :, 2] > 45))


PALETTE = {
    "tray_tan": _tan, "sling_orange": _tan, "ball_yellow": _yellow,
    "container_teal": _teal, "block_orange": _orange,
    "block_green": _green, "target_green": _green,
}
MIN_PIXELS = 25          # smallest confirmed object was 39 px at this scale
# what each task MUST show at least one of
REQUIRED = {
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
    mp4s = sorted(glob.glob(os.path.join(OUT, "*/*/*/rviz.mp4")))
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
        missing = [k for k, v in found.items() if v < MIN_PIXELS]
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
