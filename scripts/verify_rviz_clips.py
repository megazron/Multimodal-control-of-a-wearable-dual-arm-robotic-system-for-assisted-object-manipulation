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
import random
import shutil
import tempfile
import os
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")

# The seven camera angles the sweep writes beside rviz_front.mp4. Every one is
# checked for a picture; see the note in main() on why only the front is
# checked for objects.
ANGLES = ("front", "back", "left", "right", "iso", "top", "gripper", "quad")


def angle_stats(mp4):
    """Mean brightness and frame-to-frame change, in ONE ffmpeg call.

    The first version reused frames(), which seeks and spawns ffmpeg once per
    sample point: 24 clips x 8 angles x 3 points = 576 invocations, and the
    tool went from a minute to over fifteen and was killed. Decoding the whole
    clip once at 96x60 grey is far cheaper than seeking into it three times,
    and it sees every frame rather than three of them -- which matters here,
    because the failure being looked for is a clip that is black THROUGHOUT.
    """
    r = subprocess.run(
        [FFMPEG, "-loglevel", "error", "-i", mp4, "-vf",
         "scale=96:60,format=gray", "-f", "rawvideo", "-"],
        capture_output=True, timeout=120)
    n = 96 * 60
    buf = r.stdout
    if len(buf) < 2 * n:
        return None
    F = np.frombuffer(buf[:len(buf) // n * n],
                      dtype=np.uint8).reshape(-1, 60, 96).astype(np.int16)
    # MAX frame-to-frame change, not the mean over every frame and pixel.
    #
    # The mean is diluted by dwell: task A holds 4.5 s at the bin so the arm
    # can arrive, and a small arm moving in a mostly-static frame produced a
    # whole-clip mean of 0.011-0.012 against a 0.02 floor -- two clips that
    # visibly move were failed as FROZEN. The question the check asks is "did
    # ANYTHING ever happen", and the max answers it: a genuinely frozen
    # capture scores 0 no matter how long it runs, while one moving moment is
    # enough to clear the floor.
    d = np.abs(np.diff(F, axis=0))
    return float(F.mean()), float(d.mean(axis=(1, 2)).max() if len(d) else 0.0)
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
    # THE A/B/C SWEEP CARRIES NO TASK OBJECTS, and saying so is the honest
    # entry rather than the convenient one. `record_abc_sweep.py` drives the
    # arms through each mode's own command path; it does not run the scene
    # publisher, so no tray, ball or target is in the picture to find.
    # Requiring one would fail all twelve clips for a thing that was never
    # rendered, and inventing a requirement it happens to meet would be worse.
    #
    # These clips are therefore judged on what they DO evidence: a picture
    # that is not black, real colour variety, and motion between frames --
    # the same basis as f1 and f5, which also carry no object. What they
    # prove is that the mode moved the arms; the object-in-frame criterion is
    # NOT met by them and must not be claimed.
    "a": [], "b": [], "c": [],
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


def verify_clip(mp4, task):
    """Judge ONE clip. Returns a dict; `ok` is the verdict.

    Split out of main() so the negative control can drive the SAME code the
    real sweep uses. A self-test that exercises a parallel copy of the logic
    proves nothing about the copy that runs.
    """
    dur = duration(mp4)
    if dur < 2.0:
        return dict(ok=False, why="clip only %.1f s long" % dur,
                    duration_s=round(dur, 1))
    F = frames(mp4, (0.30, 0.55, 0.80))
    if len(F) < 2:
        return dict(ok=False, why="could not extract frames",
                    duration_s=round(dur, 1))
    bright = float(np.mean([f.mean() for f in F]))
    variety = int(np.mean([len(np.unique(f.reshape(-1, 3), axis=0))
                           for f in F]))
    changed = float(np.abs(F[0] - F[-1]).mean())
    want = REQUIRED.get(task)
    why = []
    if want is None:
        # A TASK THIS VERIFIER DOES NOT KNOW IS NOT A PASS. Before this, an
        # unrecognised key made REQUIRED.get(task, []) return an empty list,
        # the object loop ran zero times, `missing` was empty and the clip
        # passed its object check WITHOUT ONE EVER HAPPENING -- the same
        # 0-of-0 shape that reads exactly like a clean result.
        why.append("unknown task %r -- no object requirement is defined, so "
                   "nothing was checked" % task)
        want = []
    found = {k: max(colour_hits(f, k) for f in F) for k in want}
    missing = [k for k, v in found.items() if v < floor_for(k)]
    if bright < 8:
        why.append("picture is black (mean %.1f)" % bright)
    if variety < 500:
        why.append("almost no colour variety (%d)" % variety)
    # FROZEN, not merely still. T9's zero-sway scenario is a stationary arm
    # holding a world-fixed point ON PURPOSE, and failing it for not moving
    # would be failing the task for doing what it says. Only a genuinely
    # frozen capture -- identical frames -- is an error.
    if changed < 0.05:
        why.append("capture is FROZEN (frame delta %.3f)" % changed)
    if missing:
        why.append("object not visible: %s" % ", ".join(missing))
    return dict(ok=not why, why="; ".join(why), duration_s=round(dur, 1),
                bright=round(bright, 1), variety=variety,
                delta=round(changed, 2),
                objects={k: int(v) for k, v in found.items()})


# ===========================================================================
#  THE NEGATIVE CONTROL
# ===========================================================================
# THIS VERIFIER HAS BEEN MISCALIBRATED TWICE, AND BOTH TIMES IT FAILED CLIPS
# WHOSE OBJECTS WERE PLAINLY VISIBLE. RViz shades every surface, so a detector
# matched against a requested colour finds nothing; and the HUD text is white
# and orange-yellow, so it was counted AS the object -- 587 "ball" pixels on
# one frame of which 529 were letters.
#
# Both of those are now permanent CONTROLS rather than comments. Before any
# verdict is printed, the verifier is shown clips whose answers are known and
# must get every one right. A verifier that cannot fail a broken clip cannot
# pass a good one either: "0 clips failed" and "nothing was actually checked"
# print identically, and this file has produced the second while reporting the
# first.
#
# The clips are CONSTRUCTED, not rendered. Per the standing rule, synthetic
# input is trustworthy only where the ground truth is built rather than drawn:
# "there is a 200x200 patch of exactly this colour at exactly this position"
# is arithmetic. Nothing here claims to model RViz -- the RENDERED colours it
# uses were measured off real frames and are listed at the top of this file.
BG = (45, 45, 48)                     # the RViz background, from the config
BALL = (189, 165, 74)                 # rendered, not requested
SKIN = (170, 130, 100)                # shaded mannequin skin: a WEAK tan hit
HUD = (235, 235, 235)                 # the overlay text


def _clip(path, painter, n=36, size=(1600, 1000), fps=12):
    """Encode a clip from painted frames. Returns the mp4 path."""
    from PIL import Image as I, ImageDraw
    d = os.path.dirname(path)
    os.makedirs(d, exist_ok=True)
    frames_dir = os.path.join(d, "_src")
    os.makedirs(frames_dir, exist_ok=True)
    for i in range(n):
        im = I.new("RGB", size, BG)
        painter(ImageDraw.Draw(im), i, n, size)
        im.save(os.path.join(frames_dir, "%04d.png" % i))
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-framerate", str(fps),
                    "-i", os.path.join(frames_dir, "%04d.png"),
                    "-pix_fmt", "yuv420p", path], check=False)
    shutil.rmtree(frames_dir, ignore_errors=True)
    return path


def _noise(dr, size):
    """Enough colour variety to clear the `variety < 500` gate, so the
    controls fail for the ONE reason each is built to test and not for an
    unrelated one."""
    rnd = random.Random(7)
    for _ in range(900):
        x, y = rnd.randrange(size[0]), rnd.randrange(size[1])
        dr.rectangle([x, y, x + 3, y + 3],
                     fill=(rnd.randrange(60, 210), rnd.randrange(60, 210),
                           rnd.randrange(60, 210)))


def _good(dr, i, n, size):
    _noise(dr, size)
    x = 300 + int(500.0 * i / n)                     # it MOVES
    dr.ellipse([x, 620, x + 190, 810], fill=BALL)    # well below the HUD band


def _absent(dr, i, n, size):
    _noise(dr, size)
    x = 300 + int(500.0 * i / n)
    dr.ellipse([x, 620, x + 190, 810], fill=(90, 90, 95))   # grey, not the ball


def _hud_only(dr, i, n, size):
    """The object colour appears ONLY in the overlay band. A verifier that
    does not mask the HUD reads this as a large, perfectly present object."""
    _noise(dr, size)
    dr.rectangle([40, 30, 1400, 150], fill=BALL)
    dr.rectangle([40, 30, 1400, 60], fill=HUD)
    x = 300 + int(500.0 * i / n)
    dr.ellipse([x, 620, x + 60, 680], fill=(90, 90, 95))


def _skin_only(dr, i, n, size):
    """Wearer skin and no task object.

    SIZED TO THE MEASURED FALSE POSITIVE, not to a round number. A real frame
    containing the wearer and no object scores 54 tan / **7 yellow** pixels at
    the verifier's own 600 px sample scale, against floors of 120 and 20. The
    patch is therefore 10x10 at the 1600 px capture width, which is
    100 * (600/1600)^2 = 14 px once rescaled -- a genuine hit, comfortably
    below the floor.

    That "genuine hit" matters: the control has to prove the FLOOR rejects
    this, not that the detector is blind to it. My first attempt used a 46x46
    patch (297 px scaled), which is not skin -- it is an object -- and the
    verifier passed it correctly. The control was wrong, not the verifier.
    """
    _noise(dr, size)
    x = 300 + int(500.0 * i / n)
    dr.rectangle([x, 620, x + 10, 630], fill=SKIN)


def _frozen(dr, i, n, size):
    _noise(dr, size)
    dr.ellipse([400, 620, 590, 810], fill=BALL)      # identical every frame


def _black(dr, i, n, size):
    dr.rectangle([0, 0, size[0], size[1]], fill=(0, 0, 0))


CONTROLS = [
    # (name, painter, task, must_pass, what it proves)
    ("good_clip", _good, "t3_ball", True,
     "an object that IS there passes"),
    ("object_absent", _absent, "t3_ball", False,
     "an object that is NOT there fails"),
    ("hud_text_only", _hud_only, "t3_ball", False,
     "HUD text is not counted as the object"),
    ("wearer_skin_only", _skin_only, "t3_ball", False,
     "the wearer's skin does not clear the object floor"),
    ("frozen_capture", _frozen, "t3_ball", False,
     "a frozen capture is caught"),
    ("black_capture", _black, "t3_ball", False,
     "a black capture is caught"),
    ("unknown_task", _good, "no_such_task", False,
     "an unrecognised task is NOT a silent pass"),
]


def check_angles(d):
    """Every angle in a clip directory must contain a moving picture.

    SPLIT OUT SO A CONTROL CAN DRIVE IT. The black-capture control below used
    to build only rviz_front.mp4 and call verify_clip() directly, so it proved
    the FRONT view's blackness check worked and said nothing about the other
    seven files -- which is exactly where the failure was. Seven gripper views
    were black for 34 s each and the tool reported 24 of 24 passing, with a
    green self-test underneath it the whole time. A control that exercises a
    different code path from the one that runs is not a control.
    """
    dark = []
    for ang in ANGLES:
        f = os.path.join(d, "rviz_%s.mp4" % ang)
        if not os.path.exists(f):
            continue
        st = angle_stats(f)
        if st is None:
            dark.append("%s: unreadable" % ang)
            continue
        b, ch = st
        if b < 8:
            dark.append("%s BLACK (mean %.1f)" % (ang, b))
        elif ch < 0.02:
            dark.append("%s FROZEN (delta %.3f)" % (ang, ch))
    return dark


def angle_self_test(verbose=True):
    """A clip whose FRONT is perfect and whose GRIPPER is black must FAIL.

    This is the exact shape of the seven clips that shipped. It is a separate
    control from the ones above because it is a property of the DIRECTORY, not
    of a single file, and main() refuses to report unless it passes.
    """
    tmp = tempfile.mkdtemp(prefix="anglectl_")
    d = os.path.join(tmp, "clip")
    os.makedirs(d, exist_ok=True)
    try:
        _clip(os.path.join(d, "rviz_front.mp4"), _good)
        _clip(os.path.join(d, "rviz_gripper.mp4"), _black)
        dark = check_angles(d)
        caught = any("gripper BLACK" in x for x in dark)
        # And the converse: an all-good directory must NOT be flagged, or the
        # check would fail everything and still look vigilant.
        d2 = os.path.join(tmp, "clip_ok")
        os.makedirs(d2, exist_ok=True)
        _clip(os.path.join(d2, "rviz_front.mp4"), _good)
        _clip(os.path.join(d2, "rviz_gripper.mp4"), _good)
        clean = not check_angles(d2)
        if verbose:
            print("  %-34s %s" % ("black gripper is CAUGHT",
                                  "PASS" if caught else "FAIL"))
            print("  %-34s %s" % ("all-good clip is not flagged",
                                  "PASS" if clean else "FAIL"))
        return caught and clean
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def self_test(verbose=True):
    """Show the verifier clips whose answers are known. All must be right."""
    tmp = tempfile.mkdtemp(prefix="clipctl_")
    REQUIRED["t3_ball"] = ["ball_yellow"]
    ok_all = True
    if verbose:
        print("VERIFIER SELF-TEST -- constructed clips with known answers")
    try:
        for name, painter, task, must_pass, why in CONTROLS:
            mp4 = _clip(os.path.join(tmp, name, "rviz_front.mp4"), painter)
            r = verify_clip(mp4, task)
            good = (r["ok"] == must_pass)
            ok_all &= good
            if verbose:
                print("  %-18s want %-4s got %-4s  %-4s  %s%s"
                      % (name, "PASS" if must_pass else "FAIL",
                         "PASS" if r["ok"] else "FAIL",
                         "ok" if good else "BAD",
                         why,
                         "" if good else "   [verifier said: %s]"
                         % (r.get("why") or "nothing")))
    finally:
        REQUIRED.pop("t3_ball", None)
        shutil.rmtree(tmp, ignore_errors=True)
    if verbose:
        print("  -> %s\n" % ("ALL CONTROLS CORRECT" if ok_all
                             else "CONTROLS FAILED"))
    return ok_all


def main():
    if not self_test():
        print("\nREFUSING TO REPORT. The verifier failed a control, so any "
              "verdict it prints -- especially a pass -- means nothing.")
        return 2
    print("PER-ANGLE SELF-TEST -- a clip whose gripper view is black")
    if not angle_self_test():
        print("\nREFUSING TO REPORT. The per-angle check cannot catch a "
              "deliberately black gripper view. That is the exact failure "
              "that let seven black clips pass while this tool reported "
              "24 of 24.")
        return 2
    print()
    rows = []
    # ONE MORE PATH LEVEL. The tree is now
    # recordings/verification/<mode>/<task>/<scenario>/<condition>, because
    # control mode and autonomy condition are independent axes and the old layout
    # conflated them: "direct" under the mannequin and "direct" under VR shared a
    # folder name. A glob that still assumes three levels matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass.
    # FIND EVERY CLIP, AT ANY DEPTH. This globbed exactly four path levels
    # while the sweep writes three (<mode>/<task>/<scenario>), so it silently
    # verified 39 OLD clips, reported "39 of 39 pass", and CHECKED NONE OF THE
    # TWELVE THAT HAD JUST BEEN RECORDED. The self-test below was passing
    # throughout: controls prove the GRADING is honest, they say nothing about
    # the SCOPE. Those are two different lies and each needs its own guard.
    mp4s = sorted(glob.glob(os.path.join(OUT, "**", "rviz_front.mp4"),
                            recursive=True))
    # A rviz_front.mp4 that the walk misses is invisible; count the front
    # clips on disk independently and refuse if the two disagree.
    on_disk = 0
    for root, _dirs, files in os.walk(OUT):
        on_disk += sum(1 for f in files if f == "rviz_front.mp4")
    if on_disk != len(mp4s):
        print("REFUSING: walked %d rviz_front.mp4 on disk but the glob "
              "matched %d. A clip this tool cannot see is a clip it reports "
              "as passing." % (on_disk, len(mp4s)))
        return 2
    print("verifying %d rviz clips (%d found on disk)\n" % (len(mp4s), on_disk))
    for mp4 in mp4s:
        d = os.path.dirname(mp4)
        parts = os.path.relpath(d, OUT).split(os.sep)
        # FOUR LEVELS GLOBBED, FOUR LEVELS PARSED. This read
        # `task, scen, cond = parts[0], parts[1], parts[2]` against a tree that
        # is <mode>/<task>/<scenario>/<condition>, so `task` was the MODE --
        # "01_master_teleop" -- and REQUIRED.get(mode) returned an empty list.
        # The object check then ran zero times and every clip passed it
        # regardless of whether the object rendered at all. Caught by the
        # negative control below, which is what it is for.
        # The tree is <mode>/<task>/<scenario>[/<condition>] -- the sweep
        # writes three levels, the older recorder wrote four. Parse what is
        # actually there instead of assuming a depth.
        parts = parts + ["-"] * (4 - len(parts))
        mode, task, scen, cond = parts[0], parts[1], parts[2], parts[3]
        task = task.lower()
        r = verify_clip(mp4, task)

        # EVERY ANGLE, NOT JUST THE FRONT.
        #
        # This tool globbed rviz_front.mp4 and opened nothing else, so "24 of
        # 24 clips pass" meant 24 FRONT VIEWS passed -- while SEVEN gripper
        # views were entirely black (mean 1.0 of 255, zero changing frames,
        # 34 s of nothing) and one of them was the tight shot of the pick, the
        # single most informative angle in the set. A per-clip verdict that
        # silently covers one of eight files is the 0-of-0 failure again:
        # nothing was checked, and it read as a pass.
        #
        # Only the cheap picture checks run per angle. The object-colour check
        # stays on the front view, because the other cameras deliberately
        # frame the arm rather than the scene and a missing object there is
        # framing, not a fault.
        dark = check_angles(d)
        if dark:
            r["ok"] = False
            r["why"] = "; ".join([r["why"]] + dark).strip("; ")
        r["angles_bad"] = dark

        r.update(mode=mode, task=task, scenario=scen, condition=cond)
        rows.append(r)
        print("  %-18s %-4s %-14s %-9s %s%s"
              % (mode, task, scen, cond, "OK  " if r["ok"] else "FAIL",
                 "" if r["ok"] else "  <- " + r["why"]))

    good = [r for r in rows if r["ok"]]
    print("\n  %d of %d clips pass every automatic check" % (len(good), len(rows)))
    bad = [r for r in rows if not r["ok"]]
    if bad:
        print("  FAILING:")
        for r in bad:
            print("    %s/%s/%s -- %s"
                  % (r["task"], r["scenario"], r["condition"], r["why"]))
    os.makedirs(OUT, exist_ok=True)
    json.dump(rows, open(os.path.join(OUT, "rviz_verification.json"), "w"),
              indent=2)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())


# ---------------------------------------------------------------- subject
# THE TASK'S SUBJECT MUST BE IN THE FRAME.
#
# Two mode-06 clips passed EVERY check here while missing the thing the task
# is about: T0 showed both arms reaching for nothing -- no spheres, no bench --
# and T2 showed a tray with no ball on it. Every existing check was satisfied,
# because each asks "did something move / is it bright enough / is an object
# somewhere", and none asks "is the SUBJECT there".
SUBJECTS = {
    "t0": ("L1", "L2", "L3", "R1", "R2", "R3"),
    # THE PLANES ARE PART OF THE SUBJECT, not scenery. T1 is "blue cube to
    # blue plane, green cube to green plane": with no plane on screen the task
    # has no target and a wrong-colour placement cannot be scored, which is
    # exactly the omission this list exists to catch.
    "t1": ("cube_0", "cube_1", "cube_2", "cube_3",
           "plane_blue", "plane_green"),
    "t1s2": ("cube_left_0", "cube_left_1", "cube_right_0", "cube_right_1"),
    "t2": ("tray", "ball"),
    "t3": ("circuit_box", "multimeter"),
}


def subjects_present(clip_dir, task, events=None):
    """(ok, missing, detail) -- are the task's declared subjects recorded?

    Reads scene_events.json, which is the scene's OWN account of what it drew
    and attached. A subject absent from it was never in the picture either.

    NOT a pixel check: colour detection already exists and is per-object, and
    duplicating it here would give two answers to one question. What this adds
    is COMPLETENESS -- the pixel checks can only speak about objects they were
    told to look for.
    """
    want = SUBJECTS.get(task)
    if want is None:
        return False, [], "no subject list declared for task %r" % task
    if events is None:
        f = os.path.join(clip_dir, "scene_events.json")
        if not os.path.exists(f):
            return False, list(want), "no scene_events.json"
        try:
            events = json.load(open(f))
        except Exception as e:
            return False, list(want), "unreadable scene_events.json: %s" % e
    have = {i.get("item") for i in events.get("items", [])}
    have |= set(events.get("fixtures", []))
    missing = [w for w in want if w not in have]
    return (not missing), missing, "have=%s" % sorted(have)


def subject_control(verbose=True):
    """NEGATIVE CONTROL: a clip with furniture drawn and the SUBJECT suppressed
    must be caught, and a complete one must not be flagged.

    This is the exact shape of the two clips that shipped: the scene ran, the
    file was written, objects were listed -- and the task's own subject was not
    among them.
    """
    ok_full, miss_full, _ = subjects_present(
        "", "t2", events={"items": [{"item": "tray"}, {"item": "ball"}]})
    ok_part, miss_part, _ = subjects_present(
        "", "t2", events={"items": [{"item": "tray"}]})     # the shipped bug
    ok_t0, miss_t0, _ = subjects_present(
        "", "t0", events={"items": [], "fixtures": []})     # drew nothing
    caught = (not ok_part) and ("ball" in miss_part) and (not ok_t0)
    clean = ok_full and not miss_full
    if verbose:
        print("  %-34s %s" % ("tray without its BALL is caught",
                              "PASS" if caught else "FAIL"))
        print("  %-34s %s" % ("a complete subject list passes",
                              "PASS" if clean else "FAIL"))
    return caught and clean
