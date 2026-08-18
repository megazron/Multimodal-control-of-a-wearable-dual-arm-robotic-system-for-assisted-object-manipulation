"""THE FRAME EXTRACTOR MUST LAND A FRAME ON THE GRASP, OR SAY IT CANNOT.

WHY THIS FILE EXISTS. `scripts/read_t1_frames.py` was added so the clips could
be LOOKED at, because the brief for this task ends "the frame is the evidence".
It mapped event wall-clocks through `scene_events.json`'s own `t0_wall`, which
is when the SCENE NODE started -- under mode 06 that is ~300 s before the
capture, because the arm goes and LOOKS first. Every grasp therefore landed at
+380 s inside a 48 s clip, every one was skipped as out of range, and the script
printed "10 frames" and exit 0: openings and finals only, no grasp anywhere. The
tool built to inspect the evidence had never produced the evidence, and nothing
said so.

The clock cannot be recovered after the fact, so the sweep now writes
`clip_meta.json` beside the footage and the mapping is

    clip time = card_hold_s + (event wall - grab_t0)

This pins the three answers that matter, against a REAL mp4 of known duration
with events placed at known offsets -- constructed ground truth, per the
standing rule, since a duration and an offset are arithmetic rather than
rendered:

    correct grab_t0     grasp frames extracted          exit 0
    the OLD t0_wall     every moment outside the clip   exit 1, and it SAYS so
    no clip_meta.json   refused by name                 exit 2

The middle row is the one that matters: it is the exact defect, and it now
fails.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCRIPT = os.path.join(ROOT, "scripts", "read_t1_frames.py")

CARD_S = 6.0
FOOTAGE_S = 30.0
GRAB_T0 = 1000000.0
# Two grasps and two releases, at known seconds into the CAPTURE.
EVENTS = [("GRASPED", "cube_0", "left", 4.0),
          ("RELEASED", "cube_0", "left", 8.0),
          ("GRASPED", "cube_1", "right", 14.0),
          ("RELEASED", "cube_1", "right", 18.0)]


def _ffmpeg():
    for c in ("ffmpeg", os.path.expanduser("~/.local/bin/ffmpeg")):
        if shutil.which(c) or os.path.exists(c):
            return c
    return None


def _clip(dirpath, grab_t0, with_meta=True):
    """A real mp4 of known length, plus the two json files a clip carries."""
    ff = _ffmpeg()
    for view in ("front", "quad", "iso", "gripper", "top"):
        mp4 = os.path.join(dirpath, "rviz_%s.mp4" % view)
        subprocess.run([ff, "-y", "-loglevel", "error", "-f", "lavfi",
                        "-i", "color=c=black:s=320x240:d=%.1f"
                        % (CARD_S + FOOTAGE_S),
                        "-c:v", "libx264", "-preset", "ultrafast",
                        "-pix_fmt", "yuv420p", mp4], check=True)
    ev = dict(task="t1",
              # DELIBERATELY the old, wrong reference: 300 s before the grab,
              # which is what a staged look costs and what fooled the reader.
              t0_wall=grab_t0 - 300.0,
              events=[dict(ev=e, item=i, arm=a, wall=grab_t0 + off)
                      for e, i, a, off in EVENTS])
    json.dump(ev, open(os.path.join(dirpath, "scene_events.json"), "w"))
    if with_meta:
        json.dump(dict(mode="06_full_autonomy", task="t1",
                       grab_t0=grab_t0, grab_t1=grab_t0 + FOOTAGE_S,
                       card_hold_s=CARD_S),
                  open(os.path.join(dirpath, "clip_meta.json"), "w"))


def _run(clip_dir, out_dir):
    r = subprocess.run([sys.executable, SCRIPT, clip_dir, out_dir],
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


@pytest.mark.skipif(_ffmpeg() is None, reason="no ffmpeg on this box")
def test_a_correct_clock_puts_frames_on_every_grasp_and_release():
    tmp = tempfile.mkdtemp(prefix="clipframes_ok_")
    try:
        clip = os.path.join(tmp, "S1")
        out = os.path.join(tmp, "frames")
        os.makedirs(clip)
        _clip(clip, GRAB_T0, with_meta=True)
        rc, txt = _run(clip, out)
        assert rc == 0, txt
        made = os.listdir(out)
        # One per event per view: 4 events x 5 views.
        for e, i, a, _off in EVENTS:
            want = "%s_%s_%s" % (e.lower(), i, a)
            hits = [f for f in made if want in f]
            assert len(hits) == 5, ("%s: %d frames, expected one per view\n%s"
                                    % (want, len(hits), txt))
        assert "NO GRASP FRAME" not in txt
        assert "FELL OUTSIDE" not in txt, txt
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


@pytest.mark.skipif(_ffmpeg() is None, reason="no ffmpeg on this box")
def test_the_old_scene_node_clock_now_FAILS_instead_of_looking_clean():
    """The exact defect: t0_wall as the reference, 300 s adrift.

    Fed the same clip with `grab_t0` moved to where `t0_wall` sits, every
    moment lands past the end of the footage. The old code dropped them in
    silence and exited 0 with openings and finals. It must now exit non-zero
    and name what fell outside.
    """
    tmp = tempfile.mkdtemp(prefix="clipframes_bad_")
    try:
        clip = os.path.join(tmp, "S1")
        out = os.path.join(tmp, "frames")
        os.makedirs(clip)
        _clip(clip, GRAB_T0, with_meta=True)
        meta = json.load(open(os.path.join(clip, "clip_meta.json")))
        meta["grab_t0"] = GRAB_T0 - 300.0          # the scene node's t0_wall
        json.dump(meta, open(os.path.join(clip, "clip_meta.json"), "w"))
        rc, txt = _run(clip, out)
        assert rc == 1, ("a clip whose events miss its footage must "
                         "fail\n%s" % txt)
        assert "FELL OUTSIDE THE FOOTAGE" in txt, txt
        assert "NO GRASP FRAME WAS EXTRACTED" in txt, txt
        made = os.listdir(out) if os.path.isdir(out) else []
        assert not [f for f in made if "grasped" in f], made
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


@pytest.mark.skipif(_ffmpeg() is None, reason="no ffmpeg on this box")
def test_a_clip_with_no_meta_is_refused_by_name():
    tmp = tempfile.mkdtemp(prefix="clipframes_nometa_")
    try:
        clip = os.path.join(tmp, "S1")
        out = os.path.join(tmp, "frames")
        os.makedirs(clip)
        _clip(clip, GRAB_T0, with_meta=False)
        rc, txt = _run(clip, out)
        assert rc == 2, txt
        assert "clip_meta.json" in txt
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
