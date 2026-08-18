#!/usr/bin/env python3
"""PULL FRAMES OUT OF A T1 CLIP SO THEY CAN BE LOOKED AT.

    python3 scripts/read_t1_frames.py recordings/verification/06_full_autonomy/T1/S1_both_arms_centre

The brief for this task ends "the frame is the evidence", and every automatic
check in this repo has at some point passed a clip that was visibly wrong --
the render check that filed a scene with no table in it at 18% ink is the
worst example. So this does not judge anything. It extracts frames at the
moments that matter and prints where they are, and a person (or a model that
can see) looks at them.

WHICH MOMENTS. Not evenly spaced: evenly spaced frames of a 30 s clip are
mostly the arm traversing, which is the part nothing goes wrong in. The
interesting instants are named by the clip's OWN event log --
scene_events.json carries a wall clock on every GRASPED and RELEASED -- so
the frames come out at the card, each grasp, each release, and the final
state.

WHICH CLOCK, WHICH IS WHERE THIS WENT WRONG. scene_events stamps wall time
against `t0_wall`, and `t0_wall` is when the SCENE NODE started -- which under
06 is before a ~300 s staged look. Its offsets therefore run to +380 s inside
a 48 s clip, every one of them was silently dropped as out of range, and this
script reported "10 frames" made entirely of openings and finals. The
extractor written to look at the evidence had never once produced the frames
that ARE the evidence, and it said nothing, because a moment outside the clip
was skipped rather than counted.

The capture clock is not derivable after the fact, so the sweep now writes it
beside the footage in `clip_meta.json`:

    clip time = card_hold_s + (event wall - grab_t0)

A clip with no `clip_meta.json` predates that and CANNOT be mapped. This
refuses it by name rather than falling back to the opening frame, because
"10 frames" that contain no grasp is the failure this file exists to end.
"""
import json
import math
import os
import subprocess
import sys


def probe_duration(mp4):
    """Seconds, read out of ffmpeg's own banner.

    THIS BOX HAS NO ffprobe -- only ffmpeg, in ~/.local/bin -- and assuming
    the pair travel together is how a frame grab silently produces nothing.
    ffmpeg with no output file prints the duration to stderr and exits
    non-zero, which is expected and is not an error here.
    """
    try:
        txt = subprocess.run(["ffmpeg", "-i", mp4],
                             capture_output=True, text=True).stderr
    except Exception:                                         # noqa: BLE001
        return None
    for line in txt.splitlines():
        if "Duration:" in line:
            hms = line.split("Duration:")[1].split(",")[0].strip()
            try:
                h, m, sec = hms.split(":")
                return int(h) * 3600 + int(m) * 60 + float(sec)
            except ValueError:
                return None
    return None


def grab(mp4, t, dest):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "%.2f" % t,
                    "-i", mp4, "-frames:v", "1", dest], check=False)
    return os.path.exists(dest) and os.path.getsize(dest) > 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1].rstrip("/")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.environ.get("SRL_SCRATCH", "/tmp"), "t1_frames",
        os.path.basename(os.path.dirname(d)) + "_" + os.path.basename(d))
    os.makedirs(out, exist_ok=True)

    ev_path = os.path.join(d, "scene_events.json")
    ev = {}
    if os.path.exists(ev_path):
        ev = json.load(open(ev_path))

    # THE TWO NUMBERS THAT MAKE THE CLOCKS COMPARABLE, from the clip itself.
    meta_path = os.path.join(d, "clip_meta.json")
    if not os.path.exists(meta_path):
        print("NO clip_meta.json in %s" % d)
        print("This clip does not record when its capture started, so no "
              "event in scene_events.json can be placed in the footage.")
        print("scene_events' own t0_wall is the SCENE NODE's start, which "
              "under 06 is ~300 s before the capture -- using it puts every "
              "grasp outside the clip.")
        print("Re-record it: the sweep writes clip_meta.json now.")
        return 2
    meta = json.load(open(meta_path))
    grab_t0 = meta.get("grab_t0")
    card_s = meta.get("card_hold_s")
    # THE LOOK IS PART OF THE LEAD-IN. Under 06 the clip runs
    # card -> look -> task, and the task's own events are stamped against the
    # capture that began when the TASK started moving. Leaving the look out of
    # the offset puts every grasp ~50 s early, which lands them in the middle
    # of the observe move -- frames that look plausible and are of the wrong
    # thing, which is worse than none.
    look_s = meta.get("look_s") or 0.0
    if grab_t0 is None or card_s is None:
        print("clip_meta.json has no grab_t0/card_hold_s: %r" % (meta,))
        return 2
    lead_s = card_s + look_s

    # WHEN did each thing happen, IN SECONDS FROM THE START OF THE VIDEO.
    #
    # TWO OPENING FRAMES, because the card is prepended to EVERY angle and a
    # single frame at t=0.3 s is the card on all of them. That is how the
    # first pass at looking at these clips produced ten frames of which five
    # were the same words: the state the task STARTS in -- four cubes resting
    # on the table, nothing on the pads -- had no frame at all, and it is half
    # of what the brief asks to see.
    moments = [("00_card", 0.30)]
    if look_s:
        moments.append(("01_the_look", card_s + look_s * 0.5))
    moments.append(("02_scene_before_the_first_grasp", lead_s + 0.40))
    skipped = []
    for e in ev.get("events", []):
        if not e.get("wall"):
            continue
        off = lead_s + (e["wall"] - grab_t0)
        moments.append(("%s_%s_%s" % (e["ev"].lower(), e.get("item", "?"),
                                      e.get("arm", "?")), off))

    made = []
    for view in ("front", "quad", "iso", "gripper", "top"):
        mp4 = os.path.join(d, "rviz_%s.mp4" % view)
        if not os.path.exists(mp4):
            continue
        dur = probe_duration(mp4) or 0.0
        # THE CARD IS THE FIRST SECONDS OF THE FRONT VIEW and the footage
        # follows it, so the opening frame of `front` is the card and the
        # opening frame of every other view is the scene.
        want = list(moments) + [("99_final", max(0.0, dur - 0.40))]
        for name, t in want:
            # A MOMENT THAT FALLS OUTSIDE THE CLIP IS REPORTED, NOT DROPPED.
            # Silently skipping them is what let this print a frame count
            # that looked like a success while containing no grasp at all.
            if t is None or t < 0 or t > dur:
                skipped.append((view, name, t, dur))
                continue
            dest = os.path.join(out, "%s__%s.png" % (view, name))
            if grab(mp4, t, dest):
                made.append(dest)
        print("%-8s %5.1f s  ->  %d frames" % (view, dur, len(made)))

    if skipped:
        print("\n%d MOMENT(S) FELL OUTSIDE THE FOOTAGE:" % len(skipped))
        for view, name, t, dur in skipped[:12]:
            print("   %-8s %-28s at %+.1f s of %.1f s" % (view, name, t, dur))

    print("\n%d frames in %s" % (len(made), out))
    for p in sorted(made):
        print("   " + p)

    # A GRASP FRAME IS THE POINT. Openings and finals alone are the exact
    # non-answer this script used to give.
    n_grasp = len([p for p in made if "grasped" in os.path.basename(p)])
    if not n_grasp:
        print("\nNO GRASP FRAME WAS EXTRACTED -- the clip's events do not "
              "land inside its own footage. That is a defect in the clip or "
              "in its clock, not a clean run.")
        return 1
    print("\n%d grasp frame(s), %d release frame(s)"
          % (n_grasp,
             len([p for p in made if "released" in os.path.basename(p)])))
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
