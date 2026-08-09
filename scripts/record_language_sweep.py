#!/usr/bin/env python3
"""Full autonomy answering 40 phrasings, on video, with every outcome captioned.

WHY THIS IS THE MOST COMPLETE EVIDENCE IN THE SET. Full autonomy is the only
mode that needs no operator, so it is the only one where the whole input space
can be exercised on camera without a human in the loop.

WHAT IS ON SCREEN IS THE REAL DECISION. Every card is produced by running the
SHIPPED path -- `voice_intent.parse` then `resolve_target` against a
constructed scene -- exactly as `scripts/sweep_language_vision.py` does. The
robot's spoken reply and the outcome class are read off that call, not
written by hand.

WHAT THIS VIDEO DOES NOT SHOW, and the last card says so: the arm moving. For
CORRECT outcomes the executive would announce, wait for confirmation and then
drive the arm; for ASKED and REFUSED it deliberately moves nothing at all,
which is the entire point of those categories. Filming a stationary arm for
30 of 40 utterances would be 30 identical shots of nothing happening. The
motion clips are the five mode sweeps; this is the decision evidence.

    python3 scripts/record_language_sweep.py
"""
import json
import os
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_autonomy"))
import sweep_language_vision as SW                           # noqa: E402

OUT = os.path.join(WS, "recordings/verification/06_full_autonomy/LANGUAGE")
FFMPEG = os.environ.get("FFMPEG") or os.path.expanduser("~/.local/bin/ffmpeg")
W, H, FPS = 1280, 720, 10
SEC = 2.6

BG = (7, 11, 15)
WHITE = (238, 244, 248)
CYAN = (99, 200, 216)
AMBER = (232, 163, 61)
RED = (255, 90, 105)
VIOLET = (160, 108, 224)
MUTED = (92, 115, 130)

COLOUR = {"CORRECT": CYAN, "ASKED": AMBER, "REFUSED": VIOLET,
          "MISUNDERSTOOD": RED}
MEANING = {
    "CORRECT": "resolved what the speaker meant",
    "ASKED": "ambiguous -- asks, and picks NOTHING          [SAFE]",
    "REFUSED": "declines and says why                        [SAFE]",
    "MISUNDERSTOOD": "confidently did something else       [DANGEROUS]",
}


def font(sz, bold=False):
    from PIL import ImageFont
    p = ("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf"
         % ("-Bold" if bold else ""))
    try:
        return ImageFont.truetype(p, sz)
    except OSError:
        return ImageFont.load_default()


def card(path, lines, rule=None):
    from PIL import Image, ImageDraw
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    d.line([(60, 96), (W - 60, 96)], fill=(31, 90, 102), width=1)
    if rule:
        d.rectangle([0, 0, 10, H], fill=rule)
    y = 46
    for txt, col, sz, bold in lines:
        d.text((66, y), txt, fill=col, font=font(sz, bold))
        y += int(sz * 1.85)
    im.save(path)


def main():
    os.makedirs(OUT, exist_ok=True)
    frames = os.path.join(OUT, "_frames")
    os.makedirs(frames, exist_ok=True)
    for f in os.listdir(frames):
        os.remove(os.path.join(frames, f))

    if not SW.self_test():
        print("REFUSING: the language harness failed its own negative "
              "control, so nothing it reports is worth filming.")
        return 2

    rows, n = [], 0
    counts = {}
    scene = ", ".join(sorted(SW.PRESENT))

    def emit(lines, rule, seconds=SEC):
        nonlocal n
        for _ in range(int(seconds * FPS)):
            card(os.path.join(frames, "%05d.png" % n), lines, rule)
            n += 1

    emit([("FULL AUTONOMY  ---  FREE-FORM LANGUAGE", WHITE, 34, True),
          ("", WHITE, 10, False),
          ("40 phrasings, 13 categories, run through the SHIPPED grammar.",
           CYAN, 19, False),
          ("Most were written against the parser's STRUCTURE, not alongside",
           MUTED, 17, False),
          ("it -- negation, relational reference, superlatives, compounds,",
           MUTED, 17, False),
          ("misspellings, politeness, and objects that are not there.",
           MUTED, 17, False),
          ("", WHITE, 10, False),
          ("SCENE: " + scene, WHITE, 18, False)], CYAN, 4.5)

    cur = None
    for utt, cat, wv, wl, note in SW.CASES:
        verdict, got, detail = SW.classify(utt, wv, wl)
        counts[verdict] = counts.get(verdict, 0) + 1
        rows.append(dict(utterance=utt, category=cat, verdict=verdict,
                         resolved=got, note=note))
        col = COLOUR[verdict]
        if cat != cur:
            cur = cat
            emit([("CATEGORY", MUTED, 18, False),
                  (cat.replace("-", " ").upper(), WHITE, 40, True)],
                 CYAN, 1.6)
        say = detail.get("reason") or detail.get("note") or ""
        lines = [("SAID", MUTED, 15, False),
                 ('"%s"' % utt, WHITE, 24, True),
                 ("", WHITE, 8, False),
                 ("OUTCOME", MUTED, 15, False),
                 (verdict, col, 34, True),
                 (MEANING[verdict], col, 17, False),
                 ("", WHITE, 8, False)]
        if got:
            lines.append(("would act on:  %s" % got, WHITE, 19, False))
        if say:
            lines.append(("robot: %s" % str(say)[:76], MUTED, 16, False))
        lines.append(("why this case exists: %s" % note[:70], MUTED, 15, False))
        emit(lines, col)

    tot = sum(counts.values())
    emit([("RESULT", WHITE, 34, True),
          ("", WHITE, 10, False)] +
         [("%-14s %3d   %s" % (k, counts.get(k, 0),
                               "" if k != "MISUNDERSTOOD" else
                               "<-- the dangerous category"),
           COLOUR[k], 24, k == "MISUNDERSTOOD")
          for k in ("CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD")] +
         [("", WHITE, 10, False),
          ("ASKED and REFUSED are SUCCESSES: the robot moved nothing.",
           CYAN, 19, False),
          ("%d of %d utterances ended with the arm untouched." %
           (counts.get("ASKED", 0) + counts.get("REFUSED", 0), tot),
           MUTED, 18, False)], CYAN, 6.0)

    emit([("WHAT THIS VIDEO DOES NOT SHOW", WHITE, 30, True),
          ("", WHITE, 10, False),
          ("The arm moving. For ASKED and REFUSED the robot deliberately",
           MUTED, 18, False),
          ("moves nothing -- that is the whole point of those outcomes --",
           MUTED, 18, False),
          ("and filming 30 identical shots of a stationary arm would be",
           MUTED, 18, False),
          ("evidence of nothing. The motion is in the five mode sweeps;",
           MUTED, 18, False),
          ("this is the DECISION evidence.", MUTED, 18, False),
          ("", WHITE, 10, False),
          ("Every card above is the SHIPPED grammar's real answer.",
           CYAN, 19, False)], MUTED, 6.0)

    mp4 = os.path.join(OUT, "language_sweep.mp4")
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", os.path.join(frames, "%05d.png"),
                    "-c:v", "libx264", "-preset", "veryfast",
                    "-pix_fmt", "yuv420p", mp4], check=False)
    for f in os.listdir(frames):
        os.remove(os.path.join(frames, f))
    os.rmdir(frames)
    json.dump(dict(counts=counts, rows=rows),
              open(os.path.join(OUT, "language_sweep.json"), "w"), indent=2)
    ok = os.path.exists(mp4) and os.path.getsize(mp4) > 20000
    print("%s  %d cards, %.0f s  -> %s"
          % ("OK" if ok else "FAILED", len(rows), n / float(FPS), mp4))
    print("   " + "  ".join("%s %d" % (k, v) for k, v in sorted(counts.items())))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
