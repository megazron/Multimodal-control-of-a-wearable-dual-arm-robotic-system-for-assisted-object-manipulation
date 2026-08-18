#!/usr/bin/env python3
"""HOW TOLERANT MAY THE WAKE WORD BE? BOTH SIDES, MEASURED.

    python3 scripts/measure_wake_word.py
    python3 scripts/measure_wake_word.py --keep-wav --out ...

THE QUESTION. `voice_intent.strip_wake` accepts a transcript whose leading (or
trailing) tokens are within `WAKE_MAX_EDITS` letters of the wake phrase. Any
such threshold is a trade and it has exactly two failure directions:

    TOO TIGHT   an operator says the wake word, the transcriber spells it its
                own way, and the robot ignores a correct instruction. Measured
                on the shipped alias list: 13 of 40.
    TOO LOOSE   the robot acts on speech that was never addressed to it, which
                is the entire reason a wake word exists.

A number picked by looking at only the first of those is not a measurement, so
this speaks TWO corpora through the same Piper voice and the same
faster-whisper model `voice_listener` loads:

    WAKE      every instruction in `sweep_t1_instructions.CASES`, spoken with
              the wake phrase in front of it. These MUST be accepted.
    NO-WAKE   the same instructions spoken bare, plus overheard conversation
              of the kind a microphone in a lab actually picks up. These MUST
              NOT be accepted.

and sweeps the threshold across both. The reported number is the largest
threshold at which the no-wake corpus is still refused outright.

WHY THE NO-WAKE CORPUS INCLUDES THE INSTRUCTIONS THEMSELVES. They are the
hardest negatives available: they contain every word the grammar knows, so if
a loose threshold is going to fire on anything it will fire on these. Chatter
alone would have made the threshold look safer than it is.

THIS MEASURES A PIPELINE, NOT A PERSON. One synthetic voice, no accent, no
room, no noise. A real lab will need this re-run against real speakers before
the threshold can be called validated, and that is written into the output.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "src/srl_autonomy")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sweep_t1_instructions as SW                            # noqa: E402
from verify_voice_instruction import STT, synth, as_udp_pcm, VOICE  # noqa: E402
from srl_autonomy import voice_intent as vi                   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/wake_word.json")

# Speech a microphone in this lab would plausibly pick up with nobody
# addressing the robot. Deliberately includes near-misses on the wake phrase
# itself -- if "hey doc" alone is enough to wake it, the threshold is wrong.
CHATTER = [
    "hey can you pass me the screwdriver",
    "hey doc how long have you been running that",
    "the docking station is over by the window",
    "he took the blue one home yesterday",
    "okay so what happens if we just leave it",
    "duck under the frame there is a cable",
    "i think the green one is the spare",
    "hey are you recording this",
    "doctor ockham said the simplest answer usually wins",
    "that box on the trolley is not ours",
    "put the kettle on would you",
    "we should stop for lunch soon",
    "the pad on the left is a bit sticky",
    "did anyone move my cube of resin",
    "hey listen the fan is making that noise again",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wake", default=vi.WAKE_DEFAULT)
    ap.add_argument("--model", default="small")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--max-threshold", type=int, default=7)
    ap.add_argument("--keep-wav", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    if not os.path.exists(VOICE):
        print("REFUSING: no piper voice at %s" % VOICE)
        return 3

    scratch = os.environ.get("SRL_SCRATCH", "/tmp")
    wav_dir = os.path.join(scratch, "wake_cases")
    os.makedirs(wav_dir, exist_ok=True)

    utts = [("%s %s" % (a.wake, c[0]), True, "instruction with wake")
            for c in SW.CASES]
    utts += [(c[0], False, "instruction, bare") for c in SW.CASES]
    utts += [(t, False, "chatter") for t in CHATTER]

    stt = STT(a.model, device=a.device)
    print("WAKE WORD %r -- %d spoken utterances (%d wake, %d no-wake)"
          % (a.wake, len(utts), sum(1 for _u, w, _k in utts if w),
             sum(1 for _u, w, _k in utts if not w)))
    print("   faster-whisper %s on %s, loaded %.1f s\n"
          % (a.model, stt.device, stt.load_s))

    rows = []
    for i, (said, is_wake, kind) in enumerate(utts):
        wav = os.path.join(wav_dir, "u_%03d.wav" % i)
        synth(said, wav)
        text, ms = stt.hear(as_udp_pcm(wav))
        e, k, from_end = vi.wake_distance(text, a.wake)
        rows.append(dict(said=said, heard=text, is_wake=is_wake, kind=kind,
                         edits=e, tokens=k, from_end=from_end, stt_ms=ms))
        if not a.keep_wav:
            os.remove(wav)
        if (i + 1) % 20 == 0:
            print("   %d/%d spoken" % (i + 1, len(utts)))

    print("\nTHRESHOLD SWEEP")
    print("   edits  wake accepted      no-wake FALSELY accepted")
    table = []
    n_w = sum(1 for r in rows if r["is_wake"])
    n_n = len(rows) - n_w
    for th in range(0, a.max_threshold + 1):
        acc = sum(1 for r in rows if r["is_wake"] and r["edits"] is not None
                  and r["edits"] <= th)
        bad = sum(1 for r in rows if not r["is_wake"] and r["edits"] is not None
                  and r["edits"] <= th)
        table.append(dict(threshold=th, wake_accepted=acc, false_accepted=bad))
        print("   %5d  %3d of %-3d %5.0f%%   %3d of %-3d %5.0f%%   %s"
              % (th, acc, n_w, 100.0 * acc / max(1, n_w),
                 bad, n_n, 100.0 * bad / max(1, n_n),
                 "<-- shipped" if th == vi.WAKE_MAX_EDITS else ""))

    clean = [t for t in table if t["false_accepted"] == 0]
    best = max(clean, key=lambda t: t["threshold"]) if clean else None
    print("\n" + "=" * 72)
    if best is None:
        print("NO THRESHOLD, NOT EVEN 0, LEAVES THE NO-WAKE CORPUS ALONE. "
              "That is a fact about the wake PHRASE, not about the matcher: "
              "%r is too close to ordinary speech to be one." % a.wake)
    else:
        print("LARGEST THRESHOLD WITH ZERO FALSE ACCEPTS: %d edits  "
              "(accepts %d of %d spoken wake utterances, %.0f%%)"
              % (best["threshold"], best["wake_accepted"], n_w,
                 100.0 * best["wake_accepted"] / max(1, n_w)))
        print("SHIPPED: voice_intent.WAKE_MAX_EDITS = %d" % vi.WAKE_MAX_EDITS)
        if vi.WAKE_MAX_EDITS > best["threshold"]:
            print("   ** THE SHIPPED VALUE IS LOOSER THAN THE MEASUREMENT "
                  "SUPPORTS. **")
    missed = [r for r in rows
              if r["is_wake"] and (r["edits"] is None
                                   or r["edits"] > vi.WAKE_MAX_EDITS)]
    if missed:
        print("\nWAKE UTTERANCES THE SHIPPED THRESHOLD STILL REFUSES (%d):"
              % len(missed))
        for r in missed:
            print("   %2s edits  heard %r" % (r["edits"], r["heard"]))
    falses = [r for r in rows
              if not r["is_wake"] and r["edits"] is not None
              and r["edits"] <= vi.WAKE_MAX_EDITS]
    if falses:
        print("\nNO-WAKE UTTERANCES THE SHIPPED THRESHOLD WOULD ACCEPT (%d) "
              "-- EVERY ONE:" % len(falses))
        for r in falses:
            print("   %2d edits  said %r" % (r["edits"], r["said"]))
            print("             heard %r" % r["heard"])
    else:
        print("\nNo utterance without a wake phrase is accepted at the "
              "shipped threshold.")

    out = dict(wake=a.wake, model=a.model, device=stt.device,
               shipped_threshold=vi.WAKE_MAX_EDITS, table=table,
               recommended=None if best is None else best["threshold"],
               rows=rows,
               caveat="one synthetic voice, no accent, no room, no noise -- "
                      "this measures the PIPELINE and must be re-run against "
                      "real speakers before the threshold is validated")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
