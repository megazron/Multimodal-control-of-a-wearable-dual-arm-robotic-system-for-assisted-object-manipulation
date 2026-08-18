#!/usr/bin/env python3
"""T1 FROM A SPOKEN INSTRUCTION, THROUGH REAL AUDIO AND REAL SPEECH-TO-TEXT.

    python3 scripts/verify_voice_instruction.py
    python3 scripts/verify_voice_instruction.py --cases 8 --keep-wav

WHY THIS IS NOT `--ros-args -p source:=text`

`voice_listener` has always had a text source, and every autonomy test in this
repository uses it. That is the right default -- it makes the refusal matrix
deterministic -- but it means the SPOKEN half of "spoken or typed goal" has
never been exercised at all. The parser has been fed strings that a keyboard
produced. A microphone produces something else: dropped articles, a
transcriber's punctuation, "blue" heard as "blew", and a wake word that has to
survive being said out loud.

There is no microphone on this host and there cannot be one. MEASURED:
`/dev/snd` contains only `timer`, so ALSA has no capture device to open inside
WSL, and the shipped answer is a Windows-side sender posting PCM over UDP.
That sender needs a person to speak.

SO THE VOICE IS SYNTHESISED AND EVERYTHING AFTER IT IS REAL. Piper renders
each instruction to audio, the audio is resampled to the 16 kHz mono PCM the
UDP path carries, and faster-whisper -- the SAME model and compute type
`voice_listener` loads -- transcribes it. From the transcript on, the path is
the shipped one: `voice_intent.parse` then `t1_instruction.plan_from` against
what the camera saw.

WHAT THAT CAN AND CANNOT SHOW, said plainly:

  * IT CAN show that the grammar survives a real transcriber. A wake word the
    STT renders as "hey dr rock" is a refusal the typed sweep can never
    produce, and it is the single most likely thing to break in a lab.
  * IT CANNOT show robustness to a real speaker: one synthetic voice, no
    accent, no room, no background noise, no clipping. A perfect score here is
    evidence about the PIPELINE and not about a person.

Both halves are printed, and the second is printed whether or not the first
went well.

THE SCORING IS `sweep_t1_instructions.py`'s, imported rather than restated, so
the spoken and typed runs cannot drift into two definitions of CORRECT.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "src/srl_autonomy")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sweep_t1_instructions as SW                            # noqa: E402
import t1_instruction as TI                                   # noqa: E402
from srl_autonomy import voice_intent as vi                   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/voice_instruction.json")
VOICE = os.path.expanduser(
    "~/.local/share/piper_voices/en_US-lessac-medium.onnx")
SR = 16000


# ------------------------------------------------------------------- audio
def synth(text, wav_path):
    """Piper -> WAV. Raises if piper is not installed rather than skipping."""
    env = dict(os.environ)
    env["PATH"] = os.path.expanduser("~/.local/bin") + ":" + env.get("PATH", "")
    r = subprocess.run([sys.executable, "-m", "piper", "-m", VOICE,
                        "-f", wav_path], input=text.encode(), env=env,
                       capture_output=True)
    if r.returncode != 0 or not os.path.exists(wav_path):
        raise RuntimeError("piper failed: %s" % r.stderr.decode()[-400:])
    return wav_path


def as_udp_pcm(wav_path):
    """The WAV as the 16 kHz mono int16 the UDP path carries.

    Resampled here rather than asking piper for 16 kHz, because the sender on
    the Windows side captures at the device rate and resamples too -- so this
    is the same lossy step the real path takes, not a shortcut around it.
    """
    with wave.open(wav_path) as w:
        sr, ch = w.getframerate(), w.getnchannels()
        a = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    if ch > 1:
        a = a.reshape(-1, ch).mean(axis=1).astype("<i2")
    if sr != SR:
        n = int(round(len(a) * SR / float(sr)))
        a = np.interp(np.linspace(0.0, len(a) - 1.0, n),
                      np.arange(len(a)), a.astype(np.float64))
        a = a.astype("<i2")
    return a


class STT(object):
    """faster-whisper, loaded the way `voice_listener` loads it.

    device="auto" rather than "cuda". The shipped node hardcodes cuda, and on
    a host whose GPU is busy or absent that is an exception at construction
    with no fallback -- the node then publishes nothing, which is
    indistinguishable from an operator who said nothing. Recorded here and
    fixed in the node.
    """

    def __init__(self, model="small", compute_type="int8", device="auto"):
        from faster_whisper import WhisperModel
        import ctranslate2
        if device == "auto":
            device = "cuda" if ctranslate2.get_cuda_device_count() else "cpu"
        self.device = device
        t0 = time.monotonic()
        self.m = WhisperModel(model, device=device,
                              compute_type=compute_type
                              if device == "cuda" else "int8")
        self.load_s = round(time.monotonic() - t0, 2)

    def hear(self, pcm):
        t0 = time.monotonic()
        segs, _ = self.m.transcribe(pcm.astype("float32") / 32768.0,
                                    language="en")
        return (" ".join(s.text for s in segs).strip(),
                round((time.monotonic() - t0) * 1000.0, 1))


# ------------------------------------------------------------------ words
def wer(ref, hyp):
    """Word error rate, on the words that carry the instruction.

    Case and punctuation are stripped because the parser strips them too; a
    transcriber that writes "Put the blue ones on the blue pad." has made no
    error this pipeline can be hurt by.
    """
    r = vi.normalise(ref).split()
    h = vi.normalise(hyp).split()
    d = np.zeros((len(r) + 1, len(h) + 1), int)
    d[:, 0] = np.arange(len(r) + 1)
    d[0, :] = np.arange(len(h) + 1)
    for i in range(1, len(r) + 1):
        for j in range(1, len(h) + 1):
            d[i, j] = min(d[i - 1, j] + 1, d[i, j - 1] + 1,
                          d[i - 1, j - 1] + (r[i - 1] != h[j - 1]))
    return (float(d[len(r), len(h)]) / max(1, len(r)), int(d[len(r), len(h)]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=0,
                    help="0 = every case in sweep_t1_instructions")
    ap.add_argument("--wake", default=vi.WAKE_DEFAULT)
    ap.add_argument("--require-wake", action="store_true", default=True)
    ap.add_argument("--model", default="small")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--keep-wav", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    if not os.path.exists(VOICE):
        print("REFUSING: no piper voice at %s. There is no way to produce "
              "audio, and scoring the typed path under a spoken heading is "
              "exactly the substitution this file exists to avoid." % VOICE)
        return 3

    cases = SW.CASES if not a.cases else SW.CASES[:a.cases]
    scratch = os.environ.get("SRL_SCRATCH", "/tmp")
    wav_dir = os.path.join(scratch, "voice_cases")
    os.makedirs(wav_dir, exist_ok=True)

    print("SPOKEN T1 INSTRUCTIONS -- %d cases" % len(cases))
    stt = STT(a.model, device=a.device)
    print("   faster-whisper %s on %s, loaded in %.1f s"
          % (a.model, stt.device, stt.load_s))
    print("   wake word %r, spoken as part of every utterance" % a.wake)
    print("   scene: %s\n" % TI.describe(SW.SEEN))

    rows, tally = [], {SW.CORRECT: 0, SW.ASKED: 0, SW.REFUSED: 0,
                       SW.MISUNDERSTOOD: 0}
    words, errs, heard_exact = 0, 0, 0
    for i, case in enumerate(cases):
        utt, cat, intended, note = case
        said = "%s %s" % (a.wake, utt)
        wav = os.path.join(wav_dir, "case_%02d.wav" % i)
        synth(said, wav)
        pcm = as_udp_pcm(wav)
        text, ms = stt.hear(pcm)
        rate, n_err = wer(said, text)
        words += len(vi.normalise(said).split())
        errs += n_err
        heard_exact += 1 if n_err == 0 else 0
        # THE PARSE IS THE SHIPPED ONE, ON THE TRANSCRIPT, WITH THE WAKE WORD
        # REQUIRED -- which is what a microphone path does and what the typed
        # path deliberately does not.
        o = TI.plan_from(text, SW.SEEN, wake=a.wake,
                         require_wake=a.require_wake)
        if o.kind == TI.ASK:
            verdict, why = SW.ASKED, ""
        elif o.kind == TI.REFUSE:
            verdict, why = SW.REFUSED, ""
        else:
            got = SW._pairs(o)
            if intended is None:
                verdict, why = SW.MISUNDERSTOOD, (
                    "acted on an instruction whose only safe outcomes were "
                    "ask or refuse; it moved %s" % sorted(got))
            elif got == set(intended):
                verdict, why = SW.CORRECT, ""
            else:
                verdict, why = SW.MISUNDERSTOOD, (
                    "meant %s, planned %s" % (sorted(set(intended)),
                                              sorted(got)))
        tally[verdict] += 1
        rows.append(dict(case=utt, category=cat, said=said, heard=text,
                         wer=round(rate, 3), word_errors=n_err, stt_ms=ms,
                         verdict=verdict, why=why,
                         plan=o.as_dict(), audio_s=round(len(pcm) / float(SR), 2)))
        flag = "  <-- MISUNDERSTOOD" if verdict == SW.MISUNDERSTOOD else ""
        print("%2d %-13s %-13s wer %.2f  %.0f ms  heard %r%s"
              % (i, cat, verdict, rate, ms, text, flag))
        if not a.keep_wav:
            os.remove(wav)

    n = len(cases)
    print("\n" + "=" * 72)
    print("SPOKEN: %d correct / %d asked / %d refused / %d MISUNDERSTOOD"
          % (tally[SW.CORRECT], tally[SW.ASKED], tally[SW.REFUSED],
             tally[SW.MISUNDERSTOOD]))
    print("TRANSCRIPTION: %d of %d utterances heard word for word, "
          "WER %.3f over %d words" % (heard_exact, n, errs / float(max(1, words)),
                                      words))
    if tally[SW.MISUNDERSTOOD]:
        print("\nMISUNDERSTOOD, EVERY ONE, IN FULL:")
        for r in rows:
            if r["verdict"] == SW.MISUNDERSTOOD:
                print("   said  %r" % r["said"])
                print("   heard %r" % r["heard"])
                print("   %s\n" % r["why"])
    else:
        print("MISUNDERSTOOD: none. Nothing was acted on that should not "
              "have been.")
    print("\nWHAT THIS IS NOT EVIDENCE ABOUT: one synthetic voice, no accent, "
          "no room, no noise, no clipping. It measures the PIPELINE, not a "
          "speaker.")

    out = dict(model=a.model, device=stt.device, wake=a.wake,
               require_wake=a.require_wake, sample_rate=SR,
               voice=os.path.basename(VOICE), n_cases=n, tally=tally,
               wer=round(errs / float(max(1, words)), 4),
               exact_transcripts=heard_exact, rows=rows,
               caveat="synthesised speech; not evidence about real speakers")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
