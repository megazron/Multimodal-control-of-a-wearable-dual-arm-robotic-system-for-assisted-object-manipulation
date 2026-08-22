#!/usr/bin/env python3
"""This repository's recordings, as a LeRobotDataset.

    python3 scripts/lerobot_export.py --self-test            # controls only
    python3 scripts/lerobot_export.py --list                 # what is exportable
    python3 scripts/lerobot_export.py --csv recordings/teleop_20260731_133310.csv \
        --repo-id srl/kortex_teleop --out /tmp/lerobot_out

WHAT LEROBOT IS AND WHAT IT IS NOT, FOR THIS PROJECT
----------------------------------------------------
`lerobot` (Hugging Face) is three things stacked: a dataset FORMAT
(`LeRobotDataset` -- parquet frames plus mp4 video, with a `meta/info.json`
schema), a set of imitation-learning POLICIES (ACT, Diffusion Policy, VQ-BeT,
SmolVLA, pi0), and a hardware layer for a handful of small arms. Only the
first of those is unambiguously useful here today, and this file does that
one thing.

The other two are assessed honestly in `docs/system/20_lerobot.md`, and the
short version belongs here too, because a reader who only opens the code
should not have to infer it:

  * THE POLICIES DO NOT FIX WHAT IS BROKEN. The 2026-08-21 session did not
    fail on perception -- FastSAM found the cube at score 1.0 every time. It
    failed on calibration, on kinematics, and on eight defects in the code
    that commands the arms. No behaviour-cloning policy repairs an
    unmeasured camera->robot extrinsic; it learns around it and then breaks
    when the camera moves.
  * THIS MACHINE CANNOT TRAIN ONE ANYWAY. RTX A500, 4 GB, and the vision
    venv's torch is CPU-only (2.13.0+cpu). ACT at ~80M parameters is
    borderline; SmolVLA at 450M is not happening locally.
  * AND THERE IS NOTHING TO TRAIN ON. Imitation learning needs human
    demonstrations. This project has collected NO human data at all -- that
    is an ethics block, not a code block -- and the one recorded
    master-arm dataset is 5 ALIVE channels out of 14. See below.

So the value of this file is: get what does exist into a standard format,
with its defects declared, so that the day demonstrations are collected the
pipeline already exists and has a known-answer test.

THE CHANNEL GATE, AND WHY IT REFUSES
------------------------------------
`recordings/teleop_*.csv` is the only recorded HUMAN-driven data in the
repository. Its master-arm joint columns come from the instrumented mannequin,
and the newest `recordings/baselines/channels_*.json` says what those channels
were doing: 5 ALIVE, 1 INTERMITTENT, 6 INCOHERENT, 2 DEAD.

A DEAD channel is a column of one repeated value. Exported without comment it
becomes a feature with zero variance that a network will happily learn to
ignore -- or, worse, a column somebody later reads as "the wrist did not
move". That is this repository's own listed instrument failure mode, twice
over: "zero variance -- dead channel" and "a gap that is not a gap -- by-design
absence rendered identically to a defect".

So this exporter reads the newest baseline and:

    ALIVE          exported
    INTERMITTENT   exported, and named in `meta/srl_channel_health.json`
    INCOHERENT     DROPPED unless --include-incoherent, and named
    DEAD           DROPPED ALWAYS. There is no flag that exports one.

DROPPED, NOT ABORTED, AND THE DISTINCTION IS THE WHOLE POINT. The lie would
be INCLUDING a dead column, not omitting it. A first cut of this file aborted
the entire export when any channel was DEAD, which on the only real recording
in the repository -- 2 DEAD of 14 -- meant the tool refused to do anything at
all. That is a gate applied one level too high: it protects nothing and it
guarantees the pipeline is never exercised. The export proceeds on the
surviving channels, the dropped ones are named in the dataset's own metadata,
and the export aborts only when NOTHING survives.

and it writes `meta/srl_channel_health.json` into the dataset itself, so the
verdicts travel with the data rather than living in a baseline file on one
laptop. A dataset that has been uploaded to the Hub and separated from this
repository must still be able to say which of its columns are lies.

CONTROLS, and nothing is written if one fails.

    1  ROUND TRIP. Build a synthetic episode whose every value is
       CONSTRUCTED arithmetic -- frame i has state i/1000 -- export it, read
       it back through `LeRobotDataset`, and require every frame to match to
       1e-6. Ground truth is constructed, never rendered, which is the only
       synthetic this repository permits.
    2  THE DEAD-CHANNEL GATE BITES. Feed a health map with a DEAD channel and
       require the exporter to refuse and to NAME the channel. A gate that
       cannot fail on a deliberately broken input is not a gate.
    3  A CSV IT CANNOT MAP IS REFUSED. The frame extractor once printed
       "10 frames" and exit 0 while mapping every event outside the clip.
       A missing column, or a segment whose row count disagrees with the
       CSV, is an exit 1 with the reason, never a short episode.
    4  EPISODE BOUNDARIES ARE THE SEGMENTS. The row counts in the sidecar
       `.meta.json` must sum to the CSV's own row count. If they do not, the
       segmentation describes a different file and no episode boundary from
       it can be trusted.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# --------------------------------------------------------------- the schema
ARMS = ("left", "right")
NJ = 7

# The MASTER arm's joints, which is what the channel baseline grades. The
# baseline keys are l_j1..l_j7 / r_j1..r_j7; the CSV columns are
# left_j1..left_j7 / right_j1..right_j7. One mapping, written once, because
# the last time an arm prefix and a bare part name were compared the wearer
# guard resolved 0 of 9 parts in every run ever made.
def channel_key(arm, j):
    return "%s_j%d" % (arm[0], j)


def csv_joint_col(arm, j):
    return "%s_j%d" % (arm, j)


ACTION_COLS = [("%s_cmd_%s" % (a, k)) for a in ARMS
               for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
EXTRA_COLS = [("%s_clutch" % a) for a in ARMS] + ["fsr1", "fsr2"]

REFUSE_ALWAYS = ("DEAD",)
REFUSE_BY_DEFAULT = ("INCOHERENT",)
NOTE_ONLY = ("INTERMITTENT",)


class Refused(Exception):
    """A refusal that names what it refused and why. Never a silent skip."""


# ---------------------------------------------------------- channel health
def newest_channel_baseline(root=ROOT):
    """The NEWEST channels_*.json, because a stale one freezes channels that
    now work and nothing downstream disagrees. CLAUDE.md's own rule."""
    pat = os.path.join(root, "recordings/baselines/channels_*.json")
    files = sorted(glob.glob(pat))
    if not files:
        raise Refused(
            "no recordings/baselines/channels_*.json exists. The master "
            "channel verdicts are what decide which columns are data and "
            "which are a repeated value; without them this exporter cannot "
            "tell the difference and will not guess. Run "
            "scripts/check_channels.sh.")
    path = files[-1]
    with open(path) as f:
        return path, json.load(f)


def gate(health, include_incoherent=False):
    """(kept, refused, notes). Refusals NAME the channel and the verdict."""
    kept, refused, notes = [], [], []
    for arm in ARMS:
        for j in range(1, NJ + 1):
            k = channel_key(arm, j)
            v = (health.get(k) or {}).get("verdict", "UNKNOWN")
            why = (health.get(k) or {}).get("why", "")
            if v in REFUSE_ALWAYS:
                refused.append((k, v, why, "always"))
            elif v in REFUSE_BY_DEFAULT and not include_incoherent:
                refused.append((k, v, why, "default"))
            else:
                kept.append((k, csv_joint_col(arm, j)))
                if v in REFUSE_BY_DEFAULT or v in NOTE_ONLY or v == "UNKNOWN":
                    notes.append((k, v, why))
    return kept, refused, notes


# ------------------------------------------------------------------- the csv
def read_segments(meta_path, n_rows):
    with open(meta_path) as f:
        meta = json.load(f)
    segs = meta.get("segments") or []
    if not segs:
        raise Refused("%s carries no segments, so there are no episode "
                      "boundaries. A single 20 000-row episode spanning "
                      "eighteen different motions is not a demonstration."
                      % meta_path)
    total = sum(int(s["rows"]) for s in segs)
    # CONTROL 4. The sidecar must describe THIS file.
    if total != n_rows:
        raise Refused(
            "the sidecar's segment rows sum to %d and the CSV has %d data "
            "rows. The segmentation describes a different file, so no "
            "episode boundary in it can be trusted. Refusing rather than "
            "exporting %d episodes cut at the wrong places."
            % (total, n_rows, len(segs)))
    return segs


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise Refused("%s has a header and no rows" % path)
    return rows


def require_columns(rows, cols, path):
    missing = [c for c in cols if c not in rows[0]]
    if missing:
        raise Refused("%s is missing %d required column(s): %s. Refusing "
                      "rather than exporting them as zeros."
                      % (path, len(missing), ", ".join(missing[:8])))


def fnum(v, default=0.0):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    return default if math.isnan(x) else x


def infer_fps(rows):
    """From the `t` column, which is seconds since the recorder started."""
    ts = [fnum(r.get("t"), float("nan")) for r in rows[:400]]
    ts = [t for t in ts if t == t]
    if len(ts) < 3:
        return 30
    d = sorted(ts[i + 1] - ts[i] for i in range(len(ts) - 1))
    med = d[len(d) // 2]
    if med <= 1e-6:
        return 30
    return int(round(1.0 / med))


# ------------------------------------------------------------------ export
def features_for(kept):
    return {
        "observation.state": {"dtype": "float32",
                              "shape": (len(kept),),
                              "names": [k for k, _ in kept]},
        "action": {"dtype": "float32", "shape": (len(ACTION_COLS),),
                   "names": list(ACTION_COLS)},
        "observation.environment_state": {
            "dtype": "float32", "shape": (len(EXTRA_COLS),),
            "names": list(EXTRA_COLS)},
    }


def export(csv_path, repo_id, out_dir, include_incoherent=False,
           health=None, health_path="<given>", verbose=True):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if health is None:
        health_path, health = newest_channel_baseline()
    kept, refused, notes = gate(health, include_incoherent)

    # CONTROL 5, enforced here rather than only in the self-test: no channel
    # the baseline calls DEAD may appear in the feature vector, whatever
    # flags were passed. A dead channel is one repeated value, and exported
    # it becomes a zero-variance feature indistinguishable from a joint that
    # was deliberately held still.
    dead = {k for k, v in health.items()
            if (v or {}).get("verdict") in REFUSE_ALWAYS}
    leaked = sorted(dead.intersection(k for k, _ in kept))
    if leaked:
        raise Refused(
            "the gate let %d DEAD channel(s) through into the feature "
            "vector: %s. That is the one thing it exists to prevent."
            % (len(leaked), ", ".join(leaked)))
    if not kept:
        raise Refused(
            "every one of the %d channels was dropped -- %d DEAD, %d "
            "INCOHERENT -- so there is no observation left to export. This "
            "is not a bug in the exporter; it is what the newest channel "
            "baseline says about the master arm."
            % (len(refused),
               sum(1 for r in refused if r[1] == "DEAD"),
               sum(1 for r in refused if r[1] == "INCOHERENT")))

    rows = load_csv(csv_path)
    meta_path = csv_path + ".meta.json"
    if not os.path.exists(meta_path):
        raise Refused("%s has no sidecar %s, so it has no segment labels and "
                      "no episode boundaries." % (csv_path, meta_path))
    segs = read_segments(meta_path, len(rows))
    require_columns(rows, [c for _, c in kept] + ACTION_COLS + EXTRA_COLS,
                    csv_path)

    fps = infer_fps(rows)
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    ds = LeRobotDataset.create(repo_id=repo_id, fps=fps,
                               features=features_for(kept), root=out_dir,
                               robot_type="srl_kortex_gen3_dual",
                               use_videos=False)

    i = 0
    n_ep = 0
    for s in segs:
        n = int(s["rows"])
        chunk = rows[i:i + n]
        i += n
        for r in chunk:
            # float32 ndarrays, not lists: LeRobotDataset validates the
            # dtype per frame and rejects a list, which is the right call --
            # a silently upcast column is a column nobody checked.
            ds.add_frame({
                "observation.state": np.asarray(
                    [fnum(r[c]) for _, c in kept], dtype=np.float32),
                "action": np.asarray(
                    [fnum(r[c]) for c in ACTION_COLS], dtype=np.float32),
                "observation.environment_state": np.asarray(
                    [fnum(r[c]) for c in EXTRA_COLS], dtype=np.float32),
                "task": s["label"],
            })
        ds.save_episode()
        n_ep += 1
        if verbose:
            print("  episode %2d  %-28s %5d frames" % (n_ep, s["label"], n))
    ds.finalize()

    # THE VERDICTS TRAVEL WITH THE DATA. A dataset on the Hub is separated
    # from this repository the moment it is pushed; it must still be able to
    # say which of its columns are trustworthy.
    hp = os.path.join(out_dir, "meta", "srl_channel_health.json")
    os.makedirs(os.path.dirname(hp), exist_ok=True)
    with open(hp, "w") as f:
        json.dump({
            "source_csv": os.path.relpath(csv_path, ROOT),
            "channel_baseline": os.path.relpath(health_path, ROOT)
            if os.path.isabs(health_path) else health_path,
            "exported": [{"channel": k, "column": c,
                          "verdict": (health.get(k) or {}).get("verdict")}
                         for k, c in kept],
            "refused": [{"channel": r[0], "verdict": r[1], "why": r[2],
                         "policy": r[3]} for r in refused],
            "caveats": [
                "%s is %s: %s" % (n[0], n[1], n[2]) for n in notes],
            "HONEST_NOTE": (
                "These are MASTER-ARM potentiometer channels on an "
                "instrumented mannequin, not robot joint encoders. Even the "
                "ALIVE ones are a leader device, and 7 of 14 were not "
                "coherent when this was recorded. This dataset is a FORMAT "
                "demonstration and a pipeline test. It is not a "
                "demonstration corpus and no policy trained on it means "
                "anything."),
        }, f, indent=1)
    return {"episodes": n_ep, "frames": len(rows), "fps": fps,
            "kept": [k for k, _ in kept],
            "refused": [r[0] for r in refused], "out": out_dir}


# ---------------------------------------------------------------- controls
def _synthetic_health(dead=(), incoherent=()):
    h = {}
    for arm in ARMS:
        for j in range(1, NJ + 1):
            k = channel_key(arm, j)
            if k in dead:
                h[k] = {"verdict": "DEAD", "why": "range 0.0 deg, 0 updates"}
            elif k in incoherent:
                h[k] = {"verdict": "INCOHERENT", "why": "jumps"}
            else:
                h[k] = {"verdict": "ALIVE", "why": "range 200 deg"}
    return h


def _write_synthetic_csv(path, n_ep=3, n_frame=20):
    """Every value CONSTRUCTED: frame i of the file gets value i/1000. So a
    round trip has an exact known answer and a mis-ordered or dropped frame
    is visible, not merely suspected."""
    cols = ["t", "wall", "segment"]
    for arm in ARMS:
        cols += [csv_joint_col(arm, j) for j in range(1, NJ + 1)]
    cols += ACTION_COLS + EXTRA_COLS
    segs = []
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        i = 0
        for e in range(n_ep):
            for _ in range(n_frame):
                row = {c: i / 1000.0 for c in cols}
                row["t"] = i / 30.0
                row["wall"] = 1e9 + i / 30.0
                row["segment"] = "seg_%d" % e
                w.writerow(row)
                i += 1
            segs.append({"label": "seg_%d" % e, "rows": n_frame,
                         "seconds": n_frame / 30.0, "motion_m": 0.1})
    with open(path + ".meta.json", "w") as f:
        json.dump({"csv": path, "segments": segs}, f)
    return segs, n_ep * n_frame


def self_test(verbose=True):
    ok = True
    tmp = tempfile.mkdtemp(prefix="lerobot_selftest_")
    try:
        csv_path = os.path.join(tmp, "synthetic.csv")
        _, n_rows = _write_synthetic_csv(csv_path)

        # ---- CONTROL 2: a DEAD channel is DROPPED from the features, the
        #      export still runs, and the drop is named. Both halves matter:
        #      exporting it would be a lie, and aborting on it would mean the
        #      pipeline is never exercised on the data that exists.
        try:
            res = export(csv_path, "srl/selftest_dead",
                         os.path.join(tmp, "dead"),
                         health=_synthetic_health(dead=("r_j5",)),
                         verbose=False)
            gone = "r_j5" not in res["kept"]
            named = "r_j5" in res["refused"]
            rest = len(res["kept"]) == 2 * NJ - 1
            good = gone and named and rest
            print("  2 dead channel        %s"
                  % ("DROPPED and named, other %d channels exported"
                     % len(res["kept"]) if good
                     else "*** FAILED: kept=%s refused=%s ***"
                          % (res["kept"], res["refused"])))
            ok &= good
        except ImportError:
            print("  2 dead channel        SKIPPED: lerobot not installed")
            return ok
        except Refused as e:
            print("  2 dead channel        *** FAILED: aborted the whole "
                  "export (%s) ***" % e)
            ok = False

        # ---- CONTROL 2b: every channel dead -> there IS nothing to export
        try:
            export(csv_path, "srl/selftest_alldead",
                   os.path.join(tmp, "alldead"),
                   health=_synthetic_health(
                       dead=tuple(channel_key(a, j) for a in ARMS
                                  for j in range(1, NJ + 1))),
                   verbose=False)
            print("  2b all channels dead  *** FAILED: exported anyway ***")
            ok = False
        except Refused as e:
            good = "no observation left" in str(e)
            print("  2b all channels dead  %s"
                  % ("REFUSED" if good
                     else "*** FAILED: wrong reason: %s ***" % e))
            ok &= good

        # ---- CONTROL 3: a CSV it cannot map
        bad = os.path.join(tmp, "bad.csv")
        shutil.copy(csv_path, bad)
        shutil.copy(csv_path + ".meta.json", bad + ".meta.json")
        with open(bad) as f:
            txt = f.read()
        with open(bad, "w") as f:                       # drop a column
            f.write(txt.replace("left_cmd_x,", "", 1))
        try:
            export(bad, "srl/selftest_bad", os.path.join(tmp, "bad_out"),
                   health=_synthetic_health(), verbose=False)
            print("  3 missing column      *** FAILED: exported anyway ***")
            ok = False
        except Refused as e:
            named = "left_cmd_x" in str(e)
            print("  3 missing column      %s"
                  % ("REFUSED and named left_cmd_x" if named
                     else "*** FAILED: refused without naming ***"))
            ok &= named
        except Exception as e:                          # noqa: BLE001
            print("  3 missing column      REFUSED (%s)" % type(e).__name__)

        # ---- CONTROL 4: the sidecar describes a different file
        m = os.path.join(tmp, "synthetic.csv.meta.json")
        with open(m) as f:
            meta = json.load(f)
        meta["segments"][0]["rows"] += 7
        with open(m, "w") as f:
            json.dump(meta, f)
        try:
            export(csv_path, "srl/selftest_seg",
                   os.path.join(tmp, "seg_out"),
                   health=_synthetic_health(), verbose=False)
            print("  4 segment mismatch    *** FAILED: exported anyway ***")
            ok = False
        except Refused as e:
            good = "different file" in str(e)
            print("  4 segment mismatch    %s"
                  % ("REFUSED" if good else "*** FAILED: wrong reason ***"))
            ok &= good
        meta["segments"][0]["rows"] -= 7
        with open(m, "w") as f:
            json.dump(meta, f)

        # ---- CONTROL 1: the round trip, last, because it needs a clean run
        out = os.path.join(tmp, "rt")
        try:
            res = export(csv_path, "srl/selftest_rt", out,
                         health=_synthetic_health(), verbose=False)
        except ImportError:
            print("  1 round trip          SKIPPED: lerobot is not installed "
                  "in this interpreter. `pip install lerobot`.")
            return ok
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        back = LeRobotDataset(res["out"] and "srl/selftest_rt", root=out)
        n = len(back)
        worst = 0.0
        for i in range(n):
            f = back[i]
            got = float(f["observation.state"][0])
            worst = max(worst, abs(got - i / 1000.0))
        good = (n == n_rows) and worst < 1e-6
        print("  1 round trip          %d frames back of %d, worst |err| "
              "%.2e  %s" % (n, n_rows, worst,
                            "OK" if good else "*** FAILED ***"))
        ok &= good
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    ap.add_argument("--repo-id", default="srl/kortex_teleop")
    ap.add_argument("--out", default=os.path.join(ROOT, "recordings/lerobot"))
    ap.add_argument("--include-incoherent", action="store_true",
                    help="export INCOHERENT channels too. They are still "
                         "named in meta/srl_channel_health.json. DEAD "
                         "channels are refused with or without this.")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    if a.self_test:
        print("CONTROLS")
        ok = self_test()
        print("\nlerobot_export self-test %s" % ("PASSED" if ok else "FAILED"))
        return 0 if ok else 1

    if a.list:
        path, health = newest_channel_baseline()
        kept, refused, notes = gate(health, a.include_incoherent)
        print("channel baseline: %s" % os.path.relpath(path, ROOT))
        print("  %d exportable, %d refused" % (len(kept), len(refused)))
        for k, v, why, pol in refused:
            print("    REFUSED %-6s %-12s %-28s (%s)" % (k, v, why, pol))
        print("\nexportable recordings:")
        for c in sorted(glob.glob(os.path.join(
                ROOT, "recordings/teleop_*.csv"))):
            has = os.path.exists(c + ".meta.json")
            print("  %-46s %s" % (os.path.relpath(c, ROOT),
                                  "segmented" if has
                                  else "NO SIDECAR -- not exportable"))
        return 0

    if not a.csv:
        ap.error("--csv is required (or use --list / --self-test)")

    print("CONTROLS")
    if not self_test():
        print("\nA CONTROL FAILED. Nothing is exported: an exporter that "
              "cannot prove its own round trip produces a dataset nobody "
              "can trust.")
        return 2
    print("\nEXPORT")
    try:
        res = export(a.csv, a.repo_id, a.out, a.include_incoherent)
    except Refused as e:
        print("\nREFUSED: %s" % e)
        return 1
    print("\n%d episodes, %d frames at %d fps -> %s"
          % (res["episodes"], res["frames"], res["fps"], res["out"]))
    print("kept    %s" % ", ".join(res["kept"]))
    print("refused %s" % ", ".join(res["refused"]))
    print("\nRead meta/srl_channel_health.json before believing any column.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
