#!/usr/bin/env python3
"""Strip participant identity from the derived pilot data files.

The figure scripts read small JSON tables in figures/pilot/data/. Those
tables were first built from the raw session folders, whose names carry the
operators' first names. Before the tables enter the repository every
participant field must be a code (P1..P5, M1..M3) and every session field
must be the recording's timestamp key (YYYYMMDD_HHMMSS), from which the
folder is found again by a glob at load time. The code book that maps names
to codes lives OUTSIDE the repository, in recordings/sessions/
participant_codes.json, and is read only here.

The script refuses to finish if any participant string is not a code after
mapping, so a forgotten name cannot slip through.

    python3 extras/figures/anonymise_pilot_data.py
"""

import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 2))
DATA = os.path.join(HERE, "pilot", "data")
BOOK = os.path.join(ROOT, "recordings", "sessions", "participant_codes.json")
KEY = re.compile(r"\d{8}_\d{6}")
CODE = re.compile(r"^[PM]\d$")


def session_key(name):
    m = KEY.search(name)
    if not m:
        raise SystemExit("no timestamp key in session name %r" % name)
    return m.group(0)


def fix(items, book):
    for s in items:
        if "participant" in s:
            s["participant"] = book.get(s["participant"], s["participant"])
            if not CODE.match(s["participant"]):
                raise SystemExit("participant %r is not a code; add it to the code book" % s["participant"])
        if "session" in s:
            s["session"] = session_key(s["session"])


def main():
    if not os.path.exists(BOOK):
        raise SystemExit("code book missing: %s" % BOOK)
    b = json.load(open(BOOK))
    vr, ma = b["vr"], b["master"]
    p = os.path.join(DATA, "vr_study_sessions.json")
    d = json.load(open(p)); fix(d, vr); json.dump(d, open(p, "w"), indent=1)
    p = os.path.join(DATA, "master_arm_sessions.json")
    d = json.load(open(p)); fix(d, ma); json.dump(d, open(p, "w"), indent=1)
    p = os.path.join(DATA, "study_summary.json")
    d = json.load(open(p))
    for k in ("sessions", "cells", "paired"):
        fix(d.get(k, []), vr)
    json.dump(d, open(p, "w"), indent=1)
    p = os.path.join(DATA, "gripper_by_session.jsonl")
    if os.path.exists(p):
        rows = [json.loads(l) for l in open(p) if l.strip()]
        for r in rows:
            r["session"] = session_key(r["session"])
            if "bag" in r:
                r["bag"] = re.sub(r"sessions/[^/]+/", "sessions/<%s>/" % r["session"], r["bag"])
            if "error" in r:
                r["error"] = KEY.sub("<key>", re.sub(r"sessions/[^/\"]+", "sessions/<session>", r["error"]))
        open(p, "w").write("".join(json.dumps(r) + "\n" for r in rows))
    # final sweep: no name from the code book may survive anywhere in the data dir
    names = [n for n in list(vr) + list(ma) if not CODE.match(n)]
    for f in glob.glob(os.path.join(DATA, "*.json*")):
        txt = open(f).read()
        hit = [n for n in names if re.search(r"(?i)\b%s\b" % re.escape(n), txt)]
        if hit:
            raise SystemExit("%s still contains %s" % (f, hit))
    print("anonymised:", ", ".join(sorted(os.path.basename(f) for f in glob.glob(os.path.join(DATA, "*.json*")))))


if __name__ == "__main__":
    main()
