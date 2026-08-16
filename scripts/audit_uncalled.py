#!/usr/bin/env python3
"""WHAT EXISTS AND NOTHING CALLS. The recurring fault, enumerated once.

    python3 scripts/audit_uncalled.py

This project keeps finding capability that was built, committed, and then
invoked by nothing: `set_measured()` with no producer, `log_continuously`
declared and never read, the presentation pose reported present twice and
absent both times, `mock_rgbd_camera` with no launch file. Each was found by
accident, one at a time, after it had already cost something.

So this asks the question in bulk instead. For every executable-looking Python
entry point in the repository it counts REFERENCES from anywhere else --
imports, subprocess argv, launch files, setup.py console_scripts, shell
scripts, docs -- and lists the ones with none.

WHAT A HIT MEANS, AND WHAT IT DOES NOT. An unreferenced script is not
necessarily dead: many are meant to be run by hand, and that is a legitimate
category. The point is to make the LIST visible so each entry is a decision
rather than a discovery. Anything genuinely operator-run should say so in its
docstring; anything that ought to be wired in is a finding.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = ("/archive/", "/build/", "/install/", "/log/", "/.git/",
             "/src/ros2_kortex/", "/src/ros2_robotiq_gripper/",
             "/src/serial/", "/__pycache__/")


def tracked_files():
    out = subprocess.run(["git", "ls-files"], cwd=ROOT,
                         capture_output=True, text=True).stdout.split()
    keep = []
    for f in out:
        p = "/" + f
        if any(s in p for s in SKIP_DIRS):
            continue
        keep.append(f)
    return keep


def main():
    files = tracked_files()
    py = [f for f in files if f.endswith(".py")]
    # candidates: things that look like an entry point
    cands = []
    for f in py:
        base = os.path.basename(f)[:-3]
        if base in ("__init__", "setup", "conftest"):
            continue
        if base.startswith("test_"):
            continue
        src = open(os.path.join(ROOT, f), errors="ignore").read()
        if "__main__" not in src and "def main(" not in src:
            continue
        cands.append((f, base))

    # one pass over every tracked text file, counting references
    blob = {}
    for f in files:
        try:
            blob[f] = open(os.path.join(ROOT, f), errors="ignore").read()
        except Exception:                                        # noqa: BLE001
            continue

    # TOKENISE ONCE. Regexing every candidate against every file is
    # O(candidates x files) and takes minutes; each file's identifier set is
    # built once and membership is then O(1).
    toks = {g: set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", text))
            for g, text in blob.items()}
    rows = []
    for f, base in cands:
        where = [g for g, t in toks.items() if g != f and base in t]
        rows.append((len(where), f, where[:3]))

    rows.sort()
    dead = [r for r in rows if r[0] == 0]
    print("ENTRY POINTS WITH NO REFERENCE ANYWHERE  (%d of %d)"
          % (len(dead), len(rows)))
    for n, f, _w in dead:
        print("   %s" % f)
    thin = [r for r in rows if r[0] == 1]
    print("\nREFERENCED EXACTLY ONCE  (%d) -- usually only from a doc"
          % len(thin))
    for n, f, w in thin:
        print("   %-58s <- %s" % (f, ", ".join(w)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
