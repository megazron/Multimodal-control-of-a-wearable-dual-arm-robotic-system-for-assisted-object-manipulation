#!/usr/bin/env python3
"""The pick-and-place outcome, from the record: one LaTeX table.

Two independent signals per session, both from the recording and neither
from memory:

  * the operator's command -- the VR controller's index trigger in the 20 Hz
    trail (vrc_<hand>_trig), passed through the SAME latch the live gripper
    node applies (close above 0.30, release below 0.10), so a count here is
    a count of commands the hand would have obeyed;
  * the hand itself -- /real/gripper_<hand> from the bag, the Kinova's own
    measured finger position, normalised 0..1 (figures/extract_pilot_gripper.py).

For every VR session: closures commanded, closures measured, and the share
of the session the real gripper spent closed. Written to
figures/pilot/gripper_table.tex and read by appendix/l_pilot_study.tex.

    python3 extras/figures/make_gripper_table.py
"""

import csv
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "pilot", "data")
OUT = os.path.join(HERE, "pilot", "gripper_table.tex")
import sys  # noqa: E402
sys.path.insert(0, HERE)
from session_paths import session_dir  # noqa: E402

LATCH_CLOSE, LATCH_OPEN = 0.30, 0.10  # vr_gripper_node defaults


def commanded_closures(key, hand):
    rows = csv.DictReader(open(os.path.join(session_dir(key), "trail.csv")))
    closed, n, held, total = False, 0, 0, 0
    for r in rows:
        v = r.get("vrc_%s_trig" % hand) or "0"
        x = float(v)
        if not closed and x > LATCH_CLOSE:
            closed, n = True, n + 1
        elif closed and x < LATCH_OPEN:
            closed = False
        held += closed
        total += 1
    return n, held / max(total, 1)


def main():
    meta = json.load(open(os.path.join(DATA, "vr_study_sessions.json")))
    grip = {}
    for l in open(os.path.join(DATA, "gripper_by_session.jsonl")):
        r = json.loads(l)
        grip[r["session"]] = r
    lines = []
    summary = {}
    for m in sorted(meta, key=lambda m: (m["participant"], m["session"])):
        h = m["active_hand"]
        cmd_n, cmd_frac = commanded_closures(m["session"], h)
        g = grip.get(m["session"], {})
        meas = g.get("hands", {}).get(h, {}).get("measured", {}) if "hands" in g else {}
        if meas.get("n"):
            meas_txt = "%d & \\SI{%.1f}{\\percent} & %.2f" % (meas["closures"], 100 * meas["closed_frac"], meas["max"])
            s = summary.setdefault(m["participant"], {"closures": 0, "held_s": 0.0, "max": 0.0})
            s["closures"] += meas["closures"]
            s["held_s"] += meas["closed_frac"] * m["duration_s"]
            s["max"] = max(s["max"], meas["max"])
        else:
            meas_txt = "\\multicolumn{3}{c}{not in bag}"
        task = m["task"] if m["task"] != "Unspecified" else "(unlabelled)"
        lines.append("    %s & %s & %s & %d & \\SI{%.1f}{\\percent} & %s \\\\" % (
            m["participant"], task, m["condition"], cmd_n, 100 * cmd_frac, meas_txt))
    body = "\n".join(lines)
    tex = r"""\begin{table}[htbp]
  \centering
  \scriptsize
  \caption[Gripper use in every VR session]{Gripper use in every VR
  session, from two independent records. \emph{Commanded}: closures of the
  controller's index trigger in the \SI{20}{\hertz} trail, counted through
  the live gripper node's own latch (close above 0.30, release below 0.10),
  and the share of the session the command was held. \emph{Measured}: the
  physical gripper's own finger position from the bag
  (\repo{/real/gripper\_<hand>}, 0 open, 1 closed): closures, share of the
  session closed, and the furthest the fingers closed. The active hand only.}
  \label{tab:gripper-use}
  \begin{tabular}{@{}lllrrrrr@{}}
    \toprule
    & & & \multicolumn{2}{c}{Commanded} & \multicolumn{3}{c}{Measured} \\
    \cmidrule(lr){4-5} \cmidrule(lr){6-8}
    P & Task & Cond. & Closures & Held & Closures & Closed & Max \\
    \midrule
%s
    \bottomrule
  \end{tabular}
\end{table}
""" % body
    open(OUT, "w").write(tex)
    print("wrote", OUT)
    for p, s in sorted(summary.items()):
        print("%s: measured closures %d, held closed %.1f s in total, max %.2f" % (p, s["closures"], s["held_s"], s["max"]))


if __name__ == "__main__":
    main()
