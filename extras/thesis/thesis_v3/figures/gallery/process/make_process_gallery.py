#!/usr/bin/env python3
"""Plain-language figures about HOW the project was done, from its own
records: the 26 measurement defects (extras/thesis/thesis_v3/appendix/g_instrument.tex),
the git history, and the recording yield in recordings/sessions/.

    python3 extras/thesis/thesis_v3/figures/gallery/process/make_process_gallery.py
"""

import glob
import os
import re
import subprocess
import collections

import yaml
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.dates as mdates  # noqa: E402
from datetime import date  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 6))
GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})


def save(fig, name):
    fig.savefig(os.path.join(HERE, name + ".pdf"))
    fig.savefig(os.path.join(HERE, name + ".png"), dpi=150)
    plt.close(fig)
    print("wrote", name)


# ------------------------------------------------------- 1. the 26 defects
# Counts as stated in extras/thesis/thesis_v3/appendix/g_instrument.tex.
MECHANISM = [
    ("a missing value read as a real one", 7),
    ("the test set up the very thing it tested", 6),
    ("leftover state from an earlier run", 5),
    ("steps done in the wrong order", 5),
    ("a constant written in several places,\nmeasured in none", 3),
]
CAUGHT = [
    ("a known quantity disagreed\nwith the reading", 15),
    ("reading the code", 5),
    ("looking at the raw image or data", 6),
]


def defects():
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.2, 2.7), gridspec_kw={"width_ratios": [1.3, 1]})
    for ax, data, colour, title in [(ax0, MECHANISM, GREY, "what went wrong (26 cases)"),
                                    (ax1, CAUGHT, BLUE, "how it was caught")]:
        labels = [d[0] for d in data][::-1]
        vals = [d[1] for d in data][::-1]
        ax.barh(labels, vals, color=colour)
        for i, v in enumerate(vals):
            ax.text(v + 0.2, i, str(v), va="center", fontsize=8)
        ax.set_title(title)
        ax.set_xlim(0, max(vals) * 1.18)
        ax.set_xticks([])
        ax.spines["bottom"].set_visible(False)
        ax.tick_params(axis="y", labelsize=7.5, length=0)
    fig.subplots_adjust(left=0.29, right=0.98, top=0.88, bottom=0.05, wspace=0.9)
    save(fig, "instrument_defects")


# ------------------------------------------------------------ 2. git history
def git_days(args):
    out = subprocess.run(["git", "-C", ROOT, "log", "--date=short"] + args,
                         capture_output=True, text=True).stdout
    days = [l for l in out.splitlines() if re.match(r"^\d{4}-\d{2}-\d{2}$", l.strip())]
    return collections.Counter(d.strip() for d in days)


def history():
    commits = git_days(["--format=%ad"])
    tests = git_days(["--diff-filter=A", "--format=%ad", "--name-only", "--", "src/*/test/test_*.py"])
    # --name-only prints the date once per commit, then filenames; count files per date instead
    out = subprocess.run(["git", "-C", ROOT, "log", "--diff-filter=A", "--date=short",
                          "--format=DATE %ad", "--name-only", "--", "src/*/test/test_*.py"],
                         capture_output=True, text=True).stdout
    tests = collections.Counter()
    cur = None
    for line in out.splitlines():
        if line.startswith("DATE "):
            cur = line[5:].strip()
        elif line.strip().endswith(".py") and cur:
            tests[cur] += 1
    days = sorted(set(commits) | set(tests))
    d0, d1 = date.fromisoformat(days[0]), date.fromisoformat(days[-1])
    xs = [date.fromisoformat(d) for d in days]
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(7.2, 3.6), sharex=True)
    ax0.bar(xs, [commits.get(d, 0) for d in days], color=BLUE, width=0.8)
    ax0.set_ylabel("commits that day")
    cum, running = [], 0
    for d in days:
        running += tests.get(d, 0)
        cum.append(running)
    ax1.step(xs, cum, where="post", color=GREEN, lw=1.6)
    ax1.fill_between(xs, cum, step="post", color=GREEN, alpha=0.12)
    ax1.set_ylabel("test files, cumulative")
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%d %b"))
    ax1.xaxis.set_major_locator(mdates.DayLocator(interval=7))
    from datetime import timedelta
    ax1.set_xlim(d0 - timedelta(days=1), d1 + timedelta(days=1))
    n_tests = sum(tests.values())
    n_funcs = 0
    for f in glob.glob(os.path.join(ROOT, "src", "*", "test", "test_*.py")):
        n_funcs += sum(1 for l in open(f, errors="ignore") if l.lstrip().startswith("def test_"))
    ax1.text(0.01, 0.95, "%d test files, %d individual tests today" % (n_tests, n_funcs),
             transform=ax1.transAxes, va="top", fontsize=8, color=GREY)
    fig.subplots_adjust(left=0.1, right=0.98, top=0.97, bottom=0.1, hspace=0.12)
    save(fig, "commits_and_tests")
    return sum(commits.values()), n_tests, n_funcs


# -------------------------------------------------------- 3. recording yield
def recording_yield():
    base = os.path.join(ROOT, "recordings", "sessions")
    by = collections.defaultdict(lambda: [0, 0])
    for d in sorted(os.listdir(base)):
        p = os.path.join(base, d)
        if not os.path.isdir(p):
            continue
        m = re.search(r"(2026\d{4})", d)
        if not m:
            continue
        day = m.group(1)
        msgs = 0
        for meta in glob.glob(os.path.join(p, "**", "metadata.yaml"), recursive=True):
            try:
                msgs = max(msgs, yaml.safe_load(open(meta))["rosbag2_bagfile_information"]["message_count"] or 0)
            except Exception:
                pass
        rows = 0
        for t in ("trail.csv", "trail_extracted.csv"):
            if os.path.exists(os.path.join(p, t)):
                rows = max(rows, sum(1 for _ in open(os.path.join(p, t))) - 1)
        has = msgs > 100 or rows > 50
        by[day][0 if has else 1] += 1
    days = sorted(by)
    xs = [date(int(d[:4]), int(d[4:6]), int(d[6:])) for d in days]
    fig, ax = plt.subplots(figsize=(7.2, 2.7))
    good = [by[d][0] for d in days]
    empty = [by[d][1] for d in days]
    ax.bar(xs, good, color=GREEN, width=0.8, label="recording holds data")
    ax.bar(xs, empty, bottom=good, color="0.78", width=0.8, label="recording is empty")
    for x, g, e in zip(xs, good, empty):
        ax.text(x, g + e + 0.3, "%d/%d" % (g, g + e), ha="center", fontsize=7, color=GREY)
    ax.set_ylabel("recording sessions started")
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%d %b"))
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.14)
    save(fig, "recording_yield")
    return {d: tuple(by[d]) for d in days}


if __name__ == "__main__":
    defects()
    print("history:", history())
    print("yield:", recording_yield())
