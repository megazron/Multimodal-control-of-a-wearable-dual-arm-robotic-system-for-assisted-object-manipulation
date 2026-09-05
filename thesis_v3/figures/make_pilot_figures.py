"""Pilot-study figure panels, in the same house style as
figures/make_figures.py (muted grey/blue/red, font.size 8-9, no baked-in
display titles -- captions are added by the LaTeX \\subcaption/\\caption
around each panel, exactly as figures/master_arm_cad_views.png etc. are
composed in chapters/appendix_b_master.tex). Each panel is a small,
independent PDF sized to sit at roughly a third to a half of a text
column; main.tex assembles them into two collage figures via
\\subfigure blocks.

Run from the repository root:

    python3 thesis_v3/figures/make_pilot_figures.py
"""
import csv, json, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRATCH = "/tmp/claude-1000/-home-gausms-kortex-ws/bbebc7ab-9df1-4fc1-80b7-ee284d9eaac6/scratchpad"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot")
os.makedirs(OUT, exist_ok=True)
SESS_ROOT = "recordings/sessions"

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7,
    "figure.dpi": 150, "savefig.bbox": "tight", "axes.spines.top": False,
    "axes.spines.right": False,
})
GREY, BLUE, RED = "0.35", "#2c6fbb", "#c1392b"
ANON = {"Wen": "P1", "Hela": "P2", "Farrel": "P3", "Feifan": "P4"}

DATA = json.load(open(os.path.join(SCRATCH, "study_summary.json")))
SESSIONS = DATA["sessions"]
PAIRED = DATA["paired"]
for s in SESSIONS:
    s["participant"] = ANON.get(s["participant"], s["participant"])
for p in PAIRED:
    p["participant"] = ANON.get(p["participant"], p["participant"])

TASKS = ["Object tracking", "Target reaching", "Position matching", "Unspecified"]
TASK_SHORT = {"Object tracking": "track", "Target reaching": "reach",
              "Position matching": "match", "Unspecified": "P5*"}


def by_task_condition(metric):
    out = {t: {"Direct": [], "Shared": []} for t in TASKS}
    for s in SESSIONS:
        v = s.get(metric)
        if v is not None:
            out[s["task"]][s["condition"]].append(v)
    return out


def grouped_panel(metric, ylabel, fname, scale=1.0, figsize=(2.5, 2.1), legend=False):
    grouped = by_task_condition(metric)
    fig, ax = plt.subplots(figsize=figsize)
    x = np.arange(len(TASKS))
    w = 0.36
    rng = np.random.default_rng(3)
    for i, (cond, col) in enumerate([("Direct", BLUE), ("Shared", RED)]):
        means = [np.mean([v * scale for v in grouped[t][cond]]) for t in TASKS]
        xc = x + (i - 0.5) * w
        ax.bar(xc, means, w, color=col, alpha=0.85, label=cond)
        for j, t in enumerate(TASKS):
            vals = [v * scale for v in grouped[t][cond]]
            jitter = rng.uniform(-w * 0.16, w * 0.16, size=len(vals))
            ax.scatter(np.full(len(vals), xc[j]) + jitter, vals, s=6,
                       color="black", alpha=0.5, zorder=3, linewidths=0)
    ax.set_xticks(x)
    ax.set_xticklabels([TASK_SHORT[t] for t in TASKS])
    ax.set_ylabel(ylabel)
    if legend:
        ax.legend(frameon=False, fontsize=6.5, loc="upper right",
                  handlelength=1.2, borderaxespad=0.1)
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


grouped_panel("duration_s", "time to finish (s)", "pilot_time.pdf", legend=True)
grouped_panel("ee_path_m", "hand travel (m)", "pilot_distance.pdf")
grouped_panel("clutch_engagements", "re-grips", "pilot_regrips.pdf")
grouped_panel("active_control_frac", "active share (%)", "pilot_active.pdf", scale=100.0)
grouped_panel("mean_lag_mm", "lag (mm)", "pilot_lag.pdf")


# ---------------------------------------------------------------------------
# Paired panels (same operator, same task, both conditions)
# ---------------------------------------------------------------------------
def declutter(vals, min_gap):
    idx = sorted(range(len(vals)), key=lambda i: vals[i])
    y = [vals[i] for i in idx]
    for k in range(1, len(y)):
        if y[k] - y[k - 1] < min_gap:
            y[k] = y[k - 1] + min_gap
    for k in range(len(y) - 2, -1, -1):
        if y[k + 1] - y[k] < min_gap:
            y[k] = y[k + 1] - min_gap
    out = [0.0] * len(vals)
    for pos, i in enumerate(idx):
        out[i] = y[pos]
    return out


def slope_panel(metric, ylabel, fname, fmt="{:.0f}", figsize=(3.0, 2.6)):
    fig, ax = plt.subplots(figsize=figsize)
    xs = [0, 1]
    directs = [p[metric]["direct"] for p in PAIRED]
    shareds = [p[metric]["shared"] for p in PAIRED]
    y_range = max(directs + shareds) * 1.12
    min_gap = y_range * 0.06
    ly = declutter(directs, min_gap)
    ry = declutter(shareds, min_gap)
    for i, p in enumerate(PAIRED):
        d, s = directs[i], shareds[i]
        ax.plot(xs, [d, s], color=GREY, linewidth=0.9, zorder=2)
        ax.scatter([0], [d], s=14, color=BLUE, zorder=3, linewidths=0)
        ax.scatter([1], [s], s=14, color=RED, zorder=3, linewidths=0)
        if abs(ly[i] - d) > 1e-6:
            ax.plot([-0.12, -0.03], [ly[i], d], color=GREY, linewidth=0.4, zorder=1)
        ax.annotate(f"{p['participant']} ({p['task'].split()[0].lower()})",
                    (0, ly[i]), textcoords="offset points", xytext=(-7, 0),
                    ha="right", va="center", fontsize=6, color=GREY)
        if abs(ry[i] - s) > 1e-6:
            ax.plot([1.03, 1.11], [s, ry[i]], color=GREY, linewidth=0.4, zorder=1)
        ax.annotate(fmt.format(s), (1, ry[i]), textcoords="offset points",
                    xytext=(7, 0), ha="left", va="center", fontsize=6.5)
    ax.set_xlim(-0.95, 1.65)
    ax.set_xticks(xs)
    ax.set_xticklabels(["direct", "shared"])
    ax.set_ylabel(ylabel)
    ax.set_ylim(bottom=0)
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


slope_panel("duration_s", "time to finish (s)", "pilot_paired_time.pdf")
slope_panel("clutch_engagements", "re-grips", "pilot_paired_regrips.pdf")


# ---------------------------------------------------------------------------
# Trajectory and engagement panels, one participant, both conditions
# ---------------------------------------------------------------------------
def load_trace(session_dir, hand):
    path = os.path.join(SESS_ROOT, session_dir, "trail.csv")
    x, y, z = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                xv, yv, zv = row[f"ee_{hand}_x"], row[f"ee_{hand}_y"], row[f"ee_{hand}_z"]
                if xv == "" or yv == "" or zv == "":
                    continue
                x.append(float(xv)); y.append(float(yv)); z.append(float(zv))
            except (KeyError, ValueError):
                continue
    return np.array(x), np.array(y), np.array(z)


def load_events(session_dir):
    events = []
    with open(os.path.join(SESS_ROOT, session_dir, "events.jsonl")) as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


TRAJ = {
    "Direct": ("20260904_172909_Feifan_VRDIrect_ObjecttTracking", "left"),
    "Shared": ("20260904_174535_Feifan_VRShared_ObjectTracking", "left"),
}
COND_COLOR = {"Direct": BLUE, "Shared": RED}

for cond in ["Direct", "Shared"]:
    sess, hand = TRAJ[cond]
    x, y, z = load_trace(sess, hand)
    fig, ax = plt.subplots(figsize=(2.6, 2.3))
    ax.plot(x, z, color=COND_COLOR[cond], linewidth=0.6)
    ax.scatter([x[0]], [z[0]], s=10, color=GREY, zorder=4)
    ax.scatter([x[-1]], [z[-1]], s=10, color="black", marker="s", zorder=4)
    ax.set_xlabel("sideways (m)")
    ax.set_ylabel("height (m)")
    ax.set_aspect("equal", adjustable="datalim")
    fig.savefig(os.path.join(OUT, f"pilot_traj_{cond.lower()}.pdf"))
    plt.close(fig)

fig, ax = plt.subplots(figsize=(6.2, 1.6))
durations = [s["duration_s"] for s in SESSIONS if s["session"] in (TRAJ["Direct"][0], TRAJ["Shared"][0])]
xmax = max(durations) * 1.02
for row, cond in enumerate(["Direct", "Shared"]):
    sess, hand = TRAJ[cond]
    events = load_events(sess)
    spans, open_t = [], None
    for e in events:
        if e.get("kind") == "clutch" and e.get("hand") == hand:
            if e.get("engaged"):
                open_t = e["t"]
            elif open_t is not None:
                spans.append((open_t, e["t"] - open_t))
                open_t = None
    ax.broken_barh(spans, (row * 1.2, 0.9), facecolors=COND_COLOR[cond])
ax.set_yticks([0.45, 1.65])
ax.set_yticklabels(["direct", "shared"], fontsize=7)
ax.set_xlim(0, xmax)
ax.set_xlabel("time into the session (s)")
fig.savefig(os.path.join(OUT, "pilot_engagement.pdf"))
plt.close(fig)

print("wrote", sorted(os.listdir(OUT)))
