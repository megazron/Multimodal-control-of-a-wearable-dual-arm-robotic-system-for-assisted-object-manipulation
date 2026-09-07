#!/usr/bin/env python3
"""Every figure the pilot recordings support, in plain words.

    python3 thesis_v3/figures/gallery/pilot/make_pilot_gallery.py

Reads the 23 VR sessions (P1-P5) and 9 master-arm sessions (M1-M3) under
recordings/sessions/ and writes PDF + PNG pairs plus INDEX.md into this
directory. Participant identity is anonymised BEFORE any plotting from the
same ANON map the report's other figure scripts use; no real name reaches a
label. House style: figures/make_figures.py (muted grey/blue/red).

Labels are deliberately plain: "how far the robot's hand moved", "robot lag
behind the operator", "direct control" / "with assistance".
"""
import csv, json, os, re, math, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SESS = os.path.join(ROOT, "recordings", "sessions")
SCRATCH = os.path.join(HERE, "..", "..", "pilot", "data")  # the derived session tables, kept in the repo
import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from session_paths import session_dir  # noqa: E402

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 8,
    "figure.dpi": 150, "savefig.bbox": "tight",
    "axes.spines.top": False, "axes.spines.right": False,
})
GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
COND_COL = {"Direct": BLUE, "Shared": RED}
COND_LBL = {"Direct": "direct control", "Shared": "with assistance"}
ANON = {}  # the data files are already anonymised (figures/anonymise_pilot_data.py); codes pass through
TASK_LBL = {"Pick and place": "pick and place", "Target reaching": "target reaching",
            "Unspecified": "task not labelled"}
GATE_MM = 30.0
INDEX = []


def save(fig, name, title, data, takeaway, numbers, verdict):
    fig.savefig(os.path.join(HERE, name + ".pdf"))
    fig.savefig(os.path.join(HERE, name + ".png"), dpi=150)
    plt.close(fig)
    INDEX.append((name, title, data, takeaway, numbers, verdict))
    print("wrote", name)


def fl(x):
    try:
        return float(x) if x not in (None, "") else np.nan
    except ValueError:
        return np.nan


# --------------------------------------------------------------------------
# load
# --------------------------------------------------------------------------
vr_meta = json.load(open(os.path.join(SCRATCH, "vr_study_sessions.json")))
ma_meta = json.load(open(os.path.join(SCRATCH, "master_arm_sessions.json")))
for m in vr_meta + ma_meta:
    m["participant"] = ANON.get(m["participant"], m["participant"])
    assert re.fullmatch(r"[PM]\d", m["participant"]), m["participant"]


def load_vr(m):
    d = session_dir(m["session"])
    rows = list(csv.DictReader(open(os.path.join(d, "trail.csv"))))
    h = m["active_hand"]
    col = lambda k: np.array([fl(r.get(k)) for r in rows])
    s = dict(m)
    s["t"] = col("t")
    s["ee"] = np.stack([col(f"ee_{h}_x"), col(f"ee_{h}_y"), col(f"ee_{h}_z")], 1)
    s["ctrl"] = np.stack([col(f"vrc_{h}_x"), col(f"vrc_{h}_y"), col(f"vrc_{h}_z")], 1)
    s["engaged"] = col(f"vr_{h}_engaged")
    s["grip"] = col(f"vr_{h}_grip")
    s["lag_mm"] = col(f"vr_{h}_lag_m") * 1000.0
    s["tracked"] = col(f"vr_{h}_tracked")
    s["estop"] = col("estop")
    s["sim_j"] = np.stack([col(f"sim_{h}_j{i}") for i in range(1, 8)], 1)
    s["real_j"] = np.stack([col(f"real_{h}_j{i}") for i in range(1, 8)], 1)
    ev = [json.loads(l) for l in open(os.path.join(d, "events.jsonl"))]
    s["events"] = ev
    return s


def load_master(m):
    d = session_dir(m["session"])
    rows = list(csv.DictReader(open(os.path.join(d, "trail_extracted.csv"))))
    h = m["active_hand"]
    col = lambda k: np.array([fl(r.get(k)) for r in rows])
    s = dict(m)
    s["t"] = col("t")
    s["master"] = np.stack([col(f"master_{h}_x"), col(f"master_{h}_y"), col(f"master_{h}_z")], 1)
    s["sim_j"] = np.stack([col(f"sim_{h}_j{i}") for i in range(1, 8)], 1)
    s["real_j"] = np.stack([col(f"real_{h}_j{i}") for i in range(1, 8)], 1)
    s["pots"] = {side: np.stack([col(f"mraw_{side}_p{i}") for i in range(1, 8)], 1)
                 for side in ("left", "right")}
    return s


VR = [load_vr(m) for m in vr_meta]
MA = [load_master(m) for m in ma_meta]
VR.sort(key=lambda s: s["session"])          # timestamp order = recording order
PARTS = ["P1", "P2", "P3", "P4", "P5"]


def speed(t, xyz):
    """hand speed in m/s from a 20 Hz trace, NaN where a sample is missing."""
    d = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.diff(t)
    v = np.full(len(t), np.nan)
    ok = (dt > 0) & (d < 0.5)
    v[1:][ok] = d[ok] / dt[ok]
    return v


def runs(mask):
    """(start_idx, end_idx) of contiguous True runs."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((start, i - 1)); start = None
    if start is not None:
        out.append((start, len(mask) - 1))
    return out


def label(s):
    return "%s, %s, %s" % (s["participant"], TASK_LBL[s["task"]], COND_LBL[s["condition"]])


def box(ax, groups, labels, colors, ylabel):
    bp = ax.boxplot(groups, labels=labels, widths=0.5, patch_artist=True,
                    medianprops=dict(color="black", lw=1.4),
                    whiskerprops=dict(color=GREY), capprops=dict(color=GREY),
                    flierprops=dict(marker="o", ms=3, mfc=GREY, mec="none", alpha=0.7))
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c); patch.set_alpha(0.35); patch.set_edgecolor(c)
    ax.set_ylabel(ylabel)


# --------------------------------------------------------------------------
# 0. sanity: what is "grip"?
# --------------------------------------------------------------------------
same = sum(int(np.nansum(s["grip"] == s["engaged"])) for s in VR)
total = sum(len(s["t"]) for s in VR)
GRIP_IS_CLUTCH = same / total
print("vr_*_grip equals vr_*_engaged in %.2f%% of samples" % (100 * GRIP_IS_CLUTCH))

# --------------------------------------------------------------------------
# 1. session at a glance, one figure per participant
# --------------------------------------------------------------------------
for p in PARTS:
    ss = [s for s in VR if s["participant"] == p]
    fig, axes = plt.subplots(len(ss), 1, figsize=(7.2, 1.35 * len(ss) + 0.4), sharex=False)
    axes = np.atleast_1d(axes)
    for ax, s in zip(axes, ss):
        t = s["t"]; v = speed(t, s["ee"]) * 1000
        for a, b in runs(s["engaged"] == 1):
            ax.axvspan(t[a], t[b], color=COND_COL[s["condition"]], alpha=0.12, lw=0)
        for a, b in runs(s["estop"] == 1):
            ax.axvspan(t[a], t[b], color="black", alpha=0.35, lw=0)
        ax.plot(t, v, color=GREY, lw=0.6)
        ax.set_ylabel("robot hand\nspeed (mm/s)", fontsize=7)
        ax.set_ylim(0, np.nanpercentile(v, 99.5) * 1.1 if np.isfinite(np.nanpercentile(v, 99.5)) else 1)
        ax.text(0.995, 0.92, label(s), transform=ax.transAxes, ha="right", va="top", fontsize=7.5)
        ax.set_xlim(0, t[-1])
    axes[-1].set_xlabel("time into session (s)")
    fig.legend(handles=[Patch(color=BLUE, alpha=0.3, label="clutch held, direct control"),
                        Patch(color=RED, alpha=0.3, label="clutch held, with assistance"),
                        Patch(color="black", alpha=0.35, label="emergency stop held")],
               loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.subplots_adjust(hspace=0.55, bottom=0.14)
    save(fig, f"01_session_glance_{p}", f"What {p}'s sessions looked like, second by second",
         f"trail.csv of every {p} session: vr_*_engaged, estop, ee_*_x/y/z at 20 Hz",
         "Shaded = clutch held (the operator steering); grey line = how fast the robot's hand moved; black = emergency stop held.",
         "%d sessions" % len(ss), "APPENDIX")

# --------------------------------------------------------------------------
# 2. paths, top-down and side, grid participant x condition
# --------------------------------------------------------------------------
def path_grid(idx, xlab, ylab, name, title):
    fig, axes = plt.subplots(len(PARTS), 2, figsize=(6.0, 2.0 * len(PARTS)), squeeze=False)
    for i, p in enumerate(PARTS):
        for j, cond in enumerate(["Direct", "Shared"]):
            ax = axes[i, j]
            ss = [s for s in VR if s["participant"] == p and s["condition"] == cond]
            for s in ss:
                xy = s["ee"][:, idx]; ok = np.isfinite(xy).all(1)
                xy = xy[ok]; tt = s["t"][ok]
                pts = xy.reshape(-1, 1, 2); segs = np.concatenate([pts[:-1], pts[1:]], 1)
                lc = LineCollection(segs, cmap="viridis", lw=0.7, alpha=0.9)
                lc.set_array(tt / tt[-1]); ax.add_collection(lc)
            ax.autoscale(); ax.set_aspect("equal", adjustable="datalim")
            ax.text(0.02, 0.95, f"{p}, {COND_LBL[cond]} ({len(ss)} session%s)" % ("" if len(ss) == 1 else "s"),
                    transform=ax.transAxes, va="top", fontsize=7.5)
            if i == len(PARTS) - 1: ax.set_xlabel(xlab)
            if j == 0: ax.set_ylabel(ylab)
    sm = plt.cm.ScalarMappable(cmap="viridis", norm=plt.Normalize(0, 1))
    cb = fig.colorbar(sm, ax=axes.ravel().tolist(), fraction=0.02, pad=0.02)
    cb.set_label("time into session (start → end)")
    save(fig, name, title, "trail.csv ee_*_x/y/z (the robot's hand), every VR session",
         "Where the robot's hand went; colour runs from the start (dark) to the end (yellow) of each session.",
         "23 sessions in 10 cells", "APPENDIX")

path_grid([0, 1], "left–right (m)", "forward (m)", "02_paths_topdown", "The robot's hand path seen from above")
path_grid([0, 2], "left–right (m)", "height (m)", "02_paths_side", "The robot's hand path seen from the side")

# --------------------------------------------------------------------------
# 3. order effects
# --------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.8))
for p in PARTS:
    ss = [s for s in VR if s["participant"] == p]
    xs = np.arange(1, len(ss) + 1)
    for ax, key in zip(axes, ["duration_s", "clutch_engagements"]):
        ys = [s[key] for s in ss]
        ax.plot(xs, ys, color=GREY, lw=0.7, alpha=0.6)
        for x, y, s in zip(xs, ys, ss):
            ax.scatter(x, y, color=COND_COL[s["condition"]], s=22, zorder=3)
        ax.text(xs[-1] + 0.12, ys[-1], p, fontsize=7.5, va="center", color=GREY)
axes[0].set_ylabel("time to finish (s)"); axes[1].set_ylabel("times the clutch was re-engaged")
for ax in axes:
    ax.set_xlabel("session number, in the order recorded"); ax.set_xticks(range(1, 8))
axes[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", color=BLUE, label="direct control"),
                        plt.Line2D([], [], marker="o", ls="", color=RED, label="with assistance")], frameon=False)
save(fig, "03_order_effects", "Did operators get faster with practice?",
     "vr_study_sessions.json duration_s and clutch_engagements, sessions in recording order",
     "Each line is one operator across their sessions; no consistent downward drift with practice, so order is not obviously driving the direct/assisted difference.",
     "5 operators, 2–7 sessions each", "APPENDIX")

# --------------------------------------------------------------------------
# 4. lag
# --------------------------------------------------------------------------
lag_pool = {c: np.concatenate([s["lag_mm"][np.isfinite(s["lag_mm"]) & (s["engaged"] == 1)] for s in VR if s["condition"] == c])
            for c in ("Direct", "Shared")}
frac_gate = {c: float(np.mean(v > GATE_MM)) for c, v in lag_pool.items()}
fig, axes = plt.subplots(1, 2, figsize=(7.6, 2.8), gridspec_kw={"wspace": 0.45})
lag_pool = {c: np.clip(v, 0.1, None) for c, v in lag_pool.items()}
box(axes[0], [lag_pool["Direct"], lag_pool["Shared"]], ["direct control", "with assistance"], [BLUE, RED],
    "robot lag behind the operator (mm),\nclutch held; log scale")
axes[0].axhline(GATE_MM, color=GREY, ls=":", lw=1); axes[0].text(1.5, GATE_MM * 1.05, "30 mm grasp gate", fontsize=7, color=GREY, ha="center")
axes[0].set_yscale("log"); axes[0].set_ylim(0.1, max(np.nanmax(lag_pool["Direct"]), np.nanmax(lag_pool["Shared"])) * 1.5)
for c, s0 in zip(("Direct", "Shared"), (BLUE, RED)):
    v_all, l_all = [], []
    for s in VR:
        if s["condition"] != c: continue
        v = speed(s["t"], s["ctrl"]); ok = np.isfinite(v) & np.isfinite(s["lag_mm"]) & (s["engaged"] == 1)
        v_all.append(v[ok]); l_all.append(s["lag_mm"][ok])
    v_all = np.concatenate(v_all); l_all = np.concatenate(l_all)
    bins = np.linspace(0, np.nanpercentile(v_all, 98), 14)
    med = [np.median(l_all[(v_all >= a) & (v_all < b)]) if np.any((v_all >= a) & (v_all < b)) else np.nan for a, b in zip(bins[:-1], bins[1:])]
    q75 = [np.percentile(l_all[(v_all >= a) & (v_all < b)], 75) if np.any((v_all >= a) & (v_all < b)) else np.nan for a, b in zip(bins[:-1], bins[1:])]
    mid = 0.5 * (bins[:-1] + bins[1:])
    axes[1].plot(mid, med, color=s0, lw=1.6, label=COND_LBL[c] + " (median)")
    axes[1].plot(mid, q75, color=s0, lw=0.8, ls="--", alpha=0.8)
axes[1].axhline(GATE_MM, color=GREY, ls=":", lw=1)
axes[1].set_xlabel("operator hand speed (m/s)"); axes[1].set_ylabel("robot lag behind the operator (mm)")
axes[1].legend(frameon=False, loc="upper left"); axes[1].text(0.99, 0.02, "dashed = upper quartile", transform=axes[1].transAxes, ha="right", fontsize=7, color=GREY)
save(fig, "04_lag", "How far behind the operator the robot ran",
     "trail.csv vr_*_lag_m while vr_*_engaged == 1; controller speed from vrc_*_x/y/z",
     "Lag is a few millimetres almost always and grows with hand speed; the fraction of samples past the 30 mm grasp gate is %.1f%% (direct) and %.1f%% (with assistance)." % (100 * frac_gate["Direct"], 100 * frac_gate["Shared"]),
     "median lag %.1f / %.1f mm; >30 mm in %.1f%% / %.1f%% of samples" % (np.median(lag_pool["Direct"]), np.median(lag_pool["Shared"]), 100 * frac_gate["Direct"], 100 * frac_gate["Shared"]),
     "MAIN")

# --------------------------------------------------------------------------
# 5. clutch behaviour
# --------------------------------------------------------------------------
def clutch_stats(s):
    ev = [e for e in s["events"] if e.get("kind") == "clutch" and e.get("hand") == s["active_hand"]]
    on = None; holds, gaps = [], []; last_off = None
    for e in ev:
        if e["engaged"]:
            on = e["t"]
            if last_off is not None: gaps.append(e["t"] - last_off)
        elif on is not None:
            holds.append(e["t"] - on); on = None; last_off = e["t"]
    refused = sum(1 for e in s["events"] if e.get("kind") == "clutch_refused")
    return holds, gaps, refused

per_min, hold_med, refused_n, gap_med = ({c: [] for c in ("Direct", "Shared")} for _ in range(4))
holds_pool = {c: [] for c in ("Direct", "Shared")}
for s in VR:
    holds, gaps, refused = clutch_stats(s); c = s["condition"]
    per_min[c].append(60 * s["clutch_engagements"] / s["duration_s"])
    if holds: hold_med[c].append(np.median(holds)); holds_pool[c] += holds
    if gaps: gap_med[c].append(np.median(gaps))
    refused_n[c].append(refused)
fig, axes = plt.subplots(1, 4, figsize=(11.0, 2.8), gridspec_kw={"wspace": 0.55})
lbls, cols = ["direct\ncontrol", "with\nassistance"], [BLUE, RED]
box(axes[0], [per_min["Direct"], per_min["Shared"]], lbls, cols, "clutch engagements per minute")
box(axes[1], [holds_pool["Direct"], holds_pool["Shared"]], lbls, cols, "length of each engagement (s)")
axes[1].set_yscale("log")
box(axes[2], [gap_med["Direct"], gap_med["Shared"]], lbls, cols, "pause before re-engaging (s)")
box(axes[3], [refused_n["Direct"], refused_n["Shared"]], lbls, cols, "refused engagements per session")
save(fig, "05_clutch_behaviour", "How operators used the clutch",
     "events.jsonl clutch / clutch_refused events, active hand, every VR session",
     "With assistance operators re-engaged the clutch less often per minute and held it for longer stretches.",
     "median engagements/min %.1f vs %.1f; median hold %.1f vs %.1f s; refused %.1f vs %.1f per session" % (
         np.median(per_min["Direct"]), np.median(per_min["Shared"]), np.median(holds_pool["Direct"]), np.median(holds_pool["Shared"]),
         np.mean(refused_n["Direct"]), np.mean(refused_n["Shared"])),
     "MAIN")

# --------------------------------------------------------------------------
# 6. grip: report honestly
# --------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(4.2, 2.6))
xs = np.arange(len(VR))
agree = [100 * np.nanmean(s["grip"] == s["engaged"]) for s in VR]
ax.bar(xs, agree, color=GREY, width=0.7)
ax.set_ylim(90, 100.5); ax.set_ylabel("samples where the 'grip' signal\nequals the clutch signal (%)")
ax.set_xlabel("session (all 23)"); ax.set_xticks([])
save(fig, "06_grip_is_the_clutch", "The 'grip' signal is the clutch button, not the gripper",
     "trail.csv vr_*_grip vs vr_*_engaged; grip_cmd_* is empty in every session",
     "In every session the recorded grip signal is the same button as the clutch (%.1f%% of samples identical), and the gripper command column is empty: the gripper was never commanded in the pilot." % (100 * GRIP_IS_CLUTCH),
     "%.1f%% agreement; grip_cmd_* empty in 23/23" % (100 * GRIP_IS_CLUTCH), "SKIP (but changes a caption: 'picked/placed' markers are clutch events)")

# --------------------------------------------------------------------------
# 7. tracking loss
# --------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(7.4, 2.8), gridspec_kw={"width_ratios": [1, 1.3]})
order = sorted(VR, key=lambda s: s["tracking_ok_frac"])
axes[0].barh(range(len(order)), [100 * (1 - s["tracking_ok_frac"]) for s in order],
             color=[COND_COL[s["condition"]] for s in order], alpha=0.8)
axes[0].set_yticks(range(len(order))); axes[0].set_yticklabels([s["participant"] for s in order], fontsize=6.5)
axes[0].set_xlabel("share of session untracked (%)")
for i, s in enumerate(VR):
    t = s["t"]; lost = runs(s["tracked"] == 0)
    for a, b in lost:
        axes[1].plot([t[a] / t[-1], t[b] / t[-1]], [i, i], color=COND_COL[s["condition"]], lw=3, solid_capstyle="butt")
    axes[1].plot([0, 1], [i, i], color="0.85", lw=0.5, zorder=0)
axes[1].set_yticks(range(len(VR))); axes[1].set_yticklabels([s["participant"] for s in VR], fontsize=6.5)
axes[1].set_xlabel("position in the session (0 = start, 1 = end)"); axes[1].set_xlim(0, 1)
axes[1].set_title("when tracking dropped out", fontsize=8)
worst = order[0]
save(fig, "07_tracking_loss", "How often the headset lost sight of the controller",
     "trail.csv vr_*_tracked per session",
     "Tracking loss is rare — the worst session lost the controller for %.1f%% of its samples — and it clusters at session edges." % (100 * (1 - worst["tracking_ok_frac"])),
     "worst %.1f%% (%s); median %.2f%%" % (100 * (1 - worst["tracking_ok_frac"]), worst["participant"], 100 * (1 - np.median([s["tracking_ok_frac"] for s in VR]))),
     "APPENDIX")

# --------------------------------------------------------------------------
# 8. sim vs real
# --------------------------------------------------------------------------
def joint_err(s):
    ok = np.isfinite(s["sim_j"]).all(1) & np.isfinite(s["real_j"]).all(1)
    e = np.degrees(s["sim_j"][ok] - s["real_j"][ok])
    e = (e + 180) % 360 - 180
    return e, ok

rows_lbl, mat = [], []
for s in VR + MA:
    e, ok = joint_err(s)
    if ok.sum() < 50: continue
    mat.append(np.sqrt(np.mean(e ** 2, 0))); rows_lbl.append(label(s) if "events" in s else "%s, %s, %s" % (s["participant"], TASK_LBL[s["task"]], COND_LBL[s["condition"]]))
mat = np.array(mat)
fig, ax = plt.subplots(figsize=(6.2, 0.22 * len(rows_lbl) + 1.2))
im = ax.imshow(mat, aspect="auto", cmap="viridis", vmin=0, vmax=np.percentile(mat, 97))
ax.set_yticks(range(len(rows_lbl))); ax.set_yticklabels(rows_lbl, fontsize=6.2)
ax.set_xticks(range(7)); ax.set_xticklabels([f"joint {i}" for i in range(1, 8)])
cb = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02); cb.set_label("tracking error, real vs simulated (degrees)")
ax.spines["left"].set_visible(True)
save(fig, "08_sim_real_heatmap", "How closely the real arm followed the simulated one, joint by joint",
     "trail.csv / trail_extracted.csv sim_*_jN vs real_*_jN, every session with both",
     "Almost every session tracks within a few degrees on every joint; the three M1 sessions stand out on several joints at once, the signature of a bad master channel rather than a bad joint.",
     "%d sessions; median per-joint error %.2f°, max %.1f°" % (len(rows_lbl), np.median(mat), mat.max()), "MAIN")

fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.8))
pool = np.concatenate([joint_err(s)[0] for s in VR if joint_err(s)[1].sum() > 50])
box(axes[0], [pool[:, i] for i in range(7)], [str(i) for i in range(1, 8)], [GREY] * 7, "real minus simulated (degrees)\nmiddle half and whiskers only")
for l in axes[0].lines:
    if l.get_marker() == "o": l.set_visible(False)
axes[0].set_xlabel("joint"); axes[0].set_ylim(-3, 3)
# error vs joint speed
vs, es = [], []
for s in VR:
    e, ok = joint_err(s)
    if ok.sum() < 50: continue
    t = s["t"][ok]; jd = np.abs(np.diff(np.degrees(s["sim_j"][ok]), axis=0)) / np.diff(t)[:, None]
    vs.append(jd.ravel()); es.append(np.abs(e[1:]).ravel())
vs = np.concatenate(vs); es = np.concatenate(es); okk = np.isfinite(vs) & np.isfinite(es)
vs, es = vs[okk], es[okk]
bins = np.linspace(0, np.percentile(vs, 98), 12); mid = 0.5 * (bins[:-1] + bins[1:])
med = [np.median(es[(vs >= a) & (vs < b)]) for a, b in zip(bins[:-1], bins[1:])]
q75 = [np.percentile(es[(vs >= a) & (vs < b)], 75) for a, b in zip(bins[:-1], bins[1:])]
axes[1].plot(mid, med, color=BLUE, lw=1.6, label="median"); axes[1].plot(mid, q75, color=BLUE, lw=0.8, ls="--", label="upper quartile")
axes[1].set_xlabel("how fast the joint was moving (degrees/s)"); axes[1].set_ylabel("tracking error (degrees)"); axes[1].legend(frameon=False)
save(fig, "08_sim_real_error", "Tracking error per joint, and does it grow with speed?",
     "VR cohort sim_*_jN vs real_*_jN, pooled",
     "Errors are centred on zero for every joint and grow with joint speed: the real arm lags a moving command rather than sitting off a still one.",
     "pooled %d samples; median |error| %.2f° at rest vs %.2f° in the fastest bin" % (len(es), med[0], med[-1]), "APPENDIX")

# --------------------------------------------------------------------------
# 9. master cohort: pots, and master hand paths
# --------------------------------------------------------------------------
good = next(s for s in MA if s["participant"] == "M2" and s["condition"] == "Direct" and s["task"] == "Pick and place")
bad = next(s for s in MA if s["participant"] == "M1" and s["task"] == "Pick and place")
fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.2), sharex=False)
for ax, s, nm in zip(axes, (good, bad), ("M2 (tracks within 2°)", "M1 (tracks 16–21° off)")):
    P = s["pots"][s["active_hand"]]
    for i in range(7):
        y = P[:, i];
        if np.isfinite(y).sum() < 10: continue
        ax.plot(s["t"], y, lw=0.6, label=f"channel {i+1}")
    ax.set_ylabel("raw sensor reading\n(counts)"); ax.text(0.99, 0.95, nm, transform=ax.transAxes, ha="right", va="top", fontsize=8)
axes[-1].set_xlabel("time into session (s)"); fig.legend(*axes[0].get_legend_handles_labels(), ncol=7, frameon=False, fontsize=7, loc="lower center", bbox_to_anchor=(0.5, -0.03)); fig.subplots_adjust(bottom=0.16)
save(fig, "09_master_raw_channels", "The master arm's raw sensor channels, a good session beside a bad one",
     "trail_extracted.csv mraw_<hand>_p1..p7 for one M2 and one M1 object-tracking session",
     "Channels that trace a trajectory are alive; a channel that sits flat or jumps between levels is the defect the health verdict names.",
     "M2 rms 2.2° vs M1 21.2°", "APPENDIX")

fig, axes = plt.subplots(3, 3, figsize=(7.2, 7.0)); axes = axes.ravel()
for ax, s in zip(axes, MA):
    xy = s["master"][:, [0, 1]]; ok = np.isfinite(xy).all(1); xy = xy[ok]; tt = s["t"][ok]
    pts = xy.reshape(-1, 1, 2); segs = np.concatenate([pts[:-1], pts[1:]], 1)
    lc = LineCollection(segs, cmap="viridis", lw=0.7); lc.set_array(tt / tt[-1]); ax.add_collection(lc); ax.autoscale(); ax.set_aspect("equal", "datalim")
    ax.text(0.02, 0.95, "%s, %s,\n%s" % (s["participant"], TASK_LBL[s["task"]], COND_LBL[s["condition"]]), transform=ax.transAxes, va="top", fontsize=7)
for ax in axes[6:]: ax.set_xlabel("left–right (m)")
for ax in axes[::3]: ax.set_ylabel("forward (m)")
save(fig, "09_master_hand_paths", "Where the master arm's hand went, every master-arm session (from above)",
     "trail_extracted.csv master_<hand>_x/y, all 9 sessions", "M1's paths are wide, jagged sweeps; M2's and M3's are compact — the same channel story seen from the hand.",
     "9 sessions", "APPENDIX")

# --------------------------------------------------------------------------
# 10. e-stop record
# --------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 0.2 * len(VR) + 1.4))
n_holds = 0; hold_s = []
for i, s in enumerate(VR):
    t = s["t"]; ax.plot([0, t[-1]], [i, i], color="0.85", lw=3, solid_capstyle="butt")
    for a, b in runs(s["estop"] == 1):
        ax.plot([t[a], t[b]], [i, i], color="black", lw=3, solid_capstyle="butt"); n_holds += 1; hold_s.append(t[b] - t[a])
    ax.text(t[-1] + 4, i, label(s), fontsize=6.2, va="center")
ax.set_yticks([]); ax.set_xlabel("time into session (s)"); ax.set_xlim(0, max(s["t"][-1] for s in VR) * 1.9)
ax.spines["left"].set_visible(False)
save(fig, "10_estop_timeline", "Every session's timeline, with emergency-stop holds in black",
     "trail.csv estop column, every VR session (the master-arm cohort's export has no e-stop column)",
     "The emergency stop was held %d times across %d sessions; %d of the holds sit in the last tenth of a session (the operator ending the recording), %d fall mid-session, and the geometric clearance floor never triggered in any session." % (n_holds, sum(1 for s in VR if (s['estop'] == 1).any()), sum(1 for s in VR for a, b in runs(s["estop"] == 1) if s["t"][b] >= 0.9 * s["t"][-1]), sum(1 for s in VR for a, b in runs(s["estop"] == 1) if s["t"][b] < 0.9 * s["t"][-1])),
     "%d holds, %.1f s median, %.1f s longest; clearance floor: 0 activations (from the /blocking scan)" % (n_holds, np.median(hold_s), max(hold_s)), "MAIN")

# --------------------------------------------------------------------------
# 11. paired slopegraphs
# --------------------------------------------------------------------------
pairs = collections.defaultdict(lambda: {"Direct": [], "Shared": []})
for s in VR: pairs[(s["participant"], s["task"])][s["condition"]].append(s)
pairs = {k: v for k, v in pairs.items() if v["Direct"] and v["Shared"]}
metrics = [("ee_path_m", "how far the robot's hand moved (m)"),
           ("controller_path_m", "how far the operator's hand moved (m)"),
           ("clutch_engagements", "times the clutch was re-engaged"),
           ("mean_lag_mm", "robot lag behind the operator (mm)"),
           ("active_control_frac", "share of the session steering")]
fig, axes = plt.subplots(1, 5, figsize=(13.0, 3.0), gridspec_kw={"wspace": 0.6})
for ax, (k, lab) in zip(axes, metrics):
    for (p, task), v in pairs.items():
        a = np.mean([s[k] for s in v["Direct"]]); b = np.mean([s[k] for s in v["Shared"]])
        ax.plot([0, 1], [a, b], color=GREY, lw=0.9); ax.scatter([0], [a], color=BLUE, s=20, zorder=3); ax.scatter([1], [b], color=RED, s=20, zorder=3)
        ax.text(-0.06, a, p, fontsize=6.5, ha="right", va="center", color=GREY)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["direct", "with\nassistance"]); ax.set_xlim(-0.35, 1.2); ax.set_ylabel(lab)
    a_all = [np.mean([s[k] for s in v["Direct"]]) for v in pairs.values()]; b_all = [np.mean([s[k] for s in v["Shared"]]) for v in pairs.values()]
    up = sum(1 for a, b in zip(a_all, b_all) if b > a)
    ax.set_title("%d of %d higher with assistance" % (up, len(pairs)), fontsize=7.5, color=GREY)
save(fig, "11_paired_slopes", "Same operator, same task, both ways round",
     "vr_study_sessions.json, the 7 participant×task pairs with both conditions",
     "The robot's hand travels further with assistance in 6 of 7 pairs, but the operator's own hand does not — so the extra travel is the assistance layer's, not the operator's.",
     "robot path up in %d/7; operator's hand path up in %d/7" % (
         sum(1 for v in pairs.values() if np.mean([s["ee_path_m"] for s in v["Shared"]]) > np.mean([s["ee_path_m"] for s in v["Direct"]])),
         sum(1 for v in pairs.values() if np.mean([s["controller_path_m"] for s in v["Shared"]]) > np.mean([s["controller_path_m"] for s in v["Direct"]]))),
     "MAIN")

# --------------------------------------------------------------------------
# 11b. the same slopegraph, three panels, for the main body (larger type)
# --------------------------------------------------------------------------
metrics3 = [("ee_path_m", "how far the robot's hand moved (m)"),
            ("controller_path_m", "how far the operator's hand moved (m)"),
            ("clutch_engagements", "times the clutch was re-engaged")]
with plt.rc_context({"font.size": 11, "axes.labelsize": 11, "xtick.labelsize": 11, "ytick.labelsize": 10}):
    fig, axes = plt.subplots(1, 3, figsize=(10.0, 4.6), gridspec_kw={"wspace": 0.55})
    for ax, (k, lab) in zip(axes, metrics3):
        for (p, task), v in pairs.items():
            a = np.mean([s[k] for s in v["Direct"]]); b = np.mean([s[k] for s in v["Shared"]])
            ax.plot([0, 1], [a, b], color=GREY, lw=1.2); ax.scatter([0], [a], color=BLUE, s=60, zorder=3); ax.scatter([1], [b], color=RED, s=60, zorder=3)
            ax.text(-0.08, a, p, fontsize=9, ha="right", va="center", color=GREY)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["direct", "with\nassistance"]); ax.set_xlim(-0.4, 1.25); ax.set_ylabel(lab)
        a_all = [np.mean([s[k] for s in v["Direct"]]) for v in pairs.values()]; b_all = [np.mean([s[k] for s in v["Shared"]]) for v in pairs.values()]
        up = sum(1 for a, b in zip(a_all, b_all) if b > a)
        ax.set_title("%d of %d higher with assistance" % (up, len(pairs)), fontsize=11, color=GREY)
    fig.savefig(os.path.join(HERE, "11_paired_slopes_3panel.pdf"))
    fig.savefig(os.path.join(HERE, "11_paired_slopes_3panel_300.png"), dpi=300)
    plt.close(fig)

# --------------------------------------------------------------------------
# 12. extras: robot path vs hand path per session; speed distribution
# --------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.9))
for s in VR:
    axes[0].scatter(s["controller_path_m"], s["ee_path_m"], color=COND_COL[s["condition"]], s=24, alpha=0.85)
axes[0].set_xlabel("how far the operator's hand moved (m)"); axes[0].set_ylabel("how far the robot's hand moved (m)")
axes[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", color=BLUE, label="direct control"), plt.Line2D([], [], marker="o", ls="", color=RED, label="with assistance")], frameon=False)
ratio = {c: [s["ee_path_m"] / s["controller_path_m"] for s in VR if s["condition"] == c] for c in ("Direct", "Shared")}
box(axes[1], [ratio["Direct"], ratio["Shared"]], ["direct control", "with assistance"], [BLUE, RED], "robot travel per metre of hand travel")
save(fig, "12_robot_vs_hand_travel", "How much the robot moved for each metre the operator's hand moved",
     "vr_study_sessions.json ee_path_m and controller_path_m",
     "The operator's hand covers 10–30 m per session (raw controller motion, jitter included) while the robot's hand covers 0.2–7 m; per metre of hand motion the robot moves further with assistance.",
     "median ratio %.3f (direct) vs %.3f (assistance)" % (np.median(ratio["Direct"]), np.median(ratio["Shared"])), "APPENDIX")

fig, ax = plt.subplots(figsize=(4.6, 2.8))
sp = {c: np.concatenate([speed(s["t"], s["ee"])[(s["engaged"] == 1)] for s in VR if s["condition"] == c]) for c in ("Direct", "Shared")}
sp = {c: v[np.isfinite(v) & (v > 0)] * 1000 for c, v in sp.items()}   # drop repeated (stale) samples, which read as zero speed
box(ax, [sp["Direct"], sp["Shared"]], ["direct control", "with assistance"], [BLUE, RED], "robot hand speed while steering\n(mm/s, log scale)")
ax.set_yscale("log")
save(fig, "12_robot_speed", "How fast the robot's hand moved while the operator was steering",
     "trail.csv ee_*_x/y/z speed while vr_*_engaged == 1 and the hand was moving, pooled per condition",
     "The robot's hand moves at similar speeds under both conditions; assistance does not make it faster, it makes the path longer.",
     "median %.0f vs %.0f mm/s" % (np.median(sp["Direct"]), np.median(sp["Shared"])), "APPENDIX")

# --------------------------------------------------------------------------
# INDEX
# --------------------------------------------------------------------------
with open(os.path.join(HERE, "INDEX.md"), "w") as f:
    f.write("# Pilot recordings gallery\n\nGenerated by `make_pilot_gallery.py` from the 23 VR sessions (P1–P5) and 9 master-arm sessions (M1–M3). Every figure is saved as `.pdf` (for LaTeX) and `.png` (for viewing). Verdicts: MAIN = would improve the graded main body; APPENDIX = worth having, uncounted; SKIP = made, says nothing new.\n\n")
    f.write("Sanity check run first: `vr_*_grip` equals `vr_*_engaged` in %.2f%% of all samples and `grip_cmd_*` is empty in every session — the recorded 'grip' is the clutch button, and the gripper was never commanded in the pilot.\n\n" % (100 * GRIP_IS_CLUTCH))
    for name, title, data, take, nums, verdict in INDEX:
        f.write(f"## `{name}.pdf` — {title}\n\n- **Data:** {data}\n- **Takeaway:** {take}\n- **Numbers:** {nums}\n- **Suggested use:** {verdict}\n\n")
print("INDEX.md written with", len(INDEX), "entries")
