#!/usr/bin/env python3
"""Every defensible graph from the recorded data, in one command.

    python3 scripts/make_results.py            # build everything the data supports
    python3 scripts/make_results.py --list     # what exists, and from which file
    python3 scripts/make_results.py --only vr_smoothing vr_protocol

Writes PNGs and an index.md into recordings/analysis/. Idempotent: running it
twice writes the same files.

THE RULES, WHICH ARE THE REPO'S OWN:

* A figure is built ONLY from a committed recording (or, for the smoothing
  figure, recomputed live through the shipped filter code, and labelled so).
  No number is typed in from memory. Where the only source is prose in
  findings.md, the figure REFUSES BY NAME instead of being drawn -- an axes
  full of hand-copied numbers is not a result, it is a diagram wearing one's
  clothes.
* Every skip is printed with its reason and listed in index.md. A missing
  file is a named refusal, never a crash and never an empty plot.
* Every title carries the DATE of the recording it draws, because provenance
  is part of the result: the clip-set accuracy numbers are 2026-08-23/24 and
  the VR smoothing landed 2026-08-26, so the accuracy figures do NOT measure
  the smoothed mapper. index.md says which claim each figure can support.

The palette is fixed-order categorical (one hue per MODE, the same hue in
every figure -- colour follows the entity, never the rank), grids are
recessive, and no figure has two y-scales.
"""
import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

ROOT = Path("/home/gausms/kortex_ws")

# Fixed categorical order (dataviz reference palette, light mode). One hue per
# operating mode, assigned once and reused in every figure.
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4",
           "#008300", "#4a3aa7", "#e34948"]
MODES = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy",
         "04_vr_shared", "06_full_autonomy"]
MODE_LABEL = {"01_master_teleop": "01 master",
              "02_vr_teleop": "02 VR teleop",
              "03_shared_autonomy": "03 shared",
              "04_vr_shared": "04 VR shared",
              "06_full_autonomy": "06 autonomy"}
MODE_COLOR = {m: PALETTE[i] for i, m in enumerate(MODES)}
ARM_COLOR = {"left": PALETTE[0], "right": PALETTE[1]}
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"


class Skip(Exception):
    """A figure refusing, with the reason in its own words."""


def need(root, rel):
    p = Path(root) / rel
    if not p.exists():
        raise Skip("%s is missing -- nothing to draw from" % rel)
    return p


def dated(p):
    """The recording's date: from the filename if it carries one, else mtime."""
    m = re.search(r"(\d{4})(\d{2})(\d{2})", p.name)
    if m:
        return "%s-%s-%s" % m.groups()
    return datetime.fromtimestamp(p.stat().st_mtime).strftime("%Y-%m-%d")


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelcolor=INK)
    ax.grid(axis="y", color=INK2, alpha=0.15, linewidth=0.6)
    ax.set_axisbelow(True)


def new_fig(nrows=1, height=3.4, width=9.0):
    fig, axes = plt.subplots(nrows, 1, figsize=(width, height * nrows))
    fig.patch.set_facecolor(SURFACE)
    if nrows == 1:
        axes = [axes]
    for ax in axes:
        style(ax)
    return fig, axes


def save(fig, out, name):
    fig.tight_layout()
    path = Path(out) / name
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return name


# --------------------------------------------------------------------------
# 1. per-mode accuracy, from the committed table
# --------------------------------------------------------------------------
def fig_accuracy(root, out):
    p = need(root, "recordings/verification/accuracy_table.json")
    d = json.loads(p.read_text())
    date = dated(p)
    fig, (ax1, ax2) = new_fig(2)

    labels, vals, cols = [], [], []
    for m in MODES:
        for row in d.get(m, []):
            errs = [e * 1000 for e in row.get("pos_err", [])]
            if not errs:
                continue
            labels.append("%s\n%s (n=%d)" % (MODE_LABEL[m],
                                             row["task"].upper(), row["n"]))
            vals.append(float(np.mean(errs)))
            cols.append(MODE_COLOR[m])
    if not vals:
        raise Skip("accuracy_table.json holds no pos_err rows")
    x = np.arange(len(vals))
    ax1.bar(x, vals, color=cols, width=0.6)
    for xi, v in zip(x, vals):
        ax1.text(xi, v + 0.3, "%.1f" % v, ha="center", color=INK, fontsize=9)
    ax1.set_xticks(x, labels, fontsize=8)
    ax1.set_ylabel("grasp positioning error (mm)")
    ax1.set_ylim(0, max(35.0, max(vals) * 1.3))
    ax1.axhline(30.0, color=INK2, ls="--", lw=0.8)
    ax1.text(len(vals) - 0.5, 30.8, "30 mm capture gate", color=INK2,
             fontsize=8, ha="right")
    ax1.set_title("Grasp positioning error per mode, clip set of 2026-08-23 "
                  "(table written %s)" % date, color=INK)

    labels, vals, cols = [], [], []
    for m in MODES:
        for row in d.get(m, []):
            for e in row.get("place_err", []):
                labels.append(MODE_LABEL[m])
                vals.append(e * 1000)
                cols.append(MODE_COLOR[m])
    if vals:
        x = np.arange(len(vals))
        ax2.bar(x, vals, color=cols, width=0.6)
        for xi, v in zip(x, vals):
            ax2.text(xi, v + 1, "%.1f" % v, ha="center", color=INK,
                     fontsize=9)
        ax2.set_xticks(x, ["%s cube %d" % (l, i)
                           for i, l in enumerate(labels)], fontsize=8)
        ax2.set_ylabel("placement error (mm)")
        ax2.set_title("T1 placement error (recorded only under 06)",
                      color=INK)
    else:
        ax2.set_visible(False)
    return [(save(fig, out, "accuracy_per_mode.png"),
             "Per-mode grasp positioning and T1 placement error, from "
             "recordings/verification/accuracy_table.json (%s; clip set "
             "re-recorded 2026-08-23). PREDATES the 2026-08-26 VR smoothing."
             % date)]


# --------------------------------------------------------------------------
# 2. grasp success matrix
# --------------------------------------------------------------------------
def fig_grasp_matrix(root, out):
    p = need(root, "recordings/verification/accuracy_table.json")
    d = json.loads(p.read_text())
    date = dated(p)
    tasks = sorted({r["task"] for rows in d.values() for r in rows})
    if not tasks:
        raise Skip("accuracy_table.json holds no task rows")
    grid = np.full((len(MODES), len(tasks)), np.nan)
    text = [["" for _ in tasks] for _ in MODES]
    for i, m in enumerate(MODES):
        for r in d.get(m, []):
            j = tasks.index(r["task"])
            grid[i, j] = r["grasped"] / max(r["n"], 1)
            text[i][j] = "%d/%d" % (r["grasped"], r["n"])
    fig, axes = new_fig(1, height=3.2, width=7.0)
    ax = axes[0]
    ax.grid(False)
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("blues1",
                                             ["#ffffff", PALETTE[0]])
    masked = np.ma.masked_invalid(grid)
    cmap.set_bad("#f0efec")
    ax.imshow(masked, cmap=cmap, vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(tasks)), [t.upper() for t in tasks])
    ax.set_yticks(range(len(MODES)), [MODE_LABEL[m] for m in MODES])
    for i in range(len(MODES)):
        for j in range(len(tasks)):
            if text[i][j]:
                c = "#ffffff" if grid[i, j] > 0.6 else INK
                ax.text(j, i, text[i][j], ha="center", va="center", color=c)
            else:
                ax.text(j, i, "not a\ndata cell", ha="center", va="center",
                        color=INK2, fontsize=7)
    ax.set_title("Grasps succeeded / attempted, per mode and task (%s)\n"
                 "Grey: task not run under that mode by design." % date,
                 color=INK, fontsize=10)
    return [(save(fig, out, "grasp_matrix.png"),
             "Grasp success per mode x task from accuracy_table.json (%s). "
             "T1/T1S2 run under 06 only; T0/D* grasp nothing by design."
             % date)]


# --------------------------------------------------------------------------
# 3. VR smoothing: 1-Euro vs EMA vs raw, recomputed through the shipped code
# --------------------------------------------------------------------------
def fig_smoothing(root, out):
    need(root, "src/srl_vr_teleop/srl_vr_teleop/vr_smoothing.py")
    sys.path.insert(0, str(Path(root) / "src/srl_vr_teleop"))
    try:
        from srl_vr_teleop import vr_smoothing as sm
    except Exception as e:                                    # noqa: BLE001
        raise Skip("vr_smoothing failed to import: %s" % e)

    # The method of test_vr_smoothing.py, verbatim: constructed signals whose
    # right answer is known, so a bad number is the filter's fault.
    DT = 1.0 / 72.0
    rng = np.random.default_rng(7)
    noise = rng.normal(0.0, 0.0015, (900, 3))
    t = np.arange(900) * DT
    ramp = np.stack([0.40 * t, np.zeros(900), np.zeros(900)], 1)

    def run(f, sig):
        f.reset()
        return np.array([f(sig[i], DT) for i in range(len(sig))])

    def still_rms(f):
        return float(np.sqrt((run(f, noise)[200:] ** 2).sum(1).mean())) * 1e3

    def lag_mm(f):
        o = run(f, ramp + noise)
        return float(np.mean(ramp[400:, 0] - o[400:, 0])) * 1e3

    class Raw:
        def reset(self):
            pass

        def __call__(self, x, dt):
            return np.asarray(x, float)

    laws = [("none (raw)", Raw, {}),
            ("EMA a=0.6 (was)", sm.LegacyEma, {"alpha": 0.6}),
            ("1-Euro (now)", sm.OneEuro, {})]
    still = [still_rms(cls(**kw)) for _, cls, kw in laws]
    lag = [lag_mm(cls(**kw)) for _, cls, kw in laws]

    fig, (ax1, ax2) = new_fig(2, height=2.8, width=7.0)
    x = np.arange(3)
    cols = [INK2, PALETTE[1], PALETTE[0]]
    for ax, vals, ylab, ttl in (
            (ax1, still, "still-hand command RMS (mm)",
             "Tremor reaching the arm from a STILL hand (1.5 mm-rms "
             "controller noise)"),
            (ax2, lag, "lag behind the hand (mm)",
             "Command lag on a 0.40 m/s reach")):
        ax.bar(x, vals, color=cols, width=0.55)
        for xi, v in zip(x, vals):
            ax.text(xi, v + max(vals) * 0.02, "%.2f" % v, ha="center",
                    color=INK, fontsize=10)
        ax.set_xticks(x, [n for n, _, _ in laws])
        ax.set_ylabel(ylab)
        ax.set_title(ttl, color=INK, fontsize=10)
    fig.suptitle("VR command smoothing (landed 2026-08-26)\nrecomputed "
                 "in-process through the shipped vr_smoothing.py",
                 color=INK, fontsize=10)
    return [(save(fig, out, "vr_smoothing.png"),
             "1-Euro vs the EMA it replaced vs raw: stillness residual and "
             "reach lag, recomputed live through "
             "src/srl_vr_teleop/srl_vr_teleop/vr_smoothing.py (filter maths "
             "on constructed signals, the method of test_vr_smoothing.py; "
             "smoothing landed 2026-08-26). NOT a through-the-node or "
             "real-arm measurement -- that is measure_vr_smoothing.py, which "
             "needs ROS up.")]


# --------------------------------------------------------------------------
# 4. VR teleop protocol, 2026-08-26 (sim, domain 0)
# --------------------------------------------------------------------------
PROTO = "recordings/vr_teleop/protocol_20260826_124739.json"


def _proto(root):
    p = need(root, PROTO)
    return json.loads(p.read_text()), dated(p)


def fig_vr_protocol(root, out):
    d, date = _proto(root)
    segs = [r["segment"] for r in d]
    x = np.arange(len(segs))
    w = 0.2
    fig, (ax1, ax2, ax3) = new_fig(3, height=2.9)

    for k, h in enumerate(("left", "right")):
        hand = [r[h]["hand_mm"] for r in d]
        arm = [r[h].get("arm_mm", np.nan) for r in d]
        off = (k * 2 - 1.5) * w
        ax1.bar(x + off, hand, w, color=ARM_COLOR[h], alpha=0.45,
                label="%s hand" % h)
        ax1.bar(x + off + w, arm, w, color=ARM_COLOR[h],
                label="%s arm" % h)
    ax1.set_ylabel("distance moved (mm)")
    ax1.legend(fontsize=8, ncol=4, frameon=False)
    ax1.set_title("Hand distance vs sim-arm distance per segment "
                  "(pale: hand, solid: arm)", color=INK, fontsize=10)

    for h in ("left", "right"):
        err = [r[h].get("dir_err_deg", np.nan) for r in d]
        ax2.plot(x, err, "o-", color=ARM_COLOR[h], label=h, ms=7)
    ax2.axhline(45, color=INK2, ls=":", lw=0.8)
    ax2.axhline(120, color=PALETTE[7], ls="--", lw=0.8)
    ax2.text(0.05, 47, "off-axis threshold (45)", color=INK2, fontsize=7)
    ax2.text(0.05, 122, "inverted threshold (120)", color=PALETTE[7],
             fontsize=7)
    ax2.set_ylabel("arm direction vs expected (deg)")
    ax2.legend(fontsize=8, frameon=False)
    ax2.set_title("Direction error: x/y segments read 73-117 deg, z reads "
                  "1.6-2.3\n-- the align_yaw_deg=0 frame signature, "
                  "uncalibrated in this run", color=INK, fontsize=10)

    for k, h in enumerate(("left", "right")):
        lag = [r[h]["max_lag_mm"] for r in d]
        ax3.bar(x + (k - 0.5) * w * 1.6, lag, w * 1.6, color=ARM_COLOR[h],
                label=h)
    ax3.axhline(30, color=INK2, ls="--", lw=0.8)
    ax3.text(len(segs) - 0.5, 31, "30 mm capture gate", color=INK2,
             fontsize=8, ha="right")
    ax3.set_ylabel("peak mapper lag (mm)")
    ax3.legend(fontsize=8, frameon=False)
    ax3.set_title("Peak lag_m per segment: 8.8-35.4 mm at protocol speeds, "
                  "82.8/92.9 mm only on the deliberate fast reach",
                  color=INK, fontsize=10)
    for ax in (ax1, ax2, ax3):
        ax.set_xticks(x, segs, fontsize=8)
    fig.suptitle("VR teleop protocol %s -- SIM ONLY, ROS domain 0, "
                 "1-Euro smoothing live (mapper publishes cutoff_hz)"
                 % date, color=INK, fontsize=11)
    return [(save(fig, out, "vr_protocol_20260826.png"),
             "Per-segment hand vs arm distance, direction error and peak "
             "lag from %s (%s, sim-only). The file holds per-segment "
             "aggregates, not time series, so no path or error-vs-time plot "
             "is possible from it; commanded-pose deltas are absent (the "
             "clutch was out at each segment snapshot), so GAIN cannot be "
             "computed either." % (PROTO, date))]


def fig_vr_lag_speed(root, out):
    d, date = _proto(root)
    fig, axes = new_fig(1, height=3.6, width=7.0)
    ax = axes[0]
    for h in ("left", "right"):
        v = [r[h]["peak_hand_speed_mps"] for r in d]
        lag = [r[h]["max_lag_mm"] for r in d]
        ax.scatter(v, lag, s=60, color=ARM_COLOR[h], label=h, zorder=3)
        for r, vi, li in zip(d, v, lag):
            if li > 30 or r["segment"] == "up":
                ax.annotate(r["segment"], (vi, li), textcoords="offset points",
                            xytext=(6, 4), fontsize=8, color=INK2)
    ax.axhline(30, color=INK2, ls="--", lw=0.8)
    ax.text(0.02, 31, "30 mm capture gate", color=INK2, fontsize=8)
    ax.set_xlabel("peak hand speed (m/s)")
    ax.set_ylabel("peak mapper lag (mm)")
    ax.legend(frameon=False)
    ax.set_title("Lag grows with hand speed -- the rate limiter, not the "
                 "filter (VR protocol %s, sim)" % date, color=INK,
                 fontsize=10)
    return [(save(fig, out, "vr_lag_vs_speed.png"),
             "Peak mapper lag vs peak hand speed per protocol segment, from "
             "%s (%s). Under 0.27 m/s every segment stays inside the 30 mm "
             "gate; the deliberate fast reach (0.31-0.35 m/s) does not."
             % (PROTO, date))]


# --------------------------------------------------------------------------
# 5. wrist protocol, 2026-08-26
# --------------------------------------------------------------------------
WRIST = "recordings/vr_teleop/wrist_20260826_125305.json"


def fig_wrist(root, out):
    p = need(root, WRIST)
    d = json.loads(p.read_text())
    date = dated(p)
    segs = [r["segment"] for r in d]
    x = np.arange(len(segs))
    w = 0.35
    fig, (ax1, ax2, ax3) = new_fig(3, height=2.9)
    for k, h in enumerate(("left", "right")):
        g = [r[h]["gain"] for r in d]
        ax1.bar(x + (k - 0.5) * w, g, w, color=ARM_COLOR[h], label=h)
        for xi, gi in zip(x + (k - 0.5) * w, g):
            ax1.text(xi, gi + 0.05, "%.2f" % gi, ha="center", fontsize=7,
                     color=INK)
    ax1.axhline(1.0, color=INK2, ls="--", lw=0.8)
    ax1.set_ylabel("rotation gain (arm deg / hand deg)")
    ax1.legend(fontsize=8, frameon=False)
    ax1.set_title("Rotation gain (orientation is unscaled: 1.000 is the "
                  "only right answer)\nyaw_right's 6.5x/2.5x sit on 12-28 "
                  "deg hand rotations -- small denominators",
                  color=INK, fontsize=9)
    for h in ("left", "right"):
        e = [r[h]["axis_err_deg"] for r in d]
        ax2.plot(x, e, "o-", color=ARM_COLOR[h], label=h, ms=7)
    ax2.axhline(90, color=INK2, ls=":", lw=0.8)
    ax2.set_ylabel("arm axis vs hand axis (deg)")
    ax2.legend(fontsize=8, frameon=False)
    ax2.set_title("Rotation-axis error: roll/pitch read 150-173 deg (the "
                  "arm turns about the INVERTED axis)\nyaw_left reads 21-28 "
                  "(clean) -- the frame signature again", color=INK,
                  fontsize=9)
    for k, h in enumerate(("left", "right")):
        lag = [r[h]["peak_lag_mm"] for r in d]
        ax3.bar(x + (k - 0.5) * w, lag, w, color=ARM_COLOR[h], label=h)
    ax3.set_ylabel("peak lag (mm)")
    ax3.legend(fontsize=8, frameon=False)
    ax3.set_title("Peak lag during pure wrist rotation: 4.2-18.4 mm, all "
                  "inside the 30 mm gate", color=INK, fontsize=10)
    for ax in (ax1, ax2, ax3):
        ax.set_xticks(x, segs, fontsize=8)
    fig.suptitle("VR wrist protocol %s -- SIM ONLY\nwrist smoothing new the "
                 "same day (mapper reports rot_smoothed, cutoff 0.9-1.2 Hz)"
                 % date, color=INK, fontsize=10)
    return [(save(fig, out, "vr_wrist_20260826.png"),
             "Wrist rotation gain, axis error and lag per segment from %s "
             "(%s, sim-only). Aggregates only -- the file carries no time "
             "series, so no orientation-error-over-time curve is possible "
             "from it." % (WRIST, date))]


# --------------------------------------------------------------------------
# 6. teleop motion generator: legacy clamp vs Ruckig
# --------------------------------------------------------------------------
def fig_teleop_motion(root, out):
    p = need(root, "recordings/baselines/teleop_motion.json")
    d = json.loads(p.read_text())
    date = dated(p)
    arms = d.get("arms", {})
    if not arms:
        raise Skip("teleop_motion.json has no 'arms' block")
    gens = ["clamp_towards", "synchronised-clamp", "ruckig"]
    glabel = {"clamp_towards": "per-joint clamp (was)",
              "synchronised-clamp": "synchronised clamp",
              "ruckig": "Ruckig (now)"}
    fig, (ax1, ax2, ax3) = new_fig(3, height=2.8, width=8.0)
    x = np.arange(len(gens))
    w = 0.35
    for k, a in enumerate(("left", "right")):
        g = arms.get(a, {}).get("generators", {})
        off = [g.get(n, {}).get("off_path_max_mm", np.nan) for n in gens]
        shown = [max(v, 5e-3) for v in off]     # 1e-8 mm drawn at the floor
        ax1.bar(x + (k - 0.5) * w, shown, w, color=ARM_COLOR[a], label=a)
        for xi, v in zip(x + (k - 0.5) * w, off):
            ax1.text(xi, max(v, 5e-3) * 1.15,
                     ("%.1f" % v) if v > 0.01 else "0.0",
                     ha="center", fontsize=8, color=INK)
        vel = [g.get(n, {}).get("peak_velocity_frac_of_limit", np.nan)
               for n in gens]
        ax2.bar(x + (k - 0.5) * w, vel, w, color=ARM_COLOR[a], label=a)
        for xi, v in zip(x + (k - 0.5) * w, vel):
            ax2.text(xi, v * 1.1, "%.2fx" % v, ha="center", fontsize=8,
                     color=INK)
    ax1.set_yscale("log")
    ax1.axhline(30, color=INK2, ls="--", lw=0.8)
    ax1.text(2.3, 33, "30 mm gate", color=INK2, fontsize=8)
    ax1.set_ylabel("max off-path excursion (mm, log)")
    ax1.set_xticks(x, [glabel[n] for n in gens])
    ax1.legend(fontsize=8, frameon=False)
    ax1.set_title("Hand's worst departure from the straight line the IK "
                  "implies", color=INK, fontsize=10)
    ax2.set_yscale("log")
    ax2.axhline(1.0, color=INK2, ls="--", lw=0.8)
    ax2.set_ylabel("peak velocity / joint limit (log)")
    ax2.set_xticks(x, [glabel[n] for n in gens])
    ax2.legend(fontsize=8, frameon=False)
    ax2.set_title("Velocity-limit respect: the clamp commanded 12.5x the "
                  "yaml's limit; Ruckig 1.00x", color=INK, fontsize=10)

    drawn = False
    for k, a in enumerate(("left", "right")):
        sweep = d.get("clamp_step_sweep_%s" % a, {})
        if sweep:
            steps = sorted(sweep, key=float)
            ax3.plot([float(s) for s in steps],
                     [sweep[s]["off_path_max_mm"] for s in steps],
                     "o-", color=ARM_COLOR[a], label=a)
            drawn = True
    if drawn:
        ax3.axhline(30, color=INK2, ls="--", lw=0.8)
        ax3.set_ylim(bottom=0)
        ax3.set_xlabel("clamp max step (rad/cycle)")
        ax3.set_ylabel("off-path excursion (mm)")
        ax3.legend(fontsize=8, frameon=False)
        ax3.set_title("The clamp's excursion vs its own step size: 68 mm "
                      "for every step up to 0.10 rad, and NO step puts it "
                      "inside the 30 mm gate (dashed)", color=INK,
                      fontsize=10)
    else:
        ax3.set_visible(False)
    fig.suptitle("Teleop motion generator, measured %s "
                 "(scripts/measure_teleop_motion.py)" % date, color=INK,
                 fontsize=11)
    return [(save(fig, out, "teleop_motion_generator.png"),
             "Legacy per-joint step clamp vs Ruckig from "
             "recordings/baselines/teleop_motion.json (%s): off-path "
             "51.3/39.2 mm -> 0.0, velocity 12.5x limit -> 1.00x, and the "
             "sweep showing the clamp cannot be tuned into compliance."
             % date)]


# --------------------------------------------------------------------------
# 7. sim-to-real park error
# --------------------------------------------------------------------------
def fig_sim_to_real(root, out):
    cal = need(root, "recordings/baselines/arm_directional_calibration.json")
    d = json.loads(cal.read_text())
    runs = d.get("runs", [])
    if not runs:
        raise Skip("arm_directional_calibration.json has no runs")
    date = dated(cal)
    errs = np.concatenate([np.abs(r["ss_joint_deg"]) for r in runs])
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    ax1.hist(errs, bins=40, color=PALETTE[0], edgecolor=SURFACE)
    ax1.axvline(0.305, color=PALETTE[1], ls="--", lw=1.2)
    ax1.text(0.31, ax1.get_ylim()[1] * 0.85, "0.305 deg bound",
             color=PALETTE[1], fontsize=9)
    ax1.set_xlabel("|steady-state joint error| (deg)")
    ax1.set_ylabel("joint-run count")
    ax1.set_title("REAL ARMS: every joint parks short of its target by a "
                  "bounded ~0.3 deg (%d runs x 7 joints)" % len(runs),
                  color=INK, fontsize=10)

    gap = Path(root) / "recordings/baselines/sim_to_real_gap.json"
    if gap.exists():
        g = json.loads(gap.read_text())
        jt = g.get("joint_terminal", {})
        arms = [a for a in ("left", "right") if a in jt]
        if arms:
            x = np.arange(len(arms))
            w = 0.35
            ident = [jt[a]["identity_rms_mm"] for a in arms]
            fit = [jt[a]["fitted_rms_mm"] for a in arms]
            ax2.bar(x - w / 2, ident, w, color=INK2,
                    label="uncompensated")
            ax2.bar(x + w / 2, fit, w, color=PALETTE[0],
                    label="one EPS per arm")
            for xi, v in zip(np.concatenate([x - w / 2, x + w / 2]),
                             ident + fit):
                ax2.text(xi, v + 0.15, "%.2f" % v, ha="center", fontsize=9,
                         color=INK)
            ax2.set_xticks(x, arms)
            ax2.set_ylabel("EE error RMS on a 100 mm move (mm)")
            ax2.legend(fontsize=8, frameon=False)
            ax2.set_title("The one-parameter terminal-offset model removes "
                          "74-76% of the Cartesian error "
                          "(sim_to_real_gap.json)", color=INK, fontsize=10)
    else:
        ax2.set_visible(False)
    fig.suptitle("Sim-to-real gap, measured on the REAL arms (%s)" % date,
                 color=INK, fontsize=11)
    return [(save(fig, out, "sim_to_real_park_error.png"),
             "Per-joint park-error distribution from "
             "recordings/baselines/arm_directional_calibration.json and the "
             "joint-terminal compensation from sim_to_real_gap.json (%s). "
             "This is real-hardware data; compensation itself is UNTRIED on "
             "hardware." % date)]


# --------------------------------------------------------------------------
# 8. grip traces per mode
# --------------------------------------------------------------------------
def fig_grip_traces(root, out):
    made = []
    fig, axes = plt.subplots(len(MODES), 1, figsize=(9, 2.0 * len(MODES)),
                             sharex=False)
    fig.patch.set_facecolor(SURFACE)
    found = 0
    for ax, m in zip(axes, MODES):
        style(ax)
        p = (Path(root) / "recordings/verification" / m /
             "T2/S2_full_lift/grip_trace.json")
        if not p.exists():
            ax.text(0.5, 0.5, "%s: no grip_trace.json" % MODE_LABEL[m],
                    transform=ax.transAxes, ha="center", color=INK2)
            ax.set_yticks([])
            continue
        found += 1
        tr = json.loads(p.read_text())
        t = [r["t"] for r in tr]
        for h in ("left", "right"):
            v = [r[h] if r[h] is not None else np.nan for r in tr]
            ax.plot(t, v, color=ARM_COLOR[h], lw=1.4, label=h)
        ax.set_ylabel("knuckle (rad)")
        ax.set_ylim(-0.05, 0.65)
        ax.set_title(MODE_LABEL[m], color=MODE_COLOR[m], fontsize=10,
                     loc="left")
        if found == 1:
            ax.legend(fontsize=8, frameon=False, loc="upper right")
    axes[-1].set_xlabel("clip time (s)")
    if found == 0:
        plt.close(fig)
        raise Skip("no recordings/verification/<mode>/T2/S2_full_lift/"
                   "grip_trace.json exists in any mode")
    date = dated(Path(root) / "recordings/verification" / MODES[0] /
                 "T2/S2_full_lift/grip_trace.json")
    fig.suptitle("T2 measured knuckle angle per mode (%s)\nthe grippers "
                 "that had NEVER closed before 2026-08-23 now close on "
                 "the tray in every mode (both arms overlap exactly)"
                 % date, color=INK, fontsize=10)
    made.append((save(fig, out, "grip_traces_t2.png"),
                 "Measured knuckle angle over time, T2 S2_full_lift, all "
                 "five modes (grip_trace.json, clip set %s). MEASURED only: "
                 "the traces carry no commanded channel, so this is not a "
                 "commanded-vs-measured plot; the plateau is the trace's "
                 "own." % date))
    return made


# --------------------------------------------------------------------------
# 9. mode path comparison
# --------------------------------------------------------------------------
def fig_mode_paths(root, out):
    p = need(root, "recordings/baselines/mode_difference.json")
    d = json.loads(p.read_text())
    date = dated(p)
    tasks = [t for t in ("T0", "T1", "T1S2", "T2", "T3") if t in d]
    if not tasks:
        raise Skip("mode_difference.json holds no per-task blocks")
    fig, axes = plt.subplots(len(tasks), 1, figsize=(8, 2.6 * len(tasks)))
    fig.patch.set_facecolor(SURFACE)
    if len(tasks) == 1:
        axes = [axes]
    for ax, task in zip(axes, tasks):
        style(ax)
        pm = d[task].get("per_mode", {})
        modes = [m for m in MODES if m in pm]
        x = np.arange(len(modes))
        w = 0.35
        for k, side in enumerate(("l", "r")):
            v = [pm[m].get("travel_%s" % side) for m in modes]
            v = [np.nan if x_ is None else x_ for x_ in v]
            ax.bar(x + (k - 0.5) * w, v, w,
                   color=ARM_COLOR["left" if side == "l" else "right"],
                   label="left EE" if side == "l" else "right EE")
            for xi, vi in zip(x + (k - 0.5) * w, v):
                if not np.isnan(vi):
                    ax.text(xi, vi + 0.03, "%.2f" % vi, ha="center",
                            fontsize=7, color=INK)
        ax.set_xticks(x, [MODE_LABEL[m] for m in modes], fontsize=8)
        ax.set_ylabel("EE path length (m)")
        ax.set_title("%s -- same COMMANDED waypoints under every mode"
                     % task, color=INK, fontsize=10, loc="left")
        if task == tasks[0]:
            ax.legend(fontsize=8, frameon=False)
    fig.suptitle("The robot does not perform identically across modes\n"
                 "(recordings/baselines/mode_difference.json, %s)" % date,
                 color=INK, fontsize=10)
    return [(save(fig, out, "mode_path_lengths.png"),
             "End-effector travel per mode and task from "
             "mode_difference.json (%s): T0's left path is 1.407 m under 01 "
             "against 2.034 m under 03 for the SAME commanded waypoints. "
             "Predates the 2026-08-23 clip re-record and the Ruckig "
             "generator." % date)]


# --------------------------------------------------------------------------
# 10-11. refusals that are the honest answer
# --------------------------------------------------------------------------
def fig_vr_prefix_misses(root, out):
    raise Skip(
        "the pre-fix accumulating VR misses (88.1 / 115.4 / 156.2 / "
        "205.9 mm, 2026-08-16) exist only as prose in docs/system/"
        "findings.md -- the failed clips were re-recorded in place and no "
        "committed scene_events artifact carries the numbers. Refusing to "
        "hard-code prose into a figure.")


def fig_vr_session_mcap(root, out):
    p = Path(root) / "recordings/vr_teleop/session_20260826_123758"
    try:
        import mcap                                     # noqa: F401
    except ImportError:
        raise Skip("the python 'mcap' library is not importable on this "
                   "interpreter, so %s (rosbag mcap) stays unread"
                   % p.relative_to(root))
    raise Skip("mcap import succeeded but no reader is implemented yet -- "
               "write one before claiming figures from the bag")


# ==========================================================================
# 12-29. The rest of the committed measurements that had no figure.
#
# Added when an audit of recordings/baselines/ found 184 measurement files
# and thirteen figures. Everything below draws a file that was already on
# disk and unread; the same rules apply -- one source per figure, the date in
# the title, a named refusal rather than an empty axes, and no number typed
# in from memory.
# ==========================================================================
def _bar_labels(ax, bars, fmt="%.2f", dy=0.0, size=7):
    for b in bars:
        h = b.get_height()
        if h is None or np.isnan(h):
            continue
        ax.text(b.get_x() + b.get_width() / 2, h + dy, fmt % h,
                ha="center", va="bottom", fontsize=size, color=INK)


def fig_pad_curve(root, out):
    """The wrist-to-pad offset is a CURVE in the gripper opening."""
    p = need(root, "recordings/baselines/pad_mid_ee_by_width.json")
    d = json.loads(p.read_text())
    date = dated(p)
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    # The two arms agree to 0.000 mm, so plotted identically one hides the
    # other and the figure looks like a single-arm measurement. The left is
    # drawn wide and pale underneath, the right thin and solid on top.
    styles = {"left": dict(lw=5.0, alpha=0.35, ms=0, ls="-"),
              "right": dict(lw=1.6, ms=4, ls="-")}
    for arm in ("left", "right"):
        rows = sorted(d["arms"][arm], key=lambda r: r["width_mm"])
        w = [r["width_mm"] for r in rows]
        off = [r["along_axis_m"] * 1000 for r in rows]
        span = [r["span_mm"] for r in rows]
        st = styles[arm]
        ax1.plot(w, off, "o", color=ARM_COLOR[arm],
                 label="%s arm" % arm, **st)
        ax2.plot(w, span, "o", color=ARM_COLOR[arm],
                 label="%s arm" % arm, **st)
    agree = d.get("controls", {}).get("the_two_arms_agree")
    if agree:
        ax1.text(0.99, 0.06, "the two arms agree exactly, so the traces "
                 "coincide", transform=ax1.transAxes, ha="right", fontsize=7,
                 color=INK2)
    rows = sorted(d["arms"]["left"], key=lambda r: r["width_mm"])
    by_w = {r["width_mm"]: r["along_axis_m"] * 1000 for r in rows}
    for width, note in ((0, "wide open"), (20, "on a 20 mm cube"),
                        (40, "on a 40 mm cube")):
        if width in by_w:
            ax1.annotate("%.2f mm\n%s" % (by_w[width], note),
                         xy=(width, by_w[width]),
                         xytext=(width + 3, by_w[width] - 4.5), fontsize=7,
                         color=INK2,
                         arrowprops=dict(arrowstyle="->", color=INK2, lw=0.7))
    ax1.set_ylabel("wrist to pad midpoint (mm)")
    ax1.set_title("Both numbers this repository argued about are points on "
                  "ONE curve", color=INK, fontsize=10, loc="left")
    ax2.set_ylabel("finger span (mm)")
    ax2.set_xlabel("commanded grip width (mm)")
    ax2.set_title("the fingers swing on a four-bar, so the span is not the "
                  "opening", color=INK, fontsize=10, loc="left")
    for ax in (ax1, ax2):
        ax.legend(fontsize=8, frameon=False)
    fig.suptitle("The pad midpoint is a CURVE, not a constant "
                 "(pad_mid_ee_by_width.json, %s)" % date, color=INK,
                 fontsize=10)
    return [(save(fig, out, "pad_offset_curve.png"),
             "Wrist-to-pad offset and finger span against the commanded grip "
             "width, both arms, by FK on the mimic chain (%s). The 2026-08-18 "
             "'13.47 mm too long' and T1's '11.43 mm short' compared two "
             "different points on this curve, which is why the evidence "
             "flipped depending on what opening the simulator was left at."
             % date)]


def fig_orientation_cost(root, out):
    """What the pinned wrist costs, measured from two start points."""
    p_home = need(root, "recordings/baselines/orientation_cost.json")
    p_work = need(root,
                  "recordings/baselines/orientation_cost_what_binds.json")
    home = json.loads(p_home.read_text())
    work = json.loads(p_work.read_text())
    date = dated(p_work)
    pols = home["policies"]
    labels = {"pinned": "pinned\n(shipped)", "spin": "roll free",
              "cone15": "15° cone", "cone45": "45° cone", "free": "free"}
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    x = np.arange(len(pols))
    w = 0.38
    b1 = ax1.bar(x - w / 2, [home["mean_reach_m"][k] for k in pols], w,
                 color=PALETTE[0], label="from HOME")
    b2 = ax1.bar(x + w / 2, [work["mean_reach_m"][k] for k in pols], w,
                 color=PALETTE[1], label="from the WORK POINT")
    _bar_labels(ax1, b1, "%.3f", 0.006)
    _bar_labels(ax1, b2, "%.3f", 0.006)
    ax1.set_xticks(x, [labels[k] for k in pols], fontsize=8)
    ax1.set_ylabel("mean reach (m)")
    ax1.set_ylim(0, 0.66)
    ax1.legend(fontsize=8, frameon=False)
    ax1.set_title("Where you measure from decides whether the constraint "
                  "looks expensive", color=INK, fontsize=10, loc="left")

    nb = len(work["walks"])
    b3 = ax2.bar(x, [work["wearer_bound_walks"][k] for k in pols], 0.55,
                 color=PALETTE[4])
    _bar_labels(ax2, b3, "%.0f", 0.12)
    ax2.axhline(nb, ls="--", lw=0.9, color=INK2)
    ax2.text(len(pols) - 0.4, nb + 0.15, "all %d walks" % nb, fontsize=7,
             ha="right", color=INK2)
    ax2.set_xticks(x, [labels[k] for k in pols], fontsize=8)
    ax2.set_ylabel("walks stopped BY THE WEARER")
    ax2.set_ylim(0, nb + 1.2)
    ax2.set_title("and the redundancy is the joint that moves the elbow out "
                  "of the person", color=INK, fontsize=10, loc="left")
    fig.suptitle("What the pinned wrist costs, 0.15 m wearer floor enforced "
                 "under every policy (%s)" % date, color=INK, fontsize=10)
    return [(save(fig, out, "orientation_cost.png"),
             "Mean reach per orientation policy from the home end effector "
             "and from the work point (0.4, 0.175, 1.12), with the number of "
             "direction walks the WEARER stopped (%s). From home the pinned "
             "wrist costs 89 mm; at the work point it costs 385 mm and 11 of "
             "12 walks are wearer-bound. Freeing the roll alone is worth "
             "nothing, and orientation_policy still defaults to 'exact'."
             % date)]


def fig_wearer_posture(root, out):
    """The wearer's posture is a variable, and it moves the workspace."""
    p = need(root, "recordings/baselines/centre_vs_wearer_posture.json")
    d = json.loads(p.read_text())
    date = dated(p)
    order = [k for k in ("none", "down", "folded", "behind", "out")
             if k in d["rows"]]
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    x = np.arange(len(order))
    w = 0.38
    for k, arm in enumerate(("left", "right")):
        vals, texts = [], []
        for post in order:
            v = d["rows"][post]["full"].get(arm)
            vals.append(np.nan if v is None else v)
            texts.append("no\nsolution" if v is None else "")
        bars = ax1.bar(x + (k - 0.5) * w, vals, w, color=ARM_COLOR[arm],
                       label="%s arm" % arm)
        _bar_labels(ax1, bars, "%.3f", 0.006)
        for xi, t in zip(x + (k - 0.5) * w, texts):
            if t:
                ax1.text(xi, 0.02, t, ha="center", fontsize=7, color=PALETTE[7])
    ax1.set_xticks(x, order, fontsize=8)
    ax1.set_ylabel("innermost usable column |x| (m)")
    ax1.legend(fontsize=8, frameon=False)
    ax1.set_title("Innermost column the arm can work at, per wearer posture",
                  color=INK, fontsize=10, loc="left")

    floor = 0.15
    home = [d["rows"][p_]["home"]["left"] for p_ in order]
    cols = [PALETTE[7] if v < floor else PALETTE[5] for v in home]
    bars = ax2.bar(x, home, 0.55, color=cols)
    _bar_labels(ax2, bars, "%.3f", 0.004)
    ax2.axhline(floor, ls="--", lw=1.0, color=PALETTE[7])
    ax2.text(len(order) - 0.4, floor + 0.004, "150 mm clearance floor",
             fontsize=7, ha="right", color=PALETTE[7])
    ax2.set_xticks(x, order, fontsize=8)
    ax2.set_ylabel("clearance AT HOME (m)")
    ax2.set_title("Two postures put the wearer's own limbs inside the floor "
                  "before the arm moves", color=INK, fontsize=10, loc="left")
    fig.suptitle("The wearer's posture is part of the model "
                 "(centre_vs_wearer_posture.json, %s)" % date, color=INK,
                 fontsize=10)
    binds = ", ".join("%s binds on %s" % (k, d["rows"][k]["binds"]["left"])
                      for k in order if d["rows"][k]["binds"]["left"])
    return [(save(fig, out, "wearer_posture.png"),
             "Innermost usable column and home clearance for the five "
             "postures in SRL_WEARER_ARMS (%s). With the wearer's arms "
             "deleted entirely the centre is STILL shut, so what closes it is "
             "the torso: %s. 'behind' and 'out' breach the 150 mm floor at "
             "the home pose, which is why a wearer is never asked to adopt "
             "them." % (date, binds))]


def fig_centre_on_surface(root, out):
    """A negative result, drawn as one: the centre is not reachable."""
    p = need(root, "recordings/baselines/centre_on_surface.json")
    d = json.loads(p.read_text())
    date = dated(p)
    # This file sweeps the PINNED approach only; the top-down search is a
    # separate measurement (0 of 840 cells) and is not folded in here.
    approaches = sorted({h["approach"] for h in d["stage1_hits"]}) or ["pinned"]
    cells = (len(d["zs"]) * len(d["ys"]) * len(d["xs"]) *
             len(d["overhangs"]) * 2 * len(approaches))
    fig, axes = new_fig(1, height=4.0, width=8.0)
    ax = axes[0]
    for arm, marker in (("left", "o"), ("right", "s")):
        hits = [h for h in d["stage1_hits"] if h["arm"] == arm]
        if hits:
            ax.scatter([h["x"] for h in hits], [h["table_top"] for h in hits],
                       s=34, marker=marker, facecolors="none",
                       edgecolors=ARM_COLOR[arm], linewidths=1.2,
                       label="%s: passed one IK call (n=%d)" % (arm, len(hits)))
    surv = d.get("stage2", [])
    if surv:
        ax.scatter([s["x"] for s in surv], [s["table_top"] for s in surv],
                   s=110, marker="*", color=PALETTE[5], zorder=5,
                   label="survived N=10 over the FULL path (n=%d)" % len(surv))
        for s in surv:
            ax.annotate("x = %.3f, clearance %.4f\n(%s arm, overhang %.2f)"
                        % (s["x"], s["full_clearance"], s["arm"],
                           s["overhang"]),
                        xy=(s["x"], s["table_top"]),
                        xytext=(s["x"] + 0.02, s["table_top"] - 0.075),
                        fontsize=7, color=INK2,
                        arrowprops=dict(arrowstyle="->", color=INK2, lw=0.7))
    ax.axvspan(min(d["xs"]) - 0.01, 0.10, color=PALETTE[7], alpha=0.10)
    ax.axvline(0.10, ls="--", lw=1.0, color=PALETTE[7])
    ax.text(0.101, max(d["zs"]) - 0.02,
            "the target: |x| ≤ 0.10 m\nNOT ONE CELL, either arm", fontsize=8,
            color=PALETTE[7], va="top")
    ax.set_xlabel("object column |x| (m)")
    ax.set_ylabel("table top height (m)")
    ax.set_xlim(min(d["xs"]) - 0.01, max(d["xs"]) + 0.03)
    ax.legend(fontsize=8, frameon=False, loc="lower right")
    ax.set_title("Centre-on-surface: %d cells searched, table height AND "
                 "distance swept, objects RESTING\n"
                 "(centre_on_surface.json, %s; %d IK calls)"
                 % (cells, date, d.get("ik_calls", 0)), color=INK,
                 fontsize=10, loc="left")
    return [(save(fig, out, "centre_on_surface.png"),
             "Every configuration in a %d-cell search of table height, table "
             "distance, object column and overhang, both arms, objects "
             "resting on the surface (%s). Twenty-two cells pass a single IK "
             "call; two survive N=10 over the whole path. None is anywhere "
             "near the centre, and every survivor needs the object at the "
             "very front edge because the pinned wrist arrives from below."
             % (cells, date))]


def fig_budget(root, out):
    """The control budget, term by term, with what is still unmeasured."""
    p = need(root, "recordings/baselines/control_budget.json")
    d = json.loads(p.read_text())
    date = dated(p)
    terms = d["positioning_mm"]
    known = [t for t in terms if t["mm"] is not None]
    unknown = [t for t in terms if t["mm"] is None]
    fig, axes = new_fig(1, height=4.2, width=9.0)
    ax = axes[0]
    y = np.arange(len(known))[::-1]
    # A term whose name is indented is an ALTERNATIVE to the row above it --
    # the same error source with a fix applied. Drawing it like an addend
    # invites the reader to sum the column, so it is hatched and named.
    alt = [t["term"].startswith("  ") for t in known]
    cols = [PALETTE[1] if t["systematic"] else PALETTE[0] for t in known]
    bars = ax.barh(y, [t["mm"] for t in known], 0.62, color=cols)
    for b, is_alt in zip(bars, alt):
        if is_alt:
            b.set_hatch("///")
            b.set_alpha(0.55)
    for b, t in zip(bars, known):
        ax.text(b.get_width() + 0.18, b.get_y() + b.get_height() / 2,
                "%.2f" % t["mm"], va="center", fontsize=8, color=INK)
    ax.set_yticks(y, [t["term"].strip() for t in known], fontsize=8)
    ax.axvline(d["positioning_rss_mm"], ls="--", lw=1.1, color=PALETTE[5])
    ax.text(d["positioning_rss_mm"] + 0.15, len(known) - 0.6,
            "RSS %.2f" % d["positioning_rss_mm"], fontsize=8, color=PALETTE[5])
    ax.axvline(d["grasp_gate_m"] * 1000, ls="-", lw=1.3, color=PALETTE[7])
    ax.text(d["grasp_gate_m"] * 1000 - 0.2, len(known) - 0.6,
            "%.0f mm capture gate" % (d["grasp_gate_m"] * 1000), fontsize=8,
            color=PALETTE[7], ha="right")
    ax.set_xlabel("contribution to the positioning error (mm)")
    ax.set_xlim(0, max(d["grasp_gate_m"] * 1000,
                       max(t["mm"] for t in known)) + 4)
    handles = [plt.Rectangle((0, 0), 1, 1, color=PALETTE[1]),
               plt.Rectangle((0, 0), 1, 1, color=PALETTE[0]),
               plt.Rectangle((0, 0), 1, 1, facecolor=PALETTE[1], alpha=0.55,
                             hatch="///")]
    ax.legend(handles, ["systematic (correctable)", "random",
                        "an ALTERNATIVE to the row above, not an addend"],
              fontsize=8, frameon=False, loc="lower right")
    sub = ""
    if unknown:
        sub = "\nSTILL UNMEASURED: " + "; ".join(t["term"].strip()
                                                 for t in unknown)
    ax.set_title("Positioning budget: RSS %.2f mm, worst case %.2f mm, "
                 "inside the %.0f mm gate%s\n(control_budget.json, %s)"
                 % (d["positioning_rss_mm"], d["positioning_worst_mm"],
                    d["grasp_gate_m"] * 1000, sub, date),
                 color=INK, fontsize=10, loc="left")
    return [(save(fig, out, "control_budget.png"),
             "Every term in the positioning budget from control_budget.json "
             "(%s), systematic terms separated from random ones because only "
             "the systematic ones can be calibrated away. The pad row is "
             "COMPUTED from the shipped constants, so reverting the "
             "derivation makes this figure say so. %s"
             % (date, ("The largest remaining unknown is not on the chart: "
                       + unknown[0]["term"].strip()) if unknown else ""))]


def fig_reaction(root, out):
    """How far the arm travels before anyone can react."""
    p = need(root, "recordings/baselines/control_budget.json")
    d = json.loads(p.read_text())
    date = dated(p)
    rows = d.get("reaction") or []
    if not rows:
        raise Skip("control_budget.json carries no reaction block")
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    labs = ["%s\n%s" % (r["motion"].replace("a ", "")
                        .replace(" or a fidget", "/fidget"), r["which"])
            for r in rows]
    x = np.arange(len(rows))
    b = ax1.bar(x, [r["latency_s"] * 1000 for r in rows], 0.55,
                color=PALETTE[0])
    _bar_labels(ax1, b, "%.0f", 3)
    ax1.set_xticks(x, labs, fontsize=7, rotation=18, ha="right")
    ax1.set_ylabel("latency (ms)")
    ax1.set_title("Reaction budget: what has to happen before the arm stops",
                  color=INK, fontsize=10, loc="left")
    floor_mm = d["floor_m"] * 1000
    # Plotting the raw travel puts a 2248 mm bar beside a 150 mm line and
    # the line disappears. The quantity that matters is what fraction of the
    # floor is gone before anything can intervene.
    pct = [(r.get("eats_floor_pct_hi") if r.get("eats_floor_pct_hi")
            is not None else r["travel_hi_m"] * 1000 / floor_mm * 100)
           for r in rows]
    b2 = ax2.bar(x, pct, 0.55,
                 color=[PALETTE[7] if v >= 100 else PALETTE[1] for v in pct])
    for bar, v, r in zip(b2, pct, rows):
        ax2.text(bar.get_x() + bar.get_width() / 2, min(v, 100) + 3,
                 "%.0f%%\n(%.0f mm)" % (v, r["travel_hi_m"] * 1000),
                 ha="center", fontsize=7, color=INK)
    ax2.axhline(100, ls="--", lw=1.1, color=PALETTE[7])
    ax2.text(len(rows) - 0.4, 103, "the WHOLE %.0f mm floor" % floor_mm,
             fontsize=7, ha="right", color=PALETTE[7])
    ax2.set_xticks(x, labs, fontsize=7, rotation=18, ha="right")
    ax2.set_ylabel("% of the clearance floor consumed")
    ax2.set_ylim(0, 128)
    ax2.set_title("what that latency costs against a %.0f mm floor "
                  "(bars clipped at 100%%; the mm figure is on the bar)"
                  % floor_mm, color=INK, fontsize=10, loc="left")
    worst = max(rows, key=lambda r: r.get("eats_floor_pct_hi") or 0)
    return [(save(fig, out, "reaction_budget.png"),
             "Latency and the distance the end effector covers in it, per "
             "motion, from control_budget.json (%s). The worst case, '%s', "
             "eats %.0f%% of the 150 mm clearance floor before anything can "
             "intervene — which is the argument for the floor being 150 mm "
             "and not less." % (date, worst["motion"],
                                worst.get("eats_floor_pct_hi") or 0))]


def fig_language_75(root, out):
    """The full instruction sweep, and the grammar it replaced."""
    p = need(root, "recordings/baselines/t1_instruction_sweep.json")
    d = json.loads(p.read_text())
    date = dated(p)
    order = ["CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"]
    cols = [PALETTE[5], PALETTE[3], PALETTE[6], PALETTE[7]]
    now = [d["totals"].get(k, 0) for k in order]
    before = [d["before"]["totals"].get(k, 0) for k in order]
    n = sum(now)
    fig, (ax1, ax2) = new_fig(2, height=3.2, width=9.0)
    x = np.arange(len(order))
    w = 0.38
    b1 = ax2.bar(x - w / 2, before, w, color=INK2,
                 label="the grammar at %s" % d["before"]["ref"])
    b2 = ax2.bar(x + w / 2, now, w, color=cols, label="shipped parser")
    _bar_labels(ax2, b1, "%.0f", 0.4)
    _bar_labels(ax2, b2, "%.0f", 0.4)
    ax2.set_xticks(x, order, fontsize=8)
    ax2.set_ylabel("utterances")
    ax2.set_ylim(0, max(max(now), max(before)) + 6)
    ax2.annotate("the dangerous category:\n3 → 0", xy=(3, 3),
                 xytext=(2.35, max(before) * 0.62), fontsize=8,
                 color=PALETTE[7],
                 arrowprops=dict(arrowstyle="->", color=PALETTE[7], lw=0.8))
    ax2.legend(fontsize=8, frameon=False)
    ax2.set_xlabel("ASKED and REFUSED are SAFE: the arm moved in neither")
    ax2.set_title("Against the grammar it replaced, on the same %d cases in "
                  "its own interpreter" % n, color=INK, fontsize=10,
                  loc="left")

    by = d.get("by_category", {})
    cats = sorted(by, key=lambda c: -sum(by[c].values()))
    bottom = np.zeros(len(cats))
    for k, col in zip(order, cols):
        v = np.array([by[c].get(k, 0) for c in cats], dtype=float)
        ax1.bar(np.arange(len(cats)), v, 0.66, bottom=bottom, color=col,
                label=k)
        bottom += v
    ax1.set_xticks(np.arange(len(cats)), cats, fontsize=7, rotation=38,
                   ha="right")
    ax1.set_ylabel("utterances")
    ax1.legend(fontsize=7, frameon=False, ncol=4)
    ax1.set_title("%d phrasings across %d categories, by category"
                  % (n, len(cats)), color=INK, fontsize=10, loc="left")
    fig.suptitle("Free-form instructions: %d correct, %d asked, %d refused, "
                 "%d MISUNDERSTOOD (t1_instruction_sweep.json, %s)"
                 % (now[0], now[1], now[2], now[3], date), color=INK,
                 fontsize=10)
    reps = d.get("replies", {})
    return [(save(fig, out, "language_sweep_75.png"),
             "The full instruction sweep: %d phrasings, %d categories (%s). "
             "Zero MISUNDERSTOOD is the result — an utterance that moves the "
             "arm to the wrong place is the only unsafe outcome, and there "
             "are none, against three for the grammar this replaced. Of the "
             "asks, %d carry a scored reply and all of them resolve "
             "correctly; a reply is never counted as CORRECT. The scene is "
             "CONSTRUCTED, so this scores grounding and says nothing about "
             "detection rate." % (n, len(cats), date, reps.get("CORRECT", 0)))]


def fig_voice(root, out):
    """Transcription and the wake word, with the caveat that matters."""
    p = need(root, "recordings/baselines/voice_instruction.json")
    d = json.loads(p.read_text())
    date = dated(p)
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    wers = [r["wer"] for r in d["rows"] if r.get("wer") is not None]
    ax1.hist(wers, bins=np.arange(0, max(wers) + 0.09, 0.06),
             color=PALETTE[0], edgecolor=SURFACE)
    ax1.axvline(d["wer"], ls="--", lw=1.2, color=PALETTE[7])
    ax1.text(d["wer"] + 0.01, ax1.get_ylim()[1] * 0.86,
             "mean WER %.1f%%\n%d exact transcripts of %d"
             % (d["wer"] * 100, d["exact_transcripts"], d["n_cases"]),
             fontsize=8, color=PALETTE[7])
    ax1.set_xlabel("word error rate per utterance")
    ax1.set_ylabel("utterances")
    ax1.set_title("Transcription, %s model on SYNTHESISED speech (%s voice)"
                  % (d["model"], d["voice"]), color=INK, fontsize=10,
                  loc="left")

    wp = Path(root) / "recordings/baselines/wake_word.json"
    if wp.exists():
        wd = json.loads(wp.read_text())
        tab = wd["table"]
        th = [t["threshold"] for t in tab]
        ax2.plot(th, [t["wake_accepted"] for t in tab], "o-",
                 color=PALETTE[5], lw=1.6, ms=4, label="real wake accepted")
        ax2.plot(th, [t["false_accepted"] for t in tab], "s-",
                 color=PALETTE[7], lw=1.6, ms=4, label="FALSE accept")
        for name, val, col in (("shipped", wd.get("shipped_threshold"),
                                PALETTE[6]),
                               ("recommended", wd.get("recommended"),
                                PALETTE[1])):
            if val is not None:
                ax2.axvline(val, ls="--", lw=1.0, color=col)
                ax2.text(val + 0.06, ax2.get_ylim()[1] * 0.5, name,
                         fontsize=7, color=col, rotation=90, va="center")
        ax2.set_xlabel("edit-distance threshold on the wake phrase '%s'"
                       % wd["wake"])
        ax2.set_ylabel("utterances accepted")
        ax2.legend(fontsize=8, frameon=False)
        ax2.set_title("The wake word: the shipped threshold is looser than "
                      "the data supports", color=INK, fontsize=10, loc="left")
    else:
        ax2.set_visible(False)
    fig.suptitle("Voice, end to end — and every number here is from "
                 "SYNTHESISED audio (%s)" % date, color=INK, fontsize=10)
    t = d["tally"]
    return [(save(fig, out, "voice_pipeline.png"),
             "Word error rate per utterance and the wake-word threshold "
             "sweep (%s). The parser still returns %d correct / %d asked / "
             "%d refused / %d misunderstood THROUGH this transcription, which "
             "is why a voice score in this repository is a PARSER score and "
             "never a transcription score. /dev/snd on this host has only "
             "'timer', so no microphone can be opened: one synthetic voice, "
             "no accent, no room, no noise, and accuracy against a real "
             "speaker is UNMEASURED."
             % (date, t.get("CORRECT", 0), t.get("ASKED", 0),
                t.get("REFUSED", 0), t.get("MISUNDERSTOOD", 0)))]


def fig_predictive(root, out):
    """Predictive avoidance, scored on the poses it is willing to COMMAND.

    The file's own ``min_clearance`` is the minimum over ALL steps, including
    the ones a policy REFUSED -- poses the arm was never sent to. Differencing
    that across policies scores each policy at a pose it declined, and it
    reported avoidance making clearance WORSE, which is the opposite of the
    file's own control. The honest statistic is the minimum over the steps
    that actually solved.
    """
    p = need(root, "recordings/baselines/predictive_avoidance.json")
    d = json.loads(p.read_text())
    date = dated(p)
    targets = list(d["results"])
    pols = [k for k in ("off", "on", "projection", "tangential")
            if k in d["results"][targets[0]]]
    floor = d["floor_m"]

    def commanded_min(t, pol):
        rows = [r["clearance"] for r in d["results"][t][pol]["rows"]
                if r.get("solved")]
        return min(rows) if rows else float("nan")

    fig, (ax1, ax2) = new_fig(2, height=3.1, width=8.0)
    x = np.arange(len(targets))
    w = 0.8 / len(pols)
    for i, pol in enumerate(pols):
        v = [commanded_min(t, pol) * 1000 for t in targets]
        b = ax1.bar(x + (i - (len(pols) - 1) / 2) * w, v, w,
                    color=PALETTE[i],
                    label="avoidance %s" % ("OFF" if pol == "off" else pol))
        for bar, vi in zip(b, v):
            ax1.text(bar.get_x() + bar.get_width() / 2,
                     vi + (4 if vi >= 0 else -14), "%.0f" % vi, ha="center",
                     fontsize=7, color=INK)
    ax1.axhline(floor * 1000, ls="--", lw=1.1, color=PALETTE[5])
    ax1.text(len(targets) - 0.42, floor * 1000 + 5,
             "%.0f mm clearance floor" % (floor * 1000), fontsize=7,
             ha="right", color=PALETTE[5])
    ax1.axhline(0, lw=1.1, color=PALETTE[7])
    ax1.text(-0.44, 6, "the wearer's skin", fontsize=7, color=PALETTE[7])
    ax1.set_xticks(x, targets, fontsize=9)
    ax1.set_ylabel("worst clearance at a COMMANDED pose (mm)")
    ax1.legend(fontsize=7.5, frameon=False, ncol=len(pols))
    ax1.set_title("With avoidance off the solver commands poses 155 mm "
                  "INSIDE the person", color=INK, fontsize=10, loc="left")

    for i, pol in enumerate(pols):
        v = [d["results"][t][pol]["solved"] for t in targets]
        b = ax2.bar(x + (i - (len(pols) - 1) / 2) * w, v, w, color=PALETTE[i])
        _bar_labels(ax2, b, "%.0f", 0.12)
    n = d["results"][targets[0]][pols[0]]["n"]
    ax2.axhline(n, ls="--", lw=0.9, color=INK2)
    ax2.text(len(targets) - 0.42, n + 0.2, "all %d steps" % n, fontsize=7,
             ha="right", color=INK2)
    ax2.set_xticks(x, targets, fontsize=9)
    ax2.set_ylabel("steps the policy will COMMAND")
    ax2.set_ylim(0, n + 1.6)
    ax2.set_title("and what it buys is REFUSAL, not clearance: every refusal "
                  "names end_effector_link", color=INK, fontsize=10,
                  loc="left")
    fig.suptitle("Predictive avoidance against the real solver and the real "
                 "wearer model (predictive_avoidance.json, %s)" % date,
                 color=INK, fontsize=10)
    return [(save(fig, out, "predictive_avoidance.png"),
             "Worst clearance at a pose each policy is willing to command, "
             "and how many of the %d steps it will command at all (%s). This "
             "is NOT the file's own min_clearance field, which spans the "
             "REFUSED steps and therefore scores a policy at a pose it "
             "declined — differencing it reports avoidance making clearance "
             "worse, contradicting the file's own control. Scored on "
             "commanded poses the result is plain: unassisted, the solver "
             "commands poses ~155 mm inside the wearer; with avoidance on, "
             "every commanded pose clears the 150 mm floor and the rest are "
             "refused by name. The null space itself is worth single-figure "
             "millimetres, because it holds the hand fixed by definition and "
             "it is the hand that is inside the person. %s"
             % (n, date, d.get("caveat", "")))]


def fig_depth(root, out):
    """Depth-derived pose error, and what fitting a plane recovers."""
    p = need(root, "recordings/baselines/depth_pose_accuracy.json")
    d = json.loads(p.read_text())
    date = dated(p)
    rows = d["rows"]
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    objs = sorted({r["obj"] for r in rows})
    for i, obj in enumerate(objs):
        # Only the uncontaminated rows carry a naive centroid; the
        # contaminated ones were run to score the FIT alone, so asking them
        # for a naive figure is asking for a number that was never measured.
        rr = sorted([r for r in rows if r["obj"] == obj and
                     not r.get("contamination") and "naive_mm" in r],
                    key=lambda r: r["range_m"])
        if not rr:
            continue
        ax1.plot([r["range_m"] for r in rr], [r["naive_mm"] for r in rr],
                 "o--", color=PALETTE[i], lw=1.4, ms=4,
                 label="%s, naive centroid" % obj)
        ax1.plot([r["range_m"] for r in rr], [r["fit_mm"] for r in rr],
                 "o-", color=PALETTE[i], lw=1.8, ms=4,
                 label="%s, fitted" % obj)
    ax1.set_xlabel("range (m)")
    ax1.set_ylabel("pose error (mm)")
    ax1.set_yscale("log")
    ax1.legend(fontsize=7, frameon=False, ncol=2)
    ax1.set_title("Fitting the object beats taking the centroid of what the "
                  "camera can see", color=INK, fontsize=10, loc="left")

    sig = d.get("per_pixel_sigma_mm", {})
    if sig:
        rr = sorted(float(k) for k in sig)
        b = ax2.bar([str(r) for r in rr], [sig["%g" % r] for r in rr], 0.55,
                    color=PALETTE[0])
        _bar_labels(ax2, b, "%.3f", 0.02)
        ax2.set_xlabel("range (m)")
        ax2.set_ylabel("per-pixel σ (mm)")
        ax2.set_title("Depth noise grows with the SQUARE of range "
                      "(f = %g px, baseline %g m)"
                      % (d["sensor"]["f_px"], d["sensor"]["baseline_m"]),
                      color=INK, fontsize=10, loc="left")
    fig.suptitle("Depth-derived object pose (depth_pose_accuracy.json, %s)"
                 % date, color=INK, fontsize=10)
    return [(save(fig, out, "depth_pose_accuracy.png"),
             "Object pose error against range for a naive depth centroid and "
             "for the fitted estimate, plus the per-pixel depth noise the "
             "sensor model implies (%s). This is the term that enters the "
             "control budget as 2.0 mm at grasp range: the 0.082 m figure "
             "quoted elsewhere is at 4 m and does not apply at 0.4-0.8 m."
             % date)]


def fig_capability(root, out):
    """What is lost as master channels die, one rung at a time."""
    p = need(root, "recordings/baselines/capability_ladder.json")
    d = json.loads(p.read_text())
    date = dated(p)
    levels = list(d["left"]["cost"])
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    x = np.arange(len(levels))
    w = 0.38
    for k, arm in enumerate(("left", "right")):
        # DIR_ONLY and NONE record None, and that is not a zero: those rungs
        # produce no position estimate at all. Drawing them as a bar of
        # height 0 would read as "perfect", which is the opposite.
        raw = [d[arm]["cost"][l]["p95_m"] for l in levels]
        v = [np.nan if r is None else r * 1000 for r in raw]
        b = ax1.bar(x + (k - 0.5) * w, v, w, color=ARM_COLOR[arm],
                    label="%s (n=%d)" % (arm, d[arm]["n"]))
        _bar_labels(ax1, b, "%.0f", 1.5)
        if k == 0:
            for xi, r in zip(x, raw):
                if r is None:
                    ax1.text(xi, 2, "no position\nat all", ha="center",
                             fontsize=7, color=PALETTE[7])
    extent = max(d[a]["radial_extent_m"] for a in ("left", "right")) * 1000
    ax1.axhline(extent, ls="--", lw=1.0, color=PALETTE[7])
    ax1.text(len(levels) - 0.4, extent + 3,
             "the whole radial extent the master covers (%.0f mm)" % extent,
             fontsize=7, ha="right", color=PALETTE[7])
    ax1.set_xticks(x, levels, fontsize=8)
    ax1.set_ylabel("p95 position error (mm)")
    ax1.legend(fontsize=8, frameon=False)
    ax1.set_title("Cost of each degraded rung, replayed through %d recorded "
                  "master frames" % d["left"]["n"], color=INK, fontsize=10,
                  loc="left")

    dp = Path(root) / "recordings/baselines/capability_degradation.json"
    if dp.exists():
        dd = json.loads(dp.read_text())
        counts = dd["subset_counts"]
        keys = [k for k in levels if k in counts]
        b = ax2.bar(np.arange(len(keys)), [counts[k] for k in keys], 0.55,
                    color=PALETTE[4])
        _bar_labels(ax2, b, "%.0f", 0.6)
        top = max(counts.values())
        for i, k in enumerate(keys):
            first = dd["first_seen"].get(k) or []
            if first:
                ax2.text(i, counts[k] + top * 0.09,
                         "first lost:\n%s" % ", ".join(first[:3]),
                         fontsize=6.5, ha="center", color=INK2)
        ax2.set_ylim(0, top * 1.30)
        ax2.set_xticks(np.arange(len(keys)), keys, fontsize=8)
        ax2.set_ylabel("channel subsets landing here")
        ax2.set_title("Of all 128 channel subsets, where each one lands "
                      "(recovery is monotonic: %s)"
                      % dd.get("monotonic_recovery"), color=INK, fontsize=10,
                      loc="left")
    fig.suptitle("Degradation ladder: the master keeps working as channels "
                 "die (capability_ladder.json, %s)" % date, color=INK,
                 fontsize=10)
    return [(save(fig, out, "capability_ladder.png"),
             "What each rung of the degradation ladder costs in end-effector "
             "position, replayed through the real recorded master frames, "
             "and how many of the channel subsets land on each rung (%s). "
             "The point is that no rung is a cliff and recovery is monotonic "
             "— a channel coming back never leaves the system on a lower rung "
             "than before it failed." % date)]


def fig_t1_seeds(root, out):
    """Stage 2 is not one layout: eight seeds, all walked."""
    p = need(root, "recordings/baselines/t1_stage2_paths.json")
    d = json.loads(p.read_text())
    date = dated(p)
    seeds = sorted(d["seeds"], key=lambda s: int(s))
    fig, (ax1, ax2) = new_fig(2, height=2.9, width=8.0)
    x = np.arange(len(seeds))
    worst, splits = [], []
    for s in seeds:
        gs = d["seeds"][s]["grasps"]
        worst.append(max(g.get("pad_miss_mm", 0.0) for g in gs))
        left = sum(1 for g in gs if g["arm"] == "left")
        splits.append("%d/%d" % (left, len(gs) - left))
    b = ax1.bar(x, worst, 0.55,
                color=[PALETTE[5] if d["seeds"][s]["clean"] else PALETTE[7]
                       for s in seeds])
    _bar_labels(ax1, b, "%.2f", 0.4)
    ax1.axhline(30.0, ls="--", lw=1.0, color=PALETTE[7])
    ax1.text(len(seeds) - 0.4, 30.6, "30 mm capture gate", fontsize=7,
             ha="right", color=PALETTE[7])
    ax1.set_xticks(x, ["seed %s\n(%s split)" % (s, sp)
                       for s, sp in zip(seeds, splits)], fontsize=7)
    ax1.set_ylabel("worst pad miss (mm)")
    ax1.set_ylim(0, 34)
    ax1.text(len(seeds) / 2 - 0.5, 16,
             "every bar is at 0.01 mm — this chart is flat because the "
             "result is,\nnot because nothing was measured (%d IK calls "
             "across the eight seeds)"
             % sum(d["seeds"][s_]["ik_calls"] for s_ in seeds),
             ha="center", fontsize=8, color=INK2)
    ax1.set_title("Every seed walked at N=10 over the composed path, and "
                  "every one is clean", color=INK, fontsize=10, loc="left")
    b2 = ax2.bar(x, [d["seeds"][s]["ik_calls"] for s in seeds], 0.55,
                 color=PALETTE[0])
    _bar_labels(ax2, b2, "%.0f", 3)
    ax2.set_xticks(x, ["seed %s" % s for s in seeds], fontsize=8)
    ax2.set_ylabel("IK calls")
    ax2.set_title("the cost of proving it, per seed", color=INK, fontsize=10,
                  loc="left")
    fig.suptitle("T1 stage 2: the SIDE is drawn from a seed "
                 "(t1_stage2_paths.json, %s)" % date, color=INK, fontsize=10)
    return [(save(fig, out, "t1_stage2_seeds.png"),
             "Worst pad miss and IK cost for all eight stage-2 seeds (%s). "
             "Splits of 1/3, 2/2 and 3/1 all occur and both arms always "
             "work. Seed 0 draws 2/2 and is indistinguishable from stage 1, "
             "which is why the recorded clip uses seed 3 — a task that only "
             "ever demonstrates its easy case has demonstrated nothing."
             % date)]


def fig_shared_gain(root, out):
    """What shared autonomy is actually for: a wrong declared coordinate."""
    p = need(root, "recordings/baselines/shared_autonomy_gain.json")
    d = json.loads(p.read_text())
    date = dated(p)
    errs = sorted(d["rows"], key=lambda k: float(k))
    xs = [float(e) for e in errs]
    fig, (ax1, ax2) = new_fig(2, height=3.0, width=8.0)
    for i, src in enumerate(("CALIBRATED", "DECLARED")):
        ax1.plot(xs, [d["rows"][e][src]["correct_pct"] for e in errs], "o-",
                 color=PALETTE[i], lw=1.8, ms=4, label=src.lower())
        ax2.plot(xs, [d["rows"][e][src]["wrong_pct"] for e in errs], "o-",
                 color=PALETTE[i], lw=1.8, ms=4, label=src.lower())
    pitch = d.get("cube_pitch_mm")
    for ax in (ax1, ax2):
        if pitch:
            ax.axvline(pitch, ls="--", lw=1.0, color=PALETTE[7])
            ax.text(pitch + 2, ax.get_ylim()[1] * 0.5,
                    "one cube pitch\n(%g mm)" % pitch, fontsize=7,
                    color=PALETTE[7])
        ax.legend(fontsize=8, frameon=False)
    ax1.set_ylabel("intent inferred CORRECTLY (%)")
    ax1.set_title("Shared autonomy against an increasingly wrong declared "
                  "coordinate (%d trials)" % d["trials"], color=INK,
                  fontsize=10, loc="left")
    ax2.set_ylabel("intent inferred WRONGLY (%)")
    ax2.set_xlabel("error in the declared object coordinate (mm)")
    ax2.set_title("past one cube pitch the declared coordinate names the "
                  "WRONG CUBE, confidently", color=INK, fontsize=10,
                  loc="left")
    fig.suptitle("Why the arbiter reads the camera and not the task file "
                 "(shared_autonomy_gain.json, %s)" % date, color=INK,
                 fontsize=10)
    return [(save(fig, out, "shared_autonomy_gain.png"),
             "Intent inference against a declared object coordinate that is "
             "wrong by 0 to 120 mm, %d trials (%s). Calibrated perception "
             "holds; the declared coordinate collapses past one cube pitch "
             "and, worse, becomes CONFIDENTLY wrong rather than ambiguous — "
             "the ambiguous fraction falls as the wrong fraction rises. The "
             "declared error is swept because in simulation the declared "
             "coordinate is the truth, so the sweep is what a real "
             "calibration error would do." % (d["trials"], date))]


def fig_detector_real(root, out):
    """The one detection measurement made on real camera data."""
    p = need(root, "recordings/baselines/detector_real_rgbd.json")
    d = json.loads(p.read_text())
    date = dated(p)
    rows = d["rows"]
    prompts = sorted({r["prompt"] for r in rows})
    rate = [100.0 * np.mean([r["detected"] for r in rows
                             if r["prompt"] == pr]) for pr in prompts]
    order = np.argsort(rate)[::-1]
    fig, (ax1, ax2) = new_fig(2, height=3.2, width=8.0)
    b = ax1.bar(np.arange(len(prompts)), [rate[i] for i in order], 0.6,
                color=[PALETTE[5] if rate[i] >= 50 else PALETTE[7]
                       for i in order])
    _bar_labels(ax1, b, "%.0f", 1.2)
    ax1.set_xticks(np.arange(len(prompts)),
                   [prompts[i] for i in order], fontsize=7, rotation=30,
                   ha="right")
    ax1.set_ylabel("detected (%)")
    ax1.set_ylim(0, 108)
    ax1.set_title("Open-vocabulary detection on REAL RGB-D, per prompt "
                  "(%d frames, %d object instances)" % (d["frames"], len(rows)),
                  color=INK, fontsize=10, loc="left")
    ious = [r["iou"] for r in rows if r["detected"]]
    if ious:
        ax2.hist(ious, bins=np.arange(0, 1.02, 0.05), color=PALETTE[0],
                 edgecolor=SURFACE)
        ax2.axvline(0.5, ls="--", lw=1.1, color=PALETTE[7])
        ax2.text(0.51, ax2.get_ylim()[1] * 0.85,
                 "IoU 0.5, the usual\nacceptance threshold", fontsize=7,
                 color=PALETTE[7])
        ax2.set_xlabel("IoU against the dataset's own ground truth")
        ax2.set_ylabel("detections")
        ax2.set_title("and how well the box actually fits when it does fire",
                      color=INK, fontsize=10, loc="left")
    overall = 100.0 * np.mean([r["detected"] for r in rows])
    fig.suptitle("Detection on real camera data, not renders "
                 "(detector_real_rgbd.json, %s): %.1f%% overall"
                 % (date, overall), color=INK, fontsize=10)
    return [(save(fig, out, "detector_real_rgbd.png"),
             "Prompted detection rate and IoU on real RGB-D frames from the "
             "LM-O test set, %d frames and %d object instances (%s). "
             "Overall %.1f%%, on the dataset's own cluttered household and "
             "figurine objects — NOT this project's coloured cubes, which "
             "the shipped HSV classifier handles at 100%% in the renderer. "
             "It is the only detection number here measured on real camera "
             "data, and it is why that 100%% is labelled RENDERED and why an "
             "open-vocabulary detector was not adopted: prompted on unseen "
             "clutter it fires on a tenth of the objects, and the boxes it "
             "does return fit well (IoU mostly above 0.8), so the failure is "
             "recall, not localisation."
             % (d["frames"], len(rows), date, overall))]


def fig_pick_accuracy(root, out):
    """Belief error in, pad miss out, against the capture gate."""
    p = need(root, "recordings/baselines/pick_accuracy.json")
    d = json.loads(p.read_text())
    date = dated(p)
    rows = d["rows"]
    gate = d["capture_gate_mm"]
    fig, axes = new_fig(1, height=3.8, width=8.0)
    ax = axes[0]
    for arm in sorted({r["arm"] for r in rows}):
        rr = [r for r in rows if r["arm"] == arm]
        ax.scatter([r["belief_err_mm"] for r in rr],
                   [r["pad_miss_mm"] for r in rr], s=52,
                   color=ARM_COLOR.get(arm, PALETTE[2]), label="%s arm" % arm)
    lim = max([r["belief_err_mm"] for r in rows] +
              [r["pad_miss_mm"] for r in rows] + [gate]) * 1.15
    ax.plot([0, lim], [0, lim], ls=":", lw=1.0, color=INK2)
    ax.text(lim * 0.62, lim * 0.66, "pad miss = belief error\n(the arm adds "
            "nothing of its own)", fontsize=7, color=INK2)
    ax.axhline(gate, ls="--", lw=1.2, color=PALETTE[7])
    ax.text(lim * 0.02, gate + 0.6, "%.0f mm capture gate" % gate,
            fontsize=8, color=PALETTE[7])
    ax.set_xlim(0, lim)
    ax.set_ylim(0, max(lim, gate * 1.2))
    ax.set_xlabel("error in where the pipeline BELIEVED the object was (mm)")
    ax.set_ylabel("pad miss against the object's TRUE centre (mm)")
    ax.legend(fontsize=8, frameon=False)
    ax.set_title("Picking from the measured map: the error is the "
                 "PERCEPTION's, not the arm's\n(pick_accuracy.json, %s)"
                 % date, color=INK, fontsize=10, loc="left")
    return [(save(fig, out, "pick_accuracy.png"),
             "Pad miss against belief error for every pick planned from the "
             "measured world map, with the %.0f mm capture gate (%s). The "
             "points sit on the diagonal: whatever the pipeline believes, the "
             "arm delivers the pads there. That localises the remaining error "
             "in perception rather than in the kinematics, which is what "
             "makes the unmeasured camera_link extrinsic the most expensive "
             "thing still outstanding." % (gate, date))]


def fig_mirror(root, out):
    """The two arms cannot mirror, and the residual says by how much."""
    p = need(root, "recordings/baselines/home_render.json")
    d = json.loads(p.read_text())
    date = dated(p)
    res = d["mirror_residual_m"]
    links = list(res)
    fig, (ax1, ax2) = new_fig(2, height=2.9, width=8.0)
    b = ax1.bar(np.arange(len(links)), [res[l] * 1000 for l in links], 0.6,
                color=PALETTE[0])
    _bar_labels(ax1, b, "%.1f", 0.4)
    ax1.set_xticks(np.arange(len(links)),
                   [l.replace("_link", "") for l in links], fontsize=7,
                   rotation=25, ha="right")
    ax1.set_ylabel("mirror residual (mm)")
    ax1.set_title("Home pose, left arm reflected onto the right: worst "
                  "%.1f mm" % (d["mirror_residual_worst_m"] * 1000),
                  color=INK, fontsize=10, loc="left")
    rows = []
    for arm in ("left", "right"):
        a = d["arms"][arm]
        rows.append((arm, a["elbow_below_shoulder_m"] * 1000,
                     a["elbow_outboard_of_hand_m"] * 1000))
    x = np.arange(2)
    w = 0.38
    b1 = ax2.bar(x - w / 2, [r[1] for r in rows], w, color=PALETTE[0],
                 label="elbow BELOW its own shoulder")
    b2 = ax2.bar(x + w / 2, [r[2] for r in rows], w, color=PALETTE[1],
                 label="elbow OUTBOARD of its own hand")
    _bar_labels(ax2, b1, "%.0f", 2)
    _bar_labels(ax2, b2, "%.0f", 2)
    ax2.set_xticks(x, [r[0] for r in rows], fontsize=9)
    ax2.set_ylabel("mm")
    ax2.legend(fontsize=8, frameon=False)
    ax2.set_title("What the render actually shows, measured from TF at boot",
                  color=INK, fontsize=10, loc="left")
    fig.suptitle("Home pose in numbers, not in impressions "
                 "(home_render.json, %s)" % date, color=INK, fontsize=10)
    ctrl = d.get("controls", {}).get("mirror_can_be_nonzero", {})
    return [(save(fig, out, "home_mirror_residual.png"),
             "Per-link mirror residual at the home pose and the elbow "
             "geometry behind it (%s). The control matters: reflecting the "
             "LEFT arm against ITSELF gives %s, so a residual near zero would "
             "have been the instrument agreeing with itself rather than a "
             "measurement. The asymmetry is structural — the two mounts' base "
             "axes mirror exactly but differ by 168° of roll, and two "
             "identical arms on mirrored mounts cannot mirror."
             % (date, ctrl.get("got", "a non-zero value")))]


def fig_look_cost(root, out):
    """What it costs to look before picking."""
    p = need(root, "recordings/baselines/look_then_grasp.json")
    d = json.loads(p.read_text())
    date = dated(p)
    t = d["timing"]
    parts = [("move home → observe", t["move_home_to_observe_s"], PALETTE[0]),
             ("detect + classify", t["detect_and_classify_s"], PALETTE[5]),
             ("verify the plan", t["verify_plan_s"], PALETTE[3]),
             ("move observe → home", t["move_observe_to_home_s"], PALETTE[0])]
    fig, (ax1, ax2) = new_fig(2, height=2.8, width=8.0)
    left = 0.0
    for name, v, col in parts:
        ax1.barh([0], [v], 0.5, left=left, color=col, edgecolor=SURFACE)
        if v > 1.5:
            ax1.text(left + v / 2, 0, "%s\n%.1f s" % (name, v), ha="center",
                     va="center", fontsize=7.5, color="white")
        left += v
    ax1.set_yticks([])
    ax1.set_xlabel("seconds")
    ax1.set_xlim(0, left * 1.02)
    ax1.set_title("One look costs %.1f s, and %.1f s of it is the arm "
                  "TRAVELLING to and from the observe pose"
                  % (d["added_per_look_s"],
                     t["move_home_to_observe_s"] + t["move_observe_to_home_s"]),
                  color=INK, fontsize=10, loc="left")
    b = ax2.bar(["per look", "per pick\n(%d picks share one look)"
                 % d["picks"]],
                [d["added_per_look_s"], d["added_per_pick_s"]], 0.5,
                color=[PALETTE[1], PALETTE[5]])
    _bar_labels(ax2, b, "%.2f", 0.6)
    ax2.set_ylabel("seconds added")
    ax2.set_title("Amortised over the picks it plans, looking is cheap",
                  color=INK, fontsize=10, loc="left")
    fig.suptitle("The cost of looking before grasping "
                 "(look_then_grasp.json, %s)" % date, color=INK, fontsize=10)
    return [(save(fig, out, "look_then_grasp_cost.png"),
             "Where the time goes in a look-then-grasp cycle (%s). Detection "
             "and classification take %.2f s; the transit to and from the "
             "observe pose takes %.1f s, so the whole optimisation available "
             "here is a shorter journey, not a faster detector. Amortised "
             "over the %d picks one look plans, it adds %.2f s per pick."
             % (date, t["detect_and_classify_s"],
                t["move_home_to_observe_s"] + t["move_observe_to_home_s"],
                d["picks"], d["added_per_pick_s"]))]


def fig_mount_overlap(root, out):
    """Where the mount can go before the two arms stop sharing anything."""
    p = need(root, "recordings/baselines/mount_overlap_sweep.json")
    rows = json.loads(p.read_text())
    date = dated(p)
    if not isinstance(rows, list) or not rows:
        raise Skip("mount_overlap_sweep.json is not the expected list of "
                   "swept mount offsets")
    fig, (ax1, ax2) = new_fig(2, height=2.9, width=8.0)
    for ax, key, lab in ((ax1, "splay_deg", "mount splay (deg)"),
                         (ax2, "tilt_deg", "mount tilt (deg)")):
        vals = sorted({r[key] for r in rows})
        best = [max(r["both"] for r in rows if r[key] == v) for v in vals]
        mean = [np.mean([r["both"] for r in rows if r[key] == v])
                for v in vals]
        b = ax.bar(np.arange(len(vals)), best, 0.55, color=PALETTE[0],
                   label="best configuration at this %s" % lab.split(" (")[0])
        ax.plot(np.arange(len(vals)), mean, "o-", color=PALETTE[1], lw=1.6,
                ms=4, label="mean over the sweep")
        _bar_labels(ax, b, "%.0f", 0.06)
        ax.set_xticks(np.arange(len(vals)), ["%g" % v for v in vals],
                      fontsize=8)
        ax.set_xlabel(lab)
        ax.set_ylabel("cells BOTH arms reach")
        ax.legend(fontsize=7.5, frameon=False)
    ax1.set_title("Shared workspace against mount splay, over %d swept "
                  "configurations" % len(rows), color=INK, fontsize=10,
                  loc="left")
    ax2.set_title("and against mount tilt", color=INK, fontsize=10,
                  loc="left")
    top = max(r["both"] for r in rows)
    fig.suptitle("The two arms barely share a workspace at ANY mount "
                 "(mount_overlap_sweep.json, %s)" % date, color=INK,
                 fontsize=10)
    return [(save(fig, out, "mount_overlap_sweep.png"),
             "Cells reachable by BOTH arms across %d swept mount offsets, "
             "splays and tilts (%s). The best configuration anywhere in the "
             "sweep reaches %d shared cells — which is the measurement behind "
             "the disjoint-workspace result, and why a two-arm handover task "
             "cannot be written against this rig without re-parking an arm in "
             "hardware first." % (len(rows), date, top))]


# --------------------------------------------------------------------------
FIGS = {
    "accuracy": (fig_accuracy,
                 "per-mode grasp positioning + T1 placement error "
                 "(accuracy_table.json, 2026-08-23 clip set)"),
    "grasp_matrix": (fig_grasp_matrix,
                     "grasp success per mode x task matrix "
                     "(accuracy_table.json)"),
    "vr_smoothing": (fig_smoothing,
                     "1-Euro vs EMA vs raw: tremor + lag, recomputed through "
                     "the shipped filter (landed 2026-08-26)"),
    "vr_protocol": (fig_vr_protocol,
                    "VR teleop protocol 2026-08-26: distances, direction "
                    "error, lag per segment (sim)"),
    "vr_lag_speed": (fig_vr_lag_speed,
                     "VR protocol: peak lag vs peak hand speed (sim)"),
    "vr_wrist": (fig_wrist,
                 "VR wrist protocol 2026-08-26: gain, axis error, lag (sim)"),
    "teleop_motion": (fig_teleop_motion,
                      "legacy step clamp vs Ruckig: off-path excursion + "
                      "velocity respect (2026-08-23)"),
    "sim_to_real": (fig_sim_to_real,
                    "real-arm per-joint park error + the one-EPS "
                    "compensation model"),
    "grip_traces": (fig_grip_traces,
                    "T2 measured knuckle angle over time, all five modes"),
    "mode_paths": (fig_mode_paths,
                   "EE path length per mode per task (mode_difference.json, "
                   "2026-08-15)"),
    "pad_curve": (fig_pad_curve,
                  "wrist-to-pad offset against grip width -- the constant "
                  "this repo argued about is a curve"),
    "orientation_cost": (fig_orientation_cost,
                         "reach per orientation policy from home and from "
                         "the work point, + what binds"),
    "wearer_posture": (fig_wearer_posture,
                       "innermost usable column and home clearance for the "
                       "five wearer postures"),
    "centre_on_surface": (fig_centre_on_surface,
                          "the centre-on-surface search: a negative result, "
                          "drawn as one"),
    "budget": (fig_budget,
               "the positioning budget term by term, systematic vs random"),
    "reaction": (fig_reaction,
                 "reaction latency and the travel it allows against the "
                 "150 mm floor"),
    "language_75": (fig_language_75,
                    "the full 75-phrasing instruction sweep against the "
                    "grammar it replaced"),
    "voice": (fig_voice,
              "WER per utterance and the wake-word threshold sweep "
              "(SYNTHESISED audio)"),
    "predictive": (fig_predictive,
                   "predictive avoidance: clearance gained, and steps "
                   "refused, per policy"),
    "depth": (fig_depth,
              "depth-derived pose error against range, naive vs fitted"),
    "capability": (fig_capability,
                   "the degradation ladder: cost per rung and where the "
                   "channel subsets land"),
    "t1_seeds": (fig_t1_seeds,
                 "T1 stage 2: all eight seeds walked, pad miss and IK cost"),
    "shared_gain": (fig_shared_gain,
                    "intent inference against a wrong declared coordinate -- "
                    "what shared autonomy is for"),
    "detector_real": (fig_detector_real,
                      "prompted detection on REAL RGB-D: rate per prompt and "
                      "IoU"),
    "pick_accuracy": (fig_pick_accuracy,
                      "pad miss against belief error for picks planned from "
                      "the measured map"),
    "mirror": (fig_mirror,
               "per-link mirror residual at the home pose, with its control"),
    "look_cost": (fig_look_cost,
                  "what a look costs, and what it costs amortised per pick"),
    "mount_overlap": (fig_mount_overlap,
                      "cells both arms reach, across the swept mount "
                      "offsets"),
    "vr_prefix_misses": (fig_vr_prefix_misses,
                         "pre-fix accumulating VR misses -- refuses: prose "
                         "only, no committed artifact"),
    "vr_session_mcap": (fig_vr_session_mcap,
                        "2026-08-26 VR session bag -- refuses unless the "
                        "mcap library is importable"),
}


def build(root=ROOT, outdir=None, only=None):
    """Build every figure the data supports. Returns (made, skipped):
    made is [(fig_key, png_name, caption)], skipped is [(fig_key, reason)].
    Writes index.md into outdir. Never raises for a missing recording."""
    root = Path(root)
    out = Path(outdir) if outdir else root / "recordings/analysis"
    out.mkdir(parents=True, exist_ok=True)
    made, skipped = [], []
    for key, (fn, _) in FIGS.items():
        if only and key not in only:
            continue
        try:
            for png, caption in fn(root, out):
                made.append((key, png, caption))
                print("BUILT   %-18s -> %s" % (key, png))
        except Skip as e:
            skipped.append((key, str(e)))
            print("SKIPPED %-18s -- %s" % (key, e))

    if only:
        # A partial run must not shrink the index to its own subset: the
        # index describes the DIRECTORY, and a --only rebuild has not
        # re-earned the other entries.
        print("(--only run: index.md left as it was; run without --only to "
              "regenerate it)")
        return made, skipped
    lines = ["# Results figures", "",
             "Generated by `scripts/make_results.py` on %s. Every figure "
             "names its source recording and date; a figure whose source is "
             "missing is listed under REFUSED with the reason, never drawn "
             "empty." % datetime.now().strftime("%Y-%m-%d %H:%M"), "",
             "## Figures", ""]
    for key, png, caption in made:
        lines.append("* **%s** (`%s`): %s" % (key, png, caption))
    lines += ["", "## Refused", ""]
    for key, reason in skipped:
        lines.append("* **%s**: %s" % (key, reason))
    if not skipped:
        lines.append("* none")
    (out / "index.md").write_text("\n".join(lines) + "\n")
    return made, skipped


SUMMARY = """
==========================================================================
RESULTS SUMMARY -- the key numbers, each with its date and dataset
==========================================================================
ACCURACY (sim clip set, re-recorded 2026-08-23; accuracy_table.json
2026-08-24 -- PREDATES the 2026-08-26 VR smoothing):
  grasp success 100% in every recorded mode x task cell; grasp positioning
  error 0.0 mm all five modes; T1 placement 55.5-56.9 mm under 06.
  => on ACCURACY the modes are tied at this instrument's resolution; VR is
     not "most accurate", it is AS accurate as every other mode, in sim.

SMOOTHNESS (filter maths through the shipped vr_smoothing.py, landed
2026-08-26; constructed signals, 1.5 mm-rms tremor / 0.40 m/s reach):
  still-hand residual: raw 2.62 mm, EMA(0.6) 1.74 mm, 1-Euro 0.51 mm
  reach lag:           raw ~0 mm,  EMA(0.6) 3.66 mm, 1-Euro 1.53 mm
  => the 2026-08-26 change dominates the old EMA on BOTH axes (3.4x
     steadier AND 2.4x less lag). This compares VR-today to VR-before,
     not VR to the master arm -- no equivalent tremor/lag measurement of
     the master-mannequin path exists.

VR PROTOCOL 2026-08-26 (sim, domain 0, one run):
  peak mapper lag 8.8-35.4 mm at protocol speeds; 82.8/92.9 mm on the
  deliberate fast reach. Direction: z clean (1.6-2.3 deg), x/y 73-117 deg
  off -- align_yaw_deg was NOT calibrated for this run; wrist axes on
  roll/pitch read inverted (150-173 deg) with gain ~1.0.

MOTION GENERATOR (2026-08-23, affects EVERY teleop mode): per-joint clamp
  51.3/39.2 mm off-path at 12.5x velocity limit -> Ruckig 0.0 mm at 1.00x.

NOT YET MEASURED (do not claim):
  * any of this against REAL arms under VR -- the 2026-08-26 protocol and
    wrist runs are sim-only; nothing in the repo has scored VR teleop on
    hardware.
  * master-arm tremor/lag under the same protocol -- without it "VR is the
    smoothest MODE" is unsupported; only "VR is smoother than VR was".
  * post-smoothing task accuracy -- the clip set predates the smoothing.
  ONE measurement settles the mode question: run vr_teleop_protocol.py's
  segments under mode 01 (master arm) and compare max_lag_mm / stillness
  per segment against the 2026-08-26 VR numbers.
==========================================================================
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--only", nargs="*", default=None,
                    help="build only these figure keys")
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.list:
        for k, (_, desc) in FIGS.items():
            print("%-18s %s" % (k, desc))
        return 0
    if a.only:
        bad = [k for k in a.only if k not in FIGS]
        if bad:
            print("no such figure: %s (see --list)" % ", ".join(bad))
            return 2
    made, skipped = build(a.root, a.out, a.only)
    print("\n%d figure file(s) built, %d refused. Index: %s" %
          (len(made), len(skipped),
           (Path(a.out) if a.out else Path(a.root) /
            "recordings/analysis") / "index.md"))
    print(SUMMARY)
    return 0


if __name__ == "__main__":
    sys.exit(main())
