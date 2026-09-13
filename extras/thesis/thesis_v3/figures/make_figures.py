"""Regenerate the data figures in this thesis from the project's own records.

Run from the repository root:

    .venv_vision/bin/python extras/thesis/thesis_v2/figures/make_figures.py

Every figure is produced from a stored measurement file under
recordings/baselines/, named in the caption of the figure it feeds.
"""
import json, os, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 4))
BASE = os.path.join(ROOT, "recordings", "baselines")
OUT = os.path.join(ROOT, "thesis_v2", "figures")

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
    "figure.dpi": 150, "savefig.bbox": "tight", "axes.spines.top": False,
    "axes.spines.right": False,
})
GREY, BLUE, RED = "0.35", "#2c6fbb", "#c1392b"


def channels():
    d = json.load(open(os.path.join(BASE, "channels_20260806.json")))
    names = list(d)
    drop = [d[n]["drop_pct"] for n in names]
    jump = []
    for n in names:
        m = re.search(r"(\d+(?:\.\d+)?)%", d[n]["why"])
        jump.append(float(m.group(1)) if m and "jump" in d[n]["why"] else 0.0)
    verdict = [d[n]["verdict"] for n in names]
    colour = {"ALIVE": "#4c9a4c", "INTERMITTENT": "#d9a441",
              "INCOHERENT": "#c1392b", "DEAD": "0.25"}

    fig, ax = plt.subplots(2, 1, figsize=(7.2, 4.4), sharex=True)
    x = np.arange(len(names))
    ax[0].bar(x, jump, color=[colour[v] for v in verdict])
    ax[0].axhline(5, ls="--", c=GREY, lw=1)
    ax[0].text(len(names) - 0.4, 5.4, "5 % threshold", ha="right",
               fontsize=7, color=GREY)
    ax[0].set_ylabel("updates jumping\nover 60 degrees (%)")
    ax[0].set_title("Master arm channel health, 6 August 2026")

    ax[1].bar(x, drop, color=[colour[v] for v in verdict])
    ax[1].axhline(2, ls="--", c=GREY, lw=1)
    ax[1].text(len(names) - 0.4, 3.5, "2 % threshold", ha="right",
               fontsize=7, color=GREY)
    ax[1].set_ylabel("readings lost (%)")
    ax[1].set_xticks(x)
    ax[1].set_xticklabels([n.replace("_", " ") for n in names], rotation=45,
                          ha="right", fontsize=7)

    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in colour.values()]
    ax[0].legend(handles, [k.lower() for k in colour], fontsize=7, ncol=4,
                 frameon=False, loc="upper left")
    fig.savefig(os.path.join(OUT, "channels.pdf"))
    plt.close(fig)


def reach():
    d = json.load(open(os.path.join(BASE, "workspace_n10_20260806.json")))
    label = {"front": "forward", "back": "back", "left": "to the left",
             "right": "to the right", "up": "up", "down": "down"}
    keys = list(label)
    fig, ax = plt.subplots(figsize=(7.0, 3.0))
    w = 0.38
    x = np.arange(len(keys))
    for i, (arm, col) in enumerate([("left", BLUE), ("right", RED)]):
        vals = [d[arm][k]["median"] for k in keys]
        ax.bar(x + (i - 0.5) * w, vals, w, label=arm + " arm", color=col,
               alpha=0.85)
        for xx, v, k in zip(x + (i - 0.5) * w, vals, keys):
            ax.text(xx, 0.02, d[arm][k]["reason"].split(" (")[0],
                    ha="center", va="bottom", fontsize=6, rotation=90,
                    color="white")
    ax.set_xticks(x)
    ax.set_xticklabels([label[k] for k in keys])
    ax.set_ylabel("reach from rest (m)")
    ax.set_ylim(0, 0.80)
    ax.set_title("How far the hand travels before something stops it "
                 "(median of ten sweeps)", pad=8)
    ax.legend(frameon=False, fontsize=8)
    fig.savefig(os.path.join(OUT, "reach_directions.pdf"))
    plt.close(fig)


def clearance():
    d = json.load(open(os.path.join(BASE, "home_change_applied.json")))
    cells = d["cells"]["stored_home/stored_anchor"]["tasks"]
    order = [("t0", "reaching"), ("t1", "pick and place"),
             ("t1s2", "pick and place,\nrandomised"), ("t2", "two-arm carry"),
             ("t3", "tool handling")]
    fig, ax = plt.subplots(figsize=(7.0, 3.1))
    x = np.arange(len(order))
    w = 0.38
    for i, (arm, col) in enumerate([("left", BLUE), ("right", RED)]):
        vals = [cells[k][arm]["worst_clearance_m"] for k, _ in order]
        bars = ax.bar(x + (i - 0.5) * w, vals, w, label=arm + " arm",
                      color=col, alpha=0.85)
        for b, v, (k, _) in zip(bars, vals, order):
            part = cells[k][arm].get("to", "")
            ax.text(b.get_x() + b.get_width() / 2, v + 0.004,
                    f"{v*1000:.0f}", ha="center", fontsize=7)
            ax.text(b.get_x() + b.get_width() / 2, 0.168, part, ha="center",
                    fontsize=5.5, rotation=90, color=GREY, va="bottom")
    ax.axhline(0.15, ls="--", c=RED, lw=1.2)
    ax.text(-0.45, 0.153, "150 mm floor", ha="left", fontsize=8, color=RED)
    ax.set_xticks(x)
    ax.set_xticklabels([n for _, n in order], fontsize=8)
    ax.set_ylabel("closest approach to the wearer (m)")
    ax.set_ylim(0, 0.235)
    ax.set_title("How close each task brings the arm to the person carrying it",
                 pad=14)
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    fig.savefig(os.path.join(OUT, "clearance_tasks.pdf"))
    plt.close(fig)


if __name__ == "__main__":
    channels()
    reach()
    clearance()
    print("wrote channels.pdf, reach_directions.pdf, clearance_tasks.pdf")
