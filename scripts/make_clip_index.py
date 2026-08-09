#!/usr/bin/env python3
"""Build recordings/verification/INDEX.md -- every clip, and what it SHOULD show.

    python3 scripts/make_clip_index.py

Scans the per-run summary.json / rviz_capture.json files rather than any
index.json: the sweeps run one task per invocation and each invocation
rewrites the index, so that file only ever holds the last task. Scanning the
tree is the only complete view.

Two recordings exist per run and they answer different questions:

  rviz.mp4   REAL SCREEN CAPTURE of RViz -- what you would see sitting in
             front of the machine, with the objects visible and moving.
             This is the one to watch.
  clip.mp4   a TF-rendered 3-D + front-view plot. Ugly, but every number in
             summary.json comes from the same samples that drew it, so it is
             the one that can be checked automatically.

The "what it should show" text is written from the protocol, not from the
recording -- otherwise it would just describe whatever happened.
"""
import glob
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")
WINPATH = r"\\wsl.localhost\Ubuntu\home\gausms\kortex_ws\recordings\verification"

EXPECT = {
    "t2": ("The FILL arm picks an orange block off the slab, carries it "
           "across, and drops it into the TEAL container that the HOLD arm is "
           "carrying. Blocks left in the container turn GREEN. The hold arm "
           "holds station throughout; the two arms never swap sides."),
    "t3": ("Both grippers rise together with the TAN RIGID TRAY between them "
           "and a yellow ball on top. The tray stays straight and level -- "
           "|tilt| in the overlay should stay near 0 deg. The ball rolls to "
           "the low side and would fall off past 11.3 deg."),
    "t5": ("The RIGHT arm (screen right) picks the tool off its cradle, "
           "carries it inboard, and stops with it at the GREEN SPHERE -- the "
           "receive point at the wearer's side. The left arm never moves."),
    "t6": ("Same paths as T3, but the object is a SLING: an orange catenary "
           "between the grippers with the ball sitting in the bottom of the "
           "V. The overlay shows live separation and sag; the ball turns RED "
           "and drops if sag falls below one ball diameter (40 mm)."),
    "t7": ("Each arm chases its own GREEN TARGET SPHERE around a small closed "
           "loop -- a Lissajous on an 80 mm sphere. The arms are INDEPENDENT. "
           "In the B1/B2 baselines only ONE arm moves."),
    "t8": ("The arm reaches out to a GREEN TARGET far on its own side, "
           "labelled with the stance the wearer had to adopt. The clip shows "
           "the reach AFTER the wearer has repositioned. The other arm is "
           "parked at home."),
    "t9": ("The arm traces a circle whose radius is the sway amplitude -- it "
           "is HOLDING a world-fixed point while the base moves underneath, "
           "so in the arm's own frame the target orbits. The radius should "
           "visibly grow across S1->S4 (0, 20, 60, 100 mm)."),
}
COND = {"direct": "no smoothing -- the raw commanded path",
        "assisted": "30% smoothing, standing in for assistance",
        "shared": "55% smoothing, the most machine-shaped motion"}

# The five to watch first: one per interesting behaviour, not one per task.
TOP5 = [
    ("t6", "S4_tight", "direct",
     "THE SLING. Live separation and sag in the overlay; the ball sits in the "
     "V. S4 is the tight 330 mm case, 10 mm from the documented failure "
     "threshold, so it is the one where the object nearly fails."),
    ("t2", "S1_short_reach", "direct",
     "PICK AND PLACE. A block leaves the slab, travels with the gripper and "
     "ends up inside the container. This is the clip that proves the gripper "
     "is not closing on nothing."),
    ("t5", "S1_near", "direct",
     "THE HANDOVER -- the canonical Fusion scenario. Tool off the cradle, "
     "carried inboard, delivered at the wearer's side."),
    ("t3", "S2_long_height", "direct",
     "THE RIGID CARRY. Tray straight, tilt near zero through a full-band "
     "lift. Compare directly against the T6 clip above: same path, different "
     "object."),
    ("t9", "S4_sway_100mm", "direct",
     "WEARER MOTION at the top of the IV. The arm holds a world-fixed point "
     "while the base sways 100 mm underneath it."),
]


def main():
    rows = []
    # ONE MORE PATH LEVEL. The tree is now
    # recordings/verification/<mode>/<task>/<scenario>/<condition>, because
    # control mode and autonomy condition are independent axes and the old layout
    # conflated them: "direct" under the mannequin and "direct" under VR shared a
    # folder name. A glob that still assumes three levels matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass.
    # BOTH RECORD TYPES. summary.json comes from the TF-render pipeline and
    # rviz_capture.json from the screen recorder, and the five-task clips have
    # only the latter. Globbing summary.json alone silently dropped all 38 of
    # them and rewrote this index with the 87 RETIRED clips, discarding the
    # Task 2 caveat that has to sit at the top. A missing clip type reads as a
    # smaller index, not as an error.
    records = sorted(glob.glob(os.path.join(OUT, "*/*/*/*/summary.json"))) + \
        sorted(glob.glob(os.path.join(OUT, "*/*/*/*/rviz_capture.json")))
    seen_dirs = set()
    for p in records:
        if os.path.dirname(p) in seen_dirs:
            continue
        seen_dirs.add(os.path.dirname(p))
        s = json.load(open(p))
        d = os.path.dirname(p)
        s["dir"] = os.path.relpath(d, ROOT)
        s["has_clip"] = os.path.exists(os.path.join(d, "clip.mp4"))
        s["has_plot"] = os.path.exists(os.path.join(d, "plot_metrics.png"))
        s["has_bag"] = os.path.isdir(os.path.join(d, "bag"))
        rv = os.path.join(d, "rviz_capture.json")
        s["views"] = [v for v in ("front", "side", "top", "gripper")
                      if os.path.exists(os.path.join(d, "rviz_%s.mp4" % v))]
        s["has_quad"] = os.path.exists(os.path.join(d, "rviz_quad.mp4"))
        s["has_rviz"] = s["has_quad"]
        s["rviz"] = json.load(open(rv)) if os.path.exists(rv) else {}
        # NORMALISE ONCE, HERE. Two record types feed this index --
        # summary.json from the TF-render pipeline and rviz_capture.json from
        # the screen recorder -- and they carry different fields. Patching the
        # consumers field by field is how the previous four attempts each
        # fixed one KeyError and hit the next. Every numeric field a formatter
        # will divide, compare or print gets a real default here, and the flag
        # says which type the row is so a caller can tell "absent" from "zero".
        DEFAULTS = dict(min_clearance_m=0.0, travel_m=0.0,
                        duration_s=0.0, frames=0, bytes=0,
                        passes_through_wearer=False, below_real_floor=False,
                        below_floor=False, no_motion=False,
                        rviz=False, clip=False)
        s = dict(DEFAULTS, **{k: v for k, v in s.items() if v is not None})
        s["record_type"] = ("screen_capture" if p.endswith("rviz_capture.json")
                            else "tf_render")
        rows.append(s)
    # attach the two verifiers' verdicts so the index states what was CHECKED,
    # not merely what was recorded
    def _load(name):
        p = os.path.join(OUT, name)
        if not os.path.exists(p):
            return {}
        return {(r.get("task"), r.get("scenario"), r.get("condition")): r
                for r in json.load(open(p))}
    att = _load("attachment_check.json")
    con = _load("rviz_verification.json")
    grip = _load("gripper_check.json")
    for r in rows:
        k = (r.get("task"), r.get("scenario"), r.get("condition"))
        r["attach"] = att.get(k, {})
        r["content"] = con.get(k, {})
        r["grip"] = grip.get(k, {})
    tasks = sorted({r.get("task") for r in rows})

    def find(t, sc, cd):
        for r in rows:
            if (r.get("task"), r.get("scenario"), r.get("condition")) == (t, sc, cd):
                return r
        return None

    L = []
    L.append("# VERIFICATION CLIP INDEX\n")
    # THE CAVEAT IS GENERATED, NOT HAND-ADDED. It was written by hand once and
    # this generator silently overwrote it on the next run, which is how a
    # warning that must never be lost gets lost.
    L.append("""> ## READ FIRST: TASK 2 IS NOT OPERATOR-COMMANDABLE
>
> **No f2 clip shows teleoperated grasping, because teleoperated grasping is
> not achievable on this platform.** A top-down grasp needs **169.7 deg**
> (left) and **164.6 deg** (right) of wrist rotation from the orientation
> `orientation_mode: fixed` pins the commanded wrist to, and nothing in the
> master measures the wrist to command it. Those clips work because the
> recorder calls `/compute_ik` **directly**, bypassing the teleoperation
> orientation lock. They show that the ROBOT can execute an aligned grasp, not
> that an OPERATOR can command one. See
> `docs/research/09_task2_grasping_finding.md`.
>
> ## AND: NO CLIP HERE WAS RECORDED THROUGH A CONTROL MODE
>
> The tree is `<mode>/<task>/<scenario>/<condition>`, but the six mode
> directories are **empty**. Every clip was produced by `record_rviz.py`,
> which calls `/compute_ik` directly and never publishes
> `/master_arm_pose_*`, so no follower, clutch, anchor or orientation lock is
> in the path. They live under two honestly-named buckets instead:
> `00_unclassified_legacy_geometry` (the retired nine-task clips) and
> `00_unclassified_scripted_playback` (the current five-task clips).
""")
    L.append("**%d runs**, every task x scenario x autonomy condition, driven "
             "in sim and recorded two ways.\n" % len(rows))

    L.append("""
## START HERE -- how to play them from Windows

The files live in the WSL filesystem. From Windows, paste this into Explorer
or into a media player's Open dialog:

```
%s
```

VLC, MPC-HC and the built-in Films & TV app all open that path directly. Or
from PowerShell:

```powershell
start %s\\t6\\S4_tight\\direct\\rviz.mp4
```

If `\\\\wsl.localhost` does not resolve, the older form `\\\\wsl$\\Ubuntu\\...`
works on the same machine.

## The two recordings, and which to watch

| file | what it is |
| --- | --- |
| **`rviz_quad.mp4`** | **THE ONE TO WATCH.** All four angles tiled 2x2: front (top-left), side (top-right), top (bottom-left), gripper close-up (bottom-right). |
| `rviz_front.mp4` | facing the wearer, both arms. Carries the overlay -- the other three do not, deliberately |
| `rviz_side.mp4` | reach depth and height, which the front view flattens |
| `rviz_top.mp4` | lateral separation and the arms' relationship -- where a coordination error is obvious |
| `rviz_gripper.mp4` | tracks the ACTIVE gripper link close up. This is the view that answers "did it actually grab it" |
| `clip.mp4` | a TF-rendered 3-D + front-view plot. Uglier, but drawn from exactly the samples that produced `summary.json`, so the numbers and the picture cannot disagree. Kept for automated checking. |
| `plot_metrics.png` | tracking error and clearance against time |
| `summary.json` | the metrics, and the automatic pass/fail checks |
| `bag/` | rosbag2 of /tf, /joint_states and the commands (regenerable; not in git) |

Every `rviz.mp4` carries a burnt-in overlay: task, scenario, condition,
elapsed time, phase, and the live task metric (tilt for T3, separation and sag
for T6, tracking error elsewhere). The overlay is drawn as scene text inside
RViz rather than composited afterwards, so the number on screen is the number
from that frame.

The first ~1.5 s of every clip is the approach from home, tagged
`approach` in the overlay. It is included because it is the largest and most
collision-relevant motion, and it is EXCLUDED from the tracking metric.
""" % (WINPATH, WINPATH))

    L.append("\n## THE FIVE TO WATCH FIRST\n")
    for i, (t, sc, cd, why) in enumerate(TOP5, 1):
        r = find(t, sc, cd)
        if not r:
            continue
        rv = r.get("rviz", {})
        L.append("**%d. %s / %s / %s** — %.1f s\n"
                 % (i, t.upper(), sc, cd, rv.get("duration_s", 0.0)))
        L.append("   %s\n" % why)
        L.append("   `%s\\%s\\%s\\%s\\rviz_quad.mp4`\n"
                 % (WINPATH, t, sc, cd))

    through = [r for r in rows if r.get("passes_through_wearer")]
    low = [r for r in rows if r.get("below_real_floor")]
    still = [r for r in rows if r.get("ee_travel_m") is not None
             and r.get("ee_travel_m") < 0.05]
    norv = [r for r in rows if not r.get("has_rviz")]
    L.append("\n## Automatic checks across all %d runs\n" % len(rows))
    L.append("| check | result |")
    L.append("| --- | --- |")
    L.append("| any arm link inside the wearer | **%d** |" % len(through))
    L.append("| below the 120 mm `real_robot` clearance floor | **%d of %d** "
             "(see the clearance finding in docs/research) |"
             % (len(low), len(rows)))
    L.append("| shows no motion (EE travel < 50 mm) | **%d** |" % len(still))
    L.append("| RViz screen capture present | %d of %d |"
             % (len(rows) - len(norv), len(rows)))
    L.append("| TF clip / plot / bag present | %d / %d / %d |"
             % (sum(r.get("has_clip") for r in rows),
                sum(r.get("has_plot") for r in rows),
                sum(r.get("has_bag") for r in rows)))

    for t in tasks:
        rs = [r for r in rows if r.get("task") == t]
        L.append("\n---\n\n## %s — %d runs\n" % (t.upper(), len(rs)))
        L.append("**What it should show.** %s\n" % EXPECT.get(t, "—"))
        L.append("| scenario | cond | rviz.mp4 | duration | EE travel | "
                 "min clearance | gripper | object outcome | attachment |")
        L.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for r in sorted(rs, key=lambda r: (r.get("scenario"), r.get("condition"))):
            rv = r.get("rviz", {})
            out = ""
            if t == "t2":
                out = "%d block(s) placed" % rv.get("blocks_placed", 0)
            elif t in ("t3", "t6"):
                out = "ball FELL" if rv.get("ball_fallen") else "ball retained"
            elif t == "t5":
                out = "tool delivered" if rv.get("tool_delivered") else "-"
            flag = ""
            if r.get("passes_through_wearer"):
                flag = " **THROUGH WEARER**"
            elif r.get("below_real_floor"):
                flag = " *(below floor)*"
            a = r.get("attach", {}).get("verdict", "-")
            a = {"CARRIED": "object CARRIED"}.get(a, a)
            gk = r.get("grip", {})
            gtxt = "-"
            if gk.get("grasp_task"):
                gtxt = ("%.2f..%.2f %s" % (gk.get("grip_min") or 0,
                                           gk.get("grip_max") or 0,
                                           "OK" if gk.get("joint_ok") else "NO"))
            # A field this record type does not measure prints as "--", not
            # as a number. A screen-capture clip carries no end-effector
            # travel, and printing 0.00 m for it would assert the arm did not
            # move.
            trav = ("%.2f m" % r["ee_travel_m"]
                    if r.get("ee_travel_m") is not None else "--")
            clr = ("%.3f m" % r["min_clearance_m"]
                   if r.get("min_clearance_m") is not None else "--")
            L.append("| %s | %s | %s | %.1f s | %s | %s%s | %s | "
                     "%s | %s |"
                     % (r.get("scenario"), r.get("condition"),
                        "%d/4+quad" % len(r.get("views")) if r.get("has_quad")
                        else "**MISSING**",
                        rv.get("duration_s", 0.0), trav, clr, flag, gtxt,
                        out or "-", a))

    L.append("""
---

## Regenerate

```bash
ros2 launch srl_moveit_config demo.launch.py     # sim, arms AT HOME
python3 scripts/record_verification.py --all     # TF clips, plots, bags
python3 scripts/record_rviz.py --all             # RViz screen captures
python3 scripts/make_clip_index.py               # this file
```

`record_rviz.py` starts its own Xvfb on :99 and its own RViz. It has to:
**x11grab on WSLg's :0 records BLACK** -- a full-screen grab with RViz plainly
visible measures mean pixel value 0.0, because XWayland window pixels are
composited by Wayland and never reach the X root window that x11grab reads.
Xvfb has no compositor, and the same grab there gives mean 126.8.
""")
    p = os.path.join(OUT, "INDEX.md")
    open(p, "w").write("\n".join(L) + "\n")
    print("wrote %s" % p)
    print("  %d runs, %d with rviz.mp4, %d with clip.mp4"
          % (len(rows), sum(r.get("has_rviz") for r in rows),
             sum(r.get("has_clip") for r in rows)))
    print("  through wearer %d   below floor %d   no motion %d"
          % (len(through), len(low), len(still)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
