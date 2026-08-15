#!/usr/bin/env python3
"""THE FOUR GRASP CRITERIA, MEASURED FROM PIXELS.

    python3 scripts/verify_grasp_quality.py [--task t2]

A clip failing any of these is not a grasp clip:

  1 FINGERS   do they open, close to the object's width, hold, and reopen?
  2 WRIST     does the wrist orientation CHANGE to suit the object, or stay
              fixed? Measured as the angle of the gripper's finger-opening
              axis in the close-up view.
  3 APPROACH  is there a straight-line approach along a vector, or does the
              hand arrive from wherever it happened to be? Measured as the
              straightness of the gripper centroid path over the approach.
  4 ATTACH    does the object appear on the hand at the moment the fingers
              reach the object's width, or on proximity alone?

Measured from the recorded video, not from the code that produced it.
"""
import argparse, glob, json, os, subprocess, sys
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/verification")
FF = os.path.expanduser("~/.local/bin/ffmpeg")
TMP = "/tmp/claude-1000/-home-gausms-kortex-ws/3732aa29-5a7e-4c8e-b77e-379233bdc9c9/scratchpad/gq"
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_rviz_clips import duration, PALETTE, below_hud

def frames(mp4, n=16):
    os.makedirs(TMP, exist_ok=True)
    for f in glob.glob(TMP + "/*.png"):
        os.remove(f)
    d = duration(mp4)
    if d <= 0:
        return [], []
    ts = list(np.linspace(0.05, 0.97, n) * d)
    out = []
    for k, t in enumerate(ts):
        p = os.path.join(TMP, "f%02d.png" % k)
        subprocess.run([FF, "-y", "-loglevel", "error", "-ss", "%.2f" % t,
                        "-i", mp4, "-frames:v", "1", "-vf", "scale=640:-1", p],
                       check=False)
        out.append(np.asarray(Image.open(p).convert("RGB"), float)
                   if os.path.exists(p) else None)
    return out, ts

def dark(img):
    """Gripper body mask -- background renders at (44,44,46), so <28."""
    return (img.max(axis=2) < 28)

def silhouette(img):
    h = img.shape[0]
    return float(dark(img[int(0.25*h):int(0.80*h)]).sum())

def axis_angle(img):
    """Principal-axis angle of the gripper blob, degrees. Tracks wrist yaw."""
    h = img.shape[0]
    m = dark(img[int(0.25*h):int(0.80*h)])
    ys, xs = np.nonzero(m)
    if xs.size < 200:
        return None
    x = xs - xs.mean(); y = ys - ys.mean()
    cov = np.cov(np.vstack([x, y]))
    w, v = np.linalg.eigh(cov)
    vx, vy = v[:, -1]
    return float(np.degrees(np.arctan2(vy, vx)) % 180.0)

def centroid(img):
    h = img.shape[0]
    m = dark(img[int(0.20*h):int(0.85*h)])
    ys, xs = np.nonzero(m)
    if xs.size < 200:
        return None
    return (float(xs.mean()), float(ys.mean()))

def straightness(pts):
    """max perpendicular deviation from the chord, in px. Small = straight."""
    P = np.asarray([p for p in pts if p is not None], float)
    if len(P) < 4:
        return None
    a, b = P[0], P[-1]
    d = b - a
    n = np.linalg.norm(d)
    if n < 8:
        return None
    d = d / n
    dev = [abs(np.cross(d, p - a)) for p in P]
    return float(max(dev))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--task", default=None)
    a = ap.parse_args()
    rows = []
    # ONE MORE PATH LEVEL. The tree is now
    # recordings/verification/<mode>/<task>/<scenario>/<condition>, because
    # control mode and autonomy condition are independent axes and the old layout
    # conflated them: "direct" under the mannequin and "direct" under VR shared a
    # folder name. A glob that still assumes three levels matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass.
    _traces = sorted((sorted(glob.glob(os.path.join(OUT, "*/*/*/*/grip_trace.json")))
         # THREE levels is the MSc sweep, FOUR is the legacy A/B/C
         # recorder. Accept both: the tree carries both sets and a
         # glob that knows only one of them reports zero for the
         # other, which reads exactly like a clean pass.
         + sorted(glob.glob(os.path.join(OUT, "*/*/*/grip_trace.json")))))
    # ZERO INPUTS IS NOT A PASS, AND THE COMMENT ABOVE SAYS SO ITSELF.
    #
    # It warns that a glob assuming the wrong depth "matches NOTHING and
    # reports zero clips, which reads exactly like a clean pass" -- and then
    # the code below did exactly that and returned 0. MEASURED 2026-08-15:
    # this ran in the sweep's VERIFYING step over 25 recorded cells, printed
    # "0 of 0 clips show a GENUINE grasp" and exited 0.
    #
    # Two causes, both real: record_abc_sweep writes THREE path levels, not
    # four; and NOTHING in the clip path writes grip_trace.json at all -- 0
    # files on disk. Knowing the failure mode and still returning 0 for it is
    # the gap this refusal closes.
    if not _traces:
        print("\nREFUSING TO REPORT A PASS: this checked ZERO clips.")
        print("  looked for : */*/*/*/grip_trace.json under %s" % OUT)
        print("  found      : 0")
        print("  Nothing in the clip path writes grip_trace.json, and the")
        print("  sweep writes <mode>/<task>/<scenario>, not four levels.")
        print("  Fix the producer or the glob. 0 of 0 is not evidence.")
        return 2
    for tp in _traces:
        d = os.path.dirname(tp)
        mode, task, scen, cond = os.path.relpath(d, OUT).split(os.sep)[:4]
        if a.task and task != a.task:
            continue
        gv = os.path.join(d, "rviz_gripper.mp4")
        if not os.path.exists(gv):
            continue
        cap = json.load(open(os.path.join(d, "rviz_capture.json"))) \
            if os.path.exists(os.path.join(d, "rviz_capture.json")) else {}
        tr = json.load(open(tp))
        arm = max(("left", "right"),
                  key=lambda x: (max([v[x] for v in tr if v.get(x) is not None] or [0])
                                 - min([v[x] for v in tr if v.get(x) is not None] or [0])))
        kn = [v[arm] for v in tr if v.get(arm) is not None]
        F, ts = frames(gv, 16)
        F = [f for f in F if f is not None]
        if len(F) < 8:
            continue
        sil = [silhouette(f) for f in F]
        cen = [centroid(f) for f in F]
        # WRIST ANGLE MUST COME FROM A WORLD-FIXED VIEW. The gripper camera
        # TRACKS the gripper link, so in its own frame the hand is nearly
        # stationary by construction and its measured axis barely moves --
        # it reported 2.9 deg on a grasp whose wrist was correctly aligned.
        fv = os.path.join(d, "rviz_front.mp4")
        Ff, _ = frames(fv, 16) if os.path.exists(fv) else ([], [])
        ang = [axis_angle(f) for f in Ff if f is not None]

        # 1 FINGERS: silhouette must vary appreciably
        c1 = (max(sil) - min(sil)) / max(sil) > 0.15 if max(sil) else False
        # 2 WRIST: principal axis must change by more than measurement noise
        av = [x for x in ang if x is not None]
        wrist_range = (max(av) - min(av)) if len(av) > 3 else 0.0
        wrist_range = min(wrist_range, 180 - wrist_range) if wrist_range > 90 else wrist_range
        c2 = wrist_range > 8.0
        # 3 APPROACH: straight-line segment somewhere in the clip
        best = None
        for i in range(0, len(cen) - 4):
            s_ = straightness(cen[i:i+5])
            if s_ is not None and (best is None or s_ < best):
                best = s_
        c3 = best is not None and best < 12.0
        # 4 ATTACH: fingers reached the commanded width
        c4 = bool(cap.get("grasp_planned")) and any(
            0.10 <= v < 0.74 for v in kn)
        rows.append(dict(task=task, scenario=scen, condition=cond, arm=arm,
                         fingers=c1, wrist=c2, approach=c3, attach=c4,
                         sil_change=round((max(sil)-min(sil))/max(sil), 3) if max(sil) else 0,
                         wrist_deg=round(wrist_range, 1),
                         straight_px=round(best, 1) if best else None,
                         genuine=bool(c1 and c2 and c3 and c4),
                         refused=cap.get("grasp_refused", "")))
        r = rows[-1]
        print("  %-3s %-20s %-9s fingers=%-5s wrist=%-5s(%4.1f d) approach=%-5s(%s px) attach=%-5s  %s"
              % (task, scen, cond, c1, c2, r["wrist_deg"], c3,
                 r["straight_px"], c4,
                 "GENUINE" if r["genuine"] else "not a grasp clip"))
    g = [r for r in rows if r["genuine"]]
    print("\n  %d of %d clips show a GENUINE grasp by all four criteria"
          % (len(g), len(rows)))
    json.dump(rows, open(os.path.join(OUT, "grasp_quality.json"), "w"), indent=2)
    return 0

if __name__ == "__main__":
    sys.exit(main())
