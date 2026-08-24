#!/usr/bin/env python3
"""FILM A WHOLE SESSION: the scan, the pick, the quick check, and the window.

    ./.venv_vision/bin/python scripts/record_session.py

Writes `recordings/session/<stamp>/`

    1_full_scan.mp4     both arms finding the table and mapping it
    2_pick.mp4          picking an object, coming down onto it from above
    3_quick_check.mp4   the fast re-look -- what moved since the reference
    robot_view.png      the robot and the map it measured
    control_window.png  the window an operator actually presses
    narration.txt       every sentence the robot said, timed
    world_map.json      what it decided was on the table
    summary.txt         the numbers, and what this is NOT

WHY ONE SCRIPT AND NOT THREE
----------------------------
Because the interesting claim is the SEQUENCE. A scan on its own says the
perception works; a pick on its own says the arm works; the quick check only
means anything AFTER a scan it can compare against. Filmed separately they
would each be true and the thing that matters -- that a table can be measured
once and then kept up to date in thirty seconds -- would not be shown at all.

WHAT IS REAL IN THESE CLIPS AND WHAT IS NOT
-------------------------------------------
The arm is SIMULATED and the depth is RENDERED. No camera has ever been
attached to this host. What IS real: the sweep order, the deprojection, the
segmentation, the fusion, the plane fit, the IK, the approach-angle search,
and every refusal. `summary.txt` says so in the recording rather than leaving
it in somebody's memory.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from record_calibration import (Narration, brightness, grab,  # noqa: E402
                                start_display, DISP, W, H)

PY = os.path.join(ROOT, ".venv_vision/bin/python")


def step(name, argv, out, nar, film=None, cwd=ROOT):
    """Run one stage, filming it if `film` is given. Returns (rc, lines)."""
    print("### %s" % name, flush=True)
    cap = grab(os.path.join(out, film)) if film else None
    p = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    while p.poll() is None:
        nar.spin(0.3)
    lines = (p.stdout.read() or "").splitlines()
    if cap:
        cap.terminate()
        try:
            cap.wait(timeout=20)
        except Exception:                                      # noqa: BLE001
            cap.kill()
    return p.returncode, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both")
    ap.add_argument("--pick-arm", default="left", choices=("left", "right"))
    # THE SHIPPED SWEEP, NOT A FAST ONE.
    #
    # The first recording was filmed with one layer and one facing to keep it
    # short, and it showed: 5 of 24 cells reachable, 4 views, and one of the
    # four cubes never found. That is a fact about the settings I chose for
    # the film, not about the system, and a recording that flatters or
    # maligns the thing it records is worth nothing. These are the defaults an
    # operator gets.
    ap.add_argument("--step-m", type=float, default=0.13)
    ap.add_argument("--layers-m", type=float, nargs="+", default=[0.30, 0.42])
    ap.add_argument("--facings-deg", type=float, nargs="+",
                    default=[-25.0, 0.0, 25.0])
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-window", action="store_true")
    a = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(ROOT, "recordings/session", stamp)
    os.makedirs(out, exist_ok=True)
    mp = os.path.join(out, "world_map.json")
    print("filming into %s" % out, flush=True)

    start_display()
    nar = Narration()
    nar.start()
    log = []
    t0 = time.time()

    # ---------------------------------------------------------- 1 full scan
    rc1, l1 = step("FULL SCAN -- find the table, then map it",
                   [PY, "-u", os.path.join(HERE, "calibrate_environment.py"),
                    "--arm", a.arm, "--auto-bounds", "--step-m",
                    str(a.step_m), "--layers-m"]
                   + [str(v) for v in a.layers_m] + ["--facings-deg"]
                   + [str(v) for v in a.facings_deg]
                   + ["--reprobe", "--out", mp],
                   out, nar, film="1_full_scan.mp4")
    log += l1

    # ------------------------------------------------- the scene reference
    rc_ref, lref = step("REMEMBER VIEW -- store the scene camera's picture",
                        [PY, "-u", os.path.join(HERE, "relook.py"),
                         "--save-reference"], out, nar)
    log += lref

    # ---------------------------------------------------------------- 2 pick
    rc2 = None
    if rc1 == 0:
        objs = []
        try:
            with open(mp) as f:
                objs = [(i, o) for i, o in enumerate(json.load(f)["objects"])
                        if o.get("graspable")]
        except Exception:                                      # noqa: BLE001
            pass
        if objs:
            side = 1 if a.pick_arm == "left" else -1
            mine = [(i, o) for i, o in objs
                    if side * o["centre"][0] > 0] or objs
            idx = mine[0][0]
            rc2, l2 = step("PICK IT UP -- object %d, down onto it from above"
                           % idx,
                           [PY, "-u", os.path.join(HERE, "pick_from_map.py"),
                            "--arm", a.pick_arm, "--map", mp,
                            "--object", str(idx), "--execute",
                            "--accept-stale-map"],
                           out, nar, film="2_pick.mp4")
            log += l2

    # --------------------------------------------------------- 3 quick check
    rc3, l3 = step("QUICK CHECK -- what has moved since the reference",
                   [PY, "-u", os.path.join(HERE, "relook.py"),
                    "--changed-only", "--map", mp],
                   out, nar, film="3_quick_check.mp4")
    log += l3

    # ------------------------------------------------------------ the window
    # TWO PICTURES, AND THEY ARE NOT THE SAME PICTURE.
    #
    # This grabbed the display RViz is on and called the result `window.png`,
    # which is the robot view -- the arms, the wearer and the measured cloud.
    # A useful picture, and not the control window, and a file named for
    # something it is not is worse than no file. `record_session.py --window`
    # captures the operator's window separately; this one is now named for
    # what it holds.
    shot = None
    if not a.no_window:
        shot = os.path.join(out, "robot_view.png")
        try:
            subprocess.run(
                [os.path.expanduser("~/.local/bin/ffmpeg"), "-y", "-loglevel",
                 "error", "-f", "x11grab", "-video_size", "%dx%d" % (W, H),
                 "-i", DISP, "-frames:v", "1", shot], timeout=60, check=False)
        except Exception:                                      # noqa: BLE001
            shot = None

    n = nar.write(os.path.join(out, "narration.txt"))
    with open(os.path.join(out, "run.log"), "w") as f:
        f.write("\n".join(log))

    bright = {}
    for name in ("1_full_scan.mp4", "2_pick.mp4", "3_quick_check.mp4"):
        pth = os.path.join(out, name)
        if os.path.exists(pth):
            bright[name] = brightness(pth)

    doc = {}
    if os.path.exists(mp):
        with open(mp) as f:
            doc = json.load(f)

    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("a whole session, %s\n\n" % stamp)
        f.write("full scan exit %s, reference exit %s, pick exit %s, "
                "quick check exit %s\n" % (rc1, rc_ref, rc2, rc3))
        f.write("%d narration sentence(s), %.0f s end to end\n\n"
                % (n, time.time() - t0))
        sw = doc.get("sweep", {})
        if sw:
            f.write("arms swept        %s\n" % sw.get("arms"))
            f.write("bounds from       %s\n" % doc.get("bounds_source"))
            tf = doc.get("table_found") or {}
            if tf:
                f.write("table found at    z = %.4f m, tilt %.2f deg\n"
                        % (tf.get("surface_z", 0), tf.get("tilt_deg", 0)))
                f.write("                  x %.3f..%.3f  y %.3f..%.3f\n"
                        % (tf["extent_x"][0], tf["extent_x"][1],
                           tf["extent_y"][0], tf["extent_y"][1]))
            f.write("cells reachable   %s of %s, %s photographed\n"
                    % (sw.get("cells_reachable"), sw.get("cells"),
                       sw.get("views_used")))
            f.write("stillness refusals %s\n" % sw.get("stillness_refusals"))
        f.write("surface measured  %.4f m\n" % doc.get("surface", {}).get("z_m", 0))
        f.write("objects           %d\n" % len(doc.get("objects", [])))
        for i, o in enumerate(doc.get("objects", [])):
            f.write("   %2d  [%7.3f %7.3f %7.3f]  %5.0f mm  %s\n"
                    % (i, o["centre"][0], o["centre"][1], o["centre"][2],
                       o["width_m"] * 1000,
                       "graspable" if o.get("graspable") else "too wide"))
        # WHAT THE QUICK CHECK ACTUALLY DID, INCLUDING NOTHING.
        #
        # `relook` writes into the map only when it re-measured something. If
        # the scene camera sees no change it says so and returns without
        # touching the map -- correct, and it left the summary silent, so a
        # reader would think the stage had not run. In this recording it is
        # the expected answer: the renderer draws a static scene, so a pick
        # does not move a rendered cube and there is genuinely nothing new.
        nochange = any("NOTHING CHANGED" in ln for ln in log)
        regions = [ln.strip() for ln in log if "region(s) changed" in ln]
        f.write("\nquick check       %s\n"
                % (regions[0] if regions else "did not report"))
        if nochange:
            f.write("                  nothing had changed, so no arm motion "
                    "was needed and the map was left alone\n")
        rl = doc.get("relook") or {}
        if rl:
            f.write("\nquick check       %s viewpoint(s), %.0f s\n"
                    % (rl.get("viewpoints"), rl.get("seconds", 0)))
            d = rl.get("diff") or {}
            for k in ("appeared", "gone", "moved", "unchanged"):
                f.write("   %-10s %s\n" % (k, len(d.get(k, []))))
        f.write("\n")
        for k, v in bright.items():
            f.write("%-18s mean pixel %.1f%s\n"
                    % (k, v, "   BLACK -- the capture failed" if v < 5 else ""))
        f.write("""
WHAT THIS IS AND IS NOT
  The arm is SIMULATED and the depth is RENDERED by mock_rgbd_camera. NO
  CAMERA HAS EVER BEEN ATTACHED TO THIS HOST, so this films the pipeline and
  not the room.
  What IS real: the sweep order, the deprojection, the segmentation, the
  fusion, the plane fit, the IK, the approach-angle search, and every refusal.
""")
    print(open(os.path.join(out, "summary.txt")).read())
    bad = [k for k, v in bright.items() if v < 5]
    if bad:
        print("THE CAPTURE FAILED for %s -- a black clip has a perfectly good "
              "file size." % ", ".join(bad))
        return 5
    return 0 if rc1 == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
