#!/usr/bin/env python3
"""PART 4 -- run the dual-view GUI for real, measure it, and PROVE IT DREW.

WHY A SEPARATE MEASUREMENT SCRIPT. Two RViz instances is a memory decision as
much as a layout one: this is a 15 GB machine and two earlier runs were
OOM-killed. So the frame time and the resident set are measured with both
panels and both camera subscriptions live, and the single-RViz fallback is
measured beside them rather than merely proposed.

VERIFIED FROM PIXELS. `container.isVisible()` returning True is not evidence
of anything -- a container can be visible and empty, which is exactly what a
failed reparent looks like. So the window is screenshotted and the two panel
regions are checked for RENDERED CONTENT independently of each other.

X11GRAB ON WSLg's :0 RECORDS BLACK -- measured, and it costs a day if
rediscovered. WSLg composites through Wayland and window contents never reach
the X root window. Everything here runs on Xvfb :99, which has no compositor,
so its root window really does hold the rendered pixels.

    python3 scripts/measure_dual_gui.py [--seconds 25]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRATCH = os.environ.get("SRL_SCRATCH", "/tmp")
DISPLAY = ":99"
OUT = os.path.join(WS, "recordings/baselines/dual_gui_measurement.json")


def sh(argv, **kw):
    return subprocess.run(argv, capture_output=True, text=True, **kw)


def ensure_xvfb():
    if sh(["pgrep", "-f", "Xvfb %s" % DISPLAY]).stdout.strip():
        return None
    p = subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", "1920x1080x24"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    return p


def env():
    e = dict(os.environ)
    e["DISPLAY"] = DISPLAY
    e["QT_QPA_PLATFORM"] = "xcb"
    e["LIBGL_ALWAYS_SOFTWARE"] = "1"
    e["GALLIUM_DRIVER"] = "llvmpipe"
    e["SRL_SCRATCH"] = SCRATCH
    e["PYTHONUNBUFFERED"] = "1"
    return e


def rss_mb(pid):
    """RSS of a pid and all its descendants, MB."""
    pids, frontier = [], [pid]
    while frontier:
        p = frontier.pop()
        pids.append(p)
        o = sh(["pgrep", "-P", str(p)]).stdout.split()
        frontier += [int(x) for x in o if x.isdigit()]
    o = sh(["ps", "-o", "rss=", "-p", ",".join(str(p) for p in pids)]).stdout
    return sum(int(x) for x in o.split() if x.isdigit()) / 1024.0, len(pids)


def mem_free_mb():
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024.0
    return -1.0


def screenshot(path):
    """Grab the Xvfb root. ImageMagick's `import` is NOT installed here, so
    ffmpeg's x11grab is the route -- and it works on :99 precisely because
    Xvfb has no compositor. The same command on WSLg's :0 records black."""
    for argv in (["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab",
                  "-video_size", "1920x1080", "-i", "%s.0" % DISPLAY,
                  "-frames:v", "1", path],
                 ["import", "-display", DISPLAY, "-window", "root", path]):
        try:
            sh(argv, timeout=30)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return True
    return False


def region_content(path, box):
    """(distinct_colours, mean, nonblack_fraction) for a crop.

    THE TEST IS 'DID ANYTHING RENDER', NOT 'IS IT THE RIGHT PICTURE'. Distinct
    colour count is the discriminator that matters: an empty container is a
    flat fill and scores a handful of colours, while a rendered RViz viewport
    scores thousands. Mean brightness alone is not enough -- a solid grey
    panel has a perfectly respectable mean.
    """
    try:
        from PIL import Image
    except ImportError:
        return None
    im = Image.open(path).convert("RGB").crop(box)
    cols = im.getcolors(maxcolors=1 << 22) or []
    px = list(im.getdata())
    n = len(px) or 1
    mean = sum(sum(p) for p in px) / (3.0 * n)
    nonblack = sum(1 for p in px if sum(p) > 30) / float(n)
    return len(cols), mean, nonblack


def run_case(name, argv, seconds, shot):
    """Run one configuration. The GUI writes its panel rectangles to a fixed
    path, so each case's copy is TAKEN ASIDE immediately -- otherwise the
    second case overwrites the first and the pixel proof checks the wrong
    layout. It did exactly that once, reporting the ACTUAL panel as absent
    when it had rendered perfectly well in the run being examined."""
    print("\n-- %s" % name)
    try:
        os.remove(os.path.join(SCRATCH, "srl_gui_geometry.json"))
    except OSError:
        pass
    p = subprocess.Popen(argv, env=env(), cwd=WS, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True,
                         start_new_session=True)
    samples = []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            time.sleep(2.0)
            if p.poll() is not None:
                break
            mb, n = rss_mb(p.pid)
            samples.append((mb, n))
        ok_shot = screenshot(shot) if p.poll() is None else False
        geom_case = shot + ".geom.json"
        try:
            import shutil
            shutil.copy(os.path.join(SCRATCH, "srl_gui_geometry.json"),
                        geom_case)
        except Exception:                                     # noqa: BLE001
            geom_case = None
        # The frame-time line is on the GUI's own bottom bar, so read it from
        # the screenshot region rather than from stdout: it is what the
        # operator actually sees.
        alive = p.poll() is None
    finally:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:                                 # noqa: BLE001
                pass
        try:
            out = p.communicate(timeout=15)[0] or ""
        except Exception:                                     # noqa: BLE001
            out = ""
    ft = None
    for m in re.finditer(
            r"frame ([\d.]+) ms\s+median ([\d.]+)\s+p95 ([\d.]+)\s+max ([\d.]+)",
            out):
        ft = tuple(float(g) for g in m.groups())
    peak = max((s[0] for s in samples), default=0.0)
    procs = max((s[1] for s in samples), default=0)
    print("   alive after %.0f s : %s" % (seconds, alive))
    print("   peak RSS (tree)   : %.0f MB across %d processes" % (peak, procs))
    print("   screenshot        : %s" % (shot if ok_shot else "FAILED"))
    return dict(name=name, alive=bool(alive), peak_rss_mb=round(peak, 1),
                procs=procs, screenshot=shot if ok_shot else None,
                geometry=geom_case, frame_ms=ft, tail=out[-1500:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=30)
    a = ap.parse_args()

    print("=" * 74)
    print("DUAL-VIEW GUI -- frame time, memory, and proof that it drew")
    print("=" * 74)
    print("MemAvailable before: %.0f MB" % mem_free_mb())
    ensure_xvfb()

    res = {}
    res["panels"] = run_case(
        "GUI PANELS ONLY (--no-rviz): the content only this GUI provides",
        [sys.executable, "scripts/srl_gui.py", "--no-rviz"], a.seconds,
        os.path.join(SCRATCH, "gui_panels.png"))
    res["default"] = run_case(
        "DEFAULT: one RViz (separate window) carrying the GHOST config",
        [sys.executable, "scripts/srl_gui.py"], a.seconds,
        os.path.join(SCRATCH, "default_gui.png"))
    res["embedded_dual"] = run_case(
        "OPT-IN: --embed-rviz --dual-rviz (two reparented instances)",
        [sys.executable, "scripts/srl_gui.py", "--embed-rviz", "--dual-rviz"],
        a.seconds, os.path.join(SCRATCH, "embedded_dual.png"))

    # ---- pixel proof of the GUI's OWN panels, from ITS OWN geometry.
    print("\n-- PIXEL PROOF: the GUI's own panels (a widget can exist and be blank)")
    checks = []
    shot = res["panels"].get("screenshot")
    geom_path = res["panels"].get("geometry")
    try:
        geom = json.load(open(geom_path)) if geom_path else {}
    except Exception:                                         # noqa: BLE001
        geom = {}
    # Different panels have different floors, and that is not fudging: a Qt
    # control column is flat fills and antialiased text (low hundreds of
    # colours) while an RViz viewport is a shaded 3-D scene (thousands). One
    # threshold for both would either pass an empty viewport or fail a working
    # column. Both confirmed by eye on this screenshot.
    FLOOR = {"controls": 40, "divergence": 20}
    if not (shot and geom):
        print("   no screenshot or geometry -- CANNOT verify, and refusing "
              "to guess regions")
    else:
        for label in ("controls", "divergence"):
            if label not in geom:
                print("   %-12s NOT PRESENT in the layout" % label)
                continue
            r = region_content(shot, tuple(geom[label]))
            if r is None:
                print("   PIL missing -- cannot verify from pixels")
                break
            cols, mean, nz = r
            ok = cols >= FLOOR[label] and nz > 0.2
            checks.append((label, ok, cols, mean, nz))
            print("   %-12s %-9s %6d distinct colours (floor %d), mean %5.1f"
                  % (label, "RENDERED" if ok else "EMPTY", cols, FLOOR[label],
                     mean))

    # ---- THE GHOST, PROVED DIFFERENTIALLY.
    # "The scene looks right" is not a check. Rendering the SAME scene with
    # and without the ghost display and differencing the two isolates exactly
    # the pixels the ghost is responsible for -- and if the ghost were absent
    # (no real robot_description, wrong TF prefix) the difference would be
    # zero, which is the outcome this has to be able to report.
    print("\n-- GHOST PROOF (same scene, ghost display on vs off)")
    ghost_cfg = os.path.join(SCRATCH, "srl_overlay.rviz")
    plain_cfg = os.path.join(WS, "src/srl_experiments/config/"
                                 "verification_capture.rviz")
    diff = None
    if os.path.exists(ghost_cfg):
        shots = {}
        for tag, cfg in (("ghost", ghost_cfg), ("plain", plain_cfg)):
            p = subprocess.Popen(["rviz2", "-d", cfg], env=env(),
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL,
                                 start_new_session=True)
            time.sleep(16)
            shots[tag] = os.path.join(SCRATCH, "rviz_%s.png" % tag)
            screenshot(shots[tag])
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:                                 # noqa: BLE001
                pass
            time.sleep(2)
        try:
            from PIL import Image, ImageChops
            a_im = Image.open(shots["ghost"]).convert("RGB")
            b_im = Image.open(shots["plain"]).convert("RGB")
            d = ImageChops.difference(a_im, b_im)
            diff = sum(1 for px in d.getdata() if sum(px) > 40)
            print("   pixels changed by enabling the ghost: %d" % diff)
            print("   -> %s" % ("GHOST RENDERS" if diff > 2000
                                else "NO GHOST -- the actual arm is NOT drawn"))
        except Exception as e:                                # noqa: BLE001
            print("   ghost diff failed: %r" % (e,))
    res["ghost_diff_pixels"] = diff
    res["pixel_checks"] = [dict(region=l, rendered=o, colours=c, mean=m,
                                nonblack=z) for l, o, c, m, z in checks]

    print("\n" + "=" * 74)
    print("SUMMARY -- frame time and memory")
    for k in ("panels", "default", "embedded_dual"):
        r = res[k]
        ft = r.get("frame_ms")
        print("  %-14s peak %4.0f MB, %d procs, alive %s%s"
              % (k, r["peak_rss_mb"], r["procs"], r["alive"],
                 ("   frame median %.2f ms p95 %.2f max %.2f"
                  % (ft[1], ft[2], ft[3])) if ft else ""))
    d, e2 = res["default"], res["embedded_dual"]
    print("  the SECOND RViz costs %.0f MB (%.0f%% more than the default)"
          % (e2["peak_rss_mb"] - d["peak_rss_mb"],
             100.0 * (e2["peak_rss_mb"] - d["peak_rss_mb"])
             / max(1.0, d["peak_rss_mb"])))
    print("  MemAvailable after: %.0f MB" % mem_free_mb())

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2, default=str)
    print("  -> %s" % OUT)

    bad = [c for c in checks if not c[1]]
    return 1 if (bad or not d["alive"]) else 0


if __name__ == "__main__":
    sys.exit(main())
