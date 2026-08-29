#!/usr/bin/env python3
"""
vision_collage.py -- one labelled picture of what EVERY vision stage sees.

    python3 scripts/vision_collage.py "the green cube"
    python3 scripts/vision_collage.py "the green cube" --camera gripper --arm left
    python3 scripts/vision_collage.py "a bottle" --out recordings/vision/mine

Run it under .venv_vision/bin/python for the neural backends; under the system
interpreter the colour backend still works and the collage SAYS so in the tile
rather than leaving a gap that reads like "found nothing".

WHY A COLLAGE, AND NOT A NUMBER
==========================================================================
"The cameras never detect the objects" is not one failure. On this rig it has
been, at different times: a camera delivering no frames at all; a detector
that ran and found nothing; a detector that found the RIGHT thing with a
score below the threshold; and a detector that found something real that was
not what the operator meant. Those four produce the same empty result list
and need four different fixes, and a printed count distinguishes none of them.

Each backend gets its own tile, drawn on the SAME frame, with its name, its
count, its timing and its refusal reason burned into the tile. Backends fail
at DIFFERENT things -- measured on this rig, YOLO-World finds a person at
0.725 and cannot see the 19 px dark green cube at all, while the colour
backend finds that cube every time and cannot name a person -- so seeing them
side by side on one frame is the whole point.

EVERY TILE IS LABELLED WITH ITS PROVENANCE. A tile that could not run says
why, in the picture, because a collage with a silently missing panel is worse
than no collage: it looks like a result.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src/srl_perception"))
sys.path.insert(0, HERE)

TILE_W, TILE_H = 640, 480
PAD = 8
HDR = 34


def _cv():
    import cv2
    return cv2


def grab_scene():
    """A frame from the room camera, through the ONE probe."""
    cv2 = _cv()
    from srl_perception.srl_cameras import probe_scene_camera
    idx, tried = probe_scene_camera()
    if idx is None:
        return None, "no scene camera delivered a frame: %s" % "; ".join(tried)
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    f = None
    for _ in range(8):
        ok, x = cap.read()
        if ok and x is not None:
            f = x
    cap.release()
    if f is None:
        return None, "/dev/video%d opened but delivered no frame" % idx
    return f, "scene camera /dev/video%d" % idx


def grab_gripper(arm):
    """A frame from a wrist camera over RTSP."""
    cv2 = _cv()
    ip = {"left": "192.168.1.10", "right": "192.168.1.9"}[arm]
    url = "rtsp://%s/color" % ip
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    f = None
    if cap.isOpened():
        for _ in range(12):
            ok, x = cap.read()
            if ok and x is not None:
                f = x
                break
    cap.release()
    if f is None:
        return None, ("%s wrist camera (%s) delivered no frame. If port 554 "
                      "is open the vision MODULE is wedged: "
                      "scripts/reboot_vision_module.py %s" % (arm, url, arm))
    return f, "%s wrist camera %s" % (arm, ip)


def draw_dets(cv2, img, dets, colour=(0, 220, 0)):
    out = img.copy()
    for d in dets:
        x, y, w, h = d.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), colour, 2)
        txt = "%s %.2f" % (d.label, d.score)
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x, max(0, y - th - 6)), (x + tw + 6, y), colour, -1)
        cv2.putText(out, txt, (x + 3, max(10, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cx, cy = int(d.centre_uv[0]), int(d.centre_uv[1])
        cv2.drawMarker(out, (cx, cy), colour, cv2.MARKER_CROSS, 14, 2)
    return out


def tile(cv2, img, title, subtitle, ok=True):
    """One panel: the image, a header, and its provenance. Never unlabelled."""
    import numpy as np
    h, w = img.shape[:2]
    scale = min(TILE_W / float(w), TILE_H / float(h))
    small = cv2.resize(img, (int(w * scale), int(h * scale)))
    canvas = np.zeros((TILE_H + HDR, TILE_W, 3), dtype="uint8")
    canvas[:] = (28, 28, 30)
    y0 = HDR + (TILE_H - small.shape[0]) // 2
    x0 = (TILE_W - small.shape[1]) // 2
    canvas[y0:y0 + small.shape[0], x0:x0 + small.shape[1]] = small
    bar = (0, 130, 0) if ok else (0, 0, 150)
    cv2.rectangle(canvas, (0, 0), (TILE_W, HDR), bar, -1)
    cv2.putText(canvas, title[:46], (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, subtitle[:70], (8, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (225, 225, 225), 1, cv2.LINE_AA)
    return canvas


def collage(cv2, tiles, cols=2):
    import numpy as np
    if not tiles:
        return None
    rows = (len(tiles) + cols - 1) // cols
    th, tw = tiles[0].shape[:2]
    out = np.zeros((rows * th + (rows + 1) * PAD,
                    cols * tw + (cols + 1) * PAD, 3), dtype="uint8")
    out[:] = (18, 18, 20)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        y = PAD + r * (th + PAD)
        x = PAD + c * (tw + PAD)
        out[y:y + th, x:x + tw] = t
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prompt", nargs="?", default="the green cube")
    ap.add_argument("--camera", choices=("scene", "gripper", "both"),
                    default="scene")
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--out", default=None)
    ap.add_argument("--image", default=None,
                    help="run against a saved frame instead of a camera")
    a = ap.parse_args()

    cv2 = _cv()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    outdir = a.out or os.path.join(WS, "recordings", "vision", stamp)
    os.makedirs(outdir, exist_ok=True)

    # ---- the frames
    frames = []
    if a.image:
        img = cv2.imread(a.image)
        if img is None:
            print("cannot read %s" % a.image)
            return 1
        frames.append((img, "file %s" % os.path.basename(a.image)))
    else:
        if a.camera in ("scene", "both"):
            f, why = grab_scene()
            if f is None:
                print("scene camera: %s" % why)
            else:
                frames.append((f, why))
        if a.camera in ("gripper", "both"):
            f, why = grab_gripper(a.arm)
            if f is None:
                print("gripper camera: %s" % why)
            else:
                frames.append((f, why))
    if not frames:
        print("NO FRAME from any camera -- nothing to analyse.")
        return 1

    from srl_perception.prompt_detector import PromptDetector

    report = {"prompt": a.prompt, "stamp": stamp, "frames": [], "tiles": []}
    tiles = []
    for img, src in frames:
        tiles.append(tile(cv2, img, "RAW FRAME", src, ok=True))
        report["frames"].append(src)
        cv2.imwrite(os.path.join(outdir, "raw_%s.png"
                                 % src.split()[0].replace("/", "_")), img)

        for backend in ("colour", "yoloworld", "segment"):
            t0 = time.time()
            try:
                det = PromptDetector(backend=backend, conf=0.05)
                dets = det.detect(img, a.prompt)
                dt = time.time() - t0
                drawn = draw_dets(cv2, img, dets)
                sub = ("%d match(es) in %.2fs -- %s"
                       % (len(dets), dt,
                          (det.last_note or "no note")[:44]))
                tiles.append(tile(cv2, drawn,
                                  "%s  [%s]" % (backend.upper(), src.split()[0]),
                                  sub, ok=bool(dets)))
                report["tiles"].append(dict(
                    backend=backend, source=src, n=len(dets),
                    seconds=round(dt, 3),
                    detections=[d.as_dict() for d in dets]))
                cv2.imwrite(os.path.join(outdir, "%s_%s.png"
                                         % (backend, src.split()[0])), drawn)
            except Exception as e:                            # noqa: BLE001
                # A BACKEND THAT CANNOT RUN GETS A TILE SAYING SO. A missing
                # panel would read as "found nothing", which is a different
                # fact needing a different fix.
                dt = time.time() - t0
                tiles.append(tile(cv2, img,
                                  "%s  -- UNAVAILABLE" % backend.upper(),
                                  "%s: %s" % (type(e).__name__, str(e)[:52]),
                                  ok=False))
                report["tiles"].append(dict(backend=backend, source=src,
                                            unavailable="%s: %s"
                                                        % (type(e).__name__, e)))

    sheet = collage(cv2, tiles, cols=2)
    path = os.path.join(outdir, "collage.png")
    cv2.imwrite(path, sheet)
    with open(os.path.join(outdir, "report.json"), "w") as f:
        json.dump(report, f, indent=2)

    print("\n== VISION COLLAGE ==")
    print("  prompt   %s" % a.prompt)
    for t in report["tiles"]:
        if "unavailable" in t:
            print("  %-10s %-22s UNAVAILABLE  %s"
                  % (t["backend"], t["source"][:22], t["unavailable"][:44]))
        else:
            print("  %-10s %-22s %d match(es)  %.2fs"
                  % (t["backend"], t["source"][:22], t["n"], t["seconds"]))
    print("\n  %s" % path)
    print("  %s" % os.path.join(outdir, "report.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
