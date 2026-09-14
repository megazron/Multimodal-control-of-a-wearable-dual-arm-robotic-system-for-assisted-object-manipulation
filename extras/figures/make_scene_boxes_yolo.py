#!/usr/bin/env python3
"""The room-camera scene through a real detector: YOLO-World (open
vocabulary) asked, by name, for the mannequin, the table, the robot arms and
the objects on the table. Every box drawn is the detector's own output with
its confidence; nothing is hand-placed. Classes it did not find are listed
as missed in the printed report and in the figure legend.

    .venv_vision/bin/python extras/figures/make_scene_boxes_yolo.py

Frame: recordings/vision_thesis/20260830_073141/raw/scene_hd/colour.png
Model: yolov8s-worldv2.pt at the repository root (the project's own copy).
"""

import json
import os
import sys

import cv2
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle, Patch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 2))
FRAME = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141", "raw", "scene_hd", "colour.png")
MODEL = os.path.join(ROOT, "yolov8s-worldv2.pt")
OUT = os.path.join(HERE, "vision", "scene_boxes_yolo.pdf")

PROMPTS = ["mannequin", "table", "robot arm", "box", "cube"]
COLOURS = {"mannequin": "#c1392b", "table": "0.35", "robot arm": "#2c6fbb",
           "box": "#3f9142", "cube": "#3f9142"}
CONF = 0.10
CROP = (330, 90, 950, 720)   # x0, y0, x1, y1 in the 1280x720 frame: the mannequin, its arms and the table; the lab robots at the sides are not part of the scene


def cropped_frame():
    im = cv2.imread(FRAME)
    x0, y0, x1, y1 = CROP
    return im[y0:y1, x0:x1]


def detect():
    from ultralytics import YOLO
    model = YOLO(MODEL)
    model.set_classes(PROMPTS)
    res = model.predict(cropped_frame(), conf=CONF, imgsz=960, device="cpu", verbose=False)[0]
    dets = []
    for b in res.boxes:
        x0, y0, x1, y1 = b.xyxy[0].tolist()
        dets.append({"class": PROMPTS[int(b.cls)], "conf": float(b.conf), "box": [x0, y0, x1, y1]})
    found = sorted({d["class"] for d in dets})
    missed = [p for p in PROMPTS if p not in found]
    json.dump({"frame": os.path.relpath(FRAME, ROOT), "crop_xyxy": CROP, "model": os.path.basename(MODEL),
               "conf_threshold": CONF, "detections": dets, "missed": missed},
              open(OUT.replace(".pdf", ".json"), "w"), indent=1)
    return dets, missed


def main():
    cache = OUT.replace(".pdf", ".json")
    if "--cached" in sys.argv and os.path.exists(cache):  # redraw only; the detections are the model's
        j = json.load(open(cache))
        dets, missed = j["detections"], j["missed"]
    else:
        dets, missed = detect()
    print(json.dumps({"detections": dets, "missed": missed}, indent=1))
    im = cv2.cvtColor(cropped_frame(), cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(figsize=(5.2, 5.6))
    ax.imshow(im)
    ax.axis("off")
    placed = []  # (x0, x_end, y) of labels already drawn above a box
    for d in sorted(dets, key=lambda d: d["box"][0]):
        x0, y0, x1, y1 = d["box"]
        c = COLOURS[d["class"]]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=c, lw=2.0))
        text = "%s %.2f" % (d["class"], d["conf"])
        width = 9.0 * len(text)  # px, approximate at this font size
        clash = any(abs(py - y0) < 25 and x0 < px1 for px0, px1, py in placed)
        if clash:  # a label already sits above here; drop this one inside its own box
            ax.text(x0 + 4, y0 + 4, text, color="white", fontsize=8.5, va="top",
                    bbox=dict(facecolor=c, edgecolor="none", pad=1.5))
        else:
            ax.text(x0 + 3, y0 - 4, text, color="white", fontsize=8.5, va="bottom",
                    bbox=dict(facecolor=c, edgecolor="none", pad=1.5))
            placed.append((x0, x0 + width, y0))
    handles = [Patch(fill=False, edgecolor=COLOURS[p], label=p + (" -- not found" if p in missed else ""))
               for p in PROMPTS]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=3,
              frameon=False, fontsize=9, handlelength=1.6, columnspacing=1.5)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.14)
    fig.savefig(OUT)
    fig.savefig(OUT.replace(".pdf", ".png"), dpi=150)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
