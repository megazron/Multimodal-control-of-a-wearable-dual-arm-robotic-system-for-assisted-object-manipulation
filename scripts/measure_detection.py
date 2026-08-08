#!/usr/bin/env python3
"""Detection rate, pose error and latency on the task objects. THE MODE-6 GATE.

    .percep_venv/bin/python scripts/measure_detection.py

Under 95% at the working distance BLOCKS any user study, so this produces the
number that decides whether mode 6 is participant-ready.

WHAT THIS IS AND IS NOT
-----------------------
Images are SYNTHETIC: coloured cuboids on a table plane, rendered through a
pinhole model at known 3D positions, with blur and sensor noise. That gives
exact ground truth, which a real camera cannot.

It is NOT a measurement of the real system. Real lighting, motion blur from a
wrist-mounted camera on a moving arm, the Kinova module's true intrinsics and
rolling shutter are all unmodelled. This project has made exactly this
mistake's opposite before -- the AprilTag characterisation was synthetic and
was reported as "PASSED in simulation only" -- so the same caveat stands
here and the number must be re-measured on the real cameras before any
participant sees mode 6.
"""
import math, sys, time
import numpy as np

OBJECTS = [
    ("green cube",  (0, 200, 0),   0.05),
    ("red block",   (0, 0, 200),   0.04),
    ("blue ball",   (200, 0, 0),   0.04),
]
DISTANCES = (0.25, 0.35, 0.50, 0.75)
N_PER = 25
W, H = 640, 480
F = 550.0


def render(cx, cy, cz, colour, size, rng):
    """Pinhole render of a coloured box on a table, +noise +blur."""
    import cv2
    img = np.full((H, W, 3), 120, np.uint8)
    cv2.rectangle(img, (0, int(H * 0.62)), (W, H), (150, 150, 150), -1)
    u = int(W / 2 + F * cx / cz)
    v = int(H / 2 + F * cy / cz)
    s = max(4, int(F * size / cz))
    cv2.rectangle(img, (u - s // 2, v - s // 2), (u + s // 2, v + s // 2),
                  colour, -1)
    cv2.rectangle(img, (u - s // 2, v - s // 2), (u + s // 2, v + s // 2),
                  tuple(int(0.6 * c) for c in colour), 2)
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    img = np.clip(img.astype(np.int16)
                  + rng.normal(0, 3.0, img.shape), 0, 255).astype(np.uint8)
    return img, (u, v, s)


def main():
    import cv2
    from ultralytics import YOLOWorld
    rng = np.random.default_rng(0)
    det = YOLOWorld("yolov8s-worldv2.pt")
    det.set_classes([o[0] for o in OBJECTS])
    det.predict(np.zeros((H, W, 3), np.uint8), device=0, verbose=False)

    print("DETECTION ON THE TASK OBJECTS -- synthetic images, exact ground truth")
    print("%d trials per object per distance, YOLO-World-s, prompts = %s\n"
          % (N_PER, [o[0] for o in OBJECTS]))
    print("  %-12s %6s %10s %12s %12s %10s"
          % ("object", "dist", "detect%", "pos err mm", "p95 mm", "ms"))
    overall = {}
    for dist in DISTANCES:
        for name, colour, size in OBJECTS:
            hits, errs, lats = 0, [], []
            for _ in range(N_PER):
                cx = rng.uniform(-0.10, 0.10)
                cy = rng.uniform(-0.06, 0.06)
                img, (u, v, s) = render(cx, cy, dist, colour, size, rng)
                t0 = time.monotonic()
                r = det.predict(img, device=0, verbose=False, conf=0.05)[0]
                lats.append((time.monotonic() - t0) * 1000)
                if r.boxes is None or len(r.boxes) == 0:
                    continue
                idx = [i for i, c in enumerate(r.boxes.cls.tolist())
                       if OBJECTS[int(c)][0] == name]
                if not idx:
                    continue
                b = r.boxes.xyxy[idx[0]].tolist()
                hits += 1
                # back-project the box centre at the known depth
                bu, bv = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                ex = (bu - W / 2) * dist / F - cx
                ey = (bv - H / 2) * dist / F - cy
                errs.append(1000 * math.hypot(ex, ey))
            rate = 100.0 * hits / N_PER
            overall.setdefault(dist, []).append(rate)
            print("  %-12s %6.2f %9.0f%% %12s %12s %10.1f"
                  % (name, dist, rate,
                     "%.1f" % np.mean(errs) if errs else "-",
                     "%.1f" % np.percentile(errs, 95) if errs else "-",
                     np.mean(lats)))
    print("\n  %-12s %6s %10s" % ("", "dist", "mean detect%"))
    for d, rates in overall.items():
        m = float(np.mean(rates))
        gate = "PASS" if m >= 95 else "**BELOW THE 95% GATE**"
        print("  %-12s %6.2f %9.0f%%   %s" % ("", d, m, gate))
    print("\n  Working distance for a wrist camera on approach is ~0.25-0.35 m.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
