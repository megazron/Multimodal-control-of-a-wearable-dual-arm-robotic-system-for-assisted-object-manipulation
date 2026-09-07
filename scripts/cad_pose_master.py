#!/usr/bin/env python3
"""Pose the CAD master arm at chosen joint angles and export it: shaded
images for the thesis, and posed STL meshes that Fusion (or any CAD) can
import, because the Fusion source has no joints -- it was drawn for printing.

The link meshes (src/srl_description/meshes/master_arm_cad/*.stl) are in the
CAD's own global frame at the ZERO configuration, arm along +z, handle at the
top. Joint i sits on the z axis at height JZ[i]; the axis alternates roll (z)
/ bend (y), as master_arm_cad.urdf.xacro says. Posing is therefore: for each
joint in order, rotate every mesh downstream of it about that joint's axis
through (0, 0, JZ[i]) -- the same kinematics the URDF encodes.

    python3 scripts/cad_pose_master.py --out /mnt/c/Users/Gausms/Downloads/master_arm_posed
    python3 scripts/cad_pose_master.py --pose 0,25,0,65,0,0,0 --name reach --views iso,front,side

Angles are degrees, in joint order J1..J7, positive = right-hand rule about
the URDF axis (+z for roll, +y for bend; +bend tips the upper arm toward +x).
"""
import argparse
import os
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection      # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH = os.path.join(ROOT, "src/srl_description/meshes/master_arm_cad")
JZ = [0.023, 0.0607, 0.1030, 0.1397, 0.1800, 0.2187, 0.2600]
AXIS = [np.array(a, float) for a in ([0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1])]
LINKS = ["link%d" % i for i in range(1, 8)]

PRINT = np.array([0.82, 0.84, 0.86])
POT = np.array([0.25, 0.62, 0.70])
BASEC = np.array([0.55, 0.58, 0.60])

POSES = {
    # name: (angles deg J1..J7, caption)
    "zero":    ([0, 0, 0, 0, 0, 0, 0], "zero configuration, as modelled"),
    "forward": ([0, 20, 0, 70, 0, 0, 0], "upper arm forward, elbow bent: the handle points ahead of the wearer"),
    "reach":   ([0, 45, 0, 45, 0, 0, 0], "reaching forward, arm nearly straight"),
    "carry":   ([0, 10, 0, 90, 0, -10, 0], "elbow at a right angle, handle standing up in front of the operator"),
    "hanging": ([0, 20, 0, 70, 0, 0, 180], "arm forward, handle rolled to hang downward"),
    "sideways": ([0, 20, 0, 70, 90, 0, 0], "arm forward, handle rolled to the operator's side"),
}


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(b, dtype=rec, count=n, offset=84)["v"].astype(np.float64)


def write_stl(path, tri):
    tri = tri.astype(np.float32)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tri
    with open(path, "wb") as f:
        f.write(b"posed master arm, srl_description/meshes/master_arm_cad".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tri)))
        f.write(rec.tobytes())


def rot(axis, deg):
    a = axis / np.linalg.norm(axis)
    t = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def pose(parts, q):
    """Rotate downstream links about each joint in turn. parts: name -> (N,3,3)."""
    out = {k: v.copy() for k, v in parts.items()}
    for i, (deg, ax) in enumerate(zip(q, AXIS)):
        if abs(deg) < 1e-9:
            continue
        R = rot(ax, deg)
        c = np.array([0, 0, JZ[i]])
        for nm in LINKS[i:]:
            out[nm] = (out[nm] - c) @ R.T + c
    return out


def shaded(tri, base, light=(0.4, -0.6, 0.7)):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    l = np.array(light) / np.linalg.norm(light)
    lam = np.clip(n @ l, 0, 1)
    k = 0.45 + 0.55 * lam
    return np.clip(base[None, :] * k[:, None], 0, 1)


def draw(ax, parts, elev, azim, lim):
    for nm, tri in parts.items():
        base = BASEC if nm in ("base", "backpack") else PRINT
        col = Poly3DCollection(tri, facecolors=shaded(tri, base), edgecolor="none", linewidths=0)
        ax.add_collection3d(col)
    ax.view_init(elev=elev, azim=azim)
    (x0, x1), (y0, y1), (z0, z1) = lim
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0), zoom=1.75)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(z0, z1)
    ax.set_axis_off()


VIEWS = {"iso": (22, -55), "front": (0, -90), "side": (0, 0), "top": (89, -90), "iso2": (18, 35), "operator": (12, -20)}


def bounds(parts, pad=0.02):
    v = np.concatenate([t.reshape(-1, 3) for t in parts.values()])
    lo, hi = v.min(0) - pad, v.max(0) + pad
    return [(lo[i], hi[i]) for i in range(3)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "thesis_v3", "figures", "cad_posed"))
    ap.add_argument("--pose", help="J1..J7 degrees, comma-separated (overrides the built-in set)")
    ap.add_argument("--name", default="custom")
    ap.add_argument("--views", default="iso,front,side")
    ap.add_argument("--backpack", action="store_true", help="include the mount (slow: 133k triangles)")
    ap.add_argument("--dpi", type=int, default=200)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    parts = {nm: read_stl(os.path.join(MESH, nm + ".stl")) for nm in ["base"] + LINKS}
    if a.backpack:
        parts["backpack"] = read_stl(os.path.join(MESH, "backpack.stl"))
    poses = {a.name: ([float(x) for x in a.pose.split(",")], "custom pose")} if a.pose else POSES
    views = [v.strip() for v in a.views.split(",")]

    for name, (q, cap) in poses.items():
        P = pose(parts, q)
        lim = bounds(P)
        # STL: one file per link plus one merged, so Fusion can import either
        d = os.path.join(a.out, "stl_%s" % name)
        os.makedirs(d, exist_ok=True)
        for nm, tri in P.items():
            write_stl(os.path.join(d, "%s.stl" % nm), tri)
        write_stl(os.path.join(a.out, "master_arm_%s.stl" % name), np.concatenate(list(P.values())))
        # images: one per view, and one strip
        fig = plt.figure(figsize=(4.2 * len(views), 5.2), facecolor="white")
        for k, v in enumerate(views, 1):
            ax = fig.add_subplot(1, len(views), k, projection="3d")
            draw(ax, P, *VIEWS[v], lim)
            ax.set_title(v, fontsize=11, color="#20262b", pad=0)
        fig.suptitle("Master arm -- %s (J1..J7 = %s deg)" % (cap, ", ".join("%g" % x for x in q)),
                     fontsize=11, color="#20262b")
        fig.savefig(os.path.join(a.out, "master_arm_%s.png" % name), dpi=a.dpi, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        for v in views:
            fig = plt.figure(figsize=(5, 6), facecolor="white")
            ax = fig.add_subplot(111, projection="3d")
            draw(ax, P, *VIEWS[v], lim)
            fig.savefig(os.path.join(a.out, "master_arm_%s_%s.png" % (name, v)), dpi=a.dpi, bbox_inches="tight",
                        facecolor="white", transparent=False)
            plt.close(fig)
        print("pose %-8s q=%s -> %s" % (name, q, os.path.join(a.out, "master_arm_%s.png" % name)), flush=True)


if __name__ == "__main__":
    main()
