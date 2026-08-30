#!/usr/bin/env python3
"""Scan the table with BOTH arms and build a 3D model of what is on it.

    bash scripts/bringup_arm.sh left
    bash scripts/bringup_arm.sh right
    python3 scripts/scan_table.py                 # both arms
    python3 scripts/scan_table.py --arms left     # one
    python3 scripts/scan_table.py --dry-run       # measure from where it stands

Output: recordings/baselines/table_model.json
    table   : plane in world, plus the per-arm mount corrections that were
              needed to make the two arms agree
    objects : 3D pose and size of everything standing on the table

WHAT THIS IS FOR
----------------
Two arms with EYE-IN-HAND cameras see different parts of the table and neither
sees all of it.  Fusing them needs both cameras to agree about where the table
is -- and out of the box they do not, by 8.45 deg.  So the scan CALIBRATES as
it goes: every viewpoint contributes a table-normal observation, and
`handeye_from_table` solves for the mount correction that makes them
consistent.  The corrections are reported, because their SIZE is the honest
error bar on every object pose that follows.

THE DIVISION OF LABOUR THAT MATTERS
-----------------------------------
Object poses in world are ESTIMATES: they carry whatever mount error is left.
Use them to decide what to reach for and roughly where.  Do NOT use them to
decide when to stop descending -- for that use `srl_scene.pad_height_above`,
which is computed inside the camera frame and carries no mount error at all.
That split is the whole design.
"""
import argparse
import json
import math
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from handeye_from_table import solve as solve_handeye  # noqa: E402
from srl_scene import fit_plane, plane_rms_mm, segment_objects  # noqa: E402
from srl_fk import FK  # noqa: E402

WS = "/home/gausms/kortex_ws"
OUT = WS + "/recordings/baselines/table_model.json"

# Viewpoints are perturbations of wherever the arm starts, so the scan does not
# depend on a saved pose.  Joint 1 is given the smallest range on purpose:
# rotating it is what carries the elbow inboard across the wearer.
VIEWS = [
    (0.0, 0.0, 0.0, 0.0),
    (-6.0, 8.0, -10.0, 12.0),
    (6.0, -8.0, 10.0, -12.0),
    (0.0, 12.0, -14.0, 20.0),
    (0.0, -12.0, 14.0, -20.0),
]
VIEW_JOINTS = (0, 1, 3, 5)
MERGE_MM = 45.0


def measure_cloud(node, arm):
    """Deproject this arm's depth image. Returns (points_in_cam, q)."""
    if not node.frames():
        return None, None
    d, dk = node.img["d"], node.img["dk"]
    dep = np.frombuffer(d.data, np.uint16).reshape(
        d.height, d.width).astype(np.float32) * 0.001
    fx, fy, cx, cy = dk.k[0], dk.k[4], dk.k[2], dk.k[5]
    v, u = np.nonzero((dep > 0.08) & (dep < 2.0))
    if len(v) < 500:
        return None, None
    Z = dep[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    return P, node.fresh_q()


def scan_arm(node, fk, arm, views, dry_run):
    """Collect (points, q, plane) at several viewpoints."""
    obs = []
    q0 = node.fresh_q().copy()
    for i, dv in enumerate(views):
        if i > 0 and not dry_run:
            q = q0.copy()
            for j, deg in zip(VIEW_JOINTS, dv):
                q[j] += math.radians(deg)
            if node.goto(q, "%s-view-%d" % (arm, i)) is None:
                print("    view %d: motion guard fired, stopping this arm" % i)
                break
        P, q = measure_cloud(node, arm)
        if P is None:
            print("    view %d: no usable depth" % i)
            continue
        n, d, m = fit_plane(P)
        rms = plane_rms_mm(P, n, d, m)
        inl = float(m.mean())
        print("    view %d: %5.1f%% plane inliers, %.2f mm RMS, %d pts"
              % (i, inl * 100, rms, len(P)))
        if inl < 0.12 or rms > 4.0:
            print("      rejected: that is not a clean plane")
            continue
        obs.append({"P": P, "q": q, "n": n, "d": d, "rms": rms})
        if dry_run:
            break
    return obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", default="left,right")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    arms = [a.strip() for a in args.arms.split(",") if a.strip()]

    rclpy.init()
    fk = FK()
    per_arm, nodes = {}, {}
    for arm in arms:
        print("\n=== scanning with the %s arm ===" % arm)
        # Eye is written for the left arm's topics; import lazily per arm
        from servo_pick_left import Eye
        node = Eye(arm)
        node.fk = fk
        if not node.preflight():
            print("  retrying discovery for the %s arm" % arm)
            if not node.preflight(wait=25.0):
                print("  %s arm not available; skipping" % arm)
                continue
        node.set_deadband(0.10)
        node._hold = node.fresh_q().copy()
        nodes[arm] = node
        obs = scan_arm(node, fk, arm, VIEWS, args.dry_run)
        if obs:
            per_arm[arm] = obs
        print("  %d usable viewpoints" % len(obs))

    if not per_arm:
        raise SystemExit("no usable viewpoints from any arm")

    # ---- calibrate each arm's mount from its own views of the one table ----
    print("\n=== hand-eye correction from the table ===")
    corr = {}
    for arm, obs in per_arm.items():
        Rs = [fk.poses(arm, o["q"], ["camera_depth_frame"])[0][:3, :3] for o in obs]
        ns = [o["n"] for o in obs]
        if len(obs) < 2:
            print("  %-5s only %d view(s): cannot calibrate, using identity"
                  % (arm, len(obs)))
            corr[arm] = np.eye(3)
            continue
        C, before, after = solve_handeye(Rs, ns)
        corr[arm] = C
        ang = math.degrees(math.acos(float(np.clip((np.trace(C) - 1) / 2, -1, 1))))
        print("  %-5s views disagreed by %.2f deg -> %.2f deg after a %.2f deg "
              "correction" % (arm, before, after, ang))

    # ---- fuse: everything into world, with the correction applied ----
    print("\n=== fusing into one table model ===")
    normals, pts_world, objects = [], [], []
    for arm, obs in per_arm.items():
        C = corr[arm]
        for o in obs:
            T, = fk.poses(arm, o["q"], ["camera_depth_frame"])
            R, t = T[:3, :3] @ C, T[:3, 3]
            nw = R @ o["n"]
            if nw[2] < 0:
                nw = -nw
            normals.append(nw)
            local = segment_objects(o["P"], o["n"], o["d"])
            for ob in local:
                objects.append({
                    "arm": arm,
                    "centre_world": (R @ ob["centre"] + t).tolist(),
                    "top_world": (R @ ob["top_centre"] + t).tolist(),
                    "size_mm": ob["size_mm"],
                    "n_pts": ob["n_pts"],
                })
            keep = o["P"][np.abs(o["P"] @ o["n"] + o["d"]) < 0.006]
            if len(keep):
                sub = keep[:: max(1, len(keep) // 400)]
                pts_world.append((R @ sub.T).T + t)

    n_w = np.mean(normals, 0)
    n_w /= np.linalg.norm(n_w)
    allp = np.vstack(pts_world)
    d_w = -float(n_w @ allp.mean(0))
    tilt = math.degrees(math.acos(min(1.0, abs(n_w[2]))))
    resid = float(np.sqrt(((allp @ n_w + d_w) ** 2).mean()) * 1000)
    print("  table normal in world (%.3f, %.3f, %.3f), %.2f deg off vertical"
          % (*n_w, tilt))
    print("  fused surface residual across all arms/views: %.1f mm" % resid)

    # ---- merge duplicate detections of the same object ----
    merged = []
    for ob in sorted(objects, key=lambda o: -o["n_pts"]):
        c = np.array(ob["centre_world"])
        for m in merged:
            if np.linalg.norm(c - np.array(m["centre_world"])) < MERGE_MM / 1000.0:
                m["seen_by"] = sorted(set(m["seen_by"] + [ob["arm"]]))
                m["n_views"] += 1
                break
        else:
            ob = dict(ob)
            ob["seen_by"] = [ob.pop("arm")]
            ob["n_views"] = 1
            merged.append(ob)

    print("\n  %d object(s) on the table:" % len(merged))
    print("  %-3s %-28s %-22s %-8s %s"
          % ("#", "centre in world (m)", "size mm (w x d x h)", "views", "seen by"))
    for i, m in enumerate(merged):
        c = m["centre_world"]
        s = m["size_mm"]
        print("  %-3d (%+.3f,%+.3f,%+.3f)      %5.1f x %5.1f x %5.1f   %-8d %s"
              % (i, c[0], c[1], c[2], s[0], s[1], s[2], m["n_views"],
                 ",".join(m["seen_by"])))

    json.dump({
        "table": {"normal_world": n_w.tolist(), "offset": d_w,
                  "tilt_deg": tilt, "fused_residual_mm": resid},
        "handeye_correction_deg": {
            a: math.degrees(math.acos(float(np.clip((np.trace(C) - 1) / 2, -1, 1))))
            for a, C in corr.items()},
        "objects": merged,
        "caveat": ("object poses carry the residual mount error; use "
                   "srl_scene.pad_height_above for descent, which does not"),
    }, open(OUT, "w"), indent=1)
    print("\nwrote %s" % OUT)


if __name__ == "__main__":
    main()
