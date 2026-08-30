#!/usr/bin/env python3
"""The arm's actual SHAPE, from its collision meshes, in world coordinates.

    python3 scripts/srl_body_geometry.py --self-test
    python3 scripts/srl_body_geometry.py --tool          # finger geometry
    python3 scripts/srl_body_geometry.py --clearance     # wearer clearance

WHY THIS EXISTS
---------------
Two separate things in this repository ask "where is the arm", and both were
answering with LINK ORIGINS -- single points at the joints -- while the arm is
a set of tubes and a hand between them.

  THE WEARER CHECK.  `docs/PICK_THE_CUBE.md` records the measurement: asked
  about a pose in which the arm was PHYSICALLY JAMMED AGAINST THE MANNEQUIN,
  `ClearanceModel` returned 367.0 mm -- the same number it returns for a
  visibly clear pose. It samples origins, so the metal BETWEEN the joints is
  invisible to it, and that metal is what touches the person. CLAUDE.md lists
  HARD CONSTRAINT 11 as currently unenforced in code for exactly this reason.

  THE DESCENT.  The gripper's lowest point is not the end-effector origin and
  not the pad midpoint -- it is the bottom of the finger tips, which SWING on
  a four-bar as the hand opens. Descending to a stop computed from the wrist
  drives the fingers into the table by whatever that difference is.

Both are the same missing fact: the arm's geometry. This reads the collision
meshes the URDF already names, transforms their vertices by the same FK the
planner uses, and answers in world coordinates.

NO NEW DEPENDENCY. trimesh is not installed and this deliberately does not
install it: CLAUDE.md records that adding a package to `.venv_vision` once
took `real_calibration/check_all.py` from 4/4 to 2/4. Binary STL is 84 bytes
of header and 50 bytes per triangle; the reader below is fifteen lines.

MESHES ARE SUBSAMPLED, AND THE SUBSAMPLE IS A HULL, NOT A STRIDE. Taking
every Nth vertex of an STL biases toward wherever the exporter happened to
put dense triangles. The extreme points are what matter for a clearance
bound, so the reduction keeps the convex-hull-ish extremes: the farthest
vertex along each of a fixed set of directions, plus the axis-aligned bounds.
That keeps the bound CONSERVATIVE, which is the only safe direction to err.
"""
import argparse
import glob
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_fk import FK  # noqa: E402

WS = "/home/gausms/kortex_ws"
_CACHE = {}

# Directions used to keep extreme vertices. 26 of them: the 6 axes, 12 edges
# and 8 corners of a cube. Cheap, and it cannot drop a spike that sticks out.
_DIRS = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1)
                  for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)], float)
_DIRS /= np.linalg.norm(_DIRS, axis=1, keepdims=True)


def _resolve(uri):
    """package://pkg/rest -> a real path, preferring an .STL we can read."""
    if not uri.startswith("package://"):
        return uri if os.path.exists(uri) else None
    pkg, rest = uri[len("package://"):].split("/", 1)
    for root in ("%s/install/%s/share/%s" % (WS, pkg, pkg),
                 "/opt/ros/jazzy/share/%s" % pkg):
        base = os.path.join(root, rest)
        for cand in (base, base + ".stl", base + ".STL"):
            if os.path.exists(cand) and cand.lower().endswith(".stl"):
                return cand
        # the URDF may name a .dae while an .STL sits beside it; the STL is
        # the one this reader can take, and they are the same geometry
        stem = os.path.splitext(base)[0]
        for cand in (stem + ".STL", stem + ".stl"):
            if os.path.exists(cand):
                return cand
    return None


def read_stl(path):
    """Unique vertices of a binary or ASCII STL, as an (N, 3) array."""
    with open(path, "rb") as fh:
        head = fh.read(84)
        if len(head) < 84:
            return np.zeros((0, 3))
        n = struct.unpack("<I", head[80:84])[0]
        body = fh.read()
    expect = n * 50
    if len(body) >= expect and n > 0:                     # binary
        v = []
        for i in range(n):
            off = i * 50 + 12                             # skip the normal
            v.extend(struct.unpack("<9f", body[off:off + 36]))
        return np.array(v, float).reshape(-1, 3)
    # ASCII fallback
    out = []
    with open(path, "r", errors="replace") as fh:
        for ln in fh:
            ln = ln.strip()
            if ln.startswith("vertex"):
                out.append([float(x) for x in ln.split()[1:4]])
    return np.array(out, float) if out else np.zeros((0, 3))


def _reduce(v):
    """Extreme vertices only -- conservative, and small enough to be fast."""
    if len(v) <= 40:
        return v
    keep = set()
    for d in _DIRS:
        keep.add(int(np.argmax(v @ d)))
    for ax in range(3):
        keep.add(int(np.argmin(v[:, ax])))
        keep.add(int(np.argmax(v[:, ax])))
    return v[sorted(keep)]


def link_points(fk, link_name):
    """Collision-mesh vertices of one link, in the LINK's own frame.

    Includes the collision element's own origin transform, which several
    Robotiq links use -- ignoring it puts the fingers in the wrong place by
    centimetres, and it would look like an FK error.
    """
    if link_name in _CACHE:
        return _CACHE[link_name]
    ln = next((x for x in fk.robot.links if x.name == link_name), None)
    pts = []
    if ln is not None:
        for c in (ln.collisions or []):
            g = c.geometry
            fn = getattr(g, "filename", None)
            if fn:
                p = _resolve(fn)
                if p:
                    v = _reduce(read_stl(p))
                    if len(v):
                        s = getattr(g, "scale", None)
                        if s is not None:
                            v = v * np.asarray(s, float)
                        pts.append(_apply_origin(v, c.origin))
            elif hasattr(g, "radius") and hasattr(g, "length"):
                r, L = float(g.radius), float(g.length)
                v = []
                for zz in (-L / 2, 0.0, L / 2):
                    for a in range(0, 360, 45):
                        v.append([r * math.cos(math.radians(a)),
                                  r * math.sin(math.radians(a)), zz])
                pts.append(_apply_origin(np.array(v), c.origin))
            elif hasattr(g, "size"):
                h = np.asarray(g.size, float) / 2.0
                v = np.array([[sx * h[0], sy * h[1], sz * h[2]]
                              for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)])
                pts.append(_apply_origin(v, c.origin))
    out = np.vstack(pts) if pts else np.zeros((0, 3))
    _CACHE[link_name] = out
    return out


def _apply_origin(v, origin):
    if origin is None:
        return v
    xyz = np.asarray(origin.xyz if origin.xyz else [0, 0, 0], float)
    rpy = origin.rpy if origin.rpy else [0, 0, 0]
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    R = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
         @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    return (R @ v.T).T + xyz


# --------------------------------------------------------------------- world
ARM_LINKS = ["base_link", "shoulder_link", "half_arm_1_link",
             "half_arm_2_link", "forearm_link", "spherical_wrist_1_link",
             "spherical_wrist_2_link", "bracelet_link"]
HAND_LINKS = ["robotiq_85_base_link",
              "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
              "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
              "robotiq_85_left_inner_knuckle_link",
              "robotiq_85_right_inner_knuckle_link",
              "robotiq_85_left_finger_tip_link",
              "robotiq_85_right_finger_tip_link"]


def body_points(fk, arm, q, grip=0.0, links=None):
    """{link: (N,3) world points} for the whole arm, hand included."""
    links = links if links is not None else (ARM_LINKS + HAND_LINKS)
    full = ["%s_%s" % (arm, ln) for ln in links]
    T = fk.poses(arm, q, links, gripper=grip)
    out = {}
    for name, M in zip(full, T):
        v = link_points(fk, name)
        if len(v):
            out[name] = (M[:3, :3] @ v.T).T + M[:3, 3]
    return out


def tool_extent(fk, arm, q, grip=0.0):
    """How far the HAND reaches past the wrist, measured from the meshes.

    Returns a dict with, all in metres:
      lowest_world_z   the lowest point of the whole hand, in world
      below_ee         how far that point is below the end-effector ORIGIN,
                       along the world vertical
      along_tool       the same distance projected on the TOOL AXIS, which is
                       the number a top-down descent must use
      pad_mid_along    where the pad midpoint sits along the tool axis, for
                       comparison with the 0.09833 m this repo quotes
    """
    pts = body_points(fk, arm, q, grip, links=HAND_LINKS)
    allp = np.vstack(list(pts.values()))
    Tee, Tl, Tr = fk.poses(arm, q,
                           ["end_effector_link",
                            "robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link"], gripper=grip)
    ee = Tee[:3, 3]
    axis = Tee[:3, 2]                                   # tool axis = EE +z
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    d_along = (allp - ee) @ axis
    lowest = allp[np.argmin(allp[:, 2])]
    return dict(
        n_points=int(len(allp)),
        lowest_world_z=float(allp[:, 2].min()),
        below_ee=float(ee[2] - allp[:, 2].min()),
        along_tool=float(d_along.max()),
        pad_mid_along=float((mid - ee) @ axis),
        # THE NUMBER A DESCENT ACTUALLY NEEDS: how far the fingers stick out
        # PAST the pad midpoint. Stopping with the pads at the cube centre
        # puts the finger TIPS this much lower, and that is what hits a table.
        tips_past_pads=float(d_along.max() - (mid - ee) @ axis))


#: The wearer's body parts, and the URDF link each one's primitives are
#: defined in. `clearance.parts_for` says it plainly: the origins "are offsets
#: within each body part's OWN LINK FRAME, TF places the frames".
WEARER_PART_LINKS = {
    "torso": "torso", "head": "head", "hips": "hips",
    "human_left_upper_arm": "human_left_upper_arm",
    "human_right_upper_arm": "human_right_upper_arm",
    "human_left_lower_arm": "human_left_lower_arm",
    "human_right_lower_arm": "human_right_lower_arm",
    "human_left_hand": "human_left_hand",
    "human_right_hand": "human_right_hand",
}


def wearer_clearance(fk, arm, q, grip=0.0, model=None, pad=0.0,
                     skip_static=True):
    """Smallest distance from ANY arm surface point to the wearer's body.

    THE POINTS ARE TRANSFORMED INTO EACH BODY PART'S OWN FRAME FIRST, AND
    THE FIRST VERSION OF THIS FUNCTION DID NOT DO THAT.
    ---------------------------------------------------------------------
    `ClearanceModel.clearance` takes `{part: [p, ...]}` with the points
    ALREADY EXPRESSED IN THAT PART'S LINK FRAME -- `clearance.parts_for`
    states it: the primitive origins are offsets inside each part's own link
    frame and TF is what places the frames. This function handed it RAW WORLD
    COORDINATES, so it compared world points against primitives sitting at
    link-local origins. The result was a number, it looked plausible, and it
    meant nothing.

    WHAT THAT COST, measured 2026-08-26: the scan pose was scored at 0.597 m
    of clearance -- four times the 150 mm floor -- and the arm HIT THE
    MANNEQUIN when it was commanded. The tell was there and was dismissed:
    six very different candidate poses all returned clearance 0.340, and a
    statistic that does not move when the input moves is this repository's own
    first-listed instrument failure.

    So this is the same defect as the one it was written to fix. The original
    `ClearanceModel` was blind BETWEEN the joints; this was blind about WHERE
    THE BODY IS. Neither is a safety check.
    """
    if model is None:
        sys.path.insert(0, "%s/src/srl_teleop" % WS)
        from srl_teleop.clearance import ClearanceModel
        model = ClearanceModel()
    pts = body_points(fk, arm, q, grip)
    # Where each body part actually is, right now, in world.
    part_T = {}
    for part, link in WEARER_PART_LINKS.items():
        if part not in model.PARTS or link not in fk.links:
            continue
        try:
            part_T[part] = fk.poses(arm, q, [link])[0]
        except Exception:                                     # noqa: BLE001
            continue
    if not part_T:
        raise RuntimeError(
            "no wearer link could be placed from the URDF -- refusing to "
            "report a clearance, because a body with no position is not a "
            "body the arm can be checked against")
    best, worst_link, worst_part = float("inf"), None, None
    for link, P in pts.items():
        if skip_static and link.endswith("_base_link"):
            # THE MOUNT IS NOT A CLEARANCE RESULT.
            #
            # `base_link` is bolted to the frame: CLAUDE.md records it sitting
            # a fixed distance from the torso with NO JOINT ABLE TO MOVE IT,
            # and warns that a reading of exactly that number is "the MOUNT,
            # not the arm". Including it means every pose reports the same
            # binding pair, the number cannot be improved by any solve, and a
            # margin larger than the mount's own offset is unsatisfiable by
            # construction -- which is what made the planner refuse every
            # candidate. It is checked against the hard FLOOR separately by
            # `mount_clearance`, where a change in it means the rig has been
            # rebuilt, not that a pose is bad.
            continue
        for part, T in part_T.items():
            # world -> this part's own frame
            R, t = T[:3, :3], T[:3, 3]
            local = (R.T @ (P - t).T).T
            d, _ = model.clearance({part: local}, pad=pad)
            if d < best:
                best, worst_link, worst_part = d, link, part
    return best, worst_link, worst_part


def mount_clearance(fk, arm, model=None):
    """The MOUNT's own distance to the body. Constant by construction.

    Reported separately and never mixed into a pose score: it is a property
    of how the rig is bolted together, so a change here means somebody moved
    the mount, not that a pose is unsafe.
    """
    return wearer_clearance(fk, arm, np.zeros(7), 0.0, model, skip_static=False)



# ============================================================================
def _self_test():
    fk = FK()
    fails = []
    n = [0]

    def chk(name, cond, detail=""):
        n[0] += 1
        print("  %-56s %s   %s" % (name, "PASS" if cond else "FAIL", detail))
        if not cond:
            fails.append(name)

    print("arm geometry from the collision meshes")
    print("-" * 74)
    v = link_points(fk, "left_robotiq_85_left_finger_tip_link")
    chk("the finger-tip mesh loads at all", len(v) > 0, "%d points" % len(v))

    q = np.radians([66.28, 56.30, 127.90, -94.32, -153.78, -64.41, -6.84])
    pts = body_points(fk, "left", q, 0.0)
    chk("the whole arm has surface points",
        len(pts) >= 12, "%d links with geometry" % len(pts))

    # The hand must stick out PAST the pad midpoint, or a descent that stops
    # at the pads would be safe by accident.
    t = tool_extent(fk, "left", q, 0.0)
    chk("the pad midpoint matches the FK value this repo quotes",
        abs(t["pad_mid_along"] - 0.09833) < 0.002,
        "%.5f m vs 0.09833" % t["pad_mid_along"])
    chk("the finger tips reach PAST the pad midpoint",
        t["tips_past_pads"] > 0.0,
        "%.1f mm past" % (t["tips_past_pads"] * 1000))

    # Opening the hand changes where the tips are -- the four-bar swing.
    a = tool_extent(fk, "left", q, 0.0)["along_tool"]
    b = tool_extent(fk, "left", q, 0.447)["along_tool"]
    chk("the hand's reach changes with the opening (four-bar)",
        abs(a - b) > 0.001, "%.1f mm open vs %.1f mm at 40 mm grip"
        % (a * 1000, b * 1000))

    # THE CONTROL THAT MATTERS: geometry must see something origins cannot.
    sys.path.insert(0, "%s/src/srl_teleop" % WS)
    from srl_teleop.clearance import ClearanceModel
    model = ClearanceModel()
    surf, link, part = wearer_clearance(fk, "left", q, 0.0, model)
    orig = {}
    T = fk.poses("left", q, ARM_LINKS + HAND_LINKS, gripper=0.0)
    for name, M in zip(ARM_LINKS + HAND_LINKS, T):
        orig.setdefault("torso", []).append(M[:3, 3])
    o, _ = model.clearance(orig)
    print("     surface-based %.4f m (%s vs %s) | origin-based %.4f m"
          % (surf, link, part, o))
    chk("the surface check is TIGHTER than the origin check",
        surf <= o + 1e-9, "%.4f <= %.4f" % (surf, o))

    print("-" * 74)
    print("%d checks, %d failed" % (n[0], len(fails)))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--tool", action="store_true")
    ap.add_argument("--clearance", action="store_true")
    ap.add_argument("--arm", default="left")
    a = ap.parse_args()
    if a.self_test:
        return _self_test()
    fk = FK()
    q = np.radians([66.28, 56.30, 127.90, -94.32, -153.78, -64.41, -6.84]) \
        if a.arm == "left" else \
        np.radians([122.05, -52.58, 59.56, -93.98, 160.08, -39.96, 159.97])
    if a.tool:
        print("TOOL EXTENT (%s arm, at home)" % a.arm)
        print("%-10s %10s %12s %12s %12s"
              % ("grip_rad", "gap_mm", "along_tool", "pad_mid", "tips_past"))
        for g in (0.0, 0.2, 0.447, 0.6, 0.8):
            t = tool_extent(fk, a.arm, q, g)
            print("%-10.3f %10s %12.5f %12.5f %12.2f"
                  % (g, "--", t["along_tool"], t["pad_mid_along"],
                     t["tips_past_pads"] * 1000))
        print("\n'tips_past' is how far the finger TIPS reach past the pad")
        print("midpoint, in mm. A descent that stops with the pads at the")
        print("object centre puts the tips that much lower.")
    if a.clearance:
        d, link, part = wearer_clearance(fk, a.arm, q, 0.0)
        print("wearer clearance (%s at home): %.4f m  worst %s vs %s"
              % (a.arm, d, link, part))
    return 0


if __name__ == "__main__":
    sys.exit(main())
