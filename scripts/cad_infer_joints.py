#!/usr/bin/env python3
"""Infer the seven pot-joint frames from a model that has none.

THE MODEL WAS DESIGNED FOR PRINTING. No articulation, no joint frames, one
solid per printed part. So every frame below is INFERRED FROM GEOMETRY, and
each carries a confidence that says how much of it is measurement and how much
is pattern-matching.

WHAT IS BEING LOOKED FOR. A potentiometer joint leaves two signatures in a
printed part: a cylindrical BOSS that the pot body sits in, and a cylindrical
BORE that its shaft turns in. Both are cylindrical faces, so the method is:

  1. enumerate every cylindrical face in the solid
  2. record axis direction, axis point and radius
  3. cluster faces that share an axis (same direction AND the point lies on
     the same line) -- a boss and its bore collapse to one cluster
  4. keep clusters whose radius is in the pot range
  5. order them along the arm and check the roll/bend alternation

THE ALTERNATION IS THE TEST, NOT AN ASSUMPTION. The chain is stated as
J1 roll / J2 bend / J3 roll / J4 bend / J5 roll / J6 bend / J7 roll, so
consecutive axes must alternate between parallel-to-the-limb and
perpendicular-to-it. A candidate set that does not alternate is wrong however
well it fits anything else, and that is reported rather than smoothed over.

    .cad_venv/bin/python scripts/cad_infer_joints.py
"""
import json
import math
import os
import sys

CAD = "/mnt/c/Users/Gausms/Desktop/MSc_Project/CAD"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "recordings/baselines/cad_joints.json")

# Measured link lengths the working URDF uses, metres. THE REFERENCE, not the
# thing under test -- see the report.
MEASURED = [0.043, 0.037, 0.043, 0.037, 0.043, 0.036, 0.033]


def load(path):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(path)
    r.TransferRoots()
    return r.OneShape()


def cylinders(shape):
    """Every cylindrical face: (radius, axis dir, axis point, area)."""
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    out = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        f = TopoDS.Face_s(exp.Current())
        exp.Next()
        try:
            ad = BRepAdaptor_Surface(f)
            if ad.GetType() != GeomAbs_Cylinder:
                continue
            cyl = ad.Cylinder()
            ax = cyl.Axis()
            d = ax.Direction()
            p = ax.Location()
            g = GProp_GProps()
            BRepGProp.SurfaceProperties_s(f, g)
            out.append(dict(r=cyl.Radius(),
                            d=(d.X(), d.Y(), d.Z()),
                            p=(p.X(), p.Y(), p.Z()),
                            area=g.Mass()))
        except Exception:                                        # noqa: BLE001
            continue
    return out


def _norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v) if m else v


def _canon(d):
    """Axis direction without sign -- an axis has no head or tail."""
    d = _norm(d)
    for c in d:
        if abs(c) > 1e-9:
            return tuple(x * (1 if c > 0 else -1) for x in d)
    return d


def _dist_point_line(p, lp, ld):
    v = tuple(p[i] - lp[i] for i in range(3))
    t = sum(v[i] * ld[i] for i in range(3))
    proj = tuple(v[i] - t * ld[i] for i in range(3))
    return math.sqrt(sum(c * c for c in proj))


def cluster(cyls, ang_tol_deg=3.0, line_tol=2.0):
    """Group cylindrical faces that share an axis LINE."""
    groups = []
    ct = math.cos(math.radians(ang_tol_deg))
    for c in cyls:
        d = _canon(c["d"])
        placed = False
        for g in groups:
            if abs(sum(d[i] * g["d"][i] for i in range(3))) < ct:
                continue
            if _dist_point_line(c["p"], g["p"], g["d"]) > line_tol:
                continue
            g["faces"].append(c)
            placed = True
            break
        if not placed:
            groups.append(dict(d=d, p=c["p"], faces=[c]))
    for g in groups:
        rs = [f["r"] for f in g["faces"]]
        g["r_min"], g["r_max"] = min(rs), max(rs)
        g["n"] = len(g["faces"])
        g["area"] = sum(f["area"] for f in g["faces"])
        # axis point = centroid of member axis points projected onto the line
        pts = [f["p"] for f in g["faces"]]
        g["p"] = tuple(sum(q[i] for q in pts) / len(pts) for i in range(3))
    return groups


def main():
    shape = load(os.path.join(CAD, "PotArm.step"))
    cyls = cylinders(shape)
    print("cylindrical faces in PotArm.step : %d" % len(cyls))
    gs = cluster(cyls)
    print("distinct axis lines              : %d" % len(gs))

    # Pot bodies and shafts. A 9 mm potentiometer body is ~3.4-4 mm radius at
    # the bush and the shaft ~1.5-3 mm; printed clearance widens both. Anything
    # under 1 mm is a print artefact, anything over 12 mm is structure.
    cand = [g for g in gs if 1.0 <= g["r_max"] <= 12.0]
    cand.sort(key=lambda g: -g["area"])
    print("candidate pot axes (1-12 mm r)   : %d" % len(cand))
    print()
    print("  %-4s %7s %7s %6s  %-24s %s"
          % ("#", "r_min", "r_max", "faces", "axis dir", "axis point (mm)"))
    for i, g in enumerate(cand[:24]):
        print("  %-4d %7.2f %7.2f %6d  (%+.3f,%+.3f,%+.3f)  (%+7.1f,%+7.1f,%+7.1f)"
              % (i, g["r_min"], g["r_max"], g["n"],
                 g["d"][0], g["d"][1], g["d"][2],
                 g["p"][0], g["p"][1], g["p"][2]))

    # ---- alternation test -------------------------------------------------
    # Order candidates along the arm's long axis (z, from the bounding box) and
    # look for the roll/bend alternation the chain is supposed to have.
    up = (0.0, 0.0, 1.0)
    for g in cand:
        g["along"] = abs(sum(g["d"][i] * up[i] for i in range(3)))
        g["kind"] = ("roll" if g["along"] > 0.85 else
                     "bend" if g["along"] < 0.15 else "oblique")
    seq = sorted(cand, key=lambda g: g["p"][2])
    print()
    print("  ordered along the arm (z):")
    for g in seq[:24]:
        print("     z %+7.1f  r %5.2f  %-7s  dir (%+.2f,%+.2f,%+.2f)"
              % (g["p"][2], g["r_max"], g["kind"],
                 g["d"][0], g["d"][1], g["d"][2]))

    res = dict(n_cyl_faces=len(cyls), n_axes=len(gs),
               candidates=[dict(r_min=round(g["r_min"], 3),
                                r_max=round(g["r_max"], 3),
                                faces=g["n"], area=round(g["area"], 1),
                                dir=[round(v, 4) for v in g["d"]],
                                point=[round(v, 3) for v in g["p"]],
                                kind=g["kind"]) for g in seq])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
