#!/usr/bin/env python3
"""STEP -> per-link STL, at two resolutions.

LINKS ARE CUT FROM THE SOLIDS BY HEIGHT, because the model has no link
structure to read: one solid per printed part, no articulation. Each solid is
assigned to the link whose z-range contains its centroid, using the inferred
joint heights. That is an inference like everything else here, and a solid
straddling a joint goes to the link it is mostly in.

TWO RESOLUTIONS, BOTH KEPT. The visual mesh is tessellated fine (0.05 mm
deflection) and the collision mesh coarse (1.5 mm) -- OCCT's own deflection
parameter is the decimation, so no second tool and no second geometry kernel
is involved. The full-resolution meshes are kept alongside, never replaced.

    .cad_venv/bin/python scripts/cad_export_meshes.py
"""
import os
import sys

CAD = "/mnt/c/Users/Gausms/Desktop/MSc_Project/CAD"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH = os.path.join(ROOT, "src/srl_description/meshes/master_arm_cad")

# Inferred joint heights, mm. See docs/system/09_cad_vs_measured.md for how
# each was obtained and what confidence it carries.
JOINT_Z = [23.0, 60.7, 103.0, 139.7, 180.0, 218.7, 260.0]


def load(path):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(path)
    r.TransferRoots()
    return r.OneShape()


def solids(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    out, exp = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def com_z(s):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, p)
    return p.CentreOfMass().Z()


def write_stl(shapes, path, deflection):
    """Fuse-free STL: tessellate each solid and concatenate the triangles.

    A boolean fuse of 75 printed solids is slow and can fail on coincident
    faces; for a display and collision mesh the union is not needed, only the
    triangles.
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    tris = []
    for sh in shapes:
        # CLEAR THE CACHED TRIANGULATION FIRST. BRepMesh_IncrementalMesh
        # stores its result ON THE SHAPE, so a second call at a coarser
        # deflection sees an existing mesh and returns it unchanged. The first
        # version of this script produced collision meshes with byte-identical
        # size to the visual ones -- "0% decimated" across the board, which is
        # what gave it away.
        BRepTools.Clean_s(sh)
        BRepMesh_IncrementalMesh(sh, deflection, False, 0.5, True)
        exp = TopExp_Explorer(sh, TopAbs_FACE)
        while exp.More():
            f = TopoDS.Face_s(exp.Current())
            exp.Next()
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(f, loc)
            if tri is None:
                continue
            tr = loc.Transformation()
            nodes = [tri.Node(i + 1).Transformed(tr)
                     for i in range(tri.NbNodes())]
            for i in range(tri.NbTriangles()):
                t = tri.Triangle(i + 1)
                a, b, c = t.Get()
                tris.append((nodes[a - 1], nodes[b - 1], nodes[c - 1]))
    # BINARY STL. The ASCII form of this set came to 64 MB, which is not a
    # thing to put in a git repository when the binary form is a fifth of it
    # and every consumer reads both.
    import struct
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            # REAL FACE NORMALS, not zeros. A zero normal is legal STL and
            # most tools recompute from the winding -- Ogre (RViz) does not,
            # and rendered the whole arm matte black. Cross product of the
            # two edges, normalised.
            ux, uy, uz = b.X() - a.X(), b.Y() - a.Y(), b.Z() - a.Z()
            vx, vy, vz = c.X() - a.X(), c.Y() - a.Y(), c.Z() - a.Z()
            nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz,
                          ux * vy - uy * vx)
            m = (nx * nx + ny * ny + nz * nz) ** 0.5
            if m > 0:
                nx, ny, nz = nx / m, ny / m, nz / m
            fh.write(struct.pack("<3f", nx, ny, nz))
            for p in (a, b, c):
                # mm -> m
                fh.write(struct.pack("<3f", p.X() / 1000.0,
                                     p.Y() / 1000.0, p.Z() / 1000.0))
            fh.write(struct.pack("<H", 0))
    return len(tris)


def main():
    os.makedirs(MESH, exist_ok=True)
    shape = load(os.path.join(CAD, "PotArm.step"))
    sl = solids(shape)
    bounds = [-1e9] + JOINT_Z + [1e9]
    groups = {i: [] for i in range(len(bounds) - 1)}
    for s in sl:
        z = com_z(s)
        for i in range(len(bounds) - 1):
            if bounds[i] <= z < bounds[i + 1]:
                groups[i].append(s)
                break
    names = ["base"] + ["link%d" % (i + 1) for i in range(len(JOINT_Z))]
    print("%-8s %7s   %10s  %10s" % ("link", "solids", "visual tri",
                                     "collision tri"))
    total = 0
    for i, nm in enumerate(names):
        g = groups.get(i, [])
        if not g:
            print("  %-6s %7d   %10s  %10s" % (nm, 0, "-", "-"))
            continue
        nv = write_stl(g, os.path.join(MESH, "%s.stl" % nm), 0.05)
        nc = write_stl(g, os.path.join(MESH, "%s_collision.stl" % nm), 1.5)
        total += nv
        print("  %-6s %7d   %10d  %10d  (%.0f%% decimated)"
              % (nm, len(g), nv, nc, 100.0 * (1 - nc / max(1, nv))))
    # the backpack too
    bp = load(os.path.join(CAD, "backpack.step"))
    nv = write_stl([bp], os.path.join(MESH, "backpack.stl"), 0.05)
    nc = write_stl([bp], os.path.join(MESH, "backpack_collision.stl"), 1.5)
    print("  %-6s %7s   %10d  %10d  (%.0f%% decimated)"
          % ("backpack", "-", nv, nc, 100.0 * (1 - nc / max(1, nv))))
    print("\n-> %s" % MESH)
    return 0


if __name__ == "__main__":
    sys.exit(main())
