"""MasterArmJoints -- a Fusion 360 script.

Adds the seven revolute joints to the master-arm design so the arm can be
posed by dragging, or animated (right-click a joint -> Animate Joint). Run it
INSIDE Fusion with "Complete Master Arm" open:

    Utilities tab -> ADD-INS -> Scripts and Add-Ins -> Scripts -> "+" (green)
    -> choose the folder MasterArmJoints -> Run

What it does, and the assumption behind each step:

  1. Finds the seven potentiometers (components whose name contains
     "resist" or "pot"). Each pot sits exactly on a joint axis: its shaft is
     the axis of rotation. Its largest circular edge whose normal lies along
     the pot's thin direction gives the joint origin and axis.
  2. Orders the pots along the arm and sorts every other component into a
     LINK by where its centre lies between consecutive pots (base = link 0,
     handle = link 7). A pot belongs to the link below it. Anything far off
     the arm axis (electronics box, backpack) goes with the base.
  3. Makes each link a rigid group, grounds the base, and creates an
     as-built revolute joint J1..J7 between consecutive links at the pot
     axis, limited to +/-150 degrees (a 9 mm pot's mechanical travel).
  4. Bodies that live directly in the root component are first turned into
     components, because joints need components.

Nothing is moved and no geometry is edited; the design gets 8 rigid groups
and 7 joints, all of which you can edit or delete in the browser. If a part
lands in the wrong link, drag it out of that rigid group and into the right
one. If the pot search finds anything other than seven pots the script
stops and says so before changing anything.
"""
import math
import traceback

import adsk.core
import adsk.fusion

POT_NAMES = ("resist", "pot")
LIMIT_DEG = 150.0
OFF_AXIS_CM = 8.0   # further than this from the arm axis -> base group


def _v(p):
    return (p.x, p.y, p.z)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 1.0)


def _centre(bb):
    return ((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
            (bb.minPoint.z + bb.maxPoint.z) / 2)


def _thin_axis(bb):
    d = (bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z)
    i = min(range(3), key=lambda k: d[k])
    return tuple(1.0 if k == i else 0.0 for k in range(3))


def leaf_occurrences(root):
    out = []
    for occ in root.allOccurrences:
        if occ.childOccurrences.count == 0:
            out.append(occ)
    return out


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open the master arm design first.")
            return
        root = design.rootComponent

        # 4. root-level bodies -> components, so they can be jointed
        made = 0
        for i in range(root.bRepBodies.count - 1, -1, -1):
            root.bRepBodies.item(i).createComponent()
            made += 1

        occs = leaf_occurrences(root)
        pots = [o for o in occs if any(k in o.component.name.lower() for k in POT_NAMES)]
        if len(pots) != 7:
            ui.messageBox("Expected 7 potentiometer components (name containing 'resist' or 'pot'), found %d:\n%s\n\n"
                          "Nothing changed. Rename the pots or edit POT_NAMES at the top of the script."
                          % (len(pots), "\n".join(o.name for o in pots)))
            return

        # 1. joint axis and origin from each pot's own shaft circle
        joints_geo = []
        for pot in pots:
            bb = pot.boundingBox
            c = _centre(bb)
            thin = _thin_axis(bb)
            best = None
            for body in pot.bRepBodies:
                for e in body.edges:
                    g = e.geometry
                    if g.objectType != adsk.core.Circle3D.classType():
                        continue
                    n = _norm(_v(g.normal))
                    if abs(_dot(n, thin)) < 0.95:
                        continue
                    if best is None or g.radius > best[0]:
                        best = (g.radius, e, _v(g.center), n)
            if best is None:
                ui.messageBox("No circular edge along the thin axis of %s; cannot place its joint. Nothing changed."
                              % pot.name)
                return
            joints_geo.append({"pot": pot, "centre": best[2], "axis": best[3], "edge": best[1], "bbc": c})

        # order along the arm: direction = farthest pair of pot centres
        cs = [j["centre"] for j in joints_geo]
        far = max(((a, b) for a in cs for b in cs), key=lambda ab: _dot(_sub(ab[0], ab[1]), _sub(ab[0], ab[1])))
        axis_dir = _norm(_sub(far[1], far[0]))
        origin = far[0]
        # base end = the end nearer the largest cluster of non-pot mass; assume the
        # end with the most components within 5 cm is the base
        others = [o for o in occs if o not in pots]
        proj = lambda p: _dot(_sub(p, origin), axis_dir)
        lo_end, hi_end = min(proj(c) for c in cs), max(proj(c) for c in cs)
        n_lo = sum(1 for o in others if proj(_centre(o.boundingBox)) < lo_end)
        n_hi = sum(1 for o in others if proj(_centre(o.boundingBox)) > hi_end)
        if n_hi > n_lo:  # flip so the base is at the low end
            axis_dir = (-axis_dir[0], -axis_dir[1], -axis_dir[2])
        joints_geo.sort(key=lambda j: proj(j["centre"]))
        jpos = [proj(j["centre"]) for j in joints_geo]

        # 2. sort every component into a link
        links = [[] for _ in range(8)]
        for j, jg in enumerate(joints_geo):
            links[j].append(jg["pot"])          # pot belongs to the link below its joint
        for o in others:
            c = _centre(o.boundingBox)
            p = proj(c)
            d = _sub(c, origin)
            along = tuple(axis_dir[k] * p for k in range(3))
            perp = math.sqrt(max(_dot(_sub(d, along), _sub(d, along)), 0.0))
            if perp > OFF_AXIS_CM:
                links[0].append(o)
                continue
            k = sum(1 for q in jpos if p >= q)
            links[k].append(o)
        empty = [i for i, L in enumerate(links) if not L]
        if empty:
            ui.messageBox("Links with no components: %s. Nothing changed. Check the design is one arm on its base."
                          % empty)
            return

        # 3. rigid groups, ground, joints
        groups = []
        for i, L in enumerate(links):
            coll = adsk.core.ObjectCollection.create()
            for o in L:
                coll.add(o)
            rg = root.rigidGroups.add(coll, True)
            rg.name = "link%d" % i
            groups.append(rg)
        for o in links[0]:
            try:
                o.isGrounded = True
            except Exception:
                pass

        made_joints = []
        for i in range(7):
            below, above = links[i][0], links[i + 1][0]
            geo = adsk.fusion.JointGeometry.createByCurve(joints_geo[i]["edge"],
                                                         adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            inp = root.asBuiltJoints.createInput(below, above, geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            jt = root.asBuiltJoints.add(inp)
            jt.name = "J%d" % (i + 1)
            try:
                lim = jt.jointMotion.rotationLimits
                lim.isMinimumValueEnabled = True
                lim.minimumValue = -math.radians(LIMIT_DEG)
                lim.isMaximumValueEnabled = True
                lim.maximumValue = math.radians(LIMIT_DEG)
            except Exception:
                pass
            made_joints.append(jt.name)

        ui.messageBox("Done.\n\n%d root bodies made into components.\n8 rigid groups link0..link7 (%s parts).\n"
                      "Joints: %s, each +/-%g deg.\n\nDrag the arm, or right-click a joint in the browser -> "
                      "Animate Joint. If a part moves with the wrong link, move it between the rigid groups."
                      % (made, ", ".join(str(len(L)) for L in links), ", ".join(made_joints), LIMIT_DEG))
    except Exception:
        if ui:
            ui.messageBox("MasterArmJoints failed:\n" + traceback.format_exc())
