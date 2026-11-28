"""Unpacked geometry, RBD Point Objects, collision masks, collision shapes and Static RBD Objects."""

import hou
from b3dtest import (
    GROUND,
    four_points_at_height,
    Result,
    SetupError,
    box3d_data,
    box3d_packed_object,
    box3d_solver,
    rbd_object,
    whole_object_points,
    close,
    create,
    display,
    finish,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    point_attributes,
    set_parm,
    single_object_geo,
    sop_solver,
    step,
)


# ---------------------------------------------------------------------------
# collision_category / collision_mask / collision_group (docs/attributes.md's collision rules)
# ---------------------------------------------------------------------------


def test_collision_masks():
    """The collision rules in docs/attributes.md on packed pieces: a mask of -1 collides with everything, a mask
    that misses the other's category passes through, a negative shared group never collides."""
    cases = {
        # name: (falling box attributes, expected to land)
        "mask -1": ({"collision_category": 2, "collision_mask": -1, "collision_group": 0}, True),
        "mask misses category": ({"collision_category": 2, "collision_mask": 4, "collision_group": 0}, False),
        "shared negative group": ({"collision_category": 2, "collision_mask": -1, "collision_group": -1}, False),
        "shared positive group, masks miss": ({"collision_category": 2, "collision_mask": 4, "collision_group": 1}, True),
    }
    rows, bad = [], []
    for label, (attrs, should_land) in cases.items():
        ground = dict(GROUND)
        ground_group = attrs["collision_group"]
        ground["attrs"] = {"active": 0, "collision_category": 1, "collision_mask": -1, "collision_group": ground_group}
        box = {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1, **attrs}}
        path, names = packed_boxes("masks_" + label.replace(" ", "_").replace(",", "").replace("-", "m"), [ground, box])
        dop = new_dopnet("masks_" + str(len(rows)))
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
        y = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))[48][names[1]][1]
        landed = y > 0.3
        rows.append(f"{label}: {'landed' if landed else 'passed through'} (y={y:.2f})")
        if landed != should_land:
            bad.append(label)
    return [Result("collision_masks", Result.FAIL if bad else Result.PASS,
                   "; ".join(rows) + (f". Wrong: {', '.join(bad)}" if bad else ""))]


# Unpacked geometry on a points object (Box3D Packed Object / RBD Packed Object): as RBD Bullet Solver,
# an error when nothing is packed; packed pieces beside loose polygons, the polygons' points get no body.

UNSUPPORTED = "primitive types supported"


def _unpacked_geometry(name, kind):
    """A SOP object for the unpacked-geometry tests: 'polygons' (an unpacked polygon box at y = 3),
    'mixed' (a packed box at x = -2 and an unpacked one at x = 2, both at y = 3) or 'points' (two
    loose points at y = 3). Returns its SOP path."""
    geo = new_geo(name)
    if kind == "packedpoints":  # a packed box at x = -2 and two loose points at y = 3
        pack = create(geo, "pack", "pack")
        box = create(geo, "box", "box_packed")
        xf = create(geo, "xform", "place_packed")
        xf.setInput(0, box)
        set_parm(xf, ["t"], (-2.0, 3.0, 0.0))
        pack.setInput(0, xf)
        wr = create(geo, "attribwrangle", "points")
        wr.setInput(0, pack)
        set_parm(wr, ["class"], 0)  # detail
        set_parm(wr, ["snippet"], "addpoint(0, {1, 3, 0}); addpoint(0, {3, 3, 0});")
        display(wr)
        return wr.path()
    if kind == "points":
        wr = create(geo, "attribwrangle", "points")
        set_parm(wr, ["class"], 0)  # detail
        set_parm(wr, ["snippet"], "addpoint(0, {-1, 3, 0}); addpoint(0, {1, 3, 0});")
        display(wr)
        return wr.path()

    def box(tag, x):
        b = create(geo, "box", f"box_{tag}")
        xf = create(geo, "xform", f"place_{tag}")
        xf.setInput(0, b)
        set_parm(xf, ["t"], (x, 3.0, 0.0))
        return xf

    if kind == "polygons":
        out = box("unpacked", 0.0)
    else:
        pack = create(geo, "pack", "pack")
        pack.setInput(0, box("packed", -2.0))
        out = create(geo, "merge", "merge")
        out.setInput(0, pack)
        out.setInput(1, box("unpacked", 2.0))
    display(out)
    return out.path()


def _lowest_y(sim, obj_name):
    """{'packed': the lowest packed point's y, 'other': the lowest other point's y} of the object."""
    obj = sim.findObject(obj_name)
    geo = obj.geometry() if obj is not None else None
    if geo is None:
        return None
    packed = set()
    for prim in geo.prims():
        if isinstance(prim, hou.PackedPrim):
            packed.update(v.point().number() for v in prim.vertices())
    out = {}
    for pt in geo.points():
        key = "packed" if pt.number() in packed else "other"
        out[key] = min(out.get(key, 1e9), pt.position()[1])
    return out


def _unpacked_run(tag, kind, bullet):
    """Drops the `kind` geometry for 1 s with a Box3D Packed Object and the Box3D Solver (bullet: an
    RBD Packed Object and RBD Bullet Solver), beside a valid packed box object. Returns (lowest y
    per kind of point at frame 1 and 25, the valid box's y at 25, the solver's and object's
    messages)."""
    path = _unpacked_geometry(f"unpacked_{tag}", kind)
    good_path, good_names = packed_boxes(f"unpacked_{tag}_good", [{"size": (1, 1, 1), "center": (5, 3, 0)}])
    dop = new_dopnet(f"unpacked_{tag}")
    if bullet:
        obj = packed_object(dop, "pieces", path)
        good = packed_object(dop, "good", good_path)
        solver_node = create(dop, "bulletrbdsolver", "bullet_solver")
        solver_node.setInput(0, merge(dop, [obj, good]))
        last = create(dop, "gravity", "gravity")
        last.setInput(0, solver_node)
    else:
        obj = box3d_packed_object(dop, "pieces", path)
        good = box3d_packed_object(dop, "good", good_path)
        solver_node = last = box3d_solver(dop, merge(dop, [obj, good]))
    finish(dop, last)
    messages = set()

    def sample(sim, f):
        for n in (solver_node, obj):
            messages.update(node_messages(n))
        good_pt = point_attributes(sim, "good", good_names[0], ["P"])
        return _lowest_y(sim, "pieces"), good_pt["P"][1] if good_pt else None

    try:
        track = step(dop, 25, sample)
    except hou.Error as e:
        return None, None, None, [f"cook failed: {e}"]
    return track[1][0], track[25][0], track[25][1], sorted(messages)


def test_unpacked_geometry():
    """Box3D, as RBD Bullet Solver: polygons with nothing packed are a warning naming the object
    ("Geometry does not contain any primitive types supported by the solver") and get no bodies
    (the box stays at y = 2.5), while a valid packed box in the same solver still falls to its
    first-second y (3 - g/2, about -1.9); packed pieces beside loose polygons fall, the polygons'
    points stay, with a warning; loose points (no primitives) fall, with no message (a Box3D extension: Bullet gives the warning)."""
    results = []
    rows, ok = [], True
    for kind in ("polygons", "mixed", "points"):
        y1, y25, good_y, msgs = _unpacked_run(f"ours_{kind}", kind, bullet=False)
        unsupported = any(UNSUPPORTED in m for m in msgs)
        warned = any("not packed" in m for m in msgs)
        fell = good_y is not None and good_y < 0.0
        if y25 is None:
            this_ok = False
        elif kind == "polygons":
            this_ok = unsupported and close(y25.get("other", 0), 2.5, 1e-4) and fell
        elif kind == "mixed":
            this_ok = (not unsupported and warned and y25.get("packed", 9) < 0.0 and close(y25.get("other", 0), 2.5, 1e-4)
                       and fell)
        else:
            this_ok = not unsupported and not warned and y25.get("other", 9) < 0.0 and fell
        ok = ok and this_ok
        rows.append(f"{kind}: lowest y {y1} -> {y25}, valid box y={good_y}, unsupported-geometry warning {unsupported}, unpacked-points warning {warned}"
                    f"{'' if this_ok else ' (FAIL) messages ' + repr(msgs)}")
    results.append(Result("unpacked_geometry", Result.PASS if ok else Result.FAIL, "; ".join(rows)))
    return results


def test_unpacked_geometry_bullet():
    """RBD Bullet Solver with the same three inputs (polygons only, packed + polygons, loose points),
    recorded: which messages it gives and which points move, for test unpacked_geometry to match."""
    rows = []
    for kind in ("polygons", "mixed", "points"):
        y1, y25, good_y, msgs = _unpacked_run(f"bullet_{kind}", kind, bullet=True)
        rows.append(f"{kind}: lowest y {y1} -> {y25}, valid box y={good_y}, messages {msgs or 'none'}")
    return [Result("unpacked_geometry_bullet", Result.INFO, "; ".join(rows))]


def test_unpacked_geometry_match():
    """The same four inputs as unpacked_geometry (polygons only, packed + polygons, loose points, packed +
    loose points) through Box3D and RBD Bullet Solver, compared: for polygons only and packed + polygons the two
    agree on what falls (the packed points, the other points, the valid box) and on whether the
    unsupported-geometry message appears. Loose points are INFO: Box3D simulates them and Bullet is documented
    not to (status.md)."""
    results = []
    for kind in ("polygons", "mixed", "points", "packedpoints"):
        runs = {name: _unpacked_run(f"match_{name}_{kind}", kind, bullet=(name == "bullet"))
                for name in ("box3d", "bullet")}

        def summary(run):
            y1, y25, good_y, msgs = run
            if y25 is None:
                return None
            fell = {key: y < 0.0 for key, y in y25.items()}
            return fell, good_y is not None and good_y < 0.0, any(UNSUPPORTED in m for m in msgs)

        got = {name: summary(run) for name, run in runs.items()}
        same = got["box3d"] is not None and got["box3d"] == got["bullet"]
        status = Result.INFO if kind in ("points", "packedpoints") else (Result.PASS if same else Result.FAIL)
        results.append(Result("unpacked_geometry_match", status,
                              f"{kind}: (what fell, valid box fell, unsupported-geometry message) Box3D {got['box3d']}, "
                              f"Bullet {got['bullet']}; messages Box3D {runs['box3d'][3] or 'none'}, Bullet "
                              f"{runs['bullet'][3] or 'none'}"))
    return results


def _point_object_run(tag, bullet):
    """Drops four points (a 2 x 2 grid at y = 3) with an RBD Point Object, each instancing a unit
    box, for 1 s, with the Box3D Solver (bullet: RBD Bullet Solver). Returns (lowest y of each
    object with geometry at frame 1 and 25, the number of DOP objects at frame 25, their names and
    data, the solver's and object's messages including inside the asset)."""
    dop = new_dopnet(f"pointobj_{tag}")

    obj = create(dop, "rbdpointobject", "objects")
    set_parm(obj, ["soppath"], single_object_geo(f"pointobj_{tag}_geo", four_points_at_height))
    # Geometry Path is disabled (and ignored) while the Use / Override menu next to it is 0.
    set_parm(obj, ["usegeo"], 1)
    set_parm(obj, ["geopath", "instancepath", "geometrypath", "instance"],
             single_object_geo(f"pointobj_{tag}_box", lambda geo: create(geo, "box")))
    if bullet:
        solver_node = create(dop, "bulletrbdsolver", "bullet_solver")
        solver_node.setInput(0, obj)
        last = create(dop, "gravity", "gravity")
        last.setInput(0, solver_node)
    else:
        solver_node = last = box3d_solver(dop, obj)
    finish(dop, last)
    messages = set()

    def sample(sim, f):
        for n in [solver_node, obj] + list(obj.allSubChildren()):
            messages.update(f"{n.name()}: {m.strip()[:120]}" for m in node_messages(n))
        ys, names = [], []
        for o in sim.objects():
            names.append(f"{o.name()} {sorted(o.subData().keys())}")
            geo = o.geometry()
            pts = geo.points() if geo is not None else []
            if pts:
                xform = o.transform()
                ys.append(round(min((p.position() * xform)[1] for p in pts), 3))
        return sorted(ys), names

    try:
        track = step(dop, 25, sample)
    except hou.Error as e:
        return None, None, 0, [], [f"cook failed: {e}"]
    return track[1][0], track[25][0], len(track[25][1]), track[25][1][:4], sorted(messages)[:6]


def test_point_object_bullet():
    """RBD Point Object on RBD Bullet Solver and on the Box3D Solver, same four points and
    instance box, recorded side by side. Settles whether Bullet simulates an RBD Point Object's
    points. INFO only. (status.md: RBD Point Object made no objects in a scripted setup, so a
    result of 0 objects on both sides says nothing about either solver.)"""
    rows = []
    for label, bullet in (("bullet", True), ("box3d", False)):
        try:
            y1, y25, n, names, msgs = _point_object_run(label, bullet)
        except SetupError as e:
            rows.append(f"{label}: not available ({e})")
            continue
        fell = bool(y1) and bool(y25) and len(y1) == len(y25) and all(b < a - 0.5 for a, b in zip(y1, y25))
        rows.append(f"{label}: {n} DOP object(s) {names}, lowest y {y1} -> {y25}, fell {fell}, "
                    f"messages {msgs or 'none'}")
    return [Result("point_object_bullet", Result.INFO, "; ".join(rows))]


def shape_lines(static_georep):
    """Number of polygons in Export Collision Shapes for a static 4 x 0.4 x 4 piece with `static_georep` and a
    dynamic 1 m box (a hull: 6 faces), read back through a DOP Import of the `Box3DShapes` data."""
    pieces = [{"size": (4, 0.4, 4), "center": (0, 0, 0), "attrs": {"active": 0, "bullet_georep": static_georep}},
              {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1, "bullet_georep": "convexhull"}}]
    path, _ = packed_boxes(f"shapes_{static_georep}_pieces", pieces)
    dop = new_dopnet(f"shapes_{static_georep}")
    finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True)))
    set_parm(dop.node("box3d_solver"), ["exportshapes"], 1)
    geo = new_geo(f"shapes_{static_georep}_read")
    imp = create(geo, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "pieces")
    set_parm(imp, ["geodatapath"], "Box3DShapes")
    for f in (1, 2, 3):
        hou.setFrame(f)
    return len(imp.geometry().prims())


def test_collision_shapes_mesh():
    """Export Collision Shapes / the guide draw a static concave piece's triangle mesh: its 12 box edges (all
    creased; the flat quads' diagonals are left out), next to the dynamic box's 6 hull faces. A static convex
    piece gives its 6 hull faces instead."""
    mesh, hull = shape_lines("concave"), shape_lines("convexhull")
    ok = mesh == 18 and hull == 12
    return [Result("collision_shapes_mesh", Result.PASS if ok else Result.FAIL,
                   f"polygons drawn with a static concave piece {mesh} (expected 12 mesh edges + 6 = 18), with a "
                   f"static convex piece {hull} (expected 6 + 6 = 12)")]


def test_collision_shapes_follow():
    """Export Collision Shapes keeps its lines between steps and only moves them: three spinning boxes dropped on a
    ground, read back through a DOP Import of `Box3DShapes` every frame for 160 frames. The lines' bounding box is
    each body's own (its box corners turned by the packed rotation about its point, within 1e-3) on every frame, the polygon count never changes, the dynamic boxes are drawn at
    full brightness while they fall and every line is dimmed once they sleep."""
    pieces = [dict(GROUND, attrs={"active": 0})] + [
        {"size": (1, 1, 1), "center": (1.5 * i - 1.5, 2.0 + i, 0),
         "attrs": {"active": 1, "w": (1.0 + i, 2.0, 0.5), "v": (0.0, 0.0, 0.0)}} for i in range(3)]
    path, _ = packed_boxes("shapes_follow_pieces", pieces)
    dop = new_dopnet("shapes_follow")
    finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True)))
    set_parm(dop.node("box3d_solver"), ["exportshapes"], 1)
    geo = new_geo("shapes_follow_read")
    imp = create(geo, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "pieces")
    set_parm(imp, ["geodatapath"], "Box3DShapes")
    worst, counts, bright, dim_end = 0.0, set(), None, None
    corners = [(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)]
    for f in range(1, 161):
        hou.setFrame(f)
        lines = imp.geometry()
        body = dop.simulation().findObject("pieces").geometry()
        if f < 3 or not lines.prims():
            continue
        # Each body is 6 polygons; the exact bounds of every 1 m box (its corners through the packed transform)
        # must be the bounds of one of the groups.
        prims = list(lines.prims())
        groups = []
        for g in range(0, len(prims), 6):
            pts = [v.point().position() for pr in prims[g:g + 6] for v in pr.vertices()]
            groups.append([min(p[k] for p in pts) for k in range(3)] + [max(p[k] for p in pts) for k in range(3)])
        for pr in body.prims():
            m = pr.intrinsicValue("packedfulltransform")
            bounds = pr.intrinsicValue("bounds")
            if bounds[1] - bounds[0] > 5.0:  # the 20 m ground
                continue
            centre = pr.points()[0].position()
            world = [[sum(c[r] * m[4 * r + k] for r in range(3)) + centre[k] for k in range(3)] for c in corners]
            want = [min(p[k] for p in world) for k in range(3)] + [max(p[k] for p in world) for k in range(3)]
            worst = max(worst, min(max(abs(a - b) for a, b in zip(want, grp)) for grp in groups))
        counts.add(len(lines.prims()))
        shades = [p.attribValue("Cd")[0] for p in lines.prims()]
        if f == 3:
            bright = max(shades)
        dim_end = max(shades)
    ok = worst < 1e-3 and len(counts) == 1 and bright is not None and bright > 0.99 and dim_end is not None \
        and dim_end < 0.6
    return [Result("collision_shapes_follow", Result.PASS if ok else Result.FAIL,
                   f"largest bounding box difference from the bodies' own {worst:.2e} (expected under 1e-3), polygon "
                   f"counts {sorted(counts)} (expected one), brightest line {bright} on frame 3 (expected 1) and "
                   f"{dim_end} on frame 160 (expected under 0.6: every body asleep or static)")]


def test_collision_shapes_remove():
    """Export Collision Shapes after a body is deleted: a SOP Solver marks one of three falling boxes `dead` on frame
    20. The exported lines have 24 polygons (4 bodies) before it and 18 after, the remaining boxes' lines are
    where the boxes are (the corners of each, turned by its packed rotation, within 1e-3), and the dead one's lines
    are gone."""
    pieces = [dict(GROUND, attrs={"active": 0})] + [
        {"size": (1, 1, 1), "center": (1.5 * i - 1.5, 2.0 + i, 0), "attrs": {"active": 1, "w": (1.0, 2.0, 0.5)}}
        for i in range(3)]
    path, names = packed_boxes("shapes_remove_pieces", pieces)
    dop = new_dopnet("shapes_remove")
    objs = packed_object(dop, "pieces", path, keep_velocity=True)
    solver = box3d_solver(dop, None)
    set_parm(solver, ["exportshapes"], 1)
    kill = sop_solver(dop, f'if (@Frame >= 20 && s@name == "{names[2]}") i@dead = 1;')
    finish(dop, multisolver(dop, objs, [kill, solver]))
    geo = new_geo("shapes_remove_read")
    imp = create(geo, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "pieces")
    set_parm(imp, ["geodatapath"], "Box3DShapes")
    corners = [(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)]
    counts, worst = {}, 0.0
    for f in range(1, 41):
        hou.setFrame(f)
        lines = imp.geometry()
        body = dop.simulation().findObject("pieces").geometry()
        if f < 3 or not lines.prims():
            continue
        counts[f] = len(lines.prims())
        if 18 <= f <= 22:  # the frames where the body goes: the geometry and the lines are a step apart
            continue
        prims = list(lines.prims())
        groups = []
        for g in range(0, len(prims), 6):
            pts = [v.point().position() for pr in prims[g:g + 6] for v in pr.vertices()]
            groups.append([min(p[k] for p in pts) for k in range(3)] + [max(p[k] for p in pts) for k in range(3)])
        for pr in body.prims():
            if pr.intrinsicValue("bounds")[1] - pr.intrinsicValue("bounds")[0] > 5.0:
                continue
            m = pr.intrinsicValue("packedfulltransform")
            centre = pr.points()[0].position()
            world = [[sum(c[r] * m[4 * r + k] for r in range(3)) + centre[k] for k in range(3)] for c in corners]
            want = [min(p[k] for p in world) for k in range(3)] + [max(p[k] for p in world) for k in range(3)]
            worst = max(worst, min(max(abs(a - b) for a, b in zip(want, grp)) for grp in groups))
    before, after = counts.get(10), counts.get(40)
    ok = before == 24 and after == 18 and worst < 1e-3
    return [Result("collision_shapes_remove", Result.PASS if ok else Result.FAIL,
                   f"polygons on frame 10 {before} (expected 24) and frame 40 {after} (expected 18), largest "
                   f"difference between the remaining boxes' lines and their corners {worst:.2e} (expected under 1e-3)")]


def static_tray_rest(shape_index, static=True, body_type_expression=None):
    """The lowest point (y) of a sphere RBD Object dropped from y 3 into a static RBD Object tray: an open 4 x 2 x 4
    box (double sided, so its winding does not matter), its floor at y 0. `shape_index`: the tray's Box3D Data
    Default Shape (3 Concave, 2 Convex Hull). `body_type_expression`: Hscript for the tray's Body Type, read every step
    (0 static, 2 dynamic); the result is then the tray's and the ball's lowest points on frames 30, 60 and 96, and the
    solver's messages."""
    def tray(geo):
        box = create(geo, "box")
        set_parm(box, ["size"], (4, 2, 4))
        set_parm(box, ["t"], (0, 1, 0))
        open_top = create(geo, "attribwrangle")
        open_top.setInput(0, box)
        set_parm(open_top, ["class"], 1)
        set_parm(open_top, ["snippet"], 'vector c = primuv(0, "P", @primnum, {0.5, 0.5, 0}); if (c.y > 1.9) removeprim(0, @primnum, 1);')
        flip = create(geo, "reverse")
        flip.setInput(0, open_top)
        both = create(geo, "merge")
        both.setInput(0, open_top)
        both.setInput(1, flip)
        return both

    def ball(geo):
        sph = create(geo, "sphere")
        set_parm(sph, ["type"], 1)  # polygon
        set_parm(sph, ["rad"], (0.5, 0.5, 0.5))
        set_parm(sph, ["t"], (0, 3, 0))
        return sph

    dop = new_dopnet(f"tray_{shape_index}")
    tray_obj = rbd_object(dop, "tray", single_object_geo(f"tray_{shape_index}_geo", tray))
    data = box3d_data(dop, tray_obj, "tray_data", bodytype=0 if static else 2, defaultshape=shape_index)
    ball_obj = rbd_object(dop, "ball", single_object_geo(f"tray_{shape_index}_ball", ball))
    solver = box3d_solver(dop, merge(dop, [data, ball_obj]))
    finish(dop, solver)
    if body_type_expression:
        set_parm(data, ["parmop_bodytype"], 1)  # Set Always
        data.parm("bodytype").setExpression(body_type_expression)

        def lows(sim, f):
            return (min(p[1] for p in whole_object_points(sim, "tray")), min(p[1] for p in whole_object_points(sim, "ball")))

        track = step(dop, 96, lows)
        return [track[f] for f in (30, 60, 96)], node_messages(solver)
    low = step(dop, 96, lambda sim, f: min(p[1] for p in whole_object_points(sim, "ball")))
    return low[96]


def test_static_rbd_object_mesh():
    """A static RBD Object with Default Shape Concave collides as its exact polygons (a triangle mesh): a ball
    dropped into an open tray lands on the tray's floor (lowest point y 0), where a convex hull of the tray would
    hold it on top (y 2)."""
    mesh, hull = static_tray_rest(3), static_tray_rest(2)
    ok = abs(mesh) < 0.06 and abs(hull - 2.0) < 0.1
    return [Result("static_rbd_object_mesh", Result.PASS if ok else Result.FAIL,
                   f"ball's lowest point after landing: {mesh:.3f} with a concave static tray (expected 0, on the "
                   f"floor), {hull:.3f} with a convex one (expected 2, on top)")]


def test_static_rbd_object_retype():
    """An RBD Object whose Body Type changes mid-simulation keeps working with the mesh: static (a mesh) until
    frame 40 then dynamic (hulls) it starts falling, and dynamic then static from frame 40 it freezes where it is,
    with no solver message either way."""
    to_dynamic, msgs1 = static_tray_rest(3, body_type_expression="if($FF<40,0,2)")
    to_static, msgs2 = static_tray_rest(3, body_type_expression="if($FF<40,2,0)")
    fell = abs(to_dynamic[0][0]) < 1e-3 and to_dynamic[1][0] < -1.0 and to_dynamic[2][0] < to_dynamic[1][0] - 1.0
    froze = to_static[0][0] < -1.0 and abs(to_static[1][0] - to_static[2][0]) < 1e-3
    ok = fell and froze and not msgs1 and not msgs2
    return [Result("static_rbd_object_retype", Result.PASS if ok else Result.FAIL,
                   f"tray's lowest y on frames 30, 60, 96: static to dynamic {[round(t[0], 2) for t in to_dynamic]} "
                   f"(expected 0, then falling), dynamic to static {[round(t[0], 2) for t in to_static]} (expected "
                   f"falling, then frozen); messages {msgs1 + msgs2 or 'none'}")]


def _ring_radii(points):
    """(top, bottom): the widest horizontal distance from the points' centre axis among the points above and below
    their mean height."""
    n = len(points)
    cx, cy, cz = (sum(p[i] for p in points) / n for i in range(3))
    top = bottom = 0.0
    for p in points:
        r = ((p[0] - cx) ** 2 + (p[2] - cz) ** 2) ** 0.5
        if p[1] > cy:
            top = max(top, r)
        else:
            bottom = max(bottom, r)
    return top, bottom


def test_tapered_tube_hull():
    """A packed tapered Tube (radius 1 at one end, 0.5 at the other) is collided as the shape Houdini draws: the
    rings of the Box3D hull, read back from Export Collision Shapes, are as wide at the top and at the bottom as
    the same tube converted to polygons."""
    from checks_bullet import primitive_sop, set_type_exactly

    parms = {("rad1",): 1.0, ("rad2",): 0.5, ("height",): 2.0}
    path = primitive_sop("taper_tube_piece", "tube", "Primitive", parms, True)
    ref = new_geo("taper_tube_ref")
    tube = create(ref, "tube")
    set_type_exactly(tube, "Primitive")
    for names, value in parms.items():
        set_parm(tube, list(names), value)
    conv = create(ref, "convert")
    conv.setInput(0, tube)
    set_parm(conv, ["fromtype"], "tube")
    set_parm(conv, ["totype"], "poly")
    expect = _ring_radii([tuple(p.position()) for p in conv.geometry().points()])
    dop = new_dopnet("taper_tube")
    finish(dop, box3d_solver(dop, packed_object(dop, "piece", path), gravity=(0, 0, 0)))
    set_parm(dop.node("box3d_solver"), ["exportshapes"], 1)
    geo = new_geo("taper_tube_read")
    imp = create(geo, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "piece")
    set_parm(imp, ["geodatapath"], "Box3DShapes")
    for f in (1, 2, 3):
        hou.setFrame(f)
    pts = [tuple(p.position()) for p in imp.geometry().points()]
    got = _ring_radii(pts) if pts else (0.0, 0.0)
    ok = close(got[0], expect[0], 0.02) and close(got[1], expect[1], 0.02) and abs(expect[0] - expect[1]) > 0.2
    return [Result("tapered_tube_hull", Result.PASS if ok else Result.FAIL,
                   f"hull ring radii (top, bottom) ({got[0]:.3f}, {got[1]:.3f}), the converted tube's "
                   f"({expect[0]:.3f}, {expect[1]:.3f}) (expected equal, and different from each other)")]


def _far_scene(tag, offset, georep, solver_sop_node=False):
    """A 1 m box dropped from y 2 onto a ground piece (`georep` collider), both packed and shifted by `offset`
    along x with a 64-bit P (Attribute Cast, then a Python SOP, which keeps it) into a Box3D Packed Object. Returns
    the box's position at frame 90 minus the offset. solver_sop_node: through the Box3D Solver SOP instead of a DOP
    network."""
    pieces = [dict(GROUND, attrs={"active": 0, "bullet_georep": georep}),
              {"size": (1, 1, 1), "center": (0.1, 2, 0), "attrs": {"active": 1, "bullet_georep": "convexhull"}}]
    path, names = packed_boxes(f"{tag}_pieces", pieces)
    far = new_geo(f"{tag}_far")
    merge_node = create(far, "object_merge")
    set_parm(merge_node, ["objpath1"], path)
    cast = create(far, "attribcast")
    cast.setInput(0, merge_node)
    set_parm(cast, ["numcasts"], 1)
    set_parm(cast, ["class1"], "point")
    set_parm(cast, ["attribs1"], "P")
    set_parm(cast, ["precision1"], "fpreal64")
    shift = create(far, "python")
    shift.setInput(0, cast)
    code = "geo = hou.pwd().geometry()" + chr(10) + "for pt in geo.points():" + chr(10)
    code += f"    pt.setPosition(pt.position() + hou.Vector3({offset!r}, 0, 0))" + chr(10)
    set_parm(shift, ["python"], code)
    display(shift)
    if solver_sop_node:
        from checks_sop_common import named_positions, play, solver_sop

        sop = solver_sop(f"{tag}_sop", geometry=shift.path())
        final = play(sop, 90, lambda s, f: named_positions(s.geometry(0)) if f == 90 else None)[90]
    else:
        dop = new_dopnet(tag)
        finish(dop, box3d_solver(dop, box3d_packed_object(dop, "pieces", shift.path())))
        final = step(dop, 90, lambda sim, f: packed_positions(sim, "pieces"))[90]
    if not final or names[1] not in final:
        return None
    p = final[names[1]]
    return (p[0] - offset, p[1], p[2])


def test_far_world():
    """Packed pieces with a 64-bit P at x = 1e6 and 5e6 behave as at the origin: a box dropped onto a convex and onto
    a concave (triangle mesh) ground ends up where the same scene at the origin leaves it, to 1e-3 (a float position
    is 0.06 apart at 1e6, 0.5 at 5e6)."""
    rows, ok = [], True
    for georep, via_sop in (("convexhull", False), ("concave", False), ("convexhull", True)):
        label = georep + (" (Solver SOP)" if via_sop else "")
        home = _far_scene(f"far_{georep}{int(via_sop)}_0", 0.0, georep, via_sop)
        for offset in (1.0e6, 5.0e6):
            p = _far_scene(f"far_{georep}{int(via_sop)}_{int(offset)}", offset, georep, via_sop)
            good = (home is not None and p is not None and close(p[1], 0.5, 0.02)
                    and all(close(p[i], home[i], 1e-3) for i in range(3)))
            ok = ok and good
            rows.append(f"{label} at x {offset:g}: " + ("missing" if p is None else
                        f"({p[0]:.4f}, {p[1]:.3f}, {p[2]:.4f}) against {tuple(round(c, 4) for c in home)} at the origin"))
    return [Result("far_world", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS = {
    "unpacked_geometry": test_unpacked_geometry,
    "unpacked_geometry_bullet": test_unpacked_geometry_bullet,
    "unpacked_geometry_match": test_unpacked_geometry_match,
    "point_object_bullet": test_point_object_bullet,
    "collision_masks": test_collision_masks,
    "collision_shapes_mesh": test_collision_shapes_mesh,
    "collision_shapes_follow": test_collision_shapes_follow,
    "collision_shapes_remove": test_collision_shapes_remove,
    "static_rbd_object_mesh": test_static_rbd_object_mesh,
    "static_rbd_object_retype": test_static_rbd_object_retype,
    "tapered_tube_hull": test_tapered_tube_hull,
    "far_world": test_far_world,
}
