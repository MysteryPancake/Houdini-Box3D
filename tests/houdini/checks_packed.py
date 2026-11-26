"""Packed geometry and the Box3D Packed Object: written attributes, rest and pivots, initial state and type,
Packed Disk / Alembic, mirrored pieces, edited sources."""

import math
import os
import tempfile

import hou
from b3dtest import (
    GROUND,
    Result,
    SetupError,
    box3d_data,
    box3d_packed_object,
    box3d_solver,
    close,
    create,
    det3,
    display,
    finish,
    ground_object,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    object_bbox,
    packed_boxes,
    packed_object,
    packed_positions,
    packed_transform,
    point_attributes,
    set_parm,
    sop_solver,
    step,
    set_menu_token,
)


def drop_one(name, sop_path, frames=72, extra=None):
    """Drops the pieces of sop_path onto a ground object; returns the DOP network and the track of
    the object's bounding box."""
    dop = new_dopnet(name)
    obj = packed_object(dop, "piece", sop_path)
    upstream = extra(dop, obj) if extra else obj
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, name + "_ground"), upstream])))
    track = step(dop, frames, lambda sim, f: object_bbox(sim, "piece"))
    return dop, track


# ---------------------------------------------------------------------------
# A packed piece whose pivot is not at its centroid (status: packed points, P is the pivot)
# ---------------------------------------------------------------------------


def test_offset_pivot():
    """A packed box whose pivot is moved off its centroid (geometry left in place) lands with its
    bottom on the ground, like a normal one."""
    geo = new_geo("pivot_piece")
    box = create(geo, "box")
    xf = create(geo, "xform")
    xf.setInput(0, box)
    set_parm(xf, ["t"], (0, 2, 0))
    asm = create(geo, "assemble")
    asm.setInput(0, xf)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    wr = create(geo, "attribwrangle", "move_pivot")
    wr.setInput(0, asm)
    set_parm(wr, ["class"], 1)  # primitives
    set_parm(wr, ["snippet"], """
vector old = primintrinsic(0, "pivot", @primnum);  // Assemble: the centroid, in world space
vector piv = old + {0.4, 0.4, 0.0};                  // off-centre, still inside the box
matrix3 m = primintrinsic(0, "transform", @primnum);
setprimintrinsic(0, "pivot", @primnum, piv);
int pt = primpoint(0, @primnum, 0);
setpointattrib(0, "P", pt, point(0, "P", pt) + (piv - old) * m);
""")
    display(wr)
    start_min = wr.geometry().boundingBox().minvec()[1]
    dop = new_dopnet("pivot")
    obj = packed_object(dop, "piece", wr.path())
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "pivot_ground"), obj])))

    def sample(sim, f):
        return object_bbox(sim, "piece"), packed_transform(sim, "piece"), packed_positions(sim, "piece")

    track = step(dop, 96, sample)
    (lo, hi), r, pos = track[96]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r[1][1] / max(1e-9, math.sqrt(sum(c * c for c in r[1])))))))
    p96, p95 = list(pos.values())[0], list(track[95][2].values())[0]
    speed = (p96 - p95).length() * hou.fps()
    ok = close(lo[1], 0.0, 0.02) and close(hi[1] - lo[1], 1.0, 0.02)
    return [Result("offset_pivot", Result.PASS if ok else Result.FAIL,
                   f"pivot moved off the centroid (geometry bottom started at y={start_min:.2f}); after 4 s the box "
                   f"spans y {lo[1]:.3f} to {hi[1]:.3f} (expected 0 to 1), tilted {tilt:.1f} deg, pivot moving "
                   f"{speed:.3f}/s, pivot point at {tuple(round(c, 3) for c in p96)}")]


# ---------------------------------------------------------------------------
# Packed Disk and Alembic primitives (status: packed types other than packed geometry)
# ---------------------------------------------------------------------------


def unit_box_file(suffix):
    geo = new_geo("source_box_" + suffix.strip("."))
    box = create(geo, "box")
    path = os.path.join(tempfile.gettempdir(), f"box3d_test_box{suffix}")
    box.geometry().saveToFile(path)
    return path


def named_and_placed(geo, upstream, name):
    xf = create(geo, "xform")
    xf.setInput(0, upstream)
    set_parm(xf, ["t"], (0, 2, 0))
    wr = create(geo, "attribwrangle", "name")
    wr.setInput(0, xf)
    set_parm(wr, ["snippet"], f's@name = "{name}";')
    display(wr)
    return wr


def test_packed_disk():
    """A Packed Disk primitive (File SOP, Load: Packed Disk Primitive) collides as its box."""
    path = unit_box_file(".bgeo.sc")
    geo = new_geo("disk_piece")
    f = create(geo, "file")
    set_parm(f, ["file"], path)
    set_menu_token(f, ["loadtype"], "delayed")  # Packed Disk Primitive
    out = named_and_placed(geo, f, "disk")
    kinds = sorted({p.type().name() if hasattr(p.type(), "name") else str(p.type()) for p in out.geometry().prims()})
    _, track = drop_one("disk", out.path())
    lo, hi = track[72]
    ok = close(lo[1], 0.0, 0.03) and close(hi[1], 1.0, 0.03)
    return [Result("packed_disk", Result.PASS if ok else Result.FAIL,
                   f"primitive types {kinds}; at rest spans y {lo[1]:.3f} to {hi[1]:.3f} (expected 0 to 1)")]


def test_packed_alembic():
    """A packed Alembic primitive (written with the Alembic ROP, read with the Alembic SOP) collides
    as its box."""
    src = new_geo("abc_source")
    box = create(src, "box")
    display(box)
    path = os.path.join(tempfile.gettempdir(), "box3d_test_box.abc")
    rop = create(hou.node("/out"), "alembic", "write_abc")
    set_parm(rop, ["filename"], path)
    set_parm(rop, ["use_sop_path"], 1)
    set_parm(rop, ["sop_path"], box.path())
    rop.render(frame_range=(1, 1))
    if not os.path.exists(path):
        raise SetupError(f"the Alembic ROP did not write {path}")
    geo = new_geo("abc_piece")
    abc = create(geo, "alembic")
    set_parm(abc, ["fileName", "filename"], path)
    out = named_and_placed(geo, abc, "abc")
    _, track = drop_one("alembic", out.path())
    lo, hi = track[72]
    ok = close(lo[1], 0.0, 0.03) and close(hi[1], 1.0, 0.03)
    return [Result("packed_alembic", Result.PASS if ok else Result.FAIL,
                   f"at rest spans y {lo[1]:.3f} to {hi[1]:.3f} (expected 0 to 1)")]


# ---------------------------------------------------------------------------
# Primitive spheres and nested packing
# ---------------------------------------------------------------------------


def test_odd_packed_contents():
    """A packed primitive sphere (radius 0.5) and a packed box inside another packed primitive
    each rest on the ground spanning y 0 to 1."""
    rows = []
    ok = True
    for label in ("primitive sphere", "nested packed box"):
        geo = new_geo("odd_" + label.split()[0])
        if label == "primitive sphere":
            src = create(geo, "sphere")
            set_menu_token(src, ["type"], "prim")
            set_parm(src, ["rad"], (0.5, 0.5, 0.5))
        else:
            box = create(geo, "box")
            inner = create(geo, "pack")
            inner.setInput(0, box)
            src = inner
        asm = create(geo, "assemble")
        asm.setInput(0, src)
        set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
        xf = create(geo, "xform")
        xf.setInput(0, asm)
        set_parm(xf, ["t"], (0, 2, 0))
        display(xf)
        _, track = drop_one("odd_" + label.split()[0], xf.path())
        lo, hi = track[72]
        ok = ok and close(lo[1], 0.0, 0.03) and close(hi[1], 1.0, 0.03)
        rows.append(f"{label}: rests spanning y {lo[1]:.2f} to {hi[1]:.2f} (expected 0 to 1)")
    return [Result("odd_packed_contents", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_packed_tube_taper():
    """A tapered tube lying on its side (radius 0.5 at the top, 0.25 at the bottom, height 1) as a Tube
    primitive and as polygons: both settle with the same axis tilt, so the primitive's taper is read the
    right way round."""
    turns = {}
    for item in ("prim", "poly"):
        geo = new_geo("taper_" + item.lower())
        tube = create(geo, "tube")
        set_menu_token(tube, ["type"], item)
        set_parm(tube, ["rad1"], 0.5)
        set_parm(tube, ["rad2"], 0.25)
        set_parm(tube, ["height"], 1.0)
        asm = create(geo, "assemble")
        asm.setInput(0, tube)
        set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
        xf = create(geo, "xform")
        xf.setInput(0, asm)
        set_parm(xf, ["t"], (0, 2, 0))
        set_parm(xf, ["r"], (0, 0, 90))
        display(xf)
        dop = new_dopnet("taper_" + item.lower())
        obj = packed_object(dop, "piece", xf.path())
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "taper_ground_" + item.lower()), obj])))
        turns[item] = step(dop, 96, lambda sim, f: packed_transform(sim, "piece"))[96]
    # The tube settles on its slanted side, its axis tilted by the taper (about 14 degrees, so the Y row's
    # height is about 0.24 for the one end up and about -0.24 for the other); how far it rolled about the
    # axis is chaotic, so only the tilt is compared.
    a, b = turns["prim"][1][1], turns["poly"][1][1]
    ok = close(a, b, 0.05) and abs(a) > 0.15
    return [Result("packed_tube_taper", Result.PASS if ok else Result.FAIL,
                   f"axis tilt (the Y row's height) {a:.3f} as a primitive, {b:.3f} as polygons (expected equal, about 0.24)")]


# ---------------------------------------------------------------------------
# Mirrored packed transform (status: mirrored packed transforms)
# ---------------------------------------------------------------------------


def test_mirrored_piece():
    """A packed piece with a mirrored transform (scale -1 in x): does it land correctly, and is it
    still mirrored once it moves? INFO: the write-back keeps the mirror, so the determinant should stay
    negative."""
    geo = new_geo("mirror_piece")
    box = create(geo, "box")
    set_parm(box, ["size"], (1, 1, 1))
    asm = create(geo, "assemble")
    asm.setInput(0, box)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, asm)
    set_parm(xf, ["s"], (-1, 1, 1))
    set_parm(xf, ["t"], (0, 2, 0))
    display(xf)
    dop = new_dopnet("mirror")
    obj = packed_object(dop, "piece", xf.path())
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "mirror_ground"), obj])))
    track = step(dop, 48, lambda sim, f: (packed_transform(sim, "piece"), object_bbox(sim, "piece")))
    d1, d48 = det3(track[1][0]), det3(track[48][0])
    lo, hi = track[48][1]
    return [Result("mirrored_piece", Result.INFO,
                   f"transform determinant {d1:+.2f} on frame 1, {d48:+.2f} on frame 48 (negative = mirrored); "
                   f"rests spanning y {lo[1]:.2f} to {hi[1]:.2f}")]


def test_mirrored_midsim():
    """A packed piece that turns mirrored mid-simulation. A wedge (a box with one corner pulled in, so its mirror
    image is another shape) floats with no gravity, and a SOP Solver flips the third row of its packed transform on
    frame 12 (the flip the solver itself splits a mirror into). The collision shape (Export Collision Shapes) must
    follow the geometry from the next step on: every corner of the wedge, through the packed transform, is a corner
    of the exported shape and the other way round, before the flip and after it, and the shape did change (the
    mirror image is a different shape)."""
    geo = new_geo("mirror_mid")
    box = create(geo, "box")
    set_parm(box, ["size"], (2, 2, 2))
    wedge = create(geo, "attribwrangle", "wedge")
    wedge.setInput(0, box)
    set_parm(wedge, ["class"], "point")
    set_parm(wedge, ["snippet"], "if (@P.x > 0 && @P.y > 0 && @P.z > 0) @P.x = 0;")
    asm = create(geo, "assemble")
    asm.setInput(0, wedge)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, asm)
    set_parm(xf, ["t"], (0, 5, 0))
    display(xf)
    dop = new_dopnet("mirror_mid")
    obj = packed_object(dop, "piece", xf.path())
    solver = box3d_solver(dop, None, gravity=(0, 0, 0))
    set_parm(solver, ["exportshapes"], 1)
    flip = sop_solver(dop, "")
    set_parm(flip.node("edit"), ["class"], "primitive")
    set_parm(flip.node("edit"), ["snippet"], """
if (@Frame == 12) {
    matrix3 s = set(1, 0, 0, 0, 1, 0, 0, 0, -1);
    setprimintrinsic(0, "transform", @primnum, s * matrix3(primintrinsic(0, "transform", @primnum)));
}
""")
    finish(dop, multisolver(dop, obj, [solver, flip]))
    reader = new_geo("mirror_mid_read")
    imp = create(reader, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "piece")
    set_parm(imp, ["geodatapath"], "Box3DShapes")

    def far(a, b):
        return max(min(max(abs(x - y) for x, y in zip(p, q)) for q in b) for p in a)

    # The flip is made after frame 12's step, so the shape follows from frame 13.
    rows, ok, shapes = [], True, {}
    for f in range(1, 21):
        hou.setFrame(f)
        sim = dop.simulation()
        body = sim.findObject("piece").geometry()
        lines = imp.geometry()
        if f not in (11, 13, 14, 20):
            continue
        if not lines.prims():
            return [Result("mirrored_midsim", Result.FAIL, f"frame {f}: no shape lines ({imp.warnings()})")]
        pr = body.prims()[0]
        m = pr.intrinsicValue("packedfulltransform")
        centre = pr.points()[0].position()
        corners = set()
        for p in wedge.geometry().points():  # the piece's own points: its pivot is the origin
            q = p.position()
            corners.add(tuple(round(sum(q[r] * m[4 * r + k] for r in range(3)) + centre[k], 3) for k in range(3)))
        shapes[f] = {tuple(round(c, 3) for c in p.position()) for p in lines.points()}
        gap = max(far(corners, shapes[f]), far(shapes[f], corners))
        det = det3([list(m[0:3]), list(m[4:7]), list(m[8:11])])
        ok = ok and gap < 0.05
        rows.append(f"frame {f}: determinant {det:+.0f}, shape and geometry corners {gap:.3f} m apart")
    moved = far(shapes[11], shapes[14]) if 11 in shapes and 14 in shapes else 0.0
    rows.append(f"shape moved {moved:.2f} m between frames 11 and 14 (expected over 0.5)")
    return [Result("mirrored_midsim", Result.PASS if ok and moved > 0.5 else Result.FAIL, "; ".join(rows))]


def test_source_edit_hull():
    """A packed piece's source geometry edited between runs gives the new hull, not the cached one. A wedge (a box
    with one corner pulled in) floats with no gravity and its collision shape (Export Collision Shapes) is read back; then
    the wedge's SOP is edited to pull in a different corner and the simulation is run again in the same process (the
    source summaries are kept for the whole process, keyed on the source's identity). Every corner of the wedge
    is a corner of the shape and the other way round, both times, and the two shapes differ."""
    geo = new_geo("source_edit")
    box = create(geo, "box")
    set_parm(box, ["size"], (2, 2, 2))
    wedge = create(geo, "attribwrangle", "wedge")
    wedge.setInput(0, box)
    set_parm(wedge, ["class"], "point")
    first = "if (@P.x > 0 && @P.y > 0 && @P.z > 0) @P.x = 0;"
    second = "if (@P.x < 0 && @P.y < 0 && @P.z > 0) @P.y = 0;"
    set_parm(wedge, ["snippet"], first)
    asm = create(geo, "assemble")
    asm.setInput(0, wedge)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, asm)
    set_parm(xf, ["t"], (0, 5, 0))
    display(xf)
    dop = new_dopnet("source_edit")
    obj = packed_object(dop, "piece", xf.path())
    solver = box3d_solver(dop, None, gravity=(0, 0, 0))
    set_parm(solver, ["exportshapes"], 1)
    finish(dop, multisolver(dop, obj, [solver]))
    reader = new_geo("source_edit_read")
    imp = create(reader, "dopimport", "import_shapes")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "piece")
    set_parm(imp, ["geodatapath"], "Box3DShapes")

    def far(a, b):
        return max(min(max(abs(x - y) for x, y in zip(p, q)) for q in b) for p in a)

    def shape_and_corners():
        for f in range(1, 5):
            hou.setFrame(f)
            sim = dop.simulation()
            body = sim.findObject("piece").geometry()
            lines = imp.geometry()
        pr = body.prims()[0]
        m = pr.intrinsicValue("packedfulltransform")
        centre = pr.points()[0].position()
        corners = {tuple(round(sum(p.position()[r] * m[4 * r + k] for r in range(3)) + centre[k], 3) for k in range(3))
                   for p in wedge.geometry().points()}
        shape = {tuple(round(c, 3) for c in p.position()) for p in lines.points()}
        return corners, shape

    rows, ok, shapes = [], True, []
    for label, snippet in (("first", first), ("second", second)):
        set_parm(wedge, ["snippet"], snippet)
        resim = dop.parm("resimulate")
        if resim is not None:
            resim.pressButton()
        corners, shape = shape_and_corners()
        if not shape:
            return [Result("source_edit_hull", Result.FAIL, f"{label} wedge: no shape lines")]
        gap = max(far(corners, shape), far(shape, corners))
        shapes.append(shape)
        ok = ok and gap < 0.05
        rows.append(f"{label} wedge: shape and geometry corners {gap:.3f} m apart")
    moved = far(shapes[0], shapes[1])
    rows.append(f"the two shapes are {moved:.2f} m apart (expected over 0.5)")
    return [Result("source_edit_hull", Result.PASS if ok and moved > 0.5 else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Written v / w (status: downstream readers get v of the body origin and w in radians/s)
# ---------------------------------------------------------------------------


def test_instances_differ():
    """Instances of one packed source keep their own shapes. 60 boxes made from one source (one Copy to Points) at
    scales 1, 2 and 3 in turn, an `active` ground, 90 frames: each rests with its centre at half its scale (0.5, 1.0,
    1.5, within 0.06). Instances share their source, so a shape built for one and reused for another would put the
    three sizes at the same height."""
    from bench_scenes import copy_packed

    geo = new_geo("instances_differ_geo")
    pts = create(geo, "attribwrangle", "points")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        'addpointattrib(0, "name", ""); addpointattrib(0, "active", 1); addpointattrib(0, "scale", {1, 1, 1});\n'
        "for (int i = 0; i < 60; i++) {\n"
        "  float s = 1 + i % 3;\n"
        "  int pt = addpoint(0, set((i % 10) * 4.0 - 18.0, 2.0 + s, (i / 10) * 4.0 - 10.0));\n"
        '  setpointattrib(0, "name", pt, sprintf("p%d", i));\n'
        '  setpointattrib(0, "scale", pt, set(s, s, s));\n'
        "}"))
    box = create(geo, "box")
    set_parm(box, ["size"], (1.0, 1.0, 1.0))
    copy = copy_packed(geo, box, pts)
    dop = new_dopnet("instances_differ")
    ground_path, _ = packed_boxes("instances_differ_ground", [dict(GROUND, size=(60, 1, 60))])
    objs = [packed_object(dop, "pieces", copy.path()), packed_object(dop, "ground", ground_path)]
    finish(dop, box3d_solver(dop, merge(dop, objs)))
    final = step(dop, 90, lambda sim, f: packed_positions(sim, "pieces") if f == 90 else None)[90]
    worst = 0.0
    for i in range(60):
        want = 0.5 * (1 + i % 3)
        got = final[f"p{i}"][1]
        worst = max(worst, abs(got - want))
    heights = sorted({round(final[f"p{i}"][1], 1) for i in range(60)})
    ok = worst < 0.06 and len(heights) >= 3
    return [Result("instances_differ", Result.PASS if ok else Result.FAIL,
                   f"60 instances at scales 1, 2, 3: largest difference from the rest height {worst:.3f} (expected "
                   f"under 0.06), distinct rest heights {heights} (expected about 0.5, 1.0 and 1.5)")]


def test_rest_reset():
    """A Box3D Packed Object whose piece carries a restxform from somewhere else (turned 90 degrees
    about z) and orient identity: its SOP stage resets the rest to where the piece is on the creation
    frame, as RBD Packed Object does, so the piece stays upright (with the old rest kept it would be
    read as turned 90 degrees and jump). The restxform is the piece's own transform."""
    path, names = packed_boxes("rest_reset", [{"size": (2, 1, 1), "center": (0, 3, 0)}])
    geo = new_geo("rest_reset_stale")
    src = create(geo, "object_merge", "piece")
    set_parm(src, ["objpath1"], path)
    stale = create(geo, "attribwrangle", "stale_rest")
    stale.setInput(0, src)
    set_parm(stale, ["snippet"], 'matrix m = ident(); rotate(m, radians(90), {0, 0, 1}); 4@restxform = m; '
                                 'p@orient = {0, 0, 0, 1}; v@pivot = v@P;')
    display(stale)
    dop = new_dopnet("rest_reset")
    finish(dop, box3d_solver(dop, box3d_packed_object(dop, "piece", stale.path()), gravity=(0, 0, 0)))

    def sample(sim, f):
        if f != 6:
            return None
        return packed_transform(sim, "piece"), point_attributes(sim, "piece", names[0], ["restxform"])

    r, attrs = step(dop, 6, sample)[6]
    rest = attrs["restxform"] if attrs else None
    upright = r is not None and close(r[0][0], 1.0, 1e-3) and close(r[1][1], 1.0, 1e-3)
    rest_own = rest is not None and close(rest[0], 1.0, 1e-3) and close(rest[5], 1.0, 1e-3)
    return [Result("rest_reset", Result.PASS if upright and rest_own else Result.FAIL,
                   f"packed x axis on frame 6: {tuple(round(v, 3) for v in r[0]) if r else None} (expected "
                   f"(1, 0, 0): upright); restxform rows 0-1 diagonal "
                   f"{(round(rest[0], 3), round(rest[5], 3)) if rest else None} (expected (1, 1): the piece's own, "
                   "not the stale 90 degree one)")]


def test_rbd_packed_attributes():
    """Lists the point attributes RBD Packed Object puts on a packed piece (as the solver gets them
    on frame 1), and shows whether a Box3D Data value (Friction 0.123) reaches the body or is
    overridden by one of them. INFO: point attributes override Box3D Data by design."""
    path, names = packed_boxes("stamp_piece", [{"size": (1, 1, 1), "center": (0, 2, 0)}])
    dop = new_dopnet("stamped")
    obj = packed_object(dop, "piece", path)
    data = box3d_data(dop, obj, "data", friction=0.123, mass=7.0, computemass=0)
    solver = box3d_solver(dop, data)
    finish(dop, solver)
    attrs = step(dop, 1, lambda sim, f: point_attributes(sim, "piece", names[0]))[1] or {}
    warned = []
    step(dop, 3, lambda sim, f: warned.extend(m for m in node_messages(solver) if "override Box3D Data" in m))
    ours = {"friction", "bounce", "mass", "density", "computemass", "lineardamping", "angulardamping",
            "gravityscale", "enablesleep", "sleepthreshold", "ccdthreshold", "rollingresistance", "pscale",
            "bullet_georep", "bullet_primS", "bullet_radius", "bullet_autofit", "active", "animated", "v", "w"}
    overriding = {k: attrs[k] for k in sorted(attrs) if k in ours}
    others = sorted(k for k in attrs if k not in ours)
    results = [Result("rbd_packed_attributes", Result.INFO,
                      f"attributes that override Box3D Data: {overriding}. Other attributes: {others}. Box3D Data asked "
                      "for friction 0.123 and mass 7: any value above that differs is RBD Packed Object's")]
    results.append(Result("rbd_packed_attributes", Result.PASS if warned else Result.FAIL,
                          f"solver warning: {warned[0]!r}" if warned else
                          "no 'override Box3D Data' warning on the solver node"))
    return results


def _push_test(tag, make_object, piece_attrs=None):
    """A box pushed along the ground at 3 m/s for 1 s; make_object(dop, path) makes its DOP object.
    Returns (x after 1 s, the solver's 'override Box3D Data' warnings)."""
    attrs = {"v": (3.0, 0.0, 0.0), "active": 1}
    attrs.update(piece_attrs or {})
    # Every piece needs every attribute any piece sets (packed_boxes).
    ground = dict(GROUND, attrs=dict(attrs, active=0, v=(0.0, 0.0, 0.0)))
    path, names = packed_boxes(f"push_{tag}", [ground, {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": attrs}])
    dop = new_dopnet(f"push_{tag}")
    solver = box3d_solver(dop, make_object(dop, path))
    finish(dop, solver)
    warned, data = [], []

    def sample(sim, f):
        warned.extend(m for m in node_messages(solver) if "override Box3D Data" in m)
        if f == 2:  # the Box3D Data the solver finds on the object, and its friction
            o = sim.findObject("pieces")
            d = o.findSubData("Geometry/Box3DData") if o is not None else None
            if d is None and o is not None:
                d = o.findSubData("Box3DData")
            try:
                data.append(d.options().field("friction") if d is not None else "no Box3D Data")
            except Exception as e:  # noqa: BLE001 - a report
                data.append(f"Box3D Data present ({e})")
        a = point_attributes(sim, "pieces", names[1], ["P"])
        return a["P"][0] if a else None

    x = step(dop, 25, sample)[25]
    return x, sorted(set(warned)) + [f"[Box3D Data friction on the object: {data[0] if data else '?'}]"]


def stage_points(obj):
    """The first point of a Box3D Packed Object's SOP stage output (create_attributes/OUT), cooked
    outside the simulation (the creation-frame branch), as {attribute: value}; None if missing."""
    out = obj.node("create_attributes/OUT")
    g = out.geometry() if out is not None else None
    if g is None or not g.points():
        return None
    pt = g.points()[0]
    return {a.name(): pt.attribValue(a) for a in g.pointAttribs()}


def test_box3d_packed_object():
    """The Box3D Packed Object, like RBD Packed Object, turns its settings into point attributes when
    the object is created, but only those that differ from Box3D Data's defaults: a box pushed at
    3 m/s with the asset's Friction 0 slides about 3 m in 1 s, with no warning (its own Box3D Data
    stays at the defaults). RBD Packed Object with a Box3D Data Friction 0 stops: its stamped friction
    1 wins, and the solver warns. A point's own attribute still wins (friction 1 on the point: it
    stops). A dropped box falls as with RBD Packed Object, and Creation Frame 10 makes the object
    appear on frame 10."""
    results = []
    x_ours, w_ours = _push_test("ours", lambda dop, p: box3d_packed_object(dop, "pieces", p, friction=0.0))
    x_rbd, w_rbd = _push_test("rbd", lambda dop, p: box3d_data(
        dop, packed_object(dop, "pieces", p, keep_velocity=True), "data", friction=0.0))
    x_attr, _ = _push_test("attr", lambda dop, p: box3d_packed_object(dop, "pieces", p, friction=0.0),
                           {"friction": 1.0})
    # The stage's output: Friction 0.25 becomes a friction attribute, Bounce (at its default) none.
    probe_dop = new_dopnet("probe_stage")
    path, _ = packed_boxes("probe_stage_box", [{"size": (1, 1, 1), "center": (0, 1, 0)}])
    probe = box3d_packed_object(probe_dop, "pieces", path, friction=0.25)
    attrs = stage_points(probe)
    stage_ok = attrs is not None and close(attrs.get("friction", -1), 0.25, 1e-6) and "bounce" not in attrs
    stage_note = (f"stage output: friction {attrs.get('friction')} (expected 0.25), bounce "
                  f"{'written' if 'bounce' in attrs else 'not written'} (expected not written)" if attrs is not None
                  else f"no create_attributes/OUT in the asset (children: {[c.name() for c in probe.children()]})")

    def warnings(w):
        return [m for m in w if not m.startswith("[")]

    ok = x_ours > 2.5 and not warnings(w_ours) and x_rbd < 2.0 and bool(warnings(w_rbd)) and x_attr < 2.0 and stage_ok
    results.append(Result("box3d_packed_object", Result.PASS if ok else Result.FAIL,
                          f"Friction 0 on Box3D Packed Object: box at x={x_ours:.2f} after 1 s (expected about 3, "
                          f"slid), warnings {warnings(w_ours) or 'none'} (expected none) "
                          f"{' '.join(m for m in w_ours if m.startswith('['))}; RBD Packed Object + Box3D Data "
                          f"Friction 0: x={x_rbd:.2f} (expected stopped, below 2), warning "
                          f"{'shown' if warnings(w_rbd) else 'none'} (expected shown); point friction 1 on Box3D Packed "
                          f"Object: x={x_attr:.2f} (expected stopped: the point's value wins); {stage_note}"))

    # A dropped box: the same fall as RBD Packed Object; Creation Frame 10.
    falls = {}
    for kind in ("ours", "rbd"):
        path, names = packed_boxes(f"drop_{kind}", [GROUND, {"size": (1, 1, 1), "center": (0, 3, 0)}])
        dop = new_dopnet(f"drop_{kind}")
        obj = box3d_packed_object(dop, "pieces", path) if kind == "ours" else packed_object(dop, "pieces", path)
        finish(dop, box3d_solver(dop, obj))
        falls[kind] = step(dop, 48, lambda sim, f: point_attributes(sim, "pieces", names[1], ["P"])["P"][1]
                           if f in (12, 48) else None)
    diff = abs(falls["ours"][12] - falls["rbd"][12])
    ok = diff < 1e-3 and close(falls["ours"][48], 0.5, 0.03)
    results.append(Result("box3d_packed_object", Result.PASS if ok else Result.FAIL,
                          f"dropped box: y {falls['ours'][12]:.4f} on frame 12 (RBD Packed Object "
                          f"{falls['rbd'][12]:.4f}, differ by {diff:.1e}), rests at {falls['ours'][48]:.3f} "
                          f"(expected 0.5)"))

    path, names = packed_boxes("drop_late", [GROUND, {"size": (1, 1, 1), "center": (0, 3, 0)}])
    dop = new_dopnet("drop_late")
    obj = box3d_packed_object(dop, "pieces", path)
    label = next((pt for pt in obj.parmTuples() if pt.parmTemplate().label().lower() == "creation frame"), None)
    if label is None:
        results.append(Result("box3d_packed_object", Result.INFO, "no Creation Frame parameter on the asset"))
        return results
    label.deleteAllKeyframes()  # its default is the network's Start Frame, as an expression
    label.set((10,))
    finish(dop, box3d_solver(dop, obj))
    seen = step(dop, 12, lambda sim, f: sim.findObject("pieces") is not None)
    first = next((f for f in sorted(seen) if seen[f]), None)
    results.append(Result("box3d_packed_object", Result.PASS if first == 10 else Result.FAIL,
                          f"Creation Frame 10: object first present on frame {first} (expected 10)"))
    return results


def test_packed_object_initial_state():
    """The Box3D Packed Object's Initial State tab, as RBD Packed Object's. Inherit Velocity (off on the asset)
    keeps the point's own v: a point v of (2, 0, 0) moves 2 in 1 s whatever Velocity says. With it off,
    Velocity (1, 0, 0) and Angular Velocity (0, 90, 0) degrees per second apply: it moves 1 and w.y is
    pi / 2 radians per second."""
    results = []
    got = {}
    for inherit in (1, 0):
        path, names = packed_boxes(f"po_vel_box{inherit}", [{"size": (1, 1, 1), "center": (0, 3, 0),
                                                           "attrs": {"v": (2.0, 0.0, 0.0)}}])
        dop = new_dopnet(f"po_vel{inherit}")
        obj = box3d_packed_object(dop, "pieces", path, inheritvelocity=inherit, vel=(1.0, 0.0, 0.0),
                                  angvel=(0.0, 90.0, 0.0))
        finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))
        track = step(dop, 25, lambda sim, f: (packed_positions(sim, "pieces")[names[0]],
                                              point_attributes(sim, "pieces", names[0], ["w"])["w"])
                     if f == 25 else None)[25]
        got[inherit] = track
    x_in = got[1][0][0]
    x_out, w_out = got[0][0][0], got[0][1]
    ok = close(x_in, 2.0, 0.05) and close(x_out, 1.0, 0.05) and close(w_out[1], math.pi / 2, 0.02)
    results.append(Result("packed_object_initial_state", Result.PASS if ok else Result.FAIL,
                          f"Inherit Velocity on: x {x_in:.3f} after 1 s (expected 2, the point's v); off: x {x_out:.3f} "
                          f"(expected 1), w.y {w_out[1]:.3f} rad/s (expected {math.pi / 2:.3f})"))
    return results


def _packed_initial_run(tag, kind, parms, start=1, frames=3):
    """Two packed boxes of different sizes, made by the Box3D Packed Object (kind "box3d", in the Box3D Solver) or by
    RBD Packed Object (in RBD Bullet Solver), with `parms` on the object node and no gravity, the DOP network starting at
    frame `start`: per frame from `start`, None before the object exists, else {name: (position, 3x3 rotation)}."""
    path, names = packed_boxes(tag + "_pieces", [{"size": (1, 1, 1), "center": (2, 3, 0)},
                                                 {"size": (2, 0.5, 1), "center": (-1, 4, 1)}])
    dop = new_dopnet(tag)
    set_parm(dop, ["startframe"], start)
    if kind == "box3d":
        finish(dop, box3d_solver(dop, box3d_packed_object(dop, "pieces", path, **parms), gravity=(0, 0, 0)))
    else:
        solver = create(dop, "bulletrbdsolver", "bullet_solver")
        solver.setInput(0, packed_object(dop, "pieces", path, **parms))
        finish(dop, solver)
    out = {}
    for f in range(start, start + frames):
        hou.setFrame(f)
        sim = dop.simulation()
        obj = sim.findObject("pieces")
        geo = obj.geometry() if obj is not None else None
        if geo is None or not geo.points():
            out[f] = None
            continue
        pos = packed_positions(sim, "pieces")
        out[f] = {n: (pos[n], packed_transform(sim, "pieces", n)) for n in names}
    hou.setFrame(1)
    return out


def test_packed_object_initial_bullet():
    """The Box3D Packed Object's Initial State transform and Creation Frame Specifies Simulation Frame, against RBD
    Packed Object in RBD Bullet Solver (the same parameter names). Two boxes, Position (1, 0.5, 0) and Rotation
    (30, 45, 0), turned about each piece's own position (Inherit Pivot on) or about Pivot (0, 2, 0) (off), and with
    Angular Velocity (0, 90, 0) and no gravity: each piece's position and rotation on the first three frames match
    within 0.001 and 0.1 degree. Then the DOP network starting at frame 10 and Creation Frame 3: the first frame the
    object exists is the same in both: with Creation Frame Specifies Simulation Frame on, simulation frame 3 (scene
    frame 12); off, scene frame 3 is before the start, so neither makes it. Angular Velocity is turned by Rotation
    with the piece in both (RBD sets v and w before its initial transform)."""
    def angle(a, b):
        trace = sum(sum(a[i][k] * b[i][k] for k in range(3)) for i in range(3))
        return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))

    results = []
    base = {"t": (1.0, 0.5, 0.0), "r": (30.0, 45.0, 0.0), "p": (0.0, 2.0, 0.0), "angvel": (0.0, 90.0, 0.0),
            "inheritvelocity": 0}
    for inherit in (1, 0):
        parms = dict(base, inheritpivot=inherit)
        got = {kind: _packed_initial_run(f"po_init_{inherit}_{kind}", kind, parms) for kind in ("box3d", "bullet")}
        rows, ok = [], True
        for f in (1, 2, 3):
            a, b = got["box3d"][f], got["bullet"][f]
            if a is None or b is None:
                ok = False
                rows.append(f"frame {f}: Box3D {'has' if a else 'has no'} pieces, Bullet {'has' if b else 'has no'}")
                continue
            for n in a:
                dp = (a[n][0] - b[n][0]).length()
                da = angle(a[n][1], b[n][1])
                if dp > 1e-3 or da > 0.1:
                    ok = False
                    rows.append(f"frame {f} {n}: {dp:.4f} apart, {da:.2f} degrees")
        first = got["box3d"][1]
        where = ", ".join(f"{n} at {tuple(round(c, 3) for c in first[n][0])}" for n in first) if first else "none"
        results.append(Result("packed_object_initial_bullet", Result.PASS if ok else Result.FAIL,
                              f"Inherit Pivot {inherit}: " + ("; ".join(rows) if rows else
                                                             f"positions and rotations match on frames 1-3 ({where} "
                                                             f"on frame 1)")))
    for sim_frame in (1, 0):
        parms = {"createframe": 3, "usesimframe": sim_frame}
        first = {}
        for kind in ("box3d", "bullet"):
            t = _packed_initial_run(f"po_create_{sim_frame}_{kind}", kind, parms, start=10, frames=6)
            first[kind] = next((f for f in sorted(t) if t[f] is not None), None)
        # Off, scene frame 3 is before the start: neither makes the object.
        ok = first["box3d"] == first["bullet"] and (first["box3d"] == 12 if sim_frame else first["box3d"] is None)
        results.append(Result("packed_object_initial_bullet", Result.PASS if ok else Result.FAIL,
                              f"Creation Frame 3, Creation Frame Specifies Simulation Frame {sim_frame}, start frame "
                              f"10: first frame with the object Box3D {first['box3d']}, Bullet {first['bullet']}"))
    return results


def test_packed_object_initial_type():
    """The Box3D Packed Object's Initial Object Type, as RBD Packed Object's: Body Type and Deforming follow it. Active
    writes neither active nor deforming; Static active 0; Animated Static active 0 and animated 1; Deforming Static
    active 0 and deforming 1; Deforming Active deforming 1 only. With Custom they are the object's own (Body Type
    Kinematic: active 0 and animated 1)."""
    path, _ = packed_boxes("po_initial_type", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    want = {"active": {}, "static": {"active": 0}, "animatedstatic": {"active": 0, "animated": 1},
            "deformingstatic": {"active": 0, "deforming": 1}, "deformingactive": {"deforming": 1}}
    rows, ok = [], True
    for token, expected in list(want.items()) + [("custom", {"active": 0, "animated": 1})]:
        obj = box3d_packed_object(new_dopnet(f"po_initial_{token}"), "pieces", path)
        set_parm(obj, ["initialstate"], token)
        if token == "custom":
            set_parm(obj, ["bodytype"], 1)
        attrs = stage_points(obj) or {}
        got = {"active": attrs["active"]} if "active" in attrs else {}
        got.update({k: attrs[k] for k in ("animated", "deforming") if attrs.get(k)})
        good = got == expected
        ok = ok and good
        rows.append(f"{token}: {got}")
    return [Result("packed_object_initial_type", Result.PASS if ok else Result.FAIL,
                   "attributes written per Initial Object Type: " + "; ".join(rows))]


def test_packed_object_deforming():
    """The Box3D Packed Object's Deforming toggle writes deforming 1 on the points (off: none) and reads
    the SOP every step, as Overwrite Attributes from SOP does. A Kinematic piece whose SOP object rises
    0.1 per frame follows it (y 3 + 1.2 on frame 13) with Deforming on, and also with it off: the solver
    reads the source SOP itself, as RBD Bullet Solver does."""
    results = []
    path, _ = packed_boxes("po_def_probe", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    flags = {}
    for on in (0, 1):
        attrs = stage_points(box3d_packed_object(new_dopnet(f"po_def_attr{on}"), "pieces", path, deforming=on)) or {}
        flags[on] = attrs.get("deforming")
    ok = not flags[0] and flags[1] == 1
    results.append(Result("packed_object_deforming", Result.PASS if ok else Result.FAIL,
                          f"deforming attribute with Deforming off / on: {flags[0]} / {flags[1]} (expected none or 0 / 1)"))
    rows, ok = [], True
    for on in (0, 1):
        path, names = packed_boxes(f"po_def_box{on}", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        hou.node(path).parent().parm("ty").setExpression("($FF - 1) * 0.1", hou.exprLanguage.Hscript)
        dop = new_dopnet(f"po_def_follow{on}")
        obj = box3d_packed_object(dop, "pieces", path, usetransform=1, bodytype=1, deforming=on)
        finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))
        y = step(dop, 13, lambda sim, f: packed_positions(sim, "pieces")[names[0]][1] if f == 13 else None)[13]
        good = close(y, 4.2, 0.05)
        rows.append(f"Deforming {'on' if on else 'off'}: y {y:.3f} on frame 13 (expected 4.2)")
        ok = ok and good
    results.append(Result("packed_object_deforming", Result.PASS if ok else Result.FAIL, "; ".join(rows)))
    return results


def test_packed_object_hand_set_shape():
    """Box3D Data's hand-set shape (Box Size, Radius, Length, Offset, Rotation) and Rotational Stiffness, through
    the Box3D Packed Object: each changed setting becomes the point attribute of the same name (the others
    none), and a unit box piece with Default Shape Box, Auto Fit off and Box Size 0.5 0.25 0.5 comes to rest
    at y = 0.25 on the ground."""
    results = []
    probe_dop = new_dopnet("probe_hand_set")
    path, _ = packed_boxes("probe_hand_set_box", [{"size": (1, 1, 1), "center": (0, 1, 0)}])
    probe = box3d_packed_object(probe_dop, "pieces", path, inertialtensorstiffness=3.0,
                                bullet_primS=(0.5, 0.25, 0.5))
    attrs = stage_points(probe)
    size = attrs.get("bullet_primS") if attrs else None
    ok = (attrs is not None and close(attrs.get("inertialtensorstiffness", -1), 3.0, 1e-6)
          and size is not None and all(close(a, b, 1e-6) for a, b in zip(size, (0.5, 0.25, 0.5)))
          and "bullet_radius" not in attrs and "bullet_primT" not in attrs)
    results.append(Result("packed_object_hand_set_shape", Result.PASS if ok else Result.FAIL,
                          f"stage output: inertialtensorstiffness {attrs.get('inertialtensorstiffness') if attrs else None}"
                          f" (expected 3), bullet_primS {size} (expected 0.5 0.25 0.5), unchanged settings not written"))

    path, names = packed_boxes("hand_set_drop", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    dop = new_dopnet("hand_set_drop")
    obj = box3d_packed_object(dop, "pieces", path, defaultshape=0, autofit=0, bullet_primS=(0.5, 0.25, 0.5))
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "hand_set_ground"), obj])))
    y = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))[72][names[0]][1]
    results.append(Result("packed_object_hand_set_shape", Result.PASS if close(y, 0.25, 0.03) else Result.FAIL,
                          f"Box Size 0.5 0.25 0.5, Auto Fit off: the piece rests at y={y:.3f} (expected 0.25)"))
    return results


def _find_template(group, name):
    """The parameter template called `name` anywhere in a template group (folders included)."""
    for entry in group.entries() if hasattr(group, "entries") else group.parmTemplates():
        if entry.name() == name:
            return entry
        if hasattr(entry, "parmTemplates"):
            found = _find_template(entry, name)
            if found is not None:
                return found
    return None


def test_default_shape_menus():
    """Default Shape's menu on Box3D Data, the Box3D Packed Object and Box3D Configure ends with Plane and Cylinder
    (in that order, after None), and on the Packed Object picking them writes bullet_georep "plane" and "cylinder"
    on the points."""
    rows, ok = [], True
    for category, name, parm in ((hou.dopNodeTypeCategory(), "box3d_data", "defaultshape"),
                                 (hou.dopNodeTypeCategory(), "box3d_packedobject", "defaultshape"),
                                 (hou.sopNodeTypeCategory(), "box3d_configure", "defaultshape#")):
        node_type = hou.nodeType(category, name)
        if node_type is None:
            raise SetupError(f"node type {name} is not installed")
        template = _find_template(node_type.parmTemplateGroup(), parm)
        labels = list(template.menuLabels()) if template is not None else []
        good = [lbl for lbl in labels[-3:]] == ["None", "Plane", "Cylinder"]
        ok = ok and good
        rows.append(f"{name}: last three entries {labels[-3:]}")
    path, _ = packed_boxes("shape_menu_box", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    written = {}
    for index, token in ((6, "plane"), (7, "cylinder")):
        obj = box3d_packed_object(new_dopnet(f"shape_menu_{token}"), "pieces", path, defaultshape=index)
        written[token] = (stage_points(obj) or {}).get("bullet_georep")
        ok = ok and written[token] == token
    rows.append(f"bullet_georep written for menu items 6 and 7: {written} (expected plane, cylinder)")
    return [Result("default_shape_menus", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_box3d_data_hand_set_shape():
    """Box3D Data's hand-set shape and Rotational Stiffness set on a Box3D Data DOP itself, below a Box3D Packed Object
    left at its defaults (RBD Packed Object would stamp attributes that win over the node), at Set Always
    (the asset's own Box3D Data was made at its creation): Default Shape Box with Auto Fit off and Box Size 0.5 0.25 0.5 rests a unit box
    piece at y = 0.25 on the ground (0.5 without the node), and Rotational Stiffness 4 makes a point torque of 1
    on a 1 kg unit box spin a quarter as fast (w.z 0.25 becomes 0.0625)."""
    results = []
    heights = {}
    for kind in ("plain", "shaped"):
        path, names = packed_boxes(f"data_shape_{kind}", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        dop = new_dopnet(f"data_shape_{kind}")
        obj = box3d_packed_object(dop, "pieces", path)
        if kind == "shaped":
            obj = box3d_data(dop, obj, "data", defaultshape=0, autofit=0, bullet_primS=(0.5, 0.25, 0.5))
            for parm in ("defaultshape", "autofit", "bullet_primS"):
                set_menu_token(obj, [f"parmop_{parm}"], "always")
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, f"data_shape_{kind}_ground"), obj])))
        heights[kind] = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))[72][names[0]][1]
    ok = close(heights["plain"], 0.5, 0.03) and close(heights["shaped"], 0.25, 0.03)
    results.append(Result("box3d_data_hand_set_shape", Result.PASS if ok else Result.FAIL,
                          f"rest height without / with Box Size 0.5 0.25 0.5 on a Box3D Data DOP: "
                          f"{heights['plain']:.3f} / {heights['shaped']:.3f} (expected 0.5 / 0.25)"))

    spins = {}
    for stiffness in (1.0, 4.0):
        path, names = packed_boxes(f"data_stiff_{int(stiffness)}",
                                   [{"size": (1, 1, 1), "center": (0, 0, 0), "attrs": {"torque": (0.0, 0.0, 1.0)}}])
        dop = new_dopnet(f"data_stiff_{int(stiffness)}")
        obj = box3d_packed_object(dop, "pieces", path, computemass=0, mass=1.0)
        data = box3d_data(dop, obj, "data", inertialtensorstiffness=stiffness)
        set_menu_token(data, ["parmop_inertialtensorstiffness"], "always")
        finish(dop, box3d_solver(dop, data, gravity=(0, 0, 0)))
        spins[stiffness] = step(dop, 2, lambda sim, f: point_attributes(sim, "pieces", names[0], ["w"])["w"][2]
                                if f == 2 else None)[2]
    ok = close(spins[1.0], 0.25, 0.02) and close(spins[4.0], 0.0625, 0.01)
    results.append(Result("box3d_data_hand_set_shape", Result.PASS if ok else Result.FAIL,
                          f"w.z after one step with Rotational Stiffness 1 / 4: {spins[1.0]:.4f} / {spins[4.0]:.4f} "
                          f"(expected 0.25 / 0.0625)"))
    return results


def test_tab_submenu():
    """The three assets are listed under Box3D in the Tab menu: each has a Tools.shelf section naming that
    submenu."""
    rows, ok = [], True
    for category, name in ((hou.dopNodeTypeCategory(), "box3d_packedobject"),
                           (hou.sopNodeTypeCategory(), "box3d_configure"),
                           (hou.sopNodeTypeCategory(), "box3d_solver")):
        node_type = hou.nodeType(category, name)
        definition = node_type.definition() if node_type is not None else None
        shelf = definition.sections().get("Tools.shelf") if definition is not None else None
        good = shelf is not None and "<toolSubmenu>Box3D</toolSubmenu>" in shelf.contents()
        rows.append(f"{category.name()} {name}: {'Box3D' if good else 'no Box3D submenu'}")
        ok = ok and good
    return [Result("tab_submenu", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_packed_object_attributes():
    """The Box3D Packed Object's SOP stage, as RBD Packed Object's: Body Type Static and Default Shape
    Sphere become active 0 and bullet_georep "sphere", untouched settings write nothing, every piece
    gets a point name, and on the creation frame an input orient / pscale is baked into the packed
    transform (a 2 x 1 x 1 box with orient 90 degrees about y and pscale 0.5 is simulated 0.5 wide in
    x and 1 deep in z, orient kept as the rotation since rest)."""
    results = []
    path, _ = packed_boxes("po_attr_box", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    dop = new_dopnet("po_attr")
    obj = box3d_packed_object(dop, "pieces", path, bodytype=0, defaultshape=1)
    attrs = stage_points(obj) or {}
    untouched = [n for n in ("friction", "bounce", "density", "mass", "lineardamping", "bullet_autofit", "awake")
                 if n in attrs]
    ok = attrs.get("active") == 0 and attrs.get("bullet_georep") == "sphere" and not untouched and attrs.get("name")
    results.append(Result("packed_object_attributes", Result.PASS if ok else Result.FAIL,
                          f"stage output: active {attrs.get('active')} (expected 0), bullet_georep "
                          f"{attrs.get('bullet_georep')!r} (expected 'sphere'), untouched settings written: "
                          f"{untouched or 'none'}, name {attrs.get('name')!r}"))

    geo = new_geo("po_bake_in")
    src = create(geo, "object_merge", "pieces")
    path, _ = packed_boxes("po_bake_box", [{"size": (2, 1, 1), "center": (0, 3, 0)}])
    set_parm(src, ["objpath1"], path)
    wr = create(geo, "attribwrangle", "orient_pscale")
    wr.setInput(0, src)
    set_parm(wr, ["snippet"], "p@orient = quaternion(radians(90), {0, 1, 0}); f@pscale = 0.5;")
    display(wr)
    dop = new_dopnet("po_bake")
    finish(dop, box3d_solver(dop, box3d_packed_object(dop, "pieces", wr.path()), gravity=(0, 0, 0)))

    def sample(sim, f):
        if f != 3:
            return None
        o = sim.findObject("pieces")
        g = o.geometry() if o is not None else None
        if g is None or not g.prims():
            return None
        b = g.prims()[0].intrinsicValue("bounds")
        q = g.points()[0].attribValue("orient") if g.findPointAttrib("orient") else None
        return (round(b[1] - b[0], 4), round(b[5] - b[4], 4)), q

    got = step(dop, 3, sample)[3]
    ext, q = got if got else (None, None)
    turned = q is not None and close(abs(q[1]), 0.7071, 1e-3) and close(abs(q[3]), 0.7071, 1e-3)
    ok = ext is not None and close(ext[0], 0.5, 1e-3) and close(ext[1], 1.0, 1e-3) and turned
    results.append(Result("packed_object_attributes", Result.PASS if ok else Result.FAIL,
                          f"baked piece (x, z) extent {ext} (expected (0.5, 1)); orient "
                          f"{tuple(round(v, 4) for v in q) if q else None} (expected 90 degrees about y)"))
    return results


def test_packed_object_use_transform():
    """The Box3D Packed Object's Use Object Transform, as RBD Packed Object's: with it on, a piece whose
    SOP object is moved up 5 starts 5 higher, once (the stage's Object Merge applies the transform and
    SOP Geometry does not again); off, it starts where its SOP puts it."""
    path, names = packed_boxes("po_xform_box", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    hou.node(path).parent().parm("ty").set(5.0)
    got = {}
    for on in (1, 0):
        dop = new_dopnet(f"po_xform_{on}")
        finish(dop, box3d_solver(dop, box3d_packed_object(dop, "pieces", path, usetransform=on), gravity=(0, 0, 0)))
        track = step(dop, 6, lambda sim, f: (packed_positions(sim, "pieces") or {}).get(names[0]) if f in (1, 6) else None)
        got[on] = (track[1][1] if track[1] else None, track[6][1] if track[6] else None)
    ok = all(v is not None for pair in got.values() for v in pair) and \
        close(got[1][0], 8.0, 1e-3) and close(got[1][1], 8.0, 1e-3) and close(got[0][0], 3.0, 1e-3) and \
        close(got[0][1], 3.0, 1e-3)
    return [Result("packed_object_use_transform", Result.PASS if ok else Result.FAIL,
                   f"Use Object Transform on: piece y {got[1][0]} on frame 1, {got[1][1]} on frame 6 (expected 8, 8); "
                   f"off: {got[0][0]}, {got[0][1]} (expected 3, 3)")]


def test_data_under_geometry():
    """Box3D Data attaches under the object's Geometry (Geometry/BulletData's place), not at the
    object's top level; it is still there after the solver has replaced the geometry with its own
    writable copy, and its settings still reach bodies without the attribute: a Box3D Data after a
    Box3D Packed Object with Friction 0 at Set Always (the asset's own Box3D Data was made at its
    creation, so only Set Always values change it) makes a pushed box slide."""
    path, names = packed_boxes("data_geo", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0),
                                                     "attrs": {"v": (3.0, 0.0, 0.0)}}])
    dop = new_dopnet("data_geo")
    obj = box3d_data(dop, box3d_packed_object(dop, "pieces", path), "data", friction=0.0)
    set_menu_token(obj, ["parmop_friction"], "always")
    finish(dop, box3d_solver(dop, obj))

    def sample(sim, f):
        o = sim.findObject("pieces")
        a = point_attributes(sim, "pieces", names[1], ["P"])
        return (o.findSubData("Geometry/Box3DData") is not None, o.findSubData("Box3DData") is not None,
                a["P"][0] if a else None)

    track = step(dop, 25, sample)
    under1, top1, _ = track[1]
    under25, top25, x = track[25]
    ok = under1 and under25 and not top1 and not top25 and x is not None and x > 2.5
    return [Result("data_under_geometry", Result.PASS if ok else Result.FAIL,
                   f"Geometry/Box3DData present on frame 1: {under1}, frame 25: {under25}; top-level Box3DData: "
                   f"{top1} / {top25} (expected False); box at x={x:.2f} after 1 s (slides with Friction 0)")]


TESTS = {
    "rbd_packed_attributes": test_rbd_packed_attributes,
    "rest_reset": test_rest_reset,
    "packed_object_attributes": test_packed_object_attributes,
    "packed_object_initial_state": test_packed_object_initial_state,
    "packed_object_initial_bullet": test_packed_object_initial_bullet,
    "packed_object_deforming": test_packed_object_deforming,
    "packed_object_initial_type": test_packed_object_initial_type,
    "packed_object_hand_set_shape": test_packed_object_hand_set_shape,
    "default_shape_menus": test_default_shape_menus,
    "box3d_data_hand_set_shape": test_box3d_data_hand_set_shape,
    "tab_submenu": test_tab_submenu,
    "packed_object_use_transform": test_packed_object_use_transform,
    "data_under_geometry": test_data_under_geometry,
    "box3d_packed_object": test_box3d_packed_object,
    "offset_pivot": test_offset_pivot,
    "packed_disk": test_packed_disk,
    "packed_alembic": test_packed_alembic,
    "odd_packed_contents": test_odd_packed_contents,
    "packed_tube_taper": test_packed_tube_taper,
    "mirrored_piece": test_mirrored_piece,
    "mirrored_midsim": test_mirrored_midsim,
    "source_edit_hull": test_source_edit_hull,
    "instances_differ": test_instances_differ,
}
