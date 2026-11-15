"""Height-field colliders: a Static Object whose geometry is a HeightField SOP."""

import hou

from b3dtest import (
    box3d_solver,
    close,
    create,
    display,
    finish,
    merge,
    new_dopnet,
    new_geo,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_expression,
    set_parm,
    SetupError,
    step,
)


# ---------------------------------------------------------------------------
# Height-field colliders: a Static Object whose geometry is a HeightField SOP
# ---------------------------------------------------------------------------


def heightfield_collider(name, snippet, transform=None, size=30.0, spacing=0.25, parent_transform=None):
    """A geo object with a HeightField SOP (default ZX orientation) whose height layer is set by a
    Volume Wrangle `snippet`; transform: {parm: value} on the geo object (t, r, s). Object
    parameters have no shear: parent_transform puts the geo object under a Null with these
    parameters, so a scaled parent and a turned child make a sheared world transform.
    Returns (geo object, the height volume's SOP node)."""
    geo = new_geo(name)
    if parent_transform:
        parent = create(hou.node("/obj"), "null", name + "_parent")
        for parm, value in parent_transform.items():
            set_parm(parent, [parm], value)
        geo.setFirstInput(parent)
    hf = create(geo, "heightfield")
    set_parm(hf, ["size"], (size, size))
    set_parm(hf, ["gridspacing"], spacing)
    wr = create(geo, "volumewrangle")
    wr.setInput(0, hf)
    set_parm(wr, ["snippet"], snippet)
    display(wr)
    for parm, value in (transform or {}).items():
        set_parm(geo, [parm], value)
    return geo, wr


def _height_volume(sop):
    for prim in sop.geometry().prims():
        if isinstance(prim, hou.Volume) and prim.attribValue("name") == "height":
            return prim
    raise SetupError(f"{sop.path()} has no volume named height")


def _sim_geometry_transform(sim):
    """The collider's geometry-to-world matrix as the simulation holds it. hou.DopObject.transform()
    already includes the Geometry data's own Transform (Use Object Transform puts the object's
    matrix there and the Position shows it too), so it is not multiplied in again."""
    obj = sim.findObject("collider")
    return obj.transform() if obj is not None else hou.Matrix4(1)


def _terrain_y(m, vol, world_pos):
    """The terrain's world y under world_pos (x, z), through the geometry-to-world matrix m."""
    minv = m.inverted()
    local = hou.Vector3(world_pos) * minv
    top = None
    # A few iterations: with rotation / shear, the point under world (x, z) moves with the height.
    for _ in range(8):
        h = vol.sample(hou.Vector3(local[0], 0.0, local[2]))
        top = hou.Vector3(local[0], h, local[2]) * m
        dx, dz = world_pos[0] - top[0], world_pos[2] - top[2]
        if abs(dx) < 1e-5 and abs(dz) < 1e-5:
            break
        d = hou.Vector4(dx, 0.0, dz, 0.0) * minv  # a direction: no translation
        local = hou.Vector3(local[0] + d[0], 0.0, local[2] + d[2])
    return top[1]


def _drop_on_heightfield(tag, snippet, transform, spots, size=0.3, parent_transform=None, obj_path=False):
    geo, sop = heightfield_collider(f"hf_{tag}", snippet, transform, parent_transform=parent_transform)
    vol = _height_volume(sop)
    pieces = [{"size": (size, size, size), "center": (x, 6.0, z)} for x, z in spots]
    path, names = packed_boxes(f"hf_boxes_{tag}", pieces)
    dop = new_dopnet(f"heightfield_{tag}")
    static = create(dop, "staticobject", "collider")
    set_parm(static, ["soppath"], sop.path())
    if obj_path:  # the transform as the DOP object's Position: translation and rotation only
        set_parm(static, ["usetransform"], 0)
        set_parm(static, ["objpath", "obj_path"], geo.path())
    else:  # Use Object Transform: the whole matrix, scale and shear included
        set_parm(static, ["usetransform"], 1)
    solver = box3d_solver(dop, merge(dop, [packed_object(dop, "boxes", path), static]))
    finish(dop, solver)
    msgs = []

    held = {}

    def sample(sim, f):
        if f == 120:
            msgs.extend(m for m in node_messages(solver) if "affector" in m or "height" in m)
            held["m"] = _sim_geometry_transform(sim)
            # The volume as the simulation holds it (Houdini may have baked the transform in).
            obj = sim.findObject("collider")
            g = obj.geometry() if obj is not None else None
            held["vol"] = next((pr for pr in (g.prims() if g is not None else ())
                                if isinstance(pr, hou.Volume) and pr.attribValue("name") == "height"), None)
        return packed_positions(sim, "boxes")

    track = step(dop, 120, sample)
    m = held["m"]
    if held.get("vol") is not None:
        vol = held["vol"]
    gaps = []
    for nm in names:
        p = track[120][nm]
        gaps.append(p[1] - _terrain_y(m, vol, (p[0], 0.0, p[2])))
    # Whether Houdini handed over the geo object's whole transform (scale included).
    diff = max(abs(a - b) for ra, rb in zip(m.asTupleOfTuples(), geo.worldTransform().asTupleOfTuples())
               for a, b in zip(ra, rb))
    msgs.insert(0, "the simulation holds the geo object's transform" if diff < 1e-4 else
                f"the simulation's transform differs from the geo object's by up to {diff:.3f} "
                "(expected terrain computed from the simulation's)")
    return gaps, msgs


def _collider_report(sim):
    """What the simulation holds for the collider: its transform and its height volume."""
    obj = sim.findObject("collider")
    if obj is None:
        return "no collider object"
    parts = []
    try:
        parts.append(f"object translate {tuple(round(v, 3) for v in obj.transform().extractTranslates())}")
    except Exception as e:  # noqa: BLE001 - a report, never a failure
        parts.append(f"no transform ({e})")
    data = obj.findSubData("Geometry")
    if data is not None:
        fields = []
        try:
            for rtype in data.recordTypes():
                for rec in data.records(rtype):
                    for fname in rec.fieldNames():
                        if any(k in fname.lower() for k in ("xform", "transform", "pivot", "pos", "orient")):
                            fields.append(f"{rtype}.{fname}={rec.field(fname)}")
        except Exception as e:  # noqa: BLE001 - a report, never a failure
            fields.append(f"(records unreadable: {e})")
        parts.append(f"Geometry data {data.dataType()}: {fields or 'no transform fields'}")
    geo = obj.geometry()
    if geo is None:
        return "; ".join(parts + ["no geometry"])
    kinds = sorted({p.type().name() for p in geo.prims()})
    parts.append(f"primitive types {kinds}")
    for prim in geo.prims():
        if isinstance(prim, hou.Volume):
            name = prim.attribValue("name") if geo.findPrimAttrib("name") else "(unnamed)"
            bb = prim.boundingBox()
            parts.append(f"volume {name}: resolution {prim.resolution()}, bounds min "
                         f"{tuple(round(v, 3) for v in bb.minvec())} max {tuple(round(v, 3) for v in bb.maxvec())}, "
                         f"index (0,0,0) at {tuple(round(v, 3) for v in prim.indexToPos((0, 0, 0)))}, "
                         f"rows of its transform {[tuple(round(v, 3) for v in r) for r in prim.transform().asTupleOfTuples()]}")
    return "; ".join(parts)


def _flat_variant(tag, by_obj_path, polygons=False):
    # A 10 x 10 field at height 2, its object moved by (3, 1, 0): it covers x -2..8, z -5..5 and
    # its top is at y = 3. Boxes over it should rest at 3.5; one beyond it falls past.
    # polygons: the same surface as a Grid SOP at y = 2 (the mesh path, as a control).
    if polygons:
        def build(g):
            grid = create(g, "grid")
            set_parm(grid, ["size"], (10, 10))
            set_parm(grid, ["t"], (0, 2, 0))
            return grid
        geo = new_geo(f"hf_flat_{tag}")
        sop = build(geo)
        display(sop)
        set_parm(geo, ["t"], (3, 1, 0))
    else:
        geo, sop = heightfield_collider(f"hf_flat_{tag}", "@height = 2;", {"t": (3, 1, 0)}, size=10.0)
    spots = [(x, z) for z in (-4, 0, 4) for x in (-1, 3, 7)]
    pieces = [{"size": (1, 1, 1), "center": (x, 6, z)} for x, z in spots]
    pieces.append({"size": (1, 1, 1), "center": (12, 6, 0)})
    path, names = packed_boxes(f"hf_flat_boxes_{tag}", pieces)
    dop = new_dopnet(f"heightfield_flat_{tag}")
    static = create(dop, "staticobject", "collider")
    set_parm(static, ["soppath"], sop.path())
    if by_obj_path:
        set_parm(static, ["usetransform"], 0)
        set_parm(static, ["objpath", "obj_path"], geo.path())
    else:
        set_parm(static, ["usetransform"], 1)  # off by default
    solver = box3d_solver(dop, merge(dop, [packed_object(dop, "boxes", path), static]))
    finish(dop, solver)
    report = []

    def sample(sim, f):
        if f == 1:
            report.append(_collider_report(sim))
        return packed_positions(sim, "boxes")

    track = step(dop, 120, sample)  # bouncy contacts: give them time to settle
    on = [track[120][nm] for nm in names[:-1]]
    off = track[120][names[-1]][1]
    ok = all(close(p[1], 3.5, 0.03) for p in on) and off < 0.0
    label = "moved by OBJ Path (Use Object Transform off)" if by_obj_path else "Use Object Transform on"
    if polygons:
        label = "control, a polygon grid instead: " + label
    rest = ", ".join(f"({x}, {z}): {p[1]:.3f}" for (x, z), p in zip(spots, on))
    return Result("heightfield_flat", Result.PASS if ok else Result.FAIL,
                  f"{label}: boxes at (x, z) rest at y {rest} (expected 3.5 each); box beyond the field at "
                  f"y={off:.2f} (expected below 0). In the simulation on frame 1: {report[0] if report else '?'}")


def test_heightfield_flat():
    """Boxes dropped on a flat height field (height 2) of a Static Object whose geo object is moved
    by (3, 1, 0) rest on it at y = 3.5 wherever they land; a box beyond the field falls past. Once
    with Use Object Transform, once moved by OBJ Path. Reports the collider as the simulation
    holds it (object transform, volume bounds and transform)."""
    return [_flat_variant("objtransform", False), _flat_variant("objpath", True),
            _flat_variant("grid_objtransform", False, polygons=True)]


TERRAIN = "@height = 1.5 * sin(@P.x * 0.3) * cos(@P.z * 0.25) + 0.1 * @P.z;"
TERRAIN_SPOTS = [(-5, -4), (-3, 4), (0, 0), (2, -5), (5, 3), (5, -2), (-5, 5), (3, 6)]


def _terrain_result(name, tag, transform, path_note, parent_transform=None, obj_path=False):
    try:
        gaps, msgs = _drop_on_heightfield(tag, TERRAIN, transform, TERRAIN_SPOTS,
                                          parent_transform=parent_transform, obj_path=obj_path)
    except SetupError as e:
        return Result(name, Result.SETUP, f"{path_note}: {e}")
    # A 0.3 box resting on the slope: its centre sits 0.15 to ~0.2 above the terrain under it.
    bad = [round(g, 3) for g in gaps if not (0.1 < g < 0.3)]
    return Result(name, Result.FAIL if bad else Result.PASS,
                  f"{path_note}: box centre above the terrain under it: {[round(g, 3) for g in gaps]} "
                  f"(expected 0.15 to 0.2){'; off: ' + str(bad) if bad else ''}"
                  f"{'; ' + '; '.join(msgs) if msgs else ''}")


def test_heightfield_terrain():
    """Small boxes dropped on a bumpy, gently sloping height field (30 x 30, the boxes well inside it,
    Use Object Transform on) come to rest on its surface: each box
    centre ends 0.15-0.2 above the terrain height sampled under it (hou.Volume.sample), so the
    orientation, spacing and heights match Houdini's. Plain, then moved, turned and scaled (a Box3D
    height field posed by the transform), then sheared (built as triangles instead)."""
    return [
        _terrain_result("heightfield_terrain", "plain", None, "untransformed"),
        _terrain_result("heightfield_terrain", "posed",
                        {"t": (1, 0.5, -2), "r": (0, 30, 0), "s": (1.5, 1.0, 0.8)},
                        "moved, turned 30 degrees about y and scaled (1.5, 1, 0.8)"),
        _terrain_result("heightfield_terrain", "objpath",
                        {"t": (1, 0.5, -2), "r": (0, 30, 0), "s": (1.5, 1.0, 0.8)},
                        "the same moved by OBJ Path (Houdini passes translation and rotation only)",
                        obj_path=True),
        _terrain_result("heightfield_terrain", "sheared", {"r": (0, 30, 0)},
                        "sheared: turned 30 degrees about y under a parent scaled (1.3, 1, 1) "
                        "(triangle fallback)", parent_transform={"s": (1.3, 1.0, 1.0)}),
    ]


def test_heightfield_animated():
    """A height field Static Object whose geo object rises from y = 0 to 1 over the first second:
    a box on it ends at 2.5 on the flat field at height 1 (the body's pose follows the object
    without a rebuild)."""
    geo, sop = heightfield_collider("hf_anim", "@height = 1;", None, size=10.0)
    set_expression(geo, ["ty"], "min($T, 1)")
    path, names = packed_boxes("hf_anim_box", [{"size": (1, 1, 1), "center": (0, 5, 0)}])
    dop = new_dopnet("heightfield_animated")
    static = create(dop, "staticobject", "collider")
    set_parm(static, ["soppath"], sop.path())
    set_parm(static, ["usetransform"], 0)
    set_parm(static, ["objpath", "obj_path"], geo.path())
    finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "box", path), static])))
    track = step(dop, 72, lambda sim, f: packed_positions(sim, "box"))
    y = track[72][names[0]][1]
    return [Result("heightfield_animated", Result.PASS if close(y, 2.5, 0.03) else Result.FAIL,
                   f"box on the risen field at y={y:.3f} (expected 2.5)")]


TESTS = {
    "heightfield_flat": test_heightfield_flat,
    "heightfield_terrain": test_heightfield_terrain,
    "heightfield_animated": test_heightfield_animated,
}
