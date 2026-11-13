"""Points, packed geometry and outputs: collision masks, packed pivots, Packed Disk / Alembic,
primitive spheres, mirrored pieces; written v / w, Impacts, mass, aging, point force / torque,
Gravity Force DOPs."""

import math
import os
import tempfile

import hou

from b3dtest import (
    Result,
    SetupError,
    box3d_data,
    box3d_solver,
    close,
    create,
    det3,
    display,
    finish,
    merge,
    new_dopnet,
    new_geo,
    object_bbox,
    packed_boxes,
    packed_object,
    packed_positions,
    packed_transform,
    point_attributes,
    set_menu_by_label,
    set_parm,
    step,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}
G = 9.80665


def ground_object(dop, name):
    path, _ = packed_boxes(name + "_geo", [GROUND])
    return packed_object(dop, name, path)


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
# collision_category / collision_mask / collision_group (README's collision rules)
# ---------------------------------------------------------------------------


def test_collision_masks():
    """README's collision rules on packed pieces: a mask of -1 collides with everything, a mask
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
vector piv = {0.4, 0.4, 0.0};
vector old = primintrinsic(0, "pivot", @primnum);
matrix3 m = primintrinsic(0, "transform", @primnum);
setprimintrinsic(0, "pivot", @primnum, piv);
int pt = primpoint(0, @primnum, 0);
setpointattrib(0, "P", pt, point(0, "P", pt) + (piv - old) * m);
""")
    display(wr)
    start_min = wr.geometry().boundingBox().minvec()[1]
    _, track = drop_one("pivot", wr.path())
    lo, hi = track[72]
    ok = close(lo[1], 0.0, 0.02) and close(hi[1] - lo[1], 1.0, 0.02)
    return [Result("offset_pivot", Result.PASS if ok else Result.FAIL,
                   f"pivot moved off the centroid (geometry bottom started at y={start_min:.2f}); at rest the box spans "
                   f"y {lo[1]:.3f} to {hi[1]:.3f} (expected 0 to 1)")]


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
    set_menu_by_label(f, ["loadtype"], "Packed Disk")
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
# Primitive spheres and nested packing (status: expected to be wrong; record what happens)
# ---------------------------------------------------------------------------


def test_odd_packed_contents():
    """What happens to a packed primitive sphere (radius 0.5) and to a packed box inside another
    packed primitive. INFO: the notes expect hulls from points only to be wrong here."""
    rows = []
    for label in ("primitive sphere", "nested packed box"):
        geo = new_geo("odd_" + label.split()[0])
        if label == "primitive sphere":
            src = create(geo, "sphere")
            set_menu_by_label(src, ["type"], "Primitive")
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
        rows.append(f"{label}: rests spanning y {lo[1]:.2f} to {hi[1]:.2f} (correct: 0 to 1)")
    return [Result("odd_packed_contents", Result.INFO, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Mirrored packed transform (status: mirrored packed transforms)
# ---------------------------------------------------------------------------


def test_mirrored_piece():
    """A packed piece with a mirrored transform (scale -1 in x): does it land correctly, and is it
    still mirrored once it moves? INFO: the notes say the write-back drops the mirror."""
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


# ---------------------------------------------------------------------------
# Written v / w (status: downstream readers get v of the body origin and w in radians/s)
# ---------------------------------------------------------------------------


def test_written_velocity():
    """The solver writes v and w on the points: a falling box has v.y = -g t after one second, and
    a spinning box (no gravity) keeps w = 1 rad/s and turns 1 radian in a second."""
    results = []
    path, names = packed_boxes("vel_fall", [{"size": (1, 1, 1), "center": (0, 50, 0)}])
    dop = new_dopnet("vel_fall")
    finish(dop, box3d_solver(dop, packed_object(dop, "piece", path)))
    attrs = step(dop, 25, lambda sim, f: point_attributes(sim, "piece", names[0], ["v", "w"]))
    vy = attrs[25]["v"][1]
    results.append(Result("written_velocity", Result.PASS if close(vy, -G, 0.02 * G) else Result.FAIL,
                          f"falling box after 1 s: v.y = {vy:.3f} (expected {-G:.3f})"))

    path, names = packed_boxes("vel_spin", [{"size": (1, 1, 1), "center": (0, 0, 0), "attrs": {"w": (0.0, 1.0, 0.0)}}])
    dop = new_dopnet("vel_spin")
    finish(dop, box3d_solver(dop, packed_object(dop, "piece", path, keep_velocity=True), gravity=(0, 0, 0)))

    def sample(sim, f):
        return point_attributes(sim, "piece", names[0], ["w"]), packed_transform(sim, "piece")

    track = step(dop, 25, sample)
    w = track[25][0]["w"]
    r = track[25][1]
    angle = math.atan2(-r[0][2], r[0][0])  # rotation of the x axis about y
    ok = close(w[1], 1.0, 0.02) and close(angle, 1.0, 0.03)
    results.append(Result("written_velocity", Result.PASS if ok else Result.FAIL,
                          f"spinning box after 1 s: w = ({w[0]:.3f}, {w[1]:.3f}, {w[2]:.3f}) rad/s, turned {angle:.3f} rad "
                          "(expected w.y = 1 and 1 rad)"))
    return results


# ---------------------------------------------------------------------------
# Impacts (status: addPointImpact arguments; impulse in N*s)
# ---------------------------------------------------------------------------


def impact_records(sim, obj_name):
    obj = sim.findObject(obj_name)
    data = obj.findSubData("Impacts") if obj is not None else None
    if data is None:
        return []
    for rtype in data.recordTypes():
        if "mpact" in rtype:
            return list(data.records(rtype))
    return []


def test_impacts():
    """With Add Impact Data, a 1 kg box dropped 1 m records impacts whose impulses on the landing
    frame add up to about m*v (4.4 N*s), with its point number and a time."""
    path, names = packed_boxes("impact_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
    dop = new_dopnet("impacts")
    obj = packed_object(dop, "piece", path)
    data = box3d_data(dop, obj, "data", addimpact=1, computemass=0, mass=1.0, bounce=0.0)
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "impacts_ground"), data])))
    track = step(dop, 30, lambda sim, f: impact_records(sim, "piece"))
    first = next((f for f in sorted(track) if track[f]), None)
    if first is None:
        return [Result("impacts", Result.FAIL, "no Impacts records on any frame")]
    recs = track[first]
    fields = list(recs[0].fieldNames())
    imp_field = next((n for n in fields if "impulse" in n.lower()), None)
    if imp_field is None:
        raise SetupError(f"Impacts records have no impulse field; fields: {fields}")
    total = sum(abs(r.field(imp_field)) if not isinstance(r.field(imp_field), (tuple, list))
                else math.sqrt(sum(c * c for c in r.field(imp_field))) for r in recs)
    expected = math.sqrt(2 * G * 1.0)
    extras = {n: recs[0].field(n) for n in fields if n.lower() in ("ptnum", "time", "otherobjid", "primnum")}
    ok = 0.7 * expected <= total <= 1.6 * expected
    return [Result("impacts", Result.PASS if ok else Result.FAIL,
                   f"frame {first}: {len(recs)} record(s), impulses sum to {total:.2f} N*s (expected about "
                   f"{expected:.2f}); first record {extras}")]


# ---------------------------------------------------------------------------
# mass written only with computemass (status: mass is written only with computemass)
# ---------------------------------------------------------------------------


def test_mass_written():
    """With Compute Mass on (density 500), the points get mass = 500 for a 1 m cube; with it off
    the mass attribute is not changed by the solver."""
    rows, ok = [], True
    for compute in (1, 0):
        path, names = packed_boxes(f"mass_{compute}", [{"size": (1, 1, 1), "center": (0, 0.5, 0)}])
        dop = new_dopnet(f"mass_{compute}")
        obj = packed_object(dop, "piece", path)
        data = box3d_data(dop, obj, "data", computemass=compute, density=500.0, mass=3.0)
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, f"mass_ground_{compute}"), data])))
        attrs = step(dop, 3, lambda sim, f: point_attributes(sim, "piece", names[0], ["mass"]))
        before, after = attrs[1]["mass"], attrs[3]["mass"]
        rows.append(f"Compute Mass {'on' if compute else 'off'}: mass {before} on frame 1, {after} on frame 3")
        if compute and not (after is not None and close(after, 500.0, 1.0)):
            ok = False
        if not compute and after != before:
            ok = False
    return [Result("mass_written", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Aging (status: age counts DOP seconds, dead once age > life, life 0 never dies)
# ---------------------------------------------------------------------------


def test_aging():
    """With Enable Aging, age grows by one per second; a piece with life 0.5 is dead (or deleted)
    after a second, one with life 0 never dies."""
    pieces = [
        {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"life": 0.5, "age": 0.0}},
        {"size": (1, 1, 1), "center": (3, 0.5, 0), "attrs": {"life": 0.0, "age": 0.0}},
    ]
    path, names = packed_boxes("aging_pieces", pieces)
    dop = new_dopnet("aging")
    obj = packed_object(dop, "pieces", path)
    solver = box3d_solver(dop, merge(dop, [ground_object(dop, "aging_ground"), obj]))
    set_parm(solver, ["enableaging"], 1)
    finish(dop, solver)

    def sample(sim, f):
        return {n: point_attributes(sim, "pieces", n, ["age", "dead", "name"]) for n in names}

    track = step(dop, 25, sample)
    short, forever = track[25][names[0]], track[25][names[1]]
    short_gone = short is None or short.get("name") != names[0]
    age = forever["age"]
    rows = [f"life 0 piece: age {age:.3f} after 1 s, dead={forever['dead']}",
            "life 0.5 piece: " + ("deleted" if short_gone else f"age {short['age']:.3f}, dead={short['dead']}")]
    ok = close(age, 1.0, 0.05) and not forever["dead"] and (short_gone or short["dead"] == 1)
    return [Result("aging", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Point force / torque (status: world N and N*m, one step, then zeroed)
# ---------------------------------------------------------------------------


def test_point_force():
    """A point `force` of 24 N on a 1 kg box (no gravity) adds 1 m/s in its first 1/24 s step and
    is then zeroed; a `torque` of 1 N*m on a 1 kg unit box (I = 1/6) adds 0.25 rad/s."""
    pieces = [
        {"size": (1, 1, 1), "center": (0, 0, 0), "attrs": {"force": (24.0, 0.0, 0.0), "torque": (0.0, 0.0, 0.0)}},
        {"size": (1, 1, 1), "center": (5, 0, 0), "attrs": {"force": (0.0, 0.0, 0.0), "torque": (0.0, 0.0, 1.0)}},
    ]
    path, names = packed_boxes("force_pieces", pieces)
    dop = new_dopnet("force")
    obj = packed_object(dop, "pieces", path)
    data = box3d_data(dop, obj, "data", computemass=0, mass=1.0)
    finish(dop, box3d_solver(dop, data, gravity=(0, 0, 0)))

    def sample(sim, f):
        return {n: point_attributes(sim, "pieces", n, ["v", "w", "force"]) for n in names}

    track = step(dop, 6, sample)
    v2, v6 = track[2][names[0]]["v"][0], track[6][names[0]]["v"][0]
    f3 = track[3][names[0]]["force"]
    w2 = track[2][names[1]]["w"][2]
    ok = close(v2, 1.0, 0.05) and close(v6, v2, 1e-3) and f3 is not None and max(abs(c) for c in f3) < 1e-6 \
        and close(w2, 0.25, 0.02)
    return [Result("point_force", Result.PASS if ok else Result.FAIL,
                   f"force: v.x {v2:.3f} after the first step, {v6:.3f} on frame 6 (expected 1 and unchanged), force "
                   f"on frame 3 = {f3}; torque: w.z {w2:.3f} (expected 0.25)")]


# ---------------------------------------------------------------------------
# Gravity Force DOP adds to the solver's Gravity (status: Force DOPs evaluated once per step)
# ---------------------------------------------------------------------------


def test_gravity_dop():
    """A Gravity Force DOP pulls at g with the solver's Gravity at 0, and adds to it (2g) with the
    solver's own Gravity on."""
    rows, ok = [], True
    for solver_g, expected in (((0, 0, 0), -G), (None, -2 * G)):
        tag = "own_off" if solver_g else "own_on"
        path, names = packed_boxes("gravity_" + tag, [{"size": (1, 1, 1), "center": (0, 50, 0)}])
        dop = new_dopnet("gravity_" + tag)
        solver = box3d_solver(dop, packed_object(dop, "piece", path), gravity=solver_g)
        grav = create(dop, "gravity", "gravity_force")
        grav.setInput(0, solver)
        finish(dop, grav)
        vy = step(dop, 25, lambda sim, f: point_attributes(sim, "piece", names[0], ["v"]))[25]["v"][1]
        rows.append(f"solver Gravity {'0' if solver_g else 'on'}: v.y after 1 s = {vy:.2f} (expected {expected:.2f})")
        ok = ok and close(vy, expected, 0.03 * abs(expected))
    return [Result("gravity_dop", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS = {
    "collision_masks": test_collision_masks,
    "offset_pivot": test_offset_pivot,
    "packed_disk": test_packed_disk,
    "packed_alembic": test_packed_alembic,
    "odd_packed_contents": test_odd_packed_contents,
    "mirrored_piece": test_mirrored_piece,
    "written_velocity": test_written_velocity,
    "impacts": test_impacts,
    "mass_written": test_mass_written,
    "aging": test_aging,
    "point_force": test_point_force,
    "gravity_dop": test_gravity_dop,
}
