"""Points, packed geometry and outputs: collision masks, packed pivots, Packed Disk / Alembic,
primitive spheres, mirrored pieces; written v / w, Impacts, mass, aging, point force / torque,
Gravity Force DOPs."""

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
    single_object_geo,
    step,
    set_menu_token,
)

G = 9.80665


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
        rows.append(f"{label}: rests spanning y {lo[1]:.2f} to {hi[1]:.2f} (correct: 0 to 1)")
    return [Result("odd_packed_contents", Result.INFO, "; ".join(rows))]


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


# ---------------------------------------------------------------------------
# Impacts (status: addPointImpact arguments; impulse in N*s)
# ---------------------------------------------------------------------------


def impact_records(sim, obj_name):
    """This frame's impact records as plain {field: value} dicts. (DopRecord objects are live: read
    later, they show whatever frame is current then.)"""
    obj = sim.findObject(obj_name)
    data = obj.findSubData("Impacts") if obj is not None else None
    if data is None:
        return []
    for rtype in data.recordTypes():
        if "mpact" in rtype:
            return [{n: r.field(n) for n in r.fieldNames()} for r in data.records(rtype)]
    return []


def test_impacts():
    """With Add Impact Data, a 1 kg box dropped 1 m records impacts whose impulses on the landing
    frame add up to about m*v (4.4 N*s), with its point number and a time."""
    path, names = packed_boxes("impact_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
    dop = new_dopnet("impacts")
    obj = packed_object(dop, "piece", path, computemass=0, mass=1.0, bounce=0.0)
    data = box3d_data(dop, obj, "data", addimpact=1)
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, "impacts_ground", bounce=0.0), data])))
    track = step(dop, 30, lambda sim, f: impact_records(sim, "piece"))
    first = next((f for f in sorted(track) if track[f]), None)
    if first is None:
        return [Result("impacts", Result.FAIL, "no Impacts records on any frame")]
    recs = track[first]
    fields = list(recs[0])
    imp_field = next((n for n in fields if "impulse" in n.lower()), None)
    if imp_field is None:
        raise SetupError(f"Impacts records have no impulse field; fields: {fields}")
    total = sum(abs(r[imp_field]) if not isinstance(r[imp_field], (tuple, list))
                else math.sqrt(sum(c * c for c in r[imp_field])) for r in recs)
    expected = math.sqrt(2 * G * 1.0)
    extras = {n: recs[0][n] for n in fields if n.lower() in ("ptnum", "time", "otherobjid", "primnum")}
    results = [Result("impacts", Result.PASS if 0.7 * expected <= total <= 1.6 * expected else Result.FAIL,
                      f"frame {first}: {len(recs)} record(s), impulses sum to {total:.2f} N*s (expected about "
                      f"{expected:.2f})")]
    # The box is point 0 of its object; the time is the end of the step that hit (frame `first`).
    ptnum = next((recs[0][n] for n in fields if n.lower() == "ptnum"), None)
    t = next((recs[0][n] for n in fields if n.lower() == "time"), None)
    t_expected = (first - 1) / hou.fps()
    ok = ptnum == 0 and t is not None and abs(t - t_expected) <= 1.0 / hou.fps() + 1e-6
    results.append(Result("impacts", Result.PASS if ok else Result.FAIL,
                          f"first record: point number {ptnum} (expected 0), time {t} (expected about "
                          f"{t_expected:.4f}); all fields {extras}"))
    return results


# ---------------------------------------------------------------------------
# mass written only with computemass (status: mass is written only with computemass)
# ---------------------------------------------------------------------------


def test_mass_written():
    """With Compute Mass on (density 500, set on RBD Packed Object, which writes them onto the
    points), the solver writes mass = 500 for a 1 m cube; with it off the mass attribute (3) is
    left alone."""
    rows, ok = [], True
    for compute in (1, 0):
        path, names = packed_boxes(f"mass_{compute}", [{"size": (1, 1, 1), "center": (0, 0.5, 0)}])
        dop = new_dopnet(f"mass_{compute}")
        obj = packed_object(dop, "piece", path, computemass=compute, density=500.0, mass=3.0)
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop, f"mass_ground_{compute}"), obj])))
        attrs = step(dop, 3, lambda sim, f: point_attributes(sim, "piece", names[0], ["mass"]))
        before, after = attrs[1]["mass"], attrs[3]["mass"]
        rows.append(f"Compute Mass {'on' if compute else 'off'}: mass {before} on frame 1, {after} on frame 3")
        if compute and not (after is not None and close(after, 500.0, 1.0)):
            ok = False
        if not compute and not (after is not None and close(after, 3.0, 1e-3)):
            ok = False
    return [Result("mass_written", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# Aging (status: age counts DOP seconds, dead once age > life, life 0 never dies)
# ---------------------------------------------------------------------------


def test_aging():
    """With Enable Aging, age grows by one per second (frames 2..25: the object is not solved on its
    creation frame). A piece with life 0.5 is dead after a second and, with Reap Dead Points off (the
    default), kept; one with life 0 never dies."""
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
    rows = [f"life 0 piece: age {age:.4f} on frame 25 (1.0 expected), dead={forever['dead']}",
            "life 0.5 piece: " + ("deleted (should be kept: Reap Dead Points is off)" if short_gone
                                  else f"kept, age {short['age']:.3f}, dead={short['dead']}")]
    ok = close(age, 1.0, 1e-3) and not forever["dead"] and not short_gone and short["dead"] == 1
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
    obj = packed_object(dop, "pieces", path, computemass=0, mass=1.0)
    finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))

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


def test_force_dops_many_and_sampled():
    """Force DOPs on many bodies and over a body's size. 300 boxes under a Gravity Force DOP (the Force
    DOPs of an object are evaluated in parallel from a few hundred bodies) all take the same pull:
    v.y = -g after 1 s. A Drag Force DOP slows a 2 x 2 x 2 box by a different amount when its Sample Mode
    is Sphere or Circle (force over the body's size) than at a Point (INFO: the ratios)."""
    results = []
    count = 300
    pieces = [{"size": (1, 1, 1), "center": (3.0 * i, 50, 0)} for i in range(count)]
    path, names = packed_boxes("force_many", pieces)
    dop = new_dopnet("force_many")
    solver = box3d_solver(dop, packed_object(dop, "piece", path), gravity=(0, 0, 0))
    grav = create(dop, "gravity", "gravity_force")
    grav.setInput(0, solver)
    finish(dop, grav)
    attrs = step(dop, 25, lambda sim, f: [point_attributes(sim, "piece", n, ["v"])["v"][1] for n in names]
                 if f == 25 else None)[25]
    lo, hi = min(attrs), max(attrs)
    ok = close(lo, -G, 0.03 * G) and close(hi, -G, 0.03 * G)
    results.append(Result("force_dops_many_and_sampled", Result.PASS if ok else Result.FAIL,
                          f"{count} boxes under a Gravity Force DOP: v.y after 1 s from {lo:.3f} to {hi:.3f} "
                          f"(expected {-G:.3f} for each)"))

    slowed = {}
    for mode, label in ((1, "Point"), (2, "Circle"), (3, "Sphere")):
        path, names = packed_boxes(f"force_drag_{label.lower()}",
                                   [{"size": (2, 2, 2), "center": (0, 50, 0), "attrs": {"v": (10.0, 0.0, 0.0)}}])
        dop = new_dopnet(f"force_drag_{label.lower()}")
        solver = box3d_solver(dop, packed_object(dop, "piece", path, keep_velocity=True), gravity=(0, 0, 0))
        drag = create(dop, "drag", "drag_force")
        drag.setInput(0, solver)
        set_parm(drag, ["samplemode"], mode)
        finish(dop, drag)
        vx = step(dop, 7, lambda sim, f: point_attributes(sim, "piece", names[0], ["v"]))[7]["v"][0]
        slowed[label] = 10.0 - vx
    point = slowed["Point"]
    results.append(Result("force_dops_many_and_sampled",
                          Result.PASS if point > 1e-4 and all(
                              abs(slowed[k] - point) > 0.1 * point for k in ("Circle", "Sphere")) else Result.FAIL,
                          "Drag Force DOP on a 2 x 2 x 2 box going 10 m/s, speed lost in 0.25 s: "
                          + ", ".join(f"{k} {v:.4f}" for k, v in slowed.items())
                          + " (expected Circle and Sphere to differ from Point, which they do when the force is "
                            "evaluated over the body's size)"))
    if point > 1e-4:
        results.append(Result("force_dops_many_and_sampled", Result.INFO,
                              f"ratios to Point: Circle {slowed['Circle'] / point:.2f}, Sphere {slowed['Sphere'] / point:.2f}"))
    return results


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
    label.set((10,))
    finish(dop, box3d_solver(dop, obj))
    seen = step(dop, 12, lambda sim, f: sim.findObject("pieces") is not None)
    first = next((f for f in sorted(seen) if seen[f]), None)
    results.append(Result("box3d_packed_object", Result.PASS if first == 10 else Result.FAIL,
                          f"Creation Frame 10: object first present on frame {first} (expected 10)"))
    return results


def test_packed_object_initial_state():
    """The Box3D Packed Object's Initial State tab, as RBD Packed Object's. Inherit Velocity (the default)
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


def test_packed_object_deforming():
    """The Box3D Packed Object's Deforming toggle writes deforming 1 on the points (off: none) and reads
    the SOP every step, as Overwrite Attributes from SOP does: a Kinematic piece whose SOP object rises
    0.1 per frame follows it (y 3 + 1.2 on frame 13) with Deforming on, and stays at 3 with it off."""
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
        good = close(y, 4.2, 0.05) if on else close(y, 3.0, 0.05)
        rows.append(f"Deforming {'on' if on else 'off'}: y {y:.3f} on frame 13 (expected {4.2 if on else 3.0})")
        ok = ok and good
    results.append(Result("packed_object_deforming", Result.PASS if ok else Result.FAIL, "; ".join(rows)))
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

    def build(geo):
        grid = create(geo, "grid")
        set_parm(grid, ["size"], (10, 10))
        set_parm(grid, ["rows"], 2)
        set_parm(grid, ["cols"], 2)
        wr = create(geo, "attribwrangle")
        wr.setInput(0, grid)
        set_parm(wr, ["class"], 2)
        set_parm(wr, ["snippet"], "@P.y = 3;")
        return wr

    obj = create(dop, "rbdpointobject", "objects")
    set_parm(obj, ["soppath"], single_object_geo(f"pointobj_{tag}_geo", build))
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


TESTS = {
    "rbd_packed_attributes": test_rbd_packed_attributes,
    "rest_reset": test_rest_reset,
    "packed_object_attributes": test_packed_object_attributes,
    "packed_object_initial_state": test_packed_object_initial_state,
    "packed_object_deforming": test_packed_object_deforming,
    "tab_submenu": test_tab_submenu,
    "packed_object_use_transform": test_packed_object_use_transform,
    "data_under_geometry": test_data_under_geometry,
    "box3d_packed_object": test_box3d_packed_object,
    "unpacked_geometry": test_unpacked_geometry,
    "unpacked_geometry_bullet": test_unpacked_geometry_bullet,
    "unpacked_geometry_match": test_unpacked_geometry_match,
    "point_object_bullet": test_point_object_bullet,
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
    "force_dops_many_and_sampled": test_force_dops_many_and_sampled,
}
