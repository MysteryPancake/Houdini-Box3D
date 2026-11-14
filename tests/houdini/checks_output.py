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
    box3d_packed_object,
    box3d_solver,
    close,
    create,
    det3,
    display,
    finish,
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
    set_menu_by_label,
    set_parm,
    step,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}
G = 9.80665


def ground_object(dop, name, **parms):
    """A static packed ground box (top at y = 0). parms: RBD Packed Object parameters, e.g.
    bounce=0.0 (its default 0.5 is written onto the point)."""
    path, _ = packed_boxes(name + "_geo", [GROUND])
    return packed_object(dop, name, path, **parms)


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


def test_box3d_packed_object():
    """The Box3D Packed Object asset brings packed geometry in without RBD Packed Object's stamped
    attributes, so its Box3D Data settings apply: a box pushed at 3 m/s with its Friction 0 slides
    about 3 m in 1 s (RBD Packed Object with a Box3D Data Friction 0 stops: its stamped friction 1
    wins, and the solver warns). A point's own attribute still wins (friction 1 on the point: it
    stops). A dropped box falls as with RBD Packed Object, and Creation Frame 10 makes the object
    appear on frame 10."""
    results = []
    x_ours, w_ours = _push_test("ours", lambda dop, p: box3d_packed_object(dop, "pieces", p, friction=0.0))
    x_rbd, w_rbd = _push_test("rbd", lambda dop, p: box3d_data(
        dop, packed_object(dop, "pieces", p, keep_velocity=True), "data", friction=0.0))
    x_attr, _ = _push_test("attr", lambda dop, p: box3d_packed_object(dop, "pieces", p, friction=0.0),
                           {"friction": 1.0})
    # Does the asset's Friction reach the Box3D Data inside it?
    probe_dop = new_dopnet("probe_link")
    probe = box3d_packed_object(probe_dop, "pieces", "", friction=0.25)
    inner = probe.node("box3d_data")
    inner_f = inner.parm("friction") if inner is not None else None
    op = inner.parm("parmop_friction") if inner is not None else None
    link_note = (f"inside, box3d_data Friction = {inner_f.eval()} after setting the asset's to 0.25 "
                 f"(references {inner_f.getReferencedParm().path()}; its operation "
                 f"{op.evalAsString() if op is not None else '?'})" if inner_f is not None else
                 f"no box3d_data inside the asset (children: {[c.name() for c in probe.children()]})")
    def warnings(w):
        return [m for m in w if not m.startswith("[")]

    ok = x_ours > 2.5 and not warnings(w_ours) and x_rbd < 2.0 and bool(warnings(w_rbd)) and x_attr < 2.0
    results.append(Result("box3d_packed_object", Result.PASS if ok else Result.FAIL,
                          f"Friction 0 on Box3D Packed Object: box at x={x_ours:.2f} after 1 s (expected about 3, "
                          f"slid), warnings {warnings(w_ours) or 'none'} (expected none) "
                          f"{' '.join(m for m in w_ours if m.startswith('['))}; RBD Packed Object + Box3D Data "
                          f"Friction 0: x={x_rbd:.2f} (expected stopped, below 2), warning "
                          f"{'shown' if warnings(w_rbd) else 'none'} (expected shown); point friction 1 on Box3D Packed "
                          f"Object: x={x_attr:.2f} (expected stopped: the point's value wins); {link_note}"))

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


def test_data_under_geometry():
    """Box3D Data attaches under the object's Geometry (Geometry/Box3DData), as Bullet Data does
    (Geometry/BulletData), not at the object's top level; it is still there after the solver has
    replaced the geometry with its own writable copy, and its settings still reach the bodies
    (Friction 0: a pushed box slides)."""
    path, names = packed_boxes("data_geo", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0),
                                                     "attrs": {"v": (3.0, 0.0, 0.0)}}])
    dop = new_dopnet("data_geo")
    # Box3D Packed Object's own Box3D Data (Friction 0 on the asset). A second Box3D Data after it
    # would not change it: its Set Initial values only apply when it creates the data, and the
    # asset already has (Set Always would).
    obj = box3d_packed_object(dop, "pieces", path, friction=0.0)
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
    "data_under_geometry": test_data_under_geometry,
    "box3d_packed_object": test_box3d_packed_object,
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
