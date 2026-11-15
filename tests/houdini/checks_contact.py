"""Contact behaviour: slopes, bounce heights, CCD, sleep, rolling resistance, shape types, autofit."""

import math

import hou

from b3dtest import (
    box3d_data,
    box3d_solver,
    close,
    create,
    display,
    finish,
    GROUND,
    merge,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    point_attributes,
    Result,
    set_parm,
    solver_for,
    step,
    vex_value,
)
from checks_common import G, box3d_messages


# ---------------------------------------------------------------------------
# Contact behaviour
# ---------------------------------------------------------------------------


def test_friction_slope():
    """A box (friction 0.5) on a ramp (friction 1): under the default Multiply mixing the contact
    has friction 0.5, so it should start sliding above atan 0.5 = 26.6 degrees: it stays put at
    22 degrees and slides at 31."""
    rows, ok = [], True
    for angle, should_slide in ((22.0, False), (31.0, True)):
        t = math.radians(angle)
        n = (-math.sin(t), math.cos(t), 0.0)  # the ramp's surface normal
        pieces = [{"size": (20, 1, 20), "center": tuple(-0.5 * c for c in n), "rotate": (0, 0, angle),
                   "attrs": {"active": 0, "friction": 1.0, "bounce": 0.0, "bullet_georep": "convexhull"}},
                  {"size": (1, 1, 1), "center": tuple(0.501 * c for c in n), "rotate": (0, 0, angle),
                   "attrs": {"active": 1, "friction": 0.5, "bounce": 0.0, "bullet_georep": "convexhull"}}]
        path, names = packed_boxes(f"slope_{int(angle)}_pieces", pieces)
        dop = new_dopnet(f"slope_{int(angle)}")
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
        track = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
        moved = (track[48] - track[2]).length()
        expected = 0.5 * G * (math.sin(t) - 0.5 * math.cos(t)) * (46 / hou.fps()) ** 2
        good = moved > 0.3 if should_slide else moved < 0.05
        ok = ok and good
        rows.append(f"{angle:g} degrees: moved {moved:.3f} in 2 s ("
                    + (f"slides; about {expected:.2f} expected)" if should_slide else "expected to stay put)"))
    return [Result("friction_slope", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def ccd_run(name, wall_active, fast, continuous=True):
    """A 0.2 box fired at 200 m/s at a 0.05 thick wall, no gravity. Returns (projectile x, wall x)
    after 6 frames."""
    pieces = [{"size": (0.05, 4, 4), "center": (0, 0, 0),
               "attrs": {"active": wall_active, "fastobject": 0, "v": (0.0, 0.0, 0.0)}},
              {"size": (0.2, 0.2, 0.2), "center": (-3, 0, 0),
               "attrs": {"active": 1, "fastobject": fast, "v": (200.0, 0.0, 0.0)}}]
    path, names = packed_boxes(name + "_pieces", pieces)
    dop = new_dopnet(name)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True), gravity=(0, 0, 0))
    set_parm(solver, ["enablecontinuous"], 1 if continuous else 0)
    finish(dop, solver)
    pos = step(dop, 6, lambda sim, f: packed_positions(sim, "pieces"))[6]
    return pos[names[1]][0], pos[names[0]][0]


def test_ccd():
    """A small box at 200 m/s against a thin wall: a static wall stops it with Continuous Collision
    and is tunnelled without; a moving wall stops it only when the box is a Fast Object."""
    rows, ok = [], True
    cases = (("static wall, Continuous on", 0, 0, True, True),
             ("static wall, Continuous off", 0, 0, False, False),
             ("moving wall, Fast Object on", 1, 1, True, True),
             ("moving wall, Fast Object off", 1, 0, True, None))
    for i, (label, wall_active, fast, continuous, want_stop) in enumerate(cases):
        px, wx = ccd_run(f"ccd_{i}", wall_active, fast, continuous)
        stopped = px < wx
        if want_stop is not None:
            ok = ok and stopped == want_stop
        rows.append(f"{label}: {'stopped' if stopped else 'went through'} (box x {px:.2f}, wall x {wx:.2f})"
                    + ("" if want_stop is None else f", expected {'stopped' if want_stop else 'through'}"))
    return [Result("ccd", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_sleep():
    """A box resting on the ground falls asleep (its awake attribute is written 0); with Box3D Data's
    Allow Sleeping off it stays awake. A box with Start Asleep hangs in the air until another box
    lands on it, then both fall."""
    results = []
    awake = {}
    for allow in (1, 0):
        pieces = [dict(GROUND, attrs={"active": 0, "awake": 1}),
                  {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"active": 1, "awake": 1}}]
        path, names = packed_boxes(f"sleep_{allow}_pieces", pieces)
        dop = new_dopnet(f"sleep_{allow}")
        data = box3d_data(dop, packed_object(dop, "pieces", path), "data", enablesleep=allow)
        finish(dop, box3d_solver(dop, data))
        awake[allow] = step(dop, 96, lambda sim, f: point_attributes(sim, "pieces", names[1], ["awake"])["awake"])[96]
    ok = awake[1] == 0 and awake[0] == 1
    results.append(Result("sleep", Result.PASS if ok else Result.FAIL,
                          f"awake after 4 s at rest: {awake[1]} with Allow Sleeping, {awake[0]} without "
                          "(expected 0 and 1)"))

    heights = {}
    for hit in (0, 1):
        path_a, names_a = packed_boxes(f"asleep_{hit}_a", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        dop = new_dopnet(f"asleep_{hit}")
        sleeper = box3d_data(dop, packed_object(dop, "sleeper", path_a), "data", startasleep=1)
        objs = [sleeper]
        if hit:
            path_b, _ = packed_boxes("asleep_hammer", [{"size": (1, 1, 1), "center": (0, 6, 0)}])
            objs.append(packed_object(dop, "hammer", path_b))
        finish(dop, box3d_solver(dop, merge(dop, objs)))
        heights[hit] = step(dop, 48, lambda sim, f: packed_positions(sim, "sleeper")[names_a[0]][1])[48]
    ok = close(heights[0], 3.0, 0.01) and heights[1] < 2.0
    results.append(Result("sleep", Result.PASS if ok else Result.FAIL,
                          f"Start Asleep box in the air after 2 s: y={heights[0]:.2f} alone (expected 3), "
                          f"y={heights[1]:.2f} after another box lands on it (expected below 2)"))
    return results


def test_rolling_resistance():
    """A sphere rolling at 3 m/s (spinning to match) keeps rolling with rolling resistance 0 and
    slows down with 0.3."""
    rows, speeds = [], {}
    for rr in (0.0, 0.3):
        pieces = [dict(GROUND, attrs={"active": 0, "rollingresistance": 0.0, "bullet_georep": "box",
                                      "bullet_autofit": 1, "bounce": 0.0}),
                  {"size": (1, 1, 1), "center": (0, 0.5, 0),
                   "attrs": {"active": 1, "rollingresistance": rr, "bullet_georep": "sphere", "bullet_autofit": 1,
                             "bounce": 0.0, "v": (3.0, 0.0, 0.0), "w": (0.0, 0.0, -6.0)}}]
        path, names = packed_boxes(f"rolling_{int(rr * 10)}_pieces", pieces)
        dop = new_dopnet(f"rolling_{int(rr * 10)}")
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True)))
        v = step(dop, 48, lambda sim, f: point_attributes(sim, "pieces", names[1], ["v"])["v"])[48]
        speeds[rr] = math.hypot(v[0], v[2])
        rows.append(f"rolling resistance {rr:g}: speed {speeds[rr]:.2f} after 2 s")
    ok = speeds[0.0] > 2.8 and speeds[0.3] < 2.0
    return [Result("rolling_resistance", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected about 3, and below 2)")]


SHAPES = (
    # label, size, georep, autofit, primS (half extents), radius, length, primR (Euler degrees), rest height
    ("box, autofit", (1, 1, 1), "box", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), 0.5),
    # AutoFit as Bullet's (autofit_bullet): a sphere through the cube's corners; a capsule around the
    # 1 x 1 cross-section, as tall as the piece.
    ("sphere, autofit", (1, 1, 1), "sphere", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), math.sqrt(3) / 2),
    ("capsule, autofit (standing, 1 x 2)", (1, 2, 1), "capsule", 1, (1.0, 1.0, 1.0), 1.0, 1.0, (0, 0, 0), 1.0),
    ("box, bullet_primS 0.5 0.25 0.5", (1, 1, 1), "box", 0, (0.5, 0.25, 0.5), 1.0, 1.0, (0, 0, 0), 0.25),
    ("sphere, bullet_radius 0.3", (1, 1, 1), "sphere", 0, (1.0, 1.0, 1.0), 0.3, 1.0, (0, 0, 0), 0.3),
    ("capsule, radius 0.25 length 1 (standing)", (1, 1, 1), "capsule", 0, (1.0, 1.0, 1.0), 0.25, 1.0,
     (0, 0, 0), 0.75),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 90 (lying)", (1, 1, 1), "capsule", 0, (1.0, 1.0, 1.0),
     0.25, 1.0, (0, 0, 90), 0.25),
)


def test_shape_types():
    """Box, sphere and capsule bullet_georep, fitted to the piece and sized by bullet_primS /
    bullet_radius / bullet_length with bullet_autofit 0: each rests at its own half height. A
    capsule's axis is the body's Y; bullet_primR turns it to lie down."""
    pieces = [dict(GROUND, attrs={"active": 0, "bullet_georep": "box", "bullet_autofit": 1,
                                  "bullet_primS": (1.0, 1.0, 1.0), "bullet_radius": 1.0, "bullet_length": 1.0,
                                  "bullet_primR": (0.0, 0.0, 0.0), "bounce": 0.0})]
    for i, (_, size, rep, fit, prim_s, radius, length, prim_r, _) in enumerate(SHAPES):
        pieces.append({"size": size, "center": (0, 1.5, (i - 3) * 3),
                       "attrs": {"active": 1, "bullet_georep": rep, "bullet_autofit": fit, "bullet_primS": prim_s,
                                 "bullet_radius": radius, "bullet_length": length,
                                 "bullet_primR": tuple(float(a) for a in prim_r), "bounce": 0.0}})
    path, names = packed_boxes("shapes_pieces", pieces)
    dop = new_dopnet("shapes")
    finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
    final = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))[72]
    rows, ok = [], True
    for i, shape in enumerate(SHAPES):
        y = final[names[i + 1]][1]
        good = close(y, shape[-1], 0.02)
        ok = ok and good
        rows.append(f"{shape[0]}: rests at {y:.3f} (expected {shape[-1]:.3f})")
    return [Result("shape_types", Result.PASS if ok else Result.FAIL, "; ".join(rows))]



def run_with_solver(name, path, frames=24, keep_velocity=False):
    dop = new_dopnet(name)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=keep_velocity))
    finish(dop, solver)
    track = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces"))
    return track, box3d_messages(solver)


def add_wrangle(sop_path, snippet):
    prev = hou.node(sop_path)
    wr = create(prev.parent(), "attribwrangle", "edit")
    wr.setInput(0, prev)
    set_parm(wr, ["snippet"], snippet)
    display(wr)
    return wr.path()


AUTOFIT_SIZE = (0.6, 0.4, 0.8)
AUTOFIT_TILT = (0.0, 0.0, 30.0)  # degrees, about z
AUTOFIT_CENTER = (0.0, 0.6, 0.0)


def autofit_scene(name, solver_kind, georep, rotate_before_pack, size=AUTOFIT_SIZE, tilt=AUTOFIT_TILT):
    """A 0.6 x 0.4 x 0.8 box tilted 30 degrees about z, dropped on a static ground, both packed
    pieces of one RBD Packed Object. rotate_before_pack: the tilt is baked into the packed
    geometry (as Assemble or Pack after a Transform SOP leave it: the packed transform is
    identity), else it is on the packed transform. georep: the piece's bullet_georep (with
    bullet_autofit 1), or None to leave both attributes off (each solver's own default)."""
    geo = new_geo(name + "_geo")

    def branch(tag, size, center, rotate, before, attrs):
        box = create(geo, "box", f"{tag}_box")
        set_parm(box, ["size"], tuple(size))
        last = box
        if before:
            xf = create(geo, "xform", f"{tag}_place")
            xf.setInput(0, last)
            set_parm(xf, ["r"], tuple(rotate))
            set_parm(xf, ["t"], tuple(center))
            last = xf
        asm = create(geo, "assemble", f"{tag}_pack")
        asm.setInput(0, last)
        set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
        last = asm
        if not before:
            # The box is centred on the origin, so rotating there turns the packed piece in place.
            xf = create(geo, "xform", f"{tag}_place")
            xf.setInput(0, last)
            set_parm(xf, ["r"], tuple(rotate))
            set_parm(xf, ["t"], tuple(center))
            last = xf
        sets = [f's@name = "{tag}";'] + [f"{vex_value(v)[0]}@{k} = {vex_value(v)[1]};" for k, v in attrs.items()]
        wr = create(geo, "attribwrangle", f"{tag}_attrs")
        wr.setInput(0, last)
        set_parm(wr, ["snippet"], " ".join(sets))
        return wr

    # Both pieces get the same attributes: Merge would give the ground an empty bullet_georep and
    # bullet_autofit 0 otherwise (with RBD Packed Object's bullet_primS 1 1 1, a 2 x 2 x 2 box).
    piece_attrs = {"active": 1, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0), "bounce": 0.0}
    ground_attrs = {"active": 0, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0), "bounce": 0.0}
    if georep is not None:
        piece_attrs.update({"bullet_georep": georep, "bullet_autofit": 1})
        ground_attrs.update({"bullet_georep": "box", "bullet_autofit": 1})
    ground = branch("ground", GROUND["size"], GROUND["center"], (0, 0, 0), True, ground_attrs)
    # Another size starts higher: half the sum of its sides clears any shape fitted around it.
    lift = AUTOFIT_CENTER[1] if tuple(size) == AUTOFIT_SIZE else 0.5 * sum(size) + 0.1
    center = (AUTOFIT_CENTER[0], lift, AUTOFIT_CENTER[2])
    piece = branch("piece", size, center, tilt, rotate_before_pack, piece_attrs)
    m = create(geo, "merge", "merge")
    m.setInput(0, ground)
    m.setInput(1, piece)
    display(m)

    dop = new_dopnet(name)
    obj = packed_object(dop, "pieces", m.path(), keep_velocity=True)
    finish(dop, solver_for(solver_kind, dop, obj))
    return dop


def autofit_rest(dop, rotate_before_pack, frames=72, tilt=AUTOFIT_TILT):
    """Drops the piece and returns (rest height of its point, tilt in degrees, bullet_georep and
    bullet_autofit as the solver saw them on frame 1). Tilt: the smallest angle between one of
    the box's own axes and vertical (0 = lying on a face)."""
    def sample(sim, f):
        obj = sim.findObject("pieces")
        geo = obj.geometry()
        prim = next(pr for pr in geo.prims() if pr.points() and pr.points()[0].attribValue("name") == "piece")
        m = hou.Matrix4(prim.intrinsicValue("packedfulltransform"))
        seen = point_attributes(sim, "pieces", "piece", ["bullet_georep", "bullet_autofit"]) if f == 1 else None
        return prim.points()[0].position(), m, seen

    track = step(dop, frames, sample)
    pos, m, _ = track[frames]
    seen = track[1][2]
    # The box's axes in the packed geometry's own space: turned by the tilt when it was baked in.
    local = hou.hmath.buildRotate(hou.Vector3(tilt)) if rotate_before_pack else hou.hmath.identityTransform()
    origin = hou.Vector3(0, 0, 0) * local * m
    tilt = 90.0
    for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
        d = (hou.Vector3(axis) * local * m - origin).normalized()
        tilt = min(tilt, math.degrees(math.acos(max(-1.0, min(1.0, abs(d[1]))))))
    return pos[1], tilt, seen


def test_autofit_bullet():
    """A tilted box dropped in Box3D and in Bullet with the same shape settings ends up the same
    way: lying flat (tilt 0) when the shape fits the box, standing on an edge at its starting
    30 degrees when a box shape is fitted to the axis-aligned bounds of the tilted geometry.
    Covers convex hull, box with AutoFit with the tilt baked into the packed geometry and on the
    packed transform, sphere and capsule with AutoFit (rest height only: they roll), and no
    bullet_georep at all, where each solver uses its default (both a convex hull)."""
    cases = (
        # label, bullet_georep (None: unset), tilt baked into the packed geometry
        ("convex hull, tilt baked in", "convexhull", True),
        ("box + autofit, tilt baked in", "box", True),
        ("box + autofit, tilt on the packed transform", "box", False),
        ("sphere + autofit, tilt baked in", "sphere", True),
        ("capsule + autofit, tilt baked in", "capsule", True),
        ("no bullet_georep, tilt baked in", None, True),
    )
    results = []
    for i, case in enumerate(cases):
        label, georep, baked = case[:3]
        size, tilt = case[3:] if len(case) > 3 else (AUTOFIT_SIZE, AUTOFIT_TILT)
        # A sphere or capsule can roll, so only its rest height says how it was fitted (sphere:
        # its radius; capsule lying down: its radius too).
        rolls = georep in ("sphere", "capsule")
        got = {}
        for solver_kind in ("box3d", "bullet"):
            dop = autofit_scene(f"autofit_{i}_{solver_kind}", solver_kind, georep, baked, size, tilt)
            got[solver_kind] = autofit_rest(dop, baked, tilt=tilt)
        (yb, tb, sb), (yt, tt, st) = got["box3d"], got["bullet"]
        msg = (f"{label}: Box3D rests at y {yb:.3f}, tilt {tb:.1f} deg; Bullet y {yt:.3f}, tilt {tt:.1f} deg "
               f"(flat: tilt 0 and y 0.2 or 0.3; fitted to the tilted bounds: tilt 30, y about 0.32)")
        same = (rolls or abs(tb - tt) < 5.0) and abs(yb - yt) < 0.05
        results.append(Result("autofit_bullet", Result.PASS if same else Result.FAIL,
                              msg if same else msg + f". Attributes seen on frame 1: Box3D {sb}, Bullet {st}"))

    # Capsule length: a standing capsule topples one way or the other, so this one is static and a plate
    # dropped on it finds its top.
    top_b3, top_bt = capsule_top("box3d"), capsule_top("bullet")
    same = close(top_b3, top_bt, 0.05)
    results.append(Result("autofit_bullet", Result.PASS if same else Result.FAIL,
                          f"capsule length: the top of a static AutoFit capsule on a standing 0.4 x 1.2 x 0.4 piece is "
                          f"{top_b3:.3f} above its centre in Box3D, {top_bt:.3f} in Bullet (0.883 = enclosing the "
                          f"piece, 0.6 = caps inside it)"))
    return results


def capsule_top(solver_kind, frames=48):
    """Height of a static AutoFit capsule's top above its centre: a 0.6 x 0.1 x 0.6 plate is dropped on
    the capsule of a standing 0.4 x 1.2 x 0.4 piece; the first frame its fall stops is where it touched
    (no bounce). The plate may tip off afterwards."""
    base = {"bullet_autofit": 1, "bounce": 0.0, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0)}
    pieces = [{"size": (0.4, 1.2, 0.4), "center": (0, 1.0, 0),
               "attrs": dict(base, active=0, bullet_georep="capsule")},
              {"size": (0.6, 0.1, 0.6), "center": (0, 2.5, 0),
               "attrs": dict(base, active=1, bullet_georep="box")}]
    path, names = packed_boxes(f"capsule_top_{solver_kind}_pieces", pieces)
    dop = new_dopnet(f"capsule_top_{solver_kind}")
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    finish(dop, solver_for(solver_kind, dop, obj))
    ys = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])
    for f in range(3, frames + 1):
        fall, before = ys[f - 1] - ys[f], ys[f - 2] - ys[f - 1]
        if before > 1e-4 and fall < 0.5 * before:
            return ys[f] - 0.05 - 1.0  # the plate's underside, from the capsule's centre
    return float("nan")


TESTS = {
    "friction_slope": test_friction_slope,
    "ccd": test_ccd,
    "sleep": test_sleep,
    "rolling_resistance": test_rolling_resistance,
    "shape_types": test_shape_types,
    "autofit_bullet": test_autofit_bullet,
}
