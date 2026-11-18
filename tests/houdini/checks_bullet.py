"""Box3D against RBD Bullet Solver on the same scene, for what the plugin takes from Bullet's
conventions without a measurement of its own: the rest convention (`orient` since rest), the
hand-set shape attributes, the anchors of a joint on a piece that has already turned, which end of a
Cone Twist is its goal, the direction of an impact normal and a kinematic piece pushing another.

Each test builds one scene per solver (solver_for) and compares the numbers. A disagreement is a
FAIL with both values in the message, so the first run says which side to change."""

import math

import hou

from b3dtest import (
    box3d_data,
    box3d_solver,
    close,
    create,
    display,
    find_network_geometry,
    finish,
    GROUND,
    ground_object,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    packed_transform,
    point_attributes,
    rbd_object,
    Result,
    set_parm,
    SetupError,
    solver_for,
    sop_solver,
    step,
    whole_object_points,
)
from checks_constraints import hanging_pair
from checks_output import impact_records

KINDS = ("box3d", "bullet")


def rotation_angle(a, b):
    """The angle in degrees of the rotation taking 3x3 row matrix `a` to `b`."""
    trace = sum(sum(a[i][k] * b[i][k] for k in range(3)) for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def quat_angle(a, b):
    """The angle in degrees between two rotations given as (x, y, z, w) quaternions."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


# ---------------------------------------------------------------------------
# The rest convention: a piece that starts turned, spinning in free fall
# ---------------------------------------------------------------------------

REST_CASES = (
    # label, the piece's own rotation in degrees (baked into its geometry), initial spin (radians/s)
    ("turned 30 degrees about z, spinning about y", (0.0, 0.0, 30.0), (0.0, 3.0, 0.0)),
    ("turned (20, 35, 10), spinning about x", (20.0, 35.0, 10.0), (3.0, 0.0, 0.0)),
    ("not turned, spinning about (1, 2, 0.5)", (0.0, 0.0, 0.0), (1.0, 2.0, 0.5)),
)


def free_fall(tag, kind, rotate, spin, frames=10):
    """A 0.6 x 0.4 x 0.8 piece high above the ground, turned and spinning, with nothing to touch for
    `frames` frames: its position, packed rotation and `orient` on the last one."""
    piece = {"size": (0.6, 0.4, 0.8), "center": (0, 20, 0), "rotate": rotate,
             "attrs": {"active": 1, "v": (1.0, 0.0, 0.0), "w": spin}}
    path, names = packed_boxes(f"{tag}_{kind}_pieces", [piece])
    dop = new_dopnet(f"{tag}_{kind}")
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    finish(dop, solver_for(kind, dop, obj))

    def sample(sim, f):
        attrs = point_attributes(sim, "pieces", names[0], ["orient"]) or {}
        return (packed_positions(sim, "pieces")[names[0]], packed_transform(sim, "pieces", names[0]),
                attrs.get("orient"))

    return step(dop, frames, sample)[frames]


def test_bullet_rest_convention():
    """A piece high in the air, already turned (the rotation is in its geometry, so its pivot and
    rest transform are the starting pose) and spinning, falls for 10 frames in Box3D and in Bullet.
    Nothing touches it, so position, the packed rotation and `orient` (the rotation since rest) all
    have to agree: position within 0.02, rotations within 3 degrees."""
    results = []
    for i, (label, rotate, spin) in enumerate(REST_CASES):
        got = {kind: free_fall(f"rest_{i}", kind, rotate, spin) for kind in KINDS}
        (pb, rb, ob), (pt, rt, ot) = got["box3d"], got["bullet"]
        dpos = (pb - pt).length()
        dpacked = rotation_angle(rb, rt)
        dorient = quat_angle(ob, ot) if ob is not None and ot is not None else None
        ok = dpos < 0.02 and dpacked < 3.0 and (dorient is None or dorient < 3.0)
        results.append(Result("bullet_rest_convention", Result.PASS if ok else Result.FAIL,
                              f"{label}: position differs by {dpos:.4f}, packed rotation by {dpacked:.2f} deg, "
                              f"orient by {'n/a' if dorient is None else f'{dorient:.2f} deg'} "
                              f"(orient: Box3D {ob}, Bullet {ot}; positions Box3D {tuple(pb)}, Bullet {tuple(pt)})"))
    return results


# ---------------------------------------------------------------------------
# Hand-set shape attributes: the top of a static shape, found by dropping a plate on it
# ---------------------------------------------------------------------------

PLATE_HALF = 0.05

SHAPE_CASES = (
    # label, bullet_georep, primS (half extents), radius, length, primT (offset), primR (degrees)
    ("box, half extents 0.5 0.25 0.5", "box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0, 0), (0, 0, 0)),
    ("box, half extents 0.5 0.25 0.5 offset up 0.3", "box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0.3, 0), (0, 0, 0)),
    # Angles picked so that taking them as X, then Y, then Z gives a top of 0.26 and any other order
    # 0.57 to 0.67: the order shows. A turned box's top is a corner, which the plate stops on only
    # roughly (both solvers read 0.1 below the corner, Bullet by another 0.1 on 0 70 50), so the cases after
    # it turn the box by 90 degrees: the top is a flat face and the order shows in which face (XYZ: 0.30,
    # 0.30, 0.60; any other order gives 0.60 for the first two).
    ("box, half extents 0.6 0.1 0.3, bullet_primR 50 70 50", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (50, 70, 50)),
    ("box, half extents 0.6 0.1 0.3, bullet_primR 90 90 0", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (90, 90, 0)),
    ("box, half extents 0.6 0.1 0.3, bullet_primR 0 90 90", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (0, 90, 90)),
    ("box, half extents 0.6 0.1 0.3, bullet_primR 90 0 90", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (90, 0, 90)),
    ("sphere, bullet_radius 0.3", "sphere", (1, 1, 1), 0.3, 1.0, (0, 0, 0), (0, 0, 0)),
    ("capsule, radius 0.25 length 1 (bullet_length: straight part or total?)", "capsule", (1, 1, 1), 0.25, 1.0,
     (0, 0, 0), (0, 0, 0)),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 90", "capsule", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 90)),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 45", "capsule", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 45)),
    # Box3D takes a cylinder like a capsule: radius and length (the whole height, flat ends) along Y.
    # Whether Bullet's cylinder reads bullet_radius / bullet_length or bullet_primS is not known: the
    # second case sets both to the same cylinder, so the first one failing and it passing says which.
    ("cylinder, radius 0.25 length 1", "cylinder", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 0)),
    ("cylinder, primS 0.25 0.5 0.25, radius 0.25 length 1", "cylinder", (0.25, 0.5, 0.25), 0.25, 1.0, (0, 0, 0),
     (0, 0, 0)),
    ("cylinder, radius 0.25 length 1, bullet_primR 0 0 90", "cylinder", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 90)),
    ("cylinder, radius 0.25 length 1 offset up 0.3", "cylinder", (1, 1, 1), 0.25, 1.0, (0, 0.3, 0), (0, 0, 0)),
)


def shape_attrs(georep, prim_s, radius, length, prim_t, prim_r):
    return {"bullet_georep": georep, "bullet_autofit": 0, "bullet_autofit_valid": 1,
            "bullet_primS": tuple(float(v) for v in prim_s), "bullet_radius": float(radius),
            "bullet_length": float(length), "bullet_primT": tuple(float(v) for v in prim_t),
            "bullet_primR": tuple(float(v) for v in prim_r), "bounce": 0.0, "v": (0.0, 0.0, 0.0),
            "w": (0.0, 0.0, 0.0)}


# The ground in the tests that drop a hand-set piece: fitted to its own geometry (Auto Fit on), with the
# hand-set attributes (every piece needs them) at nothing: no offset, no turn.
FLAT_GROUND = dict(shape_attrs("box", (0.5, 0.5, 0.5), 0.5, 1.0, (0, 0, 0), (0, 0, 0)), active=0,
                   bullet_georep="box", bullet_autofit=1)


def shape_top(tag, kind, attrs, center=(0.0, 0.0, 0.0), frames=60):
    """Height of the top of a static hand-set shape above its piece's centre: a plate wider than the
    shape is dropped on it, and the first frame its fall stops is where it touched (no bounce; it
    may tip off later). NaN if the plate never stops."""
    plate = dict(attrs, active=1, bullet_georep="box", bullet_autofit=1)
    pieces = [{"size": (0.4, 1.2, 0.4), "center": center, "attrs": dict(attrs, active=0)},
              {"size": (3.0, 2 * PLATE_HALF, 3.0), "center": (center[0], 2.5, center[2]), "attrs": plate}]
    path, names = packed_boxes(f"{tag}_{kind}_pieces", pieces)
    dop = new_dopnet(f"{tag}_{kind}")
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
    ys = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])
    for f in range(3, frames + 1):
        fall, before = ys[f - 1] - ys[f], ys[f - 2] - ys[f - 1]
        if before > 1e-4 and fall < 0.5 * before:
            return ys[f] - PLATE_HALF - center[1]
    return float("nan")


def test_bullet_shape_attributes():
    """The hand-set shape attributes (bullet_autofit 0) against Bullet, by the height of the top of
    a static piece's shape (a plate dropped on it stops there): bullet_primS, bullet_primT, the
    order of bullet_primR's Euler angles (taken as XYZ), bullet_radius and bullet_length (taken as
    the capsule's straight part). The piece is at the origin (see bullet_shape_position for where
    Bullet puts a shape on a piece elsewhere). Top heights within 0.03."""
    results = []
    for i, (label, georep, prim_s, radius, length, prim_t, prim_r) in enumerate(SHAPE_CASES):
        attrs = shape_attrs(georep, prim_s, radius, length, prim_t, prim_r)
        top = {kind: shape_top(f"shape_attr_{i}", kind, attrs) for kind in KINDS}
        ok = close(top["box3d"], top["bullet"], 0.03)
        results.append(Result("bullet_shape_attributes", Result.PASS if ok else Result.FAIL,
                              f"{label}: top {top['box3d']:.3f} above the piece's centre in Box3D, "
                              f"{top['bullet']:.3f} in Bullet"))
    return results


def rotation(axis, degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return [[[1, 0, 0], [0, c, -s], [0, s, c]], [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            [[c, -s, 0], [s, c, 0], [0, 0, 1]]][axis]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


EULER_ORDERS = {"XYZ": (0, 1, 2), "XZY": (0, 2, 1), "YXZ": (1, 0, 2), "YZX": (1, 2, 0), "ZXY": (2, 0, 1),
                "ZYX": (2, 1, 0)}


def thin_axis_by_order(angles):
    """{name: the shape's thin (y) axis in body space} for bullet_primR `angles` taken in each Euler
    order, as column-vector rotations (first axis applied first: "XYZ") and as their transposes
    ("XYZ^T"). A shape lying on its large face has this axis pointing at the world's up."""
    out = {}
    for name, order in EULER_ORDERS.items():
        m = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for axis in order:
            m = matmul(rotation(axis, angles[axis]), m)
        out[name] = (m[0][1], m[1][1], m[2][1])
        out[name + "^T"] = (m[1][0], m[1][1], m[1][2])
    return out


def matching_orders(up, angles):
    """The Euler orders whose thin axis lines up with `up` (either way), best first."""
    found = [(abs(sum(a * b for a, b in zip(up, v))), n) for n, v in thin_axis_by_order(angles).items()]
    return [n for d, n in sorted(found, reverse=True) if d > 0.98]


def primr_rest(kind, prim_r, frames=120):
    """A 0.6 x 0.1 x 0.3 (half extents) hand-set box with bullet_primR `prim_r` dropped on the ground:
    (its height, the body-space direction of the world's up) once it has settled."""
    attrs = shape_attrs("box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0), prim_r)
    flat = FLAT_GROUND
    pieces = [dict(GROUND, attrs=flat), {"size": (1, 1, 1), "center": (0, 1.5, 0), "attrs": dict(attrs, active=1)}]
    tag = f"primr_{int(sum(prim_r))}_{prim_r[0]:g}_{kind}"
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))

    def sample(sim, f):
        rot = packed_transform(sim, "pieces", names[1])
        return packed_positions(sim, "pieces")[names[1]][1], (rot[0][1], rot[1][1], rot[2][1])

    return step(dop, frames, sample)[frames]


PROBE_CASES = (
    # label, half extents, bullet_primT, bullet_primR
    ("box 0.5 0.25 0.5", (0.5, 0.25, 0.5), (0, 0, 0), (0, 0, 0)),
    ("box 0.5 0.25 0.5, primT up 0.3", (0.5, 0.25, 0.5), (0, 0.3, 0), (0, 0, 0)),
    ("box 0.5 0.25 0.5, primT x 0.3", (0.5, 0.25, 0.5), (0.3, 0, 0), (0, 0, 0)),
    ("box 0.5 0.25 0.5, primT down 0.3", (0.5, 0.25, 0.5), (0, -0.3, 0), (0, 0, 0)),
    ("box 0.6 0.1 0.3", (0.6, 0.1, 0.3), (0, 0, 0), (0, 0, 0)),
    ("box 0.6 0.1 0.3, primR 0 0 90", (0.6, 0.1, 0.3), (0, 0, 0), (0, 0, 90)),
    ("box 0.6 0.1 0.3, primR 0 70 50", (0.6, 0.1, 0.3), (0, 0, 0), (0, 70, 50)),
    ("box 0.6 0.1 0.3, primT up 0.3, primR 0 0 90", (0.6, 0.1, 0.3), (0, 0.3, 0), (0, 0, 90)),
)


def probe_rest(kind, half, prim_t, prim_r, tag, frames=150):
    """A dynamic hand-set box dropped from (2, 1.5, 0) onto a ground piece: the piece's point after it
    settles, (x, y, z)."""
    attrs = shape_attrs("box", half, 0.5, 1.0, prim_t, prim_r)
    flat = FLAT_GROUND
    pieces = [dict(GROUND, attrs=flat),
              {"size": (1, 1, 1), "center": (2.0, 1.5, 0.0), "attrs": dict(attrs, active=1)}]
    path, names = packed_boxes(f"{tag}_{kind}_pieces", pieces)
    dop = new_dopnet(f"{tag}_{kind}")
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
    return tuple(step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[1]])[frames])


def test_bullet_shape_probe():
    """Where a dynamic hand-set box ends up relative to its piece's point, for different sizes,
    bullet_primT and bullet_primR, in Box3D and in Bullet: the piece's point after landing on the
    ground. INFO: read the Bullet column for the rule (the shape's lowest point sits at y 0, so
    the point's height says where the shape is relative to it) and compare with Box3D's."""
    results = []
    for i, (label, half, prim_t, prim_r) in enumerate(PROBE_CASES):
        got = {kind: probe_rest(kind, half, prim_t, prim_r, f"probe_{i}") for kind in KINDS}
        fmt = lambda p: "(" + ", ".join(f"{c:.3f}" for c in p) + ")"  # noqa: E731
        diff = tuple(b - a for a, b in zip(got["box3d"], got["bullet"]))
        results.append(Result("bullet_shape_probe", Result.INFO,
                              f"{label}: Box3D {fmt(got['box3d'])}, Bullet {fmt(got['bullet'])}, "
                              f"Bullet minus Box3D {fmt(diff)}"))
    return results


def test_bullet_primr_order():
    """The order of bullet_primR's Euler angles, against Bullet, from how a thin box ends up: it lands
    on its large face, so the body-space direction of the world's up is the shape's thin axis,
    which depends on the order and not on how it fell (the turn about the vertical is free). The
    two solvers' directions agree to within 0.98 (a dot product, either way up). The message lists
    the Euler orders each one fits, as column-vector rotations and as their transposes (^T)."""
    results = []
    for prim_r in ((50, 70, 50), (0, 70, 50), (30, 0, 60), (0, 0, 0)):
        got = {kind: primr_rest(kind, prim_r) for kind in KINDS}
        (yb, ub), (yt, ut) = got["box3d"], got["bullet"]
        agree = abs(sum(a * b for a, b in zip(ub, ut)))
        ok = agree > 0.98
        status = Result.INFO if yt < -1.0 else (Result.PASS if ok else Result.FAIL)  # fell through: nothing to compare
        results.append(Result("bullet_primr_order", status,
                              f"bullet_primR {prim_r}: Box3D rests at y {yb:.3f} with up in the body at "
                              f"{tuple(round(c, 2) for c in ub)}, fits {matching_orders(ub, prim_r) or 'none'}; "
                              f"Bullet y {yt:.3f}, {tuple(round(c, 2) for c in ut)}, fits "
                              f"{matching_orders(ut, prim_r) or 'none'} (flat on the large face: y 0.1)"))
    return results


def test_bullet_shape_position():
    """Where a hand-set shape is: Bullet's static ones were seen at the world origin, whatever the
    piece's position (a plate dropped over a piece at x 3 landed on a shape at the origin). A static
    piece at (3, 1, 0) with a box of half extents 0.5 0.25 0.5: the top of the shape above the piece's
    centre (0.25 if it follows the piece). Then a dynamic piece with the same shape offset up 0.3
    (primT) dropped on the ground: where it rests (-0.05 if the offset is from the point; 0.25, the
    half height, if the shape is centred on the body as Bullet's centre of mass would be). Both
    INFO, not a comparison: Bullet puts a hand-set shape at the world origin and ignores primT's height, and
    Box3D keeps the shape on the piece with the offset, by design (status.md)."""
    results = []
    attrs = shape_attrs("box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0, 0), (0, 0, 0))
    top = {kind: shape_top("shape_pos_static", kind, attrs, center=(3.0, 1.0, 0.0)) for kind in KINDS}
    results.append(Result("bullet_shape_position", Result.INFO,
                          f"static piece at (3, 1, 0): top {top['box3d']:.3f} above its centre in Box3D, "
                          f"{top['bullet']:.3f} in Bullet (NaN: the plate never stopped; 0.25 = the shape is "
                          f"on the piece)"))
    rest = {}
    offset = shape_attrs("box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0.3, 0), (0, 0, 0))
    for kind in KINDS:
        pieces = [dict(GROUND, attrs=FLAT_GROUND),
                  {"size": (1, 1, 1), "center": (3.0, 1.5, 0.0), "attrs": dict(offset, active=1)}]
        path, names = packed_boxes(f"shape_pos_dynamic_{kind}_pieces", pieces)
        dop = new_dopnet(f"shape_pos_dynamic_{kind}")
        finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
        rest[kind] = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])[72]
    results.append(Result("bullet_shape_position", Result.INFO,
                          f"dynamic piece with its box offset up 0.3: rests at y {rest['box3d']:.3f} in Box3D, "
                          f"{rest['bullet']:.3f} in Bullet (-0.05 = the offset is from the piece's centre)"))
    return results


def test_bullet_cylinder():
    """bullet_georep cylinder fitted to its geometry (Auto Fit on) against Bullet. A static piece of
    0.4 x 1.2 x 0.4 with a plate dropped on it: the top of the fitted cylinder (0.6 if its flat end
    sits on the bounds). A dynamic 0.6 x 1.0 x 0.3 piece dropped on the ground: where it rests (0.5
    standing on its flat end), whether the two solvers agree within 0.03. The hand-set cylinder is
    in bullet_shape_attributes."""
    results = []
    fit = dict(shape_attrs("cylinder", (1, 1, 1), 0.5, 1.0, (0, 0, 0), (0, 0, 0)), bullet_autofit=1)
    top = {kind: shape_top("cylinder_top", kind, fit) for kind in KINDS}
    results.append(Result("bullet_cylinder", Result.PASS if close(top["box3d"], top["bullet"], 0.03) else Result.FAIL,
                          f"static fitted cylinder: top {top['box3d']:.3f} above the piece's centre in Box3D, "
                          f"{top['bullet']:.3f} in Bullet"))
    rest = {}
    for kind in KINDS:
        pieces = [dict(GROUND, attrs=FLAT_GROUND),
                  {"size": (0.6, 1.0, 0.3), "center": (0.0, 2.0, 0.0), "attrs": dict(fit, active=1)}]
        path, names = packed_boxes(f"cylinder_rest_{kind}_pieces", pieces)
        dop = new_dopnet(f"cylinder_rest_{kind}")
        finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
        rest[kind] = step(dop, 120, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])[120]
    results.append(Result("bullet_cylinder", Result.PASS if close(rest["box3d"], rest["bullet"], 0.03) else Result.FAIL,
                          f"dynamic fitted cylinder: rests at y {rest['box3d']:.3f} in Box3D, "
                          f"{rest['bullet']:.3f} in Bullet"))
    return results


def anchor_id_run(kind, k, anchor_type):
    """Two fixed boxes, A at (0, 4, 0) and B at (2, 4, 0), no gravity, joined by a SideFX Hard constraint: A's anchor at
    A's centre, B's anchor at the decoy P (3, 4, 1) with `anchor_id` k (and `anchor_type` when given). Neither can
    move (a heavy B was not enough: a joint's position correction moves a body at the same speed whatever its
    mass), so the solver's `distance` feedback (the gap between the two anchors) on frame 2, if it writes one for
    two fixed bodies, is the anchors' starting gap: |B's anchor - (0, 4, 0)|."""
    def edit(path):
        net = hou.node(path)
        wr = create(net.parent(), "attribwrangle", "anchor_ids")
        wr.setInput(0, net)
        set_parm(wr, ["class"], 0)
        code = f'addpointattrib(0, "anchor_id", -1); setpointattrib(0, "anchor_id", 1, {k});'
        if anchor_type:
            code += f' addpointattrib(0, "anchor_type", ""); setpointattrib(0, "anchor_type", 1, "{anchor_type}");'
        set_parm(wr, ["snippet"], code)
        display(wr)
        return wr.path()

    pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (2, 4, 0), "attrs": {"active": 0}}]
    dop, names = hanging_pair(f"anchor_id_{kind}_{k}_{anchor_type or 'point'}", None,
                              line_attrs={"constraint_type": "position"}, conrels=[("hardconrel", {})],
                              solver_kind=kind, pieces=pieces, anchors=((0, 4, 0), (3, 4, 1)),
                              gravity=(0, 0, 0), network_edit=edit)
    if kind == "bullet":
        set_parm(dop.node("gravity"), ["force"], (0, 0, 0))

    def gap(sim, f):
        geo = find_network_geometry(sim)
        prims = list(geo.prims()) if geo is not None else []
        return prims[0].attribValue("distance") if prims and geo.findPrimAttrib("distance") else None

    value = step(dop, 2, gap)[2]
    solver = dop.node("box3d_solver") if kind == "box3d" else None
    return value, (node_messages(solver) if solver is not None else [])


def test_bullet_anchor_id():
    """What a point anchor (`anchor_id`) on a packed piece means in Bullet. Box B's anchor has P (3, 4, 1) and an
    anchor_id; both boxes are fixed, so the `distance` the solver reports on frame 2 (if it reports one) is how far
    B's anchor starts from A's anchor (0, 4, 0). That gap says which point Bullet took: P itself (the id ignored:
    3.16); point k of B's own box; point k of the object's points (one per piece: A is point 0, B point 1; for k 5
    B's); or point k of the unpacked geometry (box A's points 0 to 7, then B's). Reports Bullet's and Box3D's gap
    (Box3D's less the Rest Length 1) and the readings each fits (0.05). Bullet fits `B's own point`: the id picks a point of the
    piece's own geometry; Box3D reads it too. Passes when both fit B's own point. For ids 0 and 5, each as a point and as a vertex."""
    box = create(new_geo("anchor_id_ref"), "box")
    geometry = box.geometry()
    box_points = [pt.position() for pt in geometry.points()]
    box_vertices = [v.point().position() for pr in geometry.prims() for v in pr.vertices()]
    a_anchor, b_centre, a_centre = hou.Vector3(0, 4, 0), hou.Vector3(2, 4, 0), hou.Vector3(0, 4, 0)
    results = []
    for k in (0, 5):
        for anchor_type in (None, "vertex"):
            runs = {kind: anchor_id_run(kind, k, anchor_type) for kind in KINDS}
            got = {kind: run[0] for kind, run in runs.items()}
            pts = box_vertices if anchor_type else box_points
            readings = {
                "P (id ignored)": hou.Vector3(3, 4, 1),
                "B's own point": b_centre + pts[k],
                "object point k": a_centre if k == 0 else b_centre,
                "unpacked point k": (pts[k] + a_centre) if k < 8 else (pts[k - 8] + b_centre),
            }
            fits = {}
            # `distance` is the gap between the anchors in both.
            rest = {"bullet": 0.0, "box3d": 0.0}
            for kind, gap in got.items():
                fits[kind] = [name for name, pos in readings.items()
                              if gap is not None and abs((pos - a_anchor).length() - rest[kind] - gap) < 0.05]                     or ["none"]
            what = f"anchor_id {k}" + (f", anchor_type {anchor_type}" if anchor_type else "")
            fmt = lambda g: "n/a" if g is None else f"{g:.3f}"  # noqa: E731
            wanted = ", ".join(f"{name} {(pos - a_anchor).length():.3f}" for name, pos in readings.items())
            both = fits["bullet"] == fits["box3d"] == ["B's own point"]
            results.append(Result("bullet_anchor_id", Result.PASS if both else Result.FAIL,
                                  f"{what}: gap {fmt(got['bullet'])} in Bullet (fits {fits['bullet']}), "
                                  f"{fmt(got['box3d'])} in Box3D (fits {fits['box3d']}); the readings give {wanted}"
                                  + (f"; Box3D messages {runs['box3d'][1]}" if runs["box3d"][1] else "")))
    return results


# ---------------------------------------------------------------------------
# Joints on a piece that has already turned, and Cone Twist's goal end
# ---------------------------------------------------------------------------

CENTRES = ((0.0, 4.0, 0.0), (0.5, 2.0, 0.0))  # the fixed piece and the hanging one, 2.06 apart
SHARED = ((0.25, 3.0, 0.0), (0.25, 3.0, 0.0))  # one point between them: the centres end up at most 2.06 apart


def joint_run(tag, kind, conrel_type, line_attrs, hang_rotate, anchors, spin, conrel_parms=None):
    """A piece hanging 2 below a fixed one through a SideFX constraint, the hanging piece turned by
    `hang_rotate` before the start: (centre distance, how far it turned since frame 1 in degrees)
    after 2 s."""
    hang = {"active": 1, "w": tuple(spin)}
    fixed = {"active": 0, "w": (0.0, 0.0, 0.0)}
    pieces = [{"size": (1, 1, 1), "center": CENTRES[0], "attrs": fixed},
              {"size": (1, 1, 1), "center": CENTRES[1], "rotate": hang_rotate, "attrs": hang}]
    dop, names = hanging_pair(f"{tag}_{kind}", None, line_attrs=line_attrs, conrels=[(conrel_type, dict(conrel_parms or {}))],
                              solver_kind=kind, pieces=pieces, anchors=anchors, keep_velocity=True)

    def sample(sim, f):
        return packed_positions(sim, "pieces"), packed_transform(sim, "pieces", names[1])

    track = step(dop, 48, sample)
    pos, rot = track[48]
    return (pos[names[1]] - pos[names[0]]).length(), rotation_angle(track[1][1], rot)


def test_bullet_joint_rest():
    """Joints from authored anchors on a piece that starts turned 30 degrees about z: SideFX Hard
    (constraint_type all) with the anchors at the two centroids, held distance and how far the
    piece turned in 2 s in Box3D and in Bullet. This is where the anchors' rest-to-now conversion
    (restToNow) and the `angle` since rest show. Both solvers pull the pieces to the 1.0 rest length
    (the 2.06 start is not held, with `all` or `position`), so the boxes touch and the turn is
    contact noise: distances within 0.1, turns within 15 degrees. With both anchors at one point
    between the pieces the centres can be at most 2.06 apart: INFO."""
    results = []
    d = {kind: joint_run("joint_rest", kind, "hardconrel", {"constraint_type": "all"}, (0.0, 0.0, 30.0),
                         CENTRES, (0.0, 0.0, 0.0)) for kind in KINDS}
    ok = close(d["box3d"][0], d["bullet"][0], 0.1) and abs(d["box3d"][1] - d["bullet"][1]) < 15.0
    results.append(Result("bullet_joint_rest", Result.PASS if ok else Result.FAIL,
                          f"Hard (all), anchors at the centroids, hanging piece turned 30 deg at the start: distance "
                          f"after 2 s Box3D {d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} (start 2.06); turned "
                          f"{d['box3d'][1]:.0f} / {d['bullet'][1]:.0f} deg"))
    d = {kind: joint_run("joint_rest_shared", kind, "hardconrel", {"constraint_type": "position"},
                         (0.0, 0.0, 30.0), SHARED, (0.0, 0.0, 0.0)) for kind in KINDS}
    results.append(Result("bullet_joint_rest", Result.INFO,
                          f"Hard, both anchors at (0.25, 3, 0), hanging piece turned 30 deg: distance after 2 s "
                          f"Box3D {d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} (at most 2.06 if the anchors "
                          f"stay together); turned {d['box3d'][1]:.0f} / {d['bullet'][1]:.0f} deg"))
    return results


def test_bullet_joint_rest_held():
    """The joint-rest case with a start that can be held: SideFX Hard (constraint_type all), anchors at
    the two centroids, Rest Length set to the 2.06 start distance, hanging piece turned 30 degrees about z.
    Nothing pulls the boxes together, so both solvers should keep the distance near the start and turn
    by about the same angle: distances within 0.1 of the start and of each other, turns within 10 degrees."""
    start = math.dist(CENTRES[0], CENTRES[1])
    d = {kind: joint_run("joint_rest_held", kind, "hardconrel", {"constraint_type": "all"}, (0.0, 0.0, 30.0),
                         CENTRES, (0.0, 0.0, 0.0), conrel_parms={"restlength": start}) for kind in KINDS}
    ok = (all(close(d[k][0], start, 0.1) for k in KINDS) and close(d["box3d"][0], d["bullet"][0], 0.1)
          and abs(d["box3d"][1] - d["bullet"][1]) < 10.0)
    return [Result("bullet_joint_rest_held", Result.PASS if ok else Result.FAIL,
                   f"Hard (all), Rest Length {start:.3f}, hanging piece turned 30 deg: distance after 2 s Box3D "
                   f"{d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} (start {start:.3f}); turned "
                   f"{d['box3d'][1]:.0f} / {d['bullet'][1]:.0f} deg")]


def rbd_box_trace(kind, frames=240, offset=(0.0, 0.0, 0.0)):
    """A 1 m polygon box dropped from y 2 as a whole RBD Object onto the ground: its mean point height per
    frame, and the messages of the solver."""
    tag = f"rbd_box_{kind}"
    sop_path = primitive_sop(tag, "box", None, {("size",): (1.0, 1.0, 1.0)}, False)
    set_parm(hou.node(sop_path), ["t"], (offset[0], 2.0 + offset[1], offset[2]))
    dop = new_dopnet(tag)
    obj = rbd_object(dop, "obj", sop_path)
    ground = ground_object(dop, tag + "_ground")
    solver = solver_for(kind, dop, merge(dop, [ground, obj]))
    finish(dop, solver)

    def sample(sim, f):
        pts = whole_object_points(sim, "obj")
        return sum(p[1] for p in pts) / len(pts) if pts else None

    return step(dop, frames, sample), node_messages(solver)


def test_bullet_rbd_box_settle():
    """Whole RBD Object box dropped from y 2, once above the origin and once 5 m away in x and z (the object's
    pivot is not at the origin then). Bullet is at rest (0.500) from about frame 30 on. Prints each solver's
    height at several frames and its movement over frames 109 to 120, and passes when both have stopped there
    (under 0.005) at the same height (within 0.01)."""
    results = []
    for label, offset in (("at the origin", (0.0, 0.0, 0.0)), ("offset 5 in x and z", (5.0, 0.0, 5.0))):
        got = {kind: rbd_box_trace(kind, 120, offset) for kind in KINDS}
        marks = (30, 60, 90, 120)
        rows, final, moving = [], {}, {}
        for kind in KINDS:
            track = got[kind][0]
            tail = [track[f] for f in range(109, 121) if track[f] is not None]
            final[kind] = track[120]
            moving[kind] = (max(tail) - min(tail)) if tail else None
            text = ", ".join(f"{f}: {track[f]:.3f}" for f in marks if track[f] is not None)
            rows.append(f"{kind} [{text}] moved {moving[kind]:.4f} over frames 109 to 120"
                        if moving[kind] is not None else f"{kind}: nothing left")
        ok = (all(final[k] is not None and moving[k] < 0.005 for k in KINDS)
              and abs(final["box3d"] - final["bullet"]) < 0.01)
        results.append(Result("bullet_rbd_box_settle", Result.PASS if ok else Result.FAIL,
                              f"{label}: " + "; ".join(rows)))
    return results


def rbd_state_pivot(sim, obj_name="obj"):
    """The pivot `p` of an object's Position data (RBD State), or None."""
    obj = sim.findObject(obj_name)
    pos = obj.findSubData("Position") if obj is not None else None
    if pos is None:
        return None
    options = pos.record("Options")
    return tuple(options.field("p")) if "p" in options.fieldNames() else None


def pivot_boxes(boxes):
    """A SOP object of the given boxes ((size, centre) each) merged and moved up 3: its xform node's path."""
    geo = new_geo("rbd_pivot_geo")
    merged = create(geo, "merge")
    for i, (size, centre) in enumerate(boxes):
        b = create(geo, "box")
        set_parm(b, ["size"], size)
        set_parm(b, ["t"], centre)
        merged.setInput(i, b)
    xf = create(geo, "xform")
    xf.setInput(0, merged)
    set_parm(xf, ["t"], (0.0, 3.0, 0.0))
    display(xf)
    return xf.path()


def pivot_cone():
    """A 4-sided cone (base radius 1, height 2; the tube's first radius is its top, so the apex points down) moved up 3: its xform node's path."""
    geo = new_geo("rbd_pivot_cone_geo")
    tube = create(geo, "tube")
    set_parm(tube, ["type"], 1)  # Polygon
    set_parm(tube, ["rad"], (1.0, 0.0))
    set_parm(tube, ["height"], 2.0)
    set_parm(tube, ["cols"], 4)
    set_parm(tube, ["cap"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, tube)
    set_parm(xf, ["t"], (0.0, 3.0, 0.0))
    display(xf)
    return xf.path()


def test_bullet_rbd_pivot():
    """What pivot (RBD State `p`) an RBD Object gets with Compute Center of Mass on, in Bullet and in Box3D,
    for shapes where the candidate answers differ (positions are relative to the object, which is 3 m up):
    a single box; a 3 x 1 x 1 box with a 1 x 2 x 1 box on its right end; a 1 m box and a 2 m box 4 m apart; a
    cone. Reports `p` after the first step in both against the bounding box centre, the point mean, the
    volume centroid and the convex hull's centroid. Passes when Box3D's is within 0.06 of Bullet's (Bullet's
    is about 0.03 off the hull centroid for the boxes)."""
    cases = [
        ("one box", pivot_boxes([((1.0, 1.0, 1.0), (0.0, 0.0, 0.0))]),
         {"box centre": (0.0, 0.0, 0.0)}),
        ("L", pivot_boxes([((3.0, 1.0, 1.0), (0.0, 0.0, 0.0)), ((1.0, 2.0, 1.0), (1.0, 1.5, 0.0))]),
         {"bounding box centre": (0.0, 1.0, 0.0), "point mean": (0.5, 0.75, 0.0),
          "volume centroid": (0.4, 0.6, 0.0), "hull centroid": (0.238, 0.762, 0.0)}),
        ("two apart", pivot_boxes([((1.0, 1.0, 1.0), (-2.0, 0.0, 0.0)), ((2.0, 2.0, 2.0), (2.0, 0.5, 0.0))]),
         {"bounding box centre": (0.25, 0.5, 0.0), "point mean": (0.0, 0.25, 0.0),
          "volume centroid": (1.556, 0.389, 0.0), "hull centroid": (0.800, 0.401, 0.0)}),
        ("cone", pivot_cone(),
         {"bounding box centre": (0.0, 0.0, 0.0), "hull centroid": (0.0, 0.5, 0.0)}),
    ]
    fmt = lambda v: "n/a" if v is None else "(" + ", ".join(f"{c:.3f}" for c in v) + ")"  # noqa: E731
    results = []
    for label, path, candidates in cases:
        got = {}
        for kind in KINDS:
            tag = f"rbd_pivot_{label.replace(' ', '_')}_{kind}"
            dop = new_dopnet(tag)
            obj = rbd_object(dop, "obj", path)
            ground = ground_object(dop, tag + "_ground")
            finish(dop, solver_for(kind, dop, merge(dop, [ground, obj])))
            got[kind] = step(dop, 3, lambda sim, f: rbd_state_pivot(sim))[3]
        rel = {k: None if v is None else (v[0], v[1] - 3.0, v[2]) for k, v in got.items()}
        ok = (rel["bullet"] is not None and rel["box3d"] is not None
              and max(abs(a - b) for a, b in zip(rel["bullet"], rel["box3d"])) < 0.06)
        results.append(Result("bullet_rbd_pivot", Result.PASS if ok else Result.FAIL,
                              f"{label}: p relative to the object: Bullet {fmt(rel['bullet'])}, Box3D "
                              f"{fmt(rel['box3d'])}; candidates: "
                              + ", ".join(f"{name} {fmt(c)}" for name, c in candidates.items())))
    return results


def test_bullet_cone_twist_goal():
    """Which end of a SideFX Cone Twist is its goal. Both anchors at one point, the hanging piece
    spinning about y, once not turned (the control) and once turned 45 degrees about z so the two
    ends' frames differ: how far it turns in 2 s in Box3D and in Bullet. The plugin takes point 1
    as the goal. INFO when Bullet's piece fell freely (centres over 10 apart: its joint did not hold,
    so there is nothing to compare), else turns within 15 degrees."""
    results = []
    for label, rotate in (("not turned", (0.0, 0.0, 0.0)), ("turned 45 deg about z", (0.0, 0.0, 45.0))):
        d = {kind: joint_run("cone_goal", kind, "conetwistconrel", {}, rotate, SHARED, (0.0, 3.0, 0.0))
             for kind in KINDS}
        held = d["bullet"][0] < 10.0
        ok = abs(d["box3d"][1] - d["bullet"][1]) < 15.0 and close(d["box3d"][0], d["bullet"][0], 0.15)
        status = Result.INFO if not held else (Result.PASS if ok else Result.FAIL)
        results.append(Result("bullet_cone_twist_goal", status,
                              f"Cone Twist at (0.25, 3, 0), hanging piece {label} and spinning 3 rad/s about y: "
                              f"Box3D distance {d['box3d'][0]:.3f}, turned {d['box3d'][1]:.0f} deg; Bullet distance "
                              f"{d['bullet'][0]:.3f}, turned {d['bullet'][1]:.0f} deg"
                              + ("" if held else " (Bullet's piece fell freely)")))
    return results


# ---------------------------------------------------------------------------
# Impact normals
# ---------------------------------------------------------------------------


def impact_switches(*nodes):
    """Names of the toggles with 'impact' in their name on `nodes`, to turn Impacts data on."""
    found = []
    for node in nodes:
        for parm in node.parms():
            name = parm.name().lower()
            if "impact" in name and "parmop" not in name and \
                    parm.parmTemplate().type() == hou.parmTemplateType.Toggle:
                found.append((node, parm.name()))
    return found


def first_impact(kind):
    """The first Impacts record of a 1 kg box dropped on the ground (Add Impact Data on), or None
    with the reason: (frame, record, note)."""
    tag = f"impact_normal_{kind}"
    path, _ = packed_boxes(tag + "_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
    dop = new_dopnet(tag)
    obj = packed_object(dop, "piece", path, computemass=0, mass=1.0, bounce=0.0)
    ground = ground_object(dop, tag + "_ground", bounce=0.0)
    note = ""
    if kind == "box3d":
        last = box3d_solver(dop, merge(dop, [ground, box3d_data(dop, obj, "data", addimpact=1)]))
    else:
        last = solver_for(kind, dop, merge(dop, [ground, obj]))
        solver = next(n for n in dop.children() if n.type().name() == "bulletrbdsolver")
        switches = impact_switches(obj, solver)
        for node, name in switches:
            set_parm(node, [name], 1)
        note = f"switches turned on: {[name for _, name in switches] or 'none found'}"
    finish(dop, last)
    track = step(dop, 30, lambda sim, f: impact_records(sim, "piece"))
    frame = next((f for f in sorted(track) if track[f]), None)
    return (frame, track[frame][0], note) if frame is not None else (None, None, note)


def vector_field(record, word):
    """The first field whose name contains `word`, if it is a 3-vector: (name, value)."""
    for name, value in record.items():
        if word not in name.lower():
            continue
        try:
            vec = tuple(float(c) for c in value)  # a tuple, or a hou.Vector3
        except (TypeError, ValueError):
            continue
        if len(vec) == 3:
            return name, vec
    return None, None


def test_bullet_impact_normal():
    """The direction of the impact normal in Impacts data, for a box landing on the ground, in
    Box3D and in Bullet: the same way up or down (dot product above 0.9). The RBD Bullet Solver has no
    Impacts parameter, so this turns on any toggle with 'impact' in its name on the RBD Packed Object
    and the solver. INFO when Bullet writes no impacts that way or either has no normal."""
    recs, notes = {}, {}
    for kind in KINDS:
        _, rec, notes[kind] = first_impact(kind)
        recs[kind] = rec
    if recs["bullet"] is None or recs["box3d"] is None:
        return [Result("bullet_impact_normal", Result.INFO,
                       f"no impacts in 30 frames: Box3D {'yes' if recs['box3d'] else 'no'}, Bullet "
                       f"{'yes' if recs['bullet'] else 'no'} ({notes['bullet']}). Bullet has to be given Impacts "
                       "data some other way")]
    normals = {kind: vector_field(recs[kind], "normal") for kind in KINDS}
    if any(v[0] is None for v in normals.values()):
        return [Result("bullet_impact_normal", Result.INFO,
                       f"no normal field: Box3D fields {sorted(recs['box3d'])}, Bullet {sorted(recs['bullet'])}")]
    a, b = normals["box3d"][1], normals["bullet"][1]
    la, lb = math.sqrt(sum(c * c for c in a)), math.sqrt(sum(c * c for c in b))
    dot = sum(x * y for x, y in zip(a, b)) / (la * lb) if la > 1e-9 and lb > 1e-9 else float("nan")
    return [Result("bullet_impact_normal", Result.PASS if dot > 0.9 else Result.FAIL,
                   f"Box3D {normals['box3d'][0]} {tuple(round(c, 3) for c in a)}, Bullet {normals['bullet'][0]} "
                   f"{tuple(round(c, 3) for c in b)}, dot {dot:.2f}")]


# ---------------------------------------------------------------------------
# A kinematic piece pushing another
# ---------------------------------------------------------------------------


def kinematic_push(kind, mover_first=True):
    """A piece with animated = 1 moved along x at 1 unit/s by a SOP Solver, pushing a box that sits
    1.6 away: (the kinematic piece's x, the box's x) after 2 s. mover_first: the SOP Solver comes
    before the physics solver in the Multiple Solver (else after it)."""
    pieces = [GROUND,
              {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"active": 0, "animated": 1}},
              {"size": (1, 1, 1), "center": (1.6, 0.5, 0), "attrs": {"active": 1, "animated": 0}}]
    tag = f"kinematic_push_{kind}{'' if mover_first else '_after'}"
    path, names = packed_boxes(tag, pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path)
    mover = sop_solver(dop, f'if (s@name == "{names[1]}") @P.x = @Time;')
    if kind == "box3d":
        physics = box3d_solver(dop, None)
        last = multisolver(dop, obj, [mover, physics] if mover_first else [physics, mover])
    else:
        physics = create(dop, "bulletrbdsolver", "bullet_solver")
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, obj, [mover, physics] if mover_first else [physics, mover]))
    finish(dop, last)
    final = step(dop, 49, lambda sim, f: packed_positions(sim, "pieces"))[49]
    return final[names[1]][0], final[names[2]][0]


def test_bullet_kinematic_push():
    """An animated = 1 piece (active = 0) moved by a SOP Solver pushes a free box, in Box3D and in
    Bullet: both follow the animation (x about 2 after 2 s, within 0.05) and push the box within
    0.3 of each other. Bullet is tried with the SOP Solver before and after it in the Multiple
    Solver, and the one that follows the animation is used."""
    t = 48 / hou.fps()
    box3d = kinematic_push("box3d")
    tried = {}
    for first in (True, False):
        tried[first] = kinematic_push("bullet", mover_first=first)
        if close(tried[first][0], t, 0.05):
            break
    bullet = tried[first]
    order = "before" if first else "after"
    ok = close(box3d[0], t, 0.05) and close(bullet[0], t, 0.05) and close(box3d[1], bullet[1], 0.3)
    tried_text = ", ".join(f"{'before' if k else 'after'}: kinematic x {v[0]:.2f}" for k, v in tried.items())
    return [Result("bullet_kinematic_push", Result.PASS if ok else Result.FAIL,
                   f"after {t:.2f} s: kinematic piece x Box3D {box3d[0]:.2f}, Bullet {bullet[0]:.2f} (animation "
                   f"{t:.2f}, SOP Solver {order} the Bullet solver); pushed box x Box3D {box3d[1]:.2f}, Bullet "
                   f"{bullet[1]:.2f}. Bullet tried: {tried_text}")]


# ---------------------------------------------------------------------------
# Primitives that are not polygons
# ---------------------------------------------------------------------------

# SOPs with a Primitive Type menu: every item of it is tried (read from the menu, not listed here). SOP type,
# parameters, where the centre of a correctly sized shape rests (the ground is at y 0)
MENU_SOPS = (
    ("box", {("size",): (1.0, 1.0, 1.0)}, 0.5),
    ("sphere", {("rad",): (0.5, 0.5, 0.5)}, 0.5),
    ("tube", {("rad",): (0.5, 0.5), ("height",): 1.0}, 0.5),
    ("circle", {("rad", "radius"): (0.5, 0.5)}, 0.0),
)

# Primitives made by SOPs without such a menu: label, SOP type, parameters, rest height (None: unknown)
OTHER_SOPS = (
    ("metaball", "metaball", {("rad", "radius"): (0.5, 0.5, 0.5)}, 0.5),
    ("volume", "volume", {}, None),
)


def primitive_cases():
    """(label, SOP type, Primitive Type item or None, parameters, rest height) for every item of
    each menu in MENU_SOPS, then OTHER_SOPS. A SOP that cannot be made gives one case that says so."""
    cases = []
    probe = new_geo("prim_menu_probe")
    for sop_type, parms, ideal in MENU_SOPS:
        try:
            parm = create(probe, sop_type).parm("type")
            items = list(parm.menuLabels()) if parm is not None else [None]
        except (SetupError, hou.Error):
            items = [None]
        cases += [(f"{sop_type}, {item}" if item else sop_type, sop_type, item, parms, ideal) for item in items]
    cases += [(label, sop_type, None, parms, ideal) for label, sop_type, parms, ideal in OTHER_SOPS]
    return cases


def set_type_exactly(node, label):
    """Sets the SOP's Primitive Type menu to the item named `label` (not just containing it: a
    sphere's 'Mesh' is not 'Polygon Mesh')."""
    parm = node.parm("type")
    if parm is None:
        raise SetupError(f"{node.path()} has no 'type' menu")
    labels = list(parm.menuLabels())
    for i, item in enumerate(labels):
        if item.lower() == label.lower():
            try:
                parm.set(i)
            except (TypeError, hou.OperationFailed):
                parm.set(parm.menuItems()[i])
            return
    raise SetupError(f"{node.path()}: no Primitive Type '{label}'; items: {labels}")


def primitive_sop(tag, sop_type, menu_item, parms, pack):
    """A SOP object holding one primitive at y 2 (packed with Assemble when `pack`)."""
    geo = new_geo(tag)
    src = create(geo, sop_type)
    if menu_item is not None:
        set_type_exactly(src, menu_item)
    for names, value in parms.items():
        set_parm(src, list(names), value)
    if pack:
        asm = create(geo, "assemble")
        asm.setInput(0, src)
        set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
        src = asm
    xf = create(geo, "xform")
    xf.setInput(0, src)
    set_parm(xf, ["t"], (0.0, 2.0, 0.0))
    display(xf)
    return xf.path()


def primitive_rest(tag, kind, sop_path, packed, frames=120):
    """Where a primitive dropped on the ground ends up: the height of its centre (a packed piece's
    point, or the mean of a whole object's points) on the last frame, whether it was still moving
    (more than 0.01 over the last 12 frames), and the solver's messages."""
    dop = new_dopnet(tag)
    obj = packed_object(dop, "obj", sop_path) if packed else rbd_object(dop, "obj", sop_path)
    ground = ground_object(dop, tag + "_ground")
    solver = solver_for(kind, dop, merge(dop, [ground, obj]))
    finish(dop, solver)
    messages = []

    def sample(sim, f):
        messages.extend(node_messages(solver))
        if packed:
            pieces = packed_positions(sim, "obj")
            return next(iter(pieces.values()))[1] if pieces else None
        pts = whole_object_points(sim, "obj")
        return sum(p[1] for p in pts) / len(pts) if pts else None

    track = step(dop, frames, sample)
    tail = [track[f] for f in range(frames - 11, frames + 1) if track[f] is not None]
    return track[frames], bool(tail) and max(tail) - min(tail) > 0.01, messages


def test_bullet_primitive_types():
    """How Bullet and Box3D treat Houdini's non-polygon primitives: every Primitive Type of the Box,
    Sphere, Tube and Circle SOPs (the menus are read, so none is missed), a metaball and a volume,
    each dropped on the ground from y 2 as a packed piece (RBD Packed Object) and as a whole RBD
    Object. Reports the height each solver leaves the centre at (a correctly sized shape: 0.5 for
    the size 1 box and the radius 0.5 sphere, 0.5 for the height 1 tube, 0 for the flat circle),
    whether it fell through the ground or vanished, and the first message each node gave. INFO: it
    shows what Bullet does, not a pass mark."""
    results = []
    for i, (label, sop_type, menu_item, parms, ideal) in enumerate(primitive_cases()):
        for packed in (True, False):
            route = "packed" if packed else "RBD Object"
            tag = f"prim_{i}_{'packed' if packed else 'rbd'}"
            try:
                sop_path = primitive_sop(tag, sop_type, menu_item, parms, packed)
                got = {kind: primitive_rest(f"{tag}_{kind}", kind, sop_path, packed) for kind in KINDS}
            except (SetupError, hou.Error) as e:
                results.append(Result("bullet_primitive_types", Result.INFO, f"{label}, {route}: could not run: {e}"))
                continue

            def show(kind):
                y, moving, messages = got[kind]
                if y is None:
                    return "nothing left"
                text = "fell through" if y < -1.0 else f"y {y:.3f}"
                text += " (still moving)" if moving else ""
                return text + (f" ({messages[0][:70]})" if messages else "")

            yb, yt = got["box3d"][0], got["bullet"][0]
            same = yb is not None and yt is not None and abs(yb - yt) < 0.05
            hint = "" if ideal is None else f", a correctly sized shape rests at {ideal:g}"
            results.append(Result("bullet_primitive_types", Result.INFO,
                                  f"{label}, {route}: Box3D {show('box3d')}, Bullet {show('bullet')}"
                                  f"{hint}; {'the same' if same else 'differ'}"))
    return results


# ---------------------------------------------------------------------------
# More comparisons for what status.md lists as not compared
# ---------------------------------------------------------------------------


def rescan_fall(kind, frames=10):
    """free_fall's first case with a SOP Solver writing a point attribute every step before the physics
    solver, so the points change between steps and the solver cannot skip its scan: (position, packed
    rotation, orient) after `frames` frames."""
    piece = {"size": (0.6, 0.4, 0.8), "center": (0, 20, 0), "rotate": REST_CASES[1][1],
             "attrs": {"active": 1, "v": (1.0, 0.0, 0.0), "w": REST_CASES[1][2]}}
    path, names = packed_boxes(f"rescan_{kind}_pieces", [piece])
    dop = new_dopnet(f"rescan_{kind}")
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    toucher = sop_solver(dop, "f@scan_probe = @Frame;")
    if kind == "box3d":
        last = multisolver(dop, obj, [toucher, box3d_solver(dop, None)])
    else:
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, obj, [toucher, create(dop, "bulletrbdsolver", "bullet_solver")]))
    finish(dop, last)

    def sample(sim, f):
        attrs = point_attributes(sim, "pieces", names[0], ["orient"]) or {}
        return (packed_positions(sim, "pieces")[names[0]], packed_transform(sim, "pieces", names[0]),
                attrs.get("orient"))

    return step(dop, frames, sample)[frames]


def test_bullet_rest_rescan():
    """The rest rotation of a turned, spinning piece (the case of bullet_rest_convention) when the points
    are edited every step, so every step does the full scan that splits the rest rotation per point: Box3D
    gives the same position, rotation and `orient` as without the edits (within 0.001 and 1 degree), and
    the same as Bullet (0.02 and 3 degrees)."""
    got = {kind: rescan_fall(kind) for kind in KINDS}
    plain = free_fall("rescan_plain", "box3d", REST_CASES[1][1], REST_CASES[1][2])
    (pb, rb, ob), (pt, rt, ot), (pp, rp, op) = got["box3d"], got["bullet"], plain
    own = (pb - pp).length(), rotation_angle(rb, rp), quat_angle(ob, op) if ob is not None and op is not None else 0.0
    other = ((pb - pt).length(), rotation_angle(rb, rt),
             quat_angle(ob, ot) if ob is not None and ot is not None else 0.0)
    ok = own[0] < 0.001 and own[1] < 1.0 and own[2] < 1.0 and other[0] < 0.02 and other[1] < 3.0 and other[2] < 3.0
    return [Result("bullet_rest_rescan", Result.PASS if ok else Result.FAIL,
                   f"against the same run without edits: position {own[0]:.4f}, rotation {own[1]:.2f} deg, orient "
                   f"{own[2]:.2f} deg; against Bullet: {other[0]:.4f}, {other[1]:.2f} deg, {other[2]:.2f} deg")]


JOINT_ANGLE_CASES = (
    # label, the hanging piece's own rotation in degrees, its initial spin (radians/s)
    ("turned 30 degrees about z, spinning about y", (0.0, 0.0, 30.0), (0.0, 2.0, 0.0)),
    ("turned 40 degrees about x, spinning about z", (40.0, 0.0, 0.0), (0.0, 0.0, 2.0)),
    ("turned (20, 35, 10), not spinning", (20.0, 35.0, 10.0), (0.0, 0.0, 0.0)),
)


def test_bullet_joint_angle():
    """The `angle` since rest of a joint made on a piece that starts turned: SideFX Hard (constraint_type
    all) with the anchors at the two centroids and Rest Length set to the start distance (so nothing
    pulls the boxes together), the hanging piece turned and spinning at the start. Distance after 2 s
    within 0.1 of the start and of Bullet's, and the turn since frame 1 within 10 degrees of Bullet's."""
    start = math.dist(CENTRES[0], CENTRES[1])
    results = []
    for i, (label, rotate, spin) in enumerate(JOINT_ANGLE_CASES):
        d = {kind: joint_run(f"joint_angle_{i}", kind, "hardconrel", {"constraint_type": "all"}, rotate, CENTRES,
                             spin, conrel_parms={"restlength": start}) for kind in KINDS}
        ok = (all(close(d[k][0], start, 0.1) for k in KINDS) and close(d["box3d"][0], d["bullet"][0], 0.1)
              and abs(d["box3d"][1] - d["bullet"][1]) < 10.0)
        results.append(Result("bullet_joint_angle", Result.PASS if ok else Result.FAIL,
                              f"Hard (all), Rest Length {start:.3f}, hanging piece {label}: distance after 2 s Box3D "
                              f"{d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f}; turned {d['box3d'][1]:.0f} / "
                              f"{d['bullet'][1]:.0f} deg"))
    return results


APART = ((0.0, 3.5, 0.0), (0.5, 2.5, 0.0))  # the fixed piece's bottom face and the hanging piece's top face


def test_bullet_cone_twist_goal_apart():
    """Which end of a SideFX Cone Twist is its goal, with the two anchors apart (the bottom of the fixed
    piece and the top of the hanging one) so the joint has something to hold, which one point shared by
    both ends did not give (bullet_cone_twist_goal). The hanging piece is not turned and then turned 45
    degrees about z, spinning about y: distance and turn after 2 s in both solvers. INFO when Bullet's piece
    fell freely, else distances within 0.15 and turns within 15 degrees."""
    results = []
    for label, rotate in (("not turned", (0.0, 0.0, 0.0)), ("turned 45 deg about z", (0.0, 0.0, 45.0))):
        d = {kind: joint_run("cone_goal_apart", kind, "conetwistconrel", {}, rotate, APART, (0.0, 3.0, 0.0))
             for kind in KINDS}
        held = d["bullet"][0] < 10.0
        ok = abs(d["box3d"][1] - d["bullet"][1]) < 15.0 and close(d["box3d"][0], d["bullet"][0], 0.15)
        status = Result.INFO if not held else (Result.PASS if ok else Result.FAIL)
        results.append(Result("bullet_cone_twist_goal_apart", status,
                              f"Cone Twist between (0, 3.5, 0) and (0.5, 2.5, 0), hanging piece {label} and spinning "
                              f"3 rad/s about y: Box3D distance {d['box3d'][0]:.3f}, turned {d['box3d'][1]:.0f} deg; "
                              f"Bullet distance {d['bullet'][0]:.3f}, turned {d['bullet'][1]:.0f} deg"
                              + ("" if held else " (Bullet's piece fell freely)")))
    return results


PRIMR_THROUGH_CASES = ((30, 0, 60), (30, 0, 0), (0, 0, 60), (30, 0, 30), (0, 0, 30), (0, 0, 45), (0, 30, 0),
                       (0, 60, 0))


def test_bullet_primr_through():
    """Which bullet_primR makes Bullet's hand-set box fall through the ground (30 0 60 did in
    bullet_primr_order): the same 0.6 x 0.1 x 0.3 box dropped on the ground with the angles on their own,
    in pairs and in the failing mix. Reports the height each solver rests at and whether it fell through
    (below -1). INFO: it narrows down what Bullet does, it is not a pass mark."""
    rows = []
    for prim_r in PRIMR_THROUGH_CASES:
        got = {kind: primr_rest(kind, prim_r)[0] for kind in KINDS}
        show = lambda y: "fell through" if y < -1.0 else f"y {y:.3f}"  # noqa: E731
        rows.append(f"primR {prim_r}: Box3D {show(got['box3d'])}, Bullet {show(got['bullet'])}")
    return [Result("bullet_primr_through", Result.INFO, "; ".join(rows))]


PRIMT_PRIMR_CASES = (
    # label, half extents, bullet_primT, bullet_primR
    ("box 0.6 0.1 0.3, primT up 0.3, primR 0 0 90", (0.6, 0.1, 0.3), (0, 0.3, 0), (0, 0, 90)),
    ("box 0.6 0.1 0.3, primT x 0.4, primR 0 0 90", (0.6, 0.1, 0.3), (0.4, 0, 0), (0, 0, 90)),
    ("box 0.6 0.1 0.3, primT (0.2, 0.3, -0.2), primR 90 90 0", (0.6, 0.1, 0.3), (0.2, 0.3, -0.2), (90, 90, 0)),
    ("box 0.5 0.25 0.5, primT down 0.2, primR 0 90 0", (0.5, 0.25, 0.5), (0, -0.2, 0), (0, 90, 0)),
)


def test_bullet_primt_primr():
    """bullet_primT together with bullet_primR on a static hand-set box at the origin: the height of its
    top (a plate dropped on it stops there), within 0.03 of Bullet's. The angles are multiples of 90
    degrees so the top is a flat face; the offset is applied to the turned shape in both."""
    results = []
    for i, (label, half, prim_t, prim_r) in enumerate(PRIMT_PRIMR_CASES):
        attrs = shape_attrs("box", half, 0.5, 1.0, prim_t, prim_r)
        top = {kind: shape_top(f"primt_primr_{i}", kind, attrs) for kind in KINDS}
        ok = close(top["box3d"], top["bullet"], 0.03)
        results.append(Result("bullet_primt_primr", Result.PASS if ok else Result.FAIL,
                              f"{label}: top {top['box3d']:.3f} above the piece's centre in Box3D, "
                              f"{top['bullet']:.3f} in Bullet"))
    return results


def impact_track(kind, frames=90):
    """first_impact's scene (a 1 kg box dropped on the ground, Impacts on) over `frames` frames: {frame: (the
    simulation time, the records' time fields, the number of records)}."""
    tag = f"impact_fresh_{kind}"
    path, _ = packed_boxes(tag + "_piece", [{"size": (1, 1, 1), "center": (0, 1.5, 0)}])
    dop = new_dopnet(tag)
    obj = packed_object(dop, "piece", path, computemass=0, mass=1.0, bounce=0.0)
    ground = ground_object(dop, tag + "_ground", bounce=0.0)
    if kind == "box3d":
        last = box3d_solver(dop, merge(dop, [ground, box3d_data(dop, obj, "data", addimpact=1)]))
    else:
        last = solver_for(kind, dop, merge(dop, [ground, obj]))
        solver = next(n for n in dop.children() if n.type().name() == "bulletrbdsolver")
        for node, name in impact_switches(obj, solver):
            set_parm(node, [name], 1)
    finish(dop, last)

    def sample(sim, f):
        records = impact_records(sim, "piece")
        times = [v for r in records for n, v in r.items() if "time" in n.lower() and isinstance(v, (int, float))]
        return sim.time(), times, len(records)

    return step(dop, frames, sample)


def test_bullet_impact_fresh():
    """Impacts data is fresh each step, which is how SIM_Impacts is meant to be used: a box dropped on the
    ground and left to rest for 3 s. Box3D's records on a frame must not be older than two frames (their
    time field, when they have one), and it must record on the landing frame. Bullet's record count per
    frame and the frames it records on are INFO beside it (Bullet needs its Impacts toggle found by name,
    see bullet_impact_normal)."""
    track = {kind: impact_track(kind) for kind in KINDS}
    frames = {kind: [f for f in sorted(t) if t[f][2]] for kind, t in track.items()}
    stale = [f for f, (now, times, _) in track["box3d"].items() if any(now - x > 2.0 / hou.fps() + 1e-6 for x in times)]
    has_time = any(t[1] for t in track["box3d"].values())
    ok = bool(frames["box3d"]) and not stale
    lines = []
    for kind in KINDS:
        most = max((t[2] for t in track[kind].values()), default=0)
        lines.append(f"{kind}: records on {len(frames[kind])} of {len(track[kind])} frames, first {frames[kind][:1]}, "
                     f"at most {most} per frame")
    return [Result("bullet_impact_fresh", Result.PASS if ok else Result.FAIL,
                   "; ".join(lines) + (f"; Box3D records older than 2 frames on frames {stale}" if stale else "")
                   + ("" if has_time else "; Box3D records have no time field, so their age was not checked"))]


def test_bullet_rbd_pivot_far():
    """The RBD Object pivot (`p`) for shapes away from the origin, where the 0.03 gap to Bullet shows or
    not: the L of bullet_rbd_pivot moved 5 along x, and a T (a 3 x 1 x 1 bar with a 1 x 2 x 1 box centred
    under it), both with Compute Center of Mass on. Passes when Box3D's `p` is within 0.06 of Bullet's
    in every component, as it is taken straight from the state (not relative to the object)."""
    cases = [
        ("L moved 5 in x", pivot_boxes([((3.0, 1.0, 1.0), (5.0, 0.0, 0.0)), ((1.0, 2.0, 1.0), (6.0, 1.5, 0.0))])),
        ("T", pivot_boxes([((3.0, 1.0, 1.0), (0.0, 0.0, 0.0)), ((1.0, 2.0, 1.0), (0.0, -1.5, 0.0))])),
    ]
    fmt = lambda v: "n/a" if v is None else "(" + ", ".join(f"{c:.3f}" for c in v) + ")"  # noqa: E731
    results = []
    for label, path in cases:
        got = {}
        for kind in KINDS:
            tag = f"rbd_pivot_far_{label.split()[0]}_{kind}"
            dop = new_dopnet(tag)
            obj = rbd_object(dop, "obj", path)
            ground = ground_object(dop, tag + "_ground")
            finish(dop, solver_for(kind, dop, merge(dop, [ground, obj])))
            got[kind] = step(dop, 3, lambda sim, f: rbd_state_pivot(sim))[3]
        ok = (got["bullet"] is not None and got["box3d"] is not None
              and max(abs(a - b) for a, b in zip(got["bullet"], got["box3d"])) < 0.06)
        results.append(Result("bullet_rbd_pivot_far", Result.PASS if ok else Result.FAIL,
                              f"{label}: p Bullet {fmt(got['bullet'])}, Box3D {fmt(got['box3d'])}"))
    return results


TESTS = {
    "bullet_rest_rescan": test_bullet_rest_rescan,
    "bullet_joint_angle": test_bullet_joint_angle,
    "bullet_cone_twist_goal_apart": test_bullet_cone_twist_goal_apart,
    "bullet_primr_through": test_bullet_primr_through,
    "bullet_primt_primr": test_bullet_primt_primr,
    "bullet_impact_fresh": test_bullet_impact_fresh,
    "bullet_rbd_pivot_far": test_bullet_rbd_pivot_far,
    "bullet_rest_convention": test_bullet_rest_convention,
    "bullet_shape_attributes": test_bullet_shape_attributes,
    "bullet_shape_position": test_bullet_shape_position,
    "bullet_cylinder": test_bullet_cylinder,
    "bullet_anchor_id": test_bullet_anchor_id,
    "bullet_primr_order": test_bullet_primr_order,
    "bullet_shape_probe": test_bullet_shape_probe,
    "bullet_joint_rest": test_bullet_joint_rest,
    "bullet_joint_rest_held": test_bullet_joint_rest_held,
    "bullet_rbd_box_settle": test_bullet_rbd_box_settle,
    "bullet_rbd_pivot": test_bullet_rbd_pivot,
    "bullet_cone_twist_goal": test_bullet_cone_twist_goal,
    "bullet_impact_normal": test_bullet_impact_normal,
    "bullet_kinematic_push": test_bullet_kinematic_push,
    "bullet_primitive_types": test_bullet_primitive_types,
}
