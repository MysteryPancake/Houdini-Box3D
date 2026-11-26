"""Box3D against RBD Bullet Solver on the same scene, for what the plugin takes from Bullet's
conventions without a measurement of its own: the rest convention (`orient` since rest), the
hand-set shape attributes, the anchors of a joint on a piece that has already turned, which end of a
Cone Twist is its goal, the direction of an impact normal and a kinematic piece pushing another.

Each test builds one scene per solver (solver_for) and compares the numbers. A disagreement is a
FAIL with both values in the message, so the first run says which side to change."""

import math

import hou

from b3dtest import (
    box_sop,
    box3d_data,
    box3d_solver,
    centroid,
    close,
    constraint_lines,
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
    set_menu_token,
    set_parm,
    SetupError,
    solver_for,
    sop_solver,
    step,
    whole_object_points,
)
from checks_constraints import hanging_pair
from checks_sop_common import solver_sop
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
    dop, _ = hanging_pair(f"anchor_id_{kind}_{k}_{anchor_type or 'point'}", None,
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
# An inactive RBD Object driven by its Position
# ---------------------------------------------------------------------------


def inactive_push(kind, bounce=None, speed=2.0, frames=25):
    """An RBD Object moved along x at `speed` by a Position DOP (Set Always), with a free RBD Object box 0.7 ahead
    of its face and no gravity: per frame, the pusher's and the box's centre x. kind "bullet": Create Active
    Object off in the RBD Bullet Solver; "box3d": the same in the Box3D Solver (a static collider); "kinematic":
    a Kinematic Box3D Data on it in the Box3D Solver. bounce: both objects' Bounce (None: the default)."""
    tag = f"inactive_push_{kind}" + ("" if bounce is None else f"_b{bounce}")
    dop = new_dopnet(tag)
    pusher = rbd_object(dop, "pusher", box_sop(f"{tag}_pusher", (0, 0, 0)))
    box = rbd_object(dop, "box", box_sop(f"{tag}_box", (1.2, 0, 0)))
    if bounce is not None:
        for node in (pusher, box):
            set_parm(node, ["bounce"], bounce)
    moved = pusher
    if kind == "kinematic":
        moved = box3d_data(dop, pusher, "pusher_data", bodytype=1, **({} if bounce is None else {"bounce": bounce}))
    else:
        set_parm(pusher, ["active"], 0)
    mover = create(dop, "position", "mover")
    mover.setInput(0, moved)
    set_menu_token(mover, ["parmop_t"], "always")
    mover.parm("tx").setExpression(f"($FF-1)/$FPS*{speed}")
    both = merge(dop, [mover, box])
    if kind == "bullet":
        last = create(dop, "bulletrbdsolver", "bullet_solver")  # no Gravity DOP after it
        last.setInput(0, both)
    else:
        last = box3d_solver(dop, both, gravity=(0, 0, 0))
    finish(dop, last)
    return step(dop, frames, lambda sim, f: (centroid(whole_object_points(sim, "pusher"))[0],
                                              centroid(whole_object_points(sim, "box"))[0]))


def _inactive_summary(kind, bounce, speed=2.0):
    runs = {k: inactive_push(k, bounce, speed) for k in (kind, "bullet")}
    out = {}
    for k, track in runs.items():
        (px, bx), (_, bx_prev) = track[25], track[24]
        out[k] = (px, bx, (bx - bx_prev) * hou.fps())
    text = ", ".join(f"{k}: pusher x {v[0]:.2f}, box x {v[1]:.2f}, box speed {v[2]:.2f}" for k, v in out.items())
    text += ". Box x by frame 5, 10, 15, 20, 25: " + "; ".join(
        f"{k} {[round(runs[k][f][1], 2) for f in (5, 10, 15, 20, 25)]}" for k in runs)
    return out[kind], out["bullet"], text


def test_bullet_kinematic_object_push():
    """A whole RBD Object with a Kinematic Box3D Data and an animated Position (a Position DOP at Set Always,
    2 m/s along x, no gravity) pushes a free box as Bullet's inactive RBD Object does. With Bounce 0 on both,
    after 1 s each pusher follows the animation (x 2, within 0.05), the box is within 0.2 of the same place and
    its speed within 0.3 m/s. With the default Bounce (0.5) the result is INFO."""
    a, b, text = _inactive_summary("kinematic", 0.0)
    ok = (close(a[0], 2.0, 0.05) and close(b[0], 2.0, 0.05) and close(a[1], b[1], 0.2)
          and close(a[2], b[2], 0.3))
    return [Result("bullet_kinematic_object_push", Result.PASS if ok else Result.FAIL,
                   f"Bounce 0, after 1 s ({text}); pusher expected at x 2.0"),
            Result("bullet_kinematic_object_push", Result.INFO,
                   f"default Bounce, after 1 s ({_inactive_summary('kinematic', None)[2]})")]


def test_bullet_inactive_push():
    """An RBD Object with Create Active Object off and an animated Position (a Position DOP at Set Always, no
    gravity, Bounce 0) carries a free box in front of it in Box3D as in Bullet, at 2 m/s and at 10 m/s: after
    1 s each pusher is on its animation (within 0.05), the box within 0.3 of where Bullet's is and its speed
    within 0.6 m/s. In Box3D it is a static collider driven as a kinematic body once it moves."""
    results = []
    for speed in (2.0, 10.0):
        a, b, text = _inactive_summary("box3d", 0.0, speed)
        ok = (close(a[0], speed, 0.05) and close(b[0], speed, 0.05) and close(a[1], b[1], 0.3)
              and close(a[2], b[2], 0.6))
        results.append(Result("bullet_inactive_push", Result.PASS if ok else Result.FAIL,
                              f"{speed:g} m/s, after 1 s ({text})"))
    return results


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


def mirror_flip_run(kind, row):
    """A piece high in the air with no velocity, its packed transform's first or third row flipped by a SOP Solver on
    frame 6, before the physics solver: its packed 3x3 on frame 10."""
    piece = {"size": (0.6, 0.4, 0.8), "center": (0, 20, 0), "attrs": {"active": 1}}
    path, names = packed_boxes(f"mirror_flip_{kind}_{row}_pieces", [piece])
    dop = new_dopnet(f"mirror_flip_{kind}_{row}")
    obj = packed_object(dop, "pieces", path)
    flip = sop_solver(dop, "")
    set_parm(flip.node("edit"), ["class"], "primitive")
    signs = "-1, 1, 1" if row == "first" else "1, 1, -1"
    set_parm(flip.node("edit"), ["snippet"], f"""
if (@Frame == 6) {{
    matrix3 s = set({signs.split(", ")[0]}, 0, 0, 0, {signs.split(", ")[1]}, 0, 0, 0, {signs.split(", ")[2]});
    setprimintrinsic(0, "transform", @primnum, s * matrix3(primintrinsic(0, "transform", @primnum)));
}}
""")
    if kind == "box3d":
        last = multisolver(dop, obj, [flip, box3d_solver(dop, None)])
    else:
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, obj, [flip, create(dop, "bulletrbdsolver", "bullet_solver")]))
    finish(dop, last)
    return step(dop, 10, lambda sim, f: packed_transform(sim, "pieces", names[0]))[10]


def test_bullet_mirror_flip():
    """A packed piece whose transform is flipped mid-simulation, first row (an x mirror) and third row, in Box3D and
    in Bullet: the 3x3 each leaves on frame 10 (INFO). Box3D takes the rotation from `orient`, so an edit of the packed
    transform alone keeps it and only the mirror's sign is kept, as the third row's flip."""
    results = []
    for row in ("first", "third"):
        got = {kind: mirror_flip_run(kind, row) for kind in KINDS}
        fmt = {kind: "[" + " ".join(f"{v:+.2f}" for r in m for v in r) + "]" if m else None for kind, m in got.items()}
        results.append(Result("bullet_mirror_flip", Result.INFO,
                               f"{row} row flipped: Box3D {fmt['box3d']}, Bullet {fmt['bullet']}"))
    return results


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


FEEDBACK_ATTRS = ("force", "torque", "distance", "angle")
G = 9.80665


def feedback_run(tag, kind, conrel_type, line_attrs, frames, hang=None, pieces=None, anchors=None, packed_parms=None,
                 conrel_parms=None):
    """A hanging pair (hanging_pair) played for `frames`: {frame: {force, torque, distance, angle}} read from the
    constraint line's feedback attributes (None where the solver wrote none)."""
    dop, _names = hanging_pair(f"{tag}_{kind}", None, line_attrs=line_attrs, conrels=[(conrel_type, dict(conrel_parms or {}))],
                               solver_kind=kind, pieces=pieces, anchors=anchors, keep_velocity=True,
                               packed_parms=packed_parms)

    def sample(sim, f):
        geo = find_network_geometry(sim)
        if geo is None or not geo.prims():
            return None
        prim = geo.prims()[0]
        return {a: (prim.attribValue(a) if geo.findPrimAttrib(a) is not None else None) for a in FEEDBACK_ATTRS}

    return step(dop, frames, sample)


def test_bullet_joint_feedback():
    """The feedback attributes of a SideFX Hard line against the RBD Bullet Solver, with values known from physics.
    (1) `angle` since rest, PASS or FAIL: the hanging piece spins freely at 2 rad/s about z on a position-only Hard line
    between the two centroids (a rod, so the swing does not turn it), so it has turned 2 * t radians in both.
    (2) `torque` of a 1 kg cantilever (fixed piece at x 0, hanging piece's centre at x 1, Rest Length 0 so nothing pulls
    it), as Bullet's: the line's force (the weight, 9.81 N) taken about the fixed piece's centre, no couple, so 9.81 times
    the anchor's x (0.5 at the shared face, 0 at the fixed piece's centre, 1 at the hanging piece's). Box3D's native Weld
    and Bullet's Hard must read it. INFO: the Hard (all) readings of both."""
    results = []
    start = math.dist(CENTRES[0], CENTRES[1])
    pieces = [{"size": (1, 1, 1), "center": CENTRES[0], "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": CENTRES[1], "attrs": {"active": 1, "w": (0.0, 0.0, 2.0)}}]
    frames = (7, 13, 25)  # 0.25, 0.5 and 1 s after frame 1
    got = {kind: feedback_run("fb_angle", kind, "hardconrel", {"constraint_type": "position"}, max(frames),
                              pieces=pieces, anchors=CENTRES, conrel_parms={"restlength": start}) for kind in KINDS}
    rows, ok = [], True
    for f in frames:
        want = 2.0 * (f - 1) / 24.0
        vals = {k: (got[k][f] or {}).get("angle") for k in KINDS}
        rows.append(f"frame {f}: expected {want:.3f} rad, Box3D {vals['box3d']:.3f}, Bullet {vals['bullet']:.3f}"
                    if None not in vals.values() else f"frame {f}: {vals}")
        ok = ok and all(v is not None and close(abs(v), want, 0.05 * want) for v in vals.values())
    results.append(Result("bullet_joint_feedback", Result.PASS if ok else Result.FAIL,
                          "angle of a freely spinning piece (2 rad/s about z): " + "; ".join(rows)))

    mass = {"computemass": 0, "mass": 1.0}
    rows, hard_rows, ok = [], [], True
    for label, point in (("the shared face", (0.5, 2.0, 0.0)), ("the fixed piece's centre", (0.0, 2.0, 0.0)),
                         ("the hanging piece's centre", (1.0, 2.0, 0.0))):
        want = G * point[0]  # the weight's moment about the fixed piece's centre (no couple)
        weld = feedback_run("fb_cantilever_weld", "box3d", "box3d_weldconrel", None, 36, packed_parms=mass,
                            anchors=(point, point))[36]
        hard = {kind: feedback_run("fb_cantilever", kind, "hardconrel", {"constraint_type": "all"}, 36,
                                   packed_parms=mass, anchors=(point, point), conrel_parms={"restlength": 0.0})[36]
                for kind in KINDS}
        torque = {"Box3D Weld": weld and weld["torque"], "Bullet Hard": hard["bullet"] and hard["bullet"]["torque"]}
        ok = ok and all(v is not None and abs(v - want) < 0.05 * G for v in torque.values())
        rows.append(f"anchor at {label}: expected {want:.2f}, Box3D Weld {torque['Box3D Weld']:.2f} "
                    f"(force {weld['force']:.2f}), Bullet Hard {torque['Bullet Hard']:.2f}")
        hard_rows.append(f"{label}: Box3D {hard['box3d']['force']:.2f} / {hard['box3d']['torque']:.2f}, "
                         f"Bullet {hard['bullet']['force']:.2f} / {hard['bullet']['torque']:.2f}")
    results.append(Result("bullet_joint_feedback", Result.PASS if ok else Result.FAIL,
                          "torque of a 1 kg cantilever (Rest Length 0) on frame 36, the weight's moment about the fixed "
                          "piece's centre, as Bullet's: " + "; ".join(rows)))
    results.append(Result("bullet_joint_feedback", Result.INFO,
                          "Hard (all) force / torque of the same cantilever (the weight is 9.81 N): Box3D's Hard reads "
                          "less than the weight, which Bullet's does not, because the two pieces touch "
                          "(bullet_hard_collisions): " + "; ".join(hard_rows)))
    return results


def test_bullet_hard_collisions():
    """Why Box3D's Hard (all) `force` reads less than the weight where Bullet's does not. A 1 kg cantilever (fixed piece
    at x 0, hanging piece's centre at x 1, Rest Length 0, weight 9.81 N) on a Hard line, its anchors at the shared face
    and at the fixed piece's centre, with Disable Collisions off (the default) and on. Box3D reads the joint's own force:
    with collisions off the pieces touch and friction at their shared face carries part of the weight, so it reads
    less (between 3 and 9 N), and with them on the pieces do not touch and it reads the weight (within 0.3 N). Bullet
    reads the weight either way (its `force` is the pieces' net force, not the joint's load)."""
    mass = {"computemass": 0, "mass": 1.0}
    rows, ok = [], True
    for label, point in (("the shared face", (0.5, 2.0, 0.0)), ("the fixed piece's centre", (0.0, 2.0, 0.0))):
        got = {}
        for kind in KINDS:
            for disabled in (0, 1):
                run = feedback_run("hard_collisions", kind, "hardconrel", {"constraint_type": "all"}, 36,
                                   packed_parms=mass, anchors=(point, point),
                                   conrel_parms={"restlength": 0.0, "disablecollisions": disabled})[36]
                got[kind, disabled] = run["force"] if run else None
        good = (None not in got.values() and 3.0 < got["box3d", 0] < 9.0 and close(got["box3d", 1], G, 0.3)
                and close(got["bullet", 0], G, 0.3) and close(got["bullet", 1], G, 0.3))
        ok = ok and good
        rows.append(f"anchor at {label}: Box3D {got['box3d', 0]:.2f} N with collisions between the pieces, "
                    f"{got['box3d', 1]:.2f} with them disabled; Bullet {got['bullet', 0]:.2f} and {got['bullet', 1]:.2f}")
    return [Result("bullet_hard_collisions", Result.PASS if ok else Result.FAIL,
                   "Hard (all) force of a 1 kg cantilever on frame 36 (the weight is 9.81 N; expected Box3D between 3 and "
                   "9 with collisions and the weight without, Bullet the weight): " + "; ".join(rows))]


def soft_spring_run(kind, hz, mass):
    """A 1 m box hanging from a fixed one on a position-only Bullet Soft line (Rest Length 1.5, damping ratio 0.05, the
    anchors at the two centres, 2 apart so the spring starts stretched) for 4 s at 24 per second: (the time between the
    first two peaks of its height, the mean length of the spring between the first peak and trough)."""
    pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}]
    dop, names = hanging_pair(f"soft_spring_{kind}_{int(hz)}_{int(mass)}", None,
                              line_attrs={"constraint_type": "position"},
                              conrels=[("bulletsoftconrel", {"stiffness": hz, "dampingratio": 0.05, "restlength": 1.5})],
                              solver_kind=kind, pieces=pieces, anchors=((0, 4, 0), (0, 2, 0)),
                              packed_parms={"computemass": 0, "mass": mass})
    track = step(dop, 96, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])
    ys = [track[f] for f in range(1, 97)]
    peaks = [i for i in range(1, len(ys) - 1) if ys[i] >= ys[i - 1] and ys[i] > ys[i + 1]]
    troughs = [i for i in range(1, len(ys) - 1) if ys[i] <= ys[i - 1] and ys[i] < ys[i + 1]]
    if len(peaks) < 2 or not troughs:
        return None, None
    return (peaks[1] - peaks[0]) / 24.0, 4.0 - 0.5 * (ys[peaks[0]] + ys[troughs[0]])


def test_soft_spring_hz():
    """Bullet Soft's Stiffness is a frequency in Hz, as the Box3D mapping takes it. A box hanging on a position-only Soft
    line, Stiffness 1 and 2, 1 kg and 10 kg: in both solvers the height oscillates with a period of 1 / Stiffness
    seconds (within 0.05 s, whatever the mass) and the spring's mean length is its Rest Length plus g / (2 pi f)^2
    (within 0.08 m)."""
    rows, ok = [], True
    for hz in (1.0, 2.0):
        for mass in (1.0, 10.0):
            want_period = 1.0 / hz
            want_length = 1.5 + G / (2.0 * math.pi * hz) ** 2
            seen = {kind: soft_spring_run(kind, hz, mass) for kind in KINDS}
            for kind, (period, length) in seen.items():
                good = (period is not None and abs(period - want_period) < 0.05
                        and abs(length - want_length) < 0.08)
                ok = ok and good
            rows.append(f"Stiffness {hz:g}, {mass:g} kg: " + ", ".join(
                f"{kind} {seen[kind][0]:.2f} s / {seen[kind][1]:.2f} m" if seen[kind][0] is not None
                else f"{kind} no oscillation" for kind in KINDS) + f" (expected {want_period:.2f} s / {want_length:.2f} m)")
    return [Result("bullet_soft_spring_hz", Result.PASS if ok else Result.FAIL, "period / mean length: " + "; ".join(rows))]


def soft_start_run(kind, rest):
    """A box hanging under gravity from a position-only Bullet Soft line (Stiffness 10, damping ratio 1) whose two anchors
    start at one point (the fixed box's bottom face): how far below the fixed anchor the box's anchor is on frames 1, 2,
    3, 4, 6, 8, 12 and 24."""
    pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}]
    anchor = (0, 3.5, 0)
    dop, _names = hanging_pair(f"soft_start_{kind}_{int(rest)}", None, line_attrs={"constraint_type": "position"},
                               conrels=[("bulletsoftconrel", {"stiffness": 10.0, "dampingratio": 1.0,
                                                              "restlength": rest})],
                               solver_kind=kind, pieces=pieces, anchors=(anchor, anchor),
                               packed_parms={"computemass": 0, "mass": 1.0}, keep_velocity=True)
    first = {}

    def below(sim, f):
        x = sim.findObject("pieces").geometry().prims()[1].fullTransform()
        if f == 1:
            first["inverse"] = x.inverted()
        return anchor[1] - ((hou.Vector3(anchor) * first["inverse"]) * x)[1]

    track = step(dop, 24, below)
    return [track[f] for f in (1, 2, 3, 4, 6, 8, 12, 24)]


def test_soft_coincident_anchors():
    """A Soft line whose anchors start at one point, with a Rest Length of 1, against Bullet. A hanging box falls until
    its anchor is the Rest Length below the fixed one and stays there (1.00 within 0.02 on frames 12 and 24 in both).
    The spring has no direction while the anchors are exactly together, so Bullet applies none on the first step and
    the box gets there one frame later, in Box3D too: its joint is aimed along the anchors' line again each frame
    (0.00 on frame 2 in both). With Rest Length 0 neither moves the anchors apart."""
    frames = (1, 2, 3, 4, 6, 8, 12, 24)
    got = {(kind, rest): soft_start_run(kind, rest) for kind in KINDS for rest in (1.0, 0.0)}
    ok = all(abs(got[kind, 1.0][i] - 1.0) < 0.02 for kind in KINDS for i in (6, 7)) and all(
        abs(v) < 0.01 for kind in KINDS for v in got[kind, 0.0])
    fmt = lambda values: ", ".join(f"{v:.2f}" for v in values)
    return [Result("bullet_soft_coincident_anchors", Result.PASS if ok else Result.FAIL,
                   f"Rest Length 1, the box's anchor below the fixed one on frames {list(frames)}: Box3D "
                   f"{fmt(got['box3d', 1.0])}; Bullet {fmt(got['bullet', 1.0])} (expected 1.00 by frames 12 and 24); "
                   f"Rest Length 0: Box3D {fmt(got['box3d', 0.0])}; Bullet {fmt(got['bullet', 0.0])} (expected 0.00)"),
            Result("bullet_soft_coincident_anchors", Result.PASS if all(abs(got[k, 1.0][1]) < 0.02 for k in KINDS)
                   else Result.FAIL,
                   f"on frame 2 the box is {got['box3d', 1.0][1]:.2f} m below in Box3D and {got['bullet', 1.0][1]:.2f} in "
                   "Bullet (its spring has no direction on the first step)")]


def position_rest_run(kind, conrel_type, conrel_parms, rest=2.0, reverse=False, together=False):
    """A 1 m box beside a fixed one (their centres 2 apart along x, the anchors at the centres) on a position-only
    SideFX line with a Rest Length of 2, so nothing pulls them together. Gravity is across the line, so a radial
    spring or distance joint lets the box swing down, and a pin or spring at the offset point (Bullet's anchor
    shifted by the Rest Length along a direction fixed in the fixed box) holds it there. The box's centre after 2 s."""
    fixed = {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}}
    free = {"size": (1, 1, 1), "center": (2, 4, 0), "attrs": {"active": 1}}
    # reverse: the line starts at the free box (its anchor is the shifted one). together: both anchors at one point.
    pieces, anchors = ([free, fixed], ((2, 4, 0), (0, 4, 0))) if reverse else ([fixed, free], ((0, 4, 0), (2, 4, 0)))
    if together:
        anchors = ((1, 4, 0), (1, 4, 0))
    parms = dict(conrel_parms, restlength=rest)
    tag = f"position_rest_{conrel_type}_{kind}_{int(rest * 10)}_{int(reverse)}{int(together)}"
    dop, names = hanging_pair(tag, None, line_attrs={"constraint_type": "position"},
                              conrels=[(conrel_type, parms)], solver_kind=kind, pieces=pieces, anchors=anchors,
                              packed_parms={"computemass": 0, "mass": 1.0})
    free_name = names[0] if reverse else names[1]

    def sample(sim, f):
        centre = packed_positions(sim, "pieces")[free_name]
        if not together:
            return centre
        # The box's anchor (1 from its centre), where the line holds it: it swings about it.
        return centre + hou.Vector3(-1, 0, 0) * hou.Matrix3(packed_transform(sim, "pieces", free_name))

    return step(dop, 48, sample)[48]


def test_bullet_position_rest_length():
    """Position-only Hard, Soft and Spring lines with a Rest Length, against Bullet. Bullet shifts the anchor of
    the first piece by the Rest Length along the direction between the anchors at rest, fixed in that piece's
    frame (SIM_BulletConstraint's simSetRestLength), and pins or springs the other anchor to the shifted point in all
    three axes. A box hanging beside a fixed one, anchors at the centres and the Rest Length their distance, stays
    where it is; a distance joint or radial spring lets it swing under the fixed anchor. Passes when Box3D's
    box is within 0.3 of Bullet's after 2 s."""
    cases = (("hard", "hardconrel", {}),
             ("soft", "bulletsoftconrel", {"stiffness": 4.0, "dampingratio": 1.0}),
             ("spring", "springconrel", {"strength": 400.0, "damping": 40.0}))
    # (label, Rest Length, the line starts at the free box, both anchors at one point): the rest length that holds
    # the box where it is, and one that pulls it in, with the line either way round (the box ends at rest, so the
    # positions are compared), and the anchors together (no direction at rest: the box hangs 1 below the anchor and
    # swings about it, so its anchor's position is compared); then the line reversed with the rest length that holds it,
    # which swings, as INFO.
    settled = (("", 2.0, False, False), (", pulled in (1)", 1.0, False, False),
               (", pulled in, line reversed", 1.0, True, False), (", anchors together (1)", 1.0, False, True))
    swinging = ((", line reversed", 2.0, True, False),)

    def run(conrel_type, parms, rest, reverse, together):
        return {kind: position_rest_run(kind, conrel_type, parms, rest, reverse, together) for kind in KINDS}

    def show(got):
        return (f"box at {tuple(round(v, 2) for v in got['box3d'])} in Box3D, "
                f"{tuple(round(v, 2) for v in got['bullet'])} in Bullet")

    rows, info, ok = [], [], True
    for label, conrel_type, parms in cases:
        for extra, rest, reverse, together in settled:
            got = run(conrel_type, parms, rest, reverse, together)
            ok = ok and (got["box3d"] - got["bullet"]).length() < 0.3
            rows.append(f"{label}{extra}: {show(got)}")
        for extra, rest, reverse, together in swinging:
            info.append(f"{label}{extra}: {show(run(conrel_type, parms, rest, reverse, together))}")
    return [Result("bullet_position_rest_length", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (started at (2, 4, 0); expected within 0.3 of each other)"),
            Result("bullet_position_rest_length", Result.INFO,
                   "; ".join(info) + " (a swinging box, not compared)")]


def spring_sag_run(kind, strength, damping, substeps):
    """A 1 kg box hanging from a fixed one on a position-only SideFX Spring (rest length 0, both anchors at the point
    between them): how far the box's centre sits below where it started (mean over the last second of 4)."""
    pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}]
    anchor = (0, 3, 0)
    dop, names = hanging_pair(f"spring_sag_{kind}_{int(strength)}_{substeps}", None,
                              line_attrs={"constraint_type": "position"},
                              conrels=[("springconrel", {"strength": strength, "damping": damping, "restlength": 0.0})],
                              solver_kind=kind, pieces=pieces, anchors=(anchor, anchor),
                              packed_parms={"computemass": 0, "mass": 1.0})
    solver = next(n for n in dop.children() if n.type().name() in ("bulletrbdsolver", "box3d_solver", "box3dsolver"))
    set_parm(solver, ["substeps"], substeps)
    track = step(dop, 96, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])
    return 2.0 - sum(track[f] for f in range(73, 97)) / 24.0


def test_bullet_spring_cap():
    """Bullet's Spring (btGeneric6DofSpring2Constraint) caps its stiffness at m / (16 dt^2) and its damping at m / dt,
    dt being the frame time over Bullet's Substeps, and Box3D's mapping does not (a Box3D spring is implicit, stable at
    any frequency, and a cap taken from our Substeps would be far softer than Bullet's at its default of 10). A 1 kg box
    hangs on a Spring with Strength k and a critical Damping: the sag is m g / k below the cap, and m g / cap above it
    (Substeps 1: cap 36, so a Strength of 1000 sags 0.27 m, not 0.0098; Substeps 4: cap 576, 0.017 m). Measured: Bullet
    sags 0.26 and 0.017 there, Box3D 0.0098. INFO: the sag in each solver against Hooke's."""
    rows = []
    for substeps in (1, 4):
        for k in (20.0, 1000.0):
            c = 2.0 * math.sqrt(k)
            got = {kind: spring_sag_run(kind, k, c, substeps) for kind in KINDS}
            rows.append(f"Substeps {substeps}, Strength {k:g}, Damping {c:.1f}: sag Box3D {got['box3d']:.4f}, Bullet "
                        f"{got['bullet']:.4f} (Hooke's {G / k:.4f})")
    return [Result("bullet_spring_cap", Result.INFO, "; ".join(rows))]


PER_STEP_MOTION = ("airresist", "spinresist", "targetv", "targetw")  # zeroed after each solve, as `force`


def motion_run(kind, attrs, frames=72, spin=None, once=False):
    """One 1 kg box falling freely (nothing under it) with the drag / limit point attributes `attrs` (and an initial
    spin about z if `spin`), for `frames` frames at 24 per second: (its vertical speed, its spin about z) over the
    last frame, both as magnitudes. airresist, spinresist, targetv and targetw are inputs for one step in Bullet (it
    zeroes them after each solve, as `force`), so a SOP Solver writes them before the solver every frame, as a POP
    Drag would; the others are on the point."""
    # once: the attributes are on the point from the start and nothing writes them again.
    stays = {k: v for k, v in attrs.items() if once or k not in PER_STEP_MOTION}
    each = {k: v for k, v in attrs.items() if not once and k in PER_STEP_MOTION}
    piece = {"size": (1, 1, 1), "center": (0, 1000, 0), "attrs": dict(stays, active=1)}
    if spin is not None:
        piece["attrs"]["w"] = (0.0, 0.0, spin)
    path, names = packed_boxes(f"motion_{kind}_pieces", [piece])
    dop = new_dopnet(f"motion_{kind}")
    obj = packed_object(dop, "pieces", path, keep_velocity=spin is not None, computemass=0, mass=1.0)
    if each:
        vex = "".join(f"v@{k} = set({', '.join(str(c) for c in v)});" if isinstance(v, tuple) else f"f@{k} = {v};"
                      for k, v in each.items())
        writer = sop_solver(dop, vex)
        if kind == "box3d":
            last = multisolver(dop, obj, [writer, box3d_solver(dop, None)])
        else:
            last = create(dop, "gravity", "gravity")
            last.setInput(0, multisolver(dop, obj, [writer, create(dop, "bulletrbdsolver", "bullet_solver")]))
    else:
        last = solver_for(kind, dop, obj)
    finish(dop, last)

    def sample(sim, f):
        return packed_positions(sim, "pieces")[names[0]][1], packed_transform(sim, "pieces", names[0])

    track = step(dop, frames, sample)
    dy = abs(track[frames][0] - track[frames - 1][0]) * 24.0
    turned = rotation_angle(track[frames - 1][1], track[frames][1]) * 24.0 * math.pi / 180.0
    return dy, turned


def test_bullet_drag_limits():
    """The drag and speed limit point attributes (airresist, spinresist, drag, dragexp, targetv, speedmax, spinmax), as
    RBD Bullet Solver reads them, in both solvers. A 1 kg box falling freely: quadratic drag (airresist 2) ends at the
    speed where the drag is the weight, sqrt(g / 2); linear drag (dragexp 1, airresist 4) at g / 4; `drag` scales the
    resist (airresist 2, drag 2: sqrt(g / 4)); a target velocity moves the end speed to it plus that; speedmax caps
    the fall; a spinning box slowed by spinresist and capped by spinmax. airresist, spinresist, targetv and targetw are
    one-step inputs (Bullet zeroes them after each solve, as `force`), written every frame. Both solvers apply the drag
    as one implicit step of their own Substeps (Bullet 10, Box3D 2 here), so their end speeds differ by what that step
    size gives (up to 6 percent here, both below the formula). speedmax is applied after Box3D's step, so the speed the
    frame averages is a little over it (the gravity gained in a step, 0.2 m/s here). Passes when both solvers agree
    within 5 percent (0.2 m/s for the cap, 0.1 rad/s for the spin) and Box3D is within 8 percent of the formula."""
    g = G
    cases = (
        ("quadratic drag", {"airresist": 2.0}, None, math.sqrt(g / 2.0), "fall"),
        ("linear drag", {"airresist": 4.0, "dragexp": 1.0}, None, g / 4.0, "fall"),
        ("drag scales the resist", {"airresist": 2.0, "drag": 2.0}, None, math.sqrt(g / 4.0), "fall"),
        ("target velocity", {"airresist": 2.0, "targetv": (0.0, -3.0, 0.0)}, None, 3.0 + math.sqrt(g / 2.0), "fall"),
        ("speedmax", {"speedmax": 3.0}, None, 3.0, "fall"),
        ("spin drag", {"spinresist": 3.0}, 6.0, None, "spin"),
        ("spinmax", {"spinmax": 2.0}, 6.0, 2.0, "spin"),
    )
    rows, ok = [], True
    for label, attrs, spin, want, kind_of in cases:
        got = {kind: motion_run(kind, attrs, spin=spin) for kind in KINDS}
        if kind_of == "spin":
            a, b = got["box3d"][1], got["bullet"][1]
            good = abs(a - b) < 0.1 and (want is None or abs(a - want) < 0.1)
            rows.append(f"{label}: spin Box3D {a:.2f}, Bullet {b:.2f} rad/s" + (f" (expected {want:g})" if want else ""))
        else:
            a, b = got["box3d"][0], got["bullet"][0]
            good = abs(a - b) < 0.05 * max(b, 1e-6) + (0.2 if label == "speedmax" else 0.02)
            if kind_of == "fall" and want is not None:
                good = good and abs(a - want) < 0.08 * want
            rows.append(f"{label}: speed Box3D {a:.3f}, Bullet {b:.3f} m/s" + (f" (formula {want:.3f})" if want else ""))
        ok = ok and good
    # airresist on the point from the start, never written again: zeroed after the first solve in both, so the box
    # falls freely (about g * 3 s = 29 m/s) but for the first frame.
    once = {kind: motion_run(kind, {"airresist": 2.0}, once=True)[0] for kind in KINDS}
    good = all(v > 20.0 for v in once.values())
    ok = ok and good
    rows.append(f"airresist set once: speed Box3D {once['box3d']:.1f}, Bullet {once['bullet']:.1f} m/s (free fall: it is "
                "cleared after the first solve)")
    return [Result("bullet_drag_limits", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def accel_run(tag, kind, attrs, spin=False, frames=9):
    """A 1 m box with `attrs` (accelmax / angaccelmax) on ground of friction 1 and no bounce: landing (moving down at
    10 m/s from just above it) or, with `spin`, resting on it and spinning at 20 rad/s about y. Each frame's v.y and
    y (landing) or w.y (spinning) as the solver writes them."""
    box = {"size": (1, 1, 1), "center": (0, 0.5 if spin else 0.6, 0),
           "attrs": dict(attrs, active=1, bounce=0.0, friction=1.0 if spin else 0.5,
                         **({"w": (0.0, 20.0, 0.0)} if spin else {"v": (0.0, -10.0, 0.0)}))}
    ground = dict(GROUND, attrs=dict({k: -1.0 for k in attrs}, active=0, bounce=0.0, friction=1.0))
    path, names = packed_boxes(tag + "_pieces", [box, ground])
    dop = new_dopnet(tag)
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))

    def sample(sim, f):
        a = point_attributes(sim, "pieces", names[0], ["v", "w"])
        return a["w"][1] if spin else (a["v"][1], packed_positions(sim, "pieces")[names[0]][1])

    return step(dop, frames, sample)


def hard_pendulum_run(tag, kind, rest, anchor_a, anchor_b, hang_x):
    """A 1 m box at (hang_x, 2, 0) beside a fixed one at the origin, joined by a SideFX Hard (position) constraint from
    `anchor_a` on the fixed box to `anchor_b` on it, with Rest Length `rest`: per frame its centre (x, y) and its tilt
    about z in degrees."""
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (hang_x, 2, 0), "attrs": {"active": 1}}]
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path)
    hard = create(dop, "hardconrel", "hard")
    set_parm(hard, ["restlength"], rest)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(tag + "_network", [(names[0], anchor_a, names[1], anchor_b, "Hard",
                                                  {"constraint_type": "position"})]))
    net.setInput(1, hard)
    finish(dop, solver_for(kind, dop, net))

    def sample(sim, f):
        p = packed_positions(sim, "pieces")[names[1]]
        r = packed_transform(sim, "pieces", names[1])
        return p[0], p[1], math.degrees(math.atan2(r[0][1], r[0][0]))

    return step(dop, 12, sample)


def test_bullet_hard_pendulum():
    """A SideFX Hard (position) constraint as a pendulum, in both solvers: a 1 m box pinned at the middle of its left
    face, half a unit clear of a fixed box, swinging down from level. Once as a pin (Rest Length 0, the anchors
    together) and once with Bullet's rest offset (Rest Length 1, the anchors 1 apart: anchor A shifted onto B). Until
    the box can reach the fixed one (frame 12) it is the same pendulum in both. Passes when Box3D's centre is within
    0.02 and its tilt within 2 degrees of Bullet's on every frame. (Anchors that start together with a Rest Length
    are left out: the gap opens along whichever way each solver's first frame sags, see joints-sidefx.md.)"""
    cases = (("a pin", 0.0, (1.0, 2, 0), (1.0, 2, 0), 1.5),
             ("rest length 1", 1.0, (0.5, 2, 0), (1.5, 2, 0), 2.0))
    rows, ok = [], True
    for i, (label, rest, a, b, x) in enumerate(cases):
        got = {kind: hard_pendulum_run(f"hard_pendulum_{i}_{kind}", kind, rest, a, b, x) for kind in KINDS}
        worst_p = max(math.hypot(got["box3d"][f][0] - got["bullet"][f][0], got["box3d"][f][1] - got["bullet"][f][1])
                      for f in range(1, 13))
        worst_t = max(abs(got["box3d"][f][2] - got["bullet"][f][2]) for f in range(1, 13))
        ok = ok and worst_p < 0.02 and worst_t < 2.0
        bx, by, bt = got["box3d"][12]
        rows.append(f"{label}: on frame 12 Box3D ({bx:.2f}, {by:.2f}) {bt:.0f} deg, Bullet ({got['bullet'][12][0]:.2f}, "
                    f"{got['bullet'][12][1]:.2f}) {got['bullet'][12][2]:.0f} deg; largest differences {worst_p:.3f} "
                    f"and {worst_t:.1f} deg")
    return [Result("bullet_hard_pendulum", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def accel_damped_run(tag, attrs, spin):
    """Box3D only (Bullet reads no damping): a box falling freely with Box3D Data's Linear Damping 2, or spinning at 20
    rad/s about y with Angular Damping 2 and no gravity, with `attrs`. Its v.y or w.y after 1 s."""
    box = {"size": (1, 1, 1), "center": (0, 100, 0),
           "attrs": dict(attrs, active=1, **({"w": (0.0, 20.0, 0.0)} if spin else {}))}
    path, names = packed_boxes(tag + "_pieces", [box])
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    data = box3d_data(dop, obj, "data", **({"angulardamping": 2.0} if spin else {"lineardamping": 2.0}))
    finish(dop, box3d_solver(dop, data, gravity=(0, 0, 0) if spin else None))
    a = step(dop, 24, lambda sim, f: point_attributes(sim, "pieces", names[0], ["v", "w"]) if f == 24 else None)[24]
    return a["w"][1] if spin else a["v"][1]


def accel_joint_run(tag, kind, limit):
    """A box hanging beside a fixed one by a SideFX Hard (position) constraint, swinging down, with accelmax `limit`
    on it: its v.y on frames 18 and 24."""
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0, "accelmax": -1.0}},
              {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1, "accelmax": limit}}]
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path)
    hard = create(dop, "hardconrel", "hard")
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(tag + "_network", [(names[0], (0.5, 2, 0), names[1], (0.5, 2, 0), "Hard",
                                                  {"constraint_type": "position"})]))
    net.setInput(1, hard)
    finish(dop, solver_for(kind, dop, net))
    t = step(dop, 24, lambda sim, f: point_attributes(sim, "pieces", names[1], ["v"])["v"][1]
             if f in (18, 24) else None)
    return t[18], t[24]


def accel_chunk_run(tag, kind, light, heavy):
    """Two boxes glued side by side (1000 and 3000 kg, accelmax `light` and `heavy`) landing at 10 m/s: the chunk's
    v.y on frames 6 and 10."""
    base = {"active": 1, "density": 1000.0, "bounce": 0.0, "friction": 0.5, "v": (0.0, -10.0, 0.0)}
    pieces = [{"size": (1, 1, 1), "center": (-0.5, 0.6, 0), "attrs": dict(base, accelmax=light)},
              {"size": (1, 1, 1), "center": (0.5, 0.6, 0), "attrs": dict(base, accelmax=heavy, density=3000.0)},
              dict(GROUND, attrs=dict(base, active=0, accelmax=-1.0, v=(0.0, 0.0, 0.0)))]
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], 1e12)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(tag + "_network", [(names[0], (0, 0.6, 0), names[1], (0, 0.6, 0), "Glue")]))
    net.setInput(1, glue)
    finish(dop, solver_for(kind, dop, net))
    t = step(dop, 10, lambda sim, f: point_attributes(sim, "pieces", names[0], ["v"])["v"][1]
             if f in (6, 10) else None)
    return t[6], t[10]


def test_bullet_accel_limits():
    """accelmax and angaccelmax (the most the constraints may change a body's velocity / spin, per second), as RBD
    Bullet Solver: after the step, so the step's motion is kept and only the velocity carried on is changed. A box
    landing at 10 m/s with accelmax 1, 10 and 100: the ground still holds it (y 0.5), but its v.y loses at most
    accelmax / 24 per frame to the contact (with 1 it keeps falling faster on paper; with 100 it reaches 0 within a few
    frames). A box spinning at 20 rad/s on rough ground with angaccelmax 1 and 10: friction slows it by at most
    angaccelmax / 24 per frame (without a limit it loses about 1.7 a frame). Then: Box3D's own damping is unchanged by
    the limits (it is no constraint), a box swinging on a Hard constraint is limited as in Bullet, and a glued chunk
    takes its pieces' smallest accelmax, as Bullet's. Passes when Box3D's v.y on frames 6 and 8
    is within 0.15 of Bullet's, its box within 0.01 of y 0.5 on frame 8, and its w.y on frame 9 within 0.1 (0.3 without
    a limit, the control: plain friction, which differs a little between the solvers), the damped velocities with and
    without the limits within 0.01, and the jointed box's and the chunk's v.y within 0.15 of Bullet's."""
    rows, ok = [], True
    for limit in (1.0, 10.0, 100.0):
        got = {kind: accel_run(f"accel_land_{int(limit)}_{kind}", kind, {"accelmax": limit}) for kind in KINDS}
        a, b = got["box3d"], got["bullet"]
        good = all(abs(a[f][0] - b[f][0]) < 0.15 for f in (6, 8)) and abs(a[8][1] - 0.5) < 0.01
        ok = ok and good
        rows.append(f"landing, accelmax {limit:g}: v.y on frames 6 and 8 Box3D {a[6][0]:.2f} {a[8][0]:.2f}, Bullet "
                    f"{b[6][0]:.2f} {b[8][0]:.2f}; Box3D's box at y {a[8][1]:.3f}")
    for limit in (None, 1.0, 10.0):
        attrs = {} if limit is None else {"angaccelmax": limit}
        got = {kind: accel_run(f"accel_spin_{int(limit or 0)}_{kind}", kind, attrs, spin=True) for kind in KINDS}
        a, b = got["box3d"][9], got["bullet"][9]
        ok = ok and abs(a - b) < (0.3 if limit is None else 0.1)
        rows.append(f"spinning, angaccelmax {'none' if limit is None else f'{limit:g}'}: w.y on frame 9 Box3D {a:.2f}, "
                    f"Bullet {b:.2f}")
    # Damping is not the constraints' change: Box3D's own damping (Bullet reads none) is the same with the limits.
    for spin, attrs, label in ((False, {"accelmax": 1.0}, "fall v.y"), (True, {"angaccelmax": 1.0}, "spin w.y")):
        free = accel_damped_run(f"accel_damped_{int(spin)}_free", {}, spin)
        held = accel_damped_run(f"accel_damped_{int(spin)}_held", attrs, spin)
        ok = ok and abs(free - held) < 0.01
        rows.append(f"Box3D damping, {label} after 1 s: {free:.3f} without the limit, {held:.3f} with it")
    # A joint's change is limited as a contact's.
    got = {kind: accel_joint_run(f"accel_joint_{kind}", kind, 2.0) for kind in KINDS}
    ok = ok and all(abs(a - b) < 0.15 for a, b in zip(got["box3d"], got["bullet"]))
    rows.append(f"hanging by a Hard constraint, accelmax 2: v.y on frames 18 and 24 Box3D "
                f"{got['box3d'][0]:.2f} {got['box3d'][1]:.2f}, Bullet {got['bullet'][0]:.2f} {got['bullet'][1]:.2f}")
    # A glued chunk takes its pieces' smallest limit, whichever piece has it.
    for light, heavy in ((5.0, 50.0), (50.0, 5.0)):
        got = {kind: accel_chunk_run(f"accel_chunk_{int(light)}_{kind}", kind, light, heavy) for kind in KINDS}
        ok = ok and all(abs(a - b) < 0.15 for a, b in zip(got["box3d"], got["bullet"]))
        rows.append(f"glued chunk landing, accelmax {light:g} / {heavy:g}: v.y on frames 6 and 10 Box3D "
                    f"{got['box3d'][0]:.2f} {got['box3d'][1]:.2f}, Bullet {got['bullet'][0]:.2f} "
                    f"{got['bullet'][1]:.2f}")
    return [Result("bullet_accel_limits", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def glued_motion_run(tag, kind, a, b, spin=None, frames=96):
    """Two 1 m boxes glued side by side (a Strength nothing breaks: in Box3D one Glue as One Body compound), the first
    1000 kg, the second 3000 kg, with the drag / limit point attributes `a` and `b`. Falling freely, or with `spin`
    about y and no gravity: (the fall speed, the spin) over the last frame. The one-step inputs are written every
    frame by a SOP Solver, as in motion_run."""
    def split(attrs):
        return ({k: v for k, v in attrs.items() if k not in PER_STEP_MOTION},
                {k: v for k, v in attrs.items() if k in PER_STEP_MOTION})
    (stay_a, each_a), (stay_b, each_b) = split(a), split(b)
    base = {"active": 1, "density": 1000.0}
    pieces = [{"size": (1, 1, 1), "center": (-0.5, 1000, 0), "attrs": dict(base, **stay_a)},
              {"size": (1, 1, 1), "center": (0.5, 1000, 0), "attrs": dict(base, **stay_b, density=3000.0)}]
    if spin is not None:
        for p in pieces:
            p["attrs"]["w"] = (0.0, spin, 0.0)
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path, keep_velocity=spin is not None)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], 1e12)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(tag + "_network", [(names[0], (0, 1000, 0), names[1], (0, 1000, 0), "Glue")]))
    net.setInput(1, glue)
    vex = "".join(f'if (s@name == "{n}") {{ ' + "".join(f"f@{k} = {v}; " for k, v in d.items()) + "}\n"
                  for n, d in ((names[0], each_a), (names[1], each_b)) if d)
    if kind == "box3d":
        solver = box3d_solver(dop, None if vex else net, gravity=(0, 0, 0) if spin is not None else None)
        last = multisolver(dop, net, [sop_solver(dop, vex), solver]) if vex else solver
    else:
        solver = create(dop, "bulletrbdsolver", "bullet_solver")
        if vex:
            last = multisolver(dop, net, [sop_solver(dop, vex), solver])
        else:
            solver.setInput(0, net)
            last = solver
        if spin is None:
            gravity = create(dop, "gravity", "gravity")
            gravity.setInput(0, last)
            last = gravity
    finish(dop, last)
    track = step(dop, frames, lambda sim, f: (packed_positions(sim, "pieces")[names[0]],
                                              packed_transform(sim, "pieces", names[0])))
    fall = abs(track[frames][0][1] - track[frames - 1][0][1]) * 24.0
    turned = rotation_angle(track[frames - 1][1], track[frames][1]) * 24.0 * math.pi / 180.0
    return fall, turned


def test_bullet_glue_drag_limits():
    """The drag and speed limit attributes on a glued chunk, which Bullet runs as one body: two boxes glued side by
    side, 1000 and 3000 kg, each with its own values. Bullet's chunk takes the drag weighted by mass, the smallest
    maximum and the largest minimum (airresist 4000 on the light box ends the fall at sqrt(4000 g / 1000), on the heavy
    one at sqrt(4000 g / 3000); speedmax 3 and 6 cap it at 3 either way; spinmax 1 and 3 at 1; spinmin 2 and 4 hold
    it at 4). Box3D's Glue as One Body compound takes the same. Passes when Box3D is within 5 percent of Bullet's end
    fall speed under drag, 0.2 of its capped speed (speedmax is applied after Box3D's step, as in bullet_drag_limits)
    and 0.1 rad/s of its spin."""
    cases = (
        ("airresist on the light box", {"airresist": 4000.0}, {}, None, "drag"),
        ("airresist on the heavy box", {}, {"airresist": 4000.0}, None, "drag"),
        ("speedmax 3 and 6", {"speedmax": 3.0}, {"speedmax": 6.0}, None, "cap"),
        ("speedmax 6 and 3", {"speedmax": 6.0}, {"speedmax": 3.0}, None, "cap"),
        ("spinmax 1 and 3", {"spinmax": 1.0}, {"spinmax": 3.0}, 6.0, "spin"),
        ("spinmin 2 and 4", {"spinmin": 2.0}, {"spinmin": 4.0}, 0.5, "spin"),
    )
    rows, ok = [], True
    for i, (label, a, b, spin, what) in enumerate(cases):
        got = {kind: glued_motion_run(f"glue_motion_{i}_{kind}", kind, a, b, spin=spin) for kind in KINDS}
        if what == "spin":
            x, y = got["box3d"][1], got["bullet"][1]
            good = abs(x - y) < 0.1
            rows.append(f"{label}: spin Box3D {x:.3f}, Bullet {y:.3f} rad/s")
        else:
            x, y = got["box3d"][0], got["bullet"][0]
            good = abs(x - y) < (0.05 * y if what == "drag" else 0.2)
            rows.append(f"{label}: fall Box3D {x:.3f}, Bullet {y:.3f} m/s")
        ok = ok and good
    return [Result("bullet_glue_drag_limits", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def hull_spin_run(kind, sop_path, name, frames=12, bounds=True):
    """A 1 kg piece in free fall (gravity gives no torque) given a torque of (0, 0, 1) every frame by a SOP Solver, as
    a POP Force would: the spin it has about z on the last frame, from how far it turned over that frame (rad/s).
    `bounds`: the Box3D Solver's Bounding Box Inertia."""
    dop = new_dopnet(f"hull_spin_{name}_{kind}_{int(bounds)}")
    obj = packed_object(dop, "pieces", sop_path, computemass=0, mass=1.0)
    writer = sop_solver(dop, "v@torque = {0, 0, 1};")
    if kind == "box3d":
        solver = box3d_solver(dop, None)
        set_parm(solver, ["boundsinertia"], int(bounds))
        last = multisolver(dop, obj, [writer, solver])
    else:
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, obj, [writer, create(dop, "bulletrbdsolver", "bullet_solver")]))
    finish(dop, last)
    track = step(dop, frames, lambda sim, f: packed_transform(sim, "pieces"))
    return rotation_angle(track[frames - 1], track[frames]) * math.pi / 180.0 * 24.0


def hull_piece_sop(label):
    """A packed piece far above the origin: a box, or the platonic solid `label` (tetra, icosa)."""
    if label == "box":
        return packed_boxes("hull_spin_box_pieces", [{"size": (1, 1, 1), "center": (0, 1000, 0)}])[0]
    geo = new_geo(f"hull_spin_{label}")
    src = create(geo, "platonic")
    set_parm(src, ["solidtype", "type"], {"tetra": 0, "icosa": 4}[label])
    nm = create(geo, "attribwrangle", "name")
    nm.setInput(0, src)
    set_parm(nm, ["class"], 1)
    set_parm(nm, ["snippet"], 's@name = "piece";')
    asm = create(geo, "assemble")
    asm.setInput(0, nm)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, asm)
    set_parm(xf, ["t"], (0.0, 1000.0, 0.0))
    display(xf)
    return xf.path()


def test_bullet_hull_inertia():
    """A unit torque on a 1 kg box, tetrahedron and icosahedron for 0.5 s, spin in Bullet against Box3D. Bullet takes a
    hull's inertia from the box over it (plus its margin), which the default Bounding Box Inertia does too: PASS
    when the spins agree within 8% and a box (its own inertia is exact) within 3%, and with the toggle off the
    tetrahedron's spin differs from Bullet's by over 15% (a Box3D hull's own inertia is much smaller). The Solver SOP's
    Bounding Box Inertia is on by default."""
    rows, ok, off_apart = [], True, True
    for label in ("box", "tetra", "icosa"):
        sop_path = hull_piece_sop(label)
        bullet = hull_spin_run("bullet", sop_path, label)
        ours = hull_spin_run("box3d", sop_path, label)
        tol = 0.03 if label == "box" else 0.08
        ok = ok and abs(ours - bullet) <= tol * bullet
        text = f"{label} Bullet {bullet:.3f} Box3D {ours:.3f}"
        if label == "tetra":
            off = hull_spin_run("box3d", sop_path, label, bounds=False)
            off_apart = abs(off - bullet) > 0.15 * bullet
            text += f" (toggle off {off:.3f})"
        rows.append(text)
    sop = solver_sop("hull_inertia_sop")
    parm = sop.parm("boundsinertia")
    on_by_default = parm is not None and parm.eval() == 1
    rows.append(f"Solver SOP Bounding Box Inertia {'on' if on_by_default else 'missing or off'}")
    good = ok and off_apart and on_by_default
    return [Result("bullet_hull_inertia", Result.PASS if good else Result.FAIL, "spin about z in rad/s: " + "; ".join(rows))]


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


# ---------------------------------------------------------------------------
# computecom
# ---------------------------------------------------------------------------


def offset_bar(name):
    """One packed bar (2 x 1 x 1) whose point is at the origin and whose shape is centred 1 m along x:
     Returns (sop path, piece name)."""
    geo = new_geo(name)
    box = create(geo, "box", "bar")
    set_parm(box, ["size"], (2.0, 1.0, 1.0))
    set_parm(box, ["t"], (1.0, 0.0, 0.0))
    pack = create(geo, "pack", "pack")
    pack.setInput(0, box)
    set_parm(pack, ["pivot"], 0)  # origin: the point is not the shape's centre
    wr = create(geo, "attribwrangle", "attributes")
    wr.setInput(0, pack)
    set_parm(wr, ["class"], 0)
    set_parm(wr, ["snippet"], "s@name = \"bar\"; i@active = 1;")
    display(wr)
    return wr.path(), "bar"


def spin_reach(kind, computecom, frames=48):
    """(the furthest the bar's packed transform gets from the origin, where it ends) over `frames` frames with no
    gravity. The packed transform, not the point: Bullet moves the point to the centre of mass and writes
    `pivot`, Box3D keeps the point on the body origin, and the transform is what the geometry follows."""
    path, _ = offset_bar(f"computecom_{kind}_{computecom}_pieces")
    dop = new_dopnet(f"computecom_{kind}_{computecom}")
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, computecom=computecom, angvely=171.887),
                          gravity=(0, 0, 0)))
    if kind != "box3d":
        set_parm(dop.node("gravity"), ["force"], (0, 0, 0))

    def where(sim, f):
        prim = sim.findObject("pieces").geometry().prims()[0]
        return hou.Vector3(prim.intrinsicValue("packedfulltransform")[12:15])

    track = step(dop, frames, where)
    return max(p.length() for p in track.values()), track[frames]


def test_bullet_computecom():
    """computecom on a point: a bar whose shape is 1 m from its point, spun at 3 rad/s (RBD Packed Object's angular velocity, which also writes computecom) with no gravity. On, the
    centre of mass stays fixed and the piece swings round it (furthest reach about 2); off, the point is the
    centre and stays at the origin. Box3D must do both and agree with Bullet (within 0.02)."""
    reach = {(kind, com): spin_reach(kind, com) for kind in KINDS for com in (1, 0)}
    results = []
    for com in (1, 0):
        (rb, eb), (rt, et) = reach[("box3d", com)], reach[("bullet", com)]
        apart = (eb - et).length()
        if com:
            ok = rb > 1.5 and close(rb, rt, 0.02) and apart < 0.02
        else:
            ok = rb < 0.02 and rt < 0.02
        results.append(Result("bullet_computecom", Result.PASS if ok else Result.FAIL,
                              f"computecom {com}: the bar's transform reaches {rb:.3f} from the origin in Box3D, "
                              f"{rt:.3f} in Bullet, and the two end {apart:.4f} apart (on: about 2 and agreeing; "
                              f"off: it stays put)"))
    return results


def com_bar_scene(tag, extra=None):
    """The offset bar (2 x 1 x 1, its point at the origin, its shape centred 1 m along x, named `bar` on its point),
    merged with packed boxes `extra` (packed_boxes specs). Returns (sop path, the extra pieces' names)."""
    geo = new_geo(tag)
    box = create(geo, "box", "bar")
    set_parm(box, ["size"], (2.0, 1.0, 1.0))
    set_parm(box, ["t"], (1.0, 0.0, 0.0))
    pack = create(geo, "pack", "pack")
    pack.setInput(0, box)
    set_parm(pack, ["pivot"], 0)  # origin: the point is not the shape's centre
    wr = create(geo, "attribwrangle", "attributes")
    wr.setInput(0, pack)
    set_parm(wr, ["class"], 2)
    set_parm(wr, ["snippet"], 's@name = "bar"; i@active = 1;')
    last, names = wr, []
    if extra:
        path, names = packed_boxes(tag + "_extra", extra)
        source = create(geo, "object_merge", "extra")
        set_parm(source, ["objpath1"], path)
        last = create(geo, "merge", "both")
        last.setInput(0, wr)
        last.setInput(1, source)
    display(last)
    return last.path(), names


def com_bar_state(sim, f):
    """The bar's packed transform (position, 3x3 rotation), where its geometry is, and its v and w."""
    g = sim.findObject("pieces").geometry()
    i = next(k for k, p in enumerate(g.points()) if p.attribValue("name") == "bar")
    m = g.prims()[i].intrinsicValue("packedfulltransform")
    a = point_attributes(sim, "pieces", "bar", ["v", "w"])
    return hou.Vector3(m[12:15]), [[m[r * 4 + c] for c in range(3)] for r in range(3)], a["v"], a["w"]


def com_bar_run(tag, kind, case):
    """The offset bar with computecom on, in `case`: "spin" (3 rad/s about y and 1 m/s along x, no gravity), "pin"
    (hanging from the world by a SideFX Hard pin at its far end), "glue" (glued to a box beside its point, dropped onto
    the ground), "force" (point force (0, 20000, 0) and torque (0, 0, 2000) every frame, no gravity)."""
    extra = None
    if case == "glue":
        extra = [{"size": (1, 1, 1), "center": (-0.5, 0, 0), "attrs": {"active": 1}},
                 dict(GROUND, center=(0, -3.5, 0), attrs={"active": 0})]
    path, names = com_bar_scene(tag + "_geo", extra)
    dop = new_dopnet(tag)
    spin = {"angvely": 171.887, "vel": (1.0, 0.0, 0.0)} if case == "spin" else {}
    last = packed_object(dop, "pieces", path, computecom=1, **spin)
    if case in ("pin", "glue"):
        kind_node = create(dop, "hardconrel" if case == "pin" else "glueconrel", "joint")
        set_parm(kind_node, ["restlength"] if case == "pin" else ["strength"], 0.0 if case == "pin" else 1e12)
        net = create(dop, "constraintnetwork", "constraint_network")
        net.setInput(0, last)
        line = (("", (2, 0, 0), "bar", (2, 0, 0), "Hard", {"constraint_type": "position"}) if case == "pin"
                else ("bar", (0, 0, 0), names[0], (0, 0, 0), "Glue"))
        set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(tag + "_network", [line]))
        net.setInput(1, kind_node)
        last = net
    gravity = case in ("pin", "glue")
    writer = sop_solver(dop, "v@force = {0, 20000, 0}; v@torque = {0, 0, 2000};") if case == "force" else None
    if kind == "box3d":
        solver = box3d_solver(dop, None if writer else last, gravity=None if gravity else (0, 0, 0))
        last = multisolver(dop, last, [writer, solver]) if writer else solver
    else:
        solver = create(dop, "bulletrbdsolver", "bullet_solver")
        if writer:
            last = multisolver(dop, last, [writer, solver])
        else:
            solver.setInput(0, last)
            last = solver
        if gravity:
            pull = create(dop, "gravity", "gravity")
            pull.setInput(0, last)
            last = pull
    finish(dop, last)
    return step(dop, 36 if case == "glue" else 24, com_bar_state)


def test_bullet_computecom_cases():
    """computecom (on) with a velocity, a joint, Glue and forces, against Bullet: the offset bar (its point 1 m from its
    shape's centre; Bullet moves the point to the centre of mass, Box3D keeps it on the body origin, so the packed
    transform, which the geometry follows, is compared). Spinning at 3 rad/s while moving at 1 m/s: v and w are the
    centre's in both. Hanging by a pin at its far end, swinging. Pushed by a point force and twisted by a point torque:
    both act at the centre of mass. Glued to a box and dropped: the same through the fall. Passes when the transforms
    are within 0.01 and 0.2 degrees and v / w within 0.05 on every frame (the glued chunk to frame 12; after its
    landing within 0.15 and 1 degree: any box landing flat at speed tips slightly in Box3D, glued or not, see
    status.md)."""
    rows, ok = [], True
    for case in ("spin", "pin", "force", "glue"):
        got = {kind: com_bar_run(f"com_{case}_{kind}", kind, case) for kind in KINDS}
        worst_p = worst_r = worst_v = 0.0
        late_p = late_r = 0.0
        for f in sorted(got["box3d"]):
            a, b = got["box3d"][f], got["bullet"][f]
            dp, dr = (a[0] - b[0]).length(), rotation_angle(a[1], b[1])
            dv = max((hou.Vector3(a[2]) - hou.Vector3(b[2])).length(), (hou.Vector3(a[3]) - hou.Vector3(b[3])).length())
            if case == "glue" and f > 12:
                late_p, late_r = max(late_p, dp), max(late_r, dr)
                continue
            worst_p, worst_r, worst_v = max(worst_p, dp), max(worst_r, dr), max(worst_v, dv)
        good = worst_p < 0.01 and worst_r < 0.2 and worst_v < 0.05 and late_p < 0.15 and late_r < 1.0
        ok = ok and good
        rows.append(f"{case}: largest differences {worst_p:.4f}, {worst_r:.2f} deg, v / w {worst_v:.3f}"
                    + (f" (after landing {late_p:.3f}, {late_r:.2f} deg)" if case == "glue" else ""))
    return [Result("bullet_computecom_cases", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


# ---------------------------------------------------------------------------
# bullet_georep plane on a packed piece
# ---------------------------------------------------------------------------


def plane_rest(kind, georep, frames=90):
    """Height a 1 m box dropped from y 3 stops at, over a static piece at (0, 1, 0) whose shape is `georep`."""
    ground = {"size": (1, 1, 1), "center": (0, 1, 0),
              "attrs": {"active": 0, "bullet_georep": georep, "bullet_autofit": 1}}
    box = {"size": (1, 1, 1), "center": (0, 3, 0),
           "attrs": {"active": 1, "bullet_georep": "box", "bullet_autofit": 1}}
    path, names = packed_boxes(f"plane_{kind}_{georep}_pieces", [ground, box])
    dop = new_dopnet(f"plane_{kind}_{georep}")
    finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
    return step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])[frames]


def test_bullet_plane_piece():
    """bullet_georep plane on a static packed piece: a ground (a wide, deep box) through the piece's point facing +y,
    so a 1 m box dropped on it rests at the piece's height plus 0.5 (1.5 here). As a box the piece would
    carry the box up to 2.0. Box3D must give 1.5; Bullet's result is INFO."""
    results = []
    got = {kind: plane_rest(kind, "plane") for kind in KINDS}
    asbox = plane_rest("box3d", "box")
    ok = close(got["box3d"], 1.5, 0.03) and close(asbox, 2.0, 0.03)
    results.append(Result("bullet_plane_piece", Result.PASS if ok else Result.FAIL,
                          f"box on a plane piece at y 1 rests at {got['box3d']:.3f} in Box3D (expected 1.5); on a box "
                          f"piece {asbox:.3f} (expected 2.0)"))
    results.append(Result("bullet_plane_piece", Result.INFO,
                          f"Bullet: the same box on a plane piece ends at y {got['bullet']:.3f}"))
    return results


def test_bullet_force_sampling():
    """A Force DOP's Sample Mode in both solvers. A Drag Force DOP on a 2 x 2 x 2 box going 10 m/s along x, once per
    mode: RBD Bullet Solver samples every force at a point, so it slows the box the same with Point, Circle and Sphere;
    the Box3D Solver uses the mode, so Circle and Sphere (the force over the body's size) slow it more than Point, and
    its Point matches Bullet's. Passes when Bullet's three agree within 1 percent, the two Points within 5 percent, and
    Box3D's Circle and Sphere are more than 1.5 times its Point (the ratios in the message)."""
    lost = {}
    for kind in KINDS:
        for mode, label in ((1, "Point"), (2, "Circle"), (3, "Sphere")):
            tag = f"force_sampling_{kind}_{label.lower()}"
            path, names = packed_boxes(tag, [{"size": (2, 2, 2), "center": (0, 50, 0),
                                              "attrs": {"v": (10.0, 0.0, 0.0)}}])
            dop = new_dopnet(tag)
            obj = packed_object(dop, "piece", path, keep_velocity=True)
            solver = box3d_solver(dop, obj, gravity=(0, 0, 0)) if kind == "box3d" else solver_for(kind, dop, obj)
            drag = create(dop, "drag", "drag_force")
            drag.setInput(0, solver)
            set_parm(drag, ["samplemode"], mode)
            finish(dop, drag)
            vx = step(dop, 7, lambda sim, f: point_attributes(sim, "piece", names[0], ["v"]))[7]["v"][0]
            lost[kind, label] = 10.0 - vx
    point = {kind: lost[kind, "Point"] for kind in KINDS}
    if min(point.values()) <= 1e-6:
        return [Result("bullet_force_sampling", Result.FAIL,
                       f"the drag barely slowed the box at Point: Box3D {point['box3d']:.2e}, "
                       f"Bullet {point['bullet']:.2e} m/s")]
    ratio = {(kind, label): lost[kind, label] / point[kind] for kind in KINDS for label in ("Circle", "Sphere")}
    bullet_same = all(abs(ratio["bullet", label] - 1.0) < 0.01 for label in ("Circle", "Sphere"))
    points_match = abs(point["box3d"] - point["bullet"]) < 0.05 * point["bullet"]
    box3d_sampled = all(ratio["box3d", label] > 1.5 for label in ("Circle", "Sphere"))
    ok = bullet_same and points_match and box3d_sampled
    return [Result("bullet_force_sampling", Result.PASS if ok else Result.FAIL,
                   f"speed lost at Point: Box3D {point['box3d']:.5f}, Bullet {point['bullet']:.5f} m/s; "
                   f"Circle and Sphere over Point: Bullet {ratio['bullet', 'Circle']:.2f} and "
                   f"{ratio['bullet', 'Sphere']:.2f} (expected 1), Box3D {ratio['box3d', 'Circle']:.2f} and "
                   f"{ratio['box3d', 'Sphere']:.2f} (expected over 1.5)")]


def glued_slabs_run(tag, kind, compound, bottom, top, ground, lift=0.0, v0=0.0, frames=72):
    """Two 2 x 0.5 x 2 slabs stacked and glued (a Strength nothing breaks), so only the bottom one touches the
    ground; `bottom`, `top`, `ground`: point attributes. Starts `lift` above the ground moving at v0 along x. The
    bottom slab's position per frame. compound: the Box3D Solver's Glue as One Body."""
    slab = (2.0, 0.5, 2.0)
    pieces = [{"size": slab, "center": (0, 0.25 + lift, 0), "attrs": dict(bottom, active=1, v=(v0, 0.0, 0.0))},
              {"size": slab, "center": (0, 0.75 + lift, 0), "attrs": dict(top, active=1, v=(v0, 0.0, 0.0))},
              dict(GROUND, attrs=dict(ground, active=0))]
    path, names = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], 1e12)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    mid = (0, 0.5 + lift, 0)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(tag + "_network", [(names[0], mid, names[1], mid, "Glue")]))
    net.setInput(1, glue)
    solver = solver_for(kind, dop, net)
    if kind == "box3d":
        set_parm(solver, ["gluecompound"], int(compound))
        set_parm(solver, ["bouncethreshold"], 0.1)
    finish(dop, solver)
    return step(dop, frames, lambda sim, f: packed_positions(sim, "pieces")[names[0]])


def test_bullet_glue_materials():
    """A glued chunk's friction and bounce. Bullet runs a glue cluster as one body with the pieces' friction and bounce
    weighted by their mass, so a chunk of two stacked slabs (bottom friction 0.2 / bounce 0.2, top 1.0 / 0.8 and 3
    times as dense) slides and bounces with 0.8 and 0.65, not with the bottom slab's values, though only that one
    touches the ground. Friction from how far it slides from 3 m/s on ground of friction 1, bounce from its rebound
    dropped 2 m on ground of bounce 1. Box3D, with Glue as One Body on and off, gives every piece of the group that
    average. Then two slabs side by side, glued, sliding, with the bond taken out of the network mid-slide: each slab
    goes back to its own friction (the slippery one slides on, the grippy one stops), so they end where Bullet's do.
    Passes when each Box3D run is within 0.05 of Bullet's friction, 0.08 of its bounce (Bullet's bounce reads a little
    over the rule, as in bullet_mixing) and 0.05 of where its slabs end."""
    runs = (("bullet", False), ("box3d", True), ("box3d", False))
    base = {"density": 1000.0}
    bottom, top = dict(base, friction=0.2, bounce=0.2), dict(base, friction=1.0, bounce=0.8, density=3000.0)
    v0, height = 3.0, 2.0

    def friction(kind, compound):
        t = glued_slabs_run(f"glue_mat_slide_{kind}_{int(compound)}", kind, compound, dict(bottom, bounce=0.0),
                            dict(top, bounce=0.0), dict(base, friction=1.0, bounce=0.0), v0=v0)
        d = t[72][0]
        return v0 * v0 / (2 * G * d) if d > 1e-6 else float("inf")

    def bounce(kind, compound):
        t = glued_slabs_run(f"glue_mat_drop_{kind}_{int(compound)}", kind, compound, dict(bottom, friction=0.5),
                            dict(top, friction=0.5), dict(base, friction=0.5, bounce=1.0), lift=height)
        y = {f: p[1] - 0.25 for f, p in t.items()}
        frames = sorted(y)
        impact = next((f for f in frames[1:] if y[f] > y[f - 1] + 1e-5), None)
        if impact is None:
            return 0.0
        return math.sqrt(max(max(y[f] for f in frames if impact <= f <= impact + 40), 0.0) / height)

    def apart(kind, compound):
        """Two slabs side by side, glued, sliding along +x at 4 m/s (friction 0.2 in front, 1.0 and 3 times as dense
        behind), the bond taken out of the network from frame 7: where each ends on frame 48."""
        tag = f"glue_mat_apart_{kind}_{int(compound)}"
        slab = (1.0, 0.5, 2.0)
        moving = dict(base, bounce=0.0, active=1, v=(4.0, 0.0, 0.0))
        path, names = packed_boxes(tag + "_pieces", [
            {"size": slab, "center": (0.5, 0.25, 0), "attrs": dict(moving, friction=0.2)},
            {"size": slab, "center": (-0.5, 0.25, 0), "attrs": dict(moving, friction=1.0, density=3000.0)},
            dict(GROUND, attrs=dict(base, bounce=0.0, friction=1.0, active=0, v=(0.0, 0.0, 0.0)))])
        dop = new_dopnet(tag)
        obj = packed_object(dop, "pieces", path, keep_velocity=True)
        glue = create(dop, "glueconrel", "glue")
        set_parm(glue, ["strength"], 1e12)
        lines = hou.node(constraint_lines(tag + "_network", [(names[0], (0, 0.25, 0), names[1], (0, 0.25, 0), "Glue")]))
        cut = lines.parent().createNode("attribwrangle", "cut")
        cut.setInput(0, lines)
        set_parm(cut, ["class"], 1)
        set_parm(cut, ["snippet"], "if (@Frame >= 7) removeprim(0, @primnum, 1);")
        display(cut)
        net = create(dop, "constraintnetwork", "constraint_network")
        net.setInput(0, obj)
        set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], cut.path())
        net.setInput(1, glue)
        solver = solver_for(kind, dop, net)
        if kind == "box3d":
            set_parm(solver, ["gluecompound"], int(compound))
        finish(dop, solver)
        end = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces") if f == 48 else None)[48]
        return end[names[0]][0], end[names[1]][0]

    mu = {(k, c): friction(k, c) for k, c in runs}
    e = {(k, c): bounce(k, c) for k, c in runs}
    x = {(k, c): apart(k, c) for k, c in runs}
    ref = ("bullet", False)
    ok = all(abs(mu[r] - mu[ref]) < 0.05 and abs(e[r] - e[ref]) < 0.08
             and all(abs(a - b) < 0.05 for a, b in zip(x[r], x[ref])) for r in runs[1:])

    def name(r):
        return "Bullet" if r == ref else f"Box3D {'compound' if r[1] else 'welds'}"

    return [Result("bullet_glue_materials", Result.PASS if ok else Result.FAIL,
                   "friction " + ", ".join(f"{name(r)} {mu[r]:.3f}" for r in runs) + " (mass-weighted 0.80); bounce "
                   + ", ".join(f"{name(r)} {e[r]:.3f}" for r in runs) + " (mass-weighted 0.65); bond removed "
                   "mid-slide, where the slippery and grippy slabs end: "
                   + ", ".join(f"{name(r)} {x[r][0]:.2f} and {x[r][1]:.2f}" for r in runs)
                   + " (expected within 0.05 of Bullet: each slab back on its own friction)")]


TESTS = {
    "bullet_computecom": test_bullet_computecom,
    "bullet_computecom_cases": test_bullet_computecom_cases,
    "bullet_glue_materials": test_bullet_glue_materials,
    "bullet_force_sampling": test_bullet_force_sampling,
    "bullet_plane_piece": test_bullet_plane_piece,
    "bullet_rest_rescan": test_bullet_rest_rescan,
    "bullet_mirror_flip": test_bullet_mirror_flip,
    "bullet_joint_angle": test_bullet_joint_angle,
    "bullet_joint_feedback": test_bullet_joint_feedback,
    "bullet_hard_collisions": test_bullet_hard_collisions,
    "bullet_soft_spring_hz": test_soft_spring_hz,
    "bullet_soft_coincident_anchors": test_soft_coincident_anchors,
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
    "bullet_hull_inertia": test_bullet_hull_inertia,
    "bullet_drag_limits": test_bullet_drag_limits,
    "bullet_glue_drag_limits": test_bullet_glue_drag_limits,
    "bullet_accel_limits": test_bullet_accel_limits,
    "bullet_hard_pendulum": test_bullet_hard_pendulum,
    "bullet_spring_cap": test_bullet_spring_cap,
    "bullet_position_rest_length": test_bullet_position_rest_length,
    "bullet_joint_rest_held": test_bullet_joint_rest_held,
    "bullet_rbd_box_settle": test_bullet_rbd_box_settle,
    "bullet_rbd_pivot": test_bullet_rbd_pivot,
    "bullet_cone_twist_goal": test_bullet_cone_twist_goal,
    "bullet_impact_normal": test_bullet_impact_normal,
    "bullet_kinematic_push": test_bullet_kinematic_push,
    "bullet_kinematic_object_push": test_bullet_kinematic_object_push,
    "bullet_inactive_push": test_bullet_inactive_push,
    "bullet_primitive_types": test_bullet_primitive_types,
}
