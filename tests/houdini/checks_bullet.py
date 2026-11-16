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
    # 0.57 to 0.67: the order shows.
    ("box, half extents 0.6 0.1 0.3, bullet_primR 50 70 50", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (50, 70, 50)),
    ("box, half extents 0.6 0.1 0.3, bullet_primR 0 70 50", "box", (0.6, 0.1, 0.3), 0.5, 1.0, (0, 0, 0),
     (0, 70, 50)),
    ("sphere, bullet_radius 0.3", "sphere", (1, 1, 1), 0.3, 1.0, (0, 0, 0), (0, 0, 0)),
    ("capsule, radius 0.25 length 1 (bullet_length: straight part or total?)", "capsule", (1, 1, 1), 0.25, 1.0,
     (0, 0, 0), (0, 0, 0)),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 90", "capsule", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 90)),
    ("capsule, radius 0.25 length 1, bullet_primR 0 0 45", "capsule", (1, 1, 1), 0.25, 1.0, (0, 0, 0), (0, 0, 45)),
)


def shape_attrs(georep, prim_s, radius, length, prim_t, prim_r):
    return {"bullet_georep": georep, "bullet_autofit": 0, "bullet_autofit_valid": 1,
            "bullet_primS": tuple(float(v) for v in prim_s), "bullet_radius": float(radius),
            "bullet_length": float(length), "bullet_primT": tuple(float(v) for v in prim_t),
            "bullet_primR": tuple(float(v) for v in prim_r), "bounce": 0.0, "v": (0.0, 0.0, 0.0),
            "w": (0.0, 0.0, 0.0)}


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


def test_bullet_shape_position():
    """Where a hand-set shape is: Bullet's static ones were seen at the world origin, whatever the
    piece's position (a plate dropped over a piece at x 3 landed on a shape at the origin). A static
    piece at (3, 1, 0) with a box of half extents 0.5 0.25 0.5: the top of the shape above the piece's
    centre (0.25 if it follows the piece). Then a dynamic piece with the same shape offset up 0.3
    (primT) dropped on the ground: where it rests (-0.05 if the shape follows it). Both within
    0.03 of each other."""
    results = []
    attrs = shape_attrs("box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0, 0), (0, 0, 0))
    top = {kind: shape_top("shape_pos_static", kind, attrs, center=(3.0, 1.0, 0.0)) for kind in KINDS}
    results.append(Result("bullet_shape_position", Result.PASS if close(top["box3d"], top["bullet"], 0.03) else Result.FAIL,
                          f"static piece at (3, 1, 0): top {top['box3d']:.3f} above its centre in Box3D, "
                          f"{top['bullet']:.3f} in Bullet (NaN: the plate never stopped; 0.25 = the shape is "
                          f"on the piece)"))
    rest = {}
    offset = shape_attrs("box", (0.5, 0.25, 0.5), 0.5, 1.0, (0, 0.3, 0), (0, 0, 0))
    for kind in KINDS:
        pieces = [dict(GROUND, attrs=dict(offset, active=0, bullet_georep="box", bullet_autofit=1)),
                  {"size": (1, 1, 1), "center": (3.0, 1.5, 0.0), "attrs": dict(offset, active=1)}]
        path, names = packed_boxes(f"shape_pos_dynamic_{kind}_pieces", pieces)
        dop = new_dopnet(f"shape_pos_dynamic_{kind}")
        finish(dop, solver_for(kind, dop, packed_object(dop, "pieces", path, keep_velocity=True)))
        rest[kind] = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1])[72]
    results.append(Result("bullet_shape_position", Result.PASS if close(rest["box3d"], rest["bullet"], 0.03) else Result.FAIL,
                          f"dynamic piece with its box offset up 0.3: rests at y {rest['box3d']:.3f} in Box3D, "
                          f"{rest['bullet']:.3f} in Bullet (-0.05 = the offset is from the piece's centre)"))
    return results


# ---------------------------------------------------------------------------
# Joints on a piece that has already turned, and Cone Twist's goal end
# ---------------------------------------------------------------------------

CENTRES = ((0.0, 4.0, 0.0), (0.5, 2.0, 0.0))  # the fixed piece and the hanging one, 2.06 apart
SHARED = ((0.25, 3.0, 0.0), (0.25, 3.0, 0.0))  # one point between them: the centres end up at most 2.06 apart


def joint_run(tag, kind, conrel_type, line_attrs, hang_rotate, anchors, spin):
    """A piece hanging 2 below a fixed one through a SideFX constraint, the hanging piece turned by
    `hang_rotate` before the start: (centre distance, how far it turned since frame 1 in degrees)
    after 2 s."""
    hang = {"active": 1, "w": tuple(spin)}
    fixed = {"active": 0, "w": (0.0, 0.0, 0.0)}
    pieces = [{"size": (1, 1, 1), "center": CENTRES[0], "attrs": fixed},
              {"size": (1, 1, 1), "center": CENTRES[1], "rotate": hang_rotate, "attrs": hang}]
    dop, names = hanging_pair(f"{tag}_{kind}", None, line_attrs=line_attrs, conrels=[(conrel_type, {})],
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
    (restToNow) and the `angle` since rest show: a piece held rigidly stays at 2.06 and does not
    turn. (constraint_type position holds the 1.0 rest length instead, which pulls the two boxes
    into each other.) Distances within 0.1, turns within 15 degrees. With both anchors at one point
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


def primitive_rest(tag, kind, sop_path, packed, frames=72):
    """Where a primitive dropped on the ground ends up: the height of its centre (a packed piece's
    point, or the mean of a whole object's points) on the last frame, and the solver's messages."""
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

    return step(dop, frames, sample)[frames], messages


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
                y, messages = got[kind]
                if y is None:
                    return "nothing left"
                text = "fell through" if y < -1.0 else f"y {y:.3f}"
                return text + (f" ({messages[0][:70]})" if messages else "")

            yb, yt = got["box3d"][0], got["bullet"][0]
            same = yb is not None and yt is not None and abs(yb - yt) < 0.05
            hint = "" if ideal is None else f", a correctly sized shape rests at {ideal:g}"
            results.append(Result("bullet_primitive_types", Result.INFO,
                                  f"{label}, {route}: Box3D {show('box3d')}, Bullet {show('bullet')}"
                                  f"{hint}; {'the same' if same else 'differ'}"))
    return results


TESTS = {
    "bullet_rest_convention": test_bullet_rest_convention,
    "bullet_shape_attributes": test_bullet_shape_attributes,
    "bullet_shape_position": test_bullet_shape_position,
    "bullet_joint_rest": test_bullet_joint_rest,
    "bullet_cone_twist_goal": test_bullet_cone_twist_goal,
    "bullet_impact_normal": test_bullet_impact_normal,
    "bullet_kinematic_push": test_bullet_kinematic_push,
    "bullet_primitive_types": test_bullet_primitive_types,
}
