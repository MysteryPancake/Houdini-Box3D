"""The Box3D Solver SOP's collision input and ground: collider shapes, names and groups, animated and static
colliders, the ground box and its guide."""

import math

import hou
from b3dtest import GROUND, Result, close, constraint_lines, create, display, new_geo, packed_boxes, set_parm

from checks_sop_common import _both_sops, bullet_sop, named_positions, play, solver_sop


def test_solver_sop_collision():
    """A packed box dropped onto a ground box on the Collision input rests on it (y = 0.5) after 2 s;
    the proxy output is empty (no proxy input, as RBD Bullet Solver's), the geometry carries w (Transfer to
    Geometry), and the simulation points output has one point per piece, on the geometry's piece.
    With Enable Collision Geometry off the box falls through the ground."""
    pieces, names = packed_boxes("sop_drop_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    ground, _ = packed_boxes("sop_drop_ground", [GROUND])
    sop = solver_sop("sop_drop", geometry=pieces, collision=ground)

    def sample(s, f):
        if f != 48:
            return None
        pts = s.geometry(3)
        return (named_positions(s.geometry(0)), named_positions(s.geometry(2)), named_positions(pts),
                len(pts.prims()), s.geometry(0).findPointAttrib("w") is not None)

    geo_pos, proxy_pos, sim_pos, sim_prims, geo_w = play(sop, 48, sample)[48]
    y = geo_pos.get(names[0], hou.Vector3(0, 99, 0))[1]
    same = not proxy_pos
    pts_ok = (len(sim_pos) == 1 and sim_prims == 0 and names[0] in sim_pos
              and (sim_pos[names[0]] - geo_pos.get(names[0], hou.Vector3(0, 99, 0))).length() < 1e-4)
    ok = close(y, 0.5, 0.03) and same and pts_ok and geo_w
    results = [Result("solver_sop_collision", Result.PASS if ok else Result.FAIL,
                      f"box at y={y:.3f} after 2 s (expected 0.5); proxy output empty: {same}; "
                      f"w transferred to Geometry: {geo_w}; "
                      f"simulation points: {len(sim_pos)} point(s), {sim_prims} primitive(s), on the piece: {pts_ok} "
                      f"(expected 1 point, 0 primitives, True)")]

    # Enable Collision Geometry off: the Collision input is ignored and the box falls through.
    off = solver_sop("sop_drop_off", geometry=pieces, collision=ground)
    set_parm(off, ["usecollisions"], 0)
    pos = play(off, 48, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == 48 else None)[48]
    y_off = pos[1] if pos else None
    results.append(Result("solver_sop_collision", Result.PASS if y_off is not None and y_off < -1.0 else Result.FAIL,
                          f"Enable Collision Geometry off: box at y={y_off} after 2 s (expected below -1: fell through)"))
    return results


def test_solver_sop_collision_static():
    """Collide As Static Object on the Collision input: an unpacked grid and a flat HeightField SOP each hold a
    box at y = 0.5 (with Enable Collision Geometry on only), a packed collider stays packed pieces by default,
    the Material settings reach the Static Object's Box3D Data."""
    pieces, names = packed_boxes("sop_static_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])

    def settle(sop, frames=72):
        track = play(sop, frames, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == frames else None)
        return track[frames][1] if track[frames] else None

    results = []
    grid_geo = new_geo("sop_static_grid")
    grid = create(grid_geo, "grid", "grid")
    set_parm(grid, ["sizex"], 10)
    set_parm(grid, ["sizey"], 10)
    display(grid)
    field_geo = new_geo("sop_static_field")
    field = create(field_geo, "heightfield", "field")
    set_parm(field, ["sizex"], 20)
    set_parm(field, ["sizey"], 20)
    display(field)
    for label, path in (("polygon grid", grid.path()), ("height field", field.path())):
        sop = solver_sop(f"sop_static_{label.split()[0]}", geometry=pieces, collision=path)
        set_parm(sop, ["collisionsource"], 1)
        y = settle(sop)
        results.append(Result("solver_sop_collision_static", Result.PASS if close(y or 0, 0.5, 0.05) else Result.FAIL,
                              f"Static Object, {label}: box at y={y} after 3 s (expected 0.5)"))

    off = solver_sop("sop_static_off", geometry=pieces, collision=grid.path())
    set_parm(off, ["collisionsource"], 1)
    set_parm(off, ["usecollisions"], 0)
    y = settle(off)
    results.append(Result("solver_sop_collision_static", Result.PASS if y is not None and y < -1.0 else Result.FAIL,
                          f"Static Object with Enable Collision Geometry off: box at y={y} (expected below -1: fell)"))

    # Packed colliders stay the default: a packed ground box still holds the box.
    ground, _ = packed_boxes("sop_static_packed_ground", [GROUND])
    packed = solver_sop("sop_static_packed", geometry=pieces, collision=ground)
    y = settle(packed)
    results.append(Result("solver_sop_collision_static", Result.PASS if close(y or 0, 0.5, 0.03) else Result.FAIL,
                          f"Packed Pieces (the default): box at y={y} (expected 0.5)"))

    # Material reaches the Static Object's Box3D Data.
    mat = solver_sop("sop_static_material", geometry=pieces, collision=grid.path())
    for name, value in {"collision_friction": 0.3, "collision_bounce": 0.7}.items():
        set_parm(mat, [name], value)
    data = mat.node("dopnet/collision_static_data")
    got = (data.parm("friction").eval(), data.parm("bounce").eval())
    ok = close(got[0], 0.3, 1e-6) and close(got[1], 0.7, 1e-6)
    results.append(Result("solver_sop_collision_static", Result.PASS if ok else Result.FAIL,
                          f"Static Object's Box3D Data friction, bounce: {got} (expected 0.3, 0.7)"))
    return results


def test_solver_sop_animated_collision():
    """A ground box on the Collision input rising 0.5 m/s (keyed on its Transform SOP) lifts the box
    resting on it: with Collision Type Animated (the default) the box ends 1 m higher after 2 s; off,
    the ground stays where it started and the box stays at y = 0.5."""
    results = []
    for animated in (1, 0):
        tag = "on" if animated else "off"
        pieces, names = packed_boxes(f"sop_lift_pieces_{tag}", [{"size": (1, 1, 1), "center": (0, 0.5, 0)}])
        ground, _ = packed_boxes(f"sop_lift_ground_{tag}", [GROUND])
        place = hou.node(ground).parent().node("place0")
        place.parm("ty").setExpression("-0.5 + 0.5 * $T", hou.exprLanguage.Hscript)
        sop = solver_sop(f"sop_lift_{tag}", geometry=pieces, collision=ground)
        set_parm(sop, ["collisiontype"], 1 if animated else 0)  # Animated / Static
        track = play(sop, 49, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f in (1, 49) else None)
        y0 = track[1][1] if track[1] else None
        y1 = track[49][1] if track[49] else None
        want = 1.5 if animated else 0.5
        ok = y1 is not None and close(y1, want, 0.05)
        results.append(Result("solver_sop_animated_collision", Result.PASS if ok else Result.FAIL,
                              f"Collision Type {'Animated' if animated else 'Static'}: box y {y0} on frame 1, {y1} on frame 49 "
                              f"(expected about {want}; the ground's top rises from 0 to 1)"))
    return results


def test_solver_sop_collision_names():
    """As RBD Bullet Solver's Collisions box: an unpacked collider is assembled with the prefix
    `static`, named on its point."""
    ground = new_geo("sop_colnames_ground")
    box = create(ground, "box", "ground")
    set_parm(box, ["size"], (20, 1, 20))
    set_parm(box, ["t"], (0, -0.5, 0))
    display(box)
    pieces, _ = packed_boxes("sop_colnames_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    sop = solver_sop("sop_collision_names", geometry=pieces, collision=box.path())
    play(sop, 2, lambda s, f: None)
    dop = sop.node("dopnet")
    obj = dop.simulation().findObject("collision") if dop is not None else None
    g = obj.geometry() if obj is not None else None
    names_now = sorted({p.attribValue("name") for p in g.points()}) if g is not None and g.findPointAttrib("name") else []
    ok = len(names_now) == 1 and names_now[0].startswith("static")
    return [Result("solver_sop_collision_names", Result.PASS if ok else Result.FAIL,
                   f"collider point names {names_now} (expected one name starting with 'static')")]


GROUP_CASES = [
    # (tag, lower (group, ignore), upper (group, ignore), expected y of the upper box)
    ("plain", ("debris", ""), ("debris", ""), 1.5),
    ("lower_ignores_group_only", ("debris", "debris"), ("debris", ""), 1.5),
    ("upper_ignores_other_group_only", ("a", ""), ("b", "a"), 1.5),
    ("lower_ignores_upper_group_only", ("a", "b"), ("b", ""), 1.5),
    ("both_ignore_each_other", ("a", "b"), ("b", "a"), 0.5),
    ("both_ignore_own_group", ("debris", "debris"), ("debris", "debris"), 0.5),
    ("both_ignore_by_pattern", ("a", "d*"), ("debris", "a"), 0.5),
    ("other_group_ignored", ("a", "zzz"), ("b", ""), 1.5),
    ("pattern_one_side", ("a", "d*"), ("debris", ""), 1.5),
    ("pattern_taken_out", ("a", "d* ^debris"), ("debris", "a"), 1.5),
]


def _group_names_run(kind, tag, lower, upper, ground):
    """The height after 2 s of a box dropped on another with collisiongroup / collisionignore on the two, in the Box3D
    Solver SOP (kind "box3d") or the RBD Bullet Solver SOP."""
    (g0, i0), (g1, i1) = lower, upper
    pieces, names = packed_boxes(f"sop_cgn_{kind}_pieces_{tag}", [
        {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"collisiongroup": g0, "collisionignore": i0}},
        {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"collisiongroup": g1, "collisionignore": i1}}])
    sop = (solver_sop if kind == "box3d" else bullet_sop)(f"sop_cgn_{kind}_{tag}", geometry=pieces, collision=ground)
    pos = play(sop, 48, lambda s, f: named_positions(s.geometry(0)) if f == 48 else None)[48]
    return pos[names[1]][1] if names[1] in pos else None


def test_solver_sop_collision_group_names():
    """Bullet's collisiongroup / collisionignore on the pieces become collision bits in the Solver SOP, as in RBD Bullet
    Solver (SIM_BulletRigidBody::checkCollideWithOverride): two pieces collide when either one's `collisionignore`
    leaves the other's group alone, so only both ignoring each other lets them pass. A box dropped onto another rests
    on it (y = 1.5) unless each ignores the other's group (by name or pattern), and then it falls through to the ground
    (y = 0.5). One side ignoring, a pattern that matches no group, and a `^` that takes the group out again change
    nothing."""
    ground, _ = packed_boxes("sop_cgn_ground", [GROUND])
    results = []
    for tag, lower, upper, want in GROUP_CASES:
        y = _group_names_run("box3d", tag, lower, upper, ground)
        ok = y is not None and close(y, want, 0.05)
        results.append(Result("solver_sop_collision_group_names", Result.PASS if ok else Result.FAIL,
                              f"{tag}: lower {lower[0]!r} ignoring {lower[1]!r}, upper {upper[0]!r} ignoring "
                              f"{upper[1]!r}: upper box at y={y} after 2 s (expected {want})"))
    return results


def test_solver_sop_collision_group_names_bullet():
    """solver_sop_collision_group_names' cases (one box dropped on another, groups and ignore patterns on the two) in
    RBD Bullet Solver: the upper box ends where Box3D's does in every case, so Bullet too lets only both pieces
    ignoring each other pass."""
    ground, _ = packed_boxes("sop_cgnb_ground", [GROUND])
    differ, same = [], []
    for tag, lower, upper, want in GROUP_CASES:
        y = _group_names_run("bullet", tag, lower, upper, ground)
        (same if y is not None and close(y, want, 0.05) else differ).append(f"{tag} (y={y and round(y, 2)}, expected {want})")
    return [Result("solver_sop_collision_group_names_bullet", Result.FAIL if differ else Result.PASS,
                   f"Bullet agrees in: {', '.join(same) or 'no case'}. It differs in: {'; '.join(differ) or 'no case'}")]


def test_solver_sop_collider_group_names():
    """The collision input's pieces take part in collision groups like the pieces: its `collisiongroup` and
    `collisionignore` become the same category / mask bits, and the same rule as between pieces holds (Bullet's: they
    collide unless both ignore each other). A box dropped onto a ground piece rests on it (y = 0.5), and falls through
    (below y = -1 after 2 s) only when the box ignores the ground's group and the ground the box's, by name or pattern. One
    side ignoring, a pattern matching nothing, a ground ignoring only its own group, and a ground with no group
    attributes at all (which the box ignores nothing of) change nothing. The same cases in RBD Bullet Solver give the
    same."""
    cases = [
        # (tag, ground (group, ignore) or None for no attributes, box (group, ignore), falls through)
        ("plain", ("floor", ""), ("debris", ""), False),
        ("box_ignores_ground_only", ("floor", ""), ("debris", "floor"), False),
        ("ground_ignores_box_only", ("floor", "debris"), ("debris", ""), False),
        ("both_ignore_each_other", ("floor", "debris"), ("debris", "floor"), True),
        ("both_ignore_by_pattern", ("floor", "deb*"), ("debris", "fl*"), True),
        ("ground_pattern_no_match", ("floor", "zzz"), ("debris", ""), False),
        ("ground_ignores_itself", ("floor", "floor"), ("debris", ""), False),
        ("ground_without_names", None, ("debris", "floor"), False),
        ("ground_without_names_box_ignores_all", None, ("debris", "*"), False),
    ]
    results, bullet_notes = [], []
    for tag, ground_names, (g1, i1), through in cases:
        attrs = {"active": 0}
        if ground_names is not None:
            attrs.update(collisiongroup=ground_names[0], collisionignore=ground_names[1])
        ground, _ = packed_boxes(f"sop_cgc_ground_{tag}", [dict(GROUND, attrs=attrs)])
        pieces, names = packed_boxes(f"sop_cgc_pieces_{tag}", [
            {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"collisiongroup": g1, "collisionignore": i1}}])
        sops = _both_sops(f"sop_cgc_{tag}", geometry=pieces, collision=ground)
        ys = {}
        for kind, sop in sops.items():
            pos = play(sop, 48, lambda s, f: named_positions(s.geometry(0)) if f == 48 else None)[48]
            ys[kind] = pos[names[0]][1] if names[0] in pos else None
        fell = {kind: y is not None and y < -1.0 for kind, y in ys.items()}
        rested = {kind: y is not None and close(y, 0.5, 0.05) for kind, y in ys.items()}
        expected = fell if through else rested
        results.append(Result("solver_sop_collider_group_names",
                              Result.PASS if expected["box3d"] and expected["bullet"] else Result.FAIL,
                              f"{tag}: ground {ground_names}, box ({g1!r}, {i1!r}): box at y={ys['box3d']} in Box3D and "
                              f"{ys['bullet']} in Bullet after 2 s (expected {'below -1' if through else '0.5'} in both)"))
    return results


def collider_points(sop):
    """The collision object's points in the Solver SOP's simulation as [{attribute: value}], or None."""
    dop = sop.node("dopnet")
    obj = dop.simulation().findObject("collision") if dop is not None else None
    g = obj.geometry() if obj is not None else None
    if g is None:
        return None
    return [{a.name(): p.attribValue(a) for a in g.pointAttribs()} for p in g.points()], \
        (g.findPrimAttrib("name") is not None and [pr.attribValue("name") for pr in g.prims()])


def test_solver_sop_collision_prim_names():
    """Collision mode's naming of packed colliders, as RBD Bullet Solver's Collisions box: a packed
    collider named only on its primitive gets that name on its point and loses the primitive's
    (promoted); one named on both keeps both, the point's name being its name."""
    results = []
    for tag, snippet in (("prim_only", 'string n = point(0, "name", primpoint(0, @primnum, 0)); '
                                       's@name = n;'),
                         ("both", 's@name = "prim_name";')):
        ground, gnames = packed_boxes(f"sop_colprim_ground_{tag}", [GROUND])
        geo = new_geo(f"sop_colprim_in_{tag}")
        src = create(geo, "object_merge", "ground")
        set_parm(src, ["objpath1"], ground)
        on_prim = create(geo, "attribwrangle", "name_on_prim")
        on_prim.setInput(0, src)
        set_parm(on_prim, ["class"], "primitive")
        set_parm(on_prim, ["snippet"], snippet)
        last = on_prim
        if tag == "prim_only":
            last = create(geo, "attribdelete", "no_point_name")
            last.setInput(0, on_prim)
            set_parm(last, ["ptdel"], "name")
        display(last)
        pieces, _ = packed_boxes(f"sop_colprim_pieces_{tag}", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        sop = solver_sop(f"sop_collision_prim_names_{tag}", geometry=pieces, collision=last.path())
        play(sop, 2, lambda s_, f: None)
        got = collider_points(sop)
        pts, prim_names = got if got else (None, None)
        point_name = pts[0].get("name") if pts else None
        if tag == "prim_only":
            ok = point_name == gnames[0] and not prim_names
            want = f"point name {gnames[0]!r}, no primitive name"
        else:
            ok = point_name == gnames[0] and prim_names == ["prim_name"]
            want = f"point name {gnames[0]!r}, primitive name kept ['prim_name']"
        results.append(Result("solver_sop_collision_prim_names", Result.PASS if ok else Result.FAIL,
                              f"{tag}: collider point name {point_name!r}, primitive names {prim_names or 'none'} "
                              f"(expected {want})"))
    return results


def test_solver_sop_collision_rest():
    """An animated collider that turns (5 degrees about y per frame, Collision Type Animated) keeps the
    rest it had on the Start Frame (collision_rest, as RBD Bullet Solver's collision repack): on frame
    13 its restxform is still turned 5 degrees (frame 1) and its orient is the 60 degrees turned since."""
    bar, _ = packed_boxes("sop_colrest_bar", [{"size": (4, 1, 1), "center": (0, -0.5, 0), "attrs": {"active": 0}}])
    geo = new_geo("sop_colrest_turning")
    src = create(geo, "object_merge", "bar")
    set_parm(src, ["objpath1"], bar)
    turn = create(geo, "xform", "turn")  # turns the packed transform (after packing), not the geometry inside it
    turn.setInput(0, src)
    turn.parm("ry").setExpression("5 * $F", hou.exprLanguage.Hscript)
    display(turn)
    pieces, _ = packed_boxes("sop_colrest_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    sop = solver_sop("sop_collision_rest", geometry=pieces, collision=turn.path())
    track = play(sop, 13, lambda s_, f: collider_points(s_) if f == 13 else None)
    pts = track[13][0] if track[13] else None
    rest_deg = since_deg = None
    if pts and "restxform" in pts[0] and "orient" in pts[0]:
        m = pts[0]["restxform"]
        rest_deg = math.degrees(math.atan2(-m[2], m[0]))  # the x axis's turn about y
        q = pts[0]["orient"]
        since_deg = math.degrees(2 * math.acos(min(1.0, abs(q[3]))))
    ok = rest_deg is not None and close(abs(rest_deg), 5.0, 0.5) and close(since_deg, 60.0, 0.5)
    return [Result("solver_sop_collision_rest", Result.PASS if ok else Result.FAIL,
                   f"collider on frame 13: restxform turned {rest_deg} degrees (expected 5: the Start Frame's), "
                   f"orient turned {since_deg} degrees (expected 60: since then)")]


def test_solver_sop_world_anchor():
    """The constraints output moves each anchor with its piece and leaves a world anchor where it was
    authored (as RBD Bullet Solver's reset_pin_locations): a box on a distance joint to a world point 2
    above it, pushed sideways, swings; on frame 24 the world end is still at (0, 5, 0) and the box's
    end is at the box."""
    pieces, names = packed_boxes("sop_wa_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0),
                                                    "attrs": {"v": (3.0, 0.0, 0.0)}}])
    lines = constraint_lines("sop_wa_network", [("", (0, 5, 0), names[0], (0, 3, 0), "Distance")])
    sop = solver_sop("sop_world_anchor", geometry=pieces, constraints=lines)

    def sample(s_, f):
        if f != 24:
            return None
        con = s_.geometry(1)
        ends = {p.attribValue("name"): p.position() for p in con.points()} \
            if con is not None and con.findPointAttrib("name") else {}
        return ends, named_positions(s_.geometry(3)).get(names[0])

    ends, box = play(sop, 24, sample)[24]
    world, mine = ends.get(""), ends.get(names[0])
    ok = (world is not None and (world - hou.Vector3(0, 5, 0)).length() < 1e-4 and box is not None
          and abs(box[0]) > 0.1 and mine is not None and (mine - box).length() < 1e-3)
    return [Result("solver_sop_world_anchor", Result.PASS if ok else Result.FAIL,
                   f"frame 24: world end at {tuple(round(v, 4) for v in world) if world else None} (expected "
                   f"(0, 5, 0)); box at {tuple(round(v, 3) for v in box) if box else None} (expected swung off x = 0), "
                   f"its end at {tuple(round(v, 3) for v in mine) if mine else None} (expected at the box)")]


def test_unpacked_turning_collider():
    """Compares the Box3D Solver SOP with RBD Bullet Solver for an unpacked collider that turns: a
    4 x 1 x 1 polygon bar along x, turned 90 degrees about y (a Transform SOP on the polygons) from
    frame 3, and a box dropped at z = 1.5, which only the turned bar is under. Each solver packs the
    unpacked bar again every frame, so the turn is inside the packed geometry, not on its transform.
    Box3D with Collision Type Animated and Deforming, Bullet with Collision Type Animated (its default) and Deforming:
    the box lands on the bar (y about 1.5) only where the solver follows the turn."""
    geo = new_geo("unpacked_turn_bar")
    bar = create(geo, "box", "bar")
    set_parm(bar, ["size"], (4.0, 1.0, 1.0))
    set_parm(bar, ["t"], (0.0, 0.5, 0.0))
    turn = create(geo, "xform", "turn")
    turn.setInput(0, bar)
    turn.parm("ry").setExpression("if($F < 3, 0, 90)", hou.exprLanguage.Hscript)
    display(turn)
    pieces, _ = packed_boxes("unpacked_turn_pieces", [{"size": (1, 1, 1), "center": (0, 4, 1.5)}])

    def height(sop):
        track = play(sop, 48, lambda s_, f: s_.geometry(0) if f == 48 else None)
        g = track[48]
        return g.boundingBox().minvec()[1] if g is not None and g.points() else None

    got = {}
    sop = solver_sop("unpacked_turn_box3d", geometry=pieces, collision=turn.path())
    got["Box3D, Collision Type Animated"] = height(sop)
    deforming = solver_sop("unpacked_turn_box3d_deforming", geometry=pieces, collision=turn.path())
    set_parm(deforming, ["collisiontype"], 2)  # Deforming
    got["Box3D, Collision Type Deforming"] = height(deforming)
    for label, kind in (("Bullet, Collision Type Animated", 1), ("Bullet, Collision Type Deforming", 2)):
        g = new_geo(f"unpacked_turn_bullet_{kind}")
        try:
            bullet = create(g, "rbdbulletsolver", "rbdbulletsolver")
        except Exception:  # noqa: BLE001 - no Bullet solver: report it
            got[label] = "no rbdbulletsolver"
            continue
        for i, path in ((0, pieces), (3, turn.path())):
            merge = create(g, "object_merge", f"input{i}")
            set_parm(merge, ["objpath1"], path)
            bullet.setInput(i, merge)
        set_parm(bullet, ["collision_initialstate"], kind)
        display(bullet)
        got[label] = height(bullet)

    def show(v):
        return f"{v:.2f}" if isinstance(v, float) else str(v)

    return [Result("unpacked_turning_collider", Result.INFO,
                   "box bottom after 2 s: " + "; ".join(f"{k}: {show(v)}" for k, v in got.items())
                   + " (about 1 = landed on the turned bar; far below 0 = the solver kept the bar along x)")]


def test_solver_sop_ground():
    """The Ground Collision menu on the Box3D Solver SOP: None lets a box fall, Ground Box (Position and
    Rotation) holds it at the plane, Height Field holds it on a flat HeightField SOP (also with Use
    Deforming Geometry), and the Physical settings reach the ground's Box3D Data."""
    pieces, names = packed_boxes("sop_ground_pieces", [{"size": (1, 1, 1), "center": (0, 4, 0)}])

    def settle(sop, frames=96):
        track = play(sop, frames, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == frames else None)
        pos = track[frames]
        return pos[1] if pos else None

    results = []
    none = solver_sop("sop_ground_none", geometry=pieces)
    y = settle(none)
    results.append(Result("solver_sop_ground", Result.PASS if y is not None and y < -1.0 else Result.FAIL,
                          f"Ground Type None: box at y={y} after 3 s (expected below -1: fell)"))

    plane = solver_sop("sop_ground_plane", geometry=pieces)
    set_parm(plane, ["useground"], 1)
    set_parm(plane, ["ground_pos"], (0, 1, 0))
    y = settle(plane)
    results.append(Result("solver_sop_ground", Result.PASS if close(y or 0, 1.5, 0.05) else Result.FAIL,
                          f"Ground Plane at y=1: box at y={y} after 3 s (expected 1.5)"))

    scaled = solver_sop("sop_ground_scaled", geometry=pieces)
    set_parm(scaled, ["useground"], 1)
    set_parm(scaled, ["ground_pos"], (0, 1, 0))
    set_parm(scaled, ["usescale"], 1)
    set_parm(scaled, ["scale"], 10)
    y = settle(scaled)
    results.append(Result("solver_sop_ground", Result.PASS if close(y or 0, 1.5, 0.05) else Result.FAIL,
                          f"Ground Plane at y=1 with World Scale 10: box at y={y} after 3 s (expected 1.5)"))

    # Size (Ground Collision): the ground reaches half its x from its position: a box over it rests, one past its
    # edge falls (also with World Scale 10, which scales the size with the scene).
    for label, scale in (("", None), (" with World Scale 10", 10)):
        edge_pieces, edge_names = packed_boxes(f"sop_ground_width_pieces{scale}", [
            {"size": (1, 1, 1), "center": (2, 4, 0)}, {"size": (1, 1, 1), "center": (8, 4, 0)}])
        edge = solver_sop(f"sop_ground_width{scale}", geometry=edge_pieces)
        set_parm(edge, ["useground"], 1)
        set_parm(edge, ["groundsize"], (10.0, 20.0, 2000.0))
        if scale:
            set_parm(edge, ["usescale"], 1)
            set_parm(edge, ["scale"], scale)
        track = play(edge, 96, lambda s, f: named_positions(s.geometry(0)) if f == 96 else None)
        near_y, far_y = (track[96][edge_names[i]][1] for i in (0, 1))
        results.append(Result("solver_sop_ground", Result.PASS if close(near_y, 0.5, 0.05) and far_y < -1.0
                              else Result.FAIL,
                              f"Ground Box Size x 10{label}: box at x=2 at y={near_y:.3f} (expected 0.5), box at x=8 at "
                              f"y={far_y:.3f} (expected below -1: past the edge at 5)"))

    # Size is in the Ground Collision folder, labelled Size, and the Ground Type's plane item is the Ground Box.
    size_parm = edge.parmTuple("groundsize")
    folders = list(size_parm.containingFolders()) if size_parm is not None else []
    menu = list(edge.parm("useground").menuLabels())
    ok = "Ground Collision" in folders and size_parm.parmTemplate().label() == "Size" and menu == [
        "None", "Ground Box", "Height Field"]
    results.append(Result("solver_sop_ground", Result.PASS if ok else Result.FAIL,
                          f"Size sits in {folders} labelled {size_parm.parmTemplate().label() if size_parm else None}; "
                          f"Ground Type items {menu} (expected Ground Collision, Size, None / Ground Box / Height Field)"))

    # Rotation: a plane turned -90 degrees about z faces +x through the origin: a box starting at x = 3 rests
    # against it with its face at x = 0.5.
    wall_pieces, wall_names = packed_boxes("sop_ground_wall_pieces", [{"size": (1, 1, 1), "center": (3, 0, 0)}])
    wall = solver_sop("sop_ground_wall", geometry=wall_pieces)
    set_parm(wall, ["useground"], 1)
    set_parm(wall, ["ground_rot"], (0, 0, -90))
    set_parm(wall, ["gravity"], (-9.81, 0, 0))
    track = play(wall, 96, lambda s, f: named_positions(s.geometry(0)).get(wall_names[0]) if f == 96 else None)
    x = track[96][0] if track[96] else None
    results.append(Result("solver_sop_ground", Result.PASS if x is not None and close(x, 0.5, 0.05) else Result.FAIL,
                          f"Ground Plane turned -90 degrees about z, gravity along -x: box at x={x} (expected 0.5)"))

    def flat_field(name):
        geo = new_geo(name)
        field = create(geo, "heightfield", "field")
        set_parm(field, ["sizex"], 20)
        set_parm(field, ["sizey"], 20)
        display(field)
        return field.path()

    for deforming in (0, 1):
        height = solver_sop(f"sop_ground_height{deforming}", geometry=pieces)
        set_parm(height, ["useground"], 2)
        set_parm(height, ["groundheightfield"], flat_field(f"sop_ground_field{deforming}"))
        set_parm(height, ["ground_animategeo"], deforming)
        y = settle(height)
        results.append(Result("solver_sop_ground", Result.PASS if close(y or 0, 0.5, 0.05) else Result.FAIL,
                              f"Height Field (Use Deforming Geometry {deforming}): box at y={y} after 3 s (expected 0.5)"))

    # Physical: the settings reach the ground's Box3D Data (friction, bounce, rolling resistance, conveyor velocity).
    phys = solver_sop("sop_ground_physical", geometry=pieces)
    values = {"ground_friction": 0.3, "ground_bounce": 0.7, "ground_rollingresistance": 0.2}
    for name, value in values.items():
        set_parm(phys, [name], value)
    set_parm(phys, ["ground_tangentvelocity"], (1, 0, 2))
    data = phys.node("dopnet/ground_data")
    got = {"friction": data.parm("friction").eval(), "bounce": data.parm("bounce").eval(),
           "rollingresistance": data.parm("rollingresistance").eval(),
           "tangentvelocity": tuple(data.parmTuple("tangentvelocity").eval())}
    ok = (close(got["friction"], 0.3, 1e-6) and close(got["bounce"], 0.7, 1e-6)
          and close(got["rollingresistance"], 0.2, 1e-6) and got["tangentvelocity"] == (1.0, 0.0, 2.0))
    results.append(Result("solver_sop_ground", Result.PASS if ok else Result.FAIL,
                          f"ground Box3D Data takes the Physical settings: {got}"))
    return results


def test_solver_sop_ground_guide():
    """The ground in the viewport (the asset's Guide Geometry, GUIDE_GEO), as RBD Bullet Solver's: Ground
    Type None draws nothing; a Ground Box is the plane and the box, closed polygons (Show Ground as Solid)
    or only lines (Wireframe, the default), and nothing with Show Ground off; a Height Field is volumes."""
    pieces, _ = packed_boxes("sop_ground_guide_pieces", [{"size": (1, 1, 1), "center": (0, 4, 0)}])

    def guide(sop):
        def sample(s, f):
            return s.node("GUIDE_GEO").geometry() if f == 3 else None

        geo = play(sop, 3, sample)[3]
        prims = geo.prims()
        kinds = {"volume": sum(1 for p in prims if p.type() == hou.primType.Volume),
                 "closed": sum(1 for p in prims if p.type() == hou.primType.Polygon and p.isClosed()),
                 "open": sum(1 for p in prims if p.type() == hou.primType.Polygon and not p.isClosed())}
        return kinds

    results = []

    def check(label, sop, want):
        got = guide(sop)
        ok = all((got[k] > 0) == (k in want) for k in got)
        results.append(Result("solver_sop_ground_guide", Result.PASS if ok else Result.FAIL,
                              f"{label}: {got} (expected {'only ' + ', '.join(sorted(want)) if want else 'nothing'})"))

    none = solver_sop("sop_ground_guide_none", geometry=pieces)
    check("Ground Type None", none, set())

    solid = solver_sop("sop_ground_guide_solid", geometry=pieces)
    set_parm(solid, ["useground"], 1)
    set_parm(solid, ["shadeground"], 0)
    check("Ground Box, Solid", solid, {"closed"})

    wire = solver_sop("sop_ground_guide_wire", geometry=pieces)
    set_parm(wire, ["useground"], 1)
    check("Ground Box, Wireframe (default)", wire, {"open"})

    hidden = solver_sop("sop_ground_guide_hidden", geometry=pieces)
    set_parm(hidden, ["useground"], 1)
    set_parm(hidden, ["showground"], 0)
    check("Ground Box, Show Ground off", hidden, set())

    field_geo = new_geo("sop_ground_guide_field")
    field = create(field_geo, "heightfield", "field")
    set_parm(field, ["sizex"], 20)
    set_parm(field, ["sizey"], 20)
    display(field)
    height = solver_sop("sop_ground_guide_height", geometry=pieces)
    set_parm(height, ["useground"], 2)
    set_parm(height, ["groundheightfield"], field.path())
    check("Height Field", height, {"volume"})
    return results


TESTS = {
    "solver_sop_collision": test_solver_sop_collision,
    "solver_sop_collision_names": test_solver_sop_collision_names,
    "solver_sop_ground": test_solver_sop_ground,
    "solver_sop_collision_static": test_solver_sop_collision_static,
    "solver_sop_ground_guide": test_solver_sop_ground_guide,
    "solver_sop_collision_prim_names": test_solver_sop_collision_prim_names,
    "solver_sop_collision_group_names": test_solver_sop_collision_group_names,
    "solver_sop_collider_group_names": test_solver_sop_collider_group_names,
    "solver_sop_collision_group_names_bullet": test_solver_sop_collision_group_names_bullet,
    "solver_sop_collision_rest": test_solver_sop_collision_rest,
    "solver_sop_world_anchor": test_solver_sop_world_anchor,
    "unpacked_turning_collider": test_unpacked_turning_collider,
    "solver_sop_animated_collision": test_solver_sop_animated_collision,
}
