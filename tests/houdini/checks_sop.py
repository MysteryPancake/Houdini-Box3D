"""The Box3D Solver SOP (tools/make_solver_sop.py): pieces falling onto a collision input, impacts,
an animated collider lifting a piece, POP forces in FORCES, a proxy driving the geometry through Transform Pieces, and the
constraint output; Box3D Configure."""

import math

import hou

from b3dtest import (
    GROUND,
    Result,
    close,
    constraint_lines,
    create,
    display,
    new_geo,
    packed_boxes,
    set_parm,
)



def solver_sop(name, geometry=None, constraints=None, proxy=None, collision=None):
    """A geo object holding a Box3D Solver SOP; each input is a SOP path (Object Merge) or None.
    Returns the solver SOP."""
    geo = new_geo(name)
    solver = create(geo, "box3d_solver", "box3d_solver")
    for i, path in enumerate((geometry, constraints, proxy, collision)):
        if path is None:
            continue
        merge = create(geo, "object_merge", f"input{i}")
        set_parm(merge, ["objpath1"], path)
        solver.setInput(i, merge)
    display(solver)
    return solver


def play(sop, frames, sample):
    """Plays frames 1..frames in order (the simulation needs every frame); sample(sop, f) per frame."""
    out = {}
    for f in range(1, frames + 1):
        hou.setFrame(f)
        sop.cook(force=False)
        out[f] = sample(sop, f)
    return out


def named_positions(geo):
    has_name = geo.findPointAttrib("name") is not None
    return {(pt.attribValue("name") if has_name else str(pt.number())): pt.position() for pt in geo.points()}


def test_solver_sop_collision():
    """A packed box dropped onto a ground box on the Collision input rests on it (y = 0.5) after 2 s;
    the proxy output equals the geometry output and carries w (Transfer to Geometry), and the
    simulation points output has one point per piece, on the geometry's piece.
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
    same = names[0] in proxy_pos and (proxy_pos[names[0]] - geo_pos[names[0]]).length() < 1e-6
    pts_ok = (len(sim_pos) == 1 and sim_prims == 0 and names[0] in sim_pos
              and (sim_pos[names[0]] - geo_pos.get(names[0], hou.Vector3(0, 99, 0))).length() < 1e-4)
    ok = close(y, 0.5, 0.03) and same and pts_ok and geo_w
    results = [Result("solver_sop_collision", Result.PASS if ok else Result.FAIL,
                      f"box at y={y:.3f} after 2 s (expected 0.5); proxy output equals geometry output: {same}; "
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


def test_solver_sop_animated_setting():
    """A solver setting driven by an expression changes the simulation while it plays (the inner solver's
    parameters are Set Always): Gravity is 0 until frame 12 and then -9.81, so a box hangs at y 5 and then falls."""
    pieces, names = packed_boxes("sop_animated_pieces", [{"size": (1, 1, 1), "center": (0, 5, 0)}])
    sop = solver_sop("sop_animated", geometry=pieces)
    sop.parmTuple("gravity")[1].setExpression("if($F < 13, 0, -9.81)", hou.exprLanguage.Hscript)
    track = play(sop, 36, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f in (12, 36) else None)
    early, late = track[12], track[36]
    ok = early is not None and late is not None and close(early[1], 5.0, 0.02) and late[1] < 3.0
    return [Result("solver_sop_animated_setting", Result.PASS if ok else Result.FAIL,
                   f"box at y={early[1] if early else None} on frame 12 (expected 5, no gravity) and "
                   f"{late[1] if late else None} on frame 36 (expected below 3, falling)")]


def test_solver_sop_constraint_settings():
    """The joint settings (Advanced, Constraints): a Weld's Torque Threshold on the node reaches the inner Weld node, and World Scale 2
    makes it four times as large there (a torque is the square of the scale). The asset has its help card."""
    sop = solver_sop("sop_constraints_tab")
    inner = sop.node("dopnet/weld")
    parm = sop.parm("weld_torque_threshold")
    if inner is None or parm is None or inner.parm("torque_threshold") is None:
        return [Result("solver_sop_constraint_settings", Result.FAIL,
                       "no weld_torque_threshold on the asset, or no inner dopnet/weld torque_threshold")]
    parm.set(5.0)
    plain = inner.parm("torque_threshold").eval()
    set_parm(sop, ["usescale"], 1)
    set_parm(sop, ["scale"], 2)
    scaled = inner.parm("torque_threshold").eval()
    ok = close(plain, 5.0, 1e-6) and close(scaled, 20.0, 1e-6)
    results = [Result("solver_sop_constraint_settings", Result.PASS if ok else Result.FAIL,
                      f"Weld Torque Threshold 5: inner {plain}, with World Scale 2 inner {scaled} (expected 5, 20)")]
    glue = [p.name() for p in sop.parms() if p.name().startswith("glue_")]
    results.append(Result("solver_sop_constraint_settings", Result.PASS if glue else Result.FAIL,
                          f"Glue settings on the Constraints tab: {glue[:6]}"))
    helped = "Help" in sop.type().definition().sections()
    results.append(Result("solver_sop_constraint_settings", Result.PASS if helped else Result.FAIL,
                          f"the asset has a Help section: {helped}"))
    return results


def test_solver_sop_impacts():
    """With Add Impact Data on (the Output tab), a box landing on the ground leaves impact points in the
    asset's IMPACTS node, and Fetch Impacts adds an Object Merge beside the asset that reads them."""
    pieces, _ = packed_boxes("sop_impact_pieces", [{"size": (1, 1, 1), "center": (0, 1, 0)}])
    ground, _ = packed_boxes("sop_impact_ground", [GROUND])
    sop = solver_sop("sop_impact", geometry=pieces, collision=ground)
    set_parm(sop, ["addimpact"], 1)
    impacts = sop.node("IMPACTS")
    counts = play(sop, 24, lambda s, f: len(impacts.geometry().points()) if impacts is not None else 0)
    most = max(counts.values())
    merge = None
    if sop.parm("fetchimpacts") is not None:
        sop.parm("fetchimpacts").pressButton()
        merge = sop.parent().node(f"{sop.name()}_impacts")
    merged = len(merge.geometry().points()) if merge is not None else None
    ok = most > 0 and merged is not None and merged == counts[24]
    return [Result("solver_sop_impacts", Result.PASS if ok else Result.FAIL,
                   f"most impact points in a frame: {most} (expected > 0); Fetch Impacts merge on frame 24: "
                   f"{merged} point(s) (expected {counts[24]})")]


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


def test_solver_sop_forces():
    """A POP Force inside the FORCES subnet (the asset's dive target and an editable node, so the
    asset stays locked) wired to PRESOLVE pushes a
    box sideways (Gravity 0): the Multiple Solver runs it before the Box3D Solver, which applies its
    `force`. Wired to POSTSOLVE instead it still pushes, a step later."""
    results = []
    for output in ("PRESOLVE", "POSTSOLVE"):
        pieces, names = packed_boxes(f"sop_force_pieces_{output}", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        sop = solver_sop(f"sop_force_{output}", geometry=pieces)
        set_parm(sop, ["gravity"], (0.0, 0.0, 0.0))
        forces = sop.node("dopnet/FORCES")  # an editable node: no need to unlock the asset
        out = forces.node(output) if forces is not None else None
        if out is None:
            results.append(Result("solver_sop_forces", Result.FAIL, f"no FORCES/{output} inside the asset"))
            continue
        pop = create(forces, "popforce", "push")
        set_parm(pop, ["force"], (5000.0, 0.0, 0.0))
        out.setInput(0, pop)
        pos = play(sop, 24, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == 24 else None)[24]
        ok = pos is not None and pos[0] > 0.01 and close(pos[1], 3.0, 1e-3)
        results.append(Result("solver_sop_forces", Result.PASS if ok else Result.FAIL,
                              f"POP Force on {output}: box at {tuple(round(c, 4) for c in pos) if pos else None} "
                              f"after 1 s (expected x above 0, y 3: pushed sideways, no gravity)"))
    return results


def piece_bounds(geo):
    """{name: packed primitive bounds (xmin, xmax, ymin, ymax, zmin, zmax)} for packed pieces named
    on their points or primitives."""
    out = {}
    on_prim = geo.findPrimAttrib("name") is not None
    for p in geo.prims():
        name = p.attribValue("name") if on_prim else p.vertices()[0].point().attribValue("name")
        out[name] = p.intrinsicValue("bounds")
    return out


def test_solver_sop_pieces():
    """Several packed pieces whose packed transforms start rotated, resting on the ground: one
    simulation point per piece, each geometry piece stays where it started (no rotation applied
    twice), and the points follow Bullet's convention (orient = rotation since rest, pivot = rest
    position). Unpacked polygon boxes with no name are packed by connectivity, simulated and fall."""
    results = []
    pieces, names = packed_boxes("sop_pieces_src", [{"size": (2, 1, 1), "center": (x, 0.5, 0)} for x in (-4, 0, 4)])
    ground, _ = packed_boxes("sop_pieces_ground", [GROUND])
    rot = new_geo("sop_pieces_rot")
    src = create(rot, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    xf = create(rot, "xform", "rotate")  # rotates every packed transform 30 degrees about Y
    xf.setInput(0, src)
    set_parm(xf, ["r"], (0, 30, 0))
    display(xf)
    before = piece_bounds(xf.geometry())
    start_p = named_positions(xf.geometry())
    sop = solver_sop("sop_pieces", geometry=xf.path(), collision=ground)

    def rest_sample(s, f):
        if f != 12:
            return None
        pts = s.geometry(3)
        has = all(pts.findPointAttrib(n) is not None for n in ("orient", "pivot", "restxform"))
        conv = {}
        if has:
            for p in pts.points():
                q = p.attribValue("orient")
                conv[p.attribValue("name")] = (abs(q[3]), hou.Vector3(p.attribValue("pivot")))
        return piece_bounds(s.geometry(0)), len(pts.points()), {p.attribValue("name") for p in pts.points()}, has, conv

    after, npts, sim_names, has_rest, conv = play(sop, 12, rest_sample)[12]
    worst = max((max(abs(a - b) for a, b in zip(before[n], after.get(n, (99,) * 6))) for n in before), default=None)
    ok = npts == 3 and sim_names == set(names) and worst is not None and worst < 0.05
    results.append(Result("solver_sop_pieces", Result.PASS if ok else Result.FAIL,
                          f"simulation points: {npts} (expected 3), names match: {sim_names == set(names)}; largest "
                          f"bounds change of a resting rotated piece after 0.5 s: {worst} (expected < 0.05)"))
    # Bullet's convention: a piece that started rotated and stays put has orient ~identity (the rotation
    # since rest) and pivot at its starting position.
    near_identity = has_rest and len(conv) == 3 and all(w > 0.9999 for w, _ in conv.values())
    pivots = has_rest and all((pv - start_p[n]).length() < 1e-4 for n, (_, pv) in conv.items() if n in start_p)
    results.append(Result("solver_sop_pieces", Result.PASS if near_identity and pivots else Result.FAIL,
                          f"orient / pivot / restxform on the simulation points: {has_rest}; orient near identity "
                          f"for rotated resting pieces: {near_identity}; pivot at the start position: {pivots}"))

    raw = new_geo("sop_pieces_unpacked")
    merge = create(raw, "merge", "merge")
    for i, x in enumerate((0, 3)):
        box = create(raw, "box", f"box{i}")
        set_parm(box, ["t"], (x, 3, 0))
        merge.setInput(i, box)
    display(merge)
    sop2 = solver_sop("sop_pieces_unpacked_solver", geometry=merge.path(), collision=ground)
    low, npts2, prims = play(sop2, 48, lambda s, f: (s.geometry(0).boundingBox().minvec()[1],
                                                     len(s.geometry(3).points()), len(s.geometry(0).prims()))
                             if f == 48 else None)[48]
    ok2 = npts2 == 2 and prims == 12 and close(low, 0.0, 0.03)
    results.append(Result("solver_sop_pieces", Result.PASS if ok2 else Result.FAIL,
                          f"unpacked input: {npts2} simulation point(s) (expected 2), geometry output {prims} "
                          f"polygons (expected 12, unpacked as given), lowest y {low:.3f} after 2 s (expected 0)"))

    # A spinning, scaled piece far from the origin: the geometry output's piece keeps its centre on its
    # simulation point and its size every frame (Transform Pieces must rotate it about its pivot and
    # keep the packed scale).
    spin_src, spin_names = packed_boxes("sop_pieces_spin_src", [{"size": (2, 1, 1), "center": (6, 3, 0),
                                                                 "attrs": {"w": (0, 4, 1)}}])
    scaled = new_geo("sop_pieces_spin_scaled")
    spin_in = create(scaled, "object_merge", "pieces")
    set_parm(spin_in, ["objpath1"], spin_src)
    sc = create(scaled, "attribwrangle", "scale_packed")  # scales the packed transform about the pivot
    sc.setInput(0, spin_in)
    set_parm(sc, ["class"], "primitive")
    set_parm(sc, ["snippet"], 'setprimintrinsic(0, "transform", @primnum, '
                              'matrix3(primintrinsic(0, "transform", @primnum)) * 1.5);')
    display(sc)

    def size(b):
        return sorted((b[1] - b[0], b[3] - b[2], b[5] - b[4]))

    start = size(piece_bounds(sc.geometry())[spin_names[0]])
    sop3 = solver_sop("sop_pieces_spin", geometry=sc.path())

    def sample(s, f):
        b = piece_bounds(s.geometry(0)).get(spin_names[0])
        pts = named_positions(s.geometry(3))
        if b is None or spin_names[0] not in pts:
            return None
        centre = hou.Vector3((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2)
        return (centre - pts[spin_names[0]]).length(), b

    track = play(sop3, 12, sample)
    gaps = [v[0] for v in track.values() if v]
    gap = max(gaps) if len(gaps) == 12 else None
    # A spinning box's bounds change, but its longest edge (3 after scaling) bounds them from below.
    end = track[12][1] if track[12] else None
    grown = end is not None and max(size(end)) >= 3.0 - 1e-3 and close(max(start), 3.0, 1e-3)
    ok3 = gap is not None and gap < 1e-3 and grown
    results.append(Result("solver_sop_pieces", Result.PASS if ok3 else Result.FAIL,
                          f"spinning scaled piece: largest gap between its centre and its simulation point {gap} "
                          f"(expected < 1e-3); scale kept: {grown}"))
    return results


def test_solver_sop_rest_anchors():
    """Pieces that carry a restxform from an earlier pose (5 to the left), as a re-simulated output
    does, and a spherical joint authored where the two pieces' shared corner is now: as RBD Bullet
    Solver, the rest is reset to the pose at creation (the old restxform is ignored), so the joint
    is made at the authored point, the hanging piece swings about it and keeps its distance to it,
    and the simulation points' pivot is the starting position, not the old one."""
    shift = 5.0
    pieces, names = packed_boxes("sop_rest_pieces", [{"size": (1, 1, 1), "center": (shift, 3, 0),
                                                      "attrs": {"active": 0}},
                                                     {"size": (1, 1, 1), "center": (shift, 2, 0),
                                                      "attrs": {"active": 1}}])
    moved = new_geo("sop_rest_moved")
    src = create(moved, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    rest = create(moved, "attribwrangle", "rest_was_left")  # a stale rest: the same pieces 5 to the left
    rest.setInput(0, src)
    set_parm(rest, ["snippet"], (
        "int prims[] = pointprims(0, @ptnum);\n"
        'matrix m = primintrinsic(0, "packedfulltransform", prims[0]);\n'
        f"translate(m, set({-shift}, 0, 0));\n"
        f"4@restxform = m; v@pivot = v@P - set({shift}, 0, 0); p@orient = {{0, 0, 0, 1}};"))
    display(rest)
    corner = (shift + 0.5, 2.5, 0.0)
    lines = constraint_lines("sop_rest_network", [(names[0], corner, names[1], corner, "Spherical")])
    sop = solver_sop("sop_rest_anchors", geometry=rest.path(), constraints=lines)
    corner_v = hou.Vector3(corner)
    start = (hou.Vector3(shift, 2, 0) - corner_v).length()

    def sample(s, f):
        if f not in (1, 12, 24):
            return None
        pts = s.geometry(3)
        pos = named_positions(pts).get(names[1])
        pivot = None
        if pts.findPointAttrib("pivot") is not None and pts.findPointAttrib("name") is not None:
            pivot = next((hou.Vector3(p.attribValue("pivot")) for p in pts.points()
                          if p.attribValue("name") == names[1]), None)
        return pos, pivot

    track = play(sop, 24, sample)
    gaps = [abs((track[f][0] - corner_v).length() - start) for f in (12, 24) if track[f] and track[f][0]]
    swung = track[24] is not None and track[24][0] is not None and abs(track[24][0][0] - shift) > 0.05
    pivot = track[24][1] if track[24] else None
    pivot_ok = pivot is not None and (pivot - hou.Vector3(shift, 2, 0)).length() < 1e-3
    ok = len(gaps) == 2 and max(gaps) < 0.03 and swung and pivot_ok
    return [Result("solver_sop_rest_anchors", Result.PASS if ok else Result.FAIL,
                   f"hanging piece's distance to the joint corner {corner} changed by "
                   f"{max(gaps) if gaps else None} (expected < 0.03, start {start:.3f}); it swung: {swung}; "
                   f"its pivot {tuple(round(v, 3) for v in pivot) if pivot else None} (expected the start "
                   f"({shift}, 2, 0), not the stale ({shift - 5}, 2, 0))")]


def test_solver_sop_initial_bake():
    """As RBD Bullet Solver (RBD Packed Object on the creation frame): a packed piece's orient, pscale
    and scale are baked into its packed transform, with the rest taken before the bake. A 2 x 1 x 1
    box with orient 90 degrees about y and pscale 0.5: the simulated piece is 0.5 wide in x and 1
    deep in z; its simulation point's orient is that 90 degree turn (the rotation since rest), so the
    Geometry output, Transform Pieces of the unchanged input, is 1 wide and 2 deep (turned, not
    scaled, as in Bullet)."""
    pieces, names = packed_boxes("sop_bake_pieces", [{"size": (2, 1, 1), "center": (0, 3, 0)}])
    geo = new_geo("sop_bake_in")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    attrs = create(geo, "attribwrangle", "orient_pscale")
    attrs.setInput(0, src)
    set_parm(attrs, ["snippet"], "p@orient = quaternion(radians(90), {0, 1, 0}); f@pscale = 0.5;")
    display(attrs)
    sop = solver_sop("sop_initial_bake", geometry=attrs.path())

    def extents(b):
        return (round(b[1] - b[0], 4), round(b[5] - b[4], 4)) if b else None

    def sample(s, f):
        if f != 2:
            return None
        out = extents(piece_bounds(s.geometry(0)).get(names[0]))
        pts = s.geometry(3)
        q = pts.points()[0].attribValue("orient") if pts.points() and pts.findPointAttrib("orient") else None
        obj = s.node("dopnet").simulation().findObject("pieces") if s.node("dopnet") else None
        g = obj.geometry() if obj is not None else None
        sim = extents(g.prims()[0].intrinsicValue("bounds")) if g is not None and g.prims() else None
        return out, q, sim

    out, q, sim = play(sop, 2, sample)[2]
    turned = q is not None and close(abs(q[1]), 0.7071, 1e-3) and close(abs(q[3]), 0.7071, 1e-3)
    ok = (sim is not None and close(sim[0], 0.5, 1e-3) and close(sim[1], 1.0, 1e-3) and turned
          and out is not None and close(out[0], 1.0, 1e-3) and close(out[1], 2.0, 1e-3))
    return [Result("solver_sop_initial_bake", Result.PASS if ok else Result.FAIL,
                   f"simulated piece (x, z) extent {sim} (expected (0.5, 1): turned and scaled); simulation point "
                   f"orient {tuple(round(v, 4) for v in q) if q else None} (expected 90 degrees about y); Geometry "
                   f"output {out} (expected (1, 2): turned only)")]


def test_solver_sop_removed_pieces():
    """As RBD Bullet Solver (remove_deleted): a piece the solver deletes (dead, Reap Dead Points on)
    leaves the Geometry and Proxy outputs instead of staying behind untransformed."""
    pieces, names = packed_boxes("sop_removed_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"dead": 0}},
                                                        {"size": (1, 1, 1), "center": (3, 3, 0), "attrs": {"dead": 1}}])
    sop = solver_sop("sop_removed_pieces", geometry=pieces)
    set_parm(sop, ["reapdead"], 1)
    track = play(sop, 6, lambda s, f: (len(s.geometry(0).prims()), len(s.geometry(2).prims()),
                                       len(s.geometry(3).points())) if f == 6 else None)
    geo_n, proxy_n, pts_n = track[6]
    ok = (geo_n, proxy_n, pts_n) == (1, 1, 1)
    return [Result("solver_sop_removed_pieces", Result.PASS if ok else Result.FAIL,
                   f"on frame 6: {geo_n} geometry piece(s), {proxy_n} proxy piece(s), {pts_n} simulation point(s) "
                   f"(expected 1, 1, 1: the dead piece {names[1]} gone from all three)")]


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


def test_solver_sop_collision_group_names():
    """Bullet's collisiongroup / collisionignore on the pieces become collision bits in the Solver SOP: a box dropped
    onto another rests on it (y = 1.5) unless the lower one ignores the upper one's group, or the upper one ignores
    the lower one's (either side is enough), and then it falls through to the ground (y = 0.5). A pattern that
    matches no group, and a `^` that takes the group out again, change nothing."""
    cases = [
        # (tag, lower (group, ignore), upper (group, ignore), expected y of the upper box)
        ("plain", ("debris", ""), ("debris", ""), 1.5),
        ("lower_ignores_group", ("debris", "debris"), ("debris", ""), 0.5),
        ("upper_ignores_other_group", ("a", ""), ("b", "a"), 0.5),
        ("lower_ignores_upper_group", ("a", "b"), ("b", ""), 0.5),
        ("other_group_ignored", ("a", "zzz"), ("b", ""), 1.5),
        ("pattern", ("a", "d*"), ("debris", ""), 0.5),
        ("pattern_taken_out", ("a", "d* ^debris"), ("debris", ""), 1.5),
    ]
    ground, _ = packed_boxes("sop_cgn_ground", [GROUND])
    results = []
    for tag, (g0, i0), (g1, i1), want in cases:
        pieces, names = packed_boxes(f"sop_cgn_pieces_{tag}", [
            {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"collisiongroup": g0, "collisionignore": i0}},
            {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"collisiongroup": g1, "collisionignore": i1}}])
        sop = solver_sop(f"sop_cgn_{tag}", geometry=pieces, collision=ground)
        pos = play(sop, 48, lambda s, f: named_positions(s.geometry(0)) if f == 48 else None)[48]
        y = pos[names[1]][1] if names[1] in pos else None
        ok = y is not None and close(y, want, 0.05)
        results.append(Result("solver_sop_collision_group_names", Result.PASS if ok else Result.FAIL,
                              f"{tag}: lower {g0!r} ignoring {i0!r}, upper {g1!r} ignoring {i1!r}: upper box at "
                              f"y={y} after 2 s (expected {want})"))
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


def test_solver_sop_world_scale():
    """World Scale (RBD Bullet Solver's Bullet World Scale) leaves results in scene units: a box dropped
    from y = 3 is at the same height after 0.5 s with World Scale 1, 10 and 0.1 (free fall: gravity
    follows the scale), and rests on the ground at y = 0.5 after 3 s; the simulation points come back
    unscaled."""
    pieces, names = packed_boxes("sop_scale_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    ground, _ = packed_boxes("sop_scale_ground", [GROUND])
    got = {}
    for k in (1.0, 10.0, 0.1):
        sop = solver_sop(f"sop_world_scale_{str(k).replace('.', '_')}", geometry=pieces, collision=ground)
        set_parm(sop, ["usescale"], 1 if k != 1.0 else 0)
        set_parm(sop, ["scale"], k)

        def sample(s, f):
            if f not in (12, 72):
                return None
            return named_positions(s.geometry(3)).get(names[0])

        track = play(sop, 72, sample)
        got[k] = (track[12][1] if track[12] else None, track[72][1] if track[72] else None)
    base = got[1.0][0]
    falls = [got[k][0] for k in (10.0, 0.1)]
    rests = [got[k][1] for k in got]
    ok = base is not None and all(f is not None and abs(f - base) < 1e-3 for f in falls) and \
        all(r is not None and abs(r - 0.5) < 0.05 for r in rests)
    return [Result("solver_sop_world_scale", Result.PASS if ok else Result.FAIL,
                   "height after 0.5 s / 3 s: " + ", ".join(
                       f"World Scale {k}: {got[k][0]:.4f} / {got[k][1]:.3f}" if None not in got[k] else f"{k}: None"
                       for k in got) + " (expected the same fall in each, resting at 0.5)")]


def test_solver_sop_ground():
    """The Ground Collision menu on the Box3D Solver SOP: None lets a box fall, Ground Plane (Position and
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
    Type None draws nothing; a Ground Plane is lines (Show Ground as Wireframe) or closed polygons
    (Shaded), and nothing with Show Ground off; a Height Field is volumes."""
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

    wire = solver_sop("sop_ground_guide_wire", geometry=pieces)
    set_parm(wire, ["useground"], 1)
    check("Ground Plane, Wireframe", wire, {"open"})

    shaded = solver_sop("sop_ground_guide_shaded", geometry=pieces)
    set_parm(shaded, ["useground"], 1)
    set_parm(shaded, ["shadeground"], 1)
    check("Ground Plane, Shaded", shaded, {"closed"})

    hidden = solver_sop("sop_ground_guide_hidden", geometry=pieces)
    set_parm(hidden, ["useground"], 1)
    set_parm(hidden, ["showground"], 0)
    check("Ground Plane, Show Ground off", hidden, set())

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


def test_solver_sop_before_start():
    """Before the Start Frame the inputs pass through, as RBD Bullet Solver: with Start Frame 6, a
    box at y = 3 is still at its input position on frame 3 (geometry output) and has fallen by frame 12."""
    pieces, names = packed_boxes("sop_before_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    sop = solver_sop("sop_before_start", geometry=pieces)
    set_parm(sop, ["startframe"], 6)
    track = play(sop, 12, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f in (3, 12) else None)
    early, late = track[3], track[12]
    ok = early is not None and close(early[1], 3.0, 1e-4) and late is not None and late[1] < 2.9
    return [Result("solver_sop_before_start", Result.PASS if ok else Result.FAIL,
                   f"box y on frame 3 (before Start Frame 6): {early[1] if early else None} (expected 3, the "
                   f"input); on frame 12: {late[1] if late else None} (expected below 2.9: fell)")]


def test_solver_sop_solve_first_frame():
    """Solve on Creation Frame (Solver tab), as RBD Bullet Solver: off, a falling box is still at its
    start on the start frame; on, it has already moved down on that frame."""
    pieces, names = packed_boxes("sop_first_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    ys = {}
    for on in (0, 1):
        sop = solver_sop(f"sop_solve_first_{on}", geometry=pieces)
        set_parm(sop, ["solvefirstframe"], on)
        pos = play(sop, 1, lambda s, f: named_positions(s.geometry(0)).get(names[0]))[1]
        ys[on] = pos[1] if pos else None
    ok = ys[0] is not None and ys[1] is not None and close(ys[0], 3.0, 1e-5) and ys[1] < 3.0 - 1e-4
    return [Result("solver_sop_solve_first_frame", Result.PASS if ok else Result.FAIL,
                   f"box y on the start frame: off {ys[0]} (expected 3), on {ys[1]} (expected below 3)")]


def test_solver_sop_keep_broken():
    """Export Broken Constraints, as RBD Bullet Solver: a glued pair (SideFX Glue far too weak to hold
    a landing) dropped onto the ground breaks on the landing (Glue breaks on hit impulse, so a pair that never hits
    anything holds); off, the constraints output drops it; on, it keeps it in the broken group."""
    pieces, names = packed_boxes("sop_broken_pieces", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
                                                       {"size": (1, 1, 1), "center": (0, 1, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("sop_broken_ground", [GROUND])
    lines = constraint_lines("sop_broken_network",
                             [(names[0], (0, 1.5, 0), names[1], (0, 1.5, 0), "Glue", {"strength": 1e-6})])
    results = []
    for keep in (0, 1):
        sop = solver_sop(f"sop_keep_broken_{keep}", geometry=pieces, constraints=lines, collision=ground)
        set_parm(sop, ["constraint_keepbroken"], keep)

        def sample(s, f):
            if f != 12:
                return None
            con = s.geometry(1)
            grp = con.findPrimGroup("broken")
            return len(con.prims()), len(grp.prims()) if grp else 0

        nprims, nbroken = play(sop, 12, sample)[12]
        ok = (nprims, nbroken) == ((1, 1) if keep else (0, 0))
        results.append(Result("solver_sop_keep_broken", Result.PASS if ok else Result.FAIL,
                              f"Export Broken Constraints {keep}: {nprims} constraint(s), {nbroken} broken on frame "
                              f"12 (expected {'1, 1' if keep else '0, 0'})"))
    return results


def bullet_sop(name, geometry=None, constraints=None, proxy=None, collision=None):
    """solver_sop's scene with the RBD Bullet Solver SOP (rbdbulletsolver) in place of ours: same inputs,
    same order. SetupError if the node does not exist."""
    geo = new_geo(name)
    solver = create(geo, "rbdbulletsolver", "rbdbulletsolver")
    for i, path in enumerate((geometry, constraints, proxy, collision)):
        if path is None:
            continue
        merge = create(geo, "object_merge", f"input{i}")
        set_parm(merge, ["objpath1"], path)
        solver.setInput(i, merge)
    display(solver)
    return solver


def _both_sops(name, **inputs):
    """{"box3d": our Solver SOP, "bullet": RBD Bullet Solver SOP} on the same inputs."""
    return {"box3d": solver_sop(f"{name}_box3d", **inputs), "bullet": bullet_sop(f"{name}_bullet", **inputs)}


def _counts(sop):
    """(points, primitives) of each of the SOP's four outputs: None for an output it does not have."""
    out = []
    for i in range(4):
        try:
            geo = sop.geometry(i)
        except hou.Error:
            geo = None
        out.append((len(geo.points()), len(geo.prims())) if geo is not None else None)
    return out


def test_solver_sop_before_start_bullet():
    """Before the Start Frame, against the RBD Bullet Solver SOP: the same pieces, a constraint line and a
    collision ground, Start Frame 6. On frame 3 the box is still at its input height in both, and every
    output (geometry, constraints, proxy, simulation points) has the same number of points and
    primitives (not the proxy output, which Box3D fills from the geometry and Bullet leaves empty with no proxy
    input); on frame 12 both have dropped it."""
    pieces, names = packed_boxes("sop_before_b_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"active": 1}},
                                                         {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("sop_before_b_ground", [GROUND])
    lines = constraint_lines("sop_before_b_network", [(names[0], (0, 3.5, 0), names[1], (0, 3.5, 0), "Glue")])
    sops = _both_sops("sop_before_b", geometry=pieces, constraints=lines, collision=ground)
    got = {}
    for kind, sop in sops.items():
        if kind == "box3d":
            set_parm(sop, ["startframe"], 6)
        else:
            set_parm(sop, ["startframe"], 6)
        track = play(sop, 12, lambda s, f: (named_positions(s.geometry(0)).get(names[0]), _counts(s))
                     if f in (3, 12) else None)
        got[kind] = track
    early = {k: got[k][3][0] for k in got}
    late = {k: got[k][12][0] for k in got}
    # The proxy output (index 2) is left out: with no proxy input Box3D passes the geometry through it and
    # Bullet gives nothing.
    keep = (0, 1, 3)
    same_counts = all(got["box3d"][3][1][i] == got["bullet"][3][1][i] for i in keep)
    ok = (all(e is not None and close(e[1], 3.0, 1e-4) for e in early.values())
          and all(l is not None and l[1] < 2.9 for l in late.values()) and same_counts)
    return [Result("solver_sop_before_start_bullet", Result.PASS if ok else Result.FAIL,
                   f"frame 3 (before Start Frame 6): box y {[early[k][1] if early[k] else None for k in got]}, "
                   f"output (points, prims) Box3D {got['box3d'][3][1]}, Bullet {got['bullet'][3][1]} (the proxy, third, is "
                   f"not compared); frame 12 box y "
                   f"{[late[k][1] if late[k] else None for k in got]} (kinds: {list(got)})")]


def test_solver_sop_solve_first_frame_bullet():
    """Solve on Creation Frame, against the RBD Bullet Solver SOP, off and on: a free-falling box must be at the
    same height in both on frames 1 to 3 (within 0.05). A box resting on a box that the Collision input lifts
    0.5 a frame (Collision Type Animated) is INFO beside it: Bullet's stayed at its start, so either its
    collider did not move or the option does not mean the same. Box3D's box clipping into the collider is a known
    Box3D bug, so the case is not a pass mark."""
    pieces, names = packed_boxes("sop_first_b_pieces", [
        {"size": (1, 1, 1), "center": (0, 1.0, 0), "attrs": {"active": 1}},
        {"size": (1, 1, 1), "center": (5, 6.0, 0), "attrs": {"active": 1}}])
    geo = new_geo("sop_first_b_lift")
    lift = create(geo, "box", "lift")
    set_parm(lift, ["size"], (3.0, 1.0, 3.0))
    move = create(geo, "xform", "move")
    move.setInput(0, lift)
    move.parm("ty").setExpression("($F - 1) * 0.5", hou.exprLanguage.Hscript)
    display(move)
    results = []
    for on in (0, 1):
        sops = _both_sops(f"sop_first_b_{on}", geometry=pieces, collision=move.path())
        set_parm(sops["box3d"], ["collisiontype"], 1)
        set_parm(sops["bullet"], ["collision_initialstate"], 1)
        set_parm(sops["box3d"], ["solvefirstframe"], on)
        set_parm(sops["bullet"], ["solvefirstframe"], on)
        ys = {kind: play(sop, 3, lambda s, f: named_positions(s.geometry(0))) for kind, sop in sops.items()}
        for label, name, status in (("free-falling box", names[1], None), ("box on the lifted collider", names[0], Result.INFO)):
            ok = all(name in ys[k][f] for k in ys for f in (1, 2, 3)) and                 all(close(ys["box3d"][f][name][1], ys["bullet"][f][name][1], 0.05) for f in (1, 2, 3))
            show = lambda k: [round(ys[k][f][name][1], 3) if name in ys[k][f] else None for f in (1, 2, 3)]  # noqa: E731
            results.append(Result("solver_sop_solve_first_frame_bullet",
                                  status or (Result.PASS if ok else Result.FAIL),
                                  f"Solve on Creation Frame {on}, {label}: y on frames 1 to 3, Box3D {show('box3d')}, "
                                  f"Bullet {show('bullet')}"))
    return results


def test_solver_sop_keep_broken_bullet():
    """Export Broken Constraints, against the RBD Bullet Solver SOP, on the scene of solver_sop_keep_broken (a
    glued pair dropped on the ground, Glue far too weak to hold the landing): with it off and on, the number of
    constraint primitives and of those in the `broken` group on frame 12 are the same in both."""
    pieces, names = packed_boxes("sop_broken_b_pieces", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
                                                         {"size": (1, 1, 1), "center": (0, 1, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("sop_broken_b_ground", [GROUND])
    lines = constraint_lines("sop_broken_b_network",
                             [(names[0], (0, 1.5, 0), names[1], (0, 1.5, 0), "Glue", {"strength": 1e-6})])
    results = []
    for keep in (0, 1):
        sops = _both_sops(f"sop_keep_broken_b{keep}", geometry=pieces, constraints=lines, collision=ground)
        set_parm(sops["box3d"], ["constraint_keepbroken"], keep)
        set_parm(sops["bullet"], ["constraint_keepbroken"], keep)

        def sample(s, f):
            if f != 12:
                return None
            con = s.geometry(1)
            grp = con.findPrimGroup("broken")
            return len(con.prims()), len(grp.prims()) if grp else 0

        got = {kind: play(sop, 12, sample)[12] for kind, sop in sops.items()}
        results.append(Result("solver_sop_keep_broken_bullet", Result.PASS if got["box3d"] == got["bullet"] else Result.FAIL,
                              f"Export Broken Constraints {keep}: (constraints, broken) on frame 12 Box3D "
                              f"{got['box3d']}, Bullet {got['bullet']}"))
    return results


def _break_impact_held(kind, threshold, tag, strength=1e12, scale_by_strength=False, from_frame=None):
    """Whether a SideFX Glue pair (Strength 1e12: it cannot break by itself) landing from 3 holds with an Impact
    Threshold of `threshold` on the Break Thresholds of the Box3D Solver SOP ("box3d") or the RBD Bullet Solver SOP
    ("bullet"): the constraint primitives left on frame 48. `strength`: the Glue's own (a line attribute);
    `scale_by_strength`: Scale by Attribute, the attribute `strength`; `from_frame`: the From Frame."""
    parms = {"constraint_impactthreshold1": threshold}
    if from_frame is not None:
        parms.update({"constraint_usefromframe1": 1, "constraint_fromframe1": from_frame})
    if scale_by_strength:
        parms.update({"constraint_scaleimpact1": 1, "constraint_impactattrib1": "strength"})
    return _first_break(kind, tag, "Glue", ((0.5, 3, 0), (0.5, 3, 0)), hang={}, enable=("impact",), parms=parms,
                        line_attrs={"strength": strength}, frames=48, centers=((0, 3, 0), (1, 3, 0)), fixed=False,
                        ground=True) is None


def test_solver_sop_break_impact_bullet():
    """Break Thresholds, Impact Threshold (the Constraints tab), against the RBD Bullet Solver SOP: a glued pair
    landing from 3 breaks with a threshold of 5000 and holds with 2e6 in both; the threshold at which it stops
    breaking is found by bisection in each and must agree within a factor of 2. A threshold scaled by the
    `strength` attribute (Scale by Attribute) breaks a pair of Strength 1e7 (too strong to break itself) with a
    threshold of 0.001 and not with one of 0.1, in both. From Frame 100 holds the pair through frame 48, in both."""
    runs = iter(range(1000))
    tag = lambda: f"sop_break_{next(runs)}"  # noqa: E731
    problems, found = [], {}
    for kind in ("box3d", "bullet"):
        if _break_impact_held(kind, 5000.0, tag()):
            problems.append(f"{kind} held with a threshold of 5000")
        if not _break_impact_held(kind, 2e6, tag()):
            problems.append(f"{kind} broke with a threshold of 2e6")
        lo, hi = 5000.0, 2e6  # lo breaks it, hi holds it
        for _ in range(8):
            mid = (lo * hi) ** 0.5
            if _break_impact_held(kind, mid, tag()):
                hi = mid
            else:
                lo = mid
        found[kind] = hi
    ratio = found["bullet"] / found["box3d"]
    if not 0.5 <= ratio <= 2.0:
        problems.append(f"the thresholds differ by {ratio:.2f}x")
    for kind in ("box3d", "bullet"):  # From Frame 100: nothing breaks within the 48 frames played
        if not _break_impact_held(kind, 5000.0, tag(), from_frame=100):
            problems.append(f"{kind} broke before From Frame 100")
    for kind in ("box3d", "bullet"):  # Scale by Attribute: threshold x strength (1e7, too strong to break by itself)
        if _break_impact_held(kind, 0.001, tag(), strength=1e7, scale_by_strength=True):
            problems.append(f"{kind} held with a threshold of 0.001 x strength 1e7")
        if not _break_impact_held(kind, 0.1, tag(), strength=1e7, scale_by_strength=True):
            problems.append(f"{kind} broke with a threshold of 0.1 x strength 1e7")
    return [Result("solver_sop_break_impact_bullet", Result.FAIL if problems else Result.PASS,
                   f"pair stops breaking at an Impact Threshold of {found['box3d']:.0f} (Box3D) and "
                   f"{found['bullet']:.0f} (Bullet), Bullet / Box3D {ratio:.2f}" +
                   (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def _first_break(kind, tag, cname, anchors, hang=None, enable=(), parms=None, names="Glue", line_attrs=None, frames=24,
                 centers=((0, 4, 0), (0, 2, 0)), fixed=True, ground=False):
    """The first frame on which a two-piece scene (a fixed box over a free one, 2 apart) has no constraint left on
    the Break Thresholds of the Box3D Solver SOP ("box3d") or the RBD Bullet Solver SOP ("bullet"), None if it
    holds for `frames`. `anchors`: the two line points; `hang`: the free box's point attributes; `enable`: the
    thresholds to turn on (`parms` sets their values and the rest); every other one is off. `centers`: the boxes'
    centres; `fixed`: the first box is fixed; `ground`: a ground plane under them."""
    pieces, pnames = packed_boxes(f"{tag}_pieces", [{"size": (1, 1, 1), "center": centers[0], "attrs": {"active": 0 if fixed else 1}},
                                                    {"size": (1, 1, 1), "center": centers[1],
                                                     "attrs": dict(hang or {}, active=1)}])
    collision = packed_boxes(f"{tag}_ground", [GROUND])[0] if ground else None
    lines = constraint_lines(f"{tag}_network", [(pnames[0], anchors[0], pnames[1], anchors[1], cname, line_attrs or {})])
    inputs = {"collision": collision} if ground else {}
    sop = (solver_sop if kind == "box3d" else bullet_sop)(f"{tag}_{kind}", geometry=pieces, constraints=lines,
                                                          **inputs)
    set_parm(sop, ["enable_constraintbreaks"], 1)
    set_parm(sop, ["constraint_names1"], names)
    for key in ("angle", "distance", "impact", "torque"):
        set_parm(sop, [f"constraint_use{key}1"], 1 if key in enable else 0)
    for name, value in (parms or {}).items():
        set_parm(sop, [name], value)
    gone = []
    play(sop, frames, lambda s, f: None if s.geometry(1).prims() else gone.append(f))
    for node in [n for n in hou.node("/obj").children() if n.name().startswith(tag)]:
        node.destroy()
    return gone[0] if gone else None


def test_solver_sop_break_atframe_bullet():
    """Break Thresholds, At Frame, against the RBD Bullet Solver SOP: a Glue too strong to break (Strength 1e12) with no
    threshold on goes at Frame 5 in both: the frame it is gone on differs by at most 1. Relative to Start Frame
    with a Start Frame of 3 moves that to 8. Nothing breaks before."""
    results, problems = [], []
    got = {}
    for kind in ("box3d", "bullet"):
        for rel, at in ((0, 5), (1, 5)):
            parms = {"constraint_useatframe1": 1, "constraint_atframe1": at, "constraint_relatframe1": rel}
            if rel:
                parms["startframe"] = 3
            got[kind, rel] = _first_break(kind, f"sop_atframe_{kind}{rel}", "Glue", ((0, 3.5, 0), (0, 3.5, 0)),
                                          parms=parms, line_attrs={"strength": 1e12})
    for rel, want in ((0, 5), (1, 8)):
        for kind in ("box3d", "bullet"):
            f = got[kind, rel]
            if f is None or not want <= f <= want + 1:
                problems.append(f"{kind} (relative {rel}) went on frame {f}, expected {want}")
    return [Result("solver_sop_break_atframe_bullet", Result.FAIL if problems else Result.PASS,
                   f"gone on frame (Box3D / Bullet): At Frame 5 {got['box3d', 0]} / {got['bullet', 0]}, relative to "
                   f"Start Frame 3 {got['box3d', 1]} / {got['bullet', 1]}" +
                   (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def _break_to_next(tag, cname, names, line_attrs, anchors, frames):
    """A fixed box over a free one joined by a `cname` line with `line_attrs` (a next constraint among them) on the Box3D
    Solver SOP, Break Thresholds At Frame 5 in Switch to Next Constraint mode: (the line's attributes, the free box's
    position) on the last of `frames`."""
    pieces, pnames = packed_boxes(f"{tag}_pieces", [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
                                                    {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}])
    lines = constraint_lines(f"{tag}_network", [(pnames[0], anchors[0], pnames[1], anchors[1], cname, line_attrs)])
    sop = solver_sop(f"{tag}_sop", geometry=pieces, constraints=lines)
    set_parm(sop, ["enable_constraintbreaks"], 1)
    set_parm(sop, ["constraint_names1"], names)
    for key in ("angle", "distance", "impact", "torque"):
        set_parm(sop, [f"constraint_use{key}1"], 0)
    for name, value in {"constraint_useatframe1": 1, "constraint_atframe1": 5, "constraint_mode1": 1}.items():
        set_parm(sop, [name], value)

    def sample(s, f):
        if f != frames:
            return None
        con = s.geometry(1)
        prim = con.prims()[0] if con.prims() else None
        attrs = {a.name(): prim.attribValue(a.name()) for a in con.primAttribs()} if prim is not None else {}
        return attrs, named_positions(s.geometry(0)).get(pnames[1])

    return play(sop, frames, sample)[frames]


def test_solver_sop_break_next_settings():
    """The Solver SOP's break wrangle (Break Thresholds, At Frame 5, Switch to Next Constraint) gives a switching
    constraint its next settings: a Weld with next_constraint_name Spherical, next_enable_cone_limit 1 and
    next_cone_angle 0.25 is a Spherical line with enable_cone_limit 1 and cone_angle 0.25 once it has switched, and
    the next attributes are "not set" again."""
    attrs, _ = _break_to_next("sop_next_settings", "Weld", "Weld",
                              {"next_constraint_name": "Spherical", "next_enable_cone_limit": 1,
                               "next_cone_angle": 0.25}, ((0, 3.5, 0), (0, 3.5, 0)), 8)
    ok = (attrs.get("constraint_name") == "Spherical" and attrs.get("enable_cone_limit") == 1
          and close(attrs.get("cone_angle", -1.0), 0.25, 1e-4) and attrs.get("next_enable_cone_limit", 0) < 0
          and attrs.get("next_cone_angle", 0.0) <= -3.0e38 and not attrs.get("next_constraint_name"))
    return [Result("solver_sop_break_next_settings", Result.PASS if ok else Result.FAIL,
                   f"line attributes on frame 8: { {k: attrs.get(k) for k in ('constraint_name', 'enable_cone_limit', 'cone_angle', 'next_enable_cone_limit', 'next_cone_angle', 'next_constraint_name')} } "
                   f"(expected Spherical, 1, 0.25 and the next ones cleared)")]


def test_solver_sop_next_constraint_type():
    """next_constraint_type reaches a SideFX-typed line. A SideFX Hard line holding a box by an anchor 1 beside its centre
    (constraint_type all: rigid) breaks at frame 5 and switches to Hard again. With next_constraint_type position it
    is made as a position-only joint, so the box swings about the anchor and ends further along x (1.7 against 1.0);
    without it (constraint_type empty again: all) it stays rigid. The line has constraint_type position afterwards."""
    anchors = ((1, 3.5, 0), (1, 3.5, 0))
    base = {"constraint_type": "all", "next_constraint_name": "Hard"}
    swung_attrs, swung = _break_to_next("sop_next_type_a", "Hard", "Hard", dict(base, next_constraint_type="position"),
                                        anchors, 16)
    held_attrs, held = _break_to_next("sop_next_type_b", "Hard", "Hard", base, anchors, 16)
    ok = (swung is not None and held is not None and swung[0] - held[0] > 0.3
          and swung_attrs.get("constraint_type") == "position" and not swung_attrs.get("next_constraint_type"))
    return [Result("solver_sop_next_constraint_type", Result.PASS if ok else Result.FAIL,
                   f"box centre x on frame 16 with next_constraint_type position {swung[0] if swung else None} "
                   f"(expected more than 0.3 above the held one: swung), without {held[0] if held else None} (held); "
                   f"constraint_type after the switch {swung_attrs.get('constraint_type')!r}, "
                   f"next_constraint_type {swung_attrs.get('next_constraint_type')!r}")]


def test_solver_sop_break_distance_bullet():
    """Break Thresholds, Distance Threshold, against the RBD Bullet Solver SOP: a Hard constraint whose anchors start
    2 apart breaks within 8 frames with a threshold of 0.2 and holds for 8 frames with one of 50, in both: the
    threshold compares the anchors' distance, beyond the constraint's Rest Length."""
    problems, rows = [], []
    for kind in ("box3d", "bullet"):
        for thr, want_break in ((0.2, True), (50.0, False)):
            f = _first_break(kind, f"sop_distance_{kind}{int(thr * 10)}", "Hard", ((0, 3.5, 0), (0, 1.5, 0)),
                             enable=("distance",), parms={"constraint_distancethreshold1": thr}, names="Hard", frames=8)
            rows.append(f"{kind} {thr}: {f}")
            if (f is not None) != want_break:
                problems.append(f"{kind} with a threshold of {thr}: gone on {f}")
    return [Result("solver_sop_break_distance_bullet", Result.FAIL if problems else Result.PASS,
                   f"gone on frame: {', '.join(rows)}" + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_solver_sop_break_angle_torque_bullet():
    """INFO, Break Thresholds, Angle and Torque Threshold, against the RBD Bullet Solver SOP: a Soft constraint on a
    box spinning at 6 rad/s about z, thresholds from small to large. The frame it goes on in each is listed. They
    are not expected to agree: the two solvers' `angle` and `torque` readings differ (docs/notes.md)."""
    rows = []
    for key, values in (("angle", (0.01, 0.05, 0.2)), ("torque", (100.0, 2000.0, 20000.0))):
        for value in values:
            got = {}
            for kind in ("box3d", "bullet"):
                got[kind] = _first_break(kind, f"sop_{key}_{kind}{int(value * 100)}", "Soft",
                                         ((0, 3.5, 0), (0, 3.5, 0)), hang={"w": (0, 0, 6)}, enable=(key,),
                                         parms={f"constraint_{key}threshold1": value}, names="Soft", frames=16)
            rows.append(f"{key} {value}: {got['box3d']} / {got['bullet']}")
    return [Result("solver_sop_break_angle_torque_bullet", Result.INFO,
                   "gone on frame (Box3D / Bullet): " + "; ".join(rows))]


def test_solver_sop_global_substeps():
    """Global Substeps on the Solver tab, just above Box3D Substeps: a slider from 1 with no upper limit, default 1,
    and it sets the Substeps of the DOP network inside. Two global substeps give nearly the same height after
    0.5 s as one (within 0.05), and values above 10 can be set."""
    sop = solver_sop("sop_global_substeps")
    names = [pt.name() for pt in sop.parmTemplateGroup().entriesWithoutFolders()]
    pt = sop.parmTemplateGroup().find("globalsubsteps")
    at = names.index("globalsubsteps") if "globalsubsteps" in names else -1
    placed = at >= 0 and names[at + 1] == "substeps" and sop.parm("substeps").parmTemplate().label() == "Box3D Substeps"
    rng = pt is not None and (pt.minValue(), pt.maxValue(), pt.minIsStrict(), pt.maxIsStrict(), pt.defaultValue()) ==         (1, 10, True, False, (1,))
    sop.parm("globalsubsteps").set(14)
    linked = sop.node("dopnet").parm("substep").eval() == 14
    sop.parm("globalsubsteps").set(1)
    pieces, pnames = packed_boxes("sop_global_pieces", [{"size": (1, 1, 1), "center": (0, 6, 0)}])
    ys = {}
    for n in (1, 2):
        run = solver_sop(f"sop_global_run{n}", geometry=pieces)
        set_parm(run, ["globalsubsteps"], n)
        pos = play(run, 12, lambda s, f: named_positions(s.geometry(0)).get(pnames[0]) if f == 12 else None)[12]
        ys[n] = pos[1] if pos else None
    ok = placed and rng and linked and ys[1] is not None and ys[2] is not None and close(ys[1], ys[2], 0.05)
    return [Result("solver_sop_global_substeps", Result.PASS if ok else Result.FAIL,
                   f"just above Box3D Substeps: {placed}; range 1 to 10, only the minimum strict, default 1: {rng}; sets "
                   f"the DOP network's Substeps (14 here): {linked}; height after 0.5 s with 1 / 2: "
                   f"{ys[1]} / {ys[2]}")]


def _folders(entries, path=()):
    """(path of labels, folder template) for every folder under `entries`, depth first."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            out.append((path + (pt.label(),), pt))
            out += _folders(pt.parmTemplates(), path + (pt.label(),))
    return out


def test_solver_sop_layout():
    """The Solver SOP's tabs and folders, in RBD Bullet Solver's order: Setup, Properties, Collision, Forces,
    Constraints, Visualization, Advanced, Output (no Misc); Collision Geometry's header checkbox is Enable Collision Geometry, which sits just before it
    (as RBD Bullet Solver's); the Pieces tab is Shape, Body, Material, CCD & Contacts, Sleep (Start
    Asleep first), Override Attributes (Override Attributes from SOP); the Solver tab holds Cache Simulation
    (label hidden, joined to Cache Memory) right after Solve on Creation Frame and ends with Contacts and Sleeping
    (the solver's Allow Sleeping); Advanced ends with a Constraints folder holding a closed folder per joint type;
    the Output tab has Impact Data, Attributes (Aging) and Attribute Transfer;
    Visualization is the Visualize Geometries folder (Geometry, Ground, Collision Geometry) then the Inspector folder; no Detach Faces; every collapsible folder open but Impact Data,
    Sleeping and the joint folders."""
    sop = solver_sop("sop_layout")
    entries = sop.parmTemplateGroup().entries()
    problems = []

    def expect(what, got, want):
        if got != want:
            problems.append(f"{what}: {got!r}, expected {want!r}")

    tabs = [pt.label() for pt in entries if pt.type() == hou.parmTemplateType.Folder]
    expect("tabs", tabs,
           ["Setup", "Properties", "Collision", "Forces", "Constraints", "Visualization", "Advanced", "Output"])
    folders = _folders(entries)
    by_path = {path: pt for path, pt in folders}
    coll = by_path.get(("Collision", "Collision Geometry"))
    expect("Collision Geometry header toggle", coll.tags().get("sidefx::header_toggle") if coll else None,
           "usecollisions")
    expect("Collision tab", [pt.name() for pt in by_path[("Collision",)].parmTemplates()][:1]
           if ("Collision",) in by_path else None, ["usecollisions"])
    show_geo = sop.parm("showcollision")
    expect("Show Collision Geometry default (the guide shows the external collisions)",
           show_geo.eval() if show_geo else None, 1)
    expect("Show Collision Shapes default", sop.parm("exportshapes").eval() if sop.parm("exportshapes") else None, 0)
    expect("Collision Geometry group_type", coll.tags().get("group_type") if coll else None, "collapsible")
    pieces_tab = by_path.get(("Properties",))
    expect("Pieces tab folders", [pt.label() for pt in pieces_tab.parmTemplates()] if pieces_tab else None,
           ["Shape", "Body", "Material", "CCD & Contacts", "Sleep", "Override Attributes"])
    override = by_path.get(("Properties", "Override Attributes"))
    expect("Override Attributes from SOP", [pt.name() for pt in override.parmTemplates()] if override else None,
           ["overwritefromsop", "overwriteattributes"])
    sop.parm("overwritefromsop").set(1)
    inner_pieces = sop.node("dopnet/pieces")
    expect("Override Attributes from SOP reaches the pieces' Packed Object",
           inner_pieces.parm("overwritefromsop").eval() if inner_pieces else None, 1)
    sop.parm("overwritefromsop").set(0)
    # Each joint type's Enable, Data Name and Default Operation reach its inner node.
    glue = sop.node("dopnet/glue")
    glue_switch = sop.node("dopnet/switch_use_glue")
    sop.parm("useglue").set(0)
    switch_off = glue_switch.parm("input").eval() if glue_switch else None
    sop.parm("useglue").set(1)
    switch_on = glue_switch.parm("input").eval() if glue_switch else None
    expect("Enable Glue off / on drives its Switch", (switch_off, switch_on), (0, 1))
    sop.parm("glue_dataname").set("Bond")
    expect("Glue Data Name reaches the node", glue.parm("dataname").eval() if glue else None, "Bond")
    sop.parm("glue_dataname").set("Glue")
    sop.parm("glue_defaultparmop").set(1)
    expect("Glue Default Operation Set Always reaches its settings (Parameter Operation 1)",
           glue.parm("parmop_strength").eval() if glue else None, 1)
    sop.parm("glue_defaultparmop").set(0)
    inner_solver = sop.node("dopnet/box3d_solver")
    sop.parm("addgravity").set(0)
    off_y = inner_solver.parmTuple("gravity")[1].eval() if inner_solver else None
    sop.parm("addgravity").set(1)
    on_y = inner_solver.parmTuple("gravity")[1].eval() if inner_solver else None
    expect("Add Gravity off / on: the inner solver's gravity y (0, then -9.80665)", (off_y, on_y), (0.0, -9.80665))
    solver_tab = by_path.get(("Setup",))
    expect("Solver tab ends with", [pt.label() for pt in solver_tab.parmTemplates()][-2:] if solver_tab else None,
           ["Contacts", "Sleeping"])
    attributes = by_path.get(("Output", "Attributes"))
    expect("Output / Attributes", [pt.name() for pt in attributes.parmTemplates()] if attributes else None,
           ["enableaging", "reapdead", "reapatframeend"])
    expect("Output tab folders", [pt.label() for pt in by_path[("Output",)].parmTemplates()]
           if ("Output",) in by_path else None, ["Impact Data", "Attributes", "Attribute Transfer"])
    sleeping = by_path.get(("Setup", "Sleeping"))
    first = sleeping.parmTemplates()[0].name() if sleeping and sleeping.parmTemplates() else None
    expect("first in Solver / Sleeping", first, "enablesleep")
    piece_sleep = by_path.get(("Properties", "Sleep"))
    expect("first in Pieces / Sleep", piece_sleep.parmTemplates()[0].name() if piece_sleep else None,
           "startasleep")
    expect("Add Impact Data on Output", sop.parm("addimpact") is not None, True)
    advanced = by_path.get(("Advanced",))
    expect("Advanced tab folders", [pt.label() for pt in advanced.parmTemplates()] if advanced else None,
           ["Stability", "Performance", "Collision", "Constraints"])
    advanced_collision = by_path.get(("Advanced", "Collision"))
    expect("Advanced / Collision", [pt.name() for pt in advanced_collision.parmTemplates()] if advanced_collision
           else None, ["collision_solvefirstframe"])
    forces_tab = by_path.get(("Forces",))
    expect("Forces tab folders", [pt.label() for pt in forces_tab.parmTemplates() if pt.type() ==
                                  hou.parmTemplateType.Folder] if forces_tab else None,
           ["Gravity", "Force DOPs", "Bounce / Friction", "Glue"])
    glue = by_path.get(("Forces", "Glue"))
    expect("Forces / Glue", [pt.name() for pt in glue.parmTemplates()] if glue else None,
           ["gluecompound", "gluecompoundload"])
    expect("Solver tab folders", [pt.label() for pt in solver_tab.parmTemplates()] if solver_tab else None,
           ["General", "Simulation", "Contacts", "Sleeping"])
    general = by_path.get(("Setup", "General"))
    expect("Solver / General", [pt.name() for pt in general.parmTemplates()] if general else None,
           ["usescale", "scale", "timescale"])
    simulation = by_path.get(("Setup", "Simulation"))
    solver_names = [pt.name() for pt in simulation.parmTemplates()] if simulation else []
    cache = next((pt for pt in simulation.parmTemplates() if pt.name() == "cacheenabled"), None) if simulation else None
    expect("Cache Simulation (label hidden, joined) in Solver / Simulation, after Solve on Creation Frame",
           (cache.isLabelHidden(), cache.joinsWithNext(),
            solver_names.index("cacheenabled") == solver_names.index("solvefirstframe") + 1) if cache else None,
           (True, True, True))
    vis_tab = by_path.get(("Visualization",))
    expect("Visualization tab", [pt.name() for pt in vis_tab.parmTemplates()] if vis_tab else None,
           ["visualizegeometries", "b3dsop_visualize_geometries", "b3dsop_inspector"])
    visualize = by_path.get(("Visualization", "Visualize Geometries"))
    expect("Visualize Geometries folders", [pt.label() for pt in visualize.parmTemplates()] if visualize else None,
           ["Geometry", "Ground", "Collision Geometry", "Show Constraints", "Constraint Geometry", "Impact Data"])
    for label, names in (("Geometry", ["exportshapes", "shapecolor"]), ("Ground", ["showground", "shadeground"]),
                         ("Collision Geometry", ["showcollision", "showcollisionshapes", "collisioncolor"])):
        sub_folder = by_path.get(("Visualization", "Visualize Geometries", label))
        expect(f"Visualize Geometries / {label}", [pt.name() for pt in sub_folder.parmTemplates()] if sub_folder
               else None, names)
    # The operator toolbar along the top of the viewport (export level 3) and the viewer state's sections.
    toolbar = sorted(pt.name() for pt in sop.parmTemplateGroup().entriesWithoutFolders() if pt._exportLevel() == 3)
    expect("parameters in the viewport toolbar", toolbar, ["exportshapes", "resimulate", "showcollision", "showground"])
    definition = sop.type().definition()
    expect("default state", definition.sections().get("DefaultState") and definition.sections()["DefaultState"].contents(),
           "box3d_solver_sop")
    expect("viewer state sections", sorted(n for n in definition.sections() if n.startswith("ViewerState")),
           ["ViewerStateInstall", "ViewerStateModule", "ViewerStateName.orig", "ViewerStateUninstall"])
    expect("Detach Faces", sop.parm("collision_detachfaces"), None)
    shut = sorted("/".join(path) for path, pt in folders
                  if pt.folderType() == hou.folderType.Collapsible and pt.tags().get("group_default") != "1")
    joint_prefix = "Advanced/Constraints/"
    expect("closed folders", [p for p in shut if not p.startswith(joint_prefix)],
           ["Output/Impact Data", "Setup/Sleeping"])
    constraint_folders = [p for p in shut if p.startswith(joint_prefix)]
    all_constraint = [path for path, pt in folders if path[:2] == ("Advanced", "Constraints") and len(path) == 3]
    expect("Advanced / Constraints folders, all closed (one per constraint type)", len(constraint_folders),
           len(all_constraint))
    expect("Advanced / Constraints has folders", len(all_constraint) > 0, True)
    expect("Constraints tab folders", [pt.label() for pt in by_path[("Constraints",)].parmTemplates()
                                       if pt.type() == hou.parmTemplateType.Folder] if ("Constraints",) in by_path
           else None, ["Break Thresholds"])
    breaks = by_path.get(("Constraints", "Break Thresholds"))
    expect("Break Thresholds: open, header toggle", (breaks.tags().get("group_default"),
                                                    breaks.tags().get("sidefx::header_toggle")) if breaks else None,
           ("1", "enable_constraintbreaks"))
    expect("Constraints tab leads with the Break Thresholds toggle and folder",
           [pt.name() for pt in by_path[("Constraints",)].parmTemplates()][:2] if ("Constraints",) in by_path else None,
           ["enable_constraintbreaks", "b3dsop_break_thresholds"])
    if problems:
        return [Result("solver_sop_layout", Result.FAIL, "; ".join(problems))]
    return [Result("solver_sop_layout", Result.PASS,
                   f"tabs {tabs}; {len(folders)} folders, Impact Data and Sleeping closed; Allow Sleeping first in Solver / "
                   "Sleeping; Enable Collision Geometry the Collision Geometry header")]


def test_solver_sop_proxy():
    """With a proxy, the proxy is simulated and the geometry follows it: an unpacked polygon box
    named like a packed proxy box falls with it, its centre on the proxy's point every frame, while
    the proxy output is the simulated proxy."""
    proxy, names = packed_boxes("sop_proxy_proxy", [GROUND, {"size": (1, 1, 1), "center": (0, 3, 0)}])
    hires = new_geo("sop_proxy_hires")
    unpack = create(hires, "unpack", "unpack")
    merge_in = create(hires, "object_merge", "proxy_in")
    set_parm(merge_in, ["objpath1"], proxy)
    unpack.setInput(0, merge_in)
    set_parm(unpack, ["transfer_attributes", "transferattributes"], "name")
    display(unpack)
    sop = solver_sop("sop_proxy", geometry=unpack.path(), proxy=proxy)

    def sample(s, f):
        sim = named_positions(s.geometry(2))
        g = s.geometry(0)
        prims = [p for p in g.prims() if g.findPrimAttrib("name") and p.attribValue("name") == names[1]]
        if not prims or names[1] not in sim:
            return None
        pts = {v.point().number(): v.point().position() for p in prims for v in p.vertices()}
        centre = sum(pts.values(), hou.Vector3()) / len(pts)
        return centre, sim[names[1]]

    track = play(sop, 48, sample)
    worst = max(((c - p).length() for c, p in (v for v in track.values() if v)), default=None)
    end = track[48]
    ok = worst is not None and worst < 1e-4 and end is not None and close(end[1][1], 0.5, 0.03)
    return [Result("solver_sop_proxy", Result.PASS if ok else Result.FAIL,
                   f"geometry centre vs simulated proxy point: largest gap {worst} (expected < 1e-4); proxy "
                   f"rests at y={end[1][1]:.3f} (expected 0.5)" if end else "no matching piece in the outputs")]


def test_solver_sop_constraints():
    """A box welded to the world on the Constraints input (constraint_name Weld) stays put for 1 s,
    and the constraints output carries the solver's feedback (a force primitive attribute)."""
    pieces, names = packed_boxes("sop_weld_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    lines = constraint_lines("sop_weld_network", [("", (0, 3, 0), names[0], (0, 3, 0), "Weld")])
    sop = solver_sop("sop_weld", geometry=pieces, constraints=lines)

    def sample(s, f):
        if f != 24:
            return None
        con = s.geometry(1)
        return named_positions(s.geometry(0)).get(names[0]), con.findPrimAttrib("force") is not None, len(con.prims())

    pos, has_force, nprims = play(sop, 24, sample)[24]
    ok = pos is not None and close(pos[1], 3.0, 1e-3) and has_force and nprims == 1
    return [Result("solver_sop_constraints", Result.PASS if ok else Result.FAIL,
                   f"welded box y={pos[1] if pos else None} after 1 s (expected 3.0); constraints output: "
                   f"{nprims} primitive(s), force attribute {'present' if has_force else 'missing'}")]


def test_solver_sop_constraint_guide_radius():
    """Guide Radius and Guide Scale (the constraint lines in the viewport): a Weld line shown with Show
    Constraints has a radius of Guide Radius times its type's Guide Scale (0.05 by default; 0.15 with the Weld's
    Guide Scale 3; 0.1 with Guide Radius 0.1), on the line's points before the wire is made, and the wire in the
    asset's Guide Geometry is 0.2 thicker with Guide Scale 3 (the radius grows 0.1, twice that across)."""
    pieces, names = packed_boxes("sop_guide_radius_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    lines = constraint_lines("sop_guide_radius_network", [("", (3, 3, 0), names[0], (0, 3, 0), "Weld")])
    results = []
    thickness = {}
    for label, parms, want in (("defaults", {}, 0.05),
                               ("Weld Guide Scale 3", {"weld_guidescale": 3.0}, 0.15),
                               ("Guide Radius 0.1", {"showconstraintguidesradius": 0.1}, 0.1)):
        sop = solver_sop("sop_guide_radius_" + label.split()[0].lower(), geometry=pieces, constraints=lines)
        set_parm(sop, ["showconstraints"], 1)
        for name, value in parms.items():
            set_parm(sop, [name], value)

        def sample(s, f):
            if f != 3:
                return None
            node = s.node("constraint_guide_radius")
            geo = node.geometry() if node else None
            attr = geo.findPointAttrib("guide_radius") if geo else None
            radius = geo.points()[0].attribValue("guide_radius") if attr and geo.points() else None
            box = s.node("GUIDE_GEO").geometry().boundingBox()
            return radius, box.sizevec()[1]

        radius, height = play(sop, 3, sample)[3]
        thickness[label] = height
        ok = radius is not None and close(radius, want, 1e-4) and height > 0
        results.append(Result("solver_sop_constraint_guide_radius", Result.PASS if ok else Result.FAIL,
                              f"{label}: guide_radius on the line's points {radius} (expected {want}); wire thickness "
                              f"{height:.4f}"))
    # The wire's height is its diameter plus a little the polywire adds, so the changes are compared.
    grew = thickness["Weld Guide Scale 3"] - thickness["defaults"]
    results.append(Result("solver_sop_constraint_guide_radius", Result.PASS if close(grew, 0.2, 0.02) else Result.FAIL,
                          f"wire thickness grew {grew:.3f} with Weld Guide Scale 3 (expected 0.2: the radius grows "
                          f"from 0.05 to 0.15, twice that across)"))
    return results


def test_constraint_properties_scaling():
    """A setting scaled by an attribute, as RBD Constraint Properties does (Geometry Attribute Transfer). A Revolute
    line between anchors A and B, Stiffness 10 ticked. The pieces are A1, A2, A3 (parentpiece A, weight 1, 3, 8, the
    anchor nearest A2) and B (weight 5). Scale by Attribute `weight` multiplies by the average of the two ends'
    pieces: A is its three pieces by Transfer Mode (Closest 3, Min 1, Average 4, Max 8) and B is 5, so 40, 30, 45 and 65.
    Scale by Value multiplies by the line's own stiffness attribute (2): 20; both: 10 * 2 * 4 = 80. Randomize with
    Variance 0 leaves 10 as it is, and with 0.5 gives a value from 5 to 15."""
    geo = new_geo("cp_scaling")
    pieces = create(geo, "attribwrangle", "pieces")
    set_parm(pieces, ["class"], 0)
    set_parm(pieces, ["snippet"], 'addpointattrib(0, "name", ""); addpointattrib(0, "parentpiece", ""); '
             'addpointattrib(0, "weight", 0.0); '
             'string names[] = {"A1", "A2", "A3", "B"}; string parents[] = {"A", "A", "A", ""}; '
             'float weights[] = {1.0, 3.0, 8.0, 5.0}; float xs[] = {-1.0, 1.0, 3.0, 10.0}; '
             'for (int i = 0; i < 4; i++) { int pt = addpoint(0, set(xs[i], 0, 0)); '
             'setpointattrib(0, "name", pt, names[i]); setpointattrib(0, "parentpiece", pt, parents[i]); '
             'setpointattrib(0, "weight", pt, weights[i]); }')
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'addprimattrib(0, "stiffness", 0.0); int a = addpoint(0, set(0.9, 0, 0)); int b = addpoint(0, set(10, 0, 0)); '
             'setpointattrib(0, "name", a, "A"); setpointattrib(0, "name", b, "B"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'setprimattrib(0, "stiffness", pr, 2.0);')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, pieces)
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("addrevolute_stiffness", 1), ("revolute_stiffness", 10.0),
                        ("revolute_stiffnessattribute", "weight")):
        set_parm(cp, [name], value)

    def stiffness(scaling, mode):
        set_parm(cp, ["revolute_stiffnessscaling"], scaling)
        set_parm(cp, ["transfermode"], mode)
        g = cp.geometry(1)
        return round(g.prims()[0].attribValue("stiffness"), 3) if g and g.findPrimAttrib("stiffness") else None

    got = {"none": stiffness(0, 0), "closest": stiffness(1, 0), "min": stiffness(1, 1), "average": stiffness(1, 2),
           "max": stiffness(1, 3), "value": stiffness(2, 0), "both": stiffness(3, 0)}
    want = {"none": 10.0, "closest": 40.0, "min": 30.0, "average": 45.0, "max": 65.0, "value": 20.0, "both": 80.0}
    ok = got == want
    results = [Result("constraint_properties_scaling", Result.PASS if ok else Result.FAIL,
                       f"stiffness by scaling / Transfer Mode {got} (expected {want})")]
    # Randomize: Variance 0 changes nothing, Variance 0.5 gives 5 to 15 (and not exactly 10).
    set_parm(cp, ["revolute_stiffnessscaling"], 0)
    set_parm(cp, ["revolute_stiffnessrandomize"], 1)
    spread = {}
    for variance in (0.0, 0.5):
        set_parm(cp, ["revolute_stiffnessvariance"], variance)
        spread[variance] = round(cp.geometry(1).prims()[0].attribValue("stiffness"), 3)
    good = spread[0.0] == 10.0 and 5.0 <= spread[0.5] <= 15.0 and spread[0.5] != 10.0
    results.append(Result("constraint_properties_scaling", Result.PASS if good else Result.FAIL,
                          f"stiffness 10 randomized with Variance 0 / 0.5: {spread} (expected 10, and 5 to 15 but not 10)"))
    return results


def test_constraint_properties_next():
    """Switch Constraint Type When Broken and the next settings. A Revolute line: with the switch off, Next Constraint
    Name and a next Revolute Stiffness (99) write nothing; with it on, Edit writes next_constraint_name Weld and
    next_stiffness 99, and Set writes the next name and the next settings that differ from the defaults."""
    geo = new_geo("cp_next")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("donext_constraintname", 1), ("next_constraintname", "Weld"),
                        ("addnext_revolute_stiffness", 1), ("next_revolute_stiffness", 99.0),
                        ("next_constrainttype", 2)):
        set_parm(cp, [name], value)

    def attrs():
        g = cp.geometry(1)
        prim = g.prims()[0]
        return {a: prim.attribValue(a) for a in ("next_constraint_name", "next_stiffness")
                if g.findPrimAttrib(a) is not None}

    off = attrs()
    set_parm(cp, ["usenextconstraint"], 1)
    on = attrs()
    set_parm(cp, ["action"], 1)
    in_set = attrs()
    want_on = {"next_constraint_name": "Weld", "next_stiffness": 99.0}
    ok = ("next_constraint_name" not in off and "next_stiffness" not in off and on == want_on
          and in_set.get("next_constraint_name") == "Weld" and in_set.get("next_stiffness") == 99.0)
    return [Result("constraint_properties_next", Result.PASS if ok else Result.FAIL,
                   f"line attributes with the switch off {off}, on in Edit {on}, on in Set {in_set} (expected "
                   f"nothing, then {want_on}, then the next name and stiffness 99)")]


def test_configure_packs():
    """Box3D Configure packs unpacked input as RBD Configure does: two polygon boxes with no name come
    out as two packed pieces named on their points, with the ticked setting on the group's piece; the
    result simulates in the Box3D Solver SOP (both pieces fall)."""
    geo = new_geo("cfg_pack")
    merge = create(geo, "merge", "boxes")
    for i in range(2):
        box = create(geo, "box", f"box{i}")
        set_parm(box, ["t"], (3.0 * i, 3.0, 0.0))
        merge.setInput(i, box)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, merge)
    set_parm(grp, ["class"], "primitive")
    set_parm(grp, ["snippet"], 'if (@P.x < 1.5) setprimgroup(0, "chosen", @primnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addfriction1", 1), ("friction1", 0.25)):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    packed = [p for p in g.prims() if "Packed" in p.type().name()] if g else []
    named = g is not None and g.findPointAttrib("name") is not None and g.findPrimAttrib("name") is None
    frictions = sorted(round(p.attribValue("friction"), 4) for p in g.points()) \
        if g is not None and g.findPointAttrib("friction") else []
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    low = play(solver, 24, lambda s, f: s.geometry(0).boundingBox().maxvec()[1] if f == 24 else None)[24]
    ok = len(packed) == 2 and len(g.prims()) == 2 and named and len(frictions) == 2 and \
        frictions[0] == 0.25 and frictions[1] != 0.25 and low is not None and low < 2.0
    return [Result("configure_packs", Result.PASS if ok else Result.FAIL,
                   f"{len(packed)} packed of {len(g.prims()) if g else 0} primitives (expected 2 of 2), name on "
                   f"points only: {named}, friction per piece {frictions} (expected one 0.25); top of the "
                   f"pieces after 1 s: {low} (expected below 2: fell)")]


def test_configure():
    """Box3D Configure (tools/make_configure_sop.py) writes ticked settings onto the pieces in its
    group only: Body Type Static and Friction 0.25 on one of two boxes. Into the Box3D Solver SOP,
    that box stays put and the other falls (it keeps active 1, not the 0 VEX would give it)."""
    pieces, names = packed_boxes("cfg_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")  # the Group field takes a group name, as RBD Configure's
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    # Multiparm entry 1 (Number of Attributes): group#, add<setting># checkbox, <setting>#.
    for n, v in (("group1", "chosen"), ("addbodytype1", 1), ("bodytype1", 0), ("addfriction1", 1),
                 ("friction1", 0.25)):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    attrs = {p.attribValue("name"): (p.attribValue("active"), round(p.attribValue("friction"), 4)) for p in g.points()}
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    pos = play(solver, 24, lambda s, f: named_positions(s.geometry(0)) if f == 24 else None)[24]
    y_static, y_free = pos[names[0]][1], pos[names[1]][1]
    ok = (attrs.get(names[0], (None, None)) == (0, 0.25) and attrs.get(names[1], (None,))[0] == 1
          and close(y_static, 3.0, 1e-3) and y_free < 2.0)
    return [Result("configure", Result.PASS if ok else Result.FAIL,
                   f"(active, friction) per piece {attrs} (expected {names[0]}: (0, 0.25), {names[1]}: active 1); "
                   f"after 1 s the static box is at y={y_static:.3f} (expected 3), the other at {y_free:.3f} "
                   f"(expected below 2: fell)")]


def test_configure_group_patterns():
    """Box3D Configure's Group takes patterns as RBD Configure's does, not only group names: `@name=<piece>`,
    a wildcard over group names (`chos*`) and a point number each pick the first of two pieces, which
    alone gets Friction 0.25."""
    pieces, names = packed_boxes("cfg_pat_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg_pat")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen_a", @ptnum, 1);')
    rows, ok = [], True
    for label, pattern in (("name", f"@name={names[0]}"), ("wildcard", "chos*"), ("number", "0")):
        cfg = create(geo, "box3d_configure", f"configure_{label}")
        cfg.setInput(0, grp)
        for n, v in (("group1", pattern), ("addfriction1", 1), ("friction1", 0.25)):
            set_parm(cfg, [n], v)
        g = cfg.geometry()
        got = {p.attribValue("name"): round(p.attribValue("friction"), 4) for p in g.points()} \
            if g is not None and g.findPointAttrib("friction") else {}
        good = got.get(names[0]) == 0.25 and got.get(names[1]) not in (None, 0.25)
        rows.append(f"{label} ({pattern!r}): {got}")
        ok = ok and good
    return [Result("configure_group_patterns", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + f" (expected 0.25 on {names[0]} only)")]


def test_constraint_properties():
    """Box3D Constraint Properties (tools/make_constraint_properties_sop.py) writes ticked settings onto the
    constraints in its group only: on two lines, one in a group, Revolute Stiffness 7 and Enable Spring land
    on the first, the other keeps the "not set" values the solver skips."""
    geo = new_geo("conprops")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], "detail")
    set_parm(lines, ["snippet"],
             'for (int i = 0; i < 2; i++) { int a = addpoint(0, set(i, 0, 0)), b = addpoint(0, set(i, 1, 0)); '
             'int pr = addprim(0, "polyline", a, b); '
             'if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", ""); '
             'setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'if (i == 0) setprimgroup(0, "chosen", pr, 1); }')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    for n, v in (("action", 0), ("constraintgroup", "chosen"), ("constrainttype", 2), ("addrevolute_stiffness", 1),
                 ("revolute_stiffness", 7.0), ("addrevolute_enable_spring", 1), ("revolute_enable_spring", 1)):
        set_parm(cp, [n], v)
    g = cp.geometry(1)
    prims = g.prims()
    first = (round(prims[0].attribValue("stiffness"), 3), prims[0].attribValue("enable_spring"))
    other = (prims[1].attribValue("stiffness"), prims[1].attribValue("enable_spring"))
    ok = first == (7.0, 1) and other[0] <= -3.0e38 and other[1] < 0 and g.findPrimGroup("__tmpconpropgrp") is None
    return [Result("constraint_properties", Result.PASS if ok else Result.FAIL,
                   f"(stiffness, enable_spring) on the group's line {first} (expected (7, 1)), on the other {other} "
                   "(expected not set: below -3e38 and -1)")]


def test_constraint_properties_edit():
    """Box3D Constraint Properties' viewport edits (tools/b3d_constraint_properties_state.py): three lines, two in
    a group, the stash rotating the anchors of the selected one (what the handle writes) lands as `orient` on that
    line's two points only; Set mode writes every Revolute setting and the name, without any checkbox; the output has
    no `__idx`; the state's sections and the toolbar parameters are in the asset."""
    geo = new_geo("conprops_edit")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], "detail")
    set_parm(lines, ["snippet"],
             'for (int i = 0; i < 3; i++) { int a = addpoint(0, set(i, 0, 0)), b = addpoint(0, set(i, 1, 0)); '
             'int pr = addprim(0, "polyline", a, b); '
             'if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", ""); '
             'setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'if (i < 2) setprimgroup(0, "chosen", pr, 1); }')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    set_parm(cp, ["constraintgroup"], "chosen")
    set_parm(cp, ["__sel"], "@__idx=1")
    set_parm(cp, ["__idx"], 1)
    module = cp.type().hdaViewerStateModule()
    quat = module.quaternion_from_euler(0.0, 0.0, 90.0)
    module.set_edits(cp, {i: (quat, quat) for i in module.selected_ids(cp)})
    g = cp.geometry(1)
    orients = [tuple(round(x, 3) for x in p.attribValue("orient")) for p in g.points()]         if g.findPointAttrib("orient") else []
    turned = (0.0, 0.0, 0.707, 0.707)
    ok = (module.selected_ids(cp) == [1] and orients[2:4] == [turned, turned]
          and all(o == (0.0, 0.0, 0.0, 1.0) for o in orients[:2] + orients[4:])
          and g.findPrimAttrib("__idx") is None)
    detail = f"orient per point {orients} (expected only points 2 and 3 turned 90 degrees about Z)"
    # Toggle Motor and the wheel's motor velocity, on the two lines in the group: edited ones only, the stash keeps
    # the orient of a line whose motor is edited later.
    set_parm(cp, ["__sel"], "@__idx=0 @__idx=1")
    ids = module.selected_ids(cp)
    module.set_edits(cp, {i: (quat, quat) for i in ids})
    module.toggle_motor(cp, ids)
    module.add_motor_velocity(cp, ids, 2, 10.0)
    g = cp.geometry(1)
    motor = [p.attribValue("enable_motor") for p in g.prims()]
    velocity = [round(p.attribValue("motor_velocity")[2], 3) for p in g.prims()]
    kept = [tuple(round(x, 3) for x in p.attribValue("orient")) for p in g.points()][:4]
    ok = ok and motor == [1, 1, -1] and velocity[:2] == [10.0, 10.0] and kept == [turned] * 4
    detail += f"; enable_motor {motor}, motor_velocity z {velocity}, orient kept {kept}"
    module.clearEdits({"node": cp})
    set_parm(cp, ["action"], 1)
    set_parm(cp, ["constrainttype"], 2)
    set_parm(cp, ["revolute_stiffness"], 5.0)
    set_parm(cp, ["constraintname"], "Weld")
    g = cp.geometry(1)
    stiff = [round(p.attribValue("stiffness"), 3) for p in g.prims()] if g.findPrimAttrib("stiffness") else []
    names = [p.attribValue("constraint_name") for p in g.prims()]
    ok = ok and stiff[:2] == [5.0, 5.0] and stiff[2] <= -3.0e38 and names == ["Weld", "Weld", "Revolute"]
    detail += f"; Set mode stiffness {stiff}, names {names}"
    definition = cp.type().definition()
    sections = sorted(n for n in definition.sections() if n.startswith("ViewerState"))
    toolbar = sorted(pt.name() for pt in cp.parmTemplateGroup().entriesWithoutFolders() if pt._exportLevel() == 3)
    # A setting a toggle switches on greys out with the toggle off in both modes: Edit once the toggle is being
    # written, Set (checkboxes hidden) by its value.
    cond = cp.parm("weld_linear_stiffness").parmTemplate().conditionals().get(hou.parmCondType.DisableWhen, "")
    greyed = ("{ weld_enable_linear_spring == 0 addweld_enable_linear_spring == 1 action == 0 }" in cond
              and "{ weld_enable_linear_spring == 0 action == 1 }" in cond)
    ok = ok and greyed and len(sections) == 4 and toolbar == ["clearedits", "editparms", "importedits"]
    detail += f"; weld_linear_stiffness disable-when {cond!r}"
    detail += f"; sections {sections}, toolbar {toolbar}"
    return [Result("constraint_properties_edit", Result.PASS if ok else Result.FAIL, detail)]


def test_no_subdata_warnings():
    """Box3D Data under an object's Geometry raises no data hint warnings ("Unexpected subdata found",
    "attached to an unexpected parent data"): a Box3D Packed Object cooked on two frames, and the Box3D Solver
    SOP cooking a piece, and a Packed Object and an RBD Object stepped by the Box3D Solver DOP, have no warnings on
    the Geometry or the data node."""
    pieces, names = packed_boxes("hint_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    dop = hou.node("/obj").createNode("dopnet", "hint_dop")
    po = dop.createNode("box3d_packedobject", "po")
    set_parm(po, ["soppath"], pieces)
    po.setDisplayFlag(True)
    found = []
    for frame in (1, 2):
        hou.setFrame(frame)
        dop.cook(force=True)
        found += [f"{n.path()}: {w}" for n in [po] + list(po.allSubChildren()) for w in n.warnings()
                  if "subdata" in w or "unexpected parent" in w]
    # The Box3D Solver DOP writes the Geometry back as a SIM_GeometryCopy, an RBD Object's stays a SIM_SopGeometry.
    box = create(new_geo("hint_box"), "box", "box")
    for label, node_type, sop in (("packed", "box3d_packedobject", pieces), ("rbd", "rbdobject", box.path())):
        sim_dop = hou.node("/obj").createNode("dopnet", f"hint_{label}_dop")
        obj = sim_dop.createNode(node_type, "obj")
        set_parm(obj, ["soppath"], sop)
        data = sim_dop.createNode("box3d_data", "data")
        data.setInput(0, obj)
        sim_solver = sim_dop.createNode("box3d_solver", "solver")
        sim_solver.setInput(0, data)
        sim_solver.setDisplayFlag(True)
        for frame in (1, 2, 3):
            hou.setFrame(frame)
            sim_dop.cook(force=True)
        found += [f"{n.path()}: {w}" for n in [sim_dop] + list(sim_dop.allSubChildren()) for w in n.warnings()
                  if "subdata" in w or "unexpected parent" in w]
    geo = new_geo("hint_solver")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, src)
    hou.setFrame(2)
    solver.cook(force=True)
    found += [f"{n.path()}: {w}" for n in [solver] + list(solver.allSubChildren()) for w in n.warnings()
              if "subdata" in w or "unexpected parent" in w]
    return [Result("no_subdata_warnings", Result.PASS if not found else Result.FAIL,
                   f"data hint warnings: {found or 'none'} (expected none)")]


def test_configure_initial_state():
    """Box3D Configure's Initial State: Velocity (2, 0, 0) moves the group's piece 2 in 1 s through the Box3D
    Solver SOP (no gravity needed in x), and Angular Velocity (0, 90, 0) degrees per second is written
    as w in radians (pi / 2)."""
    pieces, names = packed_boxes("cfg_vel_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (0, 3, 5)}])
    geo = new_geo("cfg_vel")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addv1", 1), ("v1", (2.0, 0.0, 0.0)), ("addw1", 1), ("w1", (0.0, 90.0, 0.0))):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    w = {p.attribValue("name"): p.attribValue("w") for p in g.points()} if g is not None and g.findPointAttrib("w") else {}
    w_ok = names[0] in w and close(w[names[0]][1], math.pi / 2, 1e-4)
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    pos = play(solver, 25, lambda s, f: named_positions(s.geometry(0)) if f == 25 else None)[25]
    x0, x1 = pos[names[0]][0], pos[names[1]][0]
    ok = w_ok and close(x0, 2.0, 0.1) and close(x1, 0.0, 0.05)
    return [Result("configure_initial_state", Result.PASS if ok else Result.FAIL,
                   f"w on the piece {w.get(names[0])} (expected y = {math.pi / 2:.4f}); x after 1 s: grouped {x0:.3f} "
                   f"(expected 2), other {x1:.3f} (expected 0)")]


def test_configure_pin():
    """Box3D Configure's Pin adds one Weld line from the world to each pinned piece on the Constraint Geometry
    output. Wired into the Box3D Solver SOP, the pinned piece stays at y = 3 and the other falls."""
    pieces, names = packed_boxes("cfg_pin_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg_pin")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addpin1", 1), ("pin1", 1), ("pintype1", 0)):
        set_parm(cfg, [n], v)
    lines = cfg.geometry(1)
    kinds = [p.attribValue("constraint_name") for p in lines.prims()] if lines is not None \
        and lines.findPrimAttrib("constraint_name") else None
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg, 0)
    solver.setInput(1, cfg, 1)
    display(solver)
    pos = play(solver, 24, lambda s, f: named_positions(s.geometry(0)) if f == 24 else None)[24]
    y_pinned, y_free = pos[names[0]][1], pos[names[1]][1]
    ok = kinds == ["Weld"] and close(y_pinned, 3.0, 0.02) and y_free < 2.0
    return [Result("configure_pin", Result.PASS if ok else Result.FAIL,
                   f"constraint lines {kinds} (expected one 'Weld'); after 1 s the pinned piece is at y={y_pinned:.3f} "
                   f"(expected 3), the other at {y_free:.3f} (expected below 2: fell)")]


TESTS = {
    "solver_sop_collision": test_solver_sop_collision,
    "solver_sop_pieces": test_solver_sop_pieces,
    "solver_sop_global_substeps": test_solver_sop_global_substeps,
    "solver_sop_before_start_bullet": test_solver_sop_before_start_bullet,
    "solver_sop_solve_first_frame_bullet": test_solver_sop_solve_first_frame_bullet,
    "solver_sop_keep_broken_bullet": test_solver_sop_keep_broken_bullet,
    "solver_sop_break_impact_bullet": test_solver_sop_break_impact_bullet,
    "solver_sop_break_atframe_bullet": test_solver_sop_break_atframe_bullet,
    "solver_sop_break_distance_bullet": test_solver_sop_break_distance_bullet,
    "solver_sop_break_angle_torque_bullet": test_solver_sop_break_angle_torque_bullet,
    "solver_sop_before_start": test_solver_sop_before_start,
    "solver_sop_solve_first_frame": test_solver_sop_solve_first_frame,
    "solver_sop_keep_broken": test_solver_sop_keep_broken,
    "configure_packs": test_configure_packs,
    "solver_sop_rest_anchors": test_solver_sop_rest_anchors,
    "solver_sop_initial_bake": test_solver_sop_initial_bake,
    "solver_sop_removed_pieces": test_solver_sop_removed_pieces,
    "solver_sop_collision_names": test_solver_sop_collision_names,
    "solver_sop_world_scale": test_solver_sop_world_scale,
    "solver_sop_ground": test_solver_sop_ground,
    "solver_sop_collision_static": test_solver_sop_collision_static,
    "solver_sop_animated_setting": test_solver_sop_animated_setting,
    "solver_sop_constraint_settings": test_solver_sop_constraint_settings,
    "solver_sop_ground_guide": test_solver_sop_ground_guide,
    "solver_sop_constraint_guide_radius": test_solver_sop_constraint_guide_radius,
    "solver_sop_break_next_settings": test_solver_sop_break_next_settings,
    "solver_sop_next_constraint_type": test_solver_sop_next_constraint_type,
    "constraint_properties_scaling": test_constraint_properties_scaling,
    "constraint_properties_next": test_constraint_properties_next,
    "solver_sop_collision_prim_names": test_solver_sop_collision_prim_names,
    "solver_sop_collision_group_names": test_solver_sop_collision_group_names,
    "solver_sop_collision_rest": test_solver_sop_collision_rest,
    "solver_sop_world_anchor": test_solver_sop_world_anchor,
    "unpacked_turning_collider": test_unpacked_turning_collider,
    "solver_sop_impacts": test_solver_sop_impacts,
    "solver_sop_animated_collision": test_solver_sop_animated_collision,
    "solver_sop_forces": test_solver_sop_forces,
    "solver_sop_layout": test_solver_sop_layout,
    "solver_sop_proxy": test_solver_sop_proxy,
    "solver_sop_constraints": test_solver_sop_constraints,
    "configure": test_configure,
    "configure_group_patterns": test_configure_group_patterns,
    "constraint_properties": test_constraint_properties,
    "constraint_properties_edit": test_constraint_properties_edit,
    "no_subdata_warnings": test_no_subdata_warnings,
    "configure_initial_state": test_configure_initial_state,
    "configure_pin": test_configure_pin,
}
