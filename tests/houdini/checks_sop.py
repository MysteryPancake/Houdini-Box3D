"""The Box3D Solver SOP (tools/assets/make_solver_sop.py): its layout, pieces and proxy, solving before and on the first
frame, rest anchors, World Scale, Impacts and POP forces in FORCES."""

import math

import hou
from b3dtest import GROUND, Result, close, constraint_lines, create, display, new_geo, packed_boxes, set_parm

from checks_sop_common import _both_sops, bullet_sop, named_positions, play, solver_sop


def test_solver_sop_quadrics():
    """Sphere, Tube, Circle and Metaball primitives packed and dropped through the Solver SOP, whose pieces
    use Hull per Connected: each rests on the ground at the height its shape should."""
    from checks_bullet import primitive_sop

    cases = (
        ("tube", "tube", "Primitive", {("rad1",): 0.5, ("rad2",): 0.5, ("height",): 1.0}, 0.5),
        ("ellipsoid", "sphere", "Primitive", {("rad",): (0.5, 0.25, 0.5)}, 0.25),
        ("sphere", "sphere", "Primitive", {("rad",): (0.5, 0.5, 0.5)}, 0.5),
        ("metaball", "metaball", None, {("rad", "radius"): (0.5, 0.5, 0.5)}, 0.25),
        ("circle", "circle", "Polygon", {("rad", "radius"): (0.5, 0.5), ("orient",): 2}, 0.02),
    )
    ground, _ = packed_boxes("sop_quadric_ground", [GROUND])
    rows, ok = [], True
    for label, sop_type, item, parms, expect in cases:
        path = primitive_sop(f"sop_quadric_{label}_piece", sop_type, item, parms, True)
        sop = solver_sop(f"sop_quadric_{label}", geometry=path, collision=ground)

        def sample(s, f):
            pts = s.geometry(0).points() if f == 72 else None
            return pts[0].position()[1] if pts else None

        y = play(sop, 72, sample)[72]
        good = y is not None and close(y, expect, 0.03)
        ok = ok and good
        rows.append(f"{label} rests at {y if y is None else round(y, 3)} (expected {expect})")
    return [Result("solver_sop_quadrics", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


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


def test_solver_sop_forces():
    """A POP Force inside the FORCES subnet (the asset's dive target and an editable node, so the
    asset stays locked) wired to PRESOLVE pushes a
    box sideways (Gravity 0): the Multiple Solver runs it before the Box3D Solver, which applies its
    `force`. Wired to POSTSOLVE instead it still pushes, a step later. With nothing wired into either (the usual
    case) the box simply falls and neither the Solver SOP, its DOP network nor any DOP node inside has a warning or an error."""
    results = []
    pieces, names = packed_boxes("sop_force_none_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    sop = solver_sop("sop_force_none", geometry=pieces)
    pos = play(sop, 12, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == 12 else None)[12]
    dopnet = sop.node("dopnet")
    inside = list(dopnet.allSubChildren()) if dopnet is not None else []
    messages = [(n.path(), text) for n in [sop] + ([dopnet] if dopnet is not None else []) + inside
                for text in n.warnings() + n.errors()]
    forces = sop.node("dopnet/FORCES")
    wired = [n.name() for n in (forces.node("PRESOLVE"), forces.node("POSTSOLVE")) if forces and n and n.inputs()]
    ok = pos is not None and pos[1] < 2.9 and not messages and not wired
    results.append(Result("solver_sop_forces", Result.PASS if ok else Result.FAIL,
                          f"nothing wired into PRESOLVE / POSTSOLVE (wired: {wired or 'none'}): the box fell to "
                          f"y {pos[1] if pos else None} after 0.5 s (expected under 2.9); warnings and errors on the asset, "
                          f"its DOP network and its {len(inside)} DOP nodes: {messages[:3] or 'none'}"))
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


def test_solver_sop_world_scale_forces():
    """A POP Wrangle on FORCES' PRESOLVE writing force and torque in real units (Gravity 0) moves and turns a box the
    same under World Scale 1, 10 and 0.1 (`scale_forces`: forces follow the scale, torques its square), and the
    asset's scale_forces node is skipped at World Scale 1."""
    got = {}
    for k in (1.0, 10.0, 0.1):
        pieces, names = packed_boxes(f"sop_scale_force_pieces_{str(k).replace('.', '_')}",
                                     [{"size": (1, 1, 1), "center": (0, 3, 0)}])
        sop = solver_sop(f"sop_scale_force_{str(k).replace('.', '_')}", geometry=pieces)
        set_parm(sop, ["gravity"], (0.0, 0.0, 0.0))
        set_parm(sop, ["usescale"], 1 if k != 1.0 else 0)
        set_parm(sop, ["scale"], k)
        forces = sop.node("dopnet/FORCES")
        wr = create(forces, "popwrangle", "push")
        set_parm(wr, ["snippet"], "v@force = {5000, 0, 0}; v@torque = {0, 0, 300};")
        forces.node("PRESOLVE").setInput(0, wr)

        def sample(s, f):
            if f != 24:
                return None
            pt = s.geometry(3).points()[0]
            q = pt.attribValue("orient")
            return pt.position()[0], 2.0 * math.degrees(math.acos(min(1.0, abs(q[3]))))

        got[k] = play(sop, 24, sample)[24]
    base = got[1.0]
    ok = base is not None and base[0] > 0.01 and base[1] > 0.5 and all(
        got[k] is not None and abs(got[k][0] - base[0]) < 0.01 * base[0] + 1e-3
        and abs(got[k][1] - base[1]) < 0.02 * base[1] + 0.05 for k in (10.0, 0.1))
    return [Result("solver_sop_world_scale_forces", Result.PASS if ok else Result.FAIL,
                   "x moved and degrees turned after 1 s: " + ", ".join(
                       f"World Scale {k}: {got[k][0]:.4f} / {got[k][1]:.3f}" if got[k] else f"{k}: None" for k in got)
                   + " (expected the same in each, both above 0)")]


def test_solver_sop_moving_input():
    """As RBD Bullet Solver SOP, the Geometry and Proxy outputs move the input as of the Start Frame by the pieces' motion
    since: pieces whose input SOP rises 0.1 and turns 3 degrees a frame (a Transform SOP), kinematic (animated) and
    dynamic with no gravity, come out where Bullet puts them (a kinematic piece at y 3 + 0.1 per frame, a dynamic one
    where it started), not moved twice. The pieces are also the Proxy input."""
    rows, ok = [], True
    for label, attrs in (("kinematic", {"active": 0, "animated": 1}), ("dynamic", {"active": 1})):
        got = {}
        for kind, make in (("box3d", solver_sop), ("bullet", bullet_sop)):
            pieces, names = packed_boxes(f"sop_moving_{label}_{kind}",
                                         [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": attrs},
                                          {"size": (1, 1, 1), "center": (3, 3, 0), "attrs": attrs}])
            xf = create(hou.node(pieces).parent(), "xform", "move")
            xf.setInput(0, hou.node(pieces))
            xf.parm("ty").setExpression("($FF - 1) * 0.1", hou.exprLanguage.Hscript)
            xf.parm("ry").setExpression("($FF - 1) * 3", hou.exprLanguage.Hscript)
            display(xf)
            sop = make(f"sop_moving_{label}_{kind}_sop", geometry=xf.path(), proxy=xf.path())
            set_parm(sop, ["gravity"], (0.0, 0.0, 0.0))
            got[kind] = play(sop, 13, lambda s, f: tuple(tuple(named_positions(s.geometry(i))[names[1]])
                                                          for i in (0, 2)) if f == 13 else None)[13]
        want_y = 4.2 if label == "kinematic" else 3.0
        worst = max(abs(a - b) for g, p in zip(got["box3d"], got["bullet"]) for a, b in zip(g, p))
        good = worst < 1e-3 and all(close(g[1], want_y, 1e-3) for g in got["box3d"])
        ok = ok and good
        rows.append(f"{label}: Geometry / Proxy y {[round(g[1], 3) for g in got['box3d']]} on frame 13 (expected "
                    f"{want_y}), largest difference from Bullet {worst:.2e}")
    return [Result("solver_sop_moving_input", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_solver_sop_animated_scale():
    """A kinematic piece whose source grows (scale 1 + 0.1 a frame) is drawn grown by the Solver SOP's Geometry output,
    as RBD Bullet Solver SOP's: the pieces' bounds on frame 25 reach x = -1.7 in both, and the simulation points
    carry `scale` 3.4 for it and 1 for the other piece."""
    from checks_packed import growing_source

    rows, ok = [], True
    for kind, make in (("box3d", solver_sop), ("bullet", bullet_sop)):
        path, names = growing_source(f"sop_grow_{kind}")
        sop = make(f"sop_grow_{kind}_sop", geometry=path)
        set_parm(sop, ["gravity"], (0.0, 0.0, 0.0))

        def sample(s, f):
            if f != 25:
                return None
            points = s.geometry(3)
            scales = {pt.attribValue("name"): pt.attribValue("scale") for pt in points.points()}                 if points.findPointAttrib("scale") else {}
            return s.geometry(0).boundingBox().minvec()[0], scales

        low, scales = play(sop, 25, sample)[25]
        good = close(low, -1.7, 0.01) and scales and close(scales[names[0]][0], 3.4, 1e-3)             and close(scales[names[1]][0], 1.0, 1e-3)
        ok = ok and bool(good)
        rows.append(f"{kind}: Geometry bounds start at x {low:.3f}, point scale {dict((n, round(v[0], 3)) for n, v in scales.items())}")
    return [Result("solver_sop_animated_scale", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected -1.7 and scale 3.4 / 1.0 in both)")]


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
    leaves the Geometry output, and the Proxy output when one is connected, instead of staying behind untransformed.
    With no proxy input the Proxy output is empty."""
    pieces, names = packed_boxes("sop_removed_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"dead": 0}},
                                                        {"size": (1, 1, 1), "center": (3, 3, 0), "attrs": {"dead": 1}}])
    sop = solver_sop("sop_removed_pieces", geometry=pieces)
    set_parm(sop, ["reapdead"], 1)
    track = play(sop, 6, lambda s, f: (len(s.geometry(0).prims()), len(s.geometry(2).prims()),
                                       len(s.geometry(3).points())) if f == 6 else None)
    geo_n, proxy_n, pts_n = track[6]
    # The same pieces as the Proxy: the dead one leaves the proxy output too.
    with_proxy = solver_sop("sop_removed_pieces_proxy", geometry=pieces, proxy=pieces)
    set_parm(with_proxy, ["reapdead"], 1)
    proxy_track = play(with_proxy, 6, lambda s, f: len(s.geometry(2).prims()) if f == 6 else None)
    ok = (geo_n, proxy_n, pts_n) == (1, 0, 1) and proxy_track[6] == 1
    return [Result("solver_sop_removed_pieces", Result.PASS if ok else Result.FAIL,
                   f"on frame 6: {geo_n} geometry piece(s), {proxy_n} proxy piece(s) without a proxy input and "
                   f"{proxy_track[6]} with one, {pts_n} simulation point(s) (expected 1, 0, 1 and 1: the dead piece "
                   f"{names[1]} gone from the geometry, the proxy and the points; no proxy input, no proxy output)")]


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
    primitives (the proxy output is empty in both, there being no proxy input); on frame 12 both have dropped it."""
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
    same_counts = got["box3d"][3][1] == got["bullet"][3][1]
    ok = (all(e is not None and close(e[1], 3.0, 1e-4) for e in early.values())
          and all(l is not None and l[1] < 2.9 for l in late.values()) and same_counts)
    return [Result("solver_sop_before_start_bullet", Result.PASS if ok else Result.FAIL,
                   f"frame 3 (before Start Frame 6): box y {[early[k][1] if early[k] else None for k in got]}, "
                   f"output (points, prims) Box3D {got['box3d'][3][1]}, Bullet {got['bullet'][3][1]}; frame 12 box y "
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


def test_solver_sop_global_substeps():
    """Global Substeps on the Solver tab, just above Substeps: a slider from 1 with no upper limit, default 1,
    and it sets the Substeps of the DOP network inside. Two global substeps give nearly the same height after
    0.5 s as one (within 0.05), and values above 10 can be set."""
    sop = solver_sop("sop_global_substeps")
    names = [pt.name() for pt in sop.parmTemplateGroup().entriesWithoutFolders()]
    pt = sop.parmTemplateGroup().find("globalsubsteps")
    at = names.index("globalsubsteps") if "globalsubsteps" in names else -1
    placed = at >= 0 and names[at + 1] == "substeps" and sop.parm("substeps").parmTemplate().label() == "Substeps"
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
                   f"just above Substeps: {placed}; range 1 to 10, only the minimum strict, default 1: {rng}; sets "
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
    (label hidden, joined to Cache Memory) after Solve on Creation Frame, Emit RBDs and Replicate RBDs and ends with Contacts and Sleeping
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
    sop.parm("glue_dataname").set("Other")
    expect("Glue Data Name reaches the node", glue.parm("dataname").eval() if glue else None, "Other")
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
           ["enableaging", "reapdead", "reapatframeend", "addactiveage", "justactivatedgroup",
            "justactivatedtype"])
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
           ["Stability", "Mass & Inertia", "Performance", "Collision", "Constraints"])
    advanced_collision = by_path.get(("Advanced", "Collision"))
    expect("Advanced / Collision", [pt.name() for pt in advanced_collision.parmTemplates()] if advanced_collision
           else None, ["collision_solvefirstframe"])
    forces_tab = by_path.get(("Forces",))
    expect("Forces tab folders", [pt.label() for pt in forces_tab.parmTemplates() if pt.type() ==
                                  hou.parmTemplateType.Folder] if forces_tab else None,
           ["Gravity", "Force DOPs", "Bounce / Friction", "Glue"])
    glue = by_path.get(("Forces", "Glue"))
    expect("Forces / Glue", [pt.name() for pt in glue.parmTemplates()] if glue else None,
           ["gluecompound", "gluecompoundload", "compoundids"])
    expect("Solver tab folders", [pt.label() for pt in solver_tab.parmTemplates()] if solver_tab else None,
           ["General", "Simulation", "Contacts", "Sleeping"])
    general = by_path.get(("Setup", "General"))
    expect("Solver / General", [pt.name() for pt in general.parmTemplates()] if general else None,
           ["usescale", "scale", "timescale"])
    simulation = by_path.get(("Setup", "Simulation"))
    solver_names = [pt.name() for pt in simulation.parmTemplates()] if simulation else []
    cache = next((pt for pt in simulation.parmTemplates() if pt.name() == "cacheenabled"), None) if simulation else None
    expect("Emit RBDs joined to Replicate RBDs, then Cache Simulation (label hidden, joined) in Solver / Simulation, "
           "after Solve on Creation Frame", (cache.isLabelHidden(), cache.joinsWithNext(),
                                             solver_names[solver_names.index("solvefirstframe"):][:4]) if cache else None,
           (True, True, ["solvefirstframe", "useemission", "replicate", "cacheenabled"]))
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
           else None, ["Break Thresholds", "Override Attributes"])
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


def test_solver_sop_fast_transform():
    """The Geometry output's faster path (one wrangle) against Transform Pieces, for unpacked pieces that fall and tumble
    onto the ground: without normals, with normals, and with a point attribute of vector type, which the wrangle does not
    move, so Transform Pieces must be taken. P, w, v and N must differ by under 1e-5 between the two, and the Switch
    must pick path 1, 2 and 0. Then pieces that carry a shear or a scale: polygons sheared in their SOP (the wrangle
    moves their points as Transform Pieces does: path 1, under 1e-5), and packed pieces (plain, scaled 1.5, sheared),
    whose rotation is their packed transform: the packed wrangle (path 3) must give Transform Pieces' packed transform and
    points to under 1e-5, turned, with the scale and shear kept (a sheared piece tells the order of the product apart),
    and the Geometry output must be the same; packed pieces with a vector point attribute take Transform Pieces."""
    ground, _ = packed_boxes("sop_fast_ground", [GROUND])
    results = []
    for label, normals, extra, want_path in (("no normals", False, "", 1), ("normals", True, "", 2),
                                             ("typed vector", False, 'v@up = {0, 1, 0}; setattribtypeinfo(0, "point", "up", "vector");', 0)):
        tag = label.replace(" ", "_")
        raw = new_geo(f"sop_fast_{tag}")
        box = create(raw, "box", "box")
        set_parm(box, ["t"], (0.3, 2.5, 0.2))
        set_parm(box, ["r"], (25, 10, 35))
        last = box
        if normals:
            nrm = create(raw, "normal", "normals")
            set_parm(nrm, ["type"], 0)  # point normals
            nrm.setInput(0, last)
            last = nrm
        wr = create(raw, "attribwrangle", "extra")  # the piece's name on its points, as the fast path needs
        set_parm(wr, ["class"], "point")
        set_parm(wr, ["snippet"], 's@name = "piece0"; ' + extra)
        wr.setInput(0, last)
        last = wr
        display(last)
        sop = solver_sop(f"sop_fast_{tag}_solver", geometry=last.path(), collision=ground)
        play(sop, 20, lambda s, f: None)
        path = sop.node("switch_fast_transform").parm("input").eval()
        reference = sop.node("transform_geometry").geometry()
        wrangle = sop.node("move_pieces_normals_geometry" if want_path == 2 else "move_pieces_geometry").geometry()
        worst = {}
        for attrib in ("P", "w", "v", "N"):
            if reference.findPointAttrib(attrib) is None or wrangle.findPointAttrib(attrib) is None:
                continue
            for a, b in zip(reference.points(), wrangle.points()):
                va = tuple(a.position()) if attrib == "P" else a.attribValue(attrib)
                vb = tuple(b.position()) if attrib == "P" else b.attribValue(attrib)
                worst[attrib] = max(worst.get(attrib, 0.0), max(abs(x - y) for x, y in zip(va, vb)))
        ok = path == want_path and all(v < 1e-5 for v in worst.values()) and (want_path == 0 or "P" in worst)
        if normals:
            ok = ok and "N" in worst
        results.append(Result("solver_sop_fast_transform", Result.PASS if ok else Result.FAIL,
                              f"{label}: path {path} (expected {want_path}); largest difference from Transform Pieces "
                              f"{ {k: f'{v:.1e}' for k, v in worst.items()} }"))

    # Pieces with a shear or a scale: sheared polygons, and packed pieces (their rotation, scale and shear are the packed
    # transform, which only Transform Pieces moves).
    shear, scale = (0.4, 0.2, 0.3), 1.5
    for label, packed, extra, want_path in (("sheared polygons", False, {"shear": shear}, 1),
                                            ("packed", True, {}, 3),
                                            ("packed, scaled", True, {"scale": scale}, 3),
                                            ("packed, sheared", True, {"shear": shear}, 3),
                                            ("packed, typed vector", True, {"vector": True}, 0)):
        tag = label.replace(", ", "_").replace(" ", "_")
        if packed:
            path_in, _ = packed_boxes(f"sop_fast_{tag}_pieces", [{"size": (1, 1, 1), "center": (0.3, 2.5, 0.2),
                                                                    "rotate": (25, 10, 35)}])
            src = hou.node(path_in)
            last = src
            if extra.get("vector"):
                last = src.parent().createNode("attribwrangle", "typed_vector")
                last.setInput(0, src)
                set_parm(last, ["class"], "point")
                set_parm(last, ["snippet"], 'v@up = {0, 1, 0}; setattribtypeinfo(0, "point", "up", "vector");')
            elif extra:
                last = src.parent().createNode("xform", "packed_xform")
                last.setInput(0, src)
                for parm, value in extra.items():
                    set_parm(last, [parm], value)
            display(last)
        else:
            raw = new_geo(f"sop_fast_{tag}")
            last = create(raw, "box", "box")
            set_parm(last, ["t"], (0.3, 2.5, 0.2))
            set_parm(last, ["r"], (25, 10, 35))
            sheared = create(raw, "xform", "shear")
            sheared.setInput(0, last)
            set_parm(sheared, ["shear"], extra["shear"])
            wr = create(raw, "attribwrangle", "name")
            set_parm(wr, ["class"], "point")
            set_parm(wr, ["snippet"], 's@name = "piece0";')
            wr.setInput(0, sheared)
            display(wr)
            last = wr
        sop = solver_sop(f"sop_fast_{tag}_solver", geometry=last.path(), collision=ground)
        play(sop, 20, lambda s, f: None)
        path = sop.node("switch_fast_transform").parm("input").eval()
        reference = sop.node("transform_geometry").geometry()
        result = sop.geometry(0)
        if packed:
            # The output's packed transform against Transform Pieces' (the 3x3 turned by the fall, scale and shear in it).
            ref_t = reference.prims()[0].intrinsicValue("transform")
            out_t = result.prims()[0].intrinsicValue("transform")
            diff = max(abs(a - b) for a, b in zip(ref_t, out_t))
            identity = max(abs(a - b) for a, b in zip(out_t, (1, 0, 0, 0, 1, 0, 0, 0, 1)))
            turned = max(abs(a - b) for a, b in zip(ref_t, (scale if "scaled" in label else 1.0, 0, 0, 0,
                                                            scale if "scaled" in label else 1.0, 0, 0, 0,
                                                            scale if "scaled" in label else 1.0))) > 0.05
            moved = sop.node("move_pieces_packed_geometry").geometry()
            wrangle_t = max(abs(a - b) for a, b in zip(ref_t, moved.prims()[0].intrinsicValue("transform")))
            wrangle_p = max(abs(a - b) for a, b in zip(reference.points()[0].position(), moved.points()[0].position()))
            ok = path == want_path and diff < 1e-5 and turned and (want_path == 0 or max(wrangle_t, wrangle_p) < 1e-5)
            text = (f"output's packed transform differs from Transform Pieces' by {diff:.1e}, the packed wrangle's by "
                    f"{wrangle_t:.1e} (point {wrangle_p:.1e}), the output's from the input's unturned one by {identity:.2f}")
        else:
            worst = 0.0
            for attrib in ("P", "w", "v"):
                for a, b in zip(reference.points(), sop.node("move_pieces_geometry").geometry().points()):
                    va = tuple(a.position()) if attrib == "P" else a.attribValue(attrib)
                    vb = tuple(b.position()) if attrib == "P" else b.attribValue(attrib)
                    worst = max(worst, max(abs(x - y) for x, y in zip(va, vb)))
            ok = path == want_path and worst < 1e-5
            text = f"largest difference from Transform Pieces {worst:.1e}"
        results.append(Result("solver_sop_fast_transform", Result.PASS if ok else Result.FAIL,
                              f"{label}: path {path} (expected {want_path}); {text}"))
    return results


def test_solver_sop_pop_drag_bullet():
    """POP Drag (airresist) and POP Speed Limit (speedmax) in the FORCES subnet's PRESOLVE, in both Solver SOPs: a
    box falling freely ends at the speed where the drag is its weight, and with a speed limit at that limit. POP Drag
    writes `airresist` every step (Bullet zeroes it after each solve), so this is the path a POP network takes. The
    speeds agree within 8 percent (each applies the drag as one implicit step of its own Substeps; Box3D clamps after
    its step: 0.2 m/s)."""
    results = []
    for label, node_type, parms, want in (("POP Drag", "popdrag", {"airresist": 2.0}, 2.1),
                                         ("POP Speed Limit", "popspeedlimit", {"dospeedmax": 1, "speedmax": 3.0}, 3.0)):
        speeds = {}
        for kind, factory in (("box3d", solver_sop), ("bullet", bullet_sop)):
            pieces, names = packed_boxes(f"sop_pop_{node_type}_{kind}_pieces",
                                         [{"size": (1, 1, 1), "center": (0, 1000, 0)}])
            sop = factory(f"sop_pop_{node_type}_{kind}", geometry=pieces)
            forces = sop.node("dopnet/FORCES") or sop.node("dopnet/forces")  # Bullet's is lower case
            out = forces.node("PRESOLVE") if forces is not None else None
            if out is None:
                results.append(Result("solver_sop_pop_drag_bullet", Result.INFO, f"{kind}: no dopnet/FORCES/PRESOLVE"))
                break
            pop = create(forces, node_type, "limit")
            for parm, value in parms.items():
                set_parm(pop, [parm], value)
            out.setInput(0, pop)
            ys = play(sop, 72, lambda s, f: named_positions(s.geometry(0)).get(names[0]))
            speeds[kind] = abs(ys[72][1] - ys[71][1]) * hou.fps()
        if len(speeds) < 2:
            continue
        a, b = speeds["box3d"], speeds["bullet"]
        ok = abs(a - b) < 0.08 * b + 0.2 and a < 5.0
        results.append(Result("solver_sop_pop_drag_bullet", Result.PASS if ok else Result.FAIL,
                              f"{label} on PRESOLVE: speed after 3 s Box3D {a:.2f}, Bullet {b:.2f} m/s (about {want:g} "
                              "expected; a free fall is 29)"))
    return results


def _emit_sample(s, f):
    """One frame of an emitting Solver SOP: the simulation points (position, v), the Geometry and constraint counts."""
    return {"points": {p.attribValue("name"): (p.position(), p.attribValue("v")) for p in s.geometry(3).points()},
            "geometry": len(s.geometry(0).prims()), "constraints": len(s.geometry(1).prims()),
            "moved": named_positions(s.geometry(0)),
            "proxy": len(s.geometry(2).prims())}


def _emit_differences(box3d, bullet, names, frames):
    """(problems, summary) of two emitting runs played `frames` frames: the same point names, positions within 0.05, the
    same constraint count, one Geometry piece per point in Box3D's, an emitted copy that keeps its `v` (x 30) and
    `2 * frames + 1` points in the end."""
    problems = []
    for f in range(1, frames + 1):
        a, b = box3d[f], bullet[f]
        if sorted(a["points"]) != sorted(b["points"]):
            problems.append(f"frame {f} names Box3D {sorted(a['points'])}, Bullet {sorted(b['points'])}")
            continue
        far = [n for n in a["points"] if (a["points"][n][0] - b["points"][n][0]).length() > 0.05]
        if far:
            problems.append(f"frame {f}: {far} more than 0.05 from Bullet's")
        if a["constraints"] != b["constraints"]:
            problems.append(f"frame {f} constraints Box3D {a['constraints']}, Bullet {b['constraints']}")
        if a["geometry"] != len(a["points"]):
            problems.append(f"frame {f} Box3D geometry has {a['geometry']} pieces for {len(a['points'])} points")
    last = box3d[frames]["points"]
    emitted = f"{names[0]}_{frames * 100}"
    v = last.get(emitted, (None, None))[1]
    if v is None or abs(v[0] - 30.0) > 1e-3:
        problems.append(f"{emitted}'s v {v}, expected x 30")
    want = 2 * frames + 1
    if len(last) != want:
        problems.append(f"{len(last)} points on frame {frames}, expected {want}")
    return problems, f"{len(last)} points and {box3d[frames]['constraints']} constraints on frame {frames}, as Bullet's"


def test_solver_sop_emit_scale_bullet():
    """Emit RBDs with World Scale, against the RBD Bullet Solver SOP at the same World Scale: the scene of
    solver_sop_emit_bullet (Solve on Creation Frame off) at World Scale 10 and 0.1. The results are in scene units, so
    they must be what the same checks find there: the same point names, constraint counts and positions within 0.05
    as Bullet's, and an emitted copy that keeps its `v`."""
    pieces, names = packed_boxes("sop_emit_scale_pieces", [
        {"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"v": (30, 0, 0), "rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 6, 0), "attrs": {"v": (30, 0, 0), "rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 5, 5), "attrs": {"rbdbullet_emit": 0}}])
    lines = constraint_lines("sop_emit_scale_network", [(names[0], (0, 5.5, 0), names[1], (0, 5.5, 0), "Glue")])
    frames = 5
    results = []
    for k in (10.0, 0.1):
        tag = str(k).replace(".", "_")
        got = {}
        for kind, sop in _both_sops(f"sop_emit_scale_{tag}", geometry=pieces, constraints=lines).items():
            set_parm(sop, ["useemission"], 1)
            set_parm(sop, ["usescale"], 1)
            set_parm(sop, ["scale"], k)
            got[kind] = play(sop, frames, _emit_sample)
        problems, summary = _emit_differences(got["box3d"], got["bullet"], names, frames)
        results.append(Result("solver_sop_emit_scale_bullet", Result.FAIL if problems else Result.PASS,
                              f"World Scale {k:g}: " + ("; ".join(problems) if problems else summary)))
    return results


def test_solver_sop_emit_bullet():
    """Emit RBDs, against the RBD Bullet Solver SOP, with Solve on Creation Frame off and on: two glued boxes that emit,
    thrown sideways by their `v` fast enough to clear the spot before the next copy appears there, and one with
    `rbdbullet_emit` 0, played 5 frames. Both give the same simulation
    point names every frame (a copy per frame, `<name>_<frame * 100>`, the non-emitting box once), the same number of
    constraints and positions within 0.05; an emitted copy keeps its `v`. The Geometry output has one piece per point
    (Bullet's has one more with Solve on Creation Frame, so that is not compared). With a Proxy input too, the Geometry
    and Proxy outputs of Box3D have one piece per point, each Geometry piece where its point is."""
    pieces, names = packed_boxes("sop_emit_pieces", [
        {"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"v": (30, 0, 0), "rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 6, 0), "attrs": {"v": (30, 0, 0), "rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 5, 5), "attrs": {"rbdbullet_emit": 0}}])
    lines = constraint_lines("sop_emit_network", [(names[0], (0, 5.5, 0), names[1], (0, 5.5, 0), "Glue")])
    frames = 5
    results = []

    for first in (0, 1):
        sops = _both_sops(f"sop_emit_{first}", geometry=pieces, constraints=lines)
        got = {}
        for kind, sop in sops.items():
            set_parm(sop, ["useemission"], 1)
            set_parm(sop, ["solvefirstframe"], first)
            got[kind] = play(sop, frames, _emit_sample)
        problems, summary = _emit_differences(got["box3d"], got["bullet"], names, frames)
        results.append(Result("solver_sop_emit_bullet", Result.FAIL if problems else Result.PASS,
                              f"Solve on Creation Frame {first}: " + ("; ".join(problems) if problems else summary)))

    sop = solver_sop("sop_emit_proxy", geometry=pieces, constraints=lines, proxy=pieces)
    set_parm(sop, ["useemission"], 1)
    track = play(sop, frames, _emit_sample)
    bad = [f"frame {f}: {track[f]['geometry']} geometry, {track[f]['proxy']} proxy pieces for "
           f"{len(track[f]['points'])} points" for f in track
           if not track[f]["geometry"] == track[f]["proxy"] == len(track[f]["points"])]
    # Each Geometry piece where its simulation point is (packed boxes: the piece's point is its centre).
    bad += [f"frame {f}: {n} at {track[f]['moved'].get(n)}, its point at {p[0]}" for f in track
            for n, p in track[f]["points"].items()
            if n not in track[f]["moved"] or (track[f]["moved"][n] - p[0]).length() > 1e-3]
    results.append(Result("solver_sop_emit_bullet", Result.FAIL if bad else Result.PASS,
                          "with a Proxy input: " + ("; ".join(bad) if bad else
                                                    "Geometry and Proxy outputs have one piece per point, "
                                                    "each where its point is")))

    # A glued pair that emits, one of them also held by a Hard constraint to a box that is not emitted, the pair
    # moving 3 along x a frame so each copy appears clear of the last. Constraint names per frame as Bullet's: a copy
    # brings the glue between the two, not the Hard one (it stays with the first frame's pieces). Again with Overwrite
    # with SOP left on, which emission turns off in both (it would drop the emitted glue). (A Hard constraint to the
    # world is left out: Bullet drops one on a glued piece after the first frame, emitting or not.)
    anchored, (box, mate, kept) = packed_boxes("sop_emit_anchored_pieces", [
        {"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 6, 0), "attrs": {"rbdbullet_emit": 1}},
        {"size": (1, 1, 1), "center": (0, 5, -3), "attrs": {"rbdbullet_emit": 0}}])
    anchor_lines = constraint_lines("sop_emit_anchored_network", [
        (box, (0, 5.5, 0), mate, (0, 5.5, 0), "Glue"),
        (box, (0, 5, -1.5), kept, (0, 5, -1.5), "Hard")])
    moving = []
    for path in (anchored, anchor_lines):
        src = hou.node(path)
        mover = src.parent().createNode("attribwrangle", "move_emitter")
        mover.setInput(0, src)
        mover.parm("class").set(2)  # points
        mover.parm("snippet").set(f'if (s@name != "{kept}") @P.x += 3 * @Frame;')
        display(mover)
        moving.append(mover.path())

    def constraint_names(s, f):
        g = s.geometry(1)
        return sorted(tuple(sorted(p.attribValue("name") for p in pr.points())) for pr in g.prims())

    for reload in (0, 1):
        got = {}
        for kind, sop in _both_sops(f"sop_emit_anchored_{reload}", geometry=moving[0],
                                    constraints=moving[1]).items():
            set_parm(sop, ["useemission"], 1)
            set_parm(sop, ["constraint_reloadfromsops"], reload)
            got[kind] = play(sop, 4, constraint_names)
        differ = [f for f in range(1, 5) if got["box3d"][f] != got["bullet"][f]]
        glued = sum(1 for pair in got["box3d"][4] if kept not in pair)
        ok = not differ and glued == 4
        results.append(Result("solver_sop_emit_bullet", Result.PASS if ok else Result.FAIL,
                              f"Overwrite with SOP {reload}, an emitting glued pair held to a piece that is not "
                              f"emitted: constraints differ from Bullet's on frames {differ}, "
                              f"{glued} glue constraints on frame 4 (expected none differ and 4); "
                              f"Box3D's on frame 4: {got['box3d'][4]}"
                              + (f", Bullet's: {got['bullet'][4]}" if differ else "")))
    return results


def _duplicate_in_postsolve(sop, forces, name, frame, offset):
    """A SOP Solver on the FORCES subnet's POSTSOLVE (`forces`: its name in the SOP's DOP network) that copies the
    piece `name` once, on `frame`, moved by `offset`: a piece duplicated during the simulation."""
    sop.allowEditingOfContents()
    subnet = sop.node(f"dopnet/{forces}")
    solver = subnet.createNode("sopsolver", "duplicate")
    out = [c for c in solver.children() if c.type().name() == "output"][0]
    src = out.inputs()[0]
    pick = solver.createNode("blast", "pick")
    pick.setInput(0, src)
    set_parm(pick, ["group"], f"@name={name}")
    set_parm(pick, ["grouptype"], "points")
    set_parm(pick, ["negate"], 1)
    move = solver.createNode("attribwrangle", "move")
    move.setInput(0, pick)
    set_parm(move, ["snippet"], "v@P += {%g, %g, %g};" % offset)
    both = solver.createNode("merge", "with_copy")
    both.setInput(0, src)
    both.setInput(1, move)
    once = solver.createNode("switch", "once")
    once.setInput(0, src)
    once.setInput(1, both)
    once.parm("input").setExpression(f'$F == {frame} && npoints("../pick") == 1', hou.exprLanguage.Hscript)
    out.setInput(0, once)
    subnet.node("POSTSOLVE").setInput(0, solver)


def _piece_centres(geo):
    """{name: centre of its bounding box} of the pieces of `geo`, named on their primitives, else their points."""
    prim_name = geo.findPrimAttrib("name") is not None
    points = {}
    for prim in geo.prims():
        name = prim.attribValue("name") if prim_name else prim.vertices()[0].point().attribValue("name")
        # (A packed piece's one point is its centre.)
        points.setdefault(name, []).extend(v.point().position() for v in prim.vertices())
    return {n: hou.Vector3([(min(p[i] for p in ps) + max(p[i] for p in ps)) / 2 for i in range(3)])
            for n, ps in points.items()}


def _replicate_sample(s, f):
    """One frame of a replicating Solver SOP: {point name: position} of the simulation points and {name: centre} of the
    Geometry pieces."""
    return {"points": {p.attribValue("name"): p.position() for p in s.geometry(3).points()},
            "pieces": _piece_centres(s.geometry(0))}


def test_solver_sop_replicate_scale_bullet():
    """Replicate RBDs with World Scale, against the RBD Bullet Solver SOP at the same World Scale: the scene of
    solver_sop_replicate_bullet at World Scale 10 and 0.1. Both name the copy `<name>_r1`, and the copy is at the
    same place in both, as is the box that was not copied (within 0.05 of Bullet's, in scene units). The SOP Solver
    works in the solver's units (a scene length times World Scale), so it moves the copy by 3 x World Scale to put it 3
    aside in the scene. In Box3D the copy's Geometry piece is where its point is."""
    pieces, names = packed_boxes("sop_replicate_scale_pieces", [{"size": (1, 1, 1), "center": (0, 5, 0)},
                                                                {"size": (1, 1, 1), "center": (3, 5, 0)}])
    copy = f"{names[0]}_r1"
    frames = 5
    results = []
    for k in (10.0, 0.1):
        tag = str(k).replace(".", "_")
        got = {}
        for kind, sop in _both_sops(f"sop_replicate_scale_{tag}", geometry=pieces).items():
            set_parm(sop, ["replicate"], 1)
            set_parm(sop, ["usescale"], 1)
            set_parm(sop, ["scale"], k)
            _duplicate_in_postsolve(sop, "FORCES" if kind == "box3d" else "forces", names[0], 3, (0, 0, 3 * k))
            got[kind] = play(sop, frames, _replicate_sample)[frames]
        a, b = got["box3d"], got["bullet"]
        problems = []
        if sorted(a["points"]) != sorted([names[0], names[1], copy]) or sorted(a["points"]) != sorted(b["points"]):
            problems.append(f"points Box3D {sorted(a['points'])}, Bullet {sorted(b['points'])}")
        else:
            problems += [f"{n} at {a['points'][n]} in Box3D, {b['points'][n]} in Bullet" for n in (copy, names[1])
                         if (a["points"][n] - b["points"][n]).length() > 0.05]
            if copy not in a["pieces"] or (a["pieces"][copy] - a["points"][copy]).length() > 1e-3:
                problems.append(f"{copy}'s piece at {a['pieces'].get(copy)}, its point at {a['points'][copy]}")
        results.append(Result("solver_sop_replicate_scale_bullet", Result.FAIL if problems else Result.PASS,
                              f"World Scale {k:g}: " + ("; ".join(problems) if problems else
                                                        f"{copy} at {a['points'][copy]} in Box3D, {b['points'][copy]} in "
                                                        "Bullet, with a Geometry piece at its point")))
    return results


def test_solver_sop_replicate_bullet():
    """Replicate RBDs, against the RBD Bullet Solver SOP: a SOP Solver in FORCES' POSTSOLVE copies one of two boxes on
    frame 3, 3 units aside. With Replicate RBDs on, both name the copy's simulation point `<name>_r1` and give it a
    Geometry piece of its own, where its point is; off, Box3D's copy has none (as Bullet's). Then Box3D alone with
    unpacked Geometry named on its primitives: the copy's piece is made from the polygons the same way. (Bullet's
    original stops moving once copied, so only the copy is compared for position.)"""
    pieces, names = packed_boxes("sop_replicate_pieces", [{"size": (1, 1, 1), "center": (0, 5, 0)},
                                                          {"size": (1, 1, 1), "center": (3, 5, 0)}])
    copy = f"{names[0]}_r1"
    frames = 5
    results = []
    sample = _replicate_sample

    got = {}
    for replicate in (0, 1):
        sops = _both_sops(f"sop_replicate_{replicate}", geometry=pieces)
        for kind, sop in sops.items():
            set_parm(sop, ["replicate"], replicate)
            _duplicate_in_postsolve(sop, "FORCES" if kind == "box3d" else "forces", names[0], 3, (0, 0, 3))
            got[(kind, replicate)] = play(sop, frames, sample)
    problems = []
    for kind in ("box3d", "bullet"):
        last = got[(kind, 1)][frames]
        if sorted(last["points"]) != sorted([names[0], names[1], copy]):
            problems.append(f"{kind} points {sorted(last['points'])}")
        elif sorted(last["pieces"]) != sorted(last["points"]):
            problems.append(f"{kind} Geometry pieces {sorted(last['pieces'])}")
        elif (last["pieces"][copy] - last["points"][copy]).length() > 1e-3:
            problems.append(f"{kind} {copy}'s piece at {last['pieces'][copy]}, its point at {last['points'][copy]}")
    b, u = got[("box3d", 1)][frames], got[("bullet", 1)][frames]
    if copy in b["points"] and copy in u["points"] and (b["points"][copy] - u["points"][copy]).length() > 0.05:
        problems.append(f"{copy} at {b['points'][copy]} in Box3D, {u['points'][copy]} in Bullet")
    off = got[("box3d", 0)][frames]["pieces"]
    if len(off) != 2:
        problems.append(f"Replicate off: Box3D Geometry pieces {sorted(off)}, expected the two of the input")
    results.append(Result("solver_sop_replicate_bullet", Result.FAIL if problems else Result.PASS,
                          "; ".join(problems) if problems else
                          f"{copy} has its own Geometry piece at its point in both, none with Replicate RBDs off"))

    # Unpacked Geometry named on its primitives (the Solver SOP packs it for the simulation).
    geo = new_geo("sop_replicate_polys")
    merge = create(geo, "merge", "boxes")
    for i, x in enumerate((0, 3)):
        box = create(geo, "box", f"box{i}")
        set_parm(box, ["t"], (x, 5, 0))
        merge.setInput(i, box)
    name = create(geo, "attribwrangle", "name")
    name.setInput(0, merge)
    set_parm(name, ["class"], 1)
    set_parm(name, ["snippet"], 's@name = "piece" + itoa(i@primnum / 6);')
    display(name)
    sop = solver_sop("sop_replicate_polys_box3d", geometry=name.path())
    set_parm(sop, ["replicate"], 1)
    _duplicate_in_postsolve(sop, "FORCES", "piece0", 3, (0, 0, 3))
    last = play(sop, frames, sample)[frames]
    polys = len(sop.geometry(0).prims())
    ok = (sorted(last["pieces"]) == sorted(last["points"]) == ["piece0", "piece0_r1", "piece1"] and polys == 18
          and (last["pieces"]["piece0_r1"] - last["points"]["piece0_r1"]).length() < 1e-3)
    results.append(Result("solver_sop_replicate_bullet", Result.PASS if ok else Result.FAIL,
                          f"unpacked Geometry: pieces {sorted(last['pieces'])}, {polys} polygons (expected 18), points "
                          f"{sorted(last['points'])}; piece0_r1's piece at {last['pieces'].get('piece0_r1')}, its "
                          f"point at {last['points'].get('piece0_r1')}"))
    return results


TESTS = {
    "solver_sop_replicate_scale_bullet": test_solver_sop_replicate_scale_bullet,
    "solver_sop_emit_scale_bullet": test_solver_sop_emit_scale_bullet,
    "solver_sop_replicate_bullet": test_solver_sop_replicate_bullet,
    "solver_sop_emit_bullet": test_solver_sop_emit_bullet,
    "solver_sop_quadrics": test_solver_sop_quadrics,
    "solver_sop_pieces": test_solver_sop_pieces,
    "solver_sop_global_substeps": test_solver_sop_global_substeps,
    "solver_sop_before_start_bullet": test_solver_sop_before_start_bullet,
    "solver_sop_solve_first_frame_bullet": test_solver_sop_solve_first_frame_bullet,
    "solver_sop_before_start": test_solver_sop_before_start,
    "solver_sop_solve_first_frame": test_solver_sop_solve_first_frame,
    "solver_sop_rest_anchors": test_solver_sop_rest_anchors,
    "solver_sop_initial_bake": test_solver_sop_initial_bake,
    "solver_sop_removed_pieces": test_solver_sop_removed_pieces,
    "solver_sop_world_scale": test_solver_sop_world_scale,
    "solver_sop_world_scale_forces": test_solver_sop_world_scale_forces,
    "solver_sop_moving_input": test_solver_sop_moving_input,
    "solver_sop_animated_scale": test_solver_sop_animated_scale,
    "solver_sop_animated_setting": test_solver_sop_animated_setting,
    "solver_sop_fast_transform": test_solver_sop_fast_transform,
    "solver_sop_impacts": test_solver_sop_impacts,
    "solver_sop_forces": test_solver_sop_forces,
    "solver_sop_pop_drag_bullet": test_solver_sop_pop_drag_bullet,
    "solver_sop_layout": test_solver_sop_layout,
    "solver_sop_proxy": test_solver_sop_proxy,
}
