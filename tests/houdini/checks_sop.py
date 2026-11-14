"""The Box3D Solver SOP (tools/make_solver_sop.py): pieces falling onto a collision input, an
animated collider lifting a piece, POP forces in FORCES, a proxy driving the geometry through Transform Pieces, and the
constraint output; Box3D Configure."""

import hou

from b3dtest import (
    Result,
    close,
    constraint_lines,
    create,
    display,
    new_geo,
    packed_boxes,
    set_parm,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}


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
    the collision output is the input, unchanged, and the proxy output equals the geometry output.
    With Use External Collisions off the box falls through the ground."""
    pieces, names = packed_boxes("sop_drop_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    ground, _ = packed_boxes("sop_drop_ground", [GROUND])
    sop = solver_sop("sop_drop", geometry=pieces, collision=ground)

    def sample(s, f):
        if f != 48:
            return None
        return (named_positions(s.geometry(0)), named_positions(s.geometry(2)), s.geometry(3).boundingBox())

    geo_pos, proxy_pos, col_bb = play(sop, 48, sample)[48]
    y = geo_pos.get(names[0], hou.Vector3(0, 99, 0))[1]
    same = names[0] in proxy_pos and (proxy_pos[names[0]] - geo_pos[names[0]]).length() < 1e-6
    col_ok = close(col_bb.maxvec()[1], 0.0, 1e-6) and close(col_bb.minvec()[1], -1.0, 1e-6)
    ok = close(y, 0.5, 0.03) and same and col_ok
    results = [Result("solver_sop_collision", Result.PASS if ok else Result.FAIL,
                      f"box at y={y:.3f} after 2 s (expected 0.5); proxy output equals geometry output: {same}; "
                      f"collision output y range {col_bb.minvec()[1]:.3f}..{col_bb.maxvec()[1]:.3f} (expected -1..0)")]

    # Use External Collisions off: the Collision input is ignored and the box falls through.
    off = solver_sop("sop_drop_off", geometry=pieces, collision=ground)
    set_parm(off, ["usecollisions"], 0)
    pos = play(off, 48, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f == 48 else None)[48]
    y_off = pos[1] if pos else None
    results.append(Result("solver_sop_collision", Result.PASS if y_off is not None and y_off < -1.0 else Result.FAIL,
                          f"Use External Collisions off: box at y={y_off} after 2 s (expected below -1: fell through)"))
    return results


def test_solver_sop_animated_collision():
    """A ground box on the Collision input rising 0.5 m/s (keyed on its Transform SOP) lifts the box
    resting on it: with Animated Collision on (the default) the box ends 1 m higher after 2 s; off,
    the ground stays where it started and the box stays at y = 0.5."""
    results = []
    for animated in (1, 0):
        tag = "on" if animated else "off"
        pieces, names = packed_boxes(f"sop_lift_pieces_{tag}", [{"size": (1, 1, 1), "center": (0, 0.5, 0)}])
        ground, _ = packed_boxes(f"sop_lift_ground_{tag}", [GROUND])
        place = hou.node(ground).parent().node("place0")
        place.parm("ty").setExpression("-0.5 + 0.5 * $T", hou.exprLanguage.Hscript)
        sop = solver_sop(f"sop_lift_{tag}", geometry=pieces, collision=ground)
        set_parm(sop, ["animatedcollision"], animated)
        track = play(sop, 49, lambda s, f: named_positions(s.geometry(0)).get(names[0]) if f in (1, 49) else None)
        y0 = track[1][1] if track[1] else None
        y1 = track[49][1] if track[49] else None
        want = 1.5 if animated else 0.5
        ok = y1 is not None and close(y1, want, 0.05)
        results.append(Result("solver_sop_animated_collision", Result.PASS if ok else Result.FAIL,
                              f"Animated Collision {tag}: box y {y0} on frame 1, {y1} on frame 49 "
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


def test_configure():
    """Box3D Configure (tools/make_configure_sop.py) writes ticked settings onto the pieces in its
    group only: Body Type Static and Friction 0.25 on one of two boxes. Into the Box3D Solver SOP,
    that box stays put and the other falls (it keeps active 1, not the 0 VEX would give it)."""
    pieces, names = packed_boxes("cfg_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, src)
    set_parm(cfg, ["group"], f"@name={names[0]}")
    for n, v in (("set_bodytype", 1), ("bodytype", 0), ("set_friction", 1), ("friction", 0.25)):
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


TESTS = {
    "solver_sop_collision": test_solver_sop_collision,
    "solver_sop_animated_collision": test_solver_sop_animated_collision,
    "solver_sop_forces": test_solver_sop_forces,
    "solver_sop_proxy": test_solver_sop_proxy,
    "solver_sop_constraints": test_solver_sop_constraints,
    "configure": test_configure,
}
