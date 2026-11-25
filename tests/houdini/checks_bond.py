"""Box3D Bond checks beyond the statics in checks_more.py (bond_hang, bond_chain, bond_benchmarks, ...): the primitive
attributes that modify a Bond, a rotated or spinning piece, a Bond to the world, friction, impacts, scrubbing and
sleeping, compound mode against step size, long runs, and the Box3D Solver SOP with World Scale. Every expected value
is worked out from the formulas in the test's docstring, never read off the simulation."""

import itertools

import hou

from b3dtest import (
    GROUND,
    constraint_lines,
    create,
    finish,
    find_network_geometry,
    merge,
    multisolver,
    new_dopnet,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_parm,
    solver_for,
    sop_solver,
    step,
    threshold,
)
from checks_constraints import broken_count

_COUNTER = itertools.count()
WEIGHT = 9806.65  # N of a 1 m box of 1000 kg (the default density)


def bond_scene(tag, pieces, lines, bond=None, compound=None, keep_velocity=False, substeps=None, gravity=None,
               packed_parms=None, extra_nodes=None):
    """A DOP network of packed boxes joined by Box3D Bond lines. pieces: as packed_boxes. lines: (piece index or None
    for the world, point, piece index or None, point[, {primitive attribute: value}[, constraint name]]) each, the
    constraint name being Bond unless given. bond: the Bond node's parameters. extra_nodes: [(node type, parameters)]
    wired in beside the Bond node (a Weld or Glue for lines that name one). Returns (dop, piece names)."""
    dop = new_dopnet(tag)
    path, names = packed_boxes(tag + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path, keep_velocity=keep_velocity, **(packed_parms or {}))
    node = create(dop, "box3d_bondconrel", "bond")
    for parm, value in (bond or {}).items():
        set_parm(node, [parm], value)
    spec = []
    for line in lines:
        a, pa, b, pb = line[:4]
        name = line[5] if len(line) > 5 else "Bond"
        entry = ("" if a is None else names[a], pa, "" if b is None else names[b], pb, name)
        spec.append(entry + ((line[4] or {},) if len(line) > 4 else ()))
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(tag + "_network", spec))
    nodes = [node]
    for i, (type_name, parms) in enumerate(extra_nodes or []):
        extra = create(dop, type_name, f"extra{i}")
        for parm, value in parms.items():
            set_parm(extra, [parm], value)
        nodes.append(extra)
    net.setInput(1, node if len(nodes) == 1 else merge(dop, nodes, "joints"))
    solver = solver_for("box3d", dop, net, **({} if gravity is None else {"gravity": gravity}))
    if compound is not None:
        set_parm(solver, ["gluecompound"], 1 if compound else 0)
    if substeps is not None:
        dop.parm("substep").set(substeps)
    finish(dop, solver)
    return dop, names


def first_broken(dop, frames):
    """The first frame (1 to `frames`) with a broken line, or None."""
    for f in range(1, frames + 1):
        hou.setFrame(f)
        sim = dop.simulation()
        sim.objects()
        if (broken_count(sim) or 0) >= 1:
            return f
    return None


def tag(prefix):
    return f"{prefix}_{next(_COUNTER)}"


TESTS = {}


# ---------------------------------------------------------------------------
# What modifies a Bond, and where its pieces are
# ---------------------------------------------------------------------------

HANG = [{"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"active": 0}},
        {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 1}}]
AT = (0.0, 4.5, 0.0)  # the bond's point, between the two boxes
LINE = (0, AT, 1, AT)


def test_bond_attributes():
    """The primitive attributes that modify a Bond, as the Constraint Network DOP documents: a float multiplies the
    node's value, with Attributes Replace Values on it sets it. The hanging 1000 kg box (1 m^2, 9.8 kPa): a Tensile
    Strength of 4000 breaks the bond, an attribute `bond_tensile` of 4 (16000) holds it and 0.5 (2000) does not; with
    Replace, 20000 holds it and 5000 does not. `bond_area` (node 1 m^2, strength 5e4): x0.1 (98 kPa) breaks it, x1
    holds it. `bond_fracture_energy` (node 1 J/m^2, which tears it within 6 frames) x1000 holds it for them."""
    def broken(parms, attrs, frames=24):
        dop, _names = bond_scene(tag("bond_attr"), HANG, [LINE + (attrs,)], bond=parms)
        return first_broken(dop, frames) is not None

    base = {"bond_tensile": 4000.0, "bond_shear": 4000.0}
    replace = dict(base, attributes_replace=1)
    area = {"bond_tensile": 5e4, "bond_shear": 5e4, "bond_area": 1.0}
    cases = [("node 4000", base, {}, True), ("x4", base, {"bond_tensile": 4.0}, False),
             ("x0.5", base, {"bond_tensile": 0.5}, True), ("replace 20000", replace, {"bond_tensile": 20000.0}, False),
             ("replace 5000", replace, {"bond_tensile": 5000.0}, True), ("area 1", area, {}, False),
             ("area x0.1", area, {"bond_area": 0.1}, True),
             ("area replace 0.1", dict(area, attributes_replace=1), {"bond_area": 0.1}, True)]
    rows, ok = [], True
    for label, parms, attrs, expect in cases:
        got = broken(parms, attrs)
        ok = ok and got == expect
        rows.append(f"{label}: {'breaks' if got else 'holds'}{'' if got == expect else ' (WRONG)'}")
    fracture = {"bond_tensile": 4000.0, "bond_shear": 4000.0, "bond_fracture_energy": 1.0}
    torn = broken(fracture, {}, frames=6)
    held = not broken(fracture, {"bond_fracture_energy": 1000.0}, frames=6)
    ok = ok and torn and held
    rows.append(f"fracture energy 1: {'breaks' if torn else 'holds'}, x1000: {'holds' if held else 'breaks'}")
    return [Result("bond_attributes", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_attributes"] = test_bond_attributes


def test_bond_geometry():
    """Where a Bond's pieces are. (1) The box hanging under the fixed one turned 45 degrees about the vertical: the
    faces overlap in an octagon of 2 * (sqrt(2) - 1) = 0.828 m^2, so the bond lets go at 9807 / 0.828 = 11.8 kPa (5%).
    (2) A bond from the world (an empty name) to a hanging box has no second piece's faces: the area comes from the box
    alone, 1 m^2, so 9.8 kPa (5%)."""
    rows, ok = [], True

    def let_go(pieces, lines):
        def breaks(strength):
            dop, _names = bond_scene(tag("bond_geo"), pieces, lines,
                                     bond={"bond_tensile": strength, "bond_shear": strength})
            return first_broken(dop, 24) is not None
        return threshold(breaks, 1e3, 1e6, rounds=10)

    turned = [HANG[0], {"size": (1, 1, 1), "center": (0, 4, 0), "rotate": (0, 45, 0), "attrs": {"active": 1}}]
    expected = WEIGHT / (2 * (2 ** 0.5 - 1))
    found = let_go(turned, [LINE])
    good = found is not None and abs(found - expected) < 0.05 * expected
    ok = ok and good
    rows.append(f"turned 45 degrees: lets go at {found and round(found)} Pa, formula {expected:.0f} "
                f"({'ok' if good else 'wrong'})")
    found = let_go([HANG[1]], [(None, AT, 0, AT)])
    good = found is not None and abs(found - WEIGHT) < 0.05 * WEIGHT
    ok = ok and good
    rows.append(f"from the world: lets go at {found and round(found)} Pa, formula {WEIGHT:.0f} "
                f"({'ok' if good else 'wrong'})")
    return [Result("bond_geometry", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_geometry"] = test_bond_geometry


# ---------------------------------------------------------------------------
# Rotation, and the step size, in compound mode and out of it
# ---------------------------------------------------------------------------


def test_bond_spin():
    """Two 1000 kg boxes bonded and spinning about their common centre at w = 10 rad/s, falling freely (gravity cancels):
    each needs m * w^2 * r = 1000 * 100 * 0.5 = 50 kN from the bond, over 1 m^2, so it lets go at 50 kPa. Within 3% as
    welds and as a compound, at 24 and 120 frames per second (a compound has no joint to read, so its load is
    worked out from its motion; one of its poses a step behind gave 20% more once). Also a spin of 5 rad/s
    (12.5 kPa) at 24 frames per second."""
    original = hou.fps()
    rows, ok = [], True

    def pair(w):
        return [{"size": (1, 1, 1), "center": (-0.5, 12, 0), "attrs": {"v": (0.0, -0.5 * w, 0.0), "w": (0.0, 0.0, w)}},
                {"size": (1, 1, 1), "center": (0.5, 12, 0), "attrs": {"v": (0.0, 0.5 * w, 0.0), "w": (0.0, 0.0, w)}}]

    try:
        for fps, w in ((24, 10.0), (120, 10.0), (24, 5.0)):
            hou.setFps(fps)
            expected = 500.0 * w * w
            for compound in (True, False):
                def breaks(strength, w=w, compound=compound, fps=fps):
                    dop, _names = bond_scene(tag("bond_spin"), pair(w), [(0, (0.0, 12, 0.0), 1, (0.0, 12, 0.0))],
                                             bond={"bond_tensile": strength, "bond_shear": strength},
                                             compound=compound, keep_velocity=True)
                    return first_broken(dop, max(3, int(0.25 * fps))) is not None

                found = threshold(breaks, expected / 4, expected * 4, rounds=9)
                good = found is not None and abs(found - expected) < 0.03 * expected
                ok = ok and good
                rows.append(f"{fps} fps, w {w:g}, {'compound' if compound else 'welds'}: {found and round(found)} Pa "
                            f"(formula {expected:.0f}, {'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    return [Result("bond_spin", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_spin"] = test_bond_spin


# ---------------------------------------------------------------------------
# Impacts
# ---------------------------------------------------------------------------


def test_bond_impact():
    """A 1000 kg box thrown at v into a 1000 kg block, in zero gravity with no ground (so nothing but momentum moves),
    against the strength at which the Bond lets go with a Fracture Energy G. A weak bond (capacity 1 kN) takes the
    whole impulse, so it tears at G = J^2 / (2 A m), J the impulse through it and m the reduced mass of what it holds.
    (1) The block held to a static wall (m = 1000 kg): J = M v (1 + e), e the ball's rebound speed over v (about 0.12
    here). (2) The block bonded to a second free one (m = 500 kg): the second ends at the three boxes' common speed
    less the rebound: J = M v / 3 (1 + e'). So G* is between the hand value J = M v and (1.2)^2 times it, and 4 times
    larger at twice the speed (within 10%). The weld force Box3D reports is the last substep's alone and gave 3% of
    this."""
    rows, ok = [], True

    def scene(kind, v):
        ball = {"size": (1, 1, 1), "center": (-1.5, 5.0, 0), "attrs": {"v": (v, 0.0, 0.0)}}
        if kind == "pair":
            return [{"size": (1, 1, 1), "center": (0.5, 5.0, 0), "attrs": {"active": 1}},
                    {"size": (1, 1, 1), "center": (1.5, 5.0, 0), "attrs": {"active": 1}}, ball]
        return [{"size": (1, 1, 1), "center": (1.5, 5.0, 0), "attrs": {"active": 0}},
                {"size": (1, 1, 1), "center": (0.5, 5.0, 0), "attrs": {"active": 1}}, ball]

    for kind, reduced_mass, factor in (("wall", 1000.0, 1.0), ("pair", 500.0, 1.0 / 3.0)):
        found = {}
        for v in (4.0, 8.0):
            def breaks(energy, v=v, kind=kind):
                dop, _names = bond_scene(tag("bond_impact"), scene(kind, v),
                                         [(0, (1.0, 5.0, 0.0), 1, (1.0, 5.0, 0.0))],
                                         bond={"bond_tensile": 1e3, "bond_shear": 1e3, "bond_compressive": 1e3,
                                               "bond_fracture_energy": energy},
                                         keep_velocity=True, gravity=(0.0, 0.0, 0.0), packed_parms={"bounce": 0.0})
                return first_broken(dop, 24) is not None

            found[v] = threshold(breaks, 0.1, 1e6, rounds=10)
            hand = (1000.0 * v * factor) ** 2 / (2.0 * 1.0 * reduced_mass)
            good = found[v] is not None and hand * 0.95 <= found[v] <= hand * 1.45
            ok = ok and good
            rows.append(f"{kind}, {v:g} m/s: G* {found[v] and round(found[v])}, hand {hand:.0f} "
                        f"({'ok' if good else 'wrong'})")
        ratio = found[8.0] / found[4.0] if found[4.0] and found[8.0] else None
        good = ratio is not None and abs(ratio - 4.0) < 0.4
        ok = ok and good
        rows.append(f"{kind}: twice the speed, {ratio and round(ratio, 2)} times the energy (expected 4, "
                    f"{'ok' if good else 'wrong'})")
    return [Result("bond_impact", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_impact"] = test_bond_impact


def test_bond_world_anchor():
    """A piece bonded to the world does not move, so the side of a Bond that holds it has infinite mass. A 1000 kg box
    hangs from a free box that a strong Bond ties to the world (no static piece), on a bond of 5 kPa with a Fracture
    Energy of 1000 J/m^2 over 1 m^2: it holds one free box, so its reduced mass is 1000 kg and it tears after 1414 N*s
    of the 4807 N beyond its capacity, on frame 10 at 24 fps (frame 7 when the anchor was not counted, as 500 kg)."""
    original = hou.fps()
    hou.setFps(24)
    try:
        pieces = [{"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"active": 1}},
                  {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 1}}]
        # The attributes multiply the node's values; once a line has one, every line does (0 where absent).
        names = ("bond_tensile", "bond_shear", "bond_fracture_energy")
        weak = {name: 1.0 for name in names}
        strong = {name: 1e4 for name in names}
        dop, _names = bond_scene(tag("bond_anchor"), pieces,
                                 [(0, AT, 1, AT, weak), (None, (0.0, 5.5, 0.0), 0, (0.0, 5.5, 0.0), strong)],
                                 bond={"bond_tensile": 5000.0, "bond_shear": 5000.0, "bond_fracture_energy": 1000.0})
        first = first_broken(dop, 14)
    finally:
        hou.setFps(original)
    ok = first is not None and 9 <= first <= 11
    return [Result("bond_world_anchor", Result.PASS if ok else Result.FAIL,
                   f"broke on frame {first} (expected 10, within 9 to 11)")]


TESTS["bond_world_anchor"] = test_bond_world_anchor


def test_bond_gravity_and_forces():
    """A compound's Bond load follows the gravity scale and Force DOP forces, which reach it through the compound's
    change of velocity. The hanging 1000 kg box (9.8 kPa) on a bond of 15 kPa holds; with Gravity Scale 2 (19.6 kPa)
    or an extra Gravity DOP of the same size it breaks. On a bond of 6 kPa it breaks, and with Gravity Scale 0.5
    (4.9 kPa) it holds."""
    def broken(strength, scale=None, extra_gravity=False):
        # Every piece needs the attribute, so the first one carries it too.
        pieces = [dict(p, attrs=dict(p["attrs"], gravityscale=1.0 if scale is None else scale)) for p in HANG]
        dop, _names = bond_scene(tag("bond_gravity"), pieces, [LINE], compound=True, bond={"bond_tensile": strength,
                                                                          "bond_shear": strength})
        if extra_gravity:
            out = dop.node("output")
            grav = create(dop, "gravity", "extra_gravity")
            grav.setInput(0, out.inputs()[0])
            set_parm(grav, ["force"], (0.0, -9.80665, 0.0))
            out.setInput(0, grav)
        return first_broken(dop, 24) is not None

    cases = [("15 kPa, scale 1", dict(strength=15000.0), False),
             ("15 kPa, scale 2", dict(strength=15000.0, scale=2.0), True),
             ("15 kPa, Force DOP", dict(strength=15000.0, extra_gravity=True), True),
             ("6 kPa, scale 1", dict(strength=6000.0), True),
             ("6 kPa, scale 0.5", dict(strength=6000.0, scale=0.5), False)]
    rows, ok = [], True
    for label, kwargs, expect in cases:
        got = broken(**kwargs)
        ok = ok and got == expect
        rows.append(f"{label}: {'breaks' if got else 'holds'}{'' if got == expect else ' (WRONG)'}")
    return [Result("bond_gravity_and_forces", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_gravity_and_forces"] = test_bond_gravity_and_forces


def test_bond_loop_mass():
    """The mass a Bond tears with where other bonds still join its two pieces (a loop). Cutting such a bond frees
    nothing alone, so it holds its two pieces' own masses, where a bond that is the only way between two groups holds
    each group's. The tear impulse is sqrt(2 G A m) and `bond_damage` is `bond_impulse` over it, so the mass is read off
    a bond that is over its capacity but far from tearing: m = (impulse / damage)^2 / (2 G A). Four 1000 kg boxes in a
    square, the top left one static. As a ring of 4 bonds they hold 1000 kg (the static piece is fixed, so the other
    one's mass), 500, 500 and 1000 kg. With the last-but-one bond left out (a chain) the first holds the two boxes
    below it (2000 kg), the second the one under that (1000 kg) and the last the box it hangs (1000 kg). Within 3%, as
    a compound and as welds."""
    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    energy = 1.0e7
    # Static top left (0), then clockwise: top right (1), bottom right (2), bottom left (3).
    pieces = [{"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (1, 5, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (1, 4, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 1}}]
    sides = [(0, (0.5, 5.0, 0.0), 1, (0.5, 5.0, 0.0)), (1, (1.0, 4.5, 0.0), 2, (1.0, 4.5, 0.0)),
             (2, (0.5, 4.0, 0.0), 3, (0.5, 4.0, 0.0)), (3, (0.0, 4.5, 0.0), 0, (0.0, 4.5, 0.0))]
    try:
        for label, lines, expected in (("ring", sides, (1000.0, 500.0, 500.0, 1000.0)),
                                       ("chain", [sides[0], sides[1], sides[3]], (2000.0, 1000.0, 1000.0))):
            for compound in (True, False):
                mode = "compound" if compound else "welds"
                seen = {}

                def look(sim, f, seen=seen):
                    geo = find_network_geometry(sim)
                    if geo is None or f != 10:
                        return
                    for name in ("bond_impulse", "bond_damage", "bond_area"):
                        if geo.findPrimAttrib(name) is not None:
                            seen[name] = [prim.attribValue(name) for prim in geo.prims()]

                dop, _names = bond_scene(tag("bond_loop"), pieces, lines, compound=compound,
                                         bond={"bond_tensile": 1000.0, "bond_shear": 1000.0,
                                               "bond_fracture_energy": energy})
                set_parm(dop.node("box3d_solver"), ["bondreadouts"], 1)
                step(dop, 10, look)
                got = []
                for i in range(len(lines)):
                    try:
                        impulse, damage, area = (seen[n][i] for n in ("bond_impulse", "bond_damage", "bond_area"))
                    except (KeyError, IndexError):
                        got.append(None)
                        continue
                    got.append(None if not (damage > 0.0 and impulse > 0.0 and area > 0.0)
                               else (impulse / damage) ** 2 / (2.0 * energy * area))
                good = all(m is not None and abs(m - e) < 0.03 * e for m, e in zip(got, expected))
                ok = ok and good
                rows.append(f"{label}, {mode}: " + ", ".join(f"{m and round(m)}" for m in got)
                            + f" kg (expected {', '.join(str(round(e)) for e in expected)}, {'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    return [Result("bond_loop_mass", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_loop_mass"] = test_bond_loop_mass


def test_bond_tear_mass_compound():
    """The mass a Bond tears with is the graph sides' mass, the same as a compound and as welds. As in bond_impact's
    pair, a 1000 kg ball hits two bonded free boxes at 4 m/s in zero gravity (reduced mass 500 kg), and in
    bond_world_anchor's scene a free box bonded to the world holds a hanging one (1000 kg). The Fracture Energy at which
    the pair lets go is within 15% of the hand value (J = M v / 3, as bond_impact) in both modes, and the anchored
    scene breaks on frame 9 to 11 at 24 fps in both."""
    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    try:
        v = 4.0
        pair = [{"size": (1, 1, 1), "center": (0.5, 5.0, 0), "attrs": {"active": 1}},
                {"size": (1, 1, 1), "center": (1.5, 5.0, 0), "attrs": {"active": 1}},
                {"size": (1, 1, 1), "center": (-1.5, 5.0, 0), "attrs": {"v": (v, 0.0, 0.0)}}]
        hand = (1000.0 * v / 3.0) ** 2 / (2.0 * 500.0)
        for compound in (True, False):
            def breaks(energy, compound=compound):
                dop, _names = bond_scene(tag("bond_tearmass"), pair, [(0, (1.0, 5.0, 0.0), 1, (1.0, 5.0, 0.0))],
                                         bond={"bond_tensile": 1e3, "bond_shear": 1e3, "bond_compressive": 1e3,
                                               "bond_fracture_energy": energy},
                                         keep_velocity=True, gravity=(0.0, 0.0, 0.0), packed_parms={"bounce": 0.0},
                                         compound=compound)
                return first_broken(dop, 24) is not None

            found = threshold(breaks, 0.1, 1e6, rounds=10)
            good = found is not None and hand * 0.95 <= found <= hand * 1.45
            ok = ok and good
            rows.append(f"pair, {'compound' if compound else 'welds'}: G* {found and round(found)}, hand {hand:.0f} "
                        f"({'ok' if good else 'wrong'})")
        hang = [{"size": (1, 1, 1), "center": (0, 5, 0), "attrs": {"active": 1}},
                {"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 1}}]
        names = ("bond_tensile", "bond_shear", "bond_fracture_energy")
        weak = {name: 1.0 for name in names}
        strong = {name: 1e4 for name in names}
        for compound in (True, False):
            dop, _names = bond_scene(tag("bond_tearmass"), hang,
                                     [(0, AT, 1, AT, weak), (None, (0.0, 5.5, 0.0), 0, (0.0, 5.5, 0.0), strong)],
                                     bond={"bond_tensile": 5000.0, "bond_shear": 5000.0,
                                           "bond_fracture_energy": 1000.0}, compound=compound)
            first = first_broken(dop, 14)
            good = first is not None and 9 <= first <= 11
            ok = ok and good
            rows.append(f"anchored, {'compound' if compound else 'welds'}: broke on frame {first} "
                        f"({'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    return [Result("bond_tear_mass_compound", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_tear_mass_compound"] = test_bond_tear_mass_compound


def test_bond_hit_below_tear_impulse():
    """A hit that passes a Bond's capacity but not its tear impulse leaves it whole. As bond_impact's wall case, a
    1000 kg ball hits a block held to a static wall at 4 m/s in zero gravity, on a weak bond (1 kN over 1 m^2). The
    impulse (about 4.4 kN*s) is far over what the bond holds for a fraction of a second (20 N*s), so the hit is
    looked at, but with a Fracture Energy of 30000 J/m^2 the tear impulse is about 7.7 kN*s and it holds, as a
    compound and as welds. With 100 J/m^2 (tear impulse 450 N*s) it breaks in both."""
    pieces = [{"size": (1, 1, 1), "center": (1.5, 5.0, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0.5, 5.0, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (-1.5, 5.0, 0), "attrs": {"v": (4.0, 0.0, 0.0)}}]
    rows, ok = [], True
    for compound in (True, False):
        for energy, expect in ((30000.0, False), (100.0, True)):
            dop, _names = bond_scene(tag("bond_below"), pieces, [(0, (1.0, 5.0, 0.0), 1, (1.0, 5.0, 0.0))],
                                     bond={"bond_tensile": 1e3, "bond_shear": 1e3, "bond_compressive": 1e3,
                                           "bond_fracture_energy": energy},
                                     keep_velocity=True, gravity=(0.0, 0.0, 0.0), packed_parms={"bounce": 0.0},
                                     compound=compound)
            got = first_broken(dop, 24) is not None
            ok = ok and got == expect
            rows.append(f"{'compound' if compound else 'welds'}, {energy:g} J/m^2: {'breaks' if got else 'holds'}"
                        f"{'' if got == expect else ' (WRONG)'}")
    return [Result("bond_hit_below_tear_impulse", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_hit_below_tear_impulse"] = test_bond_hit_below_tear_impulse


def test_bond_sliding_friction():
    """The load of a Bond on pieces sliding on the ground, as a compound and as welds. Two 1000 kg boxes bonded side by
    side slide along the ground at 3 m/s, the rear one with a friction of 0.5 and the ground too, the front one with
    none. Only the rear one is slowed, so the pair slows at a (read off its speed over 8 frames) and the bond pulls
    the front box with m * a, whatever the friction coefficient the contact ends up with: 1000 * a Pa over 1 m^2. The
    strength at which the bond lets go within 12 frames is found by bisection in each mode and must be within -15% to
    +25% of that."""
    pieces = [{"size": (1, 1, 1), "center": (0.5, 0.5, 0), "attrs": {"v": (3.0, 0.0, 0.0), "friction": 0.5}},
              {"size": (1, 1, 1), "center": (1.5, 0.5, 0), "attrs": {"v": (3.0, 0.0, 0.0), "friction": 0.0}},
              dict(GROUND, attrs={"active": 0, "friction": 0.5})]
    for pc in pieces[:2]:
        pc["attrs"]["active"] = 1

    def scene(strength, compound):
        return bond_scene(tag("bond_slide"), pieces, [(0, (1.0, 0.5, 0.0), 1, (1.0, 0.5, 0.0))],
                          bond={"bond_tensile": strength, "bond_shear": 1e9, "bond_compressive": 1e9,
                                "bond_bending": 1e9},
                          keep_velocity=True, compound=compound)[0]

    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    try:
        for compound in (True, False):
            track = step(scene(1e9, compound), 9,
                         lambda sim, f: list(sim.findObject("pieces").geometry().points())[1].attribValue("v")[0])
            decel = (track[1] - track[9]) / (8.0 / 24.0)
            hand = 1000.0 * decel

            def breaks(strength, compound=compound):
                return first_broken(scene(strength, compound), 12) is not None

            found = threshold(breaks, 3e2, 1e5, rounds=10)
            good = found is not None and hand * 0.85 <= found <= hand * 1.25
            ok = ok and good
            rows.append(f"{'compound' if compound else 'welds'}: the pair slows at {decel:.2f} m/s^2, the bond lets go "
                        f"at {found and round(found)} Pa (m * a {hand:.0f}, {'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    return [Result("bond_sliding_friction", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_sliding_friction"] = test_bond_sliding_friction


def test_bond_sphere_fallback():
    """A Bond between shapes with no convex faces (spheres) has no contact patch, so its area is the smaller
    bounding-box face of the two across the normal: a sphere of radius 0.25 gives 0.25 m^2 (a point contact has none
    to give). A 1000 kg sphere (its mass set) hangs from a static one, so the bond lets go at its weight over that
    area, 9807 / 0.25 = 39226 Pa, within -10% to +15%, as a compound and as welds."""
    def sphere(center, active):
        return {"size": (0.5, 0.5, 0.5), "center": center,
                "attrs": {"bullet_georep": "sphere", "bullet_autofit": 0, "bullet_autofit_valid": 1,
                          "bullet_primS": (0.5, 0.5, 0.5), "bullet_radius": 0.25, "bullet_length": 1.0,
                          "bullet_primT": (0.0, 0.0, 0.0), "bullet_primR": (0.0, 0.0, 0.0), "bounce": 0.0,
                          "active": active}}

    pieces = [sphere((0, 5.0, 0), 0), sphere((0, 4.5, 0), 1)]
    hand = WEIGHT / 0.25
    rows, ok = [], True
    for compound in (True, False):
        def breaks(strength, compound=compound):
            dop, _names = bond_scene(tag("bond_sphere"), pieces, [(0, (0.0, 4.75, 0.0), 1, (0.0, 4.75, 0.0))],
                                     bond={"bond_tensile": strength, "bond_shear": 1e9, "bond_compressive": 1e9,
                                           "bond_bending": 1e9},
                                     compound=compound, packed_parms={"mass": 1000.0, "computemass": 0})
            return first_broken(dop, 24) is not None

        found = threshold(breaks, 3e3, 3e5, rounds=10)
        good = found is not None and hand * 0.9 <= found <= hand * 1.15
        ok = ok and good
        rows.append(f"{'compound' if compound else 'welds'}: lets go at {found and round(found)} Pa "
                    f"(weight over the box area {hand:.0f}, {'ok' if good else 'wrong'})")
    return [Result("bond_sphere_fallback", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_sphere_fallback"] = test_bond_sphere_fallback


def test_bond_damage_kept_unloaded():
    """The damage a Bond has taken stays when its load drops, and builds on from there. The hanging box (1000 kg, 1 m^2)
    on a bond of 8 kPa (a ratio of 1.23) with a Fracture Energy of 500 J/m^2 (a tear impulse of 1000 N*s) takes about
    1.8 kN*s per second over its capacity, so alone it goes on frame 14 or so at 24 fps. Here a force of 5 kN up on the
    box holds its load at 4.8 kN, under the capacity, from frame 10 to 31: the bond then keeps `bond_impulse` as it was
    (within 1 N*s) and does not break, and once the force is gone it breaks within 8 frames, where from nothing it would
    take about 13."""
    original = hou.fps()
    hou.setFps(24)
    try:
        # Awake throughout: a box held by the bond under a force would otherwise fall asleep and not feel it go.
        pieces = [dict(p, attrs=dict(p["attrs"], enablesleep=0)) for p in HANG]
        dop, _names = bond_scene(tag("bond_kept"), pieces, [LINE],
                                 bond={"bond_tensile": 8000.0, "bond_shear": 8000.0, "bond_fracture_energy": 500.0})
        out = dop.node("output")
        solver = out.inputs()[0]
        upstream = solver.inputs()[0]
        solver.setInput(0, None)
        lift = sop_solver(dop, "v@force = (@Frame >= 10 && @Frame < 32 && i@active == 1) ? {0, 5000, 0} : {0, 0, 0};")
        out.setInput(0, multisolver(dop, upstream, [solver, lift]))
        impulses, first = {}, None
        for f in range(1, 49):
            hou.setFrame(f)
            sim = dop.simulation()
            sim.objects()
            geo = find_network_geometry(sim)
            if geo is not None and geo.findPrimAttrib("bond_impulse") is not None:
                impulses[f] = max(prim.attribValue("bond_impulse") for prim in geo.prims())
            if (broken_count(sim) or 0) >= 1:
                first = f
                break
    finally:
        hou.setFps(original)
    before = impulses.get(12)
    held = [impulses.get(f) for f in range(13, 32)]
    flat = None not in held and before is not None and max(held) - before < 1.0
    ok = flat and before > 50.0 and first is not None and 32 <= first <= 40
    return [Result("bond_damage_kept_unloaded", Result.PASS if ok else Result.FAIL,
                   f"bond_impulse {before and round(before)} N*s on frame 12, "
                   f"{held[0] and round(held[0])} to {held[-1] and round(held[-1])} on frames 13 to 31 (flat: {flat}); "
                   f"broke on frame {first} (expected 32 to 40)")]


TESTS["bond_damage_kept_unloaded"] = test_bond_damage_kept_unloaded


def test_bond_force_end_wakes():
    """A box held by a Bond under a point `force` falls asleep (the weld does not let it move), the force stops
    reaching it and the joint's reading stays at the load it had. When the force ends the body is woken, so the bond
    feels the whole weight again. The hanging box (1000 kg, 1 m^2) on a bond of 8 kPa with a Fracture Energy of
    500 J/m^2, a force of 5 kN up from frame 1 to 31 (load 4.8 kN, under the capacity) with the default sleeping: the
    bond holds through the force and, once it ends, breaks within about 13 frames (the 1.8 kN*s per second over its
    capacity against a tear impulse of 1000 N*s, at 24 fps), where a body left asleep never would."""
    original = hou.fps()
    hou.setFps(24)
    try:
        dop, _names = bond_scene(tag("bond_force_end"), HANG, [LINE],
                                 bond={"bond_tensile": 8000.0, "bond_shear": 8000.0, "bond_fracture_energy": 500.0})
        out = dop.node("output")
        solver = out.inputs()[0]
        upstream = solver.inputs()[0]
        solver.setInput(0, None)
        lift = sop_solver(dop, "v@force = (@Frame < 32 && i@active == 1) ? {0, 5000, 0} : {0, 0, 0};")
        out.setInput(0, multisolver(dop, upstream, [solver, lift]))
        first = None
        for f in range(1, 65):
            hou.setFrame(f)
            sim = dop.simulation()
            sim.objects()
            if (broken_count(sim) or 0) >= 1:
                first = f
                break
    finally:
        hou.setFps(original)
    ok = first is not None and 38 <= first <= 52
    return [Result("bond_force_end_wakes", Result.PASS if ok else Result.FAIL,
                   f"force until frame 31, bond broke on frame {first} (expected 38 to 52, never when the box stays asleep)")]


TESTS["bond_force_end_wakes"] = test_bond_force_end_wakes


def test_bond_readouts():
    """The Bond's readouts on its line (`bond_ratio`, `bond_damage`, `bond_tension`, `bond_shear`, `bond_bending`,
    `bond_twist`, `bond_area`), with Write Bond Attributes on (off, none exists).
    The hanging 1000 kg box (9.8 kPa over 1 m^2): a Tensile
    Strength of 20 kPa reads a ratio of 0.49 (within 5%) and no damage, as a compound and as welds. At 8 kPa (1.23) with
    a Fracture Energy of 500 J/m^2 (a tear impulse of 1000 N*s) the ratio reads 1.23 (within 5%) and the damage rises
    from 0 towards 1 while it holds, a share of `bond_impulse` over the tear impulse (within 2%). The load is the box's
    weight, 9807 N of tension (within 5%) over an area of 1 m^2, with shear, bending and twist under 5% of it (N*m)."""
    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    try:
        for compound in (True, False):
            mode = "compound" if compound else "welds"
            seen = {}

            def look(sim, f, seen=seen):
                geo = find_network_geometry(sim)
                if geo is None:
                    return
                for name in ("bond_ratio", "bond_damage", "bond_impulse", "bond_tension", "bond_shear", "bond_bending",
                             "bond_twist", "bond_area"):
                    if geo.findPrimAttrib(name) is not None:
                        seen.setdefault(name, {})[f] = max(prim.attribValue(name) for prim in geo.prims())

            dop, _names = bond_scene(tag("bond_readout"), HANG, [LINE], compound=compound,
                                     bond={"bond_tensile": 20000.0, "bond_shear": 20000.0})
            set_parm(dop.node("box3d_solver"), ["bondreadouts"], 1)
            step(dop, 8, look)
            ratio = seen.get("bond_ratio", {}).get(8)
            damage = seen.get("bond_damage", {}).get(8)
            good = ratio is not None and abs(ratio - 0.49) < 0.05 * 0.49 and damage == 0.0
            ok = ok and good
            rows.append(f"{mode}, 20 kPa: ratio {ratio and round(ratio, 3)}, damage {damage} ({'ok' if good else 'wrong'})")
            seen.clear()
            dop, _names = bond_scene(tag("bond_readout"), HANG, [LINE], compound=compound,
                                     bond={"bond_tensile": 8000.0, "bond_shear": 8000.0,
                                           "bond_fracture_energy": 500.0})
            set_parm(dop.node("box3d_solver"), ["bondreadouts"], 1)
            step(dop, 8, look)
            ratio = seen.get("bond_ratio", {}).get(8)
            damage = seen.get("bond_damage", {}).get(8)
            impulse = seen.get("bond_impulse", {}).get(8)
            load = {n: seen.get(n, {}).get(8) for n in ("bond_tension", "bond_shear", "bond_bending", "bond_twist",
                                                         "bond_area")}
            loads_ok = (None not in load.values() and abs(load["bond_tension"] - WEIGHT) < 0.05 * WEIGHT
                        and abs(load["bond_area"] - 1.0) < 0.01
                        and all(abs(load[n]) < 0.05 * WEIGHT for n in ("bond_shear", "bond_bending", "bond_twist")))
            good = (loads_ok and ratio is not None and abs(ratio - 1.226) < 0.05 * 1.226 and damage is not None
                    and 0.0 < damage < 1.0 and impulse is not None and abs(damage - impulse / 1000.0) < 0.02)
            ok = ok and good
            rows.append(f"{mode}, 8 kPa: ratio {ratio and round(ratio, 3)}, damage {damage and round(damage, 3)} with "
                        f"{impulse and round(impulse)} N*s, load {({n[5:]: v and round(v, 2) for n, v in load.items()})} "
                        f"({'ok' if good else 'wrong'})")
        seen.clear()
        dop, _names = bond_scene(tag("bond_readout"), HANG, [LINE], bond={"bond_tensile": 8000.0, "bond_shear": 8000.0})
        step(dop, 8, look)
        good = not seen.get("bond_ratio") and not seen.get("bond_damage")
        ok = ok and good
        rows.append(f"switched off: no attributes ({'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    return [Result("bond_readouts", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_readouts"] = test_bond_readouts


def test_bond_compound_id():
    """`compound_id` on the points of pieces glued into a compound, with Write Bond Attributes on: the lowest point
    number in the compound for each piece of it (a row of 3 boxes bonded together reads 0, 0, 0) and -1 for a loose one
    (a fourth box). Off, the attribute does not exist. A 1000 kg box thrown into a row of 2 bonded boxes at 10 m/s
    (a Strength of 2e4: the bond goes, as in bond_wall) leaves them -1 once the bond is broken (it starts 6 m away: the ids show first)."""
    def ids(sim, f):
        geo = sim.findObject("pieces").geometry()
        if geo.findPointAttrib("compound_id") is None:
            return None
        return [p.attribValue("compound_id") for p in geo.points()]

    rows, ok = [], True
    row = [{"size": (1, 1, 1), "center": (0.5 + i, 0.5, 0), "attrs": {"active": 1}} for i in range(3)]
    loose = {"size": (1, 1, 1), "center": (8.5, 0.5, 0), "attrs": {"active": 1}}
    bonds = [(0, (1.0, 0.5, 0.0), 1, (1.0, 0.5, 0.0)), (1, (2.0, 0.5, 0.0), 2, (2.0, 0.5, 0.0))]
    strong = {"bond_tensile": 1e9, "bond_shear": 1e9, "bond_compressive": 1e10, "bond_bending": 1e9}
    for on in (True, False):
        dop, _names = bond_scene(tag("bond_cid"), row + [loose, GROUND], bonds, bond=strong, compound=True)
        if on:
            set_parm(dop.node("box3d_solver"), ["bondreadouts"], 1)
        got = step(dop, 6, ids)[6]
        good = got is None if not on else got is not None and got[:4] == [0, 0, 0, -1]
        ok = ok and good
        rows.append(f"{'on' if on else 'off'}: {got} ({'ok' if good else 'wrong'})")
    pieces = [{"size": (1, 1, 1), "center": (1.5, 0.5, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (2.5, 0.5, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (-4.5, 0.5, 0), "attrs": {"v": (10.0, 0.0, 0.0)}}, GROUND]
    dop, _names = bond_scene(tag("bond_cid"), pieces, [(0, (2.0, 0.5, 0.0), 1, (2.0, 0.5, 0.0))],
                             bond={"bond_tensile": 2e4, "bond_shear": 2e4, "bond_compressive": 2e5}, compound=True,
                             keep_velocity=True)
    set_parm(dop.node("box3d_solver"), ["bondreadouts"], 1)
    track = step(dop, 24, ids)
    before = track[4][:2] if track[4] else None
    after = track[24][:2] if track[24] else None
    good = before == [0, 0] and after == [-1, -1]
    ok = ok and good
    rows.append(f"hit pair: {before} on frame 4, {after} on frame 24 ({'ok' if good else 'wrong'})")
    return [Result("bond_compound_id", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_compound_id"] = test_bond_compound_id


# ---------------------------------------------------------------------------
# Scrubbing, next_constraint_name, live frames, sleeping
# ---------------------------------------------------------------------------


def test_bond_scrub_and_next():
    """A Bond that breaks, as any joint does. (1) A weak bond (500 Pa) lets the hanging box go: a line in the `broken`
    group on frame 24, and none after scrubbing back to frame 1. (2) The same line with `next_constraint_name` Weld (a
    Weld node beside the Bond): its name becomes Weld on the break and the box stays at its place (y = 4, a rigid
    weld holds it) instead of falling. (3) A bond that holds writes the live frames: the line's points' `current_P`
    stay at the bond's point (0, 4.5, 0) within 0.02."""
    rows, ok = [], True
    dop, names = bond_scene(tag("bond_scrub"), HANG, [LINE], bond={"bond_tensile": 500.0, "bond_shear": 500.0})
    track = step(dop, 24, lambda sim, f: (packed_positions(sim, "pieces"), broken_count(sim)))
    hou.setFrame(1)
    back = broken_count(dop.simulation())
    y, broken = track[24][0][names[1]][1], track[24][1]
    good = broken == 1 and back == 0 and y < 3.0
    ok = ok and good
    rows.append(f"breaks: {broken} broken and the box at y {y:.2f} on frame 24, {back} broken after scrubbing back "
                f"(expected 1, below 3, 0)")

    dop, names = bond_scene(tag("bond_next"), HANG, [LINE + ({"next_constraint_name": "Weld"},)],
                            bond={"bond_tensile": 500.0, "bond_shear": 500.0}, extra_nodes=[("box3d_weldconrel", {})])
    seen = {}

    def look(sim, f):
        geo = find_network_geometry(sim)
        seen[f] = (packed_positions(sim, "pieces")[names[1]][1],
                   geo.prims()[0].attribValue("constraint_name") if geo is not None else None)

    step(dop, 24, look)
    good = seen[24][1] == "Weld" and abs(seen[24][0] - 4.0) < 0.1
    ok = ok and good
    rows.append(f"next constraint: named {seen[24][1]} and the box at y {seen[24][0]:.2f} on frame 24 (expected Weld, 4)")

    dop, names = bond_scene(tag("bond_live"), HANG, [LINE], bond={"bond_tensile": 1e9, "bond_shear": 1e9})
    held = {}

    def anchors(sim, f):
        geo = find_network_geometry(sim)
        if f == 12 and geo is not None and geo.findPointAttrib("current_P") is not None:
            held["points"] = [pt.attribValue("current_P") for pt in geo.points()]

    step(dop, 12, anchors)
    points = held.get("points") or []
    worst = max((max(abs(p[i] - AT[i]) for i in range(3)) for p in points), default=None)
    good = bool(points) and worst < 0.02
    ok = ok and good
    rows.append(f"live frames: {len(points)} points, the farthest {worst if worst is None else round(worst, 4)} from the "
                f"bond's point (expected under 0.02)")
    return [Result("bond_scrub_and_next", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_scrub_and_next"] = test_bond_scrub_and_next


def test_bond_sleep_wake():
    """Two bonded boxes standing on the ground fall asleep and wake as boxes do, as a compound and as welds. A box
    dropped from 5.5 m above the first one lands after about 1.05 s (frame 27 at 24 per second): both are asleep by
    frame 22 (`awake` 0), both awake on frame 34, and nothing broke (a strong bond)."""
    ground = dict(GROUND, attrs=dict(GROUND["attrs"], awake=1))
    pieces = [{"size": (1, 1, 1), "center": (0.5, 0.5, 0), "attrs": {"awake": 1}},
              {"size": (1, 1, 1), "center": (1.5, 0.5, 0), "attrs": {"awake": 1}},
              {"size": (1, 1, 1), "center": (0.5, 7.0, 0), "attrs": {"awake": 1}}, ground]
    rows, ok = [], True
    for compound in (True, False):
        dop, _names = bond_scene(tag("bond_sleep"), pieces, [(0, (1.0, 0.5, 0.0), 1, (1.0, 0.5, 0.0))],
                                 bond={"bond_tensile": 1e9, "bond_shear": 1e9, "bond_compressive": 1e9},
                                 compound=compound)
        seen = {}

        def look(sim, f):
            if f in (22, 34):
                geo = sim.findObject("pieces").geometry()
                seen[f] = tuple(int(pt.attribValue("awake")) for pt in list(geo.points())[:2])
            if f == 34:
                seen["broken"] = broken_count(sim)

        step(dop, 34, look)
        good = seen[22] == (0, 0) and seen[34] == (1, 1) and seen["broken"] == 0
        ok = ok and good
        rows.append(f"{'compound' if compound else 'welds'}: awake {seen[22]} on frame 22 and {seen[34]} on frame 34, "
                    f"{seen['broken']} broken (expected (0, 0), (1, 1), 0)")
    return [Result("bond_sleep_wake", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_sleep_wake"] = test_bond_sleep_wake


# ---------------------------------------------------------------------------
# Compound mode against the step size, long runs
# ---------------------------------------------------------------------------


def test_bond_compound_invariance():
    """What a compound's Bond lets go at does not depend on the frame rate. The beam of two 2 x 0.5 x 0.5 m halves
    (bond_benchmarks) on two supports, as a compound: the bending stress at the middle is 223.6 kPa (0.95 half
    weights over a modulus of 0.0208 m^3). At 24 and at 120 frames per second it lets go within 3% of that, and
    the two within 2% of each other."""
    from checks_more import bond_beam_broken

    original = hou.fps()
    found, rows, ok = {}, [], True
    expected = 4658.0 / (0.25 * 0.5 / 6.0)
    try:
        for fps in (24, 120):
            hou.setFps(fps)
            found[fps] = threshold(
                lambda b, fps=fps: bond_beam_broken(tag("bond_cinv"), b, True, frames=fps) >= 1, 2e4, 2e6, rounds=9)
            good = found[fps] is not None and abs(found[fps] - expected) < 0.03 * expected
            ok = ok and good
            rows.append(f"{fps} fps: lets go at {found[fps] and round(found[fps])} Pa (formula {expected:.0f}, "
                        f"{'ok' if good else 'wrong'})")
    finally:
        hou.setFps(original)
    good = None not in found.values() and abs(found[24] - found[120]) < 0.02 * found[120]
    ok = ok and good
    rows.append(f"the two within 2% of each other: {'ok' if good else 'wrong'}")
    return [Result("bond_compound_invariance", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_compound_invariance"] = test_bond_compound_invariance


def test_bond_long_run():
    """Standing for a long time does no damage, and a load just over the strength tears at the rate the energy says.
    (1) The hanging 1000 kg box on a bond with a Fracture Energy of 100 J/m^2 and a strength 6% over its load (10400 Pa,
    ratio 0.94): 240 frames at 24 per second, nothing broken and `bond_impulse` under 1% of the tear impulse (447
    N*s). (2) A strength 2% under it (9600 Pa, 207 N over the capacity): it tears after 447 / 207 = 2.16 s, plus the
    two frames of the first test, so on frame 1 + (2.16 + 0.083) * 24 = 55, within 15%. (3) A stack of 4 bonded boxes
    on the ground (a compound) for 240 frames: nothing broken and no damage (`bond_impulse` under 1 N*s)."""
    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    try:
        seen = {}

        def impulse(sim, f):
            geo = find_network_geometry(sim)
            if geo is not None and geo.findPrimAttrib("bond_impulse") is not None:
                seen["impulse"] = max(prim.attribValue("bond_impulse") for prim in geo.prims())

        dop, _names = bond_scene(tag("bond_long"), HANG, [LINE],
                                 bond={"bond_tensile": 10400.0, "bond_shear": 10400.0, "bond_fracture_energy": 100.0})
        broken = None
        for f in range(1, 241):
            hou.setFrame(f)
            sim = dop.simulation()
            sim.objects()
            impulse(sim, f)
            if (broken_count(sim) or 0) >= 1:
                broken = f
                break
        good = broken is None and seen.get("impulse", 0.0) < 4.47
        ok = ok and good
        rows.append(f"6% under the strength for 240 frames: {'broke on ' + str(broken) if broken else 'held'}, damage "
                    f"{round(seen.get('impulse', 0.0), 2)} N*s (expected holds, under 4.47)")

        dop, _names = bond_scene(tag("bond_slow"), HANG, [LINE],
                                 bond={"bond_tensile": 9600.0, "bond_shear": 9600.0, "bond_fracture_energy": 100.0})
        first = first_broken(dop, 120)
        expected = 1 + (447.2 / (WEIGHT - 9600.0) + 2.0 / 24.0) * 24.0
        good = first is not None and abs(first - expected) < 0.15 * expected
        ok = ok and good
        rows.append(f"2% over: tore on frame {first}, formula {expected:.0f} ({'ok' if good else 'wrong'})")

        centers = [(0.5, 0.5 + i, 0) for i in range(4)]
        stack = [{"size": (1, 1, 1), "center": c, "attrs": {"active": 1}} for c in centers] + [GROUND]
        dop, _names = bond_scene(tag("bond_stack"), stack,
                                 [(i, (0.5, 1.0 + i, 0.0), i + 1, (0.5, 1.0 + i, 0.0)) for i in range(3)],
                                 bond={"bond_tensile": 1e5, "bond_shear": 1e5, "bond_fracture_energy": 100.0})
        seen.clear()
        broken = None
        for f in range(1, 241):
            hou.setFrame(f)
            sim = dop.simulation()
            sim.objects()
            impulse(sim, f)
            if (broken_count(sim) or 0) >= 1:
                broken = f
                break
        good = broken is None and seen.get("impulse", 0.0) < 1.0
        ok = ok and good
        rows.append(f"stack of 4 for 240 frames: {'broke on ' + str(broken) if broken else 'held'}, damage "
                    f"{round(seen.get('impulse', 0.0), 3)} N*s (expected holds, under 1)")
    finally:
        hou.setFps(original)
    return [Result("bond_long_run", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_long_run"] = test_bond_long_run


# ---------------------------------------------------------------------------
# A wall of Bonds hit, and Glue and Bond in one group
# ---------------------------------------------------------------------------


def test_bond_wall():
    """A wall of 6 boxes (3 wide, 2 high, 1000 kg each) bonded to its neighbours (7 bonds) on the ground, a 1000 kg box
    thrown into it at 10 m/s from 0.3 m away, as one compound (the switch on) and on welds (off), as for Glue
    (glue_compound_hit): the ball rebounds at about 1.6 m/s on frame 3 and the wall moves off at the same. At a strength
    of 2e4 every bond goes, at 5e4 some do (2, 3 and 5 on frames 3, 4 and 5) and at 2e5 none. In both modes the
    number broken is the same (within 1) on frames 3 to 5 and the ball's speed within 0.3 m/s; the extremes are 7 and 0."""
    rows, ok = [], True
    centers = [(1.5 + i, 0.5 + j, 0) for j in range(2) for i in range(3)]
    lines = []
    for j in range(2):
        for i in range(3):
            n = 3 * j + i
            if i < 2:
                lines.append((n, (2.0 + i, 0.5 + j, 0.0), n + 1, (2.0 + i, 0.5 + j, 0.0)))
            if j < 1:
                lines.append((n, (1.5 + i, 1.0, 0.0), n + 3, (1.5 + i, 1.0, 0.0)))
    pieces = [{"size": (1, 1, 1), "center": c, "attrs": {"active": 1}} for c in centers]
    pieces.append({"size": (1, 1, 1), "center": (0.2, 0.5, 0), "attrs": {"v": (10.0, 0.0, 0.0)}})
    pieces.append(GROUND)
    ball = len(pieces) - 2
    for strength, expect in ((2e4, 7), (5e4, None), (2e5, 0)):
        got = {}
        for compound in (True, False):
            dop, _names = bond_scene(tag("bond_wall"), pieces, lines,
                                     bond={"bond_tensile": strength, "bond_shear": strength,
                                           "bond_compressive": strength * 10},
                                     compound=compound, keep_velocity=True)

            def look(sim, f):
                geo = sim.findObject("pieces").geometry()
                return list(geo.points())[ball].attribValue("v")[0], broken_count(sim)

            track = step(dop, 5, look)
            got[compound] = [track[f] for f in (3, 4, 5)]
        speeds = max(abs(a[0] - b[0]) for a, b in zip(got[True], got[False]))
        counts = [(a[1], b[1]) for a, b in zip(got[True], got[False])]
        good = speeds <= 0.3 and all(abs(a - b) <= 1 for a, b in counts)
        if expect is not None:
            good = good and all(a == expect and b == expect for a, b in counts[1:])
        ok = ok and good
        rows.append(f"strength {strength:g}: broken (compound, welds) on frames 3 to 5 {counts}, the ball's speed "
                    f"differs by at most {speeds:.2f} m/s ({'ok' if good else 'wrong'})")
    return [Result("bond_wall", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_wall"] = test_bond_wall


def test_bond_mixed_glue():
    """SideFX Glue and a Box3D Bond in one group of boxes. A row of 3 on the ground, the first pair Glued (a Strength
    that never breaks) and the second pair Bonded: standing for 3 s it holds, sleeps (`awake` 0) and stays at y = 0.5.
    Thrown at at 8 m/s, with the Bond weak (1e3 Pa, 1 kN) and the Glue strong, the Bond line (the second) is the one
    that breaks and the Glue line is not."""
    ground = dict(GROUND, attrs=dict(GROUND["attrs"], awake=1))
    row = [{"size": (1, 1, 1), "center": (0.5 + i, 0.5, 0), "attrs": {"awake": 1}} for i in range(3)]
    lines = [(0, (1.0, 0.5, 0.0), 1, (1.0, 0.5, 0.0), None, "Glue"),
             (1, (2.0, 0.5, 0.0), 2, (2.0, 0.5, 0.0))]
    glue = ("glueconrel", {"strength": 1e7})
    rows, ok = [], True

    dop, names = bond_scene(tag("bond_mixed"), row + [ground], lines,
                            bond={"bond_tensile": 1e9, "bond_shear": 1e9, "bond_compressive": 1e9},
                            extra_nodes=[glue])
    seen = {}

    def look(sim, f):
        if f == 72:
            geo = sim.findObject("pieces").geometry()
            pts = list(geo.points())[:3]
            seen["awake"] = tuple(int(p.attribValue("awake")) for p in pts)
            seen["y"] = [round(p.position()[1], 3) for p in pts]
            seen["broken"] = broken_count(sim)

    step(dop, 72, look)
    good = seen["awake"] == (0, 0, 0) and all(abs(y - 0.5) < 0.02 for y in seen["y"]) and seen["broken"] == 0
    ok = ok and good
    rows.append(f"standing: awake {seen['awake']}, y {seen['y']}, {seen['broken']} broken "
                f"(expected asleep, 0.5, 0)")

    hit = row + [{"size": (1, 1, 1), "center": (-1.5, 0.5, 0), "attrs": {"awake": 1, "v": (8.0, 0.0, 0.0)}}]
    hit[0]["attrs"]["v"] = (0.0, 0.0, 0.0)
    hit[1]["attrs"]["v"] = (0.0, 0.0, 0.0)
    hit[2]["attrs"]["v"] = (0.0, 0.0, 0.0)
    dop, names = bond_scene(tag("bond_mixed_hit"), hit + [ground], lines,
                            bond={"bond_tensile": 1e3, "bond_shear": 1e3, "bond_compressive": 1e3},
                            extra_nodes=[glue], keep_velocity=True)
    got = {}

    def after(sim, f):
        if f == 24:
            geo = find_network_geometry(sim)
            grp = geo.findPrimGroup("broken") if geo is not None else None
            got["broken"] = sorted(p.number() for p in grp.prims()) if grp is not None else []

    step(dop, 24, after)
    good = got.get("broken") == [1]
    ok = ok and good
    rows.append(f"hit: lines broken {got.get('broken')} (expected [1], the Bond; the Glue, line 0, holds)")
    return [Result("bond_mixed_glue", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_mixed_glue"] = test_bond_mixed_glue


# ---------------------------------------------------------------------------
# The Box3D Solver SOP: World Scale and Material
# ---------------------------------------------------------------------------


def test_bond_solver_sop():
    """Bonds in the Box3D Solver SOP, with World Scale and a Material. The hanging 1000 kg box on a Bond with the
    strengths set on the SOP: the strength it lets go at is its weight over the 1 m^2 area, 9807 Pa, in scene units
    at World Scale 1, 2 and 0.5 (within 3%). A material (Concrete) holds it, and the same material with its scale
    at 0.001 lets it go."""
    from checks_sop import play, solver_sop

    pieces, names = packed_boxes(tag("bond_sop") + "_pieces", HANG)
    lines = constraint_lines(tag("bond_sop") + "_lines", [(names[0], AT, names[1], AT, "Bond")])

    def broken(scale, parms):
        sop = solver_sop(tag("bond_sop"), geometry=pieces, constraints=lines)
        set_parm(sop, ["constraint_keepbroken"], 1)
        set_parm(sop, ["usescale"], 0 if scale == 1.0 else 1)
        set_parm(sop, ["scale"], scale)
        set_parm(sop, ["usebond"], 1)
        for parm, value in parms.items():
            set_parm(sop, [parm], value)

        def sample(s, f):
            if f != 12:
                return None
            grp = s.geometry(1).findPrimGroup("broken")
            return len(grp.prims()) if grp is not None else 0

        return play(sop, 12, sample)[12] >= 1

    rows, ok = [], True
    for scale in (1.0, 2.0, 0.5):
        found = threshold(lambda s, scale=scale: broken(scale, {"bond_bond_tensile": s, "bond_bond_shear": s,
                                                                "bond_bond_compressive": s * 10}),
                          2e3, 5e4, rounds=9)
        good = found is not None and abs(found - WEIGHT) < 0.03 * WEIGHT
        ok = ok and good
        rows.append(f"World Scale {scale:g}: lets go at {found and round(found)} Pa (expected {WEIGHT:.0f}, "
                    f"{'ok' if good else 'wrong'})")
    held = not broken(1.0, {"bond_bond_material": 3})
    weak = broken(1.0, {"bond_bond_material": 3, "bond_bond_material_scale": 0.001})
    good = held and weak
    ok = ok and good
    rows.append(f"Concrete: {'holds' if held else 'broke'}, with its scale at 0.001 {'breaks' if weak else 'holds'} "
                f"({'ok' if good else 'wrong'})")
    return [Result("bond_solver_sop", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


TESTS["bond_solver_sop"] = test_bond_solver_sop


def test_bond_constraint_properties():
    """The Box3D Constraint Properties asset writes a Bond's settings. With Bond picked in Constraint Type (ticked) and Tensile
    Strength ticked at 5e4, the line gets the primitive attribute `bond_tensile` 5e4 (the type only picks the folder:
    the line's name stays as it was, Weld here). Fracture Energy unticked writes no `bond_fracture_energy`, and the Material
    menu is not an attribute (no `bond_material`)."""
    from b3dtest import new_geo

    geo = new_geo(tag("bond_cp"))
    pieces = create(geo, "attribwrangle", "pieces")
    set_parm(pieces, ["class"], 0)
    set_parm(pieces, ["snippet"], 'addpointattrib(0, "name", ""); int pt = addpoint(0, set(0, 0, 0)); '
             'setpointattrib(0, "name", pt, "A");')
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); setpointattrib(0, "name", a, "A"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Weld");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, pieces)
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("doconstrainttype", 1), ("constrainttype", 9), ("addbond_bond_tensile", 1),
                        ("bond_bond_tensile", 5e4)):
        set_parm(cp, [name], value)
    prim = cp.geometry(1).prims()[0]
    geo1 = cp.geometry(1)
    got = (prim.attribValue("constraint_name"),
           prim.attribValue("bond_tensile") if geo1.findPrimAttrib("bond_tensile") else None,
           geo1.findPrimAttrib("bond_fracture_energy") is not None, geo1.findPrimAttrib("bond_material") is not None)
    good = got == ("Weld", 5e4, False, False)
    return [Result("bond_constraint_properties", Result.PASS if good else Result.FAIL,
                   f"line name, bond_tensile, has bond_fracture_energy, has bond_material: {got} "
                   f"(expected Weld, 50000, False, False)")]


TESTS["bond_constraint_properties"] = test_bond_constraint_properties

