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
    new_dopnet,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_parm,
    solver_for,
    step,
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


def threshold(breaks, lo, hi, rounds=12):
    """The strength at which `breaks(strength)` flips from True to False, by bisection in log space (None: it does
    not flip between lo and hi)."""
    if not breaks(lo) or breaks(hi):
        return None
    for _ in range(rounds):
        mid = (lo * hi) ** 0.5
        if breaks(mid):
            lo = mid
        else:
            hi = mid
    return hi


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
    glue = ("glueconrel", {"strength": 1e12})
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
