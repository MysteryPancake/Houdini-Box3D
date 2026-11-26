"""SideFX Glue and Hard in Box3D, compared with Bullet where it has the same: impacts, breaking, Glue as One Body,
bridges, Glue Inertial Load."""

import importlib.util
import itertools
import os
import sys

import hou
from b3dtest import (
    GROUND,
    close,
    constraint_lines,
    create,
    display,
    find_network_geometry,
    finish,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_parm,
    solver_for,
    step,
    threshold,
)
from checks_constraints import broken_count, hanging_pair, sidefx_compare


# ---------------------------------------------------------------------------
# SideFX Glue, Hard without constraint_type, Cone Twist: compared with Bullet
# ---------------------------------------------------------------------------


def test_sidefx_glue():
    """SideFX Glue holding a hanging piece, at several Strengths (1 to 1e7): its weight is no hit, so
    it holds at every strength, in Box3D and in Bullet (Glue breaks on the impulse of collisions
    only: see glue_impact)."""
    rows = []
    held = True
    for strength in (1.0, 100.0, 1e4, 1e7):
        out = {}
        for kind in ("box3d", "bullet"):
            tag = f"glue_{kind}_{int(strength)}"
            dop, names = hanging_pair(tag, None, conrels=[("glueconrel", {"strength": strength})], solver_kind=kind)
            y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
            out[kind] = "held" if y > 1.9 else "broke"
        rows.append(f"strength {strength:g}: Box3D {out['box3d']}, Bullet {out['bullet']}")
        held = held and out["box3d"] == "held" and out["bullet"] == "held"
    return [Result("sidefx_glue", Result.PASS if held else Result.FAIL, "; ".join(rows) + " (expected held)")]


def glue_pair_run(kind, strength, height=3.0):
    """Two glued boxes dropped from `height` onto the ground: the landing is a hit. Returns the
    number of broken constraint lines on frame 48 (None when the solver reports no `broken` group)
    and whether the pair is still 1 apart."""
    pieces = [{"size": (1, 1, 1), "center": (0, height, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (1, height, 0), "attrs": {"active": 1}},
              dict(GROUND, attrs={"active": 0})]
    tag = f"glue_impact_{kind}_{int(strength)}_{int(height * 10)}"
    dop, names = hanging_pair(tag, None, conrels=[("glueconrel", {"strength": strength})], solver_kind=kind,
                              pieces=pieces, anchors=((0, height, 0), (1, height, 0)))
    track = step(dop, 48, lambda sim, f: (packed_positions(sim, "pieces"), broken_count(sim) if f == 48 else None))
    pos, broke = track[48]
    apart = (pos[names[0]] - pos[names[1]]).length()
    return broke, apart


def test_glue_impact():
    """SideFX Glue breaks on the impulse of the hits it takes: two glued boxes dropped onto the ground
    (the landing is the hit) break at Strength 1 and hold at Strength 1e7, in Box3D (the weld is
    broken by the wrapper from the contact impulses, with the half-life and propagation). Bullet's
    outcome is reported beside it."""
    got = {}
    for strength in (1.0, 1e7):
        got[strength] = {kind: glue_pair_run(kind, strength) for kind in ("box3d", "bullet")}
    low, high = got[1.0]["box3d"][0], got[1e7]["box3d"][0]
    ok = low is not None and low >= 1 and high == 0
    rows = [f"Strength {st:g}: Box3D broken {r['box3d'][0]}, Bullet broken {r['bullet'][0]}"
            for st, r in got.items()]
    return [Result("glue_impact", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected Box3D: broken at 1, none at 1e7)")]


def glue_strength_run(kind, node_strength, attribute):
    """glue_pair_run with a `strength` primitive attribute on the Glue line: broken lines on frame 48."""
    height = 3.0
    pieces = [{"size": (1, 1, 1), "center": (0, height, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (1, height, 0), "attrs": {"active": 1}},
              dict(GROUND, attrs={"active": 0})]

    def add_strength(network):
        wr = hou.node(network).parent().createNode("attribwrangle", "line_strength")
        wr.setInput(0, hou.node(network))
        set_parm(wr, ["class"], 1)
        set_parm(wr, ["snippet"], f"f@strength = {attribute!r};")
        display(wr)
        return wr.path()

    tag = f"glue_strength_{kind}_{next(_GLUE_RUNS)}"
    dop, names = hanging_pair(tag, None, conrels=[("glueconrel", {"strength": node_strength})], solver_kind=kind,
                              pieces=pieces, anchors=((0, height, 0), (1, height, 0)), network_edit=add_strength)
    return step(dop, 48, lambda sim, f: broken_count(sim) if f == 48 else None)[48]


def test_glue_strength_attribute():
    """A Glue line's `strength` attribute against the node's Strength, as Bullet's getStrength: a negative node
    Strength takes the attribute as it is (-1 with 1 breaks a landing pair, as Strength 1 does), a negative
    attribute never breaks (even over a Strength of 1), and two negatives never break (not the positive product).
    Box3D must match the expectation; Bullet's outcome is reported beside it."""
    cases = (("node -1, attribute 1", -1.0, 1.0, True),
             ("node 1e7, attribute -1", 1e7, -1.0, False),
             ("node -1, attribute -1", -1.0, -1.0, False))
    rows, ok = [], True
    for label, node, attribute, breaks in cases:
        b3d = glue_strength_run("box3d", node, attribute)
        bullet = glue_strength_run("bullet", node, attribute)
        good = b3d is not None and (b3d >= 1) == breaks
        ok = ok and good
        rows.append(f"{label}: Box3D broken {b3d}, Bullet broken {bullet} (expected {'broken' if breaks else 'none'})")
    return [Result("glue_strength_attribute", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_glue_compound_break():
    """A row of 4 glued boxes (run as one body) dropped onto the ground with a Strength of 1: every bond
    breaks on the landing, and with a Strength of 1e7 none does."""
    weak = glue_row_run("box3d", 1.0, 48, n=4, height=3.0)[0]
    strong = glue_row_run("box3d", 1e7, 48, n=4, height=3.0)[0]
    ok = weak is not None and weak >= 3 and strong == 0
    return [Result("glue_compound_break", Result.PASS if ok else Result.FAIL,
                   f"broken at Strength 1: {weak}, at 1e7: {strong} (expected 3 and 0)")]


def test_glue_global_iterations():
    """A Glue's Propagation Iterations below 0 is the network's propagate_iteration detail attribute: a box dropped on
    the end of a row of 6 glued boxes (resting on the ground) breaks a different number of bonds with -1 and
    propagate_iteration 6 than with 1 round at some Strength (spreading the impulse further both reaches more bonds and
    dilutes it); -1 with the attribute 6 matches an explicit 6 at every Strength; -1 with no attribute is one round."""
    scene = dict(n=6, drop=4.0)
    rows = []
    differs = False
    same = True
    one_round = True
    for strength in (3e2, 1e3, 3e3, 1e4, 3e4, 1e5):
        one = glue_row_run("box3d", strength, 72, iterations=1, **scene)[0]
        global_six = glue_row_run("box3d", strength, 72, iterations=-1, global_iterations=6, **scene)[0]
        six = glue_row_run("box3d", strength, 72, iterations=6, **scene)[0]
        none = glue_row_run("box3d", strength, 72, iterations=-1, **scene)[0]
        differs = differs or global_six != one
        same = same and global_six == six
        one_round = one_round and none == one
        rows.append(f"Strength {strength:g}: 1 round {one}, -1 with attribute 6: {global_six}, 6: {six}, -1 alone: {none}")
    ok = differs and same and one_round
    return [Result("glue_global_iterations", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + f" (expected a different count at some Strength: {differs}, -1 with 6 = 6: {same}, "
                   f"-1 alone = 1 round: {one_round})")]


def test_glue_compound_vs_welds():
    """Glue as One Body against the same scene on weld joints (the switch off): a row of 4 glued boxes landing from 3
    at a Strength that never breaks it stays one row on the ground (spacing 1, resting at y 0.5) in both, and the
    Strength at which it stops breaking agrees within a factor of 1.5 (Glue's break estimate, which the landing
    is judged on, is the same for both)."""
    rows, ok = [], True
    for compound in (True, False):
        dop = glue_row_scene(f"glue_vs_welds_{int(compound)}", "box3d", 1e12, n=4, height=3.0, compound=compound)
        pos = step(dop, 72, lambda sim, f: packed_positions(sim, "pieces"))[72]
        row = [pos[k] for k in sorted(pos) if abs(pos[k][1] - 0.5) < 1.0 and pos[k][1] > 0]
        spacing = max(abs((row[i + 1] - row[i]).length() - 1.0) for i in range(len(row) - 1)) if len(row) > 1 else 9.0
        low = max(abs(p[1] - 0.5) for p in row) if row else 9.0
        good = len(row) == 4 and spacing < 0.05 and low < 0.02
        ok = ok and good
        rows.append(f"{'one body' if compound else 'welds'}: {len(row)} pieces, spacing off by {spacing:.3f}, "
                    f"height off by {low:.3f} (expected 4, under 0.05, under 0.02)")
    strength = {c: glue_row_threshold("box3d", 48, n=4, height=3.0, compound=c) for c in (True, False)}
    if None in strength.values():
        ok = False
        rows.append(f"no breaking threshold found: {strength}")
    else:
        ratio = strength[True] / strength[False]
        ok = ok and 1 / 1.5 <= ratio <= 1.5
        rows.append(f"Strength at which it stops breaking {strength[True]:.0f} as one body, {strength[False]:.0f} on welds "
                    f"(ratio {ratio:.2f}, expected 0.67 to 1.5)")
    return [Result("glue_compound_vs_welds", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_glue_compound_hit():
    """The `building` example (2600 glued pieces, a 110 tonne ball thrown at it at 30 m/s): with Glue as One Body on the
    ball must do what it does on weld joints (the switch off), not be thrown back by the building as one rigid body (that
    rebounded it at -12 m/s where the welds slowed it to -1 or 3). Thrown from 32 away at 3e5, its speed after the hit is
    within 1.5 m/s of the weld speed, and on frame 130, past the time the released pieces are held for (they merge back
    into compounds), it is within 6 of where it is on welds. A throw from 12 away is not checked: the ball hits the
    building before it has settled, and the two modes then differ by a few m/s. Each run is a new scene, as the scene is
    large."""
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location("make_examples", os.path.join(here, "..", "..", "examples", "make_examples.py"))
    examples = importlib.util.module_from_spec(spec)
    sys.modules["make_examples"] = examples
    spec.loader.exec_module(examples)

    def ball_x(sim, f):
        for obj in sim.objects():
            geo = obj.geometry()
            if geo is not None and geo.findPointAttrib("name") is not None:
                for pt in geo.points():
                    if pt.attribValue("name") == "ball":
                        return pt.position()[0]
        return None

    def run(compound, strength, start_x, frames, hit_frame):
        """(the ball's speed between hit_frame - 1 and hit_frame, its x on the last frame)"""
        hou.hipFile.clear(suppress_save_prompt=True)
        dop, _frames = examples.scene_building(start_x)
        solver = next(n for n in dop.allSubChildren()
                      if "box3d" in n.type().name().lower() and "solver" in n.type().name().lower())
        glue = next(n for n in dop.allSubChildren() if "glue" in n.type().name().lower() and n.parm("strength") is not None)
        glue.parm("strength").set(strength)
        solver.parm("gluecompound").set(1 if compound else 0)
        try:
            xs = step(dop, frames, ball_x)
        finally:
            # The runner lays the scene out for review after the test, which takes over 10 GB for this one.
            hou.hipFile.clear(suppress_save_prompt=True)
        speed = (xs[hit_frame] - xs[hit_frame - 1]) * hou.fps()
        return speed, xs[frames]

    try:
        rows, ok = [], True
        (on, x_on), (off, x_off) = run(True, 3e5, -32.0, 130, 26), run(False, 3e5, -32.0, 130, 26)
        ok = ok and abs(on - off) <= 1.5 and abs(x_on - x_off) <= 6.0
        rows.append(f"from 32 away at 3e+05 its speed after the hit is {on:.1f} and {off:.1f}, and on frame 130 it is at x {x_on:.1f} "
                    f"and {x_off:.1f}")
    finally:
        sys.modules.pop("make_examples", None)
    return [Result("glue_compound_hit", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected speeds within 1.5 m/s of each other, positions within 6)")]


def test_glue_group_contacts():
    """Weight on a piece of the same glue group is no hit. A box rests on two fixed supports, bonded to the first, and the
    supports are bonded to each other (all one group), at a Strength of 100 that any hit would break: it presses on the
    second support, which it is not bonded to, and nothing breaks and it stays put (a contact with a piece of its own
    group counted as a hit before, so a building of glued pieces broke up under its own weight in weld mode)."""
    tag = "glue_group_contacts"
    dop = new_dopnet(tag)
    pieces = [{"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (1, 0.5, 0), "attrs": {"active": 0}},
              {"size": (2, 1, 1), "center": (0.5, 1.5, 0), "attrs": {"active": 1}}]
    path, names = packed_boxes(tag + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], 100.0)
    lines = [(names[2], (0.0, 1.0, 0), names[0], (0.0, 1.0, 0), "Glue"),
             (names[0], (0.5, 0.5, 0), names[1], (0.5, 0.5, 0), "Glue")]
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(tag + "_network", lines))
    net.setInput(1, glue)
    finish(dop, solver_for("box3d", dop, net))
    track = step(dop, 48, lambda sim, f: (packed_positions(sim, "pieces"), broken_count(sim)))
    pos, broken = track[48]
    y = pos[names[2]][1]
    ok = broken == 0 and abs(y - 1.5) < 0.05
    return [Result("glue_group_contacts", Result.PASS if ok else Result.FAIL,
                   f"{broken} bonds broken and the box at y {y:.2f} on frame 48 (expected 0 and 1.5)")]


def test_glue_compound_sleep():
    """Glued pieces run as one body sleep and wake as the pieces do with the switch off. A row of 3 boxes with an
    `awake` attribute, a box dropped on the first: with a Strength that never breaks them, the `awake` values over the
    first 72 frames are the same with the switch on and off, give or take 3 frames (asleep before the box lands, awake on
    landing). With a
    Strength of 1000 that breaks the bonds, all 3 are asleep before the landing, the first is awake after it and all
    are asleep again by frame 160 in both (the pieces of a dissolved or split compound can sleep again)."""
    def awake_track(compound, strength, frames):
        dop = glue_row_scene(f"glue_sleep_{int(compound)}_{int(strength)}", "box3d", strength, n=3, drop=4.0,
                             compound=compound, awake=True)

        def sample(sim, f):
            geo = sim.findObject("pieces").geometry()
            return tuple(pt.attribValue("awake") for pt in list(geo.points())[:3])

        return step(dop, frames, sample)

    rows, ok = [], True
    held = {c: awake_track(c, 1e12, 72) for c in (True, False)}
    # The switch changes when a body falls asleep by a frame or two, so a frame only differs when the other run
    # has no such value within 3 frames.
    differ = [f for f in range(1, 73)
              if not any(held[False].get(g) == held[True][f] for g in range(f - 3, f + 4))]
    cycles = any(held[True][f] == (0, 0, 0) for f in range(1, 73)) and held[True][26] == (1, 1, 1)
    ok = ok and not differ and cycles
    rows.append(f"no break: frames where on and off differ {differ}, asleep then awake on landing: {cycles}")
    for compound in (True, False):
        t = awake_track(compound, 1e3, 160)
        good = t[22] == (0, 0, 0) and t[26][0] == 1 and t[160] == (0, 0, 0)
        ok = ok and good
        rows.append(f"bonds break, switch {'on' if compound else 'off'}: frame 22 {t[22]}, 26 {t[26]}, 160 {t[160]} "
                    f"(expected all 0, the hit piece 1, all 0)")
    return [Result("glue_compound_sleep", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_glue_compound_load():
    """Glue Inertial Load: a column of 6 glued boxes (run as one body) landing on its end breaks its bottom bond on
    a hard hit, but with the load on, the bonds up the column break too, as the whole body is slowed: at some
    Strength more bonds break with Glue Inertial Load 1 than with 0, and a load of 0 matches the default."""
    scene = dict(n=6, height=3.0, column=True)
    rows = []
    more = False
    same = True
    for strength in (1e2, 1e3, 3e3, 1e4, 3e4, 1e5):
        default = glue_row_run("box3d", strength, 48, **scene)[0]
        off = glue_row_run("box3d", strength, 48, load=0.0, **scene)[0]
        on = glue_row_run("box3d", strength, 48, load=1.0, **scene)[0]
        more = more or (on or 0) > (off or 0)
        same = same and default == off
        rows.append(f"Strength {strength:g}: default {default}, load 0: {off}, load 1: {on}")
    ok = more and same
    return [Result("glue_compound_load", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + f" (expected more broken with load 1 at some Strength: {more}, "
                   f"load 0 = default: {same})")]


def glue_threshold(kind, height, lo=500.0, hi=1e7, rounds=7):
    """The Strength at which a glued pair dropped from `height` stops breaking on landing, found by
    bisection in log space between `lo` (must break) and `hi` (must hold). None if `lo` already
    holds or `hi` still breaks."""
    def breaks(strength):
        broke = glue_pair_run(kind, strength, height)[0]
        return broke is not None and broke >= 1

    return threshold(breaks, lo, hi, rounds)


def test_glue_impact_scale():
    """Whether Box3D's contact impulse is on Bullet's scale: the Strength at which a glued pair stops
    breaking when it lands, found by bisection (500 to 1e7), from two drop heights (3 and 6). They
    agree when the two solvers' thresholds are within a factor of 4 at both heights. The message
    gives the thresholds and their ratios: the same ratio at both heights is a constant factor
    between the impulses; a ratio that changes with the height is a different dependence on speed."""
    found = {height: {kind: glue_threshold(kind, height) for kind in ("box3d", "bullet")}
             for height in (3.0, 6.0)}
    rows = []
    ok = True
    for height, t in found.items():
        if t["box3d"] is None or t["bullet"] is None:
            ok = False
            rows.append(f"height {height:g}: Box3D {t['box3d']}, Bullet {t['bullet']} (no threshold in 500..1e7)")
            continue
        ratio = t["bullet"] / t["box3d"]
        ok = ok and 0.25 <= ratio <= 4.0
        rows.append(f"height {height:g}: Box3D holds from {t['box3d']:.0f}, Bullet from {t['bullet']:.0f} "
                    f"(Bullet / Box3D {ratio:.2f})")
    return [Result("glue_impact_scale", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected a ratio within 0.25 to 4)")]


def glue_impact_series(kind, height, frames=36):
    """The Glue's `impact` attribute on every frame for a glued pair dropped from `height` with a Strength
    that never breaks it: {frame: impact}, or None for a frame with no such attribute."""
    pieces = [{"size": (1, 1, 1), "center": (0, height, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (1, height, 0), "attrs": {"active": 1}},
              dict(GROUND, attrs={"active": 0})]
    dop, names = hanging_pair(f"glue_trace_{kind}_{int(height * 10)}", None,
                              conrels=[("glueconrel", {"strength": 1e12})], solver_kind=kind, pieces=pieces,
                              anchors=((0, height, 0), (1, height, 0)))

    def impact(sim, f):
        geo = find_network_geometry(sim)
        prims = list(geo.prims()) if geo is not None else []
        if not prims or geo.findPrimAttrib("impact") is None:
            return None
        return prims[0].attribValue("impact")

    return step(dop, frames, impact)


def test_glue_impact_trace():
    """The Glue `impact` attribute over time in Box3D and in Bullet for the same landing (the solver writes
    the bond's accumulated impulse there): its peak, the frame of the peak, and the values around it.
    They agree when the peaks are within a factor of 4. The series shows whether a difference is a scale
    (the same shape) or in how the impulse builds and decays."""
    rows, ok = [], True
    for height in (3.0, 6.0):
        series = {kind: glue_impact_series(kind, height) for kind in ("box3d", "bullet")}
        peaks = {}
        for kind, values in series.items():
            numeric = {f: v for f, v in values.items() if v is not None}
            if not numeric:
                rows.append(f"height {height:g}: {kind} wrote no impact attribute")
                ok = False
                continue
            frame = max(numeric, key=numeric.get)
            peaks[kind] = numeric[frame]
            around = ", ".join(f"{f}:{numeric.get(f, 0.0):.0f}" for f in range(max(1, frame - 2), frame + 5))
            rows.append(f"height {height:g} {kind}: peak {numeric[frame]:.0f} on frame {frame} (frames {around})")
        if len(peaks) == 2:
            ratio = peaks["bullet"] / peaks["box3d"] if peaks["box3d"] else float("inf")
            ok = ok and 0.25 <= ratio <= 4.0
            rows.append(f"height {height:g}: Bullet / Box3D peak {ratio:.2f}")
    return [Result("glue_impact_trace", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected the peaks within a factor of 4)")]


def fracture_glue_left(kind, strength, frames=24, sep=0.15, tag="frac", **solver_parms):
    """Glue bonds left on the last frame of a fractured cube dropped on a ground plane: a Box rotated 45
    degrees on x and lifted 3, Points from Volume (Jitter Scale 1, Particle Separation `sep`), Voronoi Fracture
    (the cube first, the points second) and the solver SOP with the fracture's two outputs (pieces and its own
    constraints, named Glue) as its inputs, ground plane on. `kind`: "box3d" (Solver SOP, solver_parms are its parameters) or
    "bullet" (RBD Bullet Solver SOP). The Glue Strength (a `strength` primitive attribute on the constraints) is `strength`. Returns (bonds left, bonds at the start)."""
    from checks_sop_common import play
    geo = new_geo(tag)
    try:
        box = create(geo, "box", "box")
        xf = create(geo, "xform", "xf")
        xf.setInput(0, box)
        set_parm(xf, ["rx"], 45)
        set_parm(xf, ["ty"], 3)
        pfv = create(geo, "pointsfromvolume", "pfv")
        pfv.setInput(0, xf)
        set_parm(pfv, ["jitterscale"], 1.0)
        set_parm(pfv, ["particlesep"], sep)
        fracture = create(geo, "voronoifracture", "fracture")
        fracture.setInput(0, xf)
        fracture.setInput(1, pfv)
        sop = create(geo, "box3d_solver" if kind == "box3d" else "rbdbulletsolver", "solver")
        sop.setInput(0, fracture, 0)
        # Voronoi's constraints have no constraint_name: the Bullet solver refuses them and ours ignores them, so
        # both get the same one-line name.
        glue = create(geo, "attribwrangle", "glue_name")
        glue.setInput(0, fracture, 1)
        set_parm(glue, ["class"], "primitive")
        set_parm(glue, ["snippet"], 's@constraint_name = "Glue"; f@strength = %r;' % float(strength))
        sop.setInput(1, glue)
        display(sop)
        set_parm(sop, ["useground"], 1)
        for parm, value in solver_parms.items():
            set_parm(sop, [parm], value)
        counts = play(sop, frames, lambda s, f: len(s.geometry(1).prims()) if f in (1, frames) else None)
    finally:
        geo.destroy()
    return counts[frames], counts[1]


def equivalent_strength(target, grid):
    """The strength at which a curve [(strength, bonds left)] (left rising with strength) reaches `target`
    bonds, interpolated in log space: None if the curve never gets there."""
    if grid and target <= grid[0][1]:
        return grid[0][0]
    for (s0, l0), (s1, l1) in zip(grid, grid[1:]):
        if l0 <= target <= l1:
            if l1 == l0:
                return s0
            t = (target - l0) / (l1 - l0)
            return s0 * (s1 / s0) ** t
    return None


def test_glue_fracture_vs_bullet():
    """A fractured cube shattering on the ground: the Glue Strength at which Box3D leaves as many bonds
    unbroken after the landing (frame 24) as Bullet does at 100, 300 and 1000. Reported as Box3D's
    equivalent / Bullet's, both with Glue as One Body off (a weld per bond) and on (its default): a ratio of 1 is
    the same stiffness, below 1 is Box3D stiffer (it needs less Strength to hold as much). Passes (per mode) when all
    three are within 0.1 to 0.4. It sits at 0.14 to 0.27: the cube lands on an edge, and Bullet's trial step (the
    estimate Glue breaks on) moves it a whole step forward, so the edge sinks into the ground and many more pieces
    have contact points than Box3D's, which stops at touching. A flat landing has the same points either way
    (glue_break_match)."""
    bullet = {s: fracture_glue_left("bullet", s, tag=f"frac_bullet_{int(s)}")[0] for s in (100.0, 300.0, 1000.0)}
    grid = (0.3, 1.0, 3.0, 10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0)
    results = []
    for compound in (0, 1):
        curve = [(s, fracture_glue_left("box3d", s, tag=f"frac_box3d_{compound}_{int(s * 10)}", gluecompound=compound)[0])
                 for s in grid]
        rows, ok = [], True
        for s, left in bullet.items():
            equivalent = equivalent_strength(left, curve)
            if equivalent is None:
                ok = False
                rows.append(f"Bullet {s:g} leaves {left}: Box3D never does (curve {[l for _, l in curve]})")
                continue
            ratio = equivalent / s
            ok = ok and 0.1 <= ratio <= 0.4
            rows.append(f"Bullet {s:g} leaves {left}, Box3D does at {equivalent:.0f} (Box3D / Bullet {ratio:.2f})")
        label = "glue_fracture_vs_bullet" + ("" if compound else "_welds")
        results.append(Result(label, Result.PASS if ok else Result.FAIL,
                              f"Glue as One Body {'on' if compound else 'off'}: " + "; ".join(rows) +
                              " (expected Box3D / Bullet within 0.1 to 0.4)"))
    return results


def glue_beam_run(strength, compound, ball, frames=36):
    """A beam of 8 glued boxes (x 0 to 7, y 5) lying on a fixed block under its middle four, so its two ends overhang by 2
    and only the glue keeps them up, and with `ball` a heavy box falling at 30 m/s onto its right tip (x 7). Returns the
    beam's positions on the last frame, by box (index 0 is the left end)."""
    tag = f"glue_beam_{int(compound)}_{int(ball)}"
    dop = new_dopnet(tag)
    pieces = [{"size": (4, 1, 1), "center": (3.5, 4, 0), "attrs": {"active": 0, "v": (0.0, 0.0, 0.0)}}]
    pieces += [{"size": (1, 1, 1), "center": (i, 5, 0), "attrs": {"active": 1, "v": (0.0, 0.0, 0.0)}} for i in range(8)]
    if ball:
        pieces.append({"size": (1.2, 1.2, 1.2), "center": (7, 8, 0), "attrs": {"active": 1, "v": (0.0, -30.0, 0.0)}})
    path, names = packed_boxes(tag + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], strength)
    lines = [(names[i], (i - 0.5, 5, 0), names[i + 1], (i - 0.5, 5, 0), "Glue") for i in range(1, 8)]
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(tag + "_network", lines))
    net.setInput(1, glue)
    solver = solver_for("box3d", dop, net)
    set_parm(solver, ["gluecompound"], 1 if compound else 0)
    finish(dop, solver)
    track = step(dop, frames, lambda sim, f: packed_positions(sim, "pieces") if f == frames else None)
    pos = track[frames]
    return [pos[n] for n in names[1:9]]


def test_glue_compound_bridges():
    """A beam of 8 glued boxes lying on a fixed block under its middle four, its ends in the air with only the glue to
    hold them, stays as it is as one body or on welds, and when a heavy ball falls on its right tip (the pieces near it
    are freed from the compound while it comes, and the rest stays in it, joined to them by bridges) the left end is
    still in the air where it is on welds. Without a ball no box is more than 0.1 from where it started after 36 frames
    (one body and welds); with it box 0 is within 0.3 of its place on welds and above y 4.5 in both."""
    strength = 2e5
    rows, ok = [], True
    for compound in (False, True):
        quiet = glue_beam_run(strength, compound, False)
        drift = max(((p[1] - 5.0) ** 2 + (p[0] - i) ** 2) ** 0.5 for i, p in enumerate(quiet))
        ok = ok and drift < 0.1
        rows.append(f"no ball, {'one body' if compound else 'welds'}: beam moved at most {drift:.3f}")
    welds = glue_beam_run(strength, False, True)
    body = glue_beam_run(strength, True, True)
    gap = sum((a - b) ** 2 for a, b in zip(welds[0], body[0])) ** 0.5
    ok = ok and gap < 0.3 and welds[0][1] > 4.5 and body[0][1] > 4.5
    rows.append(f"ball on the tip: left end at y {body[0][1]:.2f} as one body, {welds[0][1]:.2f} on welds ({gap:.3f} apart)")
    return [Result("glue_compound_bridges", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected under 0.1 and 0.1, then within 0.3 and above 4.5)")]


_GLUE_RUNS = itertools.count()


def glue_row_scene(tag, kind, strength, n=2, height=3.0, drop=None, bounce=None, iterations=None,
                   global_iterations=None, column=False, load=None, compound=None, awake=False, substeps=None):
    """n glued boxes side by side. With drop=None they fall from `height` onto the ground (the landing is the hit).
    Otherwise they rest on it and a box is dropped from `drop` above the first one. bounce: RBD Packed Object's.
    iterations: the Glue's Propagation Iterations. global_iterations: the network's propagate_iteration detail attribute.
    column: the boxes stand one on another instead (the first lowest). load: the solver's Glue Inertial Load.
    compound: the solver's Glue as One Body (None: its default). awake: the pieces carry an `awake` point attribute.
    substeps: the solver's Substeps (None: its default)."""
    dop = new_dopnet(tag)
    y = height if drop is None else 0.5
    at = (lambda i: (0, y + i, 0)) if column else (lambda i: (i, y, 0))
    attrs = {"active": 1, "awake": 1} if awake else {"active": 1}
    pieces = [{"size": (1, 1, 1), "center": at(i), "attrs": dict(attrs)} for i in range(n)]
    if drop is not None:
        pieces.append({"size": (1, 1, 1), "center": (0, 1.5 + drop, 0), "attrs": dict(attrs)})
    pieces.append(dict(GROUND, attrs=dict(attrs, active=0)))
    path, names = packed_boxes(tag + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path, **({} if bounce is None else {"bounce": bounce}))
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], strength)
    if iterations is not None:
        set_parm(glue, ["propagationiterations"], iterations)
    mid = (lambda i: (0, y + i + 0.5, 0)) if column else (lambda i: (i + 0.5, y, 0))
    lines = [(names[i], mid(i), names[i + 1], mid(i), "Glue") for i in range(n - 1)]
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    network = constraint_lines(tag + "_network", lines)
    if global_iterations is not None:
        wr = hou.node(network).parent().createNode("attribwrangle", "global_iterations")
        wr.setInput(0, hou.node(network))
        set_parm(wr, ["class"], 0)
        set_parm(wr, ["snippet"], f"i@propagate_iteration = {int(global_iterations)};")
        display(wr)
        network = wr.path()
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network)
    net.setInput(1, glue)
    solver = solver_for(kind, dop, net)
    if load is not None:
        set_parm(solver, ["gluecompoundload"], load)
    if compound is not None:
        set_parm(solver, ["gluecompound"], 1 if compound else 0)
    if substeps is not None:
        target = solver if kind == "box3d" else next(n for n in dop.children() if n.type().name() == "bulletrbdsolver")
        set_parm(target, ["substeps"], substeps)
    finish(dop, solver)
    return dop


def glue_row_run(kind, strength, frames, **scene):
    """(broken constraint lines on the last frame, the largest `impact` any bond had) for a glue_row_scene."""
    tag = f"glue_match_{next(_GLUE_RUNS)}"
    dop = glue_row_scene(tag, kind, strength, **scene)
    peak = [0.0]

    def sample(sim, f):
        geo = find_network_geometry(sim)
        if geo is not None and geo.findPrimAttrib("impact") is not None:
            for prim in geo.prims():
                peak[0] = max(peak[0], prim.attribValue("impact"))
        return broken_count(sim) if f == frames else None

    track = step(dop, frames, sample)
    for node in [n for n in hou.node("/obj").children() if n.name().startswith(tag)]:
        node.destroy()  # keeps the saved scene small
    return track[frames], peak[0]


def glue_row_threshold(kind, frames, lo=20.0, hi=1e9, rounds=8, **scene):
    """The Strength at which a glue_row_scene stops breaking, by bisection in log space (None: `lo` holds
    or `hi` still breaks)."""
    def breaks(strength):
        broke = glue_row_run(kind, strength, frames, **scene)[0]
        return broke is not None and broke >= 1

    return threshold(breaks, lo, hi, rounds)


def bullet_landing_hold(n, height, bounce=None, mass=1000.0):
    """The Strength from which Bullet's Glue stops breaking for a row of `n` unit boxes (`mass` each) landing from
    `height` (their centres), by the model of SIM_BulletState::preBreakGlue / updateGlue (the HDK's SIM_SolverBullet):
    before the step the row is one rigid body moved to where the step ends, and each contact point under it gets
    the impulse (1 + e v) v / d, with v the speed it comes in at (the end of the first frame that reaches the ground),
    e the two restitutions multiplied (the default 0.5 each) and d the body's impulse denominator at the point (1 / M +
    x^2 / Izz + z^2 / Ixx for a face on the ground); each piece sums its points', the sums spread over the bonds as
    glueSpread does, and a bond's impact is its two ends'. The largest bond's is the Strength that holds."""
    restitution = (0.5 if bounce is None else bounce) ** 2
    g, dt = 9.80665, 1.0 / 24.0
    frames = 1
    while 0.5 * g * (frames * dt) ** 2 < height - 0.5 - 0.02:
        frames += 1
    v = g * frames * dt
    total = mass * n
    ixx, izz = total * 2.0 / 12.0, total * (n * n + 1.0) / 12.0
    hits = [0.0] * n
    for k in range(n):
        for x in (k - (n - 1) / 2.0 - 0.5, k - (n - 1) / 2.0 + 0.5):
            for z in (-0.5, 0.5):
                hits[k] += (1.0 + restitution * v) * v / (1.0 / total + x * x / izz + z * z / ixx)
    weight = [1.0 + (k > 0) + (k < n - 1) for k in range(n)]
    spread = [hits[k] / weight[k] for k in range(n)]
    for k in range(n - 1):
        spread[k] += hits[k + 1] / weight[k + 1]
        spread[k + 1] += hits[k] / weight[k]
    return max(spread[k] + spread[k + 1] for k in range(n - 1))


GLUE_MATCH_SCENES = [
    ("landing from 1", dict(n=2, height=1.0), 48),
    ("landing from 3", dict(n=2, height=3.0), 48),
    ("landing from 8", dict(n=2, height=8.0), 48),
    ("landing from 3, bounce 0.9", dict(n=2, height=3.0, bounce=0.9), 96),
    ("row of 4 landing from 3", dict(n=4, height=3.0), 48),
    ("box dropped 1 on a pair", dict(n=2, drop=1.0), 72),
    ("box dropped 4 on a pair", dict(n=2, drop=4.0), 72),
    ("box dropped 4 on a row of 4", dict(n=4, drop=4.0), 72),
]


def test_glue_break_match():
    """SideFX Glue in Box3D and Bullet over several kinds of hit: the largest `impact` a bond reaches (with
    a Strength that never breaks it), and the Strength at which the bonds stop breaking. The impacts must agree (Box3D
    from 0.5 to 2 times Bullet's), and so must the Strengths the bonds hold from (Box3D from 0.7 to 1.5 times Bullet's).
    Bullet holds from about its impact when something hits a resting glued pair, and from several times its impact when
    the glued pieces land themselves (more with speed and bounce): it breaks glue before the step on an estimate of the
    impulse at every contact point of the glued body (bullet_landing_hold, in the landing scenes' rows), which a landing
    makes several times the impulse the step then has. Box3D breaks on the same estimate (core/Box3DGlueEstimate.h)."""
    rows, ok = [], True
    for label, scene, frames in GLUE_MATCH_SCENES:
        got = {}
        for kind in ("bullet", "box3d"):
            peak = glue_row_run(kind, 1e12, frames, **scene)[1]
            got[kind] = (peak, glue_row_threshold(kind, frames, **scene))
        (pb, tb), (px, tx) = got["bullet"], got["box3d"]
        ratio = px / pb if pb else float("inf")
        ok = ok and 0.5 <= ratio <= 2.0
        holds = tx / tb if tb and tx else None
        ok = ok and holds is not None and 0.7 <= holds <= 1.5

        def per_peak(peak, threshold):
            return "none" if not threshold or not peak else f"{threshold:.0f} ({threshold / peak:.1f}x)"

        model = ""
        if "drop" not in scene and tb:
            predicted = bullet_landing_hold(scene["n"], scene["height"], scene.get("bounce"))
            model = f", Bullet's pre-break model {predicted:.0f} ({predicted / tb:.2f} of it)"
        rows.append(f"{label}: impact Bullet {pb:.0f}, Box3D {px:.0f}; holds from Bullet {per_peak(pb, tb)}{model}, "
                    f"Box3D {per_peak(px, tx)}")
    return [Result("glue_break_match", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected Box3D's impacts from 0.5 to 2 times Bullet's, and the Strengths it holds "
                   "from 0.7 to 1.5 times)")]


def test_glue_break_substeps():
    """The Strength at which Glue stops breaking at Substeps 4 and 8, in Box3D and in Bullet (which estimates once a
    frame whatever the substeps are, SIM_BulletState::preBreakGlue): a landing and a box dropped on a row. The
    Strengths Box3D holds from must stay within 0.7 to 1.5 times Bullet's, as at the default (glue_break_match)."""
    scenes = [("landing from 3", dict(n=2, height=3.0), 48),
              ("row of 4 landing from 3", dict(n=4, height=3.0), 48),
              ("box dropped 1 on a pair", dict(n=2, drop=1.0), 72)]
    rows, ok = [], True
    for label, scene, frames in scenes:
        for substeps in (4, 8):
            held = {kind: glue_row_threshold(kind, frames, substeps=substeps, **scene) for kind in ("bullet", "box3d")}
            ratio = held["box3d"] / held["bullet"] if held["bullet"] and held["box3d"] else None
            ok = ok and ratio is not None and 0.7 <= ratio <= 1.5
            peaks = {kind: glue_row_run(kind, 1e12, frames, substeps=substeps, **scene)[1] for kind in held}
            rows.append(f"{label}, Substeps {substeps}: impact Bullet {peaks['bullet']:.0f}, Box3D {peaks['box3d']:.0f}; "
                        f"holds from Bullet {held['bullet']:.0f}, Box3D {held['box3d']:.0f}"
                        + ("" if ratio is None else f" ({ratio:.2f}x)"))
    return [Result("glue_break_substeps", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected Box3D from 0.7 to 1.5 times Bullet's)")]


def test_sidefx_hard_untyped():
    """SideFX Hard, Bullet Soft and Spring constraints with no constraint_type attribute (taken as 'position', as Bullet
    does), piece spinning at the start: distance and rotation after 2 s in Box3D and in Bullet. PASS if they agree, and
    the untyped line matches an explicit 'position' one in both solvers (and so not 'all', which turns less). The
    Spring is stiff (5000, damping 200, rest length 0) so that it holds the piece."""
    results = []
    stiff = {"strength": 5000.0, "damping": 200.0, "restlength": 0.0}
    for name, conrel, angle_tol, parms in (("hard", "hardconrel", 15, None), ("soft", "bulletsoftconrel", 10, None),
                                           ("spring", "springconrel", 10, stiff)):
        spin = (0.0, 3.0, 0.0)
        d = sidefx_compare(f"{name}_untyped", conrel, {}, spin=spin, conrel_parms=parms)
        p = sidefx_compare(f"{name}_position", conrel, {"constraint_type": "position"}, spin=spin, conrel_parms=parms)
        ok = (close(d["box3d"][0], d["bullet"][0], 0.1) and abs(d["box3d"][1] - d["bullet"][1]) < angle_tol
              and close(d["box3d"][0], p["box3d"][0], 0.01) and abs(d["box3d"][1] - p["box3d"][1]) < 1.0
              and abs(d["bullet"][1] - p["bullet"][1]) < 1.0)
        results.append(Result("sidefx_hard_untyped", Result.PASS if ok else Result.FAIL,
                              f"{name}: Box3D untyped distance {d['box3d'][0]:.3f}, turned {d['box3d'][1]:.0f} deg "
                              f"(explicit position {p['box3d'][0]:.3f}, {p['box3d'][1]:.0f} deg); Bullet untyped "
                              f"{d['bullet'][0]:.3f}, {d['bullet'][1]:.0f} deg (explicit position {p['bullet'][1]:.0f} deg)"))
    return results


TESTS = {
    "sidefx_glue": test_sidefx_glue,
    "glue_impact": test_glue_impact,
    "glue_strength_attribute": test_glue_strength_attribute,
    "glue_compound_break": test_glue_compound_break,
    "glue_compound_load": test_glue_compound_load,
    "glue_compound_vs_welds": test_glue_compound_vs_welds,
    "glue_compound_hit": test_glue_compound_hit,
    "glue_group_contacts": test_glue_group_contacts,
    "glue_compound_sleep": test_glue_compound_sleep,
    "glue_impact_scale": test_glue_impact_scale,
    "glue_global_iterations": test_glue_global_iterations,
    "glue_impact_trace": test_glue_impact_trace,
    "glue_fracture_vs_bullet": test_glue_fracture_vs_bullet,
    "glue_break_substeps": test_glue_break_substeps,
    "glue_compound_bridges": test_glue_compound_bridges,
    "glue_break_match": test_glue_break_match,
    "sidefx_hard_untyped": test_sidefx_hard_untyped,
}
