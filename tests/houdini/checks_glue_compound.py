"""Glue as One Body in Houdini: `compound_id`, a kinematic piece pushing, carrying and toppling glued pieces, and many
small compounds at once. The C++ side of the first of these is `core_compound`; these run the whole solver."""

import itertools
import time

import hou
from b3dtest import (
    box3d_data,
    GROUND,
    constraint_lines,
    create,
    finish,
    multisolver,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    Result,
    set_parm,
    solver_for,
    sop_solver,
    step,
)
from checks_constraints import broken_count
from checks_output import impact_records

_COUNTER = itertools.count()


def tag(prefix):
    return f"{prefix}_{next(_COUNTER)}"


def glue_scene(name, pieces, lines, strength, compound=True, keep_velocity=False, packed_parms=None, impacts=False,
               export_shapes=False):
    """A DOP network of packed boxes joined by Glue lines. pieces: as packed_boxes. lines: (piece index, point, piece
    index, point) each, the Glue's Strength `strength`. compound: the solver's Glue as One Body. impacts: Add Impact Data
    on the pieces. export_shapes: the solver's Export Collision Shapes. Returns (dop, piece names)."""
    dop = new_dopnet(name)
    path, names = packed_boxes(name + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path, keep_velocity=keep_velocity, **(packed_parms or {}))
    if impacts:
        obj = box3d_data(dop, obj, "data", addimpact=1)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], strength)
    spec = [(names[a], pa, names[b], pb, "Glue") for a, pa, b, pb in lines]
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(name + "_network", spec))
    net.setInput(1, glue)
    solver = solver_for("box3d", dop, net)
    set_parm(solver, ["gluecompound"], 1 if compound else 0)
    if export_shapes:
        set_parm(solver, ["exportshapes"], 1)
    finish(dop, solver)
    return dop, names


def move_piece(dop, vex):
    """A SOP Solver running `vex` (it moves a kinematic piece) ahead of the Box3D Solver."""
    out = dop.node("output")
    solver = out.inputs()[0]
    upstream = solver.inputs()[0]
    solver.setInput(0, None)
    out.setInput(0, multisolver(dop, upstream, [sop_solver(dop, vex), solver]))


def compound_ids(sim, f):
    """The `compound_id` of every point of the pieces object, in point order (None: the attribute does not exist)."""
    geo = sim.findObject("pieces").geometry()
    if geo.findPointAttrib("compound_id") is None:
        return None
    return [p.attribValue("compound_id") for p in geo.points()]


def test_glue_compound_id():
    """`compound_id` on the points of glued pieces run as one body, with Write Compound IDs on: the lowest point number
    in the compound for each piece of it (a row of 3 boxes glued together reads 0, 0, 0) and -1 for a loose one (a
    fourth box) and the ground. Off, the attribute does not exist. A 1000 kg box thrown at 10 m/s into a pair of glued
    boxes (a Strength of 2e4, which the resting pair holds) finds them in one compound before it arrives and out of it
    after: Glue as One Body lets the pieces go in a fast body's way. With a Strength of 1e3 the resting pair's own
    weight is a hit that breaks the bond, so they read -1 throughout."""
    rows, ok = [], True
    row = [{"size": (1, 1, 1), "center": (0.5 + i, 0.5, 0), "attrs": {"active": 1}} for i in range(3)]
    loose = {"size": (1, 1, 1), "center": (8.5, 0.5, 0), "attrs": {"active": 1}}
    lines = [(0, (1.0, 0.5, 0.0), 1, (1.0, 0.5, 0.0)), (1, (2.0, 0.5, 0.0), 2, (2.0, 0.5, 0.0))]
    for on in (True, False):
        dop, _names = glue_scene(tag("glue_cid"), row + [loose, GROUND], lines, 1e12)
        set_parm(dop.node("box3d_solver"), ["compoundids"], 1 if on else 0)
        got = step(dop, 6, compound_ids)[6]
        good = got is None if not on else got is not None and got[:5] == [0, 0, 0, -1, -1]
        ok = ok and good
        rows.append(f"{'on' if on else 'off'}: {got} ({'ok' if good else 'wrong'})")
    pieces = [{"size": (1, 1, 1), "center": (1.5, 0.5, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (2.5, 0.5, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (-4.5, 0.5, 0), "attrs": {"v": (10.0, 0.0, 0.0)}}, GROUND]
    for strength, want_before in ((2e4, [0, 0]), (1e3, [-1, -1])):
        dop, _names = glue_scene(tag("glue_cid_hit"), pieces, [(0, (2.0, 0.5, 0.0), 1, (2.0, 0.5, 0.0))], strength,
                                 keep_velocity=True)
        set_parm(dop.node("box3d_solver"), ["compoundids"], 1)
        track = step(dop, 24, compound_ids)
        before = track[4][:2] if track[4] else None
        after = track[24][:2] if track[24] else None
        good = before == want_before and after == [-1, -1]
        ok = ok and good
        rows.append(f"hit pair at Strength {strength:g}: {before} on frame 4, {after} on frame 24 "
                    f"({'ok' if good else 'wrong'}; expected {want_before}, [-1, -1])")
    return [Result("glue_compound_id", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def pusher_run(strength, compound, frames, speed):
    """A row of 4 glued boxes (1000 kg each) on the ground and a kinematic piece (animated, moved along x at `speed` by a
    SOP Solver) 2.5 m behind it. Returns (the row's x on the last frame, the kinematic piece's x, broken lines)."""
    pieces = [dict(GROUND, attrs={"active": 0, "animated": 0})]
    pieces += [{"size": (1, 1, 1), "center": (float(i), 0.5, 0), "attrs": {"active": 1, "animated": 0}}
               for i in range(4)]
    pieces += [{"size": (1, 1, 1), "center": (-2.5, 0.5, 0), "attrs": {"active": 0, "animated": 1}}]
    lines = [(1 + i, (i + 0.5, 0.5, 0.0), 2 + i, (i + 0.5, 0.5, 0.0)) for i in range(3)]
    dop, names = glue_scene(tag("glue_pusher"), pieces, lines, strength, compound=compound)
    move_piece(dop, f'if (s@name == "{names[5]}") @P.x = -2.5 + {speed} * @Time;')

    def look(sim, f):
        if f != frames:
            return None
        pos = packed_positions(sim, "pieces")
        return [pos[n][0] for n in names[1:5]], pos[names[5]][0], broken_count(sim) or 0

    return step(dop, frames, look)[frames]


def test_glue_compound_kinematic():
    """A kinematic piece (animated, moved along x at 3 m/s by a SOP Solver) and glued pieces, with Glue as One Body on
    and off. (1) It pushes a row of 4 glued boxes (1000 kg each) standing on the ground. At a Strength that never breaks
    them the row is pushed along as one: after 2 s the first box is 1 m ahead of the pusher and the others 1 m apart
    (within 0.3), nothing breaks, and the two modes agree within 0.3. At a Strength of 1 the pusher, which has no
    momentum limit, breaks every bond in both modes (it does not stall against the compound, nor is it thrown back: it
    keeps the animation within 0.05), and every box ends ahead of it, the mean of the row within 1.5 m between the
    modes. (2) Two boxes glued in a row to a kinematic piece: a bond to a body that does not simulate has a side that
    never moves, so the group is not made one body, and the boxes are carried: the first is 1 m and the second 2 m from
    the kinematic piece (within 0.3), nothing breaks. (3) The piece hits the top of a column of 4 glued boxes (a Strength
    of 5e3, which the standing column holds; RBD Bullet Solver breaks 2 of the 3 bonds by frame 24 and the third later):
    none is broken on frame 2, all 3 are on frame 24, and the boxes are in the same place in both modes within 0.1."""
    original = hou.fps()
    hou.setFps(24)
    results = []
    speed, frames = 3.0, 48
    try:
        # (1) pushed
        runs = {(label, compound): pusher_run(strength, compound, frames, speed)
                for label, strength in (("strong", 1e12), ("weak", 1.0)) for compound in (True, False)}
        want_pusher = -2.5 + speed * ((frames - 1) / hou.fps())  # @Time of the last frame
        rows, ok = [], True
        for label in ("strong", "weak"):
            (on, p_on, b_on), (off, p_off, b_off) = runs[(label, True)], runs[(label, False)]
            good = abs(p_on - want_pusher) < 0.05 and abs(p_off - want_pusher) < 0.05
            if label == "strong":
                good = good and b_on == 0 and b_off == 0
                for row, pusher in ((on, p_on), (off, p_off)):
                    good = good and all(abs(row[i] - (row[0] + i)) < 0.3 for i in range(4))
                    good = good and abs(row[0] - (pusher + 1.0)) < 0.3
                good = good and all(abs(a - b) < 0.3 for a, b in zip(on, off))
            else:
                good = good and b_on == 3 and b_off == 3
                good = good and min(on) > p_on and min(off) > p_off
                good = good and abs(sum(on) / 4 - sum(off) / 4) < 1.5
            ok = ok and good
            rows.append(f"{label}: one body row {[round(x, 2) for x in on]}, pusher {p_on:.2f}, {b_on} broken; welds row "
                        f"{[round(x, 2) for x in off]}, pusher {p_off:.2f}, {b_off} broken ({'ok' if good else 'WRONG'})")
        results.append(Result("glue_compound_kinematic", Result.PASS if ok else Result.FAIL,
                              "pushed: " + "; ".join(rows)))

        # (2) carried
        pieces = [dict(GROUND, attrs={"active": 0, "animated": 0}),
                  {"size": (1, 1, 1), "center": (0.0, 0.5, 0), "attrs": {"active": 0, "animated": 1}},
                  {"size": (1, 1, 1), "center": (1.0, 0.5, 0), "attrs": {"active": 1, "animated": 0}},
                  {"size": (1, 1, 1), "center": (2.0, 0.5, 0), "attrs": {"active": 1, "animated": 0}}]
        lines = [(1, (0.5, 0.5, 0.0), 2, (0.5, 0.5, 0.0)), (2, (1.5, 0.5, 0.0), 3, (1.5, 0.5, 0.0))]
        rows, ok = [], True
        for compound in (True, False):
            dop, names = glue_scene(tag("glue_carry"), pieces, lines, 1e12, compound=compound)
            move_piece(dop, f'if (s@name == "{names[1]}") @P.x = {speed} * @Time;')

            def look(sim, f, names=names):
                if f != frames:
                    return None
                pos = packed_positions(sim, "pieces")
                return pos[names[1]][0], pos[names[2]][0], pos[names[3]][0], broken_count(sim) or 0

            k, b, c, broken = step(dop, frames, look)[frames]
            want = speed * (frames - 1) / hou.fps()
            good = abs(k - want) < 0.05 and abs(b - k - 1.0) < 0.3 and abs(c - k - 2.0) < 0.3 and broken == 0
            ok = ok and good
            rows.append(f"{'one body' if compound else 'welds'}: kinematic {k:.2f} (animation {want:.2f}), boxes "
                        f"{b - k:+.2f} and {c - k:+.2f} from it, {broken} broken ({'ok' if good else 'WRONG'})")
        results.append(Result("glue_compound_kinematic", Result.PASS if ok else Result.FAIL,
                              "carried: " + "; ".join(rows)))

        # (3) toppled
        pieces = [dict(GROUND, attrs={"active": 0, "animated": 0})]
        pieces += [{"size": (1, 1, 1), "center": (0.0, 0.5 + i, 0), "attrs": {"active": 1, "animated": 0}}
                   for i in range(4)]
        pieces += [{"size": (1, 1, 1), "center": (-2.5, 3.5, 0), "attrs": {"active": 0, "animated": 1}}]
        lines = [(1 + i, (0.0, 1.0 + i, 0.0), 2 + i, (0.0, 1.0 + i, 0.0)) for i in range(3)]
        got = {}
        for compound in (True, False):
            dop, names = glue_scene(tag("glue_topple"), pieces, lines, 5e3, compound=compound)
            move_piece(dop, f'if (s@name == "{names[5]}") @P.x = -2.5 + {speed} * @Time;')

            def look(sim, f, names=names):
                if f not in (2, 24):
                    return None
                pos = packed_positions(sim, "pieces")
                return [(pos[n][0], pos[n][1]) for n in names[1:5]], broken_count(sim) or 0

            got[compound] = step(dop, 24, look)
        (on24, b_on), (off24, b_off) = got[True][24], got[False][24]
        standing = got[True][2][1], got[False][2][1]
        gap = max(max(abs(a - b) for a, b in zip(p, q)) for p, q in zip(on24, off24))
        ok = standing == (0, 0) and b_on == 3 and b_off == 3 and gap < 0.1
        results.append(Result("glue_compound_kinematic", Result.PASS if ok else Result.FAIL,
                              f"toppled: {standing[0]} / {standing[1]} broken on frame 2 and {b_on} / {b_off} on frame 24 "
                              f"(one body / welds; expected 0 and 3), the boxes differ by at most {gap:.3f} m on frame 24 "
                              f"(expected under 0.1)"))
    finally:
        hou.setFps(original)
    return results


def kinematic_hit_broken(kind, strength, speed):
    """Glue bonds broken on frame 40 when a kinematic piece (animated, moved along x at `speed` by a SOP Solver) pushes two
    glued boxes standing in a row (1 kg each) end-on, in Box3D (`kind` box3d, Glue as One Body on) or RBD Bullet Solver."""
    pieces = [dict(GROUND, attrs={"active": 0, "animated": 0}),
              {"size": (1, 1, 1), "center": (0.0, 0.5, 0), "attrs": {"active": 1, "animated": 0}},
              {"size": (1, 1, 1), "center": (1.0, 0.5, 0), "attrs": {"active": 1, "animated": 0}},
              {"size": (1, 1, 1), "center": (-2.5, 0.5, 0), "attrs": {"active": 0, "animated": 1}}]
    name = tag(f"glue_kinhit_{kind}")
    dop = new_dopnet(name)
    path, names = packed_boxes(name + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path)
    glue = create(dop, "glueconrel", "glue")
    set_parm(glue, ["strength"], strength)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"],
             constraint_lines(name + "_network", [(names[1], (0.5, 0.5, 0.0), names[2], (0.5, 0.5, 0.0), "Glue")]))
    net.setInput(1, glue)
    mover = sop_solver(dop, f'if (s@name == "{names[3]}") @P.x = -2.5 + {speed} * @Time;')
    if kind == "box3d":
        solver = solver_for("box3d", dop, None)
        set_parm(solver, ["gluecompound"], 1)
        last = multisolver(dop, net, [mover, solver])
    else:
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, net, [create(dop, "bulletrbdsolver", "bullet_solver"), mover]))
    finish(dop, last)
    return step(dop, 40, lambda sim, f: (broken_count(sim) or 0) if f == 40 else None)[40]


def test_glue_kinematic_hit():
    """RBD Bullet Solver's Glue counts nothing of a kinematic piece's push, however fast: a piece moved along x by a SOP Solver
    into two glued boxes (1 kg each) breaks their bond at a Strength of 4e3 and not at 8e3, at 3 and at 6 m/s alike. Box3D
    does the same (it leaves a kinematic body's contacts out of the hits and the estimate), where counting them broke
    the bond up to 2e4 at 3 m/s and up to 1e5 at 6."""
    original = hou.fps()
    hou.setFps(24)
    rows, ok = [], True
    try:
        for speed in (3.0, 6.0):
            for strength, want in ((4e3, 1), (8e3, 0)):
                got = {kind: kinematic_hit_broken(kind, strength, speed) for kind in ("box3d", "bullet")}
                good = got["box3d"] == want and got["bullet"] == want
                ok = ok and good
                rows.append(f"{speed:g} m/s, Strength {strength:g}: broken Box3D {got['box3d']}, Bullet {got['bullet']}"
                            f"{'' if good else ' (WRONG)'}")
    finally:
        hou.setFps(original)
    return [Result("glue_kinematic_hit", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected 1 at 4e3 and 0 at 8e3 in both)")]


def test_glue_compound_many_groups():
    """Many small glued groups at once. 400 groups of 3 boxes (1000 kg each, Glue between neighbours, a Strength that never
    breaks them) on a 20 x 20 grid 4 m apart, dropped 1.5 m on the ground without bounce, with Glue as One Body on and
    off. With it on every group is its own compound (Write Compound IDs shows `compound_id`: 400 different numbers, the
    three pieces of a group sharing theirs) and with it off none is. Both ways every box lands on the ground and
    stays where its group put it (y 0.5 and its x within 0.05 of the start, nothing broken) and the two modes agree
    within 0.05. INFO: the seconds each took from frame 2 to 36."""
    original = hou.fps()
    hou.setFps(24)
    side, per, frames = 20, 3, 36
    pieces = [{"size": (100, 1, 100), "center": (0, -0.5, 0), "attrs": {"active": 0}}]
    lines = []
    for gz in range(side):
        for gx in range(side):
            x0, z0 = 4.0 * (gx - side / 2), 4.0 * (gz - side / 2)
            first = len(pieces)
            for k in range(per):
                pieces.append({"size": (1, 1, 1), "center": (x0 + k, 2.0, z0), "attrs": {"active": 1}})
            lines += [(first + k, (x0 + k + 0.5, 2.0, z0), first + k + 1, (x0 + k + 0.5, 2.0, z0))
                      for k in range(per - 1)]
    results, seconds = {}, {}
    try:
        for compound in (True, False):
            dop, names = glue_scene(tag("glue_groups"), pieces, lines, 1e12, compound=compound,
                                    packed_parms={"bounce": 0.0})
            set_parm(dop.node("box3d_solver"), ["compoundids"], 1)

            marks = {}

            def look(sim, f, marks=marks):
                marks[f] = time.time()
                if f != frames:
                    return None
                geo = sim.findObject("pieces").geometry()
                ids = ({pt.attribValue("name"): pt.attribValue("compound_id") for pt in geo.points()}
                       if geo.findPointAttrib("compound_id") is not None else {})
                return packed_positions(sim, "pieces"), ids, broken_count(sim) or 0

            names = names[1:]  # the boxes, the ground being first
            results[compound] = step(dop, frames, look)[frames]
            # From frame 1 on: the first cook compiles the VEX that builds the pieces and lines (a line per piece), whose
            # time is the same in both modes but is paid only by the first, the compiled code being cached.
            seconds[compound] = marks[frames] - marks[1]
    finally:
        hou.setFps(original)
    rows, ok = [], True
    for compound in (True, False):
        pos, ids, broken = results[compound]
        landed = all(abs(pos[n][1] - 0.5) < 0.05 for n in names)
        in_place = True
        for g in range(side * side):
            for k in range(per):
                x0 = 4.0 * (g % side - side / 2)
                in_place = in_place and abs(pos[names[g * per + k]][0] - (x0 + k)) < 0.05
        group_ids = [{ids.get(names[g * per + k]) for k in range(per)} for g in range(side * side)]
        if compound:
            shared = all(len(s) == 1 and None not in s and next(iter(s)) >= 0 for s in group_ids)
            distinct = len({next(iter(s)) for s in group_ids if len(s) == 1})
            grouped = shared and distinct == side * side
        else:
            grouped = all(v < 0 for v in ids.values())
        good = len(names) == side * side * per and landed and in_place and broken == 0 and grouped
        ok = ok and good
        rows.append(f"{'one body' if compound else 'welds'}: {len(names)} pieces, landed {landed}, in place {in_place}, "
                    f"{broken} broken, ids as expected {grouped} ({'ok' if good else 'WRONG'})")
    gap = max(abs(results[True][0][n][i] - results[False][0][n][i]) for n in names for i in range(3))
    ok = ok and gap < 0.05
    rows.append(f"the two modes differ by at most {gap:.3f} m (expected under 0.05)")
    return [Result("glue_compound_many_groups", Result.PASS if ok else Result.FAIL, "; ".join(rows)),
            Result("glue_compound_many_groups", Result.INFO,
                   f"frames 2 to {frames} took {seconds[True]:.2f} s as one body each and {seconds[False]:.2f} s on welds")]


def landing_row(name, compound, **scene):
    """A row of 3 glued boxes (a Strength that never breaks them) dropped 1 m onto the ground."""
    pieces = [{"size": (1, 1, 1), "center": (float(i), 1.5, 0), "attrs": {"active": 1}} for i in range(3)]
    pieces.append(dict(GROUND, attrs={"active": 0}))
    lines = [(i, (i + 0.5, 1.5, 0.0), i + 1, (i + 0.5, 1.5, 0.0)) for i in range(2)]
    return glue_scene(tag(name), pieces, lines, 1e12, compound=compound, packed_parms={"bounce": 0.0}, **scene)


def test_glue_compound_guide():
    """The collision guide (Export Collision Shapes, read back through a DOP Import of `Box3DShapes`) follows glued pieces
    run as one body as it does welds: a row of 3 boxes landing on the ground has the same 24 polygons (4 bodies of 6
    faces) in both modes on every frame, and the top of the lines is half a box above the highest piece (within
    1e-3) on frames 6 (falling), 14 and 30 (landed)."""
    rows, ok = [], True
    for compound in (True, False):
        dop, _names = landing_row("glue_guide", compound, export_shapes=True)
        geo = new_geo(tag("glue_guide_read"))
        imp = create(geo, "dopimport", "import_shapes")
        set_parm(imp, ["doppath"], dop.path())
        set_parm(imp, ["objpattern"], "pieces")
        set_parm(imp, ["geodatapath"], "Box3DShapes")
        step(dop, 30, lambda sim, f: None)
        seen = []
        for f in (6, 14, 30):
            hou.setFrame(f)
            lines = imp.geometry()
            top = max(pt.position()[1] for pt in lines.points())
            body = dop.simulation().findObject("pieces").geometry()
            highest = max(pt.position()[1] for pt in body.points() if pt.position()[1] > 0.0)
            good = len(lines.prims()) == 24 and abs(top - (highest + 0.5)) < 1e-3
            ok = ok and good
            seen.append(f"frame {f}: {len(lines.prims())} polygons, top {top:.3f} against {highest + 0.5:.3f}")
        rows.append(f"{'one body' if compound else 'welds'}: " + "; ".join(seen))
    return [Result("glue_compound_guide", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected 24 polygons and the top half a box above the highest piece)")]


def test_glue_compound_impacts():
    """Add Impact Data on glued pieces: a row of 3 boxes landing on the ground records impacts on every piece's point
    (and the ground's) both as welds and run as one body, where each record is the piece the contact was on. What the
    pieces take from the ground adds up to about the same in both (within 10%: how a rigid body shares the landing
    between its pieces differs from welds, so the pieces' own shares are not compared)."""
    totals, points = {}, {}
    for compound in (True, False):
        dop, _names = landing_row("glue_impacts", compound, impacts=True)
        track = step(dop, 30, lambda sim, f: impact_records(sim, "pieces"))
        records = [r for f in sorted(track) for r in track[f]]
        points[compound] = sorted({r["ptnum"] for r in records})
        # The ground is point 3: what the three pieces took from it.
        totals[compound] = sum(r["impulse"] for r in records if r["ptnum"] in (0, 1, 2) and r["otherobjptnum"] == 3)
    ok = (all(points[c] == [0, 1, 2, 3] for c in points) and totals[False] > 0
          and abs(totals[True] - totals[False]) < 0.1 * totals[False])
    return [Result("glue_compound_impacts", Result.PASS if ok else Result.FAIL,
                   f"records on points {points[True]} as one body and {points[False]} as welds (expected 0 to 3 in both); "
                   f"the pieces took {totals[True]:.0f} N*s from the ground as one body and {totals[False]:.0f} as welds "
                   f"(expected within 10%)")]


TESTS = {
    "glue_compound_guide": test_glue_compound_guide,
    "glue_compound_impacts": test_glue_compound_impacts,
    "glue_compound_id": test_glue_compound_id,
    "glue_compound_kinematic": test_glue_compound_kinematic,
    "glue_kinematic_hit": test_glue_kinematic_hit,
    "glue_compound_many_groups": test_glue_compound_many_groups,
}
