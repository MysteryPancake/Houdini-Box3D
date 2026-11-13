"""The remaining scriptable checks: messages clearing, saved caches, point order, an edited packed
source, SOP Solver edits, Time Scale with kinematic bodies, attribute toggles against Bullet, DOP
Import output, dead pieces in a constraint network, an animated Static Object, objects from another
solver, Attach Internal Constraints, and SideFX Glue / Hard / Cone Twist / aging compared with
Bullet and the POP Solver."""

import os
import tempfile

import hou

from b3dtest import (
    Result,
    SetupError,
    append_sop,
    box3d_data,
    box3d_solver,
    bullet_solver,
    close,
    create,
    display,
    finish,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    object_bbox,
    packed_boxes,
    packed_object,
    packed_positions,
    plain_points,
    point_attributes,
    set_expression,
    set_menu_by_label,
    set_parm,
    sop_solver,
    sop_solver_contents,
    step,
)
from checks_constraints import hanging_pair, sidefx_compare
from checks_solver import max_difference, stack_scene

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}


def first_frame(track, test):
    """The first frame whose sample passes test(sample), or None."""
    return next((f for f in sorted(track) if test(track[f])), None)


def bullet_solver_alone(dop, name="bullet_solver"):
    return create(dop, "bulletrbdsolver", name)


# ---------------------------------------------------------------------------
# Node messages clear once the problem is gone
# ---------------------------------------------------------------------------


def test_messages_clear():
    """A constraint line naming no joint node until frame 10 (re-imported each step with Overwrite
    with SOP): the solver warns on early frames and the warning is gone once the line names the weld."""
    def animate_name(path):
        def setup(node):
            set_parm(node, ["class"], 1)  # primitives
            set_parm(node, ["snippet"], 's@constraint_name = @Frame < 10 ? "nothing" : "weld";')
        return append_sop(path, "attribwrangle", setup)

    dop, names = hanging_pair("messages_clear", "weld", conrels=[("box3d_weldconrel", {})],
                              network_edit=animate_name,
                              net_parms={("reloadfromsops", "overwrite", "overwritewithsop"): 1})
    solver = dop.node("box3d_solver")
    track = step(dop, 20, lambda sim, f: [m for m in node_messages(solver) if "Box3D" in m])
    early = [m for f in range(2, 9) for m in track[f]]
    late = [m for f in range(15, 21) for m in track[f]]
    ok = bool(early) and not late
    return [Result("messages_clear", Result.PASS if ok else Result.FAIL,
                   f"frames 2-8: {early[:1] or 'no message'}; frames 15-20: {late[:1] or 'no message'} "
                   "(expected a message early and none late)")]


# ---------------------------------------------------------------------------
# Saved caches: checkpoints written to disk, the .hip reopened, simulation continued
# ---------------------------------------------------------------------------


def test_saved_cache():
    """Write checkpoints to disk while playing to frame 24, save and reopen the .hip, then play on
    from frame 24 to 36: the result matches a straight run."""
    dop, _ = stack_scene("straight")
    straight = step(dop, 36, lambda sim, f: packed_positions(sim, "pieces"))

    folder = tempfile.mkdtemp(prefix="box3d_cache_")
    dop2, _ = stack_scene("saved")
    set_parm(dop2, ["explicitcache", "savecheckpoints"], 1)
    set_parm(dop2, ["explicitcachename", "checkpointname"], os.path.join(folder, "ckpt.$SF.sim").replace("\\", "/"))
    try:
        set_parm(dop2, ["explicitcachensteps", "checkpointtrail"], 1)
    except SetupError:
        pass  # optional: how often to write a checkpoint
    step(dop2, 24, lambda sim, f: None)
    written = [f for f in os.listdir(folder) if f.endswith(".sim")]
    hip = os.path.join(folder, "saved.hip")
    hou.hipFile.save(hip)
    hou.hipFile.load(hip, suppress_save_prompt=True)
    dop2 = hou.node("/obj/saved")
    resumed = {}
    for f in range(24, 37):
        hou.setFrame(f)
        resumed[f] = packed_positions(dop2.simulation(), "pieces")
    d24, d36 = max_difference(resumed[24], straight[24]), max_difference(resumed[36], straight[36])
    ok = written and d36 < 1e-2
    return [Result("saved_cache", Result.PASS if ok else Result.FAIL,
                   f"{len(written)} checkpoint file(s) written; after reopening: frame 24 differs by {d24:.2e}, "
                   f"frame 36 by {d36:.2e} from a straight run")]


# ---------------------------------------------------------------------------
# Point order: a Sort SOP upstream, with and without id
# ---------------------------------------------------------------------------


def test_point_order():
    """Reversing the point order upstream (Sort SOP), with and without `id`, gives every named
    piece the same result: bodies keep their identity."""
    def pieces(with_id):
        # Resting pieces sliding apart at different speeds (no drop: tumbling turns tiny order
        # differences into visible ones).
        out = []
        for i in range(3):
            attrs = {"active": 1, "v": (1.0 + i, 0.0, 0.0)}
            if with_id:
                attrs["id"] = 10 + i
            out.append({"size": (0.5, 0.5, 0.5), "center": (3.0 * i, 0.25, 0), "attrs": attrs})
        return [dict(GROUND, attrs={"active": 0, "v": (0.0, 0.0, 0.0), **({"id": 99} if with_id else {})})] + out

    runs = {}
    for label, with_id, reverse in (("plain", False, False), ("reversed", False, True), ("reversed+id", True, True)):
        tag = label.replace("+", "_")
        path, _ = packed_boxes("order_" + tag, pieces(with_id))
        if reverse:
            path = append_sop(path, "sort", lambda n: set_menu_by_label(n, ["ptsort"], "Reverse"))
        dop = new_dopnet("order_" + tag)
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True, bounce=0.0)))
        runs[label] = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))[48]
    diffs = {k: max_difference(runs[k], runs["plain"]) for k in ("reversed", "reversed+id")}
    # Identity: every piece ends nearest its own unsorted result (the pieces end about 3 apart). Exact
    # equality is not expected: bodies are created in point order and Box3D's results depend
    # slightly on body order (status.md, Points and packed geometry).
    def keeps_identity(run):
        for n, p in run.items():
            nearest = min(runs["plain"], key=lambda k: (runs["plain"][k] - p).length())
            if nearest != n:
                return False
        return True

    ok = all(keeps_identity(runs[k]) for k in ("reversed", "reversed+id"))

    def detail(run):
        return ", ".join(f"{n}: {tuple(round(c, 2) for c in runs['plain'][n])} -> {tuple(round(c, 2) for c in run[n])}"
                         for n in sorted(run) if (run[n] - runs["plain"][n]).length() >= 1e-3)

    results = [Result("point_order", Result.PASS if ok else Result.FAIL,
                      "every piece keeps its identity when the points are reversed, with and without id"
                      if ok else f"a piece swapped identity: {detail(runs['reversed'])}")]
    results.append(Result("point_order", Result.INFO,
                          f"order sensitivity: largest per-piece difference from the unsorted run, reversed "
                          f"{diffs['reversed']:.2e}, reversed with id {diffs['reversed+id']:.2e}"
                          + (f" ({detail(runs['reversed'])})" if diffs["reversed"] >= 1e-3 else "")))
    return results


# ---------------------------------------------------------------------------
# An edited packed source: the new shape is used (status: SourceSummaryCache)
# ---------------------------------------------------------------------------


def test_edited_source():
    """A convex-hull piece whose source box is resized from 1 to 2 between two runs in one session
    rests 2 tall on the second run (no stale hull from the cache)."""
    geo = new_geo("edited_piece")
    box = create(geo, "box")
    asm = create(geo, "assemble")
    asm.setInput(0, box)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)
    xf = create(geo, "xform")
    xf.setInput(0, asm)
    set_parm(xf, ["t"], (0, 3, 0))
    display(xf)
    dop = new_dopnet("edited")
    obj = packed_object(dop, "piece", xf.path())
    data = box3d_data(dop, obj, "data")
    set_menu_by_label(data, ["defaultshape"], "Convex Hull")
    ground_path, _ = packed_boxes("edited_ground", [GROUND])
    finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "ground", ground_path), data])))
    first = step(dop, 72, lambda sim, f: object_bbox(sim, "piece"))[72]
    set_parm(box, ["size"], (2, 2, 2))
    second = step(dop, 72, lambda sim, f: object_bbox(sim, "piece"))[72]
    h1, h2 = first[1][1] - first[0][1], second[1][1] - second[0][1]
    top2 = second[1][1]
    ok = close(h1, 1.0, 0.03) and close(h2, 2.0, 0.05) and close(top2, 2.0, 0.05)
    return [Result("edited_source", Result.PASS if ok else Result.FAIL,
                   f"first run rests {h1:.3f} tall; after resizing the source to 2, {h2:.3f} tall with its top at "
                   f"{top2:.3f} (expected 1, then 2 and 2)")]


# ---------------------------------------------------------------------------
# A SOP Solver moving a piece mid-simulation (status: fingerprint skip; packed points teleport)
# ---------------------------------------------------------------------------


def test_sop_solver_edit():
    """A SOP Solver (after the Box3D Solver in a Multiple Solver) moves a resting piece 5 along x at
    frame 12: the body teleports there and stays (the solver picked the edit up instead of writing
    its old pose back)."""
    path, names = packed_boxes("sopedit_pieces", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0)}])
    dop = new_dopnet("sopedit")
    obj = packed_object(dop, "pieces", path)
    edit = sop_solver(dop, f'if (s@name == "{names[1]}" && abs(@Frame - 12) < 0.5) @P.x += 5;')
    contents = sop_solver_contents(edit)
    finish(dop, multisolver(dop, obj, [box3d_solver(dop, None), edit]))
    track = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces")[names[1]])
    x11, x13, x24, y24 = track[11][0], track[13][0], track[24][0], track[24][1]
    ok = close(x11, 0.0, 0.05) and close(x13, 5.0, 0.05) and close(x24, 5.0, 0.05) and close(y24, 0.5, 0.05)
    return [Result("sop_solver_edit", Result.PASS if ok else Result.FAIL,
                   f"x: {x11:.2f} on frame 11, {x13:.2f} on frame 13, {x24:.2f} on frame 24 (expected 0, 5, 5); "
                   f"resting at y={y24:.2f}" + ("" if ok else f". SOP Solver contents: {contents}"))]


# ---------------------------------------------------------------------------
# Kinematic (animated) bodies with Time Scale != 1 (status: animated bodies)
# ---------------------------------------------------------------------------


def test_time_scale_kinematic():
    """A kinematic piece (animated = 1) moved along x at 1 unit/s by a SOP Solver pushes a box. With
    Time Scale 1 and 2: does the kinematic body track its animation, and how far does the box go?
    PASS if it tracks in both; the push distances are INFO."""
    rows, ok = [], True
    for ts in (1.0, 2.0):
        tag = f"ts{int(ts)}"
        pieces = [GROUND,
                  {"size": (1, 1, 1), "center": (0, 0.5, 0), "attrs": {"active": 0, "animated": 1}},
                  {"size": (1, 1, 1), "center": (1.6, 0.5, 0), "attrs": {"active": 1, "animated": 0}}]
        path, names = packed_boxes("kinematic_" + tag, pieces)
        dop = new_dopnet("kinematic_" + tag)
        obj = packed_object(dop, "pieces", path)
        mover = sop_solver(dop, f'if (s@name == "{names[1]}") @P.x = @Time;')
        contents = sop_solver_contents(mover)
        solver = box3d_solver(dop, None)
        set_parm(solver, ["timescale"], ts)
        finish(dop, multisolver(dop, obj, [mover, solver]))
        track = step(dop, 49, lambda sim, f: packed_positions(sim, "pieces"))
        kx, bx = track[49][names[1]][0], track[49][names[2]][0]
        t = 48 / hou.fps()
        rows.append(f"Time Scale {ts:g}: kinematic x {kx:.2f} (animation {t:.2f}), pushed box x {bx:.2f}")
        ok = ok and close(kx, t, 0.05)
    return [Result("time_scale_kinematic", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + ("" if ok else f". SOP Solver contents: {contents}"))]


# ---------------------------------------------------------------------------
# Attribute toggles mid-simulation, against Bullet (status: Bullet's attribute semantics)
# ---------------------------------------------------------------------------


def test_attribute_toggles():
    """A piece held in the air with active = 0 is switched to active = 1 at frame 12 by a SOP
    Solver: it starts falling then, in Box3D and in Bullet. Also (Box3D only) awake 0 -> 1."""
    results, falls = [], {}
    for kind in ("box3d", "bullet"):
        path, names = packed_boxes(f"toggle_{kind}", [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"active": 0}}])
        dop = new_dopnet(f"toggle_{kind}")
        obj = packed_object(dop, "piece", path)
        flip = sop_solver(dop, "if (@Frame >= 12) i@active = 1;")
        solver = box3d_solver(dop, None) if kind == "box3d" else bullet_solver_alone(dop)
        last = multisolver(dop, obj, [solver, flip])
        if kind == "bullet":
            grav = create(dop, "gravity", "gravity")
            grav.setInput(0, last)
            last = grav
        finish(dop, last)
        track = step(dop, 24, lambda sim, f: packed_positions(sim, "piece")[names[0]][1])
        falls[kind] = first_frame(track, lambda y: y < 2.99)
    ok = falls["box3d"] is not None and 12 <= falls["box3d"] <= 14 and falls["bullet"] is not None \
        and abs(falls["box3d"] - falls["bullet"]) <= 1
    results.append(Result("attribute_toggles", Result.PASS if ok else Result.FAIL,
                          f"active 0 -> 1 at frame 12: first frame lower in Box3D {falls['box3d']}, in Bullet "
                          f"{falls['bullet']}"))

    path, names = packed_boxes("toggle_awake", [{"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"awake": 0}}])
    dop = new_dopnet("toggle_awake")
    obj = packed_object(dop, "piece", path)
    wake = sop_solver(dop, "if (@Frame >= 12) i@awake = 1;")
    finish(dop, multisolver(dop, obj, [box3d_solver(dop, None), wake]))
    track = step(dop, 24, lambda sim, f: packed_positions(sim, "piece")[names[0]][1])
    fell = first_frame(track, lambda y: y < 2.99)
    results.append(Result("attribute_toggles", Result.PASS if fell is not None and 12 <= fell <= 14 else Result.FAIL,
                          f"awake 0 -> 1 at frame 12 (Box3D): first frame lower {fell} (expected 12-14)"))
    return results


# ---------------------------------------------------------------------------
# DOP Import output (status: downstream readers)
# ---------------------------------------------------------------------------


def test_dop_import():
    """A DOP Import SOP (Fetch Geometry) gives the same packed piece positions as the simulation."""
    path, names = packed_boxes("import_pieces", [GROUND, {"size": (1, 1, 1), "center": (0, 3, 0)},
                                                 {"size": (1, 1, 1), "center": (2, 4, 0)}])
    dop = new_dopnet("import_dop")
    finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path)))
    geo = new_geo("import_result")
    imp = create(geo, "dopimport")
    set_parm(imp, ["doppath"], dop.path())
    set_parm(imp, ["objpattern"], "pieces")
    set_menu_by_label(imp, ["importstyle"], "Fetch")
    display(imp)
    sim_pos = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24]
    imp.cook(force=True)
    g = imp.geometry()
    has_name = g.findPointAttrib("name") is not None
    got = {(pt.attribValue("name") if has_name else str(pt.number())): pt.position() for pt in g.points()}
    common = [k for k in sim_pos if k in got]
    if not common:
        return [Result("dop_import", Result.FAIL, f"DOP Import gave no matching pieces (got {sorted(got)[:5]})")]
    d = max((got[k] - sim_pos[k]).length() for k in common)
    return [Result("dop_import", Result.PASS if d < 1e-5 else Result.FAIL,
                   f"{len(common)} pieces compared on frame 24; largest difference {d:.2e}")]


# ---------------------------------------------------------------------------
# Dead pieces named by a constraint network (status: dead deletion)
# ---------------------------------------------------------------------------


def test_dead_with_network():
    """With Enable Aging, a welded piece whose life runs out is marked dead (or deleted) while the
    constraint network still names it: no errors from the solver."""
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0, "life": 0.0, "age": 0.0}},
              {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1, "life": 0.25, "age": 0.0}}]
    dop, names = hanging_pair("dead_network", "weld", conrels=[("box3d_weldconrel", {})], pieces=pieces)
    solver = dop.node("box3d_solver")
    set_parm(solver, ["enableaging"], 1)
    errors = []

    def sample(sim, f):
        errors.extend(solver.errors())
        return point_attributes(sim, "pieces", names[1], ["name", "dead"])

    track = step(dop, 24, sample)
    last = track[24]
    state = "deleted" if last is None or last.get("name") != names[1] else f"dead={last.get('dead')}"
    return [Result("dead_with_network", Result.FAIL if errors else Result.PASS,
                   f"the welded piece is {state} on frame 24; solver errors: {errors or 'none'}")]


# ---------------------------------------------------------------------------
# An animated Static Object (status: SIMgetGeometryTransform at the right time)
# ---------------------------------------------------------------------------


def test_animated_static():
    """A Static Object grid that rises to y = 1 over the first second (Use Object Transform): a box
    dropped from y = 8 lands on it and rests at 1.5. Also reports the collider's transform as
    Houdini gives it, and tries again with Use Deforming Geometry on."""
    results = []
    for deforming, sop_level in ((False, False), (True, False), (True, True)):
        tag = ("sop" if sop_level else "deforming") if deforming else "plain"
        geo = new_geo("rising_grid_" + tag)
        grid = create(geo, "grid")
        set_parm(grid, ["size"], (10, 10))
        if sop_level:
            # Animated inside the SOP network (a Transform SOP), not by the object's transform.
            xf = create(geo, "xform")
            xf.setInput(0, grid)
            set_expression(xf, ["ty"], "min($T, 1)")
            display(xf)
            grid = xf
        else:
            display(grid)
            set_expression(geo, ["ty"], "min($T, 1)")
        dop = new_dopnet("animated_static_" + tag)
        static = create(dop, "staticobject", "collider")
        set_parm(static, ["soppath"], grid.path())
        set_parm(static, ["usetransform"], 1)
        if deforming:
            try:
                set_parm(static, ["animategeo", "usedeforminggeo", "usedeforming"], 1)
            except SetupError as e:
                results.append(Result("animated_static", Result.INFO, f"no Use Deforming Geometry toggle: {e}"))
                continue
        path, names = packed_boxes("animated_static_box_" + tag, [{"size": (1, 1, 1), "center": (0, 8, 0)}])
        finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "box", path), static])))

        def sample(sim, f):
            box = object_bbox(sim, "collider")  # the collider's geometry, transforms included
            return packed_positions(sim, "box"), (box[1][1] if box else None)

        track = step(dop, 96, sample)
        y = track[96][0][names[0]][1]
        col_y = {f: track[f][1] for f in (1, 13, 25, 96)}
        label = {"plain": "object transform animated", "deforming": "object transform animated + animategeo",
                 "sop": "Transform SOP animated + animategeo"}[tag]
        moved = col_y[25] is not None and col_y[25] > 0.5
        hint = "" if close(y, 1.5, 0.03) else (
            " Houdini moved the collider, so Box3D ignored the move" if moved else
            " Houdini itself did not move the collider object")
        # Without animategeo Houdini keeps the Static Object at its first transform: not ours to fix.
        status = Result.PASS if close(y, 1.5, 0.03) else (Result.FAIL if moved else Result.INFO)
        results.append(Result("animated_static", status,
                              f"{label}: box rests at y={y:.3f} (expected 1.5); the collider's top in the simulation on frames "
                              f"1, 13, 25, 96: {[None if v is None else round(v, 2) for v in col_y.values()]}.{hint}"))
    return results


# ---------------------------------------------------------------------------
# Objects solved by another solver (status: one-way interaction)
# ---------------------------------------------------------------------------


def test_other_solver():
    """A Bullet-solved box falls onto a Box3D-solved box resting on the ground: where each ends up,
    and the Box3D Solver's messages. INFO: the notes expect other solvers' objects to be static
    polygon colliders for Box3D (one-way), and packed ones to have no usable collision geometry."""
    path_b, names_b = packed_boxes("other_box3d", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0)}])
    path_a, names_a = packed_boxes("other_bullet", [{"size": (1, 1, 1), "center": (0, 4, 0)}])
    dop = new_dopnet("other_solver")
    b = packed_object(dop, "box3d_obj", path_b)
    a = packed_object(dop, "bullet_obj", path_a)
    b_solver = box3d_solver(dop, b)
    a_chain = bullet_solver(dop, a)
    finish(dop, merge(dop, [a_chain, b_solver], "both"))
    msgs = []

    def sample(sim, f):
        msgs.extend(m for m in node_messages(b_solver) if "Box3D" in m)
        return packed_positions(sim, "bullet_obj"), packed_positions(sim, "box3d_obj")

    track = step(dop, 72, sample)
    ya = track[72][0][names_a[0]][1]
    pb = track[72][1][names_b[1]]
    return [Result("other_solver", Result.INFO,
                   f"Bullet box at y={ya:.2f} (1.5 = on the Box3D box, 0.5 = on the ground, lower = fell through); "
                   f"Box3D box at ({pb[0]:.2f}, {pb[1]:.2f}) (started at (0, 0.5)); Box3D messages: "
                   f"{sorted(set(msgs))[:2] or 'none'}")]


# ---------------------------------------------------------------------------
# Attach Internal Constraints to Object (status: ConstraintGeometry on the object)
# ---------------------------------------------------------------------------


def test_internal_constraints():
    """A Constraint Network with Attach Internal Constraints to Object on: the weld still holds."""
    dop, names = hanging_pair(
        "internal", "weld", conrels=[("box3d_weldconrel", {})],
        net_parms={("attachinternal", "internalconstraints", "useinternal", "attachtoobject",
                    "attachinternalconstraints"): 1})
    y = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))[48][names[1]][1]
    return [Result("internal_constraints", Result.PASS if close(y, 2.0, 0.05) else Result.FAIL,
                   f"welded piece at y={y:.3f} (2.0 = held)")]


# ---------------------------------------------------------------------------
# SideFX Glue, Hard without constraint_type, Cone Twist: compared with Bullet
# ---------------------------------------------------------------------------


def test_sidefx_glue():
    """SideFX Glue holding a hanging piece, at several Strengths: whether it breaks in Box3D and in
    Bullet. INFO: Box3D breaks at strength / timestep (our rule); masses may differ too."""
    rows = []
    for strength in (1.0, 100.0, 1e4, 1e7):
        out = {}
        for kind in ("box3d", "bullet"):
            tag = f"glue_{kind}_{int(strength)}"
            dop, names = hanging_pair(tag, None, conrels=[("glueconrel", {"strength": strength})], solver_kind=kind)
            y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
            out[kind] = "held" if y > 1.9 else "broke"
        rows.append(f"strength {strength:g}: Box3D {out['box3d']}, Bullet {out['bullet']}")
    return [Result("sidefx_glue", Result.INFO, "; ".join(rows))]


def test_sidefx_hard_untyped():
    """SideFX Hard constraint with no constraint_type attribute (taken as 'all'), piece spinning at
    the start: distance and rotation after 2 s in Box3D and in Bullet. PASS if they agree."""
    d = sidefx_compare("hard_untyped", "hardconrel", {}, spin=(0.0, 3.0, 0.0))
    ok = close(d["box3d"][0], d["bullet"][0], 0.1) and abs(d["box3d"][1] - d["bullet"][1]) < 15
    return [Result("sidefx_hard_untyped", Result.PASS if ok else Result.FAIL,
                   f"Box3D: distance {d['box3d'][0]:.3f}, turned {d['box3d'][1]:.0f} deg; Bullet: distance "
                   f"{d['bullet'][0]:.3f}, turned {d['bullet'][1]:.0f} deg")]


def test_sidefx_cone_twist():
    """SideFX Cone Twist constraint, piece spinning at the start: rotation after 2 s in Box3D and in
    Bullet. INFO (which end is the goal only matters when the two anchors' frames differ)."""
    d = sidefx_compare("cone_twist", "conetwistconrel", {}, spin=(0.0, 3.0, 0.0))
    return [Result("sidefx_cone_twist", Result.INFO,
                   f"Box3D: distance {d['box3d'][0]:.3f}, turned {d['box3d'][1]:.0f} deg; Bullet: distance "
                   f"{d['bullet'][0]:.3f}, turned {d['bullet'][1]:.0f} deg")]


# ---------------------------------------------------------------------------
# Aging compared with the POP Solver (status: dead once age > life; POP may use >=)
# ---------------------------------------------------------------------------


def test_aging_vs_pop():
    """When a piece with life 0.5 dies in Box3D, and when a particle with life 0.5 dies in the POP
    Solver. INFO."""
    path, names = packed_boxes("age_box3d", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0),
                                                      "attrs": {"active": 1, "life": 0.5, "age": 0.0}}])
    dop = new_dopnet("age_box3d")
    solver = box3d_solver(dop, packed_object(dop, "pieces", path))
    set_parm(solver, ["enableaging"], 1)
    finish(dop, solver)
    track = step(dop, 24, lambda sim, f: point_attributes(sim, "pieces", names[1], ["name", "dead"]))
    ours = first_frame(track, lambda a: a is None or a.get("name") != names[1] or a.get("dead") == 1)

    pts = plain_points("age_pop_points", [{"P": (0, 0, 0)}])
    pdop = new_dopnet("age_pop")
    pobj = create(pdop, "popobject", "particles")
    psolver = create(pdop, "popsolver", "pop_solver")
    psolver.setInput(0, pobj)
    source = create(pdop, "popsource", "source")
    set_menu_by_label(source, ["emittype"], "All Points")
    try:
        set_menu_by_label(source, ["usecontextgeo"], "Use Parameter")
    except SetupError:
        pass
    set_parm(source, ["soppath"], pts)
    set_parm(source, ["life"], 0.5)
    set_parm(source, ["lifevar"], 0.0)
    set_parm(source, ["constantactivate", "constantactivation"], 0)
    set_expression(source, ["impulseactiveate", "impulseactivate", "impulseactivation"], "$SF == 1")
    psolver.setInput(1, source)
    finish(pdop, psolver)

    def count(sim, f):
        obj = sim.findObject("particles")
        geo = obj.geometry() if obj is not None else None
        return len(geo.points()) if geo is not None else 0

    ptrack = step(pdop, 24, count)
    born = first_frame(ptrack, lambda n: n > 0)
    died = first_frame({f: n for f, n in ptrack.items() if born and f > born}, lambda n: n == 0)
    return [Result("aging_vs_pop", Result.INFO,
                   f"life 0.5 at 24 fps: Box3D marks the piece dead (or deletes it) on frame {ours}; the POP Solver's "
                   f"particle appears on frame {born} and is gone on frame {died}")]


TESTS = {
    "messages_clear": test_messages_clear,
    "saved_cache": test_saved_cache,
    "point_order": test_point_order,
    "edited_source": test_edited_source,
    "sop_solver_edit": test_sop_solver_edit,
    "time_scale_kinematic": test_time_scale_kinematic,
    "attribute_toggles": test_attribute_toggles,
    "dop_import": test_dop_import,
    "dead_with_network": test_dead_with_network,
    "animated_static": test_animated_static,
    "other_solver": test_other_solver,
    "internal_constraints": test_internal_constraints,
    "sidefx_glue": test_sidefx_glue,
    "sidefx_hard_untyped": test_sidefx_hard_untyped,
    "sidefx_cone_twist": test_sidefx_cone_twist,
    "aging_vs_pop": test_aging_vs_pop,
}

