"""The remaining scriptable checks: messages clearing, saved caches, point order, an edited packed
source, SOP Solver edits, Time Scale with kinematic bodies, attribute toggles against Bullet, DOP
Import output, dead pieces in a constraint network, an animated Static Object, objects from another
solver, Attach Internal Constraints, and SideFX Glue / Hard / Cone Twist / aging compared with
Bullet and the POP Solver."""

import importlib.util
import itertools
import os
import sys
import tempfile

import hou

from b3dtest import (
    GROUND,
    append_sop,
    box3d_data,
    box3d_packed_object,
    box3d_solver,
    close,
    constraint_lines,
    create,
    display,
    find_network_geometry,
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
    Result,
    set_expression,
    set_parm,
    SetupError,
    solver_for,
    sop_solver,
    sop_solver_contents,
    step,
    set_menu_token,
    threshold,
)
from checks_constraints import broken_count, hanging_pair, sidefx_compare
from checks_solver import max_difference, stack_scene



def first_frame(track, test):
    """The first frame whose sample passes test(sample), or None."""
    return next((f for f in sorted(track) if test(track[f])), None)


# ---------------------------------------------------------------------------
# Node messages clear once the problem is gone
# ---------------------------------------------------------------------------


def test_messages_clear():
    """A constraint line naming no joint node until frame 10 (re-imported each step with Overwrite
    with SOP): the solver warns on early frames and the warning is gone once the line names the weld."""
    def animate_name(path):
        def setup(node):
            set_parm(node, ["class"], 1)  # primitives
            set_parm(node, ["snippet"], 's@constraint_name = @Frame < 10 ? "nothing" : "Weld";')
        return append_sop(path, "attribwrangle", setup)

    dop, names = hanging_pair("messages_clear", "Weld", conrels=[("box3d_weldconrel", {})],
                              network_edit=animate_name,
                              net_parms={("reloadfromsops", "overwrite", "overwritewithsop"): 1})
    solver = dop.node("box3d_solver")
    track = step(dop, 20, lambda sim, f: node_messages(solver))
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
    piece the same result: bodies keep their identity. With `id` the result is exactly the same."""
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
    for label, with_id, reverse in (("plain", False, False), ("reversed", False, True), ("plain+id", True, False),
                                    ("reversed+id", True, True)):
        tag = label.replace("+", "_")
        path, _ = packed_boxes("order_" + tag, pieces(with_id))
        if reverse:
            path = append_sop(path, "sort", lambda n: set_menu_token(n, ["ptsort"], "rev"))
        dop = new_dopnet("order_" + tag)
        finish(dop, box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True, bounce=0.0)))
        runs[label] = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))[48]
    diffs = {k: max_difference(runs[k], runs["plain"]) for k in ("reversed", "reversed+id")}
    with_id_diff = max_difference(runs["reversed+id"], runs["plain+id"])
    # Identity: every piece ends nearest its own unsorted result (the pieces end about 3 apart). Exact
    # equality is not expected without `id`: bodies are created in id order (the point number when there
    # is no `id`) and Box3D's results depend slightly on body order.
    def keeps_identity(run):
        for n, p in run.items():
            nearest = min(runs["plain"], key=lambda k: (runs["plain"][k] - p).length())
            if nearest != n:
                return False
        return True

    ok = all(keeps_identity(runs[k]) for k in ("reversed", "reversed+id"))
    # With `id` the bodies are created in id order, so the order of the points makes no difference at all.
    exact = with_id_diff < 1e-6

    def detail(run):
        return ", ".join(f"{n}: {tuple(round(c, 2) for c in runs['plain'][n])} -> {tuple(round(c, 2) for c in run[n])}"
                         for n in sorted(run) if (run[n] - runs["plain"][n]).length() >= 1e-3)

    results = [Result("point_order", Result.PASS if ok else Result.FAIL,
                      "every piece keeps its identity when the points are reversed, with and without id"
                      if ok else f"a piece swapped identity: {detail(runs['reversed'])}"),
               Result("point_order", Result.PASS if exact else Result.FAIL,
                      f"with id, reversing the points changes a piece by {with_id_diff:.2e} (expected none)")]
    results.append(Result("point_order", Result.INFO,
                          f"order sensitivity: largest per-piece difference from the unsorted run, reversed "
                          f"{diffs['reversed']:.2e}, reversed with id {diffs['reversed+id']:.2e} (against the same "
                          f"points with id unreversed: {with_id_diff:.2e})"
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
    set_menu_token(data, ["defaultshape"], "2")  # Convex Hull
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
        solver = box3d_solver(dop, None) if kind == "box3d" else create(dop, "bulletrbdsolver", "bullet_solver")
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
    set_menu_token(imp, ["importstyle"], "fetch")
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
    dop, names = hanging_pair("dead_network", "Weld", conrels=[("box3d_weldconrel", {})], pieces=pieces)
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
    """A Static Object grid that rises to y = 1 over the first second, by its object's transform
    (followed through OBJ Path, with and without Use Deforming Geometry) and by a Transform SOP:
    a box dropped from y = 8 lands on it and rests at 1.5. Also reports the collider's transform
    as Houdini gives it."""
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
        if sop_level:
            set_parm(static, ["usetransform"], 1)
        else:
            # A Static Object follows the animated transform of its OBJ Path object (Use Object
            # Transform only bakes the transform into the geometry once, per the SideFX docs).
            set_parm(static, ["usetransform"], 0)
            set_parm(static, ["objpath", "obj_path"], geo.path())
        if deforming:
            try:
                set_parm(static, ["animategeo", "usedeforminggeo", "usedeforming"], 1)
            except SetupError as e:
                results.append(Result("animated_static", Result.INFO, f"no Use Deforming Geometry toggle: {e}"))
                continue
        path, names = packed_boxes("animated_static_box_" + tag, [{"size": (1, 1, 1), "center": (0, 8, 0)}])
        finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "box", path), static])))

        def sample(sim, f):
            box = object_bbox(sim, "collider")
            col = sim.findObject("collider")
            # The DOP object's own transform (Position data): Use Object Transform may move this
            # and leave the geometry where it is.
            ty = col.transform().extractTranslates()[1] if col is not None else None
            return packed_positions(sim, "box"), (box[1][1] if box else None), ty

        track = step(dop, 96, sample)
        y = track[96][0][names[0]][1]
        frames = (1, 13, 25, 96)
        # The collider's top as Box3D sees it: geometry top plus the object's own translation.
        col_y = {f: (None if track[f][1] is None else track[f][1] + (track[f][2] or 0.0)) for f in frames}
        obj_ty = [None if track[f][2] is None else round(track[f][2], 2) for f in frames]
        label = {"plain": "object transform animated (OBJ Path)",
                 "deforming": "object transform animated (OBJ Path) + animategeo",
                 "sop": "Transform SOP animated + animategeo"}[tag]
        moved = col_y[25] is not None and col_y[25] > 0.5
        hint = "" if close(y, 1.5, 0.03) else (
            " Houdini moved the collider, so Box3D ignored the move" if moved else
            " Houdini itself did not move the collider object")
        # Houdini did not move the collider: a test setup problem, not Box3D's.
        status = Result.PASS if close(y, 1.5, 0.03) else (Result.FAIL if moved else Result.INFO)
        results.append(Result("animated_static", status,
                              f"{label}: box rests at y={y:.3f} (expected 1.5); the collider's top in the simulation on frames "
                              f"1, 13, 25, 96: {[None if v is None else round(v, 2) for v in col_y.values()]} (object "
                              f"transform ty {obj_ty}).{hint}"))
    return results


# ---------------------------------------------------------------------------
# Objects solved by another solver (status: one-way interaction)
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Attach Internal Constraints to Object (status: ConstraintGeometry on the object)
# ---------------------------------------------------------------------------


def test_internal_constraints():
    """A Constraint Network with Attach Internal Constraints to Object on: the weld still holds."""
    dop, names = hanging_pair(
        "internal", "Weld", conrels=[("box3d_weldconrel", {})],
        net_parms={("attachinternal", "internalconstraints", "useinternal", "attachtoobject",
                    "attachinternalconstraints"): 1})
    y = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))[48][names[1]][1]
    return [Result("internal_constraints", Result.PASS if close(y, 2.0, 0.05) else Result.FAIL,
                   f"welded piece at y={y:.3f} (2.0 = held)")]


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
    Strength at which it stops breaking agrees within a factor of 1.5 (kGlueStrengthScale was measured against
    welds)."""
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
    from checks_sop import play
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
    the same stiffness, below 1 is Box3D stiffer (it needs less Strength to hold as much). Passes (per mode) when all three are within a factor of 4."""
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
            ok = ok and 0.25 <= ratio <= 4.0
            rows.append(f"Bullet {s:g} leaves {left}, Box3D does at {equivalent:.0f} (Box3D / Bullet {ratio:.2f})")
        label = "glue_fracture_vs_bullet" + ("" if compound else "_welds")
        results.append(Result(label, Result.PASS if ok else Result.FAIL,
                              f"Glue as One Body {'on' if compound else 'off'}: " + "; ".join(rows) +
                              " (expected Box3D / Bullet within 0.25 to 4)"))
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
                   global_iterations=None, column=False, load=None, compound=None, awake=False):
    """n glued boxes side by side. With drop=None they fall from `height` onto the ground (the landing is the hit).
    Otherwise they rest on it and a box is dropped from `drop` above the first one. bounce: RBD Packed Object's.
    iterations: the Glue's Propagation Iterations. global_iterations: the network's propagate_iteration detail attribute.
    column: the boxes stand one on another instead (the first lowest). load: the solver's Glue Inertial Load.
    compound: the solver's Glue as One Body (None: its default). awake: the pieces carry an `awake` point attribute."""
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
    a Strength that never breaks it), and the Strength at which the bonds stop breaking. The impacts are
    what must agree (Box3D from 0.5 to 2 times Bullet's). The Strength per impact is for information: Bullet breaks
    at about its impact when something hits a resting glued pair, and at several times its impact when
    the glued pieces land themselves (more with speed and bounce); Box3D uses one factor
    (kGlueStrengthScale)."""
    rows, ok = [], True
    for label, scene, frames in GLUE_MATCH_SCENES:
        got = {}
        for kind in ("bullet", "box3d"):
            peak = glue_row_run(kind, 1e12, frames, **scene)[1]
            got[kind] = (peak, glue_row_threshold(kind, frames, **scene))
        (pb, tb), (px, tx) = got["bullet"], got["box3d"]
        ratio = px / pb if pb else float("inf")
        ok = ok and 0.5 <= ratio <= 2.0

        def per_peak(peak, threshold):
            return "none" if not threshold or not peak else f"{threshold:.0f} ({threshold / peak:.1f}x)"

        rows.append(f"{label}: impact Bullet {pb:.0f}, Box3D {px:.0f}; holds from Bullet {per_peak(pb, tb)}, "
                    f"Box3D {per_peak(px, tx)}")
    return [Result("glue_break_match", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected Box3D's impacts from 0.4 to 2 times Bullet's)")]


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


# ---------------------------------------------------------------------------
# Aging compared with the POP Solver (same age, gone on the same frame: both objects solved on their creation frame, no birth jitter)
# ---------------------------------------------------------------------------


def _aging_pop_compare(tag, reap, frame_end):
    """A piece with life 0.5 in Box3D against a particle with life 0.5 in the POP Solver, with reaping
    and Reap At Frame End set to reap / frame_end on both. Returns the two per-frame tracks."""
    # The ground is in the same geometry, so it has a life too: long, as a piece with a life of 0 dies.
    ground = dict(GROUND, attrs=dict(GROUND["attrs"], life=1.0e6, age=0.0))
    path, names = packed_boxes(f"age_box3d_{tag}", [ground, {"size": (1, 1, 1), "center": (0, 0.5, 0),
                                                             "attrs": {"active": 1, "life": 0.5, "age": 0.0}}])
    dop = new_dopnet(f"age_box3d_{tag}")
    pieces = packed_object(dop, "pieces", path)
    # Like the POP Object here, whose particle is born and aged on frame 1.
    set_parm(pieces, ["solvefirstframe", "solveoncreationframe", "solve_first_frame"], 1)
    solver = box3d_solver(dop, pieces)
    set_parm(solver, ["enableaging"], 1)
    set_parm(solver, ["reapdead"], int(reap))
    set_parm(solver, ["reapatframeend"], int(frame_end))
    finish(dop, solver)

    def piece(sim, f):
        # (age, dead) of the piece, or None once it is gone
        a = point_attributes(sim, "pieces", names[1], ["name", "dead", "age"])
        return (a["age"], a["dead"]) if a and a.get("name") == names[1] else None

    track = step(dop, 24, piece)

    pts = plain_points(f"age_pop_points_{tag}", [{"P": (0, 0, 0)}])
    pdop = new_dopnet(f"age_pop_{tag}")
    pobj = create(pdop, "popobject", "particles")
    psolver = create(pdop, "popsolver", "pop_solver")
    psolver.setInput(0, pobj)
    set_parm(psolver, ["doreapparticles"], int(reap))
    set_parm(psolver, ["reapatend"], int(frame_end))
    source = create(pdop, "popsource", "source")
    set_menu_token(source, ["emittype"], "allpoint")
    try:
        set_menu_token(source, ["usecontextgeo"], "none")  # Use Parameter Values
    except SetupError:
        pass
    set_parm(source, ["soppath"], pts)
    set_parm(source, ["life"], 0.5)
    set_parm(source, ["lifevar"], 0.0)
    # Jitter Birth Time: None, so the particle is born at the frame (no random sub-frame age).
    set_menu_token(source, ["jitterbirthtime"], "none")
    set_parm(source, ["constantactivate", "constantactivation"], 0)
    set_expression(source, ["impulseactiveate", "impulseactivate", "impulseactivation"], "$SF == 1")
    psolver.setInput(1, source)
    finish(pdop, psolver)

    def particle(sim, f):
        # (age, dead) of the particle, or None once it is gone
        obj = sim.findObject("particles")
        geo = obj.geometry() if obj is not None else None
        pts = geo.points() if geo is not None else ()
        if not pts:
            return None
        age = pts[0].attribValue("age") if geo.findPointAttrib("age") is not None else None
        dead = pts[0].attribValue("dead") if geo.findPointAttrib("dead") is not None else 0
        return age, dead

    return track, step(pdop, 24, particle)


def _aging_pop_result(name, reap, frame_end):
    track, ptrack = _aging_pop_compare(name, reap, frame_end)

    def fmt(v):
        if v is None:
            return "-"
        return f"{v[0]:.4f}" + (" dead" if v[1] else "")

    def gone(t):
        return first_frame({f: v for f, v in t.items() if f > 1}, lambda v: v is None)

    def dead_seen(t):
        return first_frame(t, lambda v: v is not None and v[1] == 1)

    # Every frame is compared; frames 1..15 and the last are printed (enough to see aging after death).
    rows, same = [], True
    for f in sorted(track):
        a, p = track.get(f), ptrack.get(f)
        if (a is None) != (p is None) or (a is not None and (not close(a[0], p[0], 1e-4) or bool(a[1]) != bool(p[1]))):
            same = False
        if f <= 15 or f == max(track):
            rows.append(f"{f}: {fmt(a)} / {fmt(p)}")
    # Reaping: both gone, on the same frame. Not reaping: neither ever gone.
    ok = same and gone(track) == gone(ptrack) and (gone(track) is not None) == reap
    mode = f"Reap At Frame End {'on' if frame_end else 'off'}" if reap else "reaping off"
    return [Result(name, Result.PASS if ok else Result.FAIL,
                   f"life 0.5 at 24 fps, {mode}: gone on frame "
                   f"{gone(track)} (Box3D) / {gone(ptrack)} (POP), dead seen on {dead_seen(track)} / "
                   f"{dead_seen(ptrack)}. Age per frame (Box3D / POP): " + ", ".join(rows))]


def test_aging_vs_pop():
    """Reaping with Reap At Frame End on, against POP (both solved on frame 1, no birth jitter): the same
    age every frame, and both gone on the frame they die, never seen dead."""
    return _aging_pop_result("aging_vs_pop", True, True)


def test_aging_vs_pop_keep():
    """Reaping with Reap At Frame End off, against POP: both kept one frame with dead = 1, then gone."""
    return _aging_pop_result("aging_vs_pop_keep", True, False)


def test_aging_vs_pop_noreap():
    """Reaping off on both, against POP: the dead piece is kept with dead = 1 on both, and its age
    after death (still growing or not) matches POP's on every frame."""
    return _aging_pop_result("aging_vs_pop_noreap", False, True)


# ---------------------------------------------------------------------------
# Probe: packed points sharing a `name` (Bullet solves them as one object)
# ---------------------------------------------------------------------------


def _pair_scene(tag, kind, pieces, rename, frames, keep_velocity=False):
    """Simulates `pieces` (packed_boxes specs) with Box3D or Bullet after renaming the pieces whose
    index is in `rename` to "pair". Returns {frame: [(name, P, v, w, mass) per point, in point
    order]} and the solver's messages."""
    path, names = packed_boxes(f"pair_{tag}_{kind}", pieces)
    targets = " || ".join(f's@name == "{names[i]}"' for i in rename)
    prev = hou.node(path)
    wr = create(prev.parent(), "attribwrangle", "rename")
    wr.setInput(0, prev)
    set_parm(wr, ["snippet"], f'if ({targets}) s@name = "pair";')
    display(wr)
    dop = new_dopnet(f"pair_{tag}_{kind}")
    obj = packed_object(dop, "pieces", wr.path(), keep_velocity=keep_velocity)
    solver = solver_for(kind, dop, obj)
    finish(dop, solver)
    msgs = []

    def attr(pt, geo, name, default):
        return pt.attribValue(name) if geo.findPointAttrib(name) is not None else default

    def sample(sim, f):
        if f == frames:
            msgs.extend(node_messages(solver))
        o = sim.findObject("pieces")
        geo = o.geometry() if o is not None else None
        if geo is None:
            return []
        return [(attr(pt, geo, "name", ""), pt.position(), attr(pt, geo, "v", (0, 0, 0)),
                 attr(pt, geo, "w", (0, 0, 0)), attr(pt, geo, "mass", None)) for pt in geo.points()]

    return step(dop, frames, sample), msgs


def _fmt(v):
    return "(" + ", ".join(f"{x:.2f}" for x in v) + ")"


def _pair_points(sample):
    return [p for p in sample if p[0] == "pair"]


def test_name_fusion_probe():
    """INFO probe for matching Bullet: two packed points renamed to the same `name`. Bullet is said
    to solve same-named packed primitives as one object; measured on 22.0.502 it does not (they land
    separately and keep their own mass and active), but both start with the first point's velocity.
    For Bullet and Box3D, reports: (1) a pair dropped one end first: whether the two points keep
    their 3.2 spacing (one rigid object) or
    land separately (2.0 apart); (2) different v on the two points (up 2 on the left one only):
    which velocity the pair ends up with, and whether it spins (both moving at the average with
    a spin = momentum combined); (3) different density: the mass written on each point; (4) one
    point active 0, the other 1: whether the pair stays put, falls or splits."""
    results = []
    box = {"size": (1, 1, 1)}
    for kind in ("bullet", "box3d"):
        label = "Bullet" if kind == "bullet" else "Box3D"
        notes = []

        # (1) one end first
        pieces = [GROUND, dict(box, center=(-1, 3, 0)), dict(box, center=(1, 5.5, 0))]
        track, msgs = _pair_scene("tilt", kind, pieces, [1, 2], 72)
        spans = []
        for f in (1, 24, 48, 72):
            pts = _pair_points(track[f])
            spans.append(f"{(pts[0][1] - pts[1][1]).length():.2f}" if len(pts) == 2 else f"{len(pts)} pt(s)")
        pts = _pair_points(track[72])
        notes.append(f"dropped one end first: spacing on frames 1, 24, 48, 72 = {spans} (3.2 kept = one "
                     f"object; 2.0 = landed separately); at rest {[_fmt(p[1]) for p in pts]}")

        # (2) different velocities, high in the air (no contact in 12 frames)
        pieces = [dict(box, center=(-1, 20, 0), attrs={"v": (0.0, 2.0, 0.0)}),
                  dict(box, center=(1, 20, 0), attrs={"v": (0.0, 0.0, 0.0)})]
        track, _ = _pair_scene("vel", kind, pieces, [0, 1], 12, keep_velocity=True)
        pts = _pair_points(track[12])
        f1 = _pair_points(track[1])
        notes.append("left point v (0, 2, 0), right (0, 0, 0): on frame 1 v "
                     f"{[_fmt(p[2]) for p in f1]}; on frame 12 v {[_fmt(p[2]) for p in pts]}, "
                     f"w {[_fmt(p[3]) for p in pts]}, spacing "
                     f"{(pts[0][1] - pts[1][1]).length():.2f} (start 2.00)" if len(pts) == 2 else
                     f"velocity case: {len(pts)} pair point(s) on frame 12")

        # (3) different density
        pieces = [dict(box, center=(-1, 20, 0), attrs={"density": 1000.0}),
                  dict(box, center=(1, 20, 0), attrs={"density": 4000.0})]
        track, _ = _pair_scene("mass", kind, pieces, [0, 1], 3)
        pts = _pair_points(track[3])
        notes.append(f"density 1000 / 4000: mass on the points {[p[4] for p in pts]} (1000 / 4000 = each "
                     f"its own; 5000 on both = combined)")

        # (4) one static, one active
        pieces = [dict(box, center=(-1, 3, 0), attrs={"active": 0}),
                  dict(box, center=(1, 3, 0), attrs={"active": 1})]
        track, _ = _pair_scene("active", kind, pieces, [0, 1], 24)
        pts = _pair_points(track[24])
        notes.append(f"left active 0, right active 1: y after 1 s {[round(p[1][1], 2) for p in pts]} (both 3 = "
                     f"static wins; both lower = active wins; 3 and lower = split)")

        results.append(Result("name_fusion_probe", Result.INFO,
                              f"{label}: " + "; ".join(notes) + (f"; messages {sorted(set(msgs))}" if msgs else "")))
    return results

def held_by_duplicate_name(order):
    """Two dynamic pieces named `dup` (ids 20 at x 1 and 10 at x -1, as the points `order` lists them) and a fixed
    piece welded to `dup`: the id of the piece the weld holds (it stays up while the other falls)."""
    box = {"size": (1, 1, 1)}
    spots = {20: dict(box, center=(1, 2, 0), attrs={"id": 20, "active": 1}),
             10: dict(box, center=(-1, 2, 0), attrs={"id": 10, "active": 1})}
    pieces = [dict(box, center=(0, 2, 0), attrs={"id": 30, "active": 0})] + [spots[i] for i in order]
    path, names = packed_boxes("dup_names_" + "_".join(map(str, order)), pieces)
    path = append_sop(path, "attribwrangle", lambda n: (set_parm(n, ["class"], 2),
                                                         set_parm(n, ["snippet"], 'if (@ptnum >= 1) s@name = "dup";')))
    dop = new_dopnet("dup_names_" + "_".join(map(str, order)))
    obj = box3d_packed_object(dop, "pieces", path)
    node = create(dop, "box3d_weldconrel", "joint")
    network = constraint_lines(dop.name() + "_network", [(names[0], (0.5, 2, 0), "dup", (0.5, 2, 0), "Weld")])
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network)
    net.setInput(1, node)
    finish(dop, box3d_solver(dop, net))

    def heights(sim, f):
        geo = sim.findObject("pieces").geometry()
        return {int(pt.attribValue("id")): pt.position()[1] for pt in geo.points()}

    last = step(dop, 24, heights)[24]
    return max((20, 10), key=lambda i: last[i]), last


def test_duplicate_name_point_order():
    """Joint anchors that name two pieces sharing a `name` take the first one in point order, whatever the ids are (the
    pieces are created in id order, so this must not follow it)."""
    results = []
    for order in ((20, 10), (10, 20)):
        held, heights = held_by_duplicate_name(order)
        results.append(Result("duplicate_name_point_order", Result.PASS if held == order[0] else Result.FAIL,
                              f"points listed as ids {order}: the weld holds id {held} (expected {order[0]}, the "
                              f"first point); heights {{id: y}} {({k: round(v, 2) for k, v in heights.items()})}"))
    return results


def duplicate_id_run(xs):
    """Ground (id 99), two boxes sharing id 7 at the x positions `xs` (in point order), then a box with id 1, so the ids
    are out of order and pass 2 sorts. Returns (x of the box that fell, node messages)."""
    box = {"size": (1, 1, 1)}
    pieces = [dict(GROUND, attrs={"id": 99, "active": 0}),
              dict(box, center=(xs[0], 3, 0), attrs={"id": 7, "active": 1}),
              dict(box, center=(xs[1], 3, 0), attrs={"id": 7, "active": 1}),
              dict(box, center=(10, 3, 0), attrs={"id": 1, "active": 1})]
    tag = "dup_ids_" + "_".join(str(int(x)) for x in xs)
    path, _ = packed_boxes(tag + "_pieces", pieces)
    dop = new_dopnet(tag)
    solver = box3d_solver(dop, packed_object(dop, "pieces", path, keep_velocity=True))
    finish(dop, solver)

    def heights(sim, f):
        geo = sim.findObject("pieces").geometry()
        return {round(pt.position()[0]): pt.position()[1] for pt in geo.points()}

    last = step(dop, 12, heights)[12]
    fell = [x for x in (round(xs[0]), round(xs[1])) if last[x] < 2.5]
    return fell, last, node_messages(solver)


def test_duplicate_id_first_point_wins():
    """Two points with one `id` are one body: the first in point order is simulated (it falls) and the other is left
    where it is, whichever order the ids are in (pass 2 visits them in id order, point order for equal ids). The
    solver warns about the duplicate."""
    results = []
    for xs in ((0.0, 3.0), (3.0, 0.0)):
        fell, last, msgs = duplicate_id_run(xs)
        dup_warned = any("share an `id`" in m for m in msgs)
        ok = fell == [round(xs[0])] and last[10] < 2.5 and dup_warned
        results.append(Result("duplicate_id_first_point_wins", Result.PASS if ok else Result.FAIL,
                              f"duplicates listed at x {xs}: the box that fell {fell} (expected [{round(xs[0])}]); heights "
                              f"{ {k: round(v, 2) for k, v in last.items()} }; duplicate warning "
                              f"{'yes' if dup_warned else 'no'}"))
    return results


TESTS = {
    "name_fusion_probe": test_name_fusion_probe,
    "messages_clear": test_messages_clear,
    "saved_cache": test_saved_cache,
    "point_order": test_point_order,
    "duplicate_name_point_order": test_duplicate_name_point_order,
    "duplicate_id_first_point_wins": test_duplicate_id_first_point_wins,
    "edited_source": test_edited_source,
    "sop_solver_edit": test_sop_solver_edit,
    "time_scale_kinematic": test_time_scale_kinematic,
    "attribute_toggles": test_attribute_toggles,
    "dop_import": test_dop_import,
    "dead_with_network": test_dead_with_network,
    "animated_static": test_animated_static,
    "internal_constraints": test_internal_constraints,
    "sidefx_glue": test_sidefx_glue,
    "glue_impact": test_glue_impact,
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
    "glue_compound_bridges": test_glue_compound_bridges,
    "glue_break_match": test_glue_break_match,
    "sidefx_hard_untyped": test_sidefx_hard_untyped,
    "aging_vs_pop": test_aging_vs_pop,
    "aging_vs_pop_keep": test_aging_vs_pop_keep,
    "aging_vs_pop_noreap": test_aging_vs_pop_noreap,
}

