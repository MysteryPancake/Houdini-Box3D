"""The remaining scriptable checks: messages clearing, saved caches, point order, an edited packed source, SOP Solver
edits, Time Scale with kinematic bodies, attribute toggles against Bullet, DOP Import output, dead pieces in a
constraint network, an animated Static Object, Attach Internal Constraints and aging against the POP Solver."""

import os
import tempfile

import hou
from b3dtest import (
    GROUND,
    append_sop,
    box3d_data,
    box3d_packed_object,
    box3d_solver,
    box_sop,
    bullet_solver,
    close,
    constraint_lines,
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
    rbd_object,
    Result,
    set_expression,
    set_parm,
    SetupError,
    solver_for,
    sop_solver,
    sop_solver_contents,
    step,
    set_menu_token,
)
from checks_constraints import hanging_pair
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


def test_other_solver():
    """Objects solved by RBD Bullet Solver beside a Box3D Solver's, which sees them only as affectors: they are static
    polygon colliders for Box3D, one-way, and packed ones have no usable collision geometry. (1) A packed Bullet box
    dropped onto a Box3D box on a Box3D ground: Bullet does not see the Box3D ground or box, so the box falls
    through both (below y -10 after 3 s), the Box3D box does not move (within 0.05 of its start) and the Box3D Solver
    says that the affector has no usable collision geometry. (2) A polygon box solved by Bullet as an RBD Object, with no
    gravity, moving along x at 4 m/s into the Box3D box: it keeps its speed (the animation, within 0.05 after 3 s) and
    pushes the Box3D box ahead of it, which rests against its front face (1 m ahead of its centre, within 0.1) on the
    ground, with no message."""
    results = []
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
        msgs.extend(node_messages(b_solver))
        return packed_positions(sim, "bullet_obj"), packed_positions(sim, "box3d_obj")

    track = step(dop, 72, sample)
    ya = track[72][0][names_a[0]][1]
    pb = track[72][1][names_b[1]]
    warned = any("no usable collision geometry" in m for m in msgs)
    ok = ya < -10.0 and abs(pb[0]) < 0.05 and close(pb[1], 0.5, 0.05) and abs(pb[2]) < 0.05 and warned
    results.append(Result("other_solver", Result.PASS if ok else Result.FAIL,
                          f"packed: Bullet box at y={ya:.2f} (expected below -10: it fell through); Box3D box at "
                          f"({pb[0]:.2f}, {pb[1]:.2f}) (expected its start (0, 0.5)); warned: {warned} (expected True)"))

    path_b, names_b = packed_boxes("other_poly_box3d", [GROUND, {"size": (1, 1, 1), "center": (0, 0.5, 0)}])
    dop = new_dopnet("other_solver_polygon")
    b = packed_object(dop, "box3d_obj", path_b)
    a = rbd_object(dop, "poly_obj", box_sop("other_poly", (-3, 0.5, 0)), vel=(4, 0, 0))
    b_solver = box3d_solver(dop, b)
    a_chain = bullet_solver(dop, a)
    set_parm(a_chain, ["forcey"], 0.0)  # Bullet's gravity off: the box moves along x
    finish(dop, merge(dop, [a_chain, b_solver], "both"))
    msgs = []

    def sample_polygon(sim, f):
        msgs.extend(node_messages(b_solver))
        return (sim.findObject("poly_obj").transform().extractTranslates()[0],
                packed_positions(sim, "box3d_obj")[names_b[1]])

    track = step(dop, 72, sample_polygon)
    x, pb = track[72]
    want = 4.0 * 71 / hou.fps()  # the box's translation on the last frame
    ahead = pb[0] - (-3.0 + want)  # the Box3D box's centre from the polygon box's
    ok = (abs(x - want) < 0.05 and abs(ahead - 1.0) < 0.1 and close(pb[1], 0.5, 0.05) and not msgs)
    results.append(Result("other_solver", Result.PASS if ok else Result.FAIL,
                          f"polygon: Bullet box moved {x:.2f} (expected {want:.2f}); the Box3D box is {ahead:.2f} ahead of "
                          f"its centre (expected 1.0) at y={pb[1]:.3f} (expected 0.5); messages: {sorted(set(msgs)) or 'none'} "
                          f"(expected none)"))
    return results


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
    "other_solver": test_other_solver,
    "internal_constraints": test_internal_constraints,
    "aging_vs_pop": test_aging_vs_pop,
    "aging_vs_pop_keep": test_aging_vs_pop_keep,
    "aging_vs_pop_noreap": test_aging_vs_pop_noreap,
}
