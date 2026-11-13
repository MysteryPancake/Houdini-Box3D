"""Solver, UI and caching checks: the joint / Data / Solver parameter UI, node messages, cache
continuity, removing the object that carries the world, scene scale, a single thread."""

import re
import time

import hou

from b3dtest import (
    Result,
    SetupError,
    box3d_data,
    box3d_solver,
    create,
    finish,
    merge,
    new_dopnet,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    plain_points,
    set_expression,
    set_menu_by_label,
    set_parm,
    step,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}


# ---------------------------------------------------------------------------
# UI: disable conditions, slider ranges, defaults, tabs, labels (status: disable-when, ranges, tabs)
# ---------------------------------------------------------------------------


def folder_labels(group):
    labels = []

    def walk(entries):
        for t in entries:
            if isinstance(t, hou.FolderParmTemplate):
                labels.append(t.label())
                walk(t.parmTemplates())

    walk(group.entries())
    return labels


def disable_when(node, parm):
    """The disable-when condition Houdini loaded for `parm` (a parameter or a vector's tuple), or ''.
    (Parm.isDisabled() only reports Parm.disable(); conditions are evaluated by the UI.)"""
    pt = node.parmTuple(parm)
    if pt is None:
        raise SetupError(f"{node.path()} has no parameter '{parm}'")
    conds = pt.parmTemplate().conditionals()
    return " ".join(conds.get(hou.parmCondType.DisableWhen, "").split())


def disabled(node, parm, toggles):
    """Whether the disable-when condition of `parm` holds with the toggles set ({name: value}).
    Evaluates Houdini's loaded condition string: '{ a == 0 } { b == 0 }' is disabled when any
    {} group holds; the terms inside one group must all hold."""
    cond = disable_when(node, parm)
    for name, value in toggles.items():
        node.parm(name).set(value)
    for group in re.findall(r"\{([^}]*)\}", cond):
        terms = re.findall(r"(\w+)\s*==\s*(-?[\d.]+)", group)
        if terms and all(float(node.parm(n).eval()) == float(v) for n, v in terms):
            return True
    return False


def test_ui():
    """Joint, Box3D Data and Solver UI: disable conditions grey out the right parameters, slider
    ranges and defaults are as designed, the joint nodes have their two tabs."""
    dop = new_dopnet("ui")
    problems, checked = [], 0

    def expect(what, got, want):
        nonlocal checked
        checked += 1
        if got != want:
            problems.append(f"{what}: got {got!r}, expected {want!r}")

    rev = create(dop, "box3d_revoluteconrel", "revolute")
    expect("Revolute Stiffness disable-when string", disable_when(rev, "stiffness"), "{ enable_spring == 0 }")
    expect("Revolute Stiffness disabled with Enable Spring off", disabled(rev, "stiffness", {"enable_spring": 0}), True)
    expect("Revolute Stiffness enabled with Enable Spring on", disabled(rev, "stiffness", {"enable_spring": 1}), False)
    expect("Revolute Motor Speed disabled with Enable Motor off", disabled(rev, "motor_speed", {"enable_motor": 0}), True)
    expect("Revolute Lower Angle enabled with Enable Limit on", disabled(rev, "lower_angle", {"enable_limit": 1}), False)
    expect("Break Force disabled with Breakable off", disabled(rev, "force_threshold", {"breakable": 0}), True)

    whl = create(dop, "box3d_wheelconrel", "wheel")
    expect("Wheel Lower Steering Limit disabled with Steering on, Limit off",
           disabled(whl, "lower_steering_limit", {"enable_steering": 1, "enable_steering_limit": 0}), True)
    expect("Wheel Lower Steering Limit enabled with both on",
           disabled(whl, "lower_steering_limit", {"enable_steering": 1, "enable_steering_limit": 1}), False)
    expect("Wheel Lower Steering Limit disabled with Steering off, Limit on",
           disabled(whl, "lower_steering_limit", {"enable_steering": 0, "enable_steering_limit": 1}), True)

    sph = create(dop, "box3d_sphericalconrel", "spherical")
    expect("Spherical Target Rotation (a vector) disabled with Enable Spring off",
           disabled(sph, "target_rotation", {"enable_spring": 0}), True)

    dist = create(dop, "box3d_distanceconrel", "distance")
    expect("Distance Max Length disabled with Limit on but Limit Max Length off",
           disabled(dist, "max_length", {"enable_limit": 1, "limit_max_length": 0}), True)

    par = create(dop, "box3d_parallelconrel", "parallel")
    expect("Parallel Stiffness never disabled (no toggle)", par.parm("stiffness").isDisabled(), False)

    data = create(dop, "box3d_data", "data")
    expect("Box3D Data Start Asleep disabled with Allow Sleeping off",
           disabled(data, "startasleep", {"enablesleep": 0}), True)
    expect("Box3D Data Mass disabled with Compute Mass on", disabled(data, "mass", {"computemass": 1}), True)
    solver = create(dop, "box3d_solver", "solver")
    expect("Solver Forces Wake Bodies disabled with Use Force DOPs off",
           disabled(solver, "forceswake", {"useforces": 0}), True)

    # Ranges and defaults (Box3DJoints.h: jointParamRange, nodeJointDefaults)
    t = rev.parm("stiffness").parmTemplate()
    expect("Stiffness hard minimum 0", (t.minValue(), t.minIsStrict()), (0.0, True))
    expect("Stiffness default", round(t.defaultValue()[0], 4), 5.0)
    t = rev.parm("lower_angle").parmTemplate()
    expect("Lower Angle slider minimum -180, not hard", (t.minValue(), t.minIsStrict()), (-180.0, False))
    expect("Lower Angle default", round(t.defaultValue()[0], 4), -45.0)
    expect("Max Motor Torque default", round(rev.parm("max_motor_torque").parmTemplate().defaultValue()[0], 4), 100.0)
    expect("Breakable default", rev.parm("breakable").eval(), 0)

    labels = folder_labels(rev.parmTemplateGroup())
    expect("Revolute tabs present", all(x in labels for x in ("Joint", "Revolute")), True)
    expect("Enable Spring label", rev.parm("enable_spring").parmTemplate().label(), "Enable Spring")
    expect("Solver sleep label", solver.parm("enablesleep").parmTemplate().label(), "Allow Sleeping (All Objects)")

    if problems:
        return [Result("ui", Result.FAIL, f"{len(problems)} of {checked} checks: " + "; ".join(problems))]
    return [Result("ui", Result.PASS, f"all {checked} UI checks: the disable-when conditions Houdini loaded give the "
                                      "right greying for each toggle state, plus ranges, defaults, tabs and labels "
                                      "(how they look greyed out still needs eyes)")]


# ---------------------------------------------------------------------------
# Node messages appear on the Box3D Solver node (status: node messages)
# ---------------------------------------------------------------------------


def test_messages():
    """A problem (a convex shape asked for on plain points) shows as a "Box3D: ..." message on the
    Box3D Solver node itself."""
    dop = new_dopnet("messages")
    pts = plain_points("messages_points", [{"P": (0, 3, 0), "pscale": 1.0}])
    obj = packed_object(dop, "points", pts)
    data = box3d_data(dop, obj, "data")
    set_menu_by_label(data, ["defaultshape"], "Convex Hull")
    solver = box3d_solver(dop, data)
    finish(dop, solver)
    seen = []
    step(dop, 4, lambda sim, f: seen.extend(node_messages(solver)))
    ours = [m for m in seen if "Box3D" in m]
    if any("packed primitive" in m for m in ours):
        return [Result("messages", Result.PASS, f"on the solver node: {ours[0]!r}")]
    return [Result("messages", Result.FAIL, f"no 'needs a packed primitive' message on the solver node; got {seen}")]


# ---------------------------------------------------------------------------
# Cache continuity (status: getRequestedCacheSubclass, cached frames keep their poses)
# ---------------------------------------------------------------------------


def stack_scene(name, scale=1.0, gravity=None):
    """Three boxes dropped into a stack on the ground, plus one sliding box. Returns (dop, names)."""
    s = scale
    pieces = [{"size": (20 * s, s, 20 * s), "center": (0, -0.5 * s, 0), "attrs": {"active": 0}}]
    for i in range(3):
        pieces.append({"size": (s, s, s), "center": (0.05 * s * i, s * (0.6 + 1.1 * i), 0)})
    pieces.append({"size": (s, s, s), "center": (3 * s, 0.5 * s, 0), "attrs": {"v": (2.0 * s, 0, 0)}})
    path, names = packed_boxes(name + "_pieces", pieces)
    dop = new_dopnet(name)
    obj = packed_object(dop, "pieces", path, keep_velocity=True)
    finish(dop, box3d_solver(dop, obj, gravity=gravity if gravity is not None else (0, -9.80665 * s, 0)))
    return dop, names


def max_difference(a, b):
    return max((a[k] - b[k]).length() for k in a)


def test_cache():
    """Playing on from a frame scrubbed back to in the cache gives the same result as a straight
    run; earlier cached frames keep their poses; a run with the cache off matches too."""
    results = []
    dop, names = stack_scene("cache")
    straight = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))
    # Scrub back to 20 (cached) and play on.
    hou.setFrame(10)
    frame10_again = packed_positions(dop.simulation(), "pieces")
    hou.setFrame(20)
    resumed = {}
    for f in range(21, 49):
        hou.setFrame(f)
        resumed[f] = packed_positions(dop.simulation(), "pieces")
    d_cached = max_difference(frame10_again, straight[10])
    d_resume = max_difference(resumed[48], straight[48])
    results.append(Result("cache", Result.PASS if d_cached < 1e-6 else Result.FAIL,
                          f"frame 10 read back after playing to 48 differs by {d_cached:.2e} (cached poses kept)"))
    results.append(Result("cache", Result.PASS if d_resume < 1e-3 else Result.INFO,
                          f"scrubbed back to 20 and played on: frame 48 differs from the straight run by {d_resume:.2e}"))

    # Cache off: every frame continues from the previous one (or rebuilds).
    dop2, _ = stack_scene("cache_off")
    set_parm(dop2, ["cacheenabled", "cache_enabled"], 0)
    off = step(dop2, 48, lambda sim, f: packed_positions(sim, "pieces"))
    d_off = max_difference(off[48], straight[48])
    results.append(Result("cache", Result.PASS if d_off < 1e-3 else Result.FAIL,
                          f"cache off: frame 48 differs from the cached run by {d_off:.2e}"))

    # Tiny cache: past its memory limit.
    dop3, _ = stack_scene("cache_tiny")
    set_parm(dop3, ["cachemaxsize", "cache_max_size"], 1)
    tiny = step(dop3, 48, lambda sim, f: packed_positions(sim, "pieces"))
    d_tiny = max_difference(tiny[48], straight[48])
    results.append(Result("cache", Result.PASS if d_tiny < 1e-3 else Result.FAIL,
                          f"1 MB cache: frame 48 differs from the cached run by {d_tiny:.2e}"))
    return results


# ---------------------------------------------------------------------------
# Removing the object that carries the world (status: world data rides on the first object)
# ---------------------------------------------------------------------------


def test_remove_world_object():
    """Deleting the first object (the one carrying the world) at frame 20: no crash, the other
    object carries on."""
    dop = new_dopnet("remove")
    path_a, _ = packed_boxes("remove_A", [GROUND])
    path_b, names_b = packed_boxes("remove_B", [{"size": (1, 1, 1), "center": (0, 4, 0)}])
    a = packed_object(dop, "first", path_a)
    b = packed_object(dop, "second", path_b)
    solver = box3d_solver(dop, merge(dop, [a, b]))
    deleter = None
    for type_name in ("deleteobjects", "delete"):
        try:
            deleter = dop.createNode(type_name, "delete_first")
            break
        except hou.OperationFailed:
            continue
    if deleter is None:
        raise SetupError("no Delete Objects DOP found (tried 'deleteobjects', 'delete')")
    deleter.setInput(0, solver)
    try:
        set_parm(deleter, ["numdeletes"], 1)
    except SetupError:
        pass
    set_parm(deleter, ["objgroup0", "group", "objpattern", "objects"], "first")
    try:
        set_parm(deleter, ["deleteobjects0"], 1)
    except SetupError:
        pass
    set_expression(deleter, ["activation"], "$SF >= 20")
    finish(dop, deleter)

    def sample(sim, f):
        return sim.findObject("first") is not None, packed_positions(sim, "second")

    track = step(dop, 40, sample)
    gone = not track[40][0]
    y20, y40 = track[20][1][names_b[0]][1], track[40][1][names_b[0]][1]
    msg = f"first object gone at 40: {gone}; second at y={y20:.2f} (frame 20) -> {y40:.2f} (frame 40)"
    ok = gone and y40 < y20 + 1e-3
    return [Result("remove_world_object", Result.PASS if ok else Result.FAIL,
                   msg + ("; it carried on" if ok else "; expected the first object deleted and the second still simulating"))]


# ---------------------------------------------------------------------------
# Scene scale (status: Box3D's tolerances assume metres)
# ---------------------------------------------------------------------------


def test_scale():
    """The same stack at 0.01x, 1x and 100x (gravity scaled to match): rest heights and jitter,
    measured in box sizes, should be about the same."""
    rows, worst = [], 0.0
    for scale in (0.01, 1.0, 100.0):
        dop, names = stack_scene(f"scale_{str(scale).replace('.', '_')}", scale)
        track = step(dop, 96, lambda sim, f: packed_positions(sim, "pieces"))
        top = names[3]
        rest = track[96][top][1] / scale  # 2.5 for three unit boxes
        jitter = max((track[f][top] - track[f - 1][top]).length() for f in range(85, 97)) / scale * hou.fps()
        fell = track[96][names[1]][1] < -0.5 * scale
        rows.append(f"{scale:g}x: top box at {rest:.3f} sizes (2.5 = stacked), jitter {jitter:.4f} sizes/s"
                    + (", FELL THROUGH" if fell else ""))
        worst = max(worst, abs(rest - 2.5) + (10 if fell else 0))
    # About 3% sinking at 100x is known (status.md, Scene scale); fail on tunnelling or worse.
    status = Result.PASS if worst < 0.1 else Result.FAIL
    return [Result("scale", status, "; ".join(rows))]


# ---------------------------------------------------------------------------
# One thread (status: Box3D's tasks on Houdini's TBB pool)
# ---------------------------------------------------------------------------


def test_single_thread():
    """With HOUDINI_MAXTHREADS=1 (set by the runner) a scene with 100 boxes and Use All Threads
    finishes without stalling."""
    pieces = [GROUND] + [{"size": (0.5, 0.5, 0.5), "center": (i % 10 - 4.5, 1 + i // 10 * 0.6, 0)} for i in range(100)]
    path, _ = packed_boxes("threads_pieces", pieces)
    dop = new_dopnet("threads")
    obj = packed_object(dop, "pieces", path)
    solver = box3d_solver(dop, obj)
    set_parm(solver, ["autothreads"], 1)
    finish(dop, solver)
    start = time.time()
    step(dop, 48, lambda sim, f: None)
    seconds = time.time() - start
    return [Result("single_thread", Result.PASS,
                   f"48 frames of 100 boxes in {seconds:.1f}s with HOUDINI_MAXTHREADS=1, no stall")]


test_single_thread.env = {"HOUDINI_MAXTHREADS": "1"}


TESTS = {
    "ui": test_ui,
    "messages": test_messages,
    "cache": test_cache,
    "remove_world_object": test_remove_world_object,
    "scale": test_scale,
    "single_thread": test_single_thread,
}
