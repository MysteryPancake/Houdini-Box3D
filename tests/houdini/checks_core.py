"""The first Houdini checks: node registration, two RBD Objects, a Constraint Network weld,
new objects, pscale, Static Object winding, RBD Object defaults, and the probes that found the
joint-node crash. Run through run_tests.py."""

import hou

from b3dtest import (
    Result,
    SetupError,
    box3d_solver,
    bullet_solver,
    centroid,
    close,
    constraint_lines,
    create,
    finish,
    merge,
    new_dopnet,
    node_messages,
    node_type_exists,
    packed_boxes,
    packed_object,
    packed_positions,
    parm_value,
    plain_points,
    point_attributes,
    set_parm,
    single_object_geo,
    step,
    velocity,
    whole_object_points,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}  # top face at y = 0


def ground_object(dop, name="ground"):
    path, _ = packed_boxes(name + "_geo", [GROUND])
    return packed_object(dop, name, path)


# ---------------------------------------------------------------------------
# Preflight: the plugin is loaded and its node types exist (status.md: the nine DOPs appear)
# ---------------------------------------------------------------------------


def test_nodes():
    """The plugin's DOP node types are registered, and each joint node's Data Name defaults to its type."""
    cat = hou.dopNodeTypeCategory()
    joints = ["weld", "spherical", "revolute", "prismatic", "distance", "motor", "parallel", "wheel", "filter"]
    missing = [n for n in ["box3d_solver", "box3d_data"] + [f"box3d_{j}conrel" for j in joints]
               if not node_type_exists(cat, n)]
    if missing:
        return [Result("nodes", Result.FAIL,
                       f"node types not found: {', '.join(missing)}. Is the plugin in the dso folder?")]
    results = [Result("nodes", Result.PASS, "Box3D Solver, Box3D Data and the nine joint nodes are registered")]
    dop = new_dopnet()
    wrong = []
    for j in joints:
        node = create(dop, f"box3d_{j}conrel")
        label = hou.nodeType(cat, f"box3d_{j}conrel").description()
        dataname = parm_value(node, ["dataname", "data_name", "DataName"])
        if dataname is None:
            names = ", ".join(sorted(pt.name() for pt in node.parmTuples() if "name" in pt.name().lower()))
            raise SetupError(f"{node.path()}: no Data Name parameter found under the names tried; "
                             f"parameters with 'name' in them: {names}")
        if dataname != j or not label.startswith("Box3D "):
            wrong.append(f"{j}: label '{label}', Data Name '{dataname}'")
    results.append(Result("nodes", Result.FAIL if wrong else Result.PASS,
                          "; ".join(wrong) if wrong else "joint nodes have 'Box3D ...' labels and Data Name = type"))
    return results


# ---------------------------------------------------------------------------
# Item 8: two RBD Objects in one solver affect each other both ways
# ---------------------------------------------------------------------------


def rbd_object(dop, name, sop_path, v=None):
    obj = create(dop, "rbdobject", name)
    set_parm(obj, ["soppath"], sop_path)
    if v is not None:
        set_parm(obj, ["vel", "v", "velocity"], tuple(v))
    return obj


def box_sop(name, center, size=(1, 1, 1)):
    def build(geo):
        box = create(geo, "box")
        set_parm(box, ["size"], tuple(size))
        xf = create(geo, "xform")
        xf.setInput(0, box)
        set_parm(xf, ["t"], tuple(center))
        return xf

    return single_object_geo(name, build)


def collide_whole_objects(name, b_data=None, frames=48):
    """A (moving at 2 m/s along x) hits B (at rest), zero gravity. Returns the tracks and the
    velocities after the collision. b_data: Box3D Data parameters to put on B, or None."""
    dop = new_dopnet(name)
    a = rbd_object(dop, "A", box_sop(name + "_A", (-3, 0, 0)), v=(2, 0, 0))
    b = rbd_object(dop, "B", box_sop(name + "_B", (0, 0, 0)))
    b_out = b
    if b_data is not None:
        data = create(dop, "box3d_data", "B_data")
        data.setInput(0, b)
        for parm, value in b_data.items():
            set_parm(data, [parm], value)
        b_out = data
    solver = box3d_solver(dop, merge(dop, [a, b_out]), gravity=(0, 0, 0))
    finish(dop, solver)

    def sample(sim, f):
        pa, pb = whole_object_points(sim, "A"), whole_object_points(sim, "B")
        return (centroid(pa) if pa else None, centroid(pb) if pb else None)

    track = step(dop, frames, sample)
    if any(track[frames][i] is None for i in (0, 1)):
        raise SetupError("objects A / B not found in the simulation (check the RBD Object nodes' object names)")
    ta = {f: track[f][0] for f in track}
    tb = {f: track[f][1] for f in track}
    return dop, solver, ta, tb, velocity(ta, frames, 0), velocity(tb, frames, 0)


def test_two_rbd():
    """Item 8: two RBD Objects in one Box3D Solver collide and both react (they share one world)."""
    dop, solver, ta, tb, va, vb = collide_whole_objects("two_rbd")
    moved_b = vb > 0.2
    slowed_a = va < 1.8
    msg = f"after the hit: A vx={va:.2f}, B vx={vb:.2f} (A started at 2.0, B at rest)"
    if moved_b and slowed_a:
        return [Result("two_rbd", Result.PASS, msg + "; both objects reacted")]
    hint = "B never moved: A and B are probably in separate worlds (separate solve calls)" if not moved_b else \
        "B moved but A did not slow down: the interaction is one-way"
    return [Result("two_rbd", Result.FAIL, f"{msg}. {hint}. Messages: {node_messages(solver)}")]


# ---------------------------------------------------------------------------
# Item 34: a Constraint Network finds a Box3D joint node and the joint holds
# ---------------------------------------------------------------------------


def welded_pair(name, conrel_type="box3d_weldconrel", solver_kind="box3d"):
    """A piece hanging off a fixed one, held only by a constraint network line naming
    `conrel_type`'s Data Name (None: no joint node wired, the control). Returns the hanging
    piece's position after 48 frames and the solver's messages."""
    dop = new_dopnet(name)
    pieces = [
        {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}},  # fixed
        {"size": (1, 1, 1), "center": (1, 2, 0)},  # hangs off it, held only by the weld
    ]
    path, names = packed_boxes(name + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path)
    face = (0.5, 2, 0)  # both anchors on the shared face: the weld frame
    conrel = create(dop, conrel_type, "joint") if conrel_type else None
    data_name = parm_value(conrel, ["dataname", "data_name", "DataName"], "weld") if conrel else "weld"
    network_path = constraint_lines(name + "_network", [(names[0], face, names[1], face, data_name)])
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network_path)
    if conrel is not None:
        net.setInput(1, conrel)
    solver = box3d_solver(dop, net) if solver_kind == "box3d" else bullet_solver(dop, net)
    finish(dop, solver)
    messages = []

    def sample(sim, f):
        messages.extend(node_messages(solver))
        return packed_positions(sim, "pieces")

    track = step(dop, 48, sample)
    return track[48][names[1]], messages


def test_network():
    """Item 34: a weld from a Constraint Network + Box3D Weld node holds a hanging piece."""
    held, messages = welded_pair("network")
    loose, _ = welded_pair("network_control", conrel_type=None)
    results = []
    if loose[1] > 1.5:
        results.append(Result("network", Result.ERROR,
                              f"control run without the weld node: the piece did not fall (y={loose[1]:.2f}); "
                              "the test can't tell a held joint from a stuck piece"))
    name_warnings = [m for m in messages if "names no" in m or "does not exist" in m]
    if close(held[1], 2.0, 0.05) and not name_warnings:
        results.append(Result("network", Result.PASS,
                              f"welded piece stayed at y={held[1]:.3f} (control fell to y={loose[1]:.2f})"))
    else:
        results.append(Result("network", Result.FAIL,
                              f"welded piece at y={held[1]:.3f} (expected 2.0); control y={loose[1]:.2f}. "
                              f"Warnings: {name_warnings or messages}. If they say the line names no "
                              "constraint relationship, first check the Box3D Weld node is in the input of the "
                              "Constraint Network that takes constraint data (this script uses its 2nd input)"))
    return results


# ---------------------------------------------------------------------------
# Probes for a crash in `network`: each isolates one suspect. Run on their own if `network`
# crashes; a CRASH in one of them names the culprit.
# ---------------------------------------------------------------------------


def cook_data_node_alone(type_name):
    dop = new_dopnet("probe")
    node = create(dop, type_name, "data")
    finish(dop, node)
    step(dop, 3, lambda sim, f: None)


def test_probe_weld_alone():
    """Probe: a Box3D Weld node cooked on its own in a DOP network (no Constraint Network)."""
    cook_data_node_alone("box3d_weldconrel")
    return [Result("probe_weld_alone", Result.PASS, "the Box3D Weld node cooks on its own")]


def test_probe_data_alone():
    """Probe (control): a Box3D Data node, which also has folders, cooked on its own."""
    cook_data_node_alone("box3d_data")
    return [Result("probe_data_alone", Result.PASS, "the Box3D Data node cooks on its own")]


def test_probe_sequence():
    """Probe: the first run's crash order in one process: joint nodes made and the scene cleared,
    then the network scene."""
    test_nodes()
    rbd_object(new_dopnet("probe_partial"), "A", box_sop("probe_partial_A", (0, 0, 0)))  # as two_rbd left it
    hou.hipFile.clear(suppress_save_prompt=True)
    held, _ = welded_pair("probe_sequence")
    return [Result("probe_sequence", Result.PASS, f"no crash (piece at y={held[1]:.2f})")]


def test_probe_glue_box3d():
    """Probe: a SideFX Glue node in a Constraint Network with the Box3D Solver."""
    y, messages = welded_pair("probe_glue", conrel_type="glueconrel")
    return [Result("probe_glue_box3d", Result.PASS if close(y[1], 2.0, 0.1) else Result.INFO,
                   f"no crash; glued piece at y={y[1]:.3f} (2.0 = held). Messages: {messages[:3]}")]


def test_probe_weld_bullet():
    """Probe: a Box3D Weld node in a Constraint Network with the Bullet solver (which ignores it)."""
    y, messages = welded_pair("probe_weld_bullet", solver_kind="bullet")
    return [Result("probe_weld_bullet", Result.PASS,
                   f"no crash: our node cooks inside the Constraint Network (piece at y={y[1]:.2f}; "
                   "Bullet doesn't know the joint, so it may fall)")]


# ---------------------------------------------------------------------------
# Item 4: new objects first move one step after they appear, with their initial v
# ---------------------------------------------------------------------------


def test_new_objects():
    """Item 4: an object shows its initial state on its first frame and moves (with its initial v)
    on the next, both at frame 1 and when created at frame 20."""
    dop = new_dopnet("new_objects")
    v0 = 2.0
    start = {"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"v": (v0, 0, 0)}}
    path_a, names_a = packed_boxes("new_objects_A", [GROUND, start])
    a = packed_object(dop, "early", path_a)
    # RBD Packed Object replaces point v with its own initial velocity unless told to keep it.
    keep_v = ["inheritvelocity", "usepointvelocity", "inheritvel"]
    set_parm(a, keep_v, 1)
    late = {"size": (1, 1, 1), "center": (5, 3, 0), "attrs": {"v": (v0, 0, 0)}}
    path_b, names_b = packed_boxes("new_objects_B", [late])
    b = packed_object(dop, "late", path_b)
    set_parm(b, keep_v, 1)
    set_parm(b, ["createframe", "creationframe"], 20)
    finish(dop, box3d_solver(dop, merge(dop, [a, b])))

    seen_v = {}

    def sample(sim, f):
        if f == 1:
            seen_v["early"] = point_attributes(sim, "early", names_a[1], ["v"])
        return packed_positions(sim, "early"), packed_positions(sim, "late")

    track = step(dop, 22, sample)
    dx_expected = v0 / hou.fps()
    results = []

    def check(label, first, second, x0, name):
        p1, p2 = first[name], second[name]
        problems = []
        if not close(p1[1], 3.0, 1e-3) or not close(p1[0], x0, 1e-3):
            problems.append(f"on its first frame it is at ({p1[0]:.3f}, {p1[1]:.3f}), not its start ({x0}, 3)")
        if not p2[1] < p1[1] - 1e-4:
            problems.append("it did not fall on its second frame")
        dx = p2[0] - p1[0]
        if not close(dx, dx_expected, 0.5 * dx_expected):
            got_v = seen_v.get("early") if label.endswith("frame 1") else None
            problems.append(f"it moved {dx:.4f} in x on its first step, expected about {dx_expected:.4f} "
                            "from its initial v"
                            + (f" (the point's v as the solver got it on frame 1: {got_v}; (0,0,0) means "
                               "RBD Packed Object still replaced the v we set, despite Inherit Velocity)"
                               if got_v is not None else ""))
        if problems:
            results.append(Result("new_objects", Result.FAIL, f"{label}: " + "; ".join(problems)))
        else:
            results.append(Result("new_objects", Result.PASS,
                                  f"{label}: starts in place, then moves {dx:.4f} in x (initial v applied)"))

    check("object made at frame 1", track[1][0], track[2][0], 0.0, names_a[1])
    first = next((f for f in sorted(track) if track[f][1] is not None), None)
    if first is not None and first < 20:
        parms = {n: parm_value(b, [n]) for n in ("createframe", "usesimframe") if b.parm(n) is not None}
        results.append(Result("new_objects", Result.FAIL,
                              f"the object meant to appear at frame 20 already exists from frame {first} "
                              f"(its parameters: {parms}); check how RBD Packed Object reads its creation frame"))
    elif track[20][1] is None or track[21][1] is None:
        results.append(Result("new_objects", Result.FAIL, "the frame-20 object never appeared"))
    else:
        check("object made at frame 20", track[20][1], track[21][1], 5.0, names_b[0])
    return results


# ---------------------------------------------------------------------------
# Item 12: sphere size from pscale on plain points, compared with Bullet
# ---------------------------------------------------------------------------


def rest_height_of_point(solver_kind):
    name = f"pscale_{solver_kind}"
    dop = new_dopnet(name)
    pts = plain_points(name + "_points", [{"P": (0, 3, 0), "pscale": 1.0}])
    balls = packed_object(dop, "balls", pts)
    objects = merge(dop, [ground_object(dop, name + "_ground"), balls])
    last = box3d_solver(dop, objects) if solver_kind == "box3d" else bullet_solver(dop, objects)
    finish(dop, last)
    size_attrs = {}

    def sample(sim, f):
        if f == 1:
            size_attrs.update(point_attributes(
                sim, "balls", names=["pscale", "scale", "bullet_primS", "bullet_radius", "bullet_georep",
                                     "bullet_autofit"]) or {})
        return packed_positions(sim, "balls")

    track = step(dop, 96, sample)
    positions = list(track[96].values())
    return (positions[0][1] if positions else float("nan")), size_attrs


def test_pscale():
    """Item 12: a plain point with pscale 1 rests at the same height in Box3D and Bullet."""
    ours, attrs = rest_height_of_point("box3d")
    theirs, _ = rest_height_of_point("bullet")
    present = {k: v for k, v in attrs.items() if v is not None}
    msg = f"rest height of a pscale-1 point on the ground: Box3D {ours:.3f}, Bullet {theirs:.3f}"
    results = [Result("pscale", Result.PASS if close(ours, 0.5, 0.05) else Result.FAIL,
                      f"Box3D: a pscale-1 point rests at y={ours:.3f} (0.5 expected: the shape fills a box of "
                      f"size pscale). Size attributes on the point as the solver got it: {present}. "
                      "A bullet_primS there (added by RBD Packed Object) overrides pscale")]
    if close(theirs, 3.0, 0.01):
        return results + [Result("pscale", Result.INFO, "Bullet did not move the point: it doesn't simulate "
                                                        "loose points, so there is nothing to compare with")]
    if close(ours, theirs, 0.05):
        return results + [Result("pscale", Result.PASS, msg)]
    ratio = theirs / ours if ours > 1e-6 else float("nan")
    return results + [Result("pscale", Result.FAIL, f"{msg} (Bullet's shape is about {ratio:.1f}x ours)")]


# ---------------------------------------------------------------------------
# Item 32 (+31): which way a Static Object mesh is solid, and the Ground Plane
# ---------------------------------------------------------------------------


def drop_on_static(label, collider):
    """collider(dop) returns the collider node. Returns the dropped box's final height and messages."""
    name = f"winding_{label}"
    dop = new_dopnet(name)
    path, names = packed_boxes(name + "_box", [{"size": (1, 1, 1), "center": (0, 2, 0)}])
    box = packed_object(dop, "box", path)
    solver = box3d_solver(dop, merge(dop, [box, collider(dop)]))
    finish(dop, solver)
    messages = []

    def sample(sim, f):
        messages.extend(node_messages(solver))
        return packed_positions(sim, "box")

    track = step(dop, 72, sample)
    return track[72][names[0]][1], messages


def static_grid(name, reverse):
    def collider(dop):
        def build(geo):
            grid = create(geo, "grid")
            set_parm(grid, ["size"], (10, 10))
            if not reverse:
                return grid
            rev = create(geo, "reverse")
            rev.setInput(0, grid)
            return rev

        path = single_object_geo(name, build)
        static = create(dop, "staticobject", "collider")
        set_parm(static, ["soppath"], path)
        return static

    return collider


def test_winding():
    """Item 32: a box dropped on a default (upward-facing) grid Static Object lands on it, without
    Flip Static Mesh Winding. Also reports the reversed grid and a Ground Plane DOP (item 31)."""
    up, _ = drop_on_static("default", static_grid("winding_grid", reverse=False))
    down, _ = drop_on_static("reversed", static_grid("winding_grid_reversed", reverse=True))
    plane, plane_msgs = drop_on_static("groundplane", lambda dop: create(dop, "groundplane", "collider"))

    def state(y):
        return "landed" if y > 0.3 else "fell through"

    results = []
    msg = f"default grid: {state(up)} (y={up:.2f}); reversed grid: {state(down)} (y={down:.2f})"
    if up > 0.3:
        results.append(Result("winding", Result.PASS, msg))
    elif down > 0.3:
        results.append(Result("winding", Result.FAIL, msg + ". Our winding is the opposite of Houdini's: "
                                                            "Flip Static Mesh Winding should be the default"))
    else:
        results.append(Result("winding", Result.FAIL, msg + ". Neither side collides: Static Objects are not "
                                                            "reaching the solver at all"))
    warned = [m for m in plane_msgs if "usable collision" in m]
    results.append(Result("winding", Result.INFO,
                          f"Ground Plane DOP: box {state(plane)} (y={plane:.2f})"
                          + (f"; warning: {warned[0]}" if warned else "")))
    return results


# ---------------------------------------------------------------------------
# Item 44: a default RBD Object (no Box3D Data) has mass = density x volume, bounce 0.5, hull shape
# ---------------------------------------------------------------------------


def test_rbd_defaults():
    """Item 44: a default RBD Object weighs density x volume, bounces at 0.5 and collides as a
    convex hull (not a box)."""
    results = []
    # Mass and bounce from a head-on hit: A is a default RBD Object (1 m cube), B the same cube
    # with Box3D Data giving it a known mass of density x volume and bounce 1 (A's 0.5 wins: the
    # bounce mixing default is Minimum). Momentum: mA*2 = mA*vA + mB*vB.
    probe = new_dopnet("probe")
    density = parm_value(create(probe, "rbdobject", "probe"), ["density"], 1000.0)
    probe.destroy()
    expected_mass = density * 1.0
    _, _, _, _, va, vb = collide_whole_objects(
        "rbd_defaults_hit", b_data={"computemass": 0, "mass": expected_mass, "bounce": 1.0})
    mass_ratio = vb / (2.0 - va) if abs(2.0 - va) > 1e-6 else float("nan")  # mA / mB
    bounce = (vb - va) / 2.0
    msg = f"after the hit A vx={va:.2f}, B vx={vb:.2f}: mass(A)/mass(B) = {mass_ratio:.2f}, bounce = {bounce:.2f}"
    ok_mass = close(mass_ratio, 1.0, 0.1)
    ok_bounce = close(bounce, 0.5, 0.1)
    results.append(Result("rbd_defaults", Result.PASS if ok_mass else Result.FAIL,
                          msg + f" (expected mass ratio 1: A should weigh density {density:g} x 1 m^3)"))
    results.append(Result("rbd_defaults", Result.PASS if ok_bounce else Result.FAIL,
                          f"bounce {bounce:.2f} (expected 0.5 from the RBD Object's Bounce)"))

    # Shape: a 4 x 0.2 x 0.2 rod tilted 45 degrees in its SOP. A convex hull lets it fall flat;
    # an axis-aligned box fitted around it would stand about 3 units tall.
    dop = new_dopnet("rbd_defaults_shape")

    def rod(geo):
        box = create(geo, "box")
        set_parm(box, ["size"], (4, 0.2, 0.2))
        xf = create(geo, "xform")
        xf.setInput(0, box)
        set_parm(xf, ["r"], (0, 0, 45))
        set_parm(xf, ["t"], (0, 3, 0))
        return xf

    obj = rbd_object(dop, "rod", single_object_geo("rbd_defaults_rod", rod))
    finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop), obj])))
    frames = 240
    track = step(dop, frames, lambda sim, f: whole_object_points(sim, "rod"))
    pts = track[frames]
    top = max(p[1] for p in pts)
    bottom = min(p[1] for p in pts)
    moving = max((a - b).length() for a, b in zip(track[frames], track[frames - 1]))
    shape_msg = (f"tilted rod (0.2 thick) after {frames} frames spans y {bottom:.2f} to {top:.2f}, "
                 f"still moving {moving * hou.fps():.3f}/s")
    if top < 1.5 and bottom > -0.1:
        results.append(Result("rbd_defaults", Result.PASS,
                              shape_msg + ": it fell over, so it is not an axis-aligned box fitted around the "
                              "tilted rod (that would stand about 3 tall)"))
        if top > 0.35:
            results.append(Result("rbd_defaults", Result.INFO,
                                  "but it isn't lying flat (flat would top out at 0.2, or 0.28 on an edge): open "
                                  "out/rbd_defaults.hip to see what it rests on"))
    else:
        results.append(Result("rbd_defaults", Result.FAIL,
                              shape_msg + ": it stands, as an axis-aligned box fitted around the tilted rod would"))
    return results


TESTS = {
    "nodes": test_nodes,
    "two_rbd": test_two_rbd,
    "network": test_network,
    "new_objects": test_new_objects,
    "pscale": test_pscale,
    "winding": test_winding,
    "rbd_defaults": test_rbd_defaults,
    "probe_weld_alone": test_probe_weld_alone,
    "probe_data_alone": test_probe_data_alone,
    "probe_sequence": test_probe_sequence,
    "probe_glue_box3d": test_probe_glue_box3d,
    "probe_weld_bullet": test_probe_weld_bullet,
}
