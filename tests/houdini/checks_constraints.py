"""Constraint and whole-object checks: breaking and the broken group, attributes multiplying the
node's values, next_constraint_name, feedback attributes, centroid anchors, Hard / Spring compared
with Bullet; RBD Object first-frame drift, initial velocity and spin, Use Object Transform, other
RBD object types, an RBD Object's Create Active Object and inertia type, a Box3D Data change
mid-simulation, a Static Object's object transform."""

import math

import hou

from b3dtest import (
    box_sop,
    four_points_at_height,
    GROUND,
    box3d_data,
    box3d_solver,
    centroid,
    close,
    constraint_lines,
    create,
    display,
    find_network_geometry,
    finish,
    ground_object,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    node_messages,
    packed_boxes,
    packed_object,
    packed_positions,
    parm_value,
    rbd_object,
    Result,
    set_expression,
    set_parm,
    SetupError,
    single_object_geo,
    solver_for,
    sop_solver,
    step,
    whole_object_points,
    set_menu_token,
)

FACE = (0.5, 2, 0)  # the shared face of the fixed piece (0, 2, 0) and the hanging one (1, 2, 0)


def hanging_pair(name, joints, line_attrs=None, conrels=None, gravity=None, solver_kind="box3d",
                 pieces=None, anchors=None, data_parms=None, keep_velocity=False, network_edit=None,
                 net_parms=None, packed_parms=None):
    """A piece hanging off a fixed one through a constraint network line.
    joints: constraint_name of the line (None: the first joint node's Data Name). conrels:
    [(node type, {parm: value})] wired into the network (merged if several). Returns (dop, piece
    names). data_parms: Box3D Data parameters for the pieces (Box3D solver only). network_edit:
    called with the network's SOP path, returns the path to use (to add SOPs after it).
    net_parms: Constraint Network parameters ({candidate names tuple: value})."""
    dop = new_dopnet(name)
    pieces = pieces or [
        {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 0}},
        {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1}},
    ]
    path, names = packed_boxes(name + "_pieces", pieces)
    obj = packed_object(dop, "pieces", path, keep_velocity=keep_velocity, **(packed_parms or {}))
    if data_parms and solver_kind == "box3d":
        obj = box3d_data(dop, obj, "data", **data_parms)
    nodes = []
    for i, (type_name, parms) in enumerate(conrels or []):
        node = create(dop, type_name, f"joint{i}")
        for parm, value in parms.items():
            set_parm(node, [parm], value)
        nodes.append(node)
    if joints is None:
        joints = parm_value(nodes[0], ["dataname", "data_name", "DataName"])
    pa, pb = anchors or (FACE, FACE)
    line = (names[0], pa, names[1], pb, joints) + ((line_attrs,) if line_attrs else ())
    network_path = constraint_lines(name + "_network", [line])
    if network_edit:
        network_path = network_edit(network_path)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network_path)
    for names_tuple, value in (net_parms or {}).items():
        set_parm(net, list(names_tuple), value)
    if len(nodes) == 1:
        net.setInput(1, nodes[0])
    elif nodes:
        net.setInput(1, merge(dop, nodes, "joints"))
    last = solver_for(solver_kind, dop, net, gravity=gravity)
    finish(dop, last)
    return dop, names


# ---------------------------------------------------------------------------
# Breaking, the broken group, and no leak into earlier cached frames
# ---------------------------------------------------------------------------


def broken_count(sim):
    geo = find_network_geometry(sim)
    if geo is None:
        return None
    grp = geo.findPrimGroup("broken")
    return len(grp.prims()) if grp is not None else 0


def test_breaking():
    """A Breakable weld with a tiny Break Force breaks at once: the piece falls, its line joins the
    `broken` group, and scrubbing back to frame 1 shows it unbroken (no leak into cached frames)."""
    dop, names = hanging_pair("breaking", "weld",
                              conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e-3})])
    track = step(dop, 24, lambda sim, f: (packed_positions(sim, "pieces"), broken_count(sim)))
    if track[24][1] is None:
        raise SetupError("could not find the constraint network geometry in the simulation")
    hou.setFrame(1)
    back = broken_count(dop.simulation())
    y = track[24][0][names[1]][1]
    ok = y < 1.5 and track[24][1] == 1 and back == 0
    return [Result("breaking", Result.PASS if ok else Result.FAIL,
                   f"piece at y={y:.2f} on frame 24 (fell = below 1.5); broken lines: {track[24][1]} on frame 24, "
                   f"{back} after scrubbing back to frame 1 (expected 1 and 0)")]


def test_attribute_multiplies():
    """A float primitive attribute multiplies the node's value: Break Force 1e9 on the node times
    force_threshold 1e-12 on the line breaks the joint, while the node's 1e9 alone holds it (Break Torque is out of
    reach in both, as its default breaks the hanging weld by itself)."""
    ys = {}
    for kind, attrs in (("alone", None), ("times", {"force_threshold": 1e-12})):
        dop, names = hanging_pair("multiply_" + kind, "weld", line_attrs=attrs,
                                  conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e9,
                                                                 "torque_threshold": 1e12})])
        ys[kind] = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
    ok = ys["alone"] > 1.5 and ys["times"] < 1.5
    return [Result("attribute_multiplies", Result.PASS if ok else Result.FAIL,
                   f"piece at y={ys['alone']:.2f} with the node's 1e9 alone (expected held) and y={ys['times']:.2f} "
                   f"with 1e-12 on the line (expected fell: 1e9 x 1e-12 used as the break force)")]


def test_unset_attributes():
    """A line attribute at "not set" (Box3D Constraint Properties writes it on lines it does not edit: a float at
    -3.4028235e38) keeps the node's value instead of multiplying it. A Breakable weld with Break Force 1e-3 (Break
    Torque out of reach) still breaks with force_threshold unset, and the other way round with torque_threshold. Controls:
    with both out of reach and nothing on the line the weld holds, and a real factor 1e-12 on a Break Force of 1e9 breaks it."""
    unset = -3.4028235e38
    far = 1e12
    rows, ok = [], True
    for label, node, attrs, falls in (
            ("both out of reach", {"force_threshold": far, "torque_threshold": far}, None, False),
            ("force_threshold 1e-12 on 1e9", {"force_threshold": 1e9, "torque_threshold": far},
             {"force_threshold": 1e-12}, True),
            ("force_threshold unset on 1e-3", {"force_threshold": 1e-3, "torque_threshold": far},
             {"force_threshold": unset}, True),
            ("torque_threshold unset on 1e-3", {"force_threshold": far, "torque_threshold": 1e-3},
             {"torque_threshold": unset}, True)):
        dop, names = hanging_pair("unset_" + label.replace(" ", "_"), "weld", line_attrs=attrs,
                                  conrels=[("box3d_weldconrel", dict(node, breakable=1))])
        y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
        good = (y < 1.5) == falls
        ok = ok and good
        rows.append(f"{label}: y={y:.2f} ({'fell' if y < 1.5 else 'held'}, expected {'fell' if falls else 'held'})")
    return [Result("unset_attributes", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def test_next_constraint():
    """A breaking weld with next_constraint_name = spherical becomes a ball joint at the same
    place: the piece swings but stays 0.5 from the anchor, and the line's constraint_name changes."""
    dop, names = hanging_pair(
        "next_constraint", "weld", line_attrs={"next_constraint_name": "spherical"},
        conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e-3}), ("box3d_sphericalconrel", {})])

    def sample(sim, f):
        geo = find_network_geometry(sim)
        prim = geo.prims()[0] if geo is not None and geo.prims() else None
        attrs = {}
        if prim is not None:
            for a in ("constraint_name", "next_constraint_name"):
                if geo.findPrimAttrib(a) is not None:
                    attrs[a] = prim.attribValue(a)
        return packed_positions(sim, "pieces"), attrs

    track = step(dop, 36, sample)
    pos = track[36][0][names[1]]
    dist = (pos - hou.Vector3(FACE)).length()
    attrs = track[36][1]
    ok = pos[1] < 1.9 and close(dist, 0.5, 0.05) and attrs.get("constraint_name") == "spherical" \
        and not attrs.get("next_constraint_name")
    return [Result("next_constraint", Result.PASS if ok else Result.FAIL,
                   f"piece at y={pos[1]:.2f} (swung = below 1.9), {dist:.3f} from the anchor (expected 0.5); "
                   f"line attributes on frame 36: {attrs}")]


def test_next_settings():
    """A breaking weld that switches to a spherical joint also takes its next settings: next_enable_cone_limit 1 and
    next_cone_angle 0.25 become enable_cone_limit and cone_angle on the line and are cleared (not set again), so the
    ball joint is made with them."""
    dop, _ = hanging_pair(
        "next_settings", "weld",
        line_attrs={"next_constraint_name": "spherical", "next_enable_cone_limit": 1, "next_cone_angle": 0.25},
        conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e-3}), ("box3d_sphericalconrel", {})])

    def sample(sim, f):
        geo = find_network_geometry(sim)
        prim = geo.prims()[0] if geo is not None and geo.prims() else None
        attrs = {}
        if prim is not None:
            for a in ("constraint_name", "enable_cone_limit", "cone_angle", "next_enable_cone_limit",
                      "next_cone_angle"):
                if geo.findPrimAttrib(a) is not None:
                    attrs[a] = prim.attribValue(a)
        return attrs

    attrs = step(dop, 36, sample)[36]
    ok = (attrs.get("constraint_name") == "spherical" and attrs.get("enable_cone_limit") == 1
          and close(attrs.get("cone_angle", -1.0), 0.25, 1e-4) and attrs.get("next_enable_cone_limit", 0) < 0
          and attrs.get("next_cone_angle", 0.0) <= -3.0e38)
    return [Result("next_settings", Result.PASS if ok else Result.FAIL,
                   f"line attributes on frame 36: {attrs} (expected spherical, enable_cone_limit 1, cone_angle 0.25 and "
                   f"the next ones cleared)")]


def test_feedback():
    """The solver writes force, torque, distance and angle onto a holding weld's line."""
    dop, _ = hanging_pair("feedback", "weld", conrels=[("box3d_weldconrel", {})],
                              packed_parms={"computemass": 0, "mass": 1.0})

    def sample(sim, f):
        geo = find_network_geometry(sim)
        if geo is None or not geo.prims():
            return None
        prim = geo.prims()[0]
        return {a: (prim.attribValue(a) if geo.findPrimAttrib(a) is not None else None)
                for a in ("force", "torque", "distance", "angle")}

    vals = step(dop, 24, sample)[24]
    if vals is None:
        raise SetupError("could not find the constraint network geometry in the simulation")
    missing = [a for a, v in vals.items() if v is None]
    if missing:
        return [Result("feedback", Result.FAIL, f"line attributes on frame 24: {vals}; missing {missing}")]
    # A 1 kg piece hanging off the weld: the joint carries its weight (m g) and holds (distance ~0).
    force, dist = abs(vals["force"]), abs(vals["distance"])
    ok = close(force, 9.80665, 0.25 * 9.80665) and dist < 0.01
    return [Result("feedback", Result.PASS if ok else Result.FAIL,
                   f"line attributes on frame 24: {vals} (expected force about 9.8 N for 1 kg, distance near 0)")]


def test_feedback_many():
    """Feedback and live anchors on a network of 400 Hard lines (many joints are written in parallel): 400 boxes
    hanging each from a fixed one carry the same weight, so every line's `force` is within 2% of the others', its
    `distance` is near 0 and every point has a current_orient (not the unwritten 0 0 0 0)."""
    from bench_scenes import SCENARIOS

    scenario = SCENARIOS["joints"]
    scene = scenario.build("box3d", dict(scenario.defaults, type="weld", layout="pairs", n=800, frames=30))
    prims = []

    def sample(sim, f):
        nonlocal prims
        geo = find_network_geometry(sim)
        if geo is None or f != 30:
            return None
        prims = [(pr.attribValue("force"), pr.attribValue("distance")) for pr in geo.prims()]
        unwritten = sum(1 for pt in geo.points() if pt.attribValue("current_orient") == (0.0, 0.0, 0.0, 0.0))
        return unwritten

    unwritten = step(scene.dop, 30, sample)[30]
    if not prims:
        raise SetupError("could not find the constraint network geometry in the simulation")
    forces = [f for f, _ in prims]
    mean = sum(forces) / len(forces)
    ok = (len(prims) == 400 and mean > 100.0 and max(forces) - min(forces) < 0.02 * mean
          and max(abs(d) for _, d in prims) < 0.01 and unwritten == 0)
    return [Result("feedback_many", Result.PASS if ok else Result.FAIL,
                   f"{len(prims)} lines (expected 400): force {min(forces):.1f} to {max(forces):.1f} N (mean {mean:.1f}, "
                   f"expected all within 2%), largest distance {max(abs(d) for _, d in prims):.4f} (expected under 0.01), "
                   f"{unwritten} points without current_orient (expected 0)")]


def test_anchor_id():
    """anchor_id on a whole object: a Weld between a world anchor at (0, 4, 0) and point 0 (or vertex 0) of an RBD
    Object box, whose anchor's own P is far away (50, 50, 50), pulls that point of the box to the world anchor, so
    the box ends at (0, 4, 0) minus the point's offset from the box's centre. Without anchor_id the weld would go
    to P instead."""
    results = []
    box_path = box_sop("anchor_id_box", (0, 3, 0))
    box_geo = hou.node(box_path).geometry()
    for label, kind_attrs, vertex in (("point 0", None, False), ("vertex 0", "vertex", True)):
        dop = new_dopnet(f"anchor_id_{label.split()[0]}")
        obj = rbd_object(dop, "box", box_path)
        node = create(dop, "box3d_weldconrel", "joint0")
        network_path = constraint_lines(f"anchor_id_{label.split()[0]}_network",
                                        [("", (0, 4, 0), "box", (50, 50, 50), parm_value(node, ["dataname", "data_name"]))])
        net_geo = hou.node(network_path).parent()
        wr = create(net_geo, "attribwrangle", "anchor_ids")
        wr.setInput(0, hou.node(network_path))
        set_parm(wr, ["class"], 0)
        code = 'if (!haspointattrib(0, "anchor_id")) addpointattrib(0, "anchor_id", -1); setpointattrib(0, "anchor_id", 1, 0);'
        if kind_attrs:
            code += ' addpointattrib(0, "anchor_type", ""); setpointattrib(0, "anchor_type", 1, "vertex");'
        set_parm(wr, ["snippet"], code)
        display(wr)
        net = create(dop, "constraintnetwork", "constraint_network")
        net.setInput(0, obj)
        set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], wr.path())
        net.setInput(1, node)
        finish(dop, box3d_solver(dop, net, gravity=(0, 0, 0)))
        point = box_geo.iterPoints()[0].position() if not vertex else \
            box_geo.prim(0).vertex(0).point().position()
        want = hou.Vector3(0, 4, 0) - (point - hou.Vector3(0, 3, 0))
        track = step(dop, 48, lambda sim, f: centroid(whole_object_points(sim, "box")))
        got = track[48]
        results.append(Result("anchor_id", Result.PASS if (got - want).length() < 0.1 else Result.FAIL,
                              f"Weld to {label}: box centre ends at {tuple(round(c, 2) for c in got)} "
                              f"(expected {tuple(round(c, 2) for c in want)}; its anchor's P was (50, 50, 50))"))
    return results


def sidefx_compare(kind, conrel_type, line_attrs, spin=None, anchors=None, conrel_parms=None):
    """A piece 2 below a fixed one, held by a SideFX constraint with anchors at the centroids,
    in Box3D and in Bullet. Returns {solver: (centroid distance, rotation angle in degrees)}
    after 2 s. spin: initial w (rad/s) of the hanging piece. anchors: (fixed, hanging) anchor
    positions (default: the centroids). conrel_parms: the relationship node's parameters."""
    from b3dtest import packed_transform
    out = {}
    for solver_kind in ("box3d", "bullet"):
        hang = {"active": 1, "w": tuple(spin)} if spin else {"active": 1}
        fixed = {"active": 0, "w": (0.0, 0.0, 0.0)} if spin else {"active": 0}
        pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": fixed},
                  {"size": (1, 1, 1), "center": (0.5, 2, 0), "attrs": hang}]
        dop, names = hanging_pair(f"{kind}_{solver_kind}{'_spin' if spin else ''}", None, line_attrs=line_attrs,
                                  conrels=[(conrel_type, conrel_parms or {})], solver_kind=solver_kind,
                                  pieces=pieces,
                                  anchors=anchors or ((0, 4, 0), (0.5, 2, 0)), keep_velocity=bool(spin))

        def sample(sim, f):
            return packed_positions(sim, "pieces"), packed_transform(sim, "pieces", names[1])

        track = step(dop, 48, sample)
        r = track[48][1]
        trace = r[0][0] + r[1][1] + r[2][2]
        angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1) / 2))))
        out[solver_kind] = ((track[48][0][names[1]] - track[48][0][names[0]]).length(), angle)
    return out


def test_sidefx_hard():
    """SideFX Hard constraint (constraint_type position, no restlength) compared with Bullet: the
    distance it holds between a fixed piece and one hanging about 2.06 below it."""
    d = sidefx_compare("hard", "hardconrel", {"constraint_type": "position"})
    ok = close(d["box3d"][0], d["bullet"][0], 0.1)
    return [Result("sidefx_hard", Result.PASS if ok else Result.FAIL,
                   f"held distance after 2 s: Box3D {d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} (start 2.06)")]


def test_sidefx_hard_all():
    """SideFX Hard constraint with constraint_type 'all', compared with Bullet: does Bullet's 'all'
    also use the rest length (1 by default), or hold the starting offset like Box3D's (2.06)?"""
    d = sidefx_compare("hard_all", "hardconrel", {"constraint_type": "all"})
    ok = close(d["box3d"][0], d["bullet"][0], 0.1)
    return [Result("sidefx_hard_all", Result.PASS if ok else Result.FAIL,
                   f"held distance after 2 s: Box3D {d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} (start 2.06; "
                   "1.0 = the default rest length)")]


def spring_mass_run(change):
    """A 1 kg piece hanging 1 below a fixed one by a SideFX Spring (Hooke's constant 200, damping 60, rest
    length 0, no collision between the two), anchors at the centres, in Box3D. With `change` its mass
    becomes 4 kg on frame 13 (a SOP Solver writes `mass`). Returns its height after 5 s."""
    dop = new_dopnet(f"spring_mass_{int(change)}")
    pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
              {"size": (1, 1, 1), "center": (0, 3, 0), "attrs": {"active": 1}}]
    path, names = packed_boxes(f"spring_mass_{int(change)}_pieces", pieces)
    obj = packed_object(dop, "pieces", path, computemass=0, mass=1.0)
    spring = create(dop, "springconrel", "spring")
    for parm, value in (("strength", 200.0), ("damping", 60.0), ("restlength", 0.0), ("disablecollisions", 1)):
        set_parm(spring, [parm], value)
    line = (names[0], (0, 4, 0), names[1], (0, 3, 0), parm_value(spring, ["dataname", "data_name", "DataName"]),
            {"constraint_type": "position"})
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], constraint_lines(f"spring_mass_{int(change)}_net",
                                                                                     [line]))
    net.setInput(1, spring)
    if change:
        mover = sop_solver(dop, f'if (@Frame > 12 && s@name == "{names[1]}") f@mass = 4.0;')
        last = multisolver(dop, net, [mover, box3d_solver(dop, None)])
    else:
        last = box3d_solver(dop, net)
    finish(dop, last)
    return step(dop, 120, lambda sim, f: packed_positions(sim, "pieces")[names[1]][1] if f == 120 else None)[120]


def test_sidefx_spring_mass():
    """A SideFX Spring's stiffness is Hooke's constant, so it must not change when a piece's mass does (Box3D's
    own springs are set by frequency: the wrapper recomputes it from the masses). A 1 kg piece hangs from
    a spring of constant 200 and sits mg/k = 0.049 below the spring's end; when its mass becomes 4 kg it
    settles 4 x as far (0.196). With a frequency left at the 1 kg value it would not move."""
    g, k = 9.81, 200.0
    still = spring_mass_run(False)
    heavier = spring_mass_run(True)
    want_still, want_heavier = 4.0 - 1.0 * g / k, 4.0 - 4.0 * g / k
    ok = close(still, want_still, 0.03) and close(heavier, want_heavier, 0.03)
    return [Result("sidefx_spring_mass", Result.PASS if ok else Result.FAIL,
                   f"hanging piece at y {still:.3f} with 1 kg (expected {want_still:.3f}), {heavier:.3f} after its "
                   f"mass becomes 4 kg (expected {want_heavier:.3f}; {still:.3f} means the spring kept its 1 kg "
                   "frequency)")]


# ---------------------------------------------------------------------------
# Whole objects (RBD Object)
# ---------------------------------------------------------------------------


def test_rbd_first_frames():
    """An RBD Object whose geometry sits away from the origin, with no gravity or velocity, does not
    drift or jump on its first frames (position write-back convention)."""
    dop = new_dopnet("rbd_drift")
    obj = rbd_object(dop, "box", box_sop("rbd_drift_box", (3, 2, -1)))
    finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))
    track = step(dop, 12, lambda sim, f: centroid(whole_object_points(sim, "box")))
    drift = max((track[f] - hou.Vector3(3, 2, -1)).length() for f in track)
    return [Result("rbd_first_frames", Result.PASS if drift < 1e-4 else Result.FAIL,
                   f"largest distance from its start over 12 frames: {drift:.2e}")]


def test_rbd_initial_motion():
    """An RBD Object's initial velocity (1, 0, 0) and angular velocity (0, 90, 0) apply: after 1 s it
    moved 1 in x and turned 90 degrees about y (if the angular velocity is in degrees per second)."""
    dop = new_dopnet("rbd_motion")
    obj = rbd_object(dop, "box", box_sop("rbd_motion_box", (0, 0, 0), size=(2, 0.5, 0.5)),
                     vel=(1, 0, 0), angvel=(0, 90, 0))
    finish(dop, box3d_solver(dop, obj, gravity=(0, 0, 0)))

    def sample(sim, f):
        o = sim.findObject("box")
        return centroid(whole_object_points(sim, "box")), o.transform()

    track = step(dop, 25, sample)
    c, m = track[25]
    angle = math.degrees(math.atan2(-m.at(0, 2), m.at(0, 0)))
    ok_v = close(c[0], 1.0, 0.03)
    results = [Result("rbd_initial_motion", Result.PASS if ok_v else Result.FAIL,
                      f"moved {c[0]:.3f} in x after 1 s (expected 1.0)")]
    results.append(Result("rbd_initial_motion", Result.PASS if close(angle, 90.0, 3.0) else Result.FAIL,
                          f"turned {angle:.1f} degrees about y (expected 90 for 90 deg/s)"))
    return results


def test_rbd_object_transform():
    """Use Object Transform on an RBD Object whose geo object is scaled 2x: the unit box behaves
    as a 2-unit box (rests 2 tall)."""
    geo = new_geo("scaled_box")
    box = create(geo, "box")
    display(box)
    set_parm(geo, ["s"], (2, 2, 2))
    set_parm(geo, ["t"], (0, 3, 0))
    dop = new_dopnet("rbd_scaled")
    obj = rbd_object(dop, "box", box.path())
    set_parm(obj, ["usetransform"], 1)
    ground_path, _ = packed_boxes("rbd_scaled_ground", [GROUND])
    finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "ground", ground_path), obj])))
    track = step(dop, 144, lambda sim, f: whole_object_points(sim, "box"))
    pts = track[144]
    lo, hi = min(p[1] for p in pts), max(p[1] for p in pts)
    speed = max((a - b).length() for a, b in zip(track[144], track[143])) * hou.fps()
    ok = close(lo, 0.0, 0.03) and close(hi, 2.0, 0.05)
    return [Result("rbd_object_transform", Result.PASS if ok else Result.FAIL,
                   f"after 6 s spans y {lo:.3f} to {hi:.3f} (expected 0 to 2; 2.06 would be a 2-unit box tilted "
                   f"about 2 deg), fastest point moving {speed:.3f}/s")]


def test_other_rbd_objects():
    """RBD Point Object and RBD Fractured Object simulate with the Box3D Solver (their objects
    fall)."""
    results = []
    for type_name, label in (("rbdpointobject", "RBD Point Object"), ("rbdfracturedobject", "RBD Fractured Object")):
        dop = new_dopnet(type_name)
        if type_name == "rbdpointobject":
            build = four_points_at_height
        else:
            def build(geo):
                a = create(geo, "box")
                b = create(geo, "box")
                xb = create(geo, "xform")
                xb.setInput(0, b)
                set_parm(xb, ["t"], (2, 0, 0))
                m = create(geo, "merge")
                m.setInput(0, a)
                m.setInput(1, xb)
                xf = create(geo, "xform")
                xf.setInput(0, m)
                set_parm(xf, ["t"], (0, 3, 0))
                wr = create(geo, "attribwrangle")
                wr.setInput(0, xf)
                set_parm(wr, ["class"], 1)
                set_parm(wr, ["snippet"], 's@name = "piece" + itoa(@primnum / 6);')
                return wr
        try:
            obj = create(dop, type_name, "objects")
        except SetupError as e:
            results.append(Result("other_rbd_objects", Result.INFO, f"{label}: not available ({e})"))
            continue
        set_parm(obj, ["soppath"], single_object_geo(type_name + "_geo", build))
        if type_name == "rbdpointobject":
            # Each point instances this geometry (else its objects have none to collide with).
            # Geometry Path is ignored while the menu beside it (usegeo) is 0, Use Point Value.
            set_parm(obj, ["usegeo"], 1)
            set_parm(obj, ["geopath", "instancepath", "geometrypath", "instance"],
                     single_object_geo("point_instance_box", lambda geo: create(geo, "box")))
        finish(dop, box3d_solver(dop, obj))

        def heights(sim, f):
            out = []
            for o in sim.objects():
                xform = o.transform()
                geo = o.geometry()
                pts = geo.points() if geo is not None else []
                if pts:
                    out.append(min((p.position() * xform)[1] for p in pts))
            return out

        track = step(dop, 24, heights)
        n = len(track[24])
        if type_name == "rbdpointobject" and n == 0:
            sim = dop.simulation()
            objs = sim.objects()
            kinds = sorted({k for o in objs for k in o.subData().keys()})
            # Every non-empty string parameter, to see which one RBD Point Object reads its points from.
            strings = {}
            for p in obj.parms():
                try:
                    if p.parmTemplate().type() == hou.parmTemplateType.String and p.evalAsString():
                        strings[p.name()] = p.evalAsString()
                except hou.Error:
                    pass
            # Errors and warnings on the node and anything inside it (it is a digital asset).
            problems = []
            for n in [obj] + list(obj.allSubChildren()):
                try:
                    for m in list(n.errors()) + list(n.warnings()):
                        problems.append(f"{n.name()}: {m.strip()[:150]}")
                except hou.Error:
                    pass
            results.append(Result("other_rbd_objects", Result.INFO,
                                  f"{label}: {len(objs)} object(s) in the simulation, none with geometry; their data: "
                                  f"{kinds}. Its string parameters: {strings}. Node errors / warnings: "
                                  f"{problems[:4] or 'none'}"))
            continue
        fell = n > 0 and len(track[1]) == n and all(b < a - 0.5 for a, b in zip(track[1], track[24]))
        results.append(Result("other_rbd_objects", Result.PASS if fell else Result.FAIL,
                              f"{label}: {n} object(s) with geometry; lowest points {[round(y, 2) for y in track[1]]} "
                              f"on frame 1, {[round(y, 2) for y in track[24]]} on frame 24"))
    return results


def test_rbd_object_active():
    """An RBD Object with Create Active Object off (SolverParms ActiveValue) is a static body: it
    stays where it is, while the same object with it on falls. No Box3D Data on either."""
    results = []
    heights = {}
    for label, active in (("active", 1), ("inactive", 0)):
        dop = new_dopnet(f"rbd_{label}")
        obj = rbd_object(dop, "box", box_sop(f"rbd_{label}_box", (0, 3, 0)))
        set_parm(obj, ["active"], active)  # Create Active Object
        finish(dop, box3d_solver(dop, obj))
        track = step(dop, 24, lambda sim, f: centroid(whole_object_points(sim, "box"))[1])
        heights[label] = track[24]
    ok = close(heights["inactive"], 3.0, 0.01) and heights["active"] < 2.5
    results.append(Result("rbd_object_active", Result.PASS if ok else Result.FAIL,
                          f"after 1 s the box is at y {heights['active']:.3f} with Create Active Object on "
                          f"(falls) and {heights['inactive']:.3f} off (expected 3.0)"))
    return results


def test_rbd_object_kinematic_pushes():
    """An RBD Object with a Kinematic Box3D Data and its Position animated (a Position DOP at Set Always, 2 m/s
    along x) drives as a kinematic body: it carries a dynamic box in front of it at the same speed (gravity and
    Bounce off), the box staying about 1 m ahead of it and moving at about 2 m/s, with no solver message."""
    dop = new_dopnet("rbd_kinematic_pushes")
    pusher = rbd_object(dop, "pusher", box_sop("rbd_kinematic_pusher", (0, 0, 0)))
    data = box3d_data(dop, pusher, "pusher_data", bodytype=1, bounce=0.0)
    mover = create(dop, "position", "mover")
    mover.setInput(0, data)
    set_menu_token(mover, ["parmop_t"], "always")
    mover.parm("tx").setExpression("($FF-1)/$FPS*2")
    box = rbd_object(dop, "box", box_sop("rbd_kinematic_box", (1.2, 0, 0)))
    set_parm(box, ["bounce"], 0.0)
    solver = box3d_solver(dop, merge(dop, [mover, box]), gravity=(0, 0, 0))
    finish(dop, solver)

    def sample(sim, f):
        return (centroid(whole_object_points(sim, "pusher"))[0], centroid(whole_object_points(sim, "box"))[0])

    track = step(dop, 25, sample)
    (px, bx), (_, bx_prev) = track[25], track[24]
    speed = (bx - bx_prev) * hou.fps()
    msgs = node_messages(solver)
    ok = close(px, 2.0, 0.05) and close(bx - px, 1.0, 0.1) and close(speed, 2.0, 0.3) and not msgs
    return [Result("rbd_object_kinematic_pushes", Result.PASS if ok else Result.FAIL,
                   f"after 1 s the kinematic pusher is at x {px:.2f} (expected 2.0) and the box {bx - px:.2f} ahead of it "
                   f"(expected 1.0), moving at {speed:.2f} m/s (expected 2.0); messages {msgs or 'none'}")]


def _inertia_node(dop, upstream, name):
    """The RBD Object has no inertia parameters: they belong to the RBD State data. Looks for a
    DOP node type that has computeinertialtensortype (tried on a throw-away node, so the search
    does not guess a name), wires the first match after `upstream` and returns (node, tried).
    Returns (None, tried) when no node type has it."""
    tried = []
    cat = hou.dopNodeTypeCategory()
    names = sorted(n for n in cat.nodeTypes() if any(k in n.lower() for k in ("rbd", "state", "inertia", "motion")))
    for type_name in names:
        probe = None
        try:
            probe = dop.createNode(type_name, f"probe_{type_name}"[:40])
            if probe.parm("computeinertialtensortype") is not None:
                probe.destroy()
                node = dop.createNode(type_name, name)
                node.setInput(0, upstream)
                return node, tried
            tried.append(type_name)
        except hou.Error:
            tried.append(f"{type_name} (cannot create)")
        finally:
            if probe is not None:
                try:
                    probe.destroy()
                except hou.Error:
                    pass
    return None, tried


def _disc_sop(name):
    """A SOP object holding a flat 12-sided disc (radius 0.5, height 0.2, centre at y 0.1)."""

    def build(geo):
        tube = create(geo, "tube")
        # The default Primitive Type is Primitive: one quadric primitive on one point, which has
        # no extent as far as the points go (mass 8e-6, a bounding box of 0)
        set_menu_token(tube, ["type"], "poly")
        set_parm(tube, ["rad"], (0.5, 0.5))
        set_parm(tube, ["height"], 0.2)
        set_parm(tube, ["cap"], 1)
        xf = create(geo, "xform")
        xf.setInput(0, tube)
        set_parm(xf, ["t"], (0, 0.1, 0))
        return xf

    return single_object_geo(name, build)


def _position_options(sim_object, key):
    """The options of the object's Position data whose name contains `key`, for a failure message."""
    try:
        data = sim_object.findSubData("Position")
        opts = data.options()
        return {n: opts.field(n) for n in opts.fieldNames() if key in n.lower()}
    except Exception as e:  # diagnostics only
        return f"could not read: {e}"


def test_rbd_object_inertia_type():
    """An RBD Object's RBD State has Compute Inertial Tensor (computeinertialtensor) and a type
    menu, Use Volume / Use Bounding Box (computeinertialtensortype). A flat disc spinning about
    its axis on the ground is slowed by friction in proportion to its inertia: the bounding box
    of the disc (1 x 0.2 x 1) is a solid box with a third more inertia than the disc itself, so
    with Use Bounding Box it turns about a third further before friction stops it. The RBD Object
    node has no such parameters, so they are set on the node that has computeinertialtensortype
    (found by _inertia_node). Also checks that Compute Inertial Tensor off keeps the volume
    inertia."""
    results = []
    turned = {}
    seen = {}
    cases = (("volume", "sdf", 1), ("bounds", "bbox", 1), ("bounds_but_off", "bbox", 0))  # Use Volume, Use Bounding Box
    for label, token, compute in cases:
        dop = new_dopnet(f"rbd_inertia_{label}")
        obj = rbd_object(dop, "disc", _disc_sop(f"rbd_inertia_{label}_disc"), angvel=(0, 573, 0))
        holder = obj if obj.parm("computeinertialtensortype") is not None else None
        if holder is None:
            holder, tried = _inertia_node(dop, obj, "inertia_state")
        if holder is None:
            results.append(Result("rbd_object_inertia_type", Result.INFO,
                                  f"no node type has computeinertialtensortype; tried: {tried}"))
            return results
        set_menu_token(holder, ["computeinertialtensortype"], token)
        set_parm(holder, ["computeinertialtensor"], compute)
        # The object already has its RBD State: only Set Always changes it.
        for name in ("parmop_computeinertialtensortype", "parmop_computeinertialtensor"):
            set_menu_token(holder, [name], "always")
        finish(dop, box3d_solver(dop, merge(dop, [ground_object(dop), holder])))

        def sample(sim, f):
            node = sim.findObject("disc")
            if f == 1:
                seen[label] = _position_options(node, "inertial")
            m = node.transform()
            return math.degrees(math.atan2(-m.at(0, 2), m.at(0, 0)))

        turned[label] = abs(step(dop, 37, sample)[37])
    vol, box, off = turned["volume"], turned["bounds"], turned["bounds_but_off"]
    ok = vol > 20.0 and box > 1.15 * vol and close(off, vol, 0.1 * vol)
    results.append(Result("rbd_object_inertia_type", Result.PASS if ok else Result.FAIL,
                          f"the disc turned {vol:.1f} degrees with Use Volume, {box:.1f} with Use Bounding Box "
                          f"(expected about a third more) and {off:.1f} with Use Bounding Box but Compute Inertial "
                          f"Tensor off (expected the same as Use Volume)"
                          + ("" if ok else f". Inertia options on the object's Position data on frame 1: {seen}. "
                             "If Use Bounding Box shows a different value than Use Volume but the result is the "
                             "same, rebuild the plugin (build.bat) or the solver is not reading it")))
    return results


def test_data_change_midsim():
    """Changing Box3D Data's Friction mid-simulation (its parmop_friction menu at Set Always)
    updates an existing body: an RBD Object box sliding with friction 0 stops once friction
    becomes 1 at frame 13. (A whole object: RBD Packed Object would override friction with its
    own point attribute.)"""
    dop = new_dopnet("friction_change")
    obj = rbd_object(dop, "box", box_sop("friction_box", (0, 0.5, 0)), vel=(5, 0, 0))
    data = box3d_data(dop, obj, "data", bounce=0.0)
    set_expression(data, ["friction"], "if($F < 13, 0, 1)")
    target = data.parm("parmop_friction")
    if target is None:
        ops = ", ".join(p.name() for p in data.parms() if p.name().startswith("parmop"))
        raise SetupError(f"no parmop_friction on Box3D Data; operation menus: {ops}")
    set_menu_token(data, ["parmop_friction"], "always")
    ground_path, _ = packed_boxes("friction_ground", [dict(GROUND, attrs={"active": 0})])
    ground = packed_object(dop, "ground", ground_path, friction=1.0)
    finish(dop, box3d_solver(dop, merge(dop, [ground, data])))
    track = step(dop, 40, lambda sim, f: centroid(whole_object_points(sim, "box")))
    v12 = (track[12][0] - track[11][0]) * hou.fps()
    v40 = (track[40][0] - track[39][0]) * hou.fps()
    ok = v12 > 4.0 and v40 < 0.5
    return [Result("data_change_midsim", Result.PASS if ok else Result.FAIL,
                   f"speed {v12:.2f} on frame 12 (friction 0) and {v40:.2f} on frame 40 (friction 1 from frame 13; "
                   "parmop_friction = Set Always; ground friction 1)")]


def test_data_under_geometry_objects():
    """Box3D Data applied to a Static Object and to an RBD Object lands under the object's
    Geometry (Geometry/Box3DData) and is read: Gravity Scale 0 on an RBD Object keeps it in
    place, and Bounce on a Static Object grid (and on the box) decides whether a dropped RBD
    Object box rebounds (1) or stays down (0)."""
    results = []
    # RBD Object: Gravity Scale 0 holds the box where it starts, 1 lets it fall.
    held = {}
    found = {}
    for label, scale in (("held", 0.0), ("free", 1.0)):
        dop = new_dopnet(f"data_geometry_{label}")
        obj = rbd_object(dop, "box", box_sop(f"data_geometry_{label}_box", (0, 3, 0)))
        data = box3d_data(dop, obj, "data", gravityscale=scale)
        finish(dop, box3d_solver(dop, data))

        def sample(sim, f):
            box = sim.findObject("box")
            sub = box.findSubData("Geometry/Box3DData") if box is not None else None
            found[label] = sub is not None
            return centroid(whole_object_points(sim, "box"))[1]

        held[label] = step(dop, 24, sample)[24]
    ok = (close(held["held"], 3.0, 0.01) and held["free"] < 2.5
          and found.get("held") and found.get("free"))
    results.append(Result("data_under_geometry_objects", Result.PASS if ok else Result.FAIL,
                          f"after 1 s the box is at y {held['held']:.3f} with Gravity Scale 0 (expected 3.0) "
                          f"and {held['free']:.3f} with 1; Geometry/Box3DData present: {found}"))
    # Static Object: the grid's Box3D Data Bounce decides the rebound of a box that has its own Bounce set alike.
    peaks = {}
    for label, bounce in (("bouncy", 1.0), ("dead", 0.0)):
        geo = new_geo(f"data_geometry_grid_{label}")
        grid = create(geo, "grid")
        set_parm(grid, ["size"], (10, 10))
        display(grid)
        dop = new_dopnet(f"data_geometry_static_{label}")
        static = create(dop, "staticobject", "collider")
        set_parm(static, ["soppath"], grid.path())
        static_data = box3d_data(dop, static, "static_data", bounce=bounce)
        obj = rbd_object(dop, "box", box_sop(f"data_geometry_static_{label}_box", (0, 3, 0)))
        box_data = box3d_data(dop, obj, "box_data", bounce=bounce)
        finish(dop, box3d_solver(dop, merge(dop, [static_data, box_data])))
        track = step(dop, 72, lambda sim, f: centroid(whole_object_points(sim, "box"))[1])
        # The lowest frame is the landing: the highest point after it is the rebound.
        landing = min(range(1, 73), key=lambda f: track[f])
        peaks[label] = max(track[f] for f in range(landing, 73))
    ok = peaks["bouncy"] > peaks["dead"] + 0.5
    results.append(Result("data_under_geometry_objects", Result.PASS if ok else Result.FAIL,
                          f"rebound height after landing: {peaks['bouncy']:.3f} with Bounce 1 and "
                          f"{peaks['dead']:.3f} with Bounce 0 on the Static Object and the box (expected 1 well above 0)"))
    return results


def test_static_object_transform():
    """A Static Object with Use Object Transform, whose geo object is moved up 1: a box lands on the
    moved grid (rests at y = 1.5)."""
    geo = new_geo("raised_grid")
    grid = create(geo, "grid")
    set_parm(grid, ["size"], (10, 10))
    display(grid)
    set_parm(geo, ["t"], (0, 1, 0))
    dop = new_dopnet("static_transform")
    static = create(dop, "staticobject", "collider")
    set_parm(static, ["soppath"], grid.path())
    set_parm(static, ["usetransform"], 1)
    path, names = packed_boxes("static_transform_box", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    finish(dop, box3d_solver(dop, merge(dop, [packed_object(dop, "box", path), static])))
    y = step(dop, 72, lambda sim, f: packed_positions(sim, "box"))[72][names[0]][1]
    return [Result("static_object_transform", Result.PASS if close(y, 1.5, 0.03) else Result.FAIL,
                   f"box rests at y={y:.3f} (expected 1.5 on the grid moved up 1)")]


def test_constraint_type_ignored_warning():
    """constraint_type is read for SideFX data only: on a line that names one of our nodes, `position` or `rotation` earns a
    solver warning (the joint type decides what is constrained), `all` and no attribute do not, and the joint holds
    in every case."""
    results = []
    for label, attrs, expect_warning in (("none", None, False), ("all", {"constraint_type": "all"}, False),
                                         ("position", {"constraint_type": "position"}, True),
                                         ("rotation", {"constraint_type": "rotation"}, True)):
        dop, names = hanging_pair(f"conn_type_{label}", "weld", conrels=[("box3d_weldconrel", {})], line_attrs=attrs)
        solver = dop.node("box3d_solver")
        track = step(dop, 12, lambda sim, f: (packed_positions(sim, "pieces")[names[1]][1], node_messages(solver)))
        held = abs(track[12][0] - 2.0) < 0.1
        warned = any("constraint_type" in m for f in track for m in track[f][1])
        ok = held and warned == expect_warning
        results.append(Result("constraint_type_ignored_warning", Result.PASS if ok else Result.FAIL,
                              f"constraint_type {label}: warned {warned} (expected {expect_warning}), hanging piece "
                              f"at y {track[12][0]:.2f} (held at 2)"))
    return results


TESTS = {
    "breaking": test_breaking,
    "anchor_id": test_anchor_id,
    "sidefx_spring_mass": test_sidefx_spring_mass,
    "attribute_multiplies": test_attribute_multiplies,
    "next_constraint": test_next_constraint,
    "next_settings": test_next_settings,
    "unset_attributes": test_unset_attributes,
    "feedback": test_feedback,
    "feedback_many": test_feedback_many,
    "constraint_type_ignored_warning": test_constraint_type_ignored_warning,
    "sidefx_hard": test_sidefx_hard,
    "sidefx_hard_all": test_sidefx_hard_all,
    "rbd_first_frames": test_rbd_first_frames,
    "rbd_initial_motion": test_rbd_initial_motion,
    "rbd_object_transform": test_rbd_object_transform,
    "other_rbd_objects": test_other_rbd_objects,
    "rbd_object_active": test_rbd_object_active,
    "rbd_object_kinematic_pushes": test_rbd_object_kinematic_pushes,
    "rbd_object_inertia_type": test_rbd_object_inertia_type,
    "data_change_midsim": test_data_change_midsim,
    "data_under_geometry_objects": test_data_under_geometry_objects,
    "static_object_transform": test_static_object_transform,
}
