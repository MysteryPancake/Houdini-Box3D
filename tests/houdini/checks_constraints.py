"""Constraint and whole-object checks: breaking and the broken group, attributes multiplying the
node's values, next_constraint_name, feedback attributes, centroid anchors, Hard / Spring compared
with Bullet; RBD Object first-frame drift, initial velocity and spin, Use Object Transform, other
RBD object types, a Box3D Data change mid-simulation, a Static Object's object transform."""

import math

import hou

from b3dtest import (
    Result,
    SetupError,
    box3d_data,
    box3d_solver,
    bullet_solver,
    centroid,
    close,
    constraint_lines,
    create,
    display,
    find_network_geometry,
    finish,
    merge,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    packed_positions,
    parm_value,
    rbd_object,
    set_expression,
    set_menu_by_label,
    set_parm,
    single_object_geo,
    step,
    whole_object_points,
)

GROUND = {"size": (20, 1, 20), "center": (0, -0.5, 0), "attrs": {"active": 0}}
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
    last = box3d_solver(dop, net, gravity=gravity) if solver_kind == "box3d" else bullet_solver(dop, net)
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
    force_threshold 1e-12 on the line breaks the joint."""
    dop, names = hanging_pair("multiply", "weld", line_attrs={"force_threshold": 1e-12},
                              conrels=[("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e9})])
    y = step(dop, 24, lambda sim, f: packed_positions(sim, "pieces"))[24][names[1]][1]
    return [Result("attribute_multiplies", Result.PASS if y < 1.5 else Result.FAIL,
                   f"piece at y={y:.2f} (it falls only if 1e9 x 1e-12 was used as the break force)")]


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


def test_feedback():
    """The solver writes force, torque, distance and angle onto a holding weld's line."""
    dop, names = hanging_pair("feedback", "weld", conrels=[("box3d_weldconrel", {})],
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


def test_centroid_anchors():
    """Two free pieces welded with each anchor at its own centroid (as Connect Adjacent Pieces
    makes them): how far apart they end up. INFO: the notes say a Box3D weld pulls them together."""
    pieces = [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
              {"size": (1, 1, 1), "center": (1, 2, 0), "attrs": {"active": 1}}]
    dop, names = hanging_pair("centroid_anchors", "weld", conrels=[("box3d_weldconrel", {})], gravity=(0, 0, 0),
                              pieces=pieces, anchors=((0, 2, 0), (1, 2, 0)))
    track = step(dop, 48, lambda sim, f: packed_positions(sim, "pieces"))
    d = (track[48][names[0]] - track[48][names[1]]).length()
    return [Result("centroid_anchors", Result.INFO,
                   f"centroids 1.0 apart at the start, {d:.3f} after 2 s (SideFX Glue / Hard would keep 1.0)")]


def sidefx_compare(kind, conrel_type, line_attrs, spin=None):
    """A piece 2 below a fixed one, held by a SideFX constraint with anchors at the centroids,
    in Box3D and in Bullet. Returns {solver: (centroid distance, rotation angle in degrees)}
    after 2 s. spin: initial w (rad/s) of the hanging piece."""
    from b3dtest import packed_transform
    out = {}
    for solver_kind in ("box3d", "bullet"):
        hang = {"active": 1, "w": tuple(spin)} if spin else {"active": 1}
        fixed = {"active": 0, "w": (0.0, 0.0, 0.0)} if spin else {"active": 0}
        pieces = [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": fixed},
                  {"size": (1, 1, 1), "center": (0.5, 2, 0), "attrs": hang}]
        dop, names = hanging_pair(f"{kind}_{solver_kind}{'_spin' if spin else ''}", None, line_attrs=line_attrs,
                                  conrels=[(conrel_type, {})], solver_kind=solver_kind, pieces=pieces,
                                  anchors=((0, 4, 0), (0.5, 2, 0)), keep_velocity=bool(spin))

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


def test_sidefx_spring():
    """SideFX Spring constraint compared with Bullet: how far the hanging piece ends up, and
    (constraint_type all, piece spinning at the start) whether an angular spring turns it back.
    INFO."""
    d = sidefx_compare("spring", "springconrel", {"constraint_type": "position"})
    a = sidefx_compare("spring_all", "springconrel", {"constraint_type": "all"}, spin=(0.0, 3.0, 0.0))
    return [Result("sidefx_spring", Result.INFO,
                   f"position spring, distance after 2 s: Box3D {d['box3d'][0]:.3f}, Bullet {d['bullet'][0]:.3f} "
                   f"(start 2.06)"),
            Result("sidefx_spring", Result.INFO,
                   f"'all' spring, piece started spinning 3 rad/s about y: turned {a['box3d'][1]:.0f} deg in Box3D, "
                   f"{a['bullet'][1]:.0f} deg in Bullet after 2 s (small = an angular spring pulled it back)")]


# ---------------------------------------------------------------------------
# Whole objects (RBD Object)
# ---------------------------------------------------------------------------


def box_sop(name, center, size=(1, 1, 1)):
    def build(geo):
        box = create(geo, "box")
        set_parm(box, ["size"], tuple(size))
        xf = create(geo, "xform")
        xf.setInput(0, box)
        set_parm(xf, ["t"], tuple(center))
        return xf

    return single_object_geo(name, build)


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
            def build(geo):
                wr = create(geo, "attribwrangle")
                set_parm(wr, ["class"], 0)
                set_parm(wr, ["snippet"], "addpoint(0, {0,3,0}); addpoint(0, {3,3,0});")
                return wr
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
            def unit_box(geo):
                return create(geo, "box")
            set_parm(obj, ["geopath", "instancepath", "geometrypath", "instance"],
                     single_object_geo("point_instance_box", unit_box))
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
        fell = n > 0 and len(track[1]) == n and all(b < a - 0.5 for a, b in zip(track[1], track[24]))
        results.append(Result("other_rbd_objects", Result.PASS if fell else Result.FAIL,
                              f"{label}: {n} object(s) with geometry; lowest points {[round(y, 2) for y in track[1]]} "
                              f"on frame 1, {[round(y, 2) for y in track[24]]} on frame 24"))
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
    set_menu_by_label(data, ["parmop_friction"], "Always")
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


TESTS = {
    "breaking": test_breaking,
    "attribute_multiplies": test_attribute_multiplies,
    "next_constraint": test_next_constraint,
    "feedback": test_feedback,
    "centroid_anchors": test_centroid_anchors,
    "sidefx_hard": test_sidefx_hard,
    "sidefx_hard_all": test_sidefx_hard_all,
    "sidefx_spring": test_sidefx_spring,
    "rbd_first_frames": test_rbd_first_frames,
    "rbd_initial_motion": test_rbd_initial_motion,
    "rbd_object_transform": test_rbd_object_transform,
    "other_rbd_objects": test_other_rbd_objects,
    "data_change_midsim": test_data_change_midsim,
    "static_object_transform": test_static_object_transform,
}
