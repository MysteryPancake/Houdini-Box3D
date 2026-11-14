"""Helpers for the Houdini checks in run_tests.py: build small scenes from scratch, step them,
read results back. Runs inside hython (Houdini's Python); needs the Box3D plugin in the dso
folder (build.bat puts it there)."""

import math

import hou


class SetupError(Exception):
    """The scene could not be built (a node type or parameter name not found). Not a test
    failure: the message says what to fix in this script."""


class Result:
    PASS, FAIL, INFO, ERROR, SETUP, CRASH = "PASS", "FAIL", "INFO", "ERROR", "SETUP", "CRASH"

    def __init__(self, test, status, message):
        self.test, self.status, self.message = test, status, message


# ---------------------------------------------------------------------------
# Nodes and parameters
# ---------------------------------------------------------------------------


def create(parent, type_name, name=None):
    try:
        return parent.createNode(type_name, name) if name else parent.createNode(type_name)
    except hou.OperationFailed as e:
        raise SetupError(f"cannot create a '{type_name}' node in {parent.path()}: {e}")


def set_parm(node, names, value):
    """Sets the first of `names` that exists. A tuple value sets a parameter tuple. Raises
    SetupError listing the node's parameters if none exists, so a wrong guess is easy to fix."""
    if isinstance(names, str):
        names = [names]
    for n in names:
        if isinstance(value, (tuple, list)):
            pt = node.parmTuple(n)
            if pt is not None and len(pt) == len(value):
                pt.deleteAllKeyframes()
                pt.set(value)
                return n
        else:
            p = node.parm(n)
            if p is not None:
                p.deleteAllKeyframes()  # a default expression (e.g. $FSTART) would otherwise win
                p.set(value)
                return n
            pt = node.parmTuple(n)
            if pt is not None and len(pt) == 1:
                pt.set((value,))
                return n
    available = ", ".join(sorted({pt.name() for pt in node.parmTuples()}))
    raise SetupError(
        f"{node.path()} ({node.type().name()}): no parameter named {' / '.join(names)}. "
        f"Its parameters are: {available}"
    )


def set_menu_by_label(node, names, label_part):
    """Sets a menu parameter to the first item whose label contains label_part (case-insensitive)."""
    for n in names:
        p = node.parm(n)
        if p is None:
            continue
        labels = list(p.menuLabels())
        for i, label in enumerate(labels):
            if label_part.lower() in label.lower():
                p.deleteAllKeyframes()
                items = p.menuItems()
                try:
                    p.set(i)
                except (TypeError, hou.OperationFailed):
                    p.set(items[i])
                return n
        raise SetupError(f"{node.path()}: menu '{n}' has no item like '{label_part}'; items: {labels}")
    set_parm(node, names, 0)  # raises with the parameter list


def set_expression(node, names, expr):
    """Sets an HScript expression on the first of `names` that exists."""
    for n in [names] if isinstance(names, str) else names:
        p = node.parm(n)
        if p is not None:
            p.setExpression(expr, hou.exprLanguage.Hscript)
            return n
    set_parm(node, names, 0)  # raises with the parameter list


def parm_value(node, names, default=None):
    for n in names:
        p = node.parm(n)
        if p is not None:
            return p.eval()
    return default


def node_type_exists(category, name):
    return hou.nodeType(category, name) is not None


def display(node):
    node.setDisplayFlag(True)
    if hasattr(node, "setRenderFlag"):
        node.setRenderFlag(True)


def vex_value(v):
    if isinstance(v, str):
        return "s", f'"{v}"'
    if isinstance(v, bool) or isinstance(v, int):
        return "i", str(int(v))
    if isinstance(v, float):
        return "f", repr(v)
    if isinstance(v, (tuple, list)) and len(v) == 3:
        return "v", "set(%r, %r, %r)" % tuple(float(x) for x in v)
    raise ValueError(f"unsupported attribute value {v!r}")


# ---------------------------------------------------------------------------
# Geometry (SOP objects under /obj)
# ---------------------------------------------------------------------------


def new_geo(name):
    geo = create(hou.node("/obj"), "geo", name)
    for child in geo.children():  # some versions start a geo object with a File SOP
        child.destroy()
    return geo


# What a piece gets for an attribute it doesn't set (Bullet's meaning of a missing attribute).
ATTR_DEFAULTS = {"active": 1, "animated": 0, "v": (0.0, 0.0, 0.0), "w": (0.0, 0.0, 0.0), "life": 0.0, "age": 0.0}


def packed_boxes(name, pieces):
    """A SOP object of packed boxes, one packed primitive (and point) per piece.
    pieces: list of dicts with size (x, y, z), center (x, y, z), optional rotate (degrees) and
    attrs {point attribute: value}. Returns (sop_path, piece_names) with piece_names[i] the
    `name` of pieces[i]."""
    geo = new_geo(name)
    merge = create(geo, "merge")
    for i, pc in enumerate(pieces):
        box = create(geo, "box", f"box{i}")
        set_parm(box, ["size"], tuple(pc["size"]))
        xf = create(geo, "xform", f"place{i}")
        xf.setInput(0, box)
        set_parm(xf, ["r"], tuple(pc.get("rotate", (0, 0, 0))))
        set_parm(xf, ["t"], tuple(pc["center"]))
        merge.setInput(i, xf)
    asm = create(geo, "assemble")
    asm.setInput(0, merge)
    set_parm(asm, ["pack_geo", "packgeo", "pack"], 1)

    # Assemble names pieces itself; match them to our pieces by nearest centre.
    points = [(pt.attribValue("name"), pt.position()) for pt in asm.geometry().points()]
    if len(points) != len(pieces):
        raise SetupError(f"{asm.path()} made {len(points)} packed pieces, expected {len(pieces)}")
    names = []
    for pc in pieces:
        c = hou.Vector3(pc["center"])
        names.append(min(points, key=lambda p: (p[1] - c).length())[0])

    # Every piece gets every attribute any piece sets: VEX creates an attribute on all points
    # with a 0 default, so a value set on one piece alone would silently give the others 0
    # (active = 0 makes them static).
    attr_names = []
    for pc in pieces:
        for attr in pc.get("attrs", {}):
            if attr not in attr_names:
                attr_names.append(attr)
    lines = []
    for pc, nm in zip(pieces, names):
        sets = []
        for attr in attr_names:
            val = pc.get("attrs", {}).get(attr, ATTR_DEFAULTS.get(attr))
            if val is None:
                raise ValueError(f"piece {nm} has no value for '{attr}' and there is no default for it")
            t, text = vex_value(val)
            sets.append(f"{t}@{attr} = {text};")
        if sets:
            lines.append(f'if (s@name == "{nm}") {{ {" ".join(sets)} }}')
    out = asm
    if lines:
        wr = create(geo, "attribwrangle", "attributes")
        wr.setInput(0, asm)
        set_parm(wr, ["snippet"], "\n".join(lines))
        out = wr
    display(out)
    return out.path(), names


def plain_points(name, points):
    """A SOP object of loose points (no primitives). points: list of {attribute: value} with P."""
    geo = new_geo(name)
    wr = create(geo, "attribwrangle", "points")
    set_parm(wr, ["class"], 0)  # run over detail
    lines = []
    for pt in points:
        lines.append("{ int p = addpoint(0, %s);" % vex_value(pt["P"])[1])
        for attr, val in pt.items():
            if attr == "P":
                continue
            t, text = vex_value(val)
            # The default's type decides the attribute's type: 0.0, not 0, for a float.
            default = {"s": '""', "i": "0", "f": "0.0", "v": "{0,0,0}"}[t]
            lines.append(f'  addpointattrib(0, "{attr}", {default}); setpointattrib(0, "{attr}", p, {text}); ')
        lines.append("}")
    set_parm(wr, ["snippet"], "\n".join(lines))
    display(wr)
    return wr.path()


def constraint_lines(name, lines):
    """A constraint network: lines is a list of (name_a, pos_a, name_b, pos_b, constraint_name)
    with an optional sixth item, {primitive attribute: value} for that line."""
    geo = new_geo(name)
    wr = create(geo, "attribwrangle", "network")
    set_parm(wr, ["class"], 0)
    code = ['addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", "");']
    declared = set()
    for line in lines:
        for attr, val in (line[5] if len(line) > 5 else {}).items():
            if attr not in declared:
                t = vex_value(val)[0]
                default = {"s": '""', "i": "0", "f": "0.0", "v": "{0,0,0}"}[t]
                code.append(f'addprimattrib(0, "{attr}", {default});')
                declared.add(attr)
    for line in lines:
        na, pa, nb, pb, cname = line[:5]
        extra = "".join(f' setprimattrib(0, "{attr}", pr, {vex_value(val)[1]});'
                        for attr, val in (line[5] if len(line) > 5 else {}).items())
        code.append(
            "{ int a = addpoint(0, %s); int b = addpoint(0, %s);" % (vex_value(pa)[1], vex_value(pb)[1])
            + f' setpointattrib(0, "name", a, "{na}"); setpointattrib(0, "name", b, "{nb}");'
            + f' int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "{cname}");'
            + extra + " }"
        )
    set_parm(wr, ["snippet"], "\n".join(code))
    display(wr)
    return wr.path()


def single_object_geo(name, nodes):
    """A SOP object from a chain built by `nodes(geo)`, which returns the output node."""
    geo = new_geo(name)
    out = nodes(geo)
    display(out)
    return out.path()


# ---------------------------------------------------------------------------
# DOP networks
# ---------------------------------------------------------------------------


def new_dopnet(name="dop"):
    return create(hou.node("/obj"), "dopnet", name)


def packed_object(dop, name, sop_path, keep_velocity=False, **parms):
    """An RBD Packed Object. keep_velocity: keep the points' v / w (else RBD Packed Object
    replaces them with its own initial velocity). parms: its own parameters, e.g. mass=1.0,
    computemass=0: RBD Packed Object writes these onto every point as attributes, which
    override Box3D Data (see test rbd_packed_attributes)."""
    obj = create(dop, "rbdpackedobject", name)
    set_parm(obj, ["soppath"], sop_path)
    if keep_velocity:
        set_parm(obj, ["inheritvelocity", "usepointvelocity", "inheritvel"], 1)
    for parm, value in parms.items():
        set_parm(obj, [parm], value)
    return obj


def box3d_packed_object(dop, name, sop_path, **parms):
    """A Box3D Packed Object (the asset tools/make_packed_object.py builds): the packed geometry
    without RBD Packed Object's stamped attributes; parms: its parameters (Box3D Data's, e.g.
    friction=0.0). SetupError if the asset is not installed."""
    obj = create(dop, "box3d_packedobject", name)
    set_parm(obj, ["soppath"], sop_path)
    for parm, value in parms.items():
        set_parm(obj, [parm], value)
    return obj


def rbd_object(dop, name, sop_path, vel=None, angvel=None):
    obj = create(dop, "rbdobject", name)
    set_parm(obj, ["soppath"], sop_path)
    if vel is not None:
        set_parm(obj, ["vel", "v", "velocity"], tuple(vel))
    if angvel is not None:
        set_parm(obj, ["angvel", "w"], tuple(angvel))
    return obj


def box3d_data(dop, upstream, name, **parms):
    """A Box3D Data node applied to the objects coming from upstream."""
    data = create(dop, "box3d_data", name)
    data.setInput(0, upstream)
    for parm, value in parms.items():
        set_parm(data, [parm], value)
    return data


def merge(dop, nodes, name="merge"):
    m = create(dop, "merge", name)
    for i, n in enumerate(nodes):
        m.setInput(i, n)
    return m


def box3d_solver(dop, upstream, gravity=None, name="box3d_solver"):
    """A Box3D Solver; upstream None makes one to wire into a Multiple Solver."""
    solver = create(dop, "box3d_solver", name)
    if upstream is not None:
        solver.setInput(0, upstream)
    if gravity is not None:
        set_parm(solver, ["gravity"], tuple(gravity))
    return solver


def bullet_solver(dop, upstream):
    solver = create(dop, "bulletrbdsolver", "bullet_solver")
    solver.setInput(0, upstream)
    grav = create(dop, "gravity", "gravity")
    grav.setInput(0, solver)
    return grav


def multisolver(dop, upstream, solvers, name="multisolver"):
    """A Multiple Solver: objects from upstream, solved by `solvers` (nodes with no object input)
    in order."""
    ms = create(dop, "multisolver", name)
    ms.setInput(0, upstream)
    for i, s in enumerate(solvers):
        ms.setInput(i + 1, s)
    return ms


def sop_solver(dop, vex, name="sop_solver"):
    """A SOP Solver DOP whose SOP network runs `vex` over the points each step (after its
    default contents). @Frame and @Time are the current frame and time."""
    ss = create(dop, "sopsolver", name)
    wr = ss.createNode("attribwrangle", "edit")
    set_parm(wr, ["snippet"], vex)
    # An Output SOP decides what the network returns (over the display flag): go in front of it.
    outputs = [c for c in ss.children() if c.type().name() == "output"]
    if outputs:
        out = outputs[0]
        wr.setInput(0, out.input(0))
        out.setInput(0, wr)
        return ss
    last = ss.displayNode()
    if last is None:
        kids = ", ".join(f"{c.name()} ({c.type().name()})" for c in ss.children())
        raise SetupError(f"{ss.path()}: no Output or display node inside to add a wrangle to (children: {kids})")
    wr.setInput(0, last)
    display(wr)
    return ss


def sop_solver_contents(ss):
    """What is inside a SOP Solver DOP, for messages."""
    return ", ".join(f"{c.name()} ({c.type().name()}){' [display]' if c.isDisplayFlagSet() else ''}"
                     for c in ss.children())


def append_sop(sop_path, type_name, setup=None):
    """Adds a SOP after the one at sop_path (in the same object), shows it and returns its path.
    setup(node) sets its parameters."""
    prev = hou.node(sop_path)
    node = create(prev.parent(), type_name)
    node.setInput(0, prev)
    if setup:
        setup(node)
    display(node)
    return node.path()


def finish(dop, last):
    out = create(dop, "output", "output")
    out.setInput(0, last)
    display(out)
    return out


def step(dop, frames, sample):
    """Plays frames 1..frames in order; sample(sim, frame) is stored per frame."""
    out = {}
    for f in range(1, frames + 1):
        hou.setFrame(f)
        sim = dop.simulation()
        sim.objects()  # makes sure this frame is cooked
        out[f] = sample(sim, f)
    return out


def node_messages(node):
    msgs = []
    for get in ("errors", "warnings", "messages"):
        try:
            msgs.extend(getattr(node, get)())
        except Exception:
            pass
    return msgs


# ---------------------------------------------------------------------------
# Reading results
# ---------------------------------------------------------------------------


def packed_positions(sim, obj_name):
    """{piece name: world position} of a packed / points object, or None if absent."""
    obj = sim.findObject(obj_name)
    if obj is None:
        return None
    geo = obj.geometry()
    if geo is None:
        return {}
    has_name = geo.findPointAttrib("name") is not None
    return {
        (pt.attribValue("name") if has_name else str(pt.number())): pt.position() for pt in geo.points()
    }


def point_attributes(sim, obj_name, point_name=None, names=None):
    """{attribute: value} of one point of a DOP object's geometry as the solver sees it (the
    point named point_name, else the first). names: only these attributes (missing ones are
    reported as None); default all."""
    obj = sim.findObject(obj_name)
    geo = obj.geometry() if obj is not None else None
    if geo is None or not geo.points():
        return None
    pt = geo.points()[0]
    if point_name is not None and geo.findPointAttrib("name") is not None:
        pt = next((p for p in geo.points() if p.attribValue("name") == point_name), pt)
    attrs = {a.name(): a for a in geo.pointAttribs()}
    wanted = names if names is not None else sorted(attrs)
    return {n: (pt.attribValue(attrs[n]) if n in attrs else None) for n in wanted}


def whole_object_points(sim, obj_name):
    """World positions of a whole object's (RBD Object) geometry points, or None if absent."""
    obj = sim.findObject(obj_name)
    if obj is None:
        return None
    xform = obj.transform()
    return [pt.position() * xform for pt in obj.geometry().points()]


def packed_transform(sim, obj_name, piece_name=None):
    """The 3x3 rotation / scale rows of a packed piece (row vectors), or None."""
    obj = sim.findObject(obj_name)
    geo = obj.geometry() if obj is not None else None
    if geo is None:
        return None
    prims = list(geo.prims())
    if piece_name is not None and geo.findPointAttrib("name") is not None:
        for pr in prims:
            pts = pr.points()
            if pts and pts[0].attribValue("name") == piece_name:
                prims = [pr]
                break
    if not prims:
        return None
    m = prims[0].intrinsicValue("packedfulltransform")
    return [list(m[0:3]), list(m[4:7]), list(m[8:11])]


def det3(r):
    return (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))


def object_bbox(sim, obj_name):
    """(min, max) world corners of a DOP object's geometry (packed transforms included)."""
    obj = sim.findObject(obj_name)
    geo = obj.geometry() if obj is not None else None
    if geo is None:
        return None
    bb = geo.boundingBox()
    return bb.minvec(), bb.maxvec()


def find_network_geometry(sim):
    """The constraint network geometry as the solver left it (primitives with constraint_name)."""
    for rel in sim.relationships():
        candidates = [rel]
        try:
            candidates += list(rel.findAllSubData("*", recurse=True).values())
        except Exception:
            pass
        for d in candidates:
            try:
                g = d.geometry()
            except Exception:
                continue
            if g is not None and g.findPrimAttrib("constraint_name") is not None:
                return g
    return None


def centroid(points):
    n = len(points)
    return hou.Vector3(sum(p[0] for p in points) / n, sum(p[1] for p in points) / n, sum(p[2] for p in points) / n)


def velocity(track, f, axis):
    """Finite-difference velocity of a {frame: Vector3} track at frame f, in units per second."""
    return (track[f][axis] - track[f - 1][axis]) * hou.fps()


def close(a, b, tol):
    return math.isfinite(a) and abs(a - b) <= tol
