"""Scene-building helpers shared by make_examples.py and the Box3D sample ports (box3d_samples/).

A scene's bodies live in one geometry object (Pieces): one Copy to Points stream per shape, packed, one
packed primitive per body. network_sop makes a constraint network, build_dops the DOP network around a
Box3D Packed Object and a Box3D Solver, and tidy sets the frame range, lays the nodes out and adds the
sticky notes. Run inside hython (make_examples.bat).
"""

import colorsys
import math
import os
import sys

sys.dont_write_bytecode = True

import hou  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tests", "houdini"))
from b3dtest import SetupError, create, display, set_parm  # noqa: E402,F401  (the checks' scene helpers)
from review import place_note_left  # noqa: E402

# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def set_token(node, name, token):
    """Sets a menu to its item `token`."""
    p = node.parm(name)
    items = list(p.menuItems()) if p is not None else []
    if token not in items:
        raise SetupError(f"{node.path()}: menu '{name}' has no item {token!r}: {items}")
    p.set(items.index(token) if p.parmTemplate().type() == hou.parmTemplateType.Menu else token)


def quat_about(axis, angle):
    """Houdini quaternion (x, y, z, w) turning `angle` radians about a unit axis."""
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def rainbow(t):
    return colorsys.hsv_to_rgb(t % 1.0, 0.65, 0.95)


def num(x):
    """A VEX float literal (always with a decimal point, so VEX never reads it as an int). Nine significant digits
    give back any 32-bit float exactly: a hull's points rounded further can change its faces."""
    text = f"{float(x):.9g}"
    return text if any(c in text for c in ".en") else text + ".0"


def vec(v):
    """A VEX vector / vector4 literal. The cast picks set()'s overload: with four floats it
    could also make a matrix2."""
    return ("vector4" if len(v) == 4 else "vector") + "(set(" + ", ".join(num(x) for x in v) + "))"


# ---------------------------------------------------------------------------
# Building pieces
#
# A scene's bodies live in one geometry object. Each "stream" is one shape (a Box SOP, a Sphere
# SOP...) copied onto points with Copy to Points, packed: one packed primitive = one rigid body.
# The points are made by a wrangle that calls mk(...) once per body, so every stream carries the
# same attributes and the Merge never fills a missing one with 0 (active 0 would make bodies static).
# ---------------------------------------------------------------------------

PRELUDE = """\
// One rigid body: position, name, rotation, colour, active (0 = static), density, Bullet shape.
int mk(vector p; string nm; vector4 q; vector cd; int act; float dens; string rep) {
    int pt = addpoint(0, p);
    setpointattrib(0, "name", pt, nm);
    setpointattrib(0, "orient", pt, q);
    setpointattrib(0, "Cd", pt, cd);
    setpointattrib(0, "active", pt, act);
    setpointattrib(0, "density", pt, dens);
    setpointattrib(0, "bullet_georep", pt, rep);
    return pt;
}

addpointattrib(0, "name", "");
vector4 identity = {0, 0, 0, 1};  // a typed variable: set() with four floats could also be a matrix2
addpointattrib(0, "orient", identity);
setattribtypeinfo(0, "point", "orient", "quaternion");
vector grey = {0.7, 0.7, 0.7};
addpointattrib(0, "Cd", grey);
addpointattrib(0, "active", 1);
addpointattrib(0, "density", 1000.0);
addpointattrib(0, "bullet_georep", "");
addpointattrib(0, "bullet_autofit", 1);
"""


# The template point attributes Copy to Points transfers onto each packed copy.
COPY_ATTRIBUTES = "name active density bullet_georep bullet_autofit Cd"


def mk(p, name, orient=(0, 0, 0, 1), cd=(0.7, 0.7, 0.7), active=1, density=1000.0, georep="box"):
    """One mk(...) call for a stream's VEX."""
    return (f'mk({vec(p)}, "{name}", {vec(orient)}, {vec(cd)}, {int(active)}, '
            f'{num(density)}, "{georep}");')


class Pieces:
    """The geometry object holding a scene's rigid bodies."""

    def __init__(self, name):
        self.geo = create(hou.node("/obj"), "geo", name)
        for child in self.geo.children():
            child.destroy()
        self.merge = create(self.geo, "merge", "all_pieces")
        self.count = 0

    def shape_box(self, tag, size):
        box = create(self.geo, "box", f"{tag}_shape")
        set_parm(box, "size", tuple(size))
        return box

    def shape_sphere(self, tag, radius, freq=3):
        sph = create(self.geo, "sphere", f"{tag}_shape")
        set_token(sph, "type", "poly")  # Polygon. A primitive sphere is a single point: nothing to fit
        set_parm(sph, "rad", (radius, radius, radius))
        set_parm(sph, ["freq"], freq)
        return sph

    def shape_tube(self, tag, radius, height):
        tube = create(self.geo, "tube", f"{tag}_shape")
        set_token(tube, "type", "poly")  # Polygon
        set_parm(tube, "rad", (radius, radius))
        set_parm(tube, "height", height)
        set_parm(tube, "cap", 1)
        set_parm(tube, "cols", 16)
        return tube

    def shape_platonic(self, tag, kind, radius):
        pl = create(self.geo, "platonic", f"{tag}_shape")
        set_token(pl, "type", kind)
        set_parm(pl, ["rad", "radius"], radius)
        return pl

    def shape_parts(self, tag, parts):
        """One body's shape from several boxes: parts is [(size, centre)], the centres relative
        to the body's point. Pair with bullet_georep 'convexhull' (one hull around them all) or 'box'."""
        merge = create(self.geo, "merge", f"{tag}_shape")
        for i, (size, centre) in enumerate(parts):
            box = create(self.geo, "box", f"{tag}_part{i}")
            set_parm(box, "size", tuple(size))
            set_parm(box, "t", tuple(centre))
            merge.setInput(i, box)
        return merge

    def shape_wheel(self, tag, radius, width):
        """A wheel: a capped tube with its axle along z (fitted by a 'sphere' shape, it rolls smoothly)."""
        tube = self.shape_tube(f"{tag}_tube", radius, width)
        turn = create(self.geo, "xform", f"{tag}_shape")
        turn.setInput(0, tube)
        set_parm(turn, "r", (90.0, 0.0, 0.0))
        return turn

    def stream(self, tag, shape, vex_body, note=""):
        """Copies `shape` onto the points that vex_body makes (mk(...) calls or loops)."""
        pts = create(self.geo, "attribwrangle", f"{tag}_points")
        set_parm(pts, "class", 0)  # run once (detail): the code adds the points
        set_parm(pts, "snippet", (f"// {note}\n" if note else "") + PRELUDE + "\n" + vex_body)
        copy = create(self.geo, "copytopoints", f"{tag}_copy")
        copy.setInput(0, shape)
        copy.setInput(1, pts)
        set_parm(copy, ["pack", "packgeo"], 1)
        # Copy to Points carries the template points' body attributes onto the packed copies.
        set_parm(copy, "targetattribs", 1)
        set_parm(copy, "applyattribs1", COPY_ATTRIBUTES)
        self.merge.setInput(len(self.merge.inputs()), copy)
        return copy

    def boxes(self, tag, size, calls, note=""):
        return self.stream(tag, self.shape_box(tag, size), "\n".join(calls), note)

    def finish(self, extra_vex=None):
        """Shows the merged pieces (after an optional point wrangle); returns the SOP path."""
        out = self.merge
        if extra_vex:
            wr = create(self.geo, "attribwrangle", "extra_attributes")
            wr.setInput(0, out)
            set_parm(wr, "snippet", extra_vex)
            out = wr
        null = create(self.geo, "null", "OUT_pieces")
        null.setInput(0, out)
        display(null)
        geo = cooked_geometry(null)
        self.count = len(geo.points())
        for attr in ("name", "active", "density", "bullet_georep"):
            if geo.findPointAttrib(attr) is None:
                raise SetupError(f"{null.path()}: the pieces have no '{attr}' attribute")
        # Packing can drop attributes it is not told to keep: an unnamed piece matches no constraint.
        unnamed = sum(1 for pt in geo.points() if not pt.attribValue("name"))
        if unnamed:
            raise SetupError(f"{null.path()}: {unnamed} piece(s) have an empty name (an attribute lost when packing?)")
        static = [pt.attribValue("name") for pt in geo.points() if pt.attribValue("active") == 0]
        if "ground" in [pt.attribValue("name") for pt in geo.points()] and "ground" not in static:
            raise SetupError(f"{null.path()}: the ground piece is not static (active 0)")
        return null.path()


def cooked_geometry(node):
    """The node's geometry, or SetupError with every error and warning in its network."""
    try:
        node.cook(force=True)
    except hou.OperationFailed:
        pass
    geo = node.geometry()
    if geo is not None and not node.errors():
        return geo
    problems = []
    for n in node.parent().allSubChildren():
        for kind, msgs in (("error", n.errors()), ("warning", n.warnings())):
            for m in msgs:
                problems.append(f"{n.path()} {kind}: {m}")
    raise SetupError(f"{node.path()} did not cook:\n    " + "\n    ".join(problems or ["(no messages)"]))


def ground(pieces, size=40.0):
    pieces.boxes("ground", (size, 1, size),
                 [mk((0, -0.5, 0), "ground", cd=(0.35, 0.36, 0.38), active=0)],
                 "a static box for the floor (active 0); its top is at y = 0")


def network_sop(name, lines):
    """A constraint network: one polyline per joint between two pieces' anchor points.
    lines: (name_a, pos_a, name_b, pos_b, constraint_name[, orient]), or a string of VEX calling joint( name_a,
    pos_a, name_b, pos_b, constraint_name ) (no orient). An empty name pins to the
    world. Both points sit where the joint is; constraint_name picks the relationship node. The
    optional orient (a quaternion, on both ends) turns the joint's axes: a revolute hinge turns
    about its z axis, a prismatic slides along x, and so on."""
    geo = create(hou.node("/obj"), "geo", name)
    for child in geo.children():
        child.destroy()
    wr = create(geo, "attribwrangle", "constraint_lines")
    set_parm(wr, "class", 0)
    turned = any(not isinstance(line, str) and len(line) > 5 for line in lines)
    code = ['// One polyline per joint. Each end names the piece it holds ("" = the world) and',
            '// sits at the joint; constraint_name matches a relationship node\'s Data Name.',
            "void joint(string a; vector pa; string b; vector pb; string cname) {",
            "    int i = addpoint(0, pa), j = addpoint(0, pb);",
            '    setpointattrib(0, "name", i, a); setpointattrib(0, "name", j, b);',
            '    int pr = addprim(0, "polyline", i, j);',
            '    setprimattrib(0, "constraint_name", pr, cname);',
            "}"]
    if turned:
        code += ["// The same with orient on both ends: the joint's axes (no orient = the world axes).",
                 "void joint_turned(string a; vector pa; string b; vector pb; string cname; vector4 q) {",
                 "    int i = addpoint(0, pa), j = addpoint(0, pb);",
                 '    setpointattrib(0, "name", i, a); setpointattrib(0, "name", j, b);',
                 '    setpointattrib(0, "orient", i, q); setpointattrib(0, "orient", j, q);',
                 '    int pr = addprim(0, "polyline", i, j);',
                 '    setprimattrib(0, "constraint_name", pr, cname);',
                 "}"]
    code.append('addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", "");')
    if turned:
        code += ["vector4 identity = {0, 0, 0, 1};  // a typed variable: set() with four floats could be a matrix2",
                 'addpointattrib(0, "orient", identity);',
                 'setattribtypeinfo(0, "point", "orient", "quaternion");']
    for line in lines:
        if isinstance(line, str):  # VEX of its own calling joint(...), for grids of thousands of joints
            code.append(line)
            continue
        a, pa, b, pb, cname = line[:5]
        if len(line) > 5:
            code.append(f'joint_turned("{a}", {vec(pa)}, "{b}", {vec(pb)}, "{cname}", {vec(line[5])});')
        else:
            code.append(f'joint("{a}", {vec(pa)}, "{b}", {vec(pb)}, "{cname}");')
    set_parm(wr, "snippet", "\n".join(code))
    display(wr)
    geo.setDisplayFlag(False)
    return wr.path()


# ---------------------------------------------------------------------------
# DOP network
# ---------------------------------------------------------------------------


def packed_pieces(dop, pieces_path, name="pieces"):
    """The pieces' DOP object: a Box3D Packed Object (the asset the build makes), which keeps the
    points' own attributes (density ...) and writes only the settings changed on it (none here), with
    Inherit Velocity on so the points' v is used. Without the asset: an RBD Packed Object, the same."""
    try:
        obj = dop.createNode("box3d_packedobject", name)
    except hou.OperationFailed:
        print("  (no Box3D Packed Object asset: using RBD Packed Object)")
        obj = create(dop, "rbdpackedobject", name)
    # Inherit Velocity is off on both: the scenes throw things with the points' own v (the building's ball).
    set_parm(obj, ["inheritvelocity", "usepointvelocity", "inheritvel"], 1)
    set_parm(obj, "soppath", pieces_path)
    return obj


def build_dops(name, pieces_path, joints=None, network_path=None, solver_parms=None, keep_velocity=False):
    """Box3D Packed Object -> (Constraint Network) -> Box3D Solver -> Output.
    joints: [(relationship node type, node name, {parm: value}[, comment])] for the constraint
    network; a parm key may be a tuple of candidate names, and the comment is shown on the node.
    keep_velocity: unused (the pieces always keep their points' v). Returns (dopnet, solver,
    {node name: Data Name})."""
    del keep_velocity
    dop = create(hou.node("/obj"), "dopnet", name)
    obj = packed_pieces(dop, pieces_path)
    last, data_names = obj, {}
    if joints:
        rels = []
        for type_name, node_name, parms, *comment in joints:
            rel = create(dop, type_name, node_name)
            for parm, value in parms.items():
                set_parm(rel, parm, value)
            if comment:
                rel.setComment(comment[0])
                rel.setGenericFlag(hou.nodeFlag.DisplayComment, True)
            dn = next((rel.parm(n) for n in ("dataname", "data_name", "DataName") if rel.parm(n)), None)
            data_names[node_name] = dn.eval() if dn is not None else None
            rels.append(rel)
        net = create(dop, "constraintnetwork", "constraint_network")
        net.setInput(0, obj)
        set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network_path or "")
        if len(rels) == 1:
            net.setInput(1, rels[0])
        else:
            m = create(dop, "merge", "relationships")
            for i, r in enumerate(rels):
                m.setInput(i, r)
            net.setInput(1, m)
        last = net
    solver = create(dop, "box3d_solver", "box3d_solver")
    solver.setInput(0, last)
    for parm, value in (solver_parms or {}).items():
        set_parm(solver, parm, value)
    out = create(dop, "output", "output")
    out.setInput(0, solver)
    display(out)
    return dop, solver, data_names


def tidy(frames, note, dop, note_size=(9, 6), shown=()):
    """Frame range, layout, hide the source geometry and add the sticky notes. shown: geometry objects to keep
    visible (a SOP solver's); the inner note goes in `dop`, the DOP network or that object."""
    try:
        hou.playbar.setFrameRange(1, frames)
        hou.playbar.setPlaybackRange(1, frames)
    except hou.Error:
        hou.hscript(f"tset 0 {(frames - 1) / hou.fps()}")
    hou.setFrame(1)
    obj = hou.node("/obj")
    for node in obj.children():
        if node.type().name() == "geo" and node not in shown:
            node.setDisplayFlag(False)  # the DOP network shows the simulated pieces
        if node.isNetwork() and node.children():
            node.layoutChildren()
    obj.layoutChildren()
    sticky = obj.createStickyNote("about_this_scene")
    sticky.setText(note.strip())
    sticky.setSize(hou.Vector2(*note_size))
    place_note_left(obj, sticky)
    inner = dop.createStickyNote("how_it_works")
    inner.setText("Box3D Packed Object reads the pieces from the geometry object (one packed primitive per "
                  "body). The Box3D Solver simulates them; its tabs hold the world settings "
                  "(substeps, gravity, sleeping, friction / bounce mixing).")
    inner.setSize(hou.Vector2(6, 2.5))
    place_note_left(dop, inner)


def fix_constraint_names(network_path, data_names, wanted):
    """Points the network's constraint_name values at the relationship nodes' Data Names."""
    mapping = {k: data_names.get(v) or k for k, v in wanted.items()}
    if all(k == v for k, v in mapping.items()):
        return
    wr = hou.node(network_path)
    code = wr.parm("snippet").eval()
    for k, v in mapping.items():
        # The constraint_name argument: last in joint(), before the orient in joint_turned().
        code = code.replace(f'"{k}");', f'"{v}");').replace(f'"{k}", vector4', f'"{v}", vector4')
    wr.parm("snippet").set(code)


def create_any(parent, type_names, name):
    """The first of type_names that exists (node type names vary between Houdini versions)."""
    for t in type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed:
            continue
    raise SetupError(f"none of the node types {type_names} exist in {parent.path()}")
