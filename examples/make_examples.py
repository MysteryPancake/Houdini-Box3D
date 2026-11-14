"""Builds example scenes that show what the Box3D solver can do, for new users to open and play.

Run from the repository root with the plugin built and installed (build.bat):

    hython examples/make_examples.py              # every scene
    hython examples/make_examples.py building     # some scenes (names below)
    hython examples/make_examples.py --check      # also simulate each scene and print timings

On Windows, make_examples.bat (in the repository root) finds Houdini and runs this.

Each scene is saved to examples/scenes/<number>_<name>.hip. Open one and press play: a sticky
note in /obj says what it shows and what to try. Every scene is built from ordinary SOPs (Box,
Copy to Points, Attribute Wrangle; the building also RBD Material Fracture) and one RBD Packed
Object, so it doubles as a template.

Scenes:
  shapes          the collision shapes: box, sphere, capsule and convex hull
  many_bodies     40,000 boxes poured into a bin; Rows / Columns / Layers on the geometry change the count
  building        breaking constraints: a concrete building (RBD Material Fracture) glued together, hit by a ball
  bridge_chain    constraints: a hinged rope bridge (revolute) and a swinging chain (spherical)
  wrecking_ball   a ball on a rigid rope (distance joint) swinging into a brick wall
  paddle_mixer    a motor: a paddle driven by a revolute motor churning a tank of balls
  forces          a Fan Force DOP and a POP Force (in a Multiple Solver) pushing a field of boxes
  terrain         a height-field collider: bodies tumbling down hills made with a HeightField SOP
"""

import argparse
import colorsys
import math
import os
import sys
import time

sys.dont_write_bytecode = True

import hou  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "scenes")


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


class SetupError(Exception):
    """A node type or parameter was not found; the message says which."""


def create(parent, type_name, name=None):
    try:
        return parent.createNode(type_name, name) if name else parent.createNode(type_name)
    except hou.OperationFailed as e:
        raise SetupError(f"cannot create a '{type_name}' node in {parent.path()}: {e}")


def set_parm(node, names, value):
    """Sets the first of `names` that exists (a tuple value sets a parameter tuple)."""
    for n in [names] if isinstance(names, str) else names:
        if isinstance(value, (tuple, list)):
            pt = node.parmTuple(n)
            if pt is not None and len(pt) == len(value):
                pt.deleteAllKeyframes()
                pt.set(value)
                return
        else:
            p = node.parm(n)
            if p is not None:
                p.deleteAllKeyframes()
                p.set(value)
                return
    available = ", ".join(sorted({pt.name() for pt in node.parmTuples()}))
    raise SetupError(f"{node.path()}: no parameter {' / '.join(names)}. It has: {available}")


def set_menu(node, name, label_part):
    """Sets a menu to the first item whose label contains label_part."""
    p = node.parm(name)
    if p is None:
        raise SetupError(f"{node.path()}: no menu '{name}'")
    for i, label in enumerate(p.menuLabels()):
        if label_part.lower() in label.lower():
            p.set(i)
            return
    raise SetupError(f"{node.path()}: menu '{name}' has no item like '{label_part}': {list(p.menuLabels())}")


def display(node):
    node.setDisplayFlag(True)
    if hasattr(node, "setRenderFlag"):
        node.setRenderFlag(True)


def quat_about(axis, angle):
    """Houdini quaternion (x, y, z, w) turning `angle` radians about a unit axis."""
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def rainbow(t):
    return colorsys.hsv_to_rgb(t % 1.0, 0.65, 0.95)


def num(x):
    """A VEX float literal (always with a decimal point, so VEX never reads it as an int)."""
    text = f"{float(x):.6g}"
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


COPY_ATTRIBUTES = """\
// The body attributes from the template point this packed copy was made from.
s@name = point(1, "name", @ptnum);
i@active = point(1, "active", @ptnum);
f@density = point(1, "density", @ptnum);
s@bullet_georep = point(1, "bullet_georep", @ptnum);
i@bullet_autofit = point(1, "bullet_autofit", @ptnum);
v@Cd = point(1, "Cd", @ptnum);
"""


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
        set_menu(sph, "type", "Polygon")  # a primitive sphere is a single point: nothing to fit
        set_parm(sph, "rad", (radius, radius, radius))
        set_parm(sph, ["freq"], freq)
        return sph

    def shape_tube(self, tag, radius, height):
        tube = create(self.geo, "tube", f"{tag}_shape")
        set_menu(tube, "type", "Polygon")
        set_parm(tube, "rad", (radius, radius))
        set_parm(tube, "height", height)
        set_parm(tube, "cap", 1)
        set_parm(tube, "cols", 16)
        return tube

    def shape_platonic(self, tag, kind, radius):
        pl = create(self.geo, "platonic", f"{tag}_shape")
        set_menu(pl, "type", kind)
        set_parm(pl, ["rad", "radius"], radius)
        return pl

    def stream(self, tag, shape, vex_body, note=""):
        """Copies `shape` onto the points that vex_body makes (mk(...) calls or loops)."""
        pts = create(self.geo, "attribwrangle", f"{tag}_points")
        set_parm(pts, "class", 0)  # run once (detail): the code adds the points
        set_parm(pts, "snippet", (f"// {note}\n" if note else "") + PRELUDE + "\n" + vex_body)
        copy = create(self.geo, "copytopoints", f"{tag}_copy")
        copy.setInput(0, shape)
        copy.setInput(1, pts)
        set_parm(copy, ["pack", "packgeo"], 1)
        # Copy to Points leaves some point attributes behind when it packs (active was lost, so
        # static pieces fell), so copy them explicitly: copy i is made from template point i.
        attrs = create(self.geo, "attribwrangle", f"{tag}_attributes")
        attrs.setInput(0, copy)
        attrs.setInput(1, pts)
        set_parm(attrs, "snippet", COPY_ATTRIBUTES)
        self.merge.setInput(len(self.merge.inputs()), attrs)
        return attrs

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
    lines: (name_a, pos_a, name_b, pos_b, constraint_name). An empty name pins to the world.
    Both points sit where the joint is; constraint_name picks the relationship node."""
    geo = create(hou.node("/obj"), "geo", name)
    for child in geo.children():
        child.destroy()
    wr = create(geo, "attribwrangle", "constraint_lines")
    set_parm(wr, "class", 0)
    code = ['// One polyline per joint. Each end names the piece it holds ("" = the world) and',
            '// sits at the joint; constraint_name matches a relationship node\'s Data Name.',
            "void joint(string a; vector pa; string b; vector pb; string cname) {",
            "    int i = addpoint(0, pa), j = addpoint(0, pb);",
            '    setpointattrib(0, "name", i, a); setpointattrib(0, "name", j, b);',
            '    int pr = addprim(0, "polyline", i, j);',
            '    setprimattrib(0, "constraint_name", pr, cname);',
            "}",
            'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", "");']
    for a, pa, b, pb, cname in lines:
        code.append(f'joint("{a}", {vec(pa)}, "{b}", {vec(pb)}, "{cname}");')
    set_parm(wr, "snippet", "\n".join(code))
    display(wr)
    geo.setDisplayFlag(False)
    return wr.path()


# ---------------------------------------------------------------------------
# DOP network
# ---------------------------------------------------------------------------


def build_dops(name, pieces_path, joints=None, network_path=None, solver_parms=None, keep_velocity=False):
    """RBD Packed Object -> (Constraint Network) -> Box3D Solver -> Output.
    joints: [(relationship node type, node name, {parm: value})] for the constraint network.
    Returns (dopnet, solver, {node name: Data Name})."""
    dop = create(hou.node("/obj"), "dopnet", name)
    obj = create(dop, "rbdpackedobject", "pieces")
    set_parm(obj, "soppath", pieces_path)
    if keep_velocity:  # keep the points' v instead of RBD Packed Object's initial velocity
        set_parm(obj, ["inheritvelocity", "usepointvelocity", "inheritvel"], 1)
    last, data_names = obj, {}
    if joints:
        rels = []
        for type_name, node_name, parms in joints:
            rel = create(dop, type_name, node_name)
            for parm, value in parms.items():
                set_parm(rel, parm, value)
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


def place_note_left(net, note, gap=1.0):
    """Puts a sticky note to the left of every node in net, its top level with the highest node.
    Uses the bounding box of all the nodes, so it never lands on one (children()[0] is not the
    leftmost node)."""
    nodes = net.children()
    if not nodes:
        note.setPosition(hou.Vector2(-gap - note.size()[0], 0))
        return
    left = min(n.position()[0] for n in nodes)
    top = max(n.position()[1] + n.size()[1] for n in nodes)
    note.setPosition(hou.Vector2(left - gap - note.size()[0], top - note.size()[1]))


def tidy(frames, note, dop):
    """Frame range, layout, hide the source geometry and add the sticky notes."""
    try:
        hou.playbar.setFrameRange(1, frames)
        hou.playbar.setPlaybackRange(1, frames)
    except hou.Error:
        hou.hscript(f"tset 0 {(frames - 1) / hou.fps()}")
    hou.setFrame(1)
    obj = hou.node("/obj")
    for node in obj.children():
        if node.type().name() == "geo":
            node.setDisplayFlag(False)  # the DOP network shows the simulated pieces
        if node.isNetwork() and node.children():
            node.layoutChildren()
    obj.layoutChildren()
    sticky = obj.createStickyNote("about_this_scene")
    sticky.setText(note.strip())
    sticky.setSize(hou.Vector2(9, 6))
    place_note_left(obj, sticky)
    inner = dop.createStickyNote("how_it_works")
    inner.setText("RBD Packed Object reads the pieces from the geometry object (one packed primitive per "
                  "body). The Box3D Solver simulates them; its tabs hold the world settings "
                  "(substeps, gravity, sleeping, friction / bounce mixing).")
    inner.setSize(hou.Vector2(6, 2.5))
    place_note_left(dop, inner)


# ---------------------------------------------------------------------------
# Scenes. Each builds into an empty scene and returns (dopnet, frames).
# ---------------------------------------------------------------------------


def scene_shapes():
    p = Pieces("shape_pieces")
    ground(p)
    kinds = [("boxes", "box"), ("spheres", "sphere"), ("capsules", "capsule"),
             ("dodecahedra", "convexhull"), ("teapots", "convexhull")]
    shapes = [p.shape_box("boxes", (0.8, 0.8, 0.8)), p.shape_sphere("spheres", 0.45),
              p.shape_tube("capsules", 0.25, 1.0), p.shape_platonic("dodecahedra", "Dodecahedron", 0.5),
              p.shape_platonic("teapots", "Teapot", 0.45)]
    for k, ((tag, rep), shape) in enumerate(zip(kinds, shapes)):
        cd = rainbow(k / len(kinds))
        calls = []
        for i in range(12):
            for j in range(12):
                for layer in range(3):
                    x, z = (i - 5.5) * 1.6, (j - 5.5) * 1.6
                    y = 1.0 + (k * 3 + layer) * 1.4
                    spin = quat_about((0.6, 0.0, 0.8), 0.3 * (i + j + layer))  # tumble a little
                    calls.append(mk((x + 0.1 * layer, y, z), f"{tag}{i}_{j}_{layer}", spin, cd, georep=rep))
        p.stream(tag, shape, "\n".join(calls), f"{tag}: bullet_georep '{rep}', fitted to the shape (bullet_autofit 1)")
    path = p.finish()
    dop, _, _ = build_dops("shapes_sim", path)
    note = f"""SHAPES: {p.count - 1} bodies in five shapes fall onto a static floor.

Each piece's bullet_georep point attribute picks its collision shape, as with Bullet:
box, sphere, capsule, convexhull (the dodecahedra and teapots). bullet_autofit 1 fits it
to the piece. The default for a piece without bullet_georep is convexhull.

To see what the solver collides, turn on Show Collision Shapes in the Box3D Solver's
Guide Options: cyan outlines, dimmed when a body is asleep.

Try: change the shapes in shape_pieces, or add a Box3D Data node after 'pieces' in the
DOP network to set friction, bounce or the default shape for pieces without bullet_georep."""
    tidy(240, note, dop)
    return dop, 240


def scene_many_bodies():
    p = Pieces("many_pieces")
    ground(p, 80.0)
    for tag, size, pos in (("wall_left", (1, 6, 62), (-30.5, 3, 0)), ("wall_right", (1, 6, 62), (30.5, 3, 0)),
                           ("wall_back", (62, 6, 1), (0, 3, -30.5)), ("wall_front", (62, 6, 1), (0, 3, 30.5))):
        p.boxes(tag, size, [mk(pos, tag, cd=(0.45, 0.47, 0.5), active=0)], "a static wall of the bin")
    geo = p.geo
    # A "Count" tab on the object, read by the boxes' wrangle with chi("../rows") etc.
    folder = hou.FolderParmTemplate("count", "Count")
    for n, label, default in (("rows", "Rows (x)", 40), ("columns", "Columns (z)", 40), ("layers", "Layers (y)", 25)):
        folder.addParmTemplate(hou.IntParmTemplate(n, label, 1, default_value=(default,), min=1, max=80))
    group = geo.parmTemplateGroup()
    group.append(folder)
    geo.setParmTemplateGroup(group)
    vex = """\
// A lattice of Rows x Columns x Layers boxes, from the spare parameters on this object.
int nx = chi("../rows"), nz = chi("../columns"), ny = chi("../layers");
float pitch = 1.1;
int n = 0;
for (int k = 0; k < ny; k++) for (int j = 0; j < nz; j++) for (int i = 0; i < nx; i++) {
    vector jitter = 0.1 * (vector(rand(set(i, j, k) + 0.5)) - 0.5);
    vector p = set((i - (nx - 1) * 0.5) * pitch, 1.0 + k * pitch, (j - (nz - 1) * 0.5) * pitch) + jitter;
    vector4 q = quaternion(radians(20.0 * rand(n + 0.1)), normalize(vector(rand(n + 0.2)) - 0.5));
    vector cd = hsvtorgb(set(0.55 + 0.35 * k / max(ny - 1, 1), 0.6, 0.95));
    mk(p, sprintf("box%d", n), q, cd, 1, 1000.0, "box");
    n++;
}
"""
    p.stream("boxes", p.shape_box("boxes", (0.8, 0.8, 0.8)), vex, "the boxes")
    path = p.finish()
    dop, _, _ = build_dops("many_bodies_sim", path)
    note = f"""MANY BODIES: {p.count - 5} boxes poured into a bin.

Box3D is built for scenes like this: it runs the contact solver on all your cores and
puts resting bodies to sleep. The first frame builds the bodies; after that each frame
only steps the world.

Try: select many_pieces and change Rows / Columns / Layers (up to 50 x 50 fits the bin)
and see how the time per frame grows. Windows > Performance Monitor shows where the time
goes, stage by stage. On the Box3D Solver: Substeps trades speed for stiffer stacks;
Allow Sleeping is on Box3D Data. With Bullet the same scene is usually much slower."""
    tidy(360, note, dop)
    return dop, 360


def scene_building():
    p = Pieces("building_pieces")
    ground(p, 80.0)
    geo = p.geo

    # --- The building, from Box SOPs only: per floor a slab and the walls, split into panels.
    # Parts touch but never overlap (overlapping convex hulls would push apart on frame 1).
    floors, width, storey, slab, wall, bays = 8, 12.0, 3.0, 0.3, 0.4, 3
    half = width / 2
    parts = []  # (centre, size)
    for f in range(floors):
        base = f * storey
        parts.append(((0.0, base + slab / 2, 0.0), (width, slab, width)))
        h = storey - slab
        y = base + slab + h / 2
        for b in range(bays):  # walls along x (front / back), full width, split into bays
            length = width / bays
            x = -half + length * (b + 0.5)
            for z in (-half + wall / 2, half - wall / 2):
                parts.append(((x, y, z), (length, h, wall)))
        inner = width - 2 * wall
        for b in range(bays):  # walls along z (sides), between the front and back walls
            length = inner / bays
            z = -half + wall + length * (b + 0.5)
            for x in (-half + wall / 2, half - wall / 2):
                parts.append(((x, y, z), (wall, h, length)))
    parts.append(((0.0, floors * storey + slab / 2, 0.0), (width, slab, width)))  # the roof
    unit = create(geo, "box", "building_unit_box")  # a 1 m polygon box, scaled per part
    set_parm(unit, "size", (1.0, 1.0, 1.0))
    layout = create(geo, "attribwrangle", "building_parts")
    set_parm(layout, "class", 0)
    lines = ["// One point per building part (slab or wall panel): its centre, and its size as 'scale',",
             "// which Copy to Points applies to the unit box."]
    for c, s in parts:
        lines.append(f'{{ int pt = addpoint(0, {vec(c)}); setpointattrib(0, "scale", pt, {vec(s)}); }}')
    set_parm(layout, "snippet", "\n".join(lines))
    copies = create(geo, "copytopoints", "building_copy")
    copies.setInput(0, unit)
    copies.setInput(1, layout)
    set_parm(copies, ["pack", "packgeo"], 0)
    named = create(geo, "attribwrangle", "name_parts")
    named.setInput(0, copies)
    set_parm(named, "class", 1)  # primitives
    set_parm(named, "snippet", "// Each copy is a 6-face box: name every part, so each is fractured on its own.\n"
                               's@name = sprintf("part%d", @primnum / 6);')
    fracture = create(geo, "rbdmaterialfracture", "fracture")
    fracture.setInput(0, named)
    # Concrete (the default material) breaks every named part into chunks.
    pack = create(geo, "pack", "pack_pieces")
    pack.setInput(0, fracture, 0)
    set_parm(pack, ["packbyname"], 1)
    set_parm(pack, ["nameattribute"], "name")
    # Pack drops every attribute it is not told to transfer, the piece names included.
    set_parm(pack, ["transfer_attributes", "transferattributes", "transferattribs", "transfer"], "name")
    body = create(geo, "attribwrangle", "piece_attributes")
    body.setInput(0, pack)
    set_parm(body, "snippet", """\
// The body attributes every stream has: the packed piece's name on its point, dynamic, concrete.
int prims[] = pointprims(0, @ptnum);
s@name = prim(0, "name", prims[0]);
i@active = 1;
f@density = 2400.0;
s@bullet_georep = "convexhull";
float shade = 0.62 + 0.12 * rand(@ptnum + 0.5);
v@Cd = set(shade, shade * 0.98, shade * 0.93);
""")
    p.merge.setInput(len(p.merge.inputs()), body)

    # --- Glue between touching pieces, across cracks and between parts.
    connect = create(geo, "connectadjacentpieces", "connect_pieces")
    connect.setInput(0, fracture, 0)
    glue_lines = create(geo, "attribwrangle", "glue_name")
    glue_lines.setInput(0, connect)
    set_parm(glue_lines, "class", 1)  # primitives
    constraints = create(geo, "null", "OUT_constraints")
    constraints.setInput(0, glue_lines)

    # --- A heavy ball thrown at the third floor.
    radius, speed, target_y, start_x = 1.5, 30.0, 7.5, -32.0
    flight = (half - start_x) / speed
    start_y = target_y + 0.5 * 9.80665 * flight * flight  # falls to the third floor by the time it arrives
    p.stream("ball", p.shape_sphere("ball", radius, freq=4),
             mk((start_x, start_y, 0.0), "ball", cd=(0.2, 0.2, 0.22), density=8000.0, georep="sphere"),
             "the ball: steel density, so about 110 tonnes")
    path = p.finish(
        "// The ball is thrown at the building (RBD Packed Object keeps v: Inherit Velocity on).\n"
        f'v@v = s@name == "ball" ? {vec((speed, 0, 0))} : {{0, 0, 0}};')

    strength = 5.0e4
    dop, _, names_used = build_dops(
        "building_sim", path, network_path=constraints.path(), keep_velocity=True,
        joints=[("glueconrel", "glue", {"strength": strength})])
    glue_name = names_used.get("glue") or "Glue"
    set_parm(glue_lines, "snippet", "// Every line is a glue bond: constraint_name picks the Glue relationship by "
                                    f'its Data Name.\ns@constraint_name = "{glue_name}";')
    pieces = p.count - 2  # the ground and the ball
    bonds = len(cooked_geometry(constraints).prims())
    note = f"""BUILDING: an {floors}-storey concrete building in {pieces} pieces, held by {bonds}
glue bonds, hit by a 110-tonne ball.

building_pieces makes the floors and wall panels from Box SOPs, RBD Material Fracture
(Concrete) breaks each one into chunks, and Connect Adjacent Pieces joins every pair of
touching chunks with a line: one glue bond each.

Breaking: the Glue Constraint Relationship's Strength ({strength:g}) sets how hard a bond is
to break. Box3D makes each bond a weld that breaks once the force on it passes
Strength / timestep in one step (Bullet's half-life and propagation are not used). Broken
bonds go into the constraint geometry's 'broken' group, so a SOP Solver can react to them.

Try: lower Strength on 'glue' for a weaker building (too low and it falls under its own
weight); raise it until the ball only dents it; change the ball's speed or height in
building_pieces; on 'fracture', change the number of chunks."""
    tidy(240, note, dop)
    return dop, 240


def bridge_hinges(span=14.0, links=16, length=1.0, height=5.0):
    """Hinge points of a sagging bridge: links+1 points from x = -span/2 to span/2, each
    `length` apart, on a parabola deep enough to take up the slack."""

    def walk(sag):
        pts, x = [(-span / 2, height)], -span / 2
        curve = lambda u: height - sag * (1 - (2 * u / span) ** 2)  # noqa: E731
        for _ in range(links):
            lo, hi = x, x + length
            for _ in range(60):  # the next x one chord length away
                mid = (lo + hi) / 2
                if math.hypot(mid - x, curve(mid) - curve(x)) < length:
                    lo = mid
                else:
                    hi = mid
            x = (lo + hi) / 2
            pts.append((x, curve(x)))
        return pts

    lo, hi = 0.0, span
    for _ in range(60):  # the sag at which the last hinge lands on the far post
        mid = (lo + hi) / 2
        if walk(mid)[-1][0] > span / 2:
            lo = mid
        else:
            hi = mid
    return walk((lo + hi) / 2)


def scene_bridge_chain():
    p = Pieces("bridge_chain_pieces")
    ground(p)
    lines = []
    # Rope bridge: planks hinged to each other and to two static posts (revolute joints turn
    # about the joint's z axis, which runs across the bridge here).
    hinges = bridge_hinges()
    height = hinges[0][1]
    p.boxes("posts", (1.0, height, 2.0),
            [mk((-7.5, height / 2, 0), "post_left", cd=(0.5, 0.35, 0.25), active=0),
             mk((7.5, height / 2, 0), "post_right", cd=(0.5, 0.35, 0.25), active=0)], "static posts")
    planks = []
    for i in range(len(hinges) - 1):
        (x0, y0), (x1, y1) = hinges[i], hinges[i + 1]
        q = quat_about((0, 0, 1), math.atan2(y1 - y0, x1 - x0))
        planks.append(mk(((x0 + x1) / 2, (y0 + y1) / 2, 0), f"plank{i}", q, (0.75, 0.6, 0.4)))
    p.boxes("planks", (0.92, 0.1, 1.6), planks, "bridge planks, laid along the sagging curve")
    names = ["post_left"] + [f"plank{i}" for i in range(len(hinges) - 1)] + ["post_right"]
    for i, (x, y) in enumerate(hinges):
        lines.append((names[i], (x, y, 0), names[i + 1], (x, y, 0), "revolute"))
    # 20 layers of 12 crates (6 across, 2 deep), each layer 0.8 higher and shifted a little
    ncrates = 240
    crates = [mk((-5 + 2 * (i % 6) + 0.1 * ((i * 0.618) % 1), 8 + (i // 12) * 0.8, -0.35 + 0.7 * ((i // 6) % 2)
                  + 0.05 * ((i * 0.37) % 1)), f"crate{i}", quat_about((0, 1, 0), i), rainbow(i / ncrates))
              for i in range(ncrates)]
    p.boxes("crates", (0.6, 0.6, 0.6), crates, "crates dropped onto the bridge")

    # Chain: links joined end to end by spherical joints, pinned to the world at one end,
    # starting straight out so it swings down into a small tower.
    chains, n, pitch = 4, 24, 0.5
    links, tower = [], []
    for c in range(chains):
        pivot = (-6.0, 13.0, -6.0 - 3.0 * c)
        links += [mk((pivot[0] + (i + 0.5) * pitch, pivot[1], pivot[2]), f"link{c}_{i}", cd=(0.8, 0.8, 0.85))
                  for i in range(n)]
        lines.append(("", pivot, f"link{c}_0", pivot, "spherical"))
        for i in range(1, n):
            at = (pivot[0] + i * pitch, pivot[1], pivot[2])
            lines.append((f"link{c}_{i - 1}", at, f"link{c}_{i}", at, "spherical"))
        tower += [mk((-7.5, 0.5 + k * 1.001, pivot[2]), f"tower{c}_{k}", cd=rainbow(0.6 + k * 0.1)) for k in range(3)]
    p.boxes("links", (0.44, 0.12, 0.12), links, "chain links, four chains side by side")
    p.boxes("tower", (1, 1, 1), tower, "a small tower in each chain's path")

    path = p.finish()
    network = network_sop("bridge_chain_network", lines)
    dop, _, names_used = build_dops("bridge_chain_sim", path, network_path=network,
                                    joints=[("box3d_revoluteconrel", "hinges", {}),
                                            ("box3d_sphericalconrel", "ball_joints", {})])
    fix_constraint_names(network, names_used, {"revolute": "hinges", "spherical": "ball_joints"})
    note = f"""BRIDGE AND CHAINS: {p.count - 1} bodies, {len(lines)} of them joined by constraints.

A constraint network (bridge_chain_network) is a polyline per joint: each end names the
piece it holds and sits where the joint is. An empty name pins the piece to the world.
The polyline's constraint_name picks the relationship node in the DOP network:
  - the bridge planks use 'Box3D Revolute' (hinges turning about the joint's z axis);
  - the chain links use 'Box3D Spherical' (ball joints), in four chains side by side.
The crates dropped onto the bridge are free bodies.

Try: on the hinges node, turn on Enable Spring or limits; turn on Breakable on the
ball joints with a low Break Force and the chain snaps when it hits the tower."""
    tidy(240, note, dop)
    return dop, 240


def fix_constraint_names(network_path, data_names, wanted):
    """Points the network's constraint_name values at the relationship nodes' Data Names."""
    mapping = {k: data_names.get(v) or k for k, v in wanted.items()}
    if all(k == v for k, v in mapping.items()):
        return
    wr = hou.node(network_path)
    code = wr.parm("snippet").eval()
    for k, v in mapping.items():
        code = code.replace(f'"{k}");', f'"{v}");')
    wr.parm("snippet").set(code)


def scene_wrecking_ball():
    p = Pieces("wrecking_pieces")
    ground(p)
    bricks, rows, per_row, depth = [], 30, 24, 3
    for d in range(depth):
        for row in range(rows):
            offset = 0.5 if row % 2 else 0.0
            count = per_row - (1 if row % 2 else 0)
            for i in range(count):
                z = (i - (per_row - 1) / 2) * 1.002 + offset
                y = 0.25 + row * 0.501
                shade = 0.85 + 0.15 * ((i * 7 + row * 3 + d * 2) % 5) / 4
                bricks.append(mk((1.5 + d * 0.501, y, z), f"brick{d}_{row}_{i}",
                                 cd=(0.7 * shade, 0.32 * shade, 0.22 * shade)))
    p.boxes("bricks", (0.5, 0.5, 1.0), bricks, "a running-bond brick wall, three bricks thick")
    pivot, ball_start = (0.0, 14.0, 0.0), (-9.0, 14.0, 0.0)
    p.stream("ball", p.shape_sphere("ball", 1.0, freq=4),
             mk(ball_start, "ball", cd=(0.2, 0.2, 0.22), density=8000.0, georep="sphere"),
             "the wrecking ball: steel density, so about 33 tonnes")
    path = p.finish()
    network = network_sop("wrecking_network", [("", pivot, "ball", ball_start, "distance")])
    dop, _, names_used = build_dops("wrecking_ball_sim", path, network_path=network,
                                    joints=[("box3d_distanceconrel", "rope", {})])
    fix_constraint_names(network, names_used, {"distance": "rope"})
    note = f"""WRECKING BALL: a {len(bricks)}-brick wall and a heavy ball on a rigid rope.

The rope is one 'Box3D Distance' joint between the world (empty name) and the ball:
it keeps the starting distance between its two ends (9 m) and lets the ball swing freely.
The ball's density point attribute (8000) makes it much heavier than the bricks.

Try: on the rope node, turn on Enable Spring for a bungee; change the ball's start
height in wrecking_pieces; add a Box3D Weld relationship and weld the bricks together."""
    tidy(240, note, dop)
    return dop, 240


def scene_paddle_mixer():
    p = Pieces("mixer_pieces")
    ground(p, 60.0)
    # Tank inside: 2 * half_x wide (x), 2 * half_z deep (z), `height` tall. The paddle turns about z
    # through the hub, so it must clear the floor and the side walls: hub height > half its length.
    half_x, half_z, height, wall = 8.0, 5.75, 22.0, 0.3
    paddle = (2 * half_x - 3.0, 0.6, 2 * half_z - 0.5)  # length (x), thickness, depth (z)
    hub = (0.0, paddle[0] / 2 + 0.3, 0.0)
    outer_x, outer_z = half_x + wall, half_z + wall
    tank = [("tank_left", (wall, height, 2 * half_z), (-half_x - wall / 2, height / 2, 0)),
            ("tank_right", (wall, height, 2 * half_z), (half_x + wall / 2, height / 2, 0)),
            ("tank_back", (2 * outer_x, height, wall), (0, height / 2, -half_z - wall / 2)),
            ("tank_front", (2 * outer_x, height, wall), (0, height / 2, half_z + wall / 2))]
    for tag, size, pos in tank:
        p.boxes(tag, size, [mk(pos, tag, cd=(0.55, 0.65, 0.75), active=0)], "a static wall of the tank")
    p.boxes("paddle", paddle, [mk(hub, "paddle", cd=(0.9, 0.5, 0.15))],
            "the paddle, hinged to the world at its middle")
    radius, pitch = 0.17, 0.4
    nx = int((2 * half_x - 0.4) / pitch)
    nz = int((2 * half_z - 0.4) / pitch)
    y0 = hub[1] + paddle[1] / 2 + 0.3
    ny = int((height - 1.0 - y0) / pitch)
    vex = f"""\
// Balls in a lattice above the paddle, filling the tank.
int nx = {nx}, nz = {nz}, ny = {ny};
int n = 0;
for (int k = 0; k < ny; k++) for (int j = 0; j < nz; j++) for (int i = 0; i < nx; i++) {{
    vector p = set((i - (nx - 1) * 0.5) * {num(pitch)}, {num(y0)} + k * {num(pitch)}, (j - (nz - 1) * 0.5) * {num(pitch)});
    p += 0.02 * (vector(rand(n)) - 0.5);
    mk(p, sprintf("ball%d", n), identity, hsvtorgb(set(rand(n + 0.3), 0.6, 0.95)), 1, 1000.0, "sphere");
    n++;
}}
"""
    p.stream("balls", p.shape_sphere("balls", radius, freq=2), vex, "the balls")
    path = p.finish(
        "// See-through front wall (packed primitives draw with their point's Alpha).\n"
        'f@Alpha = s@name == "tank_front" ? 0.15 : 1.0;')
    network = network_sop("mixer_network", [("", hub, "paddle", hub, "revolute")])
    speed = 60.0
    dop, _, names_used = build_dops(
        "paddle_mixer_sim", path, network_path=network,
        joints=[("box3d_revoluteconrel", "motor_hinge",
                 {"enable_motor": 1, "motor_speed": speed, "max_motor_torque": 1.0e9})])
    fix_constraint_names(network, names_used, {"revolute": "motor_hinge"})
    note = f"""PADDLE MIXER: a motor-driven paddle churning {p.count - 6} balls.

The paddle ({paddle[0]:g} m long) is hinged to the world by a 'Box3D Revolute' joint (axis
along z, out of the tank) with Enable Motor on: Motor Speed {speed:g} degrees/second, and Max
Motor Torque high enough to push through hundreds of tonnes of balls. The front wall is
drawn see-through.

Try: on motor_hinge, lower Max Motor Torque until the balls stall the paddle, reverse
Motor Speed, or turn on Enable Limit to make it rock between two angles. In this script,
half_x / half_z / height size the tank and the ball count follows."""
    tidy(480, note, dop)
    return dop, 480


def create_any(parent, type_names, name):
    """The first of type_names that exists (node type names vary between Houdini versions)."""
    for t in type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed:
            continue
    raise SetupError(f"none of the node types {type_names} exist in {parent.path()}")


def set_by_label(node, labels, value):
    """Sets the first parameter whose label contains one of `labels` (case-insensitive): internal
    names are not documented for every node. A tuple value sets a whole parameter tuple."""
    tuples = node.parmTuples()
    for want in labels:
        for pt in tuples:
            if want.lower() in pt.parmTemplate().label().lower():
                if isinstance(value, (tuple, list)):
                    if len(pt) == len(value):
                        pt.deleteAllKeyframes()
                        pt.set(value)
                        return
                elif len(pt) == 1:
                    pt[0].deleteAllKeyframes()
                    pt[0].set(value)
                    return
    have = ", ".join(sorted({pt.parmTemplate().label() for pt in tuples}))
    raise SetupError(f"{node.path()}: no parameter labelled like {labels}. Its labels: {have}")


def scene_forces():
    p = Pieces("forces_pieces")
    ground(p, 80.0)
    # Light boxes (density 50, so about 1.4 kg each) on the ground, in a square field.
    rows, size, pitch = 40, 0.3, 0.55
    vex = f"""\
// A {rows} x {rows} field of light boxes resting on the ground, coloured by position.
int n = 0;
for (int j = 0; j < {rows}; j++) for (int i = 0; i < {rows}; i++) {{
    vector p = set((i - ({rows} - 1) * 0.5) * {num(pitch)}, {num(size / 2 + 0.001)}, (j - ({rows} - 1) * 0.5) * {num(pitch)});
    vector cd = hsvtorgb(set(0.08 + 0.5 * j / ({rows} - 1.0), 0.7, 0.95));
    mk(p, sprintf("box%d", n), identity, cd, 1, 50.0, "box");
    n++;
}}
"""
    p.stream("boxes", p.shape_box("boxes", (size, size, size)), vex, "the boxes")
    path = p.finish()

    dop = create(hou.node("/obj"), "dopnet", "forces_sim")
    obj = create(dop, "rbdpackedobject", "pieces")
    set_parm(obj, "soppath", path)

    # DOP force: a Fan Force DOP attaches force data to the objects; the Box3D Solver evaluates it
    # at every body's centre of mass, every step.
    half = (rows - 1) * pitch / 2
    fan = create_any(dop, ["fan", "fanforce"], "fan_force")
    fan.setInput(0, obj)
    set_by_label(fan, ["Position", "Center"], (-half - 4.0, 0.5, 0.0))
    set_by_label(fan, ["Direction"], (1.0, 0.35, 0.0))

    # POP force: a POP Force adds turbulence to each piece's `force` point attribute. In a Multiple
    # Solver it runs first; the Box3D Solver then applies `force` for one step and clears it.
    popforce = create(dop, "popforce", "pop_turbulence")
    set_by_label(popforce, ["Amplitude"], 25.0)
    set_by_label(popforce, ["Swirl Size"], 2.0)
    solver = create(dop, "box3d_solver", "box3d_solver")
    set_parm(solver, "forceswake", 1)  # resting boxes wake up when a force reaches them
    multi = create(dop, "multisolver", "pop_then_box3d")
    multi.setInput(0, fan)
    multi.setInput(1, popforce)
    multi.setInput(2, solver)
    out = create(dop, "output", "output")
    out.setInput(0, multi)
    display(out)

    note = f"""FORCES: {p.count - 1} light boxes pushed by a DOP force and a POP force.

fan_force (Fan Force DOP): blows from the left edge of the field. Force DOPs attach force
data to the objects; the Box3D Solver evaluates each at every body's centre of mass, every
step (Use Force DOPs, on the solver's Forces tab).

pop_turbulence (POP Force): adds noise to each box's 'force' point attribute. It runs before
the Box3D Solver in the Multiple Solver; the solver applies 'force' (and 'torque') for one
step and then clears it, so POP forces don't pile up.

Forces Wake Bodies is on, so boxes that have fallen asleep wake when a force reaches them.

Try: change the fan's Direction or strength; raise the POP Force's Amplitude, or set its
Force for a steady push; add a Wind, Vortex or Gravity Force DOP after fan_force; turn off
Use Force DOPs on the solver to see only the POP force."""
    tidy(240, note, dop)
    return dop, 240

def scene_terrain():
    # The terrain: a HeightField SOP whose height layer a Volume Wrangle shapes into hills
    # sloping down towards +x.
    terrain = create(hou.node("/obj"), "geo", "terrain")
    for child in terrain.children():
        child.destroy()
    hf = create(terrain, "heightfield", "heightfield")
    set_parm(hf, "size", (60.0, 60.0))
    set_parm(hf, "gridspacing", 0.4)
    hills = create(terrain, "volumewrangle", "hills")
    hills.setInput(0, hf)
    set_parm(hills, "snippet", """\
// Rolling hills on a slope that falls towards +x (the height layer, in metres).
@height = 3.0 * sin(@P.x * 0.16) * cos(@P.z * 0.13)
        + 1.2 * sin(@P.x * 0.45 + @P.z * 0.3)
        - 0.12 * @P.x;""")
    out_terrain = create(terrain, "null", "OUT_terrain")
    out_terrain.setInput(0, hills)
    display(out_terrain)
    cooked_geometry(out_terrain)

    p = Pieces("terrain_pieces")
    kinds = [("boxes", "box"), ("spheres", "sphere"), ("capsules", "capsule")]
    shapes = [p.shape_box("boxes", (0.8, 0.8, 0.8)), p.shape_sphere("spheres", 0.45),
              p.shape_tube("capsules", 0.25, 1.0)]
    for k, ((tag, rep), shape) in enumerate(zip(kinds, shapes)):
        cd = rainbow(k / len(kinds))
        calls = []
        for i in range(10):
            for j in range(10):
                x, z = -18.0 + (i - 4.5) * 2.0, (j - 4.5) * 2.0
                y = 12.0 + k * 2.5 + 0.3 * ((i + j) % 3)
                spin = quat_about((0.6, 0.0, 0.8), 0.4 * (i + 2 * j + k))
                calls.append(mk((x + 0.6 * k, y, z), f"{tag}{i}_{j}", spin, cd, georep=rep))
        p.stream(tag, shape, "\n".join(calls), f"{tag}: bullet_georep '{rep}'")
    path = p.finish()

    dop = create(hou.node("/obj"), "dopnet", "terrain_sim")
    obj = create(dop, "rbdpackedobject", "pieces")
    set_parm(obj, "soppath", path)
    # A Static Object whose geometry is a height field: the Box3D Solver collides the
    # pieces with a Box3D height field made from its height layer.
    static = create(dop, "staticobject", "terrain")
    set_parm(static, "soppath", out_terrain.path())
    merge = create(dop, "merge", "merge")
    merge.setInput(0, obj)
    merge.setInput(1, static)
    solver = create(dop, "box3d_solver", "box3d_solver")
    solver.setInput(0, merge)
    out = create(dop, "output", "output")
    out.setInput(0, solver)
    display(out)

    note = f"""TERRAIN: {p.count} bodies tumble down hills made with a HeightField SOP.

The Static Object 'terrain' reads /obj/terrain, whose geometry is a height field (a 2D
volume named 'height'). The Box3D Solver turns it into a Box3D height field: the same
grid and heights, no polygons needed. A Static Object with polygons instead becomes a
triangle mesh.

The object's transform poses the height field; moving it with an animated OBJ Path moves
the collider without rebuilding it. Shear falls back to the field's triangles. To change
the terrain while the simulation runs, turn on Use Deforming Geometry on 'terrain'.

Try: change the hills wrangle, or add a HeightField Noise or HeightField Erode SOP before
OUT_terrain; add a Box3D Data node after 'terrain' in the DOP network to set its friction
or bounce."""
    tidy(240, note, dop)
    return dop, 240


SCENES = {
    "shapes": scene_shapes,
    "many_bodies": scene_many_bodies,
    "building": scene_building,
    "bridge_chain": scene_bridge_chain,
    "wrecking_ball": scene_wrecking_ball,
    "paddle_mixer": scene_paddle_mixer,
    "forces": scene_forces,
    "terrain": scene_terrain,
}


def check(dop, frames):
    """Simulates the scene and reports timing and solver messages."""
    start = time.time()
    hou.setFrame(1)
    dop.simulation().objects()
    first = time.time() - start
    start = time.time()
    for f in range(2, frames + 1):
        hou.setFrame(f)
        dop.simulation().objects()
    per = (time.time() - start) / max(frames - 1, 1)
    msgs = []
    for node in dop.children():
        for get in ("errors", "warnings"):
            try:
                msgs += getattr(node, get)()
            except hou.Error:
                pass
    hou.setFrame(1)
    return f"first frame {first:.2f} s, then {per * 1000:.1f} ms per frame" + (
        "; messages: " + " | ".join(msgs) if msgs else "")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", nargs="*", help="scene names (default: all)")
    ap.add_argument("--check", action="store_true", help="simulate each scene and print timings")
    args = ap.parse_args()
    unknown = [s for s in args.scenes if s not in SCENES]
    if unknown:
        ap.error(f"unknown scene(s) {unknown}; choose from {list(SCENES)}")
    os.makedirs(OUT_DIR, exist_ok=True)
    failed = False
    for number, (name, build) in enumerate(SCENES.items(), 1):
        if args.scenes and name not in args.scenes:
            continue
        hou.hipFile.clear(suppress_save_prompt=True)
        path = os.path.join(OUT_DIR, f"{number:02d}_{name}.hip")
        try:
            dop, frames = build()
        except SetupError as e:
            # Saved anyway, so the broken node can be opened and looked at.
            bad = path.replace(".hip", "_FAILED.hip")
            hou.hipFile.save(bad.replace("\\", "/"))
            print(f"  FAILED {name} (saved as {os.path.relpath(bad)}): {e}")
            failed = True
            continue
        hou.hipFile.save(path.replace("\\", "/"))
        line = f"  saved  {os.path.relpath(path)}"
        if args.check:
            line += "  (" + check(dop, frames) + ")"
        print(line)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
