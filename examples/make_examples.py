"""Builds example scenes that show what the Box3D solver can do, for new users to open and play.

Run from the repository root with the plugin built and installed (build.bat):

    hython examples/make_examples.py              # every scene
    hython examples/make_examples.py dominoes     # some scenes (names below)
    hython examples/make_examples.py --check      # also simulate each scene and print timings

On Windows, examples/make_examples.bat finds Houdini and runs this.

Each scene is saved to examples/scenes/<number>_<name>.hip. Open one and press play: a sticky
note in /obj says what it shows and what to try. Every scene is built from ordinary SOPs (Box,
Copy to Points, Attribute Wrangle) and one RBD Packed Object, so it doubles as a template.

Scenes:
  shapes          the collision shapes: box, sphere, capsule and convex hull (Show Collision Shapes on)
  many_bodies     10,000 boxes poured into a bin; Rows / Columns / Layers on the geometry change the count
  dominoes        a spiral of dominoes knocked over by the first one
  bridge_chain    constraints: a hinged rope bridge (revolute) and a swinging chain (spherical)
  wrecking_ball   a ball on a rigid rope (distance joint) swinging into a brick wall
  paddle_mixer    a motor: a paddle driven by a revolute motor churning a tank of balls
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
    sticky.setPosition(obj.children()[0].position() + hou.Vector2(-11, 0) if obj.children() else hou.Vector2(-11, 0))
    inner = dop.createStickyNote("how_it_works")
    inner.setText("RBD Packed Object reads the pieces from the geometry object (one packed primitive per "
                  "body). The Box3D Solver simulates them; its tabs hold the world settings "
                  "(substeps, gravity, sleeping, friction / bounce mixing).")
    inner.setSize(hou.Vector2(6, 2.5))
    inner.setPosition(dop.children()[0].position() + hou.Vector2(-8, 0))


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
        for i in range(6):
            for j in range(6):
                for layer in range(2):
                    x, z = (i - 2.5) * 1.6, (j - 2.5) * 1.6
                    y = 1.0 + (k * 2 + layer) * 1.4
                    spin = quat_about((0.6, 0.0, 0.8), 0.3 * (i + j + layer))  # tumble a little
                    calls.append(mk((x + 0.1 * layer, y, z), f"{tag}{i}_{j}_{layer}", spin, cd, georep=rep))
        p.stream(tag, shape, "\n".join(calls), f"{tag}: bullet_georep '{rep}', fitted to the shape (bullet_autofit 1)")
    path = p.finish()
    dop, _, _ = build_dops("shapes_sim", path, solver_parms={"showguide": 1})
    note = f"""SHAPES: {p.count - 1} bodies in five shapes fall onto a static floor.

Each piece's bullet_georep point attribute picks its collision shape, as with Bullet:
box, sphere, capsule, convexhull (the dodecahedra and teapots). bullet_autofit 1 fits it
to the piece. The default for a piece without bullet_georep is convexhull.

The Box3D Solver's Guides tab has Show Collision Shapes on: the cyan outlines are what
the solver collides (dimmed when a body is asleep).

Try: change the shapes in shape_pieces, or add a Box3D Data node after 'pieces' in the
DOP network to set friction, bounce or the default shape for pieces without bullet_georep."""
    tidy(144, note, dop)
    return dop, 144


def scene_many_bodies():
    p = Pieces("many_pieces")
    ground(p, 60.0)
    for tag, size, pos in (("wall_left", (1, 4, 46), (-23.5, 2, 0)), ("wall_right", (1, 4, 46), (23.5, 2, 0)),
                           ("wall_back", (46, 4, 1), (0, 2, -23.5)), ("wall_front", (46, 4, 1), (0, 2, 23.5))):
        p.boxes(tag, size, [mk(pos, tag, cd=(0.45, 0.47, 0.5), active=0)], "a static wall of the bin")
    geo = p.geo
    # A "Count" tab on the object, read by the boxes' wrangle with chi("../rows") etc.
    folder = hou.FolderParmTemplate("count", "Count")
    for n, label, default in (("rows", "Rows (x)", 25), ("columns", "Columns (z)", 25), ("layers", "Layers (y)", 16)):
        folder.addParmTemplate(hou.IntParmTemplate(n, label, 1, default_value=(default,), min=1, max=40))
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

Try: select many_pieces and change Rows / Columns / Layers (up to 34 x 34 fits the bin)
and see how the time per frame grows. Windows > Performance Monitor shows where the time
goes, stage by stage. On the Box3D Solver: Substeps trades speed for stiffer stacks;
Allow Sleeping is on Box3D Data. With Bullet the same scene is usually much slower."""
    tidy(240, note, dop)
    return dop, 240


def scene_dominoes():
    p = Pieces("domino_pieces")
    ground(p, 60.0)
    count, spacing, turn_gap, r0 = 260, 0.55, 1.6, 3.0
    b = turn_gap / (2 * math.pi)
    theta, calls, first = 0.0, [], None
    for i in range(count):
        r = r0 + b * theta
        pos = (r * math.cos(theta), 0.5005, r * math.sin(theta))
        # tangent of the spiral (direction of travel)
        tx, tz = b * math.cos(theta) - r * math.sin(theta), b * math.sin(theta) + r * math.cos(theta)
        tl = math.hypot(tx, tz)
        tx, tz = tx / tl, tz / tl
        q = quat_about((0, 1, 0), math.atan2(-tz, tx))  # the domino's thin side (x) along the path
        calls.append(mk(pos, f"domino{i}", q, rainbow(i / count * 1.5)))
        if first is None:
            first = (tx, tz)
        theta += spacing / math.sqrt(r * r + b * b)
    p.boxes("dominoes", (0.12, 1.0, 0.5), calls, "dominoes along a spiral, thin side along the path")
    path = p.finish(
        "// The first domino gets a push along the path (RBD Packed Object keeps v: Inherit Velocity on).\n"
        f'v@v = s@name == "domino0" ? {vec((first[0] * 2.0, 0, first[1] * 2.0))} : {{0, 0, 0}};')
    dop, _, _ = build_dops("dominoes_sim", path, keep_velocity=True)
    note = f"""DOMINOES: {count} dominoes on a spiral; the first is pushed into the next.

Thin, tall boxes standing on their ends test contact stability and friction. Each
domino is a packed box whose rotation comes from the orient point attribute.

Try: change spacing in the dominoes_points wrangle's code (this script's 'spacing'),
or set friction on RBD Packed Object (it writes friction onto every piece) and watch
the dominoes slip instead of topple."""
    tidy(480, note, dop)
    return dop, 480


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
    crates = [mk((-5 + 10 * ((i * 0.618) % 1), 8 + i * 0.9, -0.4 + 0.8 * ((i * 0.37) % 1)), f"crate{i}",
                 quat_about((0, 1, 0), i), rainbow(i / 12)) for i in range(12)]
    p.boxes("crates", (0.6, 0.6, 0.6), crates, "crates dropped onto the bridge")

    # Chain: links joined end to end by spherical joints, pinned to the world at one end,
    # starting straight out so it swings down into a small tower.
    pivot, pitch, n = (-6.0, 13.0, -6.0), 0.5, 24
    links = [mk((pivot[0] + (i + 0.5) * pitch, pivot[1], pivot[2]), f"link{i}", cd=(0.8, 0.8, 0.85))
             for i in range(n)]
    p.boxes("links", (0.44, 0.12, 0.12), links, "chain links")
    lines.append(("", pivot, "link0", pivot, "spherical"))
    for i in range(1, n):
        at = (pivot[0] + i * pitch, pivot[1], pivot[2])
        lines.append((f"link{i - 1}", at, f"link{i}", at, "spherical"))
    tower = [mk((-7.5, 0.5 + k * 1.001, pivot[2]), f"tower{k}", cd=rainbow(0.6 + k * 0.1)) for k in range(3)]
    p.boxes("tower", (1, 1, 1), tower, "a small tower in the chain's path")

    path = p.finish()
    network = network_sop("bridge_chain_network", lines)
    dop, _, names_used = build_dops("bridge_chain_sim", path, network_path=network,
                                    joints=[("box3d_revoluteconrel", "hinges", {}),
                                            ("box3d_sphericalconrel", "ball_joints", {})])
    fix_constraint_names(network, names_used, {"revolute": "hinges", "spherical": "ball_joints"})
    note = f"""BRIDGE AND CHAIN: {p.count - 1} bodies joined by {len(lines)} constraints.

A constraint network (bridge_chain_network) is a polyline per joint: each end names the
piece it holds and sits where the joint is. An empty name pins the piece to the world.
The polyline's constraint_name picks the relationship node in the DOP network:
  - the bridge planks use 'Box3D Revolute' (hinges turning about the joint's z axis);
  - the chain links use 'Box3D Spherical' (ball joints).

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
    bricks, rows, per_row = [], 16, 14
    for row in range(rows):
        offset = 0.5 if row % 2 else 0.0
        count = per_row - (1 if row % 2 else 0)
        for i in range(count):
            z = (i - (per_row - 1) / 2) * 1.002 + offset
            y = 0.25 + row * 0.501
            shade = 0.85 + 0.15 * ((i * 7 + row * 3) % 5) / 4
            bricks.append(mk((1.5, y, z), f"brick{row}_{i}", cd=(0.7 * shade, 0.32 * shade, 0.22 * shade)))
    p.boxes("bricks", (0.5, 0.5, 1.0), bricks, "a running-bond brick wall")
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
    ground(p)
    hub = (0.0, 2.8, 0.0)
    tank = [("tank_left", (0.3, 6, 2.2), (-3.35, 3, 0)), ("tank_right", (0.3, 6, 2.2), (3.35, 3, 0)),
            ("tank_back", (7.0, 6, 0.3), (0, 3, -1.25)), ("tank_front", (7.0, 6, 0.3), (0, 3, 1.25))]
    for tag, size, pos in tank:
        p.boxes(tag, size, [mk(pos, tag, cd=(0.55, 0.65, 0.75), active=0)], "a static wall of the tank")
    p.boxes("paddle", (5.2, 0.35, 1.4), [mk(hub, "paddle", cd=(0.9, 0.5, 0.15))],
            "the paddle, hinged to the world at its middle")
    vex = """\
// Balls in a lattice above the paddle.
int n = 0;
for (int k = 0; k < 7; k++) for (int j = 0; j < 4; j++) for (int i = 0; i < 15; i++) {
    vector p = set(-2.8 + i * 0.4, 3.4 + k * 0.4, -0.6 + j * 0.4) + 0.02 * (vector(rand(n)) - 0.5);
    mk(p, sprintf("ball%d", n), identity, hsvtorgb(set(rand(n + 0.3), 0.6, 0.95)), 1, 1000.0, "sphere");
    n++;
}
"""
    p.stream("balls", p.shape_sphere("balls", 0.17, freq=2), vex, "the balls")
    path = p.finish(
        "// See-through front wall (packed primitives draw with their point's Alpha).\n"
        'f@Alpha = s@name == "tank_front" ? 0.15 : 1.0;')
    network = network_sop("mixer_network", [("", hub, "paddle", hub, "revolute")])
    dop, _, names_used = build_dops(
        "paddle_mixer_sim", path, network_path=network,
        joints=[("box3d_revoluteconrel", "motor_hinge",
                 {"enable_motor": 1, "motor_speed": 120.0, "max_motor_torque": 1.0e6})])
    fix_constraint_names(network, names_used, {"revolute": "motor_hinge"})
    note = f"""PADDLE MIXER: a motor-driven paddle churning {p.count - 6} balls.

The paddle is hinged to the world by a 'Box3D Revolute' joint (axis along z, out of
the tank) with Enable Motor on: Motor Speed 120 degrees/second, and Max Motor Torque
high enough to push through the balls. The front wall is drawn see-through.

Try: on motor_hinge, lower Max Motor Torque until the balls stall the paddle, reverse
Motor Speed, or turn on Enable Limit to make it rock between two angles."""
    tidy(480, note, dop)
    return dop, 480


SCENES = {
    "shapes": scene_shapes,
    "many_bodies": scene_many_bodies,
    "dominoes": scene_dominoes,
    "bridge_chain": scene_bridge_chain,
    "wrecking_ball": scene_wrecking_ball,
    "paddle_mixer": scene_paddle_mixer,
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
