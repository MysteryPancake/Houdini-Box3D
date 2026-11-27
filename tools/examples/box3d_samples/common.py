"""What the Box3D sample ports share: the registry, the solver set up as Box3D's sample app runs, and
Bodies, the geometry object holding a sample's bodies.

A sample builder is a function decorated with @sample(category, name, source, frames). It takes the
Sample record, builds the scene into an empty file and returns (dopnet, frames), as make_examples.py's
scenes do. The plan, and which samples are ported: docs/agents/sample-port-plan.md.
"""

import math
import re

import hou

from scene_kit import (SetupError, build_dops, cooked_geometry, create, display, fix_constraint_names, network_sop,
                       num, set_parm, tidy, vec)

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

SAMPLES = {}  # "category/sample" (snake case) -> Sample, in registration order


def snake(text):
    """'Card House' -> 'card_house', 'HighMassRatio1' -> 'high_mass_ratio_1', 's&box mover' -> 'sbox_mover'."""
    text = text.replace("&", "")
    text = re.sub(r"(?<=[a-z])(?=[A-Z0-9])|(?<=[0-9])(?=[A-Z][a-z])", " ", text)
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


class Sample:
    def __init__(self, category, name, source, frames, build):
        self.category, self.name, self.source, self.frames, self.build = category, name, source, frames, build
        self.key = f"{snake(category)}/{snake(name)}"
        self.tag = snake(name)


def sample(category, name, source, frames=600):
    """Registers a builder for the Box3D sample `category` / `name` in samples/`source`. frames: how long the
    scene plays (at 60 fps, one frame per Box3D step)."""

    def register(build):
        s = Sample(category, name, source, frames, build)
        if s.key in SAMPLES:
            raise ValueError(f"two builders for {s.key}")
        SAMPLES[s.key] = s
        return build

    return register


# ---------------------------------------------------------------------------
# The world as Box3D's samples run it
# ---------------------------------------------------------------------------

FPS = 60  # Box3D's sample app steps at 60 Hz with 4 substeps (SampleContext in samples/sample.h)
GRAVITY = (0.0, -10.0, 0.0)  # b3DefaultWorldDef

# Solver parameters for Box3D's b3DefaultWorldDef and sample settings, where the solver's own defaults differ:
# contact hertz / damping ratio / push speed, friction mixing geometric mean (index 0) and restitution
# mixing maximum (index 2) as Box3D's default callbacks, and the true inertia of hulls (Bullet's bounding
# box inertia off).
BOX3D_SOLVER = {
    "substeps": 1,
    "solverpasses": 4,
    "contactstiffness": 30.0,
    "contactdamping": 10.0,
    "contactpushspeed": 3.0,
    "frictionmixing": 0,
    "restitutionmixing": 2,
    "boundsinertia": 0,
}

# Box3D's shape defaults (b3DefaultShapeDef / b3DefaultSurfaceMaterial), written on every body so the solver's
# own defaults never stand in for them.
BOX3D_MATERIAL = {"friction": 0.6, "bounce": 0.0, "rollingresistance": 0.0, "density": 1000.0}


def start_scene():
    """An empty scene at Box3D's step rate."""
    hou.setFps(FPS)


def box3d_world(solver, gravity=GRAVITY, **parms):
    """Sets the Box3D Solver as Box3D's sample app runs: one Box3D step of 4 substeps per frame at 60 fps,
    Box3D's gravity, contact tuning and mixing rules. parms: other solver parameters (a sample's own)."""
    for name, value in {**BOX3D_SOLVER, "gravity": tuple(gravity), **parms}.items():
        set_parm(solver, name, value)


# ---------------------------------------------------------------------------
# Bodies
# ---------------------------------------------------------------------------

# Every body's point carries these (besides the sample's material attributes): Copy to Points moves them onto
# the packed copy. orient is not among them: the copy is already turned by it.
BODY_ATTRIBUTES = ["name", "active", "bullet_georep", "bullet_autofit", "Cd"]


def vex_literal(value):
    """(VEX type prefix, literal) for an attribute default."""
    if isinstance(value, bool) or isinstance(value, int):
        return "i", str(int(value))
    if isinstance(value, float):
        return "f", num(value)
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return "v", vec(value)
    raise SetupError(f"no VEX type for the attribute default {value!r}")


VEX_FUNCTIONS = """\
// b3ComputeCosSin( radians ): Box3D's (cos, sin), a rational approximation normalised to length 1. Not exact (40
// degrees through b3MakeQuatFromAxisAngle is 40.149), so the samples' rotations use it to start where Box3D's do.
vector2 cossin(float radians) {
    float x = radians - 2.0 * PI * rint(radians / (2.0 * PI));  // b3UnwindAngle
    float pi2 = PI * PI;
    float c, s, y, y2;
    if (x < -0.5 * PI) { y = x + PI; y2 = y * y; c = -(pi2 - 4.0 * y2) / (pi2 + y2); }
    else if (x > 0.5 * PI) { y = x - PI; y2 = y * y; c = -(pi2 - 4.0 * y2) / (pi2 + y2); }
    else { y2 = x * x; c = (pi2 - 4.0 * y2) / (pi2 + y2); }
    if (x < 0.0) { y = x + PI; s = -16.0 * y * (PI - y) / (5.0 * pi2 - 4.0 * y * (PI - y)); }
    else s = 16.0 * x * (PI - x) / (5.0 * pi2 - 4.0 * x * (PI - x));
    float m = sqrt(s * s + c * c);
    vector2 cs = m > 0.0 ? set(c / m, s / m) : set(0.0, 0.0);
    return cs;
}
// b3MakeQuatFromAxisAngle( axis, angle ): a rotation of about angle radians about axis, as Box3D makes it.
vector4 axis_angle(vector axis; float angle) {
    vector2 cs = cossin(0.5 * angle);
    vector a = normalize(axis);
    vector4 q;
    q.x = cs[1] * a.x; q.y = cs[1] * a.y; q.z = cs[1] * a.z; q.w = cs[0];
    return q;
}
// Box3D capsules here run along the body's x; Houdini's run along y. This turns one built along y to x.
vector4 along_x(vector4 q) { return qmultiply(q, quaternion(radians(-90.0), {0, 0, 1})); }
"""


POLYGON_FUNCTIONS = """\
// face(name, points): one polygon of the body `name`, through the given positions.
void face(string nm; vector p[]) {
    int pr = addprim(0, "poly");
    foreach (vector q; p) addvertex(0, pr, addpoint(0, q));
    setprimattrib(0, "name", pr, nm);
}
// prism(name, points): the hull of a ring of n points and the ring above it (points 0..n-1, then n..2n-1).
void prism(string nm; vector p[]) {
    int n = len(p) / 2;
    vector bottom[], top[];
    for (int i = 0; i < n; i++) { push(bottom, p[n - 1 - i]); push(top, p[n + i]); }
    face(nm, bottom); face(nm, top);
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        face(nm, array(p[i], p[j], p[n + j], p[n + i]));
    }
}
addprimattrib(0, "name", "");
"""


class Bodies:
    """The geometry object holding a sample's rigid bodies. Each stream is one shape copied onto the points
    its VEX makes: body(p, q) adds a dynamic body, static_body(p, q) a static one, each returning its point for
    setpointattrib calls (friction, v, ...). Every point carries the same attributes (`attribs` with
    Box3D's material defaults), so the Merge never fills one in with 0."""

    def __init__(self, tag, attribs=None):
        self.tag = tag
        self.geo = create(hou.node("/obj"), "geo", f"{tag}_bodies")
        for child in self.geo.children():
            child.destroy()
        self.merge = create(self.geo, "merge", "all_bodies")
        self.attribs = {**BOX3D_MATERIAL, **(attribs or {})}
        self.streams = []  # (point wrangle or packed output, finish callback)
        self.count = 0

    # --- shapes (SOPs), in Box3D's terms -------------------------------------------------------

    def box(self, tag, hx, hy, hz):
        """b3MakeBoxHull( hx, hy, hz ): a box of half extents hx, hy, hz."""
        box = create(self.geo, "box", f"{tag}_shape")
        set_parm(box, "size", (2.0 * hx, 2.0 * hy, 2.0 * hz))
        return box, "box"

    def sphere(self, tag, radius):
        """A sphere of the radius about the body's origin: a Sphere primitive, which is a true sphere."""
        sph = create(self.geo, "sphere", f"{tag}_shape")
        set_parm(sph, "rad", (radius, radius, radius))
        return sph, "sphere"

    def capsule(self, tag, half_length, radius):
        """b3Capsule { (0, -h, 0), (0, h, 0), r }: along y, as Houdini fits capsules (along_x turns it to x)."""
        # The fit takes the radius from the points farthest from the axis and ends the caps at the bounds. A polymesh
        # with even rows has no ring on the equator: it is sized so the two rings nearest it are at the radius (cols a
        # multiple of 4 puts points on the axes), and its halves moved apart so the poles end at h + r.
        rows = 10
        rad = radius / math.cos(math.pi / (2 * (rows - 1)))
        shift = max(radius + half_length - rad, 0.0)
        sph = create(self.geo, "sphere", f"{tag}_ball")
        _set_type(sph, "polymesh")
        set_parm(sph, "rad", (rad, rad, rad))
        set_parm(sph, "rows", rows)
        set_parm(sph, "cols", 16)
        stretch = create(self.geo, "attribwrangle", f"{tag}_shape")
        stretch.setInput(0, sph)
        set_parm(stretch, "snippet", f"// A capsule of radius {num(radius)}: the ball's halves pulled apart along y, "
                                     f"the poles at {num(radius + half_length)}.\n"
                                     f"@P.y = sign(@P.y) * min(abs(@P.y) + {num(shift)}, "
                                     f"{num(radius + half_length)});")
        return stretch, "capsule"

    def cylinder(self, tag, height, radius, y_offset=0.0, sides=12):
        """b3CreateCylinder( height, radius, yOffset, sides ): a prism of `sides` sides along y, from y_offset
        to y_offset + height in the body's frame, collided as its convex hull."""
        tube = create(self.geo, "tube", f"{tag}_shape")
        _set_type(tube)
        set_parm(tube, "rad", (radius, radius))
        set_parm(tube, "height", height)
        set_parm(tube, "t", (0.0, y_offset + 0.5 * height, 0.0))
        set_parm(tube, "rows", 2)
        set_parm(tube, "cols", sides)
        set_parm(tube, "cap", 1)
        return tube, "convexhull"

    # --- streams ---------------------------------------------------------------------------------

    def _check_defaults(self, defaults):
        """A stream's defaults must be sample attributes: one only some bodies had would be 0 on the rest."""
        for name in defaults:
            if name not in self.attribs:
                raise SetupError(f"{self.tag}: stream default {name!r} is not one of the sample's attributes "
                                 f"{sorted(self.attribs)}: add it to Bodies(attribs=...)")

    def _declarations(self, defaults):
        self._check_defaults(defaults)
        return "\n".join(f'addpointattrib(0, "{name}", {vex_literal(value)[1]});'
                         for name, value in {**self.attribs, **defaults}.items())

    def stream(self, tag, shape, vex, cd=(0.7, 0.7, 0.7), note="", **defaults):
        """Copies `shape` (from box(), sphere(), ...) onto the bodies the VEX makes. defaults: this stream's
        values of the sample's attributes (friction=0.7, ...)."""
        sop, rep = shape
        pts = create(self.geo, "attribwrangle", f"{tag}_points")
        set_parm(pts, "class", 0)  # detail: the code adds the points
        prelude = f"""\
#define TAG "{tag}"
#define REP "{rep}"
// One Box3D body: a point at p turned by q, with this stream's shape (bullet_georep REP).
int add_body(vector p; vector4 q; int act) {{
    int pt = addpoint(0, p);
    setpointattrib(0, "name", pt, sprintf("%s%d", TAG, pt));
    setpointattrib(0, "orient", pt, q);
    setpointattrib(0, "Cd", pt, {vec(cd)});
    setpointattrib(0, "active", pt, act);
    setpointattrib(0, "bullet_georep", pt, REP);
    return pt;
}}
int body(vector p; vector4 q) {{ return add_body(p, q, 1); }}   // dynamic
int static_body(vector p; vector4 q) {{ return add_body(p, q, 0); }}  // static (active 0)
{VEX_FUNCTIONS}
addpointattrib(0, "name", "");
vector4 identity = {{0, 0, 0, 1}};  // a typed variable: set() with four floats could also be a matrix2
addpointattrib(0, "orient", identity);
setattribtypeinfo(0, "point", "orient", "quaternion");
addpointattrib(0, "Cd", {vec(cd)});
addpointattrib(0, "active", 1);
addpointattrib(0, "bullet_georep", REP);
addpointattrib(0, "bullet_autofit", 1);
addpointattrib(0, "v", {{0, 0, 0}});
addpointattrib(0, "w", {{0, 0, 0}});
{self._declarations(defaults)}
"""
        set_parm(pts, "snippet", (f"// {note}\n" if note else "") + prelude + "\n" + vex.strip() + "\n")
        copy = create(self.geo, "copytopoints", f"{tag}_copy")
        copy.setInput(0, sop)
        copy.setInput(1, pts)
        set_parm(copy, ["pack", "packgeo"], 1)
        set_parm(copy, "targetattribs", 1)
        set_parm(copy, "applyattribs1", " ".join(BODY_ATTRIBUTES + ["v", "w"] + list(self.attribs)))
        self.merge.setInput(len(self.merge.inputs()), copy)
        return copy

    def polygons(self, tag, vex, cd=(0.7, 0.7, 0.7), active=1, note="", **defaults):
        """Bodies whose shapes are polygons the VEX builds (b3CreateHull of a point set): face(name, pts)
        adds a polygon to the body `name`. Each name is packed into one body, collided as its convex hull."""
        build = create(self.geo, "attribwrangle", f"{tag}_polygons")
        set_parm(build, "class", 0)
        set_parm(build, "snippet", (f"// {note}\n" if note else "") + POLYGON_FUNCTIONS + vex.strip() + "\n")
        pack = create(self.geo, "pack", f"{tag}_pack")
        pack.setInput(0, build)
        set_parm(pack, ["packbyname"], 1)
        set_parm(pack, ["nameattribute"], "name")
        set_parm(pack, ["transfer_attributes", "transferattributes", "transferattribs", "transfer"], "name")
        attrs = create(self.geo, "attribwrangle", f"{tag}_attributes")
        attrs.setInput(0, pack)
        lines = ["// The body attributes every stream has, on each packed body's point (at its centroid).",
                 "int prims[] = pointprims(0, @ptnum);",
                 's@name = prim(0, "name", prims[0]);',
                 f"i@active = {int(active)};",
                 's@bullet_georep = "convexhull";',
                 "i@bullet_autofit = 1;",
                 f"v@Cd = {vec(cd)};",
                 "v@v = {0, 0, 0};",
                 "v@w = {0, 0, 0};"]
        self._check_defaults(defaults)
        for name, value in {**self.attribs, **defaults}.items():
            t, lit = vex_literal(value)
            lines.append(f"{t}@{name} = {lit};")
        set_parm(attrs, "snippet", "\n".join(lines))
        self.merge.setInput(len(self.merge.inputs()), attrs)
        return attrs

    def ground(self, extent, note="", **defaults):
        """Sample::AddGroundBox( extent ): a static box of half extents (extent, 1, extent) whose top is y = 0."""
        return self.stream("ground", self.box("ground", extent, 1.0, extent),
                           "static_body({0, -1, 0}, identity);", cd=(0.35, 0.36, 0.38),
                           note=note or f"Sample::AddGroundBox( {num(extent)} ): the static ground, its top at y = 0",
                           **defaults)

    def finish(self):
        """Shows the merged bodies; returns the SOP path. Checks every body has a name."""
        null = create(self.geo, "null", "OUT_bodies")
        null.setInput(0, self.merge)
        display(null)
        geo = cooked_geometry(null)
        self.count = len(geo.points())
        unnamed = sum(1 for pt in geo.points() if not pt.attribValue("name"))
        if unnamed:
            raise SetupError(f"{null.path()}: {unnamed} bodies have no name (an attribute lost when packing?)")
        return null.path()


def _set_type(node, token="poly"):
    """Sets a Sphere or Tube SOP's Primitive Type (poly: Polygon, polymesh: Polygon Mesh)."""
    p = node.parm("type")
    items = list(p.menuItems())
    p.set(items.index(token) if p.parmTemplate().type() == hou.parmTemplateType.Menu else token)


# ---------------------------------------------------------------------------
# Finishing a sample
# ---------------------------------------------------------------------------


def motion_locks(names_and_points, constraint_name="motionlock"):
    """Network lines holding each body (name, position) to the world: Sample::LockMotion's motion lock joint,
    its frame the world axes."""
    return [("", p, n, p, constraint_name) for n, p in names_and_points]


def finish(s, bodies, about, ported="", try_="", joints=None, lines=None, solver=None, gravity=GRAVITY,
           note_size=(10, 6)):
    """The DOP network around the bodies, the solver set as Box3D runs, the frame range and the notes.
    joints: [(relationship type, node name, {parm: value}, constraint_name)] for the network `lines`
    (network_sop's (name_a, pos_a, name_b, pos_b, constraint_name[, orient]))."""
    path = bodies.finish()
    network, joint_nodes, wanted = None, None, {}
    if lines:
        network = network_sop(f"{s.tag}_network", lines)
        joint_nodes = []
        for type_name, node_name, parms, cname in joints:
            joint_nodes.append((type_name, node_name, parms))
            wanted[cname] = node_name
    dop, solver_node, data_names = build_dops(f"{s.tag}_sim", path, joints=joint_nodes, network_path=network)
    if network:
        fix_constraint_names(network, data_names, wanted)
    box3d_world(solver_node, gravity=gravity, **(solver or {}))
    text = f"{s.name.upper()}\nBox3D sample {s.category} / {s.name} (extern/box3d/samples/{s.source})\n\n" \
           f"{about.strip()}"
    if ported:
        text += f"\n\nPort: {ported.strip()}"
    if try_:
        text += f"\n\nTry: {try_.strip()}"
    tidy(s.frames, text, dop, note_size=note_size)
    inner = dop.findStickyNote("how_it_works")  # tidy's note in the DOP network (a sticky note is not a node)
    if inner is not None:
        inner.setText(inner.text() + "\n\nThe solver runs as Box3D's sample app: 60 fps with Substeps 1 and "
                      "Constraint Iterations 4 (one Box3D step of 4 substeps a frame), gravity -10, Box3D's "
                      "contact settings and friction / bounce mixing, Bounding Box Inertia off.")
        inner.setSize(hou.Vector2(6, 3.5))
    return dop, s.frames


def box_impulse(half, density, impulse, offset):
    """(v, w) of an unrotated box of half extents `half` after b3Body_ApplyLinearImpulse( impulse ) at
    `offset` from its centre: what the sample does before its first step, as the body's starting velocity."""
    hx, hy, hz = half
    mass = density * 8.0 * hx * hy * hz
    inertia = (mass * (hy * hy + hz * hz) / 3.0, mass * (hx * hx + hz * hz) / 3.0, mass * (hx * hx + hy * hy) / 3.0)
    rx, ry, rz = offset
    jx, jy, jz = impulse
    torque = (ry * jz - rz * jy, rz * jx - rx * jz, rx * jy - ry * jx)
    return tuple(j / mass for j in impulse), tuple(t / i for t, i in zip(torque, inertia))

