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


class Every(int):
    """An int attribute that is 1 on frames 1, 1 + n, 1 + 2n, ... and 0 between: `rbdbullet_emit` for bodies a sample
    fires again every n steps (finish's Emit RBDs). Every(0) is never: a body made once."""

    def vex(self):
        return f"(int(@Frame) - 1) % {int(self)} == 0" if self else "0"


def vex_literal(value):
    """(VEX type prefix, literal) for an attribute default."""
    if isinstance(value, Every):
        return "i", value.vex()
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
// points(positions): adds one point per position; their point numbers, in order, for face() and tri(). A body's faces
// are built on its points, so they share them as one connected piece (the Box3D Solver SOP re-assembles faces that
// do not into a body of many hulls), and a mesh's triangles share vertices and edges as Box3D's do.
function int[] points(vector p[]) {
    int ids[];
    foreach (vector q; p) push(ids, addpoint(0, q));
    return ids;
}
// face(name, pts): one polygon of the body `name` through the given point numbers.
void face(string nm; int pts[]) {
    int pr = addprim(0, "poly", pts);
    setprimattrib(0, "name", pr, nm);
}
// prism(name, positions): the hull of a ring of n points and the ring above it (points 0..n-1, then n..2n-1).
void prism(string nm; vector p[]) {
    int q[] = points(p);
    int n = len(p) / 2;
    int bottom[], top[];
    for (int i = 0; i < n; i++) { push(bottom, q[n - 1 - i]); push(top, q[n + i]); }
    face(nm, bottom); face(nm, top);
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        face(nm, array(q[i], q[j], q[n + j], q[n + i]));
    }
}
addprimattrib(0, "name", "");
"""

# Static triangle meshes (b3CreateMesh and its shape builders in src/mesh.c), for Bodies.mesh. Each builder adds its
# vertices once, in Box3D's order, and its triangles on them. A triangle is given in Box3D's order: counter-clockwise
# seen from the side it collides on. Houdini's front faces are clockwise, and the solver flips Houdini's triangles back,
# so tri() adds (a, c, b).
MESH_FUNCTIONS = """\
// tri(name, a, b, c): one triangle of the mesh `name` through three point numbers, in Box3D's winding.
void tri(string nm; int a; int b; int c) { face(nm, array(a, c, b)); }
// welded(positions, tolerance): b3MeshDef's weldVertices for a triangle soup (three positions a triangle): one point
// per position, those within the tolerance of an earlier one sharing its point. The point number of each corner.
function int[] welded(vector p[]; float tol) {
    vector seen[];
    int ids[], corner[];
    foreach (vector q; p) {
        int found = -1;
        for (int i = 0; i < len(seen) && found < 0; ++i)
            if (distance(seen[i], q) <= tol) found = ids[i];
        if (found < 0) { found = addpoint(0, q); push(seen, q); push(ids, found); }
        push(corner, found);
    }
    return corner;
}
// b3CreateWaveMesh( xCount, zCount, cellWidth, amplitude, rowFrequency, columnFrequency ), its vertices times s
// (b3CreateMeshShape's scale) plus o. Amplitude 0 is b3CreateGridMesh.
void wave_mesh(string nm; int xc; int zc; float w; float amp; float rowf; float colf; vector s; vector o) {
    vector p[];
    float omega_z = 2.0 * PI * rowf * w, omega_x = 2.0 * PI * colf * w;
    float x = -0.5 * w * xc;
    for (int ix = 0; ix <= xc; ++ix) {
        float row_height = cossin(omega_x * ix)[1];
        float z = -0.5 * w * zc;
        for (int iz = 0; iz <= zc; ++iz) {
            float column_height = cossin(omega_z * iz)[1];
            push(p, set(x, amp * row_height * column_height, z) * s + o);
            z += w;
        }
        x += w;
    }
    int v[] = points(p);
    for (int ix = 0; ix < xc; ++ix) for (int iz = 0; iz < zc; ++iz) {
        int i1 = iz + (zc + 1) * ix, i2 = i1 + 1, i3 = i2 + zc + 1, i4 = i3 - 1;
        tri(nm, v[i1], v[i2], v[i3]);
        tri(nm, v[i3], v[i4], v[i1]);
    }
}
void grid_mesh(string nm; int xc; int zc; float w; vector s; vector o) { wave_mesh(nm, xc, zc, w, 0, 0, 0, s, o); }
// b3CreateBoxMesh( center, extent ), outward faces; inward 1 is b3CreateHollowBoxMesh (the same triangles reversed).
// The mesh is then turned by q and moved by o: a static body's rotation and position.
void box_mesh(string nm; vector center; vector e; int inward; vector4 q; vector o) {
    vector p[] = array(set(e.x, e.y, e.z), set(-e.x, e.y, e.z), set(-e.x, -e.y, e.z), set(e.x, -e.y, e.z),
                       set(e.x, e.y, -e.z), set(-e.x, e.y, -e.z), set(-e.x, -e.y, -e.z), set(e.x, -e.y, -e.z));
    foreach (int i; vector c; p) p[i] = qrotate(q, c + center) + o;
    int v[] = points(p);
    int ix[] = array(0, 1, 3, 1, 2, 3,  0, 4, 1, 1, 4, 5,  0, 3, 7, 4, 0, 7,
                     4, 7, 5, 6, 5, 7,  1, 5, 2, 6, 2, 5,  3, 2, 7, 6, 7, 2);
    for (int t = 0; t < 12; ++t) {
        int a = v[ix[3 * t]], b = v[ix[3 * t + 1]], c = v[ix[3 * t + 2]];
        if (inward) tri(nm, a, c, b); else tri(nm, a, b, c);
    }
}
// b3CreateTorusMesh( radialResolution, tubularResolution, radius, thickness ): about the z axis, moved by o.
void torus_mesh(string nm; int radial; int tubular; float radius; float thickness; vector o) {
    vector p[];
    for (int r = 0; r < radial; ++r) for (int t = 0; t < tubular; ++t) {
        vector2 cu = cossin(float(t) / tubular * 2.0 * PI), cv = cossin(float(r) / radial * 2.0 * PI);
        push(p, set((radius + thickness * cv[0]) * cu[0], (radius + thickness * cv[0]) * cu[1], thickness * cv[1]) + o);
    }
    int v[] = points(p);
    for (int r1 = 0; r1 < radial; ++r1) {
        int r2 = (r1 + 1) % radial;
        for (int t1 = 0; t1 < tubular; ++t1) {
            int t2 = (t1 + 1) % tubular;
            int i1 = r1 * tubular + t1, i2 = r1 * tubular + t2, i3 = r2 * tubular + t2, i4 = r2 * tubular + t1;
            tri(nm, v[i1], v[i2], v[i3]);
            tri(nm, v[i3], v[i4], v[i1]);
        }
    }
}
// hull_points(name, positions): the convex hull of a point set (b3CreateHull), as triangles: every triple of points
// with all the others on one side. For small sets (tens of points) in general position. Only the points on the hull
// are added.
void hull_points(string nm; vector p[]) {
    int n = len(p);
    int v[];
    resize(v, n);
    foreach (int i; int x; v) v[i] = -1;
    for (int i = 0; i < n; ++i) for (int j = i + 1; j < n; ++j) for (int k = j + 1; k < n; ++k) {
        vector nrm = cross(p[j] - p[i], p[k] - p[i]);
        if (length(nrm) < 1e-9) continue;
        int above = 0, below = 0;
        for (int m = 0; m < n; ++m) {
            float d = dot(nrm, p[m] - p[i]);
            if (d > 1e-7) above++; else if (d < -1e-7) below++;
        }
        if (above && below) continue;
        int corners[] = array(i, j, k);
        foreach (int c; corners) if (v[c] < 0) v[c] = addpoint(0, p[c]);
        if (above) tri(nm, v[i], v[k], v[j]); else tri(nm, v[i], v[j], v[k]);  // facing away from the others
    }
}
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
        """b3Capsule { (0, -h, 0), (0, h, 0), r }: along y, as the solver's capsules run (along_x turns it to x).
        Set by hand (bullet_autofit 0, bullet_radius, bullet_length, bullet_primT), not fitted: AutoFit can pick a
        frame a little off the axis and make it fatter. The geometry is the same capsule, for display."""
        # A polymesh with even rows has no ring on the equator: it is sized so the two rings nearest it are at the
        # radius (cols a multiple of 4 puts points on the axes), and its halves moved apart so the poles end at h + r.
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
        return stretch, "capsule", {"bullet_autofit": 0, "bullet_radius": float(radius),
                                    "bullet_length": 2.0 * half_length, "bullet_primT": (0.0, 0.0, 0.0)}

    def cylinder(self, tag, height, radius, y_offset=0.0, sides=12):
        """b3CreateCylinder( height, radius, yOffset, sides ): a prism of `sides` sides along y, from y_offset
        to y_offset + height in the body's frame, collided as its convex hull."""
        # Its points as b3CreateCylinder makes them: in its order (bottom, top, a side at a time, from +x towards +z)
        # at b3Sin / b3Cos of each angle, which are not exact. The solver hulls the points in the order given.
        tube = create(self.geo, "attribwrangle", f"{tag}_shape")
        set_parm(tube, "class", 0)
        set_parm(tube, "snippet", VEX_FUNCTIONS + f"""\
// b3CreateCylinder( {num(height)}, {num(radius)}, {num(y_offset)}, {sides} )
int sides = {sides};
float height = {num(height)}, radius = {num(radius)}, y_offset = {num(y_offset)};
float alpha = 0.0;
for (int i = 0; i < sides; ++i) {{
    vector2 cs = cossin(alpha);
    addpoint(0, set(radius * cs[0], y_offset, radius * cs[1]));
    addpoint(0, set(radius * cs[0], y_offset + height, radius * cs[1]));
    alpha += 2.0 * PI / sides;
}}
int bottom[], top[];
for (int i = 0; i < sides; ++i) {{
    int j = (i + 1) % sides;
    addprim(0, "poly", 2 * i, 2 * j, 2 * j + 1, 2 * i + 1);
    push(bottom, 2 * i);
    insert(top, 0, 2 * i + 1);
}}
addprim(0, "poly", bottom);
addprim(0, "poly", top);
""")
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

    def stream(self, tag, shape, vex, cd=(0.7, 0.7, 0.7), note="", merged=True, **defaults):
        """Copies `shape` (from box(), sphere(), ...) onto the bodies the VEX makes. defaults: this stream's
        values of the sample's attributes (friction=0.7, ...). merged False: left out of the bodies, for choice()."""
        sop, rep, *rest = shape
        prim = rest[0] if rest else {}  # a shape set by hand: its bullet_* attributes
        prim_sets = "".join(f'    setpointattrib(0, "{k}", pt, {vex_literal(v)[1]});\n' for k, v in prim.items())
        prim_decls = "".join(f'addpointattrib(0, "{k}", {vex_literal(v)[1]});\n'
                             for k, v in prim.items() if k != "bullet_autofit")
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
{prim_sets}    return pt;
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
{prim_decls}addpointattrib(0, "v", {{0, 0, 0}});
addpointattrib(0, "w", {{0, 0, 0}});
{self._declarations(defaults)}
"""
        set_parm(pts, "snippet", (f"// {note}\n" if note else "") + prelude + "\n" + vex.strip() + "\n")
        copy = create(self.geo, "copytopoints", f"{tag}_copy")
        copy.setInput(0, sop)
        copy.setInput(1, pts)
        set_parm(copy, ["pack", "packgeo"], 1)
        # The packed pivot is the body's origin: at the point, as Box3D's body position, not the shape's centroid
        # (a cylinder's base, an offset box: b3MakeOffsetBoxHull, shifted()).
        set_parm(copy, "pivot", 0)
        set_parm(copy, "targetattribs", 1)
        set_parm(copy, "applyattribs1", " ".join(BODY_ATTRIBUTES + ["v", "w"] + list(self.attribs)
                                                 + [k for k in prim if k not in BODY_ATTRIBUTES]))
        if merged:
            self.merge.setInput(len(self.merge.inputs()), copy)
        return copy

    def choice(self, tag, label, options, default=0):
        """One of several streams (made with merged=False), picked by a menu on the bodies' object: a sample's
        radio buttons. options: [(token, menu label, stream)]. A Switch SOP reads the menu, so changing it
        rebuilds the bodies (restart the simulation to see the new one)."""
        switch = create(self.geo, "switch", f"{tag}_choice")
        for i, (_, _, node) in enumerate(options):
            switch.setInput(i, node)
        menu = hou.MenuParmTemplate(tag, label, [t for t, _, _ in options], [lab for _, lab, _ in options],
                                    default_value=default)
        group = self.geo.parmTemplateGroup()
        folder = group.findFolder("Sample")
        if folder is None:
            group.append(hou.FolderParmTemplate("sample", "Sample", [menu]))
        else:
            group.appendToFolder(folder, menu)
        self.geo.setParmTemplateGroup(group)
        switch.parm("input").setExpression(f'ch("../{tag}")')
        self.merge.setInput(len(self.merge.inputs()), switch)
        return switch

    def polygons(self, tag, vex, cd=(0.7, 0.7, 0.7), active=1, note="", rep="convexhull", v=(0.0, 0.0, 0.0),
                 w=(0.0, 0.0, 0.0), **defaults):
        """Bodies whose shapes are polygons the VEX builds (b3CreateHull of a point set): points(positions) adds a
        body's points, face(name, point numbers) a polygon of the body `name` on them, prism(name, positions) and
        hull_points(name, positions) a whole hull (MESH_FUNCTIONS has the mesh builders). Each name is packed into
        one body, collided as its convex hull (rep "concave": see mesh). v, w: every body's starting velocities."""
        build = create(self.geo, "attribwrangle", f"{tag}_polygons")
        set_parm(build, "class", 0)
        set_parm(build, "snippet", (f"// {note}\n" if note else "") + VEX_FUNCTIONS + POLYGON_FUNCTIONS
                 + MESH_FUNCTIONS + vex.strip() + "\n")
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
                 f's@bullet_georep = "{rep}";',
                 "i@bullet_autofit = 1;",
                 f"v@Cd = {vec(cd)};",
                 f"v@v = {vec(v)};",
                 f"v@w = {vec(w)};"]
        self._check_defaults(defaults)
        for name, value in {**self.attribs, **defaults}.items():
            t, lit = vex_literal(value)
            lines.append(f"{t}@{name} = {lit};")
        set_parm(attrs, "snippet", "\n".join(lines))
        self.merge.setInput(len(self.merge.inputs()), attrs)
        return attrs

    def mesh(self, tag, vex, cd=(0.45, 0.5, 0.55), note="", **defaults):
        """Static triangle meshes (b3CreateMeshShape on a static body) the VEX builds with points() and tri(),
        welded() (a triangle soup), grid_mesh(), wave_mesh(), box_mesh() and torus_mesh() (MESH_FUNCTIONS), each name
        one static body of bullet_georep "concave": the solver makes one Box3D mesh of its triangles."""
        return self.polygons(tag, vex, cd=cd, active=0, note=note, rep="concave", **defaults)

    def shifted(self, shape, offset, rotate=None):
        """A shape (from box(), capsule(), ...) moved by offset in the body's frame, after turning it by rotate
        (XYZ Euler degrees, about its centre): a shape not centred on its body's origin
        (b3Capsule { (0, 0, 0), (0, 1, 0), r }). The point stays the body's origin."""
        sop, rep, *rest = shape
        move = create(self.geo, "xform", f"{sop.name()}_moved")
        move.setInput(0, sop)
        set_parm(move, "t", tuple(offset))
        if rotate:
            set_parm(move, "r", tuple(rotate))
        if rest:  # a shape set by hand: its bullet_primT and bullet_primR (it starts centred)
            prim = {**rest[0], "bullet_primT": tuple(offset)}
            if rotate:
                prim["bullet_primR"] = tuple(rotate)
            return move, rep, prim
        if rotate and rep in ("box", "sphere", "capsule"):
            raise SetupError("shifted: a fitted box / sphere / capsule is fitted along the body's axes; turn the body")
        return move, rep

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


SOLVER_NOTE = ("The solver runs as Box3D's sample app: 60 fps with Substeps 1 and Constraint Iterations 4 (one Box3D "
               "step of 4 substeps a frame), gravity -10, Box3D's contact settings and friction / bounce mixing, "
               "Bounding Box Inertia off.")


def finish(s, bodies, about, ported="", try_="", joints=None, lines=None, solver=None, gravity=GRAVITY,
           note_size=(10, 6), emit=False):
    """The DOP network around the bodies, the solver set as Box3D runs, the frame range and the notes.
    joints: [(relationship type, node name, {parm: value}, constraint_name)] for the network `lines`
    (network_sop's (name_a, pos_a, name_b, pos_b, constraint_name[, orient])). emit: a Box3D Solver SOP with Emit RBDs
    in place of the DOP network, for samples that make bodies again during the run (their `rbdbullet_emit`: Every)."""
    if emit:
        if lines:
            raise SetupError(f"{s.tag}: emit takes no joints")
        return _finish_emit(s, bodies, _note_text(s, about, ported, try_), solver, gravity, note_size)
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
    tidy(s.frames, _note_text(s, about, ported, try_), dop, note_size=note_size)
    inner = dop.findStickyNote("how_it_works")  # tidy's note in the DOP network (a sticky note is not a node)
    if inner is not None:
        inner.setText(inner.text() + "\n\n" + SOLVER_NOTE)
        inner.setSize(hou.Vector2(6, 3.5))
    return dop, s.frames


def _note_text(s, about, ported, try_):
    text = f"{s.name.upper()}\nBox3D sample {s.category} / {s.name} (extern/box3d/samples/{s.source})\n\n" \
           f"{about.strip()}"
    if ported:
        text += f"\n\nPort: {ported.strip()}"
    if try_:
        text += f"\n\nTry: {try_.strip()}"
    return text


def _finish_emit(s, bodies, text, solver, gravity, note_size):
    """finish with emit: the bodies into a Box3D Solver SOP with Emit RBDs on. Each frame it adds the bodies whose
    `rbdbullet_emit` is 1 again as new ones (renamed `<name>_<frame * 100>`); those at 0 are made once, on the first
    frame. Returns (the solver SOP, frames)."""
    path = bodies.finish()
    sim = create(hou.node("/obj"), "geo", f"{s.tag}_sim")
    for child in sim.children():
        child.destroy()
    source = create(sim, "object_merge", "bodies")
    set_parm(source, "objpath1", path)
    sop = create(sim, "box3d_solver", "box3d_solver")
    sop.setInput(0, source)
    set_parm(sop, "useemission", 1)
    set_parm(sop, "useground", 0)  # the sample's own ground body, if it has one
    box3d_world(sop, gravity=gravity, **(solver or {}))
    out = create(sim, "null", "OUT")
    out.setInput(0, sop)
    display(out)
    tidy(s.frames, text, sim, note_size=note_size, shown=[sim])
    inner = sim.findStickyNote("how_it_works")
    if inner is not None:
        inner.setText("The Box3D Solver SOP simulates the bodies from the bodies object, with Emit RBDs on (Solver "
                      "tab): every frame it adds the bodies whose rbdbullet_emit is 1 again, as new bodies, which is "
                      "how this scene fires more of them during the run. Bodies at 0 are made once.\n\n" + SOLVER_NOTE)
        inner.setSize(hou.Vector2(6, 3.5))
    return sop, s.frames


def random_vec3(count, lo, hi, seed=12345):
    """The first `count` RandomVec3( lo, hi ) of the samples' generator (shared/utils.h: XorShift32 from RAND_SEED,
    which every Sample's constructor sets)."""
    out = []
    for _ in range(count):
        v = []
        for a, b in zip(lo, hi):
            seed ^= (seed << 13) & 0xFFFFFFFF
            seed ^= seed >> 17
            seed ^= (seed << 5) & 0xFFFFFFFF
            r = (seed % 32768) / 32767.0
            v.append(a + r * (b - a))
        out.append(tuple(v))
    return out


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

