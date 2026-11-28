"""Box3D's Mesh samples (samples/sample_mesh.cpp): bodies on static triangle meshes."""

import math

from .common import Bodies, finish, sample, vec

SOURCE = "sample_mesh.cpp"

COLOURS = {"sphere": (0.85, 0.45, 0.3), "capsule": (0.4, 0.65, 0.9), "box": (0.9, 0.6, 0.2),
           "cylinder": (0.5, 0.75, 0.45)}

SHAPE_TRY = "pick another shape: Shape on the bodies object's Sample tab, then restart the simulation."


def falling_shape(b, default, options):
    """The sample's falling body, one of several shapes as its radio buttons pick it: a stream per shape, chosen
    by the Shape menu on the bodies object (Bodies.choice). options: [(token, shape, vex, note, attributes)]."""
    streams = [(token, token.capitalize(), b.stream(token, shape, vex, cd=COLOURS[token], note=note, merged=False,
                                                    **attrs))
               for token, shape, vex, note, attrs in options]
    b.choice("shape", "Shape", streams, default=[t for t, *_ in options].index(default))


# GridMesh's and BigBoxMesh's capsule: b3Capsule { (0, 0, 1.276), (0, 0, 0.476), 0.15 }, along z and off the body's
# origin, on an unturned body. The capsule (along y, set by hand) is turned -90 degrees about x, which puts its ends in
# Box3D's order (the first one far): turned +90 the same capsule tips the other way where it starts sunk in Big Box.
Z_CAPSULE_NOTE = "b3Capsule { (0, 0, 1.276), (0, 0, 0.476), 0.15 }: turned to z, 0.876 from the body's origin"


def z_capsule(b):
    return b.shifted(b.capsule("capsule", 0.4, 0.15), (0.0, 0.0, 0.876), rotate=(-90.0, 0.0, 0.0))


@sample("Mesh", "Grid", SOURCE, frames=300)
def grid(s):
    b = Bodies(s.tag, {"angulardamping": 0.0})
    b.mesh("grid", """\
// b3CreateGridMesh( 20, 20, 1 ) with the shape's scale (2, 2, 2): a flat 40 x 40 mesh at y = 0.
grid_mesh("grid", 20, 20, 1.0, {2, 2, 2}, {0, 0, 0});
""")
    at = "{0.1, 1, -0.1}"
    falling_shape(b, "cylinder", [
        ("sphere", b.sphere("sphere", 0.5), f"body({at}, identity);", "radius 0.5", {"rollingresistance": 0.05}),
        ("capsule", z_capsule(b), f"body({at}, identity);", Z_CAPSULE_NOTE,
         {"rollingresistance": 0.05}),
        ("box", b.box("box", 0.5, 0.5, 0.5), f"body({at}, identity);", "b3MakeBoxHull( 0.5, 0.5, 0.5 )", {}),
        ("cylinder", b.cylinder("cylinder", 1.0, 0.25, 0.0, 15), f"body({at}, identity);",
         "b3CreateCylinder( 1, 0.25, 0, 15 ): its base at the body's origin",
         {"rollingresistance": 0.02, "angulardamping": 0.1}),
    ])
    return finish(s, b, about="""\
A body dropped onto a flat triangle mesh (a 20 x 20 grid scaled by 2), 1 m up. The default is a 15-sided
cylinder on its end: it lands, rocks on its rim and settles upright on the mesh's triangles. Shape on
the bodies object picks a sphere, a capsule, a box or the cylinder, as the sample's buttons do.""",
                  ported="the sample's sliders scale the mesh; the port keeps the scale of 2.",
                  try_=SHAPE_TRY)


@sample("Mesh", "Big Box", SOURCE, frames=300)
def big_box(s):
    b = Bodies(s.tag)
    b.mesh("slab", """\
// b3CreateBoxMesh( (0, -1, 0), (50, 1, 50) ): a closed box mesh 100 x 2 x 100 whose top is at y = 0.
box_mesh("slab", {0, -1, 0}, {50, 1, 50}, 0, {0, 0, 0, 1}, {0, 0, 0});
""", friction=0.5)
    at = "{0.5, 0, 0}"
    falling_shape(b, "cylinder", [
        ("sphere", b.sphere("sphere", 0.5), f"body({at}, identity);", "radius 0.5, its centre on the mesh",
         {"rollingresistance": 0.05}),
        ("capsule", z_capsule(b), f"body({at}, identity);", Z_CAPSULE_NOTE,
         {"rollingresistance": 0.1}),
        ("box", b.box("box", 0.5, 0.5, 0.5), f"body({at}, identity);", "b3MakeBoxHull( 0.5 ), its centre on the mesh",
         {"rollingresistance": 0.05}),
        ("cylinder", b.cylinder("cylinder", 0.3, 0.15, 0.0, 32), f"body({at}, identity);",
         "b3CreateCylinder( 0.3, 0.15, 0, 32 ): its base on the mesh", {"rollingresistance": 0.05}),
    ])
    return finish(s, b, about="""\
A body on a large box mesh (the ground is a mesh, not a box), its origin on the mesh's top. The default
small 32-sided cylinder stands still: a flat hull on large mesh triangles. The sphere, capsule and box
(Shape on the bodies object) start half sunk in the mesh, as in Box3D, and are pushed out.""",
                  ported="the sample's sliders scale the mesh; the port keeps it unscaled.", try_=SHAPE_TRY)


@sample("Mesh", "Box", SOURCE, frames=240)
def box(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.mesh("box_mesh", """\
// b3CreateBoxMesh( (0, 1, 0), (1, 1, 1) ) on a static body at (0, -1, 0) turned 45 degrees about y: a 2 m box
// mesh half sunk in the ground, its top at y = 1.
box_mesh("box_mesh", {0, 1, 0}, {1, 1, 1}, 0, axis_angle({0, 1, 0}, 0.25 * PI), {0, -1, 0});
""")
    falling_shape(b, "box", [
        ("sphere", b.sphere("sphere", 0.5), "body({0, 1.5, 0}, identity);", "radius 0.5", {}),
        ("capsule", b.capsule("capsule", 0.5, 0.1), "body({0, 1.5, 0}, along_x(identity));",
         "b3Capsule { (-0.5, 0, 0), (0.5, 0, 0), 0.1 }: built along y, turned to x", {}),
        ("box", b.box("box", 0.5, 0.5, 0.5), "body({0, 1.5, 0}, identity);",
         "b3MakeBoxHull( 0.5, 0.5, 0.5 ), resting on the mesh's top", {}),
        ("cylinder", b.cylinder("cylinder", 1.0, 0.75, 0.0, 8), "body({0, 1, 0}, identity);",
         "b3CreateCylinder( 1, 0.75, 0, 8 ): its base at the body's origin, on the box", {}),
    ])
    return finish(s, b, about="""\
A body on top of a box mesh turned 45 degrees. The default cube rests with its corners over the mesh's
edges and should stay still. Shape on the bodies object picks a sphere, a capsule or a wide 8-sided
cylinder standing on the box instead, as the sample's buttons do.""",
                  ported="the sample's sliders scale the mesh; the port keeps it unscaled.", try_=SHAPE_TRY)


VOXEL_ORIGIN = (5000.0, 3500.0, -7000.0)  # where the sample puts the mesh; the port puts it at the origin
VOXEL_HULL = [(-3.13548756, 3.81141949, 237.289047), (-16.2333279, -23.4977913, 235.486603),
              (-13.8834839, 6.20244455, 23.7760544), (14.0794125, 4.63170528, 24.9530792),
              (3.98322797, -16.4192238, 236.704071), (-23.3520412, -3.26714420, 236.071594),
              (13.4517860, -6.94963741, 24.4085312), (-5.24953651, 13.9316301, 24.5058060),
              (-4.65071201, -24.1484108, 235.974121), (-14.5111103, -5.37889385, 23.2315063),
              (6.33307076, 13.2810068, 24.9935150), (4.81784487, -14.6788225, 23.6787796),
              (-14.7180958, 4.46204281, 236.801331), (-23.9796677, -14.8484812, 235.527039),
              (4.61085415, -4.83788204, 237.248611), (-6.76476669, -14.0281992, 23.1910706)]


@sample("Mesh", "Voxel", SOURCE, frames=300)
def voxel(s):
    b = Bodies(s.tag)
    b.mesh_file("voxels", "collision_mesh_01.obj", scale=0.01, z_up=True, weld=0.002)
    body = tuple(a - o for a, o in zip((5020.27734, 3506.22559, -6986.48584), VOXEL_ORIGIN))
    hull = [(0.01 * y, 0.01 * z, 0.01 * x) for x, y, z in VOXEL_HULL]
    b.stream("cylinder", b.hull("cylinder", hull, note="a rough 2 m cylinder of 16 points"),
             f"body({vec(body)}, {vec((0.664546967, 0.669287264, 0.135021493, 0.303646326))});",
             cd=(0.5, 0.75, 0.45), rollingresistance=0.1)
    return finish(s, b, about="""\
A rough cylinder (a 16-point hull, 2 m long) dropped onto a voxel terrain mesh from a game
(collision_mesh_01.obj): thousands of small, nearly coplanar faces, welded within 2 mm as Box3D's mesh
builder does. It should roll and settle on the steps without catching on the faces' edges.""",
                  ported=f"""\
the sample puts the mesh {VOXEL_ORIGIN} m from the origin, which needs Box3D's double precision; here it
is at the origin. The weld is a Fuse SOP (voxels_welded). The OBJ is copied to examples/cache.""")


@sample("Mesh", "Hollow Box", SOURCE, frames=300)
def hollow_box(s):
    b = Bodies(s.tag, {"gravityscale": 1.0, "enablesleep": 1})
    b.mesh("room", """\
// b3CreateHollowBoxMesh( (0, 0, 0), (10, 10, 10) ): a 20 m box mesh facing inwards.
box_mesh("room", {0, 0, 0}, {10, 10, 10}, 1, {0, 0, 0, 1}, {0, 0, 0});
""", cd=(0.55, 0.6, 0.65))
    b.stream("cylinders", b.cylinder("cylinders", 1.0, 0.25, 0.0, 8), """\
// Six cylinders, one against each wall, some sunk into it: the floor's starts 0.2 below it.
vector at[] = array({0, -10.2, 0}, {0, 9.2, 0}, {-9.8, 0, 0}, {9.8, 0, 0}, {0, 0, -9.8}, {0, 0, 9.8});
foreach (vector p; at) body(p, identity);
""", cd=(0.5, 0.75, 0.45), note="b3CreateCylinder( 1, 0.25, 0, 8 ): its base at the body's origin",
             gravityscale=0.0, enablesleep=0)
    b.stream("capsules", b.shifted(b.capsule("capsules", 0.5, 0.25), (0.0, 0.5, 0.0)), """\
// Eight capsules from the body's origin up 1 m, against the walls, floor and ceiling.
vector at[] = array({0, -10.2, 2}, {0, 9.2, 2}, {0, -9.9, 4}, {0, 8.9, 4},
                    {-9.8, 2, 0}, {9.8, 2, 0}, {0, 2, -9.8}, {0, 2, 9.8});
foreach (vector p; at) body(p, identity);
""", cd=(0.4, 0.65, 0.9), note="b3Capsule { (0, 0, 0), (0, 1, 0), 0.25 }: from the body's origin up",
             gravityscale=0.0, enablesleep=0)
    return finish(s, b, about="""\
Cylinders and capsules against the inside of a hollow box mesh, with no gravity and sleep off. Those
that start sunk into a wall are pushed out by the mesh and drift off it; the others stay where they are.
The sample is for looking at the contact manifolds of shapes lined up with the mesh's axes.""",
                  ported="""\
the four cylinders against the side walls drift inwards at 0.5 m/s and up, as in Box3D, but one of them
drifts down: which contact point wins a tie depends on the order of the hull's vertices.""",
                  try_="give the bodies gravity: gravityscale 1 in cylinders_points and capsules_points.")


def _inscribed_radius(subdivisions):
    """The distance from the centre to the nearest face of the unit icosphere WedgedSphere::Build makes."""
    t = 0.5 * (1.0 + math.sqrt(5.0))
    corners = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
               (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
             (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11),
             (6, 2, 10), (8, 6, 7), (9, 8, 1)]

    def unit(v):
        n = math.sqrt(sum(c * c for c in v))
        return tuple(c / n for c in v)

    def add(a, c):
        return unit(tuple(x + y for x, y in zip(a, c)))

    tris = [tuple(unit(corners[i]) for i in f) for f in faces]
    for _ in range(subdivisions):
        tris = [t2 for a, b, c in tris for t2 in ((a, add(a, b), add(c, a)), (add(a, b), b, add(b, c)),
                                                  (add(c, a), add(b, c), c), (add(a, b), add(b, c), add(c, a)))]
    radius = 1.0
    for a, b, c in tris:
        u = tuple(y - x for x, y in zip(a, b))
        v = tuple(y - x for x, y in zip(a, c))
        n = unit((u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]))
        radius = min(radius, abs(sum(x * y for x, y in zip(n, a))))
    return radius


@sample("Mesh", "Wedged Sphere", SOURCE, frames=300)
def wedged_sphere(s):
    subdivisions, fit = 3, 5.0
    cavity = _inscribed_radius(subdivisions)
    radius = cavity + 0.001 * fit
    b = Bodies(s.tag)
    b.mesh("cavity", f"""\
// A unit icosphere subdivided {subdivisions} times (1280 triangles) facing inwards, as WedgedSphere::Build makes it.
float t = 0.5 * (1.0 + sqrt(5.0));
vector c[] = array(set(-1, t, 0), set(1, t, 0), set(-1, -t, 0), set(1, -t, 0), set(0, -1, t), set(0, 1, t),
                   set(0, -1, -t), set(0, 1, -t), set(t, 0, -1), set(t, 0, 1), set(-t, 0, -1), set(-t, 0, 1));
int f[] = array(0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1);
vector tris[];  // three corners a triangle
foreach (int i; f) push(tris, normalize(c[i]));
for (int level = 0; level < {subdivisions}; ++level) {{
    vector next[];
    for (int i = 0; i < len(tris); i += 3) {{
        vector a = tris[i], b = tris[i + 1], cc = tris[i + 2];
        vector ab = normalize(a + b), bc = normalize(b + cc), ca = normalize(cc + a);
        push(next, array(a, ab, ca, ab, b, bc, ca, bc, cc, ab, bc, ca));
    }}
    tris = next;
}}
// Welded as Box3D's mesh builder does (weldTolerance 1e-4), and each triangle reversed so it faces the centre.
int q[] = welded(tris, 1e-4);
for (int i = 0; i < len(q); i += 3) tri("cavity", q[i], q[i + 2], q[i + 1]);
""", cd=(0.55, 0.6, 0.65))
    b.stream("ball", b.sphere("ball", radius), "body({0, 0, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note=f"radius {radius:.4f}: the cavity's {cavity:.4f} plus {fit:g} mm")
    return finish(s, b, about=f"""\
A sphere of radius {radius:.4f} wedged inside an inward-facing sphere mesh whose faces are {cavity:.4f}
from the centre: {fit:g} mm too big, so every nearby triangle touches it with a different normal. It
should stay inside, settle and go to sleep.""",
                  ported="""\
the sample's readouts (manifolds, speed, offset from the centre, steps to sleep) are dropped:
select the ball to see its position and whether it sleeps. Its Kick button is a starting v.""",
                  try_="kick it: setpointattrib(0, \"v\", 0, {6, 7, 3.9}); in ball_points.")
