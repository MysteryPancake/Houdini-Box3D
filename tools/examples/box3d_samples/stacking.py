"""Box3D's Stacking samples (samples/sample_stacking.cpp). Each wrangle is the sample's C++ loop in VEX."""

from .common import Bodies, box_impulse, finish, motion_locks, num, sample, vec

SOURCE = "sample_stacking.cpp"


@sample("Stacking", "Card House", SOURCE, frames=300)
def card_house(s):
    b = Bodies(s.tag, {"friction": 0.6, "rollingresistance": 0.0})
    b.ground(10.0)
    b.stream("cards", b.box("cards", 0.001, 0.2, 0.1), """\
// Five rows of leaning cards, each row topped by flat cards (From PEEL).
float cardHeight = 0.2;
float angle0 = radians(25.0), angle1 = radians(-25.0), angle2 = 0.5 * PI;
int Nb = 5;
float z0 = 0.0;
float y = cardHeight - 0.02;
while (Nb > 0) {
    float z = z0;
    for (int i = 0; i < Nb; i++) {
        if (i != Nb - 1)
            body(set(z + 0.25, y + cardHeight - 0.015, 0.0), axis_angle({0, 0, 1}, angle2));
        body(set(z, y, 0.0), axis_angle({0, 0, 1}, angle1));
        z += 0.175;
        body(set(z, y, 0.0), axis_angle({0, 0, 1}, angle0));
        z += 0.175;
    }
    y += cardHeight * 2.0 - 0.03;
    z0 += 0.175;
    Nb--;
}
""", cd=(0.9, 0.88, 0.8), note="b3MakeBoxHull( 0.001, 0.2, 0.1 ) cards", friction=0.7, rollingresistance=0.05)
    return finish(s, b, about="""\
A card house: five rows of cards leaning in pairs, 2 mm thick, with flat cards across the top of each
pair. Friction 0.7 and a little rolling resistance hold it up. (Box3D's own comment says the box
hull's minimum thickness limits this test.)""", try_="lower the cards' friction in cards_points.")


@sample("Stacking", "Sphere Stack", SOURCE, frames=300)
def sphere_stack(s):
    b = Bodies(s.tag)
    b.ground(15.0)
    b.stream("spheres", b.sphere("spheres", 0.5), """\
// 30 spheres one above the other, a radius apart: they fall into a column and topple.
float r = 0.5;
float y = 1.5 * r;
for (int i = 0; i < 30; ++i) {
    body(set(0.0, y, 0.0), identity);
    y += 3.0 * r;
}
""", cd=(0.85, 0.45, 0.3), rollingresistance=0.1)
    return finish(s, b, about="30 spheres dropped in a column, rolling resistance 0.1. Exactly in line, they "
                              "land on each other and balance.",
                  try_="offset one sphere a little in x in spheres_points.")


@sample("Stacking", "Capsule Stack", SOURCE, frames=300)
def capsule_stack(s):
    b = Bodies(s.tag, {"inertialtensorstiffness": 1.0})
    b.ground(40.0)
    b.stream("capsules", b.capsule("capsules", 1.0, 0.5), """\
// 20 capsules lying along x, one above the other.
float r = 0.5;
float y = 1.5 * r;
for (int i = 0; i < 20; ++i) {
    body(set(0.0, y, 0.0), along_x(identity));
    y += 2.0 * r;
}
""", cd=(0.4, 0.65, 0.9), note="b3Capsule { (-1, 0, 0), (1, 0, 0), 0.5 }: built along y, turned to x",
             inertialtensorstiffness=1.0e4)
    lines = motion_locks([(f"capsules{i}", (0.0, 0.75 + i, 0.0)) for i in range(20)])
    return finish(s, b, about="""\
20 capsules stacked one on another. Each is held in the xy plane and cannot turn: Sample::LockMotion
with linearZ and e_fixedRotation.""", ported="""\
a Box3D Motion Lock to the world locks z, as the sample's joint does. Box3D's fixed rotation is a zero
inverse inertia, which has no attribute here: in its place the lock's Lock Tilt holds two axes and
inertialtensorstiffness 1e4 (10000 times as hard to turn) the spin about z.""", try_="set inertialtensorstiffness to 1 in capsules_points: the stack rolls apart.",
                  joints=[("box3d_motionlockconrel", "lock_plane",
                           {"lock_x": 0, "lock_y": 0, "lock_z": 1, "lock_angular_xy": 1}, "motionlock")],
                  lines=lines)


@sample("Stacking", "Single Box", SOURCE, frames=120)
def single_box(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("cube", b.box("cube", 0.5, 0.5, 0.5), "body({0, 0.5, 0}, identity);  // b3MakeCubeHull( 0.5 )",
             cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="One cube resting on the ground. It should stay exactly still.",
                  ported="the sample prints the cube's position; select it in the viewport to read it.")


@sample("Stacking", "Cylinder", SOURCE, frames=300)
def cylinder(s):
    b = Bodies(s.tag)
    b.ground(10.0)
    b.stream("cylinder", b.cylinder("cylinder", 1.0, 0.25, 0.0, 12), "body({0, 2, 0}, identity);",
             cd=(0.5, 0.75, 0.45), note="b3CreateCylinder( 1, 0.25, 0, 12 ): its base at the body's origin",
             rollingresistance=0.05)
    return finish(s, b, about="A 12-sided cylinder hull dropped on its end.",
                  try_="give it a spin: add setpointattrib(0, \"w\", 0, {0, 5, 0}); in cylinder_points.")


@sample("Stacking", "Cylinder Stack", SOURCE, frames=300)
def cylinder_stack(s):
    b = Bodies(s.tag)
    b.ground(10.0)
    b.stream("cylinders", b.cylinder("cylinders", 1.0, 0.5, 0.0, 15), """\
// Ten cylinders stacked on their ends, each scaled by one of four scales (b3CreateTransformedHullShape).
// Copy to Points scales each copy by the point's scale; the mirrors (negative scales) change nothing here.
vector scales[] = array({1, 1, 1}, {0.75, 1, 1}, {1.2, 1, 0.9}, {0.9, 0.9, 0.9});
for (int i = 0; i < 10; ++i) {
    int pt = body(set(0.0, 0.0 + 1.1 * i, 0.0), identity);
    setpointattrib(0, "scale", pt, scales[i % 4]);
}
""", cd=(0.5, 0.75, 0.45), note="b3CreateCylinder( 1, 0.5, 0, 15 )")
    return finish(s, b, about="Ten 15-sided cylinders of four sizes stacked on their ends.")


@sample("Stacking", "Box Stack", SOURCE, frames=600)
def box_stack(s):
    b = Bodies(s.tag)
    b.ground(40.0)
    b.stream("cubes", b.box("cubes", 0.5, 0.5, 0.5), """\
// 40 cubes, a quarter of their size apart: they fall onto each other into one tall stack.
float a = 0.5;
for (int i = 0; i < 40; ++i)
    body(set(0.0, 1.5 * a + 2.5 * a * i, 0.0), identity);
""", cd=(0.9, 0.6, 0.2), rollingresistance=0.1)
    return finish(s, b, about="A column of 40 cubes dropped onto each other. Box3D keeps it standing.",
                  try_="lower Constraint Iterations on the Box3D Solver to 1 and see the stack sway.")


@sample("Stacking", "Jenga Stack", SOURCE, frames=600)
def jenga_stack(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("blocks", b.box("blocks", 1.0, 0.1, 0.1), """\
// 30 layers of two blocks, each layer turned 90 degrees from the one below.
float h = 1.0, r = 0.1;
for (int i = 0; i < 30; ++i) {
    float alpha = (i & 1) == 1 ? 0.0 : 0.5 * PI;
    float x = (i & 1) == 0 ? h - 2.0 * r : 0.0;
    float z = (i & 1) == 0 ? 0.0 : h - 2.0 * r;
    body(set(x, (2.1 * i + 0.5) * r, z), axis_angle({0, 1, 0}, alpha));
    body(set(-x, (2.1 * i + 0.5) * r, -z), axis_angle({0, 1, 0}, alpha));
}
""", cd=(0.85, 0.7, 0.45), rollingresistance=0.05)
    return finish(s, b, about="A Jenga tower: 30 layers of two blocks, each layer crossing the one below.",
                  ported="the sample's Capsule option (capsule blocks) is not built; this is its default, Hull.",
                  try_="in blocks_points, swap a block out from a middle layer.")


@sample("Stacking", "Dominoes", SOURCE, frames=900)
def dominoes(s):
    half, density = (0.2, 0.8, 0.05), 1000.0
    v, w = box_impulse(half, density, (0.0, 0.0, 25.0), (0.0, 0.8, 0.0))
    b = Bodies(s.tag)
    b.ground(80.0)
    b.stream("dominoes", b.box("dominoes", *half), f"""\
// 30 rings of dominoes, one every 2 degrees, each ring spiralling in a little so its last domino
// stands inside its first. The first domino of each ring is pushed over.
for (int ring = 0; ring < 30; ++ring) {{
    float radius = 7.0 + 1.1 * ring;
    for (float alpha = 0; alpha <= 360.0; alpha += 2.0) {{
        vector2 cs = cossin(radians(alpha));  // b3ComputeCosSin
        float c = cs[0], sn = cs[1];
        vector normal = set(c, 0.0, sn);
        vector position = set(radius * c, 0.8, radius * sn) - alpha / 630.0 * normal;
        int pt = body(position, axis_angle({{0, 1, 0}}, -radians(alpha)));
        setpointattrib(0, "Cd", pt, hsvtorgb(set(ring / 30.0, 0.55, 0.95)));
        if (alpha == 0.0) {{
            // b3Body_ApplyLinearImpulse( (0, 0, 25) ) at its top, as the velocity it gives
            setpointattrib(0, "v", pt, {vec(v)});
            setpointattrib(0, "w", pt, {vec(w)});
        }}
    }}
}}
""", note="b3MakeBoxHull( 0.2, 0.8, 0.05 ) dominoes")
    return finish(s, b, about="""\
5430 dominoes in 30 rings. The first of each ring is pushed over and the fall runs round every ring.""",
                  ported=f"""\
the sample's push is b3Body_ApplyLinearImpulse( (0, 0, 25) ) at the domino's top; here it is the
velocity that gives (v {num(v[2])} along z, w {num(w[0])} about x).""",
                  try_="fewer rings (ring < 30 in dominoes_points) for a quicker scene.")


@sample("Stacking", "Wedge", SOURCE, frames=240)
def wedge(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.polygons("wedge", """\
// b3CreateHull of six points, the body at (0, 1, 0): a flat top 2 x 0.2 and an edge 1 long below it.
vector o = {0, 1, 0};
vector v[] = array({-1.0, 1.0, -0.1}, {1.0, 1.0, -0.1}, {-1.0, 1.0, 0.1}, {1.0, 1.0, 0.1},
                   {-0.5, 0.5, 0.0}, {0.5, 0.5, 0.0});
foreach (int i; vector p; v) v[i] = p + o;
face("wedge", array(v[0], v[2], v[3], v[1]));
face("wedge", array(v[0], v[1], v[5], v[4]));
face("wedge", array(v[3], v[2], v[4], v[5]));
face("wedge", array(v[0], v[4], v[2]));
face("wedge", array(v[1], v[3], v[5]));
""", cd=(0.85, 0.55, 0.3))
    return finish(s, b, about="""\
A wedge whose lower edge lands on the ground. Box3D's comment: this shape can get a wrong contact
manifold if not handled correctly. It tips over onto one of its sloped faces.""")


@sample("Stacking", "Arch", SOURCE, frames=600)
def arch(s):
    ps1 = [(16.0, 0.0), (14.93803712795643, 5.133601056842984), (13.79871746027416, 10.24928069555078),
           (12.56252963284711, 15.34107019122473), (11.20040987372525, 20.39856541571217),
           (9.66521217819836, 25.40369899225096), (7.87179930638133, 30.3179337000085),
           (5.635199558196225, 35.03820717801641), (2.405937953536585, 39.09554102558315)]
    ps2 = [(24.0, 0.0), (22.33619528222415, 6.02299846205841), (20.54936888969905, 12.00964361211476),
           (18.60854610798073, 17.9470321677465), (16.46769273811807, 23.81367936585418),
           (14.05325025774858, 29.57079353071012), (11.23551045834022, 35.13775818285372),
           (7.752568160730571, 40.30450679009583), (3.016931552701656, 44.28891593799322)]

    def arr(points):
        return "array(" + ", ".join(vec((x, y, 0.0)) for x, y in points) + ")"

    b = Bodies(s.tag)
    b.ground(40.0)
    b.polygons("stones", f"""\
// The arch's stones: hulls between an inner (ps1) and an outer (ps2) curve, scaled by 0.25, 1 deep.
vector ps1[] = {arr(ps1)};
vector ps2[] = {arr(ps2)};
float scale = 0.25, d = 0.5;
foreach (int i; vector p; ps1) ps1[i] = set(p.x * scale, p.y * scale, 0.0);
foreach (int i; vector p; ps2) ps2[i] = set(p.x * scale, p.y * scale, 0.0);
vector back = set(0, 0, -d), front = set(0, 0, d);
for (int i = 0; i < 8; ++i) {{
    // right side, then its mirror on the left
    prism(sprintf("right%d", i), array(ps1[i] + back, ps2[i] + back, ps2[i + 1] + back, ps1[i + 1] + back,
                                       ps1[i] + front, ps2[i] + front, ps2[i + 1] + front, ps1[i + 1] + front));
    vector m = {{-1, 1, 1}};
    prism(sprintf("left%d", i), array(ps2[i] * m + back, ps1[i] * m + back, ps1[i + 1] * m + back,
                                      ps2[i + 1] * m + back, ps2[i] * m + front, ps1[i] * m + front,
                                      ps1[i + 1] * m + front, ps2[i + 1] * m + front));
}}
vector m = {{-1, 1, 1}};
prism("keystone", array(ps1[8] + back, ps2[8] + back, ps2[8] * m + back, ps1[8] * m + back,
                        ps1[8] + front, ps2[8] + front, ps2[8] * m + front, ps1[8] * m + front));
""", cd=(0.72, 0.66, 0.58), density=200.0)
    b.stream("blocks", b.box("blocks", 2.0, 0.5, 0.5), f"""\
// Four blocks stacked on the keystone.
float top = {num(ps2[8][1] * 0.25)};
for (int i = 0; i < 4; ++i)
    body(set(0.0, 0.5 + top + 1.0 * i, 0.0), identity);
""", cd=(0.6, 0.55, 0.5), density=200.0)
    return finish(s, b, about="""\
A free-standing stone arch of 17 hull stones (density 200), with four blocks stacked on the keystone.
Nothing holds the stones but friction and their weight.""",
                  try_="lower friction in stones_attributes; raise Substeps if it creeps.")


@sample("Stacking", "Double Domino", SOURCE, frames=300)
def double_domino(s):
    half, density = (0.125, 0.5, 0.25), 4.0
    v, w = box_impulse(half, density, (0.2, 0.0, 0.0), (0.0, 0.5, 0.0))
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("dominoes", b.box("dominoes", *half), f"""\
// 15 dominoes 1.01 apart, the first pushed over.
int count = 15;
float x = -0.5 * count;
for (int i = 0; i < count; ++i) {{
    int pt = body(set(x, 0.5, 0.0), identity);
    if (i == 0) {{
        // b3Body_ApplyLinearImpulse( (0.2, 0, 0) ) at its top, as the velocity it gives
        setpointattrib(0, "v", pt, {vec(v)});
        setpointattrib(0, "w", pt, {vec(w)});
    }}
    x += 1.01;
}}
""", cd=(0.9, 0.85, 0.75), density=density, friction=0.6)
    return finish(s, b, about="A row of 15 light dominoes (density 4), the first pushed over.",
                  ported="the push is the velocity b3Body_ApplyLinearImpulse gives the first domino.")


@sample("Stacking", "Pyramid2D", SOURCE, frames=600)
def pyramid_2d(s):
    a, size = 0.25, 12
    b = Bodies(s.tag)
    b.ground(1000.0)
    b.stream("boxes", b.box("boxes", a, a, a), f"""\
// A pyramid 12 boxes wide in the xy plane.
float a = {num(a)};
int size = {size};
for (int row = 0; row < size; ++row)
    for (int column = 0; column < size - row; ++column)
        body(set((-10.0 + 2.0 * column + row) * a, (1.5 + 2.5 * row) * a, 0.0), identity);
""", cd=(0.9, 0.6, 0.2))
    centres = []
    for row in range(size):
        for column in range(size - row):
            centres.append((f"boxes{len(centres)}", ((-10.0 + 2.0 * column + row) * a, (1.5 + 2.5 * row) * a, 0.0)))
    return finish(s, b, about="""\
A pyramid kept two-dimensional: each box is held in the xy plane and turns only about z
(Sample::LockMotion with linearZ and e_spinZ).""", ported="""\
the lock is a Box3D Motion Lock relationship (Lock Z and Lock Tilt) from each box to the world, one network
line per box.""", try_="turn Lock Tilt off on lock_plane and see the boxes topple out of the plane.",
                  joints=[("box3d_motionlockconrel", "lock_plane",
                           {"lock_x": 0, "lock_y": 0, "lock_z": 1, "lock_angular_xy": 1}, "motionlock")],
                  lines=motion_locks(centres))


@sample("Stacking", "Edge Crossing", SOURCE, frames=300)
def edge_crossing(s):
    b = Bodies(s.tag)
    b.ground(40.0)
    loop = """\
// Three rows (z -2, 0, 2) of a flat stick with a stick dropped across it at 21 angles about a tilted axis.
// Row 0: flat and dropped are box1. Row 1: both box2 (box1 on its side). Row 2: flat box1, dropped box2.
vector h = {0.2, 0.02, 0.04};
vector axis = normalize({0.1, 0.9, 0.0});
for (int row = 0; row < 3; ++row) {
    float x = -10.0, z = -2.0 + 2.0 * row;
    for (float angle = -PI; angle < PI + 0.001; angle += 0.1 * PI) {
        float flatY = row == 1 ? h.z : h.y;
        float dropY = 20.0 * flatY;
        if (BOX_FLAT == (row == 1 ? 2 : 1)) body(set(x, flatY, z), identity);
        if (BOX_DROP == (row == 0 ? 1 : 2)) body(set(x, dropY, z), axis_angle(axis, angle));
        x += 1.0;
    }
}
"""
    b.stream("box1", b.box("box1", 0.2, 0.02, 0.04), "#define BOX_FLAT 1\n#define BOX_DROP 1\n" + loop,
             cd=(0.9, 0.6, 0.2), note="b3MakeBoxHull( 0.2, 0.02, 0.04 ) sticks")
    b.stream("box2", b.box("box2", 0.2, 0.04, 0.02), "#define BOX_FLAT 2\n#define BOX_DROP 2\n" + loop,
             cd=(0.4, 0.65, 0.9), note="b3MakeBoxHull( 0.2, 0.04, 0.02 ) sticks")
    return finish(s, b, about="""\
Thin sticks dropped across thin sticks at 21 angles, in three rows: a test of edge-on-edge contact.
Each pair should settle crossed, without jitter.""")
