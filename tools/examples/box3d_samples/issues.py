"""Box3D's Issues samples (samples/sample_issues.cpp): scenes kept from bug reports."""

from .common import Bodies, Every, finish, height_field, num, sample, vec
from .joints import node

SOURCE = "sample_issues.cpp"

# ConvexJitter's two hulls: the point sets in their source's axes (y and z swapped and scaled by 0.01 when used), the
# body positions (the same swap, plus 2 up) and rotations.
JITTER_STATIC = [(-44.8770714, -91.6598053, -1.92012548), (-92.5001831, 51.0151291, 15.8006573),
                 (-91.0282211, -9.44371605, 15.6148796), (90.2375641, 77.3870087, 15.9356089),
                 (-85.5353241, 91.3750992, -1.36629653), (88.9092178, -87.2975464, -1.86754704),
                 (83.7932816, -89.8572235, 15.4168339), (87.0243988, 88.9776535, -1.32423306),
                 (-91.6564941, -85.4949493, 15.3782759), (-90.2922516, -87.2074127, -1.92012548),
                 (-87.2944870, 89.9510498, 15.9215889), (79.2338104, 89.9690781, 15.9724140),
                 (-91.6744461, 81.0823212, -1.39959598), (90.3452759, -76.4459610, 15.4588966),
                 (-87.4021912, -89.2263107, 15.3677588), (76.3258057, 92.0059967, 1.82873762)]
JITTER_DYNAMIC = [(29.5, 17.1488495, 0.175081104), (29.5, -17.2990532, 0.125), (29.4840164, -17.3057766, 24.0200863),
                  (29.4840164, 17.1648350, 24.1781254), (-29.1345520, 17.5529804, 0.125),
                  (-29.1345520, 17.5529804, 23.7899799), (-29.1441040, 16.9679585, 24.375),
                  (-29.1345520, -17.2990532, 24.375), (-29.1345520, -17.2990532, 0.175081253),
                  (29.0720215, 17.5529785, 0.125), (29.0859070, 17.5629406, 23.8120594),
                  (29.1401348, -17.2990532, 24.375), (29.1123581, 16.9722290, 24.4027710),
                  (29.3944912, 17.2543602, 24.1206398), (-29.1345520, -17.2990532, 24.0759430),
                  (-29.1345520, -16.9722252, 24.4027710), (29.1123619, -16.9722271, 24.4027729),
                  (29.5, 17.3429642, 24.0)]


# GMod Wheel Stack's wheel: b3CreateHull of the vertices of its 37 piece hulls (s_metalWheel1Verts / Hulls in
# sample_issues.cpp), as Box3D builds it: these 64 points, printed by a program linked with Box3D to full float
# precision. The stack is that sensitive: rounded to 6 decimals, its faces change and it falls.
WHEEL_HULL = [
    (-0.49721697, -0.0847039968, -1.60001218e-05), (-0.000108003616, 0.0863350108, 0.497087955),
    (0.497002989, -0.0847039968, -2.89976597e-05), (-0.49721697, 0.0863349959, -1.60001218e-05),
    (0.497002989, 0.0863349959, -2.89976597e-05), (-0.000105001032, -0.0847039968, -0.497132987),
    (-0.000108003616, -0.0847039968, 0.497087955), (0.351408005, 0.0863349959, 0.351482987),
    (-0.351615012, 0.0863349959, -0.351534009), (-0.351617992, 0.0863349959, 0.351487011),
    (0.351404995, 0.0863349959, -0.351530999), (0.351402014, -0.0847039968, 0.351489007),
    (-0.351615012, -0.0847039968, -0.351534009), (-0.351617992, -0.0847039968, 0.351487011),
    (0.351404995, -0.0847039968, -0.351530999), (0.45916301, -0.0847039968, 0.190215006),
    (-0.459376007, 0.0863350108, -0.190259993), (0.190127, 0.0863349959, 0.459248006),
    (0.190130994, 0.0863349959, -0.459291995), (-0.190344006, 0.0863350108, 0.459246993),
    (-0.190340996, 0.0863350108, -0.459293008), (0.459163994, 0.0863349959, -0.190256998),
    (-0.459374011, 0.0863349959, 0.190219998), (-0.459376007, -0.0847039968, -0.190259993),
    (0.45916301, 0.0863350108, 0.190215006), (0.190130994, -0.0847039968, -0.459291995),
    (-0.190344006, -0.0847039968, 0.459246993), (-0.190340996, -0.0847039968, -0.459293008),
    (0.190127, -0.0847039968, 0.459248006), (0.459163994, -0.0847039968, -0.190256998),
    (-0.459374011, -0.0847039968, 0.190219998), (0.276071012, 0.0863349959, 0.413311005),
    (-0.487664998, 0.0863349959, -0.0970050022), (0.487453014, -0.0847039968, 0.0969519913),
    (-0.0970899984, 0.0863349959, 0.487536013), (-0.276288003, 0.0863349959, 0.413309008),
    (-0.413437992, 0.0863349959, -0.276203007), (0.0968730003, 0.0863349959, 0.487536013),
    (0.413224012, -0.0847039968, 0.276158988), (-0.276284993, 0.0863349959, -0.413354993),
    (-0.0970859975, 0.0863350108, -0.487581015), (-0.487664998, 0.0863349959, 0.0969569981),
    (0.0968680009, 0.0863350108, -0.487581998), (0.276066989, 0.0863349959, -0.413358003),
    (0.487452, 0.0863349959, -0.0970019996), (-0.413439989, 0.0863349959, 0.276156008),
    (0.413226008, 0.0863350108, -0.276199996), (-0.487664998, -0.0847039968, -0.0970050022),
    (0.487453014, 0.0863349959, 0.0969519913), (-0.0970899984, -0.0847039968, 0.487536013),
    (0.276071012, -0.0847039968, 0.413311005), (-0.276288003, -0.0847039968, 0.413309008),
    (-0.413437992, -0.0847039968, -0.276203007), (0.0968730003, -0.0847039968, 0.487536013),
    (-0.276284993, -0.0847039968, -0.413354993), (-0.0970859975, -0.0847039968, -0.487581015),
    (-0.487664998, -0.0847039968, 0.0969569981), (0.0968680009, -0.0847039968, -0.487581998),
    (0.276066989, -0.0847039968, -0.413358003), (0.487452, -0.0847039968, -0.0970019996),
    (-0.413439989, -0.0847039968, 0.276156008), (0.413226008, -0.0847039968, -0.276199996),
    (0.413224012, 0.0863349959, 0.276158988), (-0.000105001032, 0.0863350108, -0.497132987)
]


def _jitter_hull(name, points, base, quat):
    """VEX for one ConvexJitter hull: its points swapped and scaled as the sample does, placed by the body's pose."""
    s = 0.01
    local = [(s * x, s * z, s * y) for x, y, z in points]
    pos = (s * base[0], s * base[2] + 2.0, s * base[1])
    return (f"vector {name}[] = array({', '.join(vec(p) for p in local)});\n"
            f"foreach (int i; vector p; {name}) {name}[i] = qrotate({vec(quat)}, p) + {vec(pos)};\n"
            f'hull_points("{name}", {name});\n')


@sample("Issues", "Crash", SOURCE, frames=240)
def crash(s):
    b = Bodies(s.tag)
    b.mesh("grid", """\
// b3CreateGridMesh( 20, 20, 2 ) on a static body at (0, -1, 0): a flat 40 x 40 mesh at y = -1.
grid_mesh("grid", 20, 20, 2.0, {1, 1, 1}, {0, -1, 0});
""")
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), "body({2, 4, 0}, identity);\nbody({-2, 4, 0}, identity);",
             cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="""\
Two cubes 4 m apart welded together by a weld whose frames are at each cube's centre, so it pulls the
centres together: the cubes crash into each other, then fall onto a grid mesh as one. Adding this joint
mid-run once crashed Box3D.""",
                  ported="the sample adds the weld with its Add Joint button; here it is there from the start.",
                  joints=[node("weld", "pull_together")],
                  lines=[("boxes0", (2, 4, 0), "boxes1", (-2, 4, 0), "pull_together")])


@sample("Issues", "Multiple Prismatic", SOURCE, frames=240)
def multiple_prismatic(s):
    b = Bodies(s.tag)
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), """\
// Six cubes in a column, each on a prismatic joint to the one below (the first to the world at the origin).
for (int i = 0; i < 6; ++i) body(set(0, 0.6 + 1.2 * i, 0), identity);
""", cd=(0.9, 0.6, 0.2))
    lines = [("" if i == 0 else f"boxes{i - 1}", (0, 1.2 * i, 0), f"boxes{i}", (0, 1.2 * i, 0), "sliders")
             for i in range(6)]
    return finish(s, b, about="""\
A column of six cubes, each sliding along x on the cube below (the first on the world), limited to
6 m either way, Constraint Stiffness 240. Gravity pulls across the sliders, so the column holds still.
The sample is for dragging the cubes with the mouse.""",
                  try_="push them: v {3, 0, 0} on one cube in boxes_points.",
                  joints=[node("prismatic", "sliders", constraint_stiffness=240.0, enable_limit=1,
                               lower_translation=-6.0, upper_translation=6.0)], lines=lines)


@sample("Issues", "Convex Jitter", SOURCE, frames=300)
def convex_jitter(s):
    b = Bodies(s.tag)
    b.ground(10.0)
    b.polygons("base", "// A static hull of 16 points from a game's level (b3CreateHull).\n"
               + _jitter_hull("base", JITTER_STATIC, (-459.292877, 217.398331, 1.00115335),
                              (0.0, -0.707106769, 0.0, 0.707106769)),
               cd=(0.55, 0.6, 0.65), active=0)
    b.polygons("crate", "// A dynamic hull of 18 points resting on it.\n"
               + _jitter_hull("crate", JITTER_DYNAMIC, (-402.321838, 157.310364, 16.8169250),
                              (0.0, -0.00152086187, 0.0, 0.999998868)),
               cd=(0.9, 0.6, 0.2), rollingresistance=0.1)
    return finish(s, b, about="""\
Two hulls from a game's level: a crate made of 18 points, nearly a box, resting on a static hull of
16 points. The crate used to jitter on it; it should settle and sleep.""")


@sample("Issues", "GMod Wheel Stack", SOURCE, frames=600)
def gmod_wheel_stack(s):
    height = 0.171
    spacing, start = height + 0.006, 0.5 * height + 0.004
    b = Bodies(s.tag)
    b.ground(10.0)
    b.stream("wheels", b.hull("wheels", WHEEL_HULL, note="the wheel: one hull of 64 points (WHEEL_HULL)"), f"""\
// 30 wheels lying flat, stacked {num(spacing)} apart.
for (int i = 0; i < 30; ++i) body(set(0.0, {num(start)} + i * {num(spacing)}, 0.0), identity);
""", cd=(0.6, 0.62, 0.66))
    return finish(s, b, about="""\
30 metal wheels (1 m across, 17 cm thick) stacked flat, from a game prop. Each wheel is one hull of 64
points wrapped around its 37-piece convex decomposition: Box3D's comment says the single hull simulates
better. It settles leaning a little (the top wheel about 0.35 m off-centre after 5 s) and falls asleep
standing, as in Box3D.""",
                  ported="the sample's profile readout is dropped. Contact Stiffness 240 on the solver is the "
                         "sample's b3World_SetContactTuning; Box3D caps it at an eighth of the substep rate, 30 Hz "
                         "at one step of 4 substeps a frame, so here it is 30 either way.",
                  try_="raise Constraint Iterations on the Box3D Solver (Box3D's substeps): the 240 Hz then "
                       "takes effect.",
                  solver={"contactstiffness": 240.0, "contactdamping": 10.0, "contactpushspeed": 3.0})


@sample("Issues", "Restitution Overshoot", SOURCE, frames=600)
def restitution_overshoot(s):
    b = Bodies(s.tag, {"ccdthreshold": -1.0})
    b.stream("floor", b.box("floor", 0.375, 0.25, 0.375), "static_body({0, -0.25, 0}, identity);",
             cd=(0.55, 0.6, 0.65), note="a small static block, its top at y = 0, narrower than the box")
    b.stream("cube", b.box("cube", 0.5, 0.5, 0.5), "body({0, 10, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="bounce 1, ccdthreshold 0.1 (Box3D's safetyFactor)", bounce=1.0, ccdthreshold=0.1)
    return finish(s, b, about="""\
A cube with bounce 1 dropped from 10 m onto a block smaller than itself. It once gained energy and
bounced higher than it fell; now each bounce stays at or below 10 m (some height turns into spin).""",
                  ported="""\
the sample's height and energy readouts are dropped: watch the cube's height (its point's P.y)
against the 10 m it started from.""")


@sample("Issues", "Slide Twist Off Center Shape", SOURCE, frames=360)
def slide_twist_off_center_shape(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("plane", b.box("plane", 10.0, 0.5, 10.0), "static_body({0, 4, 0}, axis_angle({1, 0, 0}, radians(20.0)));",
             cd=(0.55, 0.6, 0.65), note="a static slope tilted 20 degrees")
    b.stream("box", b.shifted(b.box("box", 1.0, 0.5, 1.0), (1.0, 0.5, 1.0)), """\
// b3MakeOffsetBoxHull( 1, 0.5, 1, (1, 0.5, 1) ): the box's centre 1.5 m from the body's origin, which is placed so
// the box starts where Slide Twist's does, spinning at 25 rad/s about the slope's normal.
vector4 q = axis_angle({1, 0, 0}, radians(20.0));
vector offset = qrotate(q, {1, 0.5, 1});
int pt = body(set(-offset.x, 5.0 - offset.y, -offset.z), q);
setpointattrib(0, "w", pt, 25.0 * qrotate(q, {0, 1, 0}));
""", cd=(0.9, 0.6, 0.2), friction=0.3)
    return finish(s, b, about="""\
Slide Twist with the box's shape off its body's origin: a flat box spinning at 25 rad/s on a 20 degree
slope. The body's origin is not its centre of mass, so this checks the spin is about the right point:
it should behave as Slide Twist's box does.""")


@sample("Issues", "Huge Box", SOURCE, frames=300)
def huge_box(s):
    b = Bodies(s.tag)
    b.ground(400.0)
    b.stream("cube", b.box("cube", 100.0, 100.0, 100.0), "body({0, 100, 0}, axis_angle({0, 0, 1}, 0.2));",
             cd=(0.9, 0.6, 0.2), note="b3MakeCubeHull( 100 ): a 200 m cube, tilted 0.2 rad")
    return finish(s, b, about="""\
A 200 m cube (8e9 kg) standing tilted on one edge. Its inertia tensor once overflowed; it should tip
onto its face and settle like a small one.""")


@sample("Issues", "Heightfield", SOURCE, frames=300)
def heightfield(s):
    field = height_field(s.tag, 100, 100, (1.0, 1.0, 1.0))
    b = Bodies(s.tag)
    b.stream("cylinder", b.cylinder("cylinder", 20.0, 5.0, -10.0, 24), """\
// b3CreateCylinder( 20, 5, -10, 24 ): 20 m long and 10 across, centred on the body, tilted 45 degrees about x.
body({50, 20, 50}, axis_angle({1, 0, 0}, 0.25 * PI));
""", cd=(0.5, 0.75, 0.45))
    return finish(s, b, about="""\
A large tilted cylinder dropped on a flat height field (b3CreateGrid: 100 x 100 samples 1 m apart, its
corner at the origin). Its bounds cover more than 512 of the field's triangles, which once broke the
contact; it should land, roll onto its side and settle.""",
                  ported="the height field is a HeightField SOP on a Static Object (the collider0 node).",
                  statics=[field])


@sample("Issues", "Bullet Thin Plate", SOURCE, frames=240)
def bullet_thin_plate(s):
    b = Bodies(s.tag, {"fastobject": 0, "enablesleep": 1, "rbdbullet_emit": 0})
    b.stream("hull_plate", b.shifted(b.box("hull_plate", 5.0, 0.0125, 0.5), (0.0, -0.0125, 0.0)),
             "static_body({0, 1, -1}, identity);", cd=(0.55, 0.6, 0.65),
             note="a static plate 2.5 cm thick, its top at its body's origin (y = 1)", friction=0.0)
    b.mesh("mesh_plate", """\
// The same plate as a box mesh (b3CreateBoxMesh( (0, -0.0125, 0), (5, 0.0125, 0.5) ) on a body at (0, 1, 1)).
box_mesh("mesh_plate", {0, -0.0125, 0}, {5, 0.0125, 0.5}, 0, {0, 0, 0, 1}, {0, 1, 1});
""", friction=0.0)
    b.stream("bullets", b.sphere("bullets", 0.01), """\
// One 1 cm sphere over each plate, fired down at 300 m/s and sideways at 30 (fastobject: Box3D's isBullet), and
// again every 60 frames: rbdbullet_emit 1 on frames 1, 61, 121, ... (Emit RBDs).
for (int i = 0; i < 2; ++i) {
    int pt = body(set(-4.0, 3.0, i == 0 ? -1.0 : 1.0), identity);
    setpointattrib(0, "v", pt, {30, -300, 0});
}
""", cd=(0.85, 0.45, 0.3), friction=0.0, fastobject=1, enablesleep=0, rbdbullet_emit=Every(60))
    return finish(s, b, about="""\
Two 1 cm spheres fired at 300 m/s (5 m a frame, 200 times their size) at plates 2.5 cm thick: a
box hull on the left and a box mesh on the right. Both should stop on top of their plate and slide off
its end. A new pair is fired every second.""",
                  ported="""\
the Box3D Solver SOP's Emit RBDs fires the pairs (the spheres' rbdbullet_emit is 1 every 60 frames).
The sample removes the last pair when it fires; here they fall on. Its count of shots that cross is
dropped: watch for a sphere below its plate before x = 5.""",
                  try_="fire faster: v in bullets_points (Box3D's Max Linear Speed of 400 m/s caps it).",
                  emit=True)
