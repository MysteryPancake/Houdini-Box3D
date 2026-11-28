"""Box3D's Continuous samples (samples/sample_continuous.cpp): fast bodies against thin and static shapes."""

from .common import Bodies, Every, XorShift, create, finish, num, random_vec3, sample, set_parm, vec

SOURCE = "sample_continuous.cpp"


@sample("Continuous", "Thin Wall", SOURCE, frames=180)
def thin_wall(s):
    b = Bodies(s.tag)
    b.ground(40.0)
    b.stream("wall", b.box("wall", 10.0, 0.1, 10.0), "static_body({0, 10, 0}, axis_angle({1, 0, 0}, radians(90.0)));",
             cd=(0.55, 0.6, 0.65), note="a static wall 20 x 20 and 0.2 thick, standing in the xy plane")
    b.stream("sphere", b.sphere("sphere", 0.1), """\
int pt = body({-5, 10, 20}, identity);
setpointattrib(0, "v", pt, {0, 0, -180});
setpointattrib(0, "w", pt, {20, 0, 0});
""", cd=(0.85, 0.45, 0.3), rollingresistance=0.1)
    b.stream("capsule", b.capsule("capsule", 0.3, 0.1), """\
// b3Capsule { (-0.3, 0, 0), (0.3, 0, 0), 0.1 }: built along y, turned to x.
int pt = body({0, 10, 20}, along_x(identity));
setpointattrib(0, "v", pt, {0, 0, -180});
setpointattrib(0, "w", pt, {20, -5, 0});
""", cd=(0.4, 0.65, 0.9), rollingresistance=0.1)
    b.stream("box", b.box("box", 0.4, 0.1, 0.1), """\
int pt = body({5, 10, 20}, identity);
setpointattrib(0, "v", pt, {0, 0, -180});
setpointattrib(0, "w", pt, {20, 5, 0});
""", cd=(0.9, 0.6, 0.2), rollingresistance=0.1)
    return finish(s, b, about="""\
A small sphere, capsule and stick fired at 180 m/s (3 m a frame) at a wall 0.2 m thick, spinning as
they go. Continuous collision stops all three at the wall; they drop to the ground in front of it.""")


@sample("Continuous", "Bounce House", SOURCE, frames=600)
def bounce_house(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(10.0)
    b.stream("walls", b.box("walls", 0.1, 5.0, 10.0), """\
// Four static walls 10 m high around the ground (in Box3D one static body at (0, -1, 0) holding four box hulls).
static_body({10, 4, 0}, identity);
static_body({-10, 4, 0}, identity);
vector4 q = axis_angle({0, 1, 0}, radians(90.0));
static_body({0, 4, -10}, q);
static_body({0, 4, 10}, q);
""", cd=(0.55, 0.6, 0.65), note="b3MakeTransformedBoxHull( 0.1, 5, 10 ) on each side")
    b.stream("ball", b.sphere("ball", 0.5), """\
int pt = body({-8, 4, 0}, identity);
setpointattrib(0, "v", pt, {120, 0, 120});
""", cd=(0.85, 0.45, 0.3), note="no gravity, no friction, bounce 1", gravityscale=0.0, friction=0.0, bounce=1.0)
    return finish(s, b, about="""\
A ball with no gravity, friction or energy loss (bounce 1, mixed as the maximum) fired at 170 m/s
across a walled pen. It ricochets off the walls without passing through them, about 3 m a frame.""",
                  ported="""\
the walls are four static boxes (Box3D: one static body of four hulls). The sample makes a box mesh
it never uses; it is left out.""")


@sample("Continuous", "Spinning Stick", SOURCE, frames=240)
def spinning_stick(s):
    omega = random_vec3(1, (-50.0, -50.0, -50.0), (50.0, 50.0, 50.0))[0]
    b = Bodies(s.tag)
    b.ground(10.0)
    b.stream("wall", b.box("wall", 0.125, 0.5, 10.0), "static_body({0, 0.5, 0}, identity);", cd=(0.55, 0.6, 0.65),
             note="a low static wall, 0.25 thick and 1 high")
    b.stream("stick", b.box("stick", 2.0, 0.1, 0.1), f"""\
// Dropped at 100 m/s, spinning at RandomVec3( -50, 50 ) rad/s: the sample's first random vector.
int pt = body({{0, 20, 0.5}}, identity);
setpointattrib(0, "v", pt, {{0, -100, 0}});
setpointattrib(0, "w", pt, {vec(omega)});
""", cd=(0.9, 0.6, 0.2), rollingresistance=0.1)
    return finish(s, b, about=f"""\
A 4 m stick thrown down at 100 m/s and spinning at ({omega[0]:.1f}, {omega[1]:.1f}, {omega[2]:.1f}) rad/s
onto a low wall. Its ends sweep fast, but it must not pass through the wall or the ground.""",
                  try_="other spins: change w in stick_points.")


@sample("Continuous", "Needle Mesh", SOURCE, frames=180)
def needle_mesh(s):
    b = Bodies(s.tag)
    b.mesh("needles", """\
// NeedleMesh::CreateNeedle( height, 0.1, center, 8 ): an open cone of 8 triangles, its tip `height` up.
void needle(string nm; float height; float radius; vector center; int slices) {
    vector v[] = array(set(0, height, 0) + center);
    float alpha = 0.0;
    for (int i = 1; i <= slices; ++i) {
        vector2 cs = cossin(alpha);
        push(v, set(radius * cs[0], 0, radius * cs[1]) + center);
        alpha += 2.0 * PI / slices;
    }
    int q[] = points(v);
    int i1 = slices;
    for (int i = 0; i < slices; ++i) { tri(nm, q[0], q[i + 1], q[i1]); i1 = i + 1; }
}
// Four needles of slightly different heights: one static mesh each.
needle("needle1", 0.99, 0.1, {0.2, 0, 0.2}, 8);
needle("needle2", 1.01, 0.1, {0.2, 0, -0.2}, 8);
needle("needle3", 0.98, 0.1, {-0.2, 0, -0.2}, 8);
needle("needle4", 1.02, 0.1, {-0.2, 0, 0.2}, 8);
""", cd=(0.55, 0.6, 0.65))
    b.stream("plate", b.box("plate", 0.3, 0.01, 0.3), """\
int pt = body({0, 5, 0}, identity);
setpointattrib(0, "v", pt, {0, -10, 0});
""", cd=(0.9, 0.6, 0.2), note="a thin plate 0.6 x 0.02 x 0.6")
    return finish(s, b, about="""\
A thin plate thrown down onto four needle tips of slightly different heights (open cone meshes, no
ground). It lands on the tips without them poking through.""")


# shared/mesh_drop.c: a grid of small bodies dropped with random velocities onto a walled wave mesh. The sample seeds
# each generation from the clock; the port uses one fixed seed.
MESH_DROP_SEED = 1
MESH_DROP_GRID = 32  # MESH_DROP_GRID_COUNT in a Release build
# The wave: cell count, cell width, row and column frequency, amplitude (smooth, then rough).
MESH_DROP_WAVES = {False: (40, 1.0, 0.1, 0.2, 0.5), True: (640, 0.03125, 4.7, 5.9, 0.015)}


def _mesh_drop(s, rough):
    count, cell, row_f, column_f, amplitude = MESH_DROP_WAVES[rough]
    extent = 0.5 * count * cell
    b = Bodies(s.tag, {"collision_category": -1, "collision_mask": -1})
    wave = f"{count}, {count}, {num(cell)}, {num(amplitude)}, {num(row_f)}, {num(column_f)}"
    b.mesh("wave", f"""\
// CreateMeshDropMesh: b3CreateWaveMesh( {wave} ).
wave_mesh("wave", {wave}, {{1, 1, 1}}, {{0, 0, 0}});
""", collision_category=1)
    b.stream("walls_x", b.box("walls_x", extent, 1.0, 0.1), f"""\
// CreateMeshDropGround's walls (box hulls on the mesh's static body): along x at z = +-{num(extent)}.
static_body({{0, 1, {num(-extent)}}}, identity);
static_body({{0, 1, {num(extent)}}}, identity);
""", cd=(0.55, 0.6, 0.65), collision_category=1)
    b.stream("walls_z", b.box("walls_z", 0.1, 1.0, extent), f"""\
static_body({{{num(-extent)}, 1, 0}}, identity);
static_body({{{num(extent)}, 1, 0}}, identity);
""", cd=(0.55, 0.6, 0.65), collision_category=1)

    rng = XorShift(MESH_DROP_SEED)
    starts = [(rng.vec3(-1.0, 1.0), rng.vec3(-5.0, 5.0)) for _ in range(MESH_DROP_GRID * MESH_DROP_GRID)]
    speeds = create(b.geo, "attribwrangle", "drop_velocities")
    set_parm(speeds, "class", 0)
    set_parm(speeds, "snippet", f"""\
// CreateMeshDropBodies' starting velocities: per body RandomVec3Uniform( -1, 1 ) then ( -5, 5 ), XorShift32 from seed
// {MESH_DROP_SEED} (mesh_drop.c's RandomFloatRange), computed in Python by box3d_samples.common.XorShift.
v[]@drop_v = array({", ".join(vec(v) for v, _ in starts)});
v[]@drop_w = array({", ".join(vec(w) for _, w in starts)});
""")
    grid = f"""\
// A {MESH_DROP_GRID} x {MESH_DROP_GRID} grid 0.5 apart, 5 m up, each with its starting velocities from input 1. They
// collide with the mesh and walls only (collision_category 2, collision_mask 1: the sample's Collide off).
vector vs[] = detail(1, "drop_v"), ws[] = detail(1, "drop_w");
int index = 0;
for (int i = 0; i < {MESH_DROP_GRID}; ++i) for (int j = 0; j < {MESH_DROP_GRID}; ++j) {{
    int pt = body(set(0.5 * (i - 0.5 * {MESH_DROP_GRID}), 5.0, 0.5 * (j - 0.5 * {MESH_DROP_GRID})), identity);
    setpointattrib(0, "v", pt, vs[index]);
    setpointattrib(0, "w", pt, ws[index]);
    setpointattrib(0, "Cd", pt, hsvtorgb(set(index / {float(MESH_DROP_GRID * MESH_DROP_GRID)}, 0.55, 0.95)));
    index += 1;
}}
"""
    shapes = [("box", b.box("box", 0.02, 0.2, 0.04), "MakeDropBox: b3MakeBoxHull( 0.02, 0.2, 0.04 )", 0.1),
              ("capsule", b.capsule("capsule", 0.2, 0.05), "b3Capsule { (0, -0.2, 0), (0, 0.2, 0), 0.05 }", 0.4),
              ("cylinder", b.cylinder("cylinder", 0.4, 0.05, 0.0, 6), "b3CreateCylinder( 0.4, 0.05, 0, 6 )", 0.1),
              ("sphere", b.sphere("sphere", 0.05), "a sphere of radius 0.05", 0.4)]
    streams = []
    for token, shape, note, rolling in shapes:
        copy = b.stream(token, shape, grid, note=note, merged=False, rollingresistance=rolling,
                        collision_category=2, collision_mask=1)
        copy.inputs()[1].setInput(1, speeds)
        streams.append((token, token.capitalize(), copy))
    b.choice("shape", "Shape", streams, default=0)
    ground = "rough wave mesh, its bumps about 20 cm across" if rough else "wave mesh"
    return finish(s, b, about=f"""\
{MESH_DROP_GRID * MESH_DROP_GRID} small bodies (boxes by default; Shape on the bodies object picks capsules, cylinders
or spheres) dropped from 5 m with random velocities onto a walled {ground}. They do not collide
with each other, only with the mesh. None should tunnel through it (fall below y = -2).""", ported="""\
the sample seeds each generation from the clock and counts contacts at the manifold cap, the deepest sink
and tunnels; here the seed is fixed and the readouts dropped. Its Generate buttons are a rebuild of the
scene (MESH_DROP_SEED in continuous.py).""", try_="the other shapes: Shape on the bodies object's Sample tab.")


@sample("Continuous", "Mesh Drop", SOURCE, frames=300)
def mesh_drop(s):
    return _mesh_drop(s, rough=False)


@sample("Continuous", "Rough Mesh Drop", SOURCE, frames=300)
def rough_mesh_drop(s):
    return _mesh_drop(s, rough=True)


@sample("Continuous", "Hump Mesh", SOURCE, frames=180)
def hump_mesh(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.mesh("hump", """\
// HumpMesh::CreateHump( 8 ): four triangles over an 8 x 16 patch, its middle row raised 0.4.
float w = 8.0;
vector v[];
float x = -0.5 * w;
for (int ix = 0; ix <= 1; ++ix) {
    float z = -w;
    for (int iz = 0; iz <= 2; ++iz) { push(v, set(x, iz == 1 ? 0.05 * w : 0.0, z)); z += w; }
    x += w;
}
int q[] = points(v);
for (int iz = 0; iz < 2; ++iz) {
    int i1 = iz, i2 = i1 + 1, i3 = i2 + 3, i4 = i3 - 1;
    tri("hump", q[i1], q[i2], q[i3]);
    tri("hump", q[i3], q[i4], q[i1]);
}
""", cd=(0.55, 0.6, 0.65))
    b.stream("plate", b.box("plate", 0.5, 0.05, 1.0), """\
int pt = body({0, 5, 0}, identity);
setpointattrib(0, "v", pt, {0, -50, 0});
""", cd=(0.9, 0.6, 0.2), note="a thin plate 1 x 0.1 x 2, across the ridge")
    return finish(s, b, about="""\
A thin plate thrown down at 50 m/s onto the ridge of a low mesh hump. Box3D's comment: clustering
contacts by the manifold normal can clip or tunnel here. It should land on the ridge and slide off.""")


@sample("Continuous", "Is Fast", SOURCE, frames=300)
def is_fast(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(40.0)
    b.stream("posts", b.box("posts", 0.5, 10.0, 0.5), """\
// Three 20 m posts with no gravity, spinning at 4 rad/s about z, y and x.
vector at[] = array({-12, 20, 0}, {0, 20, 0}, {12, 20, 0});
vector spin[] = array({0, 0, 4}, {0, 4, 0}, {4, 0, 0});
for (int i = 0; i < 3; ++i) {
    int pt = body(at[i], identity);
    setpointattrib(0, "w", pt, spin[i]);
    setpointattrib(0, "Cd", pt, hsvtorgb(set(i / 3.0, 0.6, 0.95)));
}
""", gravityscale=0.0)
    return finish(s, b, about="""\
Three long posts spinning in the air at 4 rad/s: their tips move 40 m/s, fast enough for Box3D's
fast-body test, though they hit nothing. They should keep spinning in place.""",
                  ported="the sample's option to make them capsules is off in Box3D too.")


@sample("Continuous", "Stall", SOURCE, frames=180)
def stall(s):
    b = Bodies(s.tag, {"fastobject": 0, "rbdbullet_emit": 0})
    b.ground(500.0)
    b.mesh("torus", """\
// b3CreateTorusMesh( 200, 200, 2, 1 ) at y = 2: 80000 triangles, the ring standing in the xy plane.
torus_mesh("torus", 200, 200, 2.0, 1.0, {0, 2, 0});
""", cd=(0.55, 0.6, 0.65))
    b.polygons("rock", """\
// b3CreateRock( 0.25 ): the hull of 10 points on a Fibonacci lattice, the body at (0, 1, -10).
int n = 10;
float phi = (1.0 + sqrt(5.0)) / 2.0;
vector2 cs = set(1.0, 0.0), d = cossin(2.0 * PI / phi);
vector p[];
for (int i = 0; i < n; ++i) {
    float z = 1.0 - (2.0 * i + 1.0) / n, rxy = sqrt(1.0 - z * z);
    push(p, 0.25 * set(rxy * cs[0], rxy * cs[1], z) + {0, 1, -10});
    cs = set(d[0] * cs[0] - d[1] * cs[1], d[1] * cs[0] + d[0] * cs[1]);
}
hull_points("rock", p);
""", cd=(0.6, 0.5, 0.4), v=(0.0, 0.0, 600.0), w=(0.0, 0.0, 20.0), fastobject=1, rbdbullet_emit=Every(60),
               note="fired again every 60 frames: rbdbullet_emit 1 on frames 1, 61, 121, ... (Emit RBDs)")
    return finish(s, b, about="""\
A small rock fired at 600 m/s with fastobject on (Box3D's isBullet) at a dense torus mesh of 80000
triangles. Box3D's comment: continuous collision against a dense mesh can stall the solver, because
it queries along the whole sweep. The speed is capped at Max Linear Speed (400 m/s, Box3D's default).
A new rock is fired every second.""",
                  ported="""\
the sample fires one rock and another at each press of its Launch button, removing the last. Here the
Box3D Solver SOP's Emit RBDs fires one every 60 frames (the rock's rbdbullet_emit), and the earlier
rocks fly on.""",
                  try_="fire more often: Every(30) in place of Every(60) in continuous.py, or animate "
                       "rbdbullet_emit in rock_attributes.",
                  emit=True)


@sample("Continuous", "Safety Factor", SOURCE, frames=120)
def safety_factor(s):
    extent, height, safety = 0.5, 0.3, 0.1
    b = Bodies(s.tag, {"ccdthreshold": -1.0})
    b.ground(20.0)
    b.stream("cube", b.box("cube", extent, extent, extent), f"body({{0, {num(height + extent)}, 0}}, identity);",
             cd=(0.9, 0.6, 0.2),
             note=f"dropped from {num(height)} m, ccdthreshold {num(safety)} (Box3D's safetyFactor)",
             ccdthreshold=safety)
    return finish(s, b, about=f"""\
A cube dropped from {height:g} m with ccdthreshold {safety:g} (Box3D's safetyFactor): continuous collision
takes over once a frame's motion passes {safety:g} times its size, so it lands with less overlap than
plain collision leaves.""",
                  ported="the sample's overlap readout is dropped, and its Drop button is the scene's start.",
                  try_="ccdthreshold 0 or 1 in cube_points, or a higher drop, and step through the landing.")
