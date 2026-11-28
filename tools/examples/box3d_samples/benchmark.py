"""Box3D's Benchmark samples (samples/sample_benchmark.cpp, shared/benchmarks.c). The ones that only simulate: the
pyramids, the falling boxes, the candy cups, the convex pile, the joint grid, the mesh drop and the spinner.
Each is the C++ loop in VEX, at the sizes Box3D's release build uses."""

import math

from .common import Bodies, XorShift, complex_hull_points, cos_sin, create, f32, finish, num, sample, set_parm, vec
from .joints import node

SOURCE = "sample_benchmark.cpp"

NO_SLEEP = ("the sample turns sleeping off for the world (b3World_EnableSleeping): enablesleep 0 on every body, "
            "so nothing falls asleep and the step time stays the worst case.")


@sample("Benchmark", "Large Pyramid", SOURCE, frames=600)
def large_pyramid(s):
    count = 100
    b = Bodies(s.tag, {"enablesleep": 0})
    b.ground(400.0, note="b3MakeBoxHull( 400, 1, 400 ) at (0, -1, 0): the same as Sample::AddGroundBox( 400 )")
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), f"""\
// A pyramid of unit boxes, {count} on the bottom row: {count * (count + 1) // 2} bodies.
float h = 0.5, shift = h;
for (int i = 0; i < {count}; ++i) {{
    float y = (2.0 * i + 1.0) * shift;
    for (int j = i; j < {count}; ++j) {{
        float x = (i + 1.0) * shift + 2.0 * (j - i) * shift - h * {count};
        body(set(x, y, 0.0), identity);
    }}
}}
""", cd=(0.9, 0.6, 0.2), note="density 100", density=100.0)
    return finish(s, b, about=f"""\
A pyramid of {count * (count + 1) // 2} unit boxes ({count} on the bottom row), the stacking solver's worst
case: the pile is stable only if the contact solver converges. Box3D's own comment: the 80 box version
falls over after 1000 steps.""", ported=NO_SLEEP, try_="raise count in benchmark.py, or the density.")


@sample("Benchmark", "Wide Pyramid", SOURCE, frames=600)
def wide_pyramid(s):
    height = 15
    b = Bodies(s.tag)
    b.ground(100.0)
    b.stream("boxes", b.box("boxes", 0.975, 0.975, 0.975), f"""\
// A stepped pyramid of 2 m boxes (each 0.975 half extent), {height} layers, every other layer shifted half a box
// and 0.5 m apart vertically.
int height = {height};
float boxSize = 2.0, separation = 0.5, half = 0.5 * boxSize;
for (int i = 0; i < height; ++i)
    for (int j = i / 2; j < height - (i + 1) / 2; ++j)
        for (int k = i / 2; k < height - (i + 1) / 2; ++k) {{
            float x = -height + boxSize * j + ((i & 1) ? half : 0.0);
            float y = 1.0 + (boxSize + separation) * i;
            float z = -height + boxSize * k + ((i & 1) ? half : 0.0);
            body(set(x, y, z), identity);
        }}
""", cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="""\
A stepped pyramid of 2 m boxes, 15 layers, each 0.5 m above the one below and one box narrower on every side
(every other layer shifted half a box), which fall onto each other and settle.""")


@sample("Benchmark", "Many Pyramids", SOURCE, frames=600)
def many_pyramids(s):
    base, extent, rows, columns = 10, 0.5, 14, 14
    ground = extent * columns * (base + 1.0)
    b = Bodies(s.tag, {"enablesleep": 0})
    b.ground(ground)
    b.stream("boxes", b.box("boxes", extent, extent, extent), f"""\
// {rows} x {columns} pyramids of {base} boxes on the bottom row, {rows * columns * base * (base + 1) // 2} bodies.
int baseCount = {base};
float extent = {num(extent)}, groundExtent = {num(ground)};
float baseWidth = 2.0 * extent * baseCount;
float baseZ = -groundExtent + 2.0 * extent;
float deltaZ = 2.0 * (groundExtent - 2.0 * extent) / ({rows} - 1.0);
for (int row = 0; row < {rows}; ++row) {{
    for (int column = 0; column < {columns}; ++column) {{
        float centerX = -groundExtent + column * (baseWidth + 2.0 * extent) + 2.0 * extent;
        for (int i = 0; i < baseCount; ++i) {{
            float y = (2.0 * i + 1.0) * extent;
            for (int j = i; j < baseCount; ++j) {{
                float x = (i + 1.0) * extent + 2.0 * (j - i) * extent + centerX - 0.5;
                body(set(x, y, baseZ), identity);
            }}
        }}
    }}
    baseZ += deltaZ;
}}
""", cd=(0.9, 0.6, 0.2), note="density 100, no sleeping", density=100.0)
    return finish(s, b, about=f"""\
A {rows} x {columns} field of small pyramids ({base} boxes on the bottom row), {rows * columns * base * (base + 1) // 2}
boxes on one ground box: many separate islands, which Box3D solves in parallel.""", ported=NO_SLEEP)


@sample("Benchmark", "Joint Grid", SOURCE, frames=300)
def joint_grid(s):
    n = 100
    b = Bodies(s.tag, {"enablesleep": 0, "collision_category": 2, "collision_mask": -3})
    b.stream("balls", b.sphere("balls", 0.4), f"""\
// A {n} x {n} grid of spheres hanging in the xy plane: column k at x = k, row i at y = -i. The top row
// (i = 0) is static.
for (int k = 0; k < {n}; ++k)
    for (int i = 0; i < {n}; ++i) {{
        vector p = set(float(k), -float(i), 0.0);
        if (i == 0) static_body(p, identity); else body(p, identity);
    }}
""", cd=(0.4, 0.65, 0.9), note="radius 0.4, category 2 and mask ~2: the spheres never collide with each other",
             collision_category=2, collision_mask=-3, enablesleep=0)
    joint_vex = f"""\
// A spherical joint from each sphere to the one above it (half a unit up) and to the one to its left (half a unit
// left), where their surfaces would touch: the anchors of b3SphericalJointDef at (0, -0.5, 0) / (0, 0.5, 0) and
// (0.5, 0, 0) / (-0.5, 0, 0).
for (int k = 0; k < {n}; ++k)
    for (int i = 0; i < {n}; ++i) {{
        string name = sprintf("balls%d", k * {n} + i);
        if (i > 0) {{
            vector p = set(float(k), -float(i) + 0.5, 0.0);
            joint(sprintf("balls%d", k * {n} + i - 1), p, name, p, "grid");
        }}
        if (k > 0) {{
            vector p = set(float(k) - 0.5, -float(i), 0.0);
            joint(sprintf("balls%d", (k - 1) * {n} + i), p, name, p, "grid");
        }}
    }}
"""
    return finish(s, b, about=f"""\
A {n} x {n} cloth of spheres joined by {2 * n * (n - 1)} spherical joints, hung from its top row (static): a
swinging net, with no ground. A heavy joint benchmark in Box3D, so the scene is slow to play back.""",
                  ported="the spheres never collide with each other (category 2, mask ~2), as in the sample.",
                  try_="a smaller grid: n in benchmark.py.",
                  joints=[node("spherical", "grid")], lines=[joint_vex])


@sample("Benchmark", "Falling Boxes", SOURCE, frames=400)
def falling_boxes(s):
    n = 50
    b = Bodies(s.tag)
    b.ground(100.0)
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), f"""\
// {n} layers of 8 x 8 boxes spaced 2 m apart, the lowest 2.5 m up, all falling at once.
float a = 0.5;
for (int i = 0; i < {n}; ++i)
    for (int j = 0; j < 8; ++j)
        for (int k = 0; k < 8; ++k)
            body(set(-16.0 * a + 4.0 * a * j, 4.0 * a * i + 5.0 * a, -16.0 * a + 4.0 * a * k), identity);
""", cd=(0.9, 0.6, 0.2))
    return finish(s, b, about=f"""\
{n * 64} unit boxes in a grid of 8 x 8 columns {n} high, all dropped at once, which land on each other and
the ground in one tumbling pile.""")


@sample("Benchmark", "Candy Cups", SOURCE, frames=600)
def candy_cups(s):
    sides, radius1, height1, radius2, height2 = 8, 0.6, 0.0, 0.95, 1.0
    points = []
    for i in range(sides):
        c, sn = cos_sin(i * 2.0 * math.pi / sides)
        points.append((f32(radius1 * c), f32(height1), f32(radius1 * sn)))
        points.append((f32(radius2 * c), f32(height2), f32(radius2 * sn)))
    b = Bodies(s.tag)
    b.ground(60.0)
    b.stream("cups", b.hull("cups", points, note="CreateConvex( 0.6, 0, 0.95, 1 ): a ring of 8 points of radius 0.6 "
                                                  "at the body's origin and one of radius 0.95 a metre above"),
             """\
// 16 layers of 16 x 16 cups 2.5 m apart, one metre a layer.
for (int i = 0; i < 16; ++i)
    for (int j = 0; j < 16; ++j)
        for (int k = 0; k < 16; ++k)
            body(set(-10.0 + 2.5 * j, 1.0 * i, -10.0 + 2.5 * k), identity);
""", cd=(0.85, 0.45, 0.3))
    return finish(s, b, about="""\
4096 small convex cups (a hull of two octagons, 0.6 and 0.95 in radius, a metre apart) in 16 layers a metre
apart, which fall into one pile: a test of many small hulls.""")


@sample("Benchmark", "Convex Pile", SOURCE, frames=600)
def convex_pile(s):
    points = complex_hull_points(2.0)
    b = Bodies(s.tag)
    b.ground(250.0)
    b.stream("rocks", b.hull("rocks", points, note="b3CreateComplexHull( 2 ): the hull of 32 points of radius 2 "
                                                   "from a fixed LCG (seed 42), the same for every rock"),
             """\
// 80 layers of 8 x 8 rocks, 4 m apart: a grid tall enough to collapse into a pile.
float amplitude = 2.0, scatter = 2.0 * amplitude;
for (int layer = 0; layer < 80; ++layer)
    for (int z = 0; z < 8; ++z)
        for (int x = 0; x < 8; ++x)
            body(set((x - 4.0) * scatter, amplitude + 2.0 * amplitude * layer, (z - 4.0) * scatter), identity);
""", cd=(0.55, 0.6, 0.65))
    return finish(s, b, about="""\
5120 large convex rocks (each the hull of 32 random points on a sphere of radius 2, the same rock every
time) stacked 80 layers high in a grid and dropped to collapse into one pile. Ported from PEEL.""")


@sample("Benchmark", "Mesh Drop", SOURCE, frames=300)
def mesh_drop(s):
    cells, border, grid = 120, 10, 100
    spacing = (cells - 2 * border) / grid
    offset = 0.5 * spacing * (1 - grid)
    b = Bodies(s.tag, {"collision_category": -1, "collision_mask": -1, "rollingresistance": 0.0})
    b.mesh("wave", f"""\
// b3CreateWaveMesh( {cells}, {cells}, 1, 0.5, 0.1, 0.2 ): a {cells} m square wave mesh centred on the origin.
wave_mesh("wave", {cells}, {cells}, 1.0, 0.5, 0.1, 0.2, {{1, 1, 1}}, {{0, 0, 0}});
""", collision_category=1)
    rng = XorShift(3963634789)
    starts = [(rng.vec3(-1.0, 1.0), rng.vec3(-5.0, 5.0)) for _ in range(grid * grid)]
    speeds = create(b.geo, "attribwrangle", "drop_velocities")
    set_parm(speeds, "class", 0)
    set_parm(speeds, "snippet", f"""\
// The starting velocities: per body RandomVec3Uniform( -1, 1 ) then ( -5, 5 ), the samples' XorShift32 from seed
// 3963634789 (g_randomSeed in CreateMeshDropBenchmark), computed in Python by box3d_samples.common.XorShift.
v[]@drop_v = array({", ".join(vec(v) for v, _ in starts)});
v[]@drop_w = array({", ".join(vec(w) for _, w in starts)});
""")
    boxes = b.stream("boxes", b.box("boxes", 0.02, 0.2, 0.04), f"""\
// A {grid} x {grid} grid {num(spacing)} m apart over the mesh (a {border} m border left), 5 m up, each with its
// starting velocities from input 1. They collide with the mesh only (collision_category 2, collision_mask 1).
vector vs[] = detail(1, "drop_v"), ws[] = detail(1, "drop_w");
int index = 0;
for (int i = 0; i < {grid}; ++i) for (int j = 0; j < {grid}; ++j) {{
    int pt = body(set({num(offset)} + {num(spacing)} * i, 5.0, {num(offset)} + {num(spacing)} * j), identity);
    setpointattrib(0, "v", pt, vs[index]);
    setpointattrib(0, "w", pt, ws[index]);
    setpointattrib(0, "Cd", pt, hsvtorgb(set(index / {float(grid * grid)}, 0.55, 0.95)));
    index += 1;
}}
""", note="b3MakeBoxHull( 0.02, 0.2, 0.04 ), rolling resistance 0.1", rollingresistance=0.1, collision_category=2,
                     collision_mask=1)
    boxes.inputs()[1].setInput(1, speeds)
    return finish(s, b, about=f"""\
{grid * grid} thin boxes thrown down from 5 m with random velocities onto a {cells} m wave mesh. They do not
collide with each other, only with the mesh: a test of many small bodies against one large mesh. None
should fall through it (below y = -1).""")


@sample("Benchmark", "Spinner", SOURCE, frames=600)
def spinner(s):
    radius, half_depth, y_center, segments = 12.0, 1.0, 12.0, 90
    half_length = 0.5 * radius
    b = Bodies(s.tag, {"enablesleep": 1})
    b.mesh("drum", f"""\
// An inward facing cylinder capped on both ends, axis along z: {segments} wall segments, each wall quad two triangles,
// each cap a fan from its centre. Every triangle faces the axis, which holds the contents in.
int segments = {segments};
float radius = {num(radius)}, halfDepth = {num(half_depth)}, yCenter = {num(y_center)};
vector p[];
for (int i = 0; i < segments; ++i) {{
    vector2 cs = cossin(2.0 * PI * i / segments);
    float x = radius * cs[0], y = radius * cs[1] + yCenter;
    push(p, set(x, y, -halfDepth));
    push(p, set(x, y, halfDepth));
}}
push(p, set(0.0, yCenter, -halfDepth));
push(p, set(0.0, yCenter, halfDepth));
int v[] = points(p);
int centerNegative = 2 * segments, centerPositive = 2 * segments + 1;
for (int i = 0; i < segments; ++i) {{
    int j = (i + 1) % segments;
    tri("drum", v[2 * i], v[2 * i + 1], v[2 * j + 1]);
    tri("drum", v[2 * i], v[2 * j + 1], v[2 * j]);
    tri("drum", v[centerNegative], v[2 * i], v[2 * j]);
    tri("drum", v[centerPositive], v[2 * j + 1], v[2 * i + 1]);
}}
""", note="CreateInwardCylinderMesh( 12, 1, 12, 90 ), friction 0.1", friction=0.1)
    b.stream("paddle", b.box("paddle", 0.3, half_length + 0.1, half_depth + 0.1),
             f"body({{0, {num(y_center - half_length)}, 0}}, identity);", cd=(0.9, 0.6, 0.2),
             note="a bar 12.2 m long on a motor at its middle, friction 0, no sleeping", friction=0.0, enablesleep=0)
    b.stream("balls", b.sphere("balls", 0.2), f"""\
// 1500 small balls in a grid from the cylinder's low side, filling z, then x, then y, as the sample does.
float spacing = 0.55, yStart = 2.0, radius = 0.2;
float chordHalfWidth = sqrt({num(radius * radius)} - ({num(y_center)} - yStart) * ({num(y_center)} - yStart));
float xLimit = chordHalfWidth - 2.0 * spacing, zLimit = {num(half_depth)} - 2.0 * radius;
float x = -xLimit, y = yStart, z = -zLimit;
for (int i = 0; i < 1500; ++i) {{
    body(set(x, y, z), identity);
    z += spacing;
    if (z > zLimit) {{
        z = -zLimit;
        x += spacing;
        if (x > xLimit) {{
            x = -xLimit;
            y += spacing;
        }}
    }}
}}
""", cd=(0.4, 0.65, 0.9), note="radius 0.2, density 0.25, friction 0.1, bounce 0.1", density=0.25, friction=0.1,
             bounce=0.1)
    return finish(s, b, about="""\
A paddle on a motor (8 rad/s, unlimited torque) churning 1500 small balls inside a closed cylinder of
radius 12, which is a mesh with its triangles facing the axis. The paddle's tip scrapes the floor.""",
                  ported="the motor joins the paddle to the drum's body, as in the sample, not to the world: a "
                         "joint's bodies do not collide, and the paddle is longer and wider than the drum (its "
                         "ends would catch on the floor and caps).",
                  try_="lower Max Motor Torque on the motor node until the balls stall the paddle.",
                  joints=[node("revolute", "motor", enable_motor=1, motor_speed=math.degrees(8.0),
                               max_motor_torque=1.0e9)],
                  lines=[("drum", (0, y_center - half_length, 0), "paddle0", (0, y_center - half_length, 0), "motor")])
