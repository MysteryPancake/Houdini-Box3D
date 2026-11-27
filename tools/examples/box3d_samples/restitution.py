"""Box3D's Restitution samples (samples/sample_restitution.cpp). Bounce mixes as the maximum, Box3D's rule
(box3d_world), so a bounce on one side of a contact is the contact's bounce. The samples' energy and
rebound readouts are not reproduced."""

from .common import Bodies, finish, num, sample

SOURCE = "sample_restitution.cpp"
IMPACT_SPEED = 5.0  # m_impactSpeed of Sphere Stack and Box Restitution
STEP = 1.0 / 60.0


@sample("Restitution", "Varying", SOURCE, frames=600)
def varying(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("spheres", b.sphere("spheres", 0.5), """\
// 40 spheres dropped from 40 m, bounce rising from 0 (left) to 1 (right). No friction.
int count = 40;
float dr = 1.0 / (count - 1);
float x = -1.0 * (count - 1);
for (int i = 0; i < count; ++i) {
    int pt = body(set(x, 40.0, 0.0), identity);
    setpointattrib(0, "bounce", pt, i * dr);
    setpointattrib(0, "Cd", pt, hsvtorgb(set(0.66 * (1.0 - i * dr), 0.6, 0.95)));
    x += 2.0;
}
""", density=1.0, friction=0.0)
    return finish(s, b, about="""\
40 spheres dropped from 40 m with bounce 0 to 1. Bounce 1 should come back to its starting height
(the sample draws that line at 40.5).""", ported="the sample's Box option (cubes) is not built; this is its default, Sphere.")


@sample("Restitution", "Single Box Restitution", SOURCE, frames=600)
def single_box_restitution(s):
    b = Bodies(s.tag, {"ccdthreshold": -1.0})
    b.ground(4.0)
    b.stream("cube", b.box("cube", 0.5, 0.5, 0.5), "body({0, 5, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="b3MakeCubeHull( 0.5 ), bounce 1, safetyFactor 0.01 (ccdthreshold)",
             density=1.0, bounce=1.0, friction=0.0, ccdthreshold=0.01)
    return finish(s, b, about="""\
A cube with bounce 1 dropped flat from 5 m. Its first bounce comes back to 4.98 m; each bounce loses a
little, as in Box3D, and it must never gain energy or tip.""")


@sample("Restitution", "Single Sphere Restitution", SOURCE, frames=600)
def single_sphere_restitution(s):
    b = Bodies(s.tag, {"ccdthreshold": -1.0})
    b.ground(4.0)
    b.stream("sphere", b.sphere("sphere", 0.5), "body({0, 10, 0}, identity);", cd=(0.85, 0.45, 0.3),
             note="a sphere of radius 0.5, bounce 1, safetyFactor 0.01 (ccdthreshold)",
             density=1.0, bounce=1.0, friction=0.0, ccdthreshold=0.01)
    return finish(s, b, about="A sphere with bounce 1 dropped from 10 m. Its first bounce comes back to 9.94 m; each "
                              "bounce loses a little, as in Box3D.")


@sample("Restitution", "Sphere Stack Restitution", SOURCE, frames=240)
def sphere_stack_restitution(s):
    start = 0.5 + 1.0 * 2 + 0.5 * IMPACT_SPEED * STEP
    b = Bodies(s.tag, {"enablesleep": 1})
    b.ground(40.0, friction=0.0)
    b.stream("stack", b.sphere("stack", 0.5), """\
// Two touching spheres resting on the ground, bounce 0.
for (int i = 0; i < 2; ++i)
    body(set(0.0, 0.5 + 1.0 * i, 0.0), identity);
""", cd=(0.6, 0.6, 0.65), friction=0.0, enablesleep=0)
    b.stream("impactor", b.sphere("impactor", 0.5), f"""\
// A sphere of bounce 1 coming down on them at {num(IMPACT_SPEED)} m/s, half a step above.
int pt = body({{0, {num(start)}, 0}}, identity);
setpointattrib(0, "v", pt, {{0, {num(-IMPACT_SPEED)}, 0}});
""", cd=(0.85, 0.45, 0.3), friction=0.0, bounce=1.0, enablesleep=0)
    return finish(s, b, gravity=(0.0, 0.0, 0.0), about=f"""\
No gravity. A sphere of bounce 1 hits a stack of two spheres of bounce 0 at {num(IMPACT_SPEED)} m/s. How
much of the bounce reaches through the stack is what the sample measures.""",
                  ported="the sample's Gravity, Stack Count and Restitution controls stay at their defaults.",
                  try_="turn on Bounce Propagation on the Box3D Solver and compare the rebound.")


@sample("Restitution", "Restitution Propagation", SOURCE, frames=300)
def restitution_propagation(s):
    b = Bodies(s.tag, {"enablesleep": 1})
    b.ground(40.0)
    b.stream("crates", b.box("crates", 0.5, 0.5, 0.5), """\
// Column 0 has no crate, column 1 one, column 2 two.
for (int column = 0; column < 3; ++column)
    for (int k = 0; k < column; ++k)
        body(set(4.0 * (column - 1), 0.5 + 1.0 * k, 0.0), identity);
""", cd=(0.75, 0.6, 0.4), enablesleep=0)
    b.stream("balls", b.sphere("balls", 0.5), """\
// A ball of bounce 0.9 dropped 5 m onto each column.
for (int column = 0; column < 3; ++column)
    body(set(4.0 * (column - 1), 0.5 + 1.0 * column + 5.0, 0.0), identity);
""", cd=(0.85, 0.45, 0.3), bounce=0.9, enablesleep=0)
    return finish(s, b, about="""\
Three balls of bounce 0.9 dropped 5 m: on the ground, on one crate and on two. Without propagation the
crates (bounce 0) soak up the bounce of the balls that land on them.""",
                  try_="turn on Bounce Propagation on the Box3D Solver and compare the rebounds.")


@sample("Restitution", "Box Restitution", SOURCE, frames=120)
def box_restitution(s):
    b = Bodies(s.tag, {"enablesleep": 1})
    b.ground(40.0, friction=0.0)
    b.stream("box", b.box("box", 1.0, 0.25, 1.0), f"""\
// A flat box half a step above the ground, coming down at {num(IMPACT_SPEED)} m/s.
int pt = body({{0, {num(0.25 + 0.5 * IMPACT_SPEED * STEP)}, 0}}, identity);
setpointattrib(0, "v", pt, {{0, {num(-IMPACT_SPEED)}, 0}});
""", cd=(0.9, 0.6, 0.2), density=1.0, friction=0.0, bounce=0.9, enablesleep=0)
    return finish(s, b, gravity=(0.0, 0.0, 0.0), about=f"""\
No gravity. A flat box hits the ground face down at {num(IMPACT_SPEED)} m/s with bounce 0.9. It leaves
level and not spinning (the sample marks its four bottom corners) at 4.33 m/s, as in Box3D: a
coefficient of 0.87 against the 0.9 asked for.""",
                  try_="give it a spin, w (0, 0, 2) in box_points, as the sample's Spin control does.")


@sample("Restitution", "Rotated Box Restitution", SOURCE, frames=300)
def rotated_box_restitution(s):
    b = Bodies(s.tag, {"ccdthreshold": -1.0, "collision_group": 0})
    b.ground(25.0)
    b.stream("cubes", b.box("cubes", 0.5, 0.5, 0.5), """\
// 12 cubes dropped from 10 m, turned 0 to 82.5 degrees about z; collision_group -1: they never touch.
int count = 12;
float dq = 0.5 * PI / count;
float x = -1.0 * (count - 1);
for (int i = 0; i < count; ++i) {
    body(set(x, 10.0, 0.0), axis_angle({0, 0, 1}, i * dq));
    x += 2.0;
}
""", cd=(0.9, 0.6, 0.2), friction=0.0, bounce=0.9, ccdthreshold=0.5, collision_group=-1)
    return finish(s, b, about="""\
12 cubes of bounce 0.9 dropped from 10 m, landing on an edge at angles from 0 to 82.5 degrees. Each
should bounce back to about the same height (the sample prints each one's percentage).""")


@sample("Restitution", "Two Box Restitution", SOURCE, frames=300)
def two_box_restitution(s):
    b = Bodies(s.tag)
    b.ground(10.0)
    b.stream("cubes", b.box("cubes", 0.5, 0.5, 0.5), """\
// A cube on the ground and one dropped on it from 6 m, a little off centre. Bounce 0.9.
body({0, 0.5, 0}, identity);
body({0.1, 6, 0}, identity);
""", cd=(0.9, 0.6, 0.2), bounce=0.9)
    return finish(s, b, about="""\
A cube with bounce 0.9 dropped on another resting on the ground. The pair must not gain energy (the
sample prints the total).""")


@sample("Restitution", "Sphere Restitution", SOURCE, frames=300)
def sphere_restitution(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("spheres", b.sphere("spheres", 0.5), """\
// Ten columns of two spheres (one on the ground, one at 4 m), bounce from 0 to 1 left to right.
int count = 10;
float dr = 1.0 / (count - 1);
float x = -1.0 * (count - 1);
float heights[] = array(1.0, 4.0);
for (int i = 0; i < count; ++i) {
    foreach (float y; heights) {
        int pt = body(set(x, y, 0.0), identity);
        setpointattrib(0, "bounce", pt, i * dr);
        setpointattrib(0, "Cd", pt, hsvtorgb(set(0.66 * (1.0 - i * dr), 0.6, 0.95)));
    }
    x += 2.0;
}
""", density=1.0)
    return finish(s, b, about="""\
Ten pairs of spheres, bounce 0 to 1: the upper one lands on the lower. With bounce mixed as the maximum,
each pair bounces as its own bounce.""")


@sample("Restitution", "Bounce Impulse", SOURCE, frames=300)
def bounce_impulse(s):
    b = Bodies(s.tag, {"enablesleep": 1})
    b.ground(40.0, friction=0.0)
    b.stream("ball", b.sphere("ball", 0.5), "body({0, 20.5, 0}, identity);  // dropped from 20 m",
             cd=(0.85, 0.45, 0.3), friction=0.0, bounce=0.5, enablesleep=0)
    return finish(s, b, about="""\
A ball of bounce 0.5 dropped from 20 m. The sample checks the contact impulse of each bounce against
(1 + bounce) * mass * approach speed.""", try_="""\
turn on Add Impact Data on a Box3D Data after 'pieces' in the DOP network to read the impulses Box3D
reports.""")
