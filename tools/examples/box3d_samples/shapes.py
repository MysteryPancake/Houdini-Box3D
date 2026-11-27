"""Box3D's Shapes samples (samples/sample_shapes.cpp)."""

from .common import Bodies, finish, sample

SOURCE = "sample_shapes.cpp"


@sample("Shapes", "Inclined Plane", SOURCE, frames=480)
def inclined_plane(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("plane", b.box("plane", 16.0, 0.5, 10.0), "static_body({0, 7.5, -5}, axis_angle({1, 0, 0}, radians(40.0)));",
             cd=(0.55, 0.6, 0.65), note="a static slope tilted 40 degrees, friction 1", friction=1.0)
    b.stream("boxes", b.box("boxes", 1.0, 1.0, 1.0), """\
// Five boxes on the slope, their friction rising: 0.04, 0.16, 0.36, 0.64, 1.0.
for (int index = 0; index < 5; ++index) {
    int pt = body(set(-10.0 + 5.0 * index, 14.25, -10.6), axis_angle({1, 0, 0}, radians(40.0)));
    setpointattrib(0, "friction", pt, (index + 1) * (index + 1) * 0.04);
    setpointattrib(0, "Cd", pt, hsvtorgb(set(index / 5.0, 0.6, 0.95)));
}
""")
    return finish(s, b, about="""\
Five boxes on a 40 degree slope of friction 1, their own friction 0.04 to 1. Friction is mixed as the
geometric mean (0.2, 0.4, 0.6, 0.8, 1): below tan(40) = 0.84 a box slides, so only the last one holds.""",
                  ported="""\
the rotations are Box3D's own (b3MakeQuatFromAxisAngle approximates: its 40 degrees is 40.149), so
the boxes start 4.9 cm above the slope as in Box3D. With an exact 40 they start 7.1 cm up, land
harder, and the last box tumbles.""")


@sample("Shapes", "Rolling Resistance", SOURCE, frames=600)
def rolling_resistance(s):
    b = Bodies(s.tag)
    b.ground(80.0)
    b.stream("plane", b.box("plane", 32.0, 0.5, 15.0), "static_body({0, 2, -20}, axis_angle({1, 0, 0}, radians(10.0)));",
             cd=(0.55, 0.6, 0.65), note="a static slope tilted 10 degrees")
    b.stream("spheres", b.sphere("spheres", 1.0), """\
// Five balls rolling down the slope, rolling resistance 0, 0.05 ... 0.2.
for (int index = 0; index < 5; ++index) {
    int pt = body(set(-25.0 + 5.0 * index, 8.0, -24.0), identity);
    setpointattrib(0, "rollingresistance", pt, 0.05 * index);
}
""", cd=(0.85, 0.45, 0.3))
    b.stream("capsules", b.capsule("capsules", 1.0, 0.5), """\
// Five capsules lying along x, rolling resistance 0, 0.05 ... 0.2.
for (int index = 0; index < 5; ++index) {
    int pt = body(set(2.0 + 5.0 * index, 8.0, -24.0), along_x(identity));
    setpointattrib(0, "rollingresistance", pt, 0.05 * index);
}
""", cd=(0.4, 0.65, 0.9), note="b3Capsule { (-1, 0, 0), (1, 0, 0), 0.5 }: built along y, turned to x")
    return finish(s, b, about="""\
Balls and capsules roll down a gentle slope with rolling resistance rising from 0 (left) to 0.2: the
more resistance, the sooner each stops.""")


@sample("Shapes", "High Resistance", SOURCE, frames=360)
def high_resistance(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("capsules", b.capsule("capsules", 1.0, 0.5), """\
// Ten upright capsules tipped 30 degrees, rolling resistance 0, 0.2 ... 1.8.
for (int index = 0; index < 10; ++index) {
    int pt = body(set(-22.0 + 5.0 * index, 1.5, 0.0), axis_angle({0, 0, 1}, radians(30.0)));
    setpointattrib(0, "rollingresistance", pt, 0.2 * index);
}
""", cd=(0.4, 0.65, 0.9), note="b3Capsule { (0, -1, 0), (0, 1, 0), 0.5 }")
    return finish(s, b, about="""\
Ten tilted capsules fall over and roll, with rolling resistance up to 1.8. High values should stop them
without making them jump or stick upright.""")


@sample("Shapes", "Isotropic Friction", SOURCE, frames=300)
def isotropic_friction(s):
    b = Bodies(s.tag)
    b.ground(100.0)
    b.stream("boxes", b.box("boxes", 1.0, 1.0, 1.0), """\
// 32 boxes on a circle of radius 15, each thrown outwards at 25 m/s and turned to face its direction.
for (int index = 0; index < 32; ++index) {
    float alpha = PI / 16.0 * index;
    vector2 cs = cossin(alpha);  // b3ComputeCosSin
    float c = cs[0], sn = cs[1];
    int pt = body(set(15.0 * c, 1.0, 15.0 * sn), axis_angle({0, 1, 0}, -alpha));
    setpointattrib(0, "v", pt, set(25.0 * c, 0.0, 25.0 * sn));
    setpointattrib(0, "Cd", pt, hsvtorgb(set(index / 32.0, 0.6, 0.95)));
}
""")
    return finish(s, b, about="""\
32 boxes slide outwards in every direction at the same speed. Friction does not depend on direction, so
they all stop on one circle.""")


@sample("Shapes", "Slide Twist", SOURCE, frames=360)
def slide_twist(s):
    b = Bodies(s.tag)
    b.ground(50.0)
    b.stream("plane", b.box("plane", 10.0, 0.5, 10.0), "static_body({0, 4, 0}, axis_angle({1, 0, 0}, radians(20.0)));",
             cd=(0.55, 0.6, 0.65), note="a static slope tilted 20 degrees")
    b.stream("box", b.box("box", 1.0, 0.5, 1.0), """\
// A flat box on the slope, spinning about the slope's normal at 25 rad/s.
vector4 q = axis_angle({1, 0, 0}, radians(20.0));
int pt = body({0, 5, 0}, q);
setpointattrib(0, "w", pt, 25.0 * qrotate(q, {0, 1, 0}));
""", cd=(0.9, 0.6, 0.2), friction=0.3)
    return finish(s, b, about="""\
A box dropped spinning fast (25 rad/s about the slope's normal) onto a 20 degree slope. Box3D's own
comment asks what this sample is for; the box spins down and stays about where it lands.""")


@sample("Shapes", "Conveyor Belt", SOURCE, frames=360)
def conveyor_belt(s):
    b = Bodies(s.tag, {"tangentvelocity": (0.0, 0.0, 0.0)})
    b.ground(20.0)
    b.stream("belt", b.box("belt", 10.0, 0.25, 2.0), "static_body({-5, 5, 0}, axis_angle({0, 1, 0}, 0.2));",
             cd=(0.3, 0.3, 0.32), note="a static platform whose surface moves at 2 m/s along its own x",
             friction=0.8, tangentvelocity=(2.0, 0.0, 0.0))
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), """\
for (int i = 0; i < 5; ++i)
    body(set(-10.0 + 2.0 * i, 7.0, 0.0), identity);
""", cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="""\
A conveyor belt: the platform does not move, but its surface does (tangentvelocity (2, 0, 0) in its own
axes, friction 0.8). Boxes dropped on it ride off the end.""",
                  try_="reverse the belt: tangentvelocity -2 in belt_points.")
