"""Box3D's Bodies samples (samples/sample_bodies.cpp)."""

import math

from .common import Bodies, f32, finish, motion_locks, sample

SOURCE = "sample_bodies.cpp"


@sample("Bodies", "Spinning Book", SOURCE, frames=600)
def spinning_book(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(10.0)
    b.stream("books", b.box("books", 0.35, 0.08, 0.5), """\
// Three books floating 2 m up, each spun about one of its axes at 5 rad/s (0.01 about the others).
vector spins[] = array({5, 0.01, 0.01}, {0.01, 5, 0.01}, {0.01, 0.01, -5});
for (int i = 0; i < 3; ++i) {
    int pt = body(set(-2.0 + 2.0 * i, 2.0, 0.0), identity);
    setpointattrib(0, "w", pt, spins[i]);
}
""", cd=(0.85, 0.45, 0.3), note="b3MakeBoxHull( 0.35, 0.08, 0.5 ), gravity scale 0", gravityscale=0.0)
    return finish(s, b, about="""\
Three books (boxes of three different sides) spun in the air without gravity, each about one of its own
axes. About the axis of most or least inertia (centre: y, right: z) a book keeps turning steadily; about
the middle one (left: x) it flips end over end every few seconds (the tennis racket theorem), on the same
frames as in Box3D. The flips come from gyroscopic torque, which Box3D computes.""",
                  try_="spin the centre book about x too: w in books_points.")


@sample("Bodies", "Gyroscopic Torque", SOURCE, frames=900)
def gyroscopic_torque(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    shape = b.parts("spinner", [("cylinder", b.cylinder("spinner_cylinder", 0.6, 0.15, 0.0, 32)),
                                ("bar", b.box("spinner_bar", 1.0, 0.05, 0.1))])
    b.stream("spinner", shape, """\
// A cylinder with a bar across its base, laid on its side (-90 degrees about x) 2 m up and spun at
// (0.01, 0.01, 10) rad/s: mostly about the world z, which is the cylinder's axis turned.
int pt = body({0, 2, 0}, axis_angle({1, 0, 0}, -0.5 * PI));
setpointattrib(0, "w", pt, {0.01, 0.01, 10});
""", cd=(0.4, 0.65, 0.9), note="b3CreateCylinder( 0.6, 0.15, 0, 32 ) and b3MakeBoxHull( 1, 0.05, 0.1 ) on one body; "
                               "gravity scale 0", gravityscale=0.0)
    return finish(s, b, about="""\
The Dzhanibekov effect: a cylinder with a bar across one end, spun in the air without gravity about its
middle axis of inertia. The spin is unstable: about every 2 s the body flips over end to end and keeps
spinning the other way up. Its centre of mass stays where it starts.""",
                  ported="""\
the body's two hulls are named pieces of one packed shape (bullet_georep concave: Bodies.parts). The
point is Box3D's body origin, at the base of the cylinder; the centre of mass is above it. It flips on
the same frames as Box3D's for 8 s, then its spin runs a few degrees ahead (27 at 15 s).""",
                  try_="spin it about another axis (w in spinner_points): about the cylinder's own axis it spins "
                       "steadily.")


def _top_points():
    """The spinning top's hull points as the sample makes them: a 7-sided rim of radius 2 at height 2, in float,
    and the tip at the body origin."""
    segments, r, h = 7, 2.0, 2.0
    dphi = f32(2.0 * f32(math.pi) / segments)
    rim = [(f32(r * f32(math.cos(f32(i * dphi)))), h, f32(r * f32(math.sin(f32(i * dphi))))) for i in range(segments)]
    return rim + [(0.0, 0.0, 0.0)]


@sample("Bodies", "Gyroscopic Precession", SOURCE, frames=900)
def gyroscopic_precession(s):
    b = Bodies(s.tag, {"fastrotation": 0})
    b.ground(40.0)
    top = b.hull("tops", _top_points(), note="b3CreateHull of a 7-sided rim of radius 2 at height 2 and the tip "
                                             "at the body origin")
    b.stream("tops", top, """\
// 8 x 8 tops 6 m apart, tilted 15 degrees about z, their tips 2 m up, each spun at 75 rad/s about its own axis.
vector4 q = axis_angle({0, 0, 1}, 15.0 * PI / 180.0);
vector spin = qrotate(q, {0, 75, 0});
for (int x = 0; x < 8; ++x)
    for (int z = 0; z < 8; ++z) {
        int pt = body(set((x - 4) * 6.0, 2.0, (z - 4) * 6.0), q);
        setpointattrib(0, "w", pt, spin);
        setpointattrib(0, "Cd", pt, hsvtorgb(set((8 * x + z) / 64.0, 0.55, 0.95)));
    }
""", note="rolling resistance 0.1; fastrotation 1, as the spin is past Box3D's speed cap", rollingresistance=0.1,
             fastrotation=1)
    return finish(s, b, about="""\
64 spinning tops dropped onto their tips, tilted 15 degrees. Gravity pulls each over, and the spin turns that
pull into a slow circling of its axis about the vertical (precession) with a small wobble (nutation),
instead of a fall. Rolling resistance at the tip slows the spin, and the tilt grows slowly (15 to 26
degrees over 15 s, as in Box3D).""",
                  ported="""\
the sample's readout (the first top's precession rate against the heavy top formula) is dropped. fastrotation
lifts Box3D's cap on how far a body turns in a step (45 degrees: 47 rad/s here), as the sample's
allowFastRotation.""",
                  try_="set fastrotation to 0 in tops_points: the spin is capped at 47 rad/s from the first step.")


@sample("Bodies", "Spinning Capsule", SOURCE, frames=600)
def spinning_capsule(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("capsule", b.capsule("capsule", 2.0, 0.2), """\
// A capsule 4 m long standing on its lower cap, tilted 15 degrees about z, spun at 20 rad/s about its own axis.
vector4 q = axis_angle({0, 0, 1}, 15.0 * PI / 180.0);
vector axis = qrotate(q, {0, 1, 0});
int pt = body({0, 0.2, 0} + 2.0 * axis, q);
setpointattrib(0, "w", pt, 20.0 * axis);
""", cd=(0.4, 0.65, 0.9), note="b3Capsule { (0, -2, 0), (0, 2, 0), 0.2 }")
    return finish(s, b, about="""\
A long thin capsule stood on its end and spun fast about its own axis. A thin rod has so little inertia
about its axis that the spin cannot hold it up: it falls over within 2 s and rolls away, off the
ground's edge by 10 s. That needs gyroscopic torque, which Box3D skips for bodies whose inertia is
nearly the same about every axis (within the solver's Near-Sphere Tolerance, 0.2 here, Box3D's default).""",
                  ported="""\
the sample's Length slider (4) is the capsule's size in capsule_shape and capsule_points; its Tolerance
slider is the solver's Near-Sphere Tolerance. The readouts (anisotropy, tilt, spin) are dropped.""",
                  try_="set Near-Sphere Tolerance on the solver (Advanced) to 1: the capsule counts as a sphere, "
                       "gets no gyroscopic torque, and stays standing, circling, as it should not.")


@sample("Bodies", "Weeble", SOURCE, frames=600)
def weeble(s):
    b = Bodies(s.tag, {"computecom": 1})
    b.ground(30.0)
    # b3Capsule { (0, -1), (0, 1), 1 } on a body whose centre of mass is set 1.5 below its origin: the point (the
    # centre of mass here) is 1.5 below the capsule's middle, with computecom 0.
    shape = b.shifted(b.capsule("weeble", 1.0, 1.0), (0.0, 1.5, 0.0))
    note = "the point is the centre of mass, 1.5 below the capsule's middle (computecom 0); rolling resistance 0.1"
    flipped = b.stream("weeble_flipped", shape, """\
// The sample's Teleport button: the body's origin at (0, 5, 0), turned 0.95 * 180 degrees about z.
vector4 q = axis_angle({0, 0, 1}, 0.95 * PI);
body({0, 5, 0} + qrotate(q, {0, -1.5, 0}), q);
""", cd=(0.85, 0.45, 0.3), note=note, merged=False, rollingresistance=0.1, computecom=0)
    upright = b.stream("weeble_upright", shape, """\
// As the sample starts: the body's origin at (0, 3, 0), upright.
body({0, 1.5, 0}, identity);
""", cd=(0.85, 0.45, 0.3), note=note, merged=False, rollingresistance=0.1, computecom=0)
    b.choice("start", "Start", [("teleported", "Teleported (upside down)", flipped),
                                ("upright", "Upright, as the sample starts", upright)])
    return finish(s, b, about="""\
A weeble: a capsule whose centre of mass is set low, 1.5 below its middle (b3Body_SetMassData), so it
always rights itself. It starts nearly upside down, 5 m up, as the sample's Teleport button puts it, lands
on its head, rolls over upright and rocks, still swinging 60 degrees each way after 10 s.""",
                  ported="""\
Box3D's body origin is the capsule's middle and its centre of mass is moved; here the point is the centre of
mass (computecom 0 puts the centre at the point, with the inertia moved to it as the sample's b3Steiner
does), and the capsule is drawn 1.5 above it. The scene starts from the Teleport pose (Start on the bodies
object's Sample tab); the Explode button is dropped.""",
                  try_="Start: Upright on the bodies object's Sample tab, then restart: it drops 1 m and stays up.")


@sample("Bodies", "Kinematic", SOURCE, frames=900)
def kinematic(s):
    b = Bodies(s.tag, {"animated": 0})
    b.ground(20.0)
    b.stream("mover", b.box("mover", 0.1, 1.0, 0.2), """\
// A kinematic body (active 0, animated 1) moved along a figure of eight, turning as it goes, after 2 s of
// waiting. Box3D sets the target at the start of a step and reaches it at the end, so the pose at time T is the
// path at T - one step - 2 s.
float amplitude = 2.0;
float t = max(@Time - 1.0 / 60.0 - 2.0, 0.0);
vector p = set(2.0 * amplitude * cos(t), amplitude * (sin(2.0 * t) + 1.0) + 1.0, 0.0);
int pt = static_body(p, axis_angle({0, 0, 1}, 2.0 * t));
setpointattrib(0, "animated", pt, 1);
""", cd=(0.75, 0.55, 0.85), note="b3MakeBoxHull( 0.1, 1, 0.2 ) on a kinematic body", animated=1)
    return finish(s, b, about="""\
A kinematic body: it moves on a set path (a figure of eight, turning about z) and nothing pushes it.
Box3D drives it with b3Body_SetTargetTransform each step, which sets the velocity that reaches the next
pose, so it would push dynamic bodies out of its way.""",
                  ported="""\
the body is an animated piece (active 0, animated 1): its pose comes from mover_points at each step's
end time, and the solver sets the same target velocity. Its path is the sample's, a step and 2 s late.""",
                  try_="drop something in its way: a box stream with body() in bodies, or another object in the "
                       "DOP network.")


@sample("Bodies", "Fixed Rotation", SOURCE, frames=300)
def fixed_rotation(s):
    b = Bodies(s.tag, {"gravityscale": 1.0, "enablesleep": 1, "inertialtensorstiffness": 1.0})
    b.ground(20.0)
    # b3Capsule { (0, 0, 0), (0, 1, 0), r }: from the body origin up 1, so the capsule sits 0.5 above its point.
    b.stream("post", b.shifted(b.capsule("post", 0.5, 0.3), (0.0, 0.5, 0.0)), "static_body({0, 0.5, 0}, identity);",
             cd=(0.55, 0.6, 0.65), note="a static capsule of radius 0.3")
    b.stream("fixed", b.shifted(b.capsule("fixed", 0.5, 0.2), (0.0, 0.5, 0.0)), """\
// A capsule of radius 0.2 started 0.3 from the post's axis, overlapping it: the contacts push it out.
body({0.3, 0.5, 0}, identity);
""", cd=(0.9, 0.6, 0.2), note="no gravity, never sleeps, cannot turn (Lock Tilt and inertialtensorstiffness 1e4)",
             gravityscale=0.0, enablesleep=0, inertialtensorstiffness=1.0e4)
    return finish(s, b, about="""\
A body that cannot turn (fixedRotation: a zero inverse inertia) started overlapping a static post. It is
pushed out sideways without turning until it just touches the post (within half a second), and stays
there: no gravity, no sleeping.""",
                  ported="""\
fixedRotation has no attribute. In its place a Box3D Motion Lock to the world with only Lock Tilt on holds
the tilt about x and z, and inertialtensorstiffness 1e4 (10000 times as hard to turn) the spin about y.""",
                  try_="turn Lock Tilt off on lock_rotation and set inertialtensorstiffness to 1 in fixed_points.",
                  joints=[("box3d_motionlockconrel", "lock_rotation",
                           {"lock_x": 0, "lock_y": 0, "lock_z": 0, "lock_angular_xy": 1}, "motionlock")],
                  lines=motion_locks([("fixed0", (0.3, 0.5, 0.0))]))


@sample("Bodies", "Offset Kinematic", SOURCE, frames=600)
def offset_kinematic(s):
    b = Bodies(s.tag, {"animated": 0})
    b.ground(20.0)
    b.stream("box", b.box("box", 0.5, 0.5, 1.0), """\
// A kinematic box spinning at a constant (1, -1, 2) rad/s about its centre, (0.5, 3.5, 1): its body origin is
// (0, 3, 0), a corner of the box, and the sample moves the centre of mass to the box's middle.
vector w = {1, -1, 2};
float t = @Time;
int pt = static_body({0.5, 3.5, 1}, quaternion(length(w) * t, normalize(w)));
setpointattrib(0, "animated", pt, 1);
""", cd=(0.75, 0.55, 0.85), note="b3MakeTransformedBoxHull( 0.5, 0.5, 1, offset (0.5, 0.5, 1) ) on a kinematic body",
             animated=1)
    return finish(s, b, about="""\
A kinematic box given a constant spin. Box3D turns a body about its centre of mass, but it computes no mass
for a kinematic body, so it would turn about its origin, a corner of the box here. The sample sets the centre
of mass to the box's middle with b3Body_SetMassData, and the box spins in place.""",
                  ported="""\
an animated piece (active 0, animated 1) with its point at the box's middle, turned by the spin times the
time. The body origin of the sample is not used: the point is where the box turns.""")
