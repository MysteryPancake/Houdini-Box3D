"""Box3D's Joints samples (samples/sample_joint.cpp). A joint is a constraint network line between two bodies' anchors
(an empty name is the world, Box3D's static ground body), its settings a Box3D relationship node's parameters: the
sample's sliders and checkboxes, live in Houdini as they are in the sample app."""

import math

from .common import (VEX_FUNCTIONS, Bodies, box_impulse, finish, height_field, motion_locks, num, sample,
                     vec)

SOURCE = "sample_joint.cpp"

H = math.sqrt(0.5)
Z_TO_Y = (-H, 0.0, 0.0, H)  # b3ComputeQuatBetweenUnitVectors( z, y ): a quarter turn about -x
X_TO_Y = (0.0, 0.0, H, H)  # ( x, y ): about z
Y_TO_Z = (H, 0.0, 0.0, H)  # ( y, z ): about x
Y_TO_X = (0.0, 0.0, -H, H)  # ( y, x ): about -z

TRY_NODE = "the sample's checkboxes and sliders are the {node} node's parameters: turn on its {what} and play."


def node(kind, name, **parms):
    """A Box3D <kind> relationship node named `name`, its Data Name (the lines' constraint_name) its name too: nodes
    of one type would otherwise share the type's default Data Name, and all their lines one node's settings."""
    return (f"box3d_{kind}conrel", name, {"dataname": name, **parms}, name)


@sample("Joints", "Distance Joint", SOURCE, frames=300)
def distance_joint(s):
    count, length = 1, 1.0
    b = Bodies(s.tag, {"angulardamping": 0.0})
    b.ground(20.0)
    b.stream("balls", b.sphere("balls", 0.25), f"""\
// {count} sphere(s) of radius 0.25 a joint length apart along x, 20 m up: DistanceJoint's Count {count}.
for (int i = 0; i < {count}; ++i) body(set({num(length)} * (i + 1), 20, 0), identity);
""", cd=(0.85, 0.45, 0.3), density=20.0, angulardamping=1.0)
    lines = [("" if i == 0 else f"balls{i - 1}", (length * i, 20.0, 0.0), f"balls{i}", (length * (i + 1), 20.0, 0.0),
              "rods") for i in range(count)]
    return finish(s, b, about="""\
A ball on a 1 m distance joint to the world (its anchors 1 m apart, the length they start at), started
level: a rigid pendulum. The joint's spring and limit are off, as the sample starts.""",
                  try_=TRY_NODE.format(node="rods", what="Enable Spring (5 Hz, tension 2000 and compression 100 "
                                                         "set) or Enable Limit") + " More balls: count in joints.py.",
                  joints=[node("distance", "rods", stiffness=5.0, damping_ratio=0.5, limit_spring_force=1,
                               max_tension_force=2000.0, max_compression_force=100.0, min_length=1.0,
                               limit_max_length=1, max_length=1.0)],
                  lines=lines)


@sample("Joints", "Filter", SOURCE, frames=120)
def filter_joint(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 0.5, 0.5, 0.5), "body({2, 4, 0}, identity);\nbody({-2, 4, 0}, identity);",
             cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="""\
Two cubes joined by a filter joint: it holds nothing, only stops the pair colliding with each other. They
fall side by side onto the ground.""", try_="move one cube over the other in boxes_points: they pass through "
                                           "each other.",
                  joints=[node("filter", "no_collision")], lines=[("boxes0", (2, 4, 0), "boxes1", (-2, 4, 0),
                                                                   "no_collision")])


@sample("Joints", "Motor Joint", SOURCE, frames=600)
def motor_joint(s):
    b = Bodies(s.tag, {"animated": 0, "collision_mask": -1})
    b.ground(20.0)
    speed = b.slider("speed", "Speed", 1.0, -5.0, 5.0,
                     "How fast the target runs along its path (the sample's Speed slider, which starts at 0). "
                     "Restart the simulation after changing it.")
    b.stream("target", b.sphere("target", 0.1), f"""\
// The kinematic target (active 0, animated 1), moved each step by b3Body_SetTargetTransform along a figure of eight
// in the xy plane, turning about z. The sample advances its time by Speed each step before setting the target, so the
// pose at time T is the path at Speed * T. It collides with nothing (collision_mask 0): the sample's has no shape.
float t = {speed} * @Time;
vector p = set(6.0 * sin(2.0 * t), 10.0 + 4.0 * sin(t), 0.0);
int pt = static_body(p, axis_angle({{0, 0, 1}}, 2.0 * t));
setpointattrib(0, "animated", pt, 1);
""", cd=(0.75, 0.55, 0.85), note="a small ball to show the target; the sample's target body has no shape",
             animated=1, collision_mask=0)
    b.stream("box", b.box("box", 1.0, 0.25, 0.25), "body({0, 10, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="b3MakeBoxHull( 1, 0.25, 0.25 ): 500 kg")
    b.stream("spring_box", b.box("spring_box", 0.5, 0.5, 0.5), "body({-2, 2, 0}, identity);", cd=(0.4, 0.65, 0.9),
             note="held in the air at its starting pose by a motor joint to the world at its corner (-1.75, 2.25, 0)")
    return finish(s, b, about="""\
Motor joints: a box chasing a moving target. The target (the small ball) runs along a figure of eight,
turning as it goes; the box is joined to it by a motor joint whose springs (4 Hz, damping 0.7, at most
400000 N and 500000 N m) pull the box's position and rotation to the target's, so it follows with a lag
and swings at the turns. A second box near the ground hangs in the air on a stiffer motor joint to the
world (7.5 Hz), which holds it where it starts.""",
                  ported="""\
the target is an animated piece (active 0, animated 1) posed from target_points, and the sample's Speed
slider (0 when the sample starts: nothing moves until you drag it) is Speed on the bodies object's
Sample tab, set to 1 here. The force and torque readout is dropped. From frame 30 on both boxes are where
Box3D's are, to a millimetre; on the first frames the chasing box starts a little slower.""",
                  try_="Speed 3 (restart), or lower Max Spring Force and Max Spring Torque on the chase node: the box "
                       "falls behind and sags. The sample's Apply Impulse: v {200, 0, 0} on box_points.",
                  joints=[node("motor", "chase", linear_stiffness=4.0, linear_damping_ratio=0.7, angular_stiffness=4.0,
                               angular_damping_ratio=0.7, max_spring_force=400000.0, max_spring_torque=500000.0),
                          node("motor", "hold", collide_connected=1, linear_stiffness=7.5, linear_damping_ratio=0.7,
                               angular_stiffness=7.5, angular_damping_ratio=0.7, max_spring_force=200000.0,
                               max_spring_torque=10000.0)],
                  lines=[("target0", (0, 10, 0), "box0", (0, 10, 0), "chase"),
                         ("", (-1.75, 2.25, 0), "spring_box0", (-1.75, 2.25, 0), "hold")])


@sample("Joints", "Top Down Friction", SOURCE, frames=300)
def top_down_friction(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.stream("walls_x", b.box("walls_x", 10.0, 0.5, 4.0), "static_body({0, 0, 0}, identity);\n"
             "static_body({0, 20, 0}, identity);", cd=(0.55, 0.6, 0.65),
             note="a closed square in xy, open in z: four box hulls on one static body in Box3D")
    b.stream("walls_y", b.box("walls_y", 0.5, 10.0, 4.0), "static_body({-10, 10, 0}, identity);\n"
             "static_body({10, 10, 0}, identity);", cd=(0.55, 0.6, 0.65))
    grid =[("capsules", b.capsule("capsules", 0.25, 0.25), "k % 4 == 0", "along_x(identity)", (0.4, 0.65, 0.9),
             "b3Capsule { (-0.25, 0, 0), (0.25, 0, 0), 0.25 }: built along y, turned to x"),
            ("spheres", b.sphere("spheres", 0.35), "k % 4 == 1", "identity", (0.85, 0.45, 0.3), "radius 0.35"),
            ("cubes", b.box("cubes", 0.35, 0.35, 0.35), "k % 4 >= 2", "identity", (0.9, 0.6, 0.2),
             "b3MakeBoxHull( 0.35, 0.35, 0.35 )")]
    for tag, shape, test, turn, colour, what in grid:
        b.stream(tag, shape, f"""\
// The {tag} of a 10 x 10 grid 1 m apart in the xy plane, row by row from (-5, 15): body k is a capsule when k % 4
// is 0, a sphere when 1, a cube otherwise.
for (int k = 0; k < 100; ++k)
    if ({test}) body(set(-5.0 + k % 10, 15.0 - k / 10, 0.0), {turn});
""", cd=colour, note=f"{what}, restitution 0.8, gravity scale 0", bounce=0.8, gravityscale=0.0)
    b.explode((0.0, 10.0, 0.0), 10.0, 5.0, 10000.0)
    counts, lines = {}, []
    for k in range(100):
        tag = grid[min(k % 4, 2)][0]
        counts[tag] = counts.get(tag, -1) + 1
        p = (-5.0 + k % 10, 15.0 - k // 10, 0.0)
        lines.append(("", p, f"{tag}{counts[tag]}", p, "friction"))
    return finish(s, b, about="""\
A top-down view (look along -z): 100 capsules, spheres and cubes with no gravity in a closed square, each
on a motor joint to the world that pulls its velocity and spin toward zero with at most 1000 N and 1000 N m:
friction against an imagined floor. Explode blows them apart from the middle; they bounce off the walls
(restitution 0.8) and are slowed to a stop by the motors.""",
                  ported="the sample's Explode button is pressed before the first frame: the explode wrangle in the "
                         "bodies object gives each body the push b3World_Explode would. Without it nothing moves.",
                  try_="the Explosion settings on the bodies object's Sample tab (move the Centre, raise Impulse per "
                       "Area), or " + TRY_NODE.format(node="friction", what="Max Velocity Force and Max Velocity "
                                                                             "Torque (lower: they slide further)"),
                  joints=[node("motor", "friction", collide_connected=1, max_velocity_force=1000.0,
                               max_velocity_torque=1000.0)], lines=lines)


@sample("Joints", "Prismatic", SOURCE, frames=300)
def prismatic(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    b.stream("box", b.box("box", 0.5, 1.5, 0.25), "body({0, 4, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="no gravity", gravityscale=0.0)
    return finish(s, b, about="""\
A box on a prismatic joint to the world at its top (5.5 m up), sliding along x, with no gravity. The
joint's spring (2 Hz) holds it at its target translation 0, so it stays still until you change the
joint.""", try_=TRY_NODE.format(node="slider", what="Enable Motor or Enable Limit, or set Target "
                                                  "Translation"),
                  joints=[node("prismatic", "slider", constraint_stiffness=120.0, enable_spring=1, stiffness=2.0,
                               damping_ratio=0.7, max_motor_force=20.0, motor_speed=0.0, lower_translation=-1.0,
                               upper_translation=1.0)],
                  lines=[("", (0, 5.5, 0), "box0", (0, 5.5, 0), "slider")])


@sample("Joints", "Spherical", SOURCE, frames=300)
def spherical(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    b.stream("box", b.box("box", 0.5, 1.5, 0.25), "body({0, 4, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="density 100, no gravity", density=100.0, gravityscale=0.0)
    return finish(s, b, about="""\
A box on a spherical joint to the world at its top, with no gravity. The joint's spring (2 Hz) holds it
at its target rotation, so it stays still until you change the joint.""",
                  try_=TRY_NODE.format(node="ball_joint", what="Enable Motor (Motor Velocity), the cone and twist "
                                                             "limits, or set Target Rotation"),
                  joints=[node("spherical", "ball_joint", enable_spring=1, stiffness=2.0, damping_ratio=0.7,
                               cone_angle=30.0, lower_twist_angle=-35.0, upper_twist_angle=35.0,
                               max_motor_torque=20.0)],
                  lines=[("", (0, 5.5, 0), "box0", (0, 5.5, 0), "ball_joint")])


@sample("Joints", "Spherical Motor Frame", SOURCE, frames=480)
def spherical_motor_frame(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    turn = vec4(Y_TO_X)
    b.stream("base", b.box("base", 1.4, 0.3, 0.3), f"static_body({{0, 2, 0}}, {turn});", cd=(0.55, 0.6, 0.65),
             note="the neck, turned so its x is down (kinematic in the sample, its spin speed 0)")
    head = b.compound("head", [("skull", (0.5, 0.5, 0.5), (0, 0, 0)), ("nose", (0.1, 0.1, 0.25), (0, 0, 0.75))])
    b.stream("head", head,
             f"body({{0, 4, 0}}, {turn});", cd=(0.9, 0.6, 0.2),
             note="a cube and a nose (b3MakeOffsetBoxHull): two hulls on one body, no gravity", gravityscale=0.0)
    return finish(s, b, about="""\
A head on a spherical joint to a neck, its motor driving it at 1 rad/s about the joint frame's z (Motor
Velocity (0, 0, 57.3) degrees per second, Max Motor Torque 1000). The motor's velocity is in the frame of
the first body, the neck: its z is the way the nose points, so the head spins steadily about its nose.""",
                  ported="""\
the neck is a static body: the sample's is kinematic, spun by its Base Speed slider (0 at the start).
The head's two hulls are named pieces of one packed shape (bullet_georep concave).""",
                  try_=TRY_NODE.format(node="neck_joint", what="Motor Velocity: another axis, or several at once"),
                  joints=[node("spherical", "neck_joint", enable_motor=1,
                               motor_velocity=(0.0, 0.0, math.degrees(1.0)), max_motor_torque=1000.0)],
                  lines=[("base0", (0, 4, 0), "head0", (0, 4, 0), "neck_joint", Y_TO_X)])


@sample("Joints", "Parallel Spring", SOURCE, frames=300)
def parallel_spring(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("walls_x", b.box("walls_x", 20.0, 5.0, 0.1), "static_body({0, 4, -20}, identity);\n"
             "static_body({0, 4, 20}, identity);", cd=(0.55, 0.6, 0.65),
             note="four static walls (in Box3D box hulls on one static body at (0, -1, 0))")
    b.stream("walls_z", b.box("walls_z", 0.1, 5.0, 20.0), "static_body({-20, 4, 0}, identity);\n"
             "static_body({20, 4, 0}, identity);", cd=(0.55, 0.6, 0.65))
    b.stream("box", b.box("box", 0.5, 1.5, 0.25), "body({0, 4, 0}, axis_angle({1, 0, 0}, 0.25 * PI));",
             cd=(0.9, 0.6, 0.2), note="tilted 45 degrees about x")
    return finish(s, b, about="""\
A tilted box on a parallel joint to the world: a 10 Hz spring holds its axis at the angle it starts at
(the joint's frames turned so their z is up). It falls onto the ground and lands still tilted, its axis
kept parallel while it slides; the walls keep it in.""",
                  try_=TRY_NODE.format(node="keep_tilt", what="Limit Torque (5000 in the sample's slider)") +
                  " Throw it: v in box_points.",
                  joints=[node("parallel", "keep_tilt", stiffness=10.0, damping_ratio=0.7, max_torque=5000.0,
                               collide_connected=1)],
                  lines=[("", (0, -1, 0), "box0", (0, 4, 0), "keep_tilt", Z_TO_Y)])


@sample("Joints", "Revolute", SOURCE, frames=300)
def revolute(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("box", b.box("box", 0.5, 1.5, 0.25), "body({0, 4, 0}, identity);", cd=(0.9, 0.6, 0.2))
    return finish(s, b, about="""\
A box hanging from a revolute joint to the world at its top, turning about z. It hangs straight down,
so it stays still until you push it or change the joint.""",
                  try_=TRY_NODE.format(node="hinge", what="Enable Motor (Max Motor Torque 5000), Enable Spring "
                                                        "or Enable Limit (-35 to 35)") +
                  " Swing it: w {0, 0, 3} in box_points.",
                  joints=[node("revolute", "hinge", stiffness=2.0, damping_ratio=0.7, max_motor_torque=5000.0,
                               motor_speed=0.0, lower_angle=-35.0, upper_angle=35.0)],
                  lines=[("", (0, 5.5, 0), "box0", (0, 5.5, 0), "hinge")])


@sample("Joints", "Weld", SOURCE, frames=300)
def weld(s):
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    b.stream("box", b.box("box", 0.5, 1.5, 0.25), "body({0, 4, 0}, identity);", cd=(0.9, 0.6, 0.2),
             note="no gravity", gravityscale=0.0)
    return finish(s, b, about="""\
A box welded to the world at its top, with no gravity: rigid in position, a 2 Hz spring in rotation
(Constraint Stiffness 240). It stays still until pushed, then wobbles back.""",
                  try_=TRY_NODE.format(node="weld", what="Enable Linear Spring, or change the angular spring") +
                  " Kick it: w {3, 0, 0} in box_points.",
                  joints=[node("weld", "weld", constraint_stiffness=240.0, enable_angular_spring=1,
                               angular_stiffness=2.0, angular_damping_ratio=0.7)],
                  lines=[("", (0, 5.5, 0), "box0", (0, 5.5, 0), "weld")])


@sample("Joints", "Wheel", SOURCE, frames=240)
def wheel(s):
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("wheel", b.cylinder("wheel", 0.25, 0.4, 0.0, 12), f"body({{0, 2, 0}}, {vec4(Y_TO_Z)});",
             cd=(0.3, 0.3, 0.32), note="b3CreateCylinder( 0.25, 0.4, 0, 12 ), its axis turned to z")
    return finish(s, b, about="""\
A wheel on a wheel joint to the world: its suspension runs up and down (the joint frame's x turned to y)
and it spins about z. The sample starts with the suspension spring, its limit, the spin motor and the
steering all off, so the wheel slides down its free suspension onto the ground.""",
                  ported="the joint's frames are one orientation on both ends; Box3D's differ by a turn about the "
                         "spin axis, which is free.",
                  try_=TRY_NODE.format(node="wheel_joint", what="Enable Suspension Spring (2 Hz), Enable Spin Motor "
                                                              "or Enable Steering"),
                  joints=[node("wheel", "wheel_joint", collide_connected=1, enable_suspension_spring=0,
                               suspension_stiffness=2.0, suspension_damping_ratio=0.7, lower_suspension_limit=-1.0,
                               upper_suspension_limit=1.0, max_spin_torque=20.0, spin_speed=0.0,
                               steering_stiffness=1.0, steering_damping_ratio=0.7, max_steering_torque=20.0,
                               lower_steering_limit=-45.0, upper_steering_limit=45.0)],
                  lines=[("", (0, 2, 0), "wheel0", (0, 2, 0), "wheel_joint", X_TO_Y)])


@sample("Joints", "Ball and Chain", SOURCE, frames=480)
def ball_and_chain(s):
    links, extent, radius = 32, 0.5, 2.0
    b = Bodies(s.tag)
    b.stream("links", b.capsule("links", extent, 0.125), f"""\
// {links} capsule links along x from the world anchor at the origin.
for (int i = 0; i < {links}; ++i) body(set({num(extent)} * (1 + 2 * i), 0, 0), along_x(identity));
""", cd=(0.6, 0.62, 0.66), note="b3Capsule { (-0.5, 0, 0), (0.5, 0, 0), 0.125 }: built along y, turned to x")
    ball_x = (1.0 + 2.0 * links) * extent + radius - extent
    b.stream("ball", b.sphere("ball", radius), f"body({{{num(ball_x)}, 0, 0}}, identity);", cd=(0.85, 0.45, 0.3))
    lines = [("" if i == 0 else f"links{i - 1}", (2 * extent * i, 0, 0), f"links{i}", (2 * extent * i, 0, 0),
              "chain") for i in range(links)]
    lines.append((f"links{links - 1}", (2 * extent * links, 0, 0), "ball0", (2 * extent * links, 0, 0), "chain"))
    return finish(s, b, about=f"""\
A chain of {links} capsule links held at one end by the world, a 2 m ball on the other end, started
straight out along x. The spherical joints have a friction motor (Motor Velocity 0, Max Motor Torque
10) that damps the swing. No ground.""",
                  try_=TRY_NODE.format(node="chain", what="Max Motor Torque up or down (0 is a free chain)"),
                  joints=[node("spherical", "chain", enable_motor=1, max_motor_torque=10.0)], lines=lines)


@sample("Joints", "Bridge", SOURCE, frames=480)
def bridge(s):
    count, a = 150, 0.125
    xbase = -160.0 * a
    b = Bodies(s.tag, {"lineardamping": 0.0, "angulardamping": 0.0})
    b.ground(60.0)
    b.stream("planks", b.box("planks", a, 0.125, 0.5), f"""\
// {count} planks 0.25 wide, 20 m up, edge to edge along x.
for (int i = 0; i < {count}; ++i) body(set({num(xbase)} + {num(a)} * (1 + 2 * i), 20, 0), identity);
""", cd=(0.65, 0.5, 0.35), density=20.0, lineardamping=0.1, angulardamping=0.1)
    lines = []
    for i in range(count + 1):
        left, right = ("" if i == 0 else f"planks{i - 1}"), ("" if i == count else f"planks{i}")
        for z in (-0.5, 0.5):  # a joint at each side of the deck
            p = (xbase + 2 * a * i, 20.0, z)
            lines.append((left, p, right, p, "deck_joints"))
    return finish(s, b, about=f"""\
A rope bridge of {count} planks, each joined to the next by two spherical joints (one at each side), the
ends to the world. The joints have a 2 Hz spring holding the planks level with each other and a very
stiff Constraint Stiffness (1000 Hz), so the bridge sags little.""",
                  try_=TRY_NODE.format(node="deck_joints", what="spring off, or a lower Constraint Stiffness"),
                  joints=[node("spherical", "deck_joints", constraint_stiffness=1000.0, enable_spring=1,
                               stiffness=2.0, damping_ratio=1.0)], lines=lines)


@sample("Joints", "Fold Out", SOURCE, frames=480)
def fold_out(s):
    half, spacing = 2.0, 1.0
    b = Bodies(s.tag)
    b.ground(20.0)
    b.stream("boards", b.box("boards", half, 0.1, 0.25), f"""\
// Six boards 4 m long, stacked {num(spacing)} apart, each hinged to the one below at alternate ends.
for (int i = 0; i < 6; ++i) body(set(0, {num(0.5 * spacing)} + i * {num(spacing)}, 0), identity);
""", cd=(0.65, 0.5, 0.35), density=2000.0)
    lines = []
    for i in range(6):
        x = half if i & 1 else -half
        p = (x, 0.5 * spacing + i * spacing - 0.5 * spacing, 0.0)
        lines.append(("" if i == 0 else f"boards{i - 1}", p, f"boards{i}", p, "hinges", Z_TO_Y))
    return finish(s, b, about="""\
Six boards stacked in the air, each hinged to the one below at alternate ends about an upright axis
(the first to the ground). The hinges are upright, so gravity cannot turn them, and their friction motors
(Motor Speed 0, Max Motor Torque 1000) hold them: the stack stands as it is and falls asleep at once.""",
                  try_=TRY_NODE.format(node="hinges", what="Motor Speed (45: every hinge turns and the boards fold "
                                                         "out)"),
                  joints=[node("revolute", "hinges", constraint_stiffness=240.0, enable_motor=1,
                               max_motor_torque=1000.0, motor_speed=0.0)], lines=lines)


@sample("Joints", "Door", SOURCE, frames=300)
def door(s):
    half, density, magnitude = (0.75, 1.5, 0.1), 1000.0, 50000.0
    v, w = box_impulse(half, density, (0.0, 0.0, -magnitude), (0.75, 0.0, 0.0))
    b = Bodies(s.tag, {"gravityscale": 1.0})
    b.ground(20.0)
    b.stream("door", b.box("door", *half), f"""\
// The door: 1.5 x 3 x 0.2 m, 900 kg, its bottom edge on the ground. It starts with the velocity the sample's Impulse
// button gives it: {num(magnitude)} N s along -z at the middle of its free edge (b3Body_ApplyLinearImpulse).
int pt = body({{0, 1.5, 0}}, identity);
setpointattrib(0, "v", pt, {vec(v)});
setpointattrib(0, "w", pt, {vec(w)});
""", cd=(0.6, 0.45, 0.3), note="density 1000, gravity scale 2", density=density, gravityscale=2.0)
    hinge = node("revolute", "hinges", constraint_stiffness=120.0, constraint_damping_ratio=0.0, enable_limit=1,
                 lower_angle=-90.0, upper_angle=90.0, enable_spring=1, stiffness=1.0, damping_ratio=0.5,
                 max_motor_torque=100.0, motor_speed=0.0)
    lines = [("ground0", (-0.75, y, 0), "door0", (-0.75, y, 0), "hinges", Z_TO_Y) for y in (0.0, 3.0)]
    return finish(s, b, about=f"""\
A heavy door (900 kg, gravity scale 2) on two revolute hinges along its left edge, at its bottom and top
corners, with a 1 Hz spring that swings it shut and a limit of 90 degrees either way. It is hit with
{num(magnitude)} N s at its free edge, which slams it against the limit: the hinges' Constraint
Stiffness (120 Hz, damping 0) keeps the door on them.""",
                  ported="the Impulse button is the door's starting v and w (box_impulse); the hinges join the "
                         "door to the ground body as in the sample, so its bottom edge does not rub on the ground. "
                         "The translation error readout is not drawn.",
                  try_=TRY_NODE.format(node="hinges", what="Constraint Stiffness (15 to 240) and Constraint "
                                                           "Damping Ratio, or turn off Enable Limit") +
                  " One hinge: delete the top line in door_network. Another blow: scale v and w in door_points.",
                  joints=[hinge], lines=lines)


@sample("Joints", "Motion Lock Joint", SOURCE, frames=300)
def motion_lock_joint(s):
    presets = [("point_point", 1, 1, 1, 0), ("point_line_x", 0, 1, 1, 0), ("point_plane_xz", 0, 1, 0, 0),
               ("planar_xy", 0, 0, 1, 1)]
    b = Bodies(s.tag, {"enablesleep": 1, "computemass": 1, "mass": 0.0})
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 1.0, 0.5, 0.25), """\
// Four boxes 4 m up, each held by a different motion lock to the world at its centre.
for (int i = 0; i < 4; ++i) body(set(-9.0 + 6.0 * i, 4, 0), identity);
""", cd=(0.9, 0.6, 0.2), enablesleep=0)
    b.stream("platform", b.box("platform", 2.0, 0.05, 0.5), "body({0, 8, 6}, identity);", cd=(0.55, 0.6, 0.65),
             note="mass 1 (b3Body_SetMassData; the inertia scaled with it)", enablesleep=0, computemass=0, mass=1.0)
    b.stream("ball", b.sphere("ball", 0.5), "body({0, 9, 6}, identity);", cd=(0.85, 0.45, 0.3),
             note="mass 10", enablesleep=0, computemass=0, mass=10.0)
    joints = [node("motionlock", name, collide_connected=1, lock_x=x, lock_y=y, lock_z=z, lock_angular_xy=r)
              for name, x, y, z, r in presets]
    joints.append(node("motionlock", "spin_z", lock_x=1, lock_y=1, lock_z=1, lock_angular_xy=1))
    lines = [("", (-9.0 + 6.0 * i, 4, 0), f"boxes{i}", (-9.0 + 6.0 * i, 4, 0), presets[i][0]) for i in range(4)]
    lines.append(("", (0, 8, 6), "platform0", (0, 8, 6), "spin_z"))
    return finish(s, b, about="""\
Motion locks, each holding a box to the world in its own way, left to right: at a point (it can only
turn), on a line along x (it slides along x), on the xz plane (it can move across it) and planar xy
(moves in that plane, turns only about z). A thin platform behind them is locked to spin about z only;
a 10 kg ball drops onto it.""",
                  ported="the sample's L key pushes the four boxes (impulse (5, 5, 5), angular (2, 2, 2)); Try "
                         "gives them that as a starting v and w.",
                  try_="push the boxes: v {5, 5, 5} and w {2, 2, 2} in boxes_points; drop the ball off centre.",
                  joints=joints, lines=lines)


LOCKED_JOINTS_PORTED = """\
the sample's lock checkboxes are the locks node's Lock X, Y and Z (live), its Rotation menu at free. Spin z
is Lock Angular XY on that node; spin x or y needs the lock lines' orient turned so their z is along that
axis (as Thin Wall Push Out's), and fixed rotation has no attribute (see Fixed Rotation). The force and
torque thresholds the sample sets are not read by it, so Breakable stays off. The L key's impulse: Try."""


@sample("Joints", "Locked Joints", SOURCE, frames=300)
def locked_joints(s):
    kinds = ["distance", "motor", "prismatic", "revolute", "weld", "wheel"]
    xs = [-12.5 + 5.0 * i for i in range(len(kinds))]
    b = Bodies(s.tag, {"angulardamping": 0.0, "enablesleep": 1})
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 1.0, 1.0, 0.5), f"""\
// Six boxes 10 m up, 5 m apart along x, one per joint type: {", ".join(kinds)}.
for (int i = 0; i < {len(kinds)}; ++i) body(set(-12.5 + 5.0 * i, 10, 0), identity);
""", cd=(0.9, 0.6, 0.2), note="density 1, angular damping 0.5, never sleeping", density=1.0, angulardamping=0.5,
             enablesleep=0)
    joints = [node("distance", "distance", collide_connected=1),
              node("motor", "motor", collide_connected=1, max_spring_force=1000.0, max_spring_torque=20.0),
              node("prismatic", "prismatic", collide_connected=1),
              node("revolute", "revolute", collide_connected=1),
              node("weld", "weld", collide_connected=1, enable_angular_spring=1, angular_stiffness=2.0,
                   angular_damping_ratio=0.5),
              node("wheel", "wheel", collide_connected=1, enable_suspension_spring=1, suspension_stiffness=1.0,
                   suspension_damping_ratio=0.7, enable_suspension_limit=1, lower_suspension_limit=-1.0,
                   upper_suspension_limit=1.0, enable_spin_motor=1, max_spin_torque=10.0,
                   spin_speed=math.degrees(1.0)),
              node("motionlock", "locks", collide_connected=1, lock_x=1, lock_y=1, lock_z=1, lock_angular_xy=0)]
    # The distance joint hangs the box 2 m below a world anchor above its top face; the motor joint's frames are at
    # its centre; the others are at the middle of its left face (x - 1), their frames the world axes.
    lines = [("", (xs[0], 13.0, 0.0), "boxes0", (xs[0], 11.0, 0.0), "distance"),
             ("", (xs[1], 10.0, 0.0), "boxes1", (xs[1], 10.0, 0.0), "motor")]
    lines += [("", (xs[i] - 1.0, 10.0, 0.0), f"boxes{i}", (xs[i] - 1.0, 10.0, 0.0), kinds[i]) for i in range(2, 6)]
    lines += motion_locks([(f"boxes{i}", (xs[i], 10.0, 0.0)) for i in range(6)], "locks")
    return finish(s, b, about="""\
Joints on bodies that motion locks also hold. Six boxes hang in the air on a distance, motor, prismatic,
revolute, weld and wheel joint to the world; each is also held at its centre by a motion lock to the world
(Sample::LockMotion), here with all three positions locked and rotation free, as the sample starts, so
the boxes stay where they are. With Lock Y off each box drops as far as its own joint lets it: the motor
box (its springs have no stiffness) falls to the ground, the wheel box sags on its soft suspension and
tilts, the revolute and weld boxes dip a few centimetres, the distance and prismatic boxes hold. Both ways
the boxes are where Box3D's are, to the millimetre.""",
                  ported=LOCKED_JOINTS_PORTED,
                  try_="turn Lock Y off on the locks node (live): the boxes drop as their joints let them. The L key "
                       "pushes the distance box: v {25, 0, 0} on its first point in boxes_points (an impulse of 100 on "
                       "4 kg), with Lock X off.",
                  joints=joints, lines=lines)


DRIVING_PORTED = """\
the sample drives by keyboard (W / S throttle, A / D steering, in its third-person view); here the keys
are keyframes on the wheel nodes, held as a key is: the rear node's Spin Speed is -30 rad/s (W, the
sample's Speed slider of 30) from frame 60 to 420, and the front node's Target Steering Angle -45
degrees (D) from frame 180 to 252 (keyed settings need their node's operation menu at Set Always).
Without the keys, as the sample starts, the car stands still. The wave height field is a HeightField SOP
on a Static Object (collider0). The speed and joint readouts are not drawn. The chassis follows Box3D's
path to within 4 cm over the 47 m it drives."""

# Driving's front and rear wheel joints: b3WheelJointDef as the sample fills it (the slider defaults).
DRIVING_WHEEL = dict(enable_suspension_spring=1, suspension_stiffness=4.0, suspension_damping_ratio=0.7,
                     enable_suspension_limit=1, lower_suspension_limit=-0.2, upper_suspension_limit=0.2,
                     max_spin_torque=5.0, spin_speed=0.0, steering_stiffness=10.0, steering_damping_ratio=0.7,
                     max_steering_torque=5.0, target_steering_angle=0.0, enable_steering_limit=1,
                     lower_steering_limit=-45.0, upper_steering_limit=45.0)


@sample("Joints", "Driving", SOURCE, frames=540)
def driving(s):
    # b3CreateWave( 50, 50, (4, 2, 4), 0.02, 0.04 ) on a static body at (-20, 0, -20): heights sin(2 pi 0.02 row)
    # times sin(2 pi 0.04 column), at b3ComputeCosSin, times 2; rows run along z, columns along x.
    field = height_field(s.tag, 50, 50, (4.0, 2.0, 4.0), origin=(-20.0, 0.0, -20.0), heights=VEX_FUNCTIONS + """\
int column = int(rint((@P.x + 20.0) / 4.0)), row = int(rint((@P.z + 20.0) / 4.0));
@height = cossin(2.0 * PI * 0.02 * row)[1] * cossin(2.0 * PI * 0.04 * column)[1];
""")
    b = Bodies(s.tag, {"fastrotation": 0})
    b.stream("chassis", b.box("chassis", 2.0, 0.5, 1.0), "body({0, 2.5, 0}, identity);", cd=(0.85, 0.3, 0.25),
             note="b3MakeBoxHull( 2, 0.5, 1 ), density 0.5", density=0.5)
    wheel_vex = f"""\
// The four wheels, their axles along z (the body's y turned to z): front left, front right, rear left, rear right.
// Every wheel shape names its bodies wheels0 to wheels3, so the joints find them whichever the Wheels menu picks.
vector at[] = array(set(1.5, 2, 0.8), set(1.5, 2, -0.8), set(-1.5, 2, 0.8), set(-1.5, 2, -0.8));
foreach (vector p; at) {{
    int pt = body(p, {vec4(Y_TO_Z)});
    setpointattrib(0, "name", pt, sprintf("wheels%d", pt));
}}
"""
    wheel = dict(cd=(0.2, 0.2, 0.22), merged=False, density=2.0, friction=3.0, fastrotation=1)
    shapes = [("sphere", "Sphere", b.sphere("sphere", 0.4), "a sphere of radius 0.4"),
              ("capsule", "Capsule", b.capsule("capsule", 0.125, 0.4), "b3Capsule { (0, -0.125, 0), (0, 0.125, 0), "
                                                                       "0.4 }"),
              ("cylinder", "Cylinder", b.cylinder("cylinder", 0.375, 0.4, -0.1875, 16),
               "b3CreateCylinder( 0.375, 0.4, -0.1875, 16 ): the Sides slider at 16")]
    b.choice("wheels", "Wheels", [(token, label, b.stream(token, shape, wheel_vex, note=f"{note}; density 2, "
                                                          "friction 3, fastrotation", **wheel))
                                  for token, label, shape, note in shapes])
    speed = -math.degrees(30.0)  # the rear wheels' Spin Speed while W is held: -Speed rad/s
    front = node("wheel", "front_wheels", enable_spin_motor=0, enable_steering=1, **DRIVING_WHEEL)
    rear = node("wheel", "rear_wheels", enable_spin_motor=1, enable_steering=0, **DRIVING_WHEEL)
    lines = [("chassis0", p, f"wheels{i}", p, "front_wheels" if i < 2 else "rear_wheels", X_TO_Y)
             for i, p in enumerate([(1.5, 2.0, 0.8), (1.5, 2.0, -0.8), (-1.5, 2.0, 0.8), (-1.5, 2.0, -0.8)])]
    lines.append(("", (-20.0, 0.0, -20.0), "chassis0", (0.0, 2.5, 0.0), "upright", Z_TO_Y))
    return finish(s, b, about="""\
A car on a rolling height field: a box chassis on four wheel joints, each with a 4 Hz suspension spring
and a travel limit of 0.2 m either way. The front wheels steer (a 10 Hz steering spring, up to 45
degrees), the rear ones are driven by spin motors of at most 5 N m, and a soft parallel joint to the
world (0.5 Hz) keeps the chassis upright. Here the car drives off along x, turns right for 1.2 s onto
+z, drives on and brakes from frame 420 (the motors held at 0).""",
                  ported=DRIVING_PORTED,
                  try_="change the keys on rear_wheels' Spin Speed and front_wheels' Target Steering Angle (live "
                       "settings), or another Wheels shape on the bodies object's Sample tab (restart).",
                  joints=[front, rear, node("parallel", "upright", collide_connected=1, stiffness=0.5,
                                            damping_ratio=1.0)],
                  lines=lines, statics=[field],
                  keys={("rear_wheels", "spin_speed"): [(1, 0.0), (60, speed), (420, 0.0)],
                        ("front_wheels", "target_steering_angle"): [(1, 0.0), (180, -45.0), (252, 0.0)]})


def vec4(q):
    return "{" + ", ".join(num(c) for c in q) + "}"
