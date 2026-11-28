"""Box3D's Ragdoll samples (samples/sample_ragdoll.cpp): human.c's ragdoll (human.py) dropped on a box, a mesh, a slope,
or many of them on a mesh."""

from .common import XorShift, Bodies, finish, sample
from .human import Humans
from .joints import Z_TO_Y, node

SOURCE = "sample_ragdoll.cpp"

# What every ragdoll port says of the ragdoll.
RAGDOLL_PORTED = """\
each bone is a capsule body (bullet_primT / bullet_primR place its capsule as human.c's), its joint a
network line from the parent's frame to the bone's (the two frames start apart, as in Box3D, so the
springs pull the pose a little at the start). The calf is two capsules, the shin and the foot at half
density: two named pieces of its packed shape fitted as capsules (piece_georep, piece_densityscale on
the calf_l_shin_named / _foot_named wrangles). The joint nodes group the bones whose settings match:
spine, neck, hips, knees, shoulders, elbows."""

RAGDOLL_TRY = ("the joint nodes' Max Motor Torque (the sample's Joint Friction slider times each bone's share), "
               "Stiffness (Hertz) and Damping Ratio are live: lower the torque and the ragdoll goes limp.")


def joint_attribs():
    return {"collision_groupindex": 0}


@sample("Ragdoll", "Box", SOURCE, frames=300)
def box(s):
    b = Bodies(s.tag, joint_attribs())
    b.ground(20.0)
    people = Humans(b, [((0.0, 2.0, 0.0), 1)], friction_torque=5.0, hertz=1.0, damping_ratio=0.7)
    return finish(s, b, about="""\
A ragdoll dropped from 2 m onto a ground box: twelve capsules on spherical joints (spine, neck, hips,
shoulders) and revolute ones (knees, elbows), each with its limits, a 1 Hz spring toward the pose it
starts in and a friction motor of 5 N m (scaled per bone). It lands on its feet, stands a moment, topples
over and lies still by frame 130.""",
                  ported=RAGDOLL_PORTED + " It follows Box3D's ragdoll to 0.1 mm all the way.", try_=RAGDOLL_TRY, joints=people.nodes(), lines=people.lines())


@sample("Ragdoll", "Mesh", SOURCE, frames=300)
def mesh(s):
    b = Bodies(s.tag, joint_attribs())
    b.mesh("grid", """\
// b3CreateGridMesh( 20, 20, 2 ): a flat 40 x 40 mesh at y = 0, with four walls (box hulls on the same static body).
grid_mesh("grid", 20, 20, 2.0, {1, 1, 1}, {0, 0, 0});
""")
    b.stream("walls_x", b.box("walls_x", 20.0, 5.0, 0.1), "static_body({0, 5, -20}, identity);\n"
             "static_body({0, 5, 20}, identity);", cd=(0.55, 0.6, 0.65),
             note="b3MakeTransformedBoxHull( 20, 5, 0.1 ) at z = -20 and 20, on the mesh's body in Box3D")
    b.stream("walls_z", b.box("walls_z", 0.1, 5.0, 20.0), "static_body({-20, 5, 0}, identity);\n"
             "static_body({20, 5, 0}, identity);", cd=(0.55, 0.6, 0.65))
    people = Humans(b, [((0.0, 1.0, 0.0), 1)], friction_torque=5.0, hertz=2.0, damping_ratio=0.7)
    # Human_CreateParallelAnchors: each bone on a parallel joint (8 Hz, damping 1, at most 800 N m) to a kinematic
    # anchor at its starting pose, the frames turned so their z is the world's y: it keeps each bone's up as it was.
    anchors = [("", p, name, p, "upright", Z_TO_Y) for name, p, _ in people.poses()]
    return finish(s, b, about="""\
A ragdoll standing on a grid mesh, each of its bones held upright by a parallel joint to the world
(8 Hz, at most 800 N m): the joints keep every bone's axes turned as they start, but not where they
are, so the ragdoll sags onto its feet and stands, swaying on its knees and spine springs (2 Hz).""",
                  ported=RAGDOLL_PORTED + """
Human_CreateParallelAnchors' kinematic anchors do not move, so the parallel joints go to the world
(the upright node). It follows Box3D's ragdoll to 0.1 mm.""", try_=RAGDOLL_TRY + " Turn off the upright node's Limit Torque, or lower its "
                                           "Stiffness: the ragdoll falls.",
                  joints=people.nodes() + [node("parallel", "upright", stiffness=8.0, damping_ratio=1.0,
                                                limit_torque=1, max_torque=800.0)],
                  lines=people.lines() + anchors)


@sample("Ragdoll", "Pile", SOURCE, frames=480)
def pile(s):
    count = 20
    rng = XorShift(42)  # g_randomSeed = 42
    a = 0.1 * count
    people = []
    for i in range(count):
        offset = rng.vec3((-a, -a, -a), (a, a, a))
        people.append(((offset[0], 2.0, offset[2]), i))
    b = Bodies(s.tag, joint_attribs())
    b.mesh("grid", """\
// b3CreateGridMesh( 20, 20, 1 ) on a static body at (0, -1, 0): a flat 20 x 20 mesh at y = -1.
grid_mesh("grid", 20, 20, 1.0, {1, 1, 1}, {0, -1, 0});
""")
    crowd = Humans(b, people, friction_torque=10.0, hertz=0.5, damping_ratio=0.7)
    return finish(s, b, about=f"""\
{count} ragdolls dropped together at random spots within 2 m of the middle, all at 2 m: they land in a
heap on a grid mesh. Friction motors of 10 N m and soft springs (0.5 Hz). Each has its own groupIndex
(0 to {count - 1}), so a ragdoll's spine and thighs never touch each other (but the first's: 0 is no group).""",
                  ported=RAGDOLL_PORTED + """ The spots are the sample's RandomVec3 from seed 42, so the
ragdolls start where Box3D's do. The bodies are made bone by bone, not ragdoll by ragdoll as in Box3D,
so their contacts are solved in another order: the heap parts from Box3D's once they meet (by frame
60) and its ragdolls come to rest up to 0.5 m from Box3D's.""",
                  try_=RAGDOLL_TRY, joints=crowd.nodes(), lines=crowd.lines())


@sample("Ragdoll", "Incline", SOURCE, frames=480)
def incline(s):
    b = Bodies(s.tag, joint_attribs())
    b.mesh("slope", """\
// b3CreateGridMesh( 4, 4, 2 ) on a static body at (-10, 2, 0) turned -0.2 pi about z: an 8 x 8 slope, 36 degrees.
grid_mesh_turned("slope", 4, 4, 2.0, {1, 1, 1}, axis_angle({0, 0, 1}, -0.2 * PI), {-10, 2, 0});
""")
    b.mesh("floor", """\
// The same mesh at the origin with the shape's scale 4: a flat 32 x 32 floor at y = 0.
grid_mesh("floor", 4, 4, 2.0, {4, 4, 4}, {0, 0, 0});
""")
    people = Humans(b, [((-12.0, 6.0, 0.0), 1)], friction_torque=10.0, hertz=2.0, damping_ratio=0.7)
    # Step(): once its time passes 2 s (before step 121, the step that makes frame 123), the joint friction torque
    # drops to 0.5 and the springs to 0.5 Hz.
    switch = 123
    keys = {}
    for name, share in people.motor_nodes().items():
        keys[(name, "max_motor_torque")] = [(1, share * 10.0), (switch, share * 0.5)]
        keys[(name, "stiffness")] = [(1, 2.0), (switch, 0.5)]
    return finish(s, b, about="""\
A ragdoll dropped onto a 36 degree slope, held fairly stiff (friction motors of 10 N m, 2 Hz springs).
After 2 s (frame 123) its joints go slack (0.5 N m, 0.5 Hz) and it slides and tumbles down onto the
floor.""", ported=RAGDOLL_PORTED + """
The sample's Step() changes the joints at 2 s: keys on the joint nodes' Max Motor Torque and Stiffness.
It follows Box3D's ragdoll to 0.1 mm for 2 s and comes to rest within 1 cm of it.""",
                  try_="move the keys on the joint nodes (Max Motor Torque, Stiffness), or delete them.",
                  joints=people.nodes(), lines=people.lines(), keys=keys)
