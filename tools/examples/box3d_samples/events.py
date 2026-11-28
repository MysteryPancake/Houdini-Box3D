"""Box3D's Events samples (samples/sample_events.cpp) that have something to simulate."""

from .common import Bodies, finish, sample
from .joints import node

SOURCE = "sample_events.cpp"


@sample("Events", "Joint", SOURCE, frames=300)
def joint(s):
    b = Bodies(s.tag, {"enablesleep": 1})
    b.ground(20.0)
    b.stream("boxes", b.box("boxes", 1.0, 1.0, 0.5), """\
// Four boxes 10 m up (the sample's motor and wheel joint boxes, at -7.5 and 12.5, are compiled out).
vector at[] = array({-12.5, 10, 0}, {-2.5, 10, 0}, {2.5, 10, 0}, {7.5, 10, 0});
foreach (vector p; at) body(p, identity);
""", cd=(0.9, 0.6, 0.2), note="density 1, sleep off", density=1.0, enablesleep=0)
    breaks = {"collide_connected": 1, "breakable": 1, "force_threshold": 3000.0, "torque_threshold": 10000.0}
    joints = [node("distance", "rope", **breaks), node("prismatic", "slider", **breaks),
              node("revolute", "hinge", **breaks),
              node("weld", "weld", enable_angular_spring=1, angular_stiffness=2.0, angular_damping_ratio=0.5,
                   **breaks)]
    lines = [("", (-12.5, 13, 0), "boxes0", (-12.5, 11, 0), "rope"),
             ("", (-3.5, 10, 0), "boxes1", (-3.5, 10, 0), "slider"),
             ("", (1.5, 10, 0), "boxes2", (1.5, 10, 0), "hinge"),
             ("", (6.5, 10, 0), "boxes3", (6.5, 10, 0), "weld")]
    return finish(s, b, about="""\
Four boxes on breakable joints to the world, left to right: a 2 m distance joint at its top, a prismatic
joint (slides along x), a revolute joint at its left edge (swings down) and a weld with a 2 Hz angular
spring (sags and bounces). Each breaks past 3000 N or 10000 N m, which the boxes' weight never reaches:
in the sample you pull them with the mouse to break them.""",
                  ported="""\
Box3D's joint events become Breakable on the nodes: a broken line goes into the network's broken group.
The sample's motor and wheel joints are compiled out (#if 0), so the boxes are 10 m apart in two places.""",
                  try_="lower Break Force on a node below the box's weight (4 kg: 40 N): that joint breaks.",
                  joints=joints, lines=lines)
