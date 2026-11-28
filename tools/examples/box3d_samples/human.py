"""Box3D's ragdoll (extern/box3d/shared/human.c, CreateHuman), for the samples that drop people: twelve capsule bodies
joined by spherical and revolute joints with limits, springs and friction motors, and filter joints between the thighs
and between the neck and each upper arm.

The calf's foot is a second capsule on the calf's body, at half density: the shin and foot are two named pieces of the
calf's packed shape, each fitted as a capsule (bullet_georep), the foot's densityscale 0.5 (Bodies.parts).
"""

import math

import hou

from .common import num, vec
from .joints import node

# Box3D quaternions are { v, s }; these tuples are (x, y, z, w), as orient is.


def qmul(a, b):
    """b3MulQuat( a, b )."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + bw * ax + ay * bz - az * by, aw * by + bw * ay + az * bx - ax * bz,
            aw * bz + bw * az + ax * by - ay * bx, aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    """b3RotateVector( q, v )."""
    x, y, z, w = q
    tx, ty, tz = 2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def qnorm(q):
    """b3NormalizeQuat( q )."""
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


# b3_color* of human.c's colorize, used here for every ragdoll (the samples draw theirs uncoloured).
SHIRT = (0x48 / 255.0, 0xD1 / 255.0, 0xCC / 255.0)  # MediumTurquoise
PANTS = (0x1E / 255.0, 0x90 / 255.0, 0xFF / 255.0)  # DodgerBlue
SKIN = [(0xFF / 255.0, 0xDE / 255.0, 0xAD / 255.0), (0xFF / 255.0, 0xFF / 255.0, 0xE0 / 255.0),
        (0xCD / 255.0, 0x85 / 255.0, 0x3F / 255.0), (0xD2 / 255.0, 0xB4 / 255.0, 0x8C / 255.0)]  # by groupIndex % 4


class Bone:
    """One bone of CreateHuman: its body (referenceFrame, the capsule, the material, groupIndex -group when `own_group`)
    and the joint to its parent (localFrameA on the parent, localFrameB on the bone, limits in degrees,
    jointFriction)."""

    def __init__(self, name, parent, p, q, capsule, friction, rolling, own_group, colour, joint=None, frame_a=None,
                 frame_b=None, swing=0.0, twist=0.0, joint_friction=1.0, foot=None):
        self.name, self.parent, self.p, self.q = name, parent, p, q
        self.capsule, self.friction, self.rolling, self.own_group, self.colour = capsule, friction, rolling, own_group, \
            colour
        self.joint, self.frame_a, self.frame_b = joint, frame_a, frame_b
        self.swing, self.twist, self.joint_friction, self.foot = swing, twist, joint_friction, foot


ID_FRAME = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

# human.c's bones, in its order (bone_pelvis ... bone_lower_arm_r). Capsules are (center1, center2, radius) in the
# body's frame; a quaternion { { x, y, z }, s } is written (x, y, z, s).
BONES = [
    Bone("pelvis", None, (0.000000, 0.996219, -0.023868), (1.0, 0.0, 0.0, 0.0),
         ((0.040000, 0.000001, 0.000000), (-0.040000, -0.000001, 0.000000), 0.150000), 0.6, 0.1, False, "pants"),
    Bone("spine_01", "pelvis", (0.000000, 1.017288, -0.024882), (1.0, 0.0, 0.0, 0.0),
         ((0.029876, -0.146581, 0.006260), (-0.029876, -0.146574, 0.006260), 0.145663), 0.5, 0.1, True, "shirt",
         "spherical", ((0.000000, -0.021069, 0.001014), (-0.642737, 0.0, 0.0, -0.766087)),
         ((0.0, 0.0, 0.0), (-0.707107, 0.0, 0.0, -0.707107)), 35.0, 17.5),
    Bone("spine_03", "spine_01", (0.000000, 1.267766, -0.022320), (1.0, 0.0, 0.0, 0.0),
         ((0.063996, -0.117434, -0.040199), (-0.063996, -0.117432, -0.040199), 0.165004), 0.5, 0.1, False, "shirt",
         "spherical", ((0.000000, -0.250478, -0.002562), (-0.642766, 0.0, 0.0, -0.766063)),
         ((0.0, 0.0, 0.0), (-0.707107, 0.0, 0.0, -0.707107)), 35.0, 17.5),
    Bone("neck", "spine_03", (0.000000, 1.498783, 0.024462), (0.987879, 0.0, 0.0, 0.155228),
         ((0.000000, -0.212340, 0.000000), (0.000000, -0.087340, 0.000000), 0.110000), 0.2, 0.05, False, "skin",
         "spherical", ((0.000000, -0.231017, -0.046781), (-0.516157, -0.000002, 0.000003, -0.856494)),
         ((0.0, 0.0, 0.0), (-0.707108, -0.000002, 0.000002, -0.707106)), 30.0, 17.5),
    Bone("thigh_l", "pelvis", (0.092175, 0.971562, -0.011177), (0.471412, 0.529510, -0.499888, -0.497495),
         ((-0.047269, 0.000001, 0.000000), (-0.379769, 0.000005, 0.000000), 0.091539), 0.6, 0.1, True, "pants",
         "spherical", ((0.092175, 0.024656, -0.012691), (-0.460186, -0.365368, -0.637662, 0.498118)),
         ((0.0, 0.0, 0.0), (-0.499997, -0.500003, 0.500002, 0.499998)), 30.0, 10.0),
    Bone("calf_l", "thigh_l", (0.118720, 0.534541, -0.035362), (0.521188, 0.480597, -0.546380, -0.445935),
         ((-0.445000, 0.000001, 0.000000), (0.005000, -0.000001, 0.000000), 0.080000), 0.1, 0.0, False, "pants",
         "revolute", ((-0.438494, -0.000175, 0.000000), (0.344866, -0.938652, 0.000002, -0.000008)),
         ((0.0, 0.0, 0.0), (-0.008182, -0.999967, 0.000001, -0.000007)), 0.0, 30.0,
         foot=((-0.456371, 0.129321, 0.000013), (-0.359977, 0.014394, 0.000013), 0.070000)),
    Bone("thigh_r", "pelvis", (-0.092175, 0.971562, -0.011177), (-0.497495, 0.499888, 0.529510, -0.471412),
         ((0.047269, -0.000001, 0.000000), (0.379769, -0.000005, 0.000000), 0.092587), 0.6, 0.1, True, "pants",
         "spherical", ((-0.092175, 0.024656, -0.012691), (0.380666, 0.491846, 0.488644, -0.611889)),
         ((0.0, 0.0, 0.0), (-0.500002, 0.499998, -0.499998, 0.500002)), 30.0, 10.0),
    Bone("calf_r", "thigh_r", (-0.118720, 0.534541, -0.035362), (-0.445935, 0.546380, 0.480597, -0.521188),
         ((0.445000, -0.000002, 0.000003), (-0.005000, 0.000000, 0.000003), 0.080000), 0.1, 0.0, False, "pants",
         "revolute", ((0.438494, 0.000175, 0.000000), (0.000007, 0.000005, -0.344860, -0.938654)),
         ((0.0, 0.0, 0.0), (0.000006, 0.000005, 0.008188, -0.999966)), 0.0, 30.0,
         foot=((0.369113, -0.006830, 0.000017), (0.465507, -0.121757, 0.000017), 0.070000)),
    Bone("upperarm_l", "spine_03", (0.185817, 1.443630, 0.031306), (0.560220, -0.307179, 0.366233, -0.676512),
         ((0.296038, 0.004386, -0.000794), (-0.016086, 0.004384, -0.016127), 0.065000), 0.6, 0.1, False, "shirt",
         "spherical", ((0.185817, -0.175865, -0.053625), (-0.559466, -0.213784, -0.670199, 0.438323)),
         ((0.0, 0.0, 0.0), (-0.500953, -0.502617, -0.509318, -0.486844)), 45.0, 20.0, joint_friction=0.8),
    Bone("lowerarm_l", "upperarm_l", (0.347683, 1.193455, 0.029378), (0.406736, -0.493012, 0.089135, -0.763911),
         ((0.312329, -0.008256, -0.016454), (0.033153, 0.008259, -0.002741), 0.060000), 0.3, 0.1, False, "skin",
         "revolute", ((0.297979, 0.000361, 0.000000), (0.916726, 0.399352, 0.000839, -0.011456)),
         ((0.0, 0.0, 0.0), (0.999922, -0.003857, 0.004659, -0.010908)), 0.0, 35.0, joint_friction=0.06),
    Bone("upperarm_r", "spine_03", (-0.185817, 1.443630, 0.031306), (0.676512, 0.366233, 0.307179, 0.560220),
         ((-0.300237, -0.000001, -0.000319), (0.011887, 0.000001, 0.015013), 0.065000), 0.6, 0.1, False, "shirt",
         "spherical", ((-0.185817, -0.175865, -0.053625), (0.213793, 0.559449, 0.438334, -0.670204)),
         ((0.0, 0.0, 0.0), (-0.509319, 0.486843, 0.500954, -0.502617)), 45.0, 20.0, joint_friction=0.8),
    Bone("lowerarm_r", "upperarm_r", (-0.347683, 1.193455, 0.029378), (0.763911, 0.089135, 0.493012, 0.406736),
         ((-0.312649, -0.000003, 0.016471), (-0.032986, 0.000000, 0.002732), 0.060000), 0.3, 0.1, False, "skin",
         "revolute", ((-0.297979, -0.000361, 0.000000), (-0.405444, 0.914059, 0.010541, 0.000852)),
         ((0.0, 0.0, 0.0), (-0.002794, 0.999936, 0.010042, 0.004364)), 0.0, 35.0, joint_friction=0.06),
]

BY_NAME = {bone.name: bone for bone in BONES}

# The filter joints CreateHuman adds: the pairs that never collide besides jointed ones.
FILTERS = [("thigh_l", "thigh_r"), ("neck", "upperarm_l"), ("neck", "upperarm_r")]

# The joint nodes: bones whose joints share every setting share a node (named for them). Box3D's limits, by bone.
JOINT_NODES = {"spine_01": "spine", "spine_03": "spine", "neck": "neck", "thigh_l": "hips", "thigh_r": "hips",
               "calf_l": "knees", "calf_r": "knees", "upperarm_l": "shoulders", "upperarm_r": "shoulders",
               "lowerarm_l": "elbows", "lowerarm_r": "elbows"}


def capsule_shape(b, tag, centre1, centre2, radius):
    """b3Capsule { centre1, centre2, radius } in the body's frame: Bodies.capsule (along y) turned so its +y end is
    centre2 and moved to the centres' midpoint (bullet_primT / bullet_primR)."""
    d = tuple(c2 - c1 for c1, c2 in zip(centre1, centre2))
    length = math.sqrt(sum(c * c for c in d))
    mid = tuple(0.5 * (c1 + c2) for c1, c2 in zip(centre1, centre2))
    axis = hou.Vector3(0.0, 1.0, 0.0).cross(hou.Vector3(d))
    turn = (0.0, 0.0, 0.0)
    if axis.length() > 1e-9 * length:
        angle = math.degrees(math.atan2(axis.length(), d[1]))
        euler = hou.Quaternion(angle, axis.normalized()).extractEulerRotates("xyz")
        turn = tuple(euler)
    elif d[1] < 0.0:
        turn = (180.0, 0.0, 0.0)
    return b.shifted(b.capsule(tag, 0.5 * length, radius), mid, rotate=turn)


class Humans:
    """CreateHuman( position, frictionTorque, hertz, dampingRatio, groupIndex ) for each of `people`, [(position,
    groupIndex)], on the Bodies `b` (which needs the attribute collision_groupindex). One stream per bone (`<bone><n>`, n the
    person's index). nodes() and lines() are the joints for finish()."""

    def __init__(self, b, people, friction_torque, hertz, damping_ratio):
        self.people = people
        self.friction_torque, self.hertz, self.damping_ratio = friction_torque, hertz, damping_ratio
        listed = ", ".join(vec(p) for p, _ in people)
        groups = ", ".join(str(int(g)) for _, g in people)
        for bone in BONES:
            colours = {"pants": PANTS, "shirt": SHIRT}
            group = "-groups[i]" if bone.own_group else "0"
            capsules = f"b3Capsule {{ {vec(bone.capsule[0])}, {vec(bone.capsule[1])}, {num(bone.capsule[2])} }}"
            if bone.foot:
                shape = b.parts(bone.name, [("shin", capsule_shape(b, f"{bone.name}_shin", *bone.capsule)),
                                            ("foot", capsule_shape(b, f"{bone.name}_foot", *bone.foot), 0.5)])
                capsules += (f" and the foot's\n// {{ {vec(bone.foot[0])}, {vec(bone.foot[1])}, "
                             f"{num(bone.foot[2])} }} at half density")
            else:
                shape = capsule_shape(b, bone.name, *bone.capsule)
            skin_pick = ""
            if bone.colour == "skin":  # skinColors[ groupIndex % 4 ]
                skin_pick = ("    vector skin[] = array(" + ", ".join(vec(c) for c in SKIN) + ");\n"
                             '    setpointattrib(0, "Cd", pt, skin[groups[i] % 4]);\n')
            b.stream(bone.name, shape, f"""\
// CreateHuman's {bone.name}: {capsules}, on a body at
// the person's position plus the bone's reference frame. groupIndex {'-groupIndex' if bone.own_group else '0'}.
vector at[] = array({listed});
int groups[] = array({groups});
foreach (int i; vector o; at) {{
    int pt = body(o + {vec(bone.p)}, {vec(bone.q)});
    setpointattrib(0, "collision_groupindex", pt, {group});
{skin_pick}}}
""", cd=colours.get(bone.colour, SKIN[0]), friction=bone.friction, rollingresistance=bone.rolling)

    def nodes(self):
        """The relationship nodes: one per bone group of JOINT_NODES, and the filter."""
        out, seen = [], set()
        for bone in BONES:
            name = JOINT_NODES.get(bone.name)
            if name is None or name in seen:
                continue
            seen.add(name)
            torque = bone.joint_friction * self.friction_torque
            spring = dict(enable_spring=int(self.hertz > 0.0), stiffness=self.hertz, damping_ratio=self.damping_ratio,
                          enable_motor=1, max_motor_torque=torque)
            if bone.joint == "spherical":
                out.append(node("spherical", name, enable_cone_limit=1, cone_angle=bone.swing, enable_twist_limit=1,
                                lower_twist_angle=-bone.twist, upper_twist_angle=bone.twist, **spring))
            else:
                out.append(node("revolute", name, enable_limit=1, lower_angle=-bone.twist, upper_angle=bone.twist,
                                motor_speed=0.0, **spring))
        out.append(node("filter", "no_collide"))
        return out

    def motor_nodes(self):
        """(node name, the bone's jointFriction) for each joint node: Human_SetJointFrictionTorque sets each node's
        Max Motor Torque to jointFriction times the torque."""
        out = {}
        for bone in BONES:
            name = JOINT_NODES.get(bone.name)
            if name is not None:
                out.setdefault(name, bone.joint_friction)
        return out

    def lines(self):
        """The network lines: each bone's joint to its parent at its two frames (world positions and rotations as the
        bodies start), and the filters."""
        out = []
        for n, (position, _) in enumerate(self.people):
            for bone in BONES:
                pose_p = add(position, bone.p)
                if bone.parent is None:
                    continue
                parent = BY_NAME[bone.parent]
                parent_p = add(position, parent.p)
                (pa, qa), (pb, qb) = bone.frame_a, bone.frame_b
                qa, qb = qnorm(qa), qnorm(qb)
                out.append((f"{parent.name}{n}", add(parent_p, qrot(parent.q, pa)), f"{bone.name}{n}",
                            add(pose_p, qrot(bone.q, pb)), JOINT_NODES[bone.name], qmul(parent.q, qa),
                            qmul(bone.q, qb)))
            for a, c in FILTERS:
                pa, pc = add(position, BY_NAME[a].p), add(position, BY_NAME[c].p)
                out.append((f"{a}{n}", pa, f"{c}{n}", pc, "no_collide"))
        return out

    def poses(self):
        """(body name, position, rotation) of every bone as it starts."""
        return [(f"{bone.name}{n}", add(position, bone.p), bone.q) for n, (position, _) in enumerate(self.people)
                for bone in BONES]
