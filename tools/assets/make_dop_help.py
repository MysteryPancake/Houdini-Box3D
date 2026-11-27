"""Help cards for the plugin's own DOP nodes (F1 on the node).

    hython tools/assets/make_dop_help.py <help folder>

Writes `<help folder>/nodes/dop/<node>.txt` in Houdini's wiki markup for Box3D Solver, Box3D Data and every Box3D
constraint relationship node. Houdini finds help for a compiled node there on HOUDINI_PATH (the digital assets carry
theirs as a Help section instead: b3d_*_help.py). Only the words around the parameters are written here: the
parameters, their folders and their help are read from the node types the plugin registers, so the cards follow the
code. Needs the plugin loaded (the build runs it from make_assets.py, after the plugin).

Never fails the build: problems print a warning, as the asset builders do.
"""

import os
import sys
import textwrap

import hou

# The DOP parameters every data node has, or that only pick how parameters apply: left out of the parameter lists
# and described once in the card's text.
STANDARD = {"defaultparmop", "sharedata", "activation", "activationrules", "group", "dataname", "uniquedataname",
            "addaffectors", "solverperobject"}
WIDTH = 116  # line width of the text written

COMMON_DATA = """\
Each parameter has an operation menu beside it, as on every DOP data node: Set Initial applies it when the data is
created, Set Always every step (so an animated value acts), Set Never leaves the data as it is. Default Operation sets
them all."""

SOLVER = {
    "name": "box3d_solver",
    "title": "Box3D Solver",
    "summary": "Simulates rigid bodies with the Box3D physics engine, like the RBD Bullet Solver DOP.",
    "tags": "dynamics, rbd, box3d, solver",
    "intro": """\
The Box3D Solver simulates the objects wired into it with Box3D, Erin Catto's rigid body engine. It takes the same
objects and attributes as the RBD Bullet Solver: RBD Packed Objects (or Box3D Packed Objects) with one body per packed
piece, RBD Objects as one body each, Static Objects as colliders, Constraint Networks for joints, and Force DOPs.
Per-object settings come from Box3D Data on the object, or from point attributes of the same name, which win.

Joints use Box3D's own constraint relationships (Box3D Weld, Spherical, Revolute and the rest), or SideFX's Glue,
Hard, Spring, Soft and Cone Twist, which are mapped onto Box3D joints as Bullet would treat them.

Most Bullet setups work with only the solver swapped. Sleeping and enabling follow Box3D: `enablesleep`, `awake` and
`enabled` are read, `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not.
The Box3D Solver SOP wraps this node with its inputs and outputs, as RBD Bullet Solver wraps its DOP.""",
    "inputs": [("Objects", "The objects to solve."),
               ("Data", "Data to attach to the solver, such as Geometry Copy data.")],
    "standard": COMMON_DATA + """

Group, Data Name and Make Objects Mutual Affectors are the usual solver parameters: which objects get the solver, the
name it is attached under, and whether the objects collide with each other.""",
    "related": [("Box3D Data", "dop/box3d_data"), ("Box3D Packed Object", "dop/box3d_packedobject"),
                ("Box3D Solver SOP", "sop/box3d_solver"), ("RBD Bullet Solver", "dop/bulletrbdsolver")],
}

DATA = {
    "name": "box3d_data",
    "title": "Box3D Data",
    "summary": "The Box3D settings of an object: shape, body, material, contacts and sleep.",
    "tags": "dynamics, rbd, box3d",
    "intro": """\
Box3D Data attaches the Box3D Solver's per-object settings to an object. Each is the fallback for a point attribute
of the same name: a piece's own attribute wins, so this node sets the values for pieces that have none. RBD Packed
Object writes several attributes on every piece (mass, density, bounce, friction, ...), so set those there, or use
Box3D Packed Object, which writes only what differs from these defaults.

On an RBD Object (one body per object), every setting here applies to the whole object.""",
    "inputs": [("Objects", "The objects to attach the data to.")],
    "standard": COMMON_DATA,
    "related": [("Box3D Solver", "dop/box3d_solver"), ("Box3D Packed Object", "dop/box3d_packedobject"),
                ("Box3D Configure", "sop/box3d_configure")],
}

# Each joint type: what it does, in a sentence or two.
JOINTS = {
    "weld": ("Weld", "Holds two pieces together rigidly, or softly with linear and angular springs."),
    "spherical": ("Spherical", "A ball and socket: the pieces share a point and turn freely about it, within an optional "
                  "cone and twist limit, with an optional spring and motor."),
    "revolute": ("Revolute", "A hinge: the pieces turn about the z axis of the joint's frame, with optional limits, "
                 "spring and motor."),
    "prismatic": ("Prismatic", "A slider: the second piece slides along the x axis of the first anchor's frame, with "
                  "optional limits, spring and motor."),
    "distance": ("Distance", "Keeps two points a distance apart, within an optional length range, with an optional "
                 "spring and motor: a rope, rod or spring."),
    "motor": ("Motor", "Drives the second piece relative to the first by a velocity and springs, with a capped force "
              "and torque."),
    "parallel": ("Parallel", "Keeps the z axes of the two frames parallel with a spring."),
    "wheel": ("Wheel", "A wheel on a chassis: suspension along an axis, a spin motor and steering."),
    "filter": ("Filter", "No constraint: the two pieces only stop colliding with each other."),
    "motionlock": ("Motion Lock", "Holds the second piece to a point, line or plane of the first anchor's frame, and "
                   "can stop it turning except about that frame's z axis. It is how a piece's axes are locked."),
}

JOINT_INTRO = """\
A Box3D {label} Constraint Relationship holds the settings of the Box3D {label} joint. Wire it into a Constraint
Network's second input: each constraint line whose `constraint_name` is this node's Data Name (`{label_name}` by default)
becomes a {label} joint between the two pieces its points name (an empty name is the world). The line's points set
where the joint is and, with `orient`, which way its axes point, on the pieces as they were at the start.

{what}

A float primitive attribute on a line named like a parameter here multiplies its value for that joint (or replaces it,
with Attributes Replace Values on), and other attributes replace it, so set the typical value here and vary it per
joint, for example with Box3D Constraint Properties. A float of -3.4e38 or a toggle of -1 means not set. With Breakable
on, a joint past its Break Force or Torque goes into the `broken` group, or switches to its `next_constraint_name`.
After each step the solver writes `force`, `torque`, `distance` and `angle` onto each line."""

JOINT_STANDARD = COMMON_DATA + """ Data Name is what `constraint_name` picks this node by."""


def para(text):
    """Text rewrapped to WIDTH, paragraph by paragraph."""
    return "\n\n".join(textwrap.fill(" ".join(p.split()), WIDTH) for p in text.split("\n\n"))


def item(label, parm_id, text, indent="    "):
    lines = [f"{label}:", f"{indent}#id: {parm_id}"]
    if text:
        lines += textwrap.wrap(" ".join(text.split()), WIDTH - len(indent), initial_indent=indent,
                               subsequent_indent=indent)
    return "\n".join(lines)


def parameters(node_type):
    """The @parameters section: each folder a heading (nested folders a lower one), each parameter an entry with its
    help, the operation menus and standard parameters left out."""
    out = []

    def walk(templates, depth):
        loose = []
        for t in templates:
            if t.type() == hou.parmTemplateType.Folder:
                if loose:
                    out.append("\n\n".join(loose))
                    loose = []
                children = [c for c in t.parmTemplates()]
                # A folder that only wraps others (the solver's Data Options) gets no heading of its own.
                if t.label() not in ("Data Options",) and depth < 3:
                    out.append(("=" * (depth + 2)) + f" {t.label()} " + ("=" * (depth + 2)))
                    walk(children, depth + 1)
                else:
                    walk(children, depth)
            elif t.type() == hou.parmTemplateType.FolderSet or t.isHidden():
                continue
            elif t.name().startswith("parmop_") or t.name() in STANDARD:
                continue
            else:
                loose.append(item(t.label(), t.name(), t.help()))
        if loose:
            out.append("\n\n".join(loose))

    walk(node_type.parmTemplateGroup().entries(), 0)
    return "\n\n".join(out)


def card(spec, node_type):
    parts = [f"= {spec['title']} =", "",
             "#type: node", "#context: dop", f"#internal: {spec['name']}", f"#icon: DOP/{spec['name']}",
             f"#tags: {spec['tags']}", "", f'"""{spec["summary"]}"""', "", para(spec["intro"]), "", "@inputs", ""]
    for label, text in spec["inputs"]:
        parts += [item(label, label.lower(), text), ""]
    parts += ["@parameters", "", para(spec["standard"]), "", parameters(node_type), "", "@related", ""]
    parts += [f"- [{label}|Node:{path}]" for label, path in spec["related"]]
    return "\n".join(parts) + "\n"


def joint_spec(type_name):
    label, what = JOINTS[type_name]
    return {
        "name": f"box3d_{type_name}conrel",
        "title": f"Box3D {label} Constraint Relationship",
        "summary": f"The settings of a Box3D {label} joint, for the constraint lines of a Constraint Network.",
        "tags": "dynamics, rbd, box3d, constraints",
        "intro": JOINT_INTRO.format(label=label, label_name=label, what=what),
        "inputs": [("Objects", "The objects the relationship applies to (a Constraint Network sets this up).")],
        "standard": JOINT_STANDARD,
        "related": [("Box3D Solver", "dop/box3d_solver"), ("Constraint Network", "dop/constraintnetwork"),
                    ("Box3D Constraint Properties", "sop/box3d_constraint_properties")],
    }


def specs():
    return [SOLVER, DATA] + [joint_spec(t) for t in JOINTS]


def build(help_dir):
    """Writes every card into help_dir/nodes/dop. Returns the paths written."""
    out_dir = os.path.join(help_dir, "nodes", "dop")
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for spec in specs():
        node_type = hou.nodeType(hou.dopNodeTypeCategory(), spec["name"])
        if node_type is None:
            print(f"make_dop_help: WARNING: no DOP node {spec['name']} (is the plugin loaded?): no help card")
            continue
        path = os.path.join(out_dir, spec["name"] + ".txt")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(card(spec, node_type))
        written.append(path)
    print(f"Box3D DOP help cards: {len(written)} in {os.path.normpath(out_dir)}")
    return written


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(0)
    build(sys.argv[1])
