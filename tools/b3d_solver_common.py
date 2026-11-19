"""Box3D Solver SOP: constants and the helpers shared by the builders (parameter templates, tabs, folders,
switches, node-parameter setters)."""

import re

import hou

import b3d_hda
from b3d_hda import collapsible, labels, value_templates


TYPE_NAME = "box3d_solver"  # a SOP; the DOP of the same name lives in another node table
LABEL = "Box3D Solver"
ICON = "DOP_box3d_solver"  # installed by the build for the DOP (config/Icons)
PACKED_OBJECT = "box3d_packedobject"

# Colours of the subnet inputs, IN_* nulls and OUT_* nodes by index: constraints pink.
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}
COLLISION_TYPE = "collisiontype"  # menu: 0 Static, 1 Animated, 2 Deforming (RBD Bullet Solver's Collision Type)
OVERWRITE_FROM_SOP = "overwritefromsop"  # the Box3D Packed Object's toggle: copy attributes from the SOP every step
OVERWRITE_ATTRIBS = "overwriteattributes"  # ... and its list of them
OVERWRITE_DEFAULT = "active animated deforming"  # the list's default
USE_COLLISIONS = "usecollisions"
VISUALIZE = "visualizegeometries"  # Visualization: the header toggle of Visualize Geometries, every guide's master switch (on)
SHOW_GROUND = "showground"  # Visualization: the ground in the guide (on)
SHADE_GROUND = "shadeground"  # Visualization: a ground plane drawn as 0 wireframe, 1 shaded
GROUND_OBJECT = "groundplane"  # the DOP object name of the ground, plane or height field
USE_GROUND = "useground"  # the Ground Type menu: 0 None, 1 Ground Plane, 2 Height Field
WORLD_SCALE_TOGGLE = "usescale"  # World Scale, as RBD Bullet Solver (see b3d_solver_network.py)
WORLD_SCALE = "scale"
COLLIDER_FOLDERS = ["Shape", "Material", "CCD & Contacts"]  # the collision object's Box3D Data folders on the Collision tab
# Of the collider's CCD & Contacts folder only Contact Recycling acts on a Static / Kinematic body
# (shown as Collider Contacts); the rest, and the pieces' Allow Sleeping, are left out: on the SOP the
# solver's Allow Sleeping is the one switch for every body. Their inner values stay at the defaults.
COLLIDER_DROPPED = {"ccdthreshold", "fastobject", "enablesleep", "startasleep", "sleepthreshold", "addimpact"}
COLLIDER_RELABEL = {"CCD & Contacts": "Contacts"}
PIECES_DROPPED = {"enablesleep"}

# Collapsible folders inside the tabs, as RBD Bullet Solver's, every one open but Impact Data.
# (tab, [(folder label, members by parameter name or label)]); a parameter in none stays loose.
TAB_GROUPS = {
    # Solver has no folders (World Scale, Time Scale, Substeps ... loose), and Gravity is a folder in Forces with Add Gravity as its header checkbox.
    "Advanced": [("Stability", ["enablewarmstarting", "enablecontinuous", "maxlinearspeed",
                                "isotropicinertia"]),
                 # Cache Simulation / Cache Memory (MB) join this folder first (make_solver_sop.py).
                 ("Performance", ["autothreads", "threads"])],
}
# The Pieces tab is the pieces' Box3D Data folders (Shape, Body, Material, CCD & Contacts, Sleep).
# The solver's settings the SOP regroups by name, wherever the DOP has them: (folder label, names, open) on the
# Solver tab after Cache Simulation.
SOLVER_FOLDERS = [("Contacts", ("contactstiffness", "contactdamping", "contactpushspeed", "contactrecycledistance"), True),
                  ("Sleeping", ("enablesleep",), False)]
# Aging goes to the Output tab's Attributes folder, as RBD Bullet Solver's Age Attribute.
# Constraints: Enable Attribute Update from SOPs with its attributes and group, and Overwrite with SOP
# (b3d_solver_updates.py).
ENABLE_CONSTRAINT_UPDATES, CONSTRAINT_ATTRIBUTES = "enable_constraintupdates", "constraint_attributes"
CONSTRAINT_ATTRIB_GROUP, CONSTRAINT_RELOAD = "constraint_attribgroup", "constraint_reloadfromsops"
ENABLE_AGING = "enableaging"
AGING_PARMS = (ENABLE_AGING, "reapdead", "reapatframeend")
# Add Active Age Attribute and Just Activated (with its Type), after them (b3d_solver_activeage.py).
ADD_ACTIVE_AGE, JUST_ACTIVATED, JUST_ACTIVATED_TYPE = "addactiveage", "justactivatedgroup", "justactivatedtype"

# The DOP subnet for forces, the asset's dive target, and its outputs in the order the Multiple
# Solver runs them around the Box3D Solver.
FORCES = "FORCES"
FORCES_OUTPUTS = ["PRESOLVE", "POSTSOLVE"]
DIVE_TARGET = f"dopnet/{FORCES}"
# Sticky notes beside PRESOLVE (left) and POSTSOLVE (right).
FORCES_NOTES = [
    "Wire POP Forces into PRESOLVE to apply per-point forces.\n\n"
    "With World Scale, the force and torque they set are scaled for you: enter real values.",
    "Wire post-solve operations into POSTSOLVE.\n\n"
    "If scaling the sim, remember to apply scale accordingly.",
]

INPUT_LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry", "Collision Geometry"]
OUTPUT_LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry", "Simulation Points"]

# Collision shapes: the solver's Export Collision Shapes (moved to the Output tab as Show Collision
# Shapes) writes each object's shapes as the SHAPES_DATA geometry; the asset tints them (SHAPES_COLOR).
SHAPES_TOGGLE = "exportshapes"
SHAPES_COLOR = "shapecolor"
SHAPES_DATA = "Box3DShapes"
SHOW_COLLISION = "showcollision"  # Visualization: the external collision geometry in the guide (on)
SHOW_COLLISION_SHAPES = "showcollisionshapes"  # Visualization: the collider's collision shapes (off)
SHOW_CONSTRAINTS = "showconstraints"  # Visualization: the constraint lines in the guide (off)
FALSE_COLOR_MODE = "vis_mode"  # their false colour: 0 None, 1 Angle, 2 Distance, 3 Force, 4 Impact, 5 Torque, 6 Custom
FALSE_COLOR_ATTRIB = "customattrib"  # the attribute False Color Mode Custom colours by
GUIDE_RADIUS = "showconstraintguidesradius"  # the constraint lines' radius in the guide, times each type's Guide Scale
SHOW_IMPACTS = "showimpacts"  # Visualization: the impact points in the guide (off)
ADD_GRAVITY = "addgravity"  # Forces: Gravity times 0 when off, RBD Bullet Solver's toggle
COLLISION_COLOR = "collisioncolor"  # Visualization: their colour, RBD Bullet Solver's blue
GUIDE_GEO = "GUIDE_GEO"  # the node the asset's Guide Geometry property names (as RBD Bullet Solver)

# The DOP Imports, set like RBD Bullet Solver's. Simulation points: Create Points to Represent
# Objects, Transfer Attributes from the asset's parameter, every group. Constraints: Fetch Geometry
# of ConstraintGeometry, nothing transformed, packed or tagged.
TRANSFER_ATTRIBUTES = "transferattributes"
# Transfer to Geometry / Proxy Geometry: drive each Transform Pieces' Attributes to Copy toggle, its
# attribute list the asset's Transfer Attributes.
TRANSFER_TO = {"transform_geometry": ("transfertogeo", "Transfer to Geometry"),
               "transform_proxy": ("transfertoproxy", "Transfer to Proxy Geometry")}
TRANSFER_ATTRIBUTES_DEFAULT = "age w"  # RBD Bullet Solver's default
ATTRIBS_TO_COPY = "attribstocopy"  # Transform Pieces' Attributes to Copy list
# Simulation points: Fetch Geometry of the pieces, not Create Points to Represent Objects (that makes
# one point per piece only for SideFX's RBD Packed Object; for ours it makes one point per DOP
# object). The pieces are in SOP space, so nothing is transformed.
POINTS_TOGGLES = {"pack": False,  # Pack Geometry Before Merging
                  "adddopobjectpath": False,
                  "doposxform": False,  # Transform Geometry With Position Data
                  "dogeoxform": False,  # Transform Geometry With Geometry Data
                  "keepworldspacepos": False}  # Preserve World Space Positions
# Point attributes the simulation points keep besides P and the asset's Transfer Attributes.
POINTS_KEEP = "name orient pivot restxform v w id"
CONSTRAINTS_TOGGLES = dict(POINTS_TOGGLES)
# The Output tab, always last, as RBD Bullet Solver's: Impact Data (the pieces' Add Impact Data as the
# folder's header toggle, the solver's Impact Min Speed, Fetch Impacts; closed), then Transfer Attributes.
IMPACT_TOGGLE = "addimpact"  # the pieces' Box3D Data toggle (on the Packed Object's Object tab)
INITIAL_TYPE = "initialstate"  # the Packed Object's Initial Object Type menu
START_ASLEEP = "startasleep"  # the pieces' Box3D Data toggle (on the Packed Object's Initial State tab)
IMPACT_SPEED = "impactminspeed"  # the solver's, moved from Advanced
IMPACTS = "IMPACTS"  # the null holding the imported impact records, what Fetch Impacts merges
# Fetch Impacts: an Object Merge of IMPACTS beside the asset (reused if there), selected.
FETCH_IMPACTS = (
    "n = kwargs['node']; name = n.name() + '_impacts'; "
    "m = n.parent().node(name) or n.parent().createNode('object_merge', name); "
    f"m.parm('objpath1').set(m.relativePathTo(n.node('{IMPACTS}'))); "
    "m.setPosition(n.position() + hou.Vector2(3, -1)); m.setSelected(True, clear_all_selected=True)")
GEO, CONSTRAINTS, PROXY, COLLISION = range(4)  # the asset's inputs
# The inner DOP network's inputs, read inside by a Geometry Source menu (CONTEXT_GEOMETRY[i], as RBD Bullet Solver's
# First / Third Context Geometry): not the asset's numbers.
DOP_PIECES, DOP_CONSTRAINTS, DOP_COLLISION = range(3)
CONTEXT_GEOMETRY = ("first", "second", "third", "fourth")
# Collide As Static Object: the Collision input unpacked (a null the Static Object reads by path), also the
# DOP network's fourth input so the Switches can count its primitives (a height field has no points).
DOP_STATIC = 3
COLLISION_SOURCE = "collisionsource"  # menu: 0 Packed Pieces, 1 Static Object
STATIC_NULL = "COLLISION_STATIC"  # the SOP the collision Static Object reads
STATIC_DATA = "collision_static_data"  # the Box3D Data DOP after the collision Static Object
# Collider settings that stay on when the collider is a Static Object (Shape and Contacts are for packed pieces).
STATIC_COLLIDER_FOLDERS = ("Shape", "Contacts")

# Every constraint relationship the solver supports, wired into the constraint network so any
# constraint_name in the network finds one. (node type candidates, node name). Data Names stay at
# their defaults: Weld, Spherical ... and SideFX's Glue, Hard, ConRelSpring, Soft, ConeTwist.
RELATIONSHIPS = [
    (["box3d_weldconrel"], "weld"),
    (["box3d_sphericalconrel"], "spherical"),
    (["box3d_revoluteconrel"], "revolute"),
    (["box3d_prismaticconrel"], "prismatic"),
    (["box3d_distanceconrel"], "distance"),
    (["box3d_motorconrel"], "motor"),
    (["box3d_parallelconrel"], "parallel"),
    (["box3d_wheelconrel"], "wheel"),
    (["box3d_filterconrel"], "filter"),
    (["glueconrel"], "glue"),
    (["hardconrel"], "hard"),
    (["springconrel"], "spring"),
    (["bulletsoftconrel"], "soft"),
    (["conetwistconrel"], "conetwist"),
]

# Parameters promoted from the inner SOP-level DOP network (Start Frame above the tabs, Cache Simulation
# in the Solver tab after the solver's own settings: its label hidden, joined to Cache Memory (MB), which
# it enables, as RBD Bullet Solver's).
START_FRAME, CACHE_ENABLED = "startframe", "cacheenabled"
DOPNET_PARMS = [START_FRAME, CACHE_ENABLED]
CACHE_MEMORY = "cachemaxsize"
# Global Substeps: the DOP network's own Substeps (`substep`).
GLOBAL_SUBSTEPS = "globalsubsteps"
# DOP Import's Import Style item Fetch Geometry from DOP Network.
FETCH_GEOMETRY = "fetch"
# Box3D Data's menu tokens are the item numbers (Box3DParms::menu): Body Type and Default Shape.
BODY_STATIC, BODY_KINEMATIC = "0", "1"
SHAPE_CONVEX = "2"

# Solver folders that are not promoted (the guide's toggle and colour are on the Visualization tab).
SKIP_SOLVER_FOLDERS = {"guide options"}

warn = b3d_hda.warner("make_solver_sop")
create = b3d_hda.create


def row_of(node, name):
    """Clones of the parameters on the row of parameter `name`, in order: it and the ones joined to it (a
    checkbox and its value). Empty if there is none."""
    entries = [pt for pt in node.parmTemplateGroup().entriesWithoutFolders() if not pt.name().startswith("parmop")]
    for i, pt in enumerate(entries):
        if pt.name() != name:
            continue
        lo, hi = i, i
        while lo > 0 and entries[lo - 1].joinsWithNext():
            lo -= 1
        while hi < len(entries) - 1 and entries[hi].joinsWithNext():
            hi += 1
        return [e.clone() for e in entries[lo:hi + 1]]
    return []


def set_named(node, name, value):
    """Sets `node`'s parameter (tuple) `name`; warns when it has none."""
    pt = node.parmTuple(name)
    if pt is None:
        warn(f"{node.path()} has no parameter {name}; its parameters: {labels(node)}")
        return False
    pt.set(value if isinstance(value, (tuple, list)) else (value,))
    return True


def set_menu(node, name, token):
    """Sets `node`'s menu `name` to the item `token`; warns when either is missing."""
    p = node.parm(name)
    if p is None:
        warn(f"{node.path()} has no menu {name}; its parameters: {labels(node)}")
        return False
    return b3d_hda.set_token(p, token, warn)


def solver_folders(entries):
    """The solver's own tabs (Solver, Collision ...): found anywhere under `entries` (Houdini puts
    them inside its Data Options tab). A folder holding tabs is a container and is searched; one
    holding parameters, or only simple folders (Misc's Sleeping and Aging), is a tab."""
    out = []
    for pt in entries:
        if pt.type() != hou.parmTemplateType.Folder or pt.label().lower() in SKIP_SOLVER_FOLDERS:
            continue
        children = pt.parmTemplates()
        sub_tabs = [c for c in children if c.type() == hou.parmTemplateType.Folder
                    and c.folderType() != hou.folderType.Simple]
        if sub_tabs:
            out.extend(solver_folders(children))
        elif value_templates(children):
            out.append(pt)
    return out


def as_collapsible(folder):
    """A clone of the folder template as an open collapsible folder (a folder that is already collapsible
    keeps its own open / closed state)."""
    folder = folder.clone()
    if folder.folderType() == hou.folderType.Collapsible:
        return folder
    folder.setFolderType(hou.folderType.Collapsible)
    tags = dict(folder.tags())
    tags["group_type"] = "collapsible"
    tags["group_default"] = "1"
    folder.setTags(tags)
    return folder


def group_tab(label, contents, groups=None):
    """A tab's contents in open collapsible folders (`groups`, else TAB_GROUPS); the folders it already
    has become open collapsible folders too. Parameters in no group stay loose, first."""
    groups = TAB_GROUPS.get(label, []) if groups is None else groups
    member = {}
    for i, (_, names) in enumerate(groups):
        for n in names:
            member[n] = i
    filled = [[] for _ in groups]
    loose, folders = [], []
    for pt in contents:
        if pt.type() == hou.parmTemplateType.Folder:
            folders.append(as_collapsible(pt))
            continue
        i = member.get(pt.name(), member.get(pt.label()))
        (filled[i] if i is not None else loose).append(pt)
    out = list(loose)
    for i, ((glabel, _), pts) in enumerate(zip(groups, filled)):
        if pts:
            out.append(collapsible(f"b3dsop_{label.lower()}_group{i}", glabel, pts))
    return out + folders


def flatten_folders(entries):
    """`entries` with every folder replaced by its contents, in order."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            out += flatten_folders(pt.parmTemplates())
        else:
            out.append(pt)
    return out


def drop_templates(entries, names):
    """Clones of `entries` without the parameters named in `names`, recursing into folders; folders
    left empty are dropped."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            pt.setParmTemplates(drop_templates(pt.parmTemplates(), names))
            if not pt.parmTemplates():
                continue
        elif pt.name() in names:
            continue
        out.append(pt)
    return out


def rename_templates(entries, renames):
    """Clones of `entries` with parameters renamed by `renames` {old: new}, recursing into folders,
    disable / hide conditions rewritten to the new names."""
    out = []
    for pt in entries:
        pt = pt.clone()
        if pt.type() == hou.parmTemplateType.Folder:
            pt.setParmTemplates(rename_templates(pt.parmTemplates(), renames))
        elif pt.name() in renames:
            pt.setName(renames[pt.name()])
        for kind, cond in pt.conditionals().items():
            new = cond
            for old, repl in renames.items():
                new = re.sub(rf"\b{re.escape(old)}\b", repl, new)
            if new != cond:
                pt.setConditional(kind, new)
        out.append(pt)
    return out


def switch_on_spare_input(switch, source):
    """`switch` picks its second input while `source` has points: `source` is its spare input
    (spare_input0, read as -1 in expressions), the selection `npoints(-1) > 0`."""
    template = hou.StringParmTemplate(
        "spare_input0", "Spare Input 0", 1, string_type=hou.stringParmType.NodeReference,
        help="Geometry the expressions on this node read as input -1.",
        tags={"opfilter": "!!SOP!!", "oprelative": "."})
    switch.addSpareParmTuple(template)
    switch.parm("spare_input0").set(switch.relativePathTo(source))
    p = switch.parm("input")
    if p is None:
        warn(f"{switch.path()} has no Select Input parameter")
        return
    p.setExpression("npoints(-1) > 0", hou.exprLanguage.Hscript)


def switch_on_context_geo(switch, index):
    """A DOP Switch inside the DOP network picking its second input while the network's input
    `index` has points (as RBD Bullet Solver: npoints(opinputpath("..", i)) > 0)."""
    p = switch.parm("input")
    if p is None:
        warn(f"{switch.path()} has no Select Input parameter; its parameters: {labels(switch)}")
        return
    p.setExpression(f'npoints(opinputpath("..", {index})) > 0', hou.exprLanguage.Hscript)


def copy_attributes(xform, toggle_parm):
    """Transform Pieces' Attributes to Copy, as RBD Bullet Solver's: always on, the asset's Transfer
    Attributes while its `toggle_parm` is on, else just w. The toggle is the one named in the list's
    disable condition."""
    attribs = xform.parmTemplateGroup().find(ATTRIBS_TO_COPY)
    if attribs is None:
        warn(f"{xform.path()} has no {ATTRIBS_TO_COPY}; its parameters: {labels(xform)}")
        return
    xform.parm(ATTRIBS_TO_COPY).set(f'`ifs(ch("../{toggle_parm}"), chs("../{TRANSFER_ATTRIBUTES}"), "w")`')
    toggle = None
    for cond in attribs.conditionals().values():
        m = re.search(r"(\w+)\s*==\s*0", cond)
        if m and xform.parm(m.group(1)) is not None:
            toggle = m.group(1)
            break
    if toggle is None:
        warn(f"{xform.path()}: no toggle for Attributes to Copy found; it copies always")
        return
    xform.parm(toggle).set(1)


def set_toggles(node, toggles):
    """Each toggle named by a key of `toggles` to its value."""
    for name, value in toggles.items():
        set_named(node, name, int(value))
