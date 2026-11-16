"""Box3D Solver SOP: constants and the helpers shared by the builders (parameter templates, tabs, folders,
switches, node-parameter setters)."""

import re

import hou

import b3d_hda
from b3d_hda import collapsible, labels, NO_VALUE, value_templates


TYPE_NAME = "box3d_solver"  # a SOP; the DOP of the same name lives in another node table
LABEL = "Box3D Solver"
ICON = "DOP_box3d_solver"  # installed by the build for the DOP (config/Icons)
PACKED_OBJECT = "box3d_packedobject"

# Colours of the subnet inputs, IN_* nulls and OUT_* nodes by index: constraints pink.
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}
COLLISION_TYPE = "collisiontype"  # menu: 0 Static, 1 Animated, 2 Deforming (RBD Bullet Solver's Collision Type)
OVERWRITE_FROM_SOP = "overwritefromsop"  # the Box3D Packed Object's toggle: re-import the SOP every step
USE_COLLISIONS = "usecollisions"
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
    # Solver has no folders (World Scale, Time Scale, Substeps ... loose), and Gravity is loose, first in Forces.
    "Forces": [("Force DOPs", ["useforces", "forceswake"]),
               ("Bounce / Friction", ["restitutionpropagation", "bouncethreshold", "restitutioniterations",
                                      "frictionmixing", "restitutionmixing"])],
    "Advanced": [("Stability", ["enablewarmstarting", "enablecontinuous", "maxlinearspeed",
                                "isotropicinertia"]),
                 # Cache Simulation / Cache Memory (MB) join this folder first (make_solver_sop.py).
                 ("Performance", ["autothreads", "threads"])],
}
# The Pieces tab is the pieces' Box3D Data folders (Shape, Body, Material, CCD & Contacts, Sleep).
# The solver's settings the SOP regroups by name, wherever the DOP has them: (folder label, names, open) on the
# Solver tab after Cache Simulation, and the bounce / friction ones loose in Forces (TAB_GROUPS folders them).
SOLVER_FOLDERS = [("Contacts", ("contactstiffness", "contactdamping", "contactpushspeed", "contactrecycledistance"), True),
                  ("Sleeping", ("enablesleep",), False),
                  ("Aging", ("enableaging", "reapdead", "reapatframeend"), False)]
BOUNCE_PARMS = ("restitutionpropagation", "bouncethreshold", "restitutioniterations", "frictionmixing",
                "restitutionmixing")

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
# Simulation points: Fetch Geometry of the pieces, not Create Points to Represent Objects (that makes
# one point per piece only for SideFX's RBD Packed Object; for ours it makes one point per DOP
# object). The pieces are in SOP space, so nothing is transformed.
POINTS_TOGGLES = {"Pack Geometry Before Merging": False,
                  "Add DOP Object Path Attribute": False,
                  "Transform Geometry With Position Data": False,
                  "Transform Geometry With Geometry Data": False,
                  "Preserve World Space Positions": False}
# Point attributes the simulation points keep besides P and the asset's Transfer Attributes.
POINTS_KEEP = "name orient pivot restxform v w id"
CONSTRAINTS_TOGGLES = {"Pack Geometry Before Merging": False,
                       "Add DOP Object Path Attribute": False,
                       "Transform Geometry With Position Data": False,
                       "Transform Geometry With Geometry Data": False,
                       "Preserve World Space Positions": False}
# The Output tab, always last, as RBD Bullet Solver's: Impact Data (the pieces' Add Impact Data as the
# folder's header toggle, the solver's Impact Min Speed, Fetch Impacts; closed), then Transfer Attributes.
IMPACT_TOGGLE = "addimpact"  # the pieces' Box3D Data toggle (on the Packed Object's Object tab)
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
# The inner DOP network's inputs (read inside with dopcontextgeo(".", i)): not the asset's numbers.
DOP_PIECES, DOP_CONSTRAINTS, DOP_COLLISION = range(3)

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

# Parameters promoted from the inner SOP-level DOP network, by label (Start Frame above the tabs,
# Cache Simulation in the Solver tab after the solver's own settings: its label hidden, joined to Cache
# Memory (MB), which it enables, as RBD Bullet Solver's).
DOPNET_PARMS = ["Start Frame", "Cache Simulation"]
CACHE_MEMORY = "Cache Memory (MB)"

# Solver folders that are not promoted (the guide's toggle and colour are on the Visualization tab).
SKIP_SOLVER_FOLDERS = {"guide options"}

warn = b3d_hda.warner("make_solver_sop")
create = b3d_hda.create


def template_by_label(node, label):
    for pt in node.parmTemplateGroup().entriesWithoutFolders():
        if pt.name().startswith("parmop") or pt.type() in NO_VALUE:
            continue
        if pt.label().lower() == label.lower():
            return pt
    return None


def row_by_label(node, label):
    """Clones of the parameters on the row of the one labelled `label`, in order: it and the ones joined to
    it (a checkbox and its value). Empty if there is none."""
    entries = [pt for pt in node.parmTemplateGroup().entriesWithoutFolders() if not pt.name().startswith("parmop")]
    for i, pt in enumerate(entries):
        if pt.type() in NO_VALUE or pt.label().lower() != label.lower():
            continue
        lo, hi = i, i
        while lo > 0 and entries[lo - 1].joinsWithNext():
            lo -= 1
        while hi < len(entries) - 1 and entries[hi].joinsWithNext():
            hi += 1
        return [e.clone() for e in entries[lo:hi + 1]]
    return []


def parm_by_label(node, label):
    pt = template_by_label(node, label)
    return node.parmTuple(pt.name()) if pt is not None else None


def set_by_label(node, label, value):
    pt = parm_by_label(node, label)
    if pt is None:
        warn(f"{node.path()} has no parameter labelled {label!r}; its parameters: {labels(node)}")
        return False
    pt.set(value if isinstance(value, (tuple, list)) else (value,))
    return True


def set_menu_item(node, parm_label, item_label):
    """Sets the menu labelled `parm_label` to the item whose label contains `item_label`."""
    pt = parm_by_label(node, parm_label)
    if pt is None:
        warn(f"{node.path()} has no menu labelled {parm_label!r}; its parameters: {labels(node)}")
        return False
    p = pt[0]
    items, item_labels = list(p.menuItems()), list(p.menuLabels())
    for i, lab in enumerate(item_labels):
        if item_label.lower() in lab.lower():
            try:
                p.set(i)
            except (TypeError, hou.OperationFailed):
                p.set(items[i])
            return True
    warn(f"{p.path()}: no item like {item_label!r} in {item_labels}")
    return False


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


def set_import_style(node, item):
    """Import Style to the item whose label starts with `item` ("fetch geometry": Fetch Geometry
    from DOP Network, not the Packed / Unpacked variants; "create points": Create Points to
    Represent Objects)."""
    p = node.parm("importstyle")
    if p is None:
        warn(f"{node.path()} has no Import Style; its parameters: {labels(node)}")
        return
    item_labels = list(p.menuLabels())
    for i, lab in enumerate(item_labels):
        if lab.lower().startswith(item):
            p.set(i)
            return
    warn(f"{p.path()}: no {item!r} item in {item_labels}")


def set_by_labels(node, candidates, value):
    """The parameter labelled like the first of `candidates` that exists to `value`."""
    for label in candidates:
        if template_by_label(node, label) is not None:
            return set_by_label(node, label, value)
    warn(f"{node.path()} has none of the parameters {candidates}; its parameters: {labels(node)}")
    return False


def copy_attributes(xform, toggle_parm):
    """Transform Pieces' Attributes to Copy, as RBD Bullet Solver's: always on, the asset's Transfer
    Attributes while its `toggle_parm` is on, else just w. The toggle is the one named in the list's
    disable condition (or a toggle with the same label)."""
    group = xform.parmTemplateGroup()
    found = [pt for pt in group.entriesWithoutFolders() if pt.label().lower() == "attributes to copy"]
    attribs = next((pt for pt in found if pt.type() == hou.parmTemplateType.String), None)
    if attribs is None:
        warn(f"{xform.path()} has no Attributes to Copy list; its parameters: {labels(xform)}")
        return
    xform.parm(attribs.name()).set(f'`ifs(ch("../{toggle_parm}"), chs("../{TRANSFER_ATTRIBUTES}"), "w")`')
    toggle = next((pt.name() for pt in found if pt.type() == hou.parmTemplateType.Toggle), None)
    if toggle is None:
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
    """Each toggle labelled like a key of `toggles` to its value."""
    for label, value in toggles.items():
        set_by_label(node, label, int(value))
