"""Builds the Box3D Solver SOP digital asset (a SOP, like RBD Bullet Solver) with hython.

    hython tools/make_solver_sop.py <output .hda> [<box3d_packedobject .hda>]

The build runs this after make_packed_object.py (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_solver_sop.hda. The plugin and the Box3D Packed Object asset must exist: the
asset contains a Box3D Solver DOP and two Box3D Packed Objects, and their parameters are copied
from them. Pass the Box3D Packed Object .hda to install it first (the build does).

Inputs:

    0  Geometry      pieces, packed or not. Simulated, unless a proxy is connected
    1  Constraints   constraint network (two-point lines); its subnet input is coloured pink
    2  Proxy         pieces simulated in place of Geometry; Geometry follows them
    3  Collision     colliders, following their animation (Animated Collision) or static

Unpacked inputs are packed by `name` (by connectivity when nothing is named), as RBD Bullet Solver.

Outputs: 0-2 as the inputs (moved by the simulation), 3 Simulation Points (one point per piece).

Inside:

    IN_* nulls -> sim_source (Proxy when it has points, else Geometry) -> prepare_pieces (pack mode)
      -> dopnet (SOP-level DOP network); IN_collision -> use_detach_faces (collision_detach: Bullet's
      Detach Faces and repack, when ticked) -> prepare_collision (pack mode) -> collision_rest (rest
      from a Time Shift to the start frame) -> dopnet input 2
      dopnet: pieces (Box3D Packed Object, SOP Path `dopcontextgeo(".", 0)`)
              -> constraintnetwork (Geometry Source: Second Context Geometry, Attach Internal Constraints
                 to Object, only when input 1 has points; every Box3D and SideFX constraint
                 relationship merged into it)
              collision (Box3D Packed Object, convex hulls, `dopcontextgeo(".", 2)`, only when Use External
                         Collisions is on and input 3 has points; Animated Collision: Kinematic and re-imported every step, else Static)
                         merged in after the constraint network
              -> multisolver (FORCES/PRESOLVE, box3d_solver, FORCES/POSTSOLVE) -> output
              FORCES: DOP subnet, the asset's dive target, with subnet outputs PRESOLVE (output 0) and
                      POSTSOLVE (output 1) for POP forces and other microsolvers run before / after
                      the Box3D Solver
    import_points (DOP Import, Fetch Geometry of the pieces) -> points_only -> sim_points (one point per
      piece, Bullet's P / orient / pivot / restxform) -> simulation points output, and Transform Pieces
      (no rest) onto names_geometry and names_proxy (names mode) -> geometry, proxy outputs
    import_constraints (DOP Import of the pieces' ConstraintGeometry) -> switchif_keepbroken (broken
      removed unless Export Broken Constraints) -> Transform Pieces onto the simulation points (world
      anchors reset) -> constraints output
    Geometry, constraints and proxy outputs: the inputs before the Start Frame (switchif_beforestart*)
    import_impacts (DOP Import Records of the pieces' Impacts) -> IMPACTS null, merged out by Fetch Impacts

Its parameters: Reset Simulation and Start Frame above the tabs; then the tabs Solver (the
solver's own, Cache Simulation, then its contact settings in a Contacts folder), Collision (Use
External Collisions, Animated Collision and the collider's Shape, Material and Sleep & Contacts settings,
collision_* names), Forces (Gravity first), Pieces (Box3D Packed Object's Box3D Data folders for the
simulated pieces), Misc (Sleeping), Advanced, and Output, always last (Impact Data with Fetch Impacts,
the solver's Aging folder, Transfer Attributes and Transfer to Geometry / Proxy Geometry). Each inner parameter is a channel reference to the outer one.
A Box3D Data parameter whose name the solver already uses gets a "piece_" prefix.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import re
import sys
import traceback

import hou

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b3d_prepare  # noqa: E402  (beside this script)

TYPE_NAME = "box3d_solver"  # a SOP; the DOP of the same name lives in another node table
LABEL = "Box3D Solver"
ICON = "DOP_box3d_solver"  # installed by the build for the DOP (config/Icons)
PACKED_OBJECT = "box3d_packedobject"

# Colours of the subnet inputs, IN_* nulls and OUT_* nodes by index: constraints pink.
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}
ANIMATED_COLLISION = "animatedcollision"
USE_COLLISIONS = "usecollisions"
COLLIDER_FOLDERS = ["Shape", "Material", "Sleep & Contacts"]  # the collision object's Box3D Data folders on the Collision tab
# Of the collider's Sleep & Contacts folder only Contact Recycling acts on a Static / Kinematic body
# (shown as Collider Contacts); the rest, and the pieces' Allow Sleeping, are left out: on the SOP the
# solver's Allow Sleeping is the one switch for every body. Their inner values stay at the defaults.
COLLIDER_DROPPED = {"ccdthreshold", "fastobject", "enablesleep", "startasleep", "sleepthreshold", "addimpact"}
COLLIDER_RELABEL = {"Sleep & Contacts": "Contacts"}
PIECES_DROPPED = {"enablesleep"}

# Collapsible folders inside the tabs, as RBD Bullet Solver's: the main groups open (its General,
# Simulation, Pieces, Sleeping, Collision Geometry), secondary ones closed (its Advanced folders,
# Attribute Transfer). Gravity is open here (closed on Bullet): it is the setting most often changed.
# (tab, [(folder label, open, members by parameter name or label)]); a parameter in none stays loose.
TAB_GROUPS = {
    "Solver": [("General", True, ["usescale", "scale", "timescale"]),  # WORLD_SCALE_TOGGLE, WORLD_SCALE
               ("Simulation", True, ["stepsperframe", "substeps", "solvefirstframe", "Cache Simulation"])],
    "Forces": [("Gravity", True, ["gravity"]),
               ("Force DOPs", True, ["useforces", "forceswake"]),
               ("Bounce", False, ["restitutionpropagation", "bouncethreshold", "restitutioniterations"]),
               ("Mixing", False, ["frictionmixing", "restitutionmixing"])],
    "Advanced": [("Box3D Solver", False, ["enablewarmstarting", "enablecontinuous", "maxlinearspeed",
                                           "isotropicinertia"]),
                 ("Threads", False, ["autothreads", "threads"])],
}
OPEN_FOLDERS = {"Contacts", "Sleeping", "Aging", "Constraints"}  # folders the solver or Output brings, open
PIECES_SEPARATE = "Sleep & Contacts"  # the Pieces tab: this one beside the Pieces folder, as Bullet's Sleeping
TAB_RENAMES = {"Setup": "Solver"}  # older plugins name the solver's first tab Setup
TAB_MOVES = {"gravity": "Forces"}  # solver parameters shown first in another tab (older plugins: Setup)

# The DOP subnet for forces, the asset's dive target, and its outputs in the order the Multiple
# Solver runs them around the Box3D Solver.
FORCES = "FORCES"
FORCES_OUTPUTS = ["PRESOLVE", "POSTSOLVE"]
DIVE_TARGET = f"dopnet/{FORCES}"
# Sticky notes beside PRESOLVE (left) and POSTSOLVE (right).
FORCES_NOTES = [
    "Wire POP Forces into PRESOLVE to apply per-point forces.\n\n"
    "If scaling the sim, remember to apply scale accordingly.",
    "Wire post-solve operations into POSTSOLVE.\n\n"
    "If scaling the sim, remember to apply scale accordingly.",
]

INPUT_LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry", "Collision Geometry"]
OUTPUT_LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry", "Simulation Points"]

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
# folder's header toggle, the solver's Impact Min Speed, Fetch Impacts), the solver's Aging folder
# (moved from Misc), then Transfer Attributes.
IMPACT_TOGGLE = "addimpact"  # the pieces' Box3D Data toggle
IMPACT_SPEED = "impactminspeed"  # the solver's, moved from Advanced
AGING_FOLDER = "Aging"  # the solver's Misc subfolder
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
# Cache Simulation in the Solver tab after the solver's own settings).
DOPNET_PARMS = ["Start Frame", "Cache Simulation"]

# Solver folders that are not promoted (guides: TODO in docs/agents/status.md).
SKIP_SOLVER_FOLDERS = {"guide options"}

# Templates without a value: nothing to reference.
NO_VALUE = {hou.parmTemplateType.Folder, hou.parmTemplateType.FolderSet,
            hou.parmTemplateType.Separator, hou.parmTemplateType.Label,
            hou.parmTemplateType.Button}

def warn(msg):
    print(f"make_solver_sop: WARNING: {msg}")


def create(parent, type_names, name):
    for t in [type_names] if isinstance(type_names, str) else type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed:
            continue
    raise RuntimeError(f"none of the node types {type_names} exist in {parent.path()}")


def value_templates(entries):
    """Every template with a value under `entries`, recursing into folders."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            out.extend(value_templates(pt.parmTemplates()))
        elif pt.type() not in NO_VALUE:
            out.append(pt)
    return out


def without_parmops(entries):
    """`entries` without the DOP Parameter Operation menus (parmop_<name>), recursing into folders."""
    out = []
    for pt in entries:
        if pt.name().startswith("parmop"):
            continue
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            pt.setParmTemplates(without_parmops(pt.parmTemplates()))
        out.append(pt)
    return out


def template_by_label(node, label):
    for pt in node.parmTemplateGroup().entriesWithoutFolders():
        if pt.name().startswith("parmop") or pt.type() in NO_VALUE:
            continue
        if pt.label().lower() == label.lower():
            return pt
    return None


def parm_by_label(node, label):
    pt = template_by_label(node, label)
    return node.parmTuple(pt.name()) if pt is not None else None


def labels(node):
    return ", ".join(sorted({f"{pt.label()} ({pt.name()})" for pt in node.parmTemplateGroup().entriesWithoutFolders()
                             if not pt.name().startswith("parmop") and pt.type() not in NO_VALUE}))


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


def set_operations(node, token):
    """Every Parameter Operation menu (parmop_*) of `node` to `token`: a channel reference does not
    switch a DOP data parameter from Use Default (see make_packed_object.py)."""
    for p in node.parms():
        if not p.name().startswith("parmop"):
            continue
        try:
            if p.parmTemplate().type() == hou.parmTemplateType.String:
                p.set(token)
            elif token in p.menuItems():
                p.set(list(p.menuItems()).index(token))
        except hou.Error as e:
            warn(f"{p.path()}: {e}")


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


def collapsible(name, label, contents, open_folder):
    """A collapsible folder, open by default when `open_folder` (Bullet's group_default tag)."""
    return hou.FolderParmTemplate(name, label, contents, folder_type=hou.folderType.Collapsible,
                                  tags={"group_default": "1"} if open_folder else {})


def as_collapsible(folder, open_folder):
    """A clone of the folder template as a collapsible folder."""
    folder = folder.clone()
    folder.setFolderType(hou.folderType.Collapsible)
    tags = dict(folder.tags())
    tags.pop("group_type", None)
    if open_folder:
        tags["group_default"] = "1"
    folder.setTags(tags)
    return folder


def group_tab(label, contents):
    """A tab's contents in collapsible folders (TAB_GROUPS); the folders it already has become
    collapsible too (OPEN_FOLDERS open, the rest closed). Parameters in no group stay loose, first."""
    groups = TAB_GROUPS.get(label, [])
    member = {}
    for i, (_, _, names) in enumerate(groups):
        for n in names:
            member[n] = i
    filled = [[] for _ in groups]
    loose, folders = [], []
    for pt in contents:
        if pt.type() == hou.parmTemplateType.Folder:
            folders.append(as_collapsible(pt, pt.label() in OPEN_FOLDERS))
            continue
        i = member.get(pt.name(), member.get(pt.label()))
        (filled[i] if i is not None else loose).append(pt)
    out = list(loose)
    for i, ((glabel, open_folder, _), pts) in enumerate(zip(groups, filled)):
        if pts:
            out.append(collapsible(f"b3dsop_{label.lower()}_group{i}", glabel, pts, open_folder))
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


# Input preparation (b3d_prepare.py, shared with Box3D Configure): pack mode for what is simulated,
# collision mode for the collision, names mode for the Geometry / Proxy outputs. The rest and the
# orient / pscale / scale bake happen in the Box3D Packed Objects, as in RBD Bullet Solver.
PACK_TRANSFER_ATTRIBS = "v w active animated deforming"  # onto assembled pieces' points


def prepare_input(sub, src, name, pack, collision=False):
    """b3d_prepare.prepare_input as this asset uses it, with RBD Bullet Solver's Assemble settings:
    Create Path Attribute on, Path Attribute op:<assemble>/pack when packing (Housekeeping,
    Collisions), v w transferred in names mode (Housecleaning)."""
    if pack:
        parms = {"createpath": 1, "path": b3d_prepare.PACK_PATH,
                 "transfer_attributes": PACK_TRANSFER_ATTRIBS}
    else:
        parms = {"createpath": 1, "transfer_attributes": "v w"}
    return b3d_prepare.prepare_input(sub, src, name, pack, warn, assemble_parms=parms, collision=collision)


# The collision's rest from the start frame (input 1: the prepared collision there), as RBD Bullet
# Solver's collision repack does with a Time Shift: restxform stays the starting transform while an
# animated collider is re-imported every step, and orient is the rotation since (row-normalised
# rotations, mirrors flipped, as the solver splits packed matrices). Matched by name, else by number.
COLLISION_REST_VEX = """\
int rp = findattribval(1, "point", "name", s@name);
if (rp < 0) rp = @ptnum;
int now[] = pointprims(0, @ptnum);
int was[] = pointprims(1, rp);
if (len(now) != 1 || len(was) != 1) return;
// Both read as 4x4 first: matrix3(primintrinsic(...)) picks primintrinsic()'s matrix3 overload,
// which reads nothing from this 4x4 intrinsic.
matrix rest = primintrinsic(1, "packedfulltransform", was[0]);
matrix current = primintrinsic(0, "packedfulltransform", now[0]);
matrix3 m[] = array(matrix3(rest), matrix3(current));
matrix3 r[];
foreach (matrix3 a; m) {
    vector x = normalize(set(a.xx, a.xy, a.xz));
    vector y = normalize(set(a.yx, a.yy, a.yz));
    vector z = normalize(set(a.zx, a.zy, a.zz));
    if (determinant(a) < 0) z = -z;
    push(r, set(x, y, z));
}
4@restxform = rest;
v@pivot = point(1, "P", rp);
p@orient = quaternion(transpose(r[0]) * r[1]);
"""

# Detach Faces (RBD Bullet Solver's collision_detachfaces, a hidden toggle there): colliders are rebuilt
# from unshared points, packed ones repacked (REPACK_*: each unpacked about the origin, its points
# split, packed again with an origin pivot, and given back its transform relative to the start frame).
DETACH_FACES = "collision_detachfaces"
REPACK_ID_VEX = "i@__packedid = @ptnum;"
REPACK_TO_ORIGIN_VEX = "@P = set(0);"
REPACK_PRIM_ID_VEX = 'i@__packedid = point(0, "__packedid", @primnum);'
# Input 0: the repacked pieces at the start frame; 1: the original packed pieces at the start frame;
# 2: the original packed pieces now. Bullet's own VEX.
REPACK_XFORM_VEX = """\
matrix3 refxform = primintrinsic(1, "transform", i@ptnum);
matrix3 animxform = primintrinsic(2, "transform", i@ptnum);
matrix3 xform = primintrinsic(0, "transform", i@ptnum);
xform *= invert(refxform) * animxform;
setprimintrinsic(0, "transform", @ptnum, xform);
@P = v@opinput2_P;
"""

# Solve on Creation Frame, as RBD Bullet Solver: solvefirstframe on the Solver tab drives both the
# pieces' and the collider's Packed Object.
SOLVE_FIRST = "solvefirstframe"
SOLVE_FIRST_LABEL = "Solve on Creation Frame"  # the Box3D Packed Object's label (Empty Object's)

# Export Broken Constraints (RBD Bullet Solver's constraint_keepbroken): off, broken constraints are
# removed from the constraints output.
KEEP_BROKEN = "constraint_keepbroken"

# Constraint points anchored to the world (no piece name) keep their imported position.
RESET_ANCHORS_VEX = "@P = point(1, \"P\", @ptnum);"

# Every primitive removed, its points kept: an Add SOP's Delete Geometry But Keep the Points (one
# bulk delete; removeprim per primitive is far slower on many pieces). The wrangle is the fallback
# when no such toggle is found.
POINTS_ONLY_LABELS = ["Delete Geometry But Keep the Points", "Delete Geometry, Keep Points"]
POINTS_ONLY_VEX = "removeprim(0, @primnum, 0);"

# World Scale (RBD Bullet Solver's usescale / Bullet World Scale): the inputs are scaled by it for the
# simulation and every output scaled back, so pieces far from about 1 unit simulate at a size Box3D
# is tuned for. Lengths and speeds among the solver's and Box3D Data's settings follow it
# (SCALED_SETTINGS), densities by its cube, so masses stay the same.
WORLD_SCALE_TOGGLE = "usescale"
WORLD_SCALE = "scale"
SCALE_FACTOR = f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}"), 1)'  # from a node inside the asset
SCALE_ACTIVE = f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}") != 1, 0)'  # switches: input 1 when scaling
# (inner node relative to the asset, parameter tuple, power of the scale it follows).
SCALED_SETTINGS = [("dopnet/box3d_solver", n, 1) for n in
                   ("gravity", "maxlinearspeed", "contactpushspeed", "bouncethreshold",
                    "contactrecycledistance", "impactminspeed")] + \
    [("dopnet/pieces", "density", -3)] + \
    [("dopnet/pieces", n, 1) for n in ("sleepthreshold", "tangentvelocity")] + \
    [("dopnet/collision", "tangentvelocity", 1)]
# (The collider's density and sleep threshold are not promoted: it is never dynamic.)
# The pieces after packing (Transform scaled P, v, w and the packed transforms): Bullet's
# scale_density_and_attribs (density / s^3, w back, as the Transform scaled it), plus the hand-set
# shape sizes and the sleep threshold, which Bullet leaves.
SCALE_ATTRIBS_VEX = """\
float s = chf("scale");
if (haspointattrib(0, "density")) setpointattrib(0, "density", @ptnum, 1 / pow(s, 3), "mult");
if (haspointattrib(0, "w")) setpointattrib(0, "w", @ptnum, 1 / s, "mult");
foreach (string n; {"bullet_primS", "bullet_primT", "bullet_radius", "bullet_length", "sleepthreshold"})
    if (haspointattrib(0, n)) setpointattrib(0, n, @ptnum, s, "mult");
"""
# Outputs scaled back (chf("scale") is 1 / World Scale): Bullet's scale_simpoints, scale_attribs and
# scale_impacts. Joint feedback: force and distance scale with length, torque with its square.
UNSCALE_POINTS_VEX = """\
float s = chf("scale");
v@P *= s;
if (haspointattrib(0, "pivot")) v@pivot *= s;
if (haspointattrib(0, "v")) v@v *= s;
"""
UNSCALE_CONSTRAINTS_VEX = """\
float s = chf("scale");
foreach (string n; {"force", "distance"})
    if (hasprimattrib(0, n)) setprimattrib(0, n, @primnum, s, "mult");
if (hasprimattrib(0, "torque")) setprimattrib(0, "torque", @primnum, s * s, "mult");
"""
UNSCALE_IMPACTS_VEX = """\
float s = chf("scale");
v@P *= s;
foreach (string n; {"position", "v", "impulse"})
    if (haspointattrib(0, n)) setpointattrib(0, n, @ptnum, s, "mult");
"""

# Pieces the solver deleted (dead, reaped) leave the Geometry / Proxy outputs: RBD Bullet Solver's
# remove_deleted, on the names-mode pieces (input 1: the simulation points; input 2, Geometry only:
# what was simulated, so pieces that never were, such as geometry without a proxy piece, stay).
# removeprim only runs for the deleted few.
REMOVE_DELETED_VEX = """\
string name = prim(0, "name", @primnum);
if (name == "")
    name = point(0, "name", @ptnum);
int del = findattribval(1, "point", "name", name) < 0;
if (npoints(2))
{
    int pt = findattribval(2, "prim", "name", name);
    if (pt < 0)
        pt = findattribval(2, "point", "name", name);
    del *= pt >= 0;
}
if (del) removeprim(0, @primnum, 1);
"""


def start_frame_expr(dopnet):
    """HScript for the asset's Start Frame from a node directly inside the asset (the promoted
    parameter has the DOP network's name), or None."""
    start = template_by_label(dopnet, "Start Frame")
    return f'ch("../{start.name()}")' if start is not None else None


def before_start(sub, dopnet, name, result, original):
    """RBD Bullet Solver's switchif_beforestart: `original` before the Start Frame, else `result`."""
    sw = create(sub, "switchif", name)
    sw.setInput(0, result)
    sw.setInput(1, original)
    start = start_frame_expr(dopnet)
    if start is None or sw.parm("expr1") is None:
        warn(f"{sw.path()}: no Start Frame or no Enable expression; the simulated result is shown always")
        if sw.parm("expr1") is not None:
            sw.parm("expr1").set(0)
    else:
        sw.parm("expr1").setExpression(f"$FF < {start}", hou.exprLanguage.Hscript)
    return sw


def scale_xform(sub, name, src, inverse=False):
    """A Transform SOP scaling `src` uniformly by the World Scale (or its inverse)."""
    xf = create(sub, "xform", name)
    xf.setInput(0, src)
    p = xf.parm("scale")
    if p is None:
        warn(f"{xf.path()} has no Uniform Scale (scale); World Scale ignored there")
    else:
        p.setExpression(f"1 / {SCALE_FACTOR}" if inverse else SCALE_FACTOR, hou.exprLanguage.Hscript)
    return xf


def scale_wrangle(sub, name, src, snippet, inverse, run_over="point"):
    """A wrangle with a `scale` spare parameter (the World Scale, or its inverse) running `snippet`,
    and the Switch that uses it only while scaling, as Bullet's switch_scale*. Returns the switch."""
    wr = create(sub, "attribwrangle", name)
    wr.setInput(0, src)
    wr.parm("class").set(run_over)
    wr.addSpareParmTuple(hou.FloatParmTemplate(WORLD_SCALE, "Scale", 1, default_value=(1.0,)))
    wr.parm(WORLD_SCALE).setExpression(f"1 / {SCALE_FACTOR}" if inverse else SCALE_FACTOR,
                                       hou.exprLanguage.Hscript)
    wr.parm("snippet").set(snippet)
    sw = create(sub, "switch", f"switch_{name}")
    sw.setInput(0, src)
    sw.setInput(1, wr)
    sw.parm("input").setExpression(SCALE_ACTIVE, hou.exprLanguage.Hscript)
    return sw


def scale_settings(asset):
    """SCALED_SETTINGS: each inner channel reference times the World Scale to its power."""
    for inner_path, name, power in SCALED_SETTINGS:
        inner = asset.node(inner_path)
        tup = inner.parmTuple(name) if inner is not None else None
        if tup is None:
            warn(f"World Scale: no {inner_path}/{name}; it is not scaled")
            continue
        rel = inner.relativePathTo(asset)
        factor = f'if(ch("{rel}/{WORLD_SCALE_TOGGLE}"), ch("{rel}/{WORLD_SCALE}"), 1)'
        for p in tup:
            try:
                expr = p.expression()
            except hou.OperationFailed:
                expr = None
            if not expr:
                warn(f"World Scale: {p.path()} is not a channel reference; it is not scaled")
                continue
            if power == 1:
                p.setExpression(f"({expr}) * {factor}", hou.exprLanguage.Hscript)
            else:
                p.setExpression(f"({expr}) * pow({factor}, {power})", hou.exprLanguage.Hscript)


def detach_faces(sub, dopnet, collision):
    """RBD Bullet Solver's Detach Faces branch for the collision input: assembled by name (or
    connectivity), unpacked colliders' points split and packed again by name, packed ones repacked
    (REPACK_*). Switched in by the asset's collision_detachfaces. Returns the switch."""
    net = create(sub, "subnet", "collision_detach")
    net.setInput(0, collision)
    for child in net.children():
        child.destroy()
    inp = net.indirectInputs()[0]

    def node(parent, type_name, name, inputs=(), parms=None, exprs=None):
        n = create(parent, type_name, name)
        for i, item in enumerate(inputs):
            n.setInput(i, item[0], item[1]) if isinstance(item, tuple) else n.setInput(i, item)
        b3d_prepare._set(n, warn, parms, exprs)
        return n

    conn = node(net, "connectivity", "connectivity", [inp], {"connecttype": "prim"})
    grp = node(net, "attribwrangle", "group_packed", [inp, conn],
               {"class": "primitive", "snippet": b3d_prepare.GROUP_PACKED_VEX})
    named = node(net, "assemble", "assemble_names", [grp],
                 {"outside_group": "static", "connect": 0, "createpath": 1},
                 {"newname": '1 - hasprimattrib(opinputpath(".", 0), "name")'})
    split = node(net, "split", "split_packed", [named],
                 {"group": b3d_prepare.PACKED_GROUP, "grouptype": "prims"})

    # Packed colliders: repacked, as Bullet's repack subnet.
    start = start_frame_expr(dopnet)
    start_inner = start.replace('ch("../', 'ch("../../../') if start else None  # repack/x -> asset
    rp = create(net, "subnet", "repack")
    rp.setInput(0, split, 0)
    for child in rp.children():
        child.destroy()
    rin = rp.indirectInputs()[0]
    ids = node(rp, "attribwrangle", "packed_id", [rin], {"snippet": REPACK_ID_VEX})
    origin = node(rp, "attribwrangle", "to_origin", [ids], {"snippet": REPACK_TO_ORIGIN_VEX})
    unpack = node(rp, "unpack", "unpack", [origin], {"transfer_attributes": "__packedid"})
    unique = node(rp, "splitpoints", "unique_points", [unpack])
    packed = node(rp, "assemble", "assemble", [unique],
                  {"outside_group": "static", "newname": 0, "connect": 0, "pack_geo": 1, "createpath": 1,
                   "path": b3d_prepare.PACK_PATH, "transfer_attributes": "__packedid", "pivot": 0})
    prim_id = node(rp, "attribwrangle", "prim_id", [packed],
                   {"class": "primitive", "snippet": REPACK_PRIM_ID_VEX})
    order = node(rp, "sort", "sort", [prim_id],
                 {"ptsort": "attribute", "pointattrib": "__packedid", "primsort": "attribute",
                  "primattrib": "__packedid"})
    at_start = [node(rp, "timeshift", nm, [src]) for nm, src in (("repacked_at_start", order),
                                                                 ("original_at_start", ids))]
    for ts in at_start:
        if start_inner:
            ts.parm("frame").setExpression(start_inner, hou.exprLanguage.Hscript)
    xform = node(rp, "attribwrangle", "restore_transform", [at_start[0], at_start[1], ids],
                 {"snippet": REPACK_XFORM_VEX})
    clean = node(rp, "attribdelete", "delete_temp", [xform], {"ptdel": "__*", "primdel": "__*"})
    rout = node(rp, "output", "output0", [clean])
    rout.setDisplayFlag(True)
    rp.layoutChildren()

    # Unpacked colliders: points split, packed again by the names assemble_names gave.
    unique2 = node(net, "splitpoints", "unique_points", [(split, 1)])
    repacked = node(net, "assemble", "assemble_pieces", [unique2],
                    {"outside_group": "static", "newname": 0, "connect": 0, "pack_geo": 1, "createpath": 1,
                     "path": b3d_prepare.PACK_PATH})
    merged = node(net, "merge", "merge", [rp, repacked])
    out = node(net, "output", "output0", [merged])
    out.setDisplayFlag(True)
    net.layoutChildren()

    sw = create(sub, "switch", "use_detach_faces")
    sw.setInput(0, collision)
    sw.setInput(1, net)
    sw.parm("input").setExpression(f'ch("../{DETACH_FACES}")', hou.exprLanguage.Hscript)
    return sw


def collision_rest(sub, dopnet, collision):
    """The prepared collision with its rest from the start frame (COLLISION_REST_VEX). Returns the
    node feeding the DOP network's collision input."""
    start = template_by_label(dopnet, "Start Frame")
    at_start = create(sub, "timeshift", "collision_at_start")
    at_start.setInput(0, collision)
    p = at_start.parm("frame")
    if start is None or p is None:
        warn("no Start Frame on the DOP network or no Frame on the Time Shift: the collision's rest is "
             "its current transform")
    else:  # the asset's promoted Start Frame (same name as the DOP network's)
        p.setExpression(f'ch("../{start.name()}")', hou.exprLanguage.Hscript)
    rest = create(sub, "attribwrangle", "collision_rest")
    rest.setInput(0, collision)
    rest.setInput(1, at_start)
    rest.parm("snippet").set(COLLISION_REST_VEX)
    return rest


def build_network(sub):
    """The asset's contents, inside the subnet `sub`. Returns the nodes whose parameters the
    asset promotes: (dopnet, solver, pieces)."""
    ins = sub.indirectInputs()
    for i, rgb in INPUT_COLORS.items():
        ins[i].setColor(hou.Color(rgb))
    nulls = []
    for i, name in enumerate(["IN_geometry", "IN_constraints", "IN_proxy", "IN_collision"]):
        n = create(sub, "null", name)
        n.setInput(0, ins[i])
        if i in INPUT_COLORS:
            n.setColor(hou.Color(INPUT_COLORS[i]))
        nulls.append(n)

    # World Scale on every input, as RBD Bullet Solver's transform1 / 2 / 4 / 5 (the before-start
    # outputs pass the unscaled inputs).
    scaled = [scale_xform(sub, f"world_scale_{n[3:]}", nulls[i]) for i, n in
              enumerate(["IN_geometry", "IN_constraints", "IN_proxy", "IN_collision"])]

    # What is simulated: the proxy when connected, else the geometry; packed and named (pack mode).
    sim_source = create(sub, "switch", "sim_source")
    sim_source.setInput(0, scaled[GEO])
    sim_source.setInput(1, scaled[PROXY])
    switch_on_spare_input(sim_source, nulls[PROXY])
    # The same choice unscaled, for the Proxy output before the Start Frame.
    sim_input = create(sub, "switch", "sim_source_unscaled")
    sim_input.setInput(0, nulls[GEO])
    sim_input.setInput(1, nulls[PROXY])
    switch_on_spare_input(sim_input, nulls[PROXY])
    sim_pieces = prepare_input(sub, sim_source, "prepare_pieces", pack=True)
    sim_scaled = scale_wrangle(sub, "scale_attributes", sim_pieces, SCALE_ATTRIBS_VEX, inverse=False)

    # The DOP network, its context geometry three streams (dopcontextgeo(".", i) inside).
    dopnet = create(sub, "dopnet", "dopnet")
    dopnet.setInput(DOP_PIECES, sim_scaled)
    dopnet.setInput(DOP_CONSTRAINTS, scaled[CONSTRAINTS])
    # Collision: Detach Faces (optional), packed and named, rest from the start frame.
    collision_pieces = prepare_input(sub, detach_faces(sub, dopnet, scaled[COLLISION]), "prepare_collision",
                                     pack=True, collision=True)
    dopnet.setInput(DOP_COLLISION, collision_rest(sub, dopnet, collision_pieces))
    for child in dopnet.children():
        child.destroy()
    solver = build_dops(dopnet)

    # Simulation points, the fourth output, as RBD Bullet Solver's: one point per piece with P, name,
    # orient (the rotation since rest), pivot (the rest position), restxform (the packed transform at
    # rest), v, w, id and the Transfer Attributes.
    import_points = create(sub, "dopimport", "import_points")
    import_points.parm("doppath").set("../dopnet")
    import_points.parm("objpattern").set("pieces")
    set_import_style(import_points, "fetch geometry")
    set_toggles(import_points, POINTS_TOGGLES)
    points_only = create(sub, "add", "points_only")
    points_only.setInput(0, import_points)
    if not any(template_by_label(points_only, label) for label in POINTS_ONLY_LABELS) or \
            not set_by_labels(points_only, POINTS_ONLY_LABELS, 1):
        warn(f"{points_only.path()}: no Delete Geometry But Keep the Points toggle; removing primitives in VEX")
        points_only.destroy()
        points_only = create(sub, "attribwrangle", "points_only")
        points_only.setInput(0, import_points)
        points_only.parm("class").set("primitive")
        points_only.parm("snippet").set(POINTS_ONLY_VEX)
    sim_points = create(sub, "attribdelete", "sim_points")
    sim_points.setInput(0, points_only)
    sim_points.parm("ptdel").set(f'{POINTS_KEEP} `chs("../{TRANSFER_ATTRIBUTES}")`')
    if not set_by_labels(sim_points, ["Delete Non Selected", "Delete Non-Selected"], 1):
        # Without the inverse toggle the list would delete exactly what must stay: keep everything.
        sim_points.parm("ptdel").set("")

    # Geometry and proxy follow the simulation points, as RBD Bullet Solver: Transform Pieces by name
    # with no rest input, each point's transform being its motion since rest (pivot, orient, P). Each
    # output keeps its input's packing (names mode).
    pieces_out = []
    for name, src in (("transform_geometry", scaled[GEO]), ("transform_proxy", sim_source)):
        named = prepare_input(sub, src, f"names_{name[len('transform_'):]}", pack=False)
        removed = create(sub, "attribwrangle", f"remove_deleted_{name[len('transform_'):]}")
        removed.setInput(0, named)
        removed.setInput(1, sim_points)
        if name == "transform_geometry":
            removed.setInput(2, sim_pieces)
        removed.parm("class").set("primitive")
        removed.parm("snippet").set(REMOVE_DELETED_VEX)
        xform = create(sub, "xformpieces", name)
        xform.setInput(0, removed)
        xform.setInput(1, sim_points)
        copy_attributes(xform, TRANSFER_TO[name][0])
        pieces_out.append(scale_xform(sub, f"unscale_{name[len('transform_'):]}", xform, inverse=True))
    geo_out, proxy_out = pieces_out

    # Constraints as the solver left them: the network is attached to the pieces object
    # (Attach Internal Constraints to Object) as its ConstraintGeometry data, imported as is.
    import_con = create(sub, "dopimport", "import_constraints")
    import_con.parm("doppath").set("../dopnet")
    import_con.parm("objpattern").set("pieces")
    set_import_style(import_con, "fetch geometry")
    set_by_label(import_con, "Geometry Data Path", "ConstraintGeometry")
    set_toggles(import_con, CONSTRAINTS_TOGGLES)
    # The solver does not move the network's points: Transform Pieces carries each point with the piece
    # its `name` names, and points anchored to the world (no name) keep their place, as RBD Bullet Solver.
    # Export Broken Constraints off: the broken group's primitives removed first (as RBD Bullet Solver).
    blast_broken = create(sub, "blast", "blast_broken")
    blast_broken.setInput(0, import_con)
    blast_broken.parm("group").set("broken")
    keep_broken = create(sub, "switchif", "switchif_keepbroken")
    keep_broken.setInput(0, blast_broken)
    keep_broken.setInput(1, import_con)
    keep_broken.parm("expr1").setExpression(f'ch("../{KEEP_BROKEN}")', hou.exprLanguage.Hscript)
    con_xform = create(sub, "xformpieces", "transform_constraints")
    con_xform.setInput(0, keep_broken)
    con_xform.setInput(1, sim_points)
    reset_anchors = create(sub, "attribwrangle", "reset_world_anchors")
    reset_anchors.setInput(0, con_xform)
    reset_anchors.setInput(1, keep_broken)
    reset_anchors.parm("group").set('@name=="-1" @name==""')
    reset_anchors.parm("grouptype").set("points")
    reset_anchors.parm("snippet").set(RESET_ANCHORS_VEX)
    con_attribs = scale_wrangle(sub, "unscale_constraint_attribs", reset_anchors, UNSCALE_CONSTRAINTS_VEX,
                                inverse=True, run_over="primitive")
    con_out = create(sub, "switch", "constraints_result")
    con_out.setInput(0, nulls[CONSTRAINTS])
    con_out.setInput(1, scale_xform(sub, "unscale_constraints", con_attribs, inverse=True))
    switch_on_spare_input(con_out, nulls[CONSTRAINTS])

    # Impacts: the pieces object's Impacts records as points (only when Add Impact Data is on and
    # something pulls IMPACTS, e.g. the Object Merge Fetch Impacts makes).
    import_impacts = create(sub, "dopimportrecords", "import_impacts")
    set_by_labels(import_impacts, ["DOP Network"], "../dopnet")
    set_by_labels(import_impacts, ["Object Mask", "Objects", "Object Pattern"], "pieces")
    set_by_labels(import_impacts, ["Data", "Data Name", "Data Path", "Subdata Name"], "Impacts")
    set_by_labels(import_impacts, ["Record Type", "Records", "Record"], "Impacts")
    create(sub, "null", IMPACTS).setInput(
        0, scale_wrangle(sub, "unscale_impacts", import_impacts, UNSCALE_IMPACTS_VEX, inverse=True))

    # Before the Start Frame the inputs pass through, as RBD Bullet Solver (the proxy output shows what
    # is simulated: the proxy, else the geometry).
    geo_out = before_start(sub, dopnet, "switchif_beforestart", geo_out, nulls[GEO])
    con_out = before_start(sub, dopnet, "switchif_beforestart_constraints", con_out, nulls[CONSTRAINTS])
    proxy_out = before_start(sub, dopnet, "switchif_beforestart_proxy", proxy_out, sim_input)
    points_out = scale_wrangle(sub, "unscale_points", sim_points, UNSCALE_POINTS_VEX, inverse=True)
    for idx, src in enumerate([geo_out, con_out, proxy_out, points_out]):
        out = create(sub, "output", f"OUT_{['geometry', 'constraints', 'proxy', 'points'][idx]}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 0:
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    sub.layoutChildren()
    return dopnet, solver, dopnet.node("pieces")


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


def pull(entries, pred):
    """(`entries` without the templates `pred` picks, recursing into folders; the templates picked)."""
    kept, picked = [], []
    for pt in entries:
        if pred(pt):
            picked.append(pt)
            continue
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            inner, more = pull(pt.parmTemplates(), pred)
            pt.setParmTemplates(inner)
            picked += more
        kept.append(pt)
    return kept, picked


def copy_attributes(xform, toggle_parm):
    """Transform Pieces' Attributes to Copy: its toggle follows the asset's `toggle_parm`, its list the
    asset's Transfer Attributes. The toggle is the one named in the list's disable condition (or a
    toggle with the same label)."""
    group = xform.parmTemplateGroup()
    found = [pt for pt in group.entriesWithoutFolders() if pt.label().lower() == "attributes to copy"]
    attribs = next((pt for pt in found if pt.type() == hou.parmTemplateType.String), None)
    if attribs is None:
        warn(f"{xform.path()} has no Attributes to Copy list; its parameters: {labels(xform)}")
        return
    xform.parm(attribs.name()).set(f'`chs("../{TRANSFER_ATTRIBUTES}")`')
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
    xform.parm(toggle).setExpression(f'ch("../{toggle_parm}")', hou.exprLanguage.Hscript)


def set_toggles(node, toggles):
    """Each toggle labelled like a key of `toggles` to its value."""
    for label, value in toggles.items():
        set_by_label(node, label, int(value))


def packed_object(dopnet, name, context_input, object_name):
    obj = create(dopnet, PACKED_OBJECT, name)
    set_by_label(obj, "Object Name", object_name)
    set_by_label(obj, "SOP Path", f'`dopcontextgeo(".", {context_input})`')
    pt = parm_by_label(obj, "Use Object Transform")
    if pt is not None:
        pt.set((0,))  # SOP space, like RBD Bullet Solver
    return obj


def menu_index_expr(parm, cond, on_label, off_label):
    """Drives the menu `parm` with an HScript expression: the item labelled like `on_label` while
    `cond` is true, else the one like `off_label`. False if either item is missing."""
    found = {}
    for want in (on_label, off_label):
        for i, lab in enumerate(parm.menuLabels()):
            if want.lower() in lab.lower():
                found[want] = i
                break
    if len(found) != 2:
        warn(f"{parm.path()}: no items like {on_label!r} / {off_label!r} in {list(parm.menuLabels())}")
        return False
    if parm.parmTemplate().type() == hou.parmTemplateType.String:
        items = list(parm.menuItems())
        expr = f'ifs({cond}, "{items[found[on_label]]}", "{items[found[off_label]]}")'
    else:
        expr = f"if({cond}, {found[on_label]}, {found[off_label]})"
    parm.setExpression(expr, hou.exprLanguage.Hscript)
    return True


def animate_collision(collision):
    """Animated Collision on the asset drives the collision object: Body Type Kinematic / Static, and
    every Parameter Operation of its inner SOP Geometry Set Always / Set Initial, so the SOP is
    re-imported each step and the solver sees the new poses. The nested Packed Object is made
    editable for that (rebuilt with this asset every build)."""
    pt = parm_by_label(collision, "Body Type")
    if pt is None:
        warn(f"{collision.path()} has no Body Type; its parameters: {labels(collision)}")
    else:
        menu_index_expr(pt[0], f'ch("../../{ANIMATED_COLLISION}")', "Kinematic", "Static")
    collision.allowEditingOfContents()
    sopgeo = collision.node("geometry")
    if sopgeo is None:
        warn(f"no SOP Geometry node named 'geometry' in {collision.path()}: the collision cannot animate")
        return
    ops = [p for p in sopgeo.parms()
           if p.name().startswith("parmop") or p.parmTemplate().label().lower() == "default operation"]
    if not ops:
        warn(f"{sopgeo.path()} has no Parameter Operation menus: the collision is imported once and cannot animate")
    driven = [p.name() for p in ops if menu_index_expr(p, f'ch("../../../{ANIMATED_COLLISION}")', "Always", "Initial")]
    print(f"make_solver_sop: Animated Collision drives {sopgeo.path()}: {', '.join(driven) or 'nothing'}")


def build_dops(dopnet):
    pieces = packed_object(dopnet, "pieces", DOP_PIECES, "pieces")

    # Collision: packed colliders as convex hulls unless a point sets bullet_georep. Animated
    # Collision: Kinematic (the solver moves each body to its new pose) and the SOP re-imported
    # every step; off: Static, imported once.
    collision = packed_object(dopnet, "collision", DOP_COLLISION, "collision")
    set_menu_item(collision, "Default Shape", "Convex Hull")
    animate_collision(collision)

    # Constraints: every supported relationship, the network from the second context geometry,
    # attached to the pieces object as its ConstraintGeometry (as RBD Bullet Solver does), so the
    # constraints output can DOP Import it. Only the pieces go through it: the collision object
    # would get its own copy and every joint would be made twice. Anchors naming collision pieces
    # still resolve (piece names are looked up in every object).
    rels = []
    for type_names, name in RELATIONSHIPS:
        try:
            rels.append(create(dopnet, type_names, name))
        except RuntimeError as e:
            warn(f"{e}; constraints of that type will not work in the Box3D Solver SOP")
    rel_merge = create(dopnet, "merge", "relationships")
    for i, r in enumerate(rels):
        rel_merge.setInput(i, r)
    network = create(dopnet, "constraintnetwork", "constraintnetwork")
    network.setInput(0, pieces)
    network.setInput(1, rel_merge)
    set_menu_item(network, "Geometry Source", "Second Context")
    set_by_label(network, "Attach Internal Constraints to Object", 1)
    constrained = create(dopnet, "switch", "use_constraints")
    constrained.setInput(0, pieces)
    constrained.setInput(1, network)
    switch_on_context_geo(constrained, DOP_CONSTRAINTS)

    both = create(dopnet, "merge", "pieces_and_collision")
    both.setInput(0, constrained)
    both.setInput(1, collision)
    objects = create(dopnet, "switch", "use_collision")
    objects.setInput(0, constrained)
    objects.setInput(1, both)
    p = objects.parm("input")
    if p is None:
        warn(f"{objects.path()} has no Select Input parameter")
    else:  # Use External Collisions on, and the collision input has points (as RBD Bullet Solver)
        p.setExpression(f'ch("../../{USE_COLLISIONS}") && (npoints(opinputpath("..", {DOP_COLLISION})) > 0)',
                        hou.exprLanguage.Hscript)

    # Forces: a FORCES subnet (the asset's dive target) with PRESOLVE and POSTSOLVE outputs, run by
    # a Multiple Solver around the Box3D Solver: PRESOLVE, Box3D Solver, POSTSOLVE. POP forces
    # wired to PRESOLVE write `force`, which the solver applies and clears in the same step.
    forces = build_forces(dopnet)
    solver = create(dopnet, "box3d_solver", "box3d_solver")
    multi = create(dopnet, "multisolver", "multisolver")
    multi.setInput(0, objects)
    multi.setInput(1, forces, FORCES_OUTPUTS.index("PRESOLVE"))
    multi.setInput(2, solver)
    multi.setInput(3, forces, FORCES_OUTPUTS.index("POSTSOLVE"))
    out = create(dopnet, "output", "output")
    out.setInput(0, multi)
    out.setDisplayFlag(True)
    dopnet.layoutChildren()
    return solver


# The DOP node marking a subnet output (DOP_SubNetOutput). Not "output": in DOPs that is the Output
# DOP that saves the simulation to disk.
SUBNET_OUTPUT_TYPE = "subnetoutput"


def build_forces(dopnet):
    """The FORCES DOP subnet: one subnet-output node per FORCES_OUTPUTS entry, named after it, its
    Output Index its position, with nothing wired in. The user wires POP forces and other
    microsolvers into them."""
    forces = create(dopnet, "subnet", FORCES)
    for child in forces.children():
        child.destroy()
    outs = []
    for i, name in enumerate(FORCES_OUTPUTS):
        out = create(forces, SUBNET_OUTPUT_TYPE, name)
        outs.append(out)
        if out.parm("outputidx") is None:
            warn(f"{out.path()} has no Output Index: FORCES outputs are not set")
            continue
        out.parm("outputidx").set(i)
    layout_forces(forces, outs)
    return forces


def layout_forces(forces, outs):
    """PRESOLVE on the left, POSTSOLVE on the right, each with its sticky note on its outer side;
    the subnet's inputs moved up out of the way."""
    # Node names are drawn to the right of a node, so the right-hand note sits further out.
    gap_left, gap_right, spacing = 1.0, 3.0, 6.0
    for i, out in enumerate(outs):
        out.setPosition(hou.Vector2(i * spacing, 0.0))
    for i, item in enumerate(forces.indirectInputs()):
        item.setPosition(hou.Vector2(i * 1.5, 4.0))
    for i, (out, text) in enumerate(zip(outs, FORCES_NOTES)):
        note = forces.createStickyNote(f"{out.name().lower()}_note")
        note.setText(text)
        note.setSize(hou.Vector2(4.5, 1.6))
        w, h = note.size()[0], note.size()[1]
        nw, nh = out.size()[0], out.size()[1]
        x = out.position()[0] - gap_left - w if i == 0 else out.position()[0] + nw + gap_right
        note.setPosition(hou.Vector2(x, out.position()[1] + nh / 2 - h / 2))


def edit_dialog_script(definition):
    """Input / output labels, in the DialogScript (hou has no setter for them)."""
    section = definition.sections().get("DialogScript")
    if section is None:
        warn("the asset has no DialogScript section: input / output labels not set")
        return
    lines = [ln for ln in section.contents().splitlines()
             if not re.match(r"\s*(inputlabel|outputlabel)\b", ln)]
    extra = [f'    inputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(INPUT_LABELS)]
    extra += [f'    outputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(OUTPUT_LABELS)]
    at = next((i + 1 for i, ln in enumerate(lines) if re.match(r"\s*label\b", ln)), 1)
    section.setContents("\n".join(lines[:at] + extra + lines[at:]) + "\n")




def data_folders(packed):
    """A Box3D Packed Object's Box3D Data folders. Found by content (folders holding parameters its
    inner Box3D Data has), not by name: Houdini merges adjacent tabs into one folder set and renames
    all but the first (b3dpo_object_1 ...), and Object is the first."""
    inner = packed.node("box3d_data") if packed is not None else None
    names = {pt.name() for pt in inner.parmTuples()} if inner is not None else set()
    found = [e for e in packed.parmTemplateGroup().entries()
             if e.type() == hou.parmTemplateType.Folder and e.label() != "Object"
             and any(pt.name() in names for pt in value_templates(e.parmTemplates()))] if packed else []
    if not found:
        warn(f"no Box3D Data folders on {packed.path() if packed else '?'}: its settings are not promoted")
    return found


def build(out_path, packed_hda=None):
    if packed_hda:
        hou.hda.installFile(packed_hda, force_use_assets=True)
    if hou.nodeType(hou.dopNodeTypeCategory(), PACKED_OBJECT) is None:
        raise RuntimeError(f"the {PACKED_OBJECT} asset is not installed (build it with make_packed_object.py)")

    hou.hipFile.clear(suppress_save_prompt=True)
    geo = hou.node("/obj").createNode("geo", "build")
    for child in geo.children():
        child.destroy()
    sub = geo.createNode("subnet", TYPE_NAME)
    dopnet, solver, pieces = build_network(sub)

    # Parameters: Reset Simulation and Start Frame above the tabs; then the tabs Solver (ending with
    # Cache Simulation and the contact settings), Collision (the collision toggles and the
    # collider's settings), Forces, Pieces (Box3D Data folders), Misc, Advanced.
    # (inner node's path relative to the subnet, inner tuple name, outer tuple name): paths, since
    # the subnet node is replaced by the asset instance below.
    links = []
    top = []
    if dopnet.parm("resimulate") is not None:
        top.append(hou.ButtonParmTemplate(
            "resimulate", "Reset Simulation",
            script_callback="kwargs['node'].node('dopnet').parm('resimulate').pressButton()",
            script_callback_language=hou.scriptLanguage.Python,
            help="Clears the cached simulation and simulates again from the start frame."))
    dop_templates = {}
    for label in DOPNET_PARMS:
        pt = template_by_label(dopnet, label)
        if pt is None:
            warn(f"{dopnet.path()} has no parameter labelled {label!r}; not promoted. "
                 f"Its parameters: {labels(dopnet)}")
            continue
        dop_templates[label] = pt.clone()
        links.append((sub.relativePathTo(dopnet), pt.name(), pt.name()))
    if "Start Frame" in dop_templates:
        top.append(dop_templates["Start Frame"])
    solver_first = [dop_templates["Cache Simulation"]] if "Cache Simulation" in dop_templates else []
    # Solve on Creation Frame, before Cache Simulation (as on RBD Bullet Solver's first tab), linked to
    # the pieces' and the collider's Packed Object.
    solve_first = hou.ToggleParmTemplate(
        SOLVE_FIRST, "Solve on Creation Frame", default_value=False,
        help="Simulates pieces and colliders on the frame they appear. Off: they start moving one frame later.")
    for obj_name in ("pieces", "collision"):
        obj = dopnet.node(obj_name)
        pt = template_by_label(obj, SOLVE_FIRST_LABEL) if obj is not None else None
        if pt is None:
            warn(f"{dopnet.path()}/{obj_name} has no {SOLVE_FIRST_LABEL!r}: {SOLVE_FIRST} does nothing there")
        else:
            links.append((sub.relativePathTo(obj), pt.name(), SOLVE_FIRST))
    solver_first = [solve_first] + solver_first
    # World Scale, first on the Solver tab (RBD Bullet Solver's usescale / Bullet World Scale).
    use_scale = hou.ToggleParmTemplate(WORLD_SCALE_TOGGLE, "Use World Scale", default_value=False,
                                       join_with_next=True,
                                       help="Simulates the scene scaled by World Scale.")
    use_scale.hideLabel(True)
    world_scale = hou.FloatParmTemplate(
        WORLD_SCALE, "World Scale", 1, default_value=(1.0,), min=0.01, max=100.0, min_is_strict=True,
        help="Scales the inputs by this for the simulation and the outputs back. Use it when pieces are "
             "much smaller or larger than about 1 unit.")
    world_scale.setConditional(hou.parmCondType.DisableWhen, f"{{ {WORLD_SCALE_TOGGLE} == 0 }}")

    solver_tabs = solver_folders(without_parmops(solver.parmTemplateGroup().entries()))
    if not solver_tabs:
        warn(f"no parameter folders found on {solver.path()}: the solver's parameters are not promoted")
    taken = {pt.name() for pt in top + solver_first} | {USE_COLLISIONS, ANIMATED_COLLISION,
                                                       WORLD_SCALE_TOGGLE, WORLD_SCALE}
    for tab in solver_tabs:
        for pt in value_templates(tab.parmTemplates()):
            links.append((sub.relativePathTo(solver), pt.name(), pt.name()))
            taken.add(pt.name())

    # The collider's Shape, Material and Sleep & Contacts folders, every name prefixed (they would clash with the
    # pieces' and the solver's), Default Shape defaulting to Convex Hull.
    collision = dopnet.node("collision")
    collider_tabs = [t for t in data_folders(collision) if t.label() in COLLIDER_FOLDERS]
    if len(collider_tabs) != len(COLLIDER_FOLDERS):
        warn(f"collider folders found: {[t.label() for t in collider_tabs]}, expected {COLLIDER_FOLDERS}")
    collider_tabs = drop_templates(collider_tabs, COLLIDER_DROPPED)
    collider_renames = {pt.name(): f"collision_{pt.name()}" for pt in value_templates(collider_tabs)}
    collider_folders = []
    for i, tab in enumerate(rename_templates(collider_tabs, collider_renames)):
        templates = tab.parmTemplates()
        for t in templates:
            if t.name() == "collision_defaultshape" and t.type() == hou.parmTemplateType.Menu:
                convex = [j for j, lab in enumerate(t.menuLabels()) if "convex" in lab.lower()]
                if convex:
                    t.setDefaultValue(convex[0])
        label = COLLIDER_RELABEL.get(tab.label(), tab.label())
        collider_folders.append(hou.FolderParmTemplate(f"b3dsop_collider{i}", label, templates,
                                                       folder_type=hou.folderType.Simple))
    for old, new in collider_renames.items():
        links.append((sub.relativePathTo(collision), old, new))
        taken.add(new)
    use_collisions = hou.ToggleParmTemplate(
        USE_COLLISIONS, "Use External Collisions", default_value=True,
        help="Collides the pieces with the Collision input. Off: the Collision input is ignored.")
    animated = hou.ToggleParmTemplate(
        ANIMATED_COLLISION, "Animated Collision", default_value=True,
        help="Collision geometry follows its animation and pushes what it hits. Off: it stays where it starts.")
    animated.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    detach = hou.ToggleParmTemplate(
        DETACH_FACES, "Detach Faces", default_value=False,
        help="Rebuilds colliders from unshared points, repacking packed ones (RBD Bullet Solver's hidden option).")
    detach.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    # One open Collision Geometry folder, the collider's folders simple inside it (as RBD Bullet Solver's).
    collision_items = [collapsible("b3dsop_collision_geometry", "Collision Geometry",
                                   [use_collisions, animated, detach] + collider_folders, True)]
    taken.update([TRANSFER_ATTRIBUTES, DETACH_FACES, KEEP_BROKEN] + [n for n, _ in TRANSFER_TO.values()])

    group = hou.ParmTemplateGroup()
    for pt in top:
        group.append(pt)
    # The solver's tabs by label, with parameters moved where the SOP shows them (TAB_MOVES: Gravity
    # first in Forces). The solver's Collision tab (contact settings) goes into Solver as a Contacts
    # folder after Cache Simulation, so the SOP's Collision tab is only about the Collision input.
    tabs = [(TAB_RENAMES.get(t.label(), t.label()), list(t.parmTemplates())) for t in solver_tabs]
    for name, target in TAB_MOVES.items():
        moved = None
        for _, contents in tabs:
            for pt in list(contents):
                if pt.name() == name:
                    contents.remove(pt)
                    moved = pt
        for label, contents in tabs:
            if moved is not None and label == target:
                contents.insert(0, moved)
    # Out of the solver's tabs into Output: the Aging folder (Misc) and Impact Min Speed (Advanced).
    aging, speed = [], []
    for i, (label, contents) in enumerate(tabs):
        contents, a = pull(contents, lambda pt: pt.type() == hou.parmTemplateType.Folder
                           and pt.label() == AGING_FOLDER)
        contents, sp = pull(contents, lambda pt: pt.name() == IMPACT_SPEED)
        tabs[i] = (label, contents)
        aging += a
        speed += sp
    contacts = next((c for label, c in tabs if label == "Collision"), [])
    solver_extra = solver_first + ([collapsible("b3dsop_contacts", "Contacts", contacts, True)] if contacts else [])
    # Tab order: Solver, Collision, Forces, Pieces, any other solver tabs, then Misc, Advanced and
    # Output always last.
    by_label = {label: contents for label, contents in tabs}
    ordered = [("Solver", [use_scale, world_scale] + by_label.pop("Solver", []) + solver_extra),
               ("Collision", collision_items)]
    by_label.pop("Collision", None)
    last = [(label, by_label.pop(label)) for label in ("Misc", "Advanced") if label in by_label]
    if "Forces" in by_label:
        ordered.append(("Forces", by_label.pop("Forces")))
    ordered.append(("Pieces", None))  # filled below
    ordered += list(by_label.items()) + last + [("Output", None)]  # filled below
    print("make_solver_sop: solver tabs: " + ", ".join(t.label() for t in solver_tabs))

    data_tabs = drop_templates(data_folders(pieces), PIECES_DROPPED)
    print("make_solver_sop: Pieces folders: " + ", ".join(t.label() for t in data_tabs))
    renames = {pt.name(): f"piece_{pt.name()}" for pt in value_templates(data_tabs) if pt.name() in taken}
    if renames:
        print(f"make_solver_sop: Pieces parameters renamed (the solver has the same names): {renames}")
    folders = []
    for i, tab in enumerate(rename_templates(data_tabs, renames)):
        folders.append(hou.FolderParmTemplate(f"b3dsop_pieces{i}", tab.label(), tab.parmTemplates(),
                                              folder_type=hou.folderType.Simple))
    inverse = {new: old for old, new in renames.items()}
    for pt in value_templates(folders):
        links.append((sub.relativePathTo(pieces), inverse.get(pt.name(), pt.name()), pt.name()))
    # The pieces' Add Impact Data goes to Output (still linked by its name).
    impact_name = renames.get(IMPACT_TOGGLE, IMPACT_TOGGLE)
    folders, impact = pull(folders, lambda pt: pt.name() == impact_name)
    # The Pieces tab as Bullet's Properties tab: an open Pieces folder holding the simple Box3D Data
    # folders, and Sleep & Contacts beside it (Bullet's Sleeping).
    nested, separate = pull(folders, lambda pt: pt.label() == PIECES_SEPARATE)
    pieces_items = [collapsible("b3dsop_pieces", "Pieces", nested, True)] + \
        [as_collapsible(f, True) for f in separate]
    output_items = output_tab(impact, speed, aging)
    for i, (label, contents) in enumerate(ordered):
        contents = {"Pieces": pieces_items, "Output": output_items, "Collision": contents}.get(
            label, group_tab(label, contents) if contents is not None else contents)
        group.append(hou.FolderParmTemplate(f"b3dsop_tab{i}", label, contents, folder_type=hou.folderType.Tabs))

    if os.path.exists(out_path):
        try:
            hou.hda.uninstallFile(out_path)
        except hou.OperationFailed:
            pass
        os.remove(out_path)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    asset = sub.createDigitalAsset(name=TYPE_NAME, hda_file_name=out_path, description=LABEL,
                                   min_num_inputs=1, max_num_inputs=4)
    definition = asset.type().definition()
    definition.setParmTemplateGroup(group)
    definition.setMaxNumOutputs(4)
    definition.setIcon(ICON)
    definition.setComment("Simulates packed pieces with Box3D, like RBD Bullet Solver: geometry, "
                          "constraints, proxy and collision in; geometry, constraints, proxy and "
                          "simulation points out.")
    # Diving into the asset lands in FORCES (the DiveTarget section holds the relative path), and
    # FORCES is an editable node, so forces can be added without unlocking the asset (the
    # EditableNodes section: space-separated relative paths).
    definition.addSection("DiveTarget", DIVE_TARGET)
    definition.addSection("EditableNodes", DIVE_TARGET)

    asset.allowEditingOfContents()
    for inner_path, inner_name, outer_name in links:
        link(asset.node(inner_path), inner_name, asset, outer_name)
    scale_settings(asset)
    set_operations(asset.node("dopnet/box3d_solver"), "initial")
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    # Labels last, saved without the node so nothing regenerates the DialogScript over them.
    edit_dialog_script(definition)
    definition.save(out_path)

    check(out_path)
    print(f"Box3D Solver SOP asset: {os.path.normpath(out_path)}")


def output_tab(impact, speed, aging):
    """The Output tab's contents: Impact Data (a collapsible folder, `impact` its header toggle,
    then `speed` and Fetch Impacts), the `aging` folders, Transfer Attributes."""
    items = []
    if not impact:
        warn(f"no {IMPACT_TOGGLE} on the pieces: Impact Data is not on the Output tab")
    else:
        toggle = impact[0].clone()
        off = f"{{ {toggle.name()} == 0 }}"
        contents = [toggle]
        for pt in speed:
            pt = pt.clone()
            pt.setConditional(hou.parmCondType.DisableWhen, off)
            contents.append(pt)
        fetch = hou.ButtonParmTemplate(
            "fetchimpacts", "Fetch Impacts", script_callback=FETCH_IMPACTS,
            script_callback_language=hou.scriptLanguage.Python,
            help="Adds an Object Merge beside this node with the pieces' impacts as points.")
        fetch.setConditional(hou.parmCondType.DisableWhen, off)
        contents.append(fetch)
        items.append(hou.FolderParmTemplate("b3dsop_impacts", "Impact Data", contents,
                                            folder_type=hou.folderType.Collapsible,
                                            tags={"sidefx::header_toggle": toggle.name()}))
    if not aging:
        warn(f"no {AGING_FOLDER} folder on the solver: not on the Output tab")
    for pt in aging:
        items.append(collapsible("b3dsop_aging", pt.label(), pt.parmTemplates(), True))
    transfer = hou.StringParmTemplate(
        TRANSFER_ATTRIBUTES, "Transfer Attributes", 1, default_value=(TRANSFER_ATTRIBUTES_DEFAULT,),
        help="Point attributes copied from the simulation onto Simulation Points, and onto the pieces as ticked below.")
    to = [hou.ToggleParmTemplate(name, label, default_value=True,
                                 help=f"Copies Transfer Attributes onto the {label[12:]} output.")
          for name, label in TRANSFER_TO.values()]
    items.append(collapsible("b3dsop_transfer", "Attribute Transfer", [transfer] + to, False))
    keep = hou.ToggleParmTemplate(KEEP_BROKEN, "Export Broken Constraints", default_value=False,
                                  help="Keeps broken constraints in the Constraint Geometry output. Off: they are removed.")
    items.append(collapsible("b3dsop_constraints", "Constraints", [keep], True))
    return items


def link(inner, inner_name, outer, outer_name):
    """Every component of inner's tuple `inner_name` a channel reference to outer's `outer_name`."""
    src = inner.parmTuple(inner_name) if inner is not None else None
    dst = outer.parmTuple(outer_name)
    if src is None or dst is None or len(src) != len(dst):
        warn(f"cannot link {inner.path() if inner else '?'}/{inner_name} to {outer.path()}/{outer_name}")
        return
    for s, d in zip(src, dst):
        s.set(d)  # a hou.Parm value: a channel reference


def check(out_path):
    """On a fresh instance of the saved asset: the links hold, the labels are there."""
    hou.hda.installFile(out_path, force_use_assets=True)
    geo = hou.node("/obj").createNode("geo", "probe_geo")
    probe = geo.createNode(TYPE_NAME, "probe")
    dive = probe.type().definition().sections().get("DiveTarget")
    if dive is None or dive.contents().strip() != DIVE_TARGET or probe.node(DIVE_TARGET) is None:
        warn(f"dive target {dive.contents() if dive else None!r} does not lead to {DIVE_TARGET}")
    forces = probe.node(DIVE_TARGET)
    if forces is not None and not forces.isEditable():
        warn(f"{DIVE_TARGET} is not editable inside a locked instance (EditableNodes section)")
    labels_now = list(probe.inputLabels())
    if labels_now[:4] != INPUT_LABELS:
        warn(f"input labels are {labels_now}, expected {INPUT_LABELS}")
    if len(probe.outputConnectors()) != 4:
        warn(f"the asset has {len(probe.outputConnectors())} outputs, expected 4")
    for outer, inner_path, inner_name in (("friction", "dopnet/pieces", "friction"),
                                          ("collision_friction", "dopnet/collision", "friction"),
                                          ("stepsperframe", "dopnet/box3d_solver", "stepsperframe")):
        p, inner = probe.parm(outer), probe.node(inner_path)
        q = inner.parm(inner_name) if inner is not None else None
        if p is None or q is None:
            warn(f"cannot check the link of {outer} ({inner_path})")
            continue
        p.set(p.eval() * 0.5 + 0.125 if p.parmTemplate().type() == hou.parmTemplateType.Float else p.eval() + 1)
        if abs(q.eval() - p.eval()) > 1e-6:
            warn(f"{inner_path}/{outer} reads {q.eval()} after setting the asset's {outer} to {p.eval()}")
    # World Scale 2: the inner gravity doubles, the pieces' density is an eighth.
    if probe.parm(WORLD_SCALE_TOGGLE) is not None and probe.parm(WORLD_SCALE) is not None:
        probe.parm(WORLD_SCALE_TOGGLE).set(1)
        probe.parm(WORLD_SCALE).set(2.0)
        for outer, inner_path, inner_name, ratio in (
                ("gravity", "dopnet/box3d_solver", "gravity", 2.0),
                ("density", "dopnet/pieces", "density", 0.125)):
            inner = probe.node(inner_path)
            # By tuple (component names vary): gravity's y, the one that is not 0 by default.
            pt, qt = probe.parmTuple(outer), inner.parmTuple(inner_name) if inner is not None else None
            p, q = (pt[min(1, len(pt) - 1)], qt[min(1, len(qt) - 1)]) if pt is not None and qt is not None \
                else (None, None)
            if p is None or q is None:
                warn(f"World Scale: cannot check {outer} ({inner_path})")
            elif abs(q.eval() - p.eval() * ratio) > 1e-6 * max(1.0, abs(p.eval())):
                warn(f"World Scale 2: {inner_path}/{inner_name} reads {q.eval()}, expected {p.eval() * ratio}")
        probe.parm(WORLD_SCALE_TOGGLE).set(0)
    else:
        warn("the asset has no World Scale parameters")
    if probe.node(IMPACTS) is None or probe.parm("fetchimpacts") is None:
        warn(f"no {IMPACTS} node or Fetch Impacts button on the asset")
    tabs = [f.label() for f in probe.parmTemplateGroup().entries() if f.type() == hou.parmTemplateType.Folder]
    if not tabs or tabs[-1] != "Output":
        warn(f"the asset's tabs are {tabs}: Output is not last")
    for path, style in (("import_points", "fetch geometry"), ("import_constraints", "fetch geometry")):
        imp = probe.node(path)
        p = imp.parm("importstyle") if imp is not None else None
        if p is None or not p.menuLabels()[p.eval()].lower().startswith(style):
            warn(f"{path}'s Import Style is not {style!r}")
    pts = probe.node("sim_points")
    if pts is not None and pts.geometry() is not None and len(pts.geometry().prims()):
        warn(f"sim_points has {len(pts.geometry().prims())} primitive(s); expected points only")
    keep = probe.node("sim_points")
    if keep is None or TRANSFER_ATTRIBUTES_DEFAULT not in keep.parm("ptdel").eval():
        warn(f"sim_points does not keep the asset's {TRANSFER_ATTRIBUTES}")
    for inner_path in ("dopnet/pieces", "dopnet/collision"):
        outer = SOLVE_FIRST
        p, obj = probe.parm(outer), probe.node(inner_path)
        q = parm_by_label(obj, SOLVE_FIRST_LABEL) if obj is not None else None
        if p is None or q is None:
            warn(f"cannot check {outer} -> {inner_path}")
            continue
        p.set(1)
        if q[0].eval() != 1:
            warn(f"{inner_path}'s {SOLVE_FIRST_LABEL} does not follow the asset's {outer}")
        p.set(0)
    for parm in (DETACH_FACES, KEEP_BROKEN):
        if probe.parm(parm) is None:
            warn(f"no {parm} parameter on the asset")
    for path in ("switchif_beforestart", "switchif_keepbroken", "use_detach_faces", "collision_detach/repack"):
        if probe.node(path) is None:
            warn(f"no {path} inside the asset")
    for path in ("prepare_pieces", "prepare_collision", "names_geometry", "names_proxy"):
        prep = probe.node(path)
        if prep is None or prep.node("output0") is None:
            warn(f"no {path} subnet with an output0 inside the asset")
    for path, index in (("dopnet/pieces", DOP_PIECES), ("dopnet/collision", DOP_COLLISION)):
        obj = probe.node(path)
        pt = parm_by_label(obj, "SOP Path") if obj is not None else None
        want = f'dopcontextgeo(".", {index})'
        if pt is None or want not in pt[0].unexpandedString():
            warn(f"{path}'s SOP Path is {pt[0].unexpandedString() if pt else '?'!r}, expected `{want}`")
    geo.destroy()


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        return 0
    try:
        build(sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else None)
    except Exception:  # noqa: BLE001 - never fail the plugin build over the asset
        warn("the Box3D Solver SOP asset was not built:\n" + traceback.format_exc())
    return 0


if __name__ == "__main__":
    sys.exit(main())
