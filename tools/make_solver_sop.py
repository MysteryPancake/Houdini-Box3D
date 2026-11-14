"""Builds the Box3D Solver SOP digital asset (a SOP, like RBD Bullet Solver) with hython.

    hython tools/make_solver_sop.py <output .hda> [<box3d_packedobject .hda>]

The build runs this after make_packed_object.py (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_solver_sop.hda. The plugin and the Box3D Packed Object asset must exist: the
asset contains a Box3D Solver DOP and two Box3D Packed Objects, and their parameters are copied
from them. Pass the Box3D Packed Object .hda to install it first (the build does).

Inputs and outputs (the outputs match the inputs):

    0  Geometry      packed pieces. Simulated, unless a proxy is connected
    1  Constraints   constraint network (two-point lines); its subnet input is coloured pink
    2  Proxy         packed pieces simulated in place of Geometry; Geometry follows them
    3  Collision     packed colliders, following their animation (Animated Collision) or static

Inside:

    IN_* nulls -> sim_source (Proxy when it has points, else Geometry) -> dopnet (SOP-level DOP network)
      dopnet: pieces (Box3D Packed Object, SOP Path `dopcontextgeo(".", 0)`)
              -> constraintnetwork (Geometry Source: Second Context Geometry, Attach Internal Constraints
                 to Object, only when input 1 has points; every Box3D and SideFX constraint
                 relationship merged into it)
              collision (Box3D Packed Object, convex hulls, `dopcontextgeo(".", 2)`, only when Use External
                         Collisions is on and input 3 has points; Animated Collision: Kinematic and re-imported every step, else Static)
                         merged in after the constraint network
              -> multisolver (FORCES/PRESOLVE, box3d_solver, FORCES/POSTSOLVE) -> output
              FORCES: DOP subnet, the asset's dive target, with Output DOPs PRESOLVE (output 0) and
                      POSTSOLVE (output 1) for POP forces and other microsolvers run before / after
                      the Box3D Solver
    import_sim (DOP Import) -> proxy output, and Transform Pieces onto Geometry when a proxy is used
    import_constraints (DOP Import of the pieces' ConstraintGeometry) -> constraints output
    IN_collision -> collision output

Its parameters: Reset Simulation and Start Frame above the tabs; then the tabs Solver (the
solver's own, Cache Simulation, then its contact settings in a Contacts folder), Collision (Use
External Collisions, Animated Collision and the collider's Shape, Material and Sleep & Contacts settings,
collision_* names), Forces (Gravity first), Pieces (Box3D Packed Object's Box3D Data folders for the
simulated pieces), Misc (Sleeping and Aging folders) and Advanced, always last. Each inner parameter is a channel reference to the outer one.
A Box3D Data parameter whose name the solver already uses gets a "piece_" prefix.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import re
import sys
import traceback

import hou

TYPE_NAME = "box3d_solver"  # a SOP; the DOP of the same name lives in another node table
LABEL = "Box3D Solver"
ICON = "DOP_box3d_solver"  # installed by the build for the DOP (config/Icons)
PACKED_OBJECT = "box3d_packedobject"

# Colours of the subnet inputs, IN_* nulls and OUT_* nodes by index: constraints pink.
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}
ANIMATED_COLLISION = "animatedcollision"
USE_COLLISIONS = "usecollisions"
COLLIDER_FOLDERS = ["Shape", "Material", "Sleep & Contacts"]  # the collision object's Box3D Data folders on the Collision tab
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
OUTPUT_LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry", "Collision Geometry"]
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

    # What is simulated: the proxy when connected, else the geometry.
    sim_source = create(sub, "switch", "sim_source")
    sim_source.setInput(0, nulls[GEO])
    sim_source.setInput(1, nulls[PROXY])
    switch_on_spare_input(sim_source, nulls[PROXY])

    # The DOP network, its context geometry three streams (dopcontextgeo(".", i) inside).
    dopnet = create(sub, "dopnet", "dopnet")
    dopnet.setInput(DOP_PIECES, sim_source)
    dopnet.setInput(DOP_CONSTRAINTS, nulls[CONSTRAINTS])
    dopnet.setInput(DOP_COLLISION, nulls[COLLISION])
    for child in dopnet.children():
        child.destroy()
    solver = build_dops(dopnet)

    # Simulated pieces back out (the proxy output, or the geometry output without a proxy).
    import_sim = create(sub, "dopimport", "import_sim")
    import_sim.parm("doppath").set("../dopnet")
    import_sim.parm("objpattern").set("pieces")
    set_import_style(import_sim)

    # Geometry follows the simulated proxy: Transform Pieces, rest = the proxy as connected.
    xform = create(sub, "xformpieces", "transform_pieces")
    xform.setInput(0, nulls[GEO])
    xform.setInput(1, import_sim)
    xform.setInput(2, nulls[PROXY])
    # Every switch tests points, so an empty input counts as unconnected everywhere alike.
    geo_out = create(sub, "switch", "geometry_result")
    geo_out.setInput(0, import_sim)
    geo_out.setInput(1, xform)
    switch_on_spare_input(geo_out, nulls[PROXY])

    # Constraints as the solver left them: the network is attached to the pieces object
    # (Attach Internal Constraints to Object) as its ConstraintGeometry data.
    import_con = create(sub, "dopimport", "import_constraints")
    import_con.parm("doppath").set("../dopnet")
    import_con.parm("objpattern").set("pieces")
    set_import_style(import_con)
    set_by_label(import_con, "Geometry Data Path", "ConstraintGeometry")
    con_out = create(sub, "switch", "constraints_result")
    con_out.setInput(0, nulls[CONSTRAINTS])
    con_out.setInput(1, import_con)
    switch_on_spare_input(con_out, nulls[CONSTRAINTS])

    for idx, src in enumerate([geo_out, con_out, import_sim, nulls[COLLISION]]):
        out = create(sub, "output", f"OUT_{['geometry', 'constraints', 'proxy', 'collision'][idx]}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 0:
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    sub.layoutChildren()
    return dopnet, solver, dopnet.node("pieces")


def set_import_style(node):
    p = node.parm("importstyle")
    if p is None:
        warn(f"{node.path()} has no Import Style; its parameters: {labels(node)}")
        return
    item_labels = list(p.menuLabels())
    # "Fetch Geometry from DOP Network", not the Packed / Unpacked variants.
    for i, lab in enumerate(item_labels):
        if lab.lower().startswith("fetch geometry"):
            p.set(i)
            return
    warn(f"{p.path()}: no 'Fetch Geometry' item in {item_labels}")


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

    solver_tabs = solver_folders(without_parmops(solver.parmTemplateGroup().entries()))
    if not solver_tabs:
        warn(f"no parameter folders found on {solver.path()}: the solver's parameters are not promoted")
    taken = {pt.name() for pt in top + solver_first} | {USE_COLLISIONS, ANIMATED_COLLISION}
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
    collider_renames = {pt.name(): f"collision_{pt.name()}" for pt in value_templates(collider_tabs)}
    collider_folders = []
    for i, tab in enumerate(rename_templates(collider_tabs, collider_renames)):
        templates = tab.parmTemplates()
        for t in templates:
            if t.name() == "collision_defaultshape" and t.type() == hou.parmTemplateType.Menu:
                convex = [j for j, lab in enumerate(t.menuLabels()) if "convex" in lab.lower()]
                if convex:
                    t.setDefaultValue(convex[0])
        collider_folders.append(hou.FolderParmTemplate(f"b3dsop_collider{i}", f"Collider {tab.label()}", templates,
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
    collision_items = [use_collisions, animated] + collider_folders

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
    contacts = next((c for label, c in tabs if label == "Collision"), [])
    solver_extra = solver_first + ([hou.FolderParmTemplate("b3dsop_contacts", "Contacts", contacts,
                                                           folder_type=hou.folderType.Simple)] if contacts else [])
    # Tab order: Solver, Collision, Forces, Pieces, any other solver tabs, then Misc and Advanced
    # always last.
    by_label = {label: contents for label, contents in tabs}
    ordered = [("Solver", by_label.pop("Solver", []) + solver_extra), ("Collision", collision_items)]
    by_label.pop("Collision", None)
    last = [(label, by_label.pop(label)) for label in ("Misc", "Advanced") if label in by_label]
    if "Forces" in by_label:
        ordered.append(("Forces", by_label.pop("Forces")))
    ordered.append(("Pieces", None))  # filled below
    ordered += list(by_label.items()) + last
    print("make_solver_sop: solver tabs: " + ", ".join(t.label() for t in solver_tabs))

    data_tabs = data_folders(pieces)
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
    for i, (label, contents) in enumerate(ordered):
        group.append(hou.FolderParmTemplate(f"b3dsop_tab{i}", label, folders if label == "Pieces" else contents,
                                            folder_type=hou.folderType.Tabs))

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
                          "constraints, proxy and collision in, the same four out.")
    # Diving into the asset lands in FORCES (the DiveTarget section holds the relative path), and
    # FORCES is an editable node, so forces can be added without unlocking the asset (the
    # EditableNodes section: space-separated relative paths).
    definition.addSection("DiveTarget", DIVE_TARGET)
    definition.addSection("EditableNodes", DIVE_TARGET)

    asset.allowEditingOfContents()
    for inner_path, inner_name, outer_name in links:
        link(asset.node(inner_path), inner_name, asset, outer_name)
    set_operations(asset.node("dopnet/box3d_solver"), "initial")
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    # Labels last, saved without the node so nothing regenerates the DialogScript over them.
    edit_dialog_script(definition)
    definition.save(out_path)

    check(out_path)
    print(f"Box3D Solver SOP asset: {os.path.normpath(out_path)}")


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
    for outer, inner_path, inner_name in (("friction", "dopnet/pieces/box3d_data", "friction"),
                                          ("collision_friction", "dopnet/collision/box3d_data", "friction"),
                                          ("stepsperframe", "dopnet/box3d_solver", "stepsperframe")):
        p, inner = probe.parm(outer), probe.node(inner_path)
        q = inner.parm(inner_name) if inner is not None else None
        if p is None or q is None:
            warn(f"cannot check the link of {outer} ({inner_path})")
            continue
        p.set(p.eval() * 0.5 + 0.125 if p.parmTemplate().type() == hou.parmTemplateType.Float else p.eval() + 1)
        if abs(q.eval() - p.eval()) > 1e-6:
            warn(f"{inner_path}/{outer} reads {q.eval()} after setting the asset's {outer} to {p.eval()}")
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
