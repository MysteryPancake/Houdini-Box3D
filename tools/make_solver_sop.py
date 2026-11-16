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
    3  Collision     colliders, static, animated or deforming (Collision Type)

Unpacked inputs are packed by `name` (by connectivity when nothing is named), as RBD Bullet Solver.

Outputs: 0-2 as the inputs (moved by the simulation), 3 Simulation Points (one point per piece),
The collision shapes are the asset's Guide Geometry (GUIDE_GEO), not an output.

Inside:

    IN_* nulls -> sim_source (Proxy when it has points, else Geometry) -> prepare_pieces (pack mode)
      -> dopnet (SOP-level DOP network); IN_collision -> prepare_collision (pack mode) -> collision_rest
      (rest from a Time Shift to the start frame) -> dopnet input 2
      dopnet: pieces (Box3D Packed Object, SOP Path `dopcontextgeo(".", 0)`)
              -> constraintnetwork (Geometry Source: Second Context Geometry, Attach Internal Constraints
                 to Object, only when input 1 has points; every Box3D and SideFX constraint
                 relationship merged into it)
              collision (Box3D Packed Object, convex hulls, `dopcontextgeo(".", 2)`, only when Use External
                         Collisions is on and input 3 has points; Collision Type Animated / Deforming: Kinematic and re-imported every step, else Static)
                         merged in after the constraint network
              -> multisolver (FORCES/PRESOLVE, scale_forces, box3d_solver, FORCES/POSTSOLVE) -> output
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
    import_shapes (DOP Import of the objects' Box3DShapes data, written by the solver's Export Collision
      Shapes) -> tint_shapes (Shape Color) -> switch_show_shapes -> GUIDE_GEO

Its parameters: Reset Simulation and Start Frame above the tabs; then the tabs Solver (the
solver's own settings loose, then its contact settings in a Contacts folder, and the
closed Sleeping (Allow Sleeping) and Aging folders), Pieces
(Box3D Packed Object's Box3D Data folders for the simulated pieces: Shape, Body, Material, CCD & Contacts,
Sleep), Collision (a Collision
Geometry folder whose header toggle is Use External Collisions, holding Collision Type and the
collider's Shape, Material and Contacts settings, collision_* names), Forces (Gravity first, loose; Force DOPs and Bounce / Friction folders),
Visualization (Show Collision Geometry, Show Collision Shapes, Shape Color), Advanced (Stability and Performance folders;
Performance starts with the Cache Simulation checkbox joined to Cache Memory (MB)), and Output, always last
(Impact Data with Fetch Impacts, Transfer Attributes and Transfer to Geometry / Proxy Geometry,
Constraints). Every folder is open except Impact Data, Sleeping and Aging. Each inner parameter is a channel reference to
the outer one.
A Box3D Data parameter whose name the solver already uses gets a "piece_" prefix.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import re
import sys
import traceback

import hou

# No __pycache__ next to the scripts (they are imported from this folder).
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b3d_hda  # noqa: E402  (beside this script)
from b3d_hda import labels, pull, value_templates, without_parmops  # noqa: E402
from b3d_solver_common import (  # noqa: E402
    CACHE_MEMORY,
    COLLISION_TYPE,
    as_collapsible,
    collapsible,
    COLLIDER_DROPPED,
    COLLIDER_FOLDERS,
    COLLIDER_RELABEL,
    COLLISION_FLIP,
    COLLISION_SOURCE,
    DIVE_TARGET,
    DOP_COLLISION,
    DOP_PIECES,
    DOPNET_PARMS,
    drop_templates,
    FETCH_IMPACTS,
    group_tab,
    ICON,
    INITIAL_STATE,
    IMPACT_SPEED,
    IMPACT_TOGGLE,
    IMPACTS,
    INPUT_LABELS,
    LABEL,
    OUTPUT_LABELS,
    PACKED_OBJECT,
    parm_by_label,
    row_by_label,
    PIECES_DROPPED,
    rename_templates,
    GUIDE_GEO,
    SHAPES_COLOR,
    SHAPES_TOGGLE,
    SHOW_COLLISION,
    START_ASLEEP,
    STATIC_COLLIDER_FOLDERS,
    STATIC_DATA,
    solver_folders,
    TAB_GROUPS,
    SOLVER_FOLDERS,
    BOUNCE_PARMS,
    template_by_label,
    TRANSFER_ATTRIBUTES,
    TRANSFER_ATTRIBUTES_DEFAULT,
    TRANSFER_TO,
    TYPE_NAME,
    USE_COLLISIONS,
    warn,
)
from b3d_solver_constraints import constraint_items, finish_constraints  # noqa: E402
from b3d_solver_help import HELP_CARD  # noqa: E402
from b3d_solver_ground import (  # noqa: E402
    finish_ground,
    GROUND_DATA,
    GROUND_MATERIAL,
    ground_items,
    ground_visualization,
)
from b3d_solver_network import (  # noqa: E402
    build_network,
    IMPORT_CONSTRAINTS,
    IMPORT_PROXY,
    KEEP_BROKEN,
    POINT_VELOCITY_PARMS,
    scale_joint_settings,
    scale_settings,
    SOLVE_FIRST,
    SOLVE_FIRST_LABEL,
    WORLD_SCALE,
    WORLD_SCALE_TOGGLE,
)


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
    all but the first (b3dpo_object_1 ...), and Object and Initial State come before them (they hold
    Add Impact Data and Start Asleep, which `moved_settings` returns)."""
    inner = packed.node("box3d_data") if packed is not None else None
    names = {pt.name() for pt in inner.parmTuples()} if inner is not None else set()
    found = [e for e in packed.parmTemplateGroup().entries()
             if e.type() == hou.parmTemplateType.Folder and e.label() not in ("Object", "Initial State")
             and any(pt.name() in names for pt in value_templates(e.parmTemplates()))] if packed else []
    if not found:
        warn(f"no Box3D Data folders on {packed.path() if packed else '?'}: its settings are not promoted")
    return found


def moved_settings(packed):
    """The Box3D Data settings the Packed Object shows outside its data folders (Add Impact Data on the Object
    tab, Start Asleep on Initial State), as {name: template}."""
    group = packed.parmTemplateGroup()
    found = {name: group.find(name) for name in (IMPACT_TOGGLE, START_ASLEEP)}
    for name, pt in found.items():
        if pt is None:
            warn(f"{packed.path()} has no {name}: not on the Solver SOP")
    return {name: pt.clone() for name, pt in found.items() if pt is not None}


def initial_state_folder(packed):
    """An open Initial State folder with the Packed Object's INITIAL_STATE parameters (cloned: same names and
    conditions). None, with a warning, when one is missing."""
    group = packed.parmTemplateGroup()
    found = [group.find(name) for name in INITIAL_STATE]
    if any(pt is None for pt in found):
        warn(f"{packed.path()} lacks some of {INITIAL_STATE}: no Initial State folder on the Pieces tab")
        return None
    return collapsible("b3dsop_initial_state", "Initial State", [pt.clone() for pt in found])


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
    # the contact settings), Collision (the collision toggles and the
    # collider's settings), Forces, Pieces (Box3D Data folders, sleeping, aging), Advanced, Output.
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
    # Cache Simulation as RBD Bullet Solver's: a checkbox with its label hidden, joined to Cache Memory (MB), which
    # it enables (any checkbox the DOP network has on that row is left out). Both go in Advanced / Performance.
    cache_toggle = dop_templates.get("Cache Simulation")
    cache_memory = [pt for pt in row_by_label(dopnet, CACHE_MEMORY) if pt.type() != hou.parmTemplateType.Toggle]
    if not cache_memory:
        warn(f"{dopnet.path()} has no parameter labelled {CACHE_MEMORY!r}; not promoted. "
             f"Its parameters: {labels(dopnet)}")
    for pt in cache_memory:
        links.append((sub.relativePathTo(dopnet), pt.name(), pt.name()))
    if cache_toggle is not None and cache_memory:
        cache_toggle.hideLabel(True)
        cache_toggle.setJoinWithNext(True)
        off = f"{{ {cache_toggle.name()} == 0 }}"
        for pt in cache_memory:
            own = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
            pt.setConditional(hou.parmCondType.DisableWhen, f"{own} {off}".strip())
    cache_items = ([cache_toggle] if cache_toggle is not None else []) + cache_memory
    solver_first = []
    # Solve on Creation Frame (as on RBD Bullet Solver's first tab), linked to the pieces' and the collider's
    # Packed Object.
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
    taken = {pt.name() for pt in top + solver_first + cache_items} | {USE_COLLISIONS, COLLISION_TYPE,
                                                       WORLD_SCALE_TOGGLE, WORLD_SCALE}
    for tab in solver_tabs:
        for pt in value_templates(tab.parmTemplates()):
            links.append((sub.relativePathTo(solver), pt.name(), pt.name()))
            taken.add(pt.name())

    # The collider's Shape, Material and CCD & Contacts folders, every name prefixed (they would clash with the
    # pieces' and the solver's), Default Shape defaulting to Convex Hull.
    collision = dopnet.node("collision")
    collider_tabs = [t for t in data_folders(collision) if t.label() in COLLIDER_FOLDERS]
    if len(collider_tabs) != len(COLLIDER_FOLDERS):
        warn(f"collider folders found: {[t.label() for t in collider_tabs]}, expected {COLLIDER_FOLDERS}")
    # Flip Static Mesh Winding is the collider's Shape setting already; it is dropped here and comes back as
    # COLLISION_FLIP above the folders, so a Static Object (whose Shape folder is off) has it too.
    collider_tabs = drop_templates(collider_tabs, COLLIDER_DROPPED | {"flipwinding"})
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
    static_data = dopnet.node(STATIC_DATA)
    for old, new in collider_renames.items():
        links.append((sub.relativePathTo(collision), old, new))
        taken.add(new)
        # The Static Object's Box3D Data takes the same Material settings (friction, bounce ...).
        if static_data is not None and old in GROUND_MATERIAL:
            links.append((sub.relativePathTo(static_data), old, new))
    if static_data is None:
        warn(f"no {STATIC_DATA} in the DOP network: Collide As Static Object has no material settings")
    source = hou.MenuParmTemplate(
        COLLISION_SOURCE, "Collide As", ("0", "1"),  # numeric tokens: the conditions compare numbers
        menu_labels=("Packed Pieces", "Static Object"), default_value=0,
        help="Packed Pieces: each piece is a body, as the Geometry input. Static Object: the whole input is one "
             "static mesh, or a height field if it has one. Only for colliders that never move.")
    flip = hou.ToggleParmTemplate(
        COLLISION_FLIP, "Flip Mesh Winding", default_value=False,
        help="Static meshes are one-sided. Turn on if pieces fall through from the side that should be solid.")
    # Both colliders: a static concave piece is a mesh too.
    for node in (collision, static_data):
        if node is not None:
            links.append((sub.relativePathTo(node), "flipwinding", COLLISION_FLIP))
    taken.add(COLLISION_FLIP)
    use_collisions = hou.ToggleParmTemplate(
        USE_COLLISIONS, "Use External Collisions", default_value=True,
        help="Collides the pieces with the Collision input. Off: the Collision input is ignored.")
    animated = hou.MenuParmTemplate(
        COLLISION_TYPE, "Collision Type", ("static", "animated", "deforming"),
        menu_labels=("Static", "Animated", "Deforming"), default_value=1,
        help="Static: collision stays where it starts. Animated: it follows its animation and pushes what it hits. "
             "Deforming: Animated, with its shape rebuilt every step (slower). A Static Object only follows "
             "Deforming.")
    animated.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    source.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    flip.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    # One open Collision Geometry folder with Use External Collisions as its header checkbox, laid out as
    # RBD Bullet Solver's: the toggle just before the folder (drawn in its header), the collider's
    # folders simple inside it, each disabled with the toggle off.
    off = f"{{ {USE_COLLISIONS} == 0 }}"
    for f in collider_folders:
        # Shape and Contacts are for packed pieces: off for a Static Object (the groups OR together).
        static_off = f"{off} {{ {COLLISION_SOURCE} == 1 }}" if f.label() in STATIC_COLLIDER_FOLDERS else off
        f.setConditional(hou.parmCondType.DisableWhen, static_off)
        f.setTags({**f.tags(), "group_type": "simple"})
    # Ground Collision below it, as RBD Bullet Solver's: Ground Type in the folder's header.
    ground_folder, ground_links = ground_items(dopnet.node(GROUND_DATA))
    links += [(sub.relativePathTo(dopnet.node(GROUND_DATA)), inner, outer) for _, inner, outer in ground_links]
    collision_items = [use_collisions,
                       collapsible("b3dsop_collision_geometry", "Collision Geometry",
                                   [source, animated, flip] + collider_folders, header_toggle=USE_COLLISIONS),
                       ground_folder]
    # The Constraints tab: every relationship's own settings (b3d_solver_constraints.py).
    constraint_folders, constraint_links = constraint_items(dopnet, sub)
    links += constraint_links
    taken.update(new for _, _, new in constraint_links)
    taken.update([TRANSFER_ATTRIBUTES, KEEP_BROKEN, IMPORT_CONSTRAINTS, IMPORT_PROXY]
                 + list(POINT_VELOCITY_PARMS) + [n for n, _ in TRANSFER_TO.values()])

    group = hou.ParmTemplateGroup()
    for pt in top:
        group.append(pt)
    # The solver's tabs by label, pulled apart by parameter name: Contacts, Sleeping and Aging become folders
    # at the end of the Solver tab, Bounce / Friction goes to Forces, Impact Min Speed and Export
    # Collision Shapes to Output and Visualization. The DOP's own Collision, Aging and Output tabs end up
    # empty and are dropped, so the SOP's Collision tab is only about the Collision input.
    tabs = [(t.label(), list(t.parmTemplates())) for t in solver_tabs]

    def take(names):
        taken_templates = []
        for i, (label, contents) in enumerate(tabs):
            contents, got = pull(contents, lambda pt: pt.name() in names)
            tabs[i] = (label, contents)
            taken_templates += got
        return taken_templates

    folder_items = []
    for label, names, is_open in SOLVER_FOLDERS:
        got = take(names)
        if not got:
            warn(f"none of the {label} settings {names} found on the solver: no {label} folder on the Solver tab")
        else:
            folder_items.append(collapsible(f"b3dsop_{label.lower()}", label, got, open_folder=is_open))
    bounce = take(BOUNCE_PARMS)
    speed = take((IMPACT_SPEED,))
    shapes = take((SHAPES_TOGGLE,))
    tabs = [(label, drop_templates(contents, set())) for label, contents in tabs]  # folders left empty go
    solver_extra = solver_first + folder_items
    # Tab order: Solver, Pieces, Collision, Forces, any other solver tabs, Visualization, then Advanced and
    # Output always last.
    by_label = {label: contents for label, contents in tabs if contents}
    by_label["Forces"] = by_label.get("Forces", []) + bounce
    # Cache Simulation and Cache Memory (MB) lead the Advanced tab's Performance folder.
    by_label["Advanced"] = cache_items + by_label.get("Advanced", [])
    advanced_groups = [(glabel, names + [pt.name() for pt in cache_items] if glabel == "Performance" else names)
                       for glabel, names in TAB_GROUPS["Advanced"]]
    ordered = [("Solver", [use_scale, world_scale] + by_label.pop("Solver", []) + solver_extra),
               ("Pieces", None),  # filled below
               ("Collision", collision_items)]
    if constraint_folders:
        ordered.append(("Constraints", constraint_folders))
    last = [(label, by_label.pop(label)) for label in ("Advanced",) if label in by_label]
    if "Forces" in by_label:
        ordered.append(("Forces", by_label.pop("Forces")))
    ordered += list(by_label.items()) + [("Visualization", None)] + last + [("Output", None)]  # filled below
    print("make_solver_sop: solver tabs: " + ", ".join(t.label() for t in solver_tabs))

    data_tabs = drop_templates(data_folders(pieces), PIECES_DROPPED)
    # Start Asleep (the Packed Object's Initial State) leads the Sleep folder; Add Impact Data (its Object tab)
    # joins as a folder of its own, taken out for the Output tab just below.
    moved = moved_settings(pieces)
    sleep = next((t for t in data_tabs if t.label() == "Sleep"), None)
    if START_ASLEEP in moved and sleep is not None:
        sleep.setParmTemplates([moved[START_ASLEEP]] + list(sleep.parmTemplates()))
    elif START_ASLEEP in moved:
        warn("no Sleep folder on the pieces: Start Asleep is not on the Pieces tab")
    if IMPACT_TOGGLE in moved:
        data_tabs.append(hou.FolderParmTemplate("b3dsop_impact_tmp", "Impact", [moved[IMPACT_TOGGLE]]))
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
    # The Pieces tab: the Box3D Data folders (Shape, Body, Material, CCD & Contacts, Sleep), each an open
    # collapsible folder (the folder Add Impact Data came out of may be left empty: dropped).
    pieces_items = [as_collapsible(f) for f in folders if f.parmTemplates()]
    # Initial State (the Packed Object's Velocity, Angular Velocity and Inherit Velocity) before Sleep.
    initial = initial_state_folder(pieces)
    if initial is not None:
        at = next((i for i, f in enumerate(pieces_items) if f.label() == "Sleep"), len(pieces_items))
        pieces_items.insert(at, initial)
        links += [(sub.relativePathTo(pieces), name, name) for name in INITIAL_STATE]
    velocity = velocity_templates(sub.node("transform_geometry"))
    output_items = output_tab(impact, speed, velocity)
    vis_items = visualization_tab(shapes)
    for i, (label, contents) in enumerate(ordered):
        contents = {"Pieces": pieces_items, "Output": output_items, "Visualization": vis_items,
                    "Collision": contents, "Constraints": contents}.get(
            label, group_tab(label, contents, advanced_groups if label == "Advanced" else None)
            if contents is not None else contents)
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
    seen, clashes = set(), set()
    for pt in group.entriesWithoutFolders():
        (clashes if pt.name() in seen else seen).add(pt.name())
    if clashes:
        warn(f"parameter names used twice on the asset: {sorted(clashes)}")
    definition.setParmTemplateGroup(group)
    definition.setMaxNumOutputs(4)
    definition.setIcon(ICON)
    b3d_hda.set_tab_submenu(definition, warn)
    definition.setComment("Simulates packed pieces with Box3D, like RBD Bullet Solver: geometry, "
                          "constraints, proxy and collision in; geometry, constraints, proxy and "
                          "simulation points out.")
    # Diving into the asset lands in FORCES (the DiveTarget section holds the relative path), and
    # FORCES is an editable node, so forces can be added without unlocking the asset (the
    # EditableNodes section: space-separated relative paths).
    definition.addSection("Help", HELP_CARD)
    definition.addSection("DiveTarget", DIVE_TARGET)
    definition.addSection("EditableNodes", DIVE_TARGET)

    asset.allowEditingOfContents()
    for inner_path, inner_name, outer_name in links:
        link(asset.node(inner_path), inner_name, asset, outer_name)
    finish_ground(asset)
    finish_constraints(asset, constraint_links)
    scale_settings(asset)
    scale_joint_settings(asset)
    # Set Always, not Set Initial: the solver's settings are read every step, so an animated one (Gravity keyed
    # or driven by an expression) changes the simulation while it plays.
    b3d_hda.set_operations(asset.node("dopnet/box3d_solver"), "always", warn)
    definition.updateFromNode(asset)
    set_guide_geometry(definition)
    definition.save(out_path, asset)
    # Labels last, saved without the node so nothing regenerates the DialogScript over them.
    edit_dialog_script(definition)
    definition.save(out_path)

    check(out_path)
    print(f"Box3D Solver SOP asset: {os.path.normpath(out_path)}")


def set_guide_geometry(definition):
    """The Node tab's Guide Geometry: `guide=GUIDE_GEO` in the definition's extra info (as RBD Bullet
    Solver's), keeping any other entries."""
    kept = [t for t in definition.extraInfo().split() if not t.startswith("guide=")]
    definition.setExtraInfo(" ".join(kept + [f"guide={GUIDE_GEO}"]))


def velocity_templates(xform):
    """Transform Pieces' own Point Velocities parameters (POINT_VELOCITY_PARMS), copied from `xform`."""
    found = []
    group = xform.parmTemplateGroup() if xform is not None else None
    for name in POINT_VELOCITY_PARMS:
        pt = group.find(name) if group is not None else None
        if pt is None:
            warn(f"no {name} on the inner Transform Pieces: not on the Output tab")
        else:
            found.append(pt.clone())
    return found


def output_tab(impact, speed, velocity):
    """The Output tab's contents: Impact Data (a closed collapsible folder, `impact` its header toggle,
    then `speed` and Fetch Impacts), Attribute Transfer (with `velocity`, Transform Pieces' Point
    Velocities), Outputs (Import Constraints / Proxy), Constraints."""
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
        items.append(collapsible("b3dsop_impacts", "Impact Data", contents, open_folder=False,
                                 header_toggle=toggle.name()))
    transfer = hou.StringParmTemplate(
        TRANSFER_ATTRIBUTES, "Transfer Attributes", 1, default_value=(TRANSFER_ATTRIBUTES_DEFAULT,),
        help="Point attributes copied from the simulation onto Simulation Points, and onto the pieces as ticked below.")
    to = [hou.ToggleParmTemplate(name, label, default_value=True,
                                 help=f"Copies Transfer Attributes onto the {label[12:]} output. Off: only w.")
          for name, label in TRANSFER_TO.values()]
    items.append(collapsible("b3dsop_transfer", "Attribute Transfer", [transfer] + to + list(velocity)))
    imports = [hou.ToggleParmTemplate(IMPORT_CONSTRAINTS, "Import Constraints", default_value=True,
                                      help="Outputs the simulated constraints. Off: an empty output, which is faster."),
               hou.ToggleParmTemplate(IMPORT_PROXY, "Import Proxy", default_value=True,
                                      help="Outputs the simulated proxy. Off: an empty output, which is faster.")]
    items.append(collapsible("b3dsop_outputs", "Outputs", imports))
    keep = hou.ToggleParmTemplate(KEEP_BROKEN, "Export Broken Constraints", default_value=False,
                                  help="Keeps broken constraints in the Constraint Geometry output. Off: they are removed.")
    keep.setConditional(hou.parmCondType.DisableWhen, f"{{ {IMPORT_CONSTRAINTS} == 0 }}")
    items.append(collapsible("b3dsop_constraints", "Constraints", [keep]))
    return items


def visualization_tab(shapes):
    """The Visualization tab's contents: Show Collision Geometry, the solver's Export Collision Shapes as
    Show Collision Shapes, and Shape Color, all at the tab's top level."""
    if not shapes:
        warn(f"no {SHAPES_TOGGLE} on the solver: Show Collision Shapes is not on the Visualization tab")
        return []
    show = shapes[0].clone()
    show.setLabel("Show Collision Shapes")
    show.setHelp("Draws every body's collision shapes in the viewport. Costs time every step.")
    color = hou.FloatParmTemplate(
        SHAPES_COLOR, "Shape Color", 3, default_value=(0.2, 0.8, 1.0), min=0.0, max=1.0,
        naming_scheme=hou.parmNamingScheme.RGBA, look=hou.parmLook.ColorSquare,
        help="Colour of the collision shapes.")
    color.setConditional(hou.parmCondType.DisableWhen, f"{{ {show.name()} == 0 }}")
    # The external collision geometry, as RBD Bullet Solver's Collision Geometry > Show Geometry (on).
    geo = hou.ToggleParmTemplate(
        SHOW_COLLISION, "Show Collision Geometry", default_value=True,
        help="Draws the external collision geometry in the viewport.")
    geo.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_COLLISIONS} == 0 }}")
    return [geo, show, color, ground_visualization()]


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
    if probe.node(GUIDE_GEO) is None:
        warn(f"no {GUIDE_GEO} node inside the asset")
    if f"guide={GUIDE_GEO}" not in probe.type().definition().extraInfo().split():
        warn(f"the asset's extra info is {probe.type().definition().extraInfo()!r}: Guide Geometry is not {GUIDE_GEO}")
    if probe.parm(SHAPES_TOGGLE) is None or probe.parm(SHAPES_COLOR + "r") is None:
        warn(f"no {SHAPES_TOGGLE} / {SHAPES_COLOR} parameters on the asset")
    else:
        inner = probe.node("dopnet/box3d_solver")
        probe.parm(SHAPES_TOGGLE).set(1)
        if inner is None or inner.parm(SHAPES_TOGGLE) is None or inner.parm(SHAPES_TOGGLE).eval() != 1:
            warn(f"dopnet/box3d_solver/{SHAPES_TOGGLE} does not follow the asset's")
        probe.parm(SHAPES_TOGGLE).set(0)
    if not row_by_label(probe, CACHE_MEMORY):
        warn(f"no {CACHE_MEMORY!r} parameter on the asset")
    if probe.parm(SHOW_COLLISION) is None or probe.parm(SHOW_COLLISION).eval() != 1:
        warn(f"no {SHOW_COLLISION} parameter on the asset, or it is not on by default")
    for outer, inner_path, inner_name in (("friction", "dopnet/pieces", "friction"),
                                          ("collision_friction", "dopnet/collision", "friction"),
                                          ("weld_torque_threshold", "dopnet/weld", "torque_threshold"),
                                          ("collision_friction", "dopnet/collision_static_data", "friction"),
                                          ("collision_bounce", "dopnet/collision_static_data", "bounce"),
                                          ("substeps", "dopnet/box3d_solver", "substeps")):
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
        # Joint settings: read at World Scale 2 against scale 1 (a length doubles, a torque is four
        # times, an angular speed stays; Prismatic's Motor Speed is a length per second).
        joints = (("distance", "max_length", 2.0), ("weld", "torque_threshold", 4.0),
                  ("revolute", "motor_speed", 1.0), ("prismatic", "motor_speed", 2.0), ("glue", "strength", 2.0))
        at2 = {}
        for name, parm, _ in joints:
            q = probe.parm(f"dopnet/{name}/{parm}") if probe.node(f"dopnet/{name}") else None
            at2[(name, parm)] = q.eval() if q is not None else None
        probe.parm(WORLD_SCALE_TOGGLE).set(0)
        for name, parm, ratio in joints:
            q = probe.parm(f"dopnet/{name}/{parm}") if probe.node(f"dopnet/{name}") else None
            if q is None or at2[(name, parm)] is None:
                warn(f"World Scale: cannot check dopnet/{name}/{parm}")
            elif abs(at2[(name, parm)] - q.eval() * ratio) > 1e-6 * max(1.0, abs(q.eval())):
                warn(f"World Scale 2: dopnet/{name}/{parm} reads {at2[(name, parm)]}, expected {q.eval() * ratio}")
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
    for parm in (KEEP_BROKEN,):
        if probe.parm(parm) is None:
            warn(f"no {parm} parameter on the asset")
    for path in ("switchif_beforestart", "switchif_keepbroken"):
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
