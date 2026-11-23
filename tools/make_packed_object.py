"""Builds the Box3D Packed Object digital asset (a DOP) with hython.

    hython tools/make_packed_object.py <output .hda>

The build runs this after compiling the plugin (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_packedobject.hda next to the plugin's dso folder. The plugin must already be
installed: the asset contains a Box3D Data node, and its parameters are copied from it.

What it is: RBD Packed Object for Box3D. Like it, a SOP stage inside the object
(create_attributes, as RBD Packed Object's create_or_overwrite_attributes) prepares the SOP
geometry when the object is created:

    Empty Object -> SOP Geometry (create_attributes/OUT) -> Box3D Data -> visualize_hires
                    ^ Rendering Parameters Volatile          -> Collider Label (output)
    create_attributes: Object Merge of the SOP Path -> setup_defaults -> create_name
                       -> [creation frame] set_rest -> initial_transform -> apply_initial_xform -> setup_velocities -> setup_deforming -> OUT

setup_velocities writes Velocity / Angular Velocity as v / w on the creation frame, unless Inherit
Velocity keeps the points' own (off by default, as RBD Packed Object's; the Box3D Solver SOP turns it on
for its pieces so Box3D Configure's Initial State reaches the simulation). Deforming writes the deforming
attribute on every import and re-imports the SOP every step, as RBD Packed Object's Create Deforming Objects, so the solver rebuilds the shapes.

setup_defaults writes each per-body setting whose value differs from Box3D Data's default as a
point attribute (b3d_settings.packed_defaults_vex), unless the points already have it, so a point
attribute still wins and untouched settings write nothing. set_rest resets the rest and bakes
orient / pscale / scale, on the creation frame only (b3d_settings.REST_VEX; stamped $SF / $OBJCF,
as RBD Packed Object). The inner Box3D Data holds only the object-level settings
(b3d_settings.SKIP), channel references to the asset's; its other values stay at the defaults, so
a missing attribute means the default. Settings change mid-simulation through the attributes (a
SOP Solver, POP Wrangle), not through the asset's parameters.

Display Geometry, as RBD Packed Object: Simulation shows the object's geometry (Rendering Parameters
Volatile); Render hides it and the SOP Guide visualize_hires draws Render SOP Path instead, its pieces
moved by the simulated points with the same name (Transform Pieces, once per object the asset made).
Collider Label PackedObject, as RBD Packed Object: other solvers collide with the object as packed
pieces, and the Box3D Solver keeps it one body per point.

Its parameters: the Object tab (object name, creation frame, solve on creation frame, Geometry Source, SOP path,
object transform, overwrite from SOP, deforming, Add Impact Data, display geometry), Initial State (with
Start Asleep), then every Box3D Data folder as a tab. The asset is listed under Box3D in the Tab menu.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import sys
import traceback

import hou

# No __pycache__ next to the scripts (they are imported from this folder).
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b3d_hda  # noqa: E402  (beside this script)
import b3d_settings  # noqa: E402
from b3d_hda import labels, pull, value_templates, without_parmops  # noqa: E402

TYPE_NAME = "box3d_packedobject"
LABEL = "Box3D Packed Object"
ICON = "DOP_box3d_packedobject"  # config/Icons/DOP_box3d_packedobject.svg, installed by the build

# Parameters promoted from the inner Empty Object and SOP Geometry, under their own names. Required ones
# fail the build of the asset; the rest are skipped with a warning when this Houdini version lacks them.
OBJECT_NAME, SOP_PATH, USE_TRANSFORM = "objname", "soppath", "usetransform"
OBJECT_PARMS = [
    ("emptyobject", OBJECT_NAME, True),
    ("emptyobject", "solvefirstframe", False),
    ("geometry", SOP_PATH, True),
    ("geometry", USE_TRANSFORM, False),
]

OVERWRITE_FROM_SOP = "overwritefromsop"  # RBD Packed Object's Overwrite Attributes from SOP
OVERWRITE_ATTRIBS = "overwriteattributes"  # the attributes it copies from the SOP each step (RBD Packed Object's)
OVERWRITE_DEFAULT = "active animated deforming"
DEFORMING = "deforming"  # the pieces' shapes follow the SOP (RBD's Create Deforming Objects)
# Initial Object Type, as RBD Packed Object's: one menu for Body Type and Deforming. Custom leaves them to their own
# parameters. The stage reads the two effective (hidden) parameters, which follow the menu.
INITIAL_TYPE = "initialstate"
INITIAL_ITEMS = ("active", "static", "animatedstatic", "deformingstatic", "deformingactive", "custom")
INITIAL_LABELS = ("Create Active Objects", "Create Static Objects", "Create Animated Static Objects",
                  "Create Deforming Static Objects", "Create Deforming Active Objects", "Custom")
EFFECTIVE_BODY, EFFECTIVE_DEFORM = "effectivebodytype", "effectivedeforming"
# Geometry Source, as RBD Packed Object's: the SOP Path, or the DOP network's first to fourth input. The stage reads
# the hidden path this gives (EFFECTIVE_SOP).
GEO_SOURCE, EFFECTIVE_SOP = "geosource", "effectivesoppath"
GEO_SOURCES = (("sop", "SOP"), ("first", "First Context Geometry"), ("second", "Second Context Geometry"),
               ("third", "Third Context Geometry"), ("fourth", "Fourth Context Geometry"))
# Body Type's menu is Static 0, Kinematic 1, Dynamic 2: Active and Deforming Active are Dynamic, Animated Static is
# Kinematic, the others Static.
BODY_FROM_TYPE = (f'if(ch("{INITIAL_TYPE}") == 5, ch("bodytype"), '
                  f'if(ch("{INITIAL_TYPE}") == 0 || ch("{INITIAL_TYPE}") == 4, 2, if(ch("{INITIAL_TYPE}") == 2, 1, 0)))')
DEFORM_FROM_TYPE = (f'if(ch("{INITIAL_TYPE}") == 5, ch("{DEFORMING}"), '
                    f'ch("{INITIAL_TYPE}") == 3 || ch("{INITIAL_TYPE}") == 4)')
# Hidden, set by the Solver SOP for an animated collider: the whole SOP is read again every step, as with Deforming
# but without writing the deforming attribute.
REIMPORT_SOP = "reimportsop"
# The re-import condition of the inner SOP Geometry: Overwrite Attributes from SOP, or Deforming.
REIMPORT = f'ch("../{OVERWRITE_FROM_SOP}") || ch("../{EFFECTIVE_DEFORM}") || ch("../{REIMPORT_SOP}")'

# Initial State, as RBD Packed Object's (same names).
POS, ROT = "t", "r"
INHERIT_PIVOT, PIVOT = "inheritpivot", "p"
VEL, ANGVEL, INHERIT_VEL = "vel", "angvel", "inheritvelocity"

# setup_velocities, on the creation frame: Velocity and Angular Velocity (degrees per second, written
# as radians) as v and w, unless the points' own are inherited.
VELOCITY_VEX = """// The starting velocity, unless the points' own v / w are kept. Detail wrangle: an @-binding
// would add v and w to the points even when nothing is written.
if (!chi("../../%s")) {
    vector v = chv("../../%s");
    vector w = radians(chv("../../%s"));
    if (!haspointattrib(0, "v")) addpointattrib(0, "v", {0, 0, 0});
    if (!haspointattrib(0, "w")) addpointattrib(0, "w", {0, 0, 0});
    for (int pt = 0; pt < npoints(0); pt++) {
        setpointattrib(0, "v", pt, v);
        setpointattrib(0, "w", pt, w);
    }
}""" % (INHERIT_VEL, VEL, ANGVEL)

# initial_transform, on the creation frame after set_rest: Position and Rotation as a transform about the piece's
# pivot, for the Transform by Attribute after it (as RBD Packed Object's set_rest_and_initial_xform). The pivot is the
# point's own, else its position while Inherit Pivot is on, else Pivot.
INITIAL_XFORM_VEX = """int pts[] = primpoints(0, @primnum);
if (len(pts) != 1) return;
vector pivot = haspointattrib(0, "pivot") ? point(0, "pivot", pts[0])
             : (chi("../../%s") ? point(0, "P", pts[0]) : chv("../../%s"));
4@__initialxform = maketransform(XFORM_SRT, XFORM_XYZ, chv("../../%s"), chv("../../%s"), {1, 1, 1}, pivot);
""" % (INHERIT_PIVOT, PIVOT, POS, ROT)

# setup_deforming, on every import: the deforming attribute, so the solver rebuilds the shapes.
DEFORMING_VEX = """if (chi("../../%s")) {
    if (!haspointattrib(0, "deforming")) addpointattrib(0, "deforming", 0);
    for (int pt = 0; pt < npoints(0); pt++)
        setpointattrib(0, "deforming", pt, 1);
}""" % EFFECTIVE_DEFORM

STAGE = "create_attributes"  # the SOP stage (a DOP SOP network), as RBD Packed Object's
HIRES = "visualize_hires"  # the SOP Guide drawing the render geometry, as RBD Packed Object's

# Box3D Data's Add Impact Data, which the asset shows on the Shape tab, as RBD Packed Object does with its shape
# settings (Box3D Data has it under CCD & Contacts).
IMPACT_TOGGLE = "addimpact"

# Display Geometry / Render SOP Path, as RBD Packed Object's (same names, so its expressions carry over).
DISPLAY, DISPLAY_GEO, RENDER_PATH = "display", "displaygeo", "rendergeopath"


def initial_state_templates():
    """The Initial State tab, as RBD Packed Object's: Position and Rotation (the whole object, moved before it is
    created), Pivot, Velocity and Angular Velocity."""
    inherit = f"{{ {INHERIT_VEL} == 1 }}"
    return [
        hou.FloatParmTemplate(POS, "Position", 3, default_value=(0, 0, 0),
                              help="Moves every piece by this much when the object is created."),
        hou.FloatParmTemplate(ROT, "Rotation", 3, default_value=(0, 0, 0),
                              help="Turns every piece about its pivot, in degrees, when the object is created."),
        hou.ToggleParmTemplate(INHERIT_PIVOT, "Inherit Pivot from Point Position", default_value=True,
                               help="Rotation turns each piece about its own position. Off: about Pivot."),
        hou.FloatParmTemplate(PIVOT, "Pivot", 3, default_value=(0, 0, 0), disable_when=f"{{ {INHERIT_PIVOT} == 1 }}",
                              help="The point Rotation turns the pieces about, when not inherited."),
        hou.FloatParmTemplate(VEL, "Velocity", 3, default_value=(0, 0, 0), disable_when=inherit,
                              help="The pieces' starting velocity, in units per second."),
        hou.FloatParmTemplate(ANGVEL, "Angular Velocity", 3, default_value=(0, 0, 0), disable_when=inherit,
                              help="The pieces' starting spin, in degrees per second about each world axis."),
        hou.ToggleParmTemplate(INHERIT_VEL, "Inherit Velocity from Point Velocity", default_value=False,
                               help="Keeps the points' own v and w (Box3D Configure's Initial State). Off: "
                                    "Velocity and Angular Velocity replace them."),
    ]


def display_templates():
    """Display Geometry (toggle and Simulation / Render menu on one row) and Render SOP Path."""
    toggle = hou.ToggleParmTemplate(DISPLAY, "Display Geometry", default_value=True, join_with_next=True)
    toggle.hideLabel(True)
    menu = hou.MenuParmTemplate(
        DISPLAY_GEO, "Display Geometry", ("simgeo", "rendergeo"), ("Simulation Geometry", "Render Geometry"),
        default_value=0, disable_when=f"{{ {DISPLAY} == 0 }}",
        help="Render Geometry draws the Render SOP Path's pieces moved with the simulated ones, matched by name.")
    path = hou.StringParmTemplate(
        RENDER_PATH, "Render SOP Path", 1, default_value=("",), string_type=hou.stringParmType.NodeReference,
        disable_when=f"{{ {DISPLAY} == 0 }}", help="Hi-res geometry to draw in place of the simulated pieces.")
    path.setConditional(hou.parmCondType.HideWhen, f"{{ {DISPLAY_GEO} != rendergeo }}")
    path.setTags({"opfilter": "!!SOP!!", "oprelative": "."})
    return [toggle, menu, path]


def build_hires(guide):
    """The SOP Guide's network, as RBD Packed Object's visualize_hires: for each object this asset made
    (a For-Each over createdObjects(), the iteration from the metadata Block Begin), Transform Pieces
    moves the Render SOP Path's pieces by the matching simulated points, matched by name. The
    references to the asset's parameters are set by wire_hires."""
    for child in guide.children():
        child.destroy()
    meta = guide.createNode("block_begin", "repeat_begin1_metadata1")
    begin = guide.createNode("block_begin", "repeat_begin1")
    fetch = guide.createNode("object_merge", "fetch_hires_geo")
    points = guide.createNode("dopimport", "fetch_transforms")
    pieces = guide.createNode("xformpieces", "transformpieces1")
    end = guide.createNode("block_end", "repeat_end1")
    pieces.setInput(0, fetch)
    pieces.setInput(1, points)
    end.setInput(0, pieces)
    py = hou.exprLanguage.Python
    set_parm(meta, "method", "metadata")
    set_parm(meta, "blockpath", "../repeat_end1")
    set_parm(begin, "blockpath", "../repeat_end1")
    set_parm(end, "itermethod", "count")
    set_parm(end, "method", "merge")
    set_parm(end, "blockpath", "../repeat_begin1")
    set_parm(end, "iterations", expr='len(hou.node("../..").createdObjects())', lang=py)
    end.setComment("Merge the hi-res geometry for each created DOP object.")
    set_parm(points, "doppath", expr='hou.node("..").simulation().dopNetNode().path()', lang=py)
    set_parm(points, "objpattern", lang=py, expr=(
        'node = hou.node("../..")\n'
        'iteration = hou.node("../repeat_begin1_metadata1").geometry().intAttribValue("iteration")\n'
        "\n"
        "return node.createdObjects()[iteration].objid()"))
    set_parm(points, "importstyle", "points")
    set_parm(points, "donotsim", 1)
    end.setDisplayFlag(True)
    end.setRenderFlag(True)
    b3d_hda.tidy(guide)


def wire_hires(asset, transform_parm):
    """Display Geometry's references: Rendering Parameters Volatile shows the simulation geometry, the
    SOP Guide the render geometry, and the guide's Object Merge reads Render SOP Path (Use Object
    Transform as the stage's)."""
    if asset.node(HIRES) is None:
        return  # not built (warned)
    show = f'ch("../{DISPLAY}") && ch("../{DISPLAY_GEO}") == '
    set_parm(asset.node("renderparmsvolatile"), "display", expr=show + "0")
    guide = asset.node(HIRES)
    set_parm(guide, "showguide", expr=show + "1")
    fetch = guide.node("fetch_hires_geo")
    set_parm(fetch, "objpath1", f'`chsop("../../{RENDER_PATH}")`')
    if transform_parm:
        set_parm(fetch, "xformtype", expr=f'ch("../../{transform_parm}")')



warn = b3d_hda.warner("make_packed_object")


def link(inner, outer, name):
    """Makes every component of inner's parameter tuple `name` a channel reference to outer's."""
    pt = inner.parmTuple(name)
    if pt is None:
        warn(f"{inner.path()} has no parameter {name}")
        return
    for p in pt:
        target = outer.parm(p.name())
        if target is None:
            warn(f"{outer.path()} has no parameter {p.name()}")
            continue
        p.set(target)  # a hou.Parm value: a channel reference


def set_parm(node, name, value=None, expr=None, lang=hou.exprLanguage.Hscript):
    """Sets node's parameter `name` to `value`, or to the expression `expr`; warns when it has none."""
    parm = node.parm(name)
    if parm is None:
        warn(f"{node.path()} has no parameter {name}; its parameters: {labels(node)}")
        return False
    if expr is not None:
        parm.setExpression(expr, lang)
    else:
        parm.set(value)
    return True


def edit_template(entries, name, change):
    """`entries` with `change(template)` applied to the template called `name`, folders included (a copy)."""
    out = []
    for pt in entries:
        pt = pt.clone()
        if pt.type() == hou.parmTemplateType.Folder:
            pt.setParmTemplates(edit_template(pt.parmTemplates(), name, change))
        elif pt.name() == name:
            change(pt)
        out.append(pt)
    return out


def build_stage(sub, rows):
    """The SOP stage inside the asset (STAGE), as RBD Packed Object's create_or_overwrite_attributes:
    Object Merge of the asset's SOP Path -> setup_defaults -> create_name -> set_rest on the creation
    frame only, with setup_velocities (switch_first_frame on the stamped $SF / $OBJCF) -> setup_deforming
    -> OUT. The references to the asset's parameters are set by wire_stage once the asset has them.
    Returns OUT."""
    net = sub.createNode("sopnet", STAGE)
    merge = net.createNode("object_merge", "object_merge")
    defaults = net.createNode("attribwrangle", "setup_defaults")
    defaults.setInput(0, merge)
    defaults.parm("class").set("detail")
    defaults.parm("snippet").set(b3d_settings.packed_defaults_vex(rows, "../..", {"bodytype": EFFECTIVE_BODY}))
    name = net.createNode("attribwrangle", "create_name")
    name.setInput(0, defaults)
    name.addSpareParmTuple(hou.StringParmTemplate("objname", "Object Name", 1))
    name.parm("snippet").set(b3d_settings.NAME_VEX)
    rest = net.createNode("attribwrangle", "set_rest")
    rest.setInput(0, name)
    rest.parm("snippet").set(b3d_settings.REST_VEX)
    # Initial State's Position and Rotation, per piece about its pivot, after the rest is recorded.
    place = net.createNode("attribwrangle", "initial_transform")
    place.setInput(0, rest)
    place.parm("class").set("primitive")
    place.parm("snippet").set(INITIAL_XFORM_VEX)
    apply = net.createNode("xformbyattrib", "apply_initial_xform")
    apply.setInput(0, place)
    apply.parm("xformattrib").set("__initialxform")
    velocities = net.createNode("attribwrangle", "setup_velocities")
    velocities.setInput(0, apply)
    velocities.parm("class").set("detail")
    velocities.parm("snippet").set(VELOCITY_VEX)
    stamps = net.createNode("null", "STAMPS")
    for parm, key in (("frame", "SF"), ("objcf", "OBJCF"), ("objid", "OBJID")):
        stamps.addSpareParmTuple(hou.FloatParmTemplate(parm, key, 1))
        stamps.parm(parm).setExpression(f'stamp("..", "{key}", -1)', hou.exprLanguage.Hscript)
    # After the creation frame: the object's own simulated geometry (SIM_GEO) with the listed attributes copied from
    # the SOP by point name (Overwrite Attributes from SOP, as RBD Packed Object's copy_attribs), unless the whole SOP
    # is read again (Deforming, or an animated collider).
    sim = net.createNode("object_merge", "SIM_GEO")
    copy = net.createNode("attribcopy", "copy_attribs")
    copy.setInput(0, sim)
    copy.setInput(1, name)
    copy.parm("matchbyattribute").set(1)
    copy.parm("attributetomatch").set("name")
    copy.parm("class").set(3)  # points
    later = net.createNode("switch", "switch_reimport")
    later.setInput(0, copy)
    later.setInput(1, name)
    first = net.createNode("switch", "switch_first_frame")
    first.setInput(0, later)
    first.setInput(1, velocities)
    first.parm("input").setExpression('ch("../STAMPS/frame") == ch("../STAMPS/objcf")',
                                      hou.exprLanguage.Hscript)
    deforming = net.createNode("attribwrangle", "setup_deforming")
    deforming.setInput(0, first)
    deforming.parm("class").set("detail")
    deforming.parm("snippet").set(DEFORMING_VEX)
    out = net.createNode("null", "OUT")
    out.setInput(0, deforming)
    out.setDisplayFlag(True)
    out.setRenderFlag(True)
    b3d_hda.tidy(net)
    return out


def wire_stage(asset, sop_path_parm, transform_parm, objname_parm):
    """The stage's references to the asset's parameters: the Object Merge's path and transform (Use
    Object Transform: Into This Object, the DOP network's space, so world space), and the name
    prefix."""
    merge = asset.node(f"{STAGE}/object_merge")
    merge.parm("objpath1").set(f'`chsop("../../{sop_path_parm}")`')  # the effective path
    if transform_parm and merge.parm("xformtype") is not None:
        merge.parm("xformtype").setExpression(f'ch("../../{transform_parm}")', hou.exprLanguage.Hscript)
    elif transform_parm:
        warn(f"{merge.path()} has no Transform (xformtype): Use Object Transform does nothing")
    if transform_parm:
        asset.node("source_transform").parm("valvaluef0").setExpression(f'ch("../{transform_parm}")',
                                                                         hou.exprLanguage.Hscript)
    asset.node("save_source_geo").parm("objpath").setExpression(f'chsop("../{sop_path_parm}")', hou.exprLanguage.Hscript)
    sim = asset.node(f"{STAGE}/SIM_GEO")
    sim.parm("objpath1").set('../../..:`ch("../STAMPS/objid")`')
    asset.node(f"{STAGE}/copy_attribs").parm("attribname").set(f'`chs("../../{OVERWRITE_ATTRIBS}")`')
    asset.node(f"{STAGE}/switch_reimport").parm("input").setExpression(
        f'ch("../../{EFFECTIVE_DEFORM}") || ch("../../{REIMPORT_SOP}")', hou.exprLanguage.Hscript)
    asset.node(f"{STAGE}/create_name").parm("objname").setExpression(f'chs("../../{objname_parm}")',
                                                                       hou.exprLanguage.Hscript)


def stamp_sop_geometry(geo):
    """SOP Geometry reads the stage, stamping $SF and $OBJCF for its creation-frame switch, as RBD
    Packed Object's geometry node. Its own Use Object Transform stays off: the stage has no object
    (Houdini 22's SOP Geometry has no OBJ Path to name one), and the Object Merge applies the
    transform."""
    if geo.parm("soppath") is None:
        warn(f"{geo.path()} has no soppath; its parameters: {labels(geo)}")
    else:
        geo.parm("soppath").set(f"../{STAGE}/OUT")
    if geo.parm("usetransform") is not None:
        geo.parm("usetransform").set(0)
    if geo.parm("numstamps") is None:
        warn(f"{geo.path()} has no Number of Stamps: the rest is reset on every import")
        return
    geo.parm("numstamps").set(3)
    for i, key in enumerate(("SF", "OBJCF", "OBJID"), start=1):
        geo.parm(f"stampname{i}").set(key)
        geo.parm(f"stampvalue{i}").setExpression(f"${key}", hou.exprLanguage.Hscript)


def wire_overwrite(asset):
    """Every Parameter Operation of the inner SOP Geometry (and its Default Operation): Set Always
    while Overwrite Attributes from SOP or Deforming is on, else Set Initial."""
    geo = asset.node("geometry")
    ops = [p for p in geo.parms() if p.name().startswith("parmop") or p.name() == "defaultparmop"]
    if not ops:
        warn(f"{geo.path()} has no Parameter Operation menus: Overwrite Attributes from SOP does nothing")
    for p in ops:
        b3d_hda.menu_token_expr(p, REIMPORT, "always", "initial", warn)


def add_display(sub, geo, data):
    """After Box3D Data, as RBD Packed Object: Rendering Parameters Volatile on the SOP Geometry (shown
    with Display Geometry: Simulation), the SOP Guide drawing the render geometry (Render), then the
    Collider Label, which is the asset's output. If any of it cannot be made (another Houdini version),
    Box3D Data stays the output and Display Geometry does nothing."""
    data.setDisplayFlag(True)  # the asset's output until the Collider Label is made
    try:
        renderparms = sub.createNode("renderparmsvolatile", "renderparmsvolatile")
        geo.setInput(1, renderparms)
        guide = sub.createNode("sopguide", HIRES)
        guide.setInput(0, data)
        set_parm(guide, "wireframe", 0)
        build_hires(guide)
        # PackedObject: other solvers (FLIP, Vellum, Pyro) collide with it as packed pieces, and the
        # Box3D Solver keeps syncing it per point (any other label makes one body, see isWholeObject).
        label = sub.createNode("colliderlabel", "colliderlabel")
        label.setInput(0, guide)
        set_parm(label, "colliderlabel", "PackedObject")
        label.setDisplayFlag(True)
    except hou.Error:
        warn("Display Geometry / Collider Label not built:\n" + traceback.format_exc())
        for name in ("renderparmsvolatile", HIRES, "colliderlabel"):
            if sub.node(name) is not None:
                sub.node(name).destroy()
        data.setDisplayFlag(True)


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "build")
    sub = dopnet.createNode("subnet", TYPE_NAME)

    obj = sub.createNode("emptyobject", "emptyobject")
    geo = b3d_hda.create(sub, ["sopgeo", "sopgeometry"], "geometry")
    geo.setInput(0, obj)
    # The SOP's path as the object's SourceObject data, on the creation frame, as RBD Packed Object's save_source_geo.
    source_link = sub.createNode("linktosourceobject", "save_source_geo")
    # ... with whether Use Object Transform is on, for the solver's read of the SOP's object transform.
    source_flag = sub.createNode("modifydata", "source_transform")
    source_flag.setInput(0, source_link)
    set_parm(source_flag, "dataname", "SourceObject")
    set_parm(source_flag, "nummods", 1)
    set_parm(source_flag, "valname0", "usetransform")
    set_parm(source_flag, "valtype0", "float")
    attach = sub.createNode("applydata", "applydata1")
    attach.setInput(0, geo)
    attach.setInput(1, source_flag)
    attach.parm("activation").setExpression("$SF == $OBJCF", hou.exprLanguage.Hscript)
    data = sub.createNode("box3d_data", "box3d_data")
    data.setInput(0, attach)
    add_display(sub, geo, data)

    # The parameters: the Object tab, then each Box3D Data folder as a tab.
    inner = {"emptyobject": obj, "geometry": geo}
    promoted = []  # (inner node, template)
    for node_name, name, required in OBJECT_PARMS:
        pt = inner[node_name].parmTemplateGroup().find(name)
        if pt is None:
            msg = f"{inner[node_name].path()} has no parameter {name}. Its parameters: {labels(inner[node_name])}"
            if required:
                raise RuntimeError(msg)
            warn(msg + "; not promoted")
            continue
        pt = pt.clone()
        if name == OBJECT_NAME and pt.type() == hou.parmTemplateType.String:
            pt.setDefaultValue(("$OS",))
        promoted.append((inner[node_name], pt))
    transform_parm = USE_TRANSFORM if any(pt.name() == USE_TRANSFORM for _, pt in promoted) else None

    # Creation Frame: Empty Object has none; like RBD Packed Object, the object is created on the
    # step its Activation is true, here only on that simulation frame.
    # Defaults to the DOP network's Start Frame, as RBD Packed Object's.
    create_frame = hou.FloatParmTemplate(
        "createframe", "Creation Frame", 1, default_value=(1.0,),
        default_expression=('ch(opcreator(".")+"/startframe")',),
        default_expression_language=(hou.scriptLanguage.Hscript,),
        help="The frame on which the object is created.")
    sim_frame = hou.ToggleParmTemplate(
        "usesimframe", "Creation Frame Specifies Simulation Frame", default_value=False,
        help="On: Creation Frame counts simulation frames ($SF, from the Start Frame). Off: it is the scene frame.")
    obj.parm("activation").setExpression('if(ch("../usesimframe"), $SF, $FF) == ch("../createframe")',
                                         hou.exprLanguage.Hscript)

    # Overwrite Attributes from SOP, as RBD Packed Object's: SOP Geometry imports the SOP once on the
    # creation frame (Set Initial) or every step (Set Always), so an animated or deforming SOP reaches
    # the simulation. Wired to the inner SOP Geometry's operations below (wire_overwrite).
    overwrite = hou.ToggleParmTemplate(
        OVERWRITE_FROM_SOP, "Overwrite Attributes from SOP", default_value=False, join_with_next=True,
        help="Copies the listed attributes from the SOP onto the pieces every step, so changes to them reach the "
             "simulation. Off: read once when the object is created.")
    overwrite.hideLabel(True)
    overwrite_attribs = hou.StringParmTemplate(
        OVERWRITE_ATTRIBS, "Overwrite Attributes from SOP", 1, default_value=(OVERWRITE_DEFAULT,),
        disable_when=f"{{ {OVERWRITE_FROM_SOP} == 0 }}",
        help="The point attributes copied from the SOP each step, matched by name. Wildcards work.")
    b3d_hda.attrib_menu(overwrite_attribs, classes="point", source=f"parm:{EFFECTIVE_SOP}")
    reimport = hou.ToggleParmTemplate(REIMPORT_SOP, "Read SOP Every Step", default_value=False,
                                      help="Reads the whole SOP again every step (animated colliders).")
    reimport.hide(True)
    deforming = hou.ToggleParmTemplate(
        DEFORMING, "Deforming", default_value=False,
        help="The pieces' shapes follow the SOP: it is read again every step and the shapes are rebuilt.")

    # Initial Object Type: Body Type and Deforming follow it, unless it is Custom (then they are the user's).
    initial_type = hou.MenuParmTemplate(
        INITIAL_TYPE, "Initial Object Type", INITIAL_ITEMS, INITIAL_LABELS, default_value=0,
        help="What kind of object the pieces are made as: Body Type and Deforming follow it. Custom lets you set them "
             "yourself. A point's own active, animated or deforming attribute still wins.")
    deforming.setConditional(hou.parmCondType.DisableWhen, f"{{ {INITIAL_TYPE} != custom }}")
    effective_body = hou.IntParmTemplate(EFFECTIVE_BODY, "Body Type", 1, default_value=(2,),
                                         default_expression=(BODY_FROM_TYPE,),
                                         default_expression_language=(hou.scriptLanguage.Hscript,))
    effective_deform = hou.IntParmTemplate(EFFECTIVE_DEFORM, "Deforming", 1, default_value=(0,),
                                           default_expression=(DEFORM_FROM_TYPE,),
                                           default_expression_language=(hou.scriptLanguage.Hscript,))
    effective_body.hide(True)
    effective_deform.hide(True)

    # Each Box3D Data folder becomes a tab. Its parameters outside the folders are DOP's own data
    # settings (Activation, Group, Data Name ...): not promoted, so the inner node keeps the values a
    # Box3D Data starts with (copied, their template defaults could differ, e.g. an empty Group).
    tabs = []
    for i, entry in enumerate(without_parmops(data.parmTemplateGroup().entries())):
        if entry.type() == hou.parmTemplateType.Folder:
            tabs.append(hou.FolderParmTemplate(f"b3dpo_tab{i}", entry.label(), entry.parmTemplates(),
                                               folder_type=hou.folderType.Tabs))
    tabs = edit_template(tabs, "bodytype", lambda pt: pt.setConditional(
        hou.parmCondType.DisableWhen, f"{{ {INITIAL_TYPE} != custom }}"))
    # Box3D Data's Add Impact Data moves from CCD & Contacts to the Shape tab, as RBD Packed Object has it with its
    # shape settings (still linked / written by name, wherever it sits). Start Asleep stays in the Sleep tab.
    object_level = [pt for pt in value_templates(tabs) if pt.name() in b3d_settings.SKIP]
    tab_folders, moved = pull(tabs, lambda pt: pt.name() == IMPACT_TOGGLE)
    tabs = [t for t in tab_folders if t.parmTemplates()]
    shape_tab = next((t for t in tabs if t.label() == "Shape"), None)
    if moved and shape_tab is not None:
        shape_tab.setParmTemplates(list(shape_tab.parmTemplates()) + moved)
    elif moved:
        warn("no Shape tab: Add Impact Data is not on the asset")
    names = [pt for _, pt in promoted]
    names[1:1] = [create_frame, sim_frame]
    names += [initial_type, overwrite, overwrite_attribs, reimport, deforming, effective_body, effective_deform]
    # Geometry Source above SOP Path, which only the SOP choice uses. The stage reads the path the choice gives.
    geo_source = hou.MenuParmTemplate(
        GEO_SOURCE, "Geometry Source", tuple(token for token, _ in GEO_SOURCES),
        tuple(label for _, label in GEO_SOURCES), default_value=0,
        help="Where the pieces come from: the SOP Path, or the geometry wired into the DOP network's first to "
             "fourth input.")
    effective_sop = hou.StringParmTemplate(
        EFFECTIVE_SOP, "SOP Path", 1, default_value=("",),
        default_expression=(f'ifs(ch("{GEO_SOURCE}") == 0, chs("{SOP_PATH}"), '
                            f'dopcontextgeo(".", ch("{GEO_SOURCE}") - 1))',),
        default_expression_language=(hou.scriptLanguage.Hscript,))
    effective_sop.hide(True)
    at = next(i for i, pt in enumerate(names) if pt.name() == SOP_PATH)
    names[at].setConditional(hou.parmCondType.DisableWhen, f"{{ {GEO_SOURCE} != sop }}")
    names[at:at] = [geo_source]
    names.append(effective_sop)
    names += display_templates()
    group = hou.ParmTemplateGroup()
    group.append(hou.FolderParmTemplate("b3dpo_object", "Object", names, folder_type=hou.folderType.Tabs))
    group.append(hou.FolderParmTemplate("initial_state", "Initial State",
                                        initial_state_templates(),
                                        folder_type=hou.folderType.Tabs))
    for tab in tabs:
        group.append(tab)
    rows = [r for _, rs in b3d_settings.settings(data, warn) for r in rs]

    # The SOP stage, read by SOP Geometry (whose SOP Path and Use Object Transform are no longer the
    # asset's: the stage reads them).
    build_stage(sub, rows)
    stamp_sop_geometry(geo)
    promoted = [(node, pt) for node, pt in promoted if pt.name() not in (SOP_PATH, transform_parm)]
    b3d_hda.tidy(sub)

    asset = b3d_hda.create_asset(sub, out_path, TYPE_NAME, LABEL, 0, 0)
    definition = asset.type().definition()
    definition.setParmTemplateGroup(group)
    definition.setIcon(ICON)
    b3d_hda.set_tab_submenu(definition, warn)
    b3d_hda.add_python_module(definition)
    definition.setComment("Packed geometry as a Box3D object, like RBD Packed Object: settings that differ "
                          "from Box3D Data's defaults become point attributes; a point's own attribute wins.")

    asset.allowEditingOfContents()
    wire_stage(asset, EFFECTIVE_SOP, transform_parm, OBJECT_NAME)
    wire_hires(asset, transform_parm)
    wire_overwrite(asset)
    for node, pt in promoted:
        link(node, asset, pt.name())
    inner_data = asset.node("box3d_data")
    for pt in object_level:
        link(inner_data, asset, pt.name())
    # The linked parameters' operations: a new data node starts at Use Default, and only an edit by
    # hand (or hou.Parm.set) switches it to Set Initial; a channel reference does not, so without
    # this the inner node would keep its built-in defaults whatever the asset's values.
    b3d_hda.set_operations(inner_data, "initial", warn)
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    check(out_path, dopnet)
    print(f"Box3D Packed Object asset: {os.path.normpath(out_path)}")


def check(out_path, dopnet):
    """On a fresh instance of the saved asset: Friction 0.25 and Kinematic become point attributes
    (friction, active, animated), Bounce left at its default writes none, a point's own friction is
    kept, Add Impact Data reaches the inner Box3D Data, and Initial State / Deforming write v, w and
    deforming."""
    hou.hda.installFile(out_path, force_use_assets=True)
    sop = hou.node("/obj").createNode("geo", "probe_geo")
    for child in sop.children():
        child.destroy()
    box = sop.createNode("box", "box")
    pack = sop.createNode("pack", "pack")
    pack.setInput(0, box)
    pack.setDisplayFlag(True)
    probe = dopnet.createNode(TYPE_NAME, "probe")
    for name, value in ((SOP_PATH, pack.path()), ("friction", 0.25), (INITIAL_TYPE, "custom"), ("bodytype", 1),
                        ("defaultshape", 1),
                        (IMPACT_TOGGLE, 1)):
        pt = probe.parmTuple(name)
        if pt is None:
            warn(f"probe: no parameter {name}")
            return
        pt.set((value,))
    out = probe.node(f"{STAGE}/OUT")
    g = out.geometry() if out is not None else None
    if g is None or not g.points():
        warn(f"probe: {STAGE}/OUT has no points ({out.errors() if out is not None else 'no node'})")
        return
    pt = g.points()[0]
    got = {n: (pt.attribValue(n) if g.findPointAttrib(n) else None)
           for n in ("friction", "active", "animated", "bounce", "restxform", "name", "bullet_georep")}
    if got["friction"] is None or abs(got["friction"] - 0.25) > 1e-6 or got["active"] != 0 or got["animated"] != 1:
        warn(f"probe: friction / active / animated on the points {got}, expected 0.25 / 0 / 1")
    if got["bullet_georep"] != "sphere":
        warn(f"probe: Default Shape Sphere wrote bullet_georep {got['bullet_georep']!r}, expected 'sphere'")
    if got["bounce"] is not None:
        warn("probe: bounce left at its default was written as an attribute")
    if not got["name"]:
        warn("probe: the points got no name")
    impact = probe.node("box3d_data").parm("addimpact") if probe.node("box3d_data") else None
    if impact is None or impact.eval() != 1:
        warn("probe: Add Impact Data does not reach the inner Box3D Data")
    # Display Geometry: Simulation shows the object's geometry, Render the SOP Guide; the collider label.
    guide, renderparms = probe.node(HIRES), probe.node("renderparmsvolatile")
    if guide is None or renderparms is None or probe.parm(DISPLAY_GEO) is None:
        warn("probe: no SOP Guide, Rendering Parameters Volatile or Display Geometry menu")
    else:
        seen = []
        for value in (0, 1):
            probe.parm(DISPLAY_GEO).set(value)
            seen.append((renderparms.parm("display").eval(), guide.parm("showguide").eval()))
        probe.parm(DISPLAY_GEO).set(0)
        if seen != [(1, 0), (0, 1)]:
            warn(f"probe: Display Geometry Simulation / Render gives (geometry shown, guide shown) {seen}, "
                 "expected (1, 0) / (0, 1)")
    label = probe.node("colliderlabel")
    if label is None or label.parm("colliderlabel").eval() != "PackedObject":
        warn("probe: no Collider Label 'PackedObject' (the solver would treat the object as one body)")
    # Overwrite Attributes from SOP: the inner SOP Geometry's operations follow it (Set Initial off, Always on).
    geo = probe.node("geometry")
    ops = [q for q in geo.parms() if q.name().startswith("parmop")] if geo is not None else []
    if probe.parm(OVERWRITE_FROM_SOP) is None or not ops:
        warn(f"probe: no {OVERWRITE_FROM_SOP} parameter or no Parameter Operation menus on the SOP Geometry")
    else:
        seen = []
        for value in (0, 1):
            probe.parm(OVERWRITE_FROM_SOP).set(value)
            seen.append(ops[0].evalAsString())
        probe.parm(OVERWRITE_FROM_SOP).set(0)
        if seen != ["initial", "always"]:
            warn(f"probe: Overwrite Attributes from SOP off / on gives {seen}, expected Set Initial / Set Always")
    # A point's own friction is kept.
    attr = sop.createNode("attribwrangle", "own_friction")
    attr.setInput(0, pack)
    attr.parm("snippet").set("f@friction = 0.75;")
    attr.setDisplayFlag(True)
    probe.parmTuple(SOP_PATH).set((attr.path(),))
    g = out.geometry()
    own = g.points()[0].attribValue("friction") if g is not None and g.findPointAttrib("friction") else None
    if own is None or abs(own - 0.75) > 1e-6:
        warn(f"probe: a point's own friction 0.75 came out as {own}")
    # Initial State: with Inherit Velocity on no v is written, Velocity / Angular Velocity once it is off (its default);
    # Deforming writes deforming and re-imports.
    if probe.parm(INHERIT_VEL) is not None:
        probe.parm(INHERIT_VEL).set(1)
    g = out.geometry()
    if g is not None and g.findPointAttrib("v") is not None:
        warn("probe: Inherit Velocity on still wrote v (it should add nothing)")
    for name, value in ((INHERIT_VEL, 0), (INITIAL_TYPE, "custom"), (DEFORMING, 1)):
        if probe.parm(name) is None:
            warn(f"probe: no {name} parameter")
            sop.destroy()
            return
        probe.parm(name).set(value)
    probe.parmTuple(VEL).set((1, 2, 3))
    probe.parmTuple(ANGVEL).set((180, 0, 0))
    g = out.geometry()
    p0 = g.points()[0] if g is not None and g.points() else None
    got = {n: (p0.attribValue(n) if p0 is not None and g.findPointAttrib(n) else None) for n in ("v", "w", "deforming")}
    if got["v"] is None or max(abs(x - y) for x, y in zip(got["v"], (1, 2, 3))) > 1e-5:
        warn(f"probe: Velocity (1, 2, 3) wrote v {got['v']}")
    if got["w"] is None or abs(got["w"][0] - 3.14159265) > 1e-4:
        warn(f"probe: Angular Velocity 180 degrees per second wrote w {got['w']}, expected pi radians")
    if got["deforming"] != 1:
        warn(f"probe: Deforming wrote deforming {got['deforming']}, expected 1")
    if ops and ops[0].evalAsString() != "always":
        warn("probe: Deforming does not re-import the SOP (Set Always)")
    sop.destroy()


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 0
    try:
        build(sys.argv[1])
    except Exception:  # noqa: BLE001 - never fail the plugin build over the asset
        warn("the Box3D Packed Object asset was not built:\n" + traceback.format_exc())
    return 0


if __name__ == "__main__":
    sys.exit(main())
