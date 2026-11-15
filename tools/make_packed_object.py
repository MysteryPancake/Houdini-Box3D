"""Builds the Box3D Packed Object digital asset (a DOP) with hython.

    hython tools/make_packed_object.py <output .hda>

The build runs this after compiling the plugin (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_packedobject.hda next to the plugin's dso folder. The plugin must already be
installed: the asset contains a Box3D Data node, and its parameters are copied from it.

What it is: RBD Packed Object for Box3D. Like it, a SOP stage inside the object
(create_attributes, as RBD Packed Object's create_or_overwrite_attributes) prepares the SOP
geometry when the object is created:

    Empty Object -> SOP Geometry (create_attributes/OUT) -> Box3D Data -> (output)
    create_attributes: Object Merge of the SOP Path -> setup_defaults -> create_name
                       -> [creation frame] set_rest -> OUT

setup_defaults writes each per-body setting whose value differs from Box3D Data's default as a
point attribute (b3d_settings.packed_defaults_vex), unless the points already have it, so a point
attribute still wins and untouched settings write nothing. set_rest resets the rest and bakes
orient / pscale / scale, on the creation frame only (b3d_settings.REST_VEX; stamped $SF / $OBJCF,
as RBD Packed Object). The inner Box3D Data holds only the object-level settings
(b3d_settings.SKIP), channel references to the asset's; its other values stay at the defaults, so
a missing attribute means the default. Settings change mid-simulation through the attributes (a
SOP Solver, POP Wrangle), not through the asset's parameters.

Its parameters: the Object tab (object name, creation frame, solve on creation frame, SOP path,
object transform), then every Box3D Data folder as a tab of its own.

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
from b3d_hda import NO_VALUE, labels, value_templates, without_parmops  # noqa: E402

TYPE_NAME = "box3d_packedobject"
LABEL = "Box3D Packed Object"
ICON = "DOP_box3d_packedobject"  # config/Icons/DOP_box3d_packedobject.svg, installed by the build

# Parameters promoted from the inner Empty Object and SOP Geometry, by label (internal names are not
# documented for every node). Required ones fail the build of the asset; the rest are skipped with a
# warning when this Houdini version labels them differently.
OBJECT_PARMS = [
    ("emptyobject", "Object Name", True),
    ("emptyobject", "Solve on Creation Frame", False),
    ("geometry", "SOP Path", True),
    ("geometry", "Use Object Transform", False),
]

OVERWRITE_FROM_SOP = "overwritefromsop"  # RBD Packed Object's Overwrite Attributes from SOP

STAGE = "create_attributes"  # the SOP stage (a DOP SOP network), as RBD Packed Object's

warn = b3d_hda.warner("make_packed_object")


def template_by_label(node, label):
    """The parameter labelled `label`. DOP data nodes give each parameter a Parameter Operation menu
    (parmop_<name>) with the same label: those are skipped."""
    for pt in node.parmTemplateGroup().entriesWithoutFolders():
        if pt.name().startswith("parmop") or pt.type() in NO_VALUE:
            continue
        if pt.label().lower() == label.lower():
            return pt
    return None


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


def build_stage(sub, rows):
    """The SOP stage inside the asset (STAGE), as RBD Packed Object's create_or_overwrite_attributes:
    Object Merge of the asset's SOP Path -> setup_defaults -> create_name -> set_rest on the creation
    frame only (switch_first_frame on the stamped $SF / $OBJCF) -> OUT. The references to the asset's
    parameters are set by wire_stage once the asset has them. Returns OUT."""
    net = sub.createNode("sopnet", STAGE)
    merge = net.createNode("object_merge", "object_merge")
    defaults = net.createNode("attribwrangle", "setup_defaults")
    defaults.setInput(0, merge)
    defaults.parm("class").set("detail")
    defaults.parm("snippet").set(b3d_settings.packed_defaults_vex(rows, "../.."))
    name = net.createNode("attribwrangle", "create_name")
    name.setInput(0, defaults)
    name.addSpareParmTuple(hou.StringParmTemplate("objname", "Object Name", 1))
    name.parm("snippet").set(b3d_settings.NAME_VEX)
    rest = net.createNode("attribwrangle", "set_rest")
    rest.setInput(0, name)
    rest.parm("snippet").set(b3d_settings.REST_VEX)
    stamps = net.createNode("null", "STAMPS")
    for parm, key in (("frame", "SF"), ("objcf", "OBJCF")):
        stamps.addSpareParmTuple(hou.FloatParmTemplate(parm, key, 1))
        stamps.parm(parm).setExpression(f'stamp("..", "{key}", -1)', hou.exprLanguage.Hscript)
    first = net.createNode("switch", "switch_first_frame")
    first.setInput(0, name)
    first.setInput(1, rest)
    first.parm("input").setExpression('ch("../STAMPS/frame") == ch("../STAMPS/objcf")',
                                      hou.exprLanguage.Hscript)
    out = net.createNode("null", "OUT")
    out.setInput(0, first)
    out.setDisplayFlag(True)
    out.setRenderFlag(True)
    net.layoutChildren()
    return out


def wire_stage(asset, sop_path_parm, transform_parm, objname_parm):
    """The stage's references to the asset's parameters: the Object Merge's path and transform (Use
    Object Transform: Into This Object, the DOP network's space, so world space), and the name
    prefix."""
    merge = asset.node(f"{STAGE}/object_merge")
    merge.parm("objpath1").set(f'`chsop("../../{sop_path_parm}")`')
    if transform_parm and merge.parm("xformtype") is not None:
        merge.parm("xformtype").setExpression(f'ch("../../{transform_parm}")', hou.exprLanguage.Hscript)
    elif transform_parm:
        warn(f"{merge.path()} has no Transform (xformtype): Use Object Transform does nothing")
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
    geo.parm("numstamps").set(2)
    for i, key in enumerate(("SF", "OBJCF"), start=1):
        geo.parm(f"stampname{i}").set(key)
        geo.parm(f"stampvalue{i}").setExpression(f"${key}", hou.exprLanguage.Hscript)


def wire_overwrite(asset):
    """Every Parameter Operation of the inner SOP Geometry (and its Default Operation): Set Always
    while Overwrite Attributes from SOP is on, else Set Initial."""
    geo = asset.node("geometry")
    ops = [p for p in geo.parms()
           if p.name().startswith("parmop") or p.parmTemplate().label().lower() == "default operation"]
    if not ops:
        warn(f"{geo.path()} has no Parameter Operation menus: Overwrite Attributes from SOP does nothing")
    driven = [p.name() for p in ops
              if b3d_hda.menu_index_expr(p, f'ch("../{OVERWRITE_FROM_SOP}")', "Always", "Initial", warn)]
    print(f"make_packed_object: Overwrite Attributes from SOP drives {geo.path()}: {', '.join(driven) or 'nothing'}")


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "build")
    sub = dopnet.createNode("subnet", TYPE_NAME)

    obj = sub.createNode("emptyobject", "emptyobject")
    geo = b3d_hda.create(sub, ["sopgeo", "sopgeometry"], "geometry")
    geo.setInput(0, obj)
    data = sub.createNode("box3d_data", "box3d_data")
    data.setInput(0, geo)
    data.setDisplayFlag(True)  # the asset's output; it has no input

    # The parameters: the Object tab, then each Box3D Data folder as a tab.
    inner = {"emptyobject": obj, "geometry": geo}
    promoted = []  # (inner node, template)
    for node_name, label, required in OBJECT_PARMS:
        pt = template_by_label(inner[node_name], label)
        if pt is None:
            msg = (f"{inner[node_name].path()} has no parameter labelled {label!r}. Its parameters: "
                   f"{labels(inner[node_name])}")
            if required:
                raise RuntimeError(msg)
            warn(msg + "; not promoted")
            continue
        pt = pt.clone()
        if label == "Object Name" and pt.type() == hou.parmTemplateType.String:
            pt.setDefaultValue(("$OS",))
        promoted.append((inner[node_name], pt))
    by_label = {pt.label(): pt.name() for _, pt in promoted}
    sop_path_parm = by_label["SOP Path"]
    transform_parm = by_label.get("Use Object Transform")

    # Creation Frame: Empty Object has none; like RBD Packed Object, the object is created on the
    # step its Activation is true, here only on that simulation frame.
    create_frame = hou.IntParmTemplate(
        "createframe", "Creation Frame", 1, default_value=(1,),
        help="The simulation frame ($SF) on which the object is created.")
    obj.parm("activation").setExpression('$SF == ch("../createframe")')

    # Overwrite Attributes from SOP, as RBD Packed Object's: SOP Geometry imports the SOP once on the
    # creation frame (Set Initial) or every step (Set Always), so an animated or deforming SOP reaches
    # the simulation. Wired to the inner SOP Geometry's operations below (wire_overwrite).
    overwrite = hou.ToggleParmTemplate(
        OVERWRITE_FROM_SOP, "Overwrite Attributes from SOP", default_value=False,
        help="Reads the SOP again every step, so animated or deforming geometry and changed attributes reach \
the simulation. Off: read once when the object is created.")

    # Each Box3D Data folder becomes a tab. Its parameters outside the folders are DOP's own data
    # settings (Activation, Group, Data Name ...): not promoted, so the inner node keeps the values a
    # Box3D Data starts with (copied, their template defaults could differ, e.g. an empty Group).
    tabs = []
    for i, entry in enumerate(without_parmops(data.parmTemplateGroup().entries())):
        if entry.type() == hou.parmTemplateType.Folder:
            tabs.append(hou.FolderParmTemplate(f"b3dpo_tab{i}", entry.label(), entry.parmTemplates(),
                                               folder_type=hou.folderType.Tabs))
    names = [pt for _, pt in promoted]
    names.insert(1, create_frame)
    names.append(overwrite)
    group = hou.ParmTemplateGroup()
    group.append(hou.FolderParmTemplate("b3dpo_object", "Object", names, folder_type=hou.folderType.Tabs))
    for tab in tabs:
        group.append(tab)
    object_level = [pt for pt in value_templates(tabs) if pt.name() in b3d_settings.SKIP]
    rows = [r for _, rs in b3d_settings.settings(data, warn) for r in rs]

    # The SOP stage, read by SOP Geometry (whose SOP Path and Use Object Transform are no longer the
    # asset's: the stage reads them).
    build_stage(sub, rows)
    stamp_sop_geometry(geo)
    promoted = [(node, pt) for node, pt in promoted if pt.name() not in (sop_path_parm, transform_parm)]
    sub.layoutChildren()

    if os.path.exists(out_path):
        try:
            hou.hda.uninstallFile(out_path)
        except hou.OperationFailed:
            pass
        os.remove(out_path)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    asset = sub.createDigitalAsset(name=TYPE_NAME, hda_file_name=out_path, description=LABEL,
                                   min_num_inputs=0, max_num_inputs=0)
    definition = asset.type().definition()
    definition.setParmTemplateGroup(group)
    definition.setIcon(ICON)
    definition.setComment("Packed geometry as a Box3D object, like RBD Packed Object: settings that differ "
                          "from Box3D Data's defaults become point attributes; a point's own attribute wins.")

    asset.allowEditingOfContents()
    wire_stage(asset, sop_path_parm, transform_parm, by_label["Object Name"])
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
    kept, and Add Impact Data reaches the inner Box3D Data."""
    hou.hda.installFile(out_path, force_use_assets=True)
    sop = hou.node("/obj").createNode("geo", "probe_geo")
    for child in sop.children():
        child.destroy()
    box = sop.createNode("box", "box")
    pack = sop.createNode("pack", "pack")
    pack.setInput(0, box)
    pack.setDisplayFlag(True)
    probe = dopnet.createNode(TYPE_NAME, "probe")
    for label, value in (("SOP Path", pack.path()), ("Friction", 0.25), ("Body Type", 1), ("Default Shape", 1),
                         ("Add Impact Data", 1)):
        pt = template_by_label(probe, label)
        if pt is None:
            warn(f"probe: no parameter labelled {label!r}")
            return
        probe.parmTuple(pt.name()).set((value,))
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
    # Overwrite Attributes from SOP: the inner SOP Geometry's operations follow it (Set Initial off, Always on).
    geo = probe.node("geometry")
    ops = [q for q in geo.parms() if q.name().startswith("parmop")] if geo is not None else []
    if probe.parm(OVERWRITE_FROM_SOP) is None or not ops:
        warn(f"probe: no {OVERWRITE_FROM_SOP} parameter or no Parameter Operation menus on the SOP Geometry")
    else:
        seen = []
        for value in (0, 1):
            probe.parm(OVERWRITE_FROM_SOP).set(value)
            seen.append(ops[0].menuLabels()[ops[0].eval()] if ops[0].parmTemplate().type() == hou.parmTemplateType.Menu
                        else ops[0].evalAsString())
        probe.parm(OVERWRITE_FROM_SOP).set(0)
        if "initial" not in seen[0].lower() or "always" not in seen[1].lower():
            warn(f"probe: Overwrite Attributes from SOP off / on gives {seen}, expected Set Initial / Set Always")
    # A point's own friction is kept.
    attr = sop.createNode("attribwrangle", "own_friction")
    attr.setInput(0, pack)
    attr.parm("snippet").set("f@friction = 0.75;")
    attr.setDisplayFlag(True)
    probe.parmTuple(template_by_label(probe, "SOP Path").name()).set((attr.path(),))
    g = out.geometry()
    own = g.points()[0].attribValue("friction") if g is not None and g.findPointAttrib("friction") else None
    if own is None or abs(own - 0.75) > 1e-6:
        warn(f"probe: a point's own friction 0.75 came out as {own}")
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
