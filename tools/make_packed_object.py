"""Builds the Box3D Packed Object digital asset (a DOP) with hython.

    hython tools/make_packed_object.py <output .hda>

The build runs this after compiling the plugin (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_packedobject.hda next to the plugin's dso folder. The plugin must already be
installed: the asset contains a Box3D Data node, and its parameters are copied from it.

What it is: RBD Packed Object without the attributes. RBD Packed Object writes its own mass,
density, friction, bounce and more onto every point, and point attributes win over Box3D Data, so
Box3D Data's values for those do nothing there. This asset only brings the SOP geometry in:

    Empty Object -> SOP Geometry -> Box3D Data -> (output)

Its parameters: the Object tab (object name, creation frame, solve on creation frame, SOP path,
object transform), then every Box3D Data folder as a tab of its own. Each inner parameter is a
channel reference to the outer one, so the asset is rebuilt from the plugin's current Box3D Data
every build.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import sys
import traceback

import hou

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

# Templates without a value: nothing to reference.
NO_VALUE = {hou.parmTemplateType.Folder, hou.parmTemplateType.FolderSet,
            hou.parmTemplateType.Separator, hou.parmTemplateType.Label,
            hou.parmTemplateType.Button}


def warn(msg):
    print(f"make_packed_object: WARNING: {msg}")


def create_any(parent, type_names, name):
    for t in type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed:
            continue
    raise RuntimeError(f"none of the node types {type_names} exist in {parent.path()}")


def template_by_label(node, label):
    """The parameter labelled `label`. DOP data nodes give each parameter a Parameter Operation menu
    (parmop_<name>) with the same label: those are skipped."""
    for pt in node.parmTemplateGroup().entriesWithoutFolders():
        if pt.name().startswith("parmop") or pt.type() in NO_VALUE:
            continue
        if pt.label().lower() == label.lower():
            return pt
    return None


def labels(node):
    return ", ".join(sorted({f"{pt.label()} ({pt.name()})" for pt in node.parmTemplateGroup().entriesWithoutFolders()
                             if not pt.name().startswith("parmop") and pt.type() not in NO_VALUE}))


def without_parmops(entries):
    """`entries` without the Parameter Operation menus (parmop_<name>), recursing into folders. The
    inner Box3D Data keeps its own (Set Initial etc.): copied onto the asset they came out as Use
    Default, so the inner node took its built-in defaults whatever the asset's values."""
    out = []
    for pt in entries:
        if pt.name().startswith("parmop"):
            continue
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            pt.setParmTemplates(without_parmops(pt.parmTemplates()))
        out.append(pt)
    return out


def value_templates(entries):
    """Every template with a value under `entries`, recursing into folders."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            out.extend(value_templates(pt.parmTemplates()))
        elif pt.type() not in NO_VALUE:
            out.append(pt)
    return out


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


def set_operations(node, token):
    """Every Parameter Operation menu (parmop_*) of `node` to the item `token` (default / initial /
    always / never)."""
    changed = 0
    for p in node.parms():
        if not p.name().startswith("parmop"):
            continue
        items = list(p.menuItems()) if p.parmTemplate().type() == hou.parmTemplateType.Menu else []
        try:
            if p.parmTemplate().type() == hou.parmTemplateType.String:
                p.set(token)
            elif token in items:
                p.set(items.index(token))
            else:
                warn(f"{p.path()}: no {token!r} item in {items}")
                continue
            changed += 1
        except hou.Error as e:
            warn(f"{p.path()}: {e}")
    return changed


def _has_expr(parm):
    try:
        parm.expression()
        return True
    except hou.OperationFailed:
        return False


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "build")
    sub = dopnet.createNode("subnet", TYPE_NAME)

    obj = sub.createNode("emptyobject", "emptyobject")
    geo = create_any(sub, ["sopgeo", "sopgeometry"], "geometry")
    geo.setInput(0, obj)
    data = sub.createNode("box3d_data", "box3d_data")
    data.setInput(0, geo)
    data.setDisplayFlag(True)  # the asset's output; it has no input
    sub.layoutChildren()

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

    # Creation Frame: Empty Object has none; like RBD Packed Object, the object is created on the
    # step its Activation is true, here only on that simulation frame.
    create_frame = hou.IntParmTemplate(
        "createframe", "Creation Frame", 1, default_value=(1,),
        help="The simulation frame ($SF) on which the object is created.")
    obj.parm("activation").setExpression('$SF == ch("../createframe")')

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
    group = hou.ParmTemplateGroup()
    group.append(hou.FolderParmTemplate("b3dpo_object", "Object", names, folder_type=hou.folderType.Tabs))
    for tab in tabs:
        group.append(tab)
    data_templates = value_templates(tabs)

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
    definition.setComment("Packed geometry as a Box3D object, without RBD Packed Object's stamped "
                          "attributes: Box3D Data's settings apply unless a point sets its own.")

    asset.allowEditingOfContents()
    for node, pt in promoted:
        link(node, asset, pt.name())
    inner_data = asset.node("box3d_data")
    for pt in data_templates:
        link(inner_data, asset, pt.name())
    # Each parameter's operation: a new data node starts at Use Default, and only an edit by hand
    # (or hou.Parm.set) switches it to Set Initial; a channel reference does not, so without this
    # the inner node would keep its built-in defaults whatever the asset's values.
    set_operations(inner_data, "initial")
    definition.updateFromNode(asset)
    definition.save(out_path, asset)

    # Check on a fresh instance of the saved asset that the links hold.
    hou.hda.installFile(out_path, force_use_assets=True)
    probe = dopnet.createNode(TYPE_NAME, "probe")
    probe.parm("friction").set(0.123)
    inner_friction = probe.node("box3d_data").parm("friction")
    got = inner_friction.eval()
    ref = inner_friction.getReferencedParm()
    op = probe.node("box3d_data").parm("parmop_friction")
    if op is not None and op.evalAsString() != "initial":
        warn(f"the inner Box3D Data's Friction operation is {op.evalAsString()!r}, not 'initial'")
    if abs(got - 0.123) > 1e-6:
        warn(f"the inner Box3D Data's Friction reads {got} after setting the asset's to 0.123 "
             f"(it references {ref.path()}; expression {inner_friction.expression() if _has_expr(inner_friction) else 'none'})")

    print(f"Box3D Packed Object asset: {os.path.normpath(out_path)}")


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
