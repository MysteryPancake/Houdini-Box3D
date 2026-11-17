"""Builds the Box3D Constraint Properties digital asset (a SOP, like RBD Constraint Properties) with hython.

    hython tools/make_constraint_properties_sop.py <output .hda>

The build runs this after the plugin (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_constraint_properties.hda. The plugin must be installed: every setting, its
label, default, range and help is copied from the Box3D constraint relationship nodes, so the two
always match.

What it is: the settings of every Box3D joint type as primitive attributes on the lines of a
Constraint Network's geometry. Every setting has a checkbox; only ticked ones are written, onto the
lines in the Constraint Group (or in the Filter by Pieces groups). A line that is not edited keeps
the relationship node's value: the attributes are created with the "not set" value the solver skips
(kUnsetFloat in Box3DJoints.h: -3.4028235e38 for floats and the first component of vectors, -1 for
toggles). Also writable: the line's constraint_name (the relationship's Data Name) and
next_constraint_name (what a broken joint switches to), and a display colour.

    Inputs / outputs: Geometry, Constraint Geometry (pink), Proxy Geometry, as the Box3D Solver SOP's
    first three. Only the Constraint Geometry is edited; the others pass through (they are what Filter
    by Pieces looks at). IN_constraints -> pick_constraints (or SideFX's RBD Group Constraints with
    Filter by Pieces on) -> set_properties (one detail wrangle) -> strip_group.

Folders: Joint (the settings every type has), then one folder per joint type, showing the one picked
in Settings For. A setting is named <type>_<attribute> in a type's folder and writes <attribute>.

Never @ bindings in the wrangle: a binding creates its attribute on every primitive at compile time.

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
from b3d_hda import without_parmops  # noqa: E402
from b3d_layouts import CONSTRAINT_PROPERTIES_LAYOUT  # noqa: E402

TYPE_NAME = "box3d_constraint_properties"
LABEL = "Box3D Constraint Properties"
ICON = "DOP_box3d_data"  # installed by the build for Box3D Data (config/Icons)

LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry"]  # inputs and outputs
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}  # constraints pink, as the Box3D Solver SOP

# The joint types in Box3D's order (Box3DJoints.h JointType): DOP node `box3d_<name>conrel`, label.
JOINT_TYPES = [("weld", "Weld"), ("spherical", "Spherical"), ("revolute", "Revolute"),
               ("prismatic", "Prismatic"), ("distance", "Distance"), ("motor", "Motor"),
               ("parallel", "Parallel"), ("wheel", "Wheel"), ("filter", "Filter")]

# Parameters of the relationship nodes that are no primitive attribute (Box3DJoints.h
# jointNodeToggles, kAttributesReplaceName): left out.
NODE_ONLY = {"attributes_replace", "breakable", "set_rest_length", "limit_spring_force",
             "limit_max_length", "limit_torque"}

GROUP_ATTRIB = "__tmpconpropgrp"  # the group SideFX's RBD Group Constraints makes
UNSET_FLOAT = "-3.4028235e38"  # kUnsetFloat (Box3DJoints.h)
UNSET_INT = "-1"

warn = b3d_hda.warner("make_constraint_properties_sop")

# Nodes inside whose errors and warnings show on the asset (b3d_hda.set_message_nodes).
MESSAGE_NODES = ["warn_if_no_constraints"]


def first_sentence(text):
    m = re.match(r"(.+?[.;])(\s|$)", text or "")
    return m.group(1) if m else (text or "")


def collect(dopnet):
    """The settings, from the relationship nodes: (common, per_type). Each is a list of (template, kind),
    kind f / i / v (the attribute's type; the attribute is named like the template). `common` is the
    Joint folder of the Weld node (what every type but Filter has); `per_type` maps a type's name to
    the settings in its own folder."""
    common, per_type = [], {}
    for name, label in JOINT_TYPES:
        try:
            node = dopnet.createNode(f"box3d_{name}conrel", f"b3d_{name}")
        except hou.OperationFailed:
            raise RuntimeError(f"the box3d_{name}conrel DOP is not installed: build the plugin first")
        folders = [f for f in without_parmops(node.parmTemplateGroup().entries())
                   if f.type() == hou.parmTemplateType.Folder]
        for f in folders:
            rows = []
            for pt in b3d_hda.value_templates(f.parmTemplates()):
                if pt.name() in NODE_ONLY:
                    continue
                if pt.type() == hou.parmTemplateType.Toggle or pt.type() == hou.parmTemplateType.Int:
                    rows.append((pt, "i"))
                elif pt.type() == hou.parmTemplateType.Float:
                    rows.append((pt, "v" if pt.numComponents() == 3 else "f"))
                else:
                    warn(f"{node.name()}'s {pt.name()} ({pt.type()}) has no attribute mapping; left out")
            if f.label() == "Joint":
                if name == "weld":
                    common = rows
            elif f.label() == label:
                per_type[name] = rows
    return common, per_type


def condition(pt, prefix, names):
    """`pt`'s disable-when for the asset: each group `{ x == v }` keeps only if every name it reads is a
    setting here (a node toggle such as Breakable is not), is renamed with `prefix` and also needs
    `add<x>` ticked, so it only applies while that setting is being written."""
    cond = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
    out = []
    for body in re.findall(r"\{([^}]*)\}", cond):
        refs = list(dict.fromkeys(re.findall(r"(\w+)\s*[!=<>]=?", body)))
        if any(r not in names for r in refs):
            continue
        for r in refs:
            body = re.sub(rf"\b{r}\b", prefix + r, body)
        extra = "".join(f" add{prefix}{r} == 1" for r in refs)
        out.append("{" + body.rstrip() + extra + " }")
    return " ".join(out)


def setting_templates(rows, prefix, names):
    """The checkbox and value of each setting: add<prefix><name> and <prefix><name>."""
    items = []
    for pt, kind in rows:
        pname = prefix + pt.name()
        value = pt.clone()
        value.setName(pname)
        value.setHelp(f"{first_sentence(pt.help())} Writes the {pt.name()} primitive attribute.")
        box = hou.ToggleParmTemplate(f"add{pname}", value.label(), default_value=False, join_with_next=True,
                                     help="Write this setting onto the constraints.")
        box.hideLabel(True)
        gates = condition(pt, prefix, names)
        value.setConditional(hou.parmCondType.DisableWhen, (gates + " " if gates else "") + f"{{ add{pname} == 0 }}")
        items += [box, value]
    return items


def group_tag(inputindex):
    """RBD Constraint Properties' selection button (pick primitives in the viewport)."""
    return {"script_action": "import soputils\nkwargs['geometrytype'] = hou.geometryType.Primitives\n"
                             f"kwargs['inputindex'] = {inputindex}\nsoputils.selectGroupParm(kwargs)",
            "script_action_help": "Select geometry from an available viewport.",
            "script_action_icon": "BUTTONS_reselect"}


def ui(common, per_type):
    names_menu = [label for _, label in JOINT_TYPES]
    cgroup = hou.StringParmTemplate(
        "constraintgroup", "Constraint Group", 1,
        help="The constraints to edit: a primitive group or a pattern. Empty edits every constraint.",
        tags=dict(group_tag(1), autoscope="0000000000000000"))
    bypieces = hou.ToggleParmTemplate(
        "groupfromgeo", "Filter by Pieces", default_value=False,
        help="Edit the constraints between the pieces in Group 1 and Group 2 instead of a Constraint Group.")
    pieces = [
        hou.MenuParmTemplate("usegroups", "Groups", ("0", "1"), ("All to Group", "Group to Group"),
                             default_value=0, help="Group 1 to every piece, or Group 1 to Group 2."),
        hou.StringParmTemplate("group1", "Group 1", 1, help="The pieces on one end.", tags=group_tag(0)),
        hou.StringParmTemplate("group2", "Group 2", 1, help="The pieces on the other end.", tags=group_tag(0)),
    ]
    pieces[0].setConditional(hou.parmCondType.DisableWhen, "{ groupfromgeo == 0 }")
    pieces[1].setConditional(hou.parmCondType.DisableWhen, "{ groupfromgeo == 0 }")
    pieces[2].setConditional(hou.parmCondType.DisableWhen, "{ usegroups < 1 } { groupfromgeo == 0 }")
    enable_color = hou.ToggleParmTemplate("enablecolor", "Enable Color", default_value=False,
                                          help="Colour the edited constraints.")
    color = hou.FloatParmTemplate("constraintcolor", "Color", 3, default_value=(1.0, 1.0, 1.0),
                                  look=hou.parmLook.ColorSquare, naming_scheme=hou.parmNamingScheme.RGBA,
                                  help="The colour, written as Cd.")
    color.setConditional(hou.parmCondType.DisableWhen, "{ enablecolor == 0 }")

    def string_row(name, label, help_text):
        """A checkbox and a string with the types' Data Names in its menu."""
        box = hou.ToggleParmTemplate(f"do{name}", label, default_value=False, join_with_next=True,
                                     help="Write this onto the constraints.")
        box.hideLabel(True)
        value = hou.StringParmTemplate(name, label, 1, default_value=(names_menu[0],), menu_items=names_menu,
                                       menu_labels=names_menu, menu_type=hou.menuType.StringReplace,
                                       help=help_text)
        value.setConditional(hou.parmCondType.DisableWhen, f"{{ do{name} == 0 }}")
        return [box, value]

    which = hou.MenuParmTemplate("jointtype", "Settings For", tuple(str(i) for i in range(len(JOINT_TYPES))),
                                 tuple(names_menu), default_value=0,
                                 help="Which joint type's settings to show. Every ticked setting is written.")
    props = (string_row("constraintname", "Constraint Name",
                        "The relationship's Data Name the constraints use. Writes constraint_name.")
             + string_row("next_constraintname", "Next Constraint Name",
                          "What a broken constraint switches to. Writes next_constraint_name.")
             + [which])

    common_names = {pt.name() for pt, _ in common}
    folders = [b3d_hda.collapsible("b3dcp_common", "Joint", setting_templates(common, "", common_names))]
    for i, (name, label) in enumerate(JOINT_TYPES):
        rows = per_type.get(name, [])
        if not rows:
            continue
        own = {pt.name() for pt, _ in rows}
        f = b3d_hda.collapsible(f"b3dcp_{name}", label, setting_templates(rows, f"{name}_", own))
        f.setConditional(hou.parmCondType.HideWhen, f"{{ jointtype != {i} }}")
        folders.append(f)

    group = hou.ParmTemplateGroup()
    group.append(cgroup)
    group.append(bypieces)
    group.append(b3d_hda.collapsible("b3dcp_pieces", "Filter by Pieces", pieces, open_folder=False,
                                     header_toggle="groupfromgeo"))
    group.append(enable_color)
    group.append(color)
    group.append(b3d_hda.collapsible("b3dcp_properties", "Constraint Properties", props))
    for f in folders:
        group.append(f)
    return group


def attribute_rows(common, per_type):
    """(attribute, kind, value parameter, checkbox parameter) of every setting."""
    out = [(pt.name(), kind, pt.name(), f"add{pt.name()}") for pt, kind in common]
    for name, _ in JOINT_TYPES:
        out += [(pt.name(), kind, f"{name}_{pt.name()}", f"add{name}_{pt.name()}")
                for pt, kind in per_type.get(name, [])]
    return out


def wrangle_vex(rows):
    """The whole wrangle: one detail pass over the constraints in the group. Each ticked setting creates its
    attribute first (the "not set" value, so the other lines keep the node's value), then sets the lines."""
    unset = {"i": UNSET_INT, "f": UNSET_FLOAT, "v": "{" + UNSET_FLOAT + ", 0.0, 0.0}"}
    read = {"i": "chi", "f": "chf", "v": "chv"}

    def put(attr, default, value):
        return (f'if (!hasprimattrib(0, "{attr}")) addprimattrib(0, "{attr}", {default}); '
                f'foreach (int pr; prims) setprimattrib(0, "{attr}", pr, {value});')

    lines = [
        "// Box3D Constraint Properties: each ticked setting is written onto the constraints in the group.",
        f'string group = chi("../groupfromgeo") ? "{GROUP_ATTRIB}" : chs("../constraintgroup");',
        "int prims[];",
        "if (group == \"\") { for (int i = 0; i < nprimitives(0); i++) append(prims, i); }",
        "else prims = expandprimgroup(0, group);",
        "",
        'if (chi("../enablecolor")) { ' + put("Cd", "{1.0, 1.0, 1.0}", 'chv("../constraintcolor")') + " }",
        'if (chi("../doconstraintname")) { ' + put("constraint_name", '""', 'chs("../constraintname")') + " }",
        'if (chi("../donext_constraintname")) { '
        + put("next_constraint_name", '""', 'chs("../next_constraintname")') + " }",
    ]
    for attr, kind, value, box in rows:
        lines.append(f'if (chi("../{box}")) {{ ' + put(attr, unset[kind], f'{read[kind]}("../{value}")') + " }")
    return "\n".join(lines)


def build_network(sub, rows):
    ins = sub.indirectInputs()
    nulls = []
    for i, name in enumerate(["IN_geometry", "IN_constraints", "IN_proxy"]):
        n = sub.createNode("null", name)
        n.setInput(0, ins[i])
        if i in INPUT_COLORS:
            ins[i].setColor(hou.Color(INPUT_COLORS[i]))
            n.setColor(hou.Color(INPUT_COLORS[i]))
        nulls.append(n)
    geo_in, con_in, proxy_in = nulls

    check = sub.createNode("error", "warn_if_no_constraints")
    check.setInput(0, con_in)
    check.parm("errormsg1").set("No Constraint Geometry")
    b3d_hda.set_token(check.parm("severity1"), "warn", warn)
    check.parm("enable1").setExpression("nprims(-1) < 1", hou.exprLanguage.Hscript)
    check.addSpareParmTuple(hou.StringParmTemplate(
        "spare_input0", "Spare Input 0", 1, string_type=hou.stringParmType.NodeReference,
        help="The geometry the expression reads as input -1.", tags={"opfilter": "!!SOP!!", "oprelative": "."}))
    check.parm("spare_input0").set(check.relativePathTo(con_in))

    # Filter by Pieces: SideFX's RBD Group Constraints makes the group of the constraints between the
    # pieces. Its output 1 is the constraint geometry with the group.
    picked = check
    try:
        grouper = sub.createNode("rbdgroupconstraints", "group_constraints")
    except hou.OperationFailed:
        warn("the rbdgroupconstraints node does not exist: Filter by Pieces will not work")
    else:
        grouper.setInput(0, geo_in)
        grouper.setInput(1, check)
        grouper.setInput(2, proxy_in)
        for parm, expr in (("inputgroup", 'chs("../constraintgroup")'), ("usegroups", 'ch("../usegroups")'),
                           ("group1", 'chs("../group1")'), ("group2", 'chs("../group2")')):
            if grouper.parm(parm) is None:
                warn(f"{grouper.path()} has no {parm}: Filter by Pieces may not work")
            else:
                grouper.parm(parm).setExpression(expr, hou.exprLanguage.Hscript)
        if grouper.parm("group") is not None:
            grouper.parm("group").set(GROUP_ATTRIB)
        picked = sub.createNode("switch", "pick_constraints")
        picked.setInput(0, check)
        picked.setInput(1, grouper, 1)
        picked.parm("input").setExpression('ch("../groupfromgeo")', hou.exprLanguage.Hscript)

    setter = sub.createNode("attribwrangle", "set_properties")
    setter.setInput(0, picked)
    setter.parm("class").set("detail")
    setter.parm("snippet").set(wrangle_vex(rows))
    # A wrangle, not a Group Delete: that warns on every cook while the group does not exist (Filter by Pieces off).
    strip = sub.createNode("attribwrangle", "strip_group")
    strip.setInput(0, setter)
    strip.parm("class").set("detail")
    strip.parm("snippet").set(f'removeprimgroup(0, "{GROUP_ATTRIB}");')

    for idx, (name, src) in enumerate([("geometry", geo_in), ("constraints", strip), ("proxy", proxy_in)]):
        out = sub.createNode("output", f"OUT_{name}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 1:  # the constraint lines are what this node edits
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    b3d_hda.layout(sub, CONSTRAINT_PROPERTIES_LAYOUT)


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "templates")
    common, per_type = collect(dopnet)
    rows = attribute_rows(common, per_type)
    print("make_constraint_properties_sop: settings: " + ", ".join(sorted({a for a, _, _, _ in rows})))

    geo = hou.node("/obj").createNode("geo", "build")
    for child in geo.children():
        child.destroy()
    sub = geo.createNode("subnet", TYPE_NAME)
    # The parameters first (spare, replaced by the asset's interface below), so the expressions
    # build_network sets find them while the network is built.
    sub.setParmTemplateGroup(ui(common, per_type))
    build_network(sub, rows)

    if os.path.exists(out_path):
        try:
            hou.hda.uninstallFile(out_path)
        except hou.OperationFailed:
            pass
        os.remove(out_path)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    asset = sub.createDigitalAsset(name=TYPE_NAME, hda_file_name=out_path, description=LABEL,
                                   min_num_inputs=0, max_num_inputs=len(LABELS))
    definition = asset.type().definition()
    definition.setParmTemplateGroup(ui(common, per_type))
    definition.setMaxNumOutputs(len(LABELS))
    definition.setIcon(ICON)
    b3d_hda.set_tab_submenu(definition, warn)
    b3d_hda.add_python_module(definition)
    b3d_hda.set_message_nodes(asset, MESSAGE_NODES, warn)
    definition.setComment("Sets Box3D constraint settings as primitive attributes on the constraints in a "
                          "group, like RBD Constraint Properties.")
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    b3d_hda.edit_dialog_script(definition, LABELS, LABELS, warn)  # labels last, saved without the node (see make_solver_sop.py)
    definition.save(out_path)

    probe(out_path)
    print(f"Box3D Constraint Properties asset: {os.path.normpath(out_path)}")


def probe(out_path):
    """On a fresh instance: two Revolute lines, the first in a group. Ticking Stiffness (7) and Enable
    Spring on the Revolute folder and the constraint name writes them on that line only, the other keeps
    the "not set" values."""
    hou.hda.installFile(out_path, force_use_assets=True)
    geo = hou.node("/obj").createNode("geo", "probe_geo")
    for child in geo.children():
        child.destroy()
    lines = geo.createNode("attribwrangle", "lines")
    lines.parm("class").set("detail")
    lines.parm("snippet").set(
        'for (int i = 0; i < 2; i++) { int a = addpoint(0, set(i, 0, 0)), b = addpoint(0, set(i, 1, 0)); '
        'int pr = addprim(0, "polyline", a, b); '
        'if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", ""); '
        'setprimattrib(0, "constraint_name", pr, "Revolute"); '
        'if (i == 0) setprimgroup(0, "chosen", pr, 1); }')
    cfg = geo.createNode(TYPE_NAME, "probe")
    cfg.setInput(1, lines)
    wanted = (("constraintgroup", "chosen"), ("jointtype", 2), ("addrevolute_stiffness", 1),
              ("revolute_stiffness", 7.0), ("addrevolute_enable_spring", 1), ("revolute_enable_spring", 1),
              ("doconstraintname", 1), ("constraintname", "Weld"))
    for n, v in wanted:
        if cfg.parm(n) is None:
            warn(f"the asset has no {n} parameter")
            geo.destroy()
            return
        cfg.parm(n).set(v)
    g = cfg.geometry(1)
    if g is None or g.findPrimAttrib("stiffness") is None:
        warn("probe: the constraint output has no stiffness attribute")
        geo.destroy()
        return
    prims = g.prims()
    got = [(round(p.attribValue("stiffness"), 3), p.attribValue("enable_spring"), p.attribValue("constraint_name"))
           for p in prims]
    unset = prims[1].attribValue("stiffness") <= -3.0e38 and prims[1].attribValue("enable_spring") < 0
    if (len(got) != 2 or got[0] != (7.0, 1, "Weld") or not unset or got[1][2] != "Revolute"):
        warn(f"probe: (stiffness, enable_spring, constraint_name) per line {got}, expected the first "
             "(7, 1, Weld) and the second not set (stiffness below -3e38, enable_spring -1, Revolute)")
    if g.findPrimGroup(GROUP_ATTRIB) is not None:
        warn("probe: the temporary group reached the output")
    geo.destroy()


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 0
    try:
        build(sys.argv[1])
    except Exception:  # noqa: BLE001 - never fail the plugin build over the asset
        warn("the Box3D Constraint Properties asset was not built:\n" + traceback.format_exc())
    return 0


if __name__ == "__main__":
    sys.exit(main())
