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
next_constraint_name (what a broken joint switches to, with the settings it then has as next_<setting>: the Next
Constraint folder), and a display colour.

    Inputs / outputs: Geometry, Constraint Geometry (pink), Proxy Geometry, as the Box3D Solver SOP's
    first three. Only the Constraint Geometry is edited; the others pass through (they are what Filter
    by Pieces looks at). IN_constraints -> pick_constraints (or SideFX's RBD Group Constraints with
    Filter by Pieces on) -> set_properties (one detail wrangle) -> strip_group.

Folders: Joint (the settings every type has), then one folder per joint type, showing the one picked
in Constraint Type. A setting is named <type>_<attribute> in a type's folder and writes <attribute>.

Each single-number setting also has a Scaling menu and an Attribute, as RBD Constraint Properties': the value is
multiplied by the constraint's own attribute of the setting's name (Scale by Value) and / or by an attribute taken from
the constraint or from the pieces at its two ends (Scale by Attribute: SCALE_VEX). The Geometry Attribute Transfer folder
says how a constraint's piece is found (Cluster Attribute Name) and combined (Transfer Mode).

Never @ bindings in the wrangle: a binding creates its attribute on every primitive at compile time.

Never fails the build: problems are printed as warnings and the script exits 0.
"""

import os
import re
import sys
import traceback
import zlib

import hou

# No __pycache__ next to the scripts (they are imported from this folder).
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b3d_hda  # noqa: E402  (beside this script)
from b3d_hda import without_parmops  # noqa: E402
from b3d_settings import num  # noqa: E402
from b3d_layouts import CONSTRAINT_PROPERTIES_LAYOUT  # noqa: E402

TYPE_NAME = "box3d_constraint_properties"
LABEL = "Box3D Constraint Properties"
ICON = "DOP_box3d_data"  # installed by the build for Box3D Data (config/Icons)

LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry"]  # inputs and outputs
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}  # constraints pink, as the Box3D Solver SOP

# The joint types in Box3D's order (Box3DJoints.h JointType): DOP node `box3d_<name>conrel`, label.
JOINT_TYPES = [("weld", "Weld"), ("spherical", "Spherical"), ("revolute", "Revolute"),
               ("prismatic", "Prismatic"), ("distance", "Distance"), ("motor", "Motor"),
               ("parallel", "Parallel"), ("wheel", "Wheel"), ("filter", "Filter"),
               ("motionlock", "Motion Lock")]

# Parameters of the relationship nodes that are no primitive attribute (Box3DJoints.h
# jointNodeToggles, kAttributesReplaceName): left out.
NODE_ONLY = {"attributes_replace", "breakable", "set_rest_length", "limit_spring_force",
             "limit_max_length", "limit_torque"}

STATE_NAME = "box3d_constraint_properties"  # no dot: the viewer state's hotkey context is built from it
STATE_FILE = "b3d_constraint_properties_state.py"  # the viewer state's source, beside this script
INDEX_ATTRIB = "__idx"  # each constraint line's number, for the viewport selection and the edits

GROUP_ATTRIB = "__tmpconpropgrp"  # the group SideFX's RBD Group Constraints makes
ROTATE, ROTATE_BOX = "anchorrotate", "addanchorrotate"  # the Anchor Rotation setting (the orient attribute)
SCALING = ("No Scaling", "Scale by Attribute", "Scale by Value", "Scale by Both")  # RBD Constraint Properties' menu
TRANSFER_MODES = (("0", "Closest"), ("1", "Min"), ("3", "Max"), ("2", "Average"))  # its tokens
VEX_DEFAULT = "// Second Input: Geometry\n// Third Input: Proxy Geometry"  # RBD Constraint Properties' placeholder
UNSET_FLOAT = "-3.4028235e38"  # kUnsetFloat (Box3DJoints.h)
UNSET_INT = "-1"

warn = b3d_hda.warner("make_constraint_properties_sop")

# Nodes inside whose errors and warnings show on the asset (b3d_hda.set_message_nodes).
MESSAGE_NODES = ["warn_if_no_constraints"]


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


# The wrangle's functions for a setting scaled by an attribute, as RBD Constraint Properties does it: "Scale by Value"
# multiplies by the constraint's own attribute of the setting's name, "Scale by Attribute" by the named attribute, taken
# from the constraint (a primitive attribute) or else from the pieces at its two ends (point attributes of the Geometry,
# else the Proxy Geometry), averaged over the ends. A piece an end stands for is the one named like it, or, when the
# Geometry has the Cluster Attribute, every piece with that attribute equal to the name, combined by Transfer Mode.
SCALE_VEX = """\
#define NONE -1e38
// The value of `attr` on the piece(s) anchor `name` stands for in geometry `input`: NONE if there is no such piece.
// mode: 0 the piece nearest `P`, 1 Min, 2 Average, 3 Max.
float anchorvalue(int input; string attr; string cluster; string name; vector P; int mode)
{
    string key = "name";
    if (cluster != "" && haspointattrib(input, cluster) && findattribvalcount(input, "point", cluster, name) > 0)
        key = cluster;
    if (!haspointattrib(input, key)) return NONE;
    int n = findattribvalcount(input, "point", key, name);
    float result = NONE;
    float total = 0;
    float nearest = 1e30;
    for (int i = 0; i < n; i++) {
        int pt = findattribval(input, "point", key, name, i);
        float v = point(input, attr, pt);
        total += v;
        if (mode == 0) {
            vector at = point(input, "P", pt);
            float d = distance2(P, at);
            if (d < nearest) { nearest = d; result = v; }
        } else if (i == 0) result = v;
        else if (mode == 1) result = min(result, v);
        else if (mode == 3) result = max(result, v);
    }
    if (n > 0 && mode == 2) result = total / n;
    return result;
}

// `value` of setting `setting` on constraint `pr`, scaled (0 No Scaling, 1 by Attribute `attr`, 2 by Value, 3 Both),
// then randomized: times a number from 1 - variance to 1 + variance, from the constraint's number and `seed`.
float scaled(float base; int scaling; string setting; string attr; int pr; string cluster; int transfer;
             int randomize; float variance; float seed)
{
    float value = base;  // a parameter that is written would have to be given a variable
    if (scaling >= 2 && hasprimattrib(0, setting)) {
        float own = prim(0, setting, pr);
        if (own > -1e37) value *= own;
    }
    if (scaling == 1 || scaling == 3) {
        if (hasprimattrib(0, attr)) value *= prim(0, attr, pr);
        else {
            int input = haspointattrib(1, attr) ? 1 : haspointattrib(2, attr) ? 2 : -1;
            if (input > 0) {
                float total = 0;
                int count = 0;
                foreach (int pt; primpoints(0, pr)) {
                    string anchor = point(0, "name", pt);
                    vector at = point(0, "P", pt);
                    float v = anchorvalue(input, attr, cluster, anchor, at, transfer);
                    if (v > -1e37) { total += v; count++; }
                }
                if (count > 0) value *= total / count;
            }
        }
    }
    if (randomize && value >= 0)
        value *= fit01(rand(pr, seed), max(0, 1 - variance), 1 + variance);
    return value;
}
"""


def condition(pt, prefix, names):
    """`pt`'s disable-when for the asset: each group `{ x == v }` keeps only if every name it reads is a
    setting here (a node toggle such as Breakable is not) and is renamed with `prefix`. In Edit it also needs
    `add<x>` ticked, so it only applies while that setting is being written; in Set the checkboxes are hidden
    and the toggle's value alone counts."""
    cond = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
    out = []
    for body in re.findall(r"\{([^}]*)\}", cond):
        refs = list(dict.fromkeys(re.findall(r"(\w+)\s*[!=<>]=?", body)))
        if any(r not in names for r in refs):
            continue
        for r in refs:
            body = re.sub(rf"\b{r}\b", prefix + r, body)
        extra = "".join(f" add{prefix}{r} == 1" for r in refs)
        out.append("{" + body.rstrip() + extra + " action == 0 }")  # Edit: while that toggle is being written
        out.append("{" + body.rstrip() + " action == 1 }")  # Set: its checkbox is hidden, the toggle's value counts
    return " ".join(out)


def setting_templates(rows, prefix, names, scalable=True, gate=""):
    """The checkbox and value of each setting: add<prefix><name> and <prefix><name>. `scalable`: the number settings
    get their Scaling, Attribute, Randomize and Variance. `gate`: one more disable-when, for the next settings."""
    items = []
    for pt, kind in rows:
        pname = prefix + pt.name()
        value = pt.clone()
        value.setName(pname)
        value.setHelp(f"{b3d_hda.first_sentence(pt.help())} Writes the {pt.name()} primitive attribute.")
        box = hou.ToggleParmTemplate(f"add{pname}", value.label(), default_value=False, join_with_next=True,
                                     help="Write this setting onto the constraints.")
        box.hideLabel(True)
        box.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")  # Set writes every setting of the type
        if gate:
            box.setConditional(hou.parmCondType.DisableWhen, gate)
        gates = condition(pt, prefix, names)
        grey = (gates + " " if gates else "") + f"{{ add{pname} == 0 action == 0 }}" + (" " + gate if gate else "")
        value.setConditional(hou.parmCondType.DisableWhen, grey)
        items += [box, value]
        if kind == "f" and scalable:
            scaling = hou.MenuParmTemplate(
                f"{pname}scaling", f"{value.label()} Scaling", tuple(str(i) for i in range(len(SCALING))), SCALING,
                default_value=0,
                help="Multiply the setting by the constraint's own attribute of the same name (Scale by Value), by "
                     "an attribute (Scale by Attribute), or both. An attribute on the pieces at the two ends is "
                     "averaged (Geometry Attribute Transfer).")
            attrib = hou.StringParmTemplate(
                f"{pname}attribute", f"{value.label()} Attribute", 1, default_value=(pt.name(),),
                help="The attribute that scales the setting: a primitive attribute of the constraints, or a point "
                     "attribute of the pieces.")
            b3d_hda.menu_script(attrib, "buildScaleAttribMenu(kwargs)", toggle=False)
            # On the value's row without its label, as RBD Constraint Properties'
            value.setJoinWithNext(True)
            scaling.hideLabel(True)
            scaling.setConditional(hou.parmCondType.DisableWhen, grey)
            attrib.setConditional(hou.parmCondType.DisableWhen, grey)
            attrib.setConditional(hou.parmCondType.HideWhen, f"{{ {pname}scaling == 0 }} {{ {pname}scaling == 2 }}")
            rand_box = hou.ToggleParmTemplate(
                f"{pname}randomize", f"Randomize {value.label()}", default_value=False, join_with_next=True,
                help="Vary the setting from constraint to constraint, by up to the Variance either way.")
            rand_box.hideLabel(True)
            variance = hou.FloatParmTemplate(
                f"{pname}variance", f"{value.label()} Variance", 1, default_value=(0.8,), min=0.0, max=1.0,
                help="How far a randomized setting goes from its value: 0.8 gives 0.2 to 1.8 times it.")
            rand_box.setConditional(hou.parmCondType.DisableWhen, grey)
            variance.setConditional(hou.parmCondType.DisableWhen, grey + f" {{ {pname}randomize == 0 }}")
            items += [scaling, attrib, rand_box, variance]
    return items


def vex_templates(name):
    """Use VEXpressions and its snippet, as RBD Constraint Properties' types have: a VEX wrangle over the constraints
    in the group that runs after the settings are written, with the Geometry and Proxy Geometry as its second and
    third inputs. Only the type picked in Constraint Type runs."""
    use = hou.ToggleParmTemplate(
        f"{name}_uselocalproperties", "Use VEXpressions", default_value=False,
        help="Run the VEX below over the constraints in the group, after the settings are written.")
    snippet = hou.StringParmTemplate(
        f"{name}_localexpression", "VEXpression", 1, default_value=(VEX_DEFAULT,),
        help="Runs over each constraint (primitive), with the Constraint Geometry as its first input, the Geometry as "
             "its second and the Proxy Geometry as its third.",
        tags={"editor": "1", "editorlang": "vex"})
    snippet.setConditional(hou.parmCondType.HideWhen, f"{{ {name}_uselocalproperties == 0 }}")
    snippet.setItemGeneratorScript("import vexpressionmenu\n\nreturn vexpressionmenu.buildSnippetMenu("
                                   f"'box3d_constraint_properties/{name}localexpression', kwargs=kwargs)")
    snippet.setItemGeneratorScriptLanguage(hou.scriptLanguage.Python)
    snippet.setMenuType(hou.menuType.Normal)
    return [use, snippet]


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
    b3d_hda.group_menu(cgroup, 1, "prim")
    bypieces = hou.ToggleParmTemplate(
        "groupfromgeo", "Filter by Pieces", default_value=False,
        help="Edit the constraints between the pieces in Group 1 and Group 2 instead of a Constraint Group.")
    pieces = [
        # An integer with menu items, as RBD Constraint Properties' (not a Menu parameter).
        hou.IntParmTemplate("usegroups", "Groups", 1, default_value=(0,), menu_items=("0", "1"),
                            menu_labels=("All to Group", "Group to Group"),
                            help="Group 1 to every piece, or Group 1 to Group 2.",
                            tags={"autoscope": "0000000000000000"}),
        b3d_hda.group_menu(hou.StringParmTemplate("group1", "Group 1", 1, help="The pieces on one end.",
                                                  tags=group_tag(0))),
        b3d_hda.group_menu(hou.StringParmTemplate("group2", "Group 2", 1, help="The pieces on the other end.",
                                                  tags=group_tag(0))),
    ]
    pieces[0].setConditional(hou.parmCondType.DisableWhen, "{ groupfromgeo == 0 }")
    pieces[1].setConditional(hou.parmCondType.DisableWhen, "{ groupfromgeo == 0 }")
    pieces[2].setConditional(hou.parmCondType.DisableWhen, "{ usegroups < 1 } { groupfromgeo == 0 }")
    enable_color = hou.ToggleParmTemplate("enablecolor", "Enable Color", default_value=False, join_with_next=True,
                                          help="Colour the edited constraints.")
    enable_color.hideLabel(True)  # joined to Color, as RBD Constraint Properties'
    color = hou.FloatParmTemplate("constraintcolor", "Color", 3, default_value=(1.0, 1.0, 1.0),
                                  look=hou.parmLook.ColorSquare, naming_scheme=hou.parmNamingScheme.RGBA,
                                  help="The colour, written as Cd.")
    color.setConditional(hou.parmCondType.DisableWhen, "{ enablecolor == 0 }")

    def string_row(name, label, help_text, always_in_set=False, items=None, labels=None, gate=""):
        """A checkbox and a string with the types' Data Names (or `items`) in its menu. In Set mode the value is
        always written (`always_in_set`) and its checkbox hidden. `gate`: one more disable-when."""
        items = names_menu if items is None else items
        labels = items if labels is None else labels
        box = hou.ToggleParmTemplate(f"do{name}", label, default_value=False, join_with_next=True,
                                     help="Write this onto the constraints.")
        box.hideLabel(True)
        value = hou.StringParmTemplate(name, label, 1, default_value=(items[0],), menu_items=items,
                                       menu_labels=labels, menu_type=hou.menuType.StringReplace,
                                       help=help_text)
        extra = " " + gate if gate else ""
        if always_in_set:
            box.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")
            value.setConditional(hou.parmCondType.DisableWhen, f"{{ do{name} == 0 action == 0 }}" + extra)
        else:
            value.setConditional(hou.parmCondType.DisableWhen, f"{{ do{name} == 0 }}" + extra)
        if gate:
            box.setConditional(hou.parmCondType.DisableWhen, gate)
        return [box, value]

    # The Next Constraint folder is off until Switch Constraint Type When Broken is Enable (and, in Edit mode, ticked)
    off = "{ usenextconstraint == 0 } { action == 0 dousenextconstraint == 0 }"
    edit_off = "{ action == 0 donext_constrainttype == 0 }"  # Next Constraint Type's own tick, in Edit mode
    type_off = "{ action == 0 doconstrainttype == 0 }"  # Constraint Type's tick, in Edit mode

    which = hou.MenuParmTemplate("constrainttype", "Constraint Type", tuple(str(i) for i in range(len(JOINT_TYPES))),
                                 tuple(names_menu), default_value=0,
                                 help="Which joint type's settings to show. Every ticked setting of the type is written.")
    which.setConditional(hou.parmCondType.DisableWhen, type_off)
    # RBD Constraint Properties' tick in front of the menu: Edit mode writes the type's settings only while it is ticked.
    do_type = hou.ToggleParmTemplate(
        "doconstrainttype", "Constraint Type", default_value=False, join_with_next=True,
        help="Write the ticked settings of the Constraint Type's joint type.")
    do_type.hideLabel(True)
    do_type.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")
    which.setJoinWithNext(False)
    # Disable Collisions, in RBD Constraint Properties' words: it writes collide_connected, the Joint folder's Collide
    # Connected, the other way round (Disable is 0). It is written after the Joint folder's, so it wins if both are ticked.
    collide_box = hou.ToggleParmTemplate(
        "docollisions", "Disable Collisions", default_value=False, join_with_next=True,
        help="Write whether the two joined bodies collide. Writes the collide_connected primitive attribute.")
    collide_box.hideLabel(True)
    collide_box.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")
    collisions = hou.MenuParmTemplate(
        "collisions", "Collisions", ("0", "1"), ("Disable", "Enable"), default_value=0,
        help="Whether the two joined bodies collide with each other: Disable stops them.")
    collisions.setConditional(hou.parmCondType.DisableWhen, "{ docollisions == 0 action == 0 }")
    props = (string_row("constraintname", "Constraint Name",
                        "The relationship's Data Name the constraints use. Writes constraint_name.", True)
             + [collide_box, collisions, do_type, which])

    common_names = {pt.name() for pt, _ in common}
    # Anchor Rotation: the frame of both anchors, which the viewport handle also edits (Edit Parms).
    rotate_box = hou.ToggleParmTemplate(ROTATE_BOX, "Anchor Rotation", default_value=False, join_with_next=True,
                                        help="Write this setting onto the constraints.")
    rotate_box.hideLabel(True)
    rotate = hou.FloatParmTemplate(ROTATE, "Anchor Rotation", 3, default_value=(0.0, 0.0, 0.0),
                                   help="Rotates the frames of both anchors, in degrees. Writes the orient point "
                                        "attribute, which sets the axes of a joint. The viewport handle edits it "
                                        "with Edit Parms on.")
    rotate.setConditional(hou.parmCondType.DisableWhen, f"{{ {ROTATE_BOX} == 0 }}")
    folders = [b3d_hda.collapsible("b3dcp_common", "Joint",
                                   [rotate_box, rotate] + setting_templates(common, "", common_names))]
    for i, (name, label) in enumerate(JOINT_TYPES):
        rows = per_type.get(name, [])
        if not rows:
            continue
        own = {pt.name() for pt, _ in rows}
        f = b3d_hda.collapsible(f"b3dcp_{name}", label, setting_templates(rows, f"{name}_", own) + vex_templates(name))
        f.setConditional(hou.parmCondType.HideWhen, f"{{ constrainttype != {i} }}")
        f.setConditional(hou.parmCondType.DisableWhen, type_off)
        folders.append(f)

    # Switch Constraint Type When Broken: what a broken constraint becomes (its next_constraint_name) and the settings it
    # then has (next_<setting>), for the type in Next Constraint Type. The solver applies them when the constraint
    # switches.
    do_use_next = hou.ToggleParmTemplate(
        "dousenextconstraint", "Switch Constraint Type When Broken", default_value=False, join_with_next=True,
        help="Write the Switch Constraint Type When Broken choice.")
    do_use_next.hideLabel(True)
    do_use_next.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")
    use_next = hou.MenuParmTemplate(
        "usenextconstraint", "Switch Constraint Type When Broken", ("0", "1"), ("Disable", "Enable"), default_value=0,
        help="Enable: a constraint that breaks becomes the Next Constraint instead, with the next settings below.")
    use_next.setConditional(hou.parmCondType.DisableWhen, "{ action == 0 dousenextconstraint == 0 }")
    next_type = hou.MenuParmTemplate(
        "next_constrainttype", "Next Constraint Type", tuple(str(i) for i in range(len(JOINT_TYPES))),
        tuple(names_menu), default_value=0,
        help="Which joint type's next settings to show. Every ticked next setting is written.")
    next_type.setConditional(hou.parmCondType.DisableWhen, off + " " + edit_off)
    do_next_type = hou.ToggleParmTemplate(
        "donext_constrainttype", "Next Constraint Type", default_value=False, join_with_next=True,
        help="Write the ticked next settings of the Next Constraint Type's joint type.")
    do_next_type.hideLabel(True)
    do_next_type.setConditional(hou.parmCondType.HideWhen, "{ action == 1 }")
    do_next_type.setConditional(hou.parmCondType.DisableWhen, off)
    next_items = ([do_use_next, use_next]
                  + string_row("next_constraintname", "Next Constraint Name",
                               "What a broken constraint switches to. Writes next_constraint_name.", True, gate=off)
                  + [do_next_type, next_type])
    next_folders = [b3d_hda.collapsible("b3dcp_next_common", "Joint",
                                        setting_templates(common, "next_", common_names, scalable=False, gate=off))]
    for i, (name, label) in enumerate(JOINT_TYPES):
        rows = per_type.get(name, [])
        if not rows:
            continue
        own = {pt.name() for pt, _ in rows}
        f = b3d_hda.collapsible(f"b3dcp_next_{name}", label,
                                setting_templates(rows, f"next_{name}_", own, scalable=False, gate=off),
                                open_folder=False)
        f.setConditional(hou.parmCondType.HideWhen, f"{{ next_constrainttype != {i} }}")
        f.setConditional(hou.parmCondType.DisableWhen, edit_off)
        next_folders.append(f)
    next_folder = b3d_hda.collapsible("b3dcp_next", "Next Constraint", next_items + next_folders, open_folder=False)

    # The viewport bar (export level 3), as RBD Constraint Properties' three.
    clear = hou.ButtonParmTemplate(
        "clearedits", "Clear Edits", script_callback="kwargs['node'].hdaViewerStateModule().clearEdits(kwargs)",
        script_callback_language=hou.scriptLanguage.Python, join_with_next=True,
        help="Forgets every edit made in the viewport.")
    importing = hou.ButtonParmTemplate(
        "importedits", "Import Edits", script_callback="kwargs['node'].hdaViewerStateModule().importEdits(kwargs)",
        script_callback_language=hou.scriptLanguage.Python,
        help="Copies the selected constraints' current anchor frames into the edits.")
    edit_parms = hou.ToggleParmTemplate(
        "editparms", "Edit Parms", default_value=False,
        help="The viewport handle edits Anchor Rotation, for every constraint in the group, instead of the selected "
             "constraints' own frames.")
    b3d_hda.show_in_toolbar(clear, importing, edit_parms)
    viewer = hou.FolderParmTemplate("b3dcp_viewer", "Viewer State", [clear, importing, edit_parms],
                                    folder_type=hou.folderType.Simple, tags={"group_type": "simple"})
    guide = hou.FloatParmTemplate(
        "guidescale", "Guide Scale", 1, default_value=(0.0,), min=0.0, max=1.0, min_is_strict=True,
        help="The size of the anchor frames drawn in the viewport while the node is selected. 0 hides them.",
        tags={"autoscope": "0000000000000000"})
    action = hou.MenuParmTemplate(
        "action", "Action", ("0", "1"), ("Edit", "Set"), default_value=1,
        help="Edit writes the ticked settings. Set writes the settings of the joint type in Constraint Type that "
             "differ from the defaults.")
    # The viewport selection (the line number and its group pattern) and the edits, in a folder as RBD Constraint
    # Properties' HandleParms. The selection is not saved with the scene.
    handle = hou.FolderParmTemplate(
        "b3dcp_handleparms", "HandleParms",
        [hou.IntParmTemplate("__idx", "Index", 1, default_value=(-1,), tags={"sidefx::save_value": "0"}),
         hou.StringParmTemplate("__sel", "Selection", 1, tags={"sidefx::save_value": "0"}),
         hou.DataParmTemplate("stash", "Geometry Stash", 1, data_parm_type=hou.dataParmType.Geometry)],
        folder_type=hou.folderType.Simple, tags={"group_type": "simple"})

    group = hou.ParmTemplateGroup()
    group.append(cgroup)
    group.append(bypieces)
    group.append(b3d_hda.collapsible("b3dcp_pieces", "Filter by Pieces", pieces, open_folder=False,
                                     header_toggle="groupfromgeo"))
    group.append(enable_color)
    group.append(color)
    group.append(guide)
    group.append(action)
    group.append(viewer)
    group.append(handle)
    group.append(b3d_hda.collapsible("b3dcp_properties", "Constraint Properties", props))
    for f in folders:
        group.append(f)
    group.append(next_folder)
    cluster = hou.StringParmTemplate(
        "clusterattrib", "Cluster Attribute Name", 1, default_value=("parentpiece",),
        help="The attribute of the Geometry pieces that names the constraint piece each one belongs to, when the "
             "constraints were made on larger pieces. Pieces without it are matched by their name.")
    b3d_hda.attrib_menu(cluster, 0, "point prim", toggle=False)
    mode = hou.MenuParmTemplate(
        "transfermode", "Transfer Mode", tuple(t for t, _ in TRANSFER_MODES), tuple(l for _, l in TRANSFER_MODES),
        default_value=0,
        help="Which of a constraint's pieces gives its scaling attribute when it stands for several (Cluster "
             "Attribute Name): the one closest to it, or the smallest, largest or average value.")
    group.append(b3d_hda.collapsible("b3dcp_transfer", "Geometry Attribute Transfer", [cluster, mode],
                                     open_folder=False))
    return group


def attribute_rows(common, per_type):
    """(attribute, kind, value parameter, checkbox parameter, joint type number, default, next) of every setting,
    then of every next setting (attribute next_<setting>, parameters next_...); the number is -1 for the Joint
    folder's (every type), the default the node's, as a VEX literal."""
    def default(pt, kind):
        values = pt.defaultValue()
        if isinstance(values, bool):  # a toggle's default is no tuple
            values = (int(values),)
        if kind == "v":
            return "{" + ", ".join(num(float(v)) for v in values) + "}"
        return num(float(values[0])) if kind == "f" else str(int(values[0]))

    out = [(pt.name(), kind, pt.name(), f"add{pt.name()}", -1, default(pt, kind), False) for pt, kind in common]
    for i, (name, _) in enumerate(JOINT_TYPES):
        out += [(pt.name(), kind, f"{name}_{pt.name()}", f"add{name}_{pt.name()}", i, default(pt, kind), False)
                for pt, kind in per_type.get(name, [])]
    out += [(f"next_{pt.name()}", kind, f"next_{pt.name()}", f"addnext_{pt.name()}", -1, default(pt, kind), True)
            for pt, kind in common]
    for i, (name, _) in enumerate(JOINT_TYPES):
        out += [(f"next_{pt.name()}", kind, f"next_{name}_{pt.name()}", f"addnext_{name}_{pt.name()}", i,
                 default(pt, kind), True) for pt, kind in per_type.get(name, [])]
    return out


def wrangle_vex(rows):
    """The two wranglers that write the settings: (creating, setting). Each setting that applies (ticked in Edit; every
    one of the type in Constraint Type in Set) gets its attribute first, at the "not set" value so the other lines
    keep the node's value (`creating`, a detail wrangle that only makes the attributes), then the constraints in the
    group get the values (`setting`, over the constraints in the group, in parallel)."""
    unset = {"i": UNSET_INT, "f": UNSET_FLOAT, "v": "{" + UNSET_FLOAT + ", 0.0, 0.0}"}
    read = {"i": "chi", "f": "chf", "v": "chv"}
    # (condition, attribute, "not set" value, value expression) of every primitive attribute that can be written
    items = [
        ('chi("../enablecolor")', "Cd", "{1.0, 1.0, 1.0}", 'chv("../constraintcolor")'),
        ('setmode || chi("../doconstraintname")', "constraint_name", '""', 'chs("../constraintname")'),
        ('usenext && (setmode || chi("../donext_constraintname"))', "next_constraint_name", '""',
         'chs("../next_constraintname")'),
    ]
    for attr, kind, value, box, joint, dflt, following in rows:
        # Set writes the settings of the type that differ from the node's default; the rest stay "not set".
        differs = f'{read[kind]}("../{value}") != {dflt}'
        expression = f'{read[kind]}("../{value}")'
        if kind == "f" and not following:  # Set also writes a setting that is scaled, whatever its value
            differs = f'({differs} || chi("../{value}scaling") != 0 || chi("../{value}randomize") != 0)'
            seed = zlib.crc32(value.encode()) % 10007  # each setting its own random numbers
            expression = (f'scaled(chf("../{value}"), chi("../{value}scaling"), "{attr}", chs("../{value}attribute"), '
                          f'i@primnum, clusterattr, transfermode, chi("../{value}randomize"), chf("../{value}variance"), '
                          f"{seed}.0)")
        which = "nexttype" if following else "constrainttype"
        if joint < 0:
            applies = f'(setmode && {differs}) || (!setmode && chi("../{box}"))'
        else:  # a type's setting: in Edit mode also its type's tick
            tick = "donext_constrainttype" if following else "doconstrainttype"
            applies = f'(setmode && {which} == {joint} && {differs}) || (!setmode && chi("../{tick}") && chi("../{box}"))'
        if following:  # a next setting is only written while the constraint switches when broken
            applies = f"usenext && ({applies})"
        items.append((applies, attr, unset[kind], expression))
    # Disable Collisions: Disable 0, Enable 1. Set writes it only when it is Enable (Disable is the joint nodes' default).
    items.append(('chi("../docollisions") || (setmode && chi("../collisions") != 0)', "collide_connected", UNSET_INT,
                  'chi("../collisions")'))

    header = [
        'int setmode = chi("../action");',
        'int constrainttype = chi("../constrainttype");',
        'int usenext = chi("../usenextconstraint") && (setmode || chi("../dousenextconstraint"));',
        'int nexttype = chi("../next_constrainttype");',
        'string clusterattr = chs("../clusterattrib");',
        'int transfermode = chi("../transfermode");',
        "",
    ]
    # The frame of both anchors, from Anchor Rotation (degrees): a point attribute on the anchors.
    rotate = f'chi("../{ROTATE_BOX}")'
    creating = ["// Box3D Constraint Properties: the attributes of each setting that applies, \"not set\" until written."] + header
    creating.append(f'if ({rotate}) {{ if (!haspointattrib(0, "orient")) addpointattrib(0, "orient", {{0.0, 0.0, 0.0, 1.0}}); }}')
    for condition, attr, default, _ in items:
        creating.append(f'if ({condition}) {{ if (!hasprimattrib(0, "{attr}")) addprimattrib(0, "{attr}", {default}); }}')
    setting = [SCALE_VEX,
               "// Box3D Constraint Properties: each setting that applies is written onto a constraint of the group."] + header
    setting.append(f'if ({rotate}) {{')
    setting.append(f'    vector4 q = eulertoquaternion(radians(chv("../{ROTATE}")), 0);')
    setting.append('    foreach (int pt; primpoints(0, i@primnum)) setpointattrib(0, "orient", pt, q);')
    setting.append("}")
    for condition, attr, _, value in items:
        setting.append(f'if ({condition}) setprimattrib(0, "{attr}", i@primnum, {value});')
    return "\n".join(creating), "\n".join(setting)


APPLY_EDITS_VEX = """\
// The viewport edits, over each line: each stash point holds a line's number and what was edited on it. An orient of all
// zeros, an enable_motor below 0 and a motor_velocity below -1e30 are "not edited". The attributes are there already.
int idx = prim(0, "%(idx)s", i@primnum);  // typed: prim() alone is ambiguous for findattribval
int p = findattribval(1, "point", "%(idx)s", idx);
int pts[] = primpoints(0, i@primnum);
if (p < 0 || len(pts) < 2) return;
vector4 qa = point(1, "orient_a", p);
vector4 qb = point(1, "orient_b", p);
if (dot(qa, qa) > 0 && dot(qb, qb) > 0) {
    setpointattrib(0, "orient", pts[0], qa);
    setpointattrib(0, "orient", pts[1], qb);
}
int motor = point(1, "enable_motor", p);
if (motor >= 0) setprimattrib(0, "enable_motor", i@primnum, motor);
vector velocity = point(1, "motor_velocity", p);
if (velocity.x > -1e30) setprimattrib(0, "motor_velocity", i@primnum, velocity);
""" % {"idx": INDEX_ATTRIB}

# The first selected line's handle: at the middle of its two anchors, in the frame of the first (or Anchor Rotation).
HANDLE_VEX = """\
if (nprimitives(0) < 1) { 4@xform = ident(); return; }
int pts[] = primpoints(0, 0);
if (len(pts) < 2) { 4@xform = ident(); return; }
vector4 q = {0.0, 0.0, 0.0, 1.0};
if (chi("../editparms")) q = eulertoquaternion(radians(chv("../%(rot)s")), 0);
else if (haspointattrib(0, "orient")) q = point(0, "orient", pts[0]);
matrix3 r = qconvert(q);
matrix m = (matrix)r;
vector a = point(0, "P", pts[0]);
vector b = point(0, "P", pts[1]);
vector P = (a + b) * 0.5;
setcomp(m, P.x, 3, 0);
setcomp(m, P.y, 3, 1);
setcomp(m, P.z, 3, 2);
4@xform = m;
""" % {"rot": ROTATE}

# The frames of both anchors of every line in the group, as three coloured lines each: a Copy to Points of a frame
# (three lines, red, green, blue) onto the lines' points, turned by their `orient` and sized by `pscale`, which is a
# quarter of the line's length times Guide Scale (as RBD Constraint Properties' guides are copied onto the points).
GUIDE_SIZE_VEX = """\
int pts[] = primpoints(0, i@primnum);
vector a = {0, 0, 0}, b = {0, 0, 0};
if (len(pts) > 1) {
    a = point(0, "P", pts[0]);
    b = point(0, "P", pts[1]);
}
f@pscale = len(pts) < 2 ? 0.0 : chf("../guidescale") * 0.25 * distance(a, b);
"""

# Only the lines that have an edit in the stash.
GLOW_VEX = """\
int idx = prim(0, "%(idx)s", i@primnum);
if (findattribval(1, "point", "%(idx)s", idx) < 0) removeprim(0, i@primnum, 1);
""" % {"idx": INDEX_ATTRIB}


def viewer_network(sub, edited, stash):
    """The nodes the viewer state reads (b3d_constraint_properties_state.py): CONSTRAINT (the selected lines),
    HANDLE (the handle's transform), GUIDES (the anchor frames), GLOW (the edited lines) and `widgets` (what the
    selector picks from, a viewer-only output). None is wired to a real output."""
    group = 'ifs(ch("../groupfromgeo"), "{}", chs("../constraintgroup"))'.format(GROUP_ATTRIB)
    lines = sub.createNode("blast", "edit_lines")  # the lines in the Constraint Group: the ones the state edits
    lines.setInput(0, edited)
    lines.parm("group").setExpression(group, hou.exprLanguage.Hscript)
    lines.parm("grouptype").set("prims")
    lines.parm("negate").set(1)

    picked = sub.createNode("blast", "selected_lines")  # the ones picked in the viewport
    picked.setInput(0, lines)
    picked.parm("group").setExpression(
        'ifs(strlen(chs("../__sel")) > 0, chs("../__sel"), "@{}=-2")'.format(INDEX_ATTRIB), hou.exprLanguage.Hscript)
    picked.parm("grouptype").set("prims")
    picked.parm("negate").set(1)
    sub.createNode("null", "CONSTRAINT").setInput(0, picked)

    xform = sub.createNode("attribwrangle", "get_xform")
    xform.setInput(0, picked)
    xform.parm("class").set("detail")
    xform.parm("snippet").set(HANDLE_VEX)
    sub.createNode("null", "HANDLE").setInput(0, xform)

    size = sub.createNode("attribwrangle", "guide_size")
    size.setInput(0, lines)
    size.parm("class").set("primitive")
    size.parm("snippet").set(GUIDE_SIZE_VEX)
    to_points = sub.createNode("attribpromote", "guide_pscale")
    to_points.setInput(0, size)
    b3d_hda.set_token(to_points.parm("inclass"), "primitive", warn)
    b3d_hda.set_token(to_points.parm("outclass"), "point", warn)
    b3d_hda.set_token(to_points.parm("method"), "max", warn)
    to_points.parm("inname").set("pscale")
    frame = sub.createNode("merge", "guide_frame")
    for axis, (name, colour) in enumerate((("x", (1.0, 0.0, 0.0)), ("y", (0.0, 1.0, 0.0)), ("z", (0.0, 0.0, 1.0)))):
        line = sub.createNode("line", f"guide_axis_{name}")
        line.parmTuple("dir").set(tuple(1.0 if k == axis else 0.0 for k in range(3)))
        line.parm("dist").set(1.0)
        line.parm("points").set(2)
        tint = sub.createNode("color", f"guide_color_{name}")
        tint.setInput(0, line)
        b3d_hda.set_token(tint.parm("class"), "primitive", warn)
        tint.parmTuple("color").set(colour)
        frame.setInput(axis, tint)
    copies = sub.createNode("copytopoints::2.0", "guide_frames")
    copies.setInput(0, frame)
    copies.setInput(1, to_points)
    nothing = sub.createNode("null", "no_guides")
    guides = sub.createNode("switch", "switch_guides")  # Guide Scale 0 draws nothing
    guides.setInput(0, nothing)
    guides.setInput(1, copies)
    guides.parm("input").setExpression('ch("../guidescale") > 0', hou.exprLanguage.Hscript)
    sub.createNode("null", "GUIDES").setInput(0, guides)

    glow = sub.createNode("attribwrangle", "glow_lines")
    glow.setInput(0, lines)
    glow.setInput(1, stash)
    glow.parm("class").set("primitive")
    glow.parm("snippet").set(GLOW_VEX)
    sub.createNode("null", "GLOW").setInput(0, glow)

    # The selector picks lines, and a pick of a line's point or edge finds its number on the points too.
    promote = sub.createNode("attribpromote", "number_points")
    promote.setInput(0, lines)
    promote.parm("inname").set(INDEX_ATTRIB)
    promote.parm("inclass").set("primitive")
    promote.parm("outclass").set("point")
    widgets = sub.createNode("output", "widgets")
    widgets.setInput(0, promote)
    widgets.parm("outputidx").set(-1)


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

    # Every line gets its number, for the viewport selection and the edits (removed again before the output).
    index = sub.createNode("enumerate", "index_lines")
    index.setInput(0, con_in)
    index.parm("attribname").set(INDEX_ATTRIB)
    check = sub.createNode("error", "warn_if_no_constraints")
    check.setInput(0, index)
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

    creating_vex, setting_vex = wrangle_vex(rows)
    creator = sub.createNode("attribwrangle", "create_settings")
    creator.setInput(0, picked)
    creator.parm("class").set("detail")
    creator.parm("snippet").set(creating_vex)
    setter = sub.createNode("attribwrangle", "set_properties")
    setter.setInput(0, creator)
    setter.setInput(1, geo_in)  # the pieces a scaling attribute can come from
    setter.setInput(2, proxy_in)
    setter.parm("class").set("primitive")
    setter.parm("group").setExpression(f'ifs(ch("../groupfromgeo"), "{GROUP_ATTRIB}", chs("../constraintgroup"))',
                                       hou.exprLanguage.Hscript)
    b3d_hda.set_token(setter.parm("grouptype"), "prims", warn)
    setter.parm("snippet").set(setting_vex)
    # Use VEXpressions: the snippet of the type in Constraint Type, run over the constraints in the group after the
    # settings, when that type's toggle is on (a Switch passes the settings straight through otherwise).
    vexer = sub.createNode("attribwrangle", "run_vexpressions")
    vexer.setInput(0, setter)
    vexer.setInput(1, geo_in)
    vexer.setInput(2, proxy_in)
    vexer.parm("class").set("primitive")
    vexer.parm("group").setExpression(f'ifs(ch("../groupfromgeo"), "{GROUP_ATTRIB}", chs("../constraintgroup"))',
                                      hou.exprLanguage.Hscript)
    b3d_hda.set_token(vexer.parm("grouptype"), "prims", warn)
    snippet, enabled = '""', "0"
    with_settings = {r[4] for r in rows if r[4] >= 0}  # the types that have a folder, so a VEXpression (Filter has none)
    for i in reversed(range(len(JOINT_TYPES))):
        if i not in with_settings:
            continue
        name = JOINT_TYPES[i][0]
        snippet = f'ifs(ch("../constrainttype") == {i}, chs("../{name}_localexpression"), {snippet})'
        enabled = f'if(ch("../constrainttype") == {i}, ch("../{name}_uselocalproperties"), {enabled})'
    vexer.parm("snippet").setExpression(snippet, hou.exprLanguage.Hscript)
    use_vex = sub.createNode("switch", "switch_vexpressions")
    use_vex.setInput(0, setter)
    use_vex.setInput(1, vexer)
    use_vex.parm("input").setExpression(enabled, hou.exprLanguage.Hscript)
    # The viewport edits (the node's stash: one point per edited line) go on top of what the settings write.
    stash = sub.createNode("stash", "stash")
    stash.parm("stash").set(sub.parm("stash"))
    # Only while something is edited (the stash has points): the attributes the edits write, then the wrangle over the
    # lines, as RBD Constraint Properties' copy_motors, then the lines without any of it otherwise.
    point_attribs = b3d_hda.attribute_create(sub, "edit_point_attribs", [("orient", "float", None)], warn, write=False)
    point_attribs.setInput(0, use_vex)
    point_attribs.parm("size1").set(4)
    for c, value in enumerate((0.0, 0.0, 0.0, 1.0), 1):
        point_attribs.parm(f"default1v{c}").set(value)
    b3d_hda.set_token(point_attribs.parm("typeinfo1"), "quaternion", warn)
    line_attribs = b3d_hda.attribute_create(sub, "edit_line_attribs", [("enable_motor", "int", None),
                                                                      ("motor_velocity", "vector", None)],
                                            warn, cls="primitive", write=False)
    line_attribs.setInput(0, point_attribs)
    line_attribs.parm("default1v1").set(-1)
    for c, value in enumerate((float(UNSET_FLOAT), 0.0, 0.0), 1):
        line_attribs.parm(f"default2v{c}").set(value)
    apply_edits = sub.createNode("attribwrangle", "apply_edits")
    apply_edits.setInput(0, line_attribs)
    apply_edits.setInput(1, stash)
    apply_edits.parm("class").set("primitive")
    apply_edits.parm("snippet").set(APPLY_EDITS_VEX)
    edits = sub.createNode("switch", "switch_edits")
    edits.setInput(0, use_vex)
    edits.setInput(1, apply_edits)
    edits.parm("input").setExpression(
        'npoints("../stash") > 0', hou.exprLanguage.Hscript)
    # A wrangle, not a Group Delete: that warns on every cook while the group does not exist (Filter by Pieces off).
    strip = sub.createNode("attribwrangle", "strip_group")
    strip.setInput(0, edits)
    strip.parm("class").set("detail")
    strip.parm("snippet").set(f'removeprimgroup(0, "{GROUP_ATTRIB}");')
    strip_index = sub.createNode("attribdelete", "strip_index")
    strip_index.setInput(0, strip)
    strip_index.parm("primdel").set(INDEX_ATTRIB)
    viewer_network(sub, edits, stash)

    for idx, (name, src) in enumerate([("geometry", geo_in), ("constraints", strip_index), ("proxy", proxy_in)]):
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

    geo = hou.node("/obj").createNode("geo", "build")
    for child in geo.children():
        child.destroy()
    sub = geo.createNode("subnet", TYPE_NAME)
    # The parameters first (spare, replaced by the asset's interface below), so the expressions
    # build_network sets find them while the network is built.
    sub.setParmTemplateGroup(ui(common, per_type))
    build_network(sub, rows)

    asset = b3d_hda.create_asset(sub, out_path, TYPE_NAME, LABEL, 0, len(LABELS))
    definition = asset.type().definition()
    definition.setParmTemplateGroup(ui(common, per_type))
    definition.setMaxNumOutputs(len(LABELS))
    definition.setIcon(ICON)
    b3d_hda.set_tab_submenu(definition, warn)
    b3d_hda.add_python_module(definition)
    b3d_hda.add_viewer_state(definition, STATE_NAME, STATE_FILE)
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
    wanted = (("action", 0), ("constraintgroup", "chosen"), ("doconstrainttype", 1), ("constrainttype", 2),
              ("addrevolute_stiffness", 1),
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
