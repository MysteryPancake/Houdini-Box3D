"""Box3D Solver SOP: the Constraints tab, the relationship nodes' own settings on the asset.

Each relationship in the DOP network (every Box3D joint type, SideFX Glue, Hard, Spring, Soft and Cone Twist) gets a
closed folder holding its settings, named <relationship>_<setting> (weld_force_threshold). A float primitive
attribute multiplies the node's value, as on the nodes themselves ("Attributes Replace Values" is promoted too).
Each folder also has Guide Scale (the type's constraint lines' share of Guide Radius) and leads with Enable (a Switch in front of the node in the DOP network, off leaves the type out), Data Name
(the node's, which `constraint_name` finds it by) and Default Operation (Set Initial, Always or Never, applied to
every setting the folder drives). World Scale wraps the settings (scale_joint_settings in b3d_solver_network.py)."""

import re

import hou

import b3d_hda
from b3d_hda import collapsible, value_templates, without_parmops
from b3d_solver_common import RELATIONSHIPS, rename_templates, warn

# What a DOP data node has of its own, left out when a node has no parameter folders to take the settings from.
DATA_PARMS = {"activation", "group", "dataname", "data_name", "createobjectifmissing", "constraint_name"}

# Settings whose default on this tab is RBD Bullet Solver's own, not the DOP node's (SideFX's Glue node has
# Strength 10000 and Impulse Half-life 0.1, its SOP has 1 and 1): a `strength` attribute multiplies the setting,
# so a scene set up for one solver breaks at the same impulse in the other.
SOP_DEFAULTS = {
    ("glue", "strength"): 1.0, ("glue", "impulse_halflife"): 1.0,
    ("hard", "cfm"): 1.0, ("hard", "erp"): 1.0,
    ("soft", "stiffness"): 1.0, ("soft", "angularstiffness"): 1.0,
    ("soft", "plasticthreshold"): 1.0, ("soft", "angularplasticthreshold"): 1.0,
    ("spring", "damping"): 1.0,
}
# The same for a folder's Data Name and Enable: RBD Bullet Solver calls the Spring "Spring" (SideFX's node
# "ConRelSpring") and leaves it off.
SOP_DATA_NAMES = {"spring": "Spring"}
SOP_ENABLED = {"spring": False}


def folder_label(node):
    """'Box3D Weld Constraint Relationship' -> 'Weld'."""
    label = node.type().description()
    label = re.sub(r"^Box3D\s+", "", label)
    return re.sub(r"\s+Constraint Relationship$", "", label)


def is_setting(node, pt):
    """A setting the simulation reads: one with a Parameter Operation menu (parmop_<name>). SideFX's guide and
    display parameters (colour, radius, show link, guide_*) have none and are left off."""
    return node.parm(f"parmop_{pt.name()}") is not None and not pt.name().lower().startswith("guide")


def node_settings(node):
    """The value templates holding `node`'s settings: those in its parameter folders (a DOP data node's own
    parameters, Activation and Data Name, sit outside them), else every one but DATA_PARMS. Only the ones
    `is_setting` accepts."""
    entries = without_parmops(node.parmTemplateGroup().entries())
    found = [pt for e in entries if e.type() == hou.parmTemplateType.Folder
             for pt in value_templates(e.parmTemplates()) if is_setting(node, pt)]
    if found:
        return found
    found = [pt for pt in value_templates(entries) if pt.name().lower() not in DATA_PARMS and is_setting(node, pt)]
    if found:
        warn(f"{node.path()} has no parameter folders: promoted every setting but {sorted(DATA_PARMS)}: "
             f"{[pt.name() for pt in found]}")
    return found


def enable_name(name):
    """The Enable toggle of relationship `name` (use_<name> in b3d_solver_dops.py's Switch)."""
    return f"use{name}"


def guide_scale_name(name):
    return f"{name}_guidescale"


def default_op_name(name):
    return f"{name}_defaultparmop"


def head_templates(node, name):
    """Enable, Data Name and Default Operation of relationship `name`, and the links of the last two."""
    data_name = node.parm("dataname")
    default_op = node.parm("defaultparmop")
    items = [hou.ToggleParmTemplate(
        enable_name(name), "Enable", default_value=SOP_ENABLED.get(name, True),
        help="Off leaves this constraint type out of the simulation.")]
    links = []
    if data_name is not None:
        data_name_template = b3d_hda.menu_script(hou.StringParmTemplate(
            f"{name}_dataname", "Data Name", 1, default_value=(SOP_DATA_NAMES.get(name, data_name.eval()),),
            help="The name `constraint_name` uses to pick this type."), "buildConstraintNameMenu(kwargs, 1)",
            toggle=False)
        # One row with Default Operation, as RBD Bullet Solver's
        data_name_template.setJoinWithNext(default_op is not None)
        items.append(data_name_template)
        links.append(("dataname", f"{name}_dataname"))
    else:
        warn(f"{node.path()} has no dataname: no Data Name on its folder")
    if default_op is not None:
        default_op_template = hou.MenuParmTemplate(
            default_op_name(name), "Default Operation", ("0", "1", "2"),
            menu_labels=("Set Initial", "Set Always", "Set Never"), default_value=0,
            help="Set Initial applies the values when a constraint is created. Set Always applies them every step, "
                 "so animated values act. Set Never ignores them.")
        default_op_template.hideLabel(True)
        items.append(default_op_template)
        links.append(("defaultparmop", default_op_name(name)))
    else:
        warn(f"{node.path()} has no defaultparmop: no Default Operation on its folder")
    items.append(hou.FloatParmTemplate(
        guide_scale_name(name), "Guide Scale", 1, default_value=(1.0,), min=0.0, max=10.0, min_is_strict=True,
        help="Multiplies Guide Radius for this type's constraint lines with Show Constraints."))
    return items, links


def constraint_items(dopnet, sub):
    """The Constraints tab's folders and the links [(inner path relative to `sub`, inner name, outer name)]."""
    folders, links = [], []
    for _, name in RELATIONSHIPS:
        node = dopnet.node(name)
        if node is None:
            continue
        found = node_settings(node)
        if not found:
            warn(f"{node.path()}: no settings found, not on the Constraints tab")
            continue
        for pt in found:
            if (name, pt.name()) in SOP_DEFAULTS:
                pt.setDefaultValue((SOP_DEFAULTS[(name, pt.name())],))
        renames = {pt.name(): f"{name}_{pt.name()}" for pt in found}
        head, head_links = head_templates(node, name)
        # Enable is the folder's header checkbox; the rest greys out while it is off.
        contents = head + rename_templates(found, renames)
        for pt in contents[1:]:  # added to a setting's own Disable When, not replacing it
            own = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
            pt.setConditional(hou.parmCondType.DisableWhen, f"{own} {{ {enable_name(name)} == 0 }}".strip())
        folders.append(collapsible(f"b3dsop_con_{name}", folder_label(node), contents, open_folder=False,
                                   header_toggle=enable_name(name)))
        links += [(sub.relativePathTo(node), old, new) for old, new in renames.items()]
        links += [(sub.relativePathTo(node), old, new) for old, new in head_links]
    return folders, links


def finish_constraints(asset, links):
    """Every inner setting the Constraints tab drives follows its folder's Default Operation (a channel reference
    does not switch a DOP data node off Use Default, so each setting's Parameter Operation is set from it: the
    menu's items are initial, always, never, default)."""
    for inner_path, inner_name, outer in links:
        if inner_name in ("dataname", "defaultparmop"):
            continue
        node = asset.node(inner_path)
        menu = node.parm(f"parmop_{inner_name}") if node is not None else None
        if menu is None:
            warn(f"{inner_path} has no parmop_{inner_name}")
            continue
        relationship = inner_path.rsplit("/", 1)[-1]
        menu.setExpression(f'ch("{node.relativePathTo(asset)}/{default_op_name(relationship)}")',
                           hou.exprLanguage.Hscript)
