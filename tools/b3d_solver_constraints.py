"""Box3D Solver SOP: the Constraints tab, the relationship nodes' own settings on the asset.

Each relationship in the DOP network (every Box3D joint type, SideFX Glue, Hard, Spring, Soft and Cone Twist) gets a
closed folder holding its settings, named <relationship>_<setting> (weld_force_threshold). A float primitive
attribute multiplies the node's value, as on the nodes themselves ("Attributes Replace Values" is promoted too).
The links are Set Initial on the inner nodes, as the ground's Box3D Data, and World Scale wraps them
(scale_joint_settings in b3d_solver_network.py)."""

import re

import hou

from b3d_hda import collapsible, set_operation, value_templates, without_parmops
from b3d_solver_common import RELATIONSHIPS, rename_templates, warn

# What a DOP data node has of its own, left out when a node has no parameter folders to take the settings from.
DATA_PARMS = {"activation", "group", "dataname", "data_name", "createobjectifmissing", "constraint_name"}


def folder_label(node):
    """'Box3D Weld Constraint Relationship' -> 'Weld'."""
    label = node.type().description()
    label = re.sub(r"^Box3D\s+", "", label)
    return re.sub(r"\s+Constraint Relationship$", "", label)


def node_settings(node):
    """The value templates holding `node`'s settings: those in its parameter folders (a DOP data node's own
    parameters, Activation and Data Name, sit outside them), else every one but DATA_PARMS."""
    entries = without_parmops(node.parmTemplateGroup().entries())
    found = [pt for e in entries if e.type() == hou.parmTemplateType.Folder
             for pt in value_templates(e.parmTemplates())]
    if found:
        return found
    found = [pt for pt in value_templates(entries) if pt.name().lower() not in DATA_PARMS]
    if found:
        warn(f"{node.path()} has no parameter folders: promoted every setting but {sorted(DATA_PARMS)}: "
             f"{[pt.name() for pt in found]}")
    return found


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
        renames = {pt.name(): f"{name}_{pt.name()}" for pt in found}
        folders.append(collapsible(f"b3dsop_con_{name}", folder_label(node),
                                   rename_templates(found, renames), open_folder=False))
        links += [(sub.relativePathTo(node), old, new) for old, new in renames.items()]
    return folders, links


def finish_constraints(asset, links):
    """Set Initial on every inner setting the Constraints tab drives (a channel reference does not switch a DOP
    data node off Use Default)."""
    for inner_path, inner_name, _ in links:
        node = asset.node(inner_path)
        if node is not None:
            set_operation(node, inner_name, "initial", warn)
