"""Box3D Solver SOP: Enable Attribute Update from SOPs and Overwrite with SOP, as RBD Bullet Solver's.

The Constraint Network reads the Constraints input once. Enable Attribute Update from SOPs (Constraints tab, Override
Attributes) then copies the listed attributes from the live input onto the simulation's constraints every step, matched
by `__primid` (which `store_primid` writes), for the constraints in Group. Overwrite with SOP (Advanced, Constraints)
makes the Constraint Network read the whole input again every step instead.

Built as RBD Bullet Solver builds it: a SOP Solver DOP over the ConstraintGeometry with an Attribute Copy SOP, behind an
Enable Solver DOP, in the Multiple Solver before the break step.
"""

import hou

import b3d_hda
from b3d_solver_common import (
    CONSTRAINT_ATTRIB_GROUP,
    CONSTRAINT_ATTRIBUTES,
    CONSTRAINT_RELOAD,
    create,
    DOP_CONSTRAINTS,
    ENABLE_CONSTRAINT_UPDATES,
    set_named,
    warn,
)

CONSTRAINT_GEOMETRY = "ConstraintGeometry"
UPDATE_SOLVER = "constraint_updates"  # the SOP Solver DOP
UPDATE_ENABLE = "enable_constraint_updates"  # the Enable Solver DOP in front of it
UPDATE_COPY = "copy_attributes"
UPDATE_LIVE = "live_constraints"


def update_folder():
    """The Constraints tab's Override Attributes folder, as RBD's: the toggle (label hidden) joined to Attributes, then
    Group."""
    enable = hou.ToggleParmTemplate(
        ENABLE_CONSTRAINT_UPDATES, "Enable Attribute Update from SOPs", default_value=False, join_with_next=True,
        help="Copies the listed attributes from the live Constraints input onto the simulation's constraints every "
             "step, so ones that change over time reach it.")
    enable.hideLabel(True)
    enable.setConditional(hou.parmCondType.DisableWhen, f"{{ {CONSTRAINT_RELOAD} == 1 }}")
    attributes = hou.StringParmTemplate(
        CONSTRAINT_ATTRIBUTES, "Attributes", 1, default_value=("",),
        help="The primitive attributes to copy each step, matched by name. Wildcards work.")
    attributes.setConditional(hou.parmCondType.DisableWhen, f"{{ {ENABLE_CONSTRAINT_UPDATES} == 0 }}")
    b3d_hda.attrib_menu(attributes, 1, "prim", toggle=False)
    group = hou.StringParmTemplate(
        CONSTRAINT_ATTRIB_GROUP, "Group", 1, default_value=("",),
        help="Only update the constraints in this primitive group. Empty updates every constraint.")
    group.setConditional(hou.parmCondType.DisableWhen, f"{{ {ENABLE_CONSTRAINT_UPDATES} == 0 }}")
    b3d_hda.group_menu(group, 1, "prim")
    return hou.FolderParmTemplate("b3dsop_constraint_updates", "Override Attributes", [enable, attributes, group],
                                  folder_type=hou.folderType.Simple)


def wire_constraint_updates(asset):
    """The Attribute Copy's attributes and group, read from the asset's parameters (set once it has them: the Enable
    Solver DOP warns about a path to a parameter that does not exist yet)."""
    copy = asset.node(f"dopnet/{UPDATE_SOLVER}/{UPDATE_COPY}")
    if copy is None:
        warn(f"{asset.path()} has no {UPDATE_SOLVER}/{UPDATE_COPY}: Enable Attribute Update from SOPs does nothing")
        return
    rel = copy.relativePathTo(asset)
    copy.parm("attribname").setExpression(f'chs("{rel}/{CONSTRAINT_ATTRIBUTES}")', hou.exprLanguage.Hscript)
    copy.parm("destgroup").setExpression(f'chs("{rel}/{CONSTRAINT_ATTRIB_GROUP}")', hou.exprLanguage.Hscript)


def reload_template():
    """Overwrite with SOP, at the top of Advanced > Constraints."""
    reload = hou.ToggleParmTemplate(
        CONSTRAINT_RELOAD, "Overwrite with SOP", default_value=False,
        help="Reads the whole Constraints input again every step. Constraints that broke come back, so use it only "
             "when the input changes over time.")
    return reload


def build_constraint_updates(dopnet):
    """The SOP Solver DOP over the pieces' ConstraintGeometry and the Enable Solver DOP in front of it (the node to wire
    into the Multiple Solver). The attributes and group are read from the asset by path."""
    solver = create(dopnet, "sopsolver", UPDATE_SOLVER)
    set_named(solver, "datapath", CONSTRAINT_GEOMETRY)
    set_named(solver, "dataname", "Constraint_Updates")
    outputs = [c for c in solver.children() if c.type().name() == "output"]
    if not outputs or not outputs[0].inputs():
        warn(f"{solver.path()} has no Output fed by its geometry import: Enable Attribute Update from SOPs does "
             "nothing")
        return solver
    live = solver.createNode("object_merge", UPDATE_LIVE)
    live.parm("objpath1").set(f'`dopcontextgeo("..", {DOP_CONSTRAINTS})`')
    copy = solver.createNode("attribcopy", UPDATE_COPY)
    copy.setInput(0, outputs[0].inputs()[0])
    copy.setInput(1, live)
    outputs[0].setInput(0, copy)
    set_named(copy, "matchbyattribute", 1)
    set_named(copy, "attributetomatch", "__primid")
    set_named(copy, "class", 0)  # primitives
    set_named(copy, "srcgrouptype", 2)  # primitives
    set_named(copy, "destgrouptype", 2)
    copy.moveToGoodPosition()
    live.moveToGoodPosition()
    enable = create(dopnet, "enablesolver", UPDATE_ENABLE)
    enable.setInput(0, solver)
    erel = enable.relativePathTo(dopnet.parent())
    enable.parm("enable").setExpression(f'ch("{erel}/{ENABLE_CONSTRAINT_UPDATES}") && ch("{erel}/{CONSTRAINT_RELOAD}") == 0',
                                        hou.exprLanguage.Hscript)
    return enable
