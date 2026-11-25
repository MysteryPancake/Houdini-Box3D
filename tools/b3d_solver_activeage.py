"""Box3D Solver SOP: the age attribute from the first frame, Add Active Age Attribute and Just Activated, as RBD
Bullet Solver's Output > Attributes.

While Enable Aging is on, a SOP stage gives the pieces `age` 0 before the solver steps, as RBD Bullet Solver's Age
Attribute does, so it is there on the first frame.

Add Active Age Attribute writes `active_age` on the pieces: the time step added every step the piece is active
(`active` 1, or missing), back to 0 while it is not. Just Activated names a point group (or an attribute, by Type)
holding the pieces that became active this step. Both need Enable Aging, as RBD's need Age Attribute.

Built as RBD Bullet Solver builds it: a SOP stage before the DOP network sets the pieces' first values (one time step
for the active ones), and a Geometry Wrangle DOP in the Multiple Solver, behind an Enable Solver DOP, updates them
every step.
"""

import hou

import b3d_hda
from b3d_solver_common import (
    ADD_ACTIVE_AGE,
    create,
    ENABLE_AGING,
    JUST_ACTIVATED,
    JUST_ACTIVATED_TYPE,
    set_menu,
    set_named,
    warn,
)

ACTIVE_AGE_ATTRIB = "active_age"
ACTIVE_AGE_WRANGLE = "add_active_age"
ACTIVE_AGE_ENABLE = "enable_active_age"
ACTIVE_AGE_SOP = "set_active_age"
ACTIVE_AGE_SWITCH = "switch_active_age"
AGE_SOP = "create_age"
AGE_SWITCH = "switch_create_age"
CONSTRAINT_AGE_SOP = "create_constraint_age"
CONSTRAINT_AGE_SWITCH = "switch_create_constraint_age"
PRIMID_SOP = "store_primid"

# A piece with no `active` attribute is active (the solver's default).
ACTIVE_VEX = 'int act = haspointattrib(0, "active") ? point(0, "active", i@ptnum) : 1;'

# Before the point pass: the Just Activated attribute must exist, as a point wrangle cannot add one.
SOP_PREPARE_VEX = f"""\
string grp = chs("../{JUST_ACTIVATED}");
if (grp != "" && chi("../{JUST_ACTIVATED_TYPE}") && !haspointattrib(0, grp))
    addpointattrib(0, grp, 0);
if (!haspointattrib(0, "{ACTIVE_AGE_ATTRIB}"))
    addpointattrib(0, "{ACTIVE_AGE_ATTRIB}", 0.0);
"""

# The first values: a piece that is active starts one time step old, and is just activated.
SOP_VEX = f"""\
{ACTIVE_VEX}
string grp = chs("../{JUST_ACTIVATED}");
float age = act == 1 ? f@TimeInc : 0.0;
f@{ACTIVE_AGE_ATTRIB} = age;
if (grp != "")
{{
    if (chi("../{JUST_ACTIVATED_TYPE}"))
        setpointattrib(0, grp, i@ptnum, age > 0);
    else
        setpointgroup(0, grp, i@ptnum, age > 0);
}}
"""

# Every step: active pieces age by a time step, and one whose age was 0 is just activated; inactive ones go back to 0.
DOP_VEX = f"""\
string grp = chs("justactivatedgroup");
int dogroup = grp != "";
int doattrib = chi("type");
{ACTIVE_VEX}
if (dogroup)
{{
    if (doattrib)
        setpointattrib(0, grp, i@ptnum, 0);
    else
        setpointgroup(0, grp, i@ptnum, 0);
}}
if (act == 1)
{{
    if (dogroup && f@{ACTIVE_AGE_ATTRIB} == 0)
    {{
        if (doattrib)
            setpointattrib(0, grp, i@ptnum, 1);
        else
            setpointgroup(0, grp, i@ptnum, 1);
    }}
    f@{ACTIVE_AGE_ATTRIB} += f@TimeInc;
}}
else
    f@{ACTIVE_AGE_ATTRIB} = 0;
"""

def active_age_templates():
    """The asset's three parameters, for the Output tab's Attributes folder after Reap At Frame End."""
    add = hou.ToggleParmTemplate(
        ADD_ACTIVE_AGE, "Add Active Age Attribute", default_value=False,
        help="Adds an active_age attribute: the time each piece has been active (its active attribute is 1).")
    add.setConditional(hou.parmCondType.DisableWhen, f"{{ {ENABLE_AGING} == 0 }}")
    just = hou.StringParmTemplate(
        JUST_ACTIVATED, "Just Activated", 1, default_value=("",),
        help="A point group, or an attribute, holding the pieces that became active this step.")
    kind = hou.MenuParmTemplate(
        JUST_ACTIVATED_TYPE, "Type", ("0", "1"), ("Group", "Attribute"), default_value=0,
        help="Whether Just Activated is a point group or a point attribute.")
    just.setJoinWithNext(True)
    kind.hideLabel(True)
    for pt in (just, kind):
        pt.setConditional(hou.parmCondType.DisableWhen, f"{{ {ENABLE_AGING} == 0 }} {{ {ADD_ACTIVE_AGE} == 0 }}")
    return [add, just, kind]


def build_age_sop(sub, upstream):
    """The pieces with an `age` of 0 where they have none (a Switch, used while Enable Aging is on). Returns the
    Switch."""
    create_age = b3d_hda.attribute_create(sub, AGE_SOP, [("age", "float", None)], warn, write=False)
    create_age.setInput(0, upstream)
    switch = create(sub, "switch", AGE_SWITCH)
    switch.setInput(0, upstream)
    switch.setInput(1, create_age)
    switch.parm("input").setExpression(f'ch("../{ENABLE_AGING}")', hou.exprLanguage.Hscript)
    return switch


def build_constraint_age_sop(sub, upstream):
    """The constraints with an `age` of 0 where they have none, as RBD Bullet Solver's second Attribute Create (a
    Switch, used while Enable Aging is on). Returns the Switch."""
    create_age = b3d_hda.attribute_create(sub, CONSTRAINT_AGE_SOP, [("age", "float", None)], warn, cls="primitive",
                                          write=False)
    create_age.setInput(0, upstream)
    switch = create(sub, "switch", CONSTRAINT_AGE_SWITCH)
    switch.setInput(0, upstream)
    switch.setInput(1, create_age)
    switch.parm("input").setExpression(f'ch("../{ENABLE_AGING}")', hou.exprLanguage.Hscript)
    # Each constraint's number, as RBD Bullet Solver's store_primid: the break step finds it again in the live input.
    primid = create(sub, "attribwrangle", PRIMID_SOP)
    primid.setInput(0, switch)
    primid.parm("class").set("primitive")
    primid.parm("snippet").set("i@__primid = i@primnum;")
    return primid


def build_active_age_sop(sub, upstream):
    """The SOP stage in front of the DOP network (a Switch, used while Enable Aging and Add Active Age Attribute are
    on). Returns the Switch."""
    prepare = create(sub, "attribwrangle", f"{ACTIVE_AGE_SOP}_prepare")
    prepare.setInput(0, upstream)
    prepare.parm("class").set("detail")
    prepare.parm("snippet").set(SOP_PREPARE_VEX)
    first = create(sub, "attribwrangle", ACTIVE_AGE_SOP)
    first.setInput(0, prepare)
    first.parm("class").set("point")
    first.parm("snippet").set(SOP_VEX)
    switch = create(sub, "switch", ACTIVE_AGE_SWITCH)
    switch.setInput(0, upstream)
    switch.setInput(1, first)
    switch.parm("input").setExpression(f'ch("../{ENABLE_AGING}") * ch("../{ADD_ACTIVE_AGE}")',
                                       hou.exprLanguage.Hscript)
    return switch


def build_active_age_dop(dopnet, object_name):
    """The Geometry Wrangle DOP over the pieces and the Enable Solver DOP in front of it (the node to wire into the
    Multiple Solver)."""
    wrangle = create(dopnet, "geometrywrangle", ACTIVE_AGE_WRANGLE)
    set_named(wrangle, "bindclass", 2)  # points
    set_menu(wrangle, "bindinputmenu1", "self")  # the VEX's input 0 is the object's own geometry, as RBD's
    wrangle.addSpareParmTuple(hou.StringParmTemplate("justactivatedgroup", "Just Activated", 1))
    wrangle.addSpareParmTuple(hou.IntParmTemplate("type", "Type", 1))
    rel = wrangle.relativePathTo(dopnet.parent())
    wrangle.parm("justactivatedgroup").set(f'`chs("{rel}/{JUST_ACTIVATED}")`')
    wrangle.parm("type").setExpression(f'ch("{rel}/{JUST_ACTIVATED_TYPE}")', hou.exprLanguage.Hscript)
    wrangle.parm("snippet").set(DOP_VEX)
    enable = create(dopnet, "enablesolver", ACTIVE_AGE_ENABLE)
    set_named(enable, "group", object_name)  # the pieces only, not the collider
    enable.setInput(0, wrangle)
    enable.parm("enable").setExpression(
        f'ch("{rel}/{ENABLE_AGING}") * ch("{rel}/{ADD_ACTIVE_AGE}")', hou.exprLanguage.Hscript)
    return enable
