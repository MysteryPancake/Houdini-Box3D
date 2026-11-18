"""Box3D Solver SOP: Break Thresholds, on the Constraints tab.

RBD Bullet Solver breaks constraints in a SOP Solver after its solver (`Constraint_Solver`): a wrangle over the
object's ConstraintGeometry deletes the primitives whose `impact`, `force`, `distance`, `torque` or `angle` passes a
threshold, or switches them to their `next_constraint_name`. This is the same, for Angle, Distance, Impact and Torque
(Force is left out: Bullet's `force` on Glue is not a joint force, see docs/notes.md), and for At Frame. Impact is the
one that breaks SideFX Glue the way Houdini does: the solver writes `impact` on Glue, see docs/agents/joints.md.

The parameters follow Bullet's names (`breaks`, `constraint_names#`, `constraint_mode#`, `constraint_useimpact#`,
`constraint_impactthreshold#` ...) and its rules: a threshold is multiplied by World Scale, "Scale by Attribute"
multiplies it by that primitive attribute (`strength` breaks Glue relative to its own strength), and Variance
scatters it per constraint. The SOP Solver is wired into the Multiple Solver last, behind an Enable Solver DOP: with
Break Thresholds off the constraint geometry is never imported, so it is not rewritten every step (a rewrite makes
the solver read the whole network again)."""

import hou

from b3d_hda import collapsible
from b3d_solver_common import (create, GLOBAL_SUBSTEPS, set_named, START_FRAME, WORLD_SCALE, WORLD_SCALE_TOGGLE,
                               warn)

GLUE_COMPOUND = "gluecompound"  # the solver's Glue as One Body, shown on the same tab
BREAKS = "breaks"  # the number of break entries (a multiparm block)
BREAKS_TOGGLE = "enable_constraintbreaks"
BREAK_SOLVER = "constraint_breaks"  # the SOP Solver DOP
BREAK_ENABLE = "enable_constraint_breaks"  # the Enable Solver DOP in front of it
BREAK_WRANGLE = "break_constraints"
CONSTRAINT_GEOMETRY = "ConstraintGeometry"

# The thresholds, in the order Bullet checks them: (key, label, default threshold, default on, scaled by World Scale,
# what the threshold is compared against). Bullet multiplies Distance and Force by World Scale, not Angle or Torque.
THRESHOLDS = (
    ("angle", "Angle", 0.0, False, False, "Break a constraint whose angle (radians, since the start) passes this."),
    ("distance", "Distance", 1.0, True, True,
     "Break a constraint whose anchors are further apart than this, beyond its Rest Length."),
    ("impact", "Impact", 10000.0, True, True,
     "Break a constraint whose impact passes this. A Glue's impact is the impulse it has taken from hits."),
    ("torque", "Torque", 0.0, False, False, "Break a constraint whose torque passes this."),
)

# Every parameter name this adds on the asset (checked against the others: names must be unique).
BREAK_PARMS = [BREAKS_TOGGLE, BREAKS] + [
    f"constraint_{p}#" for p in ("names", "mode", "usefromframe", "fromframe", "relfromframe", "useatframe", "atframe",
                                 "relatframe")] + [
    n for key, *_ in THRESHOLDS
    for n in (f"constraint_use{key}#", f"constraint_{key}threshold#", f"constraint_scale{key}#",
              f"constraint_use{key}var#", f"constraint_{key}var#", f"constraint_{key}attrib#")]

# The wrangle (Run Over Primitives) in the SOP Solver. REL is the path from the wrangle to the asset.
BREAK_VEX = '''// Breaks the constraints the Break Thresholds name, as RBD Bullet Solver's break_constraints (Force excepted).
void switchnext(int primnum)
{
    if (hasprimattrib(0, "next_constraint_name"))
    {
        string nc = prim(0, "next_constraint_name", primnum);
        if (nc == "")
        {
            removeprim(0, primnum, 1);
            return;
        }
        string nt = hasprimattrib(0, "next_constraint_type") ? prim(0, "next_constraint_type", primnum) : "";
        setprimattrib(0, "constraint_name", primnum, nc);
        if (hasprimattrib(0, "constraint_type"))
            setprimattrib(0, "constraint_type", primnum, nt);
        setprimattrib(0, "next_constraint_name", primnum, "");
        if (hasprimattrib(0, "next_constraint_type"))
            setprimattrib(0, "next_constraint_type", primnum, "");
    }
    else removeprim(0, primnum, 1);
}

// Does the primitive's `key` attribute pass entry n's threshold? The threshold is the entry's, times scale, times the
// variance, times the Scale by Attribute attribute.
int passes(string key, n; float scale; int primnum, id; float rest)
{
    if (!chi("REL/constraint_use" + key + n) || !hasprimattrib(0, key))
        return 0;
    float threshold = chf("REL/constraint_" + key + "threshold" + n) * scale;
    if (chi("REL/constraint_use" + key + "var" + n))
    {
        float var = chf("REL/constraint_" + key + "var" + n);
        threshold *= fit01(rand(id), 1 - var, 1 + var);
    }
    if (chi("REL/constraint_scale" + key + n))
    {
        string attrib = chs("REL/constraint_" + key + "attrib" + n);
        if (attrib != "" && hasprimattrib(0, attrib))
            threshold *= float(prim(0, attrib, primnum));
    }
    return float(prim(0, key, primnum)) > threshold + rest;
}

if (!chi("REL/enable_constraintbreaks") || !hasprimattrib(0, "constraint_name"))
    return;
int count = chi("REL/breaks");
string cname = prim(0, "constraint_name", @primnum);
float scale = chi("REL/WORLD_SCALE_TOGGLE") ? chf("REL/WORLD_SCALE") : 1.0;
int id = hasprimattrib(0, "__primid") ? prim(0, "__primid", @primnum) : @primnum;
float rest = hasprimattrib(0, "restlength") ? prim(0, "restlength", @primnum) : 0.0;
float window = 0.5 / max(chi("REL/GLOBAL_SUBSTEPS"), 1);

for (int i = 1; i <= count; ++i)
{
    string n = itoa(i);
    string names[] = split(chs("REL/constraint_names" + n), " ");
    if (find(names, cname) < 0)
        continue;
    int mode = chi("REL/constraint_mode" + n);
    int hit = 0;
    if (chi("REL/constraint_useatframe" + n))
    {
        float at = chf("REL/constraint_atframe" + n) + chi("REL/constraint_relatframe" + n) * chf("REL/START_FRAME");
        hit = @Frame > at - window && @Frame < at + window;
    }
    int started = 1;
    if (chi("REL/constraint_usefromframe" + n))
    {
        float from = chf("REL/constraint_fromframe" + n) + chi("REL/constraint_relfromframe" + n) * chf("REL/START_FRAME");
        started = from <= @Frame;
    }
    if (!hit && started)
    {
        hit = PASSES;
    }
    if (hit)
    {
        if (mode) switchnext(@primnum);
        else removeprim(0, @primnum, 1);
        return;
    }
}
'''


def break_items():
    """The parameters for the Constraints tab: the header toggle and the Break Thresholds folder (a multiparm block)."""
    on = f"{{ {BREAKS_TOGGLE} == 0 }}"

    def toggle(name, label, default=False, join=False, hidden=False, help=None):
        pt = hou.ToggleParmTemplate(name, label, default_value=default, help=help)
        pt.setJoinWithNext(join)
        if hidden:
            pt.hideLabel(True)
        return pt

    def when(pt, *conditions, kind=hou.parmCondType.DisableWhen):
        pt.setConditional(kind, " ".join(f"{{ {c} }}" for c in conditions))
        return pt

    names = hou.StringParmTemplate(
        "constraint_names#", "Constraint Names", 1, default_value=("Glue",),
        help="The constraint types to break, by Data Name, separated by spaces. Glue is the only type with an impact.")
    mode = hou.MenuParmTemplate(
        "constraint_mode#", "Mode", ("0", "1"), menu_labels=("Delete Constraints", "Switch to Next Constraint"),
        default_value=0, help="Delete the constraint, or give it its next_constraint_name (deleted if it has none).")
    use_from = toggle("constraint_usefromframe#", "From Frame", join=True,
                      help="Only break constraints from this frame on.")
    from_frame = when(hou.FloatParmTemplate("constraint_fromframe#", "From Frame", 1, default_value=(0.0,)),
                      "constraint_usefromframe# == 0")
    from_frame.hideLabel(True)
    from_frame.setJoinWithNext(True)
    rel_from = when(toggle("constraint_relfromframe#", "Relative to Start Frame"), "constraint_usefromframe# == 0")

    use_at = toggle("constraint_useatframe#", "At Frame", join=True, help="Break the constraints at this frame.")
    at_frame = when(hou.FloatParmTemplate("constraint_atframe#", "At Frame", 1, default_value=(0.0,)),
                    "constraint_useatframe# == 0")
    at_frame.hideLabel(True)
    at_frame.setJoinWithNext(True)
    rel_at = when(toggle("constraint_relatframe#", "Relative to Start Frame"), "constraint_useatframe# == 0")

    items = [names, mode, use_from, from_frame, rel_from, use_at, at_frame, rel_at]
    for key, label, default, on_by_default, _scaled, help_text in THRESHOLDS:
        off = f"constraint_use{key}# == 0"
        use = toggle(f"constraint_use{key}#", f"{label} Threshold", default=on_by_default, join=True)
        threshold = when(hou.FloatParmTemplate(f"constraint_{key}threshold#", f"{label} Threshold", 1,
                                               default_value=(default,), min=0.0, help=help_text), off)
        threshold.hideLabel(True)
        scaling = when(hou.MenuParmTemplate(
            f"constraint_scale{key}#", "Scaling", ("0", "1"), menu_labels=("No Scaling", "Scale by Attribute"),
            default_value=0, help="Multiply the threshold by a primitive attribute (strength, for a threshold "
                                  "relative to each glue's own strength)."), off)
        use_var = when(toggle(f"constraint_use{key}var#", "Var", join=True, help="Scatter the threshold per constraint."),
                       off)
        variance = when(hou.FloatParmTemplate(f"constraint_{key}var#", f"{label} Variance", 1, default_value=(0.1,),
                                              min=0.0, max=1.0),
                        off, f"constraint_use{key}var# == 0", kind=hou.parmCondType.HideWhen)
        attrib = when(hou.StringParmTemplate(f"constraint_{key}attrib#", f"{label} Attribute", 1,
                                             default_value=("strength",)),
                      off, f"constraint_scale{key}# == 0", kind=hou.parmCondType.HideWhen)
        items += [use, threshold, scaling, use_var, variance, attrib]

    block = hou.FolderParmTemplate(BREAKS, "Number of Breaks", items,
                                   folder_type=hou.folderType.TabbedMultiparmBlock, default_value=1)
    folder = collapsible("b3dsop_break_thresholds", "Break Thresholds", [block], header_toggle=BREAKS_TOGGLE)
    folder.setConditional(hou.parmCondType.DisableWhen, on)
    header = hou.ToggleParmTemplate(BREAKS_TOGGLE, "Enable Break Thresholds", default_value=False,
                                    help="Break constraints that pass a threshold, as RBD Bullet Solver's "
                                         "Break Thresholds. Off costs nothing.")
    return [header, folder]


def build_breaks(dopnet):
    """The SOP Solver over the pieces' ConstraintGeometry and the Enable Solver DOP in front of it (the node to wire
    into the Multiple Solver). The wrangle reads the asset's parameters by path."""
    solver = create(dopnet, "sopsolver", BREAK_SOLVER)
    set_named(solver, "datapath", CONSTRAINT_GEOMETRY)
    set_named(solver, "dataname", "Constraint_Breaks")
    wrangle = solver.createNode("attribwrangle", BREAK_WRANGLE)
    outputs = [c for c in solver.children() if c.type().name() == "output"]
    if not outputs or not outputs[0].inputs():
        warn(f"{solver.path()} has no Output fed by its geometry import: Break Thresholds do nothing")
        return solver
    wrangle.setInput(0, outputs[0].inputs()[0])
    outputs[0].setInput(0, wrangle)
    set_named(wrangle, "class", 1)  # primitives
    vex = BREAK_VEX.replace("REL", wrangle.relativePathTo(dopnet.parent()))
    vex = vex.replace("WORLD_SCALE_TOGGLE", WORLD_SCALE_TOGGLE).replace("WORLD_SCALE", WORLD_SCALE)
    # The checks, in THRESHOLDS' order: scaled ones multiply by World Scale, Distance adds the Rest Length.
    passes = "\n            || ".join(
        f'passes("{key}", n, {"scale" if scaled else "1.0"}, @primnum, id, {"rest" if key == "distance" else "0.0"})'
        for key, _label, _default, _on, scaled, _help in THRESHOLDS)
    vex = vex.replace("PASSES", passes)
    vex = vex.replace("GLOBAL_SUBSTEPS", GLOBAL_SUBSTEPS).replace("START_FRAME", START_FRAME)
    wrangle.parm("snippet").set(vex)
    wrangle.moveToGoodPosition()

    enable = create(dopnet, "enablesolver", BREAK_ENABLE)
    enable.setInput(0, solver)
    rel = enable.relativePathTo(dopnet.parent())
    enable.parm("enable").setExpression(f'ch("{rel}/{BREAKS_TOGGLE}") && ch("{rel}/{BREAKS}") > 0',
                                        hou.exprLanguage.Hscript)
    return enable
