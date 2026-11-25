"""Box3D Solver SOP: Break Thresholds, on the Constraints tab.

RBD Bullet Solver breaks constraints in a SOP Solver after its solver (`Constraint_Solver`): a primitive wrangle, run
once for each break entry in a loop, over the object's ConstraintGeometry deletes the primitives whose `impact`, `force`, `distance`, `torque` or `angle` passes a
threshold, or switches them to their `next_constraint_name`. This is the same, for Angle, Distance, Impact and Torque
(Force is left out: Bullet's `force` on Glue is not a joint force, see docs/notes.md), and for At Frame. Impact is the
one that breaks SideFX Glue the way Houdini does: the solver writes `impact` on Glue, see docs/agents/joints-sidefx.md.

The parameters follow Bullet's names (`breaks`, `constraint_names#`, `constraint_mode#`, `constraint_useimpact#`,
`constraint_impactthreshold#` ...) and its rules: a threshold is multiplied by World Scale, "Scale by Attribute"
multiplies it by that primitive attribute (`strength` breaks Glue relative to its own strength), and Variance
scatters it per constraint. The SOP Solver is wired into the Multiple Solver last, behind an Enable Solver DOP: with
Break Thresholds off the constraint geometry is never imported, so it is not rewritten every step (a rewrite makes
the solver read the whole network again)."""

import hou

import b3d_hda
from b3d_hda import collapsible
from b3d_solver_common import (create, DOP_CONSTRAINTS, GLOBAL_SUBSTEPS, RELATIONSHIPS, set_named, START_FRAME,
                               WORLD_SCALE,
                               WORLD_SCALE_TOGGLE, warn)
from b3d_solver_constraints import node_settings

BREAKS = "breaks"  # the number of break entries (a multiparm block)
BREAKS_TOGGLE = "enable_constraintbreaks"
BREAK_SOLVER = "constraint_breaks"  # the SOP Solver DOP
BREAK_ENABLE = "enable_constraint_breaks"  # the Enable Solver DOP in front of it
BREAK_WRANGLE = "break_constraints"
BREAK_BEGIN, BREAK_META, BREAK_END = "break_begin", "break_metadata", "break_end"  # the loop over the entries
BREAK_LIVE_IN = "break_live"  # the loop's block begin that fetches the live constraints
BREAK_COMPILE_BEGIN, BREAK_COMPILE_LIVE, BREAK_COMPILE_END = "break_compile_begin", "break_compile_live", "break_compile_end"
BREAK_LIVE = "live_constraints"  # the SOP Solver's Object Merge of the live constraints input, the wrangle's third input
CONSTRAINT_GEOMETRY = "ConstraintGeometry"

# The thresholds, in the order Bullet checks them: (key, label, default threshold, default on, scaled by World Scale,
# what the threshold is compared against). Bullet multiplies Distance and Force by World Scale, not Angle or Torque.
THRESHOLDS = (
    ("angle", "Angle", 0.0, False, False, "Break a constraint whose angle (radians, since the start) passes this."),
    ("distance", "Distance", 1.0, True, True,
     "Break a constraint whose anchors are further apart than this, beyond its Rest Length."),
    ("impact", "Impact", 0.0, False, True,
     "Break a constraint whose impact passes this. A Glue's impact is the impulse it has taken from hits."),
    ("torque", "Torque", 0.0, False, False, "Break a constraint whose torque passes this."),
)

# Every parameter name this adds on the asset (checked against the others: names must be unique).
BREAK_PARMS = [BREAKS_TOGGLE, BREAKS] + [
    f"constraint_{p}#" for p in ("names", "mode", "usefromframe", "fromframe", "relfromframe", "useatframe", "atframe",
                                 "relatframe")] + [
    n for key, *_ in THRESHOLDS
    for n in (f"constraint_use{key}#", f"constraint_{key}threshold#", f"constraint_scale{key}#",
              f"constraint_use{key}var#", f"constraint_{key}var#", f"constraint_{key}attrib#",
              f"constraint_{key}attribupdate#")]

# The wrangle's Group: the constraints the entry's Constraint Names list, one `@constraint_name=` pattern per name, so
# the match is found natively and not by string work in VEX for every primitive. "!*" for no names, as an empty group
# would be every primitive. REL is the path to the asset.
GROUP_EXPRESSION = """words = hou.evalParm("REL/constraint_names%d" % (int(hou.hscriptExpression('detail(-1, "iteration", 0)')) + 1)).split()
return " ".join("@constraint_name=" + word for word in words) or "!*"
"""

# The wrangle (Run Over Primitives, once for each break entry by the loop around it) in the SOP Solver, as RBD Bullet
# Solver's: the entry's settings are spare parameters of the wrangle (`break_spare_parms`), which the loop's iteration
# picks, so the code reads them with plain literal channels.
BREAK_VEX = '''// Breaks the constraints one Break Thresholds entry names, as RBD Bullet Solver's break_constraints (Force excepted).
#define UNSETF -3.4028235e38
// A constraint that switches takes its next settings: each next_<setting> that is set becomes <setting> and is cleared
// (a number at UNSETF, a toggle below 0 and an empty string are "not set").
void takenext(int primnum)
{
    string floats[] = {NEXT_FLOATS};
    string ints[] = {NEXT_INTS};
    string vectors[] = {NEXT_VECTORS};
    foreach (string a; floats) {
        string n = "next_" + a;
        if (!hasprimattrib(0, n)) continue;
        float v = prim(0, n, primnum);
        if (v <= -1e37) continue;
        if (!hasprimattrib(0, a)) addprimattrib(0, a, UNSETF);
        setprimattrib(0, a, primnum, v);
        setprimattrib(0, n, primnum, UNSETF);
    }
    foreach (string a; ints) {
        string n = "next_" + a;
        if (!hasprimattrib(0, n)) continue;
        int v = prim(0, n, primnum);
        if (v < 0) continue;
        if (!hasprimattrib(0, a)) addprimattrib(0, a, -1);
        setprimattrib(0, a, primnum, v);
        setprimattrib(0, n, primnum, -1);
    }
    foreach (string a; vectors) {
        string n = "next_" + a;
        if (!hasprimattrib(0, n)) continue;
        vector v = prim(0, n, primnum);
        if (v.x <= -1e37) continue;
        if (!hasprimattrib(0, a)) addprimattrib(0, a, set(UNSETF, 0.0, 0.0));
        setprimattrib(0, a, primnum, v);
        setprimattrib(0, n, primnum, set(UNSETF, 0.0, 0.0));
    }
}

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
        if (nt != "" && !hasprimattrib(0, "constraint_type"))
            addprimattrib(0, "constraint_type", "");
        if (hasprimattrib(0, "constraint_type"))
            setprimattrib(0, "constraint_type", primnum, nt);
        takenext(primnum);
        setprimattrib(0, "next_constraint_name", primnum, "");
        if (hasprimattrib(0, "next_constraint_type"))
            setprimattrib(0, "next_constraint_type", primnum, "");
    }
    else removeprim(0, primnum, 1);
}

// Does the primitive's `key` attribute pass the threshold? `use` is the entry's toggle, `threshold` the entry's (times
// World Scale), `var` its variance (below 0: none), `attrib` the Scale by Attribute attribute ("" for none).
int passes(string key; int use; float threshold, var; string attrib; int update; float rest; int primnum, id)
{
    if (!use || !hasprimattrib(0, key))
        return 0;
    // (a copy: a function's parameters are writable, so they cannot be given channel calls)
    float limit = threshold;
    if (var >= 0)
        limit *= fit01(rand(id), 1 - var, 1 + var);
    if (attrib != "" && hasprimattrib(0, attrib))
    {
        float value = prim(0, attrib, primnum);
        // Update from Input, as RBD Bullet Solver: the same constraint's value in the live input, found by __primid
        if (update && hasprimattrib(2, attrib))
        {
            int live = findattribval(2, "prim", "__primid", id);
            if (live >= 0)
                value = prim(2, attrib, live);
        }
        limit *= value;
    }
    return float(prim(0, key, primnum)) > limit + rest;
}

// The wrangle's Group is the constraints the entry names (see GROUP_EXPRESSION), so every primitive here is one.
int pr = i@primnum;
int hit =chi("constraint_useatframe")
    && f@Frame > chf("constraint_atframe") - chf("substepwindow") && f@Frame < chf("constraint_atframe") + chf("substepwindow");
if (!hit)
{
    if (chi("constraint_usefromframe") && chf("constraint_fromframe") > f@Frame)
        return;
    int id = hasprimattrib(0, "__primid") ? prim(0, "__primid", pr) : pr;
    hit = PASSES;
}
if (!hit)
    return;
if (chi("constraint_mode")) switchnext(pr);
else removeprim(0, pr, 1);
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
        "constraint_names#", "Constraint Names", 1, default_value=("Soft Hard",),
        help="The constraint types to break, by Data Name, separated by spaces. Glue is the only type with an impact.")
    b3d_hda.menu_script(names, "buildConstraintNameMenu(kwargs, 1)")
    mode = hou.MenuParmTemplate(
        "constraint_mode#", "Mode", ("0", "1"), menu_labels=("Delete Constraints", "Switch to Next Constraint"),
        default_value=0, help="Delete the constraint, or give it its next_constraint_name (deleted if it has none).")
    # As RBD Bullet Solver lays them out: a checkbox with no label joined to its float, which keeps its label.
    use_at = toggle("constraint_useatframe#", "At Frame", join=True, hidden=True,
                    help="Break the constraints at this frame.")
    at_frame = when(hou.FloatParmTemplate("constraint_atframe#", "At Frame", 1, default_value=(0.0,)),
                    "constraint_useatframe# == 0")
    at_frame.setJoinWithNext(True)
    rel_at = when(toggle("constraint_relatframe#", "Relative to Start Frame"), "constraint_useatframe# == 0")

    use_from = toggle("constraint_usefromframe#", "From Frame", join=True, hidden=True,
                      help="Only break constraints from this frame on.")
    from_frame = when(hou.FloatParmTemplate("constraint_fromframe#", "From Frame", 1, default_value=(0.0,)),
                      "constraint_usefromframe# == 0")
    from_frame.setJoinWithNext(True)
    rel_from = when(toggle("constraint_relfromframe#", "Relative to Start Frame"), "constraint_usefromframe# == 0")

    items = [names, mode, use_at, at_frame, rel_at, use_from, from_frame, rel_from]
    for key, label, default, on_by_default, _scaled, help_text in THRESHOLDS:
        off = f"constraint_use{key}# == 0"
        use = toggle(f"constraint_use{key}#", f"{label} Threshold", default=on_by_default, join=True, hidden=True)
        threshold = when(hou.FloatParmTemplate(f"constraint_{key}threshold#", f"{label} Threshold", 1,
                                               default_value=(default,), min=0.0, help=help_text), off)
        threshold.setJoinWithNext(True)
        scaling = when(hou.MenuParmTemplate(
            f"constraint_scale{key}#", "Scaling", ("0", "1"), menu_labels=("No Scaling", "Scale by Attribute"),
            default_value=0, help="Multiply the threshold by a primitive attribute (strength, for a threshold "
                                  "relative to each glue's own strength)."), off)
        scaling.hideLabel(True)
        scaling.setJoinWithNext(True)
        use_var = when(toggle(f"constraint_use{key}var#", "Var", hidden=True,
                              help="Scatter the threshold per constraint."), off)
        variance = when(hou.FloatParmTemplate(f"constraint_{key}var#", f"{label} Variance", 1, default_value=(0.1,),
                                              min=0.0, max=1.0),
                        off, f"constraint_use{key}var# == 0", kind=hou.parmCondType.HideWhen)
        attrib = when(b3d_hda.attrib_menu(
                          hou.StringParmTemplate(f"constraint_{key}attrib#", f"{label} Attribute", 1,
                                                 default_value=("",)),
                          1, "prim", toggle=False, numeric=True),
                      off, f"constraint_scale{key}# == 0", kind=hou.parmCondType.HideWhen)
        # Update from Input on the Attribute's row, as RBD Bullet Solver's
        attrib.setJoinWithNext(True)
        update = when(toggle(f"constraint_{key}attribupdate#", "Update from Input",
                             help="Read the attribute from the live Constraints input every step, so one that changes "
                                  "over time moves the threshold. Off: its value when the constraint was made."),
                      off, f"constraint_scale{key}# == 0", kind=hou.parmCondType.HideWhen)
        items += [use, threshold, scaling, use_var, variance, attrib, update]

    block = hou.FolderParmTemplate(BREAKS, "Number of Breaks", items,
                                   folder_type=hou.folderType.TabbedMultiparmBlock, default_value=1)
    folder = collapsible("b3dsop_break_thresholds", "Break Thresholds", [block], header_toggle=BREAKS_TOGGLE)
    folder.setConditional(hou.parmCondType.DisableWhen, on)
    header = hou.ToggleParmTemplate(BREAKS_TOGGLE, "Enable Break Thresholds", default_value=True,
                                    help="Break constraints that pass a threshold, as RBD Bullet Solver's "
                                         "Break Thresholds. Off costs nothing.")
    return [header, folder]


def next_setting_names(dopnet):
    """The primitive attributes of every constraint setting (the relationship nodes'), by kind: (floats, ints,
    vectors), for a switching constraint's next_<setting> attributes."""
    floats, ints, vectors = {}, {}, {}
    for _, name in RELATIONSHIPS:
        node = dopnet.node(name)
        if node is None:
            continue
        for pt in node_settings(node):
            if pt.type() == hou.parmTemplateType.Float:
                (vectors if pt.numComponents() == 3 else floats)[pt.name()] = True
            elif pt.type() in (hou.parmTemplateType.Toggle, hou.parmTemplateType.Int):
                ints[pt.name()] = True
    return sorted(floats), sorted(ints), sorted(vectors)


def break_spare_parms(wrangle, rel, meta):
    """The spare parameters the wrangle reads (`constraint_<name>`: the asset's `constraint_<name>#` of the loop's
    current entry). `rel` is the path from the wrangle to the asset, `meta` the name of the loop's metadata block."""
    entry = '(detail(-1, "iteration", 0) + 1)'  # -1: the spare input, the loop's metadata (a path would not compile)

    def asset(name):
        return f'ch("{rel}/{name}" + {entry})'

    start = f'ch("{rel}/{START_FRAME}")'
    numbers = {
        "constraint_mode": asset("constraint_mode"),
        "constraint_useatframe": asset("constraint_useatframe"),
        "constraint_atframe": f'{asset("constraint_atframe")} + {asset("constraint_relatframe")} * {start}',
        "constraint_usefromframe": asset("constraint_usefromframe"),
        "constraint_fromframe": f'{asset("constraint_fromframe")} + {asset("constraint_relfromframe")} * {start}',
        "substepwindow": f'0.5 / max(ch("{rel}/{GLOBAL_SUBSTEPS}"), 1)',
        "worldscale": f'if(ch("{rel}/{WORLD_SCALE_TOGGLE}"), ch("{rel}/{WORLD_SCALE}"), 1)',
    }
    strings = {}
    for key, _label, _default, _on, scaled, _help in THRESHOLDS:
        numbers[f"constraint_use{key}"] = asset(f"constraint_use{key}")
        threshold = asset(f"constraint_{key}threshold")
        numbers[f"constraint_{key}threshold"] = threshold + (' * ch("worldscale")' if scaled else "")
        for name in (f"constraint_use{key}var", f"constraint_{key}var", f"constraint_scale{key}",
                     f"constraint_{key}attribupdate"):
            numbers[name] = asset(name)
        strings[f"constraint_{key}attrib"] = f'`chs("{rel}/constraint_{key}attrib" + {entry})`'
    group = wrangle.parmTemplateGroup()
    group.append(hou.StringParmTemplate("spare_input0", "Spare Input 0", 1, string_type=hou.stringParmType.NodeReference,
                                        tags={"opfilter": "!!SOP!!", "oprelative": "."}))
    for name in numbers:
        group.append(hou.FloatParmTemplate(name, name, 1))
    for name in strings:
        group.append(hou.StringParmTemplate(name, name, 1))
    wrangle.setParmTemplateGroup(group)
    for name, expression in numbers.items():
        wrangle.parm(name).setExpression(expression, hou.exprLanguage.Hscript)
    for name, value in strings.items():
        wrangle.parm(name).set(value)
    wrangle.parm("spare_input0").set(f"../{meta}")


def build_breaks(dopnet):
    """The SOP Solver over the pieces' ConstraintGeometry and the Enable Solver DOP in front of it (the node to wire
    into the Multiple Solver). The wrangle runs once for each break entry, in a loop as RBD Bullet Solver's."""
    solver = create(dopnet, "sopsolver", BREAK_SOLVER)
    set_named(solver, "datapath", CONSTRAINT_GEOMETRY)
    set_named(solver, "dataname", "Constraint_Breaks")
    outputs = [c for c in solver.children() if c.type().name() == "output"]
    if not outputs or not outputs[0].inputs():
        warn(f"{solver.path()} has no Output fed by its geometry import: Break Thresholds do nothing")
        return solver
    source = outputs[0].inputs()[0]
    begin = solver.createNode("block_begin", BREAK_BEGIN)  # Fetch Feedback: the first entry gets the import
    meta = solver.createNode("block_begin", BREAK_META)  # the iteration, which picks the entry
    end = solver.createNode("block_end", BREAK_END)
    # Compiled, as RBD Bullet Solver's: the geometry comes in through Block Begin Compile nodes.
    cend = solver.createNode("compile_end", BREAK_COMPILE_END)
    cbegin = solver.createNode("compile_begin", BREAK_COMPILE_BEGIN)
    set_named(cbegin, "blockpath", f"../{BREAK_COMPILE_END}")
    set_named(cbegin, "name", "ConstraintGeometry")
    cbegin.setInput(0, source)
    begin.setInput(0, cbegin)
    meta.setInput(0, cbegin)
    set_named(begin, "method", "feedback")
    set_named(meta, "method", "metadata")
    for node in (begin, meta):
        set_named(node, "blockpath", f"../{BREAK_END}")
    set_named(end, "blockpath", f"../{BREAK_BEGIN}")
    set_named(end, "itermethod", "count")
    set_named(end, "multithread", 1)
    end.parm("iterations").setExpression(f'ch("{end.relativePathTo(dopnet.parent())}/{BREAKS}")',
                                         hou.exprLanguage.Hscript)
    wrangle = solver.createNode("attribwrangle", BREAK_WRANGLE)
    wrangle.setInput(0, begin)
    wrangle.setInput(1, meta)
    # Update from Input: the constraints as the SOP gives them now, the wrangle's input 3 (an Object Merge of the
    # DOP network's constraints input, read only while Break Thresholds are on).
    live = solver.createNode("object_merge", BREAK_LIVE)
    live.parm("objpath1").set(f'`dopcontextgeo("..", {DOP_CONSTRAINTS})`')
    live.parm("enable1").setExpression(f'ch("{live.relativePathTo(dopnet.parent())}/{BREAKS_TOGGLE}")',
                                       hou.exprLanguage.Hscript)
    clive = solver.createNode("compile_begin", BREAK_COMPILE_LIVE)
    set_named(clive, "blockpath", f"../{BREAK_COMPILE_END}")
    set_named(clive, "name", "Live Constraints")
    clive.setInput(0, live)
    live_in = solver.createNode("block_begin", BREAK_LIVE_IN)
    set_named(live_in, "method", "input")
    set_named(live_in, "blockpath", f"../{BREAK_END}")
    live_in.setInput(0, clive)
    wrangle.setInput(2, live_in)
    end.setInput(0, wrangle)
    cend.setInput(0, end)
    outputs[0].setInput(0, cend)
    set_named(wrangle, "class", 1)  # primitive
    rel = wrangle.relativePathTo(dopnet.parent())
    break_spare_parms(wrangle, rel, BREAK_META)
    wrangle.parm("group").setExpression(GROUP_EXPRESSION.replace("REL", rel),
                                        hou.exprLanguage.Python)
    floats, ints, vectors = next_setting_names(dopnet)

    def names(values):
        return ", ".join(f'"{n}"' for n in values) or '""'

    vex = BREAK_VEX.replace("NEXT_FLOATS", names(floats)).replace("NEXT_INTS", names(ints))
    vex = vex.replace("NEXT_VECTORS", names(vectors))
    # The checks, in THRESHOLDS' order: Distance adds the Rest Length.
    checks = []
    for key, *_ in THRESHOLDS:
        rest = '(hasprimattrib(0, "restlength") ? prim(0, "restlength", pr) : 0.0)' if key == "distance" else "0.0"
        checks.append(f'passes("{key}", chi("constraint_use{key}"), chf("constraint_{key}threshold"), '
                      f'chi("constraint_use{key}var") ? chf("constraint_{key}var") : -1.0, '
                      f'chi("constraint_scale{key}") ? chs("constraint_{key}attrib") : "", '
                      f'chi("constraint_{key}attribupdate"), {rest}, pr, id)')
    vex = vex.replace("PASSES", "\n        || ".join(checks))
    wrangle.parm("snippet").set(vex)
    for node in (cbegin, clive, begin, meta, live_in, wrangle, live, end, cend):
        node.moveToGoodPosition()

    enable = create(dopnet, "enablesolver", BREAK_ENABLE)
    enable.setInput(0, solver)
    rel = enable.relativePathTo(dopnet.parent())
    # Also only while there are constraints (as the `use_constraints` switch): the compiled loop costs about a
    # millisecond a step even over nothing.
    enable.parm("enable").setExpression(
        f'ch("{rel}/{BREAKS_TOGGLE}") && ch("{rel}/{BREAKS}") > 0 && npoints(opinputpath("..", {DOP_CONSTRAINTS})) > 0',
        hou.exprLanguage.Hscript)
    return enable
