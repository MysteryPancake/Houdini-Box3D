"""Builds the Box3D Configure digital asset (a SOP, like RBD Configure) with hython.

    hython tools/make_configure_sop.py <output .hda>

The build runs this after the plugin (CMakeLists.txt, BOX3DDOP_MAKE_HDA), writing
houdiniX.Y/otls/box3d_configure.hda. The plugin must be installed: the settings, their labels,
defaults and ranges are copied from Box3D Data, so the two always match.

What it is: Box3D Data's per-body settings as point attributes, laid out like RBD Configure: a list
of entries (Number of Attributes), each with Enable, a Group and Box3D Data's folders, every
setting with an add<name> checkbox. Only ticked settings are written. A point attribute wins over
Box3D Data, so this sets values per piece where Box3D Data sets them per object.

    Inputs / outputs: Geometry, Constraint Geometry (pink), Proxy Geometry, as the Box3D Solver
    SOP's first three. Geometry and Proxy are packed first, as RBD Configure does (b3d_prepare.py,
    the chain the Box3D Solver SOP also runs): packed primitives kept, the rest assembled into named
    packed pieces, every piece named on its point. target (packed Proxy when it has points, else
    packed Geometry) -> set_attributes (one point wrangle looping over the entries) -> the proxy
    output, or the geometry output without a proxy (or without geometry); constraints pass through.

Packing parameters, as RBD Configure's: Name Prefix, Transfer Simulation Attributes (onto pieces
assembled from the simulated stream), Transfer Render Geo Attributes (onto the Geometry's pieces;
the simulation ones too when there is no proxy), Create Path Attribute / Path Attribute and Create
Packed Fragments (the Assembles').

Like RBD Configure's wrangle, each write creates its attribute first if missing, with Box3D Data's
default (addpointattrib), then sets the point (setpointattrib). Never @ bindings: a binding creates
its attribute on every point at compile time, even inside an if that never runs, and an attribute
created that way gives every point 0 (active 0: static).

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
import b3d_prepare  # noqa: E402
import b3d_settings  # noqa: E402
from b3d_settings import GEOREP_VALUES, literal  # noqa: E402

TYPE_NAME = "box3d_configure"
LABEL = "Box3D Configure"
ICON = "DOP_box3d_data"  # installed by the build for Box3D Data (config/Icons)

# Inputs and outputs, matching the Box3D Solver SOP's first three. The settings go onto the proxy
# when it has points, else onto the geometry; everything else passes through.
LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry"]
INPUT_COLORS = {1: (1.0, 0.6, 0.8)}  # constraints pink, as the Box3D Solver SOP

# Labels and help for the settings whose attribute is not their name.
SPECIAL_LABELS = {
    "defaultshape": ("Collision Shape", "Collision shape of each piece. Writes bullet_georep."),
    "autofit": (None, "Fit Box, Sphere and Capsule shapes to each piece. Writes bullet_autofit."),
    "bodytype": (None, "Static never moves, Kinematic follows its animation, Dynamic is simulated. "
                       "Writes active and animated."),
    "startasleep": (None, "Pieces begin asleep until touched. Writes awake."),
}


warn = b3d_hda.warner("make_configure_sop")


MULTIPARM = "attributes"  # Number of Attributes, as RBD Configure

# Packing parameters, RBD Configure's names and defaults (its Attributes folder, above the entries).
TRANSFER_GROUPS = "* ^*inside ^*outside"  # RBD Configure's: groups onto the packed pieces


def packing_templates():
    path = hou.StringParmTemplate("path", "Path Attribute", 1, default_value=("op:`opfullpath('.')`",),
                                  help="The path attribute value of each packed piece.")
    path.setConditional(hou.parmCondType.DisableWhen, "{ createpath == 0 }")
    return [
        hou.StringParmTemplate("nameprefix", "Name Prefix", 1, default_value=(b3d_prepare.DEFAULT_PREFIX,),
                               help="Prefix of the names given to pieces that have none."),
        hou.StringParmTemplate("transfer_attributes", "Transfer Simulation Attributes", 1,
                               default_value=("v w",),
                               help="Attributes copied onto the pieces packed from the simulated geometry."),
        hou.StringParmTemplate("transfer_renderattributes", "Transfer Render Geo Attributes", 1,
                               default_value=("shop_materialpath path",),
                               help="Attributes copied onto the pieces packed from the Geometry input."),
        hou.ToggleParmTemplate("createpath", "Create Path Attribute", default_value=False,
                               help="Gives each packed piece a path attribute."),
        path,
        hou.ToggleParmTemplate("createpackedfragments", "Create Packed Fragments", default_value=True,
                               help="Packs pieces as fragments of one shared geometry instead of one geometry each."),
    ]


def toggle_name(name):
    """The checkbox of a setting (RBD Configure's add<name>), without the multiparm's #."""
    return f"add{name}"


def first_sentence(text):
    m = re.match(r"(.+?[.;])(\s|$)", text or "")
    return m.group(1) if m else (text or "")


def rewrite_condition(cond, names):
    """Box3D Data's disable-when for one multiparm entry: every name gets the entry's #, and in
    each `{ x == v }` group `addx# == 1` is added for every x that has a checkbox here, so it only
    applies while that setting is ticked."""
    def group(m):
        body = m.group(1)
        refs = [r for r in re.findall(r"(\w+)\s*[!=<>]=?", body) if r in names]
        for r in dict.fromkeys(refs):
            body = re.sub(rf"\b{r}\b", f"{r}#", body)
        extra = "".join(f" {toggle_name(r)}# == 1" for r in dict.fromkeys(refs))
        return "{" + body.rstrip() + extra + " }"
    return re.sub(r"\{([^}]*)\}", group, cond)


def settings(data):
    """Box3D Data's folders as (folder label, [(template, kind)]) (b3d_settings.settings)."""
    return b3d_settings.settings(data, warn)


def setting_vex(pt, kind):
    """VEX for one setting inside the entry loop (`num` is the entry number): if ticked, create
    the attribute with its default when missing, then set the point."""
    n, t = pt.name(), toggle_name(pt.name())

    def ch(fn, name=n):
        return f'{fn}("../{name}" + num)'

    def put(attr, default, value):
        return (f'if (!haspointattrib(0, "{attr}")) addpointattrib(0, "{attr}", {default}); '
                f'setpointattrib(0, "{attr}", @ptnum, {value});')

    body = None
    if kind == "bullet_georep":
        values = ", ".join(f'"{v}"' for v in GEOREP_VALUES)
        body = (f'string shapes[] = {{{values}}}; '
                + put("bullet_georep", '""', f'shapes[clamp({ch("chi")}, 0, len(shapes) - 1)]'))
    elif kind == "bullet_autofit":
        body = put("bullet_autofit", literal(pt, "i"), ch("chi"))
    elif kind == "active/animated":
        body = (f'int body = {ch("chi")}; '
                + put("active", "1", "body == 2") + " " + put("animated", "0", "body == 1"))
    elif kind == "awake":
        body = put("awake", "0" if pt.defaultValue() else "1", f'!{ch("chi")}')
    elif kind == "v":
        body = put(n, literal(pt, kind), ch("chv"))
    elif kind == "f":
        body = put(n, literal(pt, kind), ch("chf"))
    else:
        body = put(n, literal(pt, kind), ch("chi"))
    return f'    if (chi("../{t}" + num)) {{ {body} }}'


def wrangle_vex(rows):
    """The whole wrangle: RBD Configure's entry loop around every setting."""
    lines = [
        "// Box3D Configure: each enabled entry writes its ticked settings onto the points in its group.",
        f'for (int x = 1; x <= chi("../{MULTIPARM}"); x++)',
        "{",
        "    string num = itoa(x);",
        '    if (!chi("../enable" + num)) continue;',
        '    string group = chs("../group" + num);',
        '    if (group != "" && !(inpointgroup(0, group, @ptnum) || inprimgroup(0, group, @primnum))) continue;',
        "",
    ]
    lines += [setting_vex(pt, kind) for pt, kind in rows]
    lines.append("}")
    return "\n".join(lines)


def ui(folders):
    """The asset's parameters: a tabbed multiparm of entries (Enable, Group, then each Box3D Data
    folder with a checkbox per setting), as RBD Configure."""
    names = {pt.name() for _, rows in folders for pt, _ in rows}
    entry = [
        hou.ToggleParmTemplate("enable#", "Enable", default_value=True,
                               help="Write this entry's settings."),
        hou.StringParmTemplate(
            "group#", "Group", 1, help="The pieces (point or primitive group) to set. Empty sets every piece.",
            tags={"script_action": "import soputils\nkwargs['geometrytype'] = (hou.geometryType.Points,)\n"
                                   "kwargs['inputindex'] = 0\nsoputils.selectGroupParm(kwargs)",
                  "script_action_icon": "BUTTONS_reselect"}),
    ]
    entry[1].setConditional(hou.parmCondType.DisableWhen, "{ enable# == 0 }")
    for i, (label, rows) in enumerate(folders):
        items = []
        for pt, kind in rows:
            n = pt.name()
            value = pt.clone()
            value.setName(f"{n}#")
            lab, help_text = SPECIAL_LABELS.get(n, (None, None))
            if lab:
                value.setLabel(lab)
            attr = n if kind in ("f", "i", "v") else kind
            value.setHelp(help_text or f"{first_sentence(pt.help())} Writes the {attr} point attribute.")
            # The checkbox has a real label, hidden (an empty label shows the name instead).
            box = hou.ToggleParmTemplate(f"{toggle_name(n)}#", value.label(), default_value=False,
                                         join_with_next=True, help="Write this setting onto the pieces.")
            box.hideLabel(True)
            box.setConditional(hou.parmCondType.DisableWhen, "{ enable# == 0 }")
            cond = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
            new = ((rewrite_condition(cond, names) + " ") if cond else "") + \
                f"{{ {toggle_name(n)}# == 0 }} {{ enable# == 0 }}"
            value.setConditional(hou.parmCondType.DisableWhen, new)
            items += [box, value]
        # Every Box3D Data folder simple (always open) in each entry.
        entry.append(hou.FolderParmTemplate(f"b3dcfg_folder{i}_#", label, items,
                                            folder_type=hou.folderType.Simple))
    multi = hou.FolderParmTemplate(MULTIPARM, "Number of Attributes", entry,
                                   folder_type=hou.folderType.TabbedMultiparmBlock, default_value=1)
    group = hou.ParmTemplateGroup()
    group.append(hou.FolderParmTemplate("b3dcfg_packing", "Packing", packing_templates(),
                                        folder_type=hou.folderType.Simple))
    group.append(multi)
    return group


def build_network(sub, rows):
    """The asset's contents. The settings go onto the proxy when it has points, else onto the
    geometry; the other streams pass through unchanged."""
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

    # Both streams packed and named as RBD Configure does. Expressions are relative to the nodes
    # inside each prepare subnet: ../../ is the asset.
    asset_parms = {k: f'ch("../../{k}")' for k in ("createpath", "createpackedfragments")}
    asset_parms["path"] = 'chs("../../path")'
    sim_attrs = 'chs("../../transfer_attributes")'
    geo_attrs = (f'ifs(npoints("../../{proxy_in.name()}") > 0, "", {sim_attrs} + " ") + '
                 'chs("../../transfer_renderattributes")')
    packed = []
    for stream, attrs in ((geo_in, geo_attrs), (proxy_in, sim_attrs)):
        packed.append(b3d_prepare.prepare_input(
            sub, stream, f"pack_{stream.name()[3:]}", True, warn, prefix_expr='chs("../../nameprefix")',
            remove_unused_groups=True,  # RBD Configure's Split (RBD Bullet Solver leaves it off)
            assemble_parms={"transfer_groups": TRANSFER_GROUPS},
            assemble_exprs=dict(asset_parms, transfer_attributes=attrs)))
    geo_packed, proxy_packed = packed

    target = switch_on_proxy(sub, "target", geo_packed, proxy_packed, proxy_packed)  # what gets configured
    setter = sub.createNode("attribwrangle", "set_attributes")
    setter.setInput(0, target)
    setter.parm("snippet").set(wrangle_vex(rows))
    # With a proxy the geometry passes through (packed), unless there is none: then, as RBD Configure,
    # the configured proxy stands in for it.
    geo_out = switch_on_proxy(sub, "geometry_result", setter, geo_packed, proxy_packed,
                              extra=geo_packed)
    proxy_out = switch_on_proxy(sub, "proxy_result", proxy_packed, setter, proxy_packed)

    for idx, src in enumerate([geo_out, con_in, proxy_out]):
        out = sub.createNode("output", f"OUT_{['geometry', 'constraints', 'proxy'][idx]}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 0:
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    sub.layoutChildren()


def switch_on_proxy(sub, name, without, with_proxy, proxy, extra=None):
    """A Switch passing `with_proxy` while `proxy` has points, else `without`: `proxy` is its spare
    input, the selection npoints(-1) > 0 (as the Box3D Solver SOP and SideFX's solvers). `extra`
    (second spare input, -2) must have points too."""
    sw = sub.createNode("switch", name)
    sw.setInput(0, without)
    sw.setInput(1, with_proxy)
    expr = "npoints(-1) > 0"
    for i, node in enumerate([proxy] + ([extra] if extra is not None else [])):
        sw.addSpareParmTuple(hou.StringParmTemplate(
            f"spare_input{i}", f"Spare Input {i}", 1, string_type=hou.stringParmType.NodeReference,
            help=f"Geometry the expressions on this node read as input {-1 - i}.",
            tags={"opfilter": "!!SOP!!", "oprelative": "."}))
        sw.parm(f"spare_input{i}").set(sw.relativePathTo(node))
    if extra is not None:
        expr += " && npoints(-2) > 0"
    sw.parm("input").setExpression(expr, hou.exprLanguage.Hscript)
    return sw


def edit_dialog_script(definition):
    """Input / output labels in the DialogScript (hou has no setter for them)."""
    section = definition.sections().get("DialogScript")
    if section is None:
        warn("the asset has no DialogScript section: input / output labels not set")
        return
    lines = [ln for ln in section.contents().splitlines()
             if not re.match(r"\s*(inputlabel|outputlabel)\b", ln)]
    extra = [f'    inputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(LABELS)]
    extra += [f'    outputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(LABELS)]
    at = next((i + 1 for i, ln in enumerate(lines) if re.match(r"\s*label\b", ln)), 1)
    section.setContents("\n".join(lines[:at] + extra + lines[at:]) + "\n")


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "templates")
    try:
        data = dopnet.createNode("box3d_data", "box3d_data")
    except hou.OperationFailed:
        raise RuntimeError("the box3d_data DOP is not installed: build the plugin first")
    folders = settings(data)
    rows = [r for _, rs in folders for r in rs]
    print("make_configure_sop: settings: " + ", ".join(pt.name() for pt, _ in rows))

    geo = hou.node("/obj").createNode("geo", "build")
    for child in geo.children():
        child.destroy()
    sub = geo.createNode("subnet", TYPE_NAME)
    # The Packing parameters first (spare, replaced by the asset's interface below), so the expressions
    # build_network sets on the Assembles and wrangles find them while the network is built.
    spare = sub.parmTemplateGroup()
    spare.append(hou.FolderParmTemplate("b3dcfg_packing", "Packing", packing_templates(),
                                        folder_type=hou.folderType.Simple))
    sub.setParmTemplateGroup(spare)
    build_network(sub, rows)

    if os.path.exists(out_path):
        try:
            hou.hda.uninstallFile(out_path)
        except hou.OperationFailed:
            pass
        os.remove(out_path)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    asset = sub.createDigitalAsset(name=TYPE_NAME, hda_file_name=out_path, description=LABEL,
                                   min_num_inputs=1, max_num_inputs=len(LABELS))
    definition = asset.type().definition()
    definition.setParmTemplateGroup(ui(folders))
    definition.setMaxNumOutputs(len(LABELS))
    definition.setIcon(ICON)
    definition.setComment("Sets Box3D body settings as point attributes on the pieces in a group, "
                          "like RBD Configure. Point attributes win over Box3D Data.")
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    edit_dialog_script(definition)  # labels last, saved without the node (see make_solver_sop.py)
    definition.save(out_path)

    check(out_path)
    print(f"Box3D Configure asset: {os.path.normpath(out_path)}")


def check(out_path):
    """On a fresh instance: two unpacked boxes come out as two named packed pieces, friction and Body
    Type Static written by entry 1 to the group's piece only; with a proxy, the settings go onto it."""
    hou.hda.installFile(out_path, force_use_assets=True)
    geo = hou.node("/obj").createNode("geo", "probe_geo")
    for child in geo.children():
        child.destroy()
    merge = geo.createNode("merge", "boxes")
    for i in range(2):
        box = geo.createNode("box", f"box{i}")
        box.parmTuple("t").set((2.0 * i, 0.0, 0.0))
        merge.setInput(i, box)
    pts = geo.createNode("attribwrangle", "chosen")  # the first box's faces: group chosen
    pts.setInput(0, merge)
    pts.parm("class").set("primitive")
    pts.parm("snippet").set('if (@P.x < 1) setprimgroup(0, "chosen", @primnum, 1);')
    cfg = geo.createNode(TYPE_NAME, "probe")
    cfg.setInput(0, pts)
    for n, v in (("group1", "chosen"), ("addfriction1", 1), ("friction1", 0.25), ("addbodytype1", 1),
                 ("bodytype1", 0)):
        if cfg.parm(n) is None:
            warn(f"the asset has no {n} parameter")
            geo.destroy()
            return
        cfg.parm(n).set(v)
    if cfg.geometry(2) is not None and len(cfg.geometry(2).points()):
        warn("probe: the proxy output has points with no proxy connected")
    g = cfg.geometry()
    if g is None or len(g.prims()) != 2 or any("Packed" not in p.type().name() for p in g.prims()):
        warn(f"probe: expected 2 packed pieces, got {[p.type().name() for p in g.prims()] if g else None}")
    if g is not None and (g.findPointAttrib("name") is None or g.findPrimAttrib("name") is not None):
        warn("probe: the packed pieces' name is not a point attribute only")
    got = sorted((round(p.attribValue("friction"), 4), p.attribValue("active")) for p in g.points()) \
        if g is not None and g.findPointAttrib("friction") else []
    want = sorted([(0.25, 0),
                   (round(float(cfg.parmTuple("friction1").parmTemplate().defaultValue()[0]), 4), 1)])
    if got != want:
        warn(f"probe: (friction, active) per piece {got}, expected {want} (only the group's piece set, the "
             "other Box3D Data's defaults)")
    # With a proxy: the settings go onto it, the geometry passes through untouched (packed).
    cfg.setInput(2, pts)
    out0, out2 = cfg.geometry(0), cfg.geometry(2)
    if out0 is None or out0.findPointAttrib("friction") is not None:
        warn("probe: with a proxy, the geometry output got the settings too")
    if out2 is None or out2.findPointAttrib("friction") is None:
        warn("probe: with a proxy, the proxy output did not get the settings")
    geo.destroy()


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 0
    try:
        build(sys.argv[1])
    except Exception:  # noqa: BLE001 - never fail the plugin build over the asset
        warn("the Box3D Configure asset was not built:\n" + traceback.format_exc())
    return 0


if __name__ == "__main__":
    sys.exit(main())
