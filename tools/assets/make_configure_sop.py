"""Builds the Box3D Configure digital asset (a SOP, like RBD Configure) with hython.

    hython tools/assets/make_configure_sop.py <output .hda>

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
import b3d_materials  # noqa: E402
from b3d_configure_help import HELP_CARD  # noqa: E402
from b3d_layouts import CONFIGURE_LAYOUT  # noqa: E402
import b3d_prepare  # noqa: E402
import b3d_settings  # noqa: E402
from b3d_settings import GEOREP_VALUES, literal, num  # noqa: E402

TYPE_NAME = "box3d_configure"
LABEL = "Box3D Configure"
ICON = "DOP_box3d_data"  # installed by the build for Box3D Data (config/Icons)

# Inputs and outputs, matching the Box3D Solver SOP's first three, plus RBD Configure's fourth input
# for Use Bounds. The settings go onto the proxy when it has points, else onto the geometry.
LABELS = ["Geometry", "Constraint Geometry", "Proxy Geometry"]  # outputs
INPUT_LABELS = LABELS + ["Static Bounds Geometry"]
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


# RBD Configure's Bounds folder: which pieces count as inside for the Use Bounds checkboxes. Bounding
# Geometry and Volume use the fourth input. Its default is RBD Configure's (Bounding Geometry).
BOUNDING_TYPES = ("Bounding Box", "Bounding Geometry", "Bounding Volume")
INSIDE_GROUP = "__static_grp"  # RBD Configure's: the pieces inside the bounds
# Pin types: constraint_name of a pin line, the Data Names of the Box3D Solver SOP's relationships.
PIN_TYPES = [("Weld", "Weld"), ("Soft", "Soft")]


# RBD Configure's Visualize menu (its last item, Collision Padding, is replaced by In Bounds). The viewer-only
# Visualize output shows the pieces coloured by the chosen value; the output is not a real one, so nothing
# downstream sees the colours.
VISUALIZE = ["None", "Active", "Animated", "Deforming", "Sleeping", "Bounce", "Density", "Friction",
             "Rotational Stiffness", "Pinned", "In Bounds"]
SCALAR_MODES = {5: "bounce", 6: "density", 7: "friction", 8: "inertialtensorstiffness"}  # coloured on a ramp
# The histogram under the viewer state (RBD Configure's): a grid of HIST_COLS x HIST_ROWS cells, one column per
# value bin. A piece's bin is `val`, 0..HIST_BINS, set with its colour.
HIST_COLS, HIST_ROWS = 152, 100
HIST_BINS = HIST_COLS - 2
# The Bounds box as RBD Configure's BBox handle: dragged in the viewport, writing Size, Center and Rotation.
BBOX_HANDLE = ([(f"bbox{p}{a}", "boundingbox", "BBox", f"{p}{a}") for p in ("center", "size") for a in "xyz"] +
               [(f"bboxr{a}", "boundingbox", "BBox", f"r{a}") for a in "xyz"] +
               [("bboxonoff", "boundingbox", "BBox", "onoff")])
STATE_NAME = "box3d_configure.pystate"
STATE_FILE = "b3d_configure_state.py"  # the viewer state's source, beside this script
# Shown while any visualization is on (-1: the Visualize output), else the first output.
VIEW_CALLBACK = ("node = kwargs['node']\n"
                 "on = node.evalParm('visualize') > 0 or node.evalParm('scale') > 0\n"
                 "node.setOutputForViewFlag(-1 if on else 0)\n"
                 "node.hm().enterState(kwargs)")
# The asset's Python Module: re-enters the node's viewer state when a Visualization parameter changes with the node
# current but another state active (RBD Configure's enterState).
PYTHON_MODULE = """\
def enterState(kwargs):
    import toolutils
    node = kwargs["node"]
    viewer = toolutils.sceneViewer()
    state = node.type().definition().sections()["DefaultState"].contents()
    if node == viewer.currentNode() and viewer.currentState() != state:
        viewer.enterCurrentNodeState(False)
"""


# Material presets, as RBD Configure's Physical Attributes has them: Type, Material and State fill Density, Bounce and
# Friction. The table is ours (b3d_materials.py), put into the asset's Python Module.
# The preset's rows are the top of the folder with this label (Bounce and Friction).
PRESET_FOLDER = "Material"
MATERIAL_TYPES = b3d_materials.TYPES
MATERIAL_STATES = b3d_materials.STATES

MATERIAL_MODULE = """
MATERIALS = %(table)r
BOUNCES = %(bounces)r


def buildMaterialMenu(kwargs):
    node = kwargs["node"]
    # The Type menu's index: 0 None, then the types
    rows = MATERIALS.get(node.evalParm("materialtype" + str(kwargs["script_multiparm_index"])) - 1, [])
    return [item for i, row in enumerate(rows) for item in (str(i), row[1])]


def typeChanged(kwargs):
    node = kwargs["node"]
    node.parm("materialpreset" + str(kwargs["script_multiparm_index"])).set(0)
    materialPresetCallback(kwargs)


def materialPresetCallback(kwargs):
    node = kwargs["node"]
    number = str(kwargs["script_multiparm_index"])
    kind = node.evalParm("materialtype" + number) - 1
    rows = MATERIALS.get(kind)
    if not rows:
        return
    row = rows[min(node.evalParm("materialpreset" + number), len(rows) - 1)]
    state = node.evalParm("materialstate" + number)
    for name, value in (("density", row[0]), ("bounce", BOUNCES[kind]), ("friction", row[2 + state])):
        node.parm(name + number).set(value)
        node.parm("add" + name + number).set(1)
"""


def material_templates():
    """Material Preset's Type, Material and State: picking one fills the entry's Density, Bounce and Friction
    (and ticks them). Type None does nothing."""
    callback = {"script_callback_language": "python"}
    kind = hou.MenuParmTemplate(
        "materialtype#", "Type", ("0", "1", "2", "3", "4"), ("None",) + MATERIAL_TYPES, default_value=0,
        tags=dict(callback, script_callback="kwargs['node'].hm().typeChanged(kwargs)"),
        help="A kind of material: Material then fills Density, Bounce and Friction for the pieces.")
    preset = hou.IntParmTemplate(
        "materialpreset#", "Material", 1, default_value=(0,),
        tags=dict(callback, script_callback="kwargs['node'].hm().materialPresetCallback(kwargs)"),
        help="The material: its density, and its friction for the State below.")
    preset.setItemGeneratorScript("kwargs['node'].hm().buildMaterialMenu(kwargs)")
    preset.setItemGeneratorScriptLanguage(hou.scriptLanguage.Python)
    preset.setMenuType(hou.menuType.Normal)
    state = hou.MenuParmTemplate(
        "materialstate#", "State", ("0", "1"), MATERIAL_STATES, default_value=0,
        tags=dict(callback, script_callback="kwargs['node'].hm().materialPresetCallback(kwargs)"),
        help="Which friction the material has: with the surfaces dry and clean, or lubricated and greasy.")
    kind.setConditional(hou.parmCondType.DisableWhen, "{ enable# == 0 }")
    for pt in (preset, state):
        pt.setConditional(hou.parmCondType.DisableWhen, "{ materialtype# == 0 } { enable# == 0 }")
    # One row, as RBD Configure's Physical Attributes has them: only Type keeps its label
    for pt in (kind, preset):
        pt.setJoinWithNext(True)
    for pt in (preset, state):
        pt.hideLabel(True)
    return [kind, preset, state]


def visualization_templates():
    tags = {"script_callback": VIEW_CALLBACK, "script_callback_language": "python", "autoscope": "0000000000000000"}
    # An integer parameter with a menu, as RBD Configure's (not an ordinal menu).
    visualize = hou.IntParmTemplate("visualize", "Visualize", 1, default_value=(0,), min=0, max=len(VISUALIZE) - 1,
                                    menu_items=tuple(str(i) for i in range(len(VISUALIZE))),
                                    menu_labels=tuple(VISUALIZE), tags=tags,
                                    help="Colours the pieces in the viewport by this value. Bounce, Density, Friction "
                                         "and Rotational Stiffness run from blue (lowest) to red. Changes nothing "
                                         "downstream.")
    scale = hou.FloatParmTemplate("scale", "Exploded Scale", 1, default_value=(0.0,), min=0.0, max=2.0, tags=tags,
                                  help="Moves the pieces apart in the viewport to see inside. 0 is off.")
    scale.setConditional(hou.parmCondType.DisableWhen, "{ visualize == 0 }")
    b3d_hda.show_in_toolbar(visualize, scale)  # the bar along the top of the viewport, as RBD Configure's two
    return [visualize, scale]


def visualization_folder():
    """RBD Configure's Visualization folder: a first-level simple folder, whose parameters (Visualize, Exploded Scale)
    the viewer shows in a bar along the top of the viewport while the node is selected."""
    return hou.FolderParmTemplate("b3dcfg_visualization", "Visualization", visualization_templates(),
                                  folder_type=hou.folderType.Simple, tags={"group_type": "simple"})


def bounds_templates():
    btype = hou.MenuParmTemplate("boundingtype", "Bounding Type", ("0", "1", "2"), BOUNDING_TYPES, default_value=1,
                                 tags={"script_callback": "kwargs['node'].hm().enterState(kwargs)",
                                       "script_callback_language": "python"},
                                 help="What Use Bounds tests: this box, or the Static Bounds Geometry input's "
                                      "bounding box or volume.")
    items = [btype]
    for name, label, default in (("bboxsize", "Size", (1.0, 1.0, 1.0)), ("bboxcenter", "Center", (0.0, 0.0, 0.0)),
                                 ("bboxr", "Rotation", (0.0, 0.0, 0.0))):
        pt = hou.FloatParmTemplate(name, label, 3, default_value=default, help=f"The bounding box's {label.lower()}.")
        pt.setConditional(hou.parmCondType.DisableWhen, "{ boundingtype > 0 }")
        pt.setConditional(hou.parmCondType.HideWhen, "{ boundingtype > 0 }")
        items.append(pt)
    return items


def pin_templates():
    """RBD Configure's Pin, per entry: a constraint from the world to each piece."""
    add = hou.ToggleParmTemplate("addpin#", "Add Pin", default_value=False, join_with_next=True,
                                 help="Add a constraint pinning each piece to the world.")
    add.hideLabel(True)
    pin = hou.IntParmTemplate("pin#", "Pin", 1, default_value=(1,), min=0, max=1, min_is_strict=True,
                              max_is_strict=True, help="Pin the pieces.")
    ptype = hou.MenuParmTemplate("pintype#", "Type", tuple(str(i) for i in range(len(PIN_TYPES))),
                                 tuple(label for label, _ in PIN_TYPES), default_value=0,
                                 help="Weld holds rigidly. Soft holds with a spring.")
    bounds = hou.ToggleParmTemplate("usepinbounds#", "Use Bounds", default_value=False,
                                    help="Pin the pieces inside the bounds and free the others.")
    for pt in (pin, ptype, bounds):
        pt.setConditional(hou.parmCondType.DisableWhen, "{ addpin# == 0 } { enable# == 0 }")
    pin.setJoinWithNext(True)
    ptype.setJoinWithNext(True)
    return [add, pin, ptype, bounds]


def simple_folder(name, label, contents):
    """A simple folder, as RBD Configure's Bounds and Attributes."""
    return hou.FolderParmTemplate(name, label, contents, folder_type=hou.folderType.Simple)


def packing_templates():
    path = hou.StringParmTemplate("path", "Path Attribute", 1, default_value=("op:`opfullpath('.')`",),
                                  help="The path attribute value of each packed piece.")
    path.setConditional(hou.parmCondType.DisableWhen, "{ createpath == 0 }")
    create_path = hou.ToggleParmTemplate("createpath", "Create Path Attribute", default_value=False,
                                         join_with_next=True, help="Gives each packed piece a path attribute.")
    create_path.hideLabel(True)
    return [
        hou.StringParmTemplate("nameprefix", "Name Prefix", 1, default_value=(b3d_prepare.DEFAULT_PREFIX,),
                               help="Prefix of the names given to pieces that have none."),
        b3d_hda.attrib_menu(
            hou.StringParmTemplate("transfer_attributes", "Transfer Simulation Attributes", 1,
                                   default_value=("v w",),
                                   help="Attributes copied onto the pieces packed from the simulated geometry."),
            0, "point", extra="age v w"),
        b3d_hda.attrib_menu(
            hou.StringParmTemplate("transfer_renderattributes", "Transfer Render Geo Attributes", 1,
                                   default_value=("shop_materialpath path",),
                                   help="Attributes copied onto the pieces packed from the Geometry input."),
            0, "point prim"),
        create_path,
        path,
        hou.ToggleParmTemplate("createpackedfragments", "Create Packed Fragments", default_value=True,
                               help="Packs pieces as fragments of one shared geometry instead of one geometry each."),
    ]


def extra_folders():
    """Settings Box3D Data has no parameter for (point attributes only), as more folders after its
    own: (folder label, [(template, kind)]) like b3d_settings.settings. Each template is named after
    its attribute, and its default is the solver's when the attribute is missing."""
    F, I, T = hou.FloatParmTemplate, hou.IntParmTemplate, hou.ToggleParmTemplate
    S = hou.StringParmTemplate
    zero3 = (0.0, 0.0, 0.0)
    return [
        ("Initial State", [
            (F("v", "Velocity", 3, default_value=zero3, min=-10.0, max=10.0,
               help="Starting velocity, in units per second."), "v"),
            (F("w", "Angular Velocity", 3, default_value=zero3, min=-360.0, max=360.0,
               help="Starting spin, in degrees per second. Writes w in radians per second."), "w"),
            (T("enabled", "Enabled", default_value=True,
               help="Off leaves the pieces out of the simulation."), "i"),
            (T("deforming", "Deforming", default_value=False,
               help="Rebuild the collision shape every step, for pieces whose geometry changes."), "i"),
        ]),
        ("Motion", [
            (T("fastrotation", "Fast Rotation", default_value=False,
               help="No spin speed limit. For round pieces such as wheels."), "i"),
        ]),
        # RBD Configure's Speed Limit folder: the same names, labels, defaults and ranges (-1: no limit).
        ("Speed Limit", [
            (F("speedmin", "Speed Min", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The slowest a moving piece may go, in units per second. Below 0: no limit."), "f"),
            (F("speedmax", "Speed Max", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The fastest a piece may go, in units per second. Below 0: no limit."), "f"),
            (F("spinmin", "Spin Min", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The slowest a spinning piece may turn, in radians per second. Below 0: no limit."), "f"),
            (F("spinmax", "Spin Max", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The fastest a piece may turn, in radians per second. Below 0: no limit."), "f"),
            (F("accelmax", "Acceleration Max", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The most collisions and constraints may change a piece's velocity, per second. Below 0: no "
                    "limit."), "f"),
            (F("angaccelmax", "Angular Acceleration Max", 1, default_value=(-1.0,), min=-1.0, max=10.0,
               help="The most collisions and constraints may change a piece's spin, per second. Below 0: no "
                    "limit."), "f"),
        ]),
        ("Collision Filtering", [
            (b3d_hda.menu_script(
                S("collisiongroup", "Collision Group", 1, default_value=("",),
                  help="The group the pieces are in, as RBD Configure's. The Box3D Solver SOP turns the names into "
                       "Category and Mask."),
                "buildCollisionGroupMenu(kwargs, 0)", toggle=False), "s"),
            (b3d_hda.menu_script(
                S("collisionignore", "Collision Ignore", 1, default_value=("",),
                  help="Name patterns of the groups the pieces pass through, separated by spaces. Needs the Box3D Solver SOP."),
                "buildCollisionGroupMenu(kwargs, 0, True)"),
             "s"),
            (I("collision_category", "Category", 1, default_value=(-1,),
               help="Bits saying what the pieces are. -1 is every bit."), "i"),
            (I("collision_mask", "Mask", 1, default_value=(-1,),
               help="Bits of the categories the pieces collide with. -1 is every bit."), "i"),
            (I("collision_group", "Group", 1, default_value=(0,), min=-10, max=10,
               help="Pieces in the same group always collide if it's positive, never if it's negative."), "i"),
        ]),
    ]


def has_bounds(pt, kind):
    """Settings with a Use Bounds checkbox (RBD Configure's): Body Type and the on / off ones."""
    return kind in ("active/animated", "awake") or (
        kind == "i" and pt.type() == hou.parmTemplateType.Toggle)


def on_off_slider(value):
    """A checkbox as an integer slider clamped to 0 - 1, as RBD Configure's on / off settings beside Use Bounds: the
    slider takes the row's width, which puts Use Bounds at the right."""
    return hou.IntParmTemplate(value.name(), value.label(), 1, default_value=(int(value.defaultValue()),),
                                 min=0, max=1, min_is_strict=True, max_is_strict=True, help=value.help(),
                                 tags=value.tags())


# What RBD Configure starts with: Density and Bounce ticked, Bounce 0.25 (Box3D Data's is 0.5).
ON_BY_DEFAULT = ("density", "bounce")
SETTING_DEFAULTS = {"bounce": 0.25}

# Settings with a Var: each piece's value is scattered up and down by up to that fraction (RBD Configure's Var).
VAR_SETTINGS = ("density", "bounce", "friction", "inertialtensorstiffness")


def var_toggle_name(name):
    return f"use{name}var"


def var_name(name):
    return f"{name}var"


def bounds_name(name):
    return f"use{name}bounds"


def toggle_name(name):
    """The checkbox of a setting (RBD Configure's add<name>), without the multiparm's #."""
    return f"add{name}"


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
                f'setpointattrib(0, "{attr}", i@ptnum, {value});')

    def bounded(expr, outside):
        """`expr` inside the bounds; `outside` for the others while Use Bounds is ticked."""
        if not has_bounds(pt, kind):
            return expr
        return (f'(chi("../{bounds_name(n)}" + num) && !inpointgroup(0, "{INSIDE_GROUP}", i@ptnum)) '
                f'? ({outside}) : ({expr})')

    body = None
    if kind == "bullet_georep":
        values = ", ".join(f'"{v}"' for v in GEOREP_VALUES)
        body = (f'string shapes[] = {{{values}}}; '
                + put("bullet_georep", '""', f'shapes[clamp({ch("chi")}, 0, len(shapes) - 1)]'))
    elif kind == "bullet_autofit":
        body = put("bullet_autofit", literal(pt, "i"), ch("chi"))
    elif kind == "active/animated":  # outside the bounds: Dynamic
        body = (f'int body = {bounded(ch("chi"), "2")}; '
                + put("active", "1", "body == 2") + " " + put("animated", "0", "body == 1"))
    elif kind == "awake":
        asleep = ch("chi")
        body = put("awake", "0" if pt.defaultValue() else "1", f'!({bounded(asleep, "!" + asleep)})')
    elif kind == "v":
        body = put(n, literal(pt, kind), ch("chv"))
    elif kind == "w":  # degrees per second on the node, radians per second on the points
        body = put("w", "{0.0, 0.0, 0.0}", f'radians({ch("chv")})')
    elif kind == "f" and n in VAR_SETTINGS:
        # As RBD Configure: times a random 1 - Var to 1 + Var per piece (rand of its point number)
        var = ch("chf", var_name(n))
        scatter = f'(chi("../{var_toggle_name(n)}" + num) ? fit01(rand(i@ptnum), 1 - {var}, 1 + {var}) : 1.0)'
        body = put(n, literal(pt, kind), f'{ch("chf")} * {scatter}')
    elif kind == "f":
        body = put(n, literal(pt, kind), ch("chf"))
    elif kind == "s":
        body = put(n, '""', ch("chs"))
    else:
        body = put(n, literal(pt, kind), bounded(ch("chi"), "!" + ch("chi")))
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
        '    if (group != "" && !(inpointgroup(0, group, i@ptnum) || inprimgroup(0, group, i@primnum))) continue;',
        "",
    ]
    lines += [setting_vex(pt, kind) for pt, kind in rows]
    lines += [
        "    // Pin: marks the piece for add_pin_constraints (1 + the Type), 0 = free.",
        '    if (chi("../addpin" + num)) {',
        '        int pin = chi("../pin" + num);',
        f'        if (chi("../usepinbounds" + num) && !inpointgroup(0, "{INSIDE_GROUP}", i@ptnum)) pin = !pin;',
        f'        if (!haspointattrib(0, "{PIN_ATTRIB}")) addpointattrib(0, "{PIN_ATTRIB}", 0);',
        f'        setpointattrib(0, "{PIN_ATTRIB}", i@ptnum, pin ? chi("../pintype" + num) + 1 : 0);',
        "    }",
    ]
    lines.append("}")
    return "\n".join(lines)


PIN_ATTRIB = "__pin"
# One constraint line per pinned piece, from the world ("") to the piece, both ends at its point,
# appended to the Constraint Geometry (input 0); input 1 is the configured pieces.
PIN_TYPE_NAMES = ", ".join('"' + name + '"' for _, name in PIN_TYPES)
PIN_VEX = """\
// Pin constraints: one line from the world to each pinned piece (RBD Configure's add_pin_constraints).
string types[] = {%(types)s};
if (!haspointattrib(1, "%(pin)s")) return;
if (!haspointattrib(0, "name")) addpointattrib(0, "name", "");
if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", "");
for (int pt = 0; pt < npoints(1); pt++) {
    int pin = point(1, "%(pin)s", pt);
    if (pin <= 0) continue;
    vector P = point(1, "P", pt);
    int a = addpoint(0, P), b = addpoint(0, P);
    setpointattrib(0, "name", a, "");
    string piece = point(1, "name", pt);  // typed: point() alone could be read as an int
    setpointattrib(0, "name", b, piece);
    int pr = addprim(0, "polyline", a, b);
    setprimattrib(0, "constraint_name", pr, types[clamp(pin - 1, 0, len(types) - 1)]);
}
""" % {"types": PIN_TYPE_NAMES, "pin": PIN_ATTRIB}


def ui(folders):
    """The asset's parameters: a tabbed multiparm of entries (Enable, Group, then each Box3D Data
    folder with a checkbox per setting), as RBD Configure."""
    names = {pt.name() for _, rows in folders for pt, _ in rows}
    entry = [
        hou.ToggleParmTemplate("enable#", "Enable", default_value=True,
                               help="Write this entry's settings."),
        hou.StringParmTemplate(
            "group#", "Group", 1, help="The pieces to set: a point or primitive group, or a pattern such as piece* or @name=piece0. Empty sets every piece.",
            # RBD Configure's selection button: pick pieces (primitives) in the viewport.
            tags={"script_action": "import soputils\nnode = kwargs['node']\n"
                                   "kwargs['geometrytype'] = hou.geometryType.Primitives\n"
                                   "kwargs['inputindex'] = 0\nkwargs['nodepath'] = node.path()\n"
                                   "soputils.selectGroupParm(kwargs)",
                  "script_action_icon": "BUTTONS_reselect"}),
    ]
    b3d_hda.group_menu(entry[1], 0, "point prim")
    entry[1].setConditional(hou.parmCondType.DisableWhen, "{ enable# == 0 }")
    for i, (label, rows) in enumerate(folders):
        items = material_templates() if label == PRESET_FOLDER else []
        for pt, kind in rows:
            n = pt.name()
            value = pt.clone()
            value.setName(f"{n}#")
            if n in SETTING_DEFAULTS:
                value.setDefaultValue((SETTING_DEFAULTS[n],))
            lab, help_text = SPECIAL_LABELS.get(n, (None, None))
            if lab:
                value.setLabel(lab)
            attr = n if kind in ("f", "i", "v", "w", "s") else kind
            if has_bounds(pt, kind) and value.type() == hou.parmTemplateType.Toggle:
                value = on_off_slider(value)
            value.setHelp(help_text or f"{b3d_hda.first_sentence(pt.help())} Writes the {attr} point attribute.")
            # The checkbox has a real label, hidden (an empty label shows the name instead).
            box = hou.ToggleParmTemplate(f"{toggle_name(n)}#", value.label(), default_value=n in ON_BY_DEFAULT,
                                         join_with_next=True, help="Write this setting onto the pieces.")
            box.hideLabel(True)
            box.setConditional(hou.parmCondType.DisableWhen, "{ enable# == 0 }")
            cond = pt.conditionals().get(hou.parmCondType.DisableWhen, "")
            new = ((rewrite_condition(cond, names) + " ") if cond else "") + \
                f"{{ {toggle_name(n)}# == 0 }} {{ enable# == 0 }}"
            value.setConditional(hou.parmCondType.DisableWhen, new)
            items += [box, value]
            if n in VAR_SETTINGS:
                # One row, as RBD Configure's: the value, Var and its fraction (label hidden)
                value.setJoinWithNext(True)
                use_var = hou.ToggleParmTemplate(
                    f"{var_toggle_name(n)}#", "Var", default_value=False, join_with_next=True,
                    help="Scatter each piece's value up or down by up to the Var fraction. 0.1 on 1000 gives 900 to 1100.")
                amount = hou.FloatParmTemplate(
                    f"{var_name(n)}#", f"{value.label()} Var", 1, default_value=(0.1,), min=0.0, max=1.0,
                    min_is_strict=True, max_is_strict=True, help="The fraction: 1 scatters from 0 to twice the value.")
                amount.hideLabel(True)
                use_var.setConditional(hou.parmCondType.DisableWhen,
                                       f"{{ {toggle_name(n)}# == 0 }} {{ enable# == 0 }}")
                amount.setConditional(hou.parmCondType.DisableWhen,
                                      f"{{ {var_toggle_name(n)}# == 0 }} {{ {toggle_name(n)}# == 0 }} {{ enable# == 0 }}")
                items += [use_var, amount]
            if has_bounds(pt, kind):
                value.setJoinWithNext(True)
                use = hou.ToggleParmTemplate(f"{bounds_name(n)}#", "Use Bounds", default_value=False,
                                             help="Inside the bounds this value, outside the opposite "
                                                  "(Body Type: Dynamic).")
                use.setConditional(hou.parmCondType.DisableWhen,
                                   f"{{ {toggle_name(n)}# == 0 }} {{ enable# == 0 }}")
                items.append(use)
        # Every Box3D Data folder an open collapsible folder in each entry.
        entry.append(b3d_hda.collapsible(f"b3dcfg_folder{i}_#", label, items))
    entry.append(b3d_hda.collapsible("b3dcfg_pin_#", "Constraints", pin_templates()))
    multi = hou.FolderParmTemplate(MULTIPARM, "Number of Attributes", entry,
                                   folder_type=hou.folderType.TabbedMultiparmBlock, default_value=1)
    group = hou.ParmTemplateGroup()
    group.append(visualization_folder())
    group.append(simple_folder("b3dcfg_bounds", "Bounds", bounds_templates()))
    # The BBox handle's on / off (RBD Configure's hidden Show Bbox): the box shows while Bounding Type is Bounding Box.
    onoff = hou.ToggleParmTemplate("bboxonoff", "Show Bbox", default_value=False,
                                   default_expression='ch("boundingtype")==0',
                                   default_expression_language=hou.scriptLanguage.Hscript)
    onoff.hide(True)
    group.append(onoff)
    # RBD Configure's Attributes folder: the packing settings, then the entries.
    group.append(simple_folder("b3dcfg_attributes", "Attributes", packing_templates() + [multi]))
    return group


def build_network(sub, rows):
    """The asset's contents. The settings go onto the proxy when it has points, else onto the
    geometry; the other streams pass through unchanged."""
    ins = sub.indirectInputs()
    nulls = []
    for i, name in enumerate(["IN_geometry", "IN_constraints", "IN_proxy", "IN_bounds"]):
        n = sub.createNode("null", name)
        n.setInput(0, ins[i])
        if i in INPUT_COLORS:
            ins[i].setColor(hou.Color(INPUT_COLORS[i]))
            n.setColor(hou.Color(INPUT_COLORS[i]))
        nulls.append(n)
    geo_in, con_in, proxy_in, bounds_in = nulls

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
    setter.setInput(0, inside_bounds(sub, target, bounds_in))
    setter.parm("snippet").set(wrangle_vex(rows))
    # Pins go onto the constraints, then the markers leave the pieces.
    pins = sub.createNode("attribwrangle", "add_pin_constraints")
    pins.setInput(0, con_in)
    pins.setInput(1, setter)
    pins.parm("class").set("detail")
    pins.parm("snippet").set(PIN_VEX)
    # The wrangle only runs while a piece is pinned: the largest pin marker (0 where there is none) is promoted to the
    # detail first, and a Switch passes the constraints straight through when it is 0.
    pin_marker = b3d_hda.attribute_create(sub, "pin_marker", [(PIN_ATTRIB, "int", None)], warn, write=False)
    pin_marker.setInput(0, setter)
    pin_any = sub.createNode("attribpromote", "pin_any")
    pin_any.setInput(0, pin_marker)
    b3d_hda.set_token(pin_any.parm("inclass"), "point", warn)
    b3d_hda.set_token(pin_any.parm("outclass"), "detail", warn)
    b3d_hda.set_token(pin_any.parm("method"), "max", warn)
    pin_any.parm("inname").set(PIN_ATTRIB)
    pin_any.parm("useoutname").set(1)
    pin_any.parm("outname").set("__pin_any")
    pin_any.parm("deletein").set(0)
    pins_on = sub.createNode("switch", "switch_pins")
    pins_on.setInput(0, con_in)
    pins_on.setInput(1, pins)
    pins_on.parm("input").setExpression(
        'detail("../pin_any", "__pin_any", 0) > 0', hou.exprLanguage.Hscript)
    strip = sub.createNode("attribdelete", "strip_pin_marker")
    strip.setInput(0, setter)
    strip.parm("ptdel").set(PIN_ATTRIB)
    configured = sub.createNode("groupdelete", "strip_bounds_group")
    configured.setInput(0, strip)
    configured.parm("group1").set(INSIDE_GROUP)
    # With a proxy the geometry passes through (packed), unless there is none: then, as RBD Configure,
    # the configured proxy stands in for it.
    geo_out = switch_on_proxy(sub, "geometry_result", configured, geo_packed, proxy_packed,
                              extra=geo_packed)
    proxy_out = switch_on_proxy(sub, "proxy_result", proxy_packed, configured, proxy_packed)

    for idx, src in enumerate([geo_out, pins_on, proxy_out]):
        out = sub.createNode("output", f"OUT_{['geometry', 'constraints', 'proxy'][idx]}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 0:
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    visualize_network(sub, setter, rows)
    b3d_hda.layout(sub, CONFIGURE_LAYOUT)


def visualize_vex(defaults):
    """The two wranglers of the Visualize output: the value range of a ramped setting (detail), then each piece's
    colour (primitive, from its point's attribute or Box3D Data's default when the attribute is missing)."""
    # The setting and its default for each ramped mode, as a VEX if chain.
    pick = "\n".join(f'if (mode == {m}) {{ attrib = "{n}"; dflt = {num(defaults[n])}; }}'
                     for m, n in SCALAR_MODES.items())
    # point(): assigned to a typed variable, so an int attribute is read as a float.
    value = ("float value(string name; int pt; float dflt) {\n"
             "    float v = dflt;\n"
             "    if (haspointattrib(0, name)) v = point(0, name, pt);\n"
             "    return v;\n"
             "}")
    # The value of the setting on every point (0 when no setting is ramped), for Attribute Promote to find the range.
    rng = "\n".join([
        "// The setting being visualized, on every point: its range sets the colour ramp.",
        value,
        'int mode = chi("../visualize");',
        'string attrib = "";',
        "float dflt = 0.0;",
        pick,
        'f@__vis = attrib != "" ? value(attrib, i@ptnum, dflt) : 0.0;',
    ])
    colour = "\n".join([
        "// Colours each piece by the chosen value. A missing attribute is Box3D Data's default.",
        value,
        'int mode = chi("../visualize");',
        "if (mode == 0) return;",
        "int pt = primpoint(0, i@primnum, 0);",
        "vector lit = {1, 0, 0}, dim = {0.3, 0.3, 0.3}, c = {1, 1, 1};",
        'string attrib = "";',
        "float dflt = 0.0;",
        pick,
        'if (mode == 1) c = value("active", pt, 1.0) > 0.0 ? set(0, 1, 0) : set(0, 0, 0);',
        'else if (mode == 2) c = value("animated", pt, 0.0) > 0.0 ? lit : dim;',
        'else if (mode == 3) c = value("deforming", pt, 0.0) > 0.0 ? lit : dim;',
        'else if (mode == 4) c = value("awake", pt, 1.0) > 0.0 ? dim : lit;',
        "else if (mode >= 5 && mode <= 8) {",
        '    float v = point(0, "__vis", pt);',
        '    float lo = detail(0, "__vmin"), hi = detail(0, "__vmax");',
        "    float t = hi > lo ? clamp((v - lo) / (hi - lo), 0.0, 1.0) : 0.5;",
        f"    i@val = int(t * {HIST_BINS});",
        "    // blue (lowest) through white to red, RBD Configure's ramp",
        "    c = t < 0.5 ? lerp({0, 0, 1}, {1, 1, 1}, 2.0 * t) : lerp({1, 1, 1}, {1, 0, 0}, 2.0 * t - 1.0);",
        "}",
        f'else if (mode == 9) c = value("{PIN_ATTRIB}", pt, 0.0) > 0.0 ? lit : dim;',
        f'else if (mode == 10) c = inpointgroup(0, "{INSIDE_GROUP}", pt) ? lit : dim;',
        "v@Cd = c;",
    ])
    return rng, colour


def visualize_network(sub, source, rows):
    """The viewer-only Visualize output (Output Index -1, as RBD Configure's): `source` coloured by the Visualize
    menu, moved apart by Exploded Scale."""
    defaults = {"bounce": 0.0, "density": 1000.0, "friction": 0.5, "inertialtensorstiffness": 1.0}
    for pt, kind in rows:
        if pt.name() in defaults and kind == "f":
            defaults[pt.name()] = float(pt.defaultValue()[0])
    range_vex, colour_vex = visualize_vex(defaults)
    ranged = sub.createNode("attribwrangle", "vis_value")
    ranged.setInput(0, source)
    ranged.parm("class").set("point")
    ranged.parm("snippet").set(range_vex)
    upstream = ranged
    for method, name in (("min", "vis_min"), ("max", "vis_max")):
        promote = b3d_hda.create(sub, "attribpromote", name)
        promote.setInput(0, upstream)
        b3d_hda.set_token(promote.parm("inclass"), "point", warn)
        b3d_hda.set_token(promote.parm("outclass"), "detail", warn)
        b3d_hda.set_token(promote.parm("method"), method, warn)
        promote.parm("inname").set("__vis")
        promote.parm("useoutname").set(1)
        promote.parm("outname").set("__vmin" if method == "min" else "__vmax")
        promote.parm("deletein").set(0)
        upstream = promote
    coloured = sub.createNode("attribwrangle", "vis_color")
    coloured.setInput(0, upstream)
    coloured.parm("class").set("primitive")
    coloured.parm("snippet").set(colour_vex)

    histogram_network(sub, coloured)

    last = coloured
    try:
        exploded = b3d_hda.create(sub, ["explodedview::2.0", "explodedview"], "explode_pieces")
    except RuntimeError as err:
        warn(f"Exploded Scale will not work: {err}")
    else:
        exploded.setInput(0, coloured)
        b3d_hda.set_token(exploded.parm("centroidmethod"), "bbox", warn)
        exploded.parm("scale").setExpression('ch("../scale")', hou.exprLanguage.Hscript)
        last = switch_on(sub, "use_explode", coloured, exploded, 'ch("../scale") > 0')

    out = sub.createNode("output", "OUT_visualize")
    out.setInput(0, last)
    out.parm("outputidx").set(-1)


HISTOGRAM_VEX = f"""// One bar per value bin, as RBD Configure's: a cell is dark when its bin holds enough pieces for its height.
v@Cd = chramp("color", v@P.x);
f@Alpha = 0.6;
float vmax = detail(1, "__vmax");
int cols = {HIST_COLS - 1};

int x = i@primnum % cols;
int nums[];
int values = 0;
for (int z = 0; z < cols; z++)
{{
    int num = findattribvalcount(1, "prim", "val", z);
    append(nums, num);
    if (num) values += 1;
}}
// The lowest bin can hold nearly every piece: leave it out of the scale so the others stay readable.
if (vmax > 0 && values > 2 && nums[0] == max(nums))
    pop(nums, 0);
nums = sort(nums);
float divider = pop(nums);

int prims[] = findattribval(1, "prim", "val", x);
int numfound = len(prims);
if (numfound)
{{
    float y = (i@primnum / cols) * 0.01;
    if ((float(numfound) / divider) > y)
    {{
        v@Cd *= 0;
        f@Alpha = 0.9;
    }}
}}
"""


def histogram_network(sub, coloured):
    """The nodes the viewer state draws (HISTOGRAM, DISPLAY), as RBD Configure's. Not connected to an output:
    the state reads their geometry. DISPLAY carries the value range for the text."""
    grid = sub.createNode("grid", "hist_grid")
    grid.parm("orient").set("xy")
    grid.parmTuple("size").set((1.0, 1.0))
    grid.parmTuple("t").set((0.5, 0.0, 0.0))
    grid.parm("rows").set(HIST_ROWS)
    grid.parm("cols").set(HIST_COLS)
    bars = sub.createNode("attribwrangle", "hist_color")
    bars.setInput(0, grid)
    bars.setInput(1, coloured)
    bars.parm("class").set("primitive")
    bars.parm("snippet").set(HISTOGRAM_VEX)
    group = bars.parmTemplateGroup()
    group.append(hou.RampParmTemplate("color", "Color", hou.rampParmType.Color, default_value=3,
                                      default_basis=hou.rampBasis.Linear))
    bars.setParmTemplateGroup(group)
    bars.parm("color").set(hou.Ramp((hou.rampBasis.Linear,) * 3, (0.0, 0.5, 1.0),
                                    ((0, 0, 1), (1, 1, 1), (1, 0, 0))))
    # Screen placement: the state's screen_space draw scales it to the viewport.
    place = sub.createNode("xform", "hist_xform")
    place.setInput(0, bars)
    place.parm("xOrd").set("tsr")
    place.parmTuple("t").set((-3.65, -0.25, 0.0))
    place.parmTuple("s").set((0.125, 0.05, 1.0))
    for c, axis in zip(place.parmTuple("p"), "XYZ"):
        c.setExpression(f"$CE{axis}", hou.exprLanguage.Hscript)
    hist = sub.createNode("null", "HISTOGRAM")
    hist.setInput(0, place)
    display = sub.createNode("null", "DISPLAY")
    display.setInput(0, coloured)


def switch_on(sub, name, off, on, expr):
    """A Switch passing `on` while the HScript expression `expr` is true, else `off`."""
    sw = sub.createNode("switch", name)
    sw.setInput(0, off)
    sw.setInput(1, on)
    sw.parm("input").setExpression(expr, hou.exprLanguage.Hscript)
    return sw


def inside_bounds(sub, target, bounds_in):
    """RBD Configure's bounds: the point group INSIDE_GROUP holds the pieces inside the Bounds box, or
    inside the Static Bounds Geometry's bounding box or volume. With no bounds geometry, no group
    (every piece outside)."""
    box = sub.createNode("box", "bounds_box")
    for parm, src in (("size", "bboxsize"), ("t", "bboxcenter"), ("r", "bboxr")):
        for c, axis in zip(box.parmTuple(parm), "xyz"):
            c.setExpression(f'ch("../{src}{axis}")', hou.exprLanguage.Hscript)
    shape = sub.createNode("switch", "use_bounds_geometry")
    shape.setInput(0, box)
    shape.setInput(1, bounds_in)
    shape.parm("input").setExpression('ch("../boundingtype") > 0', hou.exprLanguage.Hscript)
    groups = []
    for name, boundtype in (("bounds_object", "usebobject"), ("bounds_volume", "usebvolume")):
        g = sub.createNode("groupcreate", name)
        g.setInput(0, target)
        g.setInput(1, shape)
        for parm, value in (("groupname", INSIDE_GROUP), ("grouptype", "point"), ("groupbase", 0),
                            ("groupbounding", 1), ("boundtype", boundtype)):
            if g.parm(parm) is None:
                warn(f"{g.path()} has no {parm}: Use Bounds may not work")
            else:
                g.parm(parm).set(value)
        groups.append(g)
    kind = sub.createNode("switch", "bounding_type")
    kind.setInput(0, groups[0])
    kind.setInput(1, groups[1])
    kind.parm("input").setExpression('ch("../boundingtype") == 2', hou.exprLanguage.Hscript)
    any_bounds = sub.createNode("switch", "any_bounds")
    any_bounds.setInput(0, target)
    any_bounds.setInput(1, kind)
    any_bounds.addSpareParmTuple(hou.StringParmTemplate(
        "spare_input0", "Spare Input 0", 1, string_type=hou.stringParmType.NodeReference,
        help="The bounds geometry the expression reads as input -1.",
        tags={"opfilter": "!!SOP!!", "oprelative": "."}))
    any_bounds.parm("spare_input0").set(any_bounds.relativePathTo(shape))
    any_bounds.parm("input").setExpression("nprims(-1) > 0", hou.exprLanguage.Hscript)
    return any_bounds


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


def build(out_path):
    hou.hipFile.clear(suppress_save_prompt=True)
    dopnet = hou.node("/obj").createNode("dopnet", "templates")
    try:
        data = dopnet.createNode("box3d_data", "box3d_data")
    except hou.OperationFailed:
        raise RuntimeError("the box3d_data DOP is not installed: build the plugin first")
    folders = settings(data) + extra_folders()
    rows = [r for _, rs in folders for r in rs]

    geo = hou.node("/obj").createNode("geo", "build")
    for child in geo.children():
        child.destroy()
    sub = geo.createNode("subnet", TYPE_NAME)
    # The Packing parameters first (spare, replaced by the asset's interface below), so the expressions
    # build_network sets on the Assembles and wrangles find them while the network is built.
    spare = sub.parmTemplateGroup()
    spare.append(visualization_folder())
    spare.append(simple_folder("b3dcfg_bounds", "Bounds", bounds_templates()))
    spare.append(simple_folder("b3dcfg_attributes", "Attributes", packing_templates()))
    sub.setParmTemplateGroup(spare)
    build_network(sub, rows)

    asset = b3d_hda.create_asset(sub, out_path, TYPE_NAME, LABEL, 1, len(INPUT_LABELS))
    definition = asset.type().definition()
    definition.setParmTemplateGroup(ui(folders))
    definition.setMaxNumOutputs(len(LABELS))
    definition.setIcon(ICON)
    b3d_hda.set_tab_submenu(definition, warn)
    b3d_hda.add_python_module(definition, PYTHON_MODULE + MATERIAL_MODULE % {
        "table": b3d_materials.table(), "bounces": b3d_materials.bounces()})
    definition.setComment("Sets Box3D body settings as point attributes on the pieces in a group, "
                          "like RBD Configure. Point attributes win over Box3D Data.")
    definition.addSection("Help", HELP_CARD)  # the help card (F1 on the node)
    b3d_hda.add_viewer_state(definition, STATE_NAME, STATE_FILE)
    definition.updateFromNode(asset)
    definition.save(out_path, asset)
    b3d_hda.edit_dialog_script(definition, INPUT_LABELS, LABELS, warn, BBOX_HANDLE)  # labels last, saved without the node (see make_solver_sop.py)
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
    pts.parm("snippet").set('if (v@P.x < 1) setprimgroup(0, "chosen", i@primnum, 1);')
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
    # Bounds and Pin: a box around the first piece only. Body Type Static with Use Bounds makes it static
    # and the other Dynamic; Pin adds one Weld line from the world to it.
    settings_to = (("group1", ""), ("boundingtype", 0), ("bboxcenter", (0.0, 0.0, 0.0)), ("bboxsize", (1.5, 1.5, 1.5)),
                   ("usebodytypebounds1", 1), ("addpin1", 1), ("usepinbounds1", 1))
    for n, v in settings_to:
        target = cfg.parmTuple(n)
        if target is None:
            warn(f"the asset has no {n} parameter: bounds / pin not checked")
            break
        target.set(v if isinstance(v, tuple) else (v,))
    else:
        g = cfg.geometry()
        actives = sorted(round(p.position()[0]) for p in g.points() if p.attribValue("active") == 0) if g else None
        if actives != [0]:
            warn(f"probe: Use Bounds made the pieces at x {actives} static, expected only x 0")
        if g is not None and (g.findPointAttrib(PIN_ATTRIB) or g.findPointGroup(INSIDE_GROUP)):
            warn("probe: the pin marker or the bounds group reached the output")
        cons = cfg.geometry(1)
        lines = [(pr.attribValue("constraint_name"), [pt.attribValue("name") for pt in pr.points()])
                 for pr in cons.prims()] if cons is not None and cons.findPrimAttrib("constraint_name") else []
        if len(lines) != 1 or lines[0][0] != "Weld" or lines[0][1][0] != "":
            warn(f"probe: Pin gave constraint lines {lines}, expected one Weld line from the world to the piece")
        for n, _ in settings_to:
            cfg.parmTuple(n).revertToDefaults()
        cfg.parm("group1").set("chosen")
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
