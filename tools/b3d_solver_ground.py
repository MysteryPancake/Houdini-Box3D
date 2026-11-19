"""Box3D Solver SOP: Ground Collision, as RBD Bullet Solver's Ground Collision folder.

A Ground Type menu (None, Ground Box, Height Field) picks, through a DOP Switch with a Null, a Ground
Plane DOP and a Static Object DOP, what joins the pieces. The Ground Box (the Ground Plane DOP) takes Position,
Rotation and Size.
The Static Object takes the geometry of a Height Field node, through a HeightField Transform for World
Scale, and Use Deforming Geometry. A Box3D Data after the Switch gives either one its friction, bounce,
rolling resistance and conveyor velocity. Size is the solver's Ground Box Size."""

import hou

import b3d_hda
import b3d_hda
from b3d_hda import collapsible
from b3d_solver_common import (
    create,
    GROUND_OBJECT,
    SHADE_GROUND,
    SHOW_GROUND,
    USE_GROUND,
    WORLD_SCALE,
    WORLD_SCALE_TOGGLE,
    warn,
)

GROUND_POS = "ground_pos"
GROUND_ROT = "ground_rot"
GROUND_SIZE = "groundsize"  # the solver's Ground Box Size, pulled from its tab into this folder (make_solver_sop.py)
GROUND_HEIGHTFIELD = "groundheightfield"  # the Height Field node's path (a node reference)
GROUND_DEFORMING = "ground_animategeo"
GROUND_MERGE = "ground_heightfield_in"  # reads the Height Field parameter (see finish_ground)
GROUND_HF_NULL = "GROUND_HEIGHTFIELD"  # the SOP the Static Object reads
GROUND_DATA = "ground_data"  # the Box3D Data DOP
# Box3D Data's Material settings that mean something on a static ground (damping does not).
GROUND_MATERIAL = ("friction", "bounce", "rollingresistance", "tangentvelocity")
MATERIAL_FOLDER = "Material"  # Box3D Data's folder holding them


def outer_name(name):
    return f"ground_{name}"


def scale_here(depth):
    """The World Scale, read from a node `depth` levels below the asset."""
    up = "../" * depth
    return f'if(ch("{up}{WORLD_SCALE_TOGGLE}"), ch("{up}{WORLD_SCALE}"), 1)'


def setp(node, name, value=None, expr=None):
    """Sets a parameter, or its Hscript expression, with a warning for a missing one."""
    p = node.parm(name)
    if p is None:
        warn(f"{node.path()} has no parameter {name}")
        return
    if expr is not None:
        p.setExpression(expr, hou.exprLanguage.Hscript)
    else:
        p.set(value)


def build_ground_sops(sub):
    """The Height Field chain in the asset: an Object Merge of the node named by the Height Field
    parameter, scaled by World Scale (a HeightField Transform, as RBD Bullet Solver), ending in the null
    the Static Object reads."""
    merge = create(sub, "object_merge", GROUND_MERGE)
    last = merge
    try:
        xform = create(sub, "heightfield_xform", "ground_heightfield_scale")
    except RuntimeError as e:
        warn(f"{e}; World Scale does not scale the ground height field")
        xform = None
    if xform is not None:
        xform.setInput(0, merge)
        setp(xform, "scale", expr=scale_here(1))
        use = create(sub, "switch", "switch_ground_scale")
        use.setInput(0, xform)
        use.setInput(1, merge)
        setp(use, "input", expr=f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}") == 1, 1)')
        last = use
    null = create(sub, "null", GROUND_HF_NULL)
    null.setInput(0, last)
    return null


def ground_visualization():
    """The Visualization tab's Ground folder, as RBD Bullet Solver's: Show Ground as its header toggle and
    Show Ground as (Wireframe, Shaded) as its header menu."""
    show = hou.ToggleParmTemplate(SHOW_GROUND, "Show Ground", default_value=True,
                                  help="Draws the ground in the viewport.")
    show.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_GROUND} == 0 }}")
    shade = hou.MenuParmTemplate(
        SHADE_GROUND, "Show Ground as", ("0", "1"), menu_labels=("Wireframe", "Shaded"), default_value=0,
        help="How a ground box is drawn. A height field is always shaded.")
    shade.setConditional(hou.parmCondType.DisableWhen, f"{{ {USE_GROUND} != 1 }}")
    b3d_hda.show_in_toolbar(show)  # the bar along the top of the viewport
    folder = collapsible("b3dsop_show_ground", "Ground", [show, shade])
    folder.setTags({**folder.tags(), "sidefx::header_parm": SHADE_GROUND, "sidefx::header_toggle": SHOW_GROUND})
    return folder


def finish_ground(asset):
    """Once the asset has its Height Field parameter: the Object Merge reads it. (Not earlier: a reference
    to a parameter that does not exist yet is a warning.)"""
    merge = asset.node(GROUND_MERGE)
    if merge is None:
        warn(f"{asset.path()} has no {GROUND_MERGE}: the ground height field is not read")
        return
    setp(merge, "objpath1", f'`chsop("../{GROUND_HEIGHTFIELD}")`')


def build_ground_dops(dopnet):
    """None / Ground Plane / Height Field behind a Switch, then the Box3D Data. Returns the last node."""
    none = create(dopnet, "null", "ground_none")
    plane = create(dopnet, "groundplane", "groundplane")
    setp(plane, "objname", GROUND_OBJECT)
    for axis in "xyz":
        setp(plane, f"t{axis}", expr=f'ch("../../{GROUND_POS}{axis}") * {scale_here(2)}')
        setp(plane, f"r{axis}", expr=f'ch("../../{GROUND_ROT}{axis}")')

    height = create(dopnet, "staticobject", "ground_heightfield")
    setp(height, "object_name", GROUND_OBJECT)
    setp(height, "soppath", f"../../{GROUND_HF_NULL}")
    setp(height, "animategeo", expr=f'ch("../../{GROUND_DEFORMING}")')
    # Volume collision in heightfield mode (indices of the Static Object's menus, as RBD Bullet Solver's),
    # so no distance field is built from the terrain.
    setp(height, "collisiondetection", 1)
    setp(height, "mode", 7)

    choose = create(dopnet, "switch", "switch_ground")
    choose.setInput(0, none)
    choose.setInput(1, plane)
    choose.setInput(2, height)
    setp(choose, "input", expr=f'ch("../../{USE_GROUND}")')

    data = create(dopnet, "box3d_data", GROUND_DATA)
    data.setInput(0, choose)
    b3d_hda.set_operations(data, "initial", warn)
    return data


def material_templates(data):
    """Box3D Data's Material settings for the ground (GROUND_MATERIAL), renamed ground_<name>.
    Returns (templates, [(inner name, outer name)])."""
    folder = next((pt for pt in data.parmTemplateGroup().entries()
                   if pt.type() == hou.parmTemplateType.Folder and pt.label() == MATERIAL_FOLDER), None)
    if folder is None:
        warn(f"{data.path()} has no {MATERIAL_FOLDER} folder: the ground has no material settings")
        return [], []
    templates, names = [], []
    for pt in folder.parmTemplates():
        if pt.name() in GROUND_MATERIAL:
            inner = pt.name()
            pt = pt.clone()
            pt.setName(outer_name(inner))
            templates.append(pt)
            names.append((inner, pt.name()))
    return templates, names


def ground_items(data):
    """The asset's Ground Collision folder (Size is added to it once it is pulled from the solver's tabs), and
    the links (inner node path, inner name, outer name) for the material settings. `data` is the Box3D Data node the folder takes its settings from."""
    material, names = material_templates(data)
    menu = hou.MenuParmTemplate(
        USE_GROUND, "Ground Type", ("0", "1", "2"),  # numeric tokens: the hide conditions compare numbers
        menu_labels=("None", "Ground Box", "Height Field"), default_value=0,
        help="A ground for the pieces to land on, besides the Collision input. A ground box is a very wide, "
             "deep box with a flat top: Size sets how big.")
    pos = hou.FloatParmTemplate(GROUND_POS, "Position", 3, default_value=(0.0, 0.0, 0.0),
                                help="A point on the ground box's top.")
    rot = hou.FloatParmTemplate(GROUND_ROT, "Rotation", 3, default_value=(0.0, 0.0, 0.0),
                                help="Turns the ground box, in degrees. It faces up before it is turned.")
    for pt in (pos, rot):
        pt.setConditional(hou.parmCondType.HideWhen, f"{{ {USE_GROUND} != 1 }}")
    field = hou.StringParmTemplate(
        GROUND_HEIGHTFIELD, "Height Field", 1, string_type=hou.stringParmType.NodeReference,
        tags={"opfilter": "!!SOP!!", "oprelative": "."},
        help="The node holding the height field. It needs a height layer.")
    deforming = hou.ToggleParmTemplate(GROUND_DEFORMING, "Use Deforming Geometry", default_value=False,
                                       help="Rebuilds the ground every step. Turn it on if the height field changes.")
    for pt in (field, deforming):
        pt.setConditional(hou.parmCondType.HideWhen, f"{{ {USE_GROUND} != 2 }}")
    contents = [menu, pos, rot, field, deforming]
    if material:
        physical = collapsible("b3dsop_ground_physical", "Physical", material)
        physical.setConditional(hou.parmCondType.HideWhen, f"{{ {USE_GROUND} == 0 }}")
        contents.append(physical)
    folder = collapsible("b3dsop_ground", "Ground Collision", contents)
    folder.setTags({**folder.tags(), "sidefx::header_parm": USE_GROUND})
    links = [(f"dopnet/{GROUND_DATA}", inner, outer) for inner, outer in names]
    return folder, links
