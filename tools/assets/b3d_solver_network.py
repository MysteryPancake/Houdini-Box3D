"""Box3D Solver SOP: the inner SOP network (input preparation, scaling, start-frame switches, build_network)."""

import hou

import b3d_hda
import b3d_prepare
from b3d_hda import layout, set_operation
from b3d_layouts import DOP_LAYOUT, SOLVER_LAYOUT
from b3d_solver_activeage import build_active_age_sop, build_age_sop, build_constraint_age_sop
from b3d_solver_common import (
    COLLISION,
    COLLISION_TYPE,
    CONSTRAINTS,
    CONSTRAINTS_TOGGLES,
    copy_attributes,
    create,
    DOP_COLLISION,
    DOP_CONSTRAINTS,
    DOP_PIECES,
    DOP_STATIC,
    GEO,
    IMPACTS,
    INPUT_COLORS,
    POINTS_KEEP,
    POINTS_TOGGLES,
    PROXY,
    RELATIONSHIPS,
    set_menu,
    set_named,
    GUIDE_GEO,
    SHAPES_COLOR,
    SHAPES_DATA,
    SHAPES_TOGGLE,
    GROUND_OBJECT,
    SHADE_GROUND,
    SHOW_COLLISION,
    SHOW_COLLISION_SHAPES,
    SHOW_CONSTRAINTS,
    FALSE_COLOR_MODE,
    FALSE_COLOR_ATTRIB,
    SHOW_IMPACTS,
    SHOW_GUIDES,
    GUIDE_RADIUS,
    COLLISION_COLOR,
    SHOW_GROUND,
    VISUALIZE,
    USE_GROUND,
    FETCH_GEOMETRY,
    FORCES,
    FORCES_OUTPUTS,
    REPLICATE,
    set_toggles,
    STATIC_NULL,
    switch_on_spare_input,
    START_FRAME,
    TRANSFER_ATTRIBUTES,
    TRANSFER_TO,
    USE_COLLISIONS,
    WORLD_SCALE,
    WORLD_SCALE_TOGGLE,
    warn,
)
from b3d_solver_emit import (emit_constraints, emit_pieces, emit_render, replicate_points, replicate_render,
                             replicate_source)
from b3d_solver_ground import build_ground_sops
from b3d_solver_dops import build_dops


# Input preparation (b3d_prepare.py, shared with Box3D Configure): pack mode for what is simulated,
# collision mode for the collision, names mode for the Geometry / Proxy outputs. The rest and the
# orient / pscale / scale bake happen in the Box3D Packed Objects, as in RBD Bullet Solver.
PACK_TRANSFER_ATTRIBS = "v w active animated deforming"  # onto assembled pieces' points


def prepare_input(sub, src, name, pack, collision=False):
    """b3d_prepare.prepare_input as this asset uses it, with RBD Bullet Solver's Assemble settings:
    Create Path Attribute on, Path Attribute op:<assemble>/pack when packing (Housekeeping,
    Collisions), v w transferred in names mode (Housecleaning)."""
    if pack:
        parms = {"createpath": 1, "path": b3d_prepare.PACK_PATH,
                 "transfer_attributes": PACK_TRANSFER_ATTRIBS}
    else:
        parms = {"createpath": 1, "transfer_attributes": "v w"}
    return b3d_prepare.prepare_input(sub, src, name, pack, warn, assemble_parms=parms, collision=collision)


# The collision's rest from the start frame (input 1: the prepared collision there), as RBD Bullet
# Solver's collision repack does with a Time Shift: restxform stays the starting transform while an
# animated collider is re-imported every step, and orient is the rotation since (row-normalised
# rotations, mirrors flipped, as the solver splits packed matrices). Matched by name, else by number.
COLLISION_REST_VEX = """\
int rp = findattribval(1, "point", "name", s@name);
if (rp < 0) rp = i@ptnum;
int now[] = pointprims(0, i@ptnum);
int was[] = pointprims(1, rp);
if (len(now) != 1 || len(was) != 1) return;
// Both read as 4x4 first: matrix3(primintrinsic(...)) picks primintrinsic()'s matrix3 overload,
// which reads nothing from this 4x4 intrinsic.
matrix rest = primintrinsic(1, "packedfulltransform", was[0]);
matrix current = primintrinsic(0, "packedfulltransform", now[0]);
matrix3 m[] = array(matrix3(rest), matrix3(current));
matrix3 r[];
foreach (matrix3 a; m) {
    vector x = normalize(set(a.xx, a.xy, a.xz));
    vector y = normalize(set(a.yx, a.yy, a.yz));
    vector z = normalize(set(a.zx, a.zy, a.zz));
    if (determinant(a) < 0) z = -z;
    push(r, set(x, y, z));
}
4@restxform = rest;
v@pivot = point(1, "P", rp);
p@orient = quaternion(transpose(r[0]) * r[1]);
"""

# Solve on Creation Frame, as RBD Bullet Solver: solvefirstframe on the Solver tab drives the pieces' Packed Object,
# collision_solvefirstframe (Advanced > Collision, following the first by default) the collider's.
SOLVE_FIRST = "solvefirstframe"
COLLISION_SOLVE_FIRST = "collision_solvefirstframe"
SOLVE_FIRST_PARM = "solvefirstframe"  # the Box3D Packed Object's Solve on Creation Frame (Empty Object's)

# Export Broken Constraints (RBD Bullet Solver's constraint_keepbroken): off, broken constraints are
# removed from the constraints output.
KEEP_BROKEN = "constraint_keepbroken"

# Constraint points anchored to the world (no piece name) keep their imported position.
RESET_ANCHORS_VEX = "v@P = point(1, \"P\", i@ptnum);"

# Every primitive removed, its points kept: an Add SOP's Delete Geometry But Keep the Points (one
# bulk delete; removeprim per primitive is far slower on many pieces). The wrangle is the fallback
# when no such toggle is found.
POINTS_ONLY_PARM = "keep"  # Add SOP's Delete Geometry But Keep the Points
POINTS_ONLY_VEX = "removeprim(0, i@primnum, 0);"

# World Scale (RBD Bullet Solver's usescale / Bullet World Scale): the inputs are scaled by it for the
# simulation and every output scaled back, so pieces far from about 1 unit simulate at a size Box3D
# is tuned for. Lengths and speeds among the solver's and Box3D Data's settings follow it
# (SCALED_SETTINGS), densities by its cube, so masses stay the same.
SCALE_FACTOR = f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}"), 1)'  # from a node inside the asset
SCALE_ACTIVE = f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}") != 1, 0)'  # switches: input 1 when scaling
# (inner node relative to the asset, parameter tuple, power of the scale it follows).
SCALED_SETTINGS = [("dopnet/box3d_solver", n, 1) for n in
                   ("gravity", "maxlinearspeed", "contactpushspeed", "bouncethreshold",
                    "contactrecycledistance", "impactminspeed", "groundsize")] + \
    [("dopnet/pieces", "density", -3)] + \
    [("dopnet/pieces", n, 1) for n in ("sleepthreshold", "tangentvelocity")] + \
    [("dopnet/collision", "tangentvelocity", 1), ("dopnet/collision_static_data", "tangentvelocity", 1),
     ("dopnet/ground_data", "tangentvelocity", 1)]
# (The collider's density and sleep threshold are not promoted: it is never dynamic.)

# World Scale on joints: the power of the scale each inner relationship setting follows. Masses stay
# the same, so lengths, linear speeds and forces follow the scale and torques its square, while
# angles, Hz and ratios keep their values. Float primitive attributes multiply these settings, so
# they follow too. Every float setting of Box3D's nodes is listed (the build warns otherwise), so a new
# setting in Box3DJoints.h needs a power here.
JOINT_SCALE_POWERS = {name: power for power, names in (
    (1, ("restlength", "min_length", "max_length", "lower_translation", "upper_translation",
         "target_translation", "lower_suspension_limit", "upper_suspension_limit", "linear_velocity",
         "force_threshold", "max_motor_force", "max_tension_force", "max_compression_force",
         "max_velocity_force", "max_spring_force")),
    (2, ("torque_threshold", "max_motor_torque", "max_velocity_torque", "max_spring_torque", "max_torque",
         "max_spin_torque", "max_steering_torque")),
    (0, ("constraint_stiffness", "constraint_damping_ratio", "stiffness", "damping_ratio", "linear_stiffness",
         "linear_damping_ratio", "angular_stiffness", "angular_damping_ratio", "target_rotation", "cone_angle", "cone_angle_y",
         "lower_twist_angle", "upper_twist_angle", "motor_velocity", "target_angle", "lower_angle",
         "upper_angle", "motor_speed", "angular_velocity", "suspension_stiffness", "suspension_damping_ratio",
         "spin_speed", "steering_stiffness", "steering_damping_ratio", "target_steering_angle",
         "lower_steering_limit", "upper_steering_limit")),
) for name in names}
# Motor Speed is degrees per second on a revolute joint, units per second on these.
JOINT_SCALE_BY_NODE = {("prismatic", "motor_speed"): 1, ("distance", "motor_speed"): 1}
# SideFX's nodes: only what the solver reads with a length or force in it. Glue's strength is an
# impulse; Spring's is Hooke's k, force per length, so it stays.
SIDEFX_SCALE_POWERS = {"glue": {"strength": 1}, "hard": {"restlength": 1}, "spring": {"restlength": 1},
                       "soft": {"restlength": 1}}
# The constraint input: the attributes that replace a setting instead of multiplying it (vectors), and
# the rest length the solver measured for a distance joint (auto_restlength, -1 = none yet).
SCALE_CONSTRAINTS_VEX = """\
float s = chf("scale");
if (hasprimattrib(0, "linear_velocity")) {
    vector lv = prim(0, "linear_velocity", i@primnum);
    setprimattrib(0, "linear_velocity", i@primnum, lv * s);
}
if (hasprimattrib(0, "auto_restlength")) {
    float r = prim(0, "auto_restlength", i@primnum);
    if (r >= 0) setprimattrib(0, "auto_restlength", i@primnum, r * s);
}
"""
# FORCES: PRESOLVE's POP forces write force / torque in the scaled world. Scaled after it, so their
# values are real units (forces follow the scale, torques its square).
SCALE_FORCES_VEX = """\
float s = chf("scale");
if (s == 1) return;
// The bindings, not point(0, ...): a POP Wrangle has no first input, so a handle on 0 sees no points. They add
// the two attributes when no force wrote them (zeros, which the solver reads and clears).
v@force *= s;
v@torque *= s * s;
"""
# The pieces after packing (Transform scaled P, v, w and the packed transforms): Bullet's
# scale_density_and_attribs (density / s^3, w back, as the Transform scaled it, the speed and accel
# limits by s), plus the hand-set shape sizes, the sleep threshold and the drag's target velocity,
# which Bullet leaves.
SCALE_ATTRIBS_VEX = """\
float s = chf("scale");
if (haspointattrib(0, "density")) setpointattrib(0, "density", i@ptnum, 1 / pow(s, 3), "mult");
if (haspointattrib(0, "w")) setpointattrib(0, "w", i@ptnum, 1 / s, "mult");
foreach (string n; {"bullet_primS", "bullet_primT", "bullet_radius", "bullet_length", "sleepthreshold",
                          "tangentvelocity", "speedmin", "speedmax", "accelmax", "targetv"})
    if (haspointattrib(0, n)) setpointattrib(0, n, i@ptnum, s, "mult");
"""
# Outputs scaled back (chf("scale") is 1 / World Scale): Bullet's scale_simpoints, scale_attribs and
# scale_impacts. Joint feedback: force and distance scale with length, torque with its square.
UNSCALE_POINTS_VEX = """\
float s = chf("scale");
v@P *= s;
// Not v@pivot / v@v: a binding creates its attribute even where it is never reached.
if (haspointattrib(0, "pivot")) setpointattrib(0, "pivot", i@ptnum, s, "mult");
if (haspointattrib(0, "v")) setpointattrib(0, "v", i@ptnum, s, "mult");
"""
UNSCALE_CONSTRAINTS_VEX = """\
float s = chf("scale");
foreach (string n; {"force", "distance", "current_length", "current_translation", "current_speed",
                    "motor_force", "impact"})
    if (hasprimattrib(0, n)) setprimattrib(0, n, i@primnum, s, "mult");
foreach (string n; {"torque", "motor_torque", "spin_torque"})
    if (hasprimattrib(0, n)) setprimattrib(0, n, i@primnum, s * s, "mult");
if (hasprimattrib(0, "auto_restlength")) {
    float r = prim(0, "auto_restlength", i@primnum);
    if (r >= 0) setprimattrib(0, "auto_restlength", i@primnum, r * s);
}
"""
UNSCALE_IMPACTS_VEX = """\
float s = chf("scale");
v@P *= s;
foreach (string n; {"position", "v", "impulse"})
    if (haspointattrib(0, n)) setpointattrib(0, n, i@ptnum, s, "mult");
"""

# Collision shapes: the Box3DShapes polylines (white, dimmed by body state) tinted by Shape Color.
SHAPES_TINT_VEX = f'''\
v@Cd *= set(ch("../{SHAPES_COLOR}r"), ch("../{SHAPES_COLOR}g"), ch("../{SHAPES_COLOR}b"));
'''

# False colour on the constraint lines in the guide, as RBD Bullet Solver's False Color Mode: the chosen attribute
# (a float on the primitives) scaled by its largest absolute value, blue to red. The value is copied to `__fc`, an
# Attribute Promote finds the largest into the detail `__fc_top`, then the colour is set. `f@fcvalue` is bound (the
# wrangle's Bindings tab) to the attribute the mode picks, FALSE_COLOR_BOUND: a missing one reads 0.
FALSE_COLOR_VALUE_VEX = "f@__fc = abs(f@fcvalue);\n"
FALSE_COLOR_BOUND = (  # Hscript, relative to the wrangle
    f'ifs(ch("../{FALSE_COLOR_MODE}") == 6, chs("../{FALSE_COLOR_ATTRIB}"), '
    f'ifs(ch("../{FALSE_COLOR_MODE}") == 1, "angle", ifs(ch("../{FALSE_COLOR_MODE}") == 2, "distance", '
    f'ifs(ch("../{FALSE_COLOR_MODE}") == 3, "force", ifs(ch("../{FALSE_COLOR_MODE}") == 4, "impact", '
    f'ifs(ch("../{FALSE_COLOR_MODE}") == 5, "torque", "__none"))))))')
FALSE_COLOR_VEX = '''\
float top = detail(0, "__fc_top");
vector colour = hasprimattrib(0, "Cd") ? prim(0, "Cd", i@primnum) : {1, 1, 1};
if (top > 0) colour = lerp({0, 0.4, 1}, {1, 0.1, 0}, f@__fc / top);
v@Cd = colour;
'''

# The constraint lines' radius in the guide: Guide Radius times the Guide Scale of the line's type (its
# `constraint_name` against each type's Data Name), set on the line's points (PolyWire reads it there).
def guide_radius_vex():
    lines = ["int prims[] = pointprims(0, i@ptnum);",
             'string cname = len(prims) && hasprimattrib(0, "constraint_name") ? '
             'prim(0, "constraint_name", prims[0]) : "";',
             "float scale = 1;"]
    for i, (_, name) in enumerate(RELATIONSHIPS):
        lines.append(f'{"if" if i == 0 else "else if"} (cname == chs("../{name}_dataname")) '
                     f'scale = ch("../{name}_guidescale");')
    lines.append(f'f@guide_radius = max(ch("../{GUIDE_RADIUS}") * scale, 0.0001);')
    return "\n".join(lines) + "\n"


# The collider's shapes: the same polylines tinted by the collision Color instead.
COLLISION_SHAPES_TINT_VEX = f'''v@Cd *= set(ch("../{COLLISION_COLOR}r"), ch("../{COLLISION_COLOR}g"), ch("../{COLLISION_COLOR}b"));
'''

# The collision object's own geometry in the guide, dimmed (Bullet's `@Cd *= 0.5`; grey when it has no Cd).
COLLISION_GUIDE_VEX = """\
v@Cd = hasprimattrib(0, "Cd") ? v@Cd * 0.5 : {0.5, 0.5, 0.5};
"""

# Pieces the solver deleted (dead, reaped) leave the Geometry / Proxy outputs: RBD Bullet Solver's
# remove_deleted, on the names-mode pieces (input 1: the simulation points; input 2, Geometry only:
# what was simulated, so pieces that never were, such as geometry without a proxy piece, stay).
# removeprim only runs for the deleted few.
REMOVE_DELETED_VEX = """\
string name = prim(0, "name", i@primnum);
if (name == "")
    name = point(0, "name", i@ptnum);
int del = findattribval(1, "point", "name", name) < 0;
if (npoints(2))
{
    int pt = findattribval(2, "prim", "name", name);
    if (pt < 0)
        pt = findattribval(2, "point", "name", name);
    del *= pt >= 0;
}
if (del) removeprim(0, i@primnum, 1);
"""


def start_frame_expr(dopnet):
    """HScript for the asset's Start Frame from a node directly inside the asset (the promoted
    parameter has the DOP network's name), or None."""
    return f'ch("../{START_FRAME}")' if dopnet.parm(START_FRAME) is not None else None


def before_start(sub, dopnet, name, result, original):
    """RBD Bullet Solver's switchif_beforestart: `original` before the Start Frame, else `result`."""
    sw = create(sub, "switchif", name)
    sw.setInput(0, result)
    sw.setInput(1, original)
    start = start_frame_expr(dopnet)
    if start is None or sw.parm("expr1") is None:
        warn(f"{sw.path()}: no Start Frame or no Enable expression; the simulated result is shown always")
        if sw.parm("expr1") is not None:
            sw.parm("expr1").set(0)
    else:
        sw.parm("expr1").setExpression(f"$FF < {start}", hou.exprLanguage.Hscript)
    return sw


def build_ground_guide(sub, dopnet, empty):
    """The ground in the viewport. A Ground Box is the Ground Plane DOP's DisplayGeometry (DOP Import, scaled back
    by World Scale) with the collision box on top (Size, Position and Rotation of Ground Collision, in the inputs'
    units); each is closed polygons while Show Ground as is Solid, or lines while Wireframe. A Height Field is the
    DOP Import of its geometry. Chosen by Ground Type and Show Ground, nothing before the Start Frame."""
    def ground_import(name, data_path):
        node = create(sub, "dopimport", name)
        node.parm("doppath").set("../dopnet")
        node.parm("objpattern").set(GROUND_OBJECT)
        set_menu(node, "importstyle", FETCH_GEOMETRY)
        set_named(node, "geodatapath", data_path)
        set_named(node, "donotsim", 1)
        set_named(node, "keepworldspacepos", 0)
        return node

    # The box: its top face on the local plane y = 0, Size deep below it, then posed by Position / Rotation.
    box = create(sub, "box", "ground_box")
    for i, axis in enumerate("xyz"):
        box.parm(f"size{axis}").setExpression(f'ch("../groundsize{axis}")', hou.exprLanguage.Hscript)
    box.parm("ty").setExpression('-ch("../groundsizey") / 2', hou.exprLanguage.Hscript)
    pose = create(sub, "xform", "pose_ground_box")
    pose.setInput(0, box)
    for axis in "xyz":
        pose.parm(f"t{axis}").setExpression(f'ch("../ground_pos{axis}")', hou.exprLanguage.Hscript)
        pose.parm(f"r{axis}").setExpression(f'ch("../ground_rot{axis}")', hou.exprLanguage.Hscript)
    try:
        lines = create(sub, "convertline", "ground_box_edges")
        lines.setInput(0, pose)
    except RuntimeError as e:
        warn(f"{e}; Show Ground as Wireframe draws the solid box")
        lines = pose
    style = create(sub, "switch", "switch_ground_style")
    style.setInput(0, pose)
    style.setInput(1, lines)
    style.parm("input").setExpression(f'ch("../{SHADE_GROUND}")', hou.exprLanguage.Hscript)
    box_out = style

    plane = ground_import("import_ground_plane", "DisplayGeometry")
    try:
        plane_lines = create(sub, "convertline", "ground_plane_lines")
        plane_lines.setInput(0, plane)
    except RuntimeError as e:
        warn(f"{e}; the ground plane is drawn solid whatever Show Ground as says")
        plane_lines = plane
    plane_style = create(sub, "switch", "switch_ground_plane_style")
    plane_style.setInput(0, plane)
    plane_style.setInput(1, plane_lines)
    plane_style.parm("input").setExpression(f'ch("../{SHADE_GROUND}")', hou.exprLanguage.Hscript)
    plane_scaled = scale_xform(sub, "unscale_ground_plane", plane_style, inverse=True)
    plane_out = create(sub, "merge", "merge_ground_box")
    plane_out.setInput(0, plane_scaled)
    plane_out.setInput(1, box_out)

    field = ground_import("import_ground_field", "Geometry")
    field_out = field
    try:
        xform = create(sub, "heightfield_xform", "unscale_ground_field")
        xform.setInput(0, field)
        if xform.parm("scale") is not None:
            xform.parm("scale").setExpression(f"1 / {SCALE_FACTOR}", hou.exprLanguage.Hscript)
        unscale = create(sub, "switch", "switch_unscale_ground_field")
        unscale.setInput(0, xform)
        unscale.setInput(1, field)
        unscale.parm("input").setExpression(
            f'if(ch("../{WORLD_SCALE_TOGGLE}"), ch("../{WORLD_SCALE}") == 1, 1)', hou.exprLanguage.Hscript)
        field_out = unscale
    except RuntimeError as e:
        warn(f"{e}; World Scale does not scale the ground height field in the viewport")

    choose = create(sub, "switch", "switch_ground_guide")
    choose.setInput(0, empty)
    choose.setInput(1, plane_out)
    choose.setInput(2, field_out)
    choose.parm("input").setExpression(f'ch("../{USE_GROUND}") * ch("../{SHOW_GROUND}") * ch("../{VISUALIZE}")', hou.exprLanguage.Hscript)
    return before_start(sub, dopnet, "switchif_beforestart_ground", choose, empty)


# Transform Pieces is the largest cost of the asset's frame (about 2.4 microseconds a piece, even with everything asleep),
# almost all of it in the nodes that match the pieces to the points by name. For the usual case (the pieces carry a
# `name` on their points, and the only attributes to copy are `w`) one wrangle gives the same result: it finds each
# piece's point once, moves `P` and the normals by the point's motion matrix and writes `w` and `v`. (A Transform by
# Attribute with a matrix attribute per point, and a pass for `w` and `v`, took a third as long again.) Only attribute
# bindings are written (no setpointattrib), so it runs in parallel. Anything else, such as an `age` to copy, other
# Transfer Attributes or a point attribute of a type Transform Pieces would also move (a vector, quaternion or position
# other than `P`), keeps Transform Pieces.
MOVE_PIECES_VEX = """int pt = findattribval(1, "point", "name", s@name);
if (pt < 0)
    return;
matrix xf = pointtransform(1, pt);
vector w = point(1, "w", pt);
v@P *= xf;
v@w = w;
v@v = cross(w, v@P - point(1, "pivot", pt) * xf) + point(1, "v", pt);
"""
# With normals on the pieces: N moves by the inverse transpose of the matrix, as Transform by Attribute does. A second
# wrangle, not a bound attribute left out of Attributes to Create: that list also decides what may be written, so N
# would be made on pieces that have none.
MOVE_NORMALS_VEX = MOVE_PIECES_VEX + "v@N = normalize(v@N * transpose(invert(matrix3(xf))));\n"
# Packed pieces: a packed primitive's rotation, scale and shear are its transform (a 3x3 intrinsic), which the point
# wrangle would leave alone, so its own wrangle turns it too: the transform times the motion matrix's 3x3, the
# same product Transform Pieces makes (a sheared piece tells the order apart). `setprimintrinsic` on the point's own
# primitive does not stop it running in parallel.
MOVE_PACKED_VEX = MOVE_PIECES_VEX + """int prims[] = pointprims(0, @ptnum);
if (len(prims) > 0)
{
    matrix3 m = primintrinsic(0, "transform", prims[0]);
    setprimintrinsic(0, "transform", prims[0], m * matrix3(xf));
}
"""
# Whether the fast path does what Transform Pieces would: the pieces carry `name` on their points and no age, the
# simulation points carry no age and no scale (an animated piece's, which the wrangle does not apply), the attributes to
# copy are no more than age and w, and no point attribute besides P, N, w and v has a type Transform by Attribute would
# move. Then by what the primitives are: all polygons (a Mesh or a volume would need more) take the wrangle; all packed
# (PackedGeometry or PackedFragment) and without N, the packed one; else Transform Pieces. 0: Transform Pieces, 1: the
# wrangle, 2: the one with normals (the polygons have N), 3: the packed one.
FAST_TRANSFORM_EXPR = (
    '(lambda n, g, s: (lambda ok, has_n, polys, packed: ((1 + has_n) if polys else (3 if packed and not has_n else 0)) '
    'if ok else 0)('
    'g is not None and s is not None and g.findPointAttrib("name") is not None '
    'and g.findPointAttrib("age") is None and s.findPointAttrib("age") is None '
    'and s.findPointAttrib("scale") is None '
    'and set(n.parm("{attribs}").eval().split()) <= {{"age", "w"}} '
    'and all(a.qualifier() not in ("Position", "Vector", "Quaternion", "Normal", "Matrix") '
    'or a.name() in ("P", "N", "v", "w") for a in g.pointAttribs()), '
    'g is not None and g.findPointAttrib("N") is not None, '
    'g is not None and g.primCount() == g.countPrimType("Poly"), '
    'g is not None and g.primCount() == g.countPrimType("PackedGeometry") + g.countPrimType("PackedFragment")))'
    '(hou.pwd().parent(), hou.pwd().parent().node("{removed}").geometry(), '
    'hou.pwd().parent().node("{points}").geometry())')


def fast_transform(sub, removed, sim_points, xform):
    """The Geometry output's Transform Pieces with the faster paths beside it: a Switch picking one of the four (the
    others do not cook). Returns the Switch."""
    moves = []
    for name, snippet, writes in (("move_pieces_geometry", MOVE_PIECES_VEX, "P w v"),
                                  ("move_pieces_normals_geometry", MOVE_NORMALS_VEX, "P N w v"),
                                  ("move_pieces_packed_geometry", MOVE_PACKED_VEX, "P w v")):
        move = create(sub, "attribwrangle", name)
        move.setInput(0, removed)
        move.setInput(1, sim_points)
        move.parm("class").set("point")
        move.parm("snippet").set(snippet)
        move.parm("vex_precision").set("64")  # Auto rounds P to float: pieces far from the origin keep their position
        set_named(move, "exportlist", writes)  # what it may write: `w` and `v` are made if missing
        moves.append(move)
    pick = create(sub, "switch", "switch_fast_transform")
    pick.setInput(0, xform)
    for i, move in enumerate(moves):
        pick.setInput(i + 1, move)
    pick.parm("input").setExpression(
        FAST_TRANSFORM_EXPR.format(attribs=TRANSFER_ATTRIBUTES, removed=removed.name(), points=sim_points.name()),
        hou.exprLanguage.Python)
    return pick


def rest_source(sub, dopnet, src, name):
    """What the Geometry / Proxy outputs move: the input as of the Start Frame (Time Shift), as RBD Bullet Solver's
    timeshift1, since the simulation points carry each piece's motion since then; an input that moves on its own would
    otherwise move twice (a kinematic piece drawn twice as far). With a `deforming` point attribute on the input the
    current frame instead, as Bullet's switchif_deforming. Returns the Switch If."""
    start = start_frame_expr(dopnet)
    if start is None:
        warn("no Start Frame on the DOP network: the Geometry output moves the current input")
        return src
    at_start = create(sub, "timeshift", f"at_start_{name}")
    at_start.setInput(0, src)
    if at_start.parm("frame") is None:
        warn(f"{at_start.path()} has no Frame: the Geometry output moves the current input")
        at_start.destroy()
        return src
    at_start.parm("frame").setExpression(start, hou.exprLanguage.Hscript)
    pick = create(sub, "switchif", f"switchif_deforming_{name}")
    pick.setInput(0, at_start)
    pick.setInput(1, src)
    ok = all(set_menu(pick, parm, token) for parm, token in (("testinput", "second"), ("type1", "attrib"),
                                                             ("attribowner1", "any")))
    ok = all([set_named(pick, "attributes1", "deforming"), set_named(pick, "enable1", 1), ok])
    if not ok:
        warn(f"{pick.path()}: the Geometry output may move the current input")
    return pick


def scale_xform(sub, name, src, inverse=False):
    """A Transform SOP scaling `src` uniformly by the World Scale (or its inverse), and the Switch that uses it
    only while scaling. Returns the switch."""
    xf = create(sub, "xform", name)
    xf.setInput(0, src)
    p = xf.parm("scale")
    if p is None:
        warn(f"{xf.path()} has no Uniform Scale (scale); World Scale ignored there")
    else:
        p.setExpression(f"1 / {SCALE_FACTOR}" if inverse else SCALE_FACTOR, hou.exprLanguage.Hscript)
    # Behind a Switch that takes it only while scaling (as scale_wrangle's): a Transform by 1 still copies every point.
    sw = create(sub, "switch", f"switch_{name}")
    sw.setInput(0, src)
    sw.setInput(1, xf)
    sw.parm("input").setExpression(SCALE_ACTIVE, hou.exprLanguage.Hscript)
    return sw


def scale_wrangle(sub, name, src, snippet, inverse, run_over="point"):
    """A wrangle with a `scale` spare parameter (the World Scale, or its inverse) running `snippet`,
    and the Switch that uses it only while scaling, as Bullet's switch_scale*. Returns the switch."""
    wr = create(sub, "attribwrangle", name)
    wr.setInput(0, src)
    wr.parm("class").set(run_over)
    wr.addSpareParmTuple(hou.FloatParmTemplate(WORLD_SCALE, "Scale", 1, default_value=(1.0,)))
    wr.parm(WORLD_SCALE).setExpression(f"1 / {SCALE_FACTOR}" if inverse else SCALE_FACTOR,
                                       hou.exprLanguage.Hscript)
    wr.parm("snippet").set(snippet)
    wr.parm("vex_precision").set("64")  # as the move wrangles: P is scaled
    sw = create(sub, "switch", f"switch_{name}")
    sw.setInput(0, src)
    sw.setInput(1, wr)
    sw.parm("input").setExpression(SCALE_ACTIVE, hou.exprLanguage.Hscript)
    return sw


def finish_false_color(asset):
    """False Color Mode's bound attribute (the asset's parameters must exist for the expression)."""
    node = asset.node("false_color_value")
    if node is None:
        warn("no false_color_value: False Color Mode does nothing")
        return
    node.parm("bindname1").setExpression(FALSE_COLOR_BOUND, hou.exprLanguage.Hscript)


def scale_settings(asset):
    """SCALED_SETTINGS: each inner channel reference times the World Scale to its power."""
    for inner_path, name, power in SCALED_SETTINGS:
        inner = asset.node(inner_path)
        tup = inner.parmTuple(name) if inner is not None else None
        if tup is None:
            warn(f"World Scale: no {inner_path}/{name}; it is not scaled")
            continue
        rel = inner.relativePathTo(asset)
        factor = f'if(ch("{rel}/{WORLD_SCALE_TOGGLE}"), ch("{rel}/{WORLD_SCALE}"), 1)'
        for p in tup:
            try:
                expr = p.expression()
            except hou.OperationFailed:
                expr = None
            if not expr:
                warn(f"World Scale: {p.path()} is not a channel reference; it is not scaled")
                continue
            if power == 1:
                p.setExpression(f"({expr}) * {factor}", hou.exprLanguage.Hscript)
            else:
                p.setExpression(f"({expr}) * pow({factor}, {power})", hou.exprLanguage.Hscript)


def scale_joint_settings(asset):
    """World Scale on the inner relationship nodes: each setting with a length or force in it becomes
    its value times the scale to its power (JOINT_SCALE_POWERS, SIDEFX_SCALE_POWERS), Set Initial so
    the expression is used. Box3D's nodes' float settings must all be classified."""
    for _, name in RELATIONSHIPS:
        node = asset.node(f"dopnet/{name}")
        if node is None:
            continue
        box3d = node.type().name().startswith("box3d_")
        rel = node.relativePathTo(asset)
        factor = f'if(ch("{rel}/{WORLD_SCALE_TOGGLE}"), ch("{rel}/{WORLD_SCALE}"), 1)'
        for pt in node.parmTemplateGroup().entriesWithoutFolders():
            pname = pt.name()
            if pt.type() != hou.parmTemplateType.Float or pname.startswith("parmop"):
                continue
            # (A setting on the Constraints tab is already a channel reference: the scale multiplies it.)
            if not box3d:
                power = SIDEFX_SCALE_POWERS.get(name, {}).get(pname, 0)
            elif (name, pname) in JOINT_SCALE_BY_NODE:
                power = JOINT_SCALE_BY_NODE[(name, pname)]
            elif pname in JOINT_SCALE_POWERS:
                power = JOINT_SCALE_POWERS[pname]
            else:
                warn(f"World Scale: {node.path()}/{pname} has no power in JOINT_SCALE_POWERS; it is not scaled")
                continue
            if not power:
                continue
            for p in node.parmTuple(pname):
                try:
                    expr = p.expression()
                except hou.OperationFailed:
                    expr = None
                base = f"({expr})" if expr else repr(p.eval())
                scaled = factor if power == 1 else f"pow({factor}, {power})"
                p.setExpression(f"{base} * {scaled}", hou.exprLanguage.Hscript)
            set_operation(node, pname, "initial", warn)


def scale_forces(dopnet):
    """A POP Wrangle in the Multiple Solver right after FORCES/PRESOLVE scaling the pieces' force and
    torque (SCALE_FORCES_VEX), so POP force values are in real units under World Scale. The other
    solvers move one input down, each keeping the output it is wired from."""
    multi = dopnet.node("multisolver")
    if multi is None:
        warn("World Scale: no multisolver; FORCES are not scaled")
        return
    wr = create(dopnet, "popwrangle", "scale_forces")
    wr.addSpareParmTuple(hou.FloatParmTemplate(WORLD_SCALE, "Scale", 1, default_value=(1.0,)))
    rel = wr.relativePathTo(dopnet.parent())
    wr.parm(WORLD_SCALE).setExpression(f'if(ch("{rel}/{WORLD_SCALE_TOGGLE}"), ch("{rel}/{WORLD_SCALE}"), 1)',
                                       hou.exprLanguage.Hscript)
    if wr.parm("snippet") is None:
        warn(f"{wr.path()} has no snippet: FORCES are not scaled")
        wr.destroy()
        return
    wr.parm("snippet").set(SCALE_FORCES_VEX)
    # Behind an Enable Solver DOP: with World Scale off (or 1) the wrangle is not cooked, where it would still visit
    # every piece each step to return at once.
    enable = create(dopnet, "enablesolver", "enable_scale_forces")
    enable.setInput(0, wr)
    enable.parm("enable").setExpression(
        f'ch("{enable.relativePathTo(dopnet.parent())}/{WORLD_SCALE_TOGGLE}") && '
        f'ch("{enable.relativePathTo(dopnet.parent())}/{WORLD_SCALE}") != 1', hou.exprLanguage.Hscript)
    wired = {c.inputIndex(): (c.inputNode(), c.outputIndex()) for c in multi.inputConnections()}
    # The input PRESOLVE is wired to (objects, emission before the step, PRESOLVE, Box3D Solver, POSTSOLVE, ...).
    forces = dopnet.node(FORCES)
    presolve = next((i for i, (node, out) in wired.items()
                     if node == forces and out == FORCES_OUTPUTS.index("PRESOLVE")), None)
    if presolve is None:
        warn("World Scale: FORCES' PRESOLVE is not wired into the multisolver; FORCES are not scaled")
        enable.destroy()
        wr.destroy()
        return
    for i in sorted((i for i in wired if i > presolve), reverse=True):
        node, out = wired[i]
        multi.setInput(i + 1, node, out)
    multi.setInput(presolve + 1, enable)


def collision_rest(sub, dopnet, collision):
    """The prepared collision with its rest from the start frame (COLLISION_REST_VEX). Returns the
    node feeding the DOP network's collision input."""
    at_start = create(sub, "timeshift", "collision_at_start")
    at_start.setInput(0, collision)
    p = at_start.parm("frame")
    if dopnet.parm(START_FRAME) is None or p is None:
        warn("no Start Frame on the DOP network or no Frame on the Time Shift: the collision's rest is "
             "its current transform")
    else:  # the asset's promoted Start Frame (same name as the DOP network's)
        p.setExpression(f'ch("../{START_FRAME}")', hou.exprLanguage.Hscript)
    rest = create(sub, "attribwrangle", "collision_rest")
    rest.setInput(0, collision)
    rest.setInput(1, at_start)
    rest.parm("snippet").set(COLLISION_REST_VEX)
    return rest


def build_network(sub):
    """The asset's contents, inside the subnet `sub`. Returns the nodes whose parameters the
    asset promotes: (dopnet, solver, pieces)."""
    ins = sub.indirectInputs()
    for i, rgb in INPUT_COLORS.items():
        ins[i].setColor(hou.Color(rgb))
    nulls = []
    for i, name in enumerate(["IN_geometry", "IN_constraints", "IN_proxy", "IN_collision"]):
        n = create(sub, "null", name)
        n.setInput(0, ins[i])
        if i in INPUT_COLORS:
            n.setColor(hou.Color(INPUT_COLORS[i]))
        nulls.append(n)

    # World Scale on every input, as RBD Bullet Solver's transform1 / 2 / 4 / 5 (the before-start
    # outputs pass the unscaled inputs).
    scaled = [scale_xform(sub, f"world_scale_{n[3:]}", nulls[i]) for i, n in
              enumerate(["IN_geometry", "IN_constraints", "IN_proxy", "IN_collision"])]

    # What is simulated: the proxy when connected, else the geometry; packed and named (pack mode).
    sim_source = create(sub, "switch", "sim_source")
    sim_source.setInput(0, scaled[GEO])
    sim_source.setInput(1, scaled[PROXY])
    switch_on_spare_input(sim_source, nulls[PROXY])
    sim_prepared = prepare_input(sub, sim_source, "prepare_pieces", pack=True)
    # Collision: packed and named, rest from the start frame.
    collision_prepared = prepare_input(sub, scaled[COLLISION], "prepare_collision", pack=True, collision=True)
    # Bullet's collisiongroup / collisionignore names as collision bits, numbered over both.
    sim_pieces = b3d_prepare.collision_names(sub, "collision_names_pieces", sim_prepared, collision_prepared, warn)
    collision_pieces = b3d_prepare.collision_names(sub, "collision_names_collision", collision_prepared,
                                                   sim_prepared, warn)
    sim_scaled = scale_wrangle(sub, "scale_attributes", sim_pieces, SCALE_ATTRIBS_VEX, inverse=False)
    # Emit RBDs: the emitting pieces renamed with the frame (b3d_solver_emit.py); emit_flags looks them up.
    sim_emitted, emit_flags = emit_pieces(sub, sim_scaled)
    # Replicate RBDs: each piece's name as __sourcename, which copies made in the simulation carry.
    sim_emitted = replicate_source(sub, sim_emitted)

    # Ground Collision: the Height Field chain (the DOP network reads its end).
    build_ground_sops(sub)

    # The DOP network, its context geometry three streams (read inside by Geometry Source, CONTEXT_GEOMETRY).
    dopnet = create(sub, "dopnet", "dopnet")
    dopnet.setInput(DOP_PIECES, build_active_age_sop(sub, build_age_sop(sub, sim_emitted)))
    dopnet.setInput(DOP_CONSTRAINTS, build_constraint_age_sop(sub, emit_constraints(sub, scale_wrangle(
        sub, "scale_constraint_attribs", scaled[CONSTRAINTS], SCALE_CONSTRAINTS_VEX, inverse=False,
        run_over="primitive"), emit_flags)))
    # Collision Type Deforming: the colliders' `deforming`, so the solver rebuilds their shapes every step.
    deforming = b3d_hda.attribute_create(sub, "collision_deforming", [
        ("deforming", "int", [f'ch("../{COLLISION_TYPE}") == 2'])], warn)
    deforming.setInput(0, collision_rest(sub, dopnet, collision_pieces))
    dopnet.setInput(DOP_COLLISION, deforming)
    # Collide As Static Object: the same input unpacked, for the Static Object to read by path.
    unpacked = create(sub, "unpack", "collision_unpack")
    unpacked.setInput(0, scaled[COLLISION])
    create(sub, "null", STATIC_NULL).setInput(0, unpacked)
    dopnet.setInput(DOP_STATIC, sub.node(STATIC_NULL))
    for child in dopnet.children():
        child.destroy()
    solver = build_dops(dopnet)
    scale_forces(dopnet)
    layout(dopnet, DOP_LAYOUT)

    # Simulation points, the fourth output, as RBD Bullet Solver's: one point per piece with P, name,
    # orient (the rotation since rest), pivot (the rest position), restxform (the packed transform at
    # rest), v, w, id and the Transfer Attributes.
    import_points = create(sub, "dopimport", "import_points")
    import_points.parm("doppath").set("../dopnet")
    import_points.parm("objpattern").set("pieces")
    set_menu(import_points, "importstyle", FETCH_GEOMETRY)
    set_toggles(import_points, POINTS_TOGGLES)
    points_only = create(sub, "add", "points_only")
    points_only.setInput(0, import_points)
    if points_only.parm(POINTS_ONLY_PARM) is None or not set_named(points_only, POINTS_ONLY_PARM, 1):
        warn(f"{points_only.path()}: no Delete Geometry But Keep the Points toggle; removing primitives in VEX")
        points_only.destroy()
        points_only = create(sub, "attribwrangle", "points_only")
        points_only.setInput(0, import_points)
        points_only.parm("class").set("primitive")
        points_only.parm("snippet").set(POINTS_ONLY_VEX)
    sim_points = create(sub, "attribdelete", "sim_points")
    sim_points.setInput(0, points_only)
    sim_points.parm("ptdel").set(f'{POINTS_KEEP} `chs("../{TRANSFER_ATTRIBUTES}")` '
                                 f'`ifs(ch("../{REPLICATE}"), "__sourcename __restxform", "")`')
    if not set_named(sim_points, "negate", 1):  # Delete Non Selected
        # Without the inverse toggle the list would delete exactly what must stay: keep everything.
        sim_points.parm("ptdel").set("")
    # Replicate RBDs: the copies made in the simulation renamed <name>_r<n>, and a point per copy for its render piece.
    sim_points, replicas = replicate_points(sub, sim_points)

    # Geometry and proxy follow the simulation points, as RBD Bullet Solver: Transform Pieces by name
    # with no rest input, each point's transform being its motion since rest (pivot, orient, P). Each
    # output keeps its input's packing (names mode).
    pieces_out = []
    for name, src in (("transform_geometry", scaled[GEO]), ("transform_proxy", sim_source)):
        kind = name[len('transform_'):]
        src = rest_source(sub, dopnet, src, kind)
        # Emit RBDs: the emitting render pieces are the ones the simulation stored when each was emitted.
        named = emit_render(sub, prepare_input(sub, src, f"names_{kind}", pack=False), emit_flags, kind)
        named = replicate_render(sub, named, replicas, kind)  # a render piece per copy
        removed = create(sub, "attribwrangle", f"remove_deleted_{name[len('transform_'):]}")
        removed.setInput(0, named)
        removed.setInput(1, sim_points)
        if name == "transform_geometry":
            removed.setInput(2, sim_pieces)
        removed.parm("class").set("primitive")
        removed.parm("snippet").set(REMOVE_DELETED_VEX)
        xform = create(sub, "xformpieces", name)
        xform.setInput(0, removed)
        xform.setInput(1, sim_points)
        copy_attributes(xform, TRANSFER_TO[name][0])
        moved = fast_transform(sub, removed, sim_points, xform) if name == "transform_geometry" else xform
        pieces_out.append(scale_xform(sub, f"unscale_{name[len('transform_'):]}", moved, inverse=True))
    geo_out, proxy_out = pieces_out

    # Constraints as the solver left them: the network is attached to the pieces object
    # (Attach Internal Constraints to Object) as its ConstraintGeometry data, imported as is.
    import_con = create(sub, "dopimport", "import_constraints")
    import_con.parm("doppath").set("../dopnet")
    import_con.parm("objpattern").set("pieces")
    set_menu(import_con, "importstyle", FETCH_GEOMETRY)
    set_named(import_con, "geodatapath", "ConstraintGeometry")
    set_toggles(import_con, CONSTRAINTS_TOGGLES)
    # The solver does not move the network's points: Transform Pieces carries each point with the piece
    # its `name` names, and points anchored to the world (no name) keep their place, as RBD Bullet Solver.
    # Export Broken Constraints off: the broken group's primitives removed first (as RBD Bullet Solver).
    blast_broken = create(sub, "blast", "blast_broken")
    blast_broken.setInput(0, import_con)
    blast_broken.parm("group").set("broken")
    keep_broken = create(sub, "switchif", "switchif_keepbroken")
    keep_broken.setInput(0, blast_broken)
    keep_broken.setInput(1, import_con)
    keep_broken.parm("expr1").setExpression(f'ch("../{KEEP_BROKEN}")', hou.exprLanguage.Hscript)
    con_xform = create(sub, "xformpieces", "transform_constraints")
    con_xform.setInput(0, keep_broken)
    con_xform.setInput(1, sim_points)
    reset_anchors = create(sub, "attribwrangle", "reset_world_anchors")
    reset_anchors.setInput(0, con_xform)
    reset_anchors.setInput(1, keep_broken)
    reset_anchors.parm("group").set('@name=="-1" @name==""')
    reset_anchors.parm("grouptype").set("points")
    reset_anchors.parm("snippet").set(RESET_ANCHORS_VEX)
    con_attribs = scale_wrangle(sub, "unscale_constraint_attribs", reset_anchors, UNSCALE_CONSTRAINTS_VEX,
                                inverse=True, run_over="primitive")
    con_out = create(sub, "switch", "constraints_result")
    con_out.setInput(0, nulls[CONSTRAINTS])
    con_unscaled = scale_xform(sub, "unscale_constraints", con_attribs, inverse=True)
    con_out.setInput(1, con_unscaled)
    switch_on_spare_input(con_out, nulls[CONSTRAINTS])

    # Impacts: the pieces object's Impacts records as points (only when Add Impact Data is on and
    # something pulls IMPACTS, e.g. the Object Merge Fetch Impacts makes).
    import_impacts = create(sub, "dopimportrecords", "import_impacts")
    set_named(import_impacts, "doppath", "../dopnet")
    set_named(import_impacts, "objpattern", "pieces")
    set_named(import_impacts, "datapath", "Impacts")
    set_named(import_impacts, "record", "Impacts")
    impacts_unscaled = scale_wrangle(sub, "unscale_impacts", import_impacts, UNSCALE_IMPACTS_VEX, inverse=True)
    create(sub, "null", IMPACTS).setInput(0, impacts_unscaled)

    # Collision shapes (the Guide Geometry, not an output): the shapes the solver exports as each object's Box3DShapes data
    # while Show Collision Shapes is on, world space like the simulation, so unscaled. Imported only
    # while the toggle is on (the Switch does not cook the other input). Nothing before the Start Frame.
    import_shapes = create(sub, "dopimport", "import_shapes")
    import_shapes.parm("doppath").set("../dopnet")
    import_shapes.parm("objpattern").set("pieces")
    set_menu(import_shapes, "importstyle", FETCH_GEOMETRY)
    set_named(import_shapes, "geodatapath", SHAPES_DATA)
    set_toggles(import_shapes, POINTS_TOGGLES)
    tint = create(sub, "attribwrangle", "tint_shapes")
    tint.setInput(0, import_shapes)
    tint.parm("class").set("primitive")
    tint.parm("snippet").set(SHAPES_TINT_VEX)
    unscaled_shapes = scale_xform(sub, "unscale_shapes", tint, inverse=True)
    no_shapes = create(sub, "null", "no_shapes")  # nothing wired: empty geometry
    show_shapes = create(sub, "switch", "switch_show_shapes")
    show_shapes.setInput(0, no_shapes)
    show_shapes.setInput(1, unscaled_shapes)
    show_shapes.parm("input").setExpression(f'ch("../{VISUALIZE}") && ch("../{SHAPES_TOGGLE}")', hou.exprLanguage.Hscript)
    shapes_out = before_start(sub, dopnet, "switchif_beforestart_shapes", show_shapes, no_shapes)

    # The collider's shapes, apart (RBD Bullet Solver's Collision Geometry > Show Collision Shape and Color): the same
    # import for the collision object, tinted by the collision Color, behind Enable Collision Geometry.
    import_cshapes = create(sub, "dopimport", "import_collision_shapes")
    import_cshapes.parm("doppath").set("../dopnet")
    import_cshapes.parm("objpattern").set("collision")
    set_menu(import_cshapes, "importstyle", FETCH_GEOMETRY)
    set_named(import_cshapes, "geodatapath", SHAPES_DATA)
    set_toggles(import_cshapes, POINTS_TOGGLES)
    ctint = create(sub, "attribwrangle", "tint_collision_shapes")
    ctint.setInput(0, import_cshapes)
    ctint.parm("class").set("primitive")
    ctint.parm("snippet").set(COLLISION_SHAPES_TINT_VEX)
    show_cshapes = create(sub, "switch", "switch_show_collision_shapes")
    show_cshapes.setInput(0, no_shapes)
    show_cshapes.setInput(1, scale_xform(sub, "unscale_collision_shapes", ctint, inverse=True))
    show_cshapes.parm("input").setExpression(f'ch("../{VISUALIZE}") && ch("../{USE_COLLISIONS}") && ch("../{SHOW_COLLISION_SHAPES}")',
                                             hou.exprLanguage.Hscript)
    cshapes_out = before_start(sub, dopnet, "switchif_beforestart_collision_shapes", show_cshapes, no_shapes)

    # The constraint lines (RBD Bullet Solver's Show Constraints) with False Color Mode, and the impact points (Show
    # Impacts), each cooked only while its toggle is on.
    fc_value = create(sub, "attribwrangle", "false_color_value")
    fc_value.setInput(0, con_unscaled)
    fc_value.parm("class").set("primitive")
    fc_value.parm("snippet").set(FALSE_COLOR_VALUE_VEX)
    fc_value.parm("bindings").set(1)
    fc_value.parm("bindparm1").set("fcvalue")
    # Only the copy is made: a bound attribute that is missing is not created.
    fc_value.parm("exportlist").set("__fc")
    fc_top = create(sub, "attribpromote", "false_color_top")
    fc_top.setInput(0, fc_value)
    set_menu(fc_top, "inclass", "primitive")
    set_menu(fc_top, "outclass", "detail")
    set_menu(fc_top, "method", "max")
    set_named(fc_top, "inname", "__fc")
    set_named(fc_top, "useoutname", 1)
    set_named(fc_top, "outname", "__fc_top")
    set_named(fc_top, "deletein", 0)
    fc_color = create(sub, "attribwrangle", "false_color_constraints")
    fc_color.setInput(0, fc_top)
    fc_color.parm("class").set("primitive")
    fc_color.parm("snippet").set(FALSE_COLOR_VEX)
    fc_drop = create(sub, "attribdelete", "false_color_drop")
    fc_drop.setInput(0, fc_color)
    set_named(fc_drop, "primdel", "__fc")
    set_named(fc_drop, "dtldel", "__fc_top")
    # Off (False Color Mode None) the lines keep the colour they came with.
    false_color = create(sub, "switch", "switch_false_color")
    false_color.setInput(0, con_unscaled)
    false_color.setInput(1, fc_drop)
    false_color.parm("input").setExpression(f'ch("../{FALSE_COLOR_MODE}") > 0', hou.exprLanguage.Hscript)
    # The lines are plain unless Show Guides is on: the radius per point and the wire (a tube round every line) are
    # made only then, as they cost far more than the lines (seconds a frame on tens of thousands of constraints).
    guide_radius = create(sub, "attribwrangle", "constraint_guide_radius")
    guide_radius.setInput(0, false_color)
    guide_radius.parm("class").set("point")
    guide_radius.parm("snippet").set(guide_radius_vex())
    guide_wire = create(sub, "polywire", "constraint_guide_wire")  # thick lines, by each line's radius
    guide_wire.setInput(0, guide_radius)
    guide_wire.parm("radius").set(1)
    guide_wire.parm("usescaleattrib").set(1)
    guide_wire.parm("scaleattrib").set("guide_radius")
    thick = create(sub, "switch", "switch_thick_constraints")
    thick.setInput(0, false_color)
    thick.setInput(1, guide_wire)
    thick.parm("input").setExpression(f'ch("../{SHOW_GUIDES}")', hou.exprLanguage.Hscript)
    show_con = create(sub, "switch", "switch_show_constraints")
    show_con.setInput(0, no_shapes)
    show_con.setInput(1, thick)
    show_con.parm("input").setExpression(f'ch("../{VISUALIZE}") && ch("../{SHOW_CONSTRAINTS}")', hou.exprLanguage.Hscript)
    con_guide_out = before_start(sub, dopnet, "switchif_beforestart_constraint_guide", show_con, no_shapes)
    # The impact points in the guide: red.
    impact_color = create(sub, "color", "color_impacts")
    impact_color.setInput(0, impacts_unscaled)
    set_menu(impact_color, "class", "point")
    set_named(impact_color, "color", (1.0, 0.2, 0.0))
    show_impacts = create(sub, "switch", "switch_show_impacts")
    show_impacts.setInput(0, no_shapes)
    show_impacts.setInput(1, impact_color)
    show_impacts.parm("input").setExpression(f'ch("../{VISUALIZE}") && ch("../{SHOW_IMPACTS}")', hou.exprLanguage.Hscript)
    impacts_guide_out = before_start(sub, dopnet, "switchif_beforestart_impact_guide", show_impacts, no_shapes)

    # The external collision geometry, as RBD Bullet Solver's guide shows it (dopimport_collision, dimmed
    # by half, behind Enable Collision Geometry and Show Collision Geometry; on by default).
    import_cgeo = create(sub, "dopimport", "import_collision_geo")
    import_cgeo.parm("doppath").set("../dopnet")
    import_cgeo.parm("objpattern").set("collision")
    set_menu(import_cgeo, "importstyle", FETCH_GEOMETRY)
    set_named(import_cgeo, "donotsim", 1)
    dim = create(sub, "attribwrangle", "dim_collision_geo")
    dim.setInput(0, import_cgeo)
    dim.parm("class").set("primitive")
    dim.parm("snippet").set(COLLISION_GUIDE_VEX)
    show_cgeo = create(sub, "switch", "switch_show_collision_geo")
    show_cgeo.setInput(0, no_shapes)
    show_cgeo.setInput(1, scale_xform(sub, "unscale_collision_geo", dim, inverse=True))
    show_cgeo.parm("input").setExpression(f'ch("../{VISUALIZE}") && ch("../{USE_COLLISIONS}") && ch("../{SHOW_COLLISION}")',
                                          hou.exprLanguage.Hscript)
    cgeo_out = before_start(sub, dopnet, "switchif_beforestart_collision_geo", show_cgeo, no_shapes)

    # GUIDE_GEO, drawn in the viewport (the asset's Guide Geometry): both in wireframe, as Bullet's.
    guide = create(sub, "merge", "merge_guide")
    guide.setInput(0, cgeo_out)
    guide.setInput(1, shapes_out)
    guide.setInput(2, build_ground_guide(sub, dopnet, no_shapes))
    guide.setInput(3, cshapes_out)
    guide.setInput(4, con_guide_out)
    guide.setInput(5, impacts_guide_out)
    wire = create(sub, ["attribcreate::2.0", "attribcreate"], "guide_wireframe")
    wire.setInput(0, guide)
    wire.parm("name1").set("gl_wireframe")
    wire.parm("class1").set(0)  # detail
    wire.parm("type1").set(1)  # int
    wire.parm("value1v1").set(-1)
    create(sub, "null", GUIDE_GEO).setInput(0, wire)

    # Before the Start Frame the inputs pass through, as RBD Bullet Solver (the proxy output shows the simulated
    # proxy and the Proxy input before it).
    geo_out = before_start(sub, dopnet, "switchif_beforestart", geo_out, nulls[GEO])
    con_out = before_start(sub, dopnet, "switchif_beforestart_constraints", con_out, nulls[CONSTRAINTS])
    proxy_out = before_start(sub, dopnet, "switchif_beforestart_proxy", proxy_out, nulls[PROXY])
    # With no Proxy input the Proxy output is empty, as RBD Bullet Solver's (the geometry is simulated then, but is
    # not repeated there).
    no_proxy = create(sub, "null", "no_proxy")  # nothing wired: empty geometry
    proxy_shown = create(sub, "switch", "switch_proxy_connected")
    proxy_shown.setInput(0, no_proxy)
    proxy_shown.setInput(1, proxy_out)
    switch_on_spare_input(proxy_shown, nulls[PROXY])
    proxy_out = proxy_shown
    points_out = scale_wrangle(sub, "unscale_points", sim_points, UNSCALE_POINTS_VEX, inverse=True)
    for idx, src in enumerate([geo_out, con_out, proxy_out, points_out]):
        out = create(sub, "output", f"OUT_{['geometry', 'constraints', 'proxy', 'points'][idx]}")
        out.setInput(0, src)
        out.parm("outputidx").set(idx)
        if idx in INPUT_COLORS:
            out.setColor(hou.Color(INPUT_COLORS[idx]))
        if idx == 0:
            out.setDisplayFlag(True)
            out.setRenderFlag(True)
    layout(sub, SOLVER_LAYOUT)
    return dopnet, solver, dopnet.node("pieces")
