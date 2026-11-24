"""The scenes bench.py times, in Box3D and (where there is an equivalent) in RBD Bullet Solver.

Each scenario is a function `build(kind, p) -> Scene` that builds one scene in a fresh hip file, `kind` being
"box3d" or "bullet" and `p` its parameters (the scenario's DEFAULTS, overridden from the command line). The same
DOP graph is built for both solvers wherever the node types exist in both, so the numbers compare.

A scene reports its frame count and phases (frame ranges timed apart, so a settled pile is not averaged with
its fall). `SCENARIOS` is the registry bench.py reads: name -> Scenario."""

import math

from b3dtest import (
    box3d_data,
    box3d_solver,
    bullet_solver,
    create,
    display,
    finish,
    GROUND,
    merge,
    multisolver,
    new_dopnet,
    new_geo,
    packed_boxes,
    packed_object,
    parm_value,
    set_expression,
    set_menu_token,
    set_parm,
    SetupError,
    sop_solver,
)


class Scene:
    def __init__(self, dop, frames, phases, sop=None):
        self.dop = dop
        self.frames = frames
        self.phases = phases  # [(label, first frame, last frame)]
        # A SOP to cook each frame instead of the DOP network (the Box3D Solver SOP, whose inner DOP network is `dop`)
        self.sop = sop


class Scenario:
    def __init__(self, build, doc, defaults, sweep=(), kinds=("box3d", "bullet"), kinds_for=None):
        self.build = build
        self.doc = doc
        self.defaults = defaults
        self.sweep = list(sweep) or [{}]  # parameter overrides run when the scenario is named without any
        self.kinds = kinds
        self.kinds_for = kinds_for  # p -> kinds, when the parameters decide


SCENARIOS = {}


def scenario(name, doc, defaults, sweep=(), kinds=("box3d", "bullet"), kinds_for=None):
    def register(fn):
        SCENARIOS[name] = Scenario(fn, doc, defaults, sweep, kinds, kinds_for)
        return fn

    return register


# ---------------------------------------------------------------------------
# Shared pieces
# ---------------------------------------------------------------------------


def copy_packed(geo, shape, points):
    """Copy to Points packing `shape` onto `points`, carrying the points' body attributes (name, active, ...) onto the
    packed copies: Copy to Points takes none unless told to."""
    copy = create(geo, "copytopoints")
    copy.setInput(0, shape)
    copy.setInput(1, points)
    set_parm(copy, ["pack", "packgeo"], 1)
    set_parm(copy, ["targetattribs"], 1)
    set_parm(copy, ["applyattribs1"], "name active animated v w orient pscale life age id")
    display(copy)
    return copy


def solver_opts(kind, dop, solver, p):
    """The solver parameters a scenario sweeps (Box3D only, except substeps where Bullet has the parameter)."""
    if kind == "box3d":
        for key, names in (("substeps", ["substeps"]), ("passes", ["solverpasses"]), ("sleep", ["enablesleep"]),
                           ("exportshapes", ["exportshapes"]), ("compound", ["gluecompound"]),
                           ("readouts", ["bondreadouts"]), ("impacts_guide", ["showshapes"])):
            if p.get(key) is not None:
                set_parm(solver, names, p[key])
    else:
        bullet = next(n for n in dop.children() if n.type().name() == "bulletrbdsolver")
        if p.get("substeps") is not None and bullet.parm("substeps") is not None:
            bullet.parm("substeps").set(p["substeps"])


def make_solver(kind, dop, upstream, p):
    """The solver wired to `upstream`: (the last node to feed the Output, the solver node)."""
    if kind == "box3d":
        gravity = (0, 0, 0) if p.get("forcedop") else None
        solver = box3d_solver(dop, upstream, gravity=gravity)
        last = solver
        if p.get("forcedop"):  # gravity as a Force DOP, as Bullet has it
            last = create(dop, "gravity", "gravity_force")
            last.setInput(0, solver)
    else:
        last = bullet_solver(dop, upstream)
        solver = next(n for n in dop.children() if n.type().name() == "bulletrbdsolver")
    solver_opts(kind, dop, solver, p)
    return last, solver


def lattice_geo(tag, nx, ny, nz, shape="box", size=0.8, extra=""):
    """A SOP object of packed shapes on a jittered nx x ny x nz lattice, named p0, p1, ...: (the packed SOP's path,
    count). shape: box, or a polygon sphere of rows x cols given as "sphere:8x12" (a convex hull of about that many
    points). extra: VEX run for each point (`pt` is its number, `n` its body number)."""
    geo = new_geo(tag)
    pts = create(geo, "attribwrangle", "lattice")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        'addpointattrib(0, "name", ""); int n = 0;\n'
        f"for (int k = 0; k < {ny}; k++) for (int j = 0; j < {nz}; j++) for (int i = 0; i < {nx}; i++) {{\n"
        f"  vector p = set(i - {nx} * 0.5 + 0.5, 0.6 + k * 1.05, j - {nz} * 0.5 + 0.5) "
        "+ 0.05 * (rand(set(i, j, k)) - 0.5);\n"
        '  int pt = addpoint(0, p); setpointattrib(0, "name", pt, sprintf("p%d", n));\n'
        f"  {extra}\n  n++;\n}}"))
    if shape.startswith("sphere"):
        rows, cols = (int(v) for v in (shape.split(":")[1] if ":" in shape else "8x12").split("x"))
        src = create(geo, "sphere")
        set_menu_token(src, ["type"], "polymesh")
        set_parm(src, ["rad"], (size / 2,) * 3)
        set_parm(src, ["rows"], rows)
        set_parm(src, ["cols"], cols)
    else:
        src = create(geo, "box")
        set_parm(src, ["size"], (size,) * 3)
    copy = copy_packed(geo, src, pts)
    return copy.path(), nx * ny * nz


def pile_dims(n, ny=12):
    side = max(1, math.ceil(math.sqrt(n / ny)))
    return side, ny, side


def ground_for(tag, nx, nz, **attrs):
    """A static ground box wider than the lattice: its packed SOP's path."""
    width = max(40.0, 2.0 * max(nx, nz) + 6.0)
    path, _ = packed_boxes(tag, [dict(GROUND, size=(width, 1, width), attrs=dict(GROUND["attrs"], **attrs))])
    return path


def phases_for(frames):
    """A lattice pile is still creeping at frame 121 and mostly asleep by 240: `asleep` is the frames after that."""
    if frames > 260:
        return [("fall", 2, 48), ("pile", 49, 120), ("settling", 121, 240), ("asleep", 241, frames)]
    return [("fall", 2, 48), ("pile", 49, 120), ("settled", 121, frames)]


# ---------------------------------------------------------------------------
# Rigid bodies
# ---------------------------------------------------------------------------


@scenario("pile", "N boxes (or sphere hulls) dropped in a lattice onto a ground box: the fall, the pile, then the "
          "settled scene (the cost of a scene where nothing moves). Options: shape box|sphere:RxC, substeps, passes, "
          "sleep, forcedop, impacts, exportshapes.",
          {"n": 5000, "frames": 240, "shape": "box"},
          sweep=[{"n": 1000}, {"n": 5000}, {"n": 20000, "frames": 150}])
def scene_pile(kind, p):
    n = int(p["n"])
    nx, ny, nz = pile_dims(n)
    dop = new_dopnet("pile")
    path, _ = lattice_geo("pile_pieces", nx, ny, nz, p["shape"], 0.8 if p["shape"] == "box" else 0.9)
    pieces = packed_object(dop, "pieces", path)
    if p.get("impacts") and kind == "box3d":
        pieces = box3d_data(dop, pieces, "data", addimpact=1)
    ground = packed_object(dop, "ground", ground_for("pile_ground", nx, nz))
    last, _ = make_solver(kind, dop, merge(dop, [pieces, ground]), p)
    finish(dop, last)
    return Scene(dop, int(p["frames"]), phases_for(int(p["frames"])))


EDITS = {"none": None, "touch_v": "v@v = v@v;", "damp_v": "v@v *= 0.999;", "touch_P": "@P = @P;"}


@scenario("edit", "A pile with a SOP Solver before the physics solver that writes every point's v each step: touch_v "
          "writes the same value (does the skip survive?), damp_v changes it, none is the control (same Multiple "
          "Solver, no edit).",
          {"n": 5000, "frames": 240, "edit": "none"},
          sweep=[{"edit": "none"}, {"edit": "touch_v"}, {"edit": "damp_v"}])
def scene_edit(kind, p):
    n = int(p["n"])
    nx, ny, nz = pile_dims(n)
    dop = new_dopnet("edit")
    path, _ = lattice_geo("edit_pieces", nx, ny, nz)
    objs = merge(dop, [packed_object(dop, "pieces", path),
                       packed_object(dop, "ground", ground_for("edit_ground", nx, nz))])
    code = EDITS[p["edit"]]
    movers = [sop_solver(dop, code)] if code else []
    if kind == "box3d":
        physics = box3d_solver(dop, None)
        last = multisolver(dop, objs, movers + [physics])
    else:
        physics = create(dop, "bulletrbdsolver", "bullet_solver")
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, objs, movers + [physics]))
    finish(dop, last)
    return Scene(dop, int(p["frames"]), phases_for(int(p["frames"])))


@scenario("collider", "A pile dropped onto a big static triangle mesh (bullet_georep concave, R x R grid with "
          "noise), or onto a kinematic box swept back and forth (animated) with the pile around it.",
          {"n": 2000, "frames": 120, "type": "mesh", "res": 200},
          sweep=[{"type": "mesh", "res": 100}, {"type": "mesh", "res": 300}, {"type": "animated"}])
def scene_collider(kind, p):
    n = int(p["n"])
    nx, ny, nz = pile_dims(n, 6)
    dop = new_dopnet("collider")
    path, _ = lattice_geo("collider_pieces", nx, ny, nz)
    objs = [packed_object(dop, "pieces", path)]
    if p["type"] == "mesh":
        res = int(p["res"])
        geo = new_geo("collider_mesh")
        grid = create(geo, "grid")
        size = max(20.0, 2.0 * max(nx, nz))
        set_parm(grid, ["size"], (size, size))
        set_parm(grid, ["rows"], res)
        set_parm(grid, ["cols"], res)
        noise = create(geo, "attribwrangle", "noise")
        noise.setInput(0, grid)
        set_parm(noise, ["class"], 0)
        set_parm(noise, ["snippet"], "@P.y = 0.15 * noise(@P * 0.7);")
        pack = create(geo, "pack")
        pack.setInput(0, noise)
        attrs = create(geo, "attribwrangle", "attrs")
        attrs.setInput(0, pack)
        set_parm(attrs, ["snippet"], 's@name = "mesh"; i@active = 0; i@animated = 0; s@bullet_georep = "concave"; '
                                     'v@v = {0, 0, 0}; v@w = {0, 0, 0};')
        display(attrs)
        objs.append(packed_object(dop, "mesh", attrs.path()))
        merged = merge(dop, objs)
        last, _ = make_solver(kind, dop, merged, p)
    else:
        objs.append(packed_object(dop, "ground", ground_for("collider_ground", nx, nz)))
        mover_path, _ = packed_boxes("collider_mover", [
            {"size": (4, 1, 4), "center": (0, 1.0, 0), "attrs": {"active": 0, "animated": 1}}])
        objs.append(packed_object(dop, "mover", mover_path))
        merged = merge(dop, objs)
        mover = sop_solver(dop, "@P.x = sin(@Time * 2.0) * 3.0;")
        try:
            set_parm(mover, ["group", "objects"], "mover")
        except SetupError:
            pass
        if kind == "box3d":
            physics = box3d_solver(dop, None)
            last = multisolver(dop, merged, [mover, physics])
        else:
            physics = create(dop, "bulletrbdsolver", "bullet_solver")
            last = create(dop, "gravity", "gravity")
            last.setInput(0, multisolver(dop, merged, [mover, physics]))
    finish(dop, last)
    return Scene(dop, int(p["frames"]), [("fall", 2, 48), ("after", 49, int(p["frames"]))])


@scenario("animated", "N kinematic boxes (animated 1) moved by a SOP Solver every step, with 200 dynamic boxes "
          "falling onto them: the cost of following animated pieces.",
          {"n": 1000, "frames": 96},
          sweep=[{"n": 200}, {"n": 1000}, {"n": 5000}])
def scene_animated(kind, p):
    n = int(p["n"])
    side = max(1, math.ceil(math.sqrt(n)))
    geo = new_geo("animated_geo")
    pts = create(geo, "attribwrangle", "points")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        'addpointattrib(0, "name", ""); addpointattrib(0, "active", 0); addpointattrib(0, "animated", 1);\n'
        f"for (int i = 0; i < {n}; i++) {{ int pt = addpoint(0, set((i % {side}) * 1.2 - {side} * 0.6, 1.0, "
        f"(i / {side}) * 1.2 - {side} * 0.6)); setpointattrib(0, \"name\", pt, sprintf(\"a%d\", i)); }}"))
    box = create(geo, "box")
    set_parm(box, ["size"], (1.0, 0.5, 1.0))
    copy = copy_packed(geo, box, pts)
    dop = new_dopnet("animated")
    drops_path, _ = lattice_geo("animated_drops", 14, 15, 1, size=0.5)
    objs = merge(dop, [packed_object(dop, "movers", copy.path()),
                       packed_object(dop, "drops", drops_path)])
    mover = sop_solver(dop, "if (i@animated == 1) @P.y = 1.0 + 0.3 * sin(@Time * 3.0 + @ptnum * 0.1);")
    try:
        set_parm(mover, ["group", "objects"], "movers")
    except SetupError:
        pass
    if kind == "box3d":
        physics = box3d_solver(dop, None)
        last = multisolver(dop, objs, [mover, physics])
    else:
        physics = create(dop, "bulletrbdsolver", "bullet_solver")
        last = create(dop, "gravity", "gravity")
        last.setInput(0, multisolver(dop, objs, [mover, physics]))
    finish(dop, last)
    return Scene(dop, int(p["frames"]), [("fall", 2, 48), ("after", 49, int(p["frames"]))])


# ---------------------------------------------------------------------------
# Constraints
# ---------------------------------------------------------------------------

# type -> (constraint node, its parameters, primitive attributes on every line, solvers that have it)
JOINT_TYPES = {
    "glue": ("glueconrel", {"strength": 1e12}, {}, ("box3d", "bullet")),
    "hard_pos": ("hardconrel", {}, {"constraint_type": "position"}, ("box3d", "bullet")),
    "hard_all": ("hardconrel", {}, {"constraint_type": "all"}, ("box3d", "bullet")),
    "spring": ("springconrel", {}, {}, ("box3d", "bullet")),
    "soft": ("bulletsoftconrel", {}, {}, ("box3d", "bullet")),
    "conetwist": ("conetwistconrel", {}, {}, ("box3d", "bullet")),
    "weld": ("box3d_weldconrel", {}, {}, ("box3d",)),
    "weld_breakable": ("box3d_weldconrel", {"breakable": 1, "force_threshold": 1e9, "torque_threshold": 1e9}, {},
                       ("box3d",)),
    "spherical": ("box3d_sphericalconrel", {}, {}, ("box3d",)),
    "revolute": ("box3d_revoluteconrel", {}, {}, ("box3d",)),
    "prismatic": ("box3d_prismaticconrel", {}, {}, ("box3d",)),
    "distance": ("box3d_distanceconrel", {}, {}, ("box3d",)),
}


def layout_vex(layout, n, length=20):
    """(the points' VEX, the links' VEX) for `layout`: chains (`length` hanging links each), grid (a pinned-corner sheet
    with a joint to every neighbour) or pairs (one fixed box and one hanging from it). The links' VEX calls
    link(i, j, position) once per joint."""
    if layout == "chains":
        count = max(1, n // length)
        bodies = (f"for (int c = 0; c < {count}; c++) for (int l = 0; l < {length}; l++) "
                  "{ int pt = addpoint(0, set(c * 1.0, 40.0 - l * 0.5, 0)); "
                  f'setpointattrib(0, "name", pt, sprintf("p%d", c * {length} + l)); '
                  'setpointattrib(0, "active", pt, l == 0 ? 0 : 1); }')
        links = (f"for (int c = 0; c < {count}; c++) for (int l = 0; l < {length - 1}; l++) "
                 f"link(c * {length} + l, c * {length} + l + 1, set(c * 1.0, 40.0 - l * 0.5 - 0.25, 0));")
        return bodies, links
    if layout == "grid":
        side = max(2, round(math.sqrt(n)))
        bodies = (f"for (int a = 0; a < {side}; a++) for (int b = 0; b < {side}; b++) "
                  "{ int pt = addpoint(0, set(a * 0.5, 6.0, b * 0.5)); "
                  f'setpointattrib(0, "name", pt, sprintf("p%d", a * {side} + b)); '
                  f"setpointattrib(0, \"active\", pt, ((a == 0 || a == {side - 1}) && (b == 0 || b == {side - 1})) "
                  "? 0 : 1); }")
        links = (f"for (int a = 0; a < {side}; a++) for (int b = 0; b < {side}; b++) {{\n"
                 f"  if (a + 1 < {side}) link(a * {side} + b, (a + 1) * {side} + b, set(a * 0.5 + 0.25, 6.0, b * 0.5));\n"
                 f"  if (b + 1 < {side}) link(a * {side} + b, a * {side} + b + 1, set(a * 0.5, 6.0, b * 0.5 + 0.25));\n}}")
        return bodies, links
    if layout == "pairs":
        pairs = max(1, n // 2)
        side = max(1, math.ceil(math.sqrt(pairs)))
        bodies = (f"for (int k = 0; k < {pairs}; k++) {{ vector o = set((k % {side}) * 1.5, 6.0, (k / {side}) * 1.5); "
                  "int a = addpoint(0, o); int b = addpoint(0, o - {0, 0.5, 0});"
                  'setpointattrib(0, "name", a, sprintf("p%d", 2 * k)); setpointattrib(0, "active", a, 0);'
                  'setpointattrib(0, "name", b, sprintf("p%d", 2 * k + 1)); }')
        links = (f"for (int k = 0; k < {pairs}; k++) link(2 * k, 2 * k + 1, "
                 f"set((k % {side}) * 1.5, 5.75, (k / {side}) * 1.5));")
        return bodies, links
    raise SetupError(f"unknown joint layout {layout}")


def joint_scene_geo(tag, layout, n, line_attrs, constraint_name, dynamic, length=20):
    """(the bodies' packed SOP path, the constraint network's SOP path, the body count)."""
    bodies_vex, links_vex = layout_vex(layout, n, length)
    geo = new_geo(tag + "_bodies")
    pts = create(geo, "attribwrangle", "points")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], 'addpointattrib(0, "name", ""); addpointattrib(0, "active", 1);\n' + bodies_vex)
    box = create(geo, "box")
    set_parm(box, ["size"], (0.4, 0.4, 0.4))
    copy = copy_packed(geo, box, pts)
    net_geo = new_geo(tag + "_network")
    net = create(net_geo, "attribwrangle", "network")
    set_parm(net, ["class"], 0)
    extra_decl = "".join(f'addprimattrib(0, "{k}", "");' for k in line_attrs)
    extra_set = "".join(f'setprimattrib(0, "{k}", pr, "{v}");' for k, v in line_attrs.items())
    set_parm(net, ["snippet"], (
        'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", "");' + extra_decl + "\n"
        "void link(int i; int j; vector pos) {\n"
        "  int p0 = addpoint(0, pos); int p1 = addpoint(0, pos);\n"
        '  setpointattrib(0, "name", p0, sprintf("p%d", i)); setpointattrib(0, "name", p1, sprintf("p%d", j));\n'
        f'  int pr = addprim(0, "polyline", p0, p1); setprimattrib(0, "constraint_name", pr, "{constraint_name}");'
        f"{extra_set}\n}}\n" + links_vex))
    out = net
    if dynamic:  # the geometry changes every frame, so the solver has to read the whole network again
        stamp = create(net_geo, "attribwrangle", "stamp")
        stamp.setInput(0, net)
        set_parm(stamp, ["class"], 0)
        set_parm(stamp, ["snippet"], "f@stamp = @Frame;")
        out = stamp
    display(out)
    return copy.path(), out.path()


def joint_kinds(p):
    return JOINT_TYPES[p["type"]][3]


@scenario("joints", "One joint type on a layout: chains (`length` hanging links each), grid (a sheet pinned at its corners, "
          "a joint to every neighbour) or pairs (independent fixed + hanging boxes). Options: type (glue, hard_pos, "
          "hard_all, spring, soft, conetwist run in both solvers; weld, weld_breakable, spherical, revolute, "
          "prismatic, distance in Box3D only), layout, n (bodies), length (chain links), restlength (SideFX nodes; 0 holds the anchors where they are), dynamic 1 (network rewritten every frame).",
          {"type": "hard_all", "layout": "chains", "n": 5000, "frames": 200, "dynamic": 0, "length": 20, "restlength": 0.0},
          sweep=[{"layout": "chains"}, {"layout": "grid", "n": 2500}, {"layout": "pairs", "n": 10000}],
          kinds_for=joint_kinds)
def scene_joints(kind, p):
    node_type, parms, line_attrs, _ = JOINT_TYPES[p["type"]]
    dop = new_dopnet("joints")
    conrel = create(dop, node_type, "joint0")
    for name, value in parms.items():
        set_parm(conrel, [name], value)
    # SideFX's Rest Length defaults to 1: anchors that start closer are pulled apart to it, which tears a chain
    # with 0.5 links apart in both solvers. 0 holds the anchors where they start.
    if conrel.parm("restlength") is not None:
        conrel.parm("restlength").set(float(p["restlength"]))
    cname = parm_value(conrel, ["dataname", "data_name", "DataName"], "joint0")
    bodies, network = joint_scene_geo("joints", p["layout"], int(p["n"]), line_attrs, cname, int(p["dynamic"]),
                                       int(p["length"]))
    obj = packed_object(dop, "pieces", bodies)
    net = create(dop, "constraintnetwork", "constraint_network")
    net.setInput(0, obj)
    set_parm(net, ["soppath", "constraintsoppath", "geosoppath"], network)
    net.setInput(1, conrel)
    last, _ = make_solver(kind, dop, net, p)
    finish(dop, last)
    frames = int(p["frames"])
    return Scene(dop, frames, [("moving", 2, 24), ("settling", 25, 100), ("settled", 101, frames)])


# ---------------------------------------------------------------------------
# Glue, compounds, fracture
# ---------------------------------------------------------------------------


@scenario("wall", "A brick wall (W x H bricks) glued to each neighbour, a heavy box thrown at it: at rest, the hit, the "
          "debris. Options: w, h, strength (Glue; the default holds the wall intact, a low one makes it fall apart by itself), group (bricks per glued block; 0 = one block), compound 0|1 (Box3D's "
          "Glue as One Body).",
          {"w": 40, "h": 20, "strength": 1e8, "group": 0, "frames": 120},
          sweep=[{"compound": 1}, {"compound": 0}, {"w": 20, "h": 10, "compound": 1},
                 {"group": 5, "compound": 1}])
def scene_wall(kind, p):
    return build_wall(kind, p, "glue")


def build_wall(kind, p, joint):
    """The brick wall of `wall` (joint "glue": SideFX Glue) and `bondwall` (joint "bond": Box3D Bond)."""
    w, h, group = int(p["w"]), int(p["h"]), int(p["group"])
    dop = new_dopnet("wall")
    geo = new_geo("wall_bricks")
    pts = create(geo, "attribwrangle", "points")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        'addpointattrib(0, "name", ""); addpointattrib(0, "active", 1);\n'
        f"for (int a = 0; a < {w}; a++) for (int b = 0; b < {h}; b++) {{ "
        f"int pt = addpoint(0, set(a * 1.0 - {w} * 0.5, 0.25 + b * 0.5, 0)); "
        f'setpointattrib(0, "name", pt, sprintf("p%d", a * {h} + b)); }}'))
    box = create(geo, "box")
    set_parm(box, ["size"], (1.0, 0.5, 0.5))
    copy = copy_packed(geo, box, pts)
    if joint == "bond":
        glue = create(dop, "box3d_bondconrel", "bond")
        strength = float(p["strength"])
        for parm, value in (("bond_tensile", strength), ("bond_shear", strength), ("bond_bending", strength),
                            ("bond_compressive", strength * 10)):
            set_parm(glue, [parm], value)
    else:
        glue = create(dop, "glueconrel", "glue")
        set_parm(glue, ["strength"], float(p["strength"]))
    net_geo = new_geo("wall_network")
    net = create(net_geo, "attribwrangle", "network")
    set_parm(net, ["class"], 0)
    skip_h = f"&& (a + 1) % {group} != 0" if group else ""
    skip_v = f"&& (b + 1) % {group} != 0" if group else ""
    cname = parm_value(glue, ["dataname", "data_name", "DataName"], "Bond" if joint == "bond" else "Glue")
    set_parm(net, ["snippet"], (
        'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", "");\n'
        "void link(int i; int j; vector pos) {\n"
        "  int p0 = addpoint(0, pos); int p1 = addpoint(0, pos);\n"
        '  setpointattrib(0, "name", p0, sprintf("p%d", i)); setpointattrib(0, "name", p1, sprintf("p%d", j));\n'
        f'  int pr = addprim(0, "polyline", p0, p1); setprimattrib(0, "constraint_name", pr, "{cname}");\n}}\n'
        f"for (int a = 0; a < {w}; a++) for (int b = 0; b < {h}; b++) {{\n"
        f"  if (a + 1 < {w} {skip_h}) link(a * {h} + b, (a + 1) * {h} + b, set(a * 1.0 - {w} * 0.5 + 0.5, 0.25 + b * 0.5, 0));\n"
        f"  if (b + 1 < {h} {skip_v}) link(a * {h} + b, a * {h} + b + 1, set(a * 1.0 - {w} * 0.5, 0.5 + b * 0.5, 0));\n}}"))
    display(net)
    bricks = packed_object(dop, "bricks", copy.path())
    # `wait` frames of the wall standing before the ball arrives (30 m/s is 1.25 m a frame)
    wait = int(p.get("wait", 0))
    ball_path, _ = packed_boxes("wall_ball", [
        {"size": (2, 2, 2), "center": (0, h * 0.25, 25 + 1.25 * wait),
         "attrs": {"active": 1, "v": (0.0, 0.0, -30.0)}}])
    ball = packed_object(dop, "ball", ball_path, keep_velocity=True)
    ground = packed_object(dop, "ground", ground_for("wall_ground", w, w))
    cnet = create(dop, "constraintnetwork", "constraint_network")
    cnet.setInput(0, merge(dop, [bricks, ball, ground]))
    set_parm(cnet, ["soppath", "constraintsoppath", "geosoppath"], net.path())
    cnet.setInput(1, glue)
    last, _ = make_solver(kind, dop, cnet, p)
    finish(dop, last)
    frames = int(p["frames"])
    if wait:
        return Scene(dop, frames, [("rest", 2, 15), ("stand", 16, 15 + wait), ("hit", 16 + wait, 40 + wait),
                                   ("debris", 41 + wait, frames)])
    return Scene(dop, frames, [("rest", 2, 15), ("hit", 16, 40), ("debris", 41, frames)])


@scenario("fracture", "A cube fractured by Voronoi Fracture (Points from Volume, Particle Separation `sep`), every piece glued to "
          "its neighbours by the fracture's own constraints, dropped on the ground: the pieces and constraints go "
          "through the Solver SOP (Box3D Solver or RBD Bullet Solver). Options: sep (smaller = more pieces), strength "
          "(Glue; the default holds the cube whole), compound 0|1 (Box3D's Glue as One Body).",
          {"sep": 0.15, "strength": 1e9, "compound": 1, "frames": 60},
          sweep=[{"compound": 1}, {"compound": 0}, {"sep": 0.1, "compound": 1}])
def scene_fracture(kind, p):
    geo = new_geo("fracture")
    box = create(geo, "box", "box")
    xf = create(geo, "xform", "xf")
    xf.setInput(0, box)
    set_parm(xf, ["rx"], 45)
    set_parm(xf, ["ty"], 3)
    pfv = create(geo, "pointsfromvolume", "pfv")
    pfv.setInput(0, xf)
    set_parm(pfv, ["jitterscale"], 1.0)
    set_parm(pfv, ["particlesep"], float(p["sep"]))
    fracture = create(geo, "voronoifracture", "fracture")
    fracture.setInput(0, xf)
    fracture.setInput(1, pfv)
    # Voronoi's constraints have no constraint_name or strength: both solvers get the same.
    glue = create(geo, "attribwrangle", "glue_name")
    glue.setInput(0, fracture, 1)
    set_parm(glue, ["class"], "primitive")
    set_parm(glue, ["snippet"], 's@constraint_name = "Glue"; f@strength = %r;' % float(p["strength"]))
    solver = create(geo, "box3d_solver" if kind == "box3d" else "rbdbulletsolver", "solver")
    solver.setInput(0, fracture, 0)
    solver.setInput(1, glue)
    display(solver)
    set_parm(solver, ["useground"], 1)
    if kind == "box3d" and p.get("compound") is not None:
        set_parm(solver, ["gluecompound"], p["compound"])
    inner = [c for c in solver.allSubChildren() if c.type().name() == "dopnet"]
    if not inner:
        raise SetupError("the solver SOP has no DOP network inside")
    frames = int(p["frames"])
    return Scene(inner[0], frames, [("fall", 2, 18), ("land", 19, 40), ("settled", 41, frames)], sop=solver)


@scenario("bondwall", "The wall with a Box3D Bond between neighbouring bricks instead of Glue: the cost of reading each "
          "bond's load every step (as one compound, a spring network solved for every awake compound) and of its patch "
          "and tear impulse. Options: w, h, strength (Pa; tensile, shear and bending, compressive is 10 times it; the "
          "default holds the wall intact until the hit), group, compound 0|1, readouts 0|1 (Write Bond Attributes), "
          "wait (frames the wall stands before the ball arrives, a `stand` phase). Box3D only.",
          {"w": 40, "h": 20, "strength": 1e7, "group": 0, "frames": 120},
          sweep=[{"compound": 1}, {"compound": 0}, {"compound": 1, "readouts": 1}, {"w": 20, "h": 10, "compound": 1},
                 {"compound": 1, "wait": 60, "frames": 200}],
          kinds=("box3d",))
def scene_bondwall(kind, p):
    return build_wall(kind, p, "bond")


# ---------------------------------------------------------------------------
# Paths the pile scenes do not take
# ---------------------------------------------------------------------------


def ground_grid(tag, size, res=2):
    """A SOP grid (size x size, res x res points), as a static collider's geometry: its path."""
    geo = new_geo(tag)
    grid = create(geo, "grid")
    set_parm(grid, ["size"], (size, size))
    set_parm(grid, ["rows"], res)
    set_parm(grid, ["cols"], res)
    display(grid)
    return grid.path()


@scenario("objects", "N RBD Objects, one DOP object each (an RBD Point Object instancing a box on every point of a "
          "lattice), dropped onto a Static Object grid: the whole-object path, which the packed-point scenes never "
          "take. Per-object overhead shows here, not in the pile.",
          {"n": 500, "frames": 120},
          sweep=[{"n": 100}, {"n": 500}, {"n": 2000}])
def scene_objects(kind, p):
    n = int(p["n"])
    nx, ny, nz = pile_dims(n, ny=6)
    dop = new_dopnet("objects")
    geo = new_geo("objects_points")
    pts = create(geo, "attribwrangle", "lattice")
    set_parm(pts, ["class"], 0)
    set_parm(pts, ["snippet"], (
        f"for (int k = 0; k < {ny}; k++) for (int j = 0; j < {nz}; j++) for (int i = 0; i < {nx}; i++)\n"
        f"  addpoint(0, set(i - {nx} * 0.5 + 0.5, 0.6 + k * 1.05, j - {nz} * 0.5 + 0.5) "
        "+ 0.05 * (rand(set(i, j, k)) - 0.5));"))
    display(pts)
    box_geo = new_geo("objects_box")
    box = create(box_geo, "box")
    set_parm(box, ["size"], (0.8, 0.8, 0.8))
    display(box)
    objs = create(dop, "rbdpointobject", "objects")
    set_parm(objs, ["soppath"], pts.path())
    set_parm(objs, ["usegeo"], 1)
    set_parm(objs, ["geopath", "instancepath", "geometrypath", "instance"], box.path())
    ground = create(dop, "staticobject", "ground")
    set_parm(ground, ["soppath"], ground_grid("objects_ground", max(40.0, 2.0 * max(nx, nz) + 6.0)))
    last, _ = make_solver(kind, dop, merge(dop, [objs, ground]), p)
    finish(dop, last)
    return Scene(dop, int(p["frames"]), phases_for(int(p["frames"])))


@scenario("static", "A pile of N boxes dropped onto a Static Object made of SOP geometry (a noisy grid, res x res): "
          "motion none (still), move (the object's transform moves each frame) or deform (the geometry itself "
          "changes each frame, Animate Geometry on). Box3D reads the whole collider again when it changes.",
          {"n": 2000, "res": 100, "motion": "none", "frames": 120},
          sweep=[{"motion": "none"}, {"motion": "move"}, {"motion": "deform"}])
def scene_static(kind, p):
    n, res, motion = int(p["n"]), int(p["res"]), p["motion"]
    nx, ny, nz = pile_dims(n)
    dop = new_dopnet("static")
    path, _ = lattice_geo("static_pieces", nx, ny, nz)
    pieces = packed_object(dop, "pieces", path)
    width = max(40.0, 2.0 * max(nx, nz) + 6.0)
    geo = new_geo("static_collider")
    grid = create(geo, "grid")
    set_parm(grid, ["size"], (width, width))
    set_parm(grid, ["rows"], res)
    set_parm(grid, ["cols"], res)
    wr = create(geo, "attribwrangle", "noise")
    set_parm(wr, ["class"], 2)
    # `deform` makes the surface ripple with the frame; the others hold it still
    ripple = " * sin(@Frame * 0.3)" if motion == "deform" else ""
    set_parm(wr, ["snippet"], f"@P.y += 0.3 * (noise(@P * 0.4) - 0.3){ripple};")
    wr.setInput(0, grid)
    display(wr)
    ground = create(dop, "staticobject", "collider")
    set_parm(ground, ["soppath"], wr.path())
    if motion == "move":
        set_parm(ground, ["usetransform"], 1)
        set_expression(geo, ["ty"], "0.4 * sin($FF * 0.2)")
    elif motion == "deform":
        set_parm(ground, ["animategeo"], 1)
    last, _ = make_solver(kind, dop, merge(dop, [pieces, ground]), p)
    finish(dop, last)
    return Scene(dop, int(p["frames"]), phases_for(int(p["frames"])))


@scenario("churn", "A pile that loses and gains bodies every frame: `rate` boxes die each frame (Enable Aging with a "
          "staggered `life`, Reap Dead Points) and, with births 1, `rate` new boxes are made each frame by a SOP "
          "Solver. The cost of removing and creating bodies in a running scene. Box3D only.",
          {"n": 2000, "rate": 20, "births": 0, "frames": 100},
          sweep=[{"rate": 0}, {"rate": 20}, {"rate": 20, "births": 1}, {"rate": 100, "births": 1}],
          kinds=("box3d",))
def scene_churn(kind, p):
    n, rate, births = int(p["n"]), int(p["rate"]), int(p["births"])
    nx, ny, nz = pile_dims(n)
    dop = new_dopnet("churn")
    # piece number i dies on frame 2 + i // rate: its life, at 24 frames a second (a rate of 0 kills none)
    life_of = f"(2.0 + floor(n / {rate}.0)) / 24.0" if rate else "1.0e6"
    # An `id` keeps a body its own when the points around it die (without one the point number is the body, and every
    # death renumbers the points after it).
    life = ('addpointattrib(0, "life", 1.0e6); addpointattrib(0, "age", 0.0); addpointattrib(0, "id", 0); '
            f'setpointattrib(0, "life", pt, {life_of}); setpointattrib(0, "id", pt, n);')
    path, _ = lattice_geo("churn_pieces", nx, ny, nz, extra=life)
    pieces = packed_object(dop, "pieces", path)
    ground = packed_object(dop, "ground", ground_for("churn_ground", nx, nz, life=1.0e6, age=0.0))
    upstream = merge(dop, [pieces, ground])
    if births:
        ss = create(dop, "sopsolver", "emit")
        outs = [c for c in ss.children() if c.type().name() == "output"]
        if not outs:
            raise SetupError("the SOP Solver has no Output SOP to add births in front of")
        out = outs[0]
        src = ss.createNode("attribwrangle", "new_points")
        set_parm(src, ["class"], 0)
        set_parm(src, ["snippet"], (
            'addpointattrib(0, "name", ""); addpointattrib(0, "active", 1); addpointattrib(0, "life", 1.0e6); '
            'addpointattrib(0, "age", 0.0); addpointattrib(0, "id", 0);\n'
            'if (npoints("op:../dop_geometry") > 50) '
            f"for (int i = 0; i < {rate}; i++) {{ "
            f"int pt = addpoint(0, set(({nx} - 2) * (rand(set(@Frame, i, 1)) - 0.5), {ny} + 6.0, "
            f"({nz} - 2) * (rand(set(@Frame, i, 2)) - 0.5))); "
            'setpointattrib(0, "name", pt, sprintf("n%d_%d", int(@Frame), i)); '
            f'setpointattrib(0, "id", pt, 1000000 + int(@Frame) * {max(rate, 1)} + i); '
            f'setpointattrib(0, "life", pt, {max(1, n // max(rate, 1))} / 24.0); }}'))
        box = ss.createNode("box", "new_box")
        set_parm(box, ["size"], (0.8, 0.8, 0.8))
        copy = ss.createNode("copytopoints", "new_bodies")
        copy.setInput(0, box)
        copy.setInput(1, src)
        set_parm(copy, ["pack", "packgeo"], 1)
        set_parm(copy, ["targetattribs"], 1)
        set_parm(copy, ["applyattribs1"], "name active life age id")
        mrg = ss.createNode("merge", "with_new")
        mrg.setInput(0, out.input(0))
        mrg.setInput(1, copy)
        out.setInput(0, mrg)
        try:
            set_parm(ss, ["group", "objects"], "pieces")
        except SetupError:
            pass
    solver = box3d_solver(dop, None if births else upstream)
    set_parm(solver, ["enableaging"], 1)
    set_parm(solver, ["reapdead"], 1)
    finish(dop, multisolver(dop, upstream, [ss, solver]) if births else solver)
    frames = int(p["frames"])
    return Scene(dop, frames, [("fall", 2, 30), ("churn", 31, frames)])


@scenario("solversop", "The Box3D Solver SOP asset on a pile of N boxes with a ground box on its Collision input: the "
          "frame is the SOP cook (input packing, collision preparation, the simulation, the outputs), where the "
          "other scenes time the DOP network alone. Box3D only.",
          {"n": 5000, "frames": 120},
          sweep=[{"n": 1000}, {"n": 5000}, {"n": 20000, "frames": 80}],
          kinds=("box3d",))
def scene_solversop(kind, p):
    n = int(p["n"])
    nx, ny, nz = pile_dims(n)
    pieces_path, _ = lattice_geo("sop_pieces", nx, ny, nz)
    ground_path = ground_for("sop_ground", nx, nz)
    geo = new_geo("sop_solver")
    solver = create(geo, "box3d_solver", "box3d_solver")
    for i, path in ((0, pieces_path), (3, ground_path)):
        merge_in = create(geo, "object_merge", f"input{i}")
        set_parm(merge_in, ["objpath1"], path)
        solver.setInput(i, merge_in)
    display(solver)
    inner = [c for c in solver.allSubChildren() if c.type().name() == "dopnet"]
    if not inner:
        raise SetupError("the Box3D Solver SOP has no DOP network inside")
    return Scene(inner[0], int(p["frames"]), phases_for(int(p["frames"])), sop=solver)


# ---------------------------------------------------------------------------
# Shapes and features, as variants of the pile
# ---------------------------------------------------------------------------


@scenario("hulls", "The pile with convex hull pieces of different sizes (a polygon sphere of rows x cols points), "
          "for the cost of building and stepping hulls: look at `first` / frame 2 and Pre-Solve.",
          {"n": 2000, "frames": 120},
          sweep=[{"shape": "sphere:6x8"}, {"shape": "sphere:12x16"}, {"shape": "sphere:24x32"}])
def scene_hulls(kind, p):
    return scene_pile(kind, p)


def feature_kinds(p):
    return ("box3d",) if any(p.get(k) is not None for k in ("impacts", "exportshapes", "sleep")) else ("box3d", "bullet")


@scenario("features", "The 5000-box pile with one setting changed: Add Impact Data, Export Collision Shapes, gravity as a "
          "Force DOP, Substeps 4, sleeping off. Box3D only where Bullet has no such setting.",
          {"n": 5000, "frames": 120, "shape": "box"},
          sweep=[{}, {"impacts": 1}, {"exportshapes": 1}, {"forcedop": 1}, {"substeps": 4}, {"sleep": 0}],
          kinds_for=feature_kinds)
def scene_features(kind, p):
    return scene_pile(kind, p)
