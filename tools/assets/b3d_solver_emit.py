"""Box3D Solver SOP: Emit RBDs and Replicate RBDs, as RBD Bullet Solver's.

With Emit RBDs on, every step adds the input pieces again as new bodies, each renamed `<name>_<int(frame * 100)>` (the
pieces of the first frame too), unless their `rbdbullet_emit` point attribute is 0 (1 where missing). The constraints
between two emitted pieces are added with them, renamed the same way. Emission is controlled by animating the input
(pieces that appear, move or leave) or `rbdbullet_emit`; a new piece starts with its own `v` and `w`.

Built as RBD Bullet Solver builds it:
* SOP side: `emit_flag` (`rbdbullet_emit` where missing) and `rename_emitted` before the DOP network, so the creation
  frame's pieces are renamed too; `rename_emitted_constraints` on the constraints.
* A SOP Solver DOP per kind of data on the pieces object, each merging what this step adds into it: `emit_pieces`
  (Geometry: the emitting pieces from the DOP network's first input, with the rest a new piece gets on its creation
  frame, b3d_settings.REST_VEX), `emit_constraints` (ConstraintGeometry), and `emit_geometry` / `emit_proxy`, which
  keep the emitted pieces' render geometry (EmittedGeometry / EmittedProxy data, first attached by SOP Geometry DOPs on
  the creation frame) as it was when they were emitted. They run after the solver step, or before it with Solve on
  Creation Frame, so a new piece first moves on the frame after it appears, or on that frame.
* The Geometry and Proxy outputs take the non-emitting render pieces plus the stored ones before Transform Pieces.

Replicate RBDs is for pieces duplicated during the simulation (a SOP Solver or POP node in FORCES copying points):
several simulation points then share a name and a source piece. Only the outputs change, as RBD Bullet Solver's:
* `set_sourcename` gives every piece `__sourcename` (its name, after Emit RBDs' rename) before the DOP network, so a
  copy carries its source's.
* The simulation points keep `__sourcename` and `__restxform` (a copy's starting transform, identity where missing:
  the user's to set). `rename_copies` renames the points sharing a name: the first keeps it, the n-th after it is
  `<name>_r<n>`. The Simulation Points output and every Transform Pieces use these names.
* `replica_points`: one point per copy (not the first point of its source), named after its source, at the origin with
  `__restxform` as its `transform`. Copy to Points (Piece Attribute `name`) copies the source's render piece onto it,
  renamed to the copy's new name, and the copies join the render pieces before Transform Pieces moves them.
"""

import hou

import b3d_hda
import b3d_settings
from b3d_solver_common import (
    CONSTRAINTS_TOGGLES,
    create,
    DOP_CONSTRAINTS,
    DOP_PIECES,
    EMIT,
    FETCH_GEOMETRY,
    REPLICATE,
    set_menu,
    set_named,
    set_toggles,
    warn,
)

EMIT_ATTRIB = "rbdbullet_emit"  # RBD Bullet Solver's: 0 keeps a piece from being emitted
SOLVE_FIRST = "solvefirstframe"  # the asset's Solve on Creation Frame (b3d_solver_network.SOLVE_FIRST)
PIECES_OBJECT = "pieces"
# Per output: the SOP null holding the emitting render pieces of this step, renamed, and the DOP data keeping them.
RENDER = {"geometry": ("EMIT_GEOMETRY", "EmittedGeometry"), "proxy": ("EMIT_PROXY", "EmittedProxy")}
RENDER_FLAG = "__b3d_emit"  # a render piece's primitives: 1 when its simulated piece emits

SUFFIX = 'string suffix = "_" + itoa(int(@Frame * 100));'

# The emitting pieces renamed (point names: the solver reads those).
RENAME_VEX = f'''{SUFFIX}
if (i@{EMIT_ATTRIB} != 0) s@name += suffix;
'''
# A constraint point naming an emitting piece (input 1: the pieces before the rename) takes the same name.
RENAME_CONSTRAINTS_VEX = f'''{SUFFIX}
int pt = findattribval(1, "point", "name", s@name);
if (pt >= 0 && point(1, "{EMIT_ATTRIB}", pt) != 0) s@name += suffix;
'''
# Render pieces (named on their primitives, else their points): whether the simulated piece of that name emits.
RENDER_FLAG_VEX = f'''string name = hasprimattrib(0, "name") ? prim(0, "name", i@primnum) : "";
if (name == "" && haspointattrib(0, "name")) name = point(0, "name", primpoint(0, i@primnum, 0));
int pt = findattribval(1, "point", "name", name);
i@{RENDER_FLAG} = pt >= 0 && point(1, "{EMIT_ATTRIB}", pt) != 0;
'''
# Only the name attributes the pieces have are renamed (a binding would make the other).
RENAME_RENDER_VEX = f'''{SUFFIX}
if (hasprimattrib(0, "name"))
    setprimattrib(0, "name", i@primnum, prim(0, "name", i@primnum) + suffix);
if (haspointattrib(0, "name"))
    foreach (int pt; primpoints(0, i@primnum))
        setpointattrib(0, "name", pt, point(0, "name", pt) + suffix);
'''

# The emitting pieces of this step (input 0), against the pieces object's geometry (input 1) and the largest `id`
# there (input 2, detail `__maxid`). A piece whose name is there already was added this step (the creation frame's
# pieces with Solve on Creation Frame). An `id` is the body's identity, so a new piece gets one past the largest when
# the pieces have ids; a piece is given only the attributes the pieces have, as a merged attribute is made on all.
# Then the rest a piece gets on its object's creation frame.
EMIT_PIECES_VEX = '''if (findattribval(1, "point", "name", s@name) >= 0) {
    removepoint(0, i@ptnum);
    return;
}
if (haspointattrib(1, "id"))
    setpointattrib(0, "id", i@ptnum, detail(2, "__maxid", 0) + 1 + i@ptnum);
''' + b3d_settings.REST_VEX
# Constraint points of this step (input 0) are kept when they name a piece emitted this step (input 1) that the
# pieces' constraints do not name yet (input 2): a constraint losing a point is removed. As RBD Bullet Solver's, an
# emitted copy has only its constraints to other emitted pieces: those to the world, or to a piece that is not
# emitted, stay with the first frame's pieces (solver_sop_emit_bullet).
EMIT_CONSTRAINTS_VEX = '''i@group___b3d_drop = findattribval(1, "point", "name", s@name) < 0
                       || findattribval(2, "point", "name", s@name) >= 0;
'''
# The stored render pieces (input 0) lose those no piece has any more (input 1: the pieces object's geometry and this
# step's emitted pieces), and those added again this step (input 2).
PRUNE_RENDER_VEX = '''string name = hasprimattrib(0, "name") ? prim(0, "name", i@primnum) : "";
if (name == "") name = point(0, "name", primpoint(0, i@primnum, 0));
int again = findattribval(2, "prim", "name", name) >= 0 || findattribval(2, "point", "name", name) >= 0;
if (again || findattribval(1, "point", "name", name) < 0)
    removeprim(0, i@primnum, 1);
'''


def _switch(sub, name, off, on):
    """A Switch on Emit RBDs (a node in the asset's own network)."""
    sw = create(sub, "switch", name)
    sw.setInput(0, off)
    sw.setInput(1, on)
    sw.parm("input").setExpression(f'ch("../{EMIT}")', hou.exprLanguage.Hscript)
    return sw


def _wrangle(parent, name, inputs, snippet, run_over="point"):
    wr = create(parent, "attribwrangle", name)
    for i, src in enumerate(inputs):
        wr.setInput(i, src)
    wr.parm("class").set(run_over)
    wr.parm("snippet").set(snippet)
    wr.parm("vex_precision").set("64")  # Auto rounds P to float: pieces far from the origin keep their position
    return wr


def _blast(parent, name, src, group, grouptype, keep=False):
    """A Blast deleting the `grouptype` elements in `group` (or keeping only them), and its groups left empty."""
    node = create(parent, "blast", name)
    node.setInput(0, src)
    node.parm("group").set(group)
    set_menu(node, "grouptype", grouptype)
    set_named(node, "negate", int(keep))
    set_named(node, "removegrp", 1)
    return node


def emit_pieces(sub, src):
    """The pieces with `rbdbullet_emit` (1 where missing) and the emitting ones renamed, behind a Switch on Emit RBDs.
    Returns (the switch, the flagged pieces before the rename, which the constraints and outputs look up)."""
    flag = b3d_hda.attribute_create(sub, "emit_flag", [(EMIT_ATTRIB, "int", None)], warn, write=False)
    flag.setInput(0, src)
    if flag.parm("default1v1") is None:
        warn(f"{flag.path()} has no Default: pieces without {EMIT_ATTRIB} are not emitted")
    else:
        flag.parm("default1v1").set(1)
    rename = _wrangle(sub, "rename_emitted", [flag], RENAME_VEX)
    return _switch(sub, "switch_emit_pieces", src, rename), flag


def emit_constraints(sub, src, flags):
    """The constraint points naming an emitting piece renamed as it is, behind a Switch on Emit RBDs."""
    rename = _wrangle(sub, "rename_emitted_constraints", [src, flags], RENAME_CONSTRAINTS_VEX)
    return _switch(sub, "switch_emit_constraints", src, rename)


def emit_render(sub, named, flags, kind):
    """For the Geometry (kind "geometry") or Proxy ("proxy") output: the emitting render pieces of this step, renamed,
    as the null the DOP network stores them from, and in place of `named` (behind a Switch on Emit RBDs) the other
    render pieces with the stored ones. Returns the switch."""
    null_name, data = RENDER[kind]
    flagged = _wrangle(sub, f"emit_flag_{kind}", [named, flags], RENDER_FLAG_VEX, run_over="primitive")
    emitting = _blast(sub, f"emit_keep_{kind}", flagged, f"@{RENDER_FLAG}=1", "prims", keep=True)
    renamed = _wrangle(sub, f"rename_emit_{kind}", [emitting], RENAME_RENDER_VEX, run_over="primitive")
    clean = create(sub, "attribdelete", f"emit_attrib_{kind}")
    clean.setInput(0, renamed)
    set_named(clean, "primdel", RENDER_FLAG)
    create(sub, "null", null_name).setInput(0, clean)
    rest = _blast(sub, f"emit_rest_{kind}", flagged, f"@{RENDER_FLAG}=1", "prims")
    stored = create(sub, "dopimport", f"import_emitted_{kind}")
    stored.parm("doppath").set("../dopnet")
    stored.parm("objpattern").set(PIECES_OBJECT)
    set_menu(stored, "importstyle", FETCH_GEOMETRY)
    set_named(stored, "geodatapath", data)
    set_toggles(stored, CONSTRAINTS_TOGGLES)  # as stored: nothing transformed, packed or tagged
    both = create(sub, "merge", f"merge_emitted_{kind}")
    both.setInput(0, rest)
    both.setInput(1, stored)
    done = create(sub, "attribdelete", f"emit_clean_{kind}")
    done.setInput(0, both)
    set_named(done, "primdel", RENDER_FLAG)
    return _switch(sub, f"switch_emit_{kind}", named, done)


def emission_objects(dopnet, pieces):
    """The pieces object with the EmittedGeometry / EmittedProxy data its creation frame's emitting render pieces start
    (SOP Geometry DOPs, Set Initial), behind a Switch on Emit RBDs. Returns the switch."""
    last = pieces
    for kind, (null_name, data) in RENDER.items():
        geo = create(dopnet, "sopgeo", f"emitted_{kind}")
        geo.setInput(0, last)
        set_named(geo, "soppath", f"../../{null_name}")
        set_named(geo, "dataname", data)
        last = geo
    sw = create(dopnet, "switch", "use_emission")
    sw.setInput(0, pieces)
    sw.setInput(1, last)
    sw.parm("input").setExpression(f'ch("{sw.relativePathTo(dopnet.parent())}/{EMIT}")', hou.exprLanguage.Hscript)
    return sw


def _emit_solver(dopnet, name, data, add):
    """A SOP Solver DOP over the pieces object's `data`: `add(solver, existing)` (`existing` the node importing the
    data) returns (the data to keep, what this step adds), merged as the output."""
    solver = create(dopnet, "sopsolver", name)
    set_named(solver, "datapath", data)
    set_named(solver, "dataname", name)
    set_named(solver, "group", PIECES_OBJECT)  # not the collider and ground
    outputs = [c for c in solver.children() if c.type().name() == "output"]
    if not outputs or not outputs[0].inputs():
        warn(f"{solver.path()} has no Output fed by its geometry import: Emit RBDs does not add {data}")
        return solver
    kept, new = add(solver, outputs[0].inputs()[0])
    both = solver.createNode("merge", "merge_emitted")
    both.setInput(0, kept)
    both.setInput(1, new)
    outputs[0].setInput(0, both)
    b3d_hda.tidy(solver)
    return solver


def _context(solver, name, index):
    """An Object Merge of the DOP network's input `index` (the SOP feeding it), inside a SOP Solver."""
    node = solver.createNode("object_merge", name)
    node.parm("objpath1").set(f'`dopcontextgeo("..", {index})`')
    return node


def _emitting_pieces(solver):
    """This step's emitting pieces, renamed: the DOP network's pieces input without the pieces set not to emit."""
    return _blast(solver, "emitting", _context(solver, "pieces_input", DOP_PIECES), f"@{EMIT_ATTRIB}=0", "points")


def _add_pieces(solver, existing):
    top = solver.createNode("attribpromote", "max_id")
    top.setInput(0, existing)
    set_menu(top, "inclass", "point")
    set_menu(top, "outclass", "detail")
    set_menu(top, "method", "max")
    set_named(top, "inname", "id")
    set_named(top, "useoutname", 1)
    set_named(top, "outname", "__maxid")
    set_named(top, "deletein", 0)
    return existing, _wrangle(solver, "new_pieces", [_emitting_pieces(solver), existing, top], EMIT_PIECES_VEX)


def _add_constraints(solver, existing):
    marked = _wrangle(solver, "mark_constraints",
                      [_context(solver, "constraints_input", DOP_CONSTRAINTS), _emitting_pieces(solver), existing],
                      EMIT_CONSTRAINTS_VEX)
    return existing, _blast(solver, "new_constraints", marked, "__b3d_drop", "points")


def _add_render(null_name):
    def add(solver, existing):
        new = solver.createNode("object_merge", "new_render")
        new.parm("objpath1").set(f"../../../{null_name}")
        pieces = solver.createNode("dopimport", "pieces_geometry")
        pieces.parm("doppath").set('`stamps("..", "DOPNET", "../..")`')
        pieces.parm("objpattern").set(PIECES_OBJECT)
        set_menu(pieces, "importstyle", FETCH_GEOMETRY)
        set_named(pieces, "geodatapath", "Geometry")
        set_named(pieces, "donotsim", 1)
        names = solver.createNode("merge", "piece_names")
        names.setInput(0, pieces)
        names.setInput(1, _context(solver, "pieces_input", DOP_PIECES))
        return _wrangle(solver, "prune_render", [existing, names, new], PRUNE_RENDER_VEX, run_over="primitive"), new
    return add


def build_emission(dopnet):
    """The emission SOP Solvers, each behind an Enable Solver DOP for the step before the solver and one for the step
    after (Solve on Creation Frame picks), merged into `emission_before` and `emission_after` for the Multiple Solver.
    Returns (emission_before, emission_after)."""
    solvers = [
        (_emit_solver(dopnet, "emit_pieces", "Geometry", _add_pieces), None),
        # Only while the Constraints input has points: else the pieces have no ConstraintGeometry to add to.
        (_emit_solver(dopnet, "emit_constraints", "ConstraintGeometry", _add_constraints),
         f'npoints(opinputpath("..", {DOP_CONSTRAINTS})) > 0'),
        (_emit_solver(dopnet, "emit_geometry", RENDER["geometry"][1], _add_render(RENDER["geometry"][0])), None),
        # Only while a Proxy input is connected: else the Proxy output is empty.
        (_emit_solver(dopnet, "emit_proxy", RENDER["proxy"][1], _add_render(RENDER["proxy"][0])),
         'npoints("{asset}/IN_proxy") > 0'),
    ]
    merges = []
    for when, first in (("before", "1"), ("after", "0")):
        merge = create(dopnet, "merge", f"emission_{when}")
        for i, (solver, condition) in enumerate(solvers):
            enable = create(dopnet, "enablesolver", f"enable_{solver.name()}_{when}")
            enable.setInput(0, solver)
            asset = enable.relativePathTo(dopnet.parent())
            expr = f'ch("{asset}/{EMIT}") && ch("{asset}/{SOLVE_FIRST}") == {first}'
            if condition:
                expr += f" && {condition.format(asset=asset)}"
            enable.parm("enable").setExpression(expr, hou.exprLanguage.Hscript)
            merge.setInput(i, enable)
        merges.append(merge)
    return tuple(merges)


SOURCE_NAME, SOURCE_XFORM = "__sourcename", "__restxform"  # RBD Bullet Solver's names
SOURCE_VEX = f"s@{SOURCE_NAME} = s@name;"
# The simulation points sharing a name: the first keeps it, the n-th after it becomes <name>_r<n>.
RENAME_COPIES_VEX = """int pts[] = findattribval(0, "point", "name", s@name);
if (min(pts) == i@ptnum) return;
s@name += "_r" + itoa(find(pts, i@ptnum));
"""
# A copy's replica point: named after its source (the piece to copy), its new name in __newname, at the origin with its
# starting transform. The first point named after its source needs none: the render piece is its own.
REPLICA_VEX = f"""string source = s@{SOURCE_NAME};
int same[] = findattribval(0, "point", "name", source);
if (source == "" || (len(same) > 0 && min(same) == i@ptnum)) {{
    removepoint(0, i@ptnum);
    return;
}}
int pts[] = findattribval(0, "point", "name", s@name);
s@__newname = min(pts) < i@ptnum ? s@name + "_r" + itoa(find(pts, i@ptnum)) : s@name;
s@name = source;
v@P = 0;
matrix xf = ident();
if (haspointattrib(0, "{SOURCE_XFORM}"))
    xf = point(0, "{SOURCE_XFORM}", i@ptnum);
4@transform = xf;
"""
# The copied render pieces take their copy's name (on the primitives and the points, whichever the pieces have).
RENAME_REPLICAS_VEX = """string n = point(0, "__newname", primpoint(0, i@primnum, 0));
if (hasprimattrib(0, "name"))
    setprimattrib(0, "name", i@primnum, n);
if (haspointattrib(0, "name"))
    foreach (int pt; primpoints(0, i@primnum))
        setpointattrib(0, "name", pt, n);
"""


def _replicate_switch(sub, name, off, on):
    """A Switch on Replicate RBDs (a node in the asset's own network)."""
    sw = create(sub, "switch", name)
    sw.setInput(0, off)
    sw.setInput(1, on)
    sw.parm("input").setExpression(f'ch("../{REPLICATE}")', hou.exprLanguage.Hscript)
    return sw


def replicate_source(sub, src):
    """The pieces with `__sourcename` (their name), behind a Switch on Replicate RBDs."""
    return _replicate_switch(sub, "switch_set_sourcename", src, _wrangle(sub, "set_sourcename", [src], SOURCE_VEX))


def replicate_points(sub, points):
    """The simulation points (with `__sourcename` / `__restxform` while Replicate RBDs is on): returns (the points with
    the copies renamed and those two attributes removed, behind a Switch on Replicate RBDs; the replica points)."""
    renamed = _wrangle(sub, "rename_copies", [points], RENAME_COPIES_VEX)
    clean = create(sub, "attribdelete", "drop_sourcename")
    clean.setInput(0, renamed)
    set_named(clean, "ptdel", f"{SOURCE_NAME} {SOURCE_XFORM}")
    # Only what the replica points need, so Copy to Points applies no orient, scale or velocity.
    bare = create(sub, "attribdelete", "replica_attribs")
    bare.setInput(0, points)
    set_named(bare, "ptdel", f"name {SOURCE_NAME} {SOURCE_XFORM}")
    if not set_named(bare, "negate", 1):  # Delete Non Selected
        bare.parm("ptdel").set("")
    replicas = _wrangle(sub, "replica_points", [bare], REPLICA_VEX)
    return _replicate_switch(sub, "switch_rename_copies", points, clean), replicas


def replicate_render(sub, named, replicas, kind):
    """For the Geometry or Proxy output: the render pieces with a copy of its source's piece for every copy, renamed,
    behind a Switch on Replicate RBDs."""
    copies = create(sub, ["copytopoints::2.0", "copytopoints"], f"replicate_{kind}")
    copies.setInput(0, named)
    copies.setInput(1, replicas)
    set_named(copies, "useidattrib", 1)  # Piece Attribute
    set_named(copies, "idattrib", "name")
    set_named(copies, "targetattribs", 1)
    set_named(copies, "applyattribs1", "__newname")
    renamed = _wrangle(sub, f"rename_replicas_{kind}", [copies], RENAME_REPLICAS_VEX, run_over="primitive")
    both = create(sub, "merge", f"merge_replicas_{kind}")
    both.setInput(0, named)
    both.setInput(1, renamed)
    clean = create(sub, "attribdelete", f"replica_clean_{kind}")
    clean.setInput(0, both)
    set_named(clean, "ptdel", "__newname")
    return _replicate_switch(sub, f"switch_replicate_{kind}", named, clean)
