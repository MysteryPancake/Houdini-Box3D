"""Input preparation shared by make_solver_sop.py and make_configure_sop.py, copied from RBD Bullet
Solver's Housekeeping, Housecleaning and Collisions boxes and RBD Configure's packing (read with
tools/houdini_network_dump.py): packed primitives are found (one point, not a polygon, alone in its
connectivity class) and split from the rest.

Pack mode (Housekeeping, RBD Configure): packed pieces get a point `name` (the point's, else the
primitive's, else <prefix><ptnum>) and lose the primitive's; unpacked geometry is assembled into
packed pieces by `name` (or by connectivity when nothing has a name), which puts the name on the
point. The rest is left to the objects the result goes into (the Box3D Packed Object's SOP stage,
as RBD Packed Object's).
Collision mode (Collisions): as pack mode, but packed colliders keep a primitive `name` unless it
is only on the primitive (then it moves to the point), and assembled ones are prefixed `static`.
Names mode (Housecleaning): nothing is packed, pieces only get names, made the same way so they
match pack mode's.

Not a build script: imported by the two builders (run with hython, so this folder is on sys.path).
"""

import hou

PACKED_GROUP = "__packed"
DEFAULT_PREFIX = "piece"  # Assemble's and RBD Configure's Name Prefix

GROUP_PACKED_VEX = """\
if (len(primpoints(0, @primnum)) != 1) return;
if (string(primintrinsic(0, "typename", @primnum)) == "Poly") return;
int class = prim(1, "class", @primnum);
if (findattribvalcount(1, "prim", "class", class) > 1) return;
i@group___packed = 1;
"""

# Primitives: an unpacked primitive without a name takes its point's.
PRIM_NAME_FROM_POINT_VEX = """\
string name = prim(0, "name", @primnum);
if (name != "" || !haspointattrib(0, "name")) return;
name = point(0, "name", @ptnum);
if (name != "") setprimattrib(0, "name", @primnum, name);
"""

# Points of packed pieces: the point's name, else the primitive's, else <prefix><ptnum>.
POINT_NAME_VEX = """\
if (s@name != "") return;
if (hasprimattrib(0, "name")) s@name = prim(0, "name", @primnum);
if (s@name == "") s@name = chs("prefix") + itoa(@ptnum);
"""

# Names mode, points of packed pieces: as POINT_NAME_VEX, written where the name already lives.
SET_NAME_VEX = """\
string name = point(0, "name", @ptnum);
if (hasprimattrib(0, "name") && name == "")
    name = prim(0, "name", @primnum);
if (name == "") name = chs("prefix") + itoa(@ptnum);
if (hasprimattrib(0, "name") || !haspointattrib(0, "name"))
    setprimattrib(0, "name", @primnum, name);
else
    setpointattrib(0, "name", @ptnum, name);
"""

# Collision mode, packed colliders: Bullet's set_name_if_needed3 (the point's name wins when it has
# one, else the primitive's, else <prefix><ptnum>). The primitive's name is kept unless it is the
# only one (Bullet's switchif6 promotes it to the point then: delete_prim_name_if_only).
COLLISION_NAME_VEX = """\
string name = "";
if (hasprimattrib(0, "name"))
    name = prim(0, "name", @primnum);
if (haspointattrib(0, "name") || name == "")
    name = point(0, "name", @ptnum);
if (name == "") name = chs("prefix") + itoa(@ptnum);
s@name = name;
"""
COLLISION_PREFIX = "static"  # Bullet's Output Prefix for assembled colliders
# Bullet's Path Attribute on its Assembles.
PACK_PATH = "op:`opfullpath('.')`/pack"


def _set(node, warn, parms=None, exprs=None):
    """Values, then HScript expressions, onto `node`'s parameters; a missing one is a warning."""
    for k, v in (parms or {}).items():
        p = node.parm(k)
        if p is None:
            warn(f"{node.path()} has no parameter {k!r}")
            continue
        try:
            p.set(v)
        except (TypeError, hou.Error) as e:
            warn(f"{p.path()} = {v!r}: {e}")
    for k, v in (exprs or {}).items():
        p = node.parm(k)
        if p is None:
            warn(f"{node.path()} has no parameter {k!r}")
            continue
        p.setExpression(v, hou.exprLanguage.Hscript)


def prepare_input(parent, src, name, pack, warn, prefix=DEFAULT_PREFIX, prefix_expr=None,
                  assemble_parms=None, assemble_exprs=None, collision=False, remove_unused_groups=False):
    """A subnet `name` in `parent` fed by `src` that prepares it (pack, collision or names mode, see
    the module docstring). Returns the subnet.

    prefix / prefix_expr: the name prefix for pieces that get a new name, a value or an HScript
    expression (relative to a node inside the subnet, so ../../<parm> is the asset's).
    assemble_parms / assemble_exprs: extra values / expressions for the Assemble (Transfer
    Attributes, Create Path Attribute ...). collision: collision mode (needs pack). remove_unused_groups: the Split's Delete
    Unused Groups, on in RBD Configure and off in RBD Bullet Solver."""
    net = parent.createNode("subnet", name)
    net.setInput(0, src)
    for child in net.children():
        child.destroy()
    inp = net.indirectInputs()[0]

    def node(type_name, node_name, inputs=(), parms=None, exprs=None):
        n = net.createNode(type_name, node_name)
        for i, item in enumerate(inputs):
            if isinstance(item, tuple):
                n.setInput(i, item[0], item[1])
            else:
                n.setInput(i, item)
        _set(n, warn, parms, exprs)
        return n

    def with_prefix(n):
        """A `prefix` spare parameter for the VEX's chs("prefix")."""
        n.addSpareParmTuple(hou.StringParmTemplate("prefix", "Name Prefix", 1,
                                                   default_value=(DEFAULT_PREFIX,)))
        if prefix_expr:
            n.parm("prefix").setExpression(prefix_expr, hou.exprLanguage.Hscript)
        else:
            n.parm("prefix").set(prefix)
        return n

    conn = node("connectivity", "connectivity", [inp], {"connecttype": "prim"})
    grp = node("attribwrangle", "group_packed", [inp, conn],
               {"class": "primitive", "snippet": GROUP_PACKED_VEX})
    split_parms = {"group": PACKED_GROUP, "grouptype": "prims"}
    if remove_unused_groups:
        split_parms["removegrp"] = 1
    split = node("split", "split_packed", [grp], split_parms)
    has_name = 'hasprimattrib(opinputpath(".", 0), "name")'
    asm_parms = dict(assemble_parms or {})
    asm_exprs = dict(assemble_exprs or {})
    if collision:
        asm_parms.setdefault("outside_group", COLLISION_PREFIX)
        asm_parms.setdefault("connect", 0)
    elif prefix_expr:
        asm_exprs.setdefault("outside_group", prefix_expr)
    else:
        asm_parms.setdefault("outside_group", prefix)
    if collision:
        # Packed colliders (Bullet's set_name_if_needed3 / switchif6); assembled ones as pack mode.
        set_name = with_prefix(node("attribwrangle", "set_name_if_needed", [(split, 0)],
                                    {"grouptype": "points", "snippet": COLLISION_NAME_VEX,
                                     "exportlist": "name"}))
        only_prim = (f'hasprimattrib("../{split.name()}", "name") && '
                     f'!haspointattrib("../{split.name()}", "name")')
        packed = node("attribdelete", "delete_prim_name_if_only", [set_name], {},
                      {"primdel": f'ifs({only_prim}, "name", "")'})
        prim_name = node("attribwrangle", "prim_name_from_point", [(split, 1)],
                         {"class": "primitive", "snippet": PRIM_NAME_FROM_POINT_VEX})
        asm_parms["pack_geo"] = 1
        asm_exprs["newname"] = f"1 - {has_name}"
        unpacked = node("assemble", "assemble", [prim_name], asm_parms, asm_exprs)
    elif pack:
        # Packed pieces named on their points before the merge (promote_name_if_needed,
        # attribdelete_name); the Assemble names the assembled ones' points itself.
        point_name = with_prefix(node("attribwrangle", "point_name", [(split, 0)],
                                      {"grouptype": "points", "snippet": POINT_NAME_VEX,
                                       "exportlist": "name"}))
        packed = node("attribdelete", "delete_prim_name", [point_name], {"primdel": "name"})
        prim_name = node("attribwrangle", "prim_name_from_point", [(split, 1)],
                         {"class": "primitive", "snippet": PRIM_NAME_FROM_POINT_VEX})
        asm_parms["pack_geo"] = 1
        asm_exprs["newname"] = f"1 - {has_name}"
        unpacked = node("assemble", "assemble", [prim_name], asm_parms, asm_exprs)
    else:
        packed = with_prefix(node("attribwrangle", "set_name_if_needed", [(split, 0)],
                                  {"grouptype": "points", "snippet": SET_NAME_VEX, "exportlist": "name"}))
        asm_parms["pack_geo"] = 0
        asm_exprs["newname"] = f'1 - ({has_name} || haspointattrib(opinputpath(".", 0), "name"))'
        unpacked = node("assemble", "assemble", [(split, 1)], asm_parms, asm_exprs)
    # An empty branch is replaced by an empty null, as the SideFX assets do (the packed branch of
    # names mode goes straight to the merge, as in Bullet's Housecleaning box).
    empty = node("null", "empty")
    keep_packed = node("switchif", "switchif_packed", [packed, empty], {"type1": "count"}) \
        if (pack or collision) else packed
    keep_unpacked = node("switchif", "switchif_unpacked", [unpacked, empty], {"type1": "count"})
    merged = node("merge", "merge", [keep_packed, keep_unpacked])
    last = merged
    ungroup = node("groupdelete", "delete_group", [last],
                   {"grouptype1": "prims", "group1": PACKED_GROUP})
    out = node("output", "output0", [ungroup])
    out.setDisplayFlag(True)
    out.setRenderFlag(True)
    net.layoutChildren()
    return net
