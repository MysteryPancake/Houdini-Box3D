"""Input preparation shared by make_solver_sop.py and make_configure_sop.py, copied from RBD Bullet
Solver's Housekeeping, Housecleaning and Collisions boxes and RBD Configure's packing (read with
tools/dev/houdini_network_dump.py): packed primitives are found (one point, not a polygon, alone in its
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

import b3d_hda
from b3d_hda import tidy

PACKED_GROUP = "__packed"
DEFAULT_PREFIX = "piece"  # Assemble's and RBD Configure's Name Prefix

GROUP_PACKED_VEX = """\
if (len(primpoints(0, i@primnum)) != 1) return;
if (string(primintrinsic(0, "typename", i@primnum)) == "Poly") return;
int class = prim(1, "class", i@primnum);
if (findattribvalcount(1, "prim", "class", class) > 1) return;
i@group___packed = 1;
"""

# Primitives: an unpacked primitive without a name takes its point's.
PRIM_NAME_FROM_POINT_VEX = """\
string name = prim(0, "name", i@primnum);
if (name != "" || !haspointattrib(0, "name")) return;
name = point(0, "name", i@ptnum);
if (name != "") setprimattrib(0, "name", i@primnum, name);
"""

# Points of packed pieces: the point's name, else the primitive's, else <prefix><ptnum>.
POINT_NAME_VEX = """\
if (s@name != "") return;
if (hasprimattrib(0, "name")) s@name = prim(0, "name", i@primnum);
if (s@name == "") s@name = chs("prefix") + itoa(i@ptnum);
"""

# Names mode, points of packed pieces: as POINT_NAME_VEX, written where the name already lives.
SET_NAME_VEX = """\
string name = point(0, "name", i@ptnum);
if (hasprimattrib(0, "name") && name == "")
    name = prim(0, "name", i@primnum);
if (name == "") name = chs("prefix") + itoa(i@ptnum);
if (hasprimattrib(0, "name") || !haspointattrib(0, "name"))
    setprimattrib(0, "name", i@primnum, name);
else
    setpointattrib(0, "name", i@ptnum, name);
"""

# Collision mode, packed colliders: Bullet's set_name_if_needed3 (the point's name wins when it has
# one, else the primitive's, else <prefix><ptnum>). The primitive's name is kept unless it is the
# only one (Bullet's switchif6 promotes it to the point then: delete_prim_name_if_only).
COLLISION_NAME_VEX = """\
string name = "";
if (hasprimattrib(0, "name"))
    name = prim(0, "name", i@primnum);
if (haspointattrib(0, "name") || name == "")
    name = point(0, "name", i@ptnum);
if (name == "") name = chs("prefix") + itoa(i@ptnum);
s@name = name;
"""
COLLISION_PREFIX = "static"  # Bullet's Output Prefix for assembled colliders

# Bullet's `collisiongroup` / `collisionignore` as Box3D's `collision_categorybits` / `collision_maskbits`.
# Input 1 is the other geometry of the simulation (pieces <-> colliders), so both sides number the
# same group names the same way. Bit 0 is "no group"; groups take bits 1 to 30 in sorted order (more
# share bit 30), so the values fit a 32-bit int. A piece ignores a group when the pattern matches
# its name (space separated, `^` takes names out again, as Houdini patterns). Box3D collides two
# bodies only when each one's mask has the other's category, but Bullet collides them when either
# one's pattern leaves the other's group alone (SIM_BulletRigidBody::checkCollideWithOverride): only
# both ignoring each other stops them. So a piece's mask is the groups its own pattern leaves alone, and
# those of every group with a piece that leaves its group alone (`__allows`, below).
# Points with neither attribute keep whatever they had (the solver's defaults when missing).
COLLISION_IGNORED_FN = """\
int bit(int k) { return (int)pow(2.0, (float)k); }
// The bits of the groups in `used` that `ignore` matches.
int ignoredBits(string ignore; string used[])
{
    string patterns[] = split(ignore);
    int ignored = 0;
    for (int i = 0; i < len(used); i++) {
        int hit = 0;
        foreach (string p; patterns) {
            if (startswith(p, "^")) {
                if (match(p[1:], used[i])) hit = 0;
            } else if (match(p, used[i])) {
                hit = 1;
            }
        }
        if (hit) ignored |= bit(min(i + 1, 30));
    }
    return ignored;
}
"""
# The group names of both geometries, once, sorted, without the empty one: a detail attribute `__used` (Attribute
# Promote finds each geometry's unique names, as `__groups`). And `__allows`: for the "no group" slot 0 and each
# group k + 1, the bits of the groups that some piece of it leaves alone (every bit for a piece with neither attribute).
COLLISION_USED_VEX = COLLISION_IGNORED_FN + """\
string used[], first[] = detail(0, "__groups"), second[] = detail(1, "__groups");
foreach (string n; first)
    if (n != "" && find(used, n) < 0) append(used, n);
foreach (string n; second)
    if (n != "" && find(used, n) < 0) append(used, n);
string names[] = sort(used);
s[]@__used = names;
int allows[];
resize(allows, len(names) + 1);
dict seen = {};
for (int src = 0; src < 2; src++) {
    int hasGroup = haspointattrib(src, "collisiongroup");
    int hasIgnore = haspointattrib(src, "collisionignore");
    int count = npoints(src);
    if (!hasGroup && !hasIgnore) {
        if (count > 0) allows[0] |= -1;
        continue;
    }
    for (int p = 0; p < count; p++) {
        string group = hasGroup ? point(src, "collisiongroup", p) : "";
        string ignore = hasIgnore ? point(src, "collisionignore", p) : "";
        int slot = 0;
        if (group != "") {
            int index = find(names, group);
            slot = index >= 0 ? index + 1 : 0;
        }
        int bits = -1;
        if (group != "" || ignore != "") {
            if (isvalidindex(seen, ignore)) {
                bits = seen[ignore];
            } else {
                bits = ~ignoredBits(ignore, names);
                seen[ignore] = bits;
            }
        }
        allows[slot] |= bits;
    }
}
i[]@__allows = allows;
"""
# Per point (the node is skipped when the pieces have neither attribute; collision_categorybits / collision_maskbits are
# there, 1 and -1 where they were missing, so a point with neither attribute keeps what it had).
COLLISION_NAMES_VEX = COLLISION_IGNORED_FN + """\
int hasGroup = haspointattrib(0, "collisiongroup");
int hasIgnore = haspointattrib(0, "collisionignore");
string group = hasGroup ? point(0, "collisiongroup", i@ptnum) : "";
string ignore = hasIgnore ? point(0, "collisionignore", i@ptnum) : "";
if (group == "" && ignore == "") return;
string used[] = detail(1, "__used");
int allows[] = detail(1, "__allows");
int index = find(used, group);
int category = (group != "" && index >= 0) ? bit(min(index + 1, 30)) : 1;
int mask = ~ignoredBits(ignore, used);
for (int k = 0; k < len(allows); k++)
    if (allows[k] & category) mask |= bit(min(k, 30));
setpointattrib(0, "collision_categorybits", i@ptnum, category);
setpointattrib(0, "collision_maskbits", i@ptnum, mask);
"""
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


def collision_names(parent, name, src, other, warn):
    """A subnet `name` in `parent` that maps `src`'s collisiongroup / collisionignore onto collision_categorybits /
    collision_maskbits (COLLISION_NAMES_VEX); `other` is the simulation's other geometry. Each geometry's group names
    come from an Attribute Promote (unique), `__used` joins them once, and a point wrangle works out each point's
    bits. The subnet passes `src` through when it has neither attribute. Returns it."""
    net = parent.createNode("subnet", name)
    net.setInput(0, src)
    net.setInput(1, other)
    for child in net.children():
        child.destroy()
    inputs = net.indirectInputs()

    def groups(source, label):
        """`source`'s unique collisiongroup names as the detail attribute `__groups` (an empty one where the
        attribute is missing)."""
        have = b3d_hda.attribute_create(net, f"has_group_{label}", [("collisiongroup", "string", None)], warn,
                                        write=False)
        have.setInput(0, source)
        promote = net.createNode("attribpromote", f"unique_groups_{label}")
        promote.setInput(0, have)
        b3d_hda.set_token(promote.parm("inclass"), "point", warn)
        b3d_hda.set_token(promote.parm("outclass"), "detail", warn)
        b3d_hda.set_token(promote.parm("method"), "unique", warn)
        promote.parm("inname").set("collisiongroup")
        promote.parm("useoutname").set(1)
        promote.parm("outname").set("__groups")
        promote.parm("deletein").set(0)
        return promote

    used = net.createNode("attribwrangle", "used_groups")
    used.setInput(0, groups(inputs[0], "source"))
    used.setInput(1, groups(inputs[1], "other"))
    used.parm("class").set("detail")
    used.parm("snippet").set(COLLISION_USED_VEX)
    bits = b3d_hda.attribute_create(net, "make_bits", [("collision_categorybits", "int", None),
                                                       ("collision_maskbits", "int", None)], warn, write=False)
    bits.setInput(0, inputs[0])
    for k, default in (("1", 1), ("2", -1)):
        bits.parm(f"default{k}v1").set(default)
    names = net.createNode("attribwrangle", "set_bits")
    names.setInput(0, bits)
    names.setInput(1, used)
    names.parm("class").set("point")
    names.parm("snippet").set(COLLISION_NAMES_VEX)
    switch = net.createNode("switch", "switch_names")
    switch.setInput(0, inputs[0])
    switch.setInput(1, names)
    switch.parm("input").setExpression(
        'haspointattrib(opinputpath(".", 0), "collisiongroup") || haspointattrib(opinputpath(".", 0), "collisionignore")',
        hou.exprLanguage.Hscript)
    out = net.createNode("output", "output0")
    out.setInput(0, switch)
    out.setDisplayFlag(True)
    out.setRenderFlag(True)
    tidy(net)
    return net


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
    tidy(net)
    return net
