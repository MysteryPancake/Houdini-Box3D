"""Helpers shared by the digital asset builders (make_packed_object.py, make_configure_sop.py,
make_solver_sop.py).

Not a build script: imported by the builders (run with hython, so this folder is on sys.path).
"""

import hou


# Templates without a value: nothing to reference.
NO_VALUE = {hou.parmTemplateType.Folder, hou.parmTemplateType.FolderSet,
            hou.parmTemplateType.Separator, hou.parmTemplateType.Label,
            hou.parmTemplateType.Button}


def value_templates(entries):
    """Every template with a value under `entries`, recursing into folders."""
    out = []
    for pt in entries:
        if pt.type() == hou.parmTemplateType.Folder:
            out.extend(value_templates(pt.parmTemplates()))
        elif pt.type() not in NO_VALUE:
            out.append(pt)
    return out


def without_parmops(entries):
    """`entries` without the DOP Parameter Operation menus (parmop_<name>), recursing into folders."""
    out = []
    for pt in entries:
        if pt.name().startswith("parmop"):
            continue
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            pt.setParmTemplates(without_parmops(pt.parmTemplates()))
        out.append(pt)
    return out


def labels(node):
    """`node`'s parameters as "Label (name)", for the message when one is not found."""
    return ", ".join(sorted({f"{pt.label()} ({pt.name()})" for pt in node.parmTemplateGroup().entriesWithoutFolders()
                             if not pt.name().startswith("parmop") and pt.type() not in NO_VALUE}))


def warner(tool):
    """A warn(msg) function that prints `msg` as a warning from `tool` (a build never fails over
    an asset: problems are warnings)."""

    def warn(msg):
        print(f"{tool}: WARNING: {msg}")

    return warn


def create(parent, type_names, name):
    """Creates a node of the first of `type_names` (one name, or a list to try in order: the node
    type differs between Houdini versions) that exists in `parent`."""
    last = None
    for t in [type_names] if isinstance(type_names, str) else type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed as err:
            last = err
    raise RuntimeError(f"none of the node types {type_names} exist in {parent.path()}: {last}")


def set_operation(node, name, token, warn):
    """Sets the Parameter Operation menu of `node`'s parameter `name` (parmop_<name>) to `token`, as
    set_operations does for all of them. False (with a warning) when it cannot."""
    p = node.parm(f"parmop_{name}")
    if p is None:
        warn(f"{node.path()} has no parmop_{name}")
        return False
    items = list(p.menuItems()) if p.parmTemplate().type() == hou.parmTemplateType.Menu else []
    try:
        if p.parmTemplate().type() == hou.parmTemplateType.String:
            p.set(token)
        elif token in items:
            p.set(items.index(token))
        else:
            warn(f"{p.path()}: no {token!r} item in {items}")
            return False
    except hou.Error as e:
        warn(f"{p.path()}: {e}")
        return False
    return True


def set_operations(node, token, warn):
    """Sets every Parameter Operation menu (parmop_*) of `node` to the item `token` (default /
    initial / always / never) and returns how many it set; each one it cannot set is a warning.

    A new DOP data node starts at Use Default and a channel reference does not switch it to Set
    Initial, so without this the node keeps its built-in defaults whatever the referenced values."""
    changed = 0
    for p in node.parms():
        if not p.name().startswith("parmop"):
            continue
        items = list(p.menuItems()) if p.parmTemplate().type() == hou.parmTemplateType.Menu else []
        try:
            if p.parmTemplate().type() == hou.parmTemplateType.String:
                p.set(token)
            elif token in items:
                p.set(items.index(token))
            else:
                warn(f"{p.path()}: no {token!r} item in {items}")
                continue
            changed += 1
        except hou.Error as e:
            warn(f"{p.path()}: {e}")
    return changed


def menu_index_expr(parm, cond, on_label, off_label, warn):
    """Drives the menu `parm` with an HScript expression: the item labelled like `on_label` while
    `cond` is true, else the one like `off_label`. False if either item is missing."""
    found = {}
    for want in (on_label, off_label):
        for i, lab in enumerate(parm.menuLabels()):
            if want.lower() in lab.lower():
                found[want] = i
                break
    if len(found) != 2:
        warn(f"{parm.path()}: no items like {on_label!r} / {off_label!r} in {list(parm.menuLabels())}")
        return False
    if parm.parmTemplate().type() == hou.parmTemplateType.String:
        items = list(parm.menuItems())
        expr = f'ifs({cond}, "{items[found[on_label]]}", "{items[found[off_label]]}")'
    else:
        expr = f"if({cond}, {found[on_label]}, {found[off_label]})"
    parm.setExpression(expr, hou.exprLanguage.Hscript)
    return True


TAB_SUBMENU = "Box3D"  # the Tab menu folder every Box3D asset is listed under

# The asset's Tools.shelf section, as SideFX's assets: the Tab menu tool for the asset's network type, in
# TAB_SUBMENU. `{ctx}` is DOP or SOP, `{utils}` doptoolutils or soptoolutils.
TOOLS_SHELF = """<?xml version="1.0" encoding="UTF-8"?>
<shelfDocument>
  <tool name="$HDA_DEFAULT_TOOL" label="$HDA_LABEL" icon="$HDA_ICON">
    <toolMenuContext name="viewer">
      <contextNetType>{ctx}</contextNetType>
    </toolMenuContext>
    <toolMenuContext name="network">
      <contextOpType>$HDA_TABLE_AND_NAME</contextOpType>
    </toolMenuContext>
    <toolSubmenu>{submenu}</toolSubmenu>
    <script scriptType="python"><![CDATA[import {utils}

{utils}.genericTool(kwargs, '$HDA_NAME')]]></script>
  </tool>
</shelfDocument>
"""


def set_tab_submenu(definition, warn):
    """Lists the asset under TAB_SUBMENU in the Tab menu (its Tools.shelf section)."""
    ctx = definition.nodeTypeCategory().name().upper()  # "Dop" / "Sop"
    utils = {"DOP": "doptoolutils", "SOP": "soptoolutils"}.get(ctx)
    if utils is None:
        warn(f"{definition.nodeTypeName()}: no Tab menu tool for a {ctx} asset")
        return
    definition.addSection("Tools.shelf", TOOLS_SHELF.format(ctx=ctx, utils=utils, submenu=TAB_SUBMENU))


def pull(entries, pred):
    """(`entries` without the templates `pred` picks, recursing into folders; the templates picked)."""
    kept, picked = [], []
    for pt in entries:
        if pred(pt):
            picked.append(pt)
            continue
        if pt.type() == hou.parmTemplateType.Folder:
            pt = pt.clone()
            inner, more = pull(pt.parmTemplates(), pred)
            pt.setParmTemplates(inner)
            picked += more
        kept.append(pt)
    return kept, picked


def collapsible(name, label, contents, open_folder=True, header_toggle=None):
    """A collapsible folder with RBD Bullet Solver's tags: group_type collapsible, group_default 1 when
    `open_folder`, and `header_toggle` (a toggle's name) as its header checkbox."""
    tags = {"group_type": "collapsible"}
    if open_folder:
        tags["group_default"] = "1"
    if header_toggle:
        tags["sidefx::header_toggle"] = header_toggle
    return hou.FolderParmTemplate(name, label, contents, folder_type=hou.folderType.Collapsible, tags=tags)


def tidy(net, row_gap=1.1, margin=0.5):
    """Lays out the nodes of `net` top to bottom in rows, instead of `layoutChildren`.

    A node sits one row below its lowest input (an input that feeds nothing but a deep node is pulled
    down next to it). A wire that skips rows gets a hidden stop in each row it crosses, so rows are
    ordered to cut crossings and nothing is drawn through a node. Each node is centred under its
    inputs, with room for the name drawn to its right. Nodes with no wires go in a column to the
    right. The subnet's own inputs, if any, are placed with the other nodes."""
    items = list(net.children())
    try:
        items = list(net.indirectInputs()) + items
    except (AttributeError, hou.Error):
        pass
    if not items:
        return
    count = len(items)
    index = {it.path(): i for i, it in enumerate(items)}

    parents = [[] for _ in items]
    for i, it in enumerate(items):
        if not hasattr(it, "inputConnections"):
            continue
        for conn in it.inputConnections():
            src = conn.inputItem() if hasattr(conn, "inputItem") else conn.inputNode()
            j = index.get(src.path()) if src is not None else None
            if j is not None and j != i and j not in parents[i]:
                parents[i].append(j)
    children = [[] for _ in items]
    for i, ps in enumerate(parents):
        for j in ps:
            children[j].append(i)

    # Rows: longest path from a source, then sources pulled down to just above their first user.
    layer = [None] * count

    def depth(i, active):
        if layer[i] is not None:
            return layer[i]
        if i in active:
            return 0  # a loop: ignore the wire
        active.add(i)
        layer[i] = 1 + max((depth(j, active) for j in parents[i]), default=-1)
        active.discard(i)
        return layer[i]

    for i in range(count):
        depth(i, set())
    for i in range(count):
        if not parents[i] and children[i]:
            layer[i] = min(layer[c] for c in children[i]) - 1
    low = min(layer)
    layer = [row - low for row in layer]

    loose = [i for i in range(count) if not parents[i] and not children[i]]
    main = [i for i in range(count) if i not in loose]

    # Hidden stops for wires that skip rows (ids from `count` up).
    for c in list(main):
        for k, p in enumerate(list(parents[c])):
            if layer[c] - layer[p] <= 1:
                continue
            children[p].remove(c)
            previous = p
            for row in range(layer[p] + 1, layer[c]):
                stop = len(layer)
                layer.append(row)
                parents.append([previous])
                children.append([])
                children[previous].append(stop)
                previous = stop
            children[previous].append(c)
            parents[c][k] = previous
    everyone = main + list(range(count, len(layer)))
    rows = {}
    for i in everyone:
        rows.setdefault(layer[i], []).append(i)
    row_ids = sorted(rows)

    order = {}

    def renumber(row):
        for k, i in enumerate(rows[row]):
            order[i] = k

    for row in row_ids:
        renumber(row)

    def sweep(rows_in_order, links):
        for row in rows_in_order:
            def bary(i):
                near = [order[j] for j in links[i]]
                return sum(near) / len(near) if near else order[i]
            rows[row].sort(key=bary)
            renumber(row)

    def crossings(upper):
        """Wire crossings between row `upper` and the row below it."""
        wires = [(order[p], order[c]) for c in rows.get(upper + 1, []) for p in parents[c]]
        return sum(1 for a in range(len(wires)) for b in range(a + 1, len(wires))
                   if (wires[a][0] - wires[b][0]) * (wires[a][1] - wires[b][1]) < 0)

    def swap_pass():
        for row in row_ids:
            members = rows[row]
            for k in range(len(members) - 1):
                before = crossings(row - 1) + crossings(row)
                members[k], members[k + 1] = members[k + 1], members[k]
                renumber(row)
                if crossings(row - 1) + crossings(row) >= before:
                    members[k], members[k + 1] = members[k + 1], members[k]
                    renumber(row)

    for _ in range(8):
        sweep(row_ids[1:], parents)
        sweep(reversed(row_ids[:-1]), children)
    for _ in range(4):
        swap_pass()

    # Columns: packed with room for each name, then pulled under the inputs and over the users.
    def size(i):
        if i >= count:
            return 0.6
        try:
            w = items[i].size()[0]
        except (AttributeError, hou.Error):
            w = 1.0
        return w + 0.5 + 0.11 * len(items[i].name())

    centre = {}
    for row in row_ids:
        x = 0.0
        for i in rows[row]:
            centre[i] = x + size(i) / 2
            x += size(i)
        for i in rows[row]:
            centre[i] -= x / 2

    def place(row, links):
        members = rows[row]
        want = []
        for i in members:
            near = [centre[j] for j in links[i]]
            want.append(sum(near) / len(near) if near else centre[i])
        pos = []
        for k, i in enumerate(members):
            lo = pos[-1] + (size(members[k - 1]) + size(i)) / 2 if k else want[k]
            pos.append(max(want[k], lo))
        shift = sum(p - w for p, w in zip(pos, want)) / len(pos)
        for i, p in zip(members, pos):
            centre[i] = p - shift

    for _ in range(6):
        for row in row_ids[1:]:
            place(row, parents)
        for row in reversed(row_ids[:-1]):
            place(row, children)

    left = min((centre[i] - size(i) / 2 for i in everyone), default=0.0)
    for i in main:
        w = items[i].size()[0] if hasattr(items[i], "size") else 1.0
        items[i].setPosition(hou.Vector2(centre[i] - left - w / 2, -layer[i] * row_gap))
    right = max((centre[i] - left + size(i) / 2 for i in everyone), default=0.0) + margin
    for k, i in enumerate(loose):
        items[i].setPosition(hou.Vector2(right, -k * row_gap))
