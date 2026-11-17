"""Helpers shared by the digital asset builders (make_packed_object.py, make_configure_sop.py,
make_solver_sop.py).

Not a build script: imported by the builders (run with hython, so this folder is on sys.path).
"""

import os
import re

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


def edit_dialog_script(definition, input_labels, output_labels, warn, handles=()):
    """Input / output labels, and `handles` (parameter, handle type, handle name, handle parameter) bindings, in the
    DialogScript (hou has no setter for them)."""
    section = definition.sections().get("DialogScript")
    if section is None:
        warn("the asset has no DialogScript section: input / output labels not set")
        return
    lines = [ln for ln in section.contents().splitlines()
             if not re.match(r"\s*(inputlabel|outputlabel|bindhandle)\b", ln)]
    extra = [f'    inputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(input_labels)]
    extra += [f'    outputlabel\t{i + 1}\t"{lab}"' for i, lab in enumerate(output_labels)]
    extra += [f'    bindhandle {parm} {kind} "{name}" {hparm} ""' for parm, kind, name, hparm in handles]
    at = next((i + 1 for i, ln in enumerate(lines) if re.match(r"\s*label\b", ln)), 1)
    section.setContents("\n".join(lines[:at] + extra + lines[at:]) + "\n")


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


def set_token(p, token, warn):
    """Sets the menu parameter `p` to its item `token`. False (with a warning) when it has no such item."""
    try:
        if p.parmTemplate().type() == hou.parmTemplateType.String:
            p.set(token)
            return True
        items = list(p.menuItems())
        if token in items:
            p.set(items.index(token))
            return True
        warn(f"{p.path()}: no {token!r} item in {items}")
    except hou.Error as e:
        warn(f"{p.path()}: {e}")
    return False


def set_operation(node, name, token, warn):
    """Sets the Parameter Operation menu of `node`'s parameter `name` (parmop_<name>) to `token`, as
    set_operations does for all of them. False (with a warning) when it cannot."""
    p = node.parm(f"parmop_{name}")
    if p is None:
        warn(f"{node.path()} has no parmop_{name}")
        return False
    return set_token(p, token, warn)


def set_operations(node, token, warn):
    """Sets every Parameter Operation menu (parmop_*) of `node` to the item `token` (default /
    initial / always / never) and returns how many it set; each one it cannot set is a warning.

    A new DOP data node starts at Use Default and a channel reference does not switch it to Set
    Initial, so without this the node keeps its built-in defaults whatever the referenced values."""
    return sum(1 for p in node.parms() if p.name().startswith("parmop") and set_token(p, token, warn))


def menu_token_expr(parm, cond, on_token, off_token, warn):
    """Drives the menu `parm` with an HScript expression: the item `on_token` while `cond` is true, else
    `off_token`. False if either item is missing."""
    items = list(parm.menuItems())
    if on_token not in items or off_token not in items:
        warn(f"{parm.path()}: no items {on_token!r} / {off_token!r} in {items}")
        return False
    if parm.parmTemplate().type() == hou.parmTemplateType.String:
        expr = f'ifs({cond}, "{on_token}", "{off_token}")'
    else:
        expr = f"if({cond}, {items.index(on_token)}, {items.index(off_token)})"
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


def add_python_module(definition, extra=""):
    """The asset's Python Module: isVellumNode() true, which makes the viewer connect a new node's inputs
    automatically (as RBD's SOPs do), plus `extra` source."""
    code = "def isVellumNode():\n    return True\n"
    if extra:
        code += "\n" + extra
    definition.addSection("PythonModule", code)
    definition.setExtraFileOption("PythonModule/IsPython", True)
    definition.setExtraFileOption("PythonModule/IsScript", True)


def show_in_toolbar(*templates):
    """Parameters shown in the operator toolbar across the top of the viewer while the node is active (Type
    Properties' Show parm in > Main & Tool Dialogs + Toolbox, "export all" in the DialogScript). hou has no
    public setter for the export level."""
    for pt in templates:
        pt._setExportLevel(3)


def add_viewer_state(definition, state_name, source_file):
    """An embedded Python viewer state: `source_file` (beside the scripts) as the ViewerStateModule, `state_name`
    as the Node tab's default state, and the sections Houdini keeps for it."""
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), source_file), encoding="utf-8") as f:
        module = f.read()
    sections = {
        "ViewerStateModule": module,
        "DefaultState": state_name,
        "ViewerStateName.orig": state_name,  # what the uninstall script falls back to
        "ViewerStateInstall": "__import__('viewerstate.utils', fromlist=[None]).register_pystate_embedded(kwargs['type'])",
        "ViewerStateUninstall": "__import__('viewerstate.utils', fromlist=[None]).unregister_pystate_embedded(kwargs['type'])",
    }
    for name, contents in sections.items():
        definition.addSection(name, contents)
    for name in ("ViewerStateModule", "ViewerStateInstall", "ViewerStateUninstall"):
        definition.setExtraFileOption(f"{name}/IsPython", True)
        definition.setExtraFileOption(f"{name}/IsViewerState", True)
        definition.setExtraFileOption(f"{name}/IsScript", True)


def set_tab_submenu(definition, warn):
    """Lists the asset under TAB_SUBMENU in the Tab menu (its Tools.shelf section)."""
    ctx = definition.nodeTypeCategory().name().upper()  # "Dop" / "Sop"
    utils = {"DOP": "doptoolutils", "SOP": "soptoolutils"}.get(ctx)
    if utils is None:
        warn(f"{definition.nodeTypeName()}: no Tab menu tool for a {ctx} asset")
        return
    definition.addSection("Tools.shelf", TOOLS_SHELF.format(ctx=ctx, utils=utils, submenu=TAB_SUBMENU))


def set_message_nodes(asset, paths, warn):
    """The asset's Message Nodes (Node tab of its Type Properties): the nodes inside, as paths relative to
    `asset`, whose errors and warnings show on the asset itself. Without them a message from inside only
    shows after diving in. A path with no node is a warning and left out."""
    found = []
    for path in paths:
        if asset.node(path) is None:
            warn(f"{asset.type().name()}: no node {path} for Message Nodes")
        else:
            found.append(path)
    if found:
        asset.type().definition().addSection("MessageNodes", " ".join(found))


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


def tidy(net, row_gap=1.1, margin=0.5, items=None, apply=True):
    """Lays out the nodes of `net` top to bottom in rows, instead of `layoutChildren`.

    A node sits one row below its lowest input (an input that feeds nothing but a deep node is pulled
    down next to it). A wire that skips rows gets a hidden stop in each row it crosses, so rows are
    ordered to cut crossings and nothing is drawn through a node. Each node is centred under its
    inputs, with room for the name drawn to its right. Nodes with no wires go in a column to the
    right. The subnet's own inputs, if any, are placed with the other nodes."""
    if items is None:
        items = list(net.children())
        try:
            items = list(net.indirectInputs()) + items
        except (AttributeError, hou.Error):
            pass
    if not items:
        return {}
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
    placed = {}
    for i in main:
        w = items[i].size()[0] if hasattr(items[i], "size") else 1.0
        placed[i] = (centre[i] - left - w / 2, -layer[i] * row_gap)
    right = max((centre[i] - left + size(i) / 2 for i in everyone), default=0.0) + margin
    for k, i in enumerate(loose):
        placed[i] = (right if main else 0.0, -k * row_gap)
    if apply:
        for i, (x, y) in placed.items():
            items[i].setPosition(hou.Vector2(x, y))
    return {items[i].path(): (x, y) for i, (x, y) in placed.items()}


def layout(net, rows, gap=2.0, row_gap=1.1, colors=None):
    """Lays out `net` by hand in labelled boxes. `rows` is a list of rows, each a list of boxes
    `(title, names)` or `(title, names, options)`, placed left to right, rows top to bottom (flow runs
    down the page). A name is a node's name, or `#0`, `#1` ... for a subnet's inputs. Inside a box the
    nodes are laid out by `tidy`; options: `grid` (columns) puts the nodes in a grid in the order given, `color`
    is an RGB tuple. A node named in no box goes in an extra box at the end, with a warning."""
    inputs = []
    try:
        inputs = list(net.indirectInputs())
    except (AttributeError, hou.Error):
        pass

    def find(name):
        if name.startswith("#"):
            k = int(name[1:])
            return inputs[k] if k < len(inputs) else None
        return net.node(name)

    listed = set()
    boxes = []
    for r, row in enumerate(rows):
        for spec in row:
            title, names = spec[0], spec[1]
            options = spec[2] if len(spec) > 2 else {}
            members = []
            for name in names:
                item = find(name)
                if item is None:
                    continue  # not in this build (a node type that does not exist here)
                members.append(item)
                listed.add(item.path())
            if members:
                boxes.append((r, title, members, options))
    rest = [c for c in net.children() if c.path() not in listed]
    if rest:
        print(f"b3d_hda.layout: {net.path()}: not in any box: {', '.join(c.name() for c in rest)}")
        boxes.append((len(rows), "Other", rest, {}))

    def label_width(item):
        return item.size()[0] + 0.5 + 0.11 * len(item.name())

    # Each box laid out on its own: (positions relative to its top-left, width, height).
    laid = []
    for r, title, members, options in boxes:
        columns = options.get("grid")
        if columns:
            cell = max(label_width(m) for m in members) + 0.4
            local = {m.path(): ((k % columns) * cell, -(k // columns) * row_gap) for k, m in enumerate(members)}
        else:
            local = tidy(net, row_gap=row_gap, items=members, apply=False)
        width = max(local[m.path()][0] + label_width(m) for m in members)
        height = -min(y for _, y in local.values()) + 0.5
        laid.append((r, title, members, options, local, width, height))

    by_row = {}
    for entry in laid:
        by_row.setdefault(entry[0], []).append(entry)
    pad = 0.8  # between a box's edge and its nodes
    y = 0.0
    for r in sorted(by_row):
        x = 0.0
        tallest = 0.0
        for _, title, members, options, local, width, height in by_row[r]:
            for m in members:
                px, py = local[m.path()]
                m.setPosition(hou.Vector2(x + pad + px, y - pad - 0.6 + py))
            box = net.createNetworkBox()
            box.setComment(title)
            color = options.get("color") or (colors or {}).get(title)
            if color:
                box.setColor(hou.Color(color))
            for m in members:
                try:
                    box.addItem(m)
                except hou.Error:
                    pass
            box.fitAroundContents()
            # Names are drawn right of their nodes and the title along the top: make the box wide enough.
            need = max(m.position()[0] + label_width(m) for m in members) + 0.4 - box.position()[0]
            need = max(need, 0.2 * len(title) + 1.5)
            if box.size()[0] < need:
                box.setSize(hou.Vector2(need, box.size()[1]))
            x += width + 2 * pad + gap
            tallest = max(tallest, height + 2 * pad + 0.6)
        y -= tallest + gap
