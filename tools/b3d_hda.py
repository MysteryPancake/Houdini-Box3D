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
    for t in [type_names] if isinstance(type_names, str) else type_names:
        try:
            return parent.createNode(t, name)
        except hou.OperationFailed:
            continue
    raise RuntimeError(f"none of the node types {type_names} exist in {parent.path()}")


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
