"""Makes a test's saved scene easy to inspect by hand (run_tests.py calls tidy_for_review just before
saving tests/houdini/out/<test>.hip):

* every network is laid out (as pressing L in the network editor), so no nodes overlap;
* geometry objects that only feed a simulation are hidden, so the viewport shows the simulations,
  and the visible objects (DOP networks, and geometry that reads a simulation back, such as a DOP
  Import) are spaced out left to right along +X so they do not overlap;
* a sticky note at /obj says what the test checks and expects (its docstring), what this run gave,
  and which network is which; each DOP network gets a short note saying where it sits.

Only object-level translations change, and a DOP network moves all of its objects alike, so the
simulations behave the same. Nothing here may change a result: run_tests.py ignores any error.
"""

import inspect
import math
import textwrap

import hou

GAP = 4.0  # viewport units between neighbours
MAX_WIDTH = 200.0  # a body that flew off must not push everything else far away
NOTE_WIDTH = 90  # characters per line in the sticky notes


def place_along_x(extents, gap=GAP):
    """For [(xmin, xmax)] in order, the x shift that puts each one gap after the previous one,
    the first starting at 0. Empty or non-finite extents count as width 1 at 0."""
    shifts, cursor = [], 0.0
    for lo, hi in extents:
        if not (math.isfinite(lo) and math.isfinite(hi)) or hi < lo:
            lo, hi = 0.0, 1.0
        if hi - lo > MAX_WIDTH:
            hi = lo + MAX_WIDTH
        shifts.append(cursor - lo)
        cursor += (hi - lo) + gap
    return shifts


def _ordinal(i):
    return {1: "1st", 2: "2nd", 3: "3rd"}.get(i, f"{i}th")


def _x_extent(points):
    xs = [p[0] for p in points if all(math.isfinite(c) for c in p)]
    return (min(xs), max(xs)) if xs else (math.nan, math.nan)


def _box_corners(bbox, xform):
    lo, hi = bbox.minvec(), bbox.maxvec()
    corners = []
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (lo[2], hi[2]):
                corners.append(hou.Vector3(x, y, z) * xform)
    return corners


def _dopnet_extent(dopnet):
    """x range of everything the DOP network shows on the current frame."""
    points = []
    try:
        objects = dopnet.simulation().objects()
    except hou.Error:
        objects = []
    for obj in objects:
        try:
            geo = obj.geometry()
            if geo is None or not geo.points():
                continue
            points += _box_corners(geo.boundingBox(), obj.transform())
        except (hou.Error, AttributeError):
            continue
    return _x_extent(points)


def _geo_extent(node):
    try:
        geo = node.displayNode().geometry()
        if geo is None or not geo.points():
            return (math.nan, math.nan)
        return _x_extent(_box_corners(geo.boundingBox(), node.worldTransform()))
    except (hou.Error, AttributeError):
        return (math.nan, math.nan)


def _top_object(node):
    """The /obj-level object a node is inside (or is), else None."""
    while node is not None and node.parent() is not None:
        if node.parent().path() == "/obj":
            return node
        node = node.parent()
    return None


def _reads_simulation(node, dopnets):
    """True if anything inside this object reads one of the DOP networks (a DOP Import etc.)."""
    for n in node.allSubChildren():
        if n.type().name().startswith("dopimport"):
            return True
        for p in n.parms():
            try:
                if p.parmTemplate().type() != hou.parmTemplateType.String:
                    continue
                target = p.evalAsNode()
            except (hou.Error, AttributeError, TypeError):
                continue
            if target is not None and _top_object(target) in dopnets:
                return True
    return False


def _layout_all(net):
    """Lays out net and every editable network inside it."""
    try:
        if hasattr(net, "isLockedHDA") and net.isLockedHDA():
            return
        if net.children():
            net.layoutChildren()
    except hou.Error:
        return
    for child in net.children():
        try:
            if child.isNetwork():
                _layout_all(child)
        except (hou.Error, AttributeError):
            continue


def _shift_x(node, dx):
    """Moves an object along x unless its translation is animated. False if it could not."""
    p = node.parm("tx")
    if p is None:
        return False
    try:
        if p.keyframes():
            return False
        p.set(p.eval() + dx)
        return True
    except hou.Error:
        return False


def _wrap(text):
    lines = []
    for para in text.split("\n"):
        lines += textwrap.wrap(para, NOTE_WIDTH) or [""]
    return lines


def _sticky(net, name, lines, above=True):
    """A sticky note in net holding lines, placed above (or left of) its laid-out nodes."""
    note = net.createStickyNote(name)
    note.setText("\n".join(lines))
    width = max(6.0, min(18.0, 0.11 * max((len(s) for s in lines), default=20)))
    height = max(1.5, 0.22 * len(lines) + 0.6)
    note.setSize(hou.Vector2(width, height))
    nodes = [n for n in net.children()]
    if nodes:
        xs = [n.position()[0] for n in nodes]
        ys = [n.position()[1] + n.size()[1] for n in nodes]
        if above:
            note.setPosition(hou.Vector2(min(xs), max(ys) + 1.0))
        else:
            note.setPosition(hou.Vector2(min(xs) - width - 1.0, max(ys) - height))
    return note


def tidy_for_review(test_name, test_fn, results):
    """Lays out, spaces and annotates the current scene. results: [(status, message)]."""
    obj = hou.node("/obj")
    tops = sorted(obj.children(), key=lambda n: n.sessionId())
    dopnets = [n for n in tops if n.type().name() == "dopnet"]

    # Inputs off, simulations (and views of them) on.
    shown = []
    for n in tops:
        if n in dopnets:
            shown.append(n)
            continue
        try:
            if n.type().category().name() != "Object" or not hasattr(n, "setDisplayFlag"):
                continue
            if _reads_simulation(n, dopnets):
                shown.append(n)
            else:
                n.setDisplayFlag(False)
        except (hou.Error, AttributeError):
            continue

    # Side by side along +X, in the order the test made them.
    extents = [_dopnet_extent(n) if n in dopnets else _geo_extent(n) for n in shown]
    placed = []
    for n, dx, (lo, hi) in zip(shown, place_along_x(extents), extents):
        moved = _shift_x(n, dx)
        left = (lo + dx) if (moved and math.isfinite(lo)) else None
        placed.append((n, moved, left))

    _layout_all(obj)

    # Notes.
    doc = inspect.cleandoc(test_fn.__doc__ or "(no description)")
    where = []
    for i, (n, moved, left) in enumerate(placed, 1):
        pos = f"starts at x = {left:.1f}" if left is not None else "not moved"
        where.append(f"  {_ordinal(i)} from the left: {n.name()} ({pos})")
    hidden = [n.name() for n in tops if n not in shown and hasattr(n, "isDisplayFlagSet")]
    lines = [f"TEST: {test_name}", ""] + _wrap(doc) + ["", "THIS RUN:"]
    for status, message in results:
        lines += _wrap(f"{status}  {message}")
    lines += ["", "IN THE VIEWPORT (left to right along +X, play from frame 1):"] + where
    if hidden:
        lines += _wrap("Hidden (they only feed the simulations; turn their display flag on to see the "
                       "source geometry): " + ", ".join(hidden))
    _sticky(obj, "about_this_test", lines, above=False)

    for i, (n, moved, left) in enumerate(placed, 1):
        if n not in dopnets:
            continue
        pos = f", starting at x = {left:.1f}" if left is not None else ""
        note = [f"Part of test '{test_name}': the {_ordinal(i)} network from the left in the viewport{pos}.",
                "What the test checks and expects: see the note at /obj."]
        try:
            _sticky(n, "about_this_network", sum((_wrap(s) for s in note), []))
        except hou.Error:
            continue
