"""
Houdini Network Dumper
======================
Recursively dumps a node (e.g. the RBD Bullet Solver SOP) and everything inside it
to a single text file, so the network can be read and understood offline.

What gets written:
  * Root HDA info: library, version, full parameter interface (folders, defaults,
    hide/disable conditions, callbacks) and text sections (PythonModule, OnCreated, Help...)
  * A hierarchy tree of every node in the asset
  * Node type usage counts
  * For every network (subnet / dopnet / vopnet / HDA contents):
      - an ASCII picture of the node layout (top = top of network editor)
      - network boxes, sticky notes, dots, subnet inputs
      - the full wiring list (src[out] -> dst[in], with input labels)
      - for each node: type, flags, position, colour, comment, inputs/outputs,
        node references/dependents, and every NON-DEFAULT parameter
        (expressions, VEX snippets, Python code, ramps, channel refs, node-path refs)

How to run:
  1. Select the node to dump (e.g. an RBD Bullet Solver SOP) in the network editor.
     If nothing is selected you'll be offered a temporary rbdbulletsolver to dump.
  2. Paste this file into Windows > Python Source Editor and hit Apply,
     or put it on a shelf tool. A file dialog asks where to save the txt.
  3. In hython: import houdini_network_dump as d; d.main("/obj/geo1/rbdbulletsolver1", "/tmp/dump.txt")

By default it works on a temporary COPY of your node (unlocked, then deleted),
so your scene's HDA stays locked and untouched.
"""

import os
import datetime
import textwrap

import hou

# ----------------------------------------------------------------------------- config
FALLBACK_NODE_TYPE = "rbdbulletsolver"  # created temporarily if nothing is selected
WORK_ON_COPY = True                     # dump a temporary copy so your scene isn't modified
UNLOCK_HDAS = True                      # allow editing of contents (root + all nested HDAs)
MAX_DEPTH = 60                          # recursion limit into subnetworks
MAX_STRING_INLINE = 160                 # longer / multiline strings are written as blocks
DUMP_ROOT_HDA_SECTIONS = True           # PythonModule, OnCreated, Help, ...
DUMP_ROOT_INTERFACE = True              # parameter interface of the root HDA
DUMP_NESTED_HDA_SECTIONS = False        # also dump sections of every nested HDA type
MAX_SECTION_CHARS = 200000
ASCII_X_SCALE = 7.0                     # characters per network unit (horizontal)
ASCII_Y_SCALE = 1.0                     # text rows per network unit (vertical)
ASCII_LABEL_LEN = 22

SKIP_SECTIONS_PREFIX = ("Contents", "Icon", "InternalFileOptions", "ExtraFileOptions",
                        "TypePropertiesOptions", "DescriptiveParmName")
SKIP_SECTIONS_EXT = (".png", ".svg", ".jpg", ".jpeg", ".gif", ".pic", ".rat", ".bgeo",
                     ".bgeo.sc", ".hda", ".otl", ".gz", ".sc", ".bin")


# ----------------------------------------------------------------------------- helpers
class Dumper(object):
    def __init__(self, root, source_node, originally_locked):
        self.root = root
        self.source = source_node
        self.orig_locked = originally_locked
        self.lines = []
        self.nested_defs = {}

    # ---- output
    def w(self, s=""):
        self.lines.append(s)

    def block(self, text, indent="      | "):
        for ln in str(text).splitlines() or [""]:
            self.lines.append(indent + ln)

    def header(self, title, char="="):
        self.w("")
        self.w(char * 100)
        self.w(title)
        self.w(char * 100)

    # ---- naming
    def rel(self, item):
        """Path relative to the dumped root (absolute if outside it)."""
        if item is None:
            return "<none>"
        try:
            p = item.path()
        except Exception:
            return str(item)
        rp = self.root.path()
        if p == rp:
            return "<ROOT>"
        if p.startswith(rp + "/"):
            return p[len(rp) + 1:]
        return p

    def item_desc(self, item):
        if item is None:
            return "<none>"
        if isinstance(item, hou.SubnetIndirectInput):
            return "<subnet input #%d>" % (item.number() + 1)
        if isinstance(item, hou.NetworkDot):
            return "<dot %s>" % item.name()
        if isinstance(item, hou.Node):
            return item.name()
        return str(item)

    # ---- small formatters
    @staticmethod
    def fnum(v):
        if isinstance(v, float):
            s = ("%.6f" % v).rstrip("0").rstrip(".")
            return s if s not in ("", "-0") else "0"
        return str(v)

    @staticmethod
    def flags(n):
        out = []
        for label, fn in (("DISPLAY", "isDisplayFlagSet"), ("RENDER", "isRenderFlagSet"),
                          ("BYPASS", "isBypassed"), ("TEMPLATE", "isTemplateFlagSet"),
                          ("HARDLOCK", "isHardLocked"), ("SOFTLOCK", "isSoftLocked"),
                          ("UNLOAD", "isUnloadFlagSet")):
            f = getattr(n, fn, None)
            if f is None:
                continue
            try:
                if f():
                    out.append(label)
            except Exception:
                pass
        return out

    # ------------------------------------------------------------------ parameters
    def fmt_parm(self, p):
        t = p.parmTemplate()
        ttype = t.type()
        try:
            kfs = p.keyframes()
        except Exception:
            kfs = ()
        if kfs:
            try:
                lang = p.expressionLanguage().name()
            except Exception:
                lang = "?"
            try:
                expr = p.expression()
            except Exception:
                expr = "?"
            if len(kfs) == 1:
                return "EXPR(%s): %s" % (lang, expr)
            return "ANIMATED %d keys, first EXPR(%s): %s" % (len(kfs), lang, expr)

        if ttype == hou.parmTemplateType.Ramp:
            try:
                r = p.evalAsRamp()
                basis = [b.name() for b in r.basis()]
                keys = [self.fnum(k) for k in r.keys()]
                vals = r.values()
                if vals and isinstance(vals[0], tuple):
                    vals = ["(" + ",".join(self.fnum(c) for c in v) + ")" for v in vals]
                else:
                    vals = [self.fnum(v) for v in vals]
                return "RAMP basis=%s keys=[%s] values=[%s]" % (
                    basis, ", ".join(keys), ", ".join(vals))
            except Exception as e:
                return "RAMP <%s>" % e

        if ttype == hou.parmTemplateType.String:
            try:
                return p.unexpandedString()
            except Exception:
                try:
                    return p.evalAsString()
                except Exception as e:
                    return "<err %s>" % e

        if ttype == hou.parmTemplateType.Menu:
            try:
                return "%s (index %s)" % (p.evalAsString(), p.eval())
            except Exception:
                pass

        try:
            return self.fnum(p.eval())
        except Exception as e:
            return "<err %s>" % e

    def is_skippable_template(self, t):
        ttype = t.type()
        if ttype in (hou.parmTemplateType.FolderSet, hou.parmTemplateType.Separator,
                     hou.parmTemplateType.Label, hou.parmTemplateType.Button):
            return True
        if ttype == hou.parmTemplateType.Folder:
            try:
                ft = t.folderType()
                return ft not in (hou.folderType.MultiparmBlock,
                                  hou.folderType.ScrollingMultiparmBlock,
                                  hou.folderType.TabbedMultiparmBlock)
            except Exception:
                return True
        return False

    def dump_parms(self, node):
        written = 0
        for pt in node.parmTuples():
            try:
                t = pt.parmTemplate()
                if self.is_skippable_template(t):
                    continue
                # ramp point sub-parms are noise: the ramp line covers them
                try:
                    parent = pt[0].parentMultiParm()
                    if parent is not None and parent.parmTemplate().type() == hou.parmTemplateType.Ramp:
                        continue
                except Exception:
                    pass
                try:
                    at_default = pt.isAtDefault()
                except Exception:
                    at_default = False
                if at_default:
                    continue

                vals = [self.fmt_parm(p) for p in pt]
                extras = []
                # channel references to other parms
                for p in pt:
                    try:
                        rp = p.getReferencedParm()
                        if rp is not None and rp.path() != p.path():
                            extras.append("chref %s -> %s/%s" % (
                                p.name(), self.rel(rp.node()), rp.name()))
                    except Exception:
                        pass
                # node paths in string parms
                if t.type() == hou.parmTemplateType.String:
                    for v in vals:
                        vv = v.strip()
                        if vv and "\n" not in vv and len(vv) < 300 and vv[0] in "/." and "`" not in vv:
                            try:
                                target = node.node(vv)
                                if target is not None:
                                    extras.append("node ref -> %s" % self.rel(target))
                            except Exception:
                                pass
                spare = ""
                try:
                    if pt[0].isSpare():
                        spare = " [spare]"
                except Exception:
                    pass

                label = t.label()
                name = pt.name()
                if len(vals) == 1:
                    v = vals[0]
                    if "\n" in v or len(v) > MAX_STRING_INLINE:
                        self.w("    %s '%s'%s =" % (name, label, spare))
                        self.block(v)
                    else:
                        self.w("    %s '%s'%s = %s" % (name, label, spare, v))
                else:
                    self.w("    %s '%s'%s = (%s)" % (name, label, spare, ", ".join(vals)))
                for e in extras:
                    self.w("        ^ %s" % e)
                written += 1
            except Exception as e:
                self.w("    <error reading %s: %s>" % (pt.name(), e))
        if not written:
            self.w("    (all parameters at default)")

    # ------------------------------------------------------------------ interface
    def dump_templates(self, templates, depth=1):
        ind = "  " * depth
        for t in templates:
            try:
                bits = ["%s%s '%s' <%s>" % (ind, t.name(), t.label(), t.type().name())]
                if isinstance(t, hou.FolderParmTemplate):
                    bits.append("folder=%s" % t.folderType().name())
                else:
                    try:
                        if t.numComponents() > 1:
                            bits.append("size=%d" % t.numComponents())
                    except Exception:
                        pass
                    try:
                        dv = t.defaultValue()
                        if isinstance(dv, (tuple, list)):
                            dv = tuple(self.fnum(x) for x in dv)
                        bits.append("default=%s" % (dv,))
                    except Exception:
                        pass
                    try:
                        de = t.defaultExpression()
                        if de and any(de):
                            bits.append("defaultExpr=%s" % (de,))
                    except Exception:
                        pass
                    try:
                        mi = t.menuItems()
                        if mi:
                            ml = t.menuLabels()
                            bits.append("menu=%s" % list(zip(mi, ml)))
                    except Exception:
                        pass
                    try:
                        if t.isHidden():
                            bits.append("HIDDEN")
                    except Exception:
                        pass
                self.w("  ".join(bits))
                try:
                    for ctype, cond in t.conditionals().items():
                        self.w("%s    %s: %s" % (ind, ctype.name(), cond))
                except Exception:
                    pass
                try:
                    tags = t.tags()
                    if tags:
                        for k, v in sorted(tags.items()):
                            if "\n" in v or len(v) > MAX_STRING_INLINE:
                                self.w("%s    tag %s =" % (ind, k))
                                self.block(v, ind + "      | ")
                            else:
                                self.w("%s    tag %s = %s" % (ind, k, v))
                except Exception:
                    pass
                if isinstance(t, hou.FolderParmTemplate):
                    self.dump_templates(t.parmTemplates(), depth + 1)
            except Exception as e:
                self.w("%s<template error %s>" % (ind, e))

    def dump_sections(self, definition):
        try:
            secs = definition.sections()
        except Exception as e:
            self.w("  <cannot read sections: %s>" % e)
            return
        for name in sorted(secs):
            low = name.lower()
            if name.startswith(SKIP_SECTIONS_PREFIX) or low.endswith(SKIP_SECTIONS_EXT):
                self.w("  -- section '%s' (skipped binary/irrelevant)" % name)
                continue
            try:
                data = secs[name].contents()
            except Exception as e:
                self.w("  -- section '%s' <unreadable: %s>" % (name, e))
                continue
            if isinstance(data, bytes):
                try:
                    data = data.decode("utf-8")
                except Exception:
                    self.w("  -- section '%s' (binary, skipped)" % name)
                    continue
            if "\x00" in data:
                self.w("  -- section '%s' (binary, skipped)" % name)
                continue
            self.w("  -- section '%s' (%d chars)" % (name, len(data)))
            if len(data) > MAX_SECTION_CHARS:
                data = data[:MAX_SECTION_CHARS] + "\n...<truncated>"
            self.block(data, "      | ")

    def dump_definition(self, definition, with_interface, with_sections):
        self.w("  type        : %s" % definition.nodeTypeName())
        self.w("  category    : %s" % definition.nodeTypeCategory().name())
        self.w("  label       : %s" % definition.description())
        for attr in ("version", "libraryFilePath", "minNumInputs", "maxNumInputs", "maxNumOutputs"):
            try:
                self.w("  %-12s: %s" % (attr, getattr(definition, attr)()))
            except Exception:
                pass
        if with_interface:
            self.w("")
            self.w("  PARAMETER INTERFACE:")
            try:
                self.dump_templates(definition.parmTemplateGroup().entries(), 2)
            except Exception as e:
                self.w("  <cannot read interface: %s>" % e)
        if with_sections:
            self.w("")
            self.w("  HDA SECTIONS:")
            self.dump_sections(definition)

    # ------------------------------------------------------------------ layout
    def ascii_layout(self, net):
        items = []
        for n in net.children():
            try:
                pos, size = n.position(), n.size()
                cx, cy = pos[0] + size[0] * 0.5, pos[1] + size[1] * 0.5
            except Exception:
                continue
            mark = ""
            fl = self.flags(n)
            if "DISPLAY" in fl:
                mark += "*"
            if "BYPASS" in fl:
                mark += "~"
            label = n.name()[:ASCII_LABEL_LEN] + mark
            items.append((cx, cy, "[" + label + "]"))
        for d in net.networkDots():
            try:
                p = d.position()
                items.append((p[0], p[1], "o"))
            except Exception:
                pass
        try:
            for ii in net.indirectInputs():
                p = ii.position()
                items.append((p[0], p[1], "<IN%d>" % (ii.number() + 1)))
        except Exception:
            pass
        for s in net.stickyNotes():
            try:
                p, sz = s.position(), s.size()
                txt = (s.text().strip().splitlines() or [""])[0][:20]
                items.append((p[0] + sz[0] * 0.5, p[1] + sz[1], '{note:"%s"}' % txt))
            except Exception:
                pass
        if not items:
            return ["      (empty)"]

        minx = min(i[0] for i in items)
        maxy = max(i[1] for i in items)
        rows = {}
        for x, y, label in items:
            r = int(round((maxy - y) * ASCII_Y_SCALE))
            c = int(round((x - minx) * ASCII_X_SCALE))
            rows.setdefault(r, []).append((c, label))

        out, blank = [], 0
        for r in range(max(rows) + 1):
            if r not in rows:
                blank += 1
                if blank <= 1:
                    out.append("")
                continue
            blank = 0
            line = ""
            for c, label in sorted(rows[r]):
                start = max(c - len(label) // 2, len(line) + (1 if line else 0), 0)
                line += " " * (start - len(line)) + label
            out.append("      " + line.rstrip())
        return out

    # ------------------------------------------------------------------ networks
    def dump_network(self, net, depth):
        children = net.children()
        title = "NETWORK: %s   [%s]   (%d nodes)" % (
            self.rel(net), net.type().nameWithCategory(), len(children))
        self.header(title, "#")
        try:
            self.w("child category: %s" % net.childTypeCategory().name())
        except Exception:
            pass

        # layout
        self.w("")
        self.w("LAYOUT  (top of network editor at top; * = display flag, ~ = bypassed, o = dot)")
        for ln in self.ascii_layout(net):
            self.w(ln)

        # subnet inputs
        try:
            iis = net.indirectInputs()
        except Exception:
            iis = ()
        if iis:
            self.w("")
            self.w("SUBNET INPUTS (wires coming into this network from outside):")
            for ii in iis:
                try:
                    outs = ["%s[%d]" % (c.outputNode().name(), c.inputIndex())
                            for c in ii.outputConnections()]
                except Exception:
                    outs = [n.name() for n in ii.outputs()]
                outer = ""
                try:
                    src = net.inputs()[ii.number()]
                    if src is not None:
                        outer = "  (outside: fed by %s)" % self.rel(src)
                except Exception:
                    pass
                self.w("  input #%d -> %s%s" % (ii.number() + 1, ", ".join(outs) or "(unused)", outer))

        # network boxes
        boxes = net.networkBoxes()
        if boxes:
            self.w("")
            self.w("NETWORK BOXES:")
            for b in boxes:
                try:
                    p, s = b.position(), b.size()
                    self.w("  box '%s' title='%s' pos=(%.2f,%.2f) size=(%.2f,%.2f)%s" % (
                        b.name(), b.comment(), p[0], p[1], s[0], s[1],
                        " MINIMIZED" if b.isMinimized() else ""))
                    self.w("      contains: %s" % ", ".join(n.name() for n in b.nodes()))
                    try:
                        inner = [x.name() for x in b.networkBoxes()]
                        if inner:
                            self.w("      nested boxes: %s" % ", ".join(inner))
                    except Exception:
                        pass
                except Exception as e:
                    self.w("  <box error %s>" % e)

        # sticky notes
        notes = net.stickyNotes()
        if notes:
            self.w("")
            self.w("STICKY NOTES:")
            for s in notes:
                try:
                    p = s.position()
                    self.w("  note '%s' pos=(%.2f,%.2f):" % (s.name(), p[0], p[1]))
                    self.block(s.text(), "      | ")
                except Exception:
                    pass

        # dots
        dots = net.networkDots()
        if dots:
            self.w("")
            self.w("DOTS:")
            for d in dots:
                try:
                    self.w("  %s fed by %s[%d]%s" % (
                        d.name(), self.item_desc(d.inputItem()), d.inputItemOutputIndex(),
                        " (pinned)" if d.isPinned() else ""))
                except Exception:
                    pass

        # wiring
        self.w("")
        self.w("WIRING  (source[output] -> destination[input 'label']):")
        any_wire = False
        for n in sorted(children, key=lambda x: (-x.position()[1], x.position()[0])):
            try:
                labels = n.inputLabels()
            except Exception:
                labels = ()
            for c in n.inputConnections():
                try:
                    src = c.inputItem()
                    idx = c.inputIndex()
                    lbl = labels[idx] if idx < len(labels) else ""
                    via = ""
                    try:
                        real = c.inputNode()
                        if real is not None and not isinstance(src, hou.Node):
                            via = "   (resolves to %s)" % real.name()
                    except Exception:
                        pass
                    try:
                        out_idx = c.outputIndex()
                    except Exception:
                        out_idx = 0
                    self.w("  %s[%d] -> %s[%d '%s']%s" % (
                        self.item_desc(src), out_idx, n.name(), idx, lbl, via))
                    any_wire = True
                except Exception as e:
                    self.w("  <wire error on %s: %s>" % (n.name(), e))
        if not any_wire:
            self.w("  (no wires)")

        # nodes in detail
        self.w("")
        self.w("NODES:")
        for n in sorted(children, key=lambda x: (-x.position()[1], x.position()[0])):
            self.dump_node(n)

        # recurse
        if depth < MAX_DEPTH:
            for n in sorted(children, key=lambda x: (-x.position()[1], x.position()[0])):
                try:
                    if n.children():
                        self.dump_network(n, depth + 1)
                except Exception:
                    pass

    def dump_node(self, n):
        t = n.type()
        self.w("")
        self.w("  " + "-" * 96)
        self.w("  NODE %s" % self.rel(n))
        self.w("    type     : %s  ('%s')" % (t.nameWithCategory(), t.description()))
        d = t.definition()
        if d is not None:
            try:
                self.w("    HDA      : %s  lib=%s%s" % (
                    d.version() or "-", d.libraryFilePath(),
                    "  (was locked)" if n.path() in self.orig_locked else ""))
            except Exception:
                pass
            key = t.nameWithCategory()
            if key not in self.nested_defs and n != self.root:
                self.nested_defs[key] = d
        fl = self.flags(n)
        if fl:
            self.w("    flags    : %s" % " ".join(fl))
        try:
            p, s = n.position(), n.size()
            self.w("    position : (%.3f, %.3f)  size (%.2f, %.2f)" % (p[0], p[1], s[0], s[1]))
        except Exception:
            pass
        try:
            rgb = n.color().rgb()
            self.w("    color    : (%.2f, %.2f, %.2f)" % rgb)
        except Exception:
            pass
        try:
            box = n.parentNetworkBox()
            if box is not None:
                self.w("    in box   : %s '%s'" % (box.name(), box.comment()))
        except Exception:
            pass
        try:
            if n.comment():
                self.w("    comment  :")
                self.block(n.comment(), "      | ")
        except Exception:
            pass

        # inputs/outputs
        try:
            labels = n.inputLabels()
        except Exception:
            labels = ()
        ins = n.inputConnections()
        if ins:
            for c in ins:
                idx = c.inputIndex()
                lbl = labels[idx] if idx < len(labels) else ""
                try:
                    oi = c.outputIndex()
                except Exception:
                    oi = 0
                self.w("    input %d '%s' <- %s[%d]" % (idx, lbl, self.item_desc(c.inputItem()), oi))
        outs = n.outputConnections()
        if outs:
            for c in outs:
                try:
                    self.w("    output %d -> %s[%d]" % (c.outputIndex(), c.outputNode().name(), c.inputIndex()))
                except Exception:
                    pass

        # cross-node relationships (parm/expression references)
        for label, fn in (("references", "references"), ("dependents", "dependents")):
            try:
                rel = [x for x in getattr(n, fn)(False) if x.path() != n.path()]
                if rel:
                    self.w("    %-9s: %s" % (label, ", ".join(sorted(self.rel(x) for x in rel))))
            except Exception:
                pass

        try:
            ud = n.userDataDict()
            ud = {k: v for k, v in ud.items() if not k.startswith("___")}
            if ud:
                self.w("    userData : %s" % ud)
        except Exception:
            pass

        self.w("    params (non-default):")
        self.dump_parms(n)

    # ------------------------------------------------------------------ tree
    def tree(self, net, depth=0, counts=None):
        for c in sorted(net.children(), key=lambda x: (-x.position()[1], x.position()[0])):
            t = c.type()
            counts[t.nameWithCategory()] = counts.get(t.nameWithCategory(), 0) + 1
            tag = " {HDA}" if t.definition() is not None else ""
            fl = self.flags(c)
            self.w("  " + "    " * depth + "%s  [%s]%s%s" % (
                c.name(), t.nameWithCategory(), tag, ("  " + ",".join(fl)) if fl else ""))
            try:
                if c.children() and depth < MAX_DEPTH:
                    self.tree(c, depth + 1, counts)
            except Exception:
                pass

    # ------------------------------------------------------------------ main
    def run(self):
        r = self.root
        self.w("HOUDINI NETWORK DUMP")
        self.w("source node : %s" % self.source.path())
        self.w("dumped as   : %s" % r.path())
        self.w("node type   : %s ('%s')" % (r.type().nameWithCategory(), r.type().description()))
        self.w("houdini     : %s" % hou.applicationVersionString())
        self.w("date        : %s" % datetime.datetime.now().isoformat(" ", "seconds"))
        self.w("paths below are relative to the dumped node (<ROOT>)")

        d = r.type().definition()
        if d is not None:
            self.header("ROOT HDA DEFINITION")
            self.dump_definition(d, DUMP_ROOT_INTERFACE, DUMP_ROOT_HDA_SECTIONS)

        self.header("ROOT NODE")
        self.dump_node(r)

        self.header("HIERARCHY TREE")
        counts = {}
        self.tree(r, 0, counts)

        self.header("NODE TYPE USAGE")
        for k, v in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])):
            self.w("  %4d  %s" % (v, k))

        if r.children():
            self.dump_network(r, 0)

        if self.nested_defs:
            self.header("NESTED HDA TYPES USED INSIDE")
            for key in sorted(self.nested_defs):
                nd = self.nested_defs[key]
                try:
                    self.w("  %s  v%s  lib=%s" % (key, nd.version() or "-", nd.libraryFilePath()))
                except Exception:
                    self.w("  %s" % key)
            if DUMP_NESTED_HDA_SECTIONS:
                for key in sorted(self.nested_defs):
                    self.header("NESTED HDA: %s" % key, "-")
                    self.dump_definition(self.nested_defs[key], True, True)

        self.w("")
        self.w("END OF DUMP")
        return "\n".join(self.lines)


# ----------------------------------------------------------------------------- unlocking
def collect_locked(root):
    locked = set()
    for n in [root] + list(root.allSubChildren()):
        try:
            if n.type().definition() is not None and n.isLockedHDA():
                locked.add(n.path())
        except Exception:
            pass
    return locked


def unlock_all(root):
    """Allow editing of contents on the root and every nested HDA (top-down, repeated
    because unlocking a parent can expose more locked children)."""
    unlocked = 0
    for _ in range(10):
        changed = False
        nodes = [root] + list(root.allSubChildren())
        nodes.sort(key=lambda n: n.path().count("/"))
        for n in nodes:
            try:
                if n.type().definition() is not None and n.isLockedHDA():
                    n.allowEditingOfContents()
                    unlocked += 1
                    changed = True
            except Exception:
                pass
        if not changed:
            break
    return unlocked


# ----------------------------------------------------------------------------- entry point
def default_out_path(node):
    base = hou.expandString("$HIP")
    if not base or not os.path.isdir(base):
        base = os.path.expanduser("~")
    return os.path.join(base, "netdump_%s.txt" % node.name())


def main(node_path=None, out_path=None):
    ui = hou.isUIAvailable()
    temp_container = None
    copy_node = None

    if node_path:
        source = hou.node(node_path)
        if source is None:
            raise hou.Error("Node not found: %s" % node_path)
    else:
        sel = hou.selectedNodes()
        if sel:
            source = sel[0]
        else:
            if ui:
                choice = hou.ui.displayMessage(
                    "No node selected.\nCreate a temporary '%s' and dump it?" % FALLBACK_NODE_TYPE,
                    buttons=("Dump temporary %s" % FALLBACK_NODE_TYPE, "Cancel"))
                if choice != 0:
                    return
            temp_container = hou.node("/obj").createNode("geo", "__netdump_tmp")
            source = temp_container.createNode(FALLBACK_NODE_TYPE)

    target = source
    try:
        if WORK_ON_COPY and temp_container is None:
            copy_node = hou.copyNodesTo([source], source.parent())[0]
            try:
                copy_node.setName(source.name() + "_NETDUMP", unique_name=True)
            except Exception:
                pass
            target = copy_node

        originally_locked = collect_locked(target)
        # map copy paths back so "(was locked)" still makes sense
        n_unlocked = unlock_all(target) if UNLOCK_HDAS else 0

        with hou.InterruptableOperation("Dumping network", open_interrupt_dialog=True):
            text = Dumper(target, source, originally_locked).run()

        if not out_path:
            out_path = default_out_path(source)
            if ui:
                picked = hou.ui.selectFile(
                    start_directory=os.path.dirname(out_path),
                    title="Save network dump",
                    default_value=os.path.basename(out_path),
                    pattern="*.txt",
                    chooser_mode=hou.fileChooserMode.Write)
                if not picked:
                    return
                out_path = hou.expandString(picked)
                if os.path.isdir(out_path):
                    out_path = os.path.join(out_path, os.path.basename(default_out_path(source)))

        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)

        msg = "Dumped %s\n%d lines, %d HDAs unlocked\n-> %s" % (
            source.path(), text.count("\n") + 1, n_unlocked, out_path)
        print(msg)
        if ui:
            hou.ui.displayMessage(msg)
        return out_path
    finally:
        if copy_node is not None:
            try:
                copy_node.destroy()
            except Exception:
                pass
        if temp_container is not None:
            try:
                temp_container.destroy()
            except Exception:
                pass


if __name__ in ("__main__", "__builtin__", "builtins"):
    main()
