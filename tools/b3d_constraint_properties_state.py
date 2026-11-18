"""The Box3D Constraint Properties SOP's viewer state (its ViewerStateModule section, read by
make_constraint_properties_sop.py).

RBD Constraint Properties' state, adapted: pick constraint lines in the viewport, then turn a rotation handle to rotate
the frames of their two anchors (the `orient` point attribute, which sets the axes of a revolute, prismatic or wheel
joint). RBD's handle aims a hard constraint's angular motors; Box3D's motors act in the joint frame, so the handle
already aims them. Toggle Motor switches `enable_motor`, and the mouse wheel edits the spherical joint's
`motor_velocity` (Z; Shift X, Alt Y; Ctrl for ten times the step). The motor speeds and torques are floats, which
multiply the relationship node's value (zero stays zero), so the viewport does not edit them. The edits live in the
node's `stash` (one point per edited line: `__idx`, `orient_a`, `orient_b`, `enable_motor`, `motor_velocity`) and the
network applies them on top of what the settings write. The nodes the state reads inside the asset are CONSTRAINT (the
selected lines), HANDLE (the handle's transform), GUIDES (the anchor frames), GLOW (the edited lines) and `widgets`
(what the selector picks from). This file runs inside Houdini only (`hou`, `kwargs` and the viewer state loader
provide the rest), not as a script.
"""

import hou
from hou import hmath
import viewerstate.utils as su

IDENTITY = (0.0, 0.0, 0.0, 1.0)
UNSET_FLOAT = -3.4028235e38  # kUnsetFloat (Box3DJoints.h): "not edited" in the stash and the attributes
# What a stash point holds, with the value that means "not edited" (the network skips it).
FIELDS = {"orient_a": (0.0, 0.0, 0.0, 0.0), "orient_b": (0.0, 0.0, 0.0, 0.0), "enable_motor": -1,
          "motor_velocity": (UNSET_FLOAT, 0.0, 0.0)}
MOTOR_STEP = 10.0  # degrees per second for one wheel click
# The Constraint Type Spherical folder's parameters, which Edit Parms drives instead of the stash.
MOTOR_PARM = "spherical_enable_motor"
VELOCITY_PARM = "spherical_motor_velocity"


# ---------------------------------------------------------------------------------------------- the stash
def selected_ids(node):
    """The `__idx` of every selected line (the lines inside the Constraint Group that `__sel` picks)."""
    geo = node.node("CONSTRAINT").geometry()
    if geo is None or geo.findPrimAttrib("__idx") is None:
        return []
    return list(geo.primIntAttribValues("__idx"))


def line_orients(node, ids=None):
    """{__idx: (orient of anchor A, orient of anchor B)} of the selected lines as the network outputs them."""
    geo = node.node("CONSTRAINT").geometry()
    out = {}
    if geo is None or geo.findPrimAttrib("__idx") is None:
        return out
    has_orient = geo.findPointAttrib("orient") is not None
    for prim in geo.prims():
        idx = prim.attribValue("__idx")
        points = prim.points()
        if (ids is not None and idx not in ids) or len(points) < 2:
            continue
        out[idx] = tuple(tuple(p.attribValue("orient")) if has_orient else IDENTITY for p in points[:2])
    return out


def stash_fields(node):
    """{__idx: {field: value}} of the node's stash."""
    stash = node.parm("stash").eval()
    out = {}
    if stash is None:
        return out
    names = {a.name() for a in stash.pointAttribs()}
    for pt in stash.points():
        out[pt.attribValue("__idx")] = {
            f: (tuple(pt.attribValue(f)) if isinstance(default, tuple) else pt.attribValue(f))
            for f, default in FIELDS.items() if f in names}
    return out


def write_stash(node, fields):
    """Replaces the node's stash with `fields` ({__idx: {field: value}}; a field left out is "not edited")."""
    geo = hou.Geometry()
    geo.addAttrib(hou.attribType.Point, "__idx", -1)
    for f, default in FIELDS.items():
        geo.addAttrib(hou.attribType.Point, f, default)
    for idx, values in fields.items():
        if not values:
            continue
        pt = geo.createPoint()
        pt.setAttribValue("__idx", idx)
        for f, v in values.items():
            pt.setAttribValue(f, v)
    node.parm("stash").set(geo)


def update_edits(node, changes):
    """Puts `changes` ({__idx: {field: value}}) in the stash, on top of what it holds for those lines."""
    fields = stash_fields(node)
    for idx, values in changes.items():
        fields.setdefault(idx, {}).update(values)
    write_stash(node, fields)


def set_edits(node, edits):
    """Rotates the anchors: `edits` is {__idx: (orient A, orient B)}."""
    update_edits(node, {idx: {"orient_a": tuple(qa), "orient_b": tuple(qb)} for idx, (qa, qb) in edits.items()})


def remove_edits(node, ids):
    fields = stash_fields(node)
    write_stash(node, {i: v for i, v in fields.items() if i not in set(ids)})


def line_values(node, name):
    """{__idx: value} of the selected lines' primitive attribute `name` as the network outputs it (missing: none)."""
    geo = node.node("CONSTRAINT").geometry()
    out = {}
    if geo is None or geo.findPrimAttrib(name) is None or geo.findPrimAttrib("__idx") is None:
        return out
    for prim in geo.prims():
        out[prim.attribValue("__idx")] = prim.attribValue(name)
    return out


def toggle_motor(node, ids):
    """Toggle Motor: enable_motor on a line that has it off or not set, off on one that has it on."""
    current = line_values(node, "enable_motor")
    update_edits(node, {i: {"enable_motor": 0 if current.get(i, -1) == 1 else 1} for i in ids})


def add_motor_velocity(node, ids, axis, delta):
    """The mouse wheel: `delta` degrees per second on component `axis` (0 X, 1 Y, 2 Z) of each line's motor_velocity."""
    current = line_values(node, "motor_velocity")
    changes = {}
    for i in ids:
        v = list(current.get(i, (UNSET_FLOAT, 0.0, 0.0)))
        if v[0] <= -1e30:
            v = [0.0, 0.0, 0.0]
        v[axis] += delta
        changes[i] = {"motor_velocity": tuple(v)}
    update_edits(node, changes)


def quaternion_from_euler(rx, ry, rz):
    """The orient value (x, y, z, w) of a rotation in degrees, the handle's XYZ order."""
    q = hou.Quaternion(hmath.buildRotate(rx, ry, rz))
    return (q[0], q[1], q[2], q[3])


def quaternion_to_line(direction):
    """The orient that turns the X axis onto `direction`."""
    q = hou.Quaternion()
    q.setToVectors(hou.Vector3(1, 0, 0), hou.Vector3(direction).normalized())
    return (q[0], q[1], q[2], q[3])


def clearEdits(kwargs):
    """Clear Edits: forgets every edit made in the viewport."""
    with hou.undos.group("Clear Edits"):
        kwargs["node"].parm("stash").revertToDefaults()


def importEdits(kwargs):
    """Import Edits: puts the selected lines' current anchor frames in the stash, to edit from there."""
    node = kwargs["node"]
    with hou.undos.group("Import Edits"):
        edits = line_orients(node)
        if edits:
            set_edits(node, edits)


# ---------------------------------------------------------------------------------------------- the state
class State(object):
    HK_CTXT = su.hotkeyContextForState("box3d_constraint_properties", hou.sopNodeTypeCategory())
    HK_RESET = HK_CTXT + "." + "reset"
    HK_ALIGN = HK_CTXT + "." + "align"
    HK_SELECTALL = HK_CTXT + "." + "selectall"
    HK_COPY = HK_CTXT + "." + "copy"
    HK_PASTE = HK_CTXT + "." + "paste"
    HK_PARMS = HK_CTXT + "." + "parms"
    HK_MOTOR = HK_CTXT + "." + "motor"

    HUD_TEMPLATE = {
        "title": "Box3D Constraint Properties",
        "icon": "DOP_box3d_data",
        "rows": [
            {
                "type": "group", "id": "mode",
                "rows": [
                    {"label": "Reset Constraint", "key": su.hudHotkeyRef(HK_RESET)},
                    {"label": "Align Frame to Constraint Line", "key": su.hudHotkeyRef(HK_ALIGN)},
                    {"label": "Select All Constraints", "key": su.hudHotkeyRef(HK_SELECTALL)},
                    {"label": "Copy Constraint", "key": su.hudHotkeyRef(HK_COPY)},
                    {"label": "Paste Constraint", "key": su.hudHotkeyRef(HK_PASTE)},
                    {"label": "Toggle Motor", "key": su.hudHotkeyRef(HK_MOTOR)},
                    {"label": "Toggle Edit Parms", "key": su.hudHotkeyRef(HK_PARMS)},
                    {"label": "Edit Motor Velocity (Z)", "key": "mousewheel"},
                    {"label": "Edit Motor Velocity (X)", "key": "Shift+mousewheel"},
                    {"label": "Edit Motor Velocity (Y)", "key": "Alt+mousewheel"},
                ],
            }
        ],
    }

    def __init__(self, state_name, scene_viewer):
        self.state_name = state_name
        self.scene_viewer = scene_viewer
        self.scene_viewer.hudInfo(template=self.HUD_TEMPLATE)
        self.__outputflag = None
        self.__clipboard = None
        self.rotation_handle = hou.Handle(scene_viewer, "Rotation Handle")
        self.__edited = hou.GeometryDrawable(scene_viewer, hou.drawableGeometryType.Line, "__edited")
        self.__guides = hou.GeometryDrawable(
            scene_viewer, hou.drawableGeometryType.Line, "guides",
            params={"style": hou.drawableGeometryLineStyle.Plain, "color1": (1.0, 1.0, 1.0, 1.0),
                    "glow_width": 5, "fade_factor": 0.1,
                    "highlight_mode": hou.drawableHighlightMode.MatteOverGlow, "use_cd": True})

    # ---- helpers
    def _has_selection(self, node):
        return node.evalParm("__idx") >= 0 and bool(selected_ids(node))

    def _refresh(self, node):
        show = self._has_selection(node)
        self.rotation_handle.show(show)
        self.rotation_handle.update()
        self.scene_viewer.curViewport().draw()

    def _prompt(self, node):
        self.scene_viewer.clearPromptMessage()
        if node.evalParm("__idx") < 0:
            self.scene_viewer.setPromptMessage("Select one or more constraints.")

    # ---- lifecycle
    def onEnter(self, kwargs):
        node = kwargs["node"]
        self.__guides.setGeometry(node.node("GUIDES").geometry())
        self.__edited.setGeometry(node.node("GLOW").geometry())
        self.__guides.show(True)
        self.rotation_handle.show(self._has_selection(node))
        self.rotation_handle.update()
        self.scene_viewer.hudInfo(show=1)
        self._prompt(node)
        if not self.scene_viewer.isSecureSelection():
            self.scene_viewer.triggerStateSelector(hou.triggerSelectorAction.Start)
        else:
            self.scene_viewer.triggerStateSelector(hou.triggerSelectorAction.Stop)

    def onResume(self, kwargs):
        self.onEnter(kwargs)

    def onInterrupt(self, kwargs):
        self.__guides.show(True)
        node = kwargs["node"]
        if kwargs.get("interrupt_state") == "select":
            if self.__outputflag in [None, -1]:
                self.__outputflag = max(node.outputForViewFlag(), 0)
            node.setOutputForViewFlag(-1)

    def onExit(self, kwargs):
        node = kwargs.get("node")
        self.scene_viewer.clearPromptMessage()
        if node and self.__outputflag is not None:
            node.setOutputForViewFlag(self.__outputflag)
            self.__outputflag = None

    def onMouseEvent(self, kwargs):
        self._prompt(kwargs["node"])
        return False

    # ---- menu and hotkeys
    def onMenuAction(self, kwargs):
        action = kwargs["menu_item"]
        node = kwargs["node"]
        ids = selected_ids(node)

        if action == "parms":
            with hou.undos.group("Toggle Edit Parms"):
                node.parm("editparms").set(1 - node.evalParm("editparms"))
        elif action == "selectall":
            with hou.undos.group("Select All Constraints"):
                node.parm("__sel").set("*")
                if node.evalParm("__idx") < 0:
                    node.parm("__idx").set(0)
        elif not ids:
            return
        elif action == "reset":
            with hou.undos.group("Reset Constraint"):
                remove_edits(node, ids)
        elif action == "align":
            with hou.undos.group("Align Constraint"):
                edits = {}
                geo = node.node("CONSTRAINT").geometry()
                for prim in geo.prims():
                    points = prim.points()
                    if len(points) < 2:
                        continue
                    direction = points[1].position() - points[0].position()
                    if hou.almostEqual(direction.lengthSquared(), 0):
                        continue
                    q = quaternion_to_line(direction)
                    edits[prim.attribValue("__idx")] = (q, q)
                if edits:
                    set_edits(node, edits)
        elif action == "motor":
            with hou.undos.group("Toggle Motor"):
                if node.evalParm("editparms") and node.parm(MOTOR_PARM) is not None:
                    node.parm(MOTOR_PARM).set(1 - node.evalParm(MOTOR_PARM))
                    node.parm("add" + MOTOR_PARM).set(1)
                else:
                    toggle_motor(node, ids)
        elif action == "copy":
            first = line_orients(node, ids).get(ids[0])
            self.__clipboard = first
        elif action == "paste":
            if self.__clipboard is not None:
                with hou.undos.group("Paste Constraint"):
                    set_edits(node, {i: self.__clipboard for i in ids})
        self._refresh(node)

    def onMouseWheelEvent(self, kwargs):
        """The wheel edits the motor velocity: Z, with Shift X, with Alt Y, ten times the step with Ctrl."""
        node = kwargs["node"]
        ids = selected_ids(node)
        if not ids:
            return
        device = kwargs["ui_event"].device()
        axis = 0 if device.isShiftKey() else 1 if device.isAltKey() else 2
        delta = device.mouseWheel() * MOTOR_STEP * (10 if device.isCtrlKey() else 1)
        with hou.undos.group("Edit Motor Velocity"):
            if node.evalParm("editparms") and node.parmTuple(VELOCITY_PARM) is not None:
                node.parm("add" + VELOCITY_PARM).set(1)
                parm = node.parmTuple(VELOCITY_PARM)[axis]
                parm.set(parm.eval() + delta)
            else:
                add_motor_velocity(node, ids, axis, delta)
        self._refresh(node)

    # ---- the handle
    def onHandleToState(self, kwargs):
        """The handle was dragged: write the new frame into the stash, or into Anchor Rotation with Edit Parms on."""
        node = kwargs["node"]
        if kwargs["handle"] != "Rotation Handle" or not self._has_selection(node):
            return
        parms = kwargs["parms"]
        mod_parms = kwargs["mod_parms"]
        if not mod_parms or mod_parms[0].startswith("t"):
            self.rotation_handle.update()
            return
        rx, ry, rz = parms["rx"], parms["ry"], parms["rz"]
        if node.evalParm("editparms"):
            node.parm("addanchorrotate").set(1)
            node.parmTuple("anchorrotate").set((rx, ry, rz))
        else:
            q = quaternion_from_euler(rx, ry, rz)
            set_edits(node, {i: (q, q) for i in selected_ids(node)})

    def onStateToHandle(self, kwargs):
        """Put the handle on the first selected line, in its frame."""
        node = kwargs["node"]
        if kwargs["handle"] != "Rotation Handle":
            return
        show = self._has_selection(node)
        self.rotation_handle.show(show)
        if not show:
            return
        xform = hou.Matrix4(node.node("HANDLE").geometry().attribValue("xform"))
        translates = xform.extractTranslates()
        rotates = xform.extractRotates()
        parms = kwargs["parms"]
        parms["tx"], parms["ty"], parms["tz"] = translates
        parms["rx"], parms["ry"], parms["rz"] = rotates

    # ---- drawing
    def onDraw(self, kwargs):
        handle = kwargs["draw_handle"]
        node = kwargs["node"]
        self.__guides.draw(handle)
        if node.node("GLOW").geometry().prims():
            self.__edited.show(True)
            self.__edited.draw(handle, {
                "style": hou.drawableGeometryLineStyle.Dot1, "use_cd": False,
                "color1": hou.Vector4(1, 1, 1, 1), "color2": hou.Vector4(0.2, 0.2, 0.2, 0.2),
                "glow_width": 5, "fade_factor": 0,
                "highlight_mode": hou.drawableHighlightMode.GlowMinusMatte})
        else:
            self.__edited.show(False)

    def onDrawInterrupt(self, kwargs):
        self.onDraw(kwargs)

    # ---- the selector
    def onSelection(self, kwargs):
        node = kwargs.get("node")
        selection = kwargs["selection"]
        selstring = "" if not selection.nodes() else (selection.mergedSelectionString() or "*")
        with hou.undos.group("Select Constraint"):
            node.parm("__idx").set(-1)
            node.parm("__sel").set("")
            self.rotation_handle.show(False)
            if not selstring:
                return False
            group_types = ["None", hou.geometryType.Breakpoints, hou.geometryType.Edges, hou.geometryType.Points,
                           hou.geometryType.Primitives]
            if selection.geometryType() not in group_types:
                return False
            picked = hou.Geometry()
            blast = hou.sopNodeTypeCategory().nodeVerb("blast")
            blast.setParms({"group": selstring, "negate": True, "grouptype": group_types.index(selection.geometryType())})
            blast.execute(picked, [node.node("widgets").geometry()])
            try:
                ids = sorted(set(picked.pointIntAttribValues("__idx")))
            except hou.OperationFailed:
                return False
            if not ids:
                return False
            node.parm("__idx").set(ids[0])
            node.parm("__sel").set(" ".join("@__idx=%d" % i for i in ids))
            self._refresh(node)
            # Not accepted: the state shows the selection with its handle and guides.
            return False

    def onStartSelection(self, kwargs):
        node = kwargs["node"]
        if self.__outputflag in [None, -1]:
            self.__outputflag = max(node.outputForViewFlag(), 0)
        with hou.undos.disabler():
            node.setOutputForViewFlag(-1)
            sel = [hou.Selection(node.node("widgets").geometry(), hou.geometryType.Primitives, node.evalParm("__sel"))]
            self.scene_viewer.setCurrentGeometrySelection(hou.geometryType.Primitives, [node], sel)

    def onStopSelection(self, kwargs):
        node = kwargs.get("node")
        if not node:
            return
        with hou.undos.disabler():
            if self.__outputflag is not None:
                node.setOutputForViewFlag(max(0, self.__outputflag))
                self.__outputflag = None


def createViewerStateTemplate():
    """Mandatory entry point: the state template Houdini registers."""
    state_typename = kwargs["type"].definition().sections()["DefaultState"].contents()
    state_cat = hou.sopNodeTypeCategory()
    template = hou.ViewerStateTemplate(state_typename, "Box3D Constraint Properties", state_cat)
    template.bindFactory(State)
    template.bindIcon(kwargs["type"].icon())

    template.bindHandle("xform", "Rotation Handle", handle_parms=["tx", "ty", "tz", "rx", "ry", "rz"],
                        cache_previous_parms=True)

    keys = hou.PluginHotkeyDefinitions()
    reset = su.defineHotkey(keys, state_typename, "reset", "u", "Reset Constraint", state_cat=state_cat)
    copy = su.defineHotkey(keys, state_typename, "copy", "Ctrl+C", "Copy Constraint", state_cat=state_cat)
    paste = su.defineHotkey(keys, state_typename, "paste", "Ctrl+V", "Paste Constraint", state_cat=state_cat)
    parms = su.defineHotkey(keys, state_typename, "parms", "P", "Toggle Edit Parms", state_cat=state_cat)
    motor = su.defineHotkey(keys, state_typename, "motor", "M", "Toggle Motor", state_cat=state_cat)
    align = su.defineHotkey(keys, state_typename, "align", "A", "Align Frame to Constraint Line", state_cat=state_cat)
    selectall = su.defineHotkey(keys, state_typename, "selectall", "Ctrl+A", "Select All Constraints",
                                state_cat=state_cat)
    template.bindHotkeyDefinitions(keys)

    menu = hou.ViewerStateMenu(state_typename + "_menu", "Menu")
    menu.addActionItem("reset", "Reset Constraint", hotkey=reset)
    menu.addActionItem("align", "Align Frame to Constraint Line", hotkey=align)
    menu.addActionItem("selectall", "Select All Constraints", hotkey=selectall)
    menu.addActionItem("copy", "Copy Constraint", hotkey=copy)
    menu.addActionItem("paste", "Paste Constraint", hotkey=paste)
    menu.addActionItem("motor", "Toggle Motor", hotkey=motor)
    menu.addActionItem("parms", "Toggle Edit Parms", hotkey=parms)
    template.bindMenu(menu)

    template.bindGeometrySelector(
        "Select Constraints", allow_drag=True, quick_select=True, auto_start=False, toolbox=False,
        use_existing_selection=True, consume_selection=True, secure_selection=hou.secureSelectionOption.Obey,
        initial_selection="", initial_selection_type=hou.geometryType.Primitives, ordered=False, geometry_types=[],
        primitive_types=[], allow_other_sops=False, hotkey="", name="Constraint Selector")
    return template
