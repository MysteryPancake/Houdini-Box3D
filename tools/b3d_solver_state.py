"""The Box3D Solver SOP's viewer state (its ViewerStateModule section, read by make_solver_sop.py).

RBD Bullet Solver's state, trimmed to what applies here: an Option Menu in the viewport (which output the viewport
shows, and an inspector), the hotkeys 1 to 5 for them, and the HUD lines that say which output is shown. The inspector
prints the attributes of the primitive under the cursor next to it, using the Visualization tab's Attributes, Font Size
and Float Precision. RBD's false-colour constraint info, histogram and guide capture have no counterpart here. This
file runs inside Houdini only (`hou`, `kwargs` and the viewer state loader provide the rest), not as a script.
"""

import stateutils

# What the viewport can show, in the order of the node's outputs.
VIEWS = ["Geometry", "Constraint Geometry", "Proxy Geometry"]


def format_value(value, precision):
    """An attribute value as text: floats with `precision` decimals, tuples element by element."""
    fmt = "%%0.%df" % precision
    if isinstance(value, float):
        return fmt % value
    if isinstance(value, (tuple, list)) and value:
        if isinstance(value[0], float):
            return ", ".join(fmt % x for x in value)
        return ", ".join(str(x) for x in value)
    return str(value)


class SolverState(object):
    HK_CTXT = stateutils.hotkeyContextForState("box3d_solver_sop", hou.sopNodeTypeCategory())
    HK_VIEW_GEO = HK_CTXT + "." + "h1"
    HK_VIEW_CONS = HK_CTXT + "." + "h2"
    HK_VIEW_PROXY = HK_CTXT + "." + "h3"
    HK_TOGGLE_VIEW = HK_CTXT + "." + "h4"
    HK_TOGGLE_INSPECT = HK_CTXT + "." + "h5"

    HUD_TEMPLATE = {
        "title": "Box3D Solver",
        "icon": "DOP_box3d_data",
        "rows": [
            {"id": "view", "label": "View"},
            {"id": "view_g", "type": "choicegraph", "count": 3},
            {"id": "view1", "label": "View Geometry", "key": stateutils.hudHotkeyRef(HK_VIEW_GEO)},
            {"id": "view2", "label": "View Constraint Geometry", "key": stateutils.hudHotkeyRef(HK_VIEW_CONS)},
            {"id": "view3", "label": "View Proxy Geometry", "key": stateutils.hudHotkeyRef(HK_VIEW_PROXY)},
            {"label": "Toggle View", "key": stateutils.hudHotkeyRef(HK_TOGGLE_VIEW)},
            {"label": "Toggle Inspector", "key": stateutils.hudHotkeyRef(HK_TOGGLE_INSPECT)},
            {
                # The hints that only apply while the inspector is on.
                "type": "group", "id": "inspector_g",
                "rows": [
                    {
                        "type": "group", "id": "vis",
                        "rows": [
                            {"type": "divider", "label": "Inspector"},
                            {"label": "Change Font Size", "key": "mousewheel"},
                            {"label": "Change Precision", "key": "Ctrl mousewheel"},
                        ],
                    },
                    {"type": "group", "id": "invis", "rows": []},
                ],
            },
        ],
    }

    def __init__(self, state_name, scene_viewer):
        self.state_name = state_name
        self.scene_viewer = scene_viewer
        self.__mouse_screen = hou.Vector2()
        self.__text_at_cursor = ""
        self.__cursor_text = hou.TextDrawable(self.scene_viewer, "text_cursor")
        self.__cursor_text.show(True)
        self.__show_inspector = False
        self.__prev_origin = None
        self.__prev_dir = None
        self.__refresh = False
        self.scene_viewer.hudInfo(template=self.HUD_TEMPLATE)

    def onEnter(self, kwargs):
        self.__cursor_text.show(True)
        self._showActiveHUD(kwargs["node"])
        self.scene_viewer.hudInfo(show=1)

    def onResume(self, kwargs):
        self._showActiveHUD(kwargs["node"])
        self.__cursor_text.show(True)

    def onExit(self, kwargs):
        self.__cursor_text.show(False)

    def onInterrupt(self, kwargs):
        self.__cursor_text.show(False)

    def _showActiveHUD(self, node):
        output = max(0, node.outputForViewFlag())
        output = min(output, len(VIEWS) - 1)
        self.scene_viewer.hudInfo(values={
            "view": VIEWS[output],
            "view_g": output,
            "inspector_g": "vis" if self.__show_inspector else "invis",
        })

    def onMouseWheelEvent(self, kwargs):
        # The wheel changes the inspector's font size, with Ctrl its precision.
        if not self.__show_inspector:
            return
        node = kwargs["node"]
        device = kwargs["ui_event"].device()
        with hou.undos.group("Mouse Wheel"):
            wheel = device.mouseWheel()
            parm = "inspectorprecision" if device.isCtrlKey() else "inspectorfontsize"
            node.parm(parm).set(int(node.evalParm(parm) + wheel))
        self.__refresh = True
        self.onMouseEvent(kwargs)

    def onMouseEvent(self, kwargs):
        node = kwargs["node"]
        ui_event = kwargs["ui_event"]
        if not self.__refresh or not (self.__prev_origin and self.__prev_dir):
            (self.__prev_origin, self.__prev_dir) = ui_event.ray()
        else:
            self.__refresh = False
        origin, direction = self.__prev_origin, self.__prev_dir
        self._showActiveHUD(node)

        if not self.__show_inspector:
            self.__text_at_cursor = ""
            return

        geo = node.geometry(min(max(0, node.outputForViewFlag()), len(VIEWS) - 1))
        if geo is None:
            self.__text_at_cursor = ""
            return
        self.__mouse_screen = self.scene_viewer.curViewport().mapToScreen(origin)

        hit = stateutils.GeometryIntersector(geo)
        hit.intersect(origin, direction, snapping=False)
        if hit.prim_num < 0:
            self.__text_at_cursor = ""
            return

        precision = node.evalParm("inspectorprecision")
        pattern = node.evalParm("inspectorattribs") + " ^name"
        prim = geo.prim(hit.prim_num)
        points = prim.points()
        if not points:
            return
        point = points[0]
        if node.outputForViewFlag() == 1:
            # Constraint geometry: a line between two pieces, named at its ends.
            self.__text_at_cursor = "Name 1: %s<br />" % point.attribValue("name")
            self.__text_at_cursor += "Name 2: %s<br /><br />" % points[-1].attribValue("name")
        else:
            try:
                self.__text_at_cursor = "Name: %s<br />" % prim.attribValue("name")
            except hou.OperationFailed:
                self.__text_at_cursor = "Name: %s<br />" % point.attribValue("name")
            self.__text_at_cursor += "<br />"
            for attrib in sorted(a.name() for a in geo.pointAttribs() if hou.text.patternMatch(pattern, a.name())):
                self.__text_at_cursor += "%s: %s<br />" % (attrib, format_value(point.attribValue(attrib), precision))
        for attrib in sorted(a.name() for a in geo.primAttribs() if hou.text.patternMatch(pattern, a.name())):
            self.__text_at_cursor += "%s: %s<br />" % (attrib, format_value(prim.attribValue(attrib), precision))

    def onDraw(self, kwargs):
        node = kwargs["node"]
        if not self.__text_at_cursor or not self.__show_inspector or not self.__cursor_text.visible():
            return
        self.__cursor_text.draw(kwargs["draw_handle"], {
            "text": "<font size=%d>%s</font>" % (node.evalParm("inspectorfontsize"), self.__text_at_cursor),
            "multi_line": True,
            "translate": (self.__mouse_screen[0], self.__mouse_screen[1], 0.0),
            "highlight_mode": hou.drawableHighlightMode.MatteOverGlow,
            "glow_width": 3,
            "color1": hou.Color(1, 1, 1),
            "color2": (0, 0, 0, 0.7),
            "fade_factor": 0,
        })

    def onMenuPreOpen(self, kwargs):
        node = kwargs["node"]
        view = ["view_geo", "view_constraints", "view_proxy"][min(max(0, node.outputForViewFlag()), 2)]
        kwargs["menu_states"]["value"] = view
        kwargs["outputviews"] = view

    def onMenuAction(self, kwargs):
        action = kwargs["menu_item"]
        node = kwargs["node"]
        if action == "toggle_info":
            self.__show_inspector = not self.__show_inspector
        elif action == "toggle_view":
            node.setOutputForViewFlag((max(0, node.outputForViewFlag()) + 1) % 3)
        elif action == "outputviews":
            view = kwargs["outputviews"]
            node.setOutputForViewFlag({"view_geo": 0, "view_constraints": 1, "view_proxy": 2}[view])
        # Update the HUD and the inspector.
        self.__refresh = True
        self.onMouseEvent(kwargs)
        self.scene_viewer.curViewport().draw()


node_type = kwargs["type"]


def createViewerStateTemplate():
    """Mandatory entry point: the state template Houdini registers."""
    state_name = node_type.definition().sections()["DefaultState"].contents()
    category = node_type.category()
    template = hou.ViewerStateTemplate(state_name, node_type.description(), category)
    template.bindFactory(SolverState)
    template.bindIcon(node_type.icon())

    context = stateutils.hotkeyContextForState(state_name, category)
    hou.hotkeys.addContext(context, "Box3D Solver Operation", "These keys apply to the Box3D Solver operations")
    keys = (("h1", "view geo", ["1", "Pad1"]), ("h2", "view constraints", ["2", "Pad2"]),
            ("h3", "view proxy", ["3", "Pad3"]), ("h4", "toggle view", ["4", "Pad4"]),
            ("h5", "toggle inspector", ["5", "Pad5"]))
    for suffix, label, hotkeys in keys:
        hou.hotkeys.addCommand(context + "." + suffix, label, label, hotkeys)

    menu = hou.ViewerStateMenu("Option_Menu", "Option Menu")
    menu.addActionItem("toggle_info", "Toggle Inspector", hotkey=context + ".h5")
    menu.addActionItem("toggle_view", "Toggle Output for View Flag", hotkey=context + ".h4")
    menu.addSeparator()
    menu.addRadioStrip("outputviews", "Output for View Flags", "view_geo")
    menu.addRadioStripItem("outputviews", "view_geo", "View Geometry", hotkey=context + ".h1")
    menu.addRadioStripItem("outputviews", "view_constraints", "View Constraints", hotkey=context + ".h2")
    menu.addRadioStripItem("outputviews", "view_proxy", "View Proxy Geometry", hotkey=context + ".h3")
    menu.addSeparator()
    template.bindMenu(menu)
    return template
