"""The Box3D Configure SOP's viewer state (its ViewerStateModule section, read by make_configure_sop.py).

Draws what RBD Configure's state does while Visualize shows Bounce, Density, Friction or Rotational Stiffness: a
histogram of the pieces' values (the asset's HISTOGRAM node) in the lower left of the viewport and their minimum and
maximum (the asset's DISPLAY node) as text in the upper left. This file runs inside Houdini only (`hou` and `kwargs`
come from the viewer state loader), not as a script.
"""

INFO_FONT_SIZE = 4
# Visualize menu entries with a histogram, and their titles (the menu's order is in make_configure_sop.py).
TITLES = {5: "Bounce", 6: "Density", 7: "Friction", 8: "Rotational Stiffness"}
# The viewport size the histogram's placement was made for (RBD Configure's).
REFERENCE_SIZE = (2687.0, 1308.0)


class ConfigureState(object):
    def __init__(self, state_name, scene_viewer):
        self.state_name = state_name
        self.scene_viewer = scene_viewer
        self.info = hou.TextDrawable(self.scene_viewer, "text info")
        self.histogram = hou.GeometryDrawable(self.scene_viewer, hou.drawableGeometryType.Face, "histogram")
        self.histogram.show(False)

    def onDraw(self, kwargs):
        node = kwargs.get("node")
        mode = node.evalParm("visualize")
        if mode not in TITLES or (not node.isDisplayFlagSet() and not node.isCurrent()):
            self.info.show(False)
            self.histogram.show(False)
            return

        self.histogram.setGeometry(node.node("HISTOGRAM").geometry())
        self.histogram.show(True)
        width, height = self.scene_viewer.contentSize()
        scale_x = width / REFERENCE_SIZE[0]
        scale_y = height / REFERENCE_SIZE[1]
        self.histogram.draw(kwargs["draw_handle"], {"screen_space": (0, -scale_y, scale_x, scale_y, 0.025, 0.025)})

        try:
            geo = node.node("DISPLAY").geometry()
            low = geo.attribValue("__vmin")
            high = geo.attribValue("__vmax")
        except (AttributeError, hou.OperationFailed):
            low = high = "unspecified"
        text = "<b>%s:</b><br /><b>Minimum:</b> %s<br /><b>Maximum:</b> %s<br />" % (TITLES[mode], low, high)

        self.info.draw(kwargs["draw_handle"], {
            "text": "<font size=%s>%s</font>" % (INFO_FONT_SIZE, text),
            "origin": hou.drawableTextOrigin.UpperLeft,
            "multi_line": True,
            "translate": (0.0, self.scene_viewer.curViewport().size()[3], 0.0),
            "rotate": (0, 0, 0),
            "scale": hou.Vector3(1.0, 1.0, 1.0),
            "margins": (15.0, -85.0),
            "highlight_mode": hou.drawableHighlightMode.MatteOverGlow,
            "glow_width": 2,
            "fade_factor": 1,
        })
        self.info.show(True)


def createViewerStateTemplate():
    """Mandatory entry point: the state template Houdini registers."""
    state_typename = kwargs["type"].definition().sections()["DefaultState"].contents()
    template = hou.ViewerStateTemplate(state_typename, "Box3D Configure", hou.sopNodeTypeCategory())
    template.bindFactory(ConfigureState)
    template.bindIcon(kwargs["type"].icon())
    return template
