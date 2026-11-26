"""The digital assets: the Box3D Packed Object's menus and overwrite, Box3D Configure, Box3D Constraint
Properties, and that no asset leaves subdata warnings."""

import math

import hou
from b3dtest import (
    box3d_packed_object,
    finish,
    new_dopnet,
    solver_for,
    Result,
    close,
    create,
    display,
    new_geo,
    packed_boxes,
    packed_positions,
    set_menu_token,
    set_parm,
    step,
)

from checks_sop_common import named_positions, play


def test_constraint_properties_scaling():
    """A setting scaled by an attribute, as RBD Constraint Properties does (Geometry Attribute Transfer). A Revolute
    line between anchors A and B, Stiffness 10 ticked. The pieces are A1, A2, A3 (parentpiece A, weight 1, 3, 8, the
    anchor nearest A2) and B (weight 5). Scale by Attribute `weight` multiplies by the average of the two ends'
    pieces: A is its three pieces by Transfer Mode (Closest 3, Min 1, Average 4, Max 8) and B is 5, so 40, 30, 45 and 65.
    Scale by Value multiplies by the line's own stiffness attribute (2): 20; both: 10 * 2 * 4 = 80. Only the Weld's
    Linear and Angular Stiffness have Randomize and Variance (RBD Constraint Properties: Glue's Strength only); on a
    Weld line, Linear Stiffness 10 randomized with Variance 0 stays 10, and with 0.5 gives a value from 5 to 15."""
    geo = new_geo("cp_scaling")
    pieces = create(geo, "attribwrangle", "pieces")
    set_parm(pieces, ["class"], 0)
    set_parm(pieces, ["snippet"], 'addpointattrib(0, "name", ""); addpointattrib(0, "parentpiece", ""); '
             'addpointattrib(0, "weight", 0.0); '
             'string names[] = {"A1", "A2", "A3", "B"}; string parents[] = {"A", "A", "A", ""}; '
             'float weights[] = {1.0, 3.0, 8.0, 5.0}; float xs[] = {-1.0, 1.0, 3.0, 10.0}; '
             'for (int i = 0; i < 4; i++) { int pt = addpoint(0, set(xs[i], 0, 0)); '
             'setpointattrib(0, "name", pt, names[i]); setpointattrib(0, "parentpiece", pt, parents[i]); '
             'setpointattrib(0, "weight", pt, weights[i]); }')
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'addprimattrib(0, "stiffness", 0.0); int a = addpoint(0, set(0.9, 0, 0)); int b = addpoint(0, set(10, 0, 0)); '
             'setpointattrib(0, "name", a, "A"); setpointattrib(0, "name", b, "B"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'setprimattrib(0, "stiffness", pr, 2.0);')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, pieces)
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("doconstrainttype", 1), ("addrevolute_stiffness", 1),
                        ("revolute_stiffness", 10.0), ("revolute_stiffnessattribute", "weight")):
        set_parm(cp, [name], value)

    def stiffness(scaling, mode):
        set_parm(cp, ["revolute_stiffnessscaling"], scaling)
        set_parm(cp, ["transfermode"], mode)
        g = cp.geometry(1)
        return round(g.prims()[0].attribValue("stiffness"), 3) if g and g.findPrimAttrib("stiffness") else None

    got = {"none": stiffness(0, 0), "closest": stiffness(1, 0), "min": stiffness(1, 1), "average": stiffness(1, 2),
           "max": stiffness(1, 3), "value": stiffness(2, 0), "both": stiffness(3, 0)}
    want = {"none": 10.0, "closest": 40.0, "min": 30.0, "average": 45.0, "max": 65.0, "value": 20.0, "both": 80.0}
    ok = got == want
    results = [Result("constraint_properties_scaling", Result.PASS if ok else Result.FAIL,
                       f"stiffness by scaling / Transfer Mode {got} (expected {want})")]
    # Randomize and Variance are on the Weld's two stiffnesses only (RBD Constraint Properties: Glue's Strength only).
    parms = {p.name() for p in cp.parms()}
    names = sorted(n for n in parms if n.endswith(("randomize", "variance")))
    want_names = sorted(f"weld_{s}_stiffness{t}" for s in ("linear", "angular") for t in ("randomize", "variance"))
    # Weld's Linear Stiffness: Variance 0 changes nothing, Variance 0.5 gives 5 to 15 (and not exactly 10).
    weld = create(geo, "attribwrangle", "weld_lines")
    set_parm(weld, ["class"], 0)
    set_parm(weld, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); '
             'setpointattrib(0, "name", a, "A"); setpointattrib(0, "name", b, "B"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Weld");')
    wp = create(geo, "box3d_constraint_properties", "weld_props")
    wp.setInput(1, weld)
    for name, value in (("action", 0), ("doconstrainttype", 1), ("constrainttype", 0),
                        ("addweld_linear_stiffness", 1), ("weld_linear_stiffness", 10.0),
                        ("weld_linear_stiffnessrandomize", 1)):
        set_parm(wp, [name], value)
    spread = {}
    for variance in (0.0, 0.5):
        set_parm(wp, ["weld_linear_stiffnessvariance"], variance)
        g = wp.geometry(1)
        spread[variance] = round(g.prims()[0].attribValue("linear_stiffness"), 3) if g.findPrimAttrib(
            "linear_stiffness") else None
    good = (names == want_names and spread[0.0] == 10.0 and spread[0.5] is not None
            and 5.0 <= spread[0.5] <= 15.0 and spread[0.5] != 10.0)
    results.append(Result("constraint_properties_scaling", Result.PASS if good else Result.FAIL,
                          f"Randomize / Variance parameters {names} (expected {want_names}); Weld Linear Stiffness 10 "
                          f"with Variance 0 / 0.5: {spread} (expected 10, and 5 to 15 but not 10)"))
    return results


def test_constraint_properties_proxy_transfer():
    """Scaling by an attribute with a Proxy, as RBD Constraint Properties: the anchors name proxy pieces, and a proxy
    piece that is no Geometry piece stands for the Geometry pieces its parentpiece names. Geometry H1 (weight 2) and
    H2 (6) are parentpiece P1, H3 (10) is P2; proxy pieces px1 (P1, weight 100, never read) and px2 (P2) stand for them,
    and px3 has no parentpiece and weight 7, so it stands for itself. Stiffness 10 scaled by `weight`, on px1 to px2
    and on px1 to px3: Closest (the anchor at x=0.9 nearest H2) 80 and 65, Min 60 and 45, Max 80 and 65,
    Average 70 and 55."""
    geo = new_geo("cp_proxy_transfer")

    def pieces(name, rows):
        node = create(geo, "attribwrangle", name)
        set_parm(node, ["class"], 0)
        body = ('addpointattrib(0, "name", ""); addpointattrib(0, "parentpiece", ""); addpointattrib(0, "weight", 0.0); '
                'string names[] = {%s}; string parents[] = {%s}; float weights[] = {%s}; float xs[] = {%s}; '
                'for (int i = 0; i < len(names); i++) { int pt = addpoint(0, set(xs[i], 0, 0)); '
                'setpointattrib(0, "name", pt, names[i]); setpointattrib(0, "parentpiece", pt, parents[i]); '
                'setpointattrib(0, "weight", pt, weights[i]); }')
        quote = lambda items: ", ".join(f'"{i}"' for i in items)
        set_parm(node, ["snippet"], body % (quote([r[0] for r in rows]), quote([r[1] for r in rows]),
                                            ", ".join(str(float(r[2])) for r in rows),
                                            ", ".join(str(float(r[3])) for r in rows)))
        return node

    geometry = pieces("geometry", [("H1", "P1", 2, -1), ("H2", "P1", 6, 1), ("H3", "P2", 10, 10)])
    proxy = pieces("proxy", [("px1", "P1", 100, 0), ("px2", "P2", 100, 10), ("px3", "", 7, 20)])
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'string ends[] = {"px2", "px3"}; float xs[] = {10.0, 20.0}; '
             'for (int i = 0; i < 2; i++) { int a = addpoint(0, set(0.9, 0, 0)); int b = addpoint(0, set(xs[i], 0, 0)); '
             'setpointattrib(0, "name", a, "px1"); setpointattrib(0, "name", b, ends[i]); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute"); }')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, geometry)
    cp.setInput(1, lines)
    cp.setInput(2, proxy)
    for name, value in (("action", 0), ("doconstrainttype", 1), ("addrevolute_stiffness", 1),
                        ("revolute_stiffness", 10.0), ("revolute_stiffnessattribute", "weight"),
                        ("revolute_stiffnessscaling", 1)):
        set_parm(cp, [name], value)
    got = {}
    for mode, label in ((0, "closest"), (1, "min"), (3, "max"), (2, "average")):
        set_parm(cp, ["transfermode"], mode)
        g = cp.geometry(1)
        got[label] = ([round(p.attribValue("stiffness"), 3) for p in g.prims()]
                      if g and g.findPrimAttrib("stiffness") else None)
    want = {"closest": [80.0, 65.0], "min": [60.0, 45.0], "max": [80.0, 65.0], "average": [70.0, 55.0]}
    return [Result("constraint_properties_proxy_transfer", Result.PASS if got == want else Result.FAIL,
                   f"stiffness of px1-px2 and px1-px3 by Transfer Mode {got} (expected {want})")]


def test_constraint_properties_closest_surface():
    """Transfer Mode Closest measures to the pieces' surfaces, as RBD Constraint Properties does, not to their points.
    Packed pieces A1 (weight 1, a box of size 10 at the origin) and A2 (weight 3, a box of size 1 at x=8) share
    parentpiece A, and B (weight 5) is at x=20. A Revolute line from x=6 (anchor A) to x=20 (anchor B), Stiffness 10
    scaled by `weight`: A2's point is nearer the anchor (2 against 6) but A1's surface is (1 against 1.5), so the
    average of the ends is (1 + 5) / 2 * 10 = 30, not 40."""
    geo = new_geo("cp_closest_surface")
    packed = []
    for name, parent, weight, size, x in (("A1", "A", 1.0, 10, 0), ("A2", "A", 3.0, 1, 8), ("B", "", 5.0, 1, 20)):
        box = create(geo, "box", f"box_{name}")
        for axis in "xyz":
            set_parm(box, [f"size{axis}"], size)
        set_parm(box, ["tx"], x)
        attribs = create(geo, "attribwrangle", f"attribs_{name}")
        attribs.setInput(0, box)
        set_parm(attribs, ["class"], 2)
        set_parm(attribs, ["snippet"], f's@name = "{name}"; s@parentpiece = "{parent}"; f@weight = {weight};')
        pack = create(geo, "pack", f"pack_{name}")
        pack.setInput(0, attribs)
        set_parm(pack, ["nameattribute"], "name")
        set_parm(pack, ["transfer_attributes"], "name parentpiece weight")
        packed.append(pack)
    pieces = create(geo, "merge", "pieces")
    for i, pack in enumerate(packed):
        pieces.setInput(i, pack)
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(6, 0, 0)); int b = addpoint(0, set(20, 0, 0)); '
             'setpointattrib(0, "name", a, "A"); setpointattrib(0, "name", b, "B"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, pieces)
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("doconstrainttype", 1), ("addrevolute_stiffness", 1),
                        ("revolute_stiffness", 10.0), ("revolute_stiffnessattribute", "weight"),
                        ("revolute_stiffnessscaling", 1), ("transfermode", 0)):
        set_parm(cp, [name], value)
    g = cp.geometry(1)
    got = round(g.prims()[0].attribValue("stiffness"), 3) if g and g.findPrimAttrib("stiffness") else None
    return [Result("constraint_properties_closest_surface", Result.PASS if got == 30.0 else Result.FAIL,
                   f"Closest stiffness over packed pieces {got} (expected 30: the surface-nearest piece A1, not 40)")]


def test_constraint_properties_next():
    """Switch Constraint Type When Broken and the next settings. A Revolute line: with the switch off, Next Constraint
    Name and a next Revolute Stiffness (99) write nothing; with it on, Edit writes next_constraint_name Weld and
    next_stiffness 99, and Set writes the next name and the next settings that differ from the defaults."""
    geo = new_geo("cp_next")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    for name, value in (("action", 0), ("donext_constraintname", 1), ("next_constraintname", "Weld"),
                        ("addnext_revolute_stiffness", 1), ("next_revolute_stiffness", 99.0),
                        ("next_constrainttype", 2), ("donext_constrainttype", 1)):
        set_parm(cp, [name], value)

    def attrs():
        g = cp.geometry(1)
        prim = g.prims()[0]
        return {a: prim.attribValue(a) for a in ("next_constraint_name", "next_stiffness")
                if g.findPrimAttrib(a) is not None}

    off = attrs()
    set_parm(cp, ["usenextconstraint"], 1)  # Enable, but its tick (dousenextconstraint) is off in Edit mode
    unticked = attrs()
    set_parm(cp, ["dousenextconstraint"], 1)
    on = attrs()
    set_parm(cp, ["donext_constrainttype"], 0)  # the next type's settings need its tick too
    no_type = attrs()
    set_parm(cp, ["donext_constrainttype"], 1)
    set_parm(cp, ["action"], 1)
    in_set = attrs()
    want_on = {"next_constraint_name": "Weld", "next_stiffness": 99.0}
    ok = ("next_constraint_name" not in off and "next_stiffness" not in off and unticked == {}
          and no_type == {"next_constraint_name": "Weld"} and on == want_on
          and in_set.get("next_constraint_name") == "Weld" and in_set.get("next_stiffness") == 99.0)
    return [Result("constraint_properties_next", Result.PASS if ok else Result.FAIL,
                   f"line attributes with the switch off {off}, Enable but unticked {unticked}, ticked in Edit {on}, "
                   f"without the next type's tick {no_type}, in Set {in_set} (expected nothing, nothing, {want_on}, "
                   f"the name only, then the next name and stiffness 99)")]


def test_constraint_properties_vexpressions():
    """Use VEXpressions on a type's folder, as RBD Constraint Properties has it: with Revolute picked in Constraint Type
    and its toggle on, the snippet runs over the constraints in the group after the settings (it can read the
    Geometry as its second input: a piece's `weight` 4 here), so the line gets custom 5.5 and fromgeo 4. With the
    toggle off, or another type picked, it does not run."""
    geo = new_geo("cp_vex")
    pieces = create(geo, "attribwrangle", "pieces")
    set_parm(pieces, ["class"], 0)
    set_parm(pieces, ["snippet"], 'addpointattrib(0, "name", ""); addpointattrib(0, "weight", 0.0); '
             'int pt = addpoint(0, set(0, 0, 0)); setpointattrib(0, "name", pt, "A"); setpointattrib(0, "weight", pt, 4.0);')
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); setpointattrib(0, "name", a, "A"); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, pieces)
    cp.setInput(1, lines)
    set_parm(cp, ["constrainttype"], 2)
    set_parm(cp, ["revolute_localexpression"], 'f@custom = 5.5; f@fromgeo = point(1, "weight", 0);')

    def attrs():
        g = cp.geometry(1)
        return {a: g.prims()[0].attribValue(a) for a in ("custom", "fromgeo") if g.findPrimAttrib(a) is not None}

    off = attrs()
    set_parm(cp, ["revolute_uselocalproperties"], 1)
    on = attrs()
    set_parm(cp, ["constrainttype"], 0)
    other = attrs()
    ok = off == {} and on == {"custom": 5.5, "fromgeo": 4.0} and other == {}
    return [Result("constraint_properties_vexpressions", Result.PASS if ok else Result.FAIL,
                   f"line attributes with the toggle off {off}, on {on}, with Weld picked instead {other} (expected "
                   f"nothing, then custom 5.5 and fromgeo 4, then nothing)")]


def test_constraint_properties_collisions():
    """Disable Collisions (Collisions: Disable / Enable), in RBD Constraint Properties' words: ticked in Edit it
    writes the primitive attribute collide_connected, 0 for Disable and 1 for Enable; unticked it writes nothing."""
    geo = new_geo("cp_collisions")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addpointattrib(0, "name", ""); addprimattrib(0, "constraint_name", ""); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute");')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    set_parm(cp, ["action"], 0)

    def collide():
        g = cp.geometry(1)
        return g.prims()[0].attribValue("collide_connected") if g.findPrimAttrib("collide_connected") else None

    off = collide()
    set_parm(cp, ["docollisions"], 1)
    set_parm(cp, ["collisions"], 0)
    disabled = collide()
    set_parm(cp, ["collisions"], 1)
    enabled = collide()
    ok = off is None and disabled == 0 and enabled == 1
    return [Result("constraint_properties_collisions", Result.PASS if ok else Result.FAIL,
                   f"collide_connected unticked {off}, Disable {disabled}, Enable {enabled} (expected nothing, 0, 1)")]


def test_configure_material_preset():
    """Material Preset on a Box3D Configure entry, with RBD Configure's presets (the table is tools/b3d_materials.py):
    Metal and Steel fill Density 8050, Bounce 0.5 and Friction 0.45 (dry and clean surfaces) and tick them; the
    lubricated and greasy State gives friction 0.15; Glass resets the Material to Default (density 1000, bounce 0.2,
    friction 1). The pieces carry them."""
    geo = new_geo("cfg_material")
    merge = create(geo, "merge", "boxes")
    for i in range(2):
        box = create(geo, "box", f"box{i}")
        set_parm(box, ["t"], (3.0 * i, 3.0, 0.0))
        merge.setInput(i, box)
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, merge)
    module = cfg.hm()
    kwargs = {"node": cfg, "script_multiparm_index": 1}

    def values():
        return {n: cfg.parm(f"{n}1").eval() for n in ("density", "bounce", "friction")}, cfg.parm("adddensity1").eval()

    set_parm(cfg, ["materialtype1"], 4)  # Metal (the menu's 0 is None)
    items = module.buildMaterialMenu(kwargs)
    steel = items[items.index("Steel") - 1] if "Steel" in items else None
    set_parm(cfg, ["materialpreset1"], int(steel) if steel is not None else 0)
    module.materialPresetCallback(kwargs)
    dry, ticked = values()
    set_parm(cfg, ["materialstate1"], 1)
    module.materialPresetCallback(kwargs)
    greasy, _ = values()
    set_parm(cfg, ["materialstate1"], 0)  # dry and clean again (a type change keeps the State)
    set_parm(cfg, ["materialtype1"], 2)  # Glass
    module.typeChanged(kwargs)
    glass, _ = values()
    g = cfg.geometry()
    attrs = {n: g.points()[0].attribValue(n) for n in ("density", "bounce", "friction") if g.findPointAttrib(n)}
    ok = (steel is not None and dry == {"density": 8050.0, "bounce": 0.5, "friction": 0.45} and ticked == 1
          and greasy["friction"] == 0.15 and glass == {"density": 1000.0, "bounce": 0.2, "friction": 1.0}
          and attrs.get("density") == 1000.0 and attrs.get("friction") == 1.0)
    return [Result("configure_material_preset", Result.PASS if ok else Result.FAIL,
                   f"Metal / Steel {dry} (ticked {ticked}), greasy {greasy['friction']}, Glass {glass}, point attributes "
                   f"{attrs} (expected 8050 / 0.5 / 0.45, 0.15, 1000 / 0.2 / 1, 1000 and 1)")]


def test_configure_var():
    """Var on Box3D Configure's Density, Bounce, Friction and Rotational Stiffness, against RBD Configure's (its User
    value with Var): six pieces get the same value each, the base times a random 1 - Var to 1 + Var by point
    number, and with Var off every piece has the base."""
    pieces, _ = packed_boxes("cfg_var_pieces", [{"size": (1, 1, 1), "center": (3.0 * i, 1, 0)} for i in range(6)])
    base = {"density": (500.0, 0.5), "bounce": (0.4, 0.5), "friction": (0.7, 0.3), "inertialtensorstiffness": (2.0, 0.3)}
    got = {}
    for label in ("box3d", "rbd"):
        geo = new_geo(f"cfg_var_{label}")
        src = create(geo, "object_merge", "pieces")
        set_parm(src, ["objpath1"], pieces)
        cfg = create(geo, "box3d_configure" if label == "box3d" else "rbdconfigure", "configure")
        cfg.setInput(0, src)
        for name, (value, var) in base.items():
            if label == "box3d":
                set_parm(cfg, [f"add{name}1"], 1)
                set_parm(cfg, [f"{name}1"], value)
                set_parm(cfg, [f"use{name}var1"], 1)
                set_parm(cfg, [f"{name}var1"], var)
            elif name == "inertialtensorstiffness":
                set_parm(cfg, ["addrotationalstiffness1"], 1)
                set_parm(cfg, [f"{name}1"], value)
                set_parm(cfg, [f"use{name}var1"], 1)
                set_parm(cfg, [f"{name}var1"], var)
            else:
                set_parm(cfg, [f"add{name}1"], 1)
                set_parm(cfg, [f"adduser{name}1"], 1)
                set_parm(cfg, [f"user{name}1"], value)
                set_parm(cfg, [f"useuser{name}var1"], 1)
                set_parm(cfg, [f"user{name}var1"], var)
        display(cfg)
        g = cfg.geometry(0)
        got[label] = {name: [round(pt.attribValue(name), 4) for pt in g.points()] for name in base
                      if g.findPointAttrib(name)}
    # Var off: the plain base on every piece
    geo = new_geo("cfg_var_off")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, src)
    set_parm(cfg, ["adddensity1"], 1)
    set_parm(cfg, ["density1"], 500.0)
    display(cfg)
    flat = {round(pt.attribValue("density"), 4) for pt in cfg.geometry(0).points()}
    scattered = len(set(got["box3d"].get("density", []))) > 1
    ok = got["box3d"] == got["rbd"] and scattered and flat == {500.0}
    return [Result("configure_var", Result.PASS if ok else Result.FAIL,
                   f"Box3D {got['box3d']}, RBD Configure {got['rbd']}, Var off {sorted(flat)} "
                   f"(expected the same values, scattered, and 500 with Var off)")]


def test_configure_packs():
    """Box3D Configure packs unpacked input as RBD Configure does: two polygon boxes with no name come
    out as two packed pieces named on their points, with the ticked setting on the group's piece; the
    result simulates in the Box3D Solver SOP (both pieces fall)."""
    geo = new_geo("cfg_pack")
    merge = create(geo, "merge", "boxes")
    for i in range(2):
        box = create(geo, "box", f"box{i}")
        set_parm(box, ["t"], (3.0 * i, 3.0, 0.0))
        merge.setInput(i, box)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, merge)
    set_parm(grp, ["class"], "primitive")
    set_parm(grp, ["snippet"], 'if (@P.x < 1.5) setprimgroup(0, "chosen", @primnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addfriction1", 1), ("friction1", 0.25)):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    packed = [p for p in g.prims() if "Packed" in p.type().name()] if g else []
    named = g is not None and g.findPointAttrib("name") is not None and g.findPrimAttrib("name") is None
    frictions = sorted(round(p.attribValue("friction"), 4) for p in g.points()) \
        if g is not None and g.findPointAttrib("friction") else []
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    low = play(solver, 24, lambda s, f: s.geometry(0).boundingBox().maxvec()[1] if f == 24 else None)[24]
    ok = len(packed) == 2 and len(g.prims()) == 2 and named and len(frictions) == 2 and \
        frictions[0] == 0.25 and frictions[1] != 0.25 and low is not None and low < 2.0
    return [Result("configure_packs", Result.PASS if ok else Result.FAIL,
                   f"{len(packed)} packed of {len(g.prims()) if g else 0} primitives (expected 2 of 2), name on "
                   f"points only: {named}, friction per piece {frictions} (expected one 0.25); top of the "
                   f"pieces after 1 s: {low} (expected below 2: fell)")]


def test_configure():
    """Box3D Configure (tools/make_configure_sop.py) writes ticked settings onto the pieces in its
    group only: Body Type Static and Friction 0.25 on one of two boxes. Into the Box3D Solver SOP,
    that box stays put and the other falls (it keeps active 1, not the 0 VEX would give it)."""
    pieces, names = packed_boxes("cfg_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")  # the Group field takes a group name, as RBD Configure's
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    # Multiparm entry 1 (Number of Attributes): group#, add<setting># checkbox, <setting>#.
    for n, v in (("group1", "chosen"), ("addbodytype1", 1), ("bodytype1", 0), ("addfriction1", 1),
                 ("friction1", 0.25)):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    attrs = {p.attribValue("name"): (p.attribValue("active"), round(p.attribValue("friction"), 4)) for p in g.points()}
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    pos = play(solver, 24, lambda s, f: named_positions(s.geometry(0)) if f == 24 else None)[24]
    y_static, y_free = pos[names[0]][1], pos[names[1]][1]
    ok = (attrs.get(names[0], (None, None)) == (0, 0.25) and attrs.get(names[1], (None,))[0] == 1
          and close(y_static, 3.0, 1e-3) and y_free < 2.0)
    return [Result("configure", Result.PASS if ok else Result.FAIL,
                   f"(active, friction) per piece {attrs} (expected {names[0]}: (0, 0.25), {names[1]}: active 1); "
                   f"after 1 s the static box is at y={y_static:.3f} (expected 3), the other at {y_free:.3f} "
                   f"(expected below 2: fell)")]


def test_configure_group_patterns():
    """Box3D Configure's Group takes patterns as RBD Configure's does, not only group names: `@name=<piece>`,
    a wildcard over group names (`chos*`) and a point number each pick the first of two pieces, which
    alone gets Friction 0.25."""
    pieces, names = packed_boxes("cfg_pat_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg_pat")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen_a", @ptnum, 1);')
    rows, ok = [], True
    for label, pattern in (("name", f"@name={names[0]}"), ("wildcard", "chos*"), ("number", "0")):
        cfg = create(geo, "box3d_configure", f"configure_{label}")
        cfg.setInput(0, grp)
        for n, v in (("group1", pattern), ("addfriction1", 1), ("friction1", 0.25)):
            set_parm(cfg, [n], v)
        g = cfg.geometry()
        got = {p.attribValue("name"): round(p.attribValue("friction"), 4) for p in g.points()} \
            if g is not None and g.findPointAttrib("friction") else {}
        good = got.get(names[0]) == 0.25 and got.get(names[1]) not in (None, 0.25)
        rows.append(f"{label} ({pattern!r}): {got}")
        ok = ok and good
    return [Result("configure_group_patterns", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + f" (expected 0.25 on {names[0]} only)")]


def test_constraint_properties():
    """Box3D Constraint Properties (tools/make_constraint_properties_sop.py) writes ticked settings onto the
    constraints in its group only: on two lines, one in a group, Revolute Stiffness 7 and Enable Spring land
    on the first, the other keeps the "not set" values the solver skips."""
    geo = new_geo("conprops")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], "detail")
    set_parm(lines, ["snippet"],
             'for (int i = 0; i < 2; i++) { int a = addpoint(0, set(i, 0, 0)), b = addpoint(0, set(i, 1, 0)); '
             'int pr = addprim(0, "polyline", a, b); '
             'if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", ""); '
             'setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'if (i == 0) setprimgroup(0, "chosen", pr, 1); }')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    for n, v in (("action", 0), ("constraintgroup", "chosen"), ("constrainttype", 2), ("addrevolute_stiffness", 1),
                 ("revolute_stiffness", 7.0), ("addrevolute_enable_spring", 1), ("revolute_enable_spring", 1)):
        set_parm(cp, [n], v)
    # Edit mode writes a type's ticked settings only once Constraint Type is ticked
    unticked = cp.geometry(1).findPrimAttrib("stiffness") is None
    set_parm(cp, ["doconstrainttype"], 1)
    g = cp.geometry(1)
    prims = g.prims()
    first = (round(prims[0].attribValue("stiffness"), 3), prims[0].attribValue("enable_spring"))
    other = (prims[1].attribValue("stiffness"), prims[1].attribValue("enable_spring"))
    ok = (unticked and first == (7.0, 1) and other[0] <= -3.0e38 and other[1] < 0
          and g.findPrimGroup("__tmpconpropgrp") is None)
    return [Result("constraint_properties", Result.PASS if ok else Result.FAIL,
                   f"(stiffness, enable_spring) on the group's line {first} (expected (7, 1)), on the other {other} "
                   "(expected not set: below -3e38 and -1)")]


def test_constraint_properties_edit():
    """Box3D Constraint Properties' viewport edits (tools/b3d_constraint_properties_state.py): three lines, two in
    a group, the stash rotating the anchors of the selected one (what the handle writes) lands as `orient` on that
    line's two points only; Set mode writes every Revolute setting and the name, without any checkbox; the output has
    no `__idx`; the state's sections and the toolbar parameters are in the asset."""
    geo = new_geo("conprops_edit")
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], "detail")
    set_parm(lines, ["snippet"],
             'for (int i = 0; i < 3; i++) { int a = addpoint(0, set(i, 0, 0)), b = addpoint(0, set(i, 1, 0)); '
             'int pr = addprim(0, "polyline", a, b); '
             'if (!hasprimattrib(0, "constraint_name")) addprimattrib(0, "constraint_name", ""); '
             'setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'if (i < 2) setprimgroup(0, "chosen", pr, 1); }')
    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(1, lines)
    set_parm(cp, ["constraintgroup"], "chosen")
    set_parm(cp, ["__sel"], "@__idx=1")
    set_parm(cp, ["__idx"], 1)
    module = cp.type().hdaViewerStateModule()
    quat = module.quaternion_from_euler(0.0, 0.0, 90.0)
    module.set_edits(cp, {i: (quat, quat) for i in module.selected_ids(cp)})
    g = cp.geometry(1)
    orients = [tuple(round(x, 3) for x in p.attribValue("orient")) for p in g.points()]         if g.findPointAttrib("orient") else []
    turned = (0.0, 0.0, 0.707, 0.707)
    ok = (module.selected_ids(cp) == [1] and orients[2:4] == [turned, turned]
          and all(o == (0.0, 0.0, 0.0, 1.0) for o in orients[:2] + orients[4:])
          and g.findPrimAttrib("__idx") is None)
    detail = f"orient per point {orients} (expected only points 2 and 3 turned 90 degrees about Z)"
    # Toggle Motor and the wheel's motor velocity, on the two lines in the group: edited ones only, the stash keeps
    # the orient of a line whose motor is edited later.
    set_parm(cp, ["__sel"], "@__idx=0 @__idx=1")
    ids = module.selected_ids(cp)
    module.set_edits(cp, {i: (quat, quat) for i in ids})
    module.toggle_motor(cp, ids)
    module.add_motor_velocity(cp, ids, 2, 10.0)
    g = cp.geometry(1)
    motor = [p.attribValue("enable_motor") for p in g.prims()]
    velocity = [round(p.attribValue("motor_velocity")[2], 3) for p in g.prims()]
    kept = [tuple(round(x, 3) for x in p.attribValue("orient")) for p in g.points()][:4]
    ok = ok and motor == [1, 1, -1] and velocity[:2] == [10.0, 10.0] and kept == [turned] * 4
    detail += f"; enable_motor {motor}, motor_velocity z {velocity}, orient kept {kept}"
    module.clearEdits({"node": cp})
    set_parm(cp, ["action"], 1)
    set_parm(cp, ["constrainttype"], 2)
    set_parm(cp, ["revolute_stiffness"], 5.0)
    set_parm(cp, ["constraintname"], "Weld")
    g = cp.geometry(1)
    stiff = [round(p.attribValue("stiffness"), 3) for p in g.prims()] if g.findPrimAttrib("stiffness") else []
    names = [p.attribValue("constraint_name") for p in g.prims()]
    ok = ok and stiff[:2] == [5.0, 5.0] and stiff[2] <= -3.0e38 and names == ["Weld", "Weld", "Revolute"]
    detail += f"; Set mode stiffness {stiff}, names {names}"
    definition = cp.type().definition()
    sections = sorted(n for n in definition.sections() if n.startswith("ViewerState"))
    toolbar = sorted(pt.name() for pt in cp.parmTemplateGroup().entriesWithoutFolders() if pt._exportLevel() == 3)
    # A setting a toggle switches on greys out with the toggle off in both modes: Edit once the toggle is being
    # written, Set (checkboxes hidden) by its value.
    cond = cp.parm("weld_linear_stiffness").parmTemplate().conditionals().get(hou.parmCondType.DisableWhen, "")
    greyed = ("{ weld_enable_linear_spring == 0 addweld_enable_linear_spring == 1 action == 0 }" in cond
              and "{ weld_enable_linear_spring == 0 action == 1 }" in cond)
    ok = ok and greyed and len(sections) == 4 and toolbar == ["clearedits", "editparms", "importedits"]
    detail += f"; weld_linear_stiffness disable-when {cond!r}"
    detail += f"; sections {sections}, toolbar {toolbar}"
    return [Result("constraint_properties_edit", Result.PASS if ok else Result.FAIL, detail)]


def test_no_subdata_warnings():
    """Box3D Data under an object's Geometry raises no data hint warnings ("Unexpected subdata found",
    "attached to an unexpected parent data"): a Box3D Packed Object cooked on two frames, and the Box3D Solver
    SOP cooking a piece, and a Packed Object and an RBD Object stepped by the Box3D Solver DOP, have no warnings on
    the Geometry or the data node."""
    pieces, names = packed_boxes("hint_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    dop = hou.node("/obj").createNode("dopnet", "hint_dop")
    po = dop.createNode("box3d_packedobject", "po")
    set_parm(po, ["soppath"], pieces)
    po.setDisplayFlag(True)
    found = []
    for frame in (1, 2):
        hou.setFrame(frame)
        dop.cook(force=True)
        found += [f"{n.path()}: {w}" for n in [po] + list(po.allSubChildren()) for w in n.warnings()
                  if "subdata" in w or "unexpected parent" in w]
    # The Box3D Solver DOP writes the Geometry back as a SIM_GeometryCopy, an RBD Object's stays a SIM_SopGeometry.
    box = create(new_geo("hint_box"), "box", "box")
    for label, node_type, sop in (("packed", "box3d_packedobject", pieces), ("rbd", "rbdobject", box.path())):
        sim_dop = hou.node("/obj").createNode("dopnet", f"hint_{label}_dop")
        obj = sim_dop.createNode(node_type, "obj")
        set_parm(obj, ["soppath"], sop)
        data = sim_dop.createNode("box3d_data", "data")
        data.setInput(0, obj)
        sim_solver = sim_dop.createNode("box3d_solver", "solver")
        sim_solver.setInput(0, data)
        sim_solver.setDisplayFlag(True)
        for frame in (1, 2, 3):
            hou.setFrame(frame)
            sim_dop.cook(force=True)
        found += [f"{n.path()}: {w}" for n in [sim_dop] + list(sim_dop.allSubChildren()) for w in n.warnings()
                  if "subdata" in w or "unexpected parent" in w]
    geo = new_geo("hint_solver")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, src)
    hou.setFrame(2)
    solver.cook(force=True)
    found += [f"{n.path()}: {w}" for n in [solver] + list(solver.allSubChildren()) for w in n.warnings()
              if "subdata" in w or "unexpected parent" in w]
    return [Result("no_subdata_warnings", Result.PASS if not found else Result.FAIL,
                   f"data hint warnings: {found or 'none'} (expected none)")]


def test_packed_object_geometry_source():
    """The Box3D Packed Object's Geometry Source, as RBD Packed Object's: First and Second Context Geometry take the
    pieces wired into the DOP network's first and second input (the SOP Path left empty), SOP Path is greyed out
    for them, and a piece dropped from y = 3 has fallen after 1 s."""
    pieces, _ = packed_boxes("geosrc_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    rows = []
    ok = True
    for token, wire in (("first", 0), ("second", 1)):
        geo = new_geo(f"geosrc_{token}")
        src = create(geo, "object_merge", "pieces")
        set_parm(src, ["objpath1"], pieces)
        net = create(geo, "dopnet", "dop")
        net.setInput(wire, src)
        po = create(net, "box3d_packedobject", "po")
        set_menu_token(po, ["geosource"], token)
        solver = create(net, "box3d_solver", "solver")
        solver.setInput(0, po)
        out = create(net, "output", "output")
        out.setInput(0, solver)
        display(out)
        track = step(net, 25, lambda sim, f: packed_positions(sim, "po"))
        pos = track[25]
        y = next(iter(pos.values())).y() if pos else None
        # Houdini evaluates the condition in the UI (Parm.isDisabled does not): it is on SOP Path, and the menu is not on SOP.
        cond = po.parm("soppath").parmTemplate().conditionals().get(hou.parmCondType.DisableWhen, "")
        greyed = cond == "{ geosource != sop }" and po.parm("geosource").evalAsString() != "sop"
        rows.append(f"{token}: y {y} after 1 s, SOP Path greyed {greyed}")
        ok = ok and y is not None and y < 2.0 and greyed
    return [Result("packed_object_geometry_source", Result.PASS if ok else Result.FAIL,
                   "; ".join(rows) + " (expected y below 2 and SOP Path greyed out)")]


def test_configure_initial_state():
    """Box3D Configure's Initial State: Velocity (2, 0, 0) moves the group's piece 2 in 1 s through the Box3D
    Solver SOP (no gravity needed in x), and Angular Velocity (0, 90, 0) degrees per second is written
    as w in radians (pi / 2)."""
    pieces, names = packed_boxes("cfg_vel_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (0, 3, 5)}])
    geo = new_geo("cfg_vel")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addv1", 1), ("v1", (2.0, 0.0, 0.0)), ("addw1", 1), ("w1", (0.0, 90.0, 0.0))):
        set_parm(cfg, [n], v)
    g = cfg.geometry()
    w = {p.attribValue("name"): p.attribValue("w") for p in g.points()} if g is not None and g.findPointAttrib("w") else {}
    w_ok = names[0] in w and close(w[names[0]][1], math.pi / 2, 1e-4)
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    pos = play(solver, 25, lambda s, f: named_positions(s.geometry(0)) if f == 25 else None)[25]
    x0, x1 = pos[names[0]][0], pos[names[1]][0]
    ok = w_ok and close(x0, 2.0, 0.1) and close(x1, 0.0, 0.05)
    return [Result("configure_initial_state", Result.PASS if ok else Result.FAIL,
                   f"w on the piece {w.get(names[0])} (expected y = {math.pi / 2:.4f}); x after 1 s: grouped {x0:.3f} "
                   f"(expected 2), other {x1:.3f} (expected 0)")]


def test_configure_speed_limit():
    """Box3D Configure's Speed Limit folder (RBD Configure's): Speed Max 3 and Acceleration Max 5 on the group's piece
    write speedmax and accelmax on its point only (the other piece gets none, or -1, no limit). Through the Box3D
    Solver SOP the group's piece falls at about 3 m/s after 2 s and the other freely (about 19.6)."""
    pieces, names = packed_boxes("cfg_limit_pieces", [{"size": (1, 1, 1), "center": (0, 100, 0)},
                                                      {"size": (1, 1, 1), "center": (0, 100, 5)}])
    geo = new_geo("cfg_limit")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addspeedmax1", 1), ("speedmax1", 3.0), ("addaccelmax1", 1),
                 ("accelmax1", 5.0)):
        set_parm(cfg, [n], v)
    g = cfg.geometry()

    def attr(name):
        if g is None or g.findPointAttrib(name) is None:
            return {}
        return {p.attribValue("name"): p.attribValue(name) for p in g.points()}

    speed, accel = attr("speedmax"), attr("accelmax")
    written = (speed.get(names[0]) == 3.0 and accel.get(names[0]) == 5.0 and speed.get(names[1], -1.0) < 0.0
               and accel.get(names[1], -1.0) < 0.0)
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg)
    display(solver)
    pos = play(solver, 49, lambda s, f: named_positions(s.geometry(0)) if f in (48, 49) else None)
    fall = {n: (pos[48][n][1] - pos[49][n][1]) * 24.0 for n in names}
    ok = written and close(fall[names[0]], 3.0, 0.3) and fall[names[1]] > 15.0
    return [Result("configure_speed_limit", Result.PASS if ok else Result.FAIL,
                   f"speedmax {speed.get(names[0])} and accelmax {accel.get(names[0])} on the group's piece, "
                   f"{speed.get(names[1])} and {accel.get(names[1])} on the other (expected 3, 5 and none); falling at "
                   f"{fall[names[0]]:.2f} and {fall[names[1]]:.2f} m/s after 2 s (expected about 3, and free)")]


def test_configure_pin():
    """Box3D Configure's Pin adds one Weld line from the world to each pinned piece on the Constraint Geometry
    output. Wired into the Box3D Solver SOP, the pinned piece stays at y = 3 and the other falls."""
    pieces, names = packed_boxes("cfg_pin_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                    {"size": (1, 1, 1), "center": (3, 3, 0)}])
    geo = new_geo("cfg_pin")
    src = create(geo, "object_merge", "pieces")
    set_parm(src, ["objpath1"], pieces)
    grp = create(geo, "attribwrangle", "chosen")
    grp.setInput(0, src)
    set_parm(grp, ["snippet"], f'if (s@name == "{names[0]}") setpointgroup(0, "chosen", @ptnum, 1);')
    cfg = create(geo, "box3d_configure", "configure")
    cfg.setInput(0, grp)
    for n, v in (("group1", "chosen"), ("addpin1", 1), ("pin1", 1), ("pintype1", 0)):
        set_parm(cfg, [n], v)
    lines = cfg.geometry(1)
    kinds = [p.attribValue("constraint_name") for p in lines.prims()] if lines is not None \
        and lines.findPrimAttrib("constraint_name") else None
    solver = create(geo, "box3d_solver", "box3d_solver")
    solver.setInput(0, cfg, 0)
    solver.setInput(1, cfg, 1)
    display(solver)
    pos = play(solver, 24, lambda s, f: named_positions(s.geometry(0)) if f == 24 else None)[24]
    y_pinned, y_free = pos[names[0]][1], pos[names[1]][1]
    ok = kinds == ["Weld"] and close(y_pinned, 3.0, 0.02) and y_free < 2.0
    return [Result("configure_pin", Result.PASS if ok else Result.FAIL,
                   f"constraint lines {kinds} (expected one 'Weld'); after 1 s the pinned piece is at y={y_pinned:.3f} "
                   f"(expected 3), the other at {y_free:.3f} (expected below 2: fell)")]


# ---------------------------------------------------------------------------
# Packed Object: Overwrite Attributes from SOP
# ---------------------------------------------------------------------------


def overwrite_fall(overwrite, frames=30):
    """A box falling from y 20 whose SOP sets `active` to 0 from frame 13: its height on frames 12, 13 and
    `frames`, and what the Packed Object's own node reports."""
    path, names = packed_boxes(f"overwrite_{overwrite}_pieces",
                               [{"size": (1, 1, 1), "center": (0, 20, 0), "attrs": {"active": 1}}])
    sop = hou.node(path)
    wr = create(sop.parent(), "attribwrangle", "switch")
    wr.setInput(0, sop)
    set_parm(wr, ["class"], 2)
    set_parm(wr, ["snippet"], "i@active = @Frame < 13 ? 1 : 0;")
    display(wr)
    dop = new_dopnet(f"overwrite_{overwrite}")
    obj = box3d_packed_object(dop, "pieces", wr.path(), overwritefromsop=overwrite,
                              overwriteattributes="active")
    finish(dop, solver_for("box3d", dop, obj))
    ys = step(dop, frames, lambda sim, f: next(iter(packed_positions(sim, "pieces").values()))[1])
    return ys[12], ys[13], ys[frames]


def test_packed_object_overwrite_from_sop():
    """Overwrite Attributes from SOP on the Box3D Packed Object: with it on and `active` in the list, the SOP's
    `active` (1, then 0 from frame 13) reaches the solver, so the box, after falling for 12 frames, stays where it is. With it off
    the SOP's later values are ignored and the box keeps falling."""
    on, off = overwrite_fall(1), overwrite_fall(0)
    held = on[0] < 19.5 and abs(on[2] - on[1]) < 0.01 and abs(on[1] - on[0]) < 0.01
    fell = off[2] < off[1] - 1.0
    return [Result("packed_object_overwrite_from_sop", Result.PASS if held and fell else Result.FAIL,
                   f"box height on frames 12, 13 and 30: on {on[0]:.3f} {on[1]:.3f} {on[2]:.3f} (expected it to have fallen, then frozen), off "
                   f"{off[0]:.3f} {off[1]:.3f} {off[2]:.3f} (expected still falling)")]


# ---------------------------------------------------------------------------
# The group / attribute menus beside the fields (MENU_SOURCE in tools/b3d_hda.py)
# ---------------------------------------------------------------------------


def menu_tokens(parm):
    return list(parm.menuItems())


def test_asset_menus():
    """Every parameter with a group / attribute menu script: it builds without an error on connected geometry,
    and lists what the geometry holds (point and primitive groups and attributes; the SOP a Packed Object
    names; Constraint Properties' second input)."""
    geo = new_geo("menus")
    path, names = packed_boxes("menus_pieces", [{"size": (1, 1, 1), "center": (0, 0, 0)},
                                                {"size": (1, 1, 1), "center": (3, 0, 0)}])
    src = hou.node(path)
    tag = create(src.parent(), "attribwrangle", "tag")
    tag.setInput(0, src)
    set_parm(tag, ["class"], 2)
    set_parm(tag, ["snippet"], 'f@myattr = 1; setpointgroup(0, "grpA", @ptnum, 1);')
    tag2 = create(src.parent(), "attribwrangle", "tag2")
    tag2.setInput(0, tag)
    set_parm(tag2, ["class"], 1)
    set_parm(tag2, ["snippet"], 'f@pa = 1; setprimgroup(0, "primB", @primnum, 1);')
    display(tag2)
    lines = create(geo, "attribwrangle", "lines")
    set_parm(lines, ["class"], 0)
    set_parm(lines, ["snippet"], 'addprimattrib(0, "constraint_name", ""); addprimattrib(0, "stiff", 0.0); '
             'int a = addpoint(0, set(0, 0, 0)); int b = addpoint(0, set(1, 0, 0)); '
             'int pr = addprim(0, "polyline", a, b); setprimattrib(0, "constraint_name", pr, "Revolute"); '
             'setprimattrib(0, "stiff", pr, 1.0); setprimgroup(0, "lineGrp", pr, 1);')
    results = []
    problems = []

    def check(label, parm, want):
        try:
            tokens = menu_tokens(parm)
        except Exception as e:  # noqa: BLE001
            problems.append(f"{label}: menu failed ({e})")
            return
        for item in want:
            if item not in tokens:
                problems.append(f"{label}: {item} missing from {tokens}")

    cfg = create(geo, "box3d_configure", "configure")
    cfg_in = create(geo, "object_merge", "cfg_in")
    set_parm(cfg_in, ["objpath1"], tag2.path())
    set_parm(cfg_in, ["xformtype"], 0)
    cfg.setInput(0, cfg_in)
    check("Configure transfer_attributes", cfg.parm("transfer_attributes"), ["myattr", "age", "v", "w"])
    check("Configure transfer_renderattributes", cfg.parm("transfer_renderattributes"), ["myattr", "pa"])

    cp = create(geo, "box3d_constraint_properties", "props")
    cp.setInput(0, cfg_in)
    cp.setInput(1, lines)
    check("Constraint Properties group1", cp.parm("group1"), ["grpA", "primB"])
    check("Constraint Properties constraintgroup", cp.parm("constraintgroup"), ["lineGrp"])
    check("Constraint Properties clusterattrib", cp.parm("clusterattrib"), ["myattr", "pa"])
    check("Constraint Properties force_thresholdattribute", cp.parm("force_thresholdattribute"), ["stiff", "myattr"])
    scripted = [p for p in cp.parms() if "Menu" in (getattr(p.parmTemplate(), "itemGeneratorScript", lambda: "")() or "")]
    for p in scripted:
        check(f"Constraint Properties {p.name()}", p, [])

    dop = new_dopnet("menus_dop")
    obj = box3d_packed_object(dop, "po", path)
    set_parm(obj, ["soppath"], tag2.path())
    check("Packed Object overwriteattributes", obj.parm("overwriteattributes"), ["myattr"])

    ok = not problems and len(scripted) > 20
    results.append(Result("asset_menus", Result.PASS if ok else Result.FAIL,
                          f"{len(scripted)} Constraint Properties menus built; " +
                          ("every checked menu lists its geometry" if not problems else "; ".join(problems))))
    return results


TESTS = {
    "asset_menus": test_asset_menus,
    "packed_object_overwrite_from_sop": test_packed_object_overwrite_from_sop,
    "configure_packs": test_configure_packs,
    "constraint_properties_scaling": test_constraint_properties_scaling,
    "constraint_properties_closest_surface": test_constraint_properties_closest_surface,
    "constraint_properties_proxy_transfer": test_constraint_properties_proxy_transfer,
    "constraint_properties_next": test_constraint_properties_next,
    "constraint_properties_vexpressions": test_constraint_properties_vexpressions,
    "constraint_properties_collisions": test_constraint_properties_collisions,
    "configure_material_preset": test_configure_material_preset,
    "configure_var": test_configure_var,
    "packed_object_geometry_source": test_packed_object_geometry_source,
    "configure": test_configure,
    "configure_group_patterns": test_configure_group_patterns,
    "constraint_properties": test_constraint_properties,
    "constraint_properties_edit": test_constraint_properties_edit,
    "no_subdata_warnings": test_no_subdata_warnings,
    "configure_initial_state": test_configure_initial_state,
    "configure_speed_limit": test_configure_speed_limit,
    "configure_pin": test_configure_pin,
}
