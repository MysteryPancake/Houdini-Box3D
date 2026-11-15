"""Box3D Data's per-body settings as point attributes, shared by make_configure_sop.py (Box3D
Configure) and make_packed_object.py (the Box3D Packed Object's SOP stage), and the rest a packed
object sets on its creation frame (REST_VEX).

Not a build script: imported by the builders (run with hython, so this folder is on sys.path).
"""

import hou

from b3d_hda import NO_VALUE, without_parmops

# Box3D Data parameter -> how it is written. "same": the attribute of the same name. Anything
# else is a special case (setting_vex in make_configure_sop.py, packed_defaults_vex here).
# Parameters in SKIP are object-level only: no point attribute, they stay on Box3D Data.
MAPPING = {
    "defaultshape": "bullet_georep",
    "autofit": "bullet_autofit",
    "bodytype": "active/animated",
    "startasleep": "awake",
}
SKIP = {"hullperconnected", "flipwinding", "addimpact"}

# Default Shape's menu order (SIM_Box3DData.cpp) as bullet_georep values.
GEOREP_VALUES = ["box", "sphere", "convexhull", "concave", "capsule", "none"]
DYNAMIC = 2  # Body Type's Dynamic (Static, Kinematic, Dynamic)

def settings(data, warn):
    """Box3D Data's folders as (folder label, [(template, kind)]): kind is the attribute type
    for same-name settings (f, i, v) or the MAPPING entry. SKIP is left out."""
    folders = []
    for f in without_parmops(data.parmTemplateGroup().entries()):
        if f.type() != hou.parmTemplateType.Folder:
            continue  # DOP data settings outside the folders (Activation, Data Name ...)
        rows = []
        for pt in f.parmTemplates():
            if pt.type() in NO_VALUE or pt.name() in SKIP:
                continue
            if pt.name() in MAPPING:
                rows.append((pt, MAPPING[pt.name()]))
            elif pt.type() == hou.parmTemplateType.Toggle:
                rows.append((pt, "i"))
            elif pt.type() == hou.parmTemplateType.Float and pt.numComponents() == 3:
                rows.append((pt, "v"))
            elif pt.type() == hou.parmTemplateType.Float:
                rows.append((pt, "f"))
            elif pt.type() == hou.parmTemplateType.Int:
                rows.append((pt, "i"))
            else:
                warn(f"Box3D Data's {pt.name()} ({pt.type()}) has no point attribute mapping; left out")
        if rows:
            folders.append((f.label(), rows))
    return folders


def num(x):
    text = f"{float(x):.6g}"
    return text if any(c in text for c in ".en") else text + ".0"


def literal(pt, kind):
    """Box3D Data's default as a VEX literal (menus and toggles as ints)."""
    d = pt.defaultValue()
    if kind == "v":
        return "{" + ", ".join(num(x) for x in d) + "}"
    if kind == "f":
        return num(d[0])
    return str(int(d if isinstance(d, (bool, int)) else d[0]))


def packed_defaults_vex(rows, parm_path):
    """Detail-wrangle VEX for the Box3D Packed Object: each setting whose value (the parameter at
    `parm_path`/<name>) differs from Box3D Data's default becomes a point attribute holding it,
    unless the points already have that attribute (as RBD Packed Object's Attribute Creates with
    Write Values off). Settings left at their defaults write nothing: a missing attribute is the
    default to the solver."""
    lines = ["// Box3D Packed Object: settings that differ from Box3D Data's defaults, as point attributes the",
             "// points do not already have (they win, as on RBD Packed Object)."]

    def add(attr, value):
        return f'if (!haspointattrib(0, "{attr}")) addpointattrib(0, "{attr}", {value});'

    for pt, kind in rows:
        n = pt.name()
        ch = f'"{parm_path}/{n}"'
        if kind == "bullet_georep":
            shapes = ", ".join(f'"{v}"' for v in GEOREP_VALUES)
            # A string attribute's default is not applied to the points (Houdini keeps "" for every
            # point), so the value is set on each point.
            lines.append(
                f"{{ int v = chi({ch}); "
                f'if (v != {literal(pt, "i")} && !haspointattrib(0, "bullet_georep")) {{ '
                f"string s[] = {{{shapes}}}; string shape = s[clamp(v, 0, len(s) - 1)]; "
                'addpointattrib(0, "bullet_georep", ""); '
                'for (int pt = 0; pt < npoints(0); pt++) setpointattrib(0, "bullet_georep", pt, shape); } }')
        elif kind == "bullet_autofit":
            lines.append(f"{{ int v = chi({ch}); if (v != {literal(pt, 'i')}) " + add("bullet_autofit", "v") + " }")
        elif kind == "active/animated":
            lines.append(f"{{ int v = chi({ch}); if (v != {DYNAMIC}) {{ " + add("active", "0")
                         + " if (v == 1) " + add("animated", "1") + " } }")
        elif kind == "awake":
            lines.append(f"{{ int v = chi({ch}); if (v != {literal(pt, 'i')}) " + add("awake", "!v") + " }")
        elif kind == "v":
            lines.append(f"{{ vector v = chv({ch}); if (v != {literal(pt, kind)}) " + add(n, "v") + " }")
        elif kind == "f":
            lines.append(f"{{ float v = ch({ch}); if (v != {literal(pt, kind)}) " + add(n, "v") + " }")
        else:
            lines.append(f"{{ int v = chi({ch}); if (v != {literal(pt, kind)}) " + add(n, "v") + " }")
    return "\n".join(lines)


# The rest a packed piece gets on its object's creation frame, as Bullet's RBD Packed Object
# (create_or_overwrite_attributes/set_rest_and_initial_xform): every piece starts where it is,
# whatever restxform it carries (restxform = its packed transform now, rest = P). An input orient /
# pscale / scale is then baked into the packed transform about P. Bullet resets orient there and
# its solver writes the rotation since rest on the first solve; here orient keeps that rotation
# straight away (the solver reads it as the rotation since rest), so Transform Pieces turns the
# Solver SOP's Geometry output by it on the creation frame too. A point-instance transform gets an
# identity rest, as Bullet. pivot = P: the position at rest, which Transform Pieces reads.
REST_VEX = """\
int prims[] = pointprims(0, @ptnum);
if (len(prims) != 1) return;
int prim = prims[0];
v@rest = v@P;
v@pivot = v@P;
if (primintrinsic(0, "pointinstancetransform", prim)) {
    4@restxform = ident();
    return;
}
4@restxform = primintrinsic(0, "packedfulltransform", prim);
vector s = {1, 1, 1};
if (haspointattrib(0, "pscale"))
    s *= float(point(0, "pscale", @ptnum));
if (haspointattrib(0, "scale"))
    s *= vector(point(0, "scale", @ptnum));
vector4 q = {0, 0, 0, 1};
if (haspointattrib(0, "orient"))
    q = normalize(vector4(point(0, "orient", @ptnum)));
matrix3 bake = ident();
scale(bake, s);
bake *= qconvert(q);
matrix3 local = primintrinsic(0, "transform", prim);
setprimintrinsic(0, "transform", prim, local * bake);
p@orient = q;
"""

# A point name for pieces without one (joints find pieces by it), as RBD Packed Object's
# create_name_attrib: <object name>_<primitive number>, except that a primitive name is used first
# (the solver reads the point's).
NAME_VEX = """\
if (haspointattrib(0, "name")) return;
string name = hasprimattrib(0, "name") ? prim(0, "name", @primnum) : "";
s@name = name != "" ? name : chs("objname") + "_" + itoa(@primnum);
"""
