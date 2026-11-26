# Digital assets: shared notes, Box3D Packed Object

What every SOP / DOP asset shares, and the Box3D Packed Object. How the build runs the scripts: [build.md](build.md).
Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Shared by the assets

Group and attribute fields have the little menu beside them (RBD's do): `b3d_hda.group_menu` / `attrib_menu` /
`menu_script` set a Python menu script on the template that calls `kwargs["node"].hm().buildGroupMenu`
(`buildAttribMenu`, `buildConstraintNameMenu`, `buildCollisionGroupMenu`) in `MENU_SOURCE`, which `add_python_module`
puts in every asset's PythonModule. They list the groups or attributes of an input's geometry (or of the SOP a
parameter names, or the node's own output) and toggle each pick in a list. Fields with it: the Packed Object's and the
Solver SOP's Overwrite Attributes, Transfer Attributes, False Color Attribute, Inspector Attributes, Break Thresholds'
Constraint Names and Scale by Attribute, each type's Data Name; Configure's Group, Transfer Attributes, Collision
Group and Collision Ignore (both list the input's `collisiongroup` values and the other enabled entries' Collision
Groups; Ignore leaves out its own entry's); Constraint Properties' Constraint Group, Group 1 and Group 2.

Every asset has a help card (F1 on the node): `HELP_CARD` in `tools/b3d_<asset>_help.py` (Houdini wiki markup),
added by its builder as the definition's `Help` section. The compiled DOP nodes (Box3D Solver, Box3D Data, the
constraint relationships) have no definition to hold one: `tools/make_dop_help.py`, run by `make_assets.py` after the
assets, writes `help/nodes/dop/<node>.txt` next to `otls/`, where Houdini looks on HOUDINI_PATH. Only the intros are
written there (`SOLVER`, `DATA`, `JOINTS`); the parameters, folders and help come from the registered node types.

Every SOP asset gets a `PythonModule` (`b3d_hda.add_python_module`) that starts with RBD's network editor hooks
(`nodegraphvellumutils`, imported when the UI is available) and defines `isVellumNode()` returning true, which the
viewer reads to connect a new node's inputs automatically, as RBD's SOPs do.

Node layout: small networks (the packing subnets, the Packed Object's stage) use `b3d_hda.tidy` (rows by longest path
from the inputs, hidden stops for wires that skip rows, barycenter ordering plus adjacent swaps to cut crossings,
columns centred under the inputs with room for each name, unwired nodes in a column on the right). The Box3D Solver
SOP, its DOP network and Box3D Configure use `b3d_hda.layout` with the tables in `tools/b3d_layouts.py`: rows of
labelled network boxes, flow down the page, each box laid out by `tidy` (or a grid). A node the table does not name
goes in an "Other" box and the build prints it, so a new node needs a line there. The FORCES subnet keeps its own
layout (`layout_forces`).

## Box3D Packed Object asset

`tools/make_packed_object.py` builds it from the plugin just compiled, so it always matches Box3D Data. It copies RBD
Packed Object (read from `houdini_network_dump.py` dumps of RBD Bullet Solver and RBD Packed Object): Empty Object ->
SOP Geometry (data name `Geometry`) -> Box3D Data -> `visualize_hires` -> Collider Label, the output (no input); SOP
Geometry reads a SOP stage inside the asset (`STAGE`, RBD Packed Object's `create_or_overwrite_attributes`): Object
Merge of the asset's effective SOP path (`EFFECTIVE_SOP`, hidden: SOP Path, or `dopcontextgeo(".", n)` for Geometry
Source First to Fourth Context Geometry; Use Object Transform drives its Transform, Into This Object: the DOP
network's space; SOP Geometry's own stays off, as 22.0's has no OBJ Path to point at an object) -> `setup_defaults` ->
`create_name` -> `setup_velocities` (+ `switch_velocities`), `set_rest` and `initial_transform` on the creation frame
only -> `setup_deforming` (+ `switch_deforming`) -> `OUT`. The stage's references to the asset's parameters are set
once the asset exists (`wire_stage`).
* `save_source_geo` (Link to Source Object) and `applydata1` (on the creation frame) attach the SOP Path as the
  object's `SourceObject` data (`objpath`, and `usetransform` from `source_transform`, a Modify Data), as RBD Packed
  Object does. The solver follows animated bodies through it ([bodies.md](bodies.md)).
* Overwrite Attributes from SOP, as RBD Packed Object's `copy_attribs`: after the creation frame the stage outputs the
  object's own simulated geometry (`SIM_GEO`, an Object Merge of `../../..:<$OBJID stamp>`) with the
  `overwriteattributes` list copied from the SOP by point name (`copy_attribs`, Attribute Copy, points); SOP Geometry
  runs Set Always while it, Deforming or `reimportsop` is on (`REIMPORT`). Deforming, or `reimportsop` (set by the
  Solver SOP for a Deforming collider), outputs the whole SOP instead (`switch_reimport`): changing shapes need the
  full re-import. Poses do not: an Animated collider follows through `SourceObject`.
* `initial_transform` (point wrangle, `INITIAL_XFORM_VEX`) writes the point matrix `__initialxform`: Position and
  Rotation about the pivot (the point's own `pivot` from before `set_rest`, which writes P there: its input 1; else P
  while Inherit Pivot is on, else Pivot), applied by `apply_initial_xform` (Transform by Attribute: a primitive
  attribute is not found, so it must be a point one) after `set_rest`, so the rest stays at the SOP's pose, as RBD
  Packed Object's `set_rest_and_initial_xform`. It turns the points' vectors too: `v` and `w`, set before it, turn
  with the piece, as RBD's. The solver does not read a `pivot` attribute: it only decides where the initial rotation
  is about. Matched against RBD Packed Object in Bullet by `packed_object_initial_bullet`.
* `setup_defaults` (detail wrangle, `b3d_settings.packed_defaults_vex`; Default Shape is `setup_shape`, an Attribute
  Create with Write Values on behind `switch_shape`, which skips it when the points have `bullet_georep`, as a
  string's default is not applied): each per-body setting whose value differs from Box3D Data's default becomes a
  point attribute holding it, unless the points already have it (RBD Packed Object's Attribute Creates with Write
  Values off), so a point attribute wins and untouched settings write nothing. Names map as Box3D Configure's
  (`b3d_settings.MAPPING`).
* `create_name`: a point `name` where there is none: the primitive's, else `<object name>_<primitive number>` (RBD
  Packed Object's, which ignores the primitive's).
* `set_rest` (`b3d_settings.REST_VEX`): the rest reset and the `orient` / `pscale` / `scale` bake of RBD Packed
  Object's `set_rest_and_initial_xform`, with `orient` kept as the rotation since rest (see its comment). Switched in
  only on the creation frame: SOP Geometry stamps `$SF` and `$OBJCF`, which a `STAMPS` null reads, so a re-import
  (Overwrite Attributes from SOP, which the Solver SOP's Collision Type turns on) keeps the rest it was given.
* `setup_velocities` (an Attribute Create behind `switch_velocities`, skipped while Inherit Velocity is on): RBD
  Packed Object's Initial State tab (`vel`, `angvel` in degrees per second written as radians, `inheritvelocity`).
  Inherit defaults off, as RBD's; the Solver SOP turns it on for its pieces (`packed_object` in `b3d_solver_dops.py`),
  as RBD Bullet Solver reads the points' `v` / `w`, so Box3D Configure's Initial State reaches the simulation there
  and needs the toggle in a DOP network.
* `setup_deforming` (an Attribute Create behind `switch_deforming`, skipped while Deforming is off): the Deforming
  toggle writes `deforming` 1 on every import and, with Overwrite Attributes from SOP, sets the SOP Geometry to Set
  Always (`REIMPORT`), as RBD's Create Deforming Objects.
* Initial Object Type (`initialstate`: Create Active / Static / Animated Static / Deforming Static / Deforming Active
  Objects, as RBD Packed Object, plus Custom): Body Type and Deforming follow it, through two hidden parameters the
  stage reads (`effectivebodytype`, `effectivedeforming`, expressions of the menu; Custom takes the visible `bodytype`
  / `deforming`, which are greyed out otherwise). A point's own `active`, `animated` or `deforming` still wins. The
  Solver SOP's collision object sets it to Custom and drives Body Type by Collision Type; the Pieces tab has the menu
  first in Body.
* Parameters: an Object tab (Object Name `$OS`, Creation Frame (default: the network's Start Frame), Creation Frame
  Specifies Simulation Frame, Solve on Creation Frame, Geometry Source (`GEO_SOURCE`, greys out SOP Path unless SOP),
  SOP Path, Use Object Transform, Overwrite Attributes from SOP (toggle and attribute list, default
  `OVERWRITE_DEFAULT`), a hidden Read SOP Every Step (`REIMPORT_SOP`), Deforming, Display Geometry), Initial State
  (Position, Rotation, Inherit Pivot from Point Position, Pivot, Velocity, Angular Velocity, Inherit Velocity), then
  each Box3D Data folder as a tab (`pull` moves Add Impact Data from CCD & Contacts to Shape, as RBD Packed Object has
  it with its shape settings; Start Asleep stays in Sleep), read by the stage. Only the object-level settings
  (`b3d_settings.SKIP`: Hull per Connected, Add Impact Data) are channel references into the inner Box3D Data, whose
  other values stay at the defaults, so a missing attribute means the default. Settings change mid-simulation through
  the attributes, not the asset's parameters.
* Display Geometry (`add_display`, RBD Packed Object's nodes and names): Rendering Parameters Volatile on SOP Geometry's
  data input shows the object while it is Simulation; Render shows the SOP Guide `visualize_hires` instead: a For-Each
  over the asset's `createdObjects()` (the iteration from a metadata Block Begin), DOP Import of that object's points,
  Transform Pieces of Render SOP Path by them, matched by `name`. Python expressions, as SideFX's.
* Collider Label `PackedObject`, as RBD Packed Object's: other solvers collide with it as packed pieces; the solver
  still syncs it per point (`isWholeObject` takes any other label as one body). The script prints one line, or
  warnings, and never fails the build; its `check` cooks the stage of a fresh instance.

All four assets list themselves under Box3D in the Tab menu (`b3d_hda.set_tab_submenu`: a Tools.shelf section as
SideFX's, `doptoolutils` / `soptoolutils.genericTool`).

Traps it handles:
* SideFX parameters and menu items are set by name and token (`set_named`, `set_menu`, `b3d_hda.set_token`), never by
  label. Our own menus' tokens are the item numbers (`Box3DParms::menu`). Our folders are found by label: their switcher
  names are numbered.
* Empty Object has no creation frame: its Activation is `if(ch("../usesimframe"), $SF, $FF) == ch("../createframe")`
  (Creation Frame Specifies Simulation Frame, off by default, as RBD Packed Object's).
* The inner Box3D Data's `parmop_*` are set to Set Initial: a new data node starts at Use Default, and only an edit (not
  a channel reference) switches it, so the linked object-level settings would otherwise keep their built-in values.
* Box3D Data's own DOP settings outside its folders (Activation, Group, Data Name) are not promoted: promoted copies
  kept it from attaching.
* A second Box3D Data after the asset only changes Set Always parameters (Set Initial applies when the data is
  created, and the asset's already has been), and only for settings the asset left at their defaults.


## Network dump

`tools/houdini_network_dump.py` is not part of the build: run it in Houdini (Python Source Editor, a shelf tool, or
`main(node_path, out_path)` in hython) to write a node and everything inside it to a text file: the HDA's interface
and text sections, a hierarchy tree, and per network an ASCII layout, boxes, notes, wiring and each node's
non-default parameters (VEX, expressions, references). It dumps an unlocked temporary copy (`WORK_ON_COPY`), so the
scene's assets stay locked. This is how RBD Bullet Solver's and RBD Configure's packing were read for
`b3d_prepare.py`.
