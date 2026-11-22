# Box3D Configure asset

Index and rules: [CLAUDE.md](../../CLAUDE.md).

`tools/make_configure_sop.py <out.hda>` builds `box3d_configure` (SOP): Box3D Data's settings, copied from the node
(labels, defaults, ranges, conditions), written as point attributes. Laid out as RBD Configure: a tabbed multiparm
(`MULTIPARM`, Number of Attributes) of entries, each with Enable (`enable#`), Group (`group#`) and Box3D Data's folders
(`b3dcfg_folder<i>_#`), every setting `<name>#` with an `add<name>#` checkbox (`toggle_name`, label hidden). A setting with Use Bounds is an integer slider clamped to 0 - 1 when it is an on / off one (`on_off_slider`), as RBD Configure's, so the row fills and Use Bounds sits at the right. The packing
settings sit above the entries in the Attributes folder with them.
Var (`VAR_SETTINGS`: Density, Bounce, Friction, Rotational Stiffness; `use<name>var#` and `<name>var#` joined to the value, as RBD
Configure's) multiplies each piece's value by `fit01(rand(@ptnum), 1 - var, 1 + var)` in the wrangle, as RBD's (`configure_var`).
Material Preset (the top of the Material folder, above Bounce and Friction: `materialtype#` None / Concrete / Glass / Wood / Metal, `materialpreset#` the material, `materialstate#` dry and clean or lubricated and greasy
surfaces, Type and Material joined) fills the entry's `density#`, `bounce#` and `friction#` and ticks their `add...#` boxes, as RBD Configure's Physical Attributes does. The table is
`tools/b3d_materials.py`: RBD Configure's presets (materials, densities, the two frictions, Bounce by type) typed in, so nothing is read from its node, put into this asset's Python Module (`MATERIAL_MODULE`: `buildMaterialMenu`, `typeChanged`,
`materialPresetCallback`, set off the menus' script callbacks, so a script that sets them calls the functions). Names map one to one except `MAPPING`: Default Shape -> `bullet_georep` (`GEOREP_VALUES`, menu order), Auto Fit
-> `bullet_autofit`, Body Type -> `active` / `animated`, Start Asleep -> `awake` (inverted); `SKIP` holds the
object-level ones (Hull per Connected, Add Impact Data). Disable conditions also need the referenced
setting ticked (`rewrite_condition`). Bounds and Attributes are simple folders (`simple_folder`, as RBD Configure's); each Box3D Data folder and Constraints in an entry is an open collapsible folder (`b3d_hda.collapsible`).
* Geometry and Proxy are packed first, as RBD Configure does, by the chain the Box3D Solver SOP runs
  (`tools/b3d_prepare.py`, `prepare_input` in pack mode without the rest attributes): packed primitives kept, the rest
  assembled into packed pieces named on their points. RBD Configure's packing parameters drive the Assembles (Name
  Prefix, Transfer Simulation / Render Geo Attributes, Create Path Attribute, Path Attribute, Create Packed Fragments;
  `packing_templates`), and Transfer Groups is RBD Configure's (`TRANSFER_GROUPS`) so Group still finds the pieces.
  Without a proxy (or without geometry) the configured stream is the geometry output; with both, the packed geometry
  passes through and the proxy gets the settings.
* A new entry starts as RBD Configure's: Density and Bounce ticked and Bounce 0.25 (`ON_BY_DEFAULT`, `SETTING_DEFAULTS`; Box3D Data's is 0.5),
  so a scene bounces the same in both solvers.
* One point wrangle (`set_attributes`, `wrangle_vex`) loops over the entries: an enabled entry whose group holds the
  point writes each ticked setting, creating the attribute with Box3D Data's default first (`addpointattrib`) so
  points outside the group keep the default rather than VEX's 0 (`active` 0 is static). Group is tested with
  `inpointgroup` / `inprimgroup`, as RBD Configure, so patterns (`piece*`, `@name=piece0`) work; empty is every piece. The Group button
  picks primitives in the viewport (RBD Configure's `selectGroupParm` call).
* Settings Box3D Data has no parameter for (`extra_folders`): Initial State (`v`, `w` in degrees per second written as
  radians, `enabled`, `deforming`), Motion (`fastrotation`, `lock_linear` / `lock_angular`), Collision Filtering (`collision_category` / `collision_mask` / `collision_group`, and the string
  `collisiongroup` / `collisionignore`, kind `s`, which the Solver SOP's `COLLISION_NAMES_VEX` turns into the integers).
  The hand-set shape and Rotational Stiffness are Box3D Data's own (its Shape and Body folders).
  Defaults are the solver's when the attribute is missing.
* Visualization folder (`visualization_folder`, `visualization_templates`), as RBD Configure's (read from a dump): a
  first-level simple folder, which makes the viewer show its parameters in a bar along the top of the viewport (its two parameters also carry
  export level 3, `export all`, as RBD's, and Visualize is an integer with a menu, not an ordinal one). Visualize (`VISUALIZE`)
  and Exploded Scale (`scale`). A viewer-only **Visualize output** (`OUT_visualize`, Output
  Index -1, not a real output) holds the target stream coloured by Visualize (`vis_range` finds the min / max of a ramped
  setting, `vis_color` sets `Cd` per primitive from its point; a missing attribute is Box3D Data's default),
  moved apart by an Exploded View node. RBD
  shows it from a viewer state that sets the node's output-for-view flag to -1; here each of the two parameters'
  callback (`VIEW_CALLBACK`) sets the flag to -1 while either is on, else 0. Nothing downstream sees the colours. RBD's
  Collision Padding is not copied (In Bounds takes the place of the last).
* Viewer state (`tools/b3d_configure_state.py`, installed as the definition's `ViewerStateModule` / `DefaultState` /
  `ViewerStateInstall` / `ViewerStateUninstall` / `ViewerStateName.orig` sections by `add_viewer_state`, with a
  `PythonModule` whose `enterState` the Visualization callbacks call to re-enter the state), RBD Configure's trimmed: for Bounce,
  Density, Friction and Rotational Stiffness it draws a histogram of the pieces' values at the lower left and the
  minimum / maximum as text at the upper left. The state only reads two unconnected nodes: `HISTOGRAM` (`hist_grid`, a
  `HIST_COLS` x `HIST_ROWS` grid -> `hist_color` -> `hist_xform`; each bar is one value bin) and `DISPLAY` (a Null on the
  coloured stream, whose detail `__vmin` / `__vmax` are the text). `vis_color` writes the bin as primitive `val`
  (0..`HIST_BINS`) and colours on RBD's blue-white-red ramp, which the bars share. `hist_color` darkens the cells a
  bin fills up to its height, ignoring the lowest bin when it holds most pieces. RBD's state also sets the
  output-for-view flag on enter and exit: ours does not (the callbacks above own the flag).
* Bounds box handle, as RBD Configure's: the DialogScript binds Size, Center and Rotation to the viewport's `boundingbox`
  handle (`BBOX_HANDLE`, written by `b3d_hda.edit_dialog_script`), shown by the hidden `bboxonoff` toggle, which defaults
  to `boundingtype == 0` (Bounding Box). Changing Bounding Type calls `enterState`.
* Bounds and Pin, as RBD Configure's (read from a dump of it): the Bounds folder (`boundingtype`, `bbox*`) and the
  fourth input (Static Bounds Geometry) put the pieces inside into the point group `INSIDE_GROUP` (`inside_bounds`;
  no bounds geometry: none). Use Bounds (`use<name>bounds#`, on Body Type and every toggle, `has_bounds`) gives the
  value inside and the opposite outside (Body Type: Dynamic). Pin (`pin_templates`) marks pieces (`PIN_ATTRIB`) and
  `add_pin_constraints` (`PIN_VEX`) appends one line per pinned piece from the world to its point, `constraint_name`
  from `PIN_TYPES` (Weld, Soft: the Solver SOP's relationship Data Names; the lines carry no `constraint_type`, so a Soft pin holds position only, as in Bullet). The marker and group are stripped.
* It uses `setpointattrib`, never `@` bindings: a binding creates its attribute on every point at compile time, even
  inside an `if` that never runs.
* The build checks a fresh instance: two unpacked boxes come out as two packed pieces named on their points, the
  group's piece gets friction and Static, the other keeps defaults; with a proxy only the proxy gets them.
