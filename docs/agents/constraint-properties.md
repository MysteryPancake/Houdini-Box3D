# Box3D Constraint Properties asset

Index and rules: [CLAUDE.md](../../CLAUDE.md).

`tools/make_constraint_properties_sop.py <out.hda>` builds `box3d_constraint_properties` (SOP), like RBD Constraint
Properties: the settings of every joint type as primitive attributes on the Constraint Geometry's lines. Inputs and
outputs are Geometry, Constraint Geometry (pink), Proxy Geometry; only the constraints are edited, the others pass
through. No Bullet settings.
* Settings are copied from the `box3d_<type>conrel` DOP nodes (`collect`): the Weld node's Joint folder is the common
  folder, each node's own folder is that type's. `NODE_ONLY` lists the node parameters that are no attribute
  (Breakable, Set Rest Length, the Limit toggles, Attributes Replace Values). Names: `<type>_<attribute>` in a type's
  folder (the same attribute is shared by several types, with different defaults and ranges), `add...` its checkbox.
  Constraint Type (`constrainttype`) hides all but one type's folder.
* A detail wrangle (`create_settings`) makes the attributes and a primitive wrangle (`set_properties`, `wrangle_vex`,
  its group the constraint group, in parallel) writes the values to the constraints in the group (empty: all; Filter
  by Pieces: SideFX's `rbdgroupconstraints` node with its group `__tmpconpropgrp`, output 1; removed by
  `strip_group`). Each ticked setting creates its attribute with the "not set" value first (`kUnsetFloat` / -1), then
  sets the group's lines: lines outside the group keep the relationship node's value (`applyOverrides` skips unset
  values). Also `constraint_name`, `next_constraint_name` (menus of the types' default Data Names) and `Cd`.
* Float values multiply the node's value, or replace it with the node's Attributes Replace Values on; the node does not
  convert and says nothing about it (`docs/joints.md` does). Defaults and ranges are the nodes'.
* Action (`action`, as RBD Constraint Properties: Edit, Set, default Set): Edit writes the ticked settings (above). Set
  writes the settings of the joint type in Constraint Type (and the Joint folder's) that differ from the relationship
  node's default (`wrangle_vex`'s `setmode`; `attribute_rows` carries each setting's type number and default), and
  always the Constraint Name, with the checkboxes hidden. Settings left at their default stay "not set", so Set cannot
  multiply or zero a value the node holds, and a value cannot be set back to its default with Set.
* Anchor Rotation (`anchorrotate`, in the Joint folder, written in both modes when ticked): Euler degrees turned into
  the `orient` point attribute of both anchors of each line in the group.
* Viewport editing, RBD Constraint Properties' state adapted (`tools/b3d_constraint_properties_state.py`, installed by
  `b3d_hda.add_viewer_state`, state name `STATE_NAME`): pick constraint lines (a geometry selector on the viewer-only
  `widgets` output) and turn a rotation handle to rotate the frames of their two anchors; hotkeys U reset, A align the
  frame's X axis to the line, Ctrl+A select all, Ctrl+C / Ctrl+V copy / paste, M Toggle Motor, P Edit Parms. RBD's
  handle aims a hard constraint's angular motors; Box3D's motors act in the joint frame, so the handle already aims
  them. M switches `enable_motor` (off or unset becomes on, on becomes off), and the mouse wheel edits the spherical
  joint's `motor_velocity` (Z, Shift X, Alt Y, Ctrl ten times the step, `MOTOR_STEP` degrees per second). The motor
  speeds and torques are floats, which multiply the relationship node's value (zero stays zero), so the viewport does
  not edit them. The edits live in the `stash` parameter (Geometry Stash) (a Geometry data parameter, shared with the
  inner Stash SOP): one point per edited line with `__idx` and the fields `orient_a` / `orient_b`, `enable_motor`,
  `motor_velocity`, each with a "not edited" value the network skips (`FIELDS`). `index_lines` numbers the lines
  (`__idx`, removed again by `strip_index`), `apply_edits`, a primitive wrangle as RBD Constraint Properties'
  `copy_motors`, writes the stash on top of the settings (the attributes first, by `edit_point_attribs` and
  `edit_line_attribs`, all behind `switch_edits`, which skips them while the stash is empty). Box3D constraint types
  only: SideFX's (Glue, Hard, Soft, Spring, Cone Twist) are deliberately not on this node, so it stays separate from
  RBD Constraint Properties. A line can still carry their attributes written by hand or by SideFX's node. Use
  VEXpressions (`vex_templates`: `<type>_uselocalproperties`, `<type>_localexpression`, a VEX code editor with a
  snippet menu) ends each type's folder, as RBD Constraint Properties' types do: `run_vexpressions`, a primitive
  wrangle over the group (Constraint Geometry, Geometry and Proxy Geometry as inputs 0 to 2, the snippet of the type
  in Constraint Type), sits behind `switch_vexpressions` after `set_properties` and runs when that type's toggle is
  on. Its `ch()` calls read the wrangle's own parameters (RBD's button for making spare parameters from them is not
  here). Disable Collisions (`docollisions`, `collisions`: Disable 0, Enable 1) writes `collide_connected`, RBD
  Constraint Properties' wording for the Joint folder's Collide Connected; it is written after the rows, so it wins if
  both are ticked. The three type controls have RBD's hidden tick in front of the menu, hidden in Set mode: Constraint
  Type (`doconstrainttype`; in Edit mode a type's ticked settings are written only while it is ticked, and the type's
  folders and menu are greyed out without it), Switch Constraint Type When Broken (`dousenextconstraint` with
  `usenextconstraint`, a Disable / Enable menu) and Next Constraint Type (`donext_constrainttype`, the same for the
  next settings). The **Next Constraint** folder holds Switch Constraint Type When Broken (`usenextconstraint` Enable,
  ticked in Edit mode, is the master: no `next_*` attribute is written otherwise), Next Constraint Name, Next
  Constraint Type (`next_constrainttype`, which type's folder shows) and the next settings, a folder per type with the
  same checkboxes (`next_<type>_<setting>`, written as `next_<setting>`, no scaling). In Set the next name is always
  written while the master is on, and the next settings that differ from the node's defaults. The solver turns them
  into the settings when the constraint switches ([joint-networks.md](joint-networks.md)). Scaling, as RBD Constraint Properties':
  every single-number setting (`kind` f) has `<name>scaling` (No Scaling, Scale by Attribute, Scale by Value, Scale by
  Both) and `<name>attribute` (default the setting's own name; hidden for No Scaling and Scale by Value, with
  `buildScaleAttribMenu`). `set_properties` takes the Geometry (input 1) and Proxy (input 2) too and writes
  `scaled(...)` (`SCALE_VEX`): Scale by Value multiplies by the line's existing attribute of the setting's name (a
  "not set" one is ignored), Scale by Attribute by the named primitive attribute of the line, else the point attribute
  of the Geometry (else Proxy) pieces its two points' `name`s stand for, averaged over the ends. The Geometry
  Attribute Transfer folder (`clusterattrib`, default `parentpiece`; `transfermode`: Closest 0, Min 1, Max 3, Average
  2, as Bullet's tokens) picks the pieces: those whose Cluster Attribute equals the name, else the one named so. With
  a Proxy, as Bullet's `get_name_and_attrib`, an end that names a proxy piece which is no Geometry piece stands for the
  Geometry pieces its own Cluster Attribute value names (`anchorvalue`), else for the proxy piece; a Geometry piece
  of that name is used as before. Closest is the nearest to the anchor's surface (one `xyzdist` over the primitives of those pieces, as Bullet's, so a
  packed piece is measured by its geometry; a piece with no primitive by its point; a packed primitive gives its
  point's value, any other is interpolated with `primuv`), the others combine them. An end whose piece is
  not found is left out of the average, and with none found the setting is written unscaled. RBD Constraint Properties
  writes nothing when either end has no piece (a world anchor too); kept different so a line pinned to the world still
  scales by its one piece. Set also writes a scaled setting whatever its value. `RANDOMIZABLE` (the Weld's
  Linear and Angular Stiffness: RBD Constraint Properties has them on Glue's Strength only, and Glue is not on this
  node) also have `<name>randomize` (a joined checkbox) and `<name>variance` (default 0.8): after the scaling the value
  is times `fit01(rand(constraint number, seed), max(0, 1 - variance), 1 + variance)` when it is not negative, the
  seed from the setting's name. `__idx` /
  `__sel` (Index, Selection, not saved with the scene) hold the selection. All three are visible in a HandleParms
  folder, as RBD Constraint Properties'. Edit Parms makes the handle edit Anchor Rotation, for every line in the
  group, instead. Clear Edits, Import Edits and Edit Parms are in the viewport toolbar (`show_in_toolbar`). Guide
  Scale (default 0: hidden, range 0 to 1, as RBD Constraint Properties') draws the anchor frames of the lines in the
  group (`guide_size` writes `pscale` per line, `guide_pscale` promotes it to the points, and `guide_frames` copies
  the red / green / blue frame onto them by `orient` and `pscale`, behind `switch_guides` -> `GUIDES`; `glow_lines` is
  a primitive wrangle). The state reads `CONSTRAINT`, `HANDLE`, `GUIDES`, `GLOW` and `widgets` (the layout's Viewer
  box); its helpers (`set_edits`, `selected_ids`, ...) are plain functions, so a check calls them without a viewport.
* The build probes the asset (`probe`); the tests are `constraint_properties` and `constraint_properties_edit`.
