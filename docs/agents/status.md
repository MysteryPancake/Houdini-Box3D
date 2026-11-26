# Status: unverified

What is unverified: here for the general items, [status-areas.md](status-areas.md) for Bullet comparisons, whole objects
and the assets. What is still to do: [todo.md](todo.md). Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not these files).

Last full run (Houdini 22.0.502, MSVC, the Box3D fork): every check passes, `baseline` included. Changes since have
been checked one test at a time under 22.0.508, with no full run. The INFO results are not pass marks.
What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

## Unverified

**Not yet run in Houdini**

* Glue as One Body releasing the pieces in a fast body's way (`holdCompoundsForApproach`,
  [glue-compound.md](glue-compound.md)): run by the `building` example (`glue_compound_hit`), by a box thrown at a
  glued pair (`glue_compound_id`: the pair's `compound_id` goes to -1 with nothing broken) and by a kinematic piece
  (`glue_compound_kinematic`). At Strength 20000 most of the building's bonds break from its weight in both modes
  before anything hits it (Glue counts a resting body's contact impulse as a hit, as Bullet does:
  [joints-sidefx.md](joints-sidefx.md)). Not checked: other scenes, a slow heavy body, bridges of bonds remade while
  pieces are held, and `clang-tidy` on the HDK files (`src/sync`, `src/dop`).

* Glue as One Body drag and speed limits (`refreshCompoundMotion`, [motion-limits.md](motion-limits.md)):
  `bullet_glue_drag_limits` checks `airresist`, `speedmax`, `spinmax` and `spinmin` against Bullet. Not checked:
  `dragexp`, `targetv` / `targetw` and `spinresist` on a chunk (weighted by mass, as `airresist`).

* Packed Object Initial Object Type (`initialstate`): not checked are the Solver SOP's collision object (Custom), the
  Pieces tab's Body Type greying out with the menu (hython's `Parm.isDisabled()` never evaluates conditionals: Auto
  Fit's reads False too, so only the UI shows it), and Deforming Static / Active against RBD Packed Object.

* Constraint Properties Use VEXpressions (`run_vexpressions`): not checked are the snippet menu
  (`vexpressionmenu.buildSnippetMenu` with our own key lists nothing unless snippets are saved under it) and the type
  folders' layout.
* Constraint Properties scaling and Geometry Attribute Transfer (`SCALE_VEX`). Not checked against RBD Constraint
  Properties: Bullet has no `next_<setting>` attributes, only `next_constraint_name` and `next_constraint_type`: the
  next type's settings are the type's own folder, under the plain names and with its scaling, where ours writes
  separate `next_<setting>` attributes with no scaling (the Box3D solver reads those). Known difference: Bullet
  scales one integer, Glue's `propagationiterations`; ours has no integer scaling (Bullet scales no toggles or
  vectors). Randomize and Variance are on the Weld's two stiffnesses only, where Bullet has them on Glue Strength.
* The group / attribute menus beside the fields (`MENU_SOURCE` in `b3d_hda.py`, lists covered by `asset_menus`). Not
  checked: a multi-name field toggling picks in and out (a click), the menus on the Solver SOP's nested nodes, and the
  node's output (Inspector Attributes).

* Animated bodies following their source SOP (`followSourceSops`; `animated_follows_source` matches Bullet with source
  `orient` / `pscale`, 200 pieces and a Start Frame of 5). The Solver SOP's collider follows too: its context geometry
  resolves to a SOP node inside the asset (`solver_sop_animated_collision`).
* A Solver SOP piece whose input SOP moves: the Geometry and Proxy outputs match RBD Bullet Solver SOP for kinematic
  and dynamic pieces (`solver_sop_moving_input`). Still different with a `deforming` attribute on the input: Bullet's
  kinematic piece's body stays put and a dynamic one's follows the input, Box3D's kinematic one follows and a dynamic
  one's stays (`deforming` rereads shapes only); the drawn geometry is the current input in both.

* Overwrite Attributes from SOP as an Attribute Copy list (`copy_attribs`, `SIM_GEO` in the Packed Object stage): not
  checked is an animated collider (`reimportsop`) still moving.

* A box landing flat at speed tips slightly in Box3D and Bullet's does not: a 1 m cube dropped 2.5 (about 7 m/s)
  tilts up to 0.95 degrees on the bounce and ends about 0.1 to the side (Bullet: 0.32 degrees, 0.002), glued or not,
  packed or RBD Object, any thread count, with or without bounce, Continuous Collision or contact recycling; dropped
  0.1 it lands level in both. It grows with the overlap a fast landing makes (a lower Overlap Push Speed halves it), so
  it is Box3D's contact solve, which works a manifold's points one after another (`contact_solver_wide.inl`), not the
  wrapper. `bullet_computecom_cases`' glue case allows for it after the landing.

**Build**
* Linux plugin build (only the core builds in CI): `build.sh`, gcc warnings in `src/dop` / `src/sync`, the dso and
  icon copies, `hython tests/houdini/run_tests.py`.
* The Houdini-side files have not been through `-Wshadow` / `-Wconversion` (GCC / Clang) or clang-tidy.
* The Threads default (`performanceCoreCount`) on macOS (`hw.perflevel0`) and ARM Linux (`cpu_capacity`).

**Houdini behaviour**
* Glue Inertial Load (`gluecompoundload`, `pieceLoad`): the maths is checked in `core_compound` and
  `glue_compound_load` shows the load reaching the bonds of a column landing on its end. Not checked: that a cube
  dropped on a corner collapses, nor how its break values compare with Bullet's impacts.
* Escape mid-step: the step is discarded and re-cooked, earlier frames stay cached, the re-cook rebuilds once.
* Data ids surviving the DOP copy between timesteps (the sync skip; debug log `scan skipped`); our own writes not
  looking like edits (debug log `full scan, changed:`).
* `SourceSummaryCache`: a packed source edited in place without bumping its data ids would give a stale hull. hython
  cannot make one (an edit through a SOP gives a new unique id: `source_edit_hull`).
* Box3D's tasks on Houdini's TBB pool with several DOP networks or threaded SOPs cooking at once.
* The ground guide (`build_ground_guide`, test `solver_sop_ground_guide` checks the geometry): not seen in a viewport,
  so whether the ground box is drawn solid and transparent like the collider geometry under `gl_wireframe` -1.
* Box3D Data's Default Shape menu and the Packed Object / Configure menus built from it, opened in Houdini's UI
  (`default_shape_menus` checks their entries and what Plane and Cylinder write).
* `bullet_georep` `plane` on a packed piece (`ShapeType::Plane`, `buildShapes`): `bullet_plane_piece` has a box rest
  on it at the piece's height (through the piece's origin, +Y, as a Ground Plane). A box dropped on it in Bullet fell
  through, so there is nothing to compare against. A piece turning moving mid-simulation rebuilds as a box.
* Ground: a Static Object whose SDF Representation is set to Implicit Plane is taken as a ground box facing +y through
  its origin, as a Ground Plane DOP's is (only the Ground Plane is tested). A ground tilted off the axes has a large
  broad-phase box, so every body in the world keeps a cheap contact with it. A box in a 90 degree trough of two
  grounds with the default friction wedges slightly high (`ground_plane`: 0.82 against 0.71), and sits flat with a
  friction of 0.2.

**Box3D Configure** (`tools/make_configure_sop.py`, tests `configure`, `configure_packs`)
* The packing (`b3d_prepare.py`): Transfer Groups carrying primitive groups onto the pieces, the Assembles' path /
  fragment parameters from the asset, geometry-only and proxy-only inputs.
* The group selection button, labels hidden on the checkboxes.
* The viewport bar and the state load in Houdini. Not checked: the Visualize colours, the histogram's contents on real
  pieces, the Output SOP with index -1 and the callbacks that switch to it.

**Data hint for Box3D Data** (`initializeSIM`, test `no_subdata_warnings`)
* Hints are for the exact types Houdini reports for the Geometry (`SIM_SopGeometry`, `SIM_GeometryCopy`). Not known:
  whether a hint on one type is also found for a subclass, so a Static Object's or another cached Geometry may need its
  own.

**Box3D Constraint Properties** (`tools/make_constraint_properties_sop.py`, test `constraint_properties`)
* The viewer state (`b3d_constraint_properties_state.py`: selector, rotation handle, hotkeys, HUD, guides) is written
  from a dump of RBD Constraint Properties and never run in a viewport. Only its stash and network are checked
  (`constraint_properties_edit`). Not known: whether the selector picks a line with a click and with a box, whether the
  handle's Euler order matches the stored quaternion on every rotation, the guide colours and size.
* Not done: the wheel edits only the spherical joint's `motor_velocity` (the other motor
  settings are multiplying floats), and Toggle Motor does not cover the Wheel joint's spin motor.
