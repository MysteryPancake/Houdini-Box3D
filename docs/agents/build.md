# Build, tests, debug, package

How the project is built, tested and packaged. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the tests, then `return()`s before any HDK use.
* Otherwise `find_package(Houdini)` (with `$HFS/toolkit/cmake`) and `houdini_configure_target`, which builds the plugin
  into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows; close Houdini to overwrite it there).
* Post-build steps:
  * `icons/box3d.svg` to `<dso>/../config/Icons/DOP_<node name>.svg` for every node in `BOX3DDOP_NODES` (Houdini's
    default icon name is `<TABLE>_<name>`; renaming a node means renaming its icon). The two SOP assets reuse DOP
    icons (`ICON` in their scripts), so they are not in the list.
  * The digital assets (`BOX3DDOP_MAKE_HDA`, default ON), with hython (found at `${Houdini_DIR}/../../bin` or
    `$HFS/bin`), into `<dso>/../otls/`: one `tools/make_assets.py` run (one hython session) builds
    `tools/make_packed_object.py` (`box3d_packedobject.hda`), `tools/make_configure_sop.py` (`box3d_configure.hda`),
    then `tools/make_solver_sop.py` (`box3d_solver_sop.hda`, which contains the Packed Object). It is the custom target
    `Box3DAssets` (part of ALL), re-run when the plugin relinks or any `tools/make_*.py` / `b3d_*.py` changes; a stamp
    file in the build folder records it (OUTPUT cannot hold the dso-relative path). Without hython a configure warning
    names the command to run by hand. All three copy parameters from the plugin just built. See below.
* Warnings: `b3dop_warnings(target)` adds `-Wall -Wextra -Wshadow -Wconversion` (`/W4` on MSVC) to our targets, never to
  Box3D; Houdini's headers are system includes. `BOX3DDOP_WARNINGS_AS_ERRORS` (default OFF, on in CI) adds `-Werror` /
  `/WX`.
* `CMAKE_EXPORT_COMPILE_COMMANDS` feeds clang-tidy (`.clang-tidy`); the plugin's files only appear in it after an HDK
  build. `format` / `format-check` targets run clang-format; `.githooks/pre-commit` checks staged files once enabled
  (`git config core.hooksPath .githooks`).
* CI (`.github/workflows/ci.yml`): format check (clang-format 18.1.3), `tools/check_docs.py`, core-only build and
  `ctest` on Linux and Windows. No HDK, so no plugin build.
* `BOX3DDOP_DEBUG` (default OFF) defines `BOX3DDOP_DEBUG=1` for the plugin and `b3dop_core`; with it off nothing writes
  to stdout / stderr. The value is cached in `build/`: reconfigure or delete `build/` to clear a stale ON.
* `BOX3D_DIR` (default `extern/box3d`) points at Box3D's `CMakeLists.txt`.
* `BOX3DDOP_MAX_WORLDS` is passed to `box3d` as PUBLIC `B3_MAX_WORLDS`. Rebuilds keep old worlds alive while cached
  frames share them; `Box3DWorld::create` checks the count itself (Box3D would assert), and a failed rebuild fails the
  solve (`reportWorldCreationFailure` names the limit).
* `-DBOX3D_DOUBLE_PRECISION=ON` works: positions cross the boundary as `double[3]`.
* Release or RelWithDebInfo only. RelWithDebInfo enables `B3_ENABLE_ASSERT`; `Box3DWorld::create` installs an assert
  handler that logs and continues (Box3D's default breaks to the debugger and kills Houdini). Inputs Box3D asserts on
  (non-finite velocities, gravity) are sanitized in the core.
* Linux follows SideFX's HDK CMake route (`Box3DDOP.so`, no `lib` prefix, into `$HOME/houdiniX.Y/dso`). Use the gcc
  version named in the Houdini download; never link an allocator (Houdini overrides malloc). `BUILD_TYPE=RelWithDebInfo
  ./build.sh` for gdb: `gdb $HFS/bin/houdini-bin`, `run -foreground`; `HOUDINI_DSO_ERROR=1` says why a plugin failed to
  load.
* Single-config generators default to `CMAKE_BUILD_TYPE=Release` when none is given.
* `build.sh` and `build.bat` both wipe `build/` if the repository moved (`build/.configured_from`, rewritten every
  build). `build.sh` finds `HFS` (env var, else the newest `/opt/hfs*` or macOS `/Applications/Houdini` install);
  `build.bat` (env var, then the registry's ActiveVersion, then the newest `Program Files\Side Effects Software\Houdini
  X.Y.Z`, in `tools\find_houdini.bat`, which `test.bat` and `make_examples.bat` share).
* `python tools/b3d.py test|examples|ctest|hfs` is the cross-platform launcher (`tools/b3d.py`: the same HFS search
  for Windows, Linux and macOS, then hython on `tests/houdini/run_tests.py` or `examples/make_examples.py`, or
  `ctest`). Its search duplicates `find_houdini.bat` and `build.sh`: change all three together.

## Box3D Packed Object asset

`tools/make_packed_object.py` builds it from the plugin just compiled, so it always matches Box3D Data. It copies RBD
Packed Object (read from `houdini_network_dump.py` dumps of RBD Bullet Solver and RBD Packed Object): Empty Object -> SOP
Geometry (data name `Geometry`) -> Box3D Data -> `visualize_hires` -> Collider Label, the output (no input); SOP Geometry reads a SOP stage inside the asset (`STAGE`,
RBD Packed Object's `create_or_overwrite_attributes`): Object Merge of the asset's SOP Path (Use Object Transform drives
its Transform, Into This Object: the DOP network's space; SOP Geometry's own stays off, as 22.0's has no OBJ Path to
point at an object) -> `setup_defaults` -> `create_name` -> `set_rest` and `setup_velocities` on the creation frame
only -> `setup_deforming` -> `OUT`. The stage's
references to the asset's parameters are set once the asset exists (`wire_stage`).
* `setup_defaults` (detail wrangle, `b3d_settings.packed_defaults_vex`): each per-body setting whose value differs from
  Box3D Data's default becomes a point attribute holding it, unless the points already have it (RBD Packed Object's
  Attribute Creates with Write Values off), so a point attribute wins and untouched settings write nothing. Names map as
  Box3D Configure's (`b3d_settings.MAPPING`).
* `create_name`: a point `name` where there is none: the primitive's, else `<object name>_<primitive number>` (RBD
  Packed Object's, which ignores the primitive's).
* `set_rest` (`b3d_settings.REST_VEX`): the rest reset and the `orient` / `pscale` / `scale` bake of RBD Packed
  Object's `set_rest_and_initial_xform`, with `orient` kept as the rotation since rest (see its comment). Switched in
  only on the creation frame: SOP Geometry stamps `$SF` and `$OBJCF`, which a `STAMPS` null reads, so a re-import
  (Overwrite Attributes from SOP, which the Solver SOP's Collision Type turns on) keeps the rest it was given.
* `setup_velocities` (`VELOCITY_VEX`): RBD Packed Object's Initial State tab (`vel`, `angvel` in degrees per second
  written as radians, `inheritvelocity`). Inherit defaults on, unlike RBD's, so the points' own `v` / `w` (Box3D
  Configure's Initial State) are kept unless the user turns it off.
* `setup_deforming` (`DEFORMING_VEX`): the Deforming toggle writes `deforming` 1 on every import and, with Overwrite
  Attributes from SOP, sets the SOP Geometry to Set Always (`REIMPORT`), as RBD's Create Deforming Objects.
* Parameters: an Object tab (Object Name `$OS`, Creation Frame, Solve on Creation Frame, SOP Path, Use Object
  Transform, Overwrite Attributes from SOP, Deforming, Add Impact Data, Display Geometry), Initial State (with Start
  Asleep), then each Box3D Data folder as a tab (`pull` moves the two out), read by the stage. Only the object-level settings
  (`b3d_settings.SKIP`: Hull per Connected, Flip Winding, Add Impact Data) are channel references into the inner Box3D
  Data, whose other values stay at the defaults, so a missing attribute means the default. Settings change
  mid-simulation through the attributes, not the asset's parameters.
* Display Geometry (`add_display`, RBD Packed Object's nodes and names): Rendering Parameters Volatile on SOP Geometry's
  data input shows the object while it is Simulation; Render shows the SOP Guide `visualize_hires` instead: a For-Each
  over the asset's `createdObjects()` (the iteration from a metadata Block Begin), DOP Import of that object's points,
  Transform Pieces of Render SOP Path by them, matched by `name`. Python expressions, as SideFX's.
* Collider Label `PackedObject`, as RBD Packed Object's: other solvers collide with it as packed pieces; the solver
  still syncs it per point (`isWholeObject` takes any other label as one body). The script prints one line, or warnings, and never fails the build; its `check` cooks the stage of a fresh
  instance.

All three assets list themselves under Box3D in the Tab menu (`b3d_hda.set_tab_submenu`: a Tools.shelf section as
SideFX's, `doptoolutils` / `soptoolutils.genericTool`).

Traps it handles:
* Parameters are found by label, skipping the `parmop_*` menus that share their labels.
* Empty Object has no creation frame: its Activation is `$SF == ch("../createframe")`, as RBD Packed Object's.
* The inner Box3D Data's `parmop_*` are set to Set Initial: a new data node starts at Use Default, and only an edit (not
  a channel reference) switches it, so the linked object-level settings would otherwise keep their built-in values.
* Box3D Data's own DOP settings outside its folders (Activation, Group, Data Name) are not promoted: promoted copies
  kept it from attaching.
* A second Box3D Data after the asset only changes Set Always parameters (Set Initial applies when the data is
  created, and the asset's already has been), and only for settings the asset left at their defaults.

## Box3D Solver SOP asset

`tools/make_solver_sop.py <out.hda> [<packedobject.hda>]` builds `box3d_solver` in the SOP table (the DOP of the same
name is another table), after the Packed Object, which it installs first. The module docstring draws the network.
Four inputs (Geometry, Constraints, Proxy, Collision) and four outputs (Geometry, Constraints, Proxy, Simulation Points).
* A SOP-level DOP network whose inputs 0-2 are the simulated pieces (Geometry, or Proxy when connected), Constraints
  and Collision, read inside with `dopcontextgeo(".", i)`.
  Pieces and collision are Box3D Packed Objects, Use Object Transform off (SOP space). Unconnected inputs are switched
  out by point count, so an empty input counts as unconnected everywhere alike: SOP Switches with the input's `IN_*`
  null as spare input and `npoints(-1) > 0`, DOP Switches with `npoints(opinputpath("..", i)) > 0` (the DOP
  network's inputs; collision also needs Use External Collisions), as RBD Bullet Solver does.
* Collision: Default Shape Convex Hull. Collision Type (Static, Animated default, Deforming) drives Body Type (Kinematic
  unless Static) and the collision Packed Object's Overwrite Attributes from SOP (on unless Static), so the SOP is
  re-imported each step and the solver turns pose changes into kinematic targets; Deforming also sets `deforming` on
  the colliders (`collision_deforming`). Both are the Packed Object's own parameters, so the nested asset stays locked.
  A point with `active = 0` and no `animated` stays static.
* The Constraint Network DOP uses Second Context Geometry and Attach Internal Constraints to Object (as RBD Bullet
  Solver), so the network is the pieces object's `ConstraintGeometry`. Only the pieces go through it, the collision
  object is merged after: a second object would get its own copy and make every joint twice. Every relationship in
  `RELATIONSHIPS` (ours and SideFX's) is merged into it with default Data Names, so any `constraint_name` finds one.
  Their parameters are not promoted: set values per primitive (floats multiply the node's).
* Forces: a DOP subnet `FORCES` (the asset's dive target and an editable node: the `DiveTarget` and `EditableNodes`
  sections) with `subnetoutput` nodes `PRESOLVE`
  and `POSTSOLVE` (Output Index 0 and 1), nothing wired in, PRESOLVE left and
  POSTSOLVE right with a sticky note on each outer side (`FORCES_NOTES`), the subnet's inputs moved above. Not `output`: in DOPs that is the Output DOP that saves to
  disk. A DOP subnet always shows four outputs. A Multiple Solver takes the objects and runs PRESOLVE, `scale_forces`, the Box3D Solver, POSTSOLVE.
  A POP Force on PRESOLVE writes `force`, applied and cleared by the solver in the same step; on POSTSOLVE it is
  applied the next step (see [forces-affectors.md](forces-affectors.md)).
* Input preparation (`tools/b3d_prepare.py`, shared with Box3D Configure): node for node RBD Bullet Solver's
  Housekeeping (pack mode, `prepare_pieces` after `sim_source`), Collisions (collision mode, `prepare_collision`) and
  Housecleaning (names mode, `names_geometry` / `names_proxy`) boxes, read from a `houdini_network_dump.py` dump.
  Packed primitives are found by shape (one point, not a polygon, alone in its connectivity class: SideFX's own test)
  and split off; packed pieces are named on their points before the merge, the rest assembled by `name`
  (connectivity when unnamed). The Assembles use Bullet's settings (`prepare_input` here: Create Path Attribute,
  `PACK_PATH`; collision mode `COLLISION_PREFIX`, Connect Inside Edges off). The rest and the `orient` / `pscale` /
  `scale` bake happen in the Box3D Packed Objects' SOP stage, as RBD Packed Object does them inside RBD Bullet Solver.
  The collision then takes its rest from a Time Shift to the asset's Start Frame (`collision_rest`, `COLLISION_REST_VEX`, as RBD Bullet Solver's
  collision repack), so an animated collider re-imported every step keeps its starting `restxform` and gets `orient`
  as the rotation since. Box3D Configure's pack mode differs only in Split's Delete Unused Groups (on, as RBD
  Configure's; `remove_unused_groups`).
* Simulation Points: DOP Import of the pieces object, Fetch Geometry (`POINTS_TOGGLES`), primitives removed (an Add
  SOP's Delete Geometry But Keep the Points, found by `POINTS_ONLY_LABELS`; a `removeprim` wrangle only if not), point
  attributes cut to `POINTS_KEEP` plus the asset's `transferattributes`. Not Create Points to Represent Objects: that
  makes a point per packed primitive only for SideFX's RBD Packed Object, one per DOP object for ours, so this is
  the one place the asset does not copy RBD Bullet Solver (its `dopimport` uses that style). The points carry
  Bullet's rest convention ([bodies.md](bodies.md), "Rest transform").
* Geometry and Proxy outputs: Transform Pieces of the named Geometry / simulated source onto the simulation points by
  `name`, no rest input, as RBD Bullet Solver: points only, so it builds each transform from `pivot`, `orient` and `P`
  (on a one-vertex primitive's point it would use the packed transform instead). Before it, `remove_deleted_*`
  (`REMOVE_DELETED_VEX`, Bullet's `remove_deleted`) drops pieces the solver deleted: no simulation point of their
  name, and for Geometry, one in what was simulated.
* World Scale (`WORLD_SCALE_TOGGLE` / `WORLD_SCALE`, first on the Solver tab; Bullet's usescale / Bullet World Scale):
  Transform SOPs scale every input (`world_scale_*`; the before-start outputs pass the unscaled inputs);
  `scale_attributes` (`SCALE_ATTRIBS_VEX`) then divides `density` by the cube and `w` back, and scales the hand-set
  `bullet_prim*` sizes and `sleepthreshold` (Bullet leaves those). `scale_settings` turns the channel references in
  `SCALED_SETTINGS` (lengths and speeds of the solver, the pieces' and collider's Box3D Data) into reference x scale.
  The outputs come back through `unscale_*` (Transform SOPs; `UNSCALE_*_VEX` for the simulation points, the
  constraints' `force` / `distance` / `torque` and the impact records). Every wrangle sits behind a Switch
  (`SCALE_ACTIVE`), so World Scale 1 costs nothing. Joints: `scale_joint_settings` makes each inner relationship
  setting with a length or force in it its value x scale^power (`JOINT_SCALE_POWERS`, which must hold every float
  setting of our nodes, `JOINT_SCALE_BY_NODE`, `SIDEFX_SCALE_POWERS`), Set Initial. Float primitive attributes
  multiply those, so they follow. `scale_constraint_attribs` (`SCALE_CONSTRAINTS_VEX`) scales what replaces a setting
  (`linear_velocity`) and `auto_restlength`. FORCES: `scale_forces`, a POP Wrangle after PRESOLVE in the Multiple
  Solver, scales `force` and `torque` (`SCALE_FORCES_VEX`), so POP force values are real units. POSTSOLVE is not scaled.
* Constraints output: DOP Import of the pieces object, Geometry Data Path `ConstraintGeometry`, nothing transformed,
  packed or tagged (`CONSTRAINTS_TOGGLES`), so it carries `broken`, `force` and the rest. The solver does not move its
  points, so a Transform Pieces onto the simulation points carries them with their pieces; world-anchored points
  (`name` empty or `-1`) keep theirs. Export Broken Constraints (`KEEP_BROKEN`, Output tab, off) removes the `broken`
  group's primitives first, as RBD Bullet Solver's `switchif_keepbroken`.
* Solve on Creation Frame, as RBD Bullet Solver: `solvefirstframe` on the Solver tab (before Cache Simulation) is
  linked to both the pieces' and the collider's Packed Object (one toggle, as RBD Bullet Solver's). The build checks
  both links.
* Collision shapes (the asset's Guide Geometry, node `GUIDE_GEO`, set as `guide=GUIDE_GEO` in the definition's extra info): DOP Import (`import_shapes`, pieces and collision objects, Geometry Data Path `SHAPES_DATA`, `POINTS_TOGGLES`)
  -> `tint_shapes` (`SHAPES_TINT_VEX`, Shape Color) -> `unscale_shapes` -> `switch_show_shapes` (the solver's `exportshapes`, promoted
  to Output > Visualization as Show Collision Shapes; empty input when off, so nothing cooks) -> `switchif_beforestart_shapes` -> `merge_guide`.
  The external collision geometry joins it as in RBD Bullet Solver's guide: DOP Import of the `collision` object (`import_collision_geo`) -> `dim_collision_geo` (`COLLISION_GUIDE_VEX`) -> `unscale_collision_geo` -> `switch_show_collision_geo` (`usecollisions` && Show Collision Geometry, `SHOW_COLLISION`, on) -> `switchif_beforestart_collision_geo`. `merge_guide` -> `guide_wireframe` (`gl_wireframe`) -> `GUIDE_GEO`.
* Before the Start Frame the Geometry, Constraints and Proxy outputs are the inputs (`before_start`, RBD Bullet
  Solver's `switchif_beforestart`; the proxy output shows the simulated source). A Switch-If takes its second input
  while its expression is true.
* Output tab, as RBD Bullet Solver's: Impact Data, a closed collapsible folder whose header toggle is the pieces' Add
  Impact Data (`sidefx::header_toggle`), with the solver's Impact Min Speed and Fetch Impacts; Transfer Attributes, with Transfer to Geometry / Proxy Geometry choosing each Transform Pieces' Attributes to
  Copy as RBD Bullet Solver's: the list while on, else `w` (`copy_attributes`, `TRANSFER_TO`), and Transform Pieces'
  own Point Velocities parameters promoted onto both (`POINT_VELOCITY_PARMS`); Outputs, Import Constraints / Proxy
  (`IMPORT_CONSTRAINTS` / `IMPORT_PROXY`: off, an empty output, the chain not cooked). Fetch Impacts makes (or reuses) an Object Merge of the asset's `IMPACTS` null beside the
  asset, fed by a DOP Import Records of the pieces' `Impacts` data (its parameters set by label, `set_by_labels`).
* Parameters: Reset Simulation and Start Frame above the tabs, then the solver's tabs (folders found under its Data
  Options, a tab of simple folders kept whole; Guide Options skipped).
  Tab order: Solver, Pieces, Collision, Forces, any other solver tabs, Visualization (`visualization_tab`: Show
  Collision Geometry, Show Collision Shapes and Shape Color, no folders), then Advanced and Output always last; the
  solver's own Contacts, Sleeping and Aging folders (`CONTACTS_FOLDER`, `SLEEPING_FOLDER`, `AGING_FOLDER`) are pulled
  out of its Solver tab and re-placed after Cache Simulation, Sleeping and Aging closed. Solver: the solver's own
  settings, Cache Simulation, then Contacts, Sleeping and Aging. Collision: one Collision Geometry folder whose header checkbox is Use External
  Collisions (`usecollisions`, placed just before the folder, as RBD Bullet Solver's), holding Collision Type and the collision object's Shape, Material and CCD &
  Contacts folders (`COLLIDER_FOLDERS`, all named `collision_*`), the last cut to Contact Recycling as Contacts
  (`COLLIDER_DROPPED`: the rest does nothing for a Static / Kinematic body). Pieces: the pieces' Box3D Data folders
  (Shape, Body, Material, CCD & Contacts, Sleep) without their Allow Sleeping (`PIECES_DROPPED`; `drop_templates`),
  the solver's Allow Sleeping is in the Solver tab's Sleeping folder instead (one switch for every body); names the solver also uses would
  get `piece_`. Conditions are rewritten with the names (the pieces' Start Asleep and Sleep Threshold grey out
  with the solver's Allow Sleeping). Inner solver `parmop_*` set to
  Set Initial, as in the Packed Object. Inside each tab the settings sit in collapsible folders, as RBD Bullet
  Solver's (`TAB_GROUPS`, `group_tab`) and with its tags (`group_type`, `group_default`, `sidefx::header_toggle`),
  every one open except Output's Impact Data and the Solver tab's Sleeping and Aging; Collision is one Collision Geometry folder with the collider's folders
  simple inside it (each disabled with Use External Collisions off), Pieces is Shape, Body, Material, CCD & Contacts
  and Sleep (the Packed Object's Start Asleep first, `moved_settings`), and Add Impact Data is on Output. Test `solver_sop_layout`.
* Input / output labels are written into the DialogScript after the last `updateFromNode` (no hou setter). Wire
  colours come from the subnet's indirect input nodes (`INPUT_COLORS`: the constraints input pink, and its IN /
  OUT nodes).

## Box3D Configure asset

`tools/make_configure_sop.py <out.hda>` builds `box3d_configure` (SOP): Box3D Data's settings, copied from the node
(labels, defaults, ranges, conditions), written as point attributes. Laid out as RBD Configure: a tabbed multiparm
(`MULTIPARM`, Number of Attributes) of entries, each with Enable (`enable#`), Group (`group#`) and Box3D Data's folders
(`b3dcfg_folder<i>_#`), every setting `<name>#` with an `add<name>#` checkbox (`toggle_name`, label hidden); the Packing
folder sits above the entries. Names map one to one except `MAPPING`: Default Shape -> `bullet_georep` (`GEOREP_VALUES`, menu order), Auto Fit
-> `bullet_autofit`, Body Type -> `active` / `animated`, Start Asleep -> `awake` (inverted); `SKIP` holds the
object-level ones (Hull per Connected, Flip Winding, Add Impact Data). Disable conditions also need the referenced
setting ticked (`rewrite_condition`). Every Box3D Data folder in an entry is simple (always open).
* Geometry and Proxy are packed first, as RBD Configure does, by the chain the Box3D Solver SOP runs
  (`tools/b3d_prepare.py`, `prepare_input` in pack mode without the rest attributes): packed primitives kept, the rest
  assembled into packed pieces named on their points. RBD Configure's packing parameters drive the Assembles (Name
  Prefix, Transfer Simulation / Render Geo Attributes, Create Path Attribute, Path Attribute, Create Packed Fragments;
  `packing_templates`), and Transfer Groups is RBD Configure's (`TRANSFER_GROUPS`) so Group still finds the pieces.
  Without a proxy (or without geometry) the configured stream is the geometry output; with both, the packed geometry
  passes through and the proxy gets the settings.
* One point wrangle (`set_attributes`, `wrangle_vex`) loops over the entries: an enabled entry whose group holds the
  point writes each ticked setting, creating the attribute with Box3D Data's default first (`addpointattrib`) so
  points outside the group keep the default rather than VEX's 0 (`active` 0 is static). Group is tested with
  `inpointgroup` / `inprimgroup`, as RBD Configure, so patterns (`piece*`, `@name=piece0`) work; empty is every piece. The Group button
  picks primitives in the viewport (RBD Configure's `selectGroupParm` call).
* Settings Box3D Data has no parameter for (`extra_folders`): Initial State (`v`, `w` in degrees per second written as
  radians, `enabled`, `deforming`), Motion (`inertialtensorstiffness`, `fastrotation`, `lock_linear` / `lock_angular`),
  Collision Filtering, Hand-Set Shape (`bullet_prim*`, `bullet_radius`, `bullet_length`, disabled with Auto Fit on).
  Defaults are the solver's when the attribute is missing.
* Bounds and Pin, as RBD Configure's (read from a dump of it): the Bounds folder (`boundingtype`, `bbox*`) and the
  fourth input (Static Bounds Geometry) put the pieces inside into the point group `INSIDE_GROUP` (`inside_bounds`;
  no bounds geometry: none). Use Bounds (`use<name>bounds#`, on Body Type and every toggle, `has_bounds`) gives the
  value inside and the opposite outside (Body Type: Dynamic). Pin (`pin_templates`) marks pieces (`PIN_ATTRIB`) and
  `add_pin_constraints` (`PIN_VEX`) appends one line per pinned piece from the world to its point, `constraint_name`
  from `PIN_TYPES` (Weld, Soft: the Solver SOP's relationship Data Names). The marker and group are stripped.
* It uses `setpointattrib`, never `@` bindings: a binding creates its attribute on every point at compile time, even
  inside an `if` that never runs.
* The build checks a fresh instance: two unpacked boxes come out as two packed pieces named on their points, the
  group's piece gets friction and Static, the other keeps defaults; with a proxy only the proxy gets them.

## Network dump

`tools/houdini_network_dump.py` is not part of the build: run it in Houdini (Python Source Editor, a shelf tool, or
`main(node_path, out_path)` in hython) to write a node and everything inside it to a text file: the HDA's interface
and text sections, a hierarchy tree, and per network an ASCII layout, boxes, notes, wiring and each node's
non-default parameters (VEX, expressions, references). It dumps an unlocked temporary copy (`WORK_ON_COPY`), so the
scene's assets stay locked. This is how RBD Bullet Solver's and RBD Configure's packing were read for
`b3d_prepare.py`.

## Example scenes

`examples/make_examples.py` (or `make_examples.bat`; `--check` also simulates each and prints timings) builds the demo
scenes listed in its docstring into `examples/scenes/<number>_<name>.hip` (git-ignored), each from ordinary SOPs and one
Box3D Packed Object, with a sticky note saying what to try. Not part of the build or the tests.

## Tests

Houdini checks: `hython tests/houdini/run_tests.py` (test names, `--list`, `--failed`, `--info`, `--repeat N`,
`--update-baseline`), or `test.bat` on Windows. Each test builds its scene from scratch in its own hython process (a
crash, or no result after 30 minutes, is reported as CRASH; an exception as ERROR), prints PASS / FAIL / INFO, and
saves the scene to `tests/houdini/out/` tidied for opening by hand (`review.py`, with a sticky
note holding the test's docstring and results, so write docstrings for a reader of the scene). SETUP means the script
guessed a node or parameter name wrong; the message lists the real ones. The last status of each test is in
`tests/houdini/out/last_results.json`.

Tests are in `tests/houdini/checks_*.py` by area, each module with a `TESTS` dict listed in `MODULES`
(`run_tests.py`); helpers in `b3dtest.py`. The exit code is 1 if anything failed, errored or crashed. A test
function can carry `.env` (environment for its process, e.g. `HOUDINI_MAXTHREADS=1`). When a check settles an item in
[status.md](status.md), delete the item.

`baseline` (`checks_regress.py`) compares one fixed mixed scene on frame 48 with `tests/houdini/baseline.json` (1e-3).
After an intended change of behaviour, or of `baseline_scene`, re-record it with `--update-baseline`.

Core tests (`ctest`; each `tests/core_<name>.cpp` is `b3dop_core_<name>`, see `CMakeLists.txt`): smoke, physics
(handle API, shapes, mass, setters, locks, CCD, filtering, forces, ...), mesh, heightfield, joints, sidefx, rbdobject,
taskrunner and threads (`[workers] ...` arguments; threads fails if 1 vs N workers differ by more than 1e-4). The
file's header comment says what it covers.

## Debug logging (`Box3DDebug.h`)

`B3D_LOG` for one-off events (rebuilds, bodies added / removed, fallbacks); `B3D_LOG_VERBOSE` per step, printed only
with `BOX3DDOP_VERBOSE=1`; `B3D_LOG_IF(cond, ...)` always when `cond`, else only when verbose. All compile to nothing
without `BOX3DDOP_DEBUG`. Output: stderr, `[b3dop]`. Other environment variables of debug builds (the header lists
them): `BOX3DDOP_REQUESTED_CACHE` (what `getRequestedCacheSubclass` returns) and `BOX3DDOP_VERIFY_READBACK=1`
([solver.md](solver.md), writeResultsBack).

## Package

`install()` rules and CPack (zip) lay out a Houdini package: `Box3D.json` (from `package/Box3D.json.in`, enabled only
for the Houdini major.minor it was built with) and `Box3D/dso`, `Box3D/config/Icons`, `Box3D/otls`.
