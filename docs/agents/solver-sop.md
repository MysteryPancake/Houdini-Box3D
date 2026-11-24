# Box3D Solver SOP asset: network

The SOP's inner DOP network, inputs, collision, ground, forces, input preparation and outputs. Its parameters, tabs, break thresholds and viewer state: [solver-sop-ui.md](solver-sop-ui.md). The Packed Object it contains: [assets.md](assets.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Box3D Solver SOP asset

`tools/make_solver_sop.py <out.hda> [<packedobject.hda>]` builds `box3d_solver` in the SOP table (the DOP of the same
name is another table), after the Packed Object, which it installs first. The module docstring draws the network.
Four inputs (Geometry, Constraints, Proxy, Collision) and four outputs (Geometry, Constraints, Proxy, Simulation Points).
* A SOP-level DOP network whose inputs 0-2 are the simulated pieces (Geometry, or Proxy when connected), Constraints
  and Collision, read inside by the objects' Geometry Source (`CONTEXT_GEOMETRY`: First and Third Context Geometry, as RBD Bullet Solver's, the SOP Path empty).
  Pieces and collision are Box3D Packed Objects, Use Object Transform off (SOP space). Unconnected inputs are switched
  out by point count, so an empty input counts as unconnected everywhere alike: SOP Switches with the input's `IN_*`
  null as spare input and `npoints(-1) > 0`, DOP Switches with `npoints(opinputpath("..", i)) > 0` (the DOP
  network's inputs; collision also needs Enable Collision Geometry), as RBD Bullet Solver does.
* Ground Collision (`b3d_solver_ground.py`, Collision tab, a collapsible folder with Ground Type as its header, as RBD Bullet
  Solver's): `useground` None / Ground Box / Height Field drives a DOP Switch (`switch_ground`) over a Null, a Ground
  Plane (Position `ground_pos` x World Scale, Rotation `ground_rot`) and a Static Object (SOP Path the `GROUND_HEIGHTFIELD`
  null, Use Deforming Geometry `ground_animategeo`, Collision Detection Volume, Mode Heightfield). The SOP side is an Object
  Merge of the Height Field parameter (`groundheightfield`, set after the promotion by `finish_ground`), a HeightField
  Transform for World Scale and a Switch. A Box3D Data (`ground_data`, Set Initial) after the Switch takes Friction, Bounce,
  Rolling Resistance and Conveyor Velocity from the Physical folder (`ground_*`, templates cloned from the node, Conveyor
  Velocity scaled by World Scale). Both go through a Merge with the pieces (mutual affectors) and arrive as affectors
  ([forces-affectors.md](forces-affectors.md)): the Ground Box (a Ground Plane DOP) as a ground box, the Static Object as a height field. The box's
  Size is the solver's `groundsize` (Ground Box Size), pulled out of the DOP's Advanced tab into this folder after Rotation (`take`
  in `make_solver_sop.py`, shown for Ground Box) and scaled by World Scale (`SCALED_SETTINGS`).
  The guide (RBD Bullet Solver's, read from its dump; `build_ground_guide`): a Ground Box is the Ground Plane DOP's
  display grid (DOP Import `import_ground_plane` of the object `groundplane`, Geometry Data Path `DisplayGeometry`, scaled
  back by `unscale_ground_plane`) merged (`merge_ground_box`) with the collision box on top (`ground_box`, Size and
  Position / Rotation read from the outer parameters, top face on y = 0, then `pose_ground_box`, in the inputs' units,
  not dimmed). Show Ground as (`shadeground`: Solid, Wireframe, default
  Wireframe) picks polygons or lines for both (`switch_ground_plane_style` / `ground_plane_lines`,
  `switch_ground_style` / `ground_box_edges`). A Height Field is the DOP Import `import_ground_field` of the same object
  (Geometry Data Path `Geometry`), scaled back by a HeightField Transform. Then `switch_ground_guide` on `useground * showground` and
  `switchif_beforestart_ground`, into `merge_guide` (so `guide_wireframe` applies to it as to the rest). Show Ground and
  Show Ground as are the Visualization tab's Ground folder (`ground_visualization`). No Collision Padding or Dynamic
  Friction Scale (nothing in Box3D). Tests `solver_sop_ground`, `solver_sop_ground_guide`.
* Collision: Default Shape Convex Hull. Collision Type (Static, Animated default, Deforming) drives Body Type (Kinematic
  unless Static) and the collision Packed Object's Overwrite Attributes from SOP (on unless Static), so the SOP is
  re-imported each step and the solver turns pose changes into kinematic targets; Deforming also sets `deforming` on
  the colliders (`collision_deforming`). Both are the Packed Object's own parameters, so the nested asset stays locked.
  A point with `active = 0` and no `animated` stays static.
* Collide As (`collisionsource`: Packed Pieces, Static Object): a `switch` (`collision_source`) between the `collision`
  Packed Object and a Static Object (`collision_static`, SOP Path the `COLLISION_STATIC` null) through a Box3D Data
  (`collision_static_data`, Set Initial). The null follows `collision_unpack` (an Unpack SOP, since affectors ignore
  packed primitives) on the World Scale'd Collision input, also the DOP network's fourth input (`DOP_STATIC`): the
  `use_collision` Switch counts that input's primitives (a height field has no points) instead of the packed
  collision's points. Use Deforming Geometry follows Collision Type Deforming only; Animated does not move a Static
  Object. The collider's Material settings (`GROUND_MATERIAL`) link to both Box3D Data nodes; Shape and Contacts are disabled for a Static Object. Static Object keeps
  its default Collision Detection (a height field and a mesh both arrive as affectors,
  [forces-affectors.md](forces-affectors.md)). Test `solver_sop_collision_static`.
* The Constraint Network DOP uses Second Context Geometry and Attach Internal Constraints to Object, with Reload from SOPs off (as RBD Bullet
  Solver; its default expression reloads while the input changes over time, which would make every attribute follow the input), so the network is the pieces object's `ConstraintGeometry`. Only the pieces go through it, the collision
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
  SOP's Delete Geometry But Keep the Points, found by `POINTS_ONLY_PARM`; a `removeprim` wrangle only if not), point
  attributes cut to `POINTS_KEEP` plus the asset's `transferattributes`. Not Create Points to Represent Objects: that
  makes a point per packed primitive only for SideFX's RBD Packed Object, one per DOP object for ours, so this is
  the one place the asset does not copy RBD Bullet Solver (its `dopimport` uses that style). The points carry
  Bullet's rest convention ([bodies.md](bodies.md), "Rest transform").
* Geometry and Proxy outputs: Transform Pieces of the named Geometry / simulated source onto the simulation points by
  `name`, no rest input, as RBD Bullet Solver: points only, so it builds each transform from `pivot`, `orient` and `P`
  (on a one-vertex primitive's point it would use the packed transform instead). Before it, `remove_deleted_*`
  (`REMOVE_DELETED_VEX`, Bullet's `remove_deleted`) drops pieces the solver deleted: no simulation point of their
  name, and for Geometry, one in what was simulated.
  Transform Pieces is the asset's largest cost (about 2.4 microseconds a piece, with everything asleep), nearly all in
  its name matching, so the Geometry output has a faster path beside it (`fast_transform`, `switch_fast_transform`):
  `match_pieces_geometry` finds each point's simulation point once and writes its motion matrix, Transform by Attribute
  (the node Transform Pieces ends in) applies it, `motion_attribs_geometry` writes `w` and `v` as Transform Pieces does,
  and the temporary attributes go. The result equals Transform Pieces' (checked: transform, `P`, `w` and `v` differ by 0).
  Only attribute bindings are written, never `setpointattrib` and the like, whose queue stops a wrangle running in
  parallel. The Switch takes it only when the pieces carry `name` on their points, neither they nor the simulation points
  carry `age`, and the attributes to copy are no more than `age` and `w` (`FAST_TRANSFORM_EXPR`); else Transform Pieces.
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
  (`name` empty or `-1`) keep theirs. Export Broken Constraints (`KEEP_BROKEN`, Constraints tab after Break Thresholds, off) removes the `broken`
  group's primitives first, as RBD Bullet Solver's `switchif_keepbroken`.
* Solve on Creation Frame, as RBD Bullet Solver: `solvefirstframe` on the Solver tab (before Cache Simulation) is
  linked to the pieces' Packed Object, and `collision_solvefirstframe` (Advanced > Collision, as RBD Bullet Solver's) to the
  collider's. The collider's has a default expression `ch("solvefirstframe")`, so it follows the first until set on its own.
  The build checks both links.
* Collision shapes (the asset's Guide Geometry, node `GUIDE_GEO`, set as `guide=GUIDE_GEO` in the definition's extra info): DOP Import (`import_shapes`, the pieces object, Geometry Data Path `SHAPES_DATA`, `POINTS_TOGGLES`)
  -> `tint_shapes` (`SHAPES_TINT_VEX`, Shape Color) -> `unscale_shapes` -> `switch_show_shapes` (the solver's `exportshapes`, promoted
  to Output > Visualization as Show Collision Shapes; empty input when off, so nothing cooks) -> `switchif_beforestart_shapes` -> `merge_guide`.
  The constraint lines (`constraint_guide_radius` (point `guide_radius` = `GUIDE_RADIUS` x the type's `<name>_guidescale`, by `constraint_name` against `<name>_dataname`) -> `false_color_constraints`, `FALSE_COLOR_VEX`: the chosen float attribute over its largest value, blue to red) and the
  impact points (`color_impacts`, the `IMPACTS` records) are two more guide inputs behind `SHOW_CONSTRAINTS` / `SHOW_IMPACTS` switches.
  The collider's shapes are a second import (`import_collision_shapes`, `tint_collision_shapes` with `COLLISION_COLOR`, `SHOW_COLLISION_SHAPES`
  behind `usecollisions`), also merged; the inner solver's `exportshapes` is an expression on either toggle.
  The external collision geometry joins it as in RBD Bullet Solver's guide: DOP Import of the `collision` object (`import_collision_geo`) -> `dim_collision_geo` (`COLLISION_GUIDE_VEX`) -> `unscale_collision_geo` -> `switch_show_collision_geo` (`usecollisions` && Show Geometry, `SHOW_COLLISION`, on) -> `switchif_beforestart_collision_geo`. `merge_guide` -> `guide_wireframe` (`gl_wireframe`) -> `GUIDE_GEO`.
* Before the Start Frame the Geometry, Constraints and Proxy outputs are the inputs (`before_start`, RBD Bullet
  Solver's `switchif_beforestart`; the proxy output shows the simulated source). A Switch-If takes its second input
  while its expression is true.
