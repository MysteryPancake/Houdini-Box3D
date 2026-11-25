# Box3D Solver SOP asset: inputs and outputs

Input preparation, the outputs, World Scale, Solve on Creation Frame and the guide geometry. The inner network, collision, ground
and forces: [solver-sop.md](solver-sop.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

* Input preparation (`tools/b3d_prepare.py`, shared with Box3D Configure): node for node RBD Bullet Solver's
  Housekeeping (pack mode, `prepare_pieces` after `sim_source`), Collisions (collision mode, `prepare_collision`) and
  Housecleaning (names mode, `names_geometry` / `names_proxy`) boxes, read from a `houdini_network_dump.py` dump.
  Packed primitives are found by shape (one point, not a polygon, alone in its connectivity class: SideFX's own test)
  and split off; packed pieces are named on their points before the merge, the rest assembled by `name` (connectivity
  when unnamed). The Assembles use Bullet's settings (`prepare_input` here: Create Path Attribute, `PACK_PATH`;
  collision mode `COLLISION_PREFIX`, Connect Inside Edges off). The rest and the `orient` / `pscale` / `scale` bake
  happen in the Box3D Packed Objects' SOP stage, as RBD Packed Object does them inside RBD Bullet Solver. The
  collision then takes its rest from a Time Shift to the asset's Start Frame (`collision_rest`, `COLLISION_REST_VEX`,
  as RBD Bullet Solver's collision repack), so an animated collider re-imported every step keeps its starting
  `restxform` and gets `orient` as the rotation since. Box3D Configure's pack mode differs only in Split's Delete
  Unused Groups (on, as RBD Configure's; `remove_unused_groups`).
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
  name, and for Geometry, one in what was simulated. Transform Pieces is the asset's largest cost (nearly all in its
  name matching), so the Geometry output has a faster path beside it (`fast_transform`, `switch_fast_transform`): one
  point wrangle (`move_pieces_geometry`, `MOVE_PIECES_VEX`) finds each point's simulation point once, moves `P` by its
  motion matrix and writes `w` and `v` as Transform Pieces does; `move_pieces_normals_geometry` also moves `N` by the
  inverse transpose. Two wranglers, because Attributes to Create decides what may be written, and a bound `N` left out
  of it errors, while one in it is made on pieces that have none. Only attribute bindings are written, never
  `setpointattrib` and the like, whose queue stops a wrangle running in parallel. The Python expression of the Switch
  (`FAST_TRANSFORM_EXPR`, Python because HScript cannot read an attribute's type) picks 1 or 2 only when the pieces
  carry `name` on their points, neither they nor the simulation points carry `age`, the attributes to copy are no more
  than `age` and `w`, and no point attribute besides `P`, `N`, `w` and `v` is a position, vector, quaternion, normal
  or matrix; else Transform Pieces. Test `solver_sop_fast_transform` (differences from Transform Pieces under 1e-5).
* World Scale (`WORLD_SCALE_TOGGLE` / `WORLD_SCALE`, first on the Solver tab; Bullet's usescale / Bullet World Scale):
  Transform SOPs scale every input (`world_scale_*`; the before-start outputs pass the unscaled inputs), each behind a
  Switch (`switch_<name>`) that takes it only while scaling (a Transform by 1 still copies every point);
  `scale_attributes` (`SCALE_ATTRIBS_VEX`) then divides `density` by the cube and `w` back, and scales the hand-set
  `bullet_prim*` sizes and `sleepthreshold` (Bullet leaves those). `scale_settings` turns the channel references in
  `SCALED_SETTINGS` (lengths and speeds of the solver, the pieces' and collider's Box3D Data) into reference x scale.
  The outputs come back through `unscale_*` (Transform SOPs; `UNSCALE_*_VEX` for the simulation points, the
  constraints' `force` / `distance` / `torque` and the impact records). Every wrangle sits behind a Switch
  (`SCALE_ACTIVE`), so World Scale 1 costs nothing. Joints: `scale_joint_settings` makes each inner relationship
  setting with a length or force in it its value x scale^power (`JOINT_SCALE_POWERS`, which must hold every float
  setting of our nodes, `JOINT_SCALE_BY_NODE`, `SIDEFX_SCALE_POWERS`), Set Initial. Float primitive attributes
  multiply those, so they follow. `scale_constraint_attribs` (`SCALE_CONSTRAINTS_VEX`) scales what replaces a setting
  (`linear_velocity`) and `auto_restlength`. FORCES: `scale_forces`, a POP Wrangle behind `enable_scale_forces` (only
  while World Scale is on and not 1: it costs a step even when it returns at once) after PRESOLVE in the Multiple
  Solver, scales `force` and `torque` (`SCALE_FORCES_VEX`), so POP force values are real units. POSTSOLVE is not
  scaled.
* Constraints output: DOP Import of the pieces object, Geometry Data Path `ConstraintGeometry`, nothing transformed,
  packed or tagged (`CONSTRAINTS_TOGGLES`), so it carries `broken`, `force` and the rest. The solver does not move its
  points, so a Transform Pieces onto the simulation points carries them with their pieces; world-anchored points
  (`name` empty or `-1`) keep theirs. Export Broken Constraints (`KEEP_BROKEN`, Constraints tab after Break
  Thresholds, off) removes the `broken` group's primitives first, as RBD Bullet Solver's `switchif_keepbroken`.
* Solve on Creation Frame, as RBD Bullet Solver: `solvefirstframe` on the Solver tab (before Cache Simulation) is
  linked to the pieces' Packed Object, and `collision_solvefirstframe` (Advanced > Collision, as RBD Bullet Solver's)
  to the collider's. The collider's has a default expression `ch("solvefirstframe")`, so it follows the first until
  set on its own. The build checks both links.
* Collision shapes (the asset's Guide Geometry, node `GUIDE_GEO`, set as `guide=GUIDE_GEO` in the definition's extra
  info): DOP Import (`import_shapes`, the pieces object, Geometry Data Path `SHAPES_DATA`, `POINTS_TOGGLES`) ->
  `tint_shapes` (`SHAPES_TINT_VEX`, Shape Color) -> `unscale_shapes` -> `switch_show_shapes` (the solver's
  `exportshapes`, promoted to Output > Visualization as Show Collision Shapes; empty input when off, so nothing cooks)
  -> `switchif_beforestart_shapes` -> `merge_guide`. The constraint lines (`false_color_value` (the chosen float's
  absolute value as `__fc`, read through a Bindings-tab variable named by `FALSE_COLOR_BOUND`; Attributes to Create is
  `__fc`, as a bound attribute that is missing would be created otherwise) -> `false_color_top` (Attribute Promote,
  max) -> `false_color_constraints` (`FALSE_COLOR_VEX`, blue to red) -> `false_color_drop`, behind
  `switch_false_color`) are drawn plain. With Show Guides (`SHOW_GUIDES`, off, as Bullet's) `switch_thick_constraints`
  takes `constraint_guide_radius` (point `guide_radius` = `GUIDE_RADIUS` x the type's `<name>_guidescale`, by
  `constraint_name` against `<name>_dataname`) -> `constraint_guide_wire` (PolyWire) instead (the wire is far slower
  than the plain lines on a large network). The impact points (`color_impacts`, a Color SOP, the `IMPACTS` records)
  are two more guide inputs behind `SHOW_CONSTRAINTS` / `SHOW_IMPACTS` switches. The collider's shapes are a second
  import (`import_collision_shapes`, `tint_collision_shapes` with `COLLISION_COLOR`, `SHOW_COLLISION_SHAPES` behind
  `usecollisions`), also merged; the inner solver's `exportshapes` is an expression on either toggle. The external
  collision geometry joins it as in RBD Bullet Solver's guide: DOP Import of the `collision` object
  (`import_collision_geo`) -> `dim_collision_geo` (`COLLISION_GUIDE_VEX`) -> `unscale_collision_geo` ->
  `switch_show_collision_geo` (`usecollisions` && Show Geometry, `SHOW_COLLISION`, on) ->
  `switchif_beforestart_collision_geo`. `merge_guide` -> `guide_wireframe` (`gl_wireframe`) -> `GUIDE_GEO`.
* Before the Start Frame the Geometry, Constraints and Proxy outputs are the inputs (`before_start`, RBD Bullet
  Solver's `switchif_beforestart`; the proxy output shows the simulated source). A Switch-If takes its second input
  while its expression is true.
