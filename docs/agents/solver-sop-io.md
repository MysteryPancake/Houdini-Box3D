# Box3D Solver SOP asset: inputs and outputs

Input preparation, the outputs, World Scale, Solve on Creation Frame and the guide geometry. The inner network, collision, ground
and forces: [solver-sop.md](solver-sop.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

* Input preparation (`tools/assets/b3d_prepare.py`, shared with Box3D Configure): node for node RBD Bullet Solver's
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
  attributes cut to `POINTS_KEEP` (with `scale`, which only an animated piece whose source scale changes has) plus the asset's `transferattributes`. Not Create Points to Represent Objects, which
  RBD Bullet Solver's `dopimport` uses and which does make a point per piece for ours too, with Bullet's `P`, `v`,
  `w`, `orient`, `pivot`, `name` and `scale`: it drops `restxform` and `id`, and is no faster. The fetch is a shared copy
  of the object's geometry (0.02 ms for 10000 pieces, the points_only step after it nothing), where the points import
  builds them (1.4 ms), and the whole asset frame of 10000 falling pieces took 12.2 ms with the fetch and 13.6 with the
  points (with `scale` kept, 32 ms: it turns off the fast Transform Pieces path). So the asset does not copy it. The points carry
  Bullet's rest convention ([bodies.md](bodies.md), "Rest transform").
* Geometry and Proxy outputs: Transform Pieces of the named Geometry / simulated source onto the simulation points by
  `name`, no rest input, the source as of the Start Frame (`rest_source`: a Time Shift and a Switch If that takes the
  current frame instead while the input has a `deforming` attribute, as Bullet's `timeshift1` / `switchif_deforming`;
  the points carry the motion since then, so an input that moves itself would move twice; test
  `solver_sop_moving_input`), as RBD Bullet Solver: points only, so it builds each transform from `pivot`, `orient` and `P`
  (on a one-vertex primitive's point it would use the packed transform instead). Before it, `remove_deleted_*`
  (`REMOVE_DELETED_VEX`, Bullet's `remove_deleted`) drops pieces the solver deleted: no simulation point of their
  name, and for Geometry, one in what was simulated. Transform Pieces is the asset's largest cost (nearly all in its
  name matching), so the Geometry output has a faster path beside it (`fast_transform`, `switch_fast_transform`): one
  point wrangle (`move_pieces_geometry`, `MOVE_PIECES_VEX`) finds each point's simulation point once, moves `P` by its
  motion matrix and writes `w` and `v` as Transform Pieces does; `move_pieces_normals_geometry` also moves `N` by the
  inverse transpose, and `move_pieces_packed_geometry` (`MOVE_PACKED_VEX`) is for packed pieces, whose rotation, scale
  and shear are the packed primitive's 3x3 transform: it also sets it to the transform times the matrix's 3x3 with
  `setprimintrinsic` on the point's own primitive (the order matters for a sheared piece; it still runs in parallel).
  Separate wranglers, because Attributes to Create decides what may be written, and a bound `N` left out
  of it errors, while one in it is made on pieces that have none. Only attribute bindings are written, never
  `setpointattrib` and the like, whose queue stops a wrangle running in parallel. The Python expression of the Switch
  (`FAST_TRANSFORM_EXPR`, Python because HScript cannot read an attribute's type) picks one only when the pieces
  carry `name` on their points, neither they nor the simulation points carry `age`, the simulation points carry no
  `scale` (an animated piece's), the attributes to copy are no more than `age` and `w`, and no point attribute besides
  `P`, `N`, `w` and `v` is a position, vector, quaternion, normal or matrix; then 1 or 2 when the primitives are all
  polygons, 3 when all are packed (PackedGeometry or PackedFragment) and the pieces have no `N`; else Transform Pieces.
  Test `solver_sop_fast_transform` (differences from Transform Pieces under 1e-5, also for sheared polygons and for
  packed pieces, plain, scaled and sheared). Timed on the `solversop` benchmark scene (packed pieces): the packed
  wrangle's frame is 15.7 ms at 10,000 pieces and 45 ms at 30,000, Transform Pieces' 32 ms and 88 ms.
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
  scaled. It uses the `@` bindings: a POP Wrangle has no first input, so `point(0, ...)` / `haspointattrib(0, ...)`
  see no points (test `solver_sop_world_scale_forces`).
* Constraints output: DOP Import of the pieces object, Geometry Data Path `ConstraintGeometry`, nothing transformed,
  packed or tagged (`CONSTRAINTS_TOGGLES`), so it carries `broken`, `force` and the rest. The solver does not move its
  points, so a Transform Pieces onto the simulation points carries them with their pieces; world-anchored points
  (`name` empty or `-1`) keep theirs, and so do those of collision pieces (not simulation points, so not moved, a
  moving one included, as Bullet's; test `solver_sop_constraints_collision_bullet`). Export Broken Constraints (`KEEP_BROKEN`, Constraints tab after Break
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
  Solver's `switchif_beforestart`). A Switch-If takes its second input while its expression is true. With no Proxy
  input the Proxy output is empty at every frame, as RBD Bullet Solver's (`switch_proxy_connected`, on
  `npoints` of the input; the geometry is simulated then but not repeated there).
