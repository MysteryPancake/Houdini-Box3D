# Box3D Solver SOP asset: network

The SOP's inner DOP network, inputs, collision, ground, forces, input preparation and outputs. Its parameters, tabs,
break thresholds and viewer state: [solver-sop-ui.md](solver-sop-ui.md). The Packed Object it contains:
[assets.md](assets.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Box3D Solver SOP asset

`tools/make_solver_sop.py <out.hda> [<packedobject.hda>]` builds `box3d_solver` in the SOP table (the DOP of the same
name is another table), after the Packed Object, which it installs first. The module docstring draws the network. Four
inputs (Geometry, Constraints, Proxy, Collision) and four outputs (Geometry, Constraints, Proxy, Simulation Points).
* A SOP-level DOP network whose inputs 0-2 are the simulated pieces (Geometry, or Proxy when connected), Constraints
  and Collision, read inside by the objects' Geometry Source (`CONTEXT_GEOMETRY`: First and Third Context Geometry, as
  RBD Bullet Solver's, the SOP Path empty). Pieces and collision are Box3D Packed Objects, Use Object Transform off
  (SOP space). Unconnected inputs are switched out by point count, so an empty input counts as unconnected everywhere
  alike: SOP Switches with the input's `IN_*` null as spare input and `npoints(-1) > 0`, DOP Switches with
  `npoints(opinputpath("..", i)) > 0` (the DOP network's inputs; collision also needs Enable Collision Geometry), as
  RBD Bullet Solver does.
* Ground Collision (`b3d_solver_ground.py`, Collision tab, a collapsible folder with Ground Type as its header, as RBD
  Bullet Solver's): `useground` None / Ground Box / Height Field drives a DOP Switch (`switch_ground`) over a Null, a
  Ground Plane (Position `ground_pos` x World Scale, Rotation `ground_rot`) and a Static Object (SOP Path the
  `GROUND_HEIGHTFIELD` null, Use Deforming Geometry `ground_animategeo`, Collision Detection Volume, Mode
  Heightfield). The SOP side is an Object Merge of the Height Field parameter (`groundheightfield`, set after the
  promotion by `finish_ground`), a HeightField Transform for World Scale and a Switch. A Box3D Data (`ground_data`,
  Set Initial) after the Switch takes Friction, Bounce, Rolling Resistance and Conveyor Velocity from the Physical
  folder (`ground_*`, templates cloned from the node, Conveyor Velocity scaled by World Scale). Both go through a
  Merge with the pieces (mutual affectors) and arrive as affectors ([forces-affectors.md](forces-affectors.md)): the
  Ground Box (a Ground Plane DOP) as a ground box, the Static Object as a height field. The box's Size is the solver's
  `groundsize` (Ground Box Size), pulled out of the DOP's Advanced tab into this folder after Rotation (`take` in
  `make_solver_sop.py`, shown for Ground Box) and scaled by World Scale (`SCALED_SETTINGS`). The guide (RBD Bullet
  Solver's, read from its dump; `build_ground_guide`): a Ground Box is the Ground Plane DOP's display grid (DOP Import
  `import_ground_plane` of the object `groundplane`, Geometry Data Path `DisplayGeometry`, scaled back by
  `unscale_ground_plane`) merged (`merge_ground_box`) with the collision box on top (`ground_box`, Size and Position /
  Rotation read from the outer parameters, top face on y = 0, then `pose_ground_box`, in the inputs' units, not
  dimmed). Show Ground as (`shadeground`: Solid, Wireframe, default Wireframe) picks polygons or lines for both
  (`switch_ground_plane_style` / `ground_plane_lines`, `switch_ground_style` / `ground_box_edges`). A Height Field is
  the DOP Import `import_ground_field` of the same object (Geometry Data Path `Geometry`), scaled back by a
  HeightField Transform. Then `switch_ground_guide` on `useground * showground` and `switchif_beforestart_ground`,
  into `merge_guide` (so `guide_wireframe` applies to it as to the rest). Show Ground and Show Ground as are the
  Visualization tab's Ground folder (`ground_visualization`). No Collision Padding or Dynamic Friction Scale (nothing
  in Box3D). Tests `solver_sop_ground`, `solver_sop_ground_guide`.
* Collision: Default Shape Convex Hull. Collision Type (Static, Animated default, Deforming) drives Body Type
  (Kinematic unless Static) and the collision Packed Object's Overwrite Attributes from SOP (on unless Static), so the
  SOP is re-imported each step and the solver turns pose changes into kinematic targets; Deforming also sets
  `deforming` on the colliders (`collision_deforming`, an Attribute Create). Both are the Packed Object's own
  parameters, so the nested asset stays locked. A point with `active = 0` and no `animated` stays static.
* Collide As (`collisionsource`: Packed Pieces, Static Object): a `switch` (`collision_source`) between the
  `collision` Packed Object and a Static Object (`collision_static`, SOP Path the `COLLISION_STATIC` null) through a
  Box3D Data (`collision_static_data`, Set Initial). The null follows `collision_unpack` (an Unpack SOP, since
  affectors ignore packed primitives) on the World Scale'd Collision input, also the DOP network's fourth input
  (`DOP_STATIC`): the `use_collision` Switch counts that input's primitives (a height field has no points) instead of
  the packed collision's points. Use Deforming Geometry follows Collision Type Deforming only; Animated does not move
  a Static Object. The collider's Material settings (`GROUND_MATERIAL`) link to both Box3D Data nodes; Shape and
  Contacts are disabled for a Static Object. Static Object keeps its default Collision Detection (a height field and a
  mesh both arrive as affectors, [forces-affectors.md](forces-affectors.md)). Test `solver_sop_collision_static`.
* The Constraint Network DOP uses Second Context Geometry and Attach Internal Constraints to Object, with Reload from
  SOPs off (as RBD Bullet Solver; its default expression reloads while the input changes over time, which would make
  every attribute follow the input), so the network is the pieces object's `ConstraintGeometry`. Only the pieces go
  through it, the collision object is merged after: a second object would get its own copy and make every joint twice.
  Every relationship in `RELATIONSHIPS` (ours and SideFX's) is merged into it with default Data Names, so any
  `constraint_name` finds one. Their parameters are not promoted: set values per primitive (floats multiply the
  node's).
* Forces: a DOP subnet `FORCES` (the asset's dive target and an editable node: the `DiveTarget` and `EditableNodes`
  sections) with `subnetoutput` nodes `PRESOLVE` and `POSTSOLVE` (Output Index 0 and 1), nothing wired in, PRESOLVE
  left and POSTSOLVE right with a sticky note on each outer side (`FORCES_NOTES`), the subnet's inputs moved above.
  Not `output`: in DOPs that is the Output DOP that saves to disk. A DOP subnet always shows four outputs. A Multiple
  Solver takes the objects and runs PRESOLVE, `scale_forces`, the Box3D Solver, POSTSOLVE. A POP Force on PRESOLVE
  writes `force`, applied and cleared by the solver in the same step; on POSTSOLVE it is applied the next step (see
  [forces-affectors.md](forces-affectors.md)).
