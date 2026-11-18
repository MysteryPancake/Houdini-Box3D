# Status: unverified, TODO

The one list of what is unverified or still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not this file).

Last full run (Houdini 22.0.502, MSVC, the Box3D fork): every check passes, `baseline` included. The INFO results
are not pass marks.
What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

## Unverified

**Not yet run in Houdini**

* Switch Constraint Type When Broken with its next settings (`applyNextSettings`, `takenext`, the Next Constraint folder; tests `next_settings`,
  `constraint_properties_next`, `solver_sop_break_next_settings`, `solver_sop_next_constraint_type`) pass, including `next_constraint_type`
  on a SideFX Hard line.
* Constraint Properties scaling and Geometry Attribute Transfer (`SCALE_VEX`, tests `constraint_properties_scaling`) pass (Houdini
  22.0.502). Not checked against RBD Constraint Properties: Closest uses piece point positions (Bullet uses the nearest surface point),
  a piece that is missing leaves the value unscaled (Bullet skips the setting), integers, toggles and vectors have no scaling, and there
  is no Next Constraint scaling. Randomize and Variance are on every number setting here (Bullet: Glue Strength only), seeded per setting.
* The group / attribute menus beside the fields (`MENU_SOURCE` in `b3d_hda.py`): not built or run. Check each menu fills from the
  connected geometry (the SOP Path for the Packed Object's list, the node's output for Inspector Attributes), a multi-name field
  toggles picks in and out, and the menus work on the Solver SOP's nested nodes.

* Animated bodies following their source SOP (`followSourceSops`): the animated-collider, kinematic and Packed Object tests pass
  (Houdini 22.0.502). Not checked: a source `orient` / `pscale` attribute on the animated piece (only its packed transform
  is read), a Solver SOP whose collision path is a context geometry rather than a SOP node, many animated pieces (one cook and one
  name pass per step), and a DOP Start Frame other than 1.

* Overwrite Attributes from SOP as an Attribute Copy list (`copy_attribs`, `SIM_GEO` in the Packed Object stage): not built or
  run. Check the list reaches the solver (`active` changed over time), the object merge of its own geometry cooks without a
  dependency loop, and an animated collider (`reimportsop`) still moves.

* `computecom` on points (default on, as Bullet): not built or run. Bullet, measured with an L-shaped packed piece
  spun at 3 rad/s with no gravity, moves the point on an arc with it on and leaves it fixed with it off. Check Box3D
  gives the same, that `v` (the centre's velocity here) is right, and that joints, Glue and forces behave. Also unchecked: the
  Packed Object's Initial State Position / Rotation / Inherit Pivot / Pivot and Creation Frame Specifies Simulation Frame against Bullet.
* Box3D Packed Object's Display Geometry (`visualize_hires`) and Collider Label (`add_display`): checked against a
  stand-in `hou` only. The build's `check` probes both.
* Box3D Configure's Bounds / Use Bounds and World Scale on FORCES: no test, only the builders' `check`s (Use Bounds, the
  joint scaling).

**Build**
* Linux plugin build (only the core builds in CI): `build.sh`, gcc warnings in `src/dop` / `src/sync`, the dso and
  icon copies, `hython tests/houdini/run_tests.py`.
* The Houdini-side files have not been through `-Wshadow` / `-Wconversion` (GCC / Clang) or clang-tidy.
* The Threads default (`performanceCoreCount`) on macOS (`hw.perflevel0`) and ARM Linux (`cpu_capacity`).

**Houdini behaviour**
* Glue Inertial Load (`gluecompoundload`, `pieceLoad`): the maths is checked in `core_compound` and `glue_compound_load` shows
  the load reaching the bonds of a column landing on its end. Not checked: that a cube dropped on a corner collapses, nor how its
  break values compare with Bullet's impacts.
* Collision group names (`COLLISION_NAMES_VEX`, test `solver_sop_collision_group_names`): unchecked are whether the collision
  input's pieces honour category / mask, and whether Bullet also lets either side's ignore pattern be enough.
* A packed piece that turns mirrored mid-simulation: the shape rebuild is keyed on the row lengths, not the sign
  (`mirrored_piece` covers a piece that starts mirrored).
* Escape mid-step: the step is discarded and re-cooked, earlier frames stay cached, the re-cook rebuilds once.
* Data ids surviving the DOP copy between timesteps (the sync skip; debug log `scan skipped`); our own writes not
  looking like edits (debug log `full scan, changed:`).
* `SourceSummaryCache`: a packed source edited in place without a new unique id or data id gives a stale hull.
* Box3D's tasks on Houdini's TBB pool with several DOP networks or threaded SOPs cooking at once.
* Parallel Force DOP evaluation (`evaluateResolvers`, `kMinParallelForces`): 300 boxes under a Gravity Force DOP give the
  right result (`force_dops_many_and_sampled`), but nothing shows that the parallel path ran: whether the built-in forces
  report `threadsafe()` true is unchecked (a force that does not just stays serial). Circle / Sphere sampling works on a Drag
  Force DOP (the test prints its ratios to Point: 4.6 and 9.2 for a 2 m box); how a force scales over a size is the Force
  DOP's own.
* The ground guide (`build_ground_guide`, test `solver_sop_ground_guide` checks the geometry): not seen in a viewport, so
  whether `gl_wireframe` -1 on the merged guide still shows a Shaded plane shaded.
* Box3D Data's Default Shape menu (Plane and Cylinder at the end) and the Packed Object / Configure menus built from
  it: built, not opened in Houdini. `bullet_cylinder` and the cylinder cases of `bullet_shape_attributes` pass against Bullet.
* `bullet_georep` `plane` on a packed piece (`ShapeType::Plane`, `buildShapes`): written without building or running. Its
  Bullet placement (through the piece's origin, +Y, as a Ground Plane) is assumed, and a piece turning moving mid-simulation
  rebuilds as a box.
* Ground planes: a Static Object whose SDF Representation is set to Implicit Plane is taken as a plane facing +y through its
  origin, as a Ground Plane DOP's is (only the Ground Plane is tested). A plane tilted off the axes has a large broad-phase box,
  so every body in the world keeps a cheap contact with it. A fast body against a plane uses conservative advancement
  (`b3PlaneTimeOfImpact`), checked only by a straight 400 m/s sphere drop (`core_plane`).

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
* Not done: Proxy attribute transfer; the wheel edits only the spherical joint's `motor_velocity` (the other motor
  settings are multiplying floats), and Toggle Motor does not cover the Wheel joint's spin motor.
* The "not set" skip (`isUnsetFloat`, `applyOverrides`) is not covered by a core test.

**Bullet semantics not compared**
* The extra `restRotation` split per packed point in every full scan (the rest convention itself matches Bullet:
  `bullet_rest_convention`; `bullet_rest_rescan` forces a scan every step and matches).
* Solve on Creation Frame on the SOP for the collider, against Bullet's (`solver_sop_solve_first_frame_bullet`): the
  free-falling box matches with the option off and on. A box on a collider lifted 0.5 a frame ends at 1.1 to 2.3 in Box3D
  (the box clips into the collider: a bug in Box3D, see the fork list) and stays at 1.0 in Bullet, whose collider may not
  be moving (set as `collision_initialstate` 1).
* Before-start outputs and Export Broken Constraints (`solver_sop_before_start`, `solver_sop_keep_broken`), against the
  RBD Bullet Solver SOP: `solver_sop_keep_broken_bullet` matches. `solver_sop_before_start_bullet` matches on the box and on the
  geometry, constraints and points outputs; the proxy output with no proxy input is a copy of the geometry in Box3D and
  empty in Bullet.
* Joints from authored anchors through `restToNow` and the `angle` since rest, against Bullet: `bullet_joint_rest_held`
  (Hard, Rest Length set to the start distance, piece turned 30 degrees) holds in both but never turns, so the `angle` is
  untested: `bullet_joint_angle` (spin, other axes) holds with a turn of 0 in both too. With both anchors at one point (`bullet_joint_rest`, INFO) the centres end 2.19 and 2.59 apart, past the 2.06 possible.
* `bullet_primR` rotation order: XYZ Euler degrees matches Bullet. Settled thin boxes have the same face down
  (`bullet_primr_order`), and boxes turned by 90 degrees have the same face up (`bullet_shape_attributes`: 90 90 0,
  0 90 90, 90 0 90 pick XYZ out of the six orders). A tilted box's top is a corner and the plate test stops on it
  roughly: Box3D reads 0.44 and Bullet 0.33 for 0 70 50 (true top 0.437), so that case is not compared. Bullet's
  dynamic hand-set box rests at y = Box3D's (the half thickness, 0.10) minus the shape's vertical half extent as
  first turned (0.10 unturned, 0.26 for 50 70 50, 0.44 for 0 70 50, 0.52 for 0 0 50, 0.29 for 50 0 0): the rule fits
  every case probed, so Bullet's reported point sits that much lower, not the shape. Not understood: with primR
  30 0 60 the piece fell through the ground in Bullet (`bullet_primr_through`: 30 0 0, 0 0 30, 0 0 60, 30 0 30 and 0 0 45 end
  0.14 to 0.47 below it, 0 30 0 and 0 60 0 on it, Box3D always at 0.10). `bullet_primS`, `bullet_primT`, `bullet_radius` and
  `bullet_length` match Bullet. Bullet ignores `bullet_primT` for a dynamic piece's rest height
  (0.25, shape centred on the body) where Box3D keeps the offset (-0.05); a static piece away from the origin is
  not found by a plate in Bullet. Not compared: the `deforming` and `bullet_autofit_valid`
  toggles, AutoFit on whole objects and fractured pieces.
* Where Bullet puts a hand-set shape on a static piece away from the origin: a plate over a piece at x 3 never
  stopped in Bullet, where Box3D has the shape on the piece (`bullet_shape_position`; its dynamic case is rerun, the
  first run had made the ground a hand-set box too).
* Whether a fresh `SIM_Impacts` each step suits Houdini's impact tools (the normal direction matches Bullet:
  `bullet_impact_normal`). A box landing and resting for 3 s has Box3D records on 13 frames (at most 8 on one), Bullet's on
  49 (at most 40): Bullet keeps writing contacts while at rest (`bullet_impact_fresh`).
* Bullet's placement of a hand-set shape (`bullet_shape_probe`, the same with a neutral ground): a dynamic hand-set piece
  dropped from x 2 ends at x 0, z 0 (plus `bullet_primT`'s x and z), so Bullet drops the piece's position, and its
  height follows `bullet_primT`'s y (0.3 up gives 0.25 to 0.3, 0.3 down gives -0.3) rather than the shape. Box3D keeps
  the piece where it is with primT as an offset. Not copied: it would move every hand-set piece to the world origin.
* Whole RBD Object pivot (`wholeObjectPose`, tests `hull_centroid`, `bullet_rbd_pivot`): Bullet's `p` is 0.03 off the hull
  centroid Box3D gives for boxes (an L: (0.205, 0.795) against (0.238, 0.762)), unexplained, the same for an L moved 5 in x and a T
  (`bullet_rbd_pivot_far`). A concave object with Hull per Piece is not checked against Bullet.
* Which end is Cone Twist's goal (taken as point 1; test `bullet_cone_twist_goal`): Bullet's piece fell freely with both
  anchors at one point, turned or not, so it cannot say. With the anchors apart (`bullet_cone_twist_goal_apart`) it fell
  freely again (distance 20.9).
* Unpacked geometry (`unpacked_geometry_bullet`, `unpacked_geometry_match`): an object of polygons only is skipped by both (ours warns). Bullet
  ignores loose polygons beside packed pieces without a message (ours warns) and gives that message for loose points
  (ours simulates them). Loose points beside a packed piece: Bullet ignores them without a message, ours simulates them.

**Whole objects**
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.
* RBD State's Compute Inertial Tensor off is assumed to mean the computed inertia (`rbd_object_inertia_type`).

**Box3D Solver SOP** (`tools/make_solver_sop.py`, `tests/houdini/checks_sop.py`)
* Collision Type Animated / Deforming: whether the Packed Object's Overwrite Attributes from SOP (SOP Geometry's Set Always) re-imports every step (the build prints which menus it drives),
  and the solver's cost of a full rescan of the collider each step.
* The Solver SOP's Collide As Static Object, Constraints tab, help card and Set Always settings pass
  their tests. Still unchecked: the Static Object's default Collision Detection on a mesh (distance field cost), the
  help card's look in Houdini's help browser (its wiki markup parses with `bookish.wiki.wikipages.parse_to_root` in
  hython), and whether every setting tolerates being re-applied each step.
* Box3D Data's hand-set shape and Rotational Stiffness set on a Box3D Data DOP itself (`BodySettings::prim`, `inertiaScale`):
  unchecked, `packed_object_hand_set_shape` covers the attribute route through the Packed Object only. The Solver SOP's Override
  Attributes from SOP (the pieces' `overwritefromsop`) is checked for its link, not for changed attributes reaching the solver.
* `anchor_id` passes on whole objects (`anchor_id`) and on packed pieces against Bullet (`bullet_anchor_id`, ids 0 and 5,
  points and vertices; the test's pieces are fragments). Untested: other ids (a fragment's points are taken in source order,
  its vertices in primitive order), a piece that has moved before its joint is made (`nowToRest`), and non-fragment packed
  pieces.
* Glue's impulse model (`core/Box3DGlue.h`, `core_glue`, `glue_impact`, `glue_impact_trace`, `glue_break_match`, `sidefx_glue`):
  `impact` matches Bullet's in every scene tried (`glue_break_match`: pairs and rows landing from 1 to 8, with bounce, and a
  box dropped on a pair or row: peaks within 1.25x, the split along a row too; half-life and propagation rate / iterations
  barely change either). The Strength at which a bond stops breaking does not match: Bullet breaks at about its `impact`
  (1.0x) when something hits a resting glued pair or row, and at 3x to 10x when the glued pieces land themselves (more with
  speed, bounce and the number of pieces, and with a piece's height more than its mass), where Box3D uses one factor,
  `kGlueStrengthScale` (5.3, measured for a landing from 3). So a hit on a resting structure needs about 5x the Strength
  in Box3D that it does in Bullet. Anchor position and rest length change neither. Not understood: what makes Bullet's
  break value larger than its `impact`. Unchecked: the cost of reading every glued body's contacts each step on a large network.
* What the precompiled headers (`BOX3DDOP_PCH`) save: not measured.
* Transform Pieces with packed transforms that carry scale or shear.
* Constraints output through Transform Pieces: constraints between collision pieces (not in the simulation points, so
  not moved).
* FORCES: the Multiple Solver with nothing wired into PRESOLVE / POSTSOLVE (no warning expected); that diving into
  the asset lands in FORCES (the build's check passes); FORCES editable in a locked instance (`EditableNodes`).

## TODO

* Packed primitive spheres and nested packed primitives collide as one point, half sunk (`odd_packed_contents`).
* Quadric primitives (Sphere, Tube, Circle, Metaball), packed or as RBD Objects: sampled on their surface
  (`Box3DQuadrics.h`, `appendQuadricSamples` in `Box3DShapeBuild.cpp`) and sized like Bullet's. Run (`bullet_primitive_types`):
  sphere 0.500 and metaball 0.250 match Bullet; a Sphere that is the only primitive of its piece and uniformly scaled
  is a real Box3D sphere (`NamedPiece::sphereValid`). Tubes and Sphere Primitives rest at 0.500 in both, packed or as an
  RBD Object. As an RBD Object a polygon Sphere rests 0 to 0.04 low in Box3D (0.46 to 0.48) and a metaball at 0.224
  (Bullet 0.250); a Circle given as polygons or curves rests at 0.001 with a node message (Bullet 0.020). Not done:
  quadrics inside a Convex hull Per Connected Set, volumes (Bullet: packed falls through, RBD Object stays at 2, Box3D's
  falls), tube taper.
* Results depend slightly on point order even with `id` (`point_order`): bodies are created in point order.
* Objects solved by another solver: packed ones are ignored as colliders; one-way collision with polygons untested.
* Export Collision Shapes (unverified): `SIM_DATA_CREATE(obj, "Box3DShapes", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING)`
  on an object that has no such data, then an empty detail in `SIM_GeometryAutoWriteLock`; `setExtraInfo` keeping other
  entries and surviving the later `save`s (Bullet's extra info is `guide=GUIDE_GEO` among colour entries).
* Concave meshes: one mesh shared between instances, static RBD Objects as meshes, drawing meshes in the guide.
* Height fields: holes from a mask layer, height fields on points.
* Constraint networks: agent anchors (`anchor_type` agent), the `anchor_pid` / `anchor_vid` matching; `constraint_type` on our nodes. SideFX's help says a missing `constraint_type`
  is `position`, where the solver takes `all` (checked against Bullet in `bullet_joint_rest` and `sidefx_hard_*`: rerun
  before changing).
* SideFX constraint data not mapped: Soft plasticity, Cone Twist's elliptical and wide
  cones, softness, motors and translation ranges, Hard's angular motors.
* Inactive RBD Object (Create Active Object off) is a static body: animating its Position teleports it, where Bullet
  drives it as a kinematic body that pushes what it touches.
* Force DOPs: evaluation inside Box3D's Constraint Iterations (needs a callback in the fork); parallel evaluation of circle / sphere
  sampled forces (the resolver has only the point call).
* Sticky collisions; contact / sensor events beyond Impacts.
* Box3D Solver SOP:
  * Collision: Deforming following an unpacked collider that scales (a turn is checked by
    `unpacked_turning_collider`).
  * Set Always for the Box3D Data settings of the pieces and collider, the relationships and the ground (the solver's
    own settings are Set Always already).
  * Break Thresholds (`b3d_solver_breaks.py`): Impact (`solver_sop_break_impact_bullet`: a glued pair stops breaking at 22.4k
    in Box3D and 22.9k in the RBD Bullet Solver SOP) and At Frame (`solver_sop_break_atframe_bullet`: same frame as Bullet,
    also relative to Start Frame) match. Distance is `distance` = the anchors' distance (as Bullet, test
    `solver_sop_break_distance_bullet`). Angle and Torque thresholds run, but the
    `angle` and `torque` readings differ from Bullet's (`solver_sop_break_angle_torque_bullet`, INFO: a Soft constraint on
    a spinning box reads about 0.015 rad in Box3D against 0.2 to 0.4 in Bullet, and torque 6078 against 15004 on frame 2).
    Which threshold breaks when is not established beyond that listing.
    Untested: Mode Switch to Next Constraint,
    Relative to Start Frame, Variance, World Scale other than 1, and the cost on a large network with it on (the SOP Solver
    rewrites the constraint geometry each step, so the solver reads the whole network again). Not there: the Force
    Threshold (Bullet's `force` of a Glue is the difference of its pieces' forces times their mean mass,
    Box3D's is the joint's), Group, Update from Input, plastic flow, the VEX snippet.
* Glue as one body (`gluecompound`, `Box3DCompoundSync.cpp`, how it works: joints.md; on by default, so the glue tests run
  it, and `glue_compound_break` and `glue_compound_load` check its breaking). Unchecked:
  * That a group merges, moves, writes its pieces back and breaks as the weld joints do: compare the same scene with the
    switch off (stiffness, break impact and `kGlueStrengthScale`, which was measured against weld joints).
  * `setVelocityByHandle` is the centre of mass velocity (a single piece's centre is its origin, so only a compound
    can tell), repeated `applyForceByHandle` calls add, and enabling a parked body does not leave a piece on a joint
    to another parked body.
  * Pieces falling asleep with a compound, and the pieces' `awake` after a dissolve.
  * Not covered: Impacts and the collision guide for pieces, pieces with another joint or a custom inertia (they stay
    on welds, as does their whole group), a group changed only by editing a piece's settings while it never merged
    (it is worked out again when the Glue graph or the bodies change).

## Box3D fork (`extern/box3d`)

Features to add in the fork. Once one exists, the wrapper maps it and the entry leaves this list.

* **Parallel joint orbit.** A parallel joint with a twist limit orbited at about 72°: not understood.
* **Clipping into a fast kinematic collider.** A box resting on a collider lifted 0.5 a frame sinks into it instead of
  riding on top (`solver_sop_solve_first_frame_bullet`, INFO).
* **Wide and elliptical cones.** `b3SphericalJoint_SetConeLimit` asserts the angle is at most 90°, and the cone is round.
  Cone Twist's wider and elliptical cones (unmapped above) need both lifted.

Workarounds in the wrapper that the engine could own, roughly by value:

* **Worlds allocated on demand.** `B3_MAX_WORLDS` sizes the static `b3_worlds` array at compile time, so the build
  passes `BOX3DDOP_MAX_WORLDS` through to Box3D and the solver reports "world limit reached" past it.
* **A hull build that always succeeds.** `b3CreateHull` fails when the vertex, face or edge limits are exceeded, so
  `createHullWithKey` retries smaller vertex budgets (a triangulated hull hits the half-edge limit well below the
  vertex limit), `fitShapeOriented` and `hullCentroid` use the same ladder (`kHullBudgets`), and a per-group hint remembers the budget that worked. Wanted:
  simplify until it fits, or say which limit failed.
* **Mass overrides that persist.** Houdini's point is the centre of mass. `applyMassByHandle` therefore moves the
  centre to the body origin (parallel axis theorem by hand), scales the inertia or installs an explicit one with
  `b3Body_SetMassData`, and must run again after every shape change or type change, before the next step. Wanted: a
  body-level override (centre at origin, inertia scale, explicit inertia) that survives
  `b3Body_ApplyMassFromShapes`.
* **Events across steps, and impulse in hit events.** Box3D clears move, joint and hit events at the start of each
  `b3World_Step`, so with Substeps the solver merges them after every step (`brokenThisFrame`, `st.moves`,
  `hitPairs`). `b3ContactHitEvent` has point, normal and approach speed but no impulse, so Impacts scan every awake
  body's contacts after each step (`getContactsByHandle`) and match pairs against the hit events: with Impact Min
  Speed off, a hit that separates within one step is lost. Wanted: events that accumulate until cleared,
  and the normal impulse in the hit event. Impacts could then come from events alone.
* **Live contact recycling flag.** `b3Body_EnableContactRecycling` only reaches contacts made afterwards (`contact.c`
  sets the flag at creation), so `setContactRecyclingByHandle` disables and enables the body (`rebuildContacts`).
  That wakes the touching bodies and zeroes the body's velocity, which the wrapper saves and restores. Wanted: the
  setter updates existing contacts. `setSurfaceByHandle` rebuilds after a rolling resistance or tangent velocity
  change too, but `b3UpdateConvexContact` re-reads both, so that rebuild may be unneeded (unchecked).
* **Generic joint.** No 6-DOF joint with a lock, limit, spring and motor per axis, so SideFX Hard, Soft, Spring and
  Cone Twist are approximated by weld, distance, motor and spherical joints (`Box3DSideFX.cpp`, [joints.md](joints.md)):
  Hard's rotation is a motor joint with a stiff angular spring because Box3D has no rigid rotation-only joint, and
  Cone Twist loses its translation ranges and motors. One joint with per-axis settings would remove the mapping
  and the unmapped list above.
* **Joints to the world.** `b3CreateJoint` needs two valid, different bodies, so a null `BodyHandle` uses
  `Impl::anchor`, a shapeless static body at the origin made on first use. Wanted: a null body means the world.
* **Checks that survive Release.** Box3D checks most inputs with asserts, which a Release build compiles out, so every
  entry point cleans its input first: mesh vertices and indices, height field size and scale, hull points,
  quaternions (`sanitizeQuat`), negative material values, rolling resistance, tangent velocity. Wanted: validation
  that clamps or fails in every build.
* **Mixing modes in the engine.** Friction and restitution mixing are context-free callbacks, so each rule is its own
  function (`frictionRule`, `restitutionRule`) called through a pointer per contact. An enum on `b3WorldDef` would
  avoid the call.
* **Wheel angular separation.** `b3Joint_GetAngularSeparation` is a todo that asserts for wheel joints, so
  `jointFeedback` returns 0.

## HDK calls checked

Against the HDK reference, so they need no re-checking: `GU_PrimPacked` (`isPackedPrimitive`, `getPackedDetail`,
`sharedImplementation`, `getFullTransform4`, `setLocalTransform`, `getPivot`, `transformDirty`),
`GU_PackedFragment` (`getPrimitiveRange`, `detailPtr`), `UTgetInterrupt` / `opInterrupt`,
`UT_PerfMonAutoSolveEvent`, the geometry locks, `GA_Detail` / `GA_PrimitiveList` / `GA_Attribute` data ids and the
point / vertex / primitive maps, `GA_AIFSharedStringTuple`, `SIM_DATA_CREATE` flags, `SIM_Impacts::addPointImpact`,
`SIM_Object` relationships, `SIM_ConstraintNetworkRelationship::getGeometry`, `SIM_ConRel`, the
`DECLARE_DATAFACTORY` expansions, `UT_Matrix3D::extractScales`, `UT_QuaternionT` matrix conversions,
`PRM_Conditional`, `SIMgetGeometryTransform`, `SIM_Object::getConstAffectors`, the `GEO_PrimVolume` /
`UT_VoxelArrayIterator` calls in Box3DAffectors.cpp, node icons as `config/Icons/DOP_<name>.svg`.
