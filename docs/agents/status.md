# Status: unverified, TODO

The one list of what is unverified or still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not this file).

Last full run (Houdini 22.0.502, MSVC, the Box3D fork): every check passes, `baseline` included. The INFO results
are not pass marks.
What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

## Unverified

**Not yet run in Houdini**

* Glue as One Body releasing the pieces in a fast body's way (`holdCompoundsForApproach`,
  [glue-compound.md](glue-compound.md)): only the `building` example has run it (`glue_compound_hit`). At Strength
  20000 most of its bonds break from the building's weight on the ground in both modes before anything hits it
  (probably a resting body's ground contact counting as a hit: not looked into). Not checked: other scenes, a slow heavy
  body (the example's slow ball lands on the roof), many glued groups at once, the cost of a very large compound, a
  kinematic body in a scene (only the core query is tested), the force and torque a bond reads while its weld is a
  bridge's (the bond's own joint is parked, so they read zero), bridges of bonds remade while pieces are held, and what
  the `clang-tidy` checks say about the files that need the HDK (`src/sync`, `src/dop`).

* Packed Object Initial Object Type (`initialstate`): not checked are the Solver SOP's collision object (Custom), the Pieces tab's
  Body Type greying out with the menu (hython's `Parm.isDisabled()` never evaluates conditionals: Auto Fit's reads False too, so
  only the UI shows it), and Deforming Static / Active against RBD Packed Object.

* Constraint Properties Use VEXpressions (`run_vexpressions`): not checked are the snippet menu (`vexpressionmenu.buildSnippetMenu` with
  our own key lists nothing unless snippets are saved under it) and the type folders' layout.
* Constraint Properties scaling and Geometry Attribute Transfer (`SCALE_VEX`). Not checked against RBD Constraint Properties: Closest uses piece point positions (Bullet uses the nearest surface point),
  a piece that is missing leaves the value unscaled (Bullet skips the setting), integers, toggles and vectors have no scaling, and there
  is no Next Constraint scaling. Randomize and Variance are on every number setting here (Bullet: Glue Strength only), seeded per setting.
* The group / attribute menus beside the fields (`MENU_SOURCE` in `b3d_hda.py`): `asset_menus` builds every Constraint Properties menu and checks Configure's,
  Constraint Properties' and the Packed Object's list the connected geometry. Not checked: a multi-name field toggling picks in and out
  (a click), the menus on the Solver SOP's nested nodes, and the node's output (Inspector Attributes).

* Animated bodies following their source SOP (`followSourceSops`). Not checked: a source `orient` / `pscale` attribute on the animated piece (only its packed transform
  is read), a Solver SOP whose collision path is a context geometry rather than a SOP node, many animated pieces (one cook and one
  name pass per step), and a DOP Start Frame other than 1.

* Overwrite Attributes from SOP as an Attribute Copy list (`copy_attribs`, `SIM_GEO` in the Packed Object stage): not checked is an
  animated collider (`reimportsop`) still moving.

* `computecom` on points (default on, as Bullet; `bullet_computecom` matches). Bullet also moves the point to the centre of mass and writes `pivot`,
  which Box3D does not. Not checked: `v` (the centre's velocity here), and joints, Glue and forces with it on. Also unchecked: the Packed Object's Initial State Position / Rotation / Inherit Pivot / Pivot and Creation Frame Specifies Simulation Frame against Bullet.
* Box3D Packed Object's Display Geometry (`visualize_hires`) and Collider Label (`add_display`): checked against a
  stand-in `hou` only. The build's `check` probes both.
* Box3D Configure's Bounds / Use Bounds and World Scale on FORCES: no test, only the builders' `check`s (Use Bounds, the
  joint scaling).

**Bonds** (`core/Box3DBond.h`, `Box3DBondNetwork.h`, `Box3DPatch.h`, [joints.md](joints.md); `core_bond`, `core_patch` and the `bond_*` checks)
* Checked against formulas (`checks_bond.py`, `checks_more.py`): hanging loads, a spinning piece's centripetal load,
  gravity scale and a Force DOP on a compound, an impact's tear impulse (the tear mass as compound and as welds, a hit
  below it leaves the bond whole), rotated pieces, a Bond to the world, spheres' bounding-box area (`bond_sphere_fallback`), `bond_*` attributes, the Solver SOP's World
  Scale and Material, Constraint Properties, scrubbing, sleeping, long runs, a wall hit as compound and as welds, damage kept while
  the load is below the capacity (`bond_damage_kept_unloaded`), `compound_id` (`bond_compound_id`), the Bond readouts (`bond_readouts`: ratio, damage, tension, area; shear, bending and twist only at 0 in the hang).
* Not checked: compression and twisting loads, pieces of several convex shapes, the no-patch fallback for
  meshes and a Bond to the world, the fallback mass of a loop, the 2600-piece `building` with Bonds.
* A Bond sliding on the ground under friction: the compound reads about 6.4 kN and the same bond as a weld about 3.1 kN.
  No reference says which is right. With friction on one box of a pair only, both modes let the bond go at m * a of the
  pair's slowing (`bond_sliding_friction`).
* Support stiffness (`kSupportStiffness`, 0.01 of the bonds') is an assumption. `core_bond` sweeps it on an overhang: the edge
  bond's load is the statics' at every value, and the next bond's couple is the same within 0.2% from 1e-4 to 1e-2 and
  10% lower at 10. The ledge scene's 2 broken bonds against 1 for welds (`bond_compound_cantilever`) is therefore
  the rigid compound against sagging welds, not the choice of stiffness.
* Cost: the network solve, the patch on the first step, the sides' mass walk, and `breakBonds` walking every joint every
  step are not measured.
* A box held by a Bond under a point `force` fell asleep and did not wake when the force was removed, so damage stopped
  building (`bond_damage_kept_unloaded` keeps it awake with `enablesleep` 0). Not looked into.
* Linux build and `clang-tidy` of the Bond files: not run.

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
  whether the ground box is drawn solid and transparent like the collider geometry under `gl_wireframe` -1.
* Box3D Data's Default Shape menu and the Packed Object / Configure menus built from it, opened in Houdini's UI
  (`default_shape_menus` checks their entries and what Plane and Cylinder write). `bullet_cylinder` and the cylinder cases of `bullet_shape_attributes` pass against Bullet.
* `bullet_georep` `plane` on a packed piece (`ShapeType::Plane`, `buildShapes`): `bullet_plane_piece` has a box rest on it at the piece's height
  (through the piece's origin, +Y, as a Ground Plane). A box dropped on it in Bullet fell through, so there is nothing to compare against. A piece turning moving
  mid-simulation rebuilds as a box.
* Ground: a Static Object whose SDF Representation is set to Implicit Plane is taken as a ground box facing +y through its
  origin, as a Ground Plane DOP's is (only the Ground Plane is tested). A ground tilted off the axes has a large broad-phase
  box, so every body in the world keeps a cheap contact with it. A box in a 90 degree trough of two grounds with the default
  friction wedges slightly high (`ground_plane`: 0.82 against 0.71), and sits flat with a friction of 0.2.

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

**Bullet semantics not compared**
* The extra `restRotation` split per packed point in every full scan (the rest convention matches Bullet: `bullet_rest_convention`, `bullet_rest_rescan`).
* Solve on Creation Frame on the SOP's collider (`solver_sop_solve_first_frame_bullet`): a free-falling box matches. A box on a collider lifted 0.5 a frame
  ends at 1.1 to 2.3 in Box3D (it clips into the collider: fork list) and at 1.0 in Bullet.
* Before-start outputs and Export Broken Constraints match (`solver_sop_before_start_bullet`, `solver_sop_keep_broken_bullet`) except the proxy
  output with no proxy input: a copy of the geometry in Box3D, empty in Bullet.
* The joint `angle` since rest is untested: `bullet_joint_rest_held` and `bullet_joint_angle` hold with a turn of 0 in both. With both anchors at one point
  (`bullet_joint_rest`, INFO) the centres end 2.19 and 2.59 apart, past the 2.06 possible.
* Hand-set shapes: `bullet_primS`, `bullet_primR` (XYZ Euler degrees), `bullet_radius` and `bullet_length` match (`bullet_shape_attributes`, `bullet_primr_order`).
  Differences kept on purpose (`bullet_shape_position`, `bullet_shape_probe`): Bullet drops a dynamic hand-set piece's x and z (to 0 plus `bullet_primT`'s),
  takes the height from `bullet_primT`'s y, and finds a static piece away from the origin at the origin; Box3D keeps the shape on the piece with `bullet_primT`
  as an offset, as copying Bullet would move every hand-set piece to the world origin. Bullet's dynamic box rests lower than Box3D's by the shape's vertical
  half extent as first turned (the rule fits every case probed). Not understood: with `bullet_primR` 30 0 60 the piece fell through the ground in Bullet
  (`bullet_primr_through`: 30 0 0, 0 0 30, 0 0 60, 30 0 30 and 0 0 45 end 0.14 to 0.47 below it, Box3D always at 0.10). A tilted box's top is a corner the
  plate test stops on roughly (0 70 50: Box3D 0.44, Bullet 0.33, true 0.437), so that case is not compared. Not compared: `deforming`, `bullet_autofit_valid`,
  AutoFit on whole objects and fractured pieces.
* Impacts: the normal direction matches (`bullet_impact_normal`). A box landing and resting for 3 s has Box3D records on 13 frames (at most 8 on one),
  Bullet's on 49 (at most 40) because Bullet keeps writing contacts at rest (`bullet_impact_fresh`). Whether a fresh `SIM_Impacts` each step suits Houdini's
  impact tools is open.
* Whole RBD Object pivot (`wholeObjectPose`, tests `hull_centroid`, `bullet_rbd_pivot`, `bullet_rbd_pivot_far`): Bullet's `p` is 0.03 off the hull centroid
  Box3D gives for boxes (an L: (0.205, 0.795) against (0.238, 0.762); the same moved 5 in x, and for a T), unexplained. A concave object with Hull per Piece
  is not checked against Bullet.
* Which end is Cone Twist's goal (taken as point 1; `bullet_cone_twist_goal`, `bullet_cone_twist_goal_apart`): Bullet's piece fell freely in both,
  so it cannot say.
* Unpacked geometry (`unpacked_geometry_bullet`, `unpacked_geometry_match`): an object of polygons only is skipped by both (ours warns). Bullet ignores loose
  polygons beside packed pieces without a message (ours warns), gives that message for loose points alone and ignores loose points
  beside a packed piece without one (ours simulates them).

**Whole objects**
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.
* RBD State's Compute Inertial Tensor off is assumed to mean the computed inertia (`rbd_object_inertia_type`).

**Box3D Solver SOP** (`tools/make_solver_sop.py`, `tests/houdini/checks_sop.py`)
* Collision Type Animated / Deforming: whether the Packed Object's Overwrite Attributes from SOP (SOP Geometry's Set Always) re-imports every step (the build prints which menus it drives),
  and the solver's cost of a full rescan of the collider each step.
* The Solver SOP's Collide As Static Object, Constraints tab, help card and Set Always settings: unchecked are the Static Object's default Collision Detection on a mesh (distance field cost), the
  help card's look in Houdini's help browser (its wiki markup parses with `bookish.wiki.wikipages.parse_to_root` in
  hython), and whether every setting tolerates being re-applied each step.
* The Solver SOP's Override Attributes from SOP (the pieces' `overwritefromsop`) is checked for its link, not for changed
  attributes reaching the solver.
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

* `glue_fracture_vs_bullet` (Box, Points from Volume, Voronoi Fracture's constraints named Glue, frame 24): Box3D's equivalent Strength is 0.3 to 1.5x Bullet's with Glue as One Body on and 0.5 to 0.7x with it off, only checked on that scene.
* `glue_compound_bridges` (a beam balanced on a block, a ball falling on its tip) passes only at Strength 2e5. At 1e5 the hold frees
  5 of 8 pieces on the first frame, `kGlueHoldHaloScale` times the ball's size around its path, and the 3 left in the compound
  are held up by one bridge weld and creep off the block; a halo of 1 fixes that but the ball bounces back in
  `glue_compound_hit`. At 3e5 the beam tilts: a bridge weld carries the hit into the compound in the step it lands, where
  welds would have broken. Welds hold the beam at every Strength tried.
* Box3D Configure has no Units (RBD Configure's Unit Mass and Unit Length, which scale Density by `unitlength / unitmass`,
  each over 1000).
* Quadric primitives (how they are sampled: [bodies.md](bodies.md)): a denser sphere mesh (Polygon Mesh, NURBS) rests at 0.46 to 0.48
  where Bullet rests at 0.499, because a Box3D hull keeps about 44 vertices and loses the poles (keeping the farthest point per
  direction rested lower: a 38-vertex ball tips onto a face). Volumes: Bullet's packed one falls through, its RBD Object stays at 2,
  Box3D's falls.
* Objects solved by another solver: packed ones are ignored as colliders; one-way collision with polygons untested.
* Height fields: holes from a mask layer, height fields on points.
* Constraint networks: agent anchors (`anchor_type` agent), the `anchor_pid` / `anchor_vid` matching. A missing `constraint_type` is `position`
  for Hard, Soft and Spring (`sidefx_hard_untyped`).
* SideFX constraint data not mapped: Soft plasticity, Cone Twist's elliptical and wide
  cones, softness, motors and translation ranges, Hard's angular motors.
* Force DOPs: parallel evaluation of circle / sphere sampled forces (the resolver has only the point call).
* Sticky collisions. Box3D's contact begin / end and sensor events: Houdini has no output for them (Bullet has none), so
  nothing reads them beyond Impacts. Sensors (trigger zones) would be the useful part.
* Box3D Solver SOP:
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
* Glue as one body (`gluecompound`, `Box3DCompoundSync.cpp`, how it works: glue-compound.md; on by default, so the glue tests run
  it; `glue_compound_break`, `glue_compound_load`, `glue_compound_vs_welds`, `glue_compound_sleep` and `core_compound` check it).
  Not covered: Impacts and the collision guide for pieces, pieces with another joint or a custom inertia (they stay
  on welds, as does their whole group), a group changed only by editing a piece's settings while it never merged
  (it is worked out again when the Glue graph or the bodies change).
* Bonds: no softening curve and no stored elastic energy (a full cohesive law needs the bond to stretch: see
  [joints.md](joints.md)), damage over time only as the kept impulse (it also builds while the pieces sleep), no rate dependence. Patches use the dominant plane
  only (an L contact gives 1 of 1.5 m^2, `core_patch`), the network's rotational stiffness is not from the patch's
  moments (a couple between two bonds of a square and a 0.1 x 10 m patch splits 50:50 where the moments give 1:99,
  `core_bond`), supports have no friction cone and read the last step's contacts, holds count the capacity but not the tear impulse, no
  guide or viewer state shows `bond_ratio` / `bond_damage`, non-hull shapes use the bounding-box estimates, the materials are order-of-magnitude values.

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
* **Events across steps, and impulse in hit events.** Box3D clears move, joint, hit and impulse events at the start of each
  `b3World_Step`, so with Substeps the solver merges them after every step (`brokenThisFrame`, `st.moves`, `hitPairs`,
  `collectImpacts` appending). Impacts come from the fork's contact impulse events (`getContactImpulseEvents`). A
  `b3ContactHitEvent` has the approach speed but no impulse, so with Impact Min Speed on a record is kept only when its
  body pair also had a hit event (`updateHitEvents`, `hadHitEvent`). Wanted: events that accumulate until cleared, and
  the normal impulse in the hit event, so the pair matching and the per-step merging go.
* **Live contact recycling flag.** `b3Body_EnableContactRecycling` only reaches contacts made afterwards (`contact.c`
  sets the flag at creation), so `setContactRecyclingByHandle` disables and enables the body (`rebuildContacts`).
  That wakes the touching bodies and zeroes the body's velocity, which the wrapper saves and restores. Wanted: the
  setter updates existing contacts. `setSurfaceByHandle` rebuilds after a rolling resistance or tangent velocity
  change too, but `b3UpdateConvexContact` re-reads both, so that rebuild may be unneeded (unchecked).
* **Generic joint.** No 6-DOF joint with a lock, limit, spring and motor per axis, so SideFX Hard, Soft, Spring and
  Cone Twist are approximated by weld, distance, motor and spherical joints (`Box3DSideFX.cpp`, [joints-sidefx.md](joints-sidefx.md)):
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
