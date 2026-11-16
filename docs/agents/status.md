# Status: unverified, TODO

The one list of what is unverified or still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not this file).

Last full run (Houdini 22.0.502, MSVC, the Box3D fork): every check passes, `baseline` included. The INFO results
are not pass marks.
What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

## Unverified

**Not yet run in Houdini**
* Box3D Packed Object's Display Geometry (`visualize_hires`) and Collider Label (`add_display`): checked against a
  stand-in `hou` only. The build's `check` probes both.
* Box3D Configure's extra settings and Bounds / Use Bounds, and the Solver SOP's Output tab (Point Velocities,
  Import Constraints / Proxy, Bullet's `w` rule) and World Scale on joints and FORCES: built against stand-ins and
  lint only. The builders' `check`s probe Use Bounds and the joint scaling.
* An object with unpacked primitives and nothing packed returns before making bodies (`syncObject`): its bodies are
  removed and its names released (`NameIndex::finish`). Test `unpacked_geometry`.

**Build**
* Linux plugin build (only the core builds in CI): `build.sh`, gcc warnings in `src/dop` / `src/sync`, the dso and
  icon copies, `hython tests/houdini/run_tests.py`.
* The Houdini-side files have not been through `-Wshadow` / `-Wconversion` (GCC / Clang) or clang-tidy.
* The Threads default (`performanceCoreCount`) on macOS (`hw.perflevel0`) and ARM Linux (`cpu_capacity`).

**Houdini behaviour**
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
* The ground guide (`build_ground_guide`, test `solver_sop_ground_guide`): the builder runs in memory without errors and
  the nodes are wired, but it is not built or run in Houdini: what the display geometry of a Ground Plane and the Static
  Object's `Geometry` import as, and whether `gl_wireframe` -1 on the merged guide still shows a Shaded plane shaded.
* Box3D Data under Geometry on Static Objects and RBD Objects.
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

**Bullet semantics not compared**
* The extra `restRotation` split per packed point in every full scan (the rest convention itself matches Bullet:
  `bullet_rest_convention`).
* Solve on Creation Frame on the SOP (`solver_sop_solve_first_frame`) for the collider, against Bullet's.
* Before-start outputs and Export Broken Constraints (`solver_sop_before_start`, `solver_sop_keep_broken`). Both need
  a run of the RBD Bullet Solver SOP beside ours, which no test builds yet.
* Joints from authored anchors through `restToNow` and the `angle` since rest, against Bullet (test `bullet_joint_rest`,
  rerun with constraint_type all: with `position` both held the 1.0 rest length, which pushes the boxes into each
  other; with both anchors at one point the centres ended 2.19 and 2.59 apart, past the 2.06 possible).
* `bullet_primR` rotation order: XYZ Euler degrees matches Bullet. Settled thin boxes have the same face down
  (`bullet_primr_order`), and boxes turned by 90 degrees have the same face up (`bullet_shape_attributes`: 90 90 0,
  0 90 90, 90 0 90 pick XYZ out of the six orders). A tilted box's top is a corner and the plate test stops on it
  roughly: Box3D reads 0.44 and Bullet 0.33 for 0 70 50 (true top 0.437), so that case is not compared. Bullet's
  dynamic hand-set box rests at y = Box3D's (the half thickness, 0.10) minus the shape's vertical half extent as
  first turned (0.10 unturned, 0.26 for 50 70 50, 0.44 for 0 70 50, 0.52 for 0 0 50, 0.29 for 50 0 0): the rule fits
  every case probed, so Bullet's reported point sits that much lower, not the shape. Not understood: with primR
  30 0 60 the piece fell through the ground in Bullet. `bullet_primS`, `bullet_primT`, `bullet_radius` and
  `bullet_length` match Bullet. Bullet ignores `bullet_primT` for a dynamic piece's rest height
  (0.25, shape centred on the body) where Box3D keeps the offset (-0.05); a static piece away from the origin is
  not found by a plate in Bullet. Not compared: the `deforming` and `bullet_autofit_valid`
  toggles, AutoFit on whole objects and fractured pieces.
* Where Bullet puts a hand-set shape on a static piece away from the origin: a plate over a piece at x 3 never
  stopped in Bullet, where Box3D has the shape on the piece (`bullet_shape_position`; its dynamic case is rerun, the
  first run had made the ground a hand-set box too).
* Impact normal direction (test `bullet_impact_normal`, rerun: both solvers write a `normal` field, the first run
  could not read it as a Houdini vector); whether a fresh `SIM_Impacts` each step suits Houdini's impact tools.
* Bullet's placement of a hand-set shape (`bullet_shape_probe`, the same with a neutral ground): a dynamic hand-set piece
  dropped from x 2 ends at x 0, z 0 (plus `bullet_primT`'s x and z), so Bullet drops the piece's position, and its
  height follows `bullet_primT`'s y (0.3 up gives 0.25 to 0.3, 0.3 down gives -0.3) rather than the shape. Box3D keeps
  the piece where it is with primT as an offset. Not copied: it would move every hand-set piece to the world origin.
* A whole RBD Object box in Box3D is still moving at frame 120 and sits at y 0.54 (Bullet: 0.500, settled), for every box
  type; packed pieces settle at 0.500 in both. Not yet looked into (test `bullet_primitive_types`).
* Which end is Cone Twist's goal (taken as point 1; test `bullet_cone_twist_goal`): Bullet's piece fell freely with both
  anchors at one point, turned or not, so it cannot say. Needs a Bullet Cone Twist setup that holds.
* Unpacked geometry (`unpacked_geometry_bullet`): an object of polygons only is skipped by both (ours warns). This
  file had Bullet's message as an error, the author reports a warning: rerun to settle. Bullet ignores loose polygons
  beside packed pieces without a message (ours warns) and gives that message for loose points (ours simulates them).

**Whole objects**
* RBD Point Object (`other_rbd_objects`, `point_object_bullet`): needs its Geometry Path menu (`usegeo`) set to Override, else it makes no objects. With that, Bullet and Box3D match: 4 whole objects, both fall to y -2.424 (checked).
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.
* RBD Object's Create Active Object (`SolverParms/ActiveValue`, test `rbd_object_active`), written without Houdini: the option names.
* RBD State's Compute Inertial Tensor off is assumed to mean the computed inertia (`rbd_object_inertia_type`).

**Box3D Solver SOP** (`tools/make_solver_sop.py`, `tests/houdini/checks_sop.py`)
* Collision Type Animated / Deforming: whether the Packed Object's Overwrite Attributes from SOP (SOP Geometry's Set Always) re-imports every step (the build prints which menus it drives),
  and the solver's cost of a full rescan of the collider each step.
* The Solver SOP's Collide As Static Object, Initial State, Constraints tab, help card and Set Always settings pass
  their tests. Still unchecked: the Static Object's default Collision Detection on a mesh (distance field cost), the
  help card's wiki markup rendering, and whether every setting tolerates being re-applied each step.
* `anchor_id` passes on whole objects (`anchor_id`) and on packed pieces against Bullet (`bullet_anchor_id`, ids 0 and 5,
  points and vertices; the test's pieces are fragments). Untested: other ids (a fragment's points are taken in source order,
  its vertices in primitive order), a piece that has moved before its joint is made (`nowToRest`), and non-fragment packed
  pieces.
* Glue's impulse model (`core/Box3DGlue.h`, `core_glue`, `glue_impact`, `glue_impact_trace`, `sidefx_glue`): `impact` matches
  Bullet's (same decay, peak within ~20%); a Bullet bond breaks only at about 5 to 6 times that, which is unexplained, so
  `kGlueStrengthScale` is measured (`glue_impact_scale`, one scene, two drop heights: the thresholds then agree within 14%). Unchecked: other scenes (resting
  piles, several contacts, different masses), the half-life and propagation against Bullet, the global propagation
  iteration count (the `propagate_iteration` detail attribute is not read), and the cost of reading every glued body's
  contacts each step on a large network.
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
  is a real Box3D sphere (`NamedPiece::sphereValid`). The tube rested at 1.0 (unit tube taken as y -1 to 1), now taken
  as -0.5 to 0.5: rerun. A circle packed settles flat at 0.02; as an RBD Object it stood on its edge. Not done:
  quadrics inside a Convex hull Per Connected Set, volumes (Bullet: packed falls through, RBD Object stays at 2),
  tube taper.
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
  cones, softness, motors and translation ranges, Hard's angular motors, Spring's Hz following mass changes.
* Inactive RBD Object (Create Active Object off) is a static body: animating its Position teleports it, where Bullet
  drives it as a kinematic body that pushes what it touches.
* Force DOPs: evaluation inside Box3D's Solver Passes (needs a callback in the fork); parallel evaluation of circle / sphere
  sampled forces (the resolver has only the point call).
* Sticky collisions; contact / sensor events beyond Impacts.
* Box3D Solver SOP:
  * Collision: Deforming following an unpacked collider that scales (a turn is checked by
    `unpacked_turning_collider`).
  * Set Always for the Box3D Data settings of the pieces and collider, the relationships and the ground (the solver's
    own settings are Set Always already).

## Box3D fork (`extern/box3d`)

Features to add in the fork. Once one exists, the wrapper maps it and the entry leaves this list.

* **Spherical spring twist bug.** The angular spring loses its restoring torque when the body is twisted about its
  longitudinal axis. A bollard tilted under gravity, then twisted 180°, cannot be pulled upright and settles tilted at
  about 75-78° whatever it started at. Cause: `b3SolveSphericalJoint` (`spherical_joint.c`) takes one rotation vector
  from `b3DeltaQuatToRotation(relQ, targetRotation)`, whose tilt part shrinks as cos(twist / 2). Fix: separate swing
  and twist errors, each with its own spring. The motor joint's angular spring stalls the same way. The weld and
  prismatic ones make the same call and are unchecked. A twist limit avoids it (the car park's bollards). A parallel
  joint with a twist limit also orbited at about 72°: not understood.
* **Wide and elliptical cones.** `b3SphericalJoint_SetConeLimit` asserts the angle is at most 90°, and the cone is round.
  Cone Twist's wider and elliptical cones (unmapped above) need both lifted.

Workarounds in the wrapper that the engine could own, roughly by value:

* **Public world snapshot.** The fork serializes a world internally (`b3SerializeWorld`, `b3DeserializeIntoShell` in
  `world_snapshot.c`, for the recording player's keyframes) but exposes only recording and replay. So the world is a
  rebuildable cache ([solver.md](solver.md)): a scrub, re-solve, cache reload or thread change rebuilds it from point
  attributes, which loses warm starting, contact impulses and sleep state, and a saved sim cannot reload one. Expose
  save and restore into a new world. The DOP cache could then hold snapshots, and simulating on from an earlier frame
  would not need extra live worlds ("world limit reached", README).
* **Worlds allocated on demand.** `B3_MAX_WORLDS` sizes the static `b3_worlds` array at compile time, so the build
  passes `BOX3DDOP_MAX_WORLDS` through to Box3D and the solver reports "world limit reached" past it.
* **A hull build that always succeeds.** `b3CreateHull` fails when the vertex, face or edge limits are exceeded, so
  `createHullWithKey` retries smaller vertex budgets (a triangulated hull hits the half-edge limit well below the
  vertex limit), `fitShapeOriented` repeats the ladder, and a per-group hint remembers the budget that worked. Wanted:
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
