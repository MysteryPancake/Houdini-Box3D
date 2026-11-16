# Status: unverified, TODO

The one list of what is unverified or still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not this file).

Last full run (Houdini 22.0.502, MSVC): every check passes except `baseline`, which was recorded on 22.0.479
(`run_tests.py baseline --update-baseline` records it again). What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

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
* Escape mid-step: the step is discarded and re-cooked, earlier frames stay cached, the re-cook rebuilds once.
* Data ids surviving the DOP copy between timesteps (the sync skip; debug log `scan skipped`); our own writes not
  looking like edits (debug log `full scan, changed:`).
* `SourceSummaryCache`: a packed source edited in place without a new unique id or data id gives a stale hull.
* Box3D's tasks on Houdini's TBB pool with several DOP networks or threaded SOPs cooking at once.
* Whether `getForce` is thread-safe (Force DOPs are evaluated serially).
* Box3D Data under Geometry on Static Objects and RBD Objects.

**Box3D Configure** (`tools/make_configure_sop.py`, tests `configure`, `configure_packs`)
* The packing (`b3d_prepare.py`): Transfer Groups carrying primitive groups onto the pieces, the Assembles' path /
  fragment parameters from the asset, geometry-only and proxy-only inputs.
* The group selection button, labels hidden on the checkboxes.

**Bullet semantics not compared**
* The rest convention (`restxform`, `pivot`, `orient` since rest) against an RBD Bullet Solver run of the same pieces;
  the extra `restRotation` split per packed point in every full scan.
* Solve on Creation Frame on the SOP (`solver_sop_solve_first_frame`) for the collider, against Bullet's.
* Before-start outputs and Export Broken Constraints (`solver_sop_before_start`, `solver_sop_keep_broken`).
* Joints from authored anchors through `restToNow` and the `angle` since rest, against Bullet.
* `animated`, `deforming` and `bullet_autofit_valid` toggles; `bullet_primR` rotation order (taken as XYZ Euler
  degrees) and `bullet_length` (taken as the capsule's straight part); AutoFit on whole objects and fractured pieces.
* Impact normal direction; whether a fresh `SIM_Impacts` each step suits Houdini's impact tools.
* Which end is Cone Twist's goal (taken as point 1).
* Unpacked geometry (`unpacked_geometry_bullet`): an object of polygons only is skipped by both (ours warns). This
  file had Bullet's message as an error, the author reports a warning: rerun to settle. Bullet ignores loose polygons
  beside packed pieces without a message (ours warns) and gives that message for loose points (ours simulates them).

**Whole objects**
* RBD Point Object (`other_rbd_objects`, `point_object_bullet`): needs its Geometry Path menu (`usegeo`) set to Override, else it makes no objects. With that, Bullet and Box3D match: 4 whole objects, both fall to y -2.424 (checked).
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.
* RBD Object's Create Active Object (`SolverParms/ActiveValue`, test `rbd_object_active`), written without Houdini: the option names.
* RBD State's Use Bounding Box inertia (test `rbd_object_inertia_type`, `Box3DWholeObject.cpp`): the menu index (1) and
  the option names are from the node's parameters, not a probe of the data; Compute Inertial Tensor off is assumed to
  mean the computed inertia. Extents come from `collectPieces` each step.

**Box3D Solver SOP** (`tools/make_solver_sop.py`, `tests/houdini/checks_sop.py`)
* Collision Type Animated / Deforming: whether the Packed Object's Overwrite Attributes from SOP (SOP Geometry's Set Always) re-imports every step (the build prints which menus it drives),
  and the solver's cost of a full rescan of the collider each step.
* Transform Pieces with packed transforms that carry scale or shear.
* Constraints output through Transform Pieces: constraints between collision pieces (not in the simulation points, so
  not moved).
* FORCES: the Multiple Solver with nothing wired into PRESOLVE / POSTSOLVE (no warning expected); that diving into
  the asset lands in FORCES (the build's check passes); FORCES editable in a locked instance (`EditableNodes`).

## TODO

* Packed primitive spheres and nested packed primitives collide as one point, half sunk (`odd_packed_contents`).
* Mirrored packed pieces lose their mirror once they move (`mirrored_piece`). The fork's `b3CloneAndTransformHull`
  takes a mirroring scale: try it.
* `Box3DWorld::boxShape` makes a rotated box as the hull of its 8 corners through the hull cache. The fork has
  `b3MakeTransformedBoxHull`.
* `Box3DWorld::estimatedBytes` prices objects at constants measured with `b3GetByteCount` (`kBodyBytes` ...).
  `b3World_GetCounters(...).byteCount` is a per-world figure that includes the hull database: try it.
* SideFX Glue breaks under static load (Break Force = strength / timestep) where Bullet's holds (`sidefx_glue`).
* Results depend slightly on point order even with `id` (`point_order`): bodies are created in point order.
* Objects solved by another solver: packed ones are ignored as colliders; one-way collision with polygons untested.
* Export Collision Shapes (unverified): `SIM_DATA_CREATE(obj, "Box3DShapes", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING)`
  on an object that has no such data, then an empty detail in `SIM_GeometryAutoWriteLock`; `setExtraInfo` keeping other
  entries and surviving the later `save`s (Bullet's extra info is `guide=GUIDE_GEO` among colour entries).
* Concave meshes: one mesh shared between instances, static RBD Objects as meshes, drawing meshes in the guide.
* Height fields: holes from a mask layer, height fields on points. `cylinder` and `plane` shapes: see the Box3D fork list.
* Constraint networks: point / vertex / agent anchors (`anchor_id`, `anchor_type`); `constraint_type` on our nodes.
* SideFX constraint data not mapped: Glue half-life / propagation, Soft plasticity, Cone Twist's elliptical and wide
  cones, softness, motors and translation ranges, Hard's angular motors, Spring's Hz following mass changes.
* Inactive RBD Object (Create Active Object off) is a static body: animating its Position teleports it, where Bullet
  drives it as a kinematic body that pushes what it touches.
* Force DOPs: sphere / area variants, per-substep evaluation, parallel evaluation (needs thread-safe forces).
* Sticky collisions; contact / sensor events beyond Impacts.
* Box3D Solver SOP:
  * Collision: Deforming following an unpacked collider that scales (a turn is checked by
    `unpacked_turning_collider`); a Static Object option (height fields, one mesh), its own shape, friction and
    bounce on the node.
  * Constraint relationship parameters on the node (Glue strength, Weld break force ...), or Attributes Replace
    Values on the inner ones so primitive attributes are absolute.
  * Initial velocity controls, Set Always for animated solver parameters, a help card.

## Box3D fork (`extern/box3d`)

Features to add in the fork. Once one exists, the wrapper maps it and the entry leaves this list.

* **Spherical spring twist bug.** The angular spring loses its restoring torque when the body is twisted about its
  longitudinal axis. A bollard tilted under gravity, then twisted 180°, cannot be pulled upright and settles tilted at
  about 75-78° whatever it started at. Cause: `b3SolveSphericalJoint` (`spherical_joint.c`) takes one rotation vector
  from `b3DeltaQuatToRotation(relQ, targetRotation)`, whose tilt part shrinks as cos(twist / 2). Fix: separate swing
  and twist errors, each with its own spring. The motor joint's angular spring stalls the same way. The weld and
  prismatic ones make the same call and are unchecked. A twist limit avoids it (the car park's bollards). A parallel
  joint with a twist limit also orbited at about 72°: not understood.
* **Ground planes.** An infinite static plane shape, several per world, each with its own friction and bounce.
  `b3ShapeType` has none, so `plane` falls back to Box3D Data's Default Shape. It has no finite AABB, so it needs its
  own path past the broad phase, and contact against sphere, capsule and hull (a mesh if cheap). Also a native
  `cylinder` shape, which has the same fallback.
* **Glue: break on accumulated impulse.** Joints break on a per-step force or torque (`forceThreshold`,
  `torqueThreshold`), and joint events name the joints over it without saying by how much. Bullet's Glue strength is
  an accumulated impulse, so Glue maps to Break Force = strength / timestep and also breaks under steady load
  (`sidefx_glue`, [joints.md](joints.md)). Needs a break threshold on impulse that ignores steady load, and the
  joint's load readable after the step.
* **Glue: impact propagation.** A broken or overloaded glue joint passes part of its impulse on to the joints around it,
  fading by a half-life (SideFX Glue's propagation and half-life, unmapped). Box3D has no joint-to-joint spread. Needs
  the load above, then a pass over the joint graph per step, in Box3D or in the wrapper's `syncJoints`.
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
  body's contacts after the last step (`getContactsByHandle`) and match pairs against the hit events: with Impact Min
  Speed off, a hit that has separated by the end of the frame is lost. Wanted: events that accumulate until cleared,
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
