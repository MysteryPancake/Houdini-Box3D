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
* Mirrored packed pieces lose their mirror once they move (`mirrored_piece`).
* Spherical spring (and the motor joint's angular one) stalls when twisted about 180°: Box3D's error
  (`b3DeltaQuatToRotation`) is the total rotation's axis, whose tilt part shrinks as cos(twist / 2). Fix in Box3D
  (separate tilt and twist errors, upstream) or re-aim `targetRotation` at the current twist each step. A twist limit
  avoids it (the car park's bollards). A parallel joint with a twist limit also orbited at about 72°: not understood.
* SideFX Glue breaks under static load (Break Force = strength / timestep) where Bullet's holds (`sidefx_glue`).
* Results depend slightly on point order even with `id` (`point_order`): bodies are created in point order.
* Objects solved by another solver: packed ones are ignored as colliders; one-way collision with polygons untested.
* Export Collision Shapes (unverified): `SIM_DATA_CREATE(obj, "Box3DShapes", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING)`
  on an object that has no such data, then an empty detail in `SIM_GeometryAutoWriteLock`; `setExtraInfo` keeping other
  entries and surviving the later `save`s (Bullet's extra info is `guide=GUIDE_GEO` among colour entries).
* Concave meshes: one mesh shared between instances, static RBD Objects as meshes, drawing meshes in the guide.
* Height fields: holes from a mask layer, height fields on points. `cylinder` / `plane` shapes (hull or box stand-ins).
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
