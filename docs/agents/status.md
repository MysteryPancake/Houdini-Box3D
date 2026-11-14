# Status: unverified, known gaps, TODO

The one list of what is unverified, broken or still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md). Keep each
item to a line or two; remove it once settled (the test that settles it is the record, not this file).

Last full run (Houdini 22.0.502, MSVC): every check passes, 144 PASS and 20 INFO; the baseline (recorded on 22.0.479)
matches exactly. What no script checks: Escape mid-step, the viewport, motion blur, how greyed-out parameters look.

## Unverified

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
* Box3D Data under Geometry on Static Objects and RBD Objects; old scenes with the top-level `Box3DData` name.
* Scenes saved before a default changed (Default Shape, solver substeps / contact tuning): whether Houdini kept the old
  value in the file.


* `Box3DParms::subfolder()` (the solver's Misc tab): a nested simple folder counted as one parameter of its tab, as
  Houdini's switcher counts expect (the `ui` test checks the result).

**Bullet semantics not compared**
* `animated`, `deforming` and `bullet_autofit_valid` toggles; `bullet_primR` rotation order (taken as XYZ Euler
  degrees) and `bullet_length` (taken as the capsule's straight part); AutoFit on whole objects and fractured pieces.
* Impact normal direction; whether a fresh `SIM_Impacts` each step suits Houdini's impact tools.
* Which end is Cone Twist's goal (taken as point 1).

**Whole objects**
* RBD Point Object makes no objects in a scripted setup (`other_rbd_objects`); check by hand.
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.

**Box3D Solver SOP** (`tools/make_solver_sop.py` builds cleanly on 22.0; `tests/houdini/checks_sop.py` not run yet)
* Simulation results: everything below, and Unpack's Transfer Attributes name in the proxy test.
* Animated Collision: whether SOP Geometry's Set Always re-imports every step (the build prints which menus it drives),
  and the solver's cost of a full rescan of the collider each step.
* `dopcontextgeo` through the Packed Object's channel reference.
* Transform Pieces with packed transforms that carry scale or shear.

## Known gaps

* Packed primitive spheres and nested packed primitives collide as one point, half sunk (`odd_packed_contents`).
* Mirrored packed pieces lose their mirror once they move (`mirrored_piece`).
* SideFX Glue breaks under static load (Break Force = strength / timestep) where Bullet's holds (`sidefx_glue`).
* Results depend slightly on point order even with `id` (`point_order`): bodies are created in point order.
* Objects solved by another solver: packed ones are ignored as colliders; one-way collision with polygons untested.
* Float attributes multiply the node's value (Houdini's rule), so a zero default can't be changed per primitive.
* Old scenes: joint toggles added later (Breakable, Set Rest Length, Limit ...) load off, so a typed Break Force etc.
  is ignored until turned on; Bounce Mixing's menu was reordered; the mixing defaults are now Multiply / Minimum.

## TODO

* Box3D Packed Object: Overwrite with SOP / deforming import, initial velocity controls, a `Colliders` label (other
  solvers use it to collide with it), a Box3D Tab submenu.
* Concave meshes: one mesh shared between instances, static RBD Objects as meshes, drawing meshes in the guide.
* Height fields: holes from a mask layer, height fields on points. `cylinder` / `plane` shapes (hull or box stand-ins).
* Constraint networks: point / vertex / agent anchors (`anchor_id`, `anchor_type`); `constraint_type` on our nodes.
* SideFX constraint data not mapped: Glue half-life / propagation, Soft plasticity, Cone Twist's elliptical and wide
  cones, softness, motors and translation ranges, Hard's angular motors, Spring's Hz following mass changes.
* A Hold Separation option on the Weld node (`holdSeparation`, see joints.md).
* RBD Object without Box3D Data: Create Active Object (`SolverParms/ActiveValue`), RBD State's custom inertia.
* Force DOPs: sphere / area variants, per-substep evaluation, parallel evaluation (needs thread-safe forces).
* Sticky collisions; contact / sensor events beyond Impacts.
* Box3D Solver SOP:
  * Unpacked input: pack Geometry, Proxy and Collision by `name` (RBD Bullet Solver does); now each point of unpacked
    geometry is a body, and collision geometry must be packed.
  * Collision: deforming colliders (`deforming`), a Static Object option (height fields, one mesh), its own shape,
    friction and bounce on the node.
  * Constraint relationship parameters on the node (Glue strength, Weld break force ...), or setting the inner ones to
    1 so primitive attributes are absolute.
  * Output tab: Import Constraints / Proxy toggles, attributes to keep, `v` / `w` on Geometry; Transform Pieces
    options; hires geometry for pieces the solver deleted (dead).
  * Guide Options (Show Collision Shapes), a Forces / POP subnet to dive into, initial velocity controls,
    Set Always for animated solver parameters, a Tab submenu, a help card, Solve on Creation Frame.

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
