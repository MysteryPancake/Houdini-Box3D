# Status: untested, verified, TODO

The one list of unverified items, the HDK calls already checked, and the one list of work still to do. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Untested and unverified

The one list of everything not yet tested in Houdini or confirmed against SideFX's documentation. Keep it here only;
remove an item once it has been checked, and record anything new here rather than in comments or other sections.

**General**
* No `SIM_*` code has CI; `tests/box3d_test.hiplc` is a manual scene. The core tests pass, and so do the first
  Houdini checks (`checks_core.py`, Houdini 22.0.479), which settled the items they cover; they are no longer listed
  here. `checks_solver.py`, `checks_output.py`, `checks_constraints.py` and `checks_more.py` script most of the items
  below (UI, caches, scale, threads, masks, packed types, outputs, forces, constraints, SOP Solver edits, attribute
  toggles, whole objects, comparisons with Bullet and the POP Solver) and have not been run yet. What no script can
  check: Escape mid-step, the viewport, motion blur, and how greyed-out parameters look. The plugin builds against the real HDK (Houdini 22.0.479, MSVC, no warnings), so "compiled only
  against stand-in headers" notes are gone too: what remains is behaviour.
* An untouched object still skips its scan after a Box3D Data change elsewhere (debug log: `scan skipped`). (A Box3D
  Data change mid-simulation reaching existing bodies is verified, `data_change_midsim`.)
* `tests/box3d_test.hiplc` still uses the removed Box3D Joints node and needs rebuilding with a Constraint Network.

**HDK and Houdini behaviour**
* Data ids surviving the DOP copy between timesteps (the sync skip). If they do not, every step scans: correct, no gain.
  The debug log `sync: N of M object(s) unchanged, scan skipped` shows it working.
* The timestamp semantics behind `canContinue`.
* Force DOPs: a Gravity Force DOP pulls at g and adds to the solver's Gravity (verified). Whether `getForce` is
  thread-safe (evaluation is kept serial).
* Houdini's impact normal convention (Impacts data).
* Bullet's `bullet_primR` rotation order (taken as XYZ Euler degrees) and `bullet_length` meaning (taken as the
  capsule's straight section without caps).
* The `PRM_SWITCHER` tab layout (`PRM_Default(count, label)` per tab, built by `Box3DParms`) for SIM data classes:
  documented for ordinary nodes only.
* Disable-when conditions (`PRM_Conditional`, `PRM_CONDTYPE_DISABLE`) on the joint nodes' toggles and on vector
  parameters (`target_rotation`, `motor_velocity`): not yet seen greying out in Houdini. Space-separated `{ }` groups
  are taken as OR (Houdini's documented disable-when syntax).
* `Box3DParms::Range::hardMin = false` (`PRM_RANGE_UI` minimum) for the joint nodes' signed sliders, and tabs on the
  joint nodes (switcher from `Box3DParms`): not yet seen in Houdini.
**Solver step, caching and threads** (debug build: `REBUILD` lines in the log show every world rebuild)
* An Escape mid-step (`SIM_SOLVER_FAIL`, world unstamped): Houdini discards that step and re-cooks it, the frames before
  it stay cached and correct, the re-cook rebuilds once. Also that Escape reaches `opInterrupt()` at all with no
  `opStart` of ours, during a long first-frame sync of many convex hulls.
* `objects` sorted by object id (`box3dObjectById` binary search, a miss is final). Checked once per solve (`st.objectsSorted`); an array that is not sorted falls back to a linear scan, so only speed is at stake.
* Box3D's tasks on Houdini's TBB pool (`Box3DHoudiniTasks`): no stall when several DOP networks or threaded SOPs cook
  at once, or where threading is disabled for the cooking thread (tasks run inline). `HOUDINI_MAXTHREADS=1` is fine.
* Scene scale: Box3D's tolerances assume metres. Measured (`scale`): a three-box stack rests 2.495 / 2.490 / 2.417 box
  sizes tall at 0.01x / 1x / 100x (2.5 ideal), with slight jitter at 100x: centimetre-scale scenes sink about 3%.

**Points and packed geometry**
* Point order: reversing the points upstream (Sort SOP) keeps every body's identity, with and without `id` (verified,
  `point_order`), but results differ slightly (5 cm after sliding 3 units): bodies are created in point order even when
  `id` exists, and Box3D's results depend on body order. Creating them in `id` order would make results independent of
  point order whenever `id` exists. The point number written to Impacts follows the current order.
* Packed USD / packed sequences: `getPackedDetail()` gives usable geometry (Packed Disk and Packed Alembic do).
* (Confirmed, `odd_packed_contents`: a packed primitive sphere or a nested packed primitive collides as a single point,
  half sunk into the ground. Listed under Known gaps.)
* (Confirmed, `mirrored_piece`: the write-back drops the mirror, determinant -1 becomes +1 once the piece moves. Listed
  under Known gaps.)
* `SourceSummaryCache` (process-wide, survives rebuilds): relies on `GU_Detail::getUniqueId()` never being reused in a
  session and every edit to a packed source bumping its `P` or primitive-list data id. Edit a packed source in place
  (no new id) and rebuild: a stale summary would give the old hull.
* Fingerprint skip: a packed transform edited by another node (Transform Pieces in a SOP Solver, a wrangle on the
  `transform` intrinsic) bumps the primitive list data id. (Moving `P` from a SOP Solver is picked up and the body
  teleports, verified by `sop_solver_edit`.) Packed points added or deleted bump `P` or `id`.
* `animated` bodies: a kinematic body tracks its animation at Time Scale 1 and 2 (verified, `time_scale_kinematic`);
  at Time Scale 2 it pushes a box the same distance in DOP time. Not compared with Bullet.
* Bullet's attribute semantics: `active` 0 -> 1 mid-simulation starts the body falling within a frame of Bullet, and
  `awake` 0 -> 1 wakes it (verified, `attribute_toggles`). `animated`, `deforming` and `bullet_autofit_valid` toggles
  are not compared with Bullet.
* Shared-string handles (`GA_AIFSharedStringTuple::getHandle`) are stable for one scan of one detail (`bullet_georep`
  and `name` are memoised by handle).

**Write-back and outputs**
* RBD Packed Object's plain `SIM_Geometry` is replaced once by an adopting `SIM_GeometryCopy`
  (`box3dWritableGeometry`): earlier cached frames keep their own poses (scrub back), and options that re-import the
  SOP every step (Overwrite with SOP / deforming geometry) overwrite the written `P` / `orient` / `v` / `w`. Is that wanted?
* Releasing a `SIM_GeometryAutoWriteLock` may bump data ids (the fingerprint is re-recorded after release). If our own
  writes still look like edits, every object scans every step: watch for `full scan, changed:` lines in a debug build.
* Downstream readers: DOP Import (Fetch) matches the simulation, `v` is -g t for a falling body and `w` is in radians
  per second (verified). Still open: motion blur, and that a body falling asleep keeps its last small `v` / `w`.
* `addPointImpact`: impulse (m*v plus the step's support), point number and time verified (`impacts`). Whether a fresh
  `SIM_Impacts` each step suits Houdini's impact tools, and no Feedback data for other solvers in a Multi Solver.
* Aging: `age` counts seconds, life 0 never dies, and a dead packed piece is deleted (verified, also inside a
  constraint network without errors). Compared with the POP Solver at 24 fps, a life of 0.5 dies on frame 14 here and
  frame 12 there: our age starts a frame later (a new object is not solved on its creation frame) and dies at
  age > life, POP's probably at >=.
* Point `force` / `torque`: applied for one step and zeroed (verified, `point_force`: F/m and torque/I exact). A POP
  Force must cook before the Box3D Solver in the same step; the zeroing may clash with another solver reading `force`
  in a Multi Solver.
* Force DOPs are evaluated once per DOP step, at the start-of-step state (not per Box3D substep). A Gravity Force DOP
  adds to the solver's own Gravity (documented in the Gravity tooltip).

**Affectors (Static Object and similar)**
* A moved Static Object is moved, not swept: bodies are not pushed (the README says so). (Following an animated
  collider works, verified by `animated_static`, by OBJ Path or a Transform SOP.)
* Static Objects whose geometry is packed, SDF-only or not polygons: only polygons are used, so they collide with
  nothing and warn "no usable collision geometry". (The Ground Plane DOP works: a dropped box lands on it.)
* A Static Object with Use Deforming Geometry bumps `P` every step, so its mesh is rebuilt every step (cost).
* Objects solved by another solver (Bullet, RBD, POP) with a collide relationship: packed ones are ignored by Box3D
  ("no usable collision geometry", seen with a Bullet RBD Packed Object, which then fell through the Box3D body). One-way
  collision with polygon objects is untested.

**Constraint relationship nodes**
* Their icons are picked up.

**Constraint networks**
* Relationships found with `getRelationship(ingroup, i)` for both `ingroup` true and false; deduplicated by name.
* Float attributes multiply the node's value (Houdini's rule): confirm it is wanted, since zero defaults (limits,
  targets) and -1 caps cannot be changed per primitive unless the node is set to 1.

**SideFX constraint data**
* `SIM_Options::importOption` converting SideFX's int / bool options (Glue with the Box3D Solver holds, so reading
  SideFX's data works; the individual values are not checked).
* (Confirmed, `sidefx_glue`: Box3D's Glue breaks under a hanging piece's weight at Strength 1 and 100 where Bullet's
  holds; both hold at 1e4. Listed under Known gaps.)
* Which end is Cone Twist's goal (taken as point 1); only matters when goal and constrained axes differ.
* Cone Twist compared with Bullet (`sidefx_cone_twist`): Bullet's piece falls freely (20.85 apart after 2 s, the
  same with anchors at the centroids or at one shared point), so there is nothing to compare with. Box3D holds: with
  one shared anchor the pieces stay 2.015 apart (2.06 at the start); with anchors at the two centroids it pulls them
  together (0.27), as a ball joint joining two anchor points should.
* Soft / Spring `all` with a rest length shifts the frames along the anchor line when the joint is made; after a
  world rebuild from a stretched pose the offset direction is re-derived from the anchors and may differ slightly.

**Whole-object bodies**
* RBD Point Object with the Box3D Solver: check by hand. A scripted setup (Point SOP Path on a 2 x 2 grid of points,
  Geometry Path on a box) makes no objects at all and gives no errors or warnings (`other_rbd_objects`, which stays
  INFO for it; RBD Fractured Object passes). It is an older node; RBD Packed Object is the usual choice.
* The `Colliders` label on RBD Point Object and RBD Fractured Object (only RBD Object and RBD Packed Object were
  checked); its value is not used, only "present and not PackedObject".
* `SIM_DATA_CREATE(obj, "Position", SIM_Motion, 0)` when the object has no Position data, and whether writing
  `Position` from a solver needs anything more (Bullet and the RBD solver do it).
* The viewport and DOP Import SOP follow the written Position data (through the geometry's `positionpath`).
* Without Box3D Data an RBD Object now uses its own RBD State / PhysicalParms / BulletData (names from a Houdini 22.0
  probe). Its mass, bounce and shared world are checked; still open: that it collides as its convex hull (a tilted
  rod falls over, so it is not an axis-aligned fitted box, but was still settling 0.32 tall after 240 frames, where
  flat is 0.2), whether `bullet_primT` is relative to the pivot (taken so, as for points) and
  `bullet_primS` a half size for whole objects too. Scenes with RBD Objects and no Box3D Data simulate differently
  from before (mass 1, friction 0.6, bounce 0, box); attaching Box3D Data gives the old behaviour.
* Float noise: the mirror takes a read-back after each write; a constant "edited" count (teleports) every step would
  mean the read-back does not match what the next sync reads.
* Impacts record point -1 for whole objects.

## HDK calls

Checked against the HDK reference: `GU_PrimPacked::isPackedPrimitive`, `getPackedDetail`, `sharedImplementation`,
`getFullTransform4`, `setLocalTransform`, `getPivot`, `transformDirty`; `GU_PackedFragment::getPrimitiveRange`, `detailPtr`; `UTgetInterrupt`, `UT_Interrupt::opInterrupt`;
`GU_DetailHandleAutoReadLock` / `SIM_GeometryAutoReadLock::getGdp` (const pointer), `SIM_GeometryAutoWriteLock::getGdp`
(reference); `GA_Detail::getPrimitiveList()` / `GA_PrimitiveList::getDataId()` / `bumpDataId()`,
`findPointAttribute` (const), `getP`, `getNumPointOffsets`, `getNumPrimitiveOffsets`, `pointVertex`,
`vertexPrimitive`, `getPointMap`, `pointIndex`, `pointOffset`, `destroyPointOffset` (`GA_DESTROY_DEGENERATE`),
`destroyPrimitive`, `bumpAllDataIds`; `GA_Attribute::getDataId` / `bumpDataId`; `GA_AIFSharedStringTuple::getHandle`
/ `getTableString`; `GEO_Detail::addIntTuple`; `SIM_DATA_CREATE` flags (`SIM_DATA_RETURN_EXISTING` unset = replace);
`SIM_Impacts::addPointImpact`, `SIM_IMPACTS_DATANAME`; `SIM_RootData::getName`, `removeNamedSubData`;
`UT_String::isstring`, `buffer`; `SIM_Object::getNumRelationships`, `getRelationship(ingroup, index / name)` (const),
`getRelationships(ingroup)` (non-const, `SIM_RelationshipArray::findRelationshipByName`);
`SIM_Relationship::getRelationshipTypeData` / `getConstRelationshipTypeData`; `SIM_ConstraintNetworkRelationship`
(a `SIM_RelationshipData`) `getGeometry` const / non-const; `SIM_ConRel` (no pure virtuals, protected factory
constructor); `DECLARE_CLASSNAME` / `DECLARE_DATAFACTORY` expansions (`SIM_DataUtils.h`); `UT_Matrix3D::extractScales`, `UT_QuaternionT::getRotationMatrix` /
`updateFromRotationMatrix`; node icons as `config/Icons/DOP_<name>.svg`; `PRM_Conditional(const char *, PRM_ConditionalType)`
passed as a `PRM_Template`'s conditional pointer.

## Known gaps / TODO

* RBD Packed Object writes its own physical attributes onto every point and point attributes win, so Box3D Data's
  mass / density / compute mass / friction / bounce do nothing there (as with Bullet). Decided: keep Bullet's rule,
  document it (README, tooltips) and warn once per object. A Box3D Packed Object node (an asset: Empty Object, SOP
  Geometry, Box3D Data, no stamped attributes) would give Box3D Data its full role on packed geometry.
* Mirrored packed pieces lose their mirror once they move (the write-back writes a non-mirrored transform).
* Packed primitive spheres and nested packed primitives collide as one point (hulls from points only): unpack or warn.
* SideFX Glue breaks under static load (Break Force = strength / timestep) where Bullet's Glue holds.

* GCC/Clang `-Wshadow` (shadowing members and outer scopes, beyond the local shadowing MSVC `/W4` reports) and
  `-Wconversion` (about 12) are off.
* The Houdini-side files (`src/dop`, `src/sync`) have not been through clang-tidy (needs an HDK build's
  compile_commands.json); their casts were converted to `static_cast` by script, checked to change nothing else.

* Scenes saved before the joint nodes' Breakable / Set Rest Length / Limit Spring Force / Limit Max Length / Limit
  Torque toggles load with them off, so a Break Force, Rest Length etc. typed in such a scene is ignored until the
  toggle is turned on. Bounce Mixing's menu was reordered: a non-default choice saved before then reads as a
  different rule.

The one list of what is still to do. Keep it here only (no TODO comments in the code).

* Static triangle-mesh colliders for the concave *point* case (not started; affector meshes exist, see "Affector static
  colliders" in [forces-affectors.md](forces-affectors.md)). Box3D meshes (`b3CreateMeshShape`) and height fields only collide as static, which is why `concave` /
  `compound` are one hull per piece (Bullet's concave is a triangle mesh and may move). Intended split: a static
  (inactive, non-animated) `concave` point becomes a Box3D mesh of its packed geometry; a dynamic one keeps the
  compound. Mind the mesh's own memory (not the hull cache), `deforming` rebuilds, and that a mesh body cannot become
  dynamic without rebuilding its shape.
* Height-field colliders; `cylinder` / `plane` shapes (no Box3D shape; hull or big box approximations possible).
* Constraint networks: point / vertex / agent anchors (`anchor_id`, `anchor_type`); `constraint_type` on our own nodes
  (e.g. on the weld node: `position` -> spherical, `rotation` -> rotation only, `all` -> weld; SideFX data reads it).
* SideFX constraint data, the parts not mapped: Glue half-life / propagation (impulse accumulation); Soft
  plasticity (would move the joint frames over time); Cone Twist's elliptical cone (only the smaller limit), cones
  between 90 and 180 degrees (clamped to 90), softness / soft limits, motor and translation ranges; Hard's angular
  motors; Spring's Hz conversion following mass changes.
* Box3D Weld nodes between anchors at different points (Connect Adjacent Pieces puts each end at its piece's centroid)
  pull the pieces together; `holdSeparation` (see "SideFX constraint data" in [joints.md](joints.md)) could be offered as a node option.
* Whole-object bodies without Box3D Data: RBD Object's Create Active Object (`SolverParms/ActiveValue`) for the body
  type, and RBD State's custom inertia tensor.
* Force DOPs: only `getForce` at the centre of mass (no `getForceOnSphere` / area variants, no per-substep re-evaluation).
  Evaluation is serial. Computing the forces in parallel (state read serially or from `BodyState`, `getForce` in
  `UTparallelFor`, the sum applied serially) is possible only if the Force DOPs are thread-safe, which is not known for
  custom ones; it would need an opt-in or a check of the force classes.
* Sticky collisions; contact / sensor events beyond Impacts.
