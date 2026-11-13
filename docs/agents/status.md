# Status: untested, verified, TODO

The one list of unverified items, the HDK calls already checked, and the one list of work still to do. Index and rules: [AGENTS.md](../../AGENTS.md).

## Untested and unverified

The one list of everything not yet tested in Houdini or confirmed against SideFX's documentation. Keep it here only;
remove an item once it has been checked, and record anything new here rather than in comments or other sections.

**General**
* No `SIM_*` code has automated tests or CI; `tests/box3d_test.hiplc` is a manual scene. The core tests pass.
* The code is written without a Houdini install. Since the constraint networks, whole-object bodies and the split of
  the solver into `sync/` files, every `SIM_*` and `sync/` file has only been compiled against minimal stand-in HDK
  headers (plain C++ errors and cross-file symbols checked, not HDK API mistakes). `Box3DHoudiniTasks.h` was not
  compiled against the HDK when written.
* `tests/box3d_test.hiplc` still uses the removed Box3D Joints node and needs rebuilding with a Constraint Network.

**HDK and Houdini behaviour**
* Data ids surviving the DOP copy between timesteps (the sync skip). If they do not, every step scans: correct, no gain.
  The debug log `sync: N of M object(s) unchanged, scan skipped` shows it working.
* The timestamp semantics behind `canContinue`.
* Taken from the HDK samples in `DOP.zip`, not compiled against a real HDK: `SIM_Force::getForce` (public, const;
  arguments as in `SIM_ForceOrbit::getForceSubclass`), `SIM_EachDataCallback::callbackConst`,
  `SIM_DataFilterByType("SIM_Force")`, `SIM_FORCES_DATANAME`, `forEachConstSubData`, `SIM_DATA_CASTCONST`, and which
  header declares `SIM_EachDataCallback` (the solver includes `SIM/SIM_DataFilter.h` and `SIM/SIM_Force.h`). The
  force code was run against stand-in types. Whether `SIM_Force::getForce` is thread-safe (evaluation is kept serial).
* Houdini's impact normal convention (Impacts data).
* Bullet's `bullet_primR` rotation order (taken as XYZ Euler degrees) and `bullet_length` meaning (taken as the
  capsule's straight section without caps).
* Static Object affectors: the relationship type string `"SIM_RelationshipCollide"`, and Houdini's polygon winding
  (taken as clockwise front faces; Flip Static Mesh Winding exists in case it is the other way).
* The `PRM_SWITCHER` tab layout (`PRM_Default(count, label)` per tab, built by `Box3DParms`) for SIM data classes:
  documented for ordinary nodes only.
* The GETSET setters' float / double overloads as listed for `SIM_OptionsUser.h`.
* Objects in one solve call: Houdini passes several objects to one solver call only when they share the same Solver
  data and are in a mutual relationship; otherwise each object (or group) is solved in its own call, with feedback
  loops between them. The world lives on the first object of each call (`findOrCreateWorldData`), so objects that
  arrive in separate calls get separate worlds and neither collide nor join through constraints. Matters for several
  packed objects and above all for whole-object bodies (each RBD Object is its own DOP object). Check that the
  generated Box3D Solver node puts its objects in a mutual relationship (Bullet's does unless Solver Per Object is on):
  two RBD Objects in one Box3D Solver should affect each other both ways in the Geometry Spreadsheet's Affector
  Matrix. If not, the solver node has to create the relationship itself.

**Solver step, caching and threads** (debug build: `REBUILD` lines in the log show every world rebuild)
* `getRequestedCacheSubclass` = 1: with the DOP cache on, off, or past its memory limit, playing forward continues the
  world every frame (no rebuild per frame); scrubbing back inside the cache and playing on resumes from the cached
  frame (one rebuild, then continuous).
* An Escape mid-step (`SIM_SOLVER_FAIL`, world unstamped): Houdini discards that step and re-cooks it, the frames before
  it stay cached and correct, the re-cook rebuilds once. Also that Escape reaches `opInterrupt()` at all with no
  `opStart` of ours, during a long first-frame sync of many convex hulls.
* The world data rides on the first object that has it (else `objects(0)`): removing that object mid-simulation (a
  Delete Objects DOP, a changed solver group) loses the world, which should mean one rebuild, not a crash or a stall.
* Saved caches: loading a `.sim` / File Cache DOP cache or reopening a `.hip`, then simulating on from a loaded frame,
  rebuilds once from the geometry (no save / load of the world) and continues sensibly.
* Node messages (`addError` on `objects(0)` with `SIM_MESSAGE`): they appear on the Box3D Solver node (not only on the
  object), and clear on the next step once the problem is gone.
* `objects` sorted by object id (`box3dObjectById` binary search). Only speed is at stake: there is a linear fallback.
* Box3D's tasks on Houdini's TBB pool (`Box3DHoudiniTasks`): no stall or deadlock when several DOP networks or threaded
  SOPs cook at once, under `HOUDINI_MAXTHREADS=1`, or where threading is disabled for the cooking thread
  (`UT_Thread::isThreadingEnabled()` false: tasks run inline).
* Scene scale: Box3D's tolerances (contact slop, speculative margin, sleep and bounce thresholds, Contact Recycle
  Distance) assume metres. Check a scene modelled in centimetres or at 100x scale for jitter, gaps or tunnelling.

**Points and packed geometry**
* Point walk order is point-number order (`GA_Iterator` over `getPointRange()`): the walk position is the body id
  without `id`, and the point number written to Impacts. Check after a Sort SOP upstream, with and without `id`.
* Plain points (no packed primitive) are sized so the shape fills a box of `pscale * scale` (sphere diameter =
  `pscale`, `makeShapeSpec`). Compare with Bullet on the same points: if Bullet's sphere radius is `pscale`, ours are
  half its size.
* `collision_category` / `collision_mask` read through a 64-bit handle (`GA_ROHandleID`) from a 32-bit `i@` attribute:
  that it binds and that `-1` sign-extends to every bit (README promises it).
* Packed points: the body's primitive is found through the point's vertex (one point per packed primitive), `P` is the
  pivot's world position, and `getFullTransform4` includes that translation. Check pieces whose pivot is not at the
  centroid, and a wrangle that moves `P` of a packed point mid-simulation (an edit, so the body teleports).
* Packed types other than packed geometry and fragments (Packed Disk, Alembic, USD / packed sequences):
  `getPackedDetail()` gives usable geometry, and what loading it costs on the first frame.
* Packed geometry holding nested packed primitives, or non-polygon primitives (primitive spheres, tubes, volumes /
  VDBs, curves): hulls are built from their points only (a nested packed primitive is a single point, a sphere
  primitive its centre). Expected to be wrong; decide whether to unpack or warn.
* Mirrored packed transforms (negative determinant): the third row is flipped on read and the write-back writes a
  non-mirrored transform. Check what a mirrored piece looks like after it starts moving.
* `SourceSummaryCache` (process-wide, survives rebuilds): relies on `GU_Detail::getUniqueId()` never being reused in a
  session and every edit to a packed source bumping its `P` or primitive-list data id. Edit a packed source in place
  (no new id) and rebuild: a stale summary would give the old hull.
* Fingerprint skip: a packed transform edited by another node (Transform Pieces in a SOP Solver, a wrangle on the
  `transform` intrinsic) bumps the primitive list data id; if not, the edit is only seen at the next full scan.
  Packed points added or deleted bump `P` or `id` (packed topology is not counted).
* `animated` bodies: upstream nodes update the point's `P` / `orient` before the solver runs each step (RBD Packed
  Object's SOP re-import, a SOP Solver), and the target is reached over `dt * Time Scale`. With Time Scale != 1 the
  kinematic body pushes faster or slower than its animation looks; compare with Bullet.
* Bullet's attribute semantics as we copied them: `active`, `animated`, `deforming`, `bullet_autofit_valid` (written
  back as 1) and `awake`. Run one scene that toggles each mid-simulation in both solvers.
* Shared-string handles (`GA_AIFSharedStringTuple::getHandle`) are stable for one scan of one detail (`bullet_georep`
  and `name` are memoised by handle).

**Write-back and outputs**
* RBD Packed Object's plain `SIM_Geometry` is replaced once by an adopting `SIM_GeometryCopy`
  (`box3dWritableGeometry`): earlier cached frames keep their own poses (scrub back), and options that re-import the
  SOP every step (Overwrite with SOP / deforming geometry) overwrite the written `P` / `orient` / `v` / `w`. Is that wanted?
* Releasing a `SIM_GeometryAutoWriteLock` may bump data ids (the fingerprint is re-recorded after release). If our own
  writes still look like edits, every object scans every step: watch for `full scan, changed:` lines in a debug build.
* Downstream readers get what they expect: DOP Import (Transform Pieces / Fetch), RBD Unpack / I/O and motion blur
  read `P`, `orient`, packed transforms, `v` (of the body origin, which is the centre of mass) and `w` (radians per
  second, world). Only awake dynamic bodies are written, so a body that falls asleep keeps its last small `v` / `w`
  (possible motion-blur smear on resting pieces).
* `addPointImpact` arguments: impulse in N*s, the point number, the other object id and point, primitive -1, and time =
  `engine.getSimulationTime()` at the end of the step. Check against Houdini's impact tools (DOP Import of Impacts, a
  SOP Solver reading them), and that a fresh `SIM_Impacts` each step (removed when there are none) is what they want.
  No Feedback data: other solvers in a Multi Solver do not see our impacts.
* `mass` is written only with `computemass` (the computed value). Bullet's write-back may differ.
* Aging: `age` counts DOP seconds (Time Scale ignored, as documented) and `dead` is set once `age > life`. Compare with
  the POP Solver (it may use `>=`), and check that a `life` of 0 means never dies, as here.
* Dead deletion (`destroyPrimitive(prim, true)` for packed points): a constraint network naming the deleted piece drops
  its joint (anchor unresolved) without errors; other data referring to point numbers is renumbered.
* Point `force` / `torque`: world-space newtons and newton-metres (what POP Force writes), applied at the centre of mass
  for one step, then zeroed by the solver. A POP Force must cook before the Box3D Solver in the same step; the zeroing
  may clash with another solver reading `force` in a Multi Solver.
* Force DOPs are evaluated once per DOP step, at the start-of-step state (not per Box3D substep). A Gravity Force DOP
  adds to the solver's own Gravity (documented in the Gravity tooltip).

**Affectors (Static Object and similar)**
* `SIMgetGeometryTransform` gives the Static Object's world transform at the right time (start or end of the step),
  including Use Object Transform and an animated object. An animated Static Object is moved, not swept: bodies are not
  pushed (the README says so), and a fast move may pass through them.
* Ground Plane DOP and Static Objects whose geometry is packed, SDF-only or not polygons: only polygons are used, so
  they collide with nothing and warn "no usable collision geometry". The Ground Plane is a common setup: check what its
  geometry is (a finite grid?).
* A Static Object with Use Deforming Geometry bumps `P` every step, so its mesh is rebuilt every step (cost).
* Objects solved by another solver (Bullet, RBD, POP) that have a collide relationship with ours become static polygon
  meshes, posed each step: one-way interaction. Check the warning count and the behaviour.

**Constraint relationship nodes**
* The nine DOPs appear (`box3d_<type>conrel`, labels "Box3D <Type> Constraint Relationship") with a Data Name
  parameter defaulting to the type name, and their icons are picked up.

**Constraint networks**
* Where the Constraint Network DOP attaches the nodes wired into its second input. The HDK iterator's doc only says the
  constraint data's parent is a "data_root" passed in, so `networkKinds` searches the relationship, its
  `SIM_ConstraintNetworkRelationship` and its geometry. If every line warns "names no Box3D, Glue, Hard, ... constraint
  relationship", look in the DOP Details pane for the real parent. (A Houdini 22.0 probe showed SideFX's data directly
  under the `SIM_ConstraintNetworkRelationship`.)
* That the Constraint Network accepts our own `SIM_ConRel` subclasses in that input.
* Relationships found with `getRelationship(ingroup, i)` for both `ingroup` true and false; deduplicated by name.
* Writing the relationship's geometry from the solver (non-const `getGeometry()`; the Bullet solver writes it back the
  same way): results must not bleed into earlier cached frames. Overwrite with SOP on re-imports every step and loses
  the live frames / `broken` group (expected).
* Attach Internal Constraints to Object (`ConstraintGeometry` on the object): where the constraint data lives then.
* `next_constraint_name`: the switch happens, the new joint starts exactly where the old one was, the attribute is
  cleared. `GA_RWHandleS::set` taking a `UT_StringHolder`.
* Feedback: `force` / `torque` are now float magnitudes (were vectors); a scene with an existing vector `force` on the
  network writes only component 0. `distance` / `angle` (radians) match Bullet's names.
* Float attributes multiply the node's value (Houdini's rule): confirm it is wanted, since zero defaults (limits,
  targets) and -1 caps cannot be changed per primitive unless the node is set to 1.
* Networks from Connect Adjacent Pieces put each end at its piece's centroid; a Box3D Weld node pulls the two ends
  together, so pieces may snap (SideFX Glue / Hard all hold the offset instead).

**SideFX constraint data**
* Not compiled against the HDK: `dynamic_cast<const SIM_OptionsUser *>` on SideFX's data (cross-cast through RTTI
  across Houdini's libraries), `SIM_Options::importOption` for `fpreal64` and `UT_Vector3D` (and that it converts int /
  bool options), `SIM_Data::getDataType()`, `SIM_Query::getFieldFloat`.
* Glue: Break Force = strength / timestep is our own rule. Compare breakage with Bullet on a typical fracture; Box3D's
  weld also carries weight, so a tall glued wall may break at its base where Bullet's would not.
* Hard: `all` ignoring the rest length, and `position` with Houdini's default rest length 1 and no `restlength`
  attribute giving a rod of length 1 (the documented meaning), are guesses about Bullet's behaviour.
* `constraint_type` missing = all.
* Which end is Cone Twist's goal (taken as point 1); only matters when goal and constrained axes differ.
* Spring: whether Bullet's Spring applies an angular spring with the same strength (assumed, from the HDK's
  `SIM_ConRelSpring::getTorque`).
* Soft / Spring `all` with a rest length shifts the frames along the anchor line when the joint is made; after a
  world rebuild from a stretched pose the offset direction is re-derived from the anchors and may differ slightly.

**Whole-object bodies**
* `SIM_PositionSimple`'s translation convention: written as the pivot's displacement, then checked through
  `getTransform` and rewritten as the pivot's world position if the pivot does not land on the body. A drift or jump
  on the first write-back points here.
* The `Colliders` label on RBD Point Object and RBD Fractured Object (only RBD Object and RBD Packed Object were
  checked); its value is not used, only "present and not PackedObject".
* RBD State (an `RBD_State`) casts to `SIM_Motion` / `SIM_PositionSimple` through `getPointerToType` without linking
  Houdini's RBD library; its initial velocity and angular velocity are read (radians) and the body starts with them.
* `SIM_DATA_CREATE(obj, "Position", SIM_Motion, 0)` when the object has no Position data, and whether writing
  `Position` from a solver needs anything more (Bullet and the RBD solver do it).
* The viewport and DOP Import SOP follow the written Position data (through the geometry's `positionpath`).
* Use Object Transform: a scaled or sheared geometry transform is baked into the shapes.
* Without Box3D Data an RBD Object now uses its own RBD State / PhysicalParms / BulletData (names from a Houdini 22.0
  probe). Not compiled against the HDK: `getConstNamedSubData("PhysicalParms")` on the object and `"BulletData"` on
  its `SIM_Geometry`, `importOption` for strings (`bullet_georep`) and bool options read as numbers,
  `SIM_Query::getFieldString`. Check in Houdini: a default RBD Object weighs density x volume, bounces (0.5), and
  collides as a convex hull. Whether `bullet_primT` is relative to the pivot (taken so, as for points) and
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
`updateFromRotationMatrix`; node icons as `config/Icons/DOP_<name>.svg`.

## Known gaps / TODO

The one list of what is still to do. Keep it here only (no TODO comments in the code).

* `newobjects` are solved on their first step instead of keeping their initial state (`syncFromGeometry`); e.g. step
  them as kinematic once.
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
* Sticky collisions; contact / sensor events beyond Impacts.
