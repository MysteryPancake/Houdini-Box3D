# Solver: sync, skip fingerprint, write-back, threading

The per-object sync pass, the skip fingerprint, `writeResultsBack` and threading. The step order:
[solver.md](solver.md). What the sync does to a body: [bodies.md](bodies.md). Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## syncObject (per object)

`syncObject` is the driver; the passes are helpers in `Box3DPointSync.cpp`: `collectSlots`, `readSlots` / `readSlot`,
`planSlots`, `reserveMirror`, `ApplyPass` (pass 2) and `NameIndex` (the `name` map for joint anchors). A cancel in
`planSlots` or `ApplyPass::run` makes `syncObject` return early.

1. Read-locks the `SIM_Geometry` (objects without one are skipped). `PointAttrs` binds
   `id, active, v, w, orient, restxform`.
2. Box3D Data: body type and `BodyProps` (its values that change existing bodies), plus Add Impact Data.
3. **Skip check** (see below). On a skip, the object's entries are counted and nothing per point runs.
4. Otherwise binds the remaining attributes and builds the `ScanContext` (world, geometry, attributes, node settings,
   shape sources, georep lookup) that the per-point helpers take.
5. **Pass 1** (`collectSlots`: serial offsets; `readSlots`: parallel above `kMinParallelReadPlain` points,
   `kMinParallelReadPacked` with primitives): per point body id (`id`, else the point number = its position in the
   walk), key, `getPackedInfo` (with the 3x3's row scale, kept for the write-back), `readPointState` (a packed point's
   `restxform` rotation is split once and kept on its slot, `Box3DRestCache`, until the attribute's data id changes),
   `active`, the mirror entry (read only), and whether it needs shapes built: new body, or a rebuild (`deforming`,
   `bullet_autofit_valid == 0`, convex / concave scale change). **Pass 1b** (`planSlots`; parallel, chunks of
   `kPlanChunk` with an Escape check between): `planShapes` for those points, into a per-scan plan list
   (`Box3DPointSlot::plan`). Hull building, the slow part of a rebuild, happens here; `ShapeSources`, `GeorepLookup`
   (mutexes) and Box3DWorld's hull calls are thread-safe. Bodies planned from one source share their `BuiltShapes`
   (`ShapePlan::built`, a reference count, not a copy), and a new packed body's rest is worked out here too
   (`RestInit`, `computeRestInit`; the array is not cleared first, so the threads that fill it first touch its pages)
   so pass 2 only copies it. The slots that need a plan are found in parallel (`slotsNeedingPlans`). Pass 2 takes the
   plans (`takePlan`). **Pass 1c** (`stampQuietSlots`; parallel): reads the settings (`SlotPlans::props`) of the
   points that need them (new, rebuilt, every one when a settings marker changed), and stamps the entry of every point
   with nothing to apply (it owns the entry: same offset and id; pose, type and settings equal the mirror's; not
   kinematic, deforming, dead or a problem point), so pass 2 only lists it. A duplicate `id` never qualifies, so the
   entry stays with the point that last owned it.
6. **Pass 2** (`ApplyPass`; serial, in body id order, point order for equal ids, so the bodies are created in the same
   order whatever order the points are in and the result does not depend on it, test `point_order`; the `name` index
   still takes the first point in point order, sorted in `NameIndex::finish`, test `duplicate_name_point_order`):
   duplicate `id` -> counted, first wins (test `duplicate_id_first_point_wins`); non-finite `P` -> counted, its body
   removed; `dead` -> queued for deletion, its body removed; unknown key -> `addPointBody`; known key ->
   `updateExistingBody`: mid-simulation updates (shape rebuild, then settings), type change (`wantedType`), then the
   animated target or an external edit (`poseDiffers`, eps 1e-5, q and -q equal -> `setTransformByHandle` only if the
   place differs (`placeDiffers`: a `v` / `w` edit alone is not a move), then `setVelocityByHandle`). Both paths then
   share one tail: point bookkeeping and stamp, queued point writes, the object's body list. When the scan has at
   least `kMinBulkCreate` points and the mirror holds at most a quarter as many bodies (a first frame, a rebuild),
   pass 2 runs between `beginBulkCreate` / `endBulkCreate` (a scope guard in `syncObject`): the new bodies'
   broad-phase proxies are built into the tree at the end, one build instead of one insertion each (fork:
   `extern/box3d/CLAUDE.md`; test `bulk_create`). A scan adding a few bodies to a large world inserts as before.
7. Records the fingerprint; problems (failed bodies, duplicate ids, bad P), deforming bodies and driven animated bodies
   keep the object scanning.

## Skipping the sync scan

`Box3DObjectSync::fp` (`Box3DSyncFingerprint`): point count and offset range (plain points only; packed topology is
assumed fixed, a new instance gets a new `id`), data ids of `P`, `orient`, `v`, `w`, `id`, `active`, the node body
type, and the **prop markers**: data ids of `kPropAttribNames` (the attributes that change settings, ask for rebuilds
or rename pieces, looked up by name; the table is in `Box3DAttribNames.h` and `Box3DSyncFingerprint::kNumPropIds` is
its length), the primitive list's data id (`primitiveListDataId`: packed transforms are intrinsics; bumping it is up
to whoever edits a primitive, so an edit that does not is picked up on the next scan) and the node's `BodyProps` and
Add Impact Data. The fingerprint keeps the node's `BodyProps` itself (`nodeProps`) and compares it with
`BodyProps::operator==` (two NaNs are equal, so an unchanged NaN setting still skips), so there is no hand-kept field
list and no hash collision. Equal fingerprint -> skip.

* Recorded at the end of a scan and again at the end of a step that wrote the object's geometry (`noteOwnWrite`, then
  `recordFingerprintsAfterWrites` once per object after every write lock is released, since releasing may bump ids). A
  debug log showing `full scan, changed: ...` every step means something else touches the geometry after the solver.
* `forceScan` keeps an object scanning after a write the scan must act on (`active = 1`, `dead = 1`, deleted points).
* Records of objects not seen are erased, so a returning object scans. A fresh world starts without records.
* Debug builds log `sync: N of M object(s) unchanged, scan skipped` when objects are skipped.

## writeResultsBack

`SIM_SolverBox3D::writeResultsBack` is the driver (it owns the node messages); the work is in `Box3DWriteBack.cpp`:
`collectMovedBodies`, `writeWholeObjects`, then per object `writeObjectResults` (`BodyWriter` for one
body's attributes and packed transform).

0. Move events (read once into `st.moves` right after the step; impacts use them too): every awake body, its transform
   and its velocities (a body falling asleep appears once more, with zero velocity). Only live dynamic entries become
   items, which point at their move event rather than copying it; `awake` changes are noted. A compound's piece takes
   its compound's velocity (`glueLin` / `glueAng`). Items are grouped per object.
1. `box3dWritableGeometry`: a plain `SIM_Geometry` (RBD Packed Object) is replaced once by a `SIM_GeometryCopy`
   (`SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`).
2. `ensureResultAttribs` creates `v`, `w`, `orient` (a new `orient` is set to identity).
3. `resolvePointOffsets`: the stored offset if it still names the point (in range, same id / number), else by id.
4. Parallel pass: `hardenWrittenPages` first when parallel (offsets bucketed per page, see `hardenWrittenPages`; one
   full call when nearly all buckets are hit), then write each body's attributes, compute the new packed 3x3 (the row
   scale the sync stored, the new rotation: no matrix decomposition), set it on the body's own primitive
   (`setLocalTransform` + `transformDirty`), and store in the mirror what the geometry now holds without reading it
   back (`storedPosition`, `sanitizeQuat` of the written rotation). It must agree with `readPointState` within
   `poseDiffers`' tolerances (1e-5; q and -q equal), or our write looks like an external edit; a packed transform's
   float round trip stays well inside them. `BOX3DDOP_VERIFY_READBACK=1` re-reads and logs `READBACK MISMATCH` when
   `poseDiffers` would see a difference. Afterwards one `getPrimitiveList().bumpDataId()` (the HDK leaves marking
   intrinsics dirty to the caller). A point no longer on a packed primitive is counted as unmatched.
5. The items are reset in `collectMovedBodies`' parallel pass (`writeItems` is only resized), so a step does not
   rewrite them all on one thread.

Only dynamic bodies are written; of several points sharing an `id`, only the first.

**Cost.** With the simulation cache on, the first writable access to an object's Geometry makes Houdini copy the
previous frame's geometry (about 2.4 ms for 30000 packed bodies, in `box3dWritableGeometry`); it is Houdini's, and
gone with Cache Simulation off. Doing it on a pool task during `b3World_Step` was slower than the copy.

## Threading

Box3D starts no threads of its own: every world gets a `Box3DHoudiniTasks` (`b3dop::TaskRunner` over `UT_TaskGroup`,
each task in a `UT_TaskScope`), so all worlds share Houdini's TBB pool. Safe because tasks carry their worker index,
the stepping thread runs solver worker 0 (which can finish alone, so a busy pool cannot deadlock), and `step` runs
inside `tbb::this_task_arena::isolate`. Without a runner (the tests) Box3D uses its own pool. Driving the step with
`b3World_BeginStep` / `ContinueStep` over the same runner gave the same step time (piles of 4000 and 30000 boxes), so
the callbacks stay.

Worker count: `autothreads` (off by default) = `UT_Thread::getNumProcessors()`, else `threads` (default
`defaultWorkerCount()`, `Box3DCpu.h`: physical performance cores), never above Houdini's count; a change rebuilds the
world.

The step's own per-body passes run in parallel too: `mergeMoveEvents` (a frame's first step's events are the moves,
their bodies noting a slot only if a later step needs it, `moveSlotsPending`; a later step with the same bodies in the
same order replaces them without touching a body, else events are sorted into overwrite / new slot / no entry and
slots numbered by a serial pass), `collectMovedBodies` and `resolvePointOffsets` (stored offsets checked in parallel,
only the moved points looked up serially), with `kCheapGrain` tasks. The shape-type (`GeorepLookup`), source-summary
and fit lookups of the parallel shape pre-pass keep each thread's last hit, so the threads do not all take the same
shared lock per point. `planShapes` keeps each thread's last build (`ShapeMemo`: keyed by the source's shared
implementation, shape type, packed scale and pivot, pscale / scale, autofit, static, the primitive overrides; valid
for one scan, the `georep.generation`), so the instances of one packed source build their shapes once (test
`instances_differ`). A `thread_local` scratch vector must be bound to a reference before a lambda uses it: each worker
thread has its own copy. Test `parallel_matches_serial` compares one thread with all. The plugin's loops use
`UTparallelFor` past thresholds (`kMinParallelPoints` and neighbours, `kMinParallelForces` for Force DOPs, in
`Box3DSolverInternal.h`; overridable from the environment in debug builds); parallel lambdas only read shared data or
write their own element. The world and mirror are touched in serial passes; the shared-state mutex is held for the
whole step.
