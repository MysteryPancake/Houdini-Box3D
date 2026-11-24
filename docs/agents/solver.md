# Solver and world state

The shared world state, the step order, the sync pass, the skip fingerprint, write-back and threading. What the sync does to a body is in [bodies.md](bodies.md); forces in [forces-affectors.md](forces-affectors.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Shared state (`SIM_Box3DWorld`)

The Box3D world is a **rebuildable cache**; the DOP objects' point attributes (a whole object's `Position` data) are
authoritative. Snapshotting the world into the DOP cache is too slow, so don't.

* DOPs copy `SIM_Data` (`makeEqual`) every step, so copies share a `std::shared_ptr<Box3DSharedState>`.
* The stamp (`stampTime`, `stamped`) lives in the shared state, so an older timestep cannot claim the world is current.
  `canContinue(now, dt)`: `(now - stampTime) == dt` within `1e-3 * dt` (a difference, so start vs end of step does not
  matter).
* Anything else (first frame, scrub, re-solve, cache reload, changed thread count) -> `createFreshWorld`: a new shared
  state for this data only; other timesteps keep the old one. The empty mirror makes every point a new body.
* No save / load: reloaded sims rebuild. `getMemorySizeSubclass` reports the state's size
  (`Box3DSharedState::estimatedBytes`, set by `claimMemory` after each solve) from the one copy that stepped it last
  (`memoryOwner`), so sharing copies do not each count the world against the cache. `Box3DWorld::estimatedBytes` is Box3D's own
  per-world count (`b3World_GetCounters`, which includes the hulls cloned into its database) plus the meshes and
  height fields it owns; a rough figure, not exact. `getRequestedCacheSubclass` returns 1 (debug: `BOX3DDOP_REQUESTED_CACHE`).
* Because a rebuild has only the geometry, the write-back always writes `P`, `orient`, `v`, `w` and packed transforms.

`Box3DBodyMirror` (per body): the last pose / velocity given to or read from Box3D (to detect edits by other nodes),
`type` (`dynamic()` derives from it), `handle`, the point (`pointOff`, `pointId`, `pointIndex`, `objectId`), `stamp`, the packed primitive
and its 3x3, and what the body was built from (`props`, `volume`, `shapeSig`, `shapeless`, `autoScale`, `sizeKey`,
`animated`, `driven`, `sleeping`). Box3D's body user data points at the entry, so entries live in `Box3DMirrorMap`'s
slab of fixed-size chunks (never moved; a `std::deque` would give each of these 500-byte entries its own heap block), found
through a flat key table, and are erased only with their body. A fresh world's `reserveMirror` allocates the chunks on the TBB
pool, which touches their pages there, and `collectSlots` does the same for the slot list. Entries are made when handed out, so
they are in the cache when used; making one is memory bound (about 100 ns of cold writes), so a smaller `Box3DBodyMirror` is what
would cut the rest.
The entry's fields are ordered hot first (pose, handle, type, stamp, point, packed data: what the write-back, forces and the
quiet-point check read per body), then the settings it was built from; keep new per-step fields in the first part.

Scratch reused every step: `slots` (per object: offset, key, cached entry, freshly read state and this scan's shape
decisions per point; a cached entry is trusted while offset and key match; when entries are erased, only the slots of the
objects that lost them forget their cached entries), `writeItems`,
`moves`, `contacts`. `syncStamp` / `stampedCount`: equal counts mean nothing to remove.
`isLive(entry)` = its object was visited by this sync and the entry carries the object's `liveStamp`; the entry keeps a
pointer to its object's record (`sync`), valid because records are erased only after their unvisited bodies are removed.

## Solver step (`solveObjectsSubclass`)

**Two step counts.** *Substeps* (`substeps`) splits the DOP step into whole Box3D steps, each with its
own `b3Collide` (contact detection), so collisions are found `fps * Substeps` times a second. *Constraint Iterations*
(`solverpasses`) is Box3D's own
`subStepCount`: each pass is a smaller time step for the contact / joint solve, not another iteration at the same
positions (Box3D runs one solve and one relax per pass). Not to be confused with `restitutionIterations`. Kinematic
targets (`setTargetTransformByHandle`) and the Glue impact decay use the whole frame's `stepDt`, so they hold for any
Substeps. Box3D clears its move, joint and hit events at the start of every step, so each is merged after every Box3D step:
moves by body, joints over their force / torque threshold into `brokenThisFrame` (broken once, in Updating Constraint
States), and hit pairs into `hitPairs` when Impact Min Speed is on (`hitsCollected`). Impacts read Box3D's contact impulse events
after every Box3D step (`collectImpacts`, `impactsCollected`, appended; the world makes them only while some object has
Add Impact Data), so a frame's impulses add up over its steps: a contact an earlier step already recorded (same bodies and side, position to about 1 cm; looked up in `impactSlots`) takes the impulse instead of a new record, so Substeps do not multiply the records (test `impacts_substeps`); with one step they are read after it. Point `force` / `torque` are read and cleared by
the first step and applied again from `PointForceCarry` in the others.

```
StepProgress (UTgetInterrupt)                  Escape checks; Houdini's own "Stepping simulation" bar stays
worldData = findOrCreateWorldData(objects)     first object carrying "Box3DWorld", else created on objects(0)
rebuild if invalid / thread count changed / stamp mismatch -> createFreshWorld
lock st->mutex
stepDt = dt * timescale; world.applyTuning(currentWorldSettings())   (stepDt stays the whole frame's time)
syncFromGeometry   per object syncObject(); syncAffectorBodies (Static Objects); remove gone bodies; syncJoints; syncCompounds;
                   pruneHulls; warnings
world.step         n = Substeps (`substeps`, at least 1; 1 when stepDt is 0). Each of the n steps: applyExternalForces (Box3D clears
                   forces every step), b3World_Step(stepDt / n, subSteps = Constraint Iterations), then its move events are merged
                   into st.moves by body (Box3D clears them every step; the last event wins, so fell-asleep survives).
                   applyExternalForces (Use Force DOPs): SIM_Force data per object -> getForce at each dynamic body's
                   centre of mass, plus point force / torque -> applyForceByHandle
expandCompoundMoves  Glue as One Body: a move event per piece of a moved compound (glue-compound.md)
writeResultsBack   awake dynamic bodies (move events): P / orient / v / w / packed transform / awake
jointsAfterStep    broken joints, feedback and live frames onto the networks (joints.md)
recordImpacts      Add Impact Data -> SIM_Impacts (only on steps with an impact)
writePointAttrs    queued point writes: bullet_autofit_valid = 1, computed mass, rest attributes
ageObjects         Enable Aging only
deleteDeadPoints   `dead` points
stamp = now
```

**Performance Monitor.** Each stage is a `UT_PerfMonAutoSolveEvent(this, "<category>/<stage>")`: the `/` makes the
Stats view nest the stage under its category, as Bullet's Pre-Solve / Stepping Simulation / Post-Solve. Pre-Solve: Rebuilding World,
Updating Bodies, Updating Colliders, Removing Bodies, Updating Constraints. Stepping Simulation: Applying Forces, Stepping World.
Post-Solve: Exporting Simulation Results, Updating Constraint States, Recording Impacts, Updating Points, and Exporting
Collision Shapes (Export Collision Shapes on). Opened only on the solve thread.
While a profile records, `reportStepProfile` (`SIM_SolverBox3D.cpp`) adds one `Stepping Simulation/Box3D <part>` event per part of Box3D's own
step timing (`Box3DWorld::stepProfile`, `b3World_GetProfile`: Pairs, Collide, Solver Setup, Constraints and its parts (Prepare, Integrate Velocities, Warm Start, Solve, Integrate Positions, Relax, Restitution, Store
Impulses), Split Islands (runs beside the constraints), Transforms, Events, Refit, Fast Bodies, Sensors, Sleep Islands), each given Box3D's time with `overrideTime`. Their sum is about Stepping World's time and they sit beside it, so
do not add them to it.

**Guide geometry** (`Box3DGuide.cpp`; Guide Options: Show Collision Shapes, Shape Color). A `SIM_GuidePerObject`
built by `buildCollisionGuide` from the live world (`bodyShapeOutlines`: hulls, spheres, capsules), each body placed at
the pose the displayed frame's geometry shows, so cached frames draw where the pieces were. Open polylines with `Cd`,
dimmed for sleeping and static / kinematic bodies. The mutex is only tried: a guide asked for mid-step is skipped.

**Export Collision Shapes** (`exportshapes`, Visualization tab; `exportCollisionShapes` in `Box3DGuide.cpp`). The same
lines (`drawObjectShapes`, shared with the guide) as world-space open polylines in a `SIM_GeometryCopy` named
`kShapesDataName` (`Box3DShapes`) on every object with bodies, updated each step after the point writes (the solver
holds the mutex; poses are the mirror's, from the object's `bodies` list, so no geometry is read; a whole object is read from its
Position data). A hull's or mesh's lines share its vertices as points. The outlines are kept in body
space (`Box3DShapeExport`, `shapeExports`); a step only moves the points of bodies whose pose changed and recolours those whose shade
did (found in parallel, written in parallel once `kMinParallelMovers` bodies changed, after the attributes' pages are hardened). A step with no move event,
no scanned object and the same `bodyVersion` as the last export (`shapesFresh`) writes nothing: the geometry the object carries over is right. The geometry stores the signature of the bodies and shapes it was built for (`kShapesSigName` detail attribute: handle, `shapeSig`,
shape count, body type, scale of each); a different signature or a geometry that does not match (another timeline) builds it again. `Cd` is white times the dimming; the SOP tints it. The Box3D Solver
SOP's `GUIDE_GEO` imports it (see [solver-sop.md](solver-sop.md)); turning the toggle off stops writing but leaves the last data
in the cache. A static concave piece's mesh is drawn as its boundary and creased edges (`meshOutline` in `Box3DWorld.cpp`, thinned past `kMaxMeshOutlineEdges`; test `collision_shapes_mesh`). Not drawn: affector colliders, and anything on an object's creation frame without Solve On Creation Frame (no
body yet; expected).

**New objects.** Objects in `newobjects` (created this timestep) keep their initial state unless their object node has
Solve On Creation Frame on (`SIM_Object::getSolveFirstFrame()`, as Bullet): `solveObjectsSubclass` drops the others
by object id before anything else, so for this step they get no body, no write-back, no aging and no joints, and the
rest of the step sees only the other objects. They are solved from the next timestep, as ordinary objects. A step whose
objects are all dropped returns at once without a world or a stamp, so the next step builds the world as for a first
step. New objects with the toggle on are solved in this step like the rest (added from `newobjects` if `allObjects`
lacks them, never twice). Both work whether the engine lists new objects in `newobjects` only or in both arrays.

**Escape.** `StepProgress::check()` calls `UTgetInterrupt()->opInterrupt()` with no percent and no `opStart` of our
own (SideFX's `SIM_VolumetricConnectedComponentBuilder` does the same), so Houdini's own "Stepping simulation" message
and bar are left alone. SIM_Engine discards and re-cooks an interrupted step itself (`setSimulationTime`,
`myLastCookInterrupted`); the stamp check makes that re-cook rebuild the world. `check()` is called at the
start of the solve, before each object, every `kProgressEvery` points in `syncObject` pass 2 (serial, between
points, so no half-built body), and once before the step. A cancel latches. Cancelled after the sync began:
`st->stamped = false` (the half-synced world can never be continued; the next solve that would continue it rebuilds),
a `UT_ERROR_MESSAGE` "step cancelled", and `SIM_SOLVER_FAIL`. `b3World_Step` itself is not interruptible, and
once it has run the step always finishes (write-back etc. are quick), so a cached step is always whole.
`StepProgress` is at global scope in `Box3DSolverInternal.h` (the solver header forward-declares it).

Data names: `"Box3DWorld"`, `kBox3DDataName` (`"Geometry/Box3DData"`, under the object's Geometry like Bullet Data; SIM_Box3DData.h), `SIM_GEOMETRY_DATANAME`.
Houdini warns on data it has no `SIM_DataHint` for under its parent ("Unexpected subdata found" on the Geometry,
"attached to an unexpected parent data" on the data node): `initializeSIM` adds one for Box3DData under `SIM_SopGeometry` (a Packed Object's Geometry) and `SIM_GeometryCopy` (the Solver SOP's).

### syncObject (per object)

`syncObject` is the driver; the passes are helpers in `Box3DPointSync.cpp`: `collectSlots`, `readSlots` / `readSlot`,
`planSlots`, `reserveMirror`, `ApplyPass` (pass 2) and `NameIndex` (the `name` map for joint anchors). A cancel in
`planSlots` or `ApplyPass::run` makes `syncObject` return early.

1. Read-locks the `SIM_Geometry` (objects without one are skipped). `PointAttrs` binds `id, active, v, w, orient, restxform`.
2. Box3D Data: body type and `BodyProps` (its values that change existing bodies), plus Add Impact Data.
3. **Skip check** (see below). On a skip, the object's entries are counted and nothing per point runs.
4. Otherwise binds the remaining attributes and builds the `ScanContext` (world, geometry, attributes, node settings,
   shape sources, georep lookup) that the per-point helpers take.
5. **Pass 1** (`collectSlots`: serial offsets; `readSlots`: parallel above `kMinParallelReadPlain` points, `kMinParallelReadPacked` with primitives): per point body id (`id`, else the point number = its
   position in the walk), key, `getPackedInfo` (with the 3x3's row scale, kept for the write-back), `readPointState` (a packed point's `restxform` rotation is split once and kept on its slot, `Box3DRestCache`, until the attribute's data id changes),
   `active`, the mirror entry (read only), and whether it needs shapes built: new body, or a rebuild (`deforming`,
   `bullet_autofit_valid == 0`, convex / concave scale change).
   **Pass 1b** (`planSlots`; parallel, chunks of `kPlanChunk` with an Escape check between): `planShapes` for those points, into a per-scan
   plan list (`Box3DPointSlot::plan`). Hull building, the slow part of a rebuild, happens here; `ShapeSources`,
   `GeorepLookup` (mutexes) and Box3DWorld's hull calls are thread-safe. Bodies planned from one source share their `BuiltShapes` (`ShapePlan::built`, a reference count, not a copy), and a new packed body's rest is worked out here too (`RestInit`, `computeRestInit`; the array is not cleared first, so the threads that fill it first touch its pages) so pass 2 only copies it. The slots that need a plan are found in parallel (`slotsNeedingPlans`). Pass 2 takes the plans (`takePlan`).
   **Pass 1c** (`stampQuietSlots`; parallel): reads the settings (`SlotPlans::props`) of the points that need them (new, rebuilt, every one when a settings marker changed), and stamps the entry of every point with nothing to apply (it owns the entry: same offset and id; pose, type and settings equal the mirror's; not kinematic, deforming, dead or a problem point), so pass 2 only lists it. A duplicate `id` never qualifies, so the entry stays with the point that last owned it.
6. **Pass 2** (`ApplyPass`; serial, in body id order, point order for equal ids, so the bodies are created in the same order whatever order the points are in and the result does not depend on it, test `point_order`; the `name` index still takes the first point in point order, sorted in `NameIndex::finish`, test `duplicate_name_point_order`): duplicate `id` -> counted, first wins (test `duplicate_id_first_point_wins`); non-finite `P` -> counted, its body removed;
   `dead` -> queued for deletion, its body removed; unknown key -> `addPointBody`; known key -> `updateExistingBody`:
   mid-simulation updates (shape rebuild, then settings), type change (`wantedType`), then the animated target or an
   external edit (`poseDiffers`, eps 1e-5, q and -q equal -> `setTransformByHandle` only if the place differs (`placeDiffers`: a `v` / `w` edit alone is not a move), then `setVelocityByHandle`). Both
   paths then share one tail: point bookkeeping and stamp, queued point writes, the object's body list.
   When the scan has at least `kMinBulkCreate` points and the mirror holds at most a quarter as many bodies (a first frame, a rebuild), pass 2 runs between `beginBulkCreate` / `endBulkCreate` (a scope guard in `syncObject`): the new bodies' broad-phase proxies are built into the tree at the end, one build instead of one insertion each (fork: `extern/box3d/CLAUDE.md`; test `bulk_create`). A scan adding a few bodies to a large world inserts as before.
7. Records the fingerprint; problems (failed bodies, duplicate ids, bad P), deforming bodies and driven animated bodies
   keep the object scanning.

### Skipping the sync scan

`Box3DObjectSync::fp` (`Box3DSyncFingerprint`): point count and offset range (plain points only; packed topology is
assumed fixed, a new instance gets a new `id`), data ids of `P`, `orient`, `v`, `w`, `id`, `active`, the node body type,
and the **prop markers**: data ids of `kPropAttribNames` (the attributes that change settings, ask for rebuilds or rename pieces, looked
up by name; the table is in `Box3DAttribNames.h` and `Box3DSyncFingerprint::kNumPropIds` is its length), the primitive list's data id (`primitiveListDataId`: packed transforms are intrinsics; bumping it is up to
whoever edits a primitive, so an edit that does not is picked up on the next scan) and the node's `BodyProps` and Add Impact Data. The fingerprint keeps the node's `BodyProps` itself (`nodeProps`) and
compares it with `BodyProps::operator==` (two NaNs are equal, so an unchanged NaN setting still skips), so there is no hand-kept field list and no hash collision. Equal fingerprint ->
skip.

* Recorded at the end of a scan and again at the end of a step that wrote the object's geometry (`noteOwnWrite`, then
  `recordFingerprintsAfterWrites` once per object after every write lock is released, since releasing may bump ids). A debug log showing `full scan, changed: ...` every step means
  something else touches the geometry after the solver.
* `forceScan` keeps an object scanning after a write the scan must act on (`active = 1`, `dead = 1`, deleted points).
* Records of objects not seen are erased, so a returning object scans. A fresh world starts without records.
* Debug builds log `sync: N of M object(s) unchanged, scan skipped` when objects are skipped.

### writeResultsBack

`SIM_SolverBox3D::writeResultsBack` is the driver (it owns the node messages); the work is in `Box3DPostStep.cpp`:
`collectMovedBodies`, `writeWholeObjects`, then per object `writeObjectResults` (`BodyWriter` for one
body's attributes and packed transform).

0. Move events (read once into `st.moves` right after the step; impacts use them too): every awake body, its
   transform and its velocities (a body falling asleep appears once more, with zero velocity). Only live dynamic entries become items, which point at their
   move event rather than copying it; `awake` changes are noted. A compound's piece takes its compound's velocity
   (`glueLin` / `glueAng`). Items are grouped per object.
1. `box3dWritableGeometry`: a plain `SIM_Geometry` (RBD Packed Object) is replaced once by a `SIM_GeometryCopy`
   (`SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`).
2. `ensureResultAttribs` creates `v`, `w`, `orient` (a new `orient` is set to identity).
3. `resolvePointOffsets`: the stored offset if it still names the point (in range, same id / number), else by id.
4. Parallel pass: `hardenWrittenPages` first when parallel (offsets bucketed per page, see `hardenWrittenPages`; one full call when nearly all
   buckets are hit), then write each body's attributes, compute the new packed 3x3 (the row scale the sync stored, the new rotation: no matrix
   decomposition), set it on the body's own primitive (`setLocalTransform` + `transformDirty`), and store in the mirror what the geometry now holds without reading it back (`storedPosition`,
   `sanitizeQuat` of the written rotation). It must agree with `readPointState` within `poseDiffers`' tolerances (1e-5;
   q and -q equal), or our write looks like an external edit; a packed transform's float round trip stays well inside
   them. `BOX3DDOP_VERIFY_READBACK=1` re-reads and logs `READBACK MISMATCH` when `poseDiffers` would see a difference.
   Afterwards one `getPrimitiveList().bumpDataId()` (the HDK leaves marking intrinsics dirty to the caller). A point no longer on a packed
   primitive is counted as unmatched.
5. The items are reset in `collectMovedBodies`' parallel pass (`writeItems` is only resized), so a step does not rewrite them all on one thread.

Only dynamic bodies are written; of several points sharing an `id`, only the first.

**Cost.** With the simulation cache on, the first writable access to an object's Geometry makes Houdini copy the previous frame's
geometry (about 2.4 ms for 30000 packed bodies, in `box3dWritableGeometry`); it is Houdini's, and gone with Cache Simulation off.
Running it on a pool task while `b3World_Step` runs made the step slower than the copy took (Box3D's workers wait on each other at every stage).

### Threading

Box3D starts no threads of its own: every world gets a `Box3DHoudiniTasks` (`b3dop::TaskRunner` over `UT_TaskGroup`,
each task in a `UT_TaskScope`), so all worlds share Houdini's TBB pool. Safe because tasks carry their worker index, the
stepping thread runs solver worker 0 (which can finish alone, so a busy pool cannot deadlock), and `step` runs inside
`tbb::this_task_arena::isolate`. Without a runner (the tests) Box3D uses its own pool. Driving the step with `b3World_BeginStep` / `ContinueStep` over the same runner gave the same step time (piles of 4000 and 30000 boxes), so the callbacks stay.

Worker count: `autothreads` (off by default) = `UT_Thread::getNumProcessors()`, else `threads` (default
`defaultWorkerCount()`, `Box3DCpu.h`: physical performance cores), never above Houdini's count; a change rebuilds the
world.

The step's own per-body passes run in parallel too: `mergeMoveEvents` (the frame's first step's events are the moves, their bodies noting a slot only if a later step needs it, `moveSlotsPending`; a later step with the same bodies in the same order replaces them without touching a body; else events are sorted into overwrite / new slot / no entry, slots numbered by a serial pass), `collectMovedBodies` (kinds, a serial numbering pass, then the items) and `resolvePointOffsets` (stored offsets checked in parallel, only the moved points looked up serially), with `kCheapGrain` tasks. The shape-type (`GeorepLookup`), source-summary and fit lookups of the parallel shape pre-pass keep each thread's last hit (a key is a function of its content), so the threads do not all take the same shared lock per point. `planShapes` keeps each thread's last build (`ShapeMemo`: keyed by the source's shared implementation, shape type, packed scale and pivot, pscale / scale, autofit, static, the primitive overrides; valid for one scan, the `georep.generation`), so the instances of one packed source in a run build their shapes once (test `instances_differ`). A `thread_local` scratch vector must be bound to a reference before a lambda uses it: each worker thread has its own copy. Test `parallel_matches_serial` compares one thread with all. The plugin's loops use `UTparallelFor` past thresholds (`kMinParallelPoints` and neighbours, `kMinParallelForces` for Force DOPs, in
`Box3DSolverInternal.h`; overridable from the environment in debug builds); parallel lambdas only read shared data or
write their own element. The world and mirror are touched in serial passes; the shared-state mutex is held for the
whole step.
