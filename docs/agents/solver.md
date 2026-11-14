# Solver and world state

The shared world state, the step order, the sync pass, the skip fingerprint, write-back and threading. What the sync does to a body is in [bodies.md](bodies.md); forces in [forces-affectors.md](forces-affectors.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Shared state (`SIM_Box3DWorld`)

The Box3D world is a **rebuildable cache**; the DOP objects' point attributes (a whole object's `Position` data) are
authoritative.

* DOPs copy `SIM_Data` (`makeEqual`) every step, so copies share a `std::shared_ptr<Box3DSharedState>`.
* The stamp (`stampTime`, `stamped`) lives in the shared state, so an older timestep cannot claim the world is current.
  `canContinue(now, dt)`: `(now - stampTime) == dt` within `1e-3 * dt` (a difference, so start vs end of step does not
  matter).
* Anything else (first frame, scrub, re-solve, cache reload, changed thread count) -> `createFreshWorld`: a new shared
  state for this data only; other timesteps keep the old one. The empty mirror makes every point a new body.
* No save / load: reloaded sims rebuild. `getMemorySizeSubclass` returns a small constant so sharing copies do not each
  count the world against the cache. `getRequestedCacheSubclass` returns 1 (debug: `BOX3DDOP_REQUESTED_CACHE`).
* Because a rebuild has only the geometry, the write-back always writes `P`, `orient`, `v`, `w` and packed transforms.

`Box3DBodyMirror` (per body): the last pose / velocity given to or read from Box3D (to detect edits by other nodes),
`type` (`dynamic()` derives from it), `handle`, the point (`pointOff`, `pointId`, `pointIndex`, `objectId`), `stamp`, the packed primitive
and its 3x3, and what the body was built from (`props`, `volume`, `shapeSig`, `shapeless`, `autoScale`, `sizeKey`,
`animated`, `driven`, `sleeping`). Box3D's body user data points at the entry, so entries live in an `unordered_map`
(stable nodes) and are erased only with their body.

Scratch reused every step: `slots` (per object: offset, key, cached entry, freshly read state and this scan's shape
decisions per point; a cached entry is trusted while offset and key match; when entries are erased, only the slots of the
objects that lost them forget their cached entries), `writeItems`,
`packedWrites`, `moves`, `contacts`. `syncStamp` / `stampedCount`: equal counts mean nothing to remove.
`isLive(entry)` = its object was visited by this sync and the entry carries the object's `liveStamp`; the entry keeps a
pointer to its object's record (`sync`), valid because records are erased only after their unvisited bodies are removed.

## Solver step (`solveObjectsSubclass`)

```
StepProgress (UTgetInterrupt)                  Escape checks; Houdini's own "Stepping simulation" bar stays
worldData = findOrCreateWorldData(objects)     first object carrying "Box3DWorld", else created on objects(0)
rebuild if invalid / thread count changed / stamp mismatch -> createFreshWorld
lock st->mutex
stepDt = dt * timescale; world.applyTuning(currentWorldSettings())
syncFromGeometry   per object syncObject(); remove gone bodies; pruneHulls; warnings
applyExternalForces  Use Force DOPs: SIM_Force data per object -> getForce at each dynamic body's centre of mass -> applyForceByHandle
world.step         b3World_Step(stepDt, subSteps); then the move events are read once into st.moves
writeResultsBack   awake dynamic bodies (move events): P / orient / v / w / packed transform / awake
recordImpacts      Add Impact Data -> SIM_Impacts (only on steps with an impact)
writePointAttrs    queued point writes: bullet_autofit_valid = 1, computed mass
ageObjects         Enable Aging
deleteDeadPoints   `dead` points
stamp = now
```

**Performance Monitor.** Each stage opens a `UT_PerfMonAutoSolveEvent(this, "Box3D: <stage>")` (as Bullet does), so
the profiler lists the stages under the solver node instead of one entry: Rebuild World, Sync Bodies, Sync Affectors,
Remove Bodies, Sync Constraints, Apply Forces, Step World (`world.step` + move events), Write Results, Constraint
Feedback, Record Impacts, Point Updates (point writes, aging, dead points). Events are opened only on the solve
thread; threaded passes count toward the stage that runs them. When the monitor is not recording, the constructor
only checks two flags. Box3D's own step breakdown (`b3World_GetProfile`) is not reported.

**Guide geometry** (`Box3DGuide.cpp`; the solver node's Guides tab: Show Collision Shapes, Shape Color). The solver
overrides `createGuideObjectSubclass` (a `SIM_GuidePerObject`: one guide per object it is attached to) and
`buildGuideGeometrySubclass`, which calls `buildCollisionGuide`. Guide parameters are `setGuideTemplates` on the DOP
description, read with `GET_GUIDE_FUNC_B` / `_V3`. The shapes come from the live world
(`Box3DWorld::bodyShapeOutlines`: hulls with their face loops, spheres, capsules, in body space), found through the
engine's objects (the `SIM_Box3DWorld` whose state has this object in `objectSync`); each body is placed at the pose
the object's geometry shows on the displayed frame (point `P` with its packed rotation, else `orient`; whole objects
from `readWholePose`), keyed as the sync keys it, so cached frames draw where the pieces were. Drawn as open
polylines with a primitive `Cd` (dimmed for sleeping and for static / kinematic bodies). The state's mutex is only
tried: a guide asked for while the solver steps is skipped. Bodies the world no longer has are not drawn; static
colliders from affectors are not drawn.

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
a `UT_ERROR_MESSAGE` "Box3D: step cancelled", and `SIM_SOLVER_FAIL`. `b3World_Step` itself is not interruptible, and
once it has run the step always finishes (write-back etc. are quick), so a cached step is always whole.
`StepProgress` is at global scope in `Box3DSolverInternal.h` (the solver header forward-declares it).

Data names: `"Box3DWorld"`, `kBox3DDataName` (`"Box3DData"`, SIM_Box3DData.h), `SIM_GEOMETRY_DATANAME`.

### syncObject (per object)

`syncObject` is the driver; the passes are helpers in `Box3DPointSync.cpp`: `collectSlots`, `readSlots` / `readSlot`,
`planSlots`, `reserveMirror`, `ApplyPass` (pass 2) and `NameIndex` (the `name` map for joint anchors). A cancel in
`planSlots` or `ApplyPass::run` makes `syncObject` return early.

1. Read-locks the `SIM_Geometry` (objects without one are skipped). `PointAttrs` binds `id, active, v, w, orient`.
2. Box3D Data: body type and `BodyProps` (its values that change existing bodies), plus Add Impact Data.
3. **Skip check** (see below). On a skip, the object's entries are counted and nothing per point runs.
4. Otherwise binds the remaining attributes and builds the `ScanContext` (world, geometry, attributes, node settings,
   shape sources, georep lookup) that the per-point helpers take.
5. **Pass 1** (`collectSlots`: serial offsets; `readSlots`: parallel above `kMinParallelReadPlain` points, `kMinParallelReadPacked` with primitives): per point body id (`id`, else the point number = its
   position in the walk), key, `getPackedInfo` (with the 3x3's row scale, kept for the write-back), `readPointState`,
   `active`, the mirror entry (read only), and whether it needs shapes built: new body, or a rebuild (`deforming`,
   `bullet_autofit_valid == 0`, convex / concave scale change).
   **Pass 1b** (`planSlots`; parallel, chunks of `kPlanChunk` with an Escape check between): `planShapes` for those points, into a per-scan
   plan list (`Box3DPointSlot::plan`). Hull building, the slow part of a rebuild, happens here; `ShapeSources`,
   `GeorepLookup` (mutexes) and Box3DWorld's hull calls are thread-safe. Pass 2 takes the plans (`takePlan`).
6. **Pass 2** (`ApplyPass`; serial, point order): duplicate `id` -> counted, first wins; non-finite `P` -> counted, its body removed;
   `dead` -> queued for deletion, its body removed; unknown key -> `addPointBody`; known key -> `updateExistingBody`:
   mid-simulation updates (shape rebuild, then settings), type change (`wantedType`), then the animated target or an
   external edit (`poseDiffers`, eps 1e-5, q and -q equal -> `setTransformByHandle` + `setVelocityByHandle`). Both
   paths then share one tail: point bookkeeping and stamp, queued point writes, the object's body list.
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

* Recorded at the end of a scan and again after each of our own writes (`recordFingerprintAfterWrite`, once every write
  lock is released, since releasing may bump ids). A debug log showing `full scan, changed: ...` every step means
  something else touches the geometry after the solver.
* `forceScan` keeps an object scanning after a write the scan must act on (`active = 1`, `dead = 1`, deleted points).
* Records of objects not seen are erased, so a returning object scans. A fresh world starts without records.
* Debug builds log `sync: N of M object(s) unchanged, scan skipped` when objects are skipped.

### writeResultsBack

`SIM_SolverBox3D::writeResultsBack` is the driver (it owns the node messages); the work is in `Box3DPostStep.cpp`:
`collectMovedBodies`, `writeWholeObjects`, `readVelocities`, then per object `writeObjectResults` (`BodyWriter` for one
body's attributes, `writePackedRotations` for step 5).

0. Move events (read once into `st.moves` right after the step; impacts use them too): every awake body and its
   transform (a body falling asleep appears once more). Only live dynamic entries become items, which point at their
   move event rather than copying it; `awake` changes are noted. Velocities are fetched in a threaded pass
   (Box3D allows read-only calls from several threads). Items are grouped per object.
1. `getOrMakeWritableGeometry`: a plain `SIM_Geometry` (RBD Packed Object) is replaced once by a `SIM_GeometryCopy`
   (`SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`).
2. `ensureResultAttribs` creates `v`, `w`, `orient` (a new `orient` is set to identity).
3. `resolvePointOffsets`: the stored offset if it still names the point (in range, same id / number), else by id.
4. Parallel pass: `hardenWrittenPages` first when parallel (offsets bucketed per page, see `hardenWrittenPages`; one full call when nearly all
   buckets are hit), then write, compute the new packed 3x3 (the row scale the sync stored, the new rotation: no matrix
   decomposition), and store in the mirror what the geometry now holds without reading it back (`storedPosition`,
   `sanitizeQuat` of the written rotation). It must agree with `readPointState` within `poseDiffers`' tolerances (1e-5;
   q and -q equal), or our write looks like an external edit; a packed transform's float round trip stays well inside
   them. `BOX3DDOP_VERIFY_READBACK=1` re-reads and logs `READBACK MISMATCH` when `poseDiffers` would see a difference.
5. Serial pass: `setLocalTransform` + `transformDirty`, then one `getPrimitiveList().bumpDataId()` (the HDK leaves
   marking intrinsics dirty to the caller). A point no longer on a packed primitive is counted as unmatched.

Only dynamic bodies are written; of several points sharing an `id`, only the first.

### Threading

Box3D starts no threads of its own in the plugin. `SIM_Box3DWorld::createWorld` gives every world a `Box3DHoudiniTasks` (`src/sync/Box3DHoudiniTasks.h`, a `b3dop::TaskRunner` over `UT_TaskGroup`), wired to Box3D's `enqueueTask` / `finishTask` hooks in `Box3DWorld::create`, so all worlds share Houdini's TBB pool. Box3D's built-in scheduler keeps `workerCount - 1` OS threads per world until it is destroyed, which piled up across the old worlds that cached frames keep alive. `autothreads` (off by default: Box3D's solver workers spin-wait between stages, so many workers rarely pay off below very
large scenes) = `UT_Thread::getNumProcessors()` (honours `HOUDINI_MAXTHREADS`), else `threads` (default `defaultWorkerCount()`, `src/core/Box3DCpu.h`: physical performance cores, no hyper-threads or efficiency cores, `kDefaultWorkerCount` if undetectable; range 1 to `kMaxWorkerCount`), never above Houdini's count; a change still rebuilds the world. Safe with Box3D's solver because tasks carry their own worker index, the stepping thread always runs solver worker 0 (which can finish the solve alone, so a busy pool cannot deadlock), and `step` runs inside `tbb::this_task_arena::isolate`. `UT_TaskGroup` does not carry `UT_TaskScope`, so the runner wraps each task in one. Without a runner (the tests) `Box3DWorld` falls back to Box3D's own pool. The plugin's
loops use `UTparallelFor` with untuned thresholds; parallel lambdas only read shared data or write their own element.
In `BOX3DDOP_DEBUG` builds the parallel thresholds (`kMinParallelPoints` and the others beside it in `Box3DSolverInternal.h`) can be overridden from the environment (`BOX3DDOP_MIN_PARALLEL_POINTS`, `_READ_PLAIN`, `_READ_PACKED`, `_WRITE`; a count, read once per process), to sweep them without rebuilding.
The Box3D world and mirror are touched in serial passes (the write-back's parallel pass writes only its own entry). The
shared-state mutex is held for the whole step.
