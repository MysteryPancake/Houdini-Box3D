# AGENTS.md

Technical notes for AI agents (and humans) working on this codebase: how the code works and why.
Build and usage instructions for users are in `README.md`.

## What this is

A Houdini DOP plugin (HDK, C++17) wrapping [Box3D](https://github.com/erincatto/box3d) (Erin Catto's 3D rigid
body engine, C17, a git submodule in `extern/box3d`). It registers these `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver: one call per timestep with all its objects |
| Box3D Data | `SIM_Box3DData` | Per-object defaults (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |
| Box3D <Type> Constraint Relationship | `SIM_Box3DConRel<Type>` | One per joint type: the joint's settings, named by `constraint_name` in a Constraint Network |

Design goals: Box3D's native performance (handle-based hot paths, no per-body hash lookups, threaded per-point passes
where safe) and Houdini Bullet behaviour (same attribute names and rules, as documented on the RBD Packed Object page).
The exception is sleeping and enabling, which follow Box3D's own model (`enablesleep`, `awake`, `enabled`; see "Sleep and
enable state"); Bullet's `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read.

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism.
Everything not yet tested or verified is listed in one place: "Untested and unverified"; everything still to do in
another: "Known gaps / TODO". Keep them there and nowhere else (no TODO or "unverified" notes in code comments, the
README or other sections here).

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short. Leave out anything a user cannot act on or would
not notice: how the solver does something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics. Those belong here (unverified details go in "Untested and unverified"). When a change adds a feature, add one line or table
row to the README if users need it, and put the explanation here.

README rules (hard limits, not suggestions):
- Never add more than one or two sentences, or one table row, per change. Do not grow a section to explain a feature.
- No implementation details: no function, class or HDK names, locks, fingerprints, step ordering, clearing or caching
  behaviour. If a sentence says how the code works, it goes in this file.
- Do not rewrite or expand existing README text while making an unrelated change. Trim, don't pad.
- Before finishing a task, re-read any README edit and cut it down.

UI tooltips (the help text on parameters) follow the same rule: one or two short sentences, under about 150
characters. Say what the control does and, if needed, when to change it. No implementation detail, no lists of point
attributes, no restating how the sibling controls work.

## Layout

```
CMakeLists.txt        Targets: b3dop_core, the core test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
extern/box3d/         Box3D submodule (never edit; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg     Node icon; the build copies it to config/Icons as DOP_<node name>.svg for every node
src/                  The include root: includes are written "core/Box3DWorld.h" (bare name within a folder)
  core/               Houdini-free; built as b3dop_core and covered by the tests
    Box3DShapes.h       Header-only: shape math, packed-prim matrices, hashing
    Box3DJoints.h       Header-only: joint types, JointSettings, the joint parameter table
    Box3DSideFX.h/.cpp  SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data -> Box3D joints (mapping, frame offsets)
    Box3DRbdObject.h    Header-only: an RBD Object's own mass / friction / bounce / Bullet Data shape -> whole-object settings
    Box3DWorld.h/.cpp   Pimpl wrapper over one b3World. The ONLY file that includes Box3D headers; defines TaskRunner
    Box3DAffectorMath.h Header-only: affector (Static Object) transform math
    Box3DDebug.h        B3D_LOG / counters, compiled out unless BOX3DDOP_DEBUG (args still type-checked, never evaluated)
  dop/                Houdini SIM_Data classes: the nodes and the world cache
    SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache)
    SIM_Box3DData.*     Default-settings DOP node
    SIM_Box3DConRel.*   Box3D <Type> Constraint Relationship DOP nodes (one per joint type), parameters from the table
    SIM_Box3DParms.h    Box3DParms: builds a node's parameter templates (tabs, ranges, menus) from a table
    SIM_SolverBox3D.*   The solver node: parameters, node messages and the solve loop (the parts are in sync/)
    SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
  sync/               Houdini data -> the Box3D world, used by the solver
    Box3DSolverInternal.h  What the solver's parts share (namespace box3dsolver); only the solver's files include it
    Box3DShapeBuild.cpp    Packed primitives, collision source geometry (pieces, summary cache), buildShapes
    Box3DPointSync.cpp     One body per point: point state, fingerprint, body settings, shape plans, syncObject
    Box3DWholeObject.cpp   One body per DOP object (RBD Object etc.): sync and Position write-back
    Box3DPostStep.cpp      After the step: writeResultsBack, Impacts, point writes, aging, dead points
    Box3DForces.cpp        Force DOPs and point force / torque
    Box3DJointSync.*    Constraint networks -> Box3D joints: read, resolve names, create / update / remove, break, feedback
    Box3DAffectors.*    Affector objects (Static Object DOP etc.) -> static colliders
    Box3DHoudiniUtil.h  Shared Houdini helpers: writable geometry, object by id, 4x4 to array, Euler to quaternion,
                        options of SIM data by name (box3dDataOption)
    Box3DHoudiniTasks.h TaskRunner on Houdini's TBB pool (UT_TaskGroup). Only SIM_Box3DWorld.cpp includes it
tests/
  KeyedWorld.h        Test-only Box3DWorld subclass: bodies by caller-chosen id, over the handle API
  core_smoke.cpp      Version print + world create/destroy
  core_physics.cpp    Unit + physics checks (CHECK macro, nonzero exit on failure)
  core_threads.cpp    1 worker vs N workers: timing + determinism
  core_taskrunner.cpp Host-supplied TaskRunner shared by 40+ worlds: no extra threads, same results as 1 worker
  core_joints.cpp     Every joint type, world anchors, live updates, breaking, feedback, the parameter table
  core_sidefx.cpp     SideFX constraint data: option table, mapping per type / constraint_type, frame offsets, behaviour
  core_rbdobject.cpp  RBD Object settings without Box3D Data: defaults, invalid values, Bullet Data shapes, density -> mass
  box3d_test.hiplc    Houdini scene for manual testing (tests/backup/ holds older saves)
```

Layering (keep it):

1. Everything in `src/core/` knows nothing about Houdini, so the tests run without Houdini.
2. `SIM_*` files never include Box3D headers; they use `b3dop::Box3DWorld`.
3. `box3d` is linked `PRIVATE` into `b3dop_core`, so its defines (notably `BOX3D_DOUBLE_PRECISION`) cannot leak into
   plugin code.

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the tests, then `return()`s before any HDK use.
* Otherwise `find_package(Houdini)` (with `$HFS/toolkit/cmake`) and `houdini_configure_target`, which also copies the
  plugin into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows). Close Houdini to overwrite it on Windows.
* A post-build step copies `icons/box3d.svg` to `<dso>/../config/Icons/DOP_<node name>.svg` (Houdini's default icon
  name for a node type is `<TABLE>_<name>`; the HDK alternative, `OP_Operator::setIconName`, needs the operator, which
  the automatic DOPs don't expose to us). Renaming a node's internal name means renaming its icon.
* `BOX3DDOP_DEBUG` (default OFF) defines `BOX3DDOP_DEBUG=1` for the plugin and `b3dop_core` (not Box3D). With it OFF nothing writes to stdout/stderr: `Box3DWorld` replaces Box3D's printf log handler and its own assert handler with `B3D_LOG`. The option is cached in `build/`, so a stale ON survives `build.bat`; reconfigure with `-DBOX3DDOP_DEBUG=OFF` or delete `build/`.
* `BOX3D_DIR` (default `extern/box3d`) must point at Box3D's own `CMakeLists.txt`.
* `BOX3DDOP_MAX_WORLDS` (default 1024; Box3D's own default is 128) is passed to the `box3d` target as PUBLIC
  `B3_MAX_WORLDS` (Box3D wraps it in `#ifndef`). Each slot is a ~5 KB static `b3World` (5008 bytes measured).
  Rebuilds keep the previous world alive while cached frames share it, so worlds accumulate.
  `Box3DWorld::create` checks `b3GetWorldCount() >= B3_MAX_WORLDS` itself before `b3CreateWorld`, which would
  otherwise hit a `B3_ASSERT` (on in RelWithDebInfo) before failing. A failed rebuild returns false from
  `createFreshWorld` and the solve fails (the old state is valid but from another time and must not be continued);
  `reportWorldCreationFailure` names the limit when it is the cause.
* `-DBOX3D_DOUBLE_PRECISION=ON` works: positions cross the boundary as `double[3]` and `Box3DWorld.cpp` converts through
  `decltype(b3Pos::x)`.
* Release or RelWithDebInfo only. RelWithDebInfo turns on `B3_ENABLE_ASSERT`. Box3D's default assert handler breaks
  to the debugger (kills Houdini without one), so `Box3DWorld::create` installs `assertHandler` once
  (`b3SetAssertFcn`): logs to stderr (first 20), counts, returns 0 (no break). Box3D then carries on past the failed
  check, so results after an assert may be wrong, but Houdini survives.
* Inputs Box3D asserts on are guarded in the core too: velocities (`finiteVec3`: non-finite components -> 0) and
  gravity (`create`: non-finite components -> 0; `applyTuning`: non-finite gravity ignored, previous kept).
* `build.sh` wipes `build/` if the repo moved (`build/.configured_from`). `build.bat` finds `HFS` itself (env var, then
  registry `HKLM\SOFTWARE\Side Effects Software` ActiveVersion -> `Houdini <ver>\InstallPath`, then the newest
  `Program Files\Side Effects Software\Houdini X.Y.Z`). It still names the VS 2022 generator.

## Core layer: `b3dop::Box3DWorld`

**Conventions.** Positions `double[3]`; rotations `float[4]` x,y,z,w (the layout of `b3Quat` and `UT_QuaternionF`);
velocities `float[3]`. `BodyHandle` = a `b3BodyId` packed into `uint64_t` (never 0 when valid). Never use a stale
handle; the solver keeps handles in the mirror and erases entries with their bodies.

**Handles only.** `createBody`, `createShape` / `createHullShape` (taking the caller's `BodySettings`), `destroyBody`,
`setTypeByHandle` / `typeByHandle` and the `*ByHandle` calls touch no hash map. Caller-chosen ids (`addBody`,
`addShape`, `removeBody`, ...) live in `tests/KeyedWorld.h`, a test-only subclass built on the public handle API.

**Mass.** A shape add recomputes the mass unless `updateMass` is false. The solver always passes false and then calls
`applyMassByHandle` once (required before the next step): it sets every shape's density, applies the mass from the
shapes and, for dynamic bodies, can move the centre of mass to the body origin (parallel axis theorem) and scale the
inertia (`b3Body_SetMassData`; skipped when already centred with scale 1). `setTypeByHandle` recomputes from the shapes,
so the caller re-applies after a type change.

**Changes to existing bodies.** `setMaterialByHandle` (every shape; Box3D re-mixes contact friction / restitution every
step, so touching contacts follow), `setDampingByHandle`, `setGravityScaleByHandle`, `setSleepEnabledByHandle` (the
world switch applies on top; disabling wakes the body), `getCenterOfMassByHandle` / `applyForceByHandle` (external forces, per step, see below), `setSleepThresholdByHandle` (Box3D's single threshold on
`|v| + |w * maxExtent|`; negative = default), `setBulletByHandle`, `setDensityByHandle`, `setEnabledByHandle`
(`b3Body_Disable`: no motion, no collisions, shapes kept), `destroyShapesByHandle` (releases the body's hulls; the
caller re-adds shapes and applies the mass), `wakeByHandle`, `setTargetTransformByHandle` (kinematic: Box3D sets the
velocity that reaches the pose in `timeStep`; it remains until cleared) and `getContactsByHandle` (one `ContactPoint`
per manifold point with `totalNormalImpulse > 0`; position = A's world centre of mass + `anchorA`; normal flipped to
point from this body into the other). They share a `thread_local` shape-id scratch and are called serially. Box3D
accepts all of them on a disabled body (proxies are only touched when they exist).

**World settings.** `WorldSettings` (Box3D's defaults, except standard gravity): gravity, `subSteps`, `workerCount`,
`enableSleep`, `enableContinuous`, `contactStiffness` (Hz), `contactDampingRatio`, `contactPushSpeed`,
`restitutionThreshold`, `maxLinearSpeed`. `applyTuning` pushes changes into the live world (`b3World_SetGravity`,
`EnableSleeping`, `EnableContinuous`, `SetContactTuning`, `SetRestitutionThreshold`,
`SetMaximumLinearSpeed`); `workerCount` (parallel tasks per step) is fixed at creation and needs a new world.

**Collision filtering.** Box3D's own `b3Filter`, nothing custom. `CollisionFilter` (`Box3DWorld.h`: category, mask,
group; defaults every bit / every bit / 0, Box3D's `B3_DEFAULT_*_BITS`) lives in `BodySettings::filter`; `shapeDefFor`
puts it on every shape (`toB3Filter`) and Box3D checks pairs inline in its broad phase. `setFilterByHandle` calls
`b3Shape_SetFilter(..., invokeContacts = true)` on each shape, which resets the proxy (dropping contacts that no longer
pass) and is a no-op for an unchanged filter. No custom filter callback, no shape user data.

**Shapes and hull cache.** A box is a Box3D hull (`b3MakeBoxHull` / `b3MakeOffsetBoxHull`; a rotated box is the hull of
its 8 corners); sphere and capsule (along Y; Box3D turns a too-short one into a sphere) are native. `createHull` keys by
content (`hashFloats` of the points; hashing mixes 8 bytes per round); packed instances pass their own key (`findHull`
first, build on a miss). Builds try vertex budgets `{B3_MAX_HULL_VERTICES, 44, 32, 16, 8}`: a triangulated hull hits the
256 half-edge limit at ~44 vertices, so a 64 rung fails whenever the maximum did (measured: on dense round input ~75% of
hull time went to the two failing attempts). `createHullWithKey`'s `budgetGroup` (the solver passes the source hash and
kind) remembers the budget that worked, so later hulls of the group start there. Flat or non-finite results are rejected
and the caller falls back to a box. Several hulls on one body make a dynamic compound; overlapping pieces count their
overlap twice in the mass.

The hull cache is **process-wide** (`HullLibrary` in Box3DWorld.cpp, mutex-protected): Box3D copies a hull into the
world's own database (`b3AddHullToDatabase`) when a shape is made, so our entries are only needed until then and can be
shared by every world. A rebuilt world (scrub, re-solve, thread change) therefore finds every hull its predecessor built.
Entries are `shared_ptr`s; a world holds the ones it found or built (`Impl::held`) until `pruneHulls()`, which the solver
calls at the end of each sync, so eviction never frees a hull between `findHull` and `createHullShape`. Past the byte
budget (`setSharedHullBudget`, default 256 MB) the least recently used entries are evicted down to 3/4 of it.
`findHull`, `createHull` and `createHullWithKey` are thread-safe (the solver's shape pre-pass); nothing else is.
`hullCacheSize()` is the hulls a world holds; `sharedHullCount/Bytes`, `hullBuildCount/Attempts` report on the cache.

Packed sources also get a **source summary** (`SourceSummaryCache` in Box3DShapeBuild.cpp, process-wide): the group and
piece hashes, keyed by the source detail's identity (`getUniqueId`, `P` and primitive-list data ids, counts, fragment,
connected or not). `buildShapes` tries it first for packed convex / concave points: when every hull key it gives is still
cached, the source geometry is not read or hashed at all. This relies on whoever edits a source bumping its data ids.

**`Box3DShapes.h`** (all unit-tested): `parseShapeType` (exact lowercase Houdini names; `compound`
= `concave`; unknown values, including `cylinder` and `plane`, return false; the solver memoises per shared-string handle
in `GeorepLookup`); `makeShapeSpec` (`pscale * scale` as a bounding box, components made positive and >= `kMinExtent`
= 1e-4 by `clampExtent`; sphere diameter = largest component; capsule radius = half of
max(x, z)); `PrimOverrides` / `applyPrimOverrides`; `matrixRowLengths`; `packedPointsToBodyLocal` (`(p - pivot) * S`);
`packedScale` (row lengths rounded to 5 significant digits, so rotated instances share a hull; ~1e-5 scale error);
`packedHullKey` (kind 1 merged, 2 per piece); `connectedSets` (union-find); `fitShapeToPoints`; `sanitizeQuat` /
`storedPosition` (shared by the read path and the write-back's prediction).

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

**Escape.** `StepProgress::check()` calls `UTgetInterrupt()->opInterrupt()` with no percent and no `opStart` of our
own (SideFX's `SIM_VolumetricConnectedComponentBuilder` does the same), so Houdini's own "Stepping simulation" message
and bar are left alone. SIM_Engine discards and re-cooks an interrupted step itself (`setSimulationTime`,
`myLastCookInterrupted`); the stamp check makes that re-cook rebuild the world. `check()` is called at the
start of the solve, before each object, every `kProgressEvery` (64) points in `syncObject` pass 2 (serial, between
points, so no half-built body), and once before the step. A cancel latches. Cancelled after the sync began:
`st->stamped = false` (the half-synced world can never be continued; the next solve that would continue it rebuilds),
a `UT_ERROR_MESSAGE` "Box3D: step cancelled", and `SIM_SOLVER_FAIL`. `b3World_Step` itself is not interruptible, and
once it has run the step always finishes (write-back etc. are quick), so a cached step is always whole.
`StepProgress` is at global scope in `Box3DSolverInternal.h` (the solver header forward-declares it).

Data names: `"Box3DWorld"`, `kBox3DDataName` (`"Box3DData"`, SIM_Box3DData.h), `SIM_GEOMETRY_DATANAME`.

### syncObject (per object)

1. Read-locks the `SIM_Geometry` (objects without one are skipped). `PointAttrs` binds `id, active, v, w, orient`.
2. Box3D Data: body type and `nodeHash` (its values that change existing bodies, Add Impact Data included).
3. **Skip check** (see below). On a skip, the object's entries are counted and nothing per point runs.
4. Otherwise binds the remaining attributes and builds the `ScanContext` (world, geometry, attributes, node settings,
   shape sources, georep lookup) that the per-point helpers take.
5. **Pass 1** (parallel above 8192 points, 2048 with primitives): per point body id (`id`, else the point number = its
   position in the walk), key, `getPackedInfo` (with the 3x3's row scale, kept for the write-back), `readPointState`,
   `active`, the mirror entry (read only), and whether it needs shapes built: new body, or a rebuild (`deforming`,
   `bullet_autofit_valid == 0`, convex / concave scale change).
   **Pass 1b** (parallel, chunks of 256 with an Escape check between): `planShapes` for those points, into a per-scan
   plan list (`Box3DPointSlot::plan`). Hull building, the slow part of a rebuild, happens here; `ShapeSources`,
   `GeorepLookup` (mutexes) and Box3DWorld's hull calls are thread-safe. Pass 2 takes the plans (`takePlan`).
6. **Pass 2** (serial, point order): duplicate `id` -> counted, first wins; non-finite `P` -> counted, its body removed;
   `dead` -> queued for deletion, its body removed; unknown key -> `addPointBody`; known key -> `updateExistingBody`:
   mid-simulation updates (shape rebuild, then settings), type change (`wantedType`), then the animated target or an
   external edit (`poseDiffers`, eps 1e-5, q and -q equal -> `setTransformByHandle` + `setVelocityByHandle`). Both
   paths then share one tail: point bookkeeping and stamp, queued point writes, the object's body list.
7. Records the fingerprint; problems (failed bodies, duplicate ids, bad P), deforming bodies and driven animated bodies
   keep the object scanning.

### Skipping the sync scan

`Box3DObjectSync::fp` (`Box3DSyncFingerprint`): point count and offset range (plain points only; packed topology is
assumed fixed, a new instance gets a new `id`), data ids of `P`, `orient`, `v`, `w`, `id`, `active`, the node body type,
and the **prop markers**: data ids of `kPropAttribNames` (31 attributes that change settings, ask for rebuilds or rename pieces, looked
up by name), the primitive list's data id (`primitiveListDataId`: packed transforms are intrinsics; bumping it is up to
whoever edits a primitive, so an edit that does not is picked up on the next scan) and `nodeHash`. Equal fingerprint ->
skip.

* Recorded at the end of a scan and again after each of our own writes (`recordFingerprintAfterWrite`, once every write
  lock is released, since releasing may bump ids). A debug log showing `full scan, changed: ...` every step means
  something else touches the geometry after the solver.
* `forceScan` keeps an object scanning after a write the scan must act on (`active = 1`, `dead = 1`, deleted points).
* Records of objects not seen are erased, so a returning object scans. A fresh world starts without records.
* Debug builds log `sync: N of M object(s) unchanged, scan skipped` when objects are skipped.

### External forces (`applyExternalForces`)

Runs after `syncFromGeometry` and the last cancel check, before the step, when `useforces` is on. Same lookup as
Houdini's `SIM_SolverHair`: `obj->forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"), SIM_FORCES_DATANAME,
SIM_DataFilterNone())` with a `SIM_EachDataCallback` (`ForceCollector`) that casts each sub-data to `SIM_Force`.

Only objects with force DOPs or non-zero point forces are visited, and only their own bodies
(`Box3DObjectSync::bodies`, rebuilt by each full scan and kept while the object is skipped): skip bodies that are not
`dynamic()`, are not `props.enabled`, or are asleep while `forceswake` is off. For each remaining body: mass (`massByHandle`), world centre of mass
(`getCenterOfMassByHandle`) and `v` / `w` (`getVelocityByHandle`) go into `SIM_Force::getForce(object, pos, vel, angvel,
mass, force, torque)` for every force on the object, summed, and the sum goes in through `applyForceByHandle`
(`b3Body_ApplyForceToCenter` + `b3Body_ApplyTorque`).

* Box3D adds `invMass * force` and `invInertiaWorld * torque` in every substep's velocity integration
  (`b3IntegrateVelocitiesTask`) and zeroes both when the step finishes, so they are per-step: never carry them over, and
  a rebuilt world has none. `Box3DWorld` does not track them.
* Box3D ignores a force on a body that is not in the awake set. `wake = false` (the default) therefore skips sleeping
  bodies, which is what keeps gravity-like forces from holding resting bodies awake; `forceswake` passes `wake = true`.
* Non-finite sums are skipped and counted (`ForceStats::nonFinite`, one node warning). `applyForceByHandle` also
  sanitizes (`finiteVec3`) because Box3D asserts on non-finite vectors.
* Evaluation is serial.
* Gravity Force DOP + the solver's `gravity` add up (both are applied). `gravityscale` only scales the solver's gravity.
* Not used: `getForceOnCircle` / `getForceOnSphere` (area and volume variants), `getPointForces`.
* `useforces` / `forceswake` are plain options read each step; nothing is stored in the world, so they need no rebuild.

### Mid-simulation updates

Target behaviour is Houdini's Bullet solver. A new body is built from the same readers, so an untouched point compares
equal to what its body was built with.

* **Settings** (`readDynamicProps` -> `applyPropChanges`): the effective (attribute, else Box3D Data) values in
  `Box3DBodyProps`, compared with `entry->props`; only the setters for what differs run. Compared only when `checkProps`
  (prop markers changed, none recorded yet, or deforming bodies); a scan triggered by an animated `P` alone does not.
  * Mass, `computemass`, `density`, `inertialtensorstiffness` -> `applyBodyMass` (same as a fresh build).
  * Sleep and enable state: `enablesleep` (else the node) -> `setSleepEnabledByHandle`, live; `enabled` ->
    `setEnabledByHandle`. `awake` is not a setting: see "Sleep and enable state".
* **Shape rebuild** (`rebuildPointShapes`) on `deforming` (every step), `bullet_autofit_valid == 0` (once) or, with
  `autoScale` (convex / concave only, as in Houdini), a size-key change beyond 1e-4 relative (`sizeKey` = packed row
  lengths or `pscale * scale`; the tolerance absorbs our own write-back's float noise). `planShapes` (the same code as a
  new body) gives a `shapeSignature` (hull keys + primitive numbers); equal -> nothing touched. Otherwise
  `destroyShapesByHandle`, `addShapes`, `applyBodyMass` (even with no shapes, to clear Box3D's dirty mass),
  `wakeByHandle`. The body keeps its handle, pose, velocity and identity. A plan of `none`, or shapes that all fail,
  leaves the body `shapeless`, hence static.
* Editing `bullet_georep`, `bullet_autofit`, `bullet_prim*`, `bullet_radius`, `bullet_length` or a primitive's size
  alone does nothing (as in Bullet). Box3D Data `defaultshape` / `autofit` / `hullperconnected` apply at creation or
  rebuild.
* **`bullet_autofit_valid`**: every point built or rebuilt while it was 0 gets 1 written after the step. Unlike Bullet,
  fitted values are not written into `bullet_prim*` / `bullet_radius` / `bullet_length`.
* **`id`** is the identity key with no special values: a different `id` is a different body. The `id = -1` "force a
  rebuild" trick some notes describe is deliberately not implemented.

### Bullet features

* **The point is the centre of mass**, as in Houdini RBD (for a packed point, its pivot, which hulls are built around).
  `applyBodyMass` calls `applyMassByHandle(..., comAtOrigin = true, stiffness)` after creation, rebuild, a retype to
  dynamic and mass changes. Velocities are therefore the point's. No `computecom`: an off-centre pivot makes
  the piece turn about it, like in Houdini.
* **CCD threshold** (`ccdthreshold`, float; node `ccdthreshold`, default 0.5 = Box3D's): `BodySettings::ccdThreshold`
  -> `b3BodyDef::safetyFactor` (negative / non-finite keep Box3D's default; Box3D asserts on negatives); changes use
  `setCcdThresholdByHandle` (`b3Body_SetSafetyFactor`). A body is "fast" (swept, contacts not recycled) when a step's
  motion > threshold x its min extent. Renamed from Box3D's "safety factor" so the name says what it does.
* **Fast rotation** (`fastrotation`, int): `BodySettings::fastRotation` -> `b3BodyDef::allowFastRotation`; changes use
  `setFastRotationByHandle` (`b3Body_AllowFastRotation`). Without it Box3D caps angular speed at `B3_MAX_ROTATION`
  (pi/4) per Box3D step, i.e. per DOP step here (the solver's Substeps are inside one step and don't raise it).
  No node default: Box3D says round bodies only.
* **Motion locks** (`lock_linear`, `lock_angular`: vec3, nonzero component = locked world axis): `readDynamicProps`
  fills `Box3DBodyProps::locks` -> `BodySettings::locks` -> `b3BodyDef::motionLocks`; changes use
  `setMotionLocksByHandle` (`b3Body_SetMotionLocks`, which zeroes the newly locked velocity components). Box3D zeroes
  locked components of `v` / `w` every iteration, so locked bodies also ignore `v` / `w` and forces on those axes.
* **Collision filtering** (`collision_category`, `collision_mask` read as `GA_ROHandleID` so a 32-bit `-1` sign-extends
  to every bit; `collision_group` int): `readDynamicProps` fills `Box3DBodyProps::filter`, which goes into
  `BodySettings::filter`; changes use `setFilterByHandle`. Missing attributes keep Box3D's defaults. Affector bodies use
  the defaults.
* **Animated** (`animated` with `active` = 0): Kinematic. A pose edit becomes `setTargetTransformByHandle(cur, stepDt)`
  with `driven = true`; a step without one zeroes a driven body's velocity (so its object never skips while driven). Not
  written back; `v` / `w` are not applied.
* **Mass properties** (`computemass`, `density`, `inertialtensorstiffness`; node `computemass` / `density`):
  `boxDensity` = density with computemass, else mass / volume. With computemass the mass is written to `mass` (created
  if missing) on creation, rebuild or change.
* **Sleep threshold** (`sleepthreshold`, node default 0.05): Box3D's single threshold and fixed half second
  (`B3_TIME_TO_SLEEP`).
* **Sleep and enable state** (`enablesleep`, `awake`, `enabled`): one attribute per Box3D concept
  (`b3BodyDef::enableSleep` / `isAwake` / `isEnabled`).
  * `enablesleep` (node: Allow Sleeping, in `nodeSettingsHash`) and `enabled` are settings (`Box3DBodyProps`) and change
    live. `enabled` is `def.isEnabled` at creation (no create-then-disable); toggling later is `b3Body_Enable` /
    `Disable`, which Box3D documents as expensive. A disabled body has no move events, so it is not written back.
  * `awake` is state, not a setting. At creation it is `def.isAwake` (the point's value, else the node's Start Asleep).
    Box3D starts a body that may not sleep awake regardless. The initial `v` / `w` are skipped for a body that is not
    awake after creation, because `b3Body_SetLinearVelocity` wakes a sleeping body.
  * `Box3DBodyMirror::awake` is the value of the attribute as the solver last read or wrote it, not the live Box3D state.
    The write-back compares it with each move event (`!fellAsleep`) and writes the attribute, if it exists, only when
    they differ. A `checkProps` scan (`awake` is a prop marker) compares the attribute with the mirror: a difference is a
    user write, and applies `wakeByHandle` or `sleepByHandle`, then updates the mirror. Edge-triggered on purpose: a
    level check would keep retrying a sleep Box3D refused, or wake a sleeping pile on every unrelated prop change.
    `p.enabled == false` skips the check; enabling sets the mirror to awake (Box3D enables a body awake), so an `awake`
    of 0 written with it puts the body to sleep right after.
  * `sleepByHandle` does nothing for static or disabled bodies, a body with sleeping off, or when the world's sleeping is
    off. `b3Body_SetAwake(false)` sleeps the whole island and Box3D may refuse (a pending island split), in which case
    the next move event writes `awake = 1` back.
  * Known limitation: forcing a body to sleep also sleeps the bodies touching it, but Box3D reports that through no move
    event (forced sleep between steps modifies the already-consumed event array), so their `awake` attributes and mirrors
    keep saying awake until they wake and fall asleep normally. A shape rebuild wakes the body (`wakeByHandle`).
* **Fast object** (`fastobject`): Box3D's `isBullet`, renamed because Houdini's `isbullet` means "treat as a Bullet
  object". Changes mid-simulation.
* **Contact recycling** (`contactrecycling`, Box3D Data toggle default on, int point attribute): Box3D's per-body
  `enableContactRecycling` (`BodySettings::contactRecycling` -> `b3BodyDef`). The distance is world-wide:
  `WorldSettings::contactRecycleDistance` (solver `contactrecycledistance`, default 0.05 = Box3D's `10 * B3_LINEAR_SLOP`
  at its default length units; 0 = off). It is not in `b3WorldDef`, so `create` sets it after `b3CreateWorld`, and
  `applyTuning` pushes changes through `b3World_SetContactRecycleDistance` (non-finite: ignored, previous kept, like
  gravity; negative acts as 0). A pair recycles only if both bodies have the flag (and both shapes keep Box3D's speculative flag, always on here)
  (`contact.c`). Box3D copies the body flag into each contact's `b3_contactRecycleFlag` when the contact is created, and
  `b3Body_EnableContactRecycling` only changes the body, so existing contacts keep the old behaviour.
  `setContactRecyclingByHandle` therefore also rebuilds them for an awake dynamic body: `b3Body_Disable` + `b3Body_Enable`
  destroys and recreates its contacts, but zeroes velocity and wakes, so the velocity is saved and restored. Sleeping,
  static and kinematic bodies are not cycled (a cycle would wake them): they take it for new contacts only. A point-attribute
  or node change goes through the usual prop path (`kPropAttribNames`, `nodeHash`, `applyPropChanges`).
  `Box3DWorld::recycledContactCount()` (`b3World_GetCounters`) feeds the tests and a verbose log line.
* **Rolling resistance and conveyor velocity** (`rollingresistance` float, `tangentvelocity` vector; Box3D Data
  `rollingresistance` / `tangentvelocity`): Box3D's `b3SurfaceMaterial::rollingResistance` and `tangentVelocity`, set in
  `shapeDefFor` (`BodySettings::rollingResistance`, `tangentVelocity[3]`; negative or non-finite values become 0, since
  Box3D asserts on them). Rolling resistance is the larger of the two shapes' values times the larger radius (hulls use
  `0.25 * innerRadius`, `contact.c`), capped by the normal impulse. Tangent velocity is local to the shape, rotated by the
  body each step and projected onto the contact surface; the friction row drives the relative tangent velocity to it
  (the body on the belt moves with `+tangentVelocity`), so friction must be above 0. Meshes use material 0 (the wrapper
  builds them without per-triangle materials). Live changes: `setSurfaceByHandle` read-modify-writes
  `b3Shape_SetSurfaceMaterial` on every non-compound shape (friction and bounce are untouched). That alone is not
  enough: Box3D skips the material update for a *recycled* contact (`physics_world.c`, "skips updating ... material
  parameters") and a sleeping body does not update its contacts, so a resting box on a belt that is switched on would stay
  put. On a real change `setSurfaceByHandle` therefore calls `rebuildContacts` (`b3Body_Disable` + `Enable`, which
  destroys the body's contacts, wakes everything touching it and re-creates its pairs with `forcePairCreation`; works for
  static bodies; velocity restored for the others, shared with `setContactRecyclingByHandle`). A sleeping dynamic body is
  not cycled; it takes the change when it wakes. Affector (Static Object) meshes read the values from their Box3D Data
  node but keep them out of the shape signature, so an animated belt speed updates the shapes
  (`Box3DAffectorBody::rolling` / `tangent`) instead of rebuilding the mesh. Point path: `kPropAttribNames`,
  `nodeSettingsHash`, `readDynamicProps`, `applyPropChanges`. Tests: "surface material" block in `core_physics.cpp`.
* **Impacts** (node `addimpact`, Add Impact Data; no attribute): `recordImpacts` walks this step's awake bodies (the
  move events), reads each one's contacts once and records them for both bodies whose object has the option: the awake
  body itself, and the other one when it is not awake (an awake one records its own) and not an asleep dynamic body. So
  sleeping bodies cost nothing and contacts where neither body is awake are skipped. Only when there is at least one does it create a fresh `SIM_Impacts`
  (`SIM_DATA_CREATE` with flags 0 replaces existing data) and call `addPointImpact(pos, normal, impulse, ptnum, otherobjid, otherptnum, -1, 0, 0,
  0, time, 0)` per point (normal of the surface hit, pointing at the body that gets the impact; ptnum = its `pointIndex`). An object
  with Impacts last step and none now loses the data (`removeNamedSubData`, `impactObjects`). No Feedback data (that is
  for mutual interaction with other solvers).
* **Dead** (`dead`): queued as a `Box3DDeletion`; `deleteDeadPoints` checks every queued point (offset, `id` or number)
  before deleting any (`destroyPrimitive(prim, true)` for packed, else `destroyPointOffset(off,
  GA_DESTROY_DEGENERATE)`), then `bumpAllDataIds()` and `forceScan`. Without `id`, later points become new bodies.
* **Aging** (solver Enable Aging): `age += dt` (DOP step, not time-scaled) on every point, `dead = 1` past a positive
  `life` (attributes created if missing). Writes every point every step while on; in parallel past 2048 points, with
  `age` / `dead` pages hardened first.
* **Time Scale**: Box3D steps by `stepDt = dt * timescale`; animated targets use it; `canContinue` uses the DOP step.

**Point writes after the step** (`Box3DPointWrite`: `kAutofitValid`, `kMass`): grouped per object
(`forEachObjectRun`), offsets via the templated `resolvePointOffsets`, then `recordFingerprintAfterWrite`.

### Building shapes (`addPointBody`, `planShapes`, `buildShapes`)

* Type: `wantedType` (active -> node type; inactive -> static, or kinematic if animated; shapeless -> static).
* `readDynamicProps` + `settingsFor` give the settings; `planShapes` the shape type (`bullet_georep`, else node
  `defaultShape`) and pieces; `createBodyWithShapes` (shared with whole objects) makes the body, adds them
  (`addShapes`), `applyBodyMass`, then the initial `v` / `w` (after the mass, because a moving centre of mass shifts the
  velocity; not for static or animated bodies, nor for one that is not awake).
* The mirror entry is inserted (`try_emplace`) before the body exists and becomes its user data; it is erased again if
  the body cannot be built. A body is never left without shapes, except `none`. 64+ new bodies in one scan reserve the
  mirror once.
* `buildShapes`:
  1. Packed point: its geometry into `PieceGroup`s via `ShapeSources` (cached per source detail, and per
     `(source, first primitive)` for packed fragments, for one `syncObject`).
  2. Packed convex / concave: `pieceFromPoints` with a `packedHullKey` (by content if the transform is degenerate):
     `findHull` first, body-local points only on a miss, else a fitted box with a note. Convex merges all pieces
     (kind 1) unless hull-per-connected; concave is one hull per piece (kind 2).
  3. Box / sphere / capsule on a packed point with autofit: fitted to the bounds.
  4. Nothing usable (plain point, autofit off, empty geometry): `makeShapeSpec(pscale * scale)` plus
     `applyPrimOverrides`; convex / concave fall back to a box with a note.
* Notes are aggregated into one node warning per kind, on the step a shape is built.

**Explicit primitives** (Bullet Data with AutoFit off; `bullet_autofit` attribute, else the node toggle, default on):
for `box`, `sphere`, `capsule` only. `bullet_primS` = box HALF extents (Bullet Data documents Box Size as a half-size);
`bullet_radius`; `bullet_length` = capsule straight section, caps not included; `bullet_primT` = offset;
`bullet_primR` = Euler degrees, XYZ, turned into a quaternion with `UT_QuaternionF(rad, UT_XformOrder(TRS, XYZ))`. A
rotated box is an 8-corner hull; a sphere ignores rotation. The rotation order and the length's meaning live only in
that conversion and the `hasLength` line of `applyPrimOverrides`.

**Pieces and packed geometry.** Pieces are grouped by the primitive `name` attribute (unnamed share one; no primitives:
one piece). Collectors gather point offsets; `finishPieces` sorts and dedupes them, reads the positions (offset order)
and hashes each piece and the group. `hullperconnected` (convex only): one hull per connected set, ignoring `name`. Packed fragments
(`GU_PackedFragment`, Pack SOP with `name`) share one source detail, read with `GU_PackedFragment::detailPtr()`
(not `getPackedDetail()`, which is a different detail from the one `getPrimitiveRange()` indexes: using it gave
fragments wrong or empty pieces, the empty ones falling back to a pscale-sized box); a fragment's points are one piece, keyed by
its first primitive (GU_PackedFragment does not expose its attribute value; no two fragments share a primitive),
pivot and scale. The pivot is `GEO_PrimPacked::getPivot`; the body origin is the point's `P`.
Packed rotation is read from `getFullTransform4` (not `orient`) and written with `setLocalTransform`;
`splitPackedMatrix` (`UT_Matrix3D::extractScales` + `UT_QuaternionD::updateFromRotationMatrix`; shear dropped,
a mirror flips the third row) and `composePackedMatrix` (`getRotationMatrix`, per-row scale kept) use the HDK, so
packed rotations follow Houdini's own quaternion convention.

### writeResultsBack

0. Move events (read once into `st.moves` right after the step; impacts use them too): every awake body and its
   transform (a body falling asleep appears once more). Only live dynamic entries become items, which point at their
   move event rather than copying it; `awake` changes are noted. Velocities are fetched in a threaded pass
   (Box3D allows read-only calls from several threads). Items are grouped per object.
1. `getOrMakeWritableGeometry`: a plain `SIM_Geometry` (RBD Packed Object) is replaced once by a `SIM_GeometryCopy`
   (`SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`).
2. `ensureResultAttribs` creates `v`, `w`, `orient` (a new `orient` is set to identity).
3. `resolvePointOffsets`: the stored offset if it still names the point (in range, same id / number), else by id.
4. Parallel pass: `hardenWrittenPages` first when parallel (offset ranges bucketed by 1024; one full call when nearly all
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
large scenes) = `UT_Thread::getNumProcessors()` (honours `HOUDINI_MAXTHREADS`), else `threads` (default 8), 1..32, never above Houdini's count; a change still rebuilds the world. Safe with Box3D's solver because tasks carry their own worker index, the stepping thread always runs solver worker 0 (which can finish the solve alone, so a busy pool cannot deadlock), and `step` runs inside `tbb::this_task_arena::isolate`. `UT_TaskGroup` does not carry `UT_TaskScope`, so the runner wraps each task in one. Without a runner (the tests) `Box3DWorld` falls back to Box3D's own pool. The plugin's
loops use `UTparallelFor` with untuned thresholds; parallel lambdas only read shared data or write their own element.
The Box3D world and mirror are touched in serial passes (the write-back's parallel pass writes only its own entry). The
shared-state mutex is held for the whole step.

## Affector static colliders

`Box3DAffectors.cpp` (`syncAffectorBodies`, called from `syncFromGeometry` after the objects, mutex held).
Objects that the solver is not given but that affect a solved object through a collision relationship
(`SIM_Object::getConstAffectors(arr, "SIM_RelationshipCollide")`) become static Box3D bodies, one triangle mesh each.

* Gather: affectors of every solved object, deduplicated; objects that are themselves solved are skipped.
* State: `Box3DSharedState::affectors` (object id -> `Box3DAffectorBody`: handle, stamp, signature, last pose). It lives
  in the shared state, so a world rebuild starts empty. Affectors not seen this sync are destroyed.
* Mesh: polygons only, fan-triangulated, point order of first use. Box3D meshes are one-sided and want CCW front faces;
  Houdini front faces are taken as clockwise, so triangles are flipped unless `Box3DData::flipwinding` (on the affector).
* Pose: `SIMgetGeometryTransform`. Rotation + positive per-axis scale + translation (`decomposeScaledRigid`,
  Box3DAffectorMath.h, tested in tests/core_mesh.cpp): the body takes the pose and the mesh shape the scale
  (`createMeshShape(..., scale)`, later `setMeshScaleByHandle` -> `b3Shape_SetMesh`, which keeps the mesh and wakes what
  touches it), so an animated transform never rebuilds the mesh. Shear or a mirror is baked into the vertices (body at the
  origin, winding fixed for mirrors).
* Signature: P and primitive-list data ids, counts, friction / bounce / flip, and the matrix when baked. Same signature:
  only the pose (`setTransformByHandle`, which neither pushes nor wakes) and the mesh scale are updated. The mesh's
  point remap is a vector indexed by offset. A failed build keeps handle 0 and is
  not retried until the signature changes (counted as failed, one warning per step).
* Core: `Box3DWorld::createMeshShape` (static bodies only; validates indices and finiteness). Box3D keeps a pointer to the
  `b3MeshData`, so `Impl::bodyMeshes` owns it: freed after `destroyBody` / `destroyShapesByHandle`, and after the world.
* Material: an optional `Box3DData` on the affector (friction, bounce); the body type is always static.
* Not done: SDF / volume collision of Static Objects, height fields, mesh sharing between identical affectors,
  `getCollider`-style per-pair collider selection, affectors with `SIM_ColliderInfo` options.
* Verified against the HDK docs: `void SIMgetGeometryTransform(UT_DMatrix4 &result, const SIM_Object &)` (SIM_Utils.h;
  geometry local space to world space) and `void SIM_Object::getConstAffectors(SIM_ConstObjectArray &, const char *relationshiptype) const`.

## Not supported on purpose

* Box3D's mover and pogo joints (`b3CreateMoverJoint`, `b3CreatePogoJoint`) and character-mover queries are left out:
  they are game character controller tools and have no place in a Houdini DOP workflow. Likewise Box3D's explosions
  (`b3World_Explode`) and wind (`b3Shape_ApplyWind`): Houdini already has Force DOPs (Wind, Fan, POP Force, ...), which the
  solver applies, so a second way to do the same thing would only add confusion.
* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* `min_activation_impulse` (inactive bodies that activate when hit) was removed: Box3D has no equivalent, and it needed a contact scan after every step.
## Parameters

Solver (read every step; only the thread count rebuilds the world): `gravity` (0,-9.80665,0), `substeps` (min 1,
default 4), `autothreads` (off), `threads` (1..32, default 8 = `b3dop::kDefaultWorkerCount`, also `WorldSettings`'
default), `enablesleep`, `timescale` (1), `enableaging` (off), and
Box3D's tuning with Box3D's defaults: `contactstiffness` (30 Hz), `contactdamping` (10), `contactpushspeed` (3),
`bouncethreshold` (1), `contactrecycledistance` (0.05), `maxlinearspeed` (400), `enablecontinuous` (on). Force DOPs:
`useforces` (on), `forceswake` (off, disabled while `useforces` is off).

Box3D Data (per object): `defaultshape` (0 box, 1 sphere, 2 convex hull, 3 concave, 4 capsule, 5 none), `autofit` (on),
`hullperconnected`, `bodytype` (0 static, 1 kinematic, 2 dynamic), `computemass` (off), `mass` (1), `density` (1000),
`friction` (0.6), `bounce` (0), `lineardamping`, `angulardamping`, `gravityscale` (1), `enablesleep`, `sleepthreshold`
(0.05), `fastobject` (off), `contactrecycling` (on), `addimpact` (off). Names match the point attributes that override them; material values
have a hard minimum of 0 (Box3D asserts on negatives). `toBodySettings()` maps them to `b3dop::BodySettings`.

## Point attributes

`P` (required), `orient` (not for packed points), `v`, `w` (rad/s), `id`, `active`, `animated`, `mass`, `computemass`,
`density`, `inertialtensorstiffness`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`,
`enablesleep`, `sleepthreshold`, `ccdthreshold`, `awake`, `enabled`, `fastobject`, `fastrotation`, `contactrecycling`,
`collision_category`, `collision_mask`, `collision_group`, `lock_linear`, `lock_angular`, `dead`, `age`, `life`,
`deforming`, `bullet_autofit_valid`, `bullet_georep`, `bullet_autofit`, `bullet_primS` / `R` / `T`, `bullet_radius`,
`bullet_length`, `pscale`, `scale`. Negative material values are clamped to 0, non-finite ones ignored (`gravityscale`
may be negative). Identity is `(object id, id or point number)`.

Written: `P`, `orient`, `v`, `w` and packed transforms (awake dynamic bodies), `awake` (if it exists),
`bullet_autofit_valid`, `mass` (computemass), `age` / `dead` (aging); `dead` points are deleted.

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fallbacks (on the step a shape is built), points that could
not become a body, duplicate `id`, non-finite `P`, non-finite `v` / `w` (read as zero; `readPointState`'s
`badVelocity`), Geometry not writable, bodies not matched to a point at write-back. Also: non-finite solver Gravity
(ignored, previous kept), non-finite forces from Force DOPs (skipped per body), and Box3D assertions that failed during the step (`Box3DWorld::assertCount()` delta,
reported by a scope guard in `solveObjectsSubclass`).

## Debug logging (`Box3DDebug.h`)

Per-step diagnostics use `B3D_LOG_VERBOSE` / `B3D_LOG_IF(cond, ...)` and only print with `BOX3DDOP_VERBOSE=1` (a
flushed line per frame slows Houdini a lot); plain `B3D_LOG` is for one-off events (rebuilds, bodies added / removed /
rebuilt, fallback notes). All macros compile to nothing without `BOX3DDOP_DEBUG`; arguments used only by logs get
`(void)x`. Output: stderr, `[b3dop]` prefix. The only counter is `rebuilds`.

## Whole-object bodies (`syncWholeObject`)

The older RBD workflow: each DOP object (RBD Object, RBD Point Object, RBD Fractured Object's pieces) is one rigid body,
its state in the object's `Position` data instead of point attributes.

* Which objects (`isWholeObject`): the object has a `SIM_ColliderLabel` at `SIM_COLLIDERS_DATANAME` ("Colliders") whose
  label is not `SIM_COLLIDERLABEL_PACKEDOBJECT` ("PackedObject"). RBD Packed Object and RBD Object both attach the label
  (checked in Houdini); an Empty Object + SOP Geometry has none and stays per point. No user switch, as in Bullet.
* Pose: RBD Object's RBD State (an `RBD_State`, a `SIM_Motion`) is attached as `Position` (checked), and the object's
  Geometry data has `positionpath = ../Position`. `readWholePose` reads any `SIM_Position` there: pivot (object
  space), the world position of the pivot through `getTransform` (so the translation convention does not matter on
  read), orientation, and velocity / angular velocity (radians; `SIM_Motion::getAngularVelocity`) of a `SIM_Motion`.
  The body origin is the pivot, and `applyBodyMass` moves the centre of mass there, as for points. No Position data
  (only until the first write-back): the object where its geometry is, about the centre of its object-space bounding
  box, zero velocity.
* Object space = the geometry through its own transform (`SIM_Geometry::getTransform`; scale / shear baked into the
  shape points). Shapes (`buildWholeObjectShapes`) from Box3D Data's Default Shape / Create Convex Hull per Set of
  Connected Primitives: box / sphere / capsule fitted to all points, convex one hull (or per connected set), concave
  one hull per primitive `name`; unkeyed hulls (by content); `none` or no points: shapeless static. The shape signature
  (`wholeObjectShapeSig`: geometry identity, P / primitive list data ids, counts, own transform, pivot, shape settings)
  changing rebuilds the shapes on the same body.
* Settings (`readDynamicProps` with unbound point attributes; changes go through `applyPropChanges`, read every step):
  Box3D Data whenever it is attached. Without it, the RBD Object's own data (`readRbdObjectSettings`, then
  `b3dop::rbdObjectSetup` over Box3D Data's defaults; option names from a Houdini 22.0 probe, read by name with
  `box3dDataOption` since none of the classes is in the HDK):
  * RBD State (`Position`, `RBD_State`): `computemass`, `density`, `mass` (the parameter, not a computed mass: Box3D
    computes density x volume from its shapes as for points), `inertialtensorstiffness` (the inertia scale). Its
    custom inertia tensor (`computeinertialtensortype`, `itensorlocalinv`) and glue options are not read. `p` /
    `computecom` is the pivot, already read through `getPivot`.
  * `PhysicalParms` (`SIM_PhysicalParms`): `friction`, `bounce`. `dynamicfriction`, `bounceforward`, `temperature`
    have no Box3D equivalent.
  * `Geometry/BulletData` (`SIM_BulletData`): `bullet_georep` (unknown names such as cylinder / plane: box),
    `bullet_groupconnected`, `bullet_autofit`; with AutoFit off a box / sphere / capsule is built from `bullet_primS`
    (half extents, as for points), `bullet_primT` (offset from the pivot), `bullet_primR` (Euler XYZ degrees),
    `bullet_radius`, `bullet_length` instead of being fitted (`wholeObjectShapes`; the spec is in the shape signature).
    Padding / shrink and Bullet's sleep options stay unread (see "Not supported on purpose"); `bullet_add_impact` is
    not read either, so Impacts stay opt-in through Box3D Data.
  Values Box3D cannot use (non-positive mass / density / inertia, negative friction / bounce, non-finite) keep Box3D
  Data's defaults. Houdini's RBD Object defaults differ from Box3D Data's: density 1000 with Compute Mass (a 1 m box is
  1000 kg), friction 1, bounce 0.5, convex hull. Body type is the node's (no `active` / `animated`).
* Mirror: one entry, key `makeBodyKey(objectId, kWholeObjectId = -1)`, `wholeObject = true`, `pivot`, no point
  (`pointOff` -1, `pointIndex` -1 in Impacts). The object scans every step (one body: no fingerprint).
  `Box3DObjectSync::wholeObject` makes aging and point `force` / `torque` skip the object; Force DOPs still apply.
  Joint anchors name the object (Anchors: an object with one body).
* Edits: Position data differing from what the solver last wrote (`poseDiffers`) teleports the body, as an edited point
  does.
* Write-back (`writeWholeObject`, from `writeResultsBack`'s move events, serial): the `SIM_PositionSimple` at
  `Position` (created as a `SIM_Motion` if missing; other `SIM_Position` kinds are left alone with a warning): pivot,
  orientation, translation, then velocities when it is a `SIM_Motion` (RBD State stays an RBD State). The translation
  is first written as "pivot displacement" (rotate about the pivot, then translate); if `getTransform` does not then
  put the pivot on the body, it is rewritten as the pivot's world position. The mirror takes the read-back
  (`readWholePose`) so float storage is not an edit. Double-precision setters (`setPosition(UT_Vector3D)`).
* HDK, from the reference: `SIM_ColliderLabel::getColliderLabel` (`GETSET_DATA_FUNCS_S`), `SIM_COLLIDERS_DATANAME`,
  `SIM_COLLIDERLABEL_PACKEDOBJECT`, `SIM_POSITION_DATANAME`; `SIM_Position::getPivot` / `getOrientation` /
  `getTransform`; `SIM_PositionSimple` `Position` / `Pivot` (V3) and `Orientation` (Q) GETSET; `SIM_Motion` (a
  `SIM_PositionSimple`) `Velocity` GETSET, `setAngularVelocity` / `getAngularVelocity` in radians;
  `SIM_Geometry::getTransform`; the GETSET float / double overloads from `SIM_OptionsUser.h`.

## Joints

Box3D's joints, set up the way Houdini's old constraint system is: a **Constraint Network** DOP holds the network
geometry, and constraint relationship nodes wired into its second input hold the settings, one node per kind of
constraint, named by the primitives' `constraint_name`. Here those nodes are ours, one per Box3D joint type, with Box3D's
own settings and defaults. SideFX's own constraint data (Glue, Hard, Spring, Bullet Soft, Cone Twist) is mapped onto
Box3D joints, approximately (see "SideFX constraint data" below). `constraint_type` (position / rotation / all) is read
for SideFX data only; for our nodes the joint type decides what is constrained. Character-mover joints (mover, pogo)
are left out.

Naming: attributes holding the state at the initial pose are called `rest*` (`restlength`); attributes the solver
keeps current are the plain names (`P`, `orient`) or `current_*` readings. The solver writes no attribute it does
not read itself, except feedback (`force`, `distance`, `torque`, `angle`, `current_*`), as the Bullet solver does.

**Core** (`Box3DJoints.h`, `Box3DWorld`, tested in `tests/core_joints.cpp`):
* `JointType` (weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter) and `JointSettings`:
  every setting of every type in Box3D units (radians, rad/s, Hz). Box3D's FLT_MAX caps are negative values here
  (unlimited): break thresholds, parallel `maxTorque`, distance `maxLength`, `maxTensionForce`,
  `maxCompressionForce` (Box3D's signed `lowerSpringForce` / `upperSpringForce`, renamed so the sign cannot be got
  wrong). Distance `restLength < 0` (`restlength`) = the anchors' distance when the joint is made. Weld: `enableLinearSpring` /
  `enableAngularSpring` (off = rigid; Box3D's own rule is "hertz 0 = rigid", which reads backwards with the name
  stiffness). Every Box3D "hertz" is called stiffness.
* `jointParams()`: one row per setting (attribute / parameter name, label, kind, joint types using it, member
  pointer, help). The constraint relationship nodes, the primitive attribute reader and `sameJointSettings` are all driven by
  it. `angleTypes` marks a Float that is an angle for some types only (`motor_speed`: revolute deg/s, prismatic and
  distance units/s). `Rotation` is Euler XYZ degrees in Houdini and a quaternion in the core.
* `Box3DWorld::defaultJointSettings(type)` copies `b3Default*JointDef`, so the node's defaults are Box3D's.
* Houdini units in one place: `jointParamSize` (1 or 3 values), `setJointParam` (Houdini values -> settings: degrees
  -> radians, toggles, Euler rotations through a caller-supplied converter, `box3dEulerToQuat`) and `getJointParam`
  (the inverse, for node defaults). Used by the nodes and by the primitive overrides (for everything but floats,
  which multiply); a test round-trips every setting.
* `createJoint(type, bodyA, bodyB, frameA, frameB, settings, userData)`: frames are world space (anchor P +
  orient) and converted to each body's local frame from its current transform (`toLocal`). A null BodyHandle is the
  world: one shapeless static body per world, made on first use (`Impl::anchor`, not in `bodyCount`). Settings are
  clamped first (`sanitized`) to what Box3D asserts on: non-negative stiffness / damping / caps, ordered limits,
  cone angle 0..pi/2, revolute target -pi..pi, finite vectors, unit quaternions (the test checks
  `assertCount` does not move in a RelWithDebInfo build). Filter joints always have `collideConnected` off.
* `updateJoint` pushes every setting of the type through Box3D's setters (all are live) and wakes the bodies.
  Bodies, frames and type cannot change live: the caller recreates the joint.
* `jointFeedback`: constraint force / torque (vectors), linear / angular separation, and per-type readings named
  `current_*` (plus `motor_torque`, `motor_force`, `spin_torque`) so no reading shares a name with a setting (a
  test checks). Box3D's `b3Joint_GetAngularSeparation` compares the bodies' rotations, not the joint frames
  (`relQ = inv(qA) * qB` in joint.c), so it reports a non-zero angle for a perfectly held joint between bodies that
  started rotated relative to each other; we report it as Box3D gives it (the test documents this).
  For wheel joints Box3D's angular separation is a `// todo` that asserts, so it is reported as 0.
* Breaking: Box3D raises a joint event when force / torque passes the threshold (solver.c) but does not break the
  joint; `brokenJoints` returns the events' user data and the caller destroys them. `b3DestroyBody` destroys a
  body's joints, so handles can go stale: `isJointValid` before use.

**Houdini** (`Box3DJointSync.cpp`, `SIM_Box3DConRel.cpp`):
* Nodes: `SIM_Box3DConRel` (`SIM_ConRel` + `SIM_OptionsUser`, like SideFX's `SIM_ConRelSpring`; abstract,
  `DECLARE_CLASSNAME` only) and one subclass per joint type from `BOX3D_DECLARE_CONREL`, each with its own automatic DOP
  (`box3d_<type>conrel`, label "Box3D <Type> Constraint Relationship", default Data Name `<type>`). `describe(type)`
  builds the parameters (`Box3DParms`, defaults from `getJointParam`) once per type: every setting the type uses,
  common ones first, parameter name = attribute name (no per-type prefixes). `settings()` reads them through
  `setJointParam` in Box3D units over Box3D's defaults.
* Networks: every relationship of a solved object (group and affector side) whose relationship type data is a
  `SIM_ConstraintNetworkRelationship`, once per relationship name (a relationship is on every object it constrains),
  geometry from its `getGeometry()`; plus each object's own `ConstraintGeometry` (Attach Internal Constraints to
  Object). Network key = hash of the relationship name (or the object id). `anyJointNetwork` checks the same, without
  reading geometry; while true, each full object scan fills `Box3DObjectSync::byName` (point `name` -> entry, first
  wins) and an object without `namesKnown` cannot skip. `name` is a prop marker, so a renamed piece rescans.
* Kinds (`networkKinds`): the `SIM_Box3DConRel` subdata, by data name, of the relationship, its
  `SIM_ConstraintNetworkRelationship` and its geometry (internal: the ConstraintGeometry and the object); the first
  name found wins. Kinds are re-read every sync (a few subdata); any change reprocesses the network.
  The HDK's `SIM_ConstraintNetwork` iterators / accessors are not used: the geometry is read directly, which keeps
  the fingerprinting, live frames and local frames below.
* A network is reprocessed only when its fingerprint (identity, topology, data ids of `P`, `name`, `orient`,
  `constraint_name`, `auto_restlength`, every table attribute, the `broken` group), its kinds, or
  `Box3DSharedState::bodyVersion` (bumped when bodies or affectors are added / removed) changed. Otherwise its
  joints keep the network's `liveStamp`, like skipped objects' bodies.
* Per two-point primitive: `constraint_name` -> kind (missing or unknown: counted, ignored), anchors resolved
  (`Anchors::resolve`: empty = world; a piece name in any object in object order; `object/piece`; an object with
  exactly one body; an affector object's name -> its static mesh body). Unresolved: no joint, retried when bodies
  change. Settings = the kind's + matching primitive attributes, the Constraint Network DOP's rule: float attributes
  multiply the node's value (angles too: the attribute is a factor, so no degree conversion), toggles (int), vectors
  and Euler rotations (degrees) replace it. So a node value of 0 (most limits and targets) or a negative "never /
  unlimited" cap cannot be changed per primitive except by setting the node to 1. Signature = type, bodies, both
  frames: equal and the joint valid -> `updateJoint` if settings differ; else recreate. `Box3DJointEntry` (in
  `st.joints`, key network + primitive offset) is the joint's user data, so nodes must not move (unordered_map).
  Entries not stamped are destroyed and erased.
* After the step (`jointsAfterStep`): broken joints are destroyed and flagged (never remade, even if the
  primitive changes), and written back through the relationship's `getGeometry()` (non-const: a `SIM_GeometryCopy`)
  or the object's ConstraintGeometry (made a `SIM_GeometryCopy` once, like the object geometry): the `broken`
  primitive group, then `force` / `torque` (magnitudes), `distance` / `angle` (linear / angular separation, angle
  in radians, the Constraint Network DOP's names and units) and the per-type `current_*` readings (degrees, as
  before) every step. The fingerprint is re-recorded after the write so it is not seen as an edit. A network that
  re-imports its SOP every step (Overwrite with SOP) overwrites the group; `Box3DJointEntry::broken` still holds until
  a world rebuild. `Box3DJointNetwork::writable` is false when the last write found no geometry; such a network keeps
  its frames in the joint signature (see below).
* `next_constraint_name`: a broken joint whose primitive has one is not marked broken. Its `constraint_name` is set to
  it and the attribute cleared (so it switches once), its local frames (read before it was destroyed) put through the
  bodies' poses give the anchors' `P` / `orient` (`switchFrames`: written by `writeLiveFrames` as if the joint were
  live), and the fingerprint is zeroed so the next sync reprocesses the network and makes the new joint from the saved
  local frames, exactly where the old one was. `next_constraint_type` is ignored, like `constraint_type`.
* **Live frames** (written networks): after every step `writeLiveFrames` sets each live joint's
  anchor points' `P` / `orient` to where its frames are now (`Box3DWorld::jointWorldFrames`: body pose times local
  frame; a world end does not move). The network data is cached with each timestep (SIM_GeometryCopy), so a world
  rebuilt mid-simulation makes its joints from the cached `P` / `orient` against the cached body poses, which gives
  exactly the same local frames as continuing (`tests/core_joints.cpp` compares a rebuilt world with the original).
  * The authored frames are not kept (no attribute is written that the solver does not read): the network's
    `P` / `orient` are the authored values only until the first step.
  * **Local frames** (`local_P` / `local_orient`, point attributes, the names SideFX's Constraint Network uses for an
    anchor in its transform's local space): a joint made from `P` / `orient` saves the frames Box3D keeps
    (`jointLocalFrames`: relative to the piece, position in its axes, in scene units) after the step. A joint made
    later (a world rebuilt mid-simulation) is made from them (`createJoint(..., framesAreLocal)`) when they still
    agree with the live values: `localToWorld(body, local)` must land on the point's `P` / `orient` (1e-5
    relative, quaternion dot 1 - 1e-6), which holds when all three were written after the same step and fails for
    a renamed, moved or new anchor (a zero `local_orient` means none). This makes rebuilds exact: going through
    float world positions loses ~1 float step at the coordinates' size per rebuild (4.6e-4 at x = 12000 in the
    test), the local frames are small offsets kept bit for bit (`asLocal` only renormalises a quaternion that is
    not already unit length). The live `P` / `orient` are still read: to make new joints and to see edits.
  * Edits: the entry remembers what it wrote (`wroteA` / `wroteB`, as stored in float). A reprocessed network whose
    anchor differs (1e-5 relative; q and -q equal) was edited upstream (a SOP Solver): the joint is remade from the
    new `P` / `orient`. The signature therefore leaves the frames out for written networks; a network that could not
    be written (`writable` false) keeps the frames in its signature, so editing them remakes the joint.
  * A distance joint made with a negative rest length gets the anchors' distance at that moment written to the
    `auto_restlength` primitive attribute (created with default -1), so a rebuild keeps the rest length instead of
    the current, stretched distance. Not `restlength`: that attribute multiplies the node's value, and -1 times the
    distance would be negative again. It is only read while the joint's rest length is negative.
  * Our writes bump `P` / `orient`, which are in the network fingerprint; the fingerprint is re-recorded after the
    write, so an unchanged network still skips. Writing happens every step.
  * A Constraint Network with Overwrite with SOP re-imports the SOP every step and overwrites the live values, so a
    resim from a cached frame other than the first uses the authored frames there.

**SideFX constraint data** (`core/Box3DSideFX.*`, tested in `tests/core_sidefx.cpp`; read in `Box3DJointSync.cpp`):
* Found by data type among the same subdata as our nodes (`networkKinds`): `SIM_ConRelGlue`, `SIM_ConRelHard`,
  `SIM_ConRelSpring`, `SIM_ConRelBulletSoft`, `SIM_ConRelConeTwist` (names from a Houdini 22.0 probe; their default
  Data Names are Glue, Hard, ConRelSpring, Soft, ConeTwist). Only Hard and Spring have HDK headers, so all five are read
  the same way, by name (`box3dDataOption` in `Box3DHoudiniUtil.h`: `dynamic_cast` to `SIM_OptionsUser`, polymorphic
  through its virtual destructor, and `importOption`, else `SIM_Query::getFieldFloat("Options", 0, name)`, scalars
  only), per option in `sideFXParams()`. Option names are the node
  parameter names, from the same probe. `Box3DJointKind` holds the SideFX type and options instead of a joint type.
* Per primitive: matching primitive attributes modify the options (floats multiply, ints / vectors replace; our
  `jointParams` overrides are not applied), `constraint_type` is read (missing or unknown: all), then `mapSideFX`
  picks the joint:
  * Glue: rigid weld holding the pieces where they are, never colliding. Bullet's strength is an accumulated impulse;
    here Break Force = strength / DOP timestep (the force that gives that impulse over one step), < 0 never. Half-life
    and propagation are ignored, and the weld also sees steady loads (weight), not only impacts.
  * Hard: all -> rigid weld holding the current offset (rest length ignored); position -> rigid distance joint at the
    rest length, or spherical when it is ~0; rotation -> motor joint, angular spring only at 60 Hz / ratio 2 with
    unlimited torque (Box3D has no rigid rotation-only joint). CFM, ERP, iterations and the angular motors are ignored.
  * Bullet Soft: stiffness is already Hz. all -> motor joint with linear and angular springs (enableangular picks the
    angular stiffness / damping), frames offset by the rest length; position -> sprung distance joint at the rest
    length, or a linear-only motor joint when it is ~0; rotation -> angular-only motor joint. Plasticity is ignored.
  * Spring: as Soft, after converting Hooke's constant and absolute damping per joint: Hz = sqrt(k / m) / 2 pi,
    ratio = c / (2 sqrt(k m)), with m the reduced mass of the dynamic bodies (one dynamic body: its mass) for the
    linear spring, and the mean principal inertia the same way for the angular one. Read when the network is
    reprocessed, so a mass change alone does not update it.
  * Cone Twist: spherical joint. Box3D's cone is round: the smaller of Max Up / Max Out, clamped to Box3D's 90 degree
    maximum; 180 or more turns it off (Houdini's default). Twist limit +-Max Twist (180 or more: none). Bullet twists
    about x, Box3D about z, so each frame is turned so z = the twist axis and x = the up axis (constrained axes on
    point 0, goal axes on point 1). Softness, the soft-constraint options, translation ranges, rest length and the
    motor are ignored.
  * Hard / Spring / Soft / Cone Twist: Collide Connected = !disablecollisions.
* **Frame offsets**: a SideFX joint's frames need not sit on its anchors. `sideFXFrameOffsets` gives each anchor an
  offset in its own axes (`adjA` / `adjB` on `Box3DJointEntry`): a rotation (cone twist) and/or a shift along the anchor
  line so the frames meet when the anchors are `separation` apart (Soft / Spring all with a rest length), or are as
  far apart as now (`holdSeparation`: Glue, Hard all; anchors from Connect Adjacent Pieces sit at the centroids, and
  a plain weld there would pull the pieces together). The joint is made from `applyFrameOffset(anchor, adj)`;
  `writeLiveFrames` and the `next_constraint_name` switch write `removeFrameOffset(frame, adj)` back as P / orient,
  so the anchors stay anchors, and `local_P` / `local_orient` hold the joint's own frames (compared with the
  offset anchors in `localFrameAgrees`). Offsets are fixed when the joint is made; their inputs (rotations,
  separation, the hold flag, not the anchor positions) are in the joint signature. Identity offsets (our nodes)
  bypass the math, so those joints are bit for bit as before.
* The DOP timestep reaches `syncJoints` through `syncFromGeometry` (for Glue).

## Tests

Built in both configurations (`-DBOX3DDOP_CORE_ONLY=ON` builds only these):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Box3D version, create / destroy; expects `OK` |
| `b3dop_core_physics` | Handle API (and the test-side `KeyedWorld`), shape mapping, explicit primitives, mass, body types, resting heights, hulls (limits, caching, instancing), packed matrix math, `sanitizeQuat` / `storedPosition`, setters on existing bodies (material, damping, gravity, sleep, density, enable, shape replacement), live tuning, motion locks (world axes, live changes), fast rotation (the cap, live toggle), CCD threshold (sweep vs tunnel, live, invalid values), collision filtering (bits, positive / negative groups, re-filtering), contacts, mass centring / inertia scaling, sleep threshold, kinematic targets, external forces (force / torque per step, summing, non-finite input, static / kinematic / sleeping bodies, gravity as a force), contact recycling (world distance live, per-body flag at creation and on existing bodies, velocity kept, sleepers not woken) |
| `b3dop_core_joints` | Every joint type (weld holds, distance keeps length, revolute motor + live speed change, prismatic limit along a rotated frame, motor joint, wheel spin, filter), world anchors, refused joints, clamping (no Box3D asserts), breaking events, frames on moved bodies, live frames (a world rebuilt from `jointWorldFrames` matches the continued one), local frames (a joint remade from `jointLocalFrames` at x = 12000 is bit for bit the same), Box3D defaults, the parameter table, feedback names never equal setting names |
| `b3dop_core_sidefx` | SideFX data: data types and option table, `constraint_type` parsing, mapping of every type and mode (glue threshold, Spring's Hz conversion, cone twist axes / smaller cone), frame offsets (hold, separation, round trip), and in a world: glue holds the centroid offset and breaks, Soft all keeps its rest length, Soft / Spring sag as their spring constants predict, Hard position is a rigid rod, cone twist stops at the smaller limit |
| `b3dop_core_rbdobject` | RBD Object settings without Box3D Data (`rbdObjectSetup`): nothing read = Box3D Data's defaults, Houdini's RBD Object defaults, unusable values ignored, Bullet Data shapes (unknown names, none, compound, connected sets, AutoFit off box / sphere / capsule), density 1000 on a 1 m box = 1000 kg |
| `b3dop_core_taskrunner [workers]` | Worlds on one shared host pool: tasks run, results match 1 worker |
| `b3dop_core_threads [workers] [side] [steps]` | 1 vs N workers: speedup, fails if positions differ by > 1e-4 |

`tests/box3d_test.hiplc` is a manual scene for the `SIM_*` code, which has no automated tests. No CI.

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
  colliders"). Box3D meshes (`b3CreateMeshShape`) and height fields only collide as static, which is why `concave` /
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
  pull the pieces together; `holdSeparation` (see "SideFX constraint data") could be offered as a node option.
* Whole-object bodies without Box3D Data: RBD Object's Create Active Object (`SolverParms/ActiveValue`) for the body
  type, and RBD State's custom inertia tensor.
* Force DOPs: only `getForce` at the centre of mass (no `getForceOnSphere` / area variants, no per-substep re-evaluation).
* Sticky collisions; contact / sensor events beyond Impacts.

## Conventions and gotchas

* Box3D headers only in `Box3DWorld.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data` classes
  registered in `initializeSIM`.
* The solver's parts live in `namespace box3dsolver`; anything one part uses from another is declared in
  `Box3DSolverInternal.h` (default arguments there, not on the definitions). Small Houdini helpers used by several
  files go in `Box3DHoudiniUtil.h`, Houdini-free math (`kDegToRad`, `quatsDiffer`, `xformPoint`) in `Box3DShapes.h`.
* Node messages go through `SIM_SolverBox3D::message` / `warnCount` ("Box3D: " prefix, first object; counts only when
  non-zero). Whole objects count separately from points (`failedObjects`, `objectFallbacks`).
* Anything kept in the world must be reconstructible from point attributes, or a scrub / reload loses it.
* Hull keys must always imply identical body-local points (`createHullWithKey` trusts them).
* Sanitize every new input path: Box3D asserts on non-unit quaternions and negative material values.
* Row-vector matrices on the Houdini side; never mix with column-vector math.
* Style: Allman braces, return type on its own line (HDK sample style), 4-space indent, aligned declarations.

## Force and torque point attributes

`applyExternalForces` also reads per-point `force` / `torque` (V3) attributes from each object's geometry into a short
list of the non-zero ones (by offset; a body finds its point by binary search), adds them to the Force DOP result for the
same body, applies the sum, then zeroes the ones it read (a write lock, followed by
`recordFingerprintAfterWrite`, so the next sync does not see the clear as an external edit). Zeroing is needed because
nothing else clears `force` without a POP Solver. The POP Force node must run before this solver. The attributes' data
ids right after the clear (or after a read found them all zero) are kept in `Box3DObjectSync::forceId / torqueId`; while
they are unchanged nothing new was written and the read is skipped.

## World settings added from the Box3D world API

Bounce iterations / propagation, isotropic inertia tolerance, warm starting, hit event threshold and the friction /
restitution mixing rules are `WorldSettings` fields, passed to `b3WorldDef` at creation and changed live in
`applyTuning`. Isotropic tolerance only affects new bodies or bodies whose mass changes (Box3D's rule). The mixing
callbacks get no context, so each rule is its own plain function chosen by `frictionRule` / `restitutionRule`;
0 passes nullptr (Box3D's own rule). Shapes are always made with hit events on. With `impactMinSpeed > 0`,
`recordImpacts` keeps only contact points whose body pair appears in the step's hit events (`updateHitEvents`,
`hadHitEvent`); 0 keeps every contact. `tests/core_physics.cpp` covers the core side.

## Parameter tabs

Every node's parameters come from `Box3DParms` (SIM_Box3DParms.h): one call per tab and per parameter (`toggle`,
`number`, `integer`, `vector`, `menu`; ranges as `Range{ min, max, hardMax }` with a hard minimum; an optional
disable condition). It keeps the names, defaults, ranges, menus and conditions alive, counts each tab's parameters for
the `PRM_SWITCHER` (`PRM_Default(count, label)` per tab) and terminates the list. A function-local static, because
Houdini keeps pointers into it. Menu tokens are "0", "1", ... in item order (read as ints).

## Package

`CMakeLists.txt` has `install()` rules and CPack (zip) that lay out a Houdini package: `Box3D.json` (from
`package/Box3D.json.in`, `hpath` is `$HOUDINI_USER_PREF_DIR/packages/Box3D`) and `Box3D/dso`, `Box3D/config/Icons`. The
JSON is enabled only for the Houdini major.minor it was built with, taken from `Houdini_VERSION`; if that is not set
it is always enabled. Checked with a stub project, not with a real Houdini build.
