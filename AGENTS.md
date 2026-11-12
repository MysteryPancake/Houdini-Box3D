# AGENTS.md

Technical notes for AI agents (and humans) working on this codebase: how the code works and why.
Build and usage instructions for users are in `README.md`.

## What this is

A Houdini DOP plugin (HDK, C++17) wrapping [Box3D](https://github.com/erincatto/box3d) (Erin Catto's 3D rigid
body engine, C17, a git submodule in `extern/box3d`). It registers three `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver: one call per timestep with all its objects |
| Box3D Data | `SIM_Box3DData` | Per-object defaults (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |

Design goals: Box3D's native performance (handle-based hot paths, no per-body hash lookups, threaded per-point passes
where safe) and Houdini Bullet behaviour (same attribute names and rules, as documented on the RBD Packed Object page).
The exception is sleeping and enabling, which follow Box3D's own model (`enablesleep`, `awake`, `enabled`; see "Sleep and
enable state"); Bullet's `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read.

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism (see
"HDK calls").

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short. Leave out anything a user cannot act on or would
not notice: how the solver does something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics, unverified HDK details. Those belong here. When a change adds a feature, add one line or table
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
CMakeLists.txt        Targets: b3dop_core, 3 test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
extern/box3d/         Box3D submodule (never edit; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg     Node icon; the build copies it to config/Icons as DOP_box3d_solver.svg and DOP_box3d_data.svg
src/                  The include root: includes are written "core/Box3DWorld.h" (bare name within a folder)
  core/               Houdini-free; built as b3dop_core and covered by the tests
    Box3DShapes.h       Header-only: shape math, packed-prim matrices, hashing
    Box3DJoints.h       Header-only: joint types, JointSettings, the joint parameter table
    Box3DWorld.h/.cpp   Pimpl wrapper over one b3World. The ONLY file that includes Box3D headers; defines TaskRunner
    Box3DAffectorMath.h Header-only: affector (Static Object) transform math
    Box3DDebug.h        B3D_LOG / counters, compiled out unless BOX3DDOP_DEBUG (args still type-checked, never evaluated)
  dop/                Houdini SIM_Data classes: the nodes and the world cache
    SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache)
    SIM_Box3DData.*     Default-settings DOP node
    SIM_Box3DJoints.*   Box3D Joints DOP node: joint defaults per type, parameters generated from the table
    SIM_SolverBox3D.*   The solver: sync, step, write-back, post-step passes (the bulk of the plugin)
    SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
  sync/               Houdini data -> the Box3D world, used by the solver
    Box3DJointSync.*    Joint networks -> Box3D joints: read, resolve names, create / update / remove, break, feedback
    Box3DAffectors.*    Affector objects (Static Object DOP etc.) -> static colliders
    Box3DHoudiniTasks.h TaskRunner on Houdini's TBB pool (UT_TaskGroup). Only SIM_Box3DWorld.cpp includes it
tests/
  KeyedWorld.h        Test-only Box3DWorld subclass: bodies by caller-chosen id, over the handle API
  core_smoke.cpp      Version print + world create/destroy
  core_physics.cpp    Unit + physics checks (CHECK macro, nonzero exit on failure)
  core_threads.cpp    1 worker vs N workers: timing + determinism
  core_taskrunner.cpp Host-supplied TaskRunner shared by 40+ worlds: no extra threads, same results as 1 worker
  core_joints.cpp     Every joint type, world anchors, live updates, breaking, feedback, the parameter table
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
`enableSleep`, `enableContinuous`, `enableSpeculative`, `contactStiffness` (Hz), `contactDampingRatio`, `contactPushSpeed`,
`restitutionThreshold`, `maxLinearSpeed`. `applyTuning` pushes changes into the live world (`b3World_SetGravity`,
`EnableSleeping`, `EnableContinuous`, `EnableSpeculative`, `SetContactTuning`, `SetRestitutionThreshold`,
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

Packed sources also get a **source summary** (`SourceSummaryCache` in SIM_SolverBox3D.cpp, process-wide): the group and
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

The Box3D world is a **rebuildable cache**; the DOP objects' point attributes are authoritative.

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
`StepProgress` is at file scope (the header forward-declares it), not in the anonymous namespace.

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
and the **prop markers**: data ids of `kPropAttribNames` (26 attributes that change settings, ask for rebuilds or rename pieces, looked
up by name), the primitive list's data id (`primitiveListDataId`: packed transforms are intrinsics; bumping it is up to
whoever edits a primitive, so an edit that does not is picked up on the next scan) and `nodeHash`. Equal fingerprint ->
skip.

* Recorded at the end of a scan and again after each of our own writes (`recordFingerprintAfterWrite`, once every write
  lock is released, since releasing may bump ids). A debug log showing `full scan, changed: ...` every step means
  something else touches the geometry after the solver.
* `forceScan` keeps an object scanning after a write the scan must act on (`active = 1`, `dead = 1`, deleted points).
* Records of objects not seen are erased, so a returning object scans. A fresh world starts without records.
* UNVERIFIED: that data ids survive the DOP copy between timesteps. If not, every step scans (correct, no gain). Debug
  builds log `sync: N of M object(s) unchanged, scan skipped` when it works.

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
* Evaluation is serial: `SIM_Force::getForce` thread-safety is not documented here.
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
  `destroyShapesByHandle`, `addPlannedShapes`, `applyBodyMass` (even with no shapes, to clear Box3D's dirty mass),
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
  gravity; negative acts as 0). A pair recycles only if both bodies have the flag and speculative contacts are on
  (`contact.c`). Box3D copies the body flag into each contact's `b3_contactRecycleFlag` when the contact is created, and
  `b3Body_EnableContactRecycling` only changes the body, so existing contacts keep the old behaviour.
  `setContactRecyclingByHandle` therefore also rebuilds them for an awake dynamic body: `b3Body_Disable` + `b3Body_Enable`
  destroys and recreates its contacts, but zeroes velocity and wakes, so the velocity is saved and restored. Sleeping,
  static and kinematic bodies are not cycled (a cycle would wake them): they take it for new contacts only. A point-attribute
  or node change goes through the usual prop path (`kPropAttribNames`, `nodeHash`, `applyPropChanges`).
  `Box3DWorld::recycledContactCount()` (`b3World_GetCounters`) feeds the tests and a verbose log line.
* **Impacts** (node `addimpact`, Add Impact Data; no attribute): `recordImpacts` walks this step's awake bodies (the
  move events), reads each one's contacts once and records them for both bodies whose object has the option: the awake
  body itself, and the other one when it is not awake (an awake one records its own) and not an asleep dynamic body. So
  sleeping bodies cost nothing and contacts where neither body is awake are skipped. Only when there is at least one does it create a fresh `SIM_Impacts`
  (`SIM_DATA_CREATE` with flags 0 replaces existing data) and call `addPointImpact(pos, normal, impulse, ptnum, otherobjid, otherptnum, -1, 0, 0,
  0, time, 0)` per point (normal of the surface hit, pointing at the body that gets the impact; ptnum = its `pointIndex`). An object
  with Impacts last step and none now loses the data (`removeNamedSubData`, `impactObjects`). No Feedback data (that is
  for mutual interaction with other solvers). UNVERIFIED: Houdini's impact normal convention.
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
  `defaultShape`) and pieces; `addPlannedShapes` adds them; `applyBodyMass`; then the initial `v` / `w` (after the mass,
  because a moving centre of mass shifts the velocity; not for static or animated bodies, nor for one that is not awake).
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
`bullet_radius`; `bullet_length` = capsule straight section, caps not included (ASSUMED); `bullet_primT` = offset;
`bullet_primR` = Euler degrees, XYZ (ASSUMED), turned into a quaternion with
`UT_QuaternionF(rad, UT_XformOrder(TRS, XYZ))`. A rotated box is an 8-corner hull; a sphere ignores rotation. If the
assumptions are wrong, only that conversion and the `hasLength` line of `applyPrimOverrides` change.

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
large scenes) = `UT_Thread::getNumProcessors()` (honours `HOUDINI_MAXTHREADS`), else `threads` (default 8), 1..32, never above Houdini's count; a change still rebuilds the world. Safe with Box3D's solver because tasks carry their own worker index, the stepping thread always runs solver worker 0 (which can finish the solve alone, so a busy pool cannot deadlock), and `step` runs inside `tbb::this_task_arena::isolate`. `UT_TaskGroup` does not carry `UT_TaskScope`, so the runner wraps each task in one. Without a runner (the tests) `Box3DWorld` falls back to Box3D's own pool. Not compiled against the HDK when written: check it builds on your Houdini. The plugin's
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
  Houdini front faces are assumed clockwise, so triangles are flipped unless `Box3DData::flipwinding` (on the affector).
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
* UNVERIFIED (check first when building): the relationship type string `"SIM_RelationshipCollide"` and Houdini's
  polygon winding (`Flip Static Mesh Winding` exists in case it is wrong).

## Not supported on purpose, and planned

* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* `min_activation_impulse` (inactive bodies that activate when hit) was removed: Box3D has no equivalent, and it needed a contact scan after every step.
* PLANNED (not started): static triangle-mesh colliders for the concave *point* case (affector meshes exist, see "Affector static colliders"). Box3D meshes (`b3CreateMeshShape`) and
  height fields only collide as static, which is why `concave` / `compound` are one hull per piece (Bullet's concave
  is a triangle mesh and may move). Intended split: a static (inactive, non-animated) `concave` point becomes a Box3D
  mesh of its packed geometry; a dynamic one keeps the compound. Mind the mesh's own memory (not the hull cache),
  `deforming` rebuilds, and that a mesh body cannot become dynamic without rebuilding its shape.

## Parameters

Solver (read every step; only the thread count rebuilds the world): `gravity` (0,-9.80665,0), `substeps` (min 1,
default 4), `autothreads` (off), `threads` (1..32, default 8 = `b3dop::kDefaultWorkerCount`, also `WorldSettings`'
default), `enablesleep`, `timescale` (1), `enableaging` (off), and
Box3D's tuning with Box3D's defaults: `contactstiffness` (30 Hz), `contactdamping` (10), `contactpushspeed` (3),
`bouncethreshold` (1), `contactrecycledistance` (0.05), `maxlinearspeed` (400), `enablecontinuous` (on), `enablespeculative` (on). Force DOPs:
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

## Joints

Box3D's joints first: every Box3D joint type that makes sense for rigid bodies is exposed with Box3D's own
settings and defaults; Houdini's geometry is only the transport. No SideFX constraint concepts (glue, constraint
relationships, `constraint_type` position / rotation / all) are used. Character-mover joints (mover, pogo) are left
out.

Naming: attributes holding the state at the initial pose are called `rest*` (`restlength`); attributes the solver
keeps current are the plain names (`P`, `orient`) or `current_*` readings. The solver writes no attribute it does
not read itself, except outputs the user asks for (Write Feedback).

**Core** (`Box3DJoints.h`, `Box3DWorld`, tested in `tests/core_joints.cpp`):
* `JointType` (weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter) and `JointSettings`:
  every setting of every type in Box3D units (radians, rad/s, Hz). Box3D's FLT_MAX caps are negative values here
  (unlimited): break thresholds, parallel `maxTorque`, distance `maxLength`, `maxTensionForce`,
  `maxCompressionForce` (Box3D's signed `lowerSpringForce` / `upperSpringForce`, renamed so the sign cannot be got
  wrong). Distance `restLength < 0` (`restlength`) = the anchors' distance when the joint is made. Weld: `enableLinearSpring` /
  `enableAngularSpring` (off = rigid; Box3D's own rule is "hertz 0 = rigid", which reads backwards with the name
  stiffness). Every Box3D "hertz" is called stiffness.
* `jointParams()`: one row per setting (attribute / parameter name, label, kind, joint types using it, member
  pointer, help). The Box3D Joints node, the primitive attribute reader and `sameJointSettings` are all driven by
  it. `angleTypes` marks a Float that is an angle for some types only (`motor_speed`: revolute deg/s, prismatic and
  distance units/s). `Rotation` is Euler XYZ degrees in Houdini and a quaternion in the core.
* `Box3DWorld::defaultJointSettings(type)` copies `b3Default*JointDef`, so the node's defaults are Box3D's.
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

**Houdini** (`Box3DJointSync.cpp`, `SIM_Box3DJoints.cpp`):
* A network is a `SIM_Geometry` on an object named `Joints` (the Box3D Joints node's Geometry Data), attached like
  the object's own `Geometry`, usually with a SOP Geometry DOP. The node (data name `Box3DJoints`) only holds
  defaults: Common tab (parameter = attribute name) plus one tab per type (`<type>_<attribute>`). It is optional.
* `anyJointNetwork` runs first in the sync; while true, each full object scan fills `Box3DObjectSync::byName`
  (point `name` -> entry, first wins) and an object without `namesKnown` cannot skip. `name` is a prop marker, so a
  renamed piece rescans.
* Gather: one network per distinct geometry (`getUniqueId`); the same SOP on several objects is read once
  (`carriers > 1`: one warning, no write-back, since writing would give each object its own copy and the
  dedupe would break). Network key = carrier object id + data name.
* A network is reprocessed only when its fingerprint (identity, topology, data ids of `P`, `name`, `orient`,
  `joint_type`, every table attribute, the `broken` group), the Box3D Joints defaults, or
  `Box3DSharedState::bodyVersion` (bumped when bodies or affectors are added / removed) changed. Otherwise its
  joints keep the network's `liveStamp`, like skipped objects' bodies.
* Per two-point primitive: `joint_type` (unknown: counted, ignored), anchors resolved (`Anchors::resolve`: empty =
  world; a piece name in any object in object order; `object/piece`; an object with exactly one body; an affector
  object's name -> its static mesh body). Unresolved: no joint, retried when bodies change. Settings = the type's
  defaults + matching primitive attributes (int for toggles, float, vector; angles in degrees). Signature = type,
  bodies, both frames: equal and the joint valid -> `updateJoint` if settings differ; else recreate.
  `Box3DJointEntry` (in `st.joints`, key network + primitive offset) is the joint's user data, so nodes must not
  move (unordered_map). Entries not stamped are destroyed and erased.
* After the step (`jointsAfterStep`): broken joints are destroyed and flagged (never remade, even if the
  primitive changes), and for single-carrier networks written back: `broken` primitive group, and with Write
  Feedback (default on) `force`, `torque` (vectors), `linear_separation`, `angular_separation` (degrees) and the
  per-type readings (angles in degrees). The data is converted to a `SIM_GeometryCopy` once (like the object
  geometry), and the fingerprint is re-recorded after the write so it is not seen as an edit. A network whose SOP
  Geometry re-applies every step overwrites the group; `Box3DJointEntry::broken` still holds until a world rebuild.
* **Live frames** (written networks, i.e. one carrier): after every step `writeLiveFrames` sets each live joint's
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
    new `P` / `orient`. The signature therefore leaves the frames out for written networks; a network that is not
    written (several carriers) keeps the frames in its signature, so editing them remakes the joint as before.
  * A distance joint made with `restlength < 0` gets the anchors' distance at that moment written to its
    `restlength` primitive attribute (created with default -1), so a rebuild keeps the rest length instead of the
    current, stretched distance.
  * Our writes bump `P` / `orient`, which are in the network fingerprint; the fingerprint is re-recorded after the
    write, so an unchanged network still skips. Writing happens every step regardless of Write Feedback.
  * Not written: networks on several objects (warning), and a SOP Geometry set to re-apply every step overwrites
    the live values, so a resim from a cached frame other than the first uses the authored frames there.

## Tests

Built in both configurations (`-DBOX3DDOP_CORE_ONLY=ON` builds only these):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Box3D version, create / destroy; expects `OK` |
| `b3dop_core_physics` | Handle API (and the test-side `KeyedWorld`), shape mapping, explicit primitives, mass, body types, resting heights, hulls (limits, caching, instancing), packed matrix math, `sanitizeQuat` / `storedPosition`, setters on existing bodies (material, damping, gravity, sleep, density, enable, shape replacement), live tuning, motion locks (world axes, live changes), fast rotation (the cap, live toggle), CCD threshold (sweep vs tunnel, live, invalid values), collision filtering (bits, positive / negative groups, re-filtering), contacts, mass centring / inertia scaling, sleep threshold, kinematic targets, external forces (force / torque per step, summing, non-finite input, static / kinematic / sleeping bodies, gravity as a force), contact recycling (world distance live, per-body flag at creation and on existing bodies, velocity kept, sleepers not woken) |
| `b3dop_core_joints` | Every joint type (weld holds, distance keeps length, revolute motor + live speed change, prismatic limit along a rotated frame, motor joint, wheel spin, filter), world anchors, refused joints, clamping (no Box3D asserts), breaking events, frames on moved bodies, live frames (a world rebuilt from `jointWorldFrames` matches the continued one), local frames (a joint remade from `jointLocalFrames` at x = 12000 is bit for bit the same), Box3D defaults, the parameter table, feedback names never equal setting names |
| `b3dop_core_taskrunner [workers]` | Worlds on one shared host pool: tasks run, results match 1 worker |
| `b3dop_core_threads [workers] [side] [steps]` | 1 vs N workers: speedup, fails if positions differ by > 1e-4 |

The `SIM_*` code has no automated tests and has not been compiled here against a real HDK (only syntax-checked against
a minimal stand-in); `tests/box3d_test.hiplc`
is a manual scene. No CI.

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
`UT_String::isstring`, `buffer`; `UT_Matrix3D::extractScales`, `UT_QuaternionT::getRotationMatrix` /
`updateFromRotationMatrix`; node icons as `config/Icons/DOP_<name>.svg`.

Taken from the HDK samples in `DOP.zip` but not compiled against a real HDK: `SIM_Force::getForce` (public, const; arguments
as in `SIM_ForceOrbit::getForceSubclass`), `SIM_EachDataCallback::callbackConst`, `SIM_DataFilterByType("SIM_Force")`,
`SIM_FORCES_DATANAME`, `forEachConstSubData`, `SIM_DATA_CASTCONST`, and which header declares `SIM_EachDataCallback` (the
solver includes `SIM/SIM_DataFilter.h` and `SIM/SIM_Force.h`). The force code was run against stand-in types for these.

Still unchecked: the timestamp semantics behind `canContinue`, data ids surviving the DOP copy, the impact normal
convention, and Bullet's `bullet_primR` order and `bullet_length` meaning (Bullet docs, not HDK).

## Known gaps / TODO

* Constraints / joints (constraint networks); Box3D has the joints.
* Force DOPs: only `getForce` at the centre of mass (no `getForceOnSphere` / area variants, no per-substep re-evaluation).
* Static triangle-mesh colliders for `concave` points and height-field colliders (see "planned"); `cylinder` / `plane` (no Box3D shape; hull or big
  box approximations possible).
* `newobjects` are solved on their first step instead of keeping their initial state (TODO in `syncFromGeometry`).
* Sticky collisions; contact / sensor events beyond Impacts.

## Conventions and gotchas

* Box3D headers only in `Box3DWorld.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data` classes
  registered in `initializeSIM`.
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

Both nodes group their parameters with a `PRM_SWITCHER` template first in the list. Each `PRM_Default(count, label)` is
one tab and `count` must equal the number of templates listed under it, so keep the two in step when adding a
parameter. The HDK documents this for ordinary nodes; it is untested here for the SIM data classes.

## Package

`CMakeLists.txt` has `install()` rules and CPack (zip) that lay out a Houdini package: `Box3D.json` (from
`package/Box3D.json.in`, `hpath` is `$HOUDINI_USER_PREF_DIR/packages/Box3D`) and `Box3D/dso`, `Box3D/config/Icons`. The
JSON is enabled only for the Houdini major.minor it was built with, taken from `Houdini_VERSION`; if that is not set
it is always enabled. Checked with a stub project, not with a real Houdini build.
