# AGENTS.md

Technical notes for AI agents (and humans) working on this codebase. User-facing
build and usage instructions live in `README.md`; this file explains how the code
works and why.

## What this is

A Houdini DOP plugin (HDK, C++17) that wraps [Box3D](https://github.com/erincatto/box3d)
(Erin Catto's 3D rigid body engine, plain C17, vendored as a git submodule in
`extern/box3d`). It registers three `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver. One call per timestep with *all* its objects |
| Box3D Data | `SIM_Box3DData` | Per-object default settings (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |

Design goal: stay as close as possible to Box3D's native performance. Everything
on the hot path is handle-based (no hash lookups per body per step) and the
per-point passes are threaded where safe.

The code is largely LLM-written by an author still learning the HDK. Treat
HDK usage with healthy skepticism, and see "Risky HDK spots" below.

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and
attributes exist, what they do, and what behaves differently from Houdini's own solvers. Keep
it short. Leave out anything a user cannot act on or would not notice: how the solver does
something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics, unverified HDK details. Those belong here in `AGENTS.md`. When a change
adds a feature, add one line or table row to the README if users need it, and put the
explanation here.

## Layout

```
CMakeLists.txt        Targets: b3dop_core, 3 test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
extern/box3d/         Box3D submodule (never edit; built as a static lib, EXCLUDE_FROM_ALL)
src/
  Box3DShapes.h       Houdini-free, header-only: shape enums, attribute->shape math, packed-prim matrix math, hashing, connected sets
  Box3DWorld.h/.cpp   Houdini-free pimpl wrapper over one b3World. ONLY file that includes Box3D headers
  Box3DDebug.h        B3D_LOG / counters, compiled out unless BOX3DDOP_DEBUG
  SIM_Box3DWorld.*    SIM_Data that owns the shared world state (the rebuildable cache)
  SIM_Box3DData.*     Default-settings DOP node
  SIM_SolverBox3D.*   The solver: sync, step, write-back (~1600 lines, the bulk of the plugin)
  SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
tests/
  core_smoke.cpp      Version print + world create/destroy
  core_physics.cpp    Unit + physics checks (CHECK macro, returns nonzero on failure)
  core_threads.cpp    1 worker vs N workers: timing + determinism check
  box3d_test.hiplc    Houdini scene for manual testing (tests/backup/ holds older saves)
```

The layering is deliberate and should be preserved:

1. `Box3DShapes.h` and `Box3DWorld.*` know nothing about Houdini. They are what the
   tests exercise, so they can be verified on a machine with no Houdini install.
2. `SIM_*` files know nothing about Box3D headers. They talk to Box3D only through
   `b3dop::Box3DWorld`.
3. `Box3DWorld.cpp` is the single translation unit that sees `<box3d/box3d.h>`.
   `box3d` is linked `PRIVATE` into `b3dop_core` so its defines (notably
   `BOX3D_DOUBLE_PRECISION`) cannot leak into plugin code and cause a precision
   mismatch.

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the three test
  executables, then `return()`s before any HDK use. This is the no-Houdini path.
* Otherwise `find_package(Houdini)` is run with `$HFS/toolkit/cmake` on the prefix
  path and `Box3DDOP` (SHARED) is built with `houdini_configure_target`, which also
  installs the DLL/so into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows).
* `BOX3DDOP_DEBUG` (default ON) defines `BOX3DDOP_DEBUG=1` for the plugin target only
  (not `b3dop_core`).
* `BOX3D_DIR` (cache path, default `extern/box3d`) must point at the folder containing
  Box3D's own `CMakeLists.txt`.
* Large-world mode: `-DBOX3D_DOUBLE_PRECISION=ON`. The boundary API already uses
  `double[3]` for positions and `Box3DWorld.cpp` converts through `decltype(b3Pos::x)`,
  so it compiles either way.
* Release or RelWithDebInfo only (the project's guidance; Debug is not supported).
* `build.sh` records `$PWD` in `build/.configured_from` and wipes `build/` if the repo
  moved. `build.bat` hardcodes an `HFS` path at the top (currently Houdini 22.0.479)
  and the VS 2022 generator; edit when upgrading.
* The Houdini target is built as a normal DOP DSO; nothing needs installing besides
  the copy `houdini_configure_target` performs. Houdini must be closed to overwrite
  the DLL on Windows.

## Core layer: `b3dop::Box3DWorld` (`Box3DWorld.h/.cpp`)

Conventions at the boundary:

* Positions `double[3]`; rotations `float[4]` as x,y,z,w (identical layout to
  `b3Quat{v.xyz, s}` and `UT_QuaternionF`); velocities `float[3]`.
* Bodies are keyed by a caller-supplied `uint64_t`. The solver uses
  `makeBodyKey(objectId, pointId)` = `objectId << 32 | uint32(pointId)`.
* `BodyHandle` is a `b3BodyId` packed into a `uint64_t` (index1<<32 | world0<<16 | generation),
  never 0 when valid. `*ByHandle` getters/setters skip the key hash lookup. A stale handle
  (body removed, world destroyed) must not be used; the solver stores handles in the mirror
  and erases entries when bodies are removed.

Two APIs sit side by side:

* **Handle API (what the solver uses to build bodies)**: `createBody(settings, pos, rot, userData)` returns a
  `BodyHandle`; `createShape` / `createHullShape(handle, settings, ...)` take the caller's `BodySettings` (a local
  of the caller, never stored); `applyMassFromShapes(handle)`; `destroyBody(handle)`; `setTypeByHandle`.
  No hash map is touched. `reserve(n)` is only a hint that sizes the hull bookkeeping lazily.
* **Keyed API (`addBody(id, ...)`, `addShape(id, ...)`, `removeBody(id)` ...)**: a convenience layer over the handle
  API, used by the tests. It fills `bodies` (key -> handle) and `bodySettings` (key -> `BodySettings`). Bodies made
  with `createBody` are invisible to it (`hasBody` is false, `bodyHandle(key)` is 0).

State (`Impl`): the `b3WorldId`, `WorldSettings`, `liveBodies` (what `bodyCount()` returns), the keyed maps above,
the hull cache, and `bodyHulls` (body HANDLE -> hulls its shapes use; only bodies with hull shapes appear).

**Mass.** Every shape add recomputes the body's mass (`b3ShapeDef::updateBodyMass`, default true). `createShape` /
`createHullShape` take `updateMass`; the solver passes false for every shape and then calls `applyMassByHandle` once
(required before the next step), which also moves the centre of mass to the point (see "The point is the centre of
mass").

**Mid-simulation setters** (handle API, used when an attribute changes on an existing body):
`setMaterialByHandle` (friction + restitution on every shape; Box3D re-mixes contact friction / restitution from the
shapes every step, so touching contacts follow), `setDampingByHandle`, `setGravityScaleByHandle`,
`setSleepEnabledByHandle` (ANDed with the world's `enableSleep`, like `createBody`; disabling sleep wakes the body),
`setDensityByHandle` (every shape, then one `b3Body_ApplyMassFromShapes`), `setEnabledByHandle` (`b3Body_Disable` /
`b3Body_Enable`: no motion, no collisions, shapes kept), `destroyShapesByHandle` (destroys every shape with
`updateBodyMass = false` and releases the body's hulls; the caller re-adds shapes and applies the mass once) and
`wakeByHandle`. Read-back getters for the tests: `getMaterialByHandle` (first shape), `getDampingByHandle`,
`gravityScaleByHandle`, `isSleepEnabledByHandle`, `isAwakeByHandle`, `massByHandle`, `shapeCountByHandle`. They share
one `thread_local` shape-id scratch vector; the solver calls them serially. Box3D accepts all of them on a disabled
body (checked in its source: proxies are only touched when they exist).

**More handle API** (Bullet-compatible extras, see "Bullet features beyond the basics"):
`applyMassByHandle(h, density, comAtOrigin, inertiaScale)` (sets every shape's density and applies the mass from the
shapes, which is required after shapes were added with `updateMass = false`; then, for dynamic bodies, optionally moves
the centre to the body origin with the parallel axis theorem and scales the inertia, via `b3Body_SetMassData`, skipped
when the centre is already at the origin and the scale is 1; a later `setTypeByHandle` recomputes from the shapes, so
the caller re-applies), `setSleepThresholdByHandle(h, threshold)` (Box3D's single threshold on `|v| + |w * maxExtent|`,
negative = Box3D's default; also `BodySettings::sleepThreshold` at creation), `setBulletByHandle` (Box3D "bullet" =
continuous collision against moving bodies too), `setTargetTransformByHandle`
(`b3Body_SetTargetTransform`: a kinematic body gets the velocity that reaches the pose in `timeStep`; the velocity stays
until cleared), `getContactsByHandle` (`b3Body_GetContactData`; one `ContactPoint` per manifold point with
`totalNormalImpulse > 0`; world position = A's world centre of mass + `anchorA`; normal flipped to point from this
body into the other) and `applyTuning(WorldSettings)` (pushes changed world settings into the live world: gravity,
substeps, sleep, continuous, speculative, contact hertz / damping ratio / push speed via `b3World_SetContactTuning`,
restitution threshold, maximum linear speed; the worker count stays). Body sleep flags are no longer ANDed with the
world's: Box3D applies the world switch itself, so `applyTuning` can turn sleeping back on.

**Collision filtering.** `CollisionFilterTable` (`Box3DShapes.h`, Houdini-free) holds every distinct `FilterProfile`
(collision group, object name, object id as text, ignore patterns) and answers `shouldCollide(a, b)` = neither ignores
the other (`patternsMatchProfile`: ignore patterns vs group / object name / object id). Patterns are Houdini style
(`matchPatternList` / `globMatch`: space-separated globs with `*`, `?`, `[...]`, later `^pat` removes). `finish()`
precomputes an N x N byte matrix (N <= 2048, else matched on the fly); Box3D's pair finding runs on worker threads, so
the table is only mutated during the sync. Box3DWorld registers `b3World_SetCustomFilterCallback` with the table;
EVERY shape is created with `enableCustomFiltering = true` and its body's profile + 1 in its shape user data, because
Box3D has no call to turn custom filtering on later. With no ignore patterns anywhere the callback returns true at once.
The filter only runs when a pair is created, so `setFilterProfileByHandle` also drops the shape's contacts:
`b3Shape_SetFilter(..., invokeContacts = true)` resets the proxy but returns early on an unchanged filter, so it sets a
temporary `groupIndex` without the reset and then the real filter with it.

`WorldSettings`: gravity, `subSteps` (default 4), `workerCount` (Box3D's own thread
pool, sized at creation, max 32; it does not use Houdini's TBB), `enableSleep`,
`enableContinuous`.

`BodySettings`: type (Static/Kinematic/Dynamic), mass (read by the Houdini solver only: it
becomes `density = mass / volume`), density (what Box3DWorld uses), friction, restitution
(the `bounce` attribute / node parameter), linear/angular damping, gravity scale, enableSleep, isBullet, `defaultShape`,
`hullPerConnectedSet`. A body's effective `enableSleep` is `body && world`.

Shapes:

* Box -> `b3MakeBoxHull` / `b3MakeOffsetBoxHull` (a box is a hull in Box3D) via `b3CreateHullShape`.
* Sphere -> `b3CreateSphereShape`. Capsule -> `b3CreateCapsuleShape` along Y,
  centred on `center`; Box3D itself degrades a too-short capsule to a sphere.
* `ShapeSpec` carries a `center` offset because boxes/spheres/capsules fitted to packed
  geometry are generally not centred on the pivot.

Hull cache:

* `createHull(xyz, n, &vol)` hashes the points (FNV-1a over raw bytes + count) and calls
  `createHullWithKey`. Callers that already know a stable key (packed instances) call
  `findHull(key)` first and only build points on a miss.
* Hull build: `b3CreateHull(pts, n, budget)` tried with budgets
  `{B3_MAX_HULL_VERTICES, 64, 44, 32, 16, 8}`. Box3D returns NULL for degenerate input
  or when the result would exceed 128 vertices/faces/edges (a fully triangulated hull
  hits the 128-edge limit at ~44 vertices, hence that rung). Hulls with volume
  <= 1e-9 or non-finite are rejected (flat: no mass, unstable contacts).
* Failure returns 0 and the caller falls back to a bounding box. Fewer than 4 points
  or any non-finite coordinate also returns 0.
* We own the `b3HullData`; Box3D clones it into its database on shape creation.
  `refs` counts shapes created from an entry; `removeBody` decrements; `pruneHulls()`
  destroys entries with `refs <= 0`. The solver calls `pruneHulls()` once per sync. Hull use is tracked per body
  handle, so `destroyBody` releases a body's hulls.
  Entries are destroyed after the world in `destroy()`.
* Several hull shapes on one body = Box3D "runtime compound"; this is the only way to
  get a dynamic concave body (triangle meshes only collide as static).
* Mass: total hull volume is the sum over shapes, so overlapping pieces double-count.

`Box3DShapes.h` highlights (all unit-tested):

* `ShapeTypeCache`: `parseShapeType` with a memo by string value (one strcmp against the last value seen, usually a
  hit). The solver keeps one in `Box3DSharedState::shapeTypes` for the life of the world (a shape type never changes
  once a body exists). `GeorepLookup` (solver) goes one step further: `bullet_georep` is a shared-string attribute, so
  it reads the point's integer handle (`GA_AIFSharedStringTuple::getHandle`) and resolves each distinct handle once per
  scan via `getTableString`; handles are per attribute table, so that map is per scan. Falls back to the string +
  memo if the attribute has no shared-string interface. UNVERIFIED against a real HDK build. Same answers as `parseShapeType`, including "unknown leaves `out` alone".
* `parseShapeType(const char*)`: exact, case-sensitive. `convexhull`, `concave`,
  `compound` (== concave), `box`, `sphere`, `capsule`, `none`. Anything else
  (including Houdini's `cylinder`, `plane`) returns false so the caller uses the node default.
  The legacy `shape` attribute and names (`convex`, `hull`, `cube`, `ball`) are intentionally gone.
* `makeShapeSpec(type, pscale, scale)`: size = `pscale * scale` as a bounding box
  (matches Copy to Points on a unit primitive). Components are `fabs`'d and clamped
  to >= 1e-4. Sphere diameter = max component. Capsule: radius = half of max(x, z),
  straight section = `y - 2r` clamped at 0.
* Matrix convention: **Houdini is row-vector** (`p' = p * M`, translation in row 3).
  `quatToMatrix3` emits row-vector layout; `matrix3ToQuatScale` splits `M = S * R`
  (row lengths = scale; Gram-Schmidt on rows; shear dropped; mirror dropped).
* `packedPointsToBodyLocal`: `(p - pivot) * S`, i.e. relative to the packed primitive's
  pivot, scale baked in, rotation removed (the body rotation restores it).
* `packedScale` rounds S to 5 significant digits so rotated instances of one source
  (whose row lengths differ by float noise, e.g. 0.99999994) produce identical cache
  keys and share a hull. Cost: ~1e-5 relative scale error.
* `packedHullKey(sourceHash, pivot, S, kind)`: `kind` 1 = merged convex hull, 2 = per-piece
  (concave / hull-per-connected-set). `-0.0` and `+0.0` hash alike.
* `connectedSets(primPoints)`: union-find over primitives sharing points.
* `fitShapeToPoints`: bounds-fitted box/sphere/capsule for packed points and the box fallback.

## Shared state and the staleness protocol (`SIM_Box3DWorld`)

This is the subtle part of the design. The Box3D world is a **rebuildable cache**, not
authoritative state. Authoritative state is the DOP objects' point attributes.

Why: DOPs copy `SIM_Data` (`makeEqual`) on first non-const access, and the solver writes
every step, so a copy happens every step. If `makeEqualSubclass` reset the world it would
rebuild every frame.

Mechanism:

* `SIM_Box3DWorld` holds a `std::shared_ptr<Box3DSharedState>`. `makeEqualSubclass`
  copies the pointer (shares), so all timestep copies from one lineage see one world.
* `Box3DSharedState` contains the `Box3DWorld`, the `mirror` map
  (`uint64 key -> Box3DBodyMirror`), a `generation` (log only), a mutex, and the
  **stamp** (`stampTime`, `stamped`).
* The stamp lives in the shared state, not in each copy; otherwise an older timestep's
  data would keep claiming the world was current after a newer step advanced it.
* `canContinue(now, dt)`: true iff `(now - stampTime) == dt` within `1e-3 * dt`. Written
  as a difference so it does not matter whether `SIM_Engine::getSimulationTime()` is the
  start or end of the step during a solve.
* Anything else (first frame, scrub back, re-solve of the same step, sim cache reload,
  changed thread count) -> `createFreshWorld`: a **new** shared state is created and
  swapped into this data only. Other timesteps keep the old state alive until the
  last one is released. The new mirror is empty, so `syncFromGeometry` re-adds every
  point as a new body from current point attributes.
* No `saveSubclass/loadSubclass`: a world cannot be serialized; reloaded sims rebuild.
* `getMemorySizeSubclass` returns a small constant so sharing copies do not each count
  the entire world against the DOP cache budget.
* `SIM_SolverBox3D::getRequestedCacheSubclass` returns 1 (keep one past timestep alive).
  In debug builds env var `BOX3DDOP_REQUESTED_CACHE` overrides it (read every step).

Because a rebuild only has point attributes to work from, `writeResultsBack` must always
write everything needed to reconstruct bodies: `P`, `orient`, `v`, `w`, and for packed
prims the primitive transform.

`Box3DBodyMirror` (per body, per world): last `pos/rot/lin/ang` handed to or read back
from Box3D (in DOP terms), `dynamic`, cached `handle`, cached `type`, plus the body's point
(`pointOff`, `pointId`, `objectId`) and a `stamp`. It also keeps what the body was built from, for mid-simulation
updates: `props` (`Box3DBodyProps`: effective mass, friction, restitution, damping, gravity scale, sleep permission,
ignored), `volume` (total shape volume), `shapeSig`, `shapeless`, `autoScale` and `sizeKey` (see "Mid-simulation
updates"). It exists to (a) detect edits by other nodes and (b) avoid hash lookups in the per-step loops. Box3D's body user data points at the entry
(`setBodyUserData`), so entries must not move: they live in an `unordered_map` (node-stable) and
are only erased together with their body.

Per-step scratch lives in `Box3DSharedState` and is reused: `slots` (per object id, a
`vector<Box3DPointSlot>`: the point's offset, key, cached mirror entry and freshly read state)
and `writeItems`. A slot's cached entry is trusted only while its offset and key still match, so
a steady-state step does no hash lookups; every slot is dropped whenever an entry is erased.
`syncStamp` / `stampedCount` replace the old `seen` set: an entry whose stamp is not current was not
seen, and equal counts mean there is nothing to remove. "Current" is per object (see "Skipping the sync scan"):
`Box3DSharedState::isLive(entry)` = the entry's object was visited by this sync and the entry carries that object's
`liveStamp`.

Smaller per-step savings: `PointAttrs` binds only `id, active, v, w, orient` per object before the skip check; the
~21 other attributes are bound (`bindBuildAttrs`) only when the object is actually scanned; `getPackedInfo` is skipped
when the geometry has no primitives; the sync pass stores each packed point's primitive offset and
3x3 in its mirror entry and write-back reuses them (after checking the point still belongs to that
primitive) instead of re-reading the transform; `pruneHulls` only runs on steps that added, failed,
rebuilt or removed a body; bodies are built through the handle API, so no per-body `BodySettings` is ever stored; move events come from one `getMoveEvents` call into a
reused vector.

## Solver step (`SIM_SolverBox3D::solveObjectsSubclass`)

```
objects.entries()==0 -> success
worldData = findOrCreateWorldData(objects)     // first object carrying "Box3DWorld", else create on objects(0)
decide reason to rebuild (invalid / thread count changed / stamp mismatch) -> createFreshWorld
lock st->mutex
syncFromGeometry   -> per object syncObject(); remove bodies whose point is gone; pruneHulls; emit warnings
stepWorld          -> b3World_Step(dt, subSteps)
writeResultsBack   -> only for bodies in Box3D's move events (awake ones), grouped per object (+ bullet_sleeping)
recordImpacts      -> SIM_Impacts on objects with Add Impact Data on (only on steps with an impact)
activateByImpact   -> min_activation_impulse: inactive bodies hit hard enough turn dynamic
writePointAttrs    -> queued point writes: bullet_autofit_valid = 1, computed mass, active = 1
ageObjects         -> Enable Aging: age += dt, dead = 1 past life
deleteDeadPoints   -> destroy `dead` points (and their packed prims)
st->stamp = now
publishEvents      -> TODO stub
```

Data names: world is attached as `"Box3DWorld"`, defaults as `"Box3DData"`; geometry is
`SIM_GEOMETRY_DATANAME`.

### syncObject (per object)

1. Gets `SIM_Geometry` (const) under `SIM_GeometryAutoReadLock`; skips objects with none.
2. Resolves `PointAttrs` (`id, active, v, w, orient` up front; after the skip check also `mass, friction, bounce,
   pscale, scale, bullet_georep, bullet_autofit, bullet_primS/R/T, bullet_radius, bullet_length`, the other
   body-setting attributes and the mid-simulation controls `bullet_want_deactivate, bullet_ignore, deforming,
   bullet_autofit_valid`; any may be invalid = default).
3. Reads `SIM_Box3DData` fallback settings if attached (body type and a hash of its mass / friction / bounce / damping /
   gravity scale before the skip check, the full settings after it).
3b. **Skip check** (before the passes): if the object's change markers equal the ones recorded last time, passes 1 and 2
   are skipped entirely (see "Skipping the sync scan").
4. **Pass 1 (parallel when >= 2048 points, `UTparallelFor`)**: per point compute body id
   (`id` attr else point number, which equals the index in a full point-range walk), key,
   `getPackedInfo`, `readPointState`, and `active`, into the persistent slot. Reads only (a slot
   whose offset or key changed forgets its cached mirror entry).
5. **Pass 2 (serial, point order)**: duplicate `id` (entry already has the current stamp) -> counted, first wins; non-finite `P`
   -> counted and treated as absent (existing body is removed later); unknown key ->
   `addPointBody` (also `readPackedPivot`); known key -> mid-simulation updates (shape rebuild, then settings; see
   below), then `active` flips (`setBodyType`; a shapeless body always stays static) and external edits
   (`describeStateDifference` on pos/orient/lin/ang vs mirror with eps 1e-5, q and -q treated equal -> push
   `setTransformByHandle` + `setVelocityByHandle`, refresh mirror).

Edits that are **not** picked up on an existing body (by design, matching Bullet): `bullet_georep`, `bullet_autofit`,
`bullet_prim*`, `bullet_radius`, `bullet_length`, a box / sphere / capsule's `pscale` / `scale` / packed scale, unless
`bullet_autofit_valid = 0` or `deforming = 1` asks for a rebuild; `enablesleep` (creation only). Box3D Data
`defaultshape` / `autofit` / `hullperconnected` likewise only apply at creation or rebuild. Solver settings are applied
to the live world every step (`Box3DWorld::applyTuning`, a no-op when unchanged), except the thread count, which
rebuilds the world.

### Bullet features beyond the basics

All read in pass 2 through `readDynamicProps` (stored in `Box3DBodyProps`, compared by `applyPropChanges`), so they
follow the mid-simulation rules below; their attributes are all prop markers (`kPropAttribNames`, 24 names).
Impact data is the exception: it is a per-object checkbox, not an attribute.

* **Collision groups** (`collisiongroup`, `collisionignore`): per scan the object's own profile (`obj.getName()`, id) is
  registered once; a point with either attribute gets its own (`pointFilterProfile`). The profile goes into
  `BodySettings::filterProfile` (so new and rebuilt shapes carry it) and changes go through
  `setFilterProfileByHandle`. `syncFromGeometry` calls `finishFilterProfiles()` before the step. DOP groups are not
  matched (Houdini's patterns can also name DOP groups).
* **Animated** (`animated` with `active` = 0): `wantedType` gives Kinematic. Instead of teleporting on an external
  pose edit, the body gets `setTargetTransformByHandle(cur, st->stepDt)` and `driven = true`; on a step with no edit a
  driven body's velocity is zeroed. An object with a driven body never skips (`anyDriven`), since the zeroing needs a
  scan. Kinematic bodies are not written back. `v` / `w` are not applied to animated bodies.
* **The point is the centre of mass.** As in Houdini RBD, a body's anchor (the point; for a packed primitive its
  pivot, which the body-local hull is built around) is its centre of mass. Shapes are always added with
  `updateMass = false` and `applyBodyMass` then calls `applyMassByHandle(..., comAtOrigin = true, stiffness)`: Box3D's
  shape centroid is moved to the body origin, inertia shifted to match. It runs after creation and rebuild (also with no
  shapes, to settle Box3D's dirty-mass flag), after a retype to dynamic (`b3Body_SetType` recomputes from the shapes),
  after activation by impact, and on a mass change. Velocities read and written are therefore the point's. There is no
  `computecom` attribute: the SOP tools place the pivot (P = pivot) at the centroid when it is wanted; an off-centre
  pivot makes the piece turn about it, like in Houdini.
* **Mass properties** (`computemass`, `density`, `inertialtensorstiffness`; Box3D Data `computemass` / `density`):
  `boxDensity` = density with computemass, else mass / volume. With computemass the mass is written back to `mass`
  (`effectiveMass`, `kMass` point write, attribute created if missing) on creation, rebuild or mass change.
* **Sleep** (`sleepthreshold`, Box3D Data Sleep Threshold, default 0.05): Box3D's single threshold and its fixed half
  second to sleep (`B3_TIME_TO_SLEEP`, compile time), instead of Bullet's linear + angular thresholds and Sleeping Time.
  Set at creation through `BodySettings::sleepThreshold`, changed with `setSleepThresholdByHandle`.
* **Fast object** (`fastobject`, Box3D Data Fast Object): Box3D's `isBullet` (continuous collision against other moving
  bodies too). Renamed from `isbullet`, which in Houdini means "treat as a Bullet object". Changes mid-simulation via
  `setBulletByHandle`.
  `bullet_sleeping`, if the attribute exists, is written in `writeResultsBack` when Box3D's move event says the body
  fell asleep (1) or it moves again (0); only changes are written (`Box3DWriteItem::sleepState`, `entry->sleeping`). A
  user-written 0 on a body we put to sleep wakes it (checked in `checkProps` scans). `deactivation_time` is not used.
* **Impacts** (Box3D Data `addimpact`, "Add Impact Data", off; per object like Bullet's object parameter, not a
  point attribute): read every step into `Box3DObjectSync::addImpact` and folded into `nodeHash`, so toggling it makes
  the object scan. A scan with it on lists every body of the object in `Box3DObjectSync::impacts` (kept while
  skipped). `recordImpacts` gathers each listed body's contact points (`getContactsByHandle`), skipping a dynamic body
  that is asleep and any contact where neither body is awake, so a resting, sleeping pile records nothing. Only when
  there is at least one hit does it create a fresh `SIM_Impacts` (`SIM_IMPACTS_DATANAME`, `SIM_DATA_CREATE` with flags
  0) and call `addPointImpact(pos, normal, impulse, ptnum, otherobjid, otherptnum, -1, 0, 0, 0, time, 0)` per contact
  point (normal = surface normal of what was hit, pointing at this body; ptnum = `entry->pointIndex`, the point number
  at the last scan). An object that had Impacts last step and has none now (or turned the option off) loses the data
  (`removeNamedSubData`, tracked in `impactObjects`), matching Houdini where the data only shows on steps with an
  impact. No Feedback data: Houdini uses it to pass impacts between different solvers in mutual-interaction loops,
  which this solver does not take part in. Awake resting contacts do count every step (they carry impulse), which is
  the cost the parameter help warns about. UNVERIFIED: that `SIM_DATA_CREATE` without `SIM_DATA_RETURN_EXISTING`
  replaces existing data, and Houdini's normal convention for Bullet impacts.
* **Activation by impact** (`min_activation_impulse`, `activationignore`): inactive bodies with a threshold are listed in
  `Box3DObjectSync::activation`. `activateByImpact` reads their contacts after the step; a contact at or above the
  threshold whose other body's profile does not match `activationignore` makes the body dynamic right away (custom
  mass re-applied, woken) and queues `active = 1` (`kActivate`), and the object is forced to scan next step. It moves
  from the next step; Bullet can activate within the step.
* **Point writes after the step** (`Box3DPointWrite`, `writePointAttrs`): flags `kAutofitValid`, `kMass`, `kActivate`,
  grouped per object (`forEachObjectRun`), offsets via the templated `resolvePointOffsets`. The fingerprint is
  re-recorded afterwards; `forceScan` keeps an object scanning when the next sync must act on what was written
  (active, dead, deleted points).
* **Dead** (`dead` != 0): pass 2 queues a `Box3DDeletion` and skips the point (its entry is not stamped, so the body is
  removed with the gone points; a fresh placeholder entry is erased). `deleteDeadPoints` checks every queued point
  (offset, `id` or point number) before deleting any, then `destroyPrimitive(prim, true)` for a packed point or
  `destroyPointOffset(off, GA_DESTROY_DEGENERATE)`, `bumpAllDataIds()`, and forces a scan. Without `id`, later points
  are renumbered and so become new bodies.
* **Aging** (solver Enable Aging): `ageObjects` adds the DOP step (not time-scaled) to `age` (created if missing) on
  every point of every object, and with a `life` attribute sets `dead` = 1 (created if missing) past a positive life.
  Writes every point every step while on.
* **Time Scale** / **Overlap Push Speed** (solver): `st->stepDt = dt * timescale` is what Box3D steps by and what
  animated targets use; `canContinue` still uses the DOP step. Push speed is applied every step
  (`setContactPushSpeed` is a no-op when unchanged) and at world creation (`WorldSettings::contactPushSpeed`).

### Mid-simulation updates

Target behaviour is Houdini's Bullet solver as documented on the RBD Packed Object page. Everything below happens in
pass 2 for an existing body; a new body is built from the same readers, so an untouched point always compares equal to
what its body was built with.

* **Settings** (`readDynamicProps` -> `applyPropChanges`): effective `mass`, `friction`, `bounce`, `lineardamping`,
  `angulardamping`, `gravityscale` (point attribute if present and usable, else the Box3D Data value), sleep
  permission and `bullet_ignore`, compared with `entry->props`; only the setters for what differs are called.
  * `mass` (and `computemass`, `density`, `inertialtensorstiffness`) -> `applyBodyMass` with density = mass /
    `entry->volume` (or the density with computemass): same as a fresh build, inertia scales with the mass.
  * Sleep: at creation `bullet_want_deactivate`, else `enablesleep`, else the node. Afterwards only
    `bullet_want_deactivate` (when it exists) changes it; `enablesleep` is creation-only, as in Bullet.
  * `bullet_ignore` != 0 -> `setEnabledByHandle(false)` (also at creation). A disabled body has no move events, so it
    is not written back.
  * Only compared when `checkProps`: the object's prop markers (`samePropMarkers`) changed since the last scan, it has
    no recorded markers yet, or it has deforming bodies. A scan triggered by, say, an animated `P` alone does not
    read them.
* **Shape rebuild** (`rebuildPointShapes`) when `deforming != 0` (every step), `bullet_autofit_valid == 0` (once), or
  `entry->autoScale` and the size key moved by more than 1e-4 relative (`sameSizeKey`). It calls `planShapes` (the
  same code that builds a new body: georep, autofit, prim overrides, pscale/scale, packed geometry) and compares its
  `shapeSignature` (hull ids + primitive numbers) with `entry->shapeSig`: equal -> nothing touched. Otherwise
  `destroyShapesByHandle`, `addPlannedShapes` with density = mass / new volume and `applyBodyMass` (so mass is
  preserved, the centre stays at the point and inertia follows the new shape), `wakeByHandle`. The body (handle, pose, velocity, identity, mirror entry) stays. A plan of
  `none`, or one whose shapes all fail, leaves the body `shapeless`, which forces it static until a later rebuild
  gives it shapes.
  * `autoScale` is set for convex / concave only (Houdini: dynamic scale works for convex hull and concave, not for
    auto-fitted primitives). `sizeKey` = the packed 3x3's row lengths (`matrix3ToQuatScale`), or `pscale * scale` for
    a plain point (whose convex / concave is the fallback box). Our own write-back recomposes the packed rotation
    keeping row lengths, hence the tolerance.
  * Objects with a deforming body never skip (`Box3DObjectSync::anyDeforming`), and always compare settings.
* **`bullet_autofit_valid` write-back**: every point built or rebuilt while it was 0 (fresh bodies included) is queued
  in `Box3DSharedState::validWrites`; `writeAutofitValid` (after `writeResultsBack`, whether or not the body moved)
  sets it to 1 via `resolvePointOffsets`, bumps the id and re-records the fingerprint. Unlike Bullet, the fitted
  values are not written into `bullet_prim*` / `bullet_radius` / `bullet_length`.
* **`id`** has no special values. It is the identity key: a different `id` is a different body (old one removed, new
  one built). The `id = -1` "force a rebuild" trick some notes describe is NOT implemented, on purpose.
* Not tested against a live Houdini yet; `core_physics` covers the Box3DWorld setters.

### Skipping the sync scan

(Per object, a skipped step also avoids `SIM_Box3DData::toBodySettings()`: only the body type is read for the fingerprint;
the full settings are built when a scan runs. `currentWorkerCount()` is what the per-step world-usable check reads, not the
whole `currentWorldSettings()`.)

Passes 1 and 2 read and compare every point, including sleeping ones. For an object that nobody touched they find nothing,
so each object keeps a `Box3DObjectSync` record in `Box3DSharedState::objectSync` with a `Box3DSyncFingerprint`: point
count, size of the point offset range, the data ids of `P`, `orient`, `v`, `w`, `id`, `active` (absent = its own value),
the Box3D Data body type (a change retypes existing bodies, which the scan applies), and the prop markers for
mid-simulation updates: data ids of `kPropAttribNames` (`mass, friction, bounce, lineardamping, angulardamping,
gravityscale, bullet_want_deactivate, bullet_ignore, deforming, bullet_autofit_valid, pscale, scale` and the extras'
attributes, 24 in all, looked up by name),
the primitive list's data id (`primitiveListDataId`; `GA_PrimitiveList::getDataId()` is in the HDK reference, and the HDK
leaves bumping it to whoever edits primitives or their intrinsics, so an edit that does not bump it is only noticed on
the next scan) and `nodeHash` (the Box3D Data node's mass / friction / bounce / damping / gravity scale). A GA data id changes whenever the
attribute is written, which is what makes it a change marker.

* Recorded at the end of a full scan, and again (`recordFingerprintAfterWrite`, for every object the sync visited) at the
  end of `writeResultsBack` and `writeAutofitValid`: after it bumped the ids of what it wrote
  AND after every write lock is released (releasing a write lock may bump ids again), from a read of the geometry as the
  next step will see it. Otherwise our own write would defeat the skip every step. If a debug log shows
  `full scan, changed: P <old>-><new>, ...` every step, the ids are being changed by something after the solver returns
  (the line lists every marker that differs).
* Next step, equal fingerprint -> `syncObject` counts the object's entries (`entryCount`) into `stampedCount` and returns the
  stored `hasDynamic`. Nothing per point runs; the mirror entries are untouched.
* Because entries are not re-stamped, the stamp check is per object: each record has `liveStamp` (what its entries carry
  while current; unchanged by a skip) and `visited` (last sync that looked at the object). Removal, write-back pass 0 and
  `isLive` all use that. Records of objects the sync did not see are erased, so an object that disappears and returns scans.
* **Assumption: packed instances keep their point count and topology.** For geometry with primitives the fingerprint does
  not track point count / offset range, and the write-back trusts the primitive offset and 3x3 stored in the mirror entry.
  Whoever wants different topology must create a NEW instance with a new `id` (that changes the `id` / `P` markers, so the
  object rescans and the old body is removed). Plain points (no primitives) still have their count tracked.
* **Not detected on a skipped step:** a change to a packed primitive's own rotation / scale that leaves `P`, `id`, `active`,
  `orient`, `v`, `w` untouched (the primitive transform is an intrinsic, not an attribute with a data id). A translation
  edit changes `P` and is caught. The write-back keeps the entry's 3x3 current with its own writes.
* Never skipped when: the last scan had a failed body / duplicate id / non-finite P (those are retried and warned about
  every step), or the world was rebuilt (the records live in the shared state, so a fresh world starts empty).
* UNVERIFIED in Houdini: whether the data ids survive the DOP copy of the Geometry between timesteps. If they do not, the
  fingerprint never matches and every step scans as before (correct, just no gain). Debug builds log
  `sync: N of M object(s) unchanged, scan skipped` when it works, and `full scan, <marker> changed` when an object that could
  have skipped did not; check that first. Also unchecked against the HDK: `GA_Detail::getP()`, `GA_Detail::getNumPrimitiveOffsets()`.

### addPointBody / planShapes / buildShapes

* Body type = node type, forced to Static if `active == 0`.
* `readDynamicProps` + `settingsFor`: `mass`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`,
  `enablesleep` / `bullet_want_deactivate`, `sleepthreshold`, `fastobject`, `bullet_ignore` override node values. `planShapes`: shape type = `bullet_georep` string via `parseShapeType`, else node `defaultShape`, then
  `buildShapes`, `shapeSignature`, `autoScale` / `sizeKey`. `addPlannedShapes` adds the pieces.
* `none` -> static body with no shapes (the only legal shapeless body).
* Size from `pscale` (default 1) and `scale` (default 1,1,1).
* `buildShapes` flow:
  1. If the point is a packed primitive, collect its inner geometry into `PieceGroup`s
     through `ShapeSources` (cached per distinct source detail pointer, plus per
     `(source detail, fragment name)` for packed fragments, for the duration of one `syncObject`).
  2. Convex/Concave on a packed point with a valid `packedScale`: compute key via
     `packedHullKey`, `findHull` first; on a miss build body-local points and
     `createHullWithKey`. Convex without hull-per-connected merges all pieces into one hull
     (kind 1); otherwise one hull per piece (kind 2).
  3. Non-keyed path (`sets`): body-local point sets per piece -> `pieceFromPoints`
     (hull or bounds-fitted box with a note).
  4. Box/Sphere/Capsule on a packed point **with autofit**: fit to bounds of the packed
     geometry (explicit prim attributes are ignored). Without autofit the packed source is
     not even collected (step 1 skips it).
  5. Nothing usable (plain point, autofit off, or empty packed geometry):
     `makeShapeSpec(...)` box/sphere/capsule from `pscale*scale`, then `applyPrimOverrides`
     with whatever `bullet_prim*` / `bullet_radius` / `bullet_length` attributes exist.
     Convex/concave falls back to a box (no overrides) and sets a note ("needs a packed
     primitive" or "packed geometry is empty").
* Notes are logged and aggregated by text into `SyncCounts::fallbacks`, then surfaced as
  one warning per kind with a count on the first frame the body is created.
* Mass: `density = mass / totalVolume`, where `mass` is the point attribute if valid, positive
  and finite, otherwise the Box3D Data node's `mass`. If neither is usable (or the volume is 0)
  `BodySettings::density` keeps its default (1).
* `createBody` returns the handle and takes the mirror entry's address as Box3D user data (the entry is inserted
  with `try_emplace` BEFORE the body exists, so a rebuild costs one mirror hash lookup per point; if the body
  cannot be built the placeholder entry is erased again). Shapes follow with `updateMass = false` when there are
  several pieces, then one `applyMassFromShapes`. If no shape could be added the body is destroyed (never leave a
  shapeless body). Initial `v`/`w` are applied last, after the mass is final, through the handle, and only when
  nonzero on a non-static body. They are NOT put in the body def: Box3D shifts a body's velocity when its centre
  of mass moves during a mass update, which would change the values.
* A scan that has >= 64 points without a cached mirror entry (a rebuild) reserves the mirror once.

### Explicit primitives (`bullet_autofit`, `bullet_primS/R/T`, `bullet_radius`, `bullet_length`)

Mirrors Houdini's Bullet Data "AutoFit" off. Applies to `box`, `sphere`, `capsule` only
(convex / concave / none ignore them). `autofit` = `bullet_autofit` attribute if present, else
the Box3D Data `autofit` toggle (default on = the old behaviour). Autofit fits a packed
point's shape to its geometry bounds; with it off, or on a plain point (no geometry to fit),
the shape is `makeShapeSpec(pscale * scale)` plus the overrides below. Each attribute is
optional; a missing one keeps the `pscale * scale` value. Logic is `b3dop::PrimOverrides` /
`applyPrimOverrides` in `Box3DShapes.h`, tested in `core_physics`.

| Attribute | Effect | Convention |
|---|---|---|
| `bullet_primS` (vector) | box half extents | HALF size: Bullet Data documents Box Size as a half-size |
| `bullet_radius` (float) | sphere / capsule radius | |
| `bullet_length` (float) | capsule straight section along Y | ASSUMED: caps not included (Bullet's btCapsuleShape height); the Houdini docs do not say |
| `bullet_primT` (vector) | shape offset from the body origin | body space |
| `bullet_primR` (vector) | shape orientation | ASSUMED: Euler degrees, order XYZ (X first); the Houdini docs do not state units |

`ShapeSpec::rot` carries the rotation to `Box3DWorld::addShape`: a capsule's axis is its Y
turned by `rot`; a rotated box is built as an 8-corner hull (Box3D has no rotated box
helper; the content-keyed hull cache shares identical ones); a sphere ignores rotation.
`PrimOverrides` carries a ready-made quaternion: the solver converts the degrees to radians and
builds it with the HDK, `UT_QuaternionF(rad, UT_XformOrder(TRS, XYZ))`, so Houdini decides what
"XYZ" means. If the assumed units/lengths turn out wrong, only that conversion in
`SIM_SolverBox3D.cpp` and the `hasLength` line of `applyPrimOverrides` change. UNVERIFIED: not
compiled against the HDK; compare a rotated `bullet_primR` box against a Bullet Data node.

### Pieces and packed geometry

* **Pieces**: inside a packed primitive, primitives are grouped by the primitive string
  attribute `name`; unnamed primitives share one piece; a detail with points but no
  primitives is one piece. `concave`/`compound` = one hull per piece on one body.
* **hullperconnected** (Box3D Data toggle): for `convexhull` only, one hull per connected
  set of primitives (sharing points) instead of per `name` or merged. Ignores `name`.
* **Packed fragments** (Pack SOP with a `name` attribute): `GU_PackedFragment` instances
  share one whole-source detail but stand for only a range of primitives. Collision geometry
  is that fragment's points as ONE piece; key includes (source, fragment name, pivot, scale).
  Fragment name is `intValue()` if int else `name()`.
* **Pivot**: `GEO_PrimPacked::getPivot` (not the transform translation). Body origin is the
  point's `P`.
* **Rotation**: for packed points read from the primitive's full transform (`getFullTransform4`),
  not `orient`. Packed points always take their rotation from the primitive and write it back
  with `GEO_PrimPacked::setLocalTransform(const UT_Matrix3D &)` (public, inherited by
  `GU_PrimPacked`).
* Write-back to packed prims: `b3dop::composeLocal3` (in `Box3DShapes.h`, Houdini-free) keeps the
  prim's per-row scale and replaces the rotation with the Box3D quaternion, then the solver calls
  `setLocalTransform` + `transformDirty()`. The quaternion -> matrix step is the DIY
  `quatToMatrix3`, deliberately NOT the HDK's `UT_QuaternionD::getRotationMatrix`: the read side
  (`matrix3ToQuatScale`) is also DIY, and `core_physics` round-trips the pair (and fails if either
  is transposed), so the two directions are guaranteed consistent without depending on the HDK's
  undocumented matrix convention.

### writeResultsBack (per object with a moved body)

0. Pass 0 (serial): read Box3D's body move events (`moveEventCount` / `getMoveEvent`): one
   contiguous array holding every AWAKE body with its final transform, cleared at the start of
   every step. Sleeping bodies are absent: their points already hold what was written when they
   fell asleep (that body's last event, `fellAsleep`, is still included, and Box3D reports zero
   velocity for it). Bodies moved by the caller never appear. Each event's user data is the
   body's mirror entry, which knows its point; velocity is fetched by handle (events carry none).
   Static / kinematic entries are skipped. Items are grouped per object (stable sort only when
   there is more than one object and they are not already in order). Velocities are then fetched in a threaded pass
   (>= 1024 items): Box3D has no bulk getter (two API calls per body), and its FAQ allows read-only calls from several
   threads. Objects are found by binary search over `objects` (sorted by id), with no per-step map.
1. Per object: `getOrMakeWritableGeometry`: `SIM_DATA_GET<SIM_GeometryCopy>`; if the object's
   Geometry is a different `SIM_Geometry` subclass (e.g. from RBD Packed Object), replace with a
   `SIM_GeometryCopy` using `SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`.
   Fails with a warning if there is no Geometry.
2. `ensureResultAttribs` creates `v`, `w` (float3, vector type) and `orient` (float4,
   quaternion type) if missing. A freshly created `orient` is initialised to identity for
   all points (default 0,0,0,0 is not a rotation).
3. Pass 1 (serial): `resolvePointOffsets`: the entry's stored point offset is verified against the
   write geometry (in range, `id` / point index still matches) and looked up again by point id if
   it is stale. Unresolvable bodies are counted and skipped (one warning).
4. Pass 2 (parallel): first, when the pass really runs in parallel (copy-on-write pages would race; a serial pass unshares
   only the pages it writes), `hardenWrittenPages` hardens `P/orient/v/w` over just the offset ranges the awake bodies write
   (HDK: `GA_Attribute::hardenAllPages(start, end)` takes an offset range; there is no documented single-page call.
   Offsets are bucketed by 1024, runs of buckets become ranges, and when >= 90% of buckets are hit one full call is
   made. UNVERIFIED in Houdini: that `end` is half-open; the code passes the first offset after the last bucket so either
   reading is correct), write attributes, compute the new packed matrix (not written yet), then store in the (distinct)
   mirror entry what the geometry now holds, WITHOUT reading it back. The mirror must equal what `readPointState`
   returns or our own write looks like an external edit next step: `P` = `b3dop::storedPosition` (double -> float ->
   double; a non-finite result keeps Box3D's value, as the read path treats it as absent), `v`/`w` unchanged (floats),
   `rot` = `b3dop::sanitizeQuat` of the float quaternion as written, or for a packed point of the rotation of the new
   matrix. `sanitizeQuat` and `storedPosition` live in `Box3DShapes.h` and are shared by the read path, so they cannot
   drift; `core_physics` pins them. If `readPointState` changes, change the prediction with it. Set
   `BOX3DDOP_VERIFY_READBACK=1` (debug build) to have the write-back also read every point back and log
   `READBACK MISMATCH` on any bit difference. `bumpDataId()` afterwards, then the object's fingerprint is re-recorded.
5. Pass 3 (serial): packed `setLocalTransform` + `transformDirty` (kept serial because the HDK
   does not document thread-safety), then one `getPrimitiveList().bumpDataId()` if any packed transform was written
   (the HDK's documented way to mark primitive intrinsics dirty; `setLocalTransform` does not bump it). Pass 2 no longer verifies the point still belongs to its primitive: topology is
   assumed fixed (see "Skipping the sync scan"); pass 3 only checks the offset is a packed primitive, and counts a
   body that fails it as unmatched instead of writing.

The write items are small (about 88 bytes): the packed-primitive matrix and offset live in a side array
(`Box3DSharedState::packedWrites`, `Box3DPackedWrite`), filled only for packed points; an item refers to its entry by
`packedIdx`, which stays valid when the items are sorted by object.

Static points are never written. Kinematic bodies are also not written back (the mirror's
`dynamic` flag is true only for `Type::Dynamic`). Of several points sharing an `id`, only the first
(the one that owns the body) receives results.

### Threading

* Box3D's thread pool is its own (`workerCount`); `autothreads` = `hardware_concurrency()`,
  else the `threads` parm (1..32). Changing the count forces a world rebuild.
* The plugin's own parallel loops use TBB through `UTparallelFor`, with per-pass thresholds
  (untuned starting points in the code): sync read 8192 points when the geometry has no
  primitives, 2048 when it has; write-back 1024. Rule of thumb in the code: parallel lambdas only read shared data or
  write element `i` of their own output; the Box3D world and the mirror are only touched
  in serial passes (the write-back's parallel pass also updates the mirror entry of its own body).
* The shared-state mutex is held for the whole sync/step/write-back.

## Not supported on purpose, and planned

* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D uses speculative contacts on the exact shapes and needs no margin. Padding would only inflate shapes.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; the
  Box3D settings above are offered instead.
* Bullet's two sleep thresholds and Sleeping Time / `deactivation_time` are replaced by Box3D's one threshold.
* PLANNED (not started): static triangle-mesh colliders for the concave case. Box3D meshes
  (`b3CreateMeshShape`) and height fields only collide as static bodies, which is why `concave` / `compound` are
  currently one convex hull per piece on one body (Bullet's concave is a triangle mesh and may move). The intended
  split: a static (inactive, non-animated) `concave` point becomes a Box3D mesh of its packed geometry; a dynamic one
  keeps the hull-per-piece compound. Mind the mesh's own memory (not the hull cache), `deforming` rebuilds, and that a
  mesh body can never be retyped to dynamic without rebuilding its shape.

## Parameters

Solver (`SIM_SolverBox3D`, all read every step; only a thread-count change rebuilds the world): `gravity` (v3, default
0,-9.80665,0), `substeps` (min 1, default 4, UI max 16), `autothreads` (default on),
`threads` (1..32, default 8, disabled when autothreads is on), `enablesleep`, `timescale` (default 1), `enableaging`
(default off), and Box3D's tuning with Box3D's defaults: `contactstiffness` (30, contact hertz), `contactdamping` (10,
contact damping ratio), `contactpushspeed` (3, contact speed), `bouncethreshold` (1, restitution threshold),
`maxlinearspeed` (400), `enablecontinuous` (on), `enablespeculative` (on). Bullet's ERP / CFM / split impulse /
iterations are deliberately not offered: Box3D's soft-step solver with substeps has no equivalent.

Box3D Data (`SIM_Box3DData`, per object, read when a body is created):
`defaultshape` (0 box, 1 sphere, 2 convex hull, 3 concave/compound, 4 capsule, 5 none),
`hullperconnected`, `autofit` (on; point attr `bullet_autofit`), `bodytype` (0 static, 1 kinematic, 2 dynamic), `computemass` (off), `mass` (1, per body:
density = mass / shape volume; disabled with computemass), `density` (1000, used with computemass), `friction` (0.6), `bounce` (0, = Box3D restitution),
`lineardamping`, `angulardamping`, `gravityscale` (1), `enablesleep`, `sleepthreshold` (0.05), `fastobject` (off),
`addimpact` (off, no point attribute). Parameter
names match the point attributes that override them. Box3D asserts on negative
mass/friction/bounce/damping, so these have hard minimum 0.
`SIM_Box3DData::toBodySettings()` maps to `b3dop::BodySettings` (`mass`, `restitution`, ...).
The Box3D Data node's data name on the object is `Box3DData`.

## Point attributes consumed (names follow Houdini RBD)

`P` required. `orient` (quat, ignored on input for packed points), `v`, `w` (rad/s), `mass`
(density = mass/volume), `friction`, `bounce` (restitution), `lineardamping`, `angulardamping`,
`gravityscale`, `sleepthreshold` (floats), `enablesleep`, `fastobject` (ints, 0 = off), `active` (0 static, else
dynamic), `bullet_want_deactivate` (int, 1 may sleep), `bullet_ignore` (int), `deforming` (int),
`bullet_autofit_valid` (int, written back as 1), and the extras in "Bullet features beyond the basics"
(`collisiongroup`, `collisionignore`, `animated`, `computemass`, `density`, `inertialtensorstiffness`,
`bullet_sleeping`, `min_activation_impulse`,
`activationignore`, `dead`, `age`, `life`), `bullet_georep` (shape string), `bullet_autofit` (int), `bullet_primS`,
`bullet_primR`, `bullet_primT` (vectors), `bullet_radius`, `bullet_length` (floats; see
"Explicit primitives"), `pscale`, `scale`, `id`. Each of `mass`, `friction`,
`bounce`, `lineardamping`, `angulardamping`, `gravityscale`, `enablesleep`, `sleepthreshold`, `fastobject`
overrides the Box3D Data parameter of the same name; negative / non-finite values are
clamped to 0 / ignored (gravityscale may be negative). Identity is
`(object id, id attribute or point number)`; use `id` if points can be deleted/reordered.
Written back: `P`, `orient`, `v`, `w` on dynamic points (and packed primitive transform), and
`bullet_autofit_valid = 1` where a shape was (re)built while it was 0. See "Mid-simulation updates" for which
attributes an existing body follows.

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fell back to box (flat / <4
points / too complex / empty packed geometry / convex without packed prim); points that could
not become a body; duplicate `id`; non-finite `P`; Geometry not writable; points with no Box3D body
at write-back. Fallback warnings only appear on the frame the body is created, or on a step that rebuilt its shape
(including "the rebuilt collision shape could not be created").

## Debug logging (`Box3DDebug.h`)

Anything that can happen EVERY step (sync skipped / full scan lines, external edits, unmatched bodies, objects
without Geometry) uses `B3D_LOG_VERBOSE` / `B3D_LOG_IF(cond, ...)` and only prints with `BOX3DDOP_VERBOSE=1`: a flushed
stderr line per frame slows Houdini a lot. Keep new per-step diagnostics on those macros; plain `B3D_LOG` is for one-off
events (world rebuild, body creation notes, bodies added / removed).

`B3D_LOG`, `B3D_COUNT`, `B3D_UNCOUNT`, `B3D_COUNTER`, `B3D_ENV_INT` compile to no-ops unless
`BOX3DDOP_DEBUG`. Output goes to stderr with a `[b3dop]` prefix, mutex-guarded. Typical lines:
`REBUILD #n: <reason>`, `sync: +A added, -R removed, ...`, per-body fallback notes. The only counter is `rebuilds`.
Note `B3D_LOG` arguments may be unused in non-debug builds; the code uses `(void)x` for those.

## Tests

Build with `-DBOX3DDOP_CORE_ONLY=ON` (or as part of the normal build; all three are always built):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Prints Box3D version, create/destroy; expects `OK` |
| `b3dop_core_physics` | `sanitizeQuat` / `storedPosition` (the write-back's mirror prediction), World/body API, handle API equivalence, attribute->shape mapping, explicit primitive overrides (Euler order, rotated capsule / box resting heights), mass->density, body type switching, capsule/sphere/box resting heights, hull creation/limits/caching/instancing, packed-matrix math (quaternion<->matrix round trips, `composeLocal3`, row-vector convention), offset hulls, compounds under real physics |
| `b3dop_core_threads [workers=4] [side=12] [steps=120]` | Same scene on 1 vs N workers, reports speedup, fails if positions differ by > 1e-4 |

Houdini-side code (`SIM_*`) has no automated tests; `tests/box3d_test.hiplc` is a manual scene.
There is no CI configuration in this repo.

## Risky HDK spots

Written against the HDK docs, and earlier project notes flagged these as the places a compile or
behaviour problem is most likely after an HDK upgrade:

* Packed prim API in `SIM_SolverBox3D.cpp`, hardcoded to the HDK class reference signatures
  (no SFINAE shims; a mismatch is a compile error, not a silent fallback):
  `static bool GU_PrimPacked::isPackedPrimitive(const GA_Primitive &)`,
  `void GU_PrimPacked::getFullTransform4(UT_Matrix4D &) const`,
  `void GEO_PrimPacked::setLocalTransform(const UT_Matrix3D &)`,
  `void GEO_PrimPacked::getPivot(UT_Vector3 &) const`, `void GU_PrimPacked::transformDirty()`.
  Still unchecked: `getPackedDetail()`,
  fragment calls (`GU_PackedFragment` via `sharedImplementation()`, `getPrimitiveRange()`,
  `name()`, `isIntValue()`, `intValue()`), and `GU_DetailHandleAutoReadLock`.
* `gdpPtr()` overloads exist because `getGdp()` returns a pointer or a reference depending on
  the lock type.
* DOP semantics: `SIM_DATA_CREATE` flags in `getOrMakeWritableGeometry`, and the
  timestamp semantics behind `canContinue`.

## Known gaps / TODO

* Triangle-mesh (static) and height-field shapes; automatic convex decomposition.
* `cylinder` / `plane` primitives (Box3D has no shape for them), and Houdini's `bullet_primR`
  units / `bullet_length` meaning are assumed, not verified (see "Explicit primitives").
* Kinematic bodies for animated passive points (`active = 0` is static for now; kinematic only
  arises from the Box3D Data body type and is not written back).
* Live changes to shape/material/size/mass attributes and packed-geometry changes on a live body.
* Gravity / substep / sleep changes without a rebuild (`b3World_SetGravity`,
  `b3World_EnableSleeping`).
* Constraints/joints (would be a second input on a SOP solver, or DOP constraint data).
* Events (contacts, sensors, sleep): `publishEvents` is an empty stub.
* `newobjects` handling: objects appearing this step are solved immediately rather than
  keeping their initial conditions (see the TODO in `syncFromGeometry`).
* Cylinder and plane shape names: Box3D has no equivalent, they fall back to the default shape.

## Conventions and gotchas for editing

* Keep Box3D headers out of everything except `Box3DWorld.cpp`.
* Keep `UT/UT_DSOVersion.h` in exactly one file (`SIM_Box3DRegister.cpp`).
* New `SIM_Data` classes must be registered in `initializeSIM`.
* Anything stored per step in the world must be reconstructible from point attributes, or a
  scrub/reload will lose it.
* Hash keys must always imply identical body-local points (`createHullWithKey` trusts them).
* Box3D asserts on non-unit quaternions and negative material values; `sanitizeQuat` and the PRM
  ranges exist for that reason. Any new input path should sanitize similarly.
* Row-vector matrices everywhere on the Houdini side; do not mix with column-vector math.
* Style: Allman braces for functions with the return type on its own line (HDK sample style),
  4-space indent, aligned declarations.
