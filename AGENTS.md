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

State (`Impl`): the `b3WorldId`, `WorldSettings`, `bodies` (key -> `b3BodyId`),
`bodySettings` (key -> `BodySettings`, remembered because shapes are added after the
body and need density/friction/restitution), the hull cache, and `bodyHulls`
(key -> hulls its shapes use).

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
  destroys entries with `refs <= 0`. The solver calls `pruneHulls()` once per sync.
  Entries are destroyed after the world in `destroy()`.
* Several hull shapes on one body = Box3D "runtime compound"; this is the only way to
  get a dynamic concave body (triangle meshes only collide as static).
* Mass: total hull volume is the sum over shapes, so overlapping pieces double-count.

`Box3DShapes.h` highlights (all unit-tested):

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
from Box3D (in DOP terms), `dynamic`, cached `handle`, cached `type`. It exists to
(a) detect edits by other nodes and (b) avoid hash lookups in the per-step loops.

## Solver step (`SIM_SolverBox3D::solveObjectsSubclass`)

```
objects.entries()==0 -> success
worldData = findOrCreateWorldData(objects)     // first object carrying "Box3DWorld", else create on objects(0)
decide reason to rebuild (invalid / thread count changed / stamp mismatch) -> createFreshWorld
lock st->mutex
syncFromGeometry   -> per object syncObject(); remove bodies whose point is gone; pruneHulls; emit warnings
stepWorld          -> b3World_Step(dt, subSteps)
writeResultsBack   -> only for objects that have >=1 dynamic point
st->stamp = now
publishEvents      -> TODO stub
```

Data names: world is attached as `"Box3DWorld"`, defaults as `"Box3DData"`; geometry is
`SIM_GEOMETRY_DATANAME`.

### syncObject (per object)

1. Gets `SIM_Geometry` (const) under `SIM_GeometryAutoReadLock`; skips objects with none.
2. Resolves `PointAttrs` (read handles for `id, active, mass, friction, bounce, pscale,
   scale, v, w, orient, bullet_georep`; any may be invalid = default).
3. Reads `SIM_Box3DData` fallback settings if attached.
4. **Pass 1 (parallel when >= 2048 points, `UTparallelFor`)**: per point compute body id
   (`id` attr else point number, which equals the index in a full point-range walk), key,
   `getPackedInfo`, `readPointState`, and `active`. Reads only.
5. **Pass 2 (serial, point order)**: duplicate `id` -> counted, first wins; non-finite `P`
   -> counted and treated as absent (existing body is removed later); unknown key ->
   `addPointBody` (also `readPackedPivot`); known key -> handle `active` flips
   (`setBodyType`) and external edits (`describeStateDifference` on pos/orient/lin/ang vs
   mirror with eps 1e-5, q and -q treated equal -> push `setTransformByHandle` +
   `setVelocityByHandle`, refresh mirror).

Edits that are **not** picked up on an existing body: `bullet_georep`, `pscale`, `scale`,
`mass`, `friction`, `bounce`, the damping / gravity scale / bullet / sleep attributes, Box3D
Data node settings (mass, default shape...), and solver gravity/substeps/sleep. These are applied
at body/world creation; a rebuild (Reset Simulation or thread-count change) re-reads them.
(`WorldSettings` is stored at creation and `step` uses the stored `subSteps`.)

### addPointBody / buildShapes

* Body type = node type, forced to Static if `active == 0`.
* `mass`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`,
  `enablesleep`, `isbullet` attributes override node values. Shape type = `bullet_georep` string
  via `parseShapeType`, else node `defaultShape`.
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
  4. Box/Sphere/Capsule on a packed point: fit to bounds of the packed geometry.
  5. Nothing usable: `makeShapeSpec(...)` box/sphere/capsule from `pscale*scale`.
     Convex/concave falls back to a box and sets a note ("needs a packed primitive" or
     "packed geometry is empty").
* Notes are logged and aggregated by text into `SyncCounts::fallbacks`, then surfaced as
  one warning per kind with a count on the first frame the body is created.
* Mass: `density = mass / totalVolume`, where `mass` is the point attribute if valid, positive
  and finite, otherwise the Box3D Data node's `mass`. If neither is usable (or the volume is 0)
  `BodySettings::density` keeps its default (1).
* After `addBody`, shapes are added; if none could be added the body is removed again
  (never leave a shapeless body). Initial `v`/`w` are applied last (no-op for static).

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

### writeResultsBack (per object with a dynamic point)

1. `getOrMakeWritableGeometry`: `SIM_DATA_GET<SIM_GeometryCopy>`; if the object's Geometry is
   a different `SIM_Geometry` subclass (e.g. from RBD Packed Object), replace with a
   `SIM_GeometryCopy` using `SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`.
   Fails with a warning if there is no Geometry.
2. `ensureResultAttribs` creates `v`, `w` (float3, vector type) and `orient` (float4,
   quaternion type) if missing. A freshly created `orient` is initialised to identity for
   all points (default 0,0,0,0 is not a rotation).
3. Pass 1 (serial): for each point with a dynamic mirror entry, fetch transform/velocity by
   handle. A missing body is counted and skipped.
4. Pass 2 (parallel): `hardenAllPages()` on `P/orient/v/w` first (copy-on-write pages would
   race), write attributes, compute the new packed matrix (not written yet), then re-read the
   point state so the mirror records what the geometry actually holds (P may be stored at lower
   precision than Box3D; without this, our own write would look like an external edit next step).
   `bumpDataId()` afterwards.
5. Pass 3 (serial): packed `setLocalTransform` + `transformDirty` (kept serial because the HDK
   does not document thread-safety).
6. Pass 4 (serial): update the mirror.

Static points are never written. Kinematic bodies are also not written back (the mirror's
`dynamic` flag is true only for `Type::Dynamic`).

### Threading

* Box3D's thread pool is its own (`workerCount`); `autothreads` = `hardware_concurrency()`,
  else the `threads` parm (1..32). Changing the count forces a world rebuild.
* The plugin's own parallel loops use TBB through `UTparallelFor` with a 2048-point threshold
  (`kMinParallelPoints`). Rule of thumb in the code: parallel lambdas only read shared data or
  write element `i` of their own output; the Box3D world, the mirror and `seen` are only touched
  in serial passes.
* The shared-state mutex is held for the whole sync/step/write-back.

## Parameters

Solver (`SIM_SolverBox3D`, all read when the world is created): `gravity` (v3, default
0,-9.80665,0), `substeps` (min 1, default 4, UI max 16), `autothreads` (default on),
`threads` (1..32, default 8, disabled when autothreads is on), `enablesleep`.

Box3D Data (`SIM_Box3DData`, per object, read when a body is created):
`defaultshape` (0 box, 1 sphere, 2 convex hull, 3 concave/compound, 4 capsule, 5 none),
`hullperconnected`, `bodytype` (0 static, 1 kinematic, 2 dynamic), `mass` (1, per body:
density = mass / shape volume), `friction` (0.6), `bounce` (0, = Box3D restitution),
`lineardamping`, `angulardamping`, `gravityscale` (1), `enablesleep`, `isbullet`. Parameter
names match the point attributes that override them. Box3D asserts on negative
mass/friction/bounce/damping, so these have hard minimum 0.
`SIM_Box3DData::toBodySettings()` maps to `b3dop::BodySettings` (`mass`, `restitution`, ...).
The Box3D Data node's data name on the object is `Box3DData`.

## Point attributes consumed (names follow Houdini RBD)

`P` required. `orient` (quat, ignored on input for packed points), `v`, `w` (rad/s), `mass`
(density = mass/volume), `friction`, `bounce` (restitution), `lineardamping`, `angulardamping`,
`gravityscale` (floats), `enablesleep`, `isbullet` (ints, 0 = off), `active` (0 static, else
dynamic), `bullet_georep` (shape string), `pscale`, `scale`, `id`. Each of `mass`, `friction`,
`bounce`, `lineardamping`, `angulardamping`, `gravityscale`, `enablesleep`, `isbullet`
overrides the Box3D Data parameter of the same name; negative / non-finite values are
clamped to 0 / ignored (gravityscale may be negative). Identity is
`(object id, id attribute or point number)`; use `id` if points can be deleted/reordered.
Written back: `P`, `orient`, `v`, `w` on dynamic points (and packed primitive transform).

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fell back to box (flat / <4
points / too complex / empty packed geometry / convex without packed prim); points that could
not become a body; duplicate `id`; non-finite `P`; Geometry not writable; points with no Box3D body
at write-back. Fallback warnings only appear on the frame the body is created.

## Debug logging (`Box3DDebug.h`)

`B3D_LOG`, `B3D_COUNT`, `B3D_UNCOUNT`, `B3D_COUNTER`, `B3D_ENV_INT` compile to no-ops unless
`BOX3DDOP_DEBUG`. Output goes to stderr with a `[b3dop]` prefix, mutex-guarded. Typical lines:
`REBUILD #n: <reason>`, `sync: +A added, -R removed, ...`, per-body fallback notes. The only counter is `rebuilds`.
Note `B3D_LOG` arguments may be unused in non-debug builds; the code uses `(void)x` for those.

## Tests

Build with `-DBOX3DDOP_CORE_ONLY=ON` (or as part of the normal build; all three are always built):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Prints Box3D version, create/destroy; expects `OK` |
| `b3dop_core_physics` | World/body API, handle API equivalence, attribute->shape mapping, mass->density, body type switching, capsule/sphere/box resting heights, hull creation/limits/caching/instancing, packed-matrix math (quaternion<->matrix round trips, `composeLocal3`, row-vector convention), offset hulls, compounds under real physics |
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
* `bullet_primR` / `bullet_primS` / `bullet_radius` / `bullet_length` overrides (primitives are
  always auto-fitted for packed points).
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
