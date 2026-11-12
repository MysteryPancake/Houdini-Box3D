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

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism (see
"HDK calls").

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short. Leave out anything a user cannot act on or would
not notice: how the solver does something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics, unverified HDK details. Those belong here. When a change adds a feature, add one line or table
row to the README if users need it, and put the explanation here.

## Layout

```
CMakeLists.txt        Targets: b3dop_core, 3 test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
extern/box3d/         Box3D submodule (never edit; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg     Node icon; the build copies it to config/Icons as DOP_box3d_solver.svg and DOP_box3d_data.svg
src/
  Box3DShapes.h       Houdini-free, header-only: shape math, packed-prim matrices, hashing, collision-group patterns
  Box3DWorld.h/.cpp   Houdini-free pimpl wrapper over one b3World. The ONLY file that includes Box3D headers
  Box3DDebug.h        B3D_LOG / counters, compiled out unless BOX3DDOP_DEBUG
  SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache)
  SIM_Box3DData.*     Default-settings DOP node
  SIM_SolverBox3D.*   The solver: sync, step, write-back, post-step passes (the bulk of the plugin)
  SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
tests/
  core_smoke.cpp      Version print + world create/destroy
  core_physics.cpp    Unit + physics checks (CHECK macro, nonzero exit on failure)
  core_threads.cpp    1 worker vs N workers: timing + determinism
  box3d_test.hiplc    Houdini scene for manual testing (tests/backup/ holds older saves)
```

Layering (keep it):

1. `Box3DShapes.h` and `Box3DWorld.*` know nothing about Houdini, so the tests run without Houdini.
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
* `BOX3DDOP_DEBUG` (default ON) defines `BOX3DDOP_DEBUG=1` for the plugin target only.
* `BOX3D_DIR` (default `extern/box3d`) must point at Box3D's own `CMakeLists.txt`.
* `-DBOX3D_DOUBLE_PRECISION=ON` works: positions cross the boundary as `double[3]` and `Box3DWorld.cpp` converts through
  `decltype(b3Pos::x)`.
* Release or RelWithDebInfo only.
* `build.sh` wipes `build/` if the repo moved (`build/.configured_from`). `build.bat` finds `HFS` itself (env var, then
  registry `HKLM\SOFTWARE\Side Effects Software` ActiveVersion -> `Houdini <ver>\InstallPath`, then the newest
  `Program Files\Side Effects Software\Houdini X.Y.Z`). It still names the VS 2022 generator.

## Core layer: `b3dop::Box3DWorld`

**Conventions.** Positions `double[3]`; rotations `float[4]` x,y,z,w (the layout of `b3Quat` and `UT_QuaternionF`);
velocities `float[3]`. `BodyHandle` = a `b3BodyId` packed into `uint64_t` (never 0 when valid). Never use a stale
handle; the solver keeps handles in the mirror and erases entries with their bodies.

**Two APIs.** The handle API (`createBody`, `createShape` / `createHullShape` taking the caller's `BodySettings`,
`destroyBody`, `setTypeByHandle`, `*ByHandle`) touches no hash map and is what the solver uses. The keyed API (`addBody`,
`addShape`, `removeBody`, ...) is a convenience layer for the tests; bodies made with `createBody` are invisible to it.

**Mass.** A shape add recomputes the mass unless `updateMass` is false. The solver always passes false and then calls
`applyMassByHandle` once (required before the next step): it sets every shape's density, applies the mass from the
shapes and, for dynamic bodies, can move the centre of mass to the body origin (parallel axis theorem) and scale the
inertia (`b3Body_SetMassData`; skipped when already centred with scale 1). `setTypeByHandle` recomputes from the shapes,
so the caller re-applies after a type change.

**Changes to existing bodies.** `setMaterialByHandle` (every shape; Box3D re-mixes contact friction / restitution every
step, so touching contacts follow), `setDampingByHandle`, `setGravityScaleByHandle`, `setSleepEnabledByHandle` (the
world switch applies on top; disabling wakes the body), `setSleepThresholdByHandle` (Box3D's single threshold on
`|v| + |w * maxExtent|`; negative = default), `setBulletByHandle`, `setDensityByHandle`, `setEnabledByHandle`
(`b3Body_Disable`: no motion, no collisions, shapes kept), `destroyShapesByHandle` (releases the body's hulls; the
caller re-adds shapes and applies the mass), `wakeByHandle`, `setTargetTransformByHandle` (kinematic: Box3D sets the
velocity that reaches the pose in `timeStep`; it remains until cleared) and `getContactsByHandle` (one `ContactPoint`
per manifold point with `totalNormalImpulse > 0`; position = A's world centre of mass + `anchorA`; normal flipped to
point from this body into the other). They share a `thread_local` shape-id scratch and are called serially. Box3D
accepts all of them on a disabled body (proxies are only touched when they exist).

**World settings.** `WorldSettings` (Box3D's defaults, except standard gravity): gravity, `subSteps`, `workerCount`,
`enableSleep`, `enableContinuous`, `enableSpeculative`, `contactHertz`, `contactDampingRatio`, `contactPushSpeed`,
`restitutionThreshold`, `maxLinearSpeed`. `applyTuning` pushes changes into the live world (`b3World_SetGravity`,
`EnableSleeping`, `EnableContinuous`, `EnableSpeculative`, `SetContactTuning`, `SetRestitutionThreshold`,
`SetMaximumLinearSpeed`); `workerCount` sizes Box3D's own thread pool at creation and needs a new world.

**Collision filtering.** `CollisionFilterTable` (`Box3DShapes.h`) holds every distinct `FilterProfile` (group, object
name, object id as text, ignore patterns) and answers `shouldCollide(a, b)`: neither profile's ignore patterns match the
other's names. Patterns are matched by `patternMatcher()`: in Houdini `UT_String::multiMatch` (installed in
`initializeSIM`; globs separated by spaces or commas, `^pat` removes), in the tests the Houdini-free stand-in
`matchPatternList` / `globMatch`. `finish()` precomputes an N x N table (N <= 2048, else matched on the fly), so lookups are
read-only and safe from Box3D's threaded pair finding; the table is only changed during the sync. Every shape is created
with `enableCustomFiltering = true` (Box3D cannot turn it on later) and its body's profile + 1 in its user data; the
world registers `b3World_SetCustomFilterCallback`. The filter only runs on new pairs, so `setFilterProfileByHandle` also
drops the shape's contacts: `b3Shape_SetFilter(..., invokeContacts)` does that but returns early on an unchanged filter,
so a temporary `groupIndex` is set first.

**Shapes and hull cache.** A box is a Box3D hull (`b3MakeBoxHull` / `b3MakeOffsetBoxHull`; a rotated box is the hull of
its 8 corners); sphere and capsule (along Y; Box3D turns a too-short one into a sphere) are native. `createHull` keys by
content (FNV-1a of the points); packed instances pass their own key (`findHull` first, build on a miss). Builds try
vertex budgets `{B3_MAX_HULL_VERTICES, 64, 44, 32, 16, 8}` (a triangulated hull hits the 128-edge limit at ~44
vertices); flat or non-finite results are rejected and the caller falls back to a box. We own the `b3HullData` (Box3D
clones it per shape); `refs` count shapes, hull use is tracked per handle, and `pruneHulls()` frees unused entries.
Several hulls on one body make a dynamic compound; overlapping pieces count their overlap twice in the mass.

**`Box3DShapes.h`** (all unit-tested): `parseShapeType` / `ShapeTypeCache` (exact lowercase Houdini names; `compound`
= `concave`; unknown values, including `cylinder` and `plane`, return false); `makeShapeSpec` (`pscale * scale` as a
bounding box, components made positive and >= 1e-4; sphere diameter = largest component; capsule radius = half of
max(x, z)); `PrimOverrides` / `applyPrimOverrides`; `matrixRowLengths`; `packedPointsToBodyLocal` (`(p - pivot) * S`);
`packedScale` (row lengths rounded to 5 significant digits, so rotated instances share a hull; ~1e-5 scale error);
`packedHullKey` (kind 1 merged, 2 per piece); `connectedSets` (union-find); `fitShapeToPoints`; `sanitizeQuat` /
`storedPosition` (shared by the read path and the write-back's prediction); pattern matching and
`CollisionFilterTable`.

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
`dynamic`, `type`, `handle`, the point (`pointOff`, `pointId`, `pointIndex`, `objectId`), `stamp`, the packed primitive
and its 3x3, and what the body was built from (`props`, `volume`, `shapeSig`, `shapeless`, `autoScale`, `sizeKey`,
`animated`, `driven`, `sleeping`). Box3D's body user data points at the entry, so entries live in an `unordered_map`
(stable nodes) and are erased only with their body.

Scratch reused every step: `slots` (per object: offset, key, cached entry and freshly read state per point; a cached
entry is trusted while offset and key match; all slots are dropped when an entry is erased), `writeItems`,
`packedWrites`, `moves`, `contacts`. `syncStamp` / `stampedCount`: equal counts mean nothing to remove.
`isLive(entry)` = its object was visited by this sync and the entry carries the object's `liveStamp`.

## Solver step (`solveObjectsSubclass`)

```
worldData = findOrCreateWorldData(objects)     first object carrying "Box3DWorld", else created on objects(0)
rebuild if invalid / thread count changed / stamp mismatch -> createFreshWorld
lock st->mutex
stepDt = dt * timescale; world.applyTuning(currentWorldSettings())
syncFromGeometry   per object syncObject(); finishFilterProfiles; remove gone bodies; pruneHulls; warnings
stepWorld          b3World_Step(stepDt, subSteps)
writeResultsBack   awake dynamic bodies (move events): P / orient / v / w / packed transform / bullet_sleeping
recordImpacts      Add Impact Data -> SIM_Impacts (only on steps with an impact)
activateByImpact   min_activation_impulse
writePointAttrs    queued point writes: bullet_autofit_valid = 1, computed mass, active = 1
ageObjects         Enable Aging
deleteDeadPoints   `dead` points
stamp = now
```

Data names: `"Box3DWorld"`, `"Box3DData"`, `SIM_GEOMETRY_DATANAME`.

### syncObject (per object)

1. Read-locks the `SIM_Geometry` (objects without one are skipped). `PointAttrs` binds `id, active, v, w, orient`.
2. Box3D Data: body type and `nodeHash` (its values that change existing bodies, Add Impact Data included).
3. **Skip check** (see below). On a skip, the object's entries are counted and nothing per point runs.
4. Otherwise binds the remaining attributes, reads the full node settings and registers the object's filter profile.
5. **Pass 1** (parallel above 8192 points, 2048 with primitives): per point body id (`id`, else the point number = its
   position in the walk), key, `getPackedInfo`, `readPointState`, `active`.
6. **Pass 2** (serial, point order): duplicate `id` -> counted, first wins; non-finite `P` -> counted, its body removed;
   `dead` -> queued for deletion, its body removed; unknown key -> `addPointBody`; known key -> mid-simulation updates
   (shape rebuild, then settings), type change (`wantedType`), then the animated target or an external edit
   (`describeStateDifference`, eps 1e-5, q and -q equal -> `setTransformByHandle` + `setVelocityByHandle`).
7. Records the fingerprint; problems (failed bodies, duplicate ids, bad P), deforming bodies and driven animated bodies
   keep the object scanning.

### Skipping the sync scan

`Box3DObjectSync::fp` (`Box3DSyncFingerprint`): point count and offset range (plain points only; packed topology is
assumed fixed, a new instance gets a new `id`), data ids of `P`, `orient`, `v`, `w`, `id`, `active`, the node body type,
and the **prop markers**: data ids of `kPropAttribNames` (24 attributes that change settings or ask for rebuilds, looked
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

### Mid-simulation updates

Target behaviour is Houdini's Bullet solver. A new body is built from the same readers, so an untouched point compares
equal to what its body was built with.

* **Settings** (`readDynamicProps` -> `applyPropChanges`): the effective (attribute, else Box3D Data) values in
  `Box3DBodyProps`, compared with `entry->props`; only the setters for what differs run. Compared only when `checkProps`
  (prop markers changed, none recorded yet, or deforming bodies); a scan triggered by an animated `P` alone does not.
  * Mass, `computemass`, `density`, `inertialtensorstiffness` -> `applyBodyMass` (same as a fresh build).
  * Sleep: at creation `bullet_want_deactivate`, else `enablesleep`, else the node; afterwards only
    `bullet_want_deactivate` (enablesleep is creation only, as in Bullet).
  * `bullet_ignore` -> `setEnabledByHandle(false)`; a disabled body has no move events, so it is not written back.
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
  dynamic, activation and mass changes. Velocities are therefore the point's. No `computecom`: an off-centre pivot makes
  the piece turn about it, like in Houdini.
* **Collision groups** (`collisiongroup`, `collisionignore`): the object's profile (`obj.getName()`, id), or a point's
  own (`pointFilterProfile`), goes into `BodySettings::filterProfile`; changes use `setFilterProfileByHandle`. DOP groups
  are not matched.
* **Animated** (`animated` with `active` = 0): Kinematic. A pose edit becomes `setTargetTransformByHandle(cur, stepDt)`
  with `driven = true`; a step without one zeroes a driven body's velocity (so its object never skips while driven). Not
  written back; `v` / `w` are not applied.
* **Mass properties** (`computemass`, `density`, `inertialtensorstiffness`; node `computemass` / `density`):
  `boxDensity` = density with computemass, else mass / volume. With computemass the mass is written to `mass` (created
  if missing) on creation, rebuild or change.
* **Sleep** (`sleepthreshold`, node default 0.05): Box3D's single threshold and fixed half second (`B3_TIME_TO_SLEEP`).
  `bullet_sleeping`, if the attribute exists, is written by the write-back when a move event says the body fell asleep
  or it moves again (changes only; `entry->sleeping`). A user-written 0 wakes the body (checked in `checkProps` scans).
* **Fast object** (`fastobject`): Box3D's `isBullet`, renamed because Houdini's `isbullet` means "treat as a Bullet
  object". Changes mid-simulation.
* **Impacts** (node `addimpact`, Add Impact Data; no attribute): with it on, a scan lists every body in
  `Box3DObjectSync::impacts`. `recordImpacts` gathers their contact points, skipping asleep dynamic bodies and contacts
  where neither body is awake. Only when there is at least one does it create a fresh `SIM_Impacts`
  (`SIM_DATA_CREATE` with flags 0 replaces existing data) and call `addPointImpact(pos, normal, impulse, ptnum, otherobjid, otherptnum, -1, 0, 0,
  0, time, 0)` per point (normal of the surface hit, pointing at this body; ptnum = `entry->pointIndex`). An object
  with Impacts last step and none now loses the data (`removeNamedSubData`, `impactObjects`). No Feedback data (that is
  for mutual interaction with other solvers). UNVERIFIED: Houdini's impact normal convention.
* **Activation by impact** (`min_activation_impulse`, `activationignore`): inactive bodies with a threshold are listed
  in `Box3DObjectSync::activation`. After the step, a contact at or above it from a body whose profile does not match
  `activationignore` makes the body dynamic (mass re-applied, woken), queues `active = 1` and forces a scan. It moves
  from the next step; Bullet can activate within the step.
* **Dead** (`dead`): queued as a `Box3DDeletion`; `deleteDeadPoints` checks every queued point (offset, `id` or number)
  before deleting any (`destroyPrimitive(prim, true)` for packed, else `destroyPointOffset(off,
  GA_DESTROY_DEGENERATE)`), then `bumpAllDataIds()` and `forceScan`. Without `id`, later points become new bodies.
* **Aging** (solver Enable Aging): `age += dt` (DOP step, not time-scaled) on every point, `dead = 1` past a positive
  `life` (attributes created if missing). Writes every point every step while on.
* **Time Scale**: Box3D steps by `stepDt = dt * timescale`; animated targets use it; `canContinue` uses the DOP step.

**Point writes after the step** (`Box3DPointWrite`: `kAutofitValid`, `kMass`, `kActivate`): grouped per object
(`forEachObjectRun`), offsets via the templated `resolvePointOffsets`, then `recordFingerprintAfterWrite`.

### Building shapes (`addPointBody`, `planShapes`, `buildShapes`)

* Type: `wantedType` (active -> node type; inactive -> static, or kinematic if animated; shapeless -> static).
* `readDynamicProps` + `settingsFor` give the settings; `planShapes` the shape type (`bullet_georep`, else node
  `defaultShape`) and pieces; `addPlannedShapes` adds them; `applyBodyMass`; then the initial `v` / `w` (after the mass,
  because a moving centre of mass shifts the velocity; not for static or animated bodies); `bullet_ignore` disables.
* The mirror entry is inserted (`try_emplace`) before the body exists and becomes its user data; it is erased again if
  the body cannot be built. A body is never left without shapes, except `none`. 64+ new bodies in one scan reserve the
  mirror once.
* `buildShapes`:
  1. Packed point: its geometry into `PieceGroup`s via `ShapeSources` (cached per source detail, and per
     `(source, first primitive)` for packed fragments, for one `syncObject`).
  2. Packed convex / concave: `packedHullKey`, `findHull` first, body-local points only on a miss. Convex merges all
     pieces (kind 1) unless hull-per-connected; concave is one hull per piece (kind 2).
  3. Otherwise body-local point sets -> `pieceFromPoints` (hull, or a fitted box with a note).
  4. Box / sphere / capsule on a packed point with autofit: fitted to the bounds.
  5. Nothing usable (plain point, autofit off, empty geometry): `makeShapeSpec(pscale * scale)` plus
     `applyPrimOverrides`; convex / concave fall back to a box with a note.
* Notes are aggregated into one node warning per kind, on the step a shape is built.

**Explicit primitives** (Bullet Data with AutoFit off; `bullet_autofit` attribute, else the node toggle, default on):
for `box`, `sphere`, `capsule` only. `bullet_primS` = box HALF extents (Bullet Data documents Box Size as a half-size);
`bullet_radius`; `bullet_length` = capsule straight section, caps not included (ASSUMED); `bullet_primT` = offset;
`bullet_primR` = Euler degrees, XYZ (ASSUMED), turned into a quaternion with
`UT_QuaternionF(rad, UT_XformOrder(TRS, XYZ))`. A rotated box is an 8-corner hull; a sphere ignores rotation. If the
assumptions are wrong, only that conversion and the `hasLength` line of `applyPrimOverrides` change.

**Pieces and packed geometry.** Pieces are grouped by the primitive `name` attribute (unnamed share one; no primitives:
one piece). `hullperconnected` (convex only): one hull per connected set, ignoring `name`. Packed fragments
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

0. Move events (`getMoveEvents`): every awake body and its transform (a body falling asleep appears once more). Only
   live dynamic entries become items; `bullet_sleeping` changes are noted. Velocities are fetched in a threaded pass
   (Box3D allows read-only calls from several threads). Items are grouped per object.
1. `getOrMakeWritableGeometry`: a plain `SIM_Geometry` (RBD Packed Object) is replaced once by a `SIM_GeometryCopy`
   (`SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE`).
2. `ensureResultAttribs` creates `v`, `w`, `orient` (a new `orient` is set to identity).
3. `resolvePointOffsets`: the stored offset if it still names the point (in range, same id / number), else by id.
4. Parallel pass: `hardenWrittenPages` first when parallel (offset ranges bucketed by 1024; one full call when nearly all
   buckets are hit), then write, compute the new packed 3x3, and store in the mirror what the geometry now holds
   without reading it back (`storedPosition`, `sanitizeQuat`; must match `readPointState`, or our write looks like an
   external edit). `BOX3DDOP_VERIFY_READBACK=1` re-reads and logs `READBACK MISMATCH`.
5. Serial pass: `setLocalTransform` + `transformDirty`, then one `getPrimitiveList().bumpDataId()` (the HDK leaves
   marking intrinsics dirty to the caller). A point no longer on a packed primitive is counted as unmatched.

Only dynamic bodies are written; of several points sharing an `id`, only the first.

### Threading

Box3D's pool is its own (`autothreads` = all cores, else `threads`, 1..32; a change rebuilds the world). The plugin's
loops use `UTparallelFor` with untuned thresholds; parallel lambdas only read shared data or write their own element.
The Box3D world and mirror are touched in serial passes (the write-back's parallel pass writes only its own entry). The
shared-state mutex is held for the whole step.

## Not supported on purpose, and planned

* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* PLANNED (not started): static triangle-mesh colliders for the concave case. Box3D meshes (`b3CreateMeshShape`) and
  height fields only collide as static, which is why `concave` / `compound` are one hull per piece (Bullet's concave
  is a triangle mesh and may move). Intended split: a static (inactive, non-animated) `concave` point becomes a Box3D
  mesh of its packed geometry; a dynamic one keeps the compound. Mind the mesh's own memory (not the hull cache),
  `deforming` rebuilds, and that a mesh body cannot become dynamic without rebuilding its shape.

## Parameters

Solver (read every step; only the thread count rebuilds the world): `gravity` (0,-9.80665,0), `substeps` (min 1,
default 4), `autothreads` (on), `threads` (1..32, default 8), `enablesleep`, `timescale` (1), `enableaging` (off), and
Box3D's tuning with Box3D's defaults: `contactstiffness` (30 Hz), `contactdamping` (10), `contactpushspeed` (3),
`bouncethreshold` (1), `maxlinearspeed` (400), `enablecontinuous` (on), `enablespeculative` (on).

Box3D Data (per object): `defaultshape` (0 box, 1 sphere, 2 convex hull, 3 concave, 4 capsule, 5 none), `autofit` (on),
`hullperconnected`, `bodytype` (0 static, 1 kinematic, 2 dynamic), `computemass` (off), `mass` (1), `density` (1000),
`friction` (0.6), `bounce` (0), `lineardamping`, `angulardamping`, `gravityscale` (1), `enablesleep`, `sleepthreshold`
(0.05), `fastobject` (off), `addimpact` (off). Names match the point attributes that override them; material values
have a hard minimum of 0 (Box3D asserts on negatives). `toBodySettings()` maps them to `b3dop::BodySettings`.

## Point attributes

`P` (required), `orient` (not for packed points), `v`, `w` (rad/s), `id`, `active`, `animated`, `mass`, `computemass`,
`density`, `inertialtensorstiffness`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`,
`enablesleep`, `bullet_want_deactivate`, `sleepthreshold`, `bullet_sleeping`, `fastobject`, `bullet_ignore`,
`collisiongroup`, `collisionignore`, `min_activation_impulse`, `activationignore`, `dead`, `age`, `life`,
`deforming`, `bullet_autofit_valid`, `bullet_georep`, `bullet_autofit`, `bullet_primS` / `R` / `T`, `bullet_radius`,
`bullet_length`, `pscale`, `scale`. Negative material values are clamped to 0, non-finite ones ignored (`gravityscale`
may be negative). Identity is `(object id, id or point number)`.

Written: `P`, `orient`, `v`, `w` and packed transforms (awake dynamic bodies), `bullet_sleeping` (if it exists),
`bullet_autofit_valid`, `mass` (computemass), `active` (activation), `age` / `dead` (aging); `dead` points are deleted.

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fallbacks (on the step a shape is built), points that could
not become a body, duplicate `id`, non-finite `P`, Geometry not writable, bodies not matched to a point at write-back.

## Debug logging (`Box3DDebug.h`)

Per-step diagnostics use `B3D_LOG_VERBOSE` / `B3D_LOG_IF(cond, ...)` and only print with `BOX3DDOP_VERBOSE=1` (a
flushed line per frame slows Houdini a lot); plain `B3D_LOG` is for one-off events (rebuilds, bodies added / removed /
rebuilt, fallback notes). All macros compile to nothing without `BOX3DDOP_DEBUG`; arguments used only by logs get
`(void)x`. Output: stderr, `[b3dop]` prefix. The only counter is `rebuilds`.

## Tests

Built in both configurations (`-DBOX3DDOP_CORE_ONLY=ON` builds only these):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Box3D version, create / destroy; expects `OK` |
| `b3dop_core_physics` | Keyed and handle APIs, shape mapping, explicit primitives, mass, body types, resting heights, hulls (limits, caching, instancing), packed matrix math, `sanitizeQuat` / `storedPosition`, setters on existing bodies (material, damping, gravity, sleep, density, enable, shape replacement), live tuning, patterns and collision groups (incl. re-filtering), contacts, mass centring / inertia scaling, sleep threshold, kinematic targets |
| `b3dop_core_threads [workers] [side] [steps]` | 1 vs N workers: speedup, fails if positions differ by > 1e-4 |

The `SIM_*` code has no automated tests and has not been compiled here against a real HDK; `tests/box3d_test.hiplc`
is a manual scene. No CI.

## HDK calls

Checked against the HDK reference: `GU_PrimPacked::isPackedPrimitive`, `getPackedDetail`, `sharedImplementation`,
`getFullTransform4`, `setLocalTransform`, `getPivot`, `transformDirty`; `GU_PackedFragment::getPrimitiveRange`, `detailPtr`;
`GU_DetailHandleAutoReadLock` / `SIM_GeometryAutoReadLock::getGdp` (const pointer), `SIM_GeometryAutoWriteLock::getGdp`
(reference); `GA_Detail::getPrimitiveList()` / `GA_PrimitiveList::getDataId()` / `bumpDataId()`,
`findPointAttribute` (const), `getP`, `getNumPointOffsets`, `getNumPrimitiveOffsets`, `pointVertex`,
`vertexPrimitive`, `getPointMap`, `pointIndex`, `pointOffset`, `destroyPointOffset` (`GA_DESTROY_DEGENERATE`),
`destroyPrimitive`, `bumpAllDataIds`; `GA_Attribute::getDataId` / `bumpDataId`; `GA_AIFSharedStringTuple::getHandle`
/ `getTableString`; `GEO_Detail::addIntTuple`; `SIM_DATA_CREATE` flags (`SIM_DATA_RETURN_EXISTING` unset = replace);
`SIM_Impacts::addPointImpact`, `SIM_IMPACTS_DATANAME`; `SIM_RootData::getName`, `removeNamedSubData`;
`UT_String::multiMatch`, `isstring`, `buffer`; `UT_Matrix3D::extractScales`, `UT_QuaternionT::getRotationMatrix` /
`updateFromRotationMatrix`; node icons as `config/Icons/DOP_<name>.svg`.

Still unchecked: the timestamp semantics behind `canContinue`, data ids surviving the DOP copy, the impact normal
convention, and Bullet's `bullet_primR` order and `bullet_length` meaning (Bullet docs, not HDK).

## Known gaps / TODO

* Constraints / joints (constraint networks); Box3D has the joints.
* Static triangle-mesh and height-field colliders (see "planned"); `cylinder` / `plane` (no Box3D shape; hull or big
  box approximations possible).
* `newobjects` are solved on their first step instead of keeping their initial state (TODO in `syncFromGeometry`).
* DOP groups in collision / activation patterns.
* Sticky collisions; contact / sensor events beyond Impacts.

## Conventions and gotchas

* Box3D headers only in `Box3DWorld.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data` classes
  registered in `initializeSIM`.
* Anything kept in the world must be reconstructible from point attributes, or a scrub / reload loses it.
* Hull keys must always imply identical body-local points (`createHullWithKey` trusts them).
* Sanitize every new input path: Box3D asserts on non-unit quaternions and negative material values.
* Row-vector matrices on the Houdini side; never mix with column-vector math.
* Style: Allman braces, return type on its own line (HDK sample style), 4-space indent, aligned declarations.
