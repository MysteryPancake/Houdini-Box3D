# Core layer

The Houdini-free layer, `b3dop::Box3DWorld`: the API, hull cache and world settings. Index and rules: [AGENTS.md](../../AGENTS.md).

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
world switch applies on top; disabling wakes the body), `getCenterOfMassByHandle` / `bodyStateByHandle` (awake, mass, centre of mass and velocities in one call) / `applyForceByHandle` (external forces, per step, see below), `setSleepThresholdByHandle` (Box3D's single threshold on
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
budget (`setSharedHullBudget`; the default is in `HullLibrary`, `Box3DWorld.cpp`) the least recently used entries are evicted down to 3/4 of it.
`findHull`, `createHull` and `createHullWithKey` are thread-safe (the solver's shape pre-pass); nothing else is.
`hullCacheSize()` is the hulls a world holds; `sharedHullCount/Bytes`, `hullBuildCount/Attempts` report on the cache.

Packed sources also get a **source summary** (`SourceSummaryCache` in Box3DShapeBuild.cpp, process-wide): the group and
piece hashes, keyed by the source detail's identity (`getUniqueId`, `P` and primitive-list data ids, counts, fragment,
connected or not). `buildShapes` tries it first for packed convex / concave points: when every hull key it gives is still
cached, the source geometry is not read or hashed at all. This relies on whoever edits a source bumping its data ids.

**`Box3DShapes.h`** (all unit-tested): `parseShapeType` (exact lowercase Houdini names; `compound`
= `concave`; unknown values, including `cylinder` and `plane`, return false; the solver memoises per shared-string handle
in `GeorepLookup`); `makeShapeSpec` (`pscale * scale` as a bounding box, components made positive and >= `kMinExtent`
(`Box3DShapes.h`) by `clampExtent`; sphere diameter = largest component; capsule radius = half of
max(x, z)); `PrimOverrides` / `applyPrimOverrides`; `matrixRowLengths`; `packedPointsToBodyLocal` (`(p - pivot) * S`);
`packedScale` (row lengths rounded to 5 significant digits, so rotated instances share a hull; ~1e-5 scale error);
`packedHullKey` (kind 1 merged, 2 per piece); `connectedSets` (union-find); `fitShapeToPoints`; `sanitizeQuat` /
`storedPosition` (shared by the read path and the write-back's prediction).

## World settings added from the Box3D world API

Bounce iterations / propagation, isotropic inertia tolerance, warm starting, hit event threshold and the friction /
restitution mixing rules are `WorldSettings` fields, passed to `b3WorldDef` at creation and changed live in
`applyTuning`. Isotropic tolerance only affects new bodies or bodies whose mass changes (Box3D's rule). The mixing
callbacks get no context, so each rule is its own plain function chosen by `frictionRule` / `restitutionRule`;
0 passes nullptr (Box3D's own rule). Shapes are always made with hit events on. With `impactMinSpeed > 0`,
`recordImpacts` keeps only contact points whose body pair appears in the step's hit events (`updateHitEvents`,
`hadHitEvent`); 0 keeps every contact. `tests/core_physics.cpp` covers the core side.
