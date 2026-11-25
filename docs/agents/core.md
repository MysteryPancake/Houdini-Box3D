# Core layer

The Houdini-free layer, `b3dop::Box3DWorld`: the API, hull cache and world settings. Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## Core layer: `b3dop::Box3DWorld`

**Conventions.** Positions `double[3]`; rotations `float[4]` x,y,z,w (the layout of `b3Quat` and `UT_QuaternionF`);
velocities `float[3]`. `BodyHandle` = a `b3BodyId` packed into `uint64_t` (never 0 when valid). Never use a stale
handle; the solver keeps handles in the mirror and erases entries with their bodies.

**Handles only.** `createBody`, `createShape` / `createHullShape` (taking the caller's `BodySettings`), `destroyBody`,
`setTypeByHandle` / `typeByHandle` and the `*ByHandle` calls touch no hash map. Caller-chosen ids (`addBody`,
`addShape`, `removeBody`, ...) live in `tests/KeyedWorld.h`, a test-only subclass built on the public handle API.
`BodySettings` is the type, the start-awake flag and the solver-only shape options around a `BodyProps`: the settings
that can change on a live body (material, damping, sleep, filter, ...), with an exact `operator==` so a caller
can compare two and call only the setters for what differs.

**Mass.** A shape add recomputes the mass unless `updateMass` is false. The solver always passes false and then calls
`applyMassByHandle` once (required before the next step): it sets every shape's density, applies the mass from the
shapes and, for dynamic bodies, can move the centre of mass to the body origin (parallel axis theorem) and scale the
inertia (`b3Body_SetMassData`; skipped when already centred with scale 1). A custom inertia
(`BodyProps::inverseInertia`, the last argument; whole objects pass the inertia of their bounding box,
`boundsInverseInertia`) replaces the computed inertia of a dynamic body, about the centre of mass at the origin,
before the scale; `invertSymmetric3` (`Box3DMath.h`) refuses a tensor that is not positive definite, and then the call
returns false and the computed inertia stays. `getMassDataByHandle` can return the off-diagonal terms too.
`setTypeByHandle` recomputes from the shapes, so the caller re-applies after a type change.

**Lazy poses.** A body parked in a compound (disabled, placed by its compound) is made lazy with `setLazyPose`:
`invalidateLazyPoses` (O(1), once a step) makes every one stale, and the first read of a stale body through the API
(`getTransformByHandle`, `bodyStateByHandle`, `extentByHandle`, `localToWorld`, `jointWorldFrames`, `jointFeedback`,
`createJoint`, enabling it) calls the refresher the host set (`setPoseRefresher`), which sets the transform. A refresh
is serialized and stores its epoch only once the transform is set, since several threads can read one body (test
`lazy_pose`). `jointWorldFramesAt` gives a joint's frames from poses the caller already has.

**Changes to existing bodies.** `setMaterialByHandle` (every shape; Box3D re-mixes contact friction / restitution
every step, so touching contacts follow), `setDampingByHandle`, `setGravityScaleByHandle`, `setSleepEnabledByHandle`
(the world switch applies on top; disabling wakes the body), `bodyStateByHandle` (awake, mass, centre of mass and
velocities in one call) / `applyForceByHandle` (external forces, per step, see below), `setSleepThresholdByHandle`
(Box3D's single threshold on `|v| + |w * maxExtent|`; negative = default), `setBulletByHandle`, `setEnabledByHandle`
(`b3Body_Disable`: no motion, no collisions, shapes kept), `destroyShapesByHandle` (releases the body's hulls; the
caller re-adds shapes and applies the mass), `wakeByHandle`, `setTargetTransformByHandle` (kinematic: Box3D sets the
velocity that reaches the pose in `timeStep`; it remains until cleared) and `getContactsByHandle` (one `ContactPoint`
per manifold point with `totalNormalImpulse > 0`; position = A's world centre of mass + `anchorA`; normal flipped to
point from this body into the other). `getContactImpulseEvents` gives the same points for every awake contact at once
(the fork's impulse events, on with `enableContactImpulseEvents`); `getContactImpulseSums` gives them as one
`ContactImpulseSum` per body pair (the points of a contact are adjacent, so runs of the same two bodies are added).
`contactImpulsesByShape` sums the same impulses per shape in creation order (Box3D lists a body's shapes newest first,
so it reverses them); a compound's pieces are its shapes, copied with `copyShapesByHandle`. They share a
`thread_local` shape-id scratch and are called serially. Box3D accepts all of them on a disabled body (proxies are
only touched when they exist).

**World settings.** `WorldSettings` (Box3D's defaults, except standard gravity): gravity, `subSteps`, `workerCount`,
`enableSleep`, `enableContinuous`, `contactStiffness` (Hz), `contactDampingRatio`, `contactPushSpeed`,
`restitutionThreshold`, `maxLinearSpeed`, `contactRecycleDistance` (and those in the section below). `applyTuning`
pushes changes into the live world (`b3World_SetGravity`, `EnableSleeping`, `EnableContinuous`, `SetContactTuning`,
`SetRestitutionThreshold`, `SetMaximumLinearSpeed`, `SetContactRecycleDistance`, ...); `workerCount` (parallel tasks
per step) is fixed at creation and needs a new world. Box3D also limits the contact stiffness to 1/8 of the pass rate
(`physics_world.c`: fps x Substeps x Constraint Iterations, where Constraint Iterations is the `subSteps` setting
here), so 24 fps with 1 pass runs at 3 Hz whatever `contactStiffness` says (the `substeps_fps` test expects a stack to
sink there).

**Collision filtering.** Box3D's own `b3Filter`, nothing custom. `CollisionFilter` (`Box3DWorld.h`: category, mask,
group; defaults every bit / every bit / 0, Box3D's `B3_DEFAULT_*_BITS`) lives in `BodyProps::filter`; `shapeDefFor`
puts it on every shape (`toB3Filter`) and Box3D checks pairs inline in its broad phase. `setFilterByHandle` calls
`b3Shape_SetFilter(..., invokeContacts = true)` on each shape, which resets the proxy (dropping contacts that no longer
pass) and is a no-op for an unchanged filter. No custom filter callback, no shape user data.

**Shapes and hull cache.** A box is a Box3D hull (`b3MakeBoxHull` / `b3MakeOffsetBoxHull`; `b3MakeTransformedBoxHull`
when rotated); sphere and capsule (along Y; Box3D turns a too-short one into a sphere) are native. `bodyShapeOutlines`
reads a body's shapes back in body space (hull vertices and face loops, sphere, capsule) for the solver's guide
geometry. `createHull` keys by content (`hashFloats` of the points; hashing mixes 8 bytes per round); packed instances
pass their own key (`findHull` first, build on a miss). Builds try vertex budgets
`{B3_MAX_HULL_VERTICES, 44, 32, 16, 8}` (a triangulated hull hits the 256 half-edge limit at ~44 vertices).
`createHullWithKey`'s `budgetGroup` (the solver passes the source hash and kind) remembers the budget that worked, so
later hulls of the group start there. Flat, thin or non-finite results are rejected (`b3CreateHull` returns NULL for a
hull thinner than the larger of `B3_LINEAR_SLOP` and 0.1% of its largest extent) and the caller falls back to a box.
Several hulls on one body make a dynamic compound; overlapping pieces count their overlap twice in the mass.

The hull cache is **process-wide** (`HullLibrary` in Box3DWorld.cpp, mutex-protected): Box3D adds a hull to the
world's own database (`b3AddHullToDatabase`) when a shape is made: compared by content, so identical hulls share one
reference-counted clone and only a new hull is cloned. Our entries are only needed until then and can be shared by
every world. A rebuilt world (scrub, re-solve, thread change) therefore finds every hull its predecessor built.
Entries are `shared_ptr`s; a world holds the ones it found or built (`Impl::held`) until `pruneHulls()`, which the
solver calls at the end of each sync, so eviction never frees a hull between `findHull` and `createHullShape`. Past
the byte budget (`setSharedHullBudget`; the default is in `HullLibrary`, `Box3DWorld.cpp`) the least recently used
entries are evicted down to 3/4 of it. `findHull`, `createHull` and `createHullWithKey` are thread-safe (the solver's
shape pre-pass); nothing else is. `hullCacheSize()` is the hulls a world holds; `sharedHullBytes`,
`hullBuildCount/Attempts` report on the cache.

Packed sources also get a **source summary** (`SourceSummaryCache` in Box3DShapeBuild.cpp, process-wide): the group
and piece hashes, keyed by the source detail's identity (`geometryIdentity`: `getUniqueId`, `P` and primitive-list
data ids, counts; the primitive `name` attribute's data id, which groups the pieces; fragment, connected or not).
`buildShapes` tries it first for packed convex / concave points: when every hull key it gives is still cached, the
source geometry is not read or hashed at all. This relies on whoever edits a source bumping its data ids.

**Meshes and height fields.** `createMeshShape` works on static or kinematic bodies and `createHeightFieldShape` on
static ones only (Box3D's rule, plus the fork's mesh contact fix for a moving mesh), and both validate their input
(indices, finiteness, positive scale, at least 2 x 2 heights). Box3D keeps pointers to `b3MeshData` /
`b3HeightFieldData`, so `Impl::bodyGeometry` owns them until the body, its shapes or the world go. Meshes are shared
per world by content (`Impl::sharedMeshes`, keyed by a hash of vertices, indices and weld; a count of users frees one
with its last body; `meshCount()`), so instances of one piece and identical affectors build and hold one mesh; each
shape keeps its own scale. A mesh's scale changes in place (`setMeshScaleByHandle`); a height field's cannot. Height
fields are quantised to 16 bits over their own height range; `holes` become `B3_HEIGHT_FIELD_HOLE` cells.

**Ground.** `ShapeType::Plane` (a Ground Plane DOP, a static `plane` piece) is a box, `groundBoxSpec()` in
`Box3DShapes.h`: `GroundSize` (x, depth, z), its top face the plane y = 0 of the static body that holds it (a dynamic
body is refused). `groundSpec(size)` is a `ShapeSpec` of type Plane whose half extents carry the clamped size
(`kMinGroundSize`, `kMaxGroundSize`; a non-finite value is the default), so a new size is a new shape.
`WorldSettings::ground` holds it (the solver's Ground Box Size, applied each step with the rest of the tuning, read
when a ground is built). It is an ordinary hull, so it has Box3D's hull contacts and continuous collision; nothing
collides past its edge. The fork has no plane shape (a hull sometimes clipped through it and stuck). Covered by
`tests/core_ground.cpp`.

**`Box3DShapes.h`** (all unit-tested): `parseShapeType` (exact lowercase Houdini names; `compound` = `concave`;
`plane` is `ShapeType::Plane`, the ground box (static bodies only: `buildShapes` falls back to a box with a note,
whole objects ignore it); `cylinder` is `ShapeType::Cylinder`, a `ShapeSpec` like a capsule (`radius`, `halfLength` =
half the height) built by `Box3DWorld::cylinderShape` as a `b3CreateCylinder` hull (`kCylinderSides`), moved and
turned by re-hulling its points, cached under a key of size and pose; unknown values return false; the solver memoises
per shared-string handle in `GeorepLookup`); `makeShapeSpec` (`pscale * scale` as a bounding box, components made
positive and >= `kMinExtent` (`Box3DShapes.h`) by `clampExtent`; sphere diameter = largest component; capsule radius =
half of max(x, z)); `PrimOverrides` / `applyPrimOverrides`; `matrixRowLengths`; `packedPointsToBodyLocal`
(`(p - pivot) * S`); `packedScale` (row lengths rounded to 5 significant digits, so rotated instances share a hull;
~1e-5 scale error; the third entry is negative for a mirroring 3x3, which mirrors the hull points and keys a mirrored
piece apart; `keepMirror` puts the mirror back on a 3x3 written from a rotation); `packedHullKey` (kind 1 merged, 2
per piece); `connectedSets` (union-find); `fitShapeInFrame` / `fitShapeToPoints` / `canonicalFrame` (and
`Box3DWorld::fitShapeOriented`, which needs a Box3D hull); hashing (`kHashSeed`, `mixHash`, `hashFloats`).

**`Box3DMath.h`** (no dependencies; included by `Box3DShapes.h` and `Box3DJoints.h`): `kDegToRad`, `finite3`, `copy3` /
`copy4`, `quatsDiffer`, `xformPoint`, `sanitizeQuat` and `storedPosition` (shared by the read path and the
write-back's prediction).

## World settings added from the Box3D world API

Bounce iterations / propagation, isotropic inertia tolerance, warm starting, hit event threshold and the friction /
restitution mixing rules are `WorldSettings` fields, passed to `b3WorldDef` at creation and changed live in
`applyTuning`. Isotropic tolerance only affects new bodies or bodies whose mass changes (Box3D's rule). The mixing
callbacks get no context, so each rule is its own plain function chosen by `frictionRule` / `restitutionRule`;
0 passes nullptr (Box3D's own rule). Shapes are always made with hit events on. With `impactMinSpeed > 0`,
`recordImpacts` keeps only contact points whose body pair appears in the step's hit events (`updateHitEvents`,
`hadHitEvent`); 0 keeps every contact. `tests/core_physics.cpp` covers the core side.
