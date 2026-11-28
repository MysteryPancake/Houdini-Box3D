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
`applyMassByHandle` once (required before the next step): it sets every shape's density (times the shape's own
`densityScale` from `createShape` / `createHullShape`, kept in its user data by `densityScaleTag`, so
`copyShapesByHandle` carries it), applies the mass from the
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
pass) and is a no-op for an unchanged filter. No custom filter callback. A shape's user data is only its density scale (`densityScaleTag`).

**Shapes, meshes, height fields, the ground and the helper headers** (`Box3DShapes.h`, `Box3DMath.h`):
[core-shapes.md](core-shapes.md).

## World settings added from the Box3D world API

Bounce iterations / propagation, isotropic inertia tolerance, warm starting, hit event threshold and the friction /
restitution mixing rules are `WorldSettings` fields, passed to `b3WorldDef` at creation and changed live in
`applyTuning`. Isotropic tolerance only affects new bodies or bodies whose mass changes (Box3D's rule). The mixing
callbacks get no context, so each rule is its own plain function chosen by `frictionRule` / `restitutionRule`;
0 passes nullptr (Box3D's own rule). Shapes are always made with hit events on. With `impactMinSpeed > 0`,
`recordImpacts` keeps only contact points whose body pair appears in the step's hit events (`updateHitEvents`,
`hadHitEvent`); 0 keeps every contact. `tests/core_physics.cpp` covers the core side.
