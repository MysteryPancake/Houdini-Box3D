# Forces and affectors

External forces (Force DOPs, point `force` / `torque`) and Static Object colliders. Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## External forces (`applyExternalForces`)

Runs after `syncFromGeometry` and the last cancel check, before the step, when `useforces` is on. Same lookup as
Houdini's `SIM_SolverHair`: `obj->forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"), SIM_FORCES_DATANAME,
SIM_DataFilterNone())` with a `SIM_EachDataCallback` (`ForceCollector`) that casts each sub-data to `SIM_Force`.

Only objects with force DOPs or non-zero point forces are visited, and only their own bodies
(`Box3DObjectSync::bodies`, rebuilt by each full scan and kept while the object is skipped): skip bodies that are not
`dynamic()`, are not `props.enabled`, or are asleep while `forceswake` is off. For each remaining body one
`bodyStateByHandle` call reads awake, mass, world centre of mass and `v` / `w` (it replaces four separate calls; with
only point forces and no Force DOP just `isAwakeByHandle` is read, and only when `forceswake` is off). Mass, centre of
mass and velocities go into `SIM_Force::getForce(object, pos, vel, angvel, mass, force, torque)` for every force on
the object, summed, and the sum goes in through `applyForceByHandle` (`b3Body_ApplyForceToCenter` +
`b3Body_ApplyTorque`).

* Box3D adds `invMass * force` and `invInertiaWorld * torque` in every substep's velocity integration
  (`b3IntegrateVelocitiesTask`) and zeroes both when the step finishes, so they are per-step: never carry them over, and
  a rebuilt world has none. `Box3DWorld` does not track them.
* Box3D ignores a force on a body that is not in the awake set. `wake = false` (the default) therefore skips sleeping
  bodies, which is what keeps gravity-like forces from holding resting bodies awake; `forceswake` passes `wake = true`.
* Non-finite sums are skipped and counted (`ForceStats::nonFinite`, one node warning). `applyForceByHandle` also
  sanitizes (`finiteVec3`) because Box3D asserts on non-finite vectors.
* Substeps (the solver's, whole Box3D steps) apply the forces again before each step, so a force is evaluated once per
  step. The point `force` / `torque` the first step read and cleared are applied again in the others
  (`PointForceCarry`), so the frame's force acts in every step. What stays constant is Box3D's own Constraint
  Iterations inside a step (`subStepCount`): no hook runs between them.
* Evaluation (`applyToObjectBodies`) is three phases per object: read each body's state (serial, it reads Box3D),
  evaluate the forces, apply the sums (serial). From `kMinParallelForces` bodies, the forces sampled at a point whose
  `SIM_ForceResolver` reports `threadsafe()` are evaluated on the TBB pool (`evaluateResolvers`: each range of 64
  bodies takes its own `copy()` of the resolvers, as the header asks). Everything else stays serial: forces sampled
  over a circle or sphere (the resolver only has the point call), and a custom Force DOP, whose generic resolver is
  not thread-safe. Below the threshold all forces are serial, as before. A debug build logs each force's verdict when
  it splits them (`force <type>: ... parallel` / `serial`), which `force_dops_threadsafe` reads
  (`BOX3DDOP_MIN_PARALLEL_FORCES` lowers the count). In Houdini 22.0.508 Gravity, Drag, Fan, Impulse, Point, Reference
  Frame and Uniform go to the pool, Magnet, Vex and Vortex do not (resolver not thread-safe), Buoyancy and Field are
  not sampled at a point.
* Gravity Force DOP + the solver's `gravity` add up (both are applied). `gravityscale` only scales the solver's gravity.
* Sampling: each force is sampled as its own Sample Mode asks (`SIM_Force::getOptimalForceSampling`, a point unless
  the Force DOP is set otherwise). Sphere: `getForceOnSphere` with the radius of a sphere of the same volume as the
  body's bounding box. Circle: the force's direction comes from a point sample, then `getForceOnCircle` with the
  radius of a circle of the same area as the box seen along that direction. The box is `Box3DWorld::extentByHandle`
  (world axes, so a turned body counts larger; read only when a force asks). A body with no shapes falls back to the
  point sample. Not used: `getPointForces`. RBD Bullet Solver ignores Sample Mode and samples every force at a point,
  so a force set to Circle or Sphere is stronger here than in Bullet (`bullet_force_sampling`).
* `useforces` / `forceswake` are plain options read each step; nothing is stored in the world, so they need no rebuild.

## Force and torque point attributes

`applyExternalForces` also reads per-point `force` / `torque` (V3) attributes from each object's geometry into a short
list of the non-zero ones (by offset; a body finds its point by binary search), adds them to the Force DOP result for
the same body, applies the sum, then zeroes the ones it read (a write lock, followed by `noteOwnWrite`, so the next
sync does not see the clear as an external edit). Zeroing is needed because nothing else clears `force` without a POP
Solver. The POP Force node must run before this solver. The attributes' data ids right after the clear (or after a
read found them all zero) are kept in `Box3DObjectSync::forceId / torqueId`; while they are unchanged nothing new was
written and the read is skipped.

A body that got a point force last step (`Box3DBodyMirror::pointForced`) and gets none now is woken once: a sleeping
body's joints read as they were, so it would keep what it read under the force. An object with such a body is visited
even when it has no forces (`Box3DObjectSync::pointForcedAny`). Not covered: a Force DOP that ends (a sleeping body is
not evaluated, so its end is not seen) and pieces of a compound.

## Affector static colliders

`Box3DAffectors.cpp` (`syncAffectorBodies`, after the objects, mutex held). Objects the solver is not given but that
affect a solved object through a collision relationship become static bodies. Each relationship the solved objects are
in is read once (its affector group, `gatherAffectors`): asking every object for its `getConstAffectors` walked the
relationship's whole group each time, which cost 60 ms a step for 2000 RBD Objects. They become static bodies: a
height field if the geometry has one, else a triangle mesh of its polygons.

* Gather: affectors of every solved object through its Collide relationships (`kCollideRelationship`; RBD Bullet Solver
  takes the same: Source, Target, Group, Empty, Pump, Sink and Initial Overlap relationships, and a Collide one pointing
  the other way, give no collision in either, `affector_relationships`), deduplicated, solved objects skipped. State:
  `Box3DSharedState::affectors` (object id -> `Box3DAffectorBody`); a world rebuild starts empty, unseen ones are
  destroyed.
* Transform: `SIMgetGeometryTransform`. Static Object's Use Object Transform (off by default) gives the whole world
  matrix, scale and shear included; OBJ Path drives the Position data (translation and rotation only, so an
  object-level scale is lost). With neither, the collider stays in SOP space. Geometry edits only arrive with Use
  Deforming Geometry.
* Material (`affectorBodySettings`): a Box3D Data on the affector, else its `PhysicalParms` Friction / Bounce, else
  `nodeBodyDefaults()`. Bodies start static.
* Moving: a mesh affector whose pose changes (`Box3DAffectorBody::movable`: a rigid transform, not a height field or
  ground) is retyped kinematic on its first move and driven by `setTargetTransformByHandle(pose, stepDt)` each change,
  so it pushes what it touches, as Bullet does; the velocity is cleared on the first step without one (`driven`).
  Height fields and grounds are teleported (`setTransformByHandle`). An inactive RBD Object (Create Active Object off)
  is one of these: Houdini does not give the solver such an object. Tests `bullet_inactive_push` (a box carried at 2
  and 10 m/s as Bullet does), `rbd_object_active`.
* A failed build keeps handle 0 until the signature changes (one warning per step).
* Core: `createMeshShape` (static or kinematic bodies) / `createHeightFieldShape` (static only). Box3D keeps pointers
  to the mesh and height-field data, so `Impl::bodyGeometry` owns them until the body, its shapes or the world go.

**Meshes.** Polygons fan-triangulated; Houdini front faces are clockwise and Box3D's one-sided meshes want
counter-clockwise, so triangles are flipped. A rotation + positive per-axis scale
(`decomposeScaledRigid`, Box3DAffectorMath.h) goes to the body pose and the mesh scale (`setMeshScaleByHandle`), so an
animated transform never rebuilds the mesh; shear or a mirror is baked into the vertices. Signature: P and
primitive-list data ids, counts, friction / bounce, and the matrix when baked.

**Height fields** (`findHeightVolume`, `prepareHeightField`, `buildHeightFieldBody`). The float `GEO_PrimVolume`
named `height` that is one voxel thick (with no `name` attribute, the first such volume); other primitives are ignored.
* Samples at the voxel centres (`indexToPos`); heights along the thin axis of `getTransform()`, normalised, in world
  units (the volume's own scale on that axis does not scale them; an object transform does).
* Box3D's field has columns along x, heights up y, rows along z, its corner at the body origin: the volume's
  (e1, up, e2), or (e2, up, e1) with the heights transposed when that is mirrored (a HeightField SOP's is). Frame times
  object transform through `decomposeScaledRigid`: pose to the body, scale into the field's scale (in the signature,
  since a field cannot rescale in place). Shear falls back to the field's triangles (`buildHeightMesh`) as a mesh.
* Heights are re-read and hashed only when `geometryIdentity(gdp, true)` changes (`heightSource` / `heightHash`).
* Box3D quantises heights to 16 bits over the field's own range (1.5 cm steps over 1000 m). `holes` exists in the core
  but is unused here.

**Point affectors** (`syncPointSpheres`, Box3DPointAffectors.cpp). An affector with points and no primitives (POP
particles) is a kinematic sphere per point, keyed by `id` (else the point number) in `Box3DAffectorBody::spheres`: at the
point's position through the object's transform, radius `pscale` (`kMinPointRadius` when it has none or less; `scale` is
ignored), as RBD Bullet Solver sizes them. A moved particle gets `setTargetTransformByHandle` over the step, so it pushes
what it touches, a stopped one its velocity cleared; a new `pscale` rebuilds the sphere, and a dead particle (`dead`, or
gone from the geometry) loses its body. Material as for any affector. One way: the particles are infinite mass, so a box
they hit is carried at their speed (Bullet's ends slower: 7.5 m against 5.1 m in `point_affectors`), and nothing is
written back to them (Bullet writes `Feedback` impacts onto them). Test `point_affectors`.

**Ground planes** (`isGroundPlane`). An affector whose `Volume` data is a `SIM_SDF` in Implicit Plane mode
(`kImplicitPlaneMode`, the SDF Representation menu's index, which a Ground Plane DOP sets) becomes a static ground box
(`groundSpec(settings.ground)`, the solver's Box Size) whose top is the object's own `y = 0`, facing +y,
instead of a mesh of its guide grid. Pose and scale come from `SIMgetGeometryTransform` as for any affector, but only
the pose is used; a sheared transform fails the ground. Several Ground Plane DOPs give several grounds, each its own
body. Material as above (Box3D Data, else the Ground Plane's Physical tab). The signature is the material and the
size, so moving or turning the ground only moves its body. The object needs no geometry. A Static Object DOP with its Mode set to Implicit Plane takes this path too, placed by
its object transform. Tests `ground_plane`, `static_implicit_plane`.

Not done: SDF / volume collision, mask holes, per-pair collider selection.
