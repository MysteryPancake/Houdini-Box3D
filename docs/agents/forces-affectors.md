# Forces and affectors

External forces (Force DOPs, point `force` / `torque`) and Static Object colliders. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## External forces (`applyExternalForces`)

Runs after `syncFromGeometry` and the last cancel check, before the step, when `useforces` is on. Same lookup as
Houdini's `SIM_SolverHair`: `obj->forEachConstSubData(collector, SIM_DataFilterByType("SIM_Force"), SIM_FORCES_DATANAME,
SIM_DataFilterNone())` with a `SIM_EachDataCallback` (`ForceCollector`) that casts each sub-data to `SIM_Force`.

Only objects with force DOPs or non-zero point forces are visited, and only their own bodies
(`Box3DObjectSync::bodies`, rebuilt by each full scan and kept while the object is skipped): skip bodies that are not
`dynamic()`, are not `props.enabled`, or are asleep while `forceswake` is off. For each remaining body one `bodyStateByHandle` call reads awake, mass, world centre of mass and `v` / `w`
(it replaces four separate calls; with only point forces and no Force DOP just `isAwakeByHandle` is read, and only when
`forceswake` is off). Mass, centre of mass and velocities go into `SIM_Force::getForce(object, pos, vel, angvel,
mass, force, torque)` for every force on the object, summed, and the sum goes in through `applyForceByHandle`
(`b3Body_ApplyForceToCenter` + `b3Body_ApplyTorque`).

* Box3D adds `invMass * force` and `invInertiaWorld * torque` in every substep's velocity integration
  (`b3IntegrateVelocitiesTask`) and zeroes both when the step finishes, so they are per-step: never carry them over, and
  a rebuilt world has none. `Box3DWorld` does not track them.
* Box3D ignores a force on a body that is not in the awake set. `wake = false` (the default) therefore skips sleeping
  bodies, which is what keeps gravity-like forces from holding resting bodies awake; `forceswake` passes `wake = true`.
* Non-finite sums are skipped and counted (`ForceStats::nonFinite`, one node warning). `applyForceByHandle` also
  sanitizes (`finiteVec3`) because Box3D asserts on non-finite vectors.
* Evaluation is serial: `SIM_Force::getForce` is user-extensible and nothing guarantees a custom Force DOP is thread-safe. See
  "TODO" in [status.md](status.md) for the parallel option.
* Gravity Force DOP + the solver's `gravity` add up (both are applied). `gravityscale` only scales the solver's gravity.
* Not used: `getForceOnCircle` / `getForceOnSphere` (area and volume variants), `getPointForces`.
* `useforces` / `forceswake` are plain options read each step; nothing is stored in the world, so they need no rebuild.

## Force and torque point attributes

`applyExternalForces` also reads per-point `force` / `torque` (V3) attributes from each object's geometry into a short
list of the non-zero ones (by offset; a body finds its point by binary search), adds them to the Force DOP result for the
same body, applies the sum, then zeroes the ones it read (a write lock, followed by
`recordFingerprintAfterWrite`, so the next sync does not see the clear as an external edit). Zeroing is needed because
nothing else clears `force` without a POP Solver. The POP Force node must run before this solver. The attributes' data
ids right after the clear (or after a read found them all zero) are kept in `Box3DObjectSync::forceId / torqueId`; while
they are unchanged nothing new was written and the read is skipped.

## Affector static colliders

`Box3DAffectors.cpp` (`syncAffectorBodies`, after the objects, mutex held). Objects the solver is not given but that
affect a solved object through a collision relationship (`getConstAffectors(arr, "SIM_RelationshipCollide")`) become
static bodies: a height field if the geometry has one, else a triangle mesh of its polygons.

* Gather: affectors of every solved object, deduplicated, solved objects skipped. State:
  `Box3DSharedState::affectors` (object id -> `Box3DAffectorBody`); a world rebuild starts empty, unseen ones are
  destroyed.
* Transform: `SIMgetGeometryTransform`. Static Object's Use Object Transform (off by default) gives the whole world
  matrix, scale and shear included; OBJ Path drives the Position data (translation and rotation only, so an
  object-level scale is lost). With neither, the collider stays in SOP space. Geometry edits only arrive with Use
  Deforming Geometry.
* Material (`affectorBodySettings`): a Box3D Data on the affector, else its `PhysicalParms` Friction / Bounce, else
  `nodeBodyDefaults()`. Always static.
* A failed build keeps handle 0 until the signature changes (one warning per step).
* Core: `createMeshShape` / `createHeightFieldShape` (static bodies only). Box3D keeps pointers to the mesh and
  height-field data, so `Impl::bodyGeometry` owns them until the body, its shapes or the world go.

**Meshes.** Polygons fan-triangulated; Houdini front faces are clockwise and Box3D's one-sided meshes want
counter-clockwise, so triangles are flipped unless Box3D Data's `flipwinding`. A rotation + positive per-axis scale
(`decomposeScaledRigid`, Box3DAffectorMath.h) goes to the body pose and the mesh scale (`setMeshScaleByHandle`), so an
animated transform never rebuilds the mesh; shear or a mirror is baked into the vertices. Signature: P and
primitive-list data ids, counts, friction / bounce / flip, and the matrix when baked.

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

Not done: SDF / volume collision, mask holes, mesh sharing between identical affectors, per-pair collider selection.
