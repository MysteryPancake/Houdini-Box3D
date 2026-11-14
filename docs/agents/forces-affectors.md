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
  "Known gaps / TODO" in [status.md](status.md) for the parallel option.
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
  `b3MeshData`, so `Impl::bodyGeometry` owns it (and height fields' `b3HeightFieldData`): freed after `destroyBody` / `destroyShapesByHandle`, and after the world.
* Material (`affectorBodySettings`): an optional `Box3DData` on the affector (friction, bounce); without one, the
  object's own Friction and Bounce from its `PhysicalParms` (Static Object's and Ground Plane's Physical tab: 1 and
  0.5 by default), read by name as for RBD Objects; negative or non-finite values keep `nodeBodyDefaults()`. The body
  type is always static. (`static_material` checks Static Object's Friction.)
* Height fields: see below.
* Not done: SDF / volume collision of Static Objects, holes from a mask layer, mesh sharing between identical affectors,
  `getCollider`-style per-pair collider selection, affectors with `SIM_ColliderInfo` options.
* Verified against the HDK docs: `void SIMgetGeometryTransform(UT_DMatrix4 &result, const SIM_Object &)` (SIM_Utils.h;
  geometry local space to world space) and `void SIM_Object::getConstAffectors(SIM_ConstObjectArray &, const char *relationshiptype) const`.

### Height fields

An affector whose geometry has a height layer becomes one static Box3D height field instead of a mesh; its other
primitives are ignored (`findHeightVolume`, `prepareHeightField`, `buildHeightFieldBody` in Box3DAffectors.cpp).

* Which volume: the float `GEO_PrimVolume` named `height` that is one voxel thick along one index axis (Houdini's
  height fields are 2D volumes; the HeightField SOP's default layer is `height`); with no `name` attribute at all, the
  first such volume. 3D volumes and other layers (`mask` ...) are ignored.
* Grid (`readHeightFrame`): the two other index axes give the sample counts and steps (`indexToPos` of index 0 and 1,
  so the samples are the voxel centres, as Houdini draws them); heights run along the thin axis of
  `GEO_PrimVolume::getTransform()`, normalised, in world units (a volume's own scale along that axis does not scale
  them; an object-level transform does).
* Frame: Box3D's height field has columns along local x, heights up y, rows along z and its corner at the body origin.
  That is the volume's (e1, up, e2), or (e2, up, e1) with the heights transposed when the first is mirrored. The frame
  times the object transform goes through `decomposeScaledRigid`: the pose goes to the body (animated transforms only
  move it, `setTransformByHandle`), the scale times the steps into the height field's scale. A height field cannot
  change its scale in place, so the scale is in the signature (a change rebuilds it). Shear (or a scale along a
  slanted axis) falls back to the field's triangles as a mesh (`buildHeightMesh`, counter-clockwise from above,
  through the mesh path's pose / bake).
* Signature: a hash of the heights, the grid, friction / bounce, the mode and the scale (or the baked matrix). The
  heights are read and hashed (`UT_VoxelArrayIteratorF` over `getVoxelHandle()`) only when `geometryIdentity(gdp,
  true)` changes (kept in `Box3DAffectorBody::heightSource` / `heightHash`), and read again for a rebuild.
* Core: `Box3DWorld::createHeightFieldShape(h, body, heights, countX, countZ, scale, holes)` (static bodies only;
  refuses scale <= 0, non-finite heights, fewer than 2 x 2 points). Heights are quantised by Box3D to 16 bits between
  the field's own lowest and highest height (`globalMinimumHeight` / `Maximum`), so a 1000 m range has 1.5 cm steps.
  `holes` maps to material `B3_HEIGHT_FIELD_HOLE`; the Houdini side does not use it yet. Tested in
  tests/core_heightfield.cpp; Houdini checks `heightfield_flat`, `heightfield_terrain` (plain, posed, sheared fallback,
  compared with `hou.Volume.sample`), `heightfield_animated`.
* HDK calls (from the HDK docs): `GEO_PrimVolume::getRes(int&, int&, int&)`, `getStorageType()`,
  `indexToPos(exint, exint, exint, UT_Vector3D&)`, `getTransform()` (`const UT_Matrix3 &`), `getVoxelHandle()`;
  `UT_VoxelArrayIterator::setConstArray / rewind / atEnd / advance / x / y / z / getValue`.
