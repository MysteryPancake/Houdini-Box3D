# Bodies: updates, features, shapes

How a point becomes a body and how it changes mid-simulation. Step order: [solver.md](solver.md). Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## Mid-simulation updates

Target behaviour is Houdini's Bullet solver. A new body is built from the same readers, so an untouched point compares
equal to what its body was built with.

* **Settings** (`readDynamicProps` -> `applyPropChanges`): the effective values (attribute, else Box3D Data) in
  `b3dop::BodyProps`, compared with `entry->props`; only setters for what differs run. Compared only when
  `checkProps` (prop markers changed, none recorded yet, or deforming bodies). Mass settings go through
  `applyBodyMass`, the same as a fresh build.
* **Shape rebuild** (`rebuildPointShapes`): on `deforming` (every step), `bullet_autofit_valid == 0` (once), a size-key
  change beyond 1e-4 relative for convex / concave (`autoScale`; packed row lengths or `pscale * scale`), or a concave
  body changing between static and not. `planShapes` gives a `shapeSignature`; equal means nothing is touched,
  otherwise shapes are replaced on the same body (handle, pose, velocity kept), the mass reapplied and the body woken.
  No usable shape leaves it `shapeless`, hence static.
* Editing `bullet_georep`, `bullet_prim*` or the node's shape settings alone does nothing (as in Bullet); they apply at
  creation or a rebuild. After a build with `bullet_autofit_valid = 0` the solver writes 1; fitted sizes are not
  written into `bullet_prim*`.
* **`id`** is the identity key, no special values (no `id = -1` rebuild trick).

## Per-attribute features

* **Centre of mass** = the point (a packed point's pivot): `applyBodyMass` -> `applyMassByHandle(comAtOrigin = true)`
  after creation, rebuild, retype to dynamic and mass changes. No `computecom`.
* **Mass** (`computemass`, `density`, `inertialtensorstiffness`): `boxDensity` = density with computemass, else
  mass / volume. With computemass the mass is written to `mass` on creation, rebuild or change.
* **CCD threshold** (`ccdthreshold`) -> `b3BodyDef::safetyFactor` (Box3D's name; negatives keep the default).
* **Fast rotation** (`fastrotation`) -> `allowFastRotation`; without it Box3D caps spin at `B3_MAX_ROTATION` per DOP step.
* **Motion locks** (`lock_linear`, `lock_angular`, nonzero = locked world axis) -> `motionLocks`; Box3D zeroes locked
  velocity components every iteration.
* **Collision filtering** (`collision_category`, `collision_mask` read as `GA_ROHandleID` so `-1` means every bit;
  `collision_group`) -> the shapes' filter. Affector bodies use the defaults.
* **Animated** (`animated` with `active = 0`): kinematic; a pose edit becomes `setTargetTransformByHandle(cur, stepDt)`
  and a step without one zeroes the velocity. Not written back.
* **Fast object** (`fastobject`): Box3D's `isBullet`, renamed (Houdini's `isbullet` means something else).
* **Sleep and enable** (`enablesleep`, `awake`, `enabled`, Box3D's own model):
  * `enablesleep` and `enabled` are live settings; toggling `enabled` is `b3Body_Enable` / `Disable` (expensive). A
    disabled body has no move events, so it is not written back.
  * `awake` is state: `isAwake` at creation, then edge-triggered. `Box3DBodyMirror::awake` is the attribute as last read
    or written; a `checkProps` scan that finds a different value treats it as a user write (wake or sleep), and the
    write-back writes the attribute only when a move event disagrees with the mirror.
  * `sleepByHandle` sleeps the whole island and Box3D may refuse; the next move event then writes `awake = 1` back.
  * Forced sleep also sleeps touching bodies, but Box3D reports no move events for them, so their `awake` stays 1
    until they wake and sleep normally.
* **Contact recycling** (`contactrecycling`): per-body flag; the distance is world-wide
  (`WorldSettings::contactRecycleDistance`, set after `b3CreateWorld`). A pair recycles only if both bodies have the
  flag. Existing contacts keep the flag they were made with, so `setContactRecyclingByHandle` rebuilds an awake dynamic
  body's contacts (disable + enable, velocity saved and restored).
* **Rolling resistance and conveyor velocity** (`rollingresistance`, `tangentvelocity`): the shapes' surface
  material (negatives become 0). Tangent velocity is shape-local, so friction must be above 0. Live changes
  (`setSurfaceByHandle`) also rebuild the body's contacts, because Box3D skips material updates on recycled contacts.
  Affector meshes keep these out of their signature, so an animated belt speed updates the shapes, not the mesh.
* **Impacts** (Box3D Data Add Impact Data): `recordImpacts` reads this step's awake bodies' contacts once and records
  each for both bodies whose object asks (the sleeping side too, unless it is an asleep dynamic body). A fresh
  `SIM_Impacts` each step (`addPointImpact`, normal pointing at the body that gets the impact); no Feedback data.
* **Dead** (`dead`): never stamped, so the body goes. With Reap Dead Points the point is queued as a `Box3DDeletion`;
  `deleteDeadPoints` resolves all queued points before deleting any, then bumps data ids and forces a scan.
* **Aging** (solver Enable Aging): `age += dt` on every point (not on the creation frame without Solve On Creation
  Frame), `dead = 1` once `age > life` (life 0 never dies), matching POP (`aging_vs_pop*`). Reap At Frame End
  deletes it in the same step, else the next sync does. Parallel past `kMinParallelPoints`.
* **Time Scale**: Box3D steps by `stepDt = dt * timescale`.

**Point writes after the step** (`Box3DPointWrite`: `kAutofitValid`, `kMass`): grouped per object
(`forEachObjectRun`), offsets via `resolvePointOffsets`, then `recordFingerprintAfterWrite`.

## Building shapes (`addPointBody`, `planShapes`, `buildShapes`)

* Defaults: `nodeBodyDefaults()` (Box3DWorld.h), Bullet's RBD values, also for objects without Box3D Data.
  `BodySettings()` keeps the core's own values for direct use of `Box3DWorld`.
* Type: `wantedType` (active -> node type; inactive -> static, or kinematic if animated; shapeless -> static).
* `createBodyWithShapes` (shared with whole objects) makes the body, adds the shapes, applies the mass, then the initial
  `v` / `w` (after the mass; not for static, animated or not-awake bodies). The mirror entry is inserted first and
  becomes the body's user data; it is erased if the body cannot be built.
* `buildShapes`, in order:
  1. Packed point: its geometry as `PieceGroup`s via `ShapeSources` (cached per source detail, and per
     `(source, first primitive)` for fragments, for one `syncObject`).
  2. Concave on a body that will be static (`Box3DPointSlot::staticBody`): one triangle mesh of the packed polygons
     (`staticMeshPiece`, `innerTriangles`), winding flipped from Houdini's unless `flipMeshWinding`. The mirror keeps
     `concave`, `plannedStatic` and `meshShapes`. Becoming static retypes before the rebuild; leaving static rebuilds as
     hulls first, and a body with a mesh is never retyped away from static. No polygons: hulls.
  3. Convex / concave hulls: `pieceFromPoints` with a `packedHullKey`, `findHull` first, points only on a miss, else a
     fitted box with a note. Convex merges all pieces unless hull-per-connected; concave is one hull per piece.
  4. Box / sphere / capsule with autofit: `fitShapeOriented` (below).
  5. Nothing usable: `makeShapeSpec(pscale * scale)`, plus `applyPrimOverrides` only with autofit off (RBD Packed
     Object writes default `bullet_primS` / `bullet_radius` onto every point).
* Packed points sharing a `name` are separate bodies, as in Bullet (`name_fusion_probe`); Bullet's only coupling, the
  first point's starting `v`, is not copied.
* Notes become one node warning per kind, on the step a shape is built.
* Point attributes win over Box3D Data, as in Bullet. `noteOverriddenData` lists the attributes present that replace
  a Box3D Data value (once per object, `overridesReported`); the warning points at the Box3D Packed Object.

**AutoFit** (`fitShapeOriented`, also for whole objects): the smallest box among the body axes and, from a throwaway
hull, each face normal paired with each of its edges. A turned frame must beat the body axes by `kTurnedFitGain`;
`canonicalFrame` picks the equivalent frame closest to them. In that frame (`fitShapeInFrame`, Box3DShapes.h), as
measured against Bullet (`autofit_bullet`): box = the bounds; sphere = centred on them, through the farthest point;
capsule = along the axis nearest body Y, radius through the farthest point from it, caps ending at the bounds.
`fitShapeToPoints` (body axes) is the fallback when a piece cannot be hulled.

**Explicit primitives** (autofit off, `box` / `sphere` / `capsule`): `bullet_primS` = box half extents,
`bullet_radius`, `bullet_length` = capsule straight part, `bullet_primT` = offset, `bullet_primR` = XYZ Euler degrees.
A rotated box is an 8-corner hull. The rotation order and length meaning live only in `applyPrimOverrides`.

**Pieces and packed geometry.** Pieces are grouped by primitive `name` (unnamed share one); `finishPieces` sorts,
dedupes, reads and hashes them. Packed fragments share one source detail, read with `GU_PackedFragment::detailPtr()`
(not `getPackedDetail()`, whose detail does not match `getPrimitiveRange()`); a fragment is keyed by its first
primitive. The pivot is `getPivot`; the body origin is the point's `P`. Packed rotation is read from
`getFullTransform4` and written with `setLocalTransform` through `splitPackedMatrix` / `composePackedMatrix` (HDK
quaternions; shear dropped).
