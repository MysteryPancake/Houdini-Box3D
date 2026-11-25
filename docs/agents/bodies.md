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
* **Shape rebuild** (`rebuildPointShapes`): on `deforming` (every step), `bullet_autofit_valid == 0` (once), a
  size-key change beyond 1e-4 relative for convex / concave (`autoScale`; packed row lengths or `pscale * scale`), or
  a concave body changing between static and not, or a packed piece whose 3x3 turned mirrored or back
  (`mirrored_midsim`). `planShapes` gives a `shapeSignature`; equal means nothing is touched, otherwise shapes are
  replaced on the same body (handle, pose, velocity kept), the mass reapplied and the body woken. No usable shape
  leaves it `shapeless`, hence static.
* Editing `bullet_georep`, `bullet_prim*` or the node's shape settings alone does nothing (as in Bullet); they apply at
  creation or a rebuild. After a build with `bullet_autofit_valid = 0` the solver writes 1; fitted sizes are not
  written into `bullet_prim*`.
* **`id`** is the identity key, no special values (no `id = -1` rebuild trick).

## Building shapes (`addPointBody`, `planShapes`, `buildShapes`)

* Unpacked geometry: a point on a primitive that is not packed (`Box3DPointSlot::unpackedPrim`, set in `readSlot`)
  gets no body, and a body it had is removed. An object with such points and no packed point is not simulated at all
  (a warning naming it, with RBD Bullet Solver's wording and severity: `SyncCounts::unsupportedObjects`); with packed
  points too, a warning (`unpackedPrimPoints`). Loose points (no vertex) stay bodies, a Box3D extension (`pscale` /
  `bullet_georep` points; tests `pscale`, `messages`) where Bullet gives its unsupported-geometry warning
  (`unpacked_geometry_bullet`). RBD Point Object is not this path: it makes whole objects (`other_rbd_objects`).
  Either keeps the object scanning every step (`problemCount`), so the message repeats. Tests `unpacked_geometry`,
  `unpacked_geometry_bullet`.

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
     (`staticMeshPiece`, `innerTriangles`), winding flipped from Houdini's. The mirror keeps
     `concave`, `plannedStatic` and `meshShapes`. Becoming static retypes before the rebuild; leaving static rebuilds as
     hulls first, and a body with a mesh is never retyped away from static. No polygons: hulls.
  3. Convex / concave hulls: `pieceFromPoints` with a `packedHullKey`, `findHull` first, points only on a miss, else a
     fitted box with a note. Convex merges all pieces unless hull-per-connected; concave is one hull per piece, or per
     connected set with hull-per-connected.
  4. Box / sphere / capsule with autofit: `fitShapeOriented` (below).
  5. Nothing usable: `makeShapeSpec(pscale * scale)`, plus `applyPrimOverrides` only with autofit off (RBD Packed
     Object writes default `bullet_primS` / `bullet_radius` onto every point).
* Packed points sharing a `name` are separate bodies, as in Bullet (`name_fusion_probe`); Bullet's only coupling, the
  first point's starting `v`, is not copied.
* Notes become one node warning per kind, on the step a shape is built.
* A rebuild adds all of a body's shapes or none (`addAllShapes`): a body is never left with some of them. When it gets
  none it is static until a later rebuild gives it shapes (a `fallbacks` warning).
* Point attributes win over Box3D Data, as in Bullet. `noteOverriddenData` lists the attributes present that replace
  a Box3D Data value changed from its default (`nodeBodyDefaults`; once per object, `overridesReported`): a value left
  at its default is not reported, so the Box3D Packed Object's attributes over its default Box3D Data are not.

**AutoFit** (`fitShapeOriented`, also for whole objects): the smallest box among the body axes and, from a throwaway
hull, each face normal paired with each of its edges. A turned frame must beat the body axes by `kTurnedFitGain`;
`canonicalFrame` picks the equivalent frame closest to them. In that frame (`fitShapeInFrame`, Box3DShapes.h), as
measured against Bullet (`autofit_bullet`): box = the bounds; sphere = centred on them, through the farthest point;
capsule = along the axis nearest body Y, radius through the farthest point from it, caps ending at the bounds. Fits
are cached process-wide by point content and shape type (`FitLibrary`, Box3DWorldHulls.h), so instances of one piece fit
once. A packed instance of a source with a summary is looked up first by `packedFitKey` (the summary's group hash,
pivot, scale and type: `findFit` / `storeFit`), before its points are moved into body space and hashed, as the hulls
are. `fitShapeToPoints` (body axes) is the fallback when a piece cannot be hulled.

**Explicit primitives** (autofit off, `box` / `sphere` / `capsule`): `bullet_primS` = box half extents,
`bullet_radius`, `bullet_length` = capsule straight part, `bullet_primT` = offset, `bullet_primR` = XYZ Euler degrees.
Box3D Data has them as parameters of the same names (`BodySettings::prim`, read by `readPrimOverrides` under the point
attributes): a value left at its default is not set, so pscale and scale still size the shape.
A rotated box is an 8-corner hull. The rotation order and length meaning live only in `applyPrimOverrides`.

**Pieces and packed geometry.** Pieces are grouped by primitive `name` (unnamed share one); `finishPieces` sorts,
dedupes, reads and hashes them. Packed fragments share one source detail, read with `GU_PackedFragment::detailPtr()`
(not `getPackedDetail()`, whose detail does not match `getPrimitiveRange()`); a fragment is keyed by its first
primitive. The pivot is `getPivot`; the body origin is the point's `P`. Packed rotation is read from
`getFullTransform4` and written with `setLocalTransform` through `splitPackedMatrix` / `composePackedMatrix` (HDK
quaternions; shear dropped). A packed primitive inside a source contributes its own points, carried into the source's
space by `(p - pivot) * M + P` (`appendNestedPacked`, recursive); its own point is not a collision point. Quadric
primitives (Sphere, Tube, Circle, Metaball) have one point and a transform, so `appendQuadricSamples` adds points on
their surface (`Box3DQuadrics.h`, sized like Bullet's, `bullet_primitive_types`). A Tube's rings come from
`GEO_PrimTube::evaluatePoint` (taper included). A Sphere or Metaball that is a piece's only primitive and uniformly
scaled is a real Box3D sphere (`NamedPiece::sphereValid`). A flat piece that cannot be hulled (a Circle as polygons)
becomes a box `kFlatHalfThickness` thick. Nested packs and quadrics are read the same way for Hull per Connected
pieces.
