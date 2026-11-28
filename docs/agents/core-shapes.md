# Core layer: shapes

`Box3DWorld`'s shapes (hulls and their cache, meshes, height fields, the ground) and the Houdini-free helper
headers. The rest of the core layer: [core.md](core.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

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

The hull cache is **process-wide** (`HullLibrary` in Box3DWorldHulls.h, mutex-protected): Box3D adds a hull to the
world's own database (`b3AddHullToDatabase`) when a shape is made: compared by content, so identical hulls share one
reference-counted clone and only a new hull is cloned. Our entries are only needed until then and can be shared by
every world. A rebuilt world (scrub, re-solve, thread change) therefore finds every hull its predecessor built.
Entries are `shared_ptr`s; a world holds the ones it found or built (`Impl::held`) until `pruneHulls()`, which the
solver calls at the end of each sync, so eviction never frees a hull between `findHull` and `createHullShape`. Past
the byte budget (`setSharedHullBudget`; the default is in `HullLibrary`, `Box3DWorldHulls.h`) the least recently used
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
`WorldSettings::ground` holds it (the solver's Box Size, applied each step with the rest of the tuning, read
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
`Box3DWorld::fitShapeOriented`, which needs a Box3D hull; with `alongSymmetryAxis` a capsule or cylinder takes
`symmetryAxisFrame`, the principal axis by `symmetricEigen3`, as a reduced hull's faces miss a rod's axis by a degree); hashing (`kHashSeed`, `mixHash`, `hashFloats`).

**`Box3DMath.h`** (no dependencies; included by `Box3DShapes.h` and `Box3DJoints.h`): `kDegToRad`, `finite3`, `copy3` /
`copy4`, `quatsDiffer`, `xformPoint`, `sanitizeQuat` and `storedPosition` (shared by the read path and the
write-back's prediction).
