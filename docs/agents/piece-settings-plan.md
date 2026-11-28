# Per-piece settings: plan

Not started. A piece of a packed body carrying its own surface material, collision filter and explicit size, as it
already carries its own shape and density ([bodies.md](bodies.md), "Pieces and packed geometry"). Listed in
[todo.md](todo.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Where pieces stand

* Only a moving `concave` body has a shape per piece: `PieceAttrs` (`Box3DShapeBuild.cpp`) reads `bullet_georep` and
  `densityscale` from each piece's first primitive, and `Piece::densityScale` reaches `createShape` /
  `createHullShape`. A `convex` body merges its pieces into one hull, a static `concave` one is one triangle mesh and a
  plain point is one shape, so none of them can differ per piece.
* `shapeDefFor` (`Box3DWorldShapes.cpp`) gives every shape the body's material and filter, and `setMaterialByHandle`,
  `setSurfaceByHandle` and `setFilterByHandle` set every shape again. A piece's own value would be overwritten by the
  next change of the body's, or by `applyGlueMaterials` averaging a glued group.
* A shape's density scale is its `userData` (`densityScaleTag`, `Box3DWorldInternal.h`); `copyShapesByHandle` copies
  it, so Glue as One Body compounds keep it.

## Decisions

* A piece's value replaces the body's, one field at a time; missing means the body's. `densityscale` stays a
  multiplier, as a body's set `mass` is shared among its pieces by weight (`totalVolume`), which an absolute density
  could not honour. Friction, bounce and filter bits have no such total.
* A piece's attribute has the body's name (`kPropAttribNames`): on the body's point it is the body's, on a primitive
  inside the packed geometry the piece's, as a material inside packed geometry beats the packed primitive's. Only
  primitives inside are read, never points: RBD Configure on unpacked geometry leaves point `bounce` and `density`
  inside the pieces, which must not become per-piece values.
* Editing them mid-simulation does nothing: they apply at creation or a shape rebuild, as `bullet_georep` does.
* A mesh with several materials takes a body-wide change through a new fork setter, not a shape rebuild (phase 3).

## Attributes

Primitive attributes on the packed geometry, read from each piece's first primitive as `PieceAttrs` reads the others:

| Attribute | Type | Meaning |
|---|---|---|
| `friction`, `bounce`, `rollingresistance` | float | The piece's own surface |
| `tangentvelocity` | vector | Conveyor velocity, in body axes as the body's |
| `collision_categorybits`, `collision_maskbits`, `collision_groupindex` | int | The piece's filter |
| `bullet_autofit` | int | 0: sized by the attributes below instead of fitted |
| `bullet_primS`, `bullet_primR`, `bullet_primT`, `bullet_radius`, `bullet_length` | vector / float | As the body's, in the packed source's space |

Box3D uses rolling resistance only on spheres and capsules (`b3SurfaceMaterial`). New values are cleaned as the body's
are (`nonNegOr`, `cleanTangent`): Box3D's input checks are asserts that Release drops.

## Phase 1: per-shape overrides in the core

1. `ShapeOverrides` in `Box3DWorld.h`: a bitmask of what is set, friction, restitution, rolling resistance, tangent
   velocity, the three filter fields and the density scale.
2. A shape's `userData` points to an interned `ShapeOverrides` in a per-world pool (`Impl::shapeTags`, freed with the
   world) instead of `densityScaleTag`; null is none. Memory is per distinct combination, not per shape.
   `shapeDensityScale` reads the pool's entry; `copyShapesByHandle` already copies the pointer.
3. `shapeDefFor` takes the overrides: the body's values fill the fields a piece leaves unset.
4. `setMaterialByHandle`, `setSurfaceByHandle` and `setFilterByHandle` leave a shape's overridden fields alone, so glue
   averaging and attribute changes still reach the rest. Check while there whether `setSurfaceByHandle`'s contact
   rebuild is needed (todo.md, "Live contact recycling flag").
5. Tests (`tests/physics_shapes.cpp`): an overridden friction survives `setMaterialByHandle`; the density scale still
   reaches the mass; a piece whose mask leaves out a category passes through a body of it while its sibling does not; a
   compound copy keeps the overrides.

## Phase 2: reading the attributes for moving pieces

1. `PieceAttrs` reads them into a `PieceSettings` on `NamedPiece`; `Piece` carries it to `Box3DPointBody.cpp`.
2. `sourceIdentity` adds the new names to its attribute list and `SourceSummary` stores the settings per piece, or the
   summary path (no geometry read) loses them.
3. Settings stay out of hull keys (hulls are geometry only) and go into `shapeSignature`, so a rebuild takes them.
4. `buildWholeObjectShapes` reads them too, so RBD Object pieces match.
5. A node warning when they are set on a `convex` body (one merged hull).
6. Houdini checks (`checks_geometry.py`): one piece with high friction stops on a slope while the other slides; piece
   filters; a second instance of the source (the summary path) keeps the settings.

## Phase 3: per-triangle materials on static meshes

What Conveyor Mesh and Conveyor Circuit need ([sample-port-table-a.md](sample-port-table-a.md)).

1. Box3D bakes `b3MeshDef::materialIndices` (a `uint8_t` per triangle) into the mesh and takes the table per shape
   (`b3ShapeDef::materials`). `MeshPiece` gains a per-triangle index and a material table (0 the body's base, pieces
   with settings from 1); the `sharedMeshes` key adds a hash of the indices, so instances with different values still
   share a mesh.
2. More distinct materials than a `uint8_t` indexes merge, with a note.
3. A filter cannot be per triangle: a mesh whose pieces have different filters is one mesh shape per distinct filter.
4. Fork: `b3Shape_SetFriction`, `b3Shape_SetRestitution` and `b3Shape_SetSurfaceMaterial` set only `materials[0]`
   (`shape.c`), so a body-wide change misses the fields a piece left to the body in the other slots. Add
   `b3Shape_SetSurfaceMaterialAt( shapeId, index, material )` and use it from the three setters.

## Phase 4: explicit sizes

1. In `fittedPiece`, a piece with `bullet_autofit` 0 skips `fitShapeOriented`: its `ShapeSpec` comes from `bullet_prim*`
   as `applyPrimOverrides` builds one, moved into body space by the packed `pivot` and `S` as `packedPointsToBodyLocal`
   moves points.
2. `pieceFitKey` includes the values.
3. A capsule's or cylinder's radius under a non-uniform packed scale follows `makeShapeSpec`: half the larger of the two
   axes across it.

## Phase 5: samples and docs

* Conveyor Circuit and Conveyor Mesh use per-triangle materials instead of one static piece per material; Compound and
  Falling Trees use piece materials where Box3D's samples set them.
* Docs: `shapes.md` and `attributes.md` (a row each for the user), `bodies.md`, `core-shapes.md`, the sample tables.
  Then delete this file and its todo.md line.

## Left out: sensors

A sensor is Box3D's `isSensor` on a shape, so a sensor piece is cheap, but nothing reports sensor events yet (todo.md,
"Sticky collisions"). That needs its own design (for example an array attribute of the overlapping ids each frame).
