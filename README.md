# Houdini Box3D

Houdini DOP wrapper for [Box3D](https://github.com/erincatto/box3d), one of the fastest rigid body solvers currently available.

My goal is optimizing this as much as possible to ensure the great performance of the original codebase.

> [!CAUTION]
> This is heavily vibe coded, as I'm still learning the HDK!
> 
> This isn't normally how I develop software!

## Building

### Windows

#### 1. Box3D only (no Houdini needed)

```
cmake -S . -B build_core -G "Visual Studio 17 2022" -A x64 -DBOX3DDOP_CORE_ONLY=ON
cmake --build build_core --config Release
build_core\Release\b3dop_core_smoke.exe
```
Expected: prints the Box3D version, `world valid: yes`, `OK`.

#### 2. The Houdini plugin

Open "Houdini X.Y.Z > Command Line Tools" from the Start menu so %HFS% is set, then:

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
or open `build\Box3DDOP.sln` in Visual Studio. Use Release or RelWithDebInfo (not Debug).
The DLL is copied to your Documents\houdiniX.Y\dso. Close Houdini before rebuilding.

### Linux

You need g++, CMake, git and the development packages the HDK lists. On Debian-based
distributions:

```
sudo apt-get install g++ cmake git mesa-common-dev libglu1-mesa-dev libxi-dev
```
On Red Hat-based distributions: `gcc-c++ cmake git mesa-libGL-devel mesa-libGLU-devel libXi-devel`.

#### 1. Box3D only (no Houdini needed)

```
cmake -S . -B build_core -DCMAKE_BUILD_TYPE=Release -DBOX3DDOP_CORE_ONLY=ON
cmake --build build_core -j
./build_core/b3dop_core_smoke
```
Expected: prints the Box3D version, `world valid: yes`, `OK`.

#### 2. The Houdini plugin

Set up the Houdini environment so `$HFS` is set, then run the build script:

```
source /opt/hfsX.Y.ZZZ/houdini_setup
./build.sh
```
If `$HFS` is not set, `build.sh` uses the newest `/opt/hfs*`. It fetches the Box3D
submodule if needed, resets `build/` if the folder was moved, then configures
(Release) and builds. To do it by hand:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```
Use Release or RelWithDebInfo (not Debug). The plugin is copied to `~/houdiniX.Y/dso`.
Close Houdini before rebuilding.

### In Houdini

DOP network -> TAB -> "Box3D Solver" / "Box3D Data".

## Point attributes

Like Houdini's RBD solvers, the solver works on **points**. Every point in a DOP
object's Geometry (the normal `Geometry` data) is one rigid body. An object
without Geometry is ignored.

Attribute names follow Houdini's RBD names where one exists. All are optional
except `P`.

| Attribute | Type | Meaning | If missing |
|---|---|---|---|
| `P` | vector | body position | required |
| `orient` | quaternion | rotation | identity |
| `v` | vector | linear velocity | 0 |
| `w` | vector | angular velocity, radians/s | 0 |
| `mass` | float | density = mass / shape volume | density from Box3D Data node |
| `friction` | float | shape friction | Box3D Data node |
| `bounce` | float | restitution | Box3D Data node |
| `active` | int | 0 = static, otherwise dynamic | dynamic |
| `shape` | string | `box`, `sphere`, `convex`, `concave` (lowercase only, case-sensitive) | Box3D Data node's Default Shape |
| `pscale` | float | uniform size (plain points) | 1 |
| `scale` | vector | per-axis size (plain points) | 1 1 1 |
| `id` | int | stable point id | point number |

**Size.** The shape is a unit shape scaled by `pscale * scale`, the same as
Copy to Points on a unit primitive: a box is `pscale*scale` wide on each axis, a
sphere has diameter `pscale * max(scale)`.

**Convex and concave.** See the next section. `convex` is one hull, `concave` is
several hulls on one body.

**Identity.** Bodies are keyed by (object id, `id`). Use an `id` attribute if
points can be deleted or reordered, because the point number shifts when points
are removed. Duplicate ids in one object are skipped (first wins).

**Results.** Every step the solver writes `P`, `orient`, `v` and `w` back to the
dynamic points, creating `orient`, `v` and `w` if the points do not have them.
Static points are not written.

**Edits from other nodes.** If something else changes a point's `P`, `orient`,
`v` or `w` between steps, the new value is pushed into Box3D. Points that appear
get a body; points that disappear lose it; flipping `active` switches the body
between static and dynamic. Changing `shape`, `pscale`, `scale`, `mass`,
`friction` or `bounce` on an existing body is NOT picked up (the body keeps the
values it was created with) until the world is rebuilt (Reset Simulation).

The Box3D Data node is only the fallback for points that lack an attribute
(and sets damping, gravity scale, bullet, sleep for all points of the object).

## Convex and concave

There is a single input, the object's `Geometry`. A point whose `shape` is
`convex` or `concave` gets hull shapes built from the geometry **packed inside
its primitive**, so these shapes need packed primitives (one point + one packed
primitive per body, e.g. from Pack or RBD Packed Objects).

What you pack is your choice: a simplified proxy (a few convex pieces) or the
full-detail geometry. The solver does not distinguish them; it hulls whatever
is in the packed primitive. For a concave body a proxy is usually what you want,
because a hull per piece of a dense mesh gets expensive and each hull is capped
(see below).

* Primitives inside the packed geometry are grouped into **pieces** by the
  primitive string attribute `name`. Primitives without a `name` all form one
  piece. A packed detail with points and no primitives is one piece.
* `convex` merges all the pieces into **one** hull. `concave` makes **one hull
  per piece** and puts them all on the same body (Box3D calls this a runtime
  compound, and it is the way to get a dynamic concave body: triangle meshes
  only collide as static). The hulls are built at body-creation time only.
* **Create Convex Hull per Set of Connected Primitives** (Box3D Data node, like
  Bullet's option of the same name): for `convex` shapes, makes one hull per set
  of connected primitives (primitives that share points) instead of one hull of
  everything, all on the same body. It ignores `name`, and has no effect on
  `concave`, `box` or `sphere`. Like the other node settings it is read when a
  body is created. A packed detail with points and no primitives is one set.
* A plain (non-packed) point has no geometry to hull. A `convex` / `concave`
  plain point falls back to a box scaled by `pscale * scale`, and the reason
  shows in the debug log. Pack the geometry to get real hulls.
* Packed geometry empty, fewer than 4 points, flat, or too complex to hull:
  that piece (or the body) falls back to a bounding box, so the body still
  exists. The reason shows in the debug log.

**Hull limits.** Box3D hulls are limited to 128 vertices, 128 faces and 128
edges. A dense point cloud is reduced automatically (a smaller vertex budget is
retried until the hull is valid), so a very detailed piece becomes a slightly
smaller hull. Pack simplified pieces when the shape matters. Hulls are cached
by content, so many bodies sharing one source cost one hull build.

**Mass.** `mass` becomes density = mass / total hull volume. For `concave`
the volume is the sum of the pieces, so overlapping pieces count their
overlap twice.

## Packed primitives

A packed primitive is one point and one primitive. For these points:

* **Shape.** `box` / `sphere` are fitted to the packed geometry's bounds (not a
  unit shape: `pscale` and `scale` only matter as the fallback if the packed
  geometry is empty). `convex` / `concave` hull the packed geometry (see above).
* **Pivot.** Hull points are taken relative to the primitive's pivot (read with
  `GEO_PrimPacked::getPivot`, not derived from the transform's translation), with the
  primitive's scale baked in and its rotation removed, so the body origin is the
  point's `P` and shapes off-centre from the pivot are handled. Instances of one
  source (Houdini shares one packed geometry between them, e.g. copy to points)
  share one cached hull: the source's points are hashed once per sync, and each
  instance only adds its pivot and scale to the cache key, so the body-local
  points are only built the first time. The source is assumed not to deform or
  animate.
* **Packed fragments.** A Pack SOP run with a `name` attribute makes one packed
  fragment per name, all sharing one source. Each fragment is its own body and
  its collision geometry is only the points of that fragment's primitives (one
  piece), not the whole source. Fragments of one source share the source
  geometry, and a fragment's hull is keyed by (source, fragment name, pivot,
  scale). The `name`-based pieces described under "Convex and concave" are for a
  packed primitive that holds several named parts in ONE body, not for fragments.
* **Rotation.** The body rotation is read from the primitive's own transform, not
  from `orient`. Results are written back to the point's `P` **and** to the
  primitive's local transform (the scale of the primitive is kept, the rotation
  is replaced). `orient` is also written, for convenience, but ignored for packed
  points on input.
* Packed primitive types other than ordinary in-memory packed geometry work as
  long as `getPackedDetail()` returns their geometry; an unloaded packed
  primitive has no geometry and gets the fallback box.

## Verification status

The points-based solver, including convex / concave shapes and packed primitives,
has NOT been compiled or run in Houdini yet. `tests/box3d_test.hiplc` needs a
points-based setup. The core
(`Box3DShapes.h`, `Box3DWorld`) is covered by `b3dop_core_physics`: shape parsing,
sizing, mass to density, packed-primitive matrix math, hull creation / limits /
caching / instancing, offset hulls and compounds under real physics.

If the HDK build complains, the likely spots are the packed-primitive calls in
`SIM_SolverBox3D.cpp`: `GU_PrimPacked::isPackedPrimitive`,
`getPackedDetail()`, `getPivot()`, `getFullTransform4()`, `setLocalTransform()`,
`transformDirty()`, `UT_QuaternionD::getRotationMatrix()`, and the fragment calls
(`GU_PackedFragment` via `sharedImplementation()`, `getPrimitiveRange()`, `name()`), plus
`GU_DetailHandleAutoReadLock`. The first, third and fourth are written so a
different signature falls back instead of failing to compile; if
`setLocalTransform(UT_Matrix3D)` does not exist, packed rotation cannot be
written back and packed points are then treated like plain ones (rotation in
`orient`), which means packed geometry will not visibly rotate.

## Warnings on the node

Problems that change what gets simulated are shown as warnings on the solver
node (they are also in the debug log): a point that fell back to a box because
its geometry could not be hulled (flat, fewer than 4 points, too complex) or
was empty, points that could not become a body, duplicate `id`s, non-finite `P`,
a Geometry that is not writable, and points with no Box3D body at write-back.
Each kind is one warning per step with a count. Fallback shapes are decided when
a body is created, so that warning appears on the frame the body is made, not
every frame.

## Debug logging

Compiled in by default (`-DBOX3DDOP_DEBUG=ON`; pass OFF for a quiet build). Output goes to
stderr, which shows up in Houdini's console window. It only prints when something happens:
`REBUILD #n: <reason>`, bodies added/removed, external edits, and warnings/failures.

Optional, set BEFORE starting Houdini (debug builds only):

```
set BOX3DDOP_REQUESTED_CACHE=0         (override the default of 1)
```

## Current state

Implemented: shared world state + stamp, staleness detection, rebuild, per-point
bodies with box, sphere, convex hull and concave (multi-hull) shapes built
from packed geometry (proxy or full detail, one Geometry input), packed-primitive support (pivot, rotation read and write-back), a hull
cache, material/mass/active/size from point attributes, point add/remove sync,
external-edit detection, write-back of P/orient/v/w, and the Box3D core (bodies,
shapes, step, transform/velocity/body type get/set), checked against the Box3D
source (`b3dop_core_physics`).

Still TODO: triangle-mesh shapes for static points, capsule, automatic convex
decomposition of concave geometry, picking up packed-geometry changes on a live body,
kinematic bodies for animated passive points (`active = 0` is static for now),
constraints (a second input on a SOP solver), live changes to shape / material
attributes, gravity/sleep changes via `b3World_SetGravity` /
`b3World_EnableSleeping` instead of rebuilding, events.

Box3D build notes: point `BOX3D_DIR` at the folder containing Box3D's
CMakeLists.txt (the inner `box3d` folder if your zip is `box3d/box3d/...`).
Large-world mode is `-DBOX3D_DOUBLE_PRECISION=ON`; it is PUBLIC on the box3d
target and `Box3DWorld.cpp` is the only file that sees Box3D headers, so the
plugin cannot end up with a precision mismatch.
