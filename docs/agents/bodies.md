# Bodies: updates, features, shapes

Part of the solver step: how a point becomes a body and how it changes mid-simulation. The step order is in [solver.md](solver.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Mid-simulation updates

Target behaviour is Houdini's Bullet solver. A new body is built from the same readers, so an untouched point compares
equal to what its body was built with.

* **Settings** (`readDynamicProps` -> `applyPropChanges`): the effective (attribute, else Box3D Data) values in
  `b3dop::BodyProps` (core, `Box3DWorld.h`; `BodySettings::props` embeds it), compared with `entry->props`; only the
  setters for what differs run. Compared only when `checkProps`
  (prop markers changed, none recorded yet, or deforming bodies); a scan triggered by an animated `P` alone does not.
  * Mass, `computemass`, `density`, `inertialtensorstiffness` -> `applyBodyMass` (same as a fresh build).
  * Sleep and enable state: `enablesleep` (else the node) -> `setSleepEnabledByHandle`, live; `enabled` ->
    `setEnabledByHandle`. `awake` is not a setting: see "Sleep and enable state".
* **Shape rebuild** (`rebuildPointShapes`) on `deforming` (every step), `bullet_autofit_valid == 0` (once) or, with
  `autoScale` (convex / concave only, as in Houdini), a size-key change beyond 1e-4 relative (`sizeKey` = packed row
  lengths or `pscale * scale`; the tolerance absorbs our own write-back's float noise). `planShapes` (the same code as a
  new body) gives a `shapeSignature` (hull keys + primitive numbers); equal -> nothing touched. Otherwise
  `destroyShapesByHandle`, `addShapes`, `applyBodyMass` (even with no shapes, to clear Box3D's dirty mass),
  `wakeByHandle`. The body keeps its handle, pose, velocity and identity. A plan of `none`, or shapes that all fail,
  leaves the body `shapeless`, hence static.
* Editing `bullet_georep`, `bullet_autofit`, `bullet_prim*`, `bullet_radius`, `bullet_length` or a primitive's size
  alone does nothing (as in Bullet). Box3D Data `defaultshape` / `autofit` / `hullperconnected` apply at creation or
  rebuild.
* **`bullet_autofit_valid`**: every point built or rebuilt while it was 0 gets 1 written after the step. Unlike Bullet,
  fitted values are not written into `bullet_prim*` / `bullet_radius` / `bullet_length`.
* **`id`** is the identity key with no special values: a different `id` is a different body. The `id = -1` "force a
  rebuild" trick some notes describe is deliberately not implemented.

## Bullet features

* **The point is the centre of mass**, as in Houdini RBD (for a packed point, its pivot, which hulls are built around).
  `applyBodyMass` calls `applyMassByHandle(..., comAtOrigin = true, stiffness)` after creation, rebuild, a retype to
  dynamic and mass changes. Velocities are therefore the point's. No `computecom`: an off-centre pivot makes
  the piece turn about it, like in Houdini.
* **CCD threshold** (`ccdthreshold`, float; node `ccdthreshold`, default = Box3D's): `BodyProps::ccdThreshold`
  -> `b3BodyDef::safetyFactor` (negative / non-finite keep Box3D's default; Box3D asserts on negatives); changes use
  `setCcdThresholdByHandle` (`b3Body_SetSafetyFactor`). A body is "fast" (swept, contacts not recycled) when a step's
  motion > threshold x its min extent. Renamed from Box3D's "safety factor" so the name says what it does.
* **Fast rotation** (`fastrotation`, int): `BodyProps::fastRotation` -> `b3BodyDef::allowFastRotation`; changes use
  `setFastRotationByHandle` (`b3Body_AllowFastRotation`). Without it Box3D caps angular speed at `B3_MAX_ROTATION`
  (pi/4) per Box3D step, i.e. per DOP step here (the solver's Substeps are inside one step and don't raise it).
  No node default: Box3D says round bodies only.
* **Motion locks** (`lock_linear`, `lock_angular`: vec3, nonzero component = locked world axis): `readDynamicProps`
  fills `BodyProps::locks` -> `b3BodyDef::motionLocks`; changes use
  `setMotionLocksByHandle` (`b3Body_SetMotionLocks`, which zeroes the newly locked velocity components). Box3D zeroes
  locked components of `v` / `w` every iteration, so locked bodies also ignore `v` / `w` and forces on those axes.
* **Collision filtering** (`collision_category`, `collision_mask` read as `GA_ROHandleID` so a 32-bit `-1` sign-extends
  to every bit; `collision_group` int): `readDynamicProps` fills `BodyProps::filter`, which `shapeDefFor`
  puts on the shapes; changes use `setFilterByHandle`. Missing attributes keep Box3D's defaults. Affector bodies use
  the defaults.
* **Animated** (`animated` with `active` = 0): Kinematic. A pose edit becomes `setTargetTransformByHandle(cur, stepDt)`
  with `driven = true`; a step without one zeroes a driven body's velocity (so its object never skips while driven). Not
  written back; `v` / `w` are not applied.
* **Mass properties** (`computemass`, `density`, `inertialtensorstiffness`; node `computemass` / `density`):
  `boxDensity` = density with computemass, else mass / volume. With computemass the mass is written to `mass` (created
  if missing) on creation, rebuild or change.
* **Sleep threshold** (`sleepthreshold`, node default in `SIM_Box3DData.cpp`): Box3D's single threshold and fixed half second
  (`B3_TIME_TO_SLEEP`).
* **Sleep and enable state** (`enablesleep`, `awake`, `enabled`): one attribute per Box3D concept
  (`b3BodyDef::enableSleep` / `isAwake` / `isEnabled`).
  * `enablesleep` (node: Allow Sleeping) and `enabled` are settings (`BodyProps`) and change
    live. `enabled` is `def.isEnabled` at creation (no create-then-disable); toggling later is `b3Body_Enable` /
    `Disable`, which Box3D documents as expensive. A disabled body has no move events, so it is not written back.
  * `awake` is state, not a setting. At creation it is `def.isAwake` (the point's value, else the node's Start Asleep).
    Box3D starts a body that may not sleep awake regardless. The initial `v` / `w` are skipped for a body that is not
    awake after creation, because `b3Body_SetLinearVelocity` wakes a sleeping body.
  * `Box3DBodyMirror::awake` is the value of the attribute as the solver last read or wrote it, not the live Box3D state.
    The write-back compares it with each move event (`!fellAsleep`) and writes the attribute, if it exists, only when
    they differ. A `checkProps` scan (`awake` is a prop marker) compares the attribute with the mirror: a difference is a
    user write, and applies `wakeByHandle` or `sleepByHandle`, then updates the mirror. Edge-triggered on purpose: a
    level check would keep retrying a sleep Box3D refused, or wake a sleeping pile on every unrelated prop change.
    `p.enabled == false` skips the check; enabling sets the mirror to awake (Box3D enables a body awake), so an `awake`
    of 0 written with it puts the body to sleep right after.
  * `sleepByHandle` does nothing for static or disabled bodies, a body with sleeping off, or when the world's sleeping is
    off. `b3Body_SetAwake(false)` sleeps the whole island and Box3D may refuse (a pending island split), in which case
    the next move event writes `awake = 1` back.
  * Known limitation: forcing a body to sleep also sleeps the bodies touching it, but Box3D reports that through no move
    event (forced sleep between steps modifies the already-consumed event array), so their `awake` attributes and mirrors
    keep saying awake until they wake and fall asleep normally. A shape rebuild wakes the body (`wakeByHandle`).
* **Fast object** (`fastobject`): Box3D's `isBullet`, renamed because Houdini's `isbullet` means "treat as a Bullet
  object". Changes mid-simulation.
* **Contact recycling** (`contactrecycling`, Box3D Data toggle default on, int point attribute): Box3D's per-body
  `enableContactRecycling` (`BodyProps::contactRecycling` -> `b3BodyDef`). The distance is world-wide:
  `WorldSettings::contactRecycleDistance` (solver `contactrecycledistance`, default = Box3D's `10 * B3_LINEAR_SLOP`
  at its default length units; 0 = off). It is not in `b3WorldDef`, so `create` sets it after `b3CreateWorld`, and
  `applyTuning` pushes changes through `b3World_SetContactRecycleDistance` (non-finite: ignored, previous kept, like
  gravity; negative acts as 0). A pair recycles only if both bodies have the flag (and both shapes keep Box3D's speculative flag, always on here)
  (`contact.c`). Box3D copies the body flag into each contact's `b3_contactRecycleFlag` when the contact is created, and
  `b3Body_EnableContactRecycling` only changes the body, so existing contacts keep the old behaviour.
  `setContactRecyclingByHandle` therefore also rebuilds them for an awake dynamic body: `b3Body_Disable` + `b3Body_Enable`
  destroys and recreates its contacts, but zeroes velocity and wakes, so the velocity is saved and restored. Sleeping,
  static and kinematic bodies are not cycled (a cycle would wake them): they take it for new contacts only. A point-attribute
  or node change goes through the usual prop path (`kPropAttribNames`, the fingerprint's `nodeProps`, `applyPropChanges`).
  `Box3DWorld::recycledContactCount()` (`b3World_GetCounters`) feeds the tests and a verbose log line.
* **Rolling resistance and conveyor velocity** (`rollingresistance` float, `tangentvelocity` vector; Box3D Data
  `rollingresistance` / `tangentvelocity`): Box3D's `b3SurfaceMaterial::rollingResistance` and `tangentVelocity`, set in
  `shapeDefFor` (`BodyProps::rollingResistance`, `tangentVelocity[3]`; negative or non-finite values become 0, since
  Box3D asserts on them). Rolling resistance is the larger of the two shapes' values times the larger radius (hulls use
  `0.25 * innerRadius`, `contact.c`), capped by the normal impulse. Tangent velocity is local to the shape, rotated by the
  body each step and projected onto the contact surface; the friction row drives the relative tangent velocity to it
  (the body on the belt moves with `+tangentVelocity`), so friction must be above 0. Meshes use material 0 (the wrapper
  builds them without per-triangle materials). Live changes: `setSurfaceByHandle` read-modify-writes
  `b3Shape_SetSurfaceMaterial` on every non-compound shape (friction and bounce are untouched). That alone is not
  enough: Box3D skips the material update for a *recycled* contact (`physics_world.c`, "skips updating ... material
  parameters") and a sleeping body does not update its contacts, so a resting box on a belt that is switched on would stay
  put. On a real change `setSurfaceByHandle` therefore calls `rebuildContacts` (`b3Body_Disable` + `Enable`, which
  destroys the body's contacts, wakes everything touching it and re-creates its pairs with `forcePairCreation`; works for
  static bodies; velocity restored for the others, shared with `setContactRecyclingByHandle`). A sleeping dynamic body is
  not cycled; it takes the change when it wakes. Affector (Static Object) meshes read the values from their Box3D Data
  node but keep them out of the shape signature, so an animated belt speed updates the shapes
  (`Box3DAffectorBody::rolling` / `tangent`) instead of rebuilding the mesh. Point path: `kPropAttribNames`,
  the fingerprint's `nodeProps`, `readDynamicProps`, `applyPropChanges`. Tests: "surface material" block in `core_physics.cpp`.
* **Impacts** (node `addimpact`, Add Impact Data; no attribute): `recordImpacts` walks this step's awake bodies (the
  move events), reads each one's contacts once and records them for both bodies whose object has the option: the awake
  body itself, and the other one when it is not awake (an awake one records its own) and not an asleep dynamic body. So
  sleeping bodies cost nothing and contacts where neither body is awake are skipped. Only when there is at least one does it create a fresh `SIM_Impacts`
  (`SIM_DATA_CREATE` with flags 0 replaces existing data) and call `addPointImpact(pos, normal, impulse, ptnum, otherobjid, otherptnum, -1, 0, 0,
  0, time, 0)` per point (normal of the surface hit, pointing at the body that gets the impact; ptnum = its `pointIndex`). An object
  with Impacts last step and none now loses the data (`removeNamedSubData`, `impactObjects`). No Feedback data (that is
  for mutual interaction with other solvers).
* **Dead** (`dead`): never stamped, so its body goes like a gone point's. With solver Reap Dead Points
  (`st.reapDead`, off by default; POP's Reaping is on by default) it is also queued as a `Box3DDeletion`; off, the
  point stays where it died with `dead = 1` (its `v` / `w` keep their last values). `deleteDeadPoints` checks every queued point (offset, `id` or number)
  before deleting any (`destroyPrimitive(prim, true)` for packed, else `destroyPointOffset(off,
  GA_DESTROY_DEGENERATE)`), then `bumpAllDataIds()` and `forceScan`. Without `id`, later points become new bodies.
* **Aging** (solver Enable Aging): `age += dt` (DOP step, not time-scaled) on every point of every solved object (not
  on an object's creation frame unless its node has Solve On Creation Frame on, as POP Objects do by default), `dead = 1` once it exceeds a positive
  `life` (strictly, as POP), and, with Reap Dead Points and Reap At Frame End (`st.reapAtFrameEnd`, on by default as POP's), the point
  queued in `st.deletions` so `deleteDeadPoints` removes it the same step; with Reap At Frame End off it is kept that
  frame with `dead = 1` (POP: deleted before the next Pre-solve) and the next step's sync queues it (attributes created if missing). Writes every point every step while on; in parallel past `kMinParallelPoints` (`UTparallelFor` over a `GA_SplittableRange`, no offset vector), with
  `age` / `dead` pages hardened first.
* **Time Scale**: Box3D steps by `stepDt = dt * timescale`; animated targets use it; `canContinue` uses the DOP step.

**Point writes after the step** (`Box3DPointWrite`: `kAutofitValid`, `kMass`): grouped per object
(`forEachObjectRun`: a stable O(n) bucket pass, runs in first-seen order, nothing moved if already grouped), offsets via the templated `resolvePointOffsets`, then `recordFingerprintAfterWrite`.

## Building shapes (`addPointBody`, `planShapes`, `buildShapes`)

* Defaults: Box3D Data's (`nodeBodyDefaults()` in Box3DWorld.h, Bullet's RBD values where Box3D has the setting:
  Compute Mass with density 1000, friction 1, bounce 0.5), which objects without a Box3D Data node get too. The
  solver's mixing defaults are Bullet's as measured by `bullet_mixing`: friction multiplied, the smaller bounce.
  `BodySettings()` keeps the core's own values (mass 1, friction 0.6, bounce 0) for direct use of `Box3DWorld`, and
  `WorldSettings` keeps Box3D's rules (geometric mean, maximum).
* Type: `wantedType` (active -> node type; inactive -> static, or kinematic if animated; shapeless -> static).
* `readDynamicProps` gives the `BodyProps`; the `BodySettings` a body is created with is the node's, with the type and
  those props put in (density from `boxDensity`); `planShapes` the shape type (`bullet_georep`, else node
  `defaultShape`, Convex by default as in RBD Packed Object; a plain point whose shape comes from the default, not from
  `bullet_georep`, gets the pscale box silently instead of a hull fallback note) and pieces; `createBodyWithShapes` (shared with whole objects) makes the body, adds them
  (`addShapes`), `applyBodyMass`, then the initial `v` / `w` (after the mass, because a moving centre of mass shifts the
  velocity; not for static or animated bodies, nor for one that is not awake).
* The mirror entry is inserted (`try_emplace`) before the body exists and becomes its user data; it is erased again if
  the body cannot be built. A body is never left without shapes, except `none`. Many new bodies in one scan (a fresh world) reserve the
  mirror once (`reserveMirror`).
* `buildShapes`:
  1. Packed point: its geometry into `PieceGroup`s via `ShapeSources` (cached per source detail, and per
     `(source, first primitive)` for packed fragments, for one `syncObject`).
  2. Packed convex / concave: `pieceFromPoints` with a `packedHullKey` (by content if the transform is degenerate):
     `findHull` first, body-local points only on a miss, else a fitted box with a note. Convex merges all pieces
     (kind 1) unless hull-per-connected; concave is one hull per piece (kind 2).
  3. Box / sphere / capsule on a packed point with autofit: `Box3DWorld::fitShapeOriented` (see "AutoFit" below).
  4. Nothing usable (plain point, autofit off, empty geometry): `makeShapeSpec(pscale * scale)`, plus
     `applyPrimOverrides` only with autofit off; convex / concave fall back to a box with a note. RBD Packed Object
     writes default `bullet_primS` (1, 1, 1) and `bullet_radius` 1 onto every point (seen in Houdini 22.0), so
     applying them with autofit on made every plain point a size-2 box whatever its `pscale`.
* Notes are aggregated into one node warning per kind, on the step a shape is built.
* Point attributes override Box3D Data, as for Bullet (Box3D Data only fills in missing ones). RBD Packed Object
  writes several onto every point from its own parameters (`mass`, `density`, `computemass`, `friction`, `bounce`
  in Houdini 22.0), so its Box3D Data values for those do nothing. `noteOverriddenData` (Box3DPointSync.cpp) lists
  the attributes present that replace a Box3D Data value, on objects that have a Box3D Data node, once per object
  (first full scan, or when the set changes, `Box3DObjectSync::overridesReported`); the solver turns the step's
  total into one warning. The Box3D Data tooltips of those five say so.

**AutoFit** (`Box3DWorld::fitShapeOriented`, also for whole objects): the frame with the smallest box among the body
axes and, from a throwaway `b3CreateHull` of the points, each face normal paired with each of that face's edges (exact
for boxes and prisms; a fractured piece gets a face of its hull flush). A turned frame must beat the body axes by
`kTurnedFitGain` (Box3DWorld.cpp), so geometry along the body axes keeps them exactly; `canonicalFrame` picks the
equivalent frame closest to the body axes. In that frame (`fitShapeInFrame`, Box3DShapes.h), as measured against
Bullet (`autofit_bullet`: its sphere on a 0.6 x 0.4 x 0.8 box has radius 0.538, the half diagonal; its capsule radius
0.5, around the box's own x-z section): box = the bounds, turned (a turned box is an 8-corner hull, as for
`bullet_primR`); sphere = centred on them, through the farthest point; capsule = along the frame's second axis (the
one nearest the body's Y), centred on the bounds, radius through the point farthest from the axis, caps ending where
the bounds end along it (a sphere when the radius is larger; Bullet's capsule on a standing 0.4 x 1.2 x 0.4 piece tops
out 0.6 above its centre). Extents use every point. Not
cached: each AutoFit body builds one hull at creation. `fitShapeToPoints` (body axes only) is the fallback when a
piece cannot be hulled.

**Explicit primitives** (Bullet Data with AutoFit off; `bullet_autofit` attribute, else the node toggle, default on):
for `box`, `sphere`, `capsule` only. `bullet_primS` = box HALF extents (Bullet Data documents Box Size as a half-size);
`bullet_radius`; `bullet_length` = capsule straight section, caps not included; `bullet_primT` = offset;
`bullet_primR` = Euler degrees, XYZ, turned into a quaternion with `UT_QuaternionF(rad, UT_XformOrder(TRS, XYZ))`. A
rotated box is an 8-corner hull; a sphere ignores rotation. The rotation order and the length's meaning live only in
that conversion and the `hasLength` line of `applyPrimOverrides`.

**Pieces and packed geometry.** Pieces are grouped by the primitive `name` attribute (unnamed share one; no primitives:
one piece). Collectors gather point offsets; `finishPieces` sorts and dedupes them, reads the positions (offset order)
and hashes each piece and the group. `hullperconnected` (convex only): one hull per connected set, ignoring `name`. Packed fragments
(`GU_PackedFragment`, Pack SOP with `name`) share one source detail, read with `GU_PackedFragment::detailPtr()`
(not `getPackedDetail()`, which is a different detail from the one `getPrimitiveRange()` indexes: using it gave
fragments wrong or empty pieces, the empty ones falling back to a pscale-sized box); a fragment's points are one piece, keyed by
its first primitive (GU_PackedFragment does not expose its attribute value; no two fragments share a primitive),
pivot and scale. The pivot is `GEO_PrimPacked::getPivot`; the body origin is the point's `P`.
Packed rotation is read from `getFullTransform4` (not `orient`) and written with `setLocalTransform`;
`splitPackedMatrix` (`UT_Matrix3D::extractScales` + `UT_QuaternionD::updateFromRotationMatrix`; shear dropped,
a mirror flips the third row) and `composePackedMatrix` (`getRotationMatrix`, per-row scale kept) use the HDK, so
packed rotations follow Houdini's own quaternion convention.
