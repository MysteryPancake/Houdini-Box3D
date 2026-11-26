# Reference: parameters, attributes, warnings

What the nodes expose and what is deliberately left out. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Parameters

Names, defaults, ranges and menus are defined once, in the `Box3DParms` tables: the solver's in
`src/dop/SIM_SolverBox3D.cpp`, Box3D Data's in `src/dop/SIM_Box3DData.cpp` (`toBodySettings()` maps them to
`b3dop::BodySettings`). Look values up there; do not copy them into docs. The worker-count bounds are
`b3dop::kDefaultWorkerCount` / `kMaxWorkerCount` (`Box3DWorld.h`).

* Solver: read every step, except the thread count, which rebuilds the world. `threads` defaults to
  `b3dop::defaultWorkerCount()` (`Box3DCpu.h`: the machine's physical performance cores, as Box3D recommends). The
  Box3D tuning settings are Box3D's defaults except the contact stiffness and damping, tuned for bouncier, more rigid
  contacts. `gluecompound` and `gluecompoundload`: [glue-compound.md](glue-compound.md).
* Disabled conditions: `reapatframeend` while `reapdead` is off, `forceswake` while `useforces` is off, Box3D Data's
  `startasleep` / `sleepthreshold` while `enablesleep` is off, `mass` / `density` by `computemass`. Joint nodes:
  `enabledBy` in [joints.md](joints.md). Settings on one node that need a setting on the other (sleep, Fast Object,
  impacts, contact recycling) say so in the tooltip: Houdini can't disable across nodes.
* The solver's `frictionmixing` and `restitutionmixing` menus list the rules in the same order (geometric mean, minimum,
  maximum, average, multiply); `currentWorldSettings` maps the index to each enum (their orders differ).
* Box3D Data's `defaultshape` stores the menu index (order: box, sphere, convex hull, concave, capsule, none, plane,
  cylinder; default convex hull). Its names match the point attributes that override them. Material values have a hard
  minimum of 0 (Box3D asserts on negatives).

## Point attributes

The list the sync reads by name is `Box3DAttribNames.h` (user reference: [attributes.md](../attributes.md)). Negative
material values are clamped to 0, non-finite ones ignored (`gravityscale` may be negative). `name` is read for joint
anchors. Identity is `(object id, id or point number)`.

Written: `P`, `orient`, `v`, `w` and packed transforms (awake dynamic bodies), `awake` (if it exists), `restxform` /
`pivot` / `orient` on new packed bodies without a rest ([bodies.md](bodies.md)), `bullet_autofit_valid`, `mass`
(computemass), `age` / `dead` (aging), `compound_id` (Write Compound IDs: `queueCompoundIds`), `force` / `torque`
zeroed once applied; `dead` points lose their body, and are deleted with `reapdead` (on by default) or kept, frozen.
Constraint networks get `broken`, feedback and live frames ([joint-networks.md](joint-networks.md)); whole objects their `Position`
data ([whole-object.md](whole-object.md)).

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fallbacks (on the step a shape is built), points that
could not become a body, duplicate `id`, non-finite `P`, non-finite `v` / `w` (read as zero; `readPointState`'s
`badVelocity`), Geometry not writable, bodies not matched to a point at write-back, whole objects that could not
become a body or whose Position data cannot be written, affectors with no usable collision geometry, constraint
primitives that name no known relationship, name a missing piece, carry `constraint_type` position / rotation on one
of our nodes, are not two-point lines or could not be made, and point attributes that override a Box3D Data value
changed from its default (once per object; RBD Packed Object writes several). Errors: world creation failed or the
world limit was reached (`reportWorldCreationFailure`), step cancelled (a message). Also: non-finite solver Gravity
(ignored, previous kept), non-finite forces from Force DOPs (skipped per body), and Box3D assertions that failed
during the step (`Box3DWorld::assertCount()` delta, reported by a scope guard in `solveObjectsSubclass`).

## Not supported on purpose

* Box3D's mover and pogo joints and character-mover queries (game character tools), explosions and wind (Houdini's
  Force DOPs already do this).
* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* Bullet's cap on a SideFX Spring: its Spring limits the stiffness to m / (16 dt^2) and the damping to m / dt, dt
  being the frame time over Bullet's Substeps (measured: `bullet_spring_cap`; Strength 1000 on 1 kg sags 0.27 m at
  Substeps 1 and 0.017 at 4). `springToHertz` passes the full Hooke's constant: a Box3D spring is implicit and stable
  at any frequency (only the rigid joint constraint is clamped, to a quarter of the solver step rate), and a cap taken
  from our own Substeps (default 2) would be about 25 times softer than Bullet's at its default of 10. So a Spring
  stiffer than that cap is stiffer here than in Bullet.
* Bullet's restitution velocity threshold is 0 (any approach speed bounces); the solver's Bounce Threshold defaults to
  Box3D's, so slow collisions settle instead of jittering. The bounce itself is Bullet's: the two bounces multiplied
  (`bullet_mixing`).
* `min_activation_impulse` (inactive bodies that activate when hit): Box3D has no equivalent, and it would need a
  contact scan after every step.

## Parameter tabs and folders

Every node's parameters come from `Box3DParms` (SIM_Box3DParms.h): one call per tab or folder and per parameter
(`toggle`, `number`, `integer`, `vector`, `menu`; ranges as `Range{ min, max, hardMax }` with a hard minimum; an
optional disable condition). It keeps the names, defaults, ranges, menus and conditions alive and terminates the list.
`tab()` groups share one `PRM_SWITCHER` (`PRM_Default(count, label)` per tab, switcher `b3dtabs`); `folder()` groups
are simple folders, each a one-folder switcher with `PRM_SpareData::groupTypeSimple` named `<prefix><index>`. A
builder uses one kind, except `subfolder()`: a simple folder inside a tab (`<prefix>sub<index>`), counted as one
parameter of the tab. The solver uses tabs by subject, laid out like the Box3D Solver SOP's (same tab order and folder
names): Solver (time scale, Substeps, Constraint Iterations; simple folders Contacts, Sleeping, Aging), Forces
(gravity; folders Force DOPs, Bounce / Friction, Glue), Visualization (folder Geometry: Export Collision Shapes),
Advanced (folders Stability, Mass & Inertia, Ground: Box Size, and Performance) and Output (folder Impact Data); its guide
parameters are a plain list on Houdini's own Guide Options tab (so the solver's tabs sit inside Data Options). Box3D
Data and the joint nodes use simple folders: Box3D Data's are Shape, Body, Material, CCD & Contacts (CCD Threshold,
Fast Object, Contact Recycling, Add Impact Data) and Sleep (Allow Sleeping, Start Asleep, Sleep Threshold). A
function-local static, because Houdini keeps pointers into it. Menu tokens are "0", "1", ... in item order (read as
ints).
