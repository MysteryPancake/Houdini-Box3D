# Reference: parameters, attributes, warnings

What the nodes expose and what is deliberately left out. Index and rules: [AGENTS.md](../../AGENTS.md).

## Parameters

Defaults, ranges and menus are defined once, in the `Box3DParms` tables: the solver's in `src/dop/SIM_SolverBox3D.cpp`,
Box3D Data's in `src/dop/SIM_Box3DData.cpp`. Look values up there; do not copy them into docs. The worker-count bounds
are `b3dop::kDefaultWorkerCount` / `kMaxWorkerCount` (`Box3DWorld.h`).

Solver (read every step; only the thread count rebuilds the world): `gravity`, `substeps`, `autothreads`, `threads`
(`WorldSettings`' default is the same constant), `enablesleep`, `timescale`, `enableaging`, and Box3D's tuning with
Box3D's defaults: `contactstiffness`, `contactdamping`, `contactpushspeed`, `bouncethreshold`,
`contactrecycledistance`, `maxlinearspeed`, `enablecontinuous`. Force DOPs: `useforces`, `forceswake` (disabled while
`useforces` is off).

Box3D Data (per object): `defaultshape` (menu order: box, sphere, convex hull, concave, capsule, none; the stored value
is the menu index), `autofit`, `hullperconnected`, `bodytype` (static, kinematic, dynamic), `computemass`, `mass`,
`density`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`, `enablesleep`, `sleepthreshold`,
`fastobject`, `contactrecycling`, `addimpact`. Names match the point attributes that override them; material values
have a hard minimum of 0 (Box3D asserts on negatives). `toBodySettings()` maps them to `b3dop::BodySettings`.

## Point attributes

`P` (required), `orient` (not for packed points), `v`, `w` (rad/s), `id`, `active`, `animated`, `mass`, `computemass`,
`density`, `inertialtensorstiffness`, `friction`, `bounce`, `lineardamping`, `angulardamping`, `gravityscale`,
`enablesleep`, `sleepthreshold`, `ccdthreshold`, `awake`, `enabled`, `fastobject`, `fastrotation`, `contactrecycling`,
`collision_category`, `collision_mask`, `collision_group`, `lock_linear`, `lock_angular`, `dead`, `age`, `life`,
`deforming`, `bullet_autofit_valid`, `bullet_georep`, `bullet_autofit`, `bullet_primS` / `R` / `T`, `bullet_radius`,
`bullet_length`, `pscale`, `scale`. Negative material values are clamped to 0, non-finite ones ignored (`gravityscale`
may be negative). Identity is `(object id, id or point number)`.

Written: `P`, `orient`, `v`, `w` and packed transforms (awake dynamic bodies), `awake` (if it exists),
`bullet_autofit_valid`, `mass` (computemass), `age` / `dead` (aging); `dead` points are deleted.

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fallbacks (on the step a shape is built), points that could
not become a body, duplicate `id`, non-finite `P`, non-finite `v` / `w` (read as zero; `readPointState`'s
`badVelocity`), Geometry not writable, bodies not matched to a point at write-back. Also: non-finite solver Gravity
(ignored, previous kept), non-finite forces from Force DOPs (skipped per body), and Box3D assertions that failed during the step (`Box3DWorld::assertCount()` delta,
reported by a scope guard in `solveObjectsSubclass`).

## Not supported on purpose

* Box3D's mover and pogo joints (`b3CreateMoverJoint`, `b3CreatePogoJoint`) and character-mover queries are left out:
  they are game character controller tools and have no place in a Houdini DOP workflow. Likewise Box3D's explosions
  (`b3World_Explode`) and wind (`b3Shape_ApplyWind`): Houdini already has Force DOPs (Wind, Fan, POP Force, ...), which the
  solver applies, so a second way to do the same thing would only add confusion.
* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* `min_activation_impulse` (inactive bodies that activate when hit) was removed: Box3D has no equivalent, and it needed a contact scan after every step.

## Parameter tabs

Every node's parameters come from `Box3DParms` (SIM_Box3DParms.h): one call per tab and per parameter (`toggle`,
`number`, `integer`, `vector`, `menu`; ranges as `Range{ min, max, hardMax }` with a hard minimum; an optional
disable condition). It keeps the names, defaults, ranges, menus and conditions alive, counts each tab's parameters for
the `PRM_SWITCHER` (`PRM_Default(count, label)` per tab) and terminates the list. A function-local static, because
Houdini keeps pointers into it. Menu tokens are "0", "1", ... in item order (read as ints).
