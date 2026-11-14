# Reference: parameters, attributes, warnings

What the nodes expose and what is deliberately left out. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Parameters

Defaults, ranges and menus are defined once, in the `Box3DParms` tables: the solver's in `src/dop/SIM_SolverBox3D.cpp`,
Box3D Data's in `src/dop/SIM_Box3DData.cpp`. Look values up there; do not copy them into docs. The worker-count bounds
are `b3dop::kDefaultWorkerCount` / `kMaxWorkerCount` (`Box3DWorld.h`).

Solver (read every step; only the thread count rebuilds the world): `gravity`, `stepsperframe` (Substeps), `substeps` (Solver Passes), `autothreads`, `threads`
(default `b3dop::defaultWorkerCount()` in `Box3DCpu.h`: the machine's physical performance cores, as Box3D recommends;
`WorldSettings` keeps `kDefaultWorkerCount`), `enablesleep`, `timescale`, `enableaging`, `reapdead`, `reapatframeend` (disabled while `reapdead` is off), and Box3D's tuning
(Box3D's defaults except the contact stiffness and damping, tuned for bouncier, more rigid contacts): `contactstiffness`, `contactdamping`, `contactpushspeed`, `bouncethreshold`,
`contactrecycledistance`, `maxlinearspeed`, `enablecontinuous`. Force DOPs: `useforces`, `forceswake` (disabled while
`useforces` is off). Box3D Data: `startasleep` and `sleepthreshold` are disabled while `enablesleep` is off,
`mass` / `density` by `computemass`; joint nodes: see `enabledBy` in [joints.md](joints.md). Solver
`frictionmixing` and `restitutionmixing` menus list the rules in the same order (geometric mean, minimum, maximum,
average, multiply); `worldSettings` maps the index to each enum (their orders differ). Settings on one node that
need a setting on the other (sleep, Fast Object, impacts, contact recycling) say so in the tooltip: Houdini can't
disable across nodes.

Box3D Data (per object): `defaultshape` (menu order: box, sphere, convex hull, concave, capsule, none; the stored value
is the menu index; default convex hull), `autofit`, `hullperconnected`, `bodytype` (static, kinematic, dynamic), `computemass`, `mass`,
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
`bullet_autofit_valid`, `mass` (computemass), `age` / `dead` (aging); `dead` points lose their body, and are deleted with `reapdead` (off by default) or kept, frozen.

## Warnings surfaced on the node

One `UT_ERROR_WARNING` per kind per step with a count: shape fallbacks (on the step a shape is built), points that could
not become a body, duplicate `id`, non-finite `P`, non-finite `v` / `w` (read as zero; `readPointState`'s
`badVelocity`), Geometry not writable, bodies not matched to a point at write-back. Also: non-finite solver Gravity
(ignored, previous kept), non-finite forces from Force DOPs (skipped per body), and Box3D assertions that failed during the step (`Box3DWorld::assertCount()` delta,
reported by a scope guard in `solveObjectsSubclass`).

## Not supported on purpose

* Box3D's mover and pogo joints and character-mover queries (game character tools), explosions and wind (Houdini's
  Force DOPs already do this).
* Bullet's collision padding (`bullet_collision_margin`, `bullet_adjust_geometry`, `bullet_shrink_amount`) is ignored:
  Box3D collides the exact shapes with speculative contacts.
* Bullet's solver tuning (ERP, CFM, split impulse, iterations, solve tolerance, randomized order) is not mapped; Box3D's
  own settings are offered.
* Bullet's two sleep thresholds, Sleeping Time and `deactivation_time` are replaced by Box3D's one threshold.
* `min_activation_impulse` (inactive bodies that activate when hit): Box3D has no equivalent, and it would need a
  contact scan after every step.

## Parameter tabs and folders

Every node's parameters come from `Box3DParms` (SIM_Box3DParms.h): one call per tab or folder and per parameter
(`toggle`, `number`, `integer`, `vector`, `menu`; ranges as `Range{ min, max, hardMax }` with a hard minimum; an
optional disable condition). It keeps the names, defaults, ranges, menus and conditions alive and terminates the list.
`tab()` groups share one `PRM_SWITCHER` (`PRM_Default(count, label)` per tab, switcher `b3dtabs`); `folder()` groups
are simple folders, each a one-folder switcher with `PRM_SpareData::groupTypeSimple` named `<prefix><index>`. A builder
uses one kind, except `subfolder()`: a simple folder inside a tab (`<prefix>sub<index>`), counted as one parameter of
the tab. The solver uses tabs: Solver (time scale, Substeps, Solver Passes), Collision (contact tuning, recycling),
Forces (gravity, bounce, friction / bounce mixing), Misc (simple folders Sleeping: Allow Sleeping, Forces Wake Bodies;
Aging: Enable Aging, Reap Dead Points, Reap At Frame End), Advanced (Warm Starting, Continuous Collision, Use Force
DOPs, threads, speed limit, ...); its guide parameters are a plain list on Houdini's own Guide Options tab (so the
solver's tabs sit inside Data Options). Box3D Data and the joint nodes use simple folders: Box3D Data's are Shape, Body, Material, Sleep & Contacts (CCD Threshold, Fast Object, Contact Recycling, the
sleep settings, Add Impact Data last). A function-local static, because Houdini keeps pointers into it. Menu tokens are "0", "1", ... in item order (read as ints).
