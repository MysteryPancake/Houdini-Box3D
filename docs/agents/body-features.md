# Bodies: per-attribute features

What each point attribute does to a body (sleep, CCD, locks, filters, impacts, ...). How a point becomes a body and how it
is updated: [bodies.md](bodies.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Per-attribute features

* **Centre of mass** (`computecom`, default on as Bullet's): `applyBodyMass` ->
  `applyMassByHandle(comAtOrigin = !computecom)` after creation, rebuild, retype to dynamic and mass changes. On,
  Box3D keeps the shapes' own centre of mass and the point (the body origin, a packed point's pivot) orbits it;
  velocity is the centre's. Off, the centre is moved to the point. Whole objects always move it (`wholeObjectPose`
  handles RBD State's own `computecom`).
* **Mass** (`computemass`, `density`, `inertialtensorstiffness`): `boxDensity` = density with computemass, else
  mass / volume. With computemass the mass is written to `mass` on creation, rebuild or change.
* **CCD threshold** (`ccdthreshold`) -> `b3BodyDef::safetyFactor` (Box3D's name; negatives keep the default).
* **Fast rotation** (`fastrotation`) -> `allowFastRotation`; without it Box3D caps spin at `B3_MAX_ROTATION` per Box3D
  step (the DOP step over Substeps).
* **Collision filtering** (`collision_category`, `collision_mask` read as `GA_ROHandleID` so `-1` means every bit;
  `collision_group`) -> the shapes' filter. Affector bodies use the defaults. Bullet's `collisiongroup` /
  `collisionignore` names become these bits in the Box3D Solver SOP (`collision_names` in `b3d_prepare.py`, a subnet
  on the pieces and on the colliders: an Attribute Promote (unique) per geometry, `used_groups` joins the names once,
  and the point wrangle `COLLISION_NAMES_VEX` sets each point's bits, so both sides number the groups alike): bit 0 is
  no group, groups take bits 1 to 30 in sorted order, a piece's mask lacks the groups its pattern matches. Box3D
  collides two bodies only when each mask has the other's category, but Bullet does when either piece's pattern leaves
  the other's group alone (`SIM_BulletRigidBody::checkCollideWithOverride`, in the HDK's `SIM_SolverBullet`), so
  only both ignoring each other stops them: `used_groups` also writes `__allows` (per group, the groups some of its
  pieces leave alone, over the pieces and the colliders) and the wrangle adds the groups whose pieces allow its own
  to its mask. A group whose pieces have different patterns lets all of them in. Ignoring one piece by name (rather
  than a group) and ignoring a DOP object by name are not representable.
* **Animated** (`animated` with `active = 0`): kinematic; a pose edit becomes
  `setTargetTransformByHandle(cur, stepDt)` and a step without one zeroes the velocity. Not written back. An object
  with `SourceObject` data follows its source SOP (`followSourceSops`, Box3DSourceSync.cpp, as RBD Bullet Solver):
  before the sync, each such piece (or every piece of a Kinematic Box3D Data) whose pose in that SOP at the step's end
  changed since it was last read (`Box3DSharedState::sourcePoses`; the first read compares with a step earlier) gets P
  and `orient` (since rest) written as an edit, matched by `name`. The SOP object's transform applies when the data's
  `usetransform` is on. A static SOP leaves other edits (a SOP Solver) alone.
* **Drag and speed limits** (`airresist`, `spinresist`, `drag`, `dragexp`, `targetv`, `targetw`, `speedmin`,
  `speedmax`, `spinmin`, `spinmax`; `Box3DMotion.cpp`, maths in `core/Box3DMotion.h`): what RBD Bullet Solver reads from
  POP-style attributes (`SIM_BulletRigidBody::applyDrag` / `clampSpeed`). `refreshMotion` reads them once a frame, when
  their data ids or the point count change, into `Box3DObjectSync::motion` (the points with any, by offset; not
  prop markers, since a POP node may write them every step). Drag goes in `applyToObjectBodies` (`addDrag`), last, on
  the sum of the Force DOPs and point forces, so it needs Use Force DOPs: applied implicitly as Bullet does by default
  (the velocity the body would have after the step, with gravity, is slowed by `applyImplicitDrag`, and the force that
  gives the change is the drag; spin works on the world inertia, `Box3DWorld::inertiaWorldByHandle`). Every Box3D step
  of a frame applies it, so the result depends on Substeps as Bullet's does on its own. `airresist`, `spinresist`,
  `targetv` and `targetw` are inputs for one step: `clearMotionInputs` zeroes them after the frame, as Bullet's
  `clearForces`. The limits clamp the move events after every Box3D step (`applySpeedLimits`) and the body, so a body
  gains up to a step's gravity over its limit within the step. RBD Objects get none, as in Bullet. Tests `bullet_drag_limits`, `solver_sop_speed_limit_bullet`, `solver_sop_pop_drag_bullet`,
  `core_motion`.
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
* **Impacts** (Box3D Data Add Impact Data): `recordImpacts` records Box3D's contact impulse events of the step (one
  per manifold point, in the awake set) for both bodies whose object asks (the sleeping side too, unless it is an
  asleep dynamic body), as a fresh `SIM_Impacts` each step (`addPointImpact`, normal pointing at the body that gets
  the impact; no Feedback data). `collectImpacts` decides each event's records (up to four: each awake side, itself
  and the other body) in a parallel pass, with the merge key and the earlier step's hit it joins (Substeps), then
  appends them in event order serially, so the records do not depend on the threads. `addPointImpact` is most of the
  cost, so a frame's time grows with the manifold points in contact. A glued piece run in a compound gets its records
  too ([glue-compound.md](glue-compound.md)).
* **Dead** (`dead`): never stamped, so the body goes. With Reap Dead Points the point is queued as a `Box3DDeletion`;
  `deleteDeadPoints` resolves all queued points before deleting any, then bumps data ids and forces a scan.
* **Aging** (solver Enable Aging): `age += dt` on every point (not on the creation frame without Solve On Creation
  Frame), and with a `life` attribute `dead = 1` once `age > life`, a life of 0 or less included (no `life` never
  dies), as RBD Bullet Solver (`solver_sop_aging_bullet`) and, for a positive life, POP (`aging_vs_pop*`). Reap Dead
  Points is on by default, as Bullet always deletes dead points; Reap At Frame End deletes in the same step, else the
  next sync does. The Solver SOP's age stage (an Attribute Create with Write Values off) gives the pieces `age` 0
  before the first step. Constraint primitives age too (`ageConstraints` in `Box3DJointBreak.cpp` adds the time step
  to their `age`, `create_constraint_age` makes it 0 on the first frame, `life` is ignored:
  `solver_sop_constraint_aging_bullet`). Parallel past `kMinParallelPoints`.
* **Active age** (Solver SOP only, `tools/b3d_solver_activeage.py`; Add Active Age Attribute and Just Activated under
  Output > Attributes, on while Enable Aging is): RBD Bullet Solver's two wranglers, not C++. A SOP stage before the
  DOP network gives the pieces their first `active_age` (one time step if active) and Just Activated group or attribute;
  a Geometry Wrangle DOP behind an Enable Solver DOP in the Multiple Solver, bound to its own geometry, adds the time
  step while `active` is 1 (missing counts as 1) and resets it to 0 otherwise. Test `solver_sop_active_age_bullet`.
* **Time Scale**: Box3D steps by `stepDt = dt * timescale`.

**Point writes after the step** (`Box3DPointWrite`: `kAutofitValid`, `kMass`, `kRest`): grouped per object
(`forEachObjectRun`), offsets via `resolvePointOffsets`, then `noteOwnWrite`.

**Rest transform** (packed points, Bullet's convention): `restxform` (4x4, the packed transform at rest; all zero =
unset), `pivot` (P at rest) and `orient` = the rotation since rest, so Transform Pieces needs no rest input. Bodies
stay absolute inside: `readPointState` reads `restRotation(restxform) * orient` (`box3dRestToAbsolute`) once restxform
is set, else the packed rotation; the write-back writes `orient` with `box3dAbsoluteToRest` from the entry's `restRot`
and the packed transform as before. A new packed body (`initRest`) takes `restRot` from a set restxform, else from its
packed transform stored as float (what the attribute will hold, so both paths split the same matrix) and queues
`kRest`: restxform, `pivot` = P, `orient` from the mirror's pose. As in Bullet's solver an incoming restxform is kept:
resetting it when an object is created is the object's job (the Box3D Packed Object's SOP stage,
[assets.md](assets.md)), so a world rebuilt mid-simulation keeps the rest too. Editing only a packed transform after
rest is ignored, as in Bullet: edit `orient` / `P`. Only its mirror is kept: the sign of the 3x3's determinant,
written back as a flip of the third row whichever row was flipped, with the hull rebuilt (`mirrored_midsim`). Bullet
drops the mirror too (`bullet_mirror_flip`). `restPos` (the packed pivot through restxform) is where the body's origin
was at rest; joints map authored anchors with it ([joints.md](joints.md)).
