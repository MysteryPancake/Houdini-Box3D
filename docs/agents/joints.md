# Joints

Constraint networks and our joint nodes. SideFX constraint data and Glue: [joints-sidefx.md](joints-sidefx.md); Glue as One Body: [glue-compound.md](glue-compound.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Joints

Box3D's joints, set up the way Houdini's old constraint system is: a **Constraint Network** DOP holds the network
geometry, and constraint relationship nodes wired into its second input hold the settings, one node per kind of
constraint, named by the primitives' `constraint_name`. Here those nodes are ours, one per Box3D joint type, with Box3D's
own settings and defaults. SideFX's own constraint data (Glue, Hard, Spring, Bullet Soft, Cone Twist) is mapped onto
Box3D joints, approximately ([joints-sidefx.md](joints-sidefx.md)). `constraint_type` (position / rotation / all) is read
for SideFX data only; for our nodes the joint type decides what is constrained, on purpose (a generic joint in the fork would be
needed to honour it), and a `position` or `rotation` on one warns (`typeIgnored`, test `constraint_type_ignored_warning`). Character-mover joints (mover, pogo)
are left out.

Naming: attributes holding the state at the initial pose are called `rest*` (`restlength`), or are Houdini's own
(a network point's `P` / `orient`: the anchor's initial world frame, never written); attributes the solver keeps
current are `current_*` (`current_P` / `current_orient` on network points, and the per-type readings). The solver writes no attribute it does
not read itself, except feedback (`force`, `distance`, `torque`, `angle`, `current_*`), as the Bullet solver does.

**Core** (`Box3DJoints.h`, `Box3DWorld`, tested in `tests/core_joints.cpp`):
* `JointType` (weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter, motionlock) and `JointSettings`:
  every setting of every type in Box3D units (radians, rad/s, Hz). Box3D's FLT_MAX caps are negative values here
  (unlimited): break thresholds, parallel `maxTorque`, distance `maxLength`, `maxTensionForce`,
  `maxCompressionForce` (Box3D's signed `lowerSpringForce` / `upperSpringForce`, renamed so the sign cannot be got
  wrong). Distance `restLength < 0` (`restlength`) = the anchors' distance when the joint is made. Weld: `enableLinearSpring` /
  `enableAngularSpring` (off = rigid; Box3D's own rule is "hertz 0 = rigid", which reads backwards with the name
  stiffness). Every Box3D "hertz" is called stiffness.
* `jointParams()`: one row per setting (attribute / parameter name, label, kind, joint types using it, member
  pointer, help). The constraint relationship nodes, the primitive attribute reader and `sameJointSettings` are all driven by
  it. `angleTypes` marks a Float that is an angle for some types only (`motor_speed`: revolute deg/s, prismatic and
  distance units/s). `Rotation` is Euler XYZ degrees in Houdini and a quaternion in the core.
  `enabledBy` names the toggle that switches a setting on (spring, motor, limit values); `jointParamDisableWhen(p, type)`
  turns it into the node's disable-when string, following chains (steering limits need Steering) and skipping toggles
  the type lacks (the parallel joint's spring and the motor joint's springs have none). Only the UI uses it: a disabled
  parameter is still read, and its primitive attribute still overrides it.
  `jointParamUiOrder(type)` is the order the node shows: table order, but each gated setting moves to just below its
  toggle, so a toggle only greys out the block under it. Reordering is safe (parameters are saved by name); keep
  always-on settings out of a toggle's block (why `restlength` is near the top).
  `jointParamIndices(type)` is the table filtered to the rows a type uses (built once, table order); `sameJointSettings`
  runs for every joint every step and walks only those rows. Per-type loops over the table should use it.
* `Box3DWorld::defaultJointSettings(type)` copies `b3Default*JointDef`. The nodes start from `nodeJointDefaults(type, ...)`
  over it, since Box3D's spring stiffness, motor effort and limits are 0 (a toggle would do nothing, or lock a limit at
  0..0). Springs start at Bullet Soft's 10 Hz, damping ratio 1. Only the node UI uses these.
* `jointNodeToggles()`: node-only toggles (Breakable, Set Rest Length, Limit Spring Force, Limit Max Length, Limit
  Torque) in place of typing -1 for "never / unlimited". Gated settings name them in `enabledBy`; `settings()` writes -1
  into them while the toggle is off (`jointNodeToggleOf`), so the settings, primitive attributes (floats
  multiply the node's -1) and the SideFX mapping keep their meaning. Not in `jointParams()`: no attribute,
  no fingerprint, no `sameJointSettings` entry. `findJointParam` searches both lists.
* `jointParamRange(p, type)`: each float's slider; a hard 0 minimum where `sanitized` (Box3DWorldJoints.cpp) would clamp
  anyway. `motor_speed` has an angle range on revolute and a linear one elsewhere.
* Houdini units in one place: `jointParamSize` (1 or 3 values), `setJointParam` (Houdini values -> settings: degrees
  -> radians, toggles, Euler rotations through a caller-supplied converter, `box3dEulerToQuat`) and `getJointParam`
  (the inverse, for node defaults). Used by the nodes and by the primitive overrides (for everything but floats,
  which multiply); a test round-trips every setting.
* `createJoint(type, bodyA, bodyB, frameA, frameB, settings, userData)`: frames are world space (anchor P +
  orient) and converted to each body's local frame from its current transform (`toLocal`). The sync passes authored anchors
  (world space at rest) through `restToNow` first: each moved with its packed body since rest (restxform rotation and
  rest position, `Box3DBodyMirror::restPos`), as Bullet maps anchors through restxform, so a joint made after its
  pieces moved (a new primitive, a remake after an upstream edit) lands where the anchor sits on the piece now. Live
  frames (`current_*`) and saved local frames are already current and skip it; the world, affectors, whole objects
  and plain points are unchanged. A null BodyHandle is the
  world: one shapeless static body per world, made on first use (`Impl::anchor`, not in `bodyCount`). Settings are
  clamped first (`sanitized`) to what Box3D asserts on: non-negative stiffness / damping / caps, ordered limits,
  cone angle 0..pi/2, revolute target -pi..pi, finite vectors, unit quaternions (the test checks
  `assertCount` does not move in a RelWithDebInfo build). Filter joints always have `collideConnected` off.
* Motion Lock (`b3MotionLockJointDef`, settings `lockLinearX/Y/Z`, `lockAngularXY`; node parameters `lock_x`, `lock_y`, `lock_z`, `lock_angular_xy`) replaces Box3D's removed per-body motion locks, so there is no `lock_linear` / `lock_angular` point attribute. The locks are along frame A's axes; a world-anchored line (empty name on one end) with an identity `orient` locks world axes. The node defaults to the three position locks.
* `updateJoint` pushes every setting of the type through Box3D's setters (all are live) and wakes the bodies.
  Bodies, frames and type cannot change live: the caller recreates the joint.
* `jointFeedback`: constraint force / torque (vectors), linear / angular separation, and per-type readings named
  `current_*` (plus `motor_torque`, `motor_force`, `spin_torque`) so no reading shares a name with a setting (a
  test checks). Box3D's `b3Joint_GetAngularSeparation` compares the bodies' rotations, not the joint frames
  (`relQ = inv(qA) * qB` in joint.c), so a held joint between bodies that started rotated reports a non-zero angle.
  The `angle` attribute is therefore the plugin's own (`angleSinceRest`): the bodies' relative rotation since rest
  (`Box3DJointEntry::restRotA` / `B`: a packed body's restxform rotation, else its rotation when the joint was made,
  identity for the world), as Bullet. Box3D's value is only the fallback when a body's pose cannot be read.
  For wheel joints Box3D's angular separation is a `// todo` that asserts, so it is reported as 0.
* Breaking: Box3D raises a joint event when force / torque passes the threshold (solver.c) but does not break the
  joint; `brokenJoints` returns the events' user data and the caller destroys them. `b3DestroyBody` destroys a
  body's joints, so handles can go stale: `isJointValid` before use.

**Houdini** (`Box3DJointSync.cpp`, `Box3DJointBreak.cpp` for the after-step part, `SIM_Box3DConRel.cpp`):
* Nodes: `SIM_Box3DConRel` (`SIM_ConRel` + `SIM_OptionsUser`, like SideFX's `SIM_ConRelSpring`; abstract,
  `DECLARE_CLASSNAME` only) and one subclass per joint type from `BOX3D_DECLARE_CONREL`, each with its own automatic DOP
  (`box3d_<type>conrel`, label "Box3D <Type> Constraint Relationship", default Data Name the type label, `Weld` etc., capitalised like SideFX's). `describe(type)`
  builds the parameters (`Box3DParms`, defaults from `getJointParam` over `nodeJointDefaults`) once per type: a "Joint"
  tab with Attributes Replace Values (not on Filter) and the settings every type has (and Breakable), then a tab named after the type, in `jointParamUiOrder`,
  parameter name = attribute name (no per-type prefixes). `settings()` reads them through `setJointParam` in Box3D
  units over Box3D's defaults, then applies the node toggles.
* Networks: every relationship of a solved object (group and affector side) whose relationship type data is a
  `SIM_ConstraintNetworkRelationship`, once per relationship name (a relationship is on every object it constrains),
  geometry from its `getGeometry()`; plus each object's own `ConstraintGeometry` (Attach Internal Constraints to
  Object). Network key = hash of the relationship name (or the object id). `anyJointNetwork` checks the same, without
  reading geometry; while true, each full object scan fills `Box3DObjectSync::byName` (point `name` -> entry, first
  wins) and an object without `namesKnown` cannot skip. `name` is a prop marker, so a renamed piece rescans.
* Kinds (`networkKinds`): `constraint_name` matches a data name exactly, as RBD Bullet Solver's does. The `SIM_Box3DConRel` subdata, by data name, of the relationship, its
  `SIM_ConstraintNetworkRelationship` and its geometry (internal: the ConstraintGeometry and the object); the first
  name found wins. Kinds are re-read every sync (a few subdata); any change reprocesses the network.
  The HDK's `SIM_ConstraintNetwork` iterators / accessors are not used: the geometry is read directly, which keeps
  the fingerprinting, live frames and local frames below.
* A network is reprocessed only when its fingerprint (identity without the detail's unique id, since the engine copies the geometry into a new detail every timestep; topology, data ids of `P`, `name`, `orient`,
  `constraint_name`, `auto_restlength`, every table attribute, the `broken` group), its kinds, or
  the bodies it depends on changed: a body one of its joints is on was removed (`Box3DSharedState::removedBodies`), bodies were added or removed while some anchor did not resolve (`Box3DJointNetwork::needsBodies`, also set for an anchor that resolves by an object's name), a piece was renamed (`nameVersion`), or an affector came or went. Otherwise its
  joints keep the network's `liveStamp`, like skipped objects' bodies.
* Per two-point primitive: `constraint_name` -> kind (missing or unknown: counted, ignored), anchors resolved
  (`Anchors::resolve`, tables of single-body objects and affectors built on the first resolve of a step, pieces searched in each object's own name map, first match in object order: empty = world; a piece name in any object in object order; `object/piece`; an object with
  exactly one body; an affector object's name -> its static mesh body). Unresolved: no joint, retried when bodies
  change. **`anchor_id`** (point attribute, with `anchor_type` string `point` / `vertex`, or int 0 / 1): on an anchor that
  resolves to a whole object (an RBD Object) the anchor sits at that point (vertex's point) of the object's geometry
  through its geometry transform (`pointAnchorWorld`); on a packed piece, at that point of the piece's own geometry
  through its full transform (`packedAnchorWorld`: the packed point's position plus the geometry point from the pivot through the piece's
  matrix, then `nowToRest`: authored anchors are at rest), as Bullet (`bullet_anchor_id`). A packed fragment's own points are those of its primitives
  in the shared source (points in source order, vertices in primitive order: `fragmentAnchorPoint`). `orient` still applies. The position is kept in the entry (`pinA` /
  `pinB`, with the id it was taken for) at first sight, so the body moving is no edit. An id that is no point is
  `badAnchorId`: no joint. `-1` or a missing attribute: `P`. On the world and affectors the id is ignored.
  `anchor_type` agent is `agentAnchors`: no joint (nothing simulates an agent rig). `anchor_id` / `anchor_type` are
  in the network fingerprint. The `anchor_pid` / `anchor_vid` id matching is not read. Settings = the kind's + matching primitive attributes, the Constraint Network DOP's rule: float attributes
  multiply the node's value (angles too: the attribute is a factor, so no degree conversion), toggles (int), vectors
  and Euler rotations (degrees) replace it. An attribute value of `kUnsetFloat` (a float, or a vector's x) or a
  negative int is "not set" and skipped (`isUnsetFloat`, `applyOverrides`): Box3D Constraint Properties creates its
  attributes that way so the lines it does not edit keep the node's value. So a node value of 0 (most limits and targets) or a negative "never /
  unlimited" cap cannot be changed per primitive, unless the node's Attributes Replace Values
  (`kAttributesReplaceName`, `Box3DJointKind::attributesReplace`, in `sameKinds`) is on: then floats are the value
  too, in Houdini units (`applyFloatAttribute`). Every type but Filter. SideFX data keeps the multiply rule. Signature = type, bodies, both
  frames: equal and the joint valid -> `updateJoint` if settings differ; else recreate. `Box3DJointEntry` (in
  `st.joints`, key network + primitive offset) is the joint's user data, so nodes must not move (unordered_map).
  Entries not stamped are destroyed and erased.
* After the step (`jointsAfterStep`): broken joints are destroyed and flagged (never remade, even if the
  primitive changes), and written back through the relationship's `getGeometry()` (non-const: a `SIM_GeometryCopy`)
  or the object's ConstraintGeometry (made a `SIM_GeometryCopy` once, like the object geometry): the `broken`
  primitive group, then `force` / `torque` (magnitudes; `torque` is `JointFeedback::forceMoment`, the force's moment about body A's centre, zero for the world, as Bullet's), `distance` / `angle` (the anchors' distance, the angle since
  rest above, in radians, the Constraint Network DOP's names and units) and the per-type `current_*` readings (degrees) every step. The fingerprint is re-recorded after the write so it is not seen as an edit. A network that
  re-imports its SOP every step (Overwrite with SOP) overwrites the group; `Box3DJointEntry::broken` still holds until
  a world rebuild. `Box3DJointNetwork::writable` is false when the last write found no geometry; such a network keeps
  its frames in the joint signature (see below).
* `next_constraint_name`: a broken joint whose primitive has one is not marked broken. Its `constraint_name` is set to
  it and the attribute cleared (so it switches once), its local frames (read before it was destroyed) put through the
  bodies' poses give the anchors' `current_P` / `current_orient` (`switchFrames`: written by `writeLiveFrames` as if the joint were
  live), and the fingerprint is zeroed so the next sync reprocesses the network and makes the new joint from the saved
  local frames, exactly where the previous one was. It also takes its next settings (`applyNextSettings`): every primitive attribute
  `next_<setting>` that is set (a number above kUnsetFloat, a toggle or integer of 0 or more, a non-empty string) becomes `<setting>`
  (made with every other primitive "not set" if the geometry has none) and is cleared; `next_constraint_type` becomes
  `constraint_type` (written by hand: Constraint Properties does not edit SideFX constraints). The Solver SOP's break wrangle does the same from a list of the settings (`takenext`, `next_setting_names`).
  `constraint_type` itself is read for SideFX data only.
* **Live frames** (written networks): after every step `writeLiveFrames` sets each live joint's
  anchor points' `current_P` / `current_orient` to where its frames are now (`Box3DWorld::jointWorldFrames`: body
  pose times local frame; a world end does not move). `P` / `orient` are never written: the Constraint Network DOP
  documents `P` as the anchor's initial world position, and Houdini's constraint guide moves each anchor with its piece
  itself, so a live `P` would be drawn twice as far (`anchor_positions` checks both). `NetworkReader::frame` reads `current_*` when written (a zero
  `current_orient`, the value of a point added later, means not written), else `P` / `orient`. The network data is
  cached with each timestep (SIM_GeometryCopy), so a world rebuilt mid-simulation makes its joints from the cached
  `current_*` against the cached body poses, which gives exactly the same local frames as continuing
  (`tests/core_joints.cpp` compares a rebuilt world with the original).
  * **Local frames** (`local_P` / `local_orient`, point attributes, the names SideFX's Constraint Network uses for an
    anchor in its transform's local space): a joint made from `P` / `orient` saves the frames Box3D keeps
    (`jointLocalFrames`: relative to the piece, position in its axes, in scene units) after the step. A joint made
    later (a world rebuilt mid-simulation) is made from them (`createJoint(..., framesAreLocal)`) when they still
    agree with the live values: `localToWorld(body, local)` must land on the point's `current_P` / `current_orient` (1e-5
    relative, quaternion dot 1 - 1e-6), which holds when all three were written after the same step and fails for
    a renamed, moved or new anchor (a zero `local_orient` means none). This makes rebuilds exact (float world positions lose about a float step at the coordinates' size per rebuild;
    `asLocalFrame` only renormalises a quaternion). The live `current_*` are still read: to make new joints and to see edits.
  * Edits: the entry remembers the authored anchors it was made from (`authA` / `authB`) and what it wrote
    (`wroteA` / `wroteB`), as stored in float. A reprocessed network whose `P` / `orient` differ from the first, or
    whose `current_*` differ from the second (1e-5 relative; q and -q equal), was edited upstream (a SOP Solver): the
    joint is remade from the authored `P` / `orient` (an edit of the authored anchors made just before a world rebuild
    is not seen: the rebuild trusts `current_*`). The signature therefore leaves the frames out for written networks; a network that could not
    be written (`writable` false) keeps the frames in its signature, so editing them remakes the joint.
  * A distance joint made with a negative rest length gets the anchors' distance at that moment written to the
    `auto_restlength` primitive attribute (created with default -1), so a rebuild keeps the rest length instead of
    the current, stretched distance. Not `restlength`: that attribute multiplies the node's value, and -1 times the
    distance would be negative again. It is only read while the joint's rest length is negative.
  * Our writes bump `current_P` / `current_orient` (in the network fingerprint), so the fingerprint is re-recorded after
    the write and an unchanged network still skips. Writing happens every step except for a network whose joints are all
    quiet (`markQuietJoints`: bodies at rest and already written, or broken joints with their last values written); with
    nothing broken and no Enable Aging it is left alone (no write lock, new data ids or fingerprint).
  * A Constraint Network with Overwrite with SOP re-imports the SOP every step, dropping `current_*`: the joints
    carry on (their signature leaves the frames out), but a world rebuilt from a cached frame other than the first
    makes them from the authored anchors there.
