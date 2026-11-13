# Joints

Constraint networks, our joint nodes and SideFX constraint data mapping. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Joints

Box3D's joints, set up the way Houdini's old constraint system is: a **Constraint Network** DOP holds the network
geometry, and constraint relationship nodes wired into its second input hold the settings, one node per kind of
constraint, named by the primitives' `constraint_name`. Here those nodes are ours, one per Box3D joint type, with Box3D's
own settings and defaults. SideFX's own constraint data (Glue, Hard, Spring, Bullet Soft, Cone Twist) is mapped onto
Box3D joints, approximately (see "SideFX constraint data" below). `constraint_type` (position / rotation / all) is read
for SideFX data only; for our nodes the joint type decides what is constrained. Character-mover joints (mover, pogo)
are left out.

Naming: attributes holding the state at the initial pose are called `rest*` (`restlength`), or are Houdini's own
(a network point's `P` / `orient`: the anchor's initial world frame, never written); attributes the solver keeps
current are `current_*` (`current_P` / `current_orient` on network points, and the per-type readings). The solver writes no attribute it does
not read itself, except feedback (`force`, `distance`, `torque`, `angle`, `current_*`), as the Bullet solver does.

**Core** (`Box3DJoints.h`, `Box3DWorld`, tested in `tests/core_joints.cpp`):
* `JointType` (weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter) and `JointSettings`:
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
  toggle, so a toggle only greys out the block under it. Parameters are saved by name, so reordering is safe for
  existing scenes. Keep always-on settings out of the middle of a toggle's block (why `restlength` is near the top).
  `jointParamIndices(type)` is the table filtered to the rows a type uses (built once, table order); `sameJointSettings`
  runs for every joint every step and walks only those rows. Per-type loops over the table should use it.
* `Box3DWorld::defaultJointSettings(type)` copies `b3Default*JointDef`. The nodes start from `nodeJointDefaults(type, ...)`
  over it instead: Box3D's spring stiffness, motor effort and limits are all 0, so turning a toggle on did nothing (or,
  for limits, locked the joint at 0..0). Only the node UI defaults; Box3D's stay the base everywhere else.
* `jointNodeToggles()`: node-only toggles (Breakable, Set Rest Length, Limit Spring Force, Limit Max Length, Limit
  Torque) in place of typing -1 for "never / unlimited". Gated settings name them in `enabledBy`; `settings()` writes -1
  into them while the toggle is off (`jointNodeToggleOf`), so the settings, primitive attributes (float attributes
  multiply the node's -1, as before) and the SideFX mapping keep their meaning. Not in `jointParams()`: no attribute,
  no fingerprint, no `sameJointSettings` entry. `findJointParam` searches both lists.
* `jointParamRange(p, type)`: each float's slider; a hard 0 minimum where `sanitized` (Box3DWorld.cpp) would clamp
  anyway. `motor_speed` has an angle range on revolute and a linear one elsewhere.
* Houdini units in one place: `jointParamSize` (1 or 3 values), `setJointParam` (Houdini values -> settings: degrees
  -> radians, toggles, Euler rotations through a caller-supplied converter, `box3dEulerToQuat`) and `getJointParam`
  (the inverse, for node defaults). Used by the nodes and by the primitive overrides (for everything but floats,
  which multiply); a test round-trips every setting.
* `createJoint(type, bodyA, bodyB, frameA, frameB, settings, userData)`: frames are world space (anchor P +
  orient) and converted to each body's local frame from its current transform (`toLocal`). A null BodyHandle is the
  world: one shapeless static body per world, made on first use (`Impl::anchor`, not in `bodyCount`). Settings are
  clamped first (`sanitized`) to what Box3D asserts on: non-negative stiffness / damping / caps, ordered limits,
  cone angle 0..pi/2, revolute target -pi..pi, finite vectors, unit quaternions (the test checks
  `assertCount` does not move in a RelWithDebInfo build). Filter joints always have `collideConnected` off.
* `updateJoint` pushes every setting of the type through Box3D's setters (all are live) and wakes the bodies.
  Bodies, frames and type cannot change live: the caller recreates the joint.
* `jointFeedback`: constraint force / torque (vectors), linear / angular separation, and per-type readings named
  `current_*` (plus `motor_torque`, `motor_force`, `spin_torque`) so no reading shares a name with a setting (a
  test checks). Box3D's `b3Joint_GetAngularSeparation` compares the bodies' rotations, not the joint frames
  (`relQ = inv(qA) * qB` in joint.c), so it reports a non-zero angle for a perfectly held joint between bodies that
  started rotated relative to each other; we report it as Box3D gives it (the test documents this).
  For wheel joints Box3D's angular separation is a `// todo` that asserts, so it is reported as 0.
* Breaking: Box3D raises a joint event when force / torque passes the threshold (solver.c) but does not break the
  joint; `brokenJoints` returns the events' user data and the caller destroys them. `b3DestroyBody` destroys a
  body's joints, so handles can go stale: `isJointValid` before use.

**Houdini** (`Box3DJointSync.cpp`, `SIM_Box3DConRel.cpp`):
* Nodes: `SIM_Box3DConRel` (`SIM_ConRel` + `SIM_OptionsUser`, like SideFX's `SIM_ConRelSpring`; abstract,
  `DECLARE_CLASSNAME` only) and one subclass per joint type from `BOX3D_DECLARE_CONREL`, each with its own automatic DOP
  (`box3d_<type>conrel`, label "Box3D <Type> Constraint Relationship", default Data Name `<type>`). `describe(type)`
  builds the parameters (`Box3DParms`, defaults from `getJointParam` over `nodeJointDefaults`) once per type: a "Joint"
  tab with the settings every type has (and Breakable), then a tab named after the type, in `jointParamUiOrder`,
  parameter name = attribute name (no per-type prefixes). `settings()` reads them through `setJointParam` in Box3D
  units over Box3D's defaults, then applies the node toggles.
* Networks: every relationship of a solved object (group and affector side) whose relationship type data is a
  `SIM_ConstraintNetworkRelationship`, once per relationship name (a relationship is on every object it constrains),
  geometry from its `getGeometry()`; plus each object's own `ConstraintGeometry` (Attach Internal Constraints to
  Object). Network key = hash of the relationship name (or the object id). `anyJointNetwork` checks the same, without
  reading geometry; while true, each full object scan fills `Box3DObjectSync::byName` (point `name` -> entry, first
  wins) and an object without `namesKnown` cannot skip. `name` is a prop marker, so a renamed piece rescans.
* Kinds (`networkKinds`): the `SIM_Box3DConRel` subdata, by data name, of the relationship, its
  `SIM_ConstraintNetworkRelationship` and its geometry (internal: the ConstraintGeometry and the object); the first
  name found wins. Kinds are re-read every sync (a few subdata); any change reprocesses the network.
  The HDK's `SIM_ConstraintNetwork` iterators / accessors are not used: the geometry is read directly, which keeps
  the fingerprinting, live frames and local frames below.
* A network is reprocessed only when its fingerprint (identity, topology, data ids of `P`, `name`, `orient`,
  `constraint_name`, `auto_restlength`, every table attribute, the `broken` group), its kinds, or
  `Box3DSharedState::bodyVersion` (bumped when bodies or affectors are added / removed) changed. Otherwise its
  joints keep the network's `liveStamp`, like skipped objects' bodies.
* Per two-point primitive: `constraint_name` -> kind (missing or unknown: counted, ignored), anchors resolved
  (`Anchors::resolve`, hash tables built on the first resolve of a step, first match in object order: empty = world; a piece name in any object in object order; `object/piece`; an object with
  exactly one body; an affector object's name -> its static mesh body). Unresolved: no joint, retried when bodies
  change. Settings = the kind's + matching primitive attributes, the Constraint Network DOP's rule: float attributes
  multiply the node's value (angles too: the attribute is a factor, so no degree conversion), toggles (int), vectors
  and Euler rotations (degrees) replace it. So a node value of 0 (most limits and targets) or a negative "never /
  unlimited" cap cannot be changed per primitive except by setting the node to 1. Signature = type, bodies, both
  frames: equal and the joint valid -> `updateJoint` if settings differ; else recreate. `Box3DJointEntry` (in
  `st.joints`, key network + primitive offset) is the joint's user data, so nodes must not move (unordered_map).
  Entries not stamped are destroyed and erased.
* After the step (`jointsAfterStep`): broken joints are destroyed and flagged (never remade, even if the
  primitive changes), and written back through the relationship's `getGeometry()` (non-const: a `SIM_GeometryCopy`)
  or the object's ConstraintGeometry (made a `SIM_GeometryCopy` once, like the object geometry): the `broken`
  primitive group, then `force` / `torque` (magnitudes), `distance` / `angle` (linear / angular separation, angle
  in radians, the Constraint Network DOP's names and units) and the per-type `current_*` readings (degrees, as
  before) every step. The fingerprint is re-recorded after the write so it is not seen as an edit. A network that
  re-imports its SOP every step (Overwrite with SOP) overwrites the group; `Box3DJointEntry::broken` still holds until
  a world rebuild. `Box3DJointNetwork::writable` is false when the last write found no geometry; such a network keeps
  its frames in the joint signature (see below).
* `next_constraint_name`: a broken joint whose primitive has one is not marked broken. Its `constraint_name` is set to
  it and the attribute cleared (so it switches once), its local frames (read before it was destroyed) put through the
  bodies' poses give the anchors' `current_P` / `current_orient` (`switchFrames`: written by `writeLiveFrames` as if the joint were
  live), and the fingerprint is zeroed so the next sync reprocesses the network and makes the new joint from the saved
  local frames, exactly where the old one was. `next_constraint_type` is ignored, like `constraint_type`.
* **Live frames** (written networks): after every step `writeLiveFrames` sets each live joint's
  anchor points' `current_P` / `current_orient` to where its frames are now (`Box3DWorld::jointWorldFrames`: body
  pose times local frame; a world end does not move). `P` / `orient` are never written: the Constraint Network DOP
  documents `P` as the anchor's initial world position, and Houdini's constraint guide moves each anchor with its piece
  itself, so writing the live frame into `P` (as this solver once did) drew every moving anchor twice as far as its
  piece (`anchor_positions` checks both). `NetworkReader::frame` reads `current_*` when written (a zero
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
    a renamed, moved or new anchor (a zero `local_orient` means none). This makes rebuilds exact: going through
    float world positions loses ~1 float step at the coordinates' size per rebuild (4.6e-4 at x = 12000 in the
    test), the local frames are small offsets kept bit for bit (`asLocal` only renormalises a quaternion that is
    not already unit length). The live `current_*` are still read: to make new joints and to see edits.
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
  * Our writes bump `current_P` / `current_orient`, which are in the network fingerprint; the fingerprint is re-recorded after the
    write, so an unchanged network still skips. Writing happens every step.
  * A Constraint Network with Overwrite with SOP re-imports the SOP every step, dropping `current_*`: the joints
    carry on (their signature leaves the frames out), but a world rebuilt from a cached frame other than the first
    makes them from the authored anchors there.

**SideFX constraint data** (`core/Box3DSideFX.*`, tested in `tests/core_sidefx.cpp`; read in `Box3DJointSync.cpp`):
* Found by data type among the same subdata as our nodes (`networkKinds`): `SIM_ConRelGlue`, `SIM_ConRelHard`,
  `SIM_ConRelSpring`, `SIM_ConRelBulletSoft`, `SIM_ConRelConeTwist` (names from a Houdini 22.0 probe; their default
  Data Names are Glue, Hard, ConRelSpring, Soft, ConeTwist). Only Hard and Spring have HDK headers, so all five are read
  the same way, by name (`box3dDataOption` in `Box3DHoudiniUtil.h`: `dynamic_cast` to `SIM_OptionsUser`, polymorphic
  through its virtual destructor, and `importOption`, else `SIM_Query::getFieldFloat("Options", 0, name)`, scalars
  only), per option in `sideFXParams()`. Option names are the node
  parameter names, from the same probe. `Box3DJointKind` holds the SideFX type and options instead of a joint type.
* Per primitive: matching primitive attributes modify the options (floats multiply, ints / vectors replace; our
  `jointParams` overrides are not applied), `constraint_type` is read (missing or unknown: all), then `mapSideFX`
  picks the joint:
  * Glue: rigid weld holding the pieces where they are, never colliding. Bullet's strength is an accumulated impulse;
    here Break Force = strength / DOP timestep (the force that gives that impulse over one step), < 0 never. Half-life
    and propagation are ignored, and the weld also sees steady loads (weight), not only impacts.
  * Hard: all -> rigid weld with the anchors kept the rest length apart (as Bullet does; held where they are when there
    is no rest length); position -> rigid distance joint at the
    rest length, or spherical when it is ~0; rotation -> motor joint, angular spring only at 60 Hz / ratio 2 with
    unlimited torque (Box3D has no rigid rotation-only joint). CFM, ERP, iterations and the angular motors are ignored.
  * Bullet Soft: stiffness is already Hz. all -> motor joint with linear and angular springs (enableangular picks the
    angular stiffness / damping), frames offset by the rest length; position -> sprung distance joint at the rest
    length, or a linear-only motor joint when it is ~0; rotation -> angular-only motor joint. Plasticity is ignored.
  * Spring: as Soft, after converting Hooke's constant and absolute damping per joint: Hz = sqrt(k / m) / 2 pi,
    ratio = c / (2 sqrt(k m)), with m the reduced mass of the dynamic bodies (one dynamic body: its mass) for the
    linear spring, and the mean principal inertia the same way for the angular one. Read when the network is
    reprocessed, so a mass change alone does not update it.
  * Cone Twist: spherical joint. Box3D's cone is round: the smaller of Max Up / Max Out, clamped to Box3D's 90 degree
    maximum; 180 or more turns it off (Houdini's default). Twist limit +-Max Twist (180 or more: none). Bullet twists
    about x, Box3D about z, so each frame is turned so z = the twist axis and x = the up axis (constrained axes on
    point 0, goal axes on point 1). Softness, the soft-constraint options, translation ranges, rest length and the
    motor are ignored.
  * Hard / Spring / Soft / Cone Twist: Collide Connected = !disablecollisions.
* **Frame offsets**: a SideFX joint's frames need not sit on its anchors. `sideFXFrameOffsets` gives each anchor an
  offset in its own axes (`adjA` / `adjB` on `Box3DJointEntry`): a rotation (cone twist) and/or a shift along the anchor
  line so the frames meet when the anchors are `separation` apart (Soft / Spring / Hard all with a rest length), or are
  as far apart as now (`holdSeparation`: Glue, Hard all without one; anchors from Connect Adjacent Pieces sit at the
  centroids, and
  a plain weld there would pull the pieces together). The joint is made from `applyFrameOffset(anchor, adj)`;
  `writeLiveFrames` and the `next_constraint_name` switch write `removeFrameOffset(frame, adj)` back as `current_P` / `current_orient`,
  so the anchors stay anchors, and `local_P` / `local_orient` hold the joint's own frames (compared with the
  offset anchors in `localFrameAgrees`). Offsets are fixed when the joint is made; their inputs (rotations,
  separation, the hold flag, not the anchor positions) are in the joint signature. Identity offsets (our nodes)
  bypass the math, so those joints are bit for bit as before.
* The DOP timestep reaches `syncJoints` through `syncFromGeometry` (for Glue).
