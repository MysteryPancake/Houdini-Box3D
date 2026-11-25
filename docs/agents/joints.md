# Joints

Constraint networks and our joint nodes. SideFX constraint data and Glue: [joints-sidefx.md](joints-sidefx.md); Glue
as One Body: [glue-compound.md](glue-compound.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Joints

Box3D's joints, set up the way Houdini's old constraint system is: a **Constraint Network** DOP holds the network
geometry, and constraint relationship nodes wired into its second input hold the settings, one node per kind of
constraint, named by the primitives' `constraint_name`. Here those nodes are ours, one per Box3D joint type, with
Box3D's own settings and defaults. SideFX's own constraint data (Glue, Hard, Spring, Bullet Soft, Cone Twist) is
mapped onto Box3D joints, approximately ([joints-sidefx.md](joints-sidefx.md)). `constraint_type` (position / rotation
/ all) is read for SideFX data only; for our nodes the joint type decides what is constrained, on purpose (a generic
joint in the fork would be needed to honour it), and a `position` or `rotation` on one warns (`typeIgnored`, test
`constraint_type_ignored_warning`). Character-mover joints (mover, pogo) are left out.

Naming: attributes holding the state at the initial pose are called `rest*` (`restlength`), or are Houdini's own (a
network point's `P` / `orient`: the anchor's initial world frame, never written); attributes the solver keeps current
are `current_*` (`current_P` / `current_orient` on network points, and the per-type readings). The solver writes no
attribute it does not read itself, except feedback (`force`, `distance`, `torque`, `angle`, `current_*`), as the
Bullet solver does.

**Core** (`Box3DJoints.h`, `Box3DWorld`, tested in `tests/core_joints.cpp`):
* `JointType` (weld, spherical, revolute, prismatic, distance, motor, parallel, wheel, filter, motionlock) and
  `JointSettings`: every setting of every type in Box3D units (radians, rad/s, Hz). Box3D's FLT_MAX caps are negative
  values here (unlimited): break thresholds, parallel `maxTorque`, distance `maxLength`, `maxTensionForce`,
  `maxCompressionForce` (Box3D's signed `lowerSpringForce` / `upperSpringForce`, renamed so the sign cannot be got
  wrong). Distance `restLength < 0` (`restlength`) = the anchors' distance when the joint is made. Weld:
  `enableLinearSpring` / `enableAngularSpring` (off = rigid; Box3D's own rule is "hertz 0 = rigid", which reads
  backwards with the name stiffness). Every Box3D "hertz" is called stiffness.
* `jointParams()`: one row per setting (attribute / parameter name, label, kind, joint types using it, member pointer,
  help). The constraint relationship nodes, the primitive attribute reader and `sameJointSettings` are all driven by
  it. `angleTypes` marks a Float that is an angle for some types only (`motor_speed`: revolute deg/s, prismatic and
  distance units/s). `Rotation` is Euler XYZ degrees in Houdini and a quaternion in the core. `enabledBy` names the
  toggle that switches a setting on (spring, motor, limit values); `jointParamDisableWhen(p, type)` turns it into the
  node's disable-when string, following chains (steering limits need Steering) and skipping toggles the type lacks
  (the parallel joint's spring and the motor joint's springs have none). Only the UI uses it: a disabled parameter is
  still read, and its primitive attribute still overrides it. `jointParamUiOrder(type)` is the order the node shows:
  table order, but each gated setting moves to just below its toggle, so a toggle only greys out the block under it.
  Reordering is safe (parameters are saved by name); keep always-on settings out of a toggle's block (why `restlength`
  is near the top). `jointParamIndices(type)` is the table filtered to the rows a type uses (built once, table order);
  `sameJointSettings` runs for every joint every step and walks only those rows. Per-type loops over the table should
  use it.
* `Box3DWorld::defaultJointSettings(type)` copies `b3Default*JointDef`. The nodes start from
  `nodeJointDefaults(type, ...)` over it, since Box3D's spring stiffness, motor effort and limits are 0 (a toggle
  would do nothing, or lock a limit at 0..0). Springs start at Bullet Soft's 10 Hz, damping ratio 1. Only the node UI
  uses these.
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
* `createJoint(type, bodyA, bodyB, frameA, frameB, settings, userData)`: frames are world space (anchor P + orient)
  and converted to each body's local frame from its current transform (`toLocal`). The sync passes authored anchors
  (world space at rest) through `restToNow` first: each moved with its packed body since rest (restxform rotation and
  rest position, `Box3DBodyMirror::restPos`), as Bullet maps anchors through restxform, so a joint made after its
  pieces moved (a new primitive, a remake after an upstream edit) lands where the anchor sits on the piece now. Live
  frames (`current_*`) and saved local frames are already current and skip it; the world, affectors, whole objects and
  plain points are unchanged. A null BodyHandle is the world: one shapeless static body per world, made on first use
  (`Impl::anchor`, not in `bodyCount`). Settings are clamped first (`sanitized`) to what Box3D asserts on:
  non-negative stiffness / damping / caps, ordered limits, cone angle 0..pi/2, revolute target -pi..pi, finite
  vectors, unit quaternions (the test checks `assertCount` does not move in a RelWithDebInfo build). Filter joints
  always have `collideConnected` off.
* Motion Lock (`b3MotionLockJointDef`, settings `lockLinearX/Y/Z`, `lockAngularXY`; node parameters `lock_x`,
  `lock_y`, `lock_z`, `lock_angular_xy`) replaces Box3D's removed per-body motion locks, so there is no `lock_linear`
  / `lock_angular` point attribute. The locks are along frame A's axes; a world-anchored line (empty name on one end)
  with an identity `orient` locks world axes. The node defaults to the three position locks.
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
