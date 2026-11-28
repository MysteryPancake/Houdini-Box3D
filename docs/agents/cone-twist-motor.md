# Cone Twist motor

A SideFX Cone Twist's motor (the crowd ragdolls' motors and stiffness) driven through the spherical joint's own motor,
set every step. The rest of the Cone Twist mapping: [joints-sidefx.md](joints-sidefx.md). Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## What RBD Bullet Solver does (measured, Houdini 22.0)

A box on a Cone Twist to a fixed box, its motor set by line attributes (`bullet_cone_twist_motor`), and the Crowd
Solver's lines in PinnedRagdolls:
* Options (data and line attributes, floats multiply the node's as for every SideFX option): `motor_enabled`,
  `motor_targetcurrentpose`, `motor_target` / `motor_targetprev` (quaternions in the Cone Twist's space: x twist, y out,
  z up), `motor_hastargetprev`, `motor_maximpulse`, `motor_erp` (Correction Time, seconds: the node's 0.2 default times
  the line's, which is why the crowd's Cone Twist node has it at 1), `motor_normalizemaximpulse` (Ignore Mass, on).
* The Crowd Solver writes them on the agent object's ConstraintGeometry every step: the Crowd State's motor group gets
  the clip's pose as `motor_target`, last step's as `motor_targetprev`, Motor Strength as `motor_maximpulse` and
  Correction Time as `motor_erp`; every other line gets `motor_targetcurrentpose` 1, `motor_erp` 0 and
  `ragdoll_stiffness` as `motor_maximpulse`.
* Each Bullet substep the target moves on from the previous target to this step's (each clamped to the joint's limits,
  as the Cone Twist help says: the twist to Max Twist, the swing along its axis to the Max Out / Max Up ellipse, at
  most Box3D's 90 degree cone, a span under `kConeTwistClampMinSpan` not clamped), and the joint turns toward the step's target as fast as the target moves (whichever way it moves), plus substep /
  Correction Time of the gap to where the target is then, never past the step's target; one there stays. A fixed target
  is approached exponentially (26.4, 6.33 and 2.52 per second at 0.2, 0.8 and 2 s, matching that rule at 10
  substeps), the same about all three axes and for any mass.
* Max Impulse caps the change of relative angular velocity per Bullet substep: with Ignore Mass in rad/s, times the
  inertia about the joint (a box hinged at an edge holds against gravity from about 0.05 at 10 substeps and 0.5 at
  1, as that predicts; the inertia about its centre would need 0.12). The same inertia for a twist about an axis
  through the centre of mass, and with two dynamic bodies their reduced inertia: a motor limited by Max Impulse turns
  a box as fast twisting as Bullet's does to about 10% (ours faster), swinging as much slower as the box's inertia
  about its centre is under that about the joint, and the same with the other box free. Without Ignore Mass it is an
  angular impulse.
* A motor holds the joint's other axes rigidly too (a twisting motor stops a pendulum swinging).
* On a mirrored rig transform (PinnedRagdolls' left arm, every joint mirrored at both ends) the joint's frames are
  ours turned half a turn about their y axis: with every joint motored and a fast Correction Time, the left arm's
  bodies are within 0.015 (upper arm) and 0.07 (forearm) of Bullet's that way, against 0.04 and 0.15 unturned, and
  half turns about x or z are further off.
* `motor_cfm` 0 to 0.01 and Global CFM 0.025 change nothing measurable. A Correction Time of 0 with a target gives NaN
  in Bullet.

## What the plugin does

* `sideFXParams()` has the motor options; all but `motor_enabled` are `perStep`: left out of `networkFingerprint`, as
  the crowd's change every step, so the network is not read again for them. `specFor` marks a Cone Twist line with
  `motor_enabled` (`JointSpec::motor`), and each full read lists those joints in `Box3DJointNetwork::motorEntries`.
* `readJointMotors` (every sync, from `processNetwork`, read or not) applies each listed line's attributes to its
  kind's options and keeps `b3dop::coneTwistMotor` in `Box3DJointExtra::motor`: the targets clamped
  (`kConeTwistClampMinSpan`) and turned into the spherical joint's frame B relative to frame A (`twistFrame`'s axes,
  inverted: Bullet's turns the other way), then half a turn about y on each mirrored end (`motorMirroredA` / `B`, from
  `AnchorBody::mirrored` when the network is read).
* `driveJointMotors` (before the steps, after `reaimJoints`): from the joint's relative rotation now,
  `coneTwistMotorVelocity` runs Bullet's substeps (`kSideFXMotorSubsteps`, its default) over the frame and gives the
  velocity that lands where they do; Target Current Pose (or no Correction Time) is velocity 0.
  `coneTwistMotorTorque` is Max Impulse per Bullet substep, times the bodies' effective inertia about the joint with
  Ignore Mass (`sideFXBodyAbout`). `Box3DWorld::setSphericalMotor` sets the three; a motor turning faster than
  `kMotorWakeSpeed` wakes the joint's bodies. A line whose motor goes off (`motorDriven`) has it turned off.
* Not read: `motor_cfm`, the soft-constraint options (Enable Soft Constraint's target stiffness and damping), the
  position target (`motor_targetp`).

## Checking it

`tests/core_sidefx.cpp` (axes, rate, clamp, moving targets, torque) and `bullet_cone_twist_motor` (each case within 1
degree of Bullet's). PinnedRagdolls: [todo.md](todo.md), "Agent ragdolls".
