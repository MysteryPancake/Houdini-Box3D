# Box3D samples: verdicts, Issues to World

Continued from [sample-port-table-a.md](sample-port-table-a.md), with the same verdicts (**now**, **work**, **no**).
The plan: [sample-port-plan.md](sample-port-plan.md).

## Issues (`sample_issues.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Crash | now | Welded boxes on a grid mesh |
| Multiple Prismatic | now | Prismatic chain with limits, `constraint_stiffness` |
| Hull Crash | no | Hull building from point sets, no simulation |
| Thin Slab Hull | no | Hull building, no simulation |
| Convex Jitter | now | Custom hulls at rest, rolling resistance |
| s&box mover | work | `building.obj` (`obj_mesh`), height field, platform mesh; `fixedRotation` bodies (see Fixed Rotation) |
| Capsule Mesh | work | `fixedRotation` capsules on a mesh |
| s&box Ghost Collisions | work | A body walked back and forth by setting its velocity each step: drag towards a keyed `targetv`; `fixedRotation`; speculative contact off has no setting |
| GMod Wheel Stack | now | Stacked hull wheels; contact tuning on the solver; profile readout dropped |
| Restitution Overshoot | now | Box bouncing; readout dropped |
| Slide Twist Off Center Shape | now | Offset box spinning while it slides |
| Huge Box | now | One very large box |
| Heightfield | now | Cylinder on a height field |
| Thin Wall Push Out | work | Mover held still every step: `animated_piece`; Gravity |
| Rolling Deck | work | Kinematic deck rolled by `SetTargetTransform`: `animated_piece` |
| Turning Platform | work | Platform that starts turning after a settle time: `animated_piece` keyed |
| Static Load | work | Hit threshold and contact impulse readouts: Add Impact Data shows the impulses |
| Translation Locked Platform | work | Translation locked through mass data: Motion Lock (position locks) |
| Bullet Thin Plate | now | `fastobject` spheres against a thin plate mesh |

## Vehicle (`sample_issues.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Raycast Vehicle | no | Keyboard-driven, suspension from world ray casts each step |

## Joints (`sample_joint.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Distance Joint | now | Distance chain: spring, min / max length, spring force range |
| Filter | now | Filter joint |
| Motor Joint | work | Motor joint chasing a target moved each step: `animated_piece` target; max spring force / torque |
| Top Down Friction | work | Motor joints to the ground as friction (max velocity force / torque), no gravity; GUI explosion: starting `v` |
| Prismatic | now | Spring, limits, motor, target translation |
| Spherical | now | Cone and twist limits, spring, motor, target rotation |
| Spherical Motor Frame | now | Spherical motor velocity |
| Parallel Spring | now | Parallel joint spring |
| Revolute | now | Limits, spring, motor, target angle |
| Weld | now | Linear and angular springs, constraint stiffness |
| Wheel | now | Suspension, spin motor, steering and limits |
| Ball and Chain | now | Capsule links, spherical joints with friction motors |
| Door | work | Revolute door hit by a GUI impulse: starting `w`; per-joint constraint stiffness |
| Bridge | now | Revolute and spherical bridge with springs and damping |
| Fold Out | now | Revolute friction motors |
| Locked Joints | work | Joints on motion-locked bodies; lock and rotation choices are GUI (one per scene), the impulse a starting `v` |
| Motion Lock Joint | now | Motion lock joint; the key impulse as a starting `v` / `w` |
| Driving | work | Keyboard car on a height field: keyed Spin Speed and Target Steering Angle on the wheel node (live settings) |
| Gear Lift | work | Gears of offset boxes on revolute motors, a prismatic lift, rocks, custom mesh: `multi_shape_body` |

## Manifold (`sample_manifold.cpp`)

All **no**: contact manifold tools between two shapes, no world (Sphere vs Sphere, Capsule vs Sphere, Hull vs Sphere,
Triangle vs Sphere, Capsule vs Capsule, Capsule vs Hull, Triangle vs Capsule, Hull vs Hull, Hull Culling, Edge Axis
Wedge, Triangle vs Hull, Hull Ghost Manifold, Sphere Ghost Manifold, Ghost Oracle).

## Mesh (`sample_mesh.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Grid | now | Shapes dropped on a grid mesh |
| Big Box | now | Shapes inside a large box mesh |
| Box | now | Shapes inside a box mesh |
| Reflection | work | Mirrored mesh: a Transform SOP scale of -1 and Reverse before `concave` |
| Height Field | work | Spheres and boxes on a height field; the ray and shape casts dropped |
| Viewer | no | Mesh tree viewer, no simulation |
| Creation Benchmark | no | Mesh build timing |
| Voxel | work | `voxel_mesh_0*.obj` (`obj_mesh`) |
| Hollow Box | now | Capsules and cylinders in a hollow box mesh |
| Wedged Sphere | now | Sphere wedged in a mesh; readout dropped |

## Ragdoll (`sample_ragdoll.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Box | work | `human` on a ground box |
| Mesh | work | `human` on a grid mesh |
| Pile | work | Many `human`s on a grid mesh |
| Incline | work | `human` on a slope; its friction and spring drop at 2 s: keyed relationship node parameters |
| Pose | no | Compiled out (`#if 0`) |

## Replay (`sample_replay.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Viewer | no | Plays Box3D recordings |

## Restitution (`sample_restitution.cpp`)

All **now**: `bounce` on boxes and spheres, with Bounce Mixing maximum (`box3d_world`). The energy and height readouts
are dropped.

| Sample | What it uses |
|---|---|
| Varying | A row of bounces from 0 to 1 |
| Single Box Restitution | One box, `ccdthreshold` |
| Single Sphere Restitution | One sphere, `ccdthreshold` |
| Sphere Stack Restitution | A sphere stack hit from above, Gravity |
| Restitution Propagation | Balls bouncing on crates: Bounce Propagation |
| Box Restitution | A box landing at speed, Gravity |
| Rotated Box Restitution | Tilted boxes kept apart by `collision_group` |
| Two Box Restitution | Two bouncing boxes |
| Sphere Restitution | A row of spheres |
| Bounce Impulse | Sphere and box bounce, Gravity |

## Robustness (`sample_robustness.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| HighMassRatio1 | now | Light boxes under heavy ones (`density`) |
| Tiny Pyramid | now | A pyramid of tiny boxes |
| Overlap Recovery | now | Overlapping boxes pushed apart: the contact parameters |
| Overflow Color Pile | now | A dense pile; the overflow counter dropped |

## Shapes (`sample_shapes.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Inclined Plane | now | Boxes of rising `friction` on a slope |
| Rolling Resistance | now | `rollingresistance` |
| High Resistance | now | High `rollingresistance` capsule |
| Isotropic Friction | now | Boxes sliding in many directions |
| Slide Twist | now | Spinning box sliding |
| Static Invoke | work | A static body created at step 20: a static piece appearing at that frame |
| Conveyor Belt | now | `tangentvelocity` |
| Conveyor Mesh | work | `conveyor.obj` with per-triangle belt materials: one static piece per material |
| Wind | work | Spherical-jointed chains in wind: drag only (`airresist` / `targetv`), no lift |
| Wind Drop | no | A falling plate whose flutter comes from wind lift, which drag cannot give |
| Wind Flap | work | Revolute flaps with a sinusoidal target angle (keyed) in wind: drag only |

## Stacking (`sample_stacking.cpp`)

All **now** but Capsule Stack (**work**): boxes, spheres, capsules, cylinders and hulls on a ground box; a starting
push is the first piece's `v`, samples held in a plane use Motion Lock.

| Sample | What it uses |
|---|---|
| Card House | Thin boxes, friction, rolling resistance |
| Sphere Stack | Spheres |
| Capsule Stack | Capsules held in a plane with `fixedRotation` (see Fixed Rotation): Motion Lock plus the workaround |
| Single Box | One box |
| Cylinder | Rolling cylinder (`cylinder`) |
| Cylinder Stack | Stacked cylinders |
| Box Stack | Box columns |
| Jenga Stack | Boxes and capsules |
| Dominoes | Dominoes, the first pushed |
| Wedge | Custom hull wedge (`hull_points`) |
| Arch | Custom hull stones |
| Double Domino | Two dominoes, one pushed |
| Pyramid2D | Pyramid held in a plane (Motion Lock) |
| Edge Crossing | Boxes resting across edges |

## Tree (`sample_tree.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Benchmark | no | Dynamic tree benchmark, no simulation |

## World (`sample_world.cpp`)

All **no**: they show double precision far from the origin (Far Stack, Far Motion Lock, Far Pyramid, Far Ragdolls,
Far Mesh Drop). The plugin builds Box3D in single precision and Houdini's `P` is 32-bit, so the point of each is lost;
near the origin they repeat Stacking, Joints, Ragdoll and Continuous samples.
