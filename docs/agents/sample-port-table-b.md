# Box3D samples: verdicts, Issues to World

Continued from [sample-port-table-a.md](sample-port-table-a.md), with the same verdicts (**built**, **now**, **work**,
**no**).
The plan: [sample-port-plan.md](sample-port-plan.md).

## Issues (`sample_issues.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Crash | built | Welded boxes on a grid mesh; the weld there from the start, not added by its button |
| Multiple Prismatic | built | Prismatic chain with limits, `constraint_stiffness` |
| Hull Crash | no | Hull building from point sets, no simulation |
| Thin Slab Hull | no | Hull building, no simulation |
| Convex Jitter | built | Custom hulls at rest, rolling resistance |
| s&box mover | work | `building.obj` (`obj_mesh`), height field, platform mesh; `fixedRotation` bodies (see Fixed Rotation) |
| Capsule Mesh | work | `fixedRotation` capsules on a mesh |
| s&box Ghost Collisions | work | A body walked back and forth by setting its velocity each step: drag towards a keyed `targetv`; `fixedRotation`; speculative contact off has no setting |
| GMod Wheel Stack | built | Stacked hull wheels (`hull`, its 64 points from Box3D at full precision); contact tuning on the solver |
| Restitution Overshoot | built | Box bouncing; readout dropped |
| Slide Twist Off Center Shape | built | Offset box spinning while it slides |
| Huge Box | built | One very large box |
| Heightfield | built | Cylinder on a flat height field (`height_field`, a Static Object) |
| Thin Wall Push Out | built | Mover whose velocity is zeroed every step (`step_script`); Motion Lock; no gravity; Start X a slider |
| Rolling Deck | built | Kinematic deck rolled by `SetTargetTransform`: an animated stream; Deck Recycling a toggle (`Bodies.toggle`) |
| Turning Platform | built | Kinematic platform at a constant spin: an animated stream; Platform Recycling a toggle |
| Static Load | work | Hit threshold and contact impulse readouts: Add Impact Data shows the impulses |
| Translation Locked Platform | work | Translation locked through mass data: Motion Lock (position locks) |
| Bullet Thin Plate | built | `fastobject` spheres against a thin plate hull and mesh, a pair a second (`emit`); earlier pairs kept |

## Vehicle (`sample_issues.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Raycast Vehicle | no | Keyboard-driven, suspension from world ray casts each step |

## Joints (`sample_joint.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Distance Joint | built | A ball on a rigid distance joint (Count 1); spring, force range and limit set on the node, off |
| Filter | built | Filter joint |
| Motor Joint | built | Motor joint chasing an animated target; the Speed slider (0 in the sample) is 1 |
| Top Down Friction | built | Motor joints to the world as friction, no gravity; Explode as starting `v` / `w` (`Bodies.explode`) |
| Prismatic | built | Spring, limits, motor, target translation |
| Spherical | built | Cone and twist limits, spring, motor, target rotation |
| Spherical Motor Frame | built | Spherical motor velocity; the head two hulls (`compound`), the kinematic neck static |
| Parallel Spring | built | Parallel joint spring |
| Revolute | built | Limits, spring, motor, target angle |
| Weld | built | Linear and angular springs, constraint stiffness |
| Wheel | built | Suspension, spin motor, steering and limits |
| Ball and Chain | built | Capsule links, spherical joints with friction motors |
| Door | built | Revolute door on the ground body, the Impulse button as starting `v` / `w` |
| Bridge | built | Spherical-jointed planks with springs and damping |
| Fold Out | built | Revolute friction motors |
| Locked Joints | built | Six joint types on boxes also held by motion locks: the lock checkboxes are the lock node's parameters, the Rotation menu at free |
| Motion Lock Joint | built | Motion lock joint; the key impulse as a starting `v` / `w` (Try) |
| Driving | built | Car on a wave height field (`height_field`): the keyboard is keyed Spin Speed and Target Steering Angle (`finish(keys=...)`); the Wheels menu (`choice`) |
| Gear Lift | work | Gears of offset boxes on revolute motors, a prismatic lift, rocks, custom mesh: `Bodies.parts` |

## Manifold (`sample_manifold.cpp`)

All **no**: contact manifold tools between two shapes, no world (Sphere vs Sphere, Capsule vs Sphere, Hull vs Sphere,
Triangle vs Sphere, Capsule vs Capsule, Capsule vs Hull, Triangle vs Capsule, Hull vs Hull, Hull Culling, Edge Axis
Wedge, Triangle vs Hull, Hull Ghost Manifold, Sphere Ghost Manifold, Ghost Oracle).

## Mesh (`sample_mesh.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Grid | built | A body dropped on a grid mesh: the Shape menu (`choice`); the mesh scale slider dropped |
| Big Box | built | A body on a large box mesh: the Shape menu (`choice`); the mesh scale slider dropped |
| Box | built | A body on a box mesh turned 45 degrees: the Shape menu (`choice`) |
| Reflection | work | Mirrored mesh: a Transform SOP scale of -1 and Reverse before `concave` |
| Height Field | no | Ray and shape casts over a height field; its bodies are compiled out (`#if 0`) |
| Viewer | no | Mesh tree viewer, no simulation |
| Creation Benchmark | no | Mesh build timing |
| Voxel | built | A hull on `collision_mesh_01.obj` (`mesh_file`, welded); moved from 5 km out to the origin |
| Hollow Box | built | Capsules and cylinders in a hollow box mesh |
| Wedged Sphere | built | Sphere wedged in a mesh; readout dropped |

## Ragdoll (`sample_ragdoll.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Box | built | `Humans` (`human.py`) on a ground box; the calf is two capsule parts, the foot at half density |
| Mesh | built | A ragdoll on a grid mesh, its bones on parallel joints to the world (the kinematic anchors) |
| Pile | built | 20 ragdolls on a grid mesh, their spots from `XorShift(42)` |
| Incline | built | A ragdoll on a slope (`grid_mesh_turned`); its friction and spring drop at 2 s: keyed relationship node parameters |
| Pose | no | Compiled out (`#if 0`) |

## Replay (`sample_replay.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Viewer | no | Plays Box3D recordings |

## Restitution (`sample_restitution.cpp`)

All **built**: `bounce` on boxes and spheres, with Bounce Mixing maximum (`box3d_world`). The energy and height readouts
are dropped.

| Sample | What it uses |
|---|---|
| Varying | A row of bounces from 0 to 1 |
| Single Box Restitution | One box, `ccdthreshold` |
| Single Sphere Restitution | One sphere, `ccdthreshold` |
| Sphere Stack Restitution | A sphere stack hit from above, Gravity |
| Restitution Propagation | Balls bouncing on crates: Bounce Propagation |
| Box Restitution | A box landing at speed, Gravity |
| Rotated Box Restitution | Tilted boxes kept apart by `collision_groupindex` |
| Two Box Restitution | Two bouncing boxes |
| Sphere Restitution | A row of spheres |
| Bounce Impulse | Sphere and box bounce, Gravity |

## Robustness (`sample_robustness.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| HighMassRatio1 | built | Light boxes under heavy ones (`density`) |
| Tiny Pyramid | built | A pyramid of tiny boxes |
| Overlap Recovery | built | Overlapping boxes pushed apart: the contact parameters |
| Overflow Color Pile | built | A dense pile; the overflow counter dropped |

## Shapes (`sample_shapes.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Inclined Plane | built | Boxes of rising `friction` on a slope |
| Rolling Resistance | built | `rollingresistance` |
| High Resistance | built | High `rollingresistance` capsule |
| Isotropic Friction | built | Boxes sliding in many directions |
| Slide Twist | built | Spinning box sliding |
| Static Invoke | built | A static body made at step 20: emitted on that frame (`emit`); the plugin's acts as Invoke, not the default Passive |
| Conveyor Belt | built | `tangentvelocity` |
| Conveyor Mesh | work | `conveyor.obj` with per-triangle belt materials: one static piece per material |
| Wind | work | Spherical-jointed chains in wind: drag only (`airresist` / `targetv`), no lift |
| Wind Drop | no | A falling plate whose flutter comes from wind lift, which drag cannot give |
| Wind Flap | work | Revolute flaps with a sinusoidal target angle (keyed) in wind: drag only |

## Stacking (`sample_stacking.cpp`)

All **built** (Capsule Stack with its workaround): boxes, spheres, capsules, cylinders and hulls on a ground box; a
starting push is the first piece's `v`, samples held in a plane use Motion Lock.

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

They show double precision far from the origin: the pieces, their ground and anchors need a 64-bit `P` (Attribute
Cast), or the point of each is lost and near the origin they repeat Stacking, Joints, Ragdoll and Continuous samples.

| Sample | Verdict | What it uses / why |
|---|---|---|
| Far Stack | work | A box stack at 1e7 on a ground box: 64-bit `P` |
| Far Motion Lock | work | Motion-locked joints anchored to the world at the offset: 64-bit `P` |
| Far Pyramid | work | A box pyramid at the offset: 64-bit `P` |
| Far Ragdolls | work | Ragdolls piled at the offset: 64-bit `P` |
| Far Mesh Drop | work | Bodies dropped on a static mesh at the offset: 64-bit `P` |
