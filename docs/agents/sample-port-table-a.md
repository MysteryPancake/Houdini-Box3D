# Box3D samples: verdicts, Benchmark to Ghost

One row per registered sample: **built** (ported: `make_examples.py --samples` builds it), **now** (portable
as the plugin is), **work** (portable with the workaround named), **no** (not portable, and why). The plan: [sample-port-plan.md](sample-port-plan.md); the helpers named here that are still to write: [todo.md](todo.md). The
rest of the samples: [sample-port-table-b.md](sample-port-table-b.md). "Readout dropped" means the sample also draws
numbers the port does not reproduce.

## Benchmark (`sample_benchmark.cpp`, `shared/benchmarks.c`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Large Pyramid | built | Box pyramid on a ground box, no sleeping |
| Wide Pyramid | built | Stepped pyramid of boxes |
| Many Pyramids | built | A field of small pyramids, no sleeping |
| Rain | work | `human` ragdolls dropped in groups onto grid and torus meshes, the oldest group replaced each cycle: pieces appearing at frames and `dead` (`step_script`); joint tuning as Constraint Stiffness |
| Large World (first) | no | Compiled out (`#if 0`) |
| Joint Grid | built | Spheres joined in a grid by spherical joints (`network_sop` VEX) |
| Falling Boxes | built | Box hulls falling on a ground box |
| Candy Cups | built | One hull from a point set (`Bodies.hull`), many copies |
| Explosion | work | `b3World_Explode` on boxes and cylinders over a grid mesh: starting `v` / `w` (`Bodies.explode`, which needs hulls) |
| Height Field | no | Ray and shape casts over a height field; nothing moves |
| Falling Trees | work | Trees of several transformed hulls: `Bodies.parts`; wave mesh or height field ground |
| Mesh Drop | built | Boxes on a static wave mesh, starting velocities from `XorShift` |
| Village | work | One baked static compound (tiles, wave meshes, capsule and sphere props): separate static pieces; bodies dropped on it; the cast curtain (queries) dropped |
| Sensor | no | Sensor shapes and events, custom filter callback |
| Washer | built | Kinematic drum of 40 hulls spun at a constant rate: an animated `concave` piece |
| Large World | work | A million static floor boxes, a sphere dropped every few steps: a smaller floor, spheres emitted (`emit`) |
| Hull | no | Hull build timing, no simulation |
| Chains | work | Capsule chains (spherical joints, springs, friction motors) on a wave mesh; per-shape wind: drag only (`airresist` / `targetv`) |
| Destruction | work | A box grid blown apart and respawned periodically: starting `v`, `dead` and new pieces by frame (`step_script`) |
| Junkyard | built | Kinematic cylinder pushed round a circle (`SetTargetTransform`) through 10584 rocks (`rock_points`): an animated stream |
| Convex Pile | built | Hulls of 32 random points (`complex_hull_points`) |
| Spinner | built | Inward cylinder mesh, paddle on a revolute motor to the mesh's body |
| Sleep | work | Pyramids woken by a filter joint made and destroyed when asleep: `step_script` writing `awake` |
| SAT vs GJK | no | Only with `BOX3D_INTERNAL_BENCHMARKS`; collision timing, no simulation |

## Bodies (`sample_bodies.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Body Type | built | Jointed platform and bodies whose type (`Bodies.menu`) and `enabled` are set before the run; kinematic is an animated stream, the platform's bounce path worked out in VEX |
| Spinning Book | built | Boxes spun about each axis, no gravity (`w`, `gravityscale`) |
| Gyroscopic Torque | built | Cylinder plus box in one body (`Bodies.parts`) |
| Gyroscopic Precession | built | Spinning tops (one hull), `fastrotation`, rolling resistance; readout dropped |
| Spinning Capsule | built | Spinning capsule; Near-Sphere Tolerance; readout dropped |
| Weeble | built | Custom mass data (low centre): `computecom` 0 with the point at the centre; starts at Teleport's pose (Start menu), Explode dropped |
| Disable | built | Welded capsule chain from a kinematic link, an impulse every step (`step_script` writing `force`), `enabled` toggles |
| Cast | no | Body ray, shape and mover casts |
| Kinematic | built | Kinematic box on a path after a delay (`SetTargetTransform`): an animated stream |
| Fixed Rotation | built | `fixedRotation`: no attribute; Lock Tilt and a large `inertialtensorstiffness` |
| Class Ring | work | Ring of capsules with a sphere gem as one body: `multi_shape_body`; 16 steps a frame at 8 substeps (raise Substeps), `fastrotation` |
| Offset Kinematic | built | Kinematic box spinning about an offset centre: an animated stream with the point at the centre |

## Character (`sample_character.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| CapsulePlane | no | Mover plane solver (`b3World_CollideMover`, `b3SolvePlanes`) |
| MoverOverlap | no | Mover overlap queries |
| Geometric Mover | no | Keyboard character mover |
| Dynamic Mover | no | Keyboard character mover with sensors |
| Rigid Body | no | Keyboard-driven rigid body character, ray and shape casts |

## Collision (`sample_collision.cpp`)

All **no**: they show queries or narrow-phase tools, not a simulation.

| Sample | Why |
|---|---|
| Ray Curtain | Ray casts against moving shapes |
| Cast World | World ray and shape casts |
| Mesh Scale | Casts against a scaled mesh |
| Shape Cast | Shape casts |
| Overlap World | Overlap queries |
| Long Ray Cast | Long ray casts |
| Initial Overlap | Shape cast from inside a mesh |
| Shape Cast Debug | `b3ShapeCast` |
| Encroach Cast | `b3ShapeCast` |
| Distance Debug | `b3ShapeDistance` |
| Shape Distance | `b3ShapeDistance` |
| Time of Impact | `b3TimeOfImpact` |
| Capsule Cast Ray | Body ray cast |
| Hull 2D | 2D hull simplification |

## Compound (`sample_compound.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Simple | work | Baked compound of hulls on a dynamic body: `Bodies.parts`; Contact Recycle Distance |
| Spheres | work | Compound of spheres: glued sphere pieces (`multi_shape_body`) |
| Hulls | work | Compound of box hulls: `Bodies.compound` |
| Tile Floor | work | Static compound of tiles: one static box piece per tile |
| Mesh Tile | work | Static compound of box meshes: static `concave` pieces |
| Village | no | Keyboard character mover and compound queries |

## Continuous (`sample_continuous.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Thin Wall | built | Fast sphere, capsule and box against a thin wall (`v`, `w`) |
| Bounce House | built | Ball bouncing inside four static walls (hulls of one body: four static boxes), `bounce`, no gravity |
| Spinning Stick | built | Fast spinning stick, its spin the sample's first `RandomVec3` (`random_vec3`) |
| Bullet vs Stack | work | A `fastobject` sphere fired into a stack by key: emitted at a frame with its `v` (`emit`) |
| Needle Mesh | built | A thin plate thrown onto four needle meshes (open cones) |
| Mesh Drop | built | `mesh_drop.c`'s 1024 bodies on a walled wave mesh: one generation, a fixed seed (`XorShift`), the Shape menu; readouts dropped |
| Rough Mesh Drop | built | Same, rough mesh |
| Hump Mesh | built | A thin plate thrown onto the ridge of a custom mesh |
| Is Fast | built | Long boxes spinning in the air under the fast-body test (its capsule option is off) |
| Stall | built | A rock (`hull_points`) fired into a dense torus mesh, `fastobject`; Launch as one every second (`emit`) |
| Safety Factor | built | `ccdthreshold`; readout dropped |

## Determinism (`sample_determinism.cpp`, `shared/determinism.c`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Falling Ragdolls | work | `human` groups on grid and torus meshes; the hash check dropped |
| Wave Pile | work | Mixed shapes and rocks on a wave height field; the hash check dropped |
| Query Spawn | no | Spawn points come from world queries during the run |
| Mesh Drop | work | Shapes on a mesh; the hash check dropped |

## Events (`sample_events.cpp`)

| Sample | Verdict | What it uses / why |
|---|---|---|
| Sensor Visit | no | Sensor events destroy what enters |
| Hit | work | Hit events: Add Impact Data records Impacts instead of markers |
| Move | work | Move events coloured by sleep: colour by the written-back `awake` after the solver |
| Joint | built | Distance, prismatic, revolute and weld joints with Break Force / Torque (Breakable); motor and wheel compiled out |
| Contact | no | Keyboard player absorbing bodies on contact begin events |
| Persistent Contact | no | Contact begin / end events and contact data |
| Sensor Hits | no | Sensors on moving bodies |

## Geometry (`sample_geometry.cpp`)

All **no**: hull building and mass properties drawn, no simulation (Box Hull, Hull, Hull Reduction, Hull Transform,
Capsule Mass).

## Ghost (`sample_ghost.cpp`, `shared/ghost_collisions.c`)

All **work**: the probes roll over custom meshes as in Box3D, but the ghost-contact counts (contact data) and the
probes' resets are not reproduced.

| Sample | What it uses |
|---|---|
| Crest Ladder | Probes over a stepped crest mesh |
| Hub Peak | Probes rolling off a mesh peak |
| Saddle Vertex | Probe over a saddle vertex, reset by GUI |
| Diagonal Roll | Probe rolling across diagonal triangle edges |
| Fine Mesh Load | Boxes and capsules resting on a fine mesh; profile readout dropped |
| Scenarios | `ghost_collisions.c` scenarios, one scene each |
| Conveyor Circuit | Per-triangle conveyor materials: one static piece per material with its `tangentvelocity` |
