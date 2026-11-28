# Porting Box3D's samples to example scenes

How Box3D's own samples (`extern/box3d/samples`) become Houdini scenes built by `tools/examples/make_examples.py`. The
verdict for every sample: [sample-port-table-a.md](sample-port-table-a.md) (Benchmark to Ghost) and
[sample-port-table-b.md](sample-port-table-b.md) (Issues to World). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## The samples

* Each sample is a `Sample` subclass in `samples/sample_<file>.cpp`, registered by a static
  `RegisterSample( category, name, Create )` (`sample.cpp`). The app's menu groups by the category string, which
  matches the file (`sample_joint.cpp` is "Joints"), except Raycast Vehicle: category "Vehicle", in `sample_issues.cpp`.
* Shared scene builders live in `extern/box3d/shared/` (C): `benchmarks.c` (pyramids, rain, washer, junkyard, village,
  trees, spinner, sleep), `human.c` (the ragdoll), `mesh_drop.c`, `determinism.c`, `ghost_collisions.c`.
* A sample builds its world in its constructor; `Step()` may script it per step (kinematic targets, forces, spawning);
  `UpdateGui()` sliders and `Keyboard()` change it live; `Render()` draws readouts. Box3D's sample app steps at 60 Hz
  with 4 substeps (`SampleContext` in `sample.h`) and gravity (0, -10, 0) (`b3DefaultWorldDef`).
  `AddGroundBox( extent )` is a static box whose top is at y = 0, as `ground()` in `make_examples.py`.
* 209 registrations, two of them compiled out (`#if 0`: Benchmark / Large World's first copy, Ragdoll / Pose) and one
  only with `BOX3D_INTERNAL_BENCHMARKS` (Benchmark / SAT vs GJK).

## Verdicts

Of the 207 live samples: **81 portable now**, **68 with a workaround**, **58 not portable**. The tables mark the
ported ones **built** in place of their verdict.

* Not portable, by reason: no simulation, only queries or geometry tools (Collision, Manifold, Geometry, Tree, Replay,
  query benchmarks, mesh and hull viewers); character movers or keyboard control (Character, Village, Contact, Raycast
  Vehicle); sensors and contact begin / end events (no output for them, [todo.md](todo.md)); world queries during the run
  (Query Spawn); wind lift (Wind Drop).
* The common workarounds (helpers still to write: [todo.md](todo.md)): a kinematic body is an animated piece; a body of several shapes
  is glued pieces with Glue as One Body, or `compound` with Hull per Connected; a scripted step is keyed parameters, a
  SOP Solver writing point attributes, or pieces appearing at a frame; an explosion is starting `v` / `w`
  (`Bodies.explode`); wind is drag (`airresist` / `targetv`); `fixedRotation` has no attribute (a large
  `inertialtensorstiffness`, or Motion Lock's Lock Tilt for two axes); per-triangle mesh materials are one static
  piece per material.
* Readouts the samples draw (energy, ghost counts, hashes, profile counters) are not reproduced. The sticky note says
  what to look at instead.

## Output layout

* `examples/<category>/<sample>.hip`, both snake case from the registration
  (`stacking/card_house.hip`, `joints/ball_and_chain.hip`, `issues/sbox_mover.hip`, `robustness/high_mass_ratio_1.hip`).
  The category folder is the menu's, so the folders read like Box3D's sample browser; with Raycast Vehicle not ported,
  category and source file agree. Names repeated across categories (Mesh Drop in Benchmark, Continuous, Determinism)
  stay apart by folder. The category folders sit beside `cache/`, a name no Box3D category has.
* The nine existing scenes are `examples/<number>_<name>.hip`; data the builds make or copy goes in `examples/cache/`.
* Failed builds save `<sample>_FAILED.hip` as the scenes do.

## Code organisation

* `tools/examples/scene_kit.py`: the helpers `make_examples.py` and the samples share (`Pieces`, `mk`, `network_sop`,
  `build_dops`, `tidy`, ...).
* `tools/examples/box3d_samples/`: one module per Box3D source file (`stacking.py` for `sample_stacking.cpp`), one builder
  per sample, registered by `@sample(category, name, source, frames)` in `common.py` (`SAMPLES`, keyed
  `category/sample`). `__init__.py` imports the modules. Not-portable samples get no builder; the tables are their record.
* `box3d_samples/common.py`:
  * `start_scene` (`FPS`, Box3D's step rate) and `box3d_world(solver)`, applied by `finish`: Box3D's sample settings
    where the solver's own differ (`BOX3D_SOLVER`): one Box3D step per frame, so a sample's step counts are frame
    numbers; `b3DefaultWorldDef`'s gravity and contact tuning; friction mixing
    geometric mean and bounce mixing maximum (Box3D's default callbacks); Bounding Box Inertia off.
  * `Bodies`: the geometry object of a sample's bodies. Shapes in Box3D's terms: `box(hx, hy, hz)` (half extents),
    `sphere(r)` (a Sphere primitive, a true sphere), `capsule(h, r)` (along y; VEX `along_x` turns it to Box3D's x; set
    by hand with `bullet_autofit` 0, as AutoFit can tilt it), `cylinder(height, radius, y_offset, sides)`
    (`b3CreateCylinder`'s points in its order, at its approximate sines, collided as their hull),
    `shifted(shape, offset, rotate)` (a shape off its body's origin: `b3MakeOffsetBoxHull`, a capsule along z). `stream`
    copies a shape onto the bodies its VEX makes (`body(p, q)`, `static_body(p, q)`), each packed with its pivot at the
    point, so the point is Box3D's body origin; `polygons` packs per-name polygons into hull bodies (`points` adds a
    body's points, `face` a polygon on their numbers; `prism`, `hull_points`: `b3CreateHull` of a point set); `mesh`
    makes static `concave` bodies from `MESH_FUNCTIONS` (`tri` in Box3D's winding, `welded` for a triangle soup,
    `grid_mesh`, `wave_mesh`, `box_mesh`, `torus_mesh`: `src/mesh.c`'s builders, each adding Box3D's vertices once, in
    its order). Faces share their points: a mesh's triangles share vertices and edges as Box3D's do, and a hull's faces
    are one connected piece (the Box3D Solver SOP re-assembles separate faces into a body of many hulls).
    `ground(extent)` is `AddGroundBox`. Every point carries `BOX3D_MATERIAL` (Box3D's shape material defaults) and the
    sample's own attributes with their defaults, so no stream is filled in with 0 by the Merge; a stream's defaults
    must be among them.
  * VEX `cossin` and `axis_angle` are `b3ComputeCosSin` and `b3MakeQuatFromAxisAngle`, approximation included (40
    degrees comes out as 40.149): with exact angles bodies start in other places than Box3D's (Inclined Plane's boxes
    2 cm higher, enough to tumble the one that holds in Box3D).
  * `finish`: the network (`motion_locks` for `Sample::LockMotion`), the DOP network, `box3d_world`, the notes. With
    `emit=True` a Box3D Solver SOP with Emit RBDs replaces the DOP network, for samples that make bodies during the
    run: bodies whose `rbdbullet_emit` is `Every(n)` (1 on frames 1, 1 + n, ...) are made again then, the rest once.
    `statics`: Static Objects merged in, such as `height_field` (a HeightField SOP placed as Box3D places a field, its
    corner sample at the body's position).
  * More shapes: `Bodies.hull(points)` (one point set copied onto several bodies), `Bodies.compound(boxes)` (box
    hulls on one moving body: named pieces of one packed shape, `concave`; `Bodies.parts` for any hulls), `Bodies.mesh_file` (a mesh from
    `extern/box3d/data/meshes`, copied to `examples/cache` and read `$HIP`-relative, as `LoadTempMesh` reads it, its
    `weldTolerance` a Fuse). A hull's points come from a program linked with Box3D when Box3D builds them (GMod Wheel
    Stack), printed to full precision: `num` writes 9 significant digits, and rounded points can change a hull's faces.
  * `box_impulse`: the velocity a `b3Body_ApplyLinearImpulse` before the first step gives a box; `XorShift` and
    `random_vec3`: the samples' random numbers (`shared/utils.h`), computed in Python (VEX has no bit shifts);
    `complex_hull_points` (`b3CreateComplexHull`), `cos_sin` and `f32`, for points a builder lists.
  * `finish(step=vex)`: `step_script`, a sample's `Step()` before `b3World_Step` (a SOP Solver ahead of the Box3D
    Solver in a Multiple Solver; a changed `v` / `w` is an edit). `Bodies.slider` / `toggle`: a sample's GUI controls.
  * `Bodies.explode`: `b3World_Explode` before the first step, a wrangle (`EXPLODE_VEX`) adding each box, sphere or
    capsule body's push to its `v` / `w` (Box3D's to float rounding); its settings are on the Sample tab.
  * `network_sop` takes a VEX string calling `joint(...)` in place of lines, for thousands of joints (Joint Grid).
* `make_examples.py --samples` builds the samples; names select a category (`stacking`) or one sample
  (`stacking/card_house`). Without `--samples` it builds the nine scenes as before. `--check` simulates each.

## Checking a port

A port is checked against Box3D itself: a small C program linked with `build/box3d/src/Release/box3d.lib` (compiled
with `NDEBUG`) builds the sample as its C++ does, steps it at 60 Hz with 4 substeps and prints the same measurements
as a hython script reading the scene's DOP object each frame (rest heights, rebound peaks, distances travelled).
Sticky notes state only what that comparison showed.

* The program needs `/MD` (the library's runtime). A packed point's `P` is the body origin
  (`b3Body_GetPosition`, not the centre of mass), and frame f is the state after f - 1 steps.
* Ties between contact points can go the other way in Houdini (Hollow Box: one of four side cylinders drifts down
  where Box3D's drifts up), as the hull's vertex order differs; the sticky note says so.

## Mapping a sample

* Bodies: static -> `active 0`; kinematic -> an animated piece (`active 0`, `animated 1`,
  a stream whose VEX poses it by `@Time`: the pose at T is Box3D's after the step ending at T); dynamic -> `active 1`. `linearVelocity` /
  `angularVelocity` -> `v` / `w`; `gravityScale`, `linearDamping`, `angularDamping`, `enableSleep`, `isAwake`,
  `isEnabled`, `allowFastRotation`, `enableContactRecycling` -> `gravityscale`, `lineardamping`, `angulardamping`,
  `enablesleep`, `awake`, `enabled`, `fastrotation`, `contactrecycling`; `isBullet` -> `fastobject`;
  `safetyFactor` -> `ccdthreshold`. Box3D's default density is `BOX3D_MATERIAL`'s.
* Materials: `friction`, `restitution`, `rollingResistance`, `tangentVelocity` -> `friction`, `bounce`,
  `rollingresistance`, `tangentvelocity`. Filters: `categoryBits`, `maskBits`, `groupIndex` -> `collision_category`,
  `collision_mask`, `collision_group`.
* Joints: one network polyline per joint, each end at its frame's world position (the same point but for a distance
  joint's anchors), `orient` the frames' rotation on both ends (a revolute turns about the frame's z, a prismatic
  slides along x, as in Box3D). Frames that differ only by a free turn (the wheel's spin axis) make the same joint.
  A joint to the sample's ground or mesh body names that body (`ground0`, `drum`), not the world, when its other body
  touches it: a joint's bodies do not collide (Door's bottom edge, Spinner's paddle ends).
  One relationship node per distinct setting (`joints.node`: its Data Name its own name, since nodes of one type share
  a default one), or primitive attributes named like the parameters (Attributes Replace Values). `hertz` is
  Stiffness, angles and angular velocities in degrees, `constraintHertz` -> `constraint_stiffness`,
  `forceThreshold` / `torqueThreshold` -> Breakable (Box3D only reports them; the samples destroy the joint).
  A sample's joint sliders and checkboxes are the node's parameters, set to the sample's starting values.
* World: `b3World_SetGravity` -> Gravity; `SetContactTuning` -> the contact parameters; restitution threshold ->
  Bounce Threshold; Isotropic Inertia Tolerance -> Near-Sphere Tolerance; contact recycle distance -> Contact Recycle
  Distance. Joint tuning has no world setting: Constraint Stiffness per node.
* GUI sliders: the defaults are baked in; the one worth playing with becomes a spare parameter on the pieces' object, as
  Count in `scene_many_bodies`. Radio buttons that pick a body are `Bodies.choice`: a stream per option (`merged=False`)
  into a Switch SOP read from a menu on the bodies object's Sample tab (Mesh's Shape). Parameters a sample changes at a
  time become keyframes (joint settings are live).
* Sticky note: what the sample shows, `Box3D sample <Category> / <Name> (samples/sample_<file>.cpp)`, what was
  changed for the port, and what to try.
