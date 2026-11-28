# Box3D sample ports: the code

The modules and helpers that build the sample ports. The plan, output layout and how a port is checked:
[sample-port-plan.md](sample-port-plan.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

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
    body's points, `face` a polygon on their numbers; `prism`, `hull_points`: `b3CreateHull` of a point set;
    `capsule(name, a, c, r)`: a capsule's polygons that fit back exactly as a `bullet_georep` capsule); `mesh`
    makes static `concave` bodies from `MESH_FUNCTIONS` (`tri` in Box3D's winding, `welded` for a triangle soup,
    `grid_mesh`, `wave_mesh` (`_turned` for a turned body), `box_mesh`, `torus_mesh`: `src/mesh.c`'s builders, each adding Box3D's vertices once, in
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
    hulls on one moving body: named pieces of one packed shape, `concave`; `Bodies.parts` for any hulls, capsules and
    spheres, with a density scale per part), `Bodies.mesh_file` (a mesh from
    `extern/box3d/data/meshes`, copied to `examples/cache` and read `$HIP`-relative, as `LoadTempMesh` reads it, its
    `weldTolerance` a Fuse). A hull's points come from a program linked with Box3D when Box3D builds them (GMod Wheel
    Stack), printed to full precision: `num` writes 9 significant digits, and rounded points can change a hull's faces.
  * `box_impulse`: the velocity a `b3Body_ApplyLinearImpulse` before the first step gives a box; `XorShift` and
    `random_vec3`: the samples' random numbers (`shared/utils.h`), computed in Python (VEX has no bit shifts);
    `complex_hull_points` (`b3CreateComplexHull`), `rock_points` (`b3CreateRock`), `cos_sin` and `f32`, for points a
    builder lists.
  * `finish(step=vex)`: `step_script`, a sample's `Step()` before `b3World_Step` (a SOP Solver ahead of the Box3D
    Solver in a Multiple Solver; a changed `v` / `w` is an edit). `Bodies.slider` / `toggle` / `menu`: a sample's GUI controls, read by the streams' VEX.
  * `finish(keys=...)` (`key_parm`): a relationship node's parameter keyed, each key held until the next, for a key the
    sample's `Step()` presses (Driving's throttle and steering). Its operation menu (`parmop_<name>`) goes to Set
    Always: at Set Initial, as the nodes start, the joint keeps the first frame's value.
  * `Bodies.explode`: `b3World_Explode` before the first step, a wrangle (`EXPLODE_VEX`) adding each box, sphere or
    capsule body's push to its `v` / `w` (Box3D's to float rounding); its settings are on the Sample tab.
  * `network_sop` takes a VEX string calling `joint(...)` in place of lines, for thousands of joints (Joint Grid).
  * `human.py`: `shared/human.c`'s ragdoll. `Humans(bodies, [(position, groupIndex)], frictionTorque, hertz,
    dampingRatio)` makes a stream per bone for every person; `nodes()` / `lines()` are its joints for `finish`,
    `motor_nodes()` each node's share of the friction torque (for keys), `poses()` the bones' starting poses. Capsules
    off the body's axes are `bullet_primT` / `bullet_primR` (`capsule_shape`). The calf (shin and foot capsules, the
    foot at half density) is `Bodies.parts` of two capsule parts. A foot welded on instead flexes: Box3D caps a joint's
    Constraint Stiffness at a quarter of the substep rate (60 Hz here).
* `make_examples.py --samples` builds the samples; names select a category (`stacking`) or one sample
  (`stacking/card_house`). Without `--samples` it builds the nine scenes as before. `--check` simulates each.
