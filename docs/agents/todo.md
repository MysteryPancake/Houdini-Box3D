# TODO

What is still to do, and what to add to the Box3D fork. Keep each item to a line or two; remove it once done. What is
unverified: [status.md](status.md), [status-areas.md](status-areas.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

* Glue's frame cost on a 3200 brick wall (`bench.py wall:w=80,h=40`, Glue as One Body on; 5.5 ms, welds 10 to 13): of
  Updating Constraint States' 1.1 to 1.4 ms, Advancing Glue was 0.2 (below) and Writing Joints (`jointsAfterStep`) 0.9
  to 1.1 (welds 1.6 to 2.1). Timed inside it, `markQuietJoints` + `allQuiet` are 0.06 to 0.15 ms and everything else
  is `writeNetworkResults` writing the moved joints' `current_P` / `current_orient` and readings (1.0 ms awake, 0
  asleep, already parallel and only for joints that changed), so a dirty count would save nothing: only writing fewer
  attributes would. Glue Hits is 0.4 to 0.5 as a compound and 1.5 on welds, from two `collectGlueHits` calls a frame
  (Substeps 2). As a compound each call is 0.2 to 0.25 ms: the prelude and the impulse events cost nothing, 0.04 to
  0.07 is `contactImpulsesByShape` of the one compound, and 0.15 is the loop over 3200 pieces (about 50 ns each: the
  `compoundOf` lookup is 0.05 of it, the rest cache misses on the member and compound structs). Keeping each glue
  node's compound and member index (rebuilt when `compoundOf` changes: five places in `Box3DCompoundSync.cpp`) and
  looping over compounds (`Box3DCompoundPieces.cpp`) would remove most of that, about 0.25 ms of the 5.5 a frame. On
  welds `Glue Hits` is 0.65 a frame (it was 1.5 with a record per contact point: 12500 points a call to convert and
  look up twice each) from the per-pair records of `getContactImpulseSums`, `Box3D Step` is 4 to 5 and the rest is the
  weld solve. Advancing Glue is 0.14 ms awake and 0.003 asleep on that wall (6280 bonds; the bond list is kept between
  frames, `glueMirror`): `glueAdvance` 0.10 (spread 0.055, decay and break test 0.05) and the copy back 0.03 are what
  is left. Spreading only from nodes with hits would shrink the spread to the base row.
* `glue_fracture_vs_bullet` (Box, Points from Volume, Voronoi Fracture's constraints named Glue, frame 24): Box3D's
  equivalent Strength is 0.3 to 1.5x Bullet's with Glue as One Body on and 0.5 to 0.7x with it off, only checked on
  that scene.
* `glue_compound_bridges` (a beam balanced on a block, a ball falling on its tip) passes only at Strength 2e5. At 1e5
  the hold frees 5 of 8 pieces on the first frame, `kGlueHoldHaloScale` times the ball's size around its path, and the
  3 left in the compound are held up by one bridge weld and creep off the block; a halo of 1 fixes that but the ball
  bounces back in `glue_compound_hit`. At 3e5 the beam tilts: a bridge weld carries the hit into the compound in the
  step it lands, where welds would have broken. Welds hold the beam at every Strength tried.
* Point affectors answer nothing back: Bullet writes `Feedback` impacts onto the particles (a POP object with a
  Collide Relationship the box affects), which Box3D does not, and its box ends slower than ours after a row of moving
  particles hits it (`point_affectors`: 5.1 m against 7.5 m): Bullet's particles are not infinite mass. Particles with
  thousands of points are a body each, untimed.
* Glue's break timing against Bullet's (breaks before the step, Box3D after it; `glue_row_scene`, Glue as One Body on).
  A pair landing from 1, 3 or 6 breaks on the same frame in both, at every Strength tried, and its pose shows nothing of
  the late break (two free boxes sink into the ground just as far). A box dropped 4 on a pair or a row of 4 breaks one
  frame later in Box3D at Substeps 2 (24 against 23 at Strengths 1.2e4 and 2.4e4) and on the same frame at Substeps 4
  for the pair at 1.2e4 (not at 2.4e4, nor for the row). The cause is not where in the frame the bond breaks: the
  falling box is still in free fall (v.y -9.0) at the end of frame 23 with its bottom 0.14 into the pair, Box3D making no
  contact between two dynamic bodies until the next step's pair update, so no hit exists to break on before frame 24
  (Substeps 4 gets within the contact margin one substep earlier). Breaking between substeps would not move it; the
  contact has to be predicted before the step (a shape cast of each moving glued body, as `holdCompoundsForApproach`
  finds a fast body's path) or found by an overlap query after it. At Strengths so low that the weight breaks the bonds
  (3e3), Bullet breaks them on frame 2 to 4 and Box3D on 7 to 10, a different cause: Bullet estimates every contact of
  the group each frame (a resting body's gravity step is its speed), where Box3D estimates only hits and counts the rest by
  their real impulse.
* Box3D Configure has no Units (RBD Configure's Unit Mass and Unit Length, which scale Density by
  `unitlength / unitmass`, each over 1000).
* Drag and speed limits (`Box3DMotion.cpp`, [motion-limits.md](motion-limits.md)): World Scale other than 1 does not
  scale the values.
* Bounding Box Inertia ([body-features.md](body-features.md)): a Glue as One Body compound takes the true inertia of
  its merged shapes, where Bullet's chunk sums the pieces' box inertias, so a glued group spins more freely than
  Bullet's. `kBulletInertiaPad` is a fixed length, as Bullet's margin is: a tetrahedron matches at 0.2, 1 and 5 times the
  size (within 1.2%), but the pad that would match an icosahedron is 0.115 against the tetrahedron's 0.08, so it is 14%
  off at 0.2 m, 3% at 1 m and 1% at 5 m (`bullet_hull_inertia`); what sets the difference between hulls is not known.
* Whole objects and Bounding Box Inertia: a disc RBD Object's inertia is 7% below Bullet's with the toggle on (the
  pad is Bullet's for a hull; `rbd_object_inertia_bullet`). RBD State's Use Bounding Box inertia has no pad, so it
  is 14% below that, where Bullet ignores the menu.
* Quadric primitives (how they are sampled: [bodies.md](bodies.md)): a denser sphere mesh (Polygon Mesh, NURBS) rests
  at 0.46 to 0.48 where Bullet rests at 0.499, because a Box3D hull keeps about 44 vertices and loses the poles
  (keeping the farthest point per direction rested lower: a 38-vertex ball tips onto a face). Volumes: Bullet's packed
  one falls through, its RBD Object stays at 2, Box3D's falls.
* Objects solved by another solver: only RBD Bullet Solver's are tried (`other_solver`: packed ones are ignored as
  colliders, polygon ones push Box3D bodies one way). Vellum, FLIP and the rest are not.
* Height fields: holes from a mask layer, height fields on points.
* Agent ragdolls ([agent-ragdolls.md](agent-ragdolls.md): bodies, joints, the per-transform arrays and the agent's pose are
  built; the PinnedRagdolls ragdolls hang from their pinned hand as Bullet's do). Not done, in order:
  1. The gap to Bullet (PinnedRagdolls; the Box3D Solver's Gravity at 0, as the example has a Gravity DOP, which
     `agent_oracle.load` does): the bodies are 0.02 from Bullet's with its motors off on average after 4 frames, 0.05
     after 9 and 0.11 after 12 (0.04, 0.10 and 0.17 with the example's motors on), then 0.15 to 0.27 to frame 48 as
     the swinging ragdolls part. The first step solves no joint, as Bullet's. Left:
     joints that start outside their limits (the wrists' twist 41 and 22 degrees against 5, the shoulders' 41 against
     20) are eased in by Bullet over about ten frames (soft: `bias_factor` 0.3, `cfm` 0.2, `softness`, the solver's
     `globalcfm` 0.025, none read) and put back in one step by Box3D. A softer joint (`constraintStiffness` 1 to 10 Hz
     on Cone Twist) softens the point constraint too, so the ragdolls stretch off the pin. A separate softness for the
     spherical joint's limits alone, tried in the fork at 1 to 8 Hz, is closer to Bullet on frames 2 to 4 and further
     from frame 9 on (0.15 against 0.106 at 12, 0.33 to 0.38 against 0.27 at 24), so it was not kept. Bullet's five
     agents swing apart unevenly (agent 2 is 0.67 to 0.98 from its neighbours' bodies on frames 15 to 30) and Box3D's
     identically, so they come within 0.18 of each other (frame 24) and touch. Against round cones the elliptical ones
     halve the gap to frame 9 (0.106 against 0.123 at 12) and let the knees bend as Bullet's do (22 to 28 degrees by
     frame 12 against 5); after frame 16 neither is closer. Bullet's motors and
     stiffness move its bodies by 0.04 to 0.11 and are not built.
  2. Sheared transforms (a non-uniform scale above rotated transforms: `MASTER` scaled (1, 1.6, 1), rows up to 64
     degrees apart). The hull is the sheared shape; Bullet's capsule masses are 0.2 to 3.5% above ours, in no relation
     to the matrix found, and with the pins removed the bodies are 0.16 from Bullet's on frame 6 against 0.07 unsheared
     (0.09 for a uniform scale of 1.6). How Bullet fits a capsule to a sheared matrix is not known. Custom shapes:
     Bullet samples their volume (a tetrahedron's mass 0.71 to 0.98 of the density times its volume as it grows, a
     cone's 1.03 to 1.08, its centre 0.5 units off the centroid), so ours, exact, are only near.
  Unknowns: Bullet's motor model against Box3D's joint motors, the cost of 27 bodies an agent at crowd sizes. Tests so
  far are `agent_*`; the rest go in `checks_agent.py`.
* Constraint networks: the `anchor_pid` / `anchor_vid` matching. A missing
  `constraint_type` is `position` for Hard, Soft and Spring (`sidefx_hard_untyped`).
* SideFX constraint data not mapped: Soft plasticity, Cone Twist's cones wider than 90 degrees, softness, motors and
  translation ranges, Hard's angular motors. Bullet makes no Cone Twist joint for a line whose `constraint_type` is
  missing or `position` (only `rotation` and `all`); the plugin makes one for any, and `bullet_cone_twist_goal` /
  `_apart` build their lines without it, so Bullet's piece falls freely there.
* Slider Constraint Relationship (`sliderconrel`, Bullet's `btSliderConstraint`: a slide range along the first anchor's x and
  a rotation limit about it, nothing else free) is not read. A slide range of 0 is a revolute joint and a rotation limit
  of 0 a prismatic one; both free (a cylinder) has no Box3D joint. Nothing in Houdini creates one but the node itself.
* Box3D's samples as example scenes ([sample-port-plan.md](sample-port-plan.md)): the tables mark what is built.
  Left, in order:
  1. Joints with workarounds (Door, Locked Joints, Top Down Friction, Driving), then Benchmark's portable ones
     (pyramids, Convex Pile, Joint Grid, Spinner), timed with `--check` a few at a time.
  2. Animated pieces and step scripts: Bodies, Kinematic, Rolling Deck, Turning Platform, Motor Joint, Junkyard,
     Washer, Static Invoke, Disable.
  3. The ragdoll: Ragdoll, then Rain, the Determinism scenes and Mesh / Reflection (`building.obj`: `mesh_file`).
  4. Multi-shape bodies and per-material meshes: Compound, Class Ring, Falling Trees, Village (benchmark), Gear Lift,
     Ghost, Conveyor Mesh; then the wind approximations.

  Helpers these need in `tools/examples/box3d_samples/common.py`:
  * `animated_piece(vex)`: `active 0`, `animated 1`, `P` / `orient` driven by a wrangle on `@Time` (kinematic bodies).
  * `step_script(vex)`: a SOP Solver in a Multiple Solver after the Box3D Solver, writing `force`, `v`, `enabled`,
    `awake` or `dead` on given frames, for samples whose `Step()` does that.
  * `multi_shape_body(parts)`: several shapes as one body. Hulls only: `Bodies.compound` (named pieces, `concave`)
    extended past boxes. With spheres or capsules: glued pieces, Glue as One Body on, without other joints (a glued
    group with another joint stays on welds, [glue-compound.md](glue-compound.md)).
  * `human(name, position, friction_torque, hertz, damping)`: `human.c`'s ragdoll: capsule pieces, revolute and
    spherical joints with its limits, springs and friction motors (motor speed 0), filter joints; shin and foot as one
    `multi_shape_body`.
* Force DOPs: parallel evaluation of circle / sphere sampled forces (the resolver has only the point call).
* Sticky collisions. Box3D's contact begin / end and sensor events: Houdini has no output for them (Bullet has none), so
  nothing reads them beyond Impacts. Sensors (trigger zones) would be the useful part.
* Box3D Solver SOP:
  * Emit RBDs (`b3d_solver_emit.py`): untested with Global Substeps and
    Deforming pieces (whose Packed Object reads the whole SOP every step, which would drop the emitted pieces), and
    its cost on many pieces. Replicate RBDs is untested with Emit RBDs on.
  * Break Thresholds (`b3d_solver_breaks.py`): `angle` matches Bullet's for a freely spinning piece, and `torque` is
    Bullet's quantity (the force's moment about the first piece's centre, `bullet_joint_feedback`), so the Torque
    Threshold breaks on the same frames (`solver_sop_break_angle_torque_bullet`, INFO). A Hard (all) line reads only
    part of the load when its two pieces touch (7.3 N for a hanging 1 kg piece, 6.9 N with the anchor at the fixed
    piece's centre): Box3D reads the joint's own force, and friction at their shared face carries the rest. With
    Disable Collisions on it reads the weight, and Bullet's reads the weight either way, being the pieces' net force
    (`bullet_hard_collisions`). The Angle and Torque thresholds of a Soft line on a spinning box break on the
    same frames in both with Rest Length 0, and Bullet's Soft Stiffness is the frequency in Hz that the mapping takes it
    for (`bullet_soft_spring_hz`), and with the default Rest Length of 1 and the anchors at one point the Angle
    Threshold of 0.2 goes on frame 3 in both (the anchors' shift is aimed again each frame as Bullet's is:
    `bullet_soft_coincident_anchors`); the later frames' angle runs 0.50 / 0.42 on frame 5. Untested: the cost on a large network with it on (the SOP
    Solver rewrites the constraint geometry each step, so the solver reads the whole network again). Not there: the
    Force Threshold (Bullet's `force` of a Glue is the difference of its pieces' forces times their mean mass, Box3D's
    is the joint's), the Group parameter, plastic flow, the VEX snippet.
* Glue as one body (`gluecompound`, `Box3DCompound*.cpp`, how it works: glue-compound.md; on by default, so the
  glue tests run it; `glue_compound_break`, `glue_compound_load`, `glue_compound_vs_welds`, `glue_compound_hit`,
  `glue_compound_sleep`, `glue_compound_id`, `glue_compound_kinematic`, `glue_compound_many_groups`,
  `glue_compound_guide`, `glue_compound_impacts` and `core_compound` check it). Not covered: pieces with another
  joint or a custom inertia (they stay on welds, as does their whole group), a group changed only by editing a piece's settings
  while it never merged (it is worked out again when the Glue graph or the bodies change).

## Box3D fork (`extern/box3d`)

Features to add in the fork. Once one exists, the wrapper maps it and the entry leaves this list.

* **Parallel joint orbit.** A parallel joint with a twist limit orbited at about 72°: not understood.
* **Clipping into a fast kinematic collider.** A box resting on a collider lifted 0.5 a frame sinks into it instead of
  riding on top (`solver_sop_solve_first_frame_bullet`, INFO).
* **Level landings.** A box landing flat at speed tips up to about a degree, where Bullet's stays level (status.md):
  the manifold's points are solved in order. Alternating the order each pass, or solving a face contact's points
  together, would keep it level.
* **Wide cones.** `b3SphericalJoint_SetConeLimit` and `SetConeLimits` assert each angle is at most 90°: Cone Twist's
  wider cones need that lifted.

Workarounds in the wrapper that the engine could own, roughly by value:

* **Worlds allocated on demand.** `B3_MAX_WORLDS` sizes the static `b3_worlds` array at compile time, so the build
  passes `BOX3DDOP_MAX_WORLDS` through to Box3D and the solver reports "world limit reached" past it.
* **A hull build that always succeeds.** `b3CreateHull` fails when the vertex, face or edge limits are exceeded, or
  the hull is thin (thinner than the larger of `B3_LINEAR_SLOP` and 0.1% of its largest extent), so
  `createHullWithKey` retries smaller vertex budgets (a triangulated hull hits the half-edge limit well below the
  vertex limit), `fitShapeOriented` and `hullCentroid` use the same ladder (`kHullBudgets`), and a per-group hint
  remembers the budget that worked. Wanted: simplify until it fits, or say which limit failed.
* **Mass overrides that persist.** Houdini's point is the centre of mass. `applyMassByHandle` therefore moves the
  centre to the body origin (parallel axis theorem by hand), scales the inertia or installs an explicit one with
  `b3Body_SetMassData`, and must run again after every shape change or type change, before the next step. Wanted: a
  body-level override (centre at origin, inertia scale, explicit inertia) that survives
  `b3Body_ApplyMassFromShapes`.
* **Events across steps.** Box3D clears move, joint, hit and impulse events at the start of each `b3World_Step`, so
  with Substeps the solver merges them after every step (`brokenThisFrame`, `st.moves`, `hitPairs`, `collectImpacts`
  appending). Impacts come from the fork's contact impulse events (`getContactImpulseEvents`). A `b3ContactHitEvent`
  has the approach speed but no impulse, so with Impact Min Speed on a record is kept only when its body pair also had
  a hit event (`updateHitEvents`, `hadHitEvent`). Wanted: events that accumulate until cleared, so the per-step
  merging goes.
* **Live contact recycling flag.** `b3Body_EnableContactRecycling` only reaches contacts made afterwards (`contact.c`
  sets the flag at creation), so `setContactRecyclingByHandle` disables and enables the body (`rebuildContacts`).
  That wakes the touching bodies and zeroes the body's velocity, which the wrapper saves and restores. Wanted: the
  setter updates existing contacts. `setSurfaceByHandle` rebuilds after a rolling resistance or tangent velocity
  change too, but `b3UpdateConvexContact` re-reads both, so that rebuild may be unneeded (unchecked).
* **Generic joint.** No 6-DOF joint with a lock, limit, spring and motor per axis, so SideFX Hard, Soft, Spring and
  Cone Twist are approximated by weld, distance, motor and spherical joints (`Box3DSideFX.cpp`,
  [joints-sidefx.md](joints-sidefx.md)): Hard's rotation is a motor joint with a stiff angular spring because Box3D
  has no rigid rotation-only joint, and Cone Twist loses its translation ranges and motors. One joint with per-axis
  settings would remove the mapping and the unmapped list above.
* **Anchor DOF attributes.** RBD Bullet Solver reads two point attributes of a constraint network's anchors, `condof`
  (an int: the number of DOFs the anchor constrains, 3 a point, 2 a line, 1 a plane, 0 none) and `condir` (a vector: the
  line's direction or the plane's normal), and Box3D ignores both (every case below is a plain point joint). On a Hard
  `position` line with the attribute on both points, a box hung from a pivot 0.5 from its centre: `condof` 2 with `condir`
  y falls straight (a line along y), with x slides along x (y and z held) and with z hangs; 1 with y slides in the plane
  (x to 27 in 2 s, y held), with x falls; 0 falls. Only the second anchor's counts when the first is a point (first 2,
  second 3 hangs as with none; first 3, second 2 slides). With one point carrying `condof` and the other not the
  missing one reads 0, so the joint is free. `all` with `condof` 2 on `condir` z swings (the rotation about z freed) and
  the `rotation` type holds nothing in either: not worked out. No Box3D joint is a line or a plane (a prismatic one also
  locks the turn, a point joint turns freely), so this needs the generic joint above; `condof` 3 and a hinge about an axis
  (revolute) are already joints. Probes only, no test.
* **Joints to the world.** `b3CreateJoint` needs two valid, different bodies, so a null `BodyHandle` uses
  `Impl::anchor`, a shapeless static body at the origin made on first use. Wanted: a null body means the world.
* **Checks that survive Release.** Box3D checks most inputs with asserts, which a Release build compiles out, so every
  entry point cleans its input first: mesh vertices and indices, height field size and scale, hull points,
  quaternions (`sanitizeQuat`), negative material values, rolling resistance, tangent velocity. Wanted: validation
  that clamps or fails in every build.
* **Mixing modes in the engine.** Friction and restitution mixing are context-free callbacks, so each rule is its own
  function (`frictionRule`, `restitutionRule`) called through a pointer per contact. An enum on `b3WorldDef` would
  avoid the call.
* **Wheel angular separation.** `b3Joint_GetAngularSeparation` is a todo that asserts for wheel joints, so
  `jointFeedback` returns 0.
