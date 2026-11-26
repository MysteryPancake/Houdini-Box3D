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
* Box3D Configure has no Units (RBD Configure's Unit Mass and Unit Length, which scale Density by
  `unitlength / unitmass`, each over 1000).
* Quadric primitives (how they are sampled: [bodies.md](bodies.md)): a denser sphere mesh (Polygon Mesh, NURBS) rests
  at 0.46 to 0.48 where Bullet rests at 0.499, because a Box3D hull keeps about 44 vertices and loses the poles
  (keeping the farthest point per direction rested lower: a 38-vertex ball tips onto a face). Volumes: Bullet's packed
  one falls through, its RBD Object stays at 2, Box3D's falls.
* Objects solved by another solver: only RBD Bullet Solver's are tried (`other_solver`: packed ones are ignored as
  colliders, polygon ones push Box3D bodies one way). Vellum, FLIP and the rest are not.
* Height fields: holes from a mask layer, height fields on points.
* Constraint networks: agent anchors (`anchor_type` agent), the `anchor_pid` / `anchor_vid` matching. A missing
  `constraint_type` is `position` for Hard, Soft and Spring (`sidefx_hard_untyped`).
* SideFX constraint data not mapped: Soft plasticity, Cone Twist's elliptical and wide
  cones, softness, motors and translation ranges, Hard's angular motors.
* Force DOPs: parallel evaluation of circle / sphere sampled forces (the resolver has only the point call).
* Sticky collisions. Box3D's contact begin / end and sensor events: Houdini has no output for them (Bullet has none), so
  nothing reads them beyond Impacts. Sensors (trigger zones) would be the useful part.
* Box3D Solver SOP:
  * Break Thresholds (`b3d_solver_breaks.py`): `angle` matches Bullet's for a freely spinning piece, and `torque` is
    Bullet's quantity (the force's moment about the first piece's centre, `bullet_joint_feedback`), so the Torque
    Threshold breaks on the same frames (`solver_sop_break_angle_torque_bullet`, INFO). A Hard (all) line reads only
    part of the load when its two pieces touch (7.3 N for a hanging 1 kg piece, 6.9 N with the anchor at the fixed
    piece's centre): Box3D reads the joint's own force, and friction at their shared face carries the rest. With
    Disable Collisions on it reads the weight, and Bullet's reads the weight either way, being the pieces' net force
    (`bullet_hard_collisions`). The Angle and Torque thresholds of a Soft line on a spinning box break on the
    same frames in both with Rest Length 0, and Bullet's Soft Stiffness is the frequency in Hz that the mapping takes it
    for (`bullet_soft_spring_hz`). They still differ with the default Rest Length of 1 and the anchors at one point (0.2 rad:
    frame 13 in Box3D, 3 in Bullet): the spring has no direction while the anchors are exactly together, so Bullet
    applies none on the first step and Box3D's sprung Distance joint does (0.62 m of the rest length closed by frame 2
    against 0.00, at any Substeps), which holds the spinning box much tighter in the first frames (angle 0.05 against
    0.19). Both settle at the rest length and nothing moves with Rest Length 0 (`bullet_soft_coincident_anchors`). Matching
    it would mean leaving the spring out for the first step of a joint whose anchors coincide: not done, as it
    only reproduces a one-frame lag. Untested: Variance, World Scale other than 1, and the cost on a large network with it on (the SOP
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
* **Wide and elliptical cones.** `b3SphericalJoint_SetConeLimit` asserts the angle is at most 90°, and the cone is
  round. Cone Twist's wider and elliptical cones (unmapped above) need both lifted.

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
