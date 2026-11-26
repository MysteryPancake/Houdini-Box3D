# Status: unverified, by area

Unverified items for Bullet comparisons, whole objects and the digital assets. The general ones: [status.md](status.md).
What is still to do: [todo.md](todo.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Unverified

**Bullet semantics not compared**
* The extra `restRotation` split per packed point in every full scan (the rest convention matches Bullet:
  `bullet_rest_convention`, `bullet_rest_rescan`).
* Solve on Creation Frame on the SOP's collider (`solver_sop_solve_first_frame_bullet`): a box on a collider lifted
  0.5 a frame ends at 1.1 to 2.3 in Box3D (it clips into the collider: fork list) and at 1.0 in Bullet.
* With both anchors at one point (`bullet_joint_rest`, INFO) the centres end 2.55 and 2.59 apart, past the 2.06
  possible.
* Hand-set shapes: `bullet_primS`, `bullet_primR` (XYZ Euler degrees), `bullet_radius` and `bullet_length` match
  (`bullet_shape_attributes`, `bullet_primr_order`). Kept differences (`bullet_shape_position`, `bullet_shape_probe`):
  Bullet drops a dynamic hand-set piece's x and z (to 0 plus `bullet_primT`'s), takes the height from `bullet_primT`'s
  y and finds a static piece away from the origin at the origin, where Box3D keeps the shape on the piece with
  `bullet_primT` as an offset (copying Bullet would move every hand-set piece to the world origin). Bullet's dynamic
  box rests lower by the shape's vertical half extent as first turned. Not understood: with `bullet_primR` 30 0 60 the
  piece fell through the ground in Bullet (`bullet_primr_through`: other angles end 0.14 to 0.47 below it, Box3D
  always at 0.10). A tilted box's top is a corner the plate test stops on roughly, so that case is not compared. Not
  compared: `deforming`, `bullet_autofit_valid`, AutoFit on whole objects and fractured pieces.
* Impacts: a box landing and resting for 3 s has Box3D records on 13 frames (at most 8 on one), Bullet's on 49 (at
  most 40) because Bullet keeps writing contacts at rest (`bullet_impact_fresh`). Whether a fresh `SIM_Impacts` each
  step suits Houdini's impact tools is open.
* Whole RBD Object pivot (`wholeObjectPose`, tests `hull_centroid`, `bullet_rbd_pivot`, `bullet_rbd_pivot_far`):
  Bullet's `p` is 0.03 off the hull centroid Box3D gives for boxes (an L: (0.205, 0.795) against (0.238, 0.762); the
  same moved 5 in x, and for a T), unexplained. A concave object with Hull per Piece is not checked against Bullet.
* Which end is Cone Twist's goal (taken as point 1; `bullet_cone_twist_goal`, `bullet_cone_twist_goal_apart`):
  Bullet's piece fell freely in both, so it cannot say.
* Unpacked geometry (`unpacked_geometry_bullet`, `unpacked_geometry_match`): kept differences, as the HDK's
  `SIM_BulletState` shows. Bullet reads only packed primitives and warns ("does not contain any primitive types
  supported by the solver") only when the geometry has none, so an object of polygons only, or of loose points only,
  gets the message in both, and loose polygons or points beside a packed piece get no message and are ignored. Ours
  warns about the loose polygons and simulates the loose points.

**Whole objects**
* An RBD Object without Box3D Data colliding as its convex hull; `bullet_primT` / `bullet_primS` meaning there.
* RBD State's Compute Inertial Tensor off is assumed to mean the computed inertia (`rbd_object_inertia_type`).

**Box3D Solver SOP** (`tools/make_solver_sop.py`, `tests/houdini/checks_sop.py`)
* Collision Type Animated / Deforming: whether the Packed Object's Overwrite Attributes from SOP (SOP Geometry's Set
  Always) re-imports every step (the build prints which menus it drives), and the solver's cost of a full rescan of
  the collider each step.
* The Solver SOP's Collide As Static Object, Constraints tab, help card and Set Always settings: unchecked are the
  Static Object's default Collision Detection on a mesh (distance field cost), the help card's look in Houdini's help
  browser (its wiki markup parses with `bookish.wiki.wikipages.parse_to_root` in hython), and whether every setting
  tolerates being re-applied each step.
* The Solver SOP's Override Attributes from SOP (the pieces' `overwritefromsop`) is checked for its link, not for
  changed attributes reaching the solver.
* `anchor_id`: matches Bullet for ids 0 to 7 and vertex ids up to 23, on Assemble's fragments and the Pack SOP's own
  geometry, with a piece turned before the start (`bullet_anchor_id_cases`). A joint made after its piece fell or spun
  (the network's activation) acts as Bullet's: Hard pulls the piece back to its rest anchors and Glue holds it where it
  is (`bullet_late_joint`), also Glue with an `anchor_id` (`bullet_late_joint_anchor_id`). Not understood: Hard (all) with an
  `anchor_id` on a free box, made on frame 1 or later, settles at (1.25, 4.25) in Bullet and (1.5, 4.5) in Box3D (ids 0
  and 3 against an anchor at the shared face), where P anchors agree.
* Glue's break estimate (`core/Box3DGlueEstimate.h`, `Box3DGlueEstimate.cpp`, tests `glue_break_match`,
  `glue_break_substeps`, `glue_fracture_vs_bullet`): Bullet breaks glue before its step on an impulse estimate at every contact point of the
  glued body ([joints-sidefx.md](joints-sidefx.md)), the HDK's `SIM_BulletState` (`preBreakGlue`, `updateGlue`).
  Box3D's, from the step's contacts, holds from the same Strength as Bullet's in the landing and dropped-box scenes
  (within about 15%, a row of 4 hit by a box 1.3x). Not matched: Bullet's trial moves the body a whole step
  forward, so an edge landing sinks into the ground and more pieces have points than Box3D's contacts after the step: a
  fractured block holds 4 to 7 times better here (`glue_fracture_vs_bullet`). Only contacts above Box3D's hit
  threshold (`impactMinSpeed`, else Box3D's default) are estimated; slower ones count by their real impulse, as a
  resting body's weight does. A kinematic piece in a group, or one whose mass is not finite, leaves its group out of the
  estimate. The hold that frees pieces for a fast body (`kGlueHoldBreakScale`) is a heuristic of its own and frees pieces
  at Strengths where the estimate would not break the bond (a row of 4 hit by a box). Cost (`wall` of 4000 bricks,
  `bench.py`): the estimate adds 0.4 to 0.8 ms to a 11 to 14 ms frame while the wall is hit or falling apart (one glued
  block, or groups of 5), and nothing when no contact is fast. Substeps 4 and 8 (`glue_break_substeps`): Bullet makes
  one estimate a frame whatever the Substeps, and Box3D's first estimate is the same at 2 and 4 (a box landing on a row:
  about 5.4k a contact point); the landing and dropped-box scenes hold from 0.9x to 1.1x of Bullet's Strength. Not
  matched: a box dropped 4 m on a row of 4 holds from 1.2x (Substeps 8) to 1.7x (4) of Bullet's. The peak impacts agree
  and the first estimates are the same, so it comes from the later contacts at 1 to 4 m/s that follow the hit (a
  second estimate on a bond that still carries the first's impact); not traced further.
* What the precompiled headers (`BOX3DDOP_PCH`) save: not measured.
* Transform Pieces with packed transforms that carry scale or shear.
* Constraints output through Transform Pieces: constraints between collision pieces (not in the simulation points, so
  not moved).
* FORCES: the Multiple Solver with nothing wired into PRESOLVE / POSTSOLVE (no warning expected); that diving into
  the asset lands in FORCES (the build's check passes); FORCES editable in a locked instance (`EditableNodes`).
