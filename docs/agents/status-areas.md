# Status: unverified, by area

Unverified items for Bullet comparisons, whole objects and the digital assets. The general ones: [status.md](status.md).
What is still to do: [todo.md](todo.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Unverified

**Bullet semantics not compared**
* The extra `restRotation` split per packed point in every full scan (the rest convention matches Bullet:
  `bullet_rest_convention`, `bullet_rest_rescan`).
* Solve on Creation Frame on the SOP's collider (`solver_sop_solve_first_frame_bullet`): a box on a collider lifted
  0.5 a frame ends at 1.1 to 2.3 in Box3D (it clips into the collider: fork list) and at 1.0 in Bullet.
* The proxy output with no proxy input is a copy of the geometry in Box3D and empty in Bullet
  (`solver_sop_before_start_bullet`).
* With both anchors at one point (`bullet_joint_rest`, INFO) the centres end 2.19 and 2.59 apart, past the 2.06
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
* Unpacked geometry (`unpacked_geometry_bullet`, `unpacked_geometry_match`): an object of polygons only is skipped by
  both (ours warns). Bullet ignores loose polygons beside packed pieces without a message (ours warns), gives that
  message for loose points alone and ignores loose points beside a packed piece without one (ours simulates them).

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
* `anchor_id`: untested are other ids (a fragment's points are taken in source order, its vertices in primitive
  order), a piece that has moved before its joint is made (`nowToRest`), and non-fragment packed pieces.
* Glue's impulse model (`core/Box3DGlue.h`, `glue_break_match`): `impact` matches Bullet's. The Strength at which a
  bond stops breaking does not: Bullet breaks at about its `impact` (1.0x) when something hits a resting glued pair or
  row, and at 3x to 10x when the glued pieces land themselves (more with speed, bounce, piece count and a piece's
  height), where Box3D uses one factor, `kGlueStrengthScale` (5.3, measured for a landing from 3). So a hit on a
  resting structure needs about 5x the Strength in Box3D that it does in Bullet. Anchor position and rest length
  change neither. Not understood: what makes Bullet's break value larger than its `impact`. Unchecked: the cost of
  reading every glued body's contacts each step on a large network.
* What the precompiled headers (`BOX3DDOP_PCH`) save: not measured.
* Transform Pieces with packed transforms that carry scale or shear.
* Constraints output through Transform Pieces: constraints between collision pieces (not in the simulation points, so
  not moved).
* FORCES: the Multiple Solver with nothing wired into PRESOLVE / POSTSOLVE (no warning expected); that diving into
  the asset lands in FORCES (the build's check passes); FORCES editable in a locked instance (`EditableNodes`).
