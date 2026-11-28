# Drag and speed limits

The POP-style drag, speed limit and acceleration limit point attributes RBD Bullet Solver reads, and how a Glue as One
Body compound takes them. The other per-attribute features: [body-features.md](body-features.md). Index and rules:
[CLAUDE.md](../../CLAUDE.md).

* **Drag and speed limits** (`airresist`, `spinresist`, `drag`, `dragexp`, `targetv`, `targetw`, `speedmin`,
  `speedmax`, `spinmin`, `spinmax`; `Box3DMotion.cpp`, maths in `core/Box3DMotion.h`): what RBD Bullet Solver reads from
  POP-style attributes (`SIM_BulletRigidBody::applyDrag` / `clampSpeed`). `refreshMotion` reads them once a frame, when
  their data ids or the point count change, into `Box3DObjectSync::motion` (the points with any, by offset; not
  prop markers, since a POP node may write them every step). Drag goes in `applyToObjectBodies` (`addDrag`), last, on
  the sum of the Force DOPs and point forces, so it needs Use Force DOPs: applied implicitly as Bullet does by default
  (the velocity the body would have after the step, with gravity, is slowed by `applyImplicitDrag`, and the force that
  gives the change is the drag; spin works on the world inertia, `Box3DWorld::inertiaWorldByHandle`). Every Box3D step
  of a frame applies it, so the result depends on Substeps as Bullet's does on its own. `airresist`, `spinresist`,
  `targetv` and `targetw` are inputs for one step: `clearMotionInputs` zeroes them after the frame, as Bullet's
  `clearForces`. The limits clamp the move events after every Box3D step (`applySpeedLimits`) and the body, so a body
  gains up to a step's gravity over its limit within the step. RBD Objects get none, as in Bullet. A Glue as One Body
  compound takes its pieces' combined, as Bullet's glue cluster (`refreshCompoundMotion`, each frame: drag, exponent
  and targets weighted by mass, the largest minimum, the smallest maximum), its drag applied after the force pass on
  the forces its pieces put on it (`applyCompoundDrag`), its limits on its body's move events; its pieces take none of
  their own. `accelmax` / `angaccelmax` limit the change a step's constraints make (`applyAccelLimits`, after the step,
  before the speed limits; `clampConstraintChange`): Box3D has no separate constraint change, so it is the velocity
  after the step less what the one before it (`beginAccelLimits`) would be with only gravity, the forces the force
  pass put on the body (`noteAccelForce`) and Box3D's damping over the step's solver substeps (`undisturbedVelocity`),
  shortened to the limit times the step; only the velocity carried on changes, as Bullet's. Joints are limited as
  contacts are, and a compound takes its pieces' smallest limit, as Bullet's chunk. The Solver SOP's World Scale
  scales the input points' linear limits and `targetv` ([solver-sop-io.md](solver-sop-io.md)). Tests
  `bullet_drag_limits`, `bullet_glue_drag_limits`, `bullet_accel_limits`, `solver_sop_speed_limit_bullet`,
  `solver_sop_pop_drag_bullet`, `solver_sop_world_scale_speed_limit`, `core_motion`.
