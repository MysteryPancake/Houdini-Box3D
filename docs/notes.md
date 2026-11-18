# Things to know

[Back to the README](../README.md)

- Bullet's `collisiongroup` and `collisionignore` work on the Box3D Solver SOP, for up to 30 groups. Ignoring a single piece by name is not supported.
- Glue Inertial Load (Forces tab, Glue folder, off by default) breaks glue bonds farther from a hit too. Raise it if a glued object only breaks at the point it lands on.
- Bullet's collision padding and solver settings (ERP, CFM, iterations) are ignored. Use Substeps and the contact settings instead. Substeps 1 at 24 fps gives soft contacts.
- `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read. Use `enablesleep`, `awake` and `enabled` instead.
- Points sharing a `name` are separate bodies, as with Bullet.
- Sleeping a body also sleeps the bodies touching it.
- Solve On Creation Frame works as with Bullet: off, a new object starts moving one frame after it appears.
- As with Bullet, a Box3D Packed Object bakes `orient`, `pscale` and `scale` into the pieces when its object is created. The Box3D Solver SOP's World Scale simulates the scene scaled, for pieces far from 1 unit. Joint settings and PRESOLVE forces are scaled with it, so enter real values.
- Show Collision Shapes is in Guide Options on the solver, and Visualization on the Box3D Solver SOP.
- The Box3D Solver SOP's Ground Collision makes an infinite Ground Plane (Position, Rotation) or a Height Field ground (with Use Deforming Geometry). Friction, Bounce, Rolling Resistance and Conveyor Velocity are under Physical. Show Ground on the Visualization tab draws it.
- An unpacked collider that turns or scales during the simulation needs Collision Type Deforming on the Box3D Solver SOP. Animated keeps its first shape, as with Bullet.
- The Box3D Solver SOP's Constraints tab sets each constraint type's values. Initial State is on the Pieces tab.
- Break Thresholds on the Box3D Solver SOP's Constraints tab break constraints past an `angle`, `distance`, `impact` or `torque`, as in RBD Bullet Solver. Only SideFX Glue has an `impact`, and there is no Force Threshold.
- On the Box3D Solver SOP, Collide As Static Object makes the Collision input one static mesh, or a height field. It uses the Material settings, and Flip Mesh Winding if pieces fall through.
- Escape cancels a step in progress.
