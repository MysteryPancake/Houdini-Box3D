# Things to know

[Back to the README](../README.md)

- Bullet's collision padding and solver settings (ERP, CFM, iterations) are ignored; use Substeps and the contact settings. Substeps 1 at 24 fps gives soft contacts.
- `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read; use `enablesleep`, `awake` and `enabled`.
- Points sharing a `name` are separate bodies, as with Bullet.
- Sleeping a body also sleeps the bodies touching it.
- Solve On Creation Frame works as with Bullet: off, a new object starts moving one frame after it appears.
- As with Bullet, a Box3D Packed Object resets each piece's rest (`restxform`) when its object is created and bakes `orient`, `pscale` and `scale` into the packed pieces. The Box3D Solver SOP's World Scale simulates the scene scaled, for pieces far from 1 unit.
- Show Collision Shapes draws the bodies' collision shapes (Guide Options on the solver, Visualization on the Box3D Solver SOP, which also draws the external collision geometry by default).
- An unpacked collider that turns or scales during the simulation needs Collision Type Deforming on the Box3D Solver SOP; Animated keeps its first shape, as with Bullet.
- Escape cancels a step.
