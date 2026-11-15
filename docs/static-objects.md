# Static Objects

[Back to the README](../README.md)

A **Static Object** collides with solved objects through a collision relationship, as with Bullet. Its polygons become a triangle mesh; a height field (a HeightField SOP's `height` layer) becomes a Box3D height field instead.

- **Moving it:** set its OBJ Path to follow an animated object (position and rotation only). Use Object Transform also passes scale and shear, but only once.
- **Editing it mid-simulation:** turn on Use Deforming Geometry.
- **Pushing things:** an animated Static Object doesn't push bodies; use a kinematic point (`active = 0`, `animated = 1`).
- **Conveyor belts:** Conveyor Velocity on a Box3D Data attached to it carries what sits on it (needs friction).
