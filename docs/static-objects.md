# Static Objects

[Back to the README](../README.md)

A **Static Object** collides with solved objects through a collision relationship, as with Bullet. Its geometry can be polygons or a height field (a HeightField SOP's `height` layer).

- **Moving it:** set its OBJ Path to follow an animated object (position and rotation only). Use Object Transform also passes scale and shear, but only once.
- **Editing it mid-simulation:** turn on Use Deforming Geometry.
- **Pushing things:** an animated polygon Static Object (or an RBD Object with Create Active Object off) pushes what it touches. Height fields and Ground Planes don't push.
- **Ground Plane:** a Ground Plane DOP is a very wide, deep box whose top is at its Position and Rotation, facing up. Its size is the solver's Box Size (x, depth, z, default 1000, 10, 1000). Nothing collides past its edge. Use several for slopes and troughs. Friction and Bounce come from its Physical tab, or from a Box3D Data.
- **Conveyor belts:** Conveyor Velocity on a Box3D Data attached to it carries what sits on it (needs friction).
