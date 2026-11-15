# Packed objects and constraints

[Back to the README](../README.md)

- RBD Packed Object also works, but writes its own `mass`, `density`, `computemass`, `friction` and `bounce` onto every point, which win over Box3D Data.
- A Box3D Packed Object writes the settings you change on it as point attributes when its object is created. To change them mid-simulation, edit the attributes (with a SOP Solver or POP Wrangle).
- Its Initial State tab sets a starting velocity and spin. Inherit Velocity (on by default) keeps the points' own `v` and `w` instead.
- Turn on Deforming when the pieces change shape over time. The SOP is read every step and the shapes rebuilt.
- Set its Display Geometry to Render Geometry to show a hi-res version (Render SOP Path) moving with the simulated pieces, matched by `name`.
- As with RBD Bullet Solver, these objects need packed geometry. An object of unpacked polygons is skipped with the warning "Geometry does not contain any primitive types supported by the solver". Pack it first, or use the Box3D Solver SOP, which packs its inputs. Unlike with Bullet, loose points (no primitives) do simulate, for example `pscale` points.
- SideFX's Glue, Hard, Spring, Bullet Soft and Cone Twist Constraint Relationships also work, approximately.
