# Packed objects and constraints

[Back to the README](../README.md)

- RBD Packed Object also works, but writes its own `mass`, `density`, `computemass`, `friction` and `bounce` onto every point, which win over Box3D Data.
- A Box3D Packed Object writes the settings you change on it as point attributes when its object is created. To change them mid-simulation, edit the attributes (with a SOP Solver or POP Wrangle).
- A Box3D Packed Object is for the Box3D Solver only. In RBD Bullet Solver its pieces don't move, so use RBD Packed Object there.
- Its Initial State tab sets a starting velocity and spin. Inherit Velocity (off by default, always on inside the Box3D Solver SOP) keeps the points' own `v` and `w` instead.
- Turn on Deforming when the pieces change shape over time. The shapes are rebuilt every step.
- Geometry Source can take the pieces from the DOP network's first to fourth input instead of the SOP Path.
- Set its Display Geometry to Render Geometry to show a hi-res version (Render SOP Path) moving with the simulated pieces, matched by `name`.
- As with RBD Bullet Solver, these objects need packed geometry. An object of unpacked polygons is skipped with the warning "Geometry does not contain any primitive types supported by the solver". Pack it first, or use the Box3D Solver SOP, which packs its inputs. Unlike with Bullet, loose points (no primitives) do simulate, for example `pscale` points.
- SideFX's Glue, Hard, Spring, Bullet Soft and Cone Twist Constraint Relationships also work, approximately.
