# Packed objects and constraints

[Back to the README](../README.md)

- RBD Packed Object also works, but writes its own `mass`, `density`, `computemass`, `friction` and `bounce` onto every point, which win over Box3D Data (the solver warns when you set those on a Box3D Data).
- A Box3D Packed Object writes the settings you change on it as point attributes when its object is created; change them mid-simulation by editing the attributes (SOP Solver, POP Wrangle).
- As with RBD Bullet Solver, these objects need packed geometry: unpacked polygons give the error "Geometry does not contain any primitive types supported by the solver", so pack them first, or use the Box3D Solver SOP, which packs its inputs. Loose points (no primitives) still simulate, which RBD Bullet Solver does not (it gives the same error); this is a Box3D extension, e.g. for `pscale` points.
- SideFX's Glue, Hard, Spring, Bullet Soft and Cone Twist Constraint Relationships also work, approximately.
