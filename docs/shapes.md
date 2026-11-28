# Shapes

[Back to the README](../README.md)

`bullet_georep` takes Houdini's RBD names:

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule`, `cylinder` | Fitted to the packed geometry, or sized from `pscale` / `scale`. Capsules and cylinders run along Y |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Static piece: the exact polygons, as a triangle mesh. Moving piece: one convex hull per `name` piece, or per set of connected primitives when Hull per Connected is on |
| `plane` | Static piece: a very wide, deep ground box whose top is the plane through its origin, facing its +Y (sized by the solver's Box Size). Moving piece: a box |
| `none` | A static body with no collision |

A Sphere primitive (Sphere SOP at Primitive Type Primitive) that is the only primitive of its piece and uniformly scaled is a true sphere, whatever the shape type. Tube, Circle and Metaball primitives are treated like points on their surface, sized as in RBD Bullet Solver. Volumes are not shapes.

The pieces of a moving `concave` body can each have their own shape and density, set as primitive attributes on the packed geometry. `piece_georep` (`box`, `sphere`, `capsule` or `cylinder`) fits that piece as the shape, with capsules and cylinders along the piece. `piece_densityscale` multiplies the body's density for that piece. A piece without them is a convex hull at the body's density.

Anything else uses Box3D Data's Default Shape. Convex and concave shapes need packed primitives. Pack a simplified proxy for speed. Triangle meshes are one-sided: polygons must face outwards. To set a box, sphere or capsule by hand, turn Auto Fit off and use Box3D Data's Box Size, Radius, Length, Offset and Rotation, or the attributes `bullet_primS` (half extents), `bullet_radius`, `bullet_length` (capsule, along Y), `bullet_primT` (offset) and `bullet_primR` (Euler degrees).
