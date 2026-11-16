# Shapes

[Back to the README](../README.md)

`bullet_georep` takes Houdini's RBD names:

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule`, `cylinder` | Fitted to the packed geometry, or sized from `pscale` / `scale`. Capsules and cylinders run along Y. A cylinder has 32 sides |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Static piece: the exact polygons, as a triangle mesh. Moving piece: one convex hull per `name` piece |
| `plane` | Static piece: an infinite plane through its origin, facing its +Y. Moving piece: a box |
| `none` | A static body with no collision |

A Sphere primitive (Sphere SOP at Primitive Type Primitive, one point and a transform) that is the only primitive of its piece and uniformly scaled becomes a real Box3D sphere of its radius, whatever the shape type (an ellipsoid or a sphere mixed with other primitives is sampled and hulled). Tube, Circle and Metaball primitives are sampled on their surface and treated like those points. Their sizes follow the RBD Bullet Solver's (a Metaball is half its radius, a Circle is given a thickness of 0.04). Volumes are not shapes.

Anything else uses Box3D Data's Default Shape. Convex and concave shapes need packed primitives. Pack a simplified proxy for speed. Triangle meshes are one-sided: polygons must face outwards, or turn on Flip Static Mesh Winding. To set a box, sphere or capsule by hand, turn Auto Fit off and use `bullet_primS` (half extents), `bullet_radius`, `bullet_length` (capsule, along Y), `bullet_primT` (offset) and `bullet_primR` (Euler degrees).
