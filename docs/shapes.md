# Shapes

[Back to the README](../README.md)

`bullet_georep` takes Houdini's RBD names:

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule` | Fitted to the packed geometry, or sized from `pscale` / `scale` |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Static piece: the exact polygons, as a triangle mesh. Moving piece: one convex hull per `name` piece |
| `none` | A static body with no collision |

Anything else (including `cylinder` and `plane`) uses Box3D Data's Default Shape. Convex and concave shapes need packed primitives. Pack a simplified proxy for speed. Triangle meshes are one-sided: polygons must face outwards, or turn on Flip Static Mesh Winding. To set a box, sphere or capsule by hand, turn Auto Fit off and use `bullet_primS` (half extents), `bullet_radius`, `bullet_length` (capsule, along Y), `bullet_primT` (offset) and `bullet_primR` (Euler degrees).
