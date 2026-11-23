# Point attributes

[Back to the README](../README.md)

All are optional except `P`. A missing one uses the Box3D Data value. Every step the solver writes `P`, `orient`, `v` and `w` back to the moving points. Most can change mid-simulation, but collision shapes are kept once built.

| Attribute | Type | Meaning |
|---|---|---|
| `P` | vector | Position (required) |
| `orient` | quaternion | Rotation. On packed pieces, the rotation since the start, as Bullet |
| `restxform`, `pivot` | matrix / vector | Packed pieces: the transform and position at the start. Set for you |
| `v` / `w` | vector | Linear / angular velocity (radians/s) |
| `id` | int | Stable point id. Use it if points can be deleted or reordered |
| `active` | int | `0` = static |
| `animated` | int | With `active = 0`: follows its animation and pushes what it touches |
| `enabled` | int | `0` = left out of the simulation |
| `awake` | int | `0` = asleep. Sets the starting state, and is updated as the body sleeps |
| `dead` | int | Nonzero = remove the body. With Reap Dead Points (on by default), the point is deleted too |
| `age` / `life` | float | With Enable Aging: `dead` once `age` passes `life`, so a `life` of 0 dies on the first step. No `life` attribute never dies |
| `active_age` | float | Solver SOP with Add Active Age Attribute: the time the piece has been active, 0 while it is not |
| `mass`, `computemass`, `density` | float / int | Mass, or `computemass` = 1 for `density` × volume |
| `computecom` | int | 1 spins a piece about its shape's centre of mass, 0 about its point |
| `inertialtensorstiffness` | float | Scales how hard the body is to spin |
| `friction`, `bounce` | float | Grip and bounciness |
| `rollingresistance` | float | Resistance to rolling, 0 to 1 |
| `tangentvelocity` | vector | Conveyor belt velocity, in the body's own axes |
| `lineardamping`, `angulardamping` | float | Drag on movement and spin |
| `gravityscale` | float | Multiplies gravity. Negative floats upward |
| `enablesleep` | int | `0` = never sleeps |
| `sleepthreshold` | float | Speed below which the body may sleep |
| `ccdthreshold` | float | How far it may move in one step, as a fraction of its thinnest size, before fast-motion collision is used |
| `fastobject` | int | Nonzero = can't pass through other moving bodies |
| `fastrotation` | int | Nonzero = no spin speed limit (round pieces only) |
| `contactrecycling` | int | `0` = never recycle this body's contacts |
| `collision_category`, `collision_mask`, `collision_group` | int | Collision filtering (below) |
| `bullet_georep` | string | Collision shape (see [Shapes](shapes.md)) |
| `bullet_autofit`, `bullet_primS`, `bullet_radius`, `bullet_length`, `bullet_primT`, `bullet_primR` | various | Hand-set box, sphere or capsule |
| `pscale` / `scale` | float / vector | Size of the shape on plain points. On packed pieces, a Box3D Packed Object bakes it into the piece |
| `deforming` | int | Nonzero = rebuild the collision shape every step |
| `bullet_autofit_valid` | int | `0` = rebuild the collision shape once |
| `force` / `torque` | vector | Applied for one step, then cleared |

**Collision filtering.** Two bodies in the same nonzero `collision_group` always collide if it's positive, never if it's negative. Otherwise they collide when each one's `collision_mask` shares a bit with the other's `collision_category`. For example `i@collision_group = -1` for pieces that never hit each other, or `i@collision_category = 2` with `i@collision_mask = ~2` for debris that ignores other debris. Static Object colliders use every bit.
