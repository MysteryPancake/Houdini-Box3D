# Houdini Box3D

Houdini DOP wrapper for [Box3D](https://github.com/erincatto/box3d), currently one of the fastest rigid body solvers.

My goal with this project is to maintain Box3D's great performance inside Houdini.

I made design decisions based on the RBD Bullet Solver where possible, so it should feel familiar to use.

> [!CAUTION]
> This is heavily AI generated, as it was made for fun! This isn't normally how I develop software.
>
> However, it's not complete slop. I regularly test all the features in Houdini.
>
> I've tried my best to keep the code organized, clean and bug free.

## Building

You need CMake, a C++17 compiler, git and a Houdini install (for the HDK). Clone with submodules so Box3D is included:

```
git clone --recurse-submodules https://github.com/MysteryPancake/Houdini-Box3D
```

**Windows:** open **Houdini X.Y.Z > Command Line Tools** from the Start menu, then run `build.bat` (or `cmake -S . -B build -G "Visual Studio 17 2022" -A x64`, then `cmake --build build --config Release`). The DLL is copied to `Documents\houdiniX.Y\dso`.

**Linux:** install `g++ cmake git mesa-common-dev libglu1-mesa-dev libxi-dev`, then:

```
source /opt/hfsX.Y.ZZZ/houdini_setup
./build.sh
```

The plugin is copied to `~/houdiniX.Y/dso`.

Always build **Release**, never Debug, and close Houdini before rebuilding. To share the plugin, run `cpack` in the build folder and extract the zip into your `houdiniX.Y/packages` folder. It only loads in the Houdini version it was built for.

## Quick start

In a DOP network, press **TAB** and add a **Box3D Solver**, then wire your object into it like any other DOP solver. Add a **Box3D Data** node to give the object default material and shape settings.

Like Houdini's RBD solvers, it works on **points**: every point in the object's geometry is one rigid body, and the point is its **centre of mass**. For packed pieces, put the pivot at the centroid (the Pack SOP and RBD tools do this by default).

## Nodes

- Box3D Solver
- Box3D Data
- Box3D Weld Constraint Relationship
- Box3D Spherical Constraint Relationship
- Box3D Revolute Constraint Relationship
- Box3D Prismatic Constraint Relationship
- Box3D Distance Constraint Relationship
- Box3D Motor Constraint Relationship
- Box3D Parallel Constraint Relationship
- Box3D Wheel Constraint Relationship
- Box3D Filter Constraint Relationship

## Point attributes

All attributes are optional except `P`. A missing one uses the Box3D Data value. Every step the solver writes `P`, `orient`, `v` and `w` back to the dynamic points.

| Attribute | Type | Meaning |
|---|---|---|
| `P` | vector | Position (required) |
| `orient` | quaternion | Rotation |
| `v` / `w` | vector | Linear / angular velocity (radians/s) |
| `id` | int | Stable point id. Use it if points can be deleted or reordered |
| `dead` | int | Nonzero = delete the point and its body |
| `age` / `life` | float | With Enable Aging: the point is marked `dead` once `age` passes `life` |
| `active` | int | `0` = static, anything else = dynamic |
| `animated` | int | With `active` = 0: the body follows its animated point and pushes what it touches |
| `enabled` | int | `0` = leave the body out of the simulation |
| `awake` | int | `0` = asleep. Sets the starting state, then the solver keeps it up to date |
| `mass`, `computemass`, `density` | float / int | Mass, or `computemass` = 1 to use `density` × shape volume |
| `inertialtensorstiffness` | float | Scales how hard the body is to spin |
| `friction`, `bounce` | float | Grip and bounciness |
| `rollingresistance` | float | Resistance to rolling, 0 to 1 |
| `tangentvelocity` | vector | Conveyor belt velocity, in the body's own axes |
| `lineardamping`, `angulardamping` | float | Drag on movement and spin |
| `gravityscale` | float | Multiplies gravity. Negative floats upward |
| `enablesleep` | int | `0` = this body never sleeps |
| `sleepthreshold` | float | Speed below which the body may fall asleep |
| `ccdthreshold` | float | Fraction of the body's thinnest size it can move per step before being swept (default 0.5) |
| `fastobject` | int | Nonzero = can't pass through other moving bodies |
| `fastrotation` | int | Nonzero = no spin speed limit. Only for round pieces like wheels and balls |
| `contactrecycling` | int | `0` = never recycle this body's contacts |
| `lock_linear`, `lock_angular` | vector | Per world axis, nonzero = locked. `{0,0,1}` keeps a body at its starting Z |
| `collision_category`, `collision_mask`, `collision_group` | int | Collision filtering (see below) |
| `bullet_georep` | string | Collision shape (see [Shapes](#shapes)) |
| `bullet_autofit`, `bullet_primS`, `bullet_radius`, `bullet_length`, `bullet_primT`, `bullet_primR` | various | Hand-set box, sphere or capsule (see [Shapes](#shapes)) |
| `pscale` / `scale` | float / vector | Size of the shape |
| `deforming` | int | Nonzero = rebuild the collision shape every step |
| `bullet_autofit_valid` | int | `0` = rebuild the collision shape once |

Most attributes can be changed mid-simulation (a Geometry Wrangle DOP, a SOP Solver, or animating Box3D Data) and take effect on the next step. Collision shapes are kept once built, so to rebuild one set `bullet_autofit_valid = 0` or `deforming = 1`.

### Collision filtering

Two bodies in the same non-zero `collision_group` always collide if it's positive and never if it's negative. Otherwise they collide only when each one's `collision_mask` shares a bit with the other's `collision_category`.

- Pieces that never hit each other: `i@collision_group = -1`.
- Debris that ignores other debris but hits everything else: `i@collision_category = 2; i@collision_mask = ~2;`.

`i@` attributes give 31 usable layers; use 64-bit attributes for all 64. Static Object colliders use every bit, so a body hits them unless its mask is `0`.

## Shapes

Set `bullet_georep` to the same lowercase names Houdini's RBD tools use:

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule` | A simple shape fitted to the packed geometry, or sized from `pscale` / `scale` |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Several convex hulls on one body, one per `name` piece of the packed geometry |
| `none` | A static body with no collision |

Anything else (including `cylinder` and `plane`) uses the Default Shape. Convex and concave shapes need **packed primitives**, one point plus one packed primitive per body (for example from the Pack SOP or RBD Packed Objects). Pack a simplified proxy for best performance.

To set a box, sphere or capsule by hand, turn Auto Fit off (or set `bullet_autofit = 0`) and use `bullet_primS` (box half extents), `bullet_radius`, `bullet_length` (capsule, along Y), `bullet_primT` (offset) and `bullet_primR` (rotation, Euler degrees).

## Conveyor belts and rolling resistance

- Set **Conveyor Velocity** (or `tangentvelocity`) on a static body, or on a Static Object, to carry whatever sits on it. It follows the belt's own axes, so a turned belt carries in the turned direction. The belt needs friction above 0 to grip.
- You can animate it. Changing it wakes whatever touches the belt, so resting pieces pick up the new speed straight away.
- **Rolling Resistance** slows round bodies rolling on a surface. When two shapes touch, the larger value is used.

## Static Objects

A **Static Object** DOP collides with solved objects when a collision relationship links them, as with Bullet. Only its polygons are used (no SDF or height fields). Meshes are one-sided: if bodies fall through, turn on **Flip Static Mesh Winding** on a Box3D Data node attached to it. An animated Static Object doesn't push bodies; for something that pushes, use a kinematic point (`active = 0`, `animated = 1`).

## Joints

Joints use Houdini's **Constraint Network** DOP. Wire Box3D Constraint Relationship nodes into its second input, as with Bullet. Each two-point line in the network geometry is one joint:

- `constraint_name` primitive attribute: the Data Name of a Box3D Constraint Relationship (by default its type: `weld`, `revolute`, ...). It sets the joint's type and settings.
- Each end's `name` picks a piece (empty means the world).
- Each end's `P` and `orient` are that piece's joint frame, so put both ends at the pivot for a hinge or weld.
- Primitive attributes named like the node's parameters (`lower_angle`, `stiffness`, ...) modify them, as with Bullet: float attributes multiply the node's value, other attributes replace it.
- Joints past their Break Force or Break Torque go into the `broken` group, or switch to their `next_constraint_name`. Every step the solver writes `force`, `distance`, `torque` and `angle` (radians) onto each line.

## Things to know

- Bullet's collision padding and solver settings (ERP, CFM, iterations) are ignored. Use Substeps and the contact settings. `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read; use `enablesleep`, `awake` and `enabled`.
- Box3D's mover and pogo joints, explosions and wind aren't included. They are game-oriented, and Houdini already has Force DOPs for wind and explosions.
- Use `id` if you delete points with `dead`, otherwise every later point counts as a new body.
- Sleeping a body also sleeps the bodies touching it.
- Escape cancels a step.

## Troubleshooting

**"Box3D: world limit reached".** Each time the solver rebuilds (for example when you simulate on from a frame that wasn't the last one stepped) it makes a new Box3D world, and the old one stays alive while cached frames use it. Resetting the simulation or clearing the DOP cache frees them. Build with `-DBOX3DDOP_MAX_WORLDS=<n>` to change the limit (default 1024).

Debug logging is off by default; configure with `-DBOX3DDOP_DEBUG=ON` to print to Houdini's console.

## For developers

See [AGENTS.md](AGENTS.md) for a technical writeup of the codebase.

## License

MIT, see [LICENSE](LICENSE).
