# Houdini Box3D

Houdini DOP wrapper for [Box3D](https://github.com/erincatto/box3d), currently one of the fastest rigid body solvers.

My goal is keeping Box3D's great performance preserved in Houdini as much as possible.

I made design decisions based on the RBD Bullet Solver where possible, so it should feel familiar to use.

> [!CAUTION]
> This is heavily AI generated, as it was made for fun!
>
> However, it's not complete slop. I've tested all the features and cleaned up as much as possible.
>
> This isn't normally how I develop software, I just wanted to give it a try.

## Installing

A prebuilt Windows build is available in the [Releases](../../releases) section. To install it, put it in your Houdini packages folder:

```
Documents\houdini22.0\packages
```

It only loads in the Houdini version it was built for. For other versions or Linux, see [Building](#building).

## Building

You need CMake, a C++17 compiler, git and a Houdini install (for the HDK). Clone with submodules so Box3D is included:

```
git clone --recurse-submodules https://github.com/MysteryPancake/Houdini-Box3D
```

**Windows:** open **Houdini X.Y.Z > Command Line Tools** from the Start menu, then run `build.bat` (or `cmake -S . -B build -G "Visual Studio 17 2022" -A x64`, then `cmake --build build --config Release`). The DLL is copied to `Documents\houdiniX.Y\dso`.

**Linux:** install `g++ cmake git mesa-common-dev libglu1-mesa-dev libxi-dev` (Red Hat: `gcc-c++ cmake git mesa-libGL-devel mesa-libGLU-devel libXi-devel`), ideally the gcc version named in your Houdini download (`...-gccX.Y.tar.gz`), then:

```
source /opt/hfsX.Y.ZZZ/houdini_setup
./build.sh
```

The plugin is copied to `~/houdiniX.Y/dso`.

Always build **Release**, never Debug, and close Houdini before rebuilding.

### Packaging

To share the plugin, build it, then make a zip in the build folder:

```
cd build
cpack -G ZIP -C Release
```

(`-C Release` is needed on Windows; on Linux plain `cpack -G ZIP` works.) This makes `Box3D-Houdini-<version>-<system>.zip`. Extract it into your `houdiniX.Y/packages` folder so `Box3D.json` sits directly inside it. It only loads in the Houdini version it was built for.

The zip includes the Box3D Packed Object and Box3D Solver SOP assets only if hython was found during the build. If you test the package on the machine you built on, first remove the copies the build put in your `houdiniX.Y/dso` and `otls` folders.

## Quick start

In a DOP network, add a **Box3D Solver** and wire your objects into it like any DOP solver. For packed pieces, use a **Box3D Packed Object**; its settings (shape, mass, friction, bounce, ...) apply to every piece that doesn't set its own attribute.

Like Houdini's RBD solvers, every point is one rigid body and the point is its **centre of mass** (for packed pieces, the pivot; the Pack SOP and RBD tools put it at the centroid). Objects from **RBD Object** and similar nodes are one body each instead, with their results in the object's Position data.

Example scenes: run `make_examples.bat` (Linux: `hython examples/make_examples.py`) and open `examples/scenes`.

## Nodes

| Node | Use |
|---|---|
| Box3D Solver | The solver |
| Box3D Packed Object | Packed geometry as a Box3D object, with Box3D Data's settings on the node |
| Box3D Solver (SOP) | Like RBD Bullet Solver: packed geometry, constraints, proxy and collision in, simulated out |
| Box3D Data | Per-object settings, for objects made by other nodes |
| Box3D Weld / Spherical / Revolute / Prismatic / Distance / Motor / Parallel / Wheel / Filter Constraint Relationship | Joint types, used with a Constraint Network |

RBD Packed Object also works, but writes its own `mass`, `density`, `computemass`, `friction` and `bounce` onto every point, which win over Box3D Data (the solver warns). A Box3D Data after a Box3D Packed Object only changes parameters set to Set Always. SideFX's Glue, Hard, Spring, Bullet Soft and Cone Twist Constraint Relationships also work, approximately.

## Point attributes

All are optional except `P`; a missing one uses the Box3D Data value. Every step the solver writes `P`, `orient`, `v` and `w` back to the dynamic points. Most can change mid-simulation; collision shapes are kept once built (set `bullet_autofit_valid = 0` or `deforming = 1` to rebuild).

| Attribute | Type | Meaning |
|---|---|---|
| `P` | vector | Position (required) |
| `orient` | quaternion | Rotation |
| `v` / `w` | vector | Linear / angular velocity (radians/s) |
| `id` | int | Stable point id. Use it if points can be deleted or reordered |
| `active` | int | `0` = static |
| `animated` | int | With `active = 0`: follows its animation and pushes what it touches |
| `enabled` | int | `0` = left out of the simulation |
| `awake` | int | `0` = asleep. Sets the starting state; the solver keeps it up to date |
| `dead` | int | Nonzero = remove the body; the point is deleted with Reap Dead Points on |
| `age` / `life` | float | With Enable Aging: `dead` once `age` passes `life` |
| `mass`, `computemass`, `density` | float / int | Mass, or `computemass` = 1 for `density` × volume |
| `inertialtensorstiffness` | float | Scales how hard the body is to spin |
| `friction`, `bounce` | float | Grip and bounciness |
| `rollingresistance` | float | Resistance to rolling, 0 to 1 |
| `tangentvelocity` | vector | Conveyor belt velocity, in the body's own axes |
| `lineardamping`, `angulardamping` | float | Drag on movement and spin |
| `gravityscale` | float | Multiplies gravity; negative floats upward |
| `enablesleep` | int | `0` = never sleeps |
| `sleepthreshold` | float | Speed below which the body may sleep |
| `ccdthreshold` | float | Fraction of its thinnest size it can move per step before being swept |
| `fastobject` | int | Nonzero = can't pass through other moving bodies |
| `fastrotation` | int | Nonzero = no spin speed limit (round pieces only) |
| `contactrecycling` | int | `0` = never recycle this body's contacts |
| `lock_linear`, `lock_angular` | vector | Per world axis, nonzero = locked |
| `collision_category`, `collision_mask`, `collision_group` | int | Collision filtering (below) |
| `bullet_georep` | string | Collision shape (see [Shapes](#shapes)) |
| `bullet_autofit`, `bullet_primS`, `bullet_radius`, `bullet_length`, `bullet_primT`, `bullet_primR` | various | Hand-set box, sphere or capsule |
| `pscale` / `scale` | float / vector | Size of the shape on plain points |
| `deforming` | int | Nonzero = rebuild the collision shape every step |
| `bullet_autofit_valid` | int | `0` = rebuild the collision shape once |
| `force` / `torque` | vector | Applied for one step, then cleared |

**Collision filtering.** Two bodies in the same nonzero `collision_group` always collide if it's positive, never if it's negative. Otherwise they collide when each one's `collision_mask` shares a bit with the other's `collision_category`. For example `i@collision_group = -1` for pieces that never hit each other, or `i@collision_category = 2; i@collision_mask = ~2;` for debris that ignores other debris. Static Object colliders use every bit.

## Shapes

`bullet_georep` takes Houdini's RBD names:

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule` | Fitted to the packed geometry, or sized from `pscale` / `scale` |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Static piece: the exact polygons, as a triangle mesh. Moving piece: one convex hull per `name` piece |
| `none` | A static body with no collision |

Anything else (including `cylinder` and `plane`) uses Box3D Data's Default Shape. Convex and concave shapes need packed primitives; pack a simplified proxy for speed. Triangle meshes are one-sided: polygons must face outwards, or turn on Flip Static Mesh Winding. To set a box, sphere or capsule by hand, turn Auto Fit off and use `bullet_primS` (half extents), `bullet_radius`, `bullet_length` (capsule, along Y), `bullet_primT` (offset) and `bullet_primR` (Euler degrees).

## Static Objects

A **Static Object** collides with solved objects through a collision relationship, as with Bullet. Its polygons become a triangle mesh; a height field (a HeightField SOP's `height` layer) becomes a Box3D height field instead.

- **Moving it:** set its OBJ Path to follow an animated object (position and rotation only). Use Object Transform also passes scale and shear, but only once.
- **Editing it mid-simulation:** turn on Use Deforming Geometry.
- **Pushing things:** an animated Static Object doesn't push bodies; use a kinematic point (`active = 0`, `animated = 1`).
- **Conveyor belts:** Conveyor Velocity on a Box3D Data attached to it carries what sits on it (needs friction).

## Joints

Joints use Houdini's **Constraint Network** DOP: wire Box3D Constraint Relationship nodes into its second input. Each two-point line is one joint:

- `constraint_name` names a relationship's Data Name (by default its type: `Weld`, `Revolute`, ...).
- Each end's `name` picks a piece (empty = the world); its `P` and `orient` are the joint frame.
- Primitive attributes named like the node's parameters modify them: floats multiply, others replace.
- With **Breakable** on, a joint past its Break Force or Torque goes into the `broken` group, or switches to its `next_constraint_name`.
- Each step the solver writes `force`, `distance`, `torque` and `angle` (radians) onto each line.

## Things to know

- Bullet's collision padding and solver settings (ERP, CFM, iterations) are ignored; use Substeps and the contact settings. Substeps 1 at 24 fps gives soft contacts.
- `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read; use `enablesleep`, `awake` and `enabled`.
- Points sharing a `name` are separate bodies, as with Bullet.
- Sleeping a body also sleeps the bodies touching it.
- Solve On Creation Frame works as with Bullet: off, a new object starts moving one frame after it appears.
- Show Collision Shapes (Guide Options on the solver) draws the bodies' collision shapes.
- Escape cancels a step.

## Troubleshooting

**"Box3D: world limit reached".** Each rebuild (for example simulating on from a frame that wasn't the last one stepped) makes a new world, and old ones stay alive while cached frames use them. Reset the simulation or clear the DOP cache, or build with `-DBOX3DDOP_MAX_WORLDS=<n>`.

**Debug logging:** configure with `-DBOX3DDOP_DEBUG=ON` to print to Houdini's console.

## For developers

See [CLAUDE.md](CLAUDE.md).

## License

MIT, see [LICENSE](LICENSE).
