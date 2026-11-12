# Houdini Box3D

A Houdini DOP solver built on [Box3D](https://github.com/erincatto/box3d), one of the fastest rigid body engines available.

The goal of this project is to keep Box3D's great performance preserved inside Houdini.

> [!CAUTION]
> This is heavily vibe coded, as it was made for fun! This isn't normally how I develop software.
> 
> I've made many design and optimization decisions based on my own knowledge, but I'm still learning the HDK.

## Building

You need CMake, a C++17 compiler, git, and a Houdini install (for the HDK). Clone with submodules so Box3D is included:

```
git clone --recurse-submodules <this repo>
```

### Windows

Open **Houdini X.Y.Z > Command Line Tools** from the Start menu (this sets `%HFS%`), then:

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Always build **Release** or **RelWithDebInfo**, not Debug. The DLL is copied to `Documents\houdiniX.Y\dso`. Close Houdini before rebuilding.

`build.bat` does the same thing in one step, but you need to edit the Houdini path at the top of the file first.

### Linux

Install the build tools (Debian/Ubuntu shown):

```
sudo apt-get install g++ cmake git mesa-common-dev libglu1-mesa-dev libxi-dev
```

Then set up Houdini's environment and run the build script:

```
source /opt/hfsX.Y.ZZZ/houdini_setup
./build.sh
```

The plugin is copied to `~/houdiniX.Y/dso`. Close Houdini before rebuilding.

### Checking Box3D without Houdini

To confirm Box3D itself builds and runs on your machine, you can build just the core and its tests:

```
cmake -S . -B build_core -DCMAKE_BUILD_TYPE=Release -DBOX3DDOP_CORE_ONLY=ON
cmake --build build_core --config Release
```

Run `b3dop_core_smoke` from the build folder. It should print the Box3D version and `OK`.

## Using it

In a DOP network, press **TAB** and add:

- **Box3D Solver**: runs the simulation.
- **Box3D Data**: default material and shape settings for an object.

Like Houdini's RBD solvers, the solver works on **points**: every point in an object's Geometry is one rigid body. Wire the object into the solver as you would with any other DOP solver.

### Point attributes

All attributes are optional except `P`. If one is missing, the value from the Box3D Data node is used.

| Attribute | Type | Meaning |
|---|---|---|
| `P` | vector | Position (required) |
| `orient` | quaternion | Rotation |
| `v` | vector | Linear velocity |
| `w` | vector | Angular velocity (radians/s) |
| `mass` | float | Mass of the body (the solver derives density from it and the shape volume) |
| `friction` | float | Surface friction |
| `bounce` | float | Bounciness (Box3D restitution) |
| `lineardamping` | float | Linear damping (0 or more) |
| `angulardamping` | float | Angular damping (0 or more) |
| `gravityscale` | float | Multiplies gravity for this body; negative floats upward |
| `enablesleep` | int | `0` = this body never sleeps (read when the body is created) |
| `bullet_want_deactivate` | int | `1` = may sleep, `0` = kept awake; overrides `enablesleep` and can change mid-simulation |
| `isbullet` | int | Nonzero = continuous collision for fast bodies |
| `active` | int | `0` = static, anything else = dynamic |
| `bullet_ignore` | int | Nonzero = the body takes no part in the simulation (no motion, no collisions) |
| `deforming` | int | Nonzero = rebuild the collision shape from the current geometry every step |
| `bullet_autofit_valid` | int | `0` = rebuild the collision shape once; the solver sets it back to `1` |
| `bullet_georep` | string | Collision shape (see below) |
| `bullet_autofit` | int | `1` = fit a box, sphere or capsule to the packed geometry's bounds, `0` = use the attributes below (default: the Box3D Data node's Auto Fit, on) |
| `bullet_primS` | vector | Box half extents (autofit off, or no packed geometry) |
| `bullet_radius` | float | Sphere / capsule radius |
| `bullet_length` | float | Capsule length along Y, straight section only (caps not included) |
| `bullet_primT` | vector | Shape offset from the body origin |
| `bullet_primR` | vector | Shape rotation, Euler degrees (X, then Y, then Z) |
| `pscale` / `scale` | float / vector | Size of the shape |
| `id` | int | Stable point id; use it if points can be deleted or reordered |

Every step the solver writes `P`, `orient`, `v` and `w` back to the dynamic points.

### Changing attributes during the simulation

The solver follows the same rules as Houdini's Bullet solver. Change attributes with a Geometry Wrangle DOP, a SOP Solver, or by animating the Box3D Data node.

| Attribute | Changes mid-simulation? | How |
|---|---|---|
| `P`, `orient`, `v`, `w`, `active` | Yes | Takes effect on the next step |
| `mass` | Yes | Takes effect on the next step. The inertia is scaled to match, so the shape's mass distribution stays the same |
| `friction`, `bounce` | Yes | Takes effect on the next step, including on contacts that already exist |
| `lineardamping`, `angulardamping` | Yes | Takes effect on the next step |
| `gravityscale` | Yes | Takes effect on the next step |
| `bullet_want_deactivate` | Yes | `0` wakes the body and keeps it awake, `1` lets it sleep again |
| `bullet_ignore` | Yes | Takes the body out of the simulation or puts it back |
| packed scale (`convexhull`, `concave`, `compound`) | Yes | The hulls are rebuilt at the new scale automatically. The mass stays the same |
| `pscale` / `scale`, and packed scale for `box`, `sphere`, `capsule` | Only with a rebuild | Set `bullet_autofit_valid = 0` |
| `bullet_autofit`, `bullet_prim*`, `bullet_radius`, `bullet_length` | Only with a rebuild | Set `bullet_autofit_valid = 0` |
| `bullet_georep` | Only with a rebuild | Set `bullet_autofit_valid = 0` (once) or `deforming = 1` (every step) |
| packed geometry itself | Only with a rebuild | `deforming = 1` rebuilds from the current geometry every step |
| `enablesleep`, `isbullet` | No | Read only when the body is created. Use `bullet_want_deactivate` to control sleep |
| `id` | Not a rebuild switch | `id` only identifies the body. No value (including `-1`) has a special meaning. Giving a point a different `id` makes it a different body |

To force a collision shape to be rebuilt, set `bullet_autofit_valid = 0` for a one-off rebuild or `deforming = 1` to rebuild every step. A rebuild reads every shape attribute again, and the body keeps its position, velocity and identity. If the rebuilt shape is identical, nothing is changed.

### Shapes

Set the `bullet_georep` string attribute using the same names Houdini's RBD tools use (lowercase):

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule` | A simple shape sized from `pscale` / `scale` (or from the bounds of packed geometry) |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Several convex hulls on one body, one per named piece of the packed geometry |
| `none` | A static body with no collision |

Anything else, including `cylinder` and `plane`, uses the Default Shape from the Box3D Data node.

#### Setting a box, sphere or capsule by hand

By default a box, sphere or capsule on a packed point is fitted to the packed geometry's bounds (Auto Fit on). Turn Auto Fit off on the Box3D Data node, or set `bullet_autofit = 0` on a point, to build the shape from attributes instead:

- **Box**: `bullet_primS` (half extents)
- **Sphere**: `bullet_radius`
- **Capsule**: `bullet_radius` and `bullet_length` (it stands along Y)
- **Any of them**: `bullet_primT` moves the shape off the body origin, and `bullet_primR` turns it (Euler degrees, XYZ). A sphere ignores the rotation.

Every attribute is optional. Anything you leave out falls back to the `pscale` / `scale` size. Plain points have no geometry to fit, so these attributes apply to them whatever Auto Fit says. They are ignored for `convexhull`, `concave`, `compound` and `none`.

Convex and concave shapes need **packed primitives** (one point plus one packed primitive per body, for example from the Pack SOP or RBD Packed Objects). Pack a simplified proxy for best performance. Pieces are split by the primitive `name` attribute. If a shape can't be built (flat, too few points, empty), the solver falls back to a box and shows a warning on the node.

### Things to know

- Negative `friction`, `bounce` and damping values are clamped to 0 (Box3D does not accept them).
- See "Changing attributes during the simulation" above for what an existing body picks up. Resetting the simulation always rebuilds everything from the current attributes.
- When the solver auto-fits a shape it does not write the fitted values back into `bullet_primS` / `bullet_primT` / `bullet_primR` / `bullet_radius` / `bullet_length`, unlike Bullet. It only writes `bullet_autofit_valid = 1`.
- Triangle-mesh colliders, joints/constraints and collision events are not supported yet.

## Troubleshooting

Debug logging is on by default and prints to Houdini's console window. Configure with `-DBOX3DDOP_DEBUG=OFF` for a quiet build.

## For developers

See [AGENTS.md](AGENTS.md) for a detailed technical writeup of the codebase.

## License

MIT, see [LICENSE](LICENSE).
