# Houdini Box3D

Houdini DOP wrapper for [Box3D](https://github.com/erincatto/box3d), one of the fastest rigid body solvers available.

My goal with this project is to keep Box3D's great performance preserved inside Houdini.

I reused design decisions where possible from the RBD Bullet Solver, so it should feel familiar to use.

> [!CAUTION]
> This is heavily vibe coded, as it was made for fun! This isn't normally how I develop software.
> 
> I've made many design and optimization decisions based on my own knowledge, but I'm still learning the HDK.

## Building

You need CMake, a C++17 compiler, git, and a Houdini install (for the HDK). Clone with submodules so Box3D is included:

```
git clone --recurse-submodules https://github.com/MysteryPancake/Houdini-Box3D
```

### Windows

Open **Houdini X.Y.Z > Command Line Tools** from the Start menu (this sets `%HFS%`), then:

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Always build **Release** or **RelWithDebInfo**, not Debug. Use Release for production: RelWithDebInfo also compiles in Box3D's internal checks, which are slower; any that fail are counted and reported on the node rather than stopping Houdini (they are printed to Houdini's console only in debug-logging builds). The DLL is copied to `Documents\houdiniX.Y\dso` and the node icons to `Documents\houdiniX.Y\config\Icons`. Close Houdini before rebuilding.

`build.bat` does the same thing in one step and finds Houdini for you: it uses `%HFS%` if set, otherwise the install the Houdini installer recorded in the registry, otherwise the newest `Houdini X.Y.Z` folder under `Program Files\Side Effects Software`.

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

The plugin is copied to `~/houdiniX.Y/dso` and the node icons to `~/houdiniX.Y/config/Icons`. Close Houdini before rebuilding.

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

Like Houdini's RBD solvers, the solver works on **points**: every point in an object's Geometry is one rigid body, and the point is the body's **centre of mass**. For packed pieces, put the pivot at the centroid (the Pack SOP and RBD tools do this by default), or the piece will turn around an off-centre point. Wire the object into the solver as you would with any other DOP solver.

### Point attributes

All attributes are optional except `P`. If one is missing, the value from the Box3D Data node is used.

| Attribute | Type | Meaning |
|---|---|---|
| `P` | vector | Position (required) |
| `orient` | quaternion | Rotation |
| `v` | vector | Linear velocity |
| `w` | vector | Angular velocity (radians/s) |
| `mass` | float | Mass of the body (written by the solver when `computemass` is on) |
| `computemass` | int | `1` = mass is `density` × shape volume |
| `density` | float | Density used by `computemass` |
| `inertialtensorstiffness` | float | Scales how hard the body is to spin |
| `friction` | float | Surface friction |
| `bounce` | float | Bounciness |
| `lineardamping` | float | Linear damping (0 or more) |
| `angulardamping` | float | Angular damping (0 or more) |
| `gravityscale` | float | Multiplies gravity for this body; negative floats upward |
| `enablesleep` | int | `0` = this body never sleeps |
| `bullet_want_deactivate` | int | `1` = may sleep, `0` = kept awake (overrides `enablesleep`) |
| `sleepthreshold` | float | Speed below which the body may fall asleep (after half a second) |
| `bullet_sleeping` | int | Written by the solver: `1` while asleep. Set it to `0` to wake the body |
| `fastobject` | int | Nonzero = a fast body can't pass through other moving bodies (it never passes through static ones) |
| `contactrecycling` | int | `0` = this body's contacts are never recycled (see Contact Recycle Distance); default is the Box3D Data setting, on |
| `active` | int | `0` = static, anything else = dynamic |
| `animated` | int | With `active` = 0: the body follows its animated point and pushes what it touches |
| `min_activation_impulse` | float | An inactive body hit at least this hard becomes active |
| `activationignore` | string | Patterns of bodies whose hits don't activate this one |
| `collisiongroup` | string | A name that `collisionignore` and `activationignore` patterns can match |
| `collisionignore` | string | Patterns of bodies this one doesn't collide with (`*` = nothing) |
| `bullet_ignore` | int | Nonzero = the body is left out of the simulation |
| `deforming` | int | Nonzero = rebuild the collision shape from the current geometry every step |
| `bullet_autofit_valid` | int | `0` = rebuild the collision shape once (the solver sets it back to `1`) |
| `bullet_georep` | string | Collision shape (see below) |
| `bullet_autofit` | int | `1` = fit a box, sphere or capsule to the packed geometry's bounds, `0` = use the attributes below (default: the Box3D Data node's Auto Fit, on) |
| `bullet_primS` | vector | Box half extents (autofit off, or no packed geometry) |
| `bullet_radius` | float | Sphere / capsule radius |
| `bullet_length` | float | Capsule length along Y, straight section only (caps not included) |
| `bullet_primT` | vector | Shape offset from the body origin |
| `bullet_primR` | vector | Shape rotation, Euler degrees (X, then Y, then Z) |
| `pscale` / `scale` | float / vector | Size of the shape |
| `id` | int | Stable point id; use it if points can be deleted or reordered |
| `dead` | int | Nonzero = delete the point and its body |
| `age` / `life` | float | With Enable Aging: `age` counts up and the point is marked `dead` once it passes `life` |

Every step the solver writes `P`, `orient`, `v` and `w` back to the dynamic points.

`collisionignore` and `activationignore` patterns match another body's `collisiongroup`, its DOP object's name, or its object id. Separate patterns with spaces, use `*` and `?` as wildcards, and start a pattern with `^` to exclude.

Turn on **Add Impact Data** on the Box3D Data node to record the object's impacts in its **Impacts** data, which shows in the details view on steps that have at least one impact. This can increase simulation time and memory use.

The **Box3D Solver** also has **Time Scale** (slow or fast motion), **Enable Aging**, and Box3D's own tuning: **Contact Stiffness** and **Contact Damping Ratio** (how firmly contacts resolve overlap), **Overlap Push Speed** (lower it if pieces that start out overlapping fly apart), **Bounce Threshold**, **Contact Recycle Distance**, **Max Linear Speed**, **Continuous Collision**, **Speculative Contacts**, **Bounce Iterations** and **Bounce Propagation**, **Isotropic Inertia Tolerance** (stops near-spherical bodies spinning oddly), **Warm Starting** (leave on), **Impact Min Speed** (hides slow impacts from Impact Data) and **Friction Mixing** / **Bounce Mixing** (how two materials combine). All solver parameters except the thread count can be animated.

### Forces

Force DOPs connected to an object (Gravity, Wind, Fan, POP Force, Orbit, custom) are applied, as are per-point `force` and `torque` attributes.

- **Use Force DOPs** turns this off.
- **Gravity adds to a Gravity Force.** Set the solver's **Gravity** to 0 when you use one, or bodies fall twice as fast.
- **Forces Wake Bodies** (off by default) lets forces wake sleeping bodies. Turn it on for wind or fans.
- Strong drag or wind can make bodies jitter or explode; lower the strength or raise Substeps.

### Contact recycling

Reuses contacts between barely-moving pairs to save time. Very occasionally this causes ghost collisions on thin or fast bodies.

- **Contact Recycle Distance** (solver, 0.05): set to 0 to turn recycling off, raise it for more speed.
- **Contact Recycling** (Box3D Data) or the `contactrecycling` attribute turns it off per body.

### Changing attributes during the simulation

Like Houdini's Bullet solver, most attributes can be changed mid-simulation (with a Geometry Wrangle DOP, a SOP Solver, or by animating the Box3D Data node) and take effect on the next step. That covers position, rotation, velocity, `active`, `animated`, the mass, friction, bounce, damping, gravity, sleep and collision attributes above, `bullet_ignore` and `dead`.

Collision shapes are kept once built. To rebuild one, set `bullet_autofit_valid = 0` (rebuilds once) or `deforming = 1` (rebuilds every step). This is how changes to `bullet_georep`, `bullet_autofit`, the `bullet_prim*` attributes, `bullet_radius`, `bullet_length` and the size of a box, sphere or capsule take effect. Convex and concave shapes follow scale changes on their own.

`enablesleep` only applies when a body is created; use `bullet_want_deactivate` to change sleeping later. `id` has no special values: changing it turns the point into a new body.

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

### Static Objects and collision relationships

A **Static Object** DOP collides with solved objects when a collision relationship links them, as with the Bullet solver. Only its polygons are used, not its SDF. Height fields aren't supported.

- Meshes are one-sided. If bodies fall through, turn on **Flip Static Mesh Winding** on a Box3D Data node attached to the affector. Its Friction and Bounce set the surface material.
- An animated Static Object doesn't push bodies. For something that pushes, use a kinematic point (`active = 0`, `animated = 1`).

### Things to know

- Bullet's collision padding attributes and solver settings (ERP, CFM, iterations) are ignored. Use Substeps and the contact settings.
- `concave` / `compound` are built as several convex hulls.
- Auto-fitted sizes aren't written back to `bullet_prim*`, `bullet_radius` or `bullet_length`.
- Use `id` if you delete points with `dead`, otherwise every later point counts as a new body.
- Patterns don't match DOP groups yet.
- Joints and constraints aren't supported yet.
- Escape cancels a step.

## Troubleshooting

**"Box3D: world limit reached".** Each time the solver has to rebuild (for example when you simulate on from a frame that wasn't the last one stepped), it makes a new Box3D world, and the old one stays alive while cached frames still use it. Resetting the simulation or clearing the DOP cache frees them. The limit is 1024 by default; build with `-DBOX3DDOP_MAX_WORLDS=<n>` to change it.

Debug logging is off by default. Configure with `-DBOX3DDOP_DEBUG=ON` to compile it in; it prints to Houdini's console window. CMake remembers the setting in an existing `build` folder, so delete that folder (or pass the option explicitly) when switching.

## For developers

See [AGENTS.md](AGENTS.md) for a detailed technical writeup of the codebase.

## License

MIT, see [LICENSE](LICENSE).
