# Houdini Box3D

A Houdini DOP solver built on [Box3D](https://github.com/erincatto/box3d), one of the fastest rigid body engines available. The goal of this project is to keep as much of Box3D's performance as possible inside Houdini.

> [!CAUTION]
> This is heavily vibe coded, as I'm still learning the HDK! This isn't normally how I develop software.

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

- **Box3D Solver**: runs the simulation (gravity, substeps, threads, sleeping).
- **Box3D Data**: default material and shape settings for an object (mass, friction, bounce, damping, gravity scale, default shape, and so on). Its parameters have the same names as the point attributes below, and a point attribute overrides the parameter of the same name.

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
| `enablesleep` | int | `0` = this body never sleeps |
| `isbullet` | int | Nonzero = continuous collision for fast bodies |
| `active` | int | `0` = static, anything else = dynamic |
| `bullet_georep` | string | Collision shape (see below) |
| `pscale` / `scale` | float / vector | Size of the shape |
| `id` | int | Stable point id; use it if points can be deleted or reordered |

Every step the solver writes `P`, `orient`, `v` and `w` back to the dynamic points.

### Shapes

Set the `bullet_georep` string attribute using the same names Houdini's RBD tools use (lowercase):

| Value | Result |
|---|---|
| `box`, `sphere`, `capsule` | A simple shape sized from `pscale` / `scale` (or from the bounds of packed geometry) |
| `convexhull` | One convex hull around the packed geometry |
| `concave`, `compound` | Several convex hulls on one body, one per named piece of the packed geometry |
| `none` | A static body with no collision |

Anything else, including `cylinder` and `plane`, uses the Default Shape from the Box3D Data node.

Convex and concave shapes need **packed primitives** (one point plus one packed primitive per body, for example from the Pack SOP or RBD Packed Objects). Pack a simplified proxy for best performance. Pieces are split by the primitive `name` attribute. If a shape can't be built (flat, too few points, empty), the solver falls back to a box and shows a warning on the node.

### Things to know

- Negative `friction`, `bounce` and damping values are clamped to 0 (Box3D does not accept them).
- Changes to shape, size, or any of the material and body attributes above (`mass`, `friction`, `bounce`, damping, `gravityscale`, `enablesleep`, `isbullet`) on a point that already exists are not picked up until the simulation is reset. Position, rotation, velocity and `active` changes are.
- Triangle-mesh colliders, joints/constraints and collision events are not supported yet.

## Troubleshooting

Debug logging is on by default and prints to Houdini's console window. Configure with `-DBOX3DDOP_DEBUG=OFF` for a quiet build.

## For developers

See [AGENTS.md](AGENTS.md) for a detailed technical writeup of the codebase.

## License

MIT, see [LICENSE](LICENSE).
